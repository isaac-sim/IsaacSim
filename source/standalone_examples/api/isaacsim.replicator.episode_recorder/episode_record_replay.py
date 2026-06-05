# SPDX-FileCopyrightText: Copyright (c) 2025-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
# http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""End-to-end record/replay demo for :class:`EpisodeRecorder` + :class:`EpisodeReplayer`.

This script demonstrates the two-phase SDG recording workflow:

    1. **Live recording (cheap, state-only)** — Drive a Franka through a short scripted motion with
       the timeline playing. The recorder subscribes to ``SimulationManager`` events and writes one
       frame of articulation DOF state + cube pose per physics step.
    2. **Offline replay (augmentable)** — Reopen the HDF5 session, teleport the live stage to each
       recorded frame, and (optionally) hand off to ``omni.replicator.core`` to render annotated
       RGB-D frames. The replayer drives ``app.update()`` per frame so writers can attach and run at
       full quality without perturbing the original recording.

Usage::

    ./python.sh source/standalone_examples/api/isaacsim.replicator.episode_recorder/episode_record_replay.py \\
        --record_steps 180 \\
        --replay

Pass ``--render`` to additionally attach a :class:`BasicWriter` during replay to demonstrate the
"augment offline" flow (writes PNGs next to the HDF5 file).
"""

from isaacsim import SimulationApp

simulation_app = SimulationApp(launch_config={"headless": False})

import argparse
import os
import shutil
from pathlib import Path

import numpy as np
import omni.replicator.core as rep
import omni.timeline
import omni.usd
from isaacsim.core.experimental.prims import Articulation
from isaacsim.core.utils.extensions import enable_extension

enable_extension("isaacsim.replicator.episode_recorder")
from isaacsim.replicator.episode_recorder import (
    ArticulationRecordable,
    EpisodeRecorder,
    EpisodeReplayer,
    ReplayPolicy,
    RigidBodyRecordable,
    export_stage_snapshot,
    target_discovery,
)
from isaacsim.storage.native import get_assets_root_path
from pxr import Gf, UsdGeom, UsdPhysics

parser = argparse.ArgumentParser()
parser.add_argument(
    "--output_dir",
    default=os.path.join(os.getcwd(), "_out_episode_record_replay"),
    help="Where to write the HDF5 session + snapshot.",
)
parser.add_argument(
    "--record_steps",
    type=int,
    default=120,
    help="Number of simulation steps to record (deterministic across hardware; ~2s at 60Hz).",
)
parser.add_argument("--replay", action="store_true", help="After recording, load the session and replay it.")
parser.add_argument("--render", action="store_true", help="Attach a BasicWriter during replay and dump RGB frames.")
parser.add_argument("--existing_hdf5", default=None, help="Skip recording; replay this HDF5 file instead.")
parser.add_argument(
    "--test", action="store_true", help="Validate captured output files against expected counts and exit."
)
args, _ = parser.parse_known_args()


FRANKA_USD_REL_PATH = "Isaac/Robots/FrankaRobotics/FrankaPanda/franka.usd"


def build_scene() -> None:
    """Populate the default stage with a Franka articulation and a rigid cube."""
    stage = omni.usd.get_context().get_stage()
    stage.DefinePrim("/World", "Xform")
    stage.DefinePrim("/World/PhysicsScene", "PhysicsScene")

    usd_path = f"{get_assets_root_path()}/{FRANKA_USD_REL_PATH}"
    robot_prim = stage.DefinePrim("/World/Franka", "Xform")
    robot_prim.GetReferences().AddReference(usd_path)

    cube = UsdGeom.Cube.Define(stage, "/World/Cube")
    cube.AddTranslateOp().Set(Gf.Vec3d(0.5, 0.0, 0.1))
    UsdPhysics.RigidBodyAPI.Apply(cube.GetPrim())
    UsdPhysics.CollisionAPI.Apply(cube.GetPrim())


def drive_dofs(articulation: Articulation, num_steps: int, target_amplitude: float = 0.3) -> None:
    """Drive the Franka DOFs with a smooth sinusoid so the recording contains motion.

    Uses ``set_dof_position_targets`` on the underlying physics view so the PD drives honor the
    target, yielding realistic DOF trajectories the replayer can roundtrip. Driven by a fixed
    number of simulation steps (rather than wall-clock time) so the recording is deterministic
    across hardware.
    """
    n_dofs = int(articulation.num_dofs)
    target_freq_hz = 0.5
    dt = 1.0 / 60.0
    for i in range(num_steps):
        t = i * dt
        pattern = target_amplitude * np.sin(2.0 * np.pi * target_freq_hz * t + np.arange(n_dofs))
        articulation.set_dof_position_targets(pattern[np.newaxis, :].astype(np.float32))
        simulation_app.update()


def record_one_episode(output_dir: str, record_steps: int) -> str:
    """Record one episode of a short scripted motion; return the HDF5 session path."""
    build_scene()

    # Pump updates until the Franka USD reference resolves and exposes its ArticulationRootAPI.
    max_load_updates = 200
    articulations: dict = {}
    prims: dict = {}
    for _ in range(max_load_updates):
        simulation_app.update()
        articulations, prims = target_discovery.discover_all_under("/World")
        if articulations:
            break
    print(f"[Record] Discovered articulations={articulations}, prims={prims}")
    if not articulations:
        raise RuntimeError("No articulation roots discovered under /World; cannot drive DOFs.")
    # Use the discovered path with ArticulationRootAPI; /World/Franka is only the reference Xform.
    franka_path = next(iter(articulations.values()))

    os.makedirs(output_dir, exist_ok=True)
    snapshot_path = export_stage_snapshot(output_dir)
    print(f"[Record] Scene snapshot: {snapshot_path}")

    recorder = EpisodeRecorder(
        output_dir=output_dir,
        session_metadata={"task_description": "franka_demo", "script": "episode_record_replay.py"},
    )
    for name, path in articulations.items():
        recorder.add(ArticulationRecordable(group=f"state/{name}", prim_path=path))
    for name, path in prims.items():
        recorder.add(RigidBodyRecordable(group=f"state/{name}", prim_path=path))

    hdf5_path = recorder.open_session()
    print(f"[Record] Session opened: {hdf5_path}")

    timeline = omni.timeline.get_timeline_interface()
    timeline.play()
    for _ in range(5):
        simulation_app.update()

    recorder.start_episode()
    articulation = Articulation(franka_path)
    drive_dofs(articulation, num_steps=record_steps)
    recorder.end_episode(success=True)

    timeline.stop()
    for _ in range(2):
        simulation_app.update()
    recorder.close_session()
    print(f"[Record] Done. Wrote {hdf5_path}")
    return hdf5_path


def replay_episode(hdf5_path: str, *, attach_writer: bool) -> None:
    """Reopen the session, play the recorded trajectory, and optionally render it with a writer."""
    omni.usd.get_context().new_stage()
    build_scene()
    for _ in range(2):
        simulation_app.update()

    writer = None
    render_product = None
    if attach_writer:
        output_dir = os.path.dirname(hdf5_path)
        render_output = os.path.join(output_dir, "replay_frames")
        os.makedirs(render_output, exist_ok=True)

        camera_prim = rep.create.camera(position=(1.2, 1.2, 1.0), look_at=(0.0, 0.0, 0.3))
        render_product = rep.create.render_product(camera_prim, (512, 512))
        writer = rep.WriterRegistry.get("BasicWriter")
        writer.initialize(output_dir=render_output, rgb=True)
        writer.attach([render_product])
        print(f"[Replay] Attached BasicWriter → {render_output}")

    timeline = omni.timeline.get_timeline_interface()
    if timeline.is_playing():
        timeline.pause()
    for _ in range(5):
        simulation_app.update()

    def _post_frame(frame_idx: int) -> None:
        if writer is not None:
            rep.orchestrator.step(delta_time=0.0, pause_timeline=False)

    replayer = EpisodeReplayer(hdf5_path, policy=ReplayPolicy(strictness="best_effort"))
    try:
        episodes = replayer.list_episodes()
        print(f"[Replay] Episodes: {episodes} ({len(episodes)} total)")
        replayer.replay_episode(
            0,
            render_interval=1 if attach_writer else 4,
            post_frame_hook=_post_frame,
            app_update_per_frame=True,
        )
    finally:
        replayer.close()
        timeline.stop()
        for _ in range(2):
            simulation_app.update()

    if writer is not None:
        writer.detach()
    print("[Replay] Done.")


def clean_test_output(output_dir: str, *, include_render: bool) -> None:
    """Remove artifacts this example creates so test validation is repeatable."""
    out_path = Path(output_dir)
    if not out_path.exists():
        return
    for pattern in ("stage_snapshot.usd", "stage_snapshot.sidecar.json", "episode_*.hdf5"):
        for path in out_path.glob(pattern):
            if path.is_file():
                path.unlink()
    render_dir = out_path / "replay_frames"
    if include_render and render_dir.exists():
        shutil.rmtree(render_dir)


hdf5_path = args.existing_hdf5
if hdf5_path is None:
    if args.test:
        clean_test_output(args.output_dir, include_render=args.render)
    os.makedirs(args.output_dir, exist_ok=True)
    hdf5_path = record_one_episode(args.output_dir, args.record_steps)

if args.replay or args.render:
    replay_episode(hdf5_path, attach_writer=args.render)

# <start-episode-record-replay-test>
import sys

from isaacsim.core.utils.extensions import enable_extension

enable_extension("isaacsim.test.utils")
from isaacsim.test.utils.file_validation import validate_folder_contents

if args.test:
    # Recording produces:
    #   - 1 stage_snapshot.usd (from export_stage_snapshot)
    #   - 1 stage_snapshot.sidecar.json (sidecar, include_sidecar=True by default)
    #   - 1 episode_<timestamp>.hdf5 (from EpisodeRecorder.open_session)
    expected_counts = {"usd": 1, "json": 1, "hdf5": 1}
    out_dir = args.output_dir
    if not validate_folder_contents(
        path=out_dir,
        recursive=False,
        expected_counts=expected_counts,
        fail_on_empty_files=True,
    ):
        print(f"[SDG][Test][FAIL] Output validation failed for {out_dir}")
        sys.exit(1)
    print(f"[SDG][Test][PASS] Output validation succeeded for {out_dir}")

    # When --render is set, replay attaches a BasicWriter that writes rgb pngs into replay_frames/.
    if args.render:
        render_dir = os.path.join(args.output_dir, "replay_frames")
        if not validate_folder_contents(
            path=render_dir,
            recursive=True,
            expected_counts={"png": 1},
            fail_on_empty_files=True,
            exact_match=False,
        ):
            print(f"[SDG][Test][FAIL] Output validation failed for {render_dir}")
            sys.exit(1)
        print(f"[SDG][Test][PASS] Output validation succeeded for {render_dir}")
# <end-episode-record-replay-test>

simulation_app.close()
