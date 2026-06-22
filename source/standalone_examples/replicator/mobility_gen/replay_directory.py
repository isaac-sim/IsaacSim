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

r"""CLI wrapper for replaying MobilityGen recordings and rendering sensor data.

Example usage (from the Isaac Sim build directory):

    cd _build/linux-x86_64/release
    ./python.sh ../../../source/standalone_examples/replicator/mobility_gen/replay_directory.py \\
        --render_interval 6 \\
        --enable isaacsim.replicator.mobility_gen.examples \\
        --input ~/MobilityGenData/recordings \\
        --output ~/MobilityGenData/replays
"""

from isaacsim import SimulationApp

# multi_gpu disabled: segfaults on Kit 110.1.x (unsolved).
simulation_app = SimulationApp(launch_config={"headless": True, "multi_gpu": False})

import argparse
import glob
import os
import shutil
import time

import carb
import isaacsim.core.experimental.utils.app as app_utils
import omni.replicator.core as rep
import omni.timeline
from isaacsim.core.experimental.utils.stage import get_current_stage
from isaacsim.core.simulation_manager import SimulationManager

app_utils.enable_extension("isaacsim.replicator.experimental.mobility_gen")
app_utils.enable_extension("isaacsim.replicator.mobility_gen.examples")

simulation_app.update()

from isaacsim.replicator.experimental.mobility_gen import (
    COMPLETE_MARKER_NAME,
    REPLAY_CONFIG_NAME,
    MobilityGenReader,
    MobilityGenWriter,
    apply_sensor_overrides,
    is_complete,
    load_scenario,
    log_camera_properties,
    mark_replay_complete,
    replay_config_from_args,
    setup_for_replay,
    write_replay_config,
)

if "MOBILITY_GEN_DATA" in os.environ:
    DATA_DIR = os.environ["MOBILITY_GEN_DATA"]
else:
    DATA_DIR = os.path.expanduser("~/MobilityGenData")


# Rendered-sensor state subdirs a replay (re)generates. state/common (recorded
# poses) is excluded: it is the replay's input, not a regenerated output.
_RENDERED_STATE_DIRS = ("rgb", "segmentation", "depth", "normals")


def clear_replay_outputs(output_path: str) -> None:
    """Remove the files a replay regenerates, leaving any source data in place.

    Lets --output equal --input: the recorded poses, scene, and config survive
    while the rendered sensor outputs and manifest are refreshed.
    """
    targets = [os.path.join(output_path, "state", name) for name in _RENDERED_STATE_DIRS]
    targets.append(os.path.join(output_path, REPLAY_CONFIG_NAME))
    targets.append(os.path.join(output_path, COMPLETE_MARKER_NAME))
    for target in targets:
        if os.path.isdir(target):
            shutil.rmtree(target)
        elif os.path.isfile(target):
            os.remove(target)


if __name__ == "__main__":

    parser = argparse.ArgumentParser()

    parser.add_argument(
        "--input", type=str, default=os.path.join(DATA_DIR, "recordings"), help="The path to the input recordings."
    )

    parser.add_argument(
        "--output",
        type=str,
        default=os.path.join(DATA_DIR, "replays"),
        help="The path to output the recordings with rendered sensor data",
    )

    parser.add_argument(
        "--rgb_enabled",
        action=argparse.BooleanOptionalAction,
        default=True,
        help="Enable RGB image rendering (--rgb_enabled / --no-rgb_enabled).",
    )

    parser.add_argument(
        "--segmentation_enabled",
        action=argparse.BooleanOptionalAction,
        default=True,
        help="Enable semantic segmentation rendering (--segmentation_enabled / --no-segmentation_enabled).",
    )

    parser.add_argument(
        "--depth_enabled",
        action=argparse.BooleanOptionalAction,
        default=True,
        help="Enable depth image rendering (--depth_enabled / --no-depth_enabled).",
    )

    parser.add_argument(
        "--instance_id_segmentation_enabled",
        action=argparse.BooleanOptionalAction,
        default=False,
        help="Enable instance segmentation rendering (--instance_id_segmentation_enabled / --no-instance_id_segmentation_enabled).",
    )

    parser.add_argument(
        "--normals_enabled",
        action=argparse.BooleanOptionalAction,
        default=False,
        help="Enable surface normal image rendering (--normals_enabled / --no-normals_enabled).",
    )

    parser.add_argument(
        "--self_contained",
        action=argparse.BooleanOptionalAction,
        default=False,
        help="Copy the full scene into each replay output so it stands alone "
        "(--self_contained / --no-self_contained). When disabled (default), the output holds only "
        "the rendered state and a replay_config.yaml that links back to the source recording.",
    )

    parser.add_argument(
        "--render_rt_subframes",
        type=int,
        default=1,
        help="The number of subframes for RT rendering.  Increase this number to improve rendering quality at the cost of speed.",
    )

    parser.add_argument(
        "--render_interval",
        type=int,
        default=1,
        help="The number of physics steps per rendering.  For example, setting this value to 2 will render only once "
        "every 2 physics timesteps.  This may speed up the replay rendering and result in smaller datasets, but with "
        "some timesteps missing images.",
    )

    parser.add_argument(
        "--max_frames",
        type=int,
        default=None,
        help="If set, stop after rendering this many frames per recording (for quick test runs); 0 or unset renders all frames.",
    )

    parser.add_argument(
        "--warmup_frames",
        type=int,
        default=4,
        help="Render and discard this many frames before starting the replay, to warm the RTX "
        "temporal accumulator (the first frames are otherwise cold/noisy). 0 disables.",
    )

    parser.add_argument(
        "--skip_completed",
        action=argparse.BooleanOptionalAction,
        default=False,
        help="Skip recordings whose output already finished (has a .complete marker), so an "
        "interrupted batch can be resumed without re-rendering completed recordings. The stored "
        "replay_config.yaml must match the requested settings.",
    )

    cli_args, unknown = parser.parse_known_args()

    cli_args.input = os.path.expanduser(cli_args.input)
    cli_args.output = os.path.expanduser(cli_args.output)

    if cli_args.render_interval < 1:
        parser.error("--render_interval must be >= 1")
    if cli_args.render_rt_subframes < 1:
        parser.error("--render_rt_subframes must be >= 1")
    if cli_args.warmup_frames < 0:
        parser.error("--warmup_frames must be >= 0")
    if cli_args.max_frames is not None and cli_args.max_frames <= 0:
        cli_args.max_frames = None

    # Recording dirs (those with a config.json) under --input, in stable order; skip --output.
    output_abs = os.path.abspath(cli_args.output)
    recording_paths = sorted(
        p
        for p in glob.glob(os.path.join(cli_args.input, "*"))
        if os.path.isdir(p) and os.path.isfile(os.path.join(p, "config.json")) and os.path.abspath(p) != output_abs
    )

    for i, recording_path in enumerate(recording_paths, start=1):
        # Per-iteration copy of the CLI namespace: ensure_nurec_replay_flags
        # mutates this in place when a NuRec stage is detected. Without a fresh
        # copy each iteration, a NuRec recording would silently disable non-RGB
        # modalities for every subsequent non-NuRec recording in the batch.
        args = argparse.Namespace(**vars(cli_args))

        name = os.path.basename(recording_path)

        output_path = os.path.join(args.output, name)

        scenario = load_scenario(recording_path)

        # Set up the loaded stage for replay: NuRec render overrides + RGB-only replay flags
        # (no-op on non-NuRec stages).
        setup_for_replay(args, get_current_stage())

        replay_config = replay_config_from_args(recording_path, args)
        if args.skip_completed and is_complete(
            output_path,
            replay_config,
            replay_label=f"{i} / {len(recording_paths)}: {name}",
            log_warn=carb.log_warn,
        ):
            continue

        SimulationManager.initialize_physics()

        if args.rgb_enabled:
            scenario.enable_rgb_rendering()

        if args.segmentation_enabled:
            scenario.enable_segmentation_rendering()

        if args.depth_enabled:
            scenario.enable_depth_rendering()

        if args.instance_id_segmentation_enabled:
            scenario.enable_instance_id_segmentation_rendering()

        if args.normals_enabled:
            scenario.enable_normals_rendering()

        # Re-enable hydra texture updates now that all annotators are attached.
        scenario.finalize_rendering()

        # captureOnPlay=1 (set in the mobility_gen extension.toml) gates capture on a
        # playing timeline, which initialize_physics() does not start; without play()
        # the annotators capture nothing. This also inits the render graph before
        # apply_sensor_overrides() below. Do NOT replace this with a warmup
        # orchestrator.step(): a pre-loop orchestrator step leaves every subsequent
        # capture empty (the "tile cannot extend outside image" PNG crash).
        omni.timeline.get_timeline_interface().play()
        simulation_app.update()

        # Apply overrides only after the render graph is initialised (above); doing it
        # earlier races a USD change notice with SDGPipeline construction, crashing OmniGraph.
        apply_sensor_overrides("/World/robot", recording_path)
        log_camera_properties(get_current_stage(), "/World/robot")

        reader = MobilityGenReader(recording_path)
        num_steps = len(reader)

        clear_replay_outputs(output_path)

        writer = MobilityGenWriter(output_path)
        if args.self_contained:
            writer.copy_init(recording_path)
        write_replay_config(output_path, replay_config)

        carb.log_warn(f"============== Replaying {i} / {len(recording_paths)} ==============")
        carb.log_warn(f"\tInput path: {recording_path}")
        carb.log_warn(f"\tOutput path: {output_path}")
        carb.log_warn(f"\tRgb enabled: {args.rgb_enabled}")
        carb.log_warn(f"\tSegmentation enabled: {args.segmentation_enabled}")
        carb.log_warn(f"\tRendering RT subframes: {args.render_rt_subframes}")
        carb.log_warn(f"\tRender interval: {args.render_interval}")
        carb.log_warn(f"\tWarmup frames: {args.warmup_frames}")
        carb.log_warn(f"\tMax frames: {args.max_frames}")
        carb.log_warn(f"\tSelf contained: {args.self_contained}")

        # Warm the RTX temporal accumulator before capturing: render and discard
        # `warmup_frames` frames at the start pose. Simulation time advances during these
        # frames but the robot is re-asserted to the start pose each frame (held stationary),
        # and the frames are not written. The recorded state has no timestamp, so this
        # affects only render history, not the captured data.
        if args.warmup_frames > 0 and num_steps > 0:
            warmup_state = reader.read_state_dict(index=0)
            for _ in range(args.warmup_frames):
                scenario.load_state_dict(warmup_state)
                scenario.write_replay_data()
                SimulationManager.step(steps=1)
                simulation_app.update()
                rep.orchestrator.step(rt_subframes=args.render_rt_subframes, delta_time=0.0, pause_timeline=False)

        t0 = time.perf_counter()
        count = 0
        for step in range(0, num_steps, args.render_interval):
            if args.max_frames is not None and count >= args.max_frames:
                break

            carb.log_warn(f"{step} / {num_steps}")
            state_dict_original = reader.read_state_dict(index=step)

            scenario.load_state_dict(state_dict_original)
            scenario.write_replay_data()

            # Propagate tensor-API pose/joint writes to USD before rendering.
            # set_world_poses() / set_dof_positions() write into PhysX tensor buffers; PhysX
            # only syncs these back to USD during simulate() + fetch_results().
            # SimulationManager.initialize_physics() does not start the Kit timeline, so
            # simulation_app.update() does not tick physics here — a direct step() call is needed.
            SimulationManager.step(steps=1)

            simulation_app.update()

            rep.orchestrator.step(rt_subframes=args.render_rt_subframes, delta_time=0.00, pause_timeline=False)

            scenario.update_state()

            state_dict = scenario.state_dict_common()

            for k, v in state_dict_original.items():
                # overwrite with original state, to ensure physics based values are correct
                if v is not None:
                    state_dict[k] = v  # don't overwrite "None" values

            state_rgb = scenario.state_dict_rgb()
            state_segmentation = scenario.state_dict_segmentation()
            state_depth = scenario.state_dict_depth()
            state_normals = scenario.state_dict_normals()

            writer.write_state_dict_common(state_dict, step)
            writer.write_state_dict_rgb(state_rgb, step)
            writer.write_state_dict_segmentation(state_segmentation, step)
            writer.write_state_dict_depth(state_depth, step)
            writer.write_state_dict_normals(state_normals, step)

            count += 1
        t1 = time.perf_counter()

        if count:
            carb.log_warn(f"Process time per frame: {(t1 - t0) / count:.4f} s")

        rep.orchestrator.wait_until_complete()
        # Stop the timeline before disable_rendering()/the next load_scenario();
        # leaving it playing across teardown crashes Kit natively.
        omni.timeline.get_timeline_interface().stop()
        scenario.disable_rendering()
        writer.close()
        mark_replay_complete(output_path, count)

    # Stop the timeline so Kit's shutdown sequence receives the stop event and
    # can clean up physics properly.  Without this, the timeline is left paused
    # and simulation_app.close() hangs for 120 s waiting for physics teardown.
    omni.timeline.get_timeline_interface().stop()
    simulation_app.close()
