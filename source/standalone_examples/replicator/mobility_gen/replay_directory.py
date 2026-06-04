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

"""CLI wrapper for replaying MobilityGen recordings and rendering sensor data.

Example usage (from the Isaac Sim build directory):

    cd _build/linux-x86_64/release
    ./python.sh ../../../source/standalone_examples/replicator/mobility_gen/replay_directory.py \\
        --render_interval 6 \\
        --enable isaacsim.replicator.mobility_gen.examples \\
        --input ~/MobilityGenData/recordings \\
        --output ~/MobilityGenData/replays

Note: multi_gpu is disabled to avoid crashes on Kit 110.1.x with multiple GPUs.
"""

from isaacsim import SimulationApp

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
    MobilityGenReader,
    MobilityGenWriter,
    apply_nurec_replay_overrides,
    apply_sensor_overrides,
    load_scenario,
    log_camera_properties,
)

if "MOBILITY_GEN_DATA" in os.environ:
    DATA_DIR = os.environ["MOBILITY_GEN_DATA"]
else:
    DATA_DIR = os.path.expanduser("~/MobilityGenData")

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

    cli_args, unknown = parser.parse_known_args()

    cli_args.input = os.path.expanduser(cli_args.input)
    cli_args.output = os.path.expanduser(cli_args.output)

    recording_paths = glob.glob(os.path.join(cli_args.input, "*"))

    for i, recording_path in enumerate(recording_paths, start=1):
        # Per-iteration copy of the CLI namespace: apply_nurec_replay_overrides
        # mutates this in place when a NuRec stage is detected. Without a fresh
        # copy each iteration, a NuRec recording would silently disable non-RGB
        # modalities for every subsequent non-NuRec recording in the batch.
        args = argparse.Namespace(**vars(cli_args))

        name = os.path.basename(recording_path)

        output_path = os.path.join(args.output, name)

        scenario = load_scenario(recording_path)

        # If the loaded stage is NuRec (contains ParticleField prims), force
        # the per-modality replay flags to the supported subset (RGB only)
        # and re-assert the gaussian-pass tonemap setting (which the stage's
        # customLayerData can revert on open). No-op on non-NuRec stages.
        apply_nurec_replay_overrides(args, get_current_stage())

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

        rep.orchestrator.step(rt_subframes=args.render_rt_subframes, delta_time=0.0, pause_timeline=False)

        # Apply camera calibration overrides after the render graph is fully
        # initialised — applying them earlier causes Kit to queue a USD change
        # notice that races with SDGPipeline construction and crashes OmniGraph.
        apply_sensor_overrides("/World/robot", recording_path)
        log_camera_properties(get_current_stage(), "/World/robot")

        reader = MobilityGenReader(recording_path)
        num_steps = len(reader)

        if os.path.exists(output_path):
            shutil.rmtree(output_path)

        writer = MobilityGenWriter(output_path)
        writer.copy_init(recording_path)

        carb.log_warn(f"============== Replaying {i} / {len(recording_paths)} ==============")
        carb.log_warn(f"\tInput path: {recording_path}")
        carb.log_warn(f"\tOutput path: {output_path}")
        carb.log_warn(f"\tRgb enabled: {args.rgb_enabled}")
        carb.log_warn(f"\tSegmentation enabled: {args.segmentation_enabled}")
        carb.log_warn(f"\tRendering RT subframes: {args.render_rt_subframes}")
        carb.log_warn(f"\tRender interval: {args.render_interval}")

        t0 = time.perf_counter()
        count = 0
        for step in range(0, num_steps, args.render_interval):

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

        carb.log_warn(f"Process time per frame: {count / (t1 - t0)}")

        rep.orchestrator.wait_until_complete()
        scenario.disable_rendering()
        writer.close()

    # Stop the timeline so Kit's shutdown sequence receives the stop event and
    # can clean up physics properly.  Without this, the timeline is left paused
    # and simulation_app.close() hangs for 120 s waiting for physics teardown.
    omni.timeline.get_timeline_interface().stop()
    simulation_app.close()
