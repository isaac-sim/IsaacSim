# SPDX-FileCopyrightText: Copyright (c) 2023-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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

"""Generate synthetic video clips in a warehouse using the CosmosWriter."""

from isaacsim import SimulationApp

simulation_app = SimulationApp(launch_config={"headless": False})

import os

import carb
import omni.replicator.core as rep
import omni.timeline
import omni.usd
from isaacsim.core.utils.stage import add_reference_to_stage
from isaacsim.storage.native import get_assets_root_path
from pxr import UsdGeom

# Capture parameters
START_DELAY = 0.1  # Timeline duration delay before capturing the first clip
NUM_CLIPS = 2  # Number of video clips to capture with the CosmosWriter
NUM_FRAMES_PER_CLIP = 10  # Number of frames for each clip
CAPTURE_INTERVAL = 2  # Capture interval between frames (capture every N simulation steps)

# Stage and asset paths
STAGE_URL = "/Isaac/Samples/Replicator/Stage/full_warehouse_worker_and_anim_cameras.usd"
CARTER_NAV_ASSET_URL = "/Isaac/Samples/Replicator/OmniGraph/nova_carter_nav_only.usd"
CARTER_NAV_PATH = "/NavWorld/CarterNav"
CARTER_NAV_TARGET_PATH = f"{CARTER_NAV_PATH}/targetXform"
CARTER_CAMERA_PATH = f"{CARTER_NAV_PATH}/chassis_link/sensors/front_hawk/left/camera_left"
CARTER_NAV_POSITION = (-6, 4, 0)
CARTER_NAV_TARGET_POSITION = (3, 3, 0)


def advance_timeline_by_duration(duration: float, max_updates: int = 1000):
    """Advance the simulation timeline by the specified duration in seconds."""
    timeline = omni.timeline.get_timeline_interface()
    current_time = timeline.get_current_time()
    target_time = current_time + duration

    if timeline.get_end_time() < target_time:
        timeline.set_end_time(1000000)

    if not timeline.is_playing():
        timeline.play()

    print(f"Advancing timeline from {current_time:.4f}s to {target_time:.4f}s")
    step_count = 0
    while current_time < target_time:
        if step_count >= max_updates:
            print(f"Max updates reached: {step_count}, finishing timeline advance.")
            break

        prev_time = current_time
        simulation_app.update()
        current_time = timeline.get_current_time()
        step_count += 1

        if step_count % 10 == 0:
            print(f"\tStep {step_count}, {current_time:.4f}s/{target_time:.4f}s")

        if current_time <= prev_time:
            print(f"Warning: Timeline did not advance at update {step_count} (time: {current_time:.4f}s).")
    print(f"Finished advancing timeline to {current_time:.4f}s (target {target_time:.4f}s) in {step_count} steps")


def run_sdg_pipeline(
    camera_path, num_clips, num_frames_per_clip, capture_interval, use_instance_id=True, segmentation_mapping=None
):
    """Run the synthetic data generation pipeline and capture video clips."""
    rp = rep.create.render_product(camera_path, (1280, 720))
    cosmos_writer = rep.WriterRegistry.get("CosmosWriter")
    backend = rep.backends.get("DiskBackend")
    out_dir = os.path.join(os.getcwd(), f"_out_cosmos_warehouse")
    print(f"output_directory: {out_dir}")
    backend.initialize(output_dir=out_dir)
    cosmos_writer.initialize(
        backend=backend, use_instance_id=use_instance_id, segmentation_mapping=segmentation_mapping
    )
    cosmos_writer.attach(rp)

    # Make sure the timeline is playing
    timeline = omni.timeline.get_timeline_interface()
    if not timeline.is_playing():
        timeline.play()

    print(
        f"Starting SDG pipeline. Capturing {num_clips} clips with {num_frames_per_clip} frames each, every {capture_interval} simulation step(s)."
    )

    for clip_index in range(num_clips):
        print(f"Starting clip {clip_index + 1}/{num_clips}")

        frames_captured_count = 0
        simulation_step_index = 0
        while frames_captured_count < num_frames_per_clip:
            print(f"Simulation step {simulation_step_index}")
            if simulation_step_index % capture_interval == 0:
                print(f"\t Capturing frame {frames_captured_count + 1}/{num_frames_per_clip} for clip {clip_index + 1}")
                rep.orchestrator.step(pause_timeline=False)
                frames_captured_count += 1
            else:
                simulation_app.update()
            simulation_step_index += 1

        print(f"Finished clip {clip_index + 1}/{num_clips}. Captured {frames_captured_count} frames")

        # Move to next clip if not the last clip
        if clip_index < num_clips - 1:
            print(f"Moving to next clip...")
            cosmos_writer.next_clip()

    print("Waiting to finish processing and writing the data")
    rep.orchestrator.wait_until_complete()
    print(f"Finished SDG pipeline. Captured {num_clips} clips with {num_frames_per_clip} frames each")
    cosmos_writer.detach()
    rp.destroy()
    timeline.pause()


def run_example(
    num_clips,
    num_frames_per_clip,
    capture_interval,
    start_delay=0.0,
    use_instance_id=True,
    segmentation_mapping=None,
):
    """Set up the warehouse scene and run the SDG pipeline."""
    assets_root_path = get_assets_root_path()
    stage_path = assets_root_path + STAGE_URL
    print(f"Opening stage: '{stage_path}'")
    omni.usd.get_context().open_stage(stage_path)
    stage = omni.usd.get_context().get_stage()

    # Enable script nodes
    carb.settings.get_settings().set_bool("/app/omni.graph.scriptnode/opt_in", True)

    # Disable capture on play on the new stage, data is captured manually using the step function
    rep.orchestrator.set_capture_on_play(False)

    # Set DLSS to Quality mode (2) for best SDG results , options: 0 (Performance), 1 (Balanced), 2 (Quality), 3 (Auto)
    carb.settings.get_settings().set("rtx/post/dlss/execMode", 2)

    # Load carter nova asset with its navigation graph
    carter_url_path = assets_root_path + CARTER_NAV_ASSET_URL
    print(f"Loading carter nova asset: '{carter_url_path}' at prim path: '{CARTER_NAV_PATH}'")
    carter_nav_prim = add_reference_to_stage(usd_path=carter_url_path, prim_path=CARTER_NAV_PATH)

    if not carter_nav_prim.GetAttribute("xformOp:translate"):
        UsdGeom.Xformable(carter_nav_prim).AddTranslateOp()
    carter_nav_prim.GetAttribute("xformOp:translate").Set(CARTER_NAV_POSITION)

    # Set the navigation target position
    carter_navigation_target_prim = stage.GetPrimAtPath(CARTER_NAV_TARGET_PATH)
    if not carter_navigation_target_prim.IsValid():
        print(f"Carter navigation target prim not found at path: {CARTER_NAV_TARGET_PATH}, exiting")
        return
    if not carter_navigation_target_prim.GetAttribute("xformOp:translate"):
        UsdGeom.Xformable(carter_navigation_target_prim).AddTranslateOp()
    carter_navigation_target_prim.GetAttribute("xformOp:translate").Set(CARTER_NAV_TARGET_POSITION)

    # Use the carter nova front hawk camera for capturing data
    camera_prim = stage.GetPrimAtPath(CARTER_CAMERA_PATH)
    if not camera_prim.IsValid():
        print(f"Camera prim not found at path: {CARTER_CAMERA_PATH}, exiting")
        return

    # tickRate=0 forces autotrigger so the sensor cameras stay in sync with rep.orchestrator.step
    # under multi-tick rendering.
    if camera_prim.HasAttribute("omni:sensor:tickRate"):
        camera_prim.GetAttribute("omni:sensor:tickRate").Set(0.0)

    # Advance the timeline with the start delay if provided
    if start_delay is not None and start_delay > 0:
        advance_timeline_by_duration(start_delay)

    # Run the SDG pipeline
    run_sdg_pipeline(
        camera_prim.GetPath(), num_clips, num_frames_per_clip, capture_interval, use_instance_id, segmentation_mapping
    )


# Setup the environment and run the example
run_example(
    num_clips=NUM_CLIPS,
    num_frames_per_clip=NUM_FRAMES_PER_CLIP,
    capture_interval=CAPTURE_INTERVAL,
    start_delay=START_DELAY,
    use_instance_id=True,
)

simulation_app.close()
