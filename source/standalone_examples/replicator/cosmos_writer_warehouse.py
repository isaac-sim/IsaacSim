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

import argparse
import os
from typing import Any

# Capture parameters
START_DELAY = 0.1  # Timeline duration delay before capturing the first clip
NUM_CLIPS = 3  # Number of video clips to capture with the CosmosWriter
NUM_FRAMES_PER_CLIP = 30  # Number of frames for each clip
CAPTURE_INTERVAL = 2  # Capture interval between frames (capture every N simulation steps)

# Stage and asset paths
STAGE_URL = "/Isaac/Samples/Replicator/Stage/full_warehouse_worker_and_anim_cameras.usd"
CARTER_NAV_ASSET_URL = "/Isaac/Samples/Replicator/OmniGraph/nova_carter_nav_only.usd"
CARTER_NAV_PATH = "/NavWorld/CarterNav"
CARTER_NAV_TARGET_PATH = f"{CARTER_NAV_PATH}/targetXform"
CARTER_CAMERA_PATH = f"{CARTER_NAV_PATH}/chassis_link/sensors/front_hawk/left/camera_left"
CARTER_NAV_POSITION = (-6, 4, 0)
CARTER_NAV_TARGET_POSITION = (3, 3, 0)
OUT_DIR = "_out_cosmos_warehouse"


parser = argparse.ArgumentParser()
parser.add_argument("--num-clips", type=int, default=NUM_CLIPS, help="Number of video clips to capture.")
parser.add_argument(
    "--num-frames-per-clip", type=int, default=NUM_FRAMES_PER_CLIP, help="Number of frames captured per clip."
)
parser.add_argument(
    "--capture-interval",
    type=int,
    default=CAPTURE_INTERVAL,
    help="Capture interval between frames (capture every N simulation steps).",
)
parser.add_argument(
    "--start-delay",
    type=float,
    default=START_DELAY,
    help="Timeline duration delay before capturing the first clip.",
)
args, _ = parser.parse_known_args()
if args.num_clips < 1 or args.num_frames_per_clip < 1 or args.capture_interval < 1:
    parser.error("--num-clips, --num-frames-per-clip, and --capture-interval must be > 0")

from isaacsim import SimulationApp

simulation_app = SimulationApp(launch_config={"headless": False})

import carb
import isaacsim.core.experimental.utils.app as app_utils
import isaacsim.core.experimental.utils.stage as stage_utils
import omni.replicator.core as rep
import omni.timeline
from isaacsim.core.experimental.prims import XformPrim
from isaacsim.core.experimental.utils.prim import get_prim_at_path
from isaacsim.core.experimental.utils.stage import add_reference_to_stage, open_stage
from isaacsim.storage.native import get_assets_root_path


def advance_timeline_by_duration(duration: float, max_updates: int = 1000) -> None:
    """Advance the simulation timeline toward the requested duration.

    The function stops early without raising if ``max_updates`` is reached before the target timeline time.

    Args:
        duration: Amount of simulation time to advance in seconds.
        max_updates: Maximum application updates to issue when the timeline stalls or advances slowly.
    """
    timeline = omni.timeline.get_timeline_interface()
    current_time = timeline.get_current_time()
    target_time = current_time + duration

    if timeline.get_end_time() < target_time:
        timeline.set_end_time(1000000)

    if not app_utils.is_playing():
        app_utils.play()

    print(f"Advancing timeline from {current_time:.4f}s to {target_time:.4f}s")
    step_count = 0
    while current_time < target_time:
        if step_count >= max_updates:
            print(f"Max updates reached: {step_count}, finishing timeline advance.")
            break

        prev_time = current_time
        app_utils.update_app()
        current_time = timeline.get_current_time()
        step_count += 1

        if step_count % 10 == 0:
            print(f"\tStep {step_count}, {current_time:.4f}s/{target_time:.4f}s")

        if current_time <= prev_time:
            print(f"Warning: Timeline did not advance at update {step_count} (time: {current_time:.4f}s).")
    print(f"Finished advancing timeline to {current_time:.4f}s (target {target_time:.4f}s) in {step_count} steps")


def run_sdg_pipeline(
    camera_path: str,
    num_clips: int,
    num_frames_per_clip: int,
    capture_interval: int,
    use_instance_id: bool = True,
    segmentation_mapping: dict[str, Any] | None = None,
) -> None:
    """Run the synthetic data generation pipeline and capture video clips.

    Args:
        camera_path: Path of the camera prim used by the render product.
        num_clips: Number of distinct video clips to write.
        num_frames_per_clip: Number of captured frames in each clip.
        capture_interval: Positive number of simulation updates between captured frames.
        use_instance_id: Whether the writer should encode segmentation by instance identifier.
        segmentation_mapping: Mapping from source semantic labels to output labels, or ``None`` to use the
            writer's default mapping.
    """
    rp = rep.create.render_product(camera_path, (640, 480))
    cosmos_writer = rep.WriterRegistry.get("CosmosWriter")
    backend = rep.backends.get("DiskBackend")
    out_dir = os.path.join(os.getcwd(), OUT_DIR)
    print(f"output_directory: {out_dir}")
    backend.initialize(output_dir=out_dir)
    cosmos_writer.initialize(
        backend=backend, use_instance_id=use_instance_id, segmentation_mapping=segmentation_mapping
    )
    cosmos_writer.attach(rp)

    # Make sure the timeline is playing
    if not app_utils.is_playing():
        app_utils.play()

    print(
        f"Starting SDG pipeline. Capturing {num_clips} clips with {num_frames_per_clip} frames each, "
        f"every {capture_interval} simulation step(s)."
    )

    for clip_index in range(num_clips):
        print(f"Starting clip {clip_index + 1}/{num_clips}")

        frames_captured_count = 0
        simulation_step_index = 0
        while frames_captured_count < num_frames_per_clip:
            print(f"Simulation step {simulation_step_index}")
            if simulation_step_index % capture_interval == 0:
                print(
                    f"\t Capturing frame {frames_captured_count + 1}/{num_frames_per_clip} "
                    f"for clip {clip_index + 1}"
                )
                rep.orchestrator.step(pause_timeline=False, rt_subframes=4)
                frames_captured_count += 1
            else:
                app_utils.update_app()
            simulation_step_index += 1

        print(f"Finished clip {clip_index + 1}/{num_clips}. Captured {frames_captured_count} frames")

        if clip_index < num_clips - 1:
            # Move to next clip if not the last clip
            print("Moving to next clip...")
            cosmos_writer.next_clip()

    print("Waiting to finish processing and writing the data")
    rep.orchestrator.wait_until_complete()
    print(f"Finished SDG pipeline. Captured {num_clips} clips with {num_frames_per_clip} frames each")
    app_utils.update_app(steps=3)
    cosmos_writer.detach()
    rp.destroy()
    app_utils.pause()


def run_example(
    num_clips: int,
    num_frames_per_clip: int,
    capture_interval: int,
    start_delay: float = 0.0,
    use_instance_id: bool = True,
    segmentation_mapping: dict[str, Any] | None = None,
) -> None:
    """Set up the warehouse scene and run the SDG pipeline.

    Args:
        num_clips: Number of distinct video clips to write.
        num_frames_per_clip: Number of captured frames in each clip.
        capture_interval: Positive number of simulation updates between captured frames.
        start_delay: Simulation time to advance before capturing the first frame, in seconds.
        use_instance_id: Whether the writer should encode segmentation by instance identifier.
        segmentation_mapping: Mapping from source semantic labels to output labels, or ``None`` to use the
            writer's default mapping.
    """
    assets_root_path = get_assets_root_path()
    if assets_root_path is None:
        print("Assets root path not found, exiting")
        return

    stage_path = assets_root_path + STAGE_URL
    print(f"Opening stage: '{stage_path}'")
    opened, stage = open_stage(stage_path)
    if not opened or stage is None:
        print(f"Could not open stage: '{stage_path}', exiting")
        return

    # Enable script nodes
    carb.settings.get_settings().set_bool("/app/omni.graph.scriptnode/opt_in", True)

    # Disable capture on play on the new stage, data is captured manually using the step function
    rep.orchestrator.set_capture_on_play(False)

    # Set DLSS to Quality mode (2) for best SDG results , options: 0 (Performance), 1 (Balanced), 2 (Quality), 3 (Auto)
    carb.settings.get_settings().set("rtx/post/dlss/execMode", 2)

    # Load carter nova asset with its navigation graph
    carter_url_path = assets_root_path + CARTER_NAV_ASSET_URL
    print(f"Loading carter nova asset: '{carter_url_path}' at prim path: '{CARTER_NAV_PATH}'")
    add_reference_to_stage(usd_path=carter_url_path, path=CARTER_NAV_PATH)
    XformPrim(CARTER_NAV_PATH, reset_xform_op_properties=True).set_local_poses(
        translations=[CARTER_NAV_POSITION], orientations=[(1.0, 0.0, 0.0, 0.0)]
    )

    # Set the navigation target position
    if not get_prim_at_path(CARTER_NAV_TARGET_PATH).IsValid():
        print(f"Carter navigation target prim not found at path: {CARTER_NAV_TARGET_PATH}, exiting")
        return
    XformPrim(CARTER_NAV_TARGET_PATH, reset_xform_op_properties=True).set_local_poses(
        translations=[CARTER_NAV_TARGET_POSITION], orientations=[(1.0, 0.0, 0.0, 0.0)]
    )

    # Use the carter nova front hawk camera for capturing data
    camera_prim = stage_utils.get_current_stage(backend="usd").GetPrimAtPath(CARTER_CAMERA_PATH)
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
        CARTER_CAMERA_PATH,
        num_clips,
        num_frames_per_clip,
        capture_interval,
        use_instance_id,
        segmentation_mapping,
    )


# Setup the environment and run the example
run_example(
    num_clips=args.num_clips,
    num_frames_per_clip=args.num_frames_per_clip,
    capture_interval=args.capture_interval,
    start_delay=args.start_delay,
    use_instance_id=True,
)

# <start-cosmos-writer-warehouse-test>
test_parser = argparse.ArgumentParser()
test_parser.add_argument(
    "--test",
    action="store_true",
    help="Validate captured output files against expected counts and golden images.",
)
test_args, _ = test_parser.parse_known_args()

if test_args.test:
    import sys

    import omni.kit.app
    from isaacsim.core.utils.extensions import enable_extension

    enable_extension("isaacsim.test.utils")
    enable_extension("isaacsim.replicator.examples")
    from isaacsim.test.utils.file_validation import get_folder_file_summary, validate_folder_contents
    from isaacsim.test.utils.image_comparison import compare_images_in_directories

    image_mean_diff_tolerance = 7.5
    num_modalities = 5
    # shaded_seg, segmentation, and edges are not included in golden images because their output is not persistent
    cosmos_modalities_golden = ("rgb", "depth")
    replicator_examples_ext_path = (
        omni.kit.app.get_app().get_extension_manager().get_extension_path_by_module("isaacsim.replicator.examples")
    )
    golden_dir = os.path.join(
        replicator_examples_ext_path,
        "isaacsim",
        "replicator",
        "examples",
        "tests",
        "data",
        "golden",
        OUT_DIR,
    )
    out_dir = os.path.join(os.getcwd(), OUT_DIR)
    expected_png_count = args.num_frames_per_clip * num_modalities * args.num_clips
    expected_mp4_count = num_modalities * args.num_clips
    ok = validate_folder_contents(
        path=out_dir,
        recursive=True,
        expected_counts={"png": expected_png_count, "mp4": expected_mp4_count},
        fail_on_empty_extensions={"png", "mp4"},
    )
    if not ok:
        summary = get_folder_file_summary(out_dir, recursive=True)
        print(
            f"[SDG][Test][FAIL] Output validation failed for {out_dir}: "
            f"expected png={expected_png_count}, mp4={expected_mp4_count}, found {summary}"
        )
        sys.exit(1)

    for clip_index in range(args.num_clips):
        clip_name = f"clip_{clip_index:04d}"
        clip_golden_dir = os.path.join(golden_dir, clip_name)
        clip_out_dir = os.path.join(out_dir, clip_name)

        for modality in cosmos_modalities_golden:
            modality_golden_dir = os.path.join(clip_golden_dir, modality)
            modality_out_dir = os.path.join(clip_out_dir, modality)
            compare_result = compare_images_in_directories(
                golden_dir=modality_golden_dir,
                test_dir=modality_out_dir,
                path_pattern=rf"^{modality}_.*\.png$",
                allclose_rtol=None,
                allclose_atol=None,
                mean_tolerance=image_mean_diff_tolerance,
                print_all_stats=False,
            )
            if not compare_result["all_passed"]:
                print(
                    f"[SDG][Test][FAIL] {clip_name}/{modality} image comparison failed "
                    f"(tol={image_mean_diff_tolerance}). "
                    f"Golden dir: {modality_golden_dir}, output dir: {modality_out_dir}"
                )
                sys.exit(1)
    print(f"[SDG][Test][PASS] Output validation succeeded for {out_dir}")
# <end-cosmos-writer-warehouse-test>

simulation_app.close()
