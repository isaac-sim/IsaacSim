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

"""Generate offline synthetic dataset."""

import argparse
import json
import math
import os

import numpy as np
import yaml
from isaacsim import SimulationApp

# Default configuration
config = {
    "launch_config": {
        "renderer": "RealTimePathTracing",
        "headless": False,
    },
    "resolution": [512, 512],
    "rt_subframes": 32,
    "num_frames": 4,
    "env_url": "/Isaac/Environments/Simple_Warehouse/full_warehouse.usd",
    "writer": "BasicWriter",
    "backend_type": "DiskBackend",
    "backend_params": {
        "output_dir": "_out_scene_based_sdg",
    },
    "writer_config": {
        "rgb": True,
        "bounding_box_2d_tight": True,
        "semantic_segmentation": True,
        "distance_to_image_plane": True,
        "bounding_box_3d": True,
        "occlusion": True,
    },
    "clear_previous_semantics": True,
    "forklift": {
        "url": "/Isaac/Props/Forklift/forklift.usd",
        "class": "forklift",
    },
    "cone": {
        "url": "/Isaac/Environments/Simple_Warehouse/Props/S_TrafficCone.usd",
        "class": "traffic_cone",
    },
    "pallet": {
        "url": "/Isaac/Environments/Simple_Warehouse/Props/SM_PaletteA_01.usd",
        "class": "pallet",
    },
    "cardbox": {
        "url": "/Isaac/Environments/Simple_Warehouse/Props/SM_CardBoxD_04.usd",
        "class": "cardbox",
    },
}

# Parse command line arguments for optional config file
parser = argparse.ArgumentParser()
parser.add_argument("--config", required=False, help="Include specific config parameters (json or yaml))")
parser.add_argument("--headless", action="store_true", help="Run without a window or viewport rendering")
args, unknown = parser.parse_known_args()

# Load config file if provided
args_config = {}
if args.config and os.path.isfile(args.config):
    print("File exist")
    with open(args.config) as f:
        if args.config.endswith(".json"):
            args_config = json.load(f)
        elif args.config.endswith(".yaml"):
            args_config = yaml.safe_load(f)
        else:
            print(f"[SDG][WARN] File {args.config} is not json or yaml, will use default config")
else:
    print(f"[SDG][WARN] File {args.config} does not exist, will use default config")

# Clear default writer_config if overridden in args
if "writer_config" in args_config:
    config["writer_config"].clear()

# Merge args config into default config
config.update(args_config)
if args.headless:
    config["launch_config"].update({"headless": True, "disable_viewport_updates": True})

# Initialize simulation app
simulation_app = SimulationApp(launch_config=config["launch_config"])

import carb
import carb.settings
import omni.kit.app

# Runtime modules (must import after SimulationApp creation)
import omni.replicator.core as rep
import omni.usd
import scene_based_sdg_utils
from isaacsim.core.experimental.prims import XformPrim
from isaacsim.core.experimental.utils.semantics import add_labels, remove_all_labels
from isaacsim.core.experimental.utils.stage import add_reference_to_stage, define_prim, get_current_stage
from isaacsim.core.experimental.utils.transform import euler_angles_to_quaternion
from isaacsim.storage.native import get_assets_root_path
from pxr import Gf

# Get assets root path from nucleus server
assets_root_path = get_assets_root_path()
if assets_root_path is None:
    carb.log_error("Could not get nucleus server path, closing application..")
    simulation_app.close()

# Load environment stage, or create a minimal plane + dome light when env_url is None
if not scene_based_sdg_utils.setup_environment(config.get("env_url"), assets_root_path):
    carb.log_error(f"Could not open stage {config.get('env_url')}, closing application..")
    simulation_app.close()

# Initialize randomization
rep.set_global_seed(42)
rng = np.random.default_rng(42)

# Configure replicator for manual triggering
rep.orchestrator.set_capture_on_play(False)

# Set DLSS to Quality mode for best SDG results
carb.settings.get_settings().set("rtx/post/dlss/execMode", 2)

# Clear previous semantic labels
if config["clear_previous_semantics"]:
    for prim in get_current_stage().Traverse():
        remove_all_labels(prim, include_descendants=True)

# Create SDG scope for organizing all generated objects
define_prim("/SDG", "Scope")

# Spawn forklift at random pose
forklift_path = "/SDG/Forklift"
forklift_prim = define_prim(forklift_path)
add_reference_to_stage(assets_root_path + config["forklift"]["url"], forklift_path)
add_labels(forklift_prim, labels=[config["forklift"]["class"]], taxonomy="class")
XformPrim(
    forklift_path,
    positions=(rng.uniform(-20, -2), rng.uniform(-1, 3), 0),
    orientations=euler_angles_to_quaternion([0, 0, rng.uniform(0, math.pi)]).numpy(),
    reset_xform_op_properties=True,
)

# Spawn pallet in front of forklift with random offset
forklift_tf = omni.usd.get_world_transform_matrix(forklift_prim)
pallet_offset_tf = Gf.Matrix4d().SetTranslate(Gf.Vec3d(0, rng.uniform(-1.8, -1.2), 0))
pallet_pos = tuple((pallet_offset_tf * forklift_tf).ExtractTranslation())
forklift_quat = forklift_tf.ExtractRotationQuat()
forklift_quat_xyzw = (forklift_quat.GetReal(), *forklift_quat.GetImaginary())

pallet_path = "/SDG/Pallet"
pallet_prim = define_prim(pallet_path)
add_reference_to_stage(assets_root_path + config["pallet"]["url"], pallet_path)
add_labels(pallet_prim, labels=[config["pallet"]["class"]], taxonomy="class")
XformPrim(
    pallet_path,
    positions=pallet_pos,
    orientations=forklift_quat_xyzw,
    reset_xform_op_properties=True,
)

# Create cardboxes for pallet scattering
cardboxes = []
for i in range(5):
    cardbox_path = f"/SDG/CardBox_{i}"
    cardbox = define_prim(cardbox_path)
    add_reference_to_stage(assets_root_path + config["cardbox"]["url"], cardbox_path)
    add_labels(cardbox, labels=[config["cardbox"]["class"]], taxonomy="class")
    cardboxes.append(cardbox)

# Create traffic cone for corner placement
cone_path = "/SDG/Cone"
cone = define_prim(cone_path)
add_reference_to_stage(assets_root_path + config["cone"]["url"], cone_path)
add_labels(cone, labels=[config["cone"]["class"]], taxonomy="class")

# Create cameras
rep.functional.create.scope(name="Cameras", parent="/SDG")
driver_cam = rep.functional.create.camera(
    focus_distance=400.0, focal_length=24.0, clipping_range=(0.1, 10000000.0), name="DriverCam", parent="/SDG/Cameras"
)
pallet_cam = rep.functional.create.camera(name="PalletCam", parent="/SDG/Cameras")
top_view_cam = rep.functional.create.camera(clipping_range=(6.0, 1000000.0), name="TopCam", parent="/SDG/Cameras")

# Setup render products
resolution = config.get("resolution", (512, 512))
forklift_rp = rep.create.render_product(top_view_cam, resolution, name="TopView")
driver_rp = rep.create.render_product(driver_cam, resolution, name="DriverView")
pallet_rp = rep.create.render_product(pallet_cam, resolution, name="PalletView")

render_products = [forklift_rp, driver_rp, pallet_rp]
for render_product in render_products:
    render_product.hydra_texture.set_updates_enabled(False)

# Initialize writer and attach to render products
writer = scene_based_sdg_utils.setup_writer(config)
if not writer:
    carb.log_error("[SDG] Failed to setup writer, closing application.")
    simulation_app.close()

writer.attach(render_products)

for render_product in render_products:
    render_product.hydra_texture.set_updates_enabled(True)

# Configure raytracing subframes for material loading and motion artifacts
rt_subframes = config.get("rt_subframes", -1)


# Calculate camera randomization bounds
pallet_tf = omni.usd.get_world_transform_matrix(pallet_prim)
camera_bounds = scene_based_sdg_utils.setup_camera_bounds(pallet_prim, forklift_prim, pallet_tf, forklift_tf)
pallet_cam_bounds_min = camera_bounds["pallet_cam"]["min"]
pallet_cam_bounds_max = camera_bounds["pallet_cam"]["max"]
top_cam_bounds_min = camera_bounds["top_cam"]["min"]
top_cam_bounds_max = camera_bounds["top_cam"]["max"]
driver_cam_bounds_min = camera_bounds["driver_cam"]["min"]
driver_cam_bounds_max = camera_bounds["driver_cam"]["max"]

# Setup scatter plane and cone placement
scatter_plane = scene_based_sdg_utils.create_scatter_plane_for_prim(
    pallet_prim, pallet_tf, parent_path="/SDG", scale_factor=0.8
)
cone_placement_corners, forklift_rotation_deg = scene_based_sdg_utils.setup_cone_placement_corners(forklift_prim)

# Register graph-based randomizers for lights and materials
scene_based_sdg_utils.register_lights_graph_randomizer(forklift_prim, pallet_prim, event_name="randomize_lights")

cardbox_material_urls = [
    f"{assets_root_path}/Isaac/Environments/Simple_Warehouse/Materials/MI_PaperNotes_01.mdl",
    f"{assets_root_path}/Isaac/Environments/Simple_Warehouse/Materials/MI_CardBoxB_05.mdl",
]
scene_based_sdg_utils.register_cardboxes_materials_graph_randomizer(
    cardboxes, cardbox_material_urls, event_name="randomize_cardboxes_materials"
)

# Run physics simulation to settle boxes on pallet
scene_based_sdg_utils.simulate_falling_objects(forklift_prim, assets_root_path, config, rng=rng)

# SDG loop - generate frames with randomizations
num_frames = config.get("num_frames", 0)
print(f"[SDG] Running SDG for {num_frames} frames")
for i in range(num_frames):
    print(f"[SDG] Frame {i}/{num_frames}")

    print(f"[SDG]  Randomizing boxes on pallet.")
    rep.functional.randomizer.scatter_2d(
        prims=cardboxes, surface_prims=scatter_plane, check_for_collisions=True, rng=rng
    )

    print(f"[SDG]  Randomizing boxes materials.")
    rep.utils.send_og_event(event_name="randomize_cardboxes_materials")
    print(f"[SDG]  Randomizing lights.")
    rep.utils.send_og_event(event_name="randomize_lights")

    print(f"[SDG]  Randomizing pallet camera.")
    rep.functional.modify.pose(
        pallet_cam,
        position_value=rng.uniform(pallet_cam_bounds_min, pallet_cam_bounds_max),
        look_at_value=pallet_prim,
        look_at_up_axis=(0, 0, 1),
    )

    print(f"[SDG]  Randomizing driver camera.")
    rep.functional.modify.pose(
        driver_cam,
        position_value=rng.uniform(driver_cam_bounds_min, driver_cam_bounds_max),
        look_at_value=pallet_prim,
        look_at_up_axis=(0, 0, 1),
    )

    if i % 2 == 0:
        print(f"[SDG]  Randomizing cone position.")
        selected_corner = cone_placement_corners[rng.integers(0, len(cone_placement_corners))]
        rep.functional.modify.pose(
            cone,
            position_value=selected_corner,
        )

    if i % 4 == 0:
        print(f"[SDG]  Randomizing top view camera.")
        roll_angle = rng.uniform(0, 2 * np.pi)
        rep.functional.modify.pose(
            top_view_cam,
            position_value=rng.uniform(top_cam_bounds_min, top_cam_bounds_max),
            look_at_value=forklift_prim,
            look_at_up_axis=(np.cos(roll_angle), np.sin(roll_angle), 0.0),
        )

    print(f"[SDG]  Capturing frame with rt_subframes={rt_subframes}")
    rep.orchestrator.step(delta_time=0.0, rt_subframes=rt_subframes)

# Cleanup
rep.orchestrator.wait_until_complete()
writer.detach()
for render_product in render_products:
    render_product.destroy()

# <start-scene-based-sdg-test>
test_parser = argparse.ArgumentParser()
test_parser.add_argument(
    "--test",
    action="store_true",
    help="Validate captured output files against expected counts and exit.",
)
test_args, _ = test_parser.parse_known_args()

if test_args.test:
    import sys

    from isaacsim.core.utils.extensions import enable_extension

    enable_extension("isaacsim.test.utils")
    enable_extension("isaacsim.replicator.examples")
    from isaacsim.test.utils.file_validation import get_folder_file_summary, validate_folder_contents
    from isaacsim.test.utils.image_comparison import compare_images_in_directories

    rgb_mean_diff_tolerance = 7.5
    num_captures = config.get("num_frames", 4) * 3
    if config.get("backend_type"):
        out_dir = config.get("backend_params", {}).get("output_dir")
    else:
        out_dir = config.get("writer_config", {}).get("output_dir")
    if not out_dir:
        print("[SDG][Test][FAIL] Output directory not configured")
        sys.exit(1)

    ok = validate_folder_contents(
        path=out_dir,
        expected_counts={"png": num_captures},
        recursive=True,
        exact_match=False,
        fail_on_empty_files=True,
    )
    if not ok:
        summary = get_folder_file_summary(out_dir, recursive=True)
        print(
            f"[SDG][Test][FAIL] Output validation failed for {out_dir}\n"
            f"\t Expected at least: {{'png': {num_captures}}}\n"
            f"\t Found: {summary['extension_counts']}"
        )
        sys.exit(1)

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
        "_out_scene_based_sdg",
    )
    compare_rgb = os.path.basename(os.path.normpath(out_dir)) == "_out_scene_based_sdg"
    rgb_results = []
    if compare_rgb:
        for golden_root, _, golden_files in os.walk(golden_dir):
            if any(file_name.endswith(".png") for file_name in golden_files):
                relative_root = os.path.relpath(golden_root, golden_dir)
                test_root = out_dir if relative_root == "." else os.path.join(out_dir, relative_root)
                rgb_results.append(
                    compare_images_in_directories(
                        golden_dir=golden_root,
                        test_dir=test_root,
                        path_pattern=r"^rgb_.*\.png$",
                        allclose_rtol=None,
                        allclose_atol=None,
                        mean_tolerance=rgb_mean_diff_tolerance,
                        print_all_stats=False,
                    )
                )
    if compare_rgb and (not rgb_results or not all(result["all_passed"] for result in rgb_results)):
        print(
            f"[SDG][Test][FAIL] RGB image comparison failed (tol={rgb_mean_diff_tolerance}). "
            f"Golden dir: {golden_dir}, output dir: {out_dir}"
        )
        sys.exit(1)
    print(f"[SDG][Test][PASS] Output validation succeeded for {out_dir} ({num_captures} captures)")
# <end-scene-based-sdg-test>

simulation_app.close()
