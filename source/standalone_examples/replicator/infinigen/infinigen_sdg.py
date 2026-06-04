# SPDX-FileCopyrightText: Copyright (c) 2024-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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

"""Generate synthetic datasets using infinigen (https://infinigen.org/) generated environments."""

import argparse
import json
import os

import yaml
from isaacsim import SimulationApp

# Default config dict, can be updated/replaced using json/yaml config files ('--config' cli argument)
config = {
    "environments": {
        # List of background environments (list of folders or files)
        "folders": ["/Isaac/Samples/Replicator/Infinigen/dining_rooms/"],
        "files": [],
    },
    "capture": {
        # Number of captures (frames = total_captures * num_cameras)
        "total_captures": 9,
        # Number of captures per environment before running the simulation (objects in the air)
        "num_floating_captures_per_env": 2,
        # Number of captures per environment after running the simulation (objects fallen)
        "num_dropped_captures_per_env": 2,
        # Number of cameras to capture from (each camera will have a render product attached)
        "num_cameras": 2,
        # Resolution of the captured frames
        "resolution": (720, 480),
        # Disable render products throughout the piepline, enable them only when capturing the frames
        "disable_render_products": False,
        # Number of subframes to render (RealTimePathTracing) to avoid temporal rendering artifacts (e.g. ghosting)
        "rt_subframes": 8,
        # Use PathTracing renderer or RealTimePathTracing when capturing the frames
        "path_tracing": False,
        # Offset to avoid the images always being in the image center
        "camera_look_at_target_offset": 0.1,
        # Distance between the camera and the target object
        "camera_distance_to_target_range": (1.15, 1.45),
        # Number of scene lights to create in the working area
        "num_scene_lights": 3,
    },
    "writers": [
        {
            # Type of the writer to use (e.g. PoseWriter, BasicWriter, etc.) and the kwargs to pass to the writer init
            "type": "PoseWriter",
            "kwargs": {
                "output_dir": "_out_infinigen_posewriter",
                "format": None,
                "use_subfolders": True,
                "write_debug_images": True,
                "skip_empty_frames": False,
            },
        }
    ],
    "labeled_assets": {
        # Labeled assets with auto-labeling (e.g. 002_banana -> banana) using regex pattern replacement on the asset name
        "auto_label": {
            # Number of labeled assets to create from the given files/folders list
            "num": 6,
            # Chance to disable gravity for the labeled assets (0.0 - all the assets will fall, 1.0 - all the assets will float)
            "gravity_disabled_chance": 0.25,
            # List of folders and files to search for the labeled assets
            "folders": ["/Isaac/Props/YCB/Axis_Aligned/"],
            "files": ["/Isaac/Props/YCB/Axis_Aligned/036_wood_block.usd"],
            # Regex pattern to replace in the asset name (e.g. "002_banana" -> "banana")
            "regex_replace_pattern": r"^\d+_",
            "regex_replace_repl": "",
        },
        # Manually labeled assets with specific labels and properties
        "manual_label": [
            {
                "url": "/Isaac/Props/YCB/Axis_Aligned/008_pudding_box.usd",
                "label": "pudding_box",
                "num": 2,
                "gravity_disabled_chance": 0.25,
            },
            {
                "url": "/Isaac/Props/YCB/Axis_Aligned_Physics/006_mustard_bottle.usd",
                "label": "mustard_bottle",
                "num": 2,
                "gravity_disabled_chance": 0.25,
            },
        ],
    },
    "distractors": {
        # Shape distractors (unlabeled background assets) to drop in the scene (e.g. capsules, cones, cylinders)
        "shape_distractors": {
            # Amount of shape distractors to create
            "num": 20,
            # Chance to disable gravity for the shape distractors
            "gravity_disabled_chance": 0.25,
            # List of shape types to randomly choose from
            "types": ["capsule", "cone", "cylinder", "sphere", "cube"],
        },
        # Mesh distractors (unlabeled background assets) to drop in the scene
        "mesh_distractors": {
            # Amount of mesh distractors to create
            "num": 8,
            # Chance to disable gravity for the mesh distractors
            "gravity_disabled_chance": 0.25,
            # List of folders and files to search to randomly choose from
            "folders": [
                "/Isaac/Environments/Simple_Warehouse/Props/",
                "/Isaac/Environments/Office/Props/",
            ],
            "files": [
                "/Isaac/Environments/Hospital/Props/SM_MedicalBag_01a.usd",
                "/Isaac/Environments/Hospital/Props/SM_MedicalBox_01g.usd",
            ],
        },
    },
    # Physics GPU memory settings for the simulation (override defaults if scenes are complex)
    "physics": {
        # GPU collision stack size in bytes (default: 300 MB, PhysX default is only 64 MB which is
        # insufficient for infinigen scenes with many colliders)
        "gpu_collision_stack_size": 314572800,
    },
    # Hide ceilling to get a top-down view of the scene, move viewport camera to the top-down view
    "debug_mode": True,
}


# Check if there are any config files (yaml or json) are passed as arguments
parser = argparse.ArgumentParser()
parser.add_argument("--config", required=False, help="Include specific config parameters (json or yaml))")
parser.add_argument(
    "--close-on-completion", action="store_true", help="Ensure the app closes on completion even in debug mode"
)
args, unknown = parser.parse_known_args()
args_config = {}
if args.config and os.path.isfile(args.config):
    with open(args.config, "r") as f:
        if args.config.endswith(".json"):
            args_config = json.load(f)
        elif args.config.endswith(".yaml"):
            args_config = yaml.safe_load(f)
        else:
            print(f"[SDG] Warning: Config file {args.config} is not json or yaml, using default config")
else:
    print(f"[SDG] Warning: Config file {args.config} does not exist, using default config")

# Update the default config dict with the external one
config.update(args_config)

simulation_app = SimulationApp(launch_config={"headless": False})


from itertools import cycle

import carb.settings
import infinigen_sdg_utils as infinigen_utils
import numpy as np
import omni.kit.app
import omni.replicator.core as rep
import omni.usd
from isaacsim.core.utils.viewports import set_camera_view


# Run the SDG pipeline on the scenarios
def run_sdg(config):
    """Run the synthetic data generation pipeline on the configured scenarios."""
    # Load the config parameters
    env_config = config.get("environments", {})
    env_urls = infinigen_utils.get_usd_paths(
        files=env_config.get("files", []), folders=env_config.get("folders", []), skip_folder_keywords=[".thumbs"]
    )
    if not env_urls:
        print("[SDG] Error: No environment USD files found. Please check the 'environments' config.")
        return
    print(f"[SDG] Found {len(env_urls)} environment(s)")
    capture_config = config.get("capture", {})
    writers_config = config.get("writers", {})
    labeled_assets_config = config.get("labeled_assets", {})
    distractors_config = config.get("distractors", {})

    # Create a new stage
    print("[SDG] Creating a new stage")
    omni.usd.get_context().new_stage()
    stage = omni.usd.get_context().get_stage()

    # Disable capture on play
    rep.orchestrator.set_capture_on_play(False)

    # Disable UJITSO cooking ([Warning] [omni.ujitso] UJITSO : Build storage validation failed)
    carb.settings.get_settings().set("/physics/cooking/ujitsoCollisionCooking", False)

    # Set DLSS to Quality mode (2) for best SDG results , options: 0 (Performance), 1 (Balanced), 2 (Quality), 3 (Auto)
    carb.settings.get_settings().set("rtx/post/dlss/execMode", 2)

    # Initialize randomization
    rep.set_global_seed(12)
    rng = np.random.default_rng(12)

    # Debug mode (hide ceiling, move viewport camera to the top-down view)
    debug_mode = config.get("debug_mode", False)

    # Create the cameras
    cameras = []
    num_cameras = capture_config.get("num_cameras", 0)
    rep.functional.create.scope(name="Cameras")
    for i in range(num_cameras):
        cam_prim = rep.functional.create.camera(parent="/Cameras", name=f"cam_{i}", clipping_range=(0.25, 1000))
        cameras.append(cam_prim)
    print(f"[SDG] Created {len(cameras)} cameras")

    # Create the render products for the cameras
    render_products = []
    resolution = capture_config.get("resolution", (1280, 720))
    disable_render_products = capture_config.get("disable_render_products", False)
    for cam in cameras:
        rp = rep.create.render_product(cam.GetPath(), resolution, name=f"rp_{cam.GetName()}")
        if disable_render_products:
            rp.hydra_texture.set_updates_enabled(False)
        render_products.append(rp)
    print(f"[SDG] Created {len(render_products)} render products")

    # Only create the writers if there are render products to attach to
    writers = []
    if render_products:
        for writer_config in writers_config:
            writer = infinigen_utils.setup_writer(writer_config)
            if writer:
                writer.attach(render_products)
                writers.append(writer)
                print(
                    f"[SDG] {writer_config['type']}'s out dir: {writer_config.get('kwargs', {}).get('output_dir', '')}"
                )
    print(f"[SDG] Created {len(writers)} writers")

    # Load target assets with auto-labeling (e.g. 002_banana -> banana)
    auto_label_config = labeled_assets_config.get("auto_label", {})
    auto_floating_assets, auto_falling_assets = infinigen_utils.load_auto_labeled_assets(auto_label_config, rng)
    print(f"[SDG] Loaded {len(auto_floating_assets)} floating auto-labeled assets")
    print(f"[SDG] Loaded {len(auto_falling_assets)} falling auto-labeled assets")

    # Load target assets with manual labels
    manual_label_config = labeled_assets_config.get("manual_label", [])
    manual_floating_assets, manual_falling_assets = infinigen_utils.load_manual_labeled_assets(manual_label_config, rng)
    print(f"[SDG] Loaded {len(manual_floating_assets)} floating manual-labeled assets")
    print(f"[SDG] Loaded {len(manual_falling_assets)} falling manual-labeled assets")
    target_assets = auto_floating_assets + auto_falling_assets + manual_floating_assets + manual_falling_assets

    # Load the shape distractors
    shape_distractors_config = distractors_config.get("shape_distractors", {})
    floating_shapes, falling_shapes = infinigen_utils.load_shape_distractors(shape_distractors_config, rng)
    print(f"[SDG] Loaded {len(floating_shapes)} floating shape distractors")
    print(f"[SDG] Loaded {len(falling_shapes)} falling shape distractors")
    shape_distractors = floating_shapes + falling_shapes

    # Load the mesh distractors
    mesh_distractors_config = distractors_config.get("mesh_distractors", {})
    floating_meshes, falling_meshes = infinigen_utils.load_mesh_distractors(mesh_distractors_config, rng)
    print(f"[SDG] Loaded {len(floating_meshes)} floating mesh distractors")
    print(f"[SDG] Loaded {len(falling_meshes)} falling mesh distractors")
    mesh_distractors = floating_meshes + falling_meshes

    # Resolve any centimeter-meter scale issues of the assets
    infinigen_utils.resolve_scale_issues_with_metrics_assembler()

    # Create lights to randomize in the working area
    scene_lights = []
    num_scene_lights = capture_config.get("num_scene_lights", 0)
    for i in range(num_scene_lights):
        light_prim = stage.DefinePrim(f"/Lights/SphereLight_scene_{i}", "SphereLight")
        scene_lights.append(light_prim)
    print(f"[SDG] Created {len(scene_lights)} scene lights")

    # Register replicator randomizers and trigger them once
    print("[SDG] Registering replicator graph randomizers")
    infinigen_utils.register_dome_light_randomizer()
    infinigen_utils.register_shape_distractors_color_randomizer(shape_distractors)

    # Configure the PhysX scene GPU memory settings before running any simulation.
    # This prevents PxGpuDynamicsMemoryConfig::collisionStackSize buffer overflow errors
    # when simulating complex scenes with many colliders (distractors, assets, environment meshes).
    physics_config = config.get("physics", {})
    print("[SDG] Configuring physics scene GPU memory settings")
    infinigen_utils.configure_physics_scene(physics_config)

    # Check if the render mode needs to be switched to path tracing for the capture (by default: RealTimePathTracing)
    use_path_tracing = capture_config.get("path_tracing", False)

    # Capture detail using subframes (https://docs.omniverse.nvidia.com/extensions/latest/ext_replicator/subframes_examples.html)
    rt_subframes = capture_config.get("rt_subframes", 3)

    # Min and max distance between the camera and the target object
    camera_distance_to_target_range = capture_config.get("camera_distance_to_target_range", (0.5, 1.5))

    # Number of captures (frames = total_captures * num_cameras)
    # NOTE: if captured frames have no labeled data, they can be skipped (e.g. PoseWriter with skip_empty_frames=True)
    total_captures = capture_config.get("total_captures", 0)

    # Number of captures per environment with the objects in the air or dropped
    num_floating_captures_per_env = capture_config.get("num_floating_captures_per_env", 0)
    num_dropped_captures_per_env = capture_config.get("num_dropped_captures_per_env", 0)

    # Start the SDG loop
    env_cycle = cycle(env_urls)
    capture_counter = 0
    while capture_counter < total_captures:
        # Load the next environment
        env_url = next(env_cycle)

        # Load the new environment
        print(f"[SDG] Loading environment: {env_url}")
        infinigen_utils.load_env(env_url, prim_path="/Environment")

        # Setup the environment (add collision, fix lights, etc.) and update the app once to apply the changes
        print(f"[SDG] Setting up the environment")
        infinigen_utils.setup_env(root_path="/Environment", hide_top_walls=debug_mode)
        simulation_app.update()

        # Get the location of the prim above which the assets will be randomized
        working_area_loc = infinigen_utils.get_matching_prim_location(
            match_string="TableDining", root_path="/Environment"
        )

        # Move viewport above the working area to get a top-down view of the scene
        if debug_mode:
            camera_loc = (working_area_loc[0], working_area_loc[1], working_area_loc[2] + 10)
            set_camera_view(eye=np.array(camera_loc), target=np.array(working_area_loc))

        # Get the spawn areas as offseted location ranges from the working area (min_x, min_y, min_z, max_x, max_y, max_z)
        print(f"[SDG] Randomizing {len(target_assets)} target assets around the working area")
        target_loc_range = infinigen_utils.offset_range((-0.5, -0.5, 1, 0.5, 0.5, 1.5), working_area_loc)
        infinigen_utils.randomize_poses(
            target_assets,
            location_range=target_loc_range,
            rotation_range=(0, 360),
            scale_range=(0.95, 1.15),
            rng=rng,
        )

        # Mesh distractors
        print(f"[SDG] Randomizing {len(mesh_distractors)} mesh distractors around the working area")
        mesh_loc_range = infinigen_utils.offset_range((-1, -1, 1, 1, 1, 2), working_area_loc)
        infinigen_utils.randomize_poses(
            mesh_distractors,
            location_range=mesh_loc_range,
            rotation_range=(0, 360),
            scale_range=(0.3, 1.0),
            rng=rng,
        )

        # Shape distractors
        print(f"[SDG] Randomizing {len(shape_distractors)} shape distractors around the working area")
        shape_loc_range = infinigen_utils.offset_range((-1.5, -1.5, 1, 1.5, 1.5, 2), working_area_loc)
        infinigen_utils.randomize_poses(
            shape_distractors,
            location_range=shape_loc_range,
            rotation_range=(0, 360),
            scale_range=(0.01, 0.1),
            rng=rng,
        )

        print(f"[SDG] Randomizing {len(scene_lights)} scene lights properties and locations around the working area")
        lights_loc_range = infinigen_utils.offset_range((-2, -2, 1, 2, 2, 3), working_area_loc)
        infinigen_utils.randomize_lights(
            scene_lights,
            location_range=lights_loc_range,
            intensity_range=(500, 2500),
            color_range=(0.1, 0.1, 0.1, 0.9, 0.9, 0.9),
            rng=rng,
        )

        print("[SDG] Randomizing dome lights")
        rep.utils.send_og_event(event_name="randomize_dome_lights")

        print("[SDG] Randomizing shape distractor colors")
        rep.utils.send_og_event(event_name="randomize_shape_distractor_colors")

        # Run the physics simulation for a few frames to solve any collisions
        print("[SDG] Fixing collisions through physics simulation")
        simulation_app.update()
        infinigen_utils.run_simulation(num_frames=4, render=True)

        # Check if the render products need to be enabled for the capture
        if disable_render_products:
            for rp in render_products:
                rp.hydra_texture.set_updates_enabled(True)

        # Check if the render mode needs to be switched to path tracing for the capture
        if use_path_tracing:
            print("[SDG] Switching to PathTracing render mode")
            carb.settings.get_settings().set("/rtx/rendermode", "PathTracing")

        # Capture frames with the objects in the air
        for i in range(num_floating_captures_per_env):
            # Check if the total captures have been reached
            if capture_counter >= total_captures:
                break
            # Randomize the camera poses
            print(f"[SDG] Randomizing camera poses ({len(cameras)} cameras)")
            infinigen_utils.randomize_camera_poses(
                cameras, target_assets, camera_distance_to_target_range, polar_angle_range=(0, 75), rng=rng
            )
            print(
                f"[SDG] Capturing floating assets {i+1}/{num_floating_captures_per_env} (total: {capture_counter+1}/{total_captures})"
            )
            rep.orchestrator.step(rt_subframes=rt_subframes, delta_time=0.0)
            capture_counter += 1

        # Check if the render products need to be disabled until the next capture
        if disable_render_products:
            for rp in render_products:
                rp.hydra_texture.set_updates_enabled(False)

        # Check if the render mode needs to be switched back to raytracing until the next capture
        if use_path_tracing:
            carb.settings.get_settings().set("/rtx/rendermode", "RealTimePathTracing")

        print("[SDG] Running the simulation")
        infinigen_utils.run_simulation(num_frames=200, render=False)

        # Check if the render products need to be enabled for the capture
        if disable_render_products:
            for rp in render_products:
                rp.hydra_texture.set_updates_enabled(True)

        # Check if the render mode needs to be switched to path tracing for the capture
        if use_path_tracing:
            carb.settings.get_settings().set("/rtx/rendermode", "PathTracing")

        for i in range(num_dropped_captures_per_env):
            # Check if the total captures have been reached
            if capture_counter >= total_captures:
                break
            # Spawn the cameras with a smaller polar angle to have mostly a top-down view of the objects
            print("[SDG] Randomizing camera poses")
            infinigen_utils.randomize_camera_poses(
                cameras,
                target_assets,
                distance_range=camera_distance_to_target_range,
                polar_angle_range=(0, 45),
                rng=rng,
            )
            print(
                f"[SDG] Capturing dropped assets {i+1}/{num_dropped_captures_per_env} (total: {capture_counter+1}/{total_captures})"
            )
            rep.orchestrator.step(rt_subframes=rt_subframes, delta_time=0.0)
            capture_counter += 1

        # Check if the render products need to be disabled until the next capture
        if disable_render_products:
            for rp in render_products:
                rp.hydra_texture.set_updates_enabled(False)

        # Check if the render mode needs to be switched back to raytracing until the next capture
        if use_path_tracing:
            carb.settings.get_settings().set("/rtx/rendermode", "RealTimePathTracing")

    # Wait until the data is written to the disk
    rep.orchestrator.wait_until_complete()

    # Detach the writers
    print("[SDG] Detaching writers")
    for writer in writers:
        writer.detach()

    # Destroy render products
    print("[SDG] Destroying render products")
    for rp in render_products:
        rp.destroy()

    print(f"[SDG] Finished, captured {capture_counter * num_cameras} frames")


# Check if debug mode is enabled
debug_mode = config.get("debug_mode", False)

# Start the SDG pipeline
print("[SDG] Starting the SDG pipeline")
run_sdg(config)
print("[SDG] Pipeline finished")

# Make sure the app closes on completion even if in debug mode
if args.close_on_completion:
    simulation_app.close()

# In debug mode, keep the app running until manually closed
if debug_mode:
    while simulation_app.is_running():
        simulation_app.update()

simulation_app.close()
