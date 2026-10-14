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

"""Demonstrate synthetic data generation with deformable physics objects."""

from isaacsim import SimulationApp

simulation_app = SimulationApp(launch_config={"headless": False})

import os
import random

import carb.settings
import isaacsim.core.experimental.utils.app as app_utils
import isaacsim.core.experimental.utils.stage as stage_utils
import omni.replicator.core as rep
from isaacsim.core.experimental.materials import VolumeDeformableMaterial
from isaacsim.core.experimental.prims import DeformablePrim
from isaacsim.core.simulation_manager import SimulationManager
from isaacsim.storage.native import get_assets_root_path
from pxr import Gf, Sdf, Usd, UsdShade

TRIGGER_HEIGHT = 0.15  # Capture when lowest vertex falls below this (m)
BASE_DROP_HEIGHT = 0.2  # Starting height for first asset (m)
HEIGHT_STEP = 0.15  # Height increment between assets (m)
SPAWN_XY_JITTER = 0.07  # Random horizontal offset +/- (m)
RNG_SEED = 12  # Reproducible randomization seed
MAX_STEPS = 200  # Maximum simulation steps
CRATE_USD = "/Isaac/Props/PackingTable/props/SM_Crate_A08_Blue_01/SM_Crate_A08_Blue_01.usd"

# (label, count, usd_path, youngs_modulus_Pa [higher=stiffer], poissons_ratio [0=compressible, 0.5=incompressible])
ASSETS_CONFIG = [
    ("banana", 2, "/Isaac/Props/YCB/Axis_Aligned/011_banana.usd", 500_000, 0.45),
    ("large_marker", 2, "/Isaac/Props/YCB/Axis_Aligned/040_large_marker.usd", 9_000_000, 0.5),
]


def run_example(assets_config: list[tuple[str, int, str, float, float]]) -> None:
    """Run deformable drop simulation and capture frames on trigger height.

    Args:
        assets_config: Asset label, instance count, USD path, Young's modulus, and Poisson ratio for each object type.
    """
    stage_utils.create_new_stage()
    assets_root_path = get_assets_root_path()
    rng = random.Random(RNG_SEED)

    # Disable capture on play, frames will be captured manually
    rep.orchestrator.set_capture_on_play(False)

    # Set DLSS to Quality mode (2) for best SDG results (Options: 0 (Performance), 1 (Balanced), 2 (Quality), 3 (Auto)
    carb.settings.get_settings().set("rtx/post/dlss/execMode", 2)

    rep.functional.create.xform(name="World")
    rep.functional.create.dome_light(intensity=500, parent="/World", name="DomeLight")
    ground = rep.functional.create.cube(position=(0, 0, -0.5), scale=(10, 10, 1))  # Thick to prevent tunneling
    rep.functional.physics.apply_collider(ground)
    crate = rep.functional.create.reference(
        usd_path=assets_root_path + CRATE_USD,
        position=(0, 0, 0),
        scale=0.01,
        semantics={"class": "crate"},
        parent="/World",
        name="Crate",
    )
    rep.functional.physics.apply_collider(crate, approximation="none")  # Triangle mesh for concave geometry

    # Create the deformable physics materials, one per asset type
    materials = {}
    for label, _, _, youngs_modulus, poissons_ratio in assets_config:
        materials[label] = VolumeDeformableMaterial(
            f"/World/physics_materials/{label}",
            youngs_moduli=[float(youngs_modulus)],
            poissons_ratios=[float(poissons_ratio)],
        )
        print(f"[SDG]  Created deformable material for {label}: {materials[label].paths[0]}")

    # Create asset prims: cook the first of each type, clone the rest
    # Clones inherit all deformable physics, avoiding redundant cooking
    all_prims = []
    labels = []
    for label, count, usd_path, _, _ in assets_config:
        for i in range(count):
            if i == 0:
                # Create + cook deformable physics + apply material
                prim = rep.functional.create.reference(
                    usd_path=assets_root_path + usd_path,
                    semantics={"class": label},
                    parent="/World",
                    name=f"{label.capitalize()}_{i}",
                )
                first_path = prim.GetPath().pathString
                deformable = DeformablePrim(first_path, deformable_type="volume")
                deformable.apply_physics_materials(materials[label])
            else:
                # Clone inherits cooked deformable physics and material
                prim = rep.functional.create.clone(
                    first_path,
                    parent="/World",
                    name=f"{label.capitalize()}_{i}",
                )
            all_prims.append(prim)
            labels.append(label)
            print(f"[SDG]  {prim.GetPath()} ({'created' if i == 0 else 'cloned'})")
    total = len(all_prims)

    # Assign random drop heights and Z-axis rotations
    positions, rotations = [], []
    for i in range(total):
        x = rng.uniform(-SPAWN_XY_JITTER, SPAWN_XY_JITTER)
        y = rng.uniform(-SPAWN_XY_JITTER, SPAWN_XY_JITTER)
        z = BASE_DROP_HEIGHT + i * HEIGHT_STEP
        positions.append((x, y, z))
        rotations.append((0, 0, rng.uniform(0, 360)))
    rep.functional.modify.pose(all_prims, position_value=positions, rotation_value=rotations)

    # Cache asset shader prims for material randomization, each capture trigger the shader color will change once
    shaders = []
    for prim in all_prims:
        asset_shaders = []
        for child in Usd.PrimRange(prim):
            if child.IsA(UsdShade.Shader):
                asset_shaders.append(child)
        shaders.append(asset_shaders)

    # Replicator setup, render product is disabled by default and enabled only at capture time
    camera = rep.functional.create.camera(position=(1, 1, 1), look_at=(0, 0, 0), parent="/World", name="Camera")
    render_product = rep.create.render_product(camera, (720, 480))
    render_product.hydra_texture.set_updates_enabled(False)
    output_dir = os.path.join(os.getcwd(), "_out_deformable_drop")
    backend = rep.backends.get("DiskBackend")
    backend.initialize(output_dir=output_dir)
    writer = rep.writers.get("BasicWriter")
    # For deformables the bounding_box_3d transform stays at the spawn pose and the deformation is
    # carried by the local extents, so compose transform x extents to get the world space box.
    writer.initialize(
        backend=backend,
        rgb=True,
        semantic_segmentation=True,
        colorize_semantic_segmentation=True,
        bounding_box_3d=True,
    )
    writer.attach(render_product)

    # GPU physics required for deformable tensor API
    SimulationManager.set_physics_sim_device("cuda")
    SimulationManager.initialize_physics()

    # Keep track of which assets have been triggered
    triggered = [False] * total

    # Start the simulation
    print(f"[SDG] Starting simulation")
    app_utils.play()

    # Wrap deformables for tensor API access (requires active simulation, no re-cooking)
    deformables = []
    for i in range(total):
        deformables.append(DeformablePrim(all_prims[i].GetPath().pathString))

    for _ in range(MAX_STEPS):
        # Advance the app which will advance the timeline (and implicitly the simulation)
        app_utils.update_app()

        # Detect assets whose lowest vertex crossed the trigger height
        newly_triggered = []
        for i in range(total):
            # Skip if the asset has already been triggered
            if triggered[i]:
                continue

            # Use the deformable prim's get_nodal_positions to get the actual mesh vertices (not xform, which is constant for deformables)
            node_positions, _, _ = deformables[i].get_nodal_positions()
            min_z = float(node_positions[0].numpy()[:, 2].min())

            # If the lowest vertex is below the trigger height, trigger the asset
            if min_z <= TRIGGER_HEIGHT:
                triggered[i] = True
                newly_triggered.append(i)

        # If a new asset has been triggered, enable the render product, randomize the material color and capture the asset
        if newly_triggered:
            render_product.hydra_texture.set_updates_enabled(True)
        for i in newly_triggered:
            color = Gf.Vec3f(rng.random(), rng.random(), rng.random())
            for shader_prim in shaders[i]:
                attr = shader_prim.GetAttribute("inputs:diffuse_tint")
                if not attr or not attr.IsValid():
                    attr = shader_prim.CreateAttribute("inputs:diffuse_tint", Sdf.ValueTypeNames.Color3f)
                attr.Set(color)
            print(f"[SDG]  Captured {all_prims[i].GetPath()}")
            rep.orchestrator.step(delta_time=0.0, pause_timeline=False, rt_subframes=16)
        if newly_triggered:
            render_product.hydra_texture.set_updates_enabled(False)

        # If all assets have been triggered, stop the simulation
        if all(triggered):
            break

    # Pause the simulation and clean up resources
    print(f"[SDG] Simulation complete. {len(triggered)} frames saved to {output_dir}")
    app_utils.pause()
    rep.orchestrator.wait_until_complete()
    writer.detach()
    render_product.destroy()


run_example(ASSETS_CONFIG)

# <start-sdg-deformables-test>
import argparse

test_parser = argparse.ArgumentParser()
test_parser.add_argument(
    "--test",
    action="store_true",
    help="Validate captured output files against golden data and exit.",
)
test_args, _ = test_parser.parse_known_args()

if test_args.test:
    import json
    import sys

    import numpy as np
    import omni.kit.app
    from isaacsim.core.utils.extensions import enable_extension

    enable_extension("isaacsim.test.utils")
    enable_extension("isaacsim.replicator.examples")
    from isaacsim.test.utils.file_validation import get_folder_file_summary, validate_folder_contents
    from isaacsim.test.utils.image_comparison import compare_images_in_directories

    out_dir = os.path.join(os.getcwd(), "_out_deformable_drop")
    rgb_mean_diff_tolerance = 5
    bbox3d_tolerance = 0.02
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
        "_out_deformable_drop",
    )
    num_assets = sum(count for _, count, _, _, _ in ASSETS_CONFIG)
    expected_pngs = num_assets * 2  # rgb + colorized semantic segmentation per capture
    # semantic segmentation labels, plus bounding box 3d labels and prim paths, per capture
    expected_json = num_assets * 3
    expected_npy = num_assets  # bounding box 3d data per capture
    ok = validate_folder_contents(
        path=out_dir,
        recursive=True,
        expected_counts={"png": expected_pngs, "json": expected_json, "npy": expected_npy},
        fail_on_empty_extensions={"png", "json", "npy"},
    )
    if not ok:
        summary = get_folder_file_summary(out_dir, recursive=True)
        print(
            f"[SDG][Test][FAIL] Output validation failed for {out_dir}: "
            f"expected png={expected_pngs}, json={expected_json}, npy={expected_npy}, found {summary}"
        )
        sys.exit(1)

    # Semantic segmentation colors are not persistent, so compare only RGB images against golden data.
    rgb_result = compare_images_in_directories(
        golden_dir=golden_dir,
        test_dir=out_dir,
        path_pattern=r"^rgb_.*\.png$",
        allclose_rtol=None,
        allclose_atol=None,
        mean_tolerance=rgb_mean_diff_tolerance,
        print_all_stats=True,
    )
    if not rgb_result["all_passed"]:
        print(
            f"[SDG][Test][FAIL] RGB image comparison failed (tol={rgb_mean_diff_tolerance}). "
            f"Golden dir: {golden_dir}, output dir: {out_dir}"
        )
        sys.exit(1)

    # Row order and semantic ids are not stable between runs, so key the boxes by prim path.
    # Only the geometry is compared; occlusionRatio is -1 for the deformables because their
    # semantic root spans the render mesh plus the cooked simulation and collision meshes.
    def load_bbox3d_by_prim_path(directory: str, index: int) -> dict:
        boxes = np.load(os.path.join(directory, f"bounding_box_3d_{index:04d}.npy"))
        with open(os.path.join(directory, f"bounding_box_3d_prim_paths_{index:04d}.json")) as prim_paths_file:
            prim_paths = json.load(prim_paths_file)
        extent_fields = ["x_min", "y_min", "z_min", "x_max", "y_max", "z_max"]
        return {
            prim_path: np.concatenate([[box[field] for field in extent_fields], np.ravel(box["transform"])])
            for prim_path, box in zip(prim_paths, boxes)
        }

    for frame in range(num_assets):
        golden_boxes = load_bbox3d_by_prim_path(golden_dir, frame)
        test_boxes = load_bbox3d_by_prim_path(out_dir, frame)
        if golden_boxes.keys() != test_boxes.keys():
            print(
                f"[SDG][Test][FAIL] Bounding box 3d prim paths differ in frame {frame}: "
                f"only in golden={sorted(golden_boxes.keys() - test_boxes.keys())}, "
                f"only in output={sorted(test_boxes.keys() - golden_boxes.keys())}"
            )
            sys.exit(1)
        for prim_path, golden_box in golden_boxes.items():
            max_diff = float(np.abs(golden_box - test_boxes[prim_path]).max())
            if max_diff > bbox3d_tolerance:
                print(
                    f"[SDG][Test][FAIL] Bounding box 3d mismatch in frame {frame} for {prim_path}: "
                    f"max diff {max_diff:.6f} exceeds tolerance {bbox3d_tolerance}"
                )
                sys.exit(1)

    print(f"[SDG][Test][PASS] Output validation succeeded for {out_dir}")
# <end-sdg-deformables-test>

simulation_app.close()
