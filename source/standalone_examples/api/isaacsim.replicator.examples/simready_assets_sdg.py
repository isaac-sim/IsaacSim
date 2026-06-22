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

"""Demonstrate synthetic data generation using SimReady assets with randomized scenes."""

from isaacsim import SimulationApp

simulation_app = SimulationApp(launch_config={"headless": False})

import argparse
import asyncio
import os
import time
from typing import Any

import carb.settings
import numpy as np
import omni.kit.app
import omni.replicator.core as rep
import omni.usd
from isaacsim.core.experimental.utils.semantics import upgrade_prim_semantics_to_labels
from isaacsim.core.simulation_manager import SimulationManager
from pxr import Sdf, Usd, UsdGeom, UsdPhysics

parser = argparse.ArgumentParser()
parser.add_argument("--num_scenarios", type=int, default=5, help="Number of randomization scenarios to create")
args, _ = parser.parse_known_args()
num_scenarios = args.num_scenarios

# Make sure the simready explorer extension is enabled
ext_manager = omni.kit.app.get_app().get_extension_manager()
if not ext_manager.is_extension_enabled("omni.simready.explorer"):
    ext_manager.set_extension_enabled_immediate("omni.simready.explorer", True)
import omni.simready.explorer as sre


def enable_simready_explorer() -> None:
    """Enable the SimReady Explorer window if not already open."""
    if sre.get_instance().browser_model is None:
        import omni.kit.actions.core as actions

        actions.execute_action("omni.simready.explorer", "toggle_window")


def set_prim_variants(prim: Usd.Prim, variants: dict[str, str]) -> None:
    """Set variant selections on a prim from a dictionary of variant set names to values."""
    vsets = prim.GetVariantSets()
    for name, value in variants.items():
        vset = vsets.GetVariantSet(name)
        if vset:
            vset.SetVariantSelection(value)


async def search_assets_async() -> tuple[list, list, list]:
    """Search for SimReady assets (tables, dishes, items) asynchronously."""
    print(f"[SDG] Searching for SimReady assets...")
    start_time = time.time()
    tables = await sre.find_assets(["table", "furniture"])
    print(f"[SDG]   - Found {len(tables)} tables ({time.time() - start_time:.2f}s)")
    start_time = time.time()
    plates = await sre.find_assets(["plate"])
    print(f"[SDG]   - Found {len(plates)} plates ({time.time() - start_time:.2f}s)")
    start_time = time.time()
    bowls = await sre.find_assets(["bowl"])
    print(f"[SDG]   - Found {len(bowls)} bowls ({time.time() - start_time:.2f}s)")
    dishes = plates + bowls
    start_time = time.time()
    fruits = await sre.find_assets(["fruit"])
    print(f"[SDG]   - Found {len(fruits)} fruits ({time.time() - start_time:.2f}s)")
    start_time = time.time()
    vegetables = await sre.find_assets(["vegetable"])
    print(f"[SDG]   - Found {len(vegetables)} vegetables ({time.time() - start_time:.2f}s)")
    items = fruits + vegetables
    return tables, dishes, items


def run_simready_randomization(
    stage: Usd.Stage,
    camera_prim: Usd.Prim,
    render_product: Any,
    tables: list,
    dishes: list,
    items: list,
    rng: np.random.Generator = None,
) -> None:
    """Randomize a scene with SimReady assets, run physics, and capture the result."""
    if rng is None:
        rng = np.random.default_rng()

    print(f"[SDG]   Creating anonymous variation layer for the randomizations...")
    root_layer = stage.GetRootLayer()
    variation_layer = Sdf.Layer.CreateAnonymous("variation")
    root_layer.subLayerPaths.insert(0, variation_layer.identifier)
    stage.SetEditTarget(variation_layer)

    # Load the simready assets with rigid body properties
    variants = {"PhysicsVariant": "RigidBody"}
    rep.functional.create.scope(name="Assets")

    # Choose a random table and add it to the stage
    print(f"[SDG]   Loading assets...")
    table_asset = tables[rng.integers(len(tables))]
    start_time = time.time()
    table_prim = rep.functional.create.reference(usd_path=table_asset.main_url, parent="/Assets", name=table_asset.name)
    set_prim_variants(table_prim, variants)
    upgrade_prim_semantics_to_labels(table_prim)
    print(f"[SDG]     - Table: '{table_asset.name}' ({time.time() - start_time:.2f}s)")
    simulation_app.update()

    # Keep only colliders on the table (disable rigid body dynamics)
    UsdPhysics.RigidBodyAPI(table_prim).GetRigidBodyEnabledAttr().Set(False)

    # Compute table dimensions from its bounding box
    bbox_cache = UsdGeom.BBoxCache(Usd.TimeCode.Default(), includedPurposes=[UsdGeom.Tokens.default_])
    table_bbox = bbox_cache.ComputeWorldBound(table_prim)
    table_extent = table_bbox.GetRange().GetSize()

    # Choose a random dish and add it to the stage
    dish_asset = dishes[rng.integers(len(dishes))]
    start_time = time.time()
    dish_prim = rep.functional.create.reference(usd_path=dish_asset.main_url, parent="/Assets", name=dish_asset.name)
    set_prim_variants(dish_prim, variants)
    upgrade_prim_semantics_to_labels(dish_prim)
    print(f"[SDG]     - Dish: '{dish_asset.name}' ({time.time() - start_time:.2f}s)")
    simulation_app.update()

    # Compute dish dimensions from its bounding box
    dish_bbox = bbox_cache.ComputeWorldBound(dish_prim)
    dish_extent = dish_bbox.GetRange().GetSize()

    # Calculate random position for the dish near the center of the table
    center_region_scale = 0.75
    dish_range_x = max(0, (table_extent[0] - dish_extent[0]) / 2 * center_region_scale)
    dish_range_y = max(0, (table_extent[1] - dish_extent[1]) / 2 * center_region_scale)
    dish_position = (
        rng.uniform(-dish_range_x, dish_range_x) if dish_range_x > 0 else 0,
        rng.uniform(-dish_range_y, dish_range_y) if dish_range_y > 0 else 0,
        table_extent[2] + dish_extent[2] / 2,
    )
    dish_prim.GetAttribute("xformOp:translate").Set(dish_position)

    # Add random items above the dish
    num_items = rng.integers(2, 5)
    item_prims = []
    for _ in range(num_items):
        item_asset = items[rng.integers(len(items))]
        start_time = time.time()
        item_prim = rep.functional.create.reference(
            usd_path=item_asset.main_url, parent="/Assets", name=item_asset.name
        )
        set_prim_variants(item_prim, variants)
        upgrade_prim_semantics_to_labels(item_prim)
        print(f"[SDG]     - Item: '{item_asset.name}' ({time.time() - start_time:.2f}s)")
        item_prims.append(item_prim)
        simulation_app.update()

    # Position items stacked above the dish
    print(f"[SDG]   Positioning assets on table...")
    stack_height = dish_position[2]
    item_scatter_radius = max(0, dish_extent[0] / 4)
    for item_prim in item_prims:
        item_bbox = bbox_cache.ComputeWorldBound(item_prim)
        item_extent = item_bbox.GetRange().GetSize()
        scatter_x = rng.uniform(-item_scatter_radius, item_scatter_radius) if item_scatter_radius > 0 else 0
        scatter_y = rng.uniform(-item_scatter_radius, item_scatter_radius) if item_scatter_radius > 0 else 0
        item_position = (
            dish_position[0] + scatter_x,
            dish_position[1] + scatter_y,
            stack_height + item_extent[2] / 2,
        )
        item_prim.GetAttribute("xformOp:translate").Set(item_position)
        stack_height += item_extent[2]

    # Run physics simulation for items to settle (SimulationManager handles warmup on init)
    num_sim_steps = 25
    print(f"[SDG]   Running physics simulation ({num_sim_steps} steps)...")
    SimulationManager.invalidate_physics()
    SimulationManager.initialize_physics()
    SimulationManager.step(steps=num_sim_steps)

    print(f"[SDG]   Setting edit target to root layer...")
    stage.SetEditTarget(root_layer)

    print(f"[SDG]   Positioning camera and capturing frame...")
    camera_position = (
        dish_position[0] + rng.uniform(-0.5, 0.5),
        dish_position[1] + rng.uniform(-0.5, 0.5),
        dish_position[2] + 1.5 + rng.uniform(-0.5, 0.5),
    )
    rep.functional.modify.pose(
        camera_prim, position_value=camera_position, look_at_value=dish_prim, look_at_up_axis=(0, 0, 1)
    )
    render_product.hydra_texture.set_updates_enabled(True)
    rep.orchestrator.step(delta_time=0.0, rt_subframes=16)
    render_product.hydra_texture.set_updates_enabled(False)

    print(f"[SDG]   Removing temp variation layer...")
    variation_layer.Clear()
    root_layer.subLayerPaths.remove(variation_layer.identifier)


def run_simready_randomizations(num_scenarios: int) -> None:
    """Run multiple SimReady randomization scenarios and capture the results."""
    print(f"[SDG] Initializing scene...")
    omni.usd.get_context().new_stage()
    stage = omni.usd.get_context().get_stage()

    # Initialize randomization
    rng = np.random.default_rng(34)
    rep.set_global_seed(34)

    # Data capture will happen manually using step()
    rep.orchestrator.set_capture_on_play(False)
    SimulationManager.set_physics_dt(1.0 / 60.0)

    # Set DLSS to Quality mode (2) for best SDG results , options: 0 (Performance), 1 (Balanced), 2 (Quality), 3 (Auto)
    carb.settings.get_settings().set("rtx/post/dlss/execMode", 2)

    # Add lights to the scene
    print(f"[SDG] Setting up lighting...")
    rep.functional.create.xform(name="World")
    rep.functional.create.dome_light(intensity=500, parent="/World", name="DomeLight")
    rep.functional.create.distant_light(intensity=2500, parent="/World", name="DistantLight", rotation=(-75, 0, 0))

    # Simready explorer window needs to be created for the search to work
    enable_simready_explorer()

    # Search for the simready assets and wait until the task is complete
    search_task = asyncio.ensure_future(search_assets_async())
    while not search_task.done():
        simulation_app.update()
    tables, dishes, items = search_task.result()

    # Create the writer and the render product for capturing the scene
    output_dir = os.path.join(os.getcwd(), "_out_simready_assets")
    backend = rep.backends.get("DiskBackend")
    backend.initialize(output_dir=output_dir)
    writer = rep.writers.get("BasicWriter")
    print(f"[SDG] Initializing writer, output directory: {output_dir}...")
    writer.initialize(backend=backend, rgb=True)

    # Create camera and render product (disabled by default, enabled only when capturing)
    print(f"[SDG] Creating camera and render product...")
    camera_prim = rep.functional.create.camera(position=(5, 5, 5), look_at=(0, 0, 0), parent="/World", name="Camera")
    rp = rep.create.render_product(camera_prim, (512, 512))
    rp.hydra_texture.set_updates_enabled(False)
    writer.attach(rp)

    # Generate randomized scenarios
    for i in range(num_scenarios):
        print(f"[SDG] Scenario {i + 1}/{num_scenarios}")
        run_simready_randomization(
            stage=stage, camera_prim=camera_prim, render_product=rp, tables=tables, dishes=dishes, items=items, rng=rng
        )

    # Finalize and cleanup
    print("[SDG] Wait for the data to be written and cleanup render products...")
    rep.orchestrator.wait_until_complete()
    writer.detach()
    rp.destroy()


print(f"[SDG] Starting SDG pipeline with {num_scenarios} scenarios...")
run_simready_randomizations(num_scenarios)

# <start-simready-assets-sdg-test>
import sys

from isaacsim.core.utils.extensions import enable_extension

enable_extension("isaacsim.test.utils")
from isaacsim.test.utils.file_validation import validate_folder_contents

test_parser = argparse.ArgumentParser()
test_parser.add_argument(
    "--test",
    action="store_true",
    help="Validate captured output files against expected counts and exit.",
)
test_args, _ = test_parser.parse_known_args()

if test_args.test:
    # BasicWriter rgb-only writes 1 png per scenario.
    out_dir = os.path.join(os.getcwd(), "_out_simready_assets")
    ok = validate_folder_contents(
        path=out_dir,
        recursive=True,
        expected_counts={"png": num_scenarios},
        fail_on_empty_files=True,
    )
    if not ok:
        print(f"[SDG][Test][FAIL] Output validation failed for {out_dir}")
        sys.exit(1)
    print(f"[SDG][Test][PASS] Output validation succeeded for {out_dir}")
# <end-simready-assets-sdg-test>

simulation_app.close()
