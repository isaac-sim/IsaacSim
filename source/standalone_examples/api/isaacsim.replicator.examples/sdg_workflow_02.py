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

"""
Generate palletized box stacks across randomized scenes.
Each scene chooses an environment, scatters pallets, builds box stacks, and captures camera views.
"""

import math
import os

from isaacsim import SimulationApp

simulation_app = SimulationApp(launch_config={"headless": False})

import carb
import carb.settings
import isaacsim.core.experimental.utils.bounds as bounds_utils
import omni.replicator.core as rep
import omni.usd
from isaacsim.storage.native import get_assets_root_path
from omni.replicator.core.scripts.functional import utils as rep_utils

DEFAULT_ENV_URLS = [
    "/Isaac/Environments/Simple_Warehouse/warehouse.usd",
    None,
]
PALLET_URL = "/Isaac/Environments/Simple_Warehouse/Props/SM_PaletteA_01.usd"
BOX_URLS = [
    "/Isaac/Environments/Simple_Warehouse/Props/SM_CardBoxC_01.usd",
    "/Isaac/Environments/Simple_Warehouse/Props/SM_CardBoxD_01.usd",
]

TOTAL_CAPTURES = 5
CAPTURES_PER_SCENE = 3
RESOLUTION = (1280, 720)
RT_SUBFRAMES = 8
PALLET_COUNT_RANGE = range(2, 6)
STACK_COUNT_RANGE = range(1, 4)
BOXES_PER_STACK_RANGE = range(1, 6)
STACK_SCATTER_AREA_SCALE = 0.9


def create_pallets_on_floor(scene_scope_path, assets_root_path, rng):
    pallet_count = int(rng.generator.choice(PALLET_COUNT_RANGE))
    # Use a hidden floor plane as a sampling surface so scattered pallets avoid each other.
    floor_plane = rep.functional.create.plane(
        parent=scene_scope_path,
        name="PalletScatterPlane",
        position=(0.0, 0.0, 0.0),
        scale=(5.0, 5.0, 1.0),
        visible=False,
    )

    pallets = []
    for i in range(pallet_count):
        pallets.append(
            rep.functional.create.reference(
                usd_path=assets_root_path + PALLET_URL,
                parent=scene_scope_path,
                name=f"Pallet_{i}",
                position=(i * 2.0, 0.0, 1.0),
                rotation=(0.0, 0.0, float(rng.generator.choice([0.0, 90.0, 180.0, 270.0]))),
                semantics={"class": "pallet"},
            )
        )

    simulation_app.update()
    rep.functional.randomizer.scatter_2d(pallets, floor_plane, check_for_collisions=True, rng=rng)
    simulation_app.update()

    bbox_cache = bounds_utils.create_bbox_cache()
    for pallet in pallets:
        aabb = bounds_utils.compute_aabb(pallet, bbox_cache=bbox_cache, include_children=True)
        origin = rep_utils.get_world_position(pallet)
        origin_to_bottom = float(origin[2] - aabb[2])
        # Move the pallet origin so the measured bottom sits on the floor.
        rep.functional.modify.pose(
            pallet,
            position_value=(float(origin[0]), float(origin[1]), origin_to_bottom),
            write_to_usd=True,
        )

    simulation_app.update()
    return pallets


def create_stacks_on_pallet(scene_scope_path, pallet_index, pallet, assets_root_path, rng):
    bbox_cache = bounds_utils.create_bbox_cache()
    pallet_bounds = bounds_utils.compute_aabb(pallet, bbox_cache=bbox_cache, include_children=True)
    pallet_origin = rep_utils.get_world_position(pallet)
    stack_count = int(rng.generator.choice(STACK_COUNT_RANGE))
    pallet_top_z = float(pallet_bounds[5])
    pallet_center_x = float(pallet_origin[0])
    pallet_center_y = float(pallet_origin[1])
    pallet_size_x = float(pallet_bounds[3] - pallet_bounds[0])
    pallet_size_y = float(pallet_bounds[4] - pallet_bounds[1])

    base_boxes = []
    stack_data = []
    for i in range(stack_count):
        box_url = BOX_URLS[int(rng.generator.integers(0, len(BOX_URLS)))]
        box = rep.functional.create.reference(
            usd_path=assets_root_path + box_url,
            parent=scene_scope_path,
            name=f"Pallet_{pallet_index}_BoxStack_{i}_Base",
            position=(i * 2.0, 0.0, pallet_top_z + 1.0),
            semantics={"class": "cardbox"},
        )
        base_boxes.append(box)
        stack_data.append(
            {
                "box": box,
                "url": assets_root_path + box_url,
                "height_count": int(rng.generator.choice(BOXES_PER_STACK_RANGE)),
            }
        )

    simulation_app.update()
    bbox_cache = bounds_utils.create_bbox_cache()
    max_box_height = 0.0
    for stack in stack_data:
        aabb = bounds_utils.compute_aabb(stack["box"], bbox_cache=bbox_cache, include_children=True)
        origin = rep_utils.get_world_position(stack["box"])
        stack["size"] = (
            float(aabb[3] - aabb[0]),
            float(aabb[4] - aabb[1]),
            float(aabb[5] - aabb[2]),
        )
        # Referenced box assets use an origin near the bottom, so keep this offset when stacking.
        stack["origin_to_bottom"] = float(origin[2] - aabb[2])
        max_box_height = max(max_box_height, stack["size"][2])

    # Scatter stack bases on a hidden plane smaller than the pallet top.
    scatter_plane = rep.functional.create.plane(
        parent=scene_scope_path,
        name=f"Pallet_{pallet_index}_StackScatterPlane",
        position=(pallet_center_x, pallet_center_y, pallet_top_z),
        scale=(pallet_size_x * STACK_SCATTER_AREA_SCALE, pallet_size_y * STACK_SCATTER_AREA_SCALE, 1.0),
        visible=False,
    )

    while True:
        try:
            rep.functional.randomizer.scatter_2d(
                base_boxes,
                scatter_plane,
                offset=max_box_height * 0.5,
                check_for_collisions=True,
                rng=rng,
            )
            break
        except ValueError:
            if len(base_boxes) <= 1:
                raise
            stack_data.pop()
            removed_box = base_boxes.pop()
            omni.usd.get_context().get_stage().RemovePrim(removed_box.GetPath())
            print(
                f"[SDG] Warning: could not scatter {len(base_boxes) + 1} stacks on pallet {pallet_index}; "
                f"retrying with {len(base_boxes)}."
            )
            simulation_app.update()
    simulation_app.update()

    all_boxes = list(base_boxes)
    for stack_idx, stack in enumerate(stack_data):
        _, _, box_height = stack["size"]
        origin_to_bottom = stack["origin_to_bottom"]
        origin = rep_utils.get_world_position(stack["box"])
        origin_x = float(origin[0])
        origin_y = float(origin[1])

        rep.functional.modify.pose(
            stack["box"],
            position_value=(origin_x, origin_y, pallet_top_z + origin_to_bottom),
            write_to_usd=True,
        )

        for level in range(1, stack["height_count"]):
            all_boxes.append(
                rep.functional.create.reference(
                    usd_path=stack["url"],
                    parent=scene_scope_path,
                    name=f"Pallet_{pallet_index}_BoxStack_{stack_idx}_{level}",
                    position=(origin_x, origin_y, pallet_top_z + origin_to_bottom + box_height * level),
                    semantics={"class": "cardbox"},
                )
            )

    return all_boxes


def randomize_camera(camera, pallet, rng):
    bbox_cache = bounds_utils.create_bbox_cache()
    pallet_bounds = bounds_utils.compute_aabb(pallet, bbox_cache=bbox_cache, include_children=True)
    pallet_origin = rep_utils.get_world_position(pallet)
    target = (
        float(pallet_origin[0]),
        float(pallet_origin[1]),
        float(pallet_bounds[5] + 0.5),
    )
    theta = float(rng.generator.uniform(0.0, 2.0 * math.pi))
    radius = float(rng.generator.uniform(2.4, 3.2))
    height = float(rng.generator.uniform(1.4, 2.2))
    rep.functional.modify.pose(
        camera,
        position_value=(
            target[0] + radius * math.cos(theta),
            target[1] + radius * math.sin(theta),
            target[2] + height,
        ),
        look_at_value=target,
        look_at_up_axis=(0, 0, 1),
        write_to_usd=True,
    )


def run_workflow():
    assets_root_path = get_assets_root_path()
    if assets_root_path is None:
        carb.log_error("[SDG] Could not resolve assets root path; aborting.")
        return

    # Create a clean stage for this tutorial workflow.
    omni.usd.get_context().new_stage()
    stage = omni.usd.get_context().get_stage()
    if stage is None:
        carb.log_error("[SDG] Could not create a new stage; aborting.")
        return

    # Disable automatic capture so only explicit `step()` calls write frames.
    rep.orchestrator.set_capture_on_play(False)

    # Set DLSS to Quality mode to reduce low-resolution SDG rendering artifacts.
    carb.settings.get_settings().set("rtx/post/dlss/execMode", 2)

    # Seed the randomizer so re-running the script produces the same scene.
    rng = rep.rng.ReplicatorRNG(seed=42)

    rep.functional.create.xform(name="World")
    rep.functional.create.xform(parent="/World", name="SDG")

    # Keep generated SDG content separate from the randomized environment.
    rep.functional.create.dome_light(intensity=500, parent="/World/SDG", name="DomeLight")

    # Use one camera and move it to each pallet before capturing.
    cam = rep.functional.create.camera(position=(3, 3, 3), look_at=(0, 0, 0), parent="/World/SDG", name="Camera")
    rp = rep.create.render_product(cam, RESOLUTION, name="rp_workflow_02")
    rp.hydra_texture.set_updates_enabled(False)

    # Attach a `BasicWriter` to save RGB and colorized semantic labels.
    backend = rep.backends.get("DiskBackend")
    out_dir = os.path.join(os.getcwd(), "_out_workflow_02")
    backend.initialize(output_dir=out_dir)
    print(f"[SDG] Output directory: {out_dir}")
    writer = rep.writers.get("BasicWriter")
    writer.initialize(
        backend=backend,
        rgb=True,
        semantic_segmentation=True,
        colorize_semantic_segmentation=True,
    )
    writer.attach(rp)

    environment_scope_path = "/World/Environment"
    capture_count = 0
    randomization_count = 0
    prev_scene_scope_path = None

    while capture_count < TOTAL_CAPTURES:
        randomization_count += 1
        print(f"[SDG] Randomization {randomization_count}")

        # Use a unique scene scope per randomization so Replicator's scatter mesh cache stays fresh.
        scene_scope_name = f"Scene_{randomization_count}"
        scene_scope_path = f"/World/SDG/{scene_scope_name}"

        if stage.GetPrimAtPath(environment_scope_path).IsValid():
            stage.RemovePrim(environment_scope_path)
        if prev_scene_scope_path is not None and stage.GetPrimAtPath(prev_scene_scope_path).IsValid():
            stage.RemovePrim(prev_scene_scope_path)
        simulation_app.update()

        rep.functional.create.scope(name="Environment", parent="/World")
        # Pick an environment; None means use a generated ground plane.
        env_url = DEFAULT_ENV_URLS[int(rng.generator.integers(0, len(DEFAULT_ENV_URLS)))]
        if env_url is None:
            ground = rep.functional.create.plane(
                parent=environment_scope_path,
                name="GroundPlane",
                scale=(100, 100, 1),
            )
            rep.functional.physics.apply_collider(ground)
        else:
            rep.functional.create.reference(
                usd_path=assets_root_path + env_url,
                parent=environment_scope_path,
                name="Scene",
            )

        rep.functional.create.scope(name=scene_scope_name, parent="/World/SDG")
        prev_scene_scope_path = scene_scope_path

        # Choose the pallet count, scatter them, shuffle the order, then build stacks.
        pallets = create_pallets_on_floor(scene_scope_path, assets_root_path, rng)
        pallet_order = list(range(len(pallets)))
        rng.generator.shuffle(pallet_order)
        pallets = [pallets[i] for i in pallet_order]
        for pallet_idx, pallet in enumerate(pallets):
            create_stacks_on_pallet(scene_scope_path, pallet_idx, pallet, assets_root_path, rng)

        captures_this_scene = min(CAPTURES_PER_SCENE, TOTAL_CAPTURES - capture_count)
        for capture_idx in range(captures_this_scene):
            pallet = pallets[capture_idx % len(pallets)]
            randomize_camera(cam, pallet, rng)

            capture_count += 1
            print(f"[SDG] Capture {capture_count}/{TOTAL_CAPTURES}")
            rp.hydra_texture.set_updates_enabled(True)
            rep.orchestrator.step(rt_subframes=RT_SUBFRAMES)
            rp.hydra_texture.set_updates_enabled(False)

    # Cleanup.
    rep.orchestrator.wait_until_complete()
    writer.detach()
    rp.destroy()


run_workflow()

# <start-sdg-workflow-02-test>
import argparse
import sys

from isaacsim.core.utils.extensions import enable_extension

enable_extension("isaacsim.test.utils")
from isaacsim.test.utils.file_validation import validate_folder_contents

parser = argparse.ArgumentParser()
parser.add_argument(
    "--test",
    action="store_true",
    help="Validate captured output files against expected counts and exit.",
)
args, _ = parser.parse_known_args()

if args.test:
    # BasicWriter with rgb + colorized semantic_segmentation writes 2 png + 1 json per capture.
    expected_json_count = TOTAL_CAPTURES
    expected_png_count = TOTAL_CAPTURES * 2
    out_dir = os.path.join(os.getcwd(), "_out_workflow_02")
    ok = validate_folder_contents(
        path=out_dir,
        recursive=True,
        expected_counts={"png": expected_png_count, "json": expected_json_count},
        fail_on_empty_files=True,
    )
    if not ok:
        print(f"[SDG][Test][FAIL] Output validation failed for {out_dir}")
        sys.exit(1)
    print(f"[SDG][Test][PASS] Output validation succeeded for {out_dir}")
# <end-sdg-workflow-02-test>

simulation_app.close()
