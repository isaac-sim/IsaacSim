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

"""Basic SDG workflow with scene creation, asset placement, randomization, and data capture.

Boxes are randomized and simulated with physics before each capture.
"""

import math
import os
from typing import Any

from isaacsim import SimulationApp

simulation_app = SimulationApp(launch_config={"headless": False})

import carb
import carb.settings
import omni.replicator.core as rep
import omni.timeline
import omni.usd
from isaacsim.storage.native import get_assets_root_path
from pxr import Usd, UsdGeom, UsdPhysics

NUM_CAPTURES = 5
RESOLUTION = (1280, 720)
RT_SUBFRAMES = 8
NUM_DROP_BOXES = 10
NUM_SIMULATION_FRAMES = 75
NUM_PRIM_DISTRACTORS = 5
ENV_URL = "/Isaac/Environments/Grid/default_environment.usd"


def randomize_distractors(prims: list[Any], rng: Any) -> None:
    """Randomize small distractors on a loose ring around the pallet."""
    count = len(prims)
    angles = rng.generator.uniform(0.0, 2.0 * math.pi, count)
    radii = rng.generator.uniform(0.9, 1.4, count)
    heights = rng.generator.uniform(0.15, 0.45, count)
    positions = [
        (float(radius * math.cos(angle)), float(radius * math.sin(angle)), float(height))
        for angle, radius, height in zip(angles, radii, heights)
    ]
    rotations = rng.generator.uniform((0.0, 0.0, 0.0), (45.0, 45.0, 360.0), size=(count, 3)).tolist()
    scales = rng.generator.uniform(0.1, 0.2, count).tolist()
    rep.functional.modify.pose(
        prims,
        position_value=positions,
        rotation_value=rotations,
        scale_value=scales,
    )
    rep.functional.randomizer.display_color(prims, rng=rng)


def randomize_dome_light(dome_light: Any, texture_urls: list[str], rng: Any) -> None:
    """Randomize the dome light texture and intensity."""
    texture_url = texture_urls[int(rng.generator.integers(0, len(texture_urls)))]
    intensity = float(rng.generator.uniform(500.0, 900.0))
    rep.functional.modify.attribute(dome_light, "inputs:texture:file", texture_url)
    rep.functional.modify.attribute(dome_light, "inputs:intensity", intensity)


def randomize_pallet(pallet: Any, materials: list[Any], rng: Any) -> None:
    """Randomize the pallet material."""
    material = materials[int(rng.generator.integers(0, len(materials)))]
    rep.functional.modify.material(pallet, material)


def randomize_camera(camera: Any, look_at: Any, rng: Any) -> None:
    """Randomize the camera pose while keeping it aimed at the target."""
    theta = float(rng.generator.uniform(0.0, 2.0 * math.pi))
    radius = float(rng.generator.uniform(2.6, 3.4))
    position = (
        radius * math.cos(theta),
        radius * math.sin(theta),
        float(rng.generator.uniform(1.4, 2.2)),
    )
    rep.functional.modify.pose(
        camera,
        position_value=position,
        look_at_value=look_at,
        look_at_up_axis=(0, 0, 1),
        write_to_usd=True,
    )


def randomize_boxes(boxes: list[Any], start_height: float, rng: Any) -> None:
    """Randomize box poses before dropping them onto the pallet."""
    for i, box in enumerate(boxes):
        lateral_range = 0.4
        height = start_height + 0.2 * i
        tilt_range = 15.0
        rep.functional.modify.pose(
            box,
            position_value=(
                float(rng.generator.uniform(-lateral_range, lateral_range)),
                float(rng.generator.uniform(-lateral_range, lateral_range)),
                height,
            ),
            rotation_value=(
                float(rng.generator.uniform(0.0, tilt_range)),
                float(rng.generator.uniform(0.0, tilt_range)),
                float(rng.generator.uniform(0.0, 360.0)),
            ),
            write_to_usd=True,
        )


def run_workflow() -> None:
    """Run the pallet drop SDG workflow."""
    assets_root_path = get_assets_root_path()
    if assets_root_path is None:
        carb.log_error("[SDG] Could not resolve assets root path; aborting.")
        return

    # Load stage.
    env_path = assets_root_path + ENV_URL
    omni.usd.get_context().open_stage(env_path)
    stage = omni.usd.get_context().get_stage()
    if stage is None:
        carb.log_error(f"[SDG] Failed to open stage: '{env_path}', exiting.")
        return

    # Disable automatic capture so only explicit `step()` calls write frames.
    rep.orchestrator.set_capture_on_play(False)

    # Set DLSS to Quality mode to reduce low-resolution SDG rendering artifacts.
    carb.settings.get_settings().set("rtx/post/dlss/execMode", 2)

    # Seed the functional randomizer so re-running the script is reproducible.
    rng = rep.rng.ReplicatorRNG(seed=42)
    timeline = omni.timeline.get_timeline_interface()

    # Create a dome light which will be randomized by texture and brightness.
    rep.functional.create.xform(name="SDG")
    rep.functional.create.scope(name="Lights", parent="/SDG")
    dome_texture_urls = [
        assets_root_path + "/NVIDIA/Assets/Skies/Indoor/autoshop_01_4k.hdr",
        assets_root_path + "/NVIDIA/Assets/Skies/Indoor/carpentry_shop_01_4k.hdr",
        assets_root_path + "/NVIDIA/Assets/Skies/Indoor/wooden_lounge_4k.hdr",
    ]
    dome_light = rep.functional.create.dome_light(
        texture=dome_texture_urls[0],
        intensity=700,
        parent="/SDG/Lights",
        name="DomeLight",
    )
    randomize_dome_light(dome_light, dome_texture_urls, rng=rng)

    # Create a pallet with collision to drop boxes onto, its materials will be randomized.
    rep.functional.create.scope(name="Assets", parent="/SDG")
    rep.functional.create.scope(name="Materials", parent="/SDG")
    pallet_url = assets_root_path + "/Isaac/Environments/Simple_Warehouse/Props/SM_PaletteA_01.usd"
    pallet = rep.functional.create.reference(
        usd_path=pallet_url,
        parent="/SDG/Assets",
        name="Pallet",
        semantics={"class": "pallet"},
    )
    rep.functional.physics.apply_rigid_body(pallet, with_collider=True, kinematicEnabled=True)
    pallet_materials = []
    for i in range(5):
        color = rng.generator.uniform((0.45, 0.3, 0.15), (0.95, 0.85, 0.55), size=3)
        pallet_materials.append(
            rep.functional.create.material(
                mdl="OmniPBR.mdl",
                diffuse_color_constant=tuple(float(channel) for channel in color),
                reflection_roughness_constant=float(rng.generator.uniform(0.45, 0.95)),
                metallic_constant=float(rng.generator.uniform(0.0, 0.05)),
                name=f"PalletMaterial_{i}",
                parent="/SDG/Materials",
            )
        )
    randomize_pallet(pallet, pallet_materials, rng=rng)

    # Create primitive distractors to randomly place around the scene.
    rep.functional.create.scope(name="Distractors", parent="/SDG/Assets")
    cube_distractors = rep.functional.create_batch.cube(
        count=NUM_PRIM_DISTRACTORS,
        parent="/SDG/Assets/Distractors",
        name="DistractorCube",
        semantics={"class": "distractor"},
    )
    sphere_distractors = rep.functional.create_batch.sphere(
        count=NUM_PRIM_DISTRACTORS,
        parent="/SDG/Assets/Distractors",
        name="DistractorSphere",
        semantics={"class": "distractor"},
    )
    cylinder_distractors = rep.functional.create_batch.cylinder(
        count=NUM_PRIM_DISTRACTORS,
        parent="/SDG/Assets/Distractors",
        name="DistractorCylinder",
        semantics={"class": "distractor"},
    )
    cone_distractors = rep.functional.create_batch.cone(
        count=NUM_PRIM_DISTRACTORS,
        parent="/SDG/Assets/Distractors",
        name="DistractorCone",
        semantics={"class": "distractor"},
    )
    distractors = cube_distractors + sphere_distractors + cylinder_distractors + cone_distractors
    randomize_distractors(distractors, rng=rng)

    # Create boxes with rigid body dynamics to randomly drop onto the pallet.
    cardbox_url = assets_root_path + "/Isaac/Environments/Simple_Warehouse/Props/SM_CardBoxD_04.usd"
    boxes = []
    for i in range(NUM_DROP_BOXES):
        box = rep.functional.create.reference(
            usd_path=cardbox_url,
            parent="/SDG/Assets",
            name=f"Box_{i}",
            semantics={"class": "cardbox"},
        )
        # The referenced box has mesh children, so set a simple box collider on each mesh.
        for child_prim in Usd.PrimRange(box):
            if child_prim.IsA(UsdGeom.Mesh):
                mesh_collision_api = UsdPhysics.MeshCollisionAPI.Apply(child_prim)
                mesh_collision_api.CreateApproximationAttr().Set("boundingCube")
                rep.functional.physics.apply_rigid_body(child_prim, with_collider=True, approximation="boundingCube")
        rep.functional.physics.apply_rigid_body(box)
        boxes.append(box)
    randomize_boxes(boxes, start_height=0.3, rng=rng)

    # Drop the boxes.
    timeline.play()
    for _ in range(NUM_SIMULATION_FRAMES):
        simulation_app.update()
    timeline.pause()

    # Setup SDG.
    rep.functional.create.scope(name="Cameras", parent="/SDG")
    cam = rep.functional.create.camera(
        position=(3.0, 3.0, 2.0), look_at=(0, 0, 0.4), parent="/SDG/Cameras", name="Camera"
    )
    rp = rep.create.render_product(cam, RESOLUTION, name="rp_workflow_01")
    # Disable render products by default and only enable them at capture time.
    rp.hydra_texture.set_updates_enabled(False)

    # Attach a `BasicWriter` to save common annotations from the same camera view.
    backend = rep.backends.get("DiskBackend")
    out_dir = os.path.join(os.getcwd(), "_out_workflow_01")
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

    for i in range(NUM_CAPTURES):
        print(f"[SDG] Capture {i + 1}/{NUM_CAPTURES}")

        # Run the randomizers on the scene.
        randomize_dome_light(dome_light, dome_texture_urls, rng=rng)
        randomize_distractors(distractors, rng=rng)
        randomize_pallet(pallet, pallet_materials, rng=rng)

        # Re-drop one box so each capture has a slightly different physical arrangement.
        box = boxes[int(rng.generator.integers(0, len(boxes)))]
        randomize_boxes([box], start_height=1.2, rng=rng)
        timeline.play()
        for _ in range(NUM_SIMULATION_FRAMES):
            simulation_app.update()
        timeline.pause()

        # Sample a new camera position on a small orbit while looking at the pallet.
        randomize_camera(cam, pallet, rng=rng)

        # Enable rendering only for the capture step to avoid extra GPU work.
        rp.hydra_texture.set_updates_enabled(True)
        rep.orchestrator.step(delta_time=0.0, rt_subframes=RT_SUBFRAMES)
        rp.hydra_texture.set_updates_enabled(False)

    # Cleanup.
    rep.orchestrator.wait_until_complete()
    writer.detach()
    rp.destroy()


run_workflow()

# <start-sdg-workflow-01-test>
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
    expected_json_count = NUM_CAPTURES
    expected_png_count = NUM_CAPTURES * 2
    out_dir = os.path.join(os.getcwd(), "_out_workflow_01")
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
# <end-sdg-workflow-01-test>

simulation_app.close()
