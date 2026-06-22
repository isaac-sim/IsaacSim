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

"""Verify deformable SDG captures triggered by simulated asset drops against golden outputs."""

import json
import tempfile

import carb.settings
import omni.kit
import omni.usd
from isaacsim.core.simulation_manager import SimulationManager
from isaacsim.test.utils.file_validation import validate_folder_contents
from isaacsim.test.utils.image_comparison import compare_images_in_directories


class TestSDGDeformables(omni.kit.test.AsyncTestCase):
    """Runs the deformable-object drop workflow and validates RGB and semantic label data."""

    RGB_MEAN_DIFF_TOLERANCE = 5

    async def setUp(self) -> None:
        """Create a clean stage and save render and physics-device settings changed by the test."""
        await omni.kit.app.get_app().next_update_async()
        omni.usd.get_context().new_stage()
        await omni.kit.app.get_app().next_update_async()
        self.original_dlss_exec_mode = carb.settings.get_settings().get("rtx/post/dlss/execMode")
        self.original_physics_sim_device = SimulationManager.get_device()

    async def tearDown(self) -> None:
        """Close the stage, wait for pending loads, and restore render and physics-device settings."""
        omni.usd.get_context().close_stage()
        await omni.kit.app.get_app().next_update_async()
        # In some cases the test will end before the asset is loaded, in this case wait for assets to load
        while omni.usd.get_context().get_stage_loading_status()[2] > 0:
            await omni.kit.app.get_app().next_update_async()
        carb.settings.get_settings().set("rtx/post/dlss/execMode", self.original_dlss_exec_mode)
        # Make sure to reset the physics sim device to the original state for the following tests
        SimulationManager.set_physics_sim_device(self.original_physics_sim_device)

    async def test_sdg_snippet_deformables(self) -> None:
        """Drop deformable YCB assets into a crate and capture frames when each mesh crosses a height."""
        import os
        import random

        import carb.settings
        import omni.kit.app
        import omni.replicator.core as rep
        import omni.timeline
        import omni.usd
        from isaacsim.core.experimental.materials import VolumeDeformableMaterial
        from isaacsim.core.experimental.prims import DeformablePrim
        from isaacsim.core.simulation_manager import SimulationManager
        from isaacsim.storage.native import get_assets_root_path_async
        from pxr import Gf, Sdf, Usd, UsdShade

        out_dir = tempfile.mkdtemp(prefix="test_deformable_drop_")
        print(f"Output directory: {out_dir}")

        TRIGGER_HEIGHT = 0.15  # Capture when lowest vertex falls below this (m)
        BASE_DROP_HEIGHT = 0.2  # Starting height for first asset (m)
        HEIGHT_STEP = 0.15  # Height increment between assets (m)
        SPAWN_XY_JITTER = 0.07  # Random horizontal offset +/- (m)
        RNG_SEED = 12  # Reproducible randomization seed
        MAX_STEPS = 200  # Maximum simulation steps
        CRATE_USD = "/Isaac/Props/PackingTable/props/SM_Crate_A08_Blue_01/SM_Crate_A08_Blue_01.usd"

        # (label, count, usd_path, youngs_modulus_Pa [higher=stiffer], poissons_ratio [0=compressible, 0.5=incompressible])
        ASSETS_CONFIG = [
            ("banana", 6, "/Isaac/Props/YCB/Axis_Aligned/011_banana.usd", 500_000, 0.45),
            ("large_marker", 5, "/Isaac/Props/YCB/Axis_Aligned/040_large_marker.usd", 9_000_000, 0.5),
        ]

        async def run_example_async(assets_config: list[tuple[str, int, str, float, float]]) -> None:
            await omni.usd.get_context().new_stage_async()
            assets_root_path = await get_assets_root_path_async()
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
            backend = rep.backends.get("DiskBackend")
            backend.initialize(output_dir=out_dir)
            writer = rep.writers.get("BasicWriter")
            writer.initialize(
                backend=backend, rgb=True, semantic_segmentation=True, colorize_semantic_segmentation=True
            )
            writer.attach(render_product)

            # GPU physics required for deformable tensor API
            SimulationManager.set_physics_sim_device("cuda")
            SimulationManager.initialize_physics()

            # Keep track of which assets have been triggered
            triggered = [False] * total

            # Start the simulation
            print(f"[SDG] Starting simulation")
            timeline = omni.timeline.get_timeline_interface()
            timeline.play()

            # Wrap deformables for tensor API access (requires active simulation, no re-cooking)
            deformables = []
            for i in range(total):
                deformables.append(DeformablePrim(all_prims[i].GetPath().pathString))

            for _ in range(MAX_STEPS):
                # Advance the app which will advance the timeline (and implicitly the simulation)
                await omni.kit.app.get_app().next_update_async()

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
                    await rep.orchestrator.step_async(delta_time=0.0, pause_timeline=False, rt_subframes=16)
                if newly_triggered:
                    render_product.hydra_texture.set_updates_enabled(False)

                # If all assets have been triggered, stop the simulation
                if all(triggered):
                    break

            # Pause the simulation and clean up resources
            print(f"[SDG] Simulation complete. {len(triggered)} frames saved to {out_dir}")
            timeline.pause()
            await rep.orchestrator.wait_until_complete_async()
            writer.detach()
            render_product.destroy()

        # asyncio.ensure_future(run_example_async(ASSETS_CONFIG))

        # Test setup
        test_assets_config = [
            ("banana", 2, "/Isaac/Props/YCB/Axis_Aligned/011_banana.usd", 500_000, 0.45),
            ("large_marker", 2, "/Isaac/Props/YCB/Axis_Aligned/040_large_marker.usd", 9_000_000, 0.5),
        ]
        await run_example_async(test_assets_config)
        num_assets = sum(count for _, count, _, _, _ in test_assets_config)
        expected_pngs = num_assets * 2  # 2 assets * 2 (rgb+segmentation annotators)
        expected_json = num_assets * 1  # 2 assets * 1 (segmentation annotator)
        golden_dir = os.path.join(os.path.dirname(os.path.realpath(__file__)), "data", "golden", "_out_deformable_drop")
        all_data_written = validate_folder_contents(
            path=out_dir,
            recursive=True,
            expected_counts={"png": expected_pngs, "json": expected_json},
        )
        self.assertTrue(all_data_written, f"Output directory contents validation failed for {out_dir}")

        result = compare_images_in_directories(
            golden_dir=golden_dir,
            test_dir=out_dir,
            path_pattern=r"rgb_.*\.png$",
            allclose_rtol=None,
            allclose_atol=None,
            mean_tolerance=self.RGB_MEAN_DIFF_TOLERANCE,
            print_all_stats=False,
        )
        self.assertTrue(result["all_passed"], f"RGB image comparison failed for output directory: {out_dir}")

        golden_label_files = sorted(
            file_name
            for file_name in os.listdir(golden_dir)
            if file_name.startswith("semantic_segmentation_labels_") and file_name.endswith(".json")
        )
        out_label_files = sorted(
            file_name
            for file_name in os.listdir(out_dir)
            if file_name.startswith("semantic_segmentation_labels_") and file_name.endswith(".json")
        )
        self.assertEqual(
            out_label_files, golden_label_files, f"Semantic segmentation label files mismatch for {out_dir}"
        )

        for file_name in golden_label_files:
            with open(os.path.join(golden_dir, file_name), encoding="utf-8") as golden_file:
                golden_labels = json.load(golden_file)
            with open(os.path.join(out_dir, file_name), encoding="utf-8") as out_file:
                out_labels = json.load(out_file)

            golden_label_values = sorted(json.dumps(label, sort_keys=True) for label in golden_labels.values())
            out_label_values = sorted(json.dumps(label, sort_keys=True) for label in out_labels.values())

            self.assertEqual(
                out_label_values,
                golden_label_values,
                f"Semantic segmentation label comparison failed for {os.path.join(out_dir, file_name)}",
            )
