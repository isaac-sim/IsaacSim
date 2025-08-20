# SPDX-FileCopyrightText: Copyright (c) 2024-2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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

import os
import unittest

import omni.kit
import omni.usd

from .common import validate_folder_contents


class TestSDGUR10Palletizing(omni.kit.test.AsyncTestCase):
    async def setUp(self):
        await omni.kit.app.get_app().next_update_async()
        await omni.usd.get_context().new_stage_async()
        await omni.kit.app.get_app().next_update_async()

    async def tearDown(self):
        omni.usd.get_context().close_stage()
        await omni.kit.app.get_app().next_update_async()
        # In some cases the test will end before the asset is loaded, in this case wait for assets to load
        while omni.usd.get_context().get_stage_loading_status()[2] > 0:
            await omni.kit.app.get_app().next_update_async()

    @unittest.skipIf(os.getenv("ETM_ACTIVE"), "Skipped in ETM.")
    async def test_sdg_ur10_palletizing(self):
        import asyncio
        import json
        import os

        import carb.settings
        import numpy as np
        import omni
        import omni.kit.app
        import omni.kit.commands
        import omni.replicator.core as rep
        import omni.timeline
        import omni.usd
        from isaacsim.core.utils.bounds import create_bbox_cache
        from isaacsim.storage.native import get_assets_root_path
        from omni.physx import get_physx_scene_query_interface
        from PIL import Image
        from pxr import UsdShade

        class PalletizingSDGDemo:
            BINS_FOLDER_PATH = "/World/Ur10Table/bins"
            FLIP_HELPER_PATH = "/World/Ur10Table/pallet_holder"
            PALLET_PRIM_MESH_PATH = "/World/Ur10Table/pallet/Xform/Mesh_015"
            BIN_FLIP_SCENARIO_FRAMES = 4
            PALLET_SCENARIO_FRAMES = 16

            def __init__(self):
                # There are 36 bins in total
                self._bin_counter = 0
                self._num_captures = 36
                self._stage = None
                self._active_bin = None

                # Cleanup in case the user closes the stage
                self._stage_event_sub = None

                # Simulation state flags
                self._in_running_state = False
                self._bin_flip_scenario_done = False

                # Used to pause/resume the simulation
                self._timeline = None

                # Used to actively track the active bins surroundings (e.g., in contact with pallet)
                self._timeline_sub = None
                self._overlap_extent = None

                # SDG
                self._rep_camera = None
                self._output_dir = os.path.join(os.getcwd(), "_out_palletizing_sdg_demo")
                print(f"[PalletizingSDGDemo] Output directory: {self._output_dir}")

            def start(self, num_captures):
                self._num_captures = num_captures if 1 <= num_captures <= 36 else 36
                if self._init():
                    self._start()

            def is_running(self):
                return self._in_running_state

            def _init(self):
                self._stage = omni.usd.get_context().get_stage()
                self._active_bin = self._stage.GetPrimAtPath(f"{self.BINS_FOLDER_PATH}/bin_{self._bin_counter}")

                if not self._active_bin:
                    print("[PalletizingSDGDemo] Could not find bin, make sure the palletizing demo is loaded..")
                    return False

                bb_cache = create_bbox_cache()
                half_ext = bb_cache.ComputeLocalBound(self._active_bin).GetRange().GetSize() * 0.5
                self._overlap_extent = carb.Float3(half_ext[0], half_ext[1], half_ext[2] * 1.1)

                self._timeline = omni.timeline.get_timeline_interface()
                # Make sure the timeline can continueously run
                self._timeline.set_looping(True)
                if not self._timeline.is_playing():
                    print("[PalletizingSDGDemo] Please start the palletizing demo first..")
                    return False

                # Disable capture on play for replicator, data capture will be triggered manually
                rep.orchestrator.set_capture_on_play(False)

                # Clear any previously generated SDG graphs
                if self._stage.GetPrimAtPath("/Replicator"):
                    omni.kit.commands.execute("DeletePrimsCommand", paths=["/Replicator"])

                return True

            def _start(self):
                self._timeline_sub = self._timeline.get_timeline_event_stream().create_subscription_to_pop_by_type(
                    int(omni.timeline.TimelineEventType.CURRENT_TIME_TICKED), self._on_timeline_event
                )
                self._stage_event_sub = (
                    omni.usd.get_context()
                    .get_stage_event_stream()
                    .create_subscription_to_pop_by_type(
                        int(omni.usd.StageEventType.CLOSING), self._on_stage_closing_event
                    )
                )
                self._in_running_state = True
                print("[PalletizingSDGDemo] Starting the palletizing SDG demo..")

            def clear(self):
                if self._timeline_sub:
                    self._timeline_sub.unsubscribe()
                self._timeline_sub = None
                self._stage_event_sub = None
                self._in_running_state = False
                self._bin_counter = 0
                self._active_bin = None
                if self._stage.GetPrimAtPath("/Replicator"):
                    omni.kit.commands.execute("DeletePrimsCommand", paths=["/Replicator"])

            def _on_stage_closing_event(self, e: carb.events.IEvent):
                # Make sure the subscribers are unsubscribed for new stages
                self.clear()

            def _on_timeline_event(self, e: carb.events.IEvent):
                self._check_bin_overlaps()

            def _check_bin_overlaps(self):
                bin_pose = omni.usd.get_world_transform_matrix(self._active_bin)
                origin = bin_pose.ExtractTranslation()
                quat_gf = bin_pose.ExtractRotation().GetQuaternion()

                any_hit_flag = False
                hit_info = get_physx_scene_query_interface().overlap_box(
                    carb.Float3(self._overlap_extent),
                    carb.Float3(origin[0], origin[1], origin[2]),
                    carb.Float4(
                        quat_gf.GetImaginary()[0],
                        quat_gf.GetImaginary()[1],
                        quat_gf.GetImaginary()[2],
                        quat_gf.GetReal(),
                    ),
                    self._on_overlap_hit,
                    any_hit_flag,
                )

            def _on_overlap_hit(self, hit):
                if hit.rigid_body == self._active_bin.GetPrimPath():
                    return True  # Self hit, return True to continue the query

                # First contact with the flip helper
                if hit.rigid_body.startswith(self.FLIP_HELPER_PATH) and not self._bin_flip_scenario_done:
                    self._timeline.pause()
                    self._timeline_sub.unsubscribe()
                    self._timeline_sub = None
                    asyncio.ensure_future(self._run_bin_flip_scenario())
                    return False  # Relevant hit, return False to finish the hit query

                # Contact with the pallet or other bin on the pallet
                pallet_hit = hit.rigid_body.startswith(self.PALLET_PRIM_MESH_PATH)
                other_bin_hit = hit.rigid_body.startswith(f"{self.BINS_FOLDER_PATH}/bin_")
                if pallet_hit or other_bin_hit:
                    self._timeline.pause()
                    self._timeline_sub.unsubscribe()
                    self._timeline_sub = None
                    asyncio.ensure_future(self._run_pallet_scenario())
                    return False  # Relevant hit, return False to finish the hit query

                return True  # No relevant hit, return True to continue the query

            def _switch_to_pathtracing(self):
                carb.settings.get_settings().set("/rtx/rendermode", "PathTracing")
                carb.settings.get_settings().set("/rtx/pathtracing/spp", 32)
                carb.settings.get_settings().set("/rtx/pathtracing/totalSpp", 32)

            def _switch_to_raytracing(self):
                carb.settings.get_settings().set("/rtx/rendermode", "RayTracedLighting")
                # 0: Disabled, 1: TAA, 2: FXAA, 3: DLSS, 4:RTXAA
                carb.settings.get_settings().set("/rtx/post/aa/op", 3)

            async def _run_bin_flip_scenario(self):
                await omni.kit.app.get_app().next_update_async()
                print(f"[PalletizingSDGDemo] Running bin flip scenario for bin {self._bin_counter}..")

                # Util function to save rgb images to file
                def save_img(rgb_data, filename):
                    rgb_img = Image.fromarray(rgb_data, "RGBA")
                    rgb_img.save(filename + ".png")

                self._switch_to_pathtracing()
                self._create_bin_flip_graph()

                rgb_annot = rep.AnnotatorRegistry.get_annotator("rgb")
                is_annot = rep.AnnotatorRegistry.get_annotator("instance_segmentation", init_params={"colorize": True})
                rp = rep.create.render_product(self._rep_camera, (512, 512))
                rgb_annot.attach(rp)
                is_annot.attach(rp)
                out_dir = os.path.join(self._output_dir, f"annot_bin_{self._bin_counter}", "")
                os.makedirs(out_dir, exist_ok=True)

                for i in range(self.BIN_FLIP_SCENARIO_FRAMES):
                    await rep.orchestrator.step_async(delta_time=0.0)

                    rgb_data = rgb_annot.get_data()
                    rgb_filename = f"{out_dir}rgb_{i}"
                    save_img(rgb_data, rgb_filename)

                    is_data = is_annot.get_data()
                    is_filename = f"{out_dir}is_{i}"
                    is_img_data = is_data["data"]
                    height, width = is_img_data.shape[:2]
                    is_img_data = is_img_data.view(np.uint8).reshape(height, width, -1)
                    save_img(is_img_data, is_filename)
                    is_info = is_data["info"]
                    with open(f"{out_dir}is_info_{i}.json", "w") as f:
                        json.dump(is_info, f, indent=4)

                # Free up resources
                rgb_annot.detach()
                is_annot.detach()
                rp.destroy()

                # Make sure the backend finishes writing the data before clearing the generated SDG graph
                await rep.orchestrator.wait_until_complete_async()

                # Cleanup the generated SDG graph
                if self._stage.GetPrimAtPath("/Replicator"):
                    omni.kit.commands.execute("DeletePrimsCommand", paths=["/Replicator"])

                self._switch_to_raytracing()

                self._bin_flip_scenario_done = True
                self._timeline_sub = self._timeline.get_timeline_event_stream().create_subscription_to_pop_by_type(
                    int(omni.timeline.TimelineEventType.CURRENT_TIME_TICKED), self._on_timeline_event
                )
                self._timeline.play()

            def _create_bin_flip_graph(self):
                # Create new random lights using the color palette for the color attribute
                color_palette = [(1, 0, 0), (0, 1, 0), (0, 0, 1)]

                def randomize_bin_flip_lights():
                    lights = rep.create.light(
                        light_type="Sphere",
                        temperature=rep.distribution.normal(6500, 2000),
                        intensity=rep.distribution.normal(45000, 15000),
                        position=rep.distribution.uniform((0.25, 0.25, 0.5), (1, 1, 0.75)),
                        scale=rep.distribution.uniform(0.5, 0.8),
                        color=rep.distribution.choice(color_palette),
                        count=3,
                    )
                    return lights.node

                rep.randomizer.register(randomize_bin_flip_lights)

                # Move the camera to the given location sequences and look at the predefined location
                camera_positions = [(1.96, 0.72, -0.34), (1.48, 0.70, 0.90), (0.79, -0.86, 0.12), (-0.49, 1.47, 0.58)]
                self._rep_camera = rep.create.camera()
                with rep.trigger.on_frame():
                    rep.randomizer.randomize_bin_flip_lights()
                    with self._rep_camera:
                        rep.modify.pose(
                            position=rep.distribution.sequence(camera_positions), look_at=(0.78, 0.72, -0.1)
                        )

            async def _run_pallet_scenario(self):
                await omni.kit.app.get_app().next_update_async()
                print(f"[PalletizingSDGDemo] Running pallet scenario for bin {self._bin_counter}..")
                mesh_to_orig_mats = {}
                pallet_mesh = self._stage.GetPrimAtPath(self.PALLET_PRIM_MESH_PATH)
                pallet_orig_mat, _ = UsdShade.MaterialBindingAPI(pallet_mesh).ComputeBoundMaterial()
                mesh_to_orig_mats[pallet_mesh] = pallet_orig_mat
                for i in range(self._bin_counter + 1):
                    bin_mesh = self._stage.GetPrimAtPath(
                        f"{self.BINS_FOLDER_PATH}/bin_{i}/Visuals/FOF_Mesh_Magenta_Box"
                    )
                    bin_orig_mat, _ = UsdShade.MaterialBindingAPI(bin_mesh).ComputeBoundMaterial()
                    mesh_to_orig_mats[bin_mesh] = bin_orig_mat

                self._create_bin_and_pallet_graph()

                out_dir = os.path.join(self._output_dir, f"writer_bin_{self._bin_counter}", "")
                writer = rep.WriterRegistry.get("BasicWriter")
                writer.initialize(
                    output_dir=out_dir, rgb=True, instance_segmentation=True, colorize_instance_segmentation=True
                )
                rp = rep.create.render_product(self._rep_camera, (512, 512))
                writer.attach(rp)
                for i in range(self.PALLET_SCENARIO_FRAMES):
                    await rep.orchestrator.step_async(rt_subframes=24, delta_time=0.0)

                # Free up resources after the capture
                writer.detach()
                rp.destroy()

                # Restore the original materials of the randomized meshes
                for mesh, mat in mesh_to_orig_mats.items():
                    print(f"[PalletizingSDGDemo] Restoring original material({mat}) for {mesh.GetPath()}")
                    UsdShade.MaterialBindingAPI(mesh).Bind(mat, UsdShade.Tokens.strongerThanDescendants)

                # Make sure the backend finishes writing the data before clearing the generated SDG graph
                await rep.orchestrator.wait_until_complete_async()

                # Cleanup the generated SDG graph
                if self._stage.GetPrimAtPath("/Replicator"):
                    omni.kit.commands.execute("DeletePrimsCommand", paths=["/Replicator"])

                self._replicator_running = False
                self._timeline.play()
                if self._next_bin():
                    self._timeline_sub = self._timeline.get_timeline_event_stream().create_subscription_to_pop_by_type(
                        int(omni.timeline.TimelineEventType.CURRENT_TIME_TICKED), self._on_timeline_event
                    )

            def _create_bin_and_pallet_graph(self):
                # Bin material randomization
                bin_paths = [
                    f"{self.BINS_FOLDER_PATH}/bin_{i}/Visuals/FOF_Mesh_Magenta_Box"
                    for i in range(self._bin_counter + 1)
                ]
                bins_node = rep.get.prim_at_path(bin_paths)

                with rep.trigger.on_frame():
                    mats = rep.create.material_omnipbr(
                        diffuse=rep.distribution.uniform((0.2, 0.1, 0.3), (0.6, 0.6, 0.7)),
                        roughness=rep.distribution.choice([0.1, 0.9]),
                        count=10,
                    )
                    with bins_node:
                        rep.randomizer.materials(mats)

                # Camera and pallet texture randomization at a slower rate
                assets_root_path = get_assets_root_path()
                texture_paths = [
                    assets_root_path + "/NVIDIA/Materials/Base/Wood/Oak/Oak_BaseColor.png",
                    assets_root_path + "/NVIDIA/Materials/Base/Wood/Ash/Ash_BaseColor.png",
                    assets_root_path + "/NVIDIA/Materials/Base/Wood/Plywood/Plywood_BaseColor.png",
                    assets_root_path + "/NVIDIA/Materials/Base/Wood/Timber/Timber_BaseColor.png",
                ]
                pallet_node = rep.get.prim_at_path(self.PALLET_PRIM_MESH_PATH)
                pallet_prim = pallet_node.get_output_prims()["prims"][0]
                pallet_loc = omni.usd.get_world_transform_matrix(pallet_prim).ExtractTranslation()
                self._rep_camera = rep.create.camera()
                with rep.trigger.on_frame(interval=4):
                    with pallet_node:
                        rep.randomizer.texture(texture_paths, texture_rotate=rep.distribution.uniform(80, 95))
                    with self._rep_camera:
                        rep.modify.pose(
                            position=rep.distribution.uniform((0, -2, 1), (2, 1, 2)),
                            look_at=(pallet_loc[0], pallet_loc[1], pallet_loc[2]),
                        )

            def _next_bin(self):
                self._bin_counter += 1
                if self._bin_counter >= self._num_captures:
                    self.clear()
                    print("[PalletizingSDGDemo] Palletizing SDG demo finished..")
                    return False
                self._active_bin = self._stage.GetPrimAtPath(f"{self.BINS_FOLDER_PATH}/bin_{self._bin_counter}")
                print(f"[PalletizingSDGDemo] Moving to bin {self._bin_counter}..")
                self._bin_flip_scenario_done = False
                return True

        NUM_CAPTURES = 2

        async def run_example_async():
            import random

            from isaacsim.examples.interactive.ur10_palletizing.ur10_palletizing import BinStacking

            # Load the bin stacking stage and start the demo
            random.seed(42)
            rep.set_global_seed(42)
            bin_staking_sample = BinStacking()
            await bin_staking_sample.load_world_async()
            await bin_staking_sample.on_event_async()
            # Wait a few frames for the stage to fully load and the stacking demo to start
            for _ in range(3):
                await omni.kit.app.get_app().next_update_async()

            # Start the SDG pipeline on top of the palletizing demo
            sdg_demo = PalletizingSDGDemo()
            sdg_demo.start(num_captures=NUM_CAPTURES)

            # Wait until the demo is finished
            while sdg_demo.is_running():
                await omni.kit.app.get_app().next_update_async()
            print("[PalletizingSDGDemo] Done..")

            # Check if all the expected files were written
            all_data_written = validate_folder_contents(
                sdg_demo._output_dir, {"png": 80, "json": 72}, include_subfolders=True
            )
            self.assertTrue(all_data_written, f"Not all files were written in to: {sdg_demo._output_dir}")

        await run_example_async()
