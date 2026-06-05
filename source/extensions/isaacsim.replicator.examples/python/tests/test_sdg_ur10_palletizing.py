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

import os
import tempfile
import unittest

import carb.settings
import omni.kit
import omni.usd
from isaacsim.test.utils.file_validation import validate_folder_contents


class TestSDGUR10Palletizing(omni.kit.test.AsyncTestCase):

    async def setUp(self):
        await omni.kit.app.get_app().next_update_async()
        await omni.usd.get_context().new_stage_async()
        await omni.kit.app.get_app().next_update_async()
        self.original_dlss_exec_mode = carb.settings.get_settings().get("rtx/post/dlss/execMode")

    async def tearDown(self):
        omni.usd.get_context().close_stage()
        await omni.kit.app.get_app().next_update_async()
        # In some cases the test will end before the asset is loaded, in this case wait for assets to load
        while omni.usd.get_context().get_stage_loading_status()[2] > 0:
            await omni.kit.app.get_app().next_update_async()
        carb.settings.get_settings().set("rtx/post/dlss/execMode", self.original_dlss_exec_mode)

    @unittest.skipIf(os.getenv("ETM_ACTIVE"), "Skipped in ETM.")
    async def test_sdg_ur10_palletizing(self):
        import asyncio
        import json
        import os

        import carb.settings
        import omni
        import omni.kit.app
        import omni.kit.commands
        import omni.replicator.core as rep
        import omni.timeline
        import omni.usd
        from isaacsim.storage.native import get_assets_root_path
        from omni.physx import get_physx_scene_query_interface
        from omni.replicator.core.functional import write_image
        from pxr import Usd, UsdGeom, UsdShade

        DEFAULT_NUM_CAPTURES = 4  # Number bins to capture
        DEFAULT_BIN_FLIP_FRAMES = 2  # Number of frames to capture for the bin flip scenario
        DEFAULT_PALLET_FRAMES = 2  # Number of frames to capture for the pallet scenario
        MAX_BINS = 36  # Maximum number of bins available in the scene

        class PalletizingSDGDemo:
            BINS_FOLDER_PATH = "/World/Ur10Table/bins"
            FLIP_HELPER_PATH = "/World/Ur10Table/pallet_holder"
            PALLET_PRIM_MESH_PATH = "/World/Ur10Table/pallet/Xform/Mesh_015"

            def __init__(self, output_dir=None):
                # There are 36 bins in total
                self._bin_counter = 0
                self._num_captures = MAX_BINS
                self._bin_flip_frames = DEFAULT_BIN_FLIP_FRAMES
                self._pallet_frames = DEFAULT_PALLET_FRAMES
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
                self._output_dir = (
                    output_dir if output_dir is not None else os.path.join(os.getcwd(), "_out_palletizing_sdg_demo")
                )
                print(f"[PalletizingSDGDemo] Output directory: {self._output_dir}")

            def start(self, num_captures, bin_flip_frames, pallet_frames):
                self._num_captures = num_captures if 1 <= num_captures <= 36 else 36
                self._bin_flip_frames = bin_flip_frames
                self._pallet_frames = pallet_frames
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

                bb_cache = UsdGeom.BBoxCache(time=Usd.TimeCode.Default(), includedPurposes=[UsdGeom.Tokens.default_])
                half_ext = bb_cache.ComputeLocalBound(self._active_bin).GetRange().GetSize() * 0.5
                self._overlap_extent = carb.Float3(half_ext[0], half_ext[1], half_ext[2] * 1.1)

                self._timeline = omni.timeline.get_timeline_interface()
                if not self._timeline.is_playing():
                    print("[PalletizingSDGDemo] Please start the palletizing demo first..")
                    return False

                # Disable capture on play for replicator, data capture will be triggered manually
                rep.orchestrator.set_capture_on_play(False)

                # Set DLSS to Quality mode (2) for best SDG results (Options: 0 (Performance), 1 (Balanced), 2 (Quality), 3 (Auto)
                carb.settings.get_settings().set("rtx/post/dlss/execMode", 2)

                # Clear any previously generated SDG graphs
                if self._stage.GetPrimAtPath("/Replicator"):
                    omni.kit.commands.execute("DeletePrimsCommand", paths=["/Replicator"])

                return True

            def _start(self):
                self._timeline_sub = carb.eventdispatcher.get_eventdispatcher().observe_event(
                    event_name=omni.timeline.GLOBAL_EVENT_CURRENT_TIME_TICKED,
                    on_event=self._on_timeline_event,
                    observer_name="test_sdg_ur10_palletizing.PalletizingSDGDemo._on_timeline_event",
                )
                self._stage_event_sub = carb.eventdispatcher.get_eventdispatcher().observe_event(
                    event_name=omni.usd.get_context().stage_event_name(omni.usd.StageEventType.CLOSING),
                    on_event=self._on_stage_closing_event,
                    observer_name="test_sdg_ur10_palletizing.PalletizingSDGDemo._on_stage_closing_event",
                )
                self._in_running_state = True
                print("[PalletizingSDGDemo] Starting the palletizing SDG demo..")

            def clear(self):
                if self._timeline_sub:
                    self._timeline_sub.reset()
                    self._timeline_sub = None
                if self._stage_event_sub:
                    self._stage_event_sub.reset()
                    self._stage_event_sub = None
                self._in_running_state = False
                self._bin_counter = 0
                self._active_bin = None
                if self._stage.GetPrimAtPath("/Replicator"):
                    omni.kit.commands.execute("DeletePrimsCommand", paths=["/Replicator"])

            def _on_stage_closing_event(self, e: carb.eventdispatcher.Event):
                # Make sure the subscribers are unsubscribed for new stages
                self.clear()

            def _on_timeline_event(self, e: carb.eventdispatcher.Event):
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
                # Skip self-hits
                if hit.rigid_body == self._active_bin.GetPrimPath():
                    return True

                # Handle flip scenario (only once per bin)
                if not self._bin_flip_scenario_done and hit.rigid_body.startswith(self.FLIP_HELPER_PATH):
                    self._timeline.pause()
                    if self._timeline_sub:
                        self._timeline_sub.reset()
                        self._timeline_sub = None
                    asyncio.ensure_future(self._run_bin_flip_scenario())
                    return False

                # Handle pallet landing scenario
                is_pallet_hit = hit.rigid_body.startswith(self.PALLET_PRIM_MESH_PATH)
                is_other_bin_hit = hit.rigid_body.startswith(f"{self.BINS_FOLDER_PATH}/bin_")
                if is_pallet_hit or is_other_bin_hit:
                    self._timeline.pause()
                    if self._timeline_sub:
                        self._timeline_sub.reset()
                        self._timeline_sub = None
                    asyncio.ensure_future(self._run_pallet_scenario())

                return True  # No relevant hit, return True to continue the query

            def _switch_to_pathtracing(self, spp=32, total_spp=32):
                carb.settings.get_settings().set("/rtx/rendermode", "PathTracing")
                carb.settings.get_settings().set("/rtx/pathtracing/spp", spp)
                carb.settings.get_settings().set("/rtx/pathtracing/totalSpp", total_spp)

            def _switch_to_realtime_pathtracing(self):
                carb.settings.get_settings().set("/rtx/rendermode", "RealTimePathTracing")

            async def _run_bin_flip_scenario(self):
                await omni.kit.app.get_app().next_update_async()
                print(f"[PalletizingSDGDemo] Running bin flip scenario for bin {self._bin_counter}..")

                self._switch_to_pathtracing(spp=16, total_spp=32)
                await omni.kit.app.get_app().next_update_async()
                self._create_bin_flip_graph()

                rgb_annot = rep.annotators.get("rgb")
                instance_segmentation_annot = rep.annotators.get(
                    "instance_segmentation", init_params={"colorize": True}
                )
                rp = rep.create.render_product(self._rep_camera, (512, 512))
                rgb_annot.attach(rp)
                instance_segmentation_annot.attach(rp)
                out_dir = os.path.join(self._output_dir, f"annot_bin_{self._bin_counter}")
                os.makedirs(out_dir, exist_ok=True)

                print(
                    f"[PalletizingSDGDemo] Starting capturing data for bin flip scenario for bin {self._bin_counter}.."
                )
                for i in range(self._bin_flip_frames):
                    print(f"  [PalletizingSDGDemo] Capturing frame {i + 1}/{self._bin_flip_frames}")
                    await rep.orchestrator.step_async(rt_subframes=16, delta_time=0.0)

                    rgb_data = rgb_annot.get_data()
                    rgb_file_path = os.path.join(out_dir, f"rgb_{i}.png")
                    write_image(path=rgb_file_path, data=rgb_data)

                    instance_segmentation_data = instance_segmentation_annot.get_data()
                    instance_segmentation_file_path = os.path.join(out_dir, f"instance_segmentation_{i}.png")
                    write_image(path=instance_segmentation_file_path, data=instance_segmentation_data["data"])
                    with open(os.path.join(out_dir, f"instance_segmentation_info_{i}.json"), "w") as f:
                        json.dump(instance_segmentation_data["info"], f, indent=4)

                # Wait for the data to be written to disk and free up resources after the capture
                await rep.orchestrator.wait_until_complete_async()
                rgb_annot.detach()
                instance_segmentation_annot.detach()
                rp.destroy()

                # Cleanup the generated SDG graph
                if self._stage.GetPrimAtPath("/Replicator"):
                    omni.kit.commands.execute("DeletePrimsCommand", paths=["/Replicator"])

                self._switch_to_realtime_pathtracing()

                # Set the flag to indicate that the bin flip scenario is done and the simulation can continue to the next bin
                self._bin_flip_scenario_done = True
                self._timeline_sub = carb.eventdispatcher.get_eventdispatcher().observe_event(
                    event_name=omni.timeline.GLOBAL_EVENT_CURRENT_TIME_TICKED,
                    on_event=self._on_timeline_event,
                    observer_name="test_sdg_ur10_palletizing.PalletizingSDGDemo._on_timeline_event",
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
                camera_positions = [
                    (1.96, 0.72, -0.34),
                    (1.48, 0.70, 0.90),
                    (0.79, -0.86, 0.12),
                    (-0.49, 1.47, 0.58),
                ]
                self._rep_camera = rep.create.camera()
                with rep.trigger.on_frame():
                    rep.randomizer.randomize_bin_flip_lights()
                    with self._rep_camera:
                        rep.modify.pose(
                            position=rep.distribution.sequence(camera_positions),
                            look_at=(0.78, 0.72, -0.1),
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
                backend = rep.backends.get("DiskBackend")
                backend.initialize(output_dir=out_dir)
                writer = rep.WriterRegistry.get("BasicWriter")
                writer.initialize(
                    backend=backend,
                    rgb=True,
                    instance_segmentation=True,
                    colorize_instance_segmentation=True,
                )
                rp = rep.create.render_product(self._rep_camera, (512, 512))
                writer.attach(rp)

                print(f"[PalletizingSDGDemo] Starting capturing data for pallet scenario for bin {self._bin_counter}..")
                for i in range(self._pallet_frames):
                    print(f"  [PalletizingSDGDemo] Capturing frame {i + 1}/{self._pallet_frames}")
                    await rep.orchestrator.step_async(rt_subframes=16, delta_time=0.0)

                # Make sure the backend finishes writing the data before clearing the generated SDG graph
                await rep.orchestrator.wait_until_complete_async()

                # Free up resources after the capture
                writer.detach()
                rp.destroy()

                # Cleanup the generated SDG graph
                print(f"[PalletizingSDGDemo] Restoring {len(mesh_to_orig_mats)} original materials")
                for mesh, mat in mesh_to_orig_mats.items():
                    UsdShade.MaterialBindingAPI(mesh).Bind(mat, UsdShade.Tokens.strongerThanDescendants)

                # Cleanup the generated SDG graph
                if self._stage.GetPrimAtPath("/Replicator"):
                    omni.kit.commands.execute("DeletePrimsCommand", paths=["/Replicator"])

                # Return in paused state if there are no more bins to capture
                if not self._next_bin():
                    return

                # Resume the simulation and continue with the next bin
                self._timeline_sub = carb.eventdispatcher.get_eventdispatcher().observe_event(
                    event_name=omni.timeline.GLOBAL_EVENT_CURRENT_TIME_TICKED,
                    on_event=self._on_timeline_event,
                    observer_name="test_sdg_ur10_palletizing.PalletizingSDGDemo._on_timeline_event",
                )
                self._timeline.play()

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

        async def run_example_async(num_captures, bin_flip_frames, pallet_frames, output_dir=None):
            import random

            from isaacsim.cortex.examples.ur10_palletizing.ur10_palletizing import (
                BinStacking,
            )

            # Createa new stage
            await omni.usd.get_context().new_stage_async()

            # Seed for the bin drop stage(if it needs to be flipped or not)
            random.seed(42)

            # Seed for the replicator randomization
            rep.set_global_seed(42)

            # Load the bin stacking stage and start the demo
            bin_staking_sample = BinStacking()
            print(f"[PalletizingSDGDemo] Loading the bin stacking stage..")
            await bin_staking_sample.load_world_async()
            print(f"[PalletizingSDGDemo] Starting bin stacking..")
            await bin_staking_sample.on_event_async()

            # Wait a few frames for the stage to fully load then start the SDG pipeline
            for _ in range(5):
                await omni.kit.app.get_app().next_update_async()

            print(f"[PalletizingSDGDemo] Starting SDG pipeline with {num_captures} bins to capture")
            sdg_demo = PalletizingSDGDemo(output_dir=output_dir)
            sdg_demo.start(num_captures, bin_flip_frames, pallet_frames)

            # Wait until the SDG pipeline demo is finished
            while sdg_demo.is_running():
                await omni.kit.app.get_app().next_update_async()
            print("[PalletizingSDGDemo] SDG pipeline finished, pausing the simulation..")
            timeline = omni.timeline.get_timeline_interface()
            timeline.pause()

        # asyncio.ensure_future(
        #     run_example_async(
        #         num_captures=DEFAULT_NUM_CAPTURES, bin_flip_frames=DEFAULT_BIN_FLIP_FRAMES, pallet_frames=DEFAULT_PALLET_FRAMES
        #     )
        # )

        # Test scenario
        test_num_captures = 2
        test_bin_flip_frames = 2
        test_pallet_frames = 2
        out_dir = tempfile.mkdtemp(prefix="test_palletizing_sdg_")
        print(f"Output directory: {out_dir}")
        await run_example_async(test_num_captures, test_bin_flip_frames, test_pallet_frames, output_dir=out_dir)

        # Validate that all expected files were written to disk

        # Bin flip scenario happens randomly, but with seed=42, we get 2 flips out of 2 captures
        num_flips = 2

        # Bin flip scenario (uses annotators directly):
        # - Outputs per frame: 2 PNGs (rgb, instance_segmentation) + 1 JSON (instance_segmentation annotator with 1 json file)
        bin_flip_pngs = num_flips * test_bin_flip_frames * 2
        bin_flip_jsons = num_flips * test_bin_flip_frames * 1

        # Pallet scenario (uses BasicWriter with backend):
        # - Outputs per frame: 2 PNGs (rgb, instance_segmentation) + 2 JSONs (basic writer instance_segmentation with 2 json files)
        pallet_pngs = test_num_captures * test_pallet_frames * 2
        pallet_jsons = test_num_captures * test_pallet_frames * 2

        expected_pngs = bin_flip_pngs + pallet_pngs
        expected_jsons = bin_flip_jsons + pallet_jsons
        print(f"Expected PNGs: {expected_pngs}, Expected JSONs: {expected_jsons}")

        all_data_written = validate_folder_contents(
            out_dir, {"png": expected_pngs, "json": expected_jsons}, recursive=True
        )
        self.assertTrue(all_data_written, f"Not all files were written in to: {out_dir}")
