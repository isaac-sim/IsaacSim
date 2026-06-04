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

import omni.kit
import omni.replicator.core as rep
import omni.timeline
import omni.usd
from isaacsim.test.utils.file_validation import validate_folder_contents


class TestSDGUsefulSnippets(omni.kit.test.AsyncTestCase):
    async def setUp(self):
        await omni.kit.app.get_app().next_update_async()
        omni.usd.get_context().new_stage()
        await omni.kit.app.get_app().next_update_async()

    async def tearDown(self):
        omni.usd.get_context().close_stage()
        await omni.kit.app.get_app().next_update_async()
        # In some cases the test will end before the asset is loaded, in this case wait for assets to load
        while omni.usd.get_context().get_stage_loading_status()[2] > 0:
            await omni.kit.app.get_app().next_update_async()

    async def test_capture_data_with_timeline_running(self):
        await omni.kit.app.get_app().next_update_async()
        rep.orchestrator.set_capture_on_play(False)

        # Start timeline and run for a several frames
        timeline = omni.timeline.get_timeline_interface()
        timeline.play()
        print(f"Running several app updates before writer initialization")
        for i in range(10):
            await omni.kit.app.get_app().next_update_async()

        # Create and attach the writer while the timeline is running
        out_dir = tempfile.mkdtemp(prefix="test_sdg_timeline_running_")
        print(f"output_directory: {out_dir}")
        basic_writer = rep.WriterRegistry.get("BasicWriter")
        basic_writer.initialize(output_dir=out_dir, rgb=True)
        rp = rep.create.render_product("/OmniverseKit_Persp", (1280, 720))
        basic_writer.attach(rp)

        # Run the simulation for a few frames (these frames should not be captured)
        for i in range(10):
            await omni.kit.app.get_app().next_update_async()

        # Capture frames
        num_frame_captures = 5
        print(f"Capturing {num_frame_captures} frames")
        for i in range(num_frame_captures):
            print(f"  Step {i}")
            await rep.orchestrator.step_async(delta_time=0.0, pause_timeline=False)

        # Wait for the writer to finish
        await rep.orchestrator.wait_until_complete_async()
        basic_writer.detach()
        rp.destroy()
        timeline.stop()

        # Validate the output directory contents
        folder_contents_success = validate_folder_contents(path=out_dir, expected_counts={"png": num_frame_captures})
        self.assertTrue(folder_contents_success, f"Output directory contents validation failed for {out_dir}")

    async def test_toggled_render_product_captures_with_timeline_running(self):
        num_captures = 5
        num_iterations = 4
        app_updates_per_capture = 20

        for iteration in range(num_iterations):
            await omni.usd.get_context().new_stage_async()
            rep.orchestrator.set_capture_on_play(False)

            camera = rep.functional.create.camera(position=(2.0, 2.0, 2.0), look_at=(0.0, 0.0, 0.0))
            render_product = rep.create.render_product(camera, (400, 400), name="rp_camera", force_new=True)
            render_product.hydra_texture.set_updates_enabled(False)

            out_dir = tempfile.mkdtemp(prefix=f"test_sdg_toggled_rp_{iteration}_")
            print(f"output_directory[{iteration}]: {out_dir}")
            backend = rep.backends.get("DiskBackend")
            backend.initialize(output_dir=out_dir)
            writer = rep.writers.get("BasicWriter")
            writer.initialize(backend=backend, rgb=True)
            writer.attach([render_product])

            timeline = omni.timeline.get_timeline_interface()
            timeline.play()

            try:
                for _ in range(num_captures):
                    for _ in range(app_updates_per_capture):
                        await omni.kit.app.get_app().next_update_async()

                    render_product.hydra_texture.set_updates_enabled(True)
                    await rep.orchestrator.step_async()
                    render_product.hydra_texture.set_updates_enabled(False)

                await rep.orchestrator.wait_until_complete_async()
                folder_contents_success = validate_folder_contents(path=out_dir, expected_counts={"png": num_captures})
                self.assertTrue(
                    folder_contents_success,
                    f"Output directory contents validation failed for iteration {iteration}: {out_dir}",
                )
            finally:
                writer.detach()
                timeline.stop()
                render_product.destroy()
