# SPDX-FileCopyrightText: Copyright (c) 2025-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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

"""Tests for camera lifecycle management."""

import math
from unittest.mock import patch

import carb
import omni.kit.test
import omni.replicator.core as rep
import omni.usd
from isaacsim.sensors.camera import Camera


class TestCameraLifecycle(omni.kit.test.AsyncTestCase):
    """Test cases for CameraLifecycle."""

    async def setUp(self) -> None:
        """Set up test fixtures."""
        await omni.kit.app.get_app().next_update_async()
        omni.usd.get_context().new_stage()
        await omni.kit.app.get_app().next_update_async()

        # Minimal scene with a camera prim that the Camera wrapper can bind to.
        rep.functional.create.xform(name="World")
        rep.functional.create.camera(position=(0, 0, 3), look_at=(0, 0, 0), parent="/World", name="Camera")
        await omni.kit.app.get_app().next_update_async()

    async def tearDown(self) -> None:
        """Tear down test fixtures."""
        omni.usd.get_context().close_stage()
        await omni.kit.app.get_app().next_update_async()
        while omni.usd.get_context().get_stage_loading_status()[2] > 0:
            await omni.kit.app.get_app().next_update_async()

    async def test_destroy_cleans_up_observers_and_annotators(self) -> None:
        """Test destroy cleans up observers and annotators."""
        camera = Camera(prim_path="/World/Camera", resolution=(64, 64))
        camera.initialize()

        # Attach at least one custom annotator so destroy() must detach it.
        camera.add_rgb_to_frame()

        # Sanity: these should be created by initialize()
        self.assertIsNotNone(camera._fabric_time_annotator)
        self.assertIsNotNone(camera._acquisition_callback)
        self.assertIsNotNone(camera._stage_open_callback)
        self.assertIsNotNone(camera._timer_reset_callback_stop)
        self.assertIsNotNone(camera._timer_reset_callback_play)
        self.assertIn("rgb", camera._custom_annotators)
        self.assertIsNotNone(camera.get_render_product_path())

        camera.destroy()

        # These should be torn down by destroy()
        self.assertIsNone(camera._fabric_time_annotator)
        self.assertIsNone(camera._acquisition_callback)
        self.assertIsNone(camera._stage_open_callback)
        self.assertIsNone(camera._timer_reset_callback_stop)
        self.assertIsNone(camera._timer_reset_callback_play)
        self.assertEqual(len(camera._custom_annotators), 0)
        self.assertIsNone(camera.get_render_product_path())

        # Destroy should be idempotent and keep cleared state.
        camera.destroy()
        self.assertIsNone(camera._fabric_time_annotator)
        self.assertEqual(len(camera._custom_annotators), 0)
        self.assertIsNone(camera.get_render_product_path())

    async def test_attach_annotator_before_initialize_raises_clear_error(self) -> None:
        """Test attach_annotator fails clearly before initialize creates a render product."""
        camera = Camera(prim_path="/World/Camera", resolution=(64, 64))

        with patch.object(
            rep.AnnotatorRegistry,
            "get_annotator",
            side_effect=AssertionError("attach_annotator should not request annotators before initialize()"),
        ):
            with self.assertRaisesRegex(RuntimeError, r"initialize\(\) must be called before attach_annotator"):
                camera.attach_annotator("rgb")

        self.assertEqual(len(camera._custom_annotators), 0)
        self.assertNotIn("rgb", camera.get_current_frame())

    async def test_dt_and_clipping_range_edge_cases(self) -> None:
        """Test dt and clipping range edge cases."""
        camera = Camera(prim_path="/World/Camera", resolution=(64, 64))

        # Ensure 0.0 is not treated as "unset"
        camera.set_clipping_range(near_distance=0.0, far_distance=10.0)
        near, far = camera.get_clipping_range()
        self.assertTrue(math.isclose(near, 0.0, rel_tol=0.0, abs_tol=1e-8))
        self.assertTrue(math.isclose(far, 10.0, rel_tol=0.0, abs_tol=1e-8))

        # Make set_dt deterministic by forcing a known render loop frequency.
        settings = carb.settings.get_settings()
        path = "/app/runLoops/main/rateLimitFrequency"
        original = settings.get(path)
        try:
            settings.set(path, 120)
            camera.set_dt(1.0 / 60.0)  # 2 * (1/120) => valid
            self.assertTrue(math.isclose(camera.get_dt(), 1.0 / 60.0, rel_tol=0.0, abs_tol=1e-8))

            with self.assertRaises(Exception):
                camera.set_dt(1.0 / 61.0)  # not a multiple of (1/120)
        finally:
            # Restore original setting to avoid leaking config across tests.
            if original is None:
                settings.set(path, None)
            else:
                settings.set(path, original)
