# SPDX-FileCopyrightText: Copyright (c) 2021-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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

"""Tests for camera sensor warmup behavior."""

from unittest.mock import patch

import omni.replicator.core as rep
import omni.timeline
import omni.usd
from isaacsim.core.experimental.objects import DomeLight, GroundPlane
from isaacsim.sensors.camera import Camera


class TestCameraSensor(omni.kit.test.AsyncTestCase):
    """Test cases for CameraSensor."""

    async def setUp(self) -> None:
        """Set up test fixtures."""
        await omni.kit.app.get_app().next_update_async()
        omni.usd.get_context().new_stage()
        await omni.kit.app.get_app().next_update_async()

    async def tearDown(self) -> None:
        """Tear down test fixtures."""
        omni.usd.get_context().close_stage()
        await omni.kit.app.get_app().next_update_async()
        # In some cases the test will end before the asset is loaded, in this case wait for assets to load
        while omni.usd.get_context().get_stage_loading_status()[2] > 0:
            await omni.kit.app.get_app().next_update_async()

    async def test_camera_sensor_no_warmup(self) -> None:
        """Test camera sensor no warmup."""
        # Test constants
        RESOLUTION = (720, 480)
        MAX_WARMUP_FRAMES = 20
        WARMUP_WARNING_TEXT = "A few render frames may be required"

        # Create scene
        rep.functional.create.xform(name="World")
        dome_light = DomeLight("/World/DomeLight")
        dome_light.set_intensities(500)
        GroundPlane("/World/defaultGroundPlane", sizes=100.0, templates=None)
        rep.functional.create.camera(position=(0, 0, 3), look_at=(0, 0, 0), parent="/World", name="Camera")

        # Initialize camera with annotators
        camera = Camera(prim_path="/World/Camera", resolution=RESOLUTION)
        camera.initialize()
        camera.add_rgb_to_frame()
        camera.add_distance_to_image_plane_to_frame()
        camera.add_pointcloud_to_frame()

        # Test 1: Data should be unavailable before warmup and warnings should be logged
        with patch("carb.log_warn") as mock_warn:
            rgba = camera.get_rgba()
            rgb = camera.get_rgb()
            depth = camera.get_depth()
            pointcloud = camera.get_pointcloud()

            self.assertIsNone(rgba, "RGBA should be None before warmup")
            self.assertIsNone(rgb, "RGB should be None before warmup")
            self.assertIsNone(depth, "Depth should be None before warmup")
            self.assertEqual(pointcloud.size, 0, "Pointcloud should be empty before warmup")

            self.assertGreaterEqual(mock_warn.call_count, 3, "Should log warnings for unavailable data")
            warmup_warnings = [call for call in mock_warn.call_args_list if WARMUP_WARNING_TEXT in str(call)]
            self.assertGreater(len(warmup_warnings), 0, f"Should log '{WARMUP_WARNING_TEXT}' warnings")

        # Timeline needs to play for data to be available because of the frequency of the camera sensor
        timeline = omni.timeline.get_timeline_interface()
        timeline.play()
        timeline.commit()

        # Run warmup and track when data becomes available
        frames_until_ready = None
        for frame_idx in range(MAX_WARMUP_FRAMES):
            await omni.kit.app.get_app().next_update_async()
            if camera.get_rgba() is not None:
                frames_until_ready = frame_idx
                print(f"Data became available after {frame_idx} frames")
                break

        self.assertIsNotNone(frames_until_ready, f"Data should be available within {MAX_WARMUP_FRAMES} frames")

        # Test 2: Data should be available after warmup
        rgba = camera.get_rgba()
        rgb = camera.get_rgb()
        depth = camera.get_depth()
        pointcloud = camera.get_pointcloud()

        self.assertIsNotNone(rgba, "RGBA should be available after warmup")
        self.assertIsNotNone(rgb, "RGB should be available after warmup")
        self.assertIsNotNone(depth, "Depth should be available after warmup")
        self.assertIsNotNone(pointcloud, "Pointcloud should be available after warmup")

        height, width = RESOLUTION[1], RESOLUTION[0]
        self.assertEqual(rgba.shape, (height, width, 4))
        self.assertEqual(rgb.shape, (height, width, 3))
        self.assertEqual(depth.shape, (height, width))
        self.assertEqual(pointcloud.shape, (height * width, 3))
