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

"""Test camera context menu functionality."""

import isaacsim.core.experimental.utils.stage as stage_utils
import omni.kit.app
import omni.kit.ui_test as ui_test
import omni.usd
from isaacsim.sensors.camera.ui import Extension
from isaacsim.test.utils import MenuUITestCase, count_menu_items, get_all_menu_paths


class TestCameraContextMenu(MenuUITestCase):
    """Test camera context menu."""

    async def test_camera_context_menu_count(self):
        """Test that all the Camera and Depth Sensor menu items are added correctly.

        The expected count is derived dynamically from ``Extension.SENSORS`` (the same
        source-of-truth dict the menu is built from), so adding or removing a sensor in
        the extension does not require updating this test.
        """
        viewport_context_menu = await self.get_viewport_context_menu()
        self.assertIsNotNone(viewport_context_menu, "Failed to get viewport context menu")

        camera_menu_dict = viewport_context_menu["Create"]["Isaac"]["Sensors"]["Camera and Depth Sensors"]
        n_items = count_menu_items(camera_menu_dict)
        expected_items = sum(len(sensors) for sensors in Extension.SENSORS.values())

        self.assertEqual(
            n_items,
            expected_items,
            f"The number of items in the Camera and Depth Sensors menu ({n_items}) does not match the expected ({expected_items})",
        )

    async def test_camera_sensors_context_menu_click(self):
        """Test the Camera and Depth Sensors are added to stage context menus correctly."""
        viewport_context_menu = await self.get_viewport_context_menu()
        self.assertIsNotNone(viewport_context_menu, "Failed to get viewport context menu")

        camera_viewport_menu_dict = viewport_context_menu["Create"]["Isaac"]["Sensors"]["Camera and Depth Sensors"]
        all_menu_paths = get_all_menu_paths(camera_viewport_menu_dict)

        for test_path in all_menu_paths:
            full_test_path = "Create/Isaac/Sensors/Camera and Depth Sensors/" + test_path

            await stage_utils.create_new_stage_async()
            await self.wait_n_frames(2)

            await self.get_viewport_context_menu()
            await ui_test.select_context_menu(full_test_path, offset=ui_test.Vec2(10, 10))

            await self.wait_for_stage_loading()
            await self.wait_n_frames(50)

            # Check if there is more than one Camera prim on stage
            stage = omni.usd.get_context().get_stage()
            n_cameras = sum(1 for prim in stage.TraverseAll() if prim.GetTypeName() == "Camera")

            self.assertGreater(
                n_cameras, 1, f"There are {n_cameras} Camera prims on stage for {full_test_path}, expected at least 1."
            )
