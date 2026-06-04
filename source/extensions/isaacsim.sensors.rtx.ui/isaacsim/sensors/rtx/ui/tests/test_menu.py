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

"""Test menu functionality."""

from isaacsim.core.experimental.utils.stage import get_current_stage
from isaacsim.test.utils import MenuUITestCase, get_all_menu_paths
from omni.kit.mainwindow import get_main_window
from omni.kit.ui_test import get_context_menu

SENSOR_ROOT_PATH = "Create/Sensors/RTX Lidar"


class TestMenuAssets(MenuUITestCase):
    """Test class for verifying RTX Lidar sensor menu functionality."""

    async def _test_menu_option(self, test_path: str) -> None:
        """Test a specific menu option.

        Args:
            test_path: The menu path to test.
        """
        await self.menu_click_with_retry(test_path)
        await self.run_timeline_frames(5)

        num_prims = 0
        sensor_passed = False

        for prim in get_current_stage().Traverse():
            num_prims += 1
            if prim.IsA("OmniLidar"):
                sensor_passed = True
                break

        self.assertGreater(num_prims, 0, "No prims added to stage.")
        self.assertTrue(sensor_passed, f"{test_path} did not pass, missing prim or wrong prim type")

    async def test_rtx_lidar_menu_items(self):
        """Test all RTX Lidar sensor menu items."""
        # Get menu dict at runtime instead of module load time
        window = get_main_window()
        menu_dict = await get_context_menu(window._ui_main_window.main_menu_bar, get_all=False)

        sensor_menu_dict = menu_dict.get("Create", {}).get("Sensors", {}).get("RTX Lidar", {})
        sensor_menu_list = get_all_menu_paths(sensor_menu_dict, root_path=SENSOR_ROOT_PATH)

        self.assertGreater(len(sensor_menu_list), 0, f"No menu items found in {SENSOR_ROOT_PATH}")

        for test_path in sensor_menu_list:
            with self.subTest(menu_path=test_path):
                await self._test_menu_option(test_path)
                # Reset stage for next iteration
                await self.new_stage()
