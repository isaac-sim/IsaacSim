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

from isaacsim.core.utils.stage import traverse_stage
from isaacsim.test.utils import MenuUITestCase, get_all_menu_paths
from omni.kit.mainwindow import get_main_window
from omni.kit.ui_test import get_context_menu

PHYSX_LIDAR_ROOT_PATH = "Create/Sensors/PhysX Lidar"
LIGHTBEAM_ROOT_PATH = "Create/Sensors/LightBeam Sensor"


class TestPhysxMenuAssets(MenuUITestCase):
    """Test class for verifying PhysX sensor menu functionality."""

    async def _test_physx_lidar_option(self, test_path: str) -> None:
        """Test a PhysX Lidar menu option.

        Args:
            test_path: The menu path to test.
        """
        await self.menu_click_with_retry(test_path)
        await self.run_timeline_frames(5)

        num_prims = 0
        sensor_passed = False

        for prim in traverse_stage():
            num_prims += 1
            prim_type = prim.GetTypeName()
            if prim_type in ("Lidar", "Generic"):
                sensor_passed = True
                break

        self.assertGreater(num_prims, 0, "No prims added to stage.")
        self.assertTrue(sensor_passed, f"{test_path} did not pass, missing prim or wrong prim type")

    async def _test_lightbeam_option(self, test_path: str) -> None:
        """Test a LightBeam Sensor menu option.

        Args:
            test_path: The menu path to test.
        """
        await self.menu_click_with_retry(test_path)
        await self.run_timeline_frames(5)

        num_prims = 0
        sensor_passed = False

        for prim in traverse_stage():
            num_prims += 1
            prim_type = prim.GetTypeName()
            if prim_type == "IsaacLightBeamSensor":
                sensor_passed = True
                break
            if prim_type == "Xform" and "TS" in prim.GetPath().pathString:
                sensor_passed = True
                break

        self.assertGreater(num_prims, 0, "No prims added to stage.")
        self.assertTrue(sensor_passed, f"{test_path} did not pass, missing prim or wrong prim type")

    async def test_physx_sensor_menu_items(self) -> None:
        """Test all PhysX Lidar and LightBeam sensor menu items."""
        # Get menu dict at runtime instead of module load time
        window = get_main_window()
        menu_dict = await get_context_menu(window._ui_main_window.main_menu_bar, get_all=False)

        physx_lidar_menu_dict = menu_dict.get("Create", {}).get("Sensors", {}).get("PhysX Lidar", {})
        lightbeam_menu_dict = menu_dict.get("Create", {}).get("Sensors", {}).get("LightBeam Sensor", {})

        # Collect menu items
        physx_lidar_menu_list = get_all_menu_paths(physx_lidar_menu_dict, root_path=PHYSX_LIDAR_ROOT_PATH)
        lightbeam_menu_list = get_all_menu_paths(lightbeam_menu_dict, root_path=LIGHTBEAM_ROOT_PATH)
        sensor_menu_list = physx_lidar_menu_list + lightbeam_menu_list

        self.assertGreater(len(sensor_menu_list), 0, "No menu items found in PhysX Lidar or LightBeam Sensor menus")

        for test_path in sensor_menu_list:
            with self.subTest(menu_path=test_path):
                if test_path.startswith(PHYSX_LIDAR_ROOT_PATH):
                    await self._test_physx_lidar_option(test_path)
                else:
                    await self._test_lightbeam_option(test_path)
                # Reset stage for next iteration
                await self.new_stage()
