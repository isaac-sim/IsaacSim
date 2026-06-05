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

"""Test rtx context menu functionality."""

import carb
import omni.kit.app
import omni.kit.ui_test as ui_test
import omni.usd
from isaacsim.core.experimental.utils.stage import create_new_stage
from isaacsim.sensors.experimental.rtx import SUPPORTED_LIDAR_CONFIGS
from isaacsim.test.utils import MenuUITestCase, count_menu_items, get_all_menu_paths

# Known issue: omni.kit.ui_test.select_context_menu has a bug where it cannot correctly
# click items in certain vendor submenus. The click position appears to be calculated
# incorrectly for these specific submenus, causing the onclick_fn callback to not be invoked.
# This affects entire vendor categories (HESAI, Ouster, ZVISION) regardless of offset adjustment.
# Additionally, SICK/TIM781 and SICK/picoScan100 have a USD configuration issue where no
# OmniLidar prim is created.
# These sensors are skipped until the UI test framework bug is resolved.
KNOWN_UI_TEST_FAILURES = {
    "HESAI/XT32 SD10",
    "Ouster/OS0",
    "Ouster/OS1",
    "Ouster/OS2",
    "Ouster/VLS 128",
    "ZVISION/ML30S",
    "ZVISION/MLXS",
    "SICK/TIM781",
    "SICK/picoScan100",
}


class TestRTXContextMenu(MenuUITestCase):
    """Test r t x context menu."""

    async def setUp(self):
        """Set up test fixtures."""
        await super().setUp()
        self.carb_settings = carb.settings.get_settings()
        self.carb_settings.set("/rtx/rendermode", "RealTimePathTracing")
        self.carb_settings.set("/rtx-transient/resourcemanager/enableTextureStreaming", False)

    async def test_rtx_sensors_context_menu_count(self):
        """Test all the RTX sensors are added to context menus correctly."""
        viewport_context_menu = await self.get_viewport_context_menu()
        self.assertIsNotNone(viewport_context_menu, "Failed to get viewport context menu")

        rtx_viewport_menu_dict = viewport_context_menu["Create"]["Isaac"]["Sensors"]["RTX Lidar"]
        n_items_viewport_menu = count_menu_items(rtx_viewport_menu_dict)
        n_configs = len(SUPPORTED_LIDAR_CONFIGS)

        self.assertEqual(
            n_items_viewport_menu,
            n_configs,
            f"There are {n_items_viewport_menu} items in the viewport context menu, expected {n_configs}.",
        )

    async def test_rtx_sensors_context_menu_click(self):
        """Test the RTX sensors are created correctly via context menu.

        Note: Some sensors are skipped due to a known bug in omni.kit.ui_test.select_context_menu.
        See KNOWN_UI_TEST_FAILURES for the list of affected sensors.
        """
        viewport_context_menu = await self.get_viewport_context_menu()
        self.assertIsNotNone(viewport_context_menu, "Failed to get viewport context menu")

        rtx_viewport_menu_dict = viewport_context_menu["Create"]["Isaac"]["Sensors"]["RTX Lidar"]
        all_sensor_paths = get_all_menu_paths(rtx_viewport_menu_dict)

        testable_paths = [p for p in all_sensor_paths if p not in KNOWN_UI_TEST_FAILURES]
        skipped_count = len(all_sensor_paths) - len(testable_paths)

        carb.log_info(
            f"Testing {len(testable_paths)} RTX Lidar sensors from context menu (skipping {skipped_count} due to known UI test issues)"
        )

        failures = []

        for test_path in testable_paths:
            full_test_path = "Create/Isaac/Sensors/RTX Lidar/" + test_path
            carb.log_info(f"Testing sensor: {full_test_path}")

            create_new_stage()
            await self.wait_n_frames(2)

            await self.get_viewport_context_menu()
            await ui_test.select_context_menu(full_test_path, offset=ui_test.Vec2(10, 10))

            await self.wait_for_stage_loading()
            await self.wait_n_frames(50)

            stage = omni.usd.get_context().get_stage()
            n_lidars = sum(1 for prim in stage.TraverseAll() if prim.GetTypeName() == "OmniLidar")

            if n_lidars != 1:
                failures.append(f"{full_test_path}: found {n_lidars} OmniLidar prims, expected 1")

        if failures:
            failure_msg = "The following sensors failed to create OmniLidar prims:\n" + "\n".join(failures)
            self.fail(failure_msg)
