# SPDX-FileCopyrightText: Copyright (c) 2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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
"""
Test file for RTX sensors accessed via context menus.
This test file simulates context menu clicks (instead of using menu_click on the main menu)
to create RTX sensors and then verifies that a sensor prim is created on stage.
"""

import asyncio
import os
import random
import re
from pathlib import Path

import carb
import omni.kit.app
import omni.kit.commands
import omni.kit.test
import omni.kit.ui_test as ui_test
import omni.timeline
import omni.usd
from isaacsim.core.utils.stage import clear_stage, create_new_stage
from isaacsim.sensors.rtx import SUPPORTED_LIDAR_CONFIGS
from omni.ui.tests.test_base import OmniUiTest


class TestRTXContextMenu(OmniUiTest):
    async def setUp(self):
        # Create a new stage and acquire the RTX (lidar) sensor interface
        self._timeline = omni.timeline.get_timeline_interface()
        result = create_new_stage()
        self.assertTrue(result)
        await omni.kit.app.get_app().next_update_async()
        self.carb_settings = carb.settings.get_settings()
        self.carb_settings.set("/rtx/rendermode", "RaytracedLighting")
        self.carb_settings.set("/rtx-transient/resourcemanager/enableTextureStreaming", False)

    async def tearDown(self):
        await omni.kit.app.get_app().next_update_async()
        # Wait until stage loading finishes if necessary
        while omni.usd.get_context().get_stage_loading_status()[2] > 0:
            await omni.kit.app.get_app().next_update_async()
        await super().tearDown()

    async def test_rtx_sensors_context_menu_count(self):
        """
        Test all the RTX sensors are added to context menus correctly.
        """

        # find the viewport context menu with retry mechanism
        max_attempts = 5
        retry_delay = 0.5
        viewport_context_menu = None

        for attempt in range(max_attempts):
            try:
                viewport_window = ui_test.find("Viewport")
                await viewport_window.right_click()
                viewport_context_menu = await ui_test.get_context_menu()
                break  # Success, exit the loop
            except Exception as e:
                if attempt < max_attempts - 1:  # Don't sleep on the last attempt
                    carb.log_warn(f"Attempt {attempt+1} failed to get context menu: {str(e)}. Retrying...")
                    await omni.kit.app.get_app().next_update_async()
                    await asyncio.sleep(retry_delay)
                    retry_delay *= 1.5  # Exponential backoff
                else:
                    carb.log_error(f"Failed to get context menu after {max_attempts} attempts: {str(e)}")
                    raise  # Re-raise the last exception if all attempts failed

        self.assertIsNotNone(viewport_context_menu, "Failed to get viewport context menu")
        rtx_viewport_menu_dict = viewport_context_menu["Create"]["Isaac"]["Sensors"]["RTX Lidar"]

        # count the number of items in the sensor_menu_list is as expected
        def count_menu_items(menu_dict):
            count = 0
            for key, item in menu_dict.items():
                if isinstance(item, dict):
                    count += count_menu_items(item)
                else:
                    count += len(item)
            return count

        n_items_viewport_menu = count_menu_items(rtx_viewport_menu_dict)

        # number of sensors currently hardcoded in the extension
        n_configs = len(SUPPORTED_LIDAR_CONFIGS)

        self.assertEqual(
            n_items_viewport_menu,
            n_configs,
            f"There are {n_items_viewport_menu} items in the viewport context menu, expected {n_configs}.",
        )

    async def test_rtx_sensors_context_menu_click(self):
        """
        Test the RTX sensors are added to stage context menus correctly.
        """

        # find the path to the last layer of the menu by randomly traversing
        def get_random_menu_path(menu_dict):
            current_path = []
            current_dict = menu_dict

            # Keep traversing until we hit a leaf node (list or string)
            while isinstance(current_dict, dict):
                # Get random key from current level
                key = random.choice(list(current_dict.keys()))
                # Only add key to path if it's not "_"
                if key != "_":
                    current_path.append(key)
                # # TODO:Skip ZVISION by starting over, something werid about the ZVISION sensors, bad for testing.
                if key in ["ZVISION", "Ouster", "HESAI", "Velodyne"]:
                    return get_random_menu_path(menu_dict)
                current_dict = current_dict[key]

            # If we hit a list, choose random item from it
            if isinstance(current_dict, list):
                current_path.append(random.choice(current_dict))

            # Join path components with forward slashes
            return "/".join(current_path)

        # randomly click on few of sensors and check if the correct prim is created
        n_test_sensors = 10
        for _ in range(n_test_sensors):
            # Find the stage window widget, must redo this every time to avoid stale menu
            max_attempts = 5
            retry_delay = 0.5
            viewport_context_menu = None

            for attempt in range(max_attempts):
                try:
                    viewport_window = ui_test.find("Viewport")
                    await viewport_window.right_click()
                    viewport_context_menu = await ui_test.get_context_menu()
                    break  # Success, exit the loop
                except Exception as e:
                    if attempt < max_attempts - 1:  # Don't sleep on the last attempt
                        carb.log_warn(f"Attempt {attempt+1} failed to get context menu: {str(e)}. Retrying...")
                        await omni.kit.app.get_app().next_update_async()
                        await asyncio.sleep(retry_delay)
                        retry_delay *= 1.5  # Exponential backoff
                    else:
                        carb.log_error(f"Failed to get context menu after {max_attempts} attempts: {str(e)}")
                        raise  # Re-raise the last exception if all attempts failed

            self.assertIsNotNone(viewport_context_menu, "Failed to get viewport context menu")
            rtx_viewport_menu_dict = viewport_context_menu["Create"]["Isaac"]["Sensors"]["RTX Lidar"]

            test_path = get_random_menu_path(rtx_viewport_menu_dict)
            full_test_path = "Create/Isaac/Sensors/RTX Lidar/" + test_path

            clear_stage()
            await omni.kit.app.get_app().next_update_async()
            await omni.kit.app.get_app().next_update_async()

            await ui_test.select_context_menu(full_test_path)

            # Wait until stage loading finishes
            while omni.usd.get_context().get_stage_loading_status()[2] > 0:
                await omni.kit.app.get_app().next_update_async()

            # Give one more frame to ensure everything is settled
            for i in range(50):
                await omni.kit.app.get_app().next_update_async()

            # check if there is one and only one OmniLidar prim on stage
            stage = omni.usd.get_context().get_stage()
            prims = stage.TraverseAll()
            n_lidars = 0
            for prim in prims:
                if prim.GetTypeName() == "OmniLidar":
                    n_lidars += 1

            self.assertEqual(
                n_lidars, 1, f"There are {n_lidars} OmniLidar prims on stage for {full_test_path}, expected 1."
            )
