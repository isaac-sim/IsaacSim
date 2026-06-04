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

"""Test physx context menu functionality."""

import carb
import omni.kit.app
import omni.kit.ui_test as ui_test
import omni.usd
from isaacsim.core.utils.stage import clear_stage
from isaacsim.test.utils import MenuUITestCase, count_menu_items

# Known issue: omni.kit.ui_test.select_context_menu has a bug where it cannot correctly
# click items in certain submenus. The click position appears to be calculated
# incorrectly for these specific submenus, causing the onclick_fn callback to not be invoked.
# The PhysX Lidar submenu is affected by this bug. These sensors are tested via the main
# menu in test_menu.py instead.
KNOWN_UI_TEST_FAILURES = {
    "Rotating",
    "Generic",
}

# Expected PhysX sensor types and their prim type names
PHYSX_LIDAR_SENSORS = {
    "Rotating": "Lidar",
    "Generic": "Generic",
}

LIGHTBEAM_SENSORS = {
    "Generic": "IsaacLightBeamSensor",
    "Tashan TS-F-A": None,  # USD asset reference, check for Xform
}


class TestPhysxContextMenu(MenuUITestCase):
    """Test physx context menu."""

    async def test_physx_sensors_context_menu_count(self) -> None:
        """Test all the PhysX sensors are added to context menus correctly."""
        viewport_context_menu = await self.get_viewport_context_menu()
        self.assertIsNotNone(viewport_context_menu, "Failed to get viewport context menu")

        # Check PhysX Lidar menu
        physx_lidar_menu_dict = viewport_context_menu["Create"]["Isaac"]["Sensors"]["PhysX Lidar"]
        n_items_physx_lidar_menu = count_menu_items(physx_lidar_menu_dict)
        expected_physx_lidar_count = len(PHYSX_LIDAR_SENSORS)

        self.assertEqual(
            n_items_physx_lidar_menu,
            expected_physx_lidar_count,
            f"There are {n_items_physx_lidar_menu} items in the PhysX Lidar context menu, expected {expected_physx_lidar_count}.",
        )

        # Check LightBeam Sensor menu
        lightbeam_menu_dict = viewport_context_menu["Create"]["Isaac"]["Sensors"]["LightBeam Sensor"]
        n_items_lightbeam_menu = count_menu_items(lightbeam_menu_dict)
        expected_lightbeam_count = len(LIGHTBEAM_SENSORS)

        self.assertEqual(
            n_items_lightbeam_menu,
            expected_lightbeam_count,
            f"There are {n_items_lightbeam_menu} items in the LightBeam Sensor context menu, expected {expected_lightbeam_count}.",
        )

    async def test_physx_lidar_context_menu_click(self) -> None:
        """Test the PhysX Lidar sensors are created correctly via context menu.

        Note: All PhysX Lidar sensors are currently skipped due to a known bug in
        omni.kit.ui_test.select_context_menu that affects this submenu's position.
        See KNOWN_UI_TEST_FAILURES for details. These sensors are tested via the
        main menu in test_menu.py instead.
        """
        testable_sensors = {k: v for k, v in PHYSX_LIDAR_SENSORS.items() if k not in KNOWN_UI_TEST_FAILURES}
        skipped_count = len(PHYSX_LIDAR_SENSORS) - len(testable_sensors)

        if skipped_count > 0:
            carb.log_info(
                f"Skipping {skipped_count} PhysX Lidar sensors from context menu test due to known UI test issues. "
                "These sensors are tested via main menu in test_menu.py."
            )

        if not testable_sensors:
            # All sensors are in the known failures list, test passes by design
            return

        failures = []

        for sensor_name, expected_prim_type in testable_sensors.items():
            full_test_path = f"Create/Isaac/Sensors/PhysX Lidar/{sensor_name}"
            carb.log_info(f"Testing sensor: {full_test_path}")

            clear_stage()
            await self.wait_n_frames(2)

            await self.get_viewport_context_menu()
            await ui_test.select_context_menu(full_test_path, offset=ui_test.Vec2(10, 10))

            await self.wait_for_stage_loading()
            await self.wait_n_frames(50)

            stage = omni.usd.get_context().get_stage()
            n_sensors = sum(1 for prim in stage.TraverseAll() if prim.GetTypeName() == expected_prim_type)

            if n_sensors != 1:
                failures.append(f"{full_test_path}: found {n_sensors} {expected_prim_type} prims, expected 1")

        if failures:
            failure_msg = "The following PhysX Lidar sensors failed to create prims:\n" + "\n".join(failures)
            self.fail(failure_msg)

    async def test_lightbeam_context_menu_click(self) -> None:
        """Test the LightBeam sensors are created correctly via context menu."""
        failures = []

        for sensor_name, expected_prim_type in LIGHTBEAM_SENSORS.items():
            full_test_path = f"Create/Isaac/Sensors/LightBeam Sensor/{sensor_name}"
            carb.log_info(f"Testing sensor: {full_test_path}")

            clear_stage()
            await self.wait_n_frames(2)

            await self.get_viewport_context_menu()
            await ui_test.select_context_menu(full_test_path, offset=ui_test.Vec2(10, 10))

            await self.wait_for_stage_loading()
            await self.wait_n_frames(50)

            stage = omni.usd.get_context().get_stage()
            sensor_found = False

            for prim in stage.TraverseAll():
                prim_type = prim.GetTypeName()
                if expected_prim_type is not None:
                    if prim_type == expected_prim_type:
                        sensor_found = True
                        break
                else:
                    if prim_type == "Xform" and "TS" in prim.GetPath().pathString:
                        sensor_found = True
                        break

            if not sensor_found:
                failures.append(f"{full_test_path}: no sensor prim found on stage")

        if failures:
            failure_msg = "The following LightBeam sensors failed to create prims:\n" + "\n".join(failures)
            self.fail(failure_msg)
