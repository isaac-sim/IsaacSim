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

"""Tests for physics sensor main menu items."""

import omni.kit.app
from isaacsim.core.experimental.prims import RigidPrim
from isaacsim.test.utils import MenuUITestCase
from omni.kit.mainwindow import get_main_window
from omni.kit.ui_test import get_context_menu
from pxr import UsdGeom

# Expected physics sensors and their expected prim types
PHYSICS_SENSORS = {
    "Contact Sensor": "IsaacContactSensor",
    "Imu Sensor": "IsaacImuSensor",
}


class TestPhysicsMenuAssets(MenuUITestCase):
    """Validate physics sensor entries in the main menu bar."""

    async def test_physics_sensors_menu_count(self) -> None:
        """Test all the physics sensors are added to main menus correctly."""
        window = get_main_window()
        menu_dict = await get_context_menu(window._ui_main_window.main_menu_bar, get_all=False)

        self.assertIn("Create", menu_dict, "Create menu not found in menu bar")
        create_menu = menu_dict["Create"]
        self.assertIn("Sensors", create_menu, "Sensors submenu not found in Create menu")

        sensors_menu = create_menu["Sensors"]
        sensor_items = sensors_menu.get("_", [])

        found_sensors = [name for name in PHYSICS_SENSORS if name in sensor_items]
        expected_count = len(PHYSICS_SENSORS)

        self.assertEqual(
            len(found_sensors),
            expected_count,
            f"Found {len(found_sensors)} physics sensors in menu, expected {expected_count}. "
            f"Found: {found_sensors}, Expected: {list(PHYSICS_SENSORS.keys())}",
        )

    async def test_contact_sensor_menu_rigid_parent_creates_sensor(self) -> None:
        """Test clicking Contact Sensor menu with RigidPrim parent creates a functional sensor."""
        UsdGeom.Xform.Define(self._stage, "/World/RigidParent")
        await omni.kit.app.get_app().next_update_async()

        rigid_parent = RigidPrim("/World/RigidParent", masses=[1.0], reset_xform_op_properties=True)
        rigid_parent.set_enabled_rigid_bodies(True)
        await omni.kit.app.get_app().next_update_async()

        self.select_prim("/World/RigidParent")
        await self.wait_n_frames(2)

        await self.menu_click_with_retry("Create/Sensors/Contact Sensor")
        await self.run_timeline_frames(50)

        sensor_count = self.count_prims_by_type("IsaacContactSensor")
        self.assertEqual(
            sensor_count,
            1,
            f"Contact sensor should be created with RigidPrim parent via menu click, but found {sensor_count}",
        )

    async def test_imu_sensor_menu_rigid_parent_creates_sensor(self) -> None:
        """Test clicking Imu Sensor menu with RigidPrim parent creates a functional sensor."""
        UsdGeom.Xform.Define(self._stage, "/World/RigidParent")
        await omni.kit.app.get_app().next_update_async()

        rigid_parent = RigidPrim("/World/RigidParent", masses=[1.0], reset_xform_op_properties=True)
        rigid_parent.set_enabled_rigid_bodies(True)
        await omni.kit.app.get_app().next_update_async()

        self.select_prim("/World/RigidParent")
        await self.wait_n_frames(2)

        await self.menu_click_with_retry("Create/Sensors/Imu Sensor")
        await self.run_timeline_frames(50)

        sensor_count = self.count_prims_by_type("IsaacImuSensor")
        self.assertEqual(
            sensor_count,
            1,
            f"IMU sensor should be created with RigidPrim parent via menu click, but found {sensor_count}",
        )
