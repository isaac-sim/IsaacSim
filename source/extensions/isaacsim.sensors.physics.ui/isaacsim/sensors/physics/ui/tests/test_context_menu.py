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

"""Tests for physics sensor context menu items."""

import omni.kit.app
import omni.kit.ui_test as ui_test
from isaacsim.core.experimental.prims import RigidPrim
from isaacsim.test.utils import MenuUITestCase
from pxr import UsdGeom

# Expected physics sensor types
PHYSICS_SENSORS = {
    "Contact Sensor": "IsaacContactSensor",
    "Imu Sensor": "IsaacImuSensor",
}


class TestPhysicsContextMenu(MenuUITestCase):
    """Validate physics sensor entries in the viewport context menu."""

    async def test_physics_sensors_context_menu_count(self) -> None:
        """Test all the physics sensors are added to context menus correctly."""
        viewport_context_menu = await self.get_viewport_context_menu()
        self.assertIsNotNone(viewport_context_menu, "Failed to get viewport context menu")

        self.assertIn("Create", viewport_context_menu, "Create menu not found in context menu")
        self.assertIn("Isaac", viewport_context_menu["Create"], "Isaac submenu not found")
        self.assertIn("Sensors", viewport_context_menu["Create"]["Isaac"], "Sensors submenu not found")

        physics_sensor_menu_dict = viewport_context_menu["Create"]["Isaac"]["Sensors"]
        sensor_items = physics_sensor_menu_dict.get("_", [])

        physics_sensor_count = sum(1 for name in PHYSICS_SENSORS if name in sensor_items)
        expected_count = len(PHYSICS_SENSORS)

        self.assertEqual(
            physics_sensor_count,
            expected_count,
            f"There are {physics_sensor_count} physics sensor items in the context menu, expected {expected_count}. "
            f"Found items: {sensor_items}",
        )

    async def test_contact_sensor_context_menu_rigid_parent_creates_sensor(self) -> None:
        """Test context menu click for Contact Sensor with RigidPrim parent creates a functional sensor."""
        UsdGeom.Xform.Define(self._stage, "/World/RigidParent")
        await omni.kit.app.get_app().next_update_async()

        rigid_parent = RigidPrim("/World/RigidParent", masses=[1.0], reset_xform_op_properties=True)
        rigid_parent.set_enabled_rigid_bodies(True)
        await omni.kit.app.get_app().next_update_async()

        self.select_prim("/World/RigidParent")
        await self.wait_n_frames(2)

        await self.get_viewport_context_menu()
        await ui_test.select_context_menu("Create/Isaac/Sensors/Contact Sensor", offset=ui_test.Vec2(10, 10))
        await self.wait_n_frames(50)
        await self.run_timeline_frames(50)

        sensor_count = self.count_prims_by_type("IsaacContactSensor")
        self.assertEqual(
            sensor_count,
            1,
            f"Contact sensor should be created with RigidPrim parent via context menu, but found {sensor_count}",
        )

    async def test_imu_sensor_context_menu_rigid_parent_creates_sensor(self) -> None:
        """Test context menu click for Imu Sensor with RigidPrim parent creates a functional sensor."""
        UsdGeom.Xform.Define(self._stage, "/World/RigidParent")
        await omni.kit.app.get_app().next_update_async()

        rigid_parent = RigidPrim("/World/RigidParent", masses=[1.0], reset_xform_op_properties=True)
        rigid_parent.set_enabled_rigid_bodies(True)
        await omni.kit.app.get_app().next_update_async()

        self.select_prim("/World/RigidParent")
        await self.wait_n_frames(2)

        await self.get_viewport_context_menu()
        await ui_test.select_context_menu("Create/Isaac/Sensors/Imu Sensor", offset=ui_test.Vec2(10, 10))
        await self.wait_n_frames(50)
        await self.run_timeline_frames(50)

        sensor_count = self.count_prims_by_type("IsaacImuSensor")
        self.assertEqual(
            sensor_count,
            1,
            f"IMU sensor should be created with RigidPrim parent via context menu, but found {sensor_count}",
        )
