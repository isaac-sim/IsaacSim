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

"""Tests for the contact sensor example."""

import omni.kit.test
import omni.timeline
import omni.usd
from isaacsim.sensors.experimental.physics import Contact, ContactSensor
from isaacsim.storage.native import get_assets_root_path


class TestContactSensorExample(omni.kit.test.AsyncTestCase):
    """Verify the contact sensor example creates sensors and produces valid readings."""

    async def setUp(self) -> None:
        """Set up test fixtures."""
        self._timeline = omni.timeline.get_timeline_interface()

    async def tearDown(self) -> None:
        """Tear down test fixtures."""
        self._timeline.stop()
        await omni.kit.app.get_app().next_update_async()

    async def test_scene_creation_and_sensor_readings(self) -> None:
        """Load the ant asset, create contact sensors, and verify readings after physics steps."""
        assets_root = get_assets_root_path()
        if assets_root is None:
            self.skipTest("Assets root path not available")

        await omni.usd.get_context().open_stage_async(assets_root + "/Isaac/Robots/IsaacSim/Ant/ant_colored.usd")
        await omni.kit.app.get_app().next_update_async()

        stage = omni.usd.get_context().get_stage()
        self.assertIsNotNone(stage)

        leg_paths = [f"/Ant/Arm_{i + 1:02d}/Lower_Arm" for i in range(4)]
        for leg in leg_paths:
            sensor = ContactSensor(
                Contact.create(
                    f"{leg}/sensor",
                    min_threshold=0,
                    max_threshold=10000000,
                    color=(1, 0, 0, 1),
                    radius=0.12,
                    translations=[[40.0, 0.0, 0.0]],
                )
            )
            self.assertIsNotNone(sensor, f"Failed to create contact sensor at {leg}")

        for leg in leg_paths:
            prim = stage.GetPrimAtPath(leg + "/sensor")
            self.assertTrue(prim.IsValid(), f"Sensor prim not found at {leg}/sensor")

        self._timeline.play()
        for _ in range(20):
            await omni.kit.app.get_app().next_update_async()

        for leg in leg_paths:
            path = leg + "/sensor"
            reader = ContactSensor(path)
            reading = reader.get_sensor_reading()
            self.assertTrue(reading.is_valid, f"Contact sensor reading not valid at {path}")
