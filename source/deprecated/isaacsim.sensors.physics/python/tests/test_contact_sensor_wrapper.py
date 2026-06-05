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

"""Test contact sensor wrapper functionality."""

import asyncio

import omni.kit.test
import omni.timeline
from isaacsim.core.experimental.objects import Cube, GroundPlane
from isaacsim.core.experimental.prims import GeomPrim, RigidPrim
from isaacsim.core.experimental.utils.stage import create_new_stage_async
from isaacsim.core.simulation_manager import SimulationManager
from isaacsim.sensors.physics import ContactSensor

from .common import reset_timeline


class TestContactSensorWrapper(omni.kit.test.AsyncTestCase):
    """Test contact sensor wrapper."""

    # Before running each test
    async def setUp(self) -> None:
        """Set up test fixtures."""
        await create_new_stage_async()
        SimulationManager.setup_simulation(dt=1.0 / 60.0)
        self._timeline = omni.timeline.get_timeline_interface()
        GroundPlane("/World/defaultGroundPlane", positions=[0.0, 0.0, 0.0])
        Cube(
            "/World/new_cube_2",
            sizes=1.0,
            positions=[0.0, 0.0, 1.0],
            scales=[0.6, 0.5, 0.2],
        )
        GeomPrim("/World/new_cube_2", apply_collision_apis=True)
        RigidPrim("/World/new_cube_2", masses=[1.0])
        self._contact_sensor = ContactSensor(
            prim_path="/World/new_cube_2/contact_sensor",
            name="ant_contact_sensor",
            min_threshold=0,
            max_threshold=10000000,
            radius=-1,
        )
        await reset_timeline(self._timeline, steps=1)
        return

    # After running each test
    async def tearDown(self) -> None:
        """Tear down test fixtures."""
        if self._timeline.is_playing():
            self._timeline.stop()
        SimulationManager.invalidate_physics()
        await omni.kit.app.get_app().next_update_async()
        while omni.usd.get_context().get_stage_loading_status()[2] > 0:
            # print("tearDown, assets still loading, waiting to finish...")
            await asyncio.sleep(1.0)
        await omni.kit.app.get_app().next_update_async()
        return

    async def test_data_acquisition(self) -> None:
        """Verify current frame contains expected fields and optional contacts."""
        await omni.kit.app.get_app().next_update_async()
        await omni.kit.app.get_app().next_update_async()
        data = self._contact_sensor.get_current_frame()
        for key in ["time", "physics_step", "in_contact", "force", "number_of_contacts"]:
            self.assertTrue(key in data)
        await omni.kit.app.get_app().next_update_async()
        await omni.kit.app.get_app().next_update_async()
        await omni.kit.app.get_app().next_update_async()
        await omni.kit.app.get_app().next_update_async()
        self.assertTrue("contacts" not in data)
        self._contact_sensor.add_raw_contact_data_to_frame()
        await omni.kit.app.get_app().next_update_async()
        await omni.kit.app.get_app().next_update_async()
        self.assertTrue("contacts" in data)
        self._contact_sensor.remove_raw_contact_data_from_frame()
        await omni.kit.app.get_app().next_update_async()
        await omni.kit.app.get_app().next_update_async()
        self.assertTrue("contacts" not in data)
        return

    async def test_pause_resume(self) -> None:
        """Verify pause/resume freezes and resumes frame updates."""
        await omni.kit.app.get_app().next_update_async()
        await omni.kit.app.get_app().next_update_async()
        data = self._contact_sensor.get_current_frame()
        current_time = data["time"]
        current_step = data["physics_step"]
        self._contact_sensor.pause()
        await omni.kit.app.get_app().next_update_async()
        await omni.kit.app.get_app().next_update_async()
        await omni.kit.app.get_app().next_update_async()
        await omni.kit.app.get_app().next_update_async()
        data = self._contact_sensor.get_current_frame()
        self.assertTrue(data["time"] == current_time)
        self.assertTrue(data["physics_step"] == current_step)
        self.assertTrue(self._contact_sensor.is_paused())
        current_time = data["time"]
        current_step = data["physics_step"]
        self._contact_sensor.resume()
        await omni.kit.app.get_app().next_update_async()
        data = self._contact_sensor.get_current_frame()
        self.assertTrue(data["time"] != current_time)
        self.assertTrue(data["physics_step"] != current_step)
        await reset_timeline(self._timeline, steps=1)
        data = self._contact_sensor.get_current_frame()
        self.assertEqual(data["physics_step"], 3)
        self.assertAlmostEqual(data["time"], 0.03333, delta=0.01)
        return

    async def test_properties(self) -> None:
        """Verify contact sensor property setters update the underlying values."""
        self._contact_sensor.set_frequency(20)
        self.assertAlmostEqual(20, self._contact_sensor.get_frequency(), delta=2)
        self._contact_sensor.set_dt(0.2)
        self.assertAlmostEqual(0.2, self._contact_sensor.get_dt(), delta=0.01)
        self._contact_sensor.set_radius(0.1)
        self.assertAlmostEqual(0.1, self._contact_sensor.get_radius(), delta=0.01)
        self._contact_sensor.set_min_threshold(0.1)
        self.assertAlmostEqual(0.1, self._contact_sensor.get_min_threshold(), delta=0.01)
        self._contact_sensor.set_max_threshold(100000)
        self.assertAlmostEqual(100000, self._contact_sensor.get_max_threshold(), delta=0.01)
        return
