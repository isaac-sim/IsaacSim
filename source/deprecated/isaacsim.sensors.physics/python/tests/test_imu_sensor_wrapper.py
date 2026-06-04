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

"""Test imu sensor wrapper functionality."""

import asyncio

import numpy as np
import omni.kit.test
import omni.timeline
from isaacsim.core.experimental.objects import Cube, GroundPlane
from isaacsim.core.experimental.prims import GeomPrim, RigidPrim, XformPrim
from isaacsim.core.experimental.utils import prim as prim_utils
from isaacsim.core.experimental.utils import stage as stage_utils
from isaacsim.core.simulation_manager import SimulationManager
from isaacsim.sensors.physics import IMUSensor
from isaacsim.storage.native import get_assets_root_path_async

from .common import EARTH_GRAVITY, GRAVITY_TOLERANCE, ORIENTATION_TOLERANCE, SMALL_TOLERANCE, reset_timeline


class TestIMU(omni.kit.test.AsyncTestCase):
    """Test i m u."""

    # Before running each test
    async def setUp(self) -> None:
        """Set up test fixtures."""
        await stage_utils.create_new_stage_async()
        SimulationManager.setup_simulation(dt=1.0 / 60.0)
        self._timeline = omni.timeline.get_timeline_interface()
        GroundPlane("/World/defaultGroundPlane", positions=[0.0, 0.0, 0.0])
        assets_root_path = await get_assets_root_path_async()
        asset_path = assets_root_path + "/Isaac/Robots/NVIDIA/NovaCarter/nova_carter.usd"
        stage_utils.add_reference_to_stage(usd_path=asset_path, path="/World/Carter")

        XformPrim("/World/Carter", positions=[0, 0.0, 0.5], reset_xform_op_properties=True)

        self._imu = IMUSensor(prim_path="/World/Carter/chassis_link/Imu_Sensor", name="imu")

        Cube("/World/cube", sizes=1.0, positions=[2.0, 2.0, 2.5], scales=[20.0, 0.2, 5.0])
        GeomPrim("/World/cube", apply_collision_apis=True)
        RigidPrim("/World/cube", masses=[1.0])

        Cube("/World/cube_2", sizes=1.0, positions=[2.0, -2.0, 2.5], scales=[20.0, 0.2, 5.0])
        GeomPrim("/World/cube_2", apply_collision_apis=True)
        RigidPrim("/World/cube_2", masses=[1.0])

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
        """Test data acquisition."""
        await omni.kit.app.get_app().next_update_async()
        await omni.kit.app.get_app().next_update_async()
        data = self._imu.get_current_frame()
        for key in ["time", "physics_step", "lin_acc", "ang_vel", "orientation", "is_valid"]:
            self.assertTrue(key in data)
        self.assertIsInstance(data["is_valid"], bool)
        data = self._imu.get_current_frame(read_gravity=False)
        for key in ["time", "physics_step", "lin_acc", "ang_vel", "orientation", "is_valid"]:
            self.assertTrue(key in data)
        self.assertIsInstance(data["is_valid"], bool)
        return

    async def test_data_values_gravity_toggle(self) -> None:
        """Test data values gravity toggle."""
        await reset_timeline(self._timeline, steps=2)
        data = None
        for _ in range(60):
            data = self._imu.get_current_frame()
            if abs(float(data["lin_acc"][2]) - EARTH_GRAVITY) <= GRAVITY_TOLERANCE:
                break
            await omni.kit.app.get_app().next_update_async()
        self.assertIsNotNone(data)
        self.assertGreater(data["time"], 0.0)
        self.assertTrue(data["is_valid"])
        self.assertAlmostEqual(float(data["lin_acc"][2]), EARTH_GRAVITY, delta=GRAVITY_TOLERANCE)

        data_no_gravity = self._imu.get_current_frame(read_gravity=False)
        self.assertAlmostEqual(float(data_no_gravity["lin_acc"][2]), 0.0, delta=GRAVITY_TOLERANCE)

        orientation_norm = float(np.linalg.norm(data["orientation"]))
        self.assertAlmostEqual(orientation_norm, 1.0, delta=ORIENTATION_TOLERANCE)

    async def test_pause_resume(self) -> None:
        """Test pause resume."""
        await omni.kit.app.get_app().next_update_async()
        await omni.kit.app.get_app().next_update_async()
        data = self._imu.get_current_frame()
        current_time = data["time"]
        current_step = data["physics_step"]
        self._imu.pause()
        await omni.kit.app.get_app().next_update_async()
        await omni.kit.app.get_app().next_update_async()
        await omni.kit.app.get_app().next_update_async()
        await omni.kit.app.get_app().next_update_async()
        data = self._imu.get_current_frame()
        self.assertTrue(data["time"] == current_time)
        self.assertTrue(data["physics_step"] == current_step)
        self.assertTrue(self._imu.is_paused())
        current_time = data["time"]
        current_step = data["physics_step"]
        self._imu.resume()
        await omni.kit.app.get_app().next_update_async()
        data = self._imu.get_current_frame()
        self.assertTrue(data["time"] != current_time)
        self.assertTrue(data["physics_step"] != current_step)
        await reset_timeline(self._timeline, steps=1)
        data = self._imu.get_current_frame()
        self.assertAlmostEqual(data["time"], 0.033333, delta=0.0001)

        self.assertTrue(data["physics_step"] == 3)
        return

    async def test_properties(self) -> None:
        """Test properties."""
        self._imu.set_frequency(20)
        self.assertAlmostEqual(20, self._imu.get_frequency(), delta=2)
        self._imu.set_dt(0.2)
        self.assertAlmostEqual(0.2, self._imu.get_dt(), delta=SMALL_TOLERANCE)
        return

    async def test_filter_size_parameters(self) -> None:
        """Test filter size parameters."""
        filter_imu = IMUSensor(
            prim_path="/World/Carter/chassis_link/Imu_Sensor_filtered",
            name="imu_filtered",
            linear_acceleration_filter_size=5,
            angular_velocity_filter_size=7,
            orientation_filter_size=9,
        )
        imu_prim = prim_utils.get_prim_at_path(filter_imu.prim_path)
        self.assertEqual(imu_prim.GetAttribute("linearAccelerationFilterWidth").Get(), 5)
        self.assertEqual(imu_prim.GetAttribute("angularVelocityFilterWidth").Get(), 7)
        self.assertEqual(imu_prim.GetAttribute("orientationFilterWidth").Get(), 9)
