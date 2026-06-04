# SPDX-FileCopyrightText: Copyright (c) 2024-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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

"""Test lightbeam sensor functionality."""

import asyncio

import numpy as np
import omni.kit.commands
import omni.kit.test
from isaacsim.core.api import World

# Import extension python module we are testing with absolute import path, as if we are external user (other extension)
from isaacsim.sensors.physx import _range_sensor
from pxr import Gf, UsdGeom, UsdPhysics


# Having a test class dervived from omni.kit.test.AsyncTestCase declared on the root of module will make it auto-discoverable by omni.kit.test
class TestLightBeamSensor(omni.kit.test.AsyncTestCase):
    """Test light beam sensor."""

    # Before running each test
    async def setUp(self) -> None:
        """Set up test fixtures."""
        self._lb = _range_sensor.acquire_lightbeam_sensor_interface()
        await omni.usd.get_context().new_stage_async()
        self._stage = omni.usd.get_context().get_stage()
        self._physics_rate = 60.0
        self.my_world = World(
            stage_units_in_meters=1.0, physics_dt=1.0 / self._physics_rate, rendering_dt=1.0 / self._physics_rate
        )
        await self.my_world.initialize_simulation_context_async()

    # After running each test
    async def tearDown(self) -> None:
        """Tear down test fixtures."""
        self.my_world.stop()
        self.my_world.clear_instance()
        await omni.kit.app.get_app().next_update_async()
        while omni.usd.get_context().get_stage_loading_status()[2] > 0:
            print("tearDown, assets still loading, waiting to finish...")
            await asyncio.sleep(1.0)
        await omni.kit.app.get_app().next_update_async()

    async def add_cube(self, path: str, size: float, offset: object) -> UsdGeom.Cube:
        """Perform add cube operation.

        Args:
            path: USD path for the cube prim.
            size: Size of the cube.
            offset: Translation offset for the cube.

        Returns:
            The created cube geometry object.
        """
        cubeGeom = UsdGeom.Cube.Define(self._stage, path)
        cubePrim = self._stage.GetPrimAtPath(path)

        cubeGeom.CreateSizeAttr(size)
        cubeGeom.AddTranslateOp().Set(offset)
        await omni.kit.app.get_app().next_update_async()  # Need this to avoid flatcache errors
        UsdPhysics.CollisionAPI.Apply(cubePrim)  # no physics, only collision
        return cubeGeom

    # Tests a light beam sensor with a cube in front of it breaking the beam
    async def test_basic_lightbeam_sensor(self) -> None:
        """Test basic lightbeam sensor."""
        # Add a cube
        cubePath = "/World/Cube"
        await self.add_cube(cubePath, 1.000, Gf.Vec3f(0.8, -0.5, 0.0))

        # Add sensor
        result, sensor = omni.kit.commands.execute(
            "IsaacSensorCreateLightBeamSensor",
            path="/LightBeam_Sensor",
            parent=None,
            min_range=0.4,
            max_range=100.0,
            forward_axis=Gf.Vec3d(1, 0, 0),
        )
        self.assertTrue(result)
        self.assertIsNotNone(sensor)

        sensorPath = str(sensor.GetPath())
        sensor.GetPrim().GetAttribute("xformOp:translate").Set(Gf.Vec3d(0.0, 0.0, 0.0))

        # start running the simulation
        self.my_world.play()
        await omni.kit.app.get_app().next_update_async()

        # Make sure a beam was hit
        beam_hit = self._lb.get_beam_hit_data(sensorPath)
        self.assertEqual(beam_hit.any(), True)

        # Get linear depth and check the depth:
        linear_depth = self._lb.get_linear_depth_data(sensorPath)
        n_length = np.size(linear_depth)
        self.assertAlmostEqual(linear_depth[0], 0.4)
        self.assertEqual(n_length, 1)

    async def test_lightbeam_curtain(self) -> None:
        """Test lightbeam curtain."""
        # Add a cube
        cubePath = "/World/Cube"
        await self.add_cube(cubePath, 1.000, Gf.Vec3f(1.5, -0.5, 0.0))

        # Add sensor
        result, sensor = omni.kit.commands.execute(
            "IsaacSensorCreateLightBeamSensor",
            path="/LightBeam_Sensor",
            parent=None,
            num_rays=5,
            curtain_length=0.5,
            min_range=0.4,
            max_range=100.0,
            forward_axis=Gf.Vec3d(1, 0, 0),
            curtain_axis=Gf.Vec3d(0, 0, 1),
        )
        self.assertTrue(result)
        self.assertIsNotNone(sensor)

        sensorPath = str(sensor.GetPath())
        sensor.GetPrim().GetAttribute("xformOp:translate").Set(Gf.Vec3d(0.0, 0.0, 0.0))

        # start running the simulation
        self.my_world.play()
        await omni.kit.app.get_app().next_update_async()
        await omni.kit.app.get_app().next_update_async()

        # Make sure a beam was hit
        beam_hit = self._lb.get_beam_hit_data(sensorPath)
        self.assertEqual(beam_hit.any(), True)

        # Get linear depth and check the depth (cube should hit multiple rays):
        linear_depth = self._lb.get_linear_depth_data(sensorPath)
        n_length = np.size(linear_depth)
        self.assertEqual(n_length, 5)
        # all rays should hit
        for i in range(n_length):
            self.assertAlmostEqual(linear_depth[i], 1.0)
