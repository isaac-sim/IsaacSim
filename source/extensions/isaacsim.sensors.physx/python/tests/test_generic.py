# SPDX-FileCopyrightText: Copyright (c) 2018-2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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

import carb
import numpy as np
import omni.kit.commands
import omni.kit.test

# Import extension python module we are testing with absolute import path, as if we are external user (other extension)
from isaacsim.sensors.physx import _range_sensor
from pxr import Gf, Sdf, UsdGeom, UsdLux, UsdPhysics


# Having a test class dervived from omni.kit.test.AsyncTestCase declared on the root of module will make it auto-discoverable by omni.kit.test
class TestGeneric(omni.kit.test.AsyncTestCase):
    # Before running each test
    async def setUp(self):
        self._sensor = _range_sensor.acquire_generic_sensor_interface()
        self._timeline = omni.timeline.get_timeline_interface()
        await omni.usd.get_context().new_stage_async()
        self._stage = omni.usd.get_context().get_stage()

        # light
        sphereLight = UsdLux.SphereLight.Define(self._stage, Sdf.Path("/World/SphereLight"))
        sphereLight.CreateRadiusAttr(150)
        sphereLight.CreateIntensityAttr(30000)
        sphereLight.AddTranslateOp().Set(Gf.Vec3f(650.0, 0.0, 1150.0))

        # set up axis to z
        UsdGeom.SetStageUpAxis(self._stage, UsdGeom.Tokens.z)
        UsdGeom.SetStageMetersPerUnit(self._stage, 1.0)

        # Physics scene
        scene = UsdPhysics.Scene.Define(self._stage, Sdf.Path("/World/physicsScene"))
        scene.CreateGravityDirectionAttr().Set(Gf.Vec3f(0.0, 0.0, -1.0))
        scene.CreateGravityMagnitudeAttr().Set(9.81)

        rendering_hz = 60.0
        carb.settings.get_settings().set_bool("/app/runLoops/main/rateLimitEnabled", True)
        carb.settings.get_settings().set_float("/app/runLoops/main/rateLimitFrequency", rendering_hz)
        self._timeline.set_target_framerate(rendering_hz)

    # After running each test
    async def tearDown(self):
        self._timeline.stop()
        pass

    async def add_cube(self, path, size, offset):

        cubeGeom = UsdGeom.Cube.Define(self._stage, path)
        cubePrim = self._stage.GetPrimAtPath(path)

        cubeGeom.CreateSizeAttr(size)
        cubeGeom.AddTranslateOp().Set(offset)
        await omni.kit.app.get_app().next_update_async()  # Need this to avoid flatcache errors
        UsdPhysics.CollisionAPI.Apply(cubePrim)  # no physics, only collision
        return cubeGeom

    # Tests a static sensor with a cube in front of it
    async def test_set_pattern_generic(self):

        # Add a cube
        cubePath = "/World/Cube"
        await self.add_cube(cubePath, 1.000, Gf.Vec3f(-2.000, 0.0, 0.500))

        # Add sensor
        result, sensor = omni.kit.commands.execute(
            "RangeSensorCreateGeneric",
            path="/World/Generic",
            parent=None,
            min_range=0.4,
            max_range=100.0,
            draw_points=True,
            draw_lines=True,
            sampling_rate=1e4,
        )
        sensorPath = str(sensor.GetPath())
        sensor.GetPrim().GetAttribute("xformOp:translate").Set(Gf.Vec3d(0.0, 0.0, 0.250))

        # try a few patterns: purely azimuth, purely zenith, some version of 45 deg angle for both, with offsets.
        # create a sensor pattern: 4 rays forming a cross, one of them should hit the cube, the others shouldn't
        sampling_rate = 1e5
        azimuth_angles = [0, np.pi / 2, np.pi, 1.5 * np.pi]
        zenith_angles = [0, 0, 0, 0]
        sensor_pattern_single = np.stack((azimuth_angles, zenith_angles))
        sensor_pattern = np.tile(sensor_pattern_single, (1, int(sampling_rate / 4) * 2))  ## give two seconds of data

        # start running the simulation
        self._timeline.play()
        steps_per_sec = 60
        seconds = 1
        for frame in range(int(steps_per_sec * seconds)):
            await omni.kit.app.get_app().next_update_async()
            # set the pattern while running
            if self._sensor.send_next_batch(sensorPath):
                self._sensor.set_next_batch_rays(sensorPath, sensor_pattern)
        self._timeline.pause()
        await omni.kit.app.get_app().next_update_async()
        await omni.kit.app.get_app().next_update_async()
        # # Get linear depth and check the depth:
        # # for randomly selected 4 continuous terms, one of them should be a hit, the others three should be misses
        linear_depth = self._sensor.get_linear_depth_data(sensorPath)
        self.assertGreater(np.size(linear_depth), 5)
        n_length = np.size(linear_depth)
        random_idx = np.random.randint(0, n_length - 5)
        random_segment = linear_depth[random_idx : (random_idx + 4)]
        self.assertEqual(np.size(np.where(random_segment == 1.5)), 1)
        self.assertEqual(np.size(np.where(random_segment == 100)), 3)

    # Tests a static sensor with a cube in front of it
    async def test_offset_generic(self):

        # Add a cube
        cubePath = "/World/Cube"
        await self.add_cube(cubePath, 1.000, Gf.Vec3f(-2.000, 0.0, 0.500))

        # Add sensor
        result, sensor = omni.kit.commands.execute(
            "RangeSensorCreateGeneric",
            path="/World/Generic",
            parent=None,
            min_range=0.4,
            max_range=100.0,
            draw_points=True,
            draw_lines=True,
            sampling_rate=1e4,
        )
        sensorPath = str(sensor.GetPath())
        sensor.GetPrim().GetAttribute("xformOp:translate").Set(Gf.Vec3d(0.0, 0.0, 0.250))

        # try a few patterns: purely azimuth, purely zenith, some version of 45 deg angle for both, with offsets.
        # create a sensor pattern: 4 rays forming a cross, one of them should hit the cube, the others shouldn't
        sampling_rate = 1e5
        azimuth_angles = [0, np.pi / 2, np.pi, 1.5 * np.pi]
        zenith_angles = [0, 0, 0, 0]
        sensor_pattern_single = np.stack((azimuth_angles, zenith_angles))
        sensor_pattern = np.tile(sensor_pattern_single, (1, int(sampling_rate / 4) * 2))  ## give two seconds of data
        # add offset to the pattern
        L = np.shape(sensor_pattern)[1]
        origin_offset = np.tile(np.array([0, 0, -100]), (L, 1))  ## offset in z to pass the cube height

        # start running the simulation
        self._timeline.play()
        steps_per_sec = 60
        seconds = 1
        for frame in range(int(steps_per_sec * seconds)):
            await omni.kit.app.get_app().next_update_async()
            # set the pattern while running
            if self._sensor.send_next_batch(sensorPath):
                self._sensor.set_next_batch_rays(sensorPath, sensor_pattern)
                self._sensor.set_next_batch_offsets(sensorPath, origin_offset)
        self._timeline.pause()

        ## check intensity this time. all of them should be 0
        intensity = self._sensor.get_intensity_data(sensorPath)
        self.assertEqual(sum(intensity), 0)
