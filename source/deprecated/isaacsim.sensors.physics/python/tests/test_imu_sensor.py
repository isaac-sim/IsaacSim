# SPDX-FileCopyrightText: Copyright (c) 2018-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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

"""Test imu sensor functionality."""

# NOTE:
#   omni.kit.test - std python's unittest module with additional wrapping to add suport for async/await tests
#   For most things refer to unittest docs: https://docs.python.org/3/library/unittest.html

import asyncio
import math

import carb.tokens
import isaacsim.core.experimental.utils.prim as prim_utils
import isaacsim.core.experimental.utils.stage as stage_utils
import isaacsim.core.experimental.utils.transform as transform_utils
import numpy as np
import omni.kit.commands
import omni.kit.test
import omni.timeline
from isaacsim.core.experimental.objects import Cube
from isaacsim.core.experimental.prims import Articulation, GeomPrim, RigidPrim, XformPrim
from isaacsim.core.simulation_manager import SimulationManager
from isaacsim.sensors.physics import _sensor
from isaacsim.storage.native import get_assets_root_path_async
from pxr import Gf, UsdGeom

from .common import (
    ANGLE_TOLERANCE_DEG,
    ANGULAR_VEL_TOLERANCE,
    CM_GRAVITY,
    EARTH_GRAVITY,
    GRAVITY_TOLERANCE,
    MOON_GRAVITY,
    ORIENTATION_TOLERANCE,
    setup_ant_scene,
    step_simulation,
)


# Having a test class dervived from omni.kit.test.AsyncTestCase declared on the root of module will make it auto-discoverable by omni.kit.test
class TestIMUSensor(omni.kit.test.AsyncTestCase):
    """Test i m u sensor."""

    # Before running each test
    async def setUp(self) -> None:
        """Set up test fixtures."""
        self._sensor_rate = 60
        self._is = _sensor.acquire_imu_sensor_interface()
        self._assets_root_path = await get_assets_root_path_async()
        if self._assets_root_path is None:
            carb.log_error("Could not find Isaac Sim assets folder")
            return
        self._timeline = omni.timeline.get_timeline_interface()
        self._ant_config = None

    async def _setup_ant(self, physics_rate: int = 60) -> None:
        """Load the ant scene and configure ant-specific test data.

        Args:
            physics_rate: Physics simulation rate in Hz.
        """
        self._ant_config = await setup_ant_scene(physics_rate)
        self._stage = stage_utils.get_current_stage()
        await omni.kit.app.get_app().next_update_async()
        self.ant = XformPrim("/Ant", reset_xform_op_properties=True)

    # Convenience properties for ant configuration
    @property
    def leg_paths(self) -> list:
        """Return leg paths."""
        return self._ant_config.leg_paths

    @property
    def sphere_path(self) -> str:
        """Perform sphere path operation."""
        return self._ant_config.sphere_path

    @property
    def sensor_offsets(self) -> list:
        """Return sensor offsets."""
        return self._ant_config.imu_sensor_offsets

    @property
    def sensor_quatd(self) -> list:
        """Perform sensor quatd operation."""
        return self._ant_config.sensor_quatd

    async def _setup_simple_articulation(self, physics_rate: int = 60) -> None:
        """Load the simple articulation scene for articulation-based tests.

        Args:
            physics_rate: Physics simulation rate in Hz.
        """
        self.pivot_path = "/Articulation/CenterPivot"
        self.slider_path = "/Articulation/Slider"
        self.arm_path = "/Articulation/Arm"

        # load nucleus asset
        await stage_utils.open_stage_async(
            self._assets_root_path + "/Isaac/Robots/IsaacSim/SimpleArticulation/simple_articulation.usd"
        )

        await omni.kit.app.get_app().next_update_async()
        await omni.kit.app.get_app().next_update_async()
        self._stage = stage_utils.get_current_stage()
        stage_utils.set_stage_units(meters_per_unit=1.0)
        SimulationManager.setup_simulation(dt=1.0 / physics_rate)

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

    async def _add_sensor_prims(self) -> None:
        """Helper to add IMU sensors to ant legs and sphere. Requires ant to be loaded."""
        for i in range(4):
            await omni.kit.app.get_app().next_update_async()
            result, sensor = omni.kit.commands.execute(
                "IsaacSensorCreateImuSensor",
                path="/sensor",
                parent=self.leg_paths[i],
                sensor_period=1 / self._sensor_rate,
                translation=self.sensor_offsets[i],
                orientation=self.sensor_quatd[i],
            )
            self.assertTrue(result)
            self.assertIsNotNone(sensor)
            # Add sensor on body sphere
            await omni.kit.app.get_app().next_update_async()
            result, sensor = omni.kit.commands.execute(
                "IsaacSensorCreateImuSensor",
                path="/sensor",
                parent=self.sphere_path,
                sensor_period=1 / self._sensor_rate,
                translation=self.sensor_offsets[4],
                orientation=self.sensor_quatd[4],
            )
            self.assertTrue(result)
            self.assertIsNotNone(sensor)

    async def test_add_sensor_prim(self) -> None:
        """Test add sensor prim."""
        await self._setup_ant()
        await self._add_sensor_prims()

    async def test_orientation_imu(self) -> None:
        """Test orientation imu."""
        await self._setup_simple_articulation()

        result, sensor = omni.kit.commands.execute(
            "IsaacSensorCreateImuSensor",
            path="/arm_imu",
            parent=self.arm_path,
            sensor_period=1 / self._sensor_rate,
            translation=Gf.Vec3d(0, 0, 0),
            orientation=Gf.Quatd(1, 0, 0, 0),
        )
        self.assertTrue(result)
        self.assertIsNotNone(sensor)

        self._timeline.play()

        await omni.kit.app.get_app().next_update_async()

        art = Articulation("/Articulation")
        await omni.kit.app.get_app().next_update_async()
        art.set_dof_gains(np.ones(art.num_dofs) * 1e8, np.ones(art.num_dofs) * 1e8)

        ang = 0
        for i in range(70):
            art.set_dof_positions(np.array([math.radians(ang), 0.5]))

            await omni.kit.app.get_app().next_update_async()
            await omni.kit.app.get_app().next_update_async()

            euler = transform_utils.quaternion_to_euler_angles(
                np.array(self._is.get_sensor_reading(self.arm_path + "/arm_imu").orientation), degrees=True
            )
            orientation = euler.numpy()[0]

            angtest = ang % 360
            if ang >= 180:
                angtest = ang - 360

            self.assertAlmostEqual(orientation, angtest, delta=ANGLE_TOLERANCE_DEG)
            ang += 5

    async def test_ang_vel_imu(self) -> None:
        """Test ang vel imu."""
        await self._setup_simple_articulation()

        result, sensor = omni.kit.commands.execute(
            "IsaacSensorCreateImuSensor",
            path="/slider_imu",
            parent=self.slider_path,
            sensor_period=1 / self._sensor_rate,
            translation=Gf.Vec3d(0, 0, 0),
            orientation=Gf.Quatd(1, 0, 0, 0),
        )
        self.assertTrue(result)
        self.assertIsNotNone(sensor)

        self._timeline.play()

        await omni.kit.app.get_app().next_update_async()

        art = Articulation("/Articulation")
        await omni.kit.app.get_app().next_update_async()

        ang_vel_l = [x * 30 for x in range(0, 20)]

        for x in ang_vel_l:
            art.set_dof_velocities(np.array([math.radians(x), 0]))

            await omni.kit.app.get_app().next_update_async()
            ang_vel_z = self._is.get_sensor_reading(self.slider_path + "/slider_imu").ang_vel_z
            # with sensor frequency = physics rate, all should be the same
            self.assertAlmostEqual(ang_vel_z, math.radians(x), delta=ANGULAR_VEL_TOLERANCE)

            art.set_dof_positions(np.array([0, 0]))
            art.set_dof_velocities(np.array([0, 0]))
            art.set_dof_efforts(np.array([0, 0]))

    async def test_lin_acc_imu(self) -> None:
        """Ensure linear acceleration magnitudes align with applied efforts."""
        await self._setup_simple_articulation()

        result, sensor = omni.kit.commands.execute(
            "IsaacSensorCreateImuSensor",
            path="/slider_imu",
            parent=self.slider_path,
            sensor_period=1 / self._sensor_rate,
            translation=Gf.Vec3d(0, 0, 0),
            orientation=Gf.Quatd(1, 0, 0, 0),
            linear_acceleration_filter_size=10,
            angular_velocity_filter_size=10,
            orientation_filter_size=10,
        )
        self.assertTrue(result)
        self.assertIsNotNone(sensor)

        # await self.test_add_arm_imu()
        result, sensor = omni.kit.commands.execute(
            "IsaacSensorCreateImuSensor",
            path="/arm_imu",
            parent=self.arm_path,
            sensor_period=1 / 60,
            translation=Gf.Vec3d(0, 0, 0),
            orientation=Gf.Quatd(1, 0, 0, 0),
            linear_acceleration_filter_size=10,
            angular_velocity_filter_size=10,
            orientation_filter_size=10,
        )
        self.assertTrue(result)
        self.assertIsNotNone(sensor)

        self._timeline.play()

        await omni.kit.app.get_app().next_update_async()
        art = Articulation("/Articulation")
        await omni.kit.app.get_app().next_update_async()
        art.set_dof_gains(np.zeros(art.num_dofs), np.ones(art.num_dofs))

        x = 0
        for i in range(60):

            art.set_dof_efforts(np.array([math.radians(x), 0]))
            await omni.kit.app.get_app().next_update_async()
            slider_mag = np.linalg.norm(
                [
                    self._is.get_sensor_reading(self.slider_path + "/slider_imu").lin_acc_x,
                    self._is.get_sensor_reading(self.slider_path + "/slider_imu").lin_acc_y,
                ]
            )
            arm_mag = np.linalg.norm(
                [
                    self._is.get_sensor_reading(self.arm_path + "/arm_imu").lin_acc_x,
                    self._is.get_sensor_reading(self.arm_path + "/arm_imu").lin_acc_y,
                ]
            )
            self.assertGreaterEqual(slider_mag, arm_mag)

            x += 1000

    async def test_gravity_m(self) -> None:
        """Test gravity m."""
        await self._setup_ant()
        await self._add_sensor_prims()
        self.ant.set_world_poses(positions=[0, 0, 1])
        UsdGeom.SetStageMetersPerUnit(self._stage, 1.0)

        await omni.kit.app.get_app().next_update_async()

        self._timeline.play()
        for i in range(20):
            await omni.kit.app.get_app().next_update_async()
            sensor_reading = self._is.get_sensor_reading(self.sphere_path + "/sensor")
            sensor_reading_no_gravity = self._is.get_sensor_reading(
                self.sphere_path + "/sensor", use_latest_data=True, read_gravity=False
            )
        self.assertAlmostEqual(sensor_reading.lin_acc_z, 0, delta=GRAVITY_TOLERANCE)
        self.assertAlmostEqual(sensor_reading_no_gravity.lin_acc_z, -EARTH_GRAVITY, delta=GRAVITY_TOLERANCE)

        for i in range(100):
            await omni.kit.app.get_app().next_update_async()
            sensor_reading = self._is.get_sensor_reading(self.sphere_path + "/sensor")
            sensor_reading_no_gravity = self._is.get_sensor_reading(
                self.sphere_path + "/sensor", use_latest_data=True, read_gravity=False
            )
        self.assertAlmostEqual(sensor_reading.lin_acc_z, EARTH_GRAVITY, delta=GRAVITY_TOLERANCE)
        self.assertAlmostEqual(sensor_reading_no_gravity.lin_acc_z, 0, delta=GRAVITY_TOLERANCE)

    async def test_gravity_moon_m(self) -> None:
        """Test gravity moon m."""
        await self._setup_ant()
        await self._add_sensor_prims()
        self.ant.set_world_poses(positions=[0, 0, 1])
        SimulationManager.get_physics_scenes()[0].set_gravity(Gf.Vec3f(0.0, 0.0, -MOON_GRAVITY))
        UsdGeom.SetStageMetersPerUnit(self._stage, 1.0)

        await omni.kit.app.get_app().next_update_async()

        self._timeline.play()
        for i in range(20):
            await omni.kit.app.get_app().next_update_async()
            sensor_reading = self._is.get_sensor_reading(self.sphere_path + "/sensor", use_latest_data=True)
        self.assertAlmostEqual(sensor_reading.lin_acc_z, 0, delta=GRAVITY_TOLERANCE)
        for i in range(200):
            await omni.kit.app.get_app().next_update_async()
            sensor_reading = self._is.get_sensor_reading(self.sphere_path + "/sensor", use_latest_data=True)
        self.assertAlmostEqual(sensor_reading.lin_acc_z, MOON_GRAVITY, delta=GRAVITY_TOLERANCE)

    async def test_gravity_cm(self) -> None:
        """Test gravity cm."""
        await self._setup_ant()
        await self._add_sensor_prims()

        UsdGeom.SetStageMetersPerUnit(self._stage, 0.01)
        SimulationManager.get_physics_scenes()[0].set_gravity(Gf.Vec3f(0.0, 0.0, -CM_GRAVITY))

        await omni.kit.app.get_app().next_update_async()

        await omni.kit.app.get_app().next_update_async()
        self._timeline.play()
        for i in range(100):
            await omni.kit.app.get_app().next_update_async()
            sensor_reading = self._is.get_sensor_reading(self.sphere_path + "/sensor", use_latest_data=True)
        self.assertAlmostEqual(sensor_reading.lin_acc_z, CM_GRAVITY, delta=GRAVITY_TOLERANCE)

    async def test_stop_start(self) -> None:
        """Test stop start."""
        await self._setup_ant()
        await self._add_sensor_prims()

        await omni.kit.app.get_app().next_update_async()

        self._timeline.play()
        await step_simulation(0.5)

        init_reading = self._is.get_sensor_reading(self.sphere_path + "/sensor", use_latest_data=True)

        self._timeline.stop()
        await omni.kit.app.get_app().next_update_async()

        self._timeline.play()
        await step_simulation(0.5)
        sensor_reading = self._is.get_sensor_reading(self.sphere_path + "/sensor", use_latest_data=True)

        self.assertAlmostEqual(sensor_reading.lin_acc_x, init_reading.lin_acc_x, delta=ORIENTATION_TOLERANCE)
        self.assertAlmostEqual(sensor_reading.lin_acc_y, init_reading.lin_acc_y, delta=ORIENTATION_TOLERANCE)
        self.assertAlmostEqual(sensor_reading.lin_acc_z, init_reading.lin_acc_z, delta=ORIENTATION_TOLERANCE)

    async def test_no_physics_scene(self) -> None:
        """Test no physics scene."""
        await stage_utils.open_stage_async(self._assets_root_path + "/Isaac/Environments/Grid/default_environment.usd")
        await omni.kit.app.get_app().next_update_async()
        self._stage = stage_utils.get_current_stage()
        await omni.kit.app.get_app().next_update_async()
        cube_path = "/new_cube"
        Cube(cube_path, sizes=1.0, positions=[0.0, 0.0, 2.0])
        GeomPrim(cube_path, apply_collision_apis=True)
        RigidPrim(cube_path, masses=[1.0])

        await omni.kit.app.get_app().next_update_async()
        result, sensor = omni.kit.commands.execute(
            "IsaacSensorCreateImuSensor",
            path="/sensor",
            parent=cube_path,
        )

        await omni.kit.app.get_app().next_update_async()

        self._timeline.play()
        for i in range(20):
            await omni.kit.app.get_app().next_update_async()
            sensor_reading = self._is.get_sensor_reading(cube_path + "/sensor", use_latest_data=True)
        self.assertAlmostEqual(sensor_reading.lin_acc_z, 0, delta=GRAVITY_TOLERANCE)
        for i in range(100):
            await omni.kit.app.get_app().next_update_async()
            sensor_reading = self._is.get_sensor_reading(cube_path + "/sensor", use_latest_data=True)
        self.assertAlmostEqual(sensor_reading.lin_acc_z, EARTH_GRAVITY, delta=GRAVITY_TOLERANCE)
        self._timeline.stop()

    async def test_rolling_average_attributes(self) -> None:
        """Verify larger filter windows reduce sensor output variance."""
        await self._setup_ant(physics_rate=400)
        await omni.kit.app.get_app().next_update_async()

        result, sensor = omni.kit.commands.execute(
            "IsaacSensorCreateImuSensor",
            path="/sphere_imu_1",
            parent=self.sphere_path,
            sensor_period=1 / self._sensor_rate,
            translation=Gf.Vec3d(0, 0, 0),
            orientation=Gf.Quatd(1, 0, 0, 0),
            linear_acceleration_filter_size=1,
            angular_velocity_filter_size=1,
            orientation_filter_size=1,
        )
        self.assertTrue(result)
        self.assertIsNotNone(sensor)

        low_rolling_avg_size_reading = np.zeros(
            6,
        )
        high_rolling_avg_size_reading = np.zeros(
            6,
        )

        self._timeline.play()
        # wait for the ant to settle down
        for i in range(200):
            await omni.kit.app.get_app().next_update_async()

        # test 1, when the rolling average is 1, should expect larger fluctuations
        for i in range(50):
            await omni.kit.app.get_app().next_update_async()
            sensor_reading = self._is.get_sensor_reading(self.sphere_path + "/sphere_imu_1", use_latest_data=True)

            if not sensor_reading.is_valid:
                continue

            readings = np.array(
                [
                    sensor_reading.lin_acc_x,
                    sensor_reading.lin_acc_y,
                    sensor_reading.lin_acc_z,
                    sensor_reading.ang_vel_x,
                    sensor_reading.ang_vel_y,
                    sensor_reading.ang_vel_z,
                ]
            )

            low_rolling_avg_size_reading = np.vstack((low_rolling_avg_size_reading, readings))

        # Ensure we collected enough valid readings (more than just the initial zeros row)
        self.assertGreater(
            low_rolling_avg_size_reading.shape[0], 1, "No valid sensor readings collected for low filter"
        )

        low_rolling_avg_size_1th_percentile = np.percentile(low_rolling_avg_size_reading, 1, axis=0)
        low_rolling_avg_size_99th_percentile = np.percentile(low_rolling_avg_size_reading, 99, axis=0)

        low_rolling_avg_size_diff = np.subtract(
            low_rolling_avg_size_99th_percentile, low_rolling_avg_size_1th_percentile
        )

        # test 2, when the rolling average is 20, should expect lower fluctuations
        sensor.CreateLinearAccelerationFilterWidthAttr().Set(20)
        sensor.CreateAngularVelocityFilterWidthAttr().Set(20)
        sensor.CreateOrientationFilterWidthAttr().Set(20)

        await omni.kit.app.get_app().next_update_async()

        for i in range(50):
            await omni.kit.app.get_app().next_update_async()
            sensor_reading = self._is.get_sensor_reading(self.sphere_path + "/sphere_imu_1", use_latest_data=True)

            if not sensor_reading.is_valid:
                continue

            readings = np.array(
                [
                    sensor_reading.lin_acc_x,
                    sensor_reading.lin_acc_y,
                    sensor_reading.lin_acc_z,
                    sensor_reading.ang_vel_x,
                    sensor_reading.ang_vel_y,
                    sensor_reading.ang_vel_z,
                ]
            )

            high_rolling_avg_size_reading = np.vstack((high_rolling_avg_size_reading, readings))

        # Ensure we collected enough valid readings (more than just the initial zeros row)
        self.assertGreater(
            high_rolling_avg_size_reading.shape[0], 1, "No valid sensor readings collected for high filter"
        )

        high_rolling_avg_size_1th_percentile = np.percentile(high_rolling_avg_size_reading, 1, axis=0)
        high_rolling_avg_size_99th_percentile = np.percentile(high_rolling_avg_size_reading, 99, axis=0)

        high_rolling_avg_size_diff = np.subtract(
            high_rolling_avg_size_99th_percentile, high_rolling_avg_size_1th_percentile
        )

        # low rolling average size is expected to have larger variation than with high rolling average size
        for i in range(len(high_rolling_avg_size_diff)):
            self.assertGreaterEqual(low_rolling_avg_size_diff[i], high_rolling_avg_size_diff[i])

    async def test_sensor_period(self) -> None:
        """Test sensor period."""
        await self._setup_ant()
        await self._add_sensor_prims()
        result, sensor = omni.kit.commands.execute(
            "IsaacSensorCreateImuSensor",
            path="/custom_sensor",
            parent=self.sphere_path,
            sensor_period=1 / (self._sensor_rate / 2),  # 30hz, half of physics rate
            translation=self.sensor_offsets[4],
            orientation=self.sensor_quatd[4],
        )
        self.assertTrue(result)
        self.assertIsNotNone(sensor)

        await omni.kit.app.get_app().next_update_async()
        self._timeline.play()
        await omni.kit.app.get_app().next_update_async()
        readings = []

        for i in range(60):  # Simulate for one second
            await omni.kit.app.get_app().next_update_async()
            sensor_reading = self._is.get_sensor_reading(self.sphere_path + "/custom_sensor", None, False)

            # the sensor is running at 30hz, while the sim is 60hz, so expecting 1/3 readings to be new,
            # the other reading should be identical and have the same timestamp
            if not readings or readings[-1].time != sensor_reading.time:
                readings.append(sensor_reading)

        # tolerance +-1 reading (29,30,31) will be accepted)
        self.assertTrue(abs(len(readings) - 30) <= 1)

    # use a custom function that sets the values to -100. Verify that the readings are -100 as a result of the function
    # verify that the measurement time are the same as not using the custom function
    # verify that the measuremnt using and not using the custom function are not the same
    async def test_custom_interpolation_function(self) -> None:
        """Test custom interpolation function."""

        def custom_function(sensorReadings: list[_sensor.IsSensorReading], time: float) -> _sensor.IsSensorReading:
            override_sensor_reading = _sensor.IsSensorReading()
            override_sensor_reading.lin_acc_x = -100
            override_sensor_reading.ang_vel_x = -100
            override_sensor_reading.time = time
            return override_sensor_reading

        await self._setup_ant()
        await self._add_sensor_prims()
        result, sensor = omni.kit.commands.execute(
            "IsaacSensorCreateImuSensor",
            path="/custom_sensor",
            parent=self.sphere_path,
            sensor_period=1 / (self._sensor_rate / 2),  # 30hz, half of physics rate
            translation=self.sensor_offsets[4],
            orientation=self.sensor_quatd[4],
        )
        self.assertTrue(result)
        self.assertIsNotNone(sensor)

        await omni.kit.app.get_app().next_update_async()
        self._timeline.play()
        await omni.kit.app.get_app().next_update_async()

        for i in range(10):  # Simulate 10 steps
            await omni.kit.app.get_app().next_update_async()

            # The effect of gravity is applied after the interpolation function
            custom_reading = self._is.get_sensor_reading(
                self.sphere_path + "/custom_sensor", custom_function, read_gravity=False
            )
            sensor_reading = self._is.get_sensor_reading(self.sphere_path + "/custom_sensor", read_gravity=False)

            self.assertEqual(custom_reading.time, sensor_reading.time)
            self.assertEqual(custom_reading.lin_acc_x, -100)
            self.assertEqual(custom_reading.ang_vel_x, -100)
            self.assertNotEqual(custom_reading.lin_acc_x, sensor_reading.lin_acc_x)

    async def test_sensor_latest_data(self) -> None:
        """Test sensor latest data."""
        await self._setup_ant()
        await self._add_sensor_prims()
        result, sensor = omni.kit.commands.execute(
            "IsaacSensorCreateImuSensor",
            path="/custom_sensor",
            parent=self.sphere_path,
            sensor_period=1 / (self._sensor_rate / 2),  # 30hz, half of physics rate
            translation=self.sensor_offsets[4],
            orientation=self.sensor_quatd[4],
        )
        self.assertTrue(result)
        self.assertIsNotNone(sensor)

        await omni.kit.app.get_app().next_update_async()
        self._timeline.play()
        await omni.kit.app.get_app().next_update_async()

        old_time = -1
        for i in range(10):  # Simulate 10 steps
            await omni.kit.app.get_app().next_update_async()
            latest_sensor_reading = self._is.get_sensor_reading(self.sphere_path + "/custom_sensor", None, True)
            self.assertTrue(latest_sensor_reading.time > old_time)
            old_time = latest_sensor_reading.time

    async def test_wrong_sensor_path(self) -> None:
        """Test wrong sensor path."""
        await self._setup_ant()
        await self._add_sensor_prims()
        await omni.kit.app.get_app().next_update_async()
        self._timeline.play()
        # give it some time to reach the ground first
        await omni.kit.app.get_app().next_update_async()

        for i in range(10):  # Simulate for 10 steps
            await omni.kit.app.get_app().next_update_async()
            latest_sensor_reading = self._is.get_sensor_reading(self.sphere_path + "/wrong_sensor", None, True)

            self.assertFalse(latest_sensor_reading.is_valid)
            self.assertEqual(latest_sensor_reading.time, 0)

    async def test_change_buffer_size(self) -> None:
        """Ensure filter widths control the underlying reading buffer size."""
        self.filter_wdith = 20
        self.actual_buffer_length = 20

        def custom_function(sensorReadings: list[_sensor.IsSensorReading], time: float) -> _sensor.IsSensorReading:
            override_sensor_reading = _sensor.IsSensorReading()
            self.actual_buffer_length = len(sensorReadings)
            return override_sensor_reading

        await self._setup_ant()
        await self._add_sensor_prims()
        result, sensor = omni.kit.commands.execute(
            "IsaacSensorCreateImuSensor",
            path="/custom_sensor",
            parent=self.sphere_path,
            sensor_period=1 / (self._sensor_rate / 2),  # 30hz, half of physics rate
            translation=self.sensor_offsets[4],
            orientation=self.sensor_quatd[4],
        )
        self.assertTrue(result)
        self.assertIsNotNone(sensor)

        await omni.kit.app.get_app().next_update_async()
        self._timeline.play()
        await omni.kit.app.get_app().next_update_async()
        await omni.kit.app.get_app().next_update_async()

        imu_sensor = prim_utils.get_prim_at_path(self.sphere_path + "/custom_sensor")
        for i in range(10):  # Simulate 10 steps
            self.filter_wdith += 1
            imu_sensor.GetAttribute("linearAccelerationFilterWidth").Set(self.filter_wdith)
            await omni.kit.app.get_app().next_update_async()
            await omni.kit.app.get_app().next_update_async()

            custom_reading = self._is.get_sensor_reading(self.sphere_path + "/custom_sensor", custom_function)

            # the sensor readings length is 2 times the size of the highest filter width, unless if the filter widths are under 10, then it is 20
            self.assertEqual(self.actual_buffer_length, 2 * self.filter_wdith)

        # set the rolling averages to something small, check the rolling average size doesn't go below 20
        imu_sensor.GetAttribute("linearAccelerationFilterWidth").Set(2)
        imu_sensor.GetAttribute("angularVelocityFilterWidth").Set(2)
        imu_sensor.GetAttribute("orientationFilterWidth").Set(2)

        await omni.kit.app.get_app().next_update_async()
        await omni.kit.app.get_app().next_update_async()

        custom_reading = self._is.get_sensor_reading(self.sphere_path + "/custom_sensor", custom_function)
        self.assertEqual(self.actual_buffer_length, 20)

    async def test_imu_rigidbody_grandparent(self) -> None:
        """Validate IMU readings through nested transform hierarchy changes."""
        await self._setup_ant()
        Cube("/World/Cube", sizes=1.0, positions=[10.0, 0.0, 0.0])
        GeomPrim("/World/Cube", apply_collision_apis=True)
        RigidPrim("/World/Cube", masses=[1.0])

        stage_utils.define_prim("/World/Cube/xform", "Xform")
        XformPrim("/World/Cube/xform", translations=[10.0, 0.0, 0.0], reset_xform_op_properties=True)

        result, sensor = omni.kit.commands.execute(
            "IsaacSensorCreateImuSensor",
            path="/custom_sensor",
            parent="/World/Cube/xform",
            sensor_period=0,
            translation=self.sensor_offsets[4],
            orientation=self.sensor_quatd[4],
        )

        await omni.kit.app.get_app().next_update_async()
        self._timeline.play()
        await step_simulation(0.5)
        custom_reading = self._is.get_sensor_reading("/World/Cube/xform/custom_sensor")

        self.assertAlmostEqual(custom_reading.lin_acc_z, EARTH_GRAVITY, delta=GRAVITY_TOLERANCE)

        self._timeline.stop()
        await omni.kit.app.get_app().next_update_async()

        # Rotate the parent cube about y by -90 degree
        # The x axis points upward
        xform_prim = XformPrim("/World/Cube", reset_xform_op_properties=True)
        xform_prim.set_local_poses(orientations=[0.70711, 0.0, -0.70711, 0.0])
        self._timeline.play()
        await omni.kit.app.get_app().next_update_async()

        await step_simulation(0.5)
        custom_reading = self._is.get_sensor_reading("/World/Cube/xform/custom_sensor")
        self.assertAlmostEqual(custom_reading.lin_acc_x, EARTH_GRAVITY, delta=GRAVITY_TOLERANCE)

        # rotated -90 degress abouty, check if this is correct
        # note: (-0.70711, 0 0.70711, 0) and (0.70711, 0, -0.70711, 0) represent the same angle
        self.assertAlmostEqual(abs(custom_reading.orientation.w), 0.70711, delta=ORIENTATION_TOLERANCE)
        self.assertAlmostEqual(custom_reading.orientation.x, 0.0, delta=ORIENTATION_TOLERANCE)
        self.assertAlmostEqual(abs(custom_reading.orientation.y), 0.70711, delta=ORIENTATION_TOLERANCE)
        self.assertAlmostEqual(custom_reading.orientation.z, 0.0, delta=ORIENTATION_TOLERANCE)
        self.assertAlmostEqual(custom_reading.orientation.w, -custom_reading.orientation.y, delta=ORIENTATION_TOLERANCE)

    async def test_invalid_imu(self) -> None:
        """Test invalid imu."""
        # goal is to make sure an invalid imu doesn't crash the sim
        result, sensor = omni.kit.commands.execute(
            "IsaacSensorCreateImuSensor",
            path="/sensor",
            parent="/World",
            sensor_period=1 / self._sensor_rate,
            translation=Gf.Vec3d(0, 0, 0),
            orientation=Gf.Quatd(1, 0, 0, 0),
        )
        SimulationManager.setup_simulation(dt=1.0 / 60.0)
        self._timeline.play()
        await step_simulation(0.1)
        self._timeline.stop()

    async def test_is_imu_sensor(self) -> None:
        """Test is_imu_sensor returns correct values for valid sensor, invalid prim, and non-existent prim."""
        await stage_utils.create_new_stage_async()
        SimulationManager.setup_simulation(dt=1.0 / 60.0)

        # Create a cube with rigid body
        Cube("/World/Cube", sizes=1.0, positions=[0.0, 0.0, 1.0])
        GeomPrim("/World/Cube", apply_collision_apis=True)
        RigidPrim("/World/Cube", masses=[1.0])
        await omni.kit.app.get_app().next_update_async()

        # Create an IMU sensor on the cube
        result, sensor = omni.kit.commands.execute(
            "IsaacSensorCreateImuSensor",
            path="/imu_sensor",
            parent="/World/Cube",
            sensor_period=1 / self._sensor_rate,
            translation=Gf.Vec3d(0, 0, 0),
            orientation=Gf.Quatd(1, 0, 0, 0),
        )
        self.assertTrue(result)
        self.assertIsNotNone(sensor)
        await omni.kit.app.get_app().next_update_async()

        # Before simulation starts, sensor should not be registered with the manager
        self.assertFalse(self._is.is_imu_sensor("/World/Cube/imu_sensor"))

        # Start the timeline to register sensors with the manager
        self._timeline.play()
        await omni.kit.app.get_app().next_update_async()

        # After simulation starts, valid IMU sensor path should return True
        self.assertTrue(self._is.is_imu_sensor("/World/Cube/imu_sensor"))

        # Test invalid prim (cube itself, not a sensor) - should return False
        self.assertFalse(self._is.is_imu_sensor("/World/Cube"))

        # Test non-existent prim path - should return False
        self.assertFalse(self._is.is_imu_sensor("/World/NonExistent/sensor"))

    async def test_is_sensor_reading_defaults(self) -> None:
        """Verify IsSensorReading default construction values."""
        reading = _sensor.IsSensorReading()
        self.assertEqual(reading.time, 0.0)
        self.assertEqual(reading.lin_acc_x, 0.0)
        self.assertEqual(reading.lin_acc_y, 0.0)
        self.assertEqual(reading.lin_acc_z, 0.0)
        self.assertEqual(reading.ang_vel_x, 0.0)
        self.assertEqual(reading.ang_vel_y, 0.0)
        self.assertEqual(reading.ang_vel_z, 0.0)
        self.assertFalse(reading.is_valid)
