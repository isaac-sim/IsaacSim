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

"""Verifies IMU sensor authoring and runtime data for orientation, angular velocity, linear acceleration, gravity settings, timeline lifecycle, buffer and rolling-average settings, invalid prims, nested rigid bodies, free fall, and reader reinitialization."""

from __future__ import annotations

import asyncio
import math
from typing import Any

import carb
import carb.tokens
import isaacsim.core.experimental.utils.prim as prim_utils
import isaacsim.core.experimental.utils.stage as stage_utils
import isaacsim.core.experimental.utils.transform as transform_utils
import numpy as np
import omni.kit.test
import omni.timeline
from isaacsim.core.experimental.objects import Cube, GroundPlane
from isaacsim.core.experimental.prims import Articulation, GeomPrim, RigidPrim, XformPrim
from isaacsim.core.simulation_manager import SimulationManager
from isaacsim.sensors.experimental.physics import IMU, IMUSensor, IMUSensorReading
from isaacsim.storage.native import get_assets_root_path_async
from pxr import Gf, Sdf, UsdGeom, UsdPhysics, UsdUtils

from .common import (
    ANGLE_TOLERANCE_DEG,
    ANGULAR_VEL_TOLERANCE,
    CM_GRAVITY,
    EARTH_GRAVITY,
    GRAVITY_TOLERANCE,
    MOON_GRAVITY,
    ORIENTATION_TOLERANCE,
    is_physx_engine,
    reset_timeline,
    setup_ant_scene,
    step_simulation,
)

# NOTE:
#   omni.kit.test - std python's unittest module with additional wrapping to add suport for async/await tests
#   For most things refer to unittest docs: https://docs.python.org/3/library/unittest.html


# Having a test class dervived from omni.kit.test.AsyncTestCase declared on the root of module will make it auto-discoverable by omni.kit.test
class TestIMUSensor(omni.kit.test.AsyncTestCase):
    """Test i m u sensor."""

    # Before running each test
    async def setUp(self) -> None:
        """Initialize IMU sensor caches, sensor rate, timeline access, and asset root."""
        self._sensor_rate = 60
        self._imu_sensors: dict[str, IMUSensor] = {}
        self._assets_root_path = await get_assets_root_path_async()
        if self._assets_root_path is None:
            carb.log_error("Could not find Isaac Sim assets folder")
            return
        self._timeline = omni.timeline.get_timeline_interface()
        self._ant_config = None

    def _get_imu_sensor(self, prim_path: str) -> IMUSensor:
        """Get or create a cached IMUSensor for the given path.

        Args:
            prim_path: USD path to the IMU sensor prim.

        Returns:
            Cached IMU sensor for ``prim_path``.
        """
        if prim_path not in self._imu_sensors:
            self._imu_sensors[prim_path] = IMUSensor(prim_path)
        return self._imu_sensors[prim_path]

    async def _setup_ant(self, physics_rate: Any = 60, **kwargs: Any) -> None:
        """Load the ant scene and configure ant-specific test data.

        Args:
            physics_rate: Physics simulation rate in Hz.
            **kwargs: Forwarded to ``setup_ant_scene``.
        """
        self._ant_config = await setup_ant_scene(physics_rate, **kwargs)
        self._stage = stage_utils.get_current_stage()
        await omni.kit.app.get_app().next_update_async()
        self.ant = XformPrim("/Ant", reset_xform_op_properties=True)

    # Convenience properties for ant configuration
    @property
    def leg_paths(self) -> Any:
        """Return leg paths."""
        return self._ant_config.leg_paths

    @property
    def sphere_path(self) -> Any:
        """Perform sphere path operation."""
        return self._ant_config.sphere_path

    @property
    def sensor_offsets(self) -> Any:
        """Return sensor offsets."""
        return self._ant_config.imu_sensor_offsets

    @property
    def sensor_quatd(self) -> Any:
        """Perform sensor quatd operation."""
        return self._ant_config.sensor_quatd

    async def _setup_simple_articulation(self, physics_rate: Any = 60) -> None:
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
        """Reset cached IMU sensors, stop playback, and clear physics state."""
        for sensor in self._imu_sensors.values():
            sensor.reset()
            sensor.on_timeline_stop()
        self._imu_sensors.clear()
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
            sensor = IMUSensor(
                IMU.create(
                    self.leg_paths[i] + "/sensor",
                    translations=self.sensor_offsets[i],
                    orientations=self.sensor_quatd[i],
                )
            )
            self.assertIsNotNone(sensor)
            # Add sensor on body sphere
            await omni.kit.app.get_app().next_update_async()
            sensor = IMUSensor(
                IMU.create(
                    self.sphere_path + "/sensor",
                    translations=self.sensor_offsets[4],
                    orientations=self.sensor_quatd[4],
                )
            )
            self.assertIsNotNone(sensor)

    async def test_add_sensor_prim(self) -> None:
        """Test add sensor prim."""
        await self._setup_ant()
        await self._add_sensor_prims()

    async def test_physics_only_step_outputs_imu_data(self) -> None:
        """IMUSensor produces data when stepping physics without app/render updates."""
        await stage_utils.create_new_stage_async()
        await omni.kit.app.get_app().next_update_async()
        stage_utils.set_stage_units(meters_per_unit=1.0)
        SimulationManager.setup_simulation(dt=1.0 / self._sensor_rate)

        cube_path = "/World/PhysicsOnlyCube"
        Cube(cube_path, sizes=1.0, positions=[0.0, 0.0, 2.0])
        GeomPrim(cube_path, apply_collision_apis=True)
        RigidPrim(cube_path, masses=[1.0])

        sensor = IMUSensor(
            IMU.create(
                cube_path + "/physics_only_imu",
                translations=[[0.0, 0.0, 0.0]],
                orientations=[[1.0, 0.0, 0.0, 0.0]],
            )
        )

        try:
            self._timeline.play()
            SimulationManager.initialize_physics()
            SimulationManager.step(steps=3, update_fabric=False)

            reading = sensor.get_sensor_reading()
            self.assertTrue(reading.is_valid, "Reading should be valid after physics-only steps")
            self.assertGreater(reading.time, 0.0)
            self.assertTrue(np.isfinite(reading.linear_acceleration_z))
        finally:
            sensor.reset()
            if self._timeline.is_playing():
                self._timeline.stop()
                await omni.kit.app.get_app().next_update_async()

    async def test_orientation_imu(self) -> None:
        """Test orientation imu."""
        await self._setup_simple_articulation()

        sensor = IMUSensor(
            IMU.create(
                self.arm_path + "/arm_imu",
                translations=[[0.0, 0.0, 0.0]],
                orientations=[[1.0, 0.0, 0.0, 0.0]],
            )
        )
        self.assertIsNotNone(sensor)

        self._timeline.play()

        await omni.kit.app.get_app().next_update_async()

        articulation = Articulation("/Articulation")
        await omni.kit.app.get_app().next_update_async()
        articulation.set_dof_gains(np.ones(articulation.num_dofs) * 1e8, np.ones(articulation.num_dofs) * 1e8)

        angle = 0
        for i in range(70):
            articulation.set_dof_positions(np.array([math.radians(angle), 0.5]))

            await omni.kit.app.get_app().next_update_async()
            await omni.kit.app.get_app().next_update_async()

            r = self._get_imu_sensor(self.arm_path + "/arm_imu").get_sensor_reading()
            euler = transform_utils.quaternion_to_euler_angles(
                np.array([r.orientation_x, r.orientation_y, r.orientation_z, r.orientation_w]),
                degrees=True,
            )
            orientation = euler.numpy()[0]

            expected_angle = angle % 360
            if angle >= 180:
                expected_angle = angle - 360

            self.assertAlmostEqual(orientation, expected_angle, delta=ANGLE_TOLERANCE_DEG)
            angle += 5

    async def test_ang_vel_imu(self) -> None:
        """Test ang vel imu."""
        await self._setup_simple_articulation()

        sensor = IMUSensor(
            IMU.create(
                self.slider_path + "/slider_imu",
                translations=[[0.0, 0.0, 0.0]],
                orientations=[[1.0, 0.0, 0.0, 0.0]],
            )
        )
        self.assertIsNotNone(sensor)

        self._timeline.play()

        await omni.kit.app.get_app().next_update_async()

        articulation = Articulation("/Articulation")
        await omni.kit.app.get_app().next_update_async()

        angular_velocity_list = [x * 30 for x in range(0, 20)]

        for x in angular_velocity_list:
            articulation.set_dof_velocities(np.array([math.radians(x), 0]))

            await omni.kit.app.get_app().next_update_async()
            angular_velocity_z = (
                self._get_imu_sensor(self.slider_path + "/slider_imu").get_sensor_reading().angular_velocity_z
            )
            # with sensor frequency = physics rate, all should be the same
            self.assertAlmostEqual(angular_velocity_z, math.radians(x), delta=ANGULAR_VEL_TOLERANCE)

            articulation.set_dof_positions(np.array([0, 0]))
            articulation.set_dof_velocities(np.array([0, 0]))
            articulation.set_dof_efforts(np.array([0, 0]))

    async def test_lin_acc_imu(self) -> None:
        """Ensure linear acceleration magnitudes align with applied efforts."""
        await self._setup_simple_articulation()

        sensor = IMUSensor(
            IMU.create(
                self.slider_path + "/slider_imu",
                translations=[[0.0, 0.0, 0.0]],
                orientations=[[1.0, 0.0, 0.0, 0.0]],
                linear_acceleration_filter_size=10,
                angular_velocity_filter_size=10,
                orientation_filter_size=10,
            )
        )
        self.assertIsNotNone(sensor)

        # await self.test_add_arm_imu()
        sensor = IMUSensor(
            IMU.create(
                self.arm_path + "/arm_imu",
                translations=[[0.0, 0.0, 0.0]],
                orientations=[[1.0, 0.0, 0.0, 0.0]],
                linear_acceleration_filter_size=10,
                angular_velocity_filter_size=10,
                orientation_filter_size=10,
            )
        )
        self.assertIsNotNone(sensor)

        self._timeline.play()

        await omni.kit.app.get_app().next_update_async()
        articulation = Articulation("/Articulation")
        await omni.kit.app.get_app().next_update_async()
        articulation.set_dof_gains(np.zeros(articulation.num_dofs), np.ones(articulation.num_dofs))

        x = 0
        for i in range(60):

            articulation.set_dof_efforts(np.array([math.radians(x), 0]))
            await omni.kit.app.get_app().next_update_async()
            slider_reading = self._get_imu_sensor(self.slider_path + "/slider_imu").get_sensor_reading()
            slider_magnitude = np.linalg.norm(
                [slider_reading.linear_acceleration_x, slider_reading.linear_acceleration_y]
            )
            arm_reading = self._get_imu_sensor(self.arm_path + "/arm_imu").get_sensor_reading()
            arm_magnitude = np.linalg.norm([arm_reading.linear_acceleration_x, arm_reading.linear_acceleration_y])
            self.assertGreaterEqual(slider_magnitude, arm_magnitude)

            x += 1000

    async def test_gravity_m(self) -> None:
        """Test gravity m."""
        await self._setup_ant()
        await self._add_sensor_prims()
        self.ant.set_world_poses(positions=[0, 0, 1.5])
        UsdGeom.SetStageMetersPerUnit(self._stage, 1.0)

        await omni.kit.app.get_app().next_update_async()

        self._timeline.play()
        for i in range(20):
            await omni.kit.app.get_app().next_update_async()
            backend = self._get_imu_sensor(self.sphere_path + "/sensor")
            sensor_reading = backend.get_sensor_reading()
            sensor_reading_no_gravity = backend.get_sensor_reading(read_gravity=False)
        self.assertAlmostEqual(sensor_reading.linear_acceleration_z, 0, delta=GRAVITY_TOLERANCE)
        self.assertAlmostEqual(sensor_reading_no_gravity.linear_acceleration_z, -EARTH_GRAVITY, delta=GRAVITY_TOLERANCE)

        for i in range(100):
            await omni.kit.app.get_app().next_update_async()
            backend = self._get_imu_sensor(self.sphere_path + "/sensor")
            sensor_reading = backend.get_sensor_reading()
            sensor_reading_no_gravity = backend.get_sensor_reading(read_gravity=False)
        self.assertAlmostEqual(sensor_reading.linear_acceleration_z, EARTH_GRAVITY, delta=GRAVITY_TOLERANCE)
        self.assertAlmostEqual(sensor_reading_no_gravity.linear_acceleration_z, 0, delta=GRAVITY_TOLERANCE)

    @staticmethod
    def _sensor_acceleration(sensor: IMUSensor, *, read_gravity: bool) -> list[float]:
        """Read one IMU sample as a plain ``[x, y, z]`` linear acceleration list.

        Args:
            sensor: Sensor to sample.
            read_gravity: Whether the accelerometer channel includes the gravity reaction term.

        Returns:
            Linear acceleration components in the sensor frame.
        """
        reading = sensor.get_sensor_reading(read_gravity=read_gravity)
        return [
            reading.linear_acceleration_x,
            reading.linear_acceleration_y,
            reading.linear_acceleration_z,
        ]

    async def test_gravity_y_up_stage_unauthored_direction(self) -> None:
        """A resting body on a Y-up stage reads ``+g`` on y when ``gravityDirection`` is unauthored.

        ``UsdPhysicsScene.gravityDirection`` defaults to ``(0, 0, 0)``, which the USD Physics
        specification defines as a request to use the negative stage up axis. The sensor must
        resolve that the same way the physics engine does, otherwise the gravity reaction term
        lands on the wrong axis and the reading is wrong on two axes at once.

        Restricted to PhysX: the Newton backend rotates a stage whose up axis is not Z into its
        own Z-up frame, so the poses and velocities it reports back (and therefore the whole
        sensor frame) are rotated relative to USD world space. Asserting USD-frame axes against
        that backend would be testing the up-axis conversion, not the gravity fallback.
        """
        if not is_physx_engine():
            return
        await stage_utils.create_new_stage_async()
        await omni.kit.app.get_app().next_update_async()
        stage = stage_utils.get_current_stage()
        UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.y)
        stage_utils.set_stage_units(meters_per_unit=1.0)
        SimulationManager.setup_simulation(dt=1.0 / self._sensor_rate)

        # A static collision box is used instead of GroundPlane so the ground orientation does
        # not depend on the stage up axis. Leave the physics scene's gravity attributes
        # unauthored: that is what puts the up-axis fallback under test.
        ground_path = "/World/YUpGround"
        Cube(ground_path, sizes=1.0, positions=[0.0, -0.5, 0.0], scales=[20.0, 1.0, 20.0])
        GeomPrim(ground_path, apply_collision_apis=True)

        cube_path = "/World/YUpCube"
        Cube(cube_path, sizes=1.0, positions=[0.0, 3.0, 0.0])
        GeomPrim(cube_path, apply_collision_apis=True)
        RigidPrim(cube_path, masses=[1.0])

        sensor = IMUSensor(
            IMU.create(
                cube_path + "/y_up_imu",
                translations=[[0.0, 0.0, 0.0]],
                orientations=[[1.0, 0.0, 0.0, 0.0]],
            )
        )
        self._imu_sensors[sensor.imu.paths[0]] = sensor

        await omni.kit.app.get_app().next_update_async()
        self._timeline.play()

        # Free fall pins down what the physics engine actually did: coordinate acceleration is
        # engine-derived (a finite difference of the rigid body's velocity), so -g on y proves
        # the engine resolved the unauthored gravity direction to the negative stage up axis.
        await step_simulation(0.2)
        free_fall = self._sensor_acceleration(sensor, read_gravity=False)
        message = f"free-fall linear acceleration {free_fall}"
        self.assertAlmostEqual(free_fall[1], -EARTH_GRAVITY, delta=GRAVITY_TOLERANCE, msg=message)
        self.assertAlmostEqual(free_fall[2], 0.0, delta=GRAVITY_TOLERANCE, msg=message)

        # In free fall an accelerometer is weightless on every axis, so the gravity reaction
        # term has to cancel the coordinate acceleration exactly.
        weightless = self._sensor_acceleration(sensor, read_gravity=True)
        message = f"free-fall specific force {weightless}"
        for axis in range(3):
            self.assertAlmostEqual(weightless[axis], 0.0, delta=GRAVITY_TOLERANCE, msg=message)

        # At rest on the ground the accelerometer measures +g on the axis opposing gravity.
        await step_simulation(2.0)
        resting = self._sensor_acceleration(sensor, read_gravity=True)
        message = f"resting specific force {resting}"
        self.assertAlmostEqual(resting[1], EARTH_GRAVITY, delta=GRAVITY_TOLERANCE, msg=message)
        self.assertAlmostEqual(resting[0], 0.0, delta=GRAVITY_TOLERANCE, msg=message)
        self.assertAlmostEqual(resting[2], 0.0, delta=GRAVITY_TOLERANCE, msg=message)

    async def test_centripetal_acceleration_in_circular_motion(self) -> None:
        """A body in uniform circular motion measures its centripetal acceleration.

        Linear velocity has to be differentiated in the **world** frame, with the resulting
        acceleration rotated into the sensor frame afterwards. Differentiating the sensor-frame
        velocity instead drops the transport term ``omega x v``, and this scenario is the case
        where that term is the whole answer: a body going round a circle with its axes locked to
        the trajectory has the constant sensor-frame velocity ``(0, omega * r, 0)``, so a
        sensor-frame difference reports no acceleration at all while the body is really
        accelerating at ``omega^2 * r`` toward the centre.

        A revolute joint driven at a constant rate supplies the centripetal force, so the
        trajectory stays exact without writing poses every step. The body's ``+x`` axis is
        pinned radially outward by the joint, which puts the inward centripetal term on ``-x``.

        Restricted to PhysX: Newton does not carry the body around the world-anchored revolute
        joint, so the commanded motion is gone within a step and no rotation is left to put the
        transport term under test.
        """
        if not is_physx_engine():
            return

        radius = 2.0
        angular_speed = 2.0  # rad/s

        await stage_utils.create_new_stage_async()
        await omni.kit.app.get_app().next_update_async()
        stage = stage_utils.get_current_stage()
        stage_utils.set_stage_units(meters_per_unit=1.0)
        # One physics step per app update. Substepping would hide the defect: the sensor's world
        # transform is only refreshed once per app update, so consecutive substeps share an
        # orientation and their velocity difference stays in a single frame either way.
        SimulationManager.setup_simulation(dt=1.0 / self._sensor_rate)

        body_path = "/World/CarouselBody"
        Cube(body_path, sizes=0.2, positions=[radius, 0.0, 0.0])
        RigidPrim(body_path, masses=[1.0])
        # Damping would apply a torque the drive has to fight, leaving a steady-state rate error.
        body_prim = stage.GetPrimAtPath(body_path)
        body_prim.CreateAttribute("physxRigidBody:linearDamping", Sdf.ValueTypeNames.Float).Set(0.0)
        body_prim.CreateAttribute("physxRigidBody:angularDamping", Sdf.ValueTypeNames.Float).Set(0.0)

        # No body0 means the joint anchors to the world, so localPos0 is the world-space centre
        # of rotation and localPos1 places the body one radius out along its own +x axis.
        joint = UsdPhysics.RevoluteJoint.Define(stage, "/World/CarouselJoint")
        joint.CreateBody1Rel().SetTargets([body_path])
        joint.CreateAxisAttr("Z")
        joint.CreateLocalPos0Attr(Gf.Vec3f(0.0, 0.0, 0.0))
        joint.CreateLocalPos1Attr(Gf.Vec3f(-radius, 0.0, 0.0))
        # A pure velocity drive: with no stiffness the steady state needs no torque, so the rate
        # settles on the target. Angular drive targets are in degrees per second.
        drive = UsdPhysics.DriveAPI.Apply(joint.GetPrim(), "angular")
        drive.CreateTypeAttr().Set(UsdPhysics.Tokens.force)
        drive.CreateStiffnessAttr().Set(0.0)
        drive.CreateDampingAttr().Set(1.0e6)
        drive.CreateTargetVelocityAttr().Set(math.degrees(angular_speed))

        sensor = IMUSensor(
            IMU.create(
                body_path + "/carousel_imu",
                translations=[[0.0, 0.0, 0.0]],
                orientations=[[1.0, 0.0, 0.0, 0.0]],
                linear_acceleration_filter_size=1,
                angular_velocity_filter_size=1,
                orientation_filter_size=1,
            )
        )
        self._imu_sensors[sensor.imu.paths[0]] = sensor

        await omni.kit.app.get_app().next_update_async()
        self._timeline.play()
        await omni.kit.app.get_app().next_update_async()

        # Pin the step ratio rather than trusting it to fall out of the dt above. Under
        # substepping the sensor's world transform is refreshed once per app update, so the
        # substeps after the first difference velocity against an unchanged orientation and the
        # pre-fix code lands on the right answer by accident. This test would keep passing while
        # discriminating nothing, so assert the condition it depends on.
        steps_before = SimulationManager.get_num_physics_steps()
        await omni.kit.app.get_app().next_update_async()
        steps_per_update = SimulationManager.get_num_physics_steps() - steps_before
        self.assertEqual(
            steps_per_update,
            1,
            msg=f"carousel needs one physics step per app update to expose the transport term, got {steps_per_update}",
        )

        # Launch the arm at the commanded rate. The drive on its own takes several seconds to
        # spin a 2 m arm up, and during the ramp the tangential term alpha x r dominates; the
        # transport term is only isolated once the rate is constant. From the target rate the
        # drive has no error to correct, so the motion is steady immediately.
        RigidPrim(body_path).set_velocities(
            linear_velocities=[[0.0, angular_speed * radius, 0.0]],
            angular_velocities=[[0.0, 0.0, angular_speed]],
        )
        await step_simulation(0.5)

        # Guard the setup: with no rotation there is no transport term to measure, and every
        # assertion below would pass for the wrong reason.
        reading = sensor.get_sensor_reading(read_gravity=False)
        self.assertTrue(reading.is_valid, "carousel IMU reading should be valid while spinning")
        self.assertAlmostEqual(
            reading.angular_velocity_z,
            angular_speed,
            delta=ANGULAR_VEL_TOLERANCE,
            msg=f"carousel drive should hold {angular_speed} rad/s, got {reading.angular_velocity_z}",
        )
        # The guard stays coarse on purpose. The reported angular velocity is the body's
        # instantaneous rate, while the acceleration is a difference of tangential velocity over
        # the step, and the two do not agree to better than a couple of percent while the drive
        # settles: measured here as 1.958 rad/s against an acceleration of 8.007, which implies
        # 2.001. Deriving the expected magnitude from the reported rate, or tightening this guard
        # to match the magnitude tolerance, therefore fails a run whose acceleration is correct.
        expected_centripetal = angular_speed * angular_speed * radius  # stage linear units / s^2

        no_gravity = self._sensor_acceleration(sensor, read_gravity=False)
        message = f"circular-motion coordinate acceleration {no_gravity}"
        # The magnitude is what the transport term supplies, and it is free of the phase lag
        # below: without the term the in-plane reading is zero rather than short.
        in_plane = math.hypot(no_gravity[0], no_gravity[1])
        self.assertAlmostEqual(in_plane, expected_centripetal, delta=GRAVITY_TOLERANCE, msg=message)
        self.assertAlmostEqual(no_gravity[2], 0.0, delta=GRAVITY_TOLERANCE, msg=message)
        # The vector points inward, along the sensor's -x. It trails that axis by a fraction of
        # a degree per rad/s: the acceleration is the mean over the differenced step while the
        # orientation it is rotated by is sampled one step behind, and the two do not coincide.
        trailing_angle_deg = abs(math.degrees(math.atan2(no_gravity[1], -no_gravity[0])))
        self.assertLess(trailing_angle_deg, 5.0, msg=message)

        # The gravity reaction term rides on top of the centripetal one, both in the sensor
        # frame; the joint axis is vertical, so gravity stays on z.
        specific_force = self._sensor_acceleration(sensor, read_gravity=True)
        message = f"circular-motion specific force {specific_force}"
        in_plane = math.hypot(specific_force[0], specific_force[1])
        self.assertAlmostEqual(in_plane, expected_centripetal, delta=GRAVITY_TOLERANCE, msg=message)
        self.assertAlmostEqual(specific_force[2], EARTH_GRAVITY, delta=GRAVITY_TOLERANCE, msg=message)

    async def test_offset_sensor_measures_its_own_lever_arm(self) -> None:
        """A sensor mounted away from the centre of mass measures the motion of its own mount.

        PhysX reports rigid-body velocities at the **centre of mass**, so buffering that velocity
        and differencing it yields the acceleration of the centre of mass, not of the sensor. For a
        rigid body the two differ by ``w x r`` over the lever arm between them, and dropping that
        term costs the sensor the centripetal and tangential acceleration of its own mount.

        This is the case ``test_centripetal_acceleration_in_circular_motion`` cannot see: that
        sensor sits at the body origin with the centre of mass unauthored, so the lever arm is zero
        and both the corrected and uncorrected code report the same answer.

        Here two sensors ride the same carousel arm at different radii. The inner one sits on the
        body origin and the outer one is mounted ``mount_offset`` further out along ``+x``, so the
        outer sensor is turning on a strictly larger radius and must read a strictly larger
        centripetal magnitude. Without the lever-arm term both sensors report the velocity of the
        one shared centre of mass, so their readings are **identical** -- which is what the
        difference assertion below pins.

        Restricted to PhysX, for the reason given on the sibling carousel test: Newton does not
        carry the body around a world-anchored revolute joint.
        """
        if not is_physx_engine():
            return

        radius = 2.0
        angular_speed = 2.0  # rad/s
        mount_offset = 0.5  # stage linear units, along the arm's +x, i.e. radially outward

        await stage_utils.create_new_stage_async()
        await omni.kit.app.get_app().next_update_async()
        stage = stage_utils.get_current_stage()
        stage_utils.set_stage_units(meters_per_unit=1.0)
        # One physics step per app update, as the sibling carousel test explains: substepping
        # refreshes the sensor world transform once per update and hides frame-order defects.
        SimulationManager.setup_simulation(dt=1.0 / self._sensor_rate)

        body_path = "/World/CarouselBody"
        Cube(body_path, sizes=0.2, positions=[radius, 0.0, 0.0])
        RigidPrim(body_path, masses=[1.0])
        body_prim = stage.GetPrimAtPath(body_path)
        # Damping would apply a torque the drive has to fight, leaving a steady-state rate error.
        body_prim.CreateAttribute("physxRigidBody:linearDamping", Sdf.ValueTypeNames.Float).Set(0.0)
        body_prim.CreateAttribute("physxRigidBody:angularDamping", Sdf.ValueTypeNames.Float).Set(0.0)
        # Pin the centre of mass on the body origin. Unauthored it lands there anyway, but the
        # lever arm is measured from this point, so the test states it rather than inheriting it.
        UsdPhysics.MassAPI.Apply(body_prim).CreateCenterOfMassAttr().Set(Gf.Vec3f(0.0, 0.0, 0.0))

        # World-anchored revolute joint: no body0, so localPos0 is the centre of rotation and
        # localPos1 places the body one radius out along its own +x axis.
        joint = UsdPhysics.RevoluteJoint.Define(stage, "/World/CarouselJoint")
        joint.CreateBody1Rel().SetTargets([body_path])
        joint.CreateAxisAttr("Z")
        joint.CreateLocalPos0Attr(Gf.Vec3f(0.0, 0.0, 0.0))
        joint.CreateLocalPos1Attr(Gf.Vec3f(-radius, 0.0, 0.0))
        drive = UsdPhysics.DriveAPI.Apply(joint.GetPrim(), "angular")
        drive.CreateTypeAttr().Set(UsdPhysics.Tokens.force)
        drive.CreateStiffnessAttr().Set(0.0)
        drive.CreateDampingAttr().Set(1.0e6)
        drive.CreateTargetVelocityAttr().Set(math.degrees(angular_speed))

        sensors = {}
        for name, offset in (("hub_imu", 0.0), ("rim_imu", mount_offset)):
            sensor = IMUSensor(
                IMU.create(
                    f"{body_path}/{name}",
                    translations=[[offset, 0.0, 0.0]],
                    orientations=[[1.0, 0.0, 0.0, 0.0]],  # wxyz, aligned with the body
                    linear_acceleration_filter_size=1,
                    angular_velocity_filter_size=1,
                    orientation_filter_size=1,
                )
            )
            self._imu_sensors[sensor.imu.paths[0]] = sensor
            sensors[name] = sensor

        await omni.kit.app.get_app().next_update_async()
        self._timeline.play()
        await omni.kit.app.get_app().next_update_async()

        steps_before = SimulationManager.get_num_physics_steps()
        await omni.kit.app.get_app().next_update_async()
        steps_per_update = SimulationManager.get_num_physics_steps() - steps_before
        self.assertEqual(
            steps_per_update,
            1,
            msg=f"carousel needs one physics step per app update, got {steps_per_update}",
        )

        # Launch at the commanded rate so the motion is steady immediately: during a spin-up ramp
        # the tangential term alpha x r dominates and the radius comparison below is not clean.
        RigidPrim(body_path).set_velocities(
            linear_velocities=[[0.0, angular_speed * radius, 0.0]],
            angular_velocities=[[0.0, 0.0, angular_speed]],
        )
        await step_simulation(0.5)

        # Guard the setup: with no rotation there is no lever-arm term and every assertion below
        # would pass for the wrong reason.
        hub_reading = sensors["hub_imu"].get_sensor_reading(read_gravity=False)
        self.assertTrue(hub_reading.is_valid, "carousel IMU reading should be valid while spinning")
        self.assertAlmostEqual(
            hub_reading.angular_velocity_z,
            angular_speed,
            delta=ANGULAR_VEL_TOLERANCE,
            msg=f"carousel drive should hold {angular_speed} rad/s, got {hub_reading.angular_velocity_z}",
        )

        hub = self._sensor_acceleration(sensors["hub_imu"], read_gravity=False)
        rim = self._sensor_acceleration(sensors["rim_imu"], read_gravity=False)
        message = f"hub {hub} rim {rim}"

        hub_in_plane = math.hypot(hub[0], hub[1])
        rim_in_plane = math.hypot(rim[0], rim[1])
        # Each sensor turns about the same axis at the same rate on its own radius.
        self.assertAlmostEqual(
            hub_in_plane, angular_speed * angular_speed * radius, delta=GRAVITY_TOLERANCE, msg=message
        )
        self.assertAlmostEqual(
            rim_in_plane,
            angular_speed * angular_speed * (radius + mount_offset),
            delta=GRAVITY_TOLERANCE,
            msg=message,
        )
        # The discriminating assertion. Both sensors share one centre of mass, so without the
        # lever-arm correction this difference is exactly zero however far apart they are mounted.
        self.assertAlmostEqual(
            rim_in_plane - hub_in_plane,
            angular_speed * angular_speed * mount_offset,
            delta=GRAVITY_TOLERANCE,
            msg=message,
        )
        # Both point inward, along each sensor's -x, trailing it slightly for the reason the
        # sibling carousel test records: the differenced acceleration is a step mean while the
        # orientation rotating it is sampled one step behind.
        for name, accel in (("hub", hub), ("rim", rim)):
            trailing_angle_deg = abs(math.degrees(math.atan2(accel[1], -accel[0])))
            self.assertLess(trailing_angle_deg, 5.0, msg=f"{name} direction: {message}")
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
            sensor_reading = self._get_imu_sensor(self.sphere_path + "/sensor").get_sensor_reading()
        self.assertAlmostEqual(sensor_reading.linear_acceleration_z, 0, delta=GRAVITY_TOLERANCE)
        for i in range(200):
            await omni.kit.app.get_app().next_update_async()
            sensor_reading = self._get_imu_sensor(self.sphere_path + "/sensor").get_sensor_reading()
        self.assertAlmostEqual(sensor_reading.linear_acceleration_z, MOON_GRAVITY, delta=GRAVITY_TOLERANCE)

    async def test_gravity_cm(self) -> None:
        """Test gravity cm."""
        if not is_physx_engine():
            return
        await self._setup_ant()
        await self._add_sensor_prims()

        UsdGeom.SetStageMetersPerUnit(self._stage, 0.01)
        SimulationManager.get_physics_scenes()[0].set_gravity(Gf.Vec3f(0.0, 0.0, -CM_GRAVITY))

        await omni.kit.app.get_app().next_update_async()

        await omni.kit.app.get_app().next_update_async()
        self._timeline.play()
        for i in range(100):
            await omni.kit.app.get_app().next_update_async()
            sensor_reading = self._get_imu_sensor(self.sphere_path + "/sensor").get_sensor_reading()
        self.assertAlmostEqual(sensor_reading.linear_acceleration_z, CM_GRAVITY, delta=GRAVITY_TOLERANCE)

    async def test_stop_start(self) -> None:
        """Test stop start."""
        await self._setup_ant()
        await self._add_sensor_prims()

        await omni.kit.app.get_app().next_update_async()

        self._timeline.play()
        await step_simulation(0.5)

        init_reading = self._get_imu_sensor(self.sphere_path + "/sensor").get_sensor_reading()

        self._timeline.stop()
        await omni.kit.app.get_app().next_update_async()

        self._timeline.play()
        await step_simulation(0.5)
        sensor_reading = self._get_imu_sensor(self.sphere_path + "/sensor").get_sensor_reading()

        self.assertAlmostEqual(
            sensor_reading.linear_acceleration_x, init_reading.linear_acceleration_x, delta=ORIENTATION_TOLERANCE
        )
        self.assertAlmostEqual(
            sensor_reading.linear_acceleration_y, init_reading.linear_acceleration_y, delta=ORIENTATION_TOLERANCE
        )
        self.assertAlmostEqual(
            sensor_reading.linear_acceleration_z, init_reading.linear_acceleration_z, delta=ORIENTATION_TOLERANCE
        )

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
        sensor = IMUSensor(
            IMU.create(
                cube_path + "/sensor",
            )
        )

        await omni.kit.app.get_app().next_update_async()

        self._timeline.play()
        for i in range(20):
            await omni.kit.app.get_app().next_update_async()
            sensor_reading = self._get_imu_sensor(cube_path + "/sensor").get_sensor_reading()
        self.assertAlmostEqual(sensor_reading.linear_acceleration_z, 0, delta=GRAVITY_TOLERANCE)
        for i in range(100):
            await omni.kit.app.get_app().next_update_async()
            sensor_reading = self._get_imu_sensor(cube_path + "/sensor").get_sensor_reading()
        self.assertAlmostEqual(sensor_reading.linear_acceleration_z, EARTH_GRAVITY, delta=GRAVITY_TOLERANCE)
        self._timeline.stop()

    async def test_rolling_average_attributes(self) -> None:
        """Verify larger filter windows reduce sensor output variance."""
        await self._setup_ant(physics_rate=400)
        await omni.kit.app.get_app().next_update_async()

        sensor = IMUSensor(
            IMU.create(
                self.sphere_path + "/sphere_imu_1",
                translations=[[0.0, 0.0, 0.0]],
                orientations=[[1.0, 0.0, 0.0, 0.0]],
                linear_acceleration_filter_size=1,
                angular_velocity_filter_size=1,
                orientation_filter_size=1,
            )
        )
        self.assertIsNotNone(sensor)

        sensor_2 = IMUSensor(
            IMU.create(
                self.sphere_path + "/sphere_imu_2",
                translations=[[0.0, 0.0, 0.0]],
                orientations=[[1.0, 0.0, 0.0, 0.0]],
                linear_acceleration_filter_size=20,
                angular_velocity_filter_size=20,
                orientation_filter_size=20,
            )
        )
        self.assertIsNotNone(sensor_2)

        low_rolling_avg_size_reading = []
        high_rolling_avg_size_reading = []

        self._timeline.play()
        # wait for the ant to settle down
        for i in range(200):
            await omni.kit.app.get_app().next_update_async()

        # sample both filter widths on the same motion each step: the small
        # window (1) should fluctuate more than the large window (20)
        for i in range(50):
            await omni.kit.app.get_app().next_update_async()
            low_reading = self._get_imu_sensor(self.sphere_path + "/sphere_imu_1").get_sensor_reading()
            high_reading = self._get_imu_sensor(self.sphere_path + "/sphere_imu_2").get_sensor_reading()

            if not (low_reading.is_valid and high_reading.is_valid):
                continue

            low_rolling_avg_size_reading.append(
                [
                    low_reading.linear_acceleration_x,
                    low_reading.linear_acceleration_y,
                    low_reading.linear_acceleration_z,
                    low_reading.angular_velocity_x,
                    low_reading.angular_velocity_y,
                    low_reading.angular_velocity_z,
                ]
            )
            high_rolling_avg_size_reading.append(
                [
                    high_reading.linear_acceleration_x,
                    high_reading.linear_acceleration_y,
                    high_reading.linear_acceleration_z,
                    high_reading.angular_velocity_x,
                    high_reading.angular_velocity_y,
                    high_reading.angular_velocity_z,
                ]
            )

        # Ensure we collected valid readings
        self.assertGreater(len(low_rolling_avg_size_reading), 0, "No valid sensor readings collected for low filter")
        self.assertGreater(len(high_rolling_avg_size_reading), 0, "No valid sensor readings collected for high filter")

        low_rolling_avg_size_reading = np.array(low_rolling_avg_size_reading)
        high_rolling_avg_size_reading = np.array(high_rolling_avg_size_reading)

        low_rolling_avg_size_1th_percentile = np.percentile(low_rolling_avg_size_reading, 1, axis=0)
        low_rolling_avg_size_99th_percentile = np.percentile(low_rolling_avg_size_reading, 99, axis=0)

        low_rolling_avg_size_diff = np.subtract(
            low_rolling_avg_size_99th_percentile, low_rolling_avg_size_1th_percentile
        )

        high_rolling_avg_size_1th_percentile = np.percentile(high_rolling_avg_size_reading, 1, axis=0)
        high_rolling_avg_size_99th_percentile = np.percentile(high_rolling_avg_size_reading, 99, axis=0)

        high_rolling_avg_size_diff = np.subtract(
            high_rolling_avg_size_99th_percentile, high_rolling_avg_size_1th_percentile
        )

        # low rolling average size is expected to have larger variation than with high rolling average size
        for i in range(len(high_rolling_avg_size_diff)):
            self.assertGreaterEqual(low_rolling_avg_size_diff[i], high_rolling_avg_size_diff[i])

    async def test_sensor_latest_data(self) -> None:
        """Test sensor latest data."""
        await self._setup_ant()
        await self._add_sensor_prims()
        sensor = IMUSensor(
            IMU.create(
                self.sphere_path + "/custom_sensor",
                translations=self.sensor_offsets[4],
                orientations=self.sensor_quatd[4],
            )
        )
        self.assertIsNotNone(sensor)

        await omni.kit.app.get_app().next_update_async()
        self._timeline.play()
        await omni.kit.app.get_app().next_update_async()

        old_time = -1
        for i in range(10):
            await omni.kit.app.get_app().next_update_async()
            latest_sensor_reading = self._get_imu_sensor(self.sphere_path + "/custom_sensor").get_sensor_reading()
            self.assertTrue(latest_sensor_reading.time > old_time)
            old_time = latest_sensor_reading.time

    async def test_invalid_after_prim_delete(self) -> None:
        """Reading a sensor whose prim was deleted mid-simulation returns invalid.

        Replaces the legacy ``test_wrong_sensor_path`` (which constructed a
        backend at a non-existent path); after collapsing the backend layer,
        the equivalent way to exercise the invalid-reading branch is to
        invalidate the prim after creation.
        """
        await self._setup_ant()
        await self._add_sensor_prims()
        sensor_path = self.sphere_path + "/disposable_sensor"
        sensor = IMUSensor(
            IMU.create(
                sensor_path,
                translations=self.sensor_offsets[4],
                orientations=self.sensor_quatd[4],
            )
        )
        self.assertIsNotNone(sensor)

        await omni.kit.app.get_app().next_update_async()
        self._timeline.play()
        await omni.kit.app.get_app().next_update_async()
        for _ in range(5):
            await omni.kit.app.get_app().next_update_async()

        valid_reading = sensor.get_sensor_reading()
        self.assertTrue(valid_reading.is_valid, "Sensor should produce valid readings while the prim exists")

        stage_utils.delete_prim(sensor_path)
        await omni.kit.app.get_app().next_update_async()

        for _ in range(5):
            await omni.kit.app.get_app().next_update_async()

        invalid_reading = sensor.get_sensor_reading()
        self.assertFalse(invalid_reading.is_valid, "Reading should be invalid after the sensor prim is deleted")

    async def test_change_buffer_size(self) -> None:
        """Ensure changing filter widths still yields valid readings."""
        await self._setup_ant()
        await self._add_sensor_prims()
        sensor = IMUSensor(
            IMU.create(
                self.sphere_path + "/custom_sensor",
                translations=self.sensor_offsets[4],
                orientations=self.sensor_quatd[4],
            )
        )
        self.assertIsNotNone(sensor)

        await omni.kit.app.get_app().next_update_async()
        self._timeline.play()
        await omni.kit.app.get_app().next_update_async()

        imu_sensor = prim_utils.get_prim_at_path(self.sphere_path + "/custom_sensor")
        imu_sensor.GetAttribute("linearAccelerationFilterWidth").Set(5)
        imu_sensor.GetAttribute("angularVelocityFilterWidth").Set(5)
        imu_sensor.GetAttribute("orientationFilterWidth").Set(5)
        await omni.kit.app.get_app().next_update_async()

        reading = self._get_imu_sensor(self.sphere_path + "/custom_sensor").get_sensor_reading()
        self.assertTrue(reading.is_valid)

    async def test_imu_rigidbody_grandparent(self) -> None:
        """Validate IMU readings through nested transform hierarchy changes."""
        await self._setup_ant()
        Cube("/World/Cube", sizes=1.0, positions=[10.0, 0.0, 0.5])
        GeomPrim("/World/Cube", apply_collision_apis=True)
        RigidPrim("/World/Cube", masses=[1.0])

        stage_utils.define_prim("/World/Cube/xform", "Xform")
        XformPrim("/World/Cube/xform", translations=[10.0, 0.0, 0.0], reset_xform_op_properties=True)

        sensor = IMUSensor(
            IMU.create(
                "/World/Cube/xform/custom_sensor",
                translations=self.sensor_offsets[4],
                orientations=self.sensor_quatd[4],
            )
        )

        await omni.kit.app.get_app().next_update_async()
        self._timeline.play()
        await step_simulation(0.5)
        custom_reading = self._get_imu_sensor("/World/Cube/xform/custom_sensor").get_sensor_reading()

        self.assertAlmostEqual(custom_reading.linear_acceleration_z, EARTH_GRAVITY, delta=GRAVITY_TOLERANCE)

        self._timeline.stop()
        await omni.kit.app.get_app().next_update_async()

        # Rotate the parent cube about y by -90 degree
        # The x axis points upward
        xform_prim = XformPrim("/World/Cube", reset_xform_op_properties=True)
        xform_prim.set_local_poses(orientations=[0.70711, 0.0, -0.70711, 0.0])
        self._timeline.play()
        await omni.kit.app.get_app().next_update_async()

        await step_simulation(0.5)
        custom_reading = self._get_imu_sensor("/World/Cube/xform/custom_sensor").get_sensor_reading()
        self.assertAlmostEqual(custom_reading.linear_acceleration_x, EARTH_GRAVITY, delta=GRAVITY_TOLERANCE)

        # rotated -90 degress abouty, check if this is correct
        # note: (-0.70711, 0 0.70711, 0) and (0.70711, 0, -0.70711, 0) represent the same angle
        self.assertAlmostEqual(abs(custom_reading.orientation_w), 0.70711, delta=ORIENTATION_TOLERANCE)
        self.assertAlmostEqual(custom_reading.orientation_x, 0.0, delta=ORIENTATION_TOLERANCE)
        self.assertAlmostEqual(abs(custom_reading.orientation_y), 0.70711, delta=ORIENTATION_TOLERANCE)
        self.assertAlmostEqual(custom_reading.orientation_z, 0.0, delta=ORIENTATION_TOLERANCE)
        self.assertAlmostEqual(custom_reading.orientation_w, -custom_reading.orientation_y, delta=ORIENTATION_TOLERANCE)

    async def test_invalid_imu(self) -> None:
        """Test invalid imu."""
        # goal is to make sure an invalid imu doesn't crash the sim
        IMUSensor(
            IMU.create(
                "/World/sensor",
                translations=[[0.0, 0.0, 0.0]],
                orientations=[[1.0, 0.0, 0.0, 0.0]],
            )
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
        sensor = IMUSensor(
            IMU.create(
                "/World/Cube/imu_sensor",
                translations=[[0.0, 0.0, 0.0]],
                orientations=[[1.0, 0.0, 0.0, 0.0]],
            )
        )
        self.assertIsNotNone(sensor)
        await omni.kit.app.get_app().next_update_async()

        stage = stage_utils.get_current_stage()

        # Before simulation starts, sensor should not be registered with the manager
        self.assertFalse(self._timeline.is_playing())

        # Start the timeline to register sensors with the manager
        self._timeline.play()
        await omni.kit.app.get_app().next_update_async()

        # After simulation starts, valid IMU sensor path should return True
        prim = stage.GetPrimAtPath("/World/Cube/imu_sensor")
        self.assertTrue(prim.IsValid() and prim.GetTypeName() == "IsaacImuSensor")

        # Test invalid prim (cube itself, not a sensor) - should return False
        prim = stage.GetPrimAtPath("/World/Cube")
        self.assertFalse(prim.GetTypeName() == "IsaacImuSensor")

        # Test non-existent prim path - should return False
        prim = stage.GetPrimAtPath("/World/NonExistent/sensor")
        self.assertFalse(prim.IsValid())

    async def test_is_sensor_reading_defaults(self) -> None:
        """Verify IMUSensorReading default construction values."""
        reading = IMUSensorReading()
        self.assertEqual(reading.time, 0.0)
        self.assertEqual(reading.linear_acceleration_x, 0.0)
        self.assertEqual(reading.linear_acceleration_y, 0.0)
        self.assertEqual(reading.linear_acceleration_z, 0.0)
        self.assertEqual(reading.angular_velocity_x, 0.0)
        self.assertEqual(reading.angular_velocity_y, 0.0)
        self.assertEqual(reading.angular_velocity_z, 0.0)
        self.assertFalse(reading.is_valid)

    async def test_invalid_prim_sensor_reading(self) -> None:
        """Test that get_sensor_reading returns invalid/zero data for non-sensor prims.

        This test replicates the scenario from test_invalid_imu_sensor_ogn where the
        IMU prim path points to a Cube (not an actual IMU sensor). The sensor backend
        should return invalid readings with zeros, not gravity data.
        """
        await stage_utils.create_new_stage_async()
        SimulationManager.setup_simulation(dt=1.0 / 60.0)

        # Create cube with rigid body - NO ground plane, cube will be in free fall
        Cube("/World/Cube", sizes=1.0, positions=[0.0, 0.0, 10.0])
        GeomPrim("/World/Cube", apply_collision_apis=True)
        RigidPrim("/World/Cube", masses=[1.0])

        # Also create a valid IMU sensor for comparison
        sensor = IMUSensor(
            IMU.create(
                "/World/Cube/imu_sensor",
            )
        )
        self.assertIsNotNone(sensor)
        await omni.kit.app.get_app().next_update_async()

        # Start simulation - cube is in free fall
        self._timeline.play()
        await step_simulation(0.5)

        # Wrapping a non-IMU prim (the cube itself) must raise — the type guard
        # in _PhysicsSensorAuthoring rejects mismatched prim types up front so
        # callers can't accidentally bind an IMU runtime to an arbitrary prim.
        with self.assertRaises(ValueError):
            IMUSensor("/World/Cube")

        # Verify the valid sensor in free fall - IMU should read ~0 acceleration
        # (In free fall, both the sensor and its reference frame fall together)
        sensor_reading = self._get_imu_sensor("/World/Cube/imu_sensor").get_sensor_reading()
        self.assertTrue(sensor_reading.is_valid)
        self.assertAlmostEqual(
            sensor_reading.linear_acceleration_z,
            0.0,
            delta=GRAVITY_TOLERANCE,
            msg=f"In free fall, IMU should read ~0 acceleration, got {sensor_reading.linear_acceleration_z}",
        )

    async def test_rigid_prim_velocities_during_free_fall(self) -> None:
        """Test that RigidPrim.get_velocities() returns correct data during free fall simulation.

        This test verifies that the RigidPrim API correctly reports velocities during physics
        simulation. A falling cube should have increasing negative Z velocity due to gravity.
        """
        await stage_utils.create_new_stage_async()
        SimulationManager.setup_simulation(dt=1.0 / 60.0)

        # Create a cube in free fall (starting at z=10, no ground plane)
        Cube("/World/Cube", sizes=1.0, positions=[0.0, 0.0, 10.0])
        GeomPrim("/World/Cube", apply_collision_apis=True)
        RigidPrim("/World/Cube", masses=[1.0])

        await omni.kit.app.get_app().next_update_async()

        # Create RigidPrim wrapper to get velocities
        rigid_prim = RigidPrim("/World/Cube")

        # Start simulation
        self._timeline.play()

        # Step a few times to let physics initialize
        await step_simulation(0.1)

        # Check that physics tensor entity is valid
        is_valid = rigid_prim.is_physics_tensor_entity_valid()
        self.assertTrue(
            is_valid,
            "RigidPrim.is_physics_tensor_entity_valid() should return True during simulation",
        )

        # Get velocities - in free fall, the cube should have non-zero negative Z velocity
        linear_vel, _ = rigid_prim.get_velocities()

        linear_velocity_z = float(linear_vel.numpy()[0, 2])

        # After 0.1s of free fall: v = g*t ≈ 9.81 * 0.1 ≈ -0.981 m/s (negative because falling)
        # Allow some tolerance for simulation startup
        self.assertLess(
            linear_velocity_z,
            -0.5,
            f"In free fall, Z velocity should be negative (falling), got {linear_velocity_z}",
        )

        # Continue simulation and verify velocity increases
        await step_simulation(0.2)

        linear_vel2, _ = rigid_prim.get_velocities()
        linear_velocity_z2 = float(linear_vel2.numpy()[0, 2])

        # After 0.3s total: v ≈ 9.81 * 0.3 ≈ -2.94 m/s
        self.assertLess(
            linear_velocity_z2,
            linear_velocity_z,
            f"Velocity should increase in magnitude during free fall: {linear_velocity_z2} should be < {linear_velocity_z}",
        )

    async def test_reader_reinitialize_during_play(self) -> None:
        """Force reader.initialize() while IMU is active, then step physics.

        Reproduces a crash where ImuSensorImpl holds a stale
        IRigidBodyDataView pointer after the reader destroys all views.
        """
        await stage_utils.create_new_stage_async()
        await omni.kit.app.get_app().next_update_async()
        stage_utils.set_stage_units(meters_per_unit=1.0)
        SimulationManager.setup_simulation(dt=1.0 / 60.0)

        cube_path = "/World/Cube"
        Cube(cube_path, sizes=1.0, positions=[0.0, 0.0, 2.0])
        GeomPrim(cube_path, apply_collision_apis=True)
        RigidPrim(cube_path, masses=[1.0])

        sensor = IMUSensor(IMU.create(cube_path + "/imu_sensor"))
        self.assertIsNotNone(sensor)
        sensor_path = cube_path + "/imu_sensor"
        await omni.kit.app.get_app().next_update_async()

        from isaacsim.core.experimental.prims import _prims_reader

        reader = _prims_reader.acquire_prim_data_reader_interface()
        try:
            self._timeline.play()
            await step_simulation(0.25)

            backend = IMUSensor(sensor_path)
            reading = backend.get_sensor_reading()
            self.assertTrue(reading.is_valid)

            stage = omni.usd.get_context().get_stage()
            stage_id = UsdUtils.StageCache.Get().GetId(stage).ToLongInt()
            gen_before = reader.get_generation()

            reader.initialize(stage_id, -1)
            self.assertGreater(reader.get_generation(), gen_before)

            await step_simulation(0.25)

            reading_after = backend.get_sensor_reading()
            self.assertTrue(
                reading_after.is_valid,
                "IMU reading should remain valid after reader.initialize() rebuilds views",
            )
        finally:
            _prims_reader.release_prim_data_reader_interface(reader)

    async def test_multiple_reader_reinitializations(self) -> None:
        """Reinitialize the reader several times in rapid succession while IMU is active."""
        await stage_utils.create_new_stage_async()
        await omni.kit.app.get_app().next_update_async()
        stage_utils.set_stage_units(meters_per_unit=1.0)
        SimulationManager.setup_simulation(dt=1.0 / 60.0)

        cube_path = "/World/Cube"
        Cube(cube_path, sizes=1.0, positions=[0.0, 0.0, 2.0])
        GeomPrim(cube_path, apply_collision_apis=True)
        RigidPrim(cube_path, masses=[1.0])

        sensor = IMUSensor(IMU.create(cube_path + "/imu_sensor"))
        self.assertIsNotNone(sensor)
        sensor_path = cube_path + "/imu_sensor"
        await omni.kit.app.get_app().next_update_async()

        from isaacsim.core.experimental.prims import _prims_reader

        reader = _prims_reader.acquire_prim_data_reader_interface()
        try:
            self._timeline.play()
            await step_simulation(0.25)

            backend = IMUSensor(sensor_path)
            self.assertTrue(backend.get_sensor_reading().is_valid)

            stage = omni.usd.get_context().get_stage()
            stage_id = UsdUtils.StageCache.Get().GetId(stage).ToLongInt()

            for _ in range(3):
                reader.initialize(stage_id, -1)
                await step_simulation(0.1)

            self.assertTrue(
                backend.get_sensor_reading().is_valid,
                "IMU reading should remain valid after multiple reader.initialize() calls",
            )
        finally:
            _prims_reader.release_prim_data_reader_interface(reader)


class TestIMUSensorRuntimeData(omni.kit.test.AsyncTestCase):
    """Test IMUSensor runtime data helpers."""

    # Before running each test
    async def setUp(self) -> None:
        """Create a Nova Carter IMU runtime scene with ground and obstacle cubes."""
        await stage_utils.create_new_stage_async()
        SimulationManager.setup_simulation(dt=1.0 / 60.0)
        self._timeline = omni.timeline.get_timeline_interface()
        GroundPlane("/World/defaultGroundPlane", positions=[0.0, 0.0, 0.0])
        assets_root_path = await get_assets_root_path_async()
        asset_path = assets_root_path + "/Isaac/Robots/NVIDIA/NovaCarter/nova_carter.usd"
        stage_utils.add_reference_to_stage(usd_path=asset_path, path="/World/Carter")

        XformPrim("/World/Carter", positions=[0, 0.0, 0.5], reset_xform_op_properties=True)

        self._imu = IMUSensor(path="/World/Carter/chassis_link/Imu_Sensor")

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
        """Stop playback, invalidate physics, and wait for stage loading to finish."""
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
        data = self._imu.get_data()
        for key in ["time", "physics_step", "linear_acceleration", "angular_velocity", "orientation"]:
            self.assertTrue(key in data)
        data = self._imu.get_data(read_gravity=False)
        for key in ["time", "physics_step", "linear_acceleration", "angular_velocity", "orientation"]:
            self.assertTrue(key in data)
        return

    async def test_data_values_gravity_toggle(self) -> None:
        """Verify a resting body reads +g with gravity and 0 without, capturing both first."""
        await reset_timeline(self._timeline, steps=2)
        data = None
        for _ in range(60):
            data = self._imu.get_data()
            if abs(float(data["linear_acceleration"][2]) - EARTH_GRAVITY) <= GRAVITY_TOLERANCE:
                break
            await omni.kit.app.get_app().next_update_async()
        self.assertIsNotNone(data)
        self.assertGreater(data["time"], 0.0)
        orientation_norm = float(np.linalg.norm(data["orientation"]))

        # Capture both toggle states before asserting, mirroring how callers compare the two
        # readings.
        acceleration_with_gravity = float(self._imu.get_data(read_gravity=True)["linear_acceleration"][2])
        acceleration_without_gravity = float(self._imu.get_data(read_gravity=False)["linear_acceleration"][2])

        # A resting body measures the gravity reaction as specific force and has zero
        # coordinate acceleration.
        self.assertAlmostEqual(acceleration_with_gravity, EARTH_GRAVITY, delta=GRAVITY_TOLERANCE)
        self.assertAlmostEqual(acceleration_without_gravity, 0.0, delta=GRAVITY_TOLERANCE)

        self.assertAlmostEqual(orientation_norm, 1.0, delta=ORIENTATION_TOLERANCE)

    async def test_data_frame_is_independent(self) -> None:
        """Verify `get_data` returns a frame no later call can disturb."""
        await reset_timeline(self._timeline, steps=2)
        first = None
        for _ in range(60):
            first = self._imu.get_data()
            if abs(float(first["linear_acceleration"][2]) - EARTH_GRAVITY) <= GRAVITY_TOLERANCE:
                break
            await omni.kit.app.get_app().next_update_async()
        self.assertIsNotNone(first)
        self.assertGreater(first["time"], 0.0)

        # A later call must not touch an earlier result: neither the dict nor any of its arrays.
        linear_acceleration = first["linear_acceleration"]
        second = self._imu.get_data(read_gravity=False)
        self.assertIsNot(first, second)
        for key in ["linear_acceleration", "angular_velocity", "orientation"]:
            self.assertIsNot(first[key], second[key])
        self.assertAlmostEqual(float(linear_acceleration[2]), EARTH_GRAVITY, delta=GRAVITY_TOLERANCE)
        self.assertAlmostEqual(float(second["linear_acceleration"][2]), 0.0, delta=GRAVITY_TOLERANCE)

        # The channels keep their documented shapes and dtype after the slicing.
        for key, size in [("linear_acceleration", 3), ("angular_velocity", 3), ("orientation", 4)]:
            self.assertEqual(first[key].shape, (size,), msg=key)
            self.assertEqual(first[key].dtype, np.float32, msg=key)

        # The returned arrays are writable and caller-owned, and writing one must not disturb
        # any other frame. Asserting against `second` rather than a fresh call is what makes
        # this non-vacuous: a fresh call refreshes from a valid reading and would mask aliasing.
        for key in ["linear_acceleration", "angular_velocity", "orientation"]:
            self.assertTrue(first[key].flags.writeable, msg=key)
        linear_acceleration[2] = -1.0
        self.assertAlmostEqual(float(second["linear_acceleration"][2]), 0.0, delta=GRAVITY_TOLERANCE)

        # Nor may it leak into the sensor's own bookkeeping and be re-served to a later caller.
        self.assertAlmostEqual(
            float(self._imu.get_data()["linear_acceleration"][2]), EARTH_GRAVITY, delta=GRAVITY_TOLERANCE
        )

    async def test_timeline_reset(self) -> None:
        """Verify frame updates are consistent across timeline stop/start."""
        await omni.kit.app.get_app().next_update_async()
        await omni.kit.app.get_app().next_update_async()
        data = self._imu.get_data()
        self.assertGreater(data["physics_step"], 0)
        self.assertGreater(data["time"], 0)

        await reset_timeline(self._timeline, steps=1)
        data = self._imu.get_data()
        self.assertAlmostEqual(data["time"], 0.05, delta=0.01)
        self.assertTrue(data["physics_step"] == 3)
        return

    async def test_filter_size_parameters(self) -> None:
        """Test filter size parameters."""
        filter_imu = IMUSensor(
            IMU(
                "/World/Carter/chassis_link/Imu_Sensor_filtered",
                linear_acceleration_filter_size=5,
                angular_velocity_filter_size=7,
                orientation_filter_size=9,
            )
        )
        imu_prim = prim_utils.get_prim_at_path(filter_imu.imu.paths[0])
        self.assertEqual(imu_prim.GetAttribute("linearAccelerationFilterWidth").Get(), 5)
        self.assertEqual(imu_prim.GetAttribute("angularVelocityFilterWidth").Get(), 7)
        self.assertEqual(imu_prim.GetAttribute("orientationFilterWidth").Get(), 9)
