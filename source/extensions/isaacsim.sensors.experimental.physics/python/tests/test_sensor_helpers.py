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

"""Tests for sensor creation via the class-based create() API.

These tests verify that ContactSensor(Contact.create()), IMUSensor(IMU.create()), and
RaycastSensor(Raycast.create()) create prims at the correct paths, especially when the
stage has a default prim.

The key behavior being tested is that sensor paths should NOT be prepended with
the stage's default prim - they should be created exactly where specified.
"""

import carb
import isaacsim.core.experimental.utils.prim as prim_utils
import isaacsim.core.experimental.utils.stage as stage_utils
import omni.isaac.IsaacSensorSchema as IsaacSensorSchema
import omni.kit.test
import omni.usd
from isaacsim.core.experimental.objects import Cube
from isaacsim.sensors.experimental.physics import (
    IMU,
    Contact,
    ContactSensor,
    IMUSensor,
    Raycast,
    RaycastSensor,
)
from isaacsim.storage.native import get_assets_root_path_async
from pxr import Gf, PhysxSchema, UsdPhysics

from .common import setup_ant_scene


class TestSensorCreate(omni.kit.test.AsyncTestCase):
    """Test sensor class create() methods for correct path handling."""

    async def setUp(self):
        self._assets_root_path = await get_assets_root_path_async()
        if self._assets_root_path is None:
            carb.log_error("Could not find Isaac Sim assets folder")
            return

    async def tearDown(self):
        stage_utils.close_stage()
        await omni.kit.app.get_app().next_update_async()

    async def _create_empty_stage_with_world(self):
        await stage_utils.create_new_stage_async()
        stage = omni.usd.get_context().get_stage()
        stage_utils.define_prim("/World", type_name="Xform")
        await omni.kit.app.get_app().next_update_async()
        return stage

    async def _create_stage_with_default_prim(self):
        await stage_utils.create_new_stage_async()
        stage = omni.usd.get_context().get_stage()
        default_prim = stage_utils.define_prim("/DefaultRoot", type_name="Xform")
        stage.SetDefaultPrim(default_prim)
        stage_utils.define_prim("/World", type_name="Xform")
        await omni.kit.app.get_app().next_update_async()
        return stage

    # ==================== IMU Sensor Tests ====================

    async def test_imu_sensor_path_no_default_prim(self):
        await self._create_empty_stage_with_world()

        cube = Cube("/World/Cube", sizes=[1.0])
        UsdPhysics.RigidBodyAPI.Apply(cube.prims[0])
        await omni.kit.app.get_app().next_update_async()

        expected_path = "/World/Cube/Imu_Sensor"
        sensor = IMUSensor(IMU.create("/World/Cube/Imu_Sensor"))

        self.assertIsNotNone(sensor)
        self.assertEqual(sensor.imu.paths[0], expected_path)
        self.assertTrue(prim_utils.get_prim_at_path(expected_path).IsValid())

    async def test_imu_sensor_path_with_default_prim(self):
        stage = await self._create_stage_with_default_prim()

        self.assertTrue(stage.HasDefaultPrim())
        self.assertEqual(stage.GetDefaultPrim().GetPath().pathString, "/DefaultRoot")

        cube = Cube("/World/Cube", sizes=[1.0])
        UsdPhysics.RigidBodyAPI.Apply(cube.prims[0])
        await omni.kit.app.get_app().next_update_async()

        expected_path = "/World/Cube/Imu_Sensor"
        sensor = IMUSensor(IMU.create("/World/Cube/Imu_Sensor"))

        self.assertIsNotNone(sensor)
        created_path = sensor.imu.paths[0]

        self.assertFalse(
            created_path.startswith("/DefaultRoot/"),
            f"Sensor path should NOT start with default prim. Got: {created_path}",
        )
        self.assertEqual(created_path, expected_path)
        self.assertFalse(prim_utils.get_prim_at_path("/DefaultRoot/World/Cube/Imu_Sensor").IsValid())

    async def test_imu_sensor_with_ant_scene(self):
        await setup_ant_scene()
        stage = stage_utils.get_current_stage()

        self.assertTrue(stage.HasDefaultPrim())
        self.assertEqual(stage.GetDefaultPrim().GetPath().pathString, "/Ant")

        stage_utils.define_prim("/World", type_name="Xform")
        cube = Cube("/World/TestCube", sizes=[1.0])
        UsdPhysics.RigidBodyAPI.Apply(cube.prims[0])
        await omni.kit.app.get_app().next_update_async()

        expected_path = "/World/TestCube/Imu_Sensor"
        sensor = IMUSensor(IMU.create("/World/TestCube/Imu_Sensor"))

        self.assertIsNotNone(sensor)
        created_path = sensor.imu.paths[0]
        self.assertFalse(created_path.startswith("/Ant/"))
        self.assertEqual(created_path, expected_path)

    async def test_imu_sensor_nested_hierarchy(self):
        await self._create_stage_with_default_prim()

        stage_utils.define_prim("/World/Level1", type_name="Xform")
        stage_utils.define_prim("/World/Level1/Level2", type_name="Xform")
        cube = Cube("/World/Level1/Level2/Cube", sizes=[1.0])
        UsdPhysics.RigidBodyAPI.Apply(cube.prims[0])
        await omni.kit.app.get_app().next_update_async()

        expected_path = "/World/Level1/Level2/Cube/DeepSensor"
        sensor = IMUSensor(IMU.create("/World/Level1/Level2/Cube/DeepSensor"))

        self.assertIsNotNone(sensor)
        self.assertEqual(sensor.imu.paths[0], expected_path)

    async def test_imu_sensor_attributes(self):
        await self._create_empty_stage_with_world()

        cube = Cube("/World/Cube", sizes=[1.0])
        UsdPhysics.RigidBodyAPI.Apply(cube.prims[0])
        await omni.kit.app.get_app().next_update_async()

        sensor = IMUSensor(
            IMU.create(
                "/World/Cube/CustomImu",
                translations=[[1.0, 2.0, 3.0]],
                orientations=[[0.707, 0.707, 0.0, 0.0]],
                linear_acceleration_filter_size=5,
                angular_velocity_filter_size=3,
                orientation_filter_size=7,
            )
        )

        self.assertIsNotNone(sensor)
        usd_prim = sensor.imu.prims[0]
        schema_prim = IsaacSensorSchema.IsaacImuSensor(usd_prim)
        self.assertEqual(schema_prim.GetLinearAccelerationFilterWidthAttr().Get(), 5)
        self.assertEqual(schema_prim.GetAngularVelocityFilterWidthAttr().Get(), 3)
        self.assertEqual(schema_prim.GetOrientationFilterWidthAttr().Get(), 7)
        self.assertEqual(usd_prim.GetTypeName(), "IsaacImuSensor")

    async def test_imu_sensor_world_positions(self):
        """Verify ``positions`` (world-frame) successfully routes through XformPrim.set_world_poses."""
        await self._create_empty_stage_with_world()

        cube = Cube("/World/Cube", sizes=[1.0])
        UsdPhysics.RigidBodyAPI.Apply(cube.prims[0])
        await omni.kit.app.get_app().next_update_async()

        sensor = IMUSensor(IMU.create("/World/Cube/PositionedImu", positions=[[5.0, 0.0, 0.0]]))

        self.assertIsNotNone(sensor)
        world_positions, _ = sensor.imu.get_world_poses()
        positions_np = world_positions.numpy()
        self.assertAlmostEqual(float(positions_np[0][0]), 5.0, places=4)

    async def test_sensor_multi_prim_path_rejected(self):
        """A path matching multiple existing prims raises ValueError."""
        await self._create_empty_stage_with_world()

        cube_a = Cube("/World/CubeA", sizes=[1.0])
        cube_b = Cube("/World/CubeB", sizes=[1.0])
        UsdPhysics.RigidBodyAPI.Apply(cube_a.prims[0])
        UsdPhysics.RigidBodyAPI.Apply(cube_b.prims[0])
        UsdPhysics.CollisionAPI.Apply(cube_a.prims[0])
        UsdPhysics.CollisionAPI.Apply(cube_b.prims[0])
        await omni.kit.app.get_app().next_update_async()

        IsaacSensorSchema.IsaacImuSensor.Define(stage_utils.get_current_stage(), "/World/CubeA/Imu")
        IsaacSensorSchema.IsaacImuSensor.Define(stage_utils.get_current_stage(), "/World/CubeB/Imu")
        IsaacSensorSchema.IsaacContactSensor.Define(stage_utils.get_current_stage(), "/World/CubeA/Contact")
        IsaacSensorSchema.IsaacContactSensor.Define(stage_utils.get_current_stage(), "/World/CubeB/Contact")
        IsaacSensorSchema.IsaacRaycastSensor.Define(stage_utils.get_current_stage(), "/World/CubeA/Raycast")
        IsaacSensorSchema.IsaacRaycastSensor.Define(stage_utils.get_current_stage(), "/World/CubeB/Raycast")
        await omni.kit.app.get_app().next_update_async()

        with self.assertRaises(ValueError):
            IMUSensor("/World/Cube.*/Imu")
        with self.assertRaises(ValueError):
            ContactSensor("/World/Cube.*/Contact")
        with self.assertRaises(ValueError):
            RaycastSensor("/World/Cube.*/Raycast")

    async def test_sensor_wrap_wrong_prim_type_rejected(self):
        """Wrapping a prim whose type doesn't match the sensor's _PRIM_TYPE raises ValueError."""
        await self._create_empty_stage_with_world()

        cube = Cube("/World/Cube", sizes=[1.0])
        UsdPhysics.RigidBodyAPI.Apply(cube.prims[0])
        UsdPhysics.CollisionAPI.Apply(cube.prims[0])
        await omni.kit.app.get_app().next_update_async()

        # The cube is a Cube prim, not an IsaacImuSensor / IsaacContactSensor /
        # IsaacRaycastSensor — wrapping it from any of the three runtimes must
        # raise rather than silently bind the runtime to the wrong prim type.
        with self.assertRaises(ValueError):
            IMUSensor("/World/Cube")
        with self.assertRaises(ValueError):
            ContactSensor("/World/Cube")
        with self.assertRaises(ValueError):
            RaycastSensor("/World/Cube")

    async def test_imu_sensor_positions_translations_conflict(self):
        """Specifying both ``positions`` and ``translations`` raises ValueError."""
        await self._create_empty_stage_with_world()

        cube = Cube("/World/Cube", sizes=[1.0])
        UsdPhysics.RigidBodyAPI.Apply(cube.prims[0])
        await omni.kit.app.get_app().next_update_async()

        with self.assertRaises(ValueError):
            IMUSensor(
                IMU.create(
                    "/World/Cube/ConflictImu",
                    positions=[[1.0, 0.0, 0.0]],
                    translations=[[1.0, 0.0, 0.0]],
                )
            )

    async def test_imu_sensor_enabled_attribute(self):
        await self._create_empty_stage_with_world()

        cube = Cube("/World/Cube", sizes=[1.0])
        UsdPhysics.RigidBodyAPI.Apply(cube.prims[0])
        await omni.kit.app.get_app().next_update_async()

        sensor = IMUSensor(IMU.create("/World/Cube/Sensor"))

        self.assertIsNotNone(sensor)
        base_sensor = IsaacSensorSchema.IsaacBaseSensor(sensor.imu.prims[0])
        self.assertTrue(base_sensor.GetEnabledAttr().Get())

    async def test_imu_sensor_requires_parent(self):
        await self._create_empty_stage_with_world()

        with self.assertRaises(RuntimeError):
            IMUSensor(IMU.create("/Imu_Sensor"))

    # ==================== Contact Sensor Tests ====================

    async def test_contact_sensor_path_no_default_prim(self):
        await self._create_empty_stage_with_world()

        cube = Cube("/World/Cube", sizes=[1.0])
        UsdPhysics.RigidBodyAPI.Apply(cube.prims[0])
        UsdPhysics.CollisionAPI.Apply(cube.prims[0])
        await omni.kit.app.get_app().next_update_async()

        expected_path = "/World/Cube/Contact_Sensor"
        sensor = ContactSensor(Contact.create("/World/Cube/Contact_Sensor"))

        self.assertIsNotNone(sensor)
        self.assertEqual(sensor.contact.paths[0], expected_path)

    async def test_contact_sensor_path_with_default_prim(self):
        await self._create_stage_with_default_prim()

        cube = Cube("/World/Cube", sizes=[1.0])
        UsdPhysics.RigidBodyAPI.Apply(cube.prims[0])
        UsdPhysics.CollisionAPI.Apply(cube.prims[0])
        await omni.kit.app.get_app().next_update_async()

        expected_path = "/World/Cube/Contact_Sensor"
        sensor = ContactSensor(Contact.create("/World/Cube/Contact_Sensor"))

        self.assertIsNotNone(sensor)
        created_path = sensor.contact.paths[0]
        self.assertFalse(created_path.startswith("/DefaultRoot/"))
        self.assertEqual(created_path, expected_path)

    async def test_contact_sensor_attributes(self):
        stage = await self._create_empty_stage_with_world()

        cube = Cube("/World/Cube", sizes=[1.0])
        UsdPhysics.RigidBodyAPI.Apply(cube.prims[0])
        UsdPhysics.CollisionAPI.Apply(cube.prims[0])
        await omni.kit.app.get_app().next_update_async()

        sensor = ContactSensor(
            Contact.create(
                "/World/Cube/CustomContact",
                min_threshold=10.0,
                max_threshold=5000.0,
                color=Gf.Vec4f(1.0, 0.0, 0.0, 1.0),
                radius=0.5,
            )
        )

        self.assertIsNotNone(sensor)
        self.assertAlmostEqual(sensor.contact.get_min_threshold(), 10.0, places=5)
        self.assertAlmostEqual(sensor.contact.get_max_threshold(), 5000.0, places=5)
        self.assertAlmostEqual(sensor.contact.get_radius(), 0.5, places=5)

        parent_prim = stage.GetPrimAtPath("/World/Cube")
        self.assertTrue(parent_prim.HasAPI(PhysxSchema.PhysxContactReportAPI))

    async def test_contact_sensor_nested_under_rigid_body_keeps_requested_path(self):
        """Nested contact sensors are authored at the requested path and report on the rigid ancestor."""
        stage = await self._create_empty_stage_with_world()

        cube = Cube("/World/Cube", sizes=[1.0])
        UsdPhysics.RigidBodyAPI.Apply(cube.prims[0])
        UsdPhysics.CollisionAPI.Apply(cube.prims[0])
        mount = stage_utils.define_prim("/World/Cube/SensorMount", type_name="Xform")
        await omni.kit.app.get_app().next_update_async()

        contact = Contact.create("/World/Cube/SensorMount/Contact", min_threshold=2.0)

        self.assertEqual(contact.paths[0], "/World/Cube/SensorMount/Contact")
        self.assertTrue(stage.GetPrimAtPath("/World/Cube/SensorMount/Contact").IsValid())
        self.assertFalse(stage.GetPrimAtPath("/World/Cube/Contact").IsValid())
        self.assertTrue(stage.GetPrimAtPath("/World/Cube").HasAPI(PhysxSchema.PhysxContactReportAPI))
        self.assertFalse(mount.HasAPI(PhysxSchema.PhysxContactReportAPI))

    async def test_contact_sensor_collision_only_parent_rejected(self):
        """Contact authoring rejects parents that the C++ runtime cannot bind as rigid bodies."""
        await self._create_empty_stage_with_world()

        collider = Cube("/World/StaticCollider", sizes=[1.0])
        UsdPhysics.CollisionAPI.Apply(collider.prims[0])
        await omni.kit.app.get_app().next_update_async()

        with self.assertRaises(ValueError):
            Contact.create("/World/StaticCollider/Contact")

    async def test_contact_sensor_requires_parent(self):
        await self._create_empty_stage_with_world()

        with self.assertRaises(RuntimeError):
            ContactSensor(Contact.create("/Contact_Sensor"))

    # ==================== Raycast Sensor Tests ====================

    async def test_raycast_sensor_requires_parent(self):
        await self._create_empty_stage_with_world()

        with self.assertRaises(RuntimeError):
            RaycastSensor(Raycast.create("/Raycast_Sensor"))

    async def test_raycast_sensor_path_with_default_prim(self):
        await self._create_stage_with_default_prim()

        cube = Cube("/World/Cube", sizes=[1.0])
        UsdPhysics.RigidBodyAPI.Apply(cube.prims[0])
        await omni.kit.app.get_app().next_update_async()

        expected_path = "/World/Cube/Raycast_Sensor"
        sensor = RaycastSensor(
            Raycast.create(
                "/World/Cube/Raycast_Sensor",
                ray_origins=[[0, 0, 0]],
                ray_directions=[[1, 0, 0]],
            )
        )

        self.assertIsNotNone(sensor)
        created_path = sensor.raycast.paths[0]
        self.assertFalse(created_path.startswith("/DefaultRoot/"))
        self.assertEqual(created_path, expected_path)

    async def test_raycast_sensor_attributes(self):
        await self._create_empty_stage_with_world()

        cube = Cube("/World/Cube", sizes=[1.0])
        UsdPhysics.RigidBodyAPI.Apply(cube.prims[0])
        await omni.kit.app.get_app().next_update_async()

        origins = [[0, 0, 0], [0, 0, 0]]
        directions = [[1, 0, 0], [0, 1, 0]]

        sensor = RaycastSensor(
            Raycast.create(
                "/World/Cube/Raycast",
                min_range=0.5,
                max_range=50.0,
                ray_origins=origins,
                ray_directions=directions,
                output_frame="WORLD",
                report_hit_prim_paths=True,
            )
        )

        self.assertIsNotNone(sensor)
        usd_prim = sensor.raycast.prims[0]
        schema_prim = IsaacSensorSchema.IsaacRaycastSensor(usd_prim)
        self.assertAlmostEqual(schema_prim.GetMinRangeAttr().Get(), 0.5, places=5)
        self.assertAlmostEqual(schema_prim.GetMaxRangeAttr().Get(), 50.0, places=5)
        self.assertEqual(schema_prim.GetNumRaysAttr().Get(), 2)
        self.assertEqual(schema_prim.GetOutputFrameOfReferenceAttr().Get(), "WORLD")
        self.assertTrue(schema_prim.GetReportHitPrimPathsAttr().Get())

    # ==================== Path Edge Cases ====================

    async def test_path_with_trailing_slash(self):
        await self._create_empty_stage_with_world()

        cube = Cube("/World/Cube", sizes=[1.0])
        UsdPhysics.RigidBodyAPI.Apply(cube.prims[0])
        await omni.kit.app.get_app().next_update_async()

        expected_path = "/World/Cube/Sensor"
        sensor = IMUSensor(IMU.create("/World/Cube/Sensor/"))

        self.assertIsNotNone(sensor)
        self.assertEqual(sensor.imu.paths[0], expected_path)
        self.assertNotIn("//", sensor.imu.paths[0])

    async def test_unique_path_generation(self):
        await self._create_empty_stage_with_world()

        cube = Cube("/World/Cube", sizes=[1.0])
        UsdPhysics.RigidBodyAPI.Apply(cube.prims[0])
        await omni.kit.app.get_app().next_update_async()

        sensor1 = IMUSensor(IMU.create("/World/Cube/Sensor"))
        sensor2 = IMUSensor(IMU.create("/World/Cube/Sensor"))

        self.assertIsNotNone(sensor1)
        self.assertIsNotNone(sensor2)

        path1 = sensor1.imu.paths[0]
        path2 = sensor2.imu.paths[0]
        self.assertNotEqual(path1, path2, "Duplicate sensors should get unique paths")
        self.assertEqual(path1, "/World/Cube/Sensor")
        self.assertTrue(path2.startswith("/World/Cube/Sensor"))

    # ==================== Authoring / Runtime Split ====================

    async def test_authoring_only_imu_no_backend(self):
        """``IMU`` (authoring) can be instantiated without bringing up a backend."""
        await self._create_empty_stage_with_world()
        cube = Cube("/World/Cube", sizes=[1.0])
        UsdPhysics.RigidBodyAPI.Apply(cube.prims[0])
        await omni.kit.app.get_app().next_update_async()

        imu = IMU.create("/World/Cube/AuthoringImu", linear_acceleration_filter_size=7)

        self.assertIsNotNone(imu)
        self.assertFalse(hasattr(imu, "_backend"), "Authoring class should not have a runtime backend")
        self.assertEqual(imu.paths[0], "/World/Cube/AuthoringImu")
        usd_prim = imu.prims[0]
        schema_prim = IsaacSensorSchema.IsaacImuSensor(usd_prim)
        self.assertEqual(schema_prim.GetLinearAccelerationFilterWidthAttr().Get(), 7)

    async def test_runtime_wraps_existing_authoring_object(self):
        """``IMUSensor(my_imu)`` reuses an existing authoring instance."""
        await self._create_empty_stage_with_world()
        cube = Cube("/World/Cube", sizes=[1.0])
        UsdPhysics.RigidBodyAPI.Apply(cube.prims[0])
        await omni.kit.app.get_app().next_update_async()

        imu = IMU.create("/World/Cube/SharedImu")
        sensor = IMUSensor(imu)

        self.assertIs(sensor.imu, imu)
        self.assertEqual(sensor.imu.paths[0], imu.paths[0])
        # Runtime now owns the C++ interface directly via _PhysicsSensorRuntimeBase
        # (no separate backend object); verify the lifecycle attributes are present.
        self.assertTrue(hasattr(sensor, "_iface"), "Runtime should hold a C++ interface handle")
        self.assertTrue(hasattr(sensor, "_sensor_created"), "Runtime should track sensor creation state")

    async def test_contact_runtime_forwards_to_authoring_via_attribute(self):
        """Calling ``sensor.contact.set_radius(...)`` writes USD without warnings."""
        import warnings

        await self._create_empty_stage_with_world()
        cube = Cube("/World/Cube", sizes=[1.0])
        UsdPhysics.RigidBodyAPI.Apply(cube.prims[0])
        UsdPhysics.CollisionAPI.Apply(cube.prims[0])
        await omni.kit.app.get_app().next_update_async()

        sensor = ContactSensor(Contact.create("/World/Cube/Contact", radius=0.1))
        with warnings.catch_warnings():
            warnings.simplefilter("error", DeprecationWarning)
            sensor.contact.set_radius(0.2)
            self.assertAlmostEqual(sensor.contact.get_radius(), 0.2, places=5)

    async def test_authoring_only_contact_no_backend(self):
        """``Contact`` (authoring) can be instantiated without bringing up a backend."""
        await self._create_empty_stage_with_world()
        cube = Cube("/World/Cube", sizes=[1.0])
        UsdPhysics.RigidBodyAPI.Apply(cube.prims[0])
        UsdPhysics.CollisionAPI.Apply(cube.prims[0])
        await omni.kit.app.get_app().next_update_async()

        contact = Contact.create("/World/Cube/AuthoringContact", min_threshold=2.5, radius=0.05)

        self.assertIsNotNone(contact)
        self.assertFalse(hasattr(contact, "_backend"))
        self.assertAlmostEqual(contact.get_min_threshold(), 2.5, places=5)
        self.assertAlmostEqual(contact.get_radius(), 0.05, places=5)

    async def test_authoring_only_raycast_no_backend(self):
        """``Raycast`` (authoring) can be instantiated without bringing up a backend."""
        await self._create_empty_stage_with_world()
        cube = Cube("/World/Cube", sizes=[1.0])
        UsdPhysics.RigidBodyAPI.Apply(cube.prims[0])
        await omni.kit.app.get_app().next_update_async()

        raycast = Raycast.create(
            "/World/Cube/AuthoringRaycast",
            ray_origins=[[0.0, 0.0, 0.0], [0.0, 0.0, 0.0]],
            ray_directions=[[1.0, 0.0, 0.0], [0.0, 1.0, 0.0]],
        )

        self.assertIsNotNone(raycast)
        self.assertFalse(hasattr(raycast, "_backend"))
        schema_prim = IsaacSensorSchema.IsaacRaycastSensor(raycast.prims[0])
        self.assertEqual(schema_prim.GetNumRaysAttr().Get(), 2)

    async def test_raycast_runtime_property_and_create(self):
        """``Raycast.create`` and the ``raycast`` typed property work end-to-end."""
        await self._create_empty_stage_with_world()
        cube = Cube("/World/Cube", sizes=[1.0])
        UsdPhysics.RigidBodyAPI.Apply(cube.prims[0])
        await omni.kit.app.get_app().next_update_async()

        sensor = RaycastSensor(
            Raycast.create(
                "/World/Cube/Raycast",
                ray_origins=[[0.0, 0.0, 0.0]],
                ray_directions=[[1.0, 0.0, 0.0]],
            )
        )

        self.assertIsNotNone(sensor)
        self.assertIsInstance(sensor.raycast, Raycast)
        self.assertIs(sensor.raycast, sensor.authoring_object)

    async def test_contact_runtime_create_classmethod(self):
        """``Contact.create`` followed by ``ContactSensor(authoring)`` constructs both layers."""
        await self._create_empty_stage_with_world()
        cube = Cube("/World/Cube", sizes=[1.0])
        UsdPhysics.RigidBodyAPI.Apply(cube.prims[0])
        UsdPhysics.CollisionAPI.Apply(cube.prims[0])
        await omni.kit.app.get_app().next_update_async()

        sensor = ContactSensor(Contact.create("/World/Cube/Contact", radius=0.07, min_threshold=3.0))

        self.assertIsNotNone(sensor)
        self.assertIsInstance(sensor.contact, Contact)
        self.assertIs(sensor.contact, sensor.authoring_object)
        self.assertAlmostEqual(sensor.contact.get_radius(), 0.07, places=5)

    async def test_raycast_authoring_update_time_offsets_only(self):
        """Wrapping an existing raycast and updating only ``ray_time_offsets`` works.

        Regression guard: ``_validate_ray_arrays`` must not assume ``num_rays = 1``
        when wrapping an existing prim that already has multiple rays.
        """
        await self._create_empty_stage_with_world()
        cube = Cube("/World/Cube", sizes=[1.0])
        UsdPhysics.RigidBodyAPI.Apply(cube.prims[0])
        await omni.kit.app.get_app().next_update_async()

        Raycast.create(
            "/World/Cube/Raycast",
            ray_origins=[[0.0, 0.0, 0.0], [0.0, 0.0, 0.0], [0.0, 0.0, 0.0]],
            ray_directions=[[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]],
        )

        # Wrap-and-update: only time offsets, length matches existing num_rays.
        wrapped = Raycast(
            "/World/Cube/Raycast",
            ray_time_offsets=[0.0, 0.01, 0.02],
        )
        self.assertIsNotNone(wrapped)
