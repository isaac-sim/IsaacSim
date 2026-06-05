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

"""Unit tests for PhysX sensor commands.

Tests the functionality of creating different types of PhysX sensors through commands.
"""

import omni.isaac.RangeSensorSchema as RangeSensorSchema
import omni.kit.commands
import omni.kit.test
import omni.usd
from pxr import Gf, Sdf, UsdGeom


class TestPhysXSensorCommands(omni.kit.test.AsyncTestCase):
    """Test cases for PhysX sensor creation commands."""

    async def setUp(self) -> None:
        """Create a new stage for each test."""
        await omni.usd.get_context().new_stage_async()
        self.stage = omni.usd.get_context().get_stage()

    async def tearDown(self) -> None:
        """Clean up the stage after each test."""
        await omni.usd.get_context().close_stage_async()

    async def test_range_sensor_create_prim_default(self) -> None:
        """Test creating a range sensor prim with default parameters."""
        path = "/TestSensor"
        parent = ""
        schema_type = RangeSensorSchema.Lidar  # Use Lidar instead of non-existent RangeSensor
        translation = Gf.Vec3d(0.0, 0.0, 0.0)
        orientation = Gf.Quatd(1.0, 0.0, 0.0, 0.0)
        visibility = False
        min_range = 0.4
        max_range = 100.0
        draw_points = False
        draw_lines = False

        _, schema_obj = omni.kit.commands.execute(
            "RangeSensorCreatePrim",
            path=path,
            parent=parent,
            schema_type=schema_type,
            translation=translation,
            orientation=orientation,
            visibility=visibility,
            min_range=min_range,
            max_range=max_range,
            draw_points=draw_points,
            draw_lines=draw_lines,
        )

        self.assertIsNotNone(schema_obj)
        # Get the prim from the schema object for validation
        prim = schema_obj.GetPrim()
        # Check that the prim is a valid USD prim
        self.assertTrue(prim.IsValid())
        self.assertEqual(prim.GetPath(), Sdf.Path("/TestSensor"))
        self.assertEqual(prim.GetTypeName(), "Lidar")

        # Verify attributes are set correctly using the schema object
        # We can now use the schema_obj directly since that's what the command returns
        enabled_attr = schema_obj.GetEnabledAttr()
        self.assertTrue(enabled_attr.IsValid())
        self.assertTrue(enabled_attr.Get())

        draw_points_attr = schema_obj.GetDrawPointsAttr()
        self.assertTrue(draw_points_attr.IsValid())
        self.assertEqual(draw_points_attr.Get(), draw_points)

        draw_lines_attr = schema_obj.GetDrawLinesAttr()
        self.assertTrue(draw_lines_attr.IsValid())
        self.assertEqual(draw_lines_attr.Get(), draw_lines)

        min_range_attr = schema_obj.GetMinRangeAttr()
        self.assertTrue(min_range_attr.IsValid())
        self.assertAlmostEqual(min_range_attr.Get(), min_range, places=6)

        max_range_attr = schema_obj.GetMaxRangeAttr()
        self.assertTrue(max_range_attr.IsValid())
        self.assertAlmostEqual(max_range_attr.Get(), max_range, places=6)

        # Verify transform operations
        xformable = UsdGeom.Xformable(prim)
        self.assertTrue(prim.IsValid())
        ops = xformable.GetOrderedXformOps()
        self.assertGreater(len(ops), 0)

    async def test_range_sensor_create_prim_with_parent(self) -> None:
        """Test creating a range sensor prim with a parent."""
        # Create a parent Xform
        parent_path = "/World"
        parent = UsdGeom.Xform.Define(self.stage, parent_path)

        path = "/TestSensor"
        schema_type = RangeSensorSchema.Lidar  # Use Lidar instead of non-existent RangeSensor
        translation = Gf.Vec3d(1.0, 2.0, 3.0)
        orientation = Gf.Quatd(0.707, 0.0, 0.707, 0.0)

        _, schema_obj = omni.kit.commands.execute(
            "RangeSensorCreatePrim",
            path=path,
            parent=parent_path,
            schema_type=schema_type,
            translation=translation,
            orientation=orientation,
        )

        self.assertIsNotNone(schema_obj)
        # Get the prim from the schema object for validation
        prim = schema_obj.GetPrim()
        # Check that the prim is a valid USD prim
        self.assertTrue(prim.IsValid())
        self.assertTrue(prim.GetPath().HasPrefix(parent_path))

    async def test_range_sensor_create_lidar_default(self) -> None:
        """Test creating a lidar sensor with default parameters."""
        _, schema_obj = omni.kit.commands.execute("RangeSensorCreateLidar")

        self.assertIsNotNone(schema_obj)
        # Get the prim from the schema object for validation
        prim = schema_obj.GetPrim()
        # Check that the prim is a valid USD prim
        self.assertTrue(prim.IsValid())
        self.assertEqual(prim.GetPath(), Sdf.Path("/Lidar"))

        # Verify default attributes using the schema object
        # We can now use the schema_obj directly since that's what the command returns
        enabled_attr = schema_obj.GetEnabledAttr()
        self.assertTrue(enabled_attr.IsValid())
        self.assertTrue(enabled_attr.Get())

        draw_points_attr = schema_obj.GetDrawPointsAttr()
        self.assertTrue(draw_points_attr.IsValid())
        self.assertEqual(draw_points_attr.Get(), False)

        draw_lines_attr = schema_obj.GetDrawLinesAttr()
        self.assertTrue(draw_lines_attr.IsValid())
        self.assertEqual(draw_lines_attr.Get(), False)

        min_range_attr = schema_obj.GetMinRangeAttr()
        self.assertTrue(min_range_attr.IsValid())
        self.assertAlmostEqual(min_range_attr.Get(), 0.4, places=6)

        max_range_attr = schema_obj.GetMaxRangeAttr()
        self.assertTrue(max_range_attr.IsValid())
        self.assertAlmostEqual(max_range_attr.Get(), 100.0, places=6)

    async def test_range_sensor_create_lidar_custom_parameters(self) -> None:
        """Test creating a lidar sensor with custom parameters including translation and orientation."""
        custom_translation = Gf.Vec3d(5.0, 10.0, 15.0)
        custom_orientation = Gf.Quatd(0.707, 0.0, 0.707, 0.0)  # 90 degree rotation around Y
        custom_horizontal_fov = 180.0
        custom_vertical_fov = 45.0

        _, schema_obj = omni.kit.commands.execute(
            "RangeSensorCreateLidar",
            path="/CustomLidar",
            translation=custom_translation,
            orientation=custom_orientation,
            horizontal_fov=custom_horizontal_fov,
            vertical_fov=custom_vertical_fov,
        )

        self.assertIsNotNone(schema_obj)
        # Get the prim from the schema object for validation
        prim = schema_obj.GetPrim()
        self.assertTrue(prim.IsValid())
        self.assertEqual(prim.GetPath(), Sdf.Path("/CustomLidar"))

        # Verify transform operations
        xformable = UsdGeom.Xformable(prim)
        self.assertTrue(prim.IsValid())
        ops = xformable.GetOrderedXformOps()
        self.assertGreater(len(ops), 0)

        # Verify actual transform values
        transform_matrix = xformable.GetLocalTransformation()
        expected_translation = Gf.Vec3d(custom_translation)
        expected_orientation = Gf.Quatd(custom_orientation)

        # Check translation (extract from transform matrix)
        actual_translation = transform_matrix.ExtractTranslation()
        self.assertAlmostEqual(actual_translation[0], expected_translation[0], places=6)
        self.assertAlmostEqual(actual_translation[1], expected_translation[1], places=6)
        self.assertAlmostEqual(actual_translation[2], expected_translation[2], places=6)

        # Check orientation (extract from transform matrix)
        actual_orientation = transform_matrix.ExtractRotationQuat()
        # Quaternion comparison (account for sign differences)
        # Use the dot product of the quaternions to check if they represent the same rotation
        dot_product = (
            actual_orientation.GetReal() * expected_orientation.GetReal()
            + actual_orientation.GetImaginary()[0] * expected_orientation.GetImaginary()[0]
            + actual_orientation.GetImaginary()[1] * expected_orientation.GetImaginary()[1]
            + actual_orientation.GetImaginary()[2] * expected_orientation.GetImaginary()[2]
        )
        self.assertAlmostEqual(abs(dot_product), 1.0, places=3)

        # Verify lidar-specific attributes using the schema object
        horizontal_fov_attr = schema_obj.GetHorizontalFovAttr()
        self.assertTrue(horizontal_fov_attr.IsValid())
        self.assertAlmostEqual(horizontal_fov_attr.Get(), custom_horizontal_fov, places=6)

        vertical_fov_attr = schema_obj.GetVerticalFovAttr()
        self.assertTrue(vertical_fov_attr.IsValid())
        self.assertAlmostEqual(vertical_fov_attr.Get(), custom_vertical_fov, places=6)

    async def test_range_sensor_create_generic_default(self) -> None:
        """Test creating a generic range sensor with default parameters."""
        _, schema_obj = omni.kit.commands.execute("RangeSensorCreateGeneric")

        self.assertIsNotNone(schema_obj)
        # Get the prim from the schema object for validation
        prim = schema_obj.GetPrim()
        # Check that the prim is a valid USD prim
        self.assertTrue(prim.IsValid())
        self.assertEqual(prim.GetPath(), Sdf.Path("/GenericSensor"))

        # Verify default attributes using the schema object
        # We can now use the schema_obj directly since that's what the command returns
        enabled_attr = schema_obj.GetEnabledAttr()
        self.assertTrue(enabled_attr.IsValid())
        self.assertTrue(enabled_attr.Get())

        draw_points_attr = schema_obj.GetDrawPointsAttr()
        self.assertTrue(draw_points_attr.IsValid())
        self.assertEqual(draw_points_attr.Get(), False)

        draw_lines_attr = schema_obj.GetDrawLinesAttr()
        self.assertTrue(draw_lines_attr.IsValid())
        self.assertEqual(draw_lines_attr.Get(), False)

        min_range_attr = schema_obj.GetMinRangeAttr()
        self.assertTrue(min_range_attr.IsValid())
        self.assertAlmostEqual(min_range_attr.Get(), 0.4, places=6)

        max_range_attr = schema_obj.GetMaxRangeAttr()
        self.assertTrue(max_range_attr.IsValid())
        self.assertAlmostEqual(max_range_attr.Get(), 100.0, places=6)

        sampling_rate_attr = schema_obj.GetSamplingRateAttr()
        self.assertTrue(sampling_rate_attr.IsValid())
        self.assertEqual(sampling_rate_attr.Get(), 60)

    async def test_range_sensor_create_generic_custom_parameters(self) -> None:
        """Test creating a generic range sensor with custom parameters including translation and orientation."""
        custom_translation = Gf.Vec3d(-3.0, 7.0, -12.0)
        custom_orientation = Gf.Quatd(0.5, 0.5, 0.5, 0.5)  # Complex rotation
        custom_sampling_rate = 120

        _, schema_obj = omni.kit.commands.execute(
            "RangeSensorCreateGeneric",
            path="/CustomGeneric",
            translation=custom_translation,
            orientation=custom_orientation,
            sampling_rate=custom_sampling_rate,
        )

        self.assertIsNotNone(schema_obj)
        # Get the prim from the schema object for validation
        prim = schema_obj.GetPrim()
        self.assertTrue(prim.IsValid())
        self.assertEqual(prim.GetPath(), Sdf.Path("/CustomGeneric"))

        # Verify transform operations
        xformable = UsdGeom.Xformable(prim)
        self.assertTrue(prim.IsValid())
        ops = xformable.GetOrderedXformOps()
        self.assertGreater(len(ops), 0)

        # Verify actual transform values
        transform_matrix = xformable.GetLocalTransformation()
        expected_translation = Gf.Vec3d(custom_translation)
        expected_orientation = Gf.Quatd(custom_orientation)

        # Check translation (extract from transform matrix)
        actual_translation = transform_matrix.ExtractTranslation()
        self.assertAlmostEqual(actual_translation[0], expected_translation[0], places=6)
        self.assertAlmostEqual(actual_translation[1], expected_translation[1], places=6)
        self.assertAlmostEqual(actual_translation[2], expected_translation[2], places=6)

        # Check orientation (extract from transform matrix)
        actual_orientation = transform_matrix.ExtractRotationQuat()
        # Quaternion comparison (account for sign differences)
        dot_product = (
            actual_orientation.GetReal() * expected_orientation.GetReal()
            + actual_orientation.GetImaginary()[0] * expected_orientation.GetImaginary()[0]
            + actual_orientation.GetImaginary()[1] * expected_orientation.GetImaginary()[1]
            + actual_orientation.GetImaginary()[2] * expected_orientation.GetImaginary()[2]
        )
        self.assertAlmostEqual(abs(dot_product), 1.0, places=3)

        # Verify generic-specific attributes using the schema object
        sampling_rate_attr = schema_obj.GetSamplingRateAttr()
        self.assertTrue(sampling_rate_attr.IsValid())
        self.assertEqual(sampling_rate_attr.Get(), custom_sampling_rate)

    async def test_isaac_sensor_create_light_beam_sensor_default(self) -> None:
        """Test creating a light beam sensor with default parameters."""
        _, schema_obj = omni.kit.commands.execute("IsaacSensorCreateLightBeamSensor")

        self.assertIsNotNone(schema_obj)
        # Get the prim from the schema object for validation
        prim = schema_obj.GetPrim()
        # Check that the prim is a valid USD prim
        self.assertTrue(prim.IsValid())
        self.assertEqual(prim.GetPath(), Sdf.Path("/LightBeam_Sensor"))

        # Verify default attributes using the schema object
        # We can now use the schema_obj directly since that's what the command returns
        num_rays_attr = schema_obj.GetNumRaysAttr()
        self.assertTrue(num_rays_attr.IsValid())
        self.assertEqual(num_rays_attr.Get(), 1)

        curtain_length_attr = schema_obj.GetCurtainLengthAttr()
        self.assertTrue(curtain_length_attr.IsValid())
        self.assertEqual(curtain_length_attr.Get(), 0.0)

        forward_axis_attr = schema_obj.GetForwardAxisAttr()
        self.assertTrue(forward_axis_attr.IsValid())
        self.assertEqual(forward_axis_attr.Get(), Gf.Vec3d(1, 0, 0))

        curtain_axis_attr = schema_obj.GetCurtainAxisAttr()
        self.assertTrue(curtain_axis_attr.IsValid())
        self.assertEqual(curtain_axis_attr.Get(), Gf.Vec3d(0, 0, 1))

        # Note: IsaacLightBeamSensor doesn't have minRange and maxRange attributes
        # These are only available on RangeSensor schemas

    async def test_isaac_sensor_create_light_beam_sensor_custom_parameters(self) -> None:
        """Test creating a light beam sensor with custom parameters."""
        _, schema_obj = omni.kit.commands.execute(
            "IsaacSensorCreateLightBeamSensor",
            path="/CustomLightBeam",
            translation=Gf.Vec3d(10.0, 20.0, 30.0),
            orientation=Gf.Quatd(0.5, 0.5, 0.5, 0.5),
            num_rays=5,
            curtain_length=2.0,
            forward_axis=Gf.Vec3d(0, 1, 0),
            curtain_axis=Gf.Vec3d(1, 0, 0),
            min_range=1.0,
            max_range=200.0,
            draw_points=True,
            draw_lines=True,
        )

        self.assertIsNotNone(schema_obj)
        # Get the prim from the schema object for validation
        prim = schema_obj.GetPrim()
        # Check that the prim is a valid USD prim
        self.assertTrue(prim.IsValid())
        self.assertEqual(prim.GetPath(), Sdf.Path("/CustomLightBeam"))

        # Verify custom parameters using the schema object
        # We can now use the schema_obj directly since that's what the command returns
        num_rays_attr = schema_obj.GetNumRaysAttr()
        self.assertTrue(num_rays_attr.IsValid())
        self.assertEqual(num_rays_attr.Get(), 5)

        curtain_length_attr = schema_obj.GetCurtainLengthAttr()
        self.assertTrue(curtain_length_attr.IsValid())
        self.assertEqual(curtain_length_attr.Get(), 2.0)

        forward_axis_attr = schema_obj.GetForwardAxisAttr()
        self.assertTrue(forward_axis_attr.IsValid())
        self.assertEqual(forward_axis_attr.Get(), Gf.Vec3d(0, 1, 0))

        curtain_axis_attr = schema_obj.GetCurtainAxisAttr()
        self.assertTrue(curtain_axis_attr.IsValid())
        self.assertEqual(curtain_axis_attr.Get(), Gf.Vec3d(1, 0, 0))

        # Note: IsaacLightBeamSensor doesn't have minRange, maxRange, drawPoints, or drawLines attributes
        # These are only available on RangeSensor schemas

    async def test_isaac_sensor_create_light_beam_sensor_multiple_rays(self) -> None:
        """Test creating a light beam sensor with multiple rays and curtain length."""
        _, schema_obj = omni.kit.commands.execute(
            "IsaacSensorCreateLightBeamSensor",
            path="/MultiRayLightBeam",
            num_rays=10,
            curtain_length=5.0,
            draw_points=True,
            draw_lines=True,
        )

        self.assertIsNotNone(schema_obj)
        # Get the prim from the schema object for validation
        prim = schema_obj.GetPrim()
        # Check that the prim is a valid USD prim
        self.assertTrue(prim.IsValid())

        # Verify multiple ray configuration using the schema object
        # We can now use the schema_obj directly since that's what the command returns
        num_rays_attr = schema_obj.GetNumRaysAttr()
        self.assertTrue(num_rays_attr.IsValid())
        self.assertEqual(num_rays_attr.Get(), 10)

        curtain_length_attr = schema_obj.GetCurtainLengthAttr()
        self.assertTrue(curtain_length_attr.IsValid())
        self.assertEqual(curtain_length_attr.Get(), 5.0)

    async def test_isaac_sensor_create_light_beam_sensor_invalid_rays(self) -> None:
        """Test creating a light beam sensor with invalid ray configuration."""
        _, schema_obj = omni.kit.commands.execute(
            "IsaacSensorCreateLightBeamSensor",
            path="/InvalidLightBeam",
            num_rays=5,
            curtain_length=0.0,  # Invalid: multiple rays but no curtain length
            draw_points=True,
            draw_lines=True,
        )

        # Should return None due to validation error
        self.assertIsNone(schema_obj)

    async def test_sensor_undo_functionality(self) -> None:
        """Test that sensor creation commands support undo operations."""
        # Create a sensor
        _, schema_obj = omni.kit.commands.execute("RangeSensorCreateLidar")

        self.assertIsNotNone(schema_obj)
        # Get the prim from the schema object for validation
        prim = schema_obj.GetPrim()
        # Check that the prim is a valid USD prim
        self.assertTrue(prim.IsValid())

        # Verify the sensor exists
        self.assertTrue(prim.IsValid())

        # Undo the creation
        omni.kit.undo.undo()

        # Verify the sensor was removed
        self.assertFalse(prim.IsValid())

    async def test_sensor_with_parent_undo(self) -> None:
        """Test undo functionality when creating sensor with parent."""
        # Create a parent Xform
        parent_path = "/World"
        parent = UsdGeom.Xform.Define(self.stage, parent_path)

        # Create a sensor with parent
        _, schema_obj = omni.kit.commands.execute(
            "RangeSensorCreateLidar",
            path="/TestLidar",
            parent=parent_path,
        )

        self.assertIsNotNone(schema_obj)
        # Get the prim from the schema object for validation
        prim = schema_obj.GetPrim()
        # Check that the prim is a valid USD prim
        self.assertTrue(prim.IsValid())
        self.assertTrue(prim.GetPath().HasPrefix(parent_path))

        # Undo the creation
        omni.kit.undo.undo()

        # Verify the sensor was removed
        self.assertFalse(prim.IsValid())

    async def test_multiple_sensors_creation(self) -> None:
        """Test creating multiple sensors of different types."""
        # Create a lidar sensor
        _, lidar_schema = omni.kit.commands.execute("RangeSensorCreateLidar")

        self.assertIsNotNone(lidar_schema)
        # Get the prim from the schema object for validation
        lidar_prim = lidar_schema.GetPrim()
        # Check that the prim is a valid USD prim
        self.assertTrue(lidar_prim.IsValid())

        # Create a generic sensor
        _, generic_schema = omni.kit.commands.execute("RangeSensorCreateGeneric")

        self.assertIsNotNone(generic_schema)
        # Get the prim from the schema object for validation
        generic_prim = generic_schema.GetPrim()
        # Check that the prim is a valid USD prim
        self.assertTrue(generic_prim.IsValid())

        # Create a light beam sensor
        _, light_beam_schema = omni.kit.commands.execute(
            "IsaacSensorCreateLightBeamSensor",
            path="/TestLightBeam",
            num_rays=3,
            curtain_length=1.0,
        )

        self.assertIsNotNone(light_beam_schema)
        # Get the prim from the schema object for validation
        light_beam_prim = light_beam_schema.GetPrim()
        # Check that the prim is a valid USD prim
        self.assertTrue(light_beam_prim.IsValid())

        # Verify all sensors exist and are different
        self.assertNotEqual(lidar_prim.GetPath(), generic_prim.GetPath())
        self.assertNotEqual(lidar_prim.GetPath(), light_beam_prim.GetPath())
        self.assertNotEqual(generic_prim.GetPath(), light_beam_prim.GetPath())

    async def test_sensor_attributes_consistency(self) -> None:
        """Test that sensor attributes are consistently set across different sensor types."""
        # Create different types of sensors
        _, range_schema = omni.kit.commands.execute(
            "RangeSensorCreatePrim",
            path="/TestRange",
            schema_type=RangeSensorSchema.Lidar,
            min_range=2.0,
            max_range=150.0,
            draw_points=True,
            draw_lines=False,
        )

        _, lidar_schema = omni.kit.commands.execute(
            "RangeSensorCreateLidar",
            path="/TestLidar2",
            min_range=2.0,
            max_range=150.0,
            draw_points=True,
            draw_lines=False,
        )

        _, generic_schema = omni.kit.commands.execute(
            "RangeSensorCreateGeneric",
            path="/TestGeneric2",
            min_range=2.0,
            max_range=150.0,
            draw_points=True,
            draw_lines=False,
        )

        # Verify all sensors have consistent base attributes
        for schema_obj, name in [(range_schema, "Range"), (lidar_schema, "Lidar"), (generic_schema, "Generic")]:
            self.assertIsNotNone(schema_obj, f"Failed to create {name} sensor")

            # Check that the prim is a valid USD prim
            prim = schema_obj.GetPrim()
            self.assertTrue(prim.IsValid(), f"{name} prim is not valid")

            # Get the appropriate schema object for attribute access
            if name == "Range" or name == "Lidar":
                schema_obj = RangeSensorSchema.Lidar(prim)
            else:  # Generic
                schema_obj = RangeSensorSchema.Generic(prim)

            # Verify consistent attribute values
            min_range_attr = schema_obj.GetMinRangeAttr()
            self.assertTrue(min_range_attr.IsValid(), f"{name} missing minRange attribute")
            self.assertAlmostEqual(min_range_attr.Get(), 2.0, f"{name} minRange mismatch")

            max_range_attr = schema_obj.GetMaxRangeAttr()
            self.assertTrue(max_range_attr.IsValid(), f"{name} missing maxRange attribute")
            self.assertAlmostEqual(max_range_attr.Get(), 150.0, f"{name} maxRange mismatch")

            draw_points_attr = schema_obj.GetDrawPointsAttr()
            self.assertTrue(draw_points_attr.IsValid(), f"{name} missing drawPoints attribute")
            self.assertEqual(draw_points_attr.Get(), True, f"{name} drawPoints mismatch")

            draw_lines_attr = schema_obj.GetDrawLinesAttr()
            self.assertTrue(draw_lines_attr.IsValid(), f"{name} missing drawLines attribute")
            self.assertEqual(draw_lines_attr.Get(), False, f"{name} drawLines mismatch")

    async def test_isaac_sensor_create_light_beam_sensor_custom_transform(self) -> None:
        """Test creating a light beam sensor with custom translation and orientation."""
        custom_translation = Gf.Vec3d(2.5, -5.0, 8.0)
        custom_orientation = Gf.Quatd(0.866, 0.0, 0.5, 0.0)  # 60 degree rotation around Y
        custom_num_rays = 5
        custom_curtain_length = 3.0

        _, schema_obj = omni.kit.commands.execute(
            "IsaacSensorCreateLightBeamSensor",
            path="/CustomLightBeam",
            translation=custom_translation,
            orientation=custom_orientation,
            num_rays=custom_num_rays,
            curtain_length=custom_curtain_length,
        )

        self.assertIsNotNone(schema_obj)
        # Get the prim from the schema object for validation
        prim = schema_obj.GetPrim()
        self.assertTrue(prim.IsValid())
        self.assertEqual(prim.GetPath(), Sdf.Path("/CustomLightBeam"))

        # Verify transform operations
        xformable = UsdGeom.Xformable(prim)
        self.assertTrue(prim.IsValid())
        ops = xformable.GetOrderedXformOps()
        self.assertGreater(len(ops), 0)

        # Verify actual transform values
        transform_matrix = xformable.GetLocalTransformation()
        expected_translation = Gf.Vec3d(custom_translation)
        expected_orientation = Gf.Quatd(custom_orientation)

        # Check translation (extract from transform matrix)
        actual_translation = transform_matrix.ExtractTranslation()
        self.assertAlmostEqual(actual_translation[0], expected_translation[0], places=6)
        self.assertAlmostEqual(actual_translation[1], expected_translation[1], places=6)
        self.assertAlmostEqual(actual_translation[2], expected_translation[2], places=6)

        # Check orientation (extract from transform matrix)
        actual_orientation = transform_matrix.ExtractRotationQuat()
        # Quaternion comparison (account for sign differences)
        # Use the dot product of the quaternions to check if they represent the same rotation
        dot_product = (
            actual_orientation.GetReal() * expected_orientation.GetReal()
            + actual_orientation.GetImaginary()[0] * expected_orientation.GetImaginary()[0]
            + actual_orientation.GetImaginary()[1] * expected_orientation.GetImaginary()[1]
            + actual_orientation.GetImaginary()[2] * expected_orientation.GetImaginary()[2]
        )
        self.assertAlmostEqual(abs(dot_product), 1.0, places=3)

        # Verify light beam sensor-specific attributes using the schema object
        num_rays_attr = schema_obj.GetNumRaysAttr()
        self.assertTrue(num_rays_attr.IsValid())
        self.assertEqual(num_rays_attr.Get(), custom_num_rays)

        curtain_length_attr = schema_obj.GetCurtainLengthAttr()
        self.assertTrue(curtain_length_attr.IsValid())
        self.assertAlmostEqual(curtain_length_attr.Get(), custom_curtain_length, places=6)

    async def test_range_sensor_create_prim_custom_parameters(self) -> None:
        """Test creating a range sensor prim with custom parameters including translation and orientation."""
        path = "/CustomTestSensor"
        parent = ""
        schema_type = RangeSensorSchema.Lidar
        custom_translation = Gf.Vec3d(10.0, 20.0, 30.0)
        custom_orientation = Gf.Quatd(0.707, 0.0, 0.0, 0.707)  # 90 degree rotation around X
        visibility = True
        min_range = 1.0
        max_range = 200.0
        draw_points = True
        draw_lines = True

        _, schema_obj = omni.kit.commands.execute(
            "RangeSensorCreatePrim",
            path=path,
            parent=parent,
            schema_type=schema_type,
            translation=custom_translation,
            orientation=custom_orientation,
            visibility=visibility,
            min_range=min_range,
            max_range=max_range,
            draw_points=draw_points,
            draw_lines=draw_lines,
        )

        self.assertIsNotNone(schema_obj)
        # Get the prim from the schema object for validation
        prim = schema_obj.GetPrim()
        self.assertTrue(prim.IsValid())
        self.assertEqual(prim.GetPath(), Sdf.Path(path))

        # Verify transform operations
        xformable = UsdGeom.Xformable(prim)
        self.assertTrue(prim.IsValid())
        ops = xformable.GetOrderedXformOps()
        self.assertGreater(len(ops), 0)

        # Verify actual transform values
        transform_matrix = xformable.GetLocalTransformation()
        expected_translation = Gf.Vec3d(custom_translation)
        expected_orientation = Gf.Quatd(custom_orientation)

        # Check translation (extract from transform matrix)
        actual_translation = transform_matrix.ExtractTranslation()
        self.assertAlmostEqual(actual_translation[0], expected_translation[0], places=6)
        self.assertAlmostEqual(actual_translation[1], expected_translation[1], places=6)
        self.assertAlmostEqual(actual_translation[2], expected_translation[2], places=6)

        # Check orientation (extract from transform matrix)
        actual_orientation = transform_matrix.ExtractRotationQuat()
        # Quaternion comparison (account for sign differences)
        # Use the dot product of the quaternions to check if they represent the same rotation
        dot_product = (
            actual_orientation.GetReal() * expected_orientation.GetReal()
            + actual_orientation.GetImaginary()[0] * expected_orientation.GetImaginary()[0]
            + actual_orientation.GetImaginary()[1] * expected_orientation.GetImaginary()[1]
            + actual_orientation.GetImaginary()[2] * expected_orientation.GetImaginary()[2]
        )
        self.assertAlmostEqual(abs(dot_product), 1.0, places=3)

        # Verify custom attributes using the schema object
        enabled_attr = schema_obj.GetEnabledAttr()
        self.assertTrue(enabled_attr.IsValid())
        self.assertTrue(enabled_attr.Get())

        min_range_attr = schema_obj.GetMinRangeAttr()
        self.assertTrue(min_range_attr.IsValid())
        self.assertAlmostEqual(min_range_attr.Get(), min_range, places=6)

        max_range_attr = schema_obj.GetMaxRangeAttr()
        self.assertTrue(max_range_attr.IsValid())
        self.assertAlmostEqual(max_range_attr.Get(), max_range, places=6)
