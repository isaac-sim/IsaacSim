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
# ruff: noqa: D102, D103, ANN001, ANN201
# Test methods are self-documenting: their names describe the assertion under
# test and they always return None implicitly.

"""Tests for codeless sensor schemas (rangeSensorSchema, isaacSensorSchema) and Python compatibility wrappers."""

from __future__ import annotations

import importlib
import sys
import warnings

import omni.kit.app
import omni.kit.test
import omni.usd
from pxr import Plug, Tf, UsdGeom


class TestRangeSensorSchemaRegistration(omni.kit.test.AsyncTestCase):
    """Verify that the rangeSensorSchema codeless schema plugin is registered and types resolve correctly."""

    async def setUp(self):
        await omni.usd.get_context().new_stage_async()
        await omni.kit.app.get_app().next_update_async()
        self._stage = omni.usd.get_context().get_stage()

    # -- Plugin registration --------------------------------------------------

    def test_range_sensor_plugin_registered(self):
        """The rangeSensorSchema plugin must be registered in the Plug registry."""
        plugin = Plug.Registry().GetPluginWithName("rangeSensorSchema")
        self.assertIsNotNone(plugin, "rangeSensorSchema plugin not found in Plug.Registry")

    # -- TfType registration --------------------------------------------------

    def test_lidar_tf_type(self):
        tf_type = Tf.Type.FindByName("RangeSensorLidar")
        self.assertFalse(tf_type.isUnknown, "TfType 'RangeSensorLidar' not found")

    def test_generic_tf_type(self):
        tf_type = Tf.Type.FindByName("RangeSensorGeneric")
        self.assertFalse(tf_type.isUnknown, "TfType 'RangeSensorGeneric' not found")

    def test_range_sensor_base_tf_type(self):
        tf_type = Tf.Type.FindByName("RangeSensorRangeSensor")
        self.assertFalse(tf_type.isUnknown, "TfType 'RangeSensorRangeSensor' not found")

    # -- Prim creation with DefinePrim ---------------------------------------

    def test_define_lidar_prim(self):
        """stage.DefinePrim with typeName 'Lidar' must produce a valid typed prim."""
        prim = self._stage.DefinePrim("/TestLidar", "Lidar")
        self.assertTrue(prim.IsValid())
        self.assertEqual(prim.GetTypeName(), "Lidar")

    def test_define_generic_prim(self):
        prim = self._stage.DefinePrim("/TestGeneric", "Generic")
        self.assertTrue(prim.IsValid())
        self.assertEqual(prim.GetTypeName(), "Generic")

    # -- Fallback attribute values from generatedSchema.usda ------------------

    def test_lidar_fallback_enabled(self):
        prim = self._stage.DefinePrim("/TestLidar", "Lidar")
        attr = prim.GetAttribute("enabled")
        self.assertTrue(attr.IsValid(), "'enabled' attribute not found on Lidar prim")
        self.assertEqual(attr.Get(), True)

    def test_lidar_fallback_min_range(self):
        prim = self._stage.DefinePrim("/TestLidar", "Lidar")
        val = prim.GetAttribute("minRange").Get()
        self.assertAlmostEqual(val, 0.4, places=5)

    def test_lidar_fallback_max_range(self):
        prim = self._stage.DefinePrim("/TestLidar", "Lidar")
        val = prim.GetAttribute("maxRange").Get()
        self.assertAlmostEqual(val, 100.0, places=5)

    def test_lidar_fallback_horizontal_fov(self):
        prim = self._stage.DefinePrim("/TestLidar", "Lidar")
        val = prim.GetAttribute("horizontalFov").Get()
        self.assertAlmostEqual(val, 360.0, places=5)

    def test_lidar_fallback_draw_points(self):
        prim = self._stage.DefinePrim("/TestLidar", "Lidar")
        self.assertEqual(prim.GetAttribute("drawPoints").Get(), False)

    def test_lidar_fallback_draw_lines(self):
        prim = self._stage.DefinePrim("/TestLidar", "Lidar")
        self.assertEqual(prim.GetAttribute("drawLines").Get(), False)

    def test_generic_fallback_sampling_rate(self):
        prim = self._stage.DefinePrim("/TestGeneric", "Generic")
        val = prim.GetAttribute("samplingRate").Get()
        self.assertEqual(val, 60)

    # -- IsA checks via GetTypeName ------------------------------------------

    def test_lidar_is_xformable(self):
        """Lidar inherits from UsdGeomXformable through RangeSensor."""
        prim = self._stage.DefinePrim("/TestLidar", "Lidar")
        self.assertTrue(prim.IsA(UsdGeom.Xformable))


class TestIsaacSensorSchemaRegistration(omni.kit.test.AsyncTestCase):
    """Verify that the isaacSensorSchema codeless schema plugin is registered and types resolve correctly."""

    async def setUp(self):
        await omni.usd.get_context().new_stage_async()
        await omni.kit.app.get_app().next_update_async()
        self._stage = omni.usd.get_context().get_stage()

    # -- Plugin registration --------------------------------------------------

    def test_isaac_sensor_plugin_registered(self):
        plugin = Plug.Registry().GetPluginWithName("isaacSensorSchema")
        self.assertIsNotNone(plugin, "isaacSensorSchema plugin not found in Plug.Registry")

    # -- TfType registration --------------------------------------------------

    def test_contact_sensor_tf_type(self):
        tf_type = Tf.Type.FindByName("IsaacSensorIsaacContactSensor")
        self.assertFalse(tf_type.isUnknown, "TfType 'IsaacSensorIsaacContactSensor' not found")

    def test_imu_sensor_tf_type(self):
        tf_type = Tf.Type.FindByName("IsaacSensorIsaacImuSensor")
        self.assertFalse(tf_type.isUnknown, "TfType 'IsaacSensorIsaacImuSensor' not found")

    def test_light_beam_sensor_tf_type(self):
        tf_type = Tf.Type.FindByName("IsaacSensorIsaacLightBeamSensor")
        self.assertFalse(tf_type.isUnknown, "TfType 'IsaacSensorIsaacLightBeamSensor' not found")

    def test_raycast_sensor_tf_type(self):
        tf_type = Tf.Type.FindByName("IsaacSensorIsaacRaycastSensor")
        self.assertFalse(tf_type.isUnknown, "TfType 'IsaacSensorIsaacRaycastSensor' not found")

    def test_rtx_lidar_api_tf_type(self):
        tf_type = Tf.Type.FindByName("IsaacSensorIsaacRtxLidarSensorAPI")
        self.assertFalse(tf_type.isUnknown, "TfType 'IsaacSensorIsaacRtxLidarSensorAPI' not found")

    def test_rtx_radar_api_tf_type(self):
        tf_type = Tf.Type.FindByName("IsaacSensorIsaacRtxRadarSensorAPI")
        self.assertFalse(tf_type.isUnknown, "TfType 'IsaacSensorIsaacRtxRadarSensorAPI' not found")

    # -- Prim creation --------------------------------------------------------

    def test_define_contact_sensor(self):
        prim = self._stage.DefinePrim("/TestContact", "IsaacContactSensor")
        self.assertTrue(prim.IsValid())
        self.assertEqual(prim.GetTypeName(), "IsaacContactSensor")

    def test_define_imu_sensor(self):
        prim = self._stage.DefinePrim("/TestImu", "IsaacImuSensor")
        self.assertTrue(prim.IsValid())
        self.assertEqual(prim.GetTypeName(), "IsaacImuSensor")

    def test_define_light_beam_sensor(self):
        prim = self._stage.DefinePrim("/TestLightBeam", "IsaacLightBeamSensor")
        self.assertTrue(prim.IsValid())
        self.assertEqual(prim.GetTypeName(), "IsaacLightBeamSensor")

    def test_define_raycast_sensor(self):
        prim = self._stage.DefinePrim("/TestRaycast", "IsaacRaycastSensor")
        self.assertTrue(prim.IsValid())
        self.assertEqual(prim.GetTypeName(), "IsaacRaycastSensor")

    # -- Fallback attribute values -------------------------------------------

    def test_contact_sensor_fallback_threshold(self):
        prim = self._stage.DefinePrim("/TestContact", "IsaacContactSensor")
        val = prim.GetAttribute("threshold").Get()
        self.assertAlmostEqual(val[0], 0.01, places=5)
        self.assertAlmostEqual(val[1], 100000.0, places=1)

    def test_contact_sensor_fallback_radius(self):
        prim = self._stage.DefinePrim("/TestContact", "IsaacContactSensor")
        self.assertAlmostEqual(prim.GetAttribute("radius").Get(), -1.0, places=5)

    def test_contact_sensor_fallback_sensor_period(self):
        prim = self._stage.DefinePrim("/TestContact", "IsaacContactSensor")
        self.assertAlmostEqual(prim.GetAttribute("sensorPeriod").Get(), 0.0025, places=5)

    def test_imu_sensor_fallback_filter_widths(self):
        prim = self._stage.DefinePrim("/TestImu", "IsaacImuSensor")
        self.assertEqual(prim.GetAttribute("linearAccelerationFilterWidth").Get(), 1)
        self.assertEqual(prim.GetAttribute("angularVelocityFilterWidth").Get(), 1)
        self.assertEqual(prim.GetAttribute("orientationFilterWidth").Get(), 1)

    def test_contact_sensor_is_xformable(self):
        prim = self._stage.DefinePrim("/TestContact", "IsaacContactSensor")
        self.assertTrue(prim.IsA(UsdGeom.Xformable))

    # -- IsaacRaycastSensor fallback values -----------------------------------

    def test_raycast_sensor_fallback_min_range(self):
        prim = self._stage.DefinePrim("/TestRaycast", "IsaacRaycastSensor")
        self.assertAlmostEqual(prim.GetAttribute("minRange").Get(), 0.4, places=5)

    def test_raycast_sensor_fallback_max_range(self):
        prim = self._stage.DefinePrim("/TestRaycast", "IsaacRaycastSensor")
        self.assertAlmostEqual(prim.GetAttribute("maxRange").Get(), 100.0, places=5)

    def test_raycast_sensor_fallback_ray_origins(self):
        prim = self._stage.DefinePrim("/TestRaycast", "IsaacRaycastSensor")
        val = prim.GetAttribute("rayOrigins").Get()
        self.assertEqual(len(val), 0)

    def test_raycast_sensor_fallback_ray_directions(self):
        prim = self._stage.DefinePrim("/TestRaycast", "IsaacRaycastSensor")
        val = prim.GetAttribute("rayDirections").Get()
        self.assertEqual(len(val), 0)

    def test_raycast_sensor_fallback_ray_time_offsets(self):
        prim = self._stage.DefinePrim("/TestRaycast", "IsaacRaycastSensor")
        val = prim.GetAttribute("rayTimeOffsets").Get()
        self.assertEqual(len(val), 0)

    def test_raycast_sensor_fallback_output_frame(self):
        prim = self._stage.DefinePrim("/TestRaycast", "IsaacRaycastSensor")
        self.assertEqual(prim.GetAttribute("outputFrameOfReference").Get(), "SENSOR")

    def test_raycast_sensor_fallback_report_hit_prim_paths(self):
        prim = self._stage.DefinePrim("/TestRaycast", "IsaacRaycastSensor")
        self.assertEqual(prim.GetAttribute("reportHitPrimPaths").Get(), False)

    def test_raycast_sensor_inherits_enabled(self):
        prim = self._stage.DefinePrim("/TestRaycast", "IsaacRaycastSensor")
        self.assertEqual(prim.GetAttribute("enabled").Get(), True)

    def test_raycast_sensor_is_xformable(self):
        prim = self._stage.DefinePrim("/TestRaycast", "IsaacRaycastSensor")
        self.assertTrue(prim.IsA(UsdGeom.Xformable))

    # -- API schema application -----------------------------------------------

    def test_apply_rtx_lidar_api(self):
        prim = self._stage.DefinePrim("/TestCam", "Camera")
        prim.AddAppliedSchema("IsaacRtxLidarSensorAPI")
        schemas = prim.GetAppliedSchemas()
        self.assertIn("IsaacRtxLidarSensorAPI", schemas)

    def test_has_api_rtx_lidar(self):
        prim = self._stage.DefinePrim("/TestCam", "Camera")
        prim.AddAppliedSchema("IsaacRtxLidarSensorAPI")
        tf_type = Tf.Type.FindByName("IsaacSensorIsaacRtxLidarSensorAPI")
        self.assertTrue(prim.HasAPI(tf_type))

    def test_apply_rtx_radar_api(self):
        prim = self._stage.DefinePrim("/TestCam2", "Camera")
        prim.AddAppliedSchema("IsaacRtxRadarSensorAPI")
        schemas = prim.GetAppliedSchemas()
        self.assertIn("IsaacRtxRadarSensorAPI", schemas)


class TestRangeSensorSchemaCompatWrapper(omni.kit.test.AsyncTestCase):
    """Verify that the omni.isaac.RangeSensorSchema Python compatibility wrapper works."""

    async def setUp(self):
        await omni.usd.get_context().new_stage_async()
        await omni.kit.app.get_app().next_update_async()
        self._stage = omni.usd.get_context().get_stage()

    def test_import(self):
        with warnings.catch_warnings():
            warnings.simplefilter("ignore", DeprecationWarning)
            import omni.isaac.RangeSensorSchema as RSS

        self.assertTrue(hasattr(RSS, "Lidar"))
        self.assertTrue(hasattr(RSS, "Generic"))
        self.assertTrue(hasattr(RSS, "RangeSensor"))

    def test_lidar_define(self):
        with warnings.catch_warnings():
            warnings.simplefilter("ignore", DeprecationWarning)
            import omni.isaac.RangeSensorSchema as RSS

        lidar = RSS.Lidar.Define(self._stage, "/TestLidar")
        prim = lidar.GetPrim()
        self.assertTrue(prim.IsValid())
        self.assertEqual(prim.GetTypeName(), "Lidar")

    def test_generic_define(self):
        with warnings.catch_warnings():
            warnings.simplefilter("ignore", DeprecationWarning)
            import omni.isaac.RangeSensorSchema as RSS

        generic = RSS.Generic.Define(self._stage, "/TestGeneric")
        prim = generic.GetPrim()
        self.assertTrue(prim.IsValid())
        self.assertEqual(prim.GetTypeName(), "Generic")

    def test_lidar_create_and_get_attrs(self):
        with warnings.catch_warnings():
            warnings.simplefilter("ignore", DeprecationWarning)
            import omni.isaac.RangeSensorSchema as RSS

        lidar = RSS.Lidar.Define(self._stage, "/TestLidar")
        lidar.CreateHorizontalFovAttr(120.0)
        val = lidar.GetHorizontalFovAttr().Get()
        self.assertAlmostEqual(val, 120.0, places=5)

    def test_base_sensor_create_enabled(self):
        with warnings.catch_warnings():
            warnings.simplefilter("ignore", DeprecationWarning)
            import omni.isaac.RangeSensorSchema as RSS

        lidar = RSS.Lidar.Define(self._stage, "/TestLidar")
        lidar.CreateEnabledAttr(False)
        self.assertEqual(lidar.GetEnabledAttr().Get(), False)

    def test_wrap_existing_prim(self):
        with warnings.catch_warnings():
            warnings.simplefilter("ignore", DeprecationWarning)
            import omni.isaac.RangeSensorSchema as RSS

        prim = self._stage.DefinePrim("/TestLidar", "Lidar")
        wrapped = RSS.Lidar(prim)
        self.assertEqual(wrapped.GetPrim().GetPath(), prim.GetPath())

    def test_import_emits_deprecation_warning(self):
        """Importing omni.isaac.RangeSensorSchema must emit a DeprecationWarning."""
        mod_name = "omni.isaac.RangeSensorSchema"
        saved = sys.modules.pop(mod_name, None)
        try:
            with warnings.catch_warnings(record=True) as caught:
                warnings.simplefilter("always")
                importlib.import_module(mod_name)
            deprecation_msgs = [w for w in caught if issubclass(w.category, DeprecationWarning)]
            self.assertTrue(len(deprecation_msgs) > 0, "No DeprecationWarning emitted on import")
        finally:
            if saved is not None:
                sys.modules[mod_name] = saved
            else:
                sys.modules.pop(mod_name, None)


class TestIsaacSensorSchemaCompatWrapper(omni.kit.test.AsyncTestCase):
    """Verify that the omni.isaac.IsaacSensorSchema Python compatibility wrapper works."""

    async def setUp(self):
        await omni.usd.get_context().new_stage_async()
        await omni.kit.app.get_app().next_update_async()
        self._stage = omni.usd.get_context().get_stage()

    def test_import(self):
        import omni.isaac.IsaacSensorSchema as ISS

        self.assertTrue(hasattr(ISS, "IsaacContactSensor"))
        self.assertTrue(hasattr(ISS, "IsaacImuSensor"))
        self.assertTrue(hasattr(ISS, "IsaacLightBeamSensor"))
        self.assertTrue(hasattr(ISS, "IsaacRaycastSensor"))
        self.assertTrue(hasattr(ISS, "IsaacRtxLidarSensorAPI"))
        self.assertTrue(hasattr(ISS, "IsaacRtxRadarSensorAPI"))

    def test_contact_sensor_define(self):
        import omni.isaac.IsaacSensorSchema as ISS

        sensor = ISS.IsaacContactSensor.Define(self._stage, "/TestContact")
        prim = sensor.GetPrim()
        self.assertTrue(prim.IsValid())
        self.assertEqual(prim.GetTypeName(), "IsaacContactSensor")

    def test_imu_sensor_define(self):
        import omni.isaac.IsaacSensorSchema as ISS

        sensor = ISS.IsaacImuSensor.Define(self._stage, "/TestImu")
        prim = sensor.GetPrim()
        self.assertTrue(prim.IsValid())
        self.assertEqual(prim.GetTypeName(), "IsaacImuSensor")

    def test_light_beam_sensor_define(self):
        import omni.isaac.IsaacSensorSchema as ISS

        with warnings.catch_warnings():
            warnings.simplefilter("ignore", DeprecationWarning)
            sensor = ISS.IsaacLightBeamSensor.Define(self._stage, "/TestLightBeam")
        prim = sensor.GetPrim()
        self.assertTrue(prim.IsValid())
        self.assertEqual(prim.GetTypeName(), "IsaacLightBeamSensor")

    def test_contact_sensor_create_and_get_attrs(self):
        import omni.isaac.IsaacSensorSchema as ISS

        sensor = ISS.IsaacContactSensor.Define(self._stage, "/TestContact")
        sensor.CreateRadiusAttr(5.0)
        self.assertAlmostEqual(sensor.GetRadiusAttr().Get(), 5.0, places=5)

    def test_imu_sensor_filter_width_attrs(self):
        import omni.isaac.IsaacSensorSchema as ISS

        sensor = ISS.IsaacImuSensor.Define(self._stage, "/TestImu")
        sensor.CreateLinearAccelerationFilterWidthAttr(10)
        self.assertEqual(sensor.GetLinearAccelerationFilterWidthAttr().Get(), 10)

    def test_rtx_lidar_api_apply(self):
        import omni.isaac.IsaacSensorSchema as ISS

        prim = self._stage.DefinePrim("/TestCam", "Camera")
        ISS.IsaacRtxLidarSensorAPI.Apply(prim)
        self.assertIn("IsaacRtxLidarSensorAPI", prim.GetAppliedSchemas())

    def test_rtx_radar_api_apply(self):
        import omni.isaac.IsaacSensorSchema as ISS

        prim = self._stage.DefinePrim("/TestCam", "Camera")
        ISS.IsaacRtxRadarSensorAPI.Apply(prim)
        self.assertIn("IsaacRtxRadarSensorAPI", prim.GetAppliedSchemas())

    def test_raycast_sensor_define(self):
        import omni.isaac.IsaacSensorSchema as ISS

        sensor = ISS.IsaacRaycastSensor.Define(self._stage, "/TestRaycast")
        prim = sensor.GetPrim()
        self.assertTrue(prim.IsValid())
        self.assertEqual(prim.GetTypeName(), "IsaacRaycastSensor")

    def test_raycast_sensor_create_and_get_attrs(self):
        import omni.isaac.IsaacSensorSchema as ISS

        sensor = ISS.IsaacRaycastSensor.Define(self._stage, "/TestRaycast")
        sensor.CreateMinRangeAttr(0.1)
        self.assertAlmostEqual(sensor.GetMinRangeAttr().Get(), 0.1, places=5)
        sensor.CreateMaxRangeAttr(50.0)
        self.assertAlmostEqual(sensor.GetMaxRangeAttr().Get(), 50.0, places=5)

    def test_raycast_sensor_array_attrs(self):
        import omni.isaac.IsaacSensorSchema as ISS
        from pxr import Vt

        sensor = ISS.IsaacRaycastSensor.Define(self._stage, "/TestRaycast")
        origins = Vt.Vec3fArray([(0, 0, 0), (0, 0.1, 0)])
        sensor.CreateRayOriginsAttr(origins)
        result = sensor.GetRayOriginsAttr().Get()
        self.assertEqual(len(result), 2)

        directions = Vt.Vec3fArray([(1, 0, 0), (1, 0.1, 0)])
        sensor.CreateRayDirectionsAttr(directions)
        result = sensor.GetRayDirectionsAttr().Get()
        self.assertEqual(len(result), 2)

    def test_raycast_sensor_output_frame_attr(self):
        import omni.isaac.IsaacSensorSchema as ISS

        sensor = ISS.IsaacRaycastSensor.Define(self._stage, "/TestRaycast")
        sensor.CreateOutputFrameOfReferenceAttr("WORLD")
        self.assertEqual(sensor.GetOutputFrameOfReferenceAttr().Get(), "WORLD")

    def test_wrap_existing_prim(self):
        import omni.isaac.IsaacSensorSchema as ISS

        prim = self._stage.DefinePrim("/TestContact", "IsaacContactSensor")
        wrapped = ISS.IsaacContactSensor(prim)
        self.assertEqual(wrapped.GetPrim().GetPath(), prim.GetPath())

    def test_light_beam_sensor_construction_emits_deprecation_warning(self):
        """Constructing IsaacLightBeamSensor must emit a DeprecationWarning."""
        import omni.isaac.IsaacSensorSchema as ISS

        prim = self._stage.DefinePrim("/TestLightBeam", "IsaacLightBeamSensor")
        with warnings.catch_warnings(record=True) as caught:
            warnings.simplefilter("always")
            ISS.IsaacLightBeamSensor(prim)
        deprecation_msgs = [w for w in caught if issubclass(w.category, DeprecationWarning)]
        self.assertTrue(len(deprecation_msgs) > 0, "No DeprecationWarning emitted on IsaacLightBeamSensor construction")
