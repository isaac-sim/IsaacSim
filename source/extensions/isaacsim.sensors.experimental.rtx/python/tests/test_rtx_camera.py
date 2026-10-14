# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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

"""Verify RtxCamera authoring for camera prim wrapping, creation, tick rates, schemas, and camera access."""

from unittest import mock

import isaacsim.core.experimental.utils.stage as stage_utils
import omni.kit.test
from isaacsim.sensors.experimental.rtx import RtxCamera


class TestRtxCamera(omni.kit.test.AsyncTestCase):
    """Tests the RTX camera authoring wrapper against USD Camera prims."""

    async def setUp(self) -> None:
        """Create an empty stage with a /World root for RTX camera authoring tests."""
        super().setUp()
        await stage_utils.create_new_stage_async()
        stage_utils.define_prim("/World", "Xform")

    async def tearDown(self) -> None:
        """Close the stage created for each RTX camera authoring test."""
        super().tearDown()
        stage_utils.close_stage()

    # -- wrap --

    async def test_wrap_existing_prim(self) -> None:
        """Wrap an existing Camera prim that already has the OmniSensor schema."""
        prim = stage_utils.define_prim("/World/cam", "Camera")
        prim.ApplyAPI("OmniSensorAPI")
        cam = RtxCamera("/World/cam")
        self.assertEqual(cam.paths[0], "/World/cam")
        self.assertEqual(cam.prims[0].GetTypeName(), "Camera")

    async def test_wrap_wrong_type_raises(self) -> None:
        """Reject wrapping a non-camera Xform prim as an RtxCamera."""
        stage_utils.define_prim("/World/xform", "Xform")
        with self.assertRaises(ValueError):
            RtxCamera("/World/xform")

    async def test_wrap_missing_schema_applies_schema(self) -> None:
        """Apply the OmniSensor schema when wrapping a Camera prim that lacks it."""
        prim = stage_utils.define_prim("/World/cam", "Camera")
        self.assertFalse(prim.HasAPI("OmniSensorAPI"))
        cam = RtxCamera("/World/cam")
        self.assertEqual(cam.paths[0], "/World/cam")
        self.assertTrue(cam.prims[0].HasAPI("OmniSensorAPI"))
        self.assertTrue(cam.prims[0].HasAttribute("omni:sensor:tickRate"))

    async def test_wrap_missing_schema_with_tick_rate(self) -> None:
        """Author a tick rate while wrapping a Camera prim that lacks the OmniSensor schema."""
        stage_utils.define_prim("/World/cam", "Camera")
        cam = RtxCamera("/World/cam", tick_rate=30.0)
        self.assertAlmostEqual(cam.prims[0].GetAttribute("omni:sensor:tickRate").Get(), 30.0)

    async def test_wrap_with_tick_rate(self) -> None:
        """Apply a tick rate override while wrapping an existing RTX camera prim."""
        prim = stage_utils.define_prim("/World/cam", "Camera")
        prim.ApplyAPI("OmniSensorAPI")
        cam = RtxCamera("/World/cam", tick_rate=30.0)
        self.assertAlmostEqual(cam.prims[0].GetAttribute("omni:sensor:tickRate").Get(), 30.0)

    async def test_overwriting_an_authored_tick_rate_warns(self) -> None:
        """Warn when an override replaces a tick rate that was deliberately authored on the prim.

        The rate is stored on the prim, so a second wrapper changes the shared authored state.
        """
        RtxCamera("/World/cam", tick_rate=30.0)
        with mock.patch("carb.log_warn") as log_warn:
            cam = RtxCamera("/World/cam", tick_rate=5.0)
        self.assertAlmostEqual(cam.prims[0].GetAttribute("omni:sensor:tickRate").Get(), 5.0)
        self.assertTrue(
            any("overwriting the authored" in str(call) for call in log_warn.call_args_list),
            f"expected an overwrite warning, got: {log_warn.call_args_list}",
        )

    async def test_overwriting_an_authored_tick_rate_through_attributes_warns(self) -> None:
        """Warn when the attributes mapping replaces a deliberately authored tick rate."""
        RtxCamera("/World/cam", tick_rate=30.0)
        with mock.patch("carb.log_warn") as log_warn:
            cam = RtxCamera("/World/cam", attributes={"omni:sensor:tickRate": 5.0})
        self.assertAlmostEqual(cam.prims[0].GetAttribute("omni:sensor:tickRate").Get(), 5.0)
        self.assertTrue(
            any("overwriting the authored" in str(call) for call in log_warn.call_args_list),
            f"expected an overwrite warning, got: {log_warn.call_args_list}",
        )

    async def test_authoring_a_tick_rate_does_not_warn(self) -> None:
        """Stay silent when no deliberate tick rate is being replaced.

        The ``OmniSensorAPI`` schema always supplies the attribute with a default of ``0``, so the
        warning has to key off an authored opinion rather than the attribute merely existing.
        """
        prim = stage_utils.define_prim("/World/schema_default", "Camera")
        prim.ApplyAPI("OmniSensorAPI")
        with mock.patch("carb.log_warn") as log_warn:
            RtxCamera("/World/created", tick_rate=30.0)  # newly created prim
            RtxCamera("/World/schema_default", tick_rate=30.0)  # schema default, not authored
            RtxCamera("/World/created", tick_rate=30.0)  # authored, but the same value
        self.assertFalse(
            any("overwriting the authored" in str(call) for call in log_warn.call_args_list),
            f"unexpected overwrite warning: {log_warn.call_args_list}",
        )

    # -- create --

    async def test_wrap_regex_multiple_prims(self) -> None:
        """Wrap several existing Camera prims with one RtxCamera instance."""
        for i in range(3):
            stage_utils.define_prim(f"/World/cam_{i}", "Camera")
        cam = RtxCamera("/World/cam_.*")
        self.assertEqual(len(cam), 3)
        self.assertEqual(list(cam.paths), [f"/World/cam_{i}" for i in range(3)])
        for prim in cam.prims:
            self.assertTrue(prim.HasAPI("OmniSensorAPI"))
        self.assertEqual(len(cam.camera), 3)
        cam.camera.set_focal_lengths(24.0)
        fl = cam.camera.get_focal_lengths()
        self.assertEqual(fl.shape[0], 3)
        self.assertAlmostEqual(float(fl.numpy()[0]), 24.0)

    async def test_wrap_multiple_prims_validates_types_before_applying_schemas(self) -> None:
        """Reject a mixed-type batch without modifying the valid camera prims."""
        camera_prim = stage_utils.define_prim("/World/prim_0", "Camera")
        stage_utils.define_prim("/World/prim_1", "Xform")
        with self.assertRaisesRegex(ValueError, "is not an 'Camera' prim"):
            RtxCamera("/World/prim_.*")
        self.assertFalse(camera_prim.HasAPI("OmniSensorAPI"))

    async def test_create_list_of_prims(self) -> None:
        """Create several Camera prims from an explicit path list."""
        paths = [f"/World/cam_{i}" for i in range(2)]
        cam = RtxCamera(paths, tick_rate=20.0)
        self.assertEqual(list(cam.paths), paths)
        for prim in cam.prims:
            self.assertEqual(prim.GetTypeName(), "Camera")
            self.assertTrue(prim.HasAPI("OmniSensorAPI"))
            self.assertAlmostEqual(prim.GetAttribute("omni:sensor:tickRate").Get(), 20.0)

    async def test_create_new_prim(self) -> None:
        """Create a new Camera prim with the required OmniSensor schema."""
        cam = RtxCamera("/World/cam")
        self.assertEqual(cam.prims[0].GetTypeName(), "Camera")
        self.assertTrue(cam.prims[0].HasAPI("OmniSensorAPI"))

    async def test_create_with_tick_rate(self) -> None:
        """Author the tick rate attribute when creating an RTX camera prim."""
        cam = RtxCamera("/World/cam", tick_rate=60.0)
        self.assertAlmostEqual(cam.prims[0].GetAttribute("omni:sensor:tickRate").Get(), 60.0)

    # -- camera property --

    async def test_camera_property(self) -> None:
        """Expose the underlying camera wrapper and allow focal-length access."""
        cam = RtxCamera("/World/cam")
        camera = cam.camera
        self.assertIsNotNone(camera)
        # Verify Camera methods are accessible
        camera.set_focal_lengths(24.0)
        fl = camera.get_focal_lengths()
        self.assertAlmostEqual(float(fl.numpy()[0]), 24.0)

    # -- schemas --

    async def test_create_with_schema(self) -> None:
        """Apply a lens-distortion schema while creating an RTX camera prim."""
        cam = RtxCamera(
            "/World/cam",
            schemas=["OmniLensDistortionOpenCvFisheyeAPI"],
        )
        self.assertTrue(cam.prims[0].HasAPI("OmniLensDistortionOpenCvFisheyeAPI"))

    async def test_create_with_schema_and_attributes(self) -> None:
        """Apply lens-distortion schema attributes while creating an RTX camera prim."""
        cam = RtxCamera(
            "/World/cam",
            schemas=["OmniLensDistortionOpenCvFisheyeAPI"],
            attributes={
                "omni:lensdistortion:opencvFisheye:k1": 0.5,
                "omni:lensdistortion:opencvFisheye:k2": -0.1,
            },
        )
        prim = cam.prims[0]
        self.assertTrue(prim.HasAPI("OmniLensDistortionOpenCvFisheyeAPI"))
        self.assertAlmostEqual(prim.GetAttribute("omni:lensdistortion:opencvFisheye:k1").Get(), 0.5)
        self.assertAlmostEqual(prim.GetAttribute("omni:lensdistortion:opencvFisheye:k2").Get(), -0.1)

    async def test_wrap_with_schema(self) -> None:
        """Apply lens-distortion schema attributes while wrapping an existing camera prim."""
        prim = stage_utils.define_prim("/World/cam", "Camera")
        prim.ApplyAPI("OmniSensorAPI")
        cam = RtxCamera(
            "/World/cam",
            schemas=["OmniLensDistortionOpenCvFisheyeAPI"],
            attributes={"omni:lensdistortion:opencvFisheye:k1": 0.25},
        )
        self.assertTrue(cam.prims[0].HasAPI("OmniLensDistortionOpenCvFisheyeAPI"))
        self.assertAlmostEqual(cam.prims[0].GetAttribute("omni:lensdistortion:opencvFisheye:k1").Get(), 0.25)

    async def test_create_with_multiple_schemas(self) -> None:
        """Apply multiple lens-distortion schemas while creating an RTX camera prim."""
        cam = RtxCamera(
            "/World/cam",
            schemas=["OmniLensDistortionOpenCvFisheyeAPI", "OmniLensDistortionOpenCvPinholeAPI"],
        )
        self.assertTrue(cam.prims[0].HasAPI("OmniLensDistortionOpenCvFisheyeAPI"))
        self.assertTrue(cam.prims[0].HasAPI("OmniLensDistortionOpenCvPinholeAPI"))

    async def test_wrap_schema_already_applied_is_idempotent(self) -> None:
        """Leave an already-applied lens-distortion schema valid when wrapping."""
        prim = stage_utils.define_prim("/World/cam", "Camera")
        prim.ApplyAPI("OmniSensorAPI")
        prim.ApplyAPI("OmniLensDistortionOpenCvFisheyeAPI")
        cam = RtxCamera(
            "/World/cam",
            schemas=["OmniLensDistortionOpenCvFisheyeAPI"],
        )
        self.assertTrue(cam.prims[0].HasAPI("OmniLensDistortionOpenCvFisheyeAPI"))
