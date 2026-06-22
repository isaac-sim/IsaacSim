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

    async def test_wrap_missing_schema_raises(self) -> None:
        """Reject wrapping a Camera prim that lacks the OmniSensor schema."""
        stage_utils.define_prim("/World/cam", "Camera")
        with self.assertRaises(ValueError):
            RtxCamera("/World/cam")

    async def test_wrap_with_tick_rate(self) -> None:
        """Apply a tick rate override while wrapping an existing RTX camera prim."""
        prim = stage_utils.define_prim("/World/cam", "Camera")
        prim.ApplyAPI("OmniSensorAPI")
        cam = RtxCamera("/World/cam", tick_rate=30.0)
        self.assertAlmostEqual(cam.prims[0].GetAttribute("omni:sensor:tickRate").Get(), 30.0)

    # -- create --

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
