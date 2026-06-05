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

"""Tests for the RtxCamera authoring class."""

import isaacsim.core.experimental.utils.stage as stage_utils
import omni.kit.test
from isaacsim.sensors.experimental.rtx import RtxCamera


class TestRtxCamera(omni.kit.test.AsyncTestCase):
    async def setUp(self):
        super().setUp()
        await stage_utils.create_new_stage_async()
        stage_utils.define_prim("/World", "Xform")

    async def tearDown(self):
        super().tearDown()
        stage_utils.close_stage()

    # -- wrap --

    async def test_wrap_existing_prim(self):
        prim = stage_utils.define_prim("/World/cam", "Camera")
        prim.ApplyAPI("OmniSensorAPI")
        cam = RtxCamera("/World/cam")
        self.assertEqual(cam.paths[0], "/World/cam")
        self.assertEqual(cam.prims[0].GetTypeName(), "Camera")

    async def test_wrap_wrong_type_raises(self):
        stage_utils.define_prim("/World/xform", "Xform")
        with self.assertRaises(ValueError):
            RtxCamera("/World/xform")

    async def test_wrap_missing_schema_raises(self):
        stage_utils.define_prim("/World/cam", "Camera")
        with self.assertRaises(ValueError):
            RtxCamera("/World/cam")

    async def test_wrap_with_tick_rate(self):
        prim = stage_utils.define_prim("/World/cam", "Camera")
        prim.ApplyAPI("OmniSensorAPI")
        cam = RtxCamera("/World/cam", tick_rate=30.0)
        self.assertAlmostEqual(cam.prims[0].GetAttribute("omni:sensor:tickRate").Get(), 30.0)

    # -- create --

    async def test_create_new_prim(self):
        cam = RtxCamera("/World/cam")
        self.assertEqual(cam.prims[0].GetTypeName(), "Camera")
        self.assertTrue(cam.prims[0].HasAPI("OmniSensorAPI"))

    async def test_create_with_tick_rate(self):
        cam = RtxCamera("/World/cam", tick_rate=60.0)
        self.assertAlmostEqual(cam.prims[0].GetAttribute("omni:sensor:tickRate").Get(), 60.0)

    # -- camera property --

    async def test_camera_property(self):
        cam = RtxCamera("/World/cam")
        camera = cam.camera
        self.assertIsNotNone(camera)
        # Verify Camera methods are accessible
        camera.set_focal_lengths(24.0)
        fl = camera.get_focal_lengths()
        self.assertAlmostEqual(float(fl.numpy()[0]), 24.0)

    # -- schemas --

    async def test_create_with_schema(self):
        cam = RtxCamera(
            "/World/cam",
            schemas=["OmniLensDistortionOpenCvFisheyeAPI"],
        )
        self.assertTrue(cam.prims[0].HasAPI("OmniLensDistortionOpenCvFisheyeAPI"))

    async def test_create_with_schema_and_attributes(self):
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

    async def test_wrap_with_schema(self):
        prim = stage_utils.define_prim("/World/cam", "Camera")
        prim.ApplyAPI("OmniSensorAPI")
        cam = RtxCamera(
            "/World/cam",
            schemas=["OmniLensDistortionOpenCvFisheyeAPI"],
            attributes={"omni:lensdistortion:opencvFisheye:k1": 0.25},
        )
        self.assertTrue(cam.prims[0].HasAPI("OmniLensDistortionOpenCvFisheyeAPI"))
        self.assertAlmostEqual(cam.prims[0].GetAttribute("omni:lensdistortion:opencvFisheye:k1").Get(), 0.25)

    async def test_create_with_multiple_schemas(self):
        cam = RtxCamera(
            "/World/cam",
            schemas=["OmniLensDistortionOpenCvFisheyeAPI", "OmniLensDistortionOpenCvPinholeAPI"],
        )
        self.assertTrue(cam.prims[0].HasAPI("OmniLensDistortionOpenCvFisheyeAPI"))
        self.assertTrue(cam.prims[0].HasAPI("OmniLensDistortionOpenCvPinholeAPI"))

    async def test_wrap_schema_already_applied_is_idempotent(self):
        prim = stage_utils.define_prim("/World/cam", "Camera")
        prim.ApplyAPI("OmniSensorAPI")
        prim.ApplyAPI("OmniLensDistortionOpenCvFisheyeAPI")
        cam = RtxCamera(
            "/World/cam",
            schemas=["OmniLensDistortionOpenCvFisheyeAPI"],
        )
        self.assertTrue(cam.prims[0].HasAPI("OmniLensDistortionOpenCvFisheyeAPI"))
