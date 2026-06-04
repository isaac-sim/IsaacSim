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

"""Tests for the Radar authoring class."""

import carb
import isaacsim.core.experimental.utils.stage as stage_utils
import omni.kit.test
from isaacsim.sensors.experimental.rtx import Radar


class TestRadar(omni.kit.test.AsyncTestCase):
    async def setUp(self):
        super().setUp()
        self._settings = carb.settings.get_settings()
        self._original_motion_bvh = self._settings.get("/renderer/raytracingMotion/enabled")
        self._settings.set("/renderer/raytracingMotion/enabled", True)
        await stage_utils.create_new_stage_async()
        stage_utils.define_prim("/World", "Xform")

    async def tearDown(self):
        super().tearDown()
        stage_utils.close_stage()
        self._settings.set("/renderer/raytracingMotion/enabled", self._original_motion_bvh)

    # -- wrap --

    async def test_wrap_existing_prim(self):
        prim = stage_utils.define_prim("/World/radar", "OmniRadar")
        prim.ApplyAPI("OmniSensorGenericRadarWpmDmatAPI")
        radar = Radar("/World/radar")
        self.assertEqual(radar.paths[0], "/World/radar")
        self.assertEqual(radar.prims[0].GetTypeName(), "OmniRadar")

    async def test_wrap_wrong_type_raises(self):
        stage_utils.define_prim("/World/xform", "Xform")
        with self.assertRaises(ValueError):
            Radar("/World/xform")

    async def test_wrap_missing_schema_raises(self):
        stage_utils.define_prim("/World/radar", "OmniRadar")
        with self.assertRaises(ValueError):
            Radar("/World/radar")

    async def test_wrap_with_tick_rate(self):
        prim = stage_utils.define_prim("/World/radar", "OmniRadar")
        prim.ApplyAPI("OmniSensorGenericRadarWpmDmatAPI")
        radar = Radar("/World/radar", tick_rate=20.0)
        self.assertAlmostEqual(radar.prims[0].GetAttribute("omni:sensor:tickRate").Get(), 20.0)

    async def test_wrap_with_attributes(self):
        prim = stage_utils.define_prim("/World/radar", "OmniRadar")
        prim.ApplyAPI("OmniSensorGenericRadarWpmDmatAPI")
        radar = Radar("/World/radar", attributes={"omni:sensor:WpmDmat:cfarMode": "4D"})
        self.assertEqual(radar.prims[0].GetAttribute("omni:sensor:WpmDmat:cfarMode").Get(), "4D")

    async def test_wrap_with_nonexistent_attribute_does_not_raise(self):
        prim = stage_utils.define_prim("/World/radar", "OmniRadar")
        prim.ApplyAPI("OmniSensorGenericRadarWpmDmatAPI")
        radar = Radar("/World/radar", attributes={"nonexistent:attr": 42})
        self.assertEqual(radar.prims[0].GetTypeName(), "OmniRadar")

    # -- create --

    async def test_create_new_prim(self):
        radar = Radar("/World/radar")
        self.assertEqual(radar.prims[0].GetTypeName(), "OmniRadar")
        self.assertTrue(radar.prims[0].HasAPI("OmniSensorGenericRadarWpmDmatAPI"))

    async def test_create_with_tick_rate(self):
        radar = Radar("/World/radar", tick_rate=15.0)
        self.assertAlmostEqual(radar.prims[0].GetAttribute("omni:sensor:tickRate").Get(), 15.0)

    async def test_create_with_attributes(self):
        radar = Radar("/World/radar", attributes={"omni:sensor:WpmDmat:cfarMode": "4D"})
        self.assertEqual(radar.prims[0].GetAttribute("omni:sensor:WpmDmat:cfarMode").Get(), "4D")

    async def test_create_with_tick_rate_and_attributes(self):
        radar = Radar(
            "/World/radar",
            tick_rate=20.0,
            attributes={"omni:sensor:WpmDmat:cfarMode": "4D"},
        )
        self.assertAlmostEqual(radar.prims[0].GetAttribute("omni:sensor:tickRate").Get(), 20.0)
        self.assertEqual(radar.prims[0].GetAttribute("omni:sensor:WpmDmat:cfarMode").Get(), "4D")

    # -- errors --

    async def test_create_without_motion_bvh_raises(self):
        self._settings.set("/renderer/raytracingMotion/enabled", False)
        try:
            with self.assertRaises(RuntimeError):
                Radar("/World/radar")
        finally:
            self._settings.set("/renderer/raytracingMotion/enabled", True)

    async def test_multiple_prims_raises(self):
        prim = stage_utils.define_prim("/World/radar_0", "OmniRadar")
        prim.ApplyAPI("OmniSensorGenericRadarWpmDmatAPI")
        prim = stage_utils.define_prim("/World/radar_1", "OmniRadar")
        prim.ApplyAPI("OmniSensorGenericRadarWpmDmatAPI")
        with self.assertRaises(ValueError):
            Radar("/World/radar_.*")
