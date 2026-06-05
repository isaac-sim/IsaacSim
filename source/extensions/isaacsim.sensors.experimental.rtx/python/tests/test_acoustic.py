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

"""Tests for the Acoustic authoring class."""

import isaacsim.core.experimental.utils.stage as stage_utils
import omni.kit.test
from isaacsim.sensors.experimental.rtx import Acoustic


class TestAcoustic(omni.kit.test.AsyncTestCase):
    async def setUp(self):
        super().setUp()
        await stage_utils.create_new_stage_async()
        stage_utils.define_prim("/World", "Xform")

    async def tearDown(self):
        super().tearDown()
        stage_utils.close_stage()

    # -- wrap --

    async def test_wrap_existing_prim(self):
        prim = stage_utils.define_prim("/World/acoustic", "OmniAcoustic")
        prim.ApplyAPI("OmniSensorGenericAcousticWpmAPI")
        acoustic = Acoustic("/World/acoustic")
        self.assertEqual(acoustic.paths[0], "/World/acoustic")
        self.assertEqual(acoustic.prims[0].GetTypeName(), "OmniAcoustic")

    async def test_wrap_wrong_type_raises(self):
        stage_utils.define_prim("/World/xform", "Xform")
        with self.assertRaises(ValueError):
            Acoustic("/World/xform")

    async def test_wrap_missing_schema_raises(self):
        stage_utils.define_prim("/World/acoustic", "OmniAcoustic")
        with self.assertRaises(ValueError):
            Acoustic("/World/acoustic")

    async def test_wrap_with_tick_rate(self):
        prim = stage_utils.define_prim("/World/acoustic", "OmniAcoustic")
        prim.ApplyAPI("OmniSensorGenericAcousticWpmAPI")
        acoustic = Acoustic("/World/acoustic", tick_rate=30.0)
        self.assertAlmostEqual(acoustic.prims[0].GetAttribute("omni:sensor:tickRate").Get(), 30.0)

    async def test_wrap_with_attributes(self):
        prim = stage_utils.define_prim("/World/acoustic", "OmniAcoustic")
        prim.ApplyAPI("OmniSensorGenericAcousticWpmAPI")
        acoustic = Acoustic("/World/acoustic", attributes={"omni:sensor:WpmAcoustic:centerFrequency": 40000.0})
        self.assertAlmostEqual(acoustic.prims[0].GetAttribute("omni:sensor:WpmAcoustic:centerFrequency").Get(), 40000.0)

    async def test_wrap_with_nonexistent_attribute_does_not_raise(self):
        prim = stage_utils.define_prim("/World/acoustic", "OmniAcoustic")
        prim.ApplyAPI("OmniSensorGenericAcousticWpmAPI")
        acoustic = Acoustic("/World/acoustic", attributes={"nonexistent:attr": 42})
        self.assertEqual(acoustic.prims[0].GetTypeName(), "OmniAcoustic")

    # -- create --

    async def test_create_new_prim(self):
        acoustic = Acoustic("/World/acoustic")
        self.assertEqual(acoustic.prims[0].GetTypeName(), "OmniAcoustic")
        self.assertTrue(acoustic.prims[0].HasAPI("OmniSensorGenericAcousticWpmAPI"))

    async def test_create_with_tick_rate(self):
        acoustic = Acoustic("/World/acoustic", tick_rate=50.0)
        self.assertAlmostEqual(acoustic.prims[0].GetAttribute("omni:sensor:tickRate").Get(), 50.0)

    async def test_create_with_attributes(self):
        acoustic = Acoustic(
            "/World/acoustic",
            attributes={"omni:sensor:WpmAcoustic:centerFrequency": 40000.0},
        )
        self.assertAlmostEqual(acoustic.prims[0].GetAttribute("omni:sensor:WpmAcoustic:centerFrequency").Get(), 40000.0)

    async def test_create_with_nonexistent_attribute_does_not_raise(self):
        acoustic = Acoustic("/World/acoustic", attributes={"nonexistent:attr": 42})
        self.assertEqual(acoustic.prims[0].GetTypeName(), "OmniAcoustic")

    async def test_create_with_tick_rate_and_attributes(self):
        acoustic = Acoustic(
            "/World/acoustic",
            tick_rate=30.0,
            attributes={"omni:sensor:WpmAcoustic:centerFrequency": 40000.0},
        )
        self.assertAlmostEqual(acoustic.prims[0].GetAttribute("omni:sensor:tickRate").Get(), 30.0)
        self.assertAlmostEqual(acoustic.prims[0].GetAttribute("omni:sensor:WpmAcoustic:centerFrequency").Get(), 40000.0)

    # -- multi-apply schemas --

    async def test_create_with_sensor_mount_attributes(self):
        acoustic = Acoustic(
            "/World/acoustic",
            attributes={
                "omni:sensor:WpmAcoustic:sensorMount:m001:position": (0.1, 0.2, 0.3),
                "omni:sensor:WpmAcoustic:sensorMount:m001:rotation": (0.0, 0.0, 0.0),
            },
        )
        prim = acoustic.prims[0]
        self.assertTrue(prim.HasAPI("OmniSensorWpmAcousticSensorMountAPI", instanceName="m001"))

    async def test_create_with_rx_group_attributes(self):
        acoustic = Acoustic(
            "/World/acoustic",
            attributes={"omni:sensor:WpmAcoustic:rxGroup:g001:receiverIndices": [0]},
        )
        prim = acoustic.prims[0]
        self.assertTrue(prim.HasAPI("OmniSensorWpmAcousticRxGroupAPI", instanceName="g001"))

    async def test_create_with_multiple_mount_instances(self):
        acoustic = Acoustic(
            "/World/acoustic",
            attributes={
                "omni:sensor:WpmAcoustic:sensorMount:m001:position": (0.0, 0.0, 0.0),
                "omni:sensor:WpmAcoustic:sensorMount:m002:position": (1.0, 0.0, 0.0),
            },
        )
        prim = acoustic.prims[0]
        self.assertTrue(prim.HasAPI("OmniSensorWpmAcousticSensorMountAPI", instanceName="m001"))
        self.assertTrue(prim.HasAPI("OmniSensorWpmAcousticSensorMountAPI", instanceName="m002"))

    # -- errors --

    async def test_multiple_prims_raises(self):
        prim = stage_utils.define_prim("/World/acoustic_0", "OmniAcoustic")
        prim.ApplyAPI("OmniSensorGenericAcousticWpmAPI")
        prim = stage_utils.define_prim("/World/acoustic_1", "OmniAcoustic")
        prim.ApplyAPI("OmniSensorGenericAcousticWpmAPI")
        with self.assertRaises(ValueError):
            Acoustic("/World/acoustic_.*")

    # -- create from config --

    async def test_create_with_invalid_config_raises(self):
        with self.assertRaises(ValueError):
            Acoustic.create("/World/acoustic", config="NotARealAcousticConfig")

    async def test_create_with_both_config_and_usd_path_raises(self):
        with self.assertRaises(ValueError):
            Acoustic.create("/World/acoustic", config="some_config", usd_path="/some/path.usd")
