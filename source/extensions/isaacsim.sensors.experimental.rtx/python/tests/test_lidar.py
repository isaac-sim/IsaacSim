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

"""Tests for the Lidar authoring class."""

import isaacsim.core.experimental.utils.prim as prim_utils
import isaacsim.core.experimental.utils.stage as stage_utils
import omni.kit.test
from isaacsim.sensors.experimental.rtx import Lidar
from isaacsim.storage.native import get_assets_root_path


class TestLidar(omni.kit.test.AsyncTestCase):
    async def setUp(self):
        super().setUp()
        await stage_utils.create_new_stage_async()
        stage_utils.define_prim("/World", "Xform")

    async def tearDown(self):
        super().tearDown()
        stage_utils.close_stage()

    # -- wrap --

    async def test_wrap_existing_prim(self):
        prim = stage_utils.define_prim("/World/lidar", "OmniLidar")
        prim.ApplyAPI("OmniSensorGenericLidarCoreAPI")
        lidar = Lidar("/World/lidar")
        self.assertEqual(lidar.paths[0], "/World/lidar")
        self.assertEqual(lidar.prims[0].GetTypeName(), "OmniLidar")

    async def test_wrap_wrong_type_raises(self):
        stage_utils.define_prim("/World/xform", "Xform")
        with self.assertRaises(ValueError):
            Lidar("/World/xform")

    async def test_wrap_missing_schema_raises(self):
        stage_utils.define_prim("/World/lidar", "OmniLidar")
        with self.assertRaises(ValueError):
            Lidar("/World/lidar")

    async def test_wrap_with_tick_rate(self):
        prim = stage_utils.define_prim("/World/lidar", "OmniLidar")
        prim.ApplyAPI("OmniSensorGenericLidarCoreAPI")
        lidar = Lidar("/World/lidar", tick_rate=10.0)
        self.assertAlmostEqual(lidar.prims[0].GetAttribute("omni:sensor:tickRate").Get(), 10.0)

    async def test_wrap_with_accumulate_outputs_false(self):
        prim = stage_utils.define_prim("/World/lidar", "OmniLidar")
        prim.ApplyAPI("OmniSensorGenericLidarCoreAPI")
        lidar = Lidar("/World/lidar", accumulate_outputs=False)
        self.assertFalse(lidar.prims[0].GetAttribute("omni:sensor:Core:accumulateOutputs").Get())

    async def test_wrap_with_attributes(self):
        prim = stage_utils.define_prim("/World/lidar", "OmniLidar")
        prim.ApplyAPI("OmniSensorGenericLidarCoreAPI")
        lidar = Lidar("/World/lidar", attributes={"omni:sensor:Core:outputFrameOfReference": "WORLD"})
        self.assertEqual(lidar.prims[0].GetAttribute("omni:sensor:Core:outputFrameOfReference").Get(), "WORLD")

    async def test_wrap_with_nonexistent_attribute_does_not_raise(self):
        prim = stage_utils.define_prim("/World/lidar", "OmniLidar")
        prim.ApplyAPI("OmniSensorGenericLidarCoreAPI")
        lidar = Lidar("/World/lidar", attributes={"nonexistent:attr": 42})
        self.assertEqual(lidar.prims[0].GetTypeName(), "OmniLidar")

    # -- create --

    async def test_create_new_prim(self):
        lidar = Lidar("/World/lidar")
        self.assertEqual(lidar.prims[0].GetTypeName(), "OmniLidar")
        self.assertTrue(lidar.prims[0].HasAPI("OmniSensorGenericLidarCoreAPI"))

    async def test_create_with_tick_rate(self):
        lidar = Lidar("/World/lidar", tick_rate=30.0)
        self.assertAlmostEqual(lidar.prims[0].GetAttribute("omni:sensor:tickRate").Get(), 30.0)

    async def test_create_default_tick_rate(self):
        lidar = Lidar("/World/lidar")
        self.assertAlmostEqual(lidar.prims[0].GetAttribute("omni:sensor:tickRate").Get(), 10.0)

    async def test_create_default_accumulate_outputs_is_true(self):
        lidar = Lidar("/World/lidar")
        self.assertTrue(lidar.prims[0].GetAttribute("omni:sensor:Core:accumulateOutputs").Get())

    async def test_create_with_accumulate_outputs_false(self):
        lidar = Lidar("/World/lidar", accumulate_outputs=False)
        self.assertFalse(lidar.prims[0].GetAttribute("omni:sensor:Core:accumulateOutputs").Get())

    async def test_create_with_attributes(self):
        lidar = Lidar("/World/lidar", attributes={"omni:sensor:Core:outputFrameOfReference": "WORLD"})
        self.assertEqual(lidar.prims[0].GetAttribute("omni:sensor:Core:outputFrameOfReference").Get(), "WORLD")

    async def test_create_with_tick_rate_and_attributes(self):
        lidar = Lidar(
            "/World/lidar",
            tick_rate=25.0,
            attributes={"omni:sensor:Core:outputFrameOfReference": "WORLD"},
        )
        self.assertAlmostEqual(lidar.prims[0].GetAttribute("omni:sensor:tickRate").Get(), 25.0)
        self.assertEqual(lidar.prims[0].GetAttribute("omni:sensor:Core:outputFrameOfReference").Get(), "WORLD")

    async def test_tick_rate_from_attributes_overrides_parameter(self):
        prim = stage_utils.define_prim("/World/lidar", "OmniLidar")
        prim.ApplyAPI("OmniSensorGenericLidarCoreAPI")
        lidar = Lidar("/World/lidar", tick_rate=10.0, attributes={"omni:sensor:tickRate": 25.0})
        self.assertAlmostEqual(lidar.prims[0].GetAttribute("omni:sensor:tickRate").Get(), 25.0)

    async def test_accumulate_outputs_from_attributes_overrides_parameter(self):
        prim = stage_utils.define_prim("/World/lidar", "OmniLidar")
        prim.ApplyAPI("OmniSensorGenericLidarCoreAPI")
        lidar = Lidar("/World/lidar", accumulate_outputs=True, attributes={"omni:sensor:Core:accumulateOutputs": False})
        self.assertFalse(lidar.prims[0].GetAttribute("omni:sensor:Core:accumulateOutputs").Get())

    async def test_create_existing_config_maintains_attributes(self):
        stage_utils.add_reference_to_stage(
            usd_path=get_assets_root_path() + "/Isaac/Sensors/NVIDIA/Example_Rotary.usda", path="/World/reference"
        )
        reference_prim = prim_utils.get_prim_at_path("/World/reference")
        lidar = Lidar.create(path="/World/lidar", config="Example_Rotary")
        lidar_prim = prim_utils.get_prim_at_path("/World/lidar")

        def get_attributes_as_dict(prim):
            attr_dict = {}
            for attr in prim.GetAttributes():
                # Get attribute name and its value at the default time
                attr_dict[attr.GetName()] = attr.Get()
            return attr_dict

        self.maxDiff = None
        reference_prim_attributes_dict = get_attributes_as_dict(reference_prim)
        lidar_prim_attributes_dict = get_attributes_as_dict(lidar_prim)

        keys_to_remove = ["_replicator:rendervar:GenericModelOutput:channels"]
        for key in lidar_prim_attributes_dict:
            if key.startswith("xformOp"):
                keys_to_remove.append(key)
        for key in keys_to_remove:
            lidar_prim_attributes_dict.pop(key, None)
        keys_to_remove = []
        for key in reference_prim_attributes_dict:
            if key.startswith("xformOp"):
                keys_to_remove.append(key)
        for key in keys_to_remove:
            reference_prim_attributes_dict.pop(key, None)

        self.assertDictEqual(reference_prim_attributes_dict, lidar_prim_attributes_dict)

    # -- schemas --

    async def test_create_with_multi_instance_schema(self):
        lidar = Lidar("/World/lidar", schemas=["OmniSensorGenericLidarCoreEmitterStateAPI:s002"])
        self.assertTrue(lidar.prims[0].HasAPI("OmniSensorGenericLidarCoreEmitterStateAPI", "s002"))

    async def test_wrap_with_multi_instance_schema(self):
        prim = stage_utils.define_prim("/World/lidar", "OmniLidar")
        prim.ApplyAPI("OmniSensorGenericLidarCoreAPI")
        lidar = Lidar("/World/lidar", schemas=["OmniSensorGenericLidarCoreEmitterStateAPI:s002"])
        self.assertTrue(lidar.prims[0].HasAPI("OmniSensorGenericLidarCoreEmitterStateAPI", "s002"))

    # -- errors --

    async def test_multiple_prims_raises(self):
        prim = stage_utils.define_prim("/World/lidar_0", "OmniLidar")
        prim.ApplyAPI("OmniSensorGenericLidarCoreAPI")
        prim = stage_utils.define_prim("/World/lidar_1", "OmniLidar")
        prim.ApplyAPI("OmniSensorGenericLidarCoreAPI")
        with self.assertRaises(ValueError):
            Lidar("/World/lidar_.*")
