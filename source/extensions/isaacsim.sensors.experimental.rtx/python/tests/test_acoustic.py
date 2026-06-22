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

"""Verify Acoustic authoring for RTX acoustic prim wrapping, creation, schemas, attributes, and config errors."""

import isaacsim.core.experimental.utils.stage as stage_utils
import omni.kit.test
from isaacsim.sensors.experimental.rtx import Acoustic


class TestAcoustic(omni.kit.test.AsyncTestCase):
    """Tests the RTX acoustic authoring wrapper against OmniAcoustic prims."""

    async def setUp(self) -> None:
        """Create an empty stage with a /World root for acoustic authoring tests."""
        super().setUp()
        await stage_utils.create_new_stage_async()
        stage_utils.define_prim("/World", "Xform")

    async def tearDown(self) -> None:
        """Close the stage created for each acoustic authoring test."""
        super().tearDown()
        stage_utils.close_stage()

    # -- wrap --

    async def test_wrap_existing_prim(self) -> None:
        """Wrap an existing OmniAcoustic prim that already has the acoustic WPM schema."""
        prim = stage_utils.define_prim("/World/acoustic", "OmniAcoustic")
        prim.ApplyAPI("OmniSensorGenericAcousticWpmAPI")
        acoustic = Acoustic("/World/acoustic")
        self.assertEqual(acoustic.paths[0], "/World/acoustic")
        self.assertEqual(acoustic.prims[0].GetTypeName(), "OmniAcoustic")

    async def test_wrap_wrong_type_raises(self) -> None:
        """Reject wrapping a non-acoustic Xform prim as an Acoustic sensor."""
        stage_utils.define_prim("/World/xform", "Xform")
        with self.assertRaises(ValueError):
            Acoustic("/World/xform")

    async def test_wrap_missing_schema_raises(self) -> None:
        """Reject wrapping an OmniAcoustic prim that lacks the acoustic WPM schema."""
        stage_utils.define_prim("/World/acoustic", "OmniAcoustic")
        with self.assertRaises(ValueError):
            Acoustic("/World/acoustic")

    async def test_wrap_with_tick_rate(self) -> None:
        """Apply a tick rate override while wrapping an existing RTX acoustic prim."""
        prim = stage_utils.define_prim("/World/acoustic", "OmniAcoustic")
        prim.ApplyAPI("OmniSensorGenericAcousticWpmAPI")
        acoustic = Acoustic("/World/acoustic", tick_rate=30.0)
        self.assertAlmostEqual(acoustic.prims[0].GetAttribute("omni:sensor:tickRate").Get(), 30.0)

    async def test_wrap_with_attributes(self) -> None:
        """Apply acoustic WPM attributes while wrapping an existing acoustic prim."""
        prim = stage_utils.define_prim("/World/acoustic", "OmniAcoustic")
        prim.ApplyAPI("OmniSensorGenericAcousticWpmAPI")
        acoustic = Acoustic("/World/acoustic", attributes={"omni:sensor:WpmAcoustic:centerFrequency": 40000.0})
        self.assertAlmostEqual(acoustic.prims[0].GetAttribute("omni:sensor:WpmAcoustic:centerFrequency").Get(), 40000.0)

    async def test_wrap_with_nonexistent_attribute_does_not_raise(self) -> None:
        """Ignore unknown attributes without preventing an existing acoustic prim from wrapping."""
        prim = stage_utils.define_prim("/World/acoustic", "OmniAcoustic")
        prim.ApplyAPI("OmniSensorGenericAcousticWpmAPI")
        acoustic = Acoustic("/World/acoustic", attributes={"nonexistent:attr": 42})
        self.assertEqual(acoustic.prims[0].GetTypeName(), "OmniAcoustic")

    # -- create --

    async def test_create_new_prim(self) -> None:
        """Create a new OmniAcoustic prim with the required acoustic WPM schema."""
        acoustic = Acoustic("/World/acoustic")
        self.assertEqual(acoustic.prims[0].GetTypeName(), "OmniAcoustic")
        self.assertTrue(acoustic.prims[0].HasAPI("OmniSensorGenericAcousticWpmAPI"))

    async def test_create_with_tick_rate(self) -> None:
        """Author the tick rate attribute when creating an RTX acoustic prim."""
        acoustic = Acoustic("/World/acoustic", tick_rate=50.0)
        self.assertAlmostEqual(acoustic.prims[0].GetAttribute("omni:sensor:tickRate").Get(), 50.0)

    async def test_create_with_attributes(self) -> None:
        """Author acoustic WPM attributes when creating a new acoustic prim."""
        acoustic = Acoustic(
            "/World/acoustic",
            attributes={"omni:sensor:WpmAcoustic:centerFrequency": 40000.0},
        )
        self.assertAlmostEqual(acoustic.prims[0].GetAttribute("omni:sensor:WpmAcoustic:centerFrequency").Get(), 40000.0)

    async def test_create_with_nonexistent_attribute_does_not_raise(self) -> None:
        """Ignore unknown attributes without preventing new acoustic prim creation."""
        acoustic = Acoustic("/World/acoustic", attributes={"nonexistent:attr": 42})
        self.assertEqual(acoustic.prims[0].GetTypeName(), "OmniAcoustic")

    async def test_create_with_tick_rate_and_attributes(self) -> None:
        """Author both tick rate and acoustic WPM attributes on a new acoustic prim."""
        acoustic = Acoustic(
            "/World/acoustic",
            tick_rate=30.0,
            attributes={"omni:sensor:WpmAcoustic:centerFrequency": 40000.0},
        )
        self.assertAlmostEqual(acoustic.prims[0].GetAttribute("omni:sensor:tickRate").Get(), 30.0)
        self.assertAlmostEqual(acoustic.prims[0].GetAttribute("omni:sensor:WpmAcoustic:centerFrequency").Get(), 40000.0)

    # -- multi-apply schemas --

    async def test_create_with_sensor_mount_attributes(self) -> None:
        """Apply the acoustic sensor-mount API when mount attributes are provided."""
        acoustic = Acoustic(
            "/World/acoustic",
            attributes={
                "omni:sensor:WpmAcoustic:sensorMount:m001:position": (0.1, 0.2, 0.3),
                "omni:sensor:WpmAcoustic:sensorMount:m001:rotation": (0.0, 0.0, 0.0),
            },
        )
        prim = acoustic.prims[0]
        self.assertTrue(prim.HasAPI("OmniSensorWpmAcousticSensorMountAPI", instanceName="m001"))

    async def test_create_with_rx_group_attributes(self) -> None:
        """Apply the acoustic receiver-group API when rxGroup attributes are provided."""
        acoustic = Acoustic(
            "/World/acoustic",
            attributes={"omni:sensor:WpmAcoustic:rxGroup:g001:receiverIndices": [0]},
        )
        prim = acoustic.prims[0]
        self.assertTrue(prim.HasAPI("OmniSensorWpmAcousticRxGroupAPI", instanceName="g001"))

    async def test_create_with_multiple_mount_instances(self) -> None:
        """Apply multiple acoustic sensor-mount API instances from mount attributes."""
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

    async def test_multiple_prims_raises(self) -> None:
        """Reject acoustic path expressions that resolve to more than one prim."""
        prim = stage_utils.define_prim("/World/acoustic_0", "OmniAcoustic")
        prim.ApplyAPI("OmniSensorGenericAcousticWpmAPI")
        prim = stage_utils.define_prim("/World/acoustic_1", "OmniAcoustic")
        prim.ApplyAPI("OmniSensorGenericAcousticWpmAPI")
        with self.assertRaises(ValueError):
            Acoustic("/World/acoustic_.*")

    # -- create from config --

    async def test_create_with_invalid_config_raises(self) -> None:
        """Reject Acoustic.create calls that name an unsupported acoustic config."""
        with self.assertRaises(ValueError):
            Acoustic.create("/World/acoustic", config="NotARealAcousticConfig")

    async def test_create_with_both_config_and_usd_path_raises(self) -> None:
        """Reject Acoustic.create calls that specify both config and USD path inputs."""
        with self.assertRaises(ValueError):
            Acoustic.create("/World/acoustic", config="some_config", usd_path="/some/path.usd")
