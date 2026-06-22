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

"""Verify public docstring coverage for RTX sensor APIs and GMO parsing helpers."""

import carb
import isaacsim.core.experimental.utils.app as app_utils
import isaacsim.core.experimental.utils.stage as stage_utils
import isaacsim.test.docstring
from isaacsim.sensors.experimental.rtx import (
    AcousticSensor,
    LidarSensor,
    RadarSensor,
    parse_generic_model_output_data,
    parse_stable_id_map_data,
)


class TestExtensionDocstrings(isaacsim.test.docstring.AsyncDocTestCase):
    """Test extension docstrings."""

    async def setUp(self) -> None:
        """Method called to prepare the test fixture."""
        super().setUp()
        # create new stage
        await stage_utils.create_new_stage_async()
        stage_utils.define_prim(f"/World", "Xform")

    async def tearDown(self) -> None:
        """Method called immediately after the test method has been called."""
        super().tearDown()
        app_utils.stop(commit=True)
        await app_utils.update_app_async()

    # --------------------------------------------------------------------

    async def test_lidar_sensor_docstrings(self) -> None:
        """Test rtx lidar sensor docstrings."""
        # define prims
        stage_utils.define_prim(f"/World/cube", "Cube")
        prim = stage_utils.define_prim(f"/World/prim_0", "OmniLidar")
        prim.ApplyAPI("OmniSensorGenericLidarCoreAPI")
        # test case
        await self.assertDocTests(LidarSensor)

    async def test_radar_sensor_docstrings(self) -> None:
        # define prims
        """Run doctests for RadarSensor after authoring a radar prim and enabling Motion BVH."""
        stage_utils.define_prim(f"/World/cube", "Cube")
        prim = stage_utils.define_prim(f"/World/prim_0", "OmniRadar")
        prim.ApplyAPI("OmniSensorGenericRadarWpmDmatAPI")
        # enable Motion BVH for radar
        carb.settings.get_settings().set("/renderer/raytracingMotion/enabled", True)
        # test case
        await self.assertDocTests(RadarSensor)

    async def test_acoustic_sensor_docstrings(self) -> None:
        # define prims
        """Run doctests for AcousticSensor after authoring an acoustic prim."""
        stage_utils.define_prim(f"/World/cube", "Cube")
        prim = stage_utils.define_prim(f"/World/prim_0", "OmniAcoustic")
        prim.ApplyAPI("OmniSensorGenericAcousticWpmAPI")
        # test case
        await self.assertDocTests(AcousticSensor)

    async def test_parse_generic_model_output_data_docstrings(self) -> None:
        """Test parse generic model output data docstrings."""
        self.assertDocTest(parse_generic_model_output_data)

    async def test_parse_stable_id_map_data_docstrings(self) -> None:
        """Test parse stable id map data docstrings."""
        self.assertDocTest(parse_stable_id_map_data)
