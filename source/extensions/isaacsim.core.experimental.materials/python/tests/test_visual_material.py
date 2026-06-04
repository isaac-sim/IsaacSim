# SPDX-FileCopyrightText: Copyright (c) 2021-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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

"""Test for visual material."""

from typing import Literal

import isaacsim.core.experimental.utils.stage as stage_utils
import omni.kit.test
from isaacsim.core.experimental.materials import (
    OmniGlassMaterial,
    OmniPbrMaterial,
    PreviewSurfaceMaterial,
    VisualMaterial,
)


async def populate_stage(max_num_prims: int, operation: Literal["wrap", "create"]) -> None:
    """Populate stage."""
    # create new stage
    await stage_utils.create_new_stage_async()
    # define prims
    if operation == "wrap":
        for i in range(max_num_prims):
            omni.kit.commands.execute(
                "CreatePrimWithDefaultXform",
                prim_type="SphereLight",
                prim_path=f"/World/A_{i}",
                attributes={"inputs:intensity": 30000},
                select_new_prim=False,
            )


class TestVisualMaterial(omni.kit.test.AsyncTestCase):
    """Test visual material."""

    async def setUp(self):
        """Method called to prepare the test fixture."""
        super().setUp()

    async def tearDown(self):
        """Method called immediately after the test method has been called."""
        super().tearDown()

    # --------------------------------------------------------------------

    async def test_fetch_instances(self):
        """Test fetch instances."""
        await stage_utils.create_new_stage_async()
        # create materials
        OmniGlassMaterial("/World/material_01")
        OmniPbrMaterial("/World/material_02")
        PreviewSurfaceMaterial("/World/material_03")
        # fetch instances
        instances = VisualMaterial.fetch_instances(
            [
                "/World",
                "/World/material_01",
                "/World/material_02",
                "/World/material_03",
            ]
        )
        # check
        self.assertEqual(len(instances), 4)
        self.assertIsNone(instances[0])
        self.assertIsInstance(instances[1], OmniGlassMaterial)
        self.assertIsInstance(instances[2], OmniPbrMaterial)
        self.assertIsInstance(instances[3], PreviewSurfaceMaterial)
