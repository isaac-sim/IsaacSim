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

"""Test for shape sphere."""

from typing import Literal

import isaacsim.core.experimental.utils.stage as stage_utils
import omni.kit.test
import warp as wp
from isaacsim.core.experimental.objects import Sphere as TargetShape
from isaacsim.core.experimental.prims.tests.common import (
    check_allclose,
    check_array,
    cprint,
    draw_indices,
    draw_sample,
    parametrize,
)
from pxr import UsdGeom


async def populate_stage(max_num_prims: int, operation: Literal["wrap", "create"], **kwargs) -> None:
    """Populate stage."""
    # create new stage
    await stage_utils.create_new_stage_async()
    # define prims
    if operation == "wrap":
        for i in range(max_num_prims):
            stage_utils.define_prim(f"/World/A_{i}", "Sphere")


class TestSphere(omni.kit.test.AsyncTestCase):
    """Test sphere."""

    async def setUp(self):
        """Method called to prepare the test fixture."""
        super().setUp()

    async def tearDown(self):
        """Method called immediately after the test method has been called."""
        super().tearDown()

    # --------------------------------------------------------------------

    @parametrize(backends=["usd"], prim_class=TargetShape, populate_stage_func=populate_stage)
    async def test_len(self, prim, num_prims, device, backend):
        """Test len."""
        self.assertEqual(len(prim), num_prims, f"Invalid len ({num_prims} prims)")

    @parametrize(backends=["usd"], prim_class=TargetShape, populate_stage_func=populate_stage)
    async def test_geoms(self, prim, num_prims, device, backend):
        """Test geoms."""
        for usd_prim, geom in zip(prim.prims, prim.geoms):
            self.assertTrue(usd_prim.IsA(UsdGeom.Sphere), f"Invalid geom type: {usd_prim.GetTypeName()}")

    @parametrize(backends=["usd"], prim_class=TargetShape, populate_stage_func=populate_stage)
    async def test_radii(self, prim, num_prims, device, backend):
        """Test radii."""
        for indices, expected_count in draw_indices(count=num_prims, step=2):
            cprint(f"  |    |-- indices: {type(indices).__name__}, expected_count: {expected_count}")
            for v0, expected_v0 in draw_sample(shape=(expected_count, 1), dtype=wp.float32):
                prim.set_radii(v0, indices=indices)
                output = prim.get_radii(indices=indices)
                check_array(output, shape=(expected_count, 1), dtype=wp.float32, device=device)
                check_allclose(expected_v0, output, given=(v0,))
