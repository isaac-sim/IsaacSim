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

"""Test for non visual material."""

from typing import Literal

import isaacsim.core.experimental.utils.stage as stage_utils
import numpy as np
import omni.kit.test
import warp as wp
from isaacsim.core.experimental.materials import NonVisualMaterial
from isaacsim.core.experimental.materials.impl.non_visual_material import ATTRIBUTE_SPEC, BASE_SPEC, COATING_SPEC
from isaacsim.core.experimental.prims.tests.common import (
    check_allclose,
    check_array,
    check_lists,
    cprint,
    draw_choice,
    draw_indices,
    draw_sample,
    parametrize,
)
from pxr import UsdShade


async def populate_stage(max_num_prims: int, operation: Literal["wrap", "create"]) -> None:
    """Populate stage."""
    # create new stage
    stage = await stage_utils.create_new_stage_async()
    # define prims
    if operation == "wrap":
        for i in range(max_num_prims):
            UsdShade.Material.Define(stage, f"/World/A_{i}")


class TestNonVisualMaterial(omni.kit.test.AsyncTestCase):
    """Test non visual material."""

    async def setUp(self):
        """Method called to prepare the test fixture."""
        super().setUp()

    async def tearDown(self):
        """Method called immediately after the test method has been called."""
        super().tearDown()

    # --------------------------------------------------------------------

    @parametrize(backends=["usd"], prim_class=NonVisualMaterial, populate_stage_func=populate_stage)
    async def test_len(self, prim, num_prims, device, backend):
        """Test len."""
        self.assertEqual(len(prim), num_prims, f"Invalid len ({num_prims} prims)")

    @parametrize(backends=["usd"], prim_class=NonVisualMaterial, populate_stage_func=populate_stage)
    async def test_bases(self, prim, num_prims, device, backend):
        """Test bases."""
        choices = list(BASE_SPEC.keys())
        # test cases
        # - check before applying any values
        bases = prim.get_bases()
        check_lists(["none"] * num_prims, bases)
        # - by indices
        for indices, expected_count in draw_indices(count=num_prims, step=2, types=[list, np.ndarray, wp.array]):
            cprint(f"  |    |-- indices: {type(indices).__name__}, expected_count: {expected_count}")
            count = expected_count
            for v0, expected_v0 in draw_choice(shape=(count,), choices=choices):
                prim.set_bases(v0, indices=indices)
                output = prim.get_bases(indices=indices)
                check_lists(expected_v0, output)
        # - all
        count = num_prims
        for v0, expected_v0 in draw_choice(shape=(count,), choices=choices):
            prim.set_bases(v0)
            output = prim.get_bases()
            check_lists(expected_v0, output)

    @parametrize(backends=["usd"], prim_class=NonVisualMaterial, populate_stage_func=populate_stage)
    async def test_coatings(self, prim, num_prims, device, backend):
        """Test coatings."""
        choices = list(COATING_SPEC.keys())
        # test cases
        # - check before applying any values
        coatings = prim.get_coatings()
        check_lists(["none"] * num_prims, coatings)
        # - by indices
        for indices, expected_count in draw_indices(count=num_prims, step=2, types=[list, np.ndarray, wp.array]):
            cprint(f"  |    |-- indices: {type(indices).__name__}, expected_count: {expected_count}")
            count = expected_count
            for v0, expected_v0 in draw_choice(shape=(count,), choices=choices):
                prim.set_coatings(v0, indices=indices)
                output = prim.get_coatings(indices=indices)
                check_lists(expected_v0, output)
        # - all
        count = num_prims
        for v0, expected_v0 in draw_choice(shape=(count,), choices=choices):
            prim.set_coatings(v0)
            output = prim.get_coatings()
            check_lists(expected_v0, output)

    @parametrize(backends=["usd"], prim_class=NonVisualMaterial, populate_stage_func=populate_stage)
    async def test_attributes(self, prim, num_prims, device, backend):
        """Test attributes."""
        choices = list(ATTRIBUTE_SPEC.keys())
        # test cases
        # - check before applying any values
        attributes = prim.get_attributes()
        check_lists(["none"] * num_prims, attributes)
        # - by indices
        for indices, expected_count in draw_indices(count=num_prims, step=2, types=[list, np.ndarray, wp.array]):
            cprint(f"  |    |-- indices: {type(indices).__name__}, expected_count: {expected_count}")
            count = expected_count
            for v0, expected_v0 in draw_choice(shape=(count,), choices=choices):
                prim.set_attributes(v0, indices=indices)
                output = prim.get_attributes(indices=indices)
                check_lists(expected_v0, output)
        # - all
        count = num_prims
        for v0, expected_v0 in draw_choice(shape=(count,), choices=choices):
            prim.set_attributes(v0)
            output = prim.get_attributes()
            check_lists(expected_v0, output)

    @parametrize(backends=["usd"], prim_class=NonVisualMaterial, populate_stage_func=populate_stage)
    async def test_encode_decode_material_ids(self, prim, num_prims, device, backend):
        """Test encode decode material ids."""
        self.assertTrue(len(BASE_SPEC) > 0, "BASE_SPEC is empty")
        self.assertTrue(len(COATING_SPEC) > 0, "COATING_SPEC is empty")
        self.assertTrue(len(ATTRIBUTE_SPEC) > 0, "ATTRIBUTE_SPEC is empty")
        for (v0, expected_v0), (v1, expected_v1), (v2, expected_v2) in zip(
            draw_choice(shape=(num_prims,), choices=list(BASE_SPEC.keys())),
            draw_choice(shape=(num_prims,), choices=list(COATING_SPEC.keys())),
            draw_choice(shape=(num_prims,), choices=list(ATTRIBUTE_SPEC.keys())),
        ):
            prim.set_bases(v0)
            prim.set_coatings(v1)
            prim.set_attributes(v2)
            encoded_ids = NonVisualMaterial.encode_material_ids(prim)
            decoded_ids = NonVisualMaterial.decode_material_ids(encoded_ids)
            check_lists(expected_v0, [item[0] for item in decoded_ids])
            check_lists(expected_v1, [item[1] for item in decoded_ids])
            check_lists(expected_v2, [item[2] for item in decoded_ids])
