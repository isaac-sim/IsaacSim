# SPDX-FileCopyrightText: Copyright (c) 2021-2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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

from typing import Literal

import isaacsim.core.experimental.utils.stage as stage_utils
import omni.kit.test
import omni.physxcommands
import warp as wp
from isaacsim.core.experimental.materials import RigidBodyMaterial
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


async def populate_stage(max_num_prims: int, operation: Literal["wrap", "create"], **kwargs) -> None:
    # create new stage
    stage = await stage_utils.create_new_stage_async()
    # define prims
    if operation == "wrap":
        for i in range(max_num_prims):
            omni.kit.commands.execute("AddRigidBodyMaterial", stage=stage, path=f"/World/A_{i}")


class TestRigidBodyMaterial(omni.kit.test.AsyncTestCase):
    async def setUp(self):
        """Method called to prepare the test fixture"""
        super().setUp()

    async def tearDown(self):
        """Method called immediately after the test method has been called"""
        super().tearDown()

    # --------------------------------------------------------------------

    @parametrize(backends=["usd"], prim_class=RigidBodyMaterial, populate_stage_func=populate_stage)
    async def test_len(self, prim, num_prims, device, backend):
        self.assertEqual(len(prim), num_prims, f"Invalid len ({num_prims} prims)")

    @parametrize(backends=["usd"], prim_class=RigidBodyMaterial, populate_stage_func=populate_stage)
    async def test_properties_and_getters(self, prim, num_prims, device, backend):
        # test cases (properties)
        # - materials
        self.assertEqual(len(prim.materials), num_prims, f"Invalid materials len ({num_prims} prims)")
        for usd_prim in prim.prims:
            self.assertTrue(usd_prim.IsValid() and usd_prim.IsA(UsdShade.Material), f"Invalid material")

    @parametrize(backends=["usd"], prim_class=RigidBodyMaterial, populate_stage_func=populate_stage)
    async def test_friction_coefficients(self, prim, num_prims, device, backend):
        for indices, expected_count in draw_indices(count=num_prims, step=2):
            cprint(f"  |    |-- indices: {type(indices).__name__}, expected_count: {expected_count}")
            for (v0, expected_v0), (v1, expected_v1) in zip(
                draw_sample(shape=(expected_count, 1), dtype=wp.float32),
                draw_sample(shape=(expected_count, 1), dtype=wp.float32),
            ):
                prim.set_friction_coefficients(v0, v1, indices=indices)
                output = prim.get_friction_coefficients(indices=indices)
                check_array(output, shape=(expected_count, 1), dtype=wp.float32, device=device)
                check_allclose((expected_v0, expected_v1), output, given=(v0, v1))

    @parametrize(backends=["usd"], prim_class=RigidBodyMaterial, populate_stage_func=populate_stage)
    async def test_restitution_coefficients(self, prim, num_prims, device, backend):
        for indices, expected_count in draw_indices(count=num_prims, step=2):
            cprint(f"  |    |-- indices: {type(indices).__name__}, expected_count: {expected_count}")
            for v0, expected_v0 in draw_sample(shape=(expected_count, 1), dtype=wp.float32):
                prim.set_restitution_coefficients(v0, indices=indices)
                output = prim.get_restitution_coefficients(indices=indices)
                check_array(output, shape=(expected_count, 1), dtype=wp.float32, device=device)
                check_allclose(expected_v0, output, given=(v0,))

    @parametrize(backends=["usd"], prim_class=RigidBodyMaterial, populate_stage_func=populate_stage)
    async def test_densities(self, prim, num_prims, device, backend):
        for indices, expected_count in draw_indices(count=num_prims, step=2):
            cprint(f"  |    |-- indices: {type(indices).__name__}, expected_count: {expected_count}")
            for v0, expected_v0 in draw_sample(shape=(expected_count, 1), dtype=wp.float32):
                prim.set_densities(v0, indices=indices)
                output = prim.get_densities(indices=indices)
                check_array(output, shape=(expected_count, 1), dtype=wp.float32, device=device)
                check_allclose(expected_v0, output, given=(v0,))

    @parametrize(backends=["usd"], prim_class=RigidBodyMaterial, populate_stage_func=populate_stage)
    async def test_combine_modes(self, prim, num_prims, device, backend):
        choices = ["average", "max", "min", "multiply"]
        for indices, expected_count in draw_indices(count=num_prims, step=2):
            cprint(f"  |    |-- indices: {type(indices).__name__}, expected_count: {expected_count}")
            for (v0, expected_v0), (v1, expected_v1), (v2, expected_v2) in zip(
                draw_choice(shape=(expected_count,), choices=choices),
                draw_choice(shape=(expected_count,), choices=choices),
                draw_choice(shape=(expected_count,), choices=choices),
            ):
                prim.set_combine_modes(v0, v1, v2, indices=indices)
                output = prim.get_combine_modes(indices=indices)
                check_lists(expected_v0, output[0])
                check_lists(expected_v1, output[1])
                check_lists(expected_v2, output[2])

    @parametrize(backends=["usd"], prim_class=RigidBodyMaterial, populate_stage_func=populate_stage)
    async def test_enabled_compliant_contacts(self, prim, num_prims, device, backend):
        prim.set_compliant_contact_gains(stiffnesses=[100.0])  # set stiffnesses > 0 before testing
        for indices, expected_count in draw_indices(count=num_prims, step=2):
            cprint(f"  |    |-- indices: {type(indices).__name__}, expected_count: {expected_count}")
            for v0, expected_v0 in draw_sample(shape=(expected_count, 1), dtype=wp.bool):
                prim.set_enabled_compliant_contacts(v0, indices=indices)
                output = prim.get_enabled_compliant_contacts(indices=indices)
                check_array(output, shape=(expected_count, 1), dtype=wp.bool, device=device)
                check_allclose(expected_v0, output, given=(v0,))

    @parametrize(backends=["usd"], prim_class=RigidBodyMaterial, populate_stage_func=populate_stage)
    async def test_compliant_contact_gains(self, prim, num_prims, device, backend):
        for indices, expected_count in draw_indices(count=num_prims, step=2):
            cprint(f"  |    |-- indices: {type(indices).__name__}, expected_count: {expected_count}")
            for (v0, expected_v0), (v1, expected_v1) in zip(
                draw_sample(shape=(expected_count, 1), dtype=wp.float32, low=0.5),
                draw_sample(shape=(expected_count, 1), dtype=wp.float32),
            ):
                prim.set_compliant_contact_gains(v0, v1, indices=indices)
                output = prim.get_compliant_contact_gains(indices=indices)
                check_array(output, shape=(expected_count, 1), dtype=wp.float32, device=device)
                check_allclose((expected_v0, expected_v1), output, given=(v0, v1))
