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
import omni.kit.commands
import omni.kit.test
import warp as wp
from isaacsim.core.experimental.materials import PreviewSurfaceMaterial
from isaacsim.core.experimental.prims.tests.common import (
    check_allclose,
    check_array,
    cprint,
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
            omni.kit.commands.execute(
                "CreatePreviewSurfaceMaterialPrim", mtl_path=f"/World/A_{i}", select_new_prim=False
            )


class TestPreviewSurface(omni.kit.test.AsyncTestCase):
    async def setUp(self):
        """Method called to prepare the test fixture"""
        super().setUp()

    async def tearDown(self):
        """Method called immediately after the test method has been called"""
        super().tearDown()

    # --------------------------------------------------------------------

    @parametrize(backends=["usd"], prim_class=PreviewSurfaceMaterial, populate_stage_func=populate_stage)
    async def test_len(self, prim, num_prims, device, backend):
        self.assertEqual(len(prim), num_prims, f"Invalid len ({num_prims} prims)")

    @parametrize(backends=["usd"], prim_class=PreviewSurfaceMaterial, populate_stage_func=populate_stage)
    async def test_properties_and_getters(self, prim, num_prims, device, backend):
        # test cases (properties)
        # - materials
        self.assertEqual(len(prim.materials), num_prims, f"Invalid materials len ({num_prims} prims)")
        for usd_prim in prim.prims:
            self.assertTrue(usd_prim.IsValid() and usd_prim.IsA(UsdShade.Material), f"Invalid material")
        # - shaders
        self.assertEqual(len(prim.shaders), num_prims, f"Invalid shaders len ({num_prims} prims)")
        for shader in prim.shaders:
            self.assertTrue(isinstance(shader, UsdShade.Shader), f"Invalid shader")

    @parametrize(backends=["usd"], prim_class=PreviewSurfaceMaterial, populate_stage_func=populate_stage)
    async def test_input_values(self, prim, num_prims, device, backend):
        cases = {
            "diffuseColor": lambda count: draw_sample(shape=(count, 3), dtype=wp.float32),
            "emissiveColor": lambda count: draw_sample(shape=(count, 3), dtype=wp.float32),
            "specularColor": lambda count: draw_sample(shape=(count, 3), dtype=wp.float32),
            "useSpecularWorkflow": lambda count: draw_sample(
                shape=(count, 1), dtype=wp.int32, high=2
            ),  # one above (exclusive)
            "metallic": lambda count: draw_sample(shape=(count, 1), dtype=wp.float32),
            "roughness": lambda count: draw_sample(shape=(count, 1), dtype=wp.float32),
            "clearcoat": lambda count: draw_sample(shape=(count, 1), dtype=wp.float32),
            "clearcoatRoughness": lambda count: draw_sample(shape=(count, 1), dtype=wp.float32),
            "opacity": lambda count: draw_sample(shape=(count, 1), dtype=wp.float32),
            "opacityThreshold": lambda count: draw_sample(shape=(count, 1), dtype=wp.float32),
            "ior": lambda count: draw_sample(shape=(count, 1), dtype=wp.float32),
            "normal": lambda count: draw_sample(shape=(count, 3), dtype=wp.float32),
            "displacement": lambda count: draw_sample(shape=(count, 1), dtype=wp.float32),
        }
        # check inputs
        for name in cases:
            assert name in prim._inputs, f"Invalid input: {name}"
        for name in prim._inputs:
            assert name in cases, f"Missing case: {name}"
        # test cases
        for name, sample_func in cases.items():
            cprint(f"  |   input: {name}")
            for indices, expected_count in draw_indices(count=num_prims, step=2):
                cprint(f"  |    |-- indices: {type(indices).__name__}, expected_count: {expected_count}")
                for v0, expected_v0 in sample_func(expected_count):
                    prim.set_input_values(name, values=v0, indices=indices)
                    output = prim.get_input_values(name, indices=indices)
                    check_array(output, shape=expected_v0.shape, device=device)
                    check_allclose(expected_v0, output, given=(v0,))
