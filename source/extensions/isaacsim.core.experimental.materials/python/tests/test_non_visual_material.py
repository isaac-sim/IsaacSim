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

"""Verifies NonVisualMaterial batches across supported prim backends. Covers length, base material IDs, coating IDs, encoded attributes, and material ID encode/decode behavior."""

from typing import Any, Literal

import isaacsim.core.experimental.utils.stage as stage_utils
import numpy as np
import omni.kit.test
import warp as wp
from isaacsim.core.experimental.materials import NonVisualMaterial
from isaacsim.core.experimental.materials.impl import non_visual_material as non_visual_material_impl
from isaacsim.core.experimental.materials.impl.non_visual_material import ATTRIBUTE_SPEC, BASE_SPEC, COATING_SPEC
from isaacsim.core.experimental.prims.tests.common import (
    check_lists,
    cprint,
    draw_choice,
    draw_indices,
    parametrize,
)
from pxr import Sdf, UsdShade


class _SettingsWithoutNonVisualMaterialPrefix:
    """Settings test double that behaves like Kit before the prefix is contributed."""

    def __init__(self) -> None:
        self.default_path = None
        self.default_value = None

    def get(self, path: str) -> None:
        return None

    def set_default_string(self, path: str, value: str) -> None:
        self.default_path = path
        self.default_value = value


async def populate_stage(max_num_prims: int, operation: Literal["wrap", "create"]) -> None:
    """Populate stage.

    Args:
        max_num_prims: Maximum number of material prims to pre-author.
        operation: Prim setup operation requested by the parametrized test.
    """
    # create new stage
    stage = await stage_utils.create_new_stage_async()
    # define prims
    if operation == "wrap":
        for i in range(max_num_prims):
            UsdShade.Material.Define(stage, f"/World/A_{i}")


class TestNonVisualMaterial(omni.kit.test.AsyncTestCase):
    """Test non visual material."""

    async def setUp(self) -> None:
        """Method called to prepare the test fixture."""
        super().setUp()

    async def tearDown(self) -> None:
        """Method called immediately after the test method has been called."""
        super().tearDown()

    # --------------------------------------------------------------------

    @parametrize(backends=["usd"], prim_class=NonVisualMaterial, populate_stage_func=populate_stage)
    async def test_len(self, prim: Any, num_prims: Any, device: Any, backend: Any) -> None:
        """Test len.

        Args:
            prim: Material wrapper under test.
            num_prims: Number of material prims in the parametrized case.
            device: Simulation device selected by the parametrized case.
            backend: Prim backend selected by the parametrized case.
        """
        self.assertEqual(len(prim), num_prims, f"Invalid len ({num_prims} prims)")

    @parametrize(backends=["usd"], prim_class=NonVisualMaterial, populate_stage_func=populate_stage)
    async def test_bases(self, prim: Any, num_prims: Any, device: Any, backend: Any) -> None:
        """Test bases.

        Args:
            prim: Material wrapper under test.
            num_prims: Number of material prims in the parametrized case.
            device: Simulation device selected by the parametrized case.
            backend: Prim backend selected by the parametrized case.
        """
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
    async def test_coatings(self, prim: Any, num_prims: Any, device: Any, backend: Any) -> None:
        """Test coatings.

        Args:
            prim: Material wrapper under test.
            num_prims: Number of material prims in the parametrized case.
            device: Simulation device selected by the parametrized case.
            backend: Prim backend selected by the parametrized case.
        """
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
    async def test_attributes(self, prim: Any, num_prims: Any, device: Any, backend: Any) -> None:
        """Test attributes.

        Args:
            prim: Material wrapper under test.
            num_prims: Number of material prims in the parametrized case.
            device: Simulation device selected by the parametrized case.
            backend: Prim backend selected by the parametrized case.
        """
        # attributes are authored as a token[] array, so each material holds a list of attributes
        # - check the default value before applying anything
        self.assertEqual(prim.get_attributes(), [["none"]] * num_prims)
        # - broadcast a single attribute (string) to all prims
        prim.set_attributes("emissive")
        self.assertEqual(prim.get_attributes(), [["emissive"]] * num_prims)
        # - broadcast a shared multi-attribute set (list of strings) to all prims
        prim.set_attributes(["emissive", "retroreflective"])
        self.assertEqual(prim.get_attributes(), [["emissive", "retroreflective"]] * num_prims)
        # - per-prim attribute sets (list of lists)
        per_prim = [["single_sided"] if i % 2 == 0 else ["emissive", "visually_transparent"] for i in range(num_prims)]
        prim.set_attributes(per_prim)
        self.assertEqual(prim.get_attributes(), per_prim)
        # - by indices
        prim.set_attributes("none")
        prim.set_attributes("retroreflective", indices=[0])
        self.assertEqual(prim.get_attributes(), [["retroreflective"]] + [["none"]] * (num_prims - 1))
        # - invalid attribute raises
        with self.assertRaises(ValueError):
            prim.set_attributes("not_a_real_attribute")

    @parametrize(backends=["usd"], prim_class=NonVisualMaterial, populate_stage_func=populate_stage)
    async def test_attribute_usd_types(self, prim: Any, num_prims: Any, device: Any, backend: Any) -> None:
        """Test that non-visual material attributes are authored using SimReady spec USD types.

        Args:
            prim: Material wrapper under test.
            num_prims: Number of material prims in the parametrized case.
            device: Simulation device selected by the parametrized case.
            backend: Prim backend selected by the parametrized case.
        """
        from isaacsim.core.experimental.materials.impl.non_visual_material import (
            ATTRIBUTE_ATTR,
            BASE_ATTR,
            COATING_ATTR,
        )

        for usd_prim in prim.prims:
            self.assertEqual(usd_prim.GetAttribute(BASE_ATTR).GetTypeName(), Sdf.ValueTypeNames.Token)
            self.assertEqual(usd_prim.GetAttribute(COATING_ATTR).GetTypeName(), Sdf.ValueTypeNames.Token)
            self.assertEqual(usd_prim.GetAttribute(ATTRIBUTE_ATTR).GetTypeName(), Sdf.ValueTypeNames.TokenArray)

    async def test_attribute_names_use_simready_prefix_when_setting_missing(self) -> None:
        """Test that missing Kit settings fall back to the SimReady non-visual material prefix."""
        settings = _SettingsWithoutNonVisualMaterialPrefix()

        prefix = non_visual_material_impl._get_non_visual_material_prefix(settings)

        self.assertEqual(prefix, "omni:simready:nonvisual")
        self.assertEqual(
            settings.default_path,
            non_visual_material_impl._NON_VISUAL_MATERIAL_PREFIX_SETTING,
        )
        self.assertEqual(settings.default_value, "omni:simready:nonvisual")

    @parametrize(backends=["usd"], prim_class=NonVisualMaterial, populate_stage_func=populate_stage)
    async def test_no_surface_shader_authored(self, prim: Any, num_prims: Any, device: Any, backend: Any) -> None:
        """A non-visual material leaves the stage free of surface shaders it does not need.

        Args:
            prim: Material wrapper under test.
            num_prims: Number of material prims in the parametrized case.
            device: Simulation device selected by the parametrized case.
            backend: Prim backend selected by the parametrized case.
        """
        for material in prim.materials:
            self.assertFalse(
                material.GetSurfaceOutput().HasConnectedSource(),
                "material outputs:surface is connected",
            )
            self.assertFalse(
                any(child.IsA(UsdShade.Shader) for child in material.GetPrim().GetChildren()),
                "material has a shader child prim",
            )

    async def test_existing_surface_shader_preserved(self) -> None:
        """Wrapping a visual material keeps its shader and ``outputs:surface`` connection intact."""
        stage = await stage_utils.create_new_stage_async()
        material = UsdShade.Material.Define(stage, "/World/visual_material")
        shader = UsdShade.Shader.Define(stage, "/World/visual_material/Shader")
        shader.CreateIdAttr("UsdPreviewSurface")
        shader.CreateOutput("surface", Sdf.ValueTypeNames.Token)
        material.CreateSurfaceOutput().ConnectToSource(shader.ConnectableAPI(), "surface")

        wrapped = NonVisualMaterial("/World/visual_material", bases="aluminum")

        self.assertTrue(material.GetSurfaceOutput().HasConnectedSource())
        self.assertEqual(shader.GetIdAttr().Get(), "UsdPreviewSurface")
        self.assertEqual(wrapped.get_bases(), ["aluminum"])

    @parametrize(backends=["usd"], prim_class=NonVisualMaterial, populate_stage_func=populate_stage)
    async def test_encode_decode_multiple_attributes(
        self, prim: Any, num_prims: Any, device: Any, backend: Any
    ) -> None:
        """Test encoding/decoding of multiple combined attributes (bitfield).

        Args:
            prim: Material wrapper under test.
            num_prims: Number of material prims in the parametrized case.
            device: Simulation device selected by the parametrized case.
            backend: Prim backend selected by the parametrized case.
        """
        prim.set_bases("aluminum")
        prim.set_coatings("paint")
        prim.set_attributes(["emissive", "retroreflective"])
        # emissive (1) | retroreflective (2) -> 3, shifted into bits 11-15 -> 3 << 11
        expected_id = BASE_SPEC["aluminum"] + (COATING_SPEC["paint"] << 8) + (0x3 << 11)
        encoded_ids = NonVisualMaterial.encode_material_ids(prim)
        check_lists([expected_id] * num_prims, encoded_ids.numpy().flatten().tolist())
        # decoding recovers both attributes
        decoded_ids = NonVisualMaterial.decode_material_ids(encoded_ids)
        for base, coating, attributes in decoded_ids:
            self.assertEqual(base, "aluminum")
            self.assertEqual(coating, "paint")
            self.assertEqual(sorted(attributes), ["emissive", "retroreflective"])

    @parametrize(backends=["usd"], prim_class=NonVisualMaterial, populate_stage_func=populate_stage)
    async def test_encode_decode_material_ids(self, prim: Any, num_prims: Any, device: Any, backend: Any) -> None:
        """Test encode decode material ids.

        Args:
            prim: Material wrapper under test.
            num_prims: Number of material prims in the parametrized case.
            device: Simulation device selected by the parametrized case.
            backend: Prim backend selected by the parametrized case.
        """
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
            # apply one attribute per prim (as single-element attribute sets)
            prim.set_attributes([[attribute] for attribute in v2])
            encoded_ids = NonVisualMaterial.encode_material_ids(prim)
            decoded_ids = NonVisualMaterial.decode_material_ids(encoded_ids)
            check_lists(expected_v0, [item[0] for item in decoded_ids])
            check_lists(expected_v1, [item[1] for item in decoded_ids])
            # decoded attributes are returned as lists (single-element for a single attribute)
            check_lists([[attribute] for attribute in expected_v2], [item[2] for item in decoded_ids])
