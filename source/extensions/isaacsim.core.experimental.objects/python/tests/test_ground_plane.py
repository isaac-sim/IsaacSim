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

"""Test for ground plane."""

from typing import Literal

import isaacsim.core.experimental.utils.stage as stage_utils
import numpy as np
import omni.kit.commands
import omni.kit.test
import warp as wp
from isaacsim.core.experimental.objects import GroundPlane, Mesh, Plane
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
from omni.physx.scripts import physicsUtils
from pxr import PhysicsSchemaTools, UsdGeom


async def populate_stage(max_num_prims: int, operation: Literal["wrap", "create"], **kwargs) -> None:
    """Populate stage."""
    # create new stage
    stage = await stage_utils.create_new_stage_async()
    # define prims
    if operation == "wrap":
        for i in range(max_num_prims):
            if i % 2:
                physicsUtils.add_ground_plane(stage, f"/World/A_{i}", "Z", 10.0, (0, 0, i), [i / max_num_prims] * 3)
            else:
                PhysicsSchemaTools.addGroundPlane(stage, f"/World/A_{i}", "Z", 10.0, (0, 0, i), [i / max_num_prims] * 3)


class TestGroundPlane(omni.kit.test.AsyncTestCase):
    """Test ground plane."""

    async def setUp(self):
        """Method called to prepare the test fixture."""
        super().setUp()

    async def tearDown(self):
        """Method called immediately after the test method has been called."""
        super().tearDown()

    # --------------------------------------------------------------------

    async def test_instances(self):
        """Test instances."""
        await stage_utils.create_new_stage_async()
        path = "/World/ground_plane"
        ground_plane = GroundPlane(path)  # create
        self.assertEqual(len(ground_plane.prims[0].GetChildren()), 3, "Invalid number of child prims")
        ground_plane = GroundPlane(path)  # wrap
        self.assertEqual(len(ground_plane.prims[0].GetChildren()), 3, "Invalid number of child prims")

    @parametrize(backends=["usd"], prim_class=GroundPlane, populate_stage_func=populate_stage)
    async def test_len(self, prim, num_prims, device, backend):
        """Test len."""
        self.assertEqual(len(prim), num_prims, f"Invalid len ({num_prims} prims)")

    @parametrize(backends=["usd"], prim_class=GroundPlane, populate_stage_func=populate_stage)
    async def test_properties_and_getters(self, prim, num_prims, device, backend):
        """Test properties and getters."""
        # test cases (properties)
        # - geoms
        self.assertEqual(len(prim._geoms), num_prims, f"Invalid geoms len ({num_prims} prims)")
        for usd_prim, geom in zip(prim.prims, prim._geoms.geoms):
            self.assertTrue(geom.GetPrim().IsA(UsdGeom.Plane), f"Invalid geom type: {geom.GetPrim().GetTypeName()}")
            self.assertTrue(
                usd_prim.IsValid() and usd_prim.IsA(UsdGeom.Xform), f"Invalid prim type: {usd_prim.GetTypeName()}"
            )
        # - Plane and Mesh instances
        self.assertIsInstance(prim.planes, Plane)
        self.assertEqual(len(prim.planes), num_prims, f"Invalid planes len ({num_prims} prims)")
        self.assertIsInstance(prim.meshes, Mesh)
        self.assertEqual(len(prim.meshes), num_prims, f"Invalid meshes len ({num_prims} prims)")

    @parametrize(backends=["usd"], prim_class=GroundPlane, populate_stage_func=populate_stage)
    async def test_enabled_collisions(self, prim, num_prims, device, backend):
        """Test enabled collisions."""
        for indices, expected_count in draw_indices(count=num_prims, step=2):
            cprint(f"  |    |-- indices: {type(indices).__name__}, expected_count: {expected_count}")
            for v0, expected_v0 in draw_sample(shape=(expected_count, 1), dtype=wp.bool):
                prim.set_enabled_collisions(v0, indices=indices)
                output = prim.get_enabled_collisions(indices=indices)
                check_array(output, shape=(expected_count, 1), dtype=wp.bool, device=device)
                check_allclose(expected_v0, output, given=(v0,))

    @parametrize(backends=["usd"], prim_class=GroundPlane, populate_stage_func=populate_stage)
    async def test_offsets(self, prim, num_prims, device, backend):
        """Test offsets."""
        for indices, expected_count in draw_indices(count=num_prims, step=2):
            cprint(f"  |    |-- indices: {type(indices).__name__}, expected_count: {expected_count}")
            for (v0, expected_v0), (v1, expected_v1) in zip(
                draw_sample(shape=(expected_count, 1), dtype=wp.float32),
                draw_sample(shape=(expected_count, 1), dtype=wp.float32),
            ):
                prim.set_offsets(v0, v1, indices=indices)
                output = prim.get_offsets(indices=indices)
                check_array(output, shape=(expected_count, 1), dtype=wp.float32, device=device)
                check_allclose((expected_v0, expected_v1), output, given=(v0, v1))

    @parametrize(backends=["usd"], prim_class=GroundPlane, populate_stage_func=populate_stage)
    async def test_torsional_patch_radii(self, prim, num_prims, device, backend):
        """Test torsional patch radii."""
        # test cases
        # - standard
        for indices, expected_count in draw_indices(count=num_prims, step=2):
            cprint(f"  |    |-- indices: {type(indices).__name__}, expected_count: {expected_count}")
            for v0, expected_v0 in draw_sample(shape=(expected_count, 1), dtype=wp.float32):
                prim.set_torsional_patch_radii(v0, indices=indices)
                output = prim.get_torsional_patch_radii(indices=indices)
                check_array(output, shape=(expected_count, 1), dtype=wp.float32, device=device)
                check_allclose(expected_v0, output, given=(v0,))
        # - minimum
        for indices, expected_count in draw_indices(count=num_prims, step=2):
            cprint(f"  |    |-- indices: {type(indices).__name__}, expected_count: {expected_count}")
            for v0, expected_v0 in draw_sample(shape=(expected_count, 1), dtype=wp.float32):
                prim.set_torsional_patch_radii(v0, indices=indices, minimum=True)
                output = prim.get_torsional_patch_radii(indices=indices, minimum=True)
                check_array(output, shape=(expected_count, 1), dtype=wp.float32, device=device)
                check_allclose(expected_v0, output, given=(v0,))

    @parametrize(backends=["usd"], prim_class=GroundPlane, populate_stage_func=populate_stage)
    async def test_physics_materials(self, prim, num_prims, device, backend):
        """Test physics materials."""
        from isaacsim.core.experimental.materials import RigidBodyMaterial

        choices = [
            RigidBodyMaterial(
                "/physics_materials/aluminum", dynamic_frictions=[0.4], static_frictions=[1.1], restitutions=[0.1]
            ),
            RigidBodyMaterial(
                "/physics_materials/wood", dynamic_frictions=[0.2], static_frictions=[0.5], restitutions=[0.6]
            ),
        ]
        # test cases
        # - check the number of applied materials before applying any material
        output = prim.get_applied_physics_materials()
        number_of_materials = sum(1 for material in output if material is not None)
        assert number_of_materials == 0, f"No material should have been applied. Applied: {number_of_materials}"
        # - by indices
        for indices, expected_count in draw_indices(count=num_prims, step=2, types=[list, np.ndarray, wp.array]):
            cprint(f"  |    |-- indices: {type(indices).__name__}, expected_count: {expected_count}")
            count = expected_count
            for v0, expected_v0 in draw_choice(shape=(count,), choices=choices):
                prim.apply_physics_materials(v0, indices=indices)
                output = prim.get_applied_physics_materials(indices=indices)
                check_lists(expected_v0, output, predicate=lambda x: x.paths[0])
        # - check the number of applied materials after applying materials by indices
        output = prim.get_applied_physics_materials()
        number_of_materials = sum(1 for material in output if material is not None)
        assert (
            number_of_materials == count
        ), f"{count} materials should have been applied. Applied: {number_of_materials}"
        # - all
        count = num_prims
        for v0, expected_v0 in draw_choice(shape=(count,), choices=choices):
            prim.apply_physics_materials(v0)
            output = prim.get_applied_physics_materials()
            check_lists(expected_v0, output, predicate=lambda x: x.paths[0])
        # - check the number of applied materials after applying materials by all
        output = prim.get_applied_physics_materials()
        number_of_materials = sum(1 for material in output if material is not None)
        assert (
            number_of_materials == count
        ), f"{count} materials should have been applied. Applied: {number_of_materials}"

    @parametrize(backends=["usd"], prim_class=GroundPlane, populate_stage_func=populate_stage)
    async def test_apply_visual_templates(self, prim, num_prims, device, backend):
        """Test apply visual templates."""
        choices = ["wireframe-blue"]
        for indices, expected_count in draw_indices(count=num_prims, step=2):
            cprint(f"  |    |-- indices: {type(indices).__name__}, expected_count: {expected_count}")
            for v0, expected_v0 in draw_choice(shape=(expected_count,), choices=choices):
                prim.apply_visual_templates(v0, indices=indices)
