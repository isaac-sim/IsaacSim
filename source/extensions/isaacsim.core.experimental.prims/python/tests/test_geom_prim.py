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

"""Verifies GeomPrim runtime creation and collision-related property access. Covers collision enablement, collision approximation, contact offsets, torsional patch radii, and physics materials."""

from typing import Any, Literal

import isaacsim.core.experimental.utils.stage as stage_utils
import numpy as np
import omni.kit.test
import warp as wp
from isaacsim.core.experimental.prims import GeomPrim
from isaacsim.core.experimental.utils.backend import use_backend

from .common import (
    check_allclose,
    check_array,
    check_lists,
    cprint,
    draw_choice,
    draw_indices,
    draw_sample,
    parametrize,
)


async def populate_stage(max_num_prims: int, operation: Literal["wrap", "create"], **kwargs: Any) -> None:
    """Populate stage.

    Args:
        max_num_prims: Maximum number of prims to create for a test case.
        operation: Stage population operation to use.
        **kwargs: Additional keyword arguments.
    """
    assert operation == "wrap", "Other operations except 'wrap' are not supported"
    # create new stage
    await stage_utils.create_new_stage_async()
    # define prims
    stage_utils.define_prim(f"/World", "Xform")
    stage_utils.define_prim(f"/World/PhysicsScene", "PhysicsScene")
    for i in range(max_num_prims):
        stage_utils.define_prim(f"/World/A_{i}", "Xform")
        stage_utils.define_prim(f"/World/A_{i}/B", "Cube")


class TestGeomPrim(omni.kit.test.AsyncTestCase):
    """Test geom prim."""

    async def setUp(self) -> None:
        """Method called to prepare the test fixture."""
        super().setUp()

    async def tearDown(self) -> None:
        """Method called immediately after the test method has been called."""
        super().tearDown()

    # --------------------------------------------------------------------

    def check_backend(self, backend: Any, rigid_prim: Any) -> None:
        """Check backend.

        Args:
            backend: Backend name under test.
            rigid_prim: Value passed by the caller.
        """

    # --------------------------------------------------------------------

    @parametrize(
        backends=["tensor"],  # "tensor" backend plays the simulation
        instances=["one"],
        operations=["wrap"],
        prim_class=lambda *args, **kwargs: None,
        populate_stage_func=populate_stage,
        max_num_prims=1,
    )
    async def test_runtime_instance_creation(self, prim: Any, num_prims: Any, device: Any, backend: Any) -> None:
        """Test runtime instance creation.

        Args:
            prim: Prim or prim wrapper under test.
            num_prims: Number of prims under test.
            device: Device under test.
            backend: Backend name under test.
        """
        GeomPrim("/World/A_0")

    @parametrize(backends=["usd"], operations=["wrap"], prim_class=GeomPrim, populate_stage_func=populate_stage)
    async def test_len(self, prim: Any, num_prims: Any, device: Any, backend: Any) -> None:
        """Test len.

        Args:
            prim: Prim or prim wrapper under test.
            num_prims: Number of prims under test.
            device: Device under test.
            backend: Backend name under test.
        """
        self.assertEqual(len(prim), num_prims, f"Invalid GeomPrim ({num_prims} prims) len")

    @parametrize(backends=["usd"], operations=["wrap"], prim_class=GeomPrim, populate_stage_func=populate_stage)
    async def test_enabled_collisions(self, prim: Any, num_prims: Any, device: Any, backend: Any) -> None:
        """Test enabled collisions.

        Args:
            prim: Prim or prim wrapper under test.
            num_prims: Number of prims under test.
            device: Device under test.
            backend: Backend name under test.
        """
        # check backend
        self.check_backend(backend, prim)
        # test cases
        for indices, expected_count in draw_indices(count=num_prims, step=2):
            cprint(f"  |    |-- indices: {type(indices).__name__}, expected_count: {expected_count}")
            for v0, expected_v0 in draw_sample(shape=(expected_count, 1), dtype=wp.bool):
                with use_backend(backend, raise_on_unsupported=True, raise_on_fallback=True):
                    prim.set_enabled_collisions(v0, indices=indices)
                    output = prim.get_enabled_collisions(indices=indices)
                check_array(output, shape=(expected_count, 1), dtype=wp.bool, device=device)
                check_allclose(expected_v0, output, given=(v0,))

    @parametrize(backends=["usd"], operations=["wrap"], prim_class=GeomPrim, populate_stage_func=populate_stage)
    async def test_collision_approximations(self, prim: Any, num_prims: Any, device: Any, backend: Any) -> None:
        """Test collision approximations.

        Args:
            prim: Prim or prim wrapper under test.
            num_prims: Number of prims under test.
            device: Device under test.
            backend: Backend name under test.
        """
        choices = [
            "none",
            "convexDecomposition",
            "convexHull",
            "boundingSphere",
            "boundingCube",
            "meshSimplification",
            "sdf",
            "sphereFill",
        ]
        # test cases
        # - check the collision approximations before applying any approximation
        with use_backend(backend, raise_on_unsupported=True, raise_on_fallback=True):
            approximations = prim.get_collision_approximations()
        check_lists(["none"] * num_prims, approximations)
        # - by indices
        for indices, expected_count in draw_indices(count=num_prims, step=2, types=[list, np.ndarray, wp.array]):
            cprint(f"  |    |-- indices: {type(indices).__name__}, expected_count: {expected_count}")
            count = expected_count
            for v0, expected_v0 in draw_choice(shape=(count,), choices=choices):
                with use_backend(backend, raise_on_unsupported=True, raise_on_fallback=True):
                    prim.set_collision_approximations(v0, indices=indices)
                    output = prim.get_collision_approximations(indices=indices)
                check_lists(expected_v0, output)
        # - all
        count = num_prims
        for v0, expected_v0 in draw_choice(shape=(count,), choices=choices):
            with use_backend(backend, raise_on_unsupported=True, raise_on_fallback=True):
                prim.set_collision_approximations(v0)
                output = prim.get_collision_approximations()
            check_lists(expected_v0, output)

    @parametrize(backends=["usd"], operations=["wrap"], prim_class=GeomPrim, populate_stage_func=populate_stage)
    async def test_offsets(self, prim: Any, num_prims: Any, device: Any, backend: Any) -> None:
        """Test offsets.

        Args:
            prim: Prim or prim wrapper under test.
            num_prims: Number of prims under test.
            device: Device under test.
            backend: Backend name under test.
        """
        # check backend
        self.check_backend(backend, prim)
        # test cases
        for indices, expected_count in draw_indices(count=num_prims, step=2):
            cprint(f"  |    |-- indices: {type(indices).__name__}, expected_count: {expected_count}")
            for (v0, expected_v0), (v1, expected_v1) in zip(
                draw_sample(shape=(expected_count, 1), dtype=wp.float32),
                draw_sample(shape=(expected_count, 1), dtype=wp.float32),
            ):
                with use_backend(backend, raise_on_unsupported=True, raise_on_fallback=True):
                    prim.set_offsets(v0, v1, indices=indices)
                    output = prim.get_offsets(indices=indices)
                check_array(output, shape=(expected_count, 1), dtype=wp.float32, device=device)
                check_allclose((expected_v0, expected_v1), output, given=(v0, v1))

    @parametrize(backends=["usd"], operations=["wrap"], prim_class=GeomPrim, populate_stage_func=populate_stage)
    async def test_torsional_patch_radii(self, prim: Any, num_prims: Any, device: Any, backend: Any) -> None:
        """Test torsional patch radii.

        Args:
            prim: Prim or prim wrapper under test.
            num_prims: Number of prims under test.
            device: Device under test.
            backend: Backend name under test.
        """
        # check backend
        self.check_backend(backend, prim)
        # test cases
        # - standard
        for indices, expected_count in draw_indices(count=num_prims, step=2):
            cprint(f"  |    |-- indices: {type(indices).__name__}, expected_count: {expected_count}")
            for v0, expected_v0 in draw_sample(shape=(expected_count, 1), dtype=wp.float32):
                with use_backend(backend, raise_on_unsupported=True, raise_on_fallback=True):
                    prim.set_torsional_patch_radii(v0, indices=indices)
                    output = prim.get_torsional_patch_radii(indices=indices)
                check_array(output, shape=(expected_count, 1), dtype=wp.float32, device=device)
                check_allclose(expected_v0, output, given=(v0,))
        # - minimum
        for indices, expected_count in draw_indices(count=num_prims, step=2):
            cprint(f"  |    |-- indices: {type(indices).__name__}, expected_count: {expected_count}")
            for v0, expected_v0 in draw_sample(shape=(expected_count, 1), dtype=wp.float32):
                with use_backend(backend, raise_on_unsupported=True, raise_on_fallback=True):
                    prim.set_torsional_patch_radii(v0, indices=indices, minimum=True)
                    output = prim.get_torsional_patch_radii(indices=indices, minimum=True)
                check_array(output, shape=(expected_count, 1), dtype=wp.float32, device=device)
                check_allclose(expected_v0, output, given=(v0,))

    @parametrize(backends=["usd"], operations=["wrap"], prim_class=GeomPrim, populate_stage_func=populate_stage)
    async def test_physics_materials(self, prim: Any, num_prims: Any, device: Any, backend: Any) -> None:
        """Test physics materials.

        Args:
            prim: Prim or prim wrapper under test.
            num_prims: Number of prims under test.
            device: Device under test.
            backend: Backend name under test.
        """
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
        with use_backend(backend, raise_on_unsupported=True, raise_on_fallback=True):
            output = prim.get_applied_physics_materials()
        number_of_materials = sum(1 for material in output if material is not None)
        assert number_of_materials == 0, f"No material should have been applied. Applied: {number_of_materials}"
        # - by indices
        for indices, expected_count in draw_indices(count=num_prims, step=2, types=[list, np.ndarray, wp.array]):
            cprint(f"  |    |-- indices: {type(indices).__name__}, expected_count: {expected_count}")
            count = expected_count
            for v0, expected_v0 in draw_choice(shape=(count,), choices=choices):
                with use_backend(backend, raise_on_unsupported=True, raise_on_fallback=True):
                    prim.apply_physics_materials(v0, indices=indices)
                    output = prim.get_applied_physics_materials(indices=indices)
                check_lists(expected_v0, output, predicate=lambda x: x.paths[0])
        # - check the number of applied materials after applying materials by indices
        with use_backend(backend, raise_on_unsupported=True, raise_on_fallback=True):
            output = prim.get_applied_physics_materials()
        number_of_materials = sum(1 for material in output if material is not None)
        assert (
            number_of_materials == count
        ), f"{count} materials should have been applied. Applied: {number_of_materials}"
        # - all
        count = num_prims
        for v0, expected_v0 in draw_choice(shape=(count,), choices=choices):
            with use_backend(backend, raise_on_unsupported=True, raise_on_fallback=True):
                prim.apply_physics_materials(v0)
                output = prim.get_applied_physics_materials()
            check_lists(expected_v0, output, predicate=lambda x: x.paths[0])
        # - check the number of applied materials after applying materials by all
        with use_backend(backend, raise_on_unsupported=True, raise_on_fallback=True):
            output = prim.get_applied_physics_materials()
        number_of_materials = sum(1 for material in output if material is not None)
        assert (
            number_of_materials == count
        ), f"{count} materials should have been applied. Applied: {number_of_materials}"
