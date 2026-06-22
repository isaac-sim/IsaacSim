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

"""Verifies RigidPrim runtime creation, rigid body state access, force application, and material properties. Covers poses, velocities, masses, gravity, centers of mass, inertias, densities, sleep thresholds, default state, and contact data tracking."""

from __future__ import annotations

from typing import Any, Literal

import isaacsim.core.experimental.utils.stage as stage_utils
import numpy as np
import omni.kit.app
import omni.kit.test
import warp as wp
from isaacsim.core.experimental.objects import Cube, GroundPlane
from isaacsim.core.experimental.prims import GeomPrim, RigidPrim, XformPrim
from isaacsim.core.experimental.utils.backend import use_backend
from isaacsim.core.simulation_manager import SimulationManager
from pxr import Gf, PhysxSchema, UsdGeom, UsdPhysics

from .common import check_allclose, check_array, cprint, draw_indices, draw_sample, parametrize


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
        xform_prim = stage_utils.define_prim(f"/World/A_{i}", "Xform")
        cube_prim = stage_utils.define_prim(f"/World/A_{i}/B", "Cube")
        # apply mass API with proper inertia to the xform
        UsdPhysics.RigidBodyAPI.Apply(xform_prim)
        mass_api = UsdPhysics.MassAPI.Apply(xform_prim)
        mass_api.GetMassAttr().Set(1.0)
        # set diagonal inertia tensor (for a 1m cube with mass 1kg)
        mass_api.GetDiagonalInertiaAttr().Set(Gf.Vec3f(0.167, 0.167, 0.167))
        mass_api.GetCenterOfMassAttr().Set(Gf.Vec3f(0.0, 0.0, 0.0))
        # apply collision API to the cube geometry
        UsdPhysics.CollisionAPI.Apply(cube_prim)


class TestRigidPrim(omni.kit.test.AsyncTestCase):
    """Test rigid prim."""

    async def setUp(self) -> None:
        """Method called to prepare the test fixture."""
        super().setUp()

    async def tearDown(self) -> None:
        """Method called immediately after the test method has been called."""
        super().tearDown()

    # --------------------------------------------------------------------

    def check_backend(self, backend: Any, prim: Any) -> None:
        """Check backend.

        Args:
            backend: Backend name under test.
            prim: Prim or prim wrapper under test.
        """
        if backend == "tensor":
            self.assertTrue(prim.is_physics_tensor_entity_valid(), f"Tensor API should be enabled ({backend})")
        elif backend in ["usd", "usdrt", "fabric"]:
            self.assertFalse(prim.is_physics_tensor_entity_valid(), f"Tensor API should be disabled ({backend})")
        else:
            raise ValueError(f"Invalid backend: {backend}")

    # --------------------------------------------------------------------

    @parametrize(
        devices=["cpu"],  # runtime instance creation is not currently supported on GPU
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
        RigidPrim("/World/A_0")

    @parametrize(
        backends=["tensor", "usd"],
        operations=["wrap"],
        prim_class=RigidPrim,
        prim_class_kwargs={"masses": [1.0]},
        populate_stage_func=populate_stage,
    )
    async def test_len(self, prim: Any, num_prims: Any, device: Any, backend: Any) -> None:
        """Test len.

        Args:
            prim: Prim or prim wrapper under test.
            num_prims: Number of prims under test.
            device: Device under test.
            backend: Backend name under test.
        """
        self.assertEqual(len(prim), num_prims, f"Invalid RigidPrim ({num_prims} prims) len")

    @parametrize(
        operations=["wrap"],
        prim_class=RigidPrim,
        prim_class_kwargs={"masses": [1.0]},
        populate_stage_func=populate_stage,
    )
    async def test_world_poses(self, prim: Any, num_prims: Any, device: Any, backend: Any) -> None:
        """Test world poses.

        Args:
            prim: Prim or prim wrapper under test.
            num_prims: Number of prims under test.
            device: Device under test.
            backend: Backend name under test.
        """
        # check backend and define USD usage
        self.check_backend(backend, prim)
        if backend in ["usdrt", "fabric"]:
            await omni.kit.app.get_app().next_update_async()
        elif backend == "usd":
            prim.reset_xform_op_properties()
        # test cases
        for indices, expected_count in draw_indices(count=num_prims, step=2):
            cprint(f"  |    |-- indices: {type(indices).__name__}, expected_count: {expected_count}")
            for (v0, expected_v0), (v1, expected_v1) in zip(
                draw_sample(shape=(expected_count, 3), dtype=wp.float32),
                draw_sample(shape=(expected_count, 4), dtype=wp.float32, normalized=True),
            ):
                with use_backend(backend, raise_on_unsupported=True, raise_on_fallback=True):
                    prim.set_world_poses(v0, v1, indices=indices)
                    output = prim.get_world_poses(indices=indices)
                check_array(output[0], shape=(expected_count, 3), dtype=wp.float32, device=device)
                check_array(output[1], shape=(expected_count, 4), dtype=wp.float32, device=device)
                check_allclose((expected_v0, expected_v1), output, given=(v0, v1))

    @parametrize(
        operations=["wrap"],
        prim_class=RigidPrim,
        prim_class_kwargs={"masses": [1.0]},
        populate_stage_func=populate_stage,
    )
    async def test_local_poses(self, prim: Any, num_prims: Any, device: Any, backend: Any) -> None:
        """Test local poses.

        Args:
            prim: Prim or prim wrapper under test.
            num_prims: Number of prims under test.
            device: Device under test.
            backend: Backend name under test.
        """
        # check backend and define USD usage
        self.check_backend(backend, prim)
        if backend in ["usdrt", "fabric"]:
            await omni.kit.app.get_app().next_update_async()
        elif backend == "usd":
            prim.reset_xform_op_properties()
        # test cases
        for indices, expected_count in draw_indices(count=num_prims, step=2):
            cprint(f"  |    |-- indices: {type(indices).__name__}, expected_count: {expected_count}")
            for (v0, expected_v0), (v1, expected_v1) in zip(
                draw_sample(shape=(expected_count, 3), dtype=wp.float32),
                draw_sample(shape=(expected_count, 4), dtype=wp.float32, normalized=True),
            ):
                with use_backend(backend, raise_on_unsupported=True, raise_on_fallback=True):
                    prim.set_local_poses(v0, v1, indices=indices)
                    output = prim.get_local_poses(indices=indices)
                check_array(output[0], shape=(expected_count, 3), dtype=wp.float32, device=device)
                check_array(output[1], shape=(expected_count, 4), dtype=wp.float32, device=device)
                check_allclose((expected_v0, expected_v1), output, given=(v0, v1))

    @parametrize(
        backends=["tensor", "usd"],
        operations=["wrap"],
        prim_class=RigidPrim,
        prim_class_kwargs={"masses": [1.0]},
        populate_stage_func=populate_stage,
    )
    async def test_velocities(self, prim: Any, num_prims: Any, device: Any, backend: Any) -> None:
        """Test velocities.

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
                draw_sample(shape=(expected_count, 3), dtype=wp.float32),
                draw_sample(shape=(expected_count, 3), dtype=wp.float32),
            ):
                with use_backend(backend, raise_on_unsupported=True, raise_on_fallback=True):
                    prim.set_velocities(v0, v1, indices=indices)
                    output = prim.get_velocities(indices=indices)
                check_array(output, shape=(expected_count, 3), dtype=wp.float32, device=device)
                check_allclose((expected_v0, expected_v1), output, given=(v0, v1))

    @parametrize(
        backends=["tensor", "usd"],
        operations=["wrap"],
        prim_class=RigidPrim,
        prim_class_kwargs={"masses": [1.0]},
        populate_stage_func=populate_stage,
    )
    async def test_masses(self, prim: Any, num_prims: Any, device: Any, backend: Any) -> None:
        """Test masses.

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
            for v0, expected_v0 in draw_sample(shape=(expected_count, 1), dtype=wp.float32):
                with use_backend(backend, raise_on_unsupported=True, raise_on_fallback=True):
                    prim.set_masses(v0, indices=indices)
                    output = prim.get_masses(indices=indices)
                    inverse_output = prim.get_masses(indices=indices, inverse=True)
                check_array((output, inverse_output), shape=(expected_count, 1), dtype=wp.float32, device=device)
                check_allclose(expected_v0, output, given=(v0,))
                expected_inverse = 1.0 / (output.numpy() + 1e-8)
                check_allclose(expected_inverse, inverse_output, given=(v0,))

    @parametrize(
        backends=["tensor", "usd"],
        operations=["wrap"],
        supported_engines=["physx"],
        prim_class=RigidPrim,
        prim_class_kwargs={"masses": [1.0]},
        populate_stage_func=populate_stage,
    )
    async def test_enabled_rigid_bodies(self, prim: Any, num_prims: Any, device: Any, backend: Any) -> None:
        """Test enabled rigid bodies.

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
                    prim.set_enabled_rigid_bodies(v0, indices=indices)
                    output = prim.get_enabled_rigid_bodies(indices=indices)
                check_array(output, shape=(expected_count, 1), dtype=wp.bool, device=device)
                check_allclose(expected_v0, output, given=(v0,))

    @parametrize(
        backends=["tensor", "usd"],
        operations=["wrap"],
        supported_engines=["physx"],
        prim_class=RigidPrim,
        prim_class_kwargs={"masses": [1.0]},
        populate_stage_func=populate_stage,
    )
    async def test_enabled_gravities(self, prim: Any, num_prims: Any, device: Any, backend: Any) -> None:
        """Test enabled gravities.

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
                    prim.set_enabled_gravities(v0, indices=indices)
                    output = prim.get_enabled_gravities(indices=indices)
                check_array(output, shape=(expected_count, 1), dtype=wp.bool, device=device)
                check_allclose(expected_v0, output, given=(v0,))

    @parametrize(
        backends=["tensor"],
        operations=["wrap"],
        prim_class=RigidPrim,
        prim_class_kwargs={"masses": [1.0]},
        populate_stage_func=populate_stage,
    )
    async def test_coms(self, prim: Any, num_prims: Any, device: Any, backend: Any) -> None:
        """Test coms.

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
                draw_sample(shape=(expected_count, 3), dtype=wp.float32),
                draw_sample(shape=(expected_count, 4), dtype=wp.float32, normalized=True),
            ):
                with use_backend(backend, raise_on_unsupported=True, raise_on_fallback=True):
                    prim.set_coms(v0, v1, indices=indices)
                    output = prim.get_coms(indices=indices)
                check_array(output[0], shape=(expected_count, 3), dtype=wp.float32, device=device)
                check_array(output[1], shape=(expected_count, 4), dtype=wp.float32, device=device)
                check_allclose((expected_v0, expected_v1), output, given=(v0, v1))

    @parametrize(
        backends=["tensor"],
        operations=["wrap"],
        prim_class=RigidPrim,
        prim_class_kwargs={"masses": [1.0]},
        populate_stage_func=populate_stage,
    )
    async def test_inertias(self, prim: Any, num_prims: Any, device: Any, backend: Any) -> Any:
        """Test inertias.

        Args:
            prim: Prim or prim wrapper under test.
            num_prims: Number of prims under test.
            device: Device under test.
            backend: Backend name under test.

        Returns:
            Requested value.
        """

        def _transform(x: Any) -> Any:  # transform to a diagonal inertia matrix
            x[:, [1, 2, 3, 5, 6, 7]] = 0.0
            return x

        # check backend
        self.check_backend(backend, prim)
        # test cases
        for indices, expected_count in draw_indices(count=num_prims, step=2):
            cprint(f"  |    |-- indices: {type(indices).__name__}, expected_count: {expected_count}")
            for v0, expected_v0 in draw_sample(shape=(expected_count, 9), dtype=wp.float32, transform=_transform):
                with use_backend(backend, raise_on_unsupported=True, raise_on_fallback=True):
                    prim.set_inertias(v0, indices=indices)
                    output = prim.get_inertias(indices=indices)
                    inverse_output = prim.get_inertias(indices=indices, inverse=True)
                check_array((output, inverse_output), shape=(expected_count, 9), dtype=wp.float32, device=device)
                check_allclose(expected_v0, output, given=(v0,))
                expected_inverse = np.linalg.inv(output.numpy().reshape((-1, 3, 3))).reshape((expected_count, 9))
                check_allclose(expected_inverse, inverse_output, given=(v0,))

    @parametrize(
        backends=["usd"],
        operations=["wrap"],
        prim_class=RigidPrim,
        prim_class_kwargs={"masses": [1.0]},
        populate_stage_func=populate_stage,
    )
    async def test_densities(self, prim: Any, num_prims: Any, device: Any, backend: Any) -> None:
        """Test densities.

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
            for v0, expected_v0 in draw_sample(shape=(expected_count, 1), dtype=wp.float32):
                with use_backend(backend, raise_on_unsupported=True, raise_on_fallback=True):
                    prim.set_densities(v0, indices=indices)
                    output = prim.get_densities(indices=indices)
                check_array(output, shape=(expected_count, 1), dtype=wp.float32, device=device)
                check_allclose(expected_v0, output, given=(v0,))

    @parametrize(
        backends=["usd"],
        operations=["wrap"],
        prim_class=RigidPrim,
        prim_class_kwargs={"masses": [1.0]},
        populate_stage_func=populate_stage,
    )
    async def test_sleep_thresholds(self, prim: Any, num_prims: Any, device: Any, backend: Any) -> None:
        """Test sleep thresholds.

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
            for v0, expected_v0 in draw_sample(shape=(expected_count, 1), dtype=wp.float32):
                with use_backend(backend, raise_on_unsupported=True, raise_on_fallback=True):
                    prim.set_sleep_thresholds(v0, indices=indices)
                    output = prim.get_sleep_thresholds(indices=indices)
                check_array(output, shape=(expected_count, 1), dtype=wp.float32, device=device)
                check_allclose(expected_v0, output, given=(v0,))

    @parametrize(
        backends=["tensor", "usd"],
        operations=["wrap"],
        prim_class=RigidPrim,
        prim_class_kwargs={"masses": [1.0]},
        populate_stage_func=populate_stage,
    )
    async def test_default_state(self, prim: Any, num_prims: Any, device: Any, backend: Any) -> None:
        """Test default state.

        Args:
            prim: Prim or prim wrapper under test.
            num_prims: Number of prims under test.
            device: Device under test.
            backend: Backend name under test.
        """
        # check backend
        self.check_backend(backend, prim)
        if backend == "usd":
            prim.reset_xform_op_properties()
        # test cases
        for indices, expected_count in draw_indices(count=num_prims, step=2):
            cprint(f"  |    |-- indices: {type(indices).__name__}, expected_count: {expected_count}")
            for (v0, expected_v0), (v1, expected_v1), (v2, expected_v2), (v3, expected_v3) in zip(
                draw_sample(shape=(expected_count, 3), dtype=wp.float32),
                draw_sample(shape=(expected_count, 4), dtype=wp.float32, normalized=True),
                draw_sample(shape=(expected_count, 3), dtype=wp.float32),
                draw_sample(shape=(expected_count, 3), dtype=wp.float32),
            ):
                with use_backend(backend, raise_on_unsupported=True, raise_on_fallback=True):
                    prim.set_default_state(v0, v1, v2, v3, indices=indices)
                    output = prim.get_default_state(indices=indices)
                    prim.reset_to_default_state()
                check_array(output[0], shape=(expected_count, 3), dtype=wp.float32, device=device)
                check_array(output[1], shape=(expected_count, 4), dtype=wp.float32, device=device)
                check_array(output[2:], shape=(expected_count, 3), dtype=wp.float32, device=device)
                check_allclose((expected_v0, expected_v1, expected_v2, expected_v3), output, given=(v0, v1, v2, v3))

    @parametrize(
        backends=["tensor"],
        operations=["wrap"],
        prim_class=RigidPrim,
        prim_class_kwargs={"masses": [1.0]},
        populate_stage_func=populate_stage,
    )
    async def test_apply_forces(self, prim: Any, num_prims: Any, device: Any, backend: Any) -> None:
        """Test apply forces.

        Args:
            prim: Prim or prim wrapper under test.
            num_prims: Number of prims under test.
            device: Device under test.
            backend: Backend name under test.
        """
        # check backend
        self.check_backend(backend, prim)

        # test cases (forces)
        for local_frame in [True, False]:
            # - all
            for indices, expected_count in draw_indices(count=num_prims, step=2):
                cprint(f"  |    |-- indices: {type(indices).__name__}, expected_count: {expected_count}")
                for v0, _ in draw_sample(shape=(expected_count, 3), dtype=wp.float32):
                    with use_backend(backend, raise_on_unsupported=True, raise_on_fallback=True):
                        prim.apply_forces(v0, indices=indices, local_frame=local_frame)

        # test cases (forces and torques at positions)
        for local_frame in [True, False]:
            # - all
            for indices, expected_count in draw_indices(count=num_prims, step=2):
                cprint(f"  |    |-- indices: {type(indices).__name__}, expected_count: {expected_count}")
                for (v0, _), (v1, _), (positions, _) in zip(
                    draw_sample(shape=(expected_count, 3), dtype=wp.float32),
                    draw_sample(shape=(expected_count, 3), dtype=wp.float32),
                    draw_sample(shape=(expected_count, 3), dtype=wp.float32),
                ):
                    with use_backend(backend, raise_on_unsupported=True, raise_on_fallback=True):
                        prim.apply_forces_and_torques_at_pos(
                            v0, v1, positions=positions, indices=indices, local_frame=local_frame
                        )


async def populate_stage_with_ground(max_num_prims: int, operation: Literal["wrap", "create"], **kwargs: Any) -> None:
    """Populate stage with ground plane for contact testing.

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
    # create ground plane
    GroundPlane("/World/GroundPlane", positions=[[0, 0, 0]])
    # create rigid bodies
    cube_paths = [f"/World/A_{i}" for i in range(max_num_prims)]
    Cube(cube_paths)
    XformPrim(cube_paths, reset_xform_op_properties=True).set_local_poses(
        translations=[[i * 3, 0, 2.0] for i in range(max_num_prims)]
    )
    GeomPrim(cube_paths, apply_collision_apis=True)
    rigid_prims = RigidPrim(cube_paths, masses=[1.0])
    if SimulationManager.get_active_physics_engine() == "physx":
        RigidPrim.ensure_api(rigid_prims.prims, PhysxSchema.PhysxContactReportAPI)
        stage = stage_utils.get_current_stage()
        filter_prim = stage.GetPrimAtPath("/World/GroundPlane/collisionPlane")
        if filter_prim.IsValid():
            GeomPrim.ensure_api([filter_prim], PhysxSchema.PhysxContactReportAPI)


def _assert_single_cube_contact_data(
    test_case: omni.kit.test.AsyncTestCase,
    *,
    prim: RigidPrim,
    cube_path: str,
    forces: wp.array,
    points: wp.array,
    normals: wp.array,
    distances: wp.array,
    pair_counts: wp.array,
    start_indices: wp.array,
    net_forces: wp.array | None = None,
    contact_force_matrix: wp.array | None = None,
    cube_index: int = 0,
    expected_contacts: int = 4,
) -> tuple[int, int]:
    check_array(forces, shape=(prim._max_contact_count, 1), dtype=wp.float32, device=prim._device)
    check_array(points, shape=(prim._max_contact_count, 3), dtype=wp.float32, device=prim._device)
    check_array(normals, shape=(prim._max_contact_count, 3), dtype=wp.float32, device=prim._device)
    check_array(distances, shape=(prim._max_contact_count, 1), dtype=wp.float32, device=prim._device)
    check_array(pair_counts, shape=(len(prim), prim.num_contact_filters), dtype=wp.uint32, device=prim._device)
    check_array(start_indices, shape=(len(prim), prim.num_contact_filters), dtype=wp.uint32, device=prim._device)

    pair_count = int(pair_counts.numpy()[cube_index, 0])
    start_index = int(start_indices.numpy()[cube_index, 0])
    test_case.assertEqual(
        pair_count,
        expected_contacts,
        f"Expected {expected_contacts} contact points for cube index {cube_index}",
    )
    forces_slice = forces.numpy()[start_index : start_index + pair_count]
    points_slice = points.numpy()[start_index : start_index + pair_count]
    normals_slice = normals.numpy()[start_index : start_index + pair_count]
    distances_slice = distances.numpy()[start_index : start_index + pair_count]
    test_case.assertTrue(np.isfinite(forces_slice).all(), "Expected finite contact forces")
    test_case.assertTrue(np.isfinite(points_slice).all(), "Expected finite contact points")
    test_case.assertTrue(np.isfinite(normals_slice).all(), "Expected finite contact normals")
    test_case.assertTrue(np.isfinite(distances_slice).all(), "Expected finite contact distances")
    test_case.assertTrue(np.any(np.abs(forces_slice) > 0.0), "Expected non-zero contact forces")
    test_case.assertTrue(np.any(np.linalg.norm(points_slice, axis=-1) > 0.0), "Expected non-zero contact points")
    test_case.assertTrue(np.any(np.linalg.norm(normals_slice, axis=-1) > 0.0), "Expected non-zero contact normals")
    dt = SimulationManager.get_physics_dt()
    total_force = float(np.sum(forces_slice) / dt)
    test_case.assertAlmostEqual(total_force, 9.81, delta=0.1, msg="Expected total contact force ~9.81N")
    test_case.assertTrue(np.all(normals_slice[:, 2] > 0.999), "Expected contact normals to point +Z")
    test_case.assertTrue(
        np.all(np.abs(normals_slice[:, :2]) < 0.0001),
        "Expected contact normals to be mostly aligned with +Z",
    )
    stage = stage_utils.get_current_stage()
    cube = UsdGeom.Cube(stage.GetPrimAtPath(cube_path))
    cube_size = float(cube.GetSizeAttr().Get())
    half_size = cube_size / 2.0
    positions, _ = prim.get_world_poses(indices=[cube_index])
    cube_center = positions.numpy()[0]
    expected_z = cube_center[2] - half_size
    test_case.assertTrue(
        np.allclose(points_slice[:, 2], expected_z, atol=1e-2),
        "Expected contact points to lie on cube bottom face",
    )
    test_case.assertTrue(
        np.all(np.abs(points_slice[:, 0] - cube_center[0]) <= half_size + 1e-2),
        "Expected contact point X to be within cube footprint",
    )
    test_case.assertTrue(
        np.all(np.abs(points_slice[:, 1] - cube_center[1]) <= half_size + 1e-2),
        "Expected contact point Y to be within cube footprint",
    )
    if net_forces is not None:
        check_array(net_forces, shape=(len(prim), 3), dtype=wp.float32, device=prim._device)
        test_case.assertTrue(np.isfinite(net_forces.numpy()).all(), "Expected finite net contact forces")
        test_case.assertTrue(
            np.any(np.linalg.norm(net_forces.numpy(), axis=-1) > 0.0),
            "Expected non-zero contact forces",
        )
        dt = SimulationManager.get_physics_dt()
        z_force = float(net_forces.numpy()[cube_index, 2] / dt)
        test_case.assertAlmostEqual(
            z_force,
            9.81,
            delta=0.1,
            msg="Expected net Z force ~9.81N for cube",
        )
    if contact_force_matrix is not None:
        check_array(
            contact_force_matrix,
            shape=(len(prim), prim.num_contact_filters, 3),
            dtype=wp.float32,
            device=prim._device,
        )
        test_case.assertTrue(
            np.isfinite(contact_force_matrix.numpy()).all(),
            "Expected finite contact force matrix values",
        )
        test_case.assertTrue(
            np.any(np.linalg.norm(contact_force_matrix.numpy()[cube_index], axis=-1) > 0.0),
            "Expected non-zero contact forces",
        )
    return pair_count, start_index


async def _wait_for_contact_data(
    get_data: Any,
    *,
    backend: str,
    max_steps: int = 120,
    settle_frames: int = 15,
    expected_total_contacts: int | None = None,
    expected_pair_count: int | None = None,
    cube_index: int = 0,
    counts_tuple_index: int = -2,
) -> tuple[wp.array, ...]:
    """Wait until contact data reports non-zero counts for the given sensor.

    Args:
        get_data: Value passed by the caller.
        backend: Backend name under test.
        max_steps: Value passed by the caller.
        settle_frames: Value passed by the caller.
        expected_total_contacts: Value passed by the caller.
        expected_pair_count: Value passed by the caller.
        cube_index: Index of the cube under test.
        counts_tuple_index: Value passed by the caller.

    Returns:
        Contact data tuple returned once the requested contact count is available.
    """
    data = None
    frames_after = None
    for _ in range(max_steps):
        await omni.kit.app.get_app().next_update_async()
        with use_backend(backend, raise_on_unsupported=True, raise_on_fallback=True):
            data = get_data()
        counts_arr = data[counts_tuple_index].numpy()
        counts_flat = counts_arr.flatten()
        total_contacts = int(np.sum(counts_flat))
        ready_total = (
            total_contacts > 0 if expected_total_contacts is None else total_contacts == expected_total_contacts
        )
        if counts_arr.ndim >= 2:
            sensor_count = int(counts_arr[cube_index, 0])
        else:
            sensor_count = int(counts_flat[cube_index])
        ready_pair = sensor_count > 0 if expected_pair_count is None else sensor_count == expected_pair_count
        if ready_total and ready_pair:
            frames_after = 0 if frames_after is None else frames_after + 1
            if frames_after >= settle_frames:
                break
    return data


class TestRigidPrimContactTracking(omni.kit.test.AsyncTestCase):
    """Test rigid prim contact tracking."""

    async def setUp(self) -> None:
        """Method called to prepare the test fixture."""
        super().setUp()

    async def tearDown(self) -> None:
        """Method called immediately after the test method has been called."""
        super().tearDown()

    def check_backend(self, backend: Any, prim: Any) -> None:
        """Check backend.

        Args:
            backend: Backend name under test.
            prim: Prim or prim wrapper under test.
        """
        if backend == "tensor":
            self.assertTrue(prim.is_physics_tensor_entity_valid(), f"Tensor API should be enabled ({backend})")
        elif backend in ["usd", "usdrt", "fabric"]:
            self.assertFalse(prim.is_physics_tensor_entity_valid(), f"Tensor API should be disabled ({backend})")
        else:
            raise ValueError(f"Invalid backend: {backend}")

    @parametrize(
        backends=["tensor"],
        operations=["wrap"],
        instances=["many"],
        prim_class=RigidPrim,
        prim_class_kwargs={
            "masses": [1.0],
            "contact_filter_paths": ["/World/GroundPlane/collisionPlane"],
            "max_contact_count": 25,
        },
        populate_stage_func=populate_stage_with_ground,
    )
    async def test_contact_tracking_many(self, prim: Any, num_prims: Any, device: Any, backend: Any) -> None:
        """Test contact tracking many.

        Args:
            prim: Prim or prim wrapper under test.
            num_prims: Number of prims under test.
            device: Device under test.
            backend: Backend name under test.
        """
        # check backend
        self.check_backend(backend, prim)
        # enabled contact tracking (usd backend)
        with use_backend("usd", raise_on_unsupported=True, raise_on_fallback=True):
            prim.set_sleep_thresholds([0.0])
            prim.set_enabled_contact_tracking([True])
            output = prim.get_enabled_contact_tracking()
        check_array(output, shape=(num_prims, 1), dtype=wp.bool, device=device)
        expected_contacts = 4 * num_prims
        forces, points, normals, distances, pair_counts, start_indices = await _wait_for_contact_data(
            lambda: prim.get_contact_force_data(),
            backend=backend,
            expected_total_contacts=expected_contacts,
            expected_pair_count=4,
        )
        with use_backend(backend, raise_on_unsupported=True, raise_on_fallback=True):
            net_forces = prim.get_net_contact_forces()
        total_contacts = int(np.sum(pair_counts.numpy()))
        self.assertEqual(total_contacts, expected_contacts, f"Expected {expected_contacts} total contacts (4 per cube)")
        self.assertTrue(np.all(pair_counts.numpy() == 4), "Expected 4 contact points per cube")
        # contact force matrix
        with use_backend(backend, raise_on_unsupported=True, raise_on_fallback=True):
            contact_force_matrix = prim.get_contact_force_matrix()
        for cube_index in range(num_prims):
            _assert_single_cube_contact_data(
                self,
                prim=prim,
                cube_path=f"/World/A_{cube_index}",
                forces=forces,
                points=points,
                normals=normals,
                distances=distances,
                pair_counts=pair_counts,
                start_indices=start_indices,
                net_forces=net_forces,
                contact_force_matrix=contact_force_matrix,
                cube_index=cube_index,
                expected_contacts=4,
            )
        # per-prim filter list
        cube_paths = [f"/World/A_{i}" for i in range(num_prims)]
        per_prim_filters = ["/World/GroundPlane/collisionPlane"] * num_prims
        prim_per_filter = RigidPrim(
            cube_paths,
            masses=[1.0],
            contact_filter_paths=per_prim_filters,
            max_contact_count=25,
        )
        await _wait_for_contact_data(
            lambda: prim_per_filter.get_contact_force_data(),
            backend=backend,
            expected_total_contacts=expected_contacts,
            expected_pair_count=4,
        )
        with use_backend(backend, raise_on_unsupported=True, raise_on_fallback=True):
            forces = prim_per_filter.get_contact_force_matrix()
        check_array(forces, shape=(num_prims, prim_per_filter.num_contact_filters, 3), dtype=wp.float32, device=device)
        self.assertTrue(np.isfinite(forces.numpy()).all(), "Expected finite contact force matrix values")
        if SimulationManager.get_active_physics_engine() != "physx":
            return
        # friction data
        await _wait_for_contact_data(
            lambda: prim.get_contact_force_data(),
            backend=backend,
            expected_total_contacts=expected_contacts,
            expected_pair_count=4,
        )
        with use_backend(backend, raise_on_unsupported=True, raise_on_fallback=True):
            forces, points, pair_counts, start_indices = prim.get_friction_data()
        check_array(forces, shape=(prim._max_contact_count, 3), dtype=wp.float32, device=device)
        check_array(points, shape=(prim._max_contact_count, 3), dtype=wp.float32, device=device)
        check_array(pair_counts, shape=(num_prims, prim.num_contact_filters), dtype=wp.uint32, device=device)
        check_array(start_indices, shape=(num_prims, prim.num_contact_filters), dtype=wp.uint32, device=device)
        self.assertGreater(int(np.sum(pair_counts.numpy())), 0, "Expected at least one contact pair")
        self.assertTrue(np.isfinite(forces.numpy()).all(), "Expected finite friction forces")
        self.assertTrue(np.isfinite(points.numpy()).all(), "Expected finite friction points")
        self.assertTrue(np.any(np.abs(forces.numpy()) > 0.0), "Expected non-zero friction forces")
        self.assertTrue(np.any(np.linalg.norm(points.numpy(), axis=-1) > 0.0), "Expected non-zero friction points")

    @parametrize(
        backends=["tensor"],
        operations=["wrap"],
        instances=["one"],
        prim_class=RigidPrim,
        prim_class_kwargs={
            "masses": [1.0],
            "contact_filter_paths": ["/World/GroundPlane/collisionPlane"],
            "max_contact_count": 4,
        },
        populate_stage_func=populate_stage_with_ground,
        max_num_prims=1,
    )
    async def test_contact_tracking_single(self, prim: Any, num_prims: Any, device: Any, backend: Any) -> None:
        """Test contact tracking single.

        Args:
            prim: Prim or prim wrapper under test.
            num_prims: Number of prims under test.
            device: Device under test.
            backend: Backend name under test.
        """
        # check backend
        self.check_backend(backend, prim)
        # enabled contact tracking (usd backend)
        with use_backend("usd", raise_on_unsupported=True, raise_on_fallback=True):
            prim.set_sleep_thresholds([0.0])
            prim.set_enabled_contact_tracking([True])
            output = prim.get_enabled_contact_tracking()
        check_array(output, shape=(num_prims, 1), dtype=wp.bool, device=device)
        # net contact forces
        await _wait_for_contact_data(
            lambda: prim.get_contact_force_data(),
            backend=backend,
            expected_pair_count=4,
        )
        with use_backend(backend, raise_on_unsupported=True, raise_on_fallback=True):
            net_forces = prim.get_net_contact_forces()
        # contact force matrix
        with use_backend(backend, raise_on_unsupported=True, raise_on_fallback=True):
            contact_force_matrix = prim.get_contact_force_matrix()
        # contact force data
        forces, points, normals, distances, pair_counts, start_indices = await _wait_for_contact_data(
            lambda: prim.get_contact_force_data(),
            backend=backend,
            expected_pair_count=4,
        )
        _assert_single_cube_contact_data(
            self,
            prim=prim,
            cube_path="/World/A_0",
            forces=forces,
            points=points,
            normals=normals,
            distances=distances,
            pair_counts=pair_counts,
            start_indices=start_indices,
            net_forces=net_forces,
            contact_force_matrix=contact_force_matrix,
            cube_index=0,
            expected_contacts=4,
        )
        if SimulationManager.get_active_physics_engine() != "physx":
            return
        # friction data
        await _wait_for_contact_data(
            lambda: prim.get_contact_force_data(),
            backend=backend,
            expected_pair_count=4,
        )
        with use_backend(backend, raise_on_unsupported=True, raise_on_fallback=True):
            forces, points, pair_counts, start_indices = prim.get_friction_data()
        check_array(forces, shape=(prim._max_contact_count, 3), dtype=wp.float32, device=device)
        check_array(points, shape=(prim._max_contact_count, 3), dtype=wp.float32, device=device)
        check_array(pair_counts, shape=(num_prims, prim.num_contact_filters), dtype=wp.uint32, device=device)
        check_array(start_indices, shape=(num_prims, prim.num_contact_filters), dtype=wp.uint32, device=device)
        self.assertGreater(int(np.sum(pair_counts.numpy())), 0, "Expected at least one contact pair")
        self.assertTrue(np.isfinite(forces.numpy()).all(), "Expected finite friction forces")
        self.assertTrue(np.isfinite(points.numpy()).all(), "Expected finite friction points")
        self.assertTrue(np.any(np.abs(forces.numpy()) > 0.0), "Expected non-zero friction forces")
        self.assertTrue(np.any(np.linalg.norm(points.numpy(), axis=-1) > 0.0), "Expected non-zero friction points")


def _assert_single_cube_raw_contact_data(
    test_case: omni.kit.test.AsyncTestCase,
    *,
    prim: RigidPrim,
    cube_path: str,
    forces_np: np.ndarray,
    points_np: np.ndarray,
    normals_np: np.ndarray,
    separations_np: np.ndarray,
    other_actor_ids_np: np.ndarray,
    start: int,
    count: int,
    cube_index: int = 0,
    backend: str = "tensor",
) -> None:
    """Strict assertions for one cube's raw contact data at equilibrium on a ground plane.

    Args:
        test_case: Test case instance used for assertions.
        prim: Prim or prim wrapper under test.
        cube_path: Path of the cube under test.
        forces_np: Contact force data to validate.
        points_np: Contact point data to validate.
        normals_np: Contact normal data to validate.
        separations_np: Contact separation data to validate.
        other_actor_ids_np: Other actor ID data to validate.
        start: Start index of the contact records to validate.
        count: Expected number of contact records.
        cube_index: Index of the cube under test.
        backend: Backend name under test.
    """
    slc = slice(start, start + count)
    forces_slc = forces_np[slc]
    points_slc = points_np[slc]
    normals_slc = normals_np[slc]
    separations_slc = separations_np[slc]
    ids_slc = other_actor_ids_np[slc]

    # Finite and non-zero
    test_case.assertTrue(np.isfinite(forces_slc).all(), "Expected finite contact forces")
    test_case.assertTrue(np.isfinite(points_slc).all(), "Expected finite contact points")
    test_case.assertTrue(np.isfinite(normals_slc).all(), "Expected finite contact normals")
    test_case.assertTrue(np.isfinite(separations_slc).all(), "Expected finite contact separations")
    test_case.assertTrue(np.any(np.abs(forces_slc) > 0.0), "Expected non-zero contact forces")

    # Normals should be strictly +Z (ground contact at equilibrium)
    test_case.assertTrue(
        np.all(normals_slc[:, 2] > 0.999),
        f"Expected normals to point +Z (min Z={normals_slc[:, 2].min():.4f})",
    )
    test_case.assertTrue(
        np.all(np.abs(normals_slc[:, :2]) < 0.01),
        "Expected normals X/Y components ~0",
    )

    # Net contact force should equal weight: sum(force_scalar * normal_vec) / dt == [0, 0, mg]
    dt = SimulationManager.get_physics_dt()
    force_vectors = forces_slc[:, np.newaxis] * normals_slc
    net_force = np.sum(force_vectors, axis=0) / dt
    test_case.assertAlmostEqual(
        float(net_force[2]),
        9.81,
        delta=0.1,
        msg=f"Cube {cube_index}: net Z force should be ~9.81N, got {net_force[2]:.4f}",
    )
    test_case.assertAlmostEqual(
        float(net_force[0]),
        0.0,
        delta=0.1,
        msg=f"Cube {cube_index}: net X force should be ~0, got {net_force[0]:.4f}",
    )
    test_case.assertAlmostEqual(
        float(net_force[1]),
        0.0,
        delta=0.1,
        msg=f"Cube {cube_index}: net Y force should be ~0, got {net_force[1]:.4f}",
    )

    # Contact points should lie on cube bottom face
    stage = stage_utils.get_current_stage()
    cube_usd = UsdGeom.Cube(stage.GetPrimAtPath(cube_path))
    cube_size = float(cube_usd.GetSizeAttr().Get())
    half_size = cube_size / 2.0
    positions, _ = prim.get_world_poses(indices=[cube_index])
    cube_center = positions.numpy()[0]
    expected_z = cube_center[2] - half_size
    test_case.assertTrue(
        np.allclose(points_slc[:, 2], expected_z, atol=1e-2),
        f"Contact points Z should be at cube bottom ({expected_z:.3f}), got {points_slc[:, 2]}",
    )
    test_case.assertTrue(
        np.all(np.abs(points_slc[:, 0] - cube_center[0]) <= half_size + 1e-2),
        "Contact points X should be within cube footprint",
    )
    test_case.assertTrue(
        np.all(np.abs(points_slc[:, 1] - cube_center[1]) <= half_size + 1e-2),
        "Contact points Y should be within cube footprint",
    )

    # Separations should be near zero at equilibrium
    test_case.assertTrue(
        np.all(np.abs(separations_slc) < 0.01),
        f"Expected separations near zero, got max |d|={np.abs(separations_slc).max():.4f}",
    )

    # Other actor IDs should be non-zero and resolve to the ground
    test_case.assertTrue(np.all(ids_slc > 0), "Expected non-zero other actor IDs for all contacts")
    ids_cpu = wp.array(ids_slc.astype(np.uint64), dtype=wp.uint64, device="cpu")
    with use_backend(backend, raise_on_unsupported=True, raise_on_fallback=True):
        paths = prim.get_actor_paths_from_ids(ids_cpu)
    has_ground = any("Ground" in p or "ground" in p or "world" in p for p in paths if p)
    test_case.assertTrue(has_ground, f"Cube {cube_index} should contact ground. Paths: {paths}")


class TestRigidPrimRawContactTracking(omni.kit.test.AsyncTestCase):
    """Test rigid prim raw contact tracking."""

    async def setUp(self) -> None:
        """Set up test environment."""
        super().setUp()

    async def tearDown(self) -> None:
        """Tear down test environment."""
        super().tearDown()

    @parametrize(
        backends=["tensor"],
        operations=["wrap"],
        instances=["many"],
        prim_class=RigidPrim,
        prim_class_kwargs={
            "masses": [1.0],
            "max_contact_count": 25,
        },
        populate_stage_func=populate_stage_with_ground,
    )
    async def test_raw_contact_data_many(self, prim: Any, num_prims: Any, device: Any, backend: Any) -> None:
        """Test raw contact data many.

        Args:
            prim: Prim or prim wrapper under test.
            num_prims: Number of prims under test.
            device: Device under test.
            backend: Backend name under test.
        """
        self.assertTrue(prim.is_physics_tensor_entity_valid(), f"Tensor API should be enabled ({backend})")
        with use_backend("usd", raise_on_unsupported=True, raise_on_fallback=True):
            prim.set_sleep_thresholds([0.0])
            prim.set_enabled_contact_tracking([True])

        data = await _wait_for_contact_data(
            lambda: prim.get_raw_contact_data(),
            backend=backend,
            counts_tuple_index=4,
        )
        forces, points, normals, separations, counts, start_indices, other_actor_ids = data

        counts_np = counts.numpy().flatten()
        start_indices_np = start_indices.numpy().flatten()
        forces_np = forces.numpy().flatten()
        points_np = points.numpy().reshape(-1, 3)
        normals_np = normals.numpy().reshape(-1, 3)
        separations_np = separations.numpy().flatten()
        other_actor_ids_np = other_actor_ids.numpy().flatten()

        # All cubes should have contacts
        self.assertTrue(np.all(counts_np > 0), f"All sensors should have contacts. Counts: {counts_np}")

        for cube_index in range(num_prims):
            start = int(start_indices_np[cube_index])
            count = int(counts_np[cube_index])
            self.assertGreater(count, 0, f"Cube {cube_index} should have contacts")
            _assert_single_cube_raw_contact_data(
                self,
                prim=prim,
                cube_path=f"/World/A_{cube_index}",
                forces_np=forces_np,
                points_np=points_np,
                normals_np=normals_np,
                separations_np=separations_np,
                other_actor_ids_np=other_actor_ids_np,
                start=start,
                count=count,
                cube_index=cube_index,
                backend=backend,
            )

    @parametrize(
        backends=["tensor"],
        operations=["wrap"],
        instances=["one"],
        prim_class=RigidPrim,
        prim_class_kwargs={
            "masses": [1.0],
            "max_contact_count": 25,
        },
        populate_stage_func=populate_stage_with_ground,
        max_num_prims=1,
    )
    async def test_raw_contact_data_single(self, prim: Any, num_prims: Any, device: Any, backend: Any) -> None:
        """Test raw contact data single.

        Args:
            prim: Prim or prim wrapper under test.
            num_prims: Number of prims under test.
            device: Device under test.
            backend: Backend name under test.
        """
        self.assertTrue(prim.is_physics_tensor_entity_valid(), f"Tensor API should be enabled ({backend})")
        with use_backend("usd", raise_on_unsupported=True, raise_on_fallback=True):
            prim.set_sleep_thresholds([0.0])
            prim.set_enabled_contact_tracking([True])

        data = await _wait_for_contact_data(
            lambda: prim.get_raw_contact_data(),
            backend=backend,
            counts_tuple_index=4,
        )
        forces, points, normals, separations, counts, start_indices, other_actor_ids = data

        # Shape checks
        check_array(forces, shape=(prim._max_contact_count, 1), dtype=wp.float32, device=device)
        check_array(points, shape=(prim._max_contact_count, 3), dtype=wp.float32, device=device)
        check_array(normals, shape=(prim._max_contact_count, 3), dtype=wp.float32, device=device)
        check_array(separations, shape=(prim._max_contact_count, 1), dtype=wp.float32, device=device)

        counts_np = counts.numpy().flatten()
        start_indices_np = start_indices.numpy().flatten()
        self.assertGreater(int(counts_np[0]), 0, f"Single cube should have contacts. Count: {counts_np[0]}")

        _assert_single_cube_raw_contact_data(
            self,
            prim=prim,
            cube_path="/World/A_0",
            forces_np=forces.numpy().flatten(),
            points_np=points.numpy().reshape(-1, 3),
            normals_np=normals.numpy().reshape(-1, 3),
            separations_np=separations.numpy().flatten(),
            other_actor_ids_np=other_actor_ids.numpy().flatten(),
            start=int(start_indices_np[0]),
            count=int(counts_np[0]),
            cube_index=0,
            backend=backend,
        )
