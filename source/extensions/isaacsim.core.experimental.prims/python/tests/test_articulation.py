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

"""Verifies Articulation wrappers resolve USD and PhysX articulation metadata and expose runtime tensor-backed properties. Covers poses, velocities, solver settings, link and DOF properties, tendons, default state, control mode switching, and fixed versus floating base metadata."""

from typing import Any, Literal
from unittest.mock import patch

import isaacsim.core.experimental.utils.prim as prim_utils
import isaacsim.core.experimental.utils.stage as stage_utils
import numpy as np
import omni.kit.app
import omni.kit.test
import warp as wp
from isaacsim.core.experimental.prims import Articulation
from isaacsim.core.experimental.prims.impl._usd_articulation import (
    _find_containing_articulation_root_path,
    _get_dof_type,
    _query_articulation_metadata_from_usd,
)
from isaacsim.core.experimental.utils.backend import use_backend
from isaacsim.core.simulation_manager import SimulationManager
from isaacsim.storage.native import get_assets_root_path
from pxr import Sdf, UsdGeom, UsdPhysics

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
    usd_path = f"{get_assets_root_path()}/{kwargs.get('usd_path', 'Isaac/Robots/IsaacSim/SimpleArticulation/simple_articulation.usd')}"
    for i in range(max_num_prims):
        stage_utils.add_reference_to_stage(usd_path=usd_path, path=f"/World/A_{i}")


async def play_stop_timeline() -> None:
    """Play stop timeline."""
    omni.timeline.get_timeline_interface().play()
    await omni.kit.app.get_app().next_update_async()
    omni.timeline.get_timeline_interface().stop()
    await omni.kit.app.get_app().next_update_async()


class TestArticulation(omni.kit.test.AsyncTestCase):
    """Test articulation."""

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

    async def test_usd_articulation_query_resolves_descendant_joint_target(self) -> Any:
        """Verify descendant joint targets resolve to the containing articulation.

        Returns:
            Requested value.
        """
        await stage_utils.create_new_stage_async()
        stage = stage_utils.get_current_stage(backend="usd")

        def define_rigid_body(path: str) -> Any:
            prim = UsdGeom.Xform.Define(stage, path).GetPrim()
            UsdPhysics.RigidBodyAPI.Apply(prim)
            return prim

        UsdGeom.Xform.Define(stage, "/World")
        UsdGeom.Xform.Define(stage, "/World/Robot")
        define_rigid_body("/World/Robot/base")
        define_rigid_body("/World/Robot/tool")
        define_rigid_body("/World/Robot/ee_link/robotiq_base_link")
        define_rigid_body("/World/Robot/ee_link/finger")

        root_joint = UsdPhysics.FixedJoint.Define(stage, "/World/Robot/root_joint")
        root_joint.CreateBody1Rel().SetTargets([Sdf.Path("/World/Robot/base")])
        UsdPhysics.ArticulationRootAPI.Apply(root_joint.GetPrim())

        wrist_joint = UsdPhysics.RevoluteJoint.Define(stage, "/World/Robot/wrist_joint")
        wrist_joint.CreateBody0Rel().SetTargets([Sdf.Path("/World/Robot/base")])
        wrist_joint.CreateBody1Rel().SetTargets([Sdf.Path("/World/Robot/tool")])
        UsdPhysics.DriveAPI.Apply(wrist_joint.GetPrim(), "angular").CreateTargetPositionAttr(0.0)

        fixed_joint = UsdPhysics.FixedJoint.Define(stage, "/World/Robot/ee_fixed_joint")
        fixed_joint.CreateBody0Rel().SetTargets([Sdf.Path("/World/Robot/tool")])
        fixed_joint.CreateBody1Rel().SetTargets([Sdf.Path("/World/Robot/ee_link/robotiq_base_link")])

        finger_joint = UsdPhysics.RevoluteJoint.Define(stage, "/World/Robot/ee_link/finger_joint")
        finger_joint.CreateBody0Rel().SetTargets([Sdf.Path("/World/Robot/ee_link/robotiq_base_link")])
        finger_joint.CreateBody1Rel().SetTargets([Sdf.Path("/World/Robot/ee_link/finger")])
        UsdPhysics.DriveAPI.Apply(finger_joint.GetPrim(), "angular").CreateTargetPositionAttr(0.0)

        target_path = "/World/Robot/ee_link/finger_joint"
        link_paths, joint_paths, dof_paths, dof_types = _query_articulation_metadata_from_usd(stage, target_path)

        self.assertEqual(_find_containing_articulation_root_path(stage, target_path), "/World/Robot/root_joint")
        self.assertIn("/World/Robot/ee_link/finger", link_paths)
        self.assertIn("/World/Robot/ee_link/finger_joint", joint_paths)
        self.assertIn("/World/Robot/ee_link/finger_joint", dof_paths)
        self.assertEqual(
            dof_types[dof_paths.index("/World/Robot/ee_link/finger_joint")], omni.physics.tensors.DofType.Rotation
        )

    async def test_usd_dof_type_query_matches_physx_generic_drive_order(self) -> Any:
        """Verify generic PhysX drive attributes keep deterministic DOF ordering.

        Returns:
            Requested value.
        """

        class FakeAttribute:
            def __init__(self, name: str) -> None:
                self._name = name

            def get_name(self) -> str:
                return self._name

            GetName = get_name

        class FakeJoint:
            def is_a(self, _schema: object) -> bool:
                return False

            IsA = is_a

            def get_attributes(self) -> list[FakeAttribute]:
                return [
                    FakeAttribute("drive:custom:targetPosition"),
                    FakeAttribute("drive:angular:targetPosition"),
                ]

            GetAttributes = get_attributes

        self.assertEqual(_get_dof_type(FakeJoint()), omni.physics.tensors.DofType.Invalid)

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
        Articulation("/World/A_0")

    @parametrize(
        backends=["tensor", "usd"], operations=["wrap"], prim_class=Articulation, populate_stage_func=populate_stage
    )
    async def test_len(self, prim: Any, num_prims: Any, device: Any, backend: Any) -> None:
        """Test len.

        Args:
            prim: Prim or prim wrapper under test.
            num_prims: Number of prims under test.
            device: Device under test.
            backend: Backend name under test.
        """
        self.assertEqual(len(prim), num_prims, f"Invalid Articulation ({num_prims} prims) len")

    @parametrize(
        backends=["tensor", "usd"], operations=["wrap"], prim_class=Articulation, populate_stage_func=populate_stage
    )
    async def test_properties_and_getters(self, prim: Any, num_prims: Any, device: Any, backend: Any) -> None:
        """Test properties and getters.

        Args:
            prim: Prim or prim wrapper under test.
            num_prims: Number of prims under test.
            device: Device under test.
            backend: Backend name under test.
        """
        # check backend
        self.check_backend(backend, prim)
        # test cases (properties)
        # - amount
        self.assertEqual(prim.num_dofs, 2, f"Invalid num_dofs")
        self.assertEqual(prim.num_joints, 2, f"Invalid num_joints")
        self.assertEqual(prim.num_links, 3, f"Invalid num_links")
        # - names (engine-agnostic: check presence, not order)
        self.assertEqual(sorted(prim.dof_names), sorted(["RevoluteJoint", "PrismaticJoint"]), f"Invalid dof_names")
        self.assertEqual(sorted(prim.joint_names), sorted(["RevoluteJoint", "PrismaticJoint"]), f"Invalid joint_names")
        self.assertEqual(sorted(prim.link_names), sorted(["CenterPivot", "Arm", "Slider"]), f"Invalid link_names")
        # - paths
        self.assertEqual(len(prim.dof_paths), len(prim), f"Invalid dof_paths")
        for i, dof_paths in enumerate(prim.dof_paths):
            for dof_path, dof_name in zip(dof_paths, prim.dof_names):
                self.assertTrue(dof_path.startswith(f"/World/A_{i}/"), f"Invalid dof_path: {dof_path}")
                self.assertTrue(dof_path.endswith(f"/{dof_name}"), f"Invalid dof_path: {dof_path}")
        self.assertEqual(len(prim.joint_paths), len(prim), f"Invalid joint_paths")
        for i, joint_paths in enumerate(prim.joint_paths):
            for joint_path, joint_name in zip(joint_paths, prim.joint_names):
                self.assertTrue(joint_path.startswith(f"/World/A_{i}/"), f"Invalid joint_path: {joint_path}")
                self.assertTrue(joint_path.endswith(f"/{joint_name}"), f"Invalid joint_path: {joint_path}")
        self.assertEqual(len(prim.link_paths), len(prim), f"Invalid link_paths")
        for i, link_paths in enumerate(prim.link_paths):
            for link_path, link_name in zip(link_paths, prim.link_names):
                self.assertTrue(link_path.startswith(f"/World/A_{i}/"), f"Invalid link_path: {link_path}")
                self.assertTrue(link_path.endswith(f"/{link_name}"), f"Invalid link_path: {link_path}")
        # - types (engine-agnostic: check presence, not order)
        self.assertEqual(
            sorted(prim.dof_types, key=lambda x: x.value),
            sorted(
                [omni.physics.tensors.DofType.Rotation, omni.physics.tensors.DofType.Translation], key=lambda x: x.value
            ),
            f"Invalid dof_types",
        )
        # test cases (getters) - engine-agnostic: check indices are valid and consistent
        revolute_idx = prim.get_dof_indices("RevoluteJoint").numpy().tolist()
        prismatic_idx = prim.get_dof_indices("PrismaticJoint").numpy().tolist()
        self.assertEqual(len(revolute_idx), 1, f"Should have one RevoluteJoint dof index")
        self.assertEqual(len(prismatic_idx), 1, f"Should have one PrismaticJoint dof index")
        self.assertIn(revolute_idx[0], [0, 1], f"RevoluteJoint index should be 0 or 1")
        self.assertIn(prismatic_idx[0], [0, 1], f"PrismaticJoint index should be 0 or 1")
        self.assertNotEqual(revolute_idx[0], prismatic_idx[0], f"Indices should be different")
        # Check multi-name query returns correct indices
        multi_idx = prim.get_dof_indices(["PrismaticJoint", "RevoluteJoint"]).numpy().tolist()
        self.assertEqual(multi_idx, [prismatic_idx[0], revolute_idx[0]], f"Invalid multi get_dof_indices")
        # - joint indices
        revolute_joint_idx = prim.get_joint_indices("RevoluteJoint").numpy().tolist()
        prismatic_joint_idx = prim.get_joint_indices("PrismaticJoint").numpy().tolist()
        self.assertEqual(len(revolute_joint_idx), 1, f"Should have one RevoluteJoint joint index")
        self.assertEqual(len(prismatic_joint_idx), 1, f"Should have one PrismaticJoint joint index")
        multi_joint_idx = prim.get_joint_indices(["PrismaticJoint", "RevoluteJoint"]).numpy().tolist()
        self.assertEqual(
            multi_joint_idx, [prismatic_joint_idx[0], revolute_joint_idx[0]], f"Invalid multi get_joint_indices"
        )
        # - link indices
        center_idx = prim.get_link_indices("CenterPivot").numpy().tolist()
        arm_idx = prim.get_link_indices("Arm").numpy().tolist()
        slider_idx = prim.get_link_indices("Slider").numpy().tolist()
        self.assertEqual(len(center_idx), 1, f"Should have one CenterPivot link index")
        self.assertEqual(len(arm_idx), 1, f"Should have one Arm link index")
        self.assertEqual(len(slider_idx), 1, f"Should have one Slider link index")
        self.assertEqual(len({center_idx[0], arm_idx[0], slider_idx[0]}), 3, f"Link indices should be unique")
        multi_link_idx = prim.get_link_indices(["Arm", "Slider", "CenterPivot"]).numpy().tolist()
        self.assertEqual(multi_link_idx, [arm_idx[0], slider_idx[0], center_idx[0]], f"Invalid multi get_link_indices")
        # test cases (Physics tensor initialization requirement for USD backend)
        if backend == "usd":
            await play_stop_timeline()  # ensure the articulation tensor API is initialized
            assert prim.is_physics_tensor_entity_initialized(), "Tensor API should be initialized"
        # - properties (engine-agnostic: check presence, not order)
        self.assertEqual(
            sorted(prim.joint_types, key=lambda x: x.value),
            sorted(
                [omni.physics.tensors.JointType.Revolute, omni.physics.tensors.JointType.Prismatic],
                key=lambda x: x.value,
            ),
            f"Invalid joint_types",
        )
        self.assertEqual(prim.num_shapes, 3, f"Invalid num_shapes")
        self.assertEqual(prim.num_fixed_tendons, 0, f"Invalid num_fixed_tendons")
        # - getters

    @parametrize(operations=["wrap"], prim_class=Articulation, populate_stage_func=populate_stage)
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

    @parametrize(backends=["tensor"], operations=["wrap"], prim_class=Articulation, populate_stage_func=populate_stage)
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

    @parametrize(backends=["usd"], operations=["wrap"], prim_class=Articulation, populate_stage_func=populate_stage)
    async def test_enabled_self_collisions(self, prim: Any, num_prims: Any, device: Any, backend: Any) -> None:
        """Test enabled self collisions.

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
                    prim.set_enabled_self_collisions(v0, indices=indices)
                    output = prim.get_enabled_self_collisions(indices=indices)
                check_array(output, shape=(expected_count, 1), dtype=wp.bool, device=device)
                check_allclose(expected_v0, output, given=(v0,))

    @parametrize(backends=["usd"], operations=["wrap"], prim_class=Articulation, populate_stage_func=populate_stage)
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

    @parametrize(backends=["usd"], operations=["wrap"], prim_class=Articulation, populate_stage_func=populate_stage)
    async def test_stabilization_thresholds(self, prim: Any, num_prims: Any, device: Any, backend: Any) -> None:
        """Test stabilization thresholds.

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
                    prim.set_stabilization_thresholds(v0, indices=indices)
                    output = prim.get_stabilization_thresholds(indices=indices)
                check_array(output, shape=(expected_count, 1), dtype=wp.float32, device=device)
                check_allclose(expected_v0, output, given=(v0,))

    @parametrize(backends=["usd"], operations=["wrap"], prim_class=Articulation, populate_stage_func=populate_stage)
    async def test_solver_iteration_counts(self, prim: Any, num_prims: Any, device: Any, backend: Any) -> None:
        """Test solver iteration counts.

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
                draw_sample(shape=(expected_count, 1), dtype=wp.int32, high=10),
                draw_sample(shape=(expected_count, 1), dtype=wp.int32, high=10),
            ):
                with use_backend(backend, raise_on_unsupported=True, raise_on_fallback=True):
                    prim.set_solver_iteration_counts(v0, v1, indices=indices)
                    output = prim.get_solver_iteration_counts(indices=indices)
                check_array(output, shape=(expected_count, 1), dtype=wp.int32, device=device)
                check_allclose((expected_v0, expected_v1), output, given=(v0, v1))

    @parametrize(
        backends=["tensor"],
        operations=["wrap"],
        supported_engines=["physx"],
        prim_class=Articulation,
        populate_stage_func=populate_stage,
    )
    async def test_jacobians_and_mass_matrices(self, prim: Any, num_prims: Any, device: Any, backend: Any) -> None:
        """Test jacobians and mass matrices.

        Args:
            prim: Prim or prim wrapper under test.
            num_prims: Number of prims under test.
            device: Device under test.
            backend: Backend name under test.
        """
        # check backend
        self.check_backend(backend, prim)
        # test cases (shapes)
        jacobian_matrix_shape = prim.jacobian_matrix_shape
        self.assertEqual(jacobian_matrix_shape, (2, 6, 2), f"Invalid Jacobian matrix shape ({jacobian_matrix_shape})")
        mass_matrix_shape = prim.mass_matrix_shape
        self.assertEqual(mass_matrix_shape, (2, 2), f"Invalid Mass matrix shape ({mass_matrix_shape})")
        # test cases (values)
        for indices, expected_count in draw_indices(count=num_prims, step=2):
            cprint(f"  |    |-- indices: {type(indices).__name__}, expected_count: {expected_count}")
            with use_backend(backend, raise_on_unsupported=True, raise_on_fallback=True):
                jacobian_matrices = prim.get_jacobian_matrices(indices=indices)
                mass_matrices = prim.get_mass_matrices(indices=indices)
            check_array(jacobian_matrices, shape=(expected_count, 2, 6, 2), dtype=wp.float32, device=device)
            check_array(mass_matrices, shape=(expected_count, 2, 2), dtype=wp.float32, device=device)

    @parametrize(
        backends=["tensor", "usd"], operations=["wrap"], prim_class=Articulation, populate_stage_func=populate_stage
    )
    async def test_link_masses(self, prim: Any, num_prims: Any, device: Any, backend: Any) -> None:
        """Test link masses.

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
            cprint(f"  |    |-- indices: {type(indices).__name__}, count: {expected_count}")
            for link_indices, expected_link_count in draw_indices(count=prim.num_links, step=2):
                cprint(f"  |    |    |-- link_indices: {type(link_indices).__name__}, count: {expected_link_count}")
                for v0, expected_v0 in draw_sample(shape=(expected_count, expected_link_count), dtype=wp.float32):
                    with use_backend(backend, raise_on_unsupported=True, raise_on_fallback=True):
                        prim.set_link_masses(v0, indices=indices, link_indices=link_indices)
                        output = prim.get_link_masses(indices=indices, link_indices=link_indices)
                        inverse_output = prim.get_link_masses(indices=indices, link_indices=link_indices, inverse=True)
                    check_array(
                        (output, inverse_output),
                        shape=(expected_count, expected_link_count),
                        dtype=wp.float32,
                        device=device,
                    )
                    check_allclose(expected_v0, output, given=(v0,))
                    expected_inverse = 1.0 / output.numpy()
                    check_allclose(expected_inverse, inverse_output, given=(v0,))

    @parametrize(backends=["tensor"], operations=["wrap"], prim_class=Articulation, populate_stage_func=populate_stage)
    async def test_link_inertias(self, prim: Any, num_prims: Any, device: Any, backend: Any) -> Any:
        """Test link inertias.

        Args:
            prim: Prim or prim wrapper under test.
            num_prims: Number of prims under test.
            device: Device under test.
            backend: Backend name under test.

        Returns:
            Requested value.
        """

        def _transform(x: Any) -> Any:  # transform to a diagonal inertia matrix
            x[:, :, [1, 2, 3, 5, 6, 7]] = 0.0
            return x

        # check backend
        self.check_backend(backend, prim)
        # test cases
        for indices, expected_count in draw_indices(count=num_prims, step=2):
            cprint(f"  |    |-- indices: {type(indices).__name__}, count: {expected_count}")
            for link_indices, expected_link_count in draw_indices(count=prim.num_links, step=2):
                cprint(f"  |    |    |-- link_indices: {type(link_indices).__name__}, count: {expected_link_count}")
                for v0, expected_v0 in draw_sample(
                    shape=(expected_count, expected_link_count, 9), dtype=wp.float32, transform=_transform
                ):
                    with use_backend(backend, raise_on_unsupported=True, raise_on_fallback=True):
                        prim.set_link_inertias(v0, indices=indices, link_indices=link_indices)
                        output = prim.get_link_inertias(indices=indices, link_indices=link_indices)
                        inverse_output = prim.get_link_inertias(
                            indices=indices, link_indices=link_indices, inverse=True
                        )
                    check_array(
                        (output, inverse_output),
                        shape=(expected_count, expected_link_count, 9),
                        dtype=wp.float32,
                        device=device,
                    )
                    check_allclose(expected_v0, output, given=(v0,))
                    expected_inverse = np.linalg.inv(output.numpy().reshape((-1, expected_link_count, 3, 3))).reshape(
                        (expected_count, expected_link_count, 9)
                    )
                    check_allclose(expected_inverse, inverse_output, given=(v0,))

    @parametrize(backends=["tensor"], operations=["wrap"], prim_class=Articulation, populate_stage_func=populate_stage)
    async def test_link_coms(self, prim: Any, num_prims: Any, device: Any, backend: Any) -> None:
        """Test link coms.

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
            for link_indices, expected_link_count in draw_indices(count=prim.num_links, step=2):
                cprint(f"  |    |    |-- link_indices: {type(link_indices).__name__}, count: {expected_link_count}")
                for (v0, expected_v0), (v1, expected_v1) in zip(
                    draw_sample(shape=(expected_count, expected_link_count, 3), dtype=wp.float32),
                    draw_sample(shape=(expected_count, expected_link_count, 4), dtype=wp.float32, normalized=True),
                ):
                    with use_backend(backend, raise_on_unsupported=True, raise_on_fallback=True):
                        prim.set_link_coms(v0, v1, indices=indices, link_indices=link_indices)
                        output = prim.get_link_coms(indices=indices, link_indices=link_indices)
                    check_array(
                        output[0], shape=(expected_count, expected_link_count, 3), dtype=wp.float32, device=device
                    )
                    check_array(
                        output[1], shape=(expected_count, expected_link_count, 4), dtype=wp.float32, device=device
                    )
                    check_allclose((expected_v0, expected_v1), output, given=(v0, v1))

    @parametrize(
        backends=["tensor"],
        operations=["wrap"],
        supported_engines=["physx"],
        prim_class=Articulation,
        populate_stage_func=populate_stage,
    )
    async def test_dof_compensation_forces(self, prim: Any, num_prims: Any, device: Any, backend: Any) -> None:
        """Test dof compensation forces.

        Args:
            prim: Prim or prim wrapper under test.
            num_prims: Number of prims under test.
            device: Device under test.
            backend: Backend name under test.
        """
        # check backend
        self.check_backend(backend, prim)
        # test cases (coriolis and centrifugal compensation forces)
        for indices, expected_count in draw_indices(count=num_prims, step=2):
            cprint(f"  |    |-- indices: {type(indices).__name__}, expected_count: {expected_count}")
            for dof_indices, expected_dof_count in draw_indices(count=prim.num_dofs, step=2):
                cprint(f"  |    |    |-- dof_indices: {type(dof_indices).__name__}, count: {expected_dof_count}")
                with use_backend(backend, raise_on_unsupported=True, raise_on_fallback=True):
                    forces = prim.get_dof_coriolis_and_centrifugal_compensation_forces(
                        indices=indices, dof_indices=dof_indices
                    )
                check_array(forces, shape=(expected_count, expected_dof_count), dtype=wp.float32, device=device)
        # test cases (gravity compensation forces)
        for indices, expected_count in draw_indices(count=num_prims, step=2):
            cprint(f"  |    |-- indices: {type(indices).__name__}, expected_count: {expected_count}")
            for dof_indices, expected_dof_count in draw_indices(count=prim.num_dofs, step=2):
                cprint(f"  |    |    |-- dof_indices: {type(dof_indices).__name__}, count: {expected_dof_count}")
                with use_backend(backend, raise_on_unsupported=True, raise_on_fallback=True):
                    forces = prim.get_dof_gravity_compensation_forces(indices=indices, dof_indices=dof_indices)
                check_array(forces, shape=(expected_count, expected_dof_count), dtype=wp.float32, device=device)

    @parametrize(
        backends=["tensor", "usd"],
        operations=["wrap"],
        supported_engines=["physx"],
        prim_class=Articulation,
        populate_stage_func=populate_stage,
    )
    async def test_link_enabled_gravities(self, prim: Any, num_prims: Any, device: Any, backend: Any) -> None:
        """Test link enabled gravities.

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
            for link_indices, expected_link_count in draw_indices(count=prim.num_links, step=2):
                cprint(f"  |    |    |-- link_indices: {type(link_indices).__name__}, count: {expected_link_count}")
                for v0, expected_v0 in draw_sample(shape=(expected_count, expected_link_count), dtype=wp.bool):
                    with use_backend(backend, raise_on_unsupported=True, raise_on_fallback=True):
                        prim.set_link_enabled_gravities(v0, indices=indices, link_indices=link_indices)
                        output = prim.get_link_enabled_gravities(indices=indices, link_indices=link_indices)
                    check_array(output, shape=(expected_count, expected_link_count), dtype=wp.bool, device=device)
                    check_allclose(expected_v0, output, given=(v0,))

    @parametrize(
        backends=["tensor", "usd"], operations=["wrap"], prim_class=Articulation, populate_stage_func=populate_stage
    )
    async def test_dof_armatures(self, prim: Any, num_prims: Any, device: Any, backend: Any) -> None:
        """Test dof armatures.

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
            for dof_indices, expected_dof_count in draw_indices(count=prim.num_dofs, step=2):
                cprint(f"  |    |    |-- dof_indices: {type(dof_indices).__name__}, count: {expected_dof_count}")
                for v0, expected_v0 in draw_sample(shape=(expected_count, expected_dof_count), dtype=wp.float32):
                    with use_backend(backend, raise_on_unsupported=True, raise_on_fallback=True):
                        prim.set_dof_armatures(v0, indices=indices, dof_indices=dof_indices)
                        output = prim.get_dof_armatures(indices=indices, dof_indices=dof_indices)
                    check_array(output, shape=(expected_count, expected_dof_count), dtype=wp.float32, device=device)
                    check_allclose(expected_v0, output, given=(v0,))

    @parametrize(
        backends=["tensor", "usd"], operations=["wrap"], prim_class=Articulation, populate_stage_func=populate_stage
    )
    async def test_dof_max_efforts(self, prim: Any, num_prims: Any, device: Any, backend: Any) -> None:
        """Test dof max efforts.

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
            for dof_indices, expected_dof_count in draw_indices(count=prim.num_dofs, step=2):
                cprint(f"  |    |    |-- dof_indices: {type(dof_indices).__name__}, count: {expected_dof_count}")
                for v0, expected_v0 in draw_sample(shape=(expected_count, expected_dof_count), dtype=wp.float32):
                    with use_backend(backend, raise_on_unsupported=True, raise_on_fallback=True):
                        prim.set_dof_max_efforts(v0, indices=indices, dof_indices=dof_indices)
                        output = prim.get_dof_max_efforts(indices=indices, dof_indices=dof_indices)
                    check_array(output, shape=(expected_count, expected_dof_count), dtype=wp.float32, device=device)
                    check_allclose(expected_v0, output, given=(v0,))

    @parametrize(
        backends=["tensor", "usd"], operations=["wrap"], prim_class=Articulation, populate_stage_func=populate_stage
    )
    async def test_dof_max_velocities(self, prim: Any, num_prims: Any, device: Any, backend: Any) -> None:
        """Test dof max velocities.

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
            for dof_indices, expected_dof_count in draw_indices(count=prim.num_dofs, step=2):
                cprint(f"  |    |    |-- dof_indices: {type(dof_indices).__name__}, count: {expected_dof_count}")
                for v0, expected_v0 in draw_sample(shape=(expected_count, expected_dof_count), dtype=wp.float32):
                    with use_backend(backend, raise_on_unsupported=True, raise_on_fallback=True):
                        prim.set_dof_max_velocities(v0, indices=indices, dof_indices=dof_indices)
                        output = prim.get_dof_max_velocities(indices=indices, dof_indices=dof_indices)
                    check_array(output, shape=(expected_count, expected_dof_count), dtype=wp.float32, device=device)
                    check_allclose(expected_v0, output, given=(v0,))

    @parametrize(
        backends=["tensor", "usd"], operations=["wrap"], prim_class=Articulation, populate_stage_func=populate_stage
    )
    async def test_dof_gains(self, prim: Any, num_prims: Any, device: Any, backend: Any) -> None:
        """Test dof gains.

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
            for dof_indices, expected_dof_count in draw_indices(count=prim.num_dofs, step=2):
                cprint(f"  |    |    |-- dof_indices: {type(dof_indices).__name__}, count: {expected_dof_count}")
                for (v0, expected_v0), (v1, expected_v1) in zip(
                    draw_sample(shape=(expected_count, expected_dof_count), dtype=wp.float32),
                    draw_sample(shape=(expected_count, expected_dof_count), dtype=wp.float32),
                ):
                    with use_backend(backend, raise_on_unsupported=True, raise_on_fallback=True):
                        prim.set_dof_gains(v0, v1, indices=indices, dof_indices=dof_indices)
                        output = prim.get_dof_gains(indices=indices, dof_indices=dof_indices)
                    check_array(output, shape=(expected_count, expected_dof_count), dtype=wp.float32, device=device)
                    check_allclose((expected_v0, expected_v1), output, given=(v0, v1))

    @parametrize(
        backends=["tensor", "usd"], operations=["wrap"], prim_class=Articulation, populate_stage_func=populate_stage
    )
    async def test_dof_targets(self, prim: Any, num_prims: Any, device: Any, backend: Any) -> None:
        """Test dof targets.

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
            for dof_indices, expected_dof_count in draw_indices(count=prim.num_dofs, step=2):
                cprint(f"  |    |    |-- dof_indices: {type(dof_indices).__name__}, count: {expected_dof_count}")
                # position targets
                for v0, expected_v0 in draw_sample(shape=(expected_count, expected_dof_count), dtype=wp.float32):
                    with use_backend(backend, raise_on_unsupported=True, raise_on_fallback=True):
                        prim.set_dof_position_targets(v0, indices=indices, dof_indices=dof_indices)
                        output = prim.get_dof_position_targets(indices=indices, dof_indices=dof_indices)
                    check_array(output, shape=(expected_count, expected_dof_count), dtype=wp.float32, device=device)
                    check_allclose(expected_v0, output, given=(v0,))
                # velocity targets
                for v0, expected_v0 in draw_sample(shape=(expected_count, expected_dof_count), dtype=wp.float32):
                    with use_backend(backend, raise_on_unsupported=True, raise_on_fallback=True):
                        prim.set_dof_velocity_targets(v0, indices=indices, dof_indices=dof_indices)
                        output = prim.get_dof_velocity_targets(indices=indices, dof_indices=dof_indices)
                    check_array(output, shape=(expected_count, expected_dof_count), dtype=wp.float32, device=device)
                    check_allclose(expected_v0, output, given=(v0,))

    @parametrize(
        backends=["tensor"],
        operations=["wrap"],
        supported_engines=["physx"],
        prim_class=Articulation,
        populate_stage_func=populate_stage,
    )
    async def test_dof_states(self, prim: Any, num_prims: Any, device: Any, backend: Any) -> None:
        """Test dof states.

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
            # DOF-related
            for dof_indices, expected_dof_count in draw_indices(count=prim.num_dofs, step=2):
                cprint(f"  |    |    |-- dof_indices: {type(dof_indices).__name__}, count: {expected_dof_count}")
                # positions
                for v0, expected_v0 in draw_sample(shape=(expected_count, expected_dof_count), dtype=wp.float32):
                    with use_backend(backend, raise_on_unsupported=True, raise_on_fallback=True):
                        prim.set_dof_positions(v0, indices=indices, dof_indices=dof_indices)
                        output = prim.get_dof_positions(indices=indices, dof_indices=dof_indices)
                    check_array(output, shape=(expected_count, expected_dof_count), dtype=wp.float32, device=device)
                    check_allclose(expected_v0, output, given=(v0,))
                    with use_backend(backend, raise_on_unsupported=True, raise_on_fallback=True):
                        output = prim.get_dof_position_targets(indices=indices, dof_indices=dof_indices)
                    check_array(output, shape=(expected_count, expected_dof_count), dtype=wp.float32, device=device)
                    check_allclose(expected_v0, output, given=(v0,))
                # velocities
                for v0, expected_v0 in draw_sample(shape=(expected_count, expected_dof_count), dtype=wp.float32):
                    with use_backend(backend, raise_on_unsupported=True, raise_on_fallback=True):
                        prim.set_dof_velocities(v0, indices=indices, dof_indices=dof_indices)
                        output = prim.get_dof_velocities(indices=indices, dof_indices=dof_indices)
                    check_array(output, shape=(expected_count, expected_dof_count), dtype=wp.float32, device=device)
                    check_allclose(expected_v0, output, given=(v0,))
                    with use_backend(backend, raise_on_unsupported=True, raise_on_fallback=True):
                        output = prim.get_dof_velocity_targets(indices=indices, dof_indices=dof_indices)
                    check_array(output, shape=(expected_count, expected_dof_count), dtype=wp.float32, device=device)
                    check_allclose(expected_v0, output, given=(v0,))
                # efforts
                for v0, expected_v0 in draw_sample(shape=(expected_count, expected_dof_count), dtype=wp.float32):
                    with use_backend(backend, raise_on_unsupported=True, raise_on_fallback=True):
                        prim.set_dof_efforts(v0, indices=indices, dof_indices=dof_indices)
                        output = prim.get_dof_efforts(indices=indices, dof_indices=dof_indices)
                    check_array(output, shape=(expected_count, expected_dof_count), dtype=wp.float32, device=device)
                    check_allclose(expected_v0, output, given=(v0,))
                # projected joint forces
                with use_backend(backend, raise_on_unsupported=True, raise_on_fallback=True):
                    output = prim.get_dof_projected_joint_forces(indices=indices, dof_indices=dof_indices)
                check_array(output, shape=(expected_count, expected_dof_count), dtype=wp.float32, device=device)
            # link-related
            for link_indices, expected_link_count in draw_indices(count=prim.num_links, step=2):
                cprint(f"  |    |    |-- link_indices: {type(link_indices).__name__}, count: {expected_link_count}")
                # measured forces
                with use_backend(backend, raise_on_unsupported=True, raise_on_fallback=True):
                    output = prim.get_link_incoming_joint_force(indices=indices, link_indices=link_indices)
                check_array(output, shape=(expected_count, expected_link_count, 3), dtype=wp.float32, device=device)

    @parametrize(
        backends=["tensor", "usd"],
        operations=["wrap"],
        supported_engines=["physx"],
        prim_class=Articulation,
        populate_stage_func=populate_stage,
    )
    async def test_dof_drive_types(self, prim: Any, num_prims: Any, device: Any, backend: Any) -> None:
        """Test dof drive types.

        Args:
            prim: Prim or prim wrapper under test.
            num_prims: Number of prims under test.
            device: Device under test.
            backend: Backend name under test.
        """
        # check backend
        self.check_backend(backend, prim)
        # test cases
        choices = ["acceleration", "force"]
        for indices, expected_count in draw_indices(count=num_prims, step=2):
            cprint(f"  |    |-- indices: {type(indices).__name__}, expected_count: {expected_count}")
            for dof_indices, expected_dof_count in draw_indices(count=prim.num_dofs, step=2):
                cprint(f"  |    |    |-- dof_indices: {type(dof_indices).__name__}, count: {expected_dof_count}")
                for v0, expected_v0 in draw_choice(shape=(expected_count, expected_dof_count), choices=choices):
                    with use_backend(backend, raise_on_unsupported=True, raise_on_fallback=True):
                        prim.set_dof_drive_types(v0, indices=indices, dof_indices=dof_indices)
                        output = prim.get_dof_drive_types(indices=indices, dof_indices=dof_indices)
                    check_lists(expected_v0, output)

    @parametrize(
        backends=["tensor", "usd"],
        operations=["wrap"],
        supported_engines=["physx"],
        prim_class=Articulation,
        populate_stage_func=populate_stage,
    )
    async def test_dof_limits(self, prim: Any, num_prims: Any, device: Any, backend: Any) -> None:
        """Test dof limits.

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
            for dof_indices, expected_dof_count in draw_indices(count=prim.num_dofs, step=2):
                cprint(f"  |    |    |-- dof_indices: {type(dof_indices).__name__}, count: {expected_dof_count}")
                for (v0, expected_v0), (v1, expected_v1) in zip(
                    draw_sample(shape=(expected_count, expected_dof_count), dtype=wp.float32, high=0.49),
                    draw_sample(shape=(expected_count, expected_dof_count), dtype=wp.float32, low=0.51),
                ):
                    with use_backend(backend, raise_on_unsupported=True, raise_on_fallback=True):
                        prim.set_dof_limits(v0, v1, indices=indices, dof_indices=dof_indices)
                        output = prim.get_dof_limits(indices=indices, dof_indices=dof_indices)
                    check_array(output, shape=(expected_count, expected_dof_count), dtype=wp.float32, device=device)
                    if backend == "usd":
                        check_allclose((expected_v0, expected_v1), output, given=(v0, v1))
                    else:
                        with self.assertRaises(
                            AssertionError, msg="This fails if the issue has been fixed. Update test!"
                        ):
                            check_allclose((expected_v0, expected_v1), output, given=(v0, v1))

    @parametrize(
        backends=["tensor", "usd"],
        operations=["wrap"],
        supported_engines=["physx"],
        prim_class=Articulation,
        populate_stage_func=populate_stage,
    )
    async def test_dof_friction_properties(self, prim: Any, num_prims: Any, device: Any, backend: Any) -> None:
        """Test dof friction properties.

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
            for dof_indices, expected_dof_count in draw_indices(count=prim.num_dofs, step=2):
                cprint(f"  |    |    |-- dof_indices: {type(dof_indices).__name__}, count: {expected_dof_count}")
                for (v0, expected_v0), (v1, expected_v1), (v2, expected_v2) in zip(
                    draw_sample(shape=(expected_count, expected_dof_count), dtype=wp.float32, low=0.51),
                    draw_sample(shape=(expected_count, expected_dof_count), dtype=wp.float32, high=0.49),
                    draw_sample(shape=(expected_count, expected_dof_count), dtype=wp.float32),
                ):
                    with use_backend(backend, raise_on_unsupported=True, raise_on_fallback=True):
                        prim.set_dof_friction_properties(v0, v1, v2, indices=indices, dof_indices=dof_indices)
                        output = prim.get_dof_friction_properties(indices=indices, dof_indices=dof_indices)
                    check_array(output, shape=(expected_count, expected_dof_count), dtype=wp.float32, device=device)
                    check_allclose((expected_v0, expected_v1, expected_v2), output, given=(v0, v1, v2))

    @parametrize(
        backends=["tensor", "usd"],
        operations=["wrap"],
        supported_engines=["physx"],
        prim_class=Articulation,
        populate_stage_func=populate_stage,
    )
    async def test_dof_drive_model_properties(self, prim: Any, num_prims: Any, device: Any, backend: Any) -> None:
        """Test dof drive model properties.

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
            for dof_indices, expected_dof_count in draw_indices(count=prim.num_dofs, step=2):
                cprint(f"  |    |    |-- dof_indices: {type(dof_indices).__name__}, count: {expected_dof_count}")
                for (v0, expected_v0), (v1, expected_v1), (v2, expected_v2) in zip(
                    draw_sample(shape=(expected_count, expected_dof_count), dtype=wp.float32),
                    draw_sample(shape=(expected_count, expected_dof_count), dtype=wp.float32),
                    draw_sample(shape=(expected_count, expected_dof_count), dtype=wp.float32),
                ):
                    # apply mask to DOFs that do not have the PhysxDrivePerformanceEnvelopeAPI applied (tensor API only)
                    _dof_indices = dof_indices.numpy() if isinstance(dof_indices, wp.array) else dof_indices
                    mask = [
                        i
                        for i, index in enumerate(range(prim.num_dofs) if _dof_indices is None else _dof_indices)
                        if not prim_utils.get_prim_at_path(prim.dof_paths[0][index]).HasAPI(
                            "PhysxDrivePerformanceEnvelopeAPI"
                        )
                    ]
                    if backend == "tensor":
                        expected_v0 = np.copy(expected_v0)
                        expected_v1 = np.copy(expected_v1)
                        expected_v2 = np.copy(expected_v2)
                        expected_v0[:, mask] = 0
                        expected_v1[:, mask] = 0
                        expected_v2[:, mask] = 0
                    # test
                    with use_backend(backend, raise_on_unsupported=True, raise_on_fallback=True):
                        prim.set_dof_drive_model_properties(v0, v1, v2, indices=indices, dof_indices=dof_indices)
                        output = prim.get_dof_drive_model_properties(indices=indices, dof_indices=dof_indices)
                    check_array(output, shape=(expected_count, expected_dof_count), dtype=wp.float32, device=device)
                    check_allclose((expected_v0, expected_v1, expected_v2), output, given=(v0, v1, v2))

    @parametrize(
        backends=["tensor"],
        operations=["wrap"],
        instances=["many"],
        supported_engines=["physx"],
        prim_class=Articulation,
        prim_class_kwargs={"positions": [[x, 0, 0] for x in range(5)], "reset_xform_op_properties": True},
        populate_stage_func=populate_stage,
        populate_stage_func_kwargs={"usd_path": "Isaac/Robots/ShadowRobot/ShadowHand/shadow_hand.usd"},
    )
    async def test_fixed_tendons_properties(self, prim: Any, num_prims: Any, device: Any, backend: Any) -> None:
        """Test fixed tendons properties.

        Args:
            prim: Prim or prim wrapper under test.
            num_prims: Number of prims under test.
            device: Device under test.
            backend: Backend name under test.
        """
        # check backend
        self.check_backend(backend, prim)
        # test cases
        assert prim.num_fixed_tendons == 4, f"Expected 4 fixed tendons, got {prim.num_fixed_tendons}"
        for indices, expected_count in draw_indices(count=num_prims, step=2):
            cprint(f"  |    |-- indices: {type(indices).__name__}, expected_count: {expected_count}")
            for tendon_indices, expected_tendon_count in draw_indices(count=prim.num_fixed_tendons, step=2):
                cprint(
                    f"  |    |    |-- tendon_indices: {type(tendon_indices).__name__}, count: {expected_tendon_count}"
                )
                for (
                    (v0, expected_v0),
                    (v1, expected_v1),
                    (v2, expected_v2),
                    (v3, expected_v3),
                    (v4, expected_v4),
                    (v5, expected_v5),
                    (v6, expected_v6),
                ) in zip(
                    draw_sample(shape=(expected_count, expected_tendon_count), dtype=wp.float32),
                    draw_sample(shape=(expected_count, expected_tendon_count), dtype=wp.float32),
                    draw_sample(shape=(expected_count, expected_tendon_count), dtype=wp.float32),
                    draw_sample(shape=(expected_count, expected_tendon_count), dtype=wp.float32, high=0.49),
                    draw_sample(shape=(expected_count, expected_tendon_count), dtype=wp.float32, low=0.51),
                    draw_sample(shape=(expected_count, expected_tendon_count), dtype=wp.float32),
                    draw_sample(shape=(expected_count, expected_tendon_count), dtype=wp.float32),
                ):
                    with use_backend(backend, raise_on_unsupported=True, raise_on_fallback=True):
                        prim.set_fixed_tendon_properties(
                            stiffnesses=v0,
                            dampings=v1,
                            limit_stiffnesses=v2,
                            lower_limits=v3,
                            upper_limits=v4,
                            rest_lengths=v5,
                            offsets=v6,
                            indices=indices,
                            tendon_indices=tendon_indices,
                        )
                        output_v0 = prim.get_fixed_tendon_stiffnesses(indices=indices, tendon_indices=tendon_indices)
                        output_v1 = prim.get_fixed_tendon_dampings(indices=indices, tendon_indices=tendon_indices)
                        output_v2 = prim.get_fixed_tendon_limit_stiffnesses(
                            indices=indices, tendon_indices=tendon_indices
                        )
                        output_v3, output_v4 = prim.get_fixed_tendon_limits(
                            indices=indices, tendon_indices=tendon_indices
                        )
                        output_v5 = prim.get_fixed_tendon_rest_lengths(indices=indices, tendon_indices=tendon_indices)
                        output_v6 = prim.get_fixed_tendon_offsets(indices=indices, tendon_indices=tendon_indices)
                    check_array(
                        (output_v0, output_v1, output_v2, output_v3, output_v4, output_v5, output_v6),
                        shape=(expected_count, expected_tendon_count),
                        dtype=wp.float32,
                        device=device,
                    )
                    check_allclose(
                        (expected_v0, expected_v1, expected_v2, expected_v3, expected_v4, expected_v5, expected_v6),
                        (output_v0, output_v1, output_v2, output_v3, output_v4, output_v5, output_v6),
                        given=(v0, v1, v2, v3, v4, v5, v6),
                    )

    @parametrize(
        backends=["tensor", "usd"], operations=["wrap"], prim_class=Articulation, populate_stage_func=populate_stage
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
            for dof_indices, expected_dof_count in draw_indices(count=prim.num_dofs, step=2):
                cprint(f"  |    |    |-- dof_indices: {type(dof_indices).__name__}, count: {expected_dof_count}")
                for (
                    (v0, expected_v0),
                    (v1, expected_v1),
                    (v2, expected_v2),
                    (v3, expected_v3),
                    (v4, expected_v4),
                    (v5, expected_v5),
                    (v6, expected_v6),
                ) in zip(
                    draw_sample(shape=(expected_count, 3), dtype=wp.float32),
                    draw_sample(shape=(expected_count, 4), dtype=wp.float32, normalized=True),
                    draw_sample(shape=(expected_count, 3), dtype=wp.float32),
                    draw_sample(shape=(expected_count, 3), dtype=wp.float32),
                    draw_sample(shape=(expected_count, expected_dof_count), dtype=wp.float32),
                    draw_sample(shape=(expected_count, expected_dof_count), dtype=wp.float32),
                    draw_sample(shape=(expected_count, expected_dof_count), dtype=wp.float32),
                ):
                    with use_backend(backend, raise_on_unsupported=True, raise_on_fallback=True):
                        prim.set_default_state(v0, v1, v2, v3, v4, v5, v6, indices=indices, dof_indices=dof_indices)
                        output = prim.get_default_state(indices=indices, dof_indices=dof_indices)
                        if backend == "tensor":
                            prim.reset_to_default_state()
                    check_array(output[0], shape=(expected_count, 3), dtype=wp.float32, device=device)
                    check_array(output[1], shape=(expected_count, 4), dtype=wp.float32, device=device)
                    check_array(output[2:3], shape=(expected_count, 3), dtype=wp.float32, device=device)
                    check_array(output[4:], shape=(expected_count, expected_dof_count), dtype=wp.float32, device=device)
                    check_allclose(
                        (expected_v0, expected_v1, expected_v2, expected_v3, expected_v4, expected_v5, expected_v6),
                        output,
                        given=(v0, v1, v2, v3, v4, v5, v6),
                    )

    @parametrize(operations=["wrap"], prim_class=Articulation, populate_stage_func=populate_stage)
    async def test_local_poses(self, prim: Any, num_prims: Any, device: Any, backend: Any) -> None:
        """Test local poses.

        Args:
            prim: Prim or prim wrapper under test.
            num_prims: Number of prims under test.
            device: Device under test.
            backend: Backend name under test.
        """
        # check backend
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
        backends=["tensor", "usd"], operations=["wrap"], prim_class=Articulation, populate_stage_func=populate_stage
    )
    async def test_switch_control_mode(self, prim: Any, num_prims: Any, device: Any, backend: Any) -> None:
        """Test switch control mode.

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
            for dof_indices, expected_dof_count in draw_indices(count=prim.num_dofs, step=2):
                cprint(f"  |    |    |-- dof_indices: {type(dof_indices).__name__}, count: {expected_dof_count}")
                for (v0, expected_v0), (v1, expected_v1) in zip(
                    draw_sample(shape=(expected_count, expected_dof_count), dtype=wp.float32),
                    draw_sample(shape=(expected_count, expected_dof_count), dtype=wp.float32),
                ):
                    with use_backend(backend, raise_on_unsupported=True, raise_on_fallback=True):
                        prim.set_dof_gains(v0, v1, indices=indices, dof_indices=dof_indices)
                        default_v0, default_v1 = prim.get_dof_gains()
                    for mode in ["position", "velocity", "effort"]:
                        with use_backend(backend, raise_on_unsupported=True, raise_on_fallback=True):
                            prim.switch_dof_control_mode(mode, indices=indices, dof_indices=dof_indices)
                            output = prim.get_dof_gains(indices=indices, dof_indices=dof_indices)
                        if mode == "position":
                            expected_output = (expected_v0, expected_v1)
                        elif mode == "velocity":
                            expected_output = (np.zeros_like(expected_v0), expected_v1)
                        elif mode == "effort":
                            expected_output = (np.zeros_like(expected_v0), np.zeros_like(expected_v1))
                        check_array(output, shape=(expected_count, expected_dof_count), dtype=wp.float32, device=device)
                        check_allclose(expected_output, output, given=(v0, v1))
                        # check that the default (internal) gains are not updated
                        check_allclose(
                            (default_v0, default_v1), (prim._default_dof_stiffnesses, prim._default_dof_dampings)
                        )

    @parametrize(
        backends=["usd"],
        operations=["wrap"],
        instances=["many"],
        prim_class=Articulation,
        populate_stage_func=populate_stage,
    )
    async def test_fetch_articulation_root_api_prim_paths(
        self, prim: Any, num_prims: Any, device: Any, backend: Any
    ) -> None:
        """Test fetch articulation root api prim paths.

        Args:
            prim: Prim or prim wrapper under test.
            num_prims: Number of prims under test.
            device: Device under test.
            backend: Backend name under test.
        """
        for backend in ["usd", "usdrt", "fabric"]:
            with use_backend(backend, raise_on_unsupported=True, raise_on_fallback=True):
                self.assertListEqual(Articulation.fetch_articulation_root_api_prim_paths("/World"), ["/World/A_0"])
                self.assertListEqual(
                    Articulation.fetch_articulation_root_api_prim_paths("/World/A_.*"),
                    ["/World/A_0", "/World/A_1", "/World/A_2", "/World/A_3", "/World/A_4"],
                )
                self.assertListEqual(
                    Articulation.fetch_articulation_root_api_prim_paths(["/World/A_0"]), ["/World/A_0"]
                )
                self.assertListEqual(
                    Articulation.fetch_articulation_root_api_prim_paths(["/", "/World/.*1", "/World/A_2/Arm"]),
                    ["/World/A_0", "/World/A_1", None],
                )
                if backend == "usd":
                    with patch(
                        "isaacsim.core.experimental.prims.impl.articulation.SimulationManager.get_active_physics_engine",
                        return_value="remotesim",
                    ):
                        self.assertListEqual(
                            Articulation.fetch_articulation_root_api_prim_paths(["/World/A_2/Arm"]),
                            ["/World/A_2"],
                        )

    async def _assert_articulation_metadata_consistent(self, asset_relative_path: str) -> None:
        """Assert link/joint/DOF metadata is non-empty pre-physics and matches post-physics.

        Args:
            asset_relative_path: Value passed by the caller.
        """
        await stage_utils.create_new_stage_async()
        usd_path = f"{get_assets_root_path()}/{asset_relative_path}"
        stage_utils.add_reference_to_stage(usd_path=usd_path, path="/Robot")
        articulation = Articulation("/Robot")
        engine = SimulationManager.get_active_physics_engine()
        self.assertIsNone(
            SimulationManager.get_physics_simulation_view(),
            "Precondition: no physics simulation view should exist before the test reads metadata.",
        )

        pre_link_names = list(articulation.link_names)
        pre_joint_names = list(articulation.joint_names)
        pre_dof_names = list(articulation.dof_names)
        self.assertGreater(len(pre_link_names), 0, f"Pre-physics link_names empty under engine '{engine}'.")
        self.assertGreater(len(pre_joint_names), 0, f"Pre-physics joint_names empty under engine '{engine}'.")
        self.assertGreater(len(pre_dof_names), 0, f"Pre-physics dof_names empty under engine '{engine}'.")

        pre_link_indices = articulation.get_link_indices(names=pre_link_names).numpy().tolist()
        pre_joint_indices = articulation.get_joint_indices(names=pre_joint_names).numpy().tolist()
        pre_dof_indices = articulation.get_dof_indices(names=pre_dof_names).numpy().tolist()
        self.assertListEqual(pre_link_indices, list(range(len(pre_link_names))))
        self.assertListEqual(pre_joint_indices, list(range(len(pre_joint_names))))
        self.assertListEqual(pre_dof_indices, list(range(len(pre_dof_names))))

        SimulationManager.initialize_physics()
        for _ in range(10):
            await omni.kit.app.get_app().next_update_async()

        post_link_names = list(articulation.link_names)
        post_joint_names = list(articulation.joint_names)
        post_dof_names = list(articulation.dof_names)
        self.assertEqual(
            pre_link_names,
            post_link_names,
            f"link_names order changed after physics initialization under engine '{engine}'.",
        )
        self.assertEqual(
            pre_joint_names,
            post_joint_names,
            f"joint_names order changed after physics initialization under engine '{engine}'.",
        )
        self.assertEqual(
            pre_dof_names,
            post_dof_names,
            f"dof_names order changed after physics initialization under engine '{engine}'.",
        )

        post_link_indices = articulation.get_link_indices(names=post_link_names).numpy().tolist()
        post_joint_indices = articulation.get_joint_indices(names=post_joint_names).numpy().tolist()
        post_dof_indices = articulation.get_dof_indices(names=post_dof_names).numpy().tolist()
        self.assertListEqual(pre_link_indices, post_link_indices)
        self.assertListEqual(pre_joint_indices, post_joint_indices)
        self.assertListEqual(pre_dof_indices, post_dof_indices)

    async def test_articulation_metadata_consistent_fixed_base(self) -> None:
        """Fixed-base articulation (Franka): pre-physics metadata must match post-physics."""
        await self._assert_articulation_metadata_consistent("Isaac/Robots/FrankaRobotics/FrankaPanda/franka.usd")

    async def test_articulation_metadata_consistent_floating_base(self) -> None:
        """Floating-base articulation (Spot quadruped): pre-physics metadata must match post-physics."""
        await self._assert_articulation_metadata_consistent("Isaac/Robots/BostonDynamics/spot/spot.usd")
