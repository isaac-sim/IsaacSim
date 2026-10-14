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

import copy
from typing import Any, Literal
from unittest.mock import PropertyMock, patch

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
from pxr import Gf, Sdf, UsdGeom, UsdPhysics

from .common import (
    check_allclose,
    check_array,
    check_lists,
    cprint,
    draw_choice,
    draw_indices,
    draw_sample,
    parametrize,
    requires_engines,
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


async def populate_stage_with_simple_articulations(count: int = 2) -> None:
    """Create local two-link articulations for backend-neutral tensor tests."""
    await stage_utils.create_new_stage_async()
    stage = stage_utils.get_current_stage(backend="usd")

    for i in range(count):
        articulation_path = f"/World/A_{i}"
        articulation = UsdGeom.Xform.Define(stage, articulation_path)
        articulation.AddTranslateOp().Set((float(i) * 3.0, 0.0, 0.0))
        UsdPhysics.ArticulationRootAPI.Apply(articulation.GetPrim())

        root_path = f"{articulation_path}/root"
        root = UsdGeom.Cube.Define(stage, root_path)
        UsdPhysics.RigidBodyAPI.Apply(root.GetPrim())
        UsdPhysics.MassAPI.Apply(root.GetPrim()).CreateMassAttr(1.0)
        root_joint = UsdPhysics.FixedJoint.Define(stage, f"{root_path}/root_joint")
        root_joint.CreateBody1Rel().SetTargets([root_path])

        child_path = f"{articulation_path}/child"
        child = UsdGeom.Cube.Define(stage, child_path)
        child.AddTranslateOp().Set((1.0, 0.0, 0.0))
        UsdPhysics.RigidBodyAPI.Apply(child.GetPrim())
        UsdPhysics.MassAPI.Apply(child.GetPrim()).CreateMassAttr(1.0)
        child_joint = UsdPhysics.RevoluteJoint.Define(stage, f"{child_path}/joint")
        child_joint.CreateBody0Rel().SetTargets([root_path])
        child_joint.CreateBody1Rel().SetTargets([child_path])
        child_joint.CreateAxisAttr().Set("Y")


async def populate_compensation_pendulum(max_num_prims: int, operation: str, **kwargs: Any) -> None:
    """Author unit-mass, unit-length links with revolute Y joints and known analytical dynamics.

    Args:
        max_num_prims: Number of independent pendulums.
        operation: Fixture operation, always wrap.
        **kwargs: Unused fixture options.
    """
    await stage_utils.create_new_stage_async()
    stage = stage_utils.get_current_stage(backend="usd")
    UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
    UsdGeom.SetStageMetersPerUnit(stage, 1.0)
    scene = UsdPhysics.Scene.Define(stage, "/World/PhysicsScene")
    scene.CreateGravityDirectionAttr(Gf.Vec3f(0, 0, -1))
    scene.CreateGravityMagnitudeAttr(9.81)
    for i in range(max_num_prims):
        root = f"/World/A_{i}"
        UsdGeom.Xform.Define(stage, root)
        for name, x in (("base", 0.0), ("link1", 0.5), ("link2", 1.5)):
            body = UsdGeom.Xform.Define(stage, f"{root}/{name}")
            body.AddTranslateOp().Set(Gf.Vec3d(x, i * 3, 0))
            UsdPhysics.RigidBodyAPI.Apply(body.GetPrim())
            mass = UsdPhysics.MassAPI.Apply(body.GetPrim())
            mass.CreateMassAttr(1.0)
            mass.CreateCenterOfMassAttr(Gf.Vec3f(0))
            mass.CreateDiagonalInertiaAttr(Gf.Vec3f(0.01, 0.1, 0.1))
        fixed = UsdPhysics.FixedJoint.Define(stage, f"{root}/fixed")
        fixed.CreateBody1Rel().SetTargets([f"{root}/base"])
        fixed.CreateLocalPos0Attr(Gf.Vec3f(0, i * 3, 0))
        UsdPhysics.ArticulationRootAPI.Apply(fixed.GetPrim())
        for j, parent in ((1, "base"), (2, "link1")):
            joint = UsdPhysics.RevoluteJoint.Define(stage, f"{root}/joint{j}")
            joint.CreateBody0Rel().SetTargets([f"{root}/{parent}"])
            joint.CreateBody1Rel().SetTargets([f"{root}/link{j}"])
            joint.CreateAxisAttr("Y")
            joint.CreateLocalPos0Attr(Gf.Vec3f(0 if j == 1 else 0.5, 0, 0))
            joint.CreateLocalPos1Attr(Gf.Vec3f(-0.5, 0, 0))


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

    @requires_engines(supported_engines=["physx", "newton"])
    async def test_link_incoming_joint_force_tensor_backend(self) -> None:
        """Test the public incoming-joint-force API with each tensor physics engine."""
        restore_config = None
        if SimulationManager.get_active_physics_engine() == "newton":
            from isaacsim.physics.newton import NewtonConfig, configure_newton, get_newton_config

            current_config = get_newton_config()
            restore_config = copy.deepcopy(current_config) if current_config else NewtonConfig()
            config = copy.deepcopy(restore_config)
            config.solver_outputs.link_incoming_joint_force = True
            configure_newton(config)

        timeline = omni.timeline.get_timeline_interface()
        try:
            await populate_stage_with_simple_articulations()
            SimulationManager.set_physics_sim_device("cpu")
            prim = Articulation("/World/A_.*")
            timeline.play()
            await omni.kit.app.get_app().next_update_async()

            for indices, expected_count in draw_indices(count=2, step=2):
                for link_indices, expected_link_count in draw_indices(count=prim.num_links, step=2):
                    with use_backend("tensor", raise_on_unsupported=True, raise_on_fallback=True):
                        forces, torques = prim.get_link_incoming_joint_force(indices=indices, link_indices=link_indices)
                    check_array(
                        [forces, torques],
                        shape=(expected_count, expected_link_count, 3),
                        dtype=wp.float32,
                        device="cpu",
                    )
                    self.assertTrue(np.isfinite(forces.numpy()).all())
                    self.assertTrue(np.isfinite(torques.numpy()).all())
        finally:
            timeline.stop()
            await omni.kit.app.get_app().next_update_async()
            if restore_config is not None:
                configure_newton(restore_config)

    @requires_engines(supported_engines=["physx", "newton"])
    async def test_dof_projected_joint_force_tensor_backend(self) -> None:
        """Test the public projected-DOF-force API with each tensor physics engine."""
        restore_config = None
        if SimulationManager.get_active_physics_engine() == "newton":
            from isaacsim.physics.newton import NewtonConfig, configure_newton, get_newton_config

            current_config = get_newton_config()
            restore_config = copy.deepcopy(current_config) if current_config else NewtonConfig()
            config = copy.deepcopy(restore_config)
            config.solver_outputs.link_incoming_joint_force = True
            configure_newton(config)

        timeline = omni.timeline.get_timeline_interface()
        try:
            await populate_stage_with_simple_articulations()
            SimulationManager.set_physics_sim_device("cpu")
            prim = Articulation("/World/A_.*")
            timeline.play()
            await omni.kit.app.get_app().next_update_async()

            for indices, expected_count in draw_indices(count=2, step=2):
                for dof_indices, expected_dof_count in draw_indices(count=prim.num_dofs, step=2):
                    with use_backend("tensor", raise_on_unsupported=True, raise_on_fallback=True):
                        output = prim.get_dof_projected_joint_forces(indices=indices, dof_indices=dof_indices)
                    check_array(
                        output,
                        shape=(expected_count, expected_dof_count),
                        dtype=wp.float32,
                        device="cpu",
                    )
                    self.assertTrue(np.isfinite(output.numpy()).all())
        finally:
            timeline.stop()
            await omni.kit.app.get_app().next_update_async()
            if restore_config is not None:
                configure_newton(restore_config)

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
        supported_engines=["physx", "newton"],
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
                for v0, expected_v0 in draw_sample(
                    shape=(expected_count, expected_link_count), dtype=wp.float32, low=0.1
                ):
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
        supported_engines=["physx", "newton"],
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
        view = prim._physics_articulation_view
        for getter, raw_getter in (
            (prim.get_dof_gravity_compensation_forces, view.get_gravity_compensation_forces),
            (
                prim.get_dof_coriolis_and_centrifugal_compensation_forces,
                view.get_coriolis_and_centrifugal_compensation_forces,
            ),
        ):
            expected = raw_getter().numpy().copy()
            self.assertEqual(expected.shape, (num_prims, prim.num_dofs))
            self.assertTrue(np.isfinite(expected).all())
            np.testing.assert_array_equal(getter().numpy(), expected)
            np.testing.assert_array_equal(getter(dof_indices=list(range(prim.num_dofs))).numpy(), expected)
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
        backends=["tensor"],
        operations=["wrap"],
        supported_engines=["physx", "newton"],
        prim_class=Articulation,
        populate_stage_func=populate_stage,
        populate_stage_func_kwargs={"usd_path": "Isaac/Robots_Multiphysics/BostonDynamics/spot/spot.usda"},
        max_num_prims=2,
    )
    async def test_floating_compensation_forces(self, prim: Any, num_prims: Any, device: Any, backend: Any) -> None:
        """Verify floating-base forces retain the root wrench followed by all joint forces.

        Args:
            prim: Articulation wrapper under test.
            num_prims: Number of wrapped articulations.
            device: Wrapper output device.
            backend: Backend under test.
        """
        self.check_backend(backend, prim)
        prim.set_dof_velocities(np.linspace(0.3, 2.0, prim.num_dofs, dtype=np.float32))
        view = prim._physics_articulation_view
        for getter, raw_getter in (
            (prim.get_dof_gravity_compensation_forces, view.get_gravity_compensation_forces),
            (
                prim.get_dof_coriolis_and_centrifugal_compensation_forces,
                view.get_coriolis_and_centrifugal_compensation_forces,
            ),
        ):
            raw = raw_getter().numpy().copy()
            self.assertEqual(raw.shape, (num_prims, prim.num_dofs + 6))
            expected = raw
            if getter.__name__ == "get_dof_gravity_compensation_forces":
                total_mass = prim.get_link_masses().numpy().reshape(num_prims, -1).sum(axis=1)
                np.testing.assert_allclose(raw[:, 2], total_mass * 9.81, rtol=1e-4)
                np.testing.assert_allclose(raw[:, :2], 0.0, atol=1e-4)
            self.assertTrue(np.isfinite(raw).all())
            self.assertGreater(np.max(np.abs(expected[:, 6:])), 1e-6, "Force oracle must be nonzero")
            for dof_indices in (
                None,
                list(range(6 + prim.num_dofs)),
                list(range(6)),
                list(range(6, 6 + prim.num_dofs)),
                [5 + prim.num_dofs, 0, 6, 0],
                0,
            ):
                columns = np.arange(6 + prim.num_dofs) if dof_indices is None else np.atleast_1d(dof_indices)
                result = getter(dof_indices=dof_indices)
                check_array(result, shape=(num_prims, len(columns)), dtype=wp.float32, device=device)
                self.assertTrue(result.is_contiguous)
                np.testing.assert_allclose(result.numpy(), expected[:, columns], rtol=1e-5, atol=1e-6)

    @parametrize(
        backends=["tensor"],
        operations=["wrap"],
        supported_engines=["physx"],
        instances=["many"],
        prim_class=Articulation,
        populate_stage_func=populate_stage,
        max_num_prims=2,
    )
    async def test_compensation_force_tensor_contract(
        self, prim: Any, num_prims: Any, device: Any, backend: Any
    ) -> None:
        """Exercise indexing, transfer, empty selections and malformed layouts through the public getters.

        Args:
            prim: Articulation wrapper under test.
            num_prims: Number of wrapped articulations.
            device: Wrapper output device.
            backend: Backend under test.
        """
        self.check_backend(backend, prim)
        for name in ("gravity", "coriolis_and_centrifugal"):
            getter = getattr(prim, f"get_dof_{name}_compensation_forces")
            raw_name = f"get_{name}_compensation_forces"
            for source_device in ("cpu", "cuda"):
                for root_columns in (0, 6):
                    values = np.arange(num_prims * (prim.num_dofs + root_columns), dtype=np.float32).reshape(
                        num_prims, prim.num_dofs + root_columns
                    )
                    raw = wp.array(values, device=source_device)
                    with patch.object(prim._physics_articulation_view, raw_name, return_value=raw):
                        row_cases = (None, [1, 0, 0], 0, [], np.array([1]), wp.array([0], dtype=wp.int32))
                        column_cases = (
                            None,
                            list(range(prim.num_dofs + root_columns)),
                            [prim.num_dofs + root_columns - 1, 0, 0],
                            0,
                            [],
                            np.array([0]),
                            wp.array([0], dtype=wp.int32),
                        )
                        for rows in row_cases:
                            row_values = (
                                np.arange(num_prims)
                                if rows is None
                                else np.atleast_1d(rows.numpy() if isinstance(rows, wp.array) else rows).astype(
                                    np.int32
                                )
                            )
                            for columns in column_cases:
                                column_values = (
                                    np.arange(prim.num_dofs + root_columns)
                                    if columns is None
                                    else np.atleast_1d(
                                        columns.numpy() if isinstance(columns, wp.array) else columns
                                    ).astype(np.int32)
                                )
                                result = getter(indices=rows, dof_indices=columns)
                                check_array(
                                    result,
                                    shape=(len(row_values), len(column_values)),
                                    dtype=wp.float32,
                                    device=device,
                                )
                                self.assertTrue(result.is_contiguous)
                                np.testing.assert_array_equal(result.numpy(), values[np.ix_(row_values, column_values)])
                        np.testing.assert_array_equal(raw.numpy(), values)
                for shape in ((num_prims,), (num_prims, prim.num_dofs + 1), (num_prims + 1, prim.num_dofs)):
                    with patch.object(
                        prim._physics_articulation_view, raw_name, return_value=wp.zeros(shape, device=source_device)
                    ):
                        with self.assertRaisesRegex(RuntimeError, "Unexpected compensation-force tensor shape"):
                            getter()
                with patch.object(Articulation, "num_dofs", new_callable=PropertyMock, return_value=0):
                    for root_columns in (0, 6):
                        with patch.object(
                            prim._physics_articulation_view,
                            raw_name,
                            return_value=wp.zeros((num_prims, root_columns), device=source_device),
                        ):
                            result = getter()
                            check_array(result, shape=(num_prims, root_columns), dtype=wp.float32, device=device)

    @parametrize(
        backends=["tensor"],
        operations=["wrap"],
        prim_class=Articulation,
        populate_stage_func=populate_compensation_pendulum,
        max_num_prims=2,
    )
    async def test_analytical_compensation_forces(self, prim: Any, num_prims: Any, device: Any, backend: Any) -> None:
        """Compare two-link pendulum torques against analytical inverse dynamics.

        Args:
            prim: Articulation wrapper under test.
            num_prims: Number of wrapped articulations.
            device: Output device.
            backend: Backend under test.
        """
        self.check_backend(backend, prim)
        self.assertEqual(prim.num_dofs, 2)
        stage = None
        if SimulationManager.get_active_physics_engine() == "newton":
            from isaacsim.physics.newton.impl.extension import acquire_stage

            stage = acquire_stage()
        for q1, q2 in ((0.0, np.pi / 2), (0.3, -np.pi / 3)):
            for v1, v2 in ((0.0, 0.0), (0.7, -0.4)):
                angles = np.array([[q1 + 0.15 * i, q2] for i in range(num_prims)], dtype=np.float32)
                velocities = np.array([[v1 + 0.2 * i, v2] for i in range(num_prims)], dtype=np.float32)
                prim.set_dof_positions(angles)
                prim.set_dof_velocities(velocities)
                snapshot = (
                    {}
                    if stage is None
                    else {
                        name: getattr(stage.state_0, name).numpy().copy()
                        for name in ("joint_q", "joint_qd", "body_q", "body_qd", "body_f")
                    }
                )
                angle1, angle2 = angles.T
                velocity1, velocity2 = velocities.T
                gravity = np.column_stack(
                    (
                        -9.81 * (1.5 * np.cos(angle1) + 0.5 * np.cos(angle1 + angle2)),
                        -4.905 * np.cos(angle1 + angle2),
                    )
                )
                coriolis = np.column_stack(
                    (
                        -0.5 * np.sin(angle2) * (2 * velocity1 * velocity2 + velocity2 * velocity2),
                        0.5 * np.sin(angle2) * velocity1 * velocity1,
                    )
                )
                for getter, expected in (
                    (prim.get_dof_gravity_compensation_forces, gravity),
                    (prim.get_dof_coriolis_and_centrifugal_compensation_forces, coriolis),
                ):
                    result = getter()
                    check_array(result, shape=(num_prims, 2), dtype=wp.float32, device=device)
                    np.testing.assert_allclose(result.numpy(), expected, atol=2e-4, rtol=1e-4)
                    np.testing.assert_allclose(getter(dof_indices=[1, 0]).numpy(), result.numpy()[:, [1, 0]], atol=1e-6)
                if num_prims > 1:
                    # A view of only the second articulation exercises a nonzero model DOF offset and mask.
                    subset = Articulation("/World/A_1")
                    np.testing.assert_allclose(
                        subset.get_dof_gravity_compensation_forces().numpy(), gravity[1:], atol=2e-4
                    )
                    np.testing.assert_allclose(
                        subset.get_dof_coriolis_and_centrifugal_compensation_forces().numpy(), coriolis[1:], atol=2e-4
                    )
                for name, before in snapshot.items():
                    np.testing.assert_array_equal(getattr(stage.state_0, name).numpy(), before)

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

    @parametrize(
        devices=["cpu"],
        backends=["tensor"],
        instances=["one"],
        operations=["wrap"],
        supported_engines=["physx"],
        prim_class=Articulation,
        populate_stage_func=populate_stage,
    )
    async def test_dof_getters_return_independent_arrays(
        self, prim: Any, num_prims: Any, device: Any, backend: Any
    ) -> None:
        """No-index DOF getters must not expose mutable physics buffers.

        Args:
            prim: Prim wrapper under test.
            num_prims: Number of prims under test.
            device: Device under test.
            backend: Backend name under test.
        """
        getters = [
            prim.get_dof_positions,
            prim.get_dof_velocities,
            prim.get_dof_position_targets,
            prim.get_dof_velocity_targets,
        ]
        with use_backend(backend, raise_on_unsupported=True, raise_on_fallback=True):
            for getter in getters:
                first = getter()
                second = getter()
                self.assertIsNot(first, second)
                self.assertNotEqual(int(first.ptr), int(second.ptr))

            held = prim.get_dof_positions()
            expected = held.numpy().copy()
            prim.set_dof_positions(np.full((num_prims, prim.num_dofs), 0.123, dtype=np.float32))
        np.testing.assert_array_equal(held.numpy(), expected)

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
        supported_engines=["physx", "newton"],
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
                    if SimulationManager.get_active_physics_engine() == "newton":
                        check_allclose(
                            (expected_v0, expected_v0, np.zeros(output[2].shape, dtype=np.float32)),
                            output,
                            given=(v0, v1, v2),
                        )
                    else:
                        check_allclose((expected_v0, expected_v1, expected_v2), output, given=(v0, v1, v2))

        if SimulationManager.get_active_physics_engine() == "newton":
            with use_backend(backend, raise_on_unsupported=True, raise_on_fallback=True):
                before = prim.get_dof_friction_properties(indices=[0], dof_indices=[0])[0].numpy()
            with patch("isaacsim.core.experimental.prims.impl.articulation.carb.log_warn") as log_warn:
                with use_backend(backend, raise_on_unsupported=True, raise_on_fallback=True):
                    prim.set_dof_friction_properties(
                        dynamic_frictions=0.2, viscous_frictions=0.1, indices=[0], dof_indices=[0]
                    )
                    prim.set_dof_friction_properties(
                        dynamic_frictions=0.2, viscous_frictions=0.1, indices=[0], dof_indices=[0]
                    )
            log_warn.assert_called_once()
            warning = " ".join(call.args[0] for call in log_warn.call_args_list)
            self.assertIn("No value will be changed because static_frictions was not provided", warning)
            self.assertIn("dynamic_frictions and viscous_frictions are ignored", warning)
            self.assertIn("XPBD and VBD solvers ignore joint friction", warning)
            with use_backend(backend, raise_on_unsupported=True, raise_on_fallback=True):
                after = prim.get_dof_friction_properties(indices=[0], dof_indices=[0])[0].numpy()
            self.assertTrue(np.allclose(before, after))

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
        await self._assert_articulation_metadata_consistent(
            "Isaac/Robots_Multiphysics/FrankaRobotics/FrankaPanda/franka/franka.usda"
        )

    async def test_articulation_metadata_consistent_floating_base(self) -> None:
        """Floating-base articulation (Spot quadruped): pre-physics metadata must match post-physics."""
        await self._assert_articulation_metadata_consistent("Isaac/Robots_Multiphysics/BostonDynamics/spot/spot.usda")
