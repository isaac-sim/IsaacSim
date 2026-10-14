# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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

"""Verify Newton simulation stage lifecycle behavior."""

import math
from types import SimpleNamespace
from unittest.mock import Mock, patch

import newton
import numpy as np
import omni.kit.test
import omni.timeline
import omni.usd
import warp as wp
from isaacsim.physics.newton.impl import carb_config
from isaacsim.physics.newton.impl.newton_stage import NewtonStage
from isaacsim.physics.newton.impl.solver_config import MuJoCoSolverConfig
from isaacsim.physics.newton.impl.usd import UsdManager
from isaacsim.physics.newton.impl.utils import newton_solver_to_api_schema
from pxr import Gf, PhysxSchema, Usd, UsdGeom, UsdPhysics


class TestNewtonStageStopTransforms(omni.kit.test.AsyncTestCase):
    """Tests for Newton timeline Stop transform handling."""

    async def setUp(self) -> None:
        """Create a stage with nested transform prims."""
        await omni.usd.get_context().new_stage_async()
        stage = omni.usd.get_context().get_stage()

        parent = UsdGeom.Xform.Define(stage, "/World/Parent")
        parent.MakeMatrixXform().Set(Gf.Matrix4d(1.0).SetTranslate(Gf.Vec3d(10.0, 0.0, 0.0)))
        UsdPhysics.RigidBodyAPI.Apply(parent.GetPrim())
        child = UsdGeom.Xform.Define(stage, "/World/Parent/Child")
        child.MakeMatrixXform().Set(Gf.Matrix4d(1.0).SetTranslate(Gf.Vec3d(2.0, 0.0, 0.0)))
        UsdPhysics.RigidBodyAPI.Apply(child.GetPrim())

        self.body_paths = ["/World/Parent", "/World/Parent/Child"]

    async def tearDown(self) -> None:
        """Close the test stage."""
        await omni.usd.get_context().close_stage_async()

    def _make_newton_stage(self, solver_type: str = "mujoco") -> NewtonStage:
        """Create a lifecycle object with a stopped Newton pose.

        Args:
            solver_type: Solver identifier to configure.

        Returns:
            Newton stage lifecycle object backed by the current USD stage.
        """
        stage = omni.usd.get_context().get_stage()
        newton_stage = object.__new__(NewtonStage)
        newton_stage.playing = True
        newton_stage.graph = object()
        newton_stage._init_failed = True
        newton_stage.initialized = True
        newton_stage.cfg = SimpleNamespace(solver_cfg=SimpleNamespace(solver_type=solver_type))
        newton_stage.model = SimpleNamespace(
            body_label=self.body_paths,
            joint_label=[],
            joint_q_start=wp.array([0], dtype=wp.int32, device=str(wp.get_device())),
            joint_type=wp.array([], dtype=wp.int32, device=str(wp.get_device())),
        )
        newton_stage.state_0 = SimpleNamespace(
            body_q=wp.array(
                [
                    wp.transform((1.0, 0.0, 0.0), wp.quat_identity()),
                    wp.transform((4.0, 0.0, 0.0), wp.quat_identity()),
                ],
                dtype=wp.transformf,
                device=str(wp.get_device()),
            ),
            joint_q=wp.array([], dtype=wp.float32, device=str(wp.get_device())),
        )
        newton_stage.scene_scale = 1.0
        newton_stage.stage_id = omni.usd.get_context().get_stage_id()
        newton_stage.usd_manager = UsdManager(stage)
        return newton_stage

    async def test_stop_without_reset_authors_nested_body_transforms_to_usd(
        self,
    ) -> None:
        """Author final nested body transforms without replacing their xform ops."""
        expected_worlds = {
            "/World/Parent": Gf.Matrix4d().SetIdentity().SetTranslate(Gf.Vec3d(1.0, 0.0, 0.0)),
            "/World/Parent/Child": Gf.Matrix4d().SetIdentity().SetTranslate(Gf.Vec3d(4.0, 0.0, 0.0)),
        }
        expected_locals = {
            "/World/Parent": Gf.Matrix4d().SetIdentity().SetTranslate(Gf.Vec3d(1.0, 0.0, 0.0)),
            "/World/Parent/Child": Gf.Matrix4d().SetIdentity().SetTranslate(Gf.Vec3d(3.0, 0.0, 0.0)),
        }
        stage = omni.usd.get_context().get_stage()
        original_op_orders = {
            path: UsdGeom.Xformable(stage.GetPrimAtPath(path)).GetXformOpOrderAttr().Get() for path in self.body_paths
        }

        newton_stage = self._make_newton_stage()
        with patch.object(carb_config, "reset_on_stop", return_value=False):
            newton_stage.on_timeline_event(SimpleNamespace(type=int(omni.timeline.TimelineEventType.STOP)))

        for path, expected in expected_locals.items():
            xformable = UsdGeom.Xformable(stage.GetPrimAtPath(path))
            actual = xformable.GetLocalTransformation(Usd.TimeCode.Default())
            np.testing.assert_allclose(np.array(actual), np.array(expected), rtol=0.0, atol=1.0e-12)
            self.assertEqual(xformable.GetXformOpOrderAttr().Get(), original_op_orders[path])
            self.assertEqual(len(xformable.GetOrderedXformOps()), 1)

        for path, expected in expected_worlds.items():
            actual = UsdGeom.Xformable(stage.GetPrimAtPath(path)).ComputeLocalToWorldTransform(Usd.TimeCode.Default())
            np.testing.assert_allclose(np.array(actual), np.array(expected), rtol=0.0, atol=1.0e-12)

        reloaded_stage = Usd.Stage.CreateInMemory()
        reloaded_stage.GetRootLayer().TransferContent(stage.GetRootLayer())
        for path, expected in expected_worlds.items():
            actual = UsdGeom.Xformable(reloaded_stage.GetPrimAtPath(path)).ComputeLocalToWorldTransform(
                Usd.TimeCode.Default()
            )
            np.testing.assert_allclose(np.array(actual), np.array(expected), rtol=0.0, atol=1.0e-12)

    async def test_stop_updates_child_local_when_only_parent_world_moves(self) -> None:
        """Update a nested child's local pose when its target world pose is unchanged."""
        newton_stage = self._make_newton_stage()
        newton_stage.state_0.body_q = wp.array(
            [
                wp.transform((1.0, 0.0, 0.0), wp.quat_identity()),
                wp.transform((12.0, 0.0, 0.0), wp.quat_identity()),
            ],
            dtype=wp.transformf,
            device=str(wp.get_device()),
        )

        with patch.object(carb_config, "reset_on_stop", return_value=False):
            newton_stage.on_timeline_event(SimpleNamespace(type=int(omni.timeline.TimelineEventType.STOP)))

        stage = omni.usd.get_context().get_stage()
        child = UsdGeom.Xformable(stage.GetPrimAtPath(self.body_paths[1]))
        expected_local = Gf.Matrix4d(1.0).SetTranslate(Gf.Vec3d(11.0, 0.0, 0.0))
        expected_world = Gf.Matrix4d(1.0).SetTranslate(Gf.Vec3d(12.0, 0.0, 0.0))
        np.testing.assert_allclose(
            np.array(child.GetLocalTransformation(Usd.TimeCode.Default())),
            np.array(expected_local),
            rtol=0.0,
            atol=1.0e-12,
        )
        np.testing.assert_allclose(
            np.array(child.ComputeLocalToWorldTransform(Usd.TimeCode.Default())),
            np.array(expected_world),
            rtol=0.0,
            atol=1.0e-12,
        )

    async def test_stop_without_reset_warns_and_skips_non_mujoco_solvers(
        self,
    ) -> None:
        """Warn and skip USD preservation for XPBD and VBD."""
        for solver_type in ("xpbd", "vbd"):
            with self.subTest(solver_type=solver_type):
                newton_stage = self._make_newton_stage(solver_type=solver_type)
                newton_stage.usd_manager = Mock(wraps=newton_stage.usd_manager)

                with (
                    patch.object(carb_config, "reset_on_stop", return_value=False),
                    patch("isaacsim.physics.newton.impl.newton_stage.carb.log_warn") as log_warn,
                ):
                    newton_stage.on_timeline_event(SimpleNamespace(type=int(omni.timeline.TimelineEventType.STOP)))

                newton_stage.usd_manager.snapshot_body_world_transforms.assert_not_called()
                self.assertIn(solver_type.upper(), log_warn.call_args.args[0])

    async def test_stop_with_reset_uses_existing_restore_path(self) -> None:
        """Use the existing reset path without authoring the simulated pose."""
        newton_stage = self._make_newton_stage()
        newton_stage._restore_fabric_transforms = Mock()
        newton_stage._write_usd_state_on_stop = Mock()

        with patch.object(carb_config, "reset_on_stop", return_value=True):
            newton_stage.on_timeline_event(SimpleNamespace(type=int(omni.timeline.TimelineEventType.STOP)))

        newton_stage._restore_fabric_transforms.assert_called_once_with()
        newton_stage._write_usd_state_on_stop.assert_not_called()

    async def test_stop_without_reset_skips_stale_stage(self) -> None:
        """Skip USD preservation when the stage manager is stale."""
        newton_stage = self._make_newton_stage()
        newton_stage.initialized = False
        newton_stage.usd_manager = Mock(wraps=newton_stage.usd_manager)

        with (
            patch.object(carb_config, "reset_on_stop", return_value=False),
            patch("isaacsim.physics.newton.impl.newton_stage.carb.log_warn") as log_warn,
        ):
            newton_stage.on_timeline_event(SimpleNamespace(type=int(omni.timeline.TimelineEventType.STOP)))

        newton_stage.usd_manager.snapshot_body_world_transforms.assert_not_called()
        self.assertIn("stale", log_warn.call_args.args[0])

    async def test_snapshot_joint_positions_skips_trailing_fixed_joint(self) -> None:
        """Skip a trailing fixed joint without indexing beyond joint coordinates."""
        model = SimpleNamespace(
            joint_label=["/World/Revolute", "/World/Fixed"],
            joint_q_start=wp.array([0, 1, 1], dtype=wp.int32, device=str(wp.get_device())),
            joint_type=wp.array(
                [int(newton.JointType.REVOLUTE), int(newton.JointType.FIXED)],
                dtype=wp.int32,
                device=str(wp.get_device()),
            ),
        )
        state = SimpleNamespace(joint_q=wp.array([math.pi / 4.0], dtype=wp.float32, device=str(wp.get_device())))

        positions = UsdManager(omni.usd.get_context().get_stage()).snapshot_joint_positions(model, state, 1.0)

        self.assertEqual(set(positions), {"/World/Revolute"})
        self.assertAlmostEqual(positions["/World/Revolute"][1], 45.0, places=5)

    async def test_snapshot_joint_positions_warns_for_unsupported_coordinates(
        self,
    ) -> None:
        """Warn when a coordinate-bearing joint type cannot be persisted."""
        model = SimpleNamespace(
            joint_label=["/World/Ball"],
            joint_q_start=wp.array([0, 4], dtype=wp.int32, device=str(wp.get_device())),
            joint_type=wp.array(
                [int(newton.JointType.BALL)],
                dtype=wp.int32,
                device=str(wp.get_device()),
            ),
        )
        state = SimpleNamespace(joint_q=wp.zeros(4, dtype=wp.float32, device=str(wp.get_device())))

        with patch("isaacsim.physics.newton.impl.usd.carb.log_warn") as log_warn:
            positions = UsdManager(omni.usd.get_context().get_stage()).snapshot_joint_positions(model, state, 1.0)

        self.assertEqual(positions, {})
        self.assertIn("/World/Ball", log_warn.call_args.args[0])

    async def test_write_simulation_state_preserves_joint_velocity(self) -> None:
        """Preserve an authored joint velocity while updating its position."""
        stage = omni.usd.get_context().get_stage()
        joint = UsdPhysics.RevoluteJoint.Define(stage, "/World/Joint")
        joint_state = PhysxSchema.JointStateAPI.Apply(joint.GetPrim(), "angular")
        joint_state.CreateVelocityAttr().Set(12.0)

        UsdManager(stage).write_simulation_state({}, {str(joint.GetPath()): ("angular", 45.0)})

        self.assertEqual(joint_state.GetPositionAttr().Get(), 45.0)
        self.assertEqual(joint_state.GetVelocityAttr().Get(), 12.0)

    async def test_write_simulation_state_preserves_existing_matrix_op(self) -> None:
        """Update an existing matrix op without replacing it or its reset flag."""
        stage = omni.usd.get_context().get_stage()
        body = UsdGeom.Xform.Define(stage, "/World/MatrixBody")
        matrix_op = body.MakeMatrixXform()
        matrix_op.Set(Gf.Matrix4d(1.0))
        body.SetXformOpOrder([matrix_op], resetXformStack=True)
        target = Gf.Matrix4d(1.0).SetTranslate(Gf.Vec3d(2.0, 3.0, 4.0))

        UsdManager(stage).write_simulation_state({str(body.GetPath()): target}, {})

        self.assertEqual(body.GetOrderedXformOps()[0].GetName(), matrix_op.GetName())
        self.assertTrue(body.GetResetXformStack())
        np.testing.assert_allclose(np.array(matrix_op.Get()), np.array(target), rtol=0.0, atol=1.0e-12)

    async def test_write_simulation_state_rejects_time_sampled_ops(self) -> None:
        """Reject time-sampled transforms before authoring any body pose."""
        stage = omni.usd.get_context().get_stage()
        safe_body = UsdGeom.Xform.Define(stage, "/World/SafeBody")
        safe_matrix = safe_body.MakeMatrixXform()
        safe_matrix.Set(Gf.Matrix4d(1.0).SetTranslate(Gf.Vec3d(1.0)))
        body = UsdGeom.Xform.Define(stage, "/World/AnimatedBody")
        animated_matrix = body.MakeMatrixXform()
        animated_matrix.Set(Gf.Matrix4d(1.0).SetTranslate(Gf.Vec3d(1.0)), Usd.TimeCode(1.0))

        with self.assertRaises(RuntimeError):
            UsdManager(stage).write_simulation_state(
                {
                    str(safe_body.GetPath()): Gf.Matrix4d(1.0).SetTranslate(Gf.Vec3d(3.0)),
                    str(body.GetPath()): Gf.Matrix4d(1.0).SetTranslate(Gf.Vec3d(2.0)),
                },
                {},
            )

        self.assertEqual(safe_matrix.Get(), Gf.Matrix4d(1.0).SetTranslate(Gf.Vec3d(1.0)))
        self.assertEqual(animated_matrix.GetTimeSamples(), [1.0])

    async def test_snapshot_body_transforms_rejects_nonuniform_world_scale(
        self,
    ) -> None:
        """Reject lossy pose persistence for a rotated non-uniformly scaled body."""
        stage = omni.usd.get_context().get_stage()
        parent = UsdGeom.Xformable(stage.GetPrimAtPath(self.body_paths[0]))
        transform = Gf.Transform()
        transform.SetRotation(Gf.Rotation(Gf.Vec3d.ZAxis(), 30.0))
        transform.SetScale(Gf.Vec3d(1.0, 2.0, 1.0))
        parent.GetOrderedXformOps()[0].Set(transform.GetMatrix())
        model = SimpleNamespace(body_label=[self.body_paths[0]])
        state = SimpleNamespace(
            body_q=wp.array(
                [wp.transform((1.0, 0.0, 0.0), wp.quat_identity())],
                dtype=wp.transformf,
                device=str(wp.get_device()),
            )
        )

        with self.assertRaises(RuntimeError):
            UsdManager(stage).snapshot_body_world_transforms(model, state, 1.0)


class TestNewtonStageInitialization(omni.kit.test.AsyncTestCase):
    """Tests for deriving Newton's initial joint state from body poses."""

    @staticmethod
    def _make_revolute_model() -> newton.Model:
        """Create a single-link revolute articulation.

        Returns:
            Finalized Newton model.
        """
        builder = newton.ModelBuilder(gravity=(0.0, 0.0, 0.0))
        link = builder.add_link(label="/World/Link")
        joint = builder.add_joint_revolute(parent=-1, child=link, axis=newton.Axis.Z)
        builder.add_articulation([joint])
        return builder.finalize(device=str(wp.get_device()))

    @staticmethod
    def _make_authored_pose(model: newton.Model, joint_position: float) -> newton.State:
        """Create a state whose body pose and joint coordinate disagree.

        Args:
            model: Newton model for the state.
            joint_position: Position used to create the authored body pose.

        Returns:
            State with the requested body pose and the model's original joint coordinate.
        """
        posed_state = model.state()
        posed_joint_q = posed_state.joint_q.numpy()
        posed_joint_q[0] = joint_position
        posed_state.joint_q.assign(posed_joint_q)
        newton.eval_fk(model, posed_state.joint_q, posed_state.joint_qd, posed_state, None)

        authored_state = model.state()
        authored_state.body_q.assign(posed_state.body_q)
        authored_state.body_qd.assign(posed_state.body_qd)
        return authored_state

    async def test_initialize_joint_state_trusts_authored_body_pose(self) -> None:
        """Derive initial joint coordinates without changing the authored body pose."""
        model = self._make_revolute_model()
        state = self._make_authored_pose(model, 0.6)
        authored_body_q = state.body_q.numpy().copy()
        newton_stage = object.__new__(NewtonStage)
        newton_stage.model = model
        newton_stage.state_0 = state

        newton_stage._initialize_joint_state_from_body_poses()

        np.testing.assert_allclose(state.joint_q.numpy(), [0.6], rtol=0.0, atol=1.0e-6)
        np.testing.assert_allclose(model.joint_q.numpy(), [0.6], rtol=0.0, atol=1.0e-6)
        np.testing.assert_allclose(state.body_q.numpy(), authored_body_q, rtol=0.0, atol=1.0e-7)

    async def test_initialize_joint_state_keeps_nearest_revolute_angle(self) -> None:
        """Keep an IK revolute coordinate on the branch nearest its parsed value."""
        model = self._make_revolute_model()
        state = self._make_authored_pose(model, 0.6)
        newton_stage = object.__new__(NewtonStage)
        newton_stage.model = model
        newton_stage.state_0 = state
        eval_ik = newton.eval_ik

        def eval_ik_on_next_branch(*args, **kwargs) -> None:
            eval_ik(*args, **kwargs)
            joint_q = state.joint_q.numpy()
            joint_q[0] += math.tau
            state.joint_q.assign(joint_q)

        with patch("isaacsim.physics.newton.impl.newton_stage.newton.eval_ik", side_effect=eval_ik_on_next_branch):
            newton_stage._initialize_joint_state_from_body_poses()

        np.testing.assert_allclose(state.joint_q.numpy(), [0.6], rtol=0.0, atol=1.0e-6)

    async def test_mujoco_cone_override_is_forwarded_to_solver(self) -> None:
        """Forward an explicit friction-cone override while preserving scene settings by default."""
        model = Mock()
        solver_factory = Mock()
        self.assertIsNone(MuJoCoSolverConfig().cone)

        with patch.dict(
            "isaacsim.physics.newton.impl.newton_stage.newton_solver_to_solver_object",
            {"mujoco": solver_factory},
            clear=True,
        ):
            NewtonStage._create_solver(model, MuJoCoSolverConfig(cone="elliptic"))

        solver_factory.assert_called_once()
        self.assertEqual(solver_factory.call_args.kwargs["cone"], "elliptic")


class TestNewtonStagePhysicsSceneSelection(omni.kit.test.AsyncTestCase):
    """Tests for resolving which physics scene Newton initializes from."""

    async def setUp(self) -> None:
        """Open an empty stage to author physics scenes onto."""
        await omni.usd.get_context().new_stage_async()
        self.stage = omni.usd.get_context().get_stage()

    async def tearDown(self) -> None:
        """Close the test stage."""
        await omni.usd.get_context().close_stage_async()

    def _lifecycle(self) -> NewtonStage:
        """Create a lifecycle object carrying only the scene-lookup state.

        Returns:
            Newton stage lifecycle object bound to the current USD stage.
        """
        newton_stage = object.__new__(NewtonStage)
        newton_stage.usd_stage = self.stage
        newton_stage.physics_scene_prim = None
        return newton_stage

    def _define_scene(self, path: str) -> Usd.Prim:
        """Author a physics scene.

        Args:
            path: Prim path for the physics scene.

        Returns:
            The authored physics scene prim, carrying the solver APIs that
            ``UsdPhysics.Scene`` auto-applies.
        """
        return UsdPhysics.Scene.Define(self.stage, path).GetPrim()

    def _parser_order(self) -> list[Usd.Prim]:
        """List the physics scenes in the order Newton's USD parser reports them.

        The preference tests strip APIs relative to this order rather than to
        prim-path sorting, so they stay valid if parser order changes.

        Returns:
            Physics scene prims in parser order.
        """
        return [scene.GetPrim() for scene in newton.usd.get_physics_scenes(self.stage)]

    def _strip_solver_apis(self, prim: Usd.Prim) -> Usd.Prim:
        """Remove the auto-applied solver APIs to model a scene Newton cannot drive.

        Args:
            prim: Physics scene prim to strip.

        Returns:
            The same prim, with every known Newton solver API removed.
        """
        for api in newton_solver_to_api_schema.values():
            if prim.HasAPI(api):
                prim.RemoveAPI(api)
        return prim

    async def test_prefers_scene_declaring_a_newton_solver_api(self) -> None:
        """Skip a bare scene that the parser reports first for a Newton-capable one."""
        self._define_scene("/World/AScene")
        self._define_scene("/World/BScene")
        order = self._parser_order()
        self.assertEqual(len(order), 2)
        bare = self._strip_solver_apis(order[0])
        expected = order[1]
        newton_stage = self._lifecycle()

        newton_stage._get_physics_scene()

        self.assertEqual(newton_stage.physics_scene_prim.GetPath(), expected.GetPath())
        self.assertNotEqual(newton_stage.physics_scene_prim.GetPath(), bare.GetPath())

    async def test_skips_scene_declaring_multiple_newton_solver_apis(self) -> None:
        """Skip a scene that declares conflicting Newton solver APIs."""
        self._define_scene("/World/AScene")
        self._define_scene("/World/BScene")
        order = self._parser_order()
        invalid = order[0]
        additional_api = next(api for api in newton_solver_to_api_schema.values() if not invalid.HasAPI(api))
        invalid.ApplyAPI(additional_api)
        expected = order[1]
        newton_stage = self._lifecycle()

        newton_stage._get_physics_scene()

        self.assertEqual(newton_stage.physics_scene_prim.GetPath(), expected.GetPath())
        self.assertNotEqual(newton_stage.physics_scene_prim.GetPath(), invalid.GetPath())

    async def test_leaves_scene_unset_when_no_scene_declares_a_solver_api(self) -> None:
        """Leave scene selection unset when no scene declares a solver API."""
        self._define_scene("/World/AScene")
        self._define_scene("/World/BScene")
        order = self._parser_order()
        for prim in order:
            self._strip_solver_apis(prim)
        newton_stage = self._lifecycle()

        newton_stage._get_physics_scene()

        self.assertIsNone(newton_stage.physics_scene_prim)

    async def test_clears_a_stale_selection(self) -> None:
        """Drop a previously selected scene that the current stage no longer has."""
        newton_stage = self._lifecycle()
        newton_stage.physics_scene_prim = UsdGeom.Xform.Define(self.stage, "/World/Stale").GetPrim()

        newton_stage._get_physics_scene()

        self.assertIsNone(newton_stage.physics_scene_prim)

    async def test_missing_usd_stage_is_not_an_error(self) -> None:
        """Leave the selection unset when no stage has been bound yet."""
        newton_stage = self._lifecycle()
        newton_stage.usd_stage = None

        newton_stage._get_physics_scene()

        self.assertIsNone(newton_stage.physics_scene_prim)
