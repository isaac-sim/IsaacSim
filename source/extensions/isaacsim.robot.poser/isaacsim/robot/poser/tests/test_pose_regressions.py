# SPDX-FileCopyrightText: Copyright (c) 2025-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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

"""Numerical and USD regressions for posing without a running simulation."""

from __future__ import annotations

import json
import tempfile
from pathlib import Path

import numpy as np
import omni.kit.test
from isaacsim.robot.poser.kinematic_chain import KinematicChain
from isaacsim.robot.poser.lm_ik import ik_lm
from isaacsim.robot.poser.math import Joint, Transform
from isaacsim.robot.poser.robot_poser import RobotPoser, import_poses, list_named_poses
from pxr import Gf, Usd, UsdGeom, UsdPhysics
from usd.schema.isaac import robot_schema


def create_robot(stage: Usd.Stage) -> tuple[Usd.Prim, Usd.Prim, Usd.Prim]:
    """Create a schema-populated two-body fixture.

    Args:
        stage: Stage to populate.

    Returns:
        Root, child body, and joint prims.
    """
    root = UsdGeom.Xform.Define(stage, "/Robot").GetPrim()
    link = UsdGeom.Xform.Define(stage, "/Robot/link").GetPrim()
    UsdPhysics.ArticulationRootAPI.Apply(root)
    for prim in (root, link):
        UsdPhysics.RigidBodyAPI.Apply(prim)
    joint = UsdPhysics.RevoluteJoint.Define(stage, "/Robot/joint")
    joint.CreateBody0Rel().SetTargets([root.GetPath()])
    joint.CreateBody1Rel().SetTargets([link.GetPath()])
    joint.CreateAxisAttr("Z")
    joint.CreateLocalPos0Attr(Gf.Vec3f(1, 0, 0))
    robot_schema.ApplyRobotAPI(root)
    assert root.HasAPI(robot_schema.Classes.ROBOT_API.value)
    assert link.HasAPI(robot_schema.Classes.LINK_API.value)
    return root, link, joint.GetPrim()


class TestPoseRegressions(omni.kit.test.AsyncTestCase):
    """Measure derivatives, persisted transforms and import atomicity."""

    def test_position_only_ik_and_spatial_jacobian(self) -> None:
        """Check planar two-joint point velocity and reachable FK targets."""
        chain = object.__new__(KinematicChain)
        chain._debug = False
        chain._joints = [
            Joint(w=np.array([0.0, 0.0, 1.0]), v=np.zeros(3), home=Transform(), tip=Transform(t=[1.0, 0.0, 0.0])),
            Joint(w=np.array([0.0, 0.0, 1.0]), v=np.zeros(3), home=Transform(), tip=Transform(t=[0.7, 0.0, 0.0])),
        ]
        q = np.array([0.3, -0.7])
        transform, spatial = chain.compute_fk_and_jacobian(q)
        geometric = spatial[3:] + np.cross(spatial[:3].T, transform.t).T
        h = 1e-6
        for i in range(2):
            delta = np.eye(2)[i] * h
            derivative = (chain.compute_fk(q + delta)[0].t - chain.compute_fk(q - delta)[0].t) / (2 * h)
            np.testing.assert_allclose(geometric[:, i], derivative, atol=1e-6)
        for weight in (0.0, 0.1, 1.0):
            result = ik_lm(chain, q + [0.08, -0.05], transform, w_rot=weight, iters=150, tol=1e-9)
            np.testing.assert_allclose(chain.compute_fk(result)[0].t, transform.t, atol=1e-7)

    def test_prismatic_position_control(self) -> None:
        """Keep prismatic point derivatives and solves unchanged."""
        chain = object.__new__(KinematicChain)
        chain._debug = False
        chain._joints = [Joint(w=np.zeros(3), v=np.array([1.0, 0.0, 0.0]), home=Transform(), is_revolute=False)]
        target, jacobian = chain.compute_fk_and_jacobian(np.array([0.4]))
        np.testing.assert_allclose(jacobian[3:, 0], [1.0, 0.0, 0.0])
        result = ik_lm(chain, np.zeros(1), target, w_rot=0.0, tol=1e-9)
        np.testing.assert_allclose(result, [0.4], atol=1e-7)

    def test_pose_preserves_uniform_scales_and_serialization(self) -> None:
        """Apply public chain posing and reopen the authored scale values."""
        for scale in (1.0, 2.0, 0.01, 100.0):
            stage = Usd.Stage.CreateInMemory()
            root, link, joint = create_robot(stage)
            UsdGeom.Xformable(root).AddOrientOp().Set(Gf.Quatf(Gf.Rotation(Gf.Vec3d(0, 0, 1), 35).GetQuat()))
            UsdGeom.Xformable(root).AddScaleOp().Set(Gf.Vec3f(2))
            UsdGeom.Xformable(link).AddScaleOp().Set(Gf.Vec3f(scale))
            root_rotation = UsdGeom.Xformable(root).GetLocalTransformation().RemoveScaleShear()
            descendant = UsdGeom.Cube.Define(stage, "/Robot/link/shape")
            descendant.AddScaleOp().Set(Gf.Vec3f(0.25))
            chain = KinematicChain(stage, root)
            chain.teleport({str(joint.GetPath()): 0.4})
            chain.teleport_anchored({str(joint.GetPath()): 0.6}, anchor_prim=root)
            np.testing.assert_allclose(
                UsdGeom.Xformable(root).GetLocalTransformation().RemoveScaleShear(), root_rotation, atol=1e-7
            )
            world = np.asarray(UsdGeom.XformCache().GetLocalToWorldTransform(descendant.GetPrim()))
            np.testing.assert_allclose(np.linalg.norm(world[:3, :3], axis=1), [0.5 * scale] * 3, rtol=1e-6)
            with tempfile.TemporaryDirectory() as directory:
                filename = str(Path(directory) / "stage.usda")
                stage.GetRootLayer().Export(filename)
                reopened = Usd.Stage.Open(filename)
                np.testing.assert_allclose(
                    reopened.GetPrimAtPath("/Robot/link").GetAttribute("xformOp:scale").Get(), [scale] * 3
                )
                np.testing.assert_allclose(
                    reopened.GetPrimAtPath("/Robot").GetAttribute("xformOp:scale").Get(), [2.0] * 3
                )

    def test_unsupported_stack_leaves_stage_unchanged(self) -> None:
        """Reject nonuniform scale before authoring joint state or transforms."""
        stage = Usd.Stage.CreateInMemory()
        root, link, joint = create_robot(stage)
        UsdGeom.Xformable(link).AddScaleOp().Set(Gf.Vec3f(1, 2, 3))
        before = stage.GetRootLayer().ExportToString()
        with self.assertRaises(ValueError):
            KinematicChain(stage, root).teleport({str(joint.GetPath()): 0.4})
        self.assertEqual(before, stage.GetRootLayer().ExportToString())

    def test_import_validation_is_atomic_and_empty_pose_is_valid(self) -> None:
        """Isolate list joints and missing endpoints with a valid entry preceding each."""
        stage = Usd.Stage.CreateInMemory()
        root, _, _ = create_robot(stage)
        valid = {"joints": {"/Robot/joint": 0.3}, "start_link": "/Robot", "end_link": "/Robot/link"}
        for invalid in (dict(valid, joints=[]), dict(valid, start_link=""), dict(valid, target_position=[1.0])):
            before = stage.GetRootLayer().ExportToString()
            with tempfile.TemporaryDirectory() as directory:
                filename = str(Path(directory) / "poses.json")
                Path(filename).write_text(json.dumps({"good": valid, "bad": invalid}))
                with self.assertRaises(ValueError):
                    import_poses(stage, root, filename)
            self.assertEqual(before, stage.GetRootLayer().ExportToString())
        with tempfile.TemporaryDirectory() as directory:
            filename = str(Path(directory) / "poses.json")
            Path(filename).write_text(json.dumps({"empty": dict(valid, joints={}), "good": valid}))
            self.assertEqual(import_poses(stage, root, filename), 2)
        self.assertEqual(set(list_named_poses(stage, root)), {"empty", "good"})

    def test_robot_poser_reports_reachable_position_target(self) -> None:
        """Check solver success against independent FK on a schema-populated robot."""
        stage = Usd.Stage.CreateInMemory()
        root, link, joint = create_robot(stage)
        site = UsdGeom.Xform.Define(stage, "/Robot/link/tool")
        site.AddTranslateOp().Set(Gf.Vec3d(1, 0, 0))
        robot_schema.ApplySiteAPI(site.GetPrim())
        poser = RobotPoser(stage, root, root, site.GetPrim())
        chain = KinematicChain(stage, root, root, site.GetPrim())
        target = chain.compute_fk(np.array([0.4]))[0]
        for weight in (0.0, 0.1, 1.0):
            result = poser.solve_ik(target, seed=np.array([0.3]), w_rot=weight, tol=1e-8)
            self.assertTrue(result.success)
            actual = chain.compute_fk(np.array([result.joints[str(joint.GetPath())]]))[0]
            np.testing.assert_allclose(actual.t, target.t, atol=1e-7)

    def test_articulation_cache_routes_targets_after_real_stage_replacement(self) -> None:
        """Exercise real USD stages and notices with a target-recording articulation adapter."""
        import sys
        import types
        from unittest.mock import Mock, patch

        from isaacsim.robot.poser import robot_poser

        stages = [Usd.Stage.CreateInMemory(), Usd.Stage.CreateInMemory()]
        robots = [create_robot(stage)[0] for stage in stages]
        second_joint = UsdPhysics.PrismaticJoint.Define(stages[1], "/Robot/second_joint")
        adapters = [Mock(), Mock(), Mock()]
        adapters[0].dof_paths = [["/Robot/joint"]]
        for adapter in adapters[1:]:
            adapter.dof_paths = [["/Robot/second_joint", "/Robot/joint"]]
        factory = Mock(side_effect=adapters)
        module = types.ModuleType("isaacsim.core.experimental.prims")
        module.Articulation = factory
        robot_poser.invalidate_articulation_cache()
        try:
            with patch.dict(sys.modules, {module.__name__: module}):
                robot_poser._drive_robot(stages[0], robots[0], {"/Robot/joint": 0.2})
                robot_poser._drive_robot(stages[0], robots[0], {"/Robot/joint": 0.3})
                self.assertEqual(factory.call_count, 1)
                robot_poser._drive_robot(stages[1], robots[1], {"/Robot/joint": 0.7})
                self.assertEqual(factory.call_count, 2)
                call = adapters[1].set_dof_position_targets.call_args
                np.testing.assert_allclose(call.args[0], [[0.7]])
                self.assertEqual(call.kwargs["dof_indices"], [1])
                second_joint.CreateBody1Rel().SetTargets(["/Robot/link"])
                robot_poser._drive_robot(stages[1], robots[1], {"/Robot/joint": 0.8})
                self.assertEqual(factory.call_count, 3)
        finally:
            robot_poser.invalidate_articulation_cache()

    def test_scaled_joint_offsets_keep_world_anchors_coincident(self) -> None:
        """Preserve both joint anchors when connected bodies have different scales."""
        for root_scale, link_scale in ((1.0, 2.0), (2.0, 0.01), (0.01, 100.0)):
            with self.subTest(root_scale=root_scale, link_scale=link_scale):
                stage = Usd.Stage.CreateInMemory()
                root, link, joint_prim = create_robot(stage)
                root_xform = UsdGeom.Xformable(root)
                root_xform.AddOrientOp(UsdGeom.XformOp.PrecisionDouble).Set(
                    Gf.Rotation(Gf.Vec3d(0, 0, 1), 35).GetQuat()
                )
                root_xform.AddScaleOp(UsdGeom.XformOp.PrecisionDouble).Set(Gf.Vec3d(root_scale))
                link_xform = UsdGeom.Xformable(link)
                link_xform.AddTranslateOp().Set(Gf.Vec3d(1.0 - 0.5 * link_scale, 0, 0))
                link_xform.AddScaleOp(UsdGeom.XformOp.PrecisionDouble).Set(Gf.Vec3d(link_scale))
                joint = UsdPhysics.Joint(joint_prim)
                joint.CreateLocalPos1Attr(Gf.Vec3f(0.5, 0, 0))
                chain = KinematicChain(stage, root)

                def assert_anchors() -> None:
                    cache = UsdGeom.XformCache()
                    anchor0 = cache.GetLocalToWorldTransform(root).Transform(Gf.Vec3d(joint.GetLocalPos0Attr().Get()))
                    anchor1 = cache.GetLocalToWorldTransform(link).Transform(Gf.Vec3d(joint.GetLocalPos1Attr().Get()))
                    np.testing.assert_allclose(anchor0, anchor1, atol=1e-8)

                assert_anchors()
                for angle in (0.0, 0.4):
                    chain.teleport({str(joint_prim.GetPath()): angle})
                    assert_anchors()
                anchor_before = UsdGeom.XformCache().GetLocalToWorldTransform(link)
                chain.teleport_anchored({str(joint_prim.GetPath()): -0.2}, anchor_prim=link)
                assert_anchors()
                np.testing.assert_allclose(
                    UsdGeom.XformCache().GetLocalToWorldTransform(link), anchor_before, atol=1e-8
                )
                self.assertEqual(root.GetAttribute("xformOp:scale").Get(), Gf.Vec3d(root_scale))
                self.assertEqual(link.GetAttribute("xformOp:scale").Get(), Gf.Vec3d(link_scale))

    def test_import_rejects_native_value_overflow_before_any_writes(self) -> None:
        """Reject both degree-conversion overflow and finite values outside USD float range."""
        stage = Usd.Stage.CreateInMemory()
        root, _, _ = create_robot(stage)
        valid = {"joints": {"/Robot/joint": 0.3}, "start_link": "/Robot", "end_link": "/Robot/link"}
        for units, value in (("radians", 1e308), ("radians", 1e38), ("degrees", 1e100)):
            with self.subTest(units=units, value=value):
                before = stage.GetRootLayer().ExportToString()
                data = {
                    "_meta": {"units": units},
                    "poses": {"valid": valid, "overflow": dict(valid, joints={"/Robot/joint": value})},
                }
                with tempfile.TemporaryDirectory() as directory:
                    filename = str(Path(directory) / "poses.json")
                    Path(filename).write_text(json.dumps(data))
                    with self.assertRaises(ValueError):
                        import_poses(stage, root, filename)
                self.assertEqual(stage.GetRootLayer().ExportToString(), before)

    def test_articulation_cache_does_not_keep_usd_stage_alive(self) -> None:
        """Release the weak stage identity and adapter cache when its stage expires."""
        import gc
        import sys
        import types
        import weakref
        from unittest.mock import Mock, patch

        from isaacsim.robot.poser import robot_poser

        robot_poser.invalidate_articulation_cache()
        stage = Usd.Stage.CreateInMemory()
        stage_ref = weakref.ref(stage)
        adapter = Mock()
        adapter.dof_paths = [["/Robot/joint"]]
        module = types.ModuleType("isaacsim.core.experimental.prims")
        module.Articulation = Mock(return_value=adapter)
        try:
            with patch.dict(sys.modules, {module.__name__: module}):
                robot_poser._get_articulation("/Robot", stage)
            del stage
            gc.collect()
            self.assertIsNone(stage_ref())
            self.assertFalse(robot_poser._articulation_cache)
        finally:
            robot_poser.invalidate_articulation_cache()

    def test_authored_transform_stacks_preserve_values_and_joint_anchors(self) -> None:
        """Pose matrix, Euler, pivot and unitsResolve stacks without flattening their authored operations."""
        for kind in ("matrix", "euler", "pivot", "units", "reset"):
            with self.subTest(kind=kind):
                stage = Usd.Stage.CreateInMemory()
                root, link, joint_prim = create_robot(stage)
                joint = UsdPhysics.Joint(joint_prim)
                joint.CreateLocalPos1Attr(Gf.Vec3f(0.25, 0, 0))
                UsdGeom.Xformable(root).AddScaleOp().Set(Gf.Vec3f(2))
                xf = UsdGeom.Xformable(link)
                if kind == "matrix":
                    xf.AddTransformOp().Set(Gf.Matrix4d().SetScale(0.5))
                elif kind == "euler":
                    xf.AddRotateXYZOp().Set(Gf.Vec3f(20, 30, 40))
                elif kind == "pivot":
                    xf.AddTranslateOp(opSuffix="pivot").Set(Gf.Vec3d(0.2, 0.3, 0))
                    xf.AddRotateXYZOp().Set(Gf.Vec3f(0, 0, 25))
                    xf.AddTranslateOp(opSuffix="pivot", isInverseOp=True)
                elif kind == "units":
                    xf.AddScaleOp(opSuffix="unitsResolve").Set(Gf.Vec3f(0.01))
                else:
                    xf.SetResetXformStack(True)
                    xf.AddTransformOp().Set(Gf.Matrix4d(1))
                original = [(op.GetOpName(), op.Get()) for op in xf.GetOrderedXformOps()]
                original_scale = np.linalg.norm(
                    np.asarray(UsdGeom.XformCache().GetLocalToWorldTransform(link))[:3, :3], axis=1
                )
                chain = KinematicChain(stage, root)
                for angle in (0.0, 0.4, -0.2):
                    chain.teleport({str(joint_prim.GetPath()): angle})
                    cache = UsdGeom.XformCache()
                    world = cache.GetLocalToWorldTransform(link)
                    np.testing.assert_allclose(
                        np.linalg.norm(np.asarray(world)[:3, :3], axis=1), original_scale, rtol=1e-6
                    )
                    np.testing.assert_allclose(
                        world.Transform(Gf.Vec3d(joint.GetLocalPos1Attr().Get())),
                        cache.GetLocalToWorldTransform(root).Transform(Gf.Vec3d(joint.GetLocalPos0Attr().Get())),
                        atol=1e-7,
                    )
                for name, value in original:
                    self.assertEqual(link.GetAttribute(name.removeprefix("!invert!")).Get(), value)
                self.assertEqual(xf.GetResetXformStack(), kind == "reset")
                self.assertEqual(len(xf.GetOrderedXformOps()), len(original) + 1)

    def test_correction_composes_noncommuting_world_poses(self) -> None:
        """Assert complete matrices for noncommuting poses, including repeated corrections."""
        from isaacsim.robot.poser.kinematic_chain import _apply_world_transform

        for kind in ("matrix", "euler", "pivot", "reset"):
            with self.subTest(kind=kind):
                stage = Usd.Stage.CreateInMemory()
                parent = UsdGeom.Xform.Define(stage, "/Parent")
                parent.AddTranslateOp().Set(Gf.Vec3d(4, -2, 3))
                parent.AddRotateXYZOp().Set(Gf.Vec3f(15, -25, 35))
                parent.AddScaleOp().Set(Gf.Vec3f(2))
                body = UsdGeom.Xform.Define(stage, "/Parent/Body")
                if kind in ("matrix", "reset"):
                    matrix = Gf.Matrix4d().SetRotate(Gf.Rotation(Gf.Vec3d(1, 2, 3), 47))
                    matrix.SetTranslateOnly(Gf.Vec3d(2, 3, -1))
                    body.AddTransformOp().Set(matrix)
                    body.SetResetXformStack(kind == "reset")
                else:
                    body.AddTranslateOp().Set(Gf.Vec3d(2, 3, -1))
                    if kind == "pivot":
                        body.AddTranslateOp(opSuffix="pivot").Set(Gf.Vec3d(0.3, -0.4, 0.2))
                    body.AddRotateXYZOp().Set(Gf.Vec3f(25, 35, -45))
                    if kind == "pivot":
                        body.AddTranslateOp(opSuffix="pivot", isInverseOp=True)
                original = body.GetLocalTransformation()
                authored = [(op.GetOpName(), op.Get()) for op in body.GetOrderedXformOps()]
                scale = 1.0 if kind == "reset" else 2.0
                for angle, translation in ((63, (7, -4, 5)), (-29, (-2, 5, 1))):
                    rigid = Gf.Matrix4d().SetRotate(Gf.Rotation(Gf.Vec3d(-2, 1, 3), angle))
                    rigid.SetTranslateOnly(Gf.Vec3d(*translation))
                    expected_world = Gf.Matrix4d().SetScale(scale) * rigid
                    expected_local = Gf.Matrix4d(expected_world)
                    if kind != "reset":
                        expected_local *= UsdGeom.XformCache().GetLocalToWorldTransform(parent.GetPrim()).GetInverse()
                    self.assertFalse(
                        np.allclose(np.asarray(original * expected_local), np.asarray(expected_local * original))
                    )
                    _apply_world_transform(body.GetPrim(), rigid)
                    np.testing.assert_allclose(
                        np.asarray(body.GetLocalTransformation()), np.asarray(expected_local), atol=1e-6
                    )
                    np.testing.assert_allclose(
                        np.asarray(UsdGeom.XformCache().GetLocalToWorldTransform(body.GetPrim())),
                        np.asarray(expected_world),
                        atol=1e-6,
                    )
                    for name, value in authored:
                        self.assertEqual(body.GetPrim().GetAttribute(name.removeprefix("!invert!")).Get(), value)
                    self.assertEqual(len(body.GetOrderedXformOps()), len(authored) + 1)

    def test_ancestor_validation_precedes_writes(self) -> None:
        """Reject nonuniform ancestors above the robot root without partially applying a pose."""
        source = Usd.Stage.CreateInMemory()
        create_robot(source)
        stage = Usd.Stage.CreateInMemory()
        parent = UsdGeom.Xform.Define(stage, "/Scaled")
        parent.AddScaleOp().Set(Gf.Vec3f(1, 2, 1))
        root = stage.DefinePrim("/Scaled/Robot")
        root.GetReferences().AddReference(source.GetRootLayer().identifier, "/Robot")
        before = stage.GetRootLayer().ExportToString()
        with self.assertRaisesRegex(ValueError, "/Scaled"):
            KinematicChain(stage, root).teleport({"/Scaled/Robot/joint": 0.4})
        self.assertEqual(stage.GetRootLayer().ExportToString(), before)

    def test_zero_configuration_uses_each_bodys_scale(self) -> None:
        """IK zero poses and direct FK agree for independently scaled joint frames."""
        from isaacsim.robot.poser.kinematic_chain import _compute_zero_config_poses

        stage = Usd.Stage.CreateInMemory()
        root, link, joint_prim = create_robot(stage)
        UsdGeom.Xformable(root).AddScaleOp().Set(Gf.Vec3f(2))
        UsdGeom.Xformable(link).AddScaleOp().Set(Gf.Vec3f(3))
        UsdPhysics.Joint(joint_prim).CreateLocalPos1Attr(Gf.Vec3f(0.25, 0, 0))
        chain = KinematicChain(stage, root)
        zero = _compute_zero_config_poses(chain._tree_root)
        chain.teleport({str(joint_prim.GetPath()): 0.0})
        np.testing.assert_allclose(
            zero[str(link.GetPath())], UsdGeom.XformCache().GetLocalToWorldTransform(link), atol=1e-7
        )

    def test_scaled_fk_and_jacobian_match_usd_pose(self) -> None:
        """Compare rigid FK, point derivatives and IK against independently posed USD bodies."""
        from isaacsim.robot.poser.math import quat_to_matrix

        for prismatic, backward in ((False, False), (False, True), (True, False), (True, True)):
            for root_scale, link_scale in ((2.0, 1.0), (1.0, 3.0), (0.01, 0.5)):
                with self.subTest(prismatic=prismatic, backward=backward, root_scale=root_scale, link_scale=link_scale):
                    stage = Usd.Stage.CreateInMemory()
                    UsdGeom.SetStageMetersPerUnit(stage, 1.0)
                    root, link, joint_prim = create_robot(stage)
                    if prismatic:
                        joint_prim = UsdPhysics.PrismaticJoint.Define(stage, joint_prim.GetPath()).GetPrim()
                    joint = UsdPhysics.Joint(joint_prim)
                    joint.CreateLocalRot0Attr(Gf.Quatf(Gf.Rotation(Gf.Vec3d(0, 0, 1), 40).GetQuat()))
                    joint.CreateLocalPos1Attr(Gf.Vec3f(0.25, 0, 0))
                    root_xform = UsdGeom.Xformable(root)
                    root_xform.AddOrientOp(UsdGeom.XformOp.PrecisionDouble).Set(
                        Gf.Rotation(Gf.Vec3d(1, 0, 0), 35).GetQuat()
                    )
                    root_xform.AddScaleOp(UsdGeom.XformOp.PrecisionDouble).Set(Gf.Vec3d(root_scale))
                    UsdGeom.Xformable(link).AddScaleOp(UsdGeom.XformOp.PrecisionDouble).Set(Gf.Vec3d(link_scale))
                    start, end = (link, root) if backward else (root, link)
                    chain = KinematicChain(stage, root, start, end)
                    joint_path = str(joint_prim.GetPath())

                    def pose_in_robot_frame(value: float) -> Gf.Matrix4d:
                        """Read a posed body in the numerical FK reference frame.

                        Args:
                            value: Joint displacement in radians or meters.

                        Returns:
                            Rigid body matrix in the chain-local frame.
                        """
                        if backward:
                            chain.teleport_anchored({joint_path: value}, anchor_prim=start)
                        else:
                            chain.teleport({joint_path: value})
                        cache = UsdGeom.XformCache()
                        robot_inverse = cache.GetLocalToWorldTransform(root).GetInverse()
                        start_pose = (cache.GetLocalToWorldTransform(start) * robot_inverse).RemoveScaleShear()
                        end_pose = (cache.GetLocalToWorldTransform(end) * robot_inverse).RemoveScaleShear()
                        return end_pose * start_pose.GetInverse()

                    value = 0.4
                    actual = pose_in_robot_frame(value)
                    target, jacobian = chain.compute_fk_and_jacobian(np.array([value]))
                    np.testing.assert_allclose(target.t, actual.ExtractTranslation(), atol=1e-6)
                    np.testing.assert_allclose(
                        quat_to_matrix(target.q).T, np.asarray(actual.RemoveScaleShear())[:3, :3], atol=1e-6
                    )
                    h = 1e-4
                    derivative = (
                        np.asarray(pose_in_robot_frame(value + h).ExtractTranslation())
                        - np.asarray(pose_in_robot_frame(value - h).ExtractTranslation())
                    ) / (2 * h)
                    geometric = jacobian[3:] + np.cross(jacobian[:3].T, target.t).T
                    np.testing.assert_allclose(geometric[:, 0], derivative, atol=1e-5)
                    solution = ik_lm(chain, np.array([value - 0.1]), target, tol=1e-9, iters=150)
                    np.testing.assert_allclose(
                        pose_in_robot_frame(float(solution[0])).ExtractTranslation(), target.t, atol=1e-6
                    )
