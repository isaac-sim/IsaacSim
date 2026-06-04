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

"""Tests for the physics joint pose fix rule."""

from __future__ import annotations

import os
import shutil
import tempfile

import omni.kit.test
from isaacsim.asset.transformer.rules.isaac_sim.physics_joint_pose_fix import (
    PhysicsJointPoseFixRule,
    _joint_world_pose_from_body,
    _pose_equal,
)
from pxr import Gf, Sdf, Usd, UsdGeom, UsdPhysics

_JOINT_PATH = "/Robot/joints/joint1"
#: Joint path used in the programmatically-built test stages.

_BODY0_PATH = "/Robot/link0"
#: Path to the first rigid body link.

_BODY1_PATH = "/Robot/link1"
#: Path to the second rigid body link.

_ROT_30Y = Gf.Quatf(0.9659258, 0.0, 0.258819, 0.0)
#: Non-trivial rotation (~30 deg around Y) for realistic tests.


def _build_test_stage(path: str) -> Usd.Stage:
    """Build a minimal stage with two rigid-body links and one well-formed revolute joint.

    ``localPos1`` / ``localRot1`` are derived from the body0 side so that
    ``local0 * body0_world == local1 * body1_world`` (both bodies agree on the
    joint world pose).

    Hierarchy::

        /Robot  (Xform, default prim)
          /Robot/link0  (Xform, RigidBodyAPI) — rotated by ``_ROT_30Y``
          /Robot/link1  (Xform, RigidBodyAPI) — translate (0, 0.5, 0.8)
          /Robot/joints/joint1  (RevoluteJoint)
            body0 -> /Robot/link0   localPos0=(0, 0.5, 0.8)  localRot0=(1,0,0,0)
            body1 -> /Robot/link1   localPos1/localRot1 computed for consistency

    Args:
        path: File path for the new ``.usda`` stage.

    Returns:
        The created and saved USD stage.

    """
    stage = Usd.Stage.CreateNew(path)
    robot = UsdGeom.Xform.Define(stage, "/Robot")
    stage.SetDefaultPrim(robot.GetPrim())

    link0 = UsdGeom.Xform.Define(stage, _BODY0_PATH)
    UsdPhysics.RigidBodyAPI.Apply(link0.GetPrim())
    link0.AddOrientOp().Set(_ROT_30Y)

    link1 = UsdGeom.Xform.Define(stage, _BODY1_PATH)
    UsdPhysics.RigidBodyAPI.Apply(link1.GetPrim())
    link1.AddTranslateOp().Set(Gf.Vec3d(0, 0.5, 0.8))

    UsdGeom.Xform.Define(stage, "/Robot/joints")
    joint = UsdPhysics.RevoluteJoint.Define(stage, _JOINT_PATH)
    joint.CreateBody0Rel().SetTargets([Sdf.Path(_BODY0_PATH)])
    joint.CreateBody1Rel().SetTargets([Sdf.Path(_BODY1_PATH)])

    local_pos_0 = Gf.Vec3f(0, 0.5, 0.8)
    local_rot_0 = Gf.Quatf(1, 0, 0, 0)
    joint.CreateLocalPos0Attr().Set(local_pos_0)
    joint.CreateLocalRot0Attr().Set(local_rot_0)

    time = Usd.TimeCode.Default()
    body0_w = Gf.Matrix4d(UsdGeom.Xformable(link0.GetPrim()).ComputeLocalToWorldTransform(time))
    body1_w = Gf.Matrix4d(UsdGeom.Xformable(link1.GetPrim()).ComputeLocalToWorldTransform(time))
    local0_mat = Gf.Matrix4d()
    local0_mat.SetTranslate(Gf.Vec3d(local_pos_0))
    local0_mat.SetRotateOnly(Gf.Quatd(local_rot_0.GetReal(), *local_rot_0.GetImaginary()))
    joint_world = local0_mat * body0_w
    local1_mat = joint_world * body1_w.GetInverse()
    t1 = local1_mat.ExtractTranslation()
    q1 = local1_mat.ExtractRotationQuat()
    joint.CreateLocalPos1Attr().Set(Gf.Vec3f(t1))
    joint.CreateLocalRot1Attr().Set(Gf.Quatf(q1.GetReal(), *q1.GetImaginary()))

    joint.CreateAxisAttr().Set("Z")

    stage.Save()
    return stage


def _build_mismatched_joint_stage(path: str) -> Usd.Stage:
    """Build a stage where body0 and body1 intentionally disagree on joint world pose.

    Same hierarchy as ``_build_test_stage`` but the local-pose attributes on
    the joint are chosen so that ``local0 * body0_world != local1 * body1_world``.
    This simulates an asset where the joint definition is already inconsistent
    between body0 and body1 in the source data.

    Args:
        path: File path for the new ``.usda`` stage.

    Returns:
        The created and saved USD stage.

    """
    stage = Usd.Stage.CreateNew(path)
    robot = UsdGeom.Xform.Define(stage, "/Robot")
    stage.SetDefaultPrim(robot.GetPrim())

    link0 = UsdGeom.Xform.Define(stage, _BODY0_PATH)
    UsdPhysics.RigidBodyAPI.Apply(link0.GetPrim())
    link0.AddOrientOp().Set(_ROT_30Y)

    link1 = UsdGeom.Xform.Define(stage, _BODY1_PATH)
    UsdPhysics.RigidBodyAPI.Apply(link1.GetPrim())
    link1.AddTranslateOp().Set(Gf.Vec3d(0, 0.5, 0.8))

    UsdGeom.Xform.Define(stage, "/Robot/joints")
    joint = UsdPhysics.RevoluteJoint.Define(stage, _JOINT_PATH)
    joint.CreateBody0Rel().SetTargets([Sdf.Path(_BODY0_PATH)])
    joint.CreateBody1Rel().SetTargets([Sdf.Path(_BODY1_PATH)])

    joint.CreateLocalPos0Attr().Set(Gf.Vec3f(0, 0.5, 0.8))
    joint.CreateLocalRot0Attr().Set(Gf.Quatf(1, 0, 0, 0))

    # Deliberately inconsistent with body0 side.
    joint.CreateLocalPos1Attr().Set(Gf.Vec3f(0.4, -0.2, 0.15))
    joint.CreateLocalRot1Attr().Set(Gf.Quatf(0.707, 0, 0.707, 0))

    joint.CreateAxisAttr().Set("Z")
    stage.Save()
    return stage


def _assert_joint_world_matches_original(
    test_case: omni.kit.test.AsyncTestCase,
    original_stage: Usd.Stage,
    entry_stage: Usd.Stage,
    joint_path: str = _JOINT_PATH,
    tol_pos: float = 1e-4,
    tol_orient: float = 1e-4,
) -> None:
    """Assert that joint world pose from both body sides on the entry stage matches the original.

    Args:
        test_case: The running test case (for assertion methods).
        original_stage: The unmodified original stage.
        entry_stage: The stage after the rule has run.
        joint_path: USD path to the joint prim.
        tol_pos: Position tolerance.
        tol_orient: Orientation tolerance.

    """
    time = Usd.TimeCode.Default()
    orig_joint = original_stage.GetPrimAtPath(joint_path)
    entry_joint = entry_stage.GetPrimAtPath(joint_path)
    test_case.assertTrue(orig_joint.IsValid(), f"Original joint not found: {joint_path}")
    test_case.assertTrue(entry_joint.IsValid(), f"Entry joint not found: {joint_path}")

    for body_idx in (0, 1):
        orig_world = _joint_world_pose_from_body(original_stage, orig_joint, body_idx, time)
        entry_world = _joint_world_pose_from_body(entry_stage, entry_joint, body_idx, time)
        test_case.assertIsNotNone(orig_world, f"Original joint world from body{body_idx} is None")
        test_case.assertIsNotNone(entry_world, f"Entry joint world from body{body_idx} is None")
        test_case.assertTrue(
            _pose_equal(orig_world, entry_world, tol_pos, tol_orient),
            f"Joint world from body{body_idx} mismatch after fix:\n"
            f"  orig  = {orig_world.ExtractTranslation()}\n"
            f"  entry = {entry_world.ExtractTranslation()}",
        )


class TestPhysicsJointPoseFixRule(omni.kit.test.AsyncTestCase):
    """Async tests for PhysicsJointPoseFixRule."""

    async def setUp(self) -> None:
        """Create a temporary directory for test output."""
        self._tmpdir = tempfile.mkdtemp()
        self._success = False

    async def tearDown(self) -> None:
        """Remove temporary directory after successful tests."""
        if self._success:
            shutil.rmtree(self._tmpdir, ignore_errors=True)

    def _create_rule(
        self,
        stage: Usd.Stage,
        input_stage_path: str,
        params: dict | None = None,
    ) -> PhysicsJointPoseFixRule:
        """Create a rule instance wired to the given stage and original path.

        Args:
            stage: Entry-point USD stage.
            input_stage_path: Path to the original composition.
            params: Optional rule parameters.

        Returns:
            Configured ``PhysicsJointPoseFixRule`` instance.

        """
        return PhysicsJointPoseFixRule(
            source_stage=stage,
            package_root=self._tmpdir,
            destination_path="payloads",
            args={"params": params or {}, "input_stage_path": input_stage_path},
        )

    # ------------------------------------------------------------------
    # Basic parameter / skip tests
    # ------------------------------------------------------------------

    async def test_get_configuration_parameters(self) -> None:
        """Verify configuration parameters are exposed."""
        stage = Usd.Stage.CreateInMemory()
        rule = PhysicsJointPoseFixRule(
            source_stage=stage,
            package_root=self._tmpdir,
            destination_path="",
            args={"input_stage_path": "/nonexistent.usda"},
        )
        param_names = [p.name for p in rule.get_configuration_parameters()]
        self.assertIn("original_composition_path", param_names)
        self.assertIn("tolerance_position", param_names)
        self.assertIn("tolerance_orientation", param_names)
        self._success = True

    async def test_process_rule_skips_without_input_stage_path(self) -> None:
        """Verify rule skips when no original composition path is provided."""
        stage = Usd.Stage.CreateInMemory()
        rule = PhysicsJointPoseFixRule(
            source_stage=stage, package_root=self._tmpdir, destination_path="", args={"params": {}}
        )
        rule.process_rule()
        log = rule.get_operation_log()
        self.assertTrue(any("no original composition path" in m.lower() for m in log))
        self._success = True

    async def test_process_rule_skips_when_original_stage_fails_to_open(self) -> None:
        """Verify rule skips when original stage path is invalid."""
        original_path = os.path.join(self._tmpdir, "original.usda")
        entry_path = os.path.join(self._tmpdir, "entry.usda")
        _build_test_stage(original_path)
        stage = _build_test_stage(entry_path)
        rule = self._create_rule(stage, os.path.join(self._tmpdir, "nonexistent.usda"))
        rule.process_rule()
        log = rule.get_operation_log()
        self.assertTrue(any("could not open original stage" in m.lower() for m in log))
        self._success = True

    async def test_process_rule_no_joints_on_entry(self) -> None:
        """Verify rule logs when entry stage has no physics joints."""
        empty_path = os.path.join(self._tmpdir, "empty.usda")
        stage = Usd.Stage.CreateNew(empty_path)
        UsdGeom.Xform.Define(stage, "/World")
        stage.SetDefaultPrim(stage.GetPrimAtPath("/World"))
        stage.Save()
        rule = self._create_rule(stage, empty_path)
        rule.process_rule()
        log = rule.get_operation_log()
        self.assertTrue(any("no physics joints found" in m.lower() for m in log))
        self._success = True

    async def test_process_rule_identical_stages_no_correction(self) -> None:
        """Verify rule applies no correction when original and entry are identical."""
        original_path = os.path.join(self._tmpdir, "original.usda")
        entry_path = os.path.join(self._tmpdir, "entry.usda")
        _build_test_stage(original_path)
        entry_stage = _build_test_stage(entry_path)
        rule = self._create_rule(entry_stage, original_path)
        rule.process_rule()
        log = rule.get_operation_log()
        completed = [m for m in log if "completed" in m and "corrected" in m]
        self.assertTrue(completed, f"Expected completion log: {log}")
        self.assertIn("0 joint body pose(s) corrected", completed[0])
        self._success = True

    # ------------------------------------------------------------------
    # Body0-only differs
    # ------------------------------------------------------------------

    async def test_fix_body0_only(self) -> None:
        """When only body0's world transform changes, only body0 local pose is fixed."""
        original_path = os.path.join(self._tmpdir, "original.usda")
        entry_path = os.path.join(self._tmpdir, "entry.usda")
        _build_test_stage(original_path)
        entry_stage = _build_test_stage(entry_path)

        link0 = entry_stage.GetPrimAtPath(_BODY0_PATH)
        xf = UsdGeom.Xformable(link0)
        xf.ClearXformOpOrder()
        xf.AddTranslateOp().Set(Gf.Vec3d(0.2, 0, 0))
        xf.AddOrientOp().Set(Gf.Quatf(0.707, 0.707, 0, 0))
        entry_stage.Save()

        rule = self._create_rule(entry_stage, original_path)
        rule.process_rule()

        log = rule.get_operation_log()
        self.assertTrue(any("FIX body0" in m for m in log), f"Expected body0 fix: {log}")
        self.assertFalse(any("FIX body1" in m for m in log), f"Unexpected body1 fix: {log}")

        original_stage = Usd.Stage.Open(original_path)
        _assert_joint_world_matches_original(self, original_stage, entry_stage)
        self._success = True

    # ------------------------------------------------------------------
    # Body1-only differs
    # ------------------------------------------------------------------

    async def test_fix_body1_only(self) -> None:
        """When only body1's world transform changes, only body1 local pose is fixed."""
        original_path = os.path.join(self._tmpdir, "original.usda")
        entry_path = os.path.join(self._tmpdir, "entry.usda")
        _build_test_stage(original_path)
        entry_stage = _build_test_stage(entry_path)

        link1 = entry_stage.GetPrimAtPath(_BODY1_PATH)
        xf = UsdGeom.Xformable(link1)
        xf.ClearXformOpOrder()
        xf.AddTranslateOp().Set(Gf.Vec3d(0, 0.5, 0.8))
        xf.AddOrientOp().Set(Gf.Quatf(0.5, 0.5, -0.5, 0.5))
        entry_stage.Save()

        rule = self._create_rule(entry_stage, original_path)
        rule.process_rule()

        log = rule.get_operation_log()
        self.assertFalse(any("FIX body0" in m for m in log), f"Unexpected body0 fix: {log}")
        self.assertTrue(any("FIX body1" in m for m in log), f"Expected body1 fix: {log}")

        original_stage = Usd.Stage.Open(original_path)
        _assert_joint_world_matches_original(self, original_stage, entry_stage)
        self._success = True

    # ------------------------------------------------------------------
    # Both body0 and body1 differ
    # ------------------------------------------------------------------

    async def test_fix_both_bodies(self) -> None:
        """When both body0 and body1 world transforms change, both local poses are fixed."""
        original_path = os.path.join(self._tmpdir, "original.usda")
        entry_path = os.path.join(self._tmpdir, "entry.usda")
        _build_test_stage(original_path)
        entry_stage = _build_test_stage(entry_path)

        link0 = entry_stage.GetPrimAtPath(_BODY0_PATH)
        xf0 = UsdGeom.Xformable(link0)
        xf0.ClearXformOpOrder()
        xf0.AddTranslateOp().Set(Gf.Vec3d(0.15, -0.1, 0.05))
        xf0.AddOrientOp().Set(Gf.Quatf(0.707, 0, 0, 0.707))

        link1 = entry_stage.GetPrimAtPath(_BODY1_PATH)
        xf1 = UsdGeom.Xformable(link1)
        xf1.ClearXformOpOrder()
        xf1.AddTranslateOp().Set(Gf.Vec3d(-0.1, 0.6, 0.9))
        xf1.AddOrientOp().Set(Gf.Quatf(0.5, 0.5, 0.5, 0.5))

        entry_stage.Save()

        rule = self._create_rule(entry_stage, original_path)
        rule.process_rule()

        log = rule.get_operation_log()
        self.assertTrue(any("FIX body0" in m for m in log), f"Expected body0 fix: {log}")
        self.assertTrue(any("FIX body1" in m for m in log), f"Expected body1 fix: {log}")

        original_stage = Usd.Stage.Open(original_path)
        _assert_joint_world_matches_original(self, original_stage, entry_stage)
        self._success = True

    # ------------------------------------------------------------------
    # Original stage already has body0/body1 mismatch — preserve it
    # ------------------------------------------------------------------

    async def test_preserves_original_body_mismatch(self) -> None:
        """When the original joint has body0 and body1 that disagree, the rule preserves that mismatch.

        Each body side is fixed independently so that its joint world pose
        matches the original for that side.  The mismatch between body0 and
        body1 in the original must be equally present in the result.
        """
        original_path = os.path.join(self._tmpdir, "mismatch_orig.usda")
        entry_path = os.path.join(self._tmpdir, "mismatch_entry.usda")

        _build_mismatched_joint_stage(original_path)
        _build_mismatched_joint_stage(entry_path)

        original_stage = Usd.Stage.Open(original_path)
        time = Usd.TimeCode.Default()
        orig_joint = original_stage.GetPrimAtPath(_JOINT_PATH)
        orig_w0 = _joint_world_pose_from_body(original_stage, orig_joint, 0, time)
        orig_w1 = _joint_world_pose_from_body(original_stage, orig_joint, 1, time)
        self.assertIsNotNone(orig_w0)
        self.assertIsNotNone(orig_w1)
        self.assertFalse(
            _pose_equal(orig_w0, orig_w1, tol_pos=1e-4, tol_orient=1e-4),
            "Test setup error: original body0 and body1 should disagree",
        )

        entry_stage = Usd.Stage.Open(entry_path)

        link0 = entry_stage.GetPrimAtPath(_BODY0_PATH)
        xf0 = UsdGeom.Xformable(link0)
        xf0.ClearXformOpOrder()
        xf0.AddTranslateOp().Set(Gf.Vec3d(0.3, 0, 0))
        xf0.AddOrientOp().Set(Gf.Quatf(0.707, 0.707, 0, 0))

        link1 = entry_stage.GetPrimAtPath(_BODY1_PATH)
        xf1 = UsdGeom.Xformable(link1)
        xf1.ClearXformOpOrder()
        xf1.AddTranslateOp().Set(Gf.Vec3d(0, 0.7, 1.0))
        xf1.AddOrientOp().Set(Gf.Quatf(0.5, 0.5, -0.5, 0.5))

        entry_stage.Save()

        rule = self._create_rule(entry_stage, original_path)
        rule.process_rule()

        _assert_joint_world_matches_original(self, original_stage, entry_stage)

        entry_joint = entry_stage.GetPrimAtPath(_JOINT_PATH)
        entry_w0 = _joint_world_pose_from_body(entry_stage, entry_joint, 0, time)
        entry_w1 = _joint_world_pose_from_body(entry_stage, entry_joint, 1, time)
        self.assertFalse(
            _pose_equal(entry_w0, entry_w1, tol_pos=1e-4, tol_orient=1e-4),
            "Body0 and body1 should still disagree after fix (mismatch preserved)",
        )
        self._success = True


class TestPhysicsJointPoseFixHelpers(omni.kit.test.AsyncTestCase):
    """Tests for module-level helpers used by the rule."""

    async def test_pose_equal_identical(self) -> None:
        """Verify ``_pose_equal`` returns True for identical poses."""
        m = Gf.Matrix4d(1.0)
        m.SetTranslate(Gf.Vec3d(1, 2, 3))
        m.SetRotateOnly(Gf.Quatd(0.707, 0.707, 0, 0))
        self.assertTrue(_pose_equal(m, m))

    async def test_pose_equal_different_translation(self) -> None:
        """Verify ``_pose_equal`` returns False when translation differs beyond tolerance."""
        a = Gf.Matrix4d(1.0)
        a.SetTranslate(Gf.Vec3d(0, 0, 0))
        b = Gf.Matrix4d(1.0)
        b.SetTranslate(Gf.Vec3d(1e-5, 0, 0))
        self.assertFalse(_pose_equal(a, b, tol_pos=1e-6))

    async def test_pose_equal_within_tolerance(self) -> None:
        """Verify ``_pose_equal`` returns True when within position and orientation tolerance."""
        a = Gf.Matrix4d(1.0)
        a.SetTranslate(Gf.Vec3d(0, 0, 0))
        b = Gf.Matrix4d(1.0)
        b.SetTranslate(Gf.Vec3d(1e-8, 0, 0))
        self.assertTrue(_pose_equal(a, b, tol_pos=1e-6))

    async def test_joint_world_pose_from_body_both_agree(self) -> None:
        """Body0 and body1 produce the same joint world pose on a well-formed stage."""
        path = os.path.join(tempfile.mkdtemp(), "agree.usda")
        _build_test_stage(path)
        stage = Usd.Stage.Open(path)
        time = Usd.TimeCode.Default()
        joint_prim = stage.GetPrimAtPath(_JOINT_PATH)
        w0 = _joint_world_pose_from_body(stage, joint_prim, 0, time)
        w1 = _joint_world_pose_from_body(stage, joint_prim, 1, time)
        self.assertIsNotNone(w0)
        self.assertIsNotNone(w1)
        self.assertTrue(
            _pose_equal(w0, w1, tol_pos=1e-4, tol_orient=1e-4),
            f"Body0 and body1 disagree on joint world:\n"
            f"  body0={w0.ExtractTranslation()}\n"
            f"  body1={w1.ExtractTranslation()}",
        )
        shutil.rmtree(os.path.dirname(path), ignore_errors=True)
