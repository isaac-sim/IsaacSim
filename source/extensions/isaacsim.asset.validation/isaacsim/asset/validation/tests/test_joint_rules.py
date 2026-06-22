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

"""Tests for joint validation rules (JointHasCorrectTransformAndState, quaternion double-cover)."""

from __future__ import annotations

import tempfile

import omni.kit.test
from isaacsim.asset.validation.joint_rules import (
    JointHasCorrectTransformAndState,
    _rotation_error_magnitude,
    _rotations_match_orientation,
)
from pxr import Gf, PhysxSchema, Sdf, Usd, UsdGeom, UsdPhysics

_JOINT_PATH = "/Robot/joints/joint1"
_BODY0_PATH = "/Robot/link0"
_BODY1_PATH = "/Robot/link1"
# Non-trivial rotation (~30 deg around Y) so body0 and body1 can yield antipodal quaternions.
_ROT_30Y = Gf.Quatf(0.9659258, 0.0, 0.258819, 0.0)


def _build_stage_antipodal_quaternions(stage: Usd.Stage) -> None:
    """Build a minimal stage where joint world rotation from body0 is q and from body1 is -q.

    Same physical joint pose (position and orientation), but the two body paths
    produce antipodal quaternions so the validator must not report a rotation error.

    Args:
        stage: Stage to populate with the antipodal quaternion test scene.
    """
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
    joint_world_from_body0 = local0_mat * body0_w
    # Joint world from body1: same translation, antipodal rotation.
    q0 = joint_world_from_body0.ExtractRotationQuat()
    m1 = Gf.Matrix4d()
    m1.SetTranslateOnly(joint_world_from_body0.ExtractTranslation())
    m1.SetRotateOnly(Gf.Quatd(-q0.GetReal(), -q0.GetImaginary()[0], -q0.GetImaginary()[1], -q0.GetImaginary()[2]))
    local1_mat = m1 * body1_w.GetInverse()
    t1 = local1_mat.ExtractTranslation()
    q1 = local1_mat.ExtractRotationQuat()
    joint.CreateLocalPos1Attr().Set(Gf.Vec3f(t1))
    joint.CreateLocalRot1Attr().Set(Gf.Quatf(q1.GetReal(), *q1.GetImaginary()))

    joint.CreateAxisAttr().Set("Z")
    joint_state = PhysxSchema.JointStateAPI.Apply(joint.GetPrim(), "angular")
    if joint_state.GetPositionAttr():
        joint_state.GetPositionAttr().Set(0.0)
    else:
        joint_state.CreatePositionAttr(0.0)


class _ErrorCapturingChecker(JointHasCorrectTransformAndState):
    """Subclass that captures _AddError messages for testing."""

    def __init__(self) -> None:
        super().__init__()
        self.errors: list[str] = []

    def _AddError(self, message: str, **kwargs: object) -> None:  # noqa: N802
        self.errors.append(message)
        super()._AddError(message, **kwargs)


class TestJointRulesQuaternionHelpers(omni.kit.test.AsyncTestCase):
    """Unit tests for quaternion double-cover helpers."""

    async def test_rotations_match_orientation_antipodal(self) -> None:
        """Antipodal quaternions (q and -q) are treated as the same orientation."""
        q = Gf.Quatd(0.707, 0.707, 0, 0)
        q_neg = Gf.Quatd(-0.707, -0.707, 0, 0)
        self.assertTrue(_rotations_match_orientation(q, q_neg, 1e-6))
        self.assertTrue(_rotations_match_orientation(q_neg, q, 1e-6))

    async def test_rotations_match_orientation_identical(self) -> None:
        """Identical quaternions match."""
        q = Gf.Quatd(0.9659258, 0.0, 0.258819, 0.0)
        self.assertTrue(_rotations_match_orientation(q, q, 1e-6))

    async def test_rotations_match_orientation_different(self) -> None:
        """Genuinely different rotations do not match."""
        q1 = Gf.Quatd(1, 0, 0, 0)
        q2 = Gf.Quatd(0.707, 0.707, 0, 0)  # 90 deg around X
        self.assertFalse(_rotations_match_orientation(q1, q2, 1e-3))

    async def test_rotation_error_magnitude_antipodal(self) -> None:
        """Antipodal quaternions give error magnitude 0."""
        q = Gf.Quatd(0.707, 0.707, 0, 0)
        q_neg = Gf.Quatd(-0.707, -0.707, 0, 0)
        self.assertAlmostEqual(_rotation_error_magnitude(q, q_neg), 0.0, delta=1e-9)


class TestJointHasCorrectTransformAndStateAntipodal(omni.kit.test.AsyncTestCase):
    """Test that JointHasCorrectTransformAndState does not report rotation errors for antipodal quaternions."""

    async def test_antipodal_quaternions_no_rotation_error_in_memory(self) -> None:
        """Stage built in memory with antipodal joint quaternions: validator reports no rotation error."""
        stage = Usd.Stage.CreateInMemory()
        _build_stage_antipodal_quaternions(stage)
        joint_prim = stage.GetPrimAtPath(_JOINT_PATH)
        self.assertTrue(joint_prim.IsValid(), "Joint prim should exist")

        checker = _ErrorCapturingChecker()
        checker.CheckPrim(joint_prim)

        rotation_errors = [
            e for e in checker.errors if "Rotation not well defined" in e or "state not matching robot pose" in e
        ]
        self.assertEqual(
            len(rotation_errors),
            0,
            f"Expected no rotation errors for antipodal-quaternion stage, got: {rotation_errors}",
        )

    async def test_antipodal_quaternions_no_rotation_error_temp_file(self) -> None:
        """Stage saved to temp file with antipodal joint quaternions: validator reports no rotation error."""
        with tempfile.NamedTemporaryFile(suffix=".usda", delete=False) as f:
            path = f.name
        try:
            stage = Usd.Stage.CreateNew(path)
            _build_stage_antipodal_quaternions(stage)
            stage.Save()
            stage = None  # close

            stage = Usd.Stage.Open(path)
            joint_prim = stage.GetPrimAtPath(_JOINT_PATH)
            self.assertTrue(joint_prim.IsValid(), "Joint prim should exist")

            checker = _ErrorCapturingChecker()
            checker.CheckPrim(joint_prim)

            rotation_errors = [
                e for e in checker.errors if "Rotation not well defined" in e or "state not matching robot pose" in e
            ]
            self.assertEqual(
                len(rotation_errors),
                0,
                f"Expected no rotation errors for antipodal-quaternion stage (from file), got: {rotation_errors}",
            )
        finally:
            import os

            try:
                os.unlink(path)
            except OSError:
                pass
