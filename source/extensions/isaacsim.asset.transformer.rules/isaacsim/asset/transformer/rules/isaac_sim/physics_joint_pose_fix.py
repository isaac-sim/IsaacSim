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

"""Correct physics joint local poses when body world transforms change.

After the Geometries rule moves meshes and may combine parent/child transforms,
joint body relationships can end up with incorrect local poses. This rule
computes the joint world pose from both ``body0`` and ``body1`` on the original
composition and on the entry-point stage. When either side differs, it updates
that body's local pose so the joint world pose matches the original.

How the manager passes stages to this rule:

- Entry stage (``self.source_stage``): The current working stage, i.e. the
  output of the previous rule (e.g. Geometries). Its content already has
  modified transforms (references, combined parent/child transforms, etc.).
  The manager passes this as the first constructor argument (``source_stage``).
- Original composition: The manager does *not* pass a separate
  "pre-Geometries" stage. It passes the original input asset path via
  ``args["input_stage_path"]``: that is the path given to
  ``manager.run(input_stage, ...)``, i.e. the asset on disk before any
  rules ran. The pipeline writes to ``package_root`` (e.g.
  ``payloads/base.usd``); it does not overwrite ``input_stage_path``. So this
  rule opens ``Usd.Stage.Open(args["input_stage_path"])`` to obtain the
  unmodified original and compares joint world poses against the entry stage.
"""

from __future__ import annotations

import os

from isaacsim.asset.transformer import RuleConfigurationParam, RuleInterface
from pxr import Gf, Sdf, Usd, UsdGeom, UsdPhysics

_DEFAULT_TOLERANCE_POSITION: float = 1.0e-6
_DEFAULT_TOLERANCE_ORIENT: float = 1.0e-6


def _fmt(m: Gf.Matrix4d | None) -> str:
    """Format a matrix as translation + quaternion for debug logging.

    Args:
        m: 4x4 rigid transform matrix, or None.

    Returns:
        Human-readable string with translation and quaternion components.

    """
    if m is None:
        return "None"
    t = m.ExtractTranslation()
    q = m.ExtractRotationQuat()
    im = q.GetImaginary()
    return f"t=({t[0]:.8f}, {t[1]:.8f}, {t[2]:.8f}) " f"q=({q.GetReal():.8f}, {im[0]:.8f}, {im[1]:.8f}, {im[2]:.8f})"


def _get_joint_local_transform(joint: UsdPhysics.Joint, body_index: int) -> Gf.Matrix4d | None:
    """Build the joint-local transform matrix for the given body.

    Reads ``localPos0/1`` and ``localRot0/1`` from the joint and constructs a
    ``Gf.Matrix4d`` with the same convention as
    ``robot_schema.utils._get_joint_local_transform``.

    Args:
        joint: The USD physics joint schema.
        body_index: 0 for body0, 1 for body1.

    Returns:
        4x4 local transform matrix, or None if attributes are missing.

    """
    translate_attr = joint.GetLocalPos0Attr() if body_index == 0 else joint.GetLocalPos1Attr()
    rotate_attr = joint.GetLocalRot0Attr() if body_index == 0 else joint.GetLocalRot1Attr()
    if not translate_attr or not rotate_attr:
        return None
    translate = translate_attr.Get()
    rotate = rotate_attr.Get()
    if translate is None or rotate is None:
        return None
    local = Gf.Matrix4d()
    local.SetTranslate(Gf.Vec3d(translate))
    local.SetRotateOnly(Gf.Quatd(rotate.GetReal(), *rotate.GetImaginary()).GetNormalized())
    return local


def _joint_world_pose_from_body(
    stage: Usd.Stage,
    joint_prim: Usd.Prim,
    body_index: int,
    time: Usd.TimeCode,
) -> Gf.Matrix4d | None:
    """Compute joint world pose from one body side.

    Uses USD row-vector convention where local transforms go from joint space
    to body space and ``body_world`` transforms from body space to world space.
    For both bodies: ``joint_world = local * body_world``.

    Args:
        stage: The USD stage containing the joint.
        joint_prim: The joint prim to evaluate.
        body_index: 0 for body0, 1 for body1.
        time: Time code at which to evaluate transforms.

    Returns:
        4x4 joint world transform matrix, or None if the body is missing.

    """
    joint = UsdPhysics.Joint(joint_prim)
    rel = joint.GetBody0Rel() if body_index == 0 else joint.GetBody1Rel()
    targets = rel.GetTargets()
    if not targets:
        return None
    body_prim = stage.GetPrimAtPath(targets[0])
    if not body_prim or not body_prim.IsValid() or not UsdGeom.Xformable(body_prim):
        return None
    body_world = Gf.Matrix4d(UsdGeom.Xformable(body_prim).ComputeLocalToWorldTransform(time))
    local = _get_joint_local_transform(joint, body_index)
    if local is None:
        return None
    return local * body_world


def _decompose_rigid(m: Gf.Matrix4d) -> tuple[Gf.Vec3f, Gf.Quatf]:
    """Decompose a rigid 4x4 matrix into translation and rotation for USD joint attributes.

    Args:
        m: 4x4 rigid transform matrix.

    Returns:
        Tuple of (``Vec3f`` translation, ``Quatf`` rotation).

    """
    trans = Gf.Vec3f(m.ExtractTranslation())
    quat = m.ExtractRotationQuat().GetNormalized()
    return trans, Gf.Quatf(quat.GetReal(), quat.GetImaginary()[0], quat.GetImaginary()[1], quat.GetImaginary()[2])


def _pose_equal(
    a: Gf.Matrix4d,
    b: Gf.Matrix4d,
    tol_pos: float = _DEFAULT_TOLERANCE_POSITION,
    tol_orient: float = _DEFAULT_TOLERANCE_ORIENT,
) -> bool:
    """Return True if two rigid poses are equal within tolerance.

    Position is compared by Euclidean distance, orientation by quaternion dot
    product.

    Args:
        a: First rigid transform.
        b: Second rigid transform.
        tol_pos: Maximum allowed position difference.
        tol_orient: Minimum quaternion dot product deviation from 1.0.

    Returns:
        True if the poses match within tolerance.

    """
    ta = a.ExtractTranslation()
    tb = b.ExtractTranslation()
    if (Gf.Vec3d(ta) - Gf.Vec3d(tb)).GetLength() > tol_pos:
        return False
    qa = a.ExtractRotationQuat().GetNormalized()
    qb = b.ExtractRotationQuat().GetNormalized()
    dot = abs(qa.GetReal() * qb.GetReal() + Gf.Dot(qa.GetImaginary(), qb.GetImaginary()))
    return dot >= 1.0 - tol_orient


class PhysicsJointPoseFixRule(RuleInterface):
    """Fix physics joint body local poses so joint world pose matches the original composition.

    Scans the physics layer (entry-point stage) for joints, computes joint
    world pose from the original composition (``input_stage_path``) and from
    the entry stage. When the entry stage's joint world pose differs from the
    original, computes the corrective local pose for the affected body
    (``body0`` or ``body1``) and sets ``physics:localPos0/localRot0`` or
    ``physics:localPos1/localRot1`` on the joint.
    """

    def get_configuration_parameters(self) -> list[RuleConfigurationParam]:
        """Return configuration parameters for this rule.

        Returns:
            List of configuration parameter descriptors.

        """
        return [
            RuleConfigurationParam(
                name="original_composition_path",
                display_name="Original Composition Path",
                param_type=str,
                description=(
                    "Optional path to the original composition stage. "
                    "Defaults to input_stage_path from the transformer manager."
                ),
                default_value=None,
            ),
            RuleConfigurationParam(
                name="tolerance_position",
                display_name="Tolerance Position",
                param_type=float,
                description="Position tolerance when comparing joint world poses.",
                default_value=_DEFAULT_TOLERANCE_POSITION,
            ),
            RuleConfigurationParam(
                name="tolerance_orientation",
                display_name="Tolerance Orientation",
                param_type=float,
                description="Orientation tolerance (quaternion dot) when comparing joint world poses.",
                default_value=_DEFAULT_TOLERANCE_ORIENT,
            ),
        ]

    def process_rule(self) -> str | None:
        """Scan joints on the entry stage, compare world poses to the original, and fix local poses where they differ.

        Original composition is read from ``args["input_stage_path"]`` (the
        path passed to the transformer manager at run start; unmodified by the
        pipeline). Entry stage is ``self.source_stage`` (current working stage,
        e.g. after Geometries).

        Returns:
            Always None.

        """
        params = self.args.get("params", {}) or {}
        original_path = params.get("original_composition_path") or self.args.get("input_stage_path")
        if not original_path:
            self.log_operation("PhysicsJointPoseFixRule skipped: no original composition path (input_stage_path)")
            return None

        tol_pos = float(params.get("tolerance_position", _DEFAULT_TOLERANCE_POSITION))
        tol_orient = float(params.get("tolerance_orientation", _DEFAULT_TOLERANCE_ORIENT))

        try:
            original_stage = self.args.get("input_stage") or Usd.Stage.Open(original_path)
        except Exception:
            original_stage = None
        if not original_stage:
            self.log_operation(f"PhysicsJointPoseFixRule skipped: could not open original stage: {original_path}")
            return None

        entry_stage = self.source_stage
        time = Usd.TimeCode.Default()

        joint_prims = [prim for prim in entry_stage.Traverse() if prim.IsA(UsdPhysics.Joint)]
        if not joint_prims:
            self.log_operation("PhysicsJointPoseFixRule: no physics joints found on entry stage")
            return None

        self.log_operation(
            f"PhysicsJointPoseFixRule start original={original_path} joints={len(joint_prims)} "
            f"tol_pos={tol_pos} tol_orient={tol_orient}"
        )

        fixed_count = 0
        edit_layer = entry_stage.GetEditTarget().GetLayer()
        if not edit_layer:
            edit_layer = entry_stage.GetRootLayer()

        for joint_prim in joint_prims:
            fixed_count += self._process_joint(joint_prim, original_stage, entry_stage, time, tol_pos, tol_orient)

        if edit_layer and edit_layer.dirty:
            edit_path = getattr(edit_layer, "realPath", None) or getattr(edit_layer, "identifier", None)
            if edit_path and os.path.isfile(edit_path):
                edit_layer.Save()
            self.add_affected_stage(edit_path or str(edit_layer.identifier))

        self.log_operation(f"PhysicsJointPoseFixRule completed: {fixed_count} joint body pose(s) corrected")
        return None

    def _process_joint(
        self,
        joint_prim: Usd.Prim,
        original_stage: Usd.Stage,
        entry_stage: Usd.Stage,
        time: Usd.TimeCode,
        tol_pos: float,
        tol_orient: float,
    ) -> int:
        """Evaluate and fix a single joint's local poses.

        Args:
            joint_prim: The joint prim on the entry stage.
            original_stage: The unmodified original USD stage.
            entry_stage: The current working stage (post-Geometries).
            time: Time code for transform evaluation.
            tol_pos: Position tolerance.
            tol_orient: Orientation tolerance.

        Returns:
            Number of body sides fixed (0, 1, or 2).

        """
        path_str = joint_prim.GetPath().pathString
        orig_joint = original_stage.GetPrimAtPath(joint_prim.GetPath())
        if not orig_joint or not orig_joint.IsValid() or not orig_joint.IsA(UsdPhysics.Joint):
            return 0

        joint_schema = UsdPhysics.Joint(joint_prim)

        orig_local0 = _get_joint_local_transform(UsdPhysics.Joint(orig_joint), 0)
        orig_local1 = _get_joint_local_transform(UsdPhysics.Joint(orig_joint), 1)
        entry_local0 = _get_joint_local_transform(joint_schema, 0)
        entry_local1 = _get_joint_local_transform(joint_schema, 1)

        body0_targets = joint_schema.GetBody0Rel().GetTargets()
        body1_targets = joint_schema.GetBody1Rel().GetTargets()
        body0_path = body0_targets[0] if body0_targets else None
        body1_path = body1_targets[0] if body1_targets else None
        if not body0_path and not body1_path:
            return 0

        body0_world_orig = self._body_world_transform(original_stage, body0_path, time)
        body1_world_orig = self._body_world_transform(original_stage, body1_path, time)
        body0_world = self._body_world_transform(entry_stage, body0_path, time)
        body1_world = self._body_world_transform(entry_stage, body1_path, time)

        joint_world_orig_0 = _joint_world_pose_from_body(original_stage, orig_joint, 0, time)
        joint_world_orig_1 = _joint_world_pose_from_body(original_stage, orig_joint, 1, time)
        joint_world_entry_0 = _joint_world_pose_from_body(entry_stage, joint_prim, 0, time)
        joint_world_entry_1 = _joint_world_pose_from_body(entry_stage, joint_prim, 1, time)

        fix_body0 = (
            body0_world is not None
            and joint_world_orig_0 is not None
            and (
                joint_world_entry_0 is None
                or not _pose_equal(joint_world_orig_0, joint_world_entry_0, tol_pos, tol_orient)
            )
        )
        fix_body1 = (
            body1_world is not None
            and joint_world_orig_1 is not None
            and (
                joint_world_entry_1 is None
                or not _pose_equal(joint_world_orig_1, joint_world_entry_1, tol_pos, tol_orient)
            )
        )

        # Compose: joint_world = local * body_world (both bodies, row-vector convention).
        # Solve: new_local = joint_world_orig * inv(body_world).
        fixed = 0
        if fix_body0:
            new_local0 = joint_world_orig_0 * body0_world.GetInverse()
            trans0, quat0 = _decompose_rigid(new_local0)
            joint_schema.CreateLocalPos0Attr().Set(trans0)
            joint_schema.CreateLocalRot0Attr().Set(quat0)
            fixed += 1
            self._log_body_fix(
                path_str,
                0,
                body0_path,
                body1_path,
                body0_world_orig,
                orig_local0,
                joint_world_orig_0,
                body0_world,
                entry_local0,
                joint_world_entry_0,
                new_local0,
                trans0,
                quat0,
            )

        if fix_body1:
            new_local1 = joint_world_orig_1 * body1_world.GetInverse()
            trans1, quat1 = _decompose_rigid(new_local1)
            joint_schema.CreateLocalPos1Attr().Set(trans1)
            joint_schema.CreateLocalRot1Attr().Set(quat1)
            fixed += 1
            self._log_body_fix(
                path_str,
                1,
                body0_path,
                body1_path,
                body1_world_orig,
                orig_local1,
                joint_world_orig_1,
                body1_world,
                entry_local1,
                joint_world_entry_1,
                new_local1,
                trans1,
                quat1,
            )

        return fixed

    @staticmethod
    def _body_world_transform(stage: Usd.Stage, body_path: Sdf.Path | None, time: Usd.TimeCode) -> Gf.Matrix4d | None:
        """Compute the world transform for a body prim if it exists.

        Args:
            stage: USD stage containing the body.
            body_path: Sdf path to the body prim, or None.
            time: Time code for evaluation.

        Returns:
            4x4 world transform matrix, or None if the prim is invalid.

        """
        if body_path is None:
            return None
        prim = stage.GetPrimAtPath(body_path)
        if not prim or not prim.IsValid() or not UsdGeom.Xformable(prim):
            return None
        return Gf.Matrix4d(UsdGeom.Xformable(prim).ComputeLocalToWorldTransform(time))

    def _log_body_fix(
        self,
        path_str: str,
        body_index: int,
        body0_path: Sdf.Path | None,
        body1_path: Sdf.Path | None,
        body_world_orig: Gf.Matrix4d | None,
        orig_local: Gf.Matrix4d | None,
        joint_world_orig: Gf.Matrix4d,
        body_world: Gf.Matrix4d,
        entry_local: Gf.Matrix4d | None,
        joint_world_entry: Gf.Matrix4d | None,
        new_local: Gf.Matrix4d,
        trans: Gf.Vec3f,
        quat: Gf.Quatf,
    ) -> None:
        """Log debug information for a body-side fix.

        Args:
            path_str: Joint prim path string.
            body_index: 0 or 1.
            body0_path: Path to body0.
            body1_path: Path to body1.
            body_world_orig: Body world transform on the original stage.
            orig_local: Original joint local transform.
            joint_world_orig: Original joint world pose.
            body_world: Body world transform on the entry stage.
            entry_local: Entry joint local transform.
            joint_world_entry: Entry joint world pose.
            new_local: Computed corrective local transform.
            trans: New translation value.
            quat: New rotation value.

        """
        tag = f"body{body_index}"
        self.log_operation(
            f"DEBUG {path_str}:\n"
            f"  body0_path={body0_path}  body1_path={body1_path}\n"
            f"  ORIGINAL:\n"
            f"    {tag}_world_orig:    {_fmt(body_world_orig)}\n"
            f"    local{body_index}_orig:         {_fmt(orig_local)}\n"
            f"    joint_world_orig_{body_index}:  {_fmt(joint_world_orig)}\n"
            f"  ENTRY (post-Geometries):\n"
            f"    {tag}_world_entry:   {_fmt(body_world)}\n"
            f"    local{body_index}_entry:        {_fmt(entry_local)}\n"
            f"    joint_world_entry_{body_index}: {_fmt(joint_world_entry)}"
        )
        self.log_operation(
            f"  FIX {tag} for {path_str}:\n"
            f"    inv({tag}_world):  {_fmt(body_world.GetInverse())}\n"
            f"    new_local{body_index}:        {_fmt(new_local)}\n"
            f"    trans{body_index}={trans}  quat{body_index}={quat}\n"
            f"    verify: new_local{body_index} * {tag}_world = {_fmt(new_local * body_world)}"
        )
