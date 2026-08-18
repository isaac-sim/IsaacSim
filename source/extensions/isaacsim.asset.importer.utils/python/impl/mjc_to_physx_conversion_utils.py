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

"""Utilities for converting MJCF actuator/joint data to PhysX schemas."""

from __future__ import annotations

import logging
import math
import os
from collections import defaultdict

from pxr import Gf, Sdf, Usd, UsdPhysics

from .physx_types import PhysxAttr, PhysxSchema

_logger = logging.getLogger(__name__)


_REVOLUTE_AXIS_TO_D6_TOKEN = {
    "X": UsdPhysics.Tokens.rotX,
    "Y": UsdPhysics.Tokens.rotY,
    "Z": UsdPhysics.Tokens.rotZ,
}

_PRISMATIC_AXIS_TO_D6_TOKEN = {
    "X": UsdPhysics.Tokens.transX,
    "Y": UsdPhysics.Tokens.transY,
    "Z": UsdPhysics.Tokens.transZ,
}


def convert_mjc_to_physx(stage: Usd.Stage) -> None:
    """Convert all MJCF actuators to PhysX actuators.

    Args:
        stage: USD stage to update with PhysX actuators.
    """
    for prim in stage.Traverse():
        if prim.GetTypeName() == "MjcActuator":
            convert_mjc_actuator_to_physics(prim, stage)
        elif prim.IsA(UsdPhysics.RevoluteJoint) or prim.IsA(UsdPhysics.PrismaticJoint):
            convert_mjc_joint_to_physx(prim, stage)


def convert_mjc_actuator_to_physics(mjc_actuator: Usd.Prim, stage: Usd.Stage) -> None:
    """Convert an MJCF actuator to a PhysX actuator.

    Args:
        mjc_actuator: MJCF actuator prim.
        stage: USD stage containing the target joint prim.

    Raises:
        ValueError: If the actuator or its target joint prim is invalid.
    """
    if not mjc_actuator.IsValid():
        raise ValueError(f"MJCF actuator prim not found at path: {mjc_actuator.GetPath()}")
    joint_path = mjc_actuator.GetRelationship("mjc:target").GetTargets()[0]
    joint = stage.GetPrimAtPath(joint_path)
    if not joint.IsValid():
        raise ValueError(f"Joint prim not found at path: {joint_path.pathString}")

    # Determine joint type and apply to the appropriate drive instance
    if joint.IsA(UsdPhysics.RevoluteJoint):
        drive_instance = "angular"
    elif joint.IsA(UsdPhysics.PrismaticJoint):
        drive_instance = "linear"
    else:
        return

    if joint.HasAPI(UsdPhysics.DriveAPI, drive_instance):
        drive_api = UsdPhysics.DriveAPI(joint, drive_instance)
    else:
        drive_api = UsdPhysics.DriveAPI.Apply(joint, drive_instance)

    force_range_max = (
        mjc_actuator.GetAttribute("mjc:forceRange:max").Get()
        if mjc_actuator.GetAttribute("mjc:forceRange:max").IsValid()
        else None
    )
    force_range_min = (
        mjc_actuator.GetAttribute("mjc:forceRange:min").Get()
        if mjc_actuator.GetAttribute("mjc:forceRange:min").IsValid()
        else None
    )

    if force_range_max:
        drive_api.CreateMaxForceAttr().Set(force_range_max)
        if force_range_min:
            if math.fabs(force_range_min) != force_range_max:
                _logger.warning(
                    "Magnitude of force range min is not equal to force range max for actuator "
                    + f"{mjc_actuator.GetPath()} for joint {joint.GetPath()}: {abs(force_range_min)} != {force_range_max}"
                )

    # Retrieve gainPrm and biasPrm arrays from MJCF actuator
    gain_prm = (
        mjc_actuator.GetAttribute("mjc:gainPrm").Get() if mjc_actuator.GetAttribute("mjc:gainPrm").IsValid() else None
    )
    bias_prm = (
        mjc_actuator.GetAttribute("mjc:biasPrm").Get() if mjc_actuator.GetAttribute("mjc:biasPrm").IsValid() else None
    )

    # Retrieve gainType and biasType from MJCF actuator
    gain_type = (
        mjc_actuator.GetAttribute("mjc:gainType").Get() if mjc_actuator.GetAttribute("mjc:gainType").IsValid() else None
    )
    bias_type = (
        mjc_actuator.GetAttribute("mjc:biasType").Get() if mjc_actuator.GetAttribute("mjc:biasType").IsValid() else None
    )

    if not bias_prm or len(bias_prm) < 3 or not gain_prm or len(gain_prm) < 3:
        _logger.warning(
            "Gain and bias prm arrays are not available or supported for actuator "
            + f"{mjc_actuator.GetPath()} for joint {joint.GetPath()}, physics drive stiffness and damping will not be created"
        )
        return

    if not gain_type or gain_type != "fixed" or not bias_type or bias_type != "affine":
        _logger.warning(
            "Gain type or bias type not available or supported for actuator "
            + f"{mjc_actuator.GetPath()} for joint {joint.GetPath()}, physics drive stiffness and damping will not be created"
        )
        return

    # position control
    # "gainprm" = [kp, 0, 0, 0, 0, 0, 0, 0, 0, 0]
    # "biasprm" = [0, -kp, -kd, 0, 0, 0, 0, 0, 0, 0]
    # stiffness = kp
    # damping = kd

    if (
        gain_prm[0] > 0
        and gain_prm[1] == 0
        and gain_prm[2] == 0
        and bias_prm[0] == 0
        and bias_prm[1] < 0
        and bias_prm[2] < 0
        and gain_prm[0] == -bias_prm[1]
    ):
        actuator_stiffness = gain_prm[0]
        actuator_damping = -bias_prm[2]

        drive_api.CreateStiffnessAttr().Set(actuator_stiffness)
        drive_api.CreateDampingAttr().Set(actuator_damping)

    # velocity control
    # "gainprm" = [kd, 0, 0, 0, 0, 0, 0, 0, 0, 0]
    # "biasprm" = [0, 0, -kd, 0, 0, 0, 0, 0, 0, 0]
    # stiffness = 0
    # damping = kd

    elif (
        gain_prm[0] > 0
        and gain_prm[1] == 0
        and gain_prm[2] == 0
        and bias_prm[0] == 0
        and bias_prm[1] == 0
        and bias_prm[2] < 0
        and gain_prm[0] == -bias_prm[2]
    ):
        actuator_damping = -bias_prm[2]

        drive_api.CreateStiffnessAttr().Set(0)
        drive_api.CreateDampingAttr().Set(actuator_damping)
    else:
        _logger.warning(
            "Gain and bias prm arrays are not in the expected format for actuator "
            + f"{mjc_actuator.GetPath()} for joint {joint.GetPath()}, physics drive stiffness and damping will not be created"
        )


def convert_mjc_joint_to_physx(joint: Usd.Prim, stage: Usd.Stage) -> None:
    """Convert an MJCF joint to a PhysX joint.

    Args:
        joint: MJCF joint prim.
        stage: USD stage containing the joint prim.
    """
    # Set joint friction
    joint_friction = (
        joint.GetAttribute("mjc:frictionloss").Get() if joint.GetAttribute("mjc:frictionloss").IsValid() else None
    )
    if joint_friction:
        if not joint.HasAPI(PhysxSchema.JOINT_API):
            joint.ApplyAPI(PhysxSchema.JOINT_API)
        joint.CreateAttribute(PhysxAttr.JOINT_FRICTION.name, PhysxAttr.JOINT_FRICTION.type).Set(joint_friction)

    # Set armature
    joint_armature = joint.GetAttribute("mjc:armature").Get() if joint.GetAttribute("mjc:armature").IsValid() else None
    if joint_armature:
        if not joint.HasAPI(PhysxSchema.JOINT_API):
            joint.ApplyAPI(PhysxSchema.JOINT_API)
        joint.CreateAttribute(PhysxAttr.JOINT_ARMATURE.name, PhysxAttr.JOINT_ARMATURE.type).Set(joint_armature)

    # Set target_position
    joint_target_position = joint.GetAttribute("mjc:ref").Get() if joint.GetAttribute("mjc:ref").IsValid() else None
    if joint_target_position:
        if joint.IsA(UsdPhysics.RevoluteJoint):
            joint_type = "angular"
        elif joint.IsA(UsdPhysics.PrismaticJoint):
            joint_type = "linear"
        else:
            return

        if joint.HasAPI(UsdPhysics.DriveAPI, joint_type):
            drive_api = UsdPhysics.DriveAPI(joint, joint_type)
        else:
            drive_api = UsdPhysics.DriveAPI.Apply(joint, joint_type)

        drive_api.CreateTargetPositionAttr().Set(joint_target_position)


_AXIS_VECTORS = {
    "X": Gf.Vec3d(1.0, 0.0, 0.0),
    "Y": Gf.Vec3d(0.0, 1.0, 0.0),
    "Z": Gf.Vec3d(0.0, 0.0, 1.0),
}

_D6_ROTATION_AXES = (UsdPhysics.Tokens.rotX, UsdPhysics.Tokens.rotY, UsdPhysics.Tokens.rotZ)
_D6_TRANSLATION_AXES = (UsdPhysics.Tokens.transX, UsdPhysics.Tokens.transY, UsdPhysics.Tokens.transZ)
_D6_ALL_AXES = _D6_TRANSLATION_AXES + _D6_ROTATION_AXES

# Dot-product tolerance when matching source joint axes against D6 basis columns.
_AXIS_DOT_TOLERANCE = 1e-4
# Tolerance when checking that all joints of a group share the same joint frame.
_FRAME_POSITION_TOLERANCE = 1e-4


def _normalized(vec: Gf.Vec3d) -> Gf.Vec3d | None:
    """Return the unit vector for *vec*, or ``None`` when degenerate."""
    length = vec.GetLength()
    if length < 1e-12:
        return None
    return vec / length


def _joint_axis_directions(joint_prim: Usd.Prim) -> tuple[str, Gf.Vec3d, Gf.Vec3d] | None:
    """Return the physical axis of a single-axis joint in both body frames.

    mujoco-usd-converter always exports x-aligned joints and encodes the MJCF
    axis direction in ``localRot0``/``localRot1``, so the D6 axis cannot be
    derived from ``physics:axis`` alone.

    Args:
        joint_prim: The USD joint prim.

    Returns:
        ``(kind, axis_body0, axis_body1)`` where ``kind`` is ``"rotation"`` or
        ``"translation"`` and the vectors are the normalized physical axis
        expressed in body0/body1 local space, or ``None`` when the joint is not
        a single-axis joint or has no recognizable ``physics:axis``.
    """
    if joint_prim.IsA(UsdPhysics.RevoluteJoint):
        kind = "rotation"
    elif joint_prim.IsA(UsdPhysics.PrismaticJoint):
        kind = "translation"
    else:
        return None
    axis_attr = joint_prim.GetAttribute("physics:axis")
    if not axis_attr or not axis_attr.IsValid():
        return None
    axis_value = str(axis_attr.Get()).upper()
    if axis_value not in _AXIS_VECTORS:
        return None
    local_axis = _AXIS_VECTORS[axis_value]

    joint = UsdPhysics.Joint(joint_prim)
    directions = []
    for getter in (joint.GetLocalRot0Attr, joint.GetLocalRot1Attr):
        attr = getter()
        if attr and attr.HasAuthoredValue():
            quat = attr.Get()
            rotation = Gf.Rotation(Gf.Quatd(quat.GetReal(), Gf.Vec3d(quat.GetImaginary())))
        else:
            rotation = Gf.Rotation()
        direction = _normalized(rotation.TransformDir(local_axis))
        if direction is None:
            return None
        directions.append(direction)
    return kind, directions[0], directions[1]


def _assign_axes_to_d6_basis(
    directions: list[Gf.Vec3d],
) -> tuple[list[Gf.Vec3d], list[tuple[int, float] | None], list[int]]:
    """Assign source axis directions to the columns of a right-handed D6 basis.

    Args:
        directions: Physical joint axes in body1 local space, one per joint.

    Returns:
        ``(columns, assignments, dropped)`` where ``columns`` is a right-handed
        orthonormal basis of three vectors, ``assignments[i]`` is
        ``(column_index, sign)`` for joints whose axis maps onto a basis column
        (``sign`` is ``-1.0`` when the joint axis is anti-parallel to the
        column, which can happen after handedness correction) or ``None`` when
        the joint could not be represented, and ``dropped`` lists the indices
        of unrepresentable joints.
    """
    columns: list[Gf.Vec3d] = []
    assignments: list[tuple[int, float] | None] = []
    dropped: list[int] = []
    for direction in directions:
        assignment = None
        orthogonal = True
        for column_index, column in enumerate(columns):
            dot = direction * column
            if abs(dot) > 1.0 - _AXIS_DOT_TOLERANCE:
                # Same physical axis (parallel or anti-parallel): a D6 hosts a
                # single DOF per axis, so a second joint here is a duplicate.
                orthogonal = False
                break
            if abs(dot) > _AXIS_DOT_TOLERANCE:
                orthogonal = False
        if orthogonal and len(columns) < 3:
            columns.append(direction)
            assignment = (len(columns) - 1, 1.0)
        assignments.append(assignment)
        if assignment is None:
            dropped.append(len(assignments) - 1)

    # Complete the basis so it is always right-handed and orthonormal.
    if len(columns) == 1:
        c0 = columns[0]
        fallback = min(_AXIS_VECTORS.values(), key=lambda v: abs(c0 * v))
        c1 = _normalized(Gf.Cross(c0, fallback))
        columns = [c0, c1, Gf.Cross(c0, c1)]
    elif len(columns) == 2:
        c0, c1 = columns
        columns = [c0, c1, Gf.Cross(c0, c1)]
    else:
        # Three source-owned columns: flip the last one if the source axes
        # form a left-handed triple, which a D6 frame cannot represent. The
        # owning joint's sign is flipped accordingly (limits are swapped).
        if Gf.Cross(columns[0], columns[1]) * columns[2] < 0.0:
            columns[2] = -columns[2]
            for index, assignment in enumerate(assignments):
                if assignment is not None and assignment[0] == 2:
                    assignments[index] = (2, -assignment[1])
    return columns, assignments, dropped


def _group_joints_by_body_pair(stage: Usd.Stage) -> dict[tuple, list[Usd.Prim]]:
    """Group revolute/prismatic joints by their ``(body0, body1)`` targets.

    Uses ``TraverseAll`` so joints contributed by a sublayered physics
    layer (under ``over`` ancestors) are still found when the stage is
    rooted at a PhysX overlay layer.

    Args:
        stage: USD stage to traverse.

    Returns:
        Mapping from ``(body0_paths, body1_paths)`` to the joint prims
        sharing that body pair.
    """
    groups: dict[tuple, list[Usd.Prim]] = defaultdict(list)
    for prim in stage.TraverseAll():
        if not (prim.IsA(UsdPhysics.RevoluteJoint) or prim.IsA(UsdPhysics.PrismaticJoint)):
            continue
        # Skip prims a previous pass already deactivated so re-runs are idempotent.
        if not prim.IsActive():
            continue
        joint = UsdPhysics.Joint(prim)
        body0_rel = joint.GetBody0Rel()
        body1_rel = joint.GetBody1Rel()
        body0 = tuple(str(t) for t in (body0_rel.GetTargets() if body0_rel else []))
        body1 = tuple(str(t) for t in (body1_rel.GetTargets() if body1_rel else []))
        groups[(body0, body1)].append(prim)
    return groups


def _snapshot_ancestor_specifiers(layer: Sdf.Layer, path: Sdf.Path) -> list[tuple[Sdf.Path, Sdf.Specifier, str]]:
    """Snapshot ``(path, specifier, typeName)`` for each existing ancestor spec of *path*.

    Args:
        layer: Layer containing the ancestor prim specs.
        path: Prim path whose ancestors should be inspected.

    Returns:
        Existing ancestor prim paths with their original specifier and type name.
    """
    snapshot: list[tuple[Sdf.Path, Sdf.Specifier, str]] = []
    parent = path.GetParentPath()
    while parent != Sdf.Path.absoluteRootPath and not parent.isEmpty:
        spec = layer.GetPrimAtPath(parent)
        if spec is not None:
            snapshot.append((parent, spec.specifier, spec.typeName))
        parent = parent.GetParentPath()
    return snapshot


def _restore_ancestor_specifiers(layer: Sdf.Layer, snapshot: list[tuple[Sdf.Path, Sdf.Specifier, str]]) -> None:
    """Restore ancestor specifier/typeName values previously saved by ``_snapshot_ancestor_specifiers``.

    Prevents ``def Joint`` authoring from silently promoting ``over``
    ancestors to ``def``.

    Args:
        layer: Layer containing the ancestor prim specs to restore.
        snapshot: Ancestor specifier/typeName records produced by ``_snapshot_ancestor_specifiers``.
    """
    for path, specifier, type_name in snapshot:
        spec = layer.GetPrimAtPath(path)
        if spec is None:
            continue
        if spec.specifier != specifier:
            spec.specifier = specifier
        if spec.typeName != type_name:
            spec.typeName = type_name


def _quat_from_columns(columns: list[Gf.Vec3d]) -> Gf.Quatd:
    """Build a quaternion for the rotation whose matrix columns are the given basis vectors."""
    c0, c1, c2 = columns
    trace = c0[0] + c1[1] + c2[2]
    if trace > 0.0:
        s = math.sqrt(trace + 1.0) * 2.0
        quat = Gf.Quatd(0.25 * s, Gf.Vec3d((c1[2] - c2[1]) / s, (c2[0] - c0[2]) / s, (c0[1] - c1[0]) / s))
    elif c0[0] > c1[1] and c0[0] > c2[2]:
        s = math.sqrt(1.0 + c0[0] - c1[1] - c2[2]) * 2.0
        quat = Gf.Quatd(
            (c1[2] - c2[1]) / s, Gf.Vec3d(0.25 * s, (c1[0] + c0[1]) / s, (c2[0] + c0[2]) / s)
        )
    elif c1[1] > c2[2]:
        s = math.sqrt(1.0 + c1[1] - c0[0] - c2[2]) * 2.0
        quat = Gf.Quatd(
            (c2[0] - c0[2]) / s, Gf.Vec3d((c1[0] + c0[1]) / s, 0.25 * s, (c2[1] + c1[2]) / s)
        )
    else:
        s = math.sqrt(1.0 + c2[2] - c0[0] - c1[1]) * 2.0
        quat = Gf.Quatd(
            (c0[1] - c1[0]) / s, Gf.Vec3d((c2[0] + c0[2]) / s, (c2[1] + c1[2]) / s, 0.25 * s)
        )
    return Gf.Quatd(quat.GetNormalized())


def _convert_overconstrained_group_to_d6(
    stage: Usd.Stage,
    joints: list[Usd.Prim],
    body0: tuple,
    body1: tuple,
    source_joint_remap: dict[Sdf.Path, Sdf.Path],
) -> bool:
    """Combine one over-constrained joint group into a D6 joint.

    The first representable joint becomes the D6 host (its path is reused so
    external references stay valid); every other joint in the group is either
    folded in as another D6 axis or deactivated. The D6 joint frame is rebuilt
    from the physical source axes (recovered from each joint's ``localRot``),
    per-axis limits and drive parameters are mapped onto the corresponding D6
    axes, every unused D6 axis is explicitly locked, and each used axis records
    its source MJCF joint name in a ``mjcf:<axis>:name`` attribute so consumers
    can bind D6 axes back to the semantic joints.

    Args:
        stage: USD stage being edited.
        joints: Joint prims sharing the same body pair (length >= 2).
        body0: Tuple of body0 target paths (used for logging only).
        body1: Tuple of body1 target paths (used for logging only).
        source_joint_remap: Output map populated with
            ``source_joint_path -> d6_joint_path`` for each joint folded
            into the D6 — used by the Newton mimic rewriter.

    Returns:
        ``True`` if a D6 was constructed, ``False`` if no joint in the
        group had a recognizable axis.
    """
    group_paths = [j.GetPath() for j in joints]

    # Recover each joint's physical axis (in both body frames) from localRot.
    kinds: list[str] = []
    directions0: list[Gf.Vec3d] = []
    directions1: list[Gf.Vec3d] = []
    recognized: list[int] = []
    dropped_joints: list[Usd.Prim] = []
    for index, joint in enumerate(joints):
        result = _joint_axis_directions(joint)
        if result is None:
            _logger.warning(
                f"Joint {joint.GetPath()} has no recognizable physics:axis "
                "and cannot be encoded as a D6 axis; its DOF will be lost in "
                "the PhysX variant"
            )
            dropped_joints.append(joint)
            continue
        kind, axis_body0, axis_body1 = result
        recognized.append(index)
        kinds.append(kind)
        directions0.append(axis_body0)
        directions1.append(axis_body1)

    if not recognized:
        return False

    columns, axis_assignments, unassignable = _assign_axes_to_d6_basis(directions1)
    for index in unassignable:
        joint = joints[recognized[index]]
        _logger.warning(
            f"Joint {joint.GetPath()} shares its physical axis with another joint "
            f"or is non-orthogonal to the rest of over-constrained group {group_paths}; "
            "a D6 joint cannot represent this DOF, so it will be lost in the PhysX variant"
        )
        dropped_joints.append(joint)

    host_index = next(i for i, a in enumerate(axis_assignments) if a is not None)
    primary = joints[recognized[host_index]]
    primary_path = primary.GetPath()
    _logger.warning(
        f"Over-constrained joint group with {len(joints)} joints ({group_paths}) "
        f"between bodies {body0} and {body1} is being collapsed into single D6 joint at {primary_path}."
    )

    primary_joint_api = UsdPhysics.Joint(primary)

    # A single D6 has one joint frame: verify the group actually shares it.
    pos0_attr = primary_joint_api.GetLocalPos0Attr()
    pos1_attr = primary_joint_api.GetLocalPos1Attr()
    local_pos0 = pos0_attr.Get() if pos0_attr else None
    local_pos1 = pos1_attr.Get() if pos1_attr else None
    for index, assignment in enumerate(axis_assignments):
        if assignment is None:
            continue
        joint = UsdPhysics.Joint(joints[recognized[index]])
        for axis_name, attr, reference in (
            ("body0", joint.GetLocalPos0Attr(), local_pos0),
            ("body1", joint.GetLocalPos1Attr(), local_pos1),
        ):
            position = attr.Get() if attr else None
            if position is None or reference is None:
                continue
            if (Gf.Vec3d(position) - Gf.Vec3d(reference)).GetLength() > _FRAME_POSITION_TOLERANCE:
                _logger.warning(
                    f"Joint {joint.GetPath()} has a different {axis_name}-space joint position than "
                    f"{primary_path}; a single D6 joint cannot represent distinct source frames, "
                    f"so the frame of {primary_path} is used and the articulation may change."
                )

    primary_break_force = primary_joint_api.GetBreakForceAttr() if primary_joint_api.GetBreakForceAttr() else None
    primary_break_torque = primary_joint_api.GetBreakTorqueAttr() if primary_joint_api.GetBreakTorqueAttr() else None
    primary_collisions = (
        primary_joint_api.GetCollisionEnabledAttr() if primary_joint_api.GetCollisionEnabledAttr() else None
    )
    primary_excl = (
        primary_joint_api.GetExcludeFromArticulationAttr()
        if primary_joint_api.GetExcludeFromArticulationAttr()
        else None
    )

    primary_physx_attrs: list[tuple[str, "Sdf.ValueTypeName", object]] = []
    if primary.HasAPI(PhysxSchema.JOINT_API):
        for attr_enum in (PhysxAttr.JOINT_ARMATURE, PhysxAttr.JOINT_FRICTION, PhysxAttr.JOINT_MAX_VELOCITY):
            src_attr = primary.GetAttribute(attr_enum.name)
            if src_attr and src_attr.IsValid() and src_attr.HasAuthoredValue():
                primary_physx_attrs.append((attr_enum.name, attr_enum.type, src_attr.Get()))

    # Snapshot per-axis limits/drive params before retyping the primary.
    # ``sign`` is -1 when the source axis is anti-parallel to the D6 basis
    # column: limits swap and drive targets flip accordingly.
    axis_state: list[tuple[str, dict, dict, float, Usd.Prim]] = []
    used_axes: set[str] = set()
    for index, assignment in enumerate(axis_assignments):
        if assignment is None:
            continue
        column_index, sign = assignment
        joint = joints[recognized[index]]
        token = (
            _D6_ROTATION_AXES[column_index] if kinds[index] == "rotation" else _D6_TRANSLATION_AXES[column_index]
        )
        used_axes.add(token)

        limit_state: dict = {}
        lower_attr = joint.GetAttribute("physics:lowerLimit")
        upper_attr = joint.GetAttribute("physics:upperLimit")
        if lower_attr and lower_attr.IsValid() and lower_attr.HasAuthoredValue():
            limit_state["low"] = lower_attr.Get()
        if upper_attr and upper_attr.IsValid() and upper_attr.HasAuthoredValue():
            limit_state["high"] = upper_attr.Get()

        drive_state: dict = {}
        drive_instance = "angular" if joint.IsA(UsdPhysics.RevoluteJoint) else "linear"
        if joint.HasAPI(UsdPhysics.DriveAPI, drive_instance):
            src_drive = UsdPhysics.DriveAPI(joint, drive_instance)
            for key, getter_name in (
                ("damping", "GetDampingAttr"),
                ("stiffness", "GetStiffnessAttr"),
                ("max_force", "GetMaxForceAttr"),
                ("target_position", "GetTargetPositionAttr"),
                ("target_velocity", "GetTargetVelocityAttr"),
                ("type", "GetTypeAttr"),
            ):
                src_attr = getattr(src_drive, getter_name)()
                if src_attr and src_attr.IsValid() and src_attr.HasAuthoredValue():
                    drive_state[key] = src_attr.Get()

        # Limit spring gains authored through PhysxLimitAPI on the source axis.
        source_axis = str(joint.GetAttribute("physics:axis").Get())
        for gain in ("stiffness", "damping"):
            src_attr = joint.GetAttribute(f"physxLimit:{source_axis}:{gain}")
            if src_attr and src_attr.IsValid() and src_attr.HasAuthoredValue():
                limit_state[f"physx_{gain}"] = src_attr.Get()

        axis_state.append((token, limit_state, drive_state, sign, joint))

    edit_layer = stage.GetEditTarget().GetLayer()
    ancestor_snapshot = _snapshot_ancestor_specifiers(edit_layer, primary_path)

    d6_joint = UsdPhysics.Joint.Define(stage, primary_path)
    if list(body0):
        d6_joint.CreateBody0Rel().SetTargets([Sdf.Path(p) for p in body0])
    if list(body1):
        d6_joint.CreateBody1Rel().SetTargets([Sdf.Path(p) for p in body1])
    if local_pos0 is not None:
        d6_joint.CreateLocalPos0Attr().Set(local_pos0)
    if local_pos1 is not None:
        d6_joint.CreateLocalPos1Attr().Set(local_pos1)

    # Rebuild the joint frame from the physical source axes: the D6 frame in
    # body1 space has the basis columns as its axes, and the body0 frame is the
    # same physical frame expressed through the primary joint's relative pose.
    d6_joint.CreateLocalRot1Attr().Set(Gf.Quatf(_quat_from_columns(columns)))
    primary_rot0 = primary_joint_api.GetLocalRot0Attr()
    primary_rot1 = primary_joint_api.GetLocalRot1Attr()
    if primary_rot0 and primary_rot0.HasAuthoredValue() and primary_rot1 and primary_rot1.HasAuthoredValue():
        quat0 = primary_rot0.Get()
        quat1 = primary_rot1.Get()
        rotation0 = Gf.Rotation(Gf.Quatd(quat0.GetReal(), Gf.Vec3d(quat0.GetImaginary())))
        rotation1 = Gf.Rotation(Gf.Quatd(quat1.GetReal(), Gf.Vec3d(quat1.GetImaginary())))
        relative = rotation0 * rotation1.GetInverse()
        columns0 = [_normalized(relative.TransformDir(c)) for c in columns]
        d6_joint.CreateLocalRot0Attr().Set(Gf.Quatf(_quat_from_columns(columns0)))

    if primary_break_force and primary_break_force.HasAuthoredValue():
        d6_joint.CreateBreakForceAttr().Set(primary_break_force.Get())
    if primary_break_torque and primary_break_torque.HasAuthoredValue():
        d6_joint.CreateBreakTorqueAttr().Set(primary_break_torque.Get())
    if primary_collisions and primary_collisions.HasAuthoredValue():
        d6_joint.CreateCollisionEnabledAttr().Set(primary_collisions.Get())
    if primary_excl and primary_excl.HasAuthoredValue():
        d6_joint.CreateExcludeFromArticulationAttr().Set(primary_excl.Get())
    d6_joint.CreateJointEnabledAttr().Set(True)

    d6_prim = d6_joint.GetPrim()

    for token, limit_state, drive_state, sign, source_joint in axis_state:
        if "low" in limit_state and "high" in limit_state and sign < 0:
            limit_state = {**limit_state, "low": -limit_state["high"], "high": -limit_state["low"]}
        # Only author a LimitAPI when the source joint actually has limits:
        # an applied-but-unauthored LimitAPI still composes to the schema
        # fallback, and an unlimited source axis must stay truly free.
        if "low" in limit_state or "high" in limit_state:
            limit = UsdPhysics.LimitAPI.Apply(d6_prim, token)
            if "low" in limit_state:
                limit.CreateLowAttr().Set(limit_state["low"])
            if "high" in limit_state:
                limit.CreateHighAttr().Set(limit_state["high"])
        for gain in ("stiffness", "damping"):
            if f"physx_{gain}" in limit_state:
                d6_prim.CreateAttribute(f"physxLimit:{token}:{gain}", Sdf.ValueTypeNames.Float).Set(
                    limit_state[f"physx_{gain}"]
                )

        if drive_state:
            if sign < 0:
                for key in ("target_position", "target_velocity"):
                    if key in drive_state:
                        drive_state = {**drive_state, key: -drive_state[key]}
            dst_drive = UsdPhysics.DriveAPI.Apply(d6_prim, token)
            if "damping" in drive_state:
                dst_drive.CreateDampingAttr().Set(drive_state["damping"])
            if "stiffness" in drive_state:
                dst_drive.CreateStiffnessAttr().Set(drive_state["stiffness"])
            if "max_force" in drive_state:
                dst_drive.CreateMaxForceAttr().Set(drive_state["max_force"])
            if "target_position" in drive_state:
                dst_drive.CreateTargetPositionAttr().Set(drive_state["target_position"])
            if "target_velocity" in drive_state:
                dst_drive.CreateTargetVelocityAttr().Set(drive_state["target_velocity"])
            if "type" in drive_state:
                dst_drive.CreateTypeAttr().Set(drive_state["type"])

        # Bind the D6 axis back to the semantic MJCF joint name so consumers
        # can recover the per-axis mapping after the collapse.
        source_name = source_joint.GetPrim().GetDisplayName() or source_joint.GetName()
        d6_prim.CreateAttribute(f"mjcf:{token}:name", Sdf.ValueTypeNames.Token).Set(source_name)

    # Lock every D6 axis that no source joint maps to; an unlimited D6 axis is
    # free in PhysX, which would add DOFs the MJCF never had.
    for token in _D6_ALL_AXES:
        if token in used_axes:
            continue
        limit = UsdPhysics.LimitAPI.Apply(d6_prim, token)
        limit.CreateLowAttr().Set(1.0)
        limit.CreateHighAttr().Set(-1.0)

    # PhysxJointAPI tuning is single-valued per joint: take the primary's
    # values and drop the rest (the warning at the end notes the loss).
    if primary_physx_attrs:
        if not d6_prim.HasAPI(PhysxSchema.JOINT_API):
            d6_prim.ApplyAPI(PhysxSchema.JOINT_API)
        for attr_name, attr_type, attr_value in primary_physx_attrs:
            d6_prim.CreateAttribute(attr_name, attr_type).Set(attr_value)

    # Drop stale single-axis attrs at the edit target (no-op when those live
    # on a sublayer, which keeps the MuJoCo/Newton variants intact).
    edit_prim_spec = edit_layer.GetPrimAtPath(primary_path)
    if edit_prim_spec is not None:
        for prop_name in ("physics:axis", "physics:lowerLimit", "physics:upperLimit"):
            attr_spec = edit_prim_spec.attributes.get(prop_name)
            if attr_spec is not None:
                edit_prim_spec.RemoveProperty(attr_spec)

    # Deactivate every other joint; leaving any active re-triggers
    # over-constraining. Filter primary_path from both lists defensively.
    converted_joints = [joint for _, _, _, _, joint in axis_state]
    joints_to_deactivate: list[Usd.Prim] = [
        j for j in converted_joints + list(dropped_joints) if j.GetPath() != primary_path
    ]
    for joint in joints_to_deactivate:
        override = stage.OverridePrim(joint.GetPath())
        override.SetActive(False)

    for joint in converted_joints:
        source_joint_remap[joint.GetPath()] = primary_path

    _restore_ancestor_specifiers(edit_layer, ancestor_snapshot)

    chain_names = ", ".join(j.GetName() for j in converted_joints)
    _logger.warning(
        f"Combined over-constrained joints [{chain_names}] between body pair "
        f"{list(body0)} -> {list(body1)} into PhysX D6 joint '{primary_path}'. "
        "MuJoCo/Newton variants retain the original per-DOF joints, so joint "
        "frames, limits, and gains may differ between variants and a control "
        "policy trained on one variant cannot be transferred directly to the other."
    )
    return True


def _rewrite_newton_mimic_joint_references(stage: Usd.Stage, source_joint_remap: dict[Sdf.Path, Sdf.Path]) -> int:
    """Redirect ``NewtonMimicAPI`` ``newton:mimicJoint`` targets to the D6 host.

    Args:
        stage: USD stage being edited.
        source_joint_remap: ``source_joint_path -> d6_joint_path`` populated
            by :func:`_convert_overconstrained_group_to_d6`.

    Returns:
        Number of mimic prims whose reference was rewritten.
    """
    if not source_joint_remap:
        return 0

    rewrites = 0
    for prim in stage.TraverseAll():
        if not prim.HasAPI("NewtonMimicAPI"):
            continue

        ref_rel = prim.GetRelationship("newton:mimicJoint")
        if not ref_rel or not ref_rel.IsValid():
            continue

        targets = list(ref_rel.GetTargets())
        new_targets: list[Sdf.Path] = []
        changed = False
        for target in targets:
            if target in source_joint_remap:
                new_targets.append(source_joint_remap[target])
                changed = True
            else:
                new_targets.append(target)

        if not changed:
            continue

        prim.CreateRelationship("newton:mimicJoint").SetTargets(new_targets)
        rewrites += 1
        _logger.info(
            f"Rewrote NewtonMimicAPI reference on {prim.GetPath()} to D6 "
            f"joint {new_targets} following over-constrained joint conversion"
        )

    return rewrites


def combine_overconstrained_joints_to_d6(stage: Usd.Stage) -> int:
    """Combine joints sharing the same body pair into a single PhysX D6 joint.

    For each ``(body0, body1)`` group with more than one joint the first
    joint is retyped to ``PhysicsJoint`` with per-axis ``LimitAPI`` and
    ``DriveAPI`` instances; the rest are deactivated, and any
    ``NewtonMimicAPI`` ``newton:mimicJoint`` reference targeting them is
    redirected to the new D6 host. All authoring goes to the stage's
    current edit target, so the caller should set that to the PhysX
    overlay layer (e.g. ``payloads/Physics/physx.usda``) to keep
    MuJoCo/Newton variants untouched.

    Args:
        stage: USD stage to inspect for over-constrained joint groups.

    Returns:
        Number of joint groups that were combined into D6 joints.
    """
    converted_count = 0
    source_joint_remap: dict[Sdf.Path, Sdf.Path] = {}
    groups = _group_joints_by_body_pair(stage)
    for (body0, body1), joints in groups.items():
        if len(joints) < 2:
            continue
        if _convert_overconstrained_group_to_d6(stage, joints, body0, body1, source_joint_remap):
            converted_count += 1

    if source_joint_remap:
        _rewrite_newton_mimic_joint_references(stage, source_joint_remap)

    return converted_count


def combine_overconstrained_joints_in_physx_layer(physx_layer_path: str) -> int:
    """Run :func:`combine_overconstrained_joints_to_d6` against a standalone PhysX overlay layer.

    Assumes the PhysX layer sublayers the base physics layer (the asset
    transformer's ``physx.usda`` -> ``physics.usda`` layout). All edits
    are authored back into the PhysX layer only.

    Args:
        physx_layer_path: Path to the PhysX overlay layer file
            (typically ``payloads/Physics/physx.usda``).

    Returns:
        Number of joint groups combined, or 0 if the layer can't be opened.
    """
    if not os.path.exists(physx_layer_path):
        _logger.error(
            f"PhysX overlay layer not found at {physx_layer_path}; skipping D6 conversion. "
            "PhysX articulation over-constraining will NOT be corrected for this asset."
        )
        return 0

    physx_layer = Sdf.Layer.FindOrOpen(physx_layer_path)
    if physx_layer is None:
        _logger.error(
            f"Failed to open PhysX overlay layer at {physx_layer_path}; skipping D6 conversion. "
            "PhysX articulation over-constraining will NOT be corrected for this asset."
        )
        return 0

    stage = Usd.Stage.Open(physx_layer)
    if stage is None:
        _logger.error(f"Failed to open stage from PhysX overlay layer at {physx_layer_path}; skipping D6 conversion")
        return 0

    previous_target = stage.GetEditTarget()
    try:
        stage.SetEditTarget(stage.GetEditTargetForLocalLayer(physx_layer))
        converted = combine_overconstrained_joints_to_d6(stage)
    finally:
        stage.SetEditTarget(previous_target)

    if converted:
        physx_layer.Save()
    return converted
