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

from pxr import Sdf, Usd, UsdPhysics

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


def _joint_axis_d6_token(joint_prim: Usd.Prim) -> str | None:
    """Return the D6 axis token for a single-axis revolute or prismatic joint.

    Args:
        joint_prim: The USD joint prim.

    Returns:
        D6 axis token (e.g. ``"rotX"``, ``"transY"``) or ``None`` if the
        joint has no recognizable ``physics:axis`` value or is not a
        single-axis joint type.
    """
    axis_attr = joint_prim.GetAttribute("physics:axis")
    if not axis_attr or not axis_attr.IsValid():
        return None
    axis_value = axis_attr.Get()
    if not axis_value:
        return None
    axis_value = str(axis_value).upper()
    if joint_prim.IsA(UsdPhysics.RevoluteJoint):
        return _REVOLUTE_AXIS_TO_D6_TOKEN.get(axis_value)
    if joint_prim.IsA(UsdPhysics.PrismaticJoint):
        return _PRISMATIC_AXIS_TO_D6_TOKEN.get(axis_value)
    return None


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
    """Snapshot ``(path, specifier, typeName)`` for each existing ancestor spec of *path*."""
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
    """
    for path, specifier, type_name in snapshot:
        spec = layer.GetPrimAtPath(path)
        if spec is None:
            continue
        if spec.specifier != specifier:
            spec.specifier = specifier
        if spec.typeName != type_name:
            spec.typeName = type_name


def _convert_overconstrained_group_to_d6(
    stage: Usd.Stage,
    joints: list[Usd.Prim],
    body0: tuple,
    body1: tuple,
    source_joint_remap: dict[Sdf.Path, Sdf.Path],
) -> bool:
    """Combine one over-constrained joint group into a D6 joint.

    The first joint becomes the D6 host (its path is reused so external
    references stay valid); every other joint in the group is either
    folded in as another D6 axis or deactivated.

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
    # Defer picking the primary until axis assignment is done so the host
    # never lands on a path that's about to be deactivated.
    group_paths = [j.GetPath() for j in joints]
    axis_assignments: list[tuple[Usd.Prim, str]] = []
    dropped_joints: list[Usd.Prim] = []
    used_axes: set[str] = set()
    for joint in joints:
        token = _joint_axis_d6_token(joint)
        if token is None:
            _logger.warning(
                f"Joint {joint.GetPath()} has no recognizable physics:axis "
                "and cannot be encoded as a D6 axis; its DOF will be lost in "
                "the PhysX variant"
            )
            dropped_joints.append(joint)
            continue
        if token in used_axes:
            _logger.warning(
                f"Joint {joint.GetPath()} duplicates D6 axis '{token}' in over-constrained "
                f"group {group_paths} (the MJCF axis direction is encoded in localRot, "
                "which the D6 cannot represent uniquely); its DOF will be lost in the "
                "PhysX variant"
            )
            dropped_joints.append(joint)
            continue
        used_axes.add(token)
        axis_assignments.append((joint, token))

    if not axis_assignments:
        return False

    # Host the D6 on the first joint that actually contributed an axis.
    primary, _ = axis_assignments[0]
    primary_path = primary.GetPath()
    _logger.warning(
        f"Over-constrained joint group with {len(joints)} joints ({group_paths}) "
        f"between bodies {body0} and {body1} is being collapsed into single D6 joint at {primary_path}. "
        "Only one DOF per axis will be preserved; duplicate or unrecognized axes will be dropped."
    )

    primary_joint_api = UsdPhysics.Joint(primary)
    local_pos0 = primary_joint_api.GetLocalPos0Attr().Get() if primary_joint_api.GetLocalPos0Attr() else None
    local_pos1 = primary_joint_api.GetLocalPos1Attr().Get() if primary_joint_api.GetLocalPos1Attr() else None
    local_rot0 = primary_joint_api.GetLocalRot0Attr().Get() if primary_joint_api.GetLocalRot0Attr() else None
    local_rot1 = primary_joint_api.GetLocalRot1Attr().Get() if primary_joint_api.GetLocalRot1Attr() else None
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
    axis_state: list[tuple[str, dict, dict]] = []
    for joint, token in axis_assignments:
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
        axis_state.append((token, limit_state, drive_state))

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
    if local_rot0 is not None:
        d6_joint.CreateLocalRot0Attr().Set(local_rot0)
    if local_rot1 is not None:
        d6_joint.CreateLocalRot1Attr().Set(local_rot1)
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

    for token, limit_state, drive_state in axis_state:
        limit = UsdPhysics.LimitAPI.Apply(d6_prim, token)
        if "low" in limit_state:
            limit.CreateLowAttr().Set(limit_state["low"])
        if "high" in limit_state:
            limit.CreateHighAttr().Set(limit_state["high"])

        if drive_state:
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
    converted_joints = [j for j, _ in axis_assignments]
    joints_to_deactivate: list[Usd.Prim] = [
        j for j in converted_joints + list(dropped_joints) if j.GetPath() != primary_path
    ]
    for joint in joints_to_deactivate:
        override = stage.OverridePrim(joint.GetPath())
        override.SetActive(False)

    for joint, _ in axis_assignments:
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
