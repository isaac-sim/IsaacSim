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
"""Read joint data from USD physics joints for URDF export."""

from __future__ import annotations

import logging
import math
from collections.abc import Callable
from dataclasses import dataclass

from isaacsim.asset.importer.utils.impl.physx_types import PhysxAttr, PhysxMimicAttr, PhysxMimicRel
from pxr import Gf, Usd, UsdPhysics

from .robot_finder import RobotDescription
from .transform_utils import compute_joint_origin, get_prim_name

_logger = logging.getLogger(__name__)

_D6_AXIS_TOKENS = ("rotX", "rotY", "rotZ", "transX", "transY", "transZ")

_AXIS_TOKEN_TO_VECTOR: dict[str, tuple[float, float, float]] = {
    "transX": (1.0, 0.0, 0.0),
    "transY": (0.0, 1.0, 0.0),
    "transZ": (0.0, 0.0, 1.0),
    "rotX": (1.0, 0.0, 0.0),
    "rotY": (0.0, 1.0, 0.0),
    "rotZ": (0.0, 0.0, 1.0),
}


@dataclass
class JointData:
    """Complete data for a URDF joint."""

    name: str = ""
    joint_type: str = "fixed"
    parent_link: str = ""
    child_link: str = ""
    origin_xyz: tuple[float, float, float] = (0.0, 0.0, 0.0)
    origin_rpy: tuple[float, float, float] = (0.0, 0.0, 0.0)
    axis: tuple[float, float, float] = (1.0, 0.0, 0.0)
    limit_lower: float | None = None
    limit_upper: float | None = None
    limit_effort: float | None = None
    limit_velocity: float | None = None
    dynamics_damping: float | None = None
    dynamics_friction: float | None = None
    calibration_rising: float | None = None
    calibration_falling: float | None = None
    calibration_reference_position: float | None = None
    safety_k_velocity: float | None = None
    safety_k_position: float | None = None
    safety_soft_lower: float | None = None
    safety_soft_upper: float | None = None
    mimic_joint: str | None = None
    mimic_multiplier: float | None = None
    mimic_offset: float | None = None
    original_usd_type: str | None = None
    original_params: dict | None = None
    source_drive: dict | None = None


@dataclass
class LoopJointData:
    """Data for a URDF <loop_joint> element (closed kinematic chain)."""

    name: str = ""
    joint_type: str = "revolute"
    link1_name: str = ""
    link1_xyz: tuple[float, float, float] = (0.0, 0.0, 0.0)
    link1_rpy: tuple[float, float, float] = (0.0, 0.0, 0.0)
    link2_name: str = ""
    link2_xyz: tuple[float, float, float] = (0.0, 0.0, 0.0)
    link2_rpy: tuple[float, float, float] = (0.0, 0.0, 0.0)


def read_joints(
    desc: RobotDescription,
    link_name_map: dict[str, str],
    urdf_frames: dict[str, Gf.Matrix4d] | None = None,
    axis_flips: dict[str, bool] | None = None,
    actuator_map: dict[str, Usd.Prim] | None = None,
) -> tuple[list[JointData], list["LinkData"]]:
    """Read all joints from the robot description.

    Skips the root_joint (FixedJoint connecting to world) and
    MjcActuator prims.  Multi-DOF joints (SphericalJoint, D6Joint,
    generic Joint) are expanded into chains of single-DOF URDF joints
    connected by ghost links.

    Args:
        desc: RobotDescription with discovered joints.
        link_name_map: Mapping from prim path string to URDF link name.
        urdf_frames: Pre-built URDF frames for each link (from build_urdf_frames).
        axis_flips: Dict mapping joint prim path -> bool (True if axis is negated).
        actuator_map: Mapping from joint prim path to MjcActuator prim (for
            MuJoCo-origin stages).

    Returns:
        Tuple of (joints, ghost_links).

    """
    from .link_reader import LinkData

    stage = desc.root_prim.GetStage()
    if axis_flips is None:
        axis_flips = {}

    joint_name_map: dict[str, str] = {}
    results: list[JointData] = []
    ghost_links: list[LinkData] = []

    for j in desc.ordered_joints:
        if j.GetTypeName() == "MjcActuator":
            continue

        if _is_world_joint(j, desc):
            continue

        jd = _read_single_joint(j, link_name_map, stage, urdf_frames, axis_flips, actuator_map)
        if jd:
            if jd.joint_type in ("spherical", "d6"):
                chain_joints, chain_ghosts = _expand_multi_dof_joint(j, jd)
                if chain_joints:
                    for cj in chain_joints:
                        joint_name_map[str(j.GetPath())] = cj.name
                    results.extend(chain_joints)
                    ghost_links.extend(chain_ghosts)
                    continue
                _logger.warning(
                    "Joint '%s' (%s) has no free axes; exporting as fixed. " "All DOFs will be lost in the URDF.",
                    jd.name,
                    j.GetTypeName(),
                )
                jd.joint_type = "fixed"

            joint_name_map[str(j.GetPath())] = jd.name
            results.append(jd)

    _resolve_mimic_names(results, joint_name_map, desc)

    return results, ghost_links


def read_loop_joints(desc: RobotDescription, link_name_map: dict[str, str]) -> list[LoopJointData]:
    """Read loop joints (excludeFromArticulation=true) for URDF <loop_joint> elements.

    Args:
        desc: RobotDescription with discovered loop joints.
        link_name_map: Mapping from prim path string to URDF link name.

    Returns:
        List of LoopJointData.

    """
    results: list[LoopJointData] = []

    for j in desc.loop_joints:
        joint = UsdPhysics.Joint(j)
        if not joint:
            continue

        name = get_prim_name(j)

        body0_targets = joint.GetBody0Rel().GetTargets() if joint.GetBody0Rel() else []
        body1_targets = joint.GetBody1Rel().GetTargets() if joint.GetBody1Rel() else []

        link1_path = str(body0_targets[0]) if body0_targets else None
        link2_path = str(body1_targets[0]) if body1_targets else None

        link1_name = link_name_map.get(link1_path, "") if link1_path else ""
        link2_name = link_name_map.get(link2_path, "") if link2_path else ""

        if not link1_name or not link2_name:
            continue

        joint_type = _get_loop_joint_type(j)

        link1_xyz, link1_rpy = _get_loop_joint_frame(joint, 0)
        link2_xyz, link2_rpy = _get_loop_joint_frame(joint, 1)

        results.append(
            LoopJointData(
                name=name,
                joint_type=joint_type,
                link1_name=link1_name,
                link1_xyz=link1_xyz,
                link1_rpy=link1_rpy,
                link2_name=link2_name,
                link2_xyz=link2_xyz,
                link2_rpy=link2_rpy,
            )
        )

    return results


def _get_loop_joint_type(prim: Usd.Prim) -> str:
    """Map USD joint type to a URDF-compatible loop joint type string.

    Args:
        prim: USD prim to read.

    Returns:
        URDF-compatible loop joint type.
    """
    if prim.IsA(UsdPhysics.RevoluteJoint):
        return "revolute"
    if prim.IsA(UsdPhysics.PrismaticJoint):
        return "prismatic"
    if prim.IsA(UsdPhysics.FixedJoint):
        return "fixed"
    return "spherical"


def _get_loop_joint_frame(
    joint: UsdPhysics.Joint, body_index: int
) -> tuple[tuple[float, float, float], tuple[float, float, float]]:
    """Read the local frame (xyz, rpy) for one side of a loop joint.

    Args:
        joint: Joint data or USD physics joint to read.
        body_index: Body relationship index to read.

    Returns:
        Local frame translation and rotation.
    """
    from pxr import Gf

    from .transform_utils import _get_local_transform, matrix4_to_origin

    mat = _get_local_transform(joint, body_index)
    if mat is None:
        return (0.0, 0.0, 0.0), (0.0, 0.0, 0.0)
    return matrix4_to_origin(Gf.Matrix4d(mat))


def _is_world_joint(joint_prim: Usd.Prim, desc: RobotDescription) -> bool:
    """Check if this is the root joint connecting the robot to the world.

    Args:
        joint_prim: USD joint prim to read.
        desc: Robot description to read.

    Returns:
        True if the joint connects the robot to the world, False otherwise.
    """
    if not joint_prim.IsA(UsdPhysics.FixedJoint):
        return False

    joint_path = str(joint_prim.GetPath())
    bodies = desc.joint_parent_child.get(joint_path)
    if not bodies:
        return False

    body0, body1 = bodies
    root_path = str(desc.root_link.GetPath()) if desc.root_link else None

    if body1 and str(body1) == root_path:
        if body0 is None:
            return True
        stage = joint_prim.GetStage()
        body0_prim = stage.GetPrimAtPath(body0)
        if body0_prim is None or not body0_prim.HasAPI(UsdPhysics.RigidBodyAPI):
            return True

    if body0 and str(body0) == root_path:
        if body1 is None:
            return True
        stage = joint_prim.GetStage()
        body1_prim = stage.GetPrimAtPath(body1)
        if body1_prim is None or not body1_prim.HasAPI(UsdPhysics.RigidBodyAPI):
            return True

    return False


def _read_single_joint(
    joint_prim: Usd.Prim,
    link_name_map: dict[str, str],
    stage: Usd.Stage,
    urdf_frames: dict[str, Gf.Matrix4d] | None,
    axis_flips: dict[str, bool] | None = None,
    actuator_map: dict[str, Usd.Prim] | None = None,
) -> JointData | None:
    """Read a single joint's data from its USD prim.

    Args:
        joint_prim: USD joint prim to read.
        link_name_map: Mapping from link prim paths to URDF link names.
        stage: USD stage to read.
        urdf_frames: Mapping from link prim paths to URDF frames.
        axis_flips: Mapping from joint prim paths to axis flip flags.
        actuator_map: Mapping from joint prim paths to actuator prims.

    Returns:
        Joint data, or None if the joint cannot be exported.
    """
    jd = JointData(name=get_prim_name(joint_prim))

    joint = UsdPhysics.Joint(joint_prim)
    if not joint:
        return None

    jd.joint_type = _get_joint_type(joint_prim)

    body0_rel = joint.GetBody0Rel()
    body1_rel = joint.GetBody1Rel()
    body0_targets = body0_rel.GetTargets() if body0_rel else []
    body1_targets = body1_rel.GetTargets() if body1_rel else []

    parent_path = str(body0_targets[0]) if body0_targets else None
    child_path = str(body1_targets[0]) if body1_targets else None

    jd.parent_link = link_name_map.get(parent_path, "") if parent_path else ""
    jd.child_link = link_name_map.get(child_path, "") if child_path else ""

    if not jd.parent_link or not jd.child_link:
        return None

    if urdf_frames and parent_path and child_path:
        from .urdf_frames import compute_joint_origin_from_frames

        jd.origin_xyz, jd.origin_rpy = compute_joint_origin_from_frames(urdf_frames, parent_path, child_path)
    else:
        jd.origin_xyz, jd.origin_rpy = compute_joint_origin(joint)

    if jd.joint_type in ("revolute", "continuous", "prismatic", "planar"):
        flipped = (axis_flips or {}).get(str(joint_prim.GetPath()), False)
        jd.axis = _read_axis(joint_prim, joint, flipped)

    if jd.joint_type in ("revolute", "prismatic"):
        _read_limits(joint_prim, jd, actuator_map)

    _read_dynamics(joint_prim, jd, actuator_map)
    _read_calibration(joint_prim, jd)
    _read_safety_controller(joint_prim, jd)
    _read_mimic(joint_prim, jd)
    jd.source_drive = _collect_source_drive_breadcrumb(joint_prim, actuator_map)

    return jd


def _get_joint_type(prim: Usd.Prim) -> str:
    """Map USD joint type to URDF joint type string.

    Returns sentinel values ``"spherical"`` and ``"d6"`` for multi-DOF
    joints that require chain expansion.

    Args:
        prim: USD prim to read.

    Returns:
        URDF joint type string.
    """
    if prim.IsA(UsdPhysics.RevoluteJoint):
        rev = UsdPhysics.RevoluteJoint(prim)
        lower = rev.GetLowerLimitAttr().Get() if rev.GetLowerLimitAttr() else None
        upper = rev.GetUpperLimitAttr().Get() if rev.GetUpperLimitAttr() else None
        if lower is None and upper is None:
            return "continuous"
        if lower is not None and upper is not None and lower >= upper:
            return "continuous"
        return "revolute"

    if prim.IsA(UsdPhysics.PrismaticJoint):
        return "prismatic"

    if prim.IsA(UsdPhysics.FixedJoint):
        return "fixed"

    if prim.IsA(UsdPhysics.SphericalJoint):
        return "spherical"

    if prim.IsA(UsdPhysics.Joint):
        return "d6"

    _logger.warning(
        "Unsupported joint prim type '%s' at '%s'; exporting as fixed.",
        prim.GetTypeName(),
        prim.GetPath(),
    )
    return "fixed"


def _read_axis(joint_prim: Usd.Prim, joint: UsdPhysics.Joint, flipped: bool = False) -> tuple[float, float, float]:
    """Read the joint axis from the physics:axis token.

    Since the URDF child frame is set to the joint frame (via GetJointPose),
    the axis in the URDF child frame is simply the axis token direction.
    When localRot1 flips the axis (detected by build_urdf_frames), the
    URDF axis is negated.

    Args:
        joint_prim: USD joint prim to read.
        joint: Joint data or USD physics joint to read.
        flipped: Whether to negate the joint axis.

    Returns:
        Joint axis vector.
    """
    axis_attr = joint_prim.GetAttribute("physics:axis")
    axis_token = str(axis_attr.Get()).upper() if axis_attr and axis_attr.IsValid() else "X"

    axis_map = {"X": (1.0, 0.0, 0.0), "Y": (0.0, 1.0, 0.0), "Z": (0.0, 0.0, 1.0)}
    axis = axis_map.get(axis_token, (1.0, 0.0, 0.0))

    if flipped:
        axis = (-axis[0], -axis[1], -axis[2])

    return axis


def _read_limits(joint_prim: Usd.Prim, jd: JointData, actuator_map: dict[str, Usd.Prim] | None = None) -> None:
    """Read joint position limits (converting degrees to radians for revolute).

    Args:
        joint_prim: USD joint prim to read.
        jd: Joint data to populate.
        actuator_map: Mapping from joint prim paths to actuator prims.
    """
    is_revolute = joint_prim.IsA(UsdPhysics.RevoluteJoint)

    if is_revolute:
        rev = UsdPhysics.RevoluteJoint(joint_prim)
        lower_attr = rev.GetLowerLimitAttr()
        upper_attr = rev.GetUpperLimitAttr()
        if lower_attr and lower_attr.Get() is not None:
            jd.limit_lower = math.radians(float(lower_attr.Get()))
        if upper_attr and upper_attr.Get() is not None:
            jd.limit_upper = math.radians(float(upper_attr.Get()))
    else:
        pri = UsdPhysics.PrismaticJoint(joint_prim)
        lower_attr = pri.GetLowerLimitAttr()
        upper_attr = pri.GetUpperLimitAttr()
        if lower_attr and lower_attr.Get() is not None:
            jd.limit_lower = float(lower_attr.Get())
        if upper_attr and upper_attr.Get() is not None:
            jd.limit_upper = float(upper_attr.Get())

    jd.limit_effort = _get_float_attr(joint_prim, "urdf:limit:effort")

    jd.limit_velocity = _read_urdf_attr_or_physx(joint_prim, "urdf:limit:velocity", _read_physx_max_velocity)


def _read_dynamics(joint_prim: Usd.Prim, jd: JointData, actuator_map: dict[str, Usd.Prim] | None = None) -> None:
    """Read passive joint dynamics (damping, friction).

    Damping and friction are passive joint physical properties (GAP in
    UsdPhysics).  Only ``urdf:`` custom attrs and MuJoCo passive joint
    attrs (``mjc:damping``, ``mjc:frictionloss``) are valid sources.
    DriveAPI values are actuation data and belong in the
    ``isaac:source_drive`` breadcrumb, not here.

    Args:
        joint_prim: USD joint prim to read.
        jd: Joint data to populate.
        actuator_map: Mapping from joint prim paths to actuator prims.
    """
    damping = _get_float_attr(joint_prim, "urdf:dynamics:damping")
    if damping is None:
        damping = _get_float_attr(joint_prim, "mjc:damping")
    jd.dynamics_damping = damping

    friction = _read_urdf_attr_or_physx(joint_prim, "urdf:dynamics:friction", _read_physx_friction)
    if friction is None:
        friction = _get_float_attr(joint_prim, "mjc:frictionloss")
    jd.dynamics_friction = friction


def _read_calibration(joint_prim: Usd.Prim, jd: JointData) -> None:
    """Read calibration attributes.

    Args:
        joint_prim: USD joint prim to read.
        jd: Joint data to populate.
    """
    jd.calibration_rising = _get_float_attr(joint_prim, "urdf:calibration:rising")
    jd.calibration_falling = _get_float_attr(joint_prim, "urdf:calibration:falling")
    ref_pos = _get_float_attr(joint_prim, "urdf:calibration:reference_position")
    if ref_pos is None:
        ref_pos = _get_float_attr(joint_prim, "mjc:ref")
    jd.calibration_reference_position = ref_pos


def _read_safety_controller(joint_prim: Usd.Prim, jd: JointData) -> None:
    """Read safety controller attributes (only from urdf: custom attrs).

    Args:
        joint_prim: USD joint prim to read.
        jd: Joint data to populate.
    """
    jd.safety_k_velocity = _get_float_attr(joint_prim, "urdf:safety_controller:k_velocity")
    jd.safety_k_position = _get_float_attr(joint_prim, "urdf:safety_controller:k_position")
    jd.safety_soft_lower = _get_float_attr(joint_prim, "urdf:safety_controller:soft_lower_limit")
    jd.safety_soft_upper = _get_float_attr(joint_prim, "urdf:safety_controller:soft_upper_limit")


def _read_mimic(joint_prim: Usd.Prim, jd: JointData) -> None:
    """Read mimic joint data from NewtonMimicAPI or PhysxMimicJointAPI.

    Args:
        joint_prim: USD joint prim to read.
        jd: Joint data to populate.
    """
    if joint_prim.HasAPI("NewtonMimicAPI"):
        coef1 = _get_float_attr(joint_prim, "newton:mimicCoef1")
        coef0 = _get_float_attr(joint_prim, "newton:mimicCoef0")
        rel = joint_prim.GetRelationship("newton:mimicJoint")
        if rel and rel.IsValid():
            targets = rel.GetTargets()
            if targets:
                jd.mimic_joint = str(targets[0])
                jd.mimic_multiplier = coef1 if coef1 is not None else 1.0
                jd.mimic_offset = coef0 if coef0 is not None else 0.0
                return

    instance_name = _find_physx_mimic_instance(joint_prim)
    if instance_name is None:
        return

    ref_rel = joint_prim.GetRelationship(PhysxMimicRel.REFERENCE_JOINT.format(instance_name))
    if not ref_rel or not ref_rel.IsValid():
        return
    targets = ref_rel.GetTargets()
    if not targets:
        return
    jd.mimic_joint = str(targets[0])
    gearing_attr = joint_prim.GetAttribute(PhysxMimicAttr.GEARING.format(instance_name))
    offset_attr = joint_prim.GetAttribute(PhysxMimicAttr.OFFSET.format(instance_name))
    jd.mimic_multiplier = float(gearing_attr.Get()) if gearing_attr and gearing_attr.Get() is not None else 1.0
    jd.mimic_offset = float(offset_attr.Get()) if offset_attr and offset_attr.Get() is not None else 0.0


def _find_physx_mimic_instance(prim: Usd.Prim) -> str | None:
    """Find the PhysxMimicJointAPI instance name applied to a prim, if any.

    Multi-apply schemas appear in GetAppliedSchemas() as
    "PhysxMimicJointAPI:<instanceName>" (e.g. "PhysxMimicJointAPI:rotZ").

    Args:
        prim: USD prim to read.

    Returns:
        Mimic API instance name, or None if absent.
    """
    prefix = "PhysxMimicJointAPI:"
    for schema in prim.GetAppliedSchemas():
        s = str(schema)
        if s.startswith(prefix):
            return s[len(prefix) :]
    return None


def _resolve_mimic_names(joints: list[JointData], joint_name_map: dict[str, str], desc: RobotDescription) -> None:
    """Resolve mimic joint prim paths to URDF joint names.

    Args:
        joints: Joint data or prims to process.
        joint_name_map: Mapping from joint prim paths to URDF joint names.
        desc: Robot description to read.
    """
    for jd in joints:
        if jd.mimic_joint:
            jd.mimic_joint = joint_name_map.get(jd.mimic_joint, jd.mimic_joint)


# --- Multi-DOF joint expansion ---


def _analyze_multi_dof_axes(
    prim: Usd.Prim,
) -> list[tuple[str, bool, float, float]]:
    """Determine which axes of a D6/generic joint are free (unlocked).

    Inspects ``PhysicsLimitAPI`` multi-apply instances for each of the
    six possible axis tokens.  An axis is considered free when its
    low limit is strictly less than its high limit.

    Args:
        prim: USD prim to read.

    Returns:
        Free axes discovered on the joint.
    """
    free_axes: list[tuple[str, bool, float, float]] = []
    for token in _D6_AXIS_TOKENS:
        limit_api = UsdPhysics.LimitAPI.Get(prim, token)
        if not limit_api:
            continue
        low_attr = limit_api.GetLowAttr()
        high_attr = limit_api.GetHighAttr()
        if not low_attr or not high_attr:
            continue
        low = low_attr.Get()
        high = high_attr.Get()
        if low is None or high is None:
            continue
        try:
            low_f, high_f = float(low), float(high)
        except (TypeError, ValueError):
            continue
        if low_f < high_f:
            free_axes.append((token, token.startswith("rot"), low_f, high_f))
    return free_axes


def _read_joint_local_poses(
    joint: UsdPhysics.Joint,
) -> dict:
    """Read localPos0/1 and localRot0/1 as serialisable lists.

    Args:
        joint: Joint data or USD physics joint to read.

    Returns:
        Serializable local pose data.
    """
    result: dict = {}
    for idx in (0, 1):
        pos_attr = joint.GetLocalPos0Attr() if idx == 0 else joint.GetLocalPos1Attr()
        rot_attr = joint.GetLocalRot0Attr() if idx == 0 else joint.GetLocalRot1Attr()
        if pos_attr and pos_attr.Get() is not None:
            p = pos_attr.Get()
            result[f"local_pos{idx}"] = [float(p[0]), float(p[1]), float(p[2])]
        if rot_attr and rot_attr.Get() is not None:
            q = rot_attr.Get()
            result[f"local_rot{idx}"] = [
                float(q.GetReal()),
                float(q.GetImaginary()[0]),
                float(q.GetImaginary()[1]),
                float(q.GetImaginary()[2]),
            ]
    return result


def _read_per_axis_drives(prim: Usd.Prim, axis_tokens: list[str]) -> dict:
    """Read DriveAPI parameters for each axis token.

    Args:
        prim: USD prim to read.
        axis_tokens: Axis tokens to inspect.

    Returns:
        Drive parameters keyed by axis token.
    """
    drives: dict = {}
    for token in axis_tokens:
        if not prim.HasAPI(UsdPhysics.DriveAPI, token):
            continue
        drv = UsdPhysics.DriveAPI(prim, token)
        entry: dict = {}
        damp = drv.GetDampingAttr()
        if damp and damp.Get() is not None:
            entry["damping"] = float(damp.Get())
        stiff = drv.GetStiffnessAttr()
        if stiff and stiff.Get() is not None:
            entry["stiffness"] = float(stiff.Get())
        max_f = drv.GetMaxForceAttr()
        if max_f and max_f.Get() is not None:
            entry["max_force"] = float(max_f.Get())
        if entry:
            drives[token] = entry
    return drives


def _expand_multi_dof_joint(
    joint_prim: Usd.Prim,
    base_jd: JointData,
) -> tuple[list[JointData], list["LinkData"]]:
    """Expand a multi-DOF joint into a chain of single-DOF URDF joints.

    For a SphericalJoint the free axes are always rotX/rotY/rotZ.
    For D6 or generic joints the free axes come from ``_analyze_multi_dof_axes``.

    Args:
        joint_prim: USD joint prim to read.
        base_jd: Base joint data to expand.

    Returns:
        Expanded joint chain and generated ghost links.
    """
    from .link_reader import LinkData

    usd_type = str(joint_prim.GetTypeName())
    joint_name = base_jd.name

    if base_jd.joint_type == "spherical":
        free_axes: list[tuple[str, bool, float, float]] = [
            ("rotX", True, -180.0, 180.0),
            ("rotY", True, -180.0, 180.0),
            ("rotZ", True, -180.0, 180.0),
        ]
    else:
        free_axes = _analyze_multi_dof_axes(joint_prim)

    if not free_axes:
        return [], []

    joint_api = UsdPhysics.Joint(joint_prim)
    per_axis_limits: dict = {}
    for token, _is_rot, low, high in free_axes:
        per_axis_limits[token] = {"low": low, "high": high}

    axis_tokens = [fa[0] for fa in free_axes]
    per_axis_drives = _read_per_axis_drives(joint_prim, axis_tokens)
    local_poses = _read_joint_local_poses(joint_api)

    breadcrumb_params: dict = {
        "original_name": joint_name,
        "chain_joints": [f"{joint_name}_{fa[0]}" for fa in free_axes],
        "ghost_links": [f"{joint_name}_ghost_{i + 1}" for i in range(len(free_axes) - 1)],
        "per_axis_limits": per_axis_limits,
    }
    if per_axis_drives:
        breadcrumb_params["per_axis_drives"] = per_axis_drives
    breadcrumb_params.update(local_poses)

    n = len(free_axes)
    chain_joints: list[JointData] = []
    ghost_links: list[LinkData] = []

    for i, (token, is_rot, low, high) in enumerate(free_axes):
        jd = JointData()
        jd.name = f"{joint_name}_{token}"
        jd.joint_type = "revolute" if is_rot else "prismatic"
        jd.axis = _AXIS_TOKEN_TO_VECTOR[token]

        if is_rot:
            jd.limit_lower = math.radians(low)
            jd.limit_upper = math.radians(high)
        else:
            jd.limit_lower = low
            jd.limit_upper = high

        if i == 0:
            jd.parent_link = base_jd.parent_link
            jd.origin_xyz = base_jd.origin_xyz
            jd.origin_rpy = base_jd.origin_rpy
            jd.original_usd_type = usd_type
            jd.original_params = breadcrumb_params
        else:
            jd.parent_link = f"{joint_name}_ghost_{i}"
            jd.origin_xyz = (0.0, 0.0, 0.0)
            jd.origin_rpy = (0.0, 0.0, 0.0)

        if i == n - 1:
            jd.child_link = base_jd.child_link
        else:
            ghost_name = f"{joint_name}_ghost_{i + 1}"
            jd.child_link = ghost_name
            ghost_links.append(LinkData(name=ghost_name))

        chain_joints.append(jd)

    _logger.info(
        "Expanding %s '%s' into %d chained joints with %d ghost links",
        usd_type,
        joint_name,
        len(chain_joints),
        len(ghost_links),
    )

    return chain_joints, ghost_links


# --- Attribute reading helpers ---


def _get_float_attr(prim: Usd.Prim, attr_name: str) -> float | None:
    """Read a float attribute value, returning None if not present.

    Args:
        prim: USD prim to read.
        attr_name: Value to use.

    Returns:
        Float attribute value, or None if absent.
    """
    attr = prim.GetAttribute(attr_name)
    if attr and attr.IsValid():
        val = attr.Get()
        if val is not None:
            return float(val)
    return None


def _read_urdf_attr_or_physx(
    prim: Usd.Prim, urdf_attr: str, physx_fallback: Callable[[Usd.Prim], float | None]
) -> float | None:
    """Read from urdf: custom attr first, then PhysxJointAPI fallback.

    Args:
        prim: USD prim to read.
        urdf_attr: URDF custom attribute name.
        physx_fallback: Value to use.

    Returns:
        Attribute value from URDF metadata or PhysX fallback.
    """
    val = _get_float_attr(prim, urdf_attr)
    if val is not None:
        return val
    return physx_fallback(prim)


def _read_drive_max_force(prim: Usd.Prim) -> float | None:
    """Read maxForce from DriveAPI.

    Args:
        prim: USD prim to read.

    Returns:
        Drive maximum force, or None if absent.
    """
    instance = "angular" if prim.IsA(UsdPhysics.RevoluteJoint) else "linear"
    if prim.HasAPI(UsdPhysics.DriveAPI, instance):
        drive = UsdPhysics.DriveAPI(prim, instance)
        attr = drive.GetMaxForceAttr()
        if attr and attr.Get() is not None:
            return float(attr.Get())
    return None


def _read_drive_damping(prim: Usd.Prim) -> float | None:
    """Read damping from DriveAPI.

    Args:
        prim: USD prim to read.

    Returns:
        Drive damping, or None if absent.
    """
    instance = "angular" if prim.IsA(UsdPhysics.RevoluteJoint) else "linear"
    if prim.HasAPI(UsdPhysics.DriveAPI, instance):
        drive = UsdPhysics.DriveAPI(prim, instance)
        attr = drive.GetDampingAttr()
        if attr and attr.Get() is not None:
            return float(attr.Get())
    return None


def _read_drive_target_position(prim: Usd.Prim) -> float | None:
    """Read targetPosition from DriveAPI.

    Args:
        prim: USD prim to read.

    Returns:
        Drive target position, or None if absent.
    """
    instance = "angular" if prim.IsA(UsdPhysics.RevoluteJoint) else "linear"
    if prim.HasAPI(UsdPhysics.DriveAPI, instance):
        drive = UsdPhysics.DriveAPI(prim, instance)
        attr = drive.GetTargetPositionAttr()
        if attr and attr.Get() is not None:
            return float(attr.Get())
    return None


def _read_physx_max_velocity(prim: Usd.Prim) -> float | None:
    """Read maxJointVelocity from PhysxJointAPI (deg/s -> rad/s).

    Args:
        prim: USD prim to read.

    Returns:
        Maximum joint velocity, or None if absent.
    """
    attr = prim.GetAttribute(PhysxAttr.JOINT_MAX_VELOCITY.name)
    if attr and attr.Get() is not None:
        return float(attr.Get()) * math.pi / 180.0
    return None


def _read_physx_friction(prim: Usd.Prim) -> float | None:
    """Read jointFriction from PhysxJointAPI.

    Args:
        prim: USD prim to read.

    Returns:
        Joint friction, or None if absent.
    """
    attr = prim.GetAttribute(PhysxAttr.JOINT_FRICTION.name)
    if attr and attr.Get() is not None:
        return float(attr.Get())
    return None


# --- MuJoCo (mjc:*) fallback readers ---


def _read_mjc_effort(joint_prim: Usd.Prim, actuator_map: dict[str, Usd.Prim] | None) -> float | None:
    """Read effort limit from mjc: joint attrs or the associated MjcActuator.

    Checks mjc:actuatorfrcrange:max on the joint first, then
    mjc:forceRange:max on the MjcActuator targeting this joint.

    Args:
        joint_prim: USD joint prim to read.
        actuator_map: Mapping from joint prim paths to actuator prims.

    Returns:
        Effort limit, or None if absent.
    """
    val = _get_float_attr(joint_prim, "mjc:actuatorfrcrange:max")
    if val is not None:
        return val
    if actuator_map:
        actuator = actuator_map.get(str(joint_prim.GetPath()))
        if actuator is not None:
            return _get_float_attr(actuator, "mjc:forceRange:max")
    return None


def _read_actuator_damping(joint_prim: Usd.Prim, actuator_map: dict[str, Usd.Prim] | None) -> float | None:
    """Derive damping from the MjcActuator's gainPrm/biasPrm arrays.

    Only returns a value for supported gain/bias patterns (position PD
    or velocity control with gainType=fixed, biasType=affine).

    Args:
        joint_prim: USD joint prim to read.
        actuator_map: Mapping from joint prim paths to actuator prims.

    Returns:
        Actuator damping, or None if unavailable.
    """
    if not actuator_map:
        return None
    actuator = actuator_map.get(str(joint_prim.GetPath()))
    if actuator is None:
        return None
    _, damping = _decode_actuator_gains(actuator)
    return damping


def _decode_actuator_gains(actuator: Usd.Prim) -> tuple[float | None, float | None]:
    """Decode MuJoCo gainPrm/biasPrm into (stiffness, damping).

    Mirrors the conversion logic from the MJCF importer's
    ``convert_mjc_actuator_to_physics`` for the two supported patterns:

    Position PD control:
        gainPrm = [kp, 0, 0, ...], biasPrm = [0, -kp, -kd, ...]
        -> stiffness = kp, damping = kd

    Velocity control:
        gainPrm = [kd, 0, 0, ...], biasPrm = [0, 0, -kd, ...]
        -> stiffness = 0, damping = kd

    Returns (None, None) for unsupported or missing gain/bias data.

    Args:
        actuator: Actuator prim to read.

    Returns:
        Decoded stiffness and damping values.
    """
    gain_prm = _get_array_attr(actuator, "mjc:gainPrm")
    bias_prm = _get_array_attr(actuator, "mjc:biasPrm")
    if not gain_prm or len(gain_prm) < 3 or not bias_prm or len(bias_prm) < 3:
        return None, None

    gain_type = _get_token_attr(actuator, "mjc:gainType")
    bias_type = _get_token_attr(actuator, "mjc:biasType")
    if gain_type != "fixed" or bias_type != "affine":
        return None, None

    if (
        gain_prm[0] > 0
        and gain_prm[1] == 0
        and gain_prm[2] == 0
        and bias_prm[0] == 0
        and bias_prm[1] < 0
        and bias_prm[2] < 0
        and gain_prm[0] == -bias_prm[1]
    ):
        return float(gain_prm[0]), float(-bias_prm[2])

    if (
        gain_prm[0] > 0
        and gain_prm[1] == 0
        and gain_prm[2] == 0
        and bias_prm[0] == 0
        and bias_prm[1] == 0
        and bias_prm[2] < 0
        and gain_prm[0] == -bias_prm[2]
    ):
        return 0.0, float(-bias_prm[2])

    return None, None


def _get_array_attr(prim: Usd.Prim, attr_name: str) -> object | None:
    """Read an array attribute, returning None if not authored.

    Args:
        prim: USD prim to read.
        attr_name: Value to use.

    Returns:
        Array attribute value, or None if absent.
    """
    attr = prim.GetAttribute(attr_name)
    if attr and attr.IsValid():
        return attr.Get()
    return None


def _get_token_attr(prim: Usd.Prim, attr_name: str) -> str | None:
    """Read a token/string attribute, returning None if not authored.

    Args:
        prim: USD prim to read.
        attr_name: Value to use.

    Returns:
        Token attribute value, or None if absent.
    """
    attr = prim.GetAttribute(attr_name)
    if attr and attr.IsValid():
        val = attr.Get()
        if val is not None:
            return str(val)
    return None


# --- Source drive breadcrumb collection ---


def _read_armature(joint_prim: Usd.Prim) -> float | None:
    """Read armature (reflected rotor inertia) from PhysxJointAPI or mjc: attr.

    Args:
        joint_prim: USD joint prim to read.

    Returns:
        Armature value, or None if absent.
    """
    attr = joint_prim.GetAttribute(PhysxAttr.JOINT_ARMATURE.name)
    if attr and attr.Get() is not None:
        return float(attr.Get())
    return _get_float_attr(joint_prim, "mjc:armature")


def _read_actuator_attrs(actuator: Usd.Prim) -> dict:
    """Serialize all gain-relevant attributes from an MjcActuator prim.

    Args:
        actuator: Actuator prim to read.

    Returns:
        Serialized actuator attributes.
    """
    result: dict = {}
    gain_prm = _get_array_attr(actuator, "mjc:gainPrm")
    if gain_prm is not None:
        result["gainPrm"] = [float(v) for v in gain_prm]
    bias_prm = _get_array_attr(actuator, "mjc:biasPrm")
    if bias_prm is not None:
        result["biasPrm"] = [float(v) for v in bias_prm]
    gain_type = _get_token_attr(actuator, "mjc:gainType")
    if gain_type is not None:
        result["gainType"] = gain_type
    bias_type = _get_token_attr(actuator, "mjc:biasType")
    if bias_type is not None:
        result["biasType"] = bias_type
    fr_min = _get_float_attr(actuator, "mjc:forceRange:min")
    if fr_min is not None:
        result["forceRange_min"] = fr_min
    fr_max = _get_float_attr(actuator, "mjc:forceRange:max")
    if fr_max is not None:
        result["forceRange_max"] = fr_max
    return result


def _read_drive_attrs(prim: Usd.Prim, instance: str) -> dict:
    """Serialize authored DriveAPI attributes for a given instance.

    Args:
        prim: USD prim to read.
        instance: Value to use.

    Returns:
        Serialized drive attributes.
    """
    if not prim.HasAPI(UsdPhysics.DriveAPI, instance):
        return {}
    drv = UsdPhysics.DriveAPI(prim, instance)
    result: dict = {}
    stiff = drv.GetStiffnessAttr()
    if stiff and stiff.Get() is not None:
        result["stiffness"] = float(stiff.Get())
    damp = drv.GetDampingAttr()
    if damp and damp.Get() is not None:
        result["damping"] = float(damp.Get())
    max_f = drv.GetMaxForceAttr()
    if max_f and max_f.Get() is not None:
        result["max_force"] = float(max_f.Get())
    tgt = drv.GetTargetPositionAttr()
    if tgt and tgt.Get() is not None:
        result["target_position"] = float(tgt.Get())
    return result


def _collect_source_drive_breadcrumb(
    joint_prim: Usd.Prim,
    actuator_map: dict[str, Usd.Prim] | None,
) -> dict | None:
    """Snapshot actuation state for the ``isaac:source_drive`` breadcrumb.

    MuJoCo MjcActuator takes precedence over PhysX DriveAPI.
    Returns None when there is nothing to preserve.

    Args:
        joint_prim: USD joint prim to read.
        actuator_map: Mapping from joint prim paths to actuator prims.

    Returns:
        Source drive breadcrumb data, or None if absent.
    """
    is_revolute = joint_prim.IsA(UsdPhysics.RevoluteJoint)
    is_prismatic = joint_prim.IsA(UsdPhysics.PrismaticJoint)
    if not is_revolute and not is_prismatic:
        return None

    instance = "angular" if is_revolute else "linear"
    armature = _read_armature(joint_prim)

    if actuator_map:
        actuator = actuator_map.get(str(joint_prim.GetPath()))
        if actuator is not None:
            act_attrs = _read_actuator_attrs(actuator)
            if act_attrs or armature is not None:
                meta: dict = {"source": "mujoco", "actuator": act_attrs}
                if armature is not None:
                    meta["armature"] = armature
                return meta

    drive_attrs = _read_drive_attrs(joint_prim, instance)
    if drive_attrs or armature is not None:
        meta = {"source": "physx", "instance": instance}
        if drive_attrs:
            meta["drive"] = drive_attrs
        if armature is not None:
            meta["armature"] = armature
        return meta

    return None
