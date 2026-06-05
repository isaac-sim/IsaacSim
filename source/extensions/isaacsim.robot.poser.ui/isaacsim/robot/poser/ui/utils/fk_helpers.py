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

"""Shared FK/robot utility functions for the Robot Poser extension.

Extracted from ui_builder.py for reuse by the property panel and other
components that need FK computation, site candidate collection, or robot
ancestor resolution.
"""

from __future__ import annotations

import carb
import isaacsim.robot.poser.robot_poser as robot_poser
import numpy as np
import omni.usd
import usd.schema.isaac.robot_schema.math as poser_math
from pxr import Gf, Sdf, Usd, UsdGeom, UsdPhysics
from usd.schema.isaac.robot_schema import Relations
from usd.schema.isaac.robot_schema import utils as robot_schema_utils
from usd.schema.isaac.robot_schema.kinematic_chain import KinematicChain

NAMED_POSES_SCOPE = robot_poser.NAMED_POSES_SCOPE


def force_manipulator_refresh() -> None:
    """Ask the joint-connection manipulator to redraw immediately.

    Soft-imports ``isaacsim.robot.schema.ui`` so that this module has no hard
    dependency on it.  The manipulator recognises the force-redraw sentinel and
    skips its normal 500 ms debounce, updating the viewport lines on the next
    rendered frame.
    """
    try:
        from isaacsim.robot.schema.ui.scene import ConnectionInstance  # type: ignore[import]

        inst = ConnectionInstance.get_instance()
        if inst is not None and inst.model is not None:
            inst.model.force_rebuild()
    except Exception:
        pass


# ---------------------------------------------------------------------------
# Robot ancestor resolution
# ---------------------------------------------------------------------------


def find_robot_ancestor(stage: Usd.Stage, prim: Usd.Prim) -> Usd.Prim | None:
    """Walk up the prim hierarchy to find the nearest ancestor with IsaacRobotAPI.

    Args:
        stage: The USD stage.
        prim: The starting prim (typically an IsaacNamedPose).

    Returns:
        The nearest ancestor carrying IsaacRobotAPI, or None.
    """
    current = prim.GetParent()
    while current and current.IsValid() and current.GetPath() != Sdf.Path("/"):
        if robot_poser.validate_robot_schema(current):
            return current
        current = current.GetParent()
    return None


# ---------------------------------------------------------------------------
# Site candidate collection
# ---------------------------------------------------------------------------


def get_site_candidates(stage: Usd.Stage, robot_prim: Usd.Prim) -> list[str]:
    """Collect site candidate paths for a robot prim.

    Uses robot schema GetAllRobotLinks, then IsaacSiteAPI/ReferencePointAPI,
    then Xform fallback (excluding NamedPoses scope).

    Args:
        stage: The USD stage.
        robot_prim: The robot root prim.

    Returns:
        Sorted list of prim path strings.
    """
    candidates: set = set()

    # 1. Links and sites via robot schema GetAllRobotLinks (recursively expands sub-robots)
    if robot_prim.HasAPI(robot_schema_utils.Classes.ROBOT_API.value):
        for prim in robot_schema_utils.GetAllRobotLinks(stage, robot_prim, include_reference_points=True):
            if prim and prim.IsValid():
                candidates.add(str(prim.GetPath()))

    # 2. Sites (IsaacSiteAPI / IsaacReferencePointAPI)
    for p in Usd.PrimRange(robot_prim):
        schemas = p.GetAppliedSchemas()
        for s in schemas:
            if "SiteAPI" in s or "ReferencePointAPI" in s:
                candidates.add(str(p.GetPath()))
                break

    # 3. Fallback: Xform body prims excluding NamedPoses scope
    if not candidates:
        robot_path = str(robot_prim.GetPath())
        for p in Usd.PrimRange(robot_prim):
            path = str(p.GetPath())
            if path == robot_path:
                continue
            if f"/{NAMED_POSES_SCOPE}" in path:
                continue
            if p.IsA(UsdGeom.Xform) and not p.IsA(UsdGeom.Mesh):
                candidates.add(path)

    return sorted(candidates)


# ---------------------------------------------------------------------------
# FK computation and transform writing
# ---------------------------------------------------------------------------


def compute_fk_and_write_transform(
    stage: Usd.Stage,
    robot_prim: Usd.Prim,
    pose_prim_path: str,
    pose: robot_poser.PoseResult,
) -> bool:
    """Compute FK for stored joint values and update the named-pose prim transform.

    Composes body0_pose @ FK(q) in robot-base frame; authors translate and orient only.

    Args:
        stage: The USD stage.
        robot_prim: The robot root prim.
        pose_prim_path: Path of the IsaacNamedPose prim to update.
        pose: Named-pose data with joint values and start/end link paths.

    Returns:
        True on success.
    """
    if not pose.start_link or not pose.end_link:
        return False

    start_prim = stage.GetPrimAtPath(pose.start_link)
    end_prim = stage.GetPrimAtPath(pose.end_link)
    if not start_prim or not start_prim.IsValid():
        return False
    if not end_prim or not end_prim.IsValid():
        return False

    chain = KinematicChain(stage, robot_prim, start_prim, end_prim)
    if not chain.joints:
        carb.log_warn("Robot Poser: build_ik_joint_chain returned empty chain")
        return False

    first_jprim = stage.GetPrimAtPath(chain.joints[0].prim_path)
    body0_targets = UsdPhysics.Joint(first_jprim).GetBody0Rel().GetTargets()
    if not body0_targets:
        carb.log_warn("Robot Poser: could not resolve body0 for first chain joint")
        return False
    body0_prim = stage.GetPrimAtPath(body0_targets[0])
    if not body0_prim or not body0_prim.IsValid():
        return False

    fk_chain = KinematicChain(stage, robot_prim, body0_prim, end_prim)
    if not fk_chain.joints:
        carb.log_warn("Robot Poser: could not build FK chain from body0 to end")
        return False

    fk_q = np.array([pose.joints.get(j.prim_path, 0.0) for j in fk_chain.joints], dtype=float)
    body0_pose = poser_math._prim_pose_in_robot_frame(robot_prim, body0_prim)
    T_fk, _ = fk_chain.compute_fk(fk_q)
    T_site = body0_pose @ T_fk

    pose_prim = stage.GetPrimAtPath(pose_prim_path)
    if not pose_prim or not pose_prim.IsValid():
        return False

    write_transform_to_prim(pose_prim, T_site)
    return True


def write_transform_to_prim(prim: Usd.Prim, T: poser_math.Transform) -> None:
    """Author translate and orient xform ops on prim; do not modify scale.

    Reuses existing translate/orient ops from the resolved xformOpOrder when
    they are already present, and only calls Add* when an op is missing.
    This avoids the USD error raised by AddXformOp when the op name already
    appears in the xformOpOrder authored in any USD layer (ClearXformOpOrder
    only clears the opinion on the current edit target and cannot block
    opinions from stronger layers).

    Args:
        prim: The prim to author xform ops on.
        T: Transform (translation and quaternion).
    """
    xformable = UsdGeom.Xformable(prim)

    translate_op = None
    orient_op = None
    for op in xformable.GetOrderedXformOps():
        if op.IsInverseOp():
            continue
        op_type = op.GetOpType()
        if op_type == UsdGeom.XformOp.TypeTranslate and translate_op is None:
            translate_op = op
        elif op_type == UsdGeom.XformOp.TypeOrient and orient_op is None:
            orient_op = op

    if translate_op is None:
        translate_op = xformable.AddTranslateOp(UsdGeom.XformOp.PrecisionDouble)
    if orient_op is None:
        orient_op = xformable.AddOrientOp(UsdGeom.XformOp.PrecisionDouble)

    t = T.t
    q = T.q

    # Set translate value — match the existing op's precision to avoid type errors.
    if translate_op.GetPrecision() == UsdGeom.XformOp.PrecisionFloat:
        translate_op.Set(Gf.Vec3f(float(t[0]), float(t[1]), float(t[2])))
    else:
        translate_op.Set(Gf.Vec3d(float(t[0]), float(t[1]), float(t[2])))

    # Set orient value — match the existing op's precision to avoid type errors.
    if orient_op.GetPrecision() == UsdGeom.XformOp.PrecisionFloat:
        orient_op.Set(Gf.Quatf(float(q[0]), float(q[1]), float(q[2]), float(q[3])))
    else:
        orient_op.Set(Gf.Quatd(float(q[0]), float(q[1]), float(q[2]), float(q[3])))


# ---------------------------------------------------------------------------
# Joint info helpers
# ---------------------------------------------------------------------------


def read_joint_limits_native(stage: Usd.Stage, joint_prim_path: str) -> tuple[str, bool, float, float]:
    """Read joint display name, type, and limits in native USD units.

    Args:
        stage: The USD stage.
        joint_prim_path: Path of the joint prim.

    Returns:
        (display_name, is_revolute, lower, upper). Lower/upper in degrees
        (revolute) or meters (prismatic); (-inf, inf) for unlimited.
    """
    display_name = joint_prim_path.rsplit("/", 1)[-1] if "/" in joint_prim_path else joint_prim_path

    jprim = stage.GetPrimAtPath(joint_prim_path)
    if not jprim or not jprim.IsValid():
        return display_name, False, -np.inf, np.inf

    is_rev = jprim.IsA(UsdPhysics.RevoluteJoint)
    is_pri = jprim.IsA(UsdPhysics.PrismaticJoint)

    lo: float = -np.inf
    hi: float = np.inf
    if is_rev:
        rev = UsdPhysics.RevoluteJoint(jprim)
        lo_attr = rev.GetLowerLimitAttr()
        hi_attr = rev.GetUpperLimitAttr()
        if lo_attr and lo_attr.Get() is not None:
            lo = float(lo_attr.Get())  # already in degrees
        if hi_attr and hi_attr.Get() is not None:
            hi = float(hi_attr.Get())
    elif is_pri:
        pri = UsdPhysics.PrismaticJoint(jprim)
        lo_attr = pri.GetLowerLimitAttr()
        hi_attr = pri.GetUpperLimitAttr()
        if lo_attr and lo_attr.Get() is not None:
            lo = float(lo_attr.Get())
        if hi_attr and hi_attr.Get() is not None:
            hi = float(hi_attr.Get())

    # USD convention: lower > upper means unlimited
    if lo > hi:
        lo, hi = -np.inf, np.inf

    return display_name, is_rev, lo, hi


# ---------------------------------------------------------------------------
# Build a PoseResult from current robot joint state
# ---------------------------------------------------------------------------


def build_pose_from_current_joints(
    stage: Usd.Stage,
    robot_prim: Usd.Prim,
    start_link: str,
    end_link: str,
) -> robot_poser.PoseResult | None:
    """Build a PoseResult from the robot's current joint configuration.

    Reads current joint state for the chain start_link -> end_link and computes FK.

    Args:
        stage: The USD stage.
        robot_prim: The robot root prim.
        start_link: Prim path of the chain start link.
        end_link: Prim path of the chain end link / site.

    Returns:
        PoseResult on success, None when the joint chain cannot be built.
    """
    start_prim = stage.GetPrimAtPath(start_link)
    end_prim = stage.GetPrimAtPath(end_link)
    if not start_prim or not start_prim.IsValid() or not end_prim or not end_prim.IsValid():
        return None

    # 1. Build joint chain (start -> end)
    chain = KinematicChain(stage, robot_prim, start_prim, end_prim)
    if not chain.joints:
        carb.log_warn("Robot Poser: could not build joint chain between selected sites")
        return None

    # 2. Read current joint values from USD for the chain joints
    joint_dict = {}
    fixed_dict = {}
    for j in chain.joints:
        jprim = stage.GetPrimAtPath(j.prim_path)
        val = 0.0
        if jprim and jprim.IsValid():
            if jprim.IsA(UsdPhysics.RevoluteJoint):
                attr = jprim.GetAttribute("state:angular:physics:position")
                if attr and attr.Get() is not None:
                    val = float(np.radians(float(attr.Get())))
            elif jprim.IsA(UsdPhysics.PrismaticJoint):
                attr = jprim.GetAttribute("state:linear:physics:position")
                if attr and attr.Get() is not None:
                    val = float(attr.Get())
        joint_dict[j.prim_path] = val
        fixed_dict[j.prim_path] = False

    # 3. Compute FK to derive the end-site transform
    first_jprim = stage.GetPrimAtPath(chain.joints[0].prim_path)
    body0_targets = UsdPhysics.Joint(first_jprim).GetBody0Rel().GetTargets()
    if not body0_targets:
        carb.log_warn("Robot Poser: could not resolve body0 for first chain joint")
        return None
    body0_prim = stage.GetPrimAtPath(body0_targets[0])
    if not body0_prim or not body0_prim.IsValid():
        return None

    fk_chain = KinematicChain(stage, robot_prim, body0_prim, end_prim)
    if not fk_chain.joints:
        carb.log_warn("Robot Poser: could not build FK chain from body0 to end site")
        return None

    fk_q = np.array([joint_dict.get(j.prim_path, 0.0) for j in fk_chain.joints], dtype=float)
    body0_pose = poser_math._prim_pose_in_robot_frame(robot_prim, body0_prim)
    T_fk, _ = fk_chain.compute_fk(fk_q)
    T_site = body0_pose @ T_fk

    return robot_poser.PoseResult(
        success=True,
        joints=joint_dict,
        joint_fixed=fixed_dict,
        start_link=start_link,
        end_link=end_link,
        target_position=T_site.t.tolist(),
        target_orientation=T_site.q.tolist(),
    )


# ---------------------------------------------------------------------------
# IK tracking cache
# ---------------------------------------------------------------------------


def build_tracking_cache(
    stage: Usd.Stage,
    robot_prim: Usd.Prim,
    pose_prim_path: str,
) -> dict | None:
    """Build a one-time IK tracking cache for a named-pose prim.

    Cache contains pose_prim and a RobotPoser instance for the pose's chain.

    Args:
        stage: The USD stage.
        robot_prim: The robot root prim.
        pose_prim_path: Path of the IsaacNamedPose prim.

    Returns:
        Dict with 'pose_prim' and 'poser' keys, or None when cache cannot be built.
    """
    pose_prim = stage.GetPrimAtPath(pose_prim_path)
    if not pose_prim or not pose_prim.IsValid():
        return None

    pose = robot_poser.get_named_pose(stage, robot_prim, pose_prim.GetName())
    if not pose or not pose.success:
        return None

    start_prim = stage.GetPrimAtPath(pose.start_link) if pose.start_link else None
    end_prim = stage.GetPrimAtPath(pose.end_link) if pose.end_link else None
    if not start_prim or not start_prim.IsValid():
        return None
    if not end_prim or not end_prim.IsValid():
        return None

    poser = robot_poser.RobotPoser(stage, robot_prim, start_prim, end_prim)
    if not poser.joints:
        return None
    poser.set_seed(dict(pose.joints))

    return {
        "pose_prim": pose_prim,
        "poser": poser,
    }


# ---------------------------------------------------------------------------
# IK solve from cached tracking data
# ---------------------------------------------------------------------------


def solve_ik_from_cache(
    cache: dict,
) -> tuple[dict[str, float], list[float]] | None:
    """Solve IK from the cached named-pose prim's current xform transform.

    Args:
        cache: Dict from build_tracking_cache (pose_prim, poser).

    Returns:
        (result_joints, joint_values_native) on convergence; result_joints in
        radians, joint_values_native in degrees/meters for isaac:robot:pose:jointValues.
        None when IK does not converge.
    """
    pose_prim = cache["pose_prim"]
    if not pose_prim.IsValid():
        return None

    t_attr = pose_prim.GetAttribute("xformOp:translate")
    q_attr = pose_prim.GetAttribute("xformOp:orient")
    if not t_attr or not q_attr:
        return None
    t_val = t_attr.Get()
    q_val = q_attr.Get()
    if t_val is None or q_val is None:
        return None

    img = q_val.GetImaginary()
    target = poser_math.Transform(
        t=[float(t_val[0]), float(t_val[1]), float(t_val[2])],
        q=[float(q_val.GetReal()), float(img[0]), float(img[1]), float(img[2])],
    )

    # Read joint fixed flags from the named pose prim (same order as POSE_JOINTS).
    joint_fixed_dict: dict[str, bool] = {}
    joints_rel = pose_prim.GetRelationship(Relations.POSE_JOINTS.name)
    if joints_rel:
        joint_paths = [str(p) for p in joints_rel.GetTargets()]
        fixed_flags = robot_schema_utils.GetNamedPoseJointFixed(pose_prim) or []
        joint_fixed_dict = {
            path: bool(fixed_flags[i]) if i < len(fixed_flags) else False for i, path in enumerate(joint_paths)
        }
    poser: robot_poser.RobotPoser = cache["poser"]
    result = poser.solve_ik(target, joint_fixed=joint_fixed_dict)
    if not result.success:
        return None

    joint_values_native = poser.joints_to_native_values(result.joints)
    return result.joints, joint_values_native


# ---------------------------------------------------------------------------
# IK failure outline helpers
# ---------------------------------------------------------------------------

# Module-level outline group id (shared by all callers in this process).
_outline_group_id: int | None = None


def ensure_outline_group() -> int:
    """Register (once) and return the selection-group id for IK failure outlines.

    Returns:
        The selection group id used for red outline highlighting.
    """
    global _outline_group_id
    if _outline_group_id is None:
        ctx = omni.usd.get_context()
        _outline_group_id = ctx.register_selection_group()
        ctx.set_selection_group_outline_color(_outline_group_id, (1.0, 0.0, 0.0, 1.0))
        ctx.set_selection_group_shade_color(_outline_group_id, (1.0, 0.0, 0.0, 0.2))
    return _outline_group_id


def apply_ik_chain_outline(
    stage: Usd.Stage,
    caches: list[dict | None],
    current_outlined: list[str],
) -> list[str]:
    """Apply red outline to Gprim descendants of IK chain link prims.

    Args:
        stage: The USD stage.
        caches: One or more tracking cache dicts (each has 'poser' with joints).
        current_outlined: Prim paths currently outlined from a previous call.

    Returns:
        Updated list of outlined prim paths.
    """
    link_paths: set = set()
    for cache in caches:
        if cache is None:
            continue
        for j in cache["poser"].joints:
            jprim = stage.GetPrimAtPath(j.prim_path)
            if not jprim or not jprim.IsValid():
                continue
            joint = UsdPhysics.Joint(jprim)
            for rel in (joint.GetBody0Rel(), joint.GetBody1Rel()):
                for target in rel.GetTargets():
                    link_paths.add(target)

    if not link_paths:
        return current_outlined

    desired: set = set()
    for lp in link_paths:
        link_prim = stage.GetPrimAtPath(lp)
        if not link_prim or not link_prim.IsValid():
            continue
        for prim in Usd.PrimRange(
            link_prim,
            Usd.TraverseInstanceProxies(Usd.PrimAllPrimsPredicate),
        ):
            if prim.IsA(UsdGeom.Gprim):
                desired.add(str(prim.GetPath()))

    current = set(current_outlined)
    if desired == current:
        return current_outlined

    ctx = omni.usd.get_context()
    group_id = ensure_outline_group()

    for p in current - desired:
        ctx.set_selection_group(0, p)
    for p in desired - current:
        ctx.set_selection_group(group_id, p)

    return list(desired)


def clear_ik_chain_outline(current_outlined: list[str]) -> list[str]:
    """Remove the red outline from previously outlined prims.

    Args:
        current_outlined: Prim paths to clear from the outline group.

    Returns:
        Empty list.
    """
    if not current_outlined:
        return []
    ctx = omni.usd.get_context()
    for p in current_outlined:
        ctx.set_selection_group(0, p)
    return []
