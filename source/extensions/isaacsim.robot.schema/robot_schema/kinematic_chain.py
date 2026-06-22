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

"""Kinematic chain implementation for robot FK computation and USD application.

This module owns all kinematic-chain logic: joint-chain construction, FK
computation, Jacobian, and USD body-transform propagation.
:class:`KinematicChain` is the primary public interface.

:mod:`usd.schema.isaac.robot_schema.math` provides only data structures and
pure math utilities (:class:`~usd.schema.isaac.robot_schema.math.Transform`,
:class:`~usd.schema.isaac.robot_schema.math.Joint`, quaternion ops, etc.).
"""

from __future__ import annotations

from typing import Any

import numpy as np
from usd.schema.isaac.robot_schema import Classes
from usd.schema.isaac.robot_schema.math import (
    Joint,
    Mat,
    Transform,
    Vec3,
    VecN,
    _mat4_to_transform,
    _prim_pose_in_robot_frame,
    quat_mul,
    quat_rotate,
    quat_to_matrix,
)
from usd.schema.isaac.robot_schema.utils import (
    GenerateRobotLinkTree,
    GetJointPose,
    _collect_chain_joints,
    _compute_zero_config_poses,
    _find_tree_node,
    _get_joint_local_transform,
    _get_world_transform_matrix,
)

_AXIS_VEC: dict[str, np.ndarray] = {
    "X": np.array([1, 0, 0], dtype=float),
    "Y": np.array([0, 1, 0], dtype=float),
    "Z": np.array([0, 0, 1], dtype=float),
}

# ---------------------------------------------------------------------------
# Joint-type helpers and motion matrix
# ---------------------------------------------------------------------------

_AXIS_MAP_GF: dict[str, Any] = {}


def _lazy_axis_map() -> dict[str, Any]:
    """Return the axis-string to Gf.Vec3d mapping.

    Returns:
        Mapping from axis token strings to vector values.
    """
    global _AXIS_MAP_GF
    if not _AXIS_MAP_GF:
        import pxr

        _AXIS_MAP_GF = {
            "X": pxr.Gf.Vec3d(1, 0, 0),
            "Y": pxr.Gf.Vec3d(0, 1, 0),
            "Z": pxr.Gf.Vec3d(0, 0, 1),
        }
    return _AXIS_MAP_GF


def _joint_is_revolute(prim: Any) -> bool:
    import pxr

    return prim.IsA(pxr.UsdPhysics.RevoluteJoint)


def _joint_is_prismatic(prim: Any) -> bool:
    import pxr

    return prim.IsA(pxr.UsdPhysics.PrismaticJoint)


def _joint_motion_matrix(joint_prim: Any, q_value: float) -> Any:
    """Return the pxr.Gf.Matrix4d motion transform for joint_prim at q_value.

    Args:
        joint_prim: USD joint prim (RevoluteJoint or PrismaticJoint).
        q_value: Joint value in radians (revolute) or meters (prismatic).

    Returns:
        pxr.Gf.Matrix4d representing the joint motion.

    """
    import pxr

    axis_map = _lazy_axis_map()
    mat = pxr.Gf.Matrix4d()
    mat.SetIdentity()
    if _joint_is_revolute(joint_prim):
        ax_str = str(pxr.UsdPhysics.RevoluteJoint(joint_prim).GetAxisAttr().Get())
        mat.SetRotate(pxr.Gf.Rotation(axis_map.get(ax_str, axis_map["X"]), float(np.degrees(q_value))))
    elif _joint_is_prismatic(joint_prim):
        ax_str = str(pxr.UsdPhysics.PrismaticJoint(joint_prim).GetAxisAttr().Get())
        mat.SetTranslate(axis_map.get(ax_str, axis_map["X"]) * float(q_value))
    return mat


# ---------------------------------------------------------------------------
# USD body-transform propagation helpers
# ---------------------------------------------------------------------------


def _apply_world_transform(prim: Any, world_mat: Any) -> None:
    """Set prim's local Xform so its world transform equals world_mat.

    Decomposes into translate, orient, scale xformOps, preserving existing
    precision settings to avoid USD mismatch warnings.

    Args:
        prim: USD prim to update.
        world_mat: Desired world-space pxr.Gf.Matrix4d.

    """
    import pxr

    parent = prim.GetParent()
    if parent and parent.IsValid():
        parent_world = pxr.Gf.Matrix4d(_get_world_transform_matrix(parent))
        local = world_mat * parent_world.GetInverse()
    else:
        local = pxr.Gf.Matrix4d(world_mat)

    translate = local.ExtractTranslation()
    quat = local.ExtractRotation().GetQuat()
    orient_quatd = pxr.Gf.Quatd(quat.GetReal(), quat.GetImaginary())
    scale_vec3d = pxr.Gf.Vec3d(1.0, 1.0, 1.0)

    xformable = pxr.UsdGeom.Xformable(prim)
    prec_d = pxr.UsdGeom.XformOp.PrecisionDouble
    prec_f = pxr.UsdGeom.XformOp.PrecisionFloat
    prec_t = prec_d
    prec_o = prec_d
    prec_s = prec_d
    for op in xformable.GetOrderedXformOps():
        t = op.GetOpType()
        p = op.GetPrecision()
        if t == pxr.UsdGeom.XformOp.TypeTranslate:
            prec_t = p
        elif t == pxr.UsdGeom.XformOp.TypeOrient:
            prec_o = p
        elif t == pxr.UsdGeom.XformOp.TypeScale:
            prec_s = p

    xformable.ClearXformOpOrder()
    # Convert to single precision only when the existing xformOp uses PrecisionFloat.
    xformable.AddTranslateOp(prec_t).Set(pxr.Gf.Vec3f(translate) if prec_t == prec_f else translate)
    xformable.AddOrientOp(prec_o).Set(pxr.Gf.Quatf(orient_quatd) if prec_o == prec_f else orient_quatd)
    xformable.AddScaleOp(prec_s).Set(pxr.Gf.Vec3f(scale_vec3d) if prec_s == prec_f else scale_vec3d)


def _read_subtree_transforms(node: Any, transforms: dict[str, Any]) -> None:
    """Snapshot world transforms for every body in the kinematic subtree.

    Args:
        node: Robot link tree node (from GenerateRobotLinkTree).
        transforms: Dict to populate: prim-path str → pxr.Gf.Matrix4d.

    """
    import pxr

    transforms[str(node.prim.GetPath())] = pxr.Gf.Matrix4d(_get_world_transform_matrix(node.prim))
    for child in node.children:
        _read_subtree_transforms(child, transforms)


def _rigid_propagate(node: Any, old_transforms: dict[str, Any]) -> None:
    """Preserve a body's relative transform to its parent after the parent moved.

    Args:
        node: Robot link tree node to reposition.
        old_transforms: World-transform snapshot taken before any parent moves.

    """
    import pxr

    parent_path = str(node.parent.prim.GetPath())
    node_path = str(node.prim.GetPath())
    parent_new = pxr.Gf.Matrix4d(_get_world_transform_matrix(node.parent.prim))
    parent_old = old_transforms.get(parent_path, parent_new)
    body_old = old_transforms.get(node_path, pxr.Gf.Matrix4d(_get_world_transform_matrix(node.prim)))
    _apply_world_transform(node.prim, body_old * parent_old.GetInverse() * parent_new)


def _fk_propagate_node(
    stage: Any,
    node: Any,
    joint_dict: dict[str, float],
    old_transforms: dict[str, Any],
    affected: bool,
) -> None:
    """Recursively compute and apply FK body transforms through the kinematic tree.

    For a joint listed in joint_dict, the child-body world transform is computed
    from the parent body, joint local frames, and the joint value. For an
    unspecified joint whose ancestor was already moved, rigid propagation
    preserves the relative offset.

    Args:
        stage: USD stage.
        node: Robot link tree node.
        joint_dict: Joint prim-path to value (radians or meters).
        old_transforms: World-transform snapshot taken before any modifications.
        affected: True when an ancestor was already repositioned.

    """
    import pxr

    joint_prim = node._joint_to_parent
    node_affected = affected

    if joint_prim is not None:
        joint_path = str(joint_prim.GetPath())

        if joint_path in joint_dict:
            parent_world = pxr.Gf.Matrix4d(_get_world_transform_matrix(node.parent.prim))
            joint_usd = pxr.UsdPhysics.Joint(joint_prim)
            local0 = _get_joint_local_transform(joint_usd, 0)
            local1 = _get_joint_local_transform(joint_usd, 1)

            if local0 is not None and local1 is not None:
                motion = _joint_motion_matrix(joint_prim, joint_dict[joint_path])
                body_new = pxr.Gf.Matrix4d(local1).GetInverse() * motion * pxr.Gf.Matrix4d(local0) * parent_world
                _apply_world_transform(node.prim, body_new)
                node_affected = True

        elif affected:
            _rigid_propagate(node, old_transforms)
            node_affected = True
    elif affected:
        _rigid_propagate(node, old_transforms)
        node_affected = True

    for child in node.children:
        _fk_propagate_node(stage, child, joint_dict, old_transforms, node_affected)


# ---------------------------------------------------------------------------
# One-shot joint-attribute writer (no KinematicChain required)
# ---------------------------------------------------------------------------


def _set_joint_attributes_stage(stage: Any, joint_dict: dict[str, float]) -> None:
    """Write drive-target and physics-state attributes to USD joint prims.

    Args:
        stage: USD stage.
        joint_dict: Joint prim-path to value (radians or meters).

    """
    import pxr

    for joint_path, value in joint_dict.items():
        prim = stage.GetPrimAtPath(joint_path)
        if not prim or not prim.IsValid():
            continue
        if _joint_is_revolute(prim):
            deg = float(np.degrees(value))
            for attr_name in ("drive:angular:physics:targetPosition", "state:angular:physics:position"):
                attr = prim.GetAttribute(attr_name)
                if not attr:
                    attr = prim.CreateAttribute(attr_name, pxr.Sdf.ValueTypeNames.Float)
                attr.Set(deg)
        elif _joint_is_prismatic(prim):
            for attr_name in ("drive:linear:physics:targetPosition", "state:linear:physics:position"):
                attr = prim.GetAttribute(attr_name)
                if not attr:
                    attr = prim.CreateAttribute(attr_name, pxr.Sdf.ValueTypeNames.Float)
                attr.Set(float(value))


# ---------------------------------------------------------------------------
# KinematicChain
# ---------------------------------------------------------------------------


class KinematicChain:
    """A cached kinematic chain between two robot links.

    Builds the kinematic tree once at construction and optionally builds
    the joint chain when start/end prims are provided. When constructed
    with only stage and robot_prim, the cached tree is available for
    teleport operations without IK.

    Args:
        stage: USD stage containing the robot.
        robot_prim: Robot root prim (must carry IsaacRobotAPI).
        start_prim: Chain start link or site prim. Optional.
        end_prim: Chain end link or site prim. Optional.
        debug: Enable verbose chain-building and FK debug output.

    """

    def __init__(
        self,
        stage: Any,
        robot_prim: Any,
        start_prim: Any = None,
        end_prim: Any = None,
        *,
        debug: bool = False,
    ) -> None:
        self._stage = stage
        self._robot_prim = robot_prim
        self._start_prim = start_prim
        self._end_prim = end_prim
        self._debug = debug
        self._tree_root = GenerateRobotLinkTree(stage, robot_prim)
        self._joints: list[Joint] = self._build_joint_chain() if start_prim is not None and end_prim is not None else []

    # -- Construction helper ------------------------------------------------

    def _build_joint_chain(self) -> list[Joint]:
        """Build an ordered list of :class:`Joint` objects for the IK chain.

        Uses GetJointPose to obtain every joint's frame in the robot-base
        coordinate system. Traverses the kinematic tree from
        :attr:`start_prim` to :attr:`end_prim`, collecting every movable
        joint (RevoluteJoint / PrismaticJoint) in forward-kinematics order.
        Site offsets are handled via the world-space poses of the
        start/end prims.

        Returns:
            Movable joints in FK order. Each Joint carries its prim_path so
            the IK solution can be mapped back to USD.

        """
        import pxr

        if self._tree_root is None:
            return []

        # ---- 1. Compute zero-config body world poses ----
        zero_world = _compute_zero_config_poses(self._tree_root)
        robot_world = pxr.Gf.Matrix4d(_get_world_transform_matrix(self._robot_prim))
        robot_inv = robot_world.GetInverse()

        # ---- 2. Resolve each prim to its tree node (parent Link for Sites) ----
        def _resolve_node(prim: Any) -> Any:
            is_site = prim.HasAPI(Classes.SITE_API.value) or prim.HasAPI(Classes.REFERENCE_POINT_API.value)
            if is_site:
                return _find_tree_node(self._tree_root, str(prim.GetParent().GetPath()))
            return _find_tree_node(self._tree_root, str(prim.GetPath()))

        start_node = _resolve_node(self._start_prim)
        end_node = _resolve_node(self._end_prim)

        if start_node is None or end_node is None:
            return []
        if start_node is end_node:
            return []

        # ---- 3. Collect raw (joint_prim, is_forward) along the tree path ----
        raw = _collect_chain_joints(start_node, end_node)

        # ---- 4. Start / end zero-config poses in robot-base frame ----
        def _zero_pose_robot(prim: Any) -> Transform:
            prim_path = str(prim.GetPath())
            is_site = prim.HasAPI(Classes.SITE_API.value) or prim.HasAPI(Classes.REFERENCE_POINT_API.value)
            if is_site:
                parent = prim.GetParent()
                parent_path = str(parent.GetPath())
                if parent_path in zero_world:
                    parent_zero = zero_world[parent_path]
                    parent_cur = pxr.Gf.Matrix4d(_get_world_transform_matrix(parent))
                    site_cur = pxr.Gf.Matrix4d(_get_world_transform_matrix(prim))
                    site_local = site_cur * parent_cur.GetInverse()
                    return _mat4_to_transform(site_local * parent_zero * robot_inv)
            elif prim_path in zero_world:
                return _mat4_to_transform(zero_world[prim_path] * robot_inv)
            return _prim_pose_in_robot_frame(self._robot_prim, prim)

        debug = self._debug
        start_pose = _zero_pose_robot(self._start_prim)
        end_pose = _zero_pose_robot(self._end_prim)

        if debug:
            print("\n===== BUILD IK JOINT CHAIN =====")
            print(f"  start_prim: {self._start_prim.GetPath()}")
            print(f"  end_prim:   {self._end_prim.GetPath()}")
            print(f"  start_pose  t = [{start_pose.t[0]:.8f}, {start_pose.t[1]:.8f}, {start_pose.t[2]:.8f}]")
            print(
                f"              q = [{start_pose.q[0]:.8f}, {start_pose.q[1]:.8f}, {start_pose.q[2]:.8f}, {start_pose.q[3]:.8f}]"
            )
            print(f"  end_pose    t = [{end_pose.t[0]:.8f}, {end_pose.t[1]:.8f}, {end_pose.t[2]:.8f}]")
            print(
                f"              q = [{end_pose.q[0]:.8f}, {end_pose.q[1]:.8f}, {end_pose.q[2]:.8f}, {end_pose.q[3]:.8f}]"
            )
            print(f"  Chain joints ({len(raw)}):")

        # ---- 5. Convert USD joints → Joint objects via zero-config poses ----
        prev_tf = start_pose
        joints: list[Joint] = []

        for joint_prim, is_forward in raw:
            is_rev = joint_prim.IsA(pxr.UsdPhysics.RevoluteJoint)
            is_pri = joint_prim.IsA(pxr.UsdPhysics.PrismaticJoint)

            if not is_rev and not is_pri:
                if debug:
                    print(f"    SKIP (fixed): {joint_prim.GetPath()}")
                continue

            joint_usd = pxr.UsdPhysics.Joint(joint_prim)
            body0_targets = joint_usd.GetBody0Rel().GetTargets()
            body0_path = str(body0_targets[0]) if body0_targets else None

            if body0_path and body0_path in zero_world:
                local0_mat = _get_joint_local_transform(joint_usd, 0)
                if local0_mat is not None:
                    joint_zero_world = pxr.Gf.Matrix4d(local0_mat) * zero_world[body0_path]
                    T_j = _mat4_to_transform(joint_zero_world * robot_inv)
                else:
                    if debug:
                        print(f"    SKIP (no local0): {joint_prim.GetPath()}")
                    continue
            else:
                pose_mat = GetJointPose(self._robot_prim, joint_prim)
                if pose_mat is None:
                    if debug:
                        print(f"    SKIP (no pose): {joint_prim.GetPath()}")
                    continue
                T_j = _mat4_to_transform(pose_mat)

            home = prev_tf.inv() @ T_j

            if is_rev:
                ax_str = str(pxr.UsdPhysics.RevoluteJoint(joint_prim).GetAxisAttr().Get())
            else:
                ax_str = str(pxr.UsdPhysics.PrismaticJoint(joint_prim).GetAxisAttr().Get())

            local_axis: Vec3 = _AXIS_VEC.get(ax_str, _AXIS_VEC["X"]).copy()
            if not is_forward:
                local_axis = -local_axis

            w = local_axis if is_rev else np.zeros(3)
            v = np.zeros(3) if is_rev else local_axis

            if debug:
                jtype = "REV" if is_rev else "PRI"
                fwd = "fwd" if is_forward else "bwd"
                print(f"    [{len(joints)}] {joint_prim.GetPath()} ({jtype}, {fwd}, axis={ax_str})")
                print(f"        ZeroConfig    t = [{T_j.t[0]:.8f}, {T_j.t[1]:.8f}, {T_j.t[2]:.8f}]")
                print(f"                      q = [{T_j.q[0]:.8f}, {T_j.q[1]:.8f}, {T_j.q[2]:.8f}, {T_j.q[3]:.8f}]")
                print(f"        prev_tf       t = [{prev_tf.t[0]:.8f}, {prev_tf.t[1]:.8f}, {prev_tf.t[2]:.8f}]")
                print(
                    f"                      q = [{prev_tf.q[0]:.8f}, {prev_tf.q[1]:.8f}, {prev_tf.q[2]:.8f}, {prev_tf.q[3]:.8f}]"
                )
                print(f"        home          t = [{home.t[0]:.8f}, {home.t[1]:.8f}, {home.t[2]:.8f}]")
                print(f"                      q = [{home.q[0]:.8f}, {home.q[1]:.8f}, {home.q[2]:.8f}, {home.q[3]:.8f}]")
                print(f"        local_axis      = [{local_axis[0]:.8f}, {local_axis[1]:.8f}, {local_axis[2]:.8f}]")

            lo = -np.inf
            hi = np.inf
            if is_rev:
                rev_joint = pxr.UsdPhysics.RevoluteJoint(joint_prim)
                lo_attr = rev_joint.GetLowerLimitAttr()
                hi_attr = rev_joint.GetUpperLimitAttr()
                if lo_attr and lo_attr.Get() is not None:
                    lo = float(np.radians(lo_attr.Get()))
                if hi_attr and hi_attr.Get() is not None:
                    hi = float(np.radians(hi_attr.Get()))
            elif is_pri:
                pri_joint = pxr.UsdPhysics.PrismaticJoint(joint_prim)
                lo_attr = pri_joint.GetLowerLimitAttr()
                hi_attr = pri_joint.GetUpperLimitAttr()
                if lo_attr and lo_attr.Get() is not None:
                    lo = float(lo_attr.Get())
                if hi_attr and hi_attr.Get() is not None:
                    hi = float(hi_attr.Get())
            if lo > hi:
                lo, hi = -np.inf, np.inf

            joints.append(
                Joint(
                    w=w,
                    v=v,
                    home=home,
                    prim_path=str(joint_prim.GetPath()),
                    lower=lo,
                    upper=hi,
                    forward=is_forward,
                    is_revolute=is_rev,
                )
            )
            prev_tf = T_j

        # ---- 6. Store trailing offset on last joint (applied after rotation) ----
        if joints:
            trailing = prev_tf.inv() @ end_pose
            if debug:
                print("  Trailing offset (last joint → end):")
                print(f"        trailing      t = [{trailing.t[0]:.8f}, {trailing.t[1]:.8f}, {trailing.t[2]:.8f}]")
                print(
                    f"                      q = [{trailing.q[0]:.8f}, {trailing.q[1]:.8f}, {trailing.q[2]:.8f}, {trailing.q[3]:.8f}]"
                )
            joints[-1].tip = trailing

        if debug:
            print("\n  FK at q=0 (per-link accumulated poses in robot-base frame):")
            T_check = Transform()
            for i, ji in enumerate(joints):
                T_check = T_check @ ji.exp(0.0)
                print(f"    After joint {i} ({ji.prim_path}):")
                print(f"        FK pose  t = [{T_check.t[0]:.8f}, {T_check.t[1]:.8f}, {T_check.t[2]:.8f}]")
                print(
                    f"                 q = [{T_check.q[0]:.8f}, {T_check.q[1]:.8f}, {T_check.q[2]:.8f}, {T_check.q[3]:.8f}]"
                )
            print("===== END BUILD IK JOINT CHAIN =====\n")

        return joints

    # -- properties ---------------------------------------------------------

    @property
    def stage(self) -> Any:
        """USD stage."""
        return self._stage

    @property
    def robot_prim(self) -> Any:
        """Robot root prim."""
        return self._robot_prim

    @property
    def start_prim(self) -> Any:
        """Chain start prim."""
        return self._start_prim

    @property
    def end_prim(self) -> Any:
        """Chain end prim."""
        return self._end_prim

    @property
    def joints(self) -> list[Joint]:
        """Ordered joint chain (copy)."""
        return list(self._joints)

    @property
    def tree_root(self) -> Any:
        """Cached kinematic tree root (from GenerateRobotLinkTree)."""
        return self._tree_root

    # -- Tier 1: FK computation ---------------------------------------------

    def compute_fk(self, q: VecN, *, debug: bool = False) -> tuple[Transform, list[Transform]]:
        """Compute end-effector FK for joint configuration *q*.

        Args:
            q: Joint values in chain order (radians / meters).
            debug: Print per-joint FK trace when True.

        Returns:
            ``(end_effector_transform, per_joint_transforms)`` in chain-local frame.

        """
        T = Transform()
        chain: list[Transform] = []
        dbg = debug or self._debug
        if dbg:
            print("\n===== FORWARD KINEMATICS =====")
        for i, (ji, qi) in enumerate(zip(self._joints, q)):
            exp_i = ji.exp(float(qi))
            T = T @ exp_i
            chain.append(T)
            if dbg:
                print(f"  Joint {i} ({ji.prim_path}):")
                print(f"    q = {float(qi):.8e}")
                print(f"    exp(q)   t = [{exp_i.t[0]:.8f}, {exp_i.t[1]:.8f}, {exp_i.t[2]:.8f}]")
                print(f"             q = [{exp_i.q[0]:.8f}, {exp_i.q[1]:.8f}, {exp_i.q[2]:.8f}, {exp_i.q[3]:.8f}]")
                print(f"    accum FK t = [{T.t[0]:.8f}, {T.t[1]:.8f}, {T.t[2]:.8f}]")
                print(f"             q = [{T.q[0]:.8f}, {T.q[1]:.8f}, {T.q[2]:.8f}, {T.q[3]:.8f}]")
        if dbg:
            print(f"  FINAL FK   t = [{T.t[0]:.8f}, {T.t[1]:.8f}, {T.t[2]:.8f}]")
            print(f"             q = [{T.q[0]:.8f}, {T.q[1]:.8f}, {T.q[2]:.8f}, {T.q[3]:.8f}]")
            print("===== END FORWARD KINEMATICS =====\n")
        return T, chain

    def compute_fk_and_jacobian(self, q: VecN) -> tuple[Transform, Mat]:
        """Compute end-effector FK and spatial Jacobian for joint configuration *q*.

        Fused single-pass implementation; joint exponentials evaluated once.

        Args:
            q: Joint values in chain order (radians / meters).

        Returns:
            ``(end_effector_transform, 6×N_jacobian)`` in chain-local frame.

        """
        joints = self._joints
        n = len(joints)
        J = np.zeros((6, n), dtype=float)

        home_q = [j.home.q for j in joints]
        home_ht = [quat_to_matrix(j.home.q).T @ j.home.t for j in joints]

        T_q = np.array([1.0, 0.0, 0.0, 0.0])
        T_t = np.zeros(3)

        for i in range(n):
            ji = joints[i]
            qi_f = float(q[i])

            Th_q = quat_mul(T_q, home_q[i])
            R = quat_to_matrix(Th_q)
            Th_t = T_t + R @ home_ht[i]

            if ji.is_revolute:
                Rw = R @ ji.w
                J[:3, i] = Rw
                J[3:, i] = np.cross(Th_t, Rw)
                half = qi_f * 0.5
                s = np.sin(half)
                dq = np.array([np.cos(half), ji.w[0] * s, ji.w[1] * s, ji.w[2] * s])
                T_q = quat_mul(Th_q, dq)
                T_t = Th_t
            else:  # prismatic
                Rv = R @ ji.v
                J[3:, i] = Rv
                T_q = Th_q
                T_t = Th_t + Rv * qi_f

            if ji.tip is not None:
                T_t = T_t + quat_rotate(T_q, ji.tip.t)
                T_q = quat_mul(T_q, ji.tip.q)

        return Transform(T_t, T_q), J

    # -- Tier 2: USD I/O ----------------------------------------------------

    def _read_all_joint_states(self) -> dict[str, float]:
        """Read current USD state for every robot joint (radians / meters).

        Uses the cached kinematic tree — no tree rebuild.

        Returns:
            Mapping of joint prim-path to value.

        """
        all_joints: dict[str, float] = {}
        if self._tree_root is None:
            return all_joints
        stack = [self._tree_root]
        while stack:
            node = stack.pop()
            jprim = node._joint_to_parent
            if jprim is not None:
                jpath = str(jprim.GetPath())
                if _joint_is_revolute(jprim):
                    attr = jprim.GetAttribute("state:angular:physics:position")
                    if attr and attr.Get() is not None:
                        all_joints[jpath] = float(np.radians(float(attr.Get())))
                elif _joint_is_prismatic(jprim):
                    attr = jprim.GetAttribute("state:linear:physics:position")
                    if attr and attr.Get() is not None:
                        all_joints[jpath] = float(attr.Get())
            stack.extend(node.children)
        return all_joints

    def read_joint_states(self) -> dict[str, float]:
        """Read current USD state for the chain joints only (radians / meters).

        Uses the cached kinematic tree — no tree rebuild.

        Returns:
            Mapping of joint prim-path to value (radians or meters).

        """
        all_states = self._read_all_joint_states()
        chain_paths = {j.prim_path for j in self._joints}
        return {p: v for p, v in all_states.items() if p in chain_paths}

    def set_joint_attributes(self, joint_dict: dict[str, float]) -> None:
        """Write drive-target and physics-state attributes for the given joints.

        Args:
            joint_dict: Joint prim-path to value (radians or meters).

        """
        _set_joint_attributes_stage(self._stage, joint_dict)

    def teleport(self, joint_dict: dict[str, float]) -> None:
        """Apply *joint_dict* by propagating FK body transforms.

        Uses the cached kinematic tree — no tree rebuild.
        Use when simulation is stopped.

        Args:
            joint_dict: Joint prim-path to value (radians or meters).

        """
        if not joint_dict or self._tree_root is None:
            return

        _set_joint_attributes_stage(self._stage, joint_dict)

        old_transforms: dict[str, Any] = {}
        _read_subtree_transforms(self._tree_root, old_transforms)

        for child in self._tree_root.children:
            _fk_propagate_node(self._stage, child, joint_dict, old_transforms, affected=False)

    def teleport_anchored(self, joint_dict: dict[str, float], *, anchor_prim: Any = None) -> None:
        """Apply *joint_dict* keeping a fixed prim's world position unchanged.

        Merges *joint_dict* with the current USD state for all robot joints,
        propagates FK, then rigidly corrects the robot so the anchor prim
        returns to its original world position. Necessary when the chain
        contains backward (child-to-parent) joints.

        Uses the cached kinematic tree — no tree rebuild.
        Use when simulation is stopped.

        Args:
            joint_dict: Joint prim-path to value (radians or meters).
            anchor_prim: Prim to hold fixed. Defaults to :attr:`start_prim`.

        """
        import pxr

        anchor = anchor_prim if anchor_prim is not None else self._start_prim
        if not joint_dict or self._tree_root is None or anchor is None:
            return

        _set_joint_attributes_stage(self._stage, joint_dict)

        full_joints = self._read_all_joint_states()
        full_joints.update(joint_dict)

        anchor_before = pxr.Gf.Matrix4d(_get_world_transform_matrix(anchor))

        old_transforms: dict[str, Any] = {}
        _read_subtree_transforms(self._tree_root, old_transforms)

        for child in self._tree_root.children:
            _fk_propagate_node(self._stage, child, full_joints, old_transforms, affected=False)

        anchor_after = pxr.Gf.Matrix4d(_get_world_transform_matrix(anchor))
        C = anchor_after.GetInverse() * anchor_before

        root_world = pxr.Gf.Matrix4d(_get_world_transform_matrix(self._tree_root.prim))
        _apply_world_transform(self._tree_root.prim, root_world * C)

        old_corrected: dict[str, Any] = {}
        _read_subtree_transforms(self._tree_root, old_corrected)

        for child in self._tree_root.children:
            _fk_propagate_node(self._stage, child, full_joints, old_corrected, affected=True)
