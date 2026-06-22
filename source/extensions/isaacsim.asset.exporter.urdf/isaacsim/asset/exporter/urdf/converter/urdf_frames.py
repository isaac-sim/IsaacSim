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
"""Build URDF link frames from the joint kinematic chain.

In URDF, each link's coordinate frame is defined by the chain of joint
origins from the root.  The root link frame is identity (in robot coords).
Each subsequent link frame = parent_urdf_frame * joint_origin.

In USD, link prims may have arbitrary world transforms (flat-body layout).
The joint's world pose (from GetJointPose) is the ground truth for where
the joint sits in robot space.  The URDF child frame = joint world pose
(since URDF puts the joint at the child link origin).

When localRot1 flips the joint axis (180 deg rotation about an orthogonal
axis), the URDF axis is negated and the child frame is corrected to remove
the flip.
"""

from __future__ import annotations

import logging

from pxr import Gf, Sdf, Usd, UsdGeom, UsdPhysics

from .robot_finder import RobotDescription
from .transform_utils import matrix4_to_origin

_logger = logging.getLogger(__name__)

_AXIS_VECS = {"X": Gf.Vec3d(1, 0, 0), "Y": Gf.Vec3d(0, 1, 0), "Z": Gf.Vec3d(0, 0, 1)}


def _get_joint_body_path(joint_prim: Usd.Prim, body_index: int) -> str | None:
    """Get body relationship target path as string.

    Args:
        joint_prim: USD joint prim to read.
        body_index: Body relationship index to read.

    Returns:
        Body target path string, or None if absent.
    """
    joint = UsdPhysics.Joint(joint_prim)
    if not joint:
        return None
    rel = joint.GetBody0Rel() if body_index == 0 else joint.GetBody1Rel()
    if rel:
        targets = rel.GetTargets()
        if targets:
            return str(targets[0])
    return None


def _get_axis_token(joint_prim: Usd.Prim) -> str:
    """Read physics:axis token from a joint prim.

    Args:
        joint_prim: USD joint prim to read.

    Returns:
        Uppercase axis token.
    """
    attr = joint_prim.GetAttribute("physics:axis")
    if attr and attr.IsValid():
        val = attr.Get()
        if val:
            return str(val).upper()
    return "X"


def _detect_axis_flip(joint_prim: Usd.Prim) -> bool:
    """Detect if the joint's localRot1 flips the axis direction.

    A 180-degree rotation about an axis orthogonal to the joint axis
    negates the joint axis direction. Check by transforming the axis
    vector through localRot1 and comparing the dot product.

    Args:
        joint_prim: USD joint prim to read.

    Returns:
        True if localRot1 flips the joint axis, False otherwise.
    """
    joint = UsdPhysics.Joint(joint_prim)
    if not joint:
        return False

    rot1_attr = joint.GetLocalRot1Attr()
    if not rot1_attr:
        return False
    rot1 = rot1_attr.Get()
    if rot1 is None:
        return False

    rot1_real = rot1.GetReal()
    rot1_imag = rot1.GetImaginary()
    is_identity = abs(rot1_real - 1.0) < 1e-6 and all(abs(v) < 1e-6 for v in rot1_imag)
    if is_identity:
        return False

    axis_token = _get_axis_token(joint_prim)
    axis_vec = _AXIS_VECS.get(axis_token, _AXIS_VECS["X"])

    rotation = Gf.Rotation(Gf.Quatd(rot1))
    transformed = rotation.TransformDir(axis_vec)

    dot = axis_vec[0] * transformed[0] + axis_vec[1] * transformed[1] + axis_vec[2] * transformed[2]
    return dot < 0


def _make_axis_flip_correction(axis_token: str) -> Gf.Matrix4d:
    """Create a 180-degree rotation matrix about an axis orthogonal to the joint axis.

    This removes the flip from the child frame when the axis is negated.

    Args:
        axis_token: Physics axis token.

    Returns:
        Axis flip correction matrix.
    """
    if axis_token == "Z":
        rot = Gf.Rotation(Gf.Vec3d(1, 0, 0), 180.0)
    elif axis_token == "Y":
        rot = Gf.Rotation(Gf.Vec3d(1, 0, 0), 180.0)
    else:
        rot = Gf.Rotation(Gf.Vec3d(0, 1, 0), 180.0)

    mat = Gf.Matrix4d(1.0)
    mat.SetRotateOnly(rot)
    return mat


def build_urdf_frames(desc: RobotDescription) -> tuple[dict[str, Gf.Matrix4d], dict[str, bool]]:
    """Build the URDF frame (in robot coordinates) for every link.

    Uses GetJointPose from robot_schema to get each joint's world pose
    in robot coordinates. Detects axis flips and adjusts child frames.

    Args:
        desc: Robot description to read.

    Returns:
        URDF frames and axis flip flags.
    """
    try:
        from usd.schema.isaac.robot_schema.utils import GetJointPose
    except ImportError:
        _logger.warning("robot_schema not available, falling back to world transforms")
        frames = _build_urdf_frames_fallback(desc)
        return frames, {}

    stage = desc.root_prim.GetStage()
    robot_prim = desc.root_prim
    root_link_path = str(desc.root_link.GetPath()) if desc.root_link else None

    urdf_frames: dict[str, Gf.Matrix4d] = {}
    axis_flips: dict[str, bool] = {}

    urdf_frames[root_link_path] = Gf.Matrix4d(1.0)

    for joint_prim in desc.ordered_joints:
        child_path = _get_joint_body_path(joint_prim, 1)
        if not child_path or child_path in urdf_frames:
            continue

        joint_pose = GetJointPose(robot_prim, joint_prim)
        if joint_pose is None:
            child_prim = stage.GetPrimAtPath(Sdf.Path(child_path))
            if child_prim:
                xfc = UsdGeom.XformCache()
                robot_world = Gf.Matrix4d(xfc.GetLocalToWorldTransform(robot_prim))
                child_world = Gf.Matrix4d(xfc.GetLocalToWorldTransform(child_prim))
                urdf_frames[child_path] = child_world * robot_world.GetInverse()
            continue

        flipped = _detect_axis_flip(joint_prim)
        joint_path = str(joint_prim.GetPath())
        axis_flips[joint_path] = flipped

        if flipped:
            axis_token = _get_axis_token(joint_prim)
            correction = _make_axis_flip_correction(axis_token)
            urdf_frames[child_path] = correction * joint_pose
        else:
            urdf_frames[child_path] = joint_pose

    return urdf_frames, axis_flips


def _build_urdf_frames_fallback(desc: RobotDescription) -> dict[str, Gf.Matrix4d]:
    """Fallback when robot_schema is not available: use world transforms.

    Args:
        desc: Robot description to read.

    Returns:
        URDF frames keyed by link prim path.
    """
    xfc = UsdGeom.XformCache()
    robot_world = Gf.Matrix4d(xfc.GetLocalToWorldTransform(desc.root_prim))
    robot_inv = robot_world.GetInverse()
    urdf_frames: dict[str, Gf.Matrix4d] = {}
    for link_prim in desc.ordered_links:
        link_world = Gf.Matrix4d(xfc.GetLocalToWorldTransform(link_prim))
        urdf_frames[str(link_prim.GetPath())] = link_world * robot_inv
    return urdf_frames


def compute_joint_origin_from_frames(
    urdf_frames: dict[str, Gf.Matrix4d],
    parent_path: str,
    child_path: str,
) -> tuple[tuple[float, float, float], tuple[float, float, float]]:
    """Compute URDF joint origin from pre-built URDF frames.

    origin = child_urdf_frame * parent_urdf_frame^-1

    Args:
        urdf_frames: Mapping from link prim paths to URDF frames.
        parent_path: Parent link prim path.
        child_path: Child link prim path.

    Returns:
        Joint origin translation and rotation.
    """
    parent_frame = urdf_frames.get(parent_path)
    child_frame = urdf_frames.get(child_path)

    if parent_frame is None or child_frame is None:
        return (0.0, 0.0, 0.0), (0.0, 0.0, 0.0)

    relative = child_frame * parent_frame.GetInverse()
    return matrix4_to_origin(relative)


def compute_geom_to_link_transform(
    urdf_frames: dict[str, Gf.Matrix4d],
    link_path: str,
    geom_prim: Usd.Prim,
    robot_prim: Usd.Prim,
) -> Gf.Matrix4d:
    """Compute the transform from a geometry's local space to its link's URDF frame.

    The geometry's world pose is expressed in robot coordinates, then
    rebased onto the link's URDF frame:

        geom_world * robot_world^-1 * link_urdf^-1

    The result includes any scale carried by ancestor xforms. Callers
    that need only ``(xyz, rpy)`` (with scale removed) should pass the
    matrix to :func:`matrix4_to_origin`; callers that need the scale
    component as well should use :func:`matrix4_to_origin_and_scale`.

    Returns the identity matrix when *link_path* has no URDF frame.

    Args:
        urdf_frames: Mapping from link prim paths to URDF frames.
        link_path: Link prim path.
        geom_prim: Value to use.
        robot_prim: Robot root prim.

    Returns:
        Geometry-to-link transform matrix.
    """
    link_urdf = urdf_frames.get(link_path)
    if link_urdf is None:
        return Gf.Matrix4d(1.0)

    xfc = UsdGeom.XformCache()
    robot_world = Gf.Matrix4d(xfc.GetLocalToWorldTransform(robot_prim))
    geom_world = Gf.Matrix4d(xfc.GetLocalToWorldTransform(geom_prim))

    geom_in_robot = geom_world * robot_world.GetInverse()
    return geom_in_robot * link_urdf.GetInverse()


def compute_geom_origin_from_frames(
    urdf_frames: dict[str, Gf.Matrix4d],
    link_path: str,
    geom_prim: Usd.Prim,
    robot_prim: Usd.Prim,
) -> tuple[tuple[float, float, float], tuple[float, float, float]]:
    """Compute a geometry's URDF origin (xyz, rpy) relative to its link's URDF frame.

    Scale on the geometry chain is discarded by ``matrix4_to_origin``;
    URDF callers handle scale separately via the ``<mesh scale=...>``
    attribute.

    Args:
        urdf_frames: Mapping from link prim paths to URDF frames.
        link_path: Link prim path.
        geom_prim: Value to use.
        robot_prim: Robot root prim.

    Returns:
        Geometry origin translation and rotation.
    """
    if link_path not in urdf_frames:
        return (0.0, 0.0, 0.0), (0.0, 0.0, 0.0)
    return matrix4_to_origin(compute_geom_to_link_transform(urdf_frames, link_path, geom_prim, robot_prim))


def compute_mesh_bake_transform(
    urdf_frames: dict[str, Gf.Matrix4d],
    link_path: str,
    geom_prim: Usd.Prim,
    robot_prim: Usd.Prim,
) -> Gf.Matrix4d:
    """Compute the transform to bake into OBJ vertices for non-instanced meshes.

    Maps mesh-local vertices into the URDF link frame so that the URDF
    geometry origin can stay identity. Equivalent to
    :func:`compute_geom_to_link_transform` and kept as a separate name
    purely for readability at the call site.

    Args:
        urdf_frames: Mapping from link prim paths to URDF frames.
        link_path: Link prim path.
        geom_prim: Value to use.
        robot_prim: Robot root prim.

    Returns:
        Mesh bake transform matrix.
    """
    return compute_geom_to_link_transform(urdf_frames, link_path, geom_prim, robot_prim)
