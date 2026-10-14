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


def _strip_scale(m: Gf.Matrix4d) -> Gf.Matrix4d:
    """Strip scale/shear from a frame, keeping only rotation + translation.

    URDF link frames carry pose only (joint origins are xyz + rpy). Some pose
    sources (e.g. robot_schema GetJointPose) can return a child frame that
    carries an ancestor/root scale. Leaving that scale in the frame cancels the
    root scale that is re-applied, so child links export unscaled while the (identity)
    root link does not. Stripping the scale from every frame keeps scale handling
    consistent across all links.

    Args:
        m: Transform matrix to process.

    Returns:
        Transform matrix containing only rotation and translation.
    """
    t = Gf.Transform(m)
    stripped = Gf.Matrix4d(1.0)
    stripped.SetTranslateOnly(t.GetTranslation())
    stripped.SetRotateOnly(t.GetRotation())
    return stripped


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

    if desc.root_link:
        xform_cache = UsdGeom.XformCache()
        robot_world = Gf.Matrix4d(xform_cache.GetLocalToWorldTransform(robot_prim))
        root_link_world = Gf.Matrix4d(xform_cache.GetLocalToWorldTransform(desc.root_link))
        urdf_frames[root_link_path] = root_link_world * robot_world.GetInverse()

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
                urdf_frames[child_path] = _strip_scale(child_world * robot_world.GetInverse())
            continue

        flipped = _detect_axis_flip(joint_prim)
        joint_path = str(joint_prim.GetPath())
        axis_flips[joint_path] = flipped

        if flipped:
            axis_token = _get_axis_token(joint_prim)
            correction = _make_axis_flip_correction(axis_token)
            urdf_frames[child_path] = _strip_scale(correction * joint_pose)
        else:
            urdf_frames[child_path] = _strip_scale(joint_pose)

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
        urdf_frames[str(link_prim.GetPath())] = _strip_scale(link_world * robot_inv)
    return urdf_frames


def root_world_scale_matrix(robot_prim: Usd.Prim) -> Gf.Matrix4d:
    """Diagonal scale matrix of the robot root prim's world transform.

    URDF frames and geometry are expressed in robot-local coordinates, which
    divides out the root prim's full local-to-world transform (translation,
    rotation AND scale). Translation and rotation should be removed to keep the
    URDF placement-independent, but a scale on the root prim or an ancestor
    Xform must be preserved in the exported URDF. Callers re-apply this matrix
    to the frame-relative transforms so the scale reaches the output.

    Args:
        robot_prim: Robot root prim.

    Returns:
        Diagonal matrix containing the robot root's composed scale.
    """
    xfc = UsdGeom.XformCache()
    world = Gf.Matrix4d(xfc.GetLocalToWorldTransform(robot_prim))
    scale = Gf.Transform(world).GetScale()
    scale_mat = Gf.Matrix4d(1.0)
    scale_mat.SetScale(scale)
    return scale_mat


def compute_joint_origin_from_frames(
    urdf_frames: dict[str, Gf.Matrix4d],
    parent_path: str,
    child_path: str,
    scale_mat: Gf.Matrix4d | None = None,
) -> tuple[tuple[float, float, float], tuple[float, float, float]]:
    """Compute URDF joint origin from pre-built URDF frames.

    origin = child_urdf_frame * parent_urdf_frame^-1

    Args:
        urdf_frames: Mapping from link prim paths to URDF frames.
        parent_path: Parent link prim path.
        child_path: Child link prim path.
        scale_mat: Optional root-scale matrix (see :func:`root_world_scale_matrix`)
            post-applied so an ancestor/root scale reaches the joint offsets.

    Returns:
        Joint origin translation and rotation.
    """
    parent_frame = urdf_frames.get(parent_path)
    child_frame = urdf_frames.get(child_path)

    if parent_frame is None or child_frame is None:
        return (0.0, 0.0, 0.0), (0.0, 0.0, 0.0)

    relative = child_frame * parent_frame.GetInverse()
    if scale_mat is not None:
        relative = relative * scale_mat
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
    # Re-apply the root/ancestor scale that robot_world.GetInverse() removed, so
    # a scaled robot exports scaled meshes (baked vertices and <mesh scale=...>).
    scale_mat = Gf.Matrix4d(1.0)
    scale_mat.SetScale(Gf.Transform(robot_world).GetScale())
    return geom_in_robot * link_urdf.GetInverse() * scale_mat


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
