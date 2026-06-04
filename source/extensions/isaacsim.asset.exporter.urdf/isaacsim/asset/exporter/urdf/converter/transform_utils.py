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
"""Transform math utilities for USD-to-URDF conversion."""

from __future__ import annotations

import math

from pxr import Gf, Usd, UsdPhysics


def quaternion_to_rpy(quat: Gf.Quatf | Gf.Quatd) -> tuple[float, float, float]:
    """Convert a quaternion to URDF roll-pitch-yaw Euler angles (radians).

    Gf.Rotation.Decompose(Z, Y, X) returns (yaw_deg, pitch_deg, roll_deg).
    Reverse to get (roll, pitch, yaw) and convert to radians.
    This matches the convention used throughout Isaac Sim (see scene_utils.py,
    transform_utils.py in replicator extensions).

    Args:
        quat: Quaternion (w, x, y, z) as Gf.Quatf or Gf.Quatd.

    Returns:
        (roll, pitch, yaw) in radians.
    """
    rotation = Gf.Rotation(Gf.Quatd(quat))
    zyx = rotation.Decompose(Gf.Vec3d(0, 0, 1), Gf.Vec3d(0, 1, 0), Gf.Vec3d(1, 0, 0))
    roll = math.radians(zyx[2])
    pitch = math.radians(zyx[1])
    yaw = math.radians(zyx[0])
    return (roll, pitch, yaw)


def matrix4_to_origin(mat: Gf.Matrix4d) -> tuple[tuple[float, float, float], tuple[float, float, float]]:
    """Extract URDF origin (xyz, rpy) from a 4x4 transform matrix.

    Uses ``Gf.Transform`` to decompose the matrix so that any scale
    component is factored out before reading the rotation. ``ExtractRotation``
    on a scaled matrix returns a corrupted rotation (the basis vectors are
    not unit-length), which would yield wrong RPY values in URDF output.

    Args:
        mat: 4x4 transform matrix; may contain non-unit scale.

    Returns:
        (xyz, rpy) where xyz is (x, y, z) and rpy is (roll, pitch, yaw) in radians.
    """
    xyz, rpy, _ = matrix4_to_origin_and_scale(mat)
    return xyz, rpy


def matrix4_to_origin_and_scale(
    mat: Gf.Matrix4d,
) -> tuple[tuple[float, float, float], tuple[float, float, float], tuple[float, float, float]]:
    """Decompose a 4x4 transform matrix into URDF origin and scale.

    Args:
        mat: 4x4 transform matrix.

    Returns:
        Tuple of ``(xyz, rpy, scale)``. ``xyz`` is the translation,
        ``rpy`` is roll-pitch-yaw in radians (with scale removed), and
        ``scale`` is the per-axis scale factor.
    """
    transform = Gf.Transform(mat)
    translate = transform.GetTranslation()
    xyz = (float(translate[0]), float(translate[1]), float(translate[2]))

    quat = transform.GetRotation().GetQuat()
    rpy = quaternion_to_rpy(Gf.Quatd(quat))

    scale = transform.GetScale()
    scale_tuple = (float(scale[0]), float(scale[1]), float(scale[2]))

    return xyz, rpy, scale_tuple


def is_unit_scale(scale: tuple[float, float, float], tol: float = 1e-6) -> bool:
    """Return True when *scale* is effectively (1, 1, 1) within *tol*."""
    return all(abs(s - 1.0) < tol for s in scale)


def compute_joint_origin(joint: UsdPhysics.Joint) -> tuple[tuple[float, float, float], tuple[float, float, float]]:
    """Compute the URDF joint origin from USD joint local transforms.

    URDF origin = child link frame in parent link frame.
    parent_to_child = local0 * inverse(local1)

    Args:
        joint: USD physics joint prim.

    Returns:
        (xyz, rpy) of the joint origin, or ((0,0,0), (0,0,0)) if transforms are unavailable.
    """
    local0 = _get_local_transform(joint, 0)
    local1 = _get_local_transform(joint, 1)

    if local0 is None:
        return (0.0, 0.0, 0.0), (0.0, 0.0, 0.0)

    if local1 is None:
        parent_to_child = Gf.Matrix4d(local0)
    else:
        parent_to_child = Gf.Matrix4d(local1).GetInverse() * Gf.Matrix4d(local0)

    return matrix4_to_origin(parent_to_child)


def _get_local_transform(joint: UsdPhysics.Joint, body_index: int) -> Gf.Matrix4f | None:
    """Read localPos/localRot from a joint for the given body index.

    Args:
        joint: USD physics joint.
        body_index: 0 for parent body, 1 for child body.

    Returns:
        Matrix4f local transform, or None if attributes are missing.
    """
    pos_attr = joint.GetLocalPos0Attr() if body_index == 0 else joint.GetLocalPos1Attr()
    rot_attr = joint.GetLocalRot0Attr() if body_index == 0 else joint.GetLocalRot1Attr()
    if not pos_attr or not rot_attr:
        return None
    pos = pos_attr.Get()
    rot = rot_attr.Get()
    if pos is None or rot is None:
        return None
    mat = Gf.Matrix4f()
    mat.SetTranslate(pos)
    mat.SetRotateOnly(rot)
    return mat


def get_joint_axis_vector(
    joint_prim: Usd.Prim, axis_token: str, local_rot0: Gf.Quatf | None
) -> tuple[float, float, float]:
    """Recover the URDF axis vector from USD axis token and joint local rotation.

    Args:
        joint_prim: The USD joint prim.
        axis_token: The physics:axis value ("X", "Y", or "Z").
        local_rot0: The localRot0 quaternion, if any.

    Returns:
        Normalized (x, y, z) axis vector.
    """
    axis_map = {"X": Gf.Vec3d(1, 0, 0), "Y": Gf.Vec3d(0, 1, 0), "Z": Gf.Vec3d(0, 0, 1)}
    base_axis = axis_map.get(axis_token.upper(), Gf.Vec3d(1, 0, 0))

    if local_rot0 is not None:
        rot = Gf.Rotation(Gf.Quatd(local_rot0))
        rotated = rot.TransformDir(base_axis)
    else:
        rotated = base_axis

    length = rotated.GetLength()
    if length > 1e-10:
        rotated = rotated / length

    result = [float(rotated[i]) for i in range(3)]
    for i in range(3):
        if abs(result[i]) < 1e-6:
            result[i] = 0.0
        elif abs(result[i] - 1.0) < 1e-6:
            result[i] = 1.0
        elif abs(result[i] + 1.0) < 1e-6:
            result[i] = -1.0

    return tuple(result)


def linear_to_srgb(c: float) -> float:
    """Convert a single linear-space color component to sRGB.

    Args:
        c: Linear color value in [0, 1].

    Returns:
        sRGB color value in [0, 1].
    """
    if c <= 0.0031308:
        return 12.92 * c
    return 1.055 * (c ** (1.0 / 2.4)) - 0.055


def get_prim_name(prim: Usd.Prim) -> str:
    """Recover the original name from a USD prim using priority order.

    Priority:
    1. displayName metadata
    2. isaac:nameOverride (from IsaacLinkAPI / IsaacJointAPI)
    3. Prim name

    Args:
        prim: USD prim.

    Returns:
        The best available name string.
    """
    display_name = prim.GetMetadata("displayName")
    if display_name:
        return display_name

    for attr_name in ("isaac:nameOverride", "isaac:NameOverride"):
        attr = prim.GetAttribute(attr_name)
        if attr and attr.IsValid():
            val = attr.Get()
            if val:
                return val

    return prim.GetName()


def is_origin_identity(xyz: tuple[float, float, float], rpy: tuple[float, float, float], tol: float = 1e-8) -> bool:
    """Check if an origin is effectively identity (zero translation and rotation)."""
    return all(abs(v) < tol for v in xyz) and all(abs(v) < tol for v in rpy)
