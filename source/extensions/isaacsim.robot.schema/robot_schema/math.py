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

"""Pure math: data structures, quaternion operations, and USD type conversions.

This module provides the foundational types used throughout the robot kinematic
chain stack.  It has **no** dependencies on the kinematic tree, USD stage, or
any simulation state — only ``numpy`` and lazy ``pxr`` imports for type
conversion helpers.

**Data structures**

* :class:`Transform` – rigid SE(3) transform (translation + quaternion).
* :class:`Joint` – joint screw axis, home pose, and limits.

**Type aliases**

* :data:`Vec3`, :data:`Quat`, :data:`VecN`, :data:`Mat`

**Quaternion utilities**

* :func:`quat_mul`, :func:`quat_conj`, :func:`quat_rotate`,
  :func:`axis_angle_to_quat`, :func:`quat_to_matrix`

**Linear algebra**

* :func:`skew`, :func:`adjoint`

**USD type conversions** (lazy ``pxr`` import)

* :func:`_gf_quat_to_array`, :func:`_gf_vec3_to_array`, :func:`_mat4_to_transform`
* :func:`_prim_pose_in_robot_frame`
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Any, TypeAlias

import numpy as np

Vec3: TypeAlias = np.ndarray
Quat: TypeAlias = np.ndarray
VecN: TypeAlias = np.ndarray
Mat: TypeAlias = np.ndarray


# ---------------------------------------------------------------------------
# Quaternion utilities
# ---------------------------------------------------------------------------


def quat_mul(q1: Quat, q2: Quat) -> Quat:
    """Multiply two quaternions (Hamilton product).

    Args:
        q1: First quaternion [w, x, y, z].
        q2: Second quaternion [w, x, y, z].

    Returns:
        Product quaternion.

    """
    w1, x1, y1, z1 = q1
    w2, x2, y2, z2 = q2
    return np.array(
        [
            w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2,
            w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
            w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
            w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2,
        ],
        dtype=float,
    )


def quat_conj(q: Quat) -> Quat:
    """Return the conjugate of a quaternion.

    Args:
        q: Quaternion [w, x, y, z].

    Returns:
        Conjugate [w, -x, -y, -z].

    """
    w, x, y, z = q
    return np.array([w, -x, -y, -z], dtype=float)


def quat_rotate(q: Quat, v: Vec3) -> Vec3:
    """Rotate vector v by quaternion q.

    Args:
        q: Unit quaternion.
        v: 3D vector.

    Returns:
        Rotated vector.

    """
    qv = np.concatenate([[0.0], v])
    return quat_mul(quat_mul(q, qv), quat_conj(q))[1:]


def axis_angle_to_quat(axis: Vec3, angle: float) -> Quat:
    """Build a unit quaternion from axis-angle representation.

    Args:
        axis: Rotation axis (will be normalised).
        angle: Rotation angle in radians.

    Returns:
        Unit quaternion [w, x, y, z].

    """
    axis = axis / np.linalg.norm(axis)
    s = np.sin(angle / 2.0)
    return np.array([np.cos(angle / 2.0), *(axis * s)], dtype=float)


def quat_to_matrix(q: Quat) -> Mat:
    """Convert a unit quaternion to a 3×3 rotation matrix.

    Args:
        q: Unit quaternion [w, x, y, z].

    Returns:
        3×3 rotation matrix.

    """
    w, x, y, z = q
    return np.array(
        [
            [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
            [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
            [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
        ],
        dtype=float,
    )


# ---------------------------------------------------------------------------
# Transform
# ---------------------------------------------------------------------------


@dataclass
class Transform:
    """Rigid SE(3) transform (translation and quaternion rotation).

    Args:
        t: Translation [x, y, z]. Defaults to zeros.
        q: Quaternion [w, x, y, z]. Defaults to identity.

    """

    t: Vec3
    q: Quat

    def __init__(self, t: Vec3 | None = None, q: Quat | None = None) -> None:
        self.t = np.zeros(3) if t is None else np.array(t, float)
        self.q = np.array([1, 0, 0, 0], float) if q is None else np.array(q, float)

    def __matmul__(self, other: Transform) -> Transform:
        """Compose this transform with another (self @ other).

        Args:
            other: Right-hand-side transform.

        Returns:
            Composed transform.

        """
        t = self.t + quat_rotate(self.q, other.t)
        q = quat_mul(self.q, other.q)
        return Transform(t, q)

    def inv(self) -> Transform:
        """Return the inverse transform.

        Returns:
            Inverse of this transform.

        """
        qi = quat_conj(self.q)
        ti = -quat_rotate(qi, self.t)
        return Transform(ti, qi)


# ---------------------------------------------------------------------------
# Joint
# ---------------------------------------------------------------------------


@dataclass
class Joint:
    """Single joint in a kinematic chain (screw axis and home pose)."""

    w: Vec3
    v: Vec3
    home: Transform
    prim_path: str = ""
    tip: Transform | None = None
    lower: float = -np.inf
    upper: float = np.inf
    forward: bool = True
    is_revolute: bool = True

    def exp(self, q: float) -> Transform:
        """Exponential map: joint value to relative transform.

        Args:
            q: Joint value (radians for revolute, meters for prismatic).

        Returns:
            Relative transform from parent to child.

        """
        if self.w @ self.w > 0:
            dq = axis_angle_to_quat(self.w, q)
            t = np.zeros(3)
        else:
            dq = np.array([1, 0, 0, 0], float)
            t = self.v * q
        result = self.home @ Transform(t, dq)
        if self.tip is not None:
            result = result @ self.tip
        return result


# ---------------------------------------------------------------------------
# Linear algebra
# ---------------------------------------------------------------------------


def skew(v: Vec3) -> Mat:
    """Return the 3×3 skew-symmetric matrix for cross-product with v.

    Args:
        v: 3D vector.

    Returns:
        3×3 skew matrix such that skew(v) @ w == cross(v, w).

    """
    x, y, z = v
    return np.array([[0, -z, y], [z, 0, -x], [-y, x, 0]], dtype=float)


def adjoint(T: Transform) -> Mat:
    """Return the 6×6 adjoint matrix for the transform T.

    Args:
        T: Rigid transform.

    Returns:
        6×6 adjoint matrix.

    """
    R = quat_to_matrix(T.q)
    p = T.t
    px = skew(p)
    Ad = np.zeros((6, 6), dtype=float)
    Ad[:3, :3] = R
    Ad[3:, 3:] = R
    Ad[3:, :3] = px @ R
    return Ad


# ---------------------------------------------------------------------------
# USD type conversions (lazy pxr imports)
# ---------------------------------------------------------------------------


def _gf_quat_to_array(gf_quat: Any) -> Quat:
    """Convert a pxr Gf quaternion (Quatf / Quatd / Quaternion) to [w, x, y, z].

    Args:
        gf_quat: pxr Gf quaternion.

    Returns:
        Numpy array [w, x, y, z].

    """
    imag = gf_quat.GetImaginary()
    return np.array([gf_quat.GetReal(), imag[0], imag[1], imag[2]], dtype=float)


def _gf_vec3_to_array(gf_vec: Any) -> Vec3:
    """Convert a pxr Gf 3-vector to a numpy array.

    Args:
        gf_vec: pxr Gf 3-vector (e.g. Gf.Vec3f).

    Returns:
        Numpy array of shape (3,).

    """
    return np.array([gf_vec[0], gf_vec[1], gf_vec[2]], dtype=float)


def _mat4_to_transform(mat: Any) -> Transform:
    """Convert a pxr Gf Matrix4f or Matrix4d to a Transform.

    Args:
        mat: pxr.Gf.Matrix4f or Matrix4d.

    Returns:
        Transform with translation and quaternion.

    """
    import pxr

    mat_d = pxr.Gf.Matrix4d(mat)
    t = _gf_vec3_to_array(mat_d.ExtractTranslation())
    q = _gf_quat_to_array(mat_d.ExtractRotation().GetQuat())
    return Transform(t, q)


def _prim_pose_in_robot_frame(robot_prim: Any, prim: Any) -> Transform:
    """Return prim's pose expressed in robot_prim's base coordinate frame.

    Args:
        robot_prim: Robot root USD prim.
        prim: USD prim whose pose is queried.

    Returns:
        Transform in robot base frame.

    """
    import pxr
    from pxr import Usd, UsdGeom

    def _world_xform(p: Any) -> pxr.Gf.Matrix4d:
        return UsdGeom.Xformable(p).ComputeLocalToWorldTransform(Usd.TimeCode.Default())

    robot_world = pxr.Gf.Matrix4d(_world_xform(robot_prim))
    prim_world = pxr.Gf.Matrix4d(_world_xform(prim))
    return _mat4_to_transform(prim_world * robot_world.GetInverse())
