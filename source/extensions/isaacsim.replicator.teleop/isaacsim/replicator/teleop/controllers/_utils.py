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

"""Target and quaternion helpers shared across teleop controllers.

Kept NumPy-local: teleop runs these on batch-size-1 inputs (often per IK iteration)
"""

from __future__ import annotations

import math

import numpy as np

DEFAULT_ROTATION_OFFSET_DEG = 0
ROTATION_OFFSET_DEGREES = (-180, -90, 0, 90, 180)
ROTATION_OFFSET_LABELS = ("-180", "-90", "0", "+90", "+180")


def quat_mul_wxyz(a: np.ndarray, b: np.ndarray) -> np.ndarray:
    """Hamilton product of batched quaternions in ``wxyz`` convention."""
    w1, x1, y1, z1 = a[:, 0], a[:, 1], a[:, 2], a[:, 3]
    w2, x2, y2, z2 = b[:, 0], b[:, 1], b[:, 2], b[:, 3]
    ww = (z1 + x1) * (x2 + y2)
    yy = (w1 - y1) * (w2 + z2)
    zz = (w1 + y1) * (w2 - z2)
    xx = ww + yy + zz
    qq = 0.5 * (xx + (z1 - x1) * (x2 - y2))
    w = qq - ww + (z1 - y1) * (y2 - z2)
    x = qq - xx + (x1 + w1) * (x2 + w2)
    y = qq - yy + (w1 - x1) * (y2 + z2)
    z = qq - zz + (z1 + y1) * (w2 - x2)
    return np.stack([w, x, y, z], axis=-1)


def quat_conjugate_wxyz(q: np.ndarray) -> np.ndarray:
    """Quaternion conjugate for batched ``wxyz`` arrays."""
    return np.concatenate((q[:, :1], -q[:, 1:]), axis=-1)


def quat_error_wxyz(goal_wxyz: np.ndarray, current_wxyz: np.ndarray) -> np.ndarray:
    """Orientation error as a 3-vector for batched ``wxyz`` quaternions."""
    q_err = quat_mul_wxyz(goal_wxyz, quat_conjugate_wxyz(current_wxyz))
    sign = np.sign(q_err[:, [0]])
    sign[sign == 0] = 1.0
    return q_err[:, 1:] * sign


def xyzw_to_wxyz(q: tuple[float, float, float, float]) -> np.ndarray:
    """Convert an ``xyzw`` tuple to a batched ``wxyz`` array."""
    return np.array([[q[3], q[0], q[1], q[2]]], dtype=np.float64)


def quat_mul_xyzw(
    a: tuple[float, float, float, float], b: tuple[float, float, float, float]
) -> tuple[float, float, float, float]:
    """Hamilton product for scalar-last quaternions."""
    ax, ay, az, aw = float(a[0]), float(a[1]), float(a[2]), float(a[3])
    bx, by, bz, bw = float(b[0]), float(b[1]), float(b[2]), float(b[3])
    x = aw * bx + ax * bw + ay * bz - az * by
    y = aw * by - ax * bz + ay * bw + az * bx
    z = aw * bz + ax * by - ay * bx + az * bw
    w = aw * bw - ax * bx - ay * by - az * bz
    return (x, y, z, w)


def rotation_offset_quat_xyzw(
    x_deg: float = 0.0,
    y_deg: float = 0.0,
    z_deg: float = 0.0,
) -> tuple[float, float, float, float]:
    """Quaternion for local-frame XYZ rotation offsets in degrees.

    Offsets compose in local ``X -> Y -> Z`` order (``qx * qy * qz``) so each
    dropdown reads independently against the viewport gizmo.
    """

    def _axis_quat(axis_idx: int, deg: float) -> tuple[float, float, float, float]:
        half = 0.5 * math.radians(float(deg))
        s = math.sin(half)
        c = math.cos(half)
        q = [0.0, 0.0, 0.0, c]
        q[axis_idx] = s
        return (q[0], q[1], q[2], q[3])

    q = quat_mul_xyzw(quat_mul_xyzw(_axis_quat(0, x_deg), _axis_quat(1, y_deg)), _axis_quat(2, z_deg))
    n = math.sqrt(q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3])
    if n > 0.0:
        return (q[0] / n, q[1] / n, q[2] / n, q[3] / n)
    return q


def ema_blend(
    previous: np.ndarray | None,
    current: np.ndarray,
    filter_strength: float,
    *,
    normalize: bool = False,
) -> np.ndarray:
    """Blend a new sample into an EMA state."""
    alpha = 1.0 - np.clip(filter_strength, 0.0, 0.99)
    if previous is None:
        blended = np.array(current, dtype=np.float64, copy=True)
    else:
        blended = previous + alpha * (current - previous)

    if not normalize:
        return blended

    norm = np.linalg.norm(blended)
    if norm > 1e-8:
        return blended / norm
    return np.array(current, dtype=np.float64, copy=True)
