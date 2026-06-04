# SPDX-FileCopyrightText: Copyright (c) 2021-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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

# python

"""Deprecated rotation utility functions."""

from __future__ import annotations

import math

import numpy as np

# omniverse
from pxr import Gf

# internal global constants
_POLE_LIMIT = 1.0 - 1e-6


def rot_matrix_to_quat(mat: np.ndarray) -> np.ndarray:
    """Convert rotation matrix to Quaternion.

    Args:
        mat: A 3x3 rotation matrix.

    Returns:
        quaternion (w, x, y, z).
    """
    if mat.shape == (3, 3):
        tmp = np.eye(4)
        tmp[0:3, 0:3] = mat
        mat = tmp

    q = np.empty((4,), dtype=np.float64)
    t = np.trace(mat)
    if t > mat[3, 3]:
        q[0] = t
        q[3] = mat[1, 0] - mat[0, 1]
        q[2] = mat[0, 2] - mat[2, 0]
        q[1] = mat[2, 1] - mat[1, 2]
    else:
        i, j, k = 0, 1, 2
        if mat[1, 1] > mat[0, 0]:
            i, j, k = 1, 2, 0
        if mat[2, 2] > mat[i, i]:
            i, j, k = 2, 0, 1
        t = mat[i, i] - (mat[j, j] + mat[k, k]) + mat[3, 3]
        q[i + 1] = t
        q[j + 1] = mat[i, j] + mat[j, i]
        q[k + 1] = mat[k, i] + mat[i, k]
        q[0] = mat[k, j] - mat[j, k]
    q *= 0.5 / np.sqrt(t * mat[3, 3])
    return q


def quat_to_rot_matrix(quat: np.ndarray) -> np.ndarray:
    """Convert input quaternion to rotation matrix.

    Args:
        quat: Input quaternion (w, x, y, z).

    Returns:
        A 3x3 rotation matrix.
    """
    q = np.array(quat, dtype=np.float64, copy=True)
    nq = np.dot(q, q)
    if nq < 1e-10:
        return np.identity(3)
    q *= np.sqrt(2.0 / nq)
    q = np.outer(q, q)
    return np.array(
        (
            (1.0 - q[2, 2] - q[3, 3], q[1, 2] - q[3, 0], q[1, 3] + q[2, 0]),
            (q[1, 2] + q[3, 0], 1.0 - q[1, 1] - q[3, 3], q[2, 3] - q[1, 0]),
            (q[1, 3] - q[2, 0], q[2, 3] + q[1, 0], 1.0 - q[1, 1] - q[2, 2]),
        ),
        dtype=np.float64,
    )


def matrix_to_euler_angles(mat: np.ndarray, degrees: bool = False, extrinsic: bool = True) -> np.ndarray:
    """Convert rotation matrix to Euler XYZ extrinsic or intrinsic angles.

    Args:
        mat: A 3x3 rotation matrix.
        degrees: Whether returned angles should be in degrees.
        extrinsic: True if the rotation matrix follows the extrinsic matrix
                   convention (equivalent to ZYX ordering but returned in the reverse) and False if it follows
                   the intrinsic matrix conventions (equivalent to XYZ ordering).
                   Defaults to True.

    Returns:
        Euler XYZ angles (intrinsic form) if extrinsic is False and Euler XYZ angles (extrinsic form) if extrinsic is True.
    """
    if extrinsic:
        if mat[2, 0] > _POLE_LIMIT:
            roll = np.arctan2(mat[0, 1], mat[0, 2])
            pitch = -np.pi / 2
            yaw = 0.0
            return np.array([roll, pitch, yaw])

        if mat[2, 0] < -_POLE_LIMIT:
            roll = np.arctan2(mat[0, 1], mat[0, 2])
            pitch = np.pi / 2
            yaw = 0.0
            return np.array([roll, pitch, yaw])

        roll = np.arctan2(mat[2, 1], mat[2, 2])
        pitch = -np.arcsin(mat[2, 0])
        yaw = np.arctan2(mat[1, 0], mat[0, 0])
        if degrees:
            roll = math.degrees(roll)
            pitch = math.degrees(pitch)
            yaw = math.degrees(yaw)
        return np.array([roll, pitch, yaw])
    else:
        if mat[0, 2] > _POLE_LIMIT:
            roll = np.arctan2(mat[1, 0], mat[1, 1])
            pitch = np.pi / 2
            yaw = 0.0
            return np.array([roll, pitch, yaw])

        if mat[0, 2] < -_POLE_LIMIT:
            roll = np.arctan2(mat[1, 0], mat[1, 1])
            pitch = -np.pi / 2
            yaw = 0.0
            return np.array([roll, pitch, yaw])
        roll = -math.atan2(mat[1, 2], mat[2, 2])
        pitch = math.asin(mat[0, 2])
        yaw = -math.atan2(mat[0, 1], mat[0, 0])

        if degrees:
            roll = math.degrees(roll)
            pitch = math.degrees(pitch)
            yaw = math.degrees(yaw)
        return np.array([roll, pitch, yaw])


def euler_to_rot_matrix(euler_angles: np.ndarray, degrees: bool = False, extrinsic: bool = True) -> np.ndarray:
    """Convert Euler XYZ or ZYX angles to rotation matrix.

    Args:
        euler_angles: Euler angles.
        degrees: Whether passed angles are in degrees.
        extrinsic: True if the euler angles follows the extrinsic angles
                   convention (equivalent to ZYX ordering but returned in the reverse) and False if it follows
                   the intrinsic angles conventions (equivalent to XYZ ordering).
                   Defaults to True.

    Returns:
        A 3x3 rotation matrix in its extrinsic or intrinsic form depends on the extrinsic argument.
    """
    if extrinsic:
        yaw, pitch, roll = euler_angles
    else:
        roll, pitch, yaw = euler_angles
    if degrees:
        roll = math.radians(roll)
        pitch = math.radians(pitch)
        yaw = math.radians(yaw)
    cr = np.cos(roll)
    sr = np.sin(roll)
    cy = np.cos(yaw)
    sy = np.sin(yaw)
    cp = np.cos(pitch)
    sp = np.sin(pitch)
    if extrinsic:
        return np.array(
            [
                [(cp * cr), ((cr * sp * sy) - (cy * sr)), ((cr * cy * sp) + (sr * sy))],
                [(cp * sr), ((cy * cr) + (sr * sp * sy)), ((cy * sp * sr) - (cr * sy))],
                [-sp, (cp * sy), (cy * cp)],
            ]
        )
    else:
        return np.array(
            [
                [(cp * cy), (-cp * sy), sp],
                [((cy * sr * sp) + (cr * sy)), ((cr * cy) - (sr * sp * sy)), (-cp * sr)],
                [((-cr * cy * sp) + (sr * sy)), ((cy * sr) + (cr * sp * sy)), (cr * cp)],
            ]
        )


def quat_to_euler_angles(quat: np.ndarray, degrees: bool = False, extrinsic: bool = True) -> np.ndarray:
    """Convert input quaternion to Euler XYZ or ZYX angles.

    Args:
        quat: Input quaternion (w, x, y, z).
        degrees: Whether returned angles should be in degrees. Defaults to False.
        extrinsic: True if the euler angles follows the extrinsic angles
                   convention (equivalent to ZYX ordering but returned in the reverse) and False if it follows
                   the intrinsic angles conventions (equivalent to XYZ ordering).
                   Defaults to True.


    Returns:
        Euler XYZ angles (intrinsic form) if extrinsic is False and Euler XYZ angles (extrinsic form) if extrinsic is True.
    """
    return matrix_to_euler_angles(quat_to_rot_matrix(quat), degrees=degrees, extrinsic=extrinsic)


def euler_angles_to_quat(euler_angles: np.ndarray, degrees: bool = False, extrinsic: bool = True) -> np.ndarray:
    """Convert Euler angles to quaternion.

    Args:
        euler_angles: Euler XYZ angles.
        degrees: Whether input angles are in degrees. Defaults to False.
        extrinsic: True if the euler angles follows the extrinsic angles
                   convention (equivalent to ZYX ordering but returned in the reverse) and False if it follows
                   the intrinsic angles conventions (equivalent to XYZ ordering).
                   Defaults to True.

    Returns:
        quaternion (w, x, y, z).
    """
    mat = np.array(euler_to_rot_matrix(euler_angles, degrees=degrees, extrinsic=extrinsic))
    return rot_matrix_to_quat(mat)


def lookat_to_quatf(camera: Gf.Vec3f, target: Gf.Vec3f, up: Gf.Vec3f) -> Gf.Quatf:
    """Compute a quaternion rotation from camera position looking at target.

    Args:
        camera: The camera position as a 3D vector.
        target: The target position to look at as a 3D vector.
        up: The up direction vector.

    Returns:
        Pxr quaternion object representing the look-at rotation.
    """
    direction = target - camera
    if direction.GetLength() < 1e-10:
        return Gf.Quatf(1.0, Gf.Vec3f(0, 0, 0))
    F = direction.GetNormalized()
    R = Gf.Cross(up, F).GetNormalized()
    U = Gf.Cross(F, R)

    q = Gf.Quatf()
    trace = R[0] + U[1] + F[2]
    if trace > 0.0:
        s = 0.5 / math.sqrt(trace + 1.0)
        q = Gf.Quatf(0.25 / s, Gf.Vec3f((U[2] - F[1]) * s, (F[0] - R[2]) * s, (R[1] - U[0]) * s))
    else:
        if R[0] > U[1] and R[0] > F[2]:
            s = 2.0 * math.sqrt(1.0 + R[0] - U[1] - F[2])
            q = Gf.Quatf((U[2] - F[1]) / s, Gf.Vec3f(0.25 * s, (U[0] + R[1]) / s, (F[0] + R[2]) / s))
        elif U[1] > F[2]:
            s = 2.0 * math.sqrt(1.0 + U[1] - R[0] - F[2])
            q = Gf.Quatf((F[0] - R[2]) / s, Gf.Vec3f((U[0] + R[1]) / s, 0.25 * s, (F[1] + U[2]) / s))
        else:
            s = 2.0 * math.sqrt(1.0 + F[2] - R[0] - U[1])
            q = Gf.Quatf((R[1] - U[0]) / s, Gf.Vec3f((F[0] + R[2]) / s, (F[1] + U[2]) / s, 0.25 * s))
    return q


def gf_quat_to_np_array(orientation: Gf.Quatd | Gf.Quatf | Gf.Quaternion) -> np.ndarray:
    """Converts a pxr Quaternion type to a numpy array following [w, x, y, z] convention.

    Args:
        orientation: Input quaternion object.

    Returns:
        A (4,) quaternion array in (w, x, y, z).
    """
    quat = np.zeros(4)
    quat[1:] = orientation.GetImaginary()
    quat[0] = orientation.GetReal()
    return quat


def gf_rotation_to_np_array(orientation: Gf.Rotation) -> np.ndarray:
    """Converts a pxr Rotation type to a numpy array following [w, x, y, z] convention.

    Args:
        orientation: Pxr rotation object.

    Returns:
        A (4,) quaternion array in (w, x, y, z).
    """
    return gf_quat_to_np_array(orientation.GetQuat())
