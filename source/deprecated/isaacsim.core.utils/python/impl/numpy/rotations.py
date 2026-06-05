# SPDX-FileCopyrightText: Copyright (c) 2022-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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

"""Provides NumPy-based functions for 3D rotation conversions between quaternions, Euler angles, rotation matrices, and rotation vectors."""

from __future__ import annotations

import numpy as np
from pxr import Gf
from scipy.spatial.transform import Rotation


def gf_quat_to_tensor(orientation: Gf.Quatd | Gf.Quatf | Gf.Quaternion, device: object = None) -> np.ndarray:
    """Converts a pxr Quaternion type to a numpy array following [w, x, y, z] convention.

    Args:
        orientation: Input quaternion from USD.
        device: Device parameter (unused, maintained for compatibility).

    Returns:
        Quaternion as numpy array in [w, x, y, z] format.
    """
    quat = np.zeros(4)
    quat[1:] = orientation.GetImaginary()
    quat[0] = orientation.GetReal()
    return quat


def euler_angles_to_quats(
    euler_angles: np.ndarray, degrees: bool = False, extrinsic: bool = True, device: object = None
) -> np.ndarray:
    """Vectorized version of converting euler angles to quaternion (scalar first).

    Args:
        euler_angles: euler angles with shape (N, 3) or (3,) representation XYZ in extrinsic coordinates
        degrees: True if degrees, False if radians.
        extrinsic: True if the euler angles follows the extrinsic angles
                   convention (equivalent to ZYX ordering but returned in the reverse) and False if it follows
                   the intrinsic angles conventions (equivalent to XYZ ordering).
        device: Device parameter (unused, maintained for compatibility).

    Returns:
        quaternions representation of the angles (N, 4) or (4,) - scalar first.
    """
    if extrinsic:
        order = "xyz"
    else:
        order = "XYZ"
    rot = Rotation.from_euler(order, euler_angles, degrees=degrees)
    result = rot.as_quat()
    if len(result.shape) == 1:
        result = result[[3, 0, 1, 2]]
    else:
        result = result[:, [3, 0, 1, 2]]
    return result


def quats_to_euler_angles(
    quaternions: np.ndarray, degrees: bool = False, extrinsic: bool = True, device: object = None
) -> np.ndarray:
    """Vectorized version of converting quaternions (scalar first) to euler angles.

    Args:
        quaternions: quaternions with shape (N, 4) or (4,) - scalar first
        degrees: Return euler angles in degrees if True, radians if False.
        extrinsic: True if the euler angles follows the extrinsic angles
                   convention (equivalent to ZYX ordering but returned in the reverse) and False if it follows
                   the intrinsic angles conventions (equivalent to XYZ ordering).
        device: Device parameter (unused, maintained for compatibility).

    Returns:
        Euler angles in extrinsic or intrinsic coordinates XYZ order with shape (N, 3) or (3,) corresponding to the quaternion rotations
    """
    if extrinsic:
        order = "xyz"
    else:
        order = "XYZ"
    if len(quaternions.shape) == 1:
        q = quaternions[[1, 2, 3, 0]]
    else:
        q = quaternions[:, [1, 2, 3, 0]]
    rot = Rotation.from_quat(q)
    result = rot.as_euler(order, degrees)
    return result


def rot_matrices_to_quats(rotation_matrices: np.ndarray, device: object = None) -> np.ndarray:
    """Vectorized version of converting rotation matrices to quaternions.

    Args:
        rotation_matrices: N Rotation matrices with shape (N, 3, 3) or (3, 3)
        device: Device parameter (unused, maintained for compatibility).

    Returns:
        quaternion representation of the rotation matrices (N, 4) or (4,) - scalar first
    """
    rot = Rotation.from_matrix(rotation_matrices)
    result = rot.as_quat()
    if len(result.shape) == 1:
        result = result[[3, 0, 1, 2]]
    else:
        result = result[:, [3, 0, 1, 2]]
    return result


def quats_to_rot_matrices(quaternions: np.ndarray, device: object = None) -> np.ndarray:
    """Vectorized version of converting quaternions to rotation matrices.

    Args:
        quaternions: quaternions with shape (N, 4) or (4,) and scalar first
        device: Device parameter (unused, maintained for compatibility).

    Returns:
        N Rotation matrices with shape (N, 3, 3) or (3, 3)
    """
    if len(quaternions.shape) == 1:
        q = quaternions[[1, 2, 3, 0]]
    else:
        q = quaternions[:, [1, 2, 3, 0]]
    rot = Rotation.from_quat(q)
    result = rot.as_matrix()
    return result


def rotvecs_to_quats(rotation_vectors: np.ndarray, degrees: bool = False, device: object = None) -> np.ndarray:
    """Vectorized version of converting rotation vectors to quaternions.

    Args:
        rotation_vectors: N rotation vectors with shape (N, 3) or (3,).  The magnitude of the rotation vector describes the magnitude of the rotation.
            The normalized rotation vector represents the axis of rotation.
        degrees: The magnitude of the rotation vector will be interpreted as degrees if True, and radians if False.
        device: Device parameter (unused, maintained for compatibility).

    Returns:
        quaternion representation of the rotation matrices (N, 4) or (4,) - scalar first
    """
    rot = Rotation.from_rotvec(rotation_vectors, degrees)
    result = rot.as_quat()
    if len(result.shape) == 1:
        result = result[[3, 0, 1, 2]]
    else:
        result = result[:, [3, 0, 1, 2]]
    return result


def quats_to_rotvecs(quaternions: np.ndarray, device: object = None) -> np.ndarray:
    """Vectorized version of converting quaternions to rotation vectors.

    Args:
        quaternions: quaternions with shape (N, 4) or (4,) and scalar first
        device: Device parameter (unused, maintained for compatibility).

    Returns:
        N rotation vectors with shape (N,3) or (3,).  The magnitude of the rotation vector describes the magnitude of the rotation.
            The normalized rotation vector represents the axis of rotation.
    """
    if len(quaternions.shape) == 1:
        q = quaternions[[1, 2, 3, 0]]
    else:
        q = quaternions[:, [1, 2, 3, 0]]
    rot = Rotation.from_quat(q)
    result = rot.as_rotvec()
    return result


def rad2deg(radian_value: np.ndarray, device: object = None) -> np.ndarray:
    """Converts angles from radians to degrees.

    Args:
        radian_value: Angle values in radians.
        device: Device parameter (unused, maintained for compatibility).

    Returns:
        Angle values converted to degrees.
    """
    return np.rad2deg(radian_value)


def deg2rad(degree_value: np.ndarray, device: object = None) -> np.ndarray:
    """Converts angles from degrees to radians.

    Args:
        degree_value: Angle values in degrees.
        device: Device parameter (unused, maintained for compatibility).

    Returns:
        Angle values converted to radians.
    """
    return np.deg2rad(degree_value)


def xyzw2wxyz(q: np.ndarray, ret_torch: bool = False) -> np.ndarray:
    """Converts quaternion from XYZW format to WXYZ format.

    Args:
        q: Quaternion array in XYZW format (x, y, z, w).
        ret_torch: Currently unused parameter for potential torch tensor output.

    Returns:
        Quaternion array in WXYZ format (w, x, y, z).
    """
    return np.roll(q, 1, -1)


def wxyz2xyzw(q: np.ndarray, ret_torch: bool = False) -> np.ndarray:
    """Converts quaternion from WXYZ order to XYZW order.

    Args:
        q: Quaternion in WXYZ order.
        ret_torch: Return format parameter (unused, maintained for compatibility).

    Returns:
        Quaternion in XYZW order.
    """
    return np.roll(q, -1, -1)
