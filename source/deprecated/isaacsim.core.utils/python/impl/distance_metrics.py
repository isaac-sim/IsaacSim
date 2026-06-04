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

"""Deprecated distance metric functions."""

from __future__ import annotations

# omniverse
import numpy as np
from pxr import Gf

# python


def _standardize_transform_matrix(t1: np.ndarray | Gf.Matrix4d) -> np.ndarray:
    """Check that input matrix is 4x4 and convert it to a numpy array.

    .. note::
        If input matrix is a Gf.Matrix4d() object, it is transposed to convert from column-major to row-major.

    Args:
        t1: Input 4x4 matrix, either a Gf.Matrix4d or a numpy compatible type.

    Returns:
        Standardized 4x4 matrix.
    """
    if np.shape(t1) != (4, 4):
        raise ValueError(f"Input transformation matrix has the wrong shape: {np.shape(t1)} != (4, 4).")

    if isinstance(t1, Gf.Matrix4d):
        t1 = np.array(t1).T
    else:
        t1 = np.array(t1)

    return t1


def _standardize_rotation_matrix(r1: np.ndarray | Gf.Matrix3d | Gf.Matrix4d) -> np.ndarray:
    """Extract rotation matrix from input and convert it to numpy array.

    If input matrix is a 4x4 matrix, then the rotation matrix component is extracted.
    Otherwise, it is checked that a 3x3 matrix is provided.

    .. note::
        If input matrix is a Gf.Matrix3d() object, it is transposed to convert from column-major to row-major.

    Args:
        r1: Input rotation or transformation matrix.

    Returns:
        Standardized 3x3 matrix.
    """
    if np.shape(r1) == (4, 4):
        r1 = _standardize_transform_matrix(r1)
        return r1[:3, :3]

    if np.shape(r1) != (3, 3):
        raise ValueError(f"Rotation matrix has the wrong shape: {np.shape(r1)} != (3, 3).")

    if isinstance(r1, Gf.Matrix3d):
        r1 = np.array(r1).T
    else:
        r1 = np.array(r1)

    return r1


def _standardize_translation_vector(t1: np.ndarray | Gf.Matrix4d) -> np.ndarray:
    """Extract translation vector from input and convert it to numpy array.

    If input matrix is a 4x4 matrix, then the translation component is extracted.
    Otherwise, it is checked that is a 3-dimensional vector and flattened into an array.

    Args:
        t1: Input 3-D vector or transformation matrix.

    Returns:
        Standardized 3-dimensional array.
    """
    if np.shape(t1) == (4, 4):
        t1 = _standardize_transform_matrix(t1)
        return t1[:3, 3].flatten()

    t1 = np.array(t1).flatten()

    if t1.shape != (3,):
        raise ValueError(f"Translation vector has the wrong shape: {np.shape(t1)} != (3,).")

    return t1


def weighted_translational_distance(
    t1: np.ndarray | Gf.Matrix4d,
    t2: np.ndarray | Gf.Matrix4d,
    weight_matrix: np.ndarray = np.eye(3),
) -> np.ndarray:
    """Computes the weighted distance between two translation vectors.

    The distance calculation has the form sqrt(x.T W x), where

    | - x is the vector difference between t1 and t2.
    | - W is a weight matrix.

    Given the identity weight matrix, this is equivalent to the ||t1-t2||.

    Usage:
        This formulation can be used to weight an arbitrary axis of the translation difference.
        Letting x = t1-t2 = a1*b1 + a2*b2 + a3*b3 (where b1,b2,b3 are column basis vectors, and a1,a2,a3 are constants),
        When W = I: x.T W x = sqrt(a1^2 + a2^2 + a3^2).
        To weight the b1 axis by 2, let W take the form (R.T @ ([4,1,1]@I) @ R) where:

        | - I is the identity matrix.
        | - R is a rotation matrix of the form [b1,b2,b3].T

        This is effectively equivalent to ||[2*e1,e2,e3] @ [b1,b2,b3].T @ x|| = sqrt(4*a1^2 + a2^2 + a3^2).

        | - e1,e2,e3 are the elementary basis vectors.

    Args:
        t1: 3d translation vectors or 4x4 transformation matrices.
        t2: 3d translation vectors or 4x4 transformation matrices.
        weight_matrix: A 3x3 positive semidefinite matrix of weights.

    Returns:
        The weighted norm of the difference (t1-t2).
    """
    t1 = _standardize_translation_vector(t1)
    t2 = _standardize_translation_vector(t2)

    return np.sqrt((t1 - t2).T @ weight_matrix @ (t1 - t2))


def rotational_distance_angle(
    r1: np.ndarray | Gf.Matrix3d | Gf.Matrix4d, r2: np.ndarray | Gf.Matrix3d | Gf.Matrix4d
) -> np.ndarray:
    """Computes the weighted distance between two rotations using inner product.

    Note:
        If r1 and r2 are GfMatrix3d() objects, the transformation matrices will be transposed in the distance
        calculations.

    Args:
        r1: Rotation matrices or 4x4 transformation matrices.
        r2: Rotation matrices or 4x4 transformation matrices.

    Returns:
        The magnitude of the angle of rotation from r1 to r2.
    """
    r1 = _standardize_rotation_matrix(r1)
    r2 = _standardize_rotation_matrix(r2)

    # np.clip handles floating point errors that can occur in valid rotation matrices.
    return np.arccos(np.clip((np.trace(r1 @ r2.T) - 1) / 2, -1, 1))


def rotational_distance_identity_matrix_deviation(
    r1: np.ndarray | Gf.Matrix4d | Gf.Matrix3d, r2: np.ndarray | Gf.Matrix4d | Gf.Matrix3d
) -> np.ndarray:
    """Computes the distance between two rotations using deviation from identity matrix.

    Note:
        If r1 and r2 are GfMatrix3d() objects, the transformation matrices will be transposed in the distance
        calculations.

    Args:
        r1: Rotation matrices or 4x4 transformation matrices.
        r2: Rotation matrices or 4x4 transformation matrices.

    Returns:
        The Frobenius norm ||I-r1*r2^T||, where I is the identity matrix.
    """
    r1 = _standardize_rotation_matrix(r1)
    r2 = _standardize_rotation_matrix(r2)

    rotational_distance = np.linalg.norm(np.eye(3) - r1[:3, :3] @ r2[:3, :3].T)

    return rotational_distance


def rotational_distance_single_axis(
    r1: np.ndarray | Gf.Matrix4d | Gf.Matrix3d,
    r2: np.ndarray | Gf.Matrix4d | Gf.Matrix3d,
    axis: np.ndarray,
) -> np.ndarray:
    """Computes the distance between two rotations w.r.t. input axis.

    Note:
        If r1 and r2 are GfMatrix3d() objects, the transformation matrices will be transposed in the distance
        calculations.

    Usage:
        If the robot were holding a cup aligned with its z-axis,
        it would be important to align the z-axis of the robot with
        the z-axis of the world frame.  This could be accomplished by
        letting

        | -r1 be the rotation of the robot end effector
        | -r2 be any rotation matrix for a rotation about the z axis
        | -axis = [0,0,1]

    Args:
        r1: Rotation matrices or 4x4 transformation matrices.
        r2: Rotation matrices or 4x4 transformation matrices.
        axis: A 3d vector that will be rotated by r1 and r2.

    Returns:
        The angle between (r1 @ axis) and (r2 @ axis).
    """
    r1 = _standardize_rotation_matrix(r1)
    r2 = _standardize_rotation_matrix(r2)

    if len(axis) != 3:
        raise ValueError(f"Rotation axis has the wrong shape: {len(axis)} != 3.")

    axis = np.array(axis, dtype=float)
    axis_norm = np.linalg.norm(axis)
    if axis_norm == 0:
        raise ValueError("Rotation axis has zero magnitude.")
    axis = axis / axis_norm

    return np.arccos(np.clip(np.dot(r1 @ axis, r2 @ axis), -1, 1))
