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

"""Utilities for 3D transformation operations including coordinate frame conversions and pose manipulations using NumPy arrays."""

import numpy as np
from isaacsim.core.utils.numpy.rotations import gf_quat_to_tensor, wxyz2xyzw
from isaacsim.core.utils.numpy.tensor import create_zeros_tensor
from pxr import Gf
from scipy.spatial.transform import Rotation


def tf_matrices_from_poses(translations: np.ndarray, orientations: np.ndarray, device: object = None) -> np.ndarray:
    """Compute transformation matrices from translation and orientation arrays.

    Args:
        translations: Translations with shape (N, 3).
        orientations: Quaternion orientations (scalar first) with shape (N, 4).
        device: Device for tensor operations.

    Returns:
        Transformation matrices with shape (N, 4, 4).
    """
    result = np.zeros([orientations.shape[0], 4, 4], dtype=np.float32)
    r = Rotation.from_quat(orientations[:, [1, 2, 3, 0]])
    result[:, :3, :3] = r.as_matrix()
    result[:, :3, 3] = translations
    result[:, 3, 3] = 1
    return result


def get_local_from_world(
    parent_transforms: np.ndarray, positions: np.ndarray, orientations: np.ndarray, device: object = None
) -> tuple:
    """Convert world space poses to local space relative to parent transformations.

    Args:
        parent_transforms: Parent transformation matrices.
        positions: World space positions with shape (N, 3).
        orientations: World space quaternion orientations with shape (N, 4).
        device: Device for tensor operations.

    Returns:
        Tuple of (local_translations, local_orientations) in parent coordinate frame.
    """
    calculated_translations = create_zeros_tensor(shape=[positions.shape[0], 3], dtype="float32", device=device)
    calculated_orientations = create_zeros_tensor(shape=[positions.shape[0], 4], dtype="float32", device=device)
    my_world_transforms = tf_matrices_from_poses(translations=positions, orientations=orientations)
    # TODO: vectorize this
    for i in range(positions.shape[0]):
        local_transform = np.matmul(np.linalg.inv(np.transpose(parent_transforms[i])), my_world_transforms[i])
        transform = Gf.Transform()
        transform.SetMatrix(Gf.Matrix4d(np.transpose(local_transform).tolist()))
        calculated_translations[i] = np.array(transform.GetTranslation())
        calculated_orientations[i] = gf_quat_to_tensor(transform.GetRotation().GetQuat())
    return calculated_translations, calculated_orientations


def get_world_from_local(
    parent_transforms: np.ndarray, translations: np.ndarray, orientations: np.ndarray, device: object = None
) -> tuple:
    """Convert local space poses to world space using parent transformations.

    Args:
        parent_transforms: Parent transformation matrices.
        translations: Local space translations with shape (N, 3).
        orientations: Local space quaternion orientations with shape (N, 4).
        device: Device for tensor operations.

    Returns:
        Tuple of (world_positions, world_orientations) in world coordinate frame.
    """
    calculated_positions = create_zeros_tensor(shape=[translations.shape[0], 3], dtype="float32", device=device)
    calculated_orientations = create_zeros_tensor(shape=[translations.shape[0], 4], dtype="float32", device=device)
    my_local_transforms = tf_matrices_from_poses(translations=translations, orientations=orientations)
    # TODO: vectorize this
    for i in range(translations.shape[0]):
        world_transform = np.matmul(np.transpose(parent_transforms[i]), my_local_transforms[i])
        transform = Gf.Transform()
        transform.SetMatrix(Gf.Matrix4d(np.transpose(world_transform).tolist()))
        calculated_positions[i] = np.array(transform.GetTranslation())
        calculated_orientations[i] = gf_quat_to_tensor(transform.GetRotation().GetQuat())
    return calculated_positions, calculated_orientations


def get_pose(positions: np.ndarray, orientations: np.ndarray, device: object = None) -> np.ndarray:
    """Concatenate position and orientation arrays into a single pose array.

    Args:
        positions: Position array with shape (N, 3).
        orientations: Orientation array with shape (N, 4).
        device: Device for tensor operations.

    Returns:
        Combined pose array with shape (N, 7) containing positions and orientations.
    """
    pose = np.concatenate([positions, orientations], axis=-1)
    return pose


def assign_pose(
    current_positions: np.ndarray,
    current_orientations: np.ndarray,
    positions: object,
    orientations: object,
    indices: object,
    device: object = None,
    pose: object = None,
) -> np.ndarray:
    """Update pose arrays by assigning new positions and orientations at specified indices.

    Args:
        current_positions: Current position array with shape (N, 3).
        current_orientations: Current orientation array with shape (N, 4).
        positions: New positions to assign. If None, uses current positions at indices.
        orientations: New orientations to assign. If None, uses current orientations at indices.
        indices: Indices where new poses should be assigned.
        device: Device for tensor operations.
        pose: Optional pose parameter.

    Returns:
        Updated pose array with new positions and orientations assigned at specified indices.
    """
    if positions is None:
        positions = current_positions[indices]
    if orientations is None:
        orientations = current_orientations[indices]
    orientations = wxyz2xyzw(orientations)
    current_orientations = wxyz2xyzw(current_orientations)
    old_pose = get_pose(current_positions, current_orientations)
    new_pose = get_pose(positions, orientations)
    old_pose[indices] = new_pose
    return old_pose
