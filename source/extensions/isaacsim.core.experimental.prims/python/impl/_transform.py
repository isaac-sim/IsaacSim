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

from __future__ import annotations

import numpy as np
from pxr import Gf
from scipy.spatial.transform import Rotation


def _gf_quaternion_to_array(quaternion: Gf.Quatf | Gf.Quatd | Gf.Quaternion) -> np.ndarray:
    """Convert a Pixar Graphics Framework quaternion to a NumPy array.

    Args:
        quaternion: Pixar quaternion object to convert.

    Returns:
        NumPy array containing quaternion components in [w, x, y, z] format.
    """
    return np.array([quaternion.GetReal(), *quaternion.GetImaginary()], dtype=np.float32)


def _compose_transform_matrices(translations: np.ndarray, orientations: np.ndarray) -> np.ndarray:
    """Compose 4x4 transformation matrices from translation vectors and quaternion orientations.

    Args:
        translations: Translation vectors for each transform.
        orientations: Quaternion orientations in [w, x, y, z] format.

    Returns:
        Array of 4x4 transformation matrices.
    """
    matrices = np.zeros((translations.shape[0], 4, 4), dtype=translations.dtype)
    matrices[:, :3, :3] = Rotation.from_quat(orientations[:, [1, 2, 3, 0]]).as_matrix()
    matrices[:, :3, 3] = translations
    matrices[:, 3, 3] = 1.0
    return matrices


def local_from_world(
    parent_transforms: np.ndarray, positions: np.ndarray, orientations: np.ndarray
) -> tuple[np.ndarray, np.ndarray]:
    """Transform world coordinates to local coordinates relative to parent transforms.

    Args:
        parent_transforms: Parent transformation matrices.
        positions: World space positions to transform.
        orientations: World space orientations in quaternion format.

    Returns:
        A tuple of (local_translations, local_orientations) in the parent coordinate frame.
    """
    local_translations = np.zeros_like(positions)
    local_orientations = np.zeros_like(orientations)
    world_transforms = _compose_transform_matrices(positions, orientations)
    for i in range(positions.shape[0]):
        local_transform = np.matmul(np.linalg.inv(np.transpose(parent_transforms[i])), world_transforms[i])
        transform = Gf.Transform()
        transform.SetMatrix(Gf.Matrix4d(np.transpose(local_transform).tolist()))
        local_translations[i] = np.array(transform.GetTranslation())
        local_orientations[i] = _gf_quaternion_to_array(transform.GetRotation().GetQuat())
    return local_translations, local_orientations


def world_from_local(
    parent_transforms: np.ndarray, translations: np.ndarray, orientations: np.ndarray
) -> tuple[np.ndarray, np.ndarray]:
    """Transform local coordinates to world coordinates using parent transforms.

    Args:
        parent_transforms: Parent transformation matrices.
        translations: Local space translations to transform.
        orientations: Local space orientations in quaternion format.

    Returns:
        A tuple of (world_positions, world_orientations) in world coordinate frame.
    """
    world_positions = np.zeros_like(translations)
    world_orientations = np.zeros_like(orientations)
    local_transforms = _compose_transform_matrices(translations, orientations)
    for i in range(translations.shape[0]):
        world_transform = np.matmul(np.transpose(parent_transforms[i]), local_transforms[i])
        transform = Gf.Transform()
        transform.SetMatrix(Gf.Matrix4d(np.transpose(world_transform).tolist()))
        world_positions[i] = np.array(transform.GetTranslation())
        world_orientations[i] = _gf_quaternion_to_array(transform.GetRotation().GetQuat())
    return world_positions, world_orientations
