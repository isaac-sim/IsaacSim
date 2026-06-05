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

"""Deprecated transformation utility functions."""

from collections.abc import Sequence

import numpy as np

# isaacsim
from isaacsim.core.deprecation_manager import import_module
from isaacsim.core.utils.rotations import gf_quat_to_np_array

# omniverse
from pxr import Gf, Usd, UsdGeom
from scipy.spatial.transform import Rotation

torch = import_module("torch")


def tf_matrix_from_pose(translation: Sequence[float], orientation: Sequence[float]) -> np.ndarray:
    """Compute input pose to transformation matrix.

    Args:
        translation: The translation vector.
        orientation: The orientation quaternion in scalar-first (w, x, y, z) order.

    Returns:
        A 4x4 matrix.
    """
    translation = np.asarray(translation)
    orientation = np.asarray(orientation)
    mat = Gf.Transform()
    mat.SetRotation(Gf.Rotation(Gf.Quatd(*orientation.tolist())))
    mat.SetTranslation(Gf.Vec3d(*translation.tolist()))
    return np.transpose(mat.GetMatrix())


def pose_from_tf_matrix(transformation: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    """Gets pose corresponding to input transformation matrix.

    Args:
        transformation: Column-major transformation matrix. shape is (4, 4).

    Returns:
        first index is translation corresponding to transformation. shape is (3, ).
        second index is quaternion orientation corresponding to transformation.
        quaternion is scalar-first (w, x, y, z). shape is (4, ).
    """
    mat = Gf.Transform()
    mat.SetMatrix(Gf.Matrix4d(np.transpose(transformation)))
    calculated_translation = np.array(mat.GetTranslation())
    calculated_orientation = gf_quat_to_np_array(mat.GetRotation().GetQuat())
    return calculated_translation, calculated_orientation


def tf_matrices_from_poses(
    translations: np.ndarray | torch.Tensor, orientations: np.ndarray | torch.Tensor
) -> np.ndarray | torch.Tensor:
    """Compute transformation matrices from translation and orientation arrays.

    Args:
        translations: Translations with shape (N, 3).
        orientations: Quaternion orientations (scalar first) with shape (N, 4).

    Returns:
        Transformation matrices with shape (N, 4, 4).
    """
    # Note: Imported here to prevent cyclic dependency in the module.
    from isaacsim.core.api.simulation_context.simulation_context import SimulationContext

    # TODO: add a torch pathway
    backend = "numpy"
    if SimulationContext.instance() is not None:
        backend = SimulationContext.instance().backend
    if backend == "numpy":
        result = np.zeros([orientations.shape[0], 4, 4], dtype=np.float32)
        r = Rotation.from_quat(orientations[:, [1, 2, 3, 0]])
        result[:, :3, :3] = r.as_matrix()
        result[:, :3, 3] = translations
        result[:, 3, 3] = 1
    elif backend == "torch":
        device = None
        if SimulationContext.instance() is not None:
            device = SimulationContext.instance().device
        result = torch.zeros([orientations.shape[0], 4, 4], dtype=torch.float32, device=device)
        r = Rotation.from_quat(orientations[:, [1, 2, 3, 0]].detach().cpu().numpy())
        result[:, :3, :3] = torch.from_numpy(r.as_matrix()).float().to(device)
        result[:, :3, 3] = translations
        result[:, 3, 3] = 1
    return result


def get_relative_transform(source_prim: Usd.Prim, target_prim: Usd.Prim) -> np.ndarray:
    """Get the relative transformation matrix from the source prim to the target prim.

    Args:
        source_prim: source prim from which frame to compute the relative transform.
        target_prim: target prim to which frame to compute the relative transform.

    Returns:
        Column-major transformation matrix with shape (4, 4).
    """
    # Row-major transformation matrix
    source_to_world_row_major_tf = UsdGeom.Xformable(source_prim).ComputeLocalToWorldTransform(Usd.TimeCode.Default())
    target_to_world_row_major_tf = UsdGeom.Xformable(target_prim).ComputeLocalToWorldTransform(Usd.TimeCode.Default())

    # Convert to column-major transformation matrix
    source_to_world_column_major_tf = np.transpose(source_to_world_row_major_tf)
    target_to_world_column_major_tf = np.transpose(target_to_world_row_major_tf)

    world_to_target_column_major_tf = np.linalg.inv(target_to_world_column_major_tf)
    source_to_target_column_major_tf = world_to_target_column_major_tf @ source_to_world_column_major_tf

    return source_to_target_column_major_tf


def get_translation_from_target(
    translation_from_source: np.ndarray, source_prim: Usd.Prim, target_prim: Usd.Prim
) -> np.ndarray:
    """Get a translation with respect to the target's frame, from a translation in the source's frame.

    Args:
        translation_from_source: translation from the frame of the prim at source_path. Shape is (3, ).
        source_prim: prim path of the prim whose frame the original/untransformed translation
                           (translation_from_source) is defined with respect to.
        target_prim: prim path of the prim whose frame corresponds to the target frame that the returned
                           translation will be defined with respect to.

    Returns:
        translation with respect to the target's frame. Shape is (3, ).
    """
    translation_from_source_homogenous = np.pad(translation_from_source, ((0, 1)), constant_values=1.0)

    source_to_target = get_relative_transform(source_prim, target_prim)

    translation_from_target_homogenous = translation_from_source_homogenous @ np.transpose(source_to_target)
    translation_from_target = translation_from_target_homogenous[:-1]

    return translation_from_target


def get_world_pose_from_relative(
    coord_prim: Usd.Prim, relative_translation: np.ndarray, relative_orientation: np.ndarray
) -> tuple[np.ndarray, np.ndarray]:
    """Get a pose defined in the world frame from a pose defined relative to the frame of the coord_prim.

    Args:
        coord_prim: path of the prim whose frame the relative pose is defined with respect to.
        relative_translation: translation relative to the frame of the prim at prim_path. Shape is (3, ).
        relative_orientation: quaternion orientation relative to the frame of the prim at prim_path.
                                           Quaternion is scalar-first (w, x, y, z). Shape is (4, ).

    Returns:
        first index is position in the world frame. Shape is (3, ). Second index is
        quaternion orientation in the world frame. Quaternion is scalar-first
        (w, x, y, z). Shape is (4, ).
    """
    # Row-major transformation matrix from the prim's coordinate system to the world coordinate system
    prim_transform_matrix = UsdGeom.Xformable(coord_prim).ComputeLocalToWorldTransform(Usd.TimeCode.Default())

    # Convert transformation matrix to column-major
    prim_to_world = np.transpose(prim_transform_matrix)

    # Column-major transformation matrix from the pose to the frame the pose is defined with respect to
    relative_pose_to_prim = tf_matrix_from_pose(relative_translation, relative_orientation)

    # Chain the transformations
    relative_pose_to_world = prim_to_world @ relative_pose_to_prim

    # Translation and quaternion with respect to the world frame of the relatively defined pose
    world_position, world_orientation = pose_from_tf_matrix(relative_pose_to_world)

    return world_position, world_orientation


def get_transform_with_normalized_rotation(transform: np.ndarray) -> np.ndarray:
    """Get the transform after normalizing rotation component.

    Args:
        transform: transformation matrix with shape (4, 4).

    Returns:
        transformation matrix with normalized rotation with shape (4, 4).
    """
    transform_without_scale = np.copy(transform.astype(float))

    rotation_matrix = transform[:3, :3]

    column_magnitudes = np.linalg.norm(rotation_matrix, axis=0)
    column_magnitudes = np.where(column_magnitudes == 0, 1.0, column_magnitudes)
    normalized_rotation = rotation_matrix / column_magnitudes

    transform_without_scale[:3, :3] = normalized_rotation

    return transform_without_scale
