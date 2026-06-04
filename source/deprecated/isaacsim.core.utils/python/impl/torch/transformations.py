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

"""Tensor-based transformation utilities for 3D poses, coordinate frame conversions, and spatial operations."""

from isaacsim.core.deprecation_manager import import_module
from isaacsim.core.utils.torch.rotations import (
    gf_quat_to_tensor,
    quat_apply,
    quat_conjugate,
    quat_mul,
    wxyz2xyzw,
)
from isaacsim.core.utils.torch.tensor import create_zeros_tensor
from pxr import Gf
from scipy.spatial.transform import Rotation

torch = import_module("torch")


def tf_matrices_from_poses(
    translations: torch.Tensor, orientations: torch.Tensor, device: object = None
) -> torch.Tensor:
    """Compute transformation matrices from translation and orientation tensors.

    Args:
        translations: Translations with shape (N, 3).
        orientations: Quaternion orientations (scalar first) with shape (N, 4).
        device: Device for tensor operations.

    Returns:
        Transformation matrices with shape (N, 4, 4).
    """
    result = torch.zeros([orientations.shape[0], 4, 4], dtype=torch.float32, device=device)
    r = Rotation.from_quat(orientations[:, [1, 2, 3, 0]].detach().cpu().numpy())
    result[:, :3, :3] = torch.from_numpy(r.as_matrix()).float().to(device)
    result[:, :3, 3] = translations
    result[:, 3, 3] = 1
    return result


def get_local_from_world(parent_transforms: object, positions: object, orientations: object, device: object) -> tuple:
    """Converts world-space positions and orientations to local-space relative to parent transforms.

    Args:
        parent_transforms: Transformation matrices of parent objects.
        positions: World-space positions to convert.
        orientations: World-space orientations to convert.
        device: Device for tensor operations.

    Returns:
        Tuple of (local_translations, local_orientations) in parent coordinate frames.
    """
    calculated_translations = create_zeros_tensor(shape=[positions.shape[0], 3], dtype="float32", device=device)
    calculated_orientations = create_zeros_tensor(shape=[positions.shape[0], 4], dtype="float32", device=device)
    my_world_transforms = tf_matrices_from_poses(translations=positions, orientations=orientations, device=device)
    # TODO: vectorize this
    for i in range(positions.shape[0]):
        local_transform = torch.linalg.solve(
            torch.transpose(parent_transforms[i].to(device), 0, 1), my_world_transforms[i]
        )
        transform = Gf.Transform()
        transform.SetMatrix(Gf.Matrix4d(torch.transpose(local_transform, 0, 1).tolist()))
        calculated_translations[i] = torch.tensor(transform.GetTranslation(), dtype=torch.float32, device=device)
        calculated_orientations[i] = gf_quat_to_tensor(transform.GetRotation().GetQuat())
    return calculated_translations, calculated_orientations


def get_world_from_local(
    parent_transforms: object, translations: object, orientations: object, device: object
) -> tuple:
    """Converts local-space translations and orientations to world-space using parent transforms.

    Args:
        parent_transforms: Transformation matrices of parent objects.
        translations: Local-space translations to convert.
        orientations: Local-space orientations to convert.
        device: Device for tensor operations.

    Returns:
        Tuple of (world_positions, world_orientations) in global coordinate frame.
    """
    calculated_positions = create_zeros_tensor(shape=[translations.shape[0], 3], dtype="float32", device=device)
    calculated_orientations = create_zeros_tensor(shape=[translations.shape[0], 4], dtype="float32", device=device)
    my_local_transforms = tf_matrices_from_poses(translations=translations, orientations=orientations, device=device)
    # TODO: vectorize this
    for i in range(translations.shape[0]):
        world_transform = torch.matmul(torch.transpose(parent_transforms[i], 0, 1), my_local_transforms[i])
        transform = Gf.Transform()
        transform.SetMatrix(Gf.Matrix4d(torch.transpose(world_transform, 0, 1).tolist()))
        calculated_positions[i] = torch.tensor(transform.GetTranslation(), dtype=torch.float32, device=device)
        calculated_orientations[i] = gf_quat_to_tensor(transform.GetRotation().GetQuat())
    return calculated_positions, calculated_orientations


def get_pose(positions: object, orientations: object, device: object) -> object:
    """Combines position and orientation arrays into a single pose tensor.

    Args:
        positions: Position values (shape N, 3).
        orientations: Orientation quaternions (shape N, 4).
        device: Device for tensor operations.

    Returns:
        Combined pose tensor with shape (N, 7) containing positions and orientations.
    """
    if type(positions) != torch.Tensor:
        positions = torch.tensor(positions, device=device, dtype=torch.float)
    if type(orientations) != torch.Tensor:
        orientations = torch.tensor(orientations, device=device, dtype=torch.float)
    pose = torch.cat([positions.to(device), orientations.to(device)], dim=-1)
    return pose


@torch.jit.script
def get_world_from_local_position(pos_offset_local: torch.Tensor, pose_global: torch.Tensor):  # noqa: ANN201
    """Convert a point from the local frame to the global frame.

    Args:
        pos_offset_local: Point in local frame. Shape: [N, 3]
        pose_global: The spatial pose of this point. Shape: [N, 7], where
            the first 3 elements are position (x, y, z) and the last 4 elements
            are quaternion orientation in scalar-first format (w, x, y, z).

    Returns:
        Position in the global frame. Shape: [N, 3]
    """
    quat_global = pose_global[:, 3:7]
    pos_offset_global = quat_apply(quat_global, pos_offset_local)
    result_pos_global = pos_offset_global + pose_global[:, 0:3]

    return result_pos_global


# NB: do not make this function jit, since it is passed around as an argument.
def normalise_quat_in_pose(pose: object) -> object:  # noqa: N802
    """Takes a pose and normalises the quaternion portion of it.

    Args:
        pose: shape N, 7

    Returns:
        Pose with normalised quat. Shape N, 7
    """
    pos = pose[:, 0:3]
    quat = pose[:, 3:7]
    quat /= torch.norm(quat, dim=-1, p=2).reshape(-1, 1)
    return torch.cat([pos, quat], dim=-1)


@torch.jit.script
def tf_inverse(q: torch.Tensor, t: torch.Tensor) -> tuple[torch.Tensor, torch.Tensor]:
    """Compute the inverse of a transform given by quaternion and translation.

    Args:
        q: Rotation quaternion tensor of shape (N, 4) in [w, x, y, z] format.
        t: Translation tensor of shape (N, 3).

    Returns:
        Tuple of (inverse_quaternion, inverse_translation) tensors.
    """
    q_inv = quat_conjugate(q)
    return q_inv, -quat_apply(q_inv, t)


@torch.jit.script
def tf_apply(q: torch.Tensor, t: torch.Tensor, v: torch.Tensor) -> torch.Tensor:
    """Apply a rigid transform (quaternion + translation) to a vector.

    Args:
        q: Rotation quaternion tensor of shape (N, 4) in [w, x, y, z] format.
        t: Translation tensor of shape (N, 3).
        v: Vector tensor of shape (N, 3) to transform.

    Returns:
        Transformed vector tensor of shape (N, 3).
    """
    return quat_apply(q, v) + t


@torch.jit.script
def tf_vector(q: torch.Tensor, v: torch.Tensor) -> torch.Tensor:
    """Rotate a vector by a quaternion without translation.

    Args:
        q: Rotation quaternion tensor of shape (N, 4) in [w, x, y, z] format.
        v: Vector tensor of shape (N, 3) to rotate.

    Returns:
        Rotated vector tensor of shape (N, 3).
    """
    return quat_apply(q, v)


@torch.jit.script
def tf_combine(
    q1: torch.Tensor, t1: torch.Tensor, q2: torch.Tensor, t2: torch.Tensor
) -> tuple[torch.Tensor, torch.Tensor]:
    """Combine two rigid transforms into a single transform.

    Args:
        q1: First rotation quaternion tensor of shape (N, 4) in [w, x, y, z] format.
        t1: First translation tensor of shape (N, 3).
        q2: Second rotation quaternion tensor of shape (N, 4) in [w, x, y, z] format.
        t2: Second translation tensor of shape (N, 3).

    Returns:
        Tuple of (combined_quaternion, combined_translation) tensors.
    """
    return quat_mul(q1, q2), quat_apply(q1, t2) + t1


def assign_pose(
    current_positions: object,
    current_orientations: object,
    positions: object,
    orientations: object,
    indices: object,
    device: object,
    pose: object = None,
) -> object:
    """Assigns new pose values to specific indices in current pose arrays.

    Args:
        current_positions: Current position values for all objects.
        current_orientations: Current orientation values for all objects.
        positions: New position values to assign. If None, uses current positions at indices.
        orientations: New orientation values to assign. If None, uses current orientations at indices.
        indices: Array indices where new pose values should be assigned.
        device: Device for tensor operations.
        pose: Unused parameter for compatibility.

    Returns:
        Updated pose array with new values assigned at specified indices.
    """
    if positions is None:
        positions = current_positions[indices]
    if orientations is None:
        orientations = current_orientations[indices]
    orientations = wxyz2xyzw(orientations)
    current_orientations = wxyz2xyzw(current_orientations)
    old_pose = get_pose(current_positions, current_orientations, device=current_positions.device)
    new_pose = get_pose(positions, orientations, device=current_positions.device)
    old_pose[indices] = new_pose
    return old_pose
