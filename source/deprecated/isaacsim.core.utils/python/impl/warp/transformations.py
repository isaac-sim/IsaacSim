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

"""Warp-based coordinate transformation utilities for pose manipulation and coordinate space conversions."""

from typing import Any

import numpy as np
import warp as wp
from isaacsim.core.deprecation_manager import import_module
from isaacsim.core.utils.torch.rotations import gf_quat_to_tensor as torch_gf_quat_to_tensor
from isaacsim.core.utils.torch.transformations import tf_matrices_from_poses as torch_tf_matrices_from_poses
from pxr import Gf

torch = import_module("torch")


@wp.kernel
def _local_to_world(
    parent_translations: wp.array(dtype=float, ndim=2),
    parent_rotations: wp.array(dtype=float, ndim=2),
    positions: Any,
    orientations: Any,
    world_pos: wp.array(dtype=float, ndim=2),
    world_rot: wp.array(dtype=float, ndim=2),
) -> None:
    """Warp kernel that transforms local poses to world coordinates.

    Applies parent transformations to convert local positions and orientations into world space.
    Uses quaternion rotation and translation to compute the world-space pose.

    Args:
        parent_translations: Parent translation vectors in world coordinates.
        parent_rotations: Parent rotation quaternions in [qx, qy, qz, qw] format.
        positions: Local position data to transform.
        orientations: Local orientation data in [qw, qx, qy, qz] format.
        world_pos: Output array for world-space positions.
        world_rot: Output array for world-space rotations in [qw, qx, qy, qz] format.
    """
    tid = wp.tid()
    parent_rot = wp.quat(
        parent_rotations[tid, 0], parent_rotations[tid, 1], parent_rotations[tid, 2], parent_rotations[tid, 3]
    )
    parent_trans = wp.vec3(parent_translations[tid, 0], parent_translations[tid, 1], parent_translations[tid, 2])
    local_rot = wp.quat(orientations[tid, 1], orientations[tid, 2], orientations[tid, 3], orientations[tid, 0])
    local_pos = wp.vec3(positions[tid, 0], positions[tid, 1], positions[tid, 2])
    pos = quat_rotate(parent_rot, local_pos) + parent_trans  # noqa: F821
    world_pos[tid, 0] = pos[0]
    world_pos[tid, 1] = pos[1]
    world_pos[tid, 2] = pos[2]
    rot = parent_rot * local_rot
    world_rot[tid, 0] = rot[3]
    world_rot[tid, 1] = rot[0]
    world_rot[tid, 2] = rot[1]
    world_rot[tid, 3] = rot[2]


wp.overload(
    _local_to_world, {"positions": wp.array(dtype=float, ndim=2), "orientations": wp.array(dtype=float, ndim=2)}
)
wp.overload(
    _local_to_world,
    {"positions": wp.indexedarray(dtype=float, ndim=2), "orientations": wp.indexedarray(dtype=float, ndim=2)},
)


def get_local_from_world(parent_transforms: object, positions: object, orientations: object, device: object) -> tuple:
    """Converts world-space poses to local coordinates relative to parent transforms.

    Transforms world positions and orientations to local space using parent transformation matrices.
    Temporarily moves computation to CUDA for Warp kernel execution.

    Args:
        parent_transforms: Parent transformation matrices for coordinate conversion.
        positions: World-space position data to transform.
        orientations: World-space orientation data to transform.
        device: Target device for the returned data.

    Returns:
        Tuple of (local_positions, local_orientations) in parent coordinate space.
    """
    # TODO: warp kernels not working on cpu
    ret_device = device
    positions = positions.to(device="cuda:0")
    orientations = orientations.to(device="cuda:0")
    parent_rotations = []
    parent_translations = []
    parent_transforms = parent_transforms.numpy()
    for i in range(len(parent_transforms)):
        parent_rotations.append(
            np.array(
                [
                    *Gf.Matrix4d(parent_transforms[i].tolist()).ExtractRotation().GetQuat().GetImaginary(),
                    Gf.Matrix4d(parent_transforms[i].tolist()).ExtractRotation().GetQuat().GetReal(),
                ]
            )
        )
        parent_translations.append(np.array([*Gf.Matrix4d(parent_transforms[i].tolist()).ExtractTranslation()]))

    world_pos = wp.zeros(shape=(positions.shape[0], 3), dtype=wp.float32, device="cuda:0")
    world_rot = wp.zeros(shape=(orientations.shape[0], 4), dtype=wp.float32, device="cuda:0")
    parent_translations = wp.from_numpy(np.array(parent_translations), dtype=wp.float32, device="cuda:0")
    parent_rotations = wp.from_numpy(np.array(parent_rotations), dtype=wp.float32, device="cuda:0")
    wp.launch(
        _local_to_world,
        dim=positions.shape[0],
        inputs=[parent_translations, parent_rotations, positions, orientations, world_pos, world_rot],
        device=positions.device,
    )

    world_pos = world_pos.to(device=ret_device)
    world_rot = world_rot.to(device=ret_device)

    return world_pos, world_rot


def get_world_from_local(
    parent_transforms: object, translations: object, orientations: object, device: object
) -> tuple:
    """Transforms local poses to world coordinates using parent transformations.

    Converts local translations and orientations to world space by applying parent transformation
    matrices. Supports both Warp arrays and PyTorch tensors as input.

    Args:
        parent_transforms: Parent transformation matrices for coordinate conversion.
        translations: Local translation data to transform.
        orientations: Local orientation data to transform.
        device: Target device for computation and returned data.

    Returns:
        Tuple of (world_translations, world_orientations) as Warp arrays.
    """
    calculated_translations = torch.zeros(size=(translations.shape[0], 3), dtype=torch.float32, device=device)
    calculated_orientations = torch.zeros(size=(translations.shape[0], 4), dtype=torch.float32, device=device)

    if isinstance(parent_transforms, wp.array):
        parent_torch = wp.to_torch(parent_transforms)
    else:
        parent_torch = parent_transforms
    if isinstance(translations, wp.array):
        translations_torch = wp.to_torch(translations)
    else:
        translations_torch = translations
    if isinstance(orientations, wp.array):
        orientations_torch = wp.to_torch(orientations)
    else:
        orientations_torch = orientations

    my_local_transforms = torch_tf_matrices_from_poses(
        translations=translations_torch, orientations=orientations_torch, device=device
    )
    # TODO: vectorize this
    for i in range(translations.shape[0]):
        world_transform = torch.matmul(torch.transpose(parent_torch[i], 0, 1), my_local_transforms[i])
        transform = Gf.Transform()
        transform.SetMatrix(Gf.Matrix4d(torch.transpose(world_transform, 0, 1).tolist()))
        calculated_translations[i] = torch.tensor(transform.GetTranslation(), dtype=torch.float32, device=device)
        calculated_orientations[i] = torch_gf_quat_to_tensor(transform.GetRotation().GetQuat())

    translations_wp = wp.from_torch(calculated_translations)
    orientations_wp = wp.from_torch(calculated_orientations)
    return translations_wp, orientations_wp


@wp.kernel
def _assign_pose(pose: wp.array(dtype=float, ndim=2), positions: Any, orientations: Any) -> None:
    """Warp kernel that assigns position and orientation data to a pose array.

    Combines position and orientation arrays into a unified pose format.
    The pose array stores data as [x, y, z, qw, qx, qy, qz] per row.

    Args:
        pose: Output pose array to populate.
        positions: Position data array with [x, y, z] coordinates.
        orientations: Orientation data array with [qw, qx, qy, qz] quaternion values.
    """
    i = wp.tid()
    pose[i, 0] = positions[i, 0]
    pose[i, 1] = positions[i, 1]
    pose[i, 2] = positions[i, 2]
    pose[i, 3] = orientations[i, 0]
    pose[i, 4] = orientations[i, 1]
    pose[i, 5] = orientations[i, 2]
    pose[i, 6] = orientations[i, 3]


wp.overload(_assign_pose, {"positions": wp.array(dtype=float, ndim=2), "orientations": wp.array(dtype=float, ndim=2)})
wp.overload(
    _assign_pose,
    {"positions": wp.indexedarray(dtype=float, ndim=2), "orientations": wp.indexedarray(dtype=float, ndim=2)},
)


def get_pose(positions: object, orientations: object, device: object) -> wp.array:
    """Combines position and orientation arrays into a unified pose representation.

    Creates a pose array containing both position and orientation data in a single structure.
    Temporarily moves computation to CUDA for Warp kernel execution.

    Args:
        positions: Position data array.
        orientations: Orientation data array.
        device: Target device for the returned pose array.

    Returns:
        Combined pose array with position and orientation data.
    """
    # TODO: warp kernels not working on cpu
    device = positions.device
    positions = positions.to("cuda:0")
    orientations = orientations.to("cuda:0")
    pose = wp.zeros((positions.shape[0], 7), dtype=wp.float32, device=positions.device)
    wp.launch(_assign_pose, dim=positions.shape[0], inputs=[pose, positions, orientations], device=pose.device)
    pose = pose.to(device)
    return pose


@wp.kernel
def _assign_current_pose(
    pose: wp.array(dtype=wp.float32, ndim=2), current_positions: Any, current_orientations: Any
) -> None:
    """Warp kernel that assigns current pose values to a pose array.

    Copies position and orientation data from current arrays into the pose array format.
    The pose array stores data as [x, y, z, qx, qy, qz, qw] per row.

    Args:
        pose: Output pose array to populate.
        current_positions: Current position data array.
        current_orientations: Current orientation data array in [qw, qx, qy, qz] format.
    """
    i = wp.tid()
    pose[i, 0] = current_positions[i, 0]
    pose[i, 1] = current_positions[i, 1]
    pose[i, 2] = current_positions[i, 2]
    pose[i, 3] = current_orientations[i, 1]
    pose[i, 4] = current_orientations[i, 2]
    pose[i, 5] = current_orientations[i, 3]
    pose[i, 6] = current_orientations[i, 0]


wp.overload(
    _assign_current_pose,
    {
        "current_positions": wp.indexedarray(dtype=wp.float32, ndim=2),
        "current_orientations": wp.indexedarray(dtype=wp.float32, ndim=2),
    },
)
wp.overload(
    _assign_current_pose,
    {
        "current_positions": wp.array(dtype=wp.float32, ndim=2),
        "current_orientations": wp.array(dtype=wp.float32, ndim=2),
    },
)


@wp.kernel
def _assign_new_pose(
    pose: wp.array(dtype=wp.float32, ndim=2),
    positions: Any,
    orientations: Any,
    indices: wp.array(dtype=wp.int32),
    has_positions: int,
    has_orientations: int,
) -> None:
    """Warp kernel that selectively assigns new pose values to specific indices in a pose array.

    Updates pose data at specified indices based on availability flags.
    Only updates positions if has_positions is 1, and orientations if has_orientations is 1.

    Args:
        pose: Pose array to update.
        positions: New position data to assign.
        orientations: New orientation data to assign in [qw, qx, qy, qz] format.
        indices: Target indices in the pose array to update.
        has_positions: Flag indicating whether to update positions (1) or not (0).
        has_orientations: Flag indicating whether to update orientations (1) or not (0).
    """
    i = wp.tid()
    idx = indices[i]
    if has_positions == 1:
        pose[idx, 0] = positions[i, 0]
        pose[idx, 1] = positions[i, 1]
        pose[idx, 2] = positions[i, 2]
    if has_orientations == 1:
        pose[idx, 3] = orientations[i, 1]
        pose[idx, 4] = orientations[i, 2]
        pose[idx, 5] = orientations[i, 3]
        pose[idx, 6] = orientations[i, 0]


wp.overload(
    _assign_new_pose,
    {"positions": wp.indexedarray(dtype=wp.float32, ndim=2), "orientations": wp.indexedarray(dtype=wp.float32, ndim=2)},
)
wp.overload(
    _assign_new_pose,
    {"positions": wp.array(dtype=wp.float32, ndim=2), "orientations": wp.array(dtype=wp.float32, ndim=2)},
)


def assign_pose(
    current_positions: object,
    current_orientations: object,
    positions: object,
    orientations: object,
    indices: object,
    device: object,
    pose: object,
) -> wp.array:
    """Assigns pose data by combining current poses with selective updates.

    First populates the pose array with current position and orientation data, then selectively
    updates specific indices with new pose values. Handles device management for CPU/CUDA operations.

    Args:
        current_positions: Current position data for all poses.
        current_orientations: Current orientation data for all poses.
        positions: New position data to assign at specific indices.
        orientations: New orientation data to assign at specific indices.
        indices: Target indices for applying new pose data.
        device: Target device for computation ("cpu" or "cuda:0").
        pose: Pose array to populate and update.

    Returns:
        Updated pose array with combined current and new pose data.
    """
    to_cpu = False
    if device == "cpu":
        to_cpu = True
        current_positions = current_positions.to(device="cuda:0")
        current_orientations = current_orientations.to(device="cuda:0")
        positions = positions.to(device="cuda:0")
        orientations = orientations.to(device="cuda:0")
        indices = indices.to(device="cuda:0")
        pose = pose.to(device="cuda:0")
    wp.launch(
        _assign_current_pose, dim=pose.shape[0], inputs=[pose, current_positions, current_orientations], device="cuda:0"
    )
    wp.launch(
        _assign_new_pose,
        dim=positions.shape[0],
        inputs=[
            pose,
            positions,
            orientations,
            indices,
            0 if positions is None else 1,
            0 if orientations is None else 1,
        ],
        device="cuda:0",
    )

    if to_cpu:
        pose = pose.to(device="cpu")

    return pose
