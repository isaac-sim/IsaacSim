# SPDX-FileCopyrightText: Copyright (c) 2024-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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

"""Provide Warp kernels for Newton tensor operations.

These kernels handle efficient get/set operations for articulation properties
using Warp's GPU acceleration.
"""

from __future__ import annotations

from typing import Any

import newton
import numpy as np
import warp as wp

vec10 = wp.types.vector(length=10, dtype=float)


@wp.kernel(enable_backward=False)
def fill_contiguous_indices(indices: wp.array(dtype=wp.int32)) -> None:
    """Fill an array with its contiguous row indices.

    Args:
        indices: Output array to populate.

    """
    row = wp.tid()
    indices[row] = row


@wp.kernel(enable_backward=False)
def sanitize_indices(
    indices: wp.array(dtype=wp.int32),
    count: int,
    safe_indices: wp.array(dtype=wp.int32),
    active_mask: wp.array(dtype=int),
) -> None:
    """Split a selection into a dereference-safe index and an active flag.

    Index values live in device memory, so an out-of-range value cannot be rejected
    without a readback. Instead it is clamped to slot zero -- which makes every
    downstream dereference in-bounds, including the arithmetic ones like
    ``index[arti_id * max_dofs + tj]`` -- and flagged inactive, so the operation that
    consumes it writes nothing for that entity.

    Args:
        indices: Caller selection, unvalidated.
        count: Number of addressable entities.
        safe_indices: Output selection with every value inside ``[0, count)``.
        active_mask: Output flag, zero where the caller's value was out of range.

    """
    row = wp.tid()
    value = indices[row]
    if value >= 0 and value < count:
        safe_indices[row] = value
        active_mask[row] = 1
    else:
        safe_indices[row] = 0
        active_mask[row] = 0


@wp.kernel(enable_backward=False)
def gather_float_rows(
    source: wp.array2d(dtype=wp.float32),
    indices: wp.array(dtype=wp.int32),
    dropped: wp.float32,
    destination: wp.array2d(dtype=wp.float32),
) -> None:
    """Gather selected source rows into a dense destination.

    An out-of-range index yields the dropped-row marker. Index values are never read
    back to be rejected, so the bound is enforced here. Writing the marker from this
    kernel rather than pre-filling the destination keeps the read to a single pass:
    every element is written exactly once, and the common all-valid selection does
    not pay for a buffer-wide fill it immediately overwrites.

    Args:
        source: Source matrix.
        indices: Source row index for each destination row.
        dropped: Value marking a row no valid index produced.
        destination: Dense destination matrix.

    """
    row, column = wp.tid()
    index = indices[row]
    if index >= 0 and index < source.shape[0]:
        destination[row, column] = source[index, column]
    else:
        destination[row, column] = dropped


@wp.kernel(enable_backward=False)
def scatter_float_rows(
    source: wp.array2d(dtype=wp.float32),
    indices: wp.array(dtype=wp.int32),
    active_mask: wp.array(dtype=int),
    destination: wp.array2d(dtype=wp.float32),
) -> None:
    """Scatter dense source rows into selected destination rows.

    Inactive rows -- those whose caller index was out of range -- write nothing. The
    mask is required rather than relying on the bound alone: a dropped index arrives
    clamped, so without it a dropped row and a genuine write to the clamp slot would
    race for the same destination.

    Args:
        source: Dense source matrix.
        indices: Destination row index for each source row, already in bounds.
        active_mask: Zero for rows whose caller index was out of range.
        destination: Destination matrix to update.

    """
    row, column = wp.tid()
    if active_mask[row] == 0:
        return
    index = indices[row]
    if index >= 0 and index < destination.shape[0]:
        destination[index, column] = source[row, column]


@wp.kernel(enable_backward=False)
def scatter_float_matrix(
    source: wp.array3d(dtype=wp.float32),
    indices: wp.array(dtype=wp.int32),
    active_mask: wp.array(dtype=int),
    destination: wp.array3d(dtype=wp.float32),
) -> None:
    """Scatter dense source matrices into selected destination entries.

    Inactive rows -- those whose caller index was out of range -- write nothing. The
    mask is required rather than relying on the bound alone: a dropped index arrives
    clamped, so without it a dropped row and a genuine write to the clamp slot would
    race for the same destination.

    Args:
        source: Dense source matrix batch.
        indices: Destination batch index for each source matrix, already in bounds.
        active_mask: Zero for rows whose caller index was out of range.
        destination: Destination matrix batch to update.

    """
    row, r, c = wp.tid()
    if active_mask[row] == 0:
        return
    index = indices[row]
    if index >= 0 and index < destination.shape[0]:
        destination[index, r, c] = source[row, r, c]


@wp.kernel(enable_backward=False)
def get_body_pose(
    body_q: wp.array(dtype=wp.transform),
    index: wp.array(dtype=int),
    # outputs
    tensor: wp.array2d(dtype=float),
) -> None:
    """Get body poses as position and quaternion.

    Output shape is (count, 7): [x, y, z, qx, qy, qz, qw].

    Args:
        body_q: Body transforms.
        index: Body indices to read.
        tensor: Output tensor (count, 7).

    """
    tid = wp.tid()
    wid = index[tid]
    X_ws = body_q[wid]
    tensor[tid, 0] = X_ws[0]
    tensor[tid, 1] = X_ws[1]
    tensor[tid, 2] = X_ws[2]
    tensor[tid, 3] = X_ws[3]
    tensor[tid, 4] = X_ws[4]
    tensor[tid, 5] = X_ws[5]
    tensor[tid, 6] = X_ws[6]


@wp.kernel(enable_backward=False)
def get_body_velocity(
    body_qd: wp.array(dtype=wp.spatial_vector),
    index: wp.array(dtype=int),
    # outputs
    tensor: wp.array2d(dtype=float),
) -> None:
    """Get body velocities as linear and angular components.

    Output shape is (count, 6): [vx, vy, vz, wx, wy, wz].

    Args:
        body_qd: Body spatial velocities.
        index: Body indices to read.
        tensor: Output tensor (count, 6).

    """
    tid = wp.tid()
    wid = index[tid]
    spatial_vel = body_qd[wid]
    linear_vel = wp.spatial_top(spatial_vel)
    angular_vel = wp.spatial_bottom(spatial_vel)
    tensor[tid, 0] = linear_vel[0]
    tensor[tid, 1] = linear_vel[1]
    tensor[tid, 2] = linear_vel[2]
    tensor[tid, 3] = angular_vel[0]
    tensor[tid, 4] = angular_vel[1]
    tensor[tid, 5] = angular_vel[2]


@wp.kernel(enable_backward=False)
def get_body_mass(
    body_mass: wp.array(dtype=wp.float32),
    index: wp.array(dtype=int),
    # outputs
    tensor: wp.array2d(dtype=float),
) -> None:
    """Get body masses.

    Output shape is (count, 1).

    Args:
        body_mass: Body masses array.
        index: Body indices to read.
        tensor: Output tensor (count, 1).

    """
    tid = wp.tid()
    wid = index[tid]
    tensor[tid, 0] = body_mass[wid]


@wp.kernel(enable_backward=False)
def get_body_inv_mass(
    body_inv_mass: wp.array(dtype=wp.float32),
    index: wp.array(dtype=int),
    # outputs
    tensor: wp.array2d(dtype=float),
) -> None:
    """Get body inverse masses.

    Output shape is (count, 1).

    Args:
        body_inv_mass: Body inverse masses array.
        index: Body indices to read.
        tensor: Output tensor (count, 1).

    """
    tid = wp.tid()
    wid = index[tid]
    tensor[tid, 0] = body_inv_mass[wid]


@wp.kernel(enable_backward=False)
def get_body_com(
    body_com: wp.array(dtype=wp.vec3),
    index: wp.array(dtype=int),
    # outputs
    tensor: wp.array2d(dtype=float),
) -> None:
    """Get COM position and orientation for rigid bodies.

    Output shape is (count, 7): [com_x, com_y, com_z, qx, qy, qz, qw].
    Newton stores the COM as a body-frame offset, so the output orientation is identity.

    Args:
        body_com: Body center of mass positions.
        index: Body indices to read.
        tensor: Output tensor (count, 7).

    """
    tid = wp.tid()
    wid = index[tid]
    com = body_com[wid]
    tensor[tid, 0] = com[0]
    tensor[tid, 1] = com[1]
    tensor[tid, 2] = com[2]
    # Identity quaternion in [qx, qy, qz, qw] order.
    tensor[tid, 3] = 0.0  # qx
    tensor[tid, 4] = 0.0  # qy
    tensor[tid, 5] = 0.0  # qz
    tensor[tid, 6] = 1.0  # qw


@wp.kernel(enable_backward=False)
def get_body_com_position_only(
    body_com: wp.array(dtype=wp.vec3),
    index: wp.array(dtype=int),
    # outputs
    tensor: wp.array2d(dtype=float),
) -> None:
    """Get only COM position, leaving orientation untouched in output tensor.

    Updates only first 3 elements of tensor: [com_x, com_y, com_z]
    Elements 3-6 (orientation) are left unchanged.

    Args:
        body_com: Body center of mass positions.
        index: Body indices to read.
        tensor: Output tensor (count, 7).

    """
    tid = wp.tid()
    wid = index[tid]
    com = body_com[wid]
    tensor[tid, 0] = com[0]
    tensor[tid, 1] = com[1]
    tensor[tid, 2] = com[2]
    # Orientation (tensor[tid, 3:7]) is not modified


@wp.kernel(enable_backward=False)
def cache_body_com(
    tensor: wp.array2d(dtype=float),
    tensor_idx: wp.array(dtype=wp.int32),
    tensor_idx_mask: wp.array(dtype=int),
    # outputs
    com_cache: wp.array2d(dtype=float),
) -> None:
    """Cache full COM data (position + orientation) for later retrieval.

    Since Newton only stores position, we cache the full 7-element COM
    (position + orientation) in a separate buffer.

    Args:
        tensor: Input COM data (count, 7).
        tensor_idx: Indices into tensor.
        tensor_idx_mask: Optional mask for indices.
        com_cache: Output COM cache.

    """
    tid = wp.tid()
    body_id = tensor_idx[tid]
    if tensor_idx_mask:
        apply_data = bool(tensor_idx_mask[tid])
    else:
        apply_data = True

    if apply_data:
        # Cache at the view-local row (body_id), matching get_body_com_position_only
        # which writes tensor[tid]. Indexing by the model-global body_idx would
        # overflow the view-sized cache for a subset or reordered view.
        for i in range(7):
            com_cache[body_id, i] = tensor[body_id, i]


@wp.kernel(enable_backward=False)
def get_body_inertia(
    body_inertia: wp.array(dtype=wp.mat33),
    index: wp.array(dtype=int),
    # outputs
    tensor: wp.array2d(dtype=float),
) -> None:
    """Get inertia tensor as flattened 3x3 matrix.

    Output shape is (count, 9) in row-major order.

    Args:
        body_inertia: Body inertia matrices.
        index: Body indices to read.
        tensor: Output tensor (count, 9).

    """
    tid = wp.tid()
    wid = index[tid]
    inertia = body_inertia[wid]
    # Flatten 3x3 matrix in row-major order
    tensor[tid, 0] = inertia[0, 0]
    tensor[tid, 1] = inertia[0, 1]
    tensor[tid, 2] = inertia[0, 2]
    tensor[tid, 3] = inertia[1, 0]
    tensor[tid, 4] = inertia[1, 1]
    tensor[tid, 5] = inertia[1, 2]
    tensor[tid, 6] = inertia[2, 0]
    tensor[tid, 7] = inertia[2, 1]
    tensor[tid, 8] = inertia[2, 2]


@wp.kernel(enable_backward=False)
def get_body_inv_inertia(
    body_inv_inertia: wp.array(dtype=wp.mat33),
    index: wp.array(dtype=int),
    # outputs
    tensor: wp.array2d(dtype=float),
) -> None:
    """Get inverse inertia tensor as flattened 3x3 matrix.

    Output shape is (count, 9) in row-major order.

    Args:
        body_inv_inertia: Body inverse inertia matrices.
        index: Body indices to read.
        tensor: Output tensor (count, 9).

    """
    tid = wp.tid()
    wid = index[tid]
    I_inv = body_inv_inertia[wid]
    # Flatten 3x3 matrix in row-major order
    tensor[tid, 0] = I_inv[0, 0]
    tensor[tid, 1] = I_inv[0, 1]
    tensor[tid, 2] = I_inv[0, 2]
    tensor[tid, 3] = I_inv[1, 0]
    tensor[tid, 4] = I_inv[1, 1]
    tensor[tid, 5] = I_inv[1, 2]
    tensor[tid, 6] = I_inv[2, 0]
    tensor[tid, 7] = I_inv[2, 1]
    tensor[tid, 8] = I_inv[2, 2]


@wp.kernel(enable_backward=False)
def get_link_inv_mass(
    body_mass: wp.array(dtype=wp.float32),
    index: wp.array2d(dtype=int),
    # outputs
    tensor: wp.array2d(dtype=float),
) -> None:
    """Get link inverse masses for articulations.

    Output shape is (count, max_links). Computes 1/mass for each link.

    Args:
        body_mass: Body masses array.
        index: Link indices (count, max_links).
        tensor: Output tensor (count, max_links).

    """
    ti, tj = wp.tid()
    wid = index[ti, tj]
    if wid >= 0:
        mass = body_mass[wid]
        # Compute inverse mass: 1/mass, but handle zero mass case
        if mass > 0.0:
            tensor[ti, tj] = 1.0 / mass
        else:
            tensor[ti, tj] = 0.0  # Zero mass -> zero inverse mass (infinite mass)
    else:
        tensor[ti, tj] = 0.0  # Invalid index -> zero inverse mass


@wp.kernel(enable_backward=False)
def get_link_mass(
    body_mass: wp.array(dtype=wp.float32),
    index: wp.array2d(dtype=int),
    # outputs
    tensor: wp.array2d(dtype=float),
) -> None:
    """Get link masses for articulations.

    Output shape is (count, max_links).

    Args:
        body_mass: Body masses array.
        index: Link indices (count, max_links).
        tensor: Output tensor (count, max_links).

    """
    ti, tj = wp.tid()
    wid = index[ti, tj]
    if wid >= 0:
        tensor[ti, tj] = body_mass[wid]


@wp.kernel(enable_backward=False)
def set_body_mass(
    tensor: wp.array2d(dtype=float),
    tensor_idx: wp.array(dtype=wp.int32),
    tensor_idx_mask: wp.array(dtype=int),
    body_idx: wp.array(dtype=int),
    # outputs
    body_mass: wp.array(dtype=wp.float32),
) -> None:
    """Set body masses from input tensor.

    Input shape is (count, 1).

    Args:
        tensor: Input mass values (count, 1).
        tensor_idx: Indices into tensor.
        tensor_idx_mask: Optional mask for indices.
        body_idx: Body indices.
        body_mass: Output body masses.

    """
    tid = wp.tid()
    body_id = tensor_idx[tid]
    wid = body_idx[body_id]
    # if a mask array is provided then only apply the data for the indices that are True
    if tensor_idx_mask:
        apply_data = bool(tensor_idx_mask[tid])
    else:
        apply_data = True

    if apply_data:
        body_mass[wid] = tensor[body_id, 0]


@wp.kernel(enable_backward=False)
def update_body_inv_mass(
    tensor_idx: wp.array(dtype=wp.int32),
    tensor_idx_mask: wp.array(dtype=int),
    body_idx: wp.array(dtype=int),
    body_mass: wp.array(dtype=wp.float32),
    # outputs
    body_inv_mass: wp.array(dtype=wp.float32),
) -> None:
    """Update inverse mass from mass for rigid bodies.

    Args:
        tensor_idx: Indices into tensor.
        tensor_idx_mask: Optional mask for indices.
        body_idx: Body indices.
        body_mass: Body masses.
        body_inv_mass: Output inverse masses.

    """
    tid = wp.tid()
    body_id = tensor_idx[tid]
    wid = body_idx[body_id]
    if tensor_idx_mask:
        apply_data = bool(tensor_idx_mask[tid])
    else:
        apply_data = True

    if apply_data:
        mass = body_mass[wid]
        if mass > 1e-8:
            body_inv_mass[wid] = 1.0 / mass
        else:
            body_inv_mass[wid] = 0.0


@wp.kernel(enable_backward=False)
def set_body_com(
    tensor: wp.array2d(dtype=float),
    tensor_idx: wp.array(dtype=wp.int32),
    tensor_idx_mask: wp.array(dtype=int),
    body_idx: wp.array(dtype=int),
    # outputs
    body_com: wp.array(dtype=wp.vec3),
) -> None:
    """Set COM position for rigid bodies.

    Input shape (count, 7): [com_x, com_y, com_z, qx, qy, qz, qw]
    Only position (first 3 values) is used; Newton stores COM as offset, not with orientation.

    Args:
        tensor: Input COM data (count, 7).
        tensor_idx: Indices into tensor.
        tensor_idx_mask: Optional mask for indices.
        body_idx: Body indices.
        body_com: Output COM positions.

    """
    tid = wp.tid()
    body_id = tensor_idx[tid]
    wid = body_idx[body_id]
    if tensor_idx_mask:
        apply_data = bool(tensor_idx_mask[tid])
    else:
        apply_data = True

    if apply_data:
        body_com[wid] = wp.vec3(tensor[body_id, 0], tensor[body_id, 1], tensor[body_id, 2])
        # Orientation (tensor[body_id, 3:7]) is ignored - Newton inertia is already in body frame


@wp.kernel(enable_backward=False)
def set_body_inertia(
    tensor: wp.array2d(dtype=float),
    tensor_idx: wp.array(dtype=wp.int32),
    tensor_idx_mask: wp.array(dtype=int),
    body_idx: wp.array(dtype=int),
    # outputs
    body_inertia: wp.array(dtype=wp.mat33),
) -> None:
    """Set inertia tensor from flattened 3x3 matrix.

    Input shape (count, 9) in row-major order.

    Args:
        tensor: Input inertia values (count, 9).
        tensor_idx: Indices into tensor.
        tensor_idx_mask: Optional mask for indices.
        body_idx: Body indices.
        body_inertia: Output inertia matrices.

    """
    tid = wp.tid()
    body_id = tensor_idx[tid]
    wid = body_idx[body_id]
    if tensor_idx_mask:
        apply_data = bool(tensor_idx_mask[tid])
    else:
        apply_data = True

    if apply_data:
        # Reconstruct 3x3 matrix from flattened row-major array
        body_inertia[wid] = wp.mat33(
            tensor[body_id, 0],
            tensor[body_id, 1],
            tensor[body_id, 2],
            tensor[body_id, 3],
            tensor[body_id, 4],
            tensor[body_id, 5],
            tensor[body_id, 6],
            tensor[body_id, 7],
            tensor[body_id, 8],
        )


@wp.kernel(enable_backward=False)
def update_body_inv_inertia(
    tensor_idx: wp.array(dtype=wp.int32),
    tensor_idx_mask: wp.array(dtype=int),
    body_idx: wp.array(dtype=int),
    body_inertia: wp.array(dtype=wp.mat33),
    # outputs
    body_inv_inertia: wp.array(dtype=wp.mat33),
) -> None:
    """Update inverse inertia from inertia tensor for rigid bodies.

    Args:
        tensor_idx: Indices into tensor.
        tensor_idx_mask: Optional mask for indices.
        body_idx: Body indices.
        body_inertia: Body inertia matrices.
        body_inv_inertia: Output inverse inertia matrices.

    """
    tid = wp.tid()
    body_id = tensor_idx[tid]
    wid = body_idx[body_id]
    if tensor_idx_mask:
        apply_data = bool(tensor_idx_mask[tid])
    else:
        apply_data = True

    if apply_data:
        inertia = body_inertia[wid]
        det = wp.determinant(inertia)
        if wp.abs(det) > 1e-8:
            body_inv_inertia[wid] = wp.inverse(inertia)
        else:
            body_inv_inertia[wid] = wp.mat33(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0)


@wp.kernel(enable_backward=False)
def get_link_inertia(
    body_inertia: wp.array(dtype=wp.mat33),
    index: wp.array2d(dtype=int),
    # outputs
    tensor: wp.array3d(dtype=float),
) -> None:
    """Get link inertia tensors for articulations.

    Output shape is (count, max_links, 9) in row-major order.

    Args:
        body_inertia: Body inertia matrices.
        index: Link indices (count, max_links).
        tensor: Output tensor (count, max_links, 9).

    """
    ti, tj, tk = wp.tid()
    wid = index[ti, tj]
    if wid >= 0:
        i = tk // 3
        j = tk % 3
        tensor[ti, tj, tk] = body_inertia[wid][i][j]


@wp.kernel(enable_backward=False)
def get_link_inv_inertia(
    body_inv_inertia: wp.array(dtype=wp.mat33),
    index: wp.array2d(dtype=int),
    # outputs
    tensor: wp.array3d(dtype=float),
) -> None:
    """Get link inverse inertia tensors for articulations.

    Output shape is (count, max_links, 9) in row-major order.

    Args:
        body_inv_inertia: Body inverse inertia matrices.
        index: Link indices (count, max_links).
        tensor: Output tensor (count, max_links, 9).

    """
    ti, tj, tk = wp.tid()
    wid = index[ti, tj]
    if wid >= 0:
        i = tk // 3
        j = tk % 3
        tensor[ti, tj, tk] = body_inv_inertia[wid][i][j]


@wp.kernel(enable_backward=False)
def get_link_com(
    body_com: wp.array(dtype=wp.vec3),
    index: wp.array2d(dtype=int),
    # outputs
    tensor: wp.array3d(dtype=float),
) -> None:
    """Get link COM positions and orientations.

    Output tensor has shape (count, max_links, 7) where each entry is:
    [com_x, com_y, com_z, qw, qx, qy, qz] (scalar-first quaternion)

    Newton's ``body_com`` stores body-frame COM offsets. The output uses identity
    orientations [1, 0, 0, 0].

    Args:
        body_com: Body-frame center-of-mass offsets.
        index: Link indices (count, max_links).
        tensor: Output tensor (count, max_links, 7).

    """
    ti, tj, tk = wp.tid()
    wid = index[ti, tj]
    if wid >= 0:
        com = body_com[wid]
        if tk == 0:
            tensor[ti, tj, tk] = com[0]  # x
        elif tk == 1:
            tensor[ti, tj, tk] = com[1]  # y
        elif tk == 2:
            tensor[ti, tj, tk] = com[2]  # z
        elif tk == 3:
            tensor[ti, tj, tk] = 1.0  # qw (identity)
        else:
            tensor[ti, tj, tk] = 0.0  # qx, qy, qz (identity)


@wp.kernel(enable_backward=False)
def get_link_com_position_only(
    body_com: wp.array(dtype=wp.vec3),
    index: wp.array2d(dtype=int),
    # outputs
    tensor: wp.array3d(dtype=float),
) -> None:
    """Get only link COM positions, leaving orientation untouched in output tensor.

    Updates only first 3 elements of tensor: [com_x, com_y, com_z]
    Elements 3-6 (orientation) are left unchanged.

    Args:
        body_com: Body center of mass positions.
        index: Link indices (count, max_links).
        tensor: Output tensor (count, max_links, 7).

    """
    ti, tj, tk = wp.tid()
    wid = index[ti, tj]
    if wid >= 0 and tk < 3:
        com = body_com[wid]
        if tk == 0:
            tensor[ti, tj, tk] = com[0]
        elif tk == 1:
            tensor[ti, tj, tk] = com[1]
        elif tk == 2:
            tensor[ti, tj, tk] = com[2]
    # Orientation (tensor[ti, tj, 3:7]) is not modified


@wp.kernel(enable_backward=False)
def cache_link_com(
    tensor: wp.array3d(dtype=float),
    tensor_idx: wp.array(dtype=wp.int32),
    tensor_idx_mask: wp.array(dtype=int),
    link_indices: wp.array2d(dtype=int),
    # outputs
    com_cache: wp.array3d(dtype=float),
) -> None:
    """Cache full link COM data (position + orientation) for later retrieval.

    Since Newton only stores position, we cache the full 7-element COM
    (position + orientation) in a separate buffer.

    Args:
        tensor: Input COM data (count, max_links, 7).
        tensor_idx: Articulation indices.
        tensor_idx_mask: Optional mask for indices.
        link_indices: Link body indices.
        com_cache: Output COM cache.

    """
    ti, tj, tk = wp.tid()
    arti_id = tensor_idx[ti]
    wid = link_indices[arti_id, tj]
    if tensor_idx_mask:
        apply_data = bool(tensor_idx_mask[ti])
    else:
        apply_data = True

    if apply_data and wid >= 0:
        # Cache all 7 elements: position (3) + orientation (4)
        com_cache[arti_id, tj, tk] = tensor[arti_id, tj, tk]


@wp.kernel(enable_backward=False)
def set_link_com(
    tensor: wp.array3d(dtype=float),
    tensor_idx: wp.array(dtype=wp.int32),
    tensor_idx_mask: wp.array(dtype=int),
    link_indices: wp.array2d(dtype=int),
    # outputs
    body_com: wp.array(dtype=wp.vec3),
) -> None:
    """Set link COM positions from input tensor.

    Input tensor has shape (count, max_links, 7) where each entry is:
    [com_x, com_y, com_z, qw, qx, qy, qz]

    We extract the position (first 3 values) and set into body_com.
    Orientation is ignored as Newton only stores COM offset, not orientation.

    Args:
        tensor: Input COM data (count, max_links, 7).
        tensor_idx: Articulation indices.
        tensor_idx_mask: Optional mask for indices.
        link_indices: Link body indices.
        body_com: Output body COM positions.

    """
    ti, tj, tk = wp.tid()
    arti_id = tensor_idx[ti]
    wid = link_indices[arti_id, tj]
    if tensor_idx_mask:
        apply_data = bool(tensor_idx_mask[ti])
    else:
        apply_data = True

    if apply_data and wid >= 0 and tk < 3:
        # Only copy position (first 3 values), ignore orientation
        if tk == 0:
            body_com[wid] = wp.vec3(tensor[arti_id, tj, 0], tensor[arti_id, tj, 1], tensor[arti_id, tj, 2])


@wp.kernel(enable_backward=False)
def get_dof_attributes(
    joint_attr: wp.array(dtype=wp.float32),
    index: wp.array(dtype=int),
    max_dofs: int,
    # outputs
    tensor: Any,
) -> None:
    """Get DOF attributes for articulations.

    Output shape is (count, max_dofs).

    Args:
        joint_attr: Joint attribute array.
        index: DOF indices.
        max_dofs: Maximum DOFs per articulation.
        tensor: Output tensor (count, max_dofs).

    """
    ti, tj = wp.tid()
    dof_id = ti * max_dofs + tj
    wid = index[dof_id]
    tensor[ti, tj] = joint_attr[wid]


@wp.kernel(enable_backward=False)
def get_dof_limits(
    lower_limits: wp.array(dtype=wp.float32),
    upper_limits: wp.array(dtype=wp.float32),
    index: wp.array(dtype=int),
    max_dofs: int,
    # outputs
    tensor: Any,
) -> None:
    """Get DOF limits for articulations.

    Output shape is (count, max_dofs, 2) with [lower, upper] limits.

    Args:
        lower_limits: Joint lower limits.
        upper_limits: Joint upper limits.
        index: DOF indices.
        max_dofs: Maximum DOFs per articulation.
        tensor: Output tensor (count, max_dofs, 2).

    """
    ti, tj = wp.tid()
    dof_id = ti * max_dofs + tj
    wid = index[dof_id]
    tensor[ti, tj, 0] = lower_limits[wid]
    tensor[ti, tj, 1] = upper_limits[wid]


@wp.kernel(enable_backward=False)
def set_dof_limits(
    tensor: wp.array3d(dtype=float),
    tensor_idx: wp.array(dtype=wp.int32),
    tensor_idx_mask: wp.array(dtype=int),
    index: wp.array(dtype=int),
    max_dofs: int,
    # outputs
    lower_limits: wp.array(dtype=wp.float32),
    upper_limits: wp.array(dtype=wp.float32),
) -> None:
    """Set DOF limits for articulations.

    Input shape is (count, max_dofs, 2) with [lower, upper] limits.

    Args:
        tensor: Input limits (count, max_dofs, 2).
        tensor_idx: Articulation indices.
        tensor_idx_mask: Optional mask for indices.
        index: DOF indices.
        max_dofs: Maximum DOFs per articulation.
        lower_limits: Output lower limits.
        upper_limits: Output upper limits.

    """
    ti, tj = wp.tid()
    arti_id = tensor_idx[ti]
    dof_id = wp.int32(arti_id) * max_dofs + tj
    wid = index[dof_id]
    # if a mask array is provided then only apply the data for the indices that are True
    if tensor_idx_mask:
        apply_data = bool(tensor_idx_mask[ti])
    else:
        apply_data = True

    if apply_data:
        lower_limits[wid] = tensor[arti_id, tj, 0]
        upper_limits[wid] = tensor[arti_id, tj, 1]


@wp.kernel(enable_backward=False)
def set_body_pose(
    tensor: wp.array2d(dtype=float),
    tensor_idx: wp.array(dtype=wp.int32),
    tensor_idx_mask: wp.array(dtype=int),
    body_idx: wp.array(dtype=int),
    # outputs
    body_q: wp.array(dtype=wp.transform),
) -> None:
    """Set body poses from input tensor.

    Input shape is (count, 7): [x, y, z, qx, qy, qz, qw].

    Args:
        tensor: Input poses (count, 7).
        tensor_idx: Body indices into tensor.
        tensor_idx_mask: Optional mask for indices.
        body_idx: Body indices in model.
        body_q: Output body transforms.

    """
    tid = wp.tid()
    arti_id = tensor_idx[tid]
    wid = body_idx[arti_id]
    # if a mask array is provided then only apply the data for the indices that are True
    if tensor_idx_mask:
        apply_data = bool(tensor_idx_mask[tid])
    else:
        apply_data = True

    if apply_data:
        body_q[wid] = wp.transformation(
            wp.vec3(tensor[arti_id][0], tensor[arti_id][1], tensor[arti_id][2]),
            wp.quat(tensor[arti_id][3], tensor[arti_id][4], tensor[arti_id][5], tensor[arti_id][6]),
        )


@wp.kernel(enable_backward=False)
def update_free_joint_coords_from_body_q(
    body_q: wp.array(dtype=wp.transform),
    tensor_idx: wp.array(dtype=wp.int32),
    body_idx: wp.array(dtype=int),
    joint_child: wp.array(dtype=int),
    joint_type: wp.array(dtype=int),
    joint_q_start: wp.array(dtype=int),
    # outputs
    joint_q: wp.array(dtype=wp.float32),
) -> None:
    """Update FREE joint coordinates from body transforms.

    For free rigid bodies (bodies with FREE joints), the joint_q must match body_q
    so that transforms persist through USD reloads.

    Args:
        body_q: Body transforms.
        tensor_idx: Tensor indices.
        body_idx: Body indices.
        joint_child: Joint child body indices.
        joint_type: Joint types.
        joint_q_start: Joint coordinate start indices.
        joint_q: Output joint coordinates.

    """
    tid = wp.tid()
    body_id = body_idx[tensor_idx[tid]]

    # Find the FREE joint that has this body as its child
    for joint_id in range(joint_type.shape[0]):
        if joint_child[joint_id] == body_id and joint_type[joint_id] == 4:  # FREE joint type
            # Found the FREE joint for this body
            transform = body_q[body_id]
            p = wp.transform_get_translation(transform)
            q = wp.transform_get_rotation(transform)

            # FREE joint has 7 coordinates: [tx, ty, tz, qx, qy, qz, qw]
            # Use joint_q_start to get the correct offset in joint_q array
            base_idx = joint_q_start[joint_id]
            joint_q[base_idx + 0] = p[0]
            joint_q[base_idx + 1] = p[1]
            joint_q[base_idx + 2] = p[2]
            joint_q[base_idx + 3] = q[0]
            joint_q[base_idx + 4] = q[1]
            joint_q[base_idx + 5] = q[2]
            joint_q[base_idx + 6] = q[3]
            break


@wp.kernel(enable_backward=False)
def update_root_joint_coords_from_body_q(
    body_q: wp.array(dtype=wp.transform),
    tensor_idx: wp.array(dtype=wp.int32),
    tensor_idx_mask: wp.array(dtype=int),
    body_idx: wp.array(dtype=int),
    root_joint_idx: wp.array(dtype=int),
    joint_type: wp.array(dtype=int),
    joint_q_start: wp.array(dtype=int),
    joint_X_c: wp.array(dtype=wp.transform),
    # outputs
    joint_q: wp.array(dtype=wp.float32),
    joint_X_p: wp.array(dtype=wp.transform),
) -> None:
    """Sync a root body pose into the articulation's root joint for both bases.

    Floating base (FREE root joint): the free joint's transform is joint_q[0:7],
    which forward kinematics places as body_q[root] = joint_X_p * joint_q *
    inverse(joint_X_c), so joint_q = inverse(joint_X_p) * body_q[root] * joint_X_c.
    Fixed base (FIXED root joint): the root pose lives in joint_X_p, and forward
    kinematics rebuilds body_q[root] = joint_X_p * inverse(joint_X_c) (the fixed
    joint contributes an identity joint transform), so the parent transform that
    reproduces a target pose is joint_X_p = body_q[root] * joint_X_c. Only the
    root joint is touched; actuated joints follow the root rigidly.

    Args:
        body_q: Body transforms (root pose already written into body_q[root]).
        tensor_idx: Tensor indices.
        tensor_idx_mask: Optional mask; masked-out entries are skipped.
        body_idx: Root body index per articulation.
        root_joint_idx: Root joint index per articulation.
        joint_type: Joint types.
        joint_q_start: Joint coordinate start indices.
        joint_X_c: Joint child-frame transforms.
        joint_q: Output joint coordinates (floating base).
        joint_X_p: Output joint parent transforms (fixed base).

    """
    tid = wp.tid()
    if tensor_idx_mask and not bool(tensor_idx_mask[tid]):
        return
    idx = tensor_idx[tid]
    body_id = body_idx[idx]
    joint_id = root_joint_idx[idx]

    if joint_type[joint_id] == 4:  # FREE joint type (floating base)
        transform = wp.transform_inverse(joint_X_p[joint_id]) * body_q[body_id] * joint_X_c[joint_id]
        p = wp.transform_get_translation(transform)
        q = wp.transform_get_rotation(transform)
        base_idx = joint_q_start[joint_id]
        joint_q[base_idx + 0] = p[0]
        joint_q[base_idx + 1] = p[1]
        joint_q[base_idx + 2] = p[2]
        joint_q[base_idx + 3] = q[0]
        joint_q[base_idx + 4] = q[1]
        joint_q[base_idx + 5] = q[2]
        joint_q[base_idx + 6] = q[3]
    elif joint_type[joint_id] == 3:  # FIXED joint type (fixed base)
        joint_X_p[joint_id] = body_q[body_id] * joint_X_c[joint_id]


@wp.kernel(enable_backward=False)
def update_root_joint_qd_from_body_qd(
    body_qd: wp.array(dtype=wp.spatial_vector),
    tensor_idx: wp.array(dtype=wp.int32),
    tensor_idx_mask: wp.array(dtype=int),
    body_idx: wp.array(dtype=int),
    root_joint_idx: wp.array(dtype=int),
    joint_type: wp.array(dtype=int),
    joint_qd_start: wp.array(dtype=int),
    joint_X_p: wp.array(dtype=wp.transform),
    # outputs
    joint_qd: wp.array(dtype=wp.float32),
) -> None:
    """Sync a floating base's root body spatial velocity into its free-joint qd.

    Mirrors newton.eval_ik's FREE-joint branch for a world-parented root (parent
    < 0), where the parent/moment-arm terms vanish: the free-joint velocity
    coordinates are the root body's spatial velocity rotated into the parent-anchor
    frame, laid out as [linear(3), angular(3)]. Only the root joint is written, so
    actuated joint velocities stay intact -- a full eval_ik would rewrite them from
    the just-changed root body_qd. Fixed bases carry no free-joint velocity dofs and
    are skipped.

    Args:
        body_qd: Body spatial velocities (root velocity already written in).
        tensor_idx: Tensor indices.
        tensor_idx_mask: Optional mask; masked-out entries are skipped.
        body_idx: Root body index per articulation.
        root_joint_idx: Root joint index per articulation.
        joint_type: Joint types.
        joint_qd_start: Joint velocity-coordinate start indices.
        joint_X_p: Joint parent-frame transforms.
        joint_qd: Output joint velocity coordinates (floating base).

    """
    tid = wp.tid()
    if tensor_idx_mask and not bool(tensor_idx_mask[tid]):
        return
    idx = tensor_idx[tid]
    joint_id = root_joint_idx[idx]
    if joint_type[joint_id] != 4:  # only FREE (floating base) has a root velocity dof
        return
    body_id = body_idx[idx]
    q_p = wp.transform_get_rotation(joint_X_p[joint_id])
    v_lin = wp.quat_rotate_inv(q_p, wp.spatial_top(body_qd[body_id]))
    w_ang = wp.quat_rotate_inv(q_p, wp.spatial_bottom(body_qd[body_id]))
    base_idx = joint_qd_start[joint_id]
    joint_qd[base_idx + 0] = v_lin[0]
    joint_qd[base_idx + 1] = v_lin[1]
    joint_qd[base_idx + 2] = v_lin[2]
    joint_qd[base_idx + 3] = w_ang[0]
    joint_qd[base_idx + 4] = w_ang[1]
    joint_qd[base_idx + 5] = w_ang[2]


@wp.kernel(enable_backward=False)
def set_body_velocity(
    tensor: wp.array2d(dtype=float),
    tensor_idx: wp.array(dtype=wp.int32),
    tensor_idx_mask: wp.array(dtype=int),
    body_idx: wp.array(dtype=int),
    # outputs
    body_qd: wp.array(dtype=wp.spatial_vector),
) -> None:
    """Set body velocities from input tensor.

    Input shape is (count, 6): [vx, vy, vz, wx, wy, wz].

    Args:
        tensor: Input velocities (count, 6).
        tensor_idx: Body indices into tensor.
        tensor_idx_mask: Optional mask for indices.
        body_idx: Body indices in model.
        body_qd: Output body velocities.

    """
    tid = wp.tid()
    arti_id = tensor_idx[tid]
    wid = body_idx[arti_id]
    # if a mask array is provided then only apply the data for the indices that are True
    if tensor_idx_mask:
        apply_data = bool(tensor_idx_mask[tid])
    else:
        apply_data = True

    if apply_data:
        # Newton spatial vectors and the public tensor API are both linear-first.
        body_qd[wid] = wp.spatial_vector(
            wp.vec3(tensor[arti_id][0], tensor[arti_id][1], tensor[arti_id][2]),
            wp.vec3(tensor[arti_id][3], tensor[arti_id][4], tensor[arti_id][5]),
        )


@wp.kernel(enable_backward=False)
def set_link_mass(
    tensor: wp.array2d(dtype=float),
    tensor_idx: wp.array(dtype=wp.int32),
    tensor_idx_mask: wp.array(dtype=int),
    link_indices: wp.array2d(dtype=int),
    # outputs
    body_mass: wp.array(dtype=wp.float32),
) -> None:
    """Set link masses for articulations.

    Input shape is (count, max_links).

    Args:
        tensor: Input masses (count, max_links).
        tensor_idx: Articulation indices.
        tensor_idx_mask: Optional mask for indices.
        link_indices: Link body indices.
        body_mass: Output body masses.

    """
    ti, tj = wp.tid()
    arti_id = tensor_idx[ti]
    wid = link_indices[arti_id, tj]
    if tensor_idx_mask:
        apply_data = bool(tensor_idx_mask[ti])
    else:
        apply_data = True

    if apply_data and wid >= 0:
        body_mass[wid] = tensor[arti_id, tj]


@wp.kernel(enable_backward=False)
def update_inv_mass(
    tensor_idx: wp.array(dtype=wp.int32),
    tensor_idx_mask: wp.array(dtype=int),
    link_indices: wp.array2d(dtype=int),
    body_mass: wp.array(dtype=wp.float32),
    # outputs
    body_inv_mass: wp.array(dtype=wp.float32),
) -> None:
    """Update inverse mass from mass for articulation links.

    Args:
        tensor_idx: Articulation indices.
        tensor_idx_mask: Optional mask for indices.
        link_indices: Link body indices.
        body_mass: Body masses.
        body_inv_mass: Output inverse masses.

    """
    ti, tj = wp.tid()
    arti_id = tensor_idx[ti]
    wid = link_indices[arti_id, tj]
    if tensor_idx_mask:
        apply_data = bool(tensor_idx_mask[ti])
    else:
        apply_data = True

    if apply_data and wid >= 0:
        mass = body_mass[wid]
        if mass > 1e-8:
            body_inv_mass[wid] = 1.0 / mass
        else:
            body_inv_mass[wid] = 0.0


@wp.kernel(enable_backward=False)
def set_link_inertia(
    tensor: wp.array3d(dtype=float),
    tensor_idx: wp.array(dtype=wp.int32),
    tensor_idx_mask: wp.array(dtype=int),
    link_indices: wp.array2d(dtype=int),
    # outputs
    body_inertia: wp.array(dtype=wp.mat33),
) -> None:
    """Set link inertia tensors for articulations.

    Input shape is (count, max_links, 9) in row-major order.

    Args:
        tensor: Input inertias (count, max_links, 9).
        tensor_idx: Articulation indices.
        tensor_idx_mask: Optional mask for indices.
        link_indices: Link body indices.
        body_inertia: Output body inertias.

    """
    ti, tj, tk = wp.tid()
    arti_id = tensor_idx[ti]
    wid = link_indices[arti_id, tj]
    if tensor_idx_mask:
        apply_data = bool(tensor_idx_mask[ti])
    else:
        apply_data = True

    if apply_data and wid >= 0:
        i = tk // 3
        j = tk % 3
        body_inertia[wid][i][j] = tensor[arti_id, tj, tk]


@wp.kernel(enable_backward=False)
def update_inv_inertia(
    tensor_idx: wp.array(dtype=wp.int32),
    tensor_idx_mask: wp.array(dtype=int),
    link_indices: wp.array2d(dtype=int),
    body_inertia: wp.array(dtype=wp.mat33),
    # outputs
    body_inv_inertia: wp.array(dtype=wp.mat33),
) -> None:
    """Update inverse inertia from inertia tensor.

    Args:
        tensor_idx: Articulation indices.
        tensor_idx_mask: Optional mask for indices.
        link_indices: Link body indices.
        body_inertia: Body inertia matrices.
        body_inv_inertia: Output inverse inertia matrices.

    """
    ti, tj = wp.tid()
    arti_id = tensor_idx[ti]
    wid = link_indices[arti_id, tj]
    if tensor_idx_mask:
        apply_data = bool(tensor_idx_mask[ti])
    else:
        apply_data = True

    if apply_data and wid >= 0:
        inertia = body_inertia[wid]
        # Check if inertia is non-zero before computing inverse
        det = wp.determinant(inertia)
        if wp.abs(det) > 1e-8:
            body_inv_inertia[wid] = wp.inverse(inertia)
        else:
            # Zero or singular inertia -> zero inverse
            body_inv_inertia[wid] = wp.mat33(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0)


@wp.kernel(enable_backward=False)
def set_dof_attributes(
    tensor: wp.array2d(dtype=float),
    tensor_idx: wp.array(dtype=wp.int32),
    tensor_idx_mask: wp.array(dtype=int),
    index: wp.array(dtype=int),
    max_dofs: int,
    # outputs
    joint_attr: wp.array(dtype=wp.float32),
) -> None:
    """Set DOF attributes for articulations.

    Input shape is (count, max_dofs).

    Args:
        tensor: Input attributes (count, max_dofs).
        tensor_idx: Articulation indices.
        tensor_idx_mask: Optional mask for indices.
        index: DOF indices.
        max_dofs: Maximum DOFs per articulation.
        joint_attr: Output joint attributes.

    """
    ti, tj = wp.tid()
    arti_id = tensor_idx[ti]
    dof_id = wp.int32(arti_id) * max_dofs + tj
    wid = index[dof_id]
    # if a mask array is provided then only apply the data for the indices that are True
    if tensor_idx_mask:
        apply_data = bool(tensor_idx_mask[ti])
    else:
        apply_data = True

    if apply_data:
        joint_attr[wid] = tensor[arti_id, tj]


@wp.kernel(enable_backward=False)
def apply_body_forces_at_position(
    force_tensor: wp.array2d(dtype=float),
    torque_tensor: wp.array2d(dtype=float),
    position_tensor: wp.array2d(dtype=float),
    tensor_idx: wp.array(dtype=wp.int32),
    tensor_idx_mask: wp.array(dtype=int),
    body_idx: wp.array(dtype=int),
    body_q: wp.array(dtype=wp.transform),
    body_com: wp.array(dtype=wp.vec3),
    has_force: bool,
    has_torque: bool,
    has_position: bool,
    # outputs
    body_f: wp.array(dtype=wp.spatial_vector),
) -> None:
    """Apply world-frame forces and torques to rigid bodies at specified positions.

    Args:
        force_tensor: Forces to apply, shape (count, 3).
        torque_tensor: Torques to apply, shape (count, 3).
        position_tensor: Positions where forces are applied, shape (count, 3).
        tensor_idx: Indices of bodies to update.
        tensor_idx_mask: Optional mask for indices.
        body_idx: Body indices in the model.
        body_q: Body transforms.
        body_com: Body center of mass offsets.
        has_force: Whether force tensor is provided.
        has_torque: Whether torque tensor is provided.
        has_position: Whether position tensor is provided.
        body_f: Output body forces (spatial vectors).

    """
    tid = wp.tid()

    # Check mask if provided
    if tensor_idx_mask:
        if not bool(tensor_idx_mask[tid]):
            return

    body_id_in_view = tensor_idx[tid]
    wid = body_idx[body_id_in_view]

    # Get body transform and COM
    transform = body_q[wid]
    rotation = wp.transform_get_rotation(transform)
    translation = wp.transform_get_translation(transform)
    com_offset = body_com[wid]

    # Compute world-space COM position
    com_world = translation + wp.quat_rotate(rotation, com_offset)

    # Initialize force and torque in world frame
    force_world = wp.vec3(0.0, 0.0, 0.0)
    torque_world = wp.vec3(0.0, 0.0, 0.0)

    # Process force
    if has_force:
        force_world = wp.vec3(
            force_tensor[body_id_in_view, 0],
            force_tensor[body_id_in_view, 1],
            force_tensor[body_id_in_view, 2],
        )

        # If position is specified, compute additional torque from force application point
        if has_position:
            position_world = wp.vec3(
                position_tensor[body_id_in_view, 0],
                position_tensor[body_id_in_view, 1],
                position_tensor[body_id_in_view, 2],
            )

            # Compute torque from force application: τ = (r - r_com) × F
            r_offset = position_world - com_world
            torque_from_force = wp.cross(r_offset, force_world)
            torque_world = torque_world + torque_from_force

    # Process torque
    if has_torque:
        torque_world = torque_world + wp.vec3(
            torque_tensor[body_id_in_view, 0],
            torque_tensor[body_id_in_view, 1],
            torque_tensor[body_id_in_view, 2],
        )

    # Apply to body forces (set directly, don't add - forces are cleared each step)
    # spatial_vector is [linear (force), angular (torque)] - note the order!
    body_f[wid] = wp.spatial_vector(force_world, torque_world)


@wp.kernel(enable_backward=False)
def apply_link_forces_at_position(
    force_tensor: wp.array3d(dtype=float),
    torque_tensor: wp.array3d(dtype=float),
    position_tensor: wp.array3d(dtype=float),
    tensor_idx: wp.array(dtype=wp.int32),
    tensor_idx_mask: wp.array(dtype=int),
    link_indices: wp.array2d(dtype=int),
    body_q: wp.array(dtype=wp.transform),
    body_com: wp.array(dtype=wp.vec3),
    has_force: bool,
    has_torque: bool,
    has_position: bool,
    # outputs
    body_f: wp.array(dtype=wp.spatial_vector),
) -> None:
    """Apply world-frame forces and torques to articulation links at specified positions.

    Args:
        force_tensor: Forces to apply, shape (count, max_links, 3).
        torque_tensor: Torques to apply, shape (count, max_links, 3).
        position_tensor: Positions where forces are applied, shape (count, max_links, 3).
        tensor_idx: Indices of articulations to update.
        tensor_idx_mask: Optional mask for indices.
        link_indices: Link body indices, shape (count, max_links).
        body_q: Body transforms.
        body_com: Body center of mass offsets.
        has_force: Whether force tensor is provided.
        has_torque: Whether torque tensor is provided.
        has_position: Whether position tensor is provided.
        body_f: Output body forces (spatial vectors).

    """
    ti, tj = wp.tid()

    # Check mask if provided
    if tensor_idx_mask:
        if not bool(tensor_idx_mask[ti]):
            return

    arti_id = tensor_idx[ti]
    wid = link_indices[arti_id, tj]

    if wid < 0:
        return

    # Get body transform and COM
    transform = body_q[wid]
    rotation = wp.transform_get_rotation(transform)
    translation = wp.transform_get_translation(transform)
    com_offset = body_com[wid]

    # Compute world-space COM position
    com_world = translation + wp.quat_rotate(rotation, com_offset)

    # Initialize force and torque in world frame
    force_world = wp.vec3(0.0, 0.0, 0.0)
    torque_world = wp.vec3(0.0, 0.0, 0.0)

    # Process force
    if has_force:
        force_world = wp.vec3(
            force_tensor[arti_id, tj, 0],
            force_tensor[arti_id, tj, 1],
            force_tensor[arti_id, tj, 2],
        )

        # If position is specified, compute additional torque from force application point
        if has_position:
            position_world = wp.vec3(
                position_tensor[arti_id, tj, 0],
                position_tensor[arti_id, tj, 1],
                position_tensor[arti_id, tj, 2],
            )

            # Compute torque from force application: τ = (r - r_com) × F
            r_offset = position_world - com_world
            torque_from_force = wp.cross(r_offset, force_world)
            torque_world = torque_world + torque_from_force

    # Process torque
    if has_torque:
        torque_world = torque_world + wp.vec3(
            torque_tensor[arti_id, tj, 0],
            torque_tensor[arti_id, tj, 1],
            torque_tensor[arti_id, tj, 2],
        )

    # Apply to body forces (set directly, don't add - forces are cleared each step)
    # spatial_vector is [linear (force), angular (torque)] - note the order!
    body_f[wid] = wp.spatial_vector(force_world, torque_world)


@wp.kernel(enable_backward=False)
def sync_ctrl_direct_targets(
    dof_to_act: wp.array(dtype=wp.int32),
    joint_target_pos: wp.array(dtype=wp.float32),
    dofs_per_world: wp.int32,
    ctrls_per_world: wp.int32,
    # output
    mujoco_ctrl: wp.array(dtype=wp.float32),
) -> None:
    """Sync joint_target_pos to control.mujoco.ctrl for CTRL_DIRECT joint actuators.

    Args:
        dof_to_act: Template DOF -> mujoco:actuator index mapping (-1 = no mapping).
        joint_target_pos: Per-DOF position targets (flat, all worlds).
        dofs_per_world: Number of DOFs per world.
        ctrls_per_world: Number of ctrl entries per world.
        mujoco_ctrl: Output control.mujoco.ctrl array (flat, all worlds).

    """
    world, dof = wp.tid()
    act_idx = dof_to_act[dof]
    if act_idx < 0:
        return
    src = world * dofs_per_world + dof
    dst = world * ctrls_per_world + act_idx
    mujoco_ctrl[dst] = joint_target_pos[src]


@wp.kernel(enable_backward=False)
def sync_ctrl_direct_gains(
    dof_to_act: wp.array(dtype=wp.int32),
    joint_target_ke: wp.array(dtype=wp.float32),
    joint_target_kd: wp.array(dtype=wp.float32),
    # output
    actuator_gainprm: wp.array(dtype=vec10),
    actuator_biasprm: wp.array(dtype=vec10),
) -> None:
    """Sync joint_target_ke/kd to actuator gainprm/biasprm for CTRL_DIRECT joint actuators.

    Reads from template DOF (world 0). Only updates when kp > 0 or kd > 0.

    Args:
        dof_to_act: Template DOF -> mujoco:actuator index mapping (-1 = no mapping).
        joint_target_ke: Per-DOF position gains (flat, all worlds). Reads world 0.
        joint_target_kd: Per-DOF damping gains (flat, all worlds). Reads world 0.
        actuator_gainprm: Model mujoco actuator gain params (template-level).
        actuator_biasprm: Model mujoco actuator bias params (template-level).

    """
    dof = wp.tid()
    act_idx = dof_to_act[dof]
    if act_idx < 0:
        return
    kp = joint_target_ke[dof]
    kd = joint_target_kd[dof]
    if kp > 0.0 or kd > 0.0:
        actuator_gainprm[act_idx][0] = kp
        actuator_biasprm[act_idx][1] = -kp
        actuator_biasprm[act_idx][2] = -kd


def build_ctrl_direct_dof_mapping(model: newton.Model) -> wp.array | None:
    """Build a DOF-to-CTRL_DIRECT-actuator mapping from model custom attributes.

    For each template degree of freedom that has a direct-control joint
    actuator, store the corresponding actuator index.

    Args:
        model: Newton model with mujoco custom attributes.

    Returns:
        Per-template-degree-of-freedom actuator indices, or None when the model
        has no resolvable direct-control joint actuator.

    """
    mujoco_attrs = getattr(model, "mujoco", None)
    if mujoco_attrs is None:
        return None

    actuator_count = model.custom_frequency_counts.get("mujoco:actuator", 0)
    if actuator_count == 0:
        return None

    has_trnid = hasattr(mujoco_attrs, "actuator_trnid")
    has_ctrl_source = hasattr(mujoco_attrs, "ctrl_source")
    has_trntype = hasattr(mujoco_attrs, "actuator_trntype")

    if not has_trnid:
        return None

    trnid = mujoco_attrs.actuator_trnid.numpy()
    ctrl_source = mujoco_attrs.ctrl_source.numpy() if has_ctrl_source else None
    trntype = mujoco_attrs.actuator_trntype.numpy() if has_trntype else None

    # Build label-based lookup for deferred target resolution (trnid=-1).
    # The model builder may defer resolution, storing -1 in trnid and setting
    # actuator_target_label instead. The solver resolves these at init time
    # via joint_dof_label; we replicate that logic here.
    target_labels = getattr(mujoco_attrs, "actuator_target_label", None)
    joint_dof_labels = getattr(mujoco_attrs, "joint_dof_label", None)
    dof_label_to_idx: dict[str, int] = {}
    if isinstance(joint_dof_labels, list):
        for i, label in enumerate(joint_dof_labels):
            if label:
                dof_label_to_idx[label] = i

    nworlds = model.world_count if hasattr(model, "world_count") else 1
    dofs_per_world = model.joint_dof_count // nworlds if nworlds > 0 else model.joint_dof_count

    if dofs_per_world == 0:
        return None

    dof_to_act = np.full(dofs_per_world, -1, dtype=np.int32)
    found_any = False

    for act_idx in range(actuator_count):
        is_ctrl_direct = ctrl_source is None or int(ctrl_source[act_idx]) == 1
        is_joint = trntype is None or int(trntype[act_idx]) == 0
        if not (is_ctrl_direct and is_joint):
            continue

        dof_idx = int(trnid[act_idx, 0]) if trnid.ndim > 1 else int(trnid[act_idx])

        # Deferred resolution: trnid=-1 means the model builder didn't resolve
        # the target. Use actuator_target_label + joint_dof_label to resolve.
        if dof_idx < 0 and isinstance(target_labels, list) and act_idx < len(target_labels):
            label = target_labels[act_idx]
            if label in dof_label_to_idx:
                dof_idx = dof_label_to_idx[label]

        if 0 <= dof_idx < dofs_per_world:
            dof_to_act[dof_idx] = act_idx
            found_any = True

    if not found_any:
        return None

    return wp.array(dof_to_act, dtype=wp.int32, device=model.device)


@wp.kernel(enable_backward=False)
def get_link_pose(
    body_q: wp.array(dtype=wp.transform),
    index: wp.array2d(dtype=int),
    # outputs
    tensor: wp.array3d(dtype=float),
) -> None:
    """Get per-link poses for articulations, shape (count, max_links, 7).

    Each link's 7-vector is [x, y, z, qx, qy, qz, qw]. Padded link slots
    (index < 0) are written as the identity transform.

    Args:
        body_q: Body transforms.
        index: Link body indices (count, max_links); -1 marks a padded slot.
        tensor: Output tensor (count, max_links, 7).

    """
    ti, tj = wp.tid()
    wid = index[ti, tj]
    if wid >= 0:
        X_ws = body_q[wid]
        tensor[ti, tj, 0] = X_ws[0]
        tensor[ti, tj, 1] = X_ws[1]
        tensor[ti, tj, 2] = X_ws[2]
        tensor[ti, tj, 3] = X_ws[3]
        tensor[ti, tj, 4] = X_ws[4]
        tensor[ti, tj, 5] = X_ws[5]
        tensor[ti, tj, 6] = X_ws[6]
    else:
        tensor[ti, tj, 0] = 0.0
        tensor[ti, tj, 1] = 0.0
        tensor[ti, tj, 2] = 0.0
        tensor[ti, tj, 3] = 0.0
        tensor[ti, tj, 4] = 0.0
        tensor[ti, tj, 5] = 0.0
        tensor[ti, tj, 6] = 1.0


@wp.kernel(enable_backward=False)
def get_link_velocity(
    body_qd: wp.array(dtype=wp.spatial_vector),
    index: wp.array2d(dtype=int),
    # outputs
    tensor: wp.array3d(dtype=float),
) -> None:
    """Get per-link spatial velocities, shape (count, max_links, 6).

    Each link's 6-vector is [vx, vy, vz, wx, wy, wz]. Padded link slots
    (index < 0) are written as zeros.

    Args:
        body_qd: Body spatial velocities.
        index: Link body indices (count, max_links); -1 marks a padded slot.
        tensor: Output tensor (count, max_links, 6).

    """
    ti, tj = wp.tid()
    wid = index[ti, tj]
    if wid >= 0:
        spatial_vel = body_qd[wid]
        linear_vel = wp.spatial_top(spatial_vel)
        angular_vel = wp.spatial_bottom(spatial_vel)
        tensor[ti, tj, 0] = linear_vel[0]
        tensor[ti, tj, 1] = linear_vel[1]
        tensor[ti, tj, 2] = linear_vel[2]
        tensor[ti, tj, 3] = angular_vel[0]
        tensor[ti, tj, 4] = angular_vel[1]
        tensor[ti, tj, 5] = angular_vel[2]
    else:
        tensor[ti, tj, 0] = 0.0
        tensor[ti, tj, 1] = 0.0
        tensor[ti, tj, 2] = 0.0
        tensor[ti, tj, 3] = 0.0
        tensor[ti, tj, 4] = 0.0
        tensor[ti, tj, 5] = 0.0


@wp.kernel(enable_backward=False)
def gather_float_matrix(
    source: wp.array3d(dtype=wp.float32),
    indices: wp.array(dtype=wp.int32),
    dropped: wp.float32,
    # outputs
    destination: wp.array3d(dtype=wp.float32),
) -> None:
    """Gather selected matrices into a dense destination batch.

    An out-of-range index yields the dropped-row marker. Index values are never read
    back to be rejected, so the bound is enforced here. Writing the marker from this
    kernel rather than pre-filling keeps the read to a single pass.

    Args:
        source: Source matrix batch.
        indices: Source batch index for each destination matrix.
        dropped: Value marking an entry no valid index produced.
        destination: Dense destination matrix batch.

    """
    row, j, k = wp.tid()
    index = indices[row]
    if index >= 0 and index < source.shape[0]:
        destination[row, j, k] = source[index, j, k]
    else:
        destination[row, j, k] = dropped
