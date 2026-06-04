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

"""Warp kernels for Newton tensor operations.

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
def get_body_pose(
    body_q: wp.array(dtype=wp.transform),  # type: ignore[valid-type]
    index: wp.array(dtype=int),  # type: ignore[valid-type]
    # outputs
    tensor: wp.array2d(dtype=float),  # type: ignore[valid-type]
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
    body_qd: wp.array(dtype=wp.spatial_vector),  # type: ignore[valid-type]
    index: wp.array(dtype=int),  # type: ignore[valid-type]
    # outputs
    tensor: wp.array2d(dtype=float),  # type: ignore[valid-type]
) -> None:
    """Get body velocities as angular and linear components.

    Output shape is (count, 6): [wx, wy, wz, vx, vy, vz].

    Args:
        body_qd: Body spatial velocities.
        index: Body indices to read.
        tensor: Output tensor (count, 6).
    """
    tid = wp.tid()
    wid = index[tid]
    # spatial_vector is [angular, linear], extract components using proper Warp functions
    spatial_vel = body_qd[wid]

    # Use proper Warp spatial vector accessors
    # Based on debug output: spatial_top contains our angular values, spatial_bottom contains linear
    angular_vel = wp.spatial_top(spatial_vel)  # gets angular component (w)
    linear_vel = wp.spatial_bottom(spatial_vel)  # gets linear component (v)
    # Return in [angular, linear] format
    tensor[tid, 0] = angular_vel[0]  # type: ignore[index]
    tensor[tid, 1] = angular_vel[1]  # type: ignore[index]
    tensor[tid, 2] = angular_vel[2]  # type: ignore[index]
    tensor[tid, 3] = linear_vel[0]  # type: ignore[index]
    tensor[tid, 4] = linear_vel[1]  # type: ignore[index]
    tensor[tid, 5] = linear_vel[2]  # type: ignore[index]


@wp.kernel(enable_backward=False)
def get_body_mass(
    body_mass: wp.array(dtype=wp.float32),  # type: ignore[valid-type]
    index: wp.array(dtype=int),  # type: ignore[valid-type]
    # outputs
    tensor: wp.array2d(dtype=float),  # type: ignore[valid-type]
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
    body_inv_mass: wp.array(dtype=wp.float32),  # type: ignore[valid-type]
    index: wp.array(dtype=int),  # type: ignore[valid-type]
    # outputs
    tensor: wp.array2d(dtype=float),  # type: ignore[valid-type]
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
    body_com: wp.array(dtype=wp.vec3),  # type: ignore[valid-type]
    index: wp.array(dtype=int),  # type: ignore[valid-type]
    # outputs
    tensor: wp.array2d(dtype=float),  # type: ignore[valid-type]
) -> None:
    """Get COM position and orientation for rigid bodies.

    Returns shape (count, 7): [com_x, com_y, com_z, qx, qy, qz, qw]
    Newton stores COM as offset in body frame, orientation is identity.

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
    # Identity quaternion (w, x, y, z) in scalar-first format
    tensor[tid, 3] = 0.0  # qx
    tensor[tid, 4] = 0.0  # qy
    tensor[tid, 5] = 0.0  # qz
    tensor[tid, 6] = 1.0  # qw


@wp.kernel(enable_backward=False)
def get_body_com_position_only(
    body_com: wp.array(dtype=wp.vec3),  # type: ignore[valid-type]
    index: wp.array(dtype=int),  # type: ignore[valid-type]
    # outputs
    tensor: wp.array2d(dtype=float),  # type: ignore[valid-type]
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
    tensor: wp.array2d(dtype=float),  # type: ignore[valid-type]
    tensor_idx: Any,
    tenor_idx_mask: wp.array(dtype=int),  # type: ignore[valid-type]
    body_idx: wp.array(dtype=int),  # type: ignore[valid-type]
    # outputs
    com_cache: wp.array2d(dtype=float),  # type: ignore[valid-type]
) -> None:
    """Cache full COM data (position + orientation) for later retrieval.

    Since Newton only stores position, we cache the full 7-element COM
    (position + orientation) in a separate buffer.

    Args:
        tensor: Input COM data (count, 7).
        tensor_idx: Indices into tensor.
        tenor_idx_mask: Optional mask for indices.
        body_idx: Body indices.
        com_cache: Output COM cache.
    """
    tid = wp.tid()
    body_id = tensor_idx[tid]
    wid = body_idx[body_id]
    if tenor_idx_mask:
        apply_data = bool(tenor_idx_mask[tid])
    else:
        apply_data = True

    if apply_data:
        # Cache all 7 elements: position (3) + orientation (4)
        for i in range(7):
            com_cache[wid, i] = tensor[body_id, i]


wp.overload(cache_body_com, {"tensor_idx": wp.array(dtype=wp.int32)})
wp.overload(cache_body_com, {"tensor_idx": wp.array(dtype=wp.int64)})


@wp.kernel(enable_backward=False)
def get_body_inertia(
    body_inertia: wp.array(dtype=wp.mat33),  # type: ignore[valid-type]
    index: wp.array(dtype=int),  # type: ignore[valid-type]
    # outputs
    tensor: wp.array2d(dtype=float),  # type: ignore[valid-type]
) -> None:
    """Get inertia tensor as flattened 3x3 matrix.

    Returns shape (count, 9) in row-major order.

    Args:
        body_inertia: Body inertia matrices.
        index: Body indices to read.
        tensor: Output tensor (count, 9).
    """
    tid = wp.tid()
    wid = index[tid]
    I = body_inertia[wid]
    # Flatten 3x3 matrix in row-major order
    tensor[tid, 0] = I[0, 0]
    tensor[tid, 1] = I[0, 1]
    tensor[tid, 2] = I[0, 2]
    tensor[tid, 3] = I[1, 0]
    tensor[tid, 4] = I[1, 1]
    tensor[tid, 5] = I[1, 2]
    tensor[tid, 6] = I[2, 0]
    tensor[tid, 7] = I[2, 1]
    tensor[tid, 8] = I[2, 2]


@wp.kernel(enable_backward=False)
def get_body_inv_inertia(
    body_inv_inertia: wp.array(dtype=wp.mat33),  # type: ignore[valid-type]
    index: wp.array(dtype=int),  # type: ignore[valid-type]
    # outputs
    tensor: wp.array2d(dtype=float),  # type: ignore[valid-type]
) -> None:
    """Get inverse inertia tensor as flattened 3x3 matrix.

    Returns shape (count, 9) in row-major order.

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
    body_mass: wp.array(dtype=wp.float32),  # type: ignore[valid-type]
    index: wp.array2d(dtype=int),  # type: ignore[valid-type]
    # outputs
    tensor: wp.array2d(dtype=float),  # type: ignore[valid-type]
) -> None:
    """Get link inverse masses for articulations.

    Output shape is (count, max_links). Computes 1/mass for each link.

    Args:
        body_mass: Body masses array.
        index: Link indices (count, max_links).
        tensor: Output tensor (count, max_links).
    """
    ti, tj = wp.tid()  # type: ignore[misc]
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
    body_mass: wp.array(dtype=wp.float32),  # type: ignore[valid-type]
    index: wp.array2d(dtype=int),  # type: ignore[valid-type]
    # outputs
    tensor: wp.array2d(dtype=float),  # type: ignore[valid-type]
) -> None:
    """Get link masses for articulations.

    Output shape is (count, max_links).

    Args:
        body_mass: Body masses array.
        index: Link indices (count, max_links).
        tensor: Output tensor (count, max_links).
    """
    ti, tj = wp.tid()  # type: ignore[misc]
    wid = index[ti, tj]
    if wid >= 0:
        tensor[ti, tj] = body_mass[wid]


@wp.kernel(enable_backward=False)
def set_body_mass(
    tensor: wp.array2d(dtype=float),  # type: ignore[valid-type]
    tensor_idx: Any,
    tenor_idx_mask: wp.array(dtype=int),  # type: ignore[valid-type]
    body_idx: wp.array(dtype=int),  # type: ignore[valid-type]
    # outputs
    body_mass: wp.array(dtype=wp.float32),  # type: ignore[valid-type]
) -> None:
    """Set body masses from input tensor.

    Input shape is (count, 1).

    Args:
        tensor: Input mass values (count, 1).
        tensor_idx: Indices into tensor.
        tenor_idx_mask: Optional mask for indices.
        body_idx: Body indices.
        body_mass: Output body masses.
    """
    tid = wp.tid()
    body_id = tensor_idx[tid]
    wid = body_idx[body_id]
    # if a mask array is provided then only apply the data for the indices that are True
    if tenor_idx_mask:
        apply_data = bool(tenor_idx_mask[tid])
    else:
        apply_data = True

    if apply_data:
        body_mass[wid] = tensor[body_id, 0]


# Overload to support both int32 and int64
wp.overload(set_body_mass, {"tensor_idx": wp.array(dtype=wp.int32)})
wp.overload(set_body_mass, {"tensor_idx": wp.array(dtype=wp.int64)})


@wp.kernel(enable_backward=False)
def update_body_inv_mass(
    tensor_idx: Any,
    tenor_idx_mask: wp.array(dtype=int),  # type: ignore[valid-type]
    body_idx: wp.array(dtype=int),  # type: ignore[valid-type]
    body_mass: wp.array(dtype=wp.float32),  # type: ignore[valid-type]
    # outputs
    body_inv_mass: wp.array(dtype=wp.float32),  # type: ignore[valid-type]
) -> None:
    """Update inverse mass from mass for rigid bodies.

    Args:
        tensor_idx: Indices into tensor.
        tenor_idx_mask: Optional mask for indices.
        body_idx: Body indices.
        body_mass: Body masses.
        body_inv_mass: Output inverse masses.
    """
    tid = wp.tid()
    body_id = tensor_idx[tid]
    wid = body_idx[body_id]
    if tenor_idx_mask:
        apply_data = bool(tenor_idx_mask[tid])
    else:
        apply_data = True

    if apply_data:
        mass = body_mass[wid]
        if mass > 1e-8:
            body_inv_mass[wid] = 1.0 / mass
        else:
            body_inv_mass[wid] = 0.0


wp.overload(update_body_inv_mass, {"tensor_idx": wp.array(dtype=wp.int32)})
wp.overload(update_body_inv_mass, {"tensor_idx": wp.array(dtype=wp.int64)})


@wp.kernel(enable_backward=False)
def set_body_com(
    tensor: wp.array2d(dtype=float),  # type: ignore[valid-type]
    tensor_idx: Any,
    tenor_idx_mask: wp.array(dtype=int),  # type: ignore[valid-type]
    body_idx: wp.array(dtype=int),  # type: ignore[valid-type]
    # outputs
    body_com: wp.array(dtype=wp.vec3),  # type: ignore[valid-type]
) -> None:
    """Set COM position for rigid bodies.

    Input shape (count, 7): [com_x, com_y, com_z, qx, qy, qz, qw]
    Only position (first 3 values) is used; Newton stores COM as offset, not with orientation.

    Args:
        tensor: Input COM data (count, 7).
        tensor_idx: Indices into tensor.
        tenor_idx_mask: Optional mask for indices.
        body_idx: Body indices.
        body_com: Output COM positions.
    """
    tid = wp.tid()
    body_id = tensor_idx[tid]
    wid = body_idx[body_id]
    if tenor_idx_mask:
        apply_data = bool(tenor_idx_mask[tid])
    else:
        apply_data = True

    if apply_data:
        body_com[wid] = wp.vec3(tensor[body_id, 0], tensor[body_id, 1], tensor[body_id, 2])
        # Orientation (tensor[body_id, 3:7]) is ignored - Newton inertia is already in body frame


wp.overload(set_body_com, {"tensor_idx": wp.array(dtype=wp.int32)})
wp.overload(set_body_com, {"tensor_idx": wp.array(dtype=wp.int64)})


@wp.kernel(enable_backward=False)
def set_body_inertia(
    tensor: wp.array2d(dtype=float),  # type: ignore[valid-type]
    tensor_idx: Any,
    tenor_idx_mask: wp.array(dtype=int),  # type: ignore[valid-type]
    body_idx: wp.array(dtype=int),  # type: ignore[valid-type]
    # outputs
    body_inertia: wp.array(dtype=wp.mat33),  # type: ignore[valid-type]
) -> None:
    """Set inertia tensor from flattened 3x3 matrix.

    Input shape (count, 9) in row-major order.

    Args:
        tensor: Input inertia values (count, 9).
        tensor_idx: Indices into tensor.
        tenor_idx_mask: Optional mask for indices.
        body_idx: Body indices.
        body_inertia: Output inertia matrices.
    """
    tid = wp.tid()
    body_id = tensor_idx[tid]
    wid = body_idx[body_id]
    if tenor_idx_mask:
        apply_data = bool(tenor_idx_mask[tid])
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


wp.overload(set_body_inertia, {"tensor_idx": wp.array(dtype=wp.int32)})
wp.overload(set_body_inertia, {"tensor_idx": wp.array(dtype=wp.int64)})


@wp.kernel(enable_backward=False)
def update_body_inv_inertia(
    tensor_idx: Any,
    tenor_idx_mask: wp.array(dtype=int),  # type: ignore[valid-type]
    body_idx: wp.array(dtype=int),  # type: ignore[valid-type]
    body_inertia: wp.array(dtype=wp.mat33),  # type: ignore[valid-type]
    # outputs
    body_inv_inertia: wp.array(dtype=wp.mat33),  # type: ignore[valid-type]
) -> None:
    """Update inverse inertia from inertia tensor for rigid bodies.

    Args:
        tensor_idx: Indices into tensor.
        tenor_idx_mask: Optional mask for indices.
        body_idx: Body indices.
        body_inertia: Body inertia matrices.
        body_inv_inertia: Output inverse inertia matrices.
    """
    tid = wp.tid()
    body_id = tensor_idx[tid]
    wid = body_idx[body_id]
    if tenor_idx_mask:
        apply_data = bool(tenor_idx_mask[tid])
    else:
        apply_data = True

    if apply_data:
        inertia = body_inertia[wid]
        det = wp.determinant(inertia)
        if wp.abs(det) > 1e-8:
            body_inv_inertia[wid] = wp.inverse(inertia)
        else:
            body_inv_inertia[wid] = wp.mat33(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0)


wp.overload(update_body_inv_inertia, {"tensor_idx": wp.array(dtype=wp.int32)})
wp.overload(update_body_inv_inertia, {"tensor_idx": wp.array(dtype=wp.int64)})


@wp.kernel(enable_backward=False)
def get_link_inertia(
    body_inertia: wp.array(dtype=wp.mat33),  # type: ignore[valid-type]
    index: wp.array2d(dtype=int),  # type: ignore[valid-type]
    # outputs
    tensor: wp.array3d(dtype=float),  # type: ignore[valid-type]
) -> None:
    """Get link inertia tensors for articulations.

    Output shape is (count, max_links, 9) in row-major order.

    Args:
        body_inertia: Body inertia matrices.
        index: Link indices (count, max_links).
        tensor: Output tensor (count, max_links, 9).
    """
    ti, tj, tk = wp.tid()  # type: ignore[misc]
    wid = index[ti, tj]
    if wid >= 0:
        i = tk // 3
        j = tk % 3
        tensor[ti, tj, tk] = body_inertia[wid][i][j]


@wp.kernel(enable_backward=False)
def get_link_inv_inertia(
    body_inv_inertia: wp.array(dtype=wp.mat33),  # type: ignore[valid-type]
    index: wp.array2d(dtype=int),  # type: ignore[valid-type]
    # outputs
    tensor: wp.array3d(dtype=float),  # type: ignore[valid-type]
) -> None:
    """Get link inverse inertia tensors for articulations.

    Output shape is (count, max_links, 9) in row-major order.

    Args:
        body_inv_inertia: Body inverse inertia matrices.
        index: Link indices (count, max_links).
        tensor: Output tensor (count, max_links, 9).
    """
    ti, tj, tk = wp.tid()  # type: ignore[misc]
    wid = index[ti, tj]
    if wid >= 0:
        i = tk // 3
        j = tk % 3
        tensor[ti, tj, tk] = body_inv_inertia[wid][i][j]


@wp.kernel(enable_backward=False)
def get_link_com(
    body_com: wp.array(dtype=wp.vec3),  # type: ignore[valid-type]
    index: wp.array2d(dtype=int),  # type: ignore[valid-type]
    # outputs
    tensor: wp.array3d(dtype=float),  # type: ignore[valid-type]
) -> None:
    """Get link COM positions and orientations.

    Output tensor has shape (count, max_links, 7) where each entry is:
    [com_x, com_y, com_z, qw, qx, qy, qz] (scalar-first quaternion)

    Newton's body_com stores COM offset in body frame.
    For now, we return these with identity orientation [1,0,0,0].

    Args:
        body_com: Body center of mass positions.
        index: Link indices (count, max_links).
        tensor: Output tensor (count, max_links, 7).
    """
    ti, tj, tk = wp.tid()  # type: ignore[misc]
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
    body_com: wp.array(dtype=wp.vec3),  # type: ignore[valid-type]
    index: wp.array2d(dtype=int),  # type: ignore[valid-type]
    # outputs
    tensor: wp.array3d(dtype=float),  # type: ignore[valid-type]
) -> None:
    """Get only link COM positions, leaving orientation untouched in output tensor.

    Updates only first 3 elements of tensor: [com_x, com_y, com_z]
    Elements 3-6 (orientation) are left unchanged.

    Args:
        body_com: Body center of mass positions.
        index: Link indices (count, max_links).
        tensor: Output tensor (count, max_links, 7).
    """
    ti, tj, tk = wp.tid()  # type: ignore[misc]
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
    tensor: wp.array3d(dtype=float),  # type: ignore[valid-type]
    tensor_idx: Any,
    tenor_idx_mask: wp.array(dtype=int),  # type: ignore[valid-type]
    link_indices: wp.array2d(dtype=int),  # type: ignore[valid-type]
    # outputs
    com_cache: wp.array3d(dtype=float),  # type: ignore[valid-type]
) -> None:
    """Cache full link COM data (position + orientation) for later retrieval.

    Since Newton only stores position, we cache the full 7-element COM
    (position + orientation) in a separate buffer.

    Args:
        tensor: Input COM data (count, max_links, 7).
        tensor_idx: Articulation indices.
        tenor_idx_mask: Optional mask for indices.
        link_indices: Link body indices.
        com_cache: Output COM cache.
    """
    ti, tj, tk = wp.tid()  # type: ignore[misc]
    arti_id = tensor_idx[ti]
    wid = link_indices[arti_id, tj]
    if tenor_idx_mask:
        apply_data = bool(tenor_idx_mask[ti])
    else:
        apply_data = True

    if apply_data and wid >= 0:
        # Cache all 7 elements: position (3) + orientation (4)
        com_cache[arti_id, tj, tk] = tensor[arti_id, tj, tk]


wp.overload(cache_link_com, {"tensor_idx": wp.array(dtype=wp.int32)})
wp.overload(cache_link_com, {"tensor_idx": wp.array(dtype=wp.int64)})


@wp.kernel(enable_backward=False)
def set_link_com(
    tensor: wp.array3d(dtype=float),  # type: ignore[valid-type]
    tensor_idx: Any,
    tenor_idx_mask: wp.array(dtype=int),  # type: ignore[valid-type]
    link_indices: wp.array2d(dtype=int),  # type: ignore[valid-type]
    # outputs
    body_com: wp.array(dtype=wp.vec3),  # type: ignore[valid-type]
) -> None:
    """Set link COM positions from input tensor.

    Input tensor has shape (count, max_links, 7) where each entry is:
    [com_x, com_y, com_z, qw, qx, qy, qz]

    We extract the position (first 3 values) and set into body_com.
    Orientation is ignored as Newton only stores COM offset, not orientation.

    Args:
        tensor: Input COM data (count, max_links, 7).
        tensor_idx: Articulation indices.
        tenor_idx_mask: Optional mask for indices.
        link_indices: Link body indices.
        body_com: Output body COM positions.
    """
    ti, tj, tk = wp.tid()  # type: ignore[misc]
    arti_id = tensor_idx[ti]
    wid = link_indices[arti_id, tj]
    if tenor_idx_mask:
        apply_data = bool(tenor_idx_mask[ti])
    else:
        apply_data = True

    if apply_data and wid >= 0 and tk < 3:
        # Only copy position (first 3 values), ignore orientation
        if tk == 0:
            body_com[wid] = wp.vec3(tensor[arti_id, tj, 0], tensor[arti_id, tj, 1], tensor[arti_id, tj, 2])


# Overload to support both int32 and int64
wp.overload(set_link_com, {"tensor_idx": wp.array(dtype=wp.int32)})
wp.overload(set_link_com, {"tensor_idx": wp.array(dtype=wp.int64)})


@wp.kernel(enable_backward=False)
def get_dof_attributes(
    joint_attr: wp.array(dtype=wp.float32),  # type: ignore[valid-type]
    index: wp.array(dtype=int),  # type: ignore[valid-type]
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
    ti, tj = wp.tid()  # type: ignore[misc]
    dof_id = ti * max_dofs + tj
    wid = index[dof_id]
    tensor[ti, tj] = joint_attr[wid]


@wp.kernel(enable_backward=False)
def get_dof_limits(
    lower_limits: wp.array(dtype=wp.float32),  # type: ignore[valid-type]
    upper_limits: wp.array(dtype=wp.float32),  # type: ignore[valid-type]
    index: wp.array(dtype=int),  # type: ignore[valid-type]
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
    ti, tj = wp.tid()  # type: ignore[misc]
    dof_id = ti * max_dofs + tj
    wid = index[dof_id]
    tensor[ti, tj, 0] = lower_limits[wid]
    tensor[ti, tj, 1] = upper_limits[wid]


@wp.kernel(enable_backward=False)
def set_dof_limits(
    tensor: wp.array3d(dtype=float),  # type: ignore[valid-type]
    tensor_idx: Any,
    tenor_idx_mask: wp.array(dtype=int),  # type: ignore[valid-type]
    index: wp.array(dtype=int),  # type: ignore[valid-type]
    max_dofs: int,
    # outputs
    lower_limits: wp.array(dtype=wp.float32),  # type: ignore[valid-type]
    upper_limits: wp.array(dtype=wp.float32),  # type: ignore[valid-type]
) -> None:
    """Set DOF limits for articulations.

    Input shape is (count, max_dofs, 2) with [lower, upper] limits.

    Args:
        tensor: Input limits (count, max_dofs, 2).
        tensor_idx: Articulation indices.
        tenor_idx_mask: Optional mask for indices.
        index: DOF indices.
        max_dofs: Maximum DOFs per articulation.
        lower_limits: Output lower limits.
        upper_limits: Output upper limits.
    """
    ti, tj = wp.tid()  # type: ignore[misc]
    arti_id = tensor_idx[ti]
    dof_id = wp.int32(arti_id) * max_dofs + tj
    wid = index[dof_id]
    # if a mask array is provided then only apply the data for the indices that are True
    if tenor_idx_mask:
        apply_data = bool(tenor_idx_mask[ti])
    else:
        apply_data = True

    if apply_data:
        lower_limits[wid] = tensor[arti_id, tj, 0]
        upper_limits[wid] = tensor[arti_id, tj, 1]


# Overload to support both int32 and int64
wp.overload(set_dof_limits, {"tensor_idx": wp.array(dtype=wp.int32)})
wp.overload(set_dof_limits, {"tensor_idx": wp.array(dtype=wp.int64)})


@wp.kernel(enable_backward=False)
def set_body_pose(
    tensor: wp.array2d(dtype=float),  # type: ignore[valid-type]
    tensor_idx: Any,
    tenor_idx_mask: wp.array(dtype=int),  # type: ignore[valid-type]
    body_idx: wp.array(dtype=int),  # type: ignore[valid-type]
    # outputs
    body_q: wp.array(dtype=wp.transform),  # type: ignore[valid-type]
) -> None:
    """Set body poses from input tensor.

    Input shape is (count, 7): [x, y, z, qx, qy, qz, qw].

    Args:
        tensor: Input poses (count, 7).
        tensor_idx: Body indices into tensor.
        tenor_idx_mask: Optional mask for indices.
        body_idx: Body indices in model.
        body_q: Output body transforms.
    """
    tid = wp.tid()
    arti_id = tensor_idx[tid]
    wid = body_idx[arti_id]
    # if a mask array is provided then only apply the data for the indices that are True
    if tenor_idx_mask:
        apply_data = bool(tenor_idx_mask[tid])
    else:
        apply_data = True

    if apply_data:
        body_q[wid] = wp.transformation(  # type: ignore[call-overload]
            wp.vec3(tensor[arti_id][0], tensor[arti_id][1], tensor[arti_id][2]),
            wp.quat(tensor[arti_id][3], tensor[arti_id][4], tensor[arti_id][5], tensor[arti_id][6]),
        )


# Overload to support both int32 and int64
wp.overload(set_body_pose, {"tensor_idx": wp.array(dtype=wp.int32)})
wp.overload(set_body_pose, {"tensor_idx": wp.array(dtype=wp.int64)})


@wp.kernel(enable_backward=False)
def update_free_joint_coords_from_body_q(
    body_q: wp.array(dtype=wp.transform),  # type: ignore[valid-type]
    tensor_idx: Any,
    body_idx: wp.array(dtype=int),  # type: ignore[valid-type]
    joint_child: wp.array(dtype=int),  # type: ignore[valid-type]
    joint_type: wp.array(dtype=int),  # type: ignore[valid-type]
    joint_q_start: wp.array(dtype=int),  # type: ignore[valid-type]
    # outputs
    joint_q: wp.array(dtype=wp.float32),  # type: ignore[valid-type]
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
            joint_q[base_idx + 0] = p[0]  # type: ignore[index]
            joint_q[base_idx + 1] = p[1]  # type: ignore[index]
            joint_q[base_idx + 2] = p[2]  # type: ignore[index]
            joint_q[base_idx + 3] = q[0]  # type: ignore[index]
            joint_q[base_idx + 4] = q[1]  # type: ignore[index]
            joint_q[base_idx + 5] = q[2]  # type: ignore[index]
            joint_q[base_idx + 6] = q[3]  # type: ignore[index]
            break


# Overload to support both int32 and int64
wp.overload(update_free_joint_coords_from_body_q, {"tensor_idx": wp.array(dtype=wp.int32)})
wp.overload(update_free_joint_coords_from_body_q, {"tensor_idx": wp.array(dtype=wp.int64)})


@wp.kernel(enable_backward=False)
def set_body_velocity(
    tensor: wp.array2d(dtype=float),  # type: ignore[valid-type]
    tensor_idx: Any,
    tenor_idx_mask: wp.array(dtype=int),  # type: ignore[valid-type]
    body_idx: wp.array(dtype=int),  # type: ignore[valid-type]
    # outputs
    body_qd: wp.array(dtype=wp.spatial_vector),  # type: ignore[valid-type]
) -> None:
    """Set body velocities from input tensor.

    Input shape is (count, 6): [wx, wy, wz, vx, vy, vz].

    Args:
        tensor: Input velocities (count, 6).
        tensor_idx: Body indices into tensor.
        tenor_idx_mask: Optional mask for indices.
        body_idx: Body indices in model.
        body_qd: Output body velocities.
    """
    tid = wp.tid()
    arti_id = tensor_idx[tid]
    wid = body_idx[arti_id]
    # if a mask array is provided then only apply the data for the indices that are True
    if tenor_idx_mask:
        apply_data = bool(tenor_idx_mask[tid])
    else:
        apply_data = True

    if apply_data:
        # spatial_vector is [angular, linear]
        # Input tensor format: [angular_x, angular_y, angular_z, linear_x, linear_y, linear_z]
        body_qd[wid] = wp.spatial_vector(  # type: ignore[call-overload]
            wp.vec3(tensor[arti_id][0], tensor[arti_id][1], tensor[arti_id][2]),  # angular
            wp.vec3(tensor[arti_id][3], tensor[arti_id][4], tensor[arti_id][5]),  # linear
        )


# Overload to support both int32 and int64
wp.overload(set_body_velocity, {"tensor_idx": wp.array(dtype=wp.int32)})
wp.overload(set_body_velocity, {"tensor_idx": wp.array(dtype=wp.int64)})


@wp.kernel(enable_backward=False)
def set_link_mass(
    tensor: wp.array2d(dtype=float),  # type: ignore[valid-type]
    tensor_idx: Any,
    tenor_idx_mask: wp.array(dtype=int),  # type: ignore[valid-type]
    link_indices: wp.array2d(dtype=int),  # type: ignore[valid-type]
    # outputs
    body_mass: wp.array(dtype=wp.float32),  # type: ignore[valid-type]
) -> None:
    """Set link masses for articulations.

    Input shape is (count, max_links).

    Args:
        tensor: Input masses (count, max_links).
        tensor_idx: Articulation indices.
        tenor_idx_mask: Optional mask for indices.
        link_indices: Link body indices.
        body_mass: Output body masses.
    """
    ti, tj = wp.tid()  # type: ignore[misc]
    arti_id = tensor_idx[ti]
    wid = link_indices[arti_id, tj]
    if tenor_idx_mask:
        apply_data = bool(tenor_idx_mask[ti])
    else:
        apply_data = True

    if apply_data and wid >= 0:
        body_mass[wid] = tensor[arti_id, tj]


# Overload to support both int32 and int64
wp.overload(set_link_mass, {"tensor_idx": wp.array(dtype=wp.int32)})
wp.overload(set_link_mass, {"tensor_idx": wp.array(dtype=wp.int64)})


@wp.kernel(enable_backward=False)
def update_inv_mass(
    tensor_idx: Any,
    tenor_idx_mask: wp.array(dtype=int),  # type: ignore[valid-type]
    link_indices: wp.array2d(dtype=int),  # type: ignore[valid-type]
    body_mass: wp.array(dtype=wp.float32),  # type: ignore[valid-type]
    # outputs
    body_inv_mass: wp.array(dtype=wp.float32),  # type: ignore[valid-type]
) -> None:
    """Update inverse mass from mass for articulation links.

    Args:
        tensor_idx: Articulation indices.
        tenor_idx_mask: Optional mask for indices.
        link_indices: Link body indices.
        body_mass: Body masses.
        body_inv_mass: Output inverse masses.
    """
    ti, tj = wp.tid()  # type: ignore[misc]
    arti_id = tensor_idx[ti]
    wid = link_indices[arti_id, tj]
    if tenor_idx_mask:
        apply_data = bool(tenor_idx_mask[ti])
    else:
        apply_data = True

    if apply_data and wid >= 0:
        mass = body_mass[wid]
        if mass > 1e-8:
            body_inv_mass[wid] = 1.0 / mass
        else:
            body_inv_mass[wid] = 0.0


wp.overload(update_inv_mass, {"tensor_idx": wp.array(dtype=wp.int32)})
wp.overload(update_inv_mass, {"tensor_idx": wp.array(dtype=wp.int64)})


@wp.kernel(enable_backward=False)
def set_link_inertia(
    tensor: wp.array3d(dtype=float),  # type: ignore[valid-type]
    tensor_idx: Any,
    tenor_idx_mask: wp.array(dtype=int),  # type: ignore[valid-type]
    link_indices: wp.array2d(dtype=int),  # type: ignore[valid-type]
    # outputs
    body_inertia: wp.array(dtype=wp.mat33),  # type: ignore[valid-type]
) -> None:
    """Set link inertia tensors for articulations.

    Input shape is (count, max_links, 9) in row-major order.

    Args:
        tensor: Input inertias (count, max_links, 9).
        tensor_idx: Articulation indices.
        tenor_idx_mask: Optional mask for indices.
        link_indices: Link body indices.
        body_inertia: Output body inertias.
    """
    ti, tj, tk = wp.tid()  # type: ignore[misc]
    arti_id = tensor_idx[ti]
    wid = link_indices[arti_id, tj]
    if tenor_idx_mask:
        apply_data = bool(tenor_idx_mask[ti])
    else:
        apply_data = True

    if apply_data and wid >= 0:
        i = tk // 3
        j = tk % 3
        body_inertia[wid][i][j] = tensor[arti_id, tj, tk]


# Overload to support both int32 and int64
wp.overload(set_link_inertia, {"tensor_idx": wp.array(dtype=wp.int32)})
wp.overload(set_link_inertia, {"tensor_idx": wp.array(dtype=wp.int64)})


@wp.kernel(enable_backward=False)
def update_inv_inertia(
    tensor_idx: Any,
    tenor_idx_mask: wp.array(dtype=int),  # type: ignore[valid-type]
    link_indices: wp.array2d(dtype=int),  # type: ignore[valid-type]
    body_inertia: wp.array(dtype=wp.mat33),  # type: ignore[valid-type]
    # outputs
    body_inv_inertia: wp.array(dtype=wp.mat33),  # type: ignore[valid-type]
) -> None:
    """Update inverse inertia from inertia tensor.

    Args:
        tensor_idx: Articulation indices.
        tenor_idx_mask: Optional mask for indices.
        link_indices: Link body indices.
        body_inertia: Body inertia matrices.
        body_inv_inertia: Output inverse inertia matrices.
    """
    ti, tj = wp.tid()  # type: ignore[misc]
    arti_id = tensor_idx[ti]
    wid = link_indices[arti_id, tj]
    if tenor_idx_mask:
        apply_data = bool(tenor_idx_mask[ti])
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


# Overload to support both int32 and int64
wp.overload(update_inv_inertia, {"tensor_idx": wp.array(dtype=wp.int32)})
wp.overload(update_inv_inertia, {"tensor_idx": wp.array(dtype=wp.int64)})


@wp.kernel(enable_backward=False)
def update_joint_coords_from_root(
    body_q: wp.array(dtype=wp.transform),  # type: ignore[valid-type]
    arti_indices: wp.array(dtype=wp.int32),  # type: ignore[valid-type]
    arti_mask: wp.array(dtype=int),  # type: ignore[valid-type]
    root_body_indices: wp.array(dtype=int),  # type: ignore[valid-type]
    articulation_start: wp.array(dtype=int),  # type: ignore[valid-type]
    joint_q_start: wp.array(dtype=int),  # type: ignore[valid-type]
    joint_type: wp.array(dtype=int),  # type: ignore[valid-type]
    # outputs
    joint_q: wp.array(dtype=wp.float32),  # type: ignore[valid-type]
    joint_X_p: wp.array(dtype=wp.transform),  # type: ignore[valid-type]
) -> None:
    """Update joint coordinates from root body transforms.

    For floating-base articulations: copies root transform into joint_q[0:7]
    For fixed-base articulations: updates joint_X_p for the root joint

    Args:
        body_q: Body transforms.
        arti_indices: Articulation indices.
        arti_mask: Optional articulation mask.
        root_body_indices: Root body indices.
        articulation_start: Articulation start indices.
        joint_q_start: Joint coordinate start indices.
        joint_type: Joint types.
        joint_q: Output joint coordinates.
        joint_X_p: Output joint parent transforms.
    """
    tid = wp.tid()
    arti_id = arti_indices[tid]

    if arti_mask and not bool(arti_mask[tid]):
        return

    root_body_id = root_body_indices[arti_id]
    if root_body_id < 0:
        return

    root_transform = body_q[root_body_id]
    arti_start = articulation_start[arti_id]

    # Check if this is a floating base (first joint is FREE type = 4)
    if arti_start < joint_type.shape[0] and joint_type[arti_start] == 4:
        # Floating base: update joint_q[0:7] with the root transform
        q_start = joint_q_start[arti_start]
        # Store as [tx, ty, tz, qx, qy, qz, qw]
        p = wp.transform_get_translation(root_transform)
        q = wp.transform_get_rotation(root_transform)
        joint_q[q_start + 0] = p[0]  # type: ignore[index]
        joint_q[q_start + 1] = p[1]  # type: ignore[index]
        joint_q[q_start + 2] = p[2]  # type: ignore[index]
        joint_q[q_start + 3] = q[0]  # type: ignore[index]
        joint_q[q_start + 4] = q[1]  # type: ignore[index]
        joint_q[q_start + 5] = q[2]  # type: ignore[index]
        joint_q[q_start + 6] = q[3]  # type: ignore[index]
    else:
        # Fixed base: update joint_X_p for the root joint
        joint_X_p[arti_start] = root_transform


@wp.kernel(enable_backward=False)
def set_dof_attributes(
    tensor: wp.array2d(dtype=float),  # type: ignore[valid-type]
    tensor_idx: Any,
    tenor_idx_mask: wp.array(dtype=int),  # type: ignore[valid-type]
    index: wp.array(dtype=int),  # type: ignore[valid-type]
    max_dofs: int,
    # outputs
    joint_attr: wp.array(dtype=wp.float32),  # type: ignore[valid-type]
) -> None:
    """Set DOF attributes for articulations.

    Input shape is (count, max_dofs).

    Args:
        tensor: Input attributes (count, max_dofs).
        tensor_idx: Articulation indices.
        tenor_idx_mask: Optional mask for indices.
        index: DOF indices.
        max_dofs: Maximum DOFs per articulation.
        joint_attr: Output joint attributes.
    """
    ti, tj = wp.tid()  # type: ignore[misc]
    arti_id = tensor_idx[ti]
    dof_id = wp.int32(arti_id) * max_dofs + tj
    wid = index[dof_id]
    # if a mask array is provided then only apply the data for the indices that are True
    if tenor_idx_mask:
        apply_data = bool(tenor_idx_mask[ti])
    else:
        apply_data = True

    if apply_data:
        joint_attr[wid] = tensor[arti_id, tj]


# Overload to support both int32 and int64
wp.overload(set_dof_attributes, {"tensor_idx": wp.array(dtype=wp.int32)})
wp.overload(set_dof_attributes, {"tensor_idx": wp.array(dtype=wp.int64)})


# Note: set_dof_attributes_32 removed - use set_dof_attributes instead (now supports both int32 and int64)


@wp.kernel(enable_backward=False)
def assign_articulation_root_states(
    body_q: wp.array(dtype=wp.transform),  # type: ignore[valid-type]
    body_qd: wp.array(dtype=wp.spatial_vector),  # type: ignore[valid-type]
    tensor_idx: wp.array(dtype=wp.int64),  # type: ignore[valid-type]
    tenor_idx_mask: wp.array(dtype=int),  # type: ignore[valid-type]
    articulation_indices: wp.array(dtype=int),  # type: ignore[valid-type]
    body_index: wp.array(dtype=int),  # type: ignore[valid-type]
    update_fixed_base_articulations: bool,
    relative_transforms: bool,
    joint_type: wp.array(dtype=int),  # type: ignore[valid-type]
    articulation_start: wp.array(dtype=int),  # type: ignore[valid-type]
    joint_q_start: wp.array(dtype=int),  # type: ignore[valid-type]
    joint_qd_start: wp.array(dtype=int),  # type: ignore[valid-type]
    # outputs
    joint_q: wp.array(dtype=float),  # type: ignore[valid-type]
    joint_qd: wp.array(dtype=float),  # type: ignore[valid-type]
    joint_X_p: wp.array(dtype=wp.transform),  # type: ignore[valid-type]
) -> None:
    """Assign articulation root states from body transforms and velocities.

    Updates joint coordinates for floating-base or fixed-base articulations.

    Args:
        body_q: Body transforms.
        body_qd: Body velocities.
        tensor_idx: Tensor indices.
        tenor_idx_mask: Optional mask for indices.
        articulation_indices: Articulation indices.
        body_index: Body indices.
        update_fixed_base_articulations: Whether to update fixed-base articulations.
        relative_transforms: Whether transforms are relative.
        joint_type: Joint types.
        articulation_start: Articulation start indices.
        joint_q_start: Joint position start indices.
        joint_qd_start: Joint velocity start indices.
        joint_q: Output joint positions.
        joint_qd: Output joint velocities.
        joint_X_p: Output joint parent transforms.
    """
    tid = wp.tid()
    # if a mask array is provided then only apply the data for the indices that are True
    if tenor_idx_mask:
        apply_data = bool(tenor_idx_mask[tid])
    else:
        apply_data = True

    if not apply_data:
        return

    id = tensor_idx[tid]
    aid = articulation_indices[id]
    bid = body_index[id]
    root_pose = body_q[bid]
    root_vel = body_qd[bid]
    joint_start = articulation_start[aid]
    q_start = joint_q_start[joint_start]
    qd_start = joint_qd_start[joint_start]

    if joint_type[joint_start] == newton.JointType.FREE:
        if relative_transforms:
            existing_pose = wp.transform(
                wp.vec3(joint_q[q_start], joint_q[q_start + 1], joint_q[q_start + 2]),
                wp.quat(joint_q[q_start + 3], joint_q[q_start + 4], joint_q[q_start + 5], joint_q[q_start + 6]),
            )
            root_pose = root_pose * existing_pose

        for i in range(7):
            joint_q[q_start + i] = root_pose[i]
            if i < 6:
                joint_qd[qd_start + i] = root_vel[i]

    elif update_fixed_base_articulations and joint_type[joint_start] == newton.JointType.FIXED:
        if relative_transforms:
            existing_pose = joint_X_p[joint_start]
            root_pose = root_pose * existing_pose
        joint_X_p[joint_start] = root_pose


@wp.kernel(enable_backward=False)
def apply_body_forces_at_position(
    force_tensor: wp.array2d(dtype=float),  # type: ignore[valid-type]
    torque_tensor: wp.array2d(dtype=float),  # type: ignore[valid-type]
    position_tensor: wp.array2d(dtype=float),  # type: ignore[valid-type]
    tensor_idx: Any,
    tenor_idx_mask: wp.array(dtype=int),  # type: ignore[valid-type]
    body_idx: wp.array(dtype=int),  # type: ignore[valid-type]
    body_q: wp.array(dtype=wp.transform),  # type: ignore[valid-type]
    body_com: wp.array(dtype=wp.vec3),  # type: ignore[valid-type]
    is_global: bool,
    has_force: bool,
    has_torque: bool,
    has_position: bool,
    # outputs
    body_f: wp.array(dtype=wp.spatial_vector),  # type: ignore[valid-type]
) -> None:
    """Apply forces and torques to rigid bodies at specified positions.

    Args:
        force_tensor: Forces to apply, shape (count, 3).
        torque_tensor: Torques to apply, shape (count, 3).
        position_tensor: Positions where forces are applied, shape (count, 3).
        tensor_idx: Indices of bodies to update.
        tenor_idx_mask: Optional mask for indices.
        body_idx: Body indices in the model.
        body_q: Body transforms.
        body_com: Body center of mass offsets.
        is_global: Whether inputs are in global or local frame.
        has_force: Whether force tensor is provided.
        has_torque: Whether torque tensor is provided.
        has_position: Whether position tensor is provided.
        body_f: Output body forces (spatial vectors).
    """
    tid = wp.tid()

    # Check mask if provided
    if tenor_idx_mask:
        if not bool(tenor_idx_mask[tid]):
            return

    body_id_in_view = tensor_idx[tid]
    wid = body_idx[body_id_in_view]

    # Get body transform and COM
    transform = body_q[wid]
    rotation = wp.transform_get_rotation(transform)
    translation = wp.transform_get_translation(transform)
    com_offset = body_com[wid]

    # Compute world-space COM position
    com_world = translation + wp.quat_rotate(rotation, com_offset)  # type: ignore[operator]

    # Initialize force and torque in world frame
    force_world = wp.vec3(0.0, 0.0, 0.0)
    torque_world = wp.vec3(0.0, 0.0, 0.0)

    # Process force
    if has_force:
        force_local = wp.vec3(
            force_tensor[body_id_in_view, 0],
            force_tensor[body_id_in_view, 1],
            force_tensor[body_id_in_view, 2],
        )
        # Transform to global frame if needed
        if is_global:
            force_world = force_local
        else:
            force_world = wp.quat_rotate(rotation, force_local)  # type: ignore[assignment]

        # If position is specified, compute additional torque from force application point
        if has_position:
            position_local = wp.vec3(
                position_tensor[body_id_in_view, 0],
                position_tensor[body_id_in_view, 1],
                position_tensor[body_id_in_view, 2],
            )
            # Transform position to global frame
            if is_global:
                position_world = position_local
            else:
                position_world = translation + wp.quat_rotate(rotation, position_local)  # type: ignore[operator]

            # Compute torque from force application: τ = (r - r_com) × F
            r_offset = position_world - com_world
            torque_from_force = wp.cross(r_offset, force_world)
            torque_world = torque_world + torque_from_force

    # Process torque
    if has_torque:
        torque_local = wp.vec3(
            torque_tensor[body_id_in_view, 0],
            torque_tensor[body_id_in_view, 1],
            torque_tensor[body_id_in_view, 2],
        )
        # Transform to global frame if needed
        if is_global:
            torque_world = torque_world + torque_local
        else:
            torque_world = torque_world + wp.quat_rotate(rotation, torque_local)

    # Apply to body forces (set directly, don't add - forces are cleared each step)
    # spatial_vector is [linear (force), angular (torque)] - note the order!
    body_f[wid] = wp.spatial_vector(force_world, torque_world)  # type: ignore[call-overload]


# Overload to support both int32 and int64
wp.overload(apply_body_forces_at_position, {"tensor_idx": wp.array(dtype=wp.int32)})
wp.overload(apply_body_forces_at_position, {"tensor_idx": wp.array(dtype=wp.int64)})


@wp.kernel(enable_backward=False)
def apply_link_forces_at_position(
    force_tensor: wp.array3d(dtype=float),  # type: ignore[valid-type]
    torque_tensor: wp.array3d(dtype=float),  # type: ignore[valid-type]
    position_tensor: wp.array3d(dtype=float),  # type: ignore[valid-type]
    tensor_idx: Any,
    tenor_idx_mask: wp.array(dtype=int),  # type: ignore[valid-type]
    link_indices: wp.array2d(dtype=int),  # type: ignore[valid-type]
    body_q: wp.array(dtype=wp.transform),  # type: ignore[valid-type]
    body_com: wp.array(dtype=wp.vec3),  # type: ignore[valid-type]
    is_global: bool,
    has_force: bool,
    has_torque: bool,
    has_position: bool,
    # outputs
    body_f: wp.array(dtype=wp.spatial_vector),  # type: ignore[valid-type]
) -> None:
    """Apply forces and torques to articulation links at specified positions.

    Args:
        force_tensor: Forces to apply, shape (count, max_links, 3).
        torque_tensor: Torques to apply, shape (count, max_links, 3).
        position_tensor: Positions where forces are applied, shape (count, max_links, 3).
        tensor_idx: Indices of articulations to update.
        tenor_idx_mask: Optional mask for indices.
        link_indices: Link body indices, shape (count, max_links).
        body_q: Body transforms.
        body_com: Body center of mass offsets.
        is_global: Whether inputs are in global or local frame.
        has_force: Whether force tensor is provided.
        has_torque: Whether torque tensor is provided.
        has_position: Whether position tensor is provided.
        body_f: Output body forces (spatial vectors).
    """
    ti, tj = wp.tid()  # type: ignore[misc]

    # Check mask if provided
    if tenor_idx_mask:
        if not bool(tenor_idx_mask[ti]):
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
    com_world = translation + wp.quat_rotate(rotation, com_offset)  # type: ignore[operator]

    # Initialize force and torque in world frame
    force_world = wp.vec3(0.0, 0.0, 0.0)
    torque_world = wp.vec3(0.0, 0.0, 0.0)

    # Process force
    if has_force:
        force_local = wp.vec3(
            force_tensor[arti_id, tj, 0],
            force_tensor[arti_id, tj, 1],
            force_tensor[arti_id, tj, 2],
        )
        # Transform to global frame if needed
        if is_global:
            force_world = force_local
        else:
            force_world = wp.quat_rotate(rotation, force_local)  # type: ignore[assignment]

        # If position is specified, compute additional torque from force application point
        if has_position:
            position_local = wp.vec3(
                position_tensor[arti_id, tj, 0],
                position_tensor[arti_id, tj, 1],
                position_tensor[arti_id, tj, 2],
            )
            # Transform position to global frame
            if is_global:
                position_world = position_local
            else:
                position_world = translation + wp.quat_rotate(rotation, position_local)  # type: ignore[operator]

            # Compute torque from force application: τ = (r - r_com) × F
            r_offset = position_world - com_world
            torque_from_force = wp.cross(r_offset, force_world)
            torque_world = torque_world + torque_from_force

    # Process torque
    if has_torque:
        torque_local = wp.vec3(
            torque_tensor[arti_id, tj, 0],
            torque_tensor[arti_id, tj, 1],
            torque_tensor[arti_id, tj, 2],
        )
        # Transform to global frame if needed
        if is_global:
            torque_world = torque_world + torque_local
        else:
            torque_world = torque_world + wp.quat_rotate(rotation, torque_local)

    # Apply to body forces (set directly, don't add - forces are cleared each step)
    # spatial_vector is [linear (force), angular (torque)] - note the order!
    body_f[wid] = wp.spatial_vector(force_world, torque_world)  # type: ignore[call-overload]


# Overload to support both int32 and int64
wp.overload(apply_link_forces_at_position, {"tensor_idx": wp.array(dtype=wp.int32)})
wp.overload(apply_link_forces_at_position, {"tensor_idx": wp.array(dtype=wp.int64)})


@wp.kernel(enable_backward=False)
def sync_ctrl_direct_targets(
    dof_to_act: wp.array(dtype=wp.int32),  # type: ignore[valid-type]
    joint_target_pos: wp.array(dtype=wp.float32),  # type: ignore[valid-type]
    dofs_per_world: wp.int32,
    ctrls_per_world: wp.int32,
    # output
    mujoco_ctrl: wp.array(dtype=wp.float32),  # type: ignore[valid-type]
) -> None:
    """Sync joint_target_pos to control.mujoco.ctrl for CTRL_DIRECT joint actuators.

    Args:
        dof_to_act: Template DOF -> mujoco:actuator index mapping (-1 = no mapping).
        joint_target_pos: Per-DOF position targets (flat, all worlds).
        dofs_per_world: Number of DOFs per world.
        ctrls_per_world: Number of ctrl entries per world.
        mujoco_ctrl: Output control.mujoco.ctrl array (flat, all worlds).
    """
    world, dof = wp.tid()  # type: ignore[misc]
    act_idx = dof_to_act[dof]
    if act_idx < 0:
        return
    src = world * dofs_per_world + dof
    dst = world * ctrls_per_world + act_idx
    mujoco_ctrl[dst] = joint_target_pos[src]


@wp.kernel(enable_backward=False)
def sync_ctrl_direct_gains(
    dof_to_act: wp.array(dtype=wp.int32),  # type: ignore[valid-type]
    joint_target_ke: wp.array(dtype=wp.float32),  # type: ignore[valid-type]
    joint_target_kd: wp.array(dtype=wp.float32),  # type: ignore[valid-type]
    # output
    actuator_gainprm: wp.array(dtype=vec10),  # type: ignore[valid-type]
    actuator_biasprm: wp.array(dtype=vec10),  # type: ignore[valid-type]
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

    For each template DOF that has a CTRL_DIRECT joint actuator, stores the
    mujoco:actuator index. Returns None if no CTRL_DIRECT joint actuators exist.

    Args:
        model: Newton model with mujoco custom attributes.

    Returns:
        wp.array of shape (dofs_per_world,) with dtype int32, or None.
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
