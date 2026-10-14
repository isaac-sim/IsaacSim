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

"""Compute articulation tensors and gather Newton model data into tensor-view order."""

import types
from typing import Any

import newton
import warp as wp


@wp.kernel
def wrangle_matrix_kernel(
    m_warp: wp.array3d[float],
    arti_indices: wp.array[int],
    n_arti: int,
    n_rows: int,
    n_cols: int,
    m_tensor: wp.array3d[float],
) -> None:
    """Gather articulation matrices into tensor-view order.

    Args:
        m_warp: Matrices in model articulation order.
        arti_indices: Selected model articulation indices.
        n_arti: Number of selected articulations.
        n_rows: Rows per matrix.
        n_cols: Columns per matrix.
        m_tensor: Destination matrices.
    """
    art_idx = wp.tid()
    if art_idx >= n_arti:
        return
    orig_id = arti_indices[art_idx]
    for i in range(n_rows):
        for j in range(n_cols):
            m_tensor[art_idx, i, j] = m_warp[orig_id, i, j]


def wrangle_matrix(
    m_warp: wp.array, m_tensor: wp.array | None, arti_indices: wp.array, n_rows: int, n_cols: int, device: str
) -> wp.array:
    """Compact newton's matrix into what tensor API wants. Newton's matrix dimension is containing all articulations in the scene.

    Args:
        m_warp: matrix from newton
        m_tensor: matrix in the tensor format
        arti_indices: indices for selecting the articulation from the newton matrix
        n_rows: number of rows of the tensor matrix
        n_cols: number of cols of the tensor matrix
        device: device to allocate the buffer
    Returns:
        The matrix in the tensor format.
    """
    n_arti = arti_indices.shape[0]
    if not m_tensor or m_tensor.shape != (n_arti, n_rows, n_cols):
        m_tensor = wp.array(shape=(n_arti, n_rows, n_cols), dtype=wp.float32, device=device)

    wp.launch(
        kernel=wrangle_matrix_kernel,
        dim=n_arti,
        inputs=[
            m_warp,
            arti_indices,
            n_arti,
            n_rows,
            n_cols,
        ],
        outputs=[m_tensor],
        device=device,
    )
    return m_tensor


def allocate_buffer(m_buffer: wp.array | None, buffer_size: tuple | int, dtype: type, device: str) -> wp.array:
    """Allocate a buffer for Newton calculations.

    Args:
        m_buffer: buffer used for calculation.
        buffer_size: size for the buffer
        dtype: Warp element type.
        device: device to allocate the buffer
    Returns:
        The buffer.
    """
    if isinstance(buffer_size, int):
        buffer_size = (buffer_size,)
    # We only check size, not the device
    if not m_buffer or m_buffer.shape != buffer_size:
        m_buffer = wp.array(shape=buffer_size, dtype=dtype, device=device)
    return m_buffer


@wp.kernel
def gather_compensation_forces(
    forces: wp.array[float],
    articulation_start: wp.array[int],
    joint_qd_start: wp.array[int],
    articulation_indices: wp.array[int],
    output: wp.array2d[float],
) -> None:
    """Gather selected articulations' generalized forces, including their floating-root DOFs.

    Args:
        forces: Generalized forces in model DOF order.
        articulation_start: First joint index for each articulation.
        joint_qd_start: First velocity DOF index for each joint.
        articulation_indices: Articulations in view order.
        output: Destination force tensor.
    """
    row, column = wp.tid()
    joint = articulation_start[articulation_indices[row]]
    output[row, column] = forces[joint_qd_start[joint] + column]


def compute_compensation_forces(
    model: Any,
    state: Any,
    articulation_indices: wp.array,
    mask: wp.array,
    num_dofs: int,
    gravity: bool,
    cache: types.SimpleNamespace | None,
) -> types.SimpleNamespace:
    """Compute gravity or Coriolis compensation without modifying the live simulation state.

    Args:
        model: Newton model defining topology and inertial properties.
        state: Current Newton state supplying joint positions and velocities.
        articulation_indices: Model articulation indices in tensor-view order.
        mask: Mask selecting the view's articulations for inverse dynamics.
        num_dofs: Generalized DOFs per articulation, including six floating-root DOFs when present.
        gravity: Compute gravity compensation when true, otherwise Coriolis and centrifugal compensation.
        cache: View-owned scratch state and output arrays, or None on the first call.

    Returns:
        Scratch cache with the computed tensor in its ``tensor`` field.
    """
    if cache is None:
        cache = types.SimpleNamespace(
            state=model.state(),
            forces=wp.zeros(model.joint_dof_count, dtype=wp.float32, device=model.device),
            tensor=wp.empty((articulation_indices.shape[0], num_dofs), dtype=wp.float32, device=model.device),
        )
    # Joint setters can leave body transforms stale until the next simulation step. Evaluate FK into a
    # private state, preserving the live body's transforms, velocities, forces, and integration state.
    wp.copy(cache.state.joint_q, state.joint_q)
    wp.copy(cache.state.joint_qd, state.joint_qd)
    newton.eval_fk(model, cache.state.joint_q, cache.state.joint_qd, cache.state, mask=mask)
    newton.eval_inverse_dynamics_passive(
        model,
        cache.state,
        gravity_force=cache.forces if gravity else None,
        coriolis_force=None if gravity else cache.forces,
        mask=mask,
    )
    wp.launch(
        gather_compensation_forces,
        dim=cache.tensor.shape,
        inputs=[cache.forces, model.articulation_start, model.joint_qd_start, articulation_indices],
        outputs=[cache.tensor],
        device=model.device,
    )
    return cache
