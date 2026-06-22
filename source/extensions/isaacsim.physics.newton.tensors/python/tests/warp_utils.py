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

"""Provides Warp array construction helpers for Newton tensor tests. Covers deterministic arange and linspace arrays on CPU or CUDA devices."""

from __future__ import annotations

import warp as wp


@wp.kernel
def _arange_k(a: wp.array(dtype=wp.int32)) -> None:
    tid = wp.tid()
    a[tid] = tid


def arange(n: int, device: str = "cpu") -> wp.array:
    """Return a Warp int32 array with values ``[0, 1, ..., n - 1]`` on ``device``.

    Args:
        n: Number of values to generate.
        device: Warp device for the output array.

    Returns:
        Warp int32 array of length ``n``.
    """
    a = wp.empty(n, dtype=wp.int32, device=device)
    wp.launch(kernel=_arange_k, dim=n, inputs=[a], device=device)
    wp.synchronize()
    return a


@wp.kernel
def _linspace_k(a: wp.array(dtype=wp.float32), offset: wp.float32, step: wp.float32) -> None:
    tid = wp.tid()
    a[tid] = offset + float(tid) * step


def linspace(
    n: int,
    start: float,
    end: float,
    include_end: bool = False,
    include_start: bool = True,
    device: str = "cpu",
) -> wp.array:
    """Return a Warp float32 array of ``n`` values spaced between ``start`` and ``end``.

    Endpoint inclusion is controlled independently by ``include_start`` and
    ``include_end``. The spacing is adjusted so that ``n`` values always fit
    in the selected open/closed interval.

    Args:
        n: Number of values to generate.
        start: Interval start value.
        end: Interval end value.
        include_end: Whether the last value is exactly ``end``.
        include_start: Whether the first value is exactly ``start``.
        device: Warp device for the output array.

    Returns:
        Warp float32 array of length ``n``.
    """
    d = n - 1
    if not include_start:
        d += 1
    if not include_end:
        d += 1

    step = (end - start) / d
    if not include_start:
        offset = start + step
    else:
        offset = start

    a = wp.empty(n, dtype=wp.float32, device=device)
    wp.launch(kernel=_linspace_k, dim=n, inputs=[a, offset, step], device=device)
    wp.synchronize()
    return a


@wp.kernel
def _fill_float32_k(a: wp.array(dtype=wp.float32), value: wp.float32) -> None:
    tid = wp.tid()
    a[tid] = value


def fill_float32(n: int, value: float = 0.0, device: str = "cpu") -> wp.array:
    """Return a Warp float32 array of length ``n`` filled with ``value`` on ``device``.

    Args:
        n: Number of values to allocate.
        value: Value to write into each entry.
        device: Warp device for the output array.

    Returns:
        Warp float32 array filled with ``value``.
    """
    a = wp.empty(n, dtype=wp.float32, device=device)
    wp.launch(kernel=_fill_float32_k, dim=n, inputs=[a, value], device=device)
    wp.synchronize()
    return a


@wp.kernel
def _fill_vec3_k(a: wp.array(dtype=wp.vec3), value: wp.vec3) -> None:
    tid = wp.tid()
    a[tid] = value


def fill_vec3(n: int, value: wp.vec3 = wp.vec3(0.0, 0.0, 0.0), device: str = "cpu") -> wp.array:
    """Return a Warp vec3 array of length ``n`` filled with ``value`` on ``device``.

    Args:
        n: Number of values to allocate.
        value: Vector value to write into each entry.
        device: Warp device for the output array.

    Returns:
        Warp vec3 array filled with ``value``.
    """
    a = wp.empty(n, dtype=wp.vec3, device=device)
    wp.launch(kernel=_fill_vec3_k, dim=n, inputs=[a, value], device=device)
    wp.synchronize()
    return a


@wp.kernel
def _compute_dof_forces_k(
    pos: wp.array(dtype=float, ndim=2),
    vel: wp.array(dtype=float, ndim=2),
    force: wp.array(dtype=float, ndim=2),
    stiffness: float,
    damping: float,
) -> None:
    i, j = wp.tid()
    pos_target = 0.0
    force[i, j] = stiffness * (pos_target - pos[i, j]) - damping * vel[i, j]


def compute_dof_forces(
    pos: wp.array,
    vel: wp.array,
    force: wp.array,
    stiffness: float,
    damping: float,
    device: str = "cpu",
) -> None:
    """Write PD control forces into ``force`` using a target of zero.

    Computes ``force[i, j] = stiffness * (-pos[i, j]) - damping * vel[i, j]``
    for every element of ``force``.

    Args:
        pos: DOF position array.
        vel: DOF velocity array.
        force: Output DOF force array to write.
        stiffness: Proportional gain.
        damping: Derivative gain.
        device: Warp device used to launch the kernel.
    """
    wp.launch(
        kernel=_compute_dof_forces_k, dim=force.shape, inputs=[pos, vel, force, stiffness, damping], device=device
    )
    wp.synchronize()
