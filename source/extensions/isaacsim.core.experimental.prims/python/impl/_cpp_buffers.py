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

import warp as wp
from isaacsim.core.simulation_manager import SimulationManager


def get_device_ordinal() -> int:
    """Get the simulation device ordinal (-1 for CPU, >=0 for GPU).

    Returns:
        The device ordinal.
    """
    device = SimulationManager.get_device()
    if device.is_cuda:
        return device.ordinal
    return -1


def wrap_cpp_buffer(view: object, field_name: str, shape: tuple, dtype: type = wp.float32) -> wp.array:
    """Create a wp.array that wraps a C++ view's buffer (zero-copy).

    Args:
        view: A typed view object (IXformDataView, IRigidBodyDataView,
              or IArticulationDataView).
        field_name: Name of the data field (e.g., "dof_positions").
        shape: Shape of the array.
        dtype: Warp data type.

    Returns:
        wp.array wrapping C++ memory. C++ owns the memory; Python must not free it.
    """
    ptr = view.get_buffer_ptr(field_name)
    device_ordinal = view.get_buffer_device()
    device = f"cuda:{device_ordinal}" if device_ordinal >= 0 else "cpu"
    elem_size = wp.types.type_size_in_bytes(dtype)
    total_elems = 1
    for d in shape:
        total_elems *= d
    capacity = total_elems * elem_size
    return wp.array(ptr=ptr, shape=shape, dtype=dtype, device=device, capacity=capacity, deleter=None)
