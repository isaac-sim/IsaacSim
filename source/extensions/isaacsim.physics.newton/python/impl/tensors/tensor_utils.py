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

"""Tensor conversion utilities for Newton physics tensor interface.

This module provides helper functions to convert between different tensor formats
(PyTorch, NumPy, Warp) for use in Newton physics kernels.
"""

from __future__ import annotations

import numpy as np
import warp as wp
from isaacsim.core.deprecation_manager import import_module


def convert_to_warp(tensor: wp.array | "torch.Tensor" | np.ndarray, device: str) -> wp.array | None:  # type: ignore[name-defined]
    """Prepare a tensor for use as warp kernel output.

    Args:
        tensor: The frontend tensor (torch.Tensor, np.ndarray, or wp.array).
        device: Device string for the frontend.

    Returns:
        Warp array suitable for kernel output, or None if conversion fails.
    """
    if isinstance(tensor, wp.array):
        return tensor

    elif isinstance(tensor, np.ndarray):
        tensor_cont = np.ascontiguousarray(tensor)
        if np.issubdtype(tensor_cont.dtype, np.floating):
            warp_dtype = wp.float32  # type: ignore[assignment]
        elif np.issubdtype(tensor_cont.dtype, np.integer):
            warp_dtype = wp.int32  # type: ignore[assignment]
        else:
            warp_dtype = wp.float32  # type: ignore[assignment]
        return wp.array(tensor_cont, dtype=warp_dtype, device=str(device), copy=False)

    torch = import_module("torch")
    if isinstance(tensor, torch.Tensor):  # type: ignore[name-defined]
        return wp.from_torch(tensor)  # type: ignore[return-value]

    return None


def wrap_input_tensor(
    tensor: wp.array | "torch.Tensor" | np.ndarray, device: str, dtype: type | None = None  # type: ignore[name-defined]
) -> wp.array | None:
    """Wrap an input tensor as a warp array for kernel input.

    Args:
        tensor: Input tensor (torch.Tensor, np.ndarray, or wp.array).
        device: Device string for the frontend.
        dtype: Target warp dtype (optional, will be inferred if not specified).

    Returns:
        Warp array suitable for kernel input, or None if conversion fails.
    """
    if isinstance(tensor, wp.array):
        if dtype is not None and tensor.dtype != dtype:
            return wp.array(tensor, dtype=dtype, device=tensor.device)  # type: ignore[arg-type]
        return tensor

    elif isinstance(tensor, np.ndarray):
        tensor_cont = np.ascontiguousarray(tensor)
        if dtype is None:
            if np.issubdtype(tensor_cont.dtype, np.integer):
                dtype = wp.int64
            else:
                dtype = wp.float32
        return wp.array(tensor_cont, dtype=dtype, device=str(device), copy=False)

    torch = import_module("torch")
    if isinstance(tensor, torch.Tensor):
        target_device = "cuda" if "cuda" in str(device) else "cpu"
        if tensor.device.type != target_device:
            tensor = tensor.to(target_device)
        tensor_cont = tensor.contiguous()

        if dtype is None:
            if tensor_cont.dtype in (torch.int32, torch.int64):
                dtype = wp.int64
            elif tensor_cont.dtype == torch.uint8:
                dtype = wp.uint8
            elif tensor_cont.dtype == torch.float32:
                dtype = wp.float32
            else:
                dtype = wp.float32

        if dtype == wp.int64 and tensor_cont.dtype == torch.int32:
            tensor_cont = tensor_cont.to(torch.int64)
        elif dtype == wp.int32 and tensor_cont.dtype == torch.int64:
            tensor_cont = tensor_cont.to(torch.int32)
        elif dtype == wp.int32 and tensor_cont.dtype == torch.int32:
            pass

        return wp.from_torch(tensor_cont, dtype=dtype)  # type: ignore[return-value]

    return None


def move_tensor_to_cpu(tensor: wp.array | "torch.Tensor" | np.ndarray) -> "wp.array | torch.Tensor | np.ndarray | None":  # type: ignore[name-defined]
    """Move a tensor to CPU while preserving the frontend type.

    Args:
        tensor: Input tensor (torch.Tensor, np.ndarray, or wp.array).

    Returns:
        Tensor on CPU, or None if conversion fails.
    """
    if isinstance(tensor, wp.array):
        if tensor.device.is_cpu:  # type: ignore[union-attr]
            return tensor
        return tensor.to("cpu")
    elif isinstance(tensor, np.ndarray):
        return tensor
    torch = import_module("torch")
    if isinstance(tensor, torch.Tensor):
        return tensor.cpu()
    return None


def zero_tensor(tensor: wp.array | "torch.Tensor" | np.ndarray) -> None:  # type: ignore[name-defined]
    """Zero out a tensor in a backend-agnostic way.

    Args:
        tensor: Tensor to zero (torch.Tensor, np.ndarray, or wp.array).
    """
    if hasattr(tensor, "zero_"):
        tensor.zero_()
    elif hasattr(tensor, "fill_"):
        tensor.fill_(0.0)
    else:
        tensor[:] = 0.0
