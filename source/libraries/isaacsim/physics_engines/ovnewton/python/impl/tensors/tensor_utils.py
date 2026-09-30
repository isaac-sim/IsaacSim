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

"""Provide tensor conversion utilities for the Newton tensor interface.

This module provides helper functions to convert between different tensor formats
(PyTorch, NumPy, Warp) for use in Newton physics kernels.
"""

from __future__ import annotations

from importlib import import_module as _legacy_import_module
from typing import TYPE_CHECKING, Any

import numpy as np
import warp as wp

if TYPE_CHECKING:
    import torch


def import_module(name: str) -> Any:
    """Import an optional tensor frontend module.

    Args:
        name: Fully qualified module name.

    Returns:
        Imported module, or a placeholder whose missing attributes resolve to None when the module is unavailable.

    """
    try:
        return _legacy_import_module(name)
    except ImportError:
        return type("_Missing", (), {"__getattr__": lambda self, attr: None})()


def convert_to_warp(tensor: wp.array | torch.Tensor | np.ndarray, device: str) -> wp.array | None:
    """Prepare a tensor for use as Warp kernel output.

    Args:
        tensor: Frontend tensor to expose to Warp.
        device: Device for a converted NumPy array.

    Returns:
        Warp array suitable for kernel output, or None when the frontend type is unsupported.

    """
    if isinstance(tensor, wp.array):
        return tensor

    elif isinstance(tensor, np.ndarray):
        tensor_cont = np.ascontiguousarray(tensor)
        if np.issubdtype(tensor_cont.dtype, np.floating):
            warp_dtype = wp.float32
        elif np.issubdtype(tensor_cont.dtype, np.integer):
            warp_dtype = wp.int32
        else:
            warp_dtype = wp.float32
        return wp.array(tensor_cont, dtype=warp_dtype, device=str(device), copy=False)

    torch = import_module("torch")
    if isinstance(tensor, torch.Tensor):
        return wp.from_torch(tensor)

    return None


def wrap_input_tensor(
    tensor: wp.array | torch.Tensor | np.ndarray, device: str, dtype: type | None = None
) -> wp.array | None:
    """Wrap an input tensor as a Warp array for kernel input.

    Args:
        tensor: Frontend tensor to wrap.
        device: Device for the wrapped tensor.
        dtype: Requested Warp data type, or None to infer it from the input.

    Returns:
        Warp array suitable for kernel input, or None when the frontend type is unsupported.

    """
    if isinstance(tensor, wp.array):
        if dtype is not None and tensor.dtype != dtype:
            return wp.array(tensor, dtype=dtype, device=tensor.device)
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

        return wp.from_torch(tensor_cont, dtype=dtype)

    return None


def move_tensor_to_cpu(tensor: wp.array | torch.Tensor | np.ndarray) -> wp.array | torch.Tensor | np.ndarray | None:
    """Move a tensor to CPU while preserving the frontend type.

    Args:
        tensor: Frontend tensor to move.

    Returns:
        Tensor on the CPU while preserving its frontend, or None when the frontend type is unsupported.

    """
    if isinstance(tensor, wp.array):
        if tensor.device.is_cpu:
            return tensor
        return tensor.to("cpu")
    elif isinstance(tensor, np.ndarray):
        return tensor
    torch = import_module("torch")
    if isinstance(tensor, torch.Tensor):
        return tensor.cpu()
    return None


def zero_tensor(tensor: wp.array | torch.Tensor | np.ndarray) -> None:
    """Fill a supported frontend tensor with zeros.

    Args:
        tensor: Tensor to clear.

    """
    if hasattr(tensor, "zero_"):
        tensor.zero_()
    elif hasattr(tensor, "fill_"):
        tensor.fill_(0.0)
    else:
        tensor[:] = 0.0
