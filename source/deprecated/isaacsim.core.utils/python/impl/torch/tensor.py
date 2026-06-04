# SPDX-FileCopyrightText: Copyright (c) 2022-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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

"""Utility functions for PyTorch tensor operations, data type conversion, and device management."""

from __future__ import annotations

from typing import Any

from isaacsim.core.deprecation_manager import import_module

torch = import_module("torch")


def as_type(data: object, dtype: str) -> Any:
    """Convert tensor data to the specified data type.

    Args:
        data: The tensor data to convert.
        dtype: Target data type string ("float32", "bool", "int32", "int64", "long", "uint8").

    Returns:
        The tensor converted to the specified data type, or None if the data type is not supported.
    """
    if dtype == "float32":
        return data.to(torch.float32)
    elif dtype == "bool":
        return data.to(torch.bool)
    elif dtype == "int32":
        return data.to(torch.int32)
    elif dtype == "int64":
        return data.to(torch.int64)
    elif dtype == "long":
        return data.to(torch.long)
    elif dtype == "uint8":
        return data.to(torch.uint8)
    else:
        print(f"Type {dtype} not supported.")


def convert(data: object, device: object, dtype: str = "float32", indexed: object = None) -> Any:
    """Convert data to tensor format with specified device and data type.

    Args:
        data: Input data to convert. Can be tensor or array-like data.
        device: Target device for the tensor.
        dtype: Target data type for the tensor.
        indexed: Additional indexing parameter (currently unused).

    Returns:
        The converted tensor on the specified device with the specified data type.
    """
    if not isinstance(data, torch.Tensor):
        return as_type(torch.tensor(data, device=device), dtype)
    else:
        return as_type(data.to(device=device), dtype)


def create_zeros_tensor(shape: object, dtype: str, device: object = None) -> Any:
    """Create a tensor filled with zeros of the specified shape and data type.

    Args:
        shape: Shape of the tensor to create.
        dtype: Data type for the tensor elements.
        device: Target device for the tensor.

    Returns:
        A zero-filled tensor with the specified shape, data type, and device.
    """
    return as_type(torch.zeros(shape, device=device), dtype)


def create_tensor_from_list(data: list, dtype: str, device: object = None) -> Any:
    """Create a tensor from list data with specified data type and device.

    Args:
        data: List data to convert to tensor.
        dtype: Target data type for the tensor.
        device: Target device for the tensor.

    Returns:
        A tensor created from the list data with the specified data type and device.
    """
    return as_type(torch.tensor(data, device=device), dtype=dtype)


def clone_tensor(data: object, device: object) -> Any:
    """Clone tensor data to the specified device.

    Args:
        data: The tensor data to clone.
        device: Target device for the cloned tensor.

    Returns:
        A cloned copy of the tensor on the specified device.
    """
    data = data.to(device=device)
    return torch.clone(data)


def resolve_indices(indices: object, count: int, device: object) -> Any:
    """Resolve and convert indices to a proper tensor format.

    Args:
        indices: Input indices. Can be a list, tensor, or None. If None, creates a range from 0 to count-1.
        count: Total count for creating default indices when indices is None.
        device: Target device for the indices tensor.

    Returns:
        A long tensor containing the resolved indices on the specified device.
    """
    result = indices
    if isinstance(indices, list):
        result = torch.tensor(indices, dtype=torch.long, device=device)
    if indices is None:
        result = torch.arange(count, device=device)
    return result.to(dtype=torch.long, device=device)


def move_data(data: object, device: object) -> Any:
    """Move tensor data to the specified device.

    Args:
        data: The tensor data to move.
        device: Target device for the tensor.

    Returns:
        The tensor moved to the specified device.
    """
    return data.to(device=device)


def tensor_cat(data: object, device: object = None, dim: int = -1) -> Any:
    """Concatenates tensors along a specified dimension.

    Args:
        data: Sequence of tensors to concatenate.
        device: Target device for the operation.
        dim: Dimension along which to concatenate tensors.

    Returns:
        Concatenated tensor.
    """
    return torch.cat(data, dim=dim)


def expand_dims(data: object, axis: int) -> Any:
    """Add a new dimension to the tensor at the specified axis.

    Args:
        data: Input tensor data.
        axis: Axis position where to insert the new dimension.

    Returns:
        The tensor with an additional dimension at the specified axis.
    """
    return torch.unsqueeze(data, axis)


def pad(data: object, pad_width: object, mode: str = "constant", value: object = None) -> Any:
    """Add padding to tensor data.

    Args:
        data: Input tensor data to pad.
        pad_width: Padding specification. Can be a tuple or list of tuples specifying padding for each dimension.
        mode: Padding mode to use.
        value: Fill value for constant padding mode.

    Returns:
        The padded tensor.
    """
    if len(pad_width) == 2 and isinstance(pad_width[0], tuple):
        pad_width = pad_width[1] + pad_width[0]
    return torch.nn.functional.pad(data, pad_width, mode, value)


def tensor_stack(data: object, dim: int = 0) -> Any:
    """Stacks tensors along a new dimension.

    Args:
        data: Sequence of tensors to stack.
        dim: Dimension along which to stack tensors.

    Returns:
        Stacked tensor.
    """
    return torch.stack(data, dim=dim)


def to_list(data: object) -> list:
    """Converts tensor data to a Python list.

    Args:
        data: Data to convert. If already a list, returns as-is.

    Returns:
        Python list representation of the data.
    """
    if not isinstance(data, list):
        return data.cpu().numpy().tolist()
    return data


def to_numpy(data: object) -> Any:
    """Converts tensor data to a NumPy array.

    Args:
        data: Data to convert. If not a tensor, returns as-is.

    Returns:
        NumPy array representation of the data.
    """
    if isinstance(data, torch.Tensor):
        return data.cpu().numpy()
    return data


def assign(src: object, dst: object, indices: object) -> Any:
    """Assign source values to destination tensor at specified indices.

    Args:
        src: Source values to assign.
        dst: Destination tensor to modify.
        indices: Indices where to assign the values. Can be a list of coordinates or tensor indices.

    Returns:
        The modified destination tensor.
    """
    if isinstance(indices, list):
        dst[tuple(indices)] = src
    else:
        dst[indices] = src
    return dst
