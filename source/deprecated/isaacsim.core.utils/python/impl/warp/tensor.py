# SPDX-FileCopyrightText: Copyright (c) 2023-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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

"""Utility functions for working with Warp tensors and arrays, including data type conversion, device management, and array operations."""

from __future__ import annotations

from typing import Any

import numpy as np
import warp as wp
from isaacsim.core.deprecation_manager import import_module

torch = import_module("torch")


def get_type(dtype: str) -> Any:
    """Convert string data type to Warp data type.

    Maps string representations of data types to their corresponding Warp data type constants.

    Args:
        dtype: String representation of the data type.

    Returns:
        Corresponding Warp data type constant.
    """
    if dtype == "float32":
        return wp.float32
    elif dtype == "bool":
        return wp.uint8
    elif dtype == "int32":
        return wp.int32
    elif dtype == "int64":
        return wp.int64
    elif dtype == "long":
        return wp.int64
    elif dtype == "uint8":
        return wp.uint8
    else:
        raise ValueError(f"Type {dtype} not supported.")


def convert(data: object, device: object, dtype: str = "float32", indexed: bool = False) -> wp.array | wp.indexedarray:
    """Convert data to Warp array format with specified properties.

    Converts input data to a Warp array with the specified device, data type, and indexing mode.

    Args:
        data: Input data to convert.
        device: Target device for the converted array.
        dtype: Data type for the converted array.
        indexed: Whether to return an indexed array.

    Returns:
        Converted Warp array or indexed array.
    """
    arr = None
    if not isinstance(data, wp.array) and not isinstance(data, wp.indexedarray):
        arr = wp.array(data, dtype=get_type(dtype), device=device)
    else:
        arr = data.to(device)
    if indexed and not isinstance(arr, wp.indexedarray):
        return wp.indexedarray(arr, [None])
    else:
        return arr


def create_zeros_tensor(shape: object, dtype: str, device: object = None) -> wp.array:
    """Create a Warp array filled with zeros.

    Creates a new Warp array with the specified shape and data type, initialized with zero values.

    Args:
        shape: Shape of the array to create.
        dtype: Data type for the array elements.
        device: Target device for the array.

    Returns:
        Warp array filled with zeros.
    """
    return wp.zeros(shape=tuple(shape), device=device, dtype=get_type(dtype))


def create_tensor_from_list(data: list, dtype: str, device: object = None) -> wp.array:
    """Create a Warp array from a list of data.

    Creates a Warp array from the provided list data with the specified data type and device.

    Args:
        data: List of data to convert to array.
        dtype: Data type for the array elements.
        device: Target device for the array.

    Returns:
        Warp array created from the list data.
    """
    return wp.array(data, device=device, dtype=get_type(dtype))


def clone_tensor(data: object, device: object) -> wp.array:
    """Create a copy of the array on the specified device.

    Creates a deep copy of the input array and moves it to the target device.

    Args:
        data: Input array to clone.
        device: Target device for the cloned array.

    Returns:
        Cloned array on the specified device.
    """
    data = data.to(device)
    cloned_data = wp.zeros_like(data)
    wp.copy(cloned_data, data)
    wp.synchronize_device(device)

    return cloned_data


@wp.kernel
def _arange_k(a: wp.array(dtype=wp.int32)) -> None:
    """Warp kernel that fills an array with sequential indices.

    Args:
        a: Int32 array to fill with sequential values from 0 to array length-1.
    """
    tid = wp.tid()
    a[tid] = tid


def arange(n: int, device: str = "cpu") -> wp.array:
    """Creates an array with sequential integer values from 0 to n-1.

    Args:
        n: Number of elements in the array.
        device: Device to create the array on.

    Returns:
        Int32 array containing sequential values from 0 to n-1.
    """
    a = wp.empty(n, dtype=wp.int32, device=device)
    wp.launch(kernel=_arange_k, dim=n, inputs=[a], device=device)
    return a


global_arange = {}


def resolve_indices(indices: list[int] | wp.array | None, count: int, device: str) -> wp.array:
    """Resolve indices into a Warp array.

    If indices is a list, converts it to a Warp array. If indices is None, generates an array
    of consecutive integers from 0 to count-1. Otherwise, moves the existing array to the
    specified device.

    Args:
        indices: List of indices, existing Warp array, or None to generate consecutive indices.
        count: Number of indices to generate when indices is None.
        device: Target device for the resulting array.

    Returns:
        Warp array containing the resolved indices.
    """
    if isinstance(indices, list):
        result = wp.array(indices, dtype=wp.int32, device=device)
    elif indices is None:
        if count not in global_arange:
            result = arange(count, device="cuda")  # TODO: warp kernels not working on cpu
            global_arange[count] = result
        else:
            result = global_arange[count]
    else:
        result = indices.to(device)
    result = result.to(device)
    return result


def move_data(data: object, device: object) -> wp.array | wp.indexedarray:
    """Move array data to the specified device.

    Transfers array data to the target device, handling both regular and indexed arrays.

    Args:
        data: Input array to move.
        device: Target device for the data.

    Returns:
        Array moved to the specified device.
    """
    if isinstance(data, wp.array):
        return data.to(device)
    elif isinstance(data, wp.indexedarray):
        if str(device) != str(data.device):
            return wp.indexedarray(data.contiguous().to(device), indices=[None])
        else:
            return data.to(device)
    else:
        raise TypeError(f"Unsupported data type {type(data)} for move_data. Expected wp.array or wp.indexedarray.")


def tensor_cat(data: list, device: str | None = None, dim: int = -1) -> wp.array:
    """Concatenate Warp arrays or indexed arrays along a specified dimension.

    Converts Warp arrays to PyTorch tensors, performs concatenation, then converts back
    to a Warp array.

    Args:
        data: List of Warp arrays or indexed arrays to concatenate.
        device: Target device for the operation.
        dim: Dimension along which to concatenate.

    Returns:
        Concatenated Warp array.
    """
    for i, d in enumerate(data):
        if isinstance(d, wp.array):
            data[i] = wp.to_torch(d)
        elif isinstance(d, wp.indexedarray):
            data[i] = torch.tensor(d.numpy(), device=device)
    torch_cat = torch.cat(data, dim=dim)
    return wp.from_torch(torch_cat)


def expand_dims(data: object, axis: int) -> wp.array:
    """Add a new dimension to the array at the specified axis.

    Expands the dimensions of the input array by inserting a new axis at the specified position.

    Args:
        data: Input array to expand.
        axis: Position where the new axis is placed.

    Returns:
        Array with expanded dimensions.
    """
    if isinstance(data, wp.array):
        data = wp.to_torch(data)
    data_torch = torch.unsqueeze(data, axis)
    dtype = wp.int32 if data_torch.dtype in (torch.int32, torch.long) else wp.float32
    return wp.from_torch(data_torch, dtype=dtype)


def to_list(data: wp.array | wp.indexedarray | "torch.Tensor" | Any) -> list:
    """Convert data to a Python list.

    Handles conversion from Warp arrays, indexed arrays, and PyTorch tensors to Python lists.
    Returns the input unchanged if it's already in a compatible format.

    Args:
        data: Data to convert to a list.

    Returns:
        Python list representation of the data.
    """
    if isinstance(data, wp.array):
        return data.numpy().tolist()
    elif isinstance(data, wp.indexedarray):
        return data.numpy().tolist()
    elif isinstance(data, torch.Tensor):
        return data.cpu().numpy().tolist()
    return data


def to_numpy(data: Any) -> np.ndarray | Any:
    """Convert data to a NumPy array.

    Attempts to convert the input data to a NumPy array using the numpy() method.
    Returns the original data if conversion fails.

    Args:
        data: Data to convert to NumPy array.

    Returns:
        NumPy array if conversion succeeds, otherwise the original data.
    """
    try:
        return data.numpy()
    except Exception:
        return data


@wp.kernel
def _assign11(src: Any, dst: wp.array(dtype=Any), indices: wp.array(dtype=int)) -> None:
    """Warp kernel that assigns 1D source values to destination array at specified indices.

    Args:
        src: Source values to assign.
        dst: Destination array to receive values.
        indices: Array of indices specifying where to place each source value.
    """
    tid = wp.tid()
    idx = indices[tid]
    dst[idx] = src[tid]


wp.overload(_assign11, {"src": wp.array(dtype=float), "dst": wp.array(dtype=float)})
wp.overload(_assign11, {"src": wp.indexedarray(dtype=float), "dst": wp.array(dtype=float)})


@wp.kernel
def _assign12(src: Any, dst: wp.array(dtype=Any, ndim=2), indices: wp.array(dtype=int)) -> None:
    """Warp kernel that assigns 2D source values to destination array at specified row indices.

    Args:
        src: Source 2D array values to assign.
        dst: Destination 2D array to receive values.
        indices: Array of row indices specifying where to place each source row.
    """
    i, j = wp.tid()
    idx = indices[i]
    dst[idx, j] = src[i, j]


wp.overload(_assign12, {"src": wp.array(dtype=float, ndim=2), "dst": wp.array(dtype=float, ndim=2)})
wp.overload(_assign12, {"src": wp.indexedarray(dtype=float, ndim=2), "dst": wp.array(dtype=float, ndim=2)})


@wp.kernel
def _assign13(src: Any, dst: wp.array(dtype=Any, ndim=3), indices: wp.array(dtype=int)) -> None:
    """Warp kernel that assigns 3D source values to destination array at specified first-dimension indices.

    Args:
        src: Source 3D array values to assign.
        dst: Destination 3D array to receive values.
        indices: Array of first-dimension indices specifying where to place each source slice.
    """
    i, j, k = wp.tid()
    idx = indices[i]
    dst[idx, j, k] = src[i, j, k]


wp.overload(_assign13, {"src": wp.array(dtype=float, ndim=3), "dst": wp.array(dtype=float, ndim=3)})
wp.overload(_assign13, {"src": wp.indexedarray(dtype=float, ndim=3), "dst": wp.array(dtype=float, ndim=3)})


@wp.kernel
def _assign22(
    src: Any, dst: wp.array(dtype=float, ndim=2), indices1: wp.array(dtype=int), indices2: wp.array(dtype=int)
) -> None:
    """Warp kernel that assigns 2D source values to destination array using two index arrays.

    Args:
        src: Source 2D array values to assign.
        dst: Destination 2D float array to receive values.
        indices1: Array of first dimension indices.
        indices2: Array of second dimension indices.
    """
    i, j = wp.tid()
    idx1 = indices1[i]
    idx2 = indices2[j]
    dst[idx1, idx2] = src[i, j]


wp.overload(_assign22, {"src": wp.array(dtype=float, ndim=2)})
wp.overload(_assign22, {"src": wp.indexedarray(dtype=float, ndim=2)})


@wp.kernel
def _assign23(
    src: Any, dst: wp.array(dtype=float, ndim=3), indices1: wp.array(dtype=int), indices2: wp.array(dtype=int)
) -> None:
    """Warp kernel that assigns 3D source values to destination array using two index arrays.

    Args:
        src: Source 3D array values to assign.
        dst: Destination 3D float array to receive values.
        indices1: Array of first dimension indices.
        indices2: Array of second dimension indices.
    """
    i, j, k = wp.tid()
    idx1 = indices1[i]
    idx2 = indices2[j]
    dst[idx1, idx2, k] = src[i, j, k]


wp.overload(_assign23, {"src": wp.array(dtype=float, ndim=3)})
wp.overload(_assign23, {"src": wp.indexedarray(dtype=float, ndim=3)})


@wp.kernel
def _assign33(
    src: Any,
    dst: wp.array(dtype=float, ndim=3),
    indices1: wp.array(dtype=int),
    indices2: wp.array(dtype=int),
    indices3: wp.array(dtype=int),
) -> None:
    """Warp kernel that assigns 3D source values to destination array using three index arrays.

    Args:
        src: Source 3D array values to assign.
        dst: Destination 3D float array to receive values.
        indices1: Array of first dimension indices.
        indices2: Array of second dimension indices.
        indices3: Array of third dimension indices.
    """
    i, j, k = wp.tid()
    idx1 = indices1[i]
    idx2 = indices2[j]
    idx3 = indices3[k]
    dst[idx1, idx2, idx3] = src[i, j, k]


wp.overload(_assign33, {"src": wp.array(dtype=float, ndim=3)})
wp.overload(_assign33, {"src": wp.indexedarray(dtype=float, ndim=3)})


def assign(src: object, dst: object, indices: object) -> wp.array:
    """Assign values from source array to destination array at specified indices.

    Assigns values from the source array to the destination array using the provided indices.
    Supports 1D, 2D, and 3D arrays with up to 3-dimensional indexing.

    Args:
        src: Source array containing values to assign.
        dst: Destination array to receive values.
        indices: Index array or list of index arrays specifying where to assign values.

    Returns:
        The destination array with assigned values.
    """
    # TODO: warp kernels not working on cpu
    device = dst.device
    src = move_data(src, "cuda:0")
    dst = move_data(dst, "cuda:0")

    if len(indices) == 1 or not isinstance(indices, list):
        indices = indices.to("cuda:0")
        if len(src.shape) == 1:
            wp.launch(_assign11, dim=src.shape, inputs=[src, dst, indices], device=dst.device)
        elif len(src.shape) == 2:
            wp.launch(_assign12, dim=src.shape, inputs=[src, dst, indices], device=dst.device)
        elif len(src.shape) == 3:
            wp.launch(_assign13, dim=src.shape, inputs=[src, dst, indices], device=dst.device)
        else:
            print("assign does not support source array with >3 dimensions.")
    elif len(indices) == 2:
        indices[0] = indices[0].to("cuda:0")
        indices[1] = indices[1].to("cuda:0")
        if len(src.shape) == 2:
            wp.launch(_assign22, dim=src.shape, inputs=[src, dst, indices[0], indices[1]], device=dst.device)
        elif len(src.shape) == 3:
            wp.launch(_assign23, dim=src.shape, inputs=[src, dst, indices[0], indices[1]], device=dst.device)
        elif len(src.shape) < 2:
            print("source array must have dimension at least 2.")
        else:
            print("assign does not support source array with >3 dimensions.")
    elif len(indices) == 3:
        indices[0] = indices[0].to("cuda:0")
        indices[1] = indices[1].to("cuda:0")
        indices[2] = indices[2].to("cuda:0")
        if len(src.shape) == 3:
            wp.launch(
                _assign33, dim=src.shape, inputs=[src, dst, indices[0], indices[1], indices[2]], device=dst.device
            )
        elif len(src.shape) < 3:
            print("source array must have dimension at least 3.")
        else:
            print("assign does not support source array with >3 dimensions.")
    else:
        print("assign does not support indices >3 dimensions.")
    dst = move_data(dst, device=device)

    return dst


@wp.kernel
def _ones(a: wp.array(dtype=wp.int32)) -> None:
    """Warp kernel that fills an int32 array with ones.

    Args:
        a: Int32 array to fill with ones.
    """
    tid = wp.tid()
    a[tid] = 1


def ones(n: int, device: str = "cpu", dtype: wp.float32 = wp.float32) -> wp.array:
    """Create a Warp array filled with ones.

    Args:
        n: Length of the array to create.
        device: Device to create the array on.
        dtype: Data type of the array elements.

    Returns:
        Warp array filled with ones.
    """
    a = wp.empty(n, dtype=dtype, device=device)
    wp.launch(kernel=_ones, dim=n, inputs=[a], device=device)
    return a


@wp.kernel
def clamp(data: wp.array(dtype=wp.float32, ndim=2), low: float, high: float) -> None:
    """Clamp array values to specified range.

    Clamps each element in the 2D array to be within the specified lower and upper bounds.

    Args:
        data: 2D array of float values to clamp.
        low: Minimum allowed value.
        high: Maximum allowed value.
    """
    i, j = wp.tid()
    data[i, j] = wp.clamp(data[i, j], low, high)


@wp.kernel
def _finite_diff2(
    result: wp.array(dtype=wp.float32, ndim=2),
    a: wp.array(dtype=wp.float32, ndim=2),
    b: wp.array(dtype=wp.float32, ndim=2),
    dt: float,
) -> None:
    """Warp kernel that computes finite difference between two 2D arrays.

    Args:
        result: Output array to store the computed finite differences.
        a: First input 2D float array.
        b: Second input 2D float array.
        dt: Time step for finite difference calculation.
    """
    i, j = wp.tid()
    result[i, j] = (a[i, j] - b[i, j]) / dt


def finite_diff2(a: object, b: object, dt: float) -> wp.array:
    """Compute finite difference between two arrays.

    Calculates the finite difference (a - b) / dt for each element pair in the input arrays.

    Args:
        a: First input array.
        b: Second input array.
        dt: Time step for the finite difference calculation.

    Returns:
        Array containing the finite difference results.
    """
    result = wp.empty(a.shape, dtype=wp.float32, device=a.device)
    wp.launch(kernel=_finite_diff2, dim=a.shape, inputs=[result, a, b, dt], device=a.device)
    return result
