# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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

"""Provide Warp tensor conversion and allocation helpers.

The physics manager uses Warp as its tensor data plane. This module maps
registration data types to Warp data types, resolves simulation devices, and
adapts DLPack-compatible binding results to Warp arrays.
"""

from __future__ import annotations

import ctypes
from collections.abc import Sequence
from typing import TypeVar

import warp as wp
from isaacsim.physics.registration.bindings._bindings import DType

TensorValue = TypeVar("TensorValue")

_DTYPE_TO_WARP: dict[DType, wp.dtype] = {
    DType.FLOAT32: wp.float32,
    DType.FLOAT64: wp.float64,
    DType.INT8: wp.int8,
    DType.INT16: wp.int16,
    DType.INT32: wp.int32,
    DType.INT64: wp.int64,
    DType.UINT8: wp.uint8,
    DType.UINT16: wp.uint16,
    DType.UINT32: wp.uint32,
    DType.UINT64: wp.uint64,
}

_DTYPE_FROM_WARP: dict[wp.dtype, DType] = {
    wp.float32: DType.FLOAT32,
    wp.float64: DType.FLOAT64,
    wp.int8: DType.INT8,
    wp.int16: DType.INT16,
    wp.int32: DType.INT32,
    wp.int64: DType.INT64,
    wp.uint8: DType.UINT8,
    wp.uint16: DType.UINT16,
    wp.uint32: DType.UINT32,
    wp.uint64: DType.UINT64,
    # Warp's vec types still flatten to float32; the engine handles the
    # extra trailing dimension explicitly.
    wp.vec2: DType.FLOAT32,
    wp.vec3: DType.FLOAT32,
    wp.vec4: DType.FLOAT32,
}


def parse_device(device_ordinal: int) -> wp.Device:
    """Resolve a device ordinal to a Warp device.

    Args:
        device_ordinal: Device index. Use ``-1`` for the CPU or a nonnegative
            value for the corresponding CUDA device.

    Returns:
        Resolved Warp device.

    """
    if device_ordinal == -1:
        return wp.get_device("cpu")
    return wp.get_device(f"cuda:{device_ordinal}")


def device_ordinal_from_warp(wp_device: wp.Device) -> int:
    """Get the ordinal for a Warp device.

    Args:
        wp_device: Warp device to convert.

    Returns:
        ``-1`` for the CPU or the CUDA device ordinal.

    Raises:
        ValueError: If ``wp_device`` is neither a CPU nor CUDA device.

    """
    if wp_device.is_cpu:
        return -1
    if wp_device.is_cuda:
        return wp_device.ordinal
    raise ValueError(f"warp_frontend: unrecognised warp device {wp_device}")


def create_tensor(shape: Sequence[int], dtype: DType | wp.dtype, device_ordinal: int = -1) -> wp.array:
    """Create a zero-initialized Warp tensor.

    Args:
        shape: Tensor dimensions.
        dtype: Physics registration data type or native Warp data type.
        device_ordinal: Device index. Use ``-1`` for the CPU or a nonnegative
            value for the corresponding CUDA device.

    Returns:
        Contiguous Warp array with the requested shape, data type, and device.

    Raises:
        KeyError: If ``dtype`` has no Warp mapping.

    """
    # Accept either an umbrella DType (canonical) or a raw Warp dtype.
    if dtype in _DTYPE_FROM_WARP:
        wp_dtype = dtype
    else:
        wp_dtype = _DTYPE_TO_WARP[dtype]
    return wp.zeros(tuple(shape), dtype=wp_dtype, device=parse_device(device_ordinal))


def wrap_tensor(value: object | None) -> wp.array | None:
    """Wrap a binding result as a Warp array.

    DLPack imports share the source allocation and let Warp retain the lifetime
    information supplied by the producer. The array-like fallback creates an
    independent copy.

    Args:
        value: DLPack-compatible binding result, Warp array, array-like value,
            or ``None``.

    Returns:
        ``value`` unchanged when it is already a Warp array, a Warp array
        imported through DLPack or copied from an array-like value, or ``None``
        when ``value`` is ``None``.

    """
    if value is None:
        return None
    if isinstance(value, wp.array):
        return value
    try:
        return wp.from_dlpack(value)
    except Exception:  # pragma: no cover - rare fallback
        return wp.array(value, copy=True)


def unwrap_to_array(value: TensorValue) -> TensorValue:
    """Return the value unchanged for placeholder outbound conversion.

    This is the write-direction counterpart to :func:`wrap_tensor` (frontend
    tensor -> value handed to the C++ binding), kept as a seam for a future
    non-Warp frontend that would convert its native tensor here. It is a no-op
    for the Warp frontend -- ``set_data`` accepts a ``wp.array`` (or any
    DLPack-capable object) and the binding consumes it directly -- and it is not
    yet wired into the write path, so nothing calls it today.

    Args:
        value: Tensor value passed to the binding layer.

    Returns:
        The original object, unchanged.

    """
    return value


def as_contiguous(tensor: wp.array, dtype: DType | wp.dtype | None) -> wp.array:
    """Validate a Warp array's scalar data type without changing its layout.

    This function neither checks nor creates contiguous storage. It returns the
    original array, including a strided array, after comparing scalar ctypes.

    Args:
        tensor: Warp array to validate.
        dtype: Required physics registration or Warp data type. Use ``None``
            to accept the array's current data type.

    Returns:
        Original Warp array with its existing shape and strides.

    Raises:
        TypeError: If the scalar data type does not match ``dtype``.

    """
    if dtype is None:
        return tensor
    expected_ctype = wp.types.type_ctype(_DTYPE_TO_WARP.get(dtype, dtype))
    if issubclass(expected_ctype, ctypes.Array):
        expected_ctype = expected_ctype._type_
    actual = wp.types.type_ctype(tensor.dtype)
    if issubclass(actual, ctypes.Array):
        actual = actual._type_
    if actual is not expected_ctype:
        raise TypeError(f"as_contiguous: expected {expected_ctype}, got {actual} " f"(tensor dtype = {tensor.dtype})")
    return tensor


def dtype_to_warp(dtype: DType) -> wp.dtype:
    """Convert a physics registration data type to a Warp data type.

    Args:
        dtype: Physics registration data type.

    Returns:
        Corresponding Warp data type.

    Raises:
        KeyError: If ``dtype`` has no Warp mapping.

    """
    return _DTYPE_TO_WARP[dtype]


def dtype_from_warp(wp_dtype: wp.dtype) -> DType:
    """Convert a Warp data type to a physics registration data type.

    Warp vector types map to ``DType.FLOAT32`` because tensor descriptors
    represent their vector width as an additional shape dimension.

    Args:
        wp_dtype: Warp data type.

    Returns:
        Corresponding physics registration data type.

    Raises:
        KeyError: If ``wp_dtype`` has no registration mapping.

    """
    return _DTYPE_FROM_WARP[wp_dtype]
