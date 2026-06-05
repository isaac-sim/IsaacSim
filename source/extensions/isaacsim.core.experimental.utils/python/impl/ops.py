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

"""Functions for manipulating and performing operations on Warp arrays and other types."""

from __future__ import annotations

import carb
import numpy as np
import warp as wp
from warp._src.types import np_dtype_to_warp_type

_INDICES_CACHE: dict[tuple[int, type, str], wp.array] = {}


def _broadcastable_shape(src: tuple[int], dst: tuple[int]) -> tuple[tuple[int] | None, tuple[bool] | None]:
    """Determine the broadcastable shape and axis information for broadcasting operations.

    Args:
        src: Source shape tuple.
        dst: Destination shape tuple to broadcast to.

    Returns:
        A tuple containing the supporting shape and boolean flags for each axis indicating
        whether broadcasting is needed.
    """
    shape = [1] * len(dst)
    axes = [True] * len(dst)
    reversed_src = src[::-1]
    for i, item in enumerate(reversed(dst)):
        try:
            if reversed_src[i] == item:
                shape[i] = item
                axes[i] = False
            elif reversed_src[i] != 1:
                raise ValueError(f"Incompatible broadcasting: original shape: {src}, requested shape {dst}")
        except IndexError:
            break
    return shape[::-1], axes[::-1]


def _astype(src: wp.array, dtype: type) -> wp.array:
    """Cast a Warp array to a different data type.

    Args:
        src: Source Warp array to cast.
        dtype: Target data type for the output array.

    Returns:
        New Warp array with the specified data type and same shape as source.
    """
    dst = wp.empty(shape=src.shape, dtype=dtype, device=src.device)
    wp.launch(
        _WK_CAST[src.ndim],
        dim=src.shape,
        inputs=[src, dst],
        device=src.device,
    )
    return dst


def parse_device(device: str | wp.Device | None, *, raise_on_invalid: bool = False) -> wp.Device:
    """Parse the input device and return a Warp :py:class:`~warp.Device` instance.

    Args:
        device: Device specification. If the specified device is ``None`` or it cannot be resolved,
            the default available device will be returned instead.
        raise_on_invalid: Whether to raise an exception if the device is invalid.
            If ``False``, a warning is logged and the default available device is returned instead.

    Returns:
        Warp Device.

    Raises:
        ValueError: If the input device is invalid and ``raise_on_invalid`` is ``True``.

    Example:

    .. code-block:: python

        >>> import isaacsim.core.experimental.utils.ops as ops_utils
        >>>
        >>> device = ops_utils.parse_device("cpu")
        >>> print(type(device), device)
        <class 'warp._src.context.Device'> cpu
        >>> device = ops_utils.parse_device("cuda")
        >>> print(type(device), device)
        <class 'warp._src.context.Device'> cuda:0
        >>> device = ops_utils.parse_device("cuda:0")
        >>> print(type(device), device)
        <class 'warp._src.context.Device'> cuda:0
    """
    if isinstance(device, wp.Device):
        return device
    elif isinstance(device, str):
        try:
            return wp.get_device(device)
        except ValueError as e:
            if raise_on_invalid:
                raise ValueError(f"Invalid device specification ({device}): {e}")
            _default_device = wp.get_device()
            carb.log_warn(f"Invalid device specification ({device}): {e}. Using default device ({_default_device})")
            return _default_device
    return wp.get_device()


def place(
    x: bool | int | float | list | np.ndarray | wp.array,
    *,
    dtype: type | None = None,
    device: str | wp.Device | None = None,
) -> wp.array:
    """Create a Warp array from a Python primitive or list, a NumPy array, or a Warp array.

    Args:
        x: Python primitive or list, NumPy array, or Warp array.
            If the input is a Warp array with the same device and dtype, it is returned as is.
        dtype: Data type of the output array. If not provided, the data type of the input is used.
        device: Device to place the output array on. If ``None``, the default device is used,
            unless the input is a Warp array (in which case the input device is used).

    Returns:
        Warp array instance.

    Raises:
        TypeError: If the input argument ``x`` is not a supported data container.

    Example:

    .. code-block:: python

        >>> import isaacsim.core.experimental.utils.ops as ops_utils
        >>> import numpy as np
        >>> import warp as wp
        >>>
        >>> # Python primitive
        >>> # - bool
        >>> array = ops_utils.place(True, device="cpu")  # doctest: +NO_CHECK
        >>> print(array, array.dtype, array.device, array.shape)
        [ True] <class 'warp._src.types.bool'> cpu (1,)
        >>> # - int
        >>> array = ops_utils.place(1, device="cpu")  # doctest: +NO_CHECK
        >>> print(array, array.dtype, array.device, array.shape)
        [1] <class 'warp._src.types.int64'> cpu (1,)
        >>> # - float
        >>> array = ops_utils.place(1.0, device="cpu")  # doctest: +NO_CHECK
        >>> print(array, array.dtype, array.device, array.shape)
        [1.] <class 'warp._src.types.float64'> cpu (1,)
        >>>
        >>> # Python list
        >>> array = ops_utils.place([1.0, 2.0, 3.0], device="cpu")  # doctest: +NO_CHECK
        >>> print(array, array.dtype, array.device, array.shape)
        [1. 2. 3.] <class 'warp._src.types.float64'> cpu (3,)
        >>>
        >>> # NumPy array (with shape (3, 1))
        >>> array = ops_utils.place(np.array([[1], [2], [3]], dtype=np.uint8), dtype=wp.float32)  # doctest: +NO_CHECK
        >>> print(array, array.dtype, array.device, array.shape)
        [[1.] [2.] [3.]] <class 'warp._src.types.float32'> cuda:0 (3, 1)
        >>>
        >>> # Warp array (with different device)
        >>> array = ops_utils.place(wp.array([1.0, 2.0, 3.0], device="cpu"), device="cuda")  # doctest: +NO_CHECK
        >>> print(array, array.dtype, array.device, array.shape)
        [1. 2. 3.] <class 'warp._src.types.float64'> cuda:0 (3,)
    """
    # hint: don't use wp.from_numpy as it returns vector/matrix for arrays of dimensions 2/3
    if isinstance(x, wp.array):
        if device is not None:
            x = x.to(device)
        if dtype is not None and x.dtype != dtype:
            x = _astype(x, dtype)
        return x
    elif isinstance(x, np.ndarray):
        return wp.array(x, dtype=np_dtype_to_warp_type.get(x.dtype) if dtype is None else dtype, device=device)
    elif isinstance(x, (list, tuple)):
        x = np.array(x)
        return wp.array(x, dtype=np_dtype_to_warp_type.get(x.dtype) if dtype is None else dtype, device=device)
    elif isinstance(x, (bool, int, float)):
        py_dtype_to_np_type = {bool: np.bool_, int: np.int64, float: np.float64}
        x = np.array([x], dtype=py_dtype_to_np_type.get(type(x), type(x)))
        return wp.array(x, dtype=np_dtype_to_warp_type.get(x.dtype) if dtype is None else dtype, device=device)
    raise TypeError(f"Unsupported type: {type(x)}")


def resolve_indices(
    x: bool | int | float | list | np.ndarray | wp.array | None,
    *,
    count: int | None = None,
    dtype: type | None = wp.int32,
    device: str | wp.Device | None = None,
) -> wp.array:
    """Create a flattened (1D) Warp array to be used as indices from a Python primitive or list, a NumPy array, or a Warp array.

    Args:
        x: Python primitive or list, NumPy array, or Warp array.
        count: Number of indices to resolve.
            If input argument ``x`` is ``None``, the indices are generated from 0 to ``count - 1``.
            If input argument ``x`` is not ``None``, this value is ignored.
        dtype: Data type of the output array. If ``None``, ``wp.int32`` is used.
        device: Device to place the output array on. If ``None``, the default device is used,
            unless the input is a Warp array (in which case the input device is used).

    Returns:
        Flattened (1D) Warp array instance.

    Raises:
        ValueError: If input argument ``x`` is ``None`` and ``count`` is not provided.
        TypeError: If the input argument ``x`` is not a supported data container.

    Example:

    .. code-block:: python

        >>> import isaacsim.core.experimental.utils.ops as ops_utils
        >>> import numpy as np
        >>> import warp as wp
        >>>
        >>> # Python primitive
        >>> # - bool
        >>> indices = ops_utils.resolve_indices(True, device="cpu")  # doctest: +NO_CHECK
        >>> print(indices, indices.dtype, indices.device, indices.shape)
        [1] <class 'warp._src.types.int32'> cpu (1,)
        >>> # - int
        >>> indices = ops_utils.resolve_indices(2, device="cpu")  # doctest: +NO_CHECK
        >>> print(indices, indices.dtype, indices.device, indices.shape)
        [2] <class 'warp._src.types.int32'> cpu (1,)
        >>> # - float
        >>> indices = ops_utils.resolve_indices(3.0, device="cpu")  # doctest: +NO_CHECK
        >>> print(indices, indices.dtype, indices.device, indices.shape)
        [3] <class 'warp._src.types.int32'> cpu (1,)
        >>>
        >>> # Python list
        >>> indices = ops_utils.resolve_indices([1, 2, 3], device="cpu")  # doctest: +NO_CHECK
        >>> print(indices, indices.dtype, indices.device, indices.shape)
        [1 2 3] <class 'warp._src.types.int32'> cpu (3,)
        >>>
        >>> # NumPy array (with shape (3, 1))
        >>> indices = ops_utils.resolve_indices(np.array([[1], [2], [3]], dtype=np.uint8))  # doctest: +NO_CHECK
        >>> print(indices, indices.dtype, indices.device, indices.shape)
        [1 2 3] <class 'warp._src.types.int32'> cuda:0 (3,)
        >>>
        >>> # Warp array (with different device)
        >>> indices = ops_utils.resolve_indices(wp.array([1, 2, 3], device="cpu"), device="cuda")  # doctest: +NO_CHECK
        >>> print(indices, indices.dtype, indices.device, indices.shape)
        [1 2 3] <class 'warp._src.types.int32'> cuda:0 (3,)
    """
    # hint: don't use wp.from_numpy as it returns vector/matrix for arrays of dimensions 2/3
    if dtype is None:
        dtype = wp.int32
    if x is None:
        if count is None:
            raise ValueError("Either input argument `x` or `count` must be provided")
        resolved_device = parse_device(device)
        key = (count, dtype, str(resolved_device))
        cached = _INDICES_CACHE.get(key)
        if cached is None:
            cached = wp.array(np.arange(count), dtype=dtype, device=resolved_device)
            _INDICES_CACHE[key] = cached
        return cached
    elif isinstance(x, wp.array):
        if device is not None:
            x = x.to(device)
        if dtype is not None and x.dtype != dtype:
            x = _astype(x, dtype)
        return x if x.ndim == 1 else x.contiguous().flatten()
    elif isinstance(x, np.ndarray):
        return wp.array(x.flatten(), dtype=dtype, device=device)
    elif isinstance(x, (list, tuple)):
        return wp.array(x, dtype=dtype, device=device).flatten()
    elif isinstance(x, (bool, int, float)):
        return wp.array([x], dtype=dtype, device=device).flatten()
    else:
        raise TypeError(f"Unsupported type: {type(x)}")


def broadcast_to(
    x: bool | int | float | list | np.ndarray | wp.array,
    *,
    shape: list[int],
    dtype: type | None = None,
    device: str | wp.Device | None = None,
) -> wp.array:
    """Broadcast a Python primitive or list, a NumPy array, or a Warp array to a Warp array with a new shape.

    .. note::

        Broadcasting follows NumPy's rules: Two shapes are compatible if by comparing their dimensions element-wise,
        starting with the trailing dimension (i.e., rightmost) and moving leftward

        * they are equal, or
        * one of them is 1.

    Args:
        x: Python primitive or list, NumPy array, or Warp array.
        shape: Shape of the desired array.
        dtype: Data type of the output array. If ``None``, the data type of the input is used.
        device: Device to place the output array on. If ``None``, the default device is used,
            unless the input is a Warp array (in which case the input device is used).

    Returns:
        Warp array with the given shape.

    Raises:
        ValueError: If the input list or array is not compatible with the new shape according to the broadcasting rules.
        TypeError: If the input argument ``x`` is not a supported data container.

    Example:

    .. code-block:: python

        >>> import isaacsim.core.experimental.utils.ops as ops_utils
        >>> import numpy as np
        >>> import warp as wp
        >>>
        >>> # Python primitive
        >>> # - bool
        >>> array = ops_utils.broadcast_to(True, shape=(1, 3))  # doctest: +NO_CHECK
        >>> print(array)
        [[ True  True  True]]
        >>> # - int
        >>> array = ops_utils.broadcast_to(2, shape=(1, 3))  # doctest: +NO_CHECK
        >>> print(array)
        [[2 2 2]]
        >>> # - float
        >>> array = ops_utils.broadcast_to(3.0, shape=(1, 3))  # doctest: +NO_CHECK
        >>> print(array)
        [[3. 3. 3.]]
        >>>
        >>> # Python list
        >>> array = ops_utils.broadcast_to([1, 2, 3], shape=(1, 3))  # doctest: +NO_CHECK
        >>> print(array)
        [[1 2 3]]
        >>>
        >>> # NumPy array (with shape (1, 3))
        >>> array = ops_utils.broadcast_to(np.array([[1, 2, 3]]), shape=(2, 3))  # doctest: +NO_CHECK
        >>> print(array)
        [[1 2 3]
         [1 2 3]]
        >>>
        >>> # Warp array (with different device)
        >>> array = ops_utils.broadcast_to(wp.array([1, 2, 3], device="cpu"), shape=(3, 3), device="cuda")  # doctest: +NO_CHECK
        >>> print(array)
        [[1 2 3]
         [1 2 3]
         [1 2 3]]
    """
    # hint: don't use wp.from_numpy as it returns vector/matrix for arrays of dimensions 2/3
    if isinstance(x, wp.array):
        ndim = len(shape)
        if x.shape == shape:
            pass
        elif x.ndim <= ndim:
            output = wp.empty(shape=shape, dtype=x.dtype, device=x.device)
            supporting_shape, axes = _broadcastable_shape(x.shape, shape)
            wp.launch(
                _WK_BROADCAST[ndim],
                dim=shape,
                inputs=[x.reshape(supporting_shape), output, *axes],
                device=x.device,
            )
            x = output
        else:
            raise ValueError(
                f"Operands could not be broadcast together: original shape: {x.shape}, requested shape {shape}"
            )
        if device is not None:
            x = x.to(device)
        if dtype is not None and x.dtype != dtype:
            x = _astype(x, dtype)
        return x
    elif isinstance(x, np.ndarray):
        x = np.broadcast_to(x, shape=shape)
        return wp.array(x, dtype=np_dtype_to_warp_type.get(x.dtype) if dtype is None else dtype, device=device)
    elif isinstance(x, (list, tuple)):
        x = np.broadcast_to(np.array(x), shape=shape)
        return wp.array(x, dtype=np_dtype_to_warp_type.get(x.dtype) if dtype is None else dtype, device=device)
    elif isinstance(x, (bool, int, float)):
        py_dtype_to_np_type = {bool: np.bool_, int: np.int64, float: np.float64}
        x = np.broadcast_to(np.array([x], dtype=py_dtype_to_np_type.get(type(x), type(x))), shape=shape)
        return wp.array(x, dtype=np_dtype_to_warp_type.get(x.dtype) if dtype is None else dtype, device=device)
    else:
        raise TypeError(f"Unsupported type: {type(x)}")


"""
Custom Warp kernels.
"""


@wp.kernel(enable_backward=False)
def _wk_cast_1d(src: wp.array(ndim=1), dst: wp.array(ndim=1)) -> None:
    """Warp kernel for casting 1D arrays to a different data type.

    Args:
        src: Source 1D Warp array to cast.
        dst: Destination 1D Warp array with target data type.
    """
    i = wp.tid()
    dst[i] = dst.dtype(src[i])


@wp.kernel(enable_backward=False)
def _wk_cast_2d(src: wp.array(ndim=2), dst: wp.array(ndim=2)) -> None:
    """Warp kernel for casting 2D arrays to a different data type.

    Args:
        src: Source 2D Warp array to cast.
        dst: Destination 2D Warp array with target data type.
    """
    i, j = wp.tid()
    dst[i, j] = dst.dtype(src[i, j])


@wp.kernel(enable_backward=False)
def _wk_cast_3d(src: wp.array(ndim=3), dst: wp.array(ndim=3)) -> None:
    """Warp kernel for casting 3D arrays to a different data type.

    Args:
        src: Source 3D Warp array to cast.
        dst: Destination 3D Warp array with target data type.
    """
    i, j, k = wp.tid()
    dst[i, j, k] = dst.dtype(src[i, j, k])


@wp.kernel(enable_backward=False)
def _wk_cast_4d(src: wp.array(ndim=4), dst: wp.array(ndim=4)) -> None:
    """Warp kernel for casting 4D arrays to a different data type.

    Args:
        src: Source 4D Warp array to cast.
        dst: Destination 4D Warp array with target data type.
    """
    i, j, k, w = wp.tid()
    dst[i, j, k, w] = dst.dtype(src[i, j, k, w])


@wp.kernel(enable_backward=False)
def _wk_broadcast_1d(src: wp.array(ndim=1), dst: wp.array(ndim=1), axis_0: bool) -> None:
    """Warp kernel for broadcasting 1D arrays.

    Args:
        src: Source 1D Warp array.
        dst: Destination 1D Warp array to fill.
        axis_0: Whether to broadcast along axis 0.
    """
    i = wp.tid()
    index_0 = i
    if axis_0:
        index_0 = 0
    dst[i] = src[index_0]


@wp.kernel(enable_backward=False)
def _wk_broadcast_2d(src: wp.array(ndim=2), dst: wp.array(ndim=2), axis_0: bool, axis_1: bool) -> None:
    """Warp kernel for broadcasting 2D arrays.

    Args:
        src: Source 2D Warp array.
        dst: Destination 2D Warp array to fill.
        axis_0: Whether to broadcast along axis 0.
        axis_1: Whether to broadcast along axis 1.
    """
    i, j = wp.tid()
    index_0, index_1 = i, j
    if axis_0:
        index_0 = 0
    if axis_1:
        index_1 = 0
    dst[i, j] = src[index_0, index_1]


@wp.kernel(enable_backward=False)
def _wk_broadcast_3d(src: wp.array(ndim=3), dst: wp.array(ndim=3), axis_0: bool, axis_1: bool, axis_2: bool) -> None:
    """Warp kernel for broadcasting 3D arrays.

    Args:
        src: Source 3D Warp array.
        dst: Destination 3D Warp array to fill.
        axis_0: Whether to broadcast along axis 0.
        axis_1: Whether to broadcast along axis 1.
        axis_2: Whether to broadcast along axis 2.
    """
    i, j, k = wp.tid()
    index_0, index_1, index_2 = i, j, k
    if axis_0:
        index_0 = 0
    if axis_1:
        index_1 = 0
    if axis_2:
        index_2 = 0
    dst[i, j, k] = src[index_0, index_1, index_2]


@wp.kernel(enable_backward=False)
def _wk_broadcast_4d(
    src: wp.array(ndim=4), dst: wp.array(ndim=4), axis_0: bool, axis_1: bool, axis_2: bool, axis_3: bool
) -> None:
    """Warp kernel for broadcasting 4D arrays.

    Args:
        src: Source 4D Warp array.
        dst: Destination 4D Warp array to fill.
        axis_0: Whether to broadcast along axis 0.
        axis_1: Whether to broadcast along axis 1.
        axis_2: Whether to broadcast along axis 2.
        axis_3: Whether to broadcast along axis 3.
    """
    i, j, k, w = wp.tid()
    index_0, index_1, index_2, index_3 = i, j, k, w
    if axis_0:
        index_0 = 0
    if axis_1:
        index_1 = 0
    if axis_2:
        index_2 = 0
    if axis_3:
        index_3 = 0
    dst[i, j, k, w] = src[index_0, index_1, index_2, index_3]


_WK_CAST = [
    None,
    _wk_cast_1d,
    _wk_cast_2d,
    _wk_cast_3d,
    _wk_cast_4d,
]

_WK_BROADCAST = [
    None,
    _wk_broadcast_1d,
    _wk_broadcast_2d,
    _wk_broadcast_3d,
    _wk_broadcast_4d,
]
