# SPDX-FileCopyrightText: Copyright (c) 2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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

import numpy as np
import warp as wp
from warp.types import np_dtype_to_warp_type


def _broadcastable_shape(src: tuple[int], dst: tuple[int]) -> tuple[tuple[int] | None, tuple[bool] | None]:
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
    dst = wp.empty(shape=src.shape, dtype=dtype, device=src.device)
    wp.launch(
        _WK_CAST[src.ndim],
        dim=src.shape,
        inputs=[src, dst],
        device=src.device,
    )
    return dst


def place(
    x: list | np.ndarray | wp.array,
    *,
    dtype: type | None = None,
    device: str | wp.context.Device | None = None,
) -> wp.array:
    """Create a Warp array from a list, a NumPy array, or a Warp array.

    Args:
        x: List, NumPy array, or Warp array.
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
        >>> # list
        >>> array = ops_utils.place([1.0, 2.0, 3.0], device="cpu")  # doctest: +NO_CHECK
        >>> print(array, array.dtype, array.device, array.shape)
        [1. 2. 3.] <class 'warp.types.float64'> cpu (3,)
        >>>
        >>> # NumPy array (with shape (3, 1))
        >>> array = ops_utils.place(np.array([[1], [2], [3]], dtype=np.uint8), dtype=wp.float32)  # doctest: +NO_CHECK
        >>> print(array, array.dtype, array.device, array.shape)
        [[1.] [2.] [3.]] <class 'warp.types.float32'> cuda:0 (3, 1)
        >>>
        >>> # Warp array (with different device)
        >>> array = ops_utils.place(wp.array([1.0, 2.0, 3.0], device="cpu"), device="cuda")  # doctest: +NO_CHECK
        >>> print(array, array.dtype, array.device, array.shape)
        [1. 2. 3.] <class 'warp.types.float64'> cuda:0 (3,)
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
    raise TypeError(f"Unsupported type: {type(x)}")


def resolve_indices(
    x: list | np.ndarray | wp.array | None,
    *,
    count: int | None = None,
    dtype: type | None = wp.int32,
    device: str | wp.context.Device | None = None,
) -> wp.array:
    """Create a flattened (1D) Warp array to be used as indices from a list, a NumPy array, or a Warp array.

    Args:
        x: List, NumPy array, or Warp array.
        count: Number of indices to resolve.
            If input argument ``x`` is ``None``, the indices are generated from 0 to ``count - 1``.
            If the input is not ``None``, this value is ignored.
        dtype: Data type of the output array. If ``None``, ``wp.int32`` is used.
        device: Device to place the output array on. If ``None``, the default device is used,
            unless the input is a Warp array (in which case the input device is used).

    Returns:
        Flattened (1D) Warp array instance.

    Raises:
        ValueError: If input argument ``x`` is ``None`` and ``count`` is not provided.

    Example:

    .. code-block:: python

        >>> import isaacsim.core.experimental.utils.ops as ops_utils
        >>> import numpy as np
        >>> import warp as wp
        >>>
        >>> # list
        >>> indices = ops_utils.resolve_indices([1, 2, 3], device="cpu")  # doctest: +NO_CHECK
        >>> print(indices, indices.dtype, indices.device, indices.shape)
        [1 2 3] <class 'warp.types.int32'> cpu (3,)
        >>>
        >>> # NumPy array (with shape (3, 1))
        >>> indices = ops_utils.resolve_indices(np.array([[1], [2], [3]], dtype=np.uint8))  # doctest: +NO_CHECK
        >>> print(indices, indices.dtype, indices.device, indices.shape)
        [1 2 3] <class 'warp.types.int32'> cuda:0 (3,)
        >>>
        >>> # Warp array (with different device)
        >>> indices = ops_utils.resolve_indices(wp.array([1, 2, 3], device="cpu"), device="cuda")  # doctest: +NO_CHECK
        >>> print(indices, indices.dtype, indices.device, indices.shape)
        [1 2 3] <class 'warp.types.int32'> cuda:0 (3,)
    """
    # hint: don't use wp.from_numpy as it returns vector/matrix for arrays of dimensions 2/3
    if dtype is None:
        dtype = wp.int32
    if x is None:
        if count is None:
            raise ValueError("Either input argument `x` or `count` must be provided")
        return wp.array(np.arange(count), dtype=dtype, device=device)
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


def broadcast_to(
    x: list | np.ndarray | wp.array,
    *,
    shape: list[int],
    dtype: type | None = None,
    device: str | wp.context.Device | None = None,
) -> wp.array:
    """Broadcast a list, a NumPy array, or a Warp array to a Warp array with a new shape.

    .. note::

        Broadcasting follows NumPy's rules: Two shapes are compatible if by comparing their dimensions element-wise,
        starting with the trailing dimension (i.e., rightmost) and moving leftward

        * they are equal, or
        * one of them is 1.

    Args:
        x: List, NumPy array, or Warp array.
        shape: Shape of the desired array.
        dtype: Data type of the output array. If ``None``, the data type of the input is used.
        device: Device to place the output array on. If ``None``, the default device is used,
            unless the input is a Warp array (in which case the input device is used).

    Returns:
        Warp array with the given shape.

    Raises:
        ValueError: If the input list or array is not compatible with the new shape according to the broadcasting rules.

    Example:

    .. code-block:: python

        >>> import isaacsim.core.experimental.utils.ops as ops_utils
        >>> import numpy as np
        >>> import warp as wp
        >>>
        >>> # list
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


"""
Custom Warp kernels.
"""


@wp.kernel(enable_backward=False)
def _wk_cast_1d(src: wp.array(ndim=1), dst: wp.array(ndim=1)):
    i = wp.tid()
    dst[i] = dst.dtype(src[i])


@wp.kernel(enable_backward=False)
def _wk_cast_2d(src: wp.array(ndim=2), dst: wp.array(ndim=2)):
    i, j = wp.tid()
    dst[i, j] = dst.dtype(src[i, j])


@wp.kernel(enable_backward=False)
def _wk_cast_3d(src: wp.array(ndim=3), dst: wp.array(ndim=3)):
    i, j, k = wp.tid()
    dst[i, j, k] = dst.dtype(src[i, j, k])


@wp.kernel(enable_backward=False)
def _wk_cast_4d(src: wp.array(ndim=4), dst: wp.array(ndim=4)):
    i, j, k, l = wp.tid()
    dst[i, j, k, l] = dst.dtype(src[i, j, k, l])


@wp.kernel(enable_backward=False)
def _wk_broadcast_1d(src: wp.array(ndim=1), dst: wp.array(ndim=1), axis_0: bool):
    i = wp.tid()
    index_0 = i
    if axis_0:
        index_0 = 0
    dst[i] = src[index_0]


@wp.kernel(enable_backward=False)
def _wk_broadcast_2d(src: wp.array(ndim=2), dst: wp.array(ndim=2), axis_0: bool, axis_1: bool):
    i, j = wp.tid()
    index_0, index_1 = i, j
    if axis_0:
        index_0 = 0
    if axis_1:
        index_1 = 0
    dst[i, j] = src[index_0, index_1]


@wp.kernel(enable_backward=False)
def _wk_broadcast_3d(src: wp.array(ndim=3), dst: wp.array(ndim=3), axis_0: bool, axis_1: bool, axis_2: bool):
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
):
    i, j, k, l = wp.tid()
    index_0, index_1, index_2, index_3 = i, j, k, l
    if axis_0:
        index_0 = 0
    if axis_1:
        index_1 = 0
    if axis_2:
        index_2 = 0
    if axis_3:
        index_3 = 0
    dst[i, j, k, l] = src[index_0, index_1, index_2, index_3]


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
