# SPDX-FileCopyrightText: Copyright (c) 2021-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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

"""Deprecated interoperability utility functions."""

__all__ = [
    "warp2torch",
    "warp2jax",
    "warp2tensorflow",
    "warp2numpy",
    "torch2warp",
    "torch2jax",
    "torch2tensorflow",
    "torch2numpy",
    "jax2warp",
    "jax2torch",
    "jax2tensorflow",
    "jax2numpy",
    "tensorflow2warp",
    "tensorflow2torch",
    "tensorflow2jax",
    "tensorflow2numpy",
    "numpy2warp",
    "numpy2torch",
    "numpy2jax",
    "numpy2tensorflow",
]

# module configuration
_c_warp_torch_interop = True  # whether use torch interop instead of dlpack

# function references
_warp_to_dlpack = None
_warp_from_dlpack = None
_warp_to_torch = None

_torch_to_dlpack = None
_torch_from_dlpack = None
_torch_to_warp = None

_jax_to_dlpack = None
_jax_from_dlpack = None
_jax_cpu_device_context = None

_tensorflow_to_dlpack = None
_tensorflow_from_dlpack = None
_tensorflow_make_tensor_proto = None
_tensorflow_cpu_device_context = None

_numpy_array = None
_numpy_to_warp = None
_numpy_to_warp_dtype = None
_numpy_to_torch = None
_numpy_to_jax = None
_numpy_to_tensorflow = None
_numpy_from_tensorflow = None


# Warp


def warp2torch(array: "warp.array") -> "torch.Tensor":  # noqa: F821, F811
    """Convert Warp array to PyTorch tensor.

    Args:
        array: Warp array

    Returns:
        PyTorch tensor

    Example:

    .. code-block:: python

        >>> import isaacsim.core.utils.interops as interops_utils
        >>> import warp as wp
        >>>
        >>> wp.init()  # doctest: +NO_CHECK
        >>> warp_array = wp.zeros((100, 10), dtype=wp.float32, device="cuda:0")
        >>> torch_tensor = interops_utils.warp2torch(warp_array)
        >>> type(torch_tensor)
        <class 'torch.Tensor'>
    """
    global warp2torch, _warp_to_torch, _warp_to_dlpack, _torch_from_dlpack  # noqa: F811
    # warp-torch interop
    if _c_warp_torch_interop:
        # get conversion function
        if _warp_to_torch is None:
            import warp

            _warp_to_torch = warp.to_torch
        warp2torch = lambda array: _warp_to_torch(array, requires_grad=False)  # noqa: E731
    # dlpack
    else:
        # get source dlpack function
        if _warp_to_dlpack is None:
            import warp

            _warp_to_dlpack = warp.to_dlpack
        # get target dlpack function
        if _torch_from_dlpack is None:
            import torch

            _torch_from_dlpack = torch.from_dlpack

        warp2torch = lambda array: _torch_from_dlpack(_warp_to_dlpack(array))  # noqa: E731
    return warp2torch(array)


def warp2jax(array: "warp.array") -> "jax.Array":  # noqa: F821, F811
    """Convert Warp array to JAX array.

    Args:
        array: Warp array

    Returns:
        JAX array

    Example:

    .. code-block:: python

        >>> import isaacsim.core.utils.interops as interops_utils
        >>> import warp as wp
        >>>
        >>> wp.init()  # doctest: +NO_CHECK
        >>> warp_array = wp.zeros((100, 10), dtype=wp.float32, device="cuda:0")
        >>> jax_array = interops_utils.warp2jax(warp_array)
        >>> type(jax_array)
        <class 'jaxlib.xla_extension.Array...'>
    """
    global warp2jax, _warp_to_dlpack, _jax_from_dlpack  # noqa: F811
    # get source dlpack function
    if _warp_to_dlpack is None:
        import warp

        _warp_to_dlpack = warp.to_dlpack
    # get target dlpack function
    if _jax_from_dlpack is None:
        import jax.dlpack

        _jax_from_dlpack = jax.dlpack.from_dlpack

    warp2jax = lambda array: _jax_from_dlpack(_warp_to_dlpack(array))  # noqa: E731
    return warp2jax(array)


def warp2tensorflow(array: "warp.array") -> "tensorflow.Tensor":  # noqa: F821, F811
    """Convert Warp array to TensorFlow tensor.

    Args:
        array: Warp array

    Returns:
        TensorFlow tensor

    Example:

    .. code-block:: python

        >>> import isaacsim.core.utils.interops as interops_utils
        >>> import warp as wp
        >>>
        >>> wp.init()  # doctest: +NO_CHECK
        >>> warp_array = wp.zeros((100, 10), dtype=wp.float32, device="cuda:0")
        >>> tensorflow_tensor = interops_utils.warp2tensorflow(warp_array)
        >>> type(tensorflow_tensor)
        <class 'tensorflow.python...Tensor'>
    """
    global warp2tensorflow, _warp_to_dlpack, _tensorflow_from_dlpack  # noqa: F811
    # get source dlpack function
    if _warp_to_dlpack is None:
        import warp

        _warp_to_dlpack = warp.to_dlpack
    # get target dlpack function
    if _tensorflow_from_dlpack is None:
        import tensorflow

        _tensorflow_from_dlpack = tensorflow.experimental.dlpack.from_dlpack

    warp2tensorflow = lambda array: _tensorflow_from_dlpack(_warp_to_dlpack(array))  # noqa: E731
    return warp2tensorflow(array)


def warp2numpy(array: "warp.array") -> "numpy.ndarray":  # noqa: F821, F811
    """Convert Warp array to NumPy array.

    Args:
        array: Warp array

    Returns:
        NumPy array

    Example:

    .. code-block:: python

        >>> import isaacsim.core.utils.interops as interops_utils
        >>> import warp as wp
        >>>
        >>> wp.init()  # doctest: +NO_CHECK
        >>> warp_array = wp.zeros((100, 10), dtype=wp.float32, device="cuda:0")
        >>> numpy_array = interops_utils.warp2numpy(warp_array)
        >>> type(numpy_array)
        <class 'numpy.ndarray'>
    """
    global warp2numpy  # noqa: F811
    warp2numpy = lambda array: array.numpy()  # noqa: E731
    return warp2numpy(array)


# PyTorch


def torch2warp(tensor: "torch.Tensor") -> "warp.array":  # noqa: F821, F811
    """Convert PyTorch tensor to Warp array.

    Args:
        tensor: PyTorch tensor

    Returns:
        Warp array

    Example:

    .. code-block:: python

        >>> import isaacsim.core.utils.interops as interops_utils
        >>> import torch
        >>>
        >>> torch_tensor = torch.zeros((100, 10), dtype=torch.float32, device=torch.device("cuda:0"))
        >>> warp_array = interops_utils.torch2warp(torch_tensor)
        >>> type(warp_array)
        <class 'warp._src.types.array'>
    """
    global torch2warp, _torch_to_warp, _torch_to_dlpack, _warp_from_dlpack  # noqa: F811
    # warp-torch interop
    if _c_warp_torch_interop:
        # get conversion function
        if _torch_to_warp is None:
            import warp

            _torch_to_warp = warp.from_torch

        torch2warp = lambda tensor: _torch_to_warp(tensor.contiguous(), requires_grad=False)  # noqa: E731
    # dlpack
    else:
        # get source dlpack function
        if _torch_to_dlpack is None:
            import torch

            _torch_to_dlpack = torch.to_dlpack
        # get target dlpack function
        if _warp_from_dlpack is None:
            import warp

            _warp_from_dlpack = warp.from_dlpack

        torch2warp = lambda tensor: _warp_from_dlpack(_torch_to_dlpack(tensor.contiguous()))  # noqa: E731
    return torch2warp(tensor)


def torch2jax(tensor: "torch.Tensor") -> "jax.Array":  # noqa: F821, F811
    """Convert PyTorch tensor to JAX array.

    Args:
        tensor: PyTorch tensor

    Returns:
        JAX array

    Example:

    .. code-block:: python

        >>> import isaacsim.core.utils.interops as interops_utils
        >>> import torch
        >>>
        >>> torch_tensor = torch.zeros((100, 10), dtype=torch.float32, device=torch.device("cuda:0"))
        >>> jax_array = interops_utils.torch2jax(torch_tensor)
        >>> type(jax_array)
        <class 'jaxlib.xla_extension.Array...'>
    """
    global torch2jax, _torch_to_dlpack, _jax_from_dlpack  # noqa: F811
    # get source dlpack function
    if _torch_to_dlpack is None:
        import torch

        _torch_to_dlpack = torch.to_dlpack
    # get target dlpack function
    if _jax_from_dlpack is None:
        import jax.dlpack

        _jax_from_dlpack = jax.dlpack.from_dlpack

    torch2jax = lambda tensor: _jax_from_dlpack(_torch_to_dlpack(tensor.contiguous()))  # noqa: E731
    return torch2jax(tensor)


def torch2tensorflow(tensor: "torch.Tensor") -> "tensorflow.Tensor":  # noqa: F821, F811
    """Convert PyTorch tensor to TensorFlow tensor.

    Args:
        tensor: PyTorch tensor

    Returns:
        TensorFlow tensor

    Example:

    .. code-block:: python

        >>> import isaacsim.core.utils.interops as interops_utils
        >>> import torch
        >>>
        >>> torch_tensor = torch.zeros((100, 10), dtype=torch.float32, device=torch.device("cuda:0"))
        >>> tensorflow_tensor = interops_utils.torch2tensorflow(torch_tensor)
        >>> type(tensorflow_tensor)
        <class 'tensorflow.python...Tensor'>
    """
    global torch2tensorflow, _torch_to_dlpack, _tensorflow_from_dlpack  # noqa: F811
    # get source dlpack function
    if _torch_to_dlpack is None:
        import torch

        _torch_to_dlpack = torch.to_dlpack
    # get target dlpack function
    if _tensorflow_from_dlpack is None:
        import tensorflow

        _tensorflow_from_dlpack = tensorflow.experimental.dlpack.from_dlpack

    torch2tensorflow = lambda tensor: _tensorflow_from_dlpack(_torch_to_dlpack(tensor.contiguous()))  # noqa: E731
    return torch2tensorflow(tensor)


def torch2numpy(tensor: "torch.Tensor") -> "numpy.ndarray":  # noqa: F821, F811
    """Convert PyTorch tensor to NumPy array.

    Args:
        tensor: PyTorch tensor

    Returns:
        NumPy array

    Example:

    .. code-block:: python

        >>> import isaacsim.core.utils.interops as interops_utils
        >>> import torch
        >>>
        >>> torch_tensor = torch.zeros((100, 10), dtype=torch.float32, device=torch.device("cuda:0"))
        >>> numpy_array = interops_utils.torch2numpy(torch_tensor)
        >>> print(type(numpy_array))
        <class 'numpy.ndarray'>
    """
    global torch2numpy  # noqa: F811
    torch2numpy = lambda tensor: tensor.numpy(force=True)  # noqa: E731
    return torch2numpy(tensor)


# JAX


def jax2warp(array: "jax.Array") -> "warp.array":  # noqa: F821, F811
    """Convert JAX array to Warp array.

    Args:
        array: JAX array

    Returns:
        Warp array

    Example:

    .. code-block:: python

        >>> import isaacsim.core.utils.interops as interops_utils
        >>> import jax
        >>> import jax.numpy as jnp
        >>>
        >>> with jax.default_device(jax.devices("cuda")[0]):
        ...     jax_array = jnp.zeros((100, 10), dtype=jnp.float32)
        ...
        >>> warp_array = interops_utils.jax2warp(jax_array)
        >>> type(warp_array)
        <class 'warp._src.types.array'>
    """
    global jax2warp, _jax_to_dlpack, _warp_from_dlpack  # noqa: F811
    # get source dlpack function
    if _jax_to_dlpack is None:
        import jax.dlpack

        _jax_to_dlpack = jax.dlpack.to_dlpack
    # get target dlpack function
    if _warp_from_dlpack is None:
        import warp

        _warp_from_dlpack = warp.from_dlpack

    jax2warp = lambda array: _warp_from_dlpack(_jax_to_dlpack(array))  # noqa: E731
    return jax2warp(array)


def jax2torch(array: "jax.Array") -> "torch.Tensor":  # noqa: F821, F811
    """Convert JAX array to PyTorch tensor.

    Args:
        array: JAX array

    Returns:
        PyTorch tensor

    Example:

    .. code-block:: python

        >>> import isaacsim.core.utils.interops as interops_utils
        >>> import jax
        >>> import jax.numpy as jnp
        >>>
        >>> with jax.default_device(jax.devices("cuda")[0]):
        ...     jax_array = jnp.zeros((100, 10), dtype=jnp.float32)
        ...
        >>> torch_tensor = interops_utils.jax2torch(jax_array)
        >>> type(torch_tensor)
        <class 'torch.Tensor'>
    """
    global jax2torch, _jax_to_dlpack, _torch_from_dlpack  # noqa: F811
    # get source dlpack function
    if _jax_to_dlpack is None:
        import jax.dlpack

        _jax_to_dlpack = jax.dlpack.to_dlpack
    # get target dlpack function
    if _torch_from_dlpack is None:
        import torch

        _torch_from_dlpack = torch.from_dlpack

    jax2torch = lambda array: _torch_from_dlpack(_jax_to_dlpack(array))  # noqa: E731
    return jax2torch(array)


def jax2tensorflow(array: "jax.Array") -> "tensorflow.Tensor":  # noqa: F821, F811
    """Convert JAX array to TensorFlow tensor.

    Args:
        array: JAX array

    Returns:
        TensorFlow tensor

    Example:

    .. code-block:: python

        >>> import isaacsim.core.utils.interops as interops_utils
        >>> import jax
        >>> import jax.numpy as jnp
        >>>
        >>> with jax.default_device(jax.devices("cuda")[0]):
        ...     jax_array = jnp.zeros((100, 10), dtype=jnp.float32)
        ...
        >>> tensorflow_tensor = interops_utils.jax2tensorflow(jax_array)
        >>> type(tensorflow_tensor)
        <class 'tensorflow.python...Tensor'>
    """
    global jax2tensorflow, _jax_to_dlpack, _tensorflow_from_dlpack  # noqa: F811
    # get source dlpack function
    if _jax_to_dlpack is None:
        import jax.dlpack

        _jax_to_dlpack = jax.dlpack.to_dlpack
    # get target dlpack function
    if _tensorflow_from_dlpack is None:
        import tensorflow

        _tensorflow_from_dlpack = tensorflow.experimental.dlpack.from_dlpack

    jax2tensorflow = lambda array: _tensorflow_from_dlpack(_jax_to_dlpack(array))  # noqa: E731
    return jax2tensorflow(array)


def jax2numpy(array: "jax.Array") -> "numpy.ndarray":  # noqa: F821, F811
    """Convert JAX array to NumPy array.

    Args:
        array: JAX array

    Returns:
        NumPy array

    Example:

    .. code-block:: python

        >>> import isaacsim.core.utils.interops as interops_utils
        >>> import jax
        >>> import jax.numpy as jnp
        >>>
        >>> with jax.default_device(jax.devices("cuda")[0]):
        ...     jax_array = jnp.zeros((100, 10), dtype=jnp.float32)
        ...
        >>> numpy_array = interops_utils.jax2numpy(jax_array)
        >>> type(numpy_array)
        <class 'numpy.ndarray'>
    """
    global jax2numpy  # noqa: F811
    jax2numpy = lambda array: array.__array__()  # noqa: E731
    return jax2numpy(array)


# TensorFlow


def tensorflow2warp(tensor: "tensorflow.Tensor") -> "warp.array":  # noqa: F821, F811
    """Convert TensorFlow tensor to Warp array.

    Args:
        tensor: TensorFlow tensor

    Returns:
        Warp array

    .. warning::

        It is expected that the conversion of ``int32`` TensorFlow tensors to other backends will be on the CPU,
        regardless of which context device is specified (see TensorFlow issues #34071, #41307, #46833)

    Example:

    .. code-block:: python

        >>> import isaacsim.core.utils.interops as interops_utils
        >>> import tensorflow as tf  # doctest: +NO_CHECK
        >>>
        >>> with tf.device("/GPU:0"):
        ...     tensorflow_tensor = tf.zeros((100, 10), dtype=tf.float32)
        ...
        >>> warp_array = interops_utils.tensorflow2warp(tensorflow_tensor)
        >>> type(warp_array)
        <class 'warp._src.types.array'>
    """
    global tensorflow2warp, _tensorflow_to_dlpack, _warp_from_dlpack  # noqa: F811
    # get source dlpack function
    if _tensorflow_to_dlpack is None:
        import tensorflow

        _tensorflow_to_dlpack = tensorflow.experimental.dlpack.to_dlpack
    # get target dlpack function
    if _warp_from_dlpack is None:
        import warp

        _warp_from_dlpack = warp.from_dlpack

    tensorflow2warp = lambda tensor: _warp_from_dlpack(_tensorflow_to_dlpack(tensor))  # noqa: E731
    return tensorflow2warp(tensor)


def tensorflow2torch(tensor: "tensorflow.Tensor") -> "torch.Tensor":  # noqa: F821, F811
    """Convert TensorFlow tensor to PyTorch tensor.

    Args:
        tensor: TensorFlow tensor

    Returns:
        PyTorch tensor

    .. warning::

        It is expected that the conversion of ``int32`` TensorFlow tensors to other backends will be on the CPU,
        regardless of which context device is specified (see TensorFlow issues #34071, #41307, #46833)

    Example:

    .. code-block:: python

        >>> import isaacsim.core.utils.interops as interops_utils
        >>> import tensorflow as tf  # doctest: +NO_CHECK
        >>>
        >>> with tf.device("/GPU:0"):
        ...     tensorflow_tensor = tf.zeros((100, 10), dtype=tf.float32)
        ...
        >>> torch_tensor = interops_utils.tensorflow2torch(tensorflow_tensor)
        >>> type(torch_tensor)
        <class 'torch.Tensor'>
    """
    global tensorflow2torch, _tensorflow_to_dlpack, _torch_from_dlpack  # noqa: F811
    # get source dlpack function
    if _tensorflow_to_dlpack is None:
        import tensorflow

        _tensorflow_to_dlpack = tensorflow.experimental.dlpack.to_dlpack
    # get target dlpack function
    if _torch_from_dlpack is None:
        import torch

        _torch_from_dlpack = torch.from_dlpack

    tensorflow2torch = lambda tensor: _torch_from_dlpack(_tensorflow_to_dlpack(tensor))  # noqa: E731
    return tensorflow2torch(tensor)


def tensorflow2jax(tensor: "tensorflow.Tensor") -> "jax.Array":  # noqa: F821, F811
    """Convert TensorFlow tensor to JAX array.

    Args:
        tensor: TensorFlow tensor

    Returns:
        JAX array

    .. warning::

        It is expected that the conversion of ``int32`` TensorFlow tensors to other backends will be on the CPU,
        regardless of which context device is specified (see TensorFlow issues #34071, #41307, #46833)

    Example:

    .. code-block:: python

        >>> import isaacsim.core.utils.interops as interops_utils
        >>> import tensorflow as tf  # doctest: +NO_CHECK
        >>>
        >>> with tf.device("/GPU:0"):
        ...     tensorflow_tensor = tf.zeros((100, 10), dtype=tf.float32)
        ...
        >>> jax_array = interops_utils.tensorflow2jax(tensorflow_tensor)
        >>> type(jax_array)
        <class 'jaxlib.xla_extension.Array...'>
    """
    global tensorflow2jax, _tensorflow_to_dlpack, _jax_from_dlpack  # noqa: F811
    # get source dlpack function
    if _tensorflow_to_dlpack is None:
        import tensorflow

        _tensorflow_to_dlpack = tensorflow.experimental.dlpack.to_dlpack
    # get target dlpack function
    if _jax_from_dlpack is None:
        import jax.dlpack

        _jax_from_dlpack = jax.dlpack.from_dlpack

    tensorflow2jax = lambda tensor: _jax_from_dlpack(_tensorflow_to_dlpack(tensor))  # noqa: E731
    return tensorflow2jax(tensor)


def tensorflow2numpy(tensor: "tensorflow.Tensor") -> "numpy.ndarray":  # noqa: F821, F811
    """Convert TensorFlow tensor to NumPy array.

    Args:
        tensor: TensorFlow tensor

    Returns:
        NumPy array

    Example:

    .. code-block:: python

        >>> import isaacsim.core.utils.interops as interops_utils
        >>> import tensorflow as tf  # doctest: +NO_CHECK
        >>>
        >>> with tf.device("/GPU:0"):
        ...     tensorflow_tensor = tf.zeros((100, 10), dtype=tf.float32)
        ...
        >>> numpy_array = interops_utils.tensorflow2numpy(tensorflow_tensor)
        >>> type(numpy_array)
        <class 'numpy.ndarray'>
    """
    global tensorflow2numpy, _tensorflow_make_tensor_proto, _numpy_from_tensorflow, _numpy_array  # noqa: F811
    # get source internal conversion function
    if _tensorflow_make_tensor_proto is None:
        import tensorflow

        _tensorflow_make_tensor_proto = tensorflow.make_tensor_proto
    # get source conversion function
    if _numpy_from_tensorflow is None:
        import tensorflow

        _numpy_from_tensorflow = tensorflow.make_ndarray
    # get target conversion function
    if _numpy_array is None:
        import numpy

        _numpy_array = numpy.array

    # tensorflow2numpy = lambda tensor: tensorflow_to_numpy(tensorflow_make_tensor_proto(tensor))
    tensorflow2numpy = lambda tensor: _numpy_array(tensor)  # noqa: E731
    return tensorflow2numpy(tensor)


# NumPy


def numpy2warp(array: "numpy.ndarray") -> "warp.array":  # noqa: F821, F811
    """Convert NumPy array to Warp array.

    Args:
        array: NumPy array

    Returns:
        Warp array

    Example:

    .. code-block:: python

        >>> import isaacsim.core.utils.interops as interops_utils
        >>> import numpy as np
        >>>
        >>> numpy_array = np.zeros((100, 10), dtype=np.float32)
        >>> warp_array = interops_utils.numpy2warp(numpy_array)
        >>> type(warp_array)
        <class 'warp._src.types.array'>
    """
    global numpy2warp, _numpy_to_warp, _numpy_to_warp_dtype  # noqa: F811
    # get conversion function
    if _numpy_to_warp is None:
        import warp

        _numpy_to_warp = warp.from_numpy
        _numpy_to_warp_dtype = warp._src.types.np_dtype_to_warp_type

    numpy2warp = lambda array: _numpy_to_warp(
        array, dtype=_numpy_to_warp_dtype.get(array.dtype), device="cpu"
    )  # noqa: E731
    return numpy2warp(array)


def numpy2torch(array: "numpy.ndarray") -> "torch.Tensor":  # noqa: F821, F811
    """Convert NumPy array to PyTorch tensor.

    Args:
        array: NumPy array

    Returns:
        PyTorch tensor

    Example:

    .. code-block:: python

        >>> import isaacsim.core.utils.interops as interops_utils
        >>> import numpy as np
        >>>
        >>> numpy_array = np.zeros((100, 10), dtype=np.float32)
        >>> torch_tensor = interops_utils.numpy2torch(numpy_array)
        >>> type(torch_tensor)
        <class 'torch.Tensor'>
    """
    global numpy2torch, _numpy_to_torch  # noqa: F811
    # get conversion function
    if _numpy_to_torch is None:
        import torch

        _numpy_to_torch = torch.from_numpy

    numpy2torch = lambda array: _numpy_to_torch(array)  # noqa: E731
    return numpy2torch(array)


def numpy2jax(array: "numpy.ndarray") -> "jax.Array":  # noqa: F821, F811
    """Convert NumPy array to JAX array.

    Args:
        array: NumPy array

    Returns:
        JAX array

    Example:

    .. code-block:: python

        >>> import isaacsim.core.utils.interops as interops_utils
        >>> import numpy as np
        >>>
        >>> numpy_array = np.zeros((100, 10), dtype=np.float32)
        >>> jax_array = interops_utils.numpy2jax(numpy_array)
        >>> type(jax_array)
        <class 'jaxlib.xla_extension.ArrayImpl'>
    """
    global numpy2jax, _numpy_to_jax, _jax_cpu_device_context  # noqa: F811
    # get conversion function
    if _numpy_to_jax is None or _jax_cpu_device_context is None:
        import jax
        import jax.numpy as jnp

        _numpy_to_jax = jnp.asarray
        _jax_cpu_device_context = jax.devices("cpu")[0]  # unable to jax.default_device(jax.devices("cpu")[0])

    def _numpy2jax(array: "numpy.ndarray") -> "jax.Array":  # noqa: F821
        with jax.default_device(_jax_cpu_device_context):
            return _numpy_to_jax(array)

    numpy2jax = _numpy2jax
    return numpy2jax(array)


def numpy2tensorflow(array: "numpy.ndarray") -> "tensorflow.Tensor":  # noqa: F821, F811
    """Convert NumPy array to TensorFlow tensor.

    Args:
        array: NumPy array

    Returns:
        TensorFlow tensor

    Example:

    .. code-block:: python

        >>> import isaacsim.core.utils.interops as interops_utils
        >>> import numpy as np
        >>>
        >>> numpy_array = np.zeros((100, 10), dtype=np.float32)
        >>> tensorflow_tensor = interops_utils.numpy2tensorflow(numpy_array)
        >>> type(tensorflow_tensor)
        <class 'tensorflow.python...Tensor'>
    """
    global numpy2tensorflow, _numpy_to_tensorflow, _tensorflow_cpu_device_context  # noqa: F811
    # get conversion function
    if _numpy_to_tensorflow is None or _tensorflow_cpu_device_context is None:
        import tensorflow

        _numpy_to_tensorflow = tensorflow.convert_to_tensor
        _tensorflow_cpu_device_context = tensorflow.device("/CPU")

    def _numpy2tensorflow(array: "numpy.ndarray") -> "tensorflow.Tensor":  # noqa: F821
        with _tensorflow_cpu_device_context:
            return _numpy_to_tensorflow(array)

    numpy2tensorflow = _numpy2tensorflow
    return numpy2tensorflow(array)
