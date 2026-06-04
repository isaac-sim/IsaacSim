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

"""Functions for performing transform operations."""

from __future__ import annotations

from typing import Any

import numpy as np
import warp as wp

from . import ops as ops_utils


def rotation_matrix_to_quaternion(
    rotation_matrix: list | np.ndarray | wp.array,
    *,
    dtype: type | None = None,
    device: str | wp.Device | None = None,
) -> wp.array:
    """Convert rotation matrix to quaternion.

    Args:
        rotation_matrix: A 3x3 rotation matrix or batch of 3x3 rotation matrices with shape (..., 3, 3).
        dtype: Data type of the output array. If ``None``, the data type of the input is used.
        device: Device to place the output array on. If ``None``, the default device is used,
            unless the input is a Warp array (in which case the input device is used).

    Returns:
        Quaternion (w, x, y, z) or batch of quaternions with shape (..., 4).

    Example:

    .. code-block:: python

        >>> import isaacsim.core.experimental.utils.transform as transform_utils
        >>> import numpy as np
        >>>
        >>> # Identity matrix to quaternion
        >>> identity = np.eye(3)
        >>> quaternion = transform_utils.rotation_matrix_to_quaternion(identity)  # doctest: +NO_CHECK
        >>> quaternion.numpy()
        array([1., 0., 0., 0.])
    """
    rotation_matrix = ops_utils.place(rotation_matrix, dtype=dtype, device=device)

    # Handle single matrix case by adding batch dimension
    if rotation_matrix.ndim == 2:
        rotation_matrix = rotation_matrix.reshape((1, 3, 3))
        squeeze_output = True
    else:
        squeeze_output = False

    # Ensure we have proper 3x3 matrices
    if rotation_matrix.shape[-2:] != (3, 3):
        raise ValueError(f"Expected 3x3 rotation matrices, got shape {rotation_matrix.shape}")

    batch_shape = rotation_matrix.shape[:-2]
    batch_size = 1
    for dim in batch_shape:
        batch_size *= dim

    # Flatten batch dimensions
    rotation_matrix_flat = rotation_matrix.reshape((batch_size, 3, 3))
    output = wp.empty(shape=(batch_size, 4), dtype=rotation_matrix.dtype, device=rotation_matrix.device)

    wp.launch(
        _wk_rotation_matrix_to_quaternion,
        dim=batch_size,
        inputs=[rotation_matrix_flat, output],
        device=rotation_matrix.device,
    )

    # Reshape to original batch shape
    if squeeze_output:
        return output.reshape((4,))
    else:
        return output.reshape((*batch_shape, 4))


def euler_angles_to_rotation_matrix(
    euler_angles: list | np.ndarray | wp.array,
    *,
    degrees: bool = False,
    extrinsic: bool = True,
    dtype: type | None = None,
    device: str | wp.Device | None = None,
) -> wp.array:
    """Convert Euler angles to rotation matrix.

    Args:
        euler_angles: Euler angles or batch of Euler angles with shape (..., 3).
            Input order is always [X, Y, Z] = [roll, pitch, yaw] for both conventions.
        degrees: Whether passed angles are in degrees.
        extrinsic: True if the euler angles follows the extrinsic angles
            convention (rotation applied as Rz * Ry * Rx about fixed world axes) and False if it
            follows the intrinsic angles conventions (rotation applied as Rx * Ry * Rz about
            body-fixed axes).
        dtype: Data type of the output array. If ``None``, the data type of the input is used.
        device: Device to place the output array on. If ``None``, the default device is used,
            unless the input is a Warp array (in which case the input device is used).

    Returns:
        A 3x3 rotation matrix or batch of 3x3 rotation matrices with shape (..., 3, 3).

    Example:

    .. code-block:: python

        >>> import isaacsim.core.experimental.utils.transform as transform_utils
        >>> import numpy as np
        >>>
        >>> # Zero rotation
        >>> euler = np.array([0, 0, 0])
        >>> rotation_matrix = transform_utils.euler_angles_to_rotation_matrix(euler)  # doctest: +NO_CHECK
        >>> rotation_matrix.numpy()
        array([[1., 0., 0.],
               [0., 1., 0.],
               [0., 0., 1.]], dtype=float32)
    """
    euler_angles = ops_utils.place(euler_angles, dtype=dtype, device=device)

    # Ensure floating point dtype to avoid int64 trigonometric function issues
    if euler_angles.dtype not in [wp.float32, wp.float64]:
        euler_angles = wp.array(euler_angles.numpy(), dtype=wp.float32, device=euler_angles.device)

    # Handle single angle case by adding batch dimension
    if euler_angles.ndim == 1:
        euler_angles = euler_angles.reshape((1, 3))
        squeeze_output = True
    else:
        squeeze_output = False

    # Ensure we have proper 3-element angle vectors
    if euler_angles.shape[-1] != 3:
        raise ValueError(f"Expected 3-element Euler angles, got shape {euler_angles.shape}")

    batch_shape = euler_angles.shape[:-1]
    batch_size = 1
    for dim in batch_shape:
        batch_size *= dim

    # Flatten batch dimensions
    euler_angles_flat = euler_angles.reshape((batch_size, 3))
    output = wp.empty(shape=(batch_size, 3, 3), dtype=euler_angles.dtype, device=euler_angles.device)

    wp.launch(
        _wk_euler_angles_to_rotation_matrix,
        dim=batch_size,
        inputs=[euler_angles_flat, output, degrees, extrinsic],
        device=euler_angles.device,
    )

    # Reshape to original batch shape
    if squeeze_output:
        return output.reshape((3, 3))
    else:
        return output.reshape((*batch_shape, 3, 3))


def euler_angles_to_quaternion(
    euler_angles: list | np.ndarray | wp.array,
    *,
    degrees: bool = False,
    extrinsic: bool = True,
    dtype: type | None = None,
    device: str | wp.Device | None = None,
) -> wp.array:
    """Convert Euler angles to quaternion.

    Args:
        euler_angles: Euler angles or batch of Euler angles with shape (..., 3).
            Input order is always [X, Y, Z] = [roll, pitch, yaw] for both conventions.
        degrees: Whether input angles are in degrees.
        extrinsic: True if the euler angles follows the extrinsic angles
            convention (rotation applied as Rz * Ry * Rx about fixed world axes) and False if it
            follows the intrinsic angles conventions (rotation applied as Rx * Ry * Rz about
            body-fixed axes).
        dtype: Data type of the output array. If ``None``, the data type of the input is used.
        device: Device to place the output array on. If ``None``, the default device is used,
            unless the input is a Warp array (in which case the input device is used).

    Returns:
        Quaternion (w, x, y, z) or batch of quaternions with shape (..., 4).

    Example:

    .. code-block:: python

        >>> import isaacsim.core.experimental.utils.transform as transform_utils
        >>> import numpy as np
        >>>
        >>> # Convert 90 degree rotation around Y axis
        >>> euler = np.array([0, np.pi/2, 0])
        >>> quaternion = transform_utils.euler_angles_to_quaternion(euler)  # doctest: +NO_CHECK
        >>> quaternion.numpy()
        array([0.70710678, 0.        , 0.70710678, 0.        ])
    """
    rotation_matrix = euler_angles_to_rotation_matrix(
        euler_angles, degrees=degrees, extrinsic=extrinsic, dtype=dtype, device=device
    )
    return rotation_matrix_to_quaternion(rotation_matrix)


def quaternion_multiplication(
    first_quaternion: list | np.ndarray | wp.array,
    second_quaternion: list | np.ndarray | wp.array,
    *,
    dtype: type | None = None,
    device: str | wp.Device | None = None,
) -> wp.array:
    """Multiply two quaternions using Hamilton product.

    Quaternion multiplication is used for combining rotations.
    Input quaternions are in [w, x, y, z] format.

    Args:
        first_quaternion: First quaternion or batch of quaternions with shape (..., 4) in [w, x, y, z] format.
        second_quaternion: Second quaternion or batch of quaternions with shape (..., 4) in [w, x, y, z] format.
        dtype: Data type of the output array. If ``None``, the data type of the first input is used.
        device: Device to place the output array on. If ``None``, the default device is used,
            unless the input is a Warp array (in which case the input device is used).

    Returns:
        Result of quaternion multiplication with shape (..., 4).

    Example:

    .. code-block:: python

        >>> import isaacsim.core.experimental.utils.transform as transform_utils
        >>> import numpy as np
        >>>
        >>> # Identity quaternion multiplied with itself
        >>> identity = np.array([1.0, 0.0, 0.0, 0.0])
        >>> result = transform_utils.quaternion_multiplication(identity, identity)  # doctest: +NO_CHECK
        >>> result.numpy()
        array([1., 0., 0., 0.])
    """
    first_quaternion = ops_utils.place(first_quaternion, dtype=dtype, device=device)
    second_quaternion = ops_utils.place(second_quaternion, dtype=dtype, device=first_quaternion.device)

    # Handle single quaternion case by adding batch dimension
    if first_quaternion.ndim == 1:
        first_quaternion = first_quaternion.reshape((1, 4))
        squeeze_output = True
    else:
        squeeze_output = False

    if second_quaternion.ndim == 1:
        second_quaternion = second_quaternion.reshape((1, 4))

    # Ensure we have proper 4-element quaternions
    if first_quaternion.shape[-1] != 4:
        raise ValueError(f"Expected 4-element quaternions for first input, got shape {first_quaternion.shape}")
    if second_quaternion.shape[-1] != 4:
        raise ValueError(f"Expected 4-element quaternions for second input, got shape {second_quaternion.shape}")

    # Ensure both inputs have same batch shape
    if first_quaternion.shape != second_quaternion.shape:
        raise ValueError(
            f"Input quaternions must have same shape, got {first_quaternion.shape} and {second_quaternion.shape}"
        )

    batch_shape = first_quaternion.shape[:-1]
    batch_size = 1
    for dim in batch_shape:
        batch_size *= dim

    # Flatten batch dimensions
    first_quaternion_flat = first_quaternion.reshape((batch_size, 4))
    second_quaternion_flat = second_quaternion.reshape((batch_size, 4))
    output = wp.empty(shape=(batch_size, 4), dtype=first_quaternion.dtype, device=first_quaternion.device)

    wp.launch(
        _wk_quaternion_multiplication,
        dim=batch_size,
        inputs=[first_quaternion_flat, second_quaternion_flat, output],
        device=first_quaternion.device,
    )

    # Reshape to original batch shape
    if squeeze_output:
        return output.reshape((4,))
    else:
        return output.reshape((*batch_shape, 4))


def quaternion_conjugate(
    quaternion: list | np.ndarray | wp.array,
    *,
    dtype: type | None = None,
    device: str | wp.Device | None = None,
) -> wp.array:
    """Compute quaternion conjugate by negating the vector part.

    Quaternion conjugate is used to compute the inverse rotation.
    For unit quaternions, conjugate equals inverse.

    Args:
        quaternion: Quaternion or batch of quaternions with shape (..., 4) in [w, x, y, z] format.
        dtype: Data type of the output array. If ``None``, the data type of the input is used.
        device: Device to place the output array on. If ``None``, the default device is used,
            unless the input is a Warp array (in which case the input device is used).

    Returns:
        Conjugate quaternion with shape (..., 4).

    Example:

    .. code-block:: python

        >>> import isaacsim.core.experimental.utils.transform as transform_utils
        >>> import numpy as np
        >>>
        >>> # Conjugate of a simple rotation quaternion
        >>> quaternion = np.array([0.7071, 0.7071, 0.0, 0.0])  # 90 deg around X
        >>> conjugate = transform_utils.quaternion_conjugate(quaternion)  # doctest: +NO_CHECK
        >>> conjugate.numpy()
        array([ 0.7071, -0.7071,  0.    ,  0.    ])
    """
    quaternion = ops_utils.place(quaternion, dtype=dtype, device=device)

    # Handle single quaternion case by adding batch dimension
    if quaternion.ndim == 1:
        quaternion = quaternion.reshape((1, 4))
        squeeze_output = True
    else:
        squeeze_output = False

    # Ensure we have proper 4-element quaternions
    if quaternion.shape[-1] != 4:
        raise ValueError(f"Expected 4-element quaternions, got shape {quaternion.shape}")

    batch_shape = quaternion.shape[:-1]
    batch_size = 1
    for dim in batch_shape:
        batch_size *= dim

    # Flatten batch dimensions
    quaternion_flat = quaternion.reshape((batch_size, 4))
    output = wp.empty(shape=(batch_size, 4), dtype=quaternion.dtype, device=quaternion.device)

    wp.launch(
        _wk_quaternion_conjugate,
        dim=batch_size,
        inputs=[quaternion_flat, output],
        device=quaternion.device,
    )

    # Reshape to original batch shape
    if squeeze_output:
        return output.reshape((4,))
    else:
        return output.reshape((*batch_shape, 4))


def quaternion_to_rotation_matrix(
    quaternion: list | np.ndarray | wp.array,
    *,
    dtype: type | None = None,
    device: str | wp.Device | None = None,
) -> wp.array:
    """Convert quaternion to rotation matrix.

    Converts quaternions in [w, x, y, z] format to 3x3 rotation matrices
    using the standard quaternion-to-matrix formula.

    Args:
        quaternion: Quaternion or batch of quaternions with shape (..., 4) in [w, x, y, z] format.
        dtype: Data type of the output array. If ``None``, the data type of the input is used.
        device: Device to place the output array on. If ``None``, the default device is used,
            unless the input is a Warp array (in which case the input device is used).

    Returns:
        A 3x3 rotation matrix or batch of 3x3 rotation matrices with shape (..., 3, 3).

    Example:

    .. code-block:: python

        >>> import isaacsim.core.experimental.utils.transform as transform_utils
        >>> import numpy as np
        >>>
        >>> # Identity quaternion to rotation matrix
        >>> identity = np.array([1.0, 0.0, 0.0, 0.0])
        >>> rotation_matrix = transform_utils.quaternion_to_rotation_matrix(identity)  # doctest: +NO_CHECK
        >>> rotation_matrix.numpy()
        array([[1., 0., 0.],
               [0., 1., 0.],
               [0., 0., 1.]])
    """
    quaternion = ops_utils.place(quaternion, dtype=dtype, device=device)

    # Ensure we have proper 4-element quaternions
    if quaternion.shape[-1] != 4:
        raise ValueError(f"Expected 4-element quaternions, got shape {quaternion.shape}")

    # Handle single quaternion case and determine batch size
    if quaternion.ndim == 1:
        squeeze_output = True
        batch_size = 1
    else:
        squeeze_output = False
        batch_shape = quaternion.shape[:-1]
        batch_size = int(np.prod(batch_shape))

    output = wp.empty(shape=(batch_size, 3, 3), dtype=quaternion.dtype, device=quaternion.device)

    wp.launch(
        _wk_quaternion_to_rotation_matrix,
        dim=batch_size,
        inputs=[quaternion.reshape((-1, 4)), output],
        device=quaternion.device,
    )

    # Reshape to original batch shape
    if squeeze_output:
        return output.reshape((3, 3))
    else:
        return output.reshape((*batch_shape, 3, 3))


def quaternion_to_euler_angles(
    quaternion: list | np.ndarray | wp.array,
    *,
    degrees: bool = False,
    extrinsic: bool = True,
    dtype: type | None = None,
    device: str | wp.Device | None = None,
) -> wp.array:
    """Convert quaternion to Euler angles.

    Converts quaternions in [w, x, y, z] format to Euler angles. The output order matches
    :func:`isaacsim.core.utils.numpy.rotations.quat_to_euler_angles` for consistency.

    Args:
        quaternion: Quaternion or batch of quaternions with shape (..., 4) in [w, x, y, z] format.
        degrees: Whether to return angles in degrees.
        extrinsic: True if the euler angles follows the extrinsic angles
            convention (equivalent to ZYX ordering but returned in the reverse) and False if it follows
            the intrinsic angles conventions (equivalent to XYZ ordering).
        dtype: Data type of the output array. If ``None``, the data type of the input is used.
        device: Device to place the output array on. If ``None``, the default device is used,
            unless the input is a Warp array (in which case the input device is used).

    Returns:
        Euler angles with shape (..., 3). For extrinsic convention, order is [X, Y, Z] (roll, pitch, yaw).
        For intrinsic convention, order is [X, Y, Z] (roll, pitch, yaw).

    Example:

    .. code-block:: python

        >>> import isaacsim.core.experimental.utils.transform as transform_utils
        >>> import numpy as np
        >>>
        >>> # Identity quaternion to euler angles
        >>> identity = np.array([1.0, 0.0, 0.0, 0.0])
        >>> euler = transform_utils.quaternion_to_euler_angles(identity)  # doctest: +NO_CHECK
        >>> euler.numpy()
        array([0., 0., 0.])
    """
    quaternion = ops_utils.place(quaternion, dtype=dtype, device=device)

    # Ensure floating point dtype
    if quaternion.dtype not in [wp.float32, wp.float64]:
        quaternion = wp.array(quaternion.numpy(), dtype=wp.float32, device=quaternion.device)

    # Ensure we have proper 4-element quaternions
    if quaternion.shape[-1] != 4:
        raise ValueError(f"Expected 4-element quaternions, got shape {quaternion.shape}")

    # Handle single quaternion case by adding batch dimension
    if quaternion.ndim == 1:
        quaternion = quaternion.reshape((1, 4))
        squeeze_output = True
    else:
        squeeze_output = False

    batch_shape = quaternion.shape[:-1]
    batch_size = 1
    for dim in batch_shape:
        batch_size *= dim

    # Flatten batch dimensions
    quaternion_flat = quaternion.reshape((batch_size, 4))
    output = wp.empty(shape=(batch_size, 3), dtype=quaternion.dtype, device=quaternion.device)

    wp.launch(
        _wk_quaternion_to_euler_angles,
        dim=batch_size,
        inputs=[quaternion_flat, output, degrees, extrinsic],
        device=quaternion.device,
    )

    # Reshape to original batch shape
    if squeeze_output:
        return output.reshape((3,))
    else:
        return output.reshape((*batch_shape, 3))


def look_at_quaternion(
    eye: list | np.ndarray | wp.array,
    target: list | np.ndarray | wp.array,
    up: list | np.ndarray | wp.array | None = None,
    *,
    dtype: type | None = None,
    device: str | wp.Device | None = None,
) -> wp.array:
    """Compute the orientation quaternion for a look-at transform.

    Compute the quaternion that orients a frame at ``eye`` so that its
    negative-Z axis points toward ``target`` (OpenGL / USD camera convention).

    When the forward direction is nearly parallel to the given up vector,
    a perpendicular fallback is chosen automatically.

    The function supports batched inputs. When batch dimensions are present
    the shapes of ``eye`` and ``target`` must match. The ``up`` vector is
    broadcast if necessary.

    Args:
        eye: Observer position or batch of positions with shape (..., 3).
        target: Target position or batch of positions with shape (..., 3).
        up: World up vector with shape (3,) or matching batch shape (..., 3).
            Defaults to Z-up ``[0, 0, 1]``.
        dtype: Data type of the output array. If ``None``, the data type of the
            ``eye`` input is used.
        device: Device to place the output array on. If ``None``, the default
            device is used, unless the input is a Warp array (in which case the
            input device is used).

    Returns:
        Quaternion (w, x, y, z) or batch of quaternions with shape (..., 4).

    Example:

    .. code-block:: python

        >>> import isaacsim.core.experimental.utils.transform as transform_utils
        >>> import numpy as np
        >>>
        >>> # Camera at (5,5,5) looking at origin
        >>> quat = transform_utils.look_at_quaternion(
        ...     eye=np.array([5.0, 5.0, 5.0]),
        ...     target=np.array([0.0, 0.0, 0.0]),
        ... )  # doctest: +NO_CHECK
        >>> quat.numpy()  # doctest: +SKIP
        array([0.33985114, 0.1759199 , 0.42470822, 0.82047325])
    """
    eye_arr = ops_utils.place(eye, dtype=dtype, device=device)
    target_arr = ops_utils.place(target, dtype=dtype, device=eye_arr.device)

    # Determine batch vs single
    if eye_arr.ndim == 1:
        eye_arr = eye_arr.reshape((1, 3))
        target_arr = target_arr.reshape((1, 3))
        squeeze_output = True
    else:
        squeeze_output = False

    if eye_arr.shape != target_arr.shape:
        raise ValueError(f"eye and target must have the same shape, got {eye_arr.shape} and {target_arr.shape}")
    if eye_arr.shape[-1] != 3:
        raise ValueError(f"Expected 3-element vectors, got shape {eye_arr.shape}")

    batch_shape = eye_arr.shape[:-1]
    batch_size = 1
    for dim in batch_shape:
        batch_size *= dim

    # Handle up vector — broadcast single vector to batch size
    if up is None:
        up_np = np.tile(np.array([0.0, 0.0, 1.0], dtype=np.float32), (batch_size, 1))
        up_arr = ops_utils.place(up_np, dtype=eye_arr.dtype, device=eye_arr.device)
    else:
        up_arr = ops_utils.place(up, dtype=eye_arr.dtype, device=eye_arr.device)
        if up_arr.ndim == 1:
            up_np = np.tile(np.array(up_arr.numpy()), (batch_size, 1))
            up_arr = ops_utils.place(up_np, dtype=eye_arr.dtype, device=eye_arr.device)
        up_arr = up_arr.reshape((batch_size, 3))

    eye_flat = eye_arr.reshape((batch_size, 3))
    target_flat = target_arr.reshape((batch_size, 3))
    output = wp.empty(shape=(batch_size, 4), dtype=eye_arr.dtype, device=eye_arr.device)

    wp.launch(
        _wk_look_at_quaternion,
        dim=batch_size,
        inputs=[eye_flat, target_flat, up_arr, output],
        device=eye_arr.device,
    )

    if squeeze_output:
        return output.reshape((4,))
    else:
        return output.reshape((*batch_shape, 4))


def look_at_matrix(
    eye: "list | np.ndarray | Any",
    target: "list | np.ndarray | Any",
    up: "list | np.ndarray | Any | None" = None,
    *,
    epsilon: float = 1e-5,
) -> Any:
    """Compute the camera transform matrix (position + orientation) for a look-at.

    Returns the ``Gf.Matrix4d`` that places a camera at *eye* oriented so its
    negative-Z axis points toward *target* (USD / OpenGL camera convention).
    This is the inverse of the standard view (LookAt) matrix and can be used
    directly as a prim's local transform.

    When the forward direction (*target* − *eye*) is nearly parallel to *up*,
    a perpendicular fallback up vector is chosen automatically.

    Args:
        eye: Camera position with shape (3,). Accepts list, numpy array, or ``Gf.Vec3d``.
        target: Target position with shape (3,). Accepts list, numpy array, or ``Gf.Vec3d``.
        up: World up vector with shape (3,). Defaults to Z-up ``[0, 0, 1]``.
            Accepts list, numpy array, or ``Gf.Vec3d``.
        epsilon: Collinearity threshold. When the cross-product of the forward
            direction and *up* has length below this value, a fallback up vector
            is used.

    Returns:
        ``Gf.Matrix4d`` camera transform (position + orientation). This is **not**
        the view matrix — it is the prim world/local transform.

    Example:

    .. code-block:: python

        >>> import isaacsim.core.experimental.utils.transform as transform_utils
        >>> import numpy as np
        >>>
        >>> # Camera at (5, 5, 5) looking at the origin
        >>> mat = transform_utils.look_at_matrix(
        ...     eye=np.array([5.0, 5.0, 5.0]),
        ...     target=np.array([0.0, 0.0, 0.0]),
        ... )  # doctest: +NO_CHECK
        >>> from pxr import Gf
        >>> isinstance(mat, Gf.Matrix4d)
        True
    """
    from pxr import Gf

    def _to_vec3d(v: Any, fallback: list) -> Gf.Vec3d:
        if v is None:
            return Gf.Vec3d(*fallback)
        if isinstance(v, Gf.Vec3d):
            return v
        arr = np.asarray(v, dtype=np.float64).flatten()
        return Gf.Vec3d(float(arr[0]), float(arr[1]), float(arr[2]))

    eye_gf = _to_vec3d(eye, [0.0, 0.0, 0.0])
    target_gf = _to_vec3d(target, [0.0, 0.0, 0.0])
    up_gf = _to_vec3d(up, [0.0, 0.0, 1.0])

    # Collinearity check: if forward × up ≈ 0, try perpendicular fallbacks
    forward = target_gf - eye_gf
    if forward.GetCross(up_gf).GetLength() < epsilon:
        up_gf = Gf.Vec3d(0.0, 1.0, 0.0)
        if forward.GetCross(up_gf).GetLength() < epsilon:
            up_gf = Gf.Vec3d(1.0, 0.0, 0.0)

    return Gf.Matrix4d(1).SetLookAt(eye_gf, target_gf, up_gf).GetInverse()


"""
Custom Warp kernels for transform operations.
"""


@wp.kernel(enable_backward=False)
def _wk_rotation_matrix_to_quaternion(rotation_matrix: wp.array(ndim=3), output: wp.array(ndim=2)) -> None:
    """Convert rotation matrix to quaternion using Warp kernel.

    Args:
        rotation_matrix: Input rotation matrix array with shape (batch_size, 3, 3).
        output: Output array for quaternions with shape (batch_size, 4) in [w, x, y, z] format.
    """
    i = wp.tid()

    # Extract matrix elements
    m00, m01, m02 = rotation_matrix[i, 0, 0], rotation_matrix[i, 0, 1], rotation_matrix[i, 0, 2]
    m10, m11, m12 = rotation_matrix[i, 1, 0], rotation_matrix[i, 1, 1], rotation_matrix[i, 1, 2]
    m20, m21, m22 = rotation_matrix[i, 2, 0], rotation_matrix[i, 2, 1], rotation_matrix[i, 2, 2]

    # Type-safe constants derived from matrix elements
    zero = m00 - m00  # Type-safe zero
    one = zero + output.dtype(1.0)  # Type-safe one using output dtype
    two = one + one  # Type-safe two
    four = two + two  # Type-safe four
    quarter = one / four  # Type-safe 0.25

    trace = m00 + m11 + m22

    if trace > zero:
        # w is largest
        s = wp.sqrt(trace + one) * two  # s = 4 * w
        w = quarter * s
        x = (m21 - m12) / s
        y = (m02 - m20) / s
        z = (m10 - m01) / s
    elif m00 > m11 and m00 > m22:
        # x is largest
        s = wp.sqrt(one + m00 - m11 - m22) * two  # s = 4 * x
        w = (m21 - m12) / s
        x = quarter * s
        y = (m01 + m10) / s
        z = (m02 + m20) / s
    elif m11 > m22:
        # y is largest
        s = wp.sqrt(one + m11 - m00 - m22) * two  # s = 4 * y
        w = (m02 - m20) / s
        x = (m01 + m10) / s
        y = quarter * s
        z = (m12 + m21) / s
    else:
        # z is largest
        s = wp.sqrt(one + m22 - m00 - m11) * two  # s = 4 * z
        w = (m10 - m01) / s
        x = (m02 + m20) / s
        y = (m12 + m21) / s
        z = quarter * s

    output[i, 0] = w
    output[i, 1] = x
    output[i, 2] = y
    output[i, 3] = z


@wp.kernel(enable_backward=False)
def _wk_euler_angles_to_rotation_matrix(
    euler_angles: wp.array(ndim=2),
    output: wp.array(ndim=3),
    degrees: bool,
    extrinsic: bool,
) -> None:
    """Convert Euler angles to rotation matrix using Warp kernel.

    Input order is always [X, Y, Z] = [roll, pitch, yaw] for both conventions.

    Args:
        euler_angles: Euler angles or batch of Euler angles with shape (batch_size, 3).
        output: Output array for rotation matrices with shape (batch_size, 3, 3).
        degrees: Whether input angles are in degrees.
        extrinsic: Whether to use extrinsic convention (Rz * Ry * Rx) or intrinsic convention (Rx * Ry * Rz).
    """
    i = wp.tid()

    # Extract angles and ensure they are floating point for trigonometric functions
    # Use type-safe constants derived from input array elements (not output, which may be uninitialized)
    angle1 = euler_angles[i, 0]
    angle2 = euler_angles[i, 1]
    angle3 = euler_angles[i, 2]

    # Create type-safe constants from the input array (NOT output array which may contain NaN)
    zero = angle1 - angle1  # Type-safe zero derived from input

    # Convert to radians if needed
    if degrees:
        # Type-safe constants using the same pattern
        pi = zero + euler_angles.dtype(3.141592653589793)  # Type-safe pi
        one_eighty = zero + euler_angles.dtype(180.0)  # Type-safe 180
        deg_to_rad = pi / one_eighty  # Type-safe conversion factor
        angle1 = angle1 * deg_to_rad
        angle2 = angle2 * deg_to_rad
        angle3 = angle3 * deg_to_rad

    # Input is always [X, Y, Z] = [roll, pitch, yaw]
    roll = angle1
    pitch = angle2
    yaw = angle3

    cr = wp.cos(roll)
    sr = wp.sin(roll)
    cy = wp.cos(yaw)
    sy = wp.sin(yaw)
    cp = wp.cos(pitch)
    sp = wp.sin(pitch)

    if extrinsic:
        # Extrinsic ZYX rotation: R = Rz(yaw) * Ry(pitch) * Rx(roll)
        # Standard formula for extrinsic ZYX Euler angles
        output[i, 0, 0] = cp * cy
        output[i, 0, 1] = cy * sp * sr - cr * sy
        output[i, 0, 2] = sr * sy + cr * cy * sp
        output[i, 1, 0] = cp * sy
        output[i, 1, 1] = cr * cy + sp * sr * sy
        output[i, 1, 2] = cr * sp * sy - cy * sr
        output[i, 2, 0] = -sp
        output[i, 2, 1] = cp * sr
        output[i, 2, 2] = cp * cr
    else:
        # Intrinsic XYZ rotation: R = Rx(roll) * Ry(pitch) * Rz(yaw)
        output[i, 0, 0] = cp * cy
        output[i, 0, 1] = -cp * sy
        output[i, 0, 2] = sp
        output[i, 1, 0] = cy * sr * sp + cr * sy
        output[i, 1, 1] = cr * cy - sr * sp * sy
        output[i, 1, 2] = -cp * sr
        output[i, 2, 0] = -cr * cy * sp + sr * sy
        output[i, 2, 1] = cy * sr + cr * sp * sy
        output[i, 2, 2] = cr * cp


@wp.kernel(enable_backward=False)
def _wk_quaternion_multiplication(a: wp.array(ndim=2), b: wp.array(ndim=2), output: wp.array(ndim=2)) -> None:
    """Multiply two quaternions using Hamilton product.

    Args:
        a: First quaternion array with shape (batch_size, 4) in [w, x, y, z] format.
        b: Second quaternion array with shape (batch_size, 4) in [w, x, y, z] format.
        output: Output array for quaternion multiplication result with shape (batch_size, 4).
    """
    i = wp.tid()

    # Extract quaternion components [w, x, y, z]
    w1, x1, y1, z1 = a[i, 0], a[i, 1], a[i, 2], a[i, 3]
    w2, x2, y2, z2 = b[i, 0], b[i, 1], b[i, 2], b[i, 3]

    # Hamilton product formula
    ww = (z1 + x1) * (x2 + y2)
    yy = (w1 - y1) * (w2 + z2)
    zz = (w1 + y1) * (w2 - z2)
    xx = ww + yy + zz

    # Type-safe constants
    zero = w1 - w1  # Type-safe zero
    half = zero + output.dtype(0.5)  # Type-safe 0.5

    qq = half * (xx + (z1 - x1) * (x2 - y2))
    w = qq - ww + (z1 - y1) * (y2 - z2)
    x = qq - xx + (x1 + w1) * (x2 + w2)
    y = qq - yy + (w1 - x1) * (y2 + z2)
    z = qq - zz + (z1 + y1) * (w2 - x2)

    output[i, 0] = w
    output[i, 1] = x
    output[i, 2] = y
    output[i, 3] = z


@wp.kernel(enable_backward=False)
def _wk_quaternion_conjugate(q: wp.array(ndim=2), output: wp.array(ndim=2)) -> None:
    """Compute quaternion conjugate by negating the vector part.

    Args:
        q: Input quaternion array with shape (batch_size, 4) in [w, x, y, z] format.
        output: Output array for conjugate quaternions with shape (batch_size, 4).
    """
    i = wp.tid()

    # Quaternion conjugate: [w, -x, -y, -z]
    output[i, 0] = q[i, 0]  # w component unchanged
    output[i, 1] = -q[i, 1]  # negate x component
    output[i, 2] = -q[i, 2]  # negate y component
    output[i, 3] = -q[i, 3]  # negate z component


@wp.kernel(enable_backward=False)
def _wk_quaternion_to_rotation_matrix(quaternion: wp.array(ndim=2), output: wp.array(ndim=3)) -> None:
    """Convert quaternion to rotation matrix using standard formula.

    Args:
        quaternion: Input quaternion array with shape (batch_size, 4) in [w, x, y, z] format.
        output: Output array for rotation matrices with shape (batch_size, 3, 3).
    """
    i = wp.tid()

    # Extract quaternion components [w, x, y, z]
    w = quaternion[i, 0]
    x = quaternion[i, 1]
    y = quaternion[i, 2]
    z = quaternion[i, 3]

    # Type-safe constants
    zero = w - w  # Type-safe zero
    one = zero + output.dtype(1.0)  # Type-safe one using output dtype
    two = one + one  # Type-safe two

    # Compute squared components
    sqx = x * x
    sqy = y * y
    sqz = z * z
    sqw = w * w

    # Normalization factor
    s = one / (sqx + sqy + sqz + sqw)

    # Standard quaternion to rotation matrix formula
    output[i, 0, 0] = one - two * s * (sqy + sqz)
    output[i, 0, 1] = two * s * (x * y - z * w)
    output[i, 0, 2] = two * s * (x * z + y * w)
    output[i, 1, 0] = two * s * (x * y + z * w)
    output[i, 1, 1] = one - two * s * (sqx + sqz)
    output[i, 1, 2] = two * s * (y * z - x * w)
    output[i, 2, 0] = two * s * (x * z - y * w)
    output[i, 2, 1] = two * s * (y * z + x * w)
    output[i, 2, 2] = one - two * s * (sqx + sqy)


@wp.kernel(enable_backward=False)
def _wk_quaternion_to_euler_angles(
    quaternion: wp.array(ndim=2),
    output: wp.array(ndim=2),
    degrees: bool,
    extrinsic: bool,
) -> None:
    """Convert quaternion to Euler angles using Warp kernel.

    For extrinsic convention, output order is [X, Y, Z] = [roll, pitch, yaw].
    For intrinsic convention, output order is [X, Y, Z] = [roll, pitch, yaw].

    Args:
        quaternion: Input quaternion array with shape (batch_size, 4) in [w, x, y, z] format.
        output: Output array for Euler angles with shape (batch_size, 3).
        degrees: Whether to return angles in degrees.
        extrinsic: Whether to use extrinsic convention (ZYX) or intrinsic convention (XYZ).
    """
    i = wp.tid()

    # Extract quaternion components [w, x, y, z]
    w = quaternion[i, 0]
    x = quaternion[i, 1]
    y = quaternion[i, 2]
    z = quaternion[i, 3]

    # Type-safe constants
    zero = w - w  # Type-safe zero
    one = zero + output.dtype(1.0)  # Type-safe one
    two = one + one  # Type-safe two

    # Compute rotation matrix elements needed for euler angle extraction
    # Using the standard quaternion to rotation matrix formula
    sqx = x * x
    sqy = y * y
    sqz = z * z
    sqw = w * w

    # Normalization (for non-unit quaternions)
    inv_norm = one / (sqx + sqy + sqz + sqw)

    # Rotation matrix elements
    m00 = one - two * inv_norm * (sqy + sqz)
    m01 = two * inv_norm * (x * y - z * w)
    m02 = two * inv_norm * (x * z + y * w)
    m10 = two * inv_norm * (x * y + z * w)
    m11 = one - two * inv_norm * (sqx + sqz)
    m12 = two * inv_norm * (y * z - x * w)
    m20 = two * inv_norm * (x * z - y * w)
    m21 = two * inv_norm * (y * z + x * w)
    m22 = one - two * inv_norm * (sqx + sqy)

    # Extract euler angles
    if extrinsic:
        # For extrinsic ZYX convention: R = Rz(yaw) * Ry(pitch) * Rx(roll)
        # Output order is [X, Y, Z] = [roll, pitch, yaw]

        # pitch = -asin(m20)
        m20_clamped = wp.clamp(m20, -one, one)
        pitch = -wp.asin(m20_clamped)

        # Check for gimbal lock (pitch near +/- 90 degrees)
        cos_pitch = wp.cos(pitch)
        threshold = zero + output.dtype(1e-6)

        if wp.abs(cos_pitch) > threshold:
            # Normal case: no gimbal lock
            roll = wp.atan2(m21, m22)
            yaw = wp.atan2(m10, m00)
        else:
            # Gimbal lock: set roll to 0 and compute yaw
            roll = zero
            yaw = wp.atan2(-m01, m11)

        # Output in [X, Y, Z] order for extrinsic convention
        angle1 = roll  # X rotation (first element)
        angle2 = pitch  # Y rotation (second element)
        angle3 = yaw  # Z rotation (third element)
    else:
        # For intrinsic XYZ convention: R = Rx(roll) * Ry(pitch) * Rz(yaw)
        # Output order is [X, Y, Z] = [roll, pitch, yaw]

        # pitch = asin(m02)
        m02_clamped = wp.clamp(m02, -one, one)
        pitch = wp.asin(m02_clamped)

        cos_pitch = wp.cos(pitch)
        threshold = zero + output.dtype(1e-6)

        if wp.abs(cos_pitch) > threshold:
            roll = wp.atan2(-m12, m22)
            # For intrinsic XYZ, yaw uses m00 in the denominator (not m11)
            yaw = wp.atan2(-m01, m00)
        else:
            roll = zero
            # In gimbal lock (pitch ≈ ±90°), use m11 to avoid instability when m00 ≈ 0
            yaw = wp.atan2(m10, m11)

        # Output in [X, Y, Z] order for intrinsic convention
        angle1 = roll  # X rotation (first element)
        angle2 = pitch  # Y rotation (second element)
        angle3 = yaw  # Z rotation (third element)

    # Convert to degrees if requested
    if degrees:
        pi = zero + output.dtype(3.141592653589793)
        one_eighty = zero + output.dtype(180.0)
        rad_to_deg = one_eighty / pi
        angle1 = angle1 * rad_to_deg
        angle2 = angle2 * rad_to_deg
        angle3 = angle3 * rad_to_deg

    output[i, 0] = angle1
    output[i, 1] = angle2
    output[i, 2] = angle3


@wp.kernel(enable_backward=False)
def _wk_look_at_quaternion(
    eye: wp.array(ndim=2),
    target: wp.array(ndim=2),
    up: wp.array(ndim=2),
    output: wp.array(ndim=2),
) -> None:
    """Compute look-at orientation quaternion.

    The resulting rotation orients the negative-Z axis toward the target
    (OpenGL / USD camera convention).

    Args:
        eye: Observer positions with shape (batch_size, 3).
        target: Target positions with shape (batch_size, 3).
        up: World up vectors with shape (batch_size, 3).
        output: Output quaternions (w, x, y, z) with shape (batch_size, 4).
    """
    i = wp.tid()

    # Type-safe constants
    zero = eye[i, 0] - eye[i, 0]
    one = zero + output.dtype(1.0)
    eps = zero + output.dtype(1e-8)
    threshold = zero + output.dtype(0.99)
    four = one + one + one + one
    quarter = one / four

    # Forward = normalize(target - eye), then negate (camera looks along -Z)
    fx = target[i, 0] - eye[i, 0]
    fy = target[i, 1] - eye[i, 1]
    fz = target[i, 2] - eye[i, 2]
    f_len = wp.sqrt(fx * fx + fy * fy + fz * fz) + eps
    fx = fx / f_len
    fy = fy / f_len
    fz = fz / f_len

    # Up vector
    ux = up[i, 0]
    uy = up[i, 1]
    uz = up[i, 2]

    # If forward is nearly parallel to up, pick a perpendicular fallback
    dot_fu = wp.abs(fx * ux + fy * uy + fz * uz)
    if dot_fu > threshold:
        # Swap to Y-up fallback
        ux = zero
        uy = one
        uz = zero

    # right = normalize(cross(up, -forward))  = normalize(cross(up, -f))
    # -forward for camera convention
    neg_fx = -fx
    neg_fy = -fy
    neg_fz = -fz

    rx = uy * neg_fz - uz * neg_fy
    ry = uz * neg_fx - ux * neg_fz
    rz = ux * neg_fy - uy * neg_fx
    r_len = wp.sqrt(rx * rx + ry * ry + rz * rz) + eps
    rx = rx / r_len
    ry = ry / r_len
    rz = rz / r_len

    # recomputed up = cross(-forward, right)
    rux = neg_fy * rz - neg_fz * ry
    ruy = neg_fz * rx - neg_fx * rz
    ruz = neg_fx * ry - neg_fy * rx
    ru_len = wp.sqrt(rux * rux + ruy * ruy + ruz * ruz) + eps
    rux = rux / ru_len
    ruy = ruy / ru_len
    ruz = ruz / ru_len

    # Rotation matrix columns: [right, recomputed_up, -forward]
    # m[row][col]
    m00 = rx
    m10 = ry
    m20 = rz
    m01 = rux
    m11 = ruy
    m21 = ruz
    m02 = neg_fx
    m12 = neg_fy
    m22 = neg_fz

    # Rotation matrix to quaternion (same algorithm as _wk_rotation_matrix_to_quaternion)
    trace = m00 + m11 + m22
    two = one + one

    if trace > zero:
        s = wp.sqrt(trace + one) * two
        w = quarter * s
        x = (m21 - m12) / s
        y = (m02 - m20) / s
        z = (m10 - m01) / s
    elif m00 > m11 and m00 > m22:
        s = wp.sqrt(one + m00 - m11 - m22) * two
        w = (m21 - m12) / s
        x = quarter * s
        y = (m01 + m10) / s
        z = (m02 + m20) / s
    elif m11 > m22:
        s = wp.sqrt(one + m11 - m00 - m22) * two
        w = (m02 - m20) / s
        x = (m01 + m10) / s
        y = quarter * s
        z = (m12 + m21) / s
    else:
        s = wp.sqrt(one + m22 - m00 - m11) * two
        w = (m10 - m01) / s
        x = (m02 + m20) / s
        y = (m12 + m21) / s
        z = quarter * s

    output[i, 0] = w
    output[i, 1] = x
    output[i, 2] = y
    output[i, 3] = z


def compute_relative_transform(
    source_to_world: list | np.ndarray | wp.array,
    target_to_world: list | np.ndarray | wp.array,
) -> np.ndarray:
    """Compute the relative 4x4 transform from a source frame to a target frame given their world transforms.

    Both inputs are expected in USD row-major convention (as returned by
    ``UsdGeom.Xformable.ComputeLocalToWorldTransform``). The result is a column-major
    4x4 matrix that transforms points from the source local frame into the target local frame.

    Args:
        source_to_world: Row-major 4x4 world transform of the source frame.
        target_to_world: Row-major 4x4 world transform of the target frame.

    Returns:
        Column-major 4x4 relative transformation matrix.

    Example:

    .. code-block:: python

        >>> import isaacsim.core.experimental.utils.transform as transform_utils
        >>> import numpy as np
        >>>
        >>> identity = np.eye(4)
        >>> translated = np.eye(4)
        >>> translated[3, :3] = [1.0, 2.0, 3.0]  # USD row-major: translation in last row
        >>> result = transform_utils.compute_relative_transform(identity, translated)
        >>> result[:3, 3]  # column-major: translation in last column
        array([-1., -2., -3.])
    """
    if isinstance(source_to_world, wp.array):
        source_to_world = source_to_world.numpy()
    source_to_world = np.asarray(source_to_world, dtype=np.float64)

    if isinstance(target_to_world, wp.array):
        target_to_world = target_to_world.numpy()
    target_to_world = np.asarray(target_to_world, dtype=np.float64)

    source_col = np.transpose(source_to_world)
    target_col = np.transpose(target_to_world)
    world_to_target_col = np.linalg.inv(target_col)
    return world_to_target_col @ source_col
