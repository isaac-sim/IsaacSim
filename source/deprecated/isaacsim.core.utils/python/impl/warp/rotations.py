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

"""Warp-based utilities for rotation operations and quaternion conversions."""

from __future__ import annotations

from typing import Any

import numpy as np
import warp as wp
from isaacsim.core.deprecation_manager import import_module
from pxr import Gf
from scipy.spatial.transform import Rotation

torch = import_module("torch")


def gf_quat_to_tensor(orientation: Gf.Quatd | Gf.Quatf | Gf.Quaternion, device: object = None) -> wp.array:
    """Converts a pxr Quaternion type to a torch array (scalar first).

    Args:
        orientation: Input quaternion from USD.
        device: Target device for the output array.

    Returns:
        Quaternion tensor.
    """
    quat = torch.zeros(4, dtype=torch.float32, device=device)
    quat[1:] = torch.tensor(orientation.GetImaginary(), dtype=torch.float32, device=device)
    quat[0] = orientation.GetReal()

    quat = wp.from_torch(quat)
    return quat


def euler_angles_to_quats(
    euler_angles: wp.array, degrees: bool = False, extrinsic: bool = True, device: object = None
) -> wp.array:
    """Vectorized version of converting euler angles to quaternion (scalar first).

    Args:
        euler_angles: Euler angles with shape (N, 3).
        degrees: True if degrees, False if radians.
        extrinsic: True if the euler angles follows the extrinsic angles
            convention (equivalent to ZYX ordering but returned in the reverse) and False if it follows
            the intrinsic angles conventions (equivalent to XYZ ordering).
        device: Target device for the output array.

    Returns:
        Quaternions representation of the angles (N, 4) - scalar first.
    """
    if extrinsic:
        order = "xyz"
    else:
        order = "XYZ"
    euler_torch = wp.to_torch(euler_angles)
    rot = Rotation.from_euler(order, euler_torch.cpu().numpy(), degrees=degrees)
    result = rot.as_quat()[:, [3, 0, 1, 2]]
    result = wp.array(result, dtype=wp.float32, device=device)

    return result


def rad2deg(radian_value: wp.array, device: object = None) -> wp.array:
    """Converts radian values to degrees.

    Args:
        radian_value: Input array containing radian values to convert.
        device: Target device for the output array.

    Returns:
        Array containing the converted degree values.
    """
    rad_torch = wp.to_torch(radian_value)
    rad_deg = torch.rad2deg(rad_torch).float().to(device)
    return wp.from_torch(rad_deg)


def deg2rad(degree_value: wp.array, device: object = None) -> wp.array:
    """Converts degree values to radians.

    Args:
        degree_value: Input array containing degree values to convert.
        device: Target device for the output array.

    Returns:
        Array containing the converted radian values.
    """
    degree_torch = wp.to_torch(degree_value)
    rad_torch = torch.deg2rad(degree_torch).float().to(device)
    return wp.from_torch(rad_torch)


@wp.kernel
def _xyzw2wxyz1(q: Any) -> None:
    """Warp kernel to convert quaternion from XYZW to WXYZ format for 1D arrays.

    Reorders quaternion components from (x, y, z, w) to (w, x, y, z) format in-place.

    Args:
        q: Input quaternion array in XYZW format to be converted to WXYZ format.
    """
    qx = q[0]
    qy = q[1]
    qz = q[2]
    qw = q[3]
    q[0] = qw
    q[1] = qx
    q[2] = qy
    q[3] = qz


wp.overload(_xyzw2wxyz1, {"q": wp.array(dtype=float)})
wp.overload(_xyzw2wxyz1, {"q": wp.indexedarray(dtype=float)})


@wp.kernel
def _xyzw2wxyz2(q: Any) -> None:
    """Warp kernel to convert quaternion from XYZW to WXYZ format for 2D arrays.

    Reorders quaternion components from (x, y, z, w) to (w, x, y, z) format in-place for each quaternion
    in the array.

    Args:
        q: Input 2D quaternion array in XYZW format to be converted to WXYZ format.
    """
    tid = wp.tid()
    qx = q[tid, 0]
    qy = q[tid, 1]
    qz = q[tid, 2]
    qw = q[tid, 3]
    q[tid, 0] = qw
    q[tid, 1] = qx
    q[tid, 2] = qy
    q[tid, 3] = qz


wp.overload(_xyzw2wxyz2, {"q": wp.array(dtype=float, ndim=2)})
wp.overload(_xyzw2wxyz2, {"q": wp.indexedarray(dtype=float, ndim=2)})


@wp.kernel
def _xyzw2wxyz3(q: Any) -> None:
    """Warp kernel to convert quaternion from XYZW to WXYZ format for 3D arrays.

    Reorders quaternion components from (x, y, z, w) to (w, x, y, z) format in-place for each quaternion
    in the 3D array.

    Args:
        q: Input 3D quaternion array in XYZW format to be converted to WXYZ format.
    """
    i, j = wp.tid()
    qx = q[i, j, 0]
    qy = q[i, j, 1]
    qz = q[i, j, 2]
    qw = q[i, j, 3]
    q[i, j, 0] = qw
    q[i, j, 1] = qx
    q[i, j, 2] = qy
    q[i, j, 3] = qz


wp.overload(_xyzw2wxyz3, {"q": wp.array(dtype=float, ndim=3)})
wp.overload(_xyzw2wxyz3, {"q": wp.indexedarray(dtype=float, ndim=3)})


@wp.kernel
def _wxyz2xyzw1(q: Any) -> None:
    """Warp kernel to convert quaternion from WXYZ to XYZW format for 1D arrays.

    Reorders quaternion components from (w, x, y, z) to (x, y, z, w) format in-place.

    Args:
        q: Input quaternion array in WXYZ format to be converted to XYZW format.
    """
    qw = q[0]
    qx = q[1]
    qy = q[2]
    qz = q[3]
    q[0] = qx
    q[1] = qy
    q[2] = qz
    q[3] = qw


wp.overload(_wxyz2xyzw1, {"q": wp.array(dtype=float)})
wp.overload(_wxyz2xyzw1, {"q": wp.indexedarray(dtype=float)})


@wp.kernel
def _wxyz2xyzw2(q: Any) -> None:
    """Warp kernel to convert quaternion from WXYZ to XYZW format for 2D arrays.

    Reorders quaternion components from (w, x, y, z) to (x, y, z, w) format in-place for each quaternion
    in the array.

    Args:
        q: Input 2D quaternion array in WXYZ format to be converted to XYZW format.
    """
    tid = wp.tid()
    qw = q[tid, 0]
    qx = q[tid, 1]
    qy = q[tid, 2]
    qz = q[tid, 3]
    q[tid, 0] = qx
    q[tid, 1] = qy
    q[tid, 2] = qz
    q[tid, 3] = qw


wp.overload(_wxyz2xyzw2, {"q": wp.array(dtype=float, ndim=2)})
wp.overload(_wxyz2xyzw2, {"q": wp.indexedarray(dtype=float, ndim=2)})


@wp.kernel
def _wxyz2xyzw3(q: Any) -> None:
    """Warp kernel to convert quaternion from WXYZ to XYZW format for 3D arrays.

    Reorders quaternion components from (w, x, y, z) to (x, y, z, w) format in-place for each quaternion
    in the 3D array.

    Args:
        q: Input 3D quaternion array in WXYZ format to be converted to XYZW format.
    """
    i, j = wp.tid()
    qw = q[i, j, 0]
    qx = q[i, j, 1]
    qy = q[i, j, 2]
    qz = q[i, j, 3]
    q[i, j, 0] = qx
    q[i, j, 1] = qy
    q[i, j, 2] = qz
    q[i, j, 3] = qw


wp.overload(_wxyz2xyzw3, {"q": wp.array(dtype=float, ndim=3)})
wp.overload(_wxyz2xyzw3, {"q": wp.indexedarray(dtype=float, ndim=3)})


def _roll_quaternion_components(q: wp.array | wp.indexedarray, shift: int) -> wp.array:
    if isinstance(q, wp.indexedarray):
        q = q.contiguous()
    return wp.from_torch(torch.roll(wp.to_torch(q), shift, dims=-1).contiguous())


def xyzw2wxyz(q: wp.array) -> wp.array:
    """Converts quaternion from XYZW (scalar last) to WXYZ (scalar first) format.

    Supports Warp arrays with 1D, 2D, or 3D shapes and preserves the input array device.

    Args:
        q: Quaternion array in XYZW format to convert to WXYZ format.

    Returns:
        The input array with quaternion components reordered to WXYZ format.
    """
    return _roll_quaternion_components(q, 1)


def wxyz2xyzw(q: wp.array) -> wp.array:
    """Converts quaternion from WXYZ (scalar first) to XYZW (scalar last) format.

    Supports Warp arrays with 1D, 2D, or 3D shapes and preserves the input array device.

    Args:
        q: Quaternion array in WXYZ format to convert to XYZW format.

    Returns:
        The input array with quaternion components reordered to XYZW format.
    """
    return _roll_quaternion_components(q, -1)


PI = wp.constant(np.pi)
