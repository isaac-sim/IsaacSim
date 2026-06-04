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

"""Coordinate system utilities for teleop pose transformations.

Single source of truth for the OpenXR → Isaac Sim axis conversion.

The full axis remapping is::

    x_iss = -z_oxr   (OpenXR Z- forward  → Isaac Sim X+ forward)
    y_iss = -x_oxr   (OpenXR X+ right    → Isaac Sim Y- right / left)
    z_iss =  y_oxr   (OpenXR Y+ up       → Isaac Sim Z+ up)

Expressed as:
    * **Rotation matrix** - :data:`OXR_TO_ISS_ROTATION` (120° around axis (1,-1,-1)/√3)
    * **Quaternion** - :data:`OXR_TO_ISS_QUAT` = (0.5, -0.5, -0.5, 0.5) in (x, y, z, w)
"""

from enum import Enum

import numpy as np

# -- Axis conversion: OpenXR (Y-up) → Isaac Sim (Z-up) -------------------

OXR_TO_ISS_ROTATION: np.ndarray = np.array(
    [
        [0.0, 0.0, -1.0],
        [-1.0, 0.0, 0.0],
        [0.0, 1.0, 0.0],
    ],
    dtype=np.float64,
)
"""3x3 rotation matrix for the OpenXR → Isaac Sim basis change."""

OXR_TO_ISS_QUAT: tuple[float, float, float, float] = (0.5, -0.5, -0.5, 0.5)
"""Quaternion (x, y, z, w) equivalent of :data:`OXR_TO_ISS_ROTATION`."""

_CONV_QUAT = OXR_TO_ISS_QUAT


class CoordinateSystem(Enum):
    """Target coordinate system for pose data output.

    Determines which conversion is applied to raw VR input data.
    The source is always raw OpenXR (Y-up, right-handed).
    """

    # No conversion - raw VR data passed through unchanged
    RAW = "raw"
    # Convert to Isaac Sim / USD convention (Z-up, right-handed)
    ISAAC_SIM = "isaac_sim"


def transform_pose_openxr_to_isaacsim(
    position: tuple[float, float, float],
    orientation: tuple[float, float, float, float] | None = None,
) -> tuple[tuple[float, float, float], tuple[float, float, float, float] | None]:
    """Transform pose from OpenXR (Y-up) to Isaac Sim (Z-up) coordinate system.

    OpenXR: X+ = right, Y+ = up, Z- = forward
    Isaac Sim: X+ = forward, Y+ = left, Z+ = up

    Position conversion:
        x_iss = -z_oxr   (forward)
        y_iss = -x_oxr   (left)
        z_iss =  y_oxr   (up)

    Orientation: q_out = q_conv * q_in, where q_conv = (0.5, -0.5, -0.5, 0.5)
    encodes the same axis remapping as the position conversion.

    Args:
        position: (x, y, z) position in OpenXR coordinates.
        orientation: (x, y, z, w) quaternion in OpenXR coordinates, or None.

    Returns:
        Tuple of (converted_position, converted_orientation).
    """
    x_oxr, y_oxr, z_oxr = position
    pos_isaacsim = (-z_oxr, -x_oxr, y_oxr)

    if orientation is None:
        return pos_isaacsim, None

    conv_x, conv_y, conv_z, conv_w = _CONV_QUAT

    # Input quaternion
    qx, qy, qz, qw = orientation

    # q_out = q_conv * q_in
    # Using Hamilton product
    out_w = conv_w * qw - conv_x * qx - conv_y * qy - conv_z * qz
    out_x = conv_w * qx + conv_x * qw + conv_y * qz - conv_z * qy
    out_y = conv_w * qy - conv_x * qz + conv_y * qw + conv_z * qx
    out_z = conv_w * qz + conv_x * qy - conv_y * qx + conv_z * qw

    orient_isaacsim = (out_x, out_y, out_z, out_w)

    return pos_isaacsim, orient_isaacsim


def transform_pose(
    position: tuple[float, float, float],
    orientation: tuple[float, float, float, float] | None,
    target_system: CoordinateSystem,
) -> tuple[tuple[float, float, float], tuple[float, float, float, float] | None]:
    """Transform raw VR pose data to the target coordinate system.

    Input is always raw OpenXR (Y-up). The target determines which conversion
    is applied.

    Args:
        position: (x, y, z) position in raw VR coordinates.
        orientation: (x, y, z, w) quaternion in raw VR coordinates, or None.
        target_system: The target coordinate system to convert to.

    Returns:
        Tuple of (transformed_position, transformed_orientation).
    """
    if target_system == CoordinateSystem.ISAAC_SIM:
        return transform_pose_openxr_to_isaacsim(position, orientation)
    # RAW or unknown - pass through unchanged
    return position, orientation
