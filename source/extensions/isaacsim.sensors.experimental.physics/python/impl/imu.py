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

"""Authoring class for IMU sensors (USD prim creation/wrapping)."""

from __future__ import annotations

from typing import Any

import numpy as np
import omni.isaac.IsaacSensorSchema as IsaacSensorSchema
import warp as wp

from ._sensor_base import _PhysicsSensorAuthoring
from .common import _create_sensor_prim


class IMU(_PhysicsSensorAuthoring):
    """Authoring wrapper for an Isaac IMU sensor USD prim.

    Creates or wraps an ``IsaacImuSensor`` prim. Use this class when you only
    need to author the prim (set transforms, configure filter widths) without
    bringing up the C++ backend. For data acquisition, use :class:`IMUSensor`.

    Args:
        path: USD path where the sensor should be located.
        positions: World-frame positions (shape ``(N, 3)``). Mutually exclusive with ``translations``.
        translations: Local-frame translations (shape ``(N, 3)``).
        orientations: Orientations as ``wxyz`` quaternions (shape ``(N, 4)``).
        scales: Local scales forwarded to ``XformPrim``.
        reset_xform_op_properties: Whether to reset existing xform op properties.
        linear_acceleration_filter_size: Rolling average window for acceleration.
            Applied only when creating a new prim; ignored when wrapping an existing one.
        angular_velocity_filter_size: Rolling average window for angular velocity.
            Applied only when creating a new prim; ignored when wrapping an existing one.
        orientation_filter_size: Rolling average window for orientation.
            Applied only when creating a new prim; ignored when wrapping an existing one.

    Example:

    .. code-block:: python

        >>> from isaacsim.sensors.experimental.physics import IMU
        >>>
        >>> imu = IMU.create(
        ...     "/World/Robot/body/imu",
        ...     translations=[[0.0, 0.0, 0.0]],
        ...     orientations=[[1.0, 0.0, 0.0, 0.0]],
        ...     linear_acceleration_filter_size=5,
        ... )  # doctest: +NO_CHECK
    """

    _PRIM_TYPE = "IsaacImuSensor"
    _SCHEMA_CLASS = IsaacSensorSchema.IsaacImuSensor

    def __init__(
        self,
        path: str,
        *,
        positions: list | np.ndarray | wp.array | None = None,
        translations: list | np.ndarray | wp.array | None = None,
        orientations: list | np.ndarray | wp.array | None = None,
        scales: list | np.ndarray | wp.array | None = None,
        reset_xform_op_properties: bool = True,
        linear_acceleration_filter_size: int | None = 1,
        angular_velocity_filter_size: int | None = 1,
        orientation_filter_size: int | None = 1,
    ) -> None:
        # Clamp filter sizes to at least 1 (preserve historical behavior)
        if linear_acceleration_filter_size is None:
            linear_acceleration_filter_size = 1
        if angular_velocity_filter_size is None:
            angular_velocity_filter_size = 1
        if orientation_filter_size is None:
            orientation_filter_size = 1
        linear_acceleration_filter_size = max(linear_acceleration_filter_size, 1)
        angular_velocity_filter_size = max(angular_velocity_filter_size, 1)
        orientation_filter_size = max(orientation_filter_size, 1)

        super().__init__(
            path,
            positions=positions,
            translations=translations,
            orientations=orientations,
            scales=scales,
            reset_xform_op_properties=reset_xform_op_properties,
            linear_acceleration_filter_size=linear_acceleration_filter_size,
            angular_velocity_filter_size=angular_velocity_filter_size,
            orientation_filter_size=orientation_filter_size,
        )

    def _create_prim(
        self,
        *,
        linear_acceleration_filter_size: int = 1,
        angular_velocity_filter_size: int = 1,
        orientation_filter_size: int = 1,
        **_: Any,
    ) -> IsaacSensorSchema.IsaacImuSensor:
        """Create a new IsaacImuSensor prim with default filter widths applied.

        Args:
            linear_acceleration_filter_size: Rolling average window for acceleration.
            angular_velocity_filter_size: Rolling average window for angular velocity.
            orientation_filter_size: Rolling average window for orientation.
            **_: Additional keyword arguments ignored by this hook.

        Returns:
            The schema-wrapped IMU sensor prim.
        """
        prim, _path = _create_sensor_prim(
            "/" + self._sensor_name,
            self._body_prim_path,
            IsaacSensorSchema.IsaacImuSensor,
        )
        prim.CreateLinearAccelerationFilterWidthAttr().Set(linear_acceleration_filter_size)
        prim.CreateAngularVelocityFilterWidthAttr().Set(angular_velocity_filter_size)
        prim.CreateOrientationFilterWidthAttr().Set(orientation_filter_size)
        return prim
