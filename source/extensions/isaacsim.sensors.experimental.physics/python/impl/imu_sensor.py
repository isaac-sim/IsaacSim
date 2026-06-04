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

"""IMU sensor runtime providing frame-based data access via the C++ interface."""

from __future__ import annotations

import numpy as np
from isaacsim.core.simulation_manager import SimulationManager

from ._sensor_base import _PhysicsSensorRuntime
from .imu import IMU

_INVALID_IMU_READING = None


def _get_invalid_imu_reading() -> object:
    global _INVALID_IMU_READING
    if _INVALID_IMU_READING is None:
        from .. import _physics_sensors

        _INVALID_IMU_READING = _physics_sensors.ImuSensorReading()
    return _INVALID_IMU_READING


class IMUSensor(_PhysicsSensorRuntime):
    """Runtime wrapper for an Isaac IMU sensor with frame-based data access.

    Wraps an :class:`IMU` authoring object and owns the C++ ``IImuSensor``
    Carbonite interface. Exposes :meth:`get_data` for a structured per-step
    dictionary and :meth:`get_sensor_reading` for the raw C++ struct.

    Args:
        path: Either a string USD path to an existing IsaacImuSensor prim, or a
            pre-built :class:`IMU` authoring object. To create a new prim, use
            :meth:`IMU.create`.

    Example:

    .. code-block:: python

        from isaacsim.sensors.experimental.physics import IMU, IMUSensor

        # Wrap an existing IsaacImuSensor prim
        sensor = IMUSensor("/World/Robot/body/imu")

        # Create a new sensor with custom parameters
        sensor = IMUSensor(
            IMU.create(
                "/World/Robot/body/imu",
                linear_acceleration_filter_size=5,
            )
        )

        frame = sensor.get_data()
        print(f"Linear acceleration: {frame['linear_acceleration']}")
    """

    _AUTHORING_CLASS = IMU
    _AUTHORING_ATTR = "_imu"

    @property
    def imu(self) -> IMU:
        """Authoring object encapsulated by this sensor.

        Returns:
            The :class:`IMU` instance wrapping the underlying USD prim.
        """
        return self._imu

    def _acquire_interface(self) -> object | None:
        from .extension import get_imu_sensor_interface

        return get_imu_sensor_interface()

    def _get_invalid_reading(self) -> object:
        return _get_invalid_imu_reading()

    def _init_frame(self) -> dict[str, object]:
        orientation_array = np.zeros((4,), dtype=np.float32)
        orientation_array[0] = 1.0  # Identity quaternion [w, x, y, z]
        return {
            "time": 0.0,
            "physics_step": 0.0,
            "linear_acceleration": np.zeros((3,), dtype=np.float32),
            "angular_velocity": np.zeros((3,), dtype=np.float32),
            "orientation": orientation_array,
        }

    def get_sensor_reading(self, read_gravity: bool = True) -> object:
        """Get the current IMU sensor reading as the raw C++ struct.

        Args:
            read_gravity: Whether to include gravity in the reading.

        Returns:
            The C++ ``ImuSensorReading`` struct directly. Access fields via
            ``reading.linear_acceleration_x`` / ``_y`` / ``_z``,
            ``reading.angular_velocity_x`` / ``_y`` / ``_z``, and
            ``reading.orientation_w`` / ``_x`` / ``_y`` / ``_z`` (no aggregate
            ``orientation`` accessor — read the four scalar fields). For a
            ``[w, x, y, z]`` numpy array, use :meth:`get_data` instead.
        """
        return self._get_reading(read_gravity)

    def get_data(self, read_gravity: bool = True) -> dict:
        """Get the current IMU sensor data as a structured frame.

        Args:
            read_gravity: If ``True``, include gravity in acceleration readings.

        Returns:
            Frame data containing:
                - ``"linear_acceleration"``: Linear acceleration ``[x, y, z]``.
                - ``"angular_velocity"``: Angular velocity ``[x, y, z]``.
                - ``"orientation"``: Orientation as ``[w, x, y, z]`` quaternion.
                - ``"time"``: Simulation time of reading.
                - ``"physics_step"``: Physics step number.
        """
        reading = self.get_sensor_reading(read_gravity=read_gravity)

        if reading.is_valid:
            linear_acceleration = self._current_frame["linear_acceleration"]
            linear_acceleration[0] = reading.linear_acceleration_x
            linear_acceleration[1] = reading.linear_acceleration_y
            linear_acceleration[2] = reading.linear_acceleration_z

            angular_velocity = self._current_frame["angular_velocity"]
            angular_velocity[0] = reading.angular_velocity_x
            angular_velocity[1] = reading.angular_velocity_y
            angular_velocity[2] = reading.angular_velocity_z

            orientation = self._current_frame["orientation"]
            orientation[0] = reading.orientation_w
            orientation[1] = reading.orientation_x
            orientation[2] = reading.orientation_y
            orientation[3] = reading.orientation_z

            self._current_frame["time"] = reading.time
            self._current_frame["physics_step"] = float(SimulationManager.get_num_physics_steps())

        return self._current_frame
