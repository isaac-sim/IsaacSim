# SPDX-FileCopyrightText: Copyright (c) 2018-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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

"""Omniverse extension entry point for physics-based sensors.

This module manages the lifecycle of:
- The _SensorStepManager singleton for contact sensor coordination
- The C++ IImuSensor Carbonite interface (acquired here, self-driven via physics events)
- The C++ IContactSensor Carbonite interface (acquired here, self-driven via physics events)
- The C++ IEffortSensor Carbonite interface (acquired here, self-driven via physics events)
"""

from __future__ import annotations

import carb
import omni

from .common import _SensorStepManager

__all__ = [
    "Extension",
    "get_imu_sensor_interface",
    "get_contact_sensor_interface",
    "get_effort_sensor_interface",
    "get_joint_state_sensor_interface",
    "get_raycast_sensor_interface",
]

EXTENSION_NAME = "Isaac Sensor"

_imu_interface = None
_contact_sensor_interface = None
_effort_sensor_interface = None
_joint_state_interface = None
_raycast_sensor_interface = None


def get_imu_sensor_interface() -> object | None:
    """Get the cached IImuSensor Carbonite interface.

    Returns:
        The IImuSensor interface, or None if not yet acquired.
    """
    return _imu_interface


def get_contact_sensor_interface() -> object | None:
    """Get the cached IContactSensor Carbonite interface.

    Returns:
        The IContactSensor interface, or None if not yet acquired.
    """
    return _contact_sensor_interface


def get_effort_sensor_interface() -> object | None:
    """Get the cached IEffortSensor Carbonite interface.

    Returns:
        The IEffortSensor interface, or None if not yet acquired.
    """
    return _effort_sensor_interface


def get_joint_state_sensor_interface() -> object | None:
    """Get the cached IJointStateSensor Carbonite interface.

    Returns:
        The IJointStateSensor interface, or None if not yet acquired.
    """
    return _joint_state_interface


def get_raycast_sensor_interface() -> object | None:
    """Get the cached IRaycastSensor Carbonite interface.

    Returns:
        The IRaycastSensor interface, or None if not yet acquired.
    """
    return _raycast_sensor_interface


class Extension(omni.ext.IExt):
    """Omniverse extension class for physics-based sensors.

    Acquires and releases the C++ IImuSensor and IContactSensor interfaces.
    The C++ plugins self-drive via physics simulation events (eResumed / eStopped)
    and physics step subscriptions, so no Python simulation callbacks are needed.
    """

    def on_startup(self, ext_id: str) -> None:
        """Initialize the extension when it is loaded.

        Args:
            ext_id: Extension identifier provided by the extension manager.
        """
        global _imu_interface, _contact_sensor_interface, _effort_sensor_interface, _joint_state_interface, _raycast_sensor_interface

        _SensorStepManager.instance()

        try:
            from .. import _physics_sensors

            _imu_interface = _physics_sensors.acquire_imu_sensor_interface()
        except Exception as e:
            carb.log_warn(f"Failed to acquire IImuSensor C++ interface: {e}")
            _imu_interface = None

        try:
            from .. import _physics_sensors

            _contact_sensor_interface = _physics_sensors.acquire_contact_sensor_interface()
        except Exception as e:
            carb.log_warn(f"Failed to acquire IContactSensor C++ interface: {e}")
            _contact_sensor_interface = None

        try:
            from .. import _physics_sensors

            _effort_sensor_interface = _physics_sensors.acquire_effort_sensor_interface()
        except Exception as e:
            carb.log_warn(f"Failed to acquire IEffortSensor C++ interface: {e}")
            _effort_sensor_interface = None

        try:
            from .. import _physics_sensors

            _joint_state_interface = _physics_sensors.acquire_joint_state_sensor_interface()
        except Exception as e:
            carb.log_warn(f"Failed to acquire IJointStateSensor C++ interface: {e}")
            _joint_state_interface = None

        try:
            from .. import _physics_sensors

            _raycast_sensor_interface = _physics_sensors.acquire_raycast_sensor_interface()
        except Exception as e:
            carb.log_warn(f"Failed to acquire IRaycastSensor C++ interface: {e}")
            _raycast_sensor_interface = None

    def on_shutdown(self) -> None:
        """Clean up resources when the extension is unloaded."""
        global _imu_interface, _contact_sensor_interface, _effort_sensor_interface, _joint_state_interface, _raycast_sensor_interface

        try:
            from .. import _physics_sensors
        except Exception:
            _physics_sensors = None

        if _raycast_sensor_interface is not None:
            _raycast_sensor_interface.shutdown()
            if _physics_sensors is not None:
                _physics_sensors.release_raycast_sensor_interface(_raycast_sensor_interface)
            _raycast_sensor_interface = None

        if _joint_state_interface is not None:
            _joint_state_interface.shutdown()
            if _physics_sensors is not None:
                _physics_sensors.release_joint_state_sensor_interface(_joint_state_interface)
            _joint_state_interface = None

        if _effort_sensor_interface is not None:
            _effort_sensor_interface.shutdown()
            if _physics_sensors is not None:
                _physics_sensors.release_effort_sensor_interface(_effort_sensor_interface)
            _effort_sensor_interface = None

        if _contact_sensor_interface is not None:
            _contact_sensor_interface.shutdown()
            if _physics_sensors is not None:
                _physics_sensors.release_contact_sensor_interface(_contact_sensor_interface)
            _contact_sensor_interface = None

        if _imu_interface is not None:
            _imu_interface.shutdown()
            if _physics_sensors is not None:
                _physics_sensors.release_imu_sensor_interface(_imu_interface)
            _imu_interface = None
