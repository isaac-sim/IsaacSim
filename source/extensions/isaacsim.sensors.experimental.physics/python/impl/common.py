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

"""Common utilities, data classes, and managers for physics-based sensors.

This module provides shared functionality for IMU and contact sensors including:
- Data classes for sensor readings (ContactSensorReading, IMUSensorReading, IMURawData)
- Quaternion representation with compatibility for existing APIs
- Utility functions for vector/quaternion operations
- Singleton managers for sensor lifecycle and contact events
"""

from __future__ import annotations

import weakref
from collections.abc import Iterator
from dataclasses import dataclass, field
from typing import Any

import numpy as np
import warp as wp
from isaacsim.core.experimental.utils import transform as transform_utils
from isaacsim.core.simulation_manager import SimulationEvent, SimulationManager


@dataclass
class ContactSensorReading:
    """Contact sensor reading data."""

    _in_contact: bool = field(default=False, init=False)
    """Internal state indicating whether the sensor is currently in contact."""
    value: float = 0.0
    """Contact force magnitude in Newtons."""
    time: float = 0.0
    """Simulation time when the reading was taken."""
    is_valid: bool = False
    """Whether this reading contains valid data."""

    @property
    def in_contact(self) -> bool:
        """Whether the sensor is currently in contact."""
        return self._in_contact

    @in_contact.setter
    def in_contact(self, value: bool) -> None:
        self._in_contact = value


@dataclass
class IMUSensorReading:
    """IMU sensor reading data."""

    linear_acceleration_x: float = 0.0
    """Linear acceleration along X axis in m/s^2."""
    linear_acceleration_y: float = 0.0
    """Linear acceleration along Y axis in m/s^2."""
    linear_acceleration_z: float = 0.0
    """Linear acceleration along Z axis in m/s^2."""
    angular_velocity_x: float = 0.0
    """Angular velocity around X axis in rad/s."""
    angular_velocity_y: float = 0.0
    """Angular velocity around Y axis in rad/s."""
    angular_velocity_z: float = 0.0
    """Angular velocity around Z axis in rad/s."""
    orientation: Quaternion = field(default_factory=lambda: Quaternion(1.0, 0.0, 0.0, 0.0))
    """Sensor orientation as a quaternion."""
    time: float = 0.0
    """Simulation time when the reading was taken."""
    is_valid: bool = False
    """Whether this reading contains valid data."""


@dataclass
class Quaternion:
    """Quaternion class for storing orientation.

    Internally stores components in [w, x, y, z] order but provides iteration
    and array conversion in [x, y, z, w] format for compatibility with existing code.
    """

    w: float
    """Scalar (real) component."""
    x: float
    """First imaginary component."""
    y: float
    """Second imaginary component."""
    z: float
    """Third imaginary component."""

    def __len__(self) -> int:
        """Return the number of quaternion components.

        Returns:
            Always returns 4.
        """
        return 4

    def __iter__(self) -> Iterator[float]:
        """Iterate over components in [x, y, z, w] order.

        Yields:
            Each quaternion component in [x, y, z, w] order.
        """
        yield self.x
        yield self.y
        yield self.z
        yield self.w

    def __array__(self, dtype: type | None = None) -> np.ndarray:
        """Convert to numpy array in [x, y, z, w] order.

        Args:
            dtype: Optional numpy dtype for the output array.

        Returns:
            Numpy array with components in [x, y, z, w] order.
        """
        return np.array([self.x, self.y, self.z, self.w], dtype=dtype)

    def __getitem__(self, index: int) -> float:
        """Access components by index in [x, y, z, w] order.

        Args:
            index: Component index (0=x, 1=y, 2=z, 3=w).

        Returns:
            The component value at the given index.

        Raises:
            IndexError: If index is out of range.
        """
        if index == 0:
            return self.x
        if index == 1:
            return self.y
        if index == 2:
            return self.z
        if index == 3:
            return self.w
        raise IndexError(index)


@dataclass
class IMURawData:
    """Raw IMU sensor data before processing.

    Contains velocity and orientation data captured at each physics step
    before filtering and acceleration computation.
    """

    time: float = 0.0
    """Simulation time of this sample."""
    dt: float = 0.0
    """Physics timestep duration."""
    linear_velocity_x: float = 0.0
    """Linear velocity along X axis in m/s."""
    linear_velocity_y: float = 0.0
    """Linear velocity along Y axis in m/s."""
    linear_velocity_z: float = 0.0
    """Linear velocity along Z axis in m/s."""
    angular_velocity_x: float = 0.0
    """Angular velocity around X axis in rad/s."""
    angular_velocity_y: float = 0.0
    """Angular velocity around Y axis in rad/s."""
    angular_velocity_z: float = 0.0
    """Angular velocity around Z axis in rad/s."""
    orientation_w: float = 1.0
    """Quaternion w component."""
    orientation_x: float = 0.0
    """Quaternion x component."""
    orientation_y: float = 0.0
    """Quaternion y component."""
    orientation_z: float = 0.0
    """Quaternion z component."""


def _to_numpy(value: Any) -> np.ndarray:
    """Convert a value to a numpy array.

    Args:
        value: Value to convert (array-like or numpy-compatible).

    Returns:
        Numpy array representation of the value.
    """
    if hasattr(value, "numpy"):
        return value.numpy()
    return np.array(value)


def _sanitize_vector(value: np.ndarray) -> np.ndarray:
    """Replace NaN and infinite values with zeros.

    Args:
        value: Input vector.

    Returns:
        Vector with non-finite values replaced by zeros.
    """
    return np.nan_to_num(value, nan=0.0, posinf=0.0, neginf=0.0)


def _normalize_quaternion(value: np.ndarray) -> np.ndarray:
    """Normalize a quaternion to unit length.

    Args:
        value: Quaternion vector.

    Returns:
        Normalized quaternion.
    """
    value = _sanitize_vector(np.array(value, dtype=np.float64))
    norm = np.linalg.norm(value)
    if not np.isfinite(norm) or norm == 0.0:
        return np.array([1.0, 0.0, 0.0, 0.0], dtype=np.float64)
    return value / norm


def _quat_multiply(first: np.ndarray, second: np.ndarray) -> np.ndarray:
    """Multiply two quaternions using Hamilton product.

    Args:
        first: First quaternion in [w, x, y, z] order.
        second: Second quaternion in [w, x, y, z] order.

    Returns:
        Product quaternion in [w, x, y, z] order.
    """
    return _to_numpy(transform_utils.quaternion_multiplication(first, second))


def _clone_imu_reading(reading: IMUSensorReading) -> IMUSensorReading:
    """Create a deep copy of an IMU reading.

    Args:
        reading: IMU reading to clone.

    Returns:
        New IMU reading with copied values.
    """
    return IMUSensorReading(
        linear_acceleration_x=reading.linear_acceleration_x,
        linear_acceleration_y=reading.linear_acceleration_y,
        linear_acceleration_z=reading.linear_acceleration_z,
        angular_velocity_x=reading.angular_velocity_x,
        angular_velocity_y=reading.angular_velocity_y,
        angular_velocity_z=reading.angular_velocity_z,
        orientation=Quaternion(
            reading.orientation.w,
            reading.orientation.x,
            reading.orientation.y,
            reading.orientation.z,
        ),
        time=reading.time,
        is_valid=reading.is_valid,
    )


class _SensorStepManager:
    """Singleton manager for physics sensor lifecycle events."""

    _instance: _SensorStepManager | None = None

    def __init__(self) -> None:
        # WeakSet of registered sensors. Items are ``_PhysicsSensorRuntimeBase``
        # instances (defined in ``_sensor_base.py``); typed loosely here to
        # avoid a circular import.
        self._sensors: weakref.WeakSet[Any] = weakref.WeakSet()

        self._post_step_callback = SimulationManager.register_callback(
            self._on_physics_step, event=SimulationEvent.PHYSICS_POST_STEP
        )
        self._stop_callback = SimulationManager.register_callback(
            self._on_timeline_stop, event=SimulationEvent.SIMULATION_STOPPED
        )
        self._start_callback = SimulationManager.register_callback(
            self._on_simulation_start, event=SimulationEvent.SIMULATION_STARTED
        )

    @classmethod
    def instance(cls) -> _SensorStepManager:
        """Get the singleton instance, creating it if necessary.

        Returns:
            Singleton instance of the manager.
        """
        if cls._instance is None:
            cls._instance = cls()
        return cls._instance

    def register(self, sensor: Any) -> None:
        """Register a sensor to receive physics step updates.

        Args:
            sensor: A ``_PhysicsSensorRuntimeBase`` instance.
        """
        self._sensors.add(sensor)

    def _on_simulation_start(self, event: Any) -> None:
        """Handle simulation start events.

        Args:
            event: Simulation start event data.
        """

    def _on_physics_step(self, step_dt: float, context: Any = None) -> None:
        """Handle physics step events.

        Notifies the contact report manager and all registered sensors of the physics step update.

        Args:
            step_dt: Physics step duration in seconds.
            context: Physics step context data.
        """
        for sensor in list(self._sensors):
            sensor.on_physics_step(step_dt)

    def _on_timeline_stop(self, event: Any) -> None:
        """Handle timeline stop events.

        Notifies all registered sensors of the timeline stop and clears auto-discovered IMU backends and contact data.

        Args:
            event: Timeline stop event data.
        """
        for sensor in list(self._sensors):
            sensor.on_timeline_stop()


def _create_sensor_prim(
    path: str,
    parent: str,
    schema_type: type,
    *,
    positions: list | np.ndarray | wp.array | None = None,
    translations: list | np.ndarray | wp.array | None = None,
    orientations: list | np.ndarray | wp.array | None = None,
) -> tuple[Any, str]:
    """Create a sensor prim with common setup (path, schema, enabled attr, transform).

    Args:
        path: Sensor name path (e.g. ``"/SensorName"``).
        parent: Parent prim path.
        schema_type: USD schema class to ``Define`` (e.g. ``IsaacSensorSchema.IsaacImuSensor``).
        positions: World-frame positions (shape ``(N, 3)``). Mutually exclusive with ``translations``.
        translations: Local-frame translations (shape ``(N, 3)``).
        orientations: Orientations as quaternion ``wxyz`` (shape ``(N, 4)``).

    Returns:
        Tuple of (schema prim object, resolved prim path string).
    """
    import omni.isaac.IsaacSensorSchema as IsaacSensorSchema
    import omni.usd
    from isaacsim.core.experimental.prims import XformPrim
    from isaacsim.core.experimental.utils import stage as stage_utils

    if positions is not None and translations is not None:
        raise ValueError("'positions' and 'translations' can't be both specified")

    stage = omni.usd.get_context().get_stage()
    base_path = f"{parent.rstrip('/')}/{path.lstrip('/')}"
    prim_path = stage_utils.generate_next_free_path(base_path, prepend_default_prim=False)
    prim = schema_type.Define(stage, prim_path)
    IsaacSensorSchema.IsaacBaseSensor(prim).CreateEnabledAttr(True)

    XformPrim(
        prim_path,
        reset_xform_op_properties=True,
        positions=positions,
        translations=translations,
        orientations=orientations,
    )
    return prim, prim_path
