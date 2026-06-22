# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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

"""Joint state sensor for reading all DOF positions, velocities, and efforts.

Reads full joint state from an articulated robot in a single call via the
C++ ``IJointStateSensor`` Carbonite interface. The sensor inherits the C++
interface lifecycle from :class:`_PhysicsSensorRuntimeBase`.
"""

from __future__ import annotations

from typing import Any

import carb
import carb.eventdispatcher
import numpy as np
import omni.usd
from isaacsim.core.simulation_manager import SimulationManager

from ._sensor_base import _PhysicsSensorRuntimeBase

_INVALID_JOINT_STATE_READING = None


def _get_invalid_reading() -> object:
    global _INVALID_JOINT_STATE_READING
    if _INVALID_JOINT_STATE_READING is None:
        from .. import _physics_sensors

        _INVALID_JOINT_STATE_READING = _physics_sensors.JointStateSensorReading()
    return _INVALID_JOINT_STATE_READING


class JointStateSensorReading:
    """Joint state sensor reading data for all DOFs of an articulation.

    Args:
        is_valid: Whether this reading contains valid data.
        time: Simulation time when the reading was taken.
        dof_names: List of DOF name strings in articulation order.
        positions: DOF positions array (rad for revolute, m for prismatic).
        velocities: DOF velocities array (rad/s or m/s).
        efforts: DOF efforts array (Nm or N).
        dof_types: Per-DOF type: 0 = rotation (revolute), 1 = translation (prismatic).
        stage_meters_per_unit: Stage meters per USD unit for SI conversion.
    """

    def __init__(
        self,
        is_valid: bool = False,
        time: float = 0.0,
        dof_names: list[str] | None = None,
        positions: np.ndarray | None = None,
        velocities: np.ndarray | None = None,
        efforts: np.ndarray | None = None,
        dof_types: np.ndarray | None = None,
        stage_meters_per_unit: float = 0.0,
    ) -> None:
        self.is_valid = is_valid
        self.time = time
        self.dof_names: list[str] = dof_names if dof_names is not None else []
        self.positions: np.ndarray = positions if positions is not None else np.array([], dtype=np.float32)
        self.velocities: np.ndarray = velocities if velocities is not None else np.array([], dtype=np.float32)
        self.efforts: np.ndarray = efforts if efforts is not None else np.array([], dtype=np.float32)
        self.dof_types: np.ndarray = dof_types if dof_types is not None else np.array([], dtype=np.uint8)
        self.stage_meters_per_unit: float = stage_meters_per_unit


class JointStateSensor(_PhysicsSensorRuntimeBase):
    """Sensor for reading all DOF joint states from an articulation.

    Reads positions, velocities, and efforts for every DOF in the articulation
    via the C++ ``IJointStateSensor`` Carbonite interface. Analogous to a ROS2
    JointState message.

    Args:
        path: USD path to the articulation root prim.
        enabled: Whether the sensor is initially enabled.

    Example:

        .. code-block:: python

            from isaacsim.sensors.experimental.physics import JointStateSensor

            sensor = JointStateSensor("/World/Robot")

            # After playing the simulation:
            reading = sensor.get_sensor_reading()
            if reading.is_valid:
                for name, pos in zip(reading.dof_names, reading.positions):
                    print(f"{name}: {pos:.4f} rad")
    """

    def __init__(self, path: str, enabled: bool = True) -> None:
        super().__init__(path)
        self.enabled = enabled

        self._stage_open_sub = carb.eventdispatcher.get_eventdispatcher().observe_event(
            event_name=omni.usd.get_context().stage_event_name(omni.usd.StageEventType.OPENED),
            on_event=self._stage_open_callback_fn,
            observer_name="isaacsim.sensors.experimental.physics.JointStateSensor._stage_open_callback",
        )

    def _acquire_interface(self) -> object | None:
        from .extension import get_joint_state_sensor_interface

        return get_joint_state_sensor_interface()

    def _get_invalid_reading(self) -> object:
        return _get_invalid_reading()

    def _stage_open_callback_fn(self, event: Any = None) -> None:
        """Handle stage open by releasing subscriptions.

        Args:
            event: Stage-open event payload.
        """
        self._stage_open_sub = None

    def get_sensor_reading(self) -> JointStateSensorReading:
        """Get the current joint state reading for all DOFs.

        Returns:
            Reading with DOF names, positions, velocities, efforts, and validity.
        """
        if not self.enabled:
            return JointStateSensorReading()

        cpp_reading = self._get_reading()
        if not cpp_reading.is_valid:
            return JointStateSensorReading()

        # Binding returns numpy arrays (one copy from C); avoid second copy with asarray
        return JointStateSensorReading(
            is_valid=True,
            time=cpp_reading.time,
            dof_names=cpp_reading.dof_names,
            positions=np.asarray(cpp_reading.positions, dtype=np.float32),
            velocities=np.asarray(cpp_reading.velocities, dtype=np.float32),
            efforts=np.asarray(cpp_reading.efforts, dtype=np.float32),
            dof_types=np.asarray(cpp_reading.dof_types, dtype=np.uint8),
            stage_meters_per_unit=cpp_reading.stage_meters_per_unit,
        )

    def get_data(self) -> dict:
        """Get the current joint state as a structured frame.

        Returns:
            Frame data containing:
                - ``"dof_names"``: List of DOF names in articulation order.
                - ``"positions"``: Per-DOF positions (rad or m).
                - ``"velocities"``: Per-DOF velocities (rad/s or m/s).
                - ``"efforts"``: Per-DOF efforts (Nm or N).
                - ``"dof_types"``: Per-DOF type (0 = revolute, 1 = prismatic).
                - ``"stage_meters_per_unit"``: Stage meters per USD unit.
                - ``"is_valid"``: Whether the reading contains valid data.
                - ``"time"``: Simulation time of the reading.
                - ``"physics_step"``: Physics step number.
        """
        reading = self.get_sensor_reading()
        return {
            "dof_names": list(reading.dof_names),
            "positions": reading.positions,
            "velocities": reading.velocities,
            "efforts": reading.efforts,
            "dof_types": reading.dof_types,
            "stage_meters_per_unit": float(reading.stage_meters_per_unit),
            "is_valid": bool(reading.is_valid),
            "time": float(reading.time),
            "physics_step": int(SimulationManager.get_num_physics_steps()),
        }
