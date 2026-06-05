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

"""Raycast sensor runtime providing frame-based data access via the C++ interface."""

from __future__ import annotations

import numpy as np
from isaacsim.core.simulation_manager import SimulationManager

from ._sensor_base import _PhysicsSensorRuntime
from .raycast import Raycast

_INVALID_RAYCAST_READING = None


def _get_invalid_raycast_reading() -> object:
    global _INVALID_RAYCAST_READING
    if _INVALID_RAYCAST_READING is None:
        from .. import _physics_sensors

        _INVALID_RAYCAST_READING = _physics_sensors.RaycastSensorReading()
    return _INVALID_RAYCAST_READING


class RaycastSensor(_PhysicsSensorRuntime):
    """Runtime wrapper for an Isaac raycast sensor with frame-based data access.

    Wraps a :class:`Raycast` authoring object and owns the C++ ``IRaycastSensor``
    Carbonite interface. Exposes :meth:`get_data` for a structured per-step
    dictionary and :meth:`get_sensor_reading` for the raw C++ struct.

    Args:
        path: Either a string USD path to an existing IsaacRaycastSensor prim,
            or a pre-built :class:`Raycast` authoring object. To create a new
            prim, use :meth:`Raycast.create`.

    Example:

    .. code-block:: python

        from isaacsim.sensors.experimental.physics import Raycast, RaycastSensor

        sensor = RaycastSensor(
            Raycast.create(
                "/World/Robot/body/raycast",
                ray_origins=[[0, 0, 0]],
                ray_directions=[[1, 0, 0]],
            )
        )

        frame = sensor.get_data()
        print(f"Depths: {frame['depths']}")
    """

    _AUTHORING_CLASS = Raycast
    _AUTHORING_ATTR = "_raycast"

    @property
    def raycast(self) -> Raycast:
        """Authoring object encapsulated by this sensor.

        Returns:
            The :class:`Raycast` instance wrapping the underlying USD prim.
        """
        return self._raycast

    def _acquire_interface(self) -> object | None:
        from .extension import get_raycast_sensor_interface

        return get_raycast_sensor_interface()

    def _get_invalid_reading(self) -> object:
        return _get_invalid_raycast_reading()

    def get_sensor_reading(self) -> object:
        """Get the current raycast sensor reading as the raw C++ struct.

        Returns:
            The C++ ``RaycastSensorReading`` struct. Access fields via
            ``reading.depths``, ``reading.hit_positions``, etc.
        """
        return self._get_reading()

    def get_data(self) -> dict:
        """Get the current raycast sensor data as a structured frame.

        Returns:
            Frame data containing:
                - ``"depths"``: Per-ray depth values.
                - ``"hit_positions"``: Per-ray hit positions as Nx3 array.
                - ``"hit_normals"``: Per-ray surface normals as Nx3 array.
                - ``"hit_prim_paths"``: Per-ray hit prim USD paths.
                - ``"time"``: Simulation time of reading.
                - ``"physics_step"``: Physics step number.
        """
        reading = self.get_sensor_reading()

        if reading.is_valid:
            return {
                "depths": reading.depths,
                "hit_positions": reading.hit_positions,
                "hit_normals": reading.hit_normals,
                "hit_prim_paths": reading.hit_prim_paths,
                "time": reading.time,
                "physics_step": int(SimulationManager.get_num_physics_steps()),
            }

        return {
            "depths": np.array([], dtype=np.float32),
            "hit_positions": np.zeros((0, 3), dtype=np.float32),
            "hit_normals": np.zeros((0, 3), dtype=np.float32),
            "hit_prim_paths": [],
            "time": 0.0,
            "physics_step": int(SimulationManager.get_num_physics_steps()),
        }
