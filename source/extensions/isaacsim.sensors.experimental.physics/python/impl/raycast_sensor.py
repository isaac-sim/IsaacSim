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


def _get_invalid_raycast_reading() -> object:
    from .. import _physics_sensors

    return _physics_sensors.RaycastSensorReading()


class RaycastSensor(_PhysicsSensorRuntime):
    """Runtime wrapper for an Isaac raycast sensor with frame-based data access.

    Wraps a :class:`Raycast` authoring object and owns the C++ ``IRaycastSensor``
    Carbonite interface. Exposes :meth:`get_data` for a structured per-step
    dictionary, :meth:`get_sensor_reading` for an owned snapshot, and
    :meth:`get_sensor_reading_view` for a read-only view of live buffers.

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
        """Get a mutable owned snapshot of the current raycast reading.

        The snapshot and its NumPy arrays remain valid after later physics
        steps, sensor reset, or removal. Its values and ``is_valid`` flag do
        not change after capture.

        Returns:
            An owned ``RaycastSensorReading`` snapshot. Access fields via
            ``reading.depths``, ``reading.hit_positions``, and related properties.
        """
        return self._get_reading()

    def get_sensor_reading_view(self) -> object:
        """Get a read-only zero-copy view of the live raycast buffers.

        Numeric arrays returned from the view retain its native backing frame
        even if the view variable is released. Their values and ``view.time``
        update on each physics step. ``view.is_valid`` becomes ``False`` after
        reset, removal, or timeline stop; retained arrays then remain readable
        with their last values. A numeric property lookup creates a NumPy array
        header without copying data; cache the returned array in a tight loop.

        The view performs no synchronization with the physics step that writes
        its buffers, so reading it while a step is in flight yields torn values
        that mix the current and the previous reading. Read it only from the
        thread that drives physics and only between steps; use
        :meth:`get_sensor_reading` when a reader cannot be ordered against the
        step.

        Returns:
            A ``RaycastSensorReadingView`` with read-only NumPy arrays.

        Example:

        .. code-block:: python

            view = sensor.get_sensor_reading_view()
            if view.is_valid:
                depths = view.depths  # read-only and owner-backed
        """
        from .. import _physics_sensors

        if not self._sensor_created and not self._ensure_sensor():
            return _physics_sensors.RaycastSensorReadingView()
        view = self._iface.get_sensor_reading_view(self._prim_path)
        if not view.is_valid:
            self._sensor_created = False
            if not self._ensure_sensor():
                return view
            view = self._iface.get_sensor_reading_view(self._prim_path)
        return view

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
