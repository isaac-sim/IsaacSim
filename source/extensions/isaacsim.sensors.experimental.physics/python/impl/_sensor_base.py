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

"""Base classes for physics sensor authoring (USD prim) and runtime (data acquisition).

Mirrors the pattern in ``isaacsim.sensors.experimental.rtx``:
- :class:`_PhysicsSensorAuthoring` is an ``XformPrim`` subclass that owns USD prim
  creation, schema wrapping, transform application, and sensor-specific attribute
  setup. Concrete authoring classes (e.g. :class:`IMU`) define ``_PRIM_TYPE``,
  ``_SCHEMA_CLASS``, and override ``_create_prim``.
- :class:`_PhysicsSensorRuntimeBase` owns the C++ Carbonite interface lifecycle
  (lazy acquisition, ``create_sensor``/``remove_sensor``, retry-on-invalid,
  ``_SensorStepManager`` registration, timeline-stop reset). Concrete sensors
  override ``_acquire_interface`` and ``_get_invalid_reading``.
- :class:`_PhysicsSensorRuntime` extends :class:`_PhysicsSensorRuntimeBase` for
  sensors that pair with an authoring class. Concrete classes (e.g.
  :class:`IMUSensor`) define ``_AUTHORING_CLASS`` and ``_AUTHORING_ATTR``.
  Sensors with no authoring class (e.g. :class:`EffortSensor`) inherit
  :class:`_PhysicsSensorRuntimeBase` directly.
"""

from __future__ import annotations

from typing import Any

import carb
import numpy as np
import warp as wp
from isaacsim.core.experimental.prims import XformPrim
from isaacsim.core.experimental.utils import prim as prim_utils

from .common import _SensorStepManager


class _PhysicsSensorAuthoring(XformPrim):
    """Base class for physics sensor authoring (USD prim creation/wrapping).

    Handles:
        - ``positions`` / ``translations`` mutual-exclusivity check
        - ``XformPrim.resolve_paths`` + N=1 guard
        - Path parsing (``_body_prim_path``, ``_sensor_name``)
        - Wrap-existing vs. create-new branching against ``_PRIM_TYPE``
        - Transform application via ``XformPrim.__init__``

    Subclasses must define:
        ``_PRIM_TYPE``: USD prim type name (e.g. ``"IsaacImuSensor"``).
        ``_SCHEMA_CLASS``: Schema wrapper class (e.g. ``IsaacSensorSchema.IsaacImuSensor``).

    Subclasses must override:
        ``_create_prim``: Create a new prim at ``self._body_prim_path/sensor_name``
        with default attributes applied. Returns the schema-wrapped prim.

    Subclasses may override:
        ``_on_existing_prim``: Validation hook before wrapping an existing prim.
        ``_update_attributes``: Apply user-provided attribute overrides after wrapping.
    """

    _PRIM_TYPE: str
    _SCHEMA_CLASS: type

    def __init__(
        self,
        path: str,
        *,
        positions: list | np.ndarray | wp.array | None = None,
        translations: list | np.ndarray | wp.array | None = None,
        orientations: list | np.ndarray | wp.array | None = None,
        scales: list | np.ndarray | wp.array | None = None,
        reset_xform_op_properties: bool = True,
        **kwargs: Any,
    ) -> None:
        if positions is not None and translations is not None:
            raise ValueError("'positions' and 'translations' can't be both specified")

        existent_paths, nonexistent_paths = XformPrim.resolve_paths(path)
        if len(existent_paths) > 1 or len(nonexistent_paths) > 1:
            raise ValueError(
                f"Only one {self._PRIM_TYPE} prim is supported, "
                f"but the provided argument refers to {len(existent_paths) + len(nonexistent_paths)} prims: "
                f"{existent_paths + nonexistent_paths}",
            )

        self._body_prim_path = "/".join(path.split("/")[:-1])
        self._sensor_name = path.split("/")[-1]

        prim = prim_utils.get_prim_at_path(path)
        if prim.IsValid():
            type_name = prim.GetPrimTypeInfo().GetTypeName()
            if type_name != self._PRIM_TYPE:
                raise ValueError(
                    f"Prim at {path} is not an '{self._PRIM_TYPE}' prim but a '{type_name}' prim",
                )
            self._isaac_sensor_prim = self._SCHEMA_CLASS(prim)
            self._on_existing_prim(prim, **kwargs)
            super().__init__(
                path,
                positions=positions,
                translations=translations,
                orientations=orientations,
                scales=scales,
                reset_xform_op_properties=reset_xform_op_properties,
            )
            self._prim = self.prims[0]
            self._update_attributes(**kwargs)
        else:
            carb.log_info(f"Creating a new {self._PRIM_TYPE} prim at path {path}")
            self._isaac_sensor_prim = self._create_prim(**kwargs)
            super().__init__(
                path,
                positions=positions,
                translations=translations,
                orientations=orientations,
                scales=scales,
                reset_xform_op_properties=reset_xform_op_properties,
            )
            self._prim = self.prims[0]

    def _create_prim(self, **kwargs: Any) -> Any:
        """Create a new prim at ``self._body_prim_path/self._sensor_name``.

        Subclasses must override to call ``_create_sensor_prim`` (from ``common.py``)
        and apply default sensor-specific attributes. Returns the schema-wrapped prim.
        """
        raise NotImplementedError

    def _on_existing_prim(self, prim: Any, **kwargs: Any) -> None:
        """Hook invoked before ``super().__init__`` when wrapping an existing prim.

        Default no-op. Subclasses may override for validation that needs to run
        before transforms are applied (e.g., parent-API checks).
        """

    def _update_attributes(self, **kwargs: Any) -> None:
        """Hook invoked after wrapping an existing prim to apply user overrides.

        Default no-op. Subclasses may override to conditionally apply attribute
        updates passed by the user at construction time.
        """

    @classmethod
    def create(cls, path: str, **kwargs: Any) -> "_PhysicsSensorAuthoring":
        """Create a new sensor at the specified path.

        Always creates a fresh prim, auto-numbering the path if it already
        exists (e.g. ``/World/Cube/Sensor`` → ``/World/Cube/Sensor_01``).

        Args:
            path: Full USD path for the sensor (e.g. ``/World/Robot/body/sensor``).
                Trailing slashes are stripped.
            **kwargs: Sensor-specific keyword arguments forwarded to ``__init__``.

        Returns:
            New authoring instance wrapping the created prim.

        Raises:
            RuntimeError: If the path does not include a parent prim.
        """
        from isaacsim.core.experimental.utils import stage as stage_utils

        path = path.rstrip("/")
        parent = "/".join(path.split("/")[:-1])
        sensor_name = path.split("/")[-1]
        if not parent:
            raise RuntimeError(f"Path must include a parent prim (e.g., '/World/Cube/{sensor_name}')")
        unique_path = stage_utils.generate_next_free_path(path, prepend_default_prim=False)
        return cls(unique_path, **kwargs)


class _PhysicsSensorRuntimeBase:
    """Lifecycle mixin for physics sensors backed by a C++ Carbonite interface.

    Provides shared lifecycle logic: lazy interface acquisition, sensor
    creation/removal, retry-on-invalid reading, ``_SensorStepManager``
    registration, and timeline-stop reset.

    Subclasses must override ``_acquire_interface`` and ``_get_invalid_reading``.
    """

    def __init__(self, path: str) -> None:
        self._prim_path = path
        self._sensor_created: bool = False
        self._iface = None
        _SensorStepManager.instance().register(self)

    def _acquire_interface(self) -> object | None:
        """Return the C++ Carbonite interface for this sensor type.

        Raises:
            NotImplementedError: If not overridden by subclass.
        """
        raise NotImplementedError

    def _get_invalid_reading(self) -> object:
        """Return a default invalid reading for this sensor type.

        Raises:
            NotImplementedError: If not overridden by subclass.
        """
        raise NotImplementedError

    def _ensure_sensor(self) -> bool:
        """Ensure the C++ sensor is created and initialized.

        Returns:
            True if the sensor is ready, False otherwise.
        """
        if self._iface is None:
            self._iface = self._acquire_interface()
        if self._iface is None:
            return False
        if self._sensor_created:
            return True
        self._sensor_created = self._iface.create_sensor(self._prim_path)
        return self._sensor_created

    def _get_reading(self, *args: object) -> object:
        """Get a sensor reading with auto-retry on invalid.

        Args:
            *args: Additional arguments forwarded to C++ ``get_sensor_reading``.

        Returns:
            The sensor reading object.
        """
        if not self._sensor_created and not self._ensure_sensor():
            return self._get_invalid_reading()
        reading = self._iface.get_sensor_reading(self._prim_path, *args)
        if not reading.is_valid:
            self._sensor_created = False
            if not self._ensure_sensor():
                return self._get_invalid_reading()
            reading = self._iface.get_sensor_reading(self._prim_path, *args)
        return reading

    def on_physics_step(self, step_dt: float) -> None:
        """Called after each physics step. Override for custom per-step logic.

        Args:
            step_dt: Physics step duration in seconds.
        """

    def on_timeline_stop(self) -> None:
        """Reset sensor state when the timeline stops."""
        self._sensor_created = False
        self._iface = None

    def reset(self) -> None:
        """Remove the sensor from the simulation and reset state."""
        if self._iface is not None and self._sensor_created:
            self._iface.remove_sensor(self._prim_path)
        self._sensor_created = False

    def _rebind(self, path: str) -> None:
        """Re-target this sensor at a new USD path.

        Removes the existing C++ sensor (if any) and resets cached state so
        the next reading lazily re-creates the sensor at ``path``.

        Args:
            path: New USD path for the sensor.
        """
        self.reset()
        self._prim_path = path
        self._iface = None


class _PhysicsSensorRuntime(_PhysicsSensorRuntimeBase):
    """Base class for physics sensors that pair with an authoring class.

    Wraps a :class:`_PhysicsSensorAuthoring` object and inherits the C++
    interface lifecycle from :class:`_PhysicsSensorRuntimeBase`. The
    constructor accepts either a path string (which is forwarded to the
    authoring class — wrapping an existing prim or creating a new one with
    default attributes) or a pre-built authoring object. To create a new prim
    with explicit attributes, use the authoring class's ``create()`` method:

        ``IMUSensor(IMU.create("/World/Cube/Imu", translations=...))``

    Subclasses must define:
        ``_AUTHORING_CLASS``: The authoring class (e.g. ``IMU``).
        ``_AUTHORING_ATTR``: Attribute name for the encapsulated authoring object
        (e.g. ``"_imu"``). Used by typed properties on subclasses.

    Subclasses must override (from :class:`_PhysicsSensorRuntimeBase`):
        ``_acquire_interface`` and ``_get_invalid_reading``.

    Subclasses may override:
        ``_init_frame``: Returns the initial ``_current_frame`` dict. Default
        provides ``time`` and ``physics_step`` keys.
        ``get_data``: Public read method for sensor data.
    """

    _AUTHORING_CLASS: type
    _AUTHORING_ATTR: str

    def __init__(
        self,
        path: "str | _PhysicsSensorAuthoring",
    ) -> None:
        if isinstance(path, self._AUTHORING_CLASS):
            authoring = path
        else:
            authoring = self._AUTHORING_CLASS(path)
        setattr(self, self._AUTHORING_ATTR, authoring)

        _PhysicsSensorRuntimeBase.__init__(self, authoring.paths[0])
        self._current_time = 0.0
        self._current_frame: dict[str, object] = self._init_frame()

    def _init_frame(self) -> dict[str, object]:
        """Build the initial ``_current_frame`` dict.

        Default provides ``time`` and ``physics_step`` keys. Subclasses override
        to add sensor-specific fields.
        """
        return {"time": 0.0, "physics_step": 0.0}

    @property
    def authoring_object(self) -> _PhysicsSensorAuthoring:
        """Authoring object encapsulated by this sensor.

        Returns:
            The authoring object (e.g. :class:`IMU`, :class:`Contact`,
            :class:`Raycast`) wrapping the underlying USD prim.
        """
        return getattr(self, self._AUTHORING_ATTR)
