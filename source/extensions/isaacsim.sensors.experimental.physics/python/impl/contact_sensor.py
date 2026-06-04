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

"""Contact sensor runtime providing frame-based data access via the C++ interface."""

from __future__ import annotations

from typing import cast

import numpy as np
from isaacsim.core.experimental.utils import prim as prim_utils
from isaacsim.core.experimental.utils import stage as stage_utils
from isaacsim.core.simulation_manager import SimulationManager
from pxr import PhysicsSchemaTools

from ._sensor_base import _PhysicsSensorRuntime
from .common import ContactSensorReading
from .contact import Contact


class ContactSensor(_PhysicsSensorRuntime):
    """Runtime wrapper for an Isaac contact sensor with frame-based data access.

    Wraps a :class:`Contact` authoring object and owns the C++ ``IContactSensor``
    Carbonite interface. Exposes :meth:`get_data` for a structured per-step
    dictionary, :meth:`get_sensor_reading` for the cached
    :class:`ContactSensorReading`, and :meth:`get_raw_data` for raw contact
    records.

    Args:
        path: Either a string USD path to an existing IsaacContactSensor prim,
            or a pre-built :class:`Contact` authoring object. To create a new
            prim, use :meth:`Contact.create`.

    Example:

    .. code-block:: python

        from isaacsim.sensors.experimental.physics import Contact, ContactSensor

        sensor = ContactSensor(
            Contact.create(
                "/World/Robot/foot/contact_sensor",
                min_threshold=1.0,
                max_threshold=1000.0,
                radius=0.05,
            )
        )

        frame = sensor.get_data()
        if frame["in_contact"]:
            print(f"Contact force: {frame['force']}")
    """

    _AUTHORING_CLASS = Contact
    _AUTHORING_ATTR = "_contact"

    def __init__(self, path: "str | Contact") -> None:
        # Initialize per-step caching state BEFORE calling super().__init__,
        # which registers self with _SensorStepManager. Without this ordering,
        # an on_physics_step firing between registration and these assignments
        # would AttributeError on self._is_valid_sensor / self._latest_reading.
        self._is_valid_sensor = False
        self._latest_reading = ContactSensorReading()
        self._last_physics_step = -1
        super().__init__(path)
        self._is_valid_sensor = self._check_sensor_prim_type()

    @property
    def contact(self) -> Contact:
        """Authoring object encapsulated by this sensor.

        Returns:
            The :class:`Contact` instance wrapping the underlying USD prim.
        """
        return self._contact

    def _acquire_interface(self) -> object | None:
        from .extension import get_contact_sensor_interface

        return get_contact_sensor_interface()

    def _get_invalid_reading(self) -> object:
        return ContactSensorReading()

    def _check_sensor_prim_type(self) -> bool:
        """Verify that the prim is an IsaacContactSensor type.

        Returns:
            True if the prim exists and is an IsaacContactSensor, False otherwise.
        """
        stage = stage_utils.get_current_stage(backend="usd")
        prim = stage.GetPrimAtPath(self._prim_path)
        if not prim.IsValid():
            return False
        return prim.GetTypeName() == "IsaacContactSensor"

    def _init_frame(self) -> dict[str, object]:
        return {
            "time": 0.0,
            "physics_step": 0.0,
        }

    def on_physics_step(self, step_dt: float) -> None:
        """Called by ``_SensorStepManager`` after each physics step.

        Reads the latest C++ sensor reading and caches it locally.

        Args:
            step_dt: Duration of the physics step in seconds.
        """
        self._last_physics_step = SimulationManager.get_num_physics_steps()

        if not self._is_valid_sensor:
            return

        if not self._ensure_sensor():
            return

        cpp_reading = self._iface.get_sensor_reading(self._prim_path)
        if not cpp_reading.is_valid and self._sensor_created:
            self._sensor_created = False
            if self._ensure_sensor():
                cpp_reading = self._iface.get_sensor_reading(self._prim_path)

        # Convert the C++ binding struct to the public Python dataclass so
        # callers always see a consistent return type (the binding type is an
        # implementation detail that shouldn't leak through get_sensor_reading).
        py_reading = ContactSensorReading(
            value=float(cpp_reading.value),
            time=float(cpp_reading.time),
            is_valid=bool(cpp_reading.is_valid),
        )
        py_reading.in_contact = bool(cpp_reading.in_contact)
        self._latest_reading = py_reading

    def on_timeline_stop(self) -> None:
        """Reset sensor state when the timeline stops."""
        super().on_timeline_stop()
        self._latest_reading = ContactSensorReading()
        self._last_physics_step = -1

    def get_sensor_reading(self) -> ContactSensorReading:
        """Get the latest cached contact sensor reading.

        Returns:
            Reading with contact state and force value. Returns an invalid
            reading if the sensor is disabled or the prim is invalid.
        """
        if not prim_utils.get_prim_at_path(self._prim_path).IsValid():
            # Tear down stale C++ state so a recreate at the same path rebuilds
            # cleanly instead of reusing the cached map entry.
            if self._sensor_created:
                self.reset()
            self._latest_reading = ContactSensorReading()
            self._last_physics_step = -1
            self._is_valid_sensor = False
            return ContactSensorReading(is_valid=False, time=0.0)

        if not self._is_valid_sensor:
            # Re-check in case a sensor prim was just (re-)authored at this path.
            self._is_valid_sensor = self._check_sensor_prim_type()
        if not self._is_valid_sensor:
            return ContactSensorReading(is_valid=False, time=0.0)

        if not self._ensure_sensor():
            return ContactSensorReading(is_valid=False, time=0.0)

        if SimulationManager.is_simulating():
            current_step = SimulationManager.get_num_physics_steps()
            if current_step != self._last_physics_step:
                self.on_physics_step(SimulationManager.get_physics_dt())

        return self._latest_reading

    def get_raw_data(self) -> list[dict[str, object]]:
        """Get raw contact data for the sensor's parent body.

        Returns:
            Raw contact dictionaries containing body0, body1, position, normal,
            impulse, time, and dt fields. Empty when the sensor is invalid.
        """
        if not self._is_valid_sensor:
            return []

        if not self._ensure_sensor():
            return []

        return self._iface.get_raw_contacts(self._prim_path)

    def get_data(self) -> dict:
        """Get the current contact sensor data as a structured frame.

        Returns:
            Frame data containing:
                - ``"in_contact"``: Whether contact is detected.
                - ``"force"``: Contact force magnitude.
                - ``"time"``: Simulation time of reading.
                - ``"physics_step"``: Physics step number.
                - ``"number_of_contacts"``: Number of contact points.
                - ``"contacts"``: Raw contact data if enabled via ``add_raw_contact_data_to_frame``.
        """
        contact_sensor_reading = self.get_sensor_reading()

        if contact_sensor_reading.is_valid:
            self._current_frame["in_contact"] = bool(contact_sensor_reading.in_contact)
            self._current_frame["force"] = float(contact_sensor_reading.value)
            self._current_frame["time"] = float(contact_sensor_reading.time)
            self._current_frame["physics_step"] = float(SimulationManager.get_num_physics_steps())

            contact_raw_data = self.get_raw_data()
            self._current_frame["number_of_contacts"] = len(contact_raw_data)

            if isinstance(self._current_frame.get("contacts"), list):
                contacts: list[dict[str, object]] = []
                for i in range(len(contact_raw_data)):
                    contact_frame: dict[str, object] = {}
                    body0 = cast(int, contact_raw_data[i]["body0"])
                    body1 = cast(int, contact_raw_data[i]["body1"])
                    contact_frame["body0"] = str(PhysicsSchemaTools.intToSdfPath(int(body0)))
                    contact_frame["body1"] = str(PhysicsSchemaTools.intToSdfPath(int(body1)))
                    position_dict = cast(dict[str, float], contact_raw_data[i]["position"])
                    contact_frame["position"] = np.array(
                        [position_dict["x"], position_dict["y"], position_dict["z"]],
                        dtype=np.float32,
                    )
                    normal_dict = cast(dict[str, float], contact_raw_data[i]["normal"])
                    contact_frame["normal"] = np.array(
                        [normal_dict["x"], normal_dict["y"], normal_dict["z"]],
                        dtype=np.float32,
                    )
                    impulse_dict = cast(dict[str, float], contact_raw_data[i]["impulse"])
                    contact_frame["impulse"] = np.array(
                        [impulse_dict["x"], impulse_dict["y"], impulse_dict["z"]],
                        dtype=np.float32,
                    )
                    contacts.append(contact_frame)
                self._current_frame["contacts"] = contacts

        return self._current_frame

    def add_raw_contact_data_to_frame(self) -> None:
        """Enable raw contact data in frame output.

        After calling this, :meth:`get_data` will include a ``"contacts"`` list
        with detailed per-contact information.
        """
        contacts: list[dict[str, object]] = []
        self._current_frame["contacts"] = contacts

    def remove_raw_contact_data_from_frame(self) -> None:
        """Disable raw contact data in frame output.

        Removes the ``"contacts"`` key from frame output to reduce overhead.
        """
        del self._current_frame["contacts"]
