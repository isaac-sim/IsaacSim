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

"""Provide contact sensor functionality for detecting and measuring physical contact forces in simulations."""

from __future__ import annotations

import carb
import numpy as np
import omni.isaac.IsaacSensorSchema as IsaacSensorSchema
import omni.kit.commands
from isaacsim.core.api.sensors.base_sensor import BaseSensor
from isaacsim.core.simulation_manager import SimulationManager
from isaacsim.core.utils.prims import get_prim_at_path, is_prim_path_valid
from isaacsim.core.utils.stage import traverse_stage
from isaacsim.sensors.physics import _sensor
from pxr import Gf, PhysxSchema, UsdPhysics


class ContactSensor(BaseSensor):
    """A sensor that detects physical contact and measures contact forces.

    ContactSensor creates and manages a contact sensor that can detect when objects come into contact with
    it and measure the magnitude of contact forces. The sensor must be attached to a prim that has the
    UsdPhysics.CollisionAPI enabled. It provides real-time contact detection with configurable thresholds
    and can report detailed contact information including positions, normals, and impulses.

    The sensor can operate at a specified frequency or time step and includes filtering capabilities through
    min/max thresholds to ignore contacts below or above certain force values. It supports both basic contact
    detection (boolean in-contact state and force magnitude) and detailed contact data (individual contact
    points with full physics information).

    Args:
        prim_path: USD path where the contact sensor will be created.
        name: Name identifier for the sensor.
        frequency: Sensor update frequency in Hz. Cannot be used with dt.
        dt: Sensor update time step in seconds. Cannot be used with frequency.
        translation: Local translation offset of the sensor relative to its parent prim.
        position: World position where the sensor should be placed.
        min_threshold: Minimum force threshold below which contacts are ignored.
        max_threshold: Maximum force threshold above which contacts are clamped.
        radius: Detection radius of the contact sensor. Negative values indicate unlimited radius.

    Raises:
        Exception: If both frequency and dt are specified simultaneously.
        Exception: If the parent prim does not have UsdPhysics.CollisionAPI enabled.
        Exception: If sensor creation fails.
    """

    def __init__(
        self,
        prim_path: str,
        name: str | None = "contact_sensor",
        frequency: int | None = None,
        dt: float | None = None,
        translation: np.ndarray | None = None,
        position: np.ndarray | None = None,
        min_threshold: float | None = None,
        max_threshold: float | None = None,
        radius: float | None = None,
    ) -> None:
        if frequency is not None and dt is not None:
            raise Exception("Sensor Frequency and Sensor dt can't be both specified")

        if frequency is not None:
            dt = float(1 / frequency)

        self._body_prim_path = "/".join(prim_path.split("/")[:-1])
        if is_prim_path_valid(self._body_prim_path) and not get_prim_at_path(self._body_prim_path).HasAPI(
            UsdPhysics.CollisionAPI
        ):
            raise Exception("Contact Sensor needs to be created under another prim that has collision api enabled on.")
        self._sensor_name = prim_path.split("/")[-1]
        self._contact_sensor_interface = _sensor.acquire_contact_sensor_interface()
        if is_prim_path_valid(prim_path):
            self._isaac_sensor_prim = IsaacSensorSchema.IsaacContactSensor(get_prim_at_path(prim_path))
            BaseSensor.__init__(self, prim_path=prim_path, name=name, translation=translation, position=position)
            if dt is not None:
                self.set_dt(dt)
            if min_threshold is not None:
                self.set_min_threshold(min_threshold)
            if max_threshold is not None:
                self.set_max_threshold(max_threshold)
            if radius is not None:
                self.set_radius(radius)
        else:
            if min_threshold is None:
                min_threshold = 0
            if max_threshold is None:
                max_threshold = 100000
            if radius is None:
                radius = -1
            if dt is None:
                current_physics_prim = None
                for prim in traverse_stage():
                    if prim.HasAPI(PhysxSchema.PhysxSceneAPI):
                        current_physics_prim = prim
                        break
                if current_physics_prim is None:
                    raise ValueError("No PhysicsScene found on stage. Cannot determine default dt for ContactSensor.")
                physx_scene_api = PhysxSchema.PhysxSceneAPI(current_physics_prim)
                current_physics_frequency = physx_scene_api.GetTimeStepsPerSecondAttr().Get()
                dt = 1.0 / current_physics_frequency
            color_rgba = np.array([1.0, 1.0, 1.0, 1.0])
            carb.log_info(f"Creating a new contact sensor prim at path {prim_path}")
            success, self._isaac_sensor_prim = omni.kit.commands.execute(
                "IsaacSensorCreateContactSensor",
                path="/" + self._sensor_name,
                parent=self._body_prim_path,
                min_threshold=min_threshold,
                max_threshold=max_threshold,
                color=Gf.Vec4f(*color_rgba.tolist()),
                radius=radius,
                sensor_period=dt,
            )
            if not success:
                raise Exception("Not successful")
            BaseSensor.__init__(self, prim_path=prim_path, name=name, translation=translation, position=position)
        self._pause = False
        self._current_time = 0
        self._number_of_physics_steps = 0
        self._current_frame = {}
        self._current_frame["time"] = 0
        self._current_frame["physics_step"] = 0
        return

    def initialize(self, physics_sim_view: object = None) -> None:
        """Initialize the contact sensor.

        Args:
            physics_sim_view: Physics simulation view to initialize with.
        """
        BaseSensor.initialize(self, physics_sim_view=physics_sim_view)
        return

    def get_current_frame(self) -> dict:
        """Get the current sensor frame data including contact information.

        Returns:
            Dictionary containing contact data with keys for time, physics step, contact status, force, and number of contacts.
        """
        cs_sensor_reading = self._contact_sensor_interface.get_sensor_reading(self.prim_path)
        cs_raw_data = self._contact_sensor_interface.get_contact_sensor_raw_data(self.prim_path)
        if cs_sensor_reading.is_valid:
            self._current_frame["in_contact"] = bool(cs_sensor_reading.in_contact)
            self._current_frame["force"] = float(cs_sensor_reading.value)
            self._current_frame["time"] = float(cs_sensor_reading.time)
            # self._current_frame["physics_step"] = float(self._number_of_physics_steps)
            self._current_frame["number_of_contacts"] = len(cs_raw_data)
            if "contacts" in self._current_frame:
                self._current_frame["contacts"] = []
                for i in range(len(cs_raw_data)):
                    contact_frame = {}
                    contact_frame["body0"] = self._contact_sensor_interface.decode_body_name(
                        int(cs_raw_data["body0"][i])
                    )
                    contact_frame["body1"] = self._contact_sensor_interface.decode_body_name(
                        int(cs_raw_data["body1"][i])
                    )
                    contact_frame["position"] = self._backend_utils.create_tensor_from_list(
                        [
                            cs_raw_data["position"][i][0],
                            cs_raw_data["position"][i][1],
                            cs_raw_data["position"][i][2],
                        ],
                        dtype="float32",
                        device=self._device,
                    )
                    contact_frame["normal"] = self._backend_utils.create_tensor_from_list(
                        [cs_raw_data["normal"][i][0], cs_raw_data["normal"][i][1], cs_raw_data["normal"][i][2]],
                        dtype="float32",
                        device=self._device,
                    )
                    contact_frame["impulse"] = self._backend_utils.create_tensor_from_list(
                        [
                            cs_raw_data["impulse"][i][0],
                            cs_raw_data["impulse"][i][1],
                            cs_raw_data["impulse"][i][2],
                        ],
                        dtype="float32",
                        device=self._device,
                    )
                    self._current_frame["contacts"].append(contact_frame)
            self._current_frame["physics_step"] = float(SimulationManager.get_num_physics_steps())
        return self._current_frame

    def add_raw_contact_data_to_frame(self) -> None:
        """Add raw contact data to the current frame for detailed contact information."""
        self._current_frame["contacts"] = []
        return

    def remove_raw_contact_data_from_frame(self) -> None:
        """Remove raw contact data from the current frame."""
        del self._current_frame["contacts"]
        return

    def resume(self) -> None:
        """Resume sensor data collection by enabling the contact sensor."""
        self._isaac_sensor_prim.GetEnabledAttr().Set(True)
        return

    def pause(self) -> None:
        """Pause sensor data collection by disabling the contact sensor."""
        self._isaac_sensor_prim.GetEnabledAttr().Set(False)
        return

    def is_paused(self) -> bool:
        """Check if the contact sensor is currently paused.

        Returns:
            True if the sensor is paused, False otherwise.
        """
        if not self._isaac_sensor_prim.GetEnabledAttr().Get():
            return True
        return False

    def set_frequency(self, value: float) -> None:
        """Set the sensor sampling frequency.

        Args:
            value: The frequency in Hz to set for the sensor.
        """
        self._isaac_sensor_prim.GetSensorPeriodAttr().Set(1.0 / value)
        return

    def get_frequency(self) -> int:
        """Get the current sensor sampling frequency.

        Returns:
            The sensor frequency in Hz.
        """
        return int(1.0 / self._isaac_sensor_prim.GetSensorPeriodAttr().Get())

    def get_dt(self) -> float:
        """Get the sensor sampling time interval.

        Returns:
            The time interval between sensor readings in seconds.
        """
        return self._isaac_sensor_prim.GetSensorPeriodAttr().Get()

    def set_dt(self, value: float) -> None:
        """Set the sensor period (time step) for the contact sensor.

        Args:
            value: The sensor period in seconds.
        """
        self._isaac_sensor_prim.GetSensorPeriodAttr().Set(value)
        return

    def get_radius(self) -> float:
        """Radius of the contact sensor detection area.

        Returns:
            The radius value of the sensor detection area.
        """
        return self.prim.GetAttribute("radius").Get()

    def set_radius(self, value: float) -> None:
        """Set the radius of the contact sensor detection area.

        Args:
            value: The radius value to set.
        """
        if self.get_radius() is None:
            self._isaac_sensor_prim.CreateRadiusAttr().Set(value)
        else:
            self.prim.GetAttribute("radius").Set(value)
        return

    def get_min_threshold(self) -> float:
        """Minimum force threshold for contact detection.

        Returns:
            The minimum force threshold value.
        """
        threshold = self.prim.GetAttribute("threshold").Get()
        if threshold is not None:
            return threshold[0]
        else:
            return None

    def set_min_threshold(self, value: float) -> None:
        """Set the minimum force threshold for contact detection.

        Args:
            value: The minimum threshold value.
        """
        if self.get_min_threshold() is None:
            self._isaac_sensor_prim.CreateThresholdAttr().Set((value, 10000))
        else:
            self.prim.GetAttribute("threshold").Set((value, self.get_max_threshold()))
        return

    def get_max_threshold(self) -> float:
        """Maximum force threshold for contact detection.

        Returns:
            The maximum force threshold value.
        """
        threshold = self.prim.GetAttribute("threshold").Get()
        if threshold is not None:
            return threshold[1]
        else:
            return None

    def set_max_threshold(self, value: float) -> None:
        """Set the maximum force threshold for contact detection.

        Args:
            value: The maximum threshold value.
        """
        if self.get_max_threshold() is None:
            self._isaac_sensor_prim.CreateThresholdAttr().Set((0, value))
        else:
            self.prim.GetAttribute("threshold").Set((self.get_min_threshold(), value))
        return
