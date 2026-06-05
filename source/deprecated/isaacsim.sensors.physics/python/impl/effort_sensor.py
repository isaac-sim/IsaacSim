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

"""Implementation of effort sensors for measuring joint forces and torques in articulated bodies."""

import copy

import carb
import carb.eventdispatcher
import omni.kit.utils
import omni.physics.core
import omni.timeline
import omni.usd
from isaacsim.core.prims import SingleArticulation
from isaacsim.core.simulation_manager import SimulationManager


class EsSensorReading:
    """A container for effort sensor measurement data.

    This class encapsulates a single reading from an effort sensor, containing the measured value,
    timestamp, and validity status. It is used by EffortSensor to store and manage sensor data
    in buffers for interpolation and retrieval.

    Args:
        is_valid: Whether the sensor reading contains valid data.
        time: Simulation time when the measurement was taken.
        value: The measured effort value from the sensor.
    """

    def __init__(self, is_valid: bool = False, time: float = 0, value: float = 0) -> None:
        self.is_valid = is_valid
        self.time = time
        self.value = value


class EffortSensor(SingleArticulation):
    """A sensor for measuring joint effort (force/torque) in articulated bodies.

    This sensor monitors the effort applied to a specific joint in an articulated body during physics simulation.
    It provides configurable sampling rates and data interpolation capabilities for accurate measurements.
    The sensor automatically manages data acquisition through physics simulation callbacks and maintains
    internal buffers for temporal data processing.

    Args:
        prim_path: Full USD path to the joint, including the articulated body path and joint name
            (e.g., "/World/Robot/joint_name").
        sensor_period: Time interval between sensor readings in seconds. If negative or less than the physics
            step size, readings are taken at every physics step.
        use_latest_data: Whether to always return the most recent measurement instead of interpolated values.
        enabled: Whether the sensor is actively collecting data.
    """

    def __init__(
        self,
        prim_path: str,
        sensor_period: float = -1,
        use_latest_data: bool = False,
        enabled: bool = True,
    ) -> None:
        self.current_time = 0
        self.sensor_time = 0
        self.sensor_period = sensor_period
        self.enabled = enabled
        self.use_latest_data = use_latest_data
        self.step_size = 0
        self.data_buffer_size = 10
        self.sensor_reading_buffer = [EsSensorReading() for i in range(self.data_buffer_size)]
        self.interpolation_buffer = copy.deepcopy(self.sensor_reading_buffer)
        self.physics_num_steps = 0
        self.is_initialized = False
        self.dof = None

        self.body_prim_path = "/".join(prim_path.split("/")[:-1])
        self.dof_name = prim_path.split("/")[-1]

        super().__init__(prim_path=self.body_prim_path, name=prim_path)
        self.initialize_callbacks()
        return

    def initialize_callbacks(self) -> None:
        """Initialize physics and timeline event callbacks for the effort sensor.

        Sets up callbacks for physics step events, stage opening events, and timeline play/stop events
        to manage sensor data acquisition and state reset.
        """
        self._acquisition_callback = (
            omni.physics.core.get_physics_simulation_interface().subscribe_physics_on_step_events(
                pre_step=False, order=0, on_update=self._data_acquisition_callback
            )
        )
        self._stage_open_callback = carb.eventdispatcher.get_eventdispatcher().observe_event(
            event_name=omni.usd.get_context().stage_event_name(omni.usd.StageEventType.OPENED),
            on_event=self._stage_open_callback_fn,
            observer_name="isaacsim.sensors.physics.EffortSensor.initialize._stage_open_callback",
        )
        self._timer_reset_callback_stop = carb.eventdispatcher.get_eventdispatcher().observe_event(
            event_name=omni.timeline.GLOBAL_EVENT_STOP,
            on_event=self._timeline_stop_callback_fn,
            observer_name="isaacsim.sensors.physics.EffortSensor.initialize._timeline_stop_callback",
        )
        self._timer_reset_callback_play = carb.eventdispatcher.get_eventdispatcher().observe_event(
            event_name=omni.timeline.GLOBAL_EVENT_PLAY,
            on_event=self._timeline_play_callback_fn,
            observer_name="isaacsim.sensors.physics.EffortSensor.initialize._timeline_play_callback",
        )

    def lerp(self, start: float, end: float, time: float) -> float:
        """Perform linear interpolation between two values.

        Args:
            start: Starting value.
            end: Ending value.
            time: Interpolation factor between 0 and 1.

        Returns:
            Interpolated value between start and end.
        """
        return start + ((end - start) * time)

    def _stage_open_callback_fn(self, event: object = None) -> None:
        """Handle stage open events by clearing all registered callbacks.

        Args:
            event: Stage open event data.
        """
        self._acquisition_callback = None
        self._timer_reset_callback_stop = None
        self._timer_reset_callback_play = None
        self._stage_open_callback = None
        return

    def _timeline_stop_callback_fn(self, event: object) -> None:
        """Handle timeline stop events by resetting sensor state and buffers.

        Args:
            event: Timeline stop event data.
        """
        self.current_time = 0
        self.sensor_time = 0
        self.sensor_reading_buffer = [EsSensorReading() for i in range(self.data_buffer_size)]
        self.interpolation_buffer = copy.deepcopy(self.sensor_reading_buffer)
        self.physics_num_steps = 0
        return

    def _timeline_play_callback_fn(self, event: object) -> None:
        """Handle timeline play events by marking the sensor as uninitialized.

        Args:
            event: Timeline play event data.
        """
        self.is_initialized = False
        return

    def _data_acquisition_callback(self, step_size: float, context: object) -> None:
        """Acquire joint effort data during physics steps.

        Reads joint effort measurements and updates the sensor buffer with timestamped readings.
        Initializes the sensor on first valid physics step.

        Args:
            step_size: Physics simulation step size.
            context: Physics simulation context.
        """
        # update sensor reading pair
        self.step_size = step_size
        self.current_time = float(SimulationManager.get_simulation_time())
        self.physics_num_steps = float(SimulationManager.get_num_physics_steps())
        if self.physics_num_steps <= 2:
            return

        elif not self.is_initialized:
            super().initialize()
            self.dof = self.get_dof_index(self.dof_name)
            self.is_initialized = True

        if self.enabled:
            efforts = self.get_measured_joint_efforts(self.dof)
            self.sensor_reading_buffer.insert(0, EsSensorReading())
            self.sensor_reading_buffer.pop()

            self.sensor_reading_buffer[0].time = self.current_time

            if efforts is None or len(efforts) != 1:
                self.sensor_reading_buffer[0].value = 0
                self.sensor_reading_buffer[0].is_valid = False
                carb.log_warn(
                    f"Effort sensor error, none or multiple efforts found for path:  {self.prim_path} with joint {self.dof_name}"
                )

            else:
                self.sensor_reading_buffer[0].value = efforts[0]
                self.sensor_reading_buffer[0].is_valid = True

            if self.sensor_period <= self.step_size:
                self.sensor_time = self.current_time
            elif self.sensor_time + self.sensor_period <= self.current_time:
                self.interpolation_buffer = copy.deepcopy(self.sensor_reading_buffer)
                self.sensor_time += self.sensor_period

    def get_sensor_reading(
        self, interpolation_function: object = None, use_latest_data: bool = False
    ) -> EsSensorReading:
        """Get the current sensor reading with optional interpolation.

        Returns either the latest reading or an interpolated value based on sensor period and timing.
        Handles cases where sensor frequency differs from physics step frequency.

        Args:
            interpolation_function: Custom interpolation function to use instead of linear interpolation.
            use_latest_data: Whether to force using the most recent data regardless of sensor period.

        Returns:
            Sensor reading containing timestamp, effort value, and validity status.
        """
        sensor_reading = EsSensorReading()
        if self.enabled:
            # case 1: get latest reading when sensor freq is higher
            # use latest data step enabled
            # sensor time + period is slower than last step (out of sync)
            if (
                self.sensor_period <= self.step_size
                or self.use_latest_data
                or use_latest_data
                or self.sensor_time + self.sensor_period < self.sensor_reading_buffer[1].time
            ):
                sensor_reading = self.sensor_reading_buffer[0]

                # if the data is invalid, output default
                if not self.sensor_reading_buffer[0].is_valid:
                    sensor_reading = EsSensorReading()

                elif (
                    self.sensor_period > self.step_size
                    and not self.use_latest_data
                    and not use_latest_data
                    and self.sensor_time + self.sensor_period < self.sensor_reading_buffer[1].time
                ):
                    carb.log_warn("sensor time out of sync, using latest data")

            # case 2: use interpolated data
            else:
                sensor_reading.time = self.sensor_time
                # if both pairs are valid, interpolate
                if self.sensor_reading_buffer[0].is_valid and self.sensor_reading_buffer[1].is_valid:
                    if interpolation_function is None:
                        if self.interpolation_buffer[1].time == self.interpolation_buffer[0].time:
                            if self.step_size == 0:
                                sensor_reading.value = self.interpolation_buffer[0].value
                            else:
                                sensor_reading.value = self.lerp(
                                    self.interpolation_buffer[1].value,
                                    self.interpolation_buffer[0].value,
                                    float((sensor_reading.time - self.interpolation_buffer[1].time) / (self.step_size)),
                                )
                        else:
                            sensor_reading.value = self.lerp(
                                self.interpolation_buffer[1].value,
                                self.interpolation_buffer[0].value,
                                float(
                                    (sensor_reading.time - self.interpolation_buffer[1].time)
                                    / (self.interpolation_buffer[0].time - self.interpolation_buffer[1].time)
                                ),
                            )
                        sensor_reading.is_valid = True
                    else:
                        sensor_reading = interpolation_function(self.interpolation_buffer, self.sensor_time)

                else:
                    # if the most recent one is valid, but old data is not, use the most recent
                    if self.sensor_reading_buffer[0].is_valid:
                        sensor_reading = copy.copy(self.sensor_reading_buffer[0])

                    # no valid data, reset it
                    else:
                        sensor_reading = EsSensorReading()
        return sensor_reading

    def update_dof_name(self, dof_name: str) -> None:
        """Update the degree of freedom name for effort measurement.

        Changes which joint the sensor monitors for effort data. Requires at least 3 physics steps
        to have passed for proper initialization.

        Args:
            dof_name: Name of the joint degree of freedom to monitor.
        """
        if self.physics_num_steps <= 2:
            carb.log_warn("unable to update path, please call again after 3 physics steps")
            return
        self.dof_name = dof_name
        try:
            self.dof = self.get_dof_index(self.dof_name)
        except Exception:
            carb.log_warn("unable to find joint corresponding to the dof name, disabling sensor")
            self.dof = None

    def change_buffer_size(self, new_buffer_size: int) -> None:
        """Resize the sensor data buffers to a new size.

        Adjusts both the main sensor reading buffer and interpolation buffer to the specified size.

        Args:
            new_buffer_size: New size for the sensor data buffers.
        """
        current = len(self.sensor_reading_buffer)
        if new_buffer_size > current:
            extra = new_buffer_size - current
            self.sensor_reading_buffer = list(self.sensor_reading_buffer) + [EsSensorReading() for _ in range(extra)]
            self.interpolation_buffer = list(self.interpolation_buffer) + [EsSensorReading() for _ in range(extra)]
        elif new_buffer_size < current:
            self.sensor_reading_buffer = list(self.sensor_reading_buffer)[:new_buffer_size]
            self.interpolation_buffer = list(self.interpolation_buffer)[:new_buffer_size]
        self.data_buffer_size = new_buffer_size
