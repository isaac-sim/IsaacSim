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

"""Implementation of a physics-based IMU (Inertial Measurement Unit) sensor for Isaac Sim."""

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
from pxr import PhysxSchema


class IMUSensor(BaseSensor):
    """A physics-based IMU (Inertial Measurement Unit) sensor that measures linear acceleration, angular velocity, and orientation.

    This sensor provides inertial measurements from a simulated IMU attached to a physics body. It measures three-axis linear acceleration, three-axis angular velocity, and orientation as a quaternion. The sensor supports configurable sampling rates and filtering for each measurement type.

    The sensor can be created at an existing prim path or will automatically create a new IMU prim if the path doesn't exist. When creating a new prim, it uses physics simulation settings to determine default timing parameters.

    Args:
        prim_path: USD path where the IMU sensor prim is located or will be created.
        name: Identifier name for the sensor instance.
        frequency: Sampling frequency in Hz for sensor readings. Cannot be specified together with dt.
        dt: Time interval in seconds between sensor readings. Cannot be specified together with frequency.
        translation: Local translation offset of the sensor relative to its parent body.
        position: Local position offset of the sensor relative to its parent body.
        orientation: Local orientation offset of the sensor as a quaternion relative to its parent body.
        linear_acceleration_filter_size: Number of samples for moving average filter applied to linear acceleration
            measurements.
        angular_velocity_filter_size: Number of samples for moving average filter applied to angular velocity
            measurements.
        orientation_filter_size: Number of samples for moving average filter applied to orientation measurements.

    Raises:
        Exception: If both frequency and dt are specified simultaneously.
        Exception: If IMU sensor prim creation fails at the specified path.
    """

    def __init__(
        self,
        prim_path: str,
        name: str | None = "imu_sensor",
        frequency: int | None = None,
        dt: float | None = None,
        translation: np.ndarray | None = None,
        position: np.ndarray | None = None,
        orientation: np.ndarray | None = None,
        linear_acceleration_filter_size: int | None = 1,
        angular_velocity_filter_size: int | None = 1,
        orientation_filter_size: int | None = 1,
    ) -> None:
        if frequency is not None and dt is not None:
            raise Exception("Sensor Frequency and Sensor dt can't be both specified")

        if frequency is not None:
            dt = float(1 / frequency)

        self._body_prim_path = "/".join(prim_path.split("/")[:-1])
        self._sensor_name = prim_path.split("/")[-1]
        self._imu_sensor_interface = _sensor.acquire_imu_sensor_interface()

        linear_acceleration_filter_size = max(linear_acceleration_filter_size, 1)
        angular_velocity_filter_size = max(angular_velocity_filter_size, 1)
        orientation_filter_size = max(orientation_filter_size, 1)

        if is_prim_path_valid(prim_path):
            self._isaac_sensor_prim = IsaacSensorSchema.IsaacImuSensor(get_prim_at_path(prim_path))
            BaseSensor.__init__(
                self,
                prim_path=prim_path,
                name=name,
                translation=translation,
                position=position,
                orientation=orientation,
            )
            if dt is not None:
                self.set_dt(dt)
        else:
            if dt is None:
                current_physics_prim = None
                for prim in traverse_stage():
                    if prim.HasAPI(PhysxSchema.PhysxSceneAPI):
                        current_physics_prim = prim
                        break
                if current_physics_prim is None:
                    raise ValueError("No PhysicsScene found on stage. Cannot determine default dt for IMUSensor.")
                physx_scene_api = PhysxSchema.PhysxSceneAPI(current_physics_prim)
                current_physics_frequency = physx_scene_api.GetTimeStepsPerSecondAttr().Get()
                dt = 1.0 / current_physics_frequency
            carb.log_warn(f"Creating a new IMU prim at path {prim_path}")
            success, self._isaac_sensor_prim = omni.kit.commands.execute(
                "IsaacSensorCreateImuSensor",
                path="/" + self._sensor_name,
                parent=self._body_prim_path,
                sensor_period=dt,
                linear_acceleration_filter_size=linear_acceleration_filter_size,
                angular_velocity_filter_size=angular_velocity_filter_size,
                orientation_filter_size=orientation_filter_size,
            )
            if not success:
                raise Exception("Not successful")
            BaseSensor.__init__(
                self,
                prim_path=prim_path,
                name=name,
                translation=translation,
                position=position,
                orientation=orientation,
            )
        self._pause = False
        self._current_time = 0
        self._current_frame = {}
        self._current_frame["time"] = 0
        self._current_frame["physics_step"] = 0
        self._current_frame["lin_acc"] = self._backend_utils.create_zeros_tensor(
            shape=[3], dtype="float32", device=self._device
        )
        self._current_frame["ang_vel"] = self._backend_utils.create_zeros_tensor(
            shape=[3], dtype="float32", device=self._device
        )
        self._current_frame["orientation"] = self._backend_utils.create_zeros_tensor(
            shape=[4], dtype="float32", device=self._device
        )
        self._current_frame["orientation"][0] = 1
        self._current_frame["is_valid"] = False
        return

    def initialize(self, physics_sim_view: object = None) -> None:
        """Initialize the IMU sensor.

        Args:
            physics_sim_view: The physics simulation view to use for initialization.
        """
        BaseSensor.initialize(self, physics_sim_view=physics_sim_view)

    def get_current_frame(self, read_gravity: bool = True) -> dict:
        """Get the current sensor reading frame containing IMU data.

        Args:
            read_gravity: Whether to include gravity in the linear acceleration reading.

        Returns:
            Dictionary containing sensor data with keys:
            - 'lin_acc': Linear acceleration as a 3D tensor
            - 'ang_vel': Angular velocity as a 3D tensor
            - 'orientation': Orientation quaternion in scalar-first (w, x, y, z) order as a 4D tensor
            - 'time': Sensor reading timestamp
            - 'physics_step': Current physics simulation step
            - 'is_valid': Whether the current sensor reading is valid
        """
        imu_sensor_reading = self._imu_sensor_interface.get_sensor_reading(self.prim_path, read_gravity=read_gravity)
        self._current_frame["is_valid"] = bool(imu_sensor_reading.is_valid)
        if imu_sensor_reading.is_valid:
            self._current_frame["lin_acc"] = self._backend_utils.create_tensor_from_list(
                [
                    imu_sensor_reading.lin_acc_x,
                    imu_sensor_reading.lin_acc_y,
                    imu_sensor_reading.lin_acc_z,
                ],
                dtype="float32",
                device=self._device,
            )
            self._current_frame["ang_vel"] = self._backend_utils.create_tensor_from_list(
                [
                    imu_sensor_reading.ang_vel_x,
                    imu_sensor_reading.ang_vel_y,
                    imu_sensor_reading.ang_vel_z,
                ],
                dtype="float32",
                device=self._device,
            )
            self._current_frame["orientation"] = self._backend_utils.create_tensor_from_list(
                [
                    imu_sensor_reading.orientation[3],
                    imu_sensor_reading.orientation[0],
                    imu_sensor_reading.orientation[1],
                    imu_sensor_reading.orientation[2],
                ],
                dtype="float32",
                device=self._device,
            )
            self._current_frame["time"] = float(imu_sensor_reading.time)
            self._current_frame["physics_step"] = float(SimulationManager.get_num_physics_steps())
        return self._current_frame

    def resume(self) -> None:
        """Resume the IMU sensor by enabling it."""
        self._isaac_sensor_prim.GetEnabledAttr().Set(True)
        return

    def pause(self) -> None:
        """Pause the IMU sensor by disabling it."""
        self._isaac_sensor_prim.GetEnabledAttr().Set(False)
        return

    def is_paused(self) -> bool:
        """Check if the IMU sensor is currently paused.

        Returns:
            True if the sensor is paused, False if enabled.
        """
        if not self._isaac_sensor_prim.GetEnabledAttr().Get():
            return True
        return False

    def set_frequency(self, value: int) -> None:
        """Set the IMU sensor sampling frequency.

        Args:
            value: The sampling frequency in Hz.
        """
        self._isaac_sensor_prim.GetSensorPeriodAttr().Set(1.0 / value)
        return

    def get_frequency(self) -> int:
        """Get the current IMU sensor sampling frequency.

        Returns:
            The sampling frequency in Hz.
        """
        return int(1.0 / self._isaac_sensor_prim.GetSensorPeriodAttr().Get())

    def get_dt(self) -> float:
        """Get the current IMU sensor sampling period.

        Returns:
            The sampling period in seconds.
        """
        return self._isaac_sensor_prim.GetSensorPeriodAttr().Get()

    def set_dt(self, value: float) -> None:
        """Set the IMU sensor sampling period.

        Args:
            value: The sampling period in seconds.
        """
        self._isaac_sensor_prim.GetSensorPeriodAttr().Set(value)
        return
