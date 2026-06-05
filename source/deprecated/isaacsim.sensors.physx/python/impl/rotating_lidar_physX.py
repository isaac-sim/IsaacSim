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

"""Implementation of rotating lidar sensor using PhysX simulation for range detection and data acquisition."""

from typing import Optional

import carb
import carb.eventdispatcher
import numpy as np
import omni
import omni.isaac.RangeSensorSchema as RangeSensorSchema
import omni.physics.core
import omni.timeline
from isaacsim.core.api.sensors.base_sensor import BaseSensor
from isaacsim.core.utils.prims import get_prim_at_path, is_prim_path_valid
from isaacsim.core.utils.stage import get_current_stage
from isaacsim.sensors.physx import _range_sensor
from pxr import Sdf


class RotatingLidarPhysX(BaseSensor):
    """A rotating lidar sensor using PhysX simulation for range detection.

    This sensor provides rotating lidar functionality with configurable field of view, resolution, and rotation
    frequency. It captures depth, intensity, point cloud, and other lidar data types during simulation. The sensor
    can create a new lidar prim at the specified path or use an existing one.

    Args:
        prim_path: Path to the lidar prim in the USD stage.
        name: Name identifier for the sensor.
        rotation_frequency: Rotation frequency of the lidar in Hz. Cannot be specified together with rotation_dt.
        rotation_dt: Time step for rotation in seconds. Cannot be specified together with rotation_frequency.
        position: Position of the sensor in 3D space.
        translation: Translation offset for the sensor.
        orientation: Orientation of the sensor as a quaternion or rotation matrix.
        fov: Field of view as (horizontal_fov, vertical_fov) in degrees.
        resolution: Resolution as (horizontal_resolution, vertical_resolution) in degrees.
        valid_range: Valid detection range as (min_range, max_range) in meters.

    Raises:
        Exception: If both rotation_frequency and rotation_dt are specified.
    """

    def __init__(
        self,
        prim_path: str,
        name: str = "rotating_lidar_physX",
        rotation_frequency: Optional[float] = None,
        rotation_dt: Optional[float] = None,
        position: Optional[np.ndarray] = None,
        translation: Optional[np.ndarray] = None,
        orientation: Optional[np.ndarray] = None,
        fov: Optional[tuple[float, float]] = None,
        resolution: Optional[tuple[float, float]] = None,
        valid_range: Optional[tuple[float, float]] = None,
    ) -> None:
        if rotation_frequency is not None and rotation_dt is not None:
            raise Exception("Rotation Frequency and Rotation dt can't be both specified")

        if rotation_dt is not None:
            rotation_frequency = int(1 / rotation_dt)

        self._lidar_sensor_interface = _range_sensor.acquire_lidar_sensor_interface()
        if is_prim_path_valid(prim_path):
            self._lidar_prim = RangeSensorSchema.Lidar(get_prim_at_path(prim_path))
        else:
            carb.log_info(f"Creating a new Lidar prim at path {prim_path}")
            self._lidar_prim = RangeSensorSchema.Lidar.Define(get_current_stage(), Sdf.Path(prim_path))
            if rotation_frequency is None:
                rotation_frequency = 20.0
            if fov is None:
                fov = (360.0, 10.0)
            if resolution is None:
                resolution = (1.0, 1.0)
            if valid_range is None:
                valid_range = (0.4, 2000.0)
        BaseSensor.__init__(
            self, prim_path=prim_path, name=name, translation=translation, position=position, orientation=orientation
        )
        if rotation_frequency is not None:
            self.set_rotation_frequency(rotation_frequency)
        if fov is not None:
            self.set_fov(fov)
        if resolution is not None:
            self.set_resolution(resolution)
        if valid_range is not None:
            self.set_valid_range(valid_range)
        self._pause = False
        self._current_time = 0
        self._number_of_physics_steps = 0
        self._current_frame = {}
        self._current_frame["time"] = 0
        self._current_frame["physics_step"] = 0
        return

    def initialize(self, physics_sim_view: object = None) -> None:
        """Initialize the rotating lidar sensor with physics simulation callbacks.

        Sets up physics step callbacks for data acquisition and event observers for stage
        and timeline events.

        Args:
            physics_sim_view: Physics simulation view for initialization.
        """
        BaseSensor.initialize(self, physics_sim_view=physics_sim_view)
        self._acquisition_callback = (
            omni.physics.core.get_physics_simulation_interface().subscribe_physics_on_step_events(
                pre_step=False, order=0, on_update=self._data_acquisition_callback
            )
        )
        self._stage_open_callback = carb.eventdispatcher.get_eventdispatcher().observe_event(
            event_name=omni.usd.get_context().stage_event_name(omni.usd.StageEventType.OPENED),
            on_event=self._stage_open_callback_fn,
            observer_name="isaacsim.sensors.physx.RotatingLidarPhysX.initialize._stage_open_callback",
        )
        self._timer_reset_callback = carb.eventdispatcher.get_eventdispatcher().observe_event(
            event_name=omni.timeline.GLOBAL_EVENT_STOP,
            on_event=self._timeline_stop_callback_fn,
            observer_name="isaacsim.sensors.physx.RotatingLidarPhysX.initialize._timeline_stop_callback",
        )
        return

    def _stage_open_callback_fn(self, event: object) -> None:
        """Handle stage open events by cleaning up callbacks.

        Args:
            event: Stage open event data.
        """
        self._acquisition_callback = None
        self._timer_reset_callback = None
        self._stage_open_callback = None
        return

    def _timeline_stop_callback_fn(self, event: object) -> None:
        """Handle timeline stop events by resetting time and physics step counters.

        Args:
            event: Timeline stop event data.
        """
        self._current_time = 0
        self._number_of_physics_steps = 0
        return

    def post_reset(self) -> None:
        """Reset the lidar sensor state after simulation reset.

        Resets time and physics step counters to zero.
        """
        BaseSensor.post_reset(self)
        self._current_time = 0
        self._number_of_physics_steps = 0
        return

    def add_depth_data_to_frame(self) -> None:
        """Enable depth data collection in the sensor frame.

        Adds a 'depth' key to the current frame dictionary for storing depth measurements.
        """
        self._current_frame["depth"] = None
        return

    def remove_depth_data_from_frame(self) -> None:
        """Disable depth data collection in the sensor frame.

        Removes the 'depth' key from the current frame dictionary.
        """
        del self._current_frame["depth"]
        return

    def add_linear_depth_data_to_frame(self) -> None:
        """Enable linear depth data collection in the sensor frame.

        Adds a 'linear_depth' key to the current frame dictionary for storing linear depth measurements.
        """
        self._current_frame["linear_depth"] = None
        return

    def remove_linear_depth_data_from_frame(self) -> None:
        """Disable linear depth data collection in the sensor frame.

        Removes the 'linear_depth' key from the current frame dictionary.
        """
        del self._current_frame["linear_depth"]
        return

    def add_intensity_data_to_frame(self) -> None:
        """Enable intensity data collection in the sensor frame.

        Adds an 'intensity' key to the current frame dictionary for storing intensity measurements.
        """
        self._current_frame["intensity"] = None
        return

    def remove_intensity_data_from_frame(self) -> None:
        """Disable intensity data collection in the sensor frame.

        Removes the 'intensity' key from the current frame dictionary.
        """
        del self._current_frame["intensity"]
        return

    def add_zenith_data_to_frame(self) -> None:
        """Adds zenith angle data to the current lidar frame for collection during data acquisition."""
        self._current_frame["zenith"] = None
        return

    def remove_zenith_data_from_frame(self) -> None:
        """Removes zenith angle data from the current lidar frame to stop collecting this data type."""
        del self._current_frame["zenith"]
        return

    def add_azimuth_data_to_frame(self) -> None:
        """Adds azimuth angle data to the current lidar frame for collection during data acquisition."""
        self._current_frame["azimuth"] = None
        return

    def remove_azimuth_data_from_frame(self) -> None:
        """Removes azimuth angle data from the current lidar frame to stop collecting this data type."""
        del self._current_frame["azimuth"]
        return

    def add_point_cloud_data_to_frame(self) -> None:
        """Adds point cloud data to the current lidar frame for collection during data acquisition."""
        self._current_frame["point_cloud"] = None
        return

    def remove_point_cloud_data_from_frame(self) -> None:
        """Removes point cloud data from the current lidar frame to stop collecting this data type."""
        del self._current_frame["point_cloud"]
        return

    def add_semantics_data_to_frame(self) -> None:
        """Adds semantic segmentation data to the current lidar frame for collection during data acquisition.

        Automatically enables semantics on the lidar sensor if not already enabled.
        """
        if not self.is_semantics_enabled():
            self.enable_semantics()
        self._current_frame["semantics"] = None
        return

    def remove_semantics_data_from_frame(self) -> None:
        """Removes semantic segmentation data from the current lidar frame and disables semantics on the sensor."""
        self.disable_semantics()
        del self._current_frame["semantics"]
        return

    def _data_acquisition_callback(self, step_size: float, context: object) -> None:
        """Physics callback that updates lidar data each simulation step.

        Collects all enabled data types from the lidar sensor interface and updates the current frame with
        tensor data for each configured data type.

        Args:
            step_size: Physics simulation step size in seconds.
            context: Physics simulation context.
        """
        self._current_time += step_size
        self._number_of_physics_steps += 1
        if not self._pause:
            for key in self._current_frame:
                if key not in ["physics_step", "time"]:
                    if key == "semantics":
                        self._current_frame[key] = self._backend_utils.create_tensor_from_list(
                            self._lidar_sensor_interface.get_semantic_data(self.prim_path),
                            dtype="float32",
                            device=self._device,
                        )
                    else:
                        self._current_frame[key] = self._backend_utils.create_tensor_from_list(
                            getattr(self._lidar_sensor_interface, f"get_{key}_data")(self.prim_path),
                            dtype="float32",
                            device=self._device,
                        )

            self._current_frame["physics_step"] = self._number_of_physics_steps
            self._current_frame["time"] = self._current_time
        return

    def get_num_rows(self) -> int:
        """Number of vertical resolution rows in the lidar sensor.

        Returns:
            The number of rows configured for the lidar sensor.
        """
        return self._lidar_sensor_interface.get_num_rows(self.prim_path)

    def get_num_cols(self) -> int:
        """Total number of columns in the lidar sensor.

        Returns:
            The total number of columns.
        """
        return self._lidar_sensor_interface.get_num_cols(self.prim_path)

    def get_num_cols_in_last_step(self) -> int:
        """Number of columns processed in the last physics step.

        Returns:
            The number of columns that were ticked in the last step.
        """
        return self._lidar_sensor_interface.get_num_cols_ticked(self.prim_path)

    def get_current_frame(self) -> dict:
        """Current frame data from the lidar sensor.

        Returns:
            Dictionary containing the current frame data with keys like 'time', 'physics_step', and any enabled data types.
        """
        return self._current_frame

    def resume(self) -> None:
        """Resumes lidar data acquisition.

        Unpauses the sensor to continue collecting data during physics steps.
        """
        self._pause = False
        return

    def pause(self) -> None:
        """Pauses lidar data acquisition.

        Stops the sensor from collecting new data while keeping it initialized.
        """
        self._pause = True
        return

    def is_paused(self) -> bool:
        """Pause state of the lidar sensor.

        Returns:
            True if the sensor is paused, False if it is actively collecting data.
        """
        return self._pause

    def set_fov(self, value: tuple[float, float]) -> None:
        """Sets the field of view for the lidar sensor.

        Args:
            value: Tuple of (horizontal_fov, vertical_fov) in degrees.
        """
        if self.prim.GetAttribute("horizontalFov").Get() is None:
            self._lidar_prim.CreateHorizontalFovAttr().Set(value[0])
        else:
            self.prim.GetAttribute("horizontalFov").Set(value[0])
        if self.prim.GetAttribute("verticalFov").Get() is None:
            self._lidar_prim.CreateVerticalFovAttr().Set(value[1])
        else:
            self.prim.GetAttribute("verticalFov").Set(value[1])

    def get_fov(self) -> tuple[float, float]:
        """Field of view of the lidar sensor.

        Returns:
            Tuple of (horizontal_fov, vertical_fov) in degrees.
        """
        return self.prim.GetAttribute("horizontalFov").Get(), self.prim.GetAttribute("verticalFov").Get()

    def set_resolution(self, value: float) -> None:
        """Sets the resolution for the lidar sensor.

        Args:
            value: Resolution value in degrees per sample.
        """
        if self.prim.GetAttribute("horizontalResolution").Get() is None:
            self._lidar_prim.CreateHorizontalResolutionAttr().Set(value[0])
        else:
            self.prim.GetAttribute("horizontalResolution").Set(value[0])
        if self.prim.GetAttribute("verticalResolution").Get() is None:
            self._lidar_prim.CreateVerticalResolutionAttr().Set(value[1])
        else:
            self.prim.GetAttribute("verticalResolution").Set(value[1])

    def get_resolution(self) -> float:
        """Resolution of the lidar sensor.

        Returns:
            Tuple of (horizontal_resolution, vertical_resolution) in degrees per sample.
        """
        return self.prim.GetAttribute("horizontalResolution").Get(), self.prim.GetAttribute("verticalResolution").Get()

    def set_rotation_frequency(self, value: int) -> None:
        """Sets the rotation frequency of the lidar sensor.

        Args:
            value: Rotation rate in rotations per second.
        """
        if self.get_rotation_frequency() is None:
            self._lidar_prim.CreateRotationRateAttr().Set(value)
        else:
            self.prim.GetAttribute("rotationRate").Set(value)

    def get_rotation_frequency(self) -> int:
        """Rotation frequency of the lidar sensor in rotations per second.

        Returns:
            The current rotation rate.
        """
        return self.prim.GetAttribute("rotationRate").Get()

    def set_valid_range(self, value: tuple[float, float]) -> None:
        """Sets the valid range of the lidar sensor.

        Args:
            value: Tuple of (minimum_range, maximum_range) in meters.
        """
        if self.prim.GetAttribute("minRange").Get() is None:
            self._lidar_prim.CreateMinRangeAttr().Set(value[0])
        else:
            self.prim.GetAttribute("minRange").Set(value[0])
        if self.prim.GetAttribute("maxRange").Get() is None:
            self._lidar_prim.CreateMaxRangeAttr().Set(value[1])
        else:
            self.prim.GetAttribute("maxRange").Set(value[1])
        return

    def get_valid_range(self) -> tuple[float, float]:
        """Valid range of the lidar sensor.

        Returns:
            Tuple of (minimum_range, maximum_range) in meters.
        """
        return self.prim.GetAttribute("minRange").Get(), self.prim.GetAttribute("maxRange").Get()

    def enable_semantics(self) -> None:
        """Enables semantic data collection for the lidar sensor."""
        if self.prim.GetAttribute("enableSemantics").Get() is None:
            self._lidar_prim.CreateEnableSemanticsAttr().Set(True)
        else:
            self.prim.GetAttribute("enableSemantics").Set(True)

    def disable_semantics(self) -> None:
        """Disables semantic data collection for the lidar sensor."""
        if self.prim.GetAttribute("enableSemantics").Get() is None:
            self._lidar_prim.CreateEnableSemanticsAttr().Set(True)
        else:
            self.prim.GetAttribute("enableSemantics").Set(True)

    def is_semantics_enabled(self) -> bool:
        """Whether semantic data collection is enabled for the lidar sensor.

        Returns:
            True if semantics are enabled, False otherwise.
        """
        return self.prim.GetAttribute("enableSemantics").Get()

    def enable_visualization(self, high_lod: bool = False, draw_points: bool = True, draw_lines: bool = True) -> None:
        """Enables visualization of the lidar sensor data.

        Args:
            high_lod: Whether to use high level of detail for visualization.
            draw_points: Whether to draw point cloud visualization.
            draw_lines: Whether to draw line visualization.
        """
        if self.prim.GetAttribute("highLod").Get() is None:
            self._lidar_prim.CreateHighLodAttr().Set(high_lod)
        else:
            self.prim.GetAttribute("highLod").Set(high_lod)

        if self.prim.GetAttribute("drawPoints").Get() is None:
            RangeSensorSchema.RangeSensor(self._lidar_prim).CreateDrawPointsAttr().Set(draw_points)
        else:
            self.prim.GetAttribute("drawPoints").Set(draw_points)

        if self.prim.GetAttribute("drawLines").Get() is None:
            RangeSensorSchema.RangeSensor(self._lidar_prim).CreateDrawLinesAttr().Set(draw_lines)
        else:
            self.prim.GetAttribute("drawLines").Set(draw_lines)
        return

    def disable_visualization(self) -> None:
        """Disables visualization of the lidar sensor data."""
        self.enable_visualization(high_lod=False, draw_points=False, draw_lines=False)
        return
