# SPDX-FileCopyrightText: Copyright (c) 2023-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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

"""RTX-based Lidar sensor implementation for Isaac Sim.

This module provides the LidarRtx class for creating and managing RTX-based Lidar sensors
in Isaac Sim. It supports various annotators for data collection and visualization.
"""

from __future__ import annotations

from collections.abc import Callable
from typing import Any, Literal

import carb
import numpy as np
import omni
import omni.graph.core as og
import omni.replicator.core as rep
from isaacsim.core.api.sensors.base_sensor import BaseSensor
from isaacsim.core.simulation_manager import _simulation_manager
from isaacsim.core.utils.prims import get_prim_at_path, get_prim_type_name, is_prim_path_valid


class LidarRtx(BaseSensor):
    """RTX-based Lidar sensor implementation.

    This class provides functionality for creating and managing RTX-based Lidar sensors in Isaac Sim.
    It supports various annotators and writers for data collection and visualization.

    The sensor can be configured with different parameters and supports both point cloud and flat scan data collection.

    Args:
        prim_path: Path to the USD prim for the Lidar sensor.
        name: Name of the Lidar sensor.
        position: Global position of the sensor as [x, y, z].
        translation: Local translation of the sensor as [x, y, z].
        orientation: Orientation quaternion as [w, x, y, z].
        config_file_name: Path to the configuration file for the sensor.
        **kwargs: Additional keyword arguments for sensor configuration.

    Raises:
        Exception: If the prim at prim_path is not an OmniLidar or doesn't have the required API.
    """

    @staticmethod
    def make_add_remove_deprecated_attr(deprecated_attr: str) -> list[Callable]:
        """Create deprecated add/remove attribute methods.

        This is an internal helper method for creating deprecated methods that log
        warnings when called.

        Args:
            deprecated_attr: Name of the deprecated attribute to create methods for.

        Returns:
            List of method functions for adding and removing the deprecated attribute.
        """
        methods = []
        for fun_name in [f"add_{deprecated_attr}_to_frame", f"remove_{deprecated_attr}_from_frame"]:

            def attr_fun(self: object) -> None:
                carb.log_warn(
                    f"{fun_name} is deprecated as of Isaac Sim 5.0 and will be removed in a future release. This attribute is now automatically added to the current frame if the corresponding annotator is attached."
                )

            methods.append(attr_fun)
        return methods

    def __init__(
        self,
        prim_path: str,
        name: str = "lidar_rtx",
        position: np.ndarray | None = None,
        translation: np.ndarray | None = None,
        orientation: np.ndarray | None = None,
        config_file_name: str | None = None,
        **kwargs: Any,
    ) -> None:
        DEPRECATED_ARGS = [
            "firing_frequency",
            "firing_dt",
            "rotation_frequency",
            "rotation_dt",
            "valid_range",
            "scan_type",
            "elevation_range",
            "range_resolution",
            "range_accuracy",
            "avg_power",
            "wave_length",
            "pulse_time",
        ]
        for arg in DEPRECATED_ARGS:
            if arg in kwargs:
                carb.log_warn(
                    f"Argument {arg} is deprecated as of Isaac Sim 5.0 and will be removed in a future release."
                )
                kwargs.pop(arg)

        # Initialize dictionaries for annotators and writers
        self._annotators: dict[str, Any] = {}  # maps annotator name to annotator and node prim path
        self._writers: dict[str, Any] = {}  # maps writer name to writer
        self._render_product: Any = None
        self._render_product_path: str | None = None

        if is_prim_path_valid(prim_path):
            if get_prim_type_name(prim_path) == "Camera":
                carb.log_warn(
                    "Support for creating LidarRtx from Camera prims is deprecated as of Isaac Sim 5.0 and will be removed in a future release. Use OmniLidar prim instead."
                )
            elif get_prim_type_name(prim_path) != "OmniLidar":
                raise Exception(f"Prim at {prim_path} is not an OmniLidar.")
            elif not get_prim_at_path(prim_path).HasAPI("OmniSensorGenericLidarCoreAPI"):
                raise Exception(f"Prim at {prim_path} does not have the OmniSensorGenericLidarCoreAPI schema.")
            carb.log_warn(f"Using existing RTX Lidar prim at path {prim_path}")
            sensor = get_prim_at_path(prim_path)
            if sensor.HasAttribute("omni:sensor:Core:accumulateOutputs"):
                sensor.GetAttribute("omni:sensor:Core:accumulateOutputs").Set(True)
            for key, value in kwargs.items():
                if sensor.HasAttribute(key):
                    sensor.GetAttribute(key).Set(value)
                else:
                    carb.log_warn(f"Sensor at {prim_path} does not have attribute {key}")
        else:
            _, sensor = omni.kit.commands.execute(
                "IsaacSensorCreateRtxLidar",
                path=prim_path,
                parent=None,
                config=config_file_name,
                **kwargs,
            )
            prim_path = str(sensor.GetPath())

        # Move the sensor again
        BaseSensor.__init__(
            self, prim_path=prim_path, name=name, translation=translation, position=position, orientation=orientation
        )

        # Create render product
        self._render_product = rep.create.render_product(prim_path, resolution=(128, 128))
        self._render_product_path = self._render_product.path

        # Initialize simulation manager interface
        self._simulation_manager_interface = _simulation_manager.acquire_simulation_manager_interface()

        # Define data dictionary for current frame
        self._current_frame = dict[str, Any]()
        self._current_frame["rendering_time"] = 0
        self._current_frame["rendering_frame"] = 0

        return

    def __del__(self) -> None:
        """Clean up resources when the Lidar sensor is destroyed."""
        self.detach_all_writers()
        self.detach_all_annotators()
        if self._render_product:
            self._render_product.destroy()
        self._acquisition_callback = None
        self._stage_open_callback = None
        self._timer_reset_callback_pause = None
        self._timer_reset_callback_stop = None
        self._timer_reset_callback_play = None

    def get_render_product_path(self) -> str | None:
        """Get the path to the render product used by the Lidar.

        Returns:
            Path to the render product, or None if not initialized.

        Example:

        .. code-block:: python

            >>> from isaacsim.sensors.rtx import LidarRtx
            >>> lidar = LidarRtx(prim_path="/World/Lidar")
            >>> render_product_path = lidar.get_render_product_path()
        """
        return self._render_product_path

    def get_current_frame(self) -> dict:
        """Get the current frame data from the Lidar sensor.

        Returns:
            Dictionary containing the current frame data including rendering time,
            frame number, and any attached annotator data.

        Example:

        .. code-block:: python

            >>> from isaacsim.sensors.rtx import LidarRtx
            >>> lidar = LidarRtx(prim_path="/World/Lidar")
            >>> lidar.initialize()
            >>> frame_data = lidar.get_current_frame()
            >>> print(frame_data["rendering_time"])
        """
        return self._current_frame

    def get_annotators(self) -> dict:
        """Get all attached annotators.

        Returns:
            Dictionary mapping annotator names to their instances.

        Example:

        .. code-block:: python

            >>> from isaacsim.sensors.rtx import LidarRtx
            >>> lidar = LidarRtx(prim_path="/World/Lidar")
            >>> lidar.attach_annotator("IsaacComputeRTXLidarFlatScan")
            >>> annotators = lidar.get_annotators()
            >>> print(list(annotators.keys()))
        """
        return self._annotators

    def attach_annotator(
        self,
        annotator_name: Literal[
            "IsaacComputeRTXLidarFlatScan",
            "IsaacExtractRTXSensorPointCloudNoAccumulator",
            "IsaacCreateRTXLidarScanBuffer",
            "StableIdMap",
            "GenericModelOutput",
        ],
        **kwargs: object,
    ) -> None:
        """Attach an annotator to the Lidar sensor.

        Args:
            annotator_name: Name of the annotator to attach. Must be one of:
                "IsaacComputeRTXLidarFlatScan", "IsaacExtractRTXSensorPointCloudNoAccumulator",
                "IsaacCreateRTXLidarScanBuffer", "StableIdMap", or "GenericModelOutput".
            **kwargs: Additional arguments to pass to the annotator on initialization.

        Example:

        .. code-block:: python

            >>> from isaacsim.sensors.rtx import LidarRtx
            >>> lidar = LidarRtx(prim_path="/World/Lidar")
            >>> lidar.attach_annotator("IsaacComputeRTXLidarFlatScan")
            >>> lidar.attach_annotator("IsaacCreateRTXLidarScanBuffer")
        """
        if annotator_name in self._annotators:
            carb.log_warn(f"Annotator {annotator_name} already attached to {self._render_product_path}")
            return

        annotator = rep.AnnotatorRegistry.get_annotator(annotator_name)
        annotator.initialize(**kwargs)
        annotator.attach([self._render_product_path])
        self._annotators[annotator_name] = annotator
        return

    def detach_annotator(self, annotator_name: str) -> None:
        """Detach an annotator from the Lidar sensor.

        Args:
            annotator_name: Name of the annotator to detach.

        Example:

        .. code-block:: python

            >>> from isaacsim.sensors.rtx import LidarRtx
            >>> lidar = LidarRtx(prim_path="/World/Lidar")
            >>> lidar.attach_annotator("IsaacComputeRTXLidarFlatScan")
            >>> lidar.detach_annotator("IsaacComputeRTXLidarFlatScan")
        """
        if annotator_name in self._annotators:
            annotator = self._annotators.pop(annotator_name)
            annotator.detach()
        else:
            carb.log_warn(f"Annotator {annotator_name} not attached to {self._render_product_path}")
        return

    def detach_all_annotators(self) -> None:
        """Detach all annotators from the Lidar sensor.

        Example:

        .. code-block:: python

            >>> from isaacsim.sensors.rtx import LidarRtx
            >>> lidar = LidarRtx(prim_path="/World/Lidar")
            >>> lidar.attach_annotator("IsaacComputeRTXLidarFlatScan")
            >>> lidar.attach_annotator("IsaacCreateRTXLidarScanBuffer")
            >>> lidar.detach_all_annotators()
        """
        for annotator in self._annotators.values():
            annotator.detach()
        self._annotators.clear()
        return

    def get_writers(self) -> dict:
        """Get all attached writers.

        Returns:
            Dictionary mapping writer names to their instances.

        Example:

        .. code-block:: python

            >>> from isaacsim.sensors.rtx import LidarRtx
            >>> lidar = LidarRtx(prim_path="/World/Lidar")
            >>> lidar.attach_writer("RtxLidarDebugDrawPointCloud")
            >>> writers = lidar.get_writers()
            >>> print(list(writers.keys()))
        """
        return self._writers

    def attach_writer(self, writer_name: str, **kwargs: object) -> None:
        """Attach a writer to the Lidar sensor.

        Args:
            writer_name: Name of the writer to attach.
            **kwargs: Additional arguments to pass to the writer on initialization.

        Example:

        .. code-block:: python

            >>> from isaacsim.sensors.rtx import LidarRtx
            >>> lidar = LidarRtx(prim_path="/World/Lidar")
            >>> lidar.attach_writer("RtxLidarDebugDrawPointCloud")
        """
        if writer_name in self._writers:
            carb.log_warn(f"Writer {writer_name} already attached to {self._render_product_path}")
            return
        writer = rep.WriterRegistry.get(writer_name)
        writer.initialize(**kwargs)
        writer.attach([self._render_product_path])
        self._writers[writer_name] = writer

    def detach_writer(self, writer_name: str) -> None:
        """Detach a writer from the Lidar sensor.

        Args:
            writer_name: Name of the writer to detach.

        Example:

        .. code-block:: python

            >>> from isaacsim.sensors.rtx import LidarRtx
            >>> lidar = LidarRtx(prim_path="/World/Lidar")
            >>> lidar.attach_writer("RtxLidarDebugDrawPointCloud")
            >>> lidar.detach_writer("RtxLidarDebugDrawPointCloud")
        """
        if writer_name in self._writers:
            writer = self._writers.pop(writer_name)
            writer.detach()
        else:
            carb.log_warn(f"Writer {writer_name} not attached to {self._render_product_path}")
        return

    def detach_all_writers(self) -> None:
        """Detach all writers from the Lidar sensor.

        Example:

        .. code-block:: python

            >>> from isaacsim.sensors.rtx import LidarRtx
            >>> lidar = LidarRtx(prim_path="/World/Lidar")
            >>> lidar.attach_writer("RtxLidarDebugDrawPointCloud")
            >>> lidar.detach_all_writers()
        """
        for writer in self._writers.values():
            writer.detach()
        self._writers.clear()
        return

    def _create_point_cloud_graph_node(self) -> None:
        """Create a point cloud graph node for the Lidar sensor.

        This method is deprecated as of Isaac Sim 5.0. Use attach_annotator('IsaacExtractRTXSensorPointCloudNoAccumulator') instead.
        """
        carb.log_warn(
            "LidarRtx._create_point_cloud_graph_node is deprecated as of Isaac Sim 5.0 and will be removed in a future release. Use attach_annotator instead."
        )
        self.attach_annotator("IsaacExtractRTXSensorPointCloudNoAccumulator")

    def _create_flat_scan_graph_node(self) -> None:
        """Create a flat scan graph node for the Lidar sensor.

        This method is deprecated as of Isaac Sim 5.0. Use attach_annotator('IsaacComputeRTXLidarFlatScan') instead.
        """
        carb.log_warn(
            "LidarRtx._create_flat_scan_graph_node is deprecated as of Isaac Sim 5.0 and will be removed in a future release. Use attach_annotator instead."
        )
        self.attach_annotator("IsaacComputeRTXLidarFlatScan")  # type: ignore[arg-type]

    def initialize(self, physics_sim_view: Any = None) -> None:
        """Initialize the Lidar sensor.

        Args:
            physics_sim_view: Optional physics simulation view.
        """
        BaseSensor.initialize(self, physics_sim_view=physics_sim_view)
        self._acquisition_callback = carb.eventdispatcher.get_eventdispatcher().observe_event(
            event_name=omni.kit.app.GLOBAL_EVENT_UPDATE,
            on_event=self._data_acquisition_callback,
            observer_name="isaacsim.sensors.rtx.LidarRtx.initialize._data_acquisition_callback",
        )
        self._stage_open_callback = carb.eventdispatcher.get_eventdispatcher().observe_event(
            event_name=omni.usd.get_context().stage_event_name(omni.usd.StageEventType.OPENED),
            on_event=self._stage_open_callback_fn,
            observer_name="isaacsim.sensors.rtx.LidarRtx.initialize._stage_open_callback",
        )
        self._timer_reset_callback_pause = carb.eventdispatcher.get_eventdispatcher().observe_event(
            event_name=omni.timeline.GLOBAL_EVENT_PAUSE,
            on_event=self._timeline_pause_callback_fn,
            observer_name="isaacsim.sensors.rtx.LidarRtx.initialize._timeline_pause_callback",
        )
        self._timer_reset_callback_stop = carb.eventdispatcher.get_eventdispatcher().observe_event(
            event_name=omni.timeline.GLOBAL_EVENT_STOP,
            on_event=self._timeline_stop_callback_fn,
            observer_name="isaacsim.sensors.rtx.LidarRtx.initialize._timeline_stop_callback",
        )
        self._timer_reset_callback_play = carb.eventdispatcher.get_eventdispatcher().observe_event(
            event_name=omni.timeline.GLOBAL_EVENT_PLAY,
            on_event=self._timeline_play_callback_fn,
            observer_name="isaacsim.sensors.rtx.LidarRtx.initialize._timeline_play_callback",
        )

    def _stage_open_callback_fn(self, event: carb.eventdispatcher.Event) -> None:
        """Handle stage open event by cleaning up callbacks.

        Args:
            event: The stage open event.
        """
        self._acquisition_callback = None
        self._stage_open_callback = None
        self._timer_reset_callback_pause = None
        self._timer_reset_callback_stop = None
        self._timer_reset_callback_play = None

    def _timeline_pause_callback_fn(self, event: carb.eventdispatcher.Event) -> None:
        """Handle timeline pause event.

        Args:
            event: The timeline pause event.
        """
        self.pause()

    def _timeline_stop_callback_fn(self, event: carb.eventdispatcher.Event) -> None:
        """Handle timeline stop event.

        Args:
            event: The timeline stop event.
        """
        self.pause()

    def _timeline_play_callback_fn(self, event: carb.eventdispatcher.Event) -> None:
        """Handle timeline play event.

        Args:
            event: The timeline play event.
        """
        self.resume()

    def post_reset(self) -> None:
        """Perform post-reset operations for the Lidar sensor."""
        BaseSensor.post_reset(self)
        return

    def resume(self) -> None:
        """Resume data acquisition for the Lidar sensor.

        Example:

        .. code-block:: python

            >>> from isaacsim.sensors.rtx import LidarRtx
            >>> lidar = LidarRtx(prim_path="/World/Lidar")
            >>> lidar.initialize()
            >>> lidar.pause()
            >>> lidar.resume()
        """
        if self._acquisition_callback is None:
            self._acquisition_callback = carb.eventdispatcher.get_eventdispatcher().observe_event(
                event_name=omni.kit.app.GLOBAL_EVENT_UPDATE,
                on_event=self._data_acquisition_callback,
                observer_name="isaacsim.sensors.rtx.LidarRtx.resume._data_acquisition_callback",
            )
        return

    def pause(self) -> None:
        """Pause data acquisition for the Lidar sensor.

        Example:

        .. code-block:: python

            >>> from isaacsim.sensors.rtx import LidarRtx
            >>> lidar = LidarRtx(prim_path="/World/Lidar")
            >>> lidar.initialize()
            >>> lidar.pause()
        """
        self._acquisition_callback = None
        return

    def is_paused(self) -> bool:
        """Check if the Lidar sensor is paused.

        Returns:
            True if the sensor is paused, False otherwise.

        Example:

        .. code-block:: python

            >>> from isaacsim.sensors.rtx import LidarRtx
            >>> lidar = LidarRtx(prim_path="/World/Lidar")
            >>> lidar.initialize()
            >>> lidar.pause()
            >>> is_paused = lidar.is_paused()
            >>> print(is_paused)
            True
        """
        return self._acquisition_callback is None

    def _data_acquisition_callback(self, event: carb.events.IEvent) -> None:
        """Handle data acquisition callback for the Lidar sensor.

        Args:
            event: The event that triggered the callback.
        """
        if self._annotators or self._writers:
            self._current_frame["rendering_frame"] = (
                og.Controller()
                .node("/Render/PostProcess/SDGPipeline/PostProcessDispatcher")
                .get_attribute("outputs:referenceTimeNumerator")
                .get(),
                og.Controller()
                .node("/Render/PostProcess/SDGPipeline/PostProcessDispatcher")
                .get_attribute("outputs:referenceTimeDenominator")
                .get(),
            )

            if self._current_frame["rendering_frame"][1] == 0:
                carb.log_warn(
                    f"Reference time is {self._current_frame['rendering_frame'][0]}/{self._current_frame['rendering_frame'][1]}, cannot get simulation time on this frame."
                )
            else:
                self._current_frame["rendering_time"] = self._simulation_manager_interface.get_simulation_time_at_time(
                    self._current_frame["rendering_frame"]
                )

            for annotator_name, annotator in self._annotators.items():
                self._current_frame[annotator_name] = annotator.get_data()

            if "IsaacComputeRTXLidarFlatScan" in self._annotators:
                flat_scan_data = self._annotators["IsaacComputeRTXLidarFlatScan"].get_data()
                self._current_frame["linear_depth_data"] = flat_scan_data["linearDepthData"]
                self._current_frame["intensities_data"] = flat_scan_data["intensitiesData"]
                self._current_frame["azimuth_range"] = flat_scan_data["azimuthRange"]
                self._current_frame["horizontal_resolution"] = flat_scan_data["horizontalResolution"]

    def get_horizontal_resolution(self) -> float | None:
        """Get the horizontal resolution of the Lidar sensor.

        This method is deprecated as of Isaac Sim 5.0. Use the horizontal_resolution attribute
        in the current frame instead.

        Returns:
            The horizontal resolution value if available, None otherwise.
        """
        carb.log_warn(
            "LidarRtx.get_horizontal_resolution is deprecated as of Isaac Sim 5.0 and will be removed in a future release. Use the horizontal_resolution attribute in the current frame instead."
        )
        if "IsaacComputeRTXLidarFlatScan" in self._annotators:
            return self._current_frame["IsaacComputeRTXLidarFlatScan"].get("horizontalResolution")
        return None

    def get_horizontal_fov(self) -> float | None:
        """Get the horizontal field of view of the Lidar sensor.

        This method is deprecated as of Isaac Sim 5.0. Use the horizontal_fov attribute
        in the current frame instead.

        Returns:
            The horizontal field of view value if available, None otherwise.
        """
        carb.log_warn(
            "LidarRtx.get_horizontal_fov is deprecated as of Isaac Sim 5.0 and will be removed in a future release. Use the horizontal_fov attribute in the current frame instead."
        )
        if "IsaacComputeRTXLidarFlatScan" in self._annotators:
            return self._current_frame["IsaacComputeRTXLidarFlatScan"].get("horizontalFov")
        return None

    def get_num_rows(self) -> int | None:
        """Get the number of rows in the Lidar scan.

        This method is deprecated as of Isaac Sim 5.0. Use the num_rows attribute
        in the current frame instead.

        Returns:
            The number of rows if available, None otherwise.
        """
        carb.log_warn(
            "LidarRtx.get_num_rows is deprecated as of Isaac Sim 5.0 and will be removed in a future release. Use the num_rows attribute in the current frame instead."
        )
        if "IsaacComputeRTXLidarFlatScan" in self._annotators:
            return self._current_frame["IsaacComputeRTXLidarFlatScan"].get("numRows")
        return None

    def get_num_cols(self) -> int | None:
        """Get the number of columns in the Lidar scan.

        This method is deprecated as of Isaac Sim 5.0. Use the num_cols attribute
        in the current frame instead.

        Returns:
            The number of columns if available, None otherwise.
        """
        carb.log_warn(
            "LidarRtx.get_num_cols is deprecated as of Isaac Sim 5.0 and will be removed in a future release. Use the num_cols attribute in the current frame instead."
        )
        if "IsaacComputeRTXLidarFlatScan" in self._annotators:
            return self._current_frame["IsaacComputeRTXLidarFlatScan"].get("numCols")
        return None

    def get_rotation_frequency(self) -> float | None:
        """Get the rotation frequency of the Lidar sensor.

        This method is deprecated as of Isaac Sim 5.0. Use the rotation_frequency attribute
        in the current frame instead.

        Returns:
            The rotation frequency value if available, None otherwise.
        """
        carb.log_warn(
            "LidarRtx.get_rotation_frequency is deprecated as of Isaac Sim 5.0 and will be removed in a future release. Use the rotation_frequency attribute in the current frame instead."
        )
        if "IsaacComputeRTXLidarFlatScan" in self._annotators:
            return self._current_frame["IsaacComputeRTXLidarFlatScan"].get("rotationRate")
        return None

    def get_depth_range(self) -> tuple[float, float] | None:
        """Get the depth range of the Lidar sensor.

        This method is deprecated as of Isaac Sim 5.0. Use the depth_range attribute
        in the current frame instead.

        Returns:
            Tuple of (min_depth, max_depth) if available, None otherwise.
        """
        carb.log_warn(
            "LidarRtx.get_depth_range is deprecated as of Isaac Sim 5.0 and will be removed in a future release. Use the depth_range attribute in the current frame instead."
        )
        if "IsaacComputeRTXLidarFlatScan" in self._annotators:
            return self._current_frame["IsaacComputeRTXLidarFlatScan"].get("depthRange")
        return None

    def get_azimuth_range(self) -> tuple[float, float] | None:
        """Get the azimuth range of the Lidar sensor.

        This method is deprecated as of Isaac Sim 5.0. Use the azimuth_range attribute
        in the current frame instead.

        Returns:
            Tuple of (min_azimuth, max_azimuth) if available, None otherwise.
        """
        carb.log_warn(
            "LidarRtx.get_azimuth_range is deprecated as of Isaac Sim 5.0 and will be removed in a future release. Use the azimuth_range attribute in the current frame instead."
        )
        if "IsaacComputeRTXLidarFlatScan" in self._annotators:
            return self._current_frame["IsaacComputeRTXLidarFlatScan"].get("azimuthRange")
        return None

    def enable_visualization(self) -> None:
        """Enable visualization of the Lidar point cloud data.

        Example:

        .. code-block:: python

            >>> from isaacsim.sensors.rtx import LidarRtx
            >>> lidar = LidarRtx(prim_path="/World/Lidar")
            >>> lidar.enable_visualization()
        """
        self.attach_writer("RtxLidar" + "DebugDrawPointCloud")
        return

    def disable_visualization(self) -> None:
        """Disable visualization of the Lidar point cloud data.

        Example:

        .. code-block:: python

            >>> from isaacsim.sensors.rtx import LidarRtx
            >>> lidar = LidarRtx(prim_path="/World/Lidar")
            >>> lidar.enable_visualization()
            >>> lidar.disable_visualization()
        """
        self.detach_writer("RtxLidar" + "DebugDrawPointCloud")
        return

    def add_point_cloud_data_to_frame(self) -> None:
        """Add point cloud data to the current frame.

        This method is deprecated as of Isaac Sim 5.0. Use attach_annotator('IsaacComputeRTXLidarFlatScan') instead.
        """
        carb.log_warn(
            "add_point_cloud_data_to_frame is deprecated as of Isaac Sim 5.0 and will be removed in a future release. This attribute is now automatically added to the current frame if the corresponding annotator is attached."
        )
        carb.log_warn("Use attach_annotator('IsaacComputeRTXLidarFlatScan') instead.")

    def add_linear_depth_data_to_frame(self) -> None:
        """Add linear depth data to the current frame.

        This method is deprecated as of Isaac Sim 5.0. Use attach_annotator('IsaacComputeRTXLidarFlatScan') instead.
        """
        carb.log_warn(
            "add_linear_depth_data_to_frame is deprecated as of Isaac Sim 5.0 and will be removed in a future release. This attribute is now automatically added to the current frame if the corresponding annotator is attached."
        )
        carb.log_warn("Use attach_annotator('IsaacComputeRTXLidarFlatScan') instead.")

    def add_intensities_data_to_frame(self) -> None:
        """Add intensities data to the current frame.

        This method is deprecated as of Isaac Sim 5.0. Use attach_annotator('IsaacComputeRTXLidarFlatScan') instead.
        """
        carb.log_warn(
            "add_intensities_data_to_frame is deprecated as of Isaac Sim 5.0 and will be removed in a future release. This attribute is now automatically added to the current frame if the corresponding annotator is attached."
        )
        carb.log_warn("Use attach_annotator('IsaacComputeRTXLidarFlatScan') instead.")

    def add_azimuth_range_to_frame(self) -> None:
        """Add azimuth range data to the current frame.

        This method is deprecated as of Isaac Sim 5.0. Use attach_annotator('IsaacComputeRTXLidarFlatScan') instead.
        """
        carb.log_warn(
            "add_azimuth_range_to_frame is deprecated as of Isaac Sim 5.0 and will be removed in a future release. This attribute is now automatically added to the current frame if the corresponding annotator is attached."
        )
        carb.log_warn("Use attach_annotator('IsaacComputeRTXLidarFlatScan') instead.")

    def add_horizontal_resolution_to_frame(self) -> None:
        """Add horizontal resolution data to the current frame.

        This method is deprecated as of Isaac Sim 5.0. Use attach_annotator('IsaacComputeRTXLidarFlatScan') instead.
        """
        carb.log_warn(
            "add_horizontal_resolution_to_frame is deprecated as of Isaac Sim 5.0 and will be removed in a future release. This attribute is now automatically added to the current frame if the corresponding annotator is attached."
        )
        carb.log_warn("Use attach_annotator('IsaacComputeRTXLidarFlatScan') instead.")

    def add_range_data_to_frame(self) -> None:
        """Add range data to the current frame.

        This method is deprecated as of Isaac Sim 5.0 and will be removed in a future release.
        """
        carb.log_warn(
            "add_range_data_to_frame is deprecated as of Isaac Sim 5.0 and will be removed in a future release."
        )

    def add_azimuth_data_to_frame(self) -> None:
        """Add azimuth data to the current frame.

        This method is deprecated as of Isaac Sim 5.0 and will be removed in a future release.
        """
        carb.log_warn(
            "add_azimuth_data_to_frame is deprecated as of Isaac Sim 5.0 and will be removed in a future release."
        )

    def add_elevation_data_to_frame(self) -> None:
        """Add elevation data to the current frame.

        This method is deprecated as of Isaac Sim 5.0 and will be removed in a future release.
        """
        carb.log_warn(
            "add_elevation_data_to_frame is deprecated as of Isaac Sim 5.0 and will be removed in a future release."
        )

    def remove_point_cloud_data_to_frame(self) -> None:
        """Remove point cloud data from the current frame.

        This method is deprecated as of Isaac Sim 5.0. Use detach_annotator('IsaacComputeRTXLidarFlatScan') instead.
        """
        carb.log_warn(
            "remove_point_cloud_data_to_frame is deprecated as of Isaac Sim 5.0 and will be removed in a future release. This attribute is now automatically removed from the current frame if the corresponding annotator is detached."
        )
        carb.log_warn("Use detach_annotator('IsaacComputeRTXLidarFlatScan') instead.")

    def remove_linear_depth_data_to_frame(self) -> None:
        """Remove linear depth data from the current frame.

        This method is deprecated as of Isaac Sim 5.0. Use detach_annotator('IsaacComputeRTXLidarFlatScan') instead.
        """
        carb.log_warn(
            "remove_linear_depth_data_to_frame is deprecated as of Isaac Sim 5.0 and will be removed in a future release. This attribute is now automatically removed from the current frame if the corresponding annotator is detached."
        )
        carb.log_warn("Use detach_annotator('IsaacComputeRTXLidarFlatScan') instead.")

    def remove_intensities_data_to_frame(self) -> None:
        """Remove intensities data from the current frame.

        This method is deprecated as of Isaac Sim 5.0. Use detach_annotator('IsaacComputeRTXLidarFlatScan') instead.
        """
        carb.log_warn(
            "remove_intensities_data_to_frame is deprecated as of Isaac Sim 5.0 and will be removed in a future release. This attribute is now automatically removed from the current frame if the corresponding annotator is detached."
        )
        carb.log_warn("Use detach_annotator('IsaacComputeRTXLidarFlatScan') instead.")

    def remove_azimuth_range_to_frame(self) -> None:
        """Remove azimuth range data from the current frame.

        This method is deprecated as of Isaac Sim 5.0. Use detach_annotator('IsaacComputeRTXLidarFlatScan') instead.
        """
        carb.log_warn(
            "remove_azimuth_range_to_frame is deprecated as of Isaac Sim 5.0 and will be removed in a future release. This attribute is now automatically removed from the current frame if the corresponding annotator is detached."
        )
        carb.log_warn("Use detach_annotator('IsaacComputeRTXLidarFlatScan') instead.")

    def remove_horizontal_resolution_to_frame(self) -> None:
        """Remove horizontal resolution data from the current frame.

        This method is deprecated as of Isaac Sim 5.0. Use detach_annotator('IsaacComputeRTXLidarFlatScan') instead.
        """
        carb.log_warn(
            "remove_horizontal_resolution_to_frame is deprecated as of Isaac Sim 5.0 and will be removed in a future release. This attribute is now automatically removed from the current frame if the corresponding annotator is detached."
        )
        carb.log_warn("Use detach_annotator('IsaacComputeRTXLidarFlatScan') instead.")

    def remove_range_data_to_frame(self) -> None:
        """Remove range data from the current frame.

        This method is deprecated as of Isaac Sim 5.0 and will be removed in a future release.
        """
        carb.log_warn(
            "remove_range_data_to_frame is deprecated as of Isaac Sim 5.0 and will be removed in a future release."
        )

    def remove_azimuth_data_to_frame(self) -> None:
        """Remove azimuth data from the current frame.

        This method is deprecated as of Isaac Sim 5.0 and will be removed in a future release.
        """
        carb.log_warn(
            "remove_azimuth_data_to_frame is deprecated as of Isaac Sim 5.0 and will be removed in a future release."
        )

    def remove_elevation_data_to_frame(self) -> None:
        """Remove elevation data from the current frame.

        This method is deprecated as of Isaac Sim 5.0 and will be removed in a future release.
        """
        carb.log_warn(
            "remove_elevation_data_to_frame is deprecated as of Isaac Sim 5.0 and will be removed in a future release."
        )

    @staticmethod
    def decode_stable_id_mapping(stable_id_mapping_raw: bytes) -> dict:
        """Decode the StableIdMap buffer into a dictionary of stable IDs to labels.

        The buffer is a sequence of 6-byte entries, each containing:
        - 4 bytes for the stable ID (uint32)
        - 1 byte for the label length (uint8)
        - 1 byte for the label offset (uint8)
        The label is a UTF-8 string of the specified length, starting at the offset.

        Args:
            stable_id_mapping_raw: The raw StableIdMap buffer bytes.

        Returns:
            Dictionary mapping stable IDs to their label strings.

        Example:

        .. code-block:: python

            >>> from isaacsim.sensors.rtx import LidarRtx
            >>> lidar = LidarRtx(prim_path="/World/Lidar")
            >>> lidar.attach_annotator("StableIdMap")
            >>> lidar.initialize()
            >>> # After simulation steps...
            >>> frame = lidar.get_current_frame()
            >>> stable_id_data = frame.get("StableIdMap")
            >>> if stable_id_data is not None:
            ...     mapping = LidarRtx.decode_stable_id_mapping(stable_id_data)
        """
        num_entries = int.from_bytes(stable_id_mapping_raw[-4:], byteorder="little")
        output_data_type = np.dtype([("stable_id", "<u4", (4)), ("label_length", "<u4"), ("label_offset", "<u4")])
        entry_data_length = num_entries * output_data_type.itemsize
        entries = np.frombuffer(stable_id_mapping_raw[:entry_data_length], "<u4").reshape(-1, 6)

        mapping = {}
        for entry in entries:
            entry_id = int.from_bytes(entry[:4].tobytes(), byteorder="little")
            mapping[entry_id] = stable_id_mapping_raw[entry[5] : entry[5] + entry[4]].decode("utf8").rstrip()
        return mapping

    @staticmethod
    def get_object_ids(obj_ids: np.ndarray) -> list[int]:
        """Get Object IDs from the GenericModelOutput object ID buffer.

        The buffer is an array that must be converted to a list of dtype uint128
        (stride 16 bytes). Each uint128 is a unique stable ID for a prim in the
        scene, which can be used to look up the prim path in the map provided by
        the StableIdMap annotator (see above).

        Args:
            obj_ids: The object ID buffer. Can be either a uint8 array with stride 16
                (from GenericModelOutput) or a uint32 array with stride 4
                (from IsaacCreateRTXLidarScanBuffer).

        Returns:
            The object IDs as a list of uint128.

        Raises:
            ValueError: If obj_ids has an unsupported dtype.

        Example:

        .. code-block:: python

            >>> import numpy as np
            >>> from isaacsim.sensors.rtx import LidarRtx
            >>> lidar = LidarRtx(prim_path="/World/Lidar")
            >>> lidar.attach_annotator("IsaacCreateRTXLidarScanBuffer")
            >>> lidar.initialize()
            >>> # After simulation steps...
            >>> frame = lidar.get_current_frame()
            >>> scan_buffer = frame.get("IsaacCreateRTXLidarScanBuffer")
            >>> if scan_buffer is not None:
            ...     object_ids = LidarRtx.get_object_ids(scan_buffer["objectId"])
        """
        obj_ids = np.ascontiguousarray(obj_ids)

        # Determine the reshape size based on dtype to ensure 16-byte (128-bit) object IDs
        if obj_ids.dtype == np.uint8:
            # uint8: 16 elements = 16 bytes = 128 bits
            obj_ids = obj_ids.reshape(-1, 16)
        elif obj_ids.dtype == np.uint32:
            # uint32: 4 elements = 16 bytes = 128 bits
            obj_ids = obj_ids.reshape(-1, 4)
        elif obj_ids.dtype == np.uint64:
            # uint64: 2 elements = 64 bytes = 128 bits
            obj_ids = obj_ids.reshape(-1, 2)
        else:
            raise ValueError(f"Unsupported dtype for object IDs: {obj_ids.dtype}. Expected uint8, uint32, or uint64.")

        return [int.from_bytes(group.tobytes(), byteorder="little") for group in obj_ids]
