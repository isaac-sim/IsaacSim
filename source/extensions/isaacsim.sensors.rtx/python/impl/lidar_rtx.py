# SPDX-FileCopyrightText: Copyright (c) 2023-2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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

from typing import Literal, Optional, Tuple

import carb
import numpy as np
import omni
import omni.graph.core as og
import omni.replicator.core as rep
from isaacsim.core.api.sensors.base_sensor import BaseSensor
from isaacsim.core.nodes.bindings import _isaacsim_core_nodes
from isaacsim.core.utils.prims import get_prim_at_path, get_prim_type_name, is_prim_path_valid
from pxr import Gf


class LidarRtx(BaseSensor):
    """RTX-based Lidar sensor implementation.

    This class provides functionality for creating and managing RTX-based Lidar sensors in Isaac Sim.
    It supports various annotators and writers for data collection and visualization.

    The sensor can be configured with different parameters and supports both point cloud and flat scan data collection.
    """

    @staticmethod
    def make_add_remove_deprecated_attr(deprecated_attr: str):
        """Creates deprecated add/remove attribute methods.

        Args:
            param deprecated_attr (str): Name of the deprecated attribute to create methods for.

        Returns:
            list: List of method functions for adding and removing the deprecated attribute.
        """
        methods = []
        for fun_name in [f"add_{deprecated_attr}_to_frame", f"remove_{deprecated_attr}_from_frame"]:

            def attr_fun(self):
                carb.log_warn(
                    f"{fun_name} is deprecated as of Isaac Sim 5.0 and will be removed in a future release. This attribute is now automatically added to the current frame if the corresponding annotator is attached."
                )

            methods.append(attr_fun)
        return methods

    def __init__(
        self,
        prim_path: str,
        name: str = "lidar_rtx",
        position: Optional[np.ndarray] = None,
        translation: Optional[np.ndarray] = None,
        orientation: Optional[np.ndarray] = np.array([1.0, 0.0, 0.0, 0.0]),
        config_file_name: Optional[str] = None,
        **kwargs,
    ) -> None:
        """Initialize the RTX Lidar sensor.

        Args:
            param prim_path (str): Path to the USD prim for the Lidar sensor.
            param name (str): Name of the Lidar sensor.
            param position (Optional[np.ndarray]): Global position of the sensor as [x, y, z].
            param translation (Optional[np.ndarray]): Local translation of the sensor as [x, y, z].
            param orientation (Optional[np.ndarray]): Orientation quaternion as [w, x, y, z].
            param config_file_name (Optional[str]): Path to the configuration file for the sensor.
            param **kwargs: Additional keyword arguments for sensor configuration.

        Raises:
            Exception: If the prim at prim_path is not an OmniLidar or doesn't have the required API.
        """
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
        self._annotators = {}  # maps annotator name to annotator and node prim path
        self._writers = {}  # maps writer name to writer
        self._render_product = None
        self._render_product_path = None

        if position is None and translation is None:
            position = np.array([0.0, 0.0, 0.0])
        if is_prim_path_valid(prim_path):
            if get_prim_type_name(prim_path) == "Camera":
                carb.log_warn(
                    "Support for creating LidarRtx from Camera prims is deprecated as of Isaac Sim 5.0 and will be removed in a future release. Use OmniLidar prim instead."
                )
            elif get_prim_type_name(prim_path) != "OmniLidar":
                raise Exception(f"Prim at {prim_path} is not an OmniLidar.")
            elif not get_prim_at_path(prim_path).HasAPI("OmniSensorGenericLidarCoreAPI"):
                raise Exception(f"Prim at {prim_path} does not have the OmniSensorGenericLidarCoreAPI schema.")
            carb.log_warn("Using existing RTX Lidar prim at path {}".format(prim_path))
        else:
            p = position if translation is None else translation
            _, sensor = omni.kit.commands.execute(
                "IsaacSensorCreateRtxLidar",
                translation=Gf.Vec3d(p[0], p[1], p[2]),
                orientation=Gf.Quatd(orientation[0], orientation[1], orientation[2], orientation[3]),
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

        # Initialize core nodes interface
        self._core_nodes_interface = _isaacsim_core_nodes.acquire_interface()
        if position is not None and orientation is not None:
            self.set_world_pose(position=position, orientation=orientation)
        elif translation is not None and orientation is not None:
            self.set_local_pose(translation=translation, orientation=orientation)
        elif orientation is not None:
            self.set_local_pose(orientation=orientation)

        # Define data dictionary for current frame
        self._current_frame = dict()
        self._current_frame["rendering_time"] = 0
        self._current_frame["rendering_frame"] = 0

        return

    def __del__(self):
        self.detach_all_writers()
        self.detach_all_annotators()
        if self._render_product:
            self._render_product.destroy()

    def get_render_product_path(self) -> str:
        """Get the path to the render product used by the Lidar.

        Returns:
            str: Path to the render product.
        """
        return self._render_product_path

    def get_current_frame(self) -> dict:
        """Get the current frame data from the Lidar sensor.

        Returns:
            dict: Dictionary containing the current frame data including rendering time,
                frame number, and any attached annotator data.
        """
        return self._current_frame

    def get_annotators(self) -> dict:
        """Get all attached annotators.

        Returns:
            dict: Dictionary mapping annotator names to their instances.
        """
        return self._annotators

    def attach_annotator(
        self,
        annotator_name: Literal[
            "IsaacComputeRTXLidarFlatScan",
            "IsaacExtractRTXSensorPointCloudNoAccumulator",
            "IsaacCreateRTXLidarScanBuffer",
        ],
    ) -> None:
        """Attach an annotator to the Lidar sensor.

        Args:
            param annotator_name (Literal): Name of the annotator to attach. Must be one of:
                - "IsaacComputeRTXLidarFlatScan"
                - "IsaacExtractRTXSensorPointCloudNoAccumulator"
                - "IsaacCreateRTXLidarScanBuffer"
        """
        if annotator_name in self._annotators:
            carb.log_warn(f"Annotator {annotator_name} already attached to {self._render_product_path}")
            return

        annotator = rep.AnnotatorRegistry.get_annotator(annotator_name)
        annotator.attach([self._render_product_path])
        self._annotators[annotator_name] = annotator
        return

    def detach_annotator(self, annotator_name: str) -> None:
        """Detach an annotator from the Lidar sensor.

        Args:
            param annotator_name (str): Name of the annotator to detach.
        """
        if annotator_name in self._annotators:
            annotator = self._annotators.pop(annotator_name)
            annotator.detach()
        else:
            carb.log_warn(f"Annotator {annotator_name} not attached to {self._render_product_path}")
        return

    def detach_all_annotators(self) -> None:
        """Detach all annotators from the Lidar sensor."""
        for annotator in self._annotators.values():
            annotator.detach()
        self._annotators.clear()
        return

    def get_writers(self) -> dict:
        """Get all attached writers.

        Returns:
            dict: Dictionary mapping writer names to their instances.
        """
        return self._writers

    def attach_writer(self, writer_name: str) -> None:
        """Attach a writer to the Lidar sensor.

        Args:
            param writer_name (str): Name of the writer to attach.
        """
        if writer_name in self._writers:
            carb.log_warn(f"Writer {writer_name} already attached to {self._render_product_path}")
            return
        writer = rep.WriterRegistry.get(writer_name)
        writer.attach([self._render_product_path])
        self._writers[writer_name] = writer

    def detach_writer(self, writer_name: str) -> None:
        """Detach a writer from the Lidar sensor.

        Args:
            param writer_name (str): Name of the writer to detach.
        """
        if writer_name in self._writers:
            writer = self._writers.pop(writer_name)
            writer.detach()
        else:
            carb.log_warn(f"Writer {writer_name} not attached to {self._render_product_path}")
        return

    def detach_all_writers(self) -> None:
        """Detach all writers from the Lidar sensor."""
        for writer in self._writers.values():
            writer.detach()
        self._writers.clear()
        return

    def _create_point_cloud_graph_node(self):
        """Create a point cloud graph node for the Lidar sensor.

        This method is deprecated as of Isaac Sim 5.0. Use attach_annotator('IsaacExtractRTXSensorPointCloud') instead.
        """
        carb.log_warn(
            "LidarRtx._create_point_cloud_graph_node is deprecated as of Isaac Sim 5.0 and will be removed in a future release. Use attach_annotator instead."
        )
        self.attach_annotator("IsaacExtractRTXSensorPointCloud")
        return

    def _create_flat_scan_graph_node(self):
        """Create a flat scan graph node for the Lidar sensor.

        This method is deprecated as of Isaac Sim 5.0. Use attach_annotator('IsaacComputeRTXLidarFlatScanSimulationTime') instead.
        """
        carb.log_warn(
            "LidarRtx._create_flat_scan_graph_node is deprecated as of Isaac Sim 5.0 and will be removed in a future release. Use attach_annotator instead."
        )
        self.attach_annotator("IsaacComputeRTXLidarFlatScan" + "SimulationTime")
        return

    def initialize(self, physics_sim_view=None) -> None:
        """Initialize the Lidar sensor.

        Args:
            param physics_sim_view (Optional): Optional physics simulation view.
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
        timeline = omni.timeline.get_timeline_interface()
        self._timer_reset_callback = timeline.get_timeline_event_stream().create_subscription_to_pop(
            self._timeline_timer_callback_fn
        )
        return

    def _stage_open_callback_fn(self, event):
        """Handle stage open event by cleaning up callbacks.

        Args:
            param event (carb.events.IEvent): The stage open event.
        """
        self._acquisition_callback = None
        self._stage_open_callback = None
        self._timer_reset_callback = None
        return

    def _timeline_timer_callback_fn(self, event):
        """Handle timeline timer events for pausing and resuming the sensor.

        Args:
            param event (carb.events.IEvent): The timeline event.
        """
        if event.type == int(omni.timeline.TimelineEventType.PAUSE) or event.type == int(
            omni.timeline.TimelineEventType.STOP
        ):
            self.pause()
        elif event.type == int(omni.timeline.TimelineEventType.PLAY):
            self.resume()
        return

    def post_reset(self) -> None:
        """Perform post-reset operations for the Lidar sensor."""
        BaseSensor.post_reset(self)
        return

    def resume(self) -> None:
        """Resume data acquisition for the Lidar sensor."""
        if self._acquisition_callback is None:
            self._acquisition_callback = carb.eventdispatcher.get_eventdispatcher().observe_event(
                event_name=omni.kit.app.GLOBAL_EVENT_UPDATE,
                on_event=self._data_acquisition_callback,
                observer_name="isaacsim.sensors.rtx.LidarRtx.resume._data_acquisition_callback",
            )
        return

    def pause(self) -> None:
        """Pause data acquisition for the Lidar sensor."""
        self._acquisition_callback = None
        return

    def is_paused(self) -> bool:
        """Check if the Lidar sensor is paused.

        Returns:
            True if the sensor is paused, False otherwise.
        """
        return self._acquisition_callback is None

    def _data_acquisition_callback(self, event: carb.events.IEvent):
        """Handle data acquisition callback for the Lidar sensor.

        Args:
            param event (carb.events.IEvent): The event that triggered the callback.
        """
        if not self._annotators and not self._writers:
            return

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

        self._current_frame["rendering_time"] = self._core_nodes_interface.get_sim_time_at_time(
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
        return

    def get_horizontal_resolution(self) -> float:
        """Get the horizontal resolution of the Lidar sensor.

        This method is deprecated as of Isaac Sim 5.0. Use the horizontal_resolution attribute in the current frame instead.

        Returns:
            Optional[float]: The horizontal resolution value if available, None otherwise.
        """
        carb.log_warn(
            "LidarRtx.get_horizontal_resolution is deprecated as of Isaac Sim 5.0 and will be removed in a future release. Use the horizontal_resolution attribute in the current frame instead."
        )
        if "IsaacComputeRTXLidarFlatScan" in self._annotators:
            return self._current_frame["IsaacComputeRTXLidarFlatScan"].get("horizontalResolution")
        return None

    def get_horizontal_fov(self) -> float:
        """Get the horizontal field of view of the Lidar sensor.

        This method is deprecated as of Isaac Sim 5.0. Use the horizontal_fov attribute in the current frame instead.

        Returns:
            Optional[float]: The horizontal field of view value if available, None otherwise.
        """
        carb.log_warn(
            "LidarRtx.get_horizontal_fov is deprecated as of Isaac Sim 5.0 and will be removed in a future release. Use the horizontal_fov attribute in the current frame instead."
        )
        if "IsaacComputeRTXLidarFlatScan" in self._annotators:
            return self._current_frame["IsaacComputeRTXLidarFlatScan"].get("horizontalFov")
        return None

    def get_num_rows(self) -> int:
        """Get the number of rows in the Lidar scan.

        This method is deprecated as of Isaac Sim 5.0. Use the num_rows attribute in the current frame instead.

        Returns:
            Optional[int]: The number of rows if available, None otherwise.
        """
        carb.log_warn(
            "LidarRtx.get_num_rows is deprecated as of Isaac Sim 5.0 and will be removed in a future release. Use the num_rows attribute in the current frame instead."
        )
        if "IsaacComputeRTXLidarFlatScan" in self._annotators:
            return self._current_frame["IsaacComputeRTXLidarFlatScan"].get("numRows")
        return None

    def get_num_cols(self) -> int:
        """Get the number of columns in the Lidar scan.

        This method is deprecated as of Isaac Sim 5.0. Use the num_cols attribute in the current frame instead.

        Returns:
            Optional[int]: The number of columns if available, None otherwise.
        """
        carb.log_warn(
            "LidarRtx.get_num_cols is deprecated as of Isaac Sim 5.0 and will be removed in a future release. Use the num_cols attribute in the current frame instead."
        )
        if "IsaacComputeRTXLidarFlatScan" in self._annotators:
            return self._current_frame["IsaacComputeRTXLidarFlatScan"].get("numCols")
        return None

    def get_rotation_frequency(self) -> float:
        """Get the rotation frequency of the Lidar sensor.

        This method is deprecated as of Isaac Sim 5.0. Use the rotation_frequency attribute in the current frame instead.

        Returns:
            Optional[float]: The rotation frequency value if available, None otherwise.
        """
        carb.log_warn(
            "LidarRtx.get_rotation_frequency is deprecated as of Isaac Sim 5.0 and will be removed in a future release. Use the rotation_frequency attribute in the current frame instead."
        )
        if "IsaacComputeRTXLidarFlatScan" in self._annotators:
            return self._current_frame["IsaacComputeRTXLidarFlatScan"].get("rotationRate")
        return None

    def get_depth_range(self) -> Tuple[float, float]:
        """Get the depth range of the Lidar sensor.

        This method is deprecated as of Isaac Sim 5.0. Use the depth_range attribute in the current frame instead.

        Returns:
            Optional[Tuple[float, float]]: Tuple of (min_depth, max_depth) if available, None otherwise.
        """
        carb.log_warn(
            "LidarRtx.get_depth_range is deprecated as of Isaac Sim 5.0 and will be removed in a future release. Use the depth_range attribute in the current frame instead."
        )
        if "IsaacComputeRTXLidarFlatScan" in self._annotators:
            return self._current_frame["IsaacComputeRTXLidarFlatScan"].get("depthRange")
        return None

    def get_azimuth_range(self) -> Tuple[float, float]:
        """Get the azimuth range of the Lidar sensor.

        This method is deprecated as of Isaac Sim 5.0. Use the azimuth_range attribute in the current frame instead.

        Returns:
            Optional[Tuple[float, float]]: Tuple of (min_azimuth, max_azimuth) if available, None otherwise.
        """
        carb.log_warn(
            "LidarRtx.get_azimuth_range is deprecated as of Isaac Sim 5.0 and will be removed in a future release. Use the azimuth_range attribute in the current frame instead."
        )
        if "IsaacComputeRTXLidarFlatScan" in self._annotators:
            return self._current_frame["IsaacComputeRTXLidarFlatScan"].get("azimuthRange")
        return None

    def enable_visualization(self):
        """Enable visualization of the Lidar point cloud data."""
        self.attach_writer("RtxLidar" + "DebugDrawPointCloud")
        return

    def disable_visualization(self):
        """Disable visualization of the Lidar point cloud data."""
        self.detach_writer("RtxLidar" + "DebugDrawPointCloud")
        return

    def add_point_cloud_data_to_frame(self):
        """Add point cloud data to the current frame.

        This method is deprecated as of Isaac Sim 5.0. Use attach_annotator('IsaacComputeRTXLidarFlatScanSimulationTime') instead.
        """
        carb.log_warn(
            "add_point_cloud_data_to_frame is deprecated as of Isaac Sim 5.0 and will be removed in a future release. This attribute is now automatically added to the current frame if the corresponding annotator is attached."
        )
        carb.log_warn("Use attach_annotator('IsaacComputeRTXLidarFlatScanSimulationTime') instead.")
        return

    def add_linear_depth_data_to_frame(self):
        """Add linear depth data to the current frame.

        This method is deprecated as of Isaac Sim 5.0. Use attach_annotator('IsaacComputeRTXLidarFlatScanSimulationTime') instead.
        """
        carb.log_warn(
            "add_linear_depth_data_to_frame is deprecated as of Isaac Sim 5.0 and will be removed in a future release. This attribute is now automatically added to the current frame if the corresponding annotator is attached."
        )
        carb.log_warn("Use attach_annotator('IsaacComputeRTXLidarFlatScanSimulationTime') instead.")
        return

    def add_intensities_data_to_frame(self):
        """Add intensities data to the current frame.

        This method is deprecated as of Isaac Sim 5.0. Use attach_annotator('IsaacComputeRTXLidarFlatScanSimulationTime') instead.
        """
        carb.log_warn(
            "add_intensities_data_to_frame is deprecated as of Isaac Sim 5.0 and will be removed in a future release. This attribute is now automatically added to the current frame if the corresponding annotator is attached."
        )
        carb.log_warn("Use attach_annotator('IsaacComputeRTXLidarFlatScanSimulationTime') instead.")
        return

    def add_azimuth_range_to_frame(self):
        """Add azimuth range data to the current frame.

        This method is deprecated as of Isaac Sim 5.0. Use attach_annotator('IsaacComputeRTXLidarFlatScanSimulationTime') instead.
        """
        carb.log_warn(
            "add_azimuth_range_to_frame is deprecated as of Isaac Sim 5.0 and will be removed in a future release. This attribute is now automatically added to the current frame if the corresponding annotator is attached."
        )
        carb.log_warn("Use attach_annotator('IsaacComputeRTXLidarFlatScanSimulationTime') instead.")
        return

    def add_horizontal_resolution_to_frame(self):
        """Add horizontal resolution data to the current frame.

        This method is deprecated as of Isaac Sim 5.0. Use attach_annotator('IsaacComputeRTXLidarFlatScanSimulationTime') instead.
        """
        carb.log_warn(
            "add_horizontal_resolution_to_frame is deprecated as of Isaac Sim 5.0 and will be removed in a future release. This attribute is now automatically added to the current frame if the corresponding annotator is attached."
        )
        carb.log_warn("Use attach_annotator('IsaacComputeRTXLidarFlatScanSimulationTime') instead.")
        return

    def add_range_data_to_frame(self):
        """Add range data to the current frame.

        This method is deprecated as of Isaac Sim 5.0 and will be removed in a future release.
        """
        carb.log_warn(
            "add_range_data_to_frame is deprecated as of Isaac Sim 5.0 and will be removed in a future release."
        )
        return

    def add_azimuth_data_to_frame(self):
        """Add azimuth data to the current frame.

        This method is deprecated as of Isaac Sim 5.0 and will be removed in a future release.
        """
        carb.log_warn(
            "add_azimuth_data_to_frame is deprecated as of Isaac Sim 5.0 and will be removed in a future release."
        )
        return

    def add_elevation_data_to_frame(self):
        """Add elevation data to the current frame.

        This method is deprecated as of Isaac Sim 5.0 and will be removed in a future release.
        """
        carb.log_warn(
            "add_elevation_data_to_frame is deprecated as of Isaac Sim 5.0 and will be removed in a future release."
        )
        return

    def remove_point_cloud_data_to_frame(self):
        """Remove point cloud data from the current frame.

        This method is deprecated as of Isaac Sim 5.0. Use detach_annotator('IsaacComputeRTXLidarFlatScanSimulationTime') instead.
        """
        carb.log_warn(
            "remove_point_cloud_data_to_frame is deprecated as of Isaac Sim 5.0 and will be removed in a future release. This attribute is now automatically removed from the current frame if the corresponding annotator is detached."
        )
        carb.log_warn("Use detach_annotator('IsaacComputeRTXLidarFlatScanSimulationTime') instead.")
        return

    def remove_linear_depth_data_to_frame(self):
        """Remove linear depth data from the current frame.

        This method is deprecated as of Isaac Sim 5.0. Use detach_annotator('IsaacComputeRTXLidarFlatScanSimulationTime') instead.
        """
        carb.log_warn(
            "remove_linear_depth_data_to_frame is deprecated as of Isaac Sim 5.0 and will be removed in a future release. This attribute is now automatically removed from the current frame if the corresponding annotator is detached."
        )
        carb.log_warn("Use detach_annotator('IsaacComputeRTXLidarFlatScanSimulationTime') instead.")
        return

    def remove_intensities_data_to_frame(self):
        """Remove intensities data from the current frame.

        This method is deprecated as of Isaac Sim 5.0. Use detach_annotator('IsaacComputeRTXLidarFlatScanSimulationTime') instead.
        """
        carb.log_warn(
            "remove_intensities_data_to_frame is deprecated as of Isaac Sim 5.0 and will be removed in a future release. This attribute is now automatically removed from the current frame if the corresponding annotator is detached."
        )
        carb.log_warn("Use detach_annotator('IsaacComputeRTXLidarFlatScanSimulationTime') instead.")
        return

    def remove_azimuth_range_to_frame(self):
        """Remove azimuth range data from the current frame.

        This method is deprecated as of Isaac Sim 5.0. Use detach_annotator('IsaacComputeRTXLidarFlatScanSimulationTime') instead.
        """
        carb.log_warn(
            "remove_azimuth_range_to_frame is deprecated as of Isaac Sim 5.0 and will be removed in a future release. This attribute is now automatically removed from the current frame if the corresponding annotator is detached."
        )
        carb.log_warn("Use detach_annotator('IsaacComputeRTXLidarFlatScanSimulationTime') instead.")
        return

    def remove_horizontal_resolution_to_frame(self):
        """Remove horizontal resolution data from the current frame.

        This method is deprecated as of Isaac Sim 5.0. Use detach_annotator('IsaacComputeRTXLidarFlatScanSimulationTime') instead.
        """
        carb.log_warn(
            "remove_horizontal_resolution_to_frame is deprecated as of Isaac Sim 5.0 and will be removed in a future release. This attribute is now automatically removed from the current frame if the corresponding annotator is detached."
        )
        carb.log_warn("Use detach_annotator('IsaacComputeRTXLidarFlatScanSimulationTime') instead.")
        return

    def remove_range_data_to_frame(self):
        """Remove range data from the current frame.

        This method is deprecated as of Isaac Sim 5.0 and will be removed in a future release.
        """
        carb.log_warn(
            "remove_range_data_to_frame is deprecated as of Isaac Sim 5.0 and will be removed in a future release."
        )
        return

    def remove_azimuth_data_to_frame(self):
        """Remove azimuth data from the current frame.

        This method is deprecated as of Isaac Sim 5.0 and will be removed in a future release.
        """
        carb.log_warn(
            "remove_azimuth_data_to_frame is deprecated as of Isaac Sim 5.0 and will be removed in a future release."
        )
        return

    def remove_elevation_data_to_frame(self):
        """Remove elevation data from the current frame.

        This method is deprecated as of Isaac Sim 5.0 and will be removed in a future release.
        """
        carb.log_warn(
            "remove_elevation_data_to_frame is deprecated as of Isaac Sim 5.0 and will be removed in a future release."
        )
        return
