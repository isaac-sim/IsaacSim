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

"""Provides high level functions to deal with camera prims and their attributes/properties in Isaac Sim."""

from __future__ import annotations

import copy
import math
from collections.abc import Callable, Sequence

import carb
import carb.eventdispatcher
import isaacsim.core.utils.numpy as np_utils
import isaacsim.core.utils.torch as torch_utils
import isaacsim.core.utils.warp as warp_utils
import numpy as np
import omni
import omni.graph.core as og
import omni.kit.app
import omni.replicator.core as rep
import omni.syntheticdata._syntheticdata as _syntheticdata
import omni.timeline
import warp as wp
from isaacsim.core.api.sensors.base_sensor import BaseSensor
from isaacsim.core.deprecation_manager import import_module
from isaacsim.core.simulation_manager import SimulationManager
from isaacsim.core.utils.carb import get_carb_setting
from isaacsim.core.utils.prims import (
    define_prim,
    get_all_matching_child_prims,
    get_prim_at_path,
    get_prim_path,
    get_prim_type_name,
    is_prim_path_valid,
)
from isaacsim.core.utils.render_product import set_camera_prim_path, set_resolution
from omni.isaac.IsaacSensorSchema import IsaacRtxLidarSensorAPI
from pxr import Gf, Sdf, Usd, UsdGeom

torch = import_module("torch")

# Attribute maps for lens distortion models
OPENCV_PINHOLE_ATTRIBUTE_MAP = ["k1", "k2", "p1", "p2", "k3", "k4", "k5", "k6", "s1", "s2", "s3", "s4"]
OPENCV_FISHEYE_ATTRIBUTE_MAP = ["k1", "k2", "k3", "k4"]
FTHETA_ATTRIBUTE_MAP = ["k0", "k1", "k2", "k3", "k4"]
KANNALA_BRANDT_K3_ATTRIBUTE_MAP = ["k0", "k1", "k2", "k3"]
RAD_TAN_THIN_PRISM_ATTRIBUTE_MAP = ["k0", "k1", "k2", "k3", "k4", "k5", "p0", "p1", "s0", "s1", "s2", "s3"]

# transforms are read from right to left
# U_R_TRANSFORM means transformation matrix from R frame to U frame
# R indicates the ROS camera convention (computer vision community)
# U indicates the USD camera convention (computer graphics community)
# W indicates the World camera convention (robotics community)

# from ROS camera convention to USD camera convention
U_R_TRANSFORM = np.array([[1, 0, 0, 0], [0, -1, 0, 0], [0, 0, -1, 0], [0, 0, 0, 1]])

# from USD camera convention to ROS camera convention
R_U_TRANSFORM = np.array([[1, 0, 0, 0], [0, -1, 0, 0], [0, 0, -1, 0], [0, 0, 0, 1]])

# from USD camera convention to World camera convention
W_U_TRANSFORM = np.array([[0, 0, -1, 0], [-1, 0, 0, 0], [0, 1, 0, 0], [0, 0, 0, 1]])

# from World camera convention to USD camera convention
U_W_TRANSFORM = np.array([[0, -1, 0, 0], [0, 0, 1, 0], [-1, 0, 0, 0], [0, 0, 0, 1]])

# USD camera attributes are stored in tenths of a unit, this constant converts them to stage units
USD_CAMERA_TENTHS_TO_STAGE_UNIT = 10.0


def point_to_theta(camera_matrix: object, x: object, y: object) -> float:
    """This helper function returns the theta angle of the point.

    Args:
        camera_matrix: Camera calibration matrix.
        x: X coordinates of the points.
        y: Y coordinates of the points.

    Returns:
        The theta angle of the point.

    """
    (fx, _, cx), (_, fy, cy), (_, _, _) = camera_matrix
    pt_x, pt_y, _pt_z = (x - cx) / fx, (y - cy) / fy, 1.0
    r2 = pt_x * pt_x + pt_y * pt_y
    theta = np.arctan2(np.sqrt(r2), 1.0)
    return theta


def distort_point_rational_polynomial(
    camera_matrix: object, distortion_model: object, x: object, y: object
) -> np.ndarray:
    """[DEPRECATED] This helper function distorts point(s) using rational polynomial model.

    It should be equivalent to the following reference that uses OpenCV:

    .. code-block:: python

        def distort_point_rational_polynomial(x, y)
            import cv2
            ((fx,_,cx),(_,fy,cy),(_,_,_)) = camera_matrix
            pt_x, pt_y, pt_z  = (x-cx)/fx, (y-cy)/fy, np.full(x.shape, 1.0)
            points3d = np.stack((pt_x, pt_y, pt_z), axis = -1)
            rvecs, tvecs = np.array([0.0,0.0,0.0]), np.array([0.0,0.0,0.0])
            cameraMatrix, distCoeffs = np.array(camera_matrix), np.array(distortion_coefficients)
            points, jac = cv2.projectPoints(points3d, rvecs, tvecs, cameraMatrix, distCoeffs)
            return np.array([points[:,0,0], points[:,0,1]])

    Args:
        camera_matrix: Camera calibration matrix.
        distortion_model: Rational polynomial distortion model coefficients.
        x: X coordinates of the points.
        y: Y coordinates of the points.

    Returns:
        The distorted points.

    """
    omni.kit.app.log_deprecation(
        "distort_point_rational_polynomial is deprecated."
        'Please use the the "opencvFisheye" distortion model to directly specify OpenCV distortion parameters.'
    )
    (fx, _, cx), (_, fy, cy), (_, _, _) = camera_matrix
    K, P = list(distortion_model[:2]) + list(distortion_model[4:]), list(distortion_model[2:4])
    pt_x, pt_y = (x - cx) / fx, (y - cy) / fy
    r2 = pt_x * pt_x + pt_y * pt_y
    r_x = (
        pt_x
        * (
            (1 + K[0] * r2 + K[1] * r2 * r2 + K[2] * r2 * r2 * r2)
            / (1 + K[3] * r2 + K[4] * r2 * r2 + K[5] * r2 * r2 * r2)
        )
        + 2 * P[0] * pt_x * pt_y
        + P[1] * (r2 + 2 * pt_x * pt_x)
    )

    r_y = (
        pt_y
        * (
            (1 + K[0] * r2 + K[1] * r2 * r2 + K[2] * r2 * r2 * r2)
            / (1 + K[3] * r2 + K[4] * r2 * r2 + K[5] * r2 * r2 * r2)
        )
        + 2 * P[1] * pt_x * pt_y
        + P[0] * (r2 + 2 * pt_y * pt_y)
    )
    return np.array([fx * r_x + cx, fy * r_y + cy])


def distort_point_kannala_brandt(camera_matrix: object, distortion_model: object, x: object, y: object) -> np.ndarray:
    """[DEPRECATED] This helper function distorts point(s) using Kannala Brandt fisheye model.

    It should be equivalent to the following reference that uses OpenCV:

    .. code-block:: python

        def distort_point_kannala_brandt2(camera_matrix, distortion_model, x, y):
            import cv2
            ((fx,_,cx),(_,fy,cy),(_,_,_)) = camera_matrix
            pt_x, pt_y, pt_z  = (x-cx)/fx, (y-cy)/fy, np.full(x.shape, 1.0)
            points3d = np.stack((pt_x, pt_y, pt_z), axis = -1)
            rvecs, tvecs = np.array([0.0,0.0,0.0]), np.array([0.0,0.0,0.0])
            cameraMatrix, distCoeffs = np.array(camera_matrix), np.array(distortion_model)
            points, jac = cv2.fisheye.projectPoints(np.expand_dims(points3d, 1), rvecs, tvecs, cameraMatrix, distCoeffs)
            return np.array([points[:,0,0], points[:,0,1]])

    Args:
        camera_matrix: Camera calibration matrix.
        distortion_model: Kannala Brandt distortion model coefficients.
        x: X coordinates of the points.
        y: Y coordinates of the points.

    Returns:
        The distorted points.

    """
    omni.kit.app.log_deprecation(
        "distort_point_rational_polynomial is deprecated."
        'Please use the the "opencvFisheye" distortion model to directly specify OpenCV distortion parameters.'
    )
    (fx, _, cx), (_, fy, cy), (_, _, _) = camera_matrix
    pt_x, pt_y, _pt_z = (x - cx) / fx, (y - cy) / fy, 1.0
    r2 = pt_x * pt_x + pt_y * pt_y
    r = np.sqrt(r2)
    theta = np.arctan2(r, 1.0)

    t3 = theta * theta * theta
    t5 = t3 * theta * theta
    t7 = t5 * theta * theta
    t9 = t7 * theta * theta
    k1, k2, k3, k4 = list(distortion_model[:4])
    theta_d = theta + k1 * t3 + k2 * t5 + k3 * t7 + k4 * t9

    inv_r = 1.0 / r  # if r > 1e-8 else 1.0
    cdist = theta_d * inv_r  # if r > 1e-8 else 1.0

    r_x, r_y = pt_x * cdist, pt_y * cdist
    return np.array([fx * r_x + cx, fy * r_y + cy])


class Camera(BaseSensor):
    """Provides high level functions to deal with a camera prim and its attributes/ properties.

    If there is a camera prim present at the path, it will use it. Otherwise, a new Camera prim at
    the specified prim path will be created.

    Args:
        prim_path: prim path of the Camera Prim to encapsulate or create.
        name: shortname to be used as a key by Scene class.
            Note: needs to be unique if the object is added to the Scene.
        frequency: Frequency of the sensor (i.e: how often is the data frame updated).
        dt: dt of the sensor (i.e: period at which a the data frame updated).
        resolution: resolution of the camera (width, height).
        position: position in the world frame of the prim. shape is (3, ).
        orientation: quaternion orientation in the world/ local frame of the prim
            (depends if translation or position is specified).
            quaternion is scalar-first (w, x, y, z). shape is (4, ).
        translation: translation in the local frame of the prim
            (with respect to its parent prim). shape is (3, ).
        render_product_path: path to an existing render product, will be used instead of creating a new render product
            the resolution and camera attached to this render product will be set based on the input arguments.
            Note: Using same render product path on two Camera objects with different camera prims, resolutions is not supported
        annotator_device: Device to place tensors on for annotators.

    """

    def __init__(
        self,
        prim_path: str,
        name: str = "camera",
        frequency: int | None = None,
        dt: float | None = None,
        resolution: tuple[int, int] | None = None,
        position: np.ndarray | None = None,
        orientation: np.ndarray | None = None,
        translation: np.ndarray | None = None,
        render_product_path: str = None,
        annotator_device: str = None,
    ) -> None:
        self._frequency = -1
        self._render_product = None
        if frequency is not None and dt is not None:
            raise Exception("Frequency and dt can't be both specified.")
        if dt is not None:
            # Calculate frequency properly without truncating
            frequency = round(1.0 / dt)

        if frequency is not None:
            self.set_frequency(frequency)
        else:
            current_rendering_frequency = get_carb_setting(
                carb.settings.get_settings(), "/app/runLoops/main/rateLimitFrequency"
            )
            if current_rendering_frequency is not None:
                self.set_frequency(current_rendering_frequency)

        if resolution is None:
            resolution = (128, 128)
        if is_prim_path_valid(prim_path):
            self._prim = get_prim_at_path(prim_path)
            if get_prim_type_name(prim_path) != "Camera":
                raise Exception("prim path does not correspond to a Camera prim.")
        else:
            # create a camera prim
            carb.log_info(f"Creating a new Camera prim at path {prim_path}")
            self._prim = UsdGeom.Camera(define_prim(prim_path=prim_path, prim_type="Camera"))
            if orientation is None:
                orientation = [1, 0, 0, 0]
        self._render_product_path = render_product_path
        self._resolution = resolution
        self._render_product = None
        self._is_initialized = False
        self._annotator_device = annotator_device
        self._supported_annotators = [
            "normals",
            "motion_vectors",
            "occlusion",
            "distance_to_image_plane",
            "distance_to_camera",
            "bounding_box_2d_tight",
            "bounding_box_2d_loose",
            "bounding_box_3d",
            "semantic_segmentation",
            "instance_id_segmentation",
            "instance_segmentation",
            "pointcloud",
        ]

        self._custom_annotators = {}
        BaseSensor.__init__(
            self, prim_path=prim_path, name=name, position=position, translation=translation, orientation=orientation
        )
        if position is not None and orientation is not None:
            self.set_world_pose(position=position, orientation=orientation)
        elif translation is not None and orientation is not None:
            self.set_local_pose(translation=translation, orientation=orientation)
        elif orientation is not None:
            self.set_local_pose(orientation=orientation)
        self._current_frame = {}
        self._pause = False
        self._current_frame["rendering_time"] = 0
        self._current_frame["rendering_frame"] = 0
        self._sdg_interface = _syntheticdata.acquire_syntheticdata_interface()

        self._elapsed_time = 0
        self._previous_time = None
        self._og_controller = og.Controller()
        self._sdg_graph_pipeline = self._og_controller.graph("/Render/PostProcess/SDGPipeline")
        self._fabric_time_annotator = None

        # Make sure the camera aperture is set to use square pixels even if initialize() is not called
        self._maintain_square_pixel_aperture(mode="horizontal")

        return

    def __del__(self) -> None:
        """Destructor that calls destroy() to clean up resources."""
        self.destroy()

    def destroy(self) -> None:
        """Destroy the camera by detaching all annotators and destroying the internal render product, then explicitly clearing all event subscriptions."""
        custom_annotators = list(self._custom_annotators.keys())
        for annotator_name in custom_annotators:
            self.detach_annotator(annotator_name)
        if self._fabric_time_annotator is not None:
            self._fabric_time_annotator.detach([self._render_product_path])
            self._fabric_time_annotator = None
        if self._render_product is not None:
            self._render_product.destroy()
            self._render_product = None
        self._render_product_path = None
        self._is_initialized = False

        self._acquisition_callback = None
        self._stage_open_callback = None
        self._timer_reset_callback_stop = None
        self._timer_reset_callback_play = None

    @property
    def supported_annotators(self) -> list[str]:
        """List of annotators supported by the camera.

        Returns:
            List of annotator names that can be attached to this camera.

        """
        return self._supported_annotators

    def get_render_product_path(self) -> str:
        """Gets the path to the render product attached to this camera.

        Returns:
            Path to the render product attached to this camera.

        """
        return self._render_product_path

    def set_frequency(self, value: int) -> None:
        """Sets the frequency to acquire new data frames.

        If ``/app/runLoops/main/rateLimitFrequency`` is unset (common in headless launches or
        custom Kit apps without rate-limit configuration), the requested frequency cannot be
        honored and the camera falls back to processing every rendered frame; a warning is
        logged so the discrepancy is observable at the call site.

        Args:
            value: The frequency to acquire new data frames.

        Raises:
            Exception: If ``value`` is not a divisor of the configured rendering frequency.

        Returns:
            None.

        """
        current_rendering_frequency = get_carb_setting(
            carb.settings.get_settings(), "/app/runLoops/main/rateLimitFrequency"
        )
        if current_rendering_frequency is None:
            carb.log_warn(
                f"Camera.set_frequency({value}): rendering frequency unknown "
                f"(/app/runLoops/main/rateLimitFrequency is None). Camera will process all "
                f"frames. Configure the rendering rate before calling set_frequency() to "
                f"apply the requested value."
            )
            self._frequency = -1
        else:
            if current_rendering_frequency % value != 0:
                raise Exception("frequency of the camera sensor needs to be a divisor of the rendering frequency.")
            self._frequency = value
        return

    def get_frequency(self) -> float:
        """Gets the frequency to acquire new data frames.

        Returns:
            The frequency to acquire new data frames.

        """
        return self._frequency

    def set_dt(self, value: float) -> None:
        """Sets the dt to acquire new data frames.

        If ``/app/runLoops/main/rateLimitFrequency`` is unset (common in headless launches or
        custom Kit apps without rate-limit configuration), the requested dt cannot be honored
        and the camera falls back to processing every rendered frame; a warning is logged so
        the discrepancy is observable at the call site.

        Args:
            value: The dt to acquire new data frames.

        Raises:
            Exception: If ``value`` is not a multiple of the configured rendering dt.

        Returns:
            None.

        """
        current_rendering_frequency = get_carb_setting(
            carb.settings.get_settings(), "/app/runLoops/main/rateLimitFrequency"
        )
        if current_rendering_frequency is None:
            carb.log_warn(
                f"Camera.set_dt({value}): rendering frequency unknown "
                f"(/app/runLoops/main/rateLimitFrequency is None). Camera will process all "
                f"frames. Configure the rendering rate before calling set_dt() to apply the "
                f"requested value."
            )
            self._frequency = -1
        else:
            freq_as_int = round(1.0 / value)
            if current_rendering_frequency % freq_as_int != 0:
                raise Exception("dt of the camera sensor needs to be a multiple of the rendering dt.")
            self._frequency = freq_as_int
        return

    def get_dt(self) -> float:
        """Gets the dt to acquire new data frames.

        Returns:
            The dt to acquire new data frames.

        """
        if self._frequency < 0:
            return 0.0
        return 1.0 / self._frequency

    def get_current_frame(self, clone: bool = False) -> dict:
        """Gets the current frame of data.

        Args:
            clone: If True, returns a deepcopy of the current frame.

        Returns:
            The current frame of data.

        """
        if clone:
            return copy.deepcopy(self._current_frame)
        else:
            return self._current_frame

    def initialize(self, physics_sim_view: object = None, attach_rgb_annotator: bool = True) -> None:
        """To be called before using this class after a reset of the world.

        Args:
            physics_sim_view: Current physics simulation view.
            attach_rgb_annotator: True to attach the rgb annotator to the camera. Set to False to improve performance.

        Returns:
            None.

        """
        BaseSensor.initialize(self, physics_sim_view=physics_sim_view)
        if self._render_product_path:

            self.set_resolution(self._resolution)
            set_camera_prim_path(self._render_product_path, self.prim_path)
        else:
            self._render_product = rep.create.render_product(self.prim_path, resolution=self._resolution)
            self._render_product_path = self._render_product.path
        self._is_initialized = True
        if attach_rgb_annotator:
            self.attach_annotator(annotator_name="rgb")
        self._fabric_time_annotator = rep.AnnotatorRegistry.get_annotator("ReferenceTime")
        self._fabric_time_annotator.attach([self._render_product_path])

        # Must initialize here after an annotator has been attached so the graph exists
        self._sdg_graph_pipeline = self._og_controller.graph("/Render/PostProcess/SDGPipeline")

        self._acquisition_callback = carb.eventdispatcher.get_eventdispatcher().observe_event(
            event_name=omni.usd.get_context().stage_rendering_event_name(
                omni.usd.StageRenderingEventType.NEW_FRAME, True
            ),
            on_event=self._data_acquisition_callback,
            observer_name="isaacsim.sensors.camera.Camera.resume._data_acquisition_callback",
        )
        self._stage_open_callback = carb.eventdispatcher.get_eventdispatcher().observe_event(
            event_name=omni.usd.get_context().stage_event_name(omni.usd.StageEventType.OPENED),
            on_event=self._stage_open_callback_fn,
            observer_name="isaacsim.sensors.camera.Camera.initialize._stage_open_callback",
        )
        self._timer_reset_callback_stop = carb.eventdispatcher.get_eventdispatcher().observe_event(
            event_name=omni.timeline.GLOBAL_EVENT_STOP,
            on_event=self._timeline_stop_callback_fn,
            observer_name="isaacsim.sensors.camera.Camera.initialize._timeline_stop_callback",
        )
        self._timer_reset_callback_play = carb.eventdispatcher.get_eventdispatcher().observe_event(
            event_name=omni.timeline.GLOBAL_EVENT_PLAY,
            on_event=self._timeline_play_callback_fn,
            observer_name="isaacsim.sensors.camera.Camera.initialize._timeline_play_callback",
        )
        self._current_frame["rendering_frame"] = 0
        self._current_frame["rendering_time"] = 0
        return

    def post_reset(self) -> None:
        """Reset camera's elapsed time and previous time after simulation reset.

        Resets internal timing state used for data collection frequency control.

        Returns:
            None.

        """
        BaseSensor.post_reset(self)
        self._elapsed_time = 0
        self._previous_time = None
        return

    def _stage_open_callback_fn(self, event: object) -> None:
        """Handle stage open events by clearing all event callbacks.

        Args:
            event: The stage open event.

        Returns:
            None.

        """
        self._acquisition_callback = None
        self._stage_open_callback = None
        self._timer_reset_callback_stop = None
        self._timer_reset_callback_play = None
        return

    def _timeline_stop_callback_fn(self, event: object) -> None:
        """Handle timeline stop events by pausing the camera.

        Args:
            event: The timeline stop event.

        Returns:
            None.

        """
        self.pause()
        return

    def _timeline_play_callback_fn(self, event: object) -> None:
        """Handle timeline play events by resuming the camera.

        Args:
            event: The timeline play event.

        Returns:
            None.

        """
        self.resume()
        return

    def resume(self) -> None:
        """Resumes data collection and updating the data frame.

        Returns:
            None.

        """
        self._acquisition_callback = carb.eventdispatcher.get_eventdispatcher().observe_event(
            event_name=omni.usd.get_context().stage_rendering_event_name(
                omni.usd.StageRenderingEventType.NEW_FRAME, True
            ),
            on_event=self._data_acquisition_callback,
            observer_name="isaacsim.sensors.camera.Camera.resume._data_acquisition_callback",
        )
        return

    def pause(self) -> None:
        """Pauses data collection and updating the data frame.

        Returns:
            None.

        """
        self._acquisition_callback = None
        return

    def is_paused(self) -> bool:
        """Data collection pause status.

        Returns:
            Whether data collection is paused.

        """
        return self._acquisition_callback is None

    def _data_acquisition_callback(self, event: carb.eventdispatcher.Event) -> None:
        """Process rendering events and update current frame data when frequency conditions are met.

        Args:
            event: The rendering event containing product path handle and results.

        Returns:
            None.

        """
        parsed_payload = self._sdg_interface.parse_rendered_simulation_event(
            event["product_path_handle"], event["results"]
        )
        if parsed_payload[0] == self._render_product_path:
            self._og_controller.evaluate_sync(graph_id=self._sdg_graph_pipeline)
            frame_number = self._fabric_time_annotator.get_data()
            current_time = SimulationManager._simulation_manager_interface.get_simulation_time_at_time(
                (frame_number["referenceTimeNumerator"], frame_number["referenceTimeDenominator"])
            )
            if self._previous_time is not None:
                self._elapsed_time += current_time - self._previous_time
            if self._frequency < 0 or self._elapsed_time >= self.get_dt():
                self._elapsed_time = 0
                self._current_frame["rendering_frame"] = frame_number
                self._current_frame["rendering_time"] = current_time
                for key in self._current_frame:
                    if key not in ["rendering_time", "rendering_frame"]:
                        self._current_frame[key] = self._custom_annotators[key].get_data()
            self._previous_time = current_time
        return

    def _maintain_square_pixel_aperture(self, mode: str = "horizontal") -> None:
        """Ensure apertures are in sync with aspect ratio to maintain square pixels.

        Args:
            mode: 'horizontal' to use horizontal aperture as source, 'vertical' to use vertical as source.

        """
        aspect_ratio = self.get_aspect_ratio()
        horizontal_aperture = self.prim.GetAttribute("horizontalAperture").Get() / USD_CAMERA_TENTHS_TO_STAGE_UNIT
        vertical_aperture = self.prim.GetAttribute("verticalAperture").Get() / USD_CAMERA_TENTHS_TO_STAGE_UNIT

        # Default to 'horizontal' if mode is None or not 'vertical'
        if mode != "vertical":
            expected_vertical_aperture = horizontal_aperture / aspect_ratio
            if not np.isclose(vertical_aperture, expected_vertical_aperture, rtol=1e-5, atol=1e-8):
                carb.log_warn(
                    f"'verticalAperture' ({vertical_aperture}) and 'horizontalAperture' ({horizontal_aperture}) are inconsistent with the pixel resolution aspect ratio ({aspect_ratio}). Setting 'verticalAperture' to {expected_vertical_aperture} to ensure square pixels."
                )
                self.prim.GetAttribute("verticalAperture").Set(
                    expected_vertical_aperture * USD_CAMERA_TENTHS_TO_STAGE_UNIT
                )
        else:
            expected_horizontal_aperture = vertical_aperture * aspect_ratio
            if not np.isclose(horizontal_aperture, expected_horizontal_aperture, rtol=1e-5, atol=1e-8):
                carb.log_warn(
                    f"'horizontalAperture' ({horizontal_aperture}) and 'verticalAperture' ({vertical_aperture}) are inconsistent with the pixel resolution aspect ratio ({aspect_ratio}). Setting 'horizontalAperture' to {expected_horizontal_aperture} to ensure square pixels."
                )
                self.prim.GetAttribute("horizontalAperture").Set(
                    expected_horizontal_aperture * USD_CAMERA_TENTHS_TO_STAGE_UNIT
                )

    def set_resolution(self, value: tuple[int, int], maintain_square_pixels: bool = True) -> None:
        """Set the resolution of the camera sensor. This will check and update the apertures to maintain square pixels.

        Args:
            value: width and height respectively.
            maintain_square_pixels: If True, keep apertures in sync for square pixels.

        Returns:
            None.

        """
        self._resolution = value
        set_resolution(self._render_product_path, self._resolution)
        if maintain_square_pixels:
            self._maintain_square_pixel_aperture(mode="horizontal")
        return

    def get_resolution(self) -> tuple[int, int]:
        """Camera resolution in pixels.

        Returns:
            Width and height respectively.

        """
        return self._resolution

    def get_aspect_ratio(self) -> float:
        """Camera aspect ratio.

        Returns:
            Ratio between width and height.

        """
        width, height = self.get_resolution()
        return width / float(height)

    def get_world_pose(self, camera_axes: str = "world") -> tuple[np.ndarray, np.ndarray]:
        """Gets prim's pose with respect to the world's frame (always at [0, 0, 0] and unity quaternion not to be confused with /World Prim).

        Args:
            camera_axes: camera axes, world is (+Z up, +X forward), ros is (+Y up, +Z forward) and usd is (+Y up and -Z forward).

        Returns:
            first index is position in the world frame of the prim. shape is (3, ).
            second index is quaternion orientation in the world frame of the prim.
            quaternion is scalar-first (w, x, y, z). shape is (4, ).

        """
        if camera_axes not in ["world", "ros", "usd"]:
            raise Exception(
                f"camera axes passed {camera_axes} is not supported: accepted values are ["
                "world"
                ", "
                "ros"
                ", "
                "usd"
                "] only"
            )
        position, orientation = BaseSensor.get_world_pose(self)
        if camera_axes == "world":
            world_w_cam_u_R = self._backend_utils.quats_to_rot_matrices(orientation)
            u_w_R = self._backend_utils.create_tensor_from_list(
                U_W_TRANSFORM[:3, :3].tolist(), dtype="float32", device=self._device
            )
            orientation = self._backend_utils.rot_matrices_to_quats(self._backend_utils.matmul(world_w_cam_u_R, u_w_R))
        elif camera_axes == "ros":
            world_w_cam_u_R = self._backend_utils.quats_to_rot_matrices(orientation)
            u_r_R = self._backend_utils.create_tensor_from_list(
                U_R_TRANSFORM[:3, :3].tolist(), dtype="float32", device=self._device
            )
            orientation = self._backend_utils.rot_matrices_to_quats(self._backend_utils.matmul(world_w_cam_u_R, u_r_R))
        return position, orientation

    def set_world_pose(
        self,
        position: Sequence[float] | None = None,
        orientation: Sequence[float] | None = None,
        camera_axes: str = "world",
    ) -> None:
        """Sets prim's pose with respect to the world's frame (always at [0, 0, 0] and unity quaternion not to be confused with /World Prim).

        Args:
            position: position in the world frame of the prim. shape is (3, ).
            orientation: quaternion orientation in the world frame of the prim.
                quaternion is scalar-first (w, x, y, z). shape is (4, ).
            camera_axes: camera axes, world is (+Z up, +X forward), ros is (+Y up, +Z forward) and usd is (+Y up and -Z forward).

        Returns:
            None.

        """
        if camera_axes not in ["world", "ros", "usd"]:
            raise Exception(
                f"camera axes passed {camera_axes} is not supported: accepted values are ["
                "world"
                ", "
                "ros"
                ", "
                "usd"
                "] only"
            )
        if orientation is not None:
            if camera_axes == "world":
                orientation = self._backend_utils.convert(orientation, device=self._device)
                world_w_cam_w_R = self._backend_utils.quats_to_rot_matrices(orientation)
                w_u_R = self._backend_utils.create_tensor_from_list(
                    W_U_TRANSFORM[:3, :3].tolist(), dtype="float32", device=self._device
                )
                orientation = self._backend_utils.rot_matrices_to_quats(
                    self._backend_utils.matmul(world_w_cam_w_R, w_u_R)
                )
            elif camera_axes == "ros":
                orientation = self._backend_utils.convert(orientation, device=self._device)
                world_w_cam_r_R = self._backend_utils.quats_to_rot_matrices(orientation)
                r_u_R = self._backend_utils.create_tensor_from_list(
                    R_U_TRANSFORM[:3, :3].tolist(), dtype="float32", device=self._device
                )
                orientation = self._backend_utils.rot_matrices_to_quats(
                    self._backend_utils.matmul(world_w_cam_r_R, r_u_R)
                )
        return BaseSensor.set_world_pose(self, position, orientation)

    def get_local_pose(self, camera_axes: str = "world") -> tuple[np.ndarray, np.ndarray]:
        """Gets prim's pose with respect to the local frame (the prim's parent frame in the world axes).

        Args:
            camera_axes: camera axes, world is (+Z up, +X forward), ros is (+Y up, +Z forward) and usd is (+Y up and -Z forward).

        Returns:
            first index is position in the local frame of the prim. shape is (3, ).
            second index is quaternion orientation in the local frame of the prim.
            quaternion is scalar-first (w, x, y, z). shape is (4, ).

        """
        if camera_axes not in ["world", "ros", "usd"]:
            raise Exception(
                f"camera axes passed {camera_axes} is not supported: accepted values are ["
                "world"
                ", "
                "ros"
                ", "
                "usd"
                "] only"
            )
        translation, orientation = BaseSensor.get_local_pose(self)
        if camera_axes == "world":
            parent_w_cam_u_R = self._backend_utils.quats_to_rot_matrices(orientation)
            u_w_R = self._backend_utils.create_tensor_from_list(
                U_W_TRANSFORM[:3, :3].tolist(), dtype="float32", device=self._device
            )
            orientation = self._backend_utils.rot_matrices_to_quats(self._backend_utils.matmul(parent_w_cam_u_R, u_w_R))
        elif camera_axes == "ros":
            parent_w_cam_u_R = self._backend_utils.quats_to_rot_matrices(orientation)
            u_r_R = self._backend_utils.create_tensor_from_list(
                U_R_TRANSFORM[:3, :3].tolist(), dtype="float32", device=self._device
            )
            orientation = self._backend_utils.rot_matrices_to_quats(self._backend_utils.matmul(parent_w_cam_u_R, u_r_R))
        return translation, orientation

    def set_local_pose(
        self,
        translation: Sequence[float] | None = None,
        orientation: Sequence[float] | None = None,
        camera_axes: str = "world",
    ) -> None:
        """Sets prim's pose with respect to the local frame (the prim's parent frame in the world axes).

        Args:
            translation: translation in the local frame of the prim
                (with respect to its parent prim). shape is (3, ).
            orientation: quaternion orientation in the local frame of the prim.
                quaternion is scalar-first (w, x, y, z). shape is (4, ).
            camera_axes: camera axes, world is (+Z up, +X forward), ros is (+Y up, +Z forward) and usd is (+Y up and -Z forward).

        Returns:
            None.

        """
        if camera_axes not in ["world", "ros", "usd"]:
            raise Exception(
                f"camera axes passed {camera_axes} is not supported: accepted values are ["
                "world"
                ", "
                "ros"
                ", "
                "usd"
                "] only"
            )
        if orientation is not None:
            if camera_axes == "world":
                orientation = self._backend_utils.convert(orientation, device=self._device)
                parent_w_cam_w_R = self._backend_utils.quats_to_rot_matrices(orientation)
                w_u_R = self._backend_utils.create_tensor_from_list(
                    W_U_TRANSFORM[:3, :3].tolist(), dtype="float32", device=self._device
                )
                orientation = self._backend_utils.rot_matrices_to_quats(
                    self._backend_utils.matmul(parent_w_cam_w_R, w_u_R)
                )
            elif camera_axes == "ros":
                orientation = self._backend_utils.convert(orientation, device=self._device)
                parent_w_cam_r_R = self._backend_utils.quats_to_rot_matrices(orientation)
                r_u_R = self._backend_utils.create_tensor_from_list(
                    R_U_TRANSFORM[:3, :3].tolist(), dtype="float32", device=self._device
                )
                orientation = self._backend_utils.rot_matrices_to_quats(
                    self._backend_utils.matmul(parent_w_cam_r_R, r_u_R)
                )
        return BaseSensor.set_local_pose(self, translation, orientation)

    def attach_annotator(self, annotator_name: str, **kwargs: object) -> None:
        """Attach an annotator to the camera.

        The annotator data will be available in get_current_frame() using a normalized key
        (e.g., "bounding_box_2d_tight" for both "bounding_box_2d_tight" and "bounding_box_2d_tight_fast").

        Note:
            ``initialize()`` must be called before this method so the camera's render product
            exists. Calling ``attach_annotator()`` beforehand (or after ``destroy()``) will raise
            ``RuntimeError`` rather than producing an opaque error from ``omni.syntheticdata``.

        Args:
            annotator_name: Name of the annotator to attach (as registered in replicator).
            **kwargs: Additional arguments to pass to the annotator.

        Raises:
            RuntimeError: If ``initialize()`` has not been called (no render product exists yet).
            rep.annotators.AnnotatorRegistryError: If the annotator is not found.

        Returns:
            None.

        """
        # Fail fast with a clear message; otherwise annotator.attach([None]) crashes deep inside
        # omni.syntheticdata with an opaque Boost.Python.ArgumentError and leaves the SDG graph
        # in a partially-initialized state.
        if self._render_product_path is None:
            raise RuntimeError(
                "Camera.initialize() must be called before attach_annotator(). "
                "Call camera.initialize() to create the render product first."
            )

        # Normalize key by removing variant suffixes for consistent frame data access
        frame_key = annotator_name.replace("_fast", "")

        if frame_key in self._custom_annotators:
            carb.log_warn(f"Annotator {frame_key} already attached to {self._render_product_path}")
            return

        try:
            annotator = rep.AnnotatorRegistry.get_annotator(
                annotator_name, device=self._annotator_device, init_params=kwargs
            )
        except rep.annotators.AnnotatorRegistryError as e:
            raise e

        annotator.attach([self._render_product_path])
        self._custom_annotators[frame_key] = annotator
        self._current_frame[frame_key] = None

    def detach_annotator(self, annotator_name: str) -> None:
        """Detach an annotator from the camera.

        The annotator is looked up using a normalized key (e.g., "bounding_box_2d_tight" matches
        both "bounding_box_2d_tight" and "bounding_box_2d_tight_fast").

        Args:
            annotator_name: Name of the annotator to detach.

        """
        # Normalize key by removing variant suffixes for consistent frame data access
        frame_key = annotator_name.replace("_fast", "")

        if frame_key in self._custom_annotators:
            annotator = self._custom_annotators.pop(frame_key)
            annotator.detach()
            self._current_frame.pop(frame_key)
        else:
            carb.log_warn(f"Annotator {frame_key} not attached to {self._render_product_path}")

    def add_rgb_to_frame(self, init_params: dict | None = None) -> None:
        """Attach the rgb annotator to this camera.

        Args:
            init_params: Optional annotator parameters

        The rgbannotator returns:

        .. code-block:: python

            np.array
            shape: (width, height, 4)
            dtype: np.float32

        See more details: https://docs.omniverse.nvidia.com/extensions/latest/ext_replicator/annotators_details.html#ldrcolor

        """
        if init_params is None:
            init_params = {}
        self.attach_annotator(annotator_name="rgb", **init_params)

    def remove_rgb_from_frame(self) -> None:
        """Detach the rgb annotator from the camera."""
        self.detach_annotator(annotator_name="rgb")

    def add_normals_to_frame(self, init_params: dict | None = None) -> None:
        """Attach the normals annotator to this camera.

        Args:
            init_params: Optional annotator parameters

        Note:
            The normals annotator returns:

            .. code-block:: python

                np.array
                shape: (width, height, 4)
                dtype: np.float32

            See more details: https://docs.omniverse.nvidia.com/extensions/latest/ext_replicator/annotators_details.html#normals

        """
        if init_params is None:
            init_params = {}
        self.attach_annotator(annotator_name="normals", **init_params)

    def remove_normals_from_frame(self) -> None:
        """Detach the normals annotator from the camera."""
        self.detach_annotator(annotator_name="normals")

    def add_motion_vectors_to_frame(self, init_params: dict | None = None) -> None:
        """Attach the motion vectors annotator to this camera.

        Args:
            init_params: Optional annotator parameters

        Note:
            The motion vectors annotator returns:

            .. code-block:: python

                np.array
                shape: (width, height, 4)
                dtype: np.float32

            See more details: https://docs.omniverse.nvidia.com/extensions/latest/ext_replicator/annotators_details.html#motion-vectors

        """
        if init_params is None:
            init_params = {}
        self.attach_annotator(annotator_name="motion_vectors", **init_params)

    def remove_motion_vectors_from_frame(self) -> None:
        """Detach the motion vectors annotator from the camera."""
        self.detach_annotator(annotator_name="motion_vectors")

    def add_occlusion_to_frame(self, init_params: dict | None = None) -> None:
        """Attach the occlusion annotator to this camera.

        Args:
            init_params: Optional annotator parameters

        Note:
            The occlusion annotator returns:

            .. code-block:: python

                np.array
                shape: (num_objects, 1)
                dtype: np.dtype([("instanceId", "<u4"), ("semanticId", "<u4"), ("occlusionRatio", "<f4")])

        Returns:
            None.

        """
        if init_params is None:
            init_params = {}
        self.attach_annotator(annotator_name="occlusion", **init_params)
        return

    def remove_occlusion_from_frame(self) -> None:
        """Detach the occlusion annotator from the camera."""
        self.detach_annotator(annotator_name="occlusion")

    def add_distance_to_image_plane_to_frame(self, init_params: dict | None = None) -> None:
        """Attach the distance_to_image_plane annotator to this camera.

        Args:
            init_params: Optional annotator parameters

        Note:
            The distance_to_image_plane annotator returns:

            .. code-block:: python

                np.array
                shape: (width, height, 1)
                dtype: np.float32

            See more details: https://docs.omniverse.nvidia.com/extensions/latest/ext_replicator/annotators_details.html#distance-to-image-plane

        Returns:
            None.

        """
        if init_params is None:
            init_params = {}
        self.attach_annotator(annotator_name="distance_to_image_plane", **init_params)
        return

    def remove_distance_to_image_plane_from_frame(self) -> None:
        """Detach the distance_to_image_plane annotator from the camera."""
        self.detach_annotator(annotator_name="distance_to_image_plane")

    def add_distance_to_camera_to_frame(self, init_params: dict | None = None) -> None:
        """Attach the distance_to_camera_to_frame annotator to this camera.

        Args:
            init_params: Optional annotator parameters

        Note:
            The distance_to_camera_to_frame annotator returns:

            .. code-block:: python

                np.array
                shape: (width, height, 1)
                dtype: np.float32

            See more details: https://docs.omniverse.nvidia.com/extensions/latest/ext_replicator/annotators_details.html#distance-to-camera

        """
        if init_params is None:
            init_params = {}
        self.attach_annotator(annotator_name="distance_to_camera", **init_params)

    def remove_distance_to_camera_from_frame(self) -> None:
        """Detach the distance_to_camera annotator from the camera."""
        self.detach_annotator(annotator_name="distance_to_camera")

    def add_bounding_box_2d_tight_to_frame(self, init_params: dict | None = None) -> None:
        """Attach the bounding_box_2d_tight annotator to this camera.

        Args:
            init_params: Optional annotator parameters (e.g. init_params={"semanticTypes": ["prim"]})

        The bounding_box_2d_tight annotator returns:
            np.array
            shape: (num_objects, 1)
            dtype: np.dtype([

                                ("semanticId", "<u4"),
                                ("x_min", "<i4"),
                                ("y_min", "<i4"),
                                ("x_max", "<i4"),
                                ("y_max", "<i4"),
                                ("occlusionRatio", "<f4"),

                            ])

        See more details: https://docs.omniverse.nvidia.com/extensions/latest/ext_replicator/annotators_details.html#bounding-box-2d-tight

        """
        if init_params is None:
            init_params = {}
        self.attach_annotator(annotator_name="bounding_box_2d_tight_fast", **init_params)

    def remove_bounding_box_2d_tight_from_frame(self) -> None:
        """Detach the bounding_box_2d_tight annotator from the camera."""
        self.detach_annotator(annotator_name="bounding_box_2d_tight_fast")

    def add_bounding_box_2d_loose_to_frame(self, init_params: dict | None = None) -> None:
        """Attach the bounding_box_2d_loose annotator to this camera.

        Args:
            init_params: Optional annotator parameters (e.g. init_params={"semanticTypes": ["prim"]})

        The bounding_box_2d_loose annotator returns:

            np.array
            shape: (num_objects, 1)
            dtype: np.dtype([

                                ("semanticId", "<u4"),
                                ("x_min", "<i4"),
                                ("y_min", "<i4"),
                                ("x_max", "<i4"),
                                ("y_max", "<i4"),
                                ("occlusionRatio", "<f4"),

                            ])

        See more details: https://docs.omniverse.nvidia.com/extensions/latest/ext_replicator/annotators_details.html#bounding-box-2d-loose

        """
        if init_params is None:
            init_params = {}
        self.attach_annotator(annotator_name="bounding_box_2d_loose_fast", **init_params)

    def remove_bounding_box_2d_loose_from_frame(self) -> None:
        """Detach the bounding_box_2d_loose annotator from the camera."""
        self.detach_annotator(annotator_name="bounding_box_2d_loose_fast")

    def add_bounding_box_3d_to_frame(self, init_params: dict | None = None) -> None:
        """Attach the bounding_box_3d annotator to this camera.

        Args:
            init_params: Optional annotator parameters (e.g. init_params={"semanticTypes": ["prim"]})

        See more details: https://docs.omniverse.nvidia.com/extensions/latest/ext_replicator/annotators_details.html#bounding-box-3d

        """
        if init_params is None:
            init_params = {}
        self.attach_annotator(annotator_name="bounding_box_3d", **init_params)

    def remove_bounding_box_3d_from_frame(self) -> None:
        """Detach the bounding_box_3d annotator from the camera."""
        self.detach_annotator(annotator_name="bounding_box_3d")

    def add_semantic_segmentation_to_frame(self, init_params: dict | None = None) -> None:
        """Attach the semantic_segmentation annotator to this camera.

        Args:
            init_params: Optional parameters specifying the parameters to initialize the annotator with

        The semantic_segmentation annotator returns:

            np.array
            shape: (width, height, 1) or (width, height, 4) if `colorize` is set to true
            dtype: np.uint32 or np.uint8 if `colorize` is set to true (e.g. init_params={"colorize": True})

        See more details: https://docs.omniverse.nvidia.com/extensions/latest/ext_replicator/annotators_details.html#semantic-segmentation

        Returns:
            None.

        """
        if init_params is None:
            init_params = {}
        self.attach_annotator(annotator_name="semantic_segmentation", **init_params)
        return

    def remove_semantic_segmentation_from_frame(self) -> None:
        """Detach the semantic_segmentation annotator from the camera."""
        self.detach_annotator(annotator_name="semantic_segmentation")

    def add_instance_id_segmentation_to_frame(self, init_params: dict | None = None) -> None:
        """Attach the instance_id_segmentation annotator to this camera.

        Args:
            init_params: Optional parameters specifying the parameters to initialize the annotator with

        The instance_id_segmentation annotator returns:

            np.array
            shape: (width, height, 1) or (width, height, 4) if `colorize` is set to true
            dtype: np.uint32 or np.uint8 if `colorize` is set to true (e.g. init_params={"colorize": True})

        See more details: https://docs.omniverse.nvidia.com/extensions/latest/ext_replicator/annotators_details.html#instance-id-segmentation

        """
        if init_params is None:
            init_params = {}
        self.attach_annotator(annotator_name="instance_id_segmentation_fast", **init_params)

    def remove_instance_id_segmentation_from_frame(self) -> None:
        """Detach the instance_id_segmentation annotator from the camera."""
        self.detach_annotator(annotator_name="instance_id_segmentation_fast")

    def add_instance_segmentation_to_frame(self, init_params: dict | None = None) -> None:
        """Attach the instance_segmentation annotator to this camera.

        The main difference between instance id segmentation and instance segmentation are that instance segmentation annotator goes down the hierarchy to the lowest level prim which has semantic labels, which instance id segmentation always goes down to the leaf prim.

        Args:
            init_params: Optional parameters specifying the parameters to initialize the annot (e.g. init_params={"colorize": True})

        The instance_segmentation annotator returns:

            np.array
            shape: (width, height, 1) or (width, height, 4) if `colorize` is set to true
            dtype: np.uint32 or np.uint8 if `colorize` is set to true

        See more details: https://docs.omniverse.nvidia.com/extensions/latest/ext_replicator/annotators_details.html#instance-segmentation

        """
        if init_params is None:
            init_params = {}
        self.attach_annotator(annotator_name="instance_segmentation_fast", **init_params)

    def remove_instance_segmentation_from_frame(self) -> None:
        """Detach the instance_segmentation annotator from the camera."""
        self.detach_annotator(annotator_name="instance_segmentation_fast")

    def add_pointcloud_to_frame(self, include_unlabelled: bool = True, init_params: dict | None = None) -> None:
        """Attach the pointcloud annotator to this camera.

        Args:
            include_unlabelled: Optional parameter to include unlabelled points in the pointcloud
            init_params: Optional parameters specifying the parameters to initialize the annotator with

        The pointcloud annotator returns:

            np.array
            shape: (num_points, 3)
            dtype: np.float32

        See more details: https://docs.omniverse.nvidia.com/extensions/latest/ext_replicator/annotators_details.html#point-cloud

        Returns:
            None.

        """
        if init_params is None:
            init_params = {}
        if "includeUnlabelled" not in init_params:
            init_params["includeUnlabelled"] = include_unlabelled
        self.attach_annotator(annotator_name="pointcloud", **init_params)
        return

    def remove_pointcloud_from_frame(self) -> None:
        """Detach the pointcloud annotator from the camera."""
        self.detach_annotator(annotator_name="pointcloud")

    def get_rgba(self, device: str = None) -> np.ndarray | wp.array:
        """Get RGBA color data from the camera sensor.

        Args:
            device: Device to hold data in. Select from `['cpu', 'cuda', 'cuda:<device_index>']`.
                If None, uses the device specified on annotator initialization (annotator_device)

        Returns:
            (height, width, 4) RGBA color data as np.ndarray or wp.array.
            Returns None if annotator is not attached or data is invalid.

        Note:
            A few render frames may be required after initialization before valid data becomes available.

        """
        if "rgb" in self._custom_annotators:
            data = self._custom_annotators["rgb"].get_data(device=device)
            if data is None or data.ndim != 3:
                carb.log_warn(
                    f"Annotator 'rgb' returned None or unexpected shape for {self._render_product_path}. "
                    "A few render frames may be required before data is available."
                )
                return None
            if data.shape[0] == 0:
                return None
            return data
        else:
            carb.log_warn(f"Annotator 'rgb' not attached to {self._render_product_path}")
            return None

    def get_rgb(self, device: str = None) -> np.ndarray | wp.array:
        """Get RGB color data from the camera sensor.

        Args:
            device: Device to hold data in. Select from `['cpu', 'cuda', 'cuda:<device_index>']`.
                If None, uses the device specified on annotator initialization (annotator_device)

        Returns:
            (height, width, 3) RGB color data as np.ndarray or wp.array.
            Returns None if annotator is not attached or data is invalid.

        Note:
            A few render frames may be required after initialization before valid data becomes available.

        """
        if "rgb" in self._custom_annotators:
            data = self._custom_annotators["rgb"].get_data(device=device)
            if data is None or data.ndim != 3:
                carb.log_warn(
                    f"Annotator 'rgb' returned None or unexpected shape for {self._render_product_path}. "
                    "A few render frames may be required before data is available."
                )
                return None
            if data.shape[0] == 0:
                return None
            return data[:, :, :3]
        else:
            carb.log_warn(f"Annotator 'rgb' not attached to {self._render_product_path}")
            return None

    def get_depth(self, device: str = None) -> np.ndarray | wp.array:
        """Gets the depth data from the camera sensor as distance to image plane.

        Args:
            device: Device to hold data in. Select from `['cpu', 'cuda', 'cuda:<device_index>']`.
                If None, uses the device specified on annotator initialization (annotator_device)

        Returns:
            (height, width) depth data as np.ndarray or wp.array.
            Returns None if annotator is not attached or data is invalid.

        Note:
            A few render frames may be required after initialization before valid data becomes available.

        """
        if "distance_to_image_plane" in self._custom_annotators:
            data = self._custom_annotators["distance_to_image_plane"].get_data(device=device)
            if data is None or data.ndim != 2:
                carb.log_warn(
                    f"Annotator 'distance_to_image_plane' returned None or unexpected shape for {self._render_product_path}. "
                    "A few render frames may be required before data is available."
                )
                return None
            if data.shape[0] == 0:
                return None
            return data
        else:
            carb.log_warn(f"Annotator 'distance_to_image_plane' not attached to {self._render_product_path}")
            return None

    def get_pointcloud(self, device: str = None, world_frame: bool = True) -> np.ndarray | wp.array:
        """Get a 3D pointcloud from the camera sensor.

        This method attempts to use the pointcloud annotator first, falling back to depth-based
        calculation using the distance_to_image_plane annotator and perspective projection with
        the camera's intrinsic parameters.

        Args:
            device: Device to place tensors on. Select from ['cpu', 'cuda', 'cuda:<device_index>'].
                If None, uses self._annotator_device.
            world_frame: If True, returns points in world frame. If False, returns points in camera frame.

        Returns:
            A (N, 3) array of 3D points (X, Y, Z) in either world or camera frame,
            where N is the number of points. Returns empty array if data is not available.

        Note:
            A few render frames may be required after initialization before valid data becomes available.
            The fallback method uses the depth (distance_to_image_plane) annotator and
            performs a perspective projection using the camera's intrinsic parameters to generate the pointcloud.
            Point ordering may differ between the pointcloud annotator and depth-based fallback methods,
            even though the 3D locations are equivalent.

        """
        # Use annotator device as fallback if device is None
        device = self._annotator_device if device is None else device

        # Try to get pointcloud from custom annotator first
        if annot := self._custom_annotators.get("pointcloud"):
            pointcloud_data = annot.get_data(device=device).get("data")
            if pointcloud_data is None or (hasattr(pointcloud_data, "ndim") and pointcloud_data.ndim != 2):
                carb.log_warn(
                    f"Annotator 'pointcloud' returned None or unexpected shape for {self._render_product_path}. "
                    "A few render frames may be required before data is available."
                )
                return np.array([])
            if world_frame:
                return pointcloud_data
            else:
                # For warp arrays, we use torch_utils until warp has feature parity
                is_warp_array = isinstance(pointcloud_data, wp.array)
                backend_utils = torch_utils if is_warp_array else np_utils

                # If using warp array, convert to torch tensor for processing (zero-copy operation)
                if is_warp_array:
                    pointcloud_data = wp.to_torch(pointcloud_data)
                    pointcloud_data = pointcloud_data.to(device)  # Ensure tensor is on correct device

                # Convert points to homogeneous coordinates by adding a column of ones
                homogeneous_points = backend_utils.pad(pointcloud_data, ((0, 0), (0, 1)), value=1.0)

                # Get the view matrix that transforms from world to camera coordinates
                view_matrix = self.get_view_matrix_ros(device=device, backend_utils_cls=backend_utils)

                # Apply the transformation, transpose points to get shape compatible with matrix multiplication
                transposed_points = backend_utils.transpose_2d(homogeneous_points)

                # Multiply by view matrix
                transformed_points = backend_utils.matmul(view_matrix, transposed_points)
                # Take only the first 3 rows (x,y,z) and transpose back
                points_camera_frame = backend_utils.transpose_2d(transformed_points[:3, :])

                # Convert back to warp if torch was used as alternative backend
                if is_warp_array:
                    points_camera_frame = wp.from_torch(points_camera_frame)

                return points_camera_frame

        # Pointcloud annotator not available, try depth-based fallback
        carb.log_warn(
            f"Annotator 'pointcloud' not attached to {self._render_product_path}, falling back to depth-based calculation"
        )

        depth = self.get_depth(device=device)
        if depth is None or depth.shape[0] == 0:
            return np.array([])

        # Determine backend based on device and depth type
        is_warp_array = isinstance(depth, wp.array)
        backend_utils = torch_utils if is_warp_array else np_utils

        # Convert warp array to torch tensor for calculation
        if is_warp_array:
            depth = wp.to_torch(depth)
            depth = depth.to(device)  # Ensure tensor is on correct device

        # Create pixel coordinate grid centered around the image center
        im_height, im_width = depth.shape[0], depth.shape[1]

        if backend_utils == torch_utils:
            # Create coordinate grid using torch
            ww = torch.linspace(0.5, im_width - 0.5, im_width, dtype=torch.float32, device=device)
            hh = torch.linspace(0.5, im_height - 0.5, im_height, dtype=torch.float32, device=device)
            xmap, ymap = torch.meshgrid(ww, hh, indexing="xy")
            points_2d = torch.stack((xmap.flatten(), ymap.flatten()), dim=1)
        else:
            # Use numpy for non-warp arrays and CPU
            ww = np.linspace(0.5, im_width - 0.5, im_width, dtype=np.float32)
            hh = np.linspace(0.5, im_height - 0.5, im_height, dtype=np.float32)
            xmap, ymap = np.meshgrid(ww, hh, indexing="xy")
            points_2d = np.column_stack((xmap.ravel(), ymap.ravel()))

        # Project 2D pixel coordinates to 3D world points using depth values
        if world_frame:
            points_3d = self.get_world_points_from_image_coords(
                points_2d, depth.flatten(), device=device, backend_utils_cls=backend_utils
            )
        else:
            points_3d = self.get_camera_points_from_image_coords(
                points_2d, depth.flatten(), device=device, backend_utils_cls=backend_utils
            )

        # Convert back to warp array if input was warp array
        if is_warp_array and not isinstance(points_3d, wp.array):
            points_3d = wp.from_torch(points_3d)

        return points_3d

    def get_focal_length(self) -> float:
        """Focal length of camera prim, in stage units. Longer focal length corresponds to narrower FOV, shorter focal length corresponds to wider FOV.

        Returns:
            Value of camera prim focalLength attribute, converted to stage units.

        """
        return self.prim.GetAttribute("focalLength").Get() / USD_CAMERA_TENTHS_TO_STAGE_UNIT

    def set_focal_length(self, value: float) -> None:
        """Sets focal length of camera prim, in stage units. Longer focal length corresponds to narrower FOV, shorter focal length corresponds to wider FOV.

        Args:
            value: Desired focal length of camera prim, in stage units.

        Returns:
            None.

        """
        self.prim.GetAttribute("focalLength").Set(value * USD_CAMERA_TENTHS_TO_STAGE_UNIT)
        return

    def get_focus_distance(self) -> float:
        """Gets distance from the camera to the focus plane (in stage units).

        Returns:
            Value of camera prim focusDistance attribute, measuring distance from the camera to the focus plane
            (in stage units).

        """
        return self.prim.GetAttribute("focusDistance").Get()

    def set_focus_distance(self, value: float) -> None:
        """Sets distance from the camera to the focus plane (in stage units).

        Args:
            value: Sets value of camera prim focusDistance attribute (in stage units).

        Returns:
            None.

        """
        self.prim.GetAttribute("focusDistance").Set(value)
        return

    def get_lens_aperture(self) -> float:
        """Gets value of camera prim fStop attribute, which controls distance blurring. Lower numbers decrease focus.

                range, larger numbers increase it.

        Returns:
            Value of camera prim fStop attribute. 0 turns off focusing.

        """
        return self.prim.GetAttribute("fStop").Get()

    def set_lens_aperture(self, value: float) -> None:
        """Sets value of camera prim fStop attribute, which controls distance blurring. Lower numbers decrease focus.

                range, larger numbers increase it.

        Args:
            value: Sets value of camera prim fStop attribute. 0 turns off focusing.

        Returns:
            None.

        """
        self.prim.GetAttribute("fStop").Set(value)
        return

    def get_horizontal_aperture(self) -> float:
        """Get horizontal aperture (sensor width) in stage units.

        Only square pixels are supported; vertical aperture should match aspect ratio.

        Returns:
            Horizontal aperture in stage units.

        """
        return self.prim.GetAttribute("horizontalAperture").Get() / USD_CAMERA_TENTHS_TO_STAGE_UNIT

    def set_horizontal_aperture(self, value: float, maintain_square_pixels: bool = True) -> None:
        """Set horizontal aperture (sensor width) in stage units and update vertical for square pixels.

        Only square pixels are supported; vertical aperture is updated to match aspect ratio.

        Args:
            value: Horizontal aperture in stage units.
            maintain_square_pixels: If True, keep apertures in sync for square pixels.

        Returns:
            None.

        """
        self.prim.GetAttribute("horizontalAperture").Set(value * USD_CAMERA_TENTHS_TO_STAGE_UNIT)
        if maintain_square_pixels:
            self._maintain_square_pixel_aperture(mode="horizontal")
        return

    def get_vertical_aperture(self) -> float:
        """Get vertical aperture (sensor height) in stage units.

        This function ensures the vertical aperture is always synchronized with the aspect ratio and horizontal
        aperture to maintain square pixels. If not, it will automatically correct the value.

        Returns:
            Vertical aperture in stage units, always in sync with the aspect ratio and horizontal aperture.

        """
        return self.prim.GetAttribute("verticalAperture").Get() / USD_CAMERA_TENTHS_TO_STAGE_UNIT

    def set_vertical_aperture(self, value: float, maintain_square_pixels: bool = True) -> None:
        """Set vertical aperture (sensor height) in stage units and update horizontal for square pixels.

        Only square pixels are supported; horizontal aperture is updated to match aspect ratio.

        Args:
            value: Vertical aperture in stage units.
            maintain_square_pixels: If True, keep apertures in sync for square pixels.

        Returns:
            None.

        """
        self.prim.GetAttribute("verticalAperture").Set(value * USD_CAMERA_TENTHS_TO_STAGE_UNIT)
        if maintain_square_pixels:
            self._maintain_square_pixel_aperture(mode="vertical")
        return

    def get_clipping_range(self) -> tuple[float, float]:
        """Gets near and far clipping distances of camera prim.

        Returns:
            Near and far clipping distances (in stage units).

        """
        near, far = self.prim.GetAttribute("clippingRange").Get()
        return near, far

    def set_clipping_range(self, near_distance: float | None = None, far_distance: float | None = None) -> None:
        """Sets near and far clipping distances of camera prim.

        Args:
            near_distance: Value to be used for near clipping (in stage units).
            far_distance: Value to be used for far clipping (in stage units).

        Returns:
            None.

        """
        near, far = self.prim.GetAttribute("clippingRange").Get()
        if near_distance is not None:
            near = near_distance
        if far_distance is not None:
            far = far_distance
        self.prim.GetAttribute("clippingRange").Set((near, far))
        return

    def get_projection_type(self) -> str:
        """[DEPRECATED] Gets the `cameraProjectionType` property of the camera prim.

        Returns:
            omni:lensdistortion:model attribute of camera prim. If unset, returns "pinhole".

        """
        carb.log_warn(
            "Camera.get_projection_type is deprecated and will be removed in a future release. Please use Camera.get_lens_distortion_model instead."
        )
        projection_type = self.prim.GetAttribute("cameraProjectionType").Get()
        if projection_type is None:
            projection_type = "pinhole"
        return projection_type

    def set_projection_type(self, value: str) -> None:
        """[DEPRECATED] Sets the `cameraProjectionType` property of the camera prim.

        Args:
            value: Name of the projection type to apply, or "pinhole" to remove any distortion schemas and unset `omni:lensdistortion:model`.

        Returns:
            None.

        """
        carb.log_warn(
            "Camera.set_projection_type is deprecated and will be removed in a future release. Please use Camera.set_lens_distortion_model instead."
        )
        if value == "pinhole":
            self.set_lens_distortion_model(value)
        elif value == "fisheyePolynomial":
            carb.log_warn(
                "Use of 'fisheyePolynomial' in Camera.set_projection_type is deprecated. Please use 'OmniLensDistortionFthetaAPI' instead."
            )
            self.set_lens_distortion_model("OmniLensDistortionFthetaAPI")
        else:
            carb.log_warn(
                f"'{value}' is not a supported projection type. Please use a supported OmniLensDistortion*API instead."
            )
        if self.prim.HasAttribute("cameraProjectionType"):
            self.prim.GetAttribute("cameraProjectionType").Set(value)
        else:
            self.prim.CreateAttribute("cameraProjectionType", Sdf.ValueTypeNames.String, False).Set(value)
        return

    def get_lens_distortion_model(self) -> str:
        """Gets the `omni:lensdistortion:model` property of the camera prim.

        Returns:
            The lens distortion model name or "pinhole" if unset.

        """
        return self.prim.GetAttribute("omni:lensdistortion:model").Get() or "pinhole"

    def set_lens_distortion_model(self, value: str) -> None:
        """Sets the `omni:lensdistortion:model` property of the camera prim and applies the corresponding schema.

        Note: `cameraProjectionType` has been deprecated in favor of `omni:lensdistortion:model`. `fisheyeOrthographic`, `fisheyeEquidistant`, `fisheyeEquisolid`, and `fisheyeSpherical` are no longer supported.

        Args:
            value: Name of the distortion schema to apply, or "pinhole" to remove any distortion schemas and unset `omni:lensdistortion:model`.

        Returns:
            None.

        """
        if value == "pinhole":
            for schema in self.prim.GetAppliedSchemas():
                if schema.startswith("OmniLensDistortion"):
                    self.prim.RemoveAppliedSchema(schema)
                    carb.log_info(f"Removed schema {schema} from Camera {self.prim.GetPrimPath()}")
            if self.prim.HasAttribute("omni:lensdistortion:model"):
                self.prim.RemoveProperty("omni:lensdistortion:model")
                carb.log_info(f"Removed property omni:lensdistortion:model from Camera {self.prim.GetPrimPath()}")
        elif value == "OmniLensDistortionFthetaAPI":
            self.prim.ApplyAPI("OmniLensDistortionFthetaAPI")
            self.prim.GetAttribute("omni:lensdistortion:model").Set("ftheta")
        elif value == "OmniLensDistortionKannalaBrandtK3API":
            self.prim.ApplyAPI("OmniLensDistortionKannalaBrandtK3API")
            self.prim.GetAttribute("omni:lensdistortion:model").Set("kannalaBrandtK3")
        elif value == "OmniLensDistortionRadTanThinPrismAPI":
            self.prim.ApplyAPI("OmniLensDistortionRadTanThinPrismAPI")
            self.prim.GetAttribute("omni:lensdistortion:model").Set("radTanThinPrism")
        elif value == "OmniLensDistortionLutAPI":
            self.prim.ApplyAPI("OmniLensDistortionLutAPI")
            self.prim.GetAttribute("omni:lensdistortion:model").Set("lut")
        elif value == "OmniLensDistortionOpenCvFisheyeAPI":
            self.prim.ApplyAPI("OmniLensDistortionOpenCvFisheyeAPI")
            self.prim.GetAttribute("omni:lensdistortion:model").Set("opencvFisheye")
        elif value == "OmniLensDistortionOpenCvPinholeAPI":
            self.prim.ApplyAPI("OmniLensDistortionOpenCvPinholeAPI")
            self.prim.GetAttribute("omni:lensdistortion:model").Set("opencvPinhole")
        else:
            carb.log_warn(
                f"'{value}' is not a supported distortion model. Please use a supported OmniLensDistortion*API instead."
            )
        return

    def get_projection_mode(self) -> str:
        """Gets projection model of the camera prim.

        Returns:
            perspective or orthographic.

        """
        return self.prim.GetAttribute("projection").Get()

    def set_projection_mode(self, value: str) -> None:
        """Sets projection model of the camera prim to perspective or orthographic.

        Args:
            value: "perspective" or "orthographic".

        Returns:
            None.

        """
        if value not in ["perspective", "orthographic"]:
            carb.log_warn(f"'{value}' is not a supported projection mode. Please use 'perspective' or 'orthographic'.")
            return
        self.prim.GetAttribute("projection").Set(value)
        return

    def get_stereo_role(self) -> str:
        """Gets stereo role of the camera prim.

        Returns:
            "mono", "left" or "right".

        """
        return self.prim.GetAttribute("stereoRole").Get()

    def set_stereo_role(self, value: str) -> None:
        """Sets stereo role of the camera prim to mono, left or right.

        Args:
            value: "mono", "left" or "right".

        Returns:
            None.

        """
        if value not in ["mono", "left", "right"]:
            carb.log_warn(f"'{value}' is not a supported stereo role. Please use 'mono', 'left' or 'right'.")
            return
        self.prim.GetAttribute("stereoRole").Set(value)
        return

    def set_fisheye_polynomial_properties(
        self,
        nominal_width: float | None,
        nominal_height: float | None,
        optical_centre_x: float | None,
        optical_centre_y: float | None,
        max_fov: float | None,
        polynomial: Sequence[float] | None,
    ) -> None:
        """[DEPRECATED] Sets distortion parameters for the fisheyePolynomial projection model.

        Args:
            nominal_width: Rendered Width (pixels)
            nominal_height: Rendered Height (pixels)
            optical_centre_x: Horizontal Render Position (pixels)
            optical_centre_y: Vertical Render Position (pixels)
            max_fov: maximum field of view (pixels)
            polynomial: polynomial equation coefficients (sequence of 5 numbers) starting from A0, A1, A2, A3, A4

        Returns:
            None.

        """
        omni.kit.app.log_deprecation(
            "Camera.set_matching_fisheye_polynomial_properties is deprecated."
            'Please use the the "opencvFisheye" distortion model to directly specify OpenCV distortion parameters.'
        )
        if "fisheye" not in self.get_projection_type():
            raise Exception(
                "fisheye projection type is not set to be able to use set_fisheye_polynomial_properties method."
            )
        if nominal_width is not None:
            self.prim.CreateAttribute("fthetaWidth", Sdf.ValueTypeNames.Float, False).Set(nominal_width)
        if nominal_height is not None:
            self.prim.CreateAttribute("fthetaHeight", Sdf.ValueTypeNames.Float, False).Set(nominal_height)
        if optical_centre_x is not None:
            self.prim.CreateAttribute("fthetaCx", Sdf.ValueTypeNames.Float, False).Set(optical_centre_x)
        if optical_centre_y is not None:
            self.prim.CreateAttribute("fthetaCy", Sdf.ValueTypeNames.Float, False).Set(optical_centre_y)
        if max_fov is not None:
            self.prim.CreateAttribute("fthetaMaxFov", Sdf.ValueTypeNames.Float, False).Set(max_fov)
        if polynomial is not None:
            for i in range(5):
                if polynomial[i] is not None:
                    self.prim.CreateAttribute("fthetaPoly" + (chr(ord("A") + i)), Sdf.ValueTypeNames.Float, False).Set(
                        float(polynomial[i])
                    )
        return

    def set_matching_fisheye_polynomial_properties(
        self,
        nominal_width: float,
        nominal_height: float,
        optical_centre_x: float,
        optical_centre_y: float,
        max_fov: float | None,
        distortion_model: Sequence[float],
        distortion_fn: Callable,
    ) -> None:
        """[DEPRECATED] Approximates provided OpenCV fisheye distortion with ftheta fisheye polynomial coefficients.

        Args:
            nominal_width: Rendered Width (pixels)
            nominal_height: Rendered Height (pixels)
            optical_centre_x: Horizontal Render Position (pixels)
            optical_centre_y: Vertical Render Position (pixels)
            max_fov: maximum field of view (pixels)
            distortion_model: distortion model coefficients
            distortion_fn: distortion function that takes points and returns distorted points

        Returns:
            None.

        """
        omni.kit.app.log_deprecation(
            "Camera.set_matching_fisheye_polynomial_properties is deprecated."
            'Please use the the "opencvFisheye" distortion model to directly specify OpenCV distortion parameters.'
        )
        if "fisheye" not in self.get_projection_type():
            raise Exception(
                "fisheye projection type is not set to allow use set_matching_fisheye_polynomial_properties method."
            )

        cx, cy = optical_centre_x, optical_centre_y
        fx = nominal_width * self.get_focal_length() / self.get_horizontal_aperture()
        fy = nominal_height * self.get_focal_length() / self.get_vertical_aperture()
        camera_matrix = np.array([[fx, 0.0, cx], [0.0, fy, cy], [0.0, 0.0, 1.0]])

        # Convert nominal width to integer for sample count
        num_samples = int(nominal_width)
        X = np.concatenate([np.linspace(0, nominal_width, num_samples), np.linspace(0, nominal_width, num_samples)])
        Y = np.concatenate([np.linspace(0, nominal_height, num_samples), np.linspace(nominal_height, 0, num_samples)])
        theta = point_to_theta(camera_matrix, X, Y)
        r = np.linalg.norm(distortion_fn(camera_matrix, distortion_model, X, Y) - np.array([[cx], [cy]]), axis=0)
        fthetaPoly = np.polyfit(r, theta, deg=4)

        for i, coefficient in enumerate(fthetaPoly[::-1]):  # Reverse the order of the coefficients
            self.prim.GetAttribute("fthetaPoly" + (chr(ord("A") + i))).Set(float(coefficient))

        self.prim.GetAttribute("fthetaWidth").Set(nominal_width)
        self.prim.GetAttribute("fthetaHeight").Set(nominal_height)
        self.prim.GetAttribute("fthetaCx").Set(optical_centre_x)
        self.prim.GetAttribute("fthetaCy").Set(optical_centre_y)

        if max_fov:
            self.prim.GetAttribute("fthetaMaxFov").Set(max_fov)
        return

    def set_rational_polynomial_properties(
        self,
        nominal_width: float,
        nominal_height: float,
        optical_centre_x: float,
        optical_centre_y: float,
        max_fov: float | None,
        distortion_model: Sequence[float],
    ) -> None:
        """[DEPRECATED] Approximates rational polynomial distortion with ftheta fisheye polynomial coefficients.

        Note: This method was designed to approximate the OpenCV pinhole distortion model using ftheta fisheye polynomial parameterization.
        The OpenCV pinhole distortion model is now directly supported, so this method will use that model directly.

        Args:
            nominal_width: Rendered Width (pixels)
            nominal_height: Rendered Height (pixels)
            optical_centre_x: Horizontal Render Position (pixels)
            optical_centre_y: Vertical Render Position (pixels)
            max_fov: DEPRECATED. maximum field of view (pixels)
            distortion_model: rational polynomial distortion model coefficients (k1, k2, p1, p2, k3, k4, k5, k6, s1, s2, s3, s4)

        Returns:
            None.

        """
        omni.kit.app.log_deprecation(
            'Camera.set_rational_polynomial_properties is deprecated. Please use the the "pinholeOpenCV" distortion model to directly specify OpenCV pinhole camera model distortion parameters.'
        )

        num_distortion_params = len(distortion_model)
        if num_distortion_params < 8:
            carb.log_warn(
                f"set_rational_polynomial_properties: Insufficient distortion parameters provided ({num_distortion_params}), expecting at least 8 for pinholeOpenCV projection type."
            )
            return

        fx = nominal_width * self.get_focal_length() / self.get_horizontal_aperture()
        fy = nominal_height * self.get_focal_length() / self.get_vertical_aperture()
        self.set_opencv_pinhole_properties(
            cx=optical_centre_x, cy=optical_centre_y, fx=fx, fy=fy, pinhole=distortion_model
        )

        return

    def set_kannala_brandt_properties(
        self,
        nominal_width: float,
        nominal_height: float,
        optical_centre_x: float,
        optical_centre_y: float,
        max_fov: float | None,
        distortion_model: Sequence[float],
    ) -> None:
        """[DEPRECATED] Approximates kannala brandt distortion with ftheta fisheye polynomial coefficients.

        Note: This method was designed to approximate the OpenCV fisheye distortion model using ftheta fisheye polynomial parameterization.
        The OpenCV fisheye distortion model is now directly supported, so this method will use that model directly.

        Args:
            nominal_width: Rendered Width (pixels)
            nominal_height: Rendered Height (pixels)
            optical_centre_x: Horizontal Render Position (pixels)
            optical_centre_y: Vertical Render Position (pixels)
            max_fov: DEPRECATED. maximum field of view (pixels)
            distortion_model: kannala brandt generic distortion model coefficients (k1, k2, k3, k4)

        Returns:
            None.

        """
        omni.kit.app.log_deprecation(
            'Camera.set_kannala_brandt_properties is deprecated. Please use the the "fisheyeOpenCV" distortion model to directly specify OpenCV fisheye camera model distortion parameters.'
        )

        num_distortion_params = len(distortion_model)
        if num_distortion_params < 4:
            carb.log_warn(
                f"set_kannala_brandt_properties: Insufficient distortion parameters provided ({num_distortion_params}), expecting at least 4 for the Kannala-Brandt model (k1, k2, k3, k4)."
            )
            return

        fx = nominal_width * self.get_focal_length() / self.get_horizontal_aperture()
        fy = nominal_height * self.get_focal_length() / self.get_vertical_aperture()
        self.set_opencv_fisheye_properties(
            cx=optical_centre_x, cy=optical_centre_y, fx=fx, fy=fy, fisheye=distortion_model
        )

        return

    def get_fisheye_polynomial_properties(self) -> tuple[float, float, float, float, float, list]:
        """nominal_width, nominal_height, optical_centre_x, optical_centre_y, max_fov and polynomial respectively.

        Returns:
            nominal_width, nominal_height, optical_centre_x, optical_centre_y, max_fov and polynomial respectively.

        """
        if "fisheye" not in self.get_projection_type():
            raise Exception(
                "fisheye projection type is not set to be able to use get_fisheye_polynomial_properties method."
            )
        nominal_width = self.prim.GetAttribute("fthetaWidth").Get()
        nominal_height = self.prim.GetAttribute("fthetaHeight").Get()
        optical_centre_x = self.prim.GetAttribute("fthetaCx").Get()
        optical_centre_y = self.prim.GetAttribute("fthetaCy").Get()
        max_fov = self.prim.GetAttribute("fthetaMaxFov").Get()
        polynomial = [None] * 5
        for i in range(5):
            polynomial[i] = self.prim.GetAttribute("fthetaPoly" + (chr(ord("A") + i))).Get()
        return nominal_width, nominal_height, optical_centre_x, optical_centre_y, max_fov, polynomial

    def set_shutter_properties(self, delay_open: float | None = None, delay_close: float | None = None) -> None:
        """Sets the shutter properties for motion blur control.

        Args:
            delay_open: Used with Motion Blur to control blur amount, increased values delay shutter opening.
            delay_close: Used with Motion Blur to control blur amount, increased values forward the shutter close.

        Returns:
            None.

        """
        if delay_open:
            self.prim.GetAttribute("shutter:open").Set(delay_open)
        if delay_close:
            self.prim.GetAttribute("shutter:close").Set(delay_close)
        return

    def get_shutter_properties(self) -> tuple[float, float]:
        """Shutter properties for motion blur control.

        Returns:
            delay_open and delay close respectively.

        """
        return self.prim.GetAttribute("shutter:open").Get(), self.prim.GetAttribute("shutter:close").Get()

    def get_view_matrix_ros(
        self, device: str = None, backend_utils_cls: type = None
    ) -> np.ndarray | torch.Tensor | wp.array:
        """3D points in World Frame -> 3D points in Camera Ros Frame.

        Args:
            device: str, optional, default is None. If None, uses self._device.
                Device to place tensors on. Select from ['cpu', 'cuda', 'cuda:<device_index>']
            backend_utils_cls: type, optional, default is None. If None, the class will be inferred from self._backend_utils.
                Supported classes are np_utils, torch_utils, and warp_utils.

        Returns:
            The view matrix that transforms 3d points in the world frame to 3d points in the camera axes
                with ros camera convention.

        """
        # Determine backend utilities
        if backend_utils_cls is None:
            if device is None:
                backend_utils = self._backend_utils
            else:
                backend_utils = torch_utils if "cuda" in str(device) else np_utils
        else:
            backend_utils = backend_utils_cls

        # Use provided device or fall back to self._device
        device = device if device is not None else self._device

        world_w_cam_u_T = backend_utils.transpose_2d(
            backend_utils.convert(
                UsdGeom.Imageable(self.prim).ComputeLocalToWorldTransform(Usd.TimeCode.Default()),
                dtype="float32",
                device=device,
                indexed=True,
            )
        )

        r_u_transform_converted = backend_utils.convert(R_U_TRANSFORM, dtype="float32", device=device, indexed=True)

        result = backend_utils.matmul(r_u_transform_converted, backend_utils.inverse(world_w_cam_u_T))
        return result

    def get_intrinsics_matrix(
        self, device: str = None, backend_utils_cls: type = None
    ) -> np.ndarray | torch.Tensor | wp.array:
        """Get the intrinsics matrix of the camera.

        Args:
            device: str, optional, default is None. If None, uses self._device.
                Device to place tensors on. Select from ['cpu', 'cuda', 'cuda:<device_index>']
            backend_utils_cls: type, optional, default is None. If None, the class will be inferred from self._backend_utils.
                Supported classes are np_utils, torch_utils, and warp_utils.

        Returns:
            The intrinsics matrix of the camera (used for calibration)

        """
        if "pinhole" not in self.get_lens_distortion_model().lower():
            raise Exception("pinhole projection type is not set to be able to use get_intrinsics_matrix method.")

        # Determine backend utilities
        if backend_utils_cls is not None:
            if backend_utils_cls not in (np_utils, torch_utils, warp_utils):
                raise ValueError(
                    f"Unsupported backend class: {backend_utils_cls}, supported classes are np_utils, torch_utils, and warp_utils"
                )
            backend_utils = backend_utils_cls
        else:
            backend_utils = self._backend_utils

        # Use provided device or fall back to self._device
        device = device if device is not None else self._device

        # Calculate intrinsics parameters
        focal_length = self.get_focal_length()
        horizontal_aperture = self.get_horizontal_aperture()
        vertical_aperture = self.get_vertical_aperture()
        width, height = self.get_resolution()
        fx = width * focal_length / horizontal_aperture
        fy = height * focal_length / vertical_aperture
        cx = width * 0.5
        cy = height * 0.5

        return backend_utils.create_tensor_from_list(
            [[fx, 0.0, cx], [0.0, fy, cy], [0.0, 0.0, 1.0]], dtype="float32", device=device
        )

    def get_image_coords_from_world_points(self, points_3d: np.ndarray) -> np.ndarray:
        """Using pinhole perspective projection, this method projects 3d points in the world frame to the image.

           plane giving the pixel coordinates [[0, width], [0, height]].

        Args:
            points_3d: 3d points (X, Y, Z) in world frame. shape is (n, 3) where n is the number of points.

        Returns:
            2d points (u, v) corresponds to the pixel coordinates. shape is (n, 2) where n is the number of points.

        """
        if "pinhole" not in self.get_lens_distortion_model().lower():
            raise Exception(
                "pinhole projection type is not set to be able to use get_image_coords_from_world_points method which use pinhole prespective projection."
            )
        homogenous = self._backend_utils.pad(points_3d, ((0, 0), (0, 1)), value=1.0)
        projection_matrix = self._backend_utils.matmul(self.get_intrinsics_matrix(), self.get_view_matrix_ros()[:3, :])
        points = self._backend_utils.matmul(projection_matrix, self._backend_utils.transpose_2d(homogenous))
        points[:2, :] /= points[2, :]  # normalize
        return self._backend_utils.transpose_2d(points[:2, :])

    def get_camera_points_from_image_coords(
        self, points_2d: object, depth: object, device: str = None, backend_utils_cls: type = None
    ) -> np.ndarray | torch.Tensor | wp.array:
        """Using pinhole perspective projection, this method does the inverse projection given the depth of the.

            pixels to get 3D points in camera frame.

        Args:
            points_2d: 2d points (u, v) corresponds to the pixel coordinates. shape is (n, 2) where n is the number of points.
            depth: depth corresponds to each of the pixel coords. shape is (n,)
            device: str, optional, default is None. If None, uses self._device.
                Device to place tensors on. Select from ['cpu', 'cuda', 'cuda:<device_index>']
            backend_utils_cls: type, optional, default is None. If None, the class will be inferred from self._backend_utils.
                Supported classes are np_utils, torch_utils, and warp_utils.

        Returns:
            (n, 3) 3d points (X, Y, Z) in camera frame.
                +Z points forward (optical axis), +X right, +Y down

        """
        if "pinhole" not in self.get_lens_distortion_model().lower():
            raise Exception(
                "pinhole projection type is not set to be able to use get_camera_points_from_image_coords method."
            )

        # Determine backend utilities
        if backend_utils_cls is None:
            if device is None:
                backend_utils = self._backend_utils
            else:
                backend_utils = torch_utils if "cuda" in str(device) else np_utils
        else:
            backend_utils = backend_utils_cls

        # Convert image coordinates to homogeneous coordinates
        homogenous = backend_utils.pad(points_2d, ((0, 0), (0, 1)), value=1.0)

        # Get intrinsics matrix using the same backend and device
        intrinsics_matrix = self.get_intrinsics_matrix(device=device, backend_utils_cls=backend_utils)

        # Back-project to camera frame using inverse of intrinsics matrix and depth
        points_in_camera_axes = backend_utils.matmul(
            backend_utils.inverse(intrinsics_matrix),
            backend_utils.transpose_2d(homogenous) * backend_utils.expand_dims(depth, 0),
        )
        return backend_utils.transpose_2d(points_in_camera_axes)

    def get_world_points_from_image_coords(
        self, points_2d: object, depth: object, device: str = None, backend_utils_cls: type = None
    ) -> np.ndarray | torch.Tensor | wp.array:
        """Using pinhole perspective projection, this method does the inverse projection given the depth of the.

            pixels to get 3D points in world frame.

        Args:
            points_2d: 2d points (u, v) corresponds to the pixel coordinates. shape is (n, 2) where n is the number of points.
            depth: depth corresponds to each of the pixel coords. shape is (n,)
            device: str, optional, default is None. If None, uses self._device.
                Device to place tensors on. Select from ['cpu', 'cuda', 'cuda:<device_index>']
            backend_utils_cls: type, optional, default is None. If None, the class will be inferred from self._backend_utils.
                Supported classes are np_utils, torch_utils, and warp_utils.

        Returns:
            (n, 3) 3d points (X, Y, Z) in world frame.

        """
        if "pinhole" not in self.get_lens_distortion_model().lower():
            raise Exception(
                "pinhole projection type is not set to be able to use get_world_points_from_image_coords method."
            )

        # Determine backend utilities
        if backend_utils_cls is None:
            if device is None:
                backend_utils = self._backend_utils
            else:
                backend_utils = torch_utils if "cuda" in str(device) else np_utils
        else:
            backend_utils = backend_utils_cls

        # First get points in camera frame
        points_in_camera_frame = self.get_camera_points_from_image_coords(
            points_2d, depth, device=device, backend_utils_cls=backend_utils
        )

        # Convert to homogeneous coordinates
        points_in_camera_frame_homogenous = backend_utils.pad(points_in_camera_frame, ((0, 0), (0, 1)), value=1.0)

        # Get view matrix using the same backend and device
        view_matrix_ros = self.get_view_matrix_ros(device=device, backend_utils_cls=backend_utils)

        # Transform to world frame
        points_in_world_frame_homogenous = backend_utils.matmul(
            backend_utils.inverse(view_matrix_ros), backend_utils.transpose_2d(points_in_camera_frame_homogenous)
        )

        return backend_utils.transpose_2d(points_in_world_frame_homogenous[:3, :])

    def get_horizontal_fov(self) -> float:
        """Horizontal field of view in pixels.

        Returns:
            Horizontal field of view in pixels.

        """
        return 2 * math.atan(self.get_horizontal_aperture() / (2 * self.get_focal_length()))

    def get_vertical_fov(self) -> float:
        """Vertical field of view in pixels.

        Returns:
            Vertical field of view in pixels.

        """
        width, height = self.get_resolution()
        return self.get_horizontal_fov() * (height / float(width))

    def _set_lens_distortion_properties(
        self,
        distortion_model: str,
        distortion_model_attr: str,
        coefficient_map: list[str],
        coefficients: list[float],
        **kwargs: object,
    ) -> None:
        """Sets lens distortion model parameters if camera prim is using lens distortion model.

        Args:
            distortion_model: Name of the distortion model to apply.
            distortion_model_attr: Attribute name for the distortion model.
            coefficient_map: List of coefficient names to map to the distortion model.
            coefficients: List of coefficient values to set for the distortion model.
            **kwargs: Additional keyword arguments passed to the parent class.

        Returns:
            None.

        """
        self.prim.ApplyAPI(f"OmniLensDistortion{distortion_model}API")
        self.prim.GetAttribute("omni:lensdistortion:model").Set(distortion_model_attr)
        for coefficient_name, coefficient_value in zip(coefficient_map or [], coefficients or []):
            if coefficient_value is None:
                continue
            self.prim.GetAttribute(f"omni:lensdistortion:{distortion_model_attr}:{coefficient_name}").Set(
                coefficient_value
            )
        for attr_name, attr_value in kwargs.items():
            if attr_value is None:
                continue
            tokens = attr_name.split("_")
            updated_attr_name = tokens[0]
            for i in tokens[1:]:
                updated_attr_name += i.capitalize()
            self.prim.GetAttribute(f"omni:lensdistortion:{distortion_model_attr}:{updated_attr_name}").Set(attr_value)
        return

    def _get_lens_distortion_properties(
        self, distortion_model_attr: str, attr_names: list[str], coefficient_map: list[str]
    ) -> tuple[float]:
        """Gets lens distortion model parameters if camera prim is using lens distortion model.

        Args:
            distortion_model_attr: Attribute name for the distortion model.
            attr_names: List of attribute names to retrieve.
            coefficient_map: List of coefficient names to retrieve from the distortion model.

        Returns:
            A tuple containing (nominal_height, nominal_width, optical_center, max_fov, distortion_coefficients)
            where distortion_coefficients are in order:

        """
        if self.prim.GetAttribute("omni:lensdistortion:model").Get() != distortion_model_attr:
            carb.log_error(f"Camera omni:lensdistortion:model attribute not set to '{distortion_model_attr}'.")
            return None
        attrs = []
        for attr_name in attr_names:
            tokens = attr_name.split("_")
            updated_attr_name = tokens[0]
            for i in tokens[1:]:
                updated_attr_name += i.capitalize()
            if attr := self.prim.GetAttribute(f"omni:lensdistortion:{distortion_model_attr}:{updated_attr_name}"):
                attrs.append(attr.Get())
            else:
                attrs.append(None)
        if coefficient_map:
            attrs.append([])
            for coefficient_name in coefficient_map:
                if attr := self.prim.GetAttribute(f"omni:lensdistortion:{distortion_model_attr}:{coefficient_name}"):
                    attrs[-1].append(attr.Get())
                else:
                    attrs[-1].append(None)
        return tuple(attrs)

    def set_ftheta_properties(
        self,
        nominal_height: float | None = None,
        nominal_width: float | None = None,
        optical_center: tuple[float, float] | None = None,
        max_fov: float | None = None,
        distortion_coefficients: Sequence[float] | None = None,
    ) -> None:
        """Applies F-theta lens distortion model to camera prim, then sets distortion parameters.

        Args:
            nominal_height: Height of the calibrated sensor in pixels.
            nominal_width: Width of the calibrated sensor in pixels.
            optical_center: Optical center (x, y) in pixels.
            max_fov: Maximum field of view in degrees.
            distortion_coefficients: Distortion coefficients in order:
                [k0, k1, k2, k3, k4] - radial distortion coefficients
                Missing coefficients default to 0.0.

        Returns:
            None.

        """
        return self._set_lens_distortion_properties(
            distortion_model="Ftheta",
            distortion_model_attr="ftheta",
            coefficient_map=FTHETA_ATTRIBUTE_MAP,
            coefficients=distortion_coefficients,
            nominal_height=nominal_height,
            nominal_width=nominal_width,
            optical_center=optical_center,
            max_fov=max_fov,
        )

    def get_ftheta_properties(self) -> tuple[float, float, tuple[float, float], float, list[float]]:
        """Gets F-theta lens distortion model parameters if camera prim is using F-theta distortion model.

        Returns:
            A tuple containing (nominal_height (pixels), nominal_width (pixels), optical_center (x,y in pixels),
            max_fov (degrees), distortion_coefficients) where distortion_coefficients are in order:
            [k0, k1, k2, k3, k4] - radial distortion coefficients.

        """
        return self._get_lens_distortion_properties(
            distortion_model_attr="ftheta",
            attr_names=["nominalHeight", "nominalWidth", "opticalCenter", "maxFov"],
            coefficient_map=FTHETA_ATTRIBUTE_MAP,
        )

    def set_kannala_brandt_k3_properties(
        self,
        nominal_height: float | None = None,
        nominal_width: float | None = None,
        optical_center: tuple[float, float] | None = None,
        max_fov: float | None = None,
        distortion_coefficients: Sequence[float] | None = None,
    ) -> None:
        """Applies Kannala-Brandt K3 lens distortion model to camera prim, then sets distortion parameters.

        Args:
            nominal_height: Height of the calibrated sensor in pixels.
            nominal_width: Width of the calibrated sensor in pixels.
            optical_center: Optical center (x, y) in pixels.
            max_fov: Maximum field of view in degrees.
            distortion_coefficients: Distortion coefficients in order:
                [k0, k1, k2, k3] - radial distortion coefficients
                Missing coefficients default to 0.0.

        Returns:
            None.

        """
        return self._set_lens_distortion_properties(
            distortion_model="KannalaBrandtK3",
            distortion_model_attr="kannalaBrandtK3",
            coefficient_map=KANNALA_BRANDT_K3_ATTRIBUTE_MAP,
            coefficients=distortion_coefficients,
            nominal_height=nominal_height,
            nominal_width=nominal_width,
            optical_center=optical_center,
            max_fov=max_fov,
        )

    def get_kannala_brandt_k3_properties(self) -> tuple[float, float, tuple[float, float], float, list[float]]:
        """Gets Kannala-Brandt K3 lens distortion model parameters if camera prim is using Kannala-Brandt K3 distortion model.

        Returns:
            A tuple containing (nominal_height, nominal_width, optical_center, max_fov, distortion_coefficients)
            where distortion_coefficients are in order:
            [k0, k1, k2, k3] - radial distortion coefficients.

        """
        return self._get_lens_distortion_properties(
            distortion_model_attr="kannalaBrandtK3",
            attr_names=["nominalHeight", "nominalWidth", "opticalCenter", "maxFov"],
            coefficient_map=KANNALA_BRANDT_K3_ATTRIBUTE_MAP,
        )

    def set_rad_tan_thin_prism_properties(
        self,
        nominal_height: float | None = None,
        nominal_width: float | None = None,
        optical_center: tuple[float, float] | None = None,
        max_fov: float | None = None,
        distortion_coefficients: Sequence[float] | None = None,
    ) -> None:
        """Applies Radial-Tangential Thin Prism lens distortion model to camera prim, then sets distortion parameters.

        Args:
            nominal_height: Height of the calibrated sensor.
            nominal_width: Width of the calibrated sensor.
            optical_center: Optical center (x, y).
            max_fov: Maximum field of view in degrees.
            distortion_coefficients: Distortion coefficients in order:
                [k0, k1, k2, k3, k4, k5] - radial distortion coefficients
                [p0, p1] - tangential distortion coefficients
                [s0, s1, s2, s3] - thin prism distortion coefficients
                Missing coefficients default to 0.0.

        Returns:
            None.

        """
        return self._set_lens_distortion_properties(
            distortion_model="RadTanThinPrism",
            distortion_model_attr="radTanThinPrism",
            coefficient_map=RAD_TAN_THIN_PRISM_ATTRIBUTE_MAP,
            coefficients=distortion_coefficients,
            nominal_height=nominal_height,
            nominal_width=nominal_width,
            optical_center=optical_center,
            max_fov=max_fov,
        )

    def get_rad_tan_thin_prism_properties(self) -> tuple[float, float, tuple[float, float], float, list[float]]:
        """Gets Radial-Tangential Thin Prism lens distortion model parameters if camera prim is using Radial-Tangential Thin Prism distortion model.

        Returns:
            A tuple containing (nominal_height, nominal_width, optical_center, max_fov, distortion_coefficients)
            where distortion_coefficients are in order:
            [k0, k1, k2, k3, k4, k5] - radial distortion coefficients
            [p0, p1] - tangential distortion coefficients
            [s0, s1, s2, s3] - thin prism distortion coefficients.

        """
        return self._get_lens_distortion_properties(
            distortion_model_attr="radTanThinPrism",
            attr_names=["nominalHeight", "nominalWidth", "opticalCenter", "maxFov"],
            coefficient_map=RAD_TAN_THIN_PRISM_ATTRIBUTE_MAP,
        )

    def set_lut_properties(
        self,
        nominal_height: float | None = None,
        nominal_width: float | None = None,
        optical_center: tuple[float, float] | None = None,
        ray_enter_direction_texture: str | None = None,
        ray_exit_position_texture: str | None = None,
    ) -> None:
        """Applies LUT lens distortion model to camera prim, then sets distortion parameters.

        Args:
            nominal_height: Height of the calibrated sensor in pixels.
            nominal_width: Width of the calibrated sensor in pixels.
            optical_center: Optical center (x, y) in pixels.
            ray_enter_direction_texture: Path to ray enter direction texture.
            ray_exit_position_texture: Path to ray exit position texture.

        Returns:
            None.

        """
        return self._set_lens_distortion_properties(
            distortion_model="Lut",
            distortion_model_attr="lut",
            coefficient_map=[],
            coefficients=[],
            nominal_height=nominal_height,
            nominal_width=nominal_width,
            optical_center=optical_center,
            ray_enter_direction_texture=ray_enter_direction_texture,
            ray_exit_position_texture=ray_exit_position_texture,
        )

    def get_lut_properties(self) -> tuple[float, float, tuple[float, float], str, str]:
        """Gets LUT lens distortion model parameters if camera prim is using LUT distortion model.

        Returns:
            A tuple containing (nominal_height (pixels), nominal_width (pixels),
            optical_center (x,y in pixels), ray_enter_direction_texture, ray_exit_position_texture).

        """
        return self._get_lens_distortion_properties(
            distortion_model_attr="lut",
            attr_names=[
                "nominalHeight",
                "nominalWidth",
                "opticalCenter",
                "rayEnterDirectionTexture",
                "rayExitPositionTexture",
            ],
            coefficient_map=None,
        )

    def set_opencv_pinhole_properties(
        self,
        cx: float | None = None,
        cy: float | None = None,
        fx: float | None = None,
        fy: float | None = None,
        pinhole: list[float] | None = None,
    ) -> None:
        """Applies OpenCV pinhole distortion model to camera prim, then sets distortion parameters.

        Args:
            cx: Horizontal Render Position (pixels).
            cy: Vertical Render Position (pixels).
            fx: Horizontal Focal Length (pixels).
            fy: Vertical Focal Length (pixels).
            pinhole: OpenCV pinhole parameters [k1, k2, p1, p2, k3, k4, k5, k6, s1, s2, s3, s4].

        Returns:
            None.

        """
        image_size = Gf.Vec2i(self.get_resolution())
        return self._set_lens_distortion_properties(
            distortion_model="OpenCvPinhole",
            distortion_model_attr="opencvPinhole",
            coefficient_map=OPENCV_PINHOLE_ATTRIBUTE_MAP,
            coefficients=pinhole,
            cx=cx,
            cy=cy,
            fx=fx,
            fy=fy,
            image_size=image_size,
        )

    def get_opencv_pinhole_properties(self) -> tuple[float, float, float, float, list]:
        """If camera prim is using OpenCV pinhole distortion model, returns corresponding distortion parameters.

        Returns:
            A tuple containing (cx, cy, fx, fy, OpenCV pinhole parameters [k1, k2, p1, p2, k3, k4, k5, k6,
            s1, s2, s3, s4]).

        """
        return self._get_lens_distortion_properties(
            distortion_model_attr="opencvPinhole",
            attr_names=["cx", "cy", "fx", "fy"],
            coefficient_map=OPENCV_PINHOLE_ATTRIBUTE_MAP,
        )

    def set_opencv_fisheye_properties(
        self,
        cx: float | None = None,
        cy: float | None = None,
        fx: float | None = None,
        fy: float | None = None,
        fisheye: list[float] | None = None,
    ) -> None:
        """Applies OpenCV fisheye distortion model to camera prim, then sets distortion parameters.

        Args:
            cx: Horizontal Render Position (pixels).
            cy: Vertical Render Position (pixels).
            fx: Horizontal Focal Length (pixels).
            fy: Vertical Focal Length (pixels).
            fisheye: OpenCV fisheye parameters [k1, k2, k3, k4].

        Returns:
            None.

        """
        image_size = Gf.Vec2i(self.get_resolution())
        return self._set_lens_distortion_properties(
            distortion_model="OpenCvFisheye",
            distortion_model_attr="opencvFisheye",
            coefficient_map=OPENCV_FISHEYE_ATTRIBUTE_MAP,
            coefficients=fisheye,
            cx=cx,
            cy=cy,
            fx=fx,
            fy=fy,
            image_size=image_size,
        )

    def get_opencv_fisheye_properties(self) -> tuple[float, float, float, float, list]:
        """If camera prim is using OpenCV fisheye distortion model, returns corresponding distortion parameters.

        Returns:
            A tuple containing (cx, cy, fx, fy, OpenCV fisheye parameters [k1, k2, k3, k4]).

        """
        return self._get_lens_distortion_properties(
            distortion_model_attr="opencvFisheye",
            attr_names=["cx", "cy", "fx", "fy"],
            coefficient_map=OPENCV_FISHEYE_ATTRIBUTE_MAP,
        )


def get_all_camera_objects(root_prim: str = "/") -> list[Camera]:
    """Retrieve isaacsim.sensors.camera Camera objects for each camera in the scene.

    Args:
        root_prim: Root prim where the world exists.

    Returns:
        A list of isaacsim.sensors.camera Camera objects

    """
    # Get the paths of prims that are of type "Camera" from scene
    camera_prims = get_all_matching_child_prims(
        prim_path=root_prim, predicate=lambda prim: get_prim_type_name(prim) == "Camera"
    )
    # Filter LiDAR sensors
    camera_prims = [prim for prim in camera_prims if not prim.HasAPI(IsaacRtxLidarSensorAPI)]

    camera_names = []
    for prim in camera_prims:
        camera_path_split = get_prim_path(prim).split("/")
        camera_names.append(camera_path_split[-1])

    # check if camera names are unique, if not, use full camera prim path when naming Camera object
    use_camera_names = len(set(camera_names)) == len(camera_names)

    # Create a "Camera" object for them
    camera_objects = []
    for i, prim in enumerate(camera_prims):
        camera_name = camera_names[i] if use_camera_names else get_prim_path(prim)
        camera = Camera(prim_path=get_prim_path(prim), name=camera_name)
        camera_objects.append(camera)

    return camera_objects
