# SPDX-FileCopyrightText: Copyright (c) 2024-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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

"""High level class for creating, wrapping and operating single camera sensors with configurable annotators."""

from __future__ import annotations

from typing import Any, Literal

import carb
import omni.replicator.core as rep
import warp as wp

from ._camera_common import CAMERA_ANNOTATOR_SPEC
from ._sensor_base import _SensorRuntime
from .rtx_camera import RtxCamera

ANNOTATOR = Literal[
    "bounding_box_2d_loose",
    "bounding_box_2d_tight",
    "bounding_box_3d",
    "distance_to_camera",
    "distance_to_image_plane",
    "instance_id_segmentation",
    "instance_segmentation",
    "motion_vectors",
    "normals",
    "pointcloud",
    "rgb",
    "rgba",
    "semantic_segmentation",
]

# Annotators that must be attached on the host (CPU) Replicator pipeline.
# - Bounding-box annotators have no GPU implementation.
# - Single-view depth sensor render vars (DepthSensor*) must route through
#   SdPostRenderVarToHost; the device-buffer node SdPostRenderVarTextureToBuffer
#   logs "corrupted input renderVar" every frame for them.
_CPU_ANNOTATORS = frozenset(
    {
        "bounding_box_2d_tight",
        "bounding_box_2d_loose",
        "bounding_box_3d",
        "depth_sensor_distance",
        "depth_sensor_imager",
        "depth_sensor_point_cloud_color",
        "depth_sensor_point_cloud_position",
    }
)

# Annotators that return non-image data (no reshape to resolution).
_PASSTHROUGH_ANNOTATORS = frozenset({"bounding_box_2d_tight", "bounding_box_2d_loose", "bounding_box_3d", "pointcloud"})


class CameraSensor(_SensorRuntime):
    """High level class for creating/wrapping and operating single camera sensor.

    Args:
        path: :class:`RtxCamera` object or single path to existing or non-existing USD Camera prim.
            If a string path is provided, a :class:`RtxCamera` instance is created internally.
        resolution: Resolution of the sensor (following OpenCV/NumPy convention: ``(height, width)``).
        annotators: Annotator/sensor types to configure.

    Raises:
        ValueError: If no prim is found matching the specified path.
        ValueError: If the input argument refers to more than one camera prim.
        ValueError: If an unsupported annotator type is specified.

    Example:

    .. code-block:: python

        >>> import isaacsim.core.experimental.utils.app as app_utils
        >>> from isaacsim.sensors.experimental.rtx import CameraSensor
        >>>
        >>> # given a USD stage with the Camera prim: /World/prim_0
        >>> resolution = (240, 320)  # following OpenCV/NumPy convention `(height, width)`
        >>> camera_sensor = CameraSensor(
        ...     "/World/prim_0",
        ...     resolution=resolution,
        ...     annotators=["rgb", "distance_to_image_plane"],
        ... )  # doctest: +NO_CHECK
        >>>
        >>> # play the simulation so the sensor can fetch data
        >>> app_utils.play(commit=True)
    """

    _AUTHORING_CLASS = RtxCamera
    _AUTHORING_ATTR = "_rtx_camera"

    def __init__(
        self,
        path: str | RtxCamera,
        *,
        resolution: tuple[int, int],
        annotators: ANNOTATOR | list[ANNOTATOR] | None = None,
        writers: str | list[str] | None = None,
        render_vars: list[str] | None = None,
    ) -> None:
        self._resolution = resolution
        # Set camera-specific annotator spec before super().__init__ validates annotators.
        # Subclasses (e.g. SingleViewDepthCameraSensor) may set this first with an extended spec.
        if not hasattr(self, "_annotators_spec"):
            self._annotators_spec = {k: v for k, v in CAMERA_ANNOTATOR_SPEC.items() if k in ANNOTATOR.__args__}
        super().__init__(path, annotators=annotators, writers=writers, render_vars=render_vars)
        # Enforce square pixels on the underlying Camera prim
        self.authoring_object.camera.enforce_square_pixels(self._resolution, modes="horizontal")

    @property
    def camera(self):
        """Camera object for accessing optical parameters.

        Returns:
            Camera object wrapping the sensor prim.
        """
        return self.authoring_object.camera

    @property
    def resolution(self) -> tuple[int, int]:
        """Resolution of the sensor.

        Returns:
            Resolution of sensor frames (following OpenCV/NumPy convention: ``(height, width)``).
        """
        return self._resolution

    def attach_annotators(self, annotators: str | list[str]) -> None:
        """Attach annotators to the sensor.

        Args:
            annotators: Annotator/sensor types to attach.

        Raises:
            ValueError: If the specified annotator is not supported.
        """
        annotators = [annotators] if isinstance(annotators, str) else annotators
        self._validate_annotators(annotators)
        for annotator in annotators:
            spec = self._get_annotator_spec(annotator)
            device = "cpu" if annotator in _CPU_ANNOTATORS else "cuda"
            self._annotators[annotator] = rep.AnnotatorRegistry.get_annotator(
                spec["name"], device=device, do_array_copy=False
            )
        for annotator in annotators:
            self._annotators[annotator].attach(self._hydra_texture.path)

    def get_data(self, annotator: str, *, out: wp.array | None = None) -> tuple[wp.array | None, dict[str, Any]]:
        """Fetch the specified annotator/sensor data for the camera.

        Args:
            annotator: Annotator/sensor type from which fetch the data.
            out: Pre-allocated array to fill with the fetched data.

        Returns:
            Two-elements tuple. 1) Array containing the fetched data.
            If no data is available at the moment of calling the method, ``None`` is returned.
            2) Dictionary containing additional information according to the requested annotator/sensor.

        Raises:
            ValueError: If the specified annotator is not supported.
            ValueError: If the specified annotator is not configured.
        """
        self._validate_annotators(annotator)
        if annotator not in self._annotators:
            raise ValueError(f"The annotator '{annotator}' was not configured. Enable it when instantiating the class")
        data = self._annotators[annotator].get_data(device=str(out.device) if out is not None else "cuda")
        if isinstance(data, dict):
            info = data["info"]
            data = data["data"]
        else:
            info = {}
        if data is None or not data.shape[0]:
            return None, {}
        if annotator in _PASSTHROUGH_ANNOTATORS:
            info["resolution"] = self._resolution
            return data, info
        spec = self._get_annotator_spec(annotator)
        input_channels = spec["channels"]
        output_channels = spec.get("output_channels", input_channels)
        # Annotators attached on the host Replicator pipeline may return data
        # as a numpy.ndarray when a CUDA device is requested. Promote to a
        # Warp array on the requested device so reshape/slice/wp.copy work
        # uniformly regardless of attach device.
        if not isinstance(data, wp.array):
            target_device = out.device if out is not None else "cuda"
            data = wp.array(data, dtype=spec["dtype"], device=target_device)
        data = data.reshape((*self._resolution, input_channels))
        if out is None:
            out = data[:, :, :output_channels] if "output_channels" in spec else data
        else:
            wp.copy(out, data[:, :, :output_channels] if "output_channels" in spec else data)
        return out, info

    def _initialize_sensor(self, annotators: str | list[str], *, render_vars: list[str] | None = None) -> None:
        """Initialize sensor by creating a resolution-aware render product and attaching annotators."""
        self._hydra_texture = rep.create.render_product(
            camera=self.authoring_object.paths[0],
            resolution=(self._resolution[1], self._resolution[0]),  # (width, height)
            name=f"camera_sensor_{hash(self)}",
            render_vars=render_vars,
        )
        self.attach_annotators(annotators)
