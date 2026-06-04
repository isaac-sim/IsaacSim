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

"""RTX camera sensor authoring.

This module provides the RtxCamera class for creating/wrapping USD Camera prims
with the OmniSensorAPI schema applied, enabling tick-rate-controlled rendering.
"""

from __future__ import annotations

from typing import Any

import isaacsim.core.experimental.utils.prim as prim_utils
import isaacsim.core.experimental.utils.stage as stage_utils
import numpy as np
import warp as wp
from isaacsim.core.experimental.objects import Camera
from isaacsim.storage.native import get_assets_root_path
from pxr import UsdGeom

from ._sensor_base import _resolve_config_path, _SensorAuthoring
from .rtx_camera_configs import SUPPORTED_CAMERA_CONFIGS


class RtxCamera(_SensorAuthoring):
    """High level class for creating/wrapping USD Camera prims as RTX sensors.

    Applies the ``OmniSensorAPI`` schema to the underlying ``UsdGeom.Camera`` prim,
    enabling tick-rate-controlled rendering. Optical parameters (focal length,
    clipping range, aperture, etc.) are accessible via the :attr:`camera` property.

    .. note::

        This class creates or wraps (one of both) USD Camera prims according to the following rules:

        * If the prim path exists, a wrapper is placed over the USD Camera prim.
        * If the prim path does not exist, a USD Camera prim is created at the path and a wrapper is placed over it.

    Args:
        path: Single path to existing or non-existing (one of both) USD Camera prim.
            Can include regular expression for matching a prim.
        tick_rate: Sensor tick rate in Hz. When ``None`` (the default), the prim's
            ``omni:sensor:tickRate`` attribute is left untouched, so any value already authored on
            the prim (e.g. from a USD asset) is preserved. For newly-created prims, the
            ``OmniSensorAPI`` schema default of ``0`` Hz applies (autotrigger mode).
        schemas: Additional API schemas to apply to the prim (e.g. ``["OmniLensDistortionOpenCvFisheyeAPI"]``).
            Supports multi-instance schemas via ``"SchemaName:instanceName"`` syntax.
        attributes: Attributes to set on the Camera prim (applied after schemas, so schema-specific
            attributes can be set in the same call).
        positions: Positions in the world frame (shape ``(N, 3)``).
            If the input shape is smaller than expected, data will be broadcasted (following NumPy broadcast rules).
        translations: Translations in the local frame (shape ``(N, 3)``).
            If the input shape is smaller than expected, data will be broadcasted (following NumPy broadcast rules).
        orientations: Orientations in the world frame (shape ``(N, 4)``, quaternion ``wxyz``).
            If the input shape is smaller than expected, data will be broadcasted (following NumPy broadcast rules).
        scales: Scales to be applied to the prims (shape ``(N, 3)``).
            If the input shape is smaller than expected, data will be broadcasted (following NumPy broadcast rules).
        reset_xform_op_properties: Whether to reset the transformation operation attributes of the prims to a standard set.
            See :py:meth:`reset_xform_op_properties` for more details.

    Raises:
        ValueError: If no prim is found matching the specified path.
        ValueError: If the input argument refers to more than one prim.

    Example:

    .. code-block:: python

        >>> from isaacsim.sensors.experimental.rtx import RtxCamera
        >>>
        >>> cam = RtxCamera("/World/cam", tick_rate=30.0)
        >>> cam.camera.set_focal_lengths(24.0)
        >>> cam.camera.set_clipping_ranges(0.1, 100.0)
    """

    _PRIM_TYPE = "Camera"
    _SCHEMA = "OmniSensorAPI"

    def __init__(
        self,
        path: str,
        *,
        tick_rate: float | None = None,
        schemas: list[str] | None = None,
        attributes: dict[str, Any] | None = None,
        positions: list | np.ndarray | wp.array | None = None,
        translations: list | np.ndarray | wp.array | None = None,
        orientations: list | np.ndarray | wp.array | None = None,
        scales: list | np.ndarray | wp.array | None = None,
        reset_xform_op_properties: bool = True,
    ) -> None:
        super().__init__(
            path,
            tick_rate=tick_rate,
            schemas=schemas,
            attributes=attributes,
            positions=positions,
            translations=translations,
            orientations=orientations,
            scales=scales,
            reset_xform_op_properties=reset_xform_op_properties,
        )
        self._camera = None
        # Camera prims don't produce GenericModelOutput — remove the attribute
        # that the base class creates for lidar/radar/acoustic sensors.
        for p in self.paths:
            prim = prim_utils.get_prim_at_path(p)
            prim.RemoveProperty("_replicator:rendervar:GenericModelOutput:channels")

    def _create_prim(self, path: str, attributes: dict[str, Any] | None) -> str:
        """Create a USD Camera prim with the OmniSensorAPI schema applied.

        Args:
            path: USD prim path for the new camera.
            attributes: Optional mapping of attribute names to values.

        Returns:
            The USD prim path of the created camera.
        """
        stage = stage_utils.get_current_stage(backend="usd")
        UsdGeom.Camera.Define(stage, path)
        prim = prim_utils.get_prim_at_path(path)
        prim.ApplyAPI(self._SCHEMA)
        # Attributes are applied by the base class after all schemas
        # (including additional ones passed via the schemas parameter)
        # have been applied.
        return path

    @staticmethod
    def create(
        path: str,
        *,
        tick_rate: float | None = None,
        schemas: list[str] | None = None,
        attributes: dict[str, Any] | None = None,
        positions: list | np.ndarray | wp.array | None = None,
        translations: list | np.ndarray | wp.array | None = None,
        orientations: list | np.ndarray | wp.array | None = None,
        scales: list | np.ndarray | wp.array | None = None,
        reset_xform_op_properties: bool = True,
        config: str | None = None,
        usd_path: str | None = None,
        variant: str | dict[str, str] | None = None,
    ) -> "RtxCamera":
        """Create an RtxCamera instance, optionally from a known config or USD file.

        Args:
            path: Single path to existing or non-existing (one of both) USD Camera prim.
            tick_rate: Sensor tick rate in Hz. When ``None`` (the default), the asset's
                ``omni:sensor:tickRate`` attribute is preserved. Pass an explicit value to override.
            schemas: Additional API schemas to apply to the prim (e.g. ``["OmniLensDistortionOpenCvFisheyeAPI"]``).
                Supports multi-instance schemas via ``"SchemaName:instanceName"`` syntax.
            attributes: Attributes to set on the Camera prim (applied after schemas, so schema-specific
                attributes can be set in the same call).
            positions: Positions in the world frame (shape ``(N, 3)``).
            translations: Translations in the local frame (shape ``(N, 3)``).
            orientations: Orientations in the world frame (shape ``(N, 4)``, quaternion ``wxyz``).
            scales: Scales to be applied to the prims (shape ``(N, 3)``).
            reset_xform_op_properties: Whether to reset the transformation operation attributes of the prims.
            config: Configuration name for the sensor (from ``SUPPORTED_CAMERA_CONFIGS``).
            usd_path: Path to a USD file containing the camera asset.
            variant: Variant name for the camera configuration. Nested variants
                supported via dictionary; pairs applied in dict insertion order,
                so outer variant sets must come first.

        Returns:
            RtxCamera instance.

        Raises:
            ValueError: If both ``config`` and ``usd_path`` are provided.
            ValueError: If the specified ``config`` is not found in :data:`SUPPORTED_CAMERA_CONFIGS`.

        Example:

        .. code-block:: python

            >>> from isaacsim.sensors.experimental.rtx import RtxCamera
            >>>
            >>> cam = RtxCamera.create(path="/World/cam", tick_rate=30.0)
            >>> cam.camera.set_focal_lengths(24.0)
        """
        if config is not None and usd_path is not None:
            raise ValueError("Both 'config' and 'usd_path' cannot be provided")
        if config is not None:
            usd_path = get_assets_root_path() + _resolve_config_path(
                config, SUPPORTED_CAMERA_CONFIGS, sensor_type="Camera"
            )
        asset_root_path = path
        if usd_path is not None:
            path = RtxCamera._create_from_usd(path=path, usd_path=usd_path, variant=variant)
        cam = RtxCamera(
            path=path,
            tick_rate=tick_rate,
            schemas=schemas,
            attributes=attributes,
            positions=positions,
            translations=translations,
            orientations=orientations,
            scales=scales,
            reset_xform_op_properties=reset_xform_op_properties,
        )
        if usd_path is not None:
            cam._asset_root_path = asset_root_path
        return cam

    @property
    def camera(self) -> Camera:
        """Camera object for accessing optical parameters.

        Returns a :class:`~isaacsim.core.experimental.objects.Camera` wrapper over
        the same USD prim, providing access to focal length, clipping range, aperture,
        and other optical properties.

        Returns:
            Camera object wrapping the sensor prim.

        Example:

        .. code-block:: python

            >>> cam = RtxCamera("/World/cam")
            >>> cam.camera.set_focal_lengths(50.0)
            >>> cam.camera.get_focal_lengths()
        """
        if self._camera is None:
            self._camera = Camera(self.paths[0])
        return self._camera
