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

"""Radar sensor for 3D range estimation.

This module provides the Radar class for creating/wrapping USD OmniRadar prims.
"""

from __future__ import annotations

from typing import Any

import carb
import isaacsim.core.experimental.utils.prim as prim_utils
import isaacsim.core.experimental.utils.stage as stage_utils
import numpy as np
import omni.replicator.core as rep
import warp as wp
from isaacsim.storage.native import get_assets_root_path

from ._sensor_base import _resolve_config_path, _SensorAuthoring
from .rtx_radar_configs import SUPPORTED_RADAR_CONFIGS


class Radar(_SensorAuthoring):
    """High level class for creating/wrapping USD OmniRadar prims.

    This class uses ``omni.replicator.core.functional.create.omni_radar`` to create new radar prims,
    which handles defining the prim, applying schemas, and setting attributes.

    .. note::

        RTX Radar requires Motion BVH to be enabled. The setting
        ``/renderer/raytracingMotion/enabled`` must be set to ``True`` before creating a radar prim.

    .. note::

        This class creates or wraps (one of both) USD OmniRadar prims according to the following rules:

        * If the prim path exists, a wrapper is placed over the USD OmniRadar prim.
        * If the prim path does not exist, a USD OmniRadar prim is created at the path and a wrapper is placed over it.

    Args:
        path: Single path to existing or non-existing (one of both) USD OmniRadar prim.
            Can include regular expression for matching a prim.
        aux_output_level: Auxiliary data level for GenericModelOutput. Valid values:
            ``"NONE"`` (default), ``"BASIC"``.
        tick_rate: Sensor tick rate in Hz. A value of ``0`` (the default) enables autotrigger mode.
        attributes: Attributes to set on the OmniRadar prim.
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
        RuntimeError: If Motion BVH is not enabled when creating a new radar prim.

    Example:

    .. code-block:: python

        >>> from isaacsim.sensors.experimental.rtx import Radar
        >>>
        >>> # given a USD stage with the OmniRadar prim: /World/prim_0
        >>> radar = Radar("/World/prim_0")  # doctest: +NO_CHECK
    """

    _PRIM_TYPE = "OmniRadar"
    _SCHEMA = "OmniSensorGenericRadarWpmDmatAPI"
    _VALID_AUX_OUTPUT_LEVELS = ("NONE", "BASIC")

    def _create_prim(self, path: str, attributes: dict[str, Any] | None) -> str:
        """Create an OmniRadar prim via the Replicator functional API.

        Validates that Motion BVH is enabled before creating the prim, since
        RTX Radar requires it for Doppler velocity estimation.

        Args:
            path: USD prim path for the new radar.
            attributes: Optional mapping of attribute names to values, forwarded to
                ``rep.functional.create.omni_radar``.

        Returns:
            The USD prim path of the created radar.

        Raises:
            RuntimeError: If Motion BVH is not enabled.
        """
        settings = carb.settings.get_settings()
        if not settings.get("/renderer/raytracingMotion/enabled"):
            raise RuntimeError(
                "RTX Radar requires Motion BVH to be enabled. "
                "Set '--/renderer/raytracingMotion/enabled=true' when launching Isaac Sim."
            )
        path_parts = path.rsplit("/", 1)
        parent = path_parts[0] if len(path_parts) > 1 and path_parts[0] else None
        name = path_parts[-1]
        # Replicator's parent-valid check runs against the pxr USD stage.
        # ``stage.DefinePrim`` is idempotent: it creates missing ancestors as
        # typeless overs and upgrades only untyped prims to the supplied type.
        if parent is not None:
            stage = stage_utils.get_current_stage(backend="usd")
            if not stage.GetPrimAtPath(parent).IsValid():
                stage.DefinePrim(parent, "Xform")
        prims = rep.functional.create.omni_radar(
            name=name,
            parent=parent,
            **(attributes or {}),
        )
        prim = prims[0] if isinstance(prims, (list, tuple)) else prims
        return prim_utils.get_prim_path(prim)

    @staticmethod
    def create(
        path: str,
        *,
        aux_output_level: str = "NONE",
        tick_rate: float | None = None,
        attributes: dict[str, Any] | None = None,
        positions: list | np.ndarray | wp.array | None = None,
        translations: list | np.ndarray | wp.array | None = None,
        orientations: list | np.ndarray | wp.array | None = None,
        scales: list | np.ndarray | wp.array | None = None,
        reset_xform_op_properties: bool = True,
        config: str | None = None,
        usd_path: str | None = None,
        variant: str | dict[str, str] | None = None,
    ) -> Radar:
        """Create a Radar instance from a config name or USD file path.

        Args:
            path: Single path to existing or non-existing (one of both) USD OmniRadar prim.
            aux_output_level: Auxiliary data level for GenericModelOutput. Valid values:
                ``"NONE"`` (default), ``"BASIC"``.
            tick_rate: Sensor tick rate in Hz. When ``None`` (the default), the asset's
                ``omni:sensor:tickRate`` attribute is preserved. Pass an explicit value to override.
            attributes: Attributes to set on the OmniRadar prim.
            positions: Positions in the world frame (shape ``(N, 3)``).
            translations: Translations in the local frame (shape ``(N, 3)``).
            orientations: Orientations in the world frame (shape ``(N, 4)``, quaternion ``wxyz``).
            scales: Scales to be applied to the prims (shape ``(N, 3)``).
            reset_xform_op_properties: Whether to reset the transformation operation attributes of the prims.
            config: Configuration name for the sensor (from ``SUPPORTED_RADAR_CONFIGS``).
            usd_path: Path to a USD file containing the sensor asset.
            variant: Variant name for the sensor configuration. Nested variants
                supported via dictionary; pairs applied in dict insertion order,
                so outer variant sets must come first.

        Returns:
            Radar instance.

        Raises:
            ValueError: If both 'config' and 'usd_path' are provided.
            ValueError: If the specified config is not found.
            RuntimeError: If Motion BVH is not enabled.

        Example:

        .. code-block:: python

            >>> from isaacsim.sensors.experimental.rtx import Radar
            >>>
            >>> radar = Radar.create(path="/World/radar", config="IWRL6432AOP")
        """
        if config is not None and usd_path is not None:
            raise ValueError("Both 'config' and 'usd_path' cannot be provided")
        if config is not None:
            usd_path = get_assets_root_path() + _resolve_config_path(
                config, SUPPORTED_RADAR_CONFIGS, sensor_type="Radar"
            )
        if usd_path is not None:
            path = Radar._create_from_usd(path=path, usd_path=usd_path, variant=variant)
        return Radar(
            path=path,
            aux_output_level=aux_output_level,
            tick_rate=tick_rate,
            attributes=attributes,
            positions=positions,
            translations=translations,
            orientations=orientations,
            scales=scales,
            reset_xform_op_properties=reset_xform_op_properties,
        )
