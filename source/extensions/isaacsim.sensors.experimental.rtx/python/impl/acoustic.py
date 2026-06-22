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

"""Acoustic sensor for ultrasonic wave detection.

This module provides the Acoustic class for creating/wrapping USD OmniAcoustic prims.
"""

from __future__ import annotations

from typing import Any

import isaacsim.core.experimental.utils.stage as stage_utils
import numpy as np
import warp as wp
from isaacsim.storage.native import get_assets_root_path

from ._sensor_base import _resolve_config_path, _SensorAuthoring
from .rtx_acoustic_configs import SUPPORTED_ACOUSTIC_CONFIGS


class Acoustic(_SensorAuthoring):
    """High level class for creating/wrapping USD OmniAcoustic prims.

    .. note::

        This class creates or wraps (one of both) USD OmniAcoustic prims according to the following rules:

        * If the prim path exists, a wrapper is placed over the USD OmniAcoustic prim.
        * If the prim path does not exist, a USD OmniAcoustic prim is created at the path and a wrapper is placed over it.

    Args:
        path: Single path to existing or non-existing (one of both) USD OmniAcoustic prim.
            Can include regular expression for matching a prim.
        aux_output_level: Auxiliary data level for GenericModelOutput. Valid values:
            ``"NONE"`` (default), ``"BASIC"``.
        tick_rate: Sensor tick rate in Hz. A value of ``0`` (the default) enables autotrigger mode.
        attributes: Attributes to set on the OmniAcoustic prim.
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

        >>> from isaacsim.sensors.experimental.rtx import Acoustic
        >>>
        >>> # given a USD stage with the OmniAcoustic prim: /World/prim_0
        >>> acoustic = Acoustic("/World/prim_0")  # doctest: +NO_CHECK
    """

    _PRIM_TYPE = "OmniAcoustic"
    _SCHEMA = "OmniSensorGenericAcousticWpmAPI"
    _VALID_AUX_OUTPUT_LEVELS = ("NONE", "BASIC")

    # Mapping from attribute prefix to multi-apply API schema name.
    _MULTI_APPLY_SCHEMAS = {
        "omni:sensor:WpmAcoustic:sensorMount:": "OmniSensorWpmAcousticSensorMountAPI",
        "omni:sensor:WpmAcoustic:rxGroup:": "OmniSensorWpmAcousticRxGroupAPI",
    }

    def _create_prim(self, path: str, attributes: dict[str, Any] | None) -> str:
        """Create an OmniAcoustic prim with multi-instance schemas auto-applied.

        Applies ``OmniSensorGenericAcousticWpmAPI`` and automatically infers
        multi-instance schemas (sensor mount, receiver group) from attribute
        key prefixes.

        Args:
            path: USD prim path for the new acoustic sensor.
            attributes: Optional mapping of attribute names to values. Keys
                matching ``omni:sensor:WpmAcoustic:sensorMount:`` or
                ``omni:sensor:WpmAcoustic:rxGroup:`` prefixes trigger automatic
                multi-instance schema application.

        Returns:
            The USD prim path of the created acoustic sensor.
        """
        prim = stage_utils.define_prim(path, "OmniAcoustic")
        prim.ApplyAPI("OmniSensorGenericAcousticWpmAPI")
        if attributes is not None:
            # apply multi-instance schemas inferred from attribute keys
            applied_instances: set[str] = set()
            for key in attributes:
                for prefix, schema in self._MULTI_APPLY_SCHEMAS.items():
                    if key.startswith(prefix):
                        instance_name = key[len(prefix) :].split(":")[0]
                        schema_instance = f"{schema}:{instance_name}"
                        if schema_instance not in applied_instances:
                            prim.ApplyAPI(schema, instance_name)
                            applied_instances.add(schema_instance)
        # Attributes are applied by the base class after all schemas
        # (including additional ones passed via the schemas parameter)
        # have been applied.
        return path

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
    ) -> Acoustic:
        """Create an Acoustic instance from a config name or USD file path.

        Args:
            path: Single path to existing or non-existing (one of both) USD OmniAcoustic prim.
            aux_output_level: Auxiliary data level for GenericModelOutput. Valid values:
                ``"NONE"`` (default), ``"BASIC"``.
            tick_rate: Sensor tick rate in Hz. When ``None`` (the default), the asset's
                ``omni:sensor:tickRate`` attribute is preserved. Pass an explicit value to override.
            attributes: Attributes to set on the OmniAcoustic prim.
            positions: Positions in the world frame (shape ``(N, 3)``).
            translations: Translations in the local frame (shape ``(N, 3)``).
            orientations: Orientations in the world frame (shape ``(N, 4)``, quaternion ``wxyz``).
            scales: Scales to be applied to the prims (shape ``(N, 3)``).
            reset_xform_op_properties: Whether to reset the transformation operation attributes of the prims.
            config: Configuration name for the sensor (from ``SUPPORTED_ACOUSTIC_CONFIGS``).
            usd_path: Path to a USD file containing the sensor asset.
            variant: Variant name for the sensor configuration. Nested variants
                supported via dictionary; pairs applied in dict insertion order,
                so outer variant sets must come first.

        Returns:
            Acoustic instance.

        Raises:
            ValueError: If both 'config' and 'usd_path' are provided.
            ValueError: If the specified config is not found.

        Example:

        .. code-block:: python

            >>> from isaacsim.sensors.experimental.rtx import Acoustic
            >>>
            >>> acoustic = Acoustic.create(path="/World/acoustic")
        """
        if config is not None and usd_path is not None:
            raise ValueError("Both 'config' and 'usd_path' cannot be provided")
        if config is not None:
            usd_path = get_assets_root_path() + _resolve_config_path(
                config, SUPPORTED_ACOUSTIC_CONFIGS, sensor_type="Acoustic"
            )
        if usd_path is not None:
            path = Acoustic._create_from_usd(path=path, usd_path=usd_path, variant=variant)
        return Acoustic(
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
