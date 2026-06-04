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

This module provides the RadarSensor class for operating single RTX-based radar sensor.
"""

from __future__ import annotations

from ._sensor_base import _SensorRuntime
from .radar import Radar


class RadarSensor(_SensorRuntime):
    """Runtime class for operating a single RTX-based radar sensor.

    Wraps a :class:`Radar` authoring object, attaches Replicator annotators,
    and provides :meth:`get_data` to retrieve sensor output at simulation time.

    .. note::

        RTX Radar requires Motion BVH to be enabled. The setting
        ``/renderer/raytracingMotion/enabled`` must be set to ``True`` before creating the radar prim.

    Args:
        path: :class:`Radar` object or single path to an existing or non-existing USD OmniRadar prim.
            If a string path is provided, a :class:`Radar` instance is created internally.
        annotators: Annotator/sensor types to configure.

    Raises:
        ValueError: If no prim is found matching the specified path.
        ValueError: If the input argument refers to more than one prim.
        ValueError: If an unsupported annotator type is specified.
        RuntimeError: If Motion BVH is not enabled when creating a new radar prim.

    Example:

    .. code-block:: python

        >>> import isaacsim.core.experimental.utils.app as app_utils
        >>> from isaacsim.sensors.experimental.rtx import RadarSensor
        >>>
        >>> # given a USD stage with the OmniRadar prim: /World/prim_0
        >>> # and a USD Cube prim: /World/cube
        >>> sensor = RadarSensor(
        ...     "/World/prim_0",
        ...     annotators=["generic-model-output"],
        ... )  # doctest: +NO_CHECK
        >>>
        >>> # play the simulation so the sensor can fetch data
        >>> app_utils.play(commit=True)
    """

    _AUTHORING_CLASS = Radar
    _AUTHORING_ATTR = "_radar"

    @property
    def radar(self) -> Radar:
        """Radar object encapsulated by the sensor.

        Returns:
            Radar object encapsulated by the sensor.

        Example:

        .. code-block:: python

            >>> sensor.radar
            <isaacsim.sensors.experimental.rtx.impl.radar.Radar object at 0x...>
        """
        return self._radar
