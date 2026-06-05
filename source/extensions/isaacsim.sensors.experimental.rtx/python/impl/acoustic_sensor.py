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

This module provides the AcousticSensor class for operating single RTX-based acoustic sensor.
"""

from __future__ import annotations

from ._sensor_base import _SensorRuntime
from .acoustic import Acoustic


class AcousticSensor(_SensorRuntime):
    """Runtime class for operating a single RTX-based acoustic sensor.

    Wraps an :class:`Acoustic` authoring object, attaches Replicator annotators,
    and provides :meth:`get_data` to retrieve sensor output at simulation time.

    Args:
        path: :class:`Acoustic` object or single path to an existing or non-existing USD OmniAcoustic prim.
            If a string path is provided, an :class:`Acoustic` instance is created internally.
        annotators: Annotator/sensor types to configure.

    Raises:
        ValueError: If no prim is found matching the specified path.
        ValueError: If the input argument refers to more than one prim.
        ValueError: If an unsupported annotator type is specified.

    Example:

    .. code-block:: python

        >>> import isaacsim.core.experimental.utils.app as app_utils
        >>> from isaacsim.sensors.experimental.rtx import Acoustic, AcousticSensor
        >>>
        >>> # given a USD stage with the OmniAcoustic prim: /World/prim_0
        >>> # and a USD Cube prim: /World/cube
        >>> acoustic = Acoustic("/World/prim_0", aux_output_level="BASIC")
        >>> sensor = AcousticSensor(
        ...     acoustic,
        ...     annotators=["generic-model-output"],
        ... )  # doctest: +NO_CHECK
        >>>
        >>> # play the simulation so the sensor can fetch data
        >>> app_utils.play(commit=True)
    """

    _AUTHORING_CLASS = Acoustic
    _AUTHORING_ATTR = "_acoustic"

    @property
    def acoustic(self) -> Acoustic:
        """Acoustic object encapsulated by the sensor.

        Returns:
            Acoustic object encapsulated by the sensor.

        Example:

        .. code-block:: python

            >>> sensor.acoustic
            <isaacsim.sensors.experimental.rtx.impl.acoustic.Acoustic object at 0x...>
        """
        return self._acoustic
