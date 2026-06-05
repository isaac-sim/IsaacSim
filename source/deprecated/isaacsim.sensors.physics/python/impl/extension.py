# SPDX-FileCopyrightText: Copyright (c) 2018-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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

"""Extension module for Isaac Sim physics sensors, providing access to contact sensors and IMU sensors."""

__all__ = []

import gc

import omni

from .. import _sensor

EXTENSION_NAME = "Isaac Sensor"


class Extension(omni.ext.IExt):
    """Extension for Isaac Sim physics sensors.

    Provides access to physics-based sensors including contact sensors and IMU sensors through
    the Isaac Sim sensor interfaces. This extension manages the initialization and cleanup of
    sensor interfaces required for physics simulation in Isaac Sim.
    """

    def on_startup(self, ext_id: str) -> None:
        """Initialize the Isaac Sensor extension.

        Acquires the contact sensor and IMU sensor interfaces for use by the extension.

        Args:
            ext_id: The extension identifier.
        """
        self._cs = _sensor.acquire_contact_sensor_interface()
        self._is = _sensor.acquire_imu_sensor_interface()

    def on_shutdown(self) -> None:
        """Clean up the Isaac Sensor extension.

        Releases the contact sensor and IMU sensor interfaces and performs garbage collection.
        """
        _sensor.release_contact_sensor_interface(self._cs)
        _sensor.release_imu_sensor_interface(self._is)

        gc.collect()
