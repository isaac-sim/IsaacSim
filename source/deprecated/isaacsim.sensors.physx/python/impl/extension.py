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

"""Extension for physics-based sensor simulation capabilities in Isaac Sim including proximity sensors, LiDAR sensors, generic range sensors, and lightbeam sensors."""

import omni.ext
import omni.usd
from omni.physics.core import get_physics_simulation_interface

from .. import _range_sensor
from .proximity_sensor import ProximitySensorManager, clear_sensors


class Extension(omni.ext.IExt):
    """Extension class for the isaacsim.sensors.physx extension.

    This extension provides physics-based sensor simulation capabilities in Isaac Sim, including proximity sensors,
    LiDAR sensors, generic range sensors, and lightbeam sensors. It integrates with the physics simulation step to
    update sensor data in real-time during physics simulation.

    The extension manages sensor interfaces and subscribes to physics step events to ensure sensors are updated
    synchronously with the physics simulation. It provides a centralized proximity sensor manager and handles
    the lifecycle of various range sensor interfaces including LiDAR, generic range sensors, and lightbeam sensors.
    """

    def on_startup(self) -> None:
        """Initializes the extension by setting up physics step subscriptions and sensor managers.

        Subscribes to physics step events to update sensors on each physics step, creates a proximity
        sensor manager instance, and acquires interfaces for lidar, generic, and lightbeam sensors.
        """
        # step the sensor every physics step
        self._physics_step_subscription = get_physics_simulation_interface().subscribe_physics_on_step_events(
            pre_step=False, order=0, on_update=self._on_update
        )
        self._proximity_sensor_manager = ProximitySensorManager()  # Store instance to sensor manager singleton
        self._lidar = _range_sensor.acquire_lidar_sensor_interface()
        self._generic = _range_sensor.acquire_generic_sensor_interface()
        self._lightbeam = _range_sensor.acquire_lightbeam_sensor_interface()

    def on_shutdown(self) -> None:
        """Cleans up the extension by releasing resources and clearing subscriptions.

        Clears the physics step subscription, clears all proximity sensors, releases the proximity
        sensor manager instance, and releases all acquired sensor interfaces.
        """
        # Handle ProximitySensorManager
        self._physics_step_subscription = None  # clear subscription
        clear_sensors()  # clear sensors on shutdown
        self._proximity_sensor_manager = None

        # Release sensor interfaces
        _range_sensor.release_lidar_sensor_interface(self._lidar)
        _range_sensor.release_generic_sensor_interface(self._generic)
        _range_sensor.release_lightbeam_sensor_interface(self._lightbeam)

    def _on_update(self, dt: float, context: object) -> None:
        """Updates the proximity sensor manager on each physics step.

        Args:
            dt: Time delta since the last update.
            context: Physics simulation context.
        """
        self._proximity_sensor_manager.update()
