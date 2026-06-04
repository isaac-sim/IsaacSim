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

"""Viewport scene utilities for joint connection visualization."""

from typing import Any

import carb.events
import carb.settings
import omni.timeline
from omni.ui import scene as sc

from .manipulator import ConnectionManipulator
from .model import ConnectionModel


class ConnectionScene:  # pragma: no cover
    """Viewport scene manager for robot joint connection visualization.

    Manages the lifecycle of the connection manipulator and responds to
    timeline and settings changes to control visibility of joint connections.

    Args:
        icon_scale: Scale factor for icons passed to the manipulator.
        **kwargs: Additional keyword arguments passed to the parent class.
    """

    def __init__(self, icon_scale: float = 1.0, **kwargs: Any) -> None:
        self._manipulator: ConnectionManipulator | None = ConnectionManipulator(
            model=ConnectionInstance.get_instance().get_model(),
            aspect_ratio_policy=sc.AspectRatioPolicy.PRESERVE_ASPECT_HORIZONTAL,
            icon_scale=icon_scale,
        )
        self._icon_scale = icon_scale
        self._settings_subscription = carb.settings.get_settings().subscribe_to_node_change_events(
            "/persistent/physics/visualizationDisplayJoints", self._on_settings_changed
        )
        self.visible = carb.settings.get_settings().get("/persistent/physics/visualizationDisplayJoints")

        timeline = omni.timeline.get_timeline_interface()
        event_stream = timeline.get_timeline_event_stream()
        self._timeline_subscription = event_stream.create_subscription_to_pop(
            self._on_timeline_event, name="timeline_subscription"
        )

    def _on_timeline_event(self, event: carb.events.IEvent) -> None:
        """Handle timeline events to control visibility during playback.

        Hides joint connections during simulation playback and restores
        visibility based on settings when stopped.

        Args:
            event: The timeline event.
        """
        event_type = omni.timeline.TimelineEventType(event.type)
        if event_type == omni.timeline.TimelineEventType.PLAY:
            self.visible = False
        elif event_type == omni.timeline.TimelineEventType.STOP:
            self.visible = carb.settings.get_settings().get("/persistent/physics/visualizationDisplayJoints")
            model = ConnectionInstance.get_instance().get_model()
            if model:
                model.force_rebuild()

    def _on_settings_changed(self, *args: Any) -> None:
        """Handle changes to the joint visualization setting.

        Args:
            *args: Setting change callback arguments (unused).
        """
        self.visible = carb.settings.get_settings().get("/persistent/physics/visualizationDisplayJoints")

    def set_joint_connections(self, joint_connections: list[Any]) -> None:
        """Set the joint connections to visualize.

        Args:
            joint_connections: List of connection items.

        Returns:
            None.

        Example:

        .. code-block:: python

            scene.set_joint_connections(connections)
        """
        if self._manipulator is None:
            return
        self._manipulator.set_joint_connections(joint_connections)

    @property
    def visible(self) -> bool:
        """Return the visibility state of the connection manipulator.

        Returns:
            True if the manipulator is visible.

        Example:

        .. code-block:: python

            is_visible = scene.visible
        """
        if self._manipulator is None:
            return False
        return self._manipulator.visible

    @visible.setter
    def visible(self, value: bool) -> None:
        """Set the visibility state of the connection manipulator.

        Args:
            value: True to show, False to hide.

        Returns:
            None.

        Example:

        .. code-block:: python

            scene.visible = True
        """
        if self._manipulator is None:
            return
        self._manipulator.visible = bool(value)

    def destroy(self) -> None:
        """Clean up resources before destruction.

        Unsubscribes from timeline and settings, clears connections and releases references.
        """
        if self._settings_subscription is not None:
            try:
                self._settings_subscription.unsubscribe()
            except Exception:
                pass
            self._settings_subscription = None
        if self._timeline_subscription is not None:
            try:
                self._timeline_subscription.unsubscribe()
            except Exception:
                pass
            self._timeline_subscription = None
        self.clear()
        self._manipulator = None

    def clear(self) -> None:
        """Clear all connection visualizations.

        Returns:
            None.

        Example:

        .. code-block:: python

            scene.clear()
        """
        if not self._manipulator:
            return
        self._manipulator.clear()

    def __del__(self) -> None:
        """Destructor ensuring cleanup."""
        self.destroy()
        self._manipulator = None
        self._timeline_subscription = None
        self._settings_subscription = None


class ConnectionInstance:
    """Singleton providing access to the shared connection model.

    Ensures a single connection model is shared across all
    components that need to access joint connection state.

    Args:
        test: Reserved for testing purposes (unused).
    """

    _instance = None
    """The singleton instance of the connection manager."""

    def __init__(self, test: bool = False) -> None:
        self.model: ConnectionModel | None = ConnectionModel()

    @staticmethod
    def get_instance() -> "ConnectionInstance":
        """Return the singleton connection instance.

        Returns:
            The shared instance.

        Example:

        .. code-block:: python

            instance = ConnectionInstance.get_instance()
        """
        if not ConnectionInstance._instance:
            ConnectionInstance._instance = ConnectionInstance()
        return ConnectionInstance._instance

    def destroy(self) -> None:
        """Destroy the singleton instance and release resources.

        Revokes the connection model's USD listener so no stage callbacks persist.
        """
        if self.model is not None:
            self.model.destroy()
            self.model = None
        ConnectionInstance._instance = None

    def get_model(self) -> ConnectionModel | None:
        """Return the connection model managed by this instance.

        Returns:
            The shared connection model.

        Example:

        .. code-block:: python

            model = ConnectionInstance.get_instance().get_model()
        """
        return self.model

    def set_joint_connections(self, joint_connections: list[Any]) -> None:
        """Set joint connections on the model.

        Args:
            joint_connections: List of connection items.

        Returns:
            None.

        Example:

        .. code-block:: python

            ConnectionInstance.get_instance().set_joint_connections(connections)
        """
        if self.model is None:
            return
        self.model.set_joint_connections(joint_connections)
