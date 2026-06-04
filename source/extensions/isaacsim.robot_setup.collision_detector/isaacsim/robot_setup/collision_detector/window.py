# SPDX-FileCopyrightText: Copyright (c) 2025-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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

"""Window shell for the Robot Self-Collision Detector."""

__all__ = ["RobotSelfCollisionWindow"]

from collections.abc import Callable

import omni.ui as ui

from .widget import CollisionDetectorWidget


class RobotSelfCollisionWindow(ui.Window):
    """Dockable window wrapping :class:`CollisionDetectorWidget`.

    Args:
        usd_context_name: Name of the USD context. Empty string uses the default.
    """

    WINDOW_NAME = "Robot Self-Collision Detector"
    """WINDOW_NAME (str): Display title for the dockable window."""

    def __init__(self, usd_context_name: str = "") -> None:
        self._visibility_changed_listener: Callable | None = None
        self._widget: CollisionDetectorWidget | None = None

        super().__init__(
            self.WINDOW_NAME,
            width=600,
            height=400,
            flags=ui.WINDOW_FLAGS_NO_SCROLLBAR,
            dockPreference=ui.DockPreference.LEFT_BOTTOM,
        )

        self.set_visibility_changed_fn(self._on_visibility_changed)
        self.set_width_changed_fn(lambda _: self._on_resized())
        self.set_height_changed_fn(lambda _: self._on_resized())
        self.deferred_dock_in("Content", ui.DockPolicy.CURRENT_WINDOW_IS_ACTIVE)
        self.dock_order = 1

        with self.frame:
            self._widget = CollisionDetectorWidget(usd_context_name)

    def _on_resized(self) -> None:
        """Redraw the collision-pair table after the window is resized."""
        if self._widget:
            self._widget.invalidate_tree()

    def _on_visibility_changed(self, visible: bool) -> None:
        """Forward visibility changes to the listener and the widget.

        Args:
            visible: Whether the window is now visible.
        """
        if self._visibility_changed_listener:
            self._visibility_changed_listener(visible)
        if self._widget:
            self._widget.on_visibility_changed(visible)

    def set_visibility_changed_listener(self, listener: Callable | None) -> None:
        """Register an external callback for window visibility changes.

        Args:
            listener: Callback receiving a boolean visibility flag, or None to clear.
        """
        self._visibility_changed_listener = listener

    def destroy(self) -> None:
        """Release the widget and all held references."""
        self.set_visibility_changed_fn(None)
        self.set_width_changed_fn(None)
        self.set_height_changed_fn(None)
        self._visibility_changed_listener = None
        if self._widget:
            self._widget.destroy()
            self._widget = None
        super().destroy()
