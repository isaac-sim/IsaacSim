# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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

"""Viewport gesture helpers for overlay menu interactions."""

from typing import Any

from omni.kit.viewport.utility import disable_context_menu, disable_selection, get_active_viewport_window
from omni.ui import scene as sc

from .overlay_menu import OverlayMenu


class PreventOthers(sc.GestureManager):
    """Gesture manager that prevents other gestures from interrupting.

    Allows this gesture to proceed while blocking conflicting gestures
    that are in BEGAN or CHANGED states.
    """

    def __init__(self) -> None:
        super().__init__()

    def can_be_prevented(self, gesture: Any) -> bool:
        """Check if this gesture can be prevented by others.

        Args:
            gesture: The gesture attempting to prevent this one.

        Returns:
            Always returns True, allowing prevention checks.
        """
        return True

    def should_prevent(self, gesture: Any, preventer: Any) -> bool:
        """Determine if another gesture should be prevented.

        Args:
            gesture: The gesture being evaluated.
            preventer: The gesture attempting to take precedence.

        Returns:
            True if the preventer is in an active state.
        """
        if preventer.state == sc.GestureState.BEGAN or preventer.state == sc.GestureState.CHANGED:
            return True
        return super().should_prevent(gesture, preventer)


class OverlayMenuClick(sc.ClickGesture):
    """Click gesture handler for overlay menu activation.

    Handles click events on overlay circles to show a context menu
    for selecting from multiple joints at the same screen position.

    Args:
        connection: The connection item associated with this overlay.
        *args: Additional positional arguments for the base click gesture.
        **kwargs: Additional keyword arguments for the base click gesture.
    """

    def __init__(self, connection: Any, *args: Any, **kwargs: Any) -> None:
        super().__init__(*args, **kwargs)
        self._connection = connection
        self._context_menu_disabled = None
        self._viewport_disabled = None

    def on_began(self) -> None:
        """Handle the start of a click gesture.

        Disables viewport context menu and selection to prevent
        interference with the overlay menu interaction.
        """
        viewport_window = get_active_viewport_window()
        self._context_menu_disabled = disable_context_menu(viewport_window)
        self._viewport_disabled = disable_selection(viewport_window)

    def on_ended(self, *args: Any) -> None:
        """Handle the end of a click gesture.

        Shows the overlay menu and re-enables viewport interactions.

        Args:
            *args: Click end callback arguments (unused).
        """
        OverlayMenu.show_menu(self._connection)
        self._cleanup_disabled_states()

    def _cleanup_disabled_states(self) -> None:
        """Re-enable viewport context menu and selection."""
        if self._context_menu_disabled:
            del self._context_menu_disabled
            self._context_menu_disabled = None
        if self._viewport_disabled:
            del self._viewport_disabled
            self._viewport_disabled = None
