# SPDX-FileCopyrightText: Copyright (c) 2022-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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

"""Provides an extension for robot motion generation tutorials with RMP Flow in Isaac Sim."""

from __future__ import annotations

import asyncio
import gc
from typing import Any

import carb.eventdispatcher
import omni
import omni.kit.commands
import omni.physics.core
import omni.timeline
import omni.ui as ui
import omni.usd
from isaacsim.gui.components.element_wrappers import ScrollingWindow
from isaacsim.gui.components.menu import MenuItemDescription
from omni.kit.menu.utils import add_menu_items, remove_menu_items

from .global_variables import EXTENSION_TITLE, MENU_ITEM_NAME, MENU_PARENT_NAME
from .ui_builder import UIBuilder


class Extension(omni.ext.IExt):
    """Extension class for the isaacsim.robot_motion.motion_generation.examples extension.

    This extension provides a UI-based interface for robot motion generation tutorials within Isaac Sim. It creates a
    scrollable window that appears in the Isaac Sim interface, accessible through the menu system. The extension
    integrates with Isaac Sim's timeline and stage events to provide interactive tutorials on robot motion generation
    concepts.

    The extension automatically handles window management, event subscriptions for timeline and stage changes, and
    physics step callbacks. It delegates the actual UI construction and tutorial logic to a UIBuilder instance,
    following a separation of concerns pattern where this class handles the extension infrastructure and UIBuilder
    handles the tutorial-specific functionality.

    Key features include:
    - Automatic window docking to the left side of the viewport
    - Timeline event handling for play/stop simulation states
    - Stage event handling for opened/closed stage states
    - Physics step callbacks for real-time updates during simulation
    - Menu integration for easy access to the extension

    The extension window is hidden by default and becomes visible when accessed through the Isaac Sim menu system.
    When visible, it subscribes to relevant events and builds the tutorial UI. When hidden, it cleans up event
    subscriptions to optimize performance.
    """

    def on_startup(self, ext_id: str) -> None:
        """Initialize extension and UI elements.

        Args:
            ext_id: The extension identifier.
        """
        self.ext_id = ext_id
        self._usd_context = omni.usd.get_context()

        # Build Window
        self._window = ScrollingWindow(
            title=EXTENSION_TITLE, width=600, height=500, visible=False, dockPreference=ui.DockPreference.LEFT_BOTTOM
        )
        self._window.set_visibility_changed_fn(self._on_window)

        action_registry = omni.kit.actions.core.get_action_registry()
        action_registry.register_action(
            ext_id,
            f"CreateUIExtension:{EXTENSION_TITLE}",
            self._menu_callback,
            description=f"Add {EXTENSION_TITLE} Extension to UI toolbar",
        )
        self._menu_items = [
            MenuItemDescription(name=MENU_ITEM_NAME, onclick_action=(ext_id, f"CreateUIExtension:{EXTENSION_TITLE}"))
        ]

        add_menu_items(self._menu_items, MENU_PARENT_NAME)

        # Filled in with User Functions
        self.ui_builder = UIBuilder()

        # Events
        self._usd_context = omni.usd.get_context()
        self._physics_simulation_interface = omni.physics.core.get_physics_simulation_interface()
        self._physics_subscription = None
        self._timeline = omni.timeline.get_timeline_interface()

    def on_shutdown(self) -> None:
        """Clean up extension resources and remove UI elements."""
        self._models = {}
        remove_menu_items(self._menu_items, MENU_PARENT_NAME)

        action_registry = omni.kit.actions.core.get_action_registry()
        action_registry.deregister_action(self.ext_id, f"CreateUIExtension:{EXTENSION_TITLE}")

        if self._window:
            self._window = None
        self.ui_builder.cleanup()
        gc.collect()

    def _on_window(self, visible: bool) -> None:
        """Handle window visibility changes and manage event subscriptions.

        Args:
            visible: Whether the window is visible.
        """
        if self._window.visible:
            # Subscribe to Stage and Timeline Events
            self._usd_context = omni.usd.get_context()
            self._stage_event_sub_opened = carb.eventdispatcher.get_eventdispatcher().observe_event(
                event_name=self._usd_context.stage_event_name(omni.usd.StageEventType.OPENED),
                on_event=self._on_stage_opened,
                observer_name="motion_generation_rmp_flow._on_stage_opened",
            )
            self._stage_event_sub_closed = carb.eventdispatcher.get_eventdispatcher().observe_event(
                event_name=self._usd_context.stage_event_name(omni.usd.StageEventType.CLOSED),
                on_event=self._on_stage_closed,
                observer_name="motion_generation_rmp_flow._on_stage_closed",
            )
            self._timeline_event_sub_play = carb.eventdispatcher.get_eventdispatcher().observe_event(
                event_name=omni.timeline.GLOBAL_EVENT_PLAY,
                on_event=self._on_timeline_play,
                observer_name="motion_generation_rmp_flow._on_timeline_play",
            )
            self._timeline_event_sub_stop = carb.eventdispatcher.get_eventdispatcher().observe_event(
                event_name=omni.timeline.GLOBAL_EVENT_STOP,
                on_event=self._on_timeline_stop,
                observer_name="motion_generation_rmp_flow._on_timeline_stop",
            )

            self._build_ui()
        else:
            self._usd_context = None
            self._stage_event_sub_opened = None
            self._stage_event_sub_closed = None
            self._timeline_event_sub_play = None
            self._timeline_event_sub_stop = None
            self.ui_builder.cleanup()

    def _build_ui(self) -> None:
        """Create the extension UI layout and dock the window."""
        with self._window.frame:
            with ui.VStack(spacing=5, height=0):
                self._build_extension_ui()

        async def dock_window() -> None:
            await omni.kit.app.get_app().next_update_async()

            def dock(space: Any, name: Any, location: Any, pos: Any = 0.5) -> Any:
                window = omni.ui.Workspace.get_window(name)
                if window and space:
                    window.dock_in(space, location, pos)
                return window

            tgt = ui.Workspace.get_window("Viewport")
            dock(tgt, EXTENSION_TITLE, omni.ui.DockPosition.LEFT, 0.33)
            await omni.kit.app.get_app().next_update_async()

        self._task = asyncio.ensure_future(dock_window())

    #################################################################
    # Functions below this point call user functions
    #################################################################

    def _menu_callback(self) -> None:
        """Toggle window visibility and notify the UI builder when menu item is selected."""
        self._window.visible = not self._window.visible
        self.ui_builder.on_menu_callback()

    def _on_timeline_play(self, event: object) -> None:
        """Timeline play event callback.

        Args:
            event: The timeline play event.
        """
        if not self._physics_subscription:
            self._physics_subscription = self._physics_simulation_interface.subscribe_physics_on_step_events(
                pre_step=False, order=0, on_update=self._on_physics_step
            )

    def _on_timeline_stop(self, event: object) -> None:
        """Timeline stop event callback.

        Args:
            event: The timeline stop event.
        """
        self._physics_subscription = None
        self.ui_builder.on_timeline_event(event)

    def _on_physics_step(self, step: float, context: object) -> None:
        """Forward physics step events to the UI builder.

        Args:
            step: The physics step information.
            context: The physics context.
        """
        self.ui_builder.on_physics_step(step)

    def _on_stage_opened(self, event: object) -> None:
        """Stage opened event callback.

        Args:
            event: The stage opened event.
        """
        self._physics_subscription = None
        self.ui_builder.cleanup()
        self.ui_builder.on_stage_event(event)

    def _on_stage_closed(self, event: object) -> None:
        """Stage closed event callback.

        Args:
            event: The stage closed event.
        """
        self._physics_subscription = None
        self.ui_builder.cleanup()

    def _build_extension_ui(self) -> None:
        """Builds the extension UI by calling the user-defined build_ui function."""
        # Call user function for building UI
        self.ui_builder.build_ui()
