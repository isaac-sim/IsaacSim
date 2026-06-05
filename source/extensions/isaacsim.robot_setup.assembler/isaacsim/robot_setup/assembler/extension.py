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

"""Extension for robot setup and assembly in Isaac Sim."""

import asyncio
import gc

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

from .global_variables import EXTENSION_TITLE
from .ui.ui_builder import UIBuilder

"""
This file serves as a basic template for the standard boilerplate operations
that make a UI-based extension appear on the toolbar.

This implementation is meant to cover most use-cases without modification.
Various callbacks are hooked up to a seperate class UIBuilder in .ui_builder.py
Most users will be able to make their desired UI extension by interacting solely with
UIBuilder.

This class sets up standard useful callback functions in UIBuilder:
    on_menu_callback: Called when extension is opened
    on_timeline_event: Called when timeline is stopped, paused, or played
    on_physics_step: Called on every physics step
    on_stage_event: Called when stage is opened or closed
    cleanup: Called when resources such as physics subscriptions should be cleaned up
    build_ui: User function that creates the UI they want.
"""


class Extension(omni.ext.IExt):
    """Extension class for the isaacsim.robot_setup.assembler extension.

    This extension provides a UI-based robot setup and assembly tool that integrates with the Isaac Sim robotics
    workflow. It creates a dockable window interface that allows users to configure and assemble robot
    components within the simulation environment.

    The extension automatically handles standard UI extension operations including menu integration, window
    management, and event subscriptions. It provides callback hooks for timeline events (play/stop), physics
    steps, and stage events (open/close) through the UIBuilder component.

    Key features include:
    - Dockable UI window with scrolling capability
    - Menu integration under Tools > Robotics
    - Automatic event subscription management for timeline and stage changes
    - Physics step callback integration
    - Resource cleanup on window close or extension shutdown

    The main user interaction occurs through the UIBuilder class, which handles the actual UI construction
    and robot assembly logic. This extension class serves as the framework that manages the UI lifecycle
    and event handling infrastructure.
    """

    def on_startup(self, ext_id: str) -> None:
        """Initialize extension and UI elements.

        Args:
            ext_id: The extension ID string for this extension.
        """
        # Events
        self._usd_context = omni.usd.get_context()

        # Build Window
        self._window = ScrollingWindow(
            title=EXTENSION_TITLE, width=600, height=500, visible=False, dockPreference=ui.DockPreference.LEFT_BOTTOM
        )
        self._window.set_visibility_changed_fn(self._on_window)

        # UI
        self._models = {}
        self._ext_id = ext_id
        self._ext_name = omni.ext.get_extension_name(ext_id)

        # Register action for menu click
        action_registry = omni.kit.actions.core.get_action_registry()
        action_registry.register_action(
            self._ext_name,
            f"CreateUIExtension:{EXTENSION_TITLE}",
            self._menu_callback,
            description=f"Add {EXTENSION_TITLE} Extension to UI toolbar",
        )

        # Create menu items using registered action
        menu_items = [
            MenuItemDescription(
                name=EXTENSION_TITLE, onclick_action=(self._ext_name, f"CreateUIExtension:{EXTENSION_TITLE}")
            )
        ]
        self._menu_items = [MenuItemDescription(name="Robotics", sub_menu=menu_items)]
        add_menu_items(self._menu_items, "Tools")

        # Filled in with User Functions
        self.ui_builder = UIBuilder()

        # Events
        self._usd_context = omni.usd.get_context()
        self._physics_simulation_interface = omni.physics.core.get_physics_simulation_interface()
        self._physics_subscription = None
        self._stage_event_sub = None
        self._timeline = omni.timeline.get_timeline_interface()

    def on_shutdown(self) -> None:
        """Clean up extension resources when shutting down."""
        self._models = {}
        remove_menu_items(self._menu_items, "Tools")

        # Deregister action
        action_registry = omni.kit.actions.core.get_action_registry()
        action_registry.deregister_action(self._ext_name, f"CreateUIExtension:{EXTENSION_TITLE}")

        if self._window:
            self._window = None
        self.ui_builder.cleanup()
        gc.collect()

    def _on_window(self, visible: bool) -> None:
        """Handle window visibility changes to manage event subscriptions and UI state.

        Args:
            visible: Whether the window is visible.
        """
        if self._window.visible:
            # Subscribe to Stage and Timeline Events
            self._usd_context = omni.usd.get_context()
            self._stage_event_sub_opened = carb.eventdispatcher.get_eventdispatcher().observe_event(
                event_name=self._usd_context.stage_event_name(omni.usd.StageEventType.OPENED),
                on_event=self._on_stage_opened,
                observer_name="isaacsim.robot_setup.assembler.Extension._on_stage_opened",
            )
            self._stage_event_sub_closed = carb.eventdispatcher.get_eventdispatcher().observe_event(
                event_name=self._usd_context.stage_event_name(omni.usd.StageEventType.CLOSED),
                on_event=self._on_stage_closed,
                observer_name="isaacsim.robot_setup.assembler.Extension._on_stage_closed",
            )
            self._stage_event_sub_assets_loaded = carb.eventdispatcher.get_eventdispatcher().observe_event(
                event_name=self._usd_context.stage_event_name(omni.usd.StageEventType.ASSETS_LOADED),
                on_event=self._on_assets_loaded,
                observer_name="isaacsim.robot_setup.assembler.Extension._on_stage_closed",
            )
            self._timeline_event_sub_play = carb.eventdispatcher.get_eventdispatcher().observe_event(
                event_name=omni.timeline.GLOBAL_EVENT_PLAY,
                on_event=self._on_timeline_play,
                observer_name="isaacsim.robot_setup.assembler.Extension._on_timeline_play",
            )
            self._timeline_event_sub_stop = carb.eventdispatcher.get_eventdispatcher().observe_event(
                event_name=omni.timeline.GLOBAL_EVENT_STOP,
                on_event=self._on_timeline_stop,
                observer_name="isaacsim.robot_setup.assembler.Extension._on_timeline_stop",
            )

            self._build_ui()
        else:
            self._usd_context = None
            self._stage_event_sub_opened = None
            self._stage_event_sub_closed = None
            self._stage_event_sub_assets_loaded = None
            self._timeline_event_sub_play = None
            self._timeline_event_sub_stop = None
            self._physics_subscription = None
            self.ui_builder.cleanup()

    def _build_ui(self) -> None:
        """Build the main UI layout and dock the window to the viewport."""
        with self._window.frame:
            with ui.VStack(spacing=5, height=0):
                self._build_extension_ui()

        async def dock_window() -> None:
            await omni.kit.app.get_app().next_update_async()

            def dock(space: object, name: str, location: object, pos: float = 0.5) -> object:
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
        """Handle menu item clicks to toggle window visibility."""
        self._window.visible = not self._window.visible
        self.ui_builder.on_menu_callback()

    def _on_timeline_play(self, event: object) -> None:
        """Handle timeline play events by subscribing to physics step updates.

        Args:
            event: The timeline play event.
        """
        if not self._physics_subscription:
            self._physics_subscription = self._physics_simulation_interface.subscribe_physics_on_step_events(
                pre_step=False, order=0, on_update=self._on_physics_step
            )
        self.ui_builder.on_timeline_event(event)

    def _on_timeline_stop(self, event: object) -> None:
        """Handle timeline stop events by unsubscribing from physics step updates.

        Args:
            event: The timeline stop event.
        """
        self._physics_subscription = None
        self.ui_builder.on_timeline_event(event)

    def _on_physics_step(self, step: float, context: object) -> None:
        """Handle physics step events during simulation.

        Args:
            step: The physics step information.
            context: The physics simulation context.
        """
        self.ui_builder.on_physics_step(step)

    def _on_stage_opened(self, event: object) -> None:
        """Handle stage opened events by cleaning up physics subscriptions and UI state.

        Args:
            event: The stage opened event.
        """
        # stage was opened, cleanup
        self._physics_subscription = None
        self.ui_builder.cleanup()
        self.ui_builder.on_stage_event(event)

    def _on_assets_loaded(self, event: object) -> None:
        """Handle stage assets loaded events.

        Args:
            event: The stage assets loaded event.
        """
        self.ui_builder.on_stage_event(event)

    def _on_stage_closed(self, event: object) -> None:
        """Handles stage closure events by cleaning up resources and notifying the UI builder.

        Args:
            event: The stage closed event data.
        """
        # stage was closed, cleanup
        self._physics_subscription = None
        self.ui_builder.cleanup()
        self.ui_builder.on_stage_event(event)

    def _build_extension_ui(self) -> None:
        """Builds the extension UI by calling the UI builder's build_ui method."""
        # Call user function for building UI
        self.ui_builder.build_ui()
