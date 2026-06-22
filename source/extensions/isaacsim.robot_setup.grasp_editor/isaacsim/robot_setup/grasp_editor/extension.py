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

"""Extension for creating a UI-based grasp editor interface in Isaac Sim."""

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
from .ui_builder import UIBuilder


class Extension(omni.ext.IExt):
    """Extension class for the isaacsim.robot_setup.grasp_editor extension.

    This extension provides a UI-based interface for editing robot grasp configurations in Isaac Sim.
    It creates a dockable window in the Tools menu under Robotics that allows users to interact
    with grasp editing functionality through a scrollable interface.

    The extension automatically handles standard UI extension operations including:
    - Menu registration and window management
    - Timeline and physics event subscriptions
    - Stage event handling for opened, closed, and assets loaded states
    - Physics step callbacks during simulation
    - Automatic window docking to the left side of the viewport

    The actual UI functionality is delegated to a UIBuilder instance, which can be customized
    to provide specific grasp editing controls and workflows. The extension serves as a template
    for UI-based extensions with standard boilerplate operations for toolbar integration.
    """

    def on_startup(self, ext_id: str) -> None:
        """Initialize extension and UI elements.

        Args:
            ext_id: Extension identifier string.
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
        menu_entry = [
            MenuItemDescription(name=EXTENSION_TITLE, onclick_action=(ext_id, f"CreateUIExtension:{EXTENSION_TITLE}"))
        ]
        self._menu_items = [MenuItemDescription(name="Robotics", sub_menu=menu_entry)]

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
        """Cleanup extension resources and remove UI elements."""
        self._models = {}
        remove_menu_items(self._menu_items, "Tools")

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
                observer_name="isaacsim.robot_setup.grasp_editor.Extension._on_stage_opened",
            )
            self._stage_event_sub_closed = carb.eventdispatcher.get_eventdispatcher().observe_event(
                event_name=self._usd_context.stage_event_name(omni.usd.StageEventType.CLOSED),
                on_event=self._on_stage_closed,
                observer_name="isaacsim.robot_setup.grasp_editor.Extension._on_stage_closed",
            )
            self._stage_event_sub_assets_loaded = carb.eventdispatcher.get_eventdispatcher().observe_event(
                event_name=self._usd_context.stage_event_name(omni.usd.StageEventType.ASSETS_LOADED),
                on_event=self._on_assets_loaded,
                observer_name="isaacsim.robot_setup.grasp_editor.Extension._on_assets_loaded",
            )
            self._stage_event_sub_sim_stop = carb.eventdispatcher.get_eventdispatcher().observe_event(
                event_name=self._usd_context.stage_event_name(omni.usd.StageEventType.SIMULATION_STOP_PLAY),
                on_event=self._on_simulation_stop_play,
                observer_name="isaacsim.robot_setup.grasp_editor.Extension._on_simulation_stop_play",
            )
            self._timeline_event_sub_play = carb.eventdispatcher.get_eventdispatcher().observe_event(
                event_name=omni.timeline.GLOBAL_EVENT_PLAY,
                on_event=self._on_timeline_play,
                observer_name="isaacsim.robot_setup.grasp_editor.Extension._on_timeline_play",
            )
            self._timeline_event_sub_stop = carb.eventdispatcher.get_eventdispatcher().observe_event(
                event_name=omni.timeline.GLOBAL_EVENT_STOP,
                on_event=self._on_timeline_stop,
                observer_name="isaacsim.robot_setup.grasp_editor.Extension._on_timeline_stop",
            )

            self._build_ui()
        else:
            self._usd_context = None
            self._stage_event_sub_opened = None
            self._stage_event_sub_closed = None
            self._stage_event_sub_assets_loaded = None
            self._stage_event_sub_sim_stop = None
            self._timeline_event_sub_play = None
            self._timeline_event_sub_stop = None
            self.ui_builder.cleanup()

    def _build_ui(self) -> None:
        """Build the extension's UI components and dock the window."""
        with self._window.frame:
            with ui.VStack(spacing=5, height=0):
                self._build_extension_ui()

        async def dock_window() -> None:
            await omni.kit.app.get_app().next_update_async()

            def dock(space: object, name: object, location: object, pos: object = 0.5) -> object:
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
        """Handle menu item selection to toggle window visibility."""
        self._window.visible = not self._window.visible
        self.ui_builder.on_menu_callback()

    def _on_timeline_play(self, event: object) -> None:
        """Handle timeline play events and subscribe to physics step updates.

        Args:
            event: Timeline play event data.
        """
        if not self._physics_subscription:
            self._physics_subscription = self._physics_simulation_interface.subscribe_physics_on_step_events(
                pre_step=False, order=0, on_update=self._on_physics_step
            )
        self.ui_builder.on_timeline_event(event)

    def _on_timeline_stop(self, event: object) -> None:
        """Handle timeline stop events and cleanup physics subscriptions.

        Args:
            event: Timeline stop event data.
        """
        self._physics_subscription = None
        self.ui_builder.on_timeline_event(event)

    def _on_physics_step(self, step: float, context: object) -> None:
        """Handle physics simulation step events.

        Args:
            step: Physics step data.
            context: Physics simulation context.
        """
        self.ui_builder.on_physics_step(step)

    def _on_stage_opened(self, event: object) -> None:
        """Handle stage opened events and cleanup previous state.

        Args:
            event: Stage opened event data.
        """
        # stage was opened, cleanup
        self._physics_subscription = None
        self.ui_builder.cleanup()

    def _on_stage_closed(self, event: object) -> None:
        """Handle stage closed events and cleanup resources.

        Args:
            event: Stage closed event data.
        """
        # stage was closed, cleanup
        self._physics_subscription = None
        self.ui_builder.cleanup()

    def _on_assets_loaded(self, event: object) -> None:
        """Handles the stage assets loaded event.

        Called when all assets in the stage have finished loading.

        Args:
            event: The stage assets loaded event.
        """
        self.ui_builder.on_assets_loaded()

    def _on_simulation_stop_play(self, event: object) -> None:
        """Handles the simulation stop play event.

        Called when the simulation transitions from play to stop state.

        Args:
            event: The simulation stop play event.
        """
        self.ui_builder.on_simulation_stop_play()

    def _build_extension_ui(self) -> None:
        """Builds the extension's user interface.

        Delegates UI construction to the UIBuilder instance by calling its build_ui method.
        """
        # Call user function for building UI
        self.ui_builder.build_ui()
