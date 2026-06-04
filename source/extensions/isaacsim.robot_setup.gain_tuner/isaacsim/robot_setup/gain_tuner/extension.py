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

"""Extension for the isaacsim.robot_setup.gain_tuner that provides a UI-based gain tuner tool for robot setup."""

import asyncio
import gc

import carb
import carb.eventdispatcher
import omni
import omni.kit.actions.core
import omni.kit.app
import omni.physics.core
import omni.timeline
import omni.ui as ui
import omni.usd
from isaacsim.gui.components.element_wrappers import ScrollingWindow
from isaacsim.gui.components.menu import MenuItemDescription
from omni.kit.menu.utils import add_menu_items, remove_menu_items

from .gains_tuner import GainsTestMode, SinusoidalTest, StepFunctionTest
from .global_variables import EXTENSION_TITLE
from .snap_to_limits import SnapToLimitsTest
from .stress_test import StressTest
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
    """Extension class for the isaacsim.robot_setup.gain_tuner extension.

    Provides a UI-based extension that adds a gain tuner tool to the Omniverse Kit SDK interface. The extension
    creates a scrolling window under Tools > Robotics > Asset Editors, enabling users to interact with
    robot gain tuning functionality.

    The extension handles standard lifecycle operations including window management, event subscriptions for
    stage and timeline events, physics step callbacks, and UI building. It delegates core functionality to a
    UIBuilder instance that manages the specific gain tuning interface and operations.

    Key features include automatic docking to the viewport, event-driven updates during simulation playback,
    and proper resource cleanup during shutdown. The extension subscribes to physics step events during
    timeline playback and stage events for proper initialization and cleanup when stages are opened or closed.
    """

    def on_startup(self, ext_id: str) -> None:
        """Initialize extension and UI elements.

        Args:
            ext_id: The extension identifier.
        """
        self.ext_id = ext_id
        self._ext_name = omni.ext.get_extension_name(ext_id)
        self._usd_context = omni.usd.get_context()

        # Build Window
        self._window = ScrollingWindow(
            title=EXTENSION_TITLE, width=700, height=500, visible=False, dockPreference=ui.DockPreference.LEFT_BOTTOM
        )
        self._window.set_visibility_changed_fn(self._on_window)

        action_registry = omni.kit.actions.core.get_action_registry()
        action_registry.register_action(
            self._ext_name,
            f"CreateUIExtension:{EXTENSION_TITLE}",
            self._menu_callback,
            description=f"Add {EXTENSION_TITLE} Extension to UI toolbar",
        )
        self._menu_items = [
            MenuItemDescription(
                name=EXTENSION_TITLE, onclick_action=(self._ext_name, f"CreateUIExtension:{EXTENSION_TITLE}")
            )
        ]
        asset_editors = [MenuItemDescription(name="Asset Editors", sub_menu=self._menu_items)]
        self._menu_items = [MenuItemDescription(name="Robotics", sub_menu=asset_editors)]
        add_menu_items(self._menu_items, "Tools")

        # Filled in with User Functions
        self.ui_builder = UIBuilder()

        # Register built-in tests
        tuner = self.ui_builder._gains_tuner
        tuner.register_test(GainsTestMode.SINUSOIDAL, SinusoidalTest())
        tuner.register_test(GainsTestMode.STEP, StepFunctionTest())
        tuner.register_test(GainsTestMode.SNAP_TO_LIMITS, SnapToLimitsTest())
        tuner.register_test(GainsTestMode.STRESS_TEST, StressTest())

        # Events
        self._physics_simulation_interface = omni.physics.core.get_physics_simulation_interface()
        self._physics_subscription = None
        self._event_dispatcher = carb.eventdispatcher.get_eventdispatcher()
        # self._update_stream = omni.kit.app.get_app().get_update_event_stream()
        self._render_subscription = None
        self._stage_event_sub = None
        self._timeline = omni.timeline.get_timeline_interface()

    def on_shutdown(self) -> None:
        """Clean up extension resources and remove UI elements."""
        remove_menu_items(self._menu_items, "Tools")

        action_registry = omni.kit.actions.core.get_action_registry()
        action_registry.deregister_action(self._ext_name, f"CreateUIExtension:{EXTENSION_TITLE}")

        if self._window:
            self._window = None
        self.ui_builder.cleanup()
        gc.collect()

    def _on_window(self, visible: bool) -> None:
        """Handle window visibility changes and manage event subscriptions.

        Args:
            visible: Whether the window is visible.
        """
        if visible:
            # Subscribe to Stage and Timeline Events
            self._usd_context = omni.usd.get_context()
            self._stage_event_sub_opened = carb.eventdispatcher.get_eventdispatcher().observe_event(
                event_name=self._usd_context.stage_event_name(omni.usd.StageEventType.OPENED),
                on_event=self._on_stage_opened,
                observer_name="isaacsim.robot_setup.gain_tuner.Extension._on_stage_opened",
            )
            self._stage_event_sub_closed = carb.eventdispatcher.get_eventdispatcher().observe_event(
                event_name=self._usd_context.stage_event_name(omni.usd.StageEventType.CLOSED),
                on_event=self._on_stage_closed,
                observer_name="isaacsim.robot_setup.gain_tuner.Extension._on_stage_closed",
            )
            self._stage_event_sub_assets_loaded = carb.eventdispatcher.get_eventdispatcher().observe_event(
                event_name=self._usd_context.stage_event_name(omni.usd.StageEventType.ASSETS_LOADED),
                on_event=self._on_assets_loaded,
                observer_name="isaacsim.robot_setup.gain_tuner.Extension._on_assets_loaded",
            )
            self._timeline_event_sub_play = carb.eventdispatcher.get_eventdispatcher().observe_event(
                event_name=omni.timeline.GLOBAL_EVENT_PLAY,
                on_event=self._on_timeline_play,
                observer_name="isaacsim.robot_setup.gain_tuner.Extension._on_timeline_play",
            )
            self._timeline_event_sub_stop = carb.eventdispatcher.get_eventdispatcher().observe_event(
                event_name=omni.timeline.GLOBAL_EVENT_STOP,
                on_event=self._on_timeline_stop,
                observer_name="isaacsim.robot_setup.gain_tuner.Extension._on_timeline_stop",
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
            self._render_subscription = None
            self.ui_builder.cleanup()

    def _build_ui(self) -> None:
        """Build the extension UI and dock the window in the viewport."""
        with self._window.frame:
            with ui.VStack(spacing=5, height=0):
                self._build_extension_ui()

        async def dock_window() -> None:
            await omni.kit.app.get_app().next_update_async()

            def dock(
                space: ui.Window | None, name: str, location: ui.DockPosition, pos: float = 0.5
            ) -> ui.Window | None:
                window = omni.ui.Workspace.get_window(name)
                if window and space:
                    window.dock_in(space, location, pos)
                return window

            tgt = ui.Workspace.get_window("Viewport")
            dock(tgt, EXTENSION_TITLE, omni.ui.DockPosition.LEFT, 0.55)
            await omni.kit.app.get_app().next_update_async()

        self._task = asyncio.ensure_future(dock_window())

    #################################################################
    # Functions below this point call user functions
    #################################################################

    def _menu_callback(self) -> None:
        """Handle menu or action: show the window and focus it (same intent as re-opening from the menu)."""
        self._window.visible = True
        focus_fn = getattr(self._window, "focus", None)
        if callable(focus_fn):
            focus_fn()

        self.ui_builder.on_menu_callback()

        if self._timeline.is_playing():
            if not self._physics_subscription:
                self._physics_subscription = self._physics_simulation_interface.subscribe_physics_on_step_events(
                    pre_step=False, order=0, on_update=self._on_physics_step
                )
        if not self._render_subscription:
            self._render_subscription = self._event_dispatcher.observe_event(
                event_name=omni.kit.app.GLOBAL_EVENT_UPDATE,
                on_event=self.ui_builder.on_render_step,
                observer_name="isaacsim.robot_setup.gain_tuner.Extension._on_render_step",
            )

    def _on_timeline_play(self, event: object) -> None:
        """Handle timeline play event and set up physics and render subscriptions.

        Args:
            event: The timeline play event.
        """
        if not self._physics_subscription:
            self._physics_subscription = self._physics_simulation_interface.subscribe_physics_on_step_events(
                pre_step=False, order=0, on_update=self._on_physics_step
            )
        if not self._render_subscription:
            self._render_subscription = self._event_dispatcher.observe_event(
                event_name=omni.kit.app.GLOBAL_EVENT_UPDATE,
                on_event=self.ui_builder.on_render_step,
                observer_name="isaacsim.robot_setup.gain_tuner.Extension._on_render_step",
            )
        self.ui_builder.on_timeline_event(event)

    def _on_timeline_stop(self, event: object) -> None:
        """Handle timeline stop event and clean up physics subscriptions.

        Args:
            event: The timeline stop event.
        """
        self._physics_subscription = None
        self.ui_builder.on_timeline_event(event)

    def _on_physics_step(self, step: float, context: object) -> None:
        """Handle physics step events during simulation.

        Args:
            step: The physics step information.
            context: The physics step context.
        """
        self.ui_builder.on_physics_step(step)

    def _on_stage_opened(self, event: object) -> None:
        """Handle stage opened event and reset UI builder state.

        Args:
            event: The stage opened event.
        """
        # stage was opened, cleanup
        self._physics_subscription = None
        self._render_subscription = None
        self.ui_builder.reset()
        self.ui_builder.on_stage_event(event)

    def _on_assets_loaded(self, event: object) -> None:
        """Handle assets loaded event and notify UI builder.

        Args:
            event: The assets loaded event.
        """
        self.ui_builder.on_stage_event(event)

    def _on_stage_closed(self, event: object) -> None:
        """Handles stage closure events and performs cleanup operations.

        Cleans up physics and render subscriptions, resets the UI builder state, and forwards the
        event to the UI builder for additional processing.

        Args:
            event: The stage closed event containing event details.
        """
        # stage was closed, cleanup
        self._physics_subscription = None
        self._render_subscription = None
        self.ui_builder.reset()
        self.ui_builder.on_stage_event(event)

    def _build_extension_ui(self) -> None:
        """Builds the extension's user interface by calling the UI builder's build_ui method."""
        # Call user function for building UI
        self.ui_builder.build_ui()
