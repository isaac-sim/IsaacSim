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

"""Top-level orchestrator for the XRDF editor extension.

The bulk of the previous monolithic implementation now lives in:

- :mod:`xrdf_io`, :mod:`lula_io` -- serialisation
- :mod:`articulation_discovery`, :mod:`sphere_generation` -- stage utilities
- :mod:`editor_state` -- domain state
- :mod:`ui.ui_builder` -- :class:`UIBuilder` orchestrating the UI panels

This module wires those pieces together inside an ``omni.ext`` extension and
follows the standard ``isaacsim`` UI-extension boilerplate: it owns the
window, menu, and event subscriptions, and delegates all panel logic to a
:class:`UIBuilder` instance.
"""

from __future__ import annotations

import asyncio
import gc
import weakref
from typing import Any

import carb.eventdispatcher
import omni
import omni.ext
import omni.physics.core
import omni.timeline
import omni.ui as ui
import omni.usd
from isaacsim.gui.components.element_wrappers import ScrollingWindow
from isaacsim.gui.components.menu import make_menu_item_description
from omni.kit.menu.utils import MenuItemDescription, add_menu_items, remove_menu_items

from .constants import EXTENSION_NAME
from .ui import UIBuilder

__all__ = ["Extension"]


class Extension(omni.ext.IExt):
    """Extension boilerplate for the XRDF Editor.

    Owns the window, menu, and event subscriptions; delegates all UI and
    domain logic to :class:`UIBuilder`. Standard callbacks invoked on the
    builder are:

    * ``on_menu_callback``: window opened from the toolbar
    * ``on_simulation_start_play`` / ``on_simulation_stop_play``: physics
      simulation transitions
    * ``on_physics_step``: per physics step
    * ``on_stage_opened`` / ``on_stage_closed`` / ``on_selection_changed``:
      stage events
    * ``cleanup``: extension shutdown
    * ``build_ui``: window becomes visible
    """

    # ------------------------------------------------------------------
    # Lifecycle
    # ------------------------------------------------------------------
    def on_startup(self, ext_id: str) -> None:
        """Initialise window, menu, event handles, and the UI builder.

        Args:
            ext_id: Owning extension identifier.
        """
        self.ext_id = ext_id
        self._usd_context = omni.usd.get_context()

        # Build Window
        self._window = ScrollingWindow(
            title=EXTENSION_NAME,
            width=600,
            height=500,
            visible=False,
            dockPreference=ui.DockPreference.LEFT_BOTTOM,
        )
        self._window.set_visibility_changed_fn(self._on_window)

        menu_entry = [
            make_menu_item_description(ext_id, EXTENSION_NAME, lambda a=weakref.proxy(self): a._menu_callback())
        ]
        self._menu_items = [MenuItemDescription(name="Robotics", sub_menu=menu_entry)]
        add_menu_items(self._menu_items, "Tools")

        # Filled in with User Functions
        self.ui_builder = UIBuilder(ext_id, __file__)

        # Events
        self._physics_simulation_interface = omni.physics.core.get_physics_simulation_interface()
        self._physics_subscription = None
        self._stage_event_sub_selection = None
        self._stage_event_sub_opened = None
        self._stage_event_sub_closed = None
        self._stage_event_sub_sim_play = None
        self._stage_event_sub_sim_stop = None
        self._timeline = omni.timeline.get_timeline_interface()

    def on_shutdown(self) -> None:
        """Tear down panels, subscriptions, and menu entry."""
        remove_menu_items(self._menu_items, "Tools")
        if self._window:
            self._window = None
        self.ui_builder.cleanup()
        self._stage_event_sub_selection = None
        self._stage_event_sub_opened = None
        self._stage_event_sub_closed = None
        self._stage_event_sub_sim_play = None
        self._stage_event_sub_sim_stop = None
        self._physics_subscription = None
        gc.collect()

    # ------------------------------------------------------------------
    # Window
    # ------------------------------------------------------------------
    def _on_window(self, visible: bool) -> None:
        """Subscribe or unsubscribe from stage events as the window opens or closes.

        Args:
            visible: Whether the window is visible.
        """
        if self._window.visible:
            self._usd_context = omni.usd.get_context()
            ed = carb.eventdispatcher.get_eventdispatcher()
            self._stage_event_sub_selection = ed.observe_event(
                event_name=self._usd_context.stage_event_name(omni.usd.StageEventType.SELECTION_CHANGED),
                on_event=self._on_selection_changed,
                observer_name="isaacsim.robot_setup.xrdf_editor.Extension._on_selection_changed",
            )
            self._stage_event_sub_opened = ed.observe_event(
                event_name=self._usd_context.stage_event_name(omni.usd.StageEventType.OPENED),
                on_event=self._on_stage_opened,
                observer_name="isaacsim.robot_setup.xrdf_editor.Extension._on_stage_opened",
            )
            self._stage_event_sub_closed = ed.observe_event(
                event_name=self._usd_context.stage_event_name(omni.usd.StageEventType.CLOSED),
                on_event=self._on_stage_closed,
                observer_name="isaacsim.robot_setup.xrdf_editor.Extension._on_stage_closed",
            )
            self._stage_event_sub_sim_play = ed.observe_event(
                event_name=self._usd_context.stage_event_name(omni.usd.StageEventType.SIMULATION_START_PLAY),
                on_event=self._on_simulation_start_play,
                observer_name="isaacsim.robot_setup.xrdf_editor.Extension._on_simulation_start_play",
            )
            self._stage_event_sub_sim_stop = ed.observe_event(
                event_name=self._usd_context.stage_event_name(omni.usd.StageEventType.SIMULATION_STOP_PLAY),
                on_event=self._on_simulation_stop_play,
                observer_name="isaacsim.robot_setup.xrdf_editor.Extension._on_simulation_stop_play",
            )

            self._build_ui()

            # The SIMULATION_START_PLAY event only fires on the play transition;
            # if the timeline is already running when the window opens, the
            # observer above will never be triggered and the physics-step
            # subscription would stay `None`. Bootstrap it directly so panel
            # callbacks (e.g. Joint Position) take effect on the next tick.
            if not self._timeline.is_stopped():
                self._on_simulation_start_play(None)
        else:
            self._usd_context = None
            self._stage_event_sub_selection = None
            self._stage_event_sub_opened = None
            self._stage_event_sub_closed = None
            self._stage_event_sub_sim_play = None
            self._stage_event_sub_sim_stop = None
            self._physics_subscription = None

    def _build_ui(self) -> None:
        """Build the extension's UI inside the window and dock it."""
        with self._window.frame:
            with ui.VStack(spacing=5, height=0):
                self._build_extension_ui()

        async def dock_window() -> None:
            await omni.kit.app.get_app().next_update_async()

            def dock(space: Any, name: str, location: Any, pos: float = 0.5) -> Any:
                window = omni.ui.Workspace.get_window(name)
                if window and space:
                    window.dock_in(space, location, pos)
                return window

            tgt = ui.Workspace.get_window("Viewport")
            dock(tgt, EXTENSION_NAME, omni.ui.DockPosition.LEFT, 0.33)
            await omni.kit.app.get_app().next_update_async()

        self._task = asyncio.ensure_future(dock_window())

    #################################################################
    # Functions below this point call user functions
    #################################################################

    def _menu_callback(self) -> None:
        """Toggle window visibility from the Tools menu entry."""
        self._window.visible = not self._window.visible
        self.ui_builder.on_menu_callback()

    def _on_simulation_start_play(self, event: object) -> None:
        """Handle stage SIMULATION_START_PLAY by subscribing to physics steps.

        Args:
            event: Stage simulation-start-play event payload.
        """
        if not self._physics_subscription:
            self._physics_subscription = self._physics_simulation_interface.subscribe_physics_on_step_events(
                pre_step=False, order=0, on_update=self._on_physics_step
            )
        self.ui_builder.on_simulation_start_play(event)

    def _on_simulation_stop_play(self, event: object) -> None:
        """Handle stage SIMULATION_STOP_PLAY by dropping the physics subscription.

        Args:
            event: Stage simulation-stop-play event payload.
        """
        self._physics_subscription = None
        self.ui_builder.on_simulation_stop_play(event)

    def _on_physics_step(self, step: float, context: object) -> None:
        """Forward physics-step callbacks to the UI builder.

        Args:
            step: Physics step size in seconds.
            context: Physics callback context.
        """
        self.ui_builder.on_physics_step(step)

    def _on_stage_opened(self, event: object) -> None:
        """Handle stage OPENED by dropping the physics sub and delegating to the builder.

        Args:
            event: Stage opened event payload.
        """
        self._physics_subscription = None
        self.ui_builder.on_stage_opened(event)

    def _on_stage_closed(self, event: object) -> None:
        """Handle stage CLOSED by dropping the physics sub and delegating to the builder.

        Args:
            event: Stage closed event payload.
        """
        self._physics_subscription = None
        self.ui_builder.on_stage_closed(event)

    def _on_selection_changed(self, event: object) -> None:
        """Handle stage SELECTION_CHANGED by delegating to the builder.

        Args:
            event: Stage selection-changed event payload.
        """
        self.ui_builder.on_selection_changed(event)

    def _build_extension_ui(self) -> None:
        """Delegate UI construction to :class:`UIBuilder`."""
        self.ui_builder.build_ui()
