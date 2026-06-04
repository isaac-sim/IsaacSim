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

"""CumotionWorldInterface extension with Isaac Sim integration."""

import asyncio
import gc
from typing import Any

import carb.eventdispatcher
import omni
import omni.timeline
import omni.ui as ui
import omni.usd
from isaacsim.gui.components.element_wrappers import ScrollingWindow
from isaacsim.gui.components.menu import MenuItemDescription
from omni.kit.menu.utils import add_menu_items, remove_menu_items

from .global_variables import EXTENSION_TITLE
from .ui_builder import UIBuilder


class Extension(omni.ext.IExt):
    """Isaac Sim extension providing the CumotionWorldInterface example with interactive UI."""

    def on_startup(self, ext_id: str) -> None:
        """Initialize the extension and register UI elements.

        Args:
            ext_id: The extension ID.
        """
        self.ext_id = ext_id
        self._usd_context = omni.usd.get_context()

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
        # Use shorter name for menu item, register under parent "cuMotion Examples" menu
        self._menu_items = [
            MenuItemDescription(name="World Interface", onclick_action=(ext_id, f"CreateUIExtension:{EXTENSION_TITLE}"))
        ]
        add_menu_items(self._menu_items, "cuMotion Examples")

        self.ui_builder = UIBuilder()
        self._timeline = omni.timeline.get_timeline_interface()

    def on_shutdown(self) -> None:
        """Clean up the extension and deregister UI elements."""
        remove_menu_items(self._menu_items, "cuMotion Examples")

        action_registry = omni.kit.actions.core.get_action_registry()
        action_registry.deregister_action(self.ext_id, f"CreateUIExtension:{EXTENSION_TITLE}")

        if self._window:
            self._window = None
        self.ui_builder.cleanup()
        gc.collect()

    def _on_window(self, visible: bool) -> None:
        if self._window.visible:
            self._usd_context = omni.usd.get_context()
            ed = carb.eventdispatcher.get_eventdispatcher()
            self._stage_event_sub_opened = ed.observe_event(
                event_name=self._usd_context.stage_event_name(omni.usd.StageEventType.OPENED),
                on_event=self._on_stage_opened,
                observer_name="cumotion_world_interface._on_stage_opened",
            )
            self._stage_event_sub_closed = ed.observe_event(
                event_name=self._usd_context.stage_event_name(omni.usd.StageEventType.CLOSED),
                on_event=self._on_stage_closed,
                observer_name="cumotion_world_interface._on_stage_closed",
            )
            self._timeline_event_sub_stop = ed.observe_event(
                event_name=omni.timeline.GLOBAL_EVENT_STOP,
                on_event=self._on_timeline_stop,
                observer_name="cumotion_world_interface._on_timeline_stop",
            )
            self._build_ui()
        else:
            self._usd_context = None
            self._stage_event_sub_opened = None
            self._stage_event_sub_closed = None
            self._timeline_event_sub_stop = None
            self.ui_builder.cleanup()

    def _build_ui(self) -> None:
        with self._window.frame:
            with ui.VStack(spacing=5, height=0):
                self.ui_builder.build_ui()

        async def dock_window() -> None:
            await omni.kit.app.get_app().next_update_async()
            tgt = ui.Workspace.get_window("Viewport")
            window = omni.ui.Workspace.get_window(EXTENSION_TITLE)
            if window and tgt:
                window.dock_in(tgt, omni.ui.DockPosition.LEFT, 0.33)
            await omni.kit.app.get_app().next_update_async()

        self._task = asyncio.ensure_future(dock_window())

    def _menu_callback(self) -> None:
        self._window.visible = not self._window.visible

    def _on_timeline_stop(self, event: Any) -> None:
        self.ui_builder.on_timeline_event(event)

    def _on_stage_opened(self, event: Any) -> None:
        self.ui_builder.on_stage_changed(event)

    def _on_stage_closed(self, event: Any) -> None:
        self.ui_builder.on_stage_changed(event)
        # Encourage release of any prim wrappers we just dropped.
        gc.collect()
