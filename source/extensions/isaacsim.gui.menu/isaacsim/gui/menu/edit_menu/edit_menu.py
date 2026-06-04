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

"""Edit menu behaviors for selection and prim operations."""

__all__ = ["EditMenuExtension", "get_extension_path"]

import asyncio
import contextlib
import os
import sys
from collections.abc import Callable
from functools import partial
from typing import Any

import carb.input
import omni.kit.app
import omni.kit.menu.utils
import omni.kit.notification_manager as nm
import omni.kit.usd.layers as layers
import omni.ui as ui
import omni.usd
from omni.kit.menu.utils import MenuItemDescription, MenuLayout
from pxr import Kind, Sdf, Tf, Usd

from .edit_actions import deregister_actions, register_actions
from .selection import Selection
from .selection_window import SelectionSetWindow

# pylint: disable=redefined-outer-name

_extension_instance: "EditMenuExtension | None" = None
_extension_path: str | None = None

PERSISTENT_SETTINGS_PREFIX = "/persistent"
CAPTURE_FRAME_PATH = "/app/captureFrame/path"
KEEP_TRANSFORM_FOR_REPARENTING = "/persistent/app/stage/movePrimInPlace"


class EditMenuExtension:
    """Build and manage the Edit menu.

    Args:
        ext_id: Extension identifier provided by the extension manager.
    """

    def __init__(self, ext_id: str) -> None:
        omni.kit.menu.utils.set_default_menu_priority("Edit", -9)
        self._select_recent_menu_list: list[MenuItemDescription] = [MenuItemDescription(name="None", enabled=False)]
        self._selection_set_menu_list: list[MenuItemDescription] = [MenuItemDescription(name="None", enabled=False)]
        self._selection_kind_menu_list: list[MenuItemDescription] = [MenuItemDescription(name="None", enabled=False)]
        self._select_recent: list[Selection] = []
        self._selection_set: list[Selection] = []
        self._ext_name = "isaacsim.gui.menu.edit_menu"
        self._create_selection_window: SelectionSetWindow | None = None
        self._edit_menu_list: list[MenuItemDescription] | None = None
        self._context_menus: list[Any] = []
        self._preferences_page: object | None = None
        self._cm_hooks: object | None = None
        self._hooks: list[Any] = []
        self._edit_select_menu: list[MenuItemDescription] | None = None

        global _extension_instance
        _extension_instance = self

        global _extension_path
        _extension_path = omni.kit.app.get_app_interface().get_extension_manager().get_extension_path(ext_id)

        register_actions(self._ext_name, EditMenuExtension, lambda: _extension_instance)

        self._select_recent_menu_list = [MenuItemDescription(name="None", enabled=False)]
        self._selection_set_menu_list = [MenuItemDescription(name="None", enabled=False)]
        self._selection_kind_menu_list = [MenuItemDescription(name="None", enabled=False)]
        self._select_recent = []
        self._selection_set = []
        self._create_selection_window = None
        self._edit_menu_list = None
        self._build_edit_menu()

        manager = omni.kit.app.get_app().get_extension_manager()
        self._cm_hooks = manager.subscribe_to_extension_enable(
            lambda _: self._register_context_menu(),
            lambda _: self._unregister_context_menu(),
            ext_name="omni.kit.context_menu",
            hook_name="menu edit listener",
        )

        manager = omni.kit.app.get_app().get_extension_manager()
        self._hooks.append(
            manager.subscribe_to_extension_enable(
                on_enable_fn=lambda _: self._register_page(),
                on_disable_fn=lambda _: self._unregister_page(),
                ext_name="omni.kit.window.preferences",
                hook_name="isaacsim.gui.menu omni.kit.window.preferences listener",
            )
        )

        self.__menu_layout = [
            MenuLayout.Menu(
                "Edit",
                [
                    MenuLayout.Item(name="Undo", source="Edit/Undo"),
                    MenuLayout.Item(name="Redo", source="Edit/Redo"),
                    MenuLayout.Item(name="Repeat", source="Edit/Repeat"),
                    MenuLayout.Item(name="Select", source="Edit/Select"),
                    MenuLayout.Seperator(),
                    MenuLayout.Item(name="Instance", source="Edit/Instance"),
                    MenuLayout.Item(name="Duplicate", source="Edit/Duplicate"),
                    MenuLayout.Item(name="Duplicate - All Layers", source="Edit/Duplicate - All Layers"),
                    MenuLayout.Item(name="Duplicate - Collapsed", source="Edit/Duplicate - Collapsed"),
                    MenuLayout.Item(name="Parent", source="Edit/Parent"),
                    MenuLayout.Item(name="Unparent", source="Edit/Unparent"),
                    MenuLayout.Item(name="Group", source="Edit/Group"),
                    MenuLayout.Item(name="Ungroup", source="Edit/Ungroup"),
                    MenuLayout.Item(name="Toggle Visibility", source="Edit/Toggle Visibility"),
                    MenuLayout.Seperator(),
                    MenuLayout.Item(name="Deactivate", source="Edit/Deactivate"),
                    MenuLayout.Item(name="Delete", source="Edit/Delete"),
                    MenuLayout.Item(name="Delete - All Layers", source="Edit/Delete - All Layers"),
                    MenuLayout.Item(name="Rename", source="Edit/Rename"),
                    MenuLayout.Seperator(),
                    MenuLayout.Item(name="Hide Unselected", source="Edit/Hide Unselected"),
                    MenuLayout.Item(name="Unhide All", source="Edit/Unhide All"),
                    MenuLayout.Seperator(),
                    MenuLayout.Item(name="Focus", source="Edit/Focus"),
                    MenuLayout.Item(name="Capture Screenshot", source="Edit/Capture Screenshot"),
                    MenuLayout.Item(name="Toggle Visualization Mode", source="Edit/Toggle Visualization Mode"),
                    MenuLayout.Seperator(),
                    MenuLayout.Item(name="Preferences", source="Edit/Preferences"),
                ],
            )
        ]

        omni.kit.menu.utils.add_layout(self.__menu_layout)

    def shutdown(self) -> None:
        """Remove menu items, hooks, and UI resources.

        Example:
            .. code-block:: python

                menu = EditMenuExtension("ext.id")
                menu.shutdown()
        """
        global _extension_instance

        omni.kit.menu.utils.remove_layout(self.__menu_layout)
        self._unregister_page()

        self._unregister_context_menu()
        self._cm_hooks = None

        _extension_instance = None
        self._select_recent_menu_list = []
        self._selection_set_menu_list = []
        self._selection_kind_menu_list = []
        self._select_recent = []
        self._selection_set = []

        if self._create_selection_window:
            self._create_selection_window.shutdown()
            del self._create_selection_window

        omni.kit.menu.utils.remove_menu_items(self._edit_menu_list, "Edit")
        deregister_actions(self._ext_name)

    def _register_page(self) -> None:
        """Register the screenshot preferences page if available."""
        try:
            from omni.kit.window.preferences import register_page

            from .screenshot_page import ScreenshotPreferences

            self._preferences_page = register_page(ScreenshotPreferences())
        except ModuleNotFoundError:
            pass

    def _unregister_page(self) -> None:
        """Unregister the screenshot preferences page if present."""
        if self._preferences_page:
            try:
                import omni.kit.window.preferences

                omni.kit.window.preferences.unregister_page(self._preferences_page)
                self._preferences_page = None
            except ModuleNotFoundError:
                pass

    @staticmethod
    def post_notification(message: str, info: bool = False, duration: int = 3) -> None:
        """Post a notification message to the UI.

        Args:
            message: The message to display.
            info: Whether to display an info notification.
            duration: Duration in seconds for the notification.

        Example:
            .. code-block:: python

                EditMenuExtension.post_notification("Selection updated.")
        """
        _type = nm.NotificationStatus.INFO if info else nm.NotificationStatus.WARNING
        nm.post_notification(message, status=_type, duration=duration)

    def _usd_kinds(self) -> list[str]:
        """Return the USD kinds used for selection filters.

        Returns:
            USD kind tokens used for selection.
        """
        return ["model", "assembly", "group", "component", "subcomponent"]

    def _usd_kinds_display(self) -> list[str]:
        """Return displayable USD kinds for selection filters.

        Returns:
            USD kinds to show in the menu.
        """
        return self._usd_kinds()[1:]

    def _plugin_kinds(self) -> set[str]:
        """Return plugin-defined kinds not covered by USD defaults.

        Returns:
            Plugin-defined USD kinds.
        """
        all_kinds = set(Kind.Registry.GetAllKinds())
        return all_kinds - set(self._usd_kinds())

    def _create_selection_set(self) -> None:
        """Create or show the selection set naming dialog."""
        paths = omni.usd.get_context().get_selection().get_selected_prim_paths()
        if not paths:
            EditMenuExtension.post_notification("Cannot Create Selection Set as no prims are selected")
            return

        if self._create_selection_window:
            self._create_selection_window.show()
        else:
            self._create_selection_window = SelectionSetWindow(self._add_to_selection_set)

    def _build_recent_menu(self) -> None:
        """Build the Select Recent submenu items."""
        if self._select_recent:
            self._select_recent_menu_list.clear()

            self._select_recent.sort(reverse=True, key=lambda x: x.time)
            del self._select_recent[5:]

            for recent in self._select_recent:
                index = self._select_recent.index(recent)
                self._select_recent_menu_list.append(
                    MenuItemDescription(
                        name=recent.description, onclick_action=("isaacsim.gui.menu.edit_menu", "select_recent", index)
                    )
                )

            omni.kit.menu.utils.rebuild_menus()

    def _on_select_recent(self, index: int) -> None:
        """Handle selection of a recent item by index.

        Args:
            index: Index of the recent selection to apply.
        """

        async def select_func() -> None:
            """Execute a recent selection on the next update."""
            await omni.kit.app.get_app().next_update_async()
            do_recent = self._select_recent[index]
            omni.kit.commands.execute("SelectListCommand", selection=do_recent.selection)
            do_recent.touch()
            self._build_recent_menu()

        asyncio.ensure_future(select_func())

    def _add_to_selection_set(self, description: str) -> None:
        """Add the current selection to the named selection set.

        Args:
            description: Label to assign to the selection set.
        """
        new_selection = omni.usd.get_context().get_selection().get_selected_prim_paths()
        for selection in self._selection_set:
            if selection.description == description:
                EditMenuExtension.post_notification(f'Selection set "{description}" already exists')
                return

        self._selection_set.append(Selection(description, new_selection))
        self._build_selection_set_menu()

    def _add_to_recent(self, description: str, selection: list[str]) -> None:
        """Add a selection to the recent list.

        Args:
            description: Label describing the selection.
            selection: Selection paths to store.
        """
        found = False
        for prev_selection in self._select_recent:
            if prev_selection.description == description and prev_selection.selection == selection:
                prev_selection.touch()
                found = True
                break

        if not found:
            self._select_recent.append(Selection(description, selection))

        self._build_recent_menu()

    def _on_select_selection_set(self, index: int) -> None:
        """Handle selection of a saved selection set by index.

        Args:
            index: Index of the selection set to apply.
        """

        async def select_func() -> None:
            """Execute a selection set action on the next update."""
            await omni.kit.app.get_app().next_update_async()
            do_select = self._selection_set[index]
            omni.kit.commands.execute("SelectListCommand", selection=do_select.selection)

        asyncio.ensure_future(select_func())

    def _build_selection_set_menu(self) -> None:
        """Build the Select Set submenu items."""
        self._selection_set_menu_list.clear()

        self._selection_set.sort(key=lambda x: x.description)

        for recent in self._selection_set:
            index = self._selection_set.index(recent)
            self._selection_set_menu_list.append(
                MenuItemDescription(
                    name=recent.description,
                    onclick_action=("isaacsim.gui.menu.edit_menu", "select_selection_set", index),
                )
            )

        omni.kit.menu.utils.rebuild_menus()

    def _on_select_by_kind(self, kind: str) -> None:
        """Handle selection of prims by kind.

        Args:
            kind: USD kind to select.
        """

        async def select_func() -> None:
            """Execute a select-by-kind action on the next update."""
            await omni.kit.app.get_app().next_update_async()
            omni.kit.commands.execute("SelectKindCommand", kind=kind)

        asyncio.ensure_future(select_func())

    def _build_selection_kind_menu(self) -> None:
        """Build the Select by Kind submenu items."""
        self._selection_kind_menu_list.clear()

        for kind in self._usd_kinds_display():
            self._selection_kind_menu_list.append(
                MenuItemDescription(
                    name=str(kind).capitalize(), onclick_action=("isaacsim.gui.menu.edit_menu", "select_by_kind", kind)
                )
            )

        for kind in self._plugin_kinds():
            self._selection_kind_menu_list.append(
                MenuItemDescription(
                    name=str(kind).capitalize(), onclick_action=("isaacsim.gui.menu.edit_menu", "select_by_kind", kind)
                )
            )

    def _build_edit_menu(self) -> None:
        """Build the Edit menu items and register update hooks."""

        def is_edit_type_enabled(type_name: str) -> bool:
            """Return whether an edit feature is enabled by settings.

            Args:
                type_name: Settings suffix for the feature.

            Returns:
                True if the feature is enabled or unset, False otherwise.
            """
            settings = carb.settings.get_settings()
            enabled = settings.get(f"/exts/isaacsim.gui.menu/enable{type_name}")
            if enabled is True or enabled is False:
                return enabled
            return True

        self._build_recent_menu()
        self._build_selection_kind_menu()
        # setup menus
        self._edit_select_menu = [
            MenuItemDescription(name="Select Recent", sub_menu=self._select_recent_menu_list),
            MenuItemDescription(
                name="Select All",
                onclick_action=("omni.kit.selection", "all"),
                hotkey=(carb.input.KEYBOARD_MODIFIER_FLAG_CONTROL, carb.input.KeyboardInput.A),
            ),
            MenuItemDescription(
                name="Select None",
                enable_fn=EditMenuExtension.prim_selected,
                onclick_action=("omni.kit.selection", "none"),
                hotkey=(0, carb.input.KeyboardInput.ESCAPE),
            ),
            MenuItemDescription(
                name="Select Invert",
                enable_fn=EditMenuExtension.prim_selected,
                onclick_action=("isaacsim.gui.menu.edit_menu", "selection_invert"),
                hotkey=(carb.input.KEYBOARD_MODIFIER_FLAG_CONTROL, carb.input.KeyboardInput.I),
            ),
            MenuItemDescription(
                name="Select Parent",
                enable_fn=EditMenuExtension.prim_selected,
                onclick_action=("isaacsim.gui.menu.edit_menu", "selection_parent"),
                hotkey=(0, carb.input.KeyboardInput.UP),
            ),
            MenuItemDescription(
                name="Select Leaf",
                enable_fn=EditMenuExtension.prim_selected,
                onclick_action=("isaacsim.gui.menu.edit_menu", "selection_leaf"),
            ),
            MenuItemDescription(
                name="Select Hierarchy",
                onclick_action=("isaacsim.gui.menu.edit_menu", "selection_hierarchy"),
                enable_fn=EditMenuExtension.prim_selected,
            ),
            MenuItemDescription(
                name="Select Similar",
                enable_fn=EditMenuExtension.prim_selected,
                onclick_action=("isaacsim.gui.menu.edit_menu", "selection_similar"),
            ),
            MenuItemDescription(
                name="Create Selection Set",
                enable_fn=EditMenuExtension.prim_selected,
                onclick_action=("isaacsim.gui.menu.edit_menu", "create_selection_set"),
            ),
            MenuItemDescription(name="Select Set", sub_menu=self._selection_set_menu_list),
            MenuItemDescription(name="Select by Kind", sub_menu=self._selection_kind_menu_list),
        ]
        self._edit_menu_list = [
            MenuItemDescription(
                name="Undo",
                onclick_action=("omni.kit.commands", "undo"),
                hotkey=(carb.input.KEYBOARD_MODIFIER_FLAG_CONTROL, carb.input.KeyboardInput.Z),
            ),
            MenuItemDescription(
                name="Redo",
                onclick_action=("omni.kit.commands", "redo"),
                hotkey=(carb.input.KEYBOARD_MODIFIER_FLAG_CONTROL, carb.input.KeyboardInput.Y),
            ),
            MenuItemDescription(
                name="Repeat",
                onclick_action=("omni.kit.commands", "repeat"),
                hotkey=(carb.input.KEYBOARD_MODIFIER_FLAG_CONTROL, carb.input.KeyboardInput.R),
            ),
            MenuItemDescription(name="Select", sub_menu=self._edit_select_menu),
            MenuItemDescription(
                name="Instance",
                enable_fn=[EditMenuExtension.prim_selected, EditMenuExtension.can_be_instanced],
                onclick_action=("isaacsim.gui.menu.edit_menu", "instance_prim"),
                hotkey=(
                    carb.input.KEYBOARD_MODIFIER_FLAG_CONTROL | carb.input.KEYBOARD_MODIFIER_FLAG_SHIFT,
                    carb.input.KeyboardInput.I,
                ),
            ),
            MenuItemDescription(
                name="Duplicate",
                enable_fn=EditMenuExtension.prim_selected,
                onclick_action=("isaacsim.gui.menu.edit_menu", "duplicate_prim"),
                hotkey=(carb.input.KEYBOARD_MODIFIER_FLAG_CONTROL, carb.input.KeyboardInput.D),
            ),
            MenuItemDescription(
                name="Duplicate - All Layers",
                enable_fn=[EditMenuExtension.prim_selected, EditMenuExtension.is_not_in_live_session],
                onclick_action=("isaacsim.gui.menu.edit_menu", "duplicate_prim_and_layers"),
            ),
            MenuItemDescription(
                name="Duplicate - Collapsed",
                enable_fn=EditMenuExtension.prim_selected,
                onclick_action=("isaacsim.gui.menu.edit_menu", "duplicate_prim_and_combine_layers"),
            ),
            MenuItemDescription(
                name="Parent",
                enable_fn=[EditMenuExtension.can_prims_parent, EditMenuExtension.is_not_in_live_session],
                onclick_action=("isaacsim.gui.menu.edit_menu", "parent_prims"),
                hotkey=(0, carb.input.KeyboardInput.P),
            ),
            MenuItemDescription(
                name="Unparent",
                enable_fn=[EditMenuExtension.can_prims_unparent, EditMenuExtension.is_not_in_live_session],
                onclick_action=("isaacsim.gui.menu.edit_menu", "unparent_prims"),
                hotkey=(carb.input.KEYBOARD_MODIFIER_FLAG_SHIFT, carb.input.KeyboardInput.P),
            ),
            MenuItemDescription(
                name="Group",
                enable_fn=EditMenuExtension.prim_selected,
                onclick_action=("isaacsim.gui.menu.edit_menu", "create_xform_to_group"),
                hotkey=(carb.input.KEYBOARD_MODIFIER_FLAG_CONTROL, carb.input.KeyboardInput.G),
            ),
            MenuItemDescription(
                name="Ungroup",
                enable_fn=EditMenuExtension.prim_selected,
                onclick_action=("isaacsim.gui.menu.edit_menu", "ungroup_prims"),
            ),
            MenuItemDescription(
                name="Toggle Visibility",
                enable_fn=EditMenuExtension.prim_selected,
                onclick_action=("isaacsim.gui.menu.edit_menu", "toggle_visibillity"),
                hotkey=(0, carb.input.KeyboardInput.H),
            ),
            MenuItemDescription(
                name="Deactivate",
                onclick_action=("isaacsim.gui.menu.edit_menu", "deactivate_prims"),
                hotkey=(carb.input.KEYBOARD_MODIFIER_FLAG_SHIFT, carb.input.KeyboardInput.DEL),
            ),
            MenuItemDescription(
                name="Delete",
                enable_fn=EditMenuExtension.prim_selected,
                onclick_action=("isaacsim.gui.menu.edit_menu", "delete_prim"),
                hotkey=(0, carb.input.KeyboardInput.DEL),
            ),
            MenuItemDescription(
                name="Delete - All Layers",
                enable_fn=[EditMenuExtension.prim_selected, EditMenuExtension.is_not_in_live_session],
                onclick_action=("isaacsim.gui.menu.edit_menu", "delete_prim_all_layers"),
            ),
            MenuItemDescription(
                name="Rename",
                enable_fn=[
                    EditMenuExtension.prim_selected,
                    EditMenuExtension.is_one_prim_selected,
                    EditMenuExtension.can_delete,
                ],
                onclick_action=("isaacsim.gui.menu.edit_menu", "menu_rename_prim_dialog"),
                hotkey=(0, carb.input.KeyboardInput.F2),
                appear_after="Delete - All Layers",
            ),
            MenuItemDescription(
                name="Hide Unselected",
                onclick_action=("omni.kit.selection", "HideUnselected"),
            ),
            MenuItemDescription(
                name="Unhide All",
                onclick_action=("omni.kit.selection", "UnhideAllPrims"),
            ),
            MenuItemDescription(
                name="Focus",
                enable_fn=EditMenuExtension.prim_selected,
                onclick_action=("isaacsim.gui.menu.edit_menu", "focus_prim"),
                hotkey=(0, carb.input.KeyboardInput.F),
            ),
            MenuItemDescription(
                name="Toggle Visualization Mode",
                onclick_action=("omni.kit.viewport.actions", "toggle_global_visibility"),
            ),
        ]
        if is_edit_type_enabled("CaptureScreenshot"):
            self._edit_menu_list.append(
                MenuItemDescription(
                    name="Capture Screenshot",
                    onclick_action=("isaacsim.gui.menu.edit_menu", "capture_screenshot"),
                    hotkey=(0, carb.input.KeyboardInput.F10),
                )
            )

        omni.kit.menu.utils.add_menu_items(self._edit_menu_list, "Edit", -9)

        event_stream = carb.eventdispatcher.get_eventdispatcher()
        self._stage_event_sub = event_stream.observe_event(
            event_name=omni.usd.get_context().stage_event_name(omni.usd.StageEventType.SELECTION_CHANGED),
            on_event=self._on_stage_event,
            observer_name="isaacsim.gui.menu stage watcher",
        )

    def _on_stage_event(self, event: carb.events.IEvent) -> None:
        """Refresh edit menu items after a stage event.

        Args:
            event: Stage event payload.
        """

        async def refresh_menu_items() -> None:
            """Refresh the Edit menu items."""
            omni.kit.menu.utils.refresh_menu_items("Edit")

        asyncio.ensure_future(refresh_menu_items())

    def _register_context_menu(self) -> None:
        """Register Edit-related context menus."""
        import omni.kit.context_menu

        select_context_menu_dict = {
            "name": {
                "Select": [
                    {"name": {"Select Recent": self._select_recent_menu_list}},
                    {"name": "Select All", "onclick_action": ("omni.kit.selection", "all")},
                    {"name": "Select None", "onclick_action": ("omni.kit.selection", "none")},
                    {"name": "Select Invert", "onclick_action": ("isaacsim.gui.menu.edit_menu", "selection_invert")},
                    {"name": "Select Parent", "onclick_action": ("isaacsim.gui.menu.edit_menu", "selection_parent")},
                    {"name": "Select Leaf", "onclick_action": ("isaacsim.gui.menu.edit_menu", "selection_leaf")},
                    {
                        "name": "Select Hierarchy",
                        "onclick_action": ("isaacsim.gui.menu.edit_menu", "selection_hierarchy"),
                    },
                    {"name": "Select Similar", "onclick_action": ("isaacsim.gui.menu.edit_menu", "selection_similar")},
                    {"name": ""},
                    {"name": {"Select by Kind": self._selection_kind_menu_list}},
                ]
            },
            "glyph": "none.svg",
            "appear_after": "",
        }
        self._context_menus.append(
            omni.kit.context_menu.add_menu(select_context_menu_dict, "MENU", "omni.kit.window.viewport")
        )

    def _unregister_context_menu(self) -> None:
        """Unregister context menus and release resources."""
        for menu in self._context_menus:
            menu.release()
        self._context_menus.clear()

    @staticmethod
    def prim_selected() -> bool:
        """Check whether one or more prims are selected.

        Returns:
            True if any prim is selected, False otherwise.

        Example:
            .. code-block:: python

                if EditMenuExtension.prim_selected():
                    print("Selection exists")
        """
        paths = omni.usd.get_context().get_selection().get_selected_prim_paths()
        return bool(paths)

    @staticmethod
    def is_in_live_session() -> bool:
        """Check whether the stage is in a live session.

        Returns:
            True if the stage is live-syncing, False otherwise.

        Example:
            .. code-block:: python

                if EditMenuExtension.is_in_live_session():
                    print("Live session")
        """
        usd_context = omni.usd.get_context()
        live_syncing = layers.get_layers(usd_context).get_live_syncing()
        return live_syncing.is_stage_in_live_session()

    @staticmethod
    def is_not_in_live_session() -> bool:
        """Check whether the stage is not in a live session.

        Returns:
            True if the stage is not live-syncing, False otherwise.

        Example:
            .. code-block:: python

                if EditMenuExtension.is_not_in_live_session():
                    print("Not live")
        """
        return not EditMenuExtension.is_in_live_session()

    @staticmethod
    def can_delete() -> bool:
        """Check whether selected prims can be deleted.

        Returns:
            True if all selected prims are deletable, False otherwise.

        Example:
            .. code-block:: python

                if EditMenuExtension.can_delete():
                    print("Deletion allowed")
        """
        paths = omni.usd.get_context().get_selection().get_selected_prim_paths()
        stage = omni.usd.get_context().get_stage()
        if not stage:
            return False
        for path in paths:
            prim = stage.GetPrimAtPath(path)
            if not prim or not prim.IsValid():
                return False
            no_delete = prim.GetMetadata("no_delete")
            if no_delete is not None and no_delete is True:
                return False
        return True

    @staticmethod
    def is_one_prim_selected() -> bool:
        """Check whether a single prim is selected.

        Returns:
            True when exactly one prim is selected, False otherwise.

        Example:
            .. code-block:: python

                if EditMenuExtension.is_one_prim_selected():
                    print("Single selection")
        """
        paths = omni.usd.get_context().get_selection().get_selected_prim_paths()
        return len(paths) == 1

    @staticmethod
    def can_prims_parent() -> bool:
        """Check whether the current selection can be parented.

        Returns:
            True if two or more prims are selected, False otherwise.

        Example:
            .. code-block:: python

                if EditMenuExtension.can_prims_parent():
                    print("Can parent")
        """
        paths = omni.usd.get_context().get_selection().get_selected_prim_paths()
        if paths:
            return len(paths) > 1
        return False

    @staticmethod
    def can_prims_unparent() -> bool:
        """Check whether the current selection can be unparented.

        Returns:
            True if any prims are selected, False otherwise.

        Example:
            .. code-block:: python

                if EditMenuExtension.can_prims_unparent():
                    print("Can unparent")
        """
        paths = omni.usd.get_context().get_selection().get_selected_prim_paths()
        if paths:
            return True
        return False

    @staticmethod
    def can_be_instanced() -> bool:
        """Check whether selected prims can be instanced.

        Returns:
            True if all selected prims are instanceable, False otherwise.

        Example:
            .. code-block:: python

                if EditMenuExtension.can_be_instanced():
                    print("Instanceable")
        """
        allowed_types_for_instancing = ["Xform"]
        stage = omni.usd.get_context().get_stage()
        if not stage:
            return False
        paths = omni.usd.get_context().get_selection().get_selected_prim_paths()
        for path in paths:
            prim = stage.GetPrimAtPath(path)
            if not prim or prim.GetTypeName() not in allowed_types_for_instancing:
                return False
        return True

    @staticmethod
    def instance_prim() -> None:
        """Create instances for the selected prims.

        Example:
            .. code-block:: python

                EditMenuExtension.instance_prim()
        """
        usd_context = omni.usd.get_context()
        paths = usd_context.get_selection().get_selected_prim_paths()
        if not paths:
            EditMenuExtension.post_notification("Cannot instance prim as no prims are selected")
            return

        with omni.kit.usd.layers.active_authoring_layer_context(usd_context):
            paths = usd_context.get_selection().get_selected_prim_paths()
            omni.kit.commands.execute("CreateInstances", paths_from=paths)

    # When combine_layers is True, it means to duplicate with flattening references.
    @staticmethod
    def duplicate_prim(duplicate_layers: bool, combine_layers: bool) -> None:
        """Duplicate selected prims with layer options.

        Args:
            duplicate_layers: Whether to duplicate across all layers.
            combine_layers: Whether to flatten references during duplication.

        Example:
            .. code-block:: python

                EditMenuExtension.duplicate_prim(duplicate_layers=False, combine_layers=False)
        """
        paths = omni.usd.get_context().get_selection().get_selected_prim_paths()
        if not paths:
            EditMenuExtension.post_notification("Cannot duplicate prim as no prims are selected")
            return

        if EditMenuExtension.is_in_live_session() and duplicate_layers and not combine_layers:
            warn = "Destructive duplication is not supported in live-syncing."
            carb.log_warn(warn)
            EditMenuExtension.post_notification(warn)
            return

        paths = omni.usd.get_context().get_selection().get_selected_prim_paths()
        usd_context = omni.usd.get_context()
        with layers.active_authoring_layer_context(usd_context):
            omni.kit.commands.execute(
                "CopyPrims",
                paths_from=paths,
                duplicate_layers=duplicate_layers,
                combine_layers=combine_layers,
                flatten_references=combine_layers,
            )

    @staticmethod
    def parent_prims() -> None:
        """Parent selected prims to the last selected prim.

        Example:
            .. code-block:: python

                EditMenuExtension.parent_prims()
        """
        stage = omni.usd.get_context().get_stage()
        if not stage:
            EditMenuExtension.post_notification("Cannot parent prim as no stage is loaded")
            return

        if not EditMenuExtension.can_prims_parent():
            EditMenuExtension.post_notification("Cannot parent prim as two or more prims are not selected")
            return

        if EditMenuExtension.is_in_live_session():
            EditMenuExtension.post_notification("Parent is not supported in live-syncing.")
            return

        paths = omni.usd.get_context().get_selection().get_selected_prim_paths()
        if paths:

            def prim_renamed(old_prim_name: Sdf.Path, new_prim_name: Sdf.Path) -> None:
                """Update cached selection paths after a rename.

                Args:
                    old_prim_name: Original prim path.
                    new_prim_name: Updated prim path.
                """
                try:
                    index = paths.index(old_prim_name.pathString)
                    paths[index] = new_prim_name.pathString
                except Exception as exc:  # pylint: disable=broad-except
                    import traceback

                    carb.log_warn(f"prim_renamed error {exc}")
                    traceback.print_stack(file=sys.stdout)

            parent_path = paths.pop()
            prim = stage.GetPrimAtPath(parent_path)
            if prim and prim.IsInstanceable():
                EditMenuExtension.post_notification(f"{parent_path} cannot be parent as its Instanceable")
                return

            settings = carb.settings.get_settings()
            keep_transform = settings.get(KEEP_TRANSFORM_FOR_REPARENTING)
            if keep_transform is None:
                keep_transform = True

            omni.kit.commands.execute(
                "ParentPrims",
                parent_path=parent_path,
                child_paths=paths,
                on_move_fn=prim_renamed,
                keep_world_transform=keep_transform,
            )
            omni.usd.get_context().get_selection().set_selected_prim_paths(paths, True)

    @staticmethod
    def unparent_prims() -> None:
        """Unparent selected prims.

        Example:
            .. code-block:: python

                EditMenuExtension.unparent_prims()
        """
        stage = omni.usd.get_context().get_stage()
        if not stage:
            EditMenuExtension.post_notification("Cannot unparent prim as no stage is loaded")
            return

        if not EditMenuExtension.can_prims_unparent():
            EditMenuExtension.post_notification("Cannot unparent prim as no prims are not selected")
            return

        if EditMenuExtension.is_in_live_session():
            EditMenuExtension.post_notification("Unparent is not supported in live-syncing.")
            return

        paths = omni.usd.get_context().get_selection().get_selected_prim_paths()
        if paths:
            settings = carb.settings.get_settings()
            keep_transform = settings.get(KEEP_TRANSFORM_FOR_REPARENTING)
            if keep_transform is None:
                keep_transform = True

            omni.kit.commands.execute("UnparentPrims", paths=paths, keep_world_transform=keep_transform)

    @staticmethod
    def create_xform_to_group() -> bool:
        """Group selected prims under a new Xform.

        Returns:
            True if the group was created, False otherwise.

        Example:
            .. code-block:: python

                success = EditMenuExtension.create_xform_to_group()
        """
        usd_context = omni.usd.get_context()
        paths = usd_context.get_selection().get_selected_prim_paths()
        if not paths:
            EditMenuExtension.post_notification("Cannot Group as no prims are selected")
            return False

        stage = usd_context.get_stage()
        if not stage:
            EditMenuExtension.post_notification("Cannot Group prim as no stage is loaded")
            return False

        if stage.HasDefaultPrim() and stage.GetDefaultPrim().GetPath().pathString in paths:
            EditMenuExtension.post_notification("Cannot Group default prim")
            return False

        with omni.kit.usd.layers.active_authoring_layer_context(usd_context):
            omni.kit.commands.execute("GroupPrims", prim_paths=paths, destructive=False)
        return True

    @staticmethod
    def ungroup_prims() -> bool:
        """Ungroup selected prims.

        Returns:
            True if ungrouping succeeded, False otherwise.

        Example:
            .. code-block:: python

                success = EditMenuExtension.ungroup_prims()
        """
        usd_context = omni.usd.get_context()
        paths = usd_context.get_selection().get_selected_prim_paths()
        if not paths:
            EditMenuExtension.post_notification("Cannot Ungroup as no prims are selected")
            return False

        stage = usd_context.get_stage()
        if not stage:
            EditMenuExtension.post_notification("Cannot Ungroup prim as no stage is loaded")
            return False

        with omni.kit.usd.layers.active_authoring_layer_context(usd_context):
            from omni.usd.commands import UngroupPrimsCommand

            result = omni.kit.commands.execute("UngroupPrims", prim_paths=paths, destructive=False)
            if len(result) == 2:
                if result[1] == UngroupPrimsCommand.ExitCode.Success:
                    return True

                EditMenuExtension.post_notification("Failed to Ungroup prims as no valid group parent could be found.")
                return False

        EditMenuExtension.post_notification("Failed to Ungroup prims.")
        return False

    @staticmethod
    def delete_prim(destructive: bool) -> None:
        """Delete selected prims from the stage.

        Args:
            destructive: Whether to delete prims destructively.

        Example:
            .. code-block:: python

                EditMenuExtension.delete_prim(destructive=False)
        """
        usd_context = omni.usd.get_context()
        paths = usd_context.get_selection().get_selected_prim_paths()
        if not paths:
            EditMenuExtension.post_notification("Cannot Delete as no prims are selected")
            return

        stage = usd_context.get_stage()
        paths = [Sdf.Path(path) for path in paths if path]
        paths = Sdf.Path.RemoveDescendentPaths(paths)

        all_can_be_removed_non_destructively = True
        has_instance_proxies = False
        non_instance_proxy_paths = []
        for path in paths:
            prim = stage.GetPrimAtPath(path)
            if not prim:
                continue

            # Instance proxy cannot be removed.
            if prim.IsInstanceProxy():
                has_instance_proxies = True
            else:
                non_instance_proxy_paths.append(path)
                if not omni.usd.commands.prim_can_be_removed_without_destruction(usd_context, path):
                    all_can_be_removed_non_destructively = False

            if has_instance_proxies and not all_can_be_removed_non_destructively:
                break

        paths = non_instance_proxy_paths

        if has_instance_proxies:
            nm.post_notification("Cannot delete instance proxies.", status=nm.NotificationStatus.WARNING)

        if not paths:
            return

        # Silently removing all since they can be removed safely.
        if all_can_be_removed_non_destructively:
            omni.kit.commands.execute("DeletePrims", paths=paths, destructive=destructive)
        else:
            if EditMenuExtension.is_in_live_session():
                if destructive:
                    EditMenuExtension.post_notification("Delete prims destructively is not supported in live-syncing.")
                    return

                # When it's in live session, it should always deactivate those prims without warning.
                omni.kit.commands.execute("DeletePrims", paths=paths, destructive=destructive)
                return

            if len(paths) > 1:
                warning_text = (
                    "Several prims cannot be deleted directly as they are not defined in the current authoring layer "
                    "or removing them has dangling overs left in other layers. Do you want to deactivate them instead?"
                )
            else:
                stage = usd_context.get_stage()
                prim = stage.GetPrimAtPath(paths[0])
                if not prim:
                    return

                current_layer = stage.GetEditTarget().GetLayer()
                prim_spec = current_layer.GetPrimAtPath(paths[0])
                if omni.usd.check_ancestral(prim):
                    warning_text = "Cannot delete ancestral prim. Do you want to deactivate it instead?"
                elif not prim_spec or prim_spec.specifier == Sdf.SpecifierOver:
                    warning_text = (
                        "Prim is not defined in the current authoring layer. Do you want to deactivate it instead?"
                    )
                else:  # specifier == SpecifierDef
                    warning_text = (
                        "Prim has opinions in several layers, and removing it will create dangling overs. "
                        "Continuing the operation will deactivate the prim."
                    )

            def deactivating_prims() -> None:
                """Deactivate prims after user confirmation."""
                omni.kit.commands.execute(
                    "ToggleActivePrims", prim_paths=paths, stage_or_context=usd_context, active=False
                )

            deactivate_button = nm.NotificationButtonInfo("Deactivate", on_complete=deactivating_prims)
            cancel_button = nm.NotificationButtonInfo("Cancel", on_complete=None)
            nm.post_notification(
                warning_text,
                hide_after_timeout=False,
                status=nm.NotificationStatus.WARNING,
                button_infos=[deactivate_button, cancel_button],
            )

    @staticmethod
    def focus_prim() -> None:
        """Focus the viewport camera on selected prims.

        Example:
            .. code-block:: python

                EditMenuExtension.focus_prim()
        """
        from omni.kit.viewport.utility import frame_viewport_selection

        frame_viewport_selection()

    @staticmethod
    def toggle_visibillity() -> None:
        """Toggle visibility of selected prims in the viewport.

        Example:
            .. code-block:: python

                EditMenuExtension.toggle_visibillity()
        """
        paths = omni.usd.get_context().get_selection().get_selected_prim_paths()
        if not paths:
            EditMenuExtension.post_notification("Cannot Toggle Visibility as no prims are selected")
            return

        omni.kit.commands.execute("ToggleVisibilitySelectedPrims", selected_paths=paths)

    @staticmethod
    def deactivate_prims() -> None:
        """Deactivate selected prims.

        Example:
            .. code-block:: python

                EditMenuExtension.deactivate_prims()
        """
        usd_context = omni.usd.get_context()
        paths = usd_context.get_selection().get_selected_prim_paths()
        if not paths:
            return

        omni.kit.commands.execute("ToggleActivePrims", prim_paths=paths, stage_or_context=usd_context, active=False)

    @staticmethod
    def toggle_global_visibility() -> object | None:
        """Toggle global visualization mode in the viewport.

        Example:
            .. code-block:: python

                EditMenuExtension.toggle_global_visibility()

        Returns:
            Result of the action execution, or None if unavailable.
        """
        carb.log_error(
            "isaacsim.gui.menu.toggle_global_visibility is deprecated, use omni.kit.viewport.actions.toggle_global_visibility"
        )
        action_registry = omni.kit.actions.core.get_action_registry()
        exc_action = action_registry.get_action("omni.kit.viewport.actions", "toggle_global_visibility")
        if exc_action:
            return exc_action.execute()

        carb.log_error("omni.kit.viewport.actions must be enabled")
        return None

    def get_screenshot_path(self, settings: carb.settings.ISettings) -> str:
        """Get the screenshot save path from settings.

        Args:
            settings: Settings interface for reading capture paths.

        Returns:
            The screenshot directory path.

        Example:
            .. code-block:: python

                settings = carb.settings.get_settings()
                path = _extension_instance.get_screenshot_path(settings)
        """
        # get path from /persistent/app/captureFrame/path
        screenshot_path = settings.get(PERSISTENT_SETTINGS_PREFIX + CAPTURE_FRAME_PATH)
        if not screenshot_path:
            # get path from /app/captureFrame/path
            screenshot_path = settings.get(CAPTURE_FRAME_PATH)
            if not screenshot_path:
                # use cache dir as tempdirs get deleted on exit
                screenshot_path = carb.tokens.get_tokens_interface().resolve("${cache}")

        return screenshot_path

    @staticmethod
    def capture_screenshot(on_complete_fn: Callable[[bool, str], None] | None = None) -> None:
        """Capture a screenshot of the active viewport or the app.

        Args:
            on_complete_fn: Optional callback invoked after capture completes.

        Example:
            .. code-block:: python

                EditMenuExtension.capture_screenshot()
        """
        from datetime import datetime

        now = datetime.now()
        date_time = now.strftime("%Y-%m-%d %H.%M.%S")
        capture_filename = f"capture.{date_time}.png"

        # Check if the user specified the screenshots folder.
        settings = carb.settings.get_settings()
        if _extension_instance is None:
            carb.log_error("Capture screenshot failed: extension instance is not available.")
            return
        screenshot_path = _extension_instance.get_screenshot_path(settings)
        if screenshot_path:
            if os.path.isdir(screenshot_path):
                capture_filename = os.path.join(screenshot_path, capture_filename).replace("\\", "/").replace("//", "/")
            else:
                carb.log_error(f"Can't save screenshot to {str(screenshot_path)} because it is not a directory")

        async def capture_frame(capture_filename: str, on_complete_fn: Callable[[bool, str], None] | None) -> None:
            """Capture a frame and invoke the completion callback.

            Args:
                capture_filename: Output filename for the capture.
                on_complete_fn: Callback to invoke after capture completes.
            """
            import omni.kit.app

            carb.log_warn(f"Capturing {capture_filename}")

            # wait for next frame so menu has closed
            await omni.kit.app.get_app().next_update_async()

            capture_viewport = settings.get(PERSISTENT_SETTINGS_PREFIX + "/app/captureFrame/viewport")
            success = False
            with contextlib.suppress(ImportError):
                if not capture_viewport:
                    import omni.renderer_capture

                    renderer_capture = omni.renderer_capture.acquire_renderer_capture_interface()
                    renderer_capture.capture_next_frame_swapchain(capture_filename)
                    success = True
                else:
                    from omni.kit.viewport.utility import capture_viewport_to_file, get_active_viewport

                    viewport = get_active_viewport()
                    if viewport:
                        await capture_viewport_to_file(
                            viewport, file_path=capture_filename, is_hdr=False
                        ).wait_for_result()
                        success = True

            if not success:
                carb.log_error("Failed to capture " + ("viewport" if capture_viewport else "") + "screenshot")

            if on_complete_fn:
                if success:
                    # wait for screenshot to complete
                    wait_counter = 50
                    while os.path.isfile(capture_filename) is False:
                        carb.log_info(f"waiting for {capture_filename} to be created")
                        await omni.kit.app.get_app().next_update_async()
                        await omni.kit.app.get_app().next_update_async()
                        wait_counter -= 1
                        if not wait_counter:
                            carb.log_error(f"Timeout waiting for {capture_filename} to be created")
                            break

                await omni.kit.app.get_app().next_update_async()
                await omni.kit.app.get_app().next_update_async()
                on_complete_fn(success, capture_filename)

        # wait for next frame so menu has closed
        asyncio.ensure_future(capture_frame(capture_filename, on_complete_fn))

    @staticmethod
    def _rename_viewport_active_camera(old_prim_name: Sdf.Path, new_prim_name: Sdf.Path) -> None:
        """Update the active viewport camera path after a rename.

        Args:
            old_prim_name: Old camera prim path.
            new_prim_name: New camera prim path.
        """
        try:
            from omni.kit.viewport.utility import get_active_viewport

            viewport = get_active_viewport()
            if viewport and viewport.camera_path == old_prim_name:
                viewport.camera_path = new_prim_name or "/OmniverseKit_Persp"
        except ImportError:
            pass

    @staticmethod
    def rename_prim(
        stage: Usd.Stage,
        prim: Usd.Prim,
        window: ui.Widget | None,
        field_widget: ui.StringField | None,
    ) -> None:
        """Rename a prim using the provided UI field.

        Args:
            stage: Stage where the prim resides.
            prim: Prim to rename.
            window: UI window associated with the rename operation.
            field_widget: UI field that contains the new prim name.

        Example:
            .. code-block:: python

                EditMenuExtension.rename_prim(stage, prim, window, field_widget)
        """
        if window:
            window.visible = False
        if field_widget:

            def select_new_prim(old_prim_name: Sdf.Path, new_prim_name: Sdf.Path) -> None:
                """Select the renamed prim in the stage.

                Args:
                    old_prim_name: Original prim path.
                    new_prim_name: Updated prim path.
                """
                omni.usd.get_context().get_selection().set_selected_prim_paths([new_prim_name.pathString], True)

            if prim.GetPath().name != field_widget.model.get_value_as_string():
                old_prim_name = prim.GetPath()
                new_prim_name = prim.GetPath().GetParentPath()
                new_prim_name = new_prim_name.AppendChild(
                    Tf.MakeValidIdentifier(field_widget.model.get_value_as_string())
                )
                new_prim_name = omni.usd.get_stage_next_free_path(stage, new_prim_name.pathString, False)
                on_move_fn = select_new_prim

                if Sdf.Path.IsValidPathString(new_prim_name):
                    EditMenuExtension._rename_viewport_active_camera(old_prim_name, new_prim_name)
                    move_dict = {old_prim_name: new_prim_name}
                    omni.kit.commands.execute(
                        "MovePrims", paths_to_move=move_dict, on_move_fn=on_move_fn, destructive=False
                    )
                else:
                    EditMenuExtension.post_notification(
                        f"Cannot rename {old_prim_name} to {new_prim_name} as its not a valid USD path"
                    )

    def menu_rename_prim_dialog(self) -> object | None:
        """Open a rename dialog for the currently selected prim.

        Returns:
            The rename task result if handled by the stage widget, None otherwise.

        Example:
            .. code-block:: python

                EditMenuExtension("ext.id").menu_rename_prim_dialog()
        """
        import omni.usd

        paths = omni.usd.get_context().get_selection().get_selected_prim_paths()
        if not paths:
            EditMenuExtension.post_notification("Cannot Rename prim as no prims are selected")
            return None

        if len(paths) != 1:
            EditMenuExtension.post_notification("Cannot Rename more then one prim")
            return None

        stage = omni.usd.get_context().get_stage()
        if not stage or not paths:
            return None

        # try to use omni.kit.widget.stage rename...
        try:
            import omni.kit.window.stage

            window = ui.Workspace.get_window("Stage")
            stage_widget = window.get_widget()
            items = stage_widget.get_treeview().selection
            if len(items) == 1:
                return stage_widget.get_delegate().get_name_column_delegate().rename_item(items[0])
        except ModuleNotFoundError:
            pass

        # do dialog rename
        prim = stage.GetPrimAtPath(paths[0])
        window = ui.Window(
            "Rename " + prim.GetPath().name + "###context_menu_rename",
            width=200,
            height=0,
            flags=ui.WINDOW_FLAGS_NO_RESIZE | ui.WINDOW_FLAGS_NO_SCROLLBAR | ui.WINDOW_FLAGS_MODAL,
        )
        with window.frame:
            with ui.VStack(
                height=0,
                spacing=5,
                name="top_level_stack",
                style={"VStack::top_level_stack": {"margin": 5}, "Button": {"margin": 0}},
            ):

                async def focus(field: ui.StringField) -> None:
                    """Focus the field on the next update.

                    Args:
                        field: Field to focus.
                    """
                    await omni.kit.app.get_app().next_update_async()
                    field.focus_keyboard()

                new_name_widget = ui.StringField()
                new_name_widget.model.set_value(prim.GetPath().name)
                new_name_widget.focus_keyboard()

                ui.Spacer(width=5, height=5)
                with ui.HStack(spacing=5):
                    ui.Button(
                        "Ok",
                        clicked_fn=partial(EditMenuExtension.rename_prim, stage, prim, window, new_name_widget),
                    )
                    ui.Button("Cancel", clicked_fn=partial(EditMenuExtension.rename_prim, stage, prim, window, None))

                    editing_started = False

                    def on_begin() -> None:
                        """Mark the start of text editing."""
                        nonlocal editing_started
                        editing_started = True

                    def on_end() -> None:
                        """Mark the end of text editing."""
                        nonlocal editing_started
                        editing_started = False

                    def window_pressed_key(key_index: int, key_flags: int, key_down: bool) -> None:
                        """Handle key presses in the rename dialog.

                        Args:
                            key_index: Keyboard key code.
                            key_flags: Modifier flags.
                            key_down: Whether the key was pressed.
                        """
                        nonlocal editing_started
                        key_mod = key_flags & ~ui.Widget.FLAG_WANT_CAPTURE_KEYBOARD
                        if (
                            carb.input.KeyboardInput(key_index)
                            in [carb.input.KeyboardInput.ENTER, carb.input.KeyboardInput.NUMPAD_ENTER]
                            and key_mod == 0
                            and key_down
                            and editing_started
                        ):
                            on_end()
                            EditMenuExtension.rename_prim(stage, prim, window, new_name_widget)

                    new_name_widget.model.add_begin_edit_fn(lambda m: on_begin())
                    new_name_widget.model.add_end_edit_fn(lambda m: on_end())
                    window.set_key_pressed_fn(window_pressed_key)
                    asyncio.ensure_future(focus(new_name_widget))

        return None


def get_extension_path(sub_directory: str) -> str:
    """Return the extension path, optionally joined with a subdirectory.

    Args:
        sub_directory: Subdirectory to append to the base path.

    Returns:
        The normalized path for the extension or subdirectory.

    Example:
        .. code-block:: python

            data_path = get_extension_path("data")
    """
    path = _extension_path or ""
    if sub_directory:
        path = os.path.normpath(os.path.join(path, sub_directory))
    return path
