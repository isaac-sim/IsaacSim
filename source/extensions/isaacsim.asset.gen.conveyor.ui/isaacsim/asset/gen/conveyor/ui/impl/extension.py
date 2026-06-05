"""Conveyor builder UI extension for Isaac Sim."""

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

"""UI extension for creating and managing conveyor belt assets in Isaac Sim."""


import gc
import weakref

import carb
import omni.ext
import omni.kit.actions
import omni.ui as ui
from isaacsim.asset.gen.conveyor import create_conveyor_belt
from isaacsim.asset.gen.conveyor.bindings._isaacsim_asset_gen_conveyor import acquire_interface as _acquire
from isaacsim.asset.gen.conveyor.bindings._isaacsim_asset_gen_conveyor import release_interface as _release
from omni.kit.menu.utils import MenuItemDescription, add_menu_items, remove_menu_items
from omni.kit.window.preferences import register_page, unregister_page

from .conveyor_builder_widget import ConveyorBuilderWidget
from .preferences import ConveyorBuilderPreferences
from .style import UI_STYLES

EXTENSION_NAME = "Conveyor Utility"


def make_menu_item_description(
    ext_id: str, name: str, onclick_fun: object, action_name: str = ""
) -> MenuItemDescription:
    """Easily replace the onclick_fn with onclick_action when creating a menu description.

    Args:
        ext_id: The extension you are adding the menu item to.
        name: Name of the menu item displayed in UI.
        onclick_fun: The function to run when clicking the menu item.
        action_name: Name for the action, in case ext_id+name don't make a unique string.

    Returns:
        A ``MenuItemDescription`` configured with the registered action.

    Note:
        ext_id + name + action_name must concatenate to a unique identifier.
    """
    action_unique = f'{ext_id.replace(" ", "_")}{name.replace(" ", "_")}{action_name.replace(" ", "_")}'
    action_registry = omni.kit.actions.core.get_action_registry()
    action_registry.deregister_action(ext_id, action_unique)
    action_registry.register_action(ext_id, action_unique, onclick_fun)
    return MenuItemDescription(name=name, onclick_action=(ext_id, action_unique))


class Extension(omni.ext.IExt):
    """UI extension for creating and managing conveyor belt assets in Isaac Sim.

    This extension provides a user interface for building conveyor belt systems through a dedicated Conveyor Builder
    window and menu integration. It enables users to create conveyor belts with customizable parameters and
    manage conveyor-related preferences. The extension integrates with Isaac Sim's Create and Tools menus to
    provide easy access to conveyor creation functionality.

    The extension includes:
    - A Conveyor Builder window with interactive controls for configuring conveyor properties
    - Menu items in the Create > Isaac Sim > Warehouse Items submenu for quick conveyor creation
    - A Tools menu item for accessing the full Conveyor Track Builder interface
    - Integration with the preferences system for conveyor-related settings
    - Support for custom material files and styling
    """

    def __init___(self) -> None:
        """Initialize the extension."""
        try:
            super().__init__()
        except Exception:
            carb.log_error(f"Error loading {EXTENSION_NAME}")

    def on_startup(self, ext_id: str) -> None:
        """Initialize the extension when it is loaded.

        Args:
            ext_id: Extension identifier provided by the extension manager.
        """
        self.widget = None
        menu_items = [
            MenuItemDescription(
                name="Warehouse Items",
                sub_menu=[
                    make_menu_item_description(ext_id, "Conveyor", lambda a=weakref.proxy(self): a._add_conveyor())
                ],
            )
        ]
        self._menu_items_2 = [
            make_menu_item_description(ext_id, "Conveyor Track Builder", lambda a=weakref.proxy(self): a.create_ui())
        ]

        self._menu_items = [MenuItemDescription(name="Isaac Sim", glyph="plug.svg", sub_menu=menu_items)]

        add_menu_items(self._menu_items, "Create")
        add_menu_items(self._menu_items_2, "Tools")

        self.__interface = _acquire()
        self.ext_id = ext_id
        self._window = ui.Window(
            "Conveyor Builder", open=True, width=305, height=425, dock=ui.DockPreference.LEFT_BOTTOM
        )
        self._window.visible = False
        self._window.set_visibility_changed_fn(self.on_visibility_changed)
        self._style_loaded = False
        self._hooks = []

        manager = omni.kit.app.get_app().get_extension_manager()

        self._hooks.append(
            manager.subscribe_to_extension_enable(
                on_enable_fn=lambda _: self._register_preferences(),
                on_disable_fn=lambda _: self._unregister_preferences(),
                ext_name="isaacsim.asset.gen.conveyor.ui",
                hook_name="isaacsim.asset.gen.conveyor.ui omni.kit.window.preferences listener",
            )
        )

    def _register_preferences(self) -> None:
        """Register the Conveyor Builder page in Kit preferences."""
        self._conveyor_preferences = register_page(ConveyorBuilderPreferences())

    def _unregister_preferences(self) -> None:
        """Remove the Conveyor Builder preferences page if it was registered."""
        if self._conveyor_preferences:
            unregister_page(self._conveyor_preferences)
            self._conveyor_preferences = None

    def create_ui(self) -> None:
        """Create the conveyor builder UI window and widget."""
        ext_path = omni.kit.app.get_app().get_extension_manager().get_extension_path(self.ext_id)

        self.ext_path = ext_path + "/omni/isaac/conveyor"
        if not self._style_loaded:
            theme = "NvidiaDark"
            self._style = UI_STYLES[theme]
            for i in [a for a in self._style if "{}" in self._style[a].get("image_url", "")]:
                self._style[i]["image_url"] = self._style[i]["image_url"].format(self.ext_path)
            self._style_loaded = True

        with self._window.frame:
            self.widget = ConveyorBuilderWidget(style=self._style)
            self._window.visible = True

    def on_visibility_changed(self, value: bool) -> None:
        """Handle window visibility change and shut down widget when hidden.

        Args:
            value: ``True`` when the window is shown, ``False`` when hidden.
        """
        if not value:
            self.widget.shutdown()

    def on_shutdown(self) -> None:
        """Clean up resources when the extension is unloaded."""
        remove_menu_items(self._menu_items, "Create")
        remove_menu_items(self._menu_items_2, "Tools")
        if self.widget:
            self.widget.shutdown()
        _release(self.__interface)
        self.__interface = None
        gc.collect()

    def _add_conveyor(self, *args: object, **kwargs: object) -> None:
        """Create a conveyor belt prim using the create_conveyor_belt API.

        Args:
            *args: Unused positional arguments (Kit callback compatibility).
            **kwargs: Unused keyword arguments (Kit callback compatibility).
        """
        stage = omni.usd.get_context().get_stage()
        selection = omni.usd.get_context().get_selection()
        selected_paths = selection.get_selected_prim_paths()
        if selected_paths:
            conveyor_prim = stage.GetPrimAtPath(selected_paths[0])
        else:
            default_prim = stage.GetDefaultPrim()
            if default_prim and default_prim.IsValid():
                conveyor_prim = default_prim
            else:
                conveyor_prim = stage.GetPrimAtPath("/")
        create_conveyor_belt(stage, conveyor_prim)
