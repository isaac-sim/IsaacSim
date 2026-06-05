# SPDX-FileCopyrightText: Copyright (c) 2018-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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

"""Menu setup utilities for Isaac Sim application.

This module provides functions for configuring application menus including
layout shortcuts and help menu items.
"""

from __future__ import annotations

import asyncio
import inspect
from collections.abc import Callable
from functools import partial
from typing import Any

import carb.input
import omni.kit.actions.core
from omni.kit.menu.utils import MenuItemDescription, add_menu_items, build_submenu_dict

from .layout import LAYOUTS_PATH, load_layout

# Module-level storage for registered actions (for cleanup)
_registered_actions = []
_ext_name = "isaacsim.app.setup"


def _load_layout_async(layout_file: str) -> None:
    """Load a layout file asynchronously.

    Args:
        layout_file: Path to the layout JSON file to load.
    """
    asyncio.ensure_future(load_layout(layout_file))


def _execute_func_async(func: Callable[[], Any]) -> None:
    """Execute an async function.

    Args:
        func: Async function to schedule.
    """
    asyncio.ensure_future(func())


def create_layout_menu_item(
    name: str,
    layout_name_or_func: str | Callable[[], Any],
    hotkey: carb.input.KeyboardInput,
) -> MenuItemDescription:
    """Create a menu item for layout operations.

    Creates a menu item that either loads a layout file or executes
    an async function when clicked.

    Args:
        name: Display name for the menu item.
        layout_name_or_func: Either a layout filename (without extension) or an async function.
        hotkey: Keyboard shortcut key (will be combined with Ctrl modifier).

    Returns:
        Configured menu item descriptor for use with the menu system.

    Example:

        .. code-block:: python

            item = create_layout_menu_item(
                "Default",
                "default",
                carb.input.KeyboardInput.KEY_1,
            )
    """
    global _registered_actions

    menu_path = f"Layouts/{name}"
    action_id = f"layout_{name.lower().replace(' ', '_')}"

    action_registry = omni.kit.actions.core.get_action_registry()

    if inspect.isfunction(layout_name_or_func):
        action_registry.register_action(
            _ext_name,
            action_id,
            partial(_execute_func_async, layout_name_or_func),
            display_name=f"Layout: {name}",
            description=f"Execute layout action: {name}",
        )
    else:
        layout_file = str(LAYOUTS_PATH / f"{layout_name_or_func}.json")
        action_registry.register_action(
            _ext_name,
            action_id,
            partial(_load_layout_async, layout_file),
            display_name=f"Load Layout: {name}",
            description=f"Load the {name} layout",
        )

    _registered_actions.append(action_id)

    return MenuItemDescription(
        name=menu_path,
        onclick_action=(_ext_name, action_id),
        hotkey=(carb.input.KEYBOARD_MODIFIER_FLAG_CONTROL, hotkey),
    )


def setup_menus(show_ui_docs_callback: Callable[[], None]) -> None:
    """Configure application menus including layouts and help items.

    Sets up the Help menu with UI documentation link and the Layouts menu
    with predefined layout options and quick save/load functionality.

    Args:
        show_ui_docs_callback: Callback function to launch the UI documentation app.

    Example:

        .. code-block:: python

            def show_docs() -> None:
                app_utils.start_kit_app(carb.settings.get_settings(), "isaacsim.exp.uidoc.kit")

            setup_menus(show_docs)
    """
    global _registered_actions

    from omni.kit.quicklayout import QuickLayout

    action_registry = omni.kit.actions.core.get_action_registry()

    # Register action for UI docs
    action_registry.register_action(
        _ext_name,
        "show_ui_docs",
        show_ui_docs_callback,
        display_name="Show Omni UI Docs",
        description="Open the Omni UI documentation",
    )
    _registered_actions.append("show_ui_docs")

    async def quick_save() -> None:
        QuickLayout.quick_save(None, None)

    async def quick_load() -> None:
        QuickLayout.quick_load(None, None)

    menu_items = [
        MenuItemDescription(name="Help/Omni UI Docs", onclick_action=(_ext_name, "show_ui_docs")),
        create_layout_menu_item("Default", "default", carb.input.KeyboardInput.KEY_1),
        create_layout_menu_item("Visual Scripting", "visualScripting", carb.input.KeyboardInput.KEY_4),
        create_layout_menu_item("Replicator", "sdg", carb.input.KeyboardInput.KEY_5),
        create_layout_menu_item("Quick Save", quick_save, carb.input.KeyboardInput.KEY_7),
        create_layout_menu_item("Quick Load", quick_load, carb.input.KeyboardInput.KEY_8),
    ]

    menu_dict = build_submenu_dict(menu_items)
    for group in menu_dict:
        add_menu_items(menu_dict[group], group)


def cleanup_menus() -> None:
    """Clean up registered menu actions.

    Example:

        .. code-block:: python

            cleanup_menus()
    """
    global _registered_actions

    action_registry = omni.kit.actions.core.get_action_registry()
    for action_id in _registered_actions:
        action_registry.deregister_action(_ext_name, action_id)
    _registered_actions = []
