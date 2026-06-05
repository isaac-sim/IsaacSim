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

"""Extension module that provides a browser interface for Isaac Sim examples."""

from typing import Optional

import carb.settings
import omni.ext
import omni.kit.menu.utils
import omni.ui as ui
from omni.kit.browser.folder.core import TreeFolderBrowserWidgetEx

from .model import ExampleBrowserModel
from .window import ExampleBrowserWindow

_extension_instance = None
BROWSER_MENU_ROOT = "Window"
SETTING_ROOT = "/exts/isaacsim.examples.browser/"
SETTING_VISIBLE_AFTER_STARTUP = SETTING_ROOT + "visible_after_startup"


class ExampleBrowserExtension(omni.ext.IExt):
    """Extension that provides a browser interface for Isaac Sim examples.

    This extension creates a window-based browser that organizes and displays Isaac Sim examples by category,
    allowing users to discover and access robotics examples through a tree-structured interface. The browser
    integrates with the Omniverse Kit menu system and provides both programmatic and UI-based access to examples.

    The extension registers menu items under "Window > Examples > Robotics Examples" and supports toggling
    the browser window visibility. Examples can be dynamically registered and deregistered through the
    public API methods.

    The browser window uses a tree folder widget to organize examples hierarchically by category,
    making it easy to navigate through different types of robotics examples available in Isaac Sim.
    """

    @property
    def window(self) -> Optional[ExampleBrowserWindow]:
        """The example browser window instance.

        Returns:
            The example browser window if created, None otherwise.
        """
        return self._window

    @property
    def browser_widget(self) -> Optional[TreeFolderBrowserWidgetEx]:
        """The tree folder browser widget from the example browser window.

        Returns:
            The browser widget if the window exists, None otherwise.
        """
        return self._window._widget

    def on_startup(self, ext_id: str) -> None:
        """Called when the extension starts up.

        Sets up the example browser model, window registration, menu items, and visibility based on settings.

        Args:
            ext_id: The extension identifier.
        """
        self._browser_model = ExampleBrowserModel()

        self._window = None
        ui.Workspace.set_show_window_fn(
            ExampleBrowserWindow.WINDOW_TITLE,
            self._show_window,  # pylint: disable=unnecessary-lambda
        )
        self._register_menuitem()

        visible = carb.settings.get_settings().get_as_bool(SETTING_VISIBLE_AFTER_STARTUP)
        if visible:
            self._show_window(True)

        global _extension_instance
        _extension_instance = self

    def on_shutdown(self) -> None:
        """Called when the extension shuts down.

        Cleans up menu items, actions, browser model, and destroys the window if it exists.
        """
        omni.kit.menu.utils.remove_menu_items(self._menu_entry, name=BROWSER_MENU_ROOT)
        action_registry = omni.kit.actions.core.get_action_registry()
        action_registry.deregister_action("isaacsim.examples.browser", f"open_isaac_sim_examples_browser")
        action_registry.deregister_action("isaacsim.examples.browser", f"toggle_isaac_sim_examples_browser")

        self._browser_model.destroy()

        if self._window is not None:
            self._window.destroy()
            self._window = None

        global _extension_instance
        _extension_instance = None

    def register_example(self, **kwargs: object) -> None:
        """Register an example to the browser.

        Args:
            **kwargs: Additional keyword arguments passed to the browser model.

        Keyword Args:
            name: The name of the example to register.
            category: The category of the example to register.
        """
        if "name" not in kwargs:
            raise ValueError("Missing required parameter 'name' for register_example")
        if "category" not in kwargs:
            raise ValueError("Missing required parameter 'category' for register_example")
        self._browser_model.register_example(**kwargs)

    def deregister_example(self, **kwargs: object) -> None:
        """Deregister an example from the browser.

        Args:
            **kwargs: Additional keyword arguments passed to the browser model.

        Keyword Args:
            name: The name of the example to deregister.
            category: The category of the example to deregister.

        Raises:
            ValueError: If name or category parameters are missing.
            KeyError: If the category or example doesn't exist.
        """
        # Check if required parameters are provided
        if "name" not in kwargs:
            raise ValueError("Missing required parameter 'name' for deregister_example")
        if "category" not in kwargs:
            raise ValueError("Missing required parameter 'category' for deregister_example")

        name = kwargs["name"]
        category = kwargs["category"]

        # Check if the category exists
        if category not in self._browser_model._examples:
            raise KeyError(f"Category '{category}' does not exist")

        # Check if an example with the given name exists in the category
        examples_in_category = self._browser_model._examples[category]
        example_names = [example.example.name for example in examples_in_category]

        if name not in example_names:
            raise KeyError(f"Example '{name}' does not exist in category '{category}'")

        # If all validations pass, call the inner deregister_example function
        self._browser_model.deregister_example(**kwargs)

    def _show_window(self, visible: bool) -> None:
        """Shows or hides the example browser window.

        Creates the window if it doesn't exist when showing, or toggles visibility of existing window.

        Args:
            visible: Whether to show or hide the window.
        """
        if visible:
            if self._window is None:
                self._window = ExampleBrowserWindow(self._browser_model, visible=True)
                self._window.set_visibility_changed_fn(self._on_visibility_changed)
            else:
                self._window.visible = True
        else:
            self._window.visible = False

    def _toggle_window(self) -> None:
        """Toggles the visibility of the example browser window."""
        self._show_window(not self._is_visible())

    def _register_menuitem(self) -> None:
        """Registers menu items and actions for the example browser.

        Creates actions for opening and toggling the browser window, and adds menu entries under Window > Examples.
        """
        ## register the menu action
        action_registry = omni.kit.actions.core.get_action_registry()
        action_registry.register_action(
            "isaacsim.examples.browser",
            f"open_isaac_sim_examples_browser",
            lambda: self._show_window(True),
            description=f"Open Isaac Sim Examples Browser",
        )
        action_registry.register_action(
            "isaacsim.examples.browser",
            f"toggle_isaac_sim_examples_browser",
            self._toggle_window,
            description=f"Toggle Isaac Sim Examples Browser",
        )

        self._menu_entry = [
            omni.kit.menu.utils.MenuItemDescription(
                name="Examples",
                sub_menu=[
                    omni.kit.menu.utils.MenuItemDescription(
                        name="Robotics Examples",
                        ticked=True,
                        ticked_fn=self._is_visible,
                        onclick_action=("isaacsim.examples.browser", "toggle_isaac_sim_examples_browser"),
                    )
                ],
            )
        ]
        omni.kit.menu.utils.add_menu_items(self._menu_entry, BROWSER_MENU_ROOT)

    def _is_visible(self) -> bool:
        """Checks if the example browser window is currently visible.

        Returns:
            True if the window exists and is visible, False otherwise.
        """
        return self._window.visible if self._window else False

    def _on_visibility_changed(self, visible: bool) -> None:
        """Handles visibility changes of the example browser window.

        Refreshes menu items when the window visibility state changes to ensure menu indicators
        stay synchronized with the actual window state.

        Args:
            visible: Whether the window is now visible.
        """
        omni.kit.menu.utils.refresh_menu_items(BROWSER_MENU_ROOT)


def get_instance() -> Optional[ExampleBrowserExtension]:
    """Get the current instance of the Example Browser extension.

    Returns:
        The active ExampleBrowserExtension instance, or None if the extension is not loaded.
    """
    return _extension_instance
