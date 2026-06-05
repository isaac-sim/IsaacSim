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


"""Extension entry point for the ROS 2 URDF UI workflow."""

import typing

import omni.ext
import omni.kit.actions.core
from omni.kit.menu.utils import MenuItemDescription, add_menu_items, remove_menu_items

from .robot_description import EXTENSION_NAME, RobotDescription


class Extension(omni.ext.IExt):
    """Extension that registers the ROS 2 URDF importer UI."""

    def menu_click(self, menu: typing.Any, value: bool) -> None:
        """Show the ROS 2 URDF importer window.

        Args:
            menu: Menu instance that triggered the action.
            value: Menu click value.

        Example:

        .. code-block:: python

            >>> extension = Extension()  # doctest: +SKIP
            >>> extension.menu_click(None, True)  # doctest: +SKIP
        """
        self.robot_description.show_window()

    def on_startup(self, ext_id: str) -> None:
        """Initialize the extension when it is loaded.

        Registers the import action and adds a menu item to the File menu.

        Args:
            ext_id: Extension identifier provided by the extension manager.
        """
        self.ext_id = ext_id
        self.robot_description = RobotDescription()
        action_registry = omni.kit.actions.core.get_action_registry()
        action_registry.register_action(
            self.ext_id,
            "import_from_ros2_urdf",
            lambda: self.menu_click(None, True),
            display_name=EXTENSION_NAME,
            description="Import documents from a ros2 urdf description node",
            tag="Import from ROS2 Node",
        )
        self._menu_items = [
            MenuItemDescription(
                name=EXTENSION_NAME,
                glyph="none.svg",
                appear_after=["Import", "Reopen", "Open", "New From Stage Template"],
                onclick_action=(self.ext_id, "import_from_ros2_urdf"),
            )
        ]
        add_menu_items(self._menu_items, "File")

    def deregister_actions(self) -> None:
        """Deregister actions created by the extension."""
        action_registry = omni.kit.actions.core.get_action_registry()
        action_registry.deregister_all_actions_for_extension(self.ext_id)

    def on_shutdown(self) -> None:
        """Clean up resources when the extension is unloaded.

        Removes menu items, deregisters actions, and shuts down the robot description component.
        """
        remove_menu_items(self._menu_items, "File")
        self.deregister_actions()
        self.robot_description.shutdown()
