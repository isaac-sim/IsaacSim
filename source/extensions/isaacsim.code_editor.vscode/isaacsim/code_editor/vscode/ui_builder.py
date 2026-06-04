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

"""UI builder for Visual Studio Code integration extension."""

from __future__ import annotations

import os
import shutil
import subprocess
import weakref

import carb


class UIBuilder:
    """Manage the VS Code menu item and launcher.

    Args:
        ext_name: Extension name used for action registration.
        menu_name: Name of the menu where the item will be added.
        menu_item_name: Display name for the menu item.
        host: Host address for the Python server.
        port: Port number for the Python server.
    """

    def __init__(self, ext_name: str, menu_name: str, menu_item_name: str, host: str, port: int) -> None:
        self._menu_items: list = []
        self._ext_name = ext_name
        self._host = host
        self._port = port
        self._menu_name = menu_name
        self._menu_item_name = menu_item_name

        app_folder = carb.settings.get_settings().get_as_string("/app/folder")
        if not app_folder:
            app_folder = carb.tokens.get_tokens_interface().resolve("${app}")
        self._app_folder = os.path.normpath(os.path.join(app_folder, os.pardir))

    def startup(self) -> None:
        """Create the menu item for launching VS Code."""
        try:
            import omni.kit.actions.core
            from omni.kit.menu.utils import MenuItemDescription, add_menu_items

            action_registry = omni.kit.actions.core.get_action_registry()
            action_registry.register_action(
                self._ext_name,
                "launch_vscode",
                lambda p=weakref.proxy(self): p._launch(),
                display_name=self._menu_item_name,
                description=f"Launch {self._menu_item_name}",
            )

            self._menu_items = [
                MenuItemDescription(
                    name=self._menu_item_name,
                    onclick_action=(self._ext_name, "launch_vscode"),
                )
            ]
            add_menu_items(self._menu_items, self._menu_name)
        except ImportError:
            pass

    def shutdown(self) -> None:
        """Remove the menu item."""
        try:
            import omni.kit.actions.core
            from omni.kit.menu.utils import remove_menu_items

            remove_menu_items(self._menu_items, "Window")
            action_registry = omni.kit.actions.core.get_action_registry()
            action_registry.deregister_action(self._ext_name, "launch_vscode")
        except ImportError:
            pass
        self._menu_items = []

    def _post_notification(self, notification: str, warning: bool, hide_after_timeout: bool) -> None:
        """Show a Kit notification when the notification manager extension is available.

        Args:
            notification: The notification message to display.
            warning: Whether to display the notification as a warning.
            hide_after_timeout: Whether to hide the notification after a timeout.
        """
        try:
            import omni.kit.notification_manager as notification_manager
        except ImportError:
            pass
        else:
            if warning:
                status = notification_manager.NotificationStatus.WARNING
            else:
                status = notification_manager.NotificationStatus.INFO
            notification_manager.post_notification(
                notification,
                hide_after_timeout=hide_after_timeout,
                duration=3,
                status=status,
                button_infos=[notification_manager.NotificationButtonInfo("OK", on_complete=None)],
            )

    def _launch(self, *args: object, **kwargs: object) -> None:
        """Launch a new VS Code window pointed at the application directory.

        Args:
            *args: Variable length argument list (unused).
            **kwargs: Additional keyword arguments (unused).
        """
        code_executable = shutil.which("code")
        if code_executable is None:
            notification = (
                "Unable to find VS Code executable.\n\n"
                "Make sure VS Code is installed and accessible on the system via the command 'code'"
            )
            carb.log_warn(notification)
            self._post_notification(notification, warning=True, hide_after_timeout=False)
            return
        command = [code_executable, "-n", self._app_folder]
        carb.log_info(f"Launching VS Code: {command}")
        result = subprocess.run(command, close_fds=True)
        # check process execution
        notification = f"Serving at {self._host}:{self._port}"
        if result.returncode:
            notification += f"\n\nUnable to launch VS Code (error code: {result.returncode})"
            if result.returncode in (1, 127):
                notification += ".\nMake sure VS Code is installed and accessible on the system via the command 'code'"
            carb.log_warn(notification)

        self._post_notification(notification, warning=bool(result.returncode), hide_after_timeout=not result.returncode)
