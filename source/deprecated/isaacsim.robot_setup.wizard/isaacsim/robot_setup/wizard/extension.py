# SPDX-FileCopyrightText: Copyright (c) 2025-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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

"""Robot setup wizard extension that provides a guided interface for configuring robots in Isaac Sim."""

import weakref

import carb
import omni.ext
import omni.ui as ui
from omni.kit.menu.utils import MenuHelperExtension

from .window import RobotWizardWindow

_robot_window_instance = None


def get_window() -> RobotWizardWindow | None:
    """Get the current Isaac Sim robot window instance.

    Returns:
        The robot window instance if it exists and is still valid, otherwise None.
    """
    return _robot_window_instance if not _robot_window_instance else _robot_window_instance()


class WizardExtension(omni.ext.IExt, MenuHelperExtension):
    """Extension class that provides a robot setup wizard interface for Isaac Sim.

    This extension creates a guided wizard interface to help users configure and set up robots
    within Isaac Sim. It integrates with the Omniverse Kit menu system and workspace to provide
    easy access to robot configuration tools through a dedicated window interface.

    The extension manages the Robot Wizard window lifecycle, handles menu integration, and provides
    settings for automatic launch behavior. It extends both omni.ext.IExt for extension functionality
    and MenuHelperExtension for menu integration capabilities.
    """

    WINDOW_NAME = "Robot Wizard [Beta]"
    """Isaac Sim robot window name"""

    MENU_GROUP = "Window"
    """Isaac Sim robot menu group"""

    def __init__(self) -> None:
        super().__init__()
        self._window = None

    def on_startup(self, ext_id: str) -> None:
        """Initializes the robot wizard extension on startup.

        Sets up the window functionality, menu integration, and handles launch-on-startup settings.

        Args:
            ext_id: The extension ID provided by the Omniverse Kit SDK.
        """
        self.ext_id = ext_id

        ui.Workspace.set_show_window_fn(WizardExtension.WINDOW_NAME, self.show_window)
        # ui.Workspace.show_window(WizardExtension.WINDOW_NAME)

        self.menu_startup(WizardExtension.WINDOW_NAME, WizardExtension.WINDOW_NAME, WizardExtension.MENU_GROUP)

        self._launch_on_startup = carb.settings.get_settings().get_as_bool(
            "/persistent/exts/isaacsim.robot_setup.wizard/launch_on_startup"
        )
        self.show_window(self._launch_on_startup)

    def on_shutdown(self) -> None:
        """Shutdown function.

        Cleans up menu items and destroys the robot wizard window.
        """
        self.menu_shutdown()

        if self._window:
            self._window.destroy()
            self._window = None

    def show_window(self, value: bool) -> None:
        """Show/hide Isaac Sim robot window function.

        Args:
            value: True if window will be shown or False if window will be hidden.
        """
        global _robot_window_instance

        if value:
            if self._window is None:
                self._window = RobotWizardWindow(WizardExtension.WINDOW_NAME)
                self._window.set_visibility_changed_listener(self._visiblity_changed_fn)
                _robot_window_instance = weakref.ref(self._window)
            self._window.set_visible(value)

        elif self._window:
            self._window.set_visible(value)

    def _visiblity_changed_fn(self, visible: bool) -> None:
        """Handles robot wizard window visibility changes.

        Refreshes the menu state when the window visibility changes.

        Args:
            visible: Whether the window is currently visible.
        """
        self.menu_refresh()
