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

"""Application utilities for Isaac Sim.

This module provides utilities for launching Kit applications and creating
desktop integration entries (application icons).
"""

from __future__ import annotations

import os
import platform
import subprocess
import sys
from typing import TYPE_CHECKING

import carb.tokens

if TYPE_CHECKING:
    from carb.settings import ISettings
    from omni.kit.app import IApp, IExtensionManager


# Desktop entry template for Linux
_DESKTOP_ENTRY_TEMPLATE = """[Desktop Entry]
Version=1.0
Name=Isaac Sim
Exec={launch_path}
Icon={icon_path}
Terminal=false
Type=Application
StartupWMClass=IsaacSim"""


def start_kit_app(
    settings: ISettings,
    app_id: str,
    console: bool = True,
    custom_args: list[str] | None = None,
) -> None:
    """Start another Kit application with inherited settings.

    Launches a new Kit application process, passing through extension folder
    settings from the current application.

    Args:
        settings: Carb settings interface for reading extension folders.
        app_id: Identifier for the Kit application to launch (e.g., "isaacsim.exp.uidoc.kit").
        console: If True, creates a new console window on Windows.
        custom_args: Additional command line arguments to pass to the application.

    Example:

        .. code-block:: python

            settings = carb.settings.get_settings()
            start_kit_app(
                settings,
                "isaacsim.exp.uidoc.kit",
                console=False,
                custom_args=["--no-window"],
            )
    """
    kit_exe_path = os.path.join(os.path.abspath(carb.tokens.get_tokens_interface().resolve("${kit}")), "kit")
    if sys.platform == "win32":
        kit_exe_path += ".exe"

    app_path = carb.tokens.get_tokens_interface().resolve("${app}")
    kit_file_path = os.path.join(app_path, app_id)

    run_args = [kit_exe_path, kit_file_path]
    if custom_args:
        run_args.extend(custom_args)

    # Pass all extension folders
    exts_folders = settings.get("/app/exts/folders")
    if exts_folders:
        for folder in exts_folders:
            run_args.extend(["--ext-folder", folder])

    if platform.system().lower() == "windows":
        creation_flags = getattr(subprocess, "CREATE_NEW_PROCESS_GROUP", 0)
        if console:
            creation_flags |= getattr(subprocess, "CREATE_NEW_CONSOLE", 0)
        subprocess.Popen(run_args, close_fds=False, creationflags=creation_flags)
    else:
        subprocess.Popen(run_args, close_fds=False)


def create_desktop_icon(app: IApp, ext_manager: IExtensionManager, ext_id: str) -> None:
    """Create a desktop application icon entry on Linux systems.

    Creates a .desktop file in the user's applications folder for easy
    application launching from the desktop environment. This function
    is a no-op on Windows.

    Args:
        app: The Kit application instance for logging.
        ext_manager: Extension manager for getting extension paths.
        ext_id: Extension identifier used to locate icon resources.

    Example:

        .. code-block:: python

            app = omni.kit.app.get_app()
            ext_manager = app.get_extension_manager()
            create_desktop_icon(app, ext_manager, "isaacsim.app.setup")
    """
    if sys.platform == "win32":
        return

    user_apps_folder = os.path.expanduser("~/.local/share/applications")
    if not os.path.exists(user_apps_folder):
        return

    extension_path = ext_manager.get_extension_path(ext_id)
    app_base_path = os.path.abspath(carb.tokens.get_tokens_interface().resolve("${app}") + "/../")
    isaac_launch_path = os.path.join(app_base_path, "isaac-sim.sh")
    icon_path = os.path.join(extension_path, "data", "omni.isaac.sim.png")

    desktop_file_path = os.path.join(user_apps_folder, "IsaacSim.desktop")
    desktop_entry = _DESKTOP_ENTRY_TEMPLATE.format(launch_path=isaac_launch_path, icon_path=icon_path)

    with open(desktop_file_path, "w") as file:
        app.print_and_log("Writing Isaac Sim icon file")
        file.write(desktop_entry)
