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

"""Isaac Sim application setup extension.

This module contains the main extension class that coordinates application
initialization including window layout, menus, desktop integration, and
optional ROS bridge setup.
"""

from __future__ import annotations

import asyncio
from typing import TYPE_CHECKING

import carb.settings
import omni.client
import omni.ext
import omni.kit.app
from isaacsim.core.version import get_version
from omni.kit.window.title import get_main_window_title

from . import app_utils, layout, menu, startup

if TYPE_CHECKING:
    from carb.settings import ISettings
    from omni.kit.app import IApp, IExtensionManager


class CreateSetupExtension(omni.ext.IExt):
    """Isaac Sim application setup extension.

    Handles initial configuration of the Isaac Sim application including window layout,
    menus, application icon, ROS bridge integration, and startup benchmarking.
    """

    def on_startup(self, ext_id: str) -> None:
        """Initialize the extension and configure the application.

        Sets up window layout, menus, application icon, and enables optional
        components like ROS bridge based on settings.

        Args:
            ext_id: Unique identifier for this extension instance.
        """
        self._settings: ISettings = carb.settings.get_settings()
        self._ext_manager: IExtensionManager = omni.kit.app.get_app().get_extension_manager()
        self._app: IApp = omni.kit.app.get_app()
        self._pending_tasks: list[asyncio.Task] = []

        self._setup_window_title()
        self._schedule_async_tasks()
        menu.setup_menus(self._show_ui_docs)
        app_utils.create_desktop_icon(self._app, self._ext_manager, ext_id)

        # Increase hang detection timeout
        omni.client.set_hang_detection_time_ms(10000)

    def on_shutdown(self) -> None:
        """Clean up resources when the extension is disabled.

        Cancels any pending async tasks to ensure clean shutdown.
        """
        menu.cleanup_menus()
        for task in self._pending_tasks:
            if not task.done():
                task.cancel()
        self._pending_tasks.clear()

    def _setup_window_title(self) -> None:
        """Configure the main window title with Isaac Sim version information."""
        window_title = get_main_window_title()
        app_version_core, app_version_prerel, *_ = get_version()
        window_title.set_app_version(app_version_core)

        self._app_title: str = self._settings.get("/app/window/title")
        self._app.print_and_log(f"{self._app_title} Version: {app_version_core}-{app_version_prerel}")

    def _schedule_async_tasks(self) -> None:
        """Schedule all async initialization tasks."""
        update_cb = self._update_without_ready

        self._pending_tasks.append(asyncio.ensure_future(layout.dock_windows(update_cb)))
        self._pending_tasks.append(asyncio.ensure_future(layout.setup_property_window(update_cb)))
        self._pending_tasks.append(
            asyncio.ensure_future(startup.enable_ros_extensions(self._settings, self._ext_manager, update_cb))
        )
        self._pending_tasks.append(
            asyncio.ensure_future(startup.await_viewport(self._app, self._ext_manager, self._app_title, update_cb))
        )

        if self._settings.get("/isaac/startup/create_new_stage"):
            self._pending_tasks.append(asyncio.ensure_future(startup.create_new_stage(update_cb)))

    def _show_ui_docs(self) -> None:
        """Launch the Omniverse UI documentation application."""
        app_utils.start_kit_app(self._settings, "isaacsim.exp.uidoc.kit")

    async def _update_without_ready(self) -> None:
        """Perform an app update while delaying the ready signal.

        This ensures the application doesn't signal readiness prematurely
        during initialization sequences.
        """
        if not self._app.is_app_ready():
            self._app.delay_app_ready("IsaacSim")
        await self._app.next_update_async()
