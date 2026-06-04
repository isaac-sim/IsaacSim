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

"""Asset root path check utility for Isaac Sim."""

from __future__ import annotations

import asyncio
from collections.abc import Callable

import carb.settings
import omni.kit.app
import omni.ui as ui

DOCS_URL = "https://docs.isaacsim.omniverse.nvidia.com"
ASSETS_GUIDE_URL = DOCS_URL + "/latest/installation/install_faq.html#isaac-sim-setup-faq"


class AssetCheck:
    """Check that Isaac Sim assets are accessible at the configured default root path.

    On startup the check is skipped to avoid blocking the UI. Users can trigger it
    manually from the *Utilities > Check Default Assets Root Path* menu item.

    Args:
        on_visibility_changed: Optional callback invoked when window visibility changes.
    """

    def __init__(self, on_visibility_changed: Callable[[], None] | None = None) -> None:
        self._assets_check = False
        self._startup_run = True
        self._cancel_download_btn: ui.Button | None = None
        self._server_window: ui.Window | None = None
        self._check_success: ui.Window | None = None
        self._assets_server: str | None = None
        self._on_visibility_changed = on_visibility_changed

        # Run initial check (skips on first startup)
        self._await_new_scene = asyncio.ensure_future(self._assets_check_window())

    def destroy(self) -> None:
        """Release UI resources."""
        self._server_window = None
        self._check_success = None

    def is_visible(self) -> bool:
        """Return True if any asset-check window is currently visible.

        Returns:
            bool: Whether an asset-check window is visible.
        """
        if self._server_window and self._server_window.visible:
            return True
        if self._check_success and self._check_success.visible:
            return True
        return False

    def _notify_visibility_changed(self, _visible: bool | None = None) -> None:
        """Notify the visibility changed callback.

        Args:
            _visible: The visibility state of the window.
        """
        if self._on_visibility_changed:
            self._on_visibility_changed()

    def check_assets(self) -> None:
        """Trigger an asset root path check, or hide result windows if already visible.

        This is the callback invoked by the Utilities menu item.
        """
        if self.is_visible():
            if self._server_window:
                self._server_window.visible = False
                self._server_window = None
            if self._check_success:
                self._check_success.visible = False
                self._check_success = None
            self._notify_visibility_changed()
            return

        if self._cancel_download_btn and self._cancel_download_btn.visible:
            if self._server_window is not None:
                self._server_window.visible = True
            self._notify_visibility_changed()
        else:
            asyncio.ensure_future(self._assets_check_window())

    # ------------------------------------------------------------------
    # Internal helpers
    # ------------------------------------------------------------------

    @staticmethod
    def _open_browser(path: str) -> None:
        """Open *path* in the desktop web browser.

        Args:
            path: URL to open.
        """
        import platform
        import subprocess
        import webbrowser

        if platform.system().lower() == "windows":
            webbrowser.open(path)
        else:
            # use native system level open, handles snap based browsers better
            subprocess.Popen(["xdg-open", path])

    async def _assets_check_success_window(self) -> None:
        """Show a small pop-up confirming that assets were found."""
        self._check_success = ui.Window(
            "Isaac Sim Assets Check Successful",
            style={"alignment": ui.Alignment.CENTER},
            height=0,
            width=0,
            padding_x=10,
            padding_y=10,
            auto_resize=True,
            flags=ui.WINDOW_FLAGS_NO_RESIZE | ui.WINDOW_FLAGS_NO_SCROLLBAR | ui.WINDOW_FLAGS_NO_TITLE_BAR,
            visible=True,
        )
        self._check_success.set_visibility_changed_fn(self._notify_visibility_changed)

        def hide(w: ui.Window) -> None:
            w.visible = False

        with self._check_success.frame:
            with ui.VStack():
                ui.Spacer(height=1)
                ui.Label("Isaac Sim Assets found:", style={"font_size": 18}, alignment=ui.Alignment.CENTER)
                ui.Label(f"{self._assets_server}", style={"font_size": 18}, alignment=ui.Alignment.CENTER)
                ui.Spacer(height=5)
                ui.Button(
                    "OK", spacing=10, alignment=ui.Alignment.CENTER, clicked_fn=lambda w=self._check_success: hide(w)
                )
                ui.Spacer()

        await omni.kit.app.get_app().next_update_async()

    async def _assets_check_window(self) -> None:
        """Perform the actual asset root path check and show results."""
        if self._assets_check is False and self._startup_run:
            self._startup_run = False
            return

        from isaacsim.storage.native import check_server_async

        omni.kit.app.get_app().print_and_log("Checking for Isaac Sim Assets...")
        self._check_window = ui.Window("Check Isaac Sim Assets", height=120, width=600)
        with self._check_window.frame:
            with ui.VStack(height=80):
                ui.Spacer()
                ui.Label("Checking for Isaac Sim assets", alignment=ui.Alignment.CENTER, style={"font_size": 18})
                ui.Label(
                    "Please login to the Nucleus if a browser window appears",
                    alignment=ui.Alignment.CENTER,
                    style={"font_size": 18},
                )
                ui.Label(
                    "Restart of Isaac Sim is required if the browser window is closed without logging in.",
                    alignment=ui.Alignment.CENTER,
                    style={"font_size": 18},
                )
                ui.Spacer()
        await omni.kit.app.get_app().next_update_async()

        # Looks for assets root
        # Get timeout
        timeout = carb.settings.get_settings().get("/persistent/isaac/asset_root/timeout")
        if not isinstance(timeout, (int, float)):
            timeout = 10.0
        # Check /persistent/isaac/asset_root/default setting
        default_asset_root = carb.settings.get_settings().get("/persistent/isaac/asset_root/default")
        self._assets_server = await check_server_async(default_asset_root, "/Isaac", timeout)
        if self._assets_server is False:
            self._assets_server = None
        else:
            self._assets_server = default_asset_root + "/Isaac"

        self._check_window.visible = False
        self._check_window = None
        if self._assets_server is None:
            self._startup_run = False

            omni.kit.app.get_app().print_and_log(f"Warning: Isaac Sim Assets at {default_asset_root} not found")

            frame_height = 150
            self._server_window = ui.Window("Checking Isaac Sim Assets", width=350, height=frame_height, visible=True)
            self._server_window.set_visibility_changed_fn(self._notify_visibility_changed)
            with self._server_window.frame:
                with ui.VStack():
                    ui.Label("Warning: Isaac Sim Assets not found", style={"color": 0xFF00FFFF})
                    ui.Line()
                    ui.Label("See the documentation for details")
                    ui.Button("Open Documentation", clicked_fn=lambda: self._open_browser(ASSETS_GUIDE_URL))
                    ui.Spacer()
                    ui.Label("See terminal for additional information")
        else:
            omni.kit.app.get_app().print_and_log(f"Isaac Sim Assets found: {self._assets_server}")
            if not self._startup_run:
                asyncio.ensure_future(self._assets_check_success_window())
            self._startup_run = False
