# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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

"""Top-right Python server indicator and modal management window."""

from __future__ import annotations

import asyncio
import ipaddress
import os
from collections.abc import Callable, Coroutine
from typing import Any

import carb
import omni.kit.app
import omni.ui as ui
import psutil
from isaacsim.code_editor.python_server import (
    ServerState,
    ServerStatus,
    get_server_status,
    restart_server,
    start_server,
    stop_server,
    subscribe_server_status,
)
from omni.kit.menu.utils import MenuAlignment, MenuItemDescription, add_menu_items, remove_menu_items
from omni.ui import color as cl

_MENU_NAME = "Python Server Status Widget"
_TRANSITION_STATES = {
    ServerState.STARTING,
    ServerState.STOPPING,
    ServerState.RESTARTING,
}

_STATE_PRESENTATION = {
    ServerState.STARTING: ("PY Server", cl("#42A5F5"), "Starting"),
    ServerState.RUNNING: ("PY Server", cl("#4CAF50"), "Running"),
    ServerState.STOPPING: ("PY Server", cl("#FFB74D"), "Stopping"),
    ServerState.STOPPED: ("PY Server", cl("#8A8A8A"), "Stopped"),
    ServerState.RESTARTING: ("PY Server", cl("#FFB74D"), "Restarting"),
    ServerState.ERROR: ("PY Server", cl("#EF5350"), "Error"),
}


class PythonServerDelegate(ui.MenuDelegate):
    """Render a right-aligned indicator backed by Python server status."""

    def __init__(self) -> None:
        super().__init__()
        self._button = None
        self._background = None
        self._state_dot = None
        self._label = None
        self._popup = None
        self._status_label = None
        self._endpoint_label = None
        self._connections_label = None
        self._security_label = None
        self._error_label = None
        self._toggle_button = None
        self._apply_button = None
        self._close_button = None
        status = get_server_status()
        if status is None:
            raise RuntimeError("Python server is unavailable")
        self._host_model = ui.SimpleStringModel(status.configured_host)
        self._port_model = ui.SimpleIntModel(status.configured_port)
        self._status = status
        self._port_shared = False
        # ensure_future (not create_task): the extension can start from a UI callback with no running loop.
        self._port_monitor_task = asyncio.ensure_future(self._monitor_port()) if psutil.LINUX else None
        self._task: asyncio.Task[object] | None = None
        self._confirm_apply = False
        self._destroyed = False
        self._host_subscription = self._host_model.add_value_changed_fn(self._on_draft_changed)
        self._port_subscription = self._port_model.add_value_changed_fn(self._on_draft_changed)
        self._unsubscribe = subscribe_server_status(self._on_status)

    def destroy(self) -> None:
        """Destroy the popup and release every callback."""
        self._destroyed = True
        if self._port_monitor_task is not None:
            self._port_monitor_task.cancel()
            self._port_monitor_task = None
        if self._unsubscribe is not None:
            self._unsubscribe()
            self._unsubscribe = None
        if self._host_subscription is not None:
            self._host_model.remove_value_changed_fn(self._host_subscription)
            self._host_subscription = None
        if self._port_subscription is not None:
            self._port_model.remove_value_changed_fn(self._port_subscription)
            self._port_subscription = None
        if self._button is not None:
            self._button.set_clicked_fn(None)
        if self._toggle_button is not None:
            self._toggle_button.set_clicked_fn(None)
        if self._apply_button is not None:
            self._apply_button.set_clicked_fn(None)
        if self._close_button is not None:
            self._close_button.set_clicked_fn(None)
        if self._popup is not None:
            self._popup.visible = False
            self._popup.destroy()
            self._popup = None
        self._button = None
        self._background = None
        self._label = None

    def build_item(self, item: ui.MenuHelper) -> None:
        """Build the compact top-bar widget."""
        with ui.HStack(width=0, style={"margin": 0}):
            with ui.HStack(content_clipping=1, width=0):
                with ui.VStack(width=0):
                    ui.Spacer(height=2)
                    with ui.ZStack(width=94, height=20):
                        self._background = ui.Rectangle(
                            name="python_server_indicator_background",
                            style={"background_color": cl("#333333"), "border_radius": 3},
                        )
                        with ui.HStack(spacing=4):
                            ui.Spacer(width=5)
                            with ui.VStack(width=8):
                                ui.Spacer()
                                self._state_dot = ui.Rectangle(
                                    width=8,
                                    height=8,
                                    style={"background_color": cl("#42A5F5"), "border_radius": 4},
                                )
                                ui.Spacer()
                            self._label = ui.Label("PY Server", width=64, alignment=ui.Alignment.CENTER)
                        self._button = ui.InvisibleButton(
                            width=94,
                            height=20,
                            identifier="python_server_indicator",
                            clicked_fn=self._show_popup,
                        )
                    ui.Spacer(height=2)
            ui.Spacer(width=8)
        self._render(self._status)

    def get_menu_alignment(self) -> MenuAlignment:
        """Place the delegate on the right side of the application menu."""
        return MenuAlignment.RIGHT

    def update_menu_item(self, menu_item: ui.Menu | ui.MenuItem, menu_refresh: bool) -> None:
        """Hide the placeholder menu item used to host this custom delegate."""
        if isinstance(menu_item, ui.MenuItem):
            menu_item.visible = False

    def _show_popup(self) -> None:
        self._schedule(self._show_popup_deferred)

    async def _show_popup_deferred(self) -> None:
        """Open the modal after the menu-bar click finishes processing.

        Opening a window during the menu click itself allows the menu system's
        cleanup to consume the new window's first input event.
        """
        await omni.kit.app.get_app().next_update_async()
        if self._destroyed or self._button is None:
            return
        if self._popup is None:
            self._build_popup()
        self._reset_fields()
        self._render(self._status)
        if self._popup is not None:
            self._popup.visible = True
            self._popup.focus()

    def _build_popup(self) -> None:
        flags = (
            ui.WINDOW_FLAGS_NO_RESIZE
            | ui.WINDOW_FLAGS_NO_SCROLLBAR
            | ui.WINDOW_FLAGS_NO_SAVED_SETTINGS
            | ui.WINDOW_FLAGS_MODAL
        )
        self._popup = ui.Window(
            "Python Server",
            width=520,
            height=260,
            flags=flags,
            visible=False,
        )
        with self._popup.frame:
            with ui.HStack(spacing=2, style={"margin": 6}):
                with ui.VStack(width=235, height=0, spacing=0):
                    with ui.HStack(height=16):
                        ui.Label("Runtime", style={"font_size": 14})
                        ui.Spacer()
                        self._status_label = ui.Label("", width=95, alignment=ui.Alignment.RIGHT_CENTER)
                    self._error_label = ui.Label(
                        "",
                        identifier="python_server_error",
                        word_wrap=True,
                        height=32,
                        visible=False,
                        style={"color": cl("#EF5350")},
                    )
                    self._endpoint_label = self._status_row("Listening at", "python_server_endpoint")
                    self._connections_label = self._status_row("Connections", "python_server_connections")
                    self._security_label = self._status_row("Security", "python_server_security")
                    self._toggle_button = ui.Button(
                        "Stop for this session",
                        height=20,
                        identifier="python_server_toggle",
                        clicked_fn=self._on_toggle,
                    )
                ui.Separator(width=1)
                with ui.VStack(width=235, height=0, spacing=0):
                    ui.Label("Listener", height=16, style={"font_size": 14})
                    with ui.HStack(height=18):
                        ui.Label("Host", width=50)
                        ui.StringField(
                            model=self._host_model,
                            identifier="python_server_host",
                            height=16,
                        )
                    with ui.HStack(height=18):
                        ui.Label("Port", width=50)
                        ui.IntField(
                            model=self._port_model,
                            identifier="python_server_port",
                            height=16,
                        )
                    with ui.HStack(height=20, spacing=3):
                        self._apply_button = ui.Button(
                            "Apply & Restart",
                            identifier="python_server_apply",
                            clicked_fn=self._on_apply,
                        )
                        ui.Button("Reset", identifier="python_server_reset", clicked_fn=self._reset_fields)
                    ui.Spacer(height=4)
                    with ui.HStack(height=20, spacing=3):
                        self._close_button = ui.Button(
                            "Close",
                            identifier="python_server_close",
                            clicked_fn=self._close_popup,
                        )
        self._render(self._status)

    @staticmethod
    def _status_row(name: str, identifier: str) -> ui.Label:
        with ui.HStack(height=14):
            ui.Label(name, width=85, style={"color": cl("#AAAAAA")})
            label = ui.Label("", identifier=identifier, alignment=ui.Alignment.RIGHT_CENTER)
        return label

    def _on_status(self, status: ServerStatus) -> None:
        if self._destroyed:
            return
        self._status = status
        self._render(status)

    def _render(self, status: ServerStatus) -> None:
        if self._destroyed:
            return
        short_label, color, state_text = _STATE_PRESENTATION[status.state]
        endpoint = self._display_endpoint(status)
        tooltip = self._tooltip(status, state_text, endpoint)
        warning = f"Port {status.configured_port} is shared with another process." if self._port_shared else ""
        if warning and status.state == ServerState.RUNNING:
            color = cl("#FFB74D")
            tooltip = f"{warning}\n{tooltip}"
        else:
            warning = ""
        if self._label is not None:
            self._label.text = short_label
        if self._state_dot is not None:
            self._state_dot.style = {"background_color": color, "border_radius": 4}
        if self._background is not None:
            self._background.set_tooltip(tooltip)
        if self._button is not None:
            self._button.set_tooltip(tooltip)
        if self._status_label is not None:
            self._status_label.text = state_text
            self._status_label.style = {"color": color}
        if self._endpoint_label is not None:
            self._endpoint_label.text = endpoint
        if self._connections_label is not None:
            self._connections_label.text = str(status.active_connections)
        if self._security_label is not None:
            local_only = self._is_loopback(status.configured_host)
            if local_only:
                auth = "Auth required" if status.authentication_required else "Auth off"
                self._security_label.text = f"Local only - {auth}"
                self._security_label.style = {"color": cl("#AAAAAA")}
            elif status.authentication_required:
                self._security_label.text = "Network - Auth required"
                self._security_label.style = {"color": cl("#FFB74D")}
            else:
                self._security_label.text = "UNAUTHENTICATED NETWORK"
                self._security_label.style = {"color": cl("#EF5350")}
        if self._toggle_button is not None:
            self._toggle_button.text = (
                "Start for this session"
                if status.state in {ServerState.STOPPED, ServerState.ERROR}
                else "Stop for this session"
            )
            self._toggle_button.enabled = status.state not in _TRANSITION_STATES and self._task is None
        if self._error_label is not None and (status.last_error or warning):
            self._error_label.text = status.last_error or warning
            self._error_label.visible = True
            self._error_label.style = {"color": cl("#EF5350") if status.state == ServerState.ERROR else cl("#FFB74D")}
        elif self._error_label is not None:
            self._error_label.text = ""
            self._error_label.visible = False
        self._update_apply_state()

    @staticmethod
    def _is_port_shared(port: int) -> bool:
        try:
            return any(
                connection.status == psutil.CONN_LISTEN
                and connection.laddr
                and connection.laddr.port == port
                and connection.pid != os.getpid()
                for connection in psutil.net_connections(kind="tcp4")
            )
        except psutil.Error:
            return False

    async def _monitor_port(self) -> None:
        while True:
            status = self._status
            shared = status.state == ServerState.RUNNING and await asyncio.to_thread(
                self._is_port_shared, status.configured_port
            )
            if status == self._status and shared != self._port_shared:
                self._port_shared = shared
                self._render(status)
            await asyncio.sleep(1)

    @staticmethod
    def _display_endpoint(status: ServerStatus) -> str:
        if not status.bound_endpoints:
            return "Not listening"
        return ", ".join(f"{endpoint.host}:{endpoint.port}" for endpoint in status.bound_endpoints)

    @staticmethod
    def _tooltip(status: ServerStatus, state_text: str, endpoint: str) -> str:
        exposure = "Local only" if PythonServerDelegate._is_loopback(status.configured_host) else "Network exposed"
        auth = "required" if status.authentication_required else "off"
        tooltip = (
            f"Python Server - {state_text}\n"
            f"{endpoint} - {exposure}\n"
            f"{status.active_connections} active connections - Authentication {auth}\n"
        )
        if status.last_error:
            tooltip += f"{status.last_error}\n"
        return f"{tooltip}Click to manage"

    @staticmethod
    def _is_loopback(host: str) -> bool:
        if host.lower() == "localhost":
            return True
        try:
            return ipaddress.ip_address(host).is_loopback
        except ValueError:
            return False

    def _on_draft_changed(self, _model: object) -> None:
        if self._destroyed:
            return
        self._confirm_apply = False
        if self._apply_button is not None:
            self._apply_button.text = "Apply & Restart"
        if self._error_label is not None:
            self._error_label.text = ""
            self._error_label.visible = False
        self._update_apply_state()

    def _update_apply_state(self) -> None:
        if self._destroyed or self._apply_button is None:
            return
        dirty = (
            self._host_model.get_value_as_string().strip() != self._status.configured_host
            or self._port_model.get_value_as_int() != self._status.configured_port
        )
        self._apply_button.enabled = dirty and self._status.state not in _TRANSITION_STATES and self._task is None

    def _reset_fields(self) -> None:
        if self._destroyed:
            return
        self._confirm_apply = False
        self._host_model.set_value(self._status.configured_host)
        self._port_model.set_value(self._status.configured_port)
        if self._apply_button is not None:
            self._apply_button.text = "Apply & Restart"
        if self._error_label is not None:
            self._error_label.text = ""
            self._error_label.visible = False
        self._update_apply_state()

    def _on_toggle(self) -> None:
        if self._status.state in {ServerState.STOPPED, ServerState.ERROR}:
            self._schedule(start_server)
        else:
            self._schedule(stop_server)

    def _on_apply(self) -> None:
        host = self._host_model.get_value_as_string().strip()
        if host == "localhost":
            host = "127.0.0.1"
        try:
            address = ipaddress.ip_address(host)
        except ValueError:
            self._show_error("Host must be an IPv4 address or localhost.")
            return
        if address.version != 4:
            self._show_error("IPv6 is not supported by the Python server yet.")
            return
        port = self._port_model.get_value_as_int()
        if port < 1 or port > 65535:
            self._show_error("Port must be between 1 and 65535.")
            return
        confirmation_reasons = []
        if not address.is_loopback:
            confirmation_reasons.append("This exposes remote code execution to the network.")
        if self._status.active_connections:
            confirmation_reasons.append(f"{self._status.active_connections} active connection(s) may be interrupted.")
        if confirmation_reasons and not self._confirm_apply:
            self._confirm_apply = True
            self._show_error(f"{' '.join(confirmation_reasons)} Click Confirm Apply to continue.")
            if self._apply_button is not None:
                self._apply_button.text = "Confirm Apply"
            return
        self._schedule(lambda: self._apply(host, port))

    async def _apply(self, host: str, port: int) -> None:
        success = await restart_server(host, port)
        if self._destroyed:
            return
        if success:
            self._reset_fields()
        elif self._status.last_error:
            self._show_error(self._status.last_error)

    def _close_popup(self) -> None:
        if self._popup is not None:
            self._popup.visible = False

    def _show_error(self, message: str) -> None:
        if not self._destroyed and self._error_label is not None:
            self._error_label.text = message
            self._error_label.visible = True
            self._error_label.style = {"color": cl("#EF5350")}

    def _schedule(self, factory: Callable[[], Coroutine[Any, Any, object]]) -> None:
        if self._destroyed or self._task is not None:
            return
        task = asyncio.ensure_future(factory())
        self._task = task
        self._render(self._status)
        task.add_done_callback(self._on_task_done)

    def _on_task_done(self, task: asyncio.Task[object]) -> None:
        if self._task is task:
            self._task = None
        if task.cancelled():
            self._render(self._status)
            return
        try:
            task.result()
        except Exception as exc:  # noqa: BLE001 - surface lifecycle failures in the popup
            carb.log_error(f"Python server UI action failed: {exc}")
            if not self._destroyed:
                self._render(self._status)
                self._show_error(str(exc))
        else:
            self._render(self._status)


class PythonServerUi:
    """Register and own the top-right menu delegate."""

    def __init__(self) -> None:
        self._menu_items = [MenuItemDescription(name="placeholder", show_fn=lambda: False)]
        self._delegate = PythonServerDelegate()
        add_menu_items(self._menu_items, name=_MENU_NAME, delegate=self._delegate)

    def destroy(self) -> None:
        """Unregister the indicator and destroy its popup."""
        if self._menu_items is not None:
            remove_menu_items(self._menu_items, _MENU_NAME)
            self._menu_items = None
        if self._delegate is not None:
            self._delegate.destroy()
            self._delegate = None
