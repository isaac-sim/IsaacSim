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

"""About dialog extension and helper accessors."""

from collections.abc import Callable
from typing import Any

import carb
import carb.settings
import carb.tokens
import omni.client
import omni.ext
import omni.kit.actions.core
import omni.kit.app
import omni.kit.ui
from isaacsim.core.version import get_version
from omni import ui
from omni.kit.menu.utils import MenuItemDescription, add_menu_items, build_submenu_dict

WINDOW_NAME = "About"
DISCONNECTED = "** disconnected **"
QUERYING = "** querying **"

_extension_instance = None

# Plugin-row tooltip palette: dark text on pale cream so the tooltip is legible against
# the dark dialog body. ``0xFFD1F7FF`` is RGB(255, 247, 209) under omni.ui's AABBGGRR
# packing — the pale cream used by ``isaacsim.gui.components`` ``TOOLTIP_STYLE``.
#
# We render the tooltip via ``tooltip_fn`` (a callback that builds the widget tree)
# rather than the plain ``tooltip=<str>`` form. The string form goes through omni.ui's
# default Tooltip widget, whose background renders translucent under this app's
# theme — overriding the ``Tooltip`` selector (either per-Label or via an ancestor
# frame) only partially takes effect. Building the tooltip body ourselves inside a
# ``ui.Frame`` with an explicit ``background_color`` guarantees a solid backdrop.
_TOOLTIP_TEXT_COLOR = 0xFF333333
_TOOLTIP_BACKGROUND_COLOR = 0xFFD1F7FF
# Style for the parent VStack of the plugin rows. The default omni.ui Tooltip
# container renders a dark border and inset padding around our ``tooltip_fn``
# content; zero out border, padding, and margins so our pale-cream Rectangle
# reaches the popup's edge. Also paint the Tooltip's own ``background_color``
# cream so any chrome the cascade can't reach blends with our Rectangle instead
# of showing as a dark sliver.
_PLUGIN_LIST_STYLE = {
    "Tooltip": {
        "background_color": _TOOLTIP_BACKGROUND_COLOR,
        "border_width": 0,
        "border_color": 0x0,
        "border_radius": 0,
        "padding": 0,
        "margin_width": 0,
        "margin_height": 0,
    }
}


def _format_plugin_tooltip(plugin: Any) -> str:
    """Build a multi-line tooltip describing a carb plugin.

    Args:
        plugin: A ``carb._carb.PluginDesc`` (or a test fake matching its shape).

    Returns:
        Tooltip text containing the plugin description, interfaces, and library path.
    """
    description = (plugin.impl.description or "").strip()
    # ``"(none)"`` rather than an empty string so an interface-less plugin still shows
    # the labelled row and the tooltip layout doesn't collapse.
    interface_names = ", ".join(iface.name for iface in plugin.interfaces) or "(none)"

    lines = []
    if description:
        lines.append(f"Description: {description}")
    lines.append(f"Implements: {interface_names}")
    lines.append(f"Library: {plugin.libPath}")
    return "\n".join(lines)


def _make_plugin_tooltip_fn(plugin: Any) -> Callable[[], None]:
    """Return a ``tooltip_fn`` callback that renders the plugin tooltip body.

    Args:
        plugin: Plugin descriptor used to format the tooltip body.

    Returns:
        Callback that builds the plugin tooltip UI.
    """
    text = _format_plugin_tooltip(plugin)

    def _build() -> None:
        with ui.ZStack():
            ui.Rectangle(style={"background_color": _TOOLTIP_BACKGROUND_COLOR})
            with ui.VStack(style={"margin": 6}):
                ui.Label(text, style={"color": _TOOLTIP_TEXT_COLOR})

    return _build


class AboutExtension(omni.ext.IExt):
    """Extension that provides the About dialog UI."""

    def on_startup(self, ext_id: str) -> None:
        """Initialize the extension when it is loaded.

        Args:
            ext_id: Extension identifier provided by the extension manager.
        """
        self._ext_id = ext_id
        self._ext_name = omni.ext.get_extension_name(ext_id)
        self._about_window: ui.Window | None = None
        # Resolve the bold font path once at startup. ``carb.tokens.resolve`` returns the
        # input unchanged if the ``${fonts}`` token cannot be resolved, in which case
        # omni.ui silently falls back to the default font — graceful degrade.
        self._bold_font = carb.tokens.get_tokens_interface().resolve("${fonts}/NVIDIASans_Bd.ttf")

        # Register the action
        action_registry = omni.kit.actions.core.get_action_registry()
        action_registry.register_action(
            self._ext_name,
            "show_about",
            self._on_menu_show_about,
            display_name="Show About Dialog",
            description="Show the About dialog",
        )

        menu_dict = build_submenu_dict(
            [
                MenuItemDescription(name="Help/About", onclick_action=(self._ext_name, "show_about")),
            ],
        )
        for group in menu_dict:
            add_menu_items(menu_dict[group], group)

        self.get_values()

        global _extension_instance
        _extension_instance = self

    def on_shutdown(self) -> None:
        """Clean up resources when the extension is unloaded."""
        global _extension_instance
        _extension_instance = None

        # Deregister the action
        action_registry = omni.kit.actions.core.get_action_registry()
        action_registry.deregister_action(self._ext_name, "show_about")

        self._about_window = None

    def get_values(self) -> None:
        """Load application and version values for the About dialog.

        This populates cached values used by the About window, including
        kit and client library versions, plus the app name and version.

        Example:
            .. code-block:: python

                extension = get_instance()
                if extension:
                    extension.get_values()
        """
        settings = carb.settings.get_settings()
        self.kit_version = omni.kit.app.get_app().get_build_version()
        # Minimize Kit SDK version for release
        if self.kit_version:
            kit_version, _ = self.kit_version.split("+")
            self.kit_version = kit_version
        self.client_library_version = omni.client.get_version()
        # Minimize Client Library version for release
        if self.client_library_version:
            client_lib_version, _ = self.client_library_version.split("+")
            client_lib_version, _ = client_lib_version.split("-")
            self.client_library_version = client_lib_version
        # Get App Name and Version
        self.app_name = settings.get("/app/window/title")
        self.app_version_core, self.app_version_prerel, _, _, _, _, _, _ = get_version()
        self.app_version = f"{self.app_version_core}-{self.app_version_prerel}"

    def _on_menu_show_about(self) -> None:
        """Handle the menu action to show the About dialog."""
        plugins = carb.get_framework().get_plugins()
        plugins = sorted(plugins, key=lambda x: x.impl.name)
        # Keep a strong reference on the extension instance — the menu callback discards
        # the return value, and without this ref the ui.Window would be garbage-collected
        # before it ever renders (the previous version of this extension relied on lambda
        # captures from the OK button + width/height-changed callbacks to keep it alive).
        self._about_window = self.menu_show_about(plugins)

    def menu_show_about(self, plugins: list[Any]) -> ui.Window:
        """Create and show the About dialog window.

        Args:
            plugins: Plugins to list in the dialog.

        Returns:
            The created About window.

        Example:
            .. code-block:: python

                extension = get_instance()
                if extension:
                    plugins = carb.get_framework().get_plugins()
                    window = extension.menu_show_about(plugins)
        """
        rows = (
            ("App Name", self.app_name),
            ("App Version", self.app_version),
            ("Kit SDK Version", self.kit_version),
            ("Client Library Version", self.client_library_version),
        )
        info = "\n".join(f"{key}: {value}" for key, value in rows)

        # ``margin_*`` / ``padding`` are zeroed because omni.ui Labels otherwise add a
        # few pixels of vertical padding that, combined across four rows, inflate the
        # header into a tall block.
        key_style = {
            "font": self._bold_font,
            "font_size": 16,
            "margin_height": 0,
            "margin_width": 0,
            "padding": 0,
        }
        value_style = {"font_size": 16, "margin_height": 0, "margin_width": 0, "padding": 0}
        section_header_style = {"font": self._bold_font, "font_size": 16}
        # Slightly darker than the dialog's default surface so the plugin list reads as
        # an inset region rather than continuous body text.
        plugin_list_style = {
            "background_color": 0xFF1E1E1E,
            "margin_width": 2,
            "margin_height": 3,
        }

        def copy_to_clipboard(x: float, y: float, button: int, modifier: int) -> None:
            if button != 1:
                return

            try:
                import pyperclip
            except ImportError:
                carb.log_warn("Could not import pyperclip.")
                return
            try:
                pyperclip.copy(info)
            except pyperclip.PyperclipException:
                carb.log_warn(pyperclip.EXCEPT_MSG)
                return

        window = ui.Window(
            "About", width=800, height=510, flags=ui.WINDOW_FLAGS_NO_SCROLLBAR | ui.WINDOW_FLAGS_NO_DOCKING
        )

        # Dismissal is via the window's titlebar close X — the only flags set are
        # NO_SCROLLBAR | NO_DOCKING, so the close affordance stays.
        with window.frame:
            with ui.ZStack():
                # Outer dialog padding is provided by explicit Spacers rather than a
                # ``style={"margin": 8}`` on the inner VStack — omni.ui's stack margin
                # inflates each child's natural height, which spreads the header rows
                # apart.
                with ui.HStack():
                    ui.Spacer(width=ui.Pixel(8))
                    with ui.VStack(spacing=0):
                        ui.Spacer(height=8)
                        for key, value in rows:
                            with ui.HStack(height=ui.Pixel(18)):
                                ui.Label(key, width=ui.Pixel(220), height=ui.Pixel(18), style=key_style)
                                ui.Label(str(value), height=ui.Pixel(18), style=value_style)
                        ui.Spacer(height=12)
                        ui.Label("Loaded plugins", height=0, style=section_header_style)
                        ui.Spacer(height=2)
                        with ui.ScrollingFrame(
                            height=ui.Fraction(1),
                            horizontal_scrollbar_policy=ui.ScrollBarPolicy.SCROLLBAR_ALWAYS_OFF,
                            vertical_scrollbar_policy=ui.ScrollBarPolicy.SCROLLBAR_ALWAYS_ON,
                            style=plugin_list_style,
                        ):
                            with ui.VStack(height=0, style=_PLUGIN_LIST_STYLE):
                                for p in plugins:
                                    ui.Label(
                                        p.impl.name,
                                        height=0,
                                        tooltip_fn=_make_plugin_tooltip_fn(p),
                                        style=_PLUGIN_LIST_STYLE,
                                    )
                        ui.Spacer(height=8)
                    ui.Spacer(width=ui.Pixel(8))
                ui.Button(" ", height=128, style={"background_color": 0x00000000}, mouse_pressed_fn=copy_to_clipboard)

        return window


def get_instance() -> AboutExtension | None:
    """Get the current About extension instance.

    Returns:
        The extension instance if available, otherwise None.

    Example:
        .. code-block:: python

            extension = get_instance()
            if extension:
                extension.menu_show_about(carb.get_framework().get_plugins())
    """
    return _extension_instance
