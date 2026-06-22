# SPDX-FileCopyrightText: Copyright (c) 2021-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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

"""UI tests for the About dialog window."""

from pathlib import Path

import carb
import isaacsim.app.about
import omni.kit.app
import omni.kit.test
import omni.kit.ui_test as ui_test
import omni.ui as ui
from isaacsim.app.about.extension import _format_plugin_tooltip
from omni.ui.tests.test_base import OmniUiTest


class _FakeInterfaceDesc:
    """Fake ``carb._carb.InterfaceDesc`` shape for UI tests.

    Args:
        name: Interface name (e.g., ``"carb::tokens::ITokens"``).
    """

    def __init__(self, name: str) -> None:
        self.name = name


class _FakePluginImpl:
    """Fake ``carb._carb.PluginImplDesc`` shape for UI tests.

    Args:
        name: Implementation name.
        description: Human-readable description (used in the tooltip).
    """

    def __init__(self, name: str, description: str = "") -> None:
        self.name = name
        self.description = description


class _FakePlugin:
    """Fake ``carb._carb.PluginDesc`` shape for UI tests.

    Args:
        name: Name suffix used to populate fake descriptor fields.
    """

    def __init__(self, name: str) -> None:
        self.libPath = "Lib Path " + name
        self.impl = _FakePluginImpl("Impl " + name, description=f"Description for {name}")
        self.interfaces = [_FakeInterfaceDesc("Interface " + name)]


class TestFormatPluginTooltip(omni.kit.test.AsyncTestCase):
    """Unit tests for ``_format_plugin_tooltip`` branching behavior.

    The helper is exercised end-to-end by ``test_about_ui`` via a populated fake plugin,
    but its three branches (description present/absent and interfaces empty/non-empty)
    are not covered there. These tests pin the formatting contract.
    """

    async def test_full_plugin_renders_all_three_lines(self) -> None:
        """Verify a complete plugin renders description, interfaces, and library."""
        plugin = _FakePlugin("Full")
        text = _format_plugin_tooltip(plugin)
        self.assertEqual(
            text,
            "Description: Description for Full\nImplements: Interface Full\nLibrary: Lib Path Full",
        )

    async def test_empty_description_omits_description_line(self) -> None:
        """Verify empty plugin descriptions are omitted."""
        plugin = _FakePlugin("Bare")
        plugin.impl.description = ""
        text = _format_plugin_tooltip(plugin)
        self.assertNotIn("Description:", text)
        self.assertTrue(text.startswith("Implements:"))
        self.assertIn("Library: Lib Path Bare", text)

    async def test_none_description_omits_description_line(self) -> None:
        """Verify ``None`` plugin descriptions are omitted."""
        plugin = _FakePlugin("NoneDesc")
        plugin.impl.description = None
        text = _format_plugin_tooltip(plugin)
        self.assertNotIn("Description:", text)
        self.assertTrue(text.startswith("Implements:"))

    async def test_empty_interfaces_falls_back_to_none_token(self) -> None:
        """Verify empty interface lists render the explicit none token."""
        plugin = _FakePlugin("NoIface")
        plugin.interfaces = []
        text = _format_plugin_tooltip(plugin)
        # The "(none)" sentinel keeps the labelled row present so the tooltip layout
        # doesn't collapse for interface-less plugins.
        self.assertIn("Implements: (none)", text)

    async def test_multiple_interfaces_are_joined_with_comma(self) -> None:
        """Verify multiple interfaces are joined in one tooltip row."""
        plugin = _FakePlugin("Multi")
        plugin.interfaces = [_FakeInterfaceDesc("IFoo"), _FakeInterfaceDesc("IBar")]
        text = _format_plugin_tooltip(plugin)
        self.assertIn("Implements: IFoo, IBar", text)


class TestAboutWindow(OmniUiTest):
    """Tests that validate the About window rendering."""

    # Before running each test
    async def setUp(self) -> None:
        """Set up the test environment."""
        await super().setUp()

        EXTENSION_FOLDER_PATH = Path(
            omni.kit.app.get_app().get_extension_manager().get_extension_path_by_module(__name__)
        )
        TEST_DATA_PATH = EXTENSION_FOLDER_PATH.joinpath("data/tests")
        self._golden_img_dir = TEST_DATA_PATH.absolute().joinpath("golden_img").absolute()

    # After running each test
    async def tearDown(self) -> None:
        """Clean up the test environment."""
        await super().tearDown()

    async def test_about_ui(self) -> None:
        """Validate the About dialog UI."""
        about = isaacsim.app.about.get_instance()
        about.kit_version = "#Version#"
        about.client_library_version = "#Client Library Version#"
        about.app_name = "#App Name#"
        about.app_version = "#App Version#"
        about_window = about.menu_show_about(
            [_FakePlugin("Test 1"), _FakePlugin("Test 2"), _FakePlugin("Test 3"), _FakePlugin("Test 4")]
        )

        about.get_values()
        try:
            await self.docked_test_window(window=about_window, width=400, height=510)
            await self.finalize_test(golden_img_dir=self._golden_img_dir, golden_img_name="test_about_ui.png")
        except Exception as e:
            carb.log_warn(f"Could not run test because carb::windowing is not available: {e}")

    async def test_menu_callback_keeps_window_alive(self) -> None:
        """Verify the menu callback keeps the created About window alive.

        The menu action discards ``menu_show_about``'s return value, so the extension
        itself must retain a strong reference to the window. Otherwise the ``ui.Window``
        is garbage-collected before it renders and Help -> About appears to do nothing.
        """
        about = isaacsim.app.about.get_instance()
        self.assertIsNotNone(about, "isaacsim.app.about extension is not loaded")

        # Drop any reference left from a previous test so we observe a clean cycle.
        about._about_window = None

        about._on_menu_show_about()
        for _ in range(2):
            await omni.kit.app.get_app().next_update_async()

        self.assertIsNotNone(
            about._about_window,
            "AboutExtension._on_menu_show_about must store the created window on self; "
            "otherwise the only reference is the discarded return value and the window "
            "is GC'd before rendering.",
        )
        self.assertTrue(
            about._about_window.visible,
            "The About window must be visible after the menu action fires.",
        )
        # Belt-and-suspenders: confirm the window the extension is holding is the same
        # one omni.ui has registered under the title "About" — a buggy impl that
        # assigned an unrelated object would pass the assertIsNotNone above.
        self.assertIs(
            ui.Workspace.get_window("About"),
            about._about_window,
            "The window stored on AboutExtension._about_window must be the same instance "
            "omni.ui registered under the 'About' title.",
        )

        # Clean up so the next test starts from a known state.
        about._about_window.visible = False
        about._about_window = None
        for _ in range(2):
            await omni.kit.app.get_app().next_update_async()

    async def test_about_clipboard_format(self) -> None:
        """Verify the clipboard text includes the Kit SDK version row.

        The clipboard string emitted by the invisible right-click overlay must include
        ``"Kit SDK Version: "`` with the colon-space separator.
        """
        about = isaacsim.app.about.get_instance()
        self.assertIsNotNone(about, "isaacsim.app.about extension is not loaded")

        # Use stable, recognizable values so we can assert exact substrings.
        about.kit_version = "110.0.0"
        about.client_library_version = "2.55.0"
        about.app_name = "Isaac Sim"
        about.app_version = "5.1.0-rc.1"

        # The production code does a local `import pyperclip` inside the overlay's
        # `mouse_pressed_fn`. Inject a stub module into `sys.modules` so the import
        # resolves to our capture-only `copy` even when pyperclip is not installed in the
        # test environment. If a real `pyperclip` is already loaded, patch its `copy`.
        import sys
        import types

        captured: list[str] = []

        if "pyperclip" in sys.modules:
            _pyperclip = sys.modules["pyperclip"]
            original_copy = _pyperclip.copy
            stub_installed = False
        else:
            _pyperclip = types.ModuleType("pyperclip")

            class _PyperclipException(Exception):
                pass

            _pyperclip.PyperclipException = _PyperclipException
            _pyperclip.EXCEPT_MSG = ""
            original_copy = None
            sys.modules["pyperclip"] = _pyperclip
            stub_installed = True

        _pyperclip.copy = lambda value: captured.append(value)
        try:
            about_window = about.menu_show_about(
                [_FakePlugin("Test 1"), _FakePlugin("Test 2"), _FakePlugin("Test 3"), _FakePlugin("Test 4")]
            )
            try:
                # Dock the window so the overlay is realized.
                await self.docked_test_window(window=about_window, width=800, height=510)
                for _ in range(2):
                    await omni.kit.app.get_app().next_update_async()

                # The transparent overlay button at the top of the ZStack carries the
                # `mouse_pressed_fn=copy_to_clipboard` callback. Headless test runs (no
                # GLFW window) do not propagate emulated mouse events to the omni.ui
                # backend, so invoke the production callback directly via
                # `Widget.call_mouse_pressed_fn(x, y, button, modifier)`. Button id 1 ==
                # right mouse button (matches the `button != 1` guard in the extension).
                all_buttons = ui_test.find_all("About//Frame/**/Button[*]")
                overlay_btn = next(
                    (btn for btn in all_buttons if getattr(btn.widget, "text", None) == " "),
                    None,
                )
                self.assertIsNotNone(overlay_btn, "Transparent clipboard overlay button was not found")
                overlay_btn.widget.call_mouse_pressed_fn(0.0, 0.0, 1, 0)
                for _ in range(2):
                    await omni.kit.app.get_app().next_update_async()
            finally:
                about_window.visible = False
                for _ in range(2):
                    await omni.kit.app.get_app().next_update_async()
                try:
                    await self.finalize_test_no_image()
                except Exception as cleanup_exc:  # noqa: BLE001
                    carb.log_warn(f"finalize_test_no_image raised during cleanup: {cleanup_exc}")
        finally:
            if stub_installed:
                sys.modules.pop("pyperclip", None)
            else:
                _pyperclip.copy = original_copy

        self.assertEqual(
            len(captured),
            1,
            f"Expected exactly one clipboard write from the overlay right-click, got {len(captured)}: {captured!r}",
        )
        clipboard_text = captured[0]
        carb.log_info(f"About clipboard payload: {clipboard_text!r}")

        # The clipboard payload must contain the labelled, colon-separated form.
        self.assertIn(
            "Kit SDK Version: 110.0.0",
            clipboard_text,
            "Clipboard payload is missing the ': ' separator between 'Kit SDK Version' "
            f"and the version value. Full payload: {clipboard_text!r}",
        )
        self.assertNotIn(
            "Kit SDK Version110.0.0",
            clipboard_text,
            f"Clipboard payload contains the un-separated concatenation. " f"Full payload: {clipboard_text!r}",
        )
