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

"""Utilities and base class for testing menu UI components in Isaac Sim."""

from __future__ import annotations

import asyncio
from typing import Any

import carb
import omni.kit.app
import omni.kit.ui_test as ui_test
import omni.timeline
import omni.usd
from isaacsim.core.experimental.utils import stage as stage_utils
from omni.ui.tests.test_base import OmniUiTest

from .menu_utils import (
    _DEFAULT_MAX_WAIT_FRAMES,
)
from .menu_utils import find_enabled_widget_with_retry as _find_enabled_widget_with_retry
from .menu_utils import find_widget_with_retry as _find_widget_with_retry
from .menu_utils import menu_click_with_retry as _menu_click_with_retry
from .menu_utils import scroll_to_widget as _scroll_to_widget
from .menu_utils import wait_for_widget_enabled as _wait_for_widget_enabled


class MenuUITestCase(OmniUiTest):
    """Base test class for Isaac Sim menu UI tests.

    This class provides common setup, teardown, and utility methods for testing
    menu-related UI components including menus and context menus.

    Example:

    .. code-block:: python

        >>> from isaacsim.test.utils import MenuUITestCase
        >>>
        >>> class TestMyMenu(MenuUITestCase):
        ...     async def test_menu(self):
        ...         menu = await self.get_viewport_context_menu()
        ...         self.assertIn("Create", menu)
    """

    async def setUp(self) -> None:
        """Set up test environment before each test method.

        Creates a new stage, initializes the timeline interface, and waits for
        stage loading to complete.
        """
        await super().setUp()
        self._timeline = omni.timeline.get_timeline_interface()
        self._stage = await stage_utils.create_new_stage_async()
        await omni.kit.app.get_app().next_update_async()
        await self.wait_for_stage_loading()

    async def tearDown(self) -> None:
        """Clean up test environment after each test method.

        Waits for any pending stage loading operations to complete before
        calling the parent tearDown method.
        """
        await omni.kit.app.get_app().next_update_async()
        await self.wait_for_stage_loading()
        await super().tearDown()

    async def wait_for_stage_loading(self) -> None:
        """Wait until the stage has finished loading.

        Polls the stage loading status and waits until all files have been loaded.
        """
        while stage_utils.is_stage_loading():
            await omni.kit.app.get_app().next_update_async()

        await omni.kit.material.library.get_mdl_list_async()
        await ui_test.human_delay()
        omni.kit.menu.utils.rebuild_menus()
        await omni.kit.app.get_app().next_update_async()

    async def new_stage(self) -> None:
        """Create a new stage and wait for it to load.

        Useful for resetting the stage between test iterations in a loop.
        """
        self._stage = await stage_utils.create_new_stage_async()
        await omni.kit.app.get_app().next_update_async()
        await self.wait_for_stage_loading()

    async def wait_n_frames(self, n: int = 10) -> None:
        """Wait for N app update frames.

        Args:
            n: Number of frames to wait.
        """
        for _ in range(n):
            await omni.kit.app.get_app().next_update_async()

    async def get_viewport_context_menu(self, max_attempts: int = 5) -> dict[str, Any]:
        """Get the viewport context menu with retry and exponential backoff.

        Args:
            max_attempts: Maximum number of retry attempts.

        Returns:
            The context menu dictionary structure.

        Raises:
            Exception: If unable to get context menu after all attempts.
        """
        retry_delay = 0.5

        for attempt in range(max_attempts):
            try:
                viewport_window = ui_test.find("Viewport")
                await viewport_window.right_click()
                return await ui_test.get_context_menu()
            except Exception as e:
                if attempt < max_attempts - 1:
                    carb.log_warn(f"Attempt {attempt + 1} failed to get context menu: {e}. Retrying...")
                    await self.wait_n_frames(1)
                    await asyncio.sleep(retry_delay)
                    retry_delay *= 1.5
                else:
                    carb.log_error(f"Failed to get context menu after {max_attempts} attempts: {e}")
                    raise

    async def menu_click_with_retry(
        self, menu_path: str, delays: list[int] = None, window_name: str = None, wait_n_frames: int = 10
    ) -> Any:
        """Click a menu item with retry at different delay speeds.

        Some menu items require different timing to be clicked successfully.
        This method tries multiple delay values before giving up.

        Args:
            menu_path: The menu path to click (e.g., "Create/Sensors/Contact Sensor").
            delays: List of delay values to try in milliseconds.
            window_name: Optional window name to check for after clicking.
                If provided, the method returns early when the window is found
                and returns the window widget.
            wait_n_frames: Number of frames to wait.

        Returns:
            The found window widget if window_name is provided and found, else None.
        """
        return await _menu_click_with_retry(
            menu_path, delays=delays, window_name=window_name, wait_n_frames=wait_n_frames
        )

    async def find_widget_with_retry(
        self, query: str, max_frames: int = _DEFAULT_MAX_WAIT_FRAMES, parent: Any = None
    ) -> Any:
        """Poll ``ui_test.find`` until the widget is found or *max_frames* is exceeded.

        Convenience wrapper around :func:`~isaacsim.test.utils.menu_utils.find_widget_with_retry`.

        Args:
            query: The widget query string.
            max_frames: Maximum frames to poll before raising.
            parent: Optional parent widget to search within.

        Returns:
            The found widget reference.

        Raises:
            TimeoutError: If the widget is not found within *max_frames*.
        """
        return await _find_widget_with_retry(query, max_frames=max_frames, parent=parent)

    async def find_enabled_widget_with_retry(
        self, query: str, max_frames: int = _DEFAULT_MAX_WAIT_FRAMES, parent: Any = None
    ) -> Any:
        """Poll ``ui_test.find`` until the widget is found **and** enabled.

        Convenience wrapper around :func:`~isaacsim.test.utils.menu_utils.find_enabled_widget_with_retry`.

        Args:
            query: The widget query string.
            max_frames: Maximum frames to poll before raising.
            parent: Optional parent widget to search within.

        Returns:
            The found and enabled widget reference.

        Raises:
            TimeoutError: If the widget is not found and enabled within *max_frames*.
        """
        return await _find_enabled_widget_with_retry(query, max_frames=max_frames, parent=parent)

    async def scroll_to_widget(self, widget_ref: Any, settle_frames: int = 3) -> bool:
        """Scroll the enclosing ``ScrollingFrame`` so ``widget_ref`` is visible.

        Convenience wrapper around :func:`~isaacsim.test.utils.menu_utils.scroll_to_widget`.

        Args:
            widget_ref: A ``WidgetRef`` returned by ``ui_test.find`` /
                :meth:`find_widget_with_retry`.
            settle_frames: Number of frames to wait after scrolling.

        Returns:
            True if the widget was scrolled into view, False if no enclosing
            ``ScrollingFrame`` was found.
        """
        return await _scroll_to_widget(widget_ref, settle_frames=settle_frames)

    async def wait_for_widget_enabled(self, widget: Any, max_frames: int = _DEFAULT_MAX_WAIT_FRAMES) -> bool:
        """Poll until ``widget.widget.enabled`` becomes True.

        Convenience wrapper around :func:`~isaacsim.test.utils.menu_utils.wait_for_widget_enabled`.

        Args:
            widget: A ``WidgetRef`` returned by ``ui_test.find``.
            max_frames: Maximum number of app-update frames to wait.

        Returns:
            True if the widget became enabled within *max_frames*, False otherwise.
        """
        return await _wait_for_widget_enabled(widget, max_frames=max_frames)

    def count_prims_by_type(self, prim_type: str) -> int:
        """Count the number of prims of a given type on the stage.

        Args:
            prim_type: The USD prim type name to count (e.g., "IsaacContactSensor").

        Returns:
            The number of prims matching the given type.
        """
        count = 0
        for prim in self._stage.Traverse():
            if prim.GetTypeName() == prim_type:
                count += 1
        return count

    async def run_timeline_frames(self, n: int = 50) -> None:
        """Play the timeline for N frames then stop.

        Useful for tests that need physics or sensor processing to occur.

        Args:
            n: Number of frames to run the timeline.
        """
        self._timeline.play()
        await self.wait_n_frames(n)
        self._timeline.stop()

    def select_prim(self, prim_path: str) -> None:
        """Select a prim by its path.

        Args:
            prim_path: The USD path of the prim to select.
        """
        omni.usd.get_context().get_selection().set_selected_prim_paths([prim_path], False)
