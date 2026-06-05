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

"""Test suite for menu_utils functions in Isaac Sim."""

from __future__ import annotations

import omni.kit.app
import omni.kit.ui_test as ui_test
from isaacsim.core.experimental.utils import stage as stage_utils
from isaacsim.test.utils.menu_utils import (
    count_menu_items,
    find_enabled_widget_with_retry,
    find_widget_with_retry,
    get_all_menu_paths,
    list_menu_paths,
    menu_click_with_retry,
    navigate_menu_visual,
    perform_widget_action,
)
from isaacsim.test.utils.timed_async_test import TimedAsyncTestCase


def _has_menubar() -> bool:
    """Return True if the application has a populated menubar.

    The menubar exists in both windowed and ``--no-window`` headless modes.
    It may be absent only in minimal test harnesses that do not load the
    main window extension.

    Returns:
        True if the menubar exists and has items, False otherwise.
    """
    try:
        menubar = ui_test.get_menubar()
        return menubar is not None and len(menubar.find_all("*")) > 0
    except Exception:
        return False


class TestGetAllMenuPaths(TimedAsyncTestCase):
    """Tests for the get_all_menu_paths helper (pure data, no UI required)."""

    async def test_get_all_menu_paths_simple(self) -> None:
        """Verify extraction of leaf paths from a simple menu dict."""
        menu_dict = {"Sensors": {"_": ["Contact", "IMU"]}, "Robots": {"_": ["Franka"]}}
        paths = get_all_menu_paths(menu_dict)
        self.assertIn("Sensors/Contact", paths)
        self.assertIn("Sensors/IMU", paths)
        self.assertIn("Robots/Franka", paths)
        self.assertEqual(len(paths), 3)

    async def test_get_all_menu_paths_with_root_path(self) -> None:
        """Verify that root_path is prepended to all returned paths."""
        menu_dict = {"Lidar": {"_": ["Sensor A", "Sensor B"]}}
        paths = get_all_menu_paths(menu_dict, root_path="Create/Sensors")
        for path in paths:
            self.assertTrue(path.startswith("Create/Sensors/"), f"Missing root prefix: {path}")

    async def test_get_all_menu_paths_nested(self) -> None:
        """Verify traversal of nested menu dictionaries."""
        menu_dict = {
            "Physics": {
                "Joints": {"_": ["Fixed", "Revolute"]},
                "_": ["Ground Plane"],
            }
        }
        paths = get_all_menu_paths(menu_dict)
        self.assertIn("Physics/Ground Plane", paths)
        self.assertIn("Physics/Joints/Fixed", paths)
        self.assertIn("Physics/Joints/Revolute", paths)

    async def test_get_all_menu_paths_empty(self) -> None:
        """Verify that an empty dict returns an empty list."""
        self.assertEqual(get_all_menu_paths({}), [])


class TestCountMenuItems(TimedAsyncTestCase):
    """Tests for the count_menu_items helper (pure data, no UI required)."""

    async def test_count_menu_items_basic(self) -> None:
        """Verify item counting on a simple menu dict."""
        menu_dict = {"Sensors": {"_": ["Contact", "IMU"]}, "Robots": {"_": ["Franka"]}}
        self.assertEqual(count_menu_items(menu_dict), 3)

    async def test_count_menu_items_nested(self) -> None:
        """Verify counting traverses nested dicts."""
        menu_dict = {
            "A": {"B": {"_": ["x", "y"]}, "_": ["z"]},
        }
        self.assertEqual(count_menu_items(menu_dict), 3)

    async def test_count_menu_items_empty(self) -> None:
        """Verify that an empty dict yields zero."""
        self.assertEqual(count_menu_items({}), 0)


class TestListMenuPaths(TimedAsyncTestCase):
    """Tests for the list_menu_paths function (requires windowed mode with menubar)."""

    async def setUp(self) -> None:
        """Set up a new stage and wait for menus to populate."""
        await super().setUp()
        await stage_utils.create_new_stage_async()
        await omni.kit.app.get_app().next_update_async()

    async def test_list_menu_paths_returns_nonempty(self) -> None:
        """Verify that list_menu_paths returns a non-empty list of paths."""
        if not _has_menubar():
            self.skipTest("No menubar available (minimal test harness)")
        paths = list_menu_paths()
        self.assertIsInstance(paths, list)
        self.assertGreater(len(paths), 0, "Expected at least one menu path")

    async def test_list_menu_paths_returns_sorted(self) -> None:
        """Verify that list_menu_paths returns paths in sorted order."""
        if not _has_menubar():
            self.skipTest("No menubar available (minimal test harness)")
        paths = list_menu_paths()
        self.assertEqual(paths, sorted(paths))

    async def test_list_menu_paths_contains_known_paths(self) -> None:
        """Verify that well-known menu paths are present."""
        if not _has_menubar():
            self.skipTest("No menubar available (minimal test harness)")
        paths = list_menu_paths()
        known_paths = ["File/New", "File/Save", "Edit/Undo", "Create/Mesh/Cube"]
        for expected in known_paths:
            self.assertIn(expected, paths, f"Expected menu path '{expected}' not found")

    async def test_list_menu_paths_uses_slash_separator(self) -> None:
        """Verify that all returned paths use forward-slash separators."""
        if not _has_menubar():
            self.skipTest("No menubar available (minimal test harness)")
        paths = list_menu_paths()
        for path in paths:
            self.assertNotIn("\\", path, f"Path contains backslash: {path}")
            self.assertIn("/", path, f"Path has no separator: {path}")

    async def test_list_menu_paths_max_depth_1(self) -> None:
        """Verify that max_depth=1 returns only top-level item names."""
        if not _has_menubar():
            self.skipTest("No menubar available (minimal test harness)")
        paths = list_menu_paths(max_depth=1)
        for path in paths:
            self.assertLessEqual(path.count("/"), 1, f"max_depth=1 returned deep path: {path}")

    async def test_list_menu_paths_max_depth_limits_nesting(self) -> None:
        """Verify that higher max_depth returns deeper paths."""
        if not _has_menubar():
            self.skipTest("No menubar available (minimal test harness)")
        paths_shallow = list_menu_paths(max_depth=1)
        paths_deep = list_menu_paths(max_depth=3)
        self.assertGreaterEqual(len(paths_deep), len(paths_shallow))

    async def test_list_menu_paths_no_menubar_returns_empty(self) -> None:
        """Verify that list_menu_paths returns an empty list when no menubar exists."""
        if _has_menubar():
            self.skipTest("Menubar is present; cannot test no-menubar fallback")
        # In headless mode get_menubar() may raise or return None — either way
        # list_menu_paths should gracefully return an empty list.
        paths = list_menu_paths()
        self.assertIsInstance(paths, list)
        self.assertEqual(len(paths), 0)


class TestFindWidgetWithRetry(TimedAsyncTestCase):
    """Tests for find_widget_with_retry and find_enabled_widget_with_retry."""

    async def setUp(self) -> None:
        """Set up a new stage so the UI is populated."""
        await super().setUp()
        await stage_utils.create_new_stage_async()
        await omni.kit.app.get_app().next_update_async()

    async def test_find_widget_viewport(self) -> None:
        """Verify that the Viewport window can be found."""
        if not _has_menubar():
            self.skipTest("No UI available (minimal test harness)")
        widget = await find_widget_with_retry("Viewport", max_frames=50)
        self.assertIsNotNone(widget)

    async def test_find_widget_not_found_raises(self) -> None:
        """Verify TimeoutError for a widget that does not exist."""
        with self.assertRaises(TimeoutError):
            await find_widget_with_retry("NonExistentWidget_12345", max_frames=5)

    async def test_find_enabled_widget(self) -> None:
        """Verify that an enabled widget can be found."""
        if not _has_menubar():
            self.skipTest("No UI available (minimal test harness)")
        widget = await find_enabled_widget_with_retry("Viewport", max_frames=50)
        self.assertIsNotNone(widget)

    async def test_find_enabled_widget_not_found_raises(self) -> None:
        """Verify TimeoutError for a widget that does not exist."""
        with self.assertRaises(TimeoutError):
            await find_enabled_widget_with_retry("NonExistentWidget_12345", max_frames=5)


class TestPerformWidgetAction(TimedAsyncTestCase):
    """Tests for the perform_widget_action function."""

    async def setUp(self) -> None:
        """Set up a new stage so the UI is populated."""
        await super().setUp()
        await stage_utils.create_new_stage_async()
        await omni.kit.app.get_app().next_update_async()

    async def test_perform_widget_action_read(self) -> None:
        """Verify that action=read returns a dict with expected keys."""
        if not _has_menubar():
            self.skipTest("No UI available (minimal test harness)")
        result = await perform_widget_action("Viewport", action="read", max_frames=50)
        self.assertIsInstance(result, dict)
        self.assertIn("type", result)
        self.assertIn("visible", result)
        self.assertTrue(result["visible"])

    async def test_perform_widget_action_invalid_raises(self) -> None:
        """Verify ValueError for an unknown action."""
        if not _has_menubar():
            self.skipTest("No UI available (minimal test harness)")
        with self.assertRaises(ValueError):
            await perform_widget_action("Viewport", action="invalid_action", max_frames=50)

    async def test_perform_widget_action_not_found_raises(self) -> None:
        """Verify TimeoutError when the widget is not found."""
        with self.assertRaises(TimeoutError):
            await perform_widget_action("NonExistentWidget_12345", action="click", max_frames=5)


class TestMenuClickWithRetry(TimedAsyncTestCase):
    """Tests for menu_click_with_retry (requires windowed mode)."""

    async def setUp(self) -> None:
        """Set up a new stage and wait for menus."""
        await super().setUp()
        await stage_utils.create_new_stage_async()
        await omni.kit.app.get_app().next_update_async()

    async def test_menu_click_file_new(self) -> None:
        """Verify that File/New can be clicked without error."""
        if not _has_menubar():
            self.skipTest("No menubar available (minimal test harness)")
        result = await menu_click_with_retry("File/New")
        self.assertIsNone(result)

    async def test_menu_click_create_mesh_cube(self) -> None:
        """Verify that Create/Mesh/Cube adds a Cube prim to the stage."""
        if not _has_menubar():
            self.skipTest("No menubar available (minimal test harness)")
        await menu_click_with_retry("Create/Mesh/Cube")
        stage = stage_utils.get_current_stage()
        cube_found = any(prim.GetTypeName() == "Cube" for prim in stage.Traverse())
        self.assertTrue(cube_found, "No Cube prim found on stage after menu click")


class TestNavigateMenuVisual(TimedAsyncTestCase):
    """Tests for navigate_menu_visual (visual menu navigation with cursor tracking)."""

    async def setUp(self) -> None:
        """Set up a new stage and wait for menus."""
        await super().setUp()
        await stage_utils.create_new_stage_async()
        await omni.kit.app.get_app().next_update_async()

    async def test_navigate_create_mesh_cube(self) -> None:
        """Verify that navigate_menu_visual successfully clicks Create/Mesh/Cube."""
        if not _has_menubar():
            self.skipTest("No menubar available (minimal test harness)")
        result = await navigate_menu_visual("Create/Mesh/Cube")
        self.assertTrue(result, "navigate_menu_visual returned False")
        stage = stage_utils.get_current_stage()
        cube_found = any(prim.GetTypeName() == "Cube" for prim in stage.Traverse())
        self.assertTrue(cube_found, "No Cube prim found on stage after visual navigation")

    async def test_on_frame_callback_invoked(self) -> None:
        """Verify that the on_frame callback is called with cursor positions."""
        if not _has_menubar():
            self.skipTest("No menubar available (minimal test harness)")
        positions = []

        async def on_frame(x: float, y: float) -> None:
            positions.append((x, y))

        result = await navigate_menu_visual("Create/Mesh/Sphere", on_frame=on_frame)
        self.assertTrue(result)
        self.assertGreater(len(positions), 0, "on_frame callback was never called")
        # All positions should be finite numbers
        for x, y in positions:
            self.assertTrue(isinstance(x, (int, float)), f"x is not a number: {x}")
            self.assertTrue(isinstance(y, (int, float)), f"y is not a number: {y}")

    async def test_callback_receives_multiple_positions(self) -> None:
        """Verify that the callback receives positions for each menu level."""
        if not _has_menubar():
            self.skipTest("No menubar available (minimal test harness)")
        positions = []

        async def on_frame(x: float, y: float) -> None:
            positions.append((x, y))

        await navigate_menu_visual("Create/Mesh/Cube", hover_frames=2, leaf_hover_frames=2, on_frame=on_frame)
        # Should have positions from at least 3 levels: Create, Mesh, Cube
        # plus interpolation steps and hover frames
        unique_rounded = {(round(x), round(y)) for x, y in positions}
        self.assertGreaterEqual(len(unique_rounded), 3, f"Expected at least 3 distinct positions, got {unique_rounded}")

    async def test_invalid_menu_returns_false(self) -> None:
        """Verify that navigating a non-existent menu returns False."""
        if not _has_menubar():
            self.skipTest("No menubar available (minimal test harness)")
        result = await navigate_menu_visual("NonExistent/Menu/Item")
        self.assertFalse(result)

    async def test_custom_hover_frames(self) -> None:
        """Verify that hover_frames and leaf_hover_frames affect callback count."""
        if not _has_menubar():
            self.skipTest("No menubar available (minimal test harness)")
        short_positions = []
        long_positions = []

        async def on_short(x: float, y: float) -> None:
            short_positions.append((x, y))

        async def on_long(x: float, y: float) -> None:
            long_positions.append((x, y))

        await navigate_menu_visual("File/New", hover_frames=2, leaf_hover_frames=2, on_frame=on_short)
        # New stage for second test
        await stage_utils.create_new_stage_async()
        await omni.kit.app.get_app().next_update_async()
        await navigate_menu_visual("File/New", hover_frames=8, leaf_hover_frames=12, on_frame=on_long)
        self.assertGreater(
            len(long_positions), len(short_positions), "More hover frames should produce more callback invocations"
        )
