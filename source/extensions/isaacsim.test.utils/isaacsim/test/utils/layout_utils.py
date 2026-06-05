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

"""Layout and dock management utilities for UI automation and testing.

Provides generic helpers to reset the application layout, resize dock panels,
manage window visibility, and close stale windows. These utilities are not
tied to any specific panel or extension.

Each function has a sync version (uses ``update_app``) and an async version
(uses ``await update_app_async``). Prefer the async versions when running
inside the python_server to avoid blocking other asyncio tasks.
"""

from __future__ import annotations

import carb
import isaacsim.core.experimental.utils.app as app_utils
import omni.ui as ui

__all__ = [
    "close_windows",
    "ensure_dock_height",
    "ensure_dock_height_async",
    "ensure_window_visible",
    "ensure_window_visible_async",
    "reset_to_default_layout",
    "reset_to_default_layout_async",
]


def ensure_dock_height(window_title: str, min_height: int = 400) -> bool:
    """Resize the dock containing a window to at least the given height.

    Useful when UI elements at the bottom of a docked panel are off-screen
    because the dock is too short.

    Args:
        window_title: Title of the docked window.
        min_height: Minimum dock height in pixels.

    Returns:
        ``True`` if the dock was found and is now at least ``min_height``.

    Example:

    .. code-block:: python

        >>> from isaacsim.test.utils.layout_utils import ensure_dock_height
        >>> ensure_dock_height("Property", 500)
    """
    for w in ui.Workspace.get_windows():
        if w.title == window_title and hasattr(w, "dock_id"):
            current_h = ui.Workspace.get_dock_id_height(w.dock_id)
            if current_h < min_height:
                ui.Workspace.set_dock_id_height(w.dock_id, min_height)
                app_utils.update_app(steps=15)
                new_h = ui.Workspace.get_dock_id_height(w.dock_id)
                carb.log_info(f"Dock for '{window_title}' resized: {current_h:.0f} -> {new_h:.0f}")
            return True
    carb.log_warn(f"Window '{window_title}' not found or not docked")
    return False


async def ensure_dock_height_async(window_title: str, min_height: int = 400) -> bool:
    """Async version of :func:`ensure_dock_height`.

    Uses ``await update_app_async`` instead of ``update_app`` to avoid blocking
    other asyncio tasks when running inside the python_server.

    Args:
        window_title: Title of the docked window.
        min_height: Minimum dock height in pixels.

    Returns:
        ``True`` if the dock was found and is now at least ``min_height``.
    """
    for w in ui.Workspace.get_windows():
        if w.title == window_title and hasattr(w, "dock_id"):
            current_h = ui.Workspace.get_dock_id_height(w.dock_id)
            if current_h < min_height:
                ui.Workspace.set_dock_id_height(w.dock_id, min_height)
                await app_utils.update_app_async(steps=15)
                new_h = ui.Workspace.get_dock_id_height(w.dock_id)
                carb.log_info(f"Dock for '{window_title}' resized: {current_h:.0f} -> {new_h:.0f}")
            return True
    carb.log_warn(f"Window '{window_title}' not found or not docked")
    return False


def ensure_window_visible(window_title: str, focus: bool = True) -> bool:
    """Make a window visible and optionally focus it.

    Args:
        window_title: Title of the window.
        focus: Whether to also focus the window.

    Returns:
        ``True`` if the window was found, ``False`` otherwise.

    Example:

    .. code-block:: python

        >>> from isaacsim.test.utils.layout_utils import ensure_window_visible
        >>> ensure_window_visible("Console")
    """
    for w in ui.Workspace.get_windows():
        if w.title == window_title:
            w.visible = True
            if focus:
                w.focus()
            app_utils.update_app(steps=5)
            return True
    return False


async def ensure_window_visible_async(window_title: str, focus: bool = True) -> bool:
    """Async version of :func:`ensure_window_visible`.

    Args:
        window_title: Title of the window.
        focus: Whether to also focus the window.

    Returns:
        ``True`` if the window was found, ``False`` otherwise.
    """
    for w in ui.Workspace.get_windows():
        if w.title == window_title:
            w.visible = True
            if focus:
                w.focus()
            await app_utils.update_app_async(steps=5)
            return True
    return False


def close_windows(titles: list[str]) -> list[str]:
    """Hide windows by title.

    Args:
        titles: List of window titles to close (hide).

    Returns:
        List of titles that were found and closed.

    Example:

    .. code-block:: python

        >>> from isaacsim.test.utils.layout_utils import close_windows
        >>> closed = close_windows(["Inspector", "TestMenu"])
    """
    closed: list[str] = []
    for w in ui.Workspace.get_windows():
        if w.title in titles:
            w.visible = False
            closed.append(w.title)
            carb.log_info(f"Closed window: {w.title}")
    return closed


def reset_to_default_layout(
    close_extra_windows: list[str] | None = None,
    focus_window: str = "Content",
) -> None:
    """Reset the application to its default layout.

    Stops any running simulation, executes the ``layout_default`` action
    to restore the built-in window arrangement, closes stale windows, and
    focuses a specified tab.

    This function is intentionally generic — it does not resize specific
    docks or hide specific panels. Callers should chain additional setup
    (e.g. :func:`ensure_dock_height`, :func:`close_windows`) after this
    for task-specific layout needs.

    Args:
        close_extra_windows: Window titles to close after resetting.
            Defaults to ``["Inspector", "TestMenu"]``.
        focus_window: Window title to focus after reset.

    Example:

    .. code-block:: python

        >>> from isaacsim.test.utils.layout_utils import reset_to_default_layout
        >>> reset_to_default_layout()
    """
    import omni.kit.actions.core

    if close_extra_windows is None:
        close_extra_windows = ["Inspector", "TestMenu"]

    # Stop simulation if running
    if app_utils.is_playing():
        app_utils.stop()
        app_utils.update_app(steps=10)

    # Reset to default layout
    action_reg = omni.kit.actions.core.get_action_registry()
    action_reg.execute_action("isaacsim.app.setup", "layout_default")
    app_utils.update_app(steps=60)

    # Close stale windows
    if close_extra_windows:
        close_windows(close_extra_windows)

    # Focus the requested tab
    ensure_window_visible(focus_window)
    app_utils.update_app(steps=10)


async def reset_to_default_layout_async(
    close_extra_windows: list[str] | None = None,
    focus_window: str = "Content",
) -> None:
    """Async version of :func:`reset_to_default_layout`.

    Uses ``await update_app_async`` to yield to the event loop between steps,
    allowing other asyncio tasks (UI refresh, HTTP server, notifications) to
    run without triggering "Cannot enter into task" errors.

    Args:
        close_extra_windows: Window titles to close after resetting.
            Defaults to ``["Inspector", "TestMenu"]``.
        focus_window: Window title to focus after reset.

    Example:

    .. code-block:: python

        >>> from isaacsim.test.utils.layout_utils import reset_to_default_layout_async
        >>> await reset_to_default_layout_async()
    """
    import omni.kit.actions.core

    if close_extra_windows is None:
        close_extra_windows = ["Inspector", "TestMenu"]

    # Stop simulation if running
    if app_utils.is_playing():
        app_utils.stop()
        await app_utils.update_app_async(steps=10)

    # Reset to default layout
    action_reg = omni.kit.actions.core.get_action_registry()
    action_reg.execute_action("isaacsim.app.setup", "layout_default")
    await app_utils.update_app_async(steps=60)

    # Close stale windows
    if close_extra_windows:
        close_windows(close_extra_windows)

    # Focus the requested tab
    await ensure_window_visible_async(focus_window)
    await app_utils.update_app_async(steps=10)
