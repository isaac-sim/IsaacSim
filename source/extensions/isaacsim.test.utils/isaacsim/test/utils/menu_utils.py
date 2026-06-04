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

"""Utilities for navigating and testing UI menus in Isaac Sim."""

from __future__ import annotations

from typing import Any

import carb
import omni.kit.app
import omni.kit.ui_test as ui_test
from omni import ui
from omni.kit.ui_test import Vec2, emulate_mouse_move, emulate_mouse_move_and_click, get_menubar, wait_n_updates

# Maximum number of frames to poll when waiting for a widget to become
# findable, enabled, or for a submenu to become visible after clicking.
_DEFAULT_MAX_WAIT_FRAMES = 100


async def _poll_async(check_fn: callable, max_frames: int, label: str = "") -> Any:
    """Poll *check_fn* each frame until it returns a truthy value.

    Args:
        check_fn: Callable returning a truthy value on success, falsy on failure.
        max_frames: Maximum number of app-update frames to poll.
        label: Label for log messages.

    Returns:
        The first truthy value from *check_fn*, or ``None`` if timed out.
    """
    for frame in range(max_frames):
        result = check_fn()
        if result:
            if frame > 0:
                carb.log_info(f"[{label}] succeeded after {frame} extra frame(s)")
            return result
        await omni.kit.app.get_app().next_update_async()
    return None


def _ui_find(query: str, parent: Any = None) -> Any:
    """Find a widget, optionally scoped to *parent*.

    Args:
        query: The widget query string.
        parent: Optional parent widget to search within.

    Returns:
        The found widget, or None if not found.
    """
    return parent.find(query) if parent is not None else ui_test.find(query)


async def find_widget_with_retry(query: str, max_frames: int = _DEFAULT_MAX_WAIT_FRAMES, parent: Any = None) -> Any:
    """Poll ``ui_test.find`` until the widget is found or *max_frames* is exceeded.

    This is useful when a UI element may not be immediately available after a
    menu click or navigation action.  Instead of a fixed ``human_delay``, this
    function actively polls each frame so the test proceeds as soon as the
    widget appears.

    Args:
        query: The widget query string (same syntax as ``omni.kit.ui_test.find``).
        max_frames: Maximum number of app-update frames to wait before giving up.
        parent: Optional parent widget to search within.  When provided,
            ``parent.find(query)`` is used instead of ``ui_test.find(query)``.

    Returns:
        The found widget reference.

    Raises:
        TimeoutError: If the widget is not found within *max_frames*.
    """
    result = await _poll_async(lambda: _ui_find(query, parent), max_frames, "find_widget_with_retry")
    if result is None:
        raise TimeoutError(f"Widget '{query}' not found after {max_frames} frames")
    return result


async def scroll_to_widget(widget_ref: Any, settle_frames: int = 3) -> bool:
    """Scroll the enclosing ``ui.ScrollingFrame`` so ``widget_ref`` is visible.

    Walks the window tree containing ``widget_ref`` to find the nearest
    ``ui.ScrollingFrame`` ancestor, then sets ``scroll_y`` so the widget lies
    inside the visible viewport. Use this before ``click()`` when a widget may
    have been pushed off-screen by sibling content (e.g. a dynamic list
    growing above the target).

    Args:
        widget_ref: A ``WidgetRef`` (e.g. returned by :func:`find_widget_with_retry`).
        settle_frames: Number of frames to wait after scrolling so the layout settles.

    Returns:
        True if the widget was scrolled into view, False if no enclosing
        ``ScrollingFrame`` was found.
    """
    target = widget_ref.widget
    window = getattr(widget_ref, "window", None)
    if window is None or window.frame is None:
        return False

    def _find_ancestors(root: ui.Widget, needle: ui.Widget, trail: list) -> list | None:
        for child in ui.Inspector.get_children(root):
            if child is None:
                continue
            if child is needle:
                return trail
            if isinstance(child, (ui.Container, ui.TreeView)):
                found = _find_ancestors(child, needle, trail + [child])
                if found is not None:
                    return found
        return None

    ancestors = _find_ancestors(window.frame, target, [window.frame])
    if not ancestors:
        return False

    scrolling_frame = next((a for a in reversed(ancestors) if isinstance(a, ui.ScrollingFrame)), None)
    if scrolling_frame is None:
        return False

    # Use widget vs frame screen positions to compute the needed scroll
    # offset, then clamp into ``[0, scroll_y_max]``. Falling back to
    # ``scroll_y_max`` covers cases where the widget is below the viewport
    # but layout hasn't reported usable coordinates yet.
    widget_top = target.screen_position_y
    widget_bottom = widget_top + target.computed_height
    frame_top = scrolling_frame.screen_position_y
    frame_bottom = frame_top + scrolling_frame.computed_height

    if widget_bottom > frame_bottom:
        delta = widget_bottom - frame_bottom
        scrolling_frame.scroll_y = min(scrolling_frame.scroll_y + delta, scrolling_frame.scroll_y_max)
    elif widget_top < frame_top:
        delta = frame_top - widget_top
        scrolling_frame.scroll_y = max(scrolling_frame.scroll_y - delta, 0)
    else:
        return True

    for _ in range(settle_frames):
        await omni.kit.app.get_app().next_update_async()
    return True


async def wait_for_widget_enabled(widget: Any, max_frames: int = _DEFAULT_MAX_WAIT_FRAMES) -> bool:
    """Poll until ``widget.widget.enabled`` becomes True.

    Args:
        widget: A ``WidgetRef`` returned by ``ui_test.find`` or
            :func:`find_widget_with_retry`.
        max_frames: Maximum number of app-update frames to wait.

    Returns:
        True if the widget became enabled within *max_frames*, False otherwise.
    """
    result = await _poll_async(lambda: widget.widget.enabled or None, max_frames, "wait_for_widget_enabled")
    return result is not None


async def find_enabled_widget_with_retry(
    query: str, max_frames: int = _DEFAULT_MAX_WAIT_FRAMES, parent: Any = None
) -> Any:
    """Poll ``ui_test.find`` until the widget is found **and** enabled.

    Combines :func:`find_widget_with_retry` with an enabled check in a single
    polling loop.  This avoids the common two-step pattern of finding a widget
    and then waiting for it to become enabled in a separate loop.

    Args:
        query: The widget query string (same syntax as ``omni.kit.ui_test.find``).
        max_frames: Maximum number of app-update frames to wait before giving up.
        parent: Optional parent widget to search within.

    Returns:
        The found and enabled widget reference.

    Raises:
        TimeoutError: If the widget is not found and enabled within *max_frames*.
    """

    def _check() -> Any:
        r = _ui_find(query, parent)
        return r if r is not None and r.widget.enabled else None

    result = await _poll_async(_check, max_frames, "find_enabled_widget_with_retry")
    if result is None:
        raise TimeoutError(f"Widget '{query}' not found or not enabled after {max_frames} frames")
    return result


async def perform_widget_action(
    query: str,
    action: str = "click",
    text: str = "",
    max_frames: int = _DEFAULT_MAX_WAIT_FRAMES,
) -> dict:
    """Find a UI widget and perform an action on it.

    Combines :func:`find_widget_with_retry` with action dispatch so callers
    do not need to handle the find/act pattern themselves.

    Args:
        query: Widget query string (same syntax as ``omni.kit.ui_test.find``).
        action: Action to perform — one of ``click``, ``double_click``,
            ``right_click``, ``type``, ``read``.
        text: Text to type; only used when *action* is ``"type"``.
        max_frames: Maximum frames to poll for the widget to appear.

    Returns:
        For ``action="read"``: dict of widget properties (type, visible,
        enabled, width, height, and optionally text/value).
        For all other actions: empty dict.

    Raises:
        TimeoutError: If the widget is not found within *max_frames*.
        ValueError: If *action* is not one of the supported values.

    Example:

    .. code-block:: python

        >>> from isaacsim.test.utils import perform_widget_action
        >>>
        >>> await perform_widget_action("Load")
        >>> await perform_widget_action("Search", action="type", text="franka")
        >>> info = await perform_widget_action("Play", action="read")
        >>> print(info)
        {'type': 'Button', 'visible': True, 'enabled': True, ...}
    """
    widget = await find_widget_with_retry(query, max_frames=max_frames)

    if action == "click":
        await widget.click()
    elif action == "double_click":
        await widget.double_click()
    elif action == "right_click":
        await widget.right_click()
    elif action == "type":
        await widget.input(text)
    elif action == "read":
        w = widget.widget
        info: dict = {
            "type": type(w).__name__,
            "visible": getattr(w, "visible", None),
            "enabled": getattr(w, "enabled", None),
            "width": getattr(w, "width", None),
            "height": getattr(w, "height", None),
        }
        if hasattr(w, "text"):
            info["text"] = w.text
        if hasattr(w, "model") and w.model:
            try:
                info["value"] = w.model.get_value_as_string()
            except Exception:
                pass
        return info
    else:
        raise ValueError(f"Unknown action '{action}'. Use: click, double_click, right_click, type, read")

    return {}


async def _wait_for_menu_item(parent_widget: Any, menu_name: str, max_frames: int = _DEFAULT_MAX_WAIT_FRAMES) -> Any:
    """Poll until ``parent_widget.find_menu(menu_name)`` returns a non-None result.

    Args:
        parent_widget: The parent ``MenuRef`` to search within.
        menu_name: The menu item text to search for.
        max_frames: Maximum frames to poll before giving up.

    Returns:
        The found ``MenuRef``, or ``None`` if not found within *max_frames*.
    """
    return await _poll_async(lambda: parent_widget.find_menu(menu_name), max_frames, "menu_click_with_retry")


async def _wait_for_menu_path(menu_path: str, max_frames: int = _DEFAULT_MAX_WAIT_FRAMES) -> bool:
    """Poll until every segment of ``menu_path`` resolves in the menubar.

    Walks the menu tree top-down using :py:meth:`MenuRef.find_menu` without
    opening any submenus.  Used to absorb ``omni.kit.menu.utils.refresh_menu_items``
    callbacks queued by a prior test's window destruction so the next click
    sees a fully rebuilt menu tree.

    Args:
        menu_path: Full menu path separated by ``/`` (e.g. ``"Tools/Replicator/Teleop"``).
        max_frames: Maximum app-update frames to poll before giving up.

    Returns:
        True if every segment was resolved, False otherwise.
    """

    def _resolves() -> bool:
        node = get_menubar()
        for name in menu_path.split("/"):
            node = node.find_menu(name)
            if node is None:
                return False
        return True

    return bool(await _poll_async(_resolves, max_frames, f"menu_click_with_retry/await_path/{menu_path}"))


async def _wait_for_shown(menu_widget: Any, menu_name: str, max_frames: int = _DEFAULT_MAX_WAIT_FRAMES) -> bool:
    """Poll until ``menu_widget.widget.shown`` becomes True.

    Args:
        menu_widget: The ``MenuRef`` whose ``shown`` state to check.
        menu_name: Menu name (used for logging).
        max_frames: Maximum frames to poll before giving up.

    Returns:
        True if the menu became shown, False if timed out.
    """
    result = await _poll_async(
        lambda: True if menu_widget.widget.shown else None, max_frames, f"menu_click_with_retry/{menu_name}"
    )
    return result is not None


async def _navigate_menu(menu_path: str, human_delay_speed: int = 10) -> bool:
    """Navigate through a menu hierarchy step-by-step with polling.

    Unlike ``omni.kit.ui_test.menu_click``, this function polls at each step
    rather than waiting a fixed number of frames.  This avoids both the
    ``AttributeError`` (from ``find_menu`` returning ``None``) and the
    ``carb.log_error`` (from a submenu not becoming visible in time).

    Args:
        menu_path: Full menu path separated by ``/``.
        human_delay_speed: Frames to wait between mouse interactions for
            natural-feeling timing.

    Returns:
        True if the full path was navigated and clicked successfully,
        False otherwise.
    """
    import omni.appwindow

    menu_widget = get_menubar()
    app_win = omni.appwindow.get_default_app_window()

    # Move mouse away from menus first (mirrors menu_click behaviour).
    await emulate_mouse_move(Vec2(app_win.get_size().x - 10, app_win.get_size().y - 10))
    await wait_n_updates(human_delay_speed)

    segments = menu_path.split("/")
    for idx, menu_name in enumerate(segments):
        # Poll until the menu item is findable in the widget tree.
        child = await _wait_for_menu_item(menu_widget, menu_name)
        if child is None:
            carb.log_info(f"[menu_click_with_retry] could not find menu item " f"'{menu_name}' in '{menu_path}'")
            return False

        menu_widget = child

        # Move mouse to the menu item.
        menu_pos = menu_widget.center
        if idx == 0:
            menu_pos += Vec2(10 * ui.Workspace.get_dpi_scale(), 5 * ui.Workspace.get_dpi_scale())
        await emulate_mouse_move(menu_pos)
        await wait_n_updates(human_delay_speed)

        # If this is a submenu (ui.Menu), click to open and poll for shown.
        if isinstance(menu_widget.widget, ui.Menu):
            if not menu_widget.widget.shown:
                await menu_widget.click()
            elif menu_widget.widget.has_triggered_fn():
                menu_widget.widget.call_triggered_fn()

            if not await _wait_for_shown(menu_widget, menu_name):
                carb.log_info(
                    f"[menu_click_with_retry] submenu '{menu_name}' did " f"not become shown in '{menu_path}'"
                )
                return False
        else:
            # Leaf menu item -- click it.
            await menu_widget.click()
            await wait_n_updates(human_delay_speed)

    await wait_n_updates(human_delay_speed)
    return True


async def menu_click_with_retry(
    menu_path: str, delays: list[int] = None, window_name: str = None, wait_n_frames: int = 10
) -> Any:
    """Click a menu item with retry at different delay speeds.

    First attempts a direct programmatic click via ``omni.kit.ui_test.menu_click``
    which works in both windowed and headless (``--no-window``) modes.  If that
    fails (e.g. due to slow submenus), falls back to step-by-step mouse-emulation
    navigation via :func:`_navigate_menu` with increasing delays.

    Args:
        menu_path: The menu path to click (e.g., "Create/Sensors/Contact Sensor").
        delays: List of delay values (in frames) to use for ``human_delay_speed``
            on each fallback attempt.
        window_name: Optional window name to check for after clicking.
            If provided, the function returns early when the window is found
            and returns the window widget.
        wait_n_frames: Number of frames to wait after a successful click when
            ``window_name`` is not provided.

    Returns:
        The found window widget if window_name is provided and found, else None.

    Example:

    .. code-block:: python

        >>> from isaacsim.test.utils import menu_click_with_retry
        >>>
        >>> # Simple menu click
        >>> await menu_click_with_retry("Create/Sensors/Contact Sensor")
        >>>
        >>> # Menu click that waits for a window to appear
        >>> window = await menu_click_with_retry(
        ...     "Tools/Robotics/OmniGraph Controllers/Differential Controller",
        ...     window_name="Differential Controller"
        ... )
    """
    delays = delays or [5, 10, 20]

    carb.log_info(
        f"[menu_click_with_retry] starting: menu_path='{menu_path}', " f"window_name={window_name!r}, delays={delays}"
    )

    # Wait for the menu tree to settle before clicking. ``omni.kit.menu.utils``
    # rebuilds the menubar asynchronously when an extension's window is
    # destroyed (``refresh_menu_items``); without this pre-flight, the very
    # next ``ui_test.menu_click`` can race the rebuild and emit
    # ``[Error] [omni.kit.ui_test.query] ui.Menu item failed to become show``.
    if not await _wait_for_menu_path(menu_path):
        carb.log_info(
            f"[menu_click_with_retry] menu path '{menu_path}' did not resolve in the menubar before timeout; "
            f"proceeding anyway"
        )

    # --- Primary path: direct programmatic menu click (works headless) ---
    try:
        await ui_test.menu_click(menu_path, human_delay_speed=3)
        for _ in range(wait_n_frames):
            await omni.kit.app.get_app().next_update_async()

        if window_name:
            if (window := ui_test.find(window_name)) is not None:
                carb.log_info(f"[menu_click_with_retry] window '{window_name}' found via ui_test.menu_click")
                return window
            carb.log_info(
                f"[menu_click_with_retry] ui_test.menu_click succeeded but window '{window_name}' not found, "
                f"falling back to _navigate_menu"
            )
        else:
            carb.log_info(f"[menu_click_with_retry] succeeded via ui_test.menu_click for '{menu_path}'")
            return None
    except Exception as e:
        carb.log_info(f"[menu_click_with_retry] ui_test.menu_click failed for '{menu_path}': {e}")

    # --- Fallback: step-by-step mouse-emulation navigation ---
    for i, delay in enumerate(delays):
        carb.log_info(
            f"[menu_click_with_retry] fallback attempt {i + 1}/{len(delays)} "
            f"(human_delay_speed={delay}) for '{menu_path}'"
        )

        success = await _navigate_menu(menu_path, human_delay_speed=delay)

        if not success:
            carb.log_info(
                f"[menu_click_with_retry] navigation failed on attempt " f"{i + 1}/{len(delays)} for '{menu_path}'"
            )
            continue

        if window_name:
            if (window := ui_test.find(window_name)) is not None:
                carb.log_info(
                    f"[menu_click_with_retry] window '{window_name}' found " f"on attempt {i + 1}/{len(delays)}"
                )
                return window
            carb.log_info(
                f"[menu_click_with_retry] window '{window_name}' not found " f"on attempt {i + 1}/{len(delays)}"
            )
        else:
            carb.log_info(f"[menu_click_with_retry] succeeded on attempt " f"{i + 1}/{len(delays)} for '{menu_path}'")
            for _ in range(wait_n_frames):
                await omni.kit.app.get_app().next_update_async()
            return None

    carb.log_info(f"[menu_click_with_retry] all attempts exhausted for '{menu_path}'")
    return None


def list_menu_paths(max_depth: int = 3) -> list[str]:
    """Return all menu item paths from the live menubar up to *max_depth* levels.

    Walks the widget tree without opening any menus.  Returns slash-separated
    paths suitable for use with :func:`menu_click_with_retry`.

    Args:
        max_depth: Maximum recursion depth (1 = top-level names only,
            2 = one level of submenus, etc.).

    Returns:
        Sorted list of all discoverable menu paths.

    Example:

    .. code-block:: python

        >>> from isaacsim.test.utils import list_menu_paths
        >>>
        >>> paths = list_menu_paths()
        >>> for p in paths:
        ...     print(p)
        Create/Lights/Cylinder Light
        Create/Mesh/Capsule
        File/New
        File/Open
        ...
    """
    try:
        menubar = get_menubar()
    except Exception:
        return []
    if menubar is None:
        return []

    paths: list[str] = []

    def _walk(widget_ref: Any, prefix: str, depth: int) -> None:
        if depth > max_depth:
            return
        for child in widget_ref.find_all("*"):
            name = getattr(child.widget, "text", None)
            if not name:
                continue
            path = f"{prefix}/{name}" if prefix else name
            children = child.find_all("*")
            if children:
                _walk(child, path, depth + 1)
            else:
                paths.append(path)

    _walk(menubar, "", 1)
    return sorted(paths)


def get_all_menu_paths(menu_dict: dict, current_path: str = "", root_path: str = "") -> list[str]:
    """Extract all leaf menu paths from a menu dictionary structure.

    This function recursively traverses a menu dictionary (as returned by
    `omni.kit.ui_test.get_context_menu()`) and extracts all leaf menu item paths.

    Args:
        menu_dict: The menu dictionary structure from get_context_menu.
        current_path: The current path prefix (used for recursion).
        root_path: Optional prefix to prepend to all returned paths.

    Returns:
        A list of all menu paths as strings joined by forward slashes.

    Example:

    .. code-block:: python

        >>> from isaacsim.test.utils import get_all_menu_paths
        >>>
        >>> menu_dict = {"Lidar": {"_": ["Sensor A", "Sensor B"]}}
        >>> paths = get_all_menu_paths(menu_dict, root_path="Create/Sensors")
        >>> paths
        ['Create/Sensors/Lidar/Sensor A', 'Create/Sensors/Lidar/Sensor B']
    """
    paths = []
    for key, value in menu_dict.items():
        # The "_" key contains leaf items at the current level
        if key == "_":
            if isinstance(value, list):
                for item in value:
                    path = f"{current_path}/{item}" if current_path else item
                    paths.append(f"{root_path}/{path}" if root_path else path)
            elif isinstance(value, str):
                path = f"{current_path}/{value}" if current_path else value
                paths.append(f"{root_path}/{path}" if root_path else path)
        elif isinstance(value, dict):
            new_path = f"{current_path}/{key}" if current_path else key
            paths.extend(get_all_menu_paths(value, new_path, root_path))
        elif isinstance(value, list):
            for item in value:
                path = f"{current_path}/{key}/{item}" if current_path else f"{key}/{item}"
                paths.append(f"{root_path}/{path}" if root_path else path)
        elif isinstance(value, str):
            path = f"{current_path}/{key}/{value}" if current_path else f"{key}/{value}"
            paths.append(f"{root_path}/{path}" if root_path else path)
    return paths


async def navigate_menu_visual(
    menu_path: str,
    *,
    hover_frames: int = 5,
    leaf_hover_frames: int = 10,
    on_frame: callable = None,
) -> bool:
    """Navigate a menu hierarchy with visual cursor movement suitable for video capture.

    Unlike :func:`menu_click_with_retry`, this function moves the emulated mouse through
    each menu level using an L-shaped path (horizontal then vertical) that keeps the
    cursor inside menu bounds and prevents submenus from closing prematurely. At each
    step, an optional callback is invoked to allow frame capture or cursor logging.

    Args:
        menu_path: Slash-separated menu path (e.g. ``"Create/Mesh/Cube"``).
        hover_frames: Number of frames to dwell on intermediate menu items.
        leaf_hover_frames: Number of frames to dwell on the final item before clicking.
        on_frame: Optional async callback ``async def(x, y)`` invoked after each
            mouse move or hover frame, receiving the current cursor position.
            Use this to capture screenshots or log cursor positions.

    Returns:
        True if the menu was successfully navigated and clicked, False otherwise.

    Example:

    .. code-block:: python

        >>> from isaacsim.test.utils.menu_utils import navigate_menu_visual
        >>>
        >>> positions = []
        >>> async def log_pos(x, y):
        ...     positions.append((x, y))
        >>>
        >>> await navigate_menu_visual("Create/Mesh/Cube", on_frame=log_pos)
        True
    """
    menu_widget = get_menubar()
    if menu_widget is None:
        carb.log_error("[navigate_menu_visual] No menubar found")
        return False

    segments = menu_path.split("/")
    current_widget = menu_widget
    cursor_x, cursor_y = 0.0, 0.0

    async def _notify(x: float, y: float) -> None:
        nonlocal cursor_x, cursor_y
        cursor_x, cursor_y = x, y
        if on_frame is not None:
            await on_frame(x, y)

    for idx, name in enumerate(segments):
        # Find the menu item
        child = await _wait_for_menu_item(current_widget, name)
        if child is None:
            carb.log_info(f"[navigate_menu_visual] could not find '{name}' in '{menu_path}'")
            return False

        cx, cy = child.center.x, child.center.y
        is_last = idx == len(segments) - 1

        if idx == 0:
            # Top-level: smooth move from current position
            steps = 5
            sx, sy = cursor_x, cursor_y
            for s in range(1, steps + 1):
                t = s / steps
                ix = sx + (cx - sx) * t
                iy = sy + (cy - sy) * t
                await emulate_mouse_move(Vec2(ix, iy))
                await omni.kit.app.get_app().next_update_async()
                await _notify(ix, iy)
        else:
            # Submenu: L-shaped move (horizontal first, then vertical)
            old_x, old_y = cursor_x, cursor_y
            h_steps = 2
            for s in range(1, h_steps + 1):
                t = s / h_steps
                ix = old_x + (cx - old_x) * t
                await emulate_mouse_move(Vec2(ix, old_y))
                await omni.kit.app.get_app().next_update_async()
                await _notify(ix, old_y)
            v_steps = 2
            for s in range(1, v_steps + 1):
                t = s / v_steps
                iy = old_y + (cy - old_y) * t
                await emulate_mouse_move(Vec2(cx, iy))
                await omni.kit.app.get_app().next_update_async()
                await _notify(cx, iy)

        # Keep mouse on the item
        await emulate_mouse_move(Vec2(cx, cy))

        n_hover = leaf_hover_frames if is_last else hover_frames
        for _ in range(n_hover):
            await omni.kit.app.get_app().next_update_async()
            await _notify(cx, cy)

        if is_last:
            # Click the leaf item
            await emulate_mouse_move_and_click(Vec2(cx, cy))
            for _ in range(5):
                await omni.kit.app.get_app().next_update_async()
        else:
            # Open the submenu
            if isinstance(child.widget, ui.Menu):
                if not child.widget.shown:
                    await emulate_mouse_move_and_click(Vec2(cx, cy))
                    await omni.kit.app.get_app().next_update_async()
                    await emulate_mouse_move(Vec2(cx, cy))
                for _ in range(30):
                    if child.widget.shown:
                        break
                    await omni.kit.app.get_app().next_update_async()
                # Keep mouse on parent to stabilize submenu
                for _ in range(3):
                    await emulate_mouse_move(Vec2(cx, cy))
                    await omni.kit.app.get_app().next_update_async()

        current_widget = child
        await _notify(cx, cy)

    carb.log_info(f"[navigate_menu_visual] completed: '{menu_path}'")
    return True


def count_menu_items(menu_dict: dict) -> int:
    """Recursively count the total number of leaf items in a menu dictionary.

    Args:
        menu_dict: The menu dictionary structure from get_context_menu.

    Returns:
        The total number of leaf menu items.

    Example:

    .. code-block:: python

        >>> from isaacsim.test.utils import count_menu_items
        >>>
        >>> menu_dict = {"Sensors": {"_": ["Contact", "IMU"]}, "Robots": {"_": ["Franka"]}}
        >>> count_menu_items(menu_dict)
        3
    """
    count = 0
    for key, item in menu_dict.items():
        if isinstance(item, dict):
            count += count_menu_items(item)
        elif isinstance(item, list):
            count += len(item)
    return count
