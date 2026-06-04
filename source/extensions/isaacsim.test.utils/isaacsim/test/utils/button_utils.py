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

"""Button discovery and deferred click utilities for UI automation.

Provides generic helpers to discover buttons on any UI, compute their screen
coordinates, and schedule deferred mouse clicks that work around the asyncio
reentrancy issue in the python_server extension.

The deferred click pattern schedules a mouse click via ``asyncio.ensure_future()``
so it fires on the **next** event loop cycle — after the calling task returns.
This is necessary because button callbacks that use ``ensure_future()`` internally
cannot run while the python_server's ``_await_and_reply`` task holds the event loop.
"""

from __future__ import annotations

import asyncio

__all__ = [
    "get_widget_screen_center",
    "deferred_click",
    "deferred_click_widget",
    "discover_template_buttons",
]


def get_widget_screen_center(widget: object) -> tuple[float, float]:
    """Get the screen-space center coordinates of a UI widget.

    Works with any ``omni.ui`` widget that exposes ``screen_position_x``,
    ``screen_position_y``, ``computed_width``, and ``computed_height``.

    Args:
        widget: An ``omni.ui`` widget instance.

    Returns:
        Screen coordinates as (x, y).

    Example:

    .. code-block:: python

        >>> from isaacsim.test.utils.button_utils import get_widget_screen_center
        >>> x, y = get_widget_screen_center(some_button)
    """
    return (
        widget.screen_position_x + widget.computed_width / 2,
        widget.screen_position_y + widget.computed_height / 2,
    )


def deferred_click(x: float, y: float) -> None:
    """Schedule a mouse click at screen coordinates for the next event loop cycle.

    The click is dispatched via ``asyncio.ensure_future()`` so it fires
    **after** the current task (e.g. the python_server's ``_await_and_reply``)
    returns and releases the event loop. This is how a real human mouse click
    arrives — via the OS event queue on the main loop, not inside a running task.

    After calling this, the caller **must** return from the current execution
    context to let the event loop process the scheduled click.

    Args:
        x: Screen X coordinate.
        y: Screen Y coordinate.

    Example:

    .. code-block:: python

        >>> from isaacsim.test.utils.button_utils import deferred_click
        >>> deferred_click(400, 300)
        >>> # MUST return from the current python_server call after this
    """

    async def _click() -> None:
        from omni.kit.ui_test import Vec2, emulate_mouse_move_and_click

        await emulate_mouse_move_and_click(Vec2(x, y))

    asyncio.ensure_future(_click())


def deferred_click_widget(widget: object) -> tuple[float, float]:
    """Schedule a deferred mouse click on the center of a UI widget.

    Combines :func:`get_widget_screen_center` and :func:`deferred_click`.

    Args:
        widget: An ``omni.ui`` widget with screen position attributes.

    Returns:
        Screen coordinates (x, y) where the click was scheduled.

    Example:

    .. code-block:: python

        >>> from isaacsim.test.utils.button_utils import deferred_click_widget
        >>> x, y = deferred_click_widget(load_button)
    """
    pos = get_widget_screen_center(widget)
    deferred_click(*pos)
    return pos


def discover_template_buttons(template: object) -> dict[str, object]:
    """Discover all available buttons on a BaseSampleUITemplate.

    Merges buttons from ``template._buttons`` (world controls like Load World,
    Reset) and ``template.task_ui_elements`` (task-specific actions whose names
    vary per example). Only includes widgets that have screen position attributes.

    Args:
        template: A ``BaseSampleUITemplate`` instance (e.g. from
            ``detail_item.example.ui_hook.__self__``).

    Returns:
        Dict mapping button labels to ``omni.ui`` widget instances.

    Example:

    .. code-block:: python

        >>> from isaacsim.test.utils.button_utils import discover_template_buttons
        >>> buttons = discover_template_buttons(template)
        >>> for name in buttons:
        ...     print(name)
        Load World
        Reset
        Follow Target
    """
    buttons: dict[str, object] = {}
    if hasattr(template, "_buttons") and template._buttons:
        buttons.update(template._buttons)
    if hasattr(template, "task_ui_elements") and template.task_ui_elements:
        for name, widget in template.task_ui_elements.items():
            if hasattr(widget, "screen_position_x"):
                buttons[name] = widget
    return buttons
