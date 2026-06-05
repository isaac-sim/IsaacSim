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

"""Layout management utilities for Isaac Sim application setup.

This module provides functions for loading window layouts, configuring window
docking order, and setting up the property window.
"""

from __future__ import annotations

from collections.abc import Awaitable, Callable
from pathlib import Path

import omni.kit.app
import omni.ui as ui

# Path to layout JSON files
_EXTENSION_FOLDER_NAME = "isaacsim.app.setup"
LAYOUTS_PATH = Path(__file__).parent.parent.parent.parent.parent / _EXTENSION_FOLDER_NAME / "layouts"


async def load_layout(layout_file: str, keep_windows_open: bool = False) -> None:
    """Load a window layout from a JSON file.

    Applies a saved window layout configuration with a short delay to avoid
    conflicts with the main window layout initialization.

    Args:
        layout_file: Path to the layout JSON file to load.
        keep_windows_open: If True, keeps existing windows open when loading layout.

    Example:

        .. code-block:: python

            layout_file = str(LAYOUTS_PATH / "default.json")
            await load_layout(layout_file, keep_windows_open=False)
    """
    from omni.kit.quicklayout import QuickLayout

    # Delay a few frames to avoid conflict with omni.kit.mainwindow layout
    for _ in range(3):
        await omni.kit.app.get_app().next_update_async()

    QuickLayout.load_file(layout_file, keep_windows_open)


async def dock_windows(update_callback: Callable[[], Awaitable[None]]) -> None:
    """Configure default window docking order and focus.

    Sets up the standard Isaac Sim window arrangement with proper dock order
    for Stage, Layer, Console, Content, and Assets windows.

    Args:
        update_callback: Async callback to update the app without signaling ready.

    Example:

        .. code-block:: python

            async def update_callback() -> None:
                await omni.kit.app.get_app().next_update_async()

            await dock_windows(update_callback)
    """
    await update_callback()

    assets = ui.Workspace.get_window("Isaac Sim Assets")
    content = ui.Workspace.get_window("Content")
    stage = ui.Workspace.get_window("Stage")
    layer = ui.Workspace.get_window("Layer")
    console = ui.Workspace.get_window("Console")

    await update_callback()

    # Configure right panel dock order
    if layer:
        layer.dock_order = 1
    if stage:
        stage.dock_order = 0
        stage.focus()

    await update_callback()

    # Configure bottom panel dock order
    if console:
        console.dock_order = 2
    if content:
        content.dock_order = 1
    if assets:
        assets.dock_order = 0
        assets.focus()


async def setup_property_window(update_callback: Callable[[], Awaitable[None]]) -> None:
    """Configure the property window layout scheme.

    Sets up the property window with the appropriate delegate layout for
    various prim types used in Isaac Sim.

    Args:
        update_callback: Async callback to update the app without signaling ready.

    Example:

        .. code-block:: python

            async def update_callback() -> None:
                await omni.kit.app.get_app().next_update_async()

            await setup_property_window(update_callback)
    """
    await update_callback()
    import omni.kit.window.property as property_window_ext

    property_window = property_window_ext.get_window()
    property_window.set_scheme_delegate_layout(
        "Create Layout", ["path_prim", "material_prim", "xformable_prim", "shade_prim", "camera_prim"]
    )
