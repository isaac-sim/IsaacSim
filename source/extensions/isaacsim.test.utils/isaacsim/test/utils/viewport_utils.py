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

"""Viewport utility functions for UI automation and testing."""

from __future__ import annotations

import numpy as np
import omni.kit.viewport.utility

__all__ = ["project_world_to_screen"]


def project_world_to_screen(
    position: tuple[float, float, float],
    viewport: object | None = None,
) -> tuple[float, float]:
    """Project a 3D world position to app-window screen coordinates.

    Convert a world-space (x, y, z) point to the pixel coordinates in the
    application window, accounting for the viewport's view and projection
    matrices, render resolution, viewport window offset, and toolbar height.

    The returned coordinates match the coordinate system used by
    ``omni.kit.ui_test.emulate_mouse_move`` and full-app (swapchain)
    screenshot pixels.

    Args:
        position: World-space position as (x, y, z).
        viewport: Viewport API instance. If ``None``, the active viewport is used.

    Returns:
        Screen coordinates as (x, y) in app-window pixel space.

    Example:

    .. code-block:: python

        >>> from isaacsim.test.utils.viewport_utils import project_world_to_screen
        >>> screen_x, screen_y = project_world_to_screen((0.0, 0.0, 0.5))
    """
    if viewport is None:
        viewport = omni.kit.viewport.utility.get_active_viewport()

    vp_w, vp_h = viewport.resolution

    # Viewport matrices are column-major flat arrays; transpose for row-major numpy
    view = np.array(viewport.view).reshape(4, 4).T
    proj = np.array(viewport.projection).reshape(4, 4).T

    # World point to clip space
    wp = np.array([position[0], position[1], position[2], 1.0])
    cp = proj @ view @ wp

    # Clip to NDC
    nx = cp[0] / cp[3]
    ny = cp[1] / cp[3]

    # NDC to render-resolution pixels
    rpx = (nx + 1.0) / 2.0 * float(vp_w)
    rpy = (1.0 - ny) / 2.0 * float(vp_h)

    # Render pixels to app-window pixels
    vwin = omni.kit.viewport.utility.get_active_viewport_window()
    toolbar_height = 26  # viewport toolbar height in pixels
    vwx = vwin.position_x
    vwy = vwin.position_y + toolbar_height
    vww = vwin.width
    vwh = vwin.height - toolbar_height

    sx = vww / float(vp_w)
    sy = vwh / float(vp_h)

    return vwx + rpx * sx, vwy + rpy * sy
