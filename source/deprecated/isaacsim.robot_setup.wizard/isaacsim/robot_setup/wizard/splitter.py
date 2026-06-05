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

"""A UI widget module that provides a resizable horizontal split-pane layout with draggable divider functionality."""

from collections.abc import Callable
from typing import Any

import omni.ui as ui


class Splitter:
    """A UI widget that creates a resizable horizontal split-pane layout.

    This class provides a two-panel layout with a draggable splitter in the middle that allows users to adjust
    the width of the left panel. The right panel automatically takes up the remaining space. Each panel's
    content is defined by callback functions that are executed when the UI is built.

    The splitter is positioned at 300 pixels from the left by default and can be dragged horizontally, with
    a minimum width constraint of 230 pixels for the left panel.

    Args:
        build_left_fn: Callback function to build the content of the left panel.
        build_right_fn: Callback function to build the content of the right panel.
    """

    def __init__(self, build_left_fn: Callable[[], None], build_right_fn: Callable[[], None]) -> None:
        self._frame = ui.Frame()
        self._frame.set_build_fn(self.on_build)
        self.__build_left_fn = build_left_fn
        self.__build_right_fn = build_right_fn

    def destroy(self) -> None:
        """Cleans up the splitter by clearing build functions and frame references."""
        self.__build_right_fn = None
        self.__build_left_fn = None
        self._frame = None

    def on_build(self) -> None:
        """Builds the splitter UI with left and right panels separated by a draggable divider."""
        with ui.HStack():
            with ui.ZStack(width=0):
                # Left pannel
                with ui.Frame():
                    self.__build_left_fn()

                # Draggable splitter
                placer = ui.Placer(drag_axis=ui.Axis.X, offset_x=300.0, draggable=True)

                def left_moved(x: Any) -> None:
                    placer.offset_x = max(230.0, x.value)

                placer.set_offset_x_changed_fn(left_moved)
                with placer:
                    with ui.ZStack(width=10):
                        ui.Rectangle(name="splitter")
                        ui.Rectangle(name="splitter_highlight", width=4)

            # Right pannel
            with ui.Frame():
                self.__build_right_fn()
