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

"""UI components for the gain tuner frame widget interface."""

from collections.abc import Callable

import omni.ui as ui
from isaacsim.gui.components.element_wrappers import CollapsableFrame
from isaacsim.gui.components.ui_utils import get_style, on_copy_to_clipboard

from .style import get_style as get_custom_style

LABEL_WIDTH = 90


class CustomCollapsableFrame(CollapsableFrame):
    """A custom collapsable frame widget with optional copy functionality.

    Extends the base CollapsableFrame to provide additional features including an optional copy button
    in the header that allows users to copy content to the clipboard. The frame supports custom styling
    and maintains all the functionality of the parent CollapsableFrame while adding copy capabilities.

    Args:
        *args: Variable length argument list passed to the parent CollapsableFrame.
        **kwargs: Additional keyword arguments. Includes show_copy_button to display a copy button in
            the frame header, and other arguments passed to the parent CollapsableFrame.
    """

    def __init__(self, *args: object, **kwargs: object) -> None:
        self._show_copy_button = kwargs.get("show_copy_button", False)
        kwargs.pop("show_copy_button", None)
        self._copy_content = None
        super().__init__(*args, **kwargs)

    def set_copy_content(self, copy_content: any) -> None:
        """Sets the content to be copied when the copy button is clicked.

        Args:
            copy_content: The content to copy to clipboard when the copy button is pressed.
        """
        self._copy_content = copy_content

    def _build_header(self, collapsed: bool, title: str) -> None:
        """Builds the header UI for the collapsable frame.

        Creates a horizontal stack containing a triangle indicator, title label, and optional copy button.
        The triangle orientation changes based on the collapsed state.

        Args:
            collapsed: Whether the frame is currently collapsed.
            title: The title text to display in the header.
        """
        with ui.HStack(height=34, style=get_custom_style()):
            ui.Spacer(width=4)
            with ui.VStack(width=10):
                ui.Spacer()
                if collapsed:
                    triangle = ui.Triangle(height=9, width=7)
                    triangle.alignment = ui.Alignment.RIGHT_CENTER
                else:
                    triangle = ui.Triangle(height=7, width=9)
                    triangle.alignment = ui.Alignment.CENTER_BOTTOM
                ui.Spacer()
            ui.Spacer(width=4)
            ui.Label(title, name="robot_header", width=0)
            ui.Spacer()
            if self._show_copy_button:
                with ui.VStack(width=0):
                    ui.Spacer(height=6)
                    ui.Image(
                        name="copy",
                        height=22,
                        width=22,
                        mouse_pressed_fn=lambda x, y, b, a: on_copy_to_clipboard(self._copy_content),
                    )

    def _create_frame(
        self, title: str, collapsed: bool, enabled: bool, visible: bool, build_fn: Callable
    ) -> ui.CollapsableFrame:
        """Creates and configures the UI CollapsableFrame widget.

        Args:
            title: The title for the frame.
            collapsed: Whether the frame should start collapsed.
            enabled: Whether the frame is enabled for interaction.
            visible: Whether the frame is visible.
            build_fn: Callback function to build the frame content.

        Returns:
            The configured CollapsableFrame widget.
        """
        frame = ui.CollapsableFrame(
            title=title,
            name=title,
            height=0,
            collapsed=collapsed,
            visible=visible,
            enabled=enabled,
            build_fn=build_fn,
            build_header_fn=self._build_header,
            style=get_style(),
            style_type_name_override="CollapsableFrame",
            horizontal_scrollbar_policy=ui.ScrollBarPolicy.SCROLLBAR_AS_NEEDED,
            vertical_scrollbar_policy=ui.ScrollBarPolicy.SCROLLBAR_ALWAYS_ON,
        )

        return frame
