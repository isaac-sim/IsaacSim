# SPDX-FileCopyrightText: Copyright (c) 2023-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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

"""Base classes for creating wrappers around Omni UI widgets to provide simplified interfaces for specific widget types."""

import omni.ui as ui


class UIWidgetWrapper:
    """Base class for creating wrappers around any subclass of omni.ui.Widget in order to provide an easy interface.

    for creating and managing specific types of widgets such as state buttons or file pickers.

    Args:
        container_frame: The Omni UI frame that contains the widget.
    """

    def __init__(self, container_frame: ui.Frame) -> None:
        self._container_frame = container_frame

    @property
    def container_frame(self) -> ui.Frame:
        """The container frame that holds the widget.

        Returns:
            The UI frame containing this widget.
        """
        return self._container_frame

    @property
    def enabled(self) -> bool:
        """Whether the widget is enabled for user interaction.

        Returns:
            True if the widget is enabled, False otherwise.
        """
        return self.container_frame.enabled

    @enabled.setter
    def enabled(self, value: bool) -> None:
        self.container_frame.enabled = value

    @property
    def visible(self) -> bool:
        """Whether the widget is visible in the UI.

        Returns:
            True if the widget is visible, False otherwise.
        """
        return self.container_frame.visible

    @visible.setter
    def visible(self, value: bool) -> None:
        self.container_frame.visible = value

    def cleanup(self) -> None:
        """Perform any necessary cleanup."""
