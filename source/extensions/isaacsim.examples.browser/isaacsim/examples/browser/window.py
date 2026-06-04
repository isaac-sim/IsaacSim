# SPDX-FileCopyrightText: Copyright (c) 2022-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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

"""Module providing a specialized browser window for displaying Isaac Sim robotics examples with custom widget components."""

import os

import carb.settings
import omni.ui as ui
from omni.kit.browser.folder.core import TreeFolderBrowserWidgetEx

from .delegate import AssetDetailDelegate
from .model import ExampleBrowserModel
from .property_delegate import EmptyPropertyDelegate, MultiPropertyDelegate, PropAssetPropertyDelegate
from .style import THUMBNAIL_STYLE


class BrowserWidget(TreeFolderBrowserWidgetEx):
    """A specialized browser widget for displaying and navigating Isaac Sim robotics examples.

    This widget extends the TreeFolderBrowserWidgetEx to provide a customized browsing experience
    for robotics example assets. It manages thumbnail display settings and ensures labels remain
    visible across different thumbnail sizes to maintain usability in the examples browser interface.
    """

    def _on_thumbnail_size_changed(self, thumbnail_size: int) -> None:
        """Handles changes to the thumbnail size in the browser widget.

        Updates the delegate to keep labels visible and refreshes the item display.

        Args:
            thumbnail_size: The new thumbnail size in pixels.
        """
        # to keep the labels visible at all times
        self._delegate.hide_label = False  # thumbnail_size < 64
        self._delegate.item_changed(None, None)


class ExampleBrowserWindow(ui.Window):
    """Represent a window to show Assets.

    Args:
        model: The browser model containing the data to display.
        visible: Whether the window is initially visible.
    """

    WINDOW_TITLE = "Robotics Examples"
    """Window title displayed for the robotics examples browser."""

    def __init__(self, model: ExampleBrowserModel, visible: bool = True) -> None:
        super().__init__(self.WINDOW_TITLE, visible=visible)

        self.frame.set_build_fn(self._build_ui)

        self._browser_model = model
        self._delegate = None
        self._widget = None

        # Dock it to the same space where Stage is docked.
        self.deferred_dock_in("Content")

    def _build_ui(self) -> None:
        """Builds the user interface for the example browser window.

        Creates the main UI layout with a BrowserWidget that displays robotics examples using thumbnail view and
        various property delegates for different asset types.
        """
        preload_folder = os.path.abspath(carb.tokens.get_tokens_interface().resolve("${app}/../predownload"))
        self._delegate = AssetDetailDelegate(self._browser_model)

        with self.frame:
            with ui.VStack(spacing=15):
                self._widget = BrowserWidget(
                    self._browser_model,
                    detail_delegate=self._delegate,
                    predownload_folder=preload_folder,
                    min_thumbnail_size=32,
                    max_thumbnail_size=128,
                    detail_thumbnail_size=64,
                    style=THUMBNAIL_STYLE,
                    property_delegates=[EmptyPropertyDelegate(), PropAssetPropertyDelegate(), MultiPropertyDelegate()],
                )
                # Hand the widget back to the model so folder tiles in the detail view can drive
                # tree navigation when double-clicked.
                self._browser_model.set_widget(self._widget)
