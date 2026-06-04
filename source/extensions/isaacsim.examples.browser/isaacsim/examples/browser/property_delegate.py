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

"""Property delegates for the Isaac Sim browser example that handle displaying asset properties in different selection states."""

import asyncio
from typing import Optional

import omni.kit.app
import omni.ui as ui
from omni.kit.browser.folder.core import BrowserPropertyDelegate, FileDetailItem


class PropAssetPropertyDelegate(BrowserPropertyDelegate):
    """A delegate to show properties of assets of type Prop."""

    def accepted(self, items: list[FileDetailItem]) -> bool:
        """Determines if this delegate can handle the provided items.

        This delegate accepts only single item selections.

        Args:
            items: List of file detail items to check.

        Returns:
            True if exactly one item is provided, False otherwise.
        """
        return len(items) == 1

    def build_widgets(self, items: list[FileDetailItem]) -> None:
        """Builds the UI widgets to display prop asset properties.

        Creates a vertical stack containing the asset thumbnail (if available) and name label.

        Args:
            items: List of file detail items to build widgets for.
        """
        item = items[0]
        self._container = ui.VStack(height=0, spacing=5)
        with self._container:
            if item.thumbnail:
                self._build_thumbnail(item)
            with ui.HStack():
                ui.Spacer()
                self._name_label = ui.Label(item.name, height=0)
                ui.Spacer()
            item.ui_hook()

    def _build_thumbnail(self, item: FileDetailItem) -> None:
        """Builds thumbnail frame and resizes.

        Creates a thumbnail image widget within a frame that automatically resizes based on the frame width.

        Args:
            item: The file detail item containing the thumbnail to display.
        """
        self._thumbnail_frame = ui.Frame(height=0)
        self._thumbnail_frame.set_computed_content_size_changed_fn(self._on_thumbnail_frame_size_changed)

        with self._thumbnail_frame:
            with ui.HStack():
                ui.Spacer(width=4)
                with ui.ZStack():
                    ui.Rectangle()
                    self._thumbnail_img = ui.Image(
                        item.thumbnail,
                        fill_policy=ui.FillPolicy.PRESERVE_ASPECT_FIT,
                        alignment=ui.Alignment.CENTER_TOP,
                    )

    def _on_thumbnail_frame_size_changed(self) -> None:
        """Handles thumbnail frame size changes.

        Dynamically adjusts the thumbnail image height to be half of the frame width when the frame size changes.
        """

        # Dynamic change thumbnail size to be half of frame width
        async def __change_thumbnail_size_async() -> None:
            await omni.kit.app.get_app().next_update_async()
            image_size = self._thumbnail_frame.computed_width / 2
            self._thumbnail_img.height = ui.Pixel(image_size)

        asyncio.ensure_future(__change_thumbnail_size_async())


class EmptyPropertyDelegate(BrowserPropertyDelegate):
    """A delegate to show when no asset is selected."""

    def accepted(self, items: Optional[FileDetailItem]) -> bool:
        """BrowserPropertyDelegate method override.

        Determines if this delegate can handle the given items. Returns True when no items are selected.

        Args:
            items: File detail items to check for acceptance.

        Returns:
            True if no items are provided, False otherwise.
        """
        return len(items) == 0

    def build_widgets(self, items: Optional[FileDetailItem]) -> None:
        """BrowserPropertyDelegate method override.

        Builds the UI widgets to display when no asset is selected.

        Args:
            items: File detail items (expected to be empty for this delegate).
        """
        ui.Label("Please Select a Isaac Example!", alignment=ui.Alignment.CENTER)


class MultiPropertyDelegate(BrowserPropertyDelegate):
    """A delegate to show when multiple items are selected."""

    def accepted(self, items: list[FileDetailItem]) -> bool:
        """Determines if this delegate can handle the selected file items.

        Args:
            items: List of file detail items to evaluate.

        Returns:
            True if multiple items are selected, False otherwise.
        """
        return len(items) > 1

    def build_widgets(self, items: list[FileDetailItem]) -> None:
        """Builds the UI widgets to display when multiple items are selected.

        Args:
            items: List of file detail items to display information for.
        """
        label_text = f"Multiple Isaac Assets Selected [{len(items)}]"
        ui.Label(label_text, alignment=ui.Alignment.CENTER)
