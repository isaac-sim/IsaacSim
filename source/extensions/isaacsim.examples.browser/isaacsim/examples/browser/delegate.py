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

"""Delegate implementation for displaying example assets in Isaac Sim's browser detail view."""

import asyncio
from pathlib import Path
from typing import Optional

import carb
import omni.kit.app
import omni.ui as ui
from omni.kit.browser.core import DetailItem, create_drop_helper
from omni.kit.browser.folder.core import FolderDetailDelegate

from .context_menu import ContextMenu
from .model import ExampleBrowserModel

CURRENT_PATH = Path(__file__).parent
ICON_PATH = CURRENT_PATH.parent.parent.parent.parent.joinpath("isaacsim.examples.browser/icons")


class AssetDetailDelegate(FolderDetailDelegate):
    """Delegate to show asset item in detail view.

    Args:
        model: Example browser model
    """

    def __init__(self, model: ExampleBrowserModel) -> None:
        super().__init__(model=model)

        self._dragging_url = None
        self._settings = carb.settings.get_settings()
        self._context_menu: Optional[ContextMenu] = None
        self._action_item: Optional[DetailItem] = None

        self._instanceable_categories = self._settings.get("/exts/isaacsim.examples.browser/instanceable")
        if self._instanceable_categories:
            self._drop_helper = create_drop_helper(
                pickable=True,
                add_outline=True,
                on_drop_accepted_fn=self._on_drop_accepted,
                on_drop_fn=self._on_drop,
            )

    def destroy(self) -> None:
        """Cleans up the delegate by destroying the drop helper and calling parent cleanup."""
        self._drop_helper = None
        super().destroy()

    # def get_thumbnail(self, item) -> str:
    #     """Set default sky thumbnail if thumbnail is None"""
    #     if item.thumbnail is None:
    #         return f"{ICON_PATH}/usd_stage_256.png"
    #     return item.thumbnail

    def on_drag(self, item: DetailItem) -> str:
        """Could be dragged to viewport window.

        Args:
            item: The detail item being dragged.

        Returns:
            The URL of the item being dragged.
        """
        # thumbnail = self.get_thumbnail(item)
        icon_size = 128
        with ui.VStack(width=icon_size):
            # if thumbnail:
            #     ui.Spacer(height=2)
            #     with ui.HStack():
            #         ui.Spacer()
            #         ui.ImageWithProvider(thumbnail, width=icon_size, height=icon_size)
            #         ui.Spacer()
            ui.Label(
                item.name,
                word_wrap=False,
                elided_text=True,
                skip_draw_when_clipped=True,
                alignment=ui.Alignment.TOP,
            )

        self._dragging_url = None
        if self._instanceable_categories:
            # For required categories, need to set instanceable after dropped
            url = item.url
            pos = url.rfind("/")
            if pos > 0:
                url = url[:pos]
            for category in self._instanceable_categories:
                if category in url:
                    self._dragging_url = item.url
                    break
        return item.url

    def _on_drop_accepted(self, url: str) -> bool:
        """Determines if a drop operation should be accepted.

        Args:
            url: The URL of the item being dropped.

        Returns:
            True if the drop should be accepted, False otherwise.
        """
        # Only handle dragging from asset browser
        return url == self._dragging_url

    def _on_drop(
        self, url: str, target: str, viewport_name: str, context_name: str
    ) -> None:  # pylint: disable=useless-return
        """Handles the drop operation by temporarily enabling instanceable references for specific categories.

        Args:
            url: The URL of the dropped item.
            target: The drop target.
            viewport_name: Name of the viewport where the drop occurred.
            context_name: Name of the context.

        Returns:
            None to let the viewport handle asset dropping.
        """
        saved_instanceable = self._settings.get("/persistent/app/stage/instanceableOnCreatingReference")
        if not saved_instanceable and url == self._dragging_url:
            # Enable instanceable for viewport asset drop handler
            self._settings.set_bool("/persistent/app/stage/instanceableOnCreatingReference", True)

            async def __restore_instanceable_flag() -> None:
                # Waiting for viewport asset dropper handler completed
                await omni.kit.app.get_app().next_update_async()
                self._settings.set("/persistent/app/stage/instanceableOnCreatingReference", saved_instanceable)

            asyncio.ensure_future(__restore_instanceable_flag())

        self._dragging_url = None
        # Let viewport do asset dropping
        return None  # noqa: R501

    def on_right_click(self, item: DetailItem) -> None:
        """Show context menu.

        Args:
            item: The detail item that was right-clicked.
        """
        self._action_item = item
        if self._context_menu is None:
            self._context_menu = ContextMenu()

        if self._context_menu:
            self._context_menu.url = self._action_item.url
            self._context_menu.show()
