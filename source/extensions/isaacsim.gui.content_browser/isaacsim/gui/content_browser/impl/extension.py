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

"""Extension module for Isaac Sim content browser implementation with enhanced file browsing capabilities."""

import asyncio
import weakref

import carb
import omni.ext
from omni.kit.window.content_browser import get_content_window

from .detail_view import ExtendedFileInfo
from .isaac_collection import IsaacCollection


class Extension(omni.ext.IExt):
    """The Extension class."""

    def on_startup(self, ext_id: str) -> None:
        """Method called when the extension is loaded/enabled.

        Args:
            ext_id: The extension identifier.
        """
        carb.log_info(f"on_startup {ext_id}")
        self._content_browser_ref = weakref.ref(get_content_window(), lambda ref: self.destroy())
        self._add_isim_content()

    def _add_isim_content(self) -> None:
        """Adds Isaac Sim content to the content browser.

        Registers the Isaac collection, expands collections, and populates asset information.
        """
        content_browser = self._content_browser_ref()
        if not content_browser:
            return

        self._isaac_collection = IsaacCollection()
        content_browser.api.register_collection_item(self._isaac_collection)
        self._expand_collections()
        self._populate_asset_info()

    def _populate_asset_info(self) -> None:
        """Populates asset information in the content browser.

        Adds an extended file info detail frame to display additional asset information.
        """
        content_browser = self._content_browser_ref()
        if not content_browser:
            return

        assetFileInfo = ExtendedFileInfo()
        content_browser.api.add_detail_frame_from_controller("File Info", assetFileInfo)

    def _expand_collections(self) -> None:
        """Expands the Isaac Sim collection in the content browser.

        Asynchronously expands the Isaac collection while collapsing other collections after the UI is ready.
        """
        # Only expand Isaac Sim collection after the UI is ready

        async def expand_collections_async() -> None:
            # The collection expand status is set 6 frames later after the window displayed
            for _ in range(7):
                await omni.kit.app.get_app().next_update_async()

            content_browser = self._content_browser_ref()
            if not content_browser:
                return

            view = content_browser.api.view
            if view:
                for collection in view.navigation_model.collection_items:
                    view.filebrowser.set_expanded(
                        collection, expanded=collection == self._isaac_collection, recursive=False
                    )

        asyncio.ensure_future(expand_collections_async())

    def on_shutdown(self) -> None:
        """Method called when the extension is disabled."""
        carb.log_info(f"on_shutdown")
        content_browser = self._content_browser_ref()
        if content_browser:
            content_browser.api.delete_detail_frame("File Info")
            content_browser.api.deregister_collection_item(self._isaac_collection)
