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

"""Isaac Sim content browser collection implementation for browsing Isaac Sim asset folders."""

from datetime import datetime
from pathlib import Path
from typing import Optional

import carb.settings
import omni.client
import omni.ui as ui
from omni.kit.widget.filebrowser import FileBrowserItemFields, NucleusItem
from omni.kit.window.filepicker import AddNewItem, CollectionItem

CURRENT_PATH = Path(__file__).parent.absolute()
ICON_PATH = CURRENT_PATH.parent.parent.parent.parent.joinpath("icons/")
SETTING_FOLDER = "/exts/isaacsim.gui.content_browser/folders"
SETTING_ASSET_ROOT = "/persistent/isaac/asset_root/default"


class IsaacConnectionItem(NucleusItem):
    """An item for an Isaac connection.

    Sub-classed from :obj:`NucleusItem`.

    Args:
        name: Name of the item.
        path: Path of the item.
    """

    def __init__(self, name: str, path: str) -> None:
        access = omni.client.AccessFlags.READ
        fields = FileBrowserItemFields(name, datetime.now(), 0, access)
        super().__init__(path, fields, is_folder=True)
        self._models = (ui.SimpleStringModel(name), datetime.now(), ui.SimpleStringModel(""))
        self.icon = f"{ICON_PATH}/folder.svg"


class IsaacCollection(CollectionItem):
    """A collection item for Isaac Sim assets in the content browser.

    This class provides access to Isaac Sim asset folders through the file browser interface. It automatically
    detects the protocol (Omniverse or HTTPS) based on the default asset root configuration and populates
    the collection with configured folders from settings. The collection appears as "Isaac Sim" in the
    content browser with a cloud icon and allows users to browse Isaac Sim asset directories.

    The collection is read-only and does not support adding new connections. Asset folders are loaded
    asynchronously from the application settings and displayed as browsable items in the file browser.
    """

    def __init__(self) -> None:
        settings = carb.settings.get_settings()
        self._asset_root = settings.get_as_string(SETTING_ASSET_ROOT).rstrip("/")

        protocol = ""
        if self._asset_root.startswith("omniverse://"):
            protocol = "omniverse"
        elif self._asset_root.startswith("https://"):
            protocol = "https"
        super().__init__(
            identifier="Isaac Sim",
            title="Isaac Sim",
            protocol=protocol,
            icon=f"{ICON_PATH}/cloud.svg",
            access=omni.client.AccessFlags.READ,
            populated=False,
            order=5,
        )
        self._folders = settings.get(SETTING_FOLDER)

    def create_add_new_item(self) -> Optional[AddNewItem]:
        """Creates an "Add New Connection" item for the Isaac Sim collection.

        Returns:
            None to hide the "Add New Connection" option from the collection.
        """
        # Do not show "Add New Connection ..."
        return None

    def create_child_item(self, name: str, path: str, is_folder: bool = True) -> Optional[IsaacConnectionItem]:
        """Creates a child connection item for the Isaac Sim collection.

        Args:
            name: Name of the connection item.
            path: Path of the connection item.
            is_folder: Whether the item represents a folder.

        Returns:
            A new IsaacConnectionItem instance.
        """
        return IsaacConnectionItem(name, path)

    def _resolve_path(self, path: str) -> str:
        """Resolve a path, prepending the asset root if it is a relative suffix."""
        if path.startswith(("http://", "https://", "omniverse://")):
            return path
        return self._asset_root + path

    async def populate_children_async(self) -> None:
        """Populates the Isaac Sim collection with configured folder connections.

        Adds child items for each folder configured in the Isaac Sim asset root settings,
        extracting the folder name from the URL path. Relative paths are resolved against
        ``persistent.isaac.asset_root.default``.
        """
        if self._folders:
            for folder in self._folders:
                url = self._resolve_path(folder)
                parts = url.rstrip("/").split("/")
                if parts:
                    name = parts[-1]
                    self.add_path(name, url)
