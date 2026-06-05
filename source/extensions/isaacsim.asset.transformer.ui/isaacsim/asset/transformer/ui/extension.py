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

"""Extension entry point for the Asset Transformer UI."""

__all__ = ["AssetTransformerUiExtension"]

import carb
import omni.ext
from omni.kit.menu.utils import MenuHelperExtensionFull

from .window import AssetTransformerWindow

BROWSER_MENU_ROOT = "Window"
SETTING_ROOT = "/exts/isaacsim.asset.transformer/"
SETTING_VISIBLE_AFTER_STARTUP = SETTING_ROOT + "visible_after_startup"
EXTENSION_NAME = "Isaac Sim Asset Transformer"


class AssetTransformerUiExtension(omni.ext.IExt, MenuHelperExtensionFull):
    """Kit extension that registers the Asset Transformer window and menu entry."""

    EXTENSION_NAME = "Asset Transformer"
    """The display name for the Asset Transformer extension shown in the UI menu."""

    def on_startup(self, ext_id: str) -> None:
        """Initialize the extension and register the menu entry.

        Args:
            ext_id: Extension identifier provided by the extension manager.
        """
        self.menu_startup(
            lambda: AssetTransformerWindow(),
            AssetTransformerUiExtension.EXTENSION_NAME,
            AssetTransformerUiExtension.EXTENSION_NAME,
            "Tools/Robotics/Asset Editors",
        )

        visible = carb.settings.get_settings().get_as_bool(SETTING_VISIBLE_AFTER_STARTUP)
        if visible:
            self.show_window(None, True, 0)

    def on_shutdown(self) -> None:
        """Clean up resources and unregister the menu entry."""
        self.menu_shutdown()
