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

"""Utilities menu layout for Isaac Sim."""

from __future__ import annotations

import omni.kit.actions.core
import omni.kit.menu.utils
from omni.kit.menu.utils import MenuItemDescription, MenuLayout, refresh_menu_items

from .asset_check import AssetCheck


class UtilitiesMenuExtension:
    """Build and manage the Utilities menu.

    Args:
        ext_id: Extension identifier provided by the extension manager.
    """

    def __init__(self, ext_id: str) -> None:
        self._ext_id = ext_id
        self._asset_check: AssetCheck | None = AssetCheck(on_visibility_changed=self._on_asset_check_visibility_changed)

        self._menu_placeholder = [MenuItemDescription(name="placeholder", show_fn=lambda: False)]
        omni.kit.menu.utils.add_menu_items(self._menu_placeholder, "Utilities")

        self._action_id = "check_default_assets_root_path"
        action_registry = omni.kit.actions.core.get_action_registry()
        action_registry.register_action(
            ext_id,
            self._action_id,
            self._asset_check.check_assets,
            description="Check Default Assets Root Path",
        )

        self._menu_items = [
            MenuItemDescription(
                name="Check Default Assets Root Path",
                onclick_action=(ext_id, self._action_id),
                ticked=True,
                ticked_fn=self._is_visible,
            ),
        ]
        omni.kit.menu.utils.add_menu_items(self._menu_items, "Utilities")

        self.__menu_layout = [
            MenuLayout.Menu(
                "Utilities",
                [
                    MenuLayout.Seperator("Profilers & Debuggers"),
                    MenuLayout.Item("OmniGraph Tool Kit", source="Window/Visual Scripting/Toolkit"),
                    MenuLayout.Item("Physics Debugger", source="Window/Physics/Debug"),
                    MenuLayout.Item("Profiler", source="Window/Profiler"),
                    MenuLayout.Item("Statistics"),
                    MenuLayout.Seperator(),
                    MenuLayout.Item("Check Default Assets Root Path"),
                    MenuLayout.Item("Generate Extension Templates"),
                    MenuLayout.Item("Registered Actions", source="Window/Actions"),
                ],
            )
        ]
        omni.kit.menu.utils.add_layout(self.__menu_layout)

    def _is_visible(self) -> bool:
        """Return True if the asset check window is visible.

        Returns:
            bool: Whether the asset check window is visible.
        """
        return self._asset_check.is_visible() if self._asset_check else False

    def _on_asset_check_visibility_changed(self) -> None:
        """Notify the visibility changed callback."""
        refresh_menu_items("Utilities")

    def shutdown(self) -> None:
        """Remove menu layouts and placeholders.

        Example:
            .. code-block:: python

                menu = UtilitiesMenuExtension("ext.id")
                menu.shutdown()
        """
        omni.kit.menu.utils.remove_layout(self.__menu_layout)
        omni.kit.menu.utils.remove_menu_items(self._menu_items, "Utilities")
        omni.kit.menu.utils.remove_menu_items(self._menu_placeholder, "Utilities")
        action_registry = omni.kit.actions.core.get_action_registry()
        action_registry.deregister_action(self._ext_id, self._action_id)
        if self._asset_check is not None:
            self._asset_check.destroy()
            self._asset_check = None
