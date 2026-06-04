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

"""Temporary FixMe menu for legacy or placeholder entries."""

import omni.kit.menu.utils
import omni.ui as ui
from omni.kit.menu.utils import MenuAlignment, MenuItemDescription, MenuLayout


class FixmeMenuExtension:
    """Build and manage the FixMe menu.

    Args:
        ext_id: Extension identifier provided by the extension manager.
    """

    class MenuDelegate(ui.MenuDelegate):
        """Menu delegate used to hide FixMe entries."""

        def get_menu_alignment(self) -> MenuAlignment:
            """Return the menu alignment for the FixMe delegate.

            Returns:
                The alignment to use for the menu.

            Example:
                .. code-block:: python

                    alignment = FixmeMenuExtension.MenuDelegate().get_menu_alignment()
            """
            return MenuAlignment.RIGHT

        # override build fn to not show menu item...
        def build_item(self, item: ui.MenuHelper) -> None:
            """Override item rendering to hide the menu item.

            Args:
                item: Menu helper instance provided by the UI framework.

            Example:
                .. code-block:: python

                    delegate = FixmeMenuExtension.MenuDelegate()
                    delegate.build_item(item)
            """

    def __init__(self, ext_id: str) -> None:
        self._menu_placeholder = [MenuItemDescription(name="FixMe!!!", show_fn=lambda: False)]
        omni.kit.menu.utils.add_menu_items(self._menu_placeholder, "FixMe", delegate=FixmeMenuExtension.MenuDelegate())

        self.__menu_layout = [
            MenuLayout.Menu(
                "FixMe",
                [
                    MenuLayout.Item("Replicators", source="Replicator"),
                    MenuLayout.SubMenu(
                        "Replicators",
                        [
                            # have to remove hidden menu items too...
                            MenuLayout.Item("Capture On Play", source="Replicator/Capture On Play"),
                            MenuLayout.Item(name="Stop", source="Replicator/Stop"),
                            MenuLayout.Item(name="Resume", source="Replicator/Resume"),
                            MenuLayout.Item(name="Pause", source="Replicator/Pause"),
                            MenuLayout.Item(name="Starting...", source="Replicator/Starting..."),
                            MenuLayout.Item(name="Starting...", source="Replicator/Starting..."),
                            MenuLayout.Item(name="Stopping...", source="Replicator/Stopping..."),
                            MenuLayout.Item(name="Stopping...", source="Replicator/Stopping..."),
                            MenuLayout.Item(name="Capture On Play", source="Replicator/Capture On Play"),
                        ],
                    ),
                    MenuLayout.SubMenu(
                        "Profiler",
                        [
                            MenuLayout.Item("Start or Stop", source="Profiler/Start\\Stop"),
                            MenuLayout.Item("Profile Startup (Restart)", source="Profiler/Profile Startup (Restart)"),
                        ],
                    ),
                    MenuLayout.SubMenu(
                        "Help",
                        [
                            MenuLayout.Item("Discover Kit SDK", source="Help/Discover Kit SDK"),
                            MenuLayout.Item("Developers Manual", source="Help/Developers Manual"),
                        ],
                    ),
                    MenuLayout.SubMenu(
                        "Layout",
                        [
                            MenuLayout.Item("Quick Save", source="Window_Layout/Quick Save", remove=True),
                            MenuLayout.Item("Quick Load", source="Window_Layout/Quick Load", remove=True),
                        ],
                    ),
                ],
            ),
        ]
        omni.kit.menu.utils.add_layout(self.__menu_layout)

    def shutdown(self) -> None:
        """Remove menu layouts and placeholders.

        Example:
            .. code-block:: python

                menu = FixmeMenuExtension("ext.id")
                menu.shutdown()
        """
        omni.kit.menu.utils.remove_layout(self.__menu_layout)
        omni.kit.menu.utils.remove_menu_items(self._menu_placeholder, "FixMe")
