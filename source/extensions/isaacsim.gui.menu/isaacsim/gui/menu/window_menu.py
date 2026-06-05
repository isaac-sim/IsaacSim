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

"""Window menu layout for Isaac Sim."""

import omni.kit.menu.utils
from omni.kit.menu.utils import MenuItemDescription, MenuLayout, add_menu_items


class WindowMenuExtension:
    """Build and manage the Window menu.

    Args:
        ext_id: Extension identifier provided by the extension manager.
    """

    def __init__(self, ext_id: str) -> None:
        self.__menu_layout = [
            MenuLayout.Menu(
                "Window",
                [
                    MenuLayout.SubMenu(
                        "Browsers",
                        [
                            MenuLayout.Item("Content", source="Window/Content"),
                            MenuLayout.Item("Isaac Sim Assets"),
                            MenuLayout.Item("Materials"),
                            MenuLayout.Item("NVIDIA Assets", source="Window/Browsers/Assets"),
                            MenuLayout.Item("SimReady Explorer"),
                        ],
                    ),
                    MenuLayout.SubMenu(
                        "Examples",
                        [
                            MenuLayout.Item("Physics Examples", source="Window/Physics/Demo Scenes"),
                            MenuLayout.Item("Robotics Examples"),
                            MenuLayout.Item("Warp Sample Scenes", source="Window/Warp/Sample Scenes"),
                        ],
                    ),
                    MenuLayout.SubMenu(
                        "Graph Editors",
                        [
                            MenuLayout.Item("Action Graph", source="Window/Visual Scripting/Action Graph"),
                            MenuLayout.Item("Animation Graph"),  # ??
                            MenuLayout.Item("Generic Graph", source="Window/Visual Scripting/Generic Graph"),
                            MenuLayout.Item("MDL Material Graph"),  # ??
                        ],
                    ),
                    MenuLayout.SubMenu(
                        "Viewports",
                        [
                            MenuLayout.Item("Viewport 1"),
                            MenuLayout.Item("Viewport 2"),
                        ],
                    ),
                    MenuLayout.Seperator(),
                    MenuLayout.Item("Collection"),
                    MenuLayout.Item("Commands"),
                    MenuLayout.Item("Console"),
                    MenuLayout.Item("Hotkeys"),
                    MenuLayout.Item("Layer"),
                    MenuLayout.Item("Physics Stage Settings", source="Window/Physics/Settings"),
                    MenuLayout.Item("Property"),
                    MenuLayout.Item("Render Settings"),
                    MenuLayout.Item("Script Editor"),
                    MenuLayout.Item("Simulation Settings"),
                    MenuLayout.Item("Stage"),
                    MenuLayout.Sort(
                        exclude_items=["Browsers", "Examples", "Graph Editors", "Viewports", "Extensions"],
                        sort_submenus=True,
                    ),
                ],
            ),
        ]
        omni.kit.menu.utils.add_layout(self.__menu_layout)

        simulation_setting_window = MenuItemDescription(
            name="Simulation Output Settings",
            onclick_action=("omni.physx.ui", "show_simulation_output_settings_window"),
        )

        add_menu_items([simulation_setting_window], "Window")

    def shutdown(self) -> None:
        """Remove menu layouts.

        Example:
            .. code-block:: python

                menu = WindowMenuExtension("ext.id")
                menu.shutdown()
        """
        omni.kit.menu.utils.remove_layout(self.__menu_layout)
