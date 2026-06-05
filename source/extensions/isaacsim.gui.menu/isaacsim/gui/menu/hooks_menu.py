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

"""Hook utilities for menu post-processing."""

import omni.kit.menu.utils
from omni.kit.menu.utils import MenuItemDescription


class HookMenuHandler:
    """Register a hook to adjust menu item appearance."""

    def __init__(self) -> None:
        omni.kit.menu.utils.add_hook(self.__hook_func)

    def shutdown(self) -> None:
        """Remove the menu hook.

        Example:
            .. code-block:: python

                handler = HookMenuHandler()
                handler.shutdown()
        """
        omni.kit.menu.utils.remove_hook(self.__hook_func)

    def __hook_func(self, merged_menu: dict[str, list[MenuItemDescription]]) -> None:
        """Strip glyphs from non-Create menus.

        Args:
            merged_menu: Mapping of menu names to menu item lists.
        """
        for name in merged_menu:
            for i in merged_menu[name].copy():
                # remove all glyphs in all menus expect create
                if name != "Create":
                    i.glyph = None

                # HACK to show all menu items
                # i.show_fn = None
