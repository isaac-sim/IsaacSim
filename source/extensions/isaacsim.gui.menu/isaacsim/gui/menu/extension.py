# SPDX-FileCopyrightText: Copyright (c) 2018-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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

"""Isaac Sim GUI menu extension entry point."""

import gc

import omni.ext
import omni.kit.commands

from .create_menu import CreateMenuExtension
from .edit_menu.edit_menu import EditMenuExtension
from .file_menu.file_menu import FileMenuExtension
from .fixme_menu import FixmeMenuExtension
from .help_menu import HelpMenuExtension
from .hooks_menu import HookMenuHandler
from .layout_menu import LayoutMenuExtension
from .tools_menu import ToolsMenuExtension
from .utilities_menu import UtilitiesMenuExtension
from .window_menu import WindowMenuExtension

# TODO: correct colors


class Extension(omni.ext.IExt):
    """Register and manage Isaac Sim GUI menus."""

    def on_startup(self, ext_id: str) -> None:
        """Initialize menu extensions and menu ordering.

        Args:
            ext_id: Extension identifier provided by the extension manager.
        """
        # kit menus
        self.__hook_menu = HookMenuHandler()
        self.__file_menu = FileMenuExtension(ext_id)
        self.__edit_menu = EditMenuExtension(ext_id)
        self.__create_menu = CreateMenuExtension(ext_id)
        self.__window_menu = WindowMenuExtension(ext_id)
        self.__tools_menu = ToolsMenuExtension(ext_id)
        self.__utilities_menu = UtilitiesMenuExtension(ext_id)
        self.__layout_menu = LayoutMenuExtension(ext_id)
        self.__help_menu = HelpMenuExtension(ext_id)
        self.__fixme_menu = FixmeMenuExtension(ext_id)

        # update order
        menu_self = omni.kit.menu.utils.get_instance()
        menu_defs, menu_order, menu_delegates = menu_self.get_menu_data()
        menu_order["File"] = -10
        menu_order["Edit"] = -9
        menu_order["Create"] = -8
        menu_order["Window"] = -7
        menu_order["Tools"] = 4
        menu_order["Utilities"] = 5
        menu_order["Layouts"] = 6
        menu_order["Help"] = 99

    def on_shutdown(self) -> None:
        """Tear down menu extensions and release resources."""
        # remove_menu_items(self._menu_items, "Create")
        self.__hook_menu.shutdown()
        self.__file_menu.shutdown()
        self.__edit_menu.shutdown()
        self.__create_menu.shutdown()
        self.__window_menu.shutdown()
        self.__tools_menu.shutdown()
        self.__utilities_menu.shutdown()
        self.__layout_menu.shutdown()
        self.__help_menu.shutdown()
        self.__fixme_menu.shutdown()

        del self.__hook_menu
        del self.__file_menu
        del self.__edit_menu
        del self.__create_menu
        del self.__window_menu
        del self.__tools_menu
        del self.__utilities_menu
        del self.__layout_menu
        del self.__help_menu
        del self.__fixme_menu

        gc.collect()


# FIXME
# headings containing submenus don't appear
