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

"""Help menu registration and documentation helpers."""

from functools import partial

import omni.ext
import omni.kit.actions.core
import omni.kit.app
import omni.kit.menu.utils
from omni.kit.menu.utils import MenuItemDescription, MenuLayout, add_menu_items


class HelpMenuExtension:
    """Build and manage the Help menu for Isaac Sim.

    Args:
        ext_id: Extension identifier provided by the extension manager.
    """

    def __init__(self, ext_id: str) -> None:
        self._ext_id = ext_id
        self._ext_name = omni.ext.get_extension_name(ext_id)
        self._registered_actions = []

        help_items = [
            MenuLayout.Seperator("Examples"),
            MenuLayout.Item("Physics Examples"),
            MenuLayout.Item("Robotics Examples"),
            MenuLayout.Seperator("Isaac Sim Reference"),
            MenuLayout.Item("About"),
            MenuLayout.Item("Online Guide", source="Isaac Sim Online Guide"),
            MenuLayout.Item("Online Forums", source="Isaac Sim Online Forums"),
            MenuLayout.Item("Scripting Manual", source="Isaac Sim Scripting Manual"),
            MenuLayout.Seperator("Omniverse Reference"),
            MenuLayout.Item("Kit Programming Manual"),
            MenuLayout.Item("Omni UI Docs"),
            MenuLayout.Seperator("Physics Reference"),
            MenuLayout.Item("Physics Programming Manual"),
            MenuLayout.Seperator("Warp Reference"),
            MenuLayout.Item("Warp Getting Started"),
            MenuLayout.Item("Warp Documentation"),
            MenuLayout.Seperator("USD"),
            MenuLayout.Item("OpenUSD Reference Guide"),
            MenuLayout.Seperator(),
        ]

        self.__menu_layout = [MenuLayout.Menu("Help", help_items)]
        omni.kit.menu.utils.add_layout(self.__menu_layout)

        ## hack to have examples in two places
        robotics_demo = MenuItemDescription(
            name="Robotics Examples",
            onclick_action=(
                "isaacsim.examples.browser",
                "open_isaac_sim_examples_browser",
            ),
        )
        physics_demo = MenuItemDescription(
            name="Physics Examples",
            onclick_action=("omni.physxuicommon.windowmenuitem", "WindowMenuItemAction_PhysicsDemoScenes"),
        )

        demo_items = [robotics_demo, physics_demo]

        add_menu_items(demo_items, "Help")

        ISAAC_DOCS_URL = "https://docs.isaacsim.omniverse.nvidia.com/latest"
        REFERENCE_GUIDE_URL = ISAAC_DOCS_URL + "/index.html"
        MANUAL_URL = ISAAC_DOCS_URL + "/py/index.html"
        FORUMS_URL = "https://forums.developer.nvidia.com/c/omniverse/simulation/69"
        KIT_MANUAL_URL = "https://docs.omniverse.nvidia.com/py/kit/index.html"
        OPENUSD_REFERENCE_URL = "https://openusd.org/release/index.html"
        WARP_GETTING_STARTED_URL = "https://docs.omniverse.nvidia.com/extensions/latest/ext_warp.html"
        WARP_DOCUMENTATION_URL = "https://nvidia.github.io/warp/"

        action_registry = omni.kit.actions.core.get_action_registry()

        # Register actions for help menu items
        action_registry.register_action(
            self._ext_name,
            "open_isaac_online_guide",
            partial(self.open_ref_url, REFERENCE_GUIDE_URL),
            display_name="Open Isaac Sim Online Guide",
            description="Open the Isaac Sim online documentation",
        )
        self._registered_actions.append("open_isaac_online_guide")

        action_registry.register_action(
            self._ext_name,
            "open_isaac_scripting_manual",
            partial(self.open_ref_url, MANUAL_URL),
            display_name="Open Isaac Sim Scripting Manual",
            description="Open the Isaac Sim Python scripting manual",
        )
        self._registered_actions.append("open_isaac_scripting_manual")

        action_registry.register_action(
            self._ext_name,
            "open_isaac_forums",
            partial(self.open_ref_url, FORUMS_URL),
            display_name="Open Isaac Sim Online Forums",
            description="Open the Isaac Sim online forums",
        )
        self._registered_actions.append("open_isaac_forums")

        action_registry.register_action(
            self._ext_name,
            "open_kit_manual",
            partial(self.open_ref_url, KIT_MANUAL_URL),
            display_name="Open Kit Programming Manual",
            description="Open the Kit programming manual",
        )
        self._registered_actions.append("open_kit_manual")

        action_registry.register_action(
            self._ext_name,
            "open_physics_manual",
            self._open_physics_manual,
            display_name="Open Physics Programming Manual",
            description="Open the physics programming manual",
        )
        self._registered_actions.append("open_physics_manual")

        action_registry.register_action(
            self._ext_name,
            "open_openusd_reference",
            partial(self.open_ref_url, OPENUSD_REFERENCE_URL),
            display_name="Open OpenUSD Reference Guide",
            description="Open the OpenUSD reference documentation",
        )
        self._registered_actions.append("open_openusd_reference")

        action_registry.register_action(
            self._ext_name,
            "open_warp_getting_started",
            partial(self.open_ref_url, WARP_GETTING_STARTED_URL),
            display_name="Open Warp Getting Started",
            description="Open the Warp getting started documentation",
        )
        self._registered_actions.append("open_warp_getting_started")

        action_registry.register_action(
            self._ext_name,
            "open_warp_documentation",
            partial(self.open_ref_url, WARP_DOCUMENTATION_URL),
            display_name="Open Warp Documentation",
            description="Open the Warp reference documentation",
        )
        self._registered_actions.append("open_warp_documentation")

        reference_guide_menu_item = MenuItemDescription(
            name="Isaac Sim Online Guide", onclick_action=(self._ext_name, "open_isaac_online_guide")
        )
        scripting_manual_menu_item = MenuItemDescription(
            name="Isaac Sim Scripting Manual", onclick_action=(self._ext_name, "open_isaac_scripting_manual")
        )
        forums_menu_item = MenuItemDescription(
            name="Isaac Sim Online Forums", onclick_action=(self._ext_name, "open_isaac_forums")
        )
        kit_manual_menu_item = MenuItemDescription(
            name="Kit Programming Manual", onclick_action=(self._ext_name, "open_kit_manual")
        )

        physics_menu_item = MenuItemDescription(
            name="Physics Programming Manual", onclick_action=(self._ext_name, "open_physics_manual")
        )
        openusd_reference_menu_item = MenuItemDescription(
            name="OpenUSD Reference Guide", onclick_action=(self._ext_name, "open_openusd_reference")
        )
        warp_getting_started_menu_item = MenuItemDescription(
            name="Warp Getting Started", onclick_action=(self._ext_name, "open_warp_getting_started")
        )
        warp_documentation_menu_item = MenuItemDescription(
            name="Warp Documentation", onclick_action=(self._ext_name, "open_warp_documentation")
        )

        add_menu_items(
            [
                physics_menu_item,
                kit_manual_menu_item,
                reference_guide_menu_item,
                scripting_manual_menu_item,
                forums_menu_item,
                openusd_reference_menu_item,
                warp_getting_started_menu_item,
                warp_documentation_menu_item,
            ],
            "Help",
        )

    def shutdown(self) -> None:
        """Remove menu layouts and deregister actions.

        Example:
            .. code-block:: python

                menu = HelpMenuExtension("ext.id")
                menu.shutdown()
        """
        omni.kit.menu.utils.remove_layout(self.__menu_layout)

        # Deregister all actions
        action_registry = omni.kit.actions.core.get_action_registry()
        for action_id in self._registered_actions:
            action_registry.deregister_action(self._ext_name, action_id)
        self._registered_actions = []

    def _open_physics_manual(self) -> None:
        """Open the physics programming manual URL."""
        self.open_ref_url(resolve_physics_ref_url())

    def open_ref_url(self, url: str) -> None:
        """Open a documentation URL using the system browser.

        Args:
            url: URL to open.

        Example:
            .. code-block:: python

                HelpMenuExtension("ext.id").open_ref_url("https://docs.omniverse.nvidia.com/")
        """
        import platform
        import subprocess
        import webbrowser

        if platform.system().lower() == "windows":
            webbrowser.open(url)
        else:
            # use native system level open, handles snap based browsers better
            subprocess.Popen(["xdg-open", url])


def resolve_physics_ref_url() -> str:
    """Resolve the correct physics documentation URL for the current version.

    Returns:
        The resolved physics documentation URL.

    Example:
        .. code-block:: python

            url = resolve_physics_ref_url()
    """
    # get URL by physx extensions version
    try:
        manager = omni.kit.app.get_app().get_extension_manager()
        ext_id = manager.get_extension_id_by_module("omni.physx")
        ext_version = manager.get_extension_dict(ext_id)["package"]["version"]
        version = ".".join(ext_version.split(".")[:2])
        url = f"https://docs.omniverse.nvidia.com/kit/docs/omni_physics/{version}/index.html"

        # check if website exists
        import requests  # type: ignore[import-untyped]

        response = requests.head(url, timeout=5)
        if response.status_code > 400:
            raise Exception()
    except Exception:
        url = "https://docs.omniverse.nvidia.com/kit/docs/omni_physics/latest/index.html"

    return url
