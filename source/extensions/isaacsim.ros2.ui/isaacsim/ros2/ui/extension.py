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

"""Extension that provides ROS 2 shortcuts menu for creating ROS 2-related assets in the stage."""

from __future__ import annotations

from functools import partial
from typing import Any, Optional

import carb
import omni.ext
import omni.kit.actions.core
import omni.usd
from isaacsim.core.rendering_manager import ViewportManager
from isaacsim.gui.components.menu import open_content_browser_to_path
from isaacsim.storage.native.nucleus import get_assets_root_path
from omni.kit.menu.utils import MenuHelperExtensionFull, MenuItemDescription, add_menu_items, remove_menu_items

from .og_rtx_sensors import Ros2CameraGraph, Ros2RtxLidarGraph
from .og_utils import Ros2ClockGraph, Ros2GenericPubGraph, Ros2JointStatesGraph, Ros2OdometryGraph, Ros2TfPubGraph


class Extension(omni.ext.IExt, MenuHelperExtensionFull):
    """Extension providing ROS 2 OmniGraph shortcut menu items."""

    def on_startup(self, ext_id: str) -> None:
        """Initialize the extension.

        Args:
            ext_id: Extension identifier assigned by Kit.

        """
        self._ext_id = ext_id
        carb.log_info("ROS2 Shortcuts Menu startup")

        # Create ROS 2 OmniGraph menu items using MenuHelperExtensionFull
        self.menu_startup(
            lambda: Ros2CameraGraph(),
            "ROS 2 Camera",
            "Camera",
            "Tools/Robotics/ROS 2 OmniGraphs",
        )
        self.menu_startup(
            lambda: Ros2RtxLidarGraph(),
            "ROS 2 RTX Lidar",
            "RTX Lidar",
            "Tools/Robotics/ROS 2 OmniGraphs",
        )
        self.menu_startup(
            lambda: Ros2TfPubGraph(),
            "ROS 2 TF Publisher",
            "TF Publisher",
            "Tools/Robotics/ROS 2 OmniGraphs",
        )
        self.menu_startup(
            lambda: Ros2OdometryGraph(),
            "ROS 2 Odometry Publisher",
            "Odometry Publisher",
            "Tools/Robotics/ROS 2 OmniGraphs",
        )
        self.menu_startup(
            lambda: Ros2JointStatesGraph(),
            "ROS 2 Joint States",
            "Joint States",
            "Tools/Robotics/ROS 2 OmniGraphs",
        )
        self.menu_startup(
            lambda: Ros2ClockGraph(),
            "ROS 2 Clock",
            "Clock",
            "Tools/Robotics/ROS 2 OmniGraphs",
        )
        self.menu_startup(
            lambda: Ros2GenericPubGraph(),
            "ROS 2 Generic Publisher",
            "Generic Publisher",
            "Tools/Robotics/ROS 2 OmniGraphs",
        )

        # Register actions for ROS 2 Assets
        action_registry = omni.kit.actions.core.get_action_registry()
        action_registry.register_action(
            self._ext_id,
            "create_nova_carter_ros",
            lambda: self.create_asset("/Isaac/Samples/ROS2/Robots/Nova_Carter_ROS.usd", "/nova_carter_ROS"),
            description="Create Nova Carter ROS robot in scene",
        )
        action_registry.register_action(
            self._ext_id,
            "create_leatherback_ros",
            lambda: self.create_asset("/Isaac/Samples/ROS2/Robots/leatherback_ROS.usd", "/leatherback_ROS"),
            description="Create Leatherback ROS robot in scene",
        )
        action_registry.register_action(
            self._ext_id,
            "create_iw_hub_ros",
            lambda: self.create_asset("/Isaac/Samples/ROS2/Robots/iw_hub_ROS.usd", "/iw_hub_ROS"),
            description="Create iw.hub ROS robot in scene",
        )

        # ROS 2 Assets
        action_registry.register_action(
            self._ext_id,
            "open_content_browser_ros2",
            partial(open_content_browser_to_path, "/Isaac/Samples/ROS2"),
            description="Open the Content Browser to the ROS 2 assets folder",
        )

        ros_assets_sub_menu = [
            MenuItemDescription(name="Asset Browser", onclick_action=(self._ext_id, "open_content_browser_ros2")),
            MenuItemDescription(
                name="Nova Carter",
                onclick_action=(self._ext_id, "create_nova_carter_ros"),
            ),
            MenuItemDescription(
                name="Leatherback",
                onclick_action=(self._ext_id, "create_leatherback_ros"),
            ),
            MenuItemDescription(
                name="iw.hub ROS",
                onclick_action=(self._ext_id, "create_iw_hub_ros"),
            ),
        ]

        self._ros_assets_menu = [
            MenuItemDescription(name="ROS 2 Assets", glyph="plug.svg", sub_menu=ros_assets_sub_menu)
        ]

        add_menu_items(self._ros_assets_menu, "Create")

        # self.__ros_menu_layout = [
        #     MenuLayout.Menu(
        #         "Create",
        #         [
        #             MenuLayout.SubMenu(
        #                 "ROS 2 Assets",
        #                 [
        #                     MenuLayout.Item("Asset Browser", source="Create/ROS 2 Assets/Asset Browser"),
        #                     MenuLayout.Seperator("Examples"),
        #                     MenuLayout.Item("Room", source="Create/ROS 2 Assets/Room"),
        #                     MenuLayout.Item("Room 2", source="Create/ROS 2 Assets/Room 2"),
        #                 ],
        #             ),
        #         ]
        #     )
        # ]
        # add_layout(self.__ros_menu_layout)

    def create_asset(
        self, usd_path: str, stage_path: str, camera_position: Optional[Any] = None, camera_target: Optional[Any] = None
    ) -> None:
        """Create a USD asset reference on the stage.

        Args:
            usd_path: USD asset path relative to the Isaac Sim assets root.
            stage_path: Stage prim path where the reference should be created.
            camera_position: Optional viewport camera position to apply after creating the reference.
            camera_target: Optional viewport camera target to apply after creating the reference.

        """
        self._assets_root_path = get_assets_root_path()
        if self._assets_root_path is None:
            carb.log_error("Could not find Isaac Sim assets folder")
            return

        path_to = omni.kit.commands.execute(
            "CreateReferenceCommand",
            usd_context=omni.usd.get_context(),
            path_to=stage_path,
            asset_path=self._assets_root_path + usd_path,
            instanceable=False,
        )

        carb.log_info(f"Added reference to {stage_path} at {path_to}")

        if camera_position is not None and camera_target is not None:
            ViewportManager.set_camera_view("/OmniverseKit_Persp", eye=camera_position, target=camera_target)

    def on_shutdown(self) -> None:
        """Clean up resources when the extension shuts down."""
        carb.log_info("ROS2 Shortcuts Menu shutdown")
        self.menu_shutdown()
        remove_menu_items(self._ros_assets_menu, "Create")

        # Deregister actions
        action_registry = omni.kit.actions.core.get_action_registry()
        action_registry.deregister_action(self._ext_id, "create_nova_carter_ros")
        action_registry.deregister_action(self._ext_id, "create_leatherback_ros")
        action_registry.deregister_action(self._ext_id, "create_iw_hub_ros")
        action_registry.deregister_action(self._ext_id, "open_content_browser_ros2")
        # remove_layout(self.__ros_menu_layout)
