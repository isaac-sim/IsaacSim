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

"""Create menu helpers for Isaac Sim assets and tools."""

import asyncio
from collections.abc import Sequence
from functools import partial
from pathlib import Path

import carb
import omni.ext
import omni.kit.actions.core
import omni.kit.menu.utils
from isaacsim.core.rendering_manager import ViewportManager
from isaacsim.gui.components.menu import create_submenu, open_content_browser_to_path
from isaacsim.storage.native.nucleus import get_assets_root_path
from omni.kit.menu.utils import MenuItemDescription, MenuLayout, add_menu_items, remove_menu_items


# -----------------------------------------------------------------------------
# Global create_asset function
# -----------------------------------------------------------------------------
def create_asset(
    usd_path: str,
    stage_path: str,
    camera_position: Sequence[float] | None = None,
    camera_target: Sequence[float] | None = None,
) -> None:
    """Create a reference to an Isaac Sim asset in the stage.

    Args:
        usd_path: Relative USD asset path under the Isaac Sim assets root.
        stage_path: Target prim path to create in the stage.
        camera_position: Optional camera position to frame the asset.
        camera_target: Optional camera target to frame the asset.

    Example:
        .. code-block:: python

            create_asset("/Isaac/Robots/IsaacSim/Ant/ant_instanceable.usd", "/Ant")
    """
    assets_root_path = get_assets_root_path()
    if assets_root_path is None:
        carb.log_error("Could not find Isaac Sim assets folder")
        return

    path_to = omni.kit.commands.execute(
        "CreateReferenceCommand",
        usd_context=omni.usd.get_context(),
        path_to=stage_path,
        asset_path=assets_root_path + usd_path,
        instanceable=False,
    )

    carb.log_info(f"Added reference to {stage_path} at {path_to}")

    if camera_position is not None and camera_target is not None:
        ViewportManager.set_camera_view("/OmniverseKit_Persp", eye=camera_position, target=camera_target)


# -----------------------------------------------------------------------------
# Global create_apriltag function
# -----------------------------------------------------------------------------
def create_apriltag(usd_path: str, shader_name: str, stage_path: str, tag_path: str) -> None:
    """Create an AprilTag material with a selected tag texture.

    Args:
        usd_path: Relative MDL asset path under the Isaac Sim assets root.
        shader_name: Name to assign to the created shader.
        stage_path: Target prim path for the material.
        tag_path: Relative texture path for the tag mosaic.

    Example:
        .. code-block:: python

            create_apriltag(
                "/Isaac/Materials/AprilTag/AprilTag.mdl",
                "AprilTag",
                "/Looks/AprilTag",
                "/Isaac/Materials/AprilTag/Textures/tag36h11.png",
            )
    """
    from pxr import Sdf

    assets_root_path = get_assets_root_path()
    if assets_root_path is None:
        carb.log_error("Could not find Isaac Sim assets folder")
        return

    stage = omni.usd.get_context().get_stage()
    stage_path = omni.usd.get_stage_next_free_path(stage, stage_path, False)

    async def create_tag() -> None:
        omni.kit.commands.execute(
            "CreateMdlMaterialPrim",
            mtl_url=assets_root_path + usd_path,
            mtl_name=shader_name,
            mtl_path=stage_path,
            select_new_prim=True,
        )
        mtl = stage.GetPrimAtPath(stage_path + "/Shader")
        attr = mtl.CreateAttribute("inputs:tag_mosaic", Sdf.ValueTypeNames.Asset)
        attr.Set(Sdf.AssetPath(assets_root_path + tag_path))

    asyncio.ensure_future(create_tag())


# -----------------------------------------------------------------------------
# Class CreateMenuExtension
# -----------------------------------------------------------------------------
class CreateMenuExtension:
    """Build and manage the Create menu for Isaac Sim.

    Args:
        ext_id: Extension identifier provided by the extension manager.
    """

    def __init__(self, ext_id: str) -> None:
        self._ext_id = ext_id
        self._ext_name = omni.ext.get_extension_name(ext_id)
        self._menu_categories = []
        self._registered_actions = []

        self.__menu_layout = [
            MenuLayout.Menu(
                "Create",
                [
                    MenuLayout.Item("Mesh"),
                    MenuLayout.Item("Shape"),
                    MenuLayout.Item("Lights", source="Create/Light"),
                    MenuLayout.Item("Audio"),
                    MenuLayout.Item("Camera"),
                    MenuLayout.Item("Scope"),
                    MenuLayout.Item("Xform", source="Create/Xform"),
                    MenuLayout.Item("Materials", source="Create/Material"),
                    MenuLayout.Item("Graphs", source="Create/Visual Scripting"),
                    MenuLayout.Item("Arbitrary Output Variables (AOV)", source="Create/AOV"),
                    MenuLayout.Item("Physics", source="Create/Physics"),
                    MenuLayout.Seperator("Assets"),
                    MenuLayout.SubMenu(
                        "Robots",
                        [
                            MenuLayout.Item("Asset Browser"),
                            MenuLayout.Seperator("Actuators"),
                            MenuLayout.Item("Surface Gripper"),
                            MenuLayout.Seperator("Examples"),
                        ],
                    ),
                    MenuLayout.SubMenu(
                        "Environments",
                        [
                            MenuLayout.Item(name="Asset Browser"),
                            MenuLayout.Seperator("Examples"),
                        ],
                    ),
                    MenuLayout.SubMenu(
                        "Sensors",
                        [
                            MenuLayout.Item(name="Asset Browser"),
                            MenuLayout.Seperator("Generic Sensors"),
                        ],
                    ),
                    # MenuLayout.Item("ROS 2 Assets", source="Create/ROS 2 Assets"),
                    MenuLayout.SubMenu(
                        "ROS 2 Assets",
                        [
                            MenuLayout.Item("Asset Browser", source="Create/ROS 2 Assets/Asset Browser"),
                            MenuLayout.Seperator("Examples"),
                            MenuLayout.Item("Room", source="Create/ROS 2 Assets/Room"),
                            MenuLayout.Item("Room 2", source="Create/ROS 2 Assets/Room 2"),
                        ],
                    ),
                ],
            )
        ]

        omni.kit.menu.utils.add_layout(self.__menu_layout)

        icon_dir = omni.kit.app.get_app().get_extension_manager().get_extension_path_by_module(__name__)
        robot_icon_path = str(Path(icon_dir).joinpath("data/robot.svg"))

        action_registry = omni.kit.actions.core.get_action_registry()

        # Register robot asset actions
        robot_assets = [
            ("create_robot_ant", "Ant", "/Isaac/Robots/IsaacSim/Ant/ant_instanceable.usd", "/Ant"),
            ("create_robot_spot", "Boston Dynamics Spot", "/Isaac/Robots/BostonDynamics/spot/spot.usd", "/spot"),
            (
                "create_robot_franka",
                "Franka Emika Panda Arm",
                "/Isaac/Robots/FrankaRobotics/FrankaPanda/franka.usd",
                "/Franka",
            ),
            (
                "create_robot_humanoid",
                "Humanoid",
                "/Isaac/Robots/IsaacSim/Humanoid/humanoid_instanceable.usd",
                "/Humanoid",
            ),
            (
                "create_robot_nova_carter",
                "Nova Carter with Sensors",
                "/Isaac/Robots/NVIDIA/NovaCarter/nova_carter.usd",
                "/Nova_Carter",
            ),
            (
                "create_robot_quadcopter",
                "Quadcopter",
                "/Isaac/Robots/IsaacSim/Quadcopter/quadcopter.usd",
                "/Quadcopter",
            ),
        ]

        for action_id, display_name, usd_path, stage_path in robot_assets:
            action_registry.register_action(
                self._ext_name,
                action_id,
                partial(create_asset, usd_path, stage_path),
                display_name=f"Create {display_name}",
                description=f"Create a {display_name} robot",
            )
            self._registered_actions.append(action_id)

        action_registry.register_action(
            self._ext_name,
            "open_content_browser_robots",
            partial(open_content_browser_to_path, "/Isaac/Robots"),
            display_name="Open Content Browser (Robots)",
            description="Open the Content Browser to the Robots folder",
        )
        self._registered_actions.append("open_content_browser_robots")

        # Robot asset items shared between Create menu and right-click context menu
        robot_items = [
            {"name": "Ant", "onclick_action": (self._ext_name, "create_robot_ant")},
            {"name": "Boston Dynamics Spot (Quadruped)", "onclick_action": (self._ext_name, "create_robot_spot")},
            {"name": "Franka Emika Panda Arm", "onclick_action": (self._ext_name, "create_robot_franka")},
            {"name": "Humanoid", "onclick_action": (self._ext_name, "create_robot_humanoid")},
            {"name": "Nova Carter with Sensors", "onclick_action": (self._ext_name, "create_robot_nova_carter")},
            {"name": "Quadcopter", "onclick_action": (self._ext_name, "create_robot_quadcopter")},
        ]

        # Create menu includes an Asset Browser link; context menu does not
        robot_menu_dict = {
            "name": {
                "Robots": robot_items
                + [{"name": "Asset Browser", "onclick_action": (self._ext_name, "open_content_browser_robots")}]
            },
            "glyph": robot_icon_path,
        }

        self._menu_categories.append(add_menu_items(create_submenu(robot_menu_dict), "Create"))

        # Register environment asset actions
        env_assets = [
            (
                "create_env_black_grid",
                "Black Grid",
                "/Isaac/Environments/Grid/gridroom_black.usd",
                "/BlackGrid",
                None,
                None,
            ),
            (
                "create_env_flat_grid",
                "Flat Grid",
                "/Isaac/Environments/Grid/default_environment.usd",
                "/FlatGrid",
                None,
                None,
            ),
            (
                "create_env_simple_room",
                "Simple Room",
                "/Isaac/Environments/Simple_Room/simple_room.usd",
                "/SimpleRoom",
                [3.15, 3.15, 2.0],
                [0, 0, 0],
            ),
        ]

        for action_id, display_name, usd_path, stage_path, cam_pos, cam_target in env_assets:
            action_registry.register_action(
                self._ext_name,
                action_id,
                partial(create_asset, usd_path, stage_path, cam_pos, cam_target),
                display_name=f"Create {display_name}",
                description=f"Create a {display_name} environment",
            )
            self._registered_actions.append(action_id)

        ## Environments
        action_registry.register_action(
            self._ext_name,
            "open_content_browser_environments",
            partial(open_content_browser_to_path, "/Isaac/Environments"),
            display_name="Open Content Browser (Environments)",
            description="Open the Content Browser to the Environments folder",
        )
        self._registered_actions.append("open_content_browser_environments")

        # Environment asset items shared between Create menu and right-click context menu
        env_items = [
            {"name": "Black Grid", "onclick_action": (self._ext_name, "create_env_black_grid")},
            {"name": "Flat Grid", "onclick_action": (self._ext_name, "create_env_flat_grid")},
            {"name": "Simple Room", "onclick_action": (self._ext_name, "create_env_simple_room")},
        ]

        environment_menu_dict = {
            "name": {
                "Environments": env_items
                + [{"name": "Asset Browser", "onclick_action": (self._ext_name, "open_content_browser_environments")}]
            },
            "glyph": str(Path(icon_dir).joinpath("data/environment.svg")),
        }

        self._menu_categories.append(add_menu_items(create_submenu(environment_menu_dict), "Create"))

        ## Sensor
        action_registry.register_action(
            self._ext_name,
            "open_content_browser_sensors",
            partial(open_content_browser_to_path, "/Isaac/Sensors"),
            display_name="Open Content Browser (Sensors)",
            description="Open the Content Browser to the Sensors folder",
        )
        self._registered_actions.append("open_content_browser_sensors")

        sensor_sub_menu = [
            MenuItemDescription(name="Asset Browser", onclick_action=(self._ext_name, "open_content_browser_sensors")),
        ]
        sensor_icon_path = str(Path(icon_dir).joinpath("data/sensor.svg"))
        sensor_menu = [MenuItemDescription(name="Sensors", glyph=sensor_icon_path, sub_menu=sensor_sub_menu)]
        self._menu_categories.append(add_menu_items(sensor_menu, "Create"))

        ## April Tags
        apriltag_menu_dict = {
            "name": "April Tags",
            "onclick_action": (ext_id, "isaac_create_apriltag"),
            "glyph": str(Path(icon_dir).joinpath("data/apriltag.svg")),
        }
        self._menu_categories.append(add_menu_items(create_submenu(apriltag_menu_dict), "Create"))

        ## add apriltag selection
        action_registry = omni.kit.actions.core.get_action_registry()
        action_registry.register_action(
            ext_id,
            "isaac_create_apriltag",
            lambda: create_apriltag(
                "/Isaac/Materials/AprilTag/AprilTag.mdl",
                "AprilTag",
                "/Looks/AprilTag",
                "/Isaac/Materials/AprilTag/Textures/tag36h11.png",
            ),
            display_name="Create AprilTag",
            description="Create a AprilTag",
            tag="Create AprilTag",
        )

        # Right-click context menu (Viewport and Stage) uses the shared item lists
        # without the Asset Browser link
        isaac_create_menu_dict = {
            "name": {
                "Isaac": [
                    {"name": {"Robots": robot_items}, "glyph": robot_icon_path},
                    {
                        "name": {"Environments": env_items},
                        "glyph": str(Path(icon_dir).joinpath("data/environment.svg")),
                    },
                    apriltag_menu_dict,
                ]
            },
            "glyph": str(Path(icon_dir).joinpath("data/robot.svg")),
        }

        self._viewport_create_menu = omni.kit.context_menu.add_menu(
            isaac_create_menu_dict,
            "CREATE",
        )

    def shutdown(self) -> None:
        """Remove menu layouts and deregister actions.

        Example:
            .. code-block:: python

                menu = CreateMenuExtension("ext.id")
                menu.shutdown()
        """
        omni.kit.menu.utils.remove_layout(self.__menu_layout)
        for menu_item in self._menu_categories:
            remove_menu_items(menu_item, "Create")

        action_registry = omni.kit.actions.core.get_action_registry()
        action_registry.deregister_action(
            self._ext_id,
            "isaac_create_apriltag",
        )

        # Deregister all registered actions
        for action_id in self._registered_actions:
            action_registry.deregister_action(self._ext_name, action_id)
        self._registered_actions = []

        # remove_context_menus
        self._viewport_create_menu = None
