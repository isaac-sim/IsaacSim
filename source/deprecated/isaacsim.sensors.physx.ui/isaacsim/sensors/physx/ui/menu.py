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

"""Provides menu integration for creating PhysX and LightBeam range sensors in Isaac Sim."""

import weakref
from pathlib import Path

import omni.ext
import omni.kit.actions.core
import omni.kit.commands
from isaacsim.core.utils.prims import create_prim
from isaacsim.core.utils.stage import get_next_free_path
from isaacsim.gui.components.menu import create_submenu
from isaacsim.storage.native import get_assets_root_path
from omni.kit.menu.utils import add_menu_items, remove_menu_items
from pxr import Gf


class RangeSensorMenu:
    """Provides menu integration for creating PhysX and LightBeam range sensors in Isaac Sim.

    This class registers actions and creates menu entries in both the Create menu and context menu for various
    range sensor types including PhysX Lidar (rotating and generic) and LightBeam sensors (generic and
    Tashan TS-F-A). Each menu item triggers the creation of the corresponding sensor prim with predefined
    configurations.

    The menu structure includes:
    - PhysX Lidar sensors with rotating and generic variants
    - LightBeam sensors with generic and Tashan TS-F-A variants

    Args:
        ext_id: Extension identifier used to register actions and manage the menu lifecycle.
    """

    def __init__(self, ext_id: str) -> None:
        self._ext_name = omni.ext.get_extension_name(ext_id)
        self._registered_actions = []

        icon_dir = omni.kit.app.get_app().get_extension_manager().get_extension_path_by_module(__name__)
        sensor_icon_path = str(Path(icon_dir).joinpath("data/sensor.svg"))

        action_registry = omni.kit.actions.core.get_action_registry()

        # Register actions for each sensor menu item.
        action_registry.register_action(
            self._ext_name,
            "create_physx_lidar_rotating",
            lambda *_, a=weakref.proxy(self): a._add_lidar(),
            description="Create a rotating PhysX Lidar sensor",
        )
        self._registered_actions.append("create_physx_lidar_rotating")

        action_registry.register_action(
            self._ext_name,
            "create_physx_lidar_generic",
            lambda *_, a=weakref.proxy(self): a._add_generic(),
            description="Create a generic PhysX Lidar sensor",
        )
        self._registered_actions.append("create_physx_lidar_generic")

        action_registry.register_action(
            self._ext_name,
            "create_lightbeam_generic",
            lambda *_, a=weakref.proxy(self): a._add_lightbeam_sensor(),
            description="Create a generic LightBeam sensor",
        )
        self._registered_actions.append("create_lightbeam_generic")

        action_registry.register_action(
            self._ext_name,
            "create_lightbeam_tashan_ts_f_a",
            lambda *_: create_prim(
                prim_path=get_next_free_path("/TS_F_A", None),
                prim_type="Xform",
                usd_path=get_assets_root_path() + "/Isaac/Sensors/Tashan/TS-F-A/TS-F-A.usd",
            ),
            description="Create a Tashan TS-F-A LightBeam sensor",
        )
        self._registered_actions.append("create_lightbeam_tashan_ts_f_a")

        # Build menu dictionary structure
        sensors_menu_dict = {
            "name": {
                "Sensors": [
                    {
                        "name": {
                            "PhysX Lidar": [
                                {
                                    "name": "Rotating",
                                    "onclick_action": (self._ext_name, "create_physx_lidar_rotating"),
                                },
                                {
                                    "name": "Generic",
                                    "onclick_action": (self._ext_name, "create_physx_lidar_generic"),
                                },
                            ]
                        }
                    },
                    {
                        "name": {
                            "LightBeam Sensor": [
                                {
                                    "name": "Generic",
                                    "onclick_action": (self._ext_name, "create_lightbeam_generic"),
                                },
                                {
                                    "name": "Tashan TS-F-A",
                                    "onclick_action": (self._ext_name, "create_lightbeam_tashan_ts_f_a"),
                                },
                            ]
                        }
                    },
                ]
            },
            "glyph": sensor_icon_path,
        }

        # Convert the dictionary to a menu and add it
        self._menu_items = create_submenu(sensors_menu_dict)
        add_menu_items(self._menu_items, "Create")

        # Add sensor to context menu
        context_menu_dict = {
            "name": {
                "Isaac": [
                    sensors_menu_dict,
                ],
            },
            "glyph": sensor_icon_path,
        }

        self._viewport_create_menu = omni.kit.context_menu.add_menu(context_menu_dict, "CREATE")

    def _get_stage_and_path(self) -> str | None:
        """Gets the current stage and selected prim path.

        Returns:
            The path of the last selected prim, or None if no prims are selected.
        """
        self._stage = omni.usd.get_context().get_stage()
        selectedPrims = omni.usd.get_context().get_selection().get_selected_prim_paths()

        if len(selectedPrims) > 0:
            curr_prim = selectedPrims[-1]
        else:
            curr_prim = None
        return curr_prim

    def _add_lidar(self, *args: object, **kwargs: object) -> None:
        """Creates a PhysX Lidar sensor with predefined configuration.

        Args:
            *args: Variable length argument list passed to the command.
            **kwargs: Additional keyword arguments passed to the command.
        """
        result, prim = omni.kit.commands.execute(
            "RangeSensorCreateLidar",
            path="/Lidar",
            parent=self._get_stage_and_path(),
            min_range=0.4,
            max_range=100.0,
            draw_points=False,
            draw_lines=False,
            horizontal_fov=360.0,
            vertical_fov=30.0,
            horizontal_resolution=0.4,
            vertical_resolution=4.0,
            rotation_rate=20.0,
            high_lod=False,
            yaw_offset=0.0,
            enable_semantics=False,
        )

    def _add_generic(self, *args: object, **kwargs: object) -> None:
        """Creates a generic range sensor with predefined configuration.

        Args:
            *args: Variable length argument list passed to the command.
            **kwargs: Additional keyword arguments passed to the command.
        """
        result, prim = omni.kit.commands.execute(
            "RangeSensorCreateGeneric",
            path="/GenericSensor",
            parent=self._get_stage_and_path(),
            min_range=0.4,
            max_range=100.0,
            draw_points=False,
            draw_lines=False,
            sampling_rate=60,
        )

    def _add_lightbeam_sensor(self, *args: object, **kargs: object) -> None:
        """Creates a LightBeam sensor with predefined configuration.

        Args:
            *args: Variable length argument list passed to the command.
            **kargs: Additional keyword arguments passed to the command.
        """
        result, prim = omni.kit.commands.execute(
            "IsaacSensorCreateLightBeamSensor",
            path="/LightBeam_Sensor",
            parent=self._get_stage_and_path(),
            translation=Gf.Vec3d(0, 0, 0),
            orientation=Gf.Quatd(1.0, 0.0, 0.0, 0.0),
            forward_axis=Gf.Vec3d(1, 0, 0),
        )

    def shutdown(self) -> None:
        """Shuts down the range sensor menu by removing menu items and deregistering actions."""
        remove_menu_items(self._menu_items, "Create")
        self._viewport_create_menu = None

        # Deregister all registered actions.
        action_registry = omni.kit.actions.core.get_action_registry()
        for action_id in self._registered_actions:
            action_registry.deregister_action(self._ext_name, action_id)
        self._registered_actions.clear()
