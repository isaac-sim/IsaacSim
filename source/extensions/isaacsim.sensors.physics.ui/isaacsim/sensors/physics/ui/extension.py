# SPDX-FileCopyrightText: Copyright (c) 2022-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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

"""UI extension that adds physics sensors to menus."""

import gc
import math
from pathlib import Path
from typing import Any

import carb
import omni.ext
import omni.kit.actions.core
from isaacsim.gui.components.menu import create_submenu
from isaacsim.sensors.experimental.physics import (
    IMU,
    Contact,
    Raycast,
)
from omni.kit.menu.utils import add_menu_items, remove_menu_items
from pxr import Gf


class Extension(omni.ext.IExt):
    """Extension that adds physics sensors to create menus."""

    def on_startup(self, ext_id: str) -> None:
        """Register sensor menu items when the extension starts.

        Args:
            ext_id: Extension identifier provided by the extension manager.
        """
        self._ext_id = ext_id
        self._ext_name = omni.ext.get_extension_name(ext_id)
        self._registered_actions = []

        icon_dir = omni.kit.app.get_app().get_extension_manager().get_extension_path_by_module(__name__)
        sensor_icon_path = str(Path(icon_dir).joinpath("data/sensor.svg"))

        # Register actions for each sensor menu item.
        action_registry = omni.kit.actions.core.get_action_registry()

        action_registry.register_action(
            self._ext_name,
            "create_contact_sensor",
            lambda *_: self._add_contact_sensor(),
            description="Create a contact sensor",
        )
        self._registered_actions.append("create_contact_sensor")

        action_registry.register_action(
            self._ext_name,
            "create_imu_sensor",
            lambda *_: self._add_imu_sensor(),
            description="Create an IMU sensor",
        )
        self._registered_actions.append("create_imu_sensor")

        action_registry.register_action(
            self._ext_name,
            "create_solid_state_physics_raycast_sensor",
            lambda *_: self._add_solid_state_physics_raycast_sensor(),
            description="Create a solid state physics raycast sensor",
        )
        self._registered_actions.append("create_solid_state_physics_raycast_sensor")

        action_registry.register_action(
            self._ext_name,
            "create_rotating_physics_raycast_sensor",
            lambda *_: self._add_rotating_physics_raycast_sensor(),
            description="Create a rotating physics raycast sensor",
        )
        self._registered_actions.append("create_rotating_physics_raycast_sensor")

        action_registry.register_action(
            self._ext_name,
            "create_beam_curtain_physics_raycast_sensor",
            lambda *_: self._add_beam_curtain_physics_raycast_sensor(),
            description="Create a beam curtain physics raycast sensor",
        )
        self._registered_actions.append("create_beam_curtain_physics_raycast_sensor")

        # Build menu dictionary structure
        sensors_menu_dict = {
            "name": {
                "Sensors": [
                    {
                        "name": "Contact Sensor",
                        "onclick_action": (self._ext_name, "create_contact_sensor"),
                    },
                    {
                        "name": "Imu Sensor",
                        "onclick_action": (self._ext_name, "create_imu_sensor"),
                    },
                    {
                        "name": {
                            "Physics Raycast Sensor": [
                                {
                                    "name": "Solid State Physics Raycast Sensor",
                                    "onclick_action": (self._ext_name, "create_solid_state_physics_raycast_sensor"),
                                },
                                {
                                    "name": "Rotating Physics Raycast Sensor",
                                    "onclick_action": (self._ext_name, "create_rotating_physics_raycast_sensor"),
                                },
                                {
                                    "name": "Beam Curtain Physics Raycast Sensor",
                                    "onclick_action": (self._ext_name, "create_beam_curtain_physics_raycast_sensor"),
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

    def on_shutdown(self) -> None:
        """Remove menu items and clean up on shutdown."""
        remove_menu_items(self._menu_items, "Create")
        self._viewport_create_menu = None

        # Deregister all registered actions.
        action_registry = omni.kit.actions.core.get_action_registry()
        for action_id in self._registered_actions:
            action_registry.deregister_action(self._ext_name, action_id)
        self._registered_actions.clear()

        gc.collect()

    def _get_stage_and_path(self) -> str | None:
        """Get the selected prim path from the stage.

        Returns:
            The selected prim path, or None if nothing is selected.
        """
        selected_prims = omni.usd.get_context().get_selection().get_selected_prim_paths()

        if len(selected_prims) > 0:
            curr_prim = selected_prims[-1]
        else:
            curr_prim = None
        return curr_prim

    def _add_contact_sensor(self, *args: Any, **kwargs: Any) -> None:
        """Create a contact sensor under the current selection.

        Args:
            *args: Additional positional arguments from the menu callback.
            **kwargs: Additional keyword arguments from the menu callback.
        """
        parent = self._get_stage_and_path()
        if parent is None:
            carb.log_error("No prim selected for contact sensor creation.")
            return
        Contact.create(
            f"{parent}/Contact_Sensor",
            min_threshold=0.0,
            max_threshold=100000.0,
            color=Gf.Vec4f(1, 0, 0, 1),
            radius=-1,
            translations=[[0.0, 0.0, 0.0]],
        )

    def _add_imu_sensor(self, *args: Any, **kwargs: Any) -> None:
        """Create an IMU sensor under the current selection.

        Args:
            *args: Additional positional arguments from the menu callback.
            **kwargs: Additional keyword arguments from the menu callback.
        """
        parent = self._get_stage_and_path()
        if parent is None:
            carb.log_error("No prim selected for IMU sensor creation.")
            return
        imu = IMU.create(
            f"{parent}/Imu_Sensor",
            translations=[[0.0, 0.0, 0.0]],
        )
        imu.set_visibilities([False])

    def _add_solid_state_physics_raycast_sensor(self, *args: Any, **kwargs: Any) -> None:
        """Create a solid state physics raycast sensor under the current selection.

        Args:
            *args: Additional positional arguments from the menu callback.
            **kwargs: Additional keyword arguments from the menu callback.
        """
        parent = self._get_stage_and_path()
        if parent is None:
            carb.log_error("No prim selected for raycast sensor creation.")
            return
        h_count, v_count = 10, 5
        h_fov, v_fov = 60.0, 20.0
        origins = []
        directions = []
        for vi in range(v_count):
            v_angle = math.radians(-v_fov / 2 + v_fov * vi / max(v_count - 1, 1))
            for hi in range(h_count):
                h_angle = math.radians(-h_fov / 2 + h_fov * hi / max(h_count - 1, 1))
                dx = math.cos(v_angle) * math.cos(h_angle)
                dy = math.cos(v_angle) * math.sin(h_angle)
                dz = math.sin(v_angle)
                origins.append([0.0, 0.0, 0.0])
                directions.append([dx, dy, dz])

        Raycast.create(
            f"{parent}/Solid_State_Physics_Raycast_Sensor",
            min_range=0.4,
            max_range=100.0,
            ray_origins=origins,
            ray_directions=directions,
            output_frame="WORLD",
        )

    def _add_rotating_physics_raycast_sensor(self, *args: Any, **kwargs: Any) -> None:
        """Create a rotating physics raycast sensor under the current selection.

        Args:
            *args: Additional positional arguments from the menu callback.
            **kwargs: Additional keyword arguments from the menu callback.
        """
        parent = self._get_stage_and_path()
        if parent is None:
            carb.log_error("No prim selected for raycast sensor creation.")
            return
        v_count = 8
        azimuth_steps = 36
        v_fov = 30.0
        rotation_rate = 1.0
        period = 1.0 / rotation_rate

        origins = []
        directions = []
        time_offsets = []
        for ai in range(azimuth_steps):
            h_angle = math.radians(360.0 * ai / azimuth_steps)
            t_offset = period * ai / azimuth_steps
            for vi in range(v_count):
                v_angle = math.radians(-v_fov / 2 + v_fov * vi / max(v_count - 1, 1))
                dx = math.cos(v_angle) * math.cos(h_angle)
                dy = math.cos(v_angle) * math.sin(h_angle)
                dz = math.sin(v_angle)
                origins.append([0.0, 0.0, 0.0])
                directions.append([dx, dy, dz])
                time_offsets.append(t_offset)

        Raycast.create(
            f"{parent}/Rotating_Physics_Raycast_Sensor",
            min_range=0.4,
            max_range=100.0,
            ray_origins=origins,
            ray_directions=directions,
            ray_time_offsets=time_offsets,
            output_frame="WORLD",
        )

    def _add_beam_curtain_physics_raycast_sensor(self, *args: Any, **kwargs: Any) -> None:
        """Create a beam curtain physics raycast sensor under the current selection.

        Args:
            *args: Additional positional arguments from the menu callback.
            **kwargs: Additional keyword arguments from the menu callback.
        """
        parent = self._get_stage_and_path()
        if parent is None:
            carb.log_error("No prim selected for raycast sensor creation.")
            return
        beam_count = 16
        curtain_height = 0.75
        origins = []
        directions = []
        for i in range(beam_count):
            z = -curtain_height / 2 + curtain_height * i / max(beam_count - 1, 1)
            origins.append([0.0, 0.0, z])
            directions.append([1.0, 0.0, 0.0])

        Raycast.create(
            f"{parent}/Beam_Curtain_Physics_Raycast_Sensor",
            min_range=0.2,
            max_range=10.0,
            ray_origins=origins,
            ray_directions=directions,
            output_frame="WORLD",
        )
