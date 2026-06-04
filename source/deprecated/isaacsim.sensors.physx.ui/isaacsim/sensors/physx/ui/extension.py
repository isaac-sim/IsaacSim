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

"""UI extension for PhysX sensors that adds sensor functionality to the Kit interface."""

import carb
import omni.ext
from pxr import Sdf, UsdShade

from .. import _range_sensor
from .menu import RangeSensorMenu


class Extension(omni.ext.IExt):
    """Isaac Sim PhysX sensors UI extension.

    Provides user interface components and menu integration for PhysX-based sensors in Isaac Sim. This extension
    registers UI elements for managing and configuring range sensors including lidar and generic sensor types through
    the property panel and context menus.
    """

    def on_startup(self, ext_id: str) -> None:
        """Called when the extension is starting up.

        Initializes the range sensor interfaces, creates the menu, and sets up hooks for property menu registration.

        Args:
            ext_id: The unique identifier of the extension being started.
        """
        self._lidar = _range_sensor.acquire_lidar_sensor_interface()
        self._generic = _range_sensor.acquire_generic_sensor_interface()

        self._menu = RangeSensorMenu(ext_id)
        self._registered = False
        manager = omni.kit.app.get_app().get_extension_manager()
        self._hook = manager.subscribe_to_extension_enable(
            on_enable_fn=lambda _: self._register_property_menu(),
            on_disable_fn=lambda _: self._unregister_property_menu(),
            ext_name="omni.kit.property.usd",
            hook_name="isaacsim.sensors.physx omni.kit.property.usd listener",
        )

    def on_shutdown(self) -> None:
        """Called when the extension is shutting down.

        Cleans up resources by unregistering the property menu, shutting down the menu, and releasing sensor interfaces.
        """
        self._hook = None
        if self._registered:
            self._unregister_property_menu()
        self._menu.shutdown()
        self._menu = None

        _range_sensor.release_lidar_sensor_interface(self._lidar)
        _range_sensor.release_generic_sensor_interface(self._generic)

    def _register_property_menu(self) -> None:
        """Registers the property menu for the extension.

        Marks the menu as registered and sets up context menu items if the context menu is available.
        """
        self._registered = True
        # +add menu item(s)

        context_menu = omni.kit.context_menu.get_instance()
        if context_menu is None:
            carb.log_error("context_menu is disabled!")
            return

    def _unregister_property_menu(self) -> None:
        """Unregisters the property menu for the extension.

        Marks the menu as unregistered and prevents multiple unregistration attempts.
        """
        # prevent unregistering multiple times
        if self._registered is False:
            return
        self._registered = False

    def _is_material(self, objects: dict) -> bool:
        """Checks if any of the provided objects contains material prims.

        Args:
            objects: Dictionary containing prim_list and stage information.

        Returns:
            True if any prim in the list is a UsdShade.Material, False otherwise.
        """
        if not "prim_list" in objects:
            return False
        prim_list = objects["prim_list"]
        stage = objects["stage"]
        if prim_list and stage is not None:
            for prim_path in prim_list:
                if isinstance(prim_path, Sdf.Path):
                    prim = stage.GetPrimAtPath(prim_path)
                    if prim.IsA(UsdShade.Material):
                        return True

        return False
