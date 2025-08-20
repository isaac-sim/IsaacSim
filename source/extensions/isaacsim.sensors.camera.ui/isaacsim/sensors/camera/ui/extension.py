# SPDX-FileCopyrightText: Copyright (c) 2022-2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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
import gc
import weakref
from pathlib import Path

import omni.ext
import omni.kit.commands
from isaacsim.core.utils.prims import create_prim
from isaacsim.core.utils.stage import get_next_free_path
from isaacsim.gui.components.menu import make_menu_item_description
from isaacsim.sensors.camera import SingleViewDepthSensorAsset
from isaacsim.storage.native import get_assets_root_path
from omni.kit.menu.utils import MenuItemDescription, add_menu_items, remove_menu_items


class Extension(omni.ext.IExt):
    # Define sensors data organized by vendor and sensor name
    SENSORS = {
        "Intel": {
            "Intel Realsense D455": {
                "prim_prefix": "/Realsense",
                "usd_path": "/Isaac/Sensors/Intel/RealSense/rsd455.usd",
                "is_depth_sensor": True,
            }
        },
        "Orbbec": {
            "Orbbec Gemini 2": {
                "prim_prefix": "/Gemini2",
                "usd_path": "/Isaac/Sensors/Orbbec/Gemini2/orbbec_gemini2_v1.0.usd",
                "is_depth_sensor": True,
            },
            "Orbbec FemtoMega": {
                "prim_prefix": "/Femto",
                "usd_path": "/Isaac/Sensors/Orbbec/FemtoMega/orbbec_femtomega_v1.0.usd",
                "is_depth_sensor": True,
            },
            "Orbbec Gemini 335": {
                "prim_prefix": "/Gemini335",
                "usd_path": "/Isaac/Sensors/Orbbec/Gemini335/orbbec_gemini_335.usd",
                "is_depth_sensor": True,
            },
            "Orbbec Gemini 335L": {
                "prim_prefix": "/Gemini335L",
                "usd_path": "/Isaac/Sensors/Orbbec/Gemini335L/orbbec_gemini_335L.usd",
                "is_depth_sensor": True,
            },
        },
        "Leopard Imaging": {
            "Hawk": {"prim_prefix": "/Hawk", "usd_path": "/Isaac/Sensors/LeopardImaging/Hawk/hawk_v1.1_nominal.usd"},
            "Owl": {"prim_prefix": "/Owl", "usd_path": "/Isaac/Sensors/LeopardImaging/Owl/owl.usd"},
        },
        "Sensing": {
            "Sensing SG2-AR0233C-5200-G2A-H100F1A": {
                "prim_prefix": "/SG2_AR0233C_5200_G2A_H100F1A",
                "usd_path": "/Isaac/Sensors/Sensing/SG2/H100F1A/SG2-AR0233C-5200-G2A-H100F1A.usd",
            },
            "Sensing SG2-OX03CC-5200-GMSL2-H60YA": {
                "prim_prefix": "/SG2_OX03CC_5200_GMSL2_H60YA",
                "usd_path": "/Isaac/Sensors/Sensing/SG2/H60YA/Camera_SG2_OX03CC_5200_GMSL2_H60YA.usd",
            },
            "Sensing SG3-ISX031C-GMSL2F-H190XA": {
                "prim_prefix": "/SG3_ISX031C_GMSL2F_H190XA",
                "usd_path": "/Isaac/Sensors/Sensing/SG3/H190XA/SG3S-ISX031C-GMSL2F-H190XA.usd",
            },
            "Sensing SG5-IMX490C-5300-GMSL2-H110SA": {
                "prim_prefix": "/SG5_IMX490C_5300_GMSL2_H110SA",
                "usd_path": "/Isaac/Sensors/Sensing/SG5/H100SA/SG5-IMX490C-5300-GMSL2-H110SA.usd",
            },
            "Sensing SG8S-AR0820C-5300-G2A-H120YA": {
                "prim_prefix": "/SG8_AR0820C_5300_G2A_H120YA",
                "usd_path": "/Isaac/Sensors/Sensing/SG8/H120YA/SG8S-AR0820C-5300-G2A-H120YA.usd",
            },
            "Sensing SG8S-AR0820C-5300-G2A-H30YA": {
                "prim_prefix": "/SG8_AR0820C_5300_G2A_H30YA",
                "usd_path": "/Isaac/Sensors/Sensing/SG8/H30YA/SG8S-AR0820C-5300-G2A-H30YA.usd",
            },
            "Sensing SG8S-AR0820C-5300-G2A-H60SA": {
                "prim_prefix": "/SG8_AR0820C_5300_G2A_H60SA",
                "usd_path": "/Isaac/Sensors/Sensing/SG8/H60SA/SG8S-AR0820C-5300-G2A-H60SA.usd",
            },
        },
        "Stereolabs": {
            "ZED_X": {
                "prim_prefix": "/ZED_X",
                "usd_path": "/Isaac/Sensors/Stereolabs/ZED_X/ZED_X.usd",
                "is_depth_sensor": True,
            }
        },
    }

    def on_startup(self, ext_id: str) -> None:

        icon_dir = omni.kit.app.get_app().get_extension_manager().get_extension_path_by_module(__name__)
        sensor_icon_path = str(Path(icon_dir).joinpath("data/sensor.svg"))

        # Helper function to create a sensor prim
        def _add_sensor(prim_prefix, usd_path):
            return lambda *_: create_prim(
                prim_path=get_next_free_path(prim_prefix, None),
                prim_type="Xform",
                usd_path=get_assets_root_path() + usd_path,
            )

        # Build menu structure based on SENSORS dictionary
        vendor_dicts = {}
        for vendor, sensors in self.SENSORS.items():
            sensor_items = []
            for sensor_name, sensor_data in sensors.items():
                prim_prefix = sensor_data["prim_prefix"]
                usd_path = sensor_data["usd_path"]
                if sensor_data.get("is_depth_sensor", False):
                    on_click_fn = lambda *_, prim_prefix=prim_prefix, usd_path=usd_path: self._create_depth_sensor(
                        prim_prefix, usd_path
                    )
                else:
                    on_click_fn = _add_sensor(prim_prefix, usd_path)
                sensor_items.append({"name": sensor_name, "onclick_fn": on_click_fn})

            vendor_dicts[vendor] = {"name": {vendor: sensor_items}}

        # Create the menu structure
        camera_and_depth_sensors_dict = {
            "name": {
                "Camera and Depth Sensors": [
                    vendor_dicts.get("Intel", {}),
                    vendor_dicts.get("Orbbec", {}),
                    vendor_dicts.get("Leopard Imaging", {}),
                    vendor_dicts.get("Sensing", {}),
                    vendor_dicts.get("Stereolabs", {}),
                ]
            }
        }

        sensors_menu_dict = {
            "name": {
                "Sensors": [
                    camera_and_depth_sensors_dict,
                ]
            },
            "glyph": str(Path(icon_dir).joinpath("data/sensor.svg")),
        }

        def create_submenu(menu_dict):
            # Handle non-nested menu items
            if "name" in menu_dict and isinstance(menu_dict["name"], str):
                return MenuItemDescription(
                    name=menu_dict["name"],
                    onclick_fn=menu_dict.get("onclick_fn"),
                    onclick_action=menu_dict.get("onclick_action"),
                    glyph=menu_dict.get("glyph"),
                )

            # Handle nested submenus recursively
            submenu_name = next(iter(menu_dict["name"]))
            items = menu_dict["name"][submenu_name]
            sub_menu_items = []
            for item in items:
                if isinstance(item.get("name"), dict):
                    # Recursively handle nested submenu
                    sub_menu_items.append(create_submenu(item))
                else:
                    # Handle leaf menu item
                    sub_menu_items.append(
                        MenuItemDescription(
                            name=item["name"],
                            onclick_fn=item.get("onclick_fn"),
                            onclick_action=item.get("onclick_action"),
                        )
                    )

            return MenuItemDescription(name=submenu_name, sub_menu=sub_menu_items, glyph=menu_dict.get("glyph"))

        self._menu_items = create_submenu(sensors_menu_dict)
        add_menu_items([self._menu_items], "Create")

        # add sensor to context menu
        context_menu_dict = {
            "name": {
                "Isaac": [
                    sensors_menu_dict,
                ],
            },
            "glyph": str(Path(icon_dir).joinpath("data/robot.svg")),
        }

        self._viewport_create_menu = omni.kit.context_menu.add_menu(context_menu_dict, "CREATE")

    def on_shutdown(self):
        remove_menu_items(self._menu_items, "Create")
        self._viewport_create_menu = None
        gc.collect()

    def _get_stage_and_path(self):
        selectedPrims = omni.usd.get_context().get_selection().get_selected_prim_paths()

        if len(selectedPrims) > 0:
            curr_prim = selectedPrims[-1]
        else:
            curr_prim = None
        return curr_prim

    def _create_depth_sensor(self, prim_prefix, usd_path):
        depth_sensor = SingleViewDepthSensorAsset(
            prim_path=get_next_free_path(prim_prefix, None),
            asset_path=get_assets_root_path() + usd_path,
        )
        depth_sensor.initialize()
