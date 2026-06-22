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

"""Extension for the isaacsim.sensors.camera.ui extension that provides UI integration for camera and depth sensor creation."""

import gc
from pathlib import Path

import carb
import isaacsim.core.experimental.utils.stage as stage_utils
import omni.ext
import omni.kit.actions.core
import omni.kit.commands
import omni.usd
from isaacsim.gui.components.menu import create_submenu
from isaacsim.sensors.experimental.rtx import (
    SUPPORTED_CAMERA_CONFIGS,
    RtxCamera,
    SingleViewDepthCameraSensor,
    get_camera_metadata,
)
from isaacsim.storage.native import get_assets_root_path
from omni.kit.menu.utils import add_menu_items, remove_menu_items
from pxr import Usd


def _build_sensors_dict() -> dict:
    """Build the vendor-grouped sensor menu dict from ``SUPPORTED_CAMERA_CONFIGS``.

    Iterates the registry in declared order and groups entries by their derived
    vendor name (so dict insertion order also drives menu ordering). The result
    has the legacy shape so any consumer of ``Extension.SENSORS`` keeps working.

    Returns:
        Sensor menu entries grouped by vendor and display name.
    """
    sensors: dict = {}
    for config_path in SUPPORTED_CAMERA_CONFIGS:
        meta = get_camera_metadata(config_path)
        sensors.setdefault(meta["vendor"], {})[meta["display_name"]] = {
            "prim_prefix": meta["prim_prefix"],
            "usd_path": config_path,
            "is_depth_sensor": meta["is_depth_sensor"],
        }
    return sensors


def _wrap_depth_sensor_cameras(rtx_cam: RtxCamera) -> list:
    """Wrap each Camera in the loaded asset that has a depth-sensor template render product.

    The deprecated ``SingleViewDepthSensorAsset`` walked the asset tree and discovered every
    ``RenderProduct`` carrying ``OmniSensorDepthSensorSingleViewAPI`` whose ``camera``
    relationship targets a Camera prim, then created a runtime sensor per match. We mirror
    that here using the experimental :class:`SingleViewDepthCameraSensor`, which copies
    template depth-sensor attributes onto the new sensor's render product via
    ``_populate_from_asset_template``.

    Args:
        rtx_cam: Authoring object returned by :meth:`RtxCamera.create`. Its
            ``_asset_root_path`` is used as the search root.

    Returns:
        List of created :class:`SingleViewDepthCameraSensor` instances. Empty when
        the asset has no template render products or the asset root could not be resolved.
    """
    asset_root_path = getattr(rtx_cam, "_asset_root_path", None)
    if asset_root_path is None:
        return []
    stage = stage_utils.get_current_stage(backend="usd")
    root_prim = stage.GetPrimAtPath(asset_root_path)
    if not root_prim.IsValid():
        carb.log_warn(f"Asset root prim at '{asset_root_path}' is not valid; skipping depth sensor wrapping.")
        return []
    sensors: list = []
    for child in Usd.PrimRange(root_prim):
        if not (
            child.GetTypeName() == "RenderProduct"
            and child.HasAPI("OmniSensorDepthSensorSingleViewAPI")
            and child.HasRelationship("camera")
        ):
            continue
        targets = child.GetRelationship("camera").GetTargets()
        if len(targets) != 1:
            carb.log_warn(
                f"Skipping render product '{child.GetPath()}': expected exactly 1 camera target, got {len(targets)}."
            )
            continue
        # USD render product `resolution` is (width, height); the experimental sensor API
        # follows OpenCV/NumPy convention `(height, width)`, so swap when constructing.
        res_attr = child.GetAttribute("resolution").Get()
        if res_attr is None:
            carb.log_warn(
                f"Skipping render product '{child.GetPath()}': no `resolution` attribute authored on template."
            )
            continue
        resolution = (int(res_attr[1]), int(res_attr[0]))
        sensors.append(
            SingleViewDepthCameraSensor(
                str(targets[0]),
                resolution=resolution,
                annotators="depth_sensor_distance",
            )
        )
    return sensors


class Extension(omni.ext.IExt):
    """Extension for the isaacsim.sensors.camera.ui extension that provides UI integration for camera and depth sensor creation.

    This extension adds menu items to the Create menu and context menus that allow users to create various camera and depth sensor prims in the USD stage. The list of supported sensors and their metadata (vendor grouping, display name, depth-sensor flag, default stage prim prefix) is sourced from :data:`isaacsim.sensors.experimental.rtx.SUPPORTED_CAMERA_CONFIGS` so adding a new vendor sensor is a one-place change in the registry.

    All menu actions load the asset via :meth:`isaacsim.sensors.experimental.rtx.RtxCamera.create`. For sensors whose registry entry sets ``is_depth_sensor=True``, every Camera in the loaded asset that has a template render product with the ``OmniSensorDepthSensorSingleViewAPI`` schema is additionally wrapped with :class:`isaacsim.sensors.experimental.rtx.SingleViewDepthCameraSensor`, which copies the template's depth-sensor attributes onto the new render product.
    """

    SENSORS = _build_sensors_dict()
    """Vendor-grouped sensor metadata derived from ``SUPPORTED_CAMERA_CONFIGS``.

    Outer mapping is ``vendor -> {display_name -> {prim_prefix, usd_path, is_depth_sensor}}``.
    Used to dynamically generate menu items and actions for creating sensor prims in the scene.
    """

    def on_startup(self, ext_id: str) -> None:
        """Initializes the extension by setting up sensor creation actions and menu items.

        Args:
            ext_id: The extension identifier.
        """
        self._ext_id = ext_id
        self._ext_name = omni.ext.get_extension_name(ext_id)
        self._registered_actions = []
        self._depth_sensors: list = []

        icon_dir = omni.kit.app.get_app().get_extension_manager().get_extension_path_by_module(__name__)

        action_registry = omni.kit.actions.core.get_action_registry()

        # Build menu structure based on SENSORS dictionary; vendor order follows
        # registry declaration order via dict insertion ordering (Python 3.7+).
        vendor_dicts = {}
        for vendor, sensors in self.SENSORS.items():
            sensor_items = []
            for sensor_name, sensor_data in sensors.items():
                prim_prefix = sensor_data["prim_prefix"]
                usd_path = sensor_data["usd_path"]
                is_depth_sensor = sensor_data.get("is_depth_sensor", False)

                action_id = "create_camera_" + sensor_name.lower().replace(" ", "_").replace("-", "_")
                action_fn = lambda *_, pp=prim_prefix, up=usd_path, depth=is_depth_sensor: self._create_camera(
                    pp, up, depth
                )
                action_registry.register_action(
                    self._ext_name,
                    action_id,
                    action_fn,
                    description=f"Create {sensor_name} camera sensor",
                )
                self._registered_actions.append(action_id)

                sensor_items.append({"name": sensor_name, "onclick_action": (self._ext_name, action_id)})

            vendor_dicts[vendor] = {"name": {vendor: sensor_items}}

        camera_and_depth_sensors_dict = {
            "name": {
                "Camera and Depth Sensors": list(vendor_dicts.values()),
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

        self._menu_items = create_submenu(sensors_menu_dict)
        add_menu_items(self._menu_items, "Create")

        context_menu_dict = {
            "name": {
                "Isaac": [
                    sensors_menu_dict,
                ],
            },
            "glyph": str(Path(icon_dir).joinpath("data/robot.svg")),
        }

        self._viewport_create_menu = omni.kit.context_menu.add_menu(context_menu_dict, "CREATE")

    def on_shutdown(self) -> None:
        """Cleans up the extension by removing menu items and deregistering actions."""
        remove_menu_items(self._menu_items, "Create")
        self._viewport_create_menu = None

        action_registry = omni.kit.actions.core.get_action_registry()
        for action_id in self._registered_actions:
            action_registry.deregister_action(self._ext_name, action_id)
        self._registered_actions.clear()

        self._depth_sensors.clear()
        gc.collect()

    def _get_stage_and_path(self) -> str | None:
        """Gets the currently selected prim path from the USD stage.

        Returns:
            The path of the last selected prim, or None if no prims are selected.
        """
        selectedPrims = omni.usd.get_context().get_selection().get_selected_prim_paths()

        if len(selectedPrims) > 0:
            curr_prim = selectedPrims[-1]
        else:
            curr_prim = None
        return curr_prim

    def _create_camera(self, prim_prefix: str, usd_path: str, is_depth_sensor: bool) -> None:
        """Create a camera sensor on the stage from a registered config.

        Loads the USD asset via :meth:`RtxCamera.create` at the next available free path
        derived from *prim_prefix*. When *is_depth_sensor* is true, additionally wraps
        every Camera in the loaded asset that has a depth-sensor template render product
        with :class:`SingleViewDepthCameraSensor` so the template's depth-sensor
        configuration is propagated to the new sensor's render product.

        Args:
            prim_prefix: Default stage prim path prefix for the loaded asset.
            usd_path: Asset-relative path passed to :meth:`RtxCamera.create`.
            is_depth_sensor: When ``True``, also instantiate depth-sensor wrappers
                for every Camera with a matching template render product in the asset.
        """
        rtx_cam = RtxCamera.create(
            path=stage_utils.generate_next_free_path(prim_prefix),
            usd_path=get_assets_root_path() + usd_path,
        )
        if is_depth_sensor:
            self._depth_sensors.extend(_wrap_depth_sensor_cameras(rtx_cam))
