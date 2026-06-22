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

"""Extension module for RTX sensor UI integration in Isaac Sim."""

import gc
from collections.abc import Callable
from pathlib import Path
from typing import Any

import omni.ext
import omni.kit.actions.core
import omni.usd
from isaacsim.core.experimental.utils.stage import generate_next_free_path
from isaacsim.gui.components.menu import create_submenu
from isaacsim.sensors.experimental.rtx import (
    SUPPORTED_ACOUSTIC_CONFIGS,
    SUPPORTED_LIDAR_CONFIGS,
    SUPPORTED_RADAR_CONFIGS,
    Acoustic,
    Lidar,
    Radar,
)
from omni.kit.menu.utils import add_menu_items, remove_menu_items
from pxr import Tf


class Extension(omni.ext.IExt):
    """UI integration for creating RTX Lidar, Radar, and Acoustic sensors via the Create menu and viewport context menus.

    Vendor submenus are driven by ``SUPPORTED_LIDAR_CONFIGS``, ``SUPPORTED_RADAR_CONFIGS``, and
    ``SUPPORTED_ACOUSTIC_CONFIGS``. Radar and Acoustic also expose a generic NVIDIA entry that
    creates the prim without referencing a USD asset (Lidar's NVIDIA entries come from its configs).
    """

    def on_startup(self, ext_id: str) -> None:
        """Register RTX sensor creation actions and menus.

        Args:
            ext_id: Extension identifier provided by the extension manager.
        """
        self._ext_id = ext_id
        self._ext_name = omni.ext.get_extension_name(ext_id)
        self._registered_actions: list[str] = []

        icon_dir = omni.kit.app.get_app().get_extension_manager().get_extension_path_by_module(__name__)
        sensor_icon_path = str(Path(icon_dir).joinpath("data/sensor.svg"))

        rtx_lidar_vendor_list = self._register_sensor_configs(
            configs=SUPPORTED_LIDAR_CONFIGS,
            action_id_prefix="create_lidar",
            modality_label="RTX Lidar",
            create_callback=lambda sn, sc, dv: self._create_sensor_from_config(Lidar, sn, sc, dv),
        )

        radar_action_id = self._register_action(
            "create_rtx_radar",
            lambda *_: self._create_generic_sensor(Radar, "RtxRadar"),
            "Create RTX Radar sensor",
        )
        rtx_radar_vendor_list = self._register_sensor_configs(
            configs=SUPPORTED_RADAR_CONFIGS,
            action_id_prefix="create_radar",
            modality_label="RTX Radar",
            create_callback=lambda sn, sc, dv: self._create_sensor_from_config(Radar, sn, sc, dv),
            seed_vendor_entries={
                "NVIDIA": [{"name": "Generic RTX Radar", "onclick_action": (self._ext_name, radar_action_id)}]
            },
        )

        acoustic_action_id = self._register_action(
            "create_rtx_acoustic",
            lambda *_: self._create_generic_sensor(Acoustic, "RtxAcoustic"),
            "Create RTX Acoustic sensor",
        )
        rtx_acoustic_vendor_list = self._register_sensor_configs(
            configs=SUPPORTED_ACOUSTIC_CONFIGS,
            action_id_prefix="create_acoustic",
            modality_label="RTX Acoustic",
            create_callback=lambda sn, sc, dv: self._create_sensor_from_config(Acoustic, sn, sc, dv),
            seed_vendor_entries={
                "NVIDIA": [{"name": "Generic RTX Acoustic", "onclick_action": (self._ext_name, acoustic_action_id)}]
            },
        )

        sensors_menu_dict = {
            "name": {
                "Sensors": [
                    {"name": {"RTX Lidar": rtx_lidar_vendor_list}},
                    {"name": {"RTX Radar": rtx_radar_vendor_list}},
                    {"name": {"RTX Acoustic": rtx_acoustic_vendor_list}},
                ]
            },
            "glyph": sensor_icon_path,
        }

        self._menu_items = create_submenu(sensors_menu_dict)
        add_menu_items(self._menu_items, "Create")

        context_menu_dict = {"name": {"Isaac": [sensors_menu_dict]}, "glyph": sensor_icon_path}
        self._viewport_create_menu = omni.kit.context_menu.add_menu(context_menu_dict, "CREATE")

    def on_shutdown(self) -> None:
        """Remove RTX sensor menus and deregister creation actions."""
        remove_menu_items(self._menu_items, "Create")
        self._viewport_create_menu = None

        action_registry = omni.kit.actions.core.get_action_registry()
        for action_id in self._registered_actions:
            action_registry.deregister_action(self._ext_name, action_id)
        self._registered_actions.clear()

        gc.collect()

    def _register_action(self, action_id: str, fn: Callable, description: str) -> str:
        """Register an action and track it for shutdown.

        Args:
            action_id: Identifier used to register the action.
            fn: Function called when the action is invoked.
            description: Description shown for the registered action.

        Returns:
            Registered action identifier.
        """
        omni.kit.actions.core.get_action_registry().register_action(
            self._ext_name, action_id, fn, description=description
        )
        self._registered_actions.append(action_id)
        return action_id

    def _register_sensor_configs(
        self,
        *,
        configs: dict,
        action_id_prefix: str,
        modality_label: str,
        create_callback: Callable[[str, str, object], None],
        seed_vendor_entries: dict[str, list] | None = None,
    ) -> list:
        """Register one action per sensor config.

        Paths are assumed to use the ``/Isaac/Sensors/<Vendor>/<Sensor>/<Sensor>.usd`` layout.
        The first variant (if any) is picked as the default so multi-variant-set USDs (e.g. SICK
        family USDs) materialize a valid prim from a single menu click.

        Args:
            configs: Supported config paths mapped to variant metadata.
            action_id_prefix: Prefix used to build unique action identifiers.
            modality_label: Sensor modality label used in action descriptions.
            create_callback: Callback called with the sensor name, config name, and default variant.
            seed_vendor_entries: Optional vendor menu entries to include before config-derived entries.

        Returns:
            Vendor-grouped menu item list sorted by vendor name.
        """
        vendor_dict: dict[str, list] = {k: list(v) for k, v in (seed_vendor_entries or {}).items()}
        for config in configs:
            config_path = Path(config)
            vendor_name = config_path.parts[3]
            display_vendor = vendor_name.replace("_", " ")
            sensor_name = config_path.stem.replace("_", " ")
            if sensor_name.startswith(display_vendor):
                # Strip vendor prefix; assumes a single separator between vendor and sensor name.
                sensor_name = sensor_name[len(display_vendor) + 1 :]

            sensor_config = config_path.stem
            variants = configs[config]
            default_variant = next(iter(variants), None) if variants else None
            action_id = self._register_action(
                f"{action_id_prefix}_{sensor_config}",
                lambda *_, sn=sensor_name, sc=sensor_config, dv=default_variant: create_callback(sn, sc, dv),
                f"Create {vendor_name} {sensor_name} {modality_label} sensor",
            )
            vendor_dict.setdefault(display_vendor, []).append(
                {"name": sensor_name, "onclick_action": (self._ext_name, action_id)}
            )
        return [{"name": {v: vendor_dict[v]}} for v in sorted(vendor_dict)]

    def _get_stage_and_path(self) -> str | None:
        """Return the last selected prim path, or None if nothing is selected.

        Returns:
            Last selected prim path, or None if no prim is selected.
        """
        selected = omni.usd.get_context().get_selection().get_selected_prim_paths()
        return selected[-1] if selected else None

    def _resolve_prim_path(self, base_name: str) -> str:
        """Return a free prim path under the selection or default prim.

        Args:
            base_name: Base prim name to use for the generated path.

        Returns:
            Available prim path for a new sensor.
        """
        selected = self._get_stage_and_path()
        if selected:
            return generate_next_free_path(f"{selected}/{base_name}", prepend_default_prim=False)
        return generate_next_free_path(f"/{base_name}")

    def _create_sensor_from_config(
        self, sensor_cls: Any, sensor_name: str, sensor_config: str, variant: str | dict[str, str] | None
    ) -> None:
        """Create an RTX sensor from a supported config at the selected location.

        Args:
            sensor_cls: Sensor class exposing a ``create`` factory method.
            sensor_name: Display name used to derive the prim name.
            sensor_config: Sensor config name passed to the factory.
            variant: Variant metadata passed to the factory.
        """
        sensor_cls.create(
            self._resolve_prim_path(Tf.MakeValidIdentifier(sensor_name)),
            config=sensor_config,
            variant=variant,
        )

    def _create_generic_sensor(self, sensor_cls: Any, base_name: str) -> None:
        """Create a generic RTX sensor at the selected location.

        Args:
            sensor_cls: Sensor class to instantiate.
            base_name: Base prim name to use for the generated path.
        """
        sensor_cls(self._resolve_prim_path(base_name))
