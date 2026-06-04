# SPDX-FileCopyrightText: Copyright (c) 2025-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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

"""Extension for displaying sensor icons in Isaac Sim with visual representations and viewport controls for sensor prims."""

from pathlib import Path

import omni.ext
import omni.kit.widget.stage
from omni.kit.viewport.menubar.core import CategoryStateItem
from omni.kit.viewport.menubar.display import get_instance as get_menubar_display_instance
from omni.kit.viewport.registry import RegisterScene

from .model import IconModel
from .scene import VISIBLE_SETTING, IconScene

_extension = None


# Any class derived from `omni.ext.IExt` in top level module (defined in `python.modules` of `extension.toml`) will be
# instantiated when extension gets enabled and `on_startup(ext_id)` will be called. Later when extension gets disabled
# on_shutdown() is called.
class SensorIconExtension(omni.ext.IExt):
    """Extension for displaying sensor icons in Isaac Sim.

    This extension provides visual representations of sensors in the Isaac Sim interface by registering custom
    icons for sensor prims in the stage widget and adding viewport display controls. It enables users to easily
    identify and work with different types of sensors through visual indicators.

    The extension integrates with the stage widget to show sensor-specific icons for various sensor types and
    adds a "Sensors" category to the viewport's "Show By Type" menu for controlling sensor visibility.
    """

    # ext_id is current extension id. It can be used with extension manager to query additional information, like where
    # this extension is located on filesystem.
    def on_startup(self, ext_id: str) -> None:
        """Called when the extension is enabled.

        Sets up the sensor icon system including viewport scene registration, stage widget icons,
        and menubar display options for sensor visualization.

        Args:
            ext_id: Current extension identifier used to query extension information.
        """
        global _extension
        _extension = self
        self._vp2_scene = None
        self._vp2_scene = RegisterScene(IconScene, ext_id)

        # register sensor icon to stage widget
        self._sensor_icon_dir = omni.kit.app.get_app().get_extension_manager().get_extension_path_by_module(__name__)
        self._sensor_icon_path = str(Path(self._sensor_icon_dir).joinpath("icons/icoSensors.svg"))

        self._sensor_tpye = IconModel.SENSOR_TYPES
        self._stage_icons = omni.kit.widget.stage.StageIcons()
        for sensor_type in self._sensor_tpye:
            self._stage_icons.set(sensor_type, self._sensor_icon_path)

        # TODO: should we distinguish different viewport?
        # viewport_api_id = str(get_active_viewport().id)
        # sensor_icon_visible_setting = f"/persistent/app/viewport/{viewport_api_id}/sensor_icon/visible"
        self._menubar_display_inst = get_menubar_display_instance()
        self._custom_item = CategoryStateItem("Sensors", setting_path=VISIBLE_SETTING)
        self._menubar_display_inst.register_custom_category_item("Show By Type", self._custom_item)

    def on_shutdown(self) -> None:  # pragma: no cover
        """Called when the extension is disabled.

        Cleans up the sensor icon system by deregistering viewport scenes, stage widget icons,
        and menubar display items.
        """
        global _extension
        _extension = None
        self._vp2_scene = None

        # deregister sensor icon to stage widget
        for sensor_type in self._sensor_tpye:
            self._stage_icons.set(sensor_type, self._sensor_icon_path)

        self._menubar_display_inst.deregister_custom_category_item("Show By Type", self._custom_item)


def get_instance() -> SensorIconExtension | None:
    """Returns the current instance of the sensor icon extension.

    Provides access to the active SensorIconExtension instance for managing sensor icons
    in the Isaac Sim GUI.

    Returns:
        The active SensorIconExtension instance, or None if the extension is not enabled.
    """
    return _extension
