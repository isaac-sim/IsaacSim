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

"""Plugin registration helpers for robot schema modules."""

import logging
import os

from pxr import Plug

logger = logging.getLogger(__name__)


def _register_plugin_path(path: str) -> None:
    """Register USD plugins located at a given path.

    Args:
        path: Directory path containing a plugInfo.json file.

    """
    pluginfo_path = os.path.join(path, "plugInfo.json")
    if not os.path.exists(pluginfo_path):
        return

    try:
        import json

        with open(pluginfo_path) as f:
            lines = f.readlines()
            json_lines = [line for line in lines if not line.strip().startswith("#")]
            json_content = "".join(json_lines)
            data = json.loads(json_content)

        plugin_names_to_register = set()
        if "Plugins" in data:
            for plugin_info in data["Plugins"]:
                if "Name" in plugin_info:
                    plugin_names_to_register.add(plugin_info["Name"])

        if not plugin_names_to_register:
            return

        registry = Plug.Registry()
        all_plugins = registry.GetAllPlugins()
        registered_plugin_names = {plugin.name for plugin in all_plugins}

        if plugin_names_to_register.issubset(registered_plugin_names):
            return

    except Exception:
        pass

    result = Plug.Registry().RegisterPlugins(path)
    if not result:
        logger.error(f"No plugins found at path {path}")


def _register_plugins(ext_path: str) -> None:
    """Register robot schema plugin resources for an extension path.

    Args:
        ext_path: Extension root path to scan for plugin resources.

    """
    _register_plugin_path(os.path.join(ext_path, "usd", "schema", "isaac", "robot_schema"))

    _register_plugin_path(os.path.join(ext_path, "usd", "schema", "isaac", "sensor_schema"))
    _register_plugin_path(os.path.join(ext_path, "usd", "schema", "isaac", "range_sensor_schema"))


# carb.tokens.get_tokens_interface().resolve("${isaacsim.robot.schema}") can not be resolved by sphinx
ext_path = os.path.dirname(os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))))
_register_plugins(ext_path)


from . import robot_schema as robot_schema  # type: ignore[attr-defined]
