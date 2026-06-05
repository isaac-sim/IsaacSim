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

"""Visual Studio Code launcher and menu integration for Isaac Sim."""

from __future__ import annotations

import carb
import omni.ext

from . import ui_builder

_PYTHON_SERVER_SETTINGS = "/exts/isaacsim.code_editor.python_server"


class Extension(omni.ext.IExt):
    """VS Code integration extension for Isaac Sim.

    Adds a *Window > VS Code* menu item that launches VS Code pointed at the
    application directory.  The actual Python code execution server is provided
    by the ``isaacsim.code_editor.python_server`` extension.
    """

    def on_startup(self, ext_id: str) -> None:
        """Initialize the VS Code menu integration.

        Args:
            ext_id: The extension identifier.
        """
        settings = carb.settings.get_settings()
        host: str = settings.get(f"{_PYTHON_SERVER_SETTINGS}/host")
        port: int = settings.get(f"{_PYTHON_SERVER_SETTINGS}/port")

        ext_name = omni.ext.get_extension_name(ext_id)
        self._ui_builder = ui_builder.UIBuilder(ext_name, "Window", "VS Code", host, port)
        self._ui_builder.startup()

    def on_shutdown(self) -> None:
        """Clean up the VS Code menu integration."""
        self._ui_builder.shutdown()
