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

"""Extension entry point for the Isaac Sim Teleop UI window."""

import omni.ext
from omni.kit.menu.utils import MenuHelperExtensionFull, MenuItemOrder

from .teleop_window import TeleopWindow


class TeleopUIExtension(omni.ext.IExt, MenuHelperExtensionFull):
    """Extension class for the Teleop UI window."""

    WINDOW_NAME = "Teleop"
    MENU_GROUP = "Tools/Replicator"

    def on_startup(self, ext_id: str) -> None:
        """Initialize the extension.

        Args:
            ext_id: Extension identifier assigned by Kit.
        """
        self.menu_startup(
            lambda: TeleopWindow(title=self.WINDOW_NAME),
            self.WINDOW_NAME,
            self.WINDOW_NAME,
            self.MENU_GROUP,
            argv={"appear_after": ["Synthetic Data Recorder", MenuItemOrder.LAST]},
        )

    def on_shutdown(self) -> None:
        """Clean up resources."""
        self.menu_shutdown()
