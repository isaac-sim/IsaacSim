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

"""Extension that provides a user interface for grasping functionality within Isaac Sim."""

import omni.ext
from omni.kit.menu.utils import MenuHelperExtensionFull

from .grasping_window import GraspingWindow


class GraspingUIExtension(omni.ext.IExt, MenuHelperExtensionFull):
    """Extension that provides a user interface for grasping functionality within Isaac Sim.

    This extension creates a dedicated window accessible through the Tools/Replicator menu that enables
    users to interact with grasping-related features and workflows. The extension integrates with the
    Omniverse Kit SDK menu system to provide seamless access to grasping tools and configurations.
    """

    WINDOW_NAME = "Grasping"
    """Name of the grasping window."""
    MENU_GROUP = "Tools/Replicator"
    """Menu group path where the grasping window appears in the interface."""

    def on_startup(self, ext_id: str) -> None:
        """Initialize the Grasping extension UI.

        Sets up the menu item and window for the Grasping tool in the Tools/Replicator menu group.

        Args:
            ext_id: The extension ID provided by the extension system.
        """
        self.menu_startup(
            lambda: GraspingWindow(title=self.WINDOW_NAME),
            self.WINDOW_NAME,
            self.WINDOW_NAME,
            self.MENU_GROUP,
        )

    def on_shutdown(self) -> None:
        """Clean up the Grasping extension UI.

        Removes the menu item and closes any open Grasping windows.
        """
        self.menu_shutdown()
