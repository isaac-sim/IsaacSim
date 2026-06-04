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

"""Extension entry point for the Robot Self-Collision Detector."""

__all__ = ["CollisionDetectorExtension"]

import omni.ext
from omni.kit.menu.utils import MenuHelperExtensionFull

from .widget import deregister_selection_groups
from .window import RobotSelfCollisionWindow


class CollisionDetectorExtension(omni.ext.IExt, MenuHelperExtensionFull):
    """Extension providing the Robot Self-Collision Detector panel.

    Registers a dockable window for detecting and managing self-collision
    pairs between rigid body links of a robot articulation.
    """

    WINDOW_NAME = "Robot Self-Collision Detector"
    """Display title used for both the window and the menu entry."""
    #: Display title used for both the window and the menu entry.

    def on_startup(self, ext_id: str) -> None:
        """Initialize the extension and register the menu entry.

        Args:
            ext_id: Extension identifier provided by the extension manager.
        """
        self.menu_startup(
            lambda: RobotSelfCollisionWindow(),
            self.WINDOW_NAME,
            self.WINDOW_NAME,
            "Tools/Robotics/Asset Editors",
        )

    def on_shutdown(self) -> None:
        """Clean up the window, menu entry, and selection groups."""
        self.menu_shutdown()
        deregister_selection_groups()
