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

"""Provides user interface components for robotic manipulator control through OmniGraph controllers."""

import omni.ext
from omni.kit.menu.utils import MenuHelperExtensionFull

from .menu_graphs import ArticulationPositionWindow, ArticulationVelocityWindow, GripperWindow


class Extension(omni.ext.IExt, MenuHelperExtensionFull):
    """Extension class for the isaacsim.robot.manipulators.ui extension.

    This extension provides user interface components for robotic manipulator control through OmniGraph controllers.
    It creates menu entries and windows for three different types of robot controllers: articulation position control,
    articulation velocity control, and gripper control.

    The extension adds the following UI components to the Tools/Robotics/OmniGraph Controllers menu:

    - Articulation Position Controller: Provides joint position control interface
    - Articulation Velocity Controller: Provides joint velocity control interface
    - Gripper Controller: Provides open loop gripper control interface

    Each controller window offers a graphical interface for configuring and managing the corresponding OmniGraph
    controller nodes for robotic manipulators in Isaac Sim.
    """

    def on_startup(self, ext_id: str) -> None:
        """Initializes the extension by creating menu entries for OmniGraph robot controllers.

        Sets up menu items for Articulation Position Controller, Articulation Velocity Controller,
        and Gripper Controller under the Tools/Robotics/OmniGraph Controllers menu path.

        Args:
            ext_id: The extension identifier.
        """
        # Create menu using MenuHelperExtensionFull
        self.menu_startup(
            lambda: ArticulationPositionWindow(),
            "Articulation Position Controller",
            "Joint Position",
            "Tools/Robotics/OmniGraph Controllers",
        )
        self.menu_startup(
            lambda: ArticulationVelocityWindow(),
            "Articulation Velocity Controller",
            "Joint Velocity",
            "Tools/Robotics/OmniGraph Controllers",
        )
        self.menu_startup(
            lambda: GripperWindow(), "Gripper Controller", "Open Loop Gripper", "Tools/Robotics/OmniGraph Controllers"
        )

    def on_shutdown(self) -> None:
        """Cleans up the extension by removing all menu entries."""
        self.menu_shutdown()
