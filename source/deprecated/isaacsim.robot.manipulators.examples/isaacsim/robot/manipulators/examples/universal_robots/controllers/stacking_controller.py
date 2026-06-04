# SPDX-FileCopyrightText: Copyright (c) 2021-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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

"""Stacking controller for Universal Robots arms."""

from __future__ import annotations

import isaacsim.robot.manipulators.controllers as manipulators_controllers
import numpy as np
from isaacsim.core.prims import SingleArticulation
from isaacsim.core.utils.types import ArticulationAction
from isaacsim.robot.manipulators.examples.universal_robots.controllers.pick_place_controller import PickPlaceController
from isaacsim.robot.manipulators.grippers import SurfaceGripper


class StackingController(manipulators_controllers.StackingController):
    """Stacking controller for the UR10 robot.

    Args:
        name: Name identifier for the controller.
        gripper: The surface gripper to use.
        robot_articulation: The robot articulation to control.
        picking_order_cube_names: Ordered list of cube names to stack.
        robot_observation_name: Name key for robot observations.
    """

    def __init__(
        self,
        name: str,
        gripper: SurfaceGripper,
        robot_articulation: SingleArticulation,
        picking_order_cube_names: list[str],
        robot_observation_name: str,
    ) -> None:
        manipulators_controllers.StackingController.__init__(
            self,
            name=name,
            pick_place_controller=PickPlaceController(
                name=name + "_pick_place_controller", gripper=gripper, robot_articulation=robot_articulation
            ),
            picking_order_cube_names=picking_order_cube_names,
            robot_observation_name=robot_observation_name,
        )
        return

    def forward(
        self,
        observations: dict,
        end_effector_orientation: np.ndarray | None = None,
        end_effector_offset: np.ndarray | None = None,
    ) -> ArticulationAction:
        """Execute one step of the stacking controller.

        Args:
            observations: Dictionary of observations including cube positions.
            end_effector_orientation: Orientation for end effector.
            end_effector_offset: Offset for end effector.

        Returns:
            The articulation action to execute.
        """
        return super().forward(
            observations, end_effector_orientation=end_effector_orientation, end_effector_offset=end_effector_offset
        )
