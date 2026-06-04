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

"""Pick-and-place controller for Universal Robots arms."""

from __future__ import annotations

import isaacsim.robot.manipulators.controllers as manipulators_controllers
import numpy as np
from isaacsim.core.prims import SingleArticulation
from isaacsim.core.utils.rotations import euler_angles_to_quat
from isaacsim.core.utils.types import ArticulationAction
from isaacsim.robot.manipulators.examples.universal_robots.controllers.rmpflow_controller import RMPFlowController
from isaacsim.robot.manipulators.grippers.surface_gripper import SurfaceGripper


class PickPlaceController(manipulators_controllers.PickPlaceController):
    """Pick and place controller for the UR10 robot.

    Args:
        name: Name identifier for the controller.
        gripper: The surface gripper to use.
        robot_articulation: The robot articulation to control.
        events_dt: Timesteps for pick/place events.
    """

    def __init__(
        self,
        name: str,
        gripper: SurfaceGripper,
        robot_articulation: SingleArticulation,
        events_dt: list[float] | None = None,
    ) -> None:
        if events_dt is None:
            events_dt = [0.01, 0.0035, 0.01, 1.0, 0.008, 0.005, 0.005, 1, 0.01, 0.08]
        manipulators_controllers.PickPlaceController.__init__(
            self,
            name=name,
            cspace_controller=RMPFlowController(
                name=name + "_cspace_controller", robot_articulation=robot_articulation, attach_gripper=True
            ),
            gripper=gripper,
            events_dt=events_dt,
        )
        return

    def forward(
        self,
        picking_position: np.ndarray,
        placing_position: np.ndarray,
        current_joint_positions: np.ndarray,
        end_effector_offset: np.ndarray | None = None,
        end_effector_orientation: np.ndarray | None = None,
    ) -> ArticulationAction:
        """Execute one step of the pick and place controller.

        Args:
            picking_position: Position to pick from.
            placing_position: Position to place at.
            current_joint_positions: Current robot joint positions.
            end_effector_offset: Offset for end effector.
            end_effector_orientation: Orientation for end effector.

        Returns:
            The articulation action to execute.
        """
        if end_effector_orientation is None:
            end_effector_orientation = euler_angles_to_quat(np.array([0, np.pi / 2.0, 0]))
        return super().forward(
            picking_position,
            placing_position,
            current_joint_positions,
            end_effector_offset=end_effector_offset,
            end_effector_orientation=end_effector_orientation,
        )
