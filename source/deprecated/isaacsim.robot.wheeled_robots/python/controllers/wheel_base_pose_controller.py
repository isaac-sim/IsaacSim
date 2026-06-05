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

"""WheelBasePoseController implements a pose controller for wheeled robots that drives them to desired positions using underlying wheel commands."""

import math

import numpy as np
from isaacsim.core.api.controllers.base_controller import BaseController
from isaacsim.core.utils.rotations import quat_to_euler_angles
from isaacsim.core.utils.types import ArticulationAction


class WheelBasePoseController(BaseController):
    """This controller closes the control loop, returning the wheel commands that will drive the robot to a desired pose. It does this by exploiting an open loop controller for the robot passed at class initialization.

    .. hint::

        The logic for this controller is the following:

        * calculate the difference between the current position and the target position, and compare against the position tolerance. If the result is inside the tolerance, stop forward motion (ie. stop if closer than position tolerance)

        * calculate the difference between the current heading and the target heading, and compare against the heading tolerance. If the result is outside the tolerance, stop forward motion and turn to align with the target heading (ie. dont bother turning unless it's by more than the heading tolerance)

        * otherwise proceed forward

    Args:
        name: Name identifier for the controller.
        open_loop_wheel_controller: A controller that takes in a command of
            [longitudinal velocity, steering angle] and returns the
            ArticulationAction to be applied to the wheels if non holonomic.
            and [longitudinal velocity, latitude velocity, steering angle]
            if holonomic.
        is_holonomic: Whether the robot uses holonomic drive.
    """

    def __init__(self, name: str, open_loop_wheel_controller: BaseController, is_holonomic: bool = False) -> None:
        super().__init__(name)
        self._open_loop_wheel_controller = open_loop_wheel_controller
        self._is_holonomic = is_holonomic
        return

    def forward(
        self,
        start_position: np.ndarray,
        start_orientation: np.ndarray,
        goal_position: np.ndarray,
        lateral_velocity: float = 0.2,
        yaw_velocity: float = 0.5,
        heading_tol: float = 0.05,
        position_tol: float = 0.04,
    ) -> ArticulationAction:
        """Use the specified open loop controller to compute speed commands to the wheel joints of the robot that will move it towards the specified goal position.

        Args:
            start_position: The current position of the robot, (X, Y, Z).
            start_orientation: The current orientation quaternion of the robot, (W, X, Y, Z).
            goal_position: The desired target position, (X, Y, Z).
            lateral_velocity: How fast the robot will move forward.
            yaw_velocity: How fast the robot will turn.
            heading_tol: The heading tolerance.
            position_tol: The position tolerance.

        Returns:
            Action containing wheel joint commands to drive toward the goal.
        """
        # Compute steering yaw in world frame
        steering_yaw = math.atan2(goal_position[1] - start_position[1], goal_position[0] - start_position[0])

        # Extract current heading from quaternion
        current_yaw_heading = quat_to_euler_angles(start_orientation)[-1]

        # Normalize yaw error to [-π, π]
        yaw_error = (steering_yaw - current_yaw_heading + np.pi) % (2 * np.pi) - np.pi

        # Compute 2D Euclidean distance
        distance_to_goal = np.linalg.norm(start_position[:2] - goal_position[:2])

        if distance_to_goal < position_tol:
            # Stop
            command = [0.0, 0.0, 0.0] if self._is_holonomic else [0.0, 0.0]

        elif abs(yaw_error) > heading_tol:
            # Turn to align with goal
            turn_direction = 1 if yaw_error > 0 else -1
            command = (
                [0.0, 0.0, turn_direction * yaw_velocity]
                if self._is_holonomic
                else [0.0, turn_direction * yaw_velocity]
            )

        else:
            # Go forward
            command = [lateral_velocity, 0.0, 0.0] if self._is_holonomic else [lateral_velocity, 0]

        return self._open_loop_wheel_controller.forward(command)

    def reset(self) -> None:
        """Reset the controller state."""
        return
