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

"""UR10 follow target task implementation."""

from __future__ import annotations

import numpy as np
from isaacsim.core.experimental.objects import Cube, DomeLight, GroundPlane
from isaacsim.core.experimental.prims import GeomPrim
from isaacsim.robot.experimental.manipulators.examples.universal_robots.ur10 import UR10


class UR10FollowTarget:
    """Standalone UR10 follow target task.

    This class provides a complete follow target implementation for UR10 robot
    without inheriting from any base classes. It manages the robot, target cube, and
    provides the necessary interface for the simulation.
    """

    def __init__(self) -> None:
        """Initialize the UR follow target task."""
        # Initialize robot and target references
        self.robot = None
        self.target_cube = None

    def setup_scene(
        self,
        target_position: np.ndarray | None = None,
    ) -> None:
        """Set up the scene with robot, target cube, and environment.

        Args:
            target_position: Initial target position. If None, uses default.
        """
        # Set default target position if none provided
        if target_position is None:
            self.target_position = [0.5, 0.0, 0.3]
        else:
            self.target_position = target_position if isinstance(target_position, list) else list(target_position)

        GroundPlane("/World/ground_plane")
        DomeLight("/World/DomeLight").set_intensities(1000)

        self.robot = UR10(robot_path="/World/ur10_robot", create_robot=True, attach_gripper=False)

        cube_shape = Cube(
            paths="/World/TargetCube",
            positions=self.target_position,
            orientations=[1, 0, 0, 0],
            sizes=1.0,
            scales=[0.05, 0.05, 0.05],
            colors="red",
        )

        self.target_cube = GeomPrim(paths=cube_shape.paths)

    def get_target_position(self) -> np.ndarray:
        """Get the current target cube position.

        Returns:
            Current target position [x, y, z].
        """
        if self.target_cube is not None:
            position, _ = self.target_cube.get_world_poses()
            return position.numpy().flatten()
        return np.array(self.target_position)

    def get_robot_end_effector_position(self) -> np.ndarray:
        """Get the current robot end effector position.

        Returns:
            Current end effector position [x, y, z].
        """
        if self.robot is not None:
            position, _ = self.robot.end_effector_link.get_world_poses()
            return position.numpy().flatten()
        return np.zeros(3)

    def target_reached(self, threshold: float = 0.05) -> bool:
        """Check if the end effector has reached the target.

        Args:
            threshold: Distance threshold to consider target reached.

        Returns:
            True if target is reached, False otherwise.
        """
        target_pos = self.get_target_position()
        ee_pos = self.get_robot_end_effector_position()
        distance = np.linalg.norm(target_pos - ee_pos)
        return distance < threshold

    def move_to_target(self, ik_method: str = "damped-least-squares") -> None:
        """Move the robot end effector towards the target position.

        Args:
            ik_method: The inverse kinematics method to use.
        """
        if self.robot is not None and self.target_cube is not None:
            target_pos = self.get_target_position()
            # Use the enhanced UR10Controller's set_end_effector_pose method
            self.robot.set_end_effector_pose(position=target_pos, orientation=[0.0, 1.0, 0.0, 0.0], ik_method=ik_method)

    def reset_robot(self) -> None:
        """Reset the robot to its default pose."""
        if self.robot is not None:
            self.robot.reset_to_default_pose()
