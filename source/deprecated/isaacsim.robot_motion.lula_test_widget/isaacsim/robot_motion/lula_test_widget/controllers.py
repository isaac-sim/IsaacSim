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

"""Controllers for Lula-based robot motion control including inverse kinematics, trajectory execution, and path planning."""

from __future__ import annotations

from typing import Optional

import carb
import numpy as np
from isaacsim.core.api import objects
from isaacsim.core.api.controllers.base_controller import BaseController
from isaacsim.core.utils.types import ArticulationAction
from isaacsim.robot_motion.motion_generation.articulation_kinematics_solver import ArticulationKinematicsSolver
from isaacsim.robot_motion.motion_generation.articulation_trajectory import ArticulationTrajectory
from isaacsim.robot_motion.motion_generation.path_planner_visualizer import PathPlannerVisualizer


class LulaController(BaseController):
    """Base controller class for Lula-based robot motion control systems.

    This class serves as an abstract base for implementing various types of robot controllers that utilize
    the Lula motion generation framework. It provides a common interface for controllers that compute
    articulation actions based on target end-effector poses.

    The class defines the basic structure that all Lula controllers must follow, with a forward method
    that takes target end-effector position and orientation as inputs and returns an ArticulationAction.
    Concrete implementations include kinematics-based controllers, trajectory followers, and path planners.

    This base class is designed to be extended by specific controller implementations that provide different
    approaches to robot motion generation, such as inverse kinematics solving, trajectory execution, or
    motion planning with obstacle avoidance.
    """

    def __init__(self) -> None:
        pass

    def forward(
        self, target_end_effector_position: np.ndarray, target_end_effector_orientation: Optional[np.ndarray] = None
    ) -> ArticulationAction:
        """Computes control actions to reach the target end effector pose.

        Args:
            target_end_effector_position: Target position for the end effector in world coordinates.
            target_end_effector_orientation: Target orientation for the end effector.

        Returns:
            Articulation action containing joint commands to achieve the target pose.
        """
        return


class KinematicsController(LulaController):
    """A controller that uses inverse kinematics to compute joint positions for reaching target end-effector poses.

    This controller extends LulaController to provide inverse kinematics-based motion control for robotic articulations.
    It computes the joint configuration required to position the end-effector at a specified target pose using the
    provided ArticulationKinematicsSolver.

    The controller's forward method takes target end-effector position and orientation as input and returns an
    ArticulationAction containing the computed joint positions. If inverse kinematics computation fails, it logs a
    warning and returns an empty ArticulationAction.

    Args:
        name: Name identifier for the controller.
        art_kinematics: The articulation kinematics solver used for inverse kinematics computations.
    """

    def __init__(self, name: str, art_kinematics: ArticulationKinematicsSolver) -> None:
        BaseController.__init__(self, name)
        self._art_kinematics = art_kinematics

    def forward(
        self, target_end_effector_position: np.ndarray, target_end_effector_orientation: Optional[np.ndarray] = None
    ) -> ArticulationAction:
        """Computes inverse kinematics to generate joint actions for reaching the target end-effector pose.

        Args:
            target_end_effector_position: Target position for the end-effector in world coordinates.
            target_end_effector_orientation: Target orientation for the end-effector. If None, only position
                control is used.

        Returns:
            Articulation action with joint positions to reach the target pose. Returns empty action if
            inverse kinematics computation fails.
        """
        action, succ = self._art_kinematics.compute_inverse_kinematics(
            target_end_effector_position, target_end_effector_orientation
        )

        if succ:
            return action
        else:
            carb.log_warn("Failed to compute Inverse Kinematics")
            return ArticulationAction()


class TrajectoryController(LulaController):
    """A controller that executes pre-computed articulation trajectories.

    This controller plays back a sequence of joint actions from a pre-computed articulation trajectory at a fixed
    framerate of 60 FPS. It automatically handles the conversion from trajectory waypoints to individual
    ArticulationAction objects that can be applied to control robot articulations.

    The controller manages trajectory playback by:
    - Converting the trajectory into a sequence of actions at initialization
    - Sequentially returning each action when forward() is called
    - Handling joint position initialization for the first action
    - Returning the final action when the trajectory is complete

    Args:
        name: Name identifier for the controller.
        art_trajectory: The ArticulationTrajectory containing the pre-computed motion plan to execute.
    """

    def __init__(self, name: str, art_trajectory: ArticulationTrajectory) -> None:
        BaseController.__init__(self, name)
        self._art_trajectory = art_trajectory
        self._actions = self._art_trajectory.get_action_sequence(1 / 60)
        self._action_index = 0

    def forward(
        self, target_end_effector_position: np.ndarray, target_end_effector_orientation: Optional[np.ndarray] = None
    ) -> ArticulationAction:
        """Executes the next step in the pre-computed trajectory sequence.

        This method returns the next articulation action from the trajectory, handling initialization
        of joint positions on the first call and trajectory completion when all actions are consumed.

        Args:
            target_end_effector_position: Target position for the end effector (not used in trajectory
                execution but required for interface compatibility).
            target_end_effector_orientation: Target orientation for the end effector (not used in trajectory
                execution but required for interface compatibility).

        Returns:
            The next articulation action in the trajectory sequence.
        """
        # if active joints no the same as the size of the articulation, need to set active and passive joints separately
        robot_articulation = self._art_trajectory.get_robot_articulation()
        active_joints = self._art_trajectory.get_active_joints_subset()
        active_joint_indices = active_joints.joint_indices

        if self._action_index == 0:
            first_action = self._actions[0]
            desired_joint_positions = first_action.joint_positions
            current_joint_positions = robot_articulation.get_joint_positions()

            is_none_mask = desired_joint_positions is None
            desired_joint_positions[is_none_mask] = current_joint_positions[active_joint_indices][is_none_mask]

        elif self._action_index >= len(self._actions):
            desired_joint_positions = self._actions[-1].joint_positions
        else:
            desired_joint_positions = self._actions[self._action_index].joint_positions

        action = ArticulationAction(
            desired_joint_positions,
            np.zeros_like(desired_joint_positions),
            joint_indices=active_joint_indices,
        )
        self._action_index += 1

        return action


class PathPlannerController(LulaController):
    """A controller that uses path planning to generate smooth trajectories for robotic manipulation tasks.

    This controller leverages a path planner to compute collision-free paths from the current robot configuration
    to a target end-effector pose. It provides smooth motion execution by interpolating waypoints and managing
    trajectory timing. The controller can handle dynamic obstacles and supports both position and orientation
    constraints for the end-effector.

    The controller operates by generating a complete path plan when given a target pose, then executing the plan
    by stepping through waypoints at a controlled rate. Each waypoint is held for a specified number of frames
    before moving to the next, ensuring smooth and predictable motion.

    Args:
        name: Unique identifier for the controller instance.
        path_planner_visualizer: Visualizer that provides access to the underlying path planner and handles
            plan computation.
        cspace_interpolation_max_dist: Maximum distance between consecutive waypoints in configuration space
            during path interpolation.
        frames_per_waypoint: Number of simulation frames to spend at each waypoint before advancing to the next.
    """

    def __init__(
        self,
        name: str,
        path_planner_visualizer: PathPlannerVisualizer,
        cspace_interpolation_max_dist: float = 0.5,
        frames_per_waypoint: int = 30,
    ) -> None:
        BaseController.__init__(self, name)

        self._path_planner_visualizer = path_planner_visualizer
        self._path_planner = path_planner_visualizer.get_path_planner()

        self._cspace_interpolation_max_dist = cspace_interpolation_max_dist
        self._frames_per_waypoint = frames_per_waypoint

        self._plan = None

        self._frame_counter = 1

    def make_new_plan(
        self, target_end_effector_position: np.ndarray, target_end_effector_orientation: Optional[np.ndarray] = None
    ) -> None:
        """Generates a new motion plan to the specified end effector target.

        Args:
            target_end_effector_position: Target position for the end effector.
            target_end_effector_orientation: Target orientation for the end effector.
        """
        self._path_planner.set_end_effector_target(target_end_effector_position, target_end_effector_orientation)
        self._path_planner.update_world()
        self._plan = self._path_planner_visualizer.compute_plan_as_articulation_actions(
            max_cspace_dist=self._cspace_interpolation_max_dist
        )
        if self._plan is None or self._plan == []:
            carb.log_warn("No plan could be generated to target pose: " + str(target_end_effector_position))

    def forward(
        self, target_end_effector_position: np.ndarray, target_end_effector_orientation: Optional[np.ndarray] = None
    ) -> ArticulationAction:
        """Executes the motion plan by returning the next action in the sequence.

        Args:
            target_end_effector_position: Target position for the end effector.
            target_end_effector_orientation: Target orientation for the end effector.

        Returns:
            The next articulation action in the plan sequence.
        """
        if self._plan is None:
            # This will only happen the first time the forward function is used
            self.make_new_plan(target_end_effector_position, target_end_effector_orientation)

        if len(self._plan) == 0:
            # The plan is completed; return null action to remain in place
            self._frame_counter = 1
            return ArticulationAction()

        if self._frame_counter % self._frames_per_waypoint != 0:
            # Stop at each waypoint in the plan for self._frames_per_waypoint frames
            self._frame_counter += 1
            return self._plan[0]
        else:
            self._frame_counter += 1
            return self._plan.pop(0)

    def add_obstacle(self, obstacle: objects, static: bool = False) -> None:
        """Adds an obstacle to the path planner for collision avoidance.

        Args:
            obstacle: The obstacle object to add.
            static: Whether the obstacle is static or dynamic.
        """
        self._path_planner.add_obstacle(obstacle, static)

    def remove_obstacle(self, obstacle: objects) -> None:
        """Removes an obstacle from the path planner.

        Args:
            obstacle: The obstacle object to remove.
        """
        self._path_planner.remove_obstacle(obstacle)

    def reset(self) -> None:
        """Resets the path planner controller state and clears the current plan."""
        # PathPlannerController will make one plan per reset
        self._path_planner.reset()
        self._plan = None
        self._frame_counter = 1
