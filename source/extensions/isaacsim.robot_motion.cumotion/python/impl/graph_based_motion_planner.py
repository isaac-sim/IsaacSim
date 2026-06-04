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

"""Graph-based motion planner module using cuMotion for collision-free path planning in robotics applications."""

from __future__ import annotations

import pathlib

import cumotion
import numpy as np
import warp as wp
from isaacsim.robot_motion.experimental.motion_generation import Path

from .configuration_loader import CumotionRobot
from .cumotion_world_interface import CumotionWorldInterface
from .utils import isaac_sim_to_cumotion_pose, isaac_sim_to_cumotion_translation


class GraphBasedMotionPlanner:
    """Graph-based motion planner using cuMotion for collision-free path planning.

    This planner uses sampling-based algorithms (RRT variants) to find collision-free paths
    for robotic manipulators. It supports planning to configuration space targets, task space
    pose targets, and translation-only targets.

    Args:
        cumotion_robot: Robot configuration containing the robot description
            (URDF/XRDF) and configuration directory.
        cumotion_world_interface: World interface providing collision geometry and world view.
        tool_frame: Name of the tool frame to use for task space planning. Defaults to None,
            which uses the first tool frame defined in the robot description.
        graph_planner_config_filename: Path to a YAML configuration file for the planner
            parameters. If a relative path is provided, it is resolved relative to
            cumotion_robot.directory. If an absolute path is provided,
            it is used as-is. Defaults to None, which uses default planner parameters.

    Example:

        .. code-block:: python

            from isaacsim.robot_motion.cumotion import (
                CumotionRobot,
                CumotionWorldInterface,
                GraphBasedMotionPlanner,
            )

            robot_config = CumotionRobot(franka_directory)
            world_interface = CumotionWorldInterface(robot_config)
            planner = GraphBasedMotionPlanner(robot_config, world_interface)

            # Plan to a joint configuration target
            path = planner.plan_to_cspace_target(q_initial, q_target)
    """

    def __init__(
        self,
        cumotion_robot: CumotionRobot,
        cumotion_world_interface: CumotionWorldInterface,
        tool_frame: str | None = None,
        graph_planner_config_filename: pathlib.Path | str | None = None,
    ) -> None:
        # if there is no tool_frame, we will take the default (first one):
        if not tool_frame:
            tool_frame_names = cumotion_robot.robot_description.tool_frame_names()
            if not tool_frame_names:
                raise RuntimeError("No tool frames available in robot description and no tool_frame was provided.")
            tool_frame = tool_frame_names[0]

        if not graph_planner_config_filename:
            motion_planner_config = cumotion.create_default_motion_planner_config(
                robot_description=cumotion_robot.robot_description,
                tool_frame_name=tool_frame,
                world_view=cumotion_world_interface.world_view,
            )
        else:
            config_path = pathlib.Path(graph_planner_config_filename)
            if config_path.is_absolute():
                full_config_path = config_path
            else:
                full_config_path = cumotion_robot.directory / config_path
            motion_planner_config = cumotion.create_motion_planner_config_from_file(
                motion_planner_config_file=full_config_path,
                robot_description=cumotion_robot.robot_description,
                tool_frame_name=tool_frame,
                world_view=cumotion_world_interface.world_view,
            )

        # store the cumotion world interface:
        self._cumotion_world_interface = cumotion_world_interface
        self._motion_planner_config = motion_planner_config
        self._cumotion_robot = cumotion_robot

    def get_cumotion_robot(self) -> CumotionRobot:
        """Get the robot configuration.

        Returns:
            The robot configuration used by this planner.
        """
        return self._cumotion_robot

    def get_graph_planner_config(self) -> cumotion.MotionPlannerConfig:
        """Get the motion planner configuration.

        Returns the underlying cuMotion configuration object, allowing users to modify
        planner parameters before planning.

        Returns:
            The cuMotion motion planner configuration object.

        Example:

            .. code-block:: python

                config = planner.get_graph_planner_config()
                config.set_param("max_iterations", cumotion.MotionPlannerConfig.ParamValue(1000))
        """
        return self._motion_planner_config

    def plan_to_cspace_target(
        self,
        q_initial: wp.array | np.ndarray | list[float],
        q_final: wp.array | np.ndarray | list[float],
    ) -> Path | None:
        """Plan a collision-free path to a target joint configuration.

        Uses graph-based planning (RRT) to find a collision-free path from the initial
        joint configuration to the target joint configuration in configuration space.

        Args:
            q_initial: Initial joint configuration.
            q_final: Target joint configuration.

        Returns:
            Path object if a path was found, None otherwise.

        Example:

            .. code-block:: python

                q_initial = [0.0, -0.5, 0.0, -2.0, 0.0, 1.5, 0.75]
                q_target = [1.0, -0.3, 0.2, -1.8, 0.1, 1.2, 0.5]
                path = planner.plan_to_cspace_target(q_initial, q_target)
                if path is not None:
                    trajectory = path.to_minimal_time_joint_trajectory()
        """
        wp.synchronize()
        # make sure we have the correct types:
        if isinstance(q_initial, wp.array):
            q_initial = q_initial.numpy()

        q_initial = np.array(q_initial, dtype=np.float64).flatten()

        if isinstance(q_final, wp.array):
            q_final = q_final.numpy()

        q_final = np.array(q_final, dtype=np.float64).flatten()

        if not (q_initial.size == q_final.size == len(self._cumotion_robot.controlled_joint_names)):
            raise RuntimeError(
                f"Initial and final joint positions must be of length: {len(self._cumotion_robot.controlled_joint_names)}."
            )

        # create the planner with whatever the current parameters happen to be:
        planner = cumotion.create_motion_planner(config=self._motion_planner_config)

        # update the world view:
        self._cumotion_world_interface.world_view.update()
        planning_result = planner.plan_to_cspace_target(q_initial, q_final)

        if not planning_result.path_found:
            return None

        # return Path type, such that user can directly convert to a trajectory.
        return Path(planning_result.path)

    def plan_to_pose_target(
        self,
        q_initial: wp.array | np.ndarray | list[float],
        position: wp.array | np.ndarray | list[float],
        orientation: wp.array | np.ndarray | list[float],
    ) -> Path | None:
        """Plan a collision-free path to a target pose (position + orientation) defined relative to the world frame.

        Uses JtRRT (Jacobian-transpose RRT) to find a collision-free path from the initial
        joint configuration to a task space pose target. The orientation is fully constrained.

        Args:
            q_initial: Initial joint configuration.
            position: Target position as [x, y, z] relative to world origin.
            orientation: Target orientation as quaternion [w, x, y, z] or 3x3 rotation matrix relative to world frame.

        Returns:
            Path object if a path was found, None otherwise.

        Raises:
            ValueError: If orientation is not a valid quaternion (4 elements) or 3x3 rotation matrix.

        Example:

            .. code-block:: python

                q_initial = [0.0, -0.5, 0.0, -2.0, 0.0, 1.5, 0.75]
                position = [0.5, 0.2, 0.4]
                orientation = [1.0, 0.0, 0.0, 0.0]  # wxyz quaternion (identity)
                path = planner.plan_to_pose_target(q_initial, position, orientation)
        """
        wp.synchronize()
        # make sure we have the correct types for q_initial:
        if isinstance(q_initial, wp.array):
            q_initial = q_initial.numpy()

        q_initial = np.array(q_initial, dtype=np.float64).flatten()

        if not (q_initial.size == len(self._cumotion_robot.controlled_joint_names)):
            raise RuntimeError(
                f"Initial joint positions must be of length: {len(self._cumotion_robot.controlled_joint_names)}."
            )

        position_world_to_base, quaternion_world_to_base = (
            self._cumotion_world_interface.get_world_to_robot_base_transform()
        )

        pose_base_target = isaac_sim_to_cumotion_pose(
            position_world_to_target=position,
            orientation_world_to_target=orientation,
            position_world_to_base=position_world_to_base,
            orientation_world_to_base=quaternion_world_to_base,
        )

        # create the planner with whatever the current parameters happen to be:
        self._cumotion_world_interface.world_view.update()
        planner = cumotion.create_motion_planner(config=self._motion_planner_config)

        planning_result = planner.plan_to_pose_target(q_initial, pose_base_target)

        if not planning_result.path_found:
            return None

        # return Path type, such that user can directly convert to a trajectory.
        return Path(planning_result.path)

    def plan_to_translation_target(
        self,
        q_initial: wp.array | np.ndarray | list[float],
        translation_target: wp.array | np.ndarray | list[float],
    ) -> Path | None:
        """Plan a collision-free path to a target position (translation only) defined relative to the world frame.

        Uses JtRRT (Jacobian-transpose RRT) to find a collision-free path from the initial
        joint configuration to a task space translation target. The end-effector orientation
        is unconstrained, allowing the planner more flexibility in finding a valid path.

        Args:
            q_initial: Initial joint configuration.
            translation_target: Target position as [x, y, z] relative to world frame origin.

        Returns:
            Path object if a path was found, None otherwise.

        Example:

            .. code-block:: python

                q_initial = [0.0, -0.5, 0.0, -2.0, 0.0, 1.5, 0.75]
                translation_target = [0.5, 0.2, 0.4]
                path = planner.plan_to_translation_target(q_initial, translation_target)
        """
        wp.synchronize()
        # make sure we have the correct types for q_initial:
        if isinstance(q_initial, wp.array):
            q_initial = q_initial.numpy()

        q_initial = np.array(q_initial, dtype=np.float64).flatten()

        if not (q_initial.size == len(self._cumotion_robot.controlled_joint_names)):
            raise RuntimeError(
                f"Initial joint positions must be of length: {len(self._cumotion_robot.controlled_joint_names)}."
            )

        position_world_to_base, quaternion_world_to_base = (
            self._cumotion_world_interface.get_world_to_robot_base_transform()
        )

        translation_base_target = isaac_sim_to_cumotion_translation(
            position_world_to_target=translation_target,
            position_world_to_base=position_world_to_base,
            orientation_world_to_base=quaternion_world_to_base,
        )

        # create the planner with whatever the current parameters happen to be:
        planner = cumotion.create_motion_planner(config=self._motion_planner_config)

        self._cumotion_world_interface.world_view.update()
        planning_result = planner.plan_to_translation_target(q_initial, translation_base_target)

        if not planning_result.path_found:
            return None

        # return Path type, such that user can directly convert to a trajectory.
        return Path(planning_result.path)
