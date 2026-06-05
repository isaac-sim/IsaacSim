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

"""Provides trajectory optimization capabilities using cuMotion's Trajectory Optimizer algorithm for generating smooth, collision-free robot trajectories."""

from __future__ import annotations

import pathlib
import warnings

import cumotion
import numpy as np
import warp as wp

from .configuration_loader import CumotionRobot
from .cumotion_trajectory import CumotionTrajectory
from .cumotion_world_interface import CumotionWorldInterface


class TrajectoryOptimizer:
    """Trajectory optimizer using cuMotion's Trajectory Optimizer algorithm.

    This class provides trajectory optimization capabilities.
    It generates smooth, collision-free trajectories to configuration space or task space targets by optimizing
    over trajectory costs including smoothness, collision avoidance, and goal achievement.

    Args:
        cumotion_robot: Robot configuration containing kinematics and joint information.
        robot_joint_space: List of joint names defining the robot's joint space.
        cumotion_world_interface: World interface providing collision geometry.
        tool_frame: Name of the tool frame for task space planning. Defaults to None,
            which uses the first tool frame defined in the robot description.
        trajectory_optimizer_config_filename: Path to the YAML configuration file.
            If a relative path is provided, it is resolved relative to
            cumotion_robot.directory. If an absolute path is provided,
            it is used as-is. Defaults to None (uses default parameters).

    Example:

        .. code-block:: python

            optimizer = TrajectoryOptimizer(
                cumotion_robot=robot_config,
                robot_joint_space=joint_names,
                cumotion_world_interface=world_interface,
            )
            # See cuMotion documentation for creating target specifications
            trajectory = optimizer.plan_to_goal(q_initial, cspace_target)
    """

    def __init__(
        self,
        cumotion_robot: CumotionRobot,
        robot_joint_space: list[str],
        cumotion_world_interface: CumotionWorldInterface,
        tool_frame: str | None = None,
        trajectory_optimizer_config_filename: pathlib.Path | str | None = None,
    ) -> None:
        # if there is no tool_frame, we will take the default (first one):
        if not tool_frame:
            tool_frame_names = cumotion_robot.robot_description.tool_frame_names()
            if not tool_frame_names:
                raise RuntimeError("No tool frames available in robot description and no tool_frame was provided.")
            tool_frame = tool_frame_names[0]

        if not set(cumotion_robot.controlled_joint_names).issubset(set(robot_joint_space)):
            raise ValueError(
                f"Cumotion controlled joints {cumotion_robot.controlled_joint_names} are not a subset of the robot_joint_space {robot_joint_space}."
            )

        if not trajectory_optimizer_config_filename:
            trajectory_optimizer_config = cumotion.create_default_trajectory_optimizer_config(
                robot_description=cumotion_robot.robot_description,
                tool_frame_name=tool_frame,
                world_view=cumotion_world_interface.world_view,
            )
        else:
            config_path = pathlib.Path(trajectory_optimizer_config_filename)
            if config_path.is_absolute():
                full_config_path = config_path
            else:
                full_config_path = cumotion_robot.directory / config_path
            trajectory_optimizer_config = cumotion.create_trajectory_optimizer_config_from_file(
                trajectory_optimizer_config_file=full_config_path,
                robot_description=cumotion_robot.robot_description,
                tool_frame_name=tool_frame,
                world_view=cumotion_world_interface.world_view,
            )

        # store the cumotion world interface:
        self._cumotion_world_interface = cumotion_world_interface
        self._trajectory_optimizer_config = trajectory_optimizer_config
        self._cumotion_robot = cumotion_robot
        self._world_view = cumotion_world_interface.world_view
        self._robot_joint_space = robot_joint_space

    def get_cumotion_robot(self) -> CumotionRobot:
        """Get the robot configuration.

        Returns:
            The robot configuration used by this optimizer.
        """
        return self._cumotion_robot

    def get_trajectory_optimizer_config(self) -> cumotion.TrajectoryOptimizerConfig:
        """Get the trajectory optimizer configuration.

        Returns the underlying cuMotion configuration object, allowing users to modify
        optimizer parameters before planning.

        Returns:
            The cuMotion trajectory optimizer configuration object.

        Example:

            .. code-block:: python

                config = optimizer.get_trajectory_optimizer_config()
                config.set_param("trajopt/pbo/num_iterations", cumotion.TrajectoryOptimizerConfig.ParamValue(100))
        """
        return self._trajectory_optimizer_config

    def plan_to_goal(
        self,
        initial_cspace_position: wp.array | np.ndarray | list[float],
        goal: (
            cumotion.TrajectoryOptimizer.CSpaceTarget
            | cumotion.TrajectoryOptimizer.TaskSpaceTargetGoalset
            | cumotion.TrajectoryOptimizer.TaskSpaceTarget
        ),
    ) -> CumotionTrajectory | None:
        """Plan a trajectory to a specified goal.

        Generates a smooth, collision-free trajectory from the initial configuration
        to the specified goal. The goal can be a configuration space target, task space
        target, or task space goalset.

        Args:
            initial_cspace_position: Initial joint configuration.
            goal: Target goal specification (CSpaceTarget, TaskSpaceTarget, or TaskSpaceTargetGoalset). Note that all goals must be defined in the base frame.

        Returns:
            Optimized trajectory if successful, None if planning failed.

        Raises:
            ValueError: If goal type is not one of the supported types.

        Example:

            .. code-block:: python

                # Configuration space target
                q_initial = [0.0, -0.5, 0.0, -2.0, 0.0, 1.5, 0.75]
                # See cuMotion documentation for constructing CSpaceTarget,
                # TaskSpaceTarget, or TaskSpaceTargetGoalset objects
                trajectory = optimizer.plan_to_goal(q_initial, target)
        """
        wp.synchronize()
        trajectory_optimizer = cumotion.create_trajectory_optimizer(self._trajectory_optimizer_config)

        self._world_view.update()

        if isinstance(initial_cspace_position, wp.array):
            initial_cspace_position = initial_cspace_position.numpy().flatten()
        elif isinstance(initial_cspace_position, list):
            initial_cspace_position = np.array(initial_cspace_position).flatten()
        elif isinstance(initial_cspace_position, np.ndarray):
            initial_cspace_position = initial_cspace_position.flatten()
        else:
            raise ValueError(
                f"Initial cspace position must be a wp.array, list, or np.ndarray. Got {type(initial_cspace_position)}."
            )

        if not (initial_cspace_position.size == len(self._cumotion_robot.controlled_joint_names)):
            raise ValueError(
                f"Initial cspace position must be of length: {len(self._cumotion_robot.controlled_joint_names)}."
            )

        if isinstance(goal, cumotion.TrajectoryOptimizer.CSpaceTarget):
            result = trajectory_optimizer.plan_to_cspace_target(initial_cspace_position, goal)
        elif isinstance(goal, cumotion.TrajectoryOptimizer.TaskSpaceTargetGoalset):
            result = trajectory_optimizer.plan_to_task_space_goalset(initial_cspace_position, goal)
        elif isinstance(goal, cumotion.TrajectoryOptimizer.TaskSpaceTarget):
            result = trajectory_optimizer.plan_to_task_space_target(initial_cspace_position, goal)
        else:
            raise ValueError(
                f"Goal must be one of: CSpaceTarget, TaskSpaceTargetGoalset or TaskSpaceTarget. See Cumotion documentation."
            )

        status = result.status()
        if status != cumotion.TrajectoryOptimizer.Results.Status.SUCCESS:
            warnings.warn(f"Trajectory optimizer failed with status: {status.name}.")
            return None

        # otherwise, return the trajectory:
        return CumotionTrajectory(
            trajectory=result.trajectory(),
            cumotion_robot=self._cumotion_robot,
            robot_joint_space=self._robot_joint_space,
        )
