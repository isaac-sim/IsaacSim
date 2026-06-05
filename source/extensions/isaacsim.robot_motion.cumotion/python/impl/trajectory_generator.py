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

"""Implementation of trajectory generation functionality for creating smooth, time-optimal robot trajectories."""

from __future__ import annotations

import cumotion
import numpy as np
import warp as wp

from .configuration_loader import CumotionRobot
from .cumotion_trajectory import CumotionTrajectory


class TrajectoryGenerator:
    """Trajectory generator for creating smooth, time-optimal trajectories (not collision-aware).

    This class provides utilities for generating smooth trajectories from discrete
    waypoints or path specifications. It handles conversion between configuration
    space and task space representations and generates time-optimal trajectories
    respecting joint velocity and acceleration limits.

    Args:
        cumotion_robot: Robot configuration containing kinematics and joint information.
        robot_joint_space: List of joint names defining the robot's joint space.

    Raises:
        ValueError: If cumotion controlled joints are not a subset of the robot_joint_space.

    Example:

        .. code-block:: python

            generator = TrajectoryGenerator(robot_config, joint_names)
            trajectory = generator.generate_trajectory_from_cspace_waypoints(
                waypoints=[[0, 0, 0], [1, 1, 1], [2, 2, 2]]
            )
    """

    def __init__(
        self,
        cumotion_robot: CumotionRobot,
        robot_joint_space: list[str],
    ) -> None:
        if not set(cumotion_robot.controlled_joint_names).issubset(set(robot_joint_space)):
            raise ValueError(
                f"Cumotion controlled joints {cumotion_robot.controlled_joint_names} are not a subset of the robot_joint_space {robot_joint_space}."
            )

        self._cumotion_robot = cumotion_robot
        self._robot_joint_space = robot_joint_space
        self._cspace_trajectory_generator = cumotion.create_cspace_trajectory_generator(cumotion_robot.kinematics)

    def get_cspace_trajectory_generator(self) -> cumotion.CSpaceTrajectoryGenerator:
        """Get the underlying cuMotion trajectory generator.

        Returns the cuMotion generator object, allowing users to modify parameters
        directly.

        Returns:
            The cuMotion configuration space trajectory generator.

        Example:

            .. code-block:: python

                gen = generator.get_cspace_trajectory_generator()
                gen.set_velocity_limits(np.array([0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5]))
        """
        return self._cspace_trajectory_generator

    def generate_trajectory_from_path_specification(
        self,
        path_specification: cumotion.CSpacePathSpec | cumotion.TaskSpacePathSpec | cumotion.CompositePathSpec,
        tool_frame_name: str | None = None,
        task_space_conversion_config: cumotion.TaskSpacePathConversionConfig | None = None,
        inverse_kinematics_config: cumotion.IkConfig | None = None,
    ) -> CumotionTrajectory | None:
        """Generate a trajectory from a cuMotion path specification.

        Converts the provided path specification (which may be in configuration space,
        task space, or composite) into a smooth time-optimal trajectory (not collision-aware). Task space
        paths are converted to configuration space using inverse kinematics.

        Args:
            path_specification: Path specification in cuMotion format (defined in base frame).
            tool_frame_name: Name of the tool frame for task space planning. Defaults to None,
                which uses the first tool frame in the robot description.
            task_space_conversion_config: Configuration for task space to configuration space
                conversion. Defaults to None.
            inverse_kinematics_config: Configuration for inverse kinematics solver. Defaults to None.

        Returns:
            Generated trajectory, or None if generation failed.

        Raises:
            RuntimeError: If the path specification has fewer than 2 waypoints.

        Example:

            .. code-block:: python

                path_spec = cumotion.create_cspace_path_spec(
                    initial_cspace_position=np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]),
                )
                path_spec.add_cspace_waypoint(np.array([1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0]))
                path_spec.add_cspace_waypoint(np.array([2.0, 2.0, 2.0, 2.0, 2.0, 2.0, 2.0]))
                trajectory = generator.generate_trajectory_from_path_specification(path_spec)
        """
        if tool_frame_name is None:
            tool_frame_names = self._cumotion_robot.robot_description.tool_frame_names()
            if not tool_frame_names:
                raise RuntimeError("No tool frames available in robot description and no tool_frame_name was provided.")
            tool_frame_name = tool_frame_names[0]

        kwargs = {}
        if task_space_conversion_config is not None:
            kwargs["task_space_path_conversion_config"] = task_space_conversion_config
        if inverse_kinematics_config is not None:
            kwargs["ik_config"] = inverse_kinematics_config

        # if our path specification is not cspace, then we must convert it:
        if isinstance(path_specification, cumotion.CompositePathSpec):
            c_space_path = cumotion.convert_composite_path_spec_to_cspace(
                path_specification,
                self._cumotion_robot.kinematics,
                tool_frame_name,
                **kwargs,
            )
        elif isinstance(path_specification, cumotion.TaskSpacePathSpec):
            c_space_path = cumotion.convert_task_space_path_spec_to_cspace(
                path_specification,
                self._cumotion_robot.kinematics,
                tool_frame_name,
                **kwargs,
            )
        else:
            c_space_path = path_specification

        if np.array(c_space_path.waypoints()).shape[0] < 2:
            raise RuntimeError(f"Cannot generate a trajectory with less than two waypoints.")

        # generate the trajectory from the cspace configuration:
        trajectory = self._cspace_trajectory_generator.generate_trajectory(c_space_path.waypoints())
        if trajectory is None:
            return None

        return CumotionTrajectory(
            trajectory=trajectory,
            cumotion_robot=self._cumotion_robot,
            robot_joint_space=self._robot_joint_space,
        )

    def generate_trajectory_from_cspace_waypoints(
        self,
        waypoints: wp.array | np.ndarray | list[list[float]],
        times: wp.array | np.ndarray | list[float] | None = None,
        interpolation_mode: cumotion.CSpaceTrajectoryGenerator.InterpolationMode | None = None,
    ) -> CumotionTrajectory | None:
        """Generate a trajectory from configuration space waypoints.

        Creates a smooth time-optimal trajectory passing through the specified joint
        configuration waypoints. Optionally, specific timing for each waypoint can be
        provided, or an interpolation mode can be specified.

        Args:
            waypoints: Array of joint configurations, shape (n_waypoints, n_joints).
            times: Optional time stamps for each waypoint. Defaults to None (automatic timing).
            interpolation_mode: Interpolation method to use. Defaults to None (cuMotion default).

        Returns:
            Generated trajectory, or None if generation failed.

        Raises:
            RuntimeError: If waypoints is not two-dimensional.
            RuntimeError: If fewer than 2 waypoints are provided.
            RuntimeError: If waypoint dimension doesn't match the number of controlled joints.
            RuntimeError: If times is not one-dimensional.
            RuntimeError: If the number of times doesn't match the number of waypoints.
            RuntimeError: If any time value is negative.
            RuntimeError: If times are not strictly increasing.

        Example:

            .. code-block:: python

                waypoints = [[0.0, -0.5, 0.0], [0.5, 0.0, 0.5], [1.0, 0.5, 1.0]]
                trajectory = generator.generate_trajectory_from_cspace_waypoints(waypoints)

                # With explicit timing
                waypoints = [[0, 0], [1, 1], [2, 2]]
                times = [0.0, 1.0, 2.0]
                trajectory = generator.generate_trajectory_from_cspace_waypoints(
                    waypoints, times
                )
        """
        # convert the waypoints to numpy:
        if isinstance(waypoints, wp.array):
            waypoints = waypoints.numpy()
        elif isinstance(waypoints, list):
            waypoints = np.array(waypoints)

        if waypoints.ndim != 2:
            raise RuntimeError(f"Waypoints must be a two-dimensional array of size [n_points, n_controlled_joints]")

        if waypoints.shape[0] < 2:
            raise RuntimeError(f"Cannot generate a trajectory with less than two waypoints.")

        correct_joint_count = waypoints.shape[1] == len(self._cumotion_robot.controlled_joint_names)
        if correct_joint_count is False:
            raise RuntimeError(f"Waypoints must control {len(self._cumotion_robot.controlled_joint_names)} joints.")

        # simplified case, no timing:
        if times is None:
            trajectory = self._cspace_trajectory_generator.generate_trajectory(waypoints)
            if trajectory is None:
                return None
            return CumotionTrajectory(
                trajectory=trajectory,
                cumotion_robot=self._cumotion_robot,
                robot_joint_space=self._robot_joint_space,
            )

        # convert times to numpy:
        if isinstance(times, wp.array):
            times = times.numpy()
        elif isinstance(times, list):
            times = np.array(times)

        if times.ndim != 1:
            raise RuntimeError(f"Time array must be one-dimensional.")

        if len(times) != len(waypoints):
            raise RuntimeError("The times and the waypoints must have the same number of entries.")

        for i, time in enumerate(times):
            if time < 0.0:
                raise RuntimeError(f"Time {i} is negative.  Times must be non-negative.")
            if i > 0 and times[i] <= times[i - 1]:
                raise RuntimeError(f"Time {i} is not greater than time {i-1}.  Times must be strictly increasing.")

        kwargs = {}
        if interpolation_mode is not None:
            kwargs["interpolation_mode"] = interpolation_mode

        trajectory = self._cspace_trajectory_generator.generate_time_stamped_trajectory(waypoints, times, **kwargs)
        if trajectory is None:
            return None

        return CumotionTrajectory(
            trajectory=trajectory,
            cumotion_robot=self._cumotion_robot,
            robot_joint_space=self._robot_joint_space,
        )
