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

"""Module providing cuMotion trajectory implementation for continuous-time robot motion in Isaac Sim."""

from __future__ import annotations

import cumotion
import isaacsim.robot_motion.experimental.motion_generation as mg
import warp as wp

from .configuration_loader import CumotionRobot


class CumotionTrajectory(mg.Trajectory):
    """Continuous-time trajectory for a robot using cuMotion.

    This class wraps a cuMotion trajectory and provides the Isaac Sim trajectory
    interface for querying robot states at specific times. All trajectories start
    at time 0.0 and end after their duration.

    Args:
        trajectory: cuMotion trajectory object.
        robot_joint_space: List of robot joint names in the joint space.
        cumotion_robot: Robot configuration containing joint names and descriptions.
        device: Warp device for computation. Defaults to None.

    Raises:
        ValueError: If cuMotion active joints are not a subset of the robot_joint_space.

    Example:

        .. code-block:: python

            trajectory = CumotionTrajectory(
                trajectory=cumotion_traj,
                robot_joint_space=joint_names,
                cumotion_robot=robot_config
            )
            state = trajectory.get_target_state(time=1.0)
    """

    def __init__(
        self,
        trajectory: cumotion.Trajectory,
        robot_joint_space: list[str],
        cumotion_robot: CumotionRobot,
        device: wp.Device | None = None,
    ) -> None:
        if not set(cumotion_robot.controlled_joint_names).issubset(set(robot_joint_space)):
            raise ValueError("Cumotion active joints are not a subset of the robot_joint_space.")

        # save the cumotion internal representation of the trajectory,
        # and the robot it is associated with.
        self._trajectory = trajectory
        self._cumotion_robot = cumotion_robot
        self._device = device
        self._robot_joint_space = robot_joint_space

    @property
    def duration(self) -> float:
        """Get the duration of the trajectory.

        Returns:
            Duration of the trajectory in seconds.
        """
        return self._trajectory.domain().span()

    def get_active_joints(self) -> list[str]:
        """Get the list of controlled joint names.

        Returns:
            List of joint names that are controlled in this trajectory.
        """
        return self._cumotion_robot.controlled_joint_names

    def get_target_state(self, time: float) -> mg.RobotState | None:
        """Get the target robot state at a specified time.

        Returns joint positions and velocities for the robot at the given time along
        the trajectory. If the time is outside the trajectory domain, None is returned.

        Args:
            time: Time in trajectory at which to return joint targets.

        Returns:
            Robot state containing joint positions and velocities, or None if time
            is outside the trajectory domain.

        Example:

        .. code-block:: python

            state = trajectory.get_target_state(time=2.5)
            if state is not None:
                print(state.joints.positions)
        """
        if (time < 0) or (time > self.duration):
            # return an empty RobotState.
            return None

        absolute_trajectory_time = time + self._trajectory.domain().lower
        position, velocity, _1, _2 = self._trajectory.eval_all(absolute_trajectory_time)

        # compute the position and velocity:
        position_wp = wp.from_numpy(position, dtype=wp.float32, device=self._device)
        velocity_wp = wp.from_numpy(velocity, dtype=wp.float32, device=self._device)

        return mg.RobotState(
            joints=mg.JointState.from_name(
                robot_joint_space=self._robot_joint_space,
                positions=(self._cumotion_robot.controlled_joint_names, position_wp),
                velocities=(self._cumotion_robot.controlled_joint_names, velocity_wp),
                efforts=None,
            )
        )
