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

"""Provides utilities for converting trajectory objects into discrete articulation actions for robot control."""

from __future__ import annotations

import carb
import numpy as np
from isaacsim.core.api.articulations import ArticulationSubset
from isaacsim.core.prims import SingleArticulation
from isaacsim.core.utils.types import ArticulationAction

from .trajectory import Trajectory


class ArticulationTrajectory:
    """Wrapper class which takes in a Trajectory object and converts the output to discrete ArticulationActions that may be sent to the provided robot Articulation.

    Args:
        robot_articulation: Initialized robot Articulation object representing the simulated USD robot.
        trajectory: An instance of a class that implements the Trajectory interface.
        physics_dt: Duration of a physics step in Isaac Sim (typically 1/60 s).
    """

    def __init__(self, robot_articulation: SingleArticulation, trajectory: Trajectory, physics_dt: float) -> None:
        self._articulation = robot_articulation
        self._trajectory = trajectory
        self._physics_dt = physics_dt

        self._active_joints_view = ArticulationSubset(robot_articulation, trajectory.get_active_joints())

    def get_action_at_time(self, time: float) -> ArticulationAction:
        """Get an ArticulationAction that will send the robot to the desired position/velocity at a given time in the provided Trajectory.

        Args:
            time: Time between the start and end times in the provided Trajectory. If the time is out of bounds, an error will be thrown.

        Returns:
            ArticulationAction that may be passed directly to the robot Articulation to send it to the desired position/velocity at the given time.
        """
        if time < self._trajectory.start_time:
            message = f"Provided time {time} is before the start time {self._trajectory.start_time} of the Trajectory"
            carb.log_error(message)
            raise ValueError(message)

        if time > self._trajectory.end_time:
            message = f"Provided time {time} is after the end time {self._trajectory.end_time} of the Trajectory"
            carb.log_error(message)
            raise ValueError(message)

        position_target, velocity_target = self._trajectory.get_joint_targets(time)

        return self._active_joints_view.make_articulation_action(position_target, velocity_target)

    def get_action_sequence(self, timestep: float = None) -> list[ArticulationAction]:
        """Get a sequence of ArticulationActions which sample the entire Trajectory according to the provided timestep.

        Args:
            timestep: Timestep used for sampling the provided Trajectory.
                A vlue of 1/60, for example, returns ArticulationActions that represent the desired position/velocity of
                the robot at 1/60 second intervals. I.e. a one second trajectory with timestep=1/60 would result in 60 ArticulationActions.
                When not provided, the framerate of Isaac Sim is used.

        Returns:
            Sequence of ArticulationActions that may be passed to the robot Articulation to produce the desired trajectory.
        """
        if timestep is None:
            timestep = self._physics_dt
        actions = []
        for t in np.arange(self._trajectory.start_time, self._trajectory.end_time, timestep):
            actions.append(self.get_action_at_time(t))
        return actions

    def get_trajectory_duration(self) -> float:
        """Returns the duration of the provided Trajectory.

        Returns:
            Duration of the provided trajectory.
        """
        return self._trajectory.end_time - self._trajectory.start_time

    def get_active_joints_subset(self) -> ArticulationSubset:
        """Get view into active joints.

        Returns:
            Robot states for active joints in an order compatible with the TrajectoryGenerator.
        """
        return self._active_joints_view

    def get_robot_articulation(self) -> SingleArticulation:
        """Get the robot Articulation.

        Returns:
            Articulation object describing the robot.
        """
        return self._articulation

    def get_trajectory(self) -> Trajectory:
        """Trajectory object used by this ArticulationTrajectory.

        Returns:
            The underlying Trajectory object.
        """
        return self._trajectory
