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

"""Provides wrapper class for running motion policies on simulated robots."""

from __future__ import annotations

import carb
from isaacsim.core.api.articulations import ArticulationSubset
from isaacsim.core.deprecation_manager import import_module
from isaacsim.core.prims import SingleArticulation
from isaacsim.core.utils.types import ArticulationAction

from .motion_policy_interface import MotionPolicy

torch = import_module("torch")


class ArticulationMotionPolicy:
    """Wrapper class for running MotionPolicy on simulated robots.

    Args:
        robot_articulation: An initialized robot Articulation object.
        motion_policy: An instance of a class that implements the MotionPolicy interface.
        default_physics_dt: Default physics step size to use when computing actions. A MotionPolicy computes a target
            position/velocity for the next frame of the simulation using the provided physics dt to know how far in the
            future that will be. Isaac Sim can be run with a constant or variable physics framerate. When not specified
            on an individual frame, the dt of the frame is assumed to be the provided default value.
    """

    def __init__(
        self, robot_articulation: SingleArticulation, motion_policy: MotionPolicy, default_physics_dt: float = 1 / 60.0
    ) -> None:

        self.physics_dt = default_physics_dt
        self._robot_articulation = robot_articulation

        self.motion_policy = motion_policy

        self._articulation_controller = self._robot_articulation.get_articulation_controller()

        self._active_joints_view = ArticulationSubset(robot_articulation, motion_policy.get_active_joints())
        self._watched_joints_view = ArticulationSubset(robot_articulation, motion_policy.get_watched_joints())

        self._default_physics_dt = default_physics_dt

    def move(self, physics_dt: float = None) -> None:
        """Use underlying MotionPolicy to compute and apply joint targets to the robot over the next frame.

        Args:
            physics_dt: Physics dt to use on this frame to calculate the next action. This overrides
                the default_physics_dt argument, but does not change the default on future calls.
        """
        action = self.get_next_articulation_action(physics_dt=physics_dt)
        self._articulation_controller.apply_action(action)

    def get_next_articulation_action(self, physics_dt: float = None) -> ArticulationAction:
        """Use underlying MotionPolicy to compute joint targets for the robot over the next frame.

        Args:
            physics_dt: Physics dt to use on this frame to calculate the next action. This overrides
                the default_physics_dt argument, but does not change the default on future calls.

        Returns:
            Desired position/velocity target for the robot in the next frame.
        """
        if physics_dt is None:
            physics_dt = self._default_physics_dt

        joint_positions, joint_velocities = (
            self._active_joints_view.get_joint_positions(),
            self._active_joints_view.get_joint_velocities(),
        )
        watched_joint_positions, watched_joint_velocities = (
            self._watched_joints_view.get_joint_positions(),
            self._watched_joints_view.get_joint_velocities(),
        )

        if joint_positions is None:
            carb.log_error(
                "Attempted to compute an action, but the robot Articulation has not been initialized.  Cannot get joint positions or velocities."
            )

        # convert to numpy if torch tensor
        if isinstance(joint_positions, torch.Tensor):
            joint_positions = joint_positions.cpu().numpy()
        if isinstance(joint_velocities, torch.Tensor):
            joint_velocities = joint_velocities.cpu().numpy()
        if isinstance(watched_joint_positions, torch.Tensor):
            watched_joint_positions = watched_joint_positions.cpu().numpy()
        if isinstance(watched_joint_velocities, torch.Tensor):
            watched_joint_velocities = watched_joint_velocities.cpu().numpy()

        position_targets, velocity_targets = self.motion_policy.compute_joint_targets(
            joint_positions, joint_velocities, watched_joint_positions, watched_joint_velocities, physics_dt
        )

        return self._active_joints_view.make_articulation_action(position_targets, velocity_targets)

    def get_active_joints_subset(self) -> ArticulationSubset:
        """Get view into active joints.

        Returns:
            Returns robot states for active joints in an order compatible with the MotionPolicy.
        """
        return self._active_joints_view

    def get_watched_joints_subset(self) -> ArticulationSubset:
        """Get view into watched joints.

        Returns:
            Returns robot states for watched joints in an order compatible with the MotionPolicy.
        """
        return self._watched_joints_view

    def get_robot_articulation(self) -> SingleArticulation:
        """Get the underlying Articulation object representing the robot.

        Returns:
            Articulation object representing the robot.
        """
        return self._robot_articulation

    def get_motion_policy(self) -> MotionPolicy:
        """Get MotionPolicy that is being used to compute ArticulationActions.

        Returns:
            MotionPolicy being used to compute ArticulationActions.
        """
        return self.motion_policy

    def get_default_physics_dt(self) -> float:
        """Get the default value of the physics dt that is used to compute actions when none is provided.

        Returns:
            Default physics dt.
        """
        return self._default_physics_dt

    def set_default_physics_dt(self, physics_dt: float) -> None:
        """Set the default value of the physics dt that is used to compute actions when none is provided.

        Args:
            physics_dt: Default physics dt.
        """
        self._default_physics_dt = physics_dt
