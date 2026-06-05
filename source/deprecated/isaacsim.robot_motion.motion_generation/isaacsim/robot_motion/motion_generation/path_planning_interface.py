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

"""Interface for implementing path planning algorithms that generate collision-free paths from starting configurations to target poses in configuration or task space."""

from __future__ import annotations

import numpy as np
from isaacsim.robot_motion.motion_generation.world_interface import WorldInterface


class PathPlanner(WorldInterface):
    """Interface for implementing a PathPlanner: An algorithm that outputs a series of configuration space waypoints, which.

    when linearly interpolated, produce a collision-free path from a starting c-space pose to a c-space or task-space target pose.
    """

    def __init__(self) -> None:
        pass

    def set_robot_base_pose(self, robot_translation: np.array, robot_orientation: np.array) -> None:
        """Set the position of the robot base. Computed paths will assume that the robot base remains stationary.

        Args:
            robot_translation: (3 x 1) translation vector describing the translation of the robot base relative to the USD stage origin.
                The translation vector should be specified in the units of the USD stage
            robot_orientation: (4 x 1) quaternion describing the orientation of the robot base relative to the USD stage global frame
        """

    def compute_path(self, active_joint_positions: np.array, watched_joint_positions: np.array) -> np.array:
        """Compute a set of c-space waypoints, which when linearly interpolated,.

            produce a collision-free path from a starting c-space pose to a c-space or task-space target pose.

        Args:
            active_joint_positions: current positions of joints specified by get_active_joints()
            watched_joint_positions: current positions of joints specified by get_watched_joints()

        Returns:
            An (N x m) sequence of joint positions for the active joints in the robot where N is the path length and
                m is the number of active joints in the robot. If no plan is found, or no target positions have been set, None is returned
        """
        return active_joint_positions

    def get_active_joints(self) -> list[str]:
        """Active joints are directly controlled by this PathPlanner.

            Some articulated robot joints may be ignored by some policies. E.g., the gripper of the Franka arm is not used
            to follow targets, and the RMPflow config files excludes the joints in the gripper from the list of articulated
            joints.

        Returns:
            Names of active joints. The order of joints in this list determines the order in which a
                PathPlanner expects joint states to be specified in functions like compute_path(active_joint_positions,...)
        """
        return []

    def get_watched_joints(self) -> list[str]:
        """Watched joints are joints whose position matters to the PathPlanner, but are not directly controlled.

            e.g. A robot may have a watched joint in the middle of its kinematic chain. Watched joints will be assumed
            to remain watched during the rollout of a path.

        Returns:
            Names of joints that are being watched by this PathPlanner. The order of joints in this list determines the order in which a
                PathPlanner expects joint states to be specified in functions like compute_path(...,watched_joint_positions,...)
        """
        return []

    def set_cspace_target(self, active_joint_targets: np.array) -> None:
        """Set configuration space target for the robot.

        Args:
            active_joint_targets: Desired configuration for the robot as (m x 1) vector where m is the number of active
                joints
        """

    def set_end_effector_target(self, target_translation: object, target_orientation: object = None) -> None:
        """Set end effector target.

        Args:
            target_translation: Translation vector (3x1) for robot end effector.
                Target translation should be specified in the same units as the USD stage, relative to the stage origin.
            target_orientation: Quaternion of desired rotation for robot end effector relative to USD stage global frame
        """
