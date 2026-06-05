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

"""Interface for implementing collision-aware motion policies that dynamically move robots to targets."""

from __future__ import annotations

import numpy as np
from isaacsim.robot_motion.motion_generation.world_interface import WorldInterface


class MotionPolicy(WorldInterface):
    """Interface for implementing a MotionPolicy: a collision-aware algorithm for dynamically moving a robot to a target. The MotionPolicy interface inherits.

    from the WorldInterface class. A MotionPolicy can be passed to an ArticulationMotionPolicy to streamline moving the simulated robot.
    """

    def __init__(self) -> None:
        pass

    def set_robot_base_pose(self, robot_translation: np.array, robot_orientation: np.array) -> None:
        """Update position of the robot base.

        Args:
            robot_translation: (3 x 1) translation vector describing the translation of the robot base relative to the USD stage origin.
                The translation vector should be specified in the units of the USD stage
            robot_orientation: (4 x 1) quaternion describing the orientation of the robot base relative to the USD stage global frame
        """

    def compute_joint_targets(
        self,
        active_joint_positions: np.array,
        active_joint_velocities: np.array,
        watched_joint_positions: np.array,
        watched_joint_velocities: np.array,
        frame_duration: float,
    ) -> tuple[np.array, np.array]:
        """Compute position and velocity targets for the next frame given the current robot state.

            Position and velocity targets are used in Isaac Sim to generate forces using the PD equation
            kp*(joint_position_targets-joint_positions) + kd*(joint_velocity_targets-joint_velocities).

        Args:
            active_joint_positions: current positions of joints specified by get_active_joints()
            active_joint_velocities: current velocities of joints specified by get_active_joints()
            watched_joint_positions: current positions of joints specified by get_watched_joints()
            watched_joint_velocities: current velocities of joints specified by get_watched_joints()
            frame_duration: duration of the physics frame

        Returns:
            A tuple containing (joint position targets, joint velocity targets) for the active robot joints for the next frame.
        """
        return active_joint_positions, np.zeros_like(active_joint_velocities)

    def get_active_joints(self) -> list[str]:
        """Names of active joints directly controlled by this MotionPolicy.

            Some articulated robot joints may be ignored by some policies. E.g., the gripper of the Franka arm is not used
            to follow targets, and the RMPflow config files excludes the joints in the gripper from the list of articulated
            joints.

        Returns:
            Names of active joints. The order of joints in this list determines the order in which a
            MotionPolicy expects joint states to be specified in functions like compute_joint_targets(active_joint_positions,...).
        """
        return []

    def get_watched_joints(self) -> list[str]:
        """Names of watched joints whose position/velocity matters to the MotionPolicy, but are not directly controlled.

            e.g. A MotionPolicy may control a robot arm on a mobile robot. The joint states in the rest of the robot directly affect the position of the arm, but they are not actively controlled by this MotionPolicy.

        Returns:
            Names of joints that are being watched by this MotionPolicy. The order of joints in this list determines the order in which a
            MotionPolicy expects joint states to be specified in functions like compute_joint_targets(...,watched_joint_positions,...).
        """
        return []

    def set_cspace_target(self, active_joint_targets: np.array) -> None:
        """Set configuration space target for the robot.

        Args:
            active_joint_targets: Desired configuration for the robot as (m x 1) vector where m is the number of active
                joints.
        """

    def set_end_effector_target(self, target_translation: object = None, target_orientation: object = None) -> None:
        """Set end effector target.

        Args:
            target_translation: Translation vector (3x1) for robot end effector.
                Target translation should be specified in the same units as the USD stage, relative to the stage origin.
            target_orientation: Quaternion of desired rotation for robot end effector relative to USD stage global frame
        """
