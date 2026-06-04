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

"""Provides a kinematic solver interface for computing robot forward and inverse kinematics."""

from __future__ import annotations

from typing import Optional

import numpy as np

from .world_interface import WorldInterface


class KinematicsSolver(WorldInterface):
    """A limited interface for computing robot kinematics that includes forward and inverse kinematics.

    This interface omits more advanced kinematics such as Jacobians, as they are not required for most use-cases.

    This interface inherits from the WorldInterface to standardize the inputs to collision-aware IK solvers, but it is not necessary for
    all implementations to implement the WorldInterface.  See KinematicsSolver.supports_collision_avoidance()
    """

    def __init__(self) -> None:
        pass

    def set_robot_base_pose(self, robot_positions: np.array, robot_orientation: np.array) -> None:
        """Update position of the robot base. This will be used to compute kinematics relative to the USD stage origin.

        Args:
            robot_positions: (3 x 1) translation vector describing the translation of the robot base relative to the USD stage origin.
                The translation vector should be specified in the units of the USD stage
            robot_orientation: (4 x 1) quaternion describing the orientation of the robot base relative to the USD stage global frame
        """

    def get_joint_names(self) -> list[str]:
        """Return a list containing the names of all joints in the given kinematic structure. The order of this list.

        determines the order in which the joint positions are expected in compute_forward_kinematics(joint_positions,...) and
        the order in which they are returned in compute_inverse_kinematics().

        Returns:
            Names of all joints in the robot.
        """
        return []

    def get_all_frame_names(self) -> list[str]:
        """Return a list of all the frame names in the given kinematic structure.

        Returns:
            All frame names in the kinematic structure. Any of which can be used to compute forward or inverse kinematics.
        """
        return []

    def compute_forward_kinematics(
        self, frame_name: str, joint_positions: np.array, position_only: Optional[bool] = False
    ) -> tuple[np.array, np.array]:
        """Compute the position of a given frame in the robot relative to the USD stage global frame.

        Args:
            frame_name: Name of robot frame on which to calculate forward kinematics
            joint_positions: Joint positions for the joints returned by get_joint_names()
            position_only: If True, only the frame positions need to be calculated and the returned rotation may be left undefined.

        Returns:
            A tuple of (frame_positions, frame_rotation) where frame_positions is a (3x1) vector describing the translation
            of the frame relative to the USD stage origin and frame_rotation is a (3x3) rotation matrix describing the
            rotation of the frame relative to the USD stage global frame.
        """
        return np.zeros(3), np.eye(3)

    def compute_inverse_kinematics(
        self,
        frame_name: str,
        target_positions: np.array,
        target_orientation: Optional[np.array] = None,
        warm_start: Optional[np.array] = None,
        position_tolerance: Optional[float] = None,
        orientation_tolerance: Optional[float] = None,
    ) -> tuple[np.array, bool]:
        """Compute joint positions such that the specified robot frame will reach the desired translations and rotations.

        Args:
            frame_name: Name of the target frame for inverse kinematics
            target_positions: Target translation of the target frame (in stage units) relative to the USD stage origin
            target_orientation: Target orientation of the target frame relative to the USD stage global frame.
            warm_start: A starting position that will be used when solving the IK problem.
            position_tolerance: L-2 norm of acceptable position error (in stage units) between the target and achieved translations.
            orientation_tolerance: Magnitude of rotation (in radians) separating the target orientation from the achieved orientation.
                orientation_tolerance is well defined for values between 0 and pi.

        Returns:
            A tuple of (joint_positions, success) where joint_positions are in the order specified by get_joint_names()
            which result in the target frame achieving the desired position and success is True if the solver converged to
            a solution within the given tolerances.
        """
        return np.empty()

    def supports_collision_avoidance(self) -> bool:
        """Returns a bool describing whether the inverse kinematics support collision avoidance. If the policy does not support collision.

        avoidance, none of the obstacle add/remove/enable/disable functions need to be implemented.

        Returns:
            True if the IK solver will avoid any obstacles that have been added.
        """
        return False
