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

"""Provides a wrapper class for computing robot kinematics that integrates with simulated robot articulations."""

from __future__ import annotations

from typing import Optional

import carb
import numpy as np
from isaacsim.core.api.articulations import ArticulationSubset
from isaacsim.core.prims import SingleArticulation
from isaacsim.core.utils.types import ArticulationAction
from isaacsim.robot_motion.motion_generation.kinematics_interface import KinematicsSolver


class ArticulationKinematicsSolver:
    """Wrapper class for computing robot kinematics in a way that is easily transferable to the simulated robot Articulation.  A KinematicsSolver.

    computes FK and IK at any frame, possibly only using a subset of joints available on the simulated robot.
    This wrapper simplifies computing the current position of the simulated robot's end effector, as well as wrapping an IK result in an ArticulationAction that is
    recognized by the robot Articulation.

    Args:
        robot_articulation: Initialized robot Articulation object representing the simulated USD robot.
        kinematics_solver: An instance of a class that implements the KinematicsSolver.
        end_effector_frame_name: The name of the robot's end effector frame. This frame must appear in
            kinematics_solver.get_all_frame_names().
    """

    def __init__(
        self, robot_articulation: SingleArticulation, kinematics_solver: KinematicsSolver, end_effector_frame_name: str
    ) -> None:
        self._robot_articulation = robot_articulation
        self._kinematics_solver = kinematics_solver
        self.set_end_effector_frame(end_effector_frame_name)
        self._joints_view = ArticulationSubset(robot_articulation, kinematics_solver.get_joint_names())
        return

    def compute_end_effector_pose(self, position_only: bool = False) -> tuple[np.array, np.array]:
        """Compute the pose of the robot end effector using the simulated robot's current joint positions.

        Args:
            position_only: If True, only the frame positions need to be calculated. The returned rotation may be
                left undefined.

        Returns:
            A tuple containing (position, rotation) where position is translation vector describing the translation
            of the robot end effector relative to the USD global frame (in stage units) and rotation is (3x3)
            rotation matrix describing the rotation of the frame relative to the USD stage global frame.
        """
        joint_positions = self._joints_view.get_joint_positions()
        if joint_positions is None:
            carb.log_error(
                "Attempted to compute forward kinematics for an uninitialized robot Articulation. Cannot get joint positions"
            )
            return None, None

        return self._kinematics_solver.compute_forward_kinematics(
            self._ee_frame, joint_positions, position_only=position_only
        )

    def compute_inverse_kinematics(
        self,
        target_position: np.array,
        target_orientation: Optional[np.array] = None,
        position_tolerance: Optional[float] = None,
        orientation_tolerance: Optional[float] = None,
    ) -> tuple[ArticulationAction, bool]:
        """Compute inverse kinematics for the end effector frame using the current robot position as a warm start.  The result is returned.

        in an articulation action that can be directly applied to the robot.

        Args:
            target_position: Target translation of the target frame (in stage units) relative to the USD stage origin.
            target_orientation: Target orientation of the target frame relative to the USD stage global frame.
            position_tolerance: l-2 norm of acceptable position error (in stage units) between the target and achieved
                translations.
            orientation_tolerance: Magnitude of rotation (in radians) separating the target orientation from the
                achieved orientation. orientation_tolerance is well defined for values between 0 and pi.

        Returns:
            A tuple containing (ik_result, success) where ik_result is an ArticulationAction that can be applied to
            the robot to move the end effector frame to the desired position and success indicates if solver
            converged successfully.
        """
        warm_start = self._joints_view.get_joint_positions()
        if warm_start is None:
            carb.log_error(
                "Attempted to compute inverse kinematics for an uninitialized robot Articulation.  Cannot get joint positions"
            )
            return None, False

        ik_result, succ = self._kinematics_solver.compute_inverse_kinematics(
            self._ee_frame, target_position, target_orientation, warm_start, position_tolerance, orientation_tolerance
        )

        return self._joints_view.make_articulation_action(ik_result, None), succ

    def set_end_effector_frame(self, end_effector_frame_name: str) -> None:
        """Set the name for the end effector frame. If the frame is not recognized by the internal KinematicsSolver.

        instance, an error will be thrown.

        Args:
            end_effector_frame_name: Name of the robot end effector frame.
        """
        if end_effector_frame_name not in self._kinematics_solver.get_all_frame_names():
            raise ValueError(
                f"Frame name '{end_effector_frame_name}' not recognized by KinematicsSolver. "
                f"Valid frames: {self._kinematics_solver.get_all_frame_names()}"
            )

        self._ee_frame = end_effector_frame_name

    def get_end_effector_frame(self) -> str:
        """Get the end effector frame name.

        Returns:
            Name of the end effector frame.
        """
        return self._ee_frame

    def get_joints_subset(self) -> ArticulationSubset:
        """A wrapper class for querying USD robot joint states in the order expected by the kinematics solver.

        Returns:
            A wrapper class for querying USD robot joint states in the order expected by the kinematics solver.
        """
        return self._joints_view

    def get_kinematics_solver(self) -> KinematicsSolver:
        """Get the underlying KinematicsSolver instance used by this class.

        Returns:
            A class that can solve forward and inverse kinematics for a specified robot.
        """
        return self._kinematics_solver
