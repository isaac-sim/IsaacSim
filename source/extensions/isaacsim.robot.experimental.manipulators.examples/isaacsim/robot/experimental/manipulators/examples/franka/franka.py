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

"""Franka experimental robot controller with inverse kinematics and gripper control."""

from __future__ import annotations

import isaacsim.core.experimental.utils.stage as stage_utils
import numpy as np
import warp as wp
from isaacsim.core.experimental.prims import Articulation, RigidPrim
from isaacsim.core.experimental.utils.impl.transform import quaternion_conjugate, quaternion_multiplication
from isaacsim.storage.native import get_assets_root_path


class Franka(Articulation):
    """Franka robot controller with inverse kinematics and gripper control.

    This class inherits from Articulation and provides high-level control commands for the Franka robot,
    including inverse kinematics for end-effector positioning and gripper control.
    It can either use an existing robot path or create a new one from USD assets.

    Args:
        robot_path: USD path where the robot should be created or exists.
        create_robot: Whether to create a new robot from USD assets.
        end_effector_link: The end effector rigid body link. If None, creates from robot_path.

    Raises:
        ValueError: If create_robot is False but no robot exists at robot_path.
    """

    def __init__(
        self,
        robot_path: str = "/World/robot",
        create_robot: bool = True,
        end_effector_link: RigidPrim | None = None,
    ) -> None:
        if create_robot:
            # Load Franka Panda robot from USD asset with specific gripper and mesh variants
            robot_prim = stage_utils.add_reference_to_stage(
                usd_path=get_assets_root_path() + "/Isaac/Robots/FrankaRobotics/FrankaPanda/franka.usd",
                path=robot_path,
                variants=[("Gripper", "AlternateFinger"), ("Mesh", "Performance")],
            )

        # Initialize the parent Articulation class
        super().__init__(robot_path)

        # Set up end effector link
        if end_effector_link is None:
            self.end_effector_link = RigidPrim(f"{robot_path}/panda_hand")
        else:
            self.end_effector_link = end_effector_link

        if create_robot:
            # Set robot default state with OPEN gripper
            # Joint positions: [joint1, joint2, joint3, joint4, joint5, joint6, joint7, finger1, finger2]
            # Last two values (0.04, 0.04) represent open gripper finger positions
            self.set_default_state(dof_positions=[0.012, -0.568, 0.0, -2.811, 0.0, 3.037, 0.741, 0.04, 0.04])

        self.end_effector_link_index = self.get_link_indices("panda_hand").list()[0]

        # Default gripper positions
        self.gripper_open_position = [0.04, 0.04]
        self.gripper_closed_position = [0.0, 0.0]

    def differential_inverse_kinematics(
        self,
        jacobian_end_effector: np.ndarray,
        current_position: np.ndarray,
        current_orientation: np.ndarray,
        goal_position: np.ndarray,
        goal_orientation: np.ndarray | None = None,
        method: str = "damped-least-squares",
        method_cfg: dict[str, float] = None,
    ) -> np.ndarray:
        """Compute the differential inverse kinematics (dIK) for a given end-effector Jacobian.

        This function calculates the change in joint positions (delta_q) needed to move the
        end-effector from its current pose (current_position, current_orientation) to a
        desired pose (goal_position, goal_orientation).

        Args:
            jacobian_end_effector: The Jacobian matrix of the end-effector link.
            current_position: The current position of the end-effector.
            current_orientation: The current orientation of the end-effector as a quaternion [w, x, y, z].
            goal_position: The desired position of the end-effector.
            goal_orientation: The desired orientation of the end-effector as a quaternion [w, x, y, z].
                If None, the goal orientation is assumed to be the same as the current orientation.
            method: The method to use for computing the inverse kinematics.
                Options: "singular-value-decomposition", "pseudoinverse", "transpose", "damped-least-squares".
            method_cfg: Configuration for the selected method.

        Returns:
            The computed delta joint positions [N, 7] for the arm joints.

        Raises:
            ValueError: If an invalid IK method is specified.
        """
        if method_cfg is None:
            method_cfg = {"scale": 1.0, "damping": 0.05, "min_singular_value": 1e-5}

        scale = method_cfg.get("scale", 1.0)
        # Compute velocity error
        goal_orientation = current_orientation if goal_orientation is None else goal_orientation

        # Convert numpy arrays to warp arrays for quaternion operations
        goal_quat_wp = wp.from_numpy(goal_orientation, dtype=wp.float32)
        current_quat_wp = wp.from_numpy(current_orientation, dtype=wp.float32)

        # Compute quaternion difference using warp functions
        current_quat_conjugate_wp = quaternion_conjugate(current_quat_wp)
        q_wp = quaternion_multiplication(goal_quat_wp, current_quat_conjugate_wp)
        q_np = q_wp.numpy()  # Convert back to numpy for further processing

        error = np.expand_dims(
            np.concatenate([goal_position - current_position, q_np[:, 1:] * np.sign(q_np[:, [0]])], axis=-1), axis=2
        )
        # Compute delta DOF positions
        # - Adaptive Singular Value Decomposition (SVD)
        if method == "singular-value-decomposition":
            min_singular_value = method_cfg.get("min_singular_value", 1e-5)
            U, S, Vh = np.linalg.svd(jacobian_end_effector)
            inv_s = np.where(min_singular_value < S, 1.0 / S, np.zeros_like(S))
            pseudoinverse = np.swapaxes(Vh, 1, 2)[:, :, :6] @ np.diagflat(inv_s) @ np.swapaxes(U, 1, 2)
            return (scale * pseudoinverse @ error).squeeze(-1)
        # - Moore-Penrose pseudoinverse
        elif method == "pseudoinverse":
            pseudoinverse = np.linalg.pinv(jacobian_end_effector)
            return (scale * pseudoinverse @ error).squeeze(-1)
        # - Transpose of matrix
        elif method == "transpose":
            transpose = np.swapaxes(jacobian_end_effector, 1, 2)
            return (scale * transpose @ error).squeeze(-1)
        # - Damped Least-Squares
        elif method == "damped-least-squares":
            damping = method_cfg.get("damping", 0.05)
            transpose = np.swapaxes(jacobian_end_effector, 1, 2)
            lmbda = np.eye(jacobian_end_effector.shape[1]) * (damping**2)
            return (scale * transpose @ np.linalg.inv(jacobian_end_effector @ transpose + lmbda) @ error).squeeze(-1)
        else:
            raise ValueError(f"Invalid IK method: {method}")

    def get_current_state(self) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
        """Get current robot state including DOF positions and end effector pose.

        Returns:
            A tuple containing (current_dof_positions, current_end_effector_position, current_end_effector_orientation)
            where current_dof_positions are current joint positions [N, 9], current_end_effector_position is current
            end effector position [N, 3], and current_end_effector_orientation is current end effector orientation [N, 4]
            as quaternion.
        """
        current_dof_positions = self.get_dof_positions().numpy()
        current_end_effector_position, current_end_effector_orientation = self.end_effector_link.get_world_poses()
        current_end_effector_position = current_end_effector_position.numpy()
        current_end_effector_orientation = current_end_effector_orientation.numpy()

        return current_dof_positions, current_end_effector_position, current_end_effector_orientation

    def set_end_effector_pose(
        self,
        position: np.ndarray,
        orientation: np.ndarray,
        ik_method: str = "damped-least-squares",
    ) -> None:
        """Set the end effector to a specific pose (position and orientation).

        This method uses inverse kinematics to move the end effector to the target pose.
        If no orientation is provided, uses a default downward-facing orientation.

        Args:
            position: Target position [x, y, z] for the end effector.
            orientation: Target orientation as quaternion [w, x, y, z].
                If None, uses downward-facing orientation.
            ik_method: The inverse kinematics method to use.

        Example:

        .. code-block:: python

            # Set pose with default downward orientation
            >>> robot.set_end_effector_pose([0.5, 0.0, 0.3])

            # Set pose with specific orientation
            >>> robot.set_end_effector_pose(
            ...     position=[0.5, 0.0, 0.3],
            ...     orientation=[0.0, 1.0, 0.0, 0.0]
            ... )
        """
        # Get current robot state
        current_dof_positions, current_end_effector_position, current_end_effector_orientation = (
            self.get_current_state()
        )

        position = np.asarray(position)
        orientation = np.asarray(orientation)
        if position.ndim == 1:
            position = position.reshape(1, -1)
        if orientation.ndim == 1:
            orientation = orientation.reshape(1, -1)

        # Get the Jacobian matrix for the end-effector
        jacobian_matrices = self.get_jacobian_matrices().numpy()
        jacobian_end_effector = jacobian_matrices[:, self.end_effector_link_index - 1, :, :7]

        # Compute delta DOF positions for arm
        delta_dof_positions = self.differential_inverse_kinematics(
            jacobian_end_effector=jacobian_end_effector,
            current_position=current_end_effector_position,
            current_orientation=current_end_effector_orientation,
            goal_position=position,
            goal_orientation=orientation,
            method=ik_method,
        )

        # Set DOF targets for arm only (joints 0-6)
        dof_position_targets = current_dof_positions[:, :7] + delta_dof_positions
        self.set_dof_position_targets(dof_position_targets, dof_indices=list(range(7)))

    def open_gripper(self) -> None:
        """Open the gripper to the default open position."""
        self.set_dof_position_targets(self.gripper_open_position, dof_indices=[7, 8])

    def close_gripper(self) -> None:
        """Close the gripper to the default closed position."""
        self.set_dof_position_targets(self.gripper_closed_position, dof_indices=[7, 8])

    def set_gripper_position(self, position: np.ndarray) -> None:
        """Set gripper to a specific position.

        Args:
            position: Gripper position [finger1, finger2] where 0.0 is closed and 0.04 is open.
        """
        if position.ndim == 1:
            position = position.reshape(1, -1)
        self.set_dof_position_targets(position, dof_indices=[7, 8])

    def get_downward_orientation(self) -> np.ndarray:
        """Get the standard downward-facing orientation for the end effector.

        Returns:
            Quaternion [w, x, y, z] representing downward-facing orientation.
        """
        return [0.0, 1.0, 0.0, 0.0]

    def reset_to_default_pose(self) -> None:
        """Reset the robot to its default pose with open gripper."""
        default_positions = [0.012, -0.568, 0.0, -2.811, 0.0, 3.037, 0.741, 0.04, 0.04]
        self.set_dof_positions(default_positions)
        self.set_dof_position_targets(default_positions)
