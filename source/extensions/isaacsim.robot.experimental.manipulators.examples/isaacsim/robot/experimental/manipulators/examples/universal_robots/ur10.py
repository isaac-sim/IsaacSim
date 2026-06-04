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

"""UR10 experimental robot controller with inverse kinematics and gripper control."""

from __future__ import annotations

import isaacsim.core.experimental.utils.stage as stage_utils
import numpy as np
import warp as wp
from isaacsim.core.experimental.prims import Articulation, RigidPrim
from isaacsim.core.experimental.utils.impl.transform import quaternion_conjugate, quaternion_multiplication
from isaacsim.robot.surface_gripper import _surface_gripper as surface_gripper
from isaacsim.storage.native import get_assets_root_path


class UR10(Articulation):
    """UR10 robot controller with inverse kinematics and gripper control.

    This class inherits from Articulation and provides high-level control commands for the UR10 robot,
    including inverse kinematics for end-effector positioning and gripper control.
    It can either use an existing robot path or create a new one from USD assets.

    Args:
        robot_path: USD path where the robot should be created or exists.
        create_robot: Whether to create a new robot from USD assets.
        end_effector_link: The end effector rigid body link. If None, creates from robot_path.
        attach_gripper: Whether to attach a gripper to the robot.
        usd_path: Optional path to a custom UR10 USD file. When provided the
            ``Short_Suction`` gripper variant is **not** applied automatically
            (the custom file is assumed to include its own gripper setup).
            When ``None`` (default), the packaged UR10 asset is used.

    Raises:
        ValueError: If create_robot is False but no robot exists at robot_path.
    """

    def __init__(
        self,
        robot_path: str = "/World/robot",
        create_robot: bool = True,
        end_effector_link: RigidPrim | None = None,
        attach_gripper: bool = True,
        usd_path: str | None = None,
    ) -> None:
        if create_robot:
            if usd_path is None:
                usd_path = get_assets_root_path() + "/Isaac/Robots/UniversalRobots/ur10/ur10.usd"
                variants = [("Gripper", "Short_Suction")] if attach_gripper else []
            else:
                variants = []
            robot_prim = stage_utils.add_reference_to_stage(
                usd_path=usd_path,
                path=robot_path,
                variants=variants,
            )

        # Initialize the parent Articulation class
        super().__init__(robot_path)

        # Set up end effector link
        if end_effector_link is None:
            self.end_effector_link = RigidPrim(f"{robot_path}/ee_link")
        else:
            self.end_effector_link = end_effector_link

        self._attach_gripper = attach_gripper
        self._surface_gripper_path: str | None = None
        self._surface_gripper_iface = None
        if attach_gripper:
            end_effector_path = f"{robot_path}/ee_link"
            self._surface_gripper_path = f"{end_effector_path}/SurfaceGripper"
            self._surface_gripper_iface = surface_gripper.acquire_surface_gripper_interface()

        if create_robot:
            # Set robot default state with good working pose
            # Joint positions: [shoulder_pan, shoulder_lift, elbow, wrist_1, wrist_2, wrist_3]
            # This pose positions the robot in a good working configuration
            default_joint_positions = [
                -np.pi / 2,  # shoulder_pan: -90 degrees
                -np.pi / 2,  # shoulder_lift: -90 degrees
                -np.pi / 2,  # elbow: -90 degrees
                -np.pi / 2,  # wrist_1: -90 degrees
                np.pi / 2,  # wrist_2: 90 degrees
                0,  # wrist_3: 0 degrees
            ]
            self.set_default_state(dof_positions=default_joint_positions)

        # Get the end effector link index for Jacobian calculations
        self.end_effector_link_index = self.get_link_indices("ee_link").list()[0]

        # UR10 has 6 arm DOFs
        self._num_arm_dofs = 6

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
            The computed delta joint positions [N, 6] for the arm joints.

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

        # Compute delta DOF positions using selected method
        if method == "singular-value-decomposition":
            min_singular_value = method_cfg.get("min_singular_value", 1e-5)
            U, S, Vh = np.linalg.svd(jacobian_end_effector)
            inv_s = np.where(min_singular_value < S, 1.0 / S, np.zeros_like(S))
            pseudoinverse = np.swapaxes(Vh, 1, 2)[:, :, :6] @ np.diagflat(inv_s) @ np.swapaxes(U, 1, 2)
            return (scale * pseudoinverse @ error).squeeze(-1)
        elif method == "pseudoinverse":
            pseudoinverse = np.linalg.pinv(jacobian_end_effector)
            return (scale * pseudoinverse @ error).squeeze(-1)
        elif method == "transpose":
            transpose = np.swapaxes(jacobian_end_effector, 1, 2)
            return (scale * transpose @ error).squeeze(-1)
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
            where current_dof_positions are current joint positions [N, 6 or 7] (6 for arm, +1 if gripper),
            current_end_effector_position is current end effector position [N, 3], and
            current_end_effector_orientation is current end effector orientation [N, 4] as quaternion.
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

        # Convert target position/orientation to proper shape if needed
        position = np.asarray(position)
        orientation = np.asarray(orientation)
        if position.ndim == 1:
            position = position.reshape(1, -1)
        if orientation.ndim == 1:
            orientation = orientation.reshape(1, -1)

        # Get the Jacobian matrix for the end-effector
        jacobian_matrices = self.get_jacobian_matrices().numpy()
        jacobian_end_effector = jacobian_matrices[:, self.end_effector_link_index - 1, :, :6]  # UR10 has 6 DOF

        # Compute delta DOF positions for arm
        delta_dof_positions = self.differential_inverse_kinematics(
            jacobian_end_effector=jacobian_end_effector,
            current_position=current_end_effector_position,
            current_orientation=current_end_effector_orientation,
            goal_position=position,
            goal_orientation=orientation,
            method=ik_method,
        )

        # Set DOF targets for arm only (joints 0-5 for UR10)
        dof_position_targets = current_dof_positions[:, :6] + delta_dof_positions
        self.set_dof_position_targets(dof_position_targets, dof_indices=list(range(6)))

    def get_actual_dof_count(self) -> int:
        """Get the actual number of DOFs this robot has.

        Returns:
            Total number of DOFs.
        """
        try:
            dof_positions = self.get_dof_positions()
            return dof_positions.shape[1] if len(dof_positions.shape) > 1 else len(dof_positions)
        except Exception:
            return self._num_arm_dofs  # Fallback to arm DOFs only

    def get_downward_orientation(self) -> np.ndarray:
        """Get the standard downward-facing orientation for the end effector.

        UR10 ee_link frame differs from Franka: [0, 1, 0, 0] (180° about X) yields
        a horizontal gripper facing the user. This returns a quaternion that tilts
        the approach axis so the gripper points down (world -Y in Isaac Sim).

        Returns:
            Quaternion [w, x, y, z] representing downward-facing orientation,
            shape (1, 4).
        """
        return [0.0, 0.70710678, 0.0, -0.70710678]

    def open_gripper(self) -> None:
        """Open the gripper (release suction)."""
        iface, path = self._surface_gripper_iface, self._surface_gripper_path
        if iface is None or path is None:
            return
        iface.open_gripper(path)

    def close_gripper(self) -> None:
        """Close the gripper (activate suction to hold an object)."""
        iface, path = self._surface_gripper_iface, self._surface_gripper_path
        if iface is None or path is None:
            return
        if iface.get_gripper_status(path) != surface_gripper.GripperStatus.Closed:
            iface.close_gripper(path)

    def reset_to_default_pose(self) -> None:
        """Reset the robot to its default pose with open gripper if attached."""
        # Get the actual DOF count to create the right sized array
        total_dofs = self.get_actual_dof_count()

        # Base arm positions (always 6 DOFs for UR10)
        arm_positions = [0, 0, 0, 0, 0, 0]

        if total_dofs > self._num_arm_dofs:
            gripper_positions = [0.0] * (total_dofs - self._num_arm_dofs)
            default_positions = arm_positions + gripper_positions
        else:
            default_positions = arm_positions

        print(f"Resetting robot with {total_dofs} DOFs, using positions: {len(default_positions)} values")

        self.set_dof_positions(default_positions)
        self.set_dof_position_targets(default_positions)
        if self._surface_gripper_path is not None:
            self.open_gripper()
