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

"""Provide gripper control functionality for articulation-based grippers with open and closed state management."""

from __future__ import annotations

import numpy as np
from isaacsim.core.api.controllers import ArticulationController
from isaacsim.core.prims import SingleArticulation
from isaacsim.core.utils.types import ArticulationAction


class ArticulationGripper(object):
    """Gripper controller for articulation-based grippers.

    Args:
        gripper_dof_names: List of DOF names for the gripper joints.
        gripper_open_position: Joint positions for open state.
        gripper_closed_position: Joint positions for closed state.

    """

    def __init__(
        self,
        gripper_dof_names: list,
        gripper_open_position: np.ndarray | None = None,
        gripper_closed_position: np.ndarray | None = None,
    ) -> None:
        self._articulation = None
        self._grippers_dof_names = gripper_dof_names
        self._grippers_dof_indices = [None] * len(self._grippers_dof_names)
        self._articulation_controller = None
        self._gripper_open_position = gripper_open_position
        self._gripper_closed_position = gripper_closed_position
        return

    @property
    def open_position(self) -> np.ndarray:
        """Get the gripper open joint positions.

        Returns:
            Array of joint positions for open state.

        """
        return self._gripper_open_position

    @property
    def closed_position(self) -> np.ndarray:
        """Get the gripper closed joint positions.

        Returns:
            Array of joint positions for closed state.

        """
        return self._gripper_closed_position

    @property
    def dof_indices(self) -> np.ndarray:
        """Get the gripper DOF indices.

        Returns:
            Array of DOF indices for the gripper joints.

        """
        return self._grippers_dof_indices

    def initialize(self, root_prim_path: str, articulation_controller: ArticulationController) -> None:
        """Initialize the gripper with an articulation.

        Args:
            root_prim_path: Root prim path of the articulation.
            articulation_controller: Controller for the articulation.

        Raises:
            Exception: If not all gripper DOF names could be resolved.

        """
        self._articulation = SingleArticulation(root_prim_path)
        for index in range(len(self._articulation.dof_names)):
            dof_name = self._articulation.dof_names[index]
            for j in range(len(self._grippers_dof_names)):
                if self._grippers_dof_names[j] == dof_name:
                    self._grippers_dof_indices[j] = index
        # make sure that all gripper dof names were resolved
        for i in range(len(self._grippers_dof_names)):
            if self._grippers_dof_indices[i] is None:
                raise Exception("Not all gripper dof names were resolved to dof handles and dof indices.")
        self._grippers_dof_indices = np.array(self._grippers_dof_indices)
        self._articulation_controller = articulation_controller
        return

    def set_positions(self, positions: np.ndarray) -> None:
        """Set the gripper joint positions.

        Args:
            positions: Target positions for the gripper joints.

        """
        self._articulation.set_joint_positions(positions, self._grippers_dof_indices)
        self._articulation_controller.apply_action(
            ArticulationAction(
                joint_positions=positions,
                joint_velocities=None,
                joint_efforts=None,
                joint_indices=self._grippers_dof_indices,
            )
        )
        return

    def get_positions(self) -> np.ndarray:
        """Get the current gripper joint positions.

        Returns:
            Array of current gripper joint positions.

        """
        # gripper_positions = np.zeros(len(self._grippers_dof_handles))
        # for i in range(len(self._grippers_dof_handles)):
        #     gripper_positions[i] = self._dc_interface.get_dof_position(self._grippers_dof_handles[i])
        # return gripper_positions
        return self._articulation.get_joint_positions(self._grippers_dof_indices)

    def get_velocities(self) -> np.ndarray:
        """Get the current gripper joint velocities.

        Returns:
            Array of current gripper joint velocities.

        """
        # gripper_velocities = np.zeros(len(self._grippers_dof_handles))
        # for i in range(len(self._grippers_dof_handles)):
        #     gripper_velocities[i] = self._dc_interface.get_dof_velocity(self._grippers_dof_handles[i])
        # return gripper_velocities
        return self._articulation.get_joint_velocities(self._grippers_dof_indices)

    def set_velocities(self, velocities: np.ndarray) -> None:
        """Set the gripper joint velocities.

        Args:
            velocities: Target velocities for the gripper joints.

        """
        # for i in range(len(self._grippers_dof_handles)):
        #     self._dc_interface.set_dof_velocity(self._grippers_dof_handles[i], velocities[i])
        self._articulation.set_joint_velocities(velocities, self._grippers_dof_indices)
        self._articulation_controller.apply_action(
            ArticulationAction(
                joint_positions=None,
                joint_velocities=velocities,
                joint_efforts=None,
                joint_indices=self._grippers_dof_indices,
            )
        )
        return

    def apply_action(self, action: ArticulationAction) -> None:
        """Apply an articulation action to the gripper.

        Args:
            action: The articulation action to apply.

        """
        action.joint_indices = self._grippers_dof_indices
        self._articulation_controller.apply_action(action)
        return
