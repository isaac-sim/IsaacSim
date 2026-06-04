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

"""Provides high level functions for controlling parallel grippers with two fingers through joint position control."""

from collections.abc import Callable

import numpy as np
import omni.kit.app
from isaacsim.core.utils.types import ArticulationAction
from isaacsim.robot.manipulators.grippers.gripper import Gripper


class ParallelGripper(Gripper):
    """Provides high level functions to set/ get properties and actions of a parallel gripper.

    (a gripper that has two fingers).

    Args:
        end_effector_prim_path: Prim path of the Prim that corresponds to the gripper root/ end effector.
        joint_prim_names: The left finger joint prim name and the right finger joint prim name respectively.
        joint_opened_positions: Joint positions of the left finger joint and the right finger joint respectively when
            opened.
        joint_closed_positions: Joint positions of the left finger joint and the right finger joint respectively when
            closed.
        action_deltas: Deltas to apply for finger joint positions when opening or closing the gripper.
        use_mimic_joints: Whether to use mimic joints. If True, only the drive joint is used.
    """

    def __init__(
        self,
        end_effector_prim_path: str,
        joint_prim_names: list[str],
        joint_opened_positions: np.ndarray,
        joint_closed_positions: np.ndarray,
        action_deltas: np.ndarray = None,
        use_mimic_joints: bool = False,
    ) -> None:
        Gripper.__init__(self, end_effector_prim_path=end_effector_prim_path)
        self._joint_prim_names = joint_prim_names
        self._joint_dof_indicies = np.array([None, None])
        self._joint_opened_positions = joint_opened_positions
        self._joint_closed_positions = joint_closed_positions
        self._get_joint_positions_func = None
        self._set_joint_positions_func = None
        self._action_deltas = action_deltas
        self._articulation_num_dofs = None
        self._use_mimic_joints = use_mimic_joints
        return

    @property
    def joint_opened_positions(self) -> np.ndarray:
        """Joint positions of the left finger joint and the right finger joint respectively when opened.

        Returns:
            Joint positions of the left finger joint and the right finger joint respectively when opened.
        """
        return self._joint_opened_positions

    @property
    def joint_closed_positions(self) -> np.ndarray:
        """Joint positions of the left finger joint and the right finger joint respectively when closed.

        Returns:
            Joint positions of the left finger joint and the right finger joint respectively when closed.
        """
        return self._joint_closed_positions

    @property
    def joint_dof_indicies(self) -> np.ndarray:
        """Joint DOF indices in the articulation of the left finger joint and the right finger joint respectively.

        Returns:
            Joint DOF indices in the articulation of the left finger joint and the right finger joint respectively.
        """
        return self._joint_dof_indicies

    @property
    def joint_prim_names(self) -> list[str]:
        """The left finger joint prim name and the right finger joint prim name respectively.

        Returns:
            The left finger joint prim name and the right finger joint prim name respectively.
        """
        return self._joint_prim_names

    @property
    def active_joint_indices(self) -> list[int]:
        """Indices of active joints based on the use_mimic_joints setting.

        Returns:
            List of active joint indices.
        """
        if self._use_mimic_joints:
            return [self._joint_dof_indicies[0]]
        return [self._joint_dof_indicies[0], self._joint_dof_indicies[1]]

    @property
    def active_joint_count(self) -> int:
        """Number of active joints based on the use_mimic_joints setting.

        Returns:
            Number of active joints (1 or 2).
        """
        return 1 if self._use_mimic_joints else 2

    def initialize(
        self,
        articulation_apply_action_func: Callable,
        get_joint_positions_func: Callable,
        set_joint_positions_func: Callable,
        dof_names: list,
        physics_sim_view: omni.physics.tensors.SimulationView = None,
    ) -> None:
        """Create a physics simulation view if not passed and creates a rigid prim view using physX tensor api.

        This needs to be called after each hard reset (i.e stop + play on the timeline) before interacting with any
        of the functions of this class.

        Args:
            articulation_apply_action_func: apply_action function from the Articulation class.
            get_joint_positions_func: get_joint_positions function from the Articulation class.
            set_joint_positions_func: set_joint_positions function from the Articulation class.
            dof_names: dof names from the Articulation class.
            physics_sim_view: current physics simulation view.

        Raises:
            Exception: If not all gripper dof names were resolved to dof handles and dof indices.
        """
        Gripper.initialize(self, physics_sim_view=physics_sim_view)
        self._get_joint_positions_func = get_joint_positions_func
        self._articulation_num_dofs = len(dof_names)

        # Find joint indices
        for index, dof_name in enumerate(dof_names):
            if dof_name == self._joint_prim_names[0]:
                self._joint_dof_indicies[0] = index
            elif not self._use_mimic_joints and dof_name == self._joint_prim_names[1]:
                self._joint_dof_indicies[1] = index

        # Validate that required joints were found
        required_joints = 1 if self._use_mimic_joints else 2
        indices_found = sum(1 for idx in self._joint_dof_indicies[:required_joints] if idx is not None)
        if indices_found != required_joints:
            raise Exception("Not all gripper dof names were resolved to dof handles and dof indices.")

        self._articulation_apply_action_func = articulation_apply_action_func
        current_joint_positions = get_joint_positions_func()

        if self._default_state is None:
            self._default_state = np.array([current_joint_positions[idx] for idx in self.active_joint_indices])

        self._set_joint_positions_func = set_joint_positions_func
        return

    def open(self) -> None:
        """Applies actions to the articulation that opens the gripper (ex: to release an object held)."""
        self._articulation_apply_action_func(self.forward(action="open"))
        return

    def close(self) -> None:
        """Applies actions to the articulation that closes the gripper (ex: to hold an object)."""
        self._articulation_apply_action_func(self.forward(action="close"))
        return

    def set_action_deltas(self, value: np.ndarray) -> None:
        """Sets the deltas to apply for finger joint positions when opening or closing the gripper.

        Args:
            value: deltas to apply for finger joint positions when openning or closing the gripper.
                [left, right].
        """
        self._action_deltas = value
        return

    def get_action_deltas(self) -> np.ndarray:
        """Deltas that will be applied for finger joint positions when opening or closing the gripper.

        Returns:
            Deltas for left and right finger joints.
        """
        return self._action_deltas

    def set_default_state(self, joint_positions: np.ndarray) -> None:
        """Sets the default state of the gripper.

        Args:
            joint_positions: Joint positions of the left finger joint and the right finger joint respectively.
        """
        self._default_state = joint_positions
        return

    def get_default_state(self) -> np.ndarray:
        """Gets the default state of the gripper.

        Returns:
            Joint positions of the left finger joint and the right finger joint respectively.
        """
        return self._default_state

    def post_reset(self) -> None:
        """Reset the gripper to its default state."""
        Gripper.post_reset(self)
        if self._set_joint_positions_func is not None:
            self._set_joint_positions_func(positions=self._default_state, joint_indices=self.active_joint_indices)
        return

    def set_joint_positions(self, positions: np.ndarray) -> None:
        """Sets the joint positions of the gripper fingers.

        Args:
            positions: Joint positions of the left finger joint and the right finger joint respectively.
        """
        self._set_joint_positions_func(positions=positions, joint_indices=self.active_joint_indices)
        return

    def get_joint_positions(self) -> np.ndarray:
        """Joint positions of the left finger joint and the right finger joint respectively.

        Returns:
            Joint positions of the left finger joint and the right finger joint respectively.
        """
        return self._get_joint_positions_func(joint_indices=self.active_joint_indices)

    def forward(self, action: str) -> ArticulationAction:
        """Calculates the ArticulationAction for all of the articulation joints that corresponds to "open".

        or "close" actions.

        Args:
            action: "open" or "close" as an abstract action.

        Raises:
            Exception: If action is not "open" or "close".

        Returns:
            Articulation action to be passed to the articulation itself
            (includes all joints of the articulation).
        """
        target_joint_positions = [None] * self._articulation_num_dofs

        if action == "open":
            if self._action_deltas is None:
                # Use predefined open positions
                for i, joint_idx in enumerate(self.active_joint_indices):
                    target_joint_positions[joint_idx] = self._joint_opened_positions[i]
            else:
                # Apply deltas to current positions
                current_positions = self._get_joint_positions_func()
                for i, joint_idx in enumerate(self.active_joint_indices):
                    target_joint_positions[joint_idx] = current_positions[joint_idx] + self._action_deltas[i]

        elif action == "close":
            if self._action_deltas is None:
                # Use predefined closed positions
                for i, joint_idx in enumerate(self.active_joint_indices):
                    target_joint_positions[joint_idx] = self._joint_closed_positions[i]
            else:
                # Apply negative deltas to current positions
                current_positions = self._get_joint_positions_func()
                for i, joint_idx in enumerate(self.active_joint_indices):
                    target_joint_positions[joint_idx] = current_positions[joint_idx] - self._action_deltas[i]
        else:
            raise Exception(f"action {action} is not defined for ParallelGripper")

        return ArticulationAction(joint_positions=target_joint_positions)

    def apply_action(self, control_actions: ArticulationAction) -> None:
        """Applies actions to all the joints of an articulation that corresponds to the ArticulationAction of the finger joints only.

        Args:
            control_actions: ArticulationAction for the left finger joint and the right finger joint respectively.
        """
        joint_actions = ArticulationAction()
        if control_actions.joint_positions is not None:
            joint_actions.joint_positions = [None] * self._articulation_num_dofs
            for i, joint_idx in enumerate(self.active_joint_indices):
                joint_actions.joint_positions[joint_idx] = control_actions.joint_positions[i]

        if control_actions.joint_velocities is not None:
            joint_actions.joint_velocities = [None] * self._articulation_num_dofs
            for i, joint_idx in enumerate(self.active_joint_indices):
                joint_actions.joint_velocities[joint_idx] = control_actions.joint_velocities[i]

        if control_actions.joint_efforts is not None:
            joint_actions.joint_efforts = [None] * self._articulation_num_dofs
            for i, joint_idx in enumerate(self.active_joint_indices):
                joint_actions.joint_efforts[joint_idx] = control_actions.joint_efforts[i]

        self._articulation_apply_action_func(control_actions=joint_actions)
        return
