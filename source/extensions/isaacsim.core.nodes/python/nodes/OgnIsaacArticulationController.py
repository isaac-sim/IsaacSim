# SPDX-FileCopyrightText: Copyright (c) 2022-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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

"""Command selected articulation joints from OmniGraph position, velocity, and effort inputs."""

from typing import Any

import numpy as np
from isaacsim.core.experimental.prims import Articulation
from isaacsim.core.nodes import BaseResetNode
from isaacsim.core.nodes.ogn.OgnIsaacArticulationControllerDatabase import OgnIsaacArticulationControllerDatabase


class OgnIsaacArticulationControllerInternalState(BaseResetNode):
    """nodes for moving an articulated robot with joint commands."""

    def __init__(self) -> None:
        self.prim_path = None
        self.articulation = None
        self.joint_names = None
        self.joint_indices = None
        self.joint_picked = False
        self.command_error_message = None
        self.node = None
        super().__init__(initialize=False)

    def initialize_controller(self) -> None:
        """Create the articulation handle for the selected robot prim and mark the state initialized."""
        self.articulation = Articulation(self.prim_path)
        self.initialized = True

    def joint_indicator(self) -> None:
        """Resolve the current joint name or index selection to DOF indices, or target all DOFs."""
        if self.joint_names:
            self.joint_indices = self.articulation.get_dof_indices(self.joint_names).numpy().flatten()
        elif np.size(self.joint_indices) > 0:
            self.joint_indices = self.joint_indices
        else:
            # when indices is none (not []), it defaults too all DOFs
            self.joint_indices = None
        self.joint_picked = True

    def apply_action(
        self,
        joint_positions: np.ndarray | list | tuple,
        joint_velocities: np.ndarray | list | tuple,
        joint_efforts: np.ndarray | list | tuple,
    ) -> bool:
        """Apply position, velocity, and effort commands to the articulation.

        All commands are validated and filtered before any targets are written, so an invalid command prevents partial
        application of the remaining valid commands.

        Args:
            joint_positions: Position targets for the selected joints.
            joint_velocities: Velocity targets for the selected joints.
            joint_efforts: Effort targets for the selected joints.

        Returns:
            True when all provided commands were valid and applied; False when the controller is not initialized or any
            command is invalid.
        """
        self.command_error_message = None
        if not self.initialized:
            self.command_error_message = "Articulation controller is not initialized; ignoring command."
            return False

        position_command = self._prepare_command(joint_positions, "positionCommand")
        velocity_command = self._prepare_command(joint_velocities, "velocityCommand")
        effort_command = self._prepare_command(joint_efforts, "effortCommand")
        if position_command is None or velocity_command is None or effort_command is None:
            return False

        joint_positions, position_indices = position_command
        joint_velocities, velocity_indices = velocity_command
        joint_efforts, effort_indices = effort_command
        if np.size(joint_positions) > 0:
            self.articulation.set_dof_position_targets(joint_positions, dof_indices=position_indices)
        if np.size(joint_velocities) > 0:
            self.articulation.set_dof_velocity_targets(joint_velocities, dof_indices=velocity_indices)
        if np.size(joint_efforts) > 0:
            self.articulation.set_dof_efforts(joint_efforts, dof_indices=effort_indices)
        return True

    def _prepare_command(
        self, command: np.ndarray | list | tuple, command_name: str
    ) -> tuple[np.ndarray | list | tuple, np.ndarray | list | None] | None:
        """Validate and filter a command before any articulation targets are written.

        Args:
            command: Joint command values for the currently selected joints.
            command_name: Name of the OGN command input being validated.

        Returns:
            A tuple of filtered command values and matching DOF indices. Returns None if the command width does not
            match the explicit joint selection or if the command shape cannot be resolved.
        """
        if np.size(command) == 0:
            return command, self.joint_indices
        command_valid, dof_indices = self._resolve_command_indices(command)
        if not command_valid:
            self.command_error_message = self._format_command_mismatch(command_name, command)
            return None
        filtered_command, dof_indices = self._filter_finite_command(command, dof_indices)
        if filtered_command is None:
            self.command_error_message = self._format_command_mismatch(command_name, command)
            return None
        return filtered_command, dof_indices

    def _format_command_mismatch(self, command_name: str, command: np.ndarray | list | tuple) -> str:
        prim_path = getattr(self, "prim_path", None)
        selected_joint_count = 0 if self.joint_indices is None else np.asarray(self.joint_indices).reshape(-1).size
        selected_joints = "all DOFs" if self.joint_indices is None else f"jointIndices={self.joint_indices}"
        return (
            f"Articulation controller command mismatch for prim '{prim_path}': {command_name} has "
            f"{np.size(command)} value(s), but the selected joint count is {selected_joint_count} "
            f"({selected_joints}). Ignoring all commands."
        )

    def _resolve_command_indices(self, command: np.ndarray | list | tuple) -> tuple[bool, np.ndarray | list | None]:
        """Validate that a command width matches the explicitly selected joints.

        Args:
            command: Joint command values to validate.

        Returns:
            A success flag and the DOF indices matched to the command.
        """
        if self.joint_indices is None:
            return True, self.joint_indices
        command_size = np.size(command)
        joint_indices = np.asarray(self.joint_indices).reshape(-1)
        if command_size == joint_indices.size:
            return True, self.joint_indices
        return False, self.joint_indices

    def _filter_finite_command(
        self, command: np.ndarray | list | tuple, dof_indices: np.ndarray | list | None
    ) -> tuple[np.ndarray | list | tuple | None, np.ndarray | list | None]:
        """Drop NaN entries so omitted targets are left unchanged by the backend.

        Args:
            command: Joint command values to filter.
            dof_indices: DOF indices associated with the command values.

        Returns:
            Filtered command values and their matching DOF indices.
        """
        command_array = np.asarray(command)
        if not np.isnan(command_array).any():
            return command, dof_indices

        command_values = command_array.reshape(-1)
        # NaN command entries mean "leave the current target unchanged".  Write
        # only the finite entries so backends without local tensor state do not
        # need to read the previous target value.
        finite_mask = ~np.isnan(command_values)
        if dof_indices is None:
            selected_dof_indices = np.arange(command_values.size, dtype=np.int64)
        else:
            selected_dof_indices = np.asarray(dof_indices).reshape(-1)
            if selected_dof_indices.size != command_values.size:
                return None, dof_indices

        return command_values[finite_mask], selected_dof_indices[finite_mask]

    def custom_reset(self) -> None:
        """Clear the articulation handle and reset command inputs after the node is reset."""
        self.articulation = None
        if self.initialized:
            self.node.get_attribute("inputs:positionCommand").set(np.empty(shape=(0, 0), dtype=np.double))
            self.node.get_attribute("inputs:velocityCommand").set(np.empty(shape=(0, 0), dtype=np.double))
            self.node.get_attribute("inputs:effortCommand").set(np.empty(shape=(0, 0), dtype=np.double))


class OgnIsaacArticulationController:
    """nodes for moving an articulated robot with joint commands."""

    @staticmethod
    def init_instance(node: Any, graph_instance_id: Any) -> None:
        """Store the OmniGraph node on the per-instance state so reset can clear its input attributes.

        Args:
            node: OmniGraph node instance.
            graph_instance_id: Graph instance identifier.
        """
        state = OgnIsaacArticulationControllerDatabase.get_internal_state(node, graph_instance_id)
        state.node = node

    @staticmethod
    def internal_state() -> OgnIsaacArticulationControllerInternalState:
        """Create the per-instance articulation controller state.

        Returns:
            Per-instance articulation controller state.
        """
        return OgnIsaacArticulationControllerInternalState()

    @staticmethod
    def compute(db: Any) -> bool:
        """Resolve the robot and selected joints, then apply valid joint commands.

        The node accepts either `robotPath` or `targetPrim`, refreshes joint selection when name
        or index inputs change, writes finite position, velocity, and effort targets, and returns
        False with a logged error when initialization or command validation fails.

        Args:
            db: OmniGraph database for this node.

        Returns:
            True when commands are applied successfully, False otherwise.
        """
        state = db.per_instance_state
        try:
            if not state.initialized:
                if len(db.inputs.robotPath) != 0:
                    state.prim_path = db.inputs.robotPath
                else:
                    if len(db.inputs.targetPrim) == 0:
                        db.log_error("No robot prim found for the articulation controller")
                        return False
                    else:
                        state.prim_path = db.inputs.targetPrim[0].GetString()

                # initialize the controller handle for the robot
                state.initialize_controller()

            # pick the joints that are being commanded, this can be different at every step
            joint_names = db.inputs.jointNames
            if joint_names and np.asarray([joint_names != state.joint_names]).flatten().any():
                state.joint_names = joint_names
                state.joint_picked = False

            joint_indices = db.inputs.jointIndices
            if np.asarray(joint_indices).any() and not np.array_equal(joint_indices, state.joint_indices):
                state.joint_indices = np.array(joint_indices)
                state.joint_picked = False

            if not state.joint_picked:
                state.joint_indicator()

            if not state.apply_action(db.inputs.positionCommand, db.inputs.velocityCommand, db.inputs.effortCommand):
                db.log_error(
                    getattr(state, "command_error_message", None)
                    or f"Articulation controller command failed for prim '{getattr(state, 'prim_path', None)}'; ignoring command."
                )
                return False

        except Exception as error:
            db.log_error(f"Articulation controller failed for prim '{getattr(state, 'prim_path', None)}': {error}")
            return False

        return True

    @staticmethod
    def release_instance(node: Any, graph_instance_id: Any) -> None:
        """Reset per-instance controller state when the OmniGraph node instance is released.

        Args:
            node: OmniGraph node instance.
            graph_instance_id: Graph instance identifier.
        """
        try:
            state = OgnIsaacArticulationControllerDatabase.get_internal_state(node, graph_instance_id)
        except Exception:
            state = None

        if state is not None:
            state.reset()
            state.initialized = False
