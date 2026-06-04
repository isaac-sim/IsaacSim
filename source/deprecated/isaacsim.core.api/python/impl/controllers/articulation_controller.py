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

"""PD controller implementation for articulated bodies with position, velocity, and effort control capabilities."""

from __future__ import annotations

import numpy as np
from isaacsim.core.utils.types import ArticulationAction, ArticulationActions


class ArticulationController(object):
    """PD Controller of all degrees of freedom of an articulation, can apply position targets, velocity targets and efforts.

    Checkout the required tutorials at https://docs.isaacsim.omniverse.nvidia.com/latest/index.html
    """

    def __init__(self) -> None:
        self._dof_controllers = []
        self._articulation_view = None
        self._default_kps = None
        self._default_kds = None
        return

    def initialize(self, articulation_view: object) -> None:
        """Initialize the controller with an articulation view.

        Args:
            articulation_view: The articulation view to control.

        """
        self._articulation_view = articulation_view
        return

    def _require_initialized(self) -> None:
        if self._articulation_view is None:
            raise RuntimeError("ArticulationController is not initialized. Call initialize() first.")

    def apply_action(self, control_actions: ArticulationAction) -> None:
        """Apply control actions to the articulation for the next physics step.

        Args:
            control_actions: Actions to be applied for next physics step.

        Raises:
            RuntimeError: If the articulation view is not initialized.

        """
        self._require_initialized()
        applied_actions = self.get_applied_action()
        joint_positions = control_actions.joint_positions
        if control_actions.joint_indices is None:
            joint_indices = self._articulation_view._backend_utils.resolve_indices(
                control_actions.joint_indices, applied_actions.joint_positions.shape[0], self._articulation_view._device
            )
        else:
            joint_indices = control_actions.joint_indices

        if control_actions.joint_positions is not None:
            joint_positions = self._articulation_view._backend_utils.convert(
                control_actions.joint_positions, device=self._articulation_view._device
            )
            joint_positions = self._articulation_view._backend_utils.expand_dims(joint_positions, 0)
            for i in range(control_actions.get_length()):
                if joint_positions[0][i] is None or np.isnan(
                    self._articulation_view._backend_utils.to_numpy(joint_positions[0][i])
                ):
                    joint_positions[0][i] = applied_actions.joint_positions[joint_indices[i]]
        joint_velocities = control_actions.joint_velocities
        if control_actions.joint_velocities is not None:
            joint_velocities = self._articulation_view._backend_utils.convert(
                control_actions.joint_velocities, device=self._articulation_view._device
            )
            joint_velocities = self._articulation_view._backend_utils.expand_dims(joint_velocities, 0)
            for i in range(control_actions.get_length()):
                if joint_velocities[0][i] is None or np.isnan(
                    self._articulation_view._backend_utils.to_numpy(joint_velocities[0][i])
                ):
                    joint_velocities[0][i] = applied_actions.joint_velocities[joint_indices[i]]
        joint_efforts = control_actions.joint_efforts
        if control_actions.joint_efforts is not None:
            joint_efforts = self._articulation_view._backend_utils.convert(
                control_actions.joint_efforts, device=self._articulation_view._device
            )
            joint_efforts = self._articulation_view._backend_utils.expand_dims(joint_efforts, 0)
            for i in range(control_actions.get_length()):
                if joint_efforts[0][i] is None or np.isnan(
                    self._articulation_view._backend_utils.to_numpy(joint_efforts[0][i])
                ):
                    joint_efforts[0][i] = applied_actions.joint_efforts[joint_indices[i]]
        self._articulation_view.apply_action(
            ArticulationActions(
                joint_positions=joint_positions,
                joint_velocities=joint_velocities,
                joint_efforts=joint_efforts,
                joint_indices=control_actions.joint_indices,
            )
        )
        return

    def set_gains(
        self, kps: np.ndarray | None = None, kds: np.ndarray | None = None, save_to_usd: bool = False
    ) -> None:
        """Set the PD controller gains.

        Args:
            kps: Proportional gains for each DOF.
            kds: Derivative gains for each DOF.
            save_to_usd: Whether to save the gains to USD.

        Raises:
            Exception: If the articulation view is not initialized.

        """
        if kps is not None:
            kps = self._articulation_view._backend_utils.expand_dims(kps, 0)
        if kds is not None:
            kds = self._articulation_view._backend_utils.expand_dims(kds, 0)
        self._articulation_view.set_gains(kps=kps, kds=kds, save_to_usd=save_to_usd)
        return

    def get_gains(self) -> tuple[np.ndarray, np.ndarray]:
        """Get the current PD controller gains.

        Raises:
            Exception: If the articulation view is not initialized.

        Returns:
            Tuple of (kps, kds) proportional and derivative gains.

        """
        kps, kds = self._articulation_view.get_gains()
        return kps[0], kds[0]

    def switch_control_mode(self, mode: str) -> None:
        """Switch the control mode for all DOFs.

        Args:
            mode: The control mode ("position", "velocity", or "effort").

        Raises:
            Exception: If the articulation view is not initialized.

        """
        self._require_initialized()
        self._articulation_view.switch_control_mode(mode=mode)
        return

    def switch_dof_control_mode(self, dof_index: int, mode: str) -> None:
        """Switch the control mode for a specific DOF.

        Args:
            dof_index: Index of the DOF to switch control mode for.
            mode: The control mode ("position", "velocity", or "effort").

        Raises:
            Exception: If the articulation view is not initialized.

        """
        self._require_initialized()
        self._articulation_view.switch_dof_control_mode(dof_index=dof_index, mode=mode)

    def set_max_efforts(self, values: np.ndarray, joint_indices: np.ndarray | list | None = None) -> None:
        """Set maximum efforts for specified joints.

        Args:
            values: Maximum effort values to set.
            joint_indices: Indices of joints to set.

        Raises:
            Exception: If the articulation view is not initialized.

        """
        values = self._articulation_view._backend_utils.create_tensor_from_list(
            [values], dtype="float32", device=self._articulation_view._device
        )
        self._articulation_view.set_max_efforts(values=values, joint_indices=joint_indices)
        return

    def get_max_efforts(self) -> np.ndarray:
        """Get the maximum efforts for all joints.

        Raises:
            Exception: If the articulation view is not initialized.

        Returns:
            Array of maximum effort values for each joint.

        """
        result = self._articulation_view.get_max_efforts()
        if result is not None:
            return result[0]
        else:
            return None

    def set_effort_modes(self, mode: str, joint_indices: np.ndarray | list | None = None) -> None:
        """Set effort modes for specified joints.

        Args:
            mode: The effort mode to set.
            joint_indices: Indices of joints to set.

        Raises:
            Exception: If the articulation view is not initialized.

        Returns:
            The result from the underlying articulation view's set_effort_modes method.

        """
        return self._articulation_view.set_effort_modes(mode=mode, joint_indices=joint_indices)

    def get_effort_modes(self) -> list[str]:
        """Get the effort modes for all joints.

        Raises:
            Exception: If the articulation view is not initialized.

        Returns:
            List of effort mode strings for each joint.

        """
        result = self._articulation_view.get_effort_modes()
        if result is not None:
            return result[0]
        else:
            return None

    def get_joint_limits(self) -> np.ndarray | None:
        """Get the joint limits for all DOFs.

        Raises:
            Exception: If the articulation view is not initialized.

        Returns:
            Tuple of (lower_limits, upper_limits) arrays.

        """
        result = self._articulation_view.get_dof_limits()
        if result is not None:
            return result[0]
        else:
            return None

    def get_applied_action(self) -> ArticulationAction:
        """Get the last applied articulation action.

        Raises:
            Exception: If the articulation view is not initialized.

        Returns:
            Last applied action.

        """
        self._require_initialized()
        applied_actions = self._articulation_view.get_applied_actions()

        if applied_actions is not None:
            return ArticulationAction(
                joint_positions=applied_actions.joint_positions[0],
                joint_velocities=applied_actions.joint_velocities[0],
                joint_efforts=applied_actions.joint_efforts[0],
            )
        else:
            return None
