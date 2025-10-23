# SPDX-FileCopyrightText: Copyright (c) 2021-2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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
from typing import List, Optional, Tuple, Union

import numpy as np
from isaacsim.core.utils.types import ArticulationAction, ArticulationActions


class ArticulationController(object):
    """PD Controller of all degrees of freedom of an articulation, can apply position targets, velocity targets and efforts.

    Checkout the required tutorials at https://docs.isaacsim.omniverse.nvidia.com/latest/index.html
    """

    def __init__(self) -> None:
        self._dof_controllers = list()
        self._articulation_view = None
        self._default_kps = None
        self._default_kds = None
        return

    def initialize(self, articulation_view) -> None:
        """[summary]

        Args:
            articulation_view ([type]): [description]
        """
        self._articulation_view = articulation_view
        return

    def apply_action(self, control_actions: ArticulationAction) -> None:
        """[summary]

        Args:
            control_actions (ArticulationAction): actions to be applied for next physics step.
            indices (Optional[Union[list, np.ndarray]], optional): degree of freedom indices to apply actions to.
                                                                   Defaults to all degrees of freedom.

        Raises:
            Exception: [description]
        """
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
                    joint_efforts[0][i] = 0
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
        self, kps: Optional[np.ndarray] = None, kds: Optional[np.ndarray] = None, save_to_usd: bool = False
    ) -> None:
        """[summary]

        Args:
            kps (Optional[np.ndarray], optional): [description]. Defaults to None.
            kds (Optional[np.ndarray], optional): [description]. Defaults to None.

        Raises:
            Exception: [description]
        """
        if kps is not None:
            kps = self._articulation_view._backend_utils.expand_dims(kps, 0)
        if kds is not None:
            kds = self._articulation_view._backend_utils.expand_dims(kds, 0)
        self._articulation_view.set_gains(kps=kps, kds=kds, save_to_usd=save_to_usd)
        return

    def get_gains(self) -> Tuple[np.ndarray, np.ndarray]:
        """[summary]

        Raises:
            Exception: [description]

        Returns:
            Tuple[np.ndarray, np.ndarray]: [description]
        """
        kps, kds = self._articulation_view.get_gains()
        return kps[0], kds[0]

    def switch_control_mode(self, mode: str) -> None:
        """[summary]

        Args:
            mode (str): [description]

        Raises:
            Exception: [description]
        """
        self._articulation_view.switch_control_mode(mode=mode)
        return

    def switch_dof_control_mode(self, dof_index: int, mode: str) -> None:
        """[summary]

        Args:
            dof_index (int): [description]
            mode (str): [description]

        Raises:
            Exception: [description]
        """
        self._articulation_view.switch_dof_control_mode(dof_index=dof_index, mode=mode)

    def set_max_efforts(self, values: np.ndarray, joint_indices: Optional[Union[np.ndarray, list]] = None) -> None:
        """[summary]

        Args:
            value (float, optional): [description]. Defaults to None.
            indices (Optional[Union[np.ndarray, list]], optional): [description]. Defaults to None.

        Raises:
            Exception: [description]
        """
        values = self._articulation_view._backend_utils.create_tensor_from_list(
            [values], dtype="float32", device=self._articulation_view._device
        )
        self._articulation_view.set_max_efforts(values=values, joint_indices=joint_indices)
        return

    def get_max_efforts(self) -> np.ndarray:
        """[summary]

        Raises:
            Exception: [description]

        Returns:
            np.ndarray: [description]
        """
        result = self._articulation_view.get_max_efforts()
        if result is not None:
            return result[0]
        else:
            return None

    def set_effort_modes(self, mode: str, joint_indices: Optional[Union[np.ndarray, list]] = None) -> None:
        """[summary]

        Args:
            mode (str): [description]
            indices (Optional[Union[np.ndarray, list]], optional): [description]. Defaults to None.

        Raises:
            Exception: [description]
            Exception: [description]
        """
        return self._articulation_view.set_effort_modes(mode=mode, joint_indices=joint_indices)

    def get_effort_modes(self) -> List[str]:
        """[summary]

        Raises:
            Exception: [description]
            NotImplementedError: [description]

        Returns:
            np.ndarray: [description]
        """
        result = self._articulation_view.get_effort_modes()
        if result is not None:
            return result[0]
        else:
            return None

    def get_joint_limits(self) -> Tuple[np.ndarray, np.ndarray]:
        """[summary]

        Raises:
            Exception: [description]

        Returns:
            Tuple[np.ndarray, np.ndarray]: [description]
        """
        result = self._articulation_view.get_dof_limits()
        if result is not None:
            return result[0]
        else:
            return None

    def get_applied_action(self) -> ArticulationAction:
        """

        Raises:
            Exception: [description]

        Returns:
            ArticulationAction: Gets last applied action.
        """
        applied_actions = self._articulation_view.get_applied_actions()

        if applied_actions is not None:
            return ArticulationAction(
                joint_positions=applied_actions.joint_positions[0],
                joint_velocities=applied_actions.joint_velocities[0],
                joint_efforts=applied_actions.joint_efforts[0],
            )
        else:
            return None
