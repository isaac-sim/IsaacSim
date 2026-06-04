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

"""Base classes for implementing gripper controllers that can open and close grippers in Isaac Sim."""

from abc import abstractmethod

import numpy as np
from isaacsim.core.api.controllers.base_controller import BaseController
from isaacsim.core.utils.types import ArticulationAction


class BaseGripperController(BaseController):
    """Abstract base class for gripper controllers.

    Args:
        name: Name identifier for the controller.

    """

    def __init__(self, name: str) -> None:
        self._name = name
        return

    def forward(self, action: str, current_joint_positions: np.ndarray) -> ArticulationAction:
        """Routes gripper actions to appropriate open or close methods.

        Args:
            action: "open" or "close".
            current_joint_positions: Current positions of the gripper joints.

        Raises:
            Exception: If action is not "open" or "close".

        Returns:
            Action to apply to the gripper joints.

        """
        if action == "open":
            return self.open(current_joint_positions)
        elif action == "close":
            return self.close(current_joint_positions)
        else:
            raise Exception("The action is not recognized, it has to be either open or close")

    @abstractmethod
    def open(self, current_joint_positions: np.ndarray) -> ArticulationAction:
        """Open the gripper.

        Args:
            current_joint_positions: Current positions of the gripper joints.

        Raises:
            NotImplementedError: Must be implemented by subclass.

        Returns:
            Action to open the gripper.

        """
        raise NotImplementedError

    @abstractmethod
    def close(self, current_joint_positions: np.ndarray) -> ArticulationAction:
        """Close the gripper.

        Args:
            current_joint_positions: Current positions of the gripper joints.

        Raises:
            NotImplementedError: Must be implemented by subclass.

        Returns:
            Action to close the gripper.

        """
        raise NotImplementedError

    def reset(self) -> None:
        """Reset the gripper controller state."""
        return
