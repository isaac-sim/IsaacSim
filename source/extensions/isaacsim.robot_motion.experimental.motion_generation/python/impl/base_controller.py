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

"""Base controller interface for robot motion generation in Isaac Sim."""

from abc import ABC, abstractmethod
from typing import Optional

from .types import RobotState


class BaseController(ABC):
    """Interface class for controllers.

    Controllers implement reset and forward to produce desired robot states based on
    estimates and optional setpoints.
    """

    @abstractmethod
    def forward(
        self, estimated_state: RobotState, setpoint_state: Optional[RobotState], t: float, **kwargs
    ) -> Optional[RobotState]:
        """Define the desired state of the robot at the next time-step.

        Args:
            estimated_state: Current estimated state of the robot.
            setpoint_state: Optional setpoint state of the robot.
            t: Current clock time (simulation or real).
            **kwargs: Additional keyword arguments for the controller.

        Returns:
            Desired robot state for the robot to track, or None if the controller cannot
            produce a valid output.

        Example:

        .. code-block:: python

            >>> controller = SomeController()
            >>> desired = controller.forward(estimated_state, setpoint_state, t)
            >>> if desired is None:
            ...     print("Controller failed")
        """

    @abstractmethod
    def reset(self, estimated_state: RobotState, setpoint_state: Optional[RobotState], t: float, **kwargs) -> bool:
        """Reset the internal state of the controller to safe values.

        This should be called immediately before running the controller for the first time.
        This function is intended to bring the controller to a safe initial state.

        Args:
            estimated_state: Current estimated state of the robot.
            setpoint_state: Optional setpoint state of the robot.
            t: Current clock time (simulation or real).
            **kwargs: Additional keyword arguments for the controller.

        Returns:
            True if reset succeeded, False otherwise.

        Example:

        .. code-block:: python

            >>> controller = SomeController()
            >>> if not controller.reset(estimated_state, setpoint_state, t):
            ...     raise RuntimeError("Controller reset failed")
        """
