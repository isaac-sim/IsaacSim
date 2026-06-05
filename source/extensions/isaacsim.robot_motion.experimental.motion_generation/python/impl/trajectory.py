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

"""Interface for defining continuous-time robot trajectories in Isaac Sim with abstract methods for duration and state retrieval."""

from abc import ABC, abstractmethod
from typing import Optional

from .types import RobotState


class Trajectory(ABC):
    """Interface class for defining a continuous-time trajectory for a robot in Isaac Sim.

    All trajectories start at time input == 0.0, and end after their duration.
    """

    @property
    @abstractmethod
    def duration(self) -> float:
        """Return the duration of the trajectory.

        Returns:
            Duration of the trajectory in seconds.
        """

    @abstractmethod
    def get_target_state(self, time: float) -> Optional[RobotState]:
        """Return the target robot state at the given time.

        The Trajectory interface assumes trajectories to be represented continuously between
        a start time and end time. An instance of this class that internally generates
        discrete time trajectories will need to implement interpolation for
        times that have not been computed.

        Args:
            time: Time along the trajectory at which to return the target state.

        Returns:
            Desired robot state at the given time, or None if the trajectory cannot provide
            a target.

        Example:

        .. code-block:: python

            >>> target = trajectory.get_target_state(0.5)
            >>> if target is None:
            ...     raise RuntimeError("No target available")
        """
