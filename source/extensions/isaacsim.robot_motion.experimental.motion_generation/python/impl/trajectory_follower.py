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

"""Implementation for robot motion trajectory following controllers."""

from typing import Optional

from .base_controller import BaseController
from .trajectory import Trajectory
from .types import RobotState


class TrajectoryFollower(BaseController):
    """Controller implementation to follow a generic trajectory.

    The controller requires a trajectory to be set via `set_trajectory()` before calling
    `reset()`. If no trajectory is set, `reset()` will return False and `forward()` will
    return None.

    If `forward()` is called with a time outside of the trajectory bounds, the start time
    and trajectory are cleared and None is returned.
    """

    def __init__(self):
        self._trajectory = None

        # Time when the trajectory starts to run in the sim/real world.
        self._start_time = None

    def reset(self, estimated_state: RobotState, setpoint_state: Optional[RobotState], t: float, **kwargs) -> bool:
        """Set the trajectory start time for the current trajectory.

        Sets the start time for the current trajectory to the provided time `t`. This
        must be called after `set_trajectory()` and before the first call to `forward()`.

        Args:
            estimated_state: Current estimated state of the robot.
            setpoint_state: Optional setpoint state of the robot.
            t: Current clock time (simulation or real).
            **kwargs: Additional keyword arguments for the controller.

        Returns:
            True if reset succeeded, False if no trajectory had been set.
        """
        # If there is no trajectory, this is not ready to run.
        if self._trajectory is None:
            return False

        # Set the start time to the present time.
        self._start_time = t
        return True

    def forward(
        self, estimated_state: RobotState, setpoint_state: Optional[RobotState], t: float, **kwargs
    ) -> Optional[RobotState]:
        """Compute the desired robot state from the trajectory at the current time.

        Args:
            estimated_state: Current estimated state of the robot.
            setpoint_state: Optional setpoint state of the robot.
            t: Current clock time (simulation or real).
            **kwargs: Additional keyword arguments for the controller.

        Returns:
            Desired robot state for the current time, or None if no trajectory is set,
            if `reset()` has not been called, if the time is outside of the trajectory
            bounds.

        Example:

        .. code-block:: python

            >>> follower.set_trajectory(trajectory)
            >>> follower.reset(estimated_state, setpoint_state, 0.0)
            >>> target = follower.forward(estimated_state, setpoint_state, t)
            >>> if target is None:
            ...     print("Controller could not produce a valid result.")
        """
        if (self._trajectory is None) or (self._start_time is None):
            # Error, there is no trajectory or `reset()` was not called.
            return None

        # get the time since the trajectory started:
        trajectory_time = t - self._start_time

        if (trajectory_time < 0.0) or (trajectory_time > self._trajectory.duration):
            # Either the clock is stale (time cannot move backwards), or the trajectory
            # is stale (we are beyond its end point). Clear both of these values.
            self._start_time = None
            self._trajectory = None
            return None

        # read the trajectory, and return the desired joint states.
        return self._trajectory.get_target_state(trajectory_time)

    def set_trajectory(self, trajectory: Trajectory):
        """Set the trajectory to follow.

        Sets the trajectory and clears the start time. Call `reset()` after setting
        the trajectory to initialize the start time before calling `forward()`.

        Args:
            trajectory: The trajectory to follow.
        """
        self._trajectory = trajectory
        self._start_time = None

    def has_trajectory(self) -> bool:
        """Check if the trajectory follower has a trajectory set.

        Returns:
            True if a trajectory is set, False otherwise.
        """
        return self._trajectory is not None
