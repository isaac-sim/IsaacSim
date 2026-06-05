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

"""Differential drive controller for wheeled robots."""

from __future__ import annotations

import numpy as np


class DifferentialController:
    """Unicycle differential drive controller.

    Convert [linear_speed, angular_speed] commands into [left, right] wheel velocities
    using the standard differential-drive kinematic model.

    Args:
        wheel_radius: Radius of each drive wheel in m.
        wheel_base: Distance between left and right wheels in m.
        max_linear_speed: Maximum forward/backward speed in m/s.
        max_angular_speed: Maximum yaw rate in rad/s.
        max_wheel_speed: Maximum individual wheel angular velocity in rad/s.

    Raises:
        ValueError: If any speed limit is negative.
    """

    def __init__(
        self,
        *,
        wheel_radius: float,
        wheel_base: float,
        max_linear_speed: float = 1.0e20,
        max_angular_speed: float = 1.0e20,
        max_wheel_speed: float = 1.0e20,
    ) -> None:
        self.wheel_radius = wheel_radius
        self.wheel_base = wheel_base
        self.max_linear_speed = max_linear_speed
        self.max_angular_speed = max_angular_speed
        self.max_wheel_speed = max_wheel_speed
        if self.max_linear_speed < 0:
            raise ValueError(f"max_linear_speed must be >= 0, got {self.max_linear_speed}")
        if self.max_angular_speed < 0:
            raise ValueError(f"max_angular_speed must be >= 0, got {self.max_angular_speed}")
        if self.max_wheel_speed < 0:
            raise ValueError(f"max_wheel_speed must be >= 0, got {self.max_wheel_speed}")

    def forward(self, command: np.ndarray) -> np.ndarray:
        """Convert [linear_speed, angular_speed] to [left_wheel, right_wheel] velocities.

        Args:
            command: Shape (2,) — [forward speed, angular speed].

        Returns:
            Shape (2,) — [left wheel velocity, right wheel velocity].

        Raises:
            ValueError: If command does not have length 2.
        """
        if isinstance(command, list):
            command = np.array(command, dtype=np.float64)
        if command.shape[0] != 2:
            raise ValueError("command must have length 2")
        command = np.clip(
            command,
            a_min=[-self.max_linear_speed, -self.max_angular_speed],
            a_max=[self.max_linear_speed, self.max_angular_speed],
        )
        # omega_L = (2V - omega*b)/(2r), omega_R = (2V + omega*b)/(2r)
        left = ((2 * command[0]) - (command[1] * self.wheel_base)) / (2 * self.wheel_radius)
        right = ((2 * command[0]) + (command[1] * self.wheel_base)) / (2 * self.wheel_radius)
        joint_velocities = np.clip(
            np.array([left, right], dtype=np.float64),
            a_min=[-self.max_wheel_speed, -self.max_wheel_speed],
            a_max=[self.max_wheel_speed, self.max_wheel_speed],
        )
        return joint_velocities
