# SPDX-FileCopyrightText: Copyright (c) 2024-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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

"""Ackermann steering controller for wheeled robots."""

from __future__ import annotations

import numpy as np


class AckermannController:
    """Ackermann steering controller using a bicycle model.

    Computes left and right steering angles and per-wheel rotation velocities for a
    four-wheel Ackermann robot. Returns plain tuples (no ArticulationAction).

    Args:
        wheel_base: Distance between front and rear axles in m.
        track_width: Distance between left and right wheels in m.
        front_wheel_radius: Radius of front wheels in m.
        back_wheel_radius: Radius of back wheels in m.
        max_wheel_velocity: Maximum angular velocity of wheels in rad/s. Ignored if 0.
        invert_steering: True for rear-wheel-steered robots such as forklifts, where the
            rear axle carries the steering joints and the front wheels stay fixed in heading.
        max_wheel_rotation_angle: Maximum steering angle in rad. Ignored if 0.
        max_acceleration: Maximum linear acceleration in m/s^2. Ignored if 0.
        max_steering_angle_velocity: Maximum steering rate in rad/s. Ignored if 0.
    """

    def __init__(
        self,
        *,
        wheel_base: float,
        track_width: float,
        front_wheel_radius: float = 0.0,
        back_wheel_radius: float = 0.0,
        max_wheel_velocity: float = 0.0,
        invert_steering: bool = False,
        max_wheel_rotation_angle: float = 6.28,
        max_acceleration: float = 0.0,
        max_steering_angle_velocity: float = 0.0,
    ) -> None:
        self.wheel_base = np.fabs(wheel_base)
        self.track_width = np.fabs(track_width)
        self.front_wheel_radius = np.fabs(front_wheel_radius)
        self.back_wheel_radius = np.fabs(back_wheel_radius)
        self.max_wheel_velocity = np.fabs(max_wheel_velocity)
        self.invert_steering = invert_steering
        self.max_wheel_rotation_angle = np.fabs(max_wheel_rotation_angle)
        self.max_acceleration = np.fabs(max_acceleration)
        self.max_steering_angle_velocity = np.fabs(max_steering_angle_velocity)

        self.prev_linear_velocity = 0.0
        self.prev_steering_angle = 0.0

    def forward(self, command: np.ndarray) -> tuple[
        tuple[float, float] | None,
        tuple[float, float, float, float] | None,
    ]:
        """Compute wheel angles and wheel rotation velocities from an Ackermann command.

        Args:
            command: Length 5 — [steering_angle (rad), steering_angle_velocity (rad/s),
                speed (m/s), acceleration (m/s^2), dt (s)].

        Returns:
            Tuple of (joint_positions, joint_velocities). joint_positions is
            (left_wheel_angle, right_wheel_angle); joint_velocities is
            (v_FL, v_FR, v_BL, v_BR). Returns (None, None) on invalid input.
        """
        if isinstance(command, list):
            command = np.array(command, dtype=np.float64)

        if len(command) != 5:
            return (None, None)

        fwr = self.front_wheel_radius
        bwr = self.back_wheel_radius
        if fwr == 0.0 and bwr == 0.0:
            return (None, None)
        if fwr == 0.0:
            fwr = bwr
        if bwr == 0.0:
            bwr = fwr

        max_wv = self.max_wheel_velocity if self.max_wheel_velocity > 0 else np.inf
        max_rot = self.max_wheel_rotation_angle if self.max_wheel_rotation_angle > 0 else np.inf
        max_acc = self.max_acceleration if self.max_acceleration > 0 else np.inf
        max_steer_vel = self.max_steering_angle_velocity if self.max_steering_angle_velocity > 0 else np.inf

        effective_radius = np.maximum(fwr, bwr)
        max_linear_velocity = np.fabs(max_wv * effective_radius)

        command = np.array(command, dtype=np.float64, copy=True)
        command[0] = np.clip(command[0], -max_rot, max_rot)
        command[2] = np.clip(command[2], -max_linear_velocity, max_linear_velocity)
        if max_acc != np.inf:
            command[3] = np.minimum(np.fabs(command[3]), max_acc)
        if max_steer_vel != np.inf:
            command[1] = np.minimum(np.fabs(command[1]), max_steer_vel)
        command[4] = np.fabs(command[4])

        if command[4] == 0.0 and (command[1] != 0.0 or command[3] != 0.0):
            return (None, None)

        forward_vel = self.prev_linear_velocity
        if command[3] == 0.0:
            forward_vel = command[2]
        else:
            velocity_diff = command[2] - self.prev_linear_velocity
            if np.fabs(velocity_diff) > 0.0001:
                if velocity_diff > 0:
                    forward_vel = self.prev_linear_velocity + command[3] * command[4]
                    forward_vel = np.minimum(forward_vel, command[2])
                else:
                    forward_vel = self.prev_linear_velocity - command[3] * command[4]
                    forward_vel = np.maximum(forward_vel, command[2])
        self.prev_linear_velocity = float(forward_vel)

        steering_angle = self.prev_steering_angle
        if command[1] == 0.0:
            steering_angle = command[0]
        else:
            steering_angle_diff = command[0] - self.prev_steering_angle
            if np.fabs(steering_angle_diff) > 0.00174533:
                if steering_angle_diff > 0:
                    steering_angle = self.prev_steering_angle + command[1] * command[4]
                    steering_angle = np.minimum(steering_angle, command[0])
                else:
                    steering_angle = self.prev_steering_angle - command[1] * command[4]
                    steering_angle = np.maximum(steering_angle, command[0])
        self.prev_steering_angle = float(steering_angle)

        if np.fabs(steering_angle) < 0.0157:
            left_wheel_angle = 0.0
            right_wheel_angle = 0.0
            v_fl = forward_vel / fwr
            v_fr = forward_vel / fwr
            v_bl = forward_vel / bwr
            v_br = forward_vel / bwr
        else:
            R = ((-1.0 if self.invert_steering else 1.0) * self.wheel_base) / np.tan(steering_angle)
            left_wheel_angle = np.arctan(self.wheel_base / (R - 0.5 * self.track_width))
            right_wheel_angle = np.arctan(self.wheel_base / (R + 0.5 * self.track_width))

            steering_joint_half_dist = self.track_width / 2.0
            cy = np.fabs(R)
            sign = 1.0 if steering_angle > 0 else -1.0

            if self.invert_steering:
                wheel_dist_FL = cy - sign * steering_joint_half_dist
                wheel_dist_FR = cy + sign * steering_joint_half_dist
                wheel_dist_BL = np.sqrt((cy - sign * steering_joint_half_dist) ** 2 + self.wheel_base**2)
                wheel_dist_BR = np.sqrt((cy + sign * steering_joint_half_dist) ** 2 + self.wheel_base**2)
            else:
                wheel_dist_FL = np.sqrt((cy - sign * steering_joint_half_dist) ** 2 + self.wheel_base**2)
                wheel_dist_FR = np.sqrt((cy + sign * steering_joint_half_dist) ** 2 + self.wheel_base**2)
                wheel_dist_BL = cy - sign * steering_joint_half_dist
                wheel_dist_BR = cy + sign * steering_joint_half_dist

            body_ang_vel = forward_vel / cy
            v_fl = body_ang_vel * (wheel_dist_FL / fwr)
            v_fr = body_ang_vel * (wheel_dist_FR / fwr)
            v_bl = body_ang_vel * (wheel_dist_BL / bwr)
            v_br = body_ang_vel * (wheel_dist_BR / bwr)

        v_fl = np.clip(v_fl, -max_wv, max_wv)
        v_fr = np.clip(v_fr, -max_wv, max_wv)
        v_bl = np.clip(v_bl, -max_wv, max_wv)
        v_br = np.clip(v_br, -max_wv, max_wv)
        left_wheel_angle = np.clip(left_wheel_angle, -max_rot, max_rot)
        right_wheel_angle = np.clip(right_wheel_angle, -max_rot, max_rot)

        joint_positions = (float(left_wheel_angle), float(right_wheel_angle))
        joint_velocities = (float(v_fl), float(v_fr), float(v_bl), float(v_br))
        return (joint_positions, joint_velocities)
