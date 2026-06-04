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

"""Path tracking simulation with Stanley steering control and PID speed control.

author: Atsushi Sakai (@Atsushi_twi)
Source: https://github.com/AtsushiSakai/PythonRobotics/blob/master/PathTracking/stanley_controller/stanley_controller.py
Distributed under the MIT license:

The MIT License (MIT)

Copyright (c) 2016 - 2021 Atsushi Sakai

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.

Ref:
    - [Stanley: The robot that won the DARPA grand challenge](http://isl.ecst.csuchico.edu/DOCS/darpa2005/DARPA%202005%20Stanley.pdf)
    - [Autonomous Automobile Path Tracking](https://www.ri.cmu.edu/pub_files/2009/2/Automatic_Steering_Methods_for_Autonomous_Automobile_Path_Tracking.pdf)

CHANGELOG:
[2021-11-19]
- Remove __main__ function
- Remove plot and animation function
- Code formatting
- Add wheelbase length as a param instead of global variable
- increase max steering angle
"""

import numpy as np


class State(object):
    """Class representing the state of a vehicle.

    Args:
        wheel_base: Wheel base length of the vehicle.
        x: x-coordinate.
        y: y-coordinate.
        yaw: Yaw angle.
        v: Speed.
        Ks: Maximum steering angle.
    """

    def __init__(
        self,
        wheel_base: float,
        x: float = 0.0,
        y: float = 0.0,
        yaw: float = 0.0,
        v: float = 0.0,
        Ks: float = np.radians(5.0),
    ) -> None:
        super().__init__()
        self.wheel_base = wheel_base
        self.x = x
        self.y = y
        self.yaw = yaw
        self.v = v
        self.w = 0
        self.Ks = Ks

    def update(self, acceleration: float, delta: float, dt: float) -> None:
        """Update the state of the vehicle.

            Stanley Control uses bicycle model.

        Args:
            acceleration: Acceleration.
            delta: Steering angle.
            dt: Time step.
        """
        delta = np.clip(delta, -self.Ks, self.Ks)

        self.x += self.v * np.cos(self.yaw) * dt
        self.y += self.v * np.sin(self.yaw) * dt
        self.w = self.v / self.wheel_base * np.tan(delta)
        self.yaw += self.w * dt
        self.yaw = normalize_angle(self.yaw)
        self.v += acceleration * dt


def pid_control(target: float, current: float, Kp: float = 0.1) -> float:
    """Proportional control for the speed.

    Args:
        target: Target speed value.
        current: Current speed value.
        Kp: Proportional gain coefficient.

    Returns:
        Control output for speed adjustment.
    """
    return Kp * (target - current)


def stanley_control(
    state: "State",
    cx: list[float],
    cy: list[float],
    cyaw: list[float],
    last_target_idx: int,
    p: float = 0.5,
    i: float = 0.01,
    d: float = 10,
    k: float = 0.5,
) -> tuple[float, int]:
    """Stanley steering control.

    Args:
        state: State object containing vehicle position and orientation.
        cx: X-coordinates of the reference path.
        cy: Y-coordinates of the reference path.
        cyaw: Yaw angles of the reference path.
        last_target_idx: Previous target index in the trajectory.
        p: Proportional gain (unused, reserved for future PID extension).
        i: Integral gain (unused, reserved for future PID extension).
        d: Derivative gain (unused, reserved for future PID extension).
        k: Cross track error gain.

    Returns:
        A tuple containing (steering angle, current target index).
    """
    current_target_idx, error_front_axle = calc_target_index(state, cx, cy)

    if last_target_idx >= current_target_idx:
        current_target_idx = last_target_idx

    # theta_e corrects the heading error
    theta_e = normalize_angle(cyaw[current_target_idx] - state.yaw)

    # theta_d corrects the cross track error
    theta_d = np.arctan2(k * normalize_angle(error_front_axle), state.v)

    # Steering control
    delta = theta_e + theta_d

    return delta, current_target_idx


def normalize_angle(angle: float) -> float:
    """Normalize an angle to [-pi, pi].

    Args:
        angle: Input angle in radians.

    Returns:
        Angle in radian in [-pi, pi].

    Raises:
        ValueError: If angle is not finite.
    """
    if not np.isfinite(angle):
        raise ValueError("angle must be finite")

    normalized = (angle + np.pi) % (2.0 * np.pi) - np.pi
    if normalized == -np.pi and angle > 0.0:
        return np.pi
    return normalized


def calc_target_index(state: "State", cx: list[float], cy: list[float]) -> tuple[int, float]:
    """Compute index in the trajectory list of the target.

    Args:
        state: State object containing vehicle position and orientation.
        cx: X-coordinates of the reference path.
        cy: Y-coordinates of the reference path.

    Returns:
        A tuple containing (target index, cross track error projected onto front axle vector).
    """
    # Calc front axle position
    fx = state.x + state.wheel_base * np.cos(state.yaw)
    fy = state.y + state.wheel_base * np.sin(state.yaw)

    # Search nearest point index
    dx = [fx - icx for icx in cx]
    dy = [fy - icy for icy in cy]
    d = np.hypot(dx, dy)
    target_idx = np.argmin(d)

    # Project RMS error onto front axle vector
    front_axle_vec = [-np.cos(state.yaw + np.pi / 2), -np.sin(state.yaw + np.pi / 2)]
    error_front_axle = np.dot([dx[target_idx], dy[target_idx]], front_axle_vec)

    return target_idx, error_front_axle
