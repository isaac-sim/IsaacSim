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

"""Quintic Polynomials Planner.

author: Atsushi Sakai (@Atsushi_twi)
Source: https://github.com/AtsushiSakai/PythonRobotics/blob/master/PathPlanning/QuinticPolynomialsPlanner/quintic_polynomials_planner.py
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
- [Local Path planning And Motion Control For Agv In Positioning](http://ieeexplore.ieee.org/document/637936/)

CHANGELOG:
[2021-11-19]
- Remove __main__ function
- Remove plot and animation function
- Code formatting
"""

from __future__ import annotations

import math

import numpy as np

MAX_T = 100.0
#: Maximum planning horizon in seconds.

MIN_T = 5.0
#: Minimum planning horizon in seconds.


class QuinticPolynomial:
    """Quintic (5th-order) polynomial for one-dimensional trajectory interpolation.

    Solves for coefficients a0..a5 that satisfy the boundary conditions on position,
    velocity, and acceleration at times 0 and `time`.

    Args:
        xs: Start position.
        vxs: Start velocity.
        axs: Start acceleration.
        xe: End position.
        vxe: End velocity.
        axe: End acceleration.
        time: Duration of the segment in seconds.
    """

    def __init__(self, xs: float, vxs: float, axs: float, xe: float, vxe: float, axe: float, time: float) -> None:
        self.a0 = xs
        self.a1 = vxs
        self.a2 = axs / 2.0

        A = np.array(
            [
                [time**3, time**4, time**5],
                [3 * time**2, 4 * time**3, 5 * time**4],
                [6 * time, 12 * time**2, 20 * time**3],
            ]
        )
        b = np.array(
            [xe - self.a0 - self.a1 * time - self.a2 * time**2, vxe - self.a1 - 2 * self.a2 * time, axe - 2 * self.a2]
        )
        x = np.linalg.solve(A, b)

        self.a3 = x[0]
        self.a4 = x[1]
        self.a5 = x[2]

    def calc_point(self, t: float) -> float:
        """Evaluate the polynomial at time `t`.

        Args:
            t: Time in seconds.

        Returns:
            Position at time `t`.
        """
        return self.a0 + self.a1 * t + self.a2 * t**2 + self.a3 * t**3 + self.a4 * t**4 + self.a5 * t**5

    def calc_first_derivative(self, t: float) -> float:
        """Evaluate the first derivative (velocity) at time `t`.

        Args:
            t: Time in seconds.

        Returns:
            Velocity at time `t`.
        """
        return self.a1 + 2 * self.a2 * t + 3 * self.a3 * t**2 + 4 * self.a4 * t**3 + 5 * self.a5 * t**4

    def calc_second_derivative(self, t: float) -> float:
        """Evaluate the second derivative (acceleration) at time `t`.

        Args:
            t: Time in seconds.

        Returns:
            Acceleration at time `t`.
        """
        return 2 * self.a2 + 6 * self.a3 * t + 12 * self.a4 * t**2 + 20 * self.a5 * t**3

    def calc_third_derivative(self, t: float) -> float:
        """Evaluate the third derivative (jerk) at time `t`.

        Args:
            t: Time in seconds.

        Returns:
            Jerk at time `t`.
        """
        return 6 * self.a3 + 24 * self.a4 * t + 60 * self.a5 * t**2


def quintic_polynomials_planner(
    sx: float,
    sy: float,
    syaw: float,
    sv: float,
    sa: float,
    gx: float,
    gy: float,
    gyaw: float,
    gv: float,
    ga: float,
    max_accel: float,
    max_jerk: float,
    dt: float,
) -> tuple[list[float], list[float], list[float], list[float], list[float], list[float], list[float]]:
    """Plan a trajectory using quintic polynomials between start and goal states.

    Args:
        sx: Start x position in m.
        sy: Start y position in m.
        syaw: Start yaw angle in rad.
        sv: Start velocity in m/s.
        sa: Start acceleration in m/s^2.
        gx: Goal x position in m.
        gy: Goal y position in m.
        gyaw: Goal yaw angle in rad.
        gv: Goal velocity in m/s.
        ga: Goal acceleration in m/s^2.
        max_accel: Maximum acceleration in m/s^2.
        max_jerk: Maximum jerk in m/s^3.
        dt: Time tick in s.

    Returns:
        Tuple of (time, rx, ry, ryaw, rv, ra, rj) lists for the planned trajectory.

    Raises:
        ValueError: If no trajectory satisfies the acceleration and jerk constraints.
    """
    vxs = sv * math.cos(syaw)
    vys = sv * math.sin(syaw)
    vxg = gv * math.cos(gyaw)
    vyg = gv * math.sin(gyaw)

    axs = sa * math.cos(syaw)
    ays = sa * math.sin(syaw)
    axg = ga * math.cos(gyaw)
    ayg = ga * math.sin(gyaw)

    time: list[float] = []
    rx: list[float] = []
    ry: list[float] = []
    ryaw: list[float] = []
    rv: list[float] = []
    ra: list[float] = []
    rj: list[float] = []

    for T in np.arange(MIN_T, MAX_T, MIN_T):
        xqp = QuinticPolynomial(sx, vxs, axs, gx, vxg, axg, float(T))
        yqp = QuinticPolynomial(sy, vys, ays, gy, vyg, ayg, float(T))

        time, rx, ry, ryaw, rv, ra, rj = [], [], [], [], [], [], []

        for t in np.arange(0.0, T + dt, dt):
            time.append(t)
            rx.append(xqp.calc_point(t))
            ry.append(yqp.calc_point(t))

            vx = xqp.calc_first_derivative(t)
            vy = yqp.calc_first_derivative(t)
            v = np.hypot(vx, vy)
            yaw = math.atan2(vy, vx)
            rv.append(v)
            ryaw.append(yaw)

            ax = xqp.calc_second_derivative(t)
            ay = yqp.calc_second_derivative(t)
            a = np.hypot(ax, ay)
            if len(rv) >= 2 and rv[-1] - rv[-2] < 0.0:
                a *= -1
            ra.append(a)

            jx = xqp.calc_third_derivative(t)
            jy = yqp.calc_third_derivative(t)
            j = np.hypot(jx, jy)
            if len(ra) >= 2 and ra[-1] - ra[-2] < 0.0:
                j *= -1
            rj.append(j)

        if max([abs(i) for i in ra]) <= max_accel and max([abs(i) for i in rj]) <= max_jerk:
            return time, rx, ry, ryaw, rv, ra, rj

    raise ValueError(
        f"could not find a valid trajectory with max_accel={max_accel}, max_jerk={max_jerk}, "
        f"and dt={dt} for T in [{MIN_T}, {MAX_T})"
    )
