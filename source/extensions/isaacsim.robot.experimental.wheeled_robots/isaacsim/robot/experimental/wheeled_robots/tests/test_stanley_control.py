# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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

"""Tests for Stanley controller helpers."""

import math

import omni.kit.test
from isaacsim.robot.experimental.wheeled_robots.controllers.stanley_control import (
    State,
    normalize_angle,
    stanley_control,
)


class TestStanleyControl(omni.kit.test.AsyncTestCase):
    """Test Stanley controller helper behavior."""

    async def test_normalize_angle_maps_finite_angles_to_pi_range(self) -> None:
        """Verify finite angles are normalized into [-pi, pi]."""
        self.assertAlmostEqual(normalize_angle(5.0 * math.pi), math.pi)
        self.assertAlmostEqual(normalize_angle(-5.0 * math.pi), -math.pi)
        self.assertAlmostEqual(normalize_angle(1.5 * math.pi), -0.5 * math.pi)

    async def test_normalize_angle_rejects_non_finite_angles(self) -> None:
        """Verify non-finite angles fail instead of looping indefinitely."""
        with self.assertRaises(ValueError):
            normalize_angle(math.inf)
        with self.assertRaises(ValueError):
            normalize_angle(-math.inf)
        with self.assertRaises(ValueError):
            normalize_angle(math.nan)

    async def test_stanley_control_does_not_normalize_cross_track_distance(self) -> None:
        """Verify large cross-track errors steer toward the path without angle wrapping."""
        state = State(wheel_base=0.5, x=25.0, y=4.0, yaw=0.0, v=5.0)
        path_x = [float(x) for x in range(51)]
        path_y = [0.0] * len(path_x)
        path_yaw = [0.0] * len(path_x)

        steering_angle, _ = stanley_control(state, path_x, path_y, path_yaw, last_target_idx=0)

        expected_angle = math.atan2(0.5 * -4.0, state.v)
        self.assertAlmostEqual(steering_angle, expected_angle)

    async def test_stanley_control_rejects_non_finite_cross_track_error(self) -> None:
        """Verify non-finite path coordinates cannot produce an invalid steering command."""
        state = State(wheel_base=0.5, v=1.0)

        for path_y in (math.nan, math.inf, -math.inf):
            with self.subTest(path_y=path_y), self.assertRaisesRegex(ValueError, "Cross-track error must be finite"):
                stanley_control(state, [0.0], [path_y], [0.0], last_target_idx=0)
