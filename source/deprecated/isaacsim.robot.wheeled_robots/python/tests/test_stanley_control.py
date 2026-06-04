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

"""Unit tests for Stanley controller helpers."""

import math

import omni.kit.test
from isaacsim.robot.wheeled_robots.controllers.stanley_control import normalize_angle


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
