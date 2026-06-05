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

"""Unit tests for quintic path planner."""

import omni.kit.test
from isaacsim.robot.wheeled_robots.controllers.quintic_path_planner import quintic_polynomials_planner


class TestQuinticPathPlanner(omni.kit.test.AsyncTestCase):
    """Test quintic path planner failure handling."""

    async def test_planner_raises_when_constraints_are_exhausted(self) -> None:
        """Verify invalid constraints fail instead of returning the final invalid trajectory."""
        with self.assertRaises(ValueError):
            quintic_polynomials_planner(
                sx=0.0,
                sy=0.0,
                syaw=0.0,
                sv=0.0,
                sa=0.0,
                gx=1.0,
                gy=0.0,
                gyaw=0.0,
                gv=0.0,
                ga=0.0,
                max_accel=0.0,
                max_jerk=0.0,
                dt=1.0,
            )
