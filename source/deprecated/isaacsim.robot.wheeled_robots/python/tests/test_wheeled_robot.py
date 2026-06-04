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

"""Unit tests for WheeledRobot validation."""

import omni.kit.test
from isaacsim.robot.wheeled_robots.robots.wheeled_robot import WheeledRobot


class TestWheeledRobot(omni.kit.test.AsyncTestCase):
    """Test WheeledRobot constructor validation."""

    async def test_requires_wheel_dof_names_or_indices(self) -> None:
        """Verify missing wheel DOF identifiers fail with a clear exception."""
        with self.assertRaises(ValueError):
            WheeledRobot(prim_path="/World/MissingWheelConfig")
