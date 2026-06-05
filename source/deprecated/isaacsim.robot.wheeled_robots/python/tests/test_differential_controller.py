# SPDX-FileCopyrightText: Copyright (c) 2018-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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

"""Unit tests for the DifferentialController and DifferentialController node functionality in wheeled robots."""

import omni.kit.test
from isaacsim.robot.wheeled_robots.controllers.differential_controller import DifferentialController


class TestDifferentialController(omni.kit.test.AsyncTestCase):
    """Test class for the DifferentialController.

    This class contains unit tests for validating the functionality of the DifferentialController class,
    including differential drive calculations, wheel speed limits, and controller behavior with various
    configurations.

    The test cases cover:
    - Basic differential drive kinematics calculations
    - Wheel speed limiting functionality
    - Controller response to linear and angular velocity commands
    """

    async def setUp(self) -> None:
        """Set up test environment for differential controller tests."""

    # ----------------------------------------------------------------------
    async def tearDown(self) -> None:
        """Clean up test environment after differential controller tests."""

    # ----------------------------------------------------------------------

    async def test_differential_drive(self) -> None:
        """Test differential drive calculations and wheel speed limits.

        Validates that the DifferentialController correctly computes joint velocities from linear and angular speed
        commands, and properly enforces maximum wheel speed constraints.
        """
        # test the actual calculation of differential drive
        wheel_radius = 0.03
        wheel_base = 0.1125
        controller = DifferentialController("test_controller", wheel_radius, wheel_base)

        linear_speed = 0.3
        angular_speed = 1.0
        command = [linear_speed, angular_speed]
        actions = controller.forward(command)
        self.assertEqual(actions.joint_velocities.tolist(), [8.125, 11.875])

        ## test setting wheel limits
        controller.max_wheel_speed = 9
        actions = controller.forward(command)
        self.assertEqual(actions.joint_velocities.tolist(), [8.125, 9])
