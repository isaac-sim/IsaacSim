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

"""Tests for the HolonomicController class and its OmniGraph node implementation."""

import omni.kit.test
from isaacsim.robot.wheeled_robots.controllers.holonomic_controller import HolonomicController


class TestHolonomicController(omni.kit.test.AsyncTestCase):
    """Test suite for the HolonomicController class.

    This test class validates the functionality of the HolonomicController, which computes joint velocities
    for holonomic wheeled robots based on desired velocity commands. The tests verify that the controller
    correctly transforms linear and angular velocity commands into appropriate wheel velocities for
    mecanum-wheel configurations.

    The test suite includes a single test case that validates the forward kinematics computation by
    checking that specific velocity inputs produce the expected joint velocity outputs within acceptable
    tolerance levels.
    """

    async def setUp(self) -> None:
        """Set up the test environment."""

    # ----------------------------------------------------------------------
    async def tearDown(self) -> None:
        """Clean up the test environment."""

    # ----------------------------------------------------------------------

    async def test_holonomic_drive(self) -> None:
        """Test the holonomic controller forward method with velocity commands.

        Validates that the controller correctly computes joint velocities for a three-wheeled holonomic drive
        system with mecanum wheels.
        """
        wheel_radius = [0.04, 0.04, 0.04]
        wheel_orientations = [[0, 0, 0, 1], [0.866, 0, 0, -0.5], [0.866, 0, 0, 0.5]]
        wheel_positions = [
            [-0.0980432, 0.000636773, -0.050501],
            [0.0493475, -0.084525, -0.050501],
            [0.0495291, 0.0856937, -0.050501],
        ]
        mecanum_angles = [90, 90, 90]
        velocity_command = [1.0, 1.0, 0.1]

        controller = HolonomicController(
            "test_controller", wheel_radius, wheel_positions, wheel_orientations, mecanum_angles
        )
        actions = controller.forward(velocity_command)
        self.assertAlmostEqual(actions.joint_velocities[0], -25.105, delta=0.001)
        self.assertAlmostEqual(actions.joint_velocities[1], 14.3182, delta=0.001)
        self.assertAlmostEqual(actions.joint_velocities[2], -14.5417, delta=0.001)
