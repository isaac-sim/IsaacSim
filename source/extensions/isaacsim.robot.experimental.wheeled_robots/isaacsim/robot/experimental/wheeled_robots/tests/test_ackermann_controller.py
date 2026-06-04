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

"""Tests for Ackermann steering controller."""

import numpy as np
import omni.kit.test
from isaacsim.robot.experimental.wheeled_robots.controllers.ackermann_controller import AckermannController


class TestAckermannController(omni.kit.test.AsyncTestCase):
    """Tests for AckermannController steering and velocity behavior."""

    async def setUp(self):
        """Set up test fixtures."""

    # ----------------------------------------------------------------------

    async def tearDown(self):
        """Tear down test fixtures."""

    # ----------------------------------------------------------------------

    async def test_ackermann_steering_control(self):
        """Test steering angle and wheel velocities for constant-radius turns."""
        # First case check that it snaps to correct angle, no steering velocity

        # These tests are only valid for positive angles and positive forward velocity
        wheel_base = 1.65
        track_width = 1.25
        wheel_radius = 0.25

        def controller_calcs(
            wheel_base, track_width, wheel_radius, desired_forward_vel, radius_of_turn, desired_steering_angle
        ):
            controller = AckermannController(
                wheel_base=wheel_base, track_width=track_width, front_wheel_radius=wheel_radius
            )

            expected_steering_angle_left = np.arctan(wheel_base / (radius_of_turn - 0.5 * track_width))
            expected_steering_angle_right = np.arctan(wheel_base / (radius_of_turn + 0.5 * track_width))

            r_front_r = np.sqrt((radius_of_turn + 0.5 * track_width) ** 2 + wheel_base**2)
            r_back_r = radius_of_turn + 0.5 * track_width
            r_front_l = np.sqrt((radius_of_turn - 0.5 * track_width) ** 2 + wheel_base**2)
            r_back_l = radius_of_turn - 0.5 * track_width

            wheel_speed_front_r = desired_forward_vel / radius_of_turn * r_front_r / wheel_radius
            wheel_speed_back_r = desired_forward_vel / radius_of_turn * r_back_r / wheel_radius
            wheel_speed_front_l = desired_forward_vel / radius_of_turn * r_front_l / wheel_radius
            wheel_speed_back_l = desired_forward_vel / radius_of_turn * r_back_l / wheel_radius

            # command (np.ndarray): [desired steering angle (rad), steering_angle_velocity (rad/s), desired velocity of robot (m/s), acceleration (m/s^2), delta time (s)]
            joint_positions, joint_velocities = controller.forward(
                [desired_steering_angle, 0.0, desired_forward_vel, 0.0, 0.0]
            )

            self.assertIsNotNone(joint_positions)
            self.assertIsNotNone(joint_velocities)
            self.assertNotEqual(joint_positions[0], None)
            self.assertNotEqual(joint_positions[1], None)

            self.assertAlmostEqual(joint_positions[0], expected_steering_angle_left, delta=0.001)
            self.assertAlmostEqual(joint_positions[1], expected_steering_angle_right, delta=0.001)

            self.assertNotEqual(joint_velocities[0], None)
            self.assertNotEqual(joint_velocities[1], None)
            self.assertNotEqual(joint_velocities[2], None)
            self.assertNotEqual(joint_velocities[3], None)

            self.assertAlmostEqual(joint_velocities[0], wheel_speed_front_l, delta=0.001)
            self.assertAlmostEqual(joint_velocities[1], wheel_speed_front_r, delta=0.001)
            self.assertAlmostEqual(joint_velocities[2], wheel_speed_back_l, delta=0.001)
            self.assertAlmostEqual(joint_velocities[3], wheel_speed_back_r, delta=0.001)

        # Case 1
        desired_angular_vel = 0.4  # rad/s
        desired_forward_vel = 1.5  # rad/s
        radius_of_turn = desired_forward_vel / desired_angular_vel

        desired_steering_angle = np.arctan(wheel_base / radius_of_turn)  # rad

        controller_calcs(
            wheel_base, track_width, wheel_radius, desired_forward_vel, radius_of_turn, desired_steering_angle
        )

        # Case 2
        desired_angular_vel = 0.0001  # rad/s
        desired_forward_vel = 10.5  # rad/s
        radius_of_turn = desired_forward_vel / desired_angular_vel

        desired_steering_angle = np.arctan(wheel_base / radius_of_turn)  # rad
        controller_calcs(
            wheel_base, track_width, wheel_radius, desired_forward_vel, radius_of_turn, desired_steering_angle
        )

    async def test_ackermann_steering_velocity_drive_acceleration(self):
        """Test steering velocity and drive acceleration behavior."""
        # First case check that it snaps to correct angle, no steering velocity

        # These tests are only valid for positive angles and positive forward velocity
        wheel_base = 1.65
        track_width = 1.25
        wheel_radius = 0.25

        def controller_calcs(wheel_base, track_width, wheel_radius, desired_forward_vel, radius_of_turn):

            expected_steering_angle_left = np.arctan(wheel_base / (radius_of_turn - 0.5 * track_width))
            expected_steering_angle_right = np.arctan(wheel_base / (radius_of_turn + 0.5 * track_width))

            r_front_r = np.sqrt((radius_of_turn + 0.5 * track_width) ** 2 + wheel_base**2)
            r_back_r = radius_of_turn + 0.5 * track_width
            r_front_l = np.sqrt((radius_of_turn - 0.5 * track_width) ** 2 + wheel_base**2)
            r_back_l = radius_of_turn - 0.5 * track_width

            wheel_speed_front_r = desired_forward_vel / radius_of_turn * r_front_r / wheel_radius
            wheel_speed_back_r = desired_forward_vel / radius_of_turn * r_back_r / wheel_radius
            wheel_speed_front_l = desired_forward_vel / radius_of_turn * r_front_l / wheel_radius
            wheel_speed_back_l = desired_forward_vel / radius_of_turn * r_back_l / wheel_radius

            return [
                expected_steering_angle_left,
                expected_steering_angle_right,
                wheel_speed_front_l,
                wheel_speed_front_r,
                wheel_speed_back_l,
                wheel_speed_back_r,
            ]

        # Case 1
        desired_angular_vel = 0.2  # rad/s
        desired_forward_vel = 1.1  # rad/s
        radius_of_turn = desired_forward_vel / desired_angular_vel  # m
        desired_steering_angle = np.arctan(wheel_base / radius_of_turn)  # rad
        acceleration = 0.02  # m/s^2
        steering_velocity = 0.05  # rad/s
        dt = 0.05  # secs

        num_iterations_steering = int(np.abs(desired_steering_angle / (steering_velocity * dt))) - 1
        num_iterations_acceleration = int(np.abs(desired_forward_vel / (acceleration * dt))) - 1

        max_iter = max(num_iterations_acceleration, num_iterations_steering)

        controller = AckermannController(
            wheel_base=wheel_base, track_width=track_width, front_wheel_radius=wheel_radius
        )

        expected_joint_values = controller_calcs(
            wheel_base, track_width, wheel_radius, desired_forward_vel, radius_of_turn
        )

        for i in range(max_iter):
            # command (np.ndarray): [desired steering angle (rad), steering_angle_velocity (rad/s), desired velocity of robot (m/s), acceleration (m/s^2), delta time (s)]
            joint_positions, joint_velocities = controller.forward(
                [desired_steering_angle, steering_velocity, desired_forward_vel, acceleration, dt]
            )

            self.assertIsNotNone(joint_positions)
            self.assertIsNotNone(joint_velocities)
            self.assertNotEqual(joint_positions[0], None)
            self.assertNotEqual(joint_positions[1], None)

            if i < num_iterations_steering:
                self.assertLess(joint_positions[0], expected_joint_values[0])
                self.assertLess(joint_positions[1], expected_joint_values[1])
            else:
                self.assertAlmostEqual(joint_positions[0], expected_joint_values[0], delta=0.01)
                self.assertAlmostEqual(joint_positions[1], expected_joint_values[1], delta=0.01)

            self.assertNotEqual(joint_velocities[0], None)
            self.assertNotEqual(joint_velocities[1], None)
            self.assertNotEqual(joint_velocities[2], None)
            self.assertNotEqual(joint_velocities[3], None)

            if i < num_iterations_acceleration:
                self.assertLess(joint_velocities[0], expected_joint_values[2])
                self.assertLess(joint_velocities[1], expected_joint_values[3])
                self.assertLess(joint_velocities[2], expected_joint_values[4])
                self.assertLess(joint_velocities[3], expected_joint_values[5])
            else:
                self.assertAlmostEqual(joint_velocities[0], expected_joint_values[2], delta=0.01)
                self.assertAlmostEqual(joint_velocities[1], expected_joint_values[3], delta=0.01)
                self.assertAlmostEqual(joint_velocities[2], expected_joint_values[4], delta=0.01)
                self.assertAlmostEqual(joint_velocities[3], expected_joint_values[5], delta=0.01)

        # Case 2
        desired_angular_vel = 0.15  # rad/s
        desired_forward_vel = 3.1  # rad/s
        radius_of_turn = desired_forward_vel / desired_angular_vel  # m
        desired_steering_angle = np.arctan(wheel_base / radius_of_turn)  # rad
        acceleration = 0.2  # m/s^2
        steering_velocity = 0.07  # rad/s
        dt = 0.015  # secs

        num_iterations_steering = int(np.abs(desired_steering_angle / (steering_velocity * dt))) - 1
        num_iterations_acceleration = int(np.abs(desired_forward_vel / (acceleration * dt))) - 1
        max_iter = max(num_iterations_acceleration, num_iterations_steering)

        controller = AckermannController(
            wheel_base=wheel_base, track_width=track_width, front_wheel_radius=wheel_radius
        )

        expected_joint_values = controller_calcs(
            wheel_base, track_width, wheel_radius, desired_forward_vel, radius_of_turn
        )

        for i in range(max_iter):
            # command (np.ndarray): [desired steering angle (rad), steering_angle_velocity (rad/s), desired velocity of robot (m/s), acceleration (m/s^2), delta time (s)]
            joint_positions, joint_velocities = controller.forward(
                [desired_steering_angle, steering_velocity, desired_forward_vel, acceleration, dt]
            )

            self.assertIsNotNone(joint_positions)
            self.assertIsNotNone(joint_velocities)
            self.assertNotEqual(joint_positions[0], None)
            self.assertNotEqual(joint_positions[1], None)

            if i < num_iterations_steering:
                self.assertLess(joint_positions[0], expected_joint_values[0])
                self.assertLess(joint_positions[1], expected_joint_values[1])
            else:
                self.assertAlmostEqual(joint_positions[0], expected_joint_values[0], delta=0.01)
                self.assertAlmostEqual(joint_positions[1], expected_joint_values[1], delta=0.01)

            self.assertNotEqual(joint_velocities[0], None)
            self.assertNotEqual(joint_velocities[1], None)
            self.assertNotEqual(joint_velocities[2], None)
            self.assertNotEqual(joint_velocities[3], None)

            if i < num_iterations_acceleration:
                self.assertLess(joint_velocities[0], expected_joint_values[2])
                self.assertLess(joint_velocities[1], expected_joint_values[3])
                self.assertLess(joint_velocities[2], expected_joint_values[4])
                self.assertLess(joint_velocities[3], expected_joint_values[5])
            else:
                self.assertAlmostEqual(joint_velocities[0], expected_joint_values[2], delta=0.01)
                self.assertAlmostEqual(joint_velocities[1], expected_joint_values[3], delta=0.01)
                self.assertAlmostEqual(joint_velocities[2], expected_joint_values[4], delta=0.01)
                self.assertAlmostEqual(joint_velocities[3], expected_joint_values[5], delta=0.01)

        # Case 3
        desired_angular_vel = 0.5  # rad/s
        desired_forward_vel = 3.1  # rad/s
        radius_of_turn = desired_forward_vel / desired_angular_vel  # m
        desired_steering_angle = np.arctan(wheel_base / radius_of_turn)  # rad
        acceleration = 0.13  # m/s^2
        steering_velocity = 0.12  # rad/s
        dt = 1 / 60.0  # secs

        num_iterations_steering = int(np.abs(desired_steering_angle / (steering_velocity * dt))) - 1
        num_iterations_acceleration = int(np.abs(desired_forward_vel / (acceleration * dt))) - 1
        max_iter = max(num_iterations_acceleration, num_iterations_steering)

        controller = AckermannController(
            wheel_base=wheel_base, track_width=track_width, front_wheel_radius=wheel_radius
        )

        expected_joint_values = controller_calcs(
            wheel_base, track_width, wheel_radius, desired_forward_vel, radius_of_turn
        )

        for i in range(max_iter):
            # command (np.ndarray): [desired steering angle (rad), steering_angle_velocity (rad/s), desired velocity of robot (m/s), acceleration (m/s^2), delta time (s)]
            joint_positions, joint_velocities = controller.forward(
                [desired_steering_angle, steering_velocity, desired_forward_vel, acceleration, dt]
            )

            self.assertIsNotNone(joint_positions)
            self.assertIsNotNone(joint_velocities)
            self.assertNotEqual(joint_positions[0], None)
            self.assertNotEqual(joint_positions[1], None)

            if i < num_iterations_steering:
                self.assertLess(joint_positions[0], expected_joint_values[0])
                self.assertLess(joint_positions[1], expected_joint_values[1])
            else:
                self.assertAlmostEqual(joint_positions[0], expected_joint_values[0], delta=0.01)
                self.assertAlmostEqual(joint_positions[1], expected_joint_values[1], delta=0.01)

            self.assertNotEqual(joint_velocities[0], None)
            self.assertNotEqual(joint_velocities[1], None)
            self.assertNotEqual(joint_velocities[2], None)
            self.assertNotEqual(joint_velocities[3], None)

            if i < num_iterations_acceleration:
                self.assertLess(joint_velocities[0], expected_joint_values[2])
                self.assertLess(joint_velocities[1], expected_joint_values[3])
                self.assertLess(joint_velocities[2], expected_joint_values[4])
                self.assertLess(joint_velocities[3], expected_joint_values[5])
            else:
                self.assertAlmostEqual(joint_velocities[0], expected_joint_values[2], delta=0.01)
                self.assertAlmostEqual(joint_velocities[1], expected_joint_values[3], delta=0.01)
                self.assertAlmostEqual(joint_velocities[2], expected_joint_values[4], delta=0.01)
                self.assertAlmostEqual(joint_velocities[3], expected_joint_values[5], delta=0.01)
