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

"""Ackermann steering controller that computes wheel angles and velocities for wheeled robots using a bicycle model."""

import carb
import numpy as np

# Import packages.
from isaacsim.core.api.controllers.base_controller import BaseController
from isaacsim.core.utils.types import ArticulationAction


class AckermannController(BaseController):
    """This controller uses a bicycle model for Ackermann steering. The controller computes the left turning angle, right turning angle, and the rotation velocity of each wheel of a robot with no slip angle. The controller can be used to find the appropriate joint values of a wheeled robot with an Ackermann steering mechanism.

    Args:
        name: Name identifier for the controller.
        wheel_base: Distance between front and rear axles in m.
        track_width: Distance between left and right wheels of the robot in m.
        front_wheel_radius: Radius of the front wheels of the robot in m.
            Will equal back_wheel_radius if no value is inputted.
        back_wheel_radius: Radius of the back wheels of the robot in m.
            Will equal front_wheel_radius if no value is inputted.
        max_wheel_velocity: Maximum angular velocity of the robot wheel in rad/s.
            Parameter is ignored if set to 0.0.
        invert_steering: Set to true for rear wheel steering.
        max_wheel_rotation_angle: The maximum wheel steering angle for the steering wheels.
            Parameter is ignored if set to 0.0.
        max_acceleration: The maximum magnitude of acceleration for the robot in m/s^2.
            Parameter is ignored if set to 0.0.
        max_steering_angle_velocity: The maximum magnitude of desired rate of change for steering angle in rad/s.
            Parameter is ignored if set to 0.0.
    """

    def __init__(
        self,
        name: str,
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
        super().__init__(name)
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

        self.current_speed = 0.0
        self.current_steering_angle = 0.0
        self.current_acceleration = 0.0
        self.current_steering_angle_velocity = 0.0

        self.wheel_rotation_velocity_FL = 0.0
        self.wheel_rotation_velocity_FR = 0.0
        self.wheel_rotation_velocity_BL = 0.0
        self.wheel_rotation_velocity_BR = 0.0

    def forward(self, command: np.ndarray) -> ArticulationAction:
        """Calculate right and left wheel angles and angular velocity of each wheel given steering angle and desired forward velocity.

        Args:
            command: [desired steering angle (rad), steering_angle_velocity (rad/s), desired velocity of robot (m/s), acceleration (m/s^2), delta time (s)]

        Returns:
            Articulation action containing joint velocities for front left, front right, back left, back right wheels and joint positions for left and right wheel angles.
        """
        if isinstance(command, list):
            command = np.array(command)

        if len(command) != 5:
            carb.log_warn(
                f"Given command is of length {len(command)}. Please ensure it contains 5 elements, skipping current step"
            )
            return ArticulationAction()

        # Ensure wheel radii are valid
        if self.front_wheel_radius == 0.0 and self.back_wheel_radius == 0.0:
            carb.log_warn("Both wheel radii are equal to 0, skipping current step")
            return ArticulationAction()

        # If the front wheel radius is invalid, use the back wheel radius if it's valid
        if self.front_wheel_radius == 0.0:
            self.front_wheel_radius = self.back_wheel_radius

        # If the back wheel radius is invalid, use the front wheel radius if it's valid
        if self.back_wheel_radius == 0.0:
            self.back_wheel_radius = self.front_wheel_radius

        # if max wheel velocity is equal to zero, interpret max value as infinity
        if self.max_wheel_velocity == 0.0:
            self.max_wheel_velocity = np.inf

        # if max wheel rotation angle is equal to zero, interpret max value as infinity
        if self.max_wheel_rotation_angle == 0.0:
            self.max_wheel_rotation_angle = np.inf

        # if max acceleration is equal to zero, interpret max value as infinity
        if self.max_acceleration == 0.0:
            self.max_acceleration = np.inf

        # if max steering angle is equal to zero, interpret max value as infinity
        if self.max_steering_angle_velocity == 0.0:
            self.max_steering_angle_velocity = np.inf

        # Use the larger of the two radii to determine maximum linear velocity
        effective_radius = np.maximum(self.front_wheel_radius, self.back_wheel_radius)
        self.max_linear_velocity = np.fabs(self.max_wheel_velocity * effective_radius)

        # limit linear velocity and steering angle and acceleration
        command[0] = np.clip(command[0], -self.max_wheel_rotation_angle, self.max_wheel_rotation_angle)
        command[2] = np.clip(command[2], -self.max_linear_velocity, self.max_linear_velocity)

        if self.max_acceleration != np.inf:
            command[3] = np.minimum(np.fabs(command[3]), self.max_acceleration)

        if self.max_steering_angle_velocity != np.inf:
            command[1] = np.minimum(np.fabs(command[1]), self.max_steering_angle_velocity)

        # Ensure DT is always positive
        command[4] = np.fabs(command[4])

        # Check delta time assuming acceleration is used (set to non-zero)
        if command[4] == 0.0 and (command[1] != 0.0 or command[3] != 0.0):
            carb.log_warn(
                f"invalid dt {command[4]}, cannot check for acceleration or steering_angle_velocity limits, skipping current step"
            )
            return ArticulationAction()

        forward_vel = self.prev_linear_velocity
        # Instant change to desired speed
        if command[3] == 0.0:
            forward_vel = command[2]  # Snap to desired speed

        else:
            # Calculate the velocity difference (target - prev velocity)
            velocity_diff = command[2] - self.prev_linear_velocity

            # If velocity difference is larger than 0.0001 m/s then compute new forward_vel
            if np.fabs(velocity_diff) > 0.0001:

                # Determine the direction of acceleration based on the velocity difference
                if velocity_diff > 0:  # Need to speed up
                    # compute new velocity --> (current linear velocity + acceleration * dt)
                    forward_vel = self.prev_linear_velocity + command[3] * command[4]
                    forward_vel = np.minimum(forward_vel, command[2])  # Don't exceed the target

                elif velocity_diff < 0:  # Need to slow down
                    # compute new velocity --> (current linear velocity + acceleration * dt)
                    forward_vel = self.prev_linear_velocity - command[3] * command[4]
                    forward_vel = np.maximum(forward_vel, command[2])  # Don't go below the target

        self.prev_linear_velocity = forward_vel

        steering_angle = self.prev_steering_angle

        # Instant change to desired angle
        if command[1] == 0.0:
            steering_angle = command[0]  # Snap to desired speed

        else:
            # Calculate the steering angle difference (target - prev steering angel)
            steering_angle_diff = command[0] - self.prev_steering_angle

            # If steering angle difference is larger than 0.1 degrees ~ 0.00174533 rad compute new steering_angle
            if np.fabs(steering_angle_diff) > 0.00174533:

                # Determine the direction of angle based on the angle difference
                if steering_angle_diff > 0:  # Need to increase angle
                    # compute new steering angle --> (current steering angle + steering_angle_velocity * dt)
                    steering_angle = self.prev_steering_angle + command[1] * command[4]
                    steering_angle = np.minimum(steering_angle, command[0])  # Don't exceed the target

                elif steering_angle_diff < 0:  # Need to decrease angle
                    # compute new steering angle --> (current steering angle + steering_angle_velocity * dt)
                    steering_angle = self.prev_steering_angle - command[1] * command[4]
                    steering_angle = np.maximum(steering_angle, command[0])  # Don't go below the target

        self.prev_steering_angle = steering_angle

        # If input steering angle is less than 0.9 degrees ~ 0.0157 rad
        if np.fabs(steering_angle) < 0.0157:

            self.left_wheel_angle = self.right_wheel_angle = 0.0

            self.wheel_rotation_velocity_FL = forward_vel / self.front_wheel_radius
            self.wheel_rotation_velocity_FR = forward_vel / self.front_wheel_radius
            self.wheel_rotation_velocity_BL = forward_vel / self.back_wheel_radius
            self.wheel_rotation_velocity_BR = forward_vel / self.back_wheel_radius

        else:
            # the distance between the center of rotation and the center of movement for robot body
            R = ((-1.0 if self.invert_steering else 1.0) * self.wheel_base) / np.tan(steering_angle)

            # Equations were simplied from the ones shown in https://www.mathworks.com/help/vdynblks/ref/kinematicsteering.html
            # compute the wheel angles by taking into account their offset from the center of the turning axle (where the bicycle model is centered), then computing the angles of each wheel relative to the turning point of the robot
            # Assuming four wheel drive with forward steering
            self.left_wheel_angle = np.arctan(self.wheel_base / (R - 0.5 * self.track_width))
            self.right_wheel_angle = np.arctan(self.wheel_base / (R + 0.5 * self.track_width))

            # distance of left and right wheels to middle of front axle
            steering_joint_half_dist = self.track_width / 2.0

            cy = np.fabs(R)

            # Calculate distances based on drive type and steering angle
            sign = 1.0 if steering_angle > 0 else -1.0  # 1 for positive, -1 for negative

            if self.invert_steering:
                # Rear wheel steering case

                # finding the distance between each wheel and the ICR
                self.wheel_dist_FL = cy - sign * steering_joint_half_dist
                self.wheel_dist_FR = cy + sign * steering_joint_half_dist
                self.wheel_dist_BL = np.sqrt(
                    np.square(cy - sign * steering_joint_half_dist) + np.square(self.wheel_base)
                )
                self.wheel_dist_BR = np.sqrt(
                    np.square(cy + sign * steering_joint_half_dist) + np.square(self.wheel_base)
                )

            else:
                # Forward steering case

                # finding the distance between each wheel and the ICR
                self.wheel_dist_FL = np.sqrt(
                    np.square(cy - sign * steering_joint_half_dist) + np.square(self.wheel_base)
                )
                self.wheel_dist_FR = np.sqrt(
                    np.square(cy + sign * steering_joint_half_dist) + np.square(self.wheel_base)
                )
                self.wheel_dist_BL = cy - sign * steering_joint_half_dist
                self.wheel_dist_BR = cy + sign * steering_joint_half_dist

            # angular velocity of the robot body
            body_ang_vel = forward_vel / cy

            # angular velocity of each wheel
            self.wheel_rotation_velocity_FL = body_ang_vel * (self.wheel_dist_FL / self.front_wheel_radius)
            self.wheel_rotation_velocity_FR = body_ang_vel * (self.wheel_dist_FR / self.front_wheel_radius)
            self.wheel_rotation_velocity_BL = body_ang_vel * (self.wheel_dist_BL / self.back_wheel_radius)
            self.wheel_rotation_velocity_BR = body_ang_vel * (self.wheel_dist_BR / self.back_wheel_radius)

        # clamp wheel rotation velocities to max wheel velocity
        self.wheel_rotation_velocity_FL = np.clip(
            self.wheel_rotation_velocity_FL, -self.max_wheel_velocity, self.max_wheel_velocity
        )
        self.wheel_rotation_velocity_FR = np.clip(
            self.wheel_rotation_velocity_FR, -self.max_wheel_velocity, self.max_wheel_velocity
        )
        self.wheel_rotation_velocity_BL = np.clip(
            self.wheel_rotation_velocity_BL, -self.max_wheel_velocity, self.max_wheel_velocity
        )
        self.wheel_rotation_velocity_BR = np.clip(
            self.wheel_rotation_velocity_BR, -self.max_wheel_velocity, self.max_wheel_velocity
        )

        # clamp wheel angles to max wheel rotation
        self.left_wheel_angle = np.clip(
            self.left_wheel_angle, -self.max_wheel_rotation_angle, self.max_wheel_rotation_angle
        )
        self.right_wheel_angle = np.clip(
            self.right_wheel_angle, -self.max_wheel_rotation_angle, self.max_wheel_rotation_angle
        )

        # output wheel rotation angular velocity and wheel angles
        return ArticulationAction(
            joint_velocities=(
                self.wheel_rotation_velocity_FL,
                self.wheel_rotation_velocity_FR,
                self.wheel_rotation_velocity_BL,
                self.wheel_rotation_velocity_BR,
            ),
            joint_positions=(self.left_wheel_angle, self.right_wheel_angle),
        )
