# SPDX-FileCopyrightText: Copyright (c) 2024-2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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
import asyncio
import sys

import carb
import numpy as np
import omni.graph.core as og
import omni.graph.core.tests as ogts
import omni.kit.test
import usdrt.Sdf
from isaacsim.core.api import World
from isaacsim.core.api.robots import Robot
from isaacsim.core.utils import rotations
from isaacsim.core.utils.physics import simulate_async
from isaacsim.core.utils.prims import create_prim, delete_prim, get_prim_at_path
from isaacsim.core.utils.stage import create_new_stage_async
from isaacsim.robot.wheeled_robots.controllers.ackermann_controller import AckermannController
from isaacsim.robot.wheeled_robots.robots import WheeledRobot
from isaacsim.storage.native import get_assets_root_path_async
from pxr import Gf


class TestAckermannController(omni.kit.test.AsyncTestCase):
    async def setUp(self):
        pass

    # ----------------------------------------------------------------------

    async def tearDown(self):
        pass

    # ----------------------------------------------------------------------

    async def test_ackermann_steering_control(self):

        # First case check that it snaps to correct angle, no steering velocity

        # These tests are only valid for positive angles and positive forward velocity
        wheel_base = 1.65
        track_width = 1.25
        wheel_radius = 0.25

        def controller_calcs(
            wheel_base, track_width, wheel_radius, desired_forward_vel, radius_of_turn, desired_steering_angle
        ):
            controller = AckermannController(
                "test_controller", wheel_base=wheel_base, track_width=track_width, front_wheel_radius=wheel_radius
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
            actions = controller.forward([desired_steering_angle, 0.0, desired_forward_vel, 0.0, 0.0])

            self.assertNotEquals(actions.joint_positions[0], None)
            self.assertNotEquals(actions.joint_positions[1], None)

            self.assertAlmostEquals(actions.joint_positions[0], expected_steering_angle_left, delta=0.001)
            self.assertAlmostEquals(actions.joint_positions[1], expected_steering_angle_right, delta=0.001)

            self.assertNotEquals(actions.joint_velocities[0], None)
            self.assertNotEquals(actions.joint_velocities[1], None)
            self.assertNotEquals(actions.joint_velocities[2], None)
            self.assertNotEquals(actions.joint_velocities[3], None)

            self.assertAlmostEquals(actions.joint_velocities[0], wheel_speed_front_l, delta=0.001)
            self.assertAlmostEquals(actions.joint_velocities[1], wheel_speed_front_r, delta=0.001)
            self.assertAlmostEquals(actions.joint_velocities[2], wheel_speed_back_l, delta=0.001)
            self.assertAlmostEquals(actions.joint_velocities[3], wheel_speed_back_r, delta=0.001)

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
            "test_controller", wheel_base=wheel_base, track_width=track_width, front_wheel_radius=wheel_radius
        )

        expected_joint_values = controller_calcs(
            wheel_base, track_width, wheel_radius, desired_forward_vel, radius_of_turn
        )

        for i in range(max_iter):
            # command (np.ndarray): [desired steering angle (rad), steering_angle_velocity (rad/s), desired velocity of robot (m/s), acceleration (m/s^2), delta time (s)]
            actions = controller.forward(
                [desired_steering_angle, steering_velocity, desired_forward_vel, acceleration, dt]
            )

            self.assertNotEquals(actions.joint_positions[0], None)
            self.assertNotEquals(actions.joint_positions[1], None)

            if i < num_iterations_steering:
                self.assertLess(actions.joint_positions[0], expected_joint_values[0])
                self.assertLess(actions.joint_positions[1], expected_joint_values[1])
            else:
                self.assertAlmostEquals(actions.joint_positions[0], expected_joint_values[0], delta=0.01)
                self.assertAlmostEquals(actions.joint_positions[1], expected_joint_values[1], delta=0.01)

            self.assertNotEquals(actions.joint_velocities[0], None)
            self.assertNotEquals(actions.joint_velocities[1], None)
            self.assertNotEquals(actions.joint_velocities[2], None)
            self.assertNotEquals(actions.joint_velocities[3], None)

            if i < num_iterations_acceleration:
                self.assertLess(actions.joint_velocities[0], expected_joint_values[2])
                self.assertLess(actions.joint_velocities[1], expected_joint_values[3])
                self.assertLess(actions.joint_velocities[2], expected_joint_values[4])
                self.assertLess(actions.joint_velocities[3], expected_joint_values[5])
            else:
                self.assertAlmostEquals(actions.joint_velocities[0], expected_joint_values[2], delta=0.01)
                self.assertAlmostEquals(actions.joint_velocities[1], expected_joint_values[3], delta=0.01)
                self.assertAlmostEquals(actions.joint_velocities[2], expected_joint_values[4], delta=0.01)
                self.assertAlmostEquals(actions.joint_velocities[3], expected_joint_values[5], delta=0.01)

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
            "test_controller", wheel_base=wheel_base, track_width=track_width, front_wheel_radius=wheel_radius
        )

        expected_joint_values = controller_calcs(
            wheel_base, track_width, wheel_radius, desired_forward_vel, radius_of_turn
        )

        for i in range(max_iter):
            # command (np.ndarray): [desired steering angle (rad), steering_angle_velocity (rad/s), desired velocity of robot (m/s), acceleration (m/s^2), delta time (s)]
            actions = controller.forward(
                [desired_steering_angle, steering_velocity, desired_forward_vel, acceleration, dt]
            )

            self.assertNotEquals(actions.joint_positions[0], None)
            self.assertNotEquals(actions.joint_positions[1], None)

            if i < num_iterations_steering:
                self.assertLess(actions.joint_positions[0], expected_joint_values[0])
                self.assertLess(actions.joint_positions[1], expected_joint_values[1])
            else:
                self.assertAlmostEquals(actions.joint_positions[0], expected_joint_values[0], delta=0.01)
                self.assertAlmostEquals(actions.joint_positions[1], expected_joint_values[1], delta=0.01)

            self.assertNotEquals(actions.joint_velocities[0], None)
            self.assertNotEquals(actions.joint_velocities[1], None)
            self.assertNotEquals(actions.joint_velocities[2], None)
            self.assertNotEquals(actions.joint_velocities[3], None)

            if i < num_iterations_acceleration:
                self.assertLess(actions.joint_velocities[0], expected_joint_values[2])
                self.assertLess(actions.joint_velocities[1], expected_joint_values[3])
                self.assertLess(actions.joint_velocities[2], expected_joint_values[4])
                self.assertLess(actions.joint_velocities[3], expected_joint_values[5])
            else:
                self.assertAlmostEquals(actions.joint_velocities[0], expected_joint_values[2], delta=0.01)
                self.assertAlmostEquals(actions.joint_velocities[1], expected_joint_values[3], delta=0.01)
                self.assertAlmostEquals(actions.joint_velocities[2], expected_joint_values[4], delta=0.01)
                self.assertAlmostEquals(actions.joint_velocities[3], expected_joint_values[5], delta=0.01)

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
            "test_controller", wheel_base=wheel_base, track_width=track_width, front_wheel_radius=wheel_radius
        )

        expected_joint_values = controller_calcs(
            wheel_base, track_width, wheel_radius, desired_forward_vel, radius_of_turn
        )

        for i in range(max_iter):
            # command (np.ndarray): [desired steering angle (rad), steering_angle_velocity (rad/s), desired velocity of robot (m/s), acceleration (m/s^2), delta time (s)]
            actions = controller.forward(
                [desired_steering_angle, steering_velocity, desired_forward_vel, acceleration, dt]
            )

            self.assertNotEquals(actions.joint_positions[0], None)
            self.assertNotEquals(actions.joint_positions[1], None)

            if i < num_iterations_steering:
                self.assertLess(actions.joint_positions[0], expected_joint_values[0])
                self.assertLess(actions.joint_positions[1], expected_joint_values[1])
            else:
                self.assertAlmostEquals(actions.joint_positions[0], expected_joint_values[0], delta=0.01)
                self.assertAlmostEquals(actions.joint_positions[1], expected_joint_values[1], delta=0.01)

            self.assertNotEquals(actions.joint_velocities[0], None)
            self.assertNotEquals(actions.joint_velocities[1], None)
            self.assertNotEquals(actions.joint_velocities[2], None)
            self.assertNotEquals(actions.joint_velocities[3], None)

            if i < num_iterations_acceleration:
                self.assertLess(actions.joint_velocities[0], expected_joint_values[2])
                self.assertLess(actions.joint_velocities[1], expected_joint_values[3])
                self.assertLess(actions.joint_velocities[2], expected_joint_values[4])
                self.assertLess(actions.joint_velocities[3], expected_joint_values[5])
            else:
                self.assertAlmostEquals(actions.joint_velocities[0], expected_joint_values[2], delta=0.01)
                self.assertAlmostEquals(actions.joint_velocities[1], expected_joint_values[3], delta=0.01)
                self.assertAlmostEquals(actions.joint_velocities[2], expected_joint_values[4], delta=0.01)
                self.assertAlmostEquals(actions.joint_velocities[3], expected_joint_values[5], delta=0.01)


class TestAckermannControllerOgn(ogts.OmniGraphTestCase):
    async def setUp(self):
        """Set up  test environment, to be torn down when done"""

        await create_new_stage_async()
        await self.setup_environment()
        await self.setup_ogn()

        await self.my_world.initialize_simulation_context_async()

    # ----------------------------------------------------------------------

    async def setup_environment(self):
        self.my_world = World(stage_units_in_meters=1.0, physics_dt=1.0 / 60, rendering_dt=1.0 / 60)
        self.my_world.scene.add_default_ground_plane(z_position=-0.03)

        self._timeline = omni.timeline.get_timeline_interface()

        self._assets_root_path = await get_assets_root_path_async()
        if self._assets_root_path is None:
            carb.log_error("Could not find Isaac Sim assets folder")
            return

        self.stage = omni.usd.get_context().get_stage()
        await omni.kit.app.get_app().next_update_async()

    # ----------------------------------------------------------------------
    async def setup_ogn(self):
        self.graph_path = "/ActionGraph"
        self.prim_path = "/World/Forklift"

        if get_prim_at_path(self.graph_path):
            delete_prim(self.graph_path)

    # ----------------------------------------------------------------------

    async def tearDown(self):
        """Get rid of temporary data used by the test"""
        await omni.kit.stage_templates.new_stage_async()
        self._timeline = None

    # ----------------------------------------------------------------------

    async def test_ackermann_controller_robot(self):
        # Add forklift USD
        create_prim(
            "/World/Forklift",
            "Xform",
            position=np.array([0, 0, 0.0]),
            usd_path=self._assets_root_path + "/Isaac/Robots/IsaacSim/ForkliftC/forklift_c.usd",
        )

        self._timeline = omni.timeline.get_timeline_interface()
        (
            test_acker_graph,
            [play_node, acker_node, art_steer_node, art_drive_node, compute_odom_node],
            _,
            _,
        ) = og.Controller.edit(
            {"graph_path": "/ActionGraph", "evaluator_name": "execution"},
            {
                og.Controller.Keys.CREATE_NODES: [
                    ("OnPlaybackTick", "omni.graph.action.OnPlaybackTick"),
                    ("AckermannController", "isaacsim.robot.wheeled_robots.AckermannController"),
                    ("ArticulationControllerSteer", "isaacsim.core.nodes.IsaacArticulationController"),
                    ("ArticulationControllerDrive", "isaacsim.core.nodes.IsaacArticulationController"),
                    ("ComputeOdometryNode", "isaacsim.core.nodes.IsaacComputeOdometry"),
                ],
                og.Controller.Keys.CONNECT: [
                    ("OnPlaybackTick.outputs:tick", "AckermannController.inputs:execIn"),
                    ("OnPlaybackTick.outputs:tick", "ArticulationControllerSteer.inputs:execIn"),
                    ("OnPlaybackTick.outputs:tick", "ArticulationControllerDrive.inputs:execIn"),
                    ("OnPlaybackTick.outputs:tick", "ComputeOdometryNode.inputs:execIn"),
                    ("OnPlaybackTick.outputs:deltaSeconds", "AckermannController.inputs:dt"),
                    ("AckermannController.outputs:wheelAngles", "ArticulationControllerSteer.inputs:positionCommand"),
                    (
                        "AckermannController.outputs:wheelRotationVelocity",
                        "ArticulationControllerDrive.inputs:velocityCommand",
                    ),
                ],
                og.Controller.Keys.SET_VALUES: [
                    ("AckermannController.inputs:invertSteering", True),
                    ("AckermannController.inputs:wheelBase", 1.65),
                    ("AckermannController.inputs:frontWheelRadius", 0.325),
                    ("AckermannController.inputs:backWheelRadius", 0.255),
                    ("AckermannController.inputs:trackWidth", 1.05),
                    ("ArticulationControllerSteer.inputs:robotPath", "/World/Forklift"),
                    (
                        "ArticulationControllerSteer.inputs:jointNames",
                        [
                            "left_rotator_joint",
                            "right_rotator_joint",
                        ],
                    ),
                    ("ArticulationControllerDrive.inputs:robotPath", "/World/Forklift"),
                    (
                        "ArticulationControllerDrive.inputs:jointNames",
                        [
                            "left_front_wheel_joint",
                            "right_front_wheel_joint",
                            "left_back_wheel_joint",
                            "right_back_wheel_joint",
                        ],
                    ),
                    (
                        "ComputeOdometryNode.inputs:chassisPrim",
                        [
                            "/World/Forklift",
                        ],
                    ),
                ],
            },
        )

        self._timeline.play()
        await omni.kit.app.get_app().next_update_async()

        # Move robot in a circle and check it is at quarter turn position.
        desired_forward_vel = 1.5  # m/s
        desired_steer_angle = 0.3  # rad
        wheel_base = og.Controller.attribute("inputs:wheelBase", acker_node).get()

        og.Controller.attribute("inputs:speed", acker_node).set(desired_forward_vel)
        og.Controller.attribute("inputs:steeringAngle", acker_node).set(desired_steer_angle)

        turning_radius = wheel_base / np.tan(desired_steer_angle)
        desired_ang_vel = desired_forward_vel / turning_radius

        total_expected_time = 2 * np.pi / desired_ang_vel

        def calculate_pose(r, w, t):
            return [
                r * np.cos(w * t + 1.5 * np.pi),
                r * np.sin(w * t + 1.5 * np.pi) + r,
                w * t,
            ]

        def standard_checks():
            lin_vel = og.Controller.attribute("outputs:linearVelocity", compute_odom_node).get()
            acceleration = og.Controller.attribute("outputs:linearAcceleration", compute_odom_node).get()
            ang_vel = og.Controller.attribute("outputs:angularVelocity", compute_odom_node).get()

            # Compare forward linear velocity in x axis to desired
            self.assertAlmostEquals(lin_vel[0], desired_forward_vel, delta=0.2)

            # Compare angular velocity in z axis to desired
            self.assertAlmostEquals(ang_vel[2], desired_ang_vel, delta=0.2)

            # Compare linear acceleration in x axis to 0
            self.assertAlmostEquals(acceleration[0], 0.0, delta=0.3)

        # Simulate quarter of circle turn
        await simulate_async(total_expected_time / 4.0)
        standard_checks()
        position = og.Controller.attribute("outputs:position", compute_odom_node).get()
        orientation = og.Controller.attribute("outputs:orientation", compute_odom_node).get()
        des_pose = calculate_pose(turning_radius, desired_ang_vel, total_expected_time / 4.0)
        curr_orientation = 2.0 * np.arctan2(orientation[2], orientation[3])
        # Compare pose to desired pose
        self.assertAlmostEquals(des_pose[0], position[0], delta=1)
        self.assertAlmostEquals(des_pose[1], position[1], delta=1)
        self.assertAlmostEquals(des_pose[2], curr_orientation, delta=0.3)
        self._timeline.stop()
        await omni.kit.app.get_app().next_update_async()

        # Simulate full circle turn
        desired_forward_vel = 1.5  # m/s
        desired_steer_angle = -0.3  # rad
        wheel_base = og.Controller.attribute("inputs:wheelBase", acker_node).get()

        og.Controller.attribute("inputs:speed", acker_node).set(desired_forward_vel)
        og.Controller.attribute("inputs:steeringAngle", acker_node).set(desired_steer_angle)

        turning_radius = wheel_base / np.tan(desired_steer_angle)
        desired_ang_vel = desired_forward_vel / turning_radius

        total_expected_time = 2 * np.pi / np.fabs(desired_ang_vel)
        self._timeline.play()

        # Simulate next quarter of circle turn
        await simulate_async(total_expected_time / 4.0)
        standard_checks()
        position = og.Controller.attribute("outputs:position", compute_odom_node).get()
        orientation = og.Controller.attribute("outputs:orientation", compute_odom_node).get()
        curr_orientation = 2.0 * np.arctan2(orientation[2], orientation[3])
        # Compare pose to desired pose
        self.assertAlmostEquals(np.fabs(des_pose[0]), np.fabs(position[0]), delta=1)
        self.assertAlmostEquals(np.fabs(des_pose[1]), np.fabs(position[1]), delta=1)
        self.assertAlmostEquals(np.fabs(des_pose[2]), np.fabs(curr_orientation), delta=0.3)

    # ----------------------------------------------------------------------

    async def test_ackermann_controller_robot_acceleration(self):
        # Add forklift USD
        create_prim(
            "/World/Forklift",
            "Xform",
            position=np.array([0, 0, 0.0]),
            usd_path=self._assets_root_path + "/Isaac/Robots/IsaacSim/ForkliftC/forklift_c.usd",
        )

        self._timeline = omni.timeline.get_timeline_interface()
        (
            test_acker_graph,
            [play_node, acker_node, art_steer_node, art_drive_node, compute_odom_node],
            _,
            _,
        ) = og.Controller.edit(
            {"graph_path": "/ActionGraph", "evaluator_name": "execution"},
            {
                og.Controller.Keys.CREATE_NODES: [
                    ("OnPlaybackTick", "omni.graph.action.OnPlaybackTick"),
                    ("AckermannController", "isaacsim.robot.wheeled_robots.AckermannController"),
                    ("ArticulationControllerSteer", "isaacsim.core.nodes.IsaacArticulationController"),
                    ("ArticulationControllerDrive", "isaacsim.core.nodes.IsaacArticulationController"),
                    ("ComputeOdometryNode", "isaacsim.core.nodes.IsaacComputeOdometry"),
                ],
                og.Controller.Keys.CONNECT: [
                    ("OnPlaybackTick.outputs:tick", "AckermannController.inputs:execIn"),
                    ("OnPlaybackTick.outputs:tick", "ArticulationControllerSteer.inputs:execIn"),
                    ("OnPlaybackTick.outputs:tick", "ArticulationControllerDrive.inputs:execIn"),
                    ("OnPlaybackTick.outputs:tick", "ComputeOdometryNode.inputs:execIn"),
                    ("OnPlaybackTick.outputs:deltaSeconds", "AckermannController.inputs:dt"),
                    ("AckermannController.outputs:wheelAngles", "ArticulationControllerSteer.inputs:positionCommand"),
                    (
                        "AckermannController.outputs:wheelRotationVelocity",
                        "ArticulationControllerDrive.inputs:velocityCommand",
                    ),
                ],
                og.Controller.Keys.SET_VALUES: [
                    ("AckermannController.inputs:invertSteering", True),
                    ("AckermannController.inputs:wheelBase", 1.65),
                    ("AckermannController.inputs:frontWheelRadius", 0.325),
                    ("AckermannController.inputs:backWheelRadius", 0.255),
                    ("AckermannController.inputs:trackWidth", 1.05),
                    ("ArticulationControllerSteer.inputs:robotPath", "/World/Forklift"),
                    (
                        "ArticulationControllerSteer.inputs:jointNames",
                        [
                            "left_rotator_joint",
                            "right_rotator_joint",
                        ],
                    ),
                    ("ArticulationControllerDrive.inputs:robotPath", "/World/Forklift"),
                    (
                        "ArticulationControllerDrive.inputs:jointNames",
                        [
                            "left_front_wheel_joint",
                            "right_front_wheel_joint",
                            "left_back_wheel_joint",
                            "right_back_wheel_joint",
                        ],
                    ),
                    (
                        "ComputeOdometryNode.inputs:chassisPrim",
                        [
                            "/World/Forklift",
                        ],
                    ),
                ],
            },
        )

        self._timeline.play()
        await omni.kit.app.get_app().next_update_async()

        # Move robot in a circle and check it is at quarter turn position.
        desired_forward_vel = 1.5  # m/s
        desired_steer_angle = 0.0  # rad

        steering_angle_vel = 0.05
        acceleration = 0.4

        og.Controller.attribute("inputs:speed", acker_node).set(desired_forward_vel)
        og.Controller.attribute("inputs:steeringAngle", acker_node).set(desired_steer_angle)
        og.Controller.attribute("inputs:steeringAngleVelocity", acker_node).set(steering_angle_vel)
        og.Controller.attribute("inputs:acceleration", acker_node).set(acceleration)

        curr_lin_vel = og.Controller.attribute("outputs:linearVelocity", compute_odom_node).get()
        curr_accel = og.Controller.attribute("outputs:linearAcceleration", compute_odom_node).get()

        await simulate_async(0.5)

        # Compare forward linear velocity in x axis to desired
        self.assertLess(curr_lin_vel[0], desired_forward_vel)

        # Compare linear acceleration in x axis to desired
        self.assertAlmostEquals(curr_accel[0], acceleration, delta=0.1)

        await simulate_async(4.0)

        # Compare forward linear velocity in x axis to desired
        self.assertAlmostEquals(curr_lin_vel[0], desired_forward_vel, delta=0.2)

        # Compare linear acceleration in x axis to 0
        self.assertAlmostEquals(curr_accel[0], 0.0, delta=0.1)

    async def test_ackermann_controller_robot_steer_velocity(self):
        # Add forklift USD
        create_prim(
            "/World/Forklift",
            "Xform",
            position=np.array([0, 0, 0.0]),
            usd_path=self._assets_root_path + "/Isaac/Robots/IsaacSim/ForkliftC/forklift_c.usd",
        )

        self._timeline = omni.timeline.get_timeline_interface()
        (
            test_acker_graph,
            [play_node, acker_node, art_steer_node, art_drive_node, compute_odom_node],
            _,
            _,
        ) = og.Controller.edit(
            {"graph_path": "/ActionGraph", "evaluator_name": "execution"},
            {
                og.Controller.Keys.CREATE_NODES: [
                    ("OnPlaybackTick", "omni.graph.action.OnPlaybackTick"),
                    ("AckermannController", "isaacsim.robot.wheeled_robots.AckermannController"),
                    ("ArticulationControllerSteer", "isaacsim.core.nodes.IsaacArticulationController"),
                    ("ArticulationControllerDrive", "isaacsim.core.nodes.IsaacArticulationController"),
                    ("ComputeOdometryNode", "isaacsim.core.nodes.IsaacComputeOdometry"),
                ],
                og.Controller.Keys.CONNECT: [
                    ("OnPlaybackTick.outputs:tick", "AckermannController.inputs:execIn"),
                    ("OnPlaybackTick.outputs:tick", "ArticulationControllerSteer.inputs:execIn"),
                    ("OnPlaybackTick.outputs:tick", "ArticulationControllerDrive.inputs:execIn"),
                    ("OnPlaybackTick.outputs:tick", "ComputeOdometryNode.inputs:execIn"),
                    ("OnPlaybackTick.outputs:deltaSeconds", "AckermannController.inputs:dt"),
                    ("AckermannController.outputs:wheelAngles", "ArticulationControllerSteer.inputs:positionCommand"),
                    (
                        "AckermannController.outputs:wheelRotationVelocity",
                        "ArticulationControllerDrive.inputs:velocityCommand",
                    ),
                ],
                og.Controller.Keys.SET_VALUES: [
                    ("AckermannController.inputs:invertSteering", True),
                    ("AckermannController.inputs:wheelBase", 1.65),
                    ("AckermannController.inputs:frontWheelRadius", 0.325),
                    ("AckermannController.inputs:backWheelRadius", 0.255),
                    ("AckermannController.inputs:trackWidth", 1.05),
                    ("ArticulationControllerSteer.inputs:robotPath", "/World/Forklift"),
                    (
                        "ArticulationControllerSteer.inputs:jointNames",
                        [
                            "left_rotator_joint",
                            "right_rotator_joint",
                        ],
                    ),
                    ("ArticulationControllerDrive.inputs:robotPath", "/World/Forklift"),
                    (
                        "ArticulationControllerDrive.inputs:jointNames",
                        [
                            "left_front_wheel_joint",
                            "right_front_wheel_joint",
                            "left_back_wheel_joint",
                            "right_back_wheel_joint",
                        ],
                    ),
                    (
                        "ComputeOdometryNode.inputs:chassisPrim",
                        [
                            "/World/Forklift",
                        ],
                    ),
                ],
            },
        )

        robot = Robot(prim_path="/World/Forklift", name="Forklift")

        self._timeline.play()
        await omni.kit.app.get_app().next_update_async()

        robot.initialize()
        left_rotator_joint_index = robot.dof_names.index("left_rotator_joint")
        right_rotator_joint_index = robot.dof_names.index("right_rotator_joint")

        joint_pos = robot.get_joint_positions()

        sign = 1.0

        if og.Controller.attribute("outputs:deltaSeconds", play_node).get():
            sign = -1.0

        self.assertAlmostEquals(sign * joint_pos[left_rotator_joint_index], 0.0, delta=0.05)
        self.assertAlmostEquals(sign * joint_pos[right_rotator_joint_index], 0.0, delta=0.05)

        desired_forward_vel = 0.0  # m/s
        desired_steer_angle = 0.4  # rad

        steering_angle_vel = 0.1
        acceleration = 0.4

        og.Controller.attribute("inputs:speed", acker_node).set(desired_forward_vel)
        og.Controller.attribute("inputs:steeringAngle", acker_node).set(desired_steer_angle)
        og.Controller.attribute("inputs:steeringAngleVelocity", acker_node).set(steering_angle_vel)
        og.Controller.attribute("inputs:acceleration", acker_node).set(acceleration)

        await simulate_async(1.0)
        joint_pos = robot.get_joint_positions()
        self.assertLess(sign * joint_pos[left_rotator_joint_index], desired_steer_angle)
        self.assertLess(sign * joint_pos[right_rotator_joint_index], desired_steer_angle)

        await simulate_async(2.0)
        joint_pos = robot.get_joint_positions()
        self.assertAlmostEquals(sign * joint_pos[left_rotator_joint_index], desired_steer_angle, delta=0.2)
        self.assertAlmostEquals(sign * joint_pos[right_rotator_joint_index], desired_steer_angle, delta=0.2)

        self._timeline.stop()
        await omni.kit.app.get_app().next_update_async()

        # Test in other direction with faster steering velocity
        robot = Robot(prim_path="/World/Forklift", name="Forklift")

        self._timeline.play()
        await omni.kit.app.get_app().next_update_async()

        robot.initialize()
        left_rotator_joint_index = robot.dof_names.index("left_rotator_joint")
        right_rotator_joint_index = robot.dof_names.index("right_rotator_joint")

        joint_pos = robot.get_joint_positions()

        sign = 1.0

        if og.Controller.attribute("outputs:deltaSeconds", play_node).get():
            sign = -1.0

        self.assertAlmostEquals(sign * joint_pos[left_rotator_joint_index], 0.0, delta=0.05)
        self.assertAlmostEquals(sign * joint_pos[right_rotator_joint_index], 0.0, delta=0.05)

        desired_forward_vel = 0.0  # m/s
        desired_steer_angle = -0.4  # rad

        steering_angle_vel = 0.2
        acceleration = 0.4

        og.Controller.attribute("inputs:speed", acker_node).set(desired_forward_vel)
        og.Controller.attribute("inputs:steeringAngle", acker_node).set(desired_steer_angle)
        og.Controller.attribute("inputs:steeringAngleVelocity", acker_node).set(steering_angle_vel)
        og.Controller.attribute("inputs:acceleration", acker_node).set(acceleration)

        await simulate_async(0.5)
        joint_pos = robot.get_joint_positions()
        self.assertGreater(sign * joint_pos[left_rotator_joint_index], desired_steer_angle)
        self.assertGreater(sign * joint_pos[right_rotator_joint_index], desired_steer_angle)

        await simulate_async(4.0)
        joint_pos = robot.get_joint_positions()
        self.assertAlmostEquals(sign * joint_pos[left_rotator_joint_index], desired_steer_angle, delta=0.2)
        self.assertAlmostEquals(sign * joint_pos[right_rotator_joint_index], desired_steer_angle, delta=0.2)
