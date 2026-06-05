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

"""Unit tests for the AckermannController OmniGraph node."""

import carb
import isaacsim.core.experimental.utils.app as app_utils
import isaacsim.core.experimental.utils.stage as stage_utils
import numpy as np
import omni.graph.core as og
import omni.graph.core.tests as ogts
import omni.kit.test
from isaacsim.core.experimental.objects import GroundPlane
from isaacsim.core.experimental.prims import Articulation
from isaacsim.core.simulation_manager import SimulationManager
from isaacsim.storage.native import get_assets_root_path_async


class TestAckermannControllerOgn(ogts.OmniGraphTestCase):
    """Test suite for validating Ackermann controller functionality in OmniGraph environments.

    This class provides comprehensive testing of the AckermannController OmniGraph node through multiple
    scenarios including basic steering control, acceleration dynamics, and steering velocity behavior.
    Tests utilize a forklift robot model to validate controller performance in realistic simulation
    conditions with proper physics integration.

    The test suite covers three main areas:
    1. Basic Ackermann steering control with circular motion validation
    2. Acceleration control testing for gradual speed changes
    3. Steering velocity control for smooth angle transitions

    All tests verify that the controller produces expected joint positions and velocities while
    maintaining proper kinematic relationships for Ackermann steering geometry.
    """

    async def setUp(self) -> None:
        """Set up test environment, to be torn down when done."""
        await stage_utils.create_new_stage_async()
        stage_utils.set_stage_up_axis("Z")
        stage_utils.set_stage_units(meters_per_unit=1.0)
        GroundPlane("/World/GroundPlane", positions=[0, 0, -0.03])

        self._assets_root_path = await get_assets_root_path_async()
        if self._assets_root_path is None:
            carb.log_error("Could not find Isaac Sim assets folder")
            return

        await app_utils.update_app_async()
        SimulationManager.setup_simulation(dt=1.0 / 60.0)

        self.graph_path = "/ActionGraph"
        self.prim_path = "/World/Forklift"

        stage = stage_utils.get_current_stage()
        graph_prim = stage.GetPrimAtPath(self.graph_path)
        if graph_prim.IsValid():
            stage_utils.delete_prim(self.graph_path)

    # ----------------------------------------------------------------------

    async def tearDown(self) -> None:
        """Get rid of temporary data used by the test."""
        app_utils.stop()
        await app_utils.update_app_async()
        await omni.kit.stage_templates.new_stage_async()

    # ----------------------------------------------------------------------

    async def test_ackermann_controller_robot(self) -> None:
        """Test the Ackermann controller with a robot in circular motion.

        Creates a forklift robot and sets up an OmniGraph with Ackermann controller nodes. Tests the robot's
        ability to follow circular paths by commanding forward velocity and steering angle, then verifies the
        robot's position, orientation, linear velocity, and angular velocity match expected values for quarter
        and full circle turns.
        """
        stage_utils.add_reference_to_stage(
            self._assets_root_path + "/Isaac/Robots/IsaacSim/ForkliftC/forklift_c.usd",
            "/World/Forklift",
        )

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

        app_utils.play()
        await app_utils.update_app_async()

        # Move robot in a circle and check it is at quarter turn position.
        desired_forward_vel = 1.5  # m/s
        desired_steer_angle = 0.3  # rad
        wheel_base = og.Controller.attribute("inputs:wheelBase", acker_node).get()

        og.Controller.attribute("inputs:speed", acker_node).set(desired_forward_vel)
        og.Controller.attribute("inputs:steeringAngle", acker_node).set(desired_steer_angle)

        turning_radius = wheel_base / np.tan(desired_steer_angle)
        desired_ang_vel = desired_forward_vel / turning_radius

        total_expected_time = 2 * np.pi / desired_ang_vel

        def calculate_pose(r: float, w: float, t: float) -> list[float]:
            return [
                r * np.cos(w * t + 1.5 * np.pi),
                r * np.sin(w * t + 1.5 * np.pi) + r,
                w * t,
            ]

        def standard_checks() -> None:
            lin_vel = og.Controller.attribute("outputs:linearVelocity", compute_odom_node).get()
            acceleration = og.Controller.attribute("outputs:linearAcceleration", compute_odom_node).get()
            ang_vel = og.Controller.attribute("outputs:angularVelocity", compute_odom_node).get()

            self.assertAlmostEqual(lin_vel[0], desired_forward_vel, delta=0.2)
            self.assertAlmostEqual(ang_vel[2], desired_ang_vel, delta=0.2)
            self.assertAlmostEqual(acceleration[0], 0.0, delta=0.3)

        # Simulate quarter of circle turn
        await app_utils.update_app_async(steps=int(total_expected_time / 4.0 * 60))
        standard_checks()
        position = og.Controller.attribute("outputs:position", compute_odom_node).get()
        orientation = og.Controller.attribute("outputs:orientation", compute_odom_node).get()
        des_pose = calculate_pose(turning_radius, desired_ang_vel, total_expected_time / 4.0)
        curr_orientation = 2.0 * np.arctan2(orientation[2], orientation[3])
        self.assertAlmostEqual(des_pose[0], position[0], delta=1)
        self.assertAlmostEqual(des_pose[1], position[1], delta=1)
        self.assertAlmostEqual(des_pose[2], curr_orientation, delta=0.3)
        app_utils.stop()
        await app_utils.update_app_async()

        # Simulate full circle turn
        desired_forward_vel = 1.5  # m/s
        desired_steer_angle = -0.3  # rad
        wheel_base = og.Controller.attribute("inputs:wheelBase", acker_node).get()

        og.Controller.attribute("inputs:speed", acker_node).set(desired_forward_vel)
        og.Controller.attribute("inputs:steeringAngle", acker_node).set(desired_steer_angle)

        turning_radius = wheel_base / np.tan(desired_steer_angle)
        desired_ang_vel = desired_forward_vel / turning_radius

        total_expected_time = 2 * np.pi / np.fabs(desired_ang_vel)
        app_utils.play()

        # Simulate next quarter of circle turn
        await app_utils.update_app_async(steps=int(total_expected_time / 4.0 * 60))
        standard_checks()
        position = og.Controller.attribute("outputs:position", compute_odom_node).get()
        orientation = og.Controller.attribute("outputs:orientation", compute_odom_node).get()
        curr_orientation = 2.0 * np.arctan2(orientation[2], orientation[3])
        self.assertAlmostEqual(np.fabs(des_pose[0]), np.fabs(position[0]), delta=1)
        self.assertAlmostEqual(np.fabs(des_pose[1]), np.fabs(position[1]), delta=1)
        self.assertAlmostEqual(np.fabs(des_pose[2]), np.fabs(curr_orientation), delta=0.3)

    # ----------------------------------------------------------------------

    async def test_ackermann_controller_robot_acceleration(self) -> None:
        """Test the Ackermann controller's acceleration behavior.

        Creates a forklift robot with an Ackermann controller and tests the robot's acceleration response.
        Verifies that the robot gradually accelerates to the desired velocity and that the linear acceleration
        matches the commanded acceleration during the ramp-up phase, then drops to zero once the target
        velocity is reached.
        """
        stage_utils.add_reference_to_stage(
            self._assets_root_path + "/Isaac/Robots/IsaacSim/ForkliftC/forklift_c.usd",
            "/World/Forklift",
        )

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

        app_utils.play()
        await app_utils.update_app_async()

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

        await app_utils.update_app_async(steps=30)

        self.assertLess(curr_lin_vel[0], desired_forward_vel)
        self.assertAlmostEqual(curr_accel[0], acceleration, delta=0.2)

        await app_utils.update_app_async(steps=480)

        self.assertAlmostEqual(curr_lin_vel[0], desired_forward_vel, delta=0.5)
        self.assertAlmostEqual(curr_accel[0], 0.0, delta=0.2)

    async def test_ackermann_controller_robot_steer_velocity(self) -> None:
        """Test the Ackermann controller's steering velocity behavior.

        Creates a forklift robot with an Ackermann controller and tests the robot's steering response with
        specified steering angle velocity. Verifies that the steering joints gradually rotate to the desired
        angle at the commanded velocity rate, testing both positive and negative steering angles with different
        steering velocities.
        """
        stage_utils.add_reference_to_stage(
            self._assets_root_path + "/Isaac/Robots/IsaacSim/ForkliftC/forklift_c.usd",
            "/World/Forklift",
        )

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

        articulation = Articulation("/World/Forklift")

        app_utils.play()
        await app_utils.update_app_async()

        left_rotator_joint_index = articulation.dof_names.index("left_rotator_joint")
        right_rotator_joint_index = articulation.dof_names.index("right_rotator_joint")

        joint_pos = articulation.get_dof_positions().numpy()[0]

        sign = 1.0

        if og.Controller.attribute("outputs:deltaSeconds", play_node).get():
            sign = -1.0

        self.assertAlmostEqual(sign * joint_pos[left_rotator_joint_index], 0.0, delta=0.05)
        self.assertAlmostEqual(sign * joint_pos[right_rotator_joint_index], 0.0, delta=0.05)

        desired_forward_vel = 0.0  # m/s
        desired_steer_angle = 0.4  # rad

        steering_angle_vel = 0.1
        acceleration = 0.4

        og.Controller.attribute("inputs:speed", acker_node).set(desired_forward_vel)
        og.Controller.attribute("inputs:steeringAngle", acker_node).set(desired_steer_angle)
        og.Controller.attribute("inputs:steeringAngleVelocity", acker_node).set(steering_angle_vel)
        og.Controller.attribute("inputs:acceleration", acker_node).set(acceleration)

        await app_utils.update_app_async(steps=60)
        joint_pos = articulation.get_dof_positions().numpy()[0]
        self.assertLess(sign * joint_pos[left_rotator_joint_index], desired_steer_angle)
        self.assertLess(sign * joint_pos[right_rotator_joint_index], desired_steer_angle)

        await app_utils.update_app_async(steps=120)
        joint_pos = articulation.get_dof_positions().numpy()[0]
        self.assertAlmostEqual(sign * joint_pos[left_rotator_joint_index], desired_steer_angle, delta=0.2)
        self.assertAlmostEqual(sign * joint_pos[right_rotator_joint_index], desired_steer_angle, delta=0.2)

        app_utils.stop()
        await app_utils.update_app_async()

        # Test in other direction with faster steering velocity
        articulation = Articulation("/World/Forklift")

        app_utils.play()
        await app_utils.update_app_async()

        left_rotator_joint_index = articulation.dof_names.index("left_rotator_joint")
        right_rotator_joint_index = articulation.dof_names.index("right_rotator_joint")

        joint_pos = articulation.get_dof_positions().numpy()[0]

        sign = 1.0

        if og.Controller.attribute("outputs:deltaSeconds", play_node).get():
            sign = -1.0

        self.assertAlmostEqual(sign * joint_pos[left_rotator_joint_index], 0.0, delta=0.05)
        self.assertAlmostEqual(sign * joint_pos[right_rotator_joint_index], 0.0, delta=0.05)

        desired_forward_vel = 0.0  # m/s
        desired_steer_angle = -0.4  # rad

        steering_angle_vel = 0.2
        acceleration = 0.4

        og.Controller.attribute("inputs:speed", acker_node).set(desired_forward_vel)
        og.Controller.attribute("inputs:steeringAngle", acker_node).set(desired_steer_angle)
        og.Controller.attribute("inputs:steeringAngleVelocity", acker_node).set(steering_angle_vel)
        og.Controller.attribute("inputs:acceleration", acker_node).set(acceleration)

        await app_utils.update_app_async(steps=30)
        joint_pos = articulation.get_dof_positions().numpy()[0]
        self.assertGreater(sign * joint_pos[left_rotator_joint_index], desired_steer_angle)
        self.assertGreater(sign * joint_pos[right_rotator_joint_index], desired_steer_angle)

        await app_utils.update_app_async(steps=240)
        joint_pos = articulation.get_dof_positions().numpy()[0]
        self.assertAlmostEqual(sign * joint_pos[left_rotator_joint_index], desired_steer_angle, delta=0.2)
        self.assertAlmostEqual(sign * joint_pos[right_rotator_joint_index], desired_steer_angle, delta=0.2)
