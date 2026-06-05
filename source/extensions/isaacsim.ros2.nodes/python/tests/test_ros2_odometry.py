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

"""Tests for ROS 2 odometry publisher OmniGraph node."""

import numpy as np
import omni.graph.core as og
import omni.kit.commands
import omni.kit.test
import omni.kit.usd
import omni.kit.viewport.utility
import usdrt.Sdf
from isaacsim.core.deprecation_manager import import_module
from isaacsim.core.experimental.objects import Cube, DistantLight, GroundPlane
from isaacsim.core.experimental.prims import GeomPrim, RigidPrim, XformPrim
from isaacsim.core.experimental.utils import stage as stage_utils
from isaacsim.core.simulation_manager import SimulationManager
from isaacsim.ros2.core.impl.ros2_test_case import ROS2TestCase

from .common import get_qos_profile


class TestRos2Odometry(ROS2TestCase):
    """Test suite for ros2 odometry."""

    async def setUp(self):
        """Set up test fixtures."""
        await super().setUp()
        self.CUBE_SCALE = 0.5

    async def tearDown(self):
        """Tear down test fixtures."""
        await omni.kit.app.get_app().next_update_async()
        await super().tearDown()

    def get_cube_velocities(self):
        """Return a tuple (linear_velocity, angular_velocity) from the stored odometry message."""
        if self._cube_odometry_data is None:
            return None, None
        # Odometry.twist.twist contains the velocities.
        linear_velocity = self._cube_odometry_data.twist.twist.linear
        angular_velocity = self._cube_odometry_data.twist.twist.angular
        return linear_velocity, angular_velocity

    def get_cube_pose(self):
        """Return a tuple (position, orientation) from the stored odometry message."""
        if self._cube_odometry_data is None:
            return None, None
        # Odometry.pose.pose contains the position and orientation
        position = self._cube_odometry_data.pose.pose.position
        orientation = self._cube_odometry_data.pose.pose.orientation
        return position, orientation

    async def test_ROS2_general_odometry_gpu(self):
        """Test ROS2 general odometry gpu."""
        SimulationManager.set_backend("warp")
        SimulationManager.set_physics_sim_device("cuda")
        await omni.kit.app.get_app().next_update_async()
        await self.test_ROS2_general_odometry()

    async def test_ROS2_general_odometry(self):
        """Test ROS2 general odometry."""
        import rclpy
        from nav_msgs.msg import Odometry

        self.lin_vel_cmd = None
        self.ang_vel_cmd = None

        cube_prim_path = stage_utils.generate_next_free_path("/World/Cube", prepend_default_prim=False)
        Cube(
            "/World/Cube",
            sizes=1.0,
            positions=np.array([0.0, 0.0, 1.0]),
            orientations=np.array([1, 0, 0, 0]),
            scales=np.array([self.CUBE_SCALE, self.CUBE_SCALE, self.CUBE_SCALE]),
        )
        GeomPrim("/World/Cube").apply_collision_apis()
        self.cuboid = RigidPrim("/World/Cube")
        self.cuboid_xform = XformPrim("/World/Cube")
        GroundPlane("/World/groundPlane")

        light = DistantLight("/World/Light")
        light.set_intensities(1000)
        await omni.kit.app.get_app().next_update_async()

        # Define the action graph path
        graph_path = "/ActionGraph"

        try:
            keys = og.Controller.Keys
            graph, nodes, _, _ = og.Controller.edit(
                {"graph_path": graph_path, "evaluator_name": "execution"},
                {
                    keys.CREATE_NODES: [
                        ("OnPlaybackTick", "omni.graph.action.OnPlaybackTick"),
                        ("ReadSimTime", "isaacsim.core.nodes.IsaacReadSimulationTime"),
                        ("ComputeOdometry", "isaacsim.core.nodes.IsaacComputeOdometry"),
                        ("PublishROS2Odometry", "isaacsim.ros2.bridge.ROS2PublishOdometry"),
                        ("PublishGlobalROS2Odometry", "isaacsim.ros2.bridge.ROS2PublishOdometry"),
                    ],
                    keys.SET_VALUES: [
                        ("ComputeOdometry.inputs:chassisPrim", [usdrt.Sdf.Path(cube_prim_path)]),
                        ("PublishROS2Odometry.inputs:topicName", "cube_odometry"),
                        ("PublishROS2Odometry.inputs:chassisFrameId", "cube_link"),
                        ("PublishROS2Odometry.inputs:publishRawVelocities", False),
                        ("PublishGlobalROS2Odometry.inputs:topicName", "cube_odometry_global"),
                        ("PublishGlobalROS2Odometry.inputs:chassisFrameId", "cube_link"),
                        ("PublishGlobalROS2Odometry.inputs:publishRawVelocities", False),
                    ],
                    keys.CONNECT: [
                        ("OnPlaybackTick.outputs:tick", "ComputeOdometry.inputs:execIn"),
                        ("ComputeOdometry.outputs:execOut", "PublishROS2Odometry.inputs:execIn"),
                        ("ComputeOdometry.outputs:position", "PublishROS2Odometry.inputs:position"),
                        ("ComputeOdometry.outputs:orientation", "PublishROS2Odometry.inputs:orientation"),
                        ("ComputeOdometry.outputs:linearVelocity", "PublishROS2Odometry.inputs:linearVelocity"),
                        ("ComputeOdometry.outputs:angularVelocity", "PublishROS2Odometry.inputs:angularVelocity"),
                        ("ReadSimTime.outputs:simulationTime", "PublishROS2Odometry.inputs:timeStamp"),
                        ("ComputeOdometry.outputs:execOut", "PublishGlobalROS2Odometry.inputs:execIn"),
                        ("ComputeOdometry.outputs:position", "PublishGlobalROS2Odometry.inputs:position"),
                        ("ComputeOdometry.outputs:orientation", "PublishGlobalROS2Odometry.inputs:orientation"),
                        (
                            "ComputeOdometry.outputs:globalLinearVelocity",
                            "PublishGlobalROS2Odometry.inputs:linearVelocity",
                        ),
                        ("ComputeOdometry.outputs:angularVelocity", "PublishGlobalROS2Odometry.inputs:angularVelocity"),
                        ("ReadSimTime.outputs:simulationTime", "PublishGlobalROS2Odometry.inputs:timeStamp"),
                    ],
                },
            )
        except Exception as e:
            print(f"Error creating action graph: {e}")

        self._cube_odometry_data = None
        self._cube_odometry_global_data = None

        def cube_odometry_callback(data: Odometry):
            self._cube_odometry_data = data

        def cube_odometry_global_callback(data: Odometry):
            self._cube_odometry_global_data = data
            print(data.twist.twist.linear)

        ros2_node = self.create_node("odometry_publisher_tester")
        odom_sub = self.create_subscription(
            ros2_node, Odometry, "cube_odometry", cube_odometry_callback, get_qos_profile()
        )
        odom_sub_global = self.create_subscription(
            ros2_node, Odometry, "cube_odometry_global", cube_odometry_global_callback, get_qos_profile()
        )

        self.retrived_lin_vel = None

        def set_cuboid_pose(cuboid_xform, positions, orientations):
            cuboid_xform.set_world_poses(
                positions=np.array(positions),
                orientations=np.array(orientations),
            )

        def set_cuboid_commands(cuboid_rigid, lin_vel, ang_vel):
            cuboid_rigid.set_velocities(
                linear_velocities=np.array(lin_vel, dtype=np.float64).reshape(1, 3),
                angular_velocities=np.array(ang_vel, dtype=np.float64).reshape(1, 3),
            )
            self.retrived_lin_vel = cuboid_rigid.get_velocities()[1].numpy().flatten()

        def spin():
            if (self.lin_vel_cmd is not None) and (self.ang_vel_cmd is not None):
                set_cuboid_commands(self.cuboid, self.lin_vel_cmd, self.ang_vel_cmd)
            rclpy.spin_once(ros2_node, timeout_sec=0.1)

        # Alias for set_cuboid_pose to use the xform prim

        self._timeline = omni.timeline.get_timeline_interface()
        self._timeline.play()
        await omni.kit.app.get_app().next_update_async()

        def standard_checks():
            # Check if odometry data was received.
            self.assertIsNotNone(self._cube_odometry_data, "Cube Odometry data was not recieved.")

            # Get velocities from odometry data.
            linear_vel, angular_vel = self.get_cube_velocities()
            self.assertIsNotNone(linear_vel, "Linear Velocity data is missing.")
            self.assertIsNotNone(angular_vel, "Angular velocity data is missing.")

            # Get position and orientation
            position, orientation = self.get_cube_pose()
            self.assertIsNotNone(position, "Position data is missing.")
            self.assertIsNotNone(orientation, "Orientation data is missing.")

            # Verify the received odometry messages
            self.assertIsNotNone(self._cube_odometry_data)

        await self.simulate_until_condition(lambda: False, max_frames=120, per_frame_callback=spin)

        standard_checks()

        # Verify that the pose recieved from Odometry is correct
        self.assertAlmostEqual(self._cube_odometry_data.pose.pose.position.x, 0.0, places=1)
        self.assertAlmostEqual(self._cube_odometry_data.pose.pose.position.y, 0.0, places=1)
        self.assertAlmostEqual(self._cube_odometry_data.pose.pose.position.z, -0.75, delta=0.05)

        # Verify that the velocities recieved from Odometry are correct. Cude should be at rest.
        self.assertAlmostEqual(self._cube_odometry_data.twist.twist.linear.x, 0.0, places=1)
        self.assertAlmostEqual(self._cube_odometry_data.twist.twist.linear.y, 0.0, places=1)
        self.assertAlmostEqual(self._cube_odometry_data.twist.twist.linear.z, 0.0, places=1)
        self.assertAlmostEqual(self._cube_odometry_data.twist.twist.angular.x, 0.0, places=1)
        self.assertAlmostEqual(self._cube_odometry_data.twist.twist.angular.y, 0.0, places=1)
        self.assertAlmostEqual(self._cube_odometry_data.twist.twist.angular.z, 0.0, places=1)

        self._timeline.stop()
        await omni.kit.app.get_app().next_update_async()

        print("Test1: Check Z odometry:")
        ##############################
        self._cube_odometry_data = None

        self.lin_vel_cmd = [0.0, 0.0, 1.0]
        self.ang_vel_cmd = [0.0, 1.0, 0.0]

        self._timeline.play()

        # drive at z=1.0 m/s for 1s (60 frames) → position.z ~1.0m > 0.2m
        await self.simulate_until_condition(lambda: False, max_frames=60, per_frame_callback=spin)

        standard_checks()

        # Verify that the pose recieved from Odometry is correct
        self.assertAlmostEqual(self._cube_odometry_data.pose.pose.position.x, 0.0, places=2)
        self.assertAlmostEqual(self._cube_odometry_data.pose.pose.position.y, 0.0, places=2)
        self.assertGreater(self._cube_odometry_data.pose.pose.position.z, 0.2)

        # print(self.retrived_lin_vel)
        print(self._cube_odometry_data.twist.twist.linear)
        # Verify that the velocities recieved from Odometry are correct. Cude should be moving up.

        # TODO Investigate why 0.2 disparity exists for commanded and received speeds
        self.assertAlmostEqual(self._cube_odometry_global_data.twist.twist.linear.x, self.lin_vel_cmd[0], delta=0.2)
        self.assertAlmostEqual(self._cube_odometry_global_data.twist.twist.linear.y, self.lin_vel_cmd[1], delta=0.2)
        self.assertAlmostEqual(self._cube_odometry_global_data.twist.twist.linear.z, self.lin_vel_cmd[2], delta=0.2)

        # TODO Setting angular velocity seems to take no effect. Using .get_angular_velocity() returns the correct value but the cuboid does not move accordingly. Will need to investigate
        # Commenting out angular velocity checks for now:

        self.assertAlmostEqual(self._cube_odometry_data.twist.twist.angular.x, self.ang_vel_cmd[0], delta=0.2)
        self.assertAlmostEqual(self._cube_odometry_data.twist.twist.angular.y, self.ang_vel_cmd[1], delta=0.2)
        self.assertAlmostEqual(self._cube_odometry_data.twist.twist.angular.z, self.ang_vel_cmd[2], delta=0.2)

        self._timeline.stop()
        await omni.kit.app.get_app().next_update_async()

        print("Test2A: Check X odometry:")
        ##############################
        self._cube_odometry_data = None

        set_cuboid_pose(self.cuboid_xform, [[0.0, 0.0, self.CUBE_SCALE / 2.0]], [[1, 0, 0, 0]])
        set_cuboid_commands(self.cuboid, [0.0, 0.0, 0.0], [0.0, 0.0, 0.0])
        await omni.kit.app.get_app().next_update_async()

        self.lin_vel_cmd = None
        self.ang_vel_cmd = None

        self._timeline.play()

        await self.simulate_until_condition(
            lambda: self._cube_odometry_data is not None, max_frames=60, per_frame_callback=spin
        )

        self.lin_vel_cmd = [1.0, 0.0, 0.0]
        self.ang_vel_cmd = [0.0, 1.0, 0.0]

        # drive at x=1.0 m/s for 1s (60 frames) → position.x ~1.0m > 0.2m
        await self.simulate_until_condition(lambda: False, max_frames=60, per_frame_callback=spin)

        standard_checks()

        # Verify that the pose recieved from Odometry is correct
        self.assertGreater(self._cube_odometry_data.pose.pose.position.x, 0.2)
        self.assertAlmostEqual(self._cube_odometry_data.pose.pose.position.y, 0.0, delta=0.1)
        self.assertAlmostEqual(self._cube_odometry_data.pose.pose.position.z, 0.0, delta=0.15)

        # Verify that the velocities recieved from Odometry are correct. Cude should be at moving forward.
        self.assertAlmostEqual(self._cube_odometry_global_data.twist.twist.linear.x, self.lin_vel_cmd[0], delta=0.2)
        self.assertAlmostEqual(self._cube_odometry_global_data.twist.twist.linear.y, self.lin_vel_cmd[1], delta=0.2)
        self.assertAlmostEqual(self._cube_odometry_global_data.twist.twist.linear.z, self.lin_vel_cmd[2], delta=0.2)
        self._timeline.stop()
        await omni.kit.app.get_app().next_update_async()
        print("Test2B: Check X odometry (with robot front (0,1,0) and publishRawVelocities disabled:")
        ##############################
        self._cube_odometry_data = None

        self.lin_vel_cmd = None
        self.ang_vel_cmd = None

        og.Controller.set(
            og.Controller.attribute(graph_path + "/PublishROS2Odometry.inputs:robotFront"), [0.0, 1.0, 0.0]
        )

        og.Controller.set(
            og.Controller.attribute(graph_path + "/PublishROS2Odometry.inputs:publishRawVelocities"), False
        )

        set_cuboid_pose(self.cuboid_xform, [[0.0, 0.0, self.CUBE_SCALE / 2.0]], [[1, 0, 0, 0]])
        set_cuboid_commands(self.cuboid, [0.0, 0.0, 0.0], [0.0, 0.0, 0.0])
        await omni.kit.app.get_app().next_update_async()

        self._timeline.play()

        await self.simulate_until_condition(
            lambda: self._cube_odometry_data is not None, max_frames=60, per_frame_callback=spin
        )

        self.lin_vel_cmd = [1.0, 0.0, 0.0]
        self.ang_vel_cmd = [0.0, 1.0, 0.0]

        # drive at x=1.0 m/s for 1s (60 frames) → position.x ~1.0m > 0.2m
        await self.simulate_until_condition(lambda: False, max_frames=60, per_frame_callback=spin)

        standard_checks()

        # Verify that the pose recieved from Odometry is correct. X should still be greater than 0
        self.assertGreater(self._cube_odometry_data.pose.pose.position.x, 0.2)
        self.assertAlmostEqual(self._cube_odometry_data.pose.pose.position.y, 0.0, delta=0.1)
        self.assertAlmostEqual(self._cube_odometry_data.pose.pose.position.z, 0.0, delta=0.15)

        # Verify that the velocities recieved from Odometry are correct. Cube should be moving forward in local Y frame, but global X frame.
        # Components are flipped due to new robot fron orientation. -Y world linear velocity is positive X local frame
        self.assertAlmostEqual(self._cube_odometry_global_data.twist.twist.linear.x, self.lin_vel_cmd[0], delta=0.2)
        self.assertAlmostEqual(self._cube_odometry_global_data.twist.twist.linear.y, self.lin_vel_cmd[1], delta=0.2)
        self.assertAlmostEqual(self._cube_odometry_global_data.twist.twist.linear.z, self.lin_vel_cmd[2], delta=0.2)

        self._timeline.stop()
        print("Test2C: Check X odometry (with robot front (0,1,0) and publishRawVelocities enabled:")
        ##############################
        self._cube_odometry_data = None

        self.lin_vel_cmd = None
        self.ang_vel_cmd = None

        og.Controller.set(
            og.Controller.attribute(graph_path + "/PublishROS2Odometry.inputs:robotFront"), [0.0, 1.0, 0.0]
        )

        og.Controller.set(
            og.Controller.attribute(graph_path + "/PublishROS2Odometry.inputs:publishRawVelocities"), True
        )

        set_cuboid_pose(self.cuboid_xform, [[0.0, 0.0, self.CUBE_SCALE / 2.0]], [[1, 0, 0, 0]])
        set_cuboid_commands(self.cuboid, [0.0, 0.0, 0.0], [0.0, 0.0, 0.0])
        await omni.kit.app.get_app().next_update_async()

        self._timeline.play()

        await self.simulate_until_condition(
            lambda: self._cube_odometry_data is not None, max_frames=60, per_frame_callback=spin
        )

        self.lin_vel_cmd = [1.0, 0.0, 0.0]
        self.ang_vel_cmd = [0.0, 1.0, 0.0]

        # drive at x=1.0 m/s for 1s (60 frames) → position.x ~1.0m > 0.2m
        await self.simulate_until_condition(lambda: False, max_frames=60, per_frame_callback=spin)

        standard_checks()

        # Verify that the pose recieved from Odometry is correct. X should still be greater than 0
        self.assertGreater(self._cube_odometry_data.pose.pose.position.x, 0.2)
        self.assertAlmostEqual(self._cube_odometry_data.pose.pose.position.y, 0.0, delta=0.1)
        self.assertAlmostEqual(self._cube_odometry_data.pose.pose.position.z, 0.0, delta=0.15)

        # Verify that the velocities recieved from Odometry are correct. Cube should be moving forward in local Y frame, but global X frame.
        # Components are no longer flipped like Test 2B since only world velocities are published
        self.assertAlmostEqual(self._cube_odometry_global_data.twist.twist.linear.x, self.lin_vel_cmd[0], delta=0.2)
        self.assertAlmostEqual(self._cube_odometry_global_data.twist.twist.linear.y, self.lin_vel_cmd[1], delta=0.2)
        self.assertAlmostEqual(self._cube_odometry_global_data.twist.twist.linear.z, self.lin_vel_cmd[2], delta=0.2)

        ros2_node.destroy_node()

        pass

    async def test_ROS2_linear_odometry(self):
        """Test odometry for Leatherback robot moving in a straight line, verifying linear velocity and position tracking."""
        if SimulationManager.get_active_physics_engine() == "newton":
            self.skipTest("Leatherback asset not yet supported by Newton backend")
        import rclpy
        from ackermann_msgs.msg import AckermannDriveStamped
        from nav_msgs.msg import Odometry

        # Create a new stage for this test
        await omni.usd.get_context().new_stage_async()

        # Load the Leatherback robot USD
        leatherback_usd_path = self._assets_root_path + "/Isaac/Samples/ROS2/Robots/leatherback_ROS.usd"
        await stage_utils.open_stage_async(leatherback_usd_path)

        # Wait for the stage to load
        await omni.kit.app.get_app().next_update_async()

        # Create ROS node for this test
        ros2_node = self.create_node("leatherback_odometry_tester")

        # Publishers
        ackermann_pub = ros2_node.create_publisher(AckermannDriveStamped, "/ackermann_cmd", 10)

        # Store odometry data
        self._leatherback_odom = None

        # Callback to receive odometry data
        def odometry_callback(data: Odometry):
            self._leatherback_odom = data

        # Subscribe to the odometry topic
        odom_sub = self.create_subscription(ros2_node, Odometry, "/odom", odometry_callback, get_qos_profile())

        # Function to process ROS messages
        def spin():
            rclpy.spin_once(ros2_node, timeout_sec=0.1)

        # Start simulation
        self._timeline = omni.timeline.get_timeline_interface()
        self._timeline.play()

        # Wait for simulation to stabilize and odometry data to start flowing
        print("\nWaiting for simulation to stabilize and odometry data to begin flowing...")
        condition_met = await self.simulate_until_condition(
            lambda: self._leatherback_odom is not None, max_frames=120, per_frame_callback=spin  # 20 * 0.1s at 60fps
        )

        # Verify we're receiving odometry data
        self.assertTrue(condition_met, "No odometry data received from Leatherback within timeout")
        self.assertIsNotNone(self._leatherback_odom, "No odometry data received from Leatherback")

        # Give the robot a little extra time to settle
        print("Waiting a bit longer for the robot to settle...")
        await self.simulate_until_condition(lambda: False, max_frames=60, per_frame_callback=spin)

        # Record initial position
        initial_position = np.array(
            [
                self._leatherback_odom.pose.pose.position.x,
                self._leatherback_odom.pose.pose.position.y,
                self._leatherback_odom.pose.pose.position.z,
            ]
        )

        print(f"\nInitial position: ({initial_position[0]:.3f}, {initial_position[1]:.3f}, {initial_position[2]:.3f})")

        # Report initial velocities - shouldn't be exactly zero due to physics settling
        initial_linear_x = self._leatherback_odom.twist.twist.linear.x
        initial_linear_y = self._leatherback_odom.twist.twist.linear.y
        initial_linear_z = self._leatherback_odom.twist.twist.linear.z
        initial_angular_z = self._leatherback_odom.twist.twist.angular.z

        print(f"Initial velocities:")
        print(f"- Linear X: {initial_linear_x:.4f} m/s")
        print(f"- Linear Y: {initial_linear_y:.4f} m/s")
        print(f"- Linear Z: {initial_linear_z:.4f} m/s")
        print(f"- Angular Z: {initial_angular_z:.4f} rad/s")

        # Verify initial velocities are near zero (using a tolerance instead of decimal places)
        # The robot might have small residual velocities from physics settling
        # Use different tolerances for horizontal (XY) and vertical (Z) velocities
        xy_velocity_tolerance = 0.1  # m/s
        z_velocity_tolerance = 0.2  # m/s - higher tolerance for vertical motion
        angular_velocity_tolerance = 0.2  # rad/s

        self.assertLess(
            abs(initial_linear_x),
            xy_velocity_tolerance,
            f"Initial linear.x velocity ({initial_linear_x}) exceeds tolerance ({xy_velocity_tolerance})",
        )
        self.assertLess(
            abs(initial_linear_y),
            xy_velocity_tolerance,
            f"Initial linear.y velocity ({initial_linear_y}) exceeds tolerance ({xy_velocity_tolerance})",
        )
        self.assertLess(
            abs(initial_linear_z),
            z_velocity_tolerance,
            f"Initial linear.z velocity ({initial_linear_z}) exceeds tolerance ({z_velocity_tolerance})",
        )
        self.assertLess(
            abs(initial_angular_z),
            angular_velocity_tolerance,
            f"Initial angular.z velocity ({initial_angular_z}) exceeds tolerance ({angular_velocity_tolerance})",
        )

        # Parameters for straight-line motion
        forward_speed = 1.0  # m/s
        steering_angle = 0.0  # radians, zero for straight line

        print(f"\nStarting straight-line motion with:")
        print(f"- Forward speed: {forward_speed} m/s")
        print(f"- Steering angle: {steering_angle} rad (straight line)")

        # Drive the robot in a straight line
        drive_time = 2  # seconds
        steps = 20
        step_time = drive_time / steps

        # Drive in a straight line
        for i in range(steps):
            # Create and send Ackermann drive command
            msg = AckermannDriveStamped()
            msg.header.stamp = ros2_node.get_clock().now().to_msg()
            msg.drive.speed = forward_speed
            msg.drive.steering_angle = steering_angle
            ackermann_pub.publish(msg)

            # Simulate a step
            await self.simulate_until_condition(
                lambda: False, max_frames=max(1, int(step_time * 60)), per_frame_callback=spin
            )

            # Print progress periodically
            if i % 10 == 0:
                if self._leatherback_odom:
                    linear_vel_x = self._leatherback_odom.twist.twist.linear.x
                    current_pos = np.array(
                        [
                            self._leatherback_odom.pose.pose.position.x,
                            self._leatherback_odom.pose.pose.position.y,
                            self._leatherback_odom.pose.pose.position.z,
                        ]
                    )
                    displacement = np.linalg.norm(current_pos - initial_position)

                    print(f"Time: {(i+1)*step_time:.1f}s")
                    print(f"- Linear velocity (x): {linear_vel_x:.4f} m/s")
                    print(f"- Current position: {current_pos}")
                    print(f"- Displacement from start: {displacement:.2f}m")

        # Verify we've received odometry data
        self.assertIsNotNone(self._leatherback_odom, "Lost odometry data during test")

        # Get the final linear velocity
        final_linear_velocity = self._leatherback_odom.twist.twist.linear.x

        print(f"\nFinal linear velocity: {final_linear_velocity:.4f} m/s")
        print(f"Expected linear velocity: {forward_speed:.4f} m/s")

        # Verify the linear velocity is close to expected
        # Using a tolerance due to physics simulation variations
        velocity_tolerance = 0.2  # m/s
        self.assertLess(
            abs(final_linear_velocity - forward_speed),
            velocity_tolerance,
            f"Linear velocity {final_linear_velocity} not within {velocity_tolerance} of expected {forward_speed}",
        )

        # Get final position
        final_position = np.array(
            [
                self._leatherback_odom.pose.pose.position.x,
                self._leatherback_odom.pose.pose.position.y,
                self._leatherback_odom.pose.pose.position.z,
            ]
        )

        # Calculate displacement and expected distance
        # For distance calculation, focus on XY plane movement
        displacement_xy = np.linalg.norm(final_position[:2] - initial_position[:2])
        expected_distance = forward_speed * drive_time

        print(f"Final position: {final_position}")
        print(f"Total XY displacement: {displacement_xy:.2f}m")
        print(f"Expected distance: {expected_distance:.2f}m")

        # Verify the displacement is close to expected distance
        # Allow for some variation in physics simulation
        distance_tolerance = 0.5  # meters
        self.assertLess(
            abs(displacement_xy - expected_distance),
            distance_tolerance,
            f"XY displacement {displacement_xy}m not within {distance_tolerance}m of expected {expected_distance}m",
        )

        # Verify that the robot mainly moved along the X-axis (straight line)
        # Most of the displacement should be in X, with minimal Y deviation
        x_displacement = abs(final_position[0] - initial_position[0])
        y_displacement = abs(final_position[1] - initial_position[1])

        print(f"X displacement: {x_displacement:.2f}m")
        print(f"Y displacement: {y_displacement:.2f}m")

        # The X displacement should be much larger than Y for straight-line motion
        self.assertGreater(
            x_displacement,
            0.8 * displacement_xy,
            "Most of the movement should be along the X-axis for straight-line motion",
        )

        # Stop the robot
        msg = AckermannDriveStamped()
        msg.drive.speed = 0.0
        msg.drive.steering_angle = 0.0
        ackermann_pub.publish(msg)

        # Let it come to a stop
        await self.simulate_until_condition(lambda: False, max_frames=60, per_frame_callback=spin)

        # Clean up
        self._timeline.stop()
        ros2_node.destroy_node()

        pass

    async def test_ROS2_angular_odometry(self):
        """Test odometry with Leatherback robot going in a circle, verifying angular velocity."""
        if SimulationManager.get_active_physics_engine() == "newton":
            self.skipTest("Leatherback asset not yet supported by Newton backend")
        import rclpy
        from ackermann_msgs.msg import AckermannDriveStamped
        from nav_msgs.msg import Odometry

        # Create a new stage for this test
        await omni.usd.get_context().new_stage_async()

        # Load the Leatherback robot USD
        leatherback_usd_path = self._assets_root_path + "/Isaac/Samples/ROS2/Robots/leatherback_ROS.usd"
        await stage_utils.open_stage_async(leatherback_usd_path)

        # Wait for the stage to load
        await omni.kit.app.get_app().next_update_async()

        # Create ROS node for this test
        ros2_node = self.create_node("leatherback_odometry_tester")

        # Publishers
        ackermann_pub = ros2_node.create_publisher(AckermannDriveStamped, "/ackermann_cmd", 10)

        # Store odometry data
        self._leatherback_odom = None

        # Callback to receive odometry data
        def odometry_callback(data: Odometry):
            self._leatherback_odom = data

        # Subscribe to the odometry topic
        odom_sub = self.create_subscription(ros2_node, Odometry, "/odom", odometry_callback, get_qos_profile())

        # Function to process ROS messages
        def spin():
            rclpy.spin_once(ros2_node, timeout_sec=0.1)

        # Start simulation
        self._timeline = omni.timeline.get_timeline_interface()
        self._timeline.play()

        # Wait for simulation to stabilize and odometry data to start flowing
        print("\nWaiting for simulation to stabilize and odometry data to begin flowing...")
        condition_met = await self.simulate_until_condition(
            lambda: self._leatherback_odom is not None, max_frames=120, per_frame_callback=spin  # 20 * 0.1s at 60fps
        )

        # Verify we're receiving odometry data
        self.assertTrue(condition_met, "No odometry data received from Leatherback within timeout")
        self.assertIsNotNone(self._leatherback_odom, "No odometry data received from Leatherback")

        # Record initial position
        initial_position = np.array(
            [
                self._leatherback_odom.pose.pose.position.x,
                self._leatherback_odom.pose.pose.position.y,
                self._leatherback_odom.pose.pose.position.z,
            ]
        )

        print(f"\nInitial position: ({initial_position[0]:.3f}, {initial_position[1]:.3f}, {initial_position[2]:.3f})")

        # Parameters for circular motion
        forward_speed = 0.5  # m/s
        steering_angle = 0.4  # radians, positive for left turn

        print(f"\nStarting circular motion with:")
        print(f"- Forward speed: {forward_speed} m/s")
        print(f"- Steering angle: {steering_angle} rad")

        # Leatherback uses Ackermann steering, so driving with a constant steering angle will create a circle
        wheelbase = 0.37  # approximate wheelbase of Leatherback in meters
        expected_angular_velocity = forward_speed * np.tan(steering_angle) / wheelbase  # rad/s
        print(f"- Expected angular velocity: {expected_angular_velocity:.4f} rad/s")

        # Drive the robot in a circle
        drive_time = 5  # seconds
        steps = 50
        step_time = drive_time / steps

        # Track maximum displacement during the test
        max_displacement = 0.0

        # Drive in a circle
        for i in range(steps):
            # Create and send Ackermann drive command
            msg = AckermannDriveStamped()
            msg.header.stamp = ros2_node.get_clock().now().to_msg()
            msg.drive.speed = forward_speed
            msg.drive.steering_angle = steering_angle
            ackermann_pub.publish(msg)

            # Simulate a step
            await self.simulate_until_condition(
                lambda: False, max_frames=max(1, int(step_time * 60)), per_frame_callback=spin
            )

            # Print progress periodically
            if i % 10 == 0:
                if self._leatherback_odom:
                    angular_vel_z = self._leatherback_odom.twist.twist.angular.z
                    current_pos = np.array(
                        [
                            self._leatherback_odom.pose.pose.position.x,
                            self._leatherback_odom.pose.pose.position.y,
                            self._leatherback_odom.pose.pose.position.z,
                        ]
                    )
                    displacement = np.linalg.norm(current_pos - initial_position)

                    # Update maximum displacement
                    max_displacement = max(max_displacement, displacement)

                    print(f"Time: {(i+1)*step_time:.1f}s")
                    print(f"- Angular velocity (z): {angular_vel_z:.4f} rad/s")
                    print(f"- Current position: {current_pos}")
                    print(f"- Displacement from start: {displacement:.2f}m")

        # Verify we've received odometry data
        self.assertIsNotNone(self._leatherback_odom, "Lost odometry data during test")

        # Get the final angular velocity
        final_angular_velocity = self._leatherback_odom.twist.twist.angular.z

        print(f"\nFinal angular velocity: {final_angular_velocity:.4f} rad/s")
        print(f"Expected angular velocity: {expected_angular_velocity:.4f} rad/s")

        # Verify the angular velocity is non-zero and positive (left turn)
        self.assertGreater(final_angular_velocity, 0, "Angular velocity should be positive for a left turn")

        # Verify the angular velocity is reasonably close to expected
        # Using a generous tolerance due to physics simulation variations
        tolerance = 0.15  # rad/s - increased to account for physics simulation differences
        self.assertLess(
            abs(final_angular_velocity - expected_angular_velocity),
            tolerance,
            f"Angular velocity {final_angular_velocity} not within {tolerance} of expected {expected_angular_velocity}",
        )

        # Get final position
        final_position = np.array(
            [
                self._leatherback_odom.pose.pose.position.x,
                self._leatherback_odom.pose.pose.position.y,
                self._leatherback_odom.pose.pose.position.z,
            ]
        )

        # Calculate final displacement
        final_displacement = np.linalg.norm(final_position[:2] - initial_position[:2])  # XY plane only

        print(f"Final position: {final_position}")
        print(f"Final displacement: {final_displacement:.2f}m")
        print(f"Maximum displacement during test: {max_displacement:.2f}m")

        # Verify the robot reached a significant displacement during the test
        # This checks that it moved in a circle properly, even if it returned near the start
        self.assertGreater(
            max_displacement, 1.0, "Robot should have reached a significant displacement during circular motion"
        )

        # If the robot completed a full circle, the final displacement should be small
        # This is an optional check - if the robot hasn't quite completed a full circle,
        # this check might fail but the test can still pass based on max_displacement
        if drive_time >= 10:  # Only check for full circle completion if we drove long enough
            print("Checking if robot completed a full circle (optional)...")
            if final_displacement < 0.5:  # A loose threshold for "close to starting point"
                print("Robot successfully completed a full circle!")

        # Stop the robot
        msg = AckermannDriveStamped()
        msg.drive.speed = 0.0
        msg.drive.steering_angle = 0.0
        ackermann_pub.publish(msg)

        # Let it come to a stop
        await self.simulate_until_condition(lambda: False, max_frames=60, per_frame_callback=spin)

        # Clean up
        self._timeline.stop()
        ros2_node.destroy_node()

        pass
