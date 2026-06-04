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

"""Tests for ROS 2 differential base OmniGraph node."""

from copy import deepcopy

import carb
import omni.graph.core as og
import omni.kit.commands
import omni.kit.test
import omni.kit.usd
import usdrt.Sdf
from isaacsim.core.simulation_manager import SimulationManager
from isaacsim.ros2.core.impl.ros2_test_case import ROS2TestCase
from pxr import Gf

from .common import (
    add_carter,
    add_carter_ros,
    add_nova_carter_ros,
    get_qos_profile,
    set_rotate,
    set_translate,
)


class TestRos2DifferentialBase(ROS2TestCase):
    """Test suite for ros2 differential base."""

    async def setUp(self):
        """Set up test fixtures."""
        await super().setUp()
        await omni.usd.get_context().new_stage_async()
        await omni.kit.app.get_app().next_update_async()
        SimulationManager.enable_fabric(enable=False)

        # Initialize class members
        self._trans = None
        self._odom_data = None

        # Create ROS2 node for this test

        self.node = self.create_node("isaac_sim_test_diff_drive")

    async def tearDown(self):
        """Tear down test fixtures."""
        self.node = None

        # Reset class members
        self._trans = None
        self._odom_data = None

        await super().tearDown()

    def spin(self):
        """Spin the ROS2 node once with a timeout."""
        import rclpy

        rclpy.spin_once(self.node, timeout_sec=0.1)

    def tf_callback(self, data):
        """Handle tf callback."""
        self._trans = data.transforms[-1]

    def odom_callback(self, data):
        """Handle odom callback."""
        self._odom_data = data.pose.pose

    def move_cmd_msg(self, x, y, z, ax, ay, az):
        """Create a move cmd msg message."""
        from geometry_msgs.msg import Twist

        msg = Twist()
        msg.linear.x = x
        msg.linear.y = y
        msg.linear.z = z
        msg.angular.x = ax
        msg.angular.y = ay
        msg.angular.z = az
        return msg

    async def test_carter_differential_base(self):
        """Test carter differential base."""
        if SimulationManager.get_active_physics_engine() == "newton":
            self.skipTest("Odometry node not yet supported by Newton backend")
        from geometry_msgs.msg import Twist
        from nav_msgs.msg import Odometry
        from tf2_msgs.msg import TFMessage

        await add_carter_ros(self._assets_root_path)
        stage = omni.usd.get_context().get_stage()

        # add an odom prim to carter
        odom_prim = stage.DefinePrim("/Carter/chassis_link/odom", "Xform")

        graph_path = "/Carter/ActionGraph"

        # add an tf publisher for world->odom
        try:
            og.Controller.edit(
                graph_path,
                {
                    og.Controller.Keys.CREATE_NODES: [("PublishTF", "isaacsim.ros2.bridge.ROS2PublishTransformTree")],
                    og.Controller.Keys.SET_VALUES: [
                        ("PublishTF.inputs:topicName", "tf_test"),
                        ("PublishTF.inputs:targetPrims", [usdrt.Sdf.Path("/Carter/chassis_link/odom")]),
                    ],
                    og.Controller.Keys.CONNECT: [
                        (graph_path + "/on_playback_tick.outputs:tick", "PublishTF.inputs:execIn"),
                        (
                            graph_path + "/isaac_read_simulation_time.outputs:simulationTime",
                            "PublishTF.inputs:timeStamp",
                        ),
                    ],
                },
            )
        except Exception as e:
            print(e)

        # move carter off origin
        carter_prim = stage.GetPrimAtPath("/Carter")
        new_translate = Gf.Vec3d(1.00, -3.00, 0.25)
        new_rotate = Gf.Rotation(Gf.Vec3d(0, 0, 1), 45)
        set_translate(carter_prim, new_translate)
        set_rotate(carter_prim, new_rotate)
        await omni.kit.app.get_app().next_update_async()

        tf_sub = self.create_subscription(self.node, TFMessage, "tf_test", self.tf_callback, get_qos_profile())
        odom_sub = self.create_subscription(self.node, Odometry, "odom", self.odom_callback, get_qos_profile())
        cmd_vel_pub = self.create_publisher(self.node, Twist, "cmd_vel", 1)

        await omni.kit.app.get_app().next_update_async()
        self._timeline.play()
        await omni.kit.app.get_app().next_update_async()

        # wait for physics to settle (60 frames = original simulate_async(1,60) duration),
        # then wait for subscriber data — odom.z reflects settling at this timing
        await self.simulate_until_condition(lambda: False, max_frames=60, per_frame_callback=self.spin)
        await self.simulate_until_condition(
            lambda: self._trans is not None and self._odom_data is not None,
            max_frames=120,
            per_frame_callback=self.spin,
        )

        # check 0: is carter initial tf position and odometry position
        # [tx, ty, tz, rx, ry, rz, rw]
        expected_trans = [1.0, -3.0, -0.01, 0, 0, 0.38268, 0.9238]
        # [px, py, pz, ox, oy, oz, ow]
        expected_odom = [0, 0, -0.23, 0, 0, 0, 1]
        self.check_pose(expected_trans, expected_odom, tolerance=1)

        # straight forward for 3s at 0.1 m/s → ~0.3m accumulated
        move_cmd = self.move_cmd_msg(0.1, 0.0, 0.0, 0.0, 0.0, 0.0)
        cmd_vel_pub.publish(move_cmd)
        await self.simulate_until_condition(lambda: False, max_frames=180, per_frame_callback=self.spin)

        # stop
        move_cmd = self.move_cmd_msg(0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
        cmd_vel_pub.publish(move_cmd)
        await self.simulate_until_condition(
            lambda: self._trans is not None and self._odom_data is not None, max_frames=60, per_frame_callback=self.spin
        )

        # check 1: location using default param
        print(self._trans.transform, self._odom_data)
        # [tx, ty, tz, rx, ry, rz, rw]
        expected_trans = [1.22, -2.78, -0.01, 0, 0, 0.38268, 0.9238]
        # [px, py, pz, ox, oy, oz, ow]
        expected_odom = [0.3, 0, -0.23, 0, 0, 0, 1]
        self.check_pose(expected_trans, expected_odom, tolerance=1)

        self._timeline.stop()
        await omni.kit.app.get_app().next_update_async()
        self.spin()
        self._trans = None
        self._odom_data = None

        # change wheel rotation and wheel base
        og.Controller.set(og.Controller.attribute(graph_path + "/differential_controller.inputs:wheelRadius"), 0.1)
        og.Controller.set(og.Controller.attribute(graph_path + "/differential_controller.inputs:wheelDistance"), 0.5)

        self._timeline.play()

        # fixed 60 frames to let physics settle before reading subscriber data
        await self.simulate_until_condition(lambda: False, max_frames=60, per_frame_callback=self.spin)
        await self.simulate_until_condition(
            lambda: self._trans is not None and self._odom_data is not None,
            max_frames=120,
            per_frame_callback=self.spin,
        )

        # check 3: is carter initial tf position and odometry position
        # [tx, ty, tz, rx, ry, rz, rw]
        expected_trans = [1.0, -3.0, 0, 0, 0, 0.38268, 0.9238]
        # [px, py, pz, ox, oy, oz, ow]
        expected_odom = [0, 0, -0.23, 0, 0, 0, 1]
        self.check_pose(expected_trans, expected_odom, tolerance=1)

        # straight forward for 3s at 0.1 m/s (odometry units, new wheel params) → ~0.7m accumulated
        move_cmd = self.move_cmd_msg(0.1, 0.0, 0.0, 0.0, 0.0, 0.0)
        cmd_vel_pub.publish(move_cmd)
        await self.simulate_until_condition(lambda: False, max_frames=180, per_frame_callback=self.spin)

        # stop
        move_cmd = self.move_cmd_msg(0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
        cmd_vel_pub.publish(move_cmd)

        # check 4: location after change radius
        # wait for all data to be received by subscribers
        await self.simulate_until_condition(
            lambda: self._trans is not None and self._odom_data is not None,
            max_frames=120,
            per_frame_callback=self.spin,
        )

        # [tx, ty, tz, rx, ry, rz, rw]
        expected_trans = [1.51, -2.49, 0, 0, 0, 0.3846, 0.9230]
        # [px, py, pz, ox, oy, oz, ow]
        expected_odom = [0.7, 0, -0.23, 0, 0, 0, 1]
        self.check_pose(expected_trans, expected_odom, tolerance=1)

        self._timeline.stop()
        self.spin()
        pass

    # add carter and ROS topic from scratch
    async def test_differential_base_scratch(self):
        """Test differential base scratch."""
        if SimulationManager.get_active_physics_engine() == "newton":
            self.skipTest("Odometry node not yet supported by Newton backend")
        from geometry_msgs.msg import Twist
        from nav_msgs.msg import Odometry

        await add_carter(self._assets_root_path)

        odom_sub = self.create_subscription(self.node, Odometry, "odom", self.odom_callback, 10)
        cmd_vel_pub = self.create_publisher(self.node, Twist, "cmd_vel", 1)

        graph_path = "/ActionGraph"
        graph_id, created_nodes = self.add_differential_drive(graph_path, "/Carter")
        og.Controller.attribute(graph_path + "/diffController.inputs:wheelRadius").set(0.1)

        self._timeline.play()
        await omni.kit.app.get_app().next_update_async()
        # fixed 60 frames: match original simulate_async(1, 60) settle before checking data
        await self.simulate_until_condition(lambda: False, max_frames=60, per_frame_callback=self.spin)

        # check 0: is carter initially stationary
        # No transform expected in this test, only check odometry
        # [px, py, pz, ox, oy, oz, ow]
        expected_odom = [0, 0, 0, 0, 0, 0, 1]
        self.check_pose(None, expected_odom, tolerance=1)

        # rotate for 2s at angular_z=0.2 rad/s → orientation.z ~0.40
        move_cmd = self.move_cmd_msg(0.0, 0.0, 0.0, 0.0, 0.0, 0.2)
        cmd_vel_pub.publish(move_cmd)
        self._timeline.play()
        await omni.kit.app.get_app().next_update_async()
        await self.simulate_until_condition(lambda: False, max_frames=120, per_frame_callback=self.spin)

        # stop
        move_cmd = self.move_cmd_msg(0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
        cmd_vel_pub.publish(move_cmd)
        self._timeline.play()
        await omni.kit.app.get_app().next_update_async()
        await self.simulate_until_condition(lambda: False, max_frames=60, per_frame_callback=self.spin)

        # check 1: location using default param
        odom_data = deepcopy(self._odom_data)
        self.assertAlmostEqual(odom_data.position.x, 0, 1)
        self.assertAlmostEqual(odom_data.position.y, 0, 1)
        self.assertAlmostEqual(odom_data.orientation.x, 0, 1)
        self.assertAlmostEqual(odom_data.orientation.y, 0, 1)
        self.assertAlmostEqual(odom_data.orientation.z, 0.40, 1)
        self.assertAlmostEqual(odom_data.orientation.w, 0.916, 1)

        # change wheel rotation and wheel base
        omni.kit.commands.execute("DeletePrims", paths=[graph_path])

        graph_id, created_nodes = self.add_differential_drive(graph_path, "/Carter")
        og.Controller.attribute(graph_path + "/diffController.inputs:wheelRadius").set(0.1)
        og.Controller.attribute(graph_path + "/diffController.inputs:wheelDistance").set(0.8)

        self._timeline.play()
        await omni.kit.app.get_app().next_update_async()
        # fixed 60 frames: match original simulate_async(1, 60) settle before issuing rotation command
        await self.simulate_until_condition(lambda: False, max_frames=60, per_frame_callback=self.spin)

        # rotate back for 2s at angular_z=-0.2 rad/s (wider wheelbase → faster rotation) → orientation.z ~-0.61
        move_cmd = self.move_cmd_msg(0.0, 0.0, 0.0, 0.0, 0.0, -0.2)
        cmd_vel_pub.publish(move_cmd)
        self._timeline.play()
        await omni.kit.app.get_app().next_update_async()
        await self.simulate_until_condition(lambda: False, max_frames=120, per_frame_callback=self.spin)

        # stop
        move_cmd = self.move_cmd_msg(0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
        cmd_vel_pub.publish(move_cmd)
        self._timeline.play()
        await omni.kit.app.get_app().next_update_async()
        await self.simulate_until_condition(lambda: False, max_frames=60, per_frame_callback=self.spin)

        # check 3: location after change radius
        odom_data = deepcopy(self._odom_data)
        self.assertAlmostEqual(odom_data.position.x, 0, 1)
        self.assertAlmostEqual(odom_data.position.y, 0, 1)
        self.assertAlmostEqual(odom_data.orientation.x, 0, 1)
        self.assertAlmostEqual(odom_data.orientation.y, 0, 1)
        self.assertAlmostEqual(odom_data.orientation.z, -0.61, 1)
        self.assertAlmostEqual(odom_data.orientation.w, 0.79, 1)

        self._timeline.stop()
        self.spin()
        pass

    async def test_nova_carter_differential_base(self):
        """Test nova carter differential base."""
        from geometry_msgs.msg import Twist
        from nav_msgs.msg import Odometry
        from tf2_msgs.msg import TFMessage

        await add_nova_carter_ros(self._assets_root_path)
        stage = omni.usd.get_context().get_stage()

        graph_path = "/nova_carter_ros2_sensors/transform_tree_odometry"
        drive_graph_path = "/nova_carter_ros2_sensors/differential_drive"
        # add an tf publisher for world->base_link
        try:
            og.Controller.edit(
                graph_path,
                {
                    og.Controller.Keys.CREATE_NODES: [("PublishTF", "isaacsim.ros2.bridge.ROS2PublishTransformTree")],
                    og.Controller.Keys.SET_VALUES: [
                        ("PublishTF.inputs:topicName", "tf_test"),
                        (
                            "PublishTF.inputs:targetPrims",
                            [usdrt.Sdf.Path("/nova_carter_ros2_sensors/chassis_link/base_link")],
                        ),
                    ],
                    og.Controller.Keys.CONNECT: [
                        (graph_path + "/on_playback_tick.outputs:tick", "PublishTF.inputs:execIn"),
                        (
                            graph_path + "/isaac_read_simulation_time.outputs:simulationTime",
                            "PublishTF.inputs:timeStamp",
                        ),
                    ],
                },
            )
        except Exception as e:
            print(e)
        await omni.kit.app.get_app().next_update_async()
        # move carter off origin
        carter_prim = stage.GetPrimAtPath("/nova_carter_ros2_sensors")
        new_translate = Gf.Vec3d(1.00, -3.00, 0.0)
        new_rotate = Gf.Rotation(Gf.Vec3d(0, 0, 1), 45)
        set_translate(carter_prim, new_translate)
        set_rotate(carter_prim, new_rotate)
        await omni.kit.app.get_app().next_update_async()

        tf_sub = self.create_subscription(self.node, TFMessage, "tf_test", self.tf_callback, get_qos_profile())
        odom_sub = self.create_subscription(self.node, Odometry, "chassis/odom", self.odom_callback, get_qos_profile())
        cmd_vel_pub = self.create_publisher(self.node, Twist, "cmd_vel", 1)

        await omni.kit.app.get_app().next_update_async()
        self._timeline.play()
        await omni.kit.app.get_app().next_update_async()

        # wait for physics to settle and for all data to be received by subscribers
        await self.simulate_until_condition(
            lambda: self._trans is not None and self._odom_data is not None,
            max_frames=240,
            per_frame_callback=self.spin,
        )

        # check 0: is carter initial tf position and odometry position
        # [tx, ty, tz, rx, ry, rz, rw]
        expected_trans = [1.0, -3.0, 0, 0, 0, 0.38268, 0.9238]
        # [px, py, pz, ox, oy, oz, ow]
        expected_odom = [0, 0, 0, 0, 0, 0, 1]
        self.check_pose(expected_trans, expected_odom, delta=0.1)

        # straight forward for 3s at 0.1 m/s → ~0.3m accumulated
        move_cmd = self.move_cmd_msg(0.1, 0.0, 0.0, 0.0, 0.0, 0.0)
        cmd_vel_pub.publish(move_cmd)
        self._timeline.play()
        await omni.kit.app.get_app().next_update_async()
        await self.simulate_until_condition(lambda: False, max_frames=180, per_frame_callback=self.spin)

        # stop
        move_cmd = self.move_cmd_msg(0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
        cmd_vel_pub.publish(move_cmd)

        self._timeline.play()
        await omni.kit.app.get_app().next_update_async()
        await self.simulate_until_condition(
            lambda: self._trans is not None and self._odom_data is not None, max_frames=60, per_frame_callback=self.spin
        )

        # check 1: location using default param
        carb.log_info(str(self._trans.transform))
        carb.log_info(str(self._odom_data))
        # [tx, ty, tz, rx, ry, rz, rw]
        expected_trans = [1.22, -2.78, 0, 0, 0, 0.38268, 0.9238]
        # [px, py, pz, ox, oy, oz, ow]
        expected_odom = [0.3, 0, 0, 0, 0, 0, 1]
        self.check_pose(expected_trans, expected_odom, delta=0.1)

        self._timeline.stop()
        await omni.kit.app.get_app().next_update_async()
        self.spin()
        self._trans = None
        self._odom_data = None

        # change wheel rotation and wheel base
        og.Controller.set(
            og.Controller.attribute(drive_graph_path + "/differential_controller_01.inputs:wheelRadius"), 0.1
        )
        og.Controller.set(
            og.Controller.attribute(drive_graph_path + "/differential_controller_01.inputs:wheelDistance"), 0.5
        )

        self._timeline.play()
        await self.simulate_until_condition(
            lambda: self._trans is not None and self._odom_data is not None,
            max_frames=240,
            per_frame_callback=self.spin,
        )

        # check 3: is carter initial tf position and odometry position
        # [tx, ty, tz, rx, ry, rz, rw]
        expected_trans = [1.0, -3.0, 0, 0, 0, 0.38268, 0.9238]
        # [px, py, pz, ox, oy, oz, ow]
        expected_odom = [0, 0, 0, 0, 0, 0, 1]
        self.check_pose(expected_trans, expected_odom, delta=0.1)

        # straight forward for 3s at 0.1 m/s (odometry units, new wheel params) → ~0.43m accumulated
        move_cmd = self.move_cmd_msg(0.1, 0.0, 0.0, 0.0, 0.0, 0.0)
        cmd_vel_pub.publish(move_cmd)
        self._timeline.play()
        await omni.kit.app.get_app().next_update_async()
        await self.simulate_until_condition(lambda: False, max_frames=180, per_frame_callback=self.spin)

        # stop
        move_cmd = self.move_cmd_msg(0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
        cmd_vel_pub.publish(move_cmd)
        self._timeline.play()
        await omni.kit.app.get_app().next_update_async()
        await self.simulate_until_condition(
            lambda: self._trans is not None and self._odom_data is not None, max_frames=60, per_frame_callback=self.spin
        )

        # check 4: location after change radius
        carb.log_info(str(self._trans.transform))
        carb.log_info(str(self._odom_data))
        # [tx, ty, tz, rx, ry, rz, rw]
        expected_trans = [1.30, -2.69, 0, 0, 0, 0.3815, 0.9244]
        # [px, py, pz, ox, oy, oz, ow]
        expected_odom = [0.43, 0, 0, 0, 0, 0, 1]
        self.check_pose(expected_trans, expected_odom, delta=0.1)

        self._timeline.stop()
        self.spin()
        pass

    def check_pose(self, expected_trans, expected_odom, tolerance=1, delta=None):
        """Verify robot pose against expected transform and odometry values.

        Args:
            expected_trans: Expected [tx, ty, tz, rx, ry, rz, rw] or None to skip.
            expected_odom: Expected [px, py, pz, ox, oy, oz, ow] or None to skip.
            tolerance: Decimal places for assertAlmostEqual (used when delta is None).
            delta: Absolute tolerance for assertAlmostEqual (overrides tolerance).
        """

        def _assert_close(actual, expected, msg=""):
            if delta is not None:
                self.assertAlmostEqual(actual, expected, delta=delta, msg=msg)
            else:
                self.assertAlmostEqual(actual, expected, tolerance, msg=msg)

        if expected_trans is not None:
            self.assertIsNotNone(self._trans)
            if len(expected_trans) >= 3:
                _assert_close(self._trans.transform.translation.x, expected_trans[0])
                _assert_close(self._trans.transform.translation.y, expected_trans[1])
                _assert_close(self._trans.transform.translation.z, expected_trans[2])
            if len(expected_trans) >= 7:
                _assert_close(self._trans.transform.rotation.x, expected_trans[3])
                _assert_close(self._trans.transform.rotation.y, expected_trans[4])
                _assert_close(self._trans.transform.rotation.z, expected_trans[5])
                _assert_close(self._trans.transform.rotation.w, expected_trans[6])

        if expected_odom is not None:
            self.assertIsNotNone(self._odom_data)
            odom_data = deepcopy(self._odom_data)
            if len(expected_odom) >= 3:
                _assert_close(odom_data.position.x, expected_odom[0])
                _assert_close(odom_data.position.y, expected_odom[1])
                _assert_close(odom_data.position.z, expected_odom[2])
            if len(expected_odom) >= 7:
                _assert_close(odom_data.orientation.x, expected_odom[3])
                _assert_close(odom_data.orientation.y, expected_odom[4])
                _assert_close(odom_data.orientation.z, expected_odom[5])
                _assert_close(odom_data.orientation.w, expected_odom[6])

    def add_differential_drive(self, graph_path, robot_path):
        """Add differential drive to the test scene."""
        try:
            keys = og.Controller.Keys
            graph, nodes, _, _ = og.Controller.edit(
                {"graph_path": graph_path, "evaluator_name": "execution"},
                {
                    keys.CREATE_NODES: [
                        ("OnPlaybackTick", "omni.graph.action.OnPlaybackTick"),
                        ("ReadSimTime", "isaacsim.core.nodes.IsaacReadSimulationTime"),
                        # Added nodes used for Odometry publisher
                        ("computeOdom", "isaacsim.core.nodes.IsaacComputeOdometry"),
                        ("publishOdom", "isaacsim.ros2.bridge.ROS2PublishOdometry"),
                        ("publishRawTF", "isaacsim.ros2.bridge.ROS2PublishRawTransformTree"),
                        # Added nodes used for Twist subscriber, differential drive
                        ("subscribeTwist", "isaacsim.ros2.bridge.ROS2SubscribeTwist"),
                        ("breakLinVel", "omni.graph.nodes.BreakVector3"),
                        ("breakAngVel", "omni.graph.nodes.BreakVector3"),
                        ("diffController", "isaacsim.robot.wheeled_robots.DifferentialController"),
                        ("artController", "isaacsim.core.nodes.IsaacArticulationController"),
                    ],
                    keys.CONNECT: [
                        ("OnPlaybackTick.outputs:tick", "computeOdom.inputs:execIn"),
                        ("OnPlaybackTick.outputs:tick", "publishOdom.inputs:execIn"),
                        ("OnPlaybackTick.outputs:tick", "publishRawTF.inputs:execIn"),
                        ("ReadSimTime.outputs:simulationTime", "publishOdom.inputs:timeStamp"),
                        ("ReadSimTime.outputs:simulationTime", "publishRawTF.inputs:timeStamp"),
                        ("computeOdom.outputs:angularVelocity", "publishOdom.inputs:angularVelocity"),
                        ("computeOdom.outputs:linearVelocity", "publishOdom.inputs:linearVelocity"),
                        ("computeOdom.outputs:orientation", "publishOdom.inputs:orientation"),
                        ("computeOdom.outputs:position", "publishOdom.inputs:position"),
                        ("computeOdom.outputs:orientation", "publishRawTF.inputs:rotation"),
                        ("computeOdom.outputs:position", "publishRawTF.inputs:translation"),
                        ("OnPlaybackTick.outputs:tick", "subscribeTwist.inputs:execIn"),
                        ("OnPlaybackTick.outputs:tick", "artController.inputs:execIn"),
                        ("subscribeTwist.outputs:execOut", "diffController.inputs:execIn"),
                        ("subscribeTwist.outputs:linearVelocity", "breakLinVel.inputs:tuple"),
                        ("breakLinVel.outputs:x", "diffController.inputs:linearVelocity"),
                        ("subscribeTwist.outputs:angularVelocity", "breakAngVel.inputs:tuple"),
                        ("breakAngVel.outputs:z", "diffController.inputs:angularVelocity"),
                        ("diffController.outputs:velocityCommand", "artController.inputs:velocityCommand"),
                    ],
                    og.Controller.Keys.SET_VALUES: [
                        ("diffController.inputs:wheelRadius", 0.24),
                        ("diffController.inputs:wheelDistance", 0.5),
                        ("artController.inputs:jointNames", ["left_wheel", "right_wheel"]),
                        ("computeOdom.inputs:chassisPrim", [usdrt.Sdf.Path(robot_path)]),
                        ("artController.inputs:targetPrim", [usdrt.Sdf.Path(robot_path)]),
                    ],
                },
            )
        except Exception as e:
            print(e)

        return graph, nodes
