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

"""Tests for ROS 2 subscriber OmniGraph nodes."""

import random
import time

import numpy as np
import omni.graph.core as og
import omni.kit.commands
import omni.kit.test
import omni.kit.usd
from isaacsim.core.experimental.objects import Cube
from isaacsim.core.experimental.prims import RigidPrim
from isaacsim.core.experimental.utils import stage as stage_utils
from isaacsim.core.experimental.utils import xform as xform_utils
from isaacsim.ros2.core.impl.ros2_test_case import ROS2TestCase
from pxr import Gf, Sdf, UsdGeom, UsdPhysics


class TestRos2Subscribers(ROS2TestCase):
    """Test suite for ros2 subscribers."""

    MAX_COUNT = 100
    # Queue-size ranges sampled randomly per test.  Kept well below MAX_COUNT
    # so there is always a clean prefix of dropped messages to validate the
    # ring-buffer semantics of the OG subscribe node.
    SMALL_QUEUE_RANGE = (1, 10)
    LARGE_QUEUE_RANGE = (15, 25)

    async def setUp(self):
        """Set up test fixtures."""
        await super().setUp()

        await omni.usd.get_context().new_stage_async()

        self.sub_data = []
        self.prev_seq = 0
        self.queue_size = 10
        self.sub_node_time_attribute_path = None

        await omni.kit.app.get_app().next_update_async()

    async def tearDown(self):
        """Tear down test fixtures."""
        await super().tearDown()

    def spin(self):
        """Sample the OG subscribe node's sequence attribute into ``sub_data``.

        Used as the per-frame callback of :meth:`simulate_until_condition` so
        each incoming message bumps a counter that the assertion can compare
        against the expected tail of the published sequence.
        """
        if self.sub_node_time_attribute_path is None:
            return
        seq = og.Controller.get(og.Controller.attribute(self.sub_node_time_attribute_path))
        if isinstance(seq, np.ndarray):
            seq = seq[0] if seq is not None else self.prev_seq

        if self.prev_seq != seq:
            self.sub_data.append(int(seq))
            self.prev_seq = seq

    def _choose_queue_size(self, queue_range):
        """Pick a random queue size inside the given inclusive range."""
        lo, hi = queue_range
        queue_size = random.randint(lo, hi)
        print("Choosing queue size of", queue_size)
        return queue_size

    async def _run_queue_test(
        self,
        *,
        node_name: str,
        ros_topic: str,
        msg_type,
        subscribe_node_name: str,
        subscribe_node_type: str,
        time_output_attr: str,
        publish_fn,
        queue_size: int,
    ):
        """Shared body for every ``*_subscriber_queue`` test.

        Exercises exactly one timeline cycle: build the graph with the
        requested ``queue_size``, play, wait for DDS discovery, publish
        ``MAX_COUNT - 1`` messages carrying a monotonically increasing
        sequence number, then assert that the OG subscribe node produced
        the last ``queue_size`` sequence numbers in order.

        Keeping each test method to a single timeline cycle avoids the DDS
        discovery stale-match issues that show up when the OG subscribe
        endpoint is torn down and recreated inside the same method.
        Multi-size coverage is achieved by having separate ``_small`` and
        ``_large`` test methods rather than two runs in one method.

        Args:
            node_name: rclpy node name used for the test-side publisher.
            ros_topic: ROS 2 topic shared by the OG subscriber and the
                rclpy publisher.
            msg_type: rclpy message class used for the publisher.
            subscribe_node_name: Short OG node name (e.g.
                ``"SubscribeJointState"``).
            subscribe_node_type: Fully-qualified OG node type (e.g.
                ``"isaacsim.ros2.bridge.ROS2SubscribeJointState"``).
            time_output_attr: Name of the OG output attribute whose value
                carries the per-message sequence number.
            publish_fn: Callable ``(publisher, seq_count) -> None`` that
                publishes a single message carrying ``seq_count`` in the
                corresponding field.
            queue_size: Value to set on ``inputs:queueSize`` before play.
        """
        node = self.create_node(node_name)
        test_pub = self.create_publisher(node, msg_type, ros_topic, self.MAX_COUNT)

        graph_path = "/ActionGraph"
        self.sub_node_time_attribute_path = f"{graph_path}/{subscribe_node_name}.outputs:{time_output_attr}"
        sub_node_queue_attribute_path = f"{graph_path}/{subscribe_node_name}.inputs:queueSize"

        try:
            og.Controller.edit(
                {"graph_path": graph_path, "evaluator_name": "execution"},
                {
                    og.Controller.Keys.CREATE_NODES: [
                        ("OnPlaybackTick", "omni.graph.action.OnPlaybackTick"),
                        (subscribe_node_name, subscribe_node_type),
                    ],
                    og.Controller.Keys.SET_VALUES: [
                        (f"{subscribe_node_name}.inputs:topicName", ros_topic),
                    ],
                    og.Controller.Keys.CONNECT: [
                        ("OnPlaybackTick.outputs:tick", f"{subscribe_node_name}.inputs:execIn"),
                    ],
                },
            )
        except Exception as e:
            print(e)

        og.Controller.set(og.Controller.attribute(sub_node_queue_attribute_path), queue_size)
        self.queue_size = queue_size
        self.sub_data = []
        self.prev_seq = 0
        expected_data = [self.MAX_COUNT - queue_size + i for i in range(queue_size)]

        self._timeline.play()
        await self.wait_for_subscribers_on_topic(test_pub)

        for count in range(1, self.MAX_COUNT):
            publish_fn(test_pub, count)
            time.sleep(0.005)

        condition_met = await self.simulate_until_condition(
            lambda: len(self.sub_data) >= queue_size,
            per_frame_callback=self.spin,
        )

        self._timeline.stop()

        self.assertTrue(
            condition_met,
            msg=(
                f"{subscribe_node_name} queue test timeout.\n"
                f"  Expected: {expected_data}\n"
                f"  Actual:   {self.sub_data}\n"
                f"  Queue Size: {queue_size}\n"
                f"  MAX_COUNT: {self.MAX_COUNT}"
            ),
        )

        self.assertEqual(
            self.sub_data,
            expected_data,
            msg=(
                f"{subscribe_node_name} queue test mismatch.\n"
                f"  Expected: {expected_data}\n"
                f"  Actual:   {self.sub_data}\n"
                f"  Queue Size: {queue_size}\n"
                f"  MAX_COUNT: {self.MAX_COUNT}"
            ),
        )

    # ------------------------------------------------------------------
    # JointState
    # ------------------------------------------------------------------

    @staticmethod
    def _publish_joint_state(publisher, count: int) -> None:
        from builtin_interfaces.msg import Time
        from sensor_msgs.msg import JointState

        msg = JointState()
        msg.header.stamp = Time(sec=count)
        msg.name = ["test1", "test2"]
        msg.position = [0.0, 0.0]
        publisher.publish(msg)

    async def test_joint_state_subscriber_queue_small(self):
        """JointState subscriber buffers the last N messages (small N)."""
        from sensor_msgs.msg import JointState

        await self._run_queue_test(
            node_name="isaac_sim_test_joint_state_sub_queue_small",
            ros_topic="joint_sub_small",
            msg_type=JointState,
            subscribe_node_name="SubscribeJointState",
            subscribe_node_type="isaacsim.ros2.bridge.ROS2SubscribeJointState",
            time_output_attr="timeStamp",
            publish_fn=self._publish_joint_state,
            queue_size=self._choose_queue_size(self.SMALL_QUEUE_RANGE),
        )

    async def test_joint_state_subscriber_queue_large(self):
        """JointState subscriber buffers the last N messages (large N)."""
        from sensor_msgs.msg import JointState

        await self._run_queue_test(
            node_name="isaac_sim_test_joint_state_sub_queue_large",
            ros_topic="joint_sub_large",
            msg_type=JointState,
            subscribe_node_name="SubscribeJointState",
            subscribe_node_type="isaacsim.ros2.bridge.ROS2SubscribeJointState",
            time_output_attr="timeStamp",
            publish_fn=self._publish_joint_state,
            queue_size=self._choose_queue_size(self.LARGE_QUEUE_RANGE),
        )

    # ------------------------------------------------------------------
    # Clock
    # ------------------------------------------------------------------

    @staticmethod
    def _publish_clock(publisher, count: int) -> None:
        from builtin_interfaces.msg import Time
        from rosgraph_msgs.msg import Clock

        msg = Clock()
        msg.clock = Time(sec=count)
        publisher.publish(msg)

    async def test_clock_subscriber_queue_small(self):
        """Clock subscriber buffers the last N messages (small N)."""
        from rosgraph_msgs.msg import Clock

        await self._run_queue_test(
            node_name="isaac_sim_test_clock_sub_queue_small",
            ros_topic="clock_sub_small",
            msg_type=Clock,
            subscribe_node_name="SubscribeClock",
            subscribe_node_type="isaacsim.ros2.bridge.ROS2SubscribeClock",
            time_output_attr="timeStamp",
            publish_fn=self._publish_clock,
            queue_size=self._choose_queue_size(self.SMALL_QUEUE_RANGE),
        )

    async def test_clock_subscriber_queue_large(self):
        """Clock subscriber buffers the last N messages (large N)."""
        from rosgraph_msgs.msg import Clock

        await self._run_queue_test(
            node_name="isaac_sim_test_clock_sub_queue_large",
            ros_topic="clock_sub_large",
            msg_type=Clock,
            subscribe_node_name="SubscribeClock",
            subscribe_node_type="isaacsim.ros2.bridge.ROS2SubscribeClock",
            time_output_attr="timeStamp",
            publish_fn=self._publish_clock,
            queue_size=self._choose_queue_size(self.LARGE_QUEUE_RANGE),
        )

    # ------------------------------------------------------------------
    # Twist
    # ------------------------------------------------------------------

    @staticmethod
    def _publish_twist(publisher, count: int) -> None:
        from geometry_msgs.msg import Twist

        msg = Twist()
        msg.linear.x = float(count)
        publisher.publish(msg)

    async def test_twist_subscriber_queue_small(self):
        """Twist subscriber buffers the last N messages (small N)."""
        from geometry_msgs.msg import Twist

        await self._run_queue_test(
            node_name="isaac_sim_test_twist_sub_queue_small",
            ros_topic="twist_sub_small",
            msg_type=Twist,
            subscribe_node_name="SubscribeTwist",
            subscribe_node_type="isaacsim.ros2.bridge.ROS2SubscribeTwist",
            time_output_attr="linearVelocity",
            publish_fn=self._publish_twist,
            queue_size=self._choose_queue_size(self.SMALL_QUEUE_RANGE),
        )

    async def test_twist_subscriber_queue_large(self):
        """Twist subscriber buffers the last N messages (large N)."""
        from geometry_msgs.msg import Twist

        await self._run_queue_test(
            node_name="isaac_sim_test_twist_sub_queue_large",
            ros_topic="twist_sub_large",
            msg_type=Twist,
            subscribe_node_name="SubscribeTwist",
            subscribe_node_type="isaacsim.ros2.bridge.ROS2SubscribeTwist",
            time_output_attr="linearVelocity",
            publish_fn=self._publish_twist,
            queue_size=self._choose_queue_size(self.LARGE_QUEUE_RANGE),
        )

    # ------------------------------------------------------------------
    # AckermannDriveStamped
    # ------------------------------------------------------------------

    @staticmethod
    def _publish_ackermann(publisher, count: int) -> None:
        from ackermann_msgs.msg import AckermannDriveStamped
        from builtin_interfaces.msg import Time

        msg = AckermannDriveStamped()
        msg.header.stamp = Time(sec=count)
        publisher.publish(msg)

    async def test_ackermann_subscriber_queue_small(self):
        """Ackermann subscriber buffers the last N messages (small N)."""
        from ackermann_msgs.msg import AckermannDriveStamped

        await self._run_queue_test(
            node_name="isaac_sim_test_ackermann_sub_queue_small",
            ros_topic="ackermann_sub_small",
            msg_type=AckermannDriveStamped,
            subscribe_node_name="SubscribeAckermannDrive",
            subscribe_node_type="isaacsim.ros2.bridge.ROS2SubscribeAckermannDrive",
            time_output_attr="timeStamp",
            publish_fn=self._publish_ackermann,
            queue_size=self._choose_queue_size(self.SMALL_QUEUE_RANGE),
        )

    async def test_ackermann_subscriber_queue_large(self):
        """Ackermann subscriber buffers the last N messages (large N)."""
        from ackermann_msgs.msg import AckermannDriveStamped

        await self._run_queue_test(
            node_name="isaac_sim_test_ackermann_sub_queue_large",
            ros_topic="ackermann_sub_large",
            msg_type=AckermannDriveStamped,
            subscribe_node_name="SubscribeAckermannDrive",
            subscribe_node_type="isaacsim.ros2.bridge.ROS2SubscribeAckermannDrive",
            time_output_attr="timeStamp",
            publish_fn=self._publish_ackermann,
            queue_size=self._choose_queue_size(self.LARGE_QUEUE_RANGE),
        )

    # ------------------------------------------------------------------
    # Transform tree
    # ------------------------------------------------------------------

    async def test_transform_tree_subscriber(self):
        """Test transform tree subscriber."""
        from geometry_msgs.msg import TransformStamped
        from tf2_msgs.msg import TFMessage

        self._stage = omni.usd.get_context().get_stage()

        # Create a node to subscribe to TFs
        node = self.create_node("isaac_sim_test_transform_tree_sub_queue")
        ros_topic = "tf_sub"
        test_pub = self.create_publisher(node, TFMessage, ros_topic, self.MAX_COUNT)

        self.graph_path = "/ActionGraph"

        try:
            og.Controller.edit(
                {"graph_path": self.graph_path, "evaluator_name": "execution"},
                {
                    og.Controller.Keys.CREATE_NODES: [
                        ("OnPlaybackTick", "omni.graph.action.OnPlaybackTick"),
                        ("SubscribeTransformTree", "isaacsim.ros2.bridge.ROS2SubscribeTransformTree"),
                    ],
                    og.Controller.Keys.SET_VALUES: [
                        ("SubscribeTransformTree.inputs:topicName", ros_topic),
                        ("SubscribeTransformTree.inputs:frameNamesMap", ["/World", "world", "/World/cube", "cube"]),
                    ],
                    og.Controller.Keys.CONNECT: [
                        ("OnPlaybackTick.outputs:tick", "SubscribeTransformTree.inputs:execIn"),
                    ],
                },
            )
        except Exception as e:
            print(e)

        # Create our scene
        UsdGeom.Xform.Define(self._stage, "/World")

        scene = UsdPhysics.Scene.Define(self._stage, Sdf.Path("/World/physicsScene"))
        scene.CreateGravityDirectionAttr().Set(Gf.Vec3f(0.0, 0.0, -1.0))
        scene.CreateGravityMagnitudeAttr().Set(9.81)

        Cube("/World/cube", positions=np.array([[0, 0, 5]]))
        RigidPrim("/World/cube")

        self._timeline.play()
        # Wait for DDS discovery to match publisher and the OG subscribe
        # endpoint; otherwise the first publish below can race and get
        # dropped before the subscriber is ready.
        await self.wait_for_subscribers_on_topic(test_pub)

        for i in range(10):
            msg = TransformStamped()
            msg.header.stamp = node.get_clock().now().to_msg()
            msg.child_frame_id = "cube"
            msg.header.frame_id = "world"

            pos = np.array([float(i) / 5.0, float(i) * float(i) / 25.0, 1.0])

            msg.transform.translation.x = pos[0]
            msg.transform.translation.y = pos[1]
            msg.transform.translation.z = pos[2]

            # Rotation of i radians around (1,1,1)
            a = np.sin(float(i) / 2.0) / np.sqrt(3.0)

            rot = [np.cos(float(i) / 2.0), a, a, a]
            msg.transform.rotation.x = rot[1]
            msg.transform.rotation.y = rot[2]
            msg.transform.rotation.z = rot[3]
            msg.transform.rotation.w = rot[0]

            tf_msg = TFMessage()
            tf_msg.transforms.append(msg)

            test_pub.publish(tf_msg)

            time.sleep(0.005)

            # Wait until pose condition is met or timeout
            def pose_condition():
                _prim = self._stage.GetPrimAtPath("/World/cube")
                _pos_wp, _rot_wp = xform_utils.get_world_pose(_prim)
                x = _pos_wp.numpy().flatten()
                r = _rot_wp.numpy().flatten()
                pos_match = np.linalg.norm(x - pos) < 1e-5
                rot_match = np.linalg.norm(r - rot) < 1e-5 or np.linalg.norm(r + rot) < 1e-5
                return pos_match and rot_match

            condition_met = await self.simulate_until_condition(pose_condition)

            self.assertTrue(
                condition_met,
                msg=f"Transform tree test failed for iteration {i}: Pose condition not met within time limit",
            )

            _prim = self._stage.GetPrimAtPath("/World/cube")
            _pos_wp, _rot_wp = xform_utils.get_world_pose(_prim)
            x = _pos_wp.numpy().flatten()
            r = _rot_wp.numpy().flatten()

            # NOTE : a quaterion q and -q represent the same rotation
            self.assertTrue(np.linalg.norm(x - pos) < 1e-5)
            self.assertTrue(np.linalg.norm(r - rot) < 1e-5 or np.linalg.norm(r + rot) < 1e-5)

        self._timeline.stop()

    async def test_transform_tree_subscriber_nova_carter(self):
        """Test transform tree subscriber nova carter."""
        from geometry_msgs.msg import TransformStamped
        from tf2_msgs.msg import TFMessage

        # Load our Nova Carter ROS stage
        stage_path = "/Isaac/Robots/NVIDIA/NovaCarter/nova_carter.usd"
        await stage_utils.open_stage_async(self._assets_root_path + stage_path)

        self._stage = omni.usd.get_context().get_stage()

        # Create a node to subscribe to TFs
        node = self.create_node("isaac_sim_test_transform_tree_sub_nova_carter")
        ros_topic = "tf_sub"
        test_pub = self.create_publisher(node, TFMessage, ros_topic, self.MAX_COUNT)

        self.graph_path = "/ActionGraph"

        try:
            og.Controller.edit(
                {"graph_path": self.graph_path, "evaluator_name": "execution"},
                {
                    og.Controller.Keys.CREATE_NODES: [
                        ("OnPlaybackTick", "omni.graph.action.OnPlaybackTick"),
                        ("SubscribeTransformTree", "isaacsim.ros2.bridge.ROS2SubscribeTransformTree"),
                    ],
                    og.Controller.Keys.SET_VALUES: [
                        ("SubscribeTransformTree.inputs:topicName", ros_topic),
                        (
                            "SubscribeTransformTree.inputs:frameNamesMap",
                            ["/nova_carter/chassis_link", "base_link", "/nova_carter", "odom"],
                        ),
                        ("SubscribeTransformTree.inputs:articulationRoots", ["/nova_carter/chassis_link"]),
                    ],
                    og.Controller.Keys.CONNECT: [
                        ("OnPlaybackTick.outputs:tick", "SubscribeTransformTree.inputs:execIn"),
                    ],
                },
            )
        except Exception as e:
            print(e)

        self._timeline.play()
        # Wait for DDS discovery to match publisher and the OG subscribe
        # endpoint; otherwise the first publish below can race and get
        # dropped before the subscriber is ready.
        await self.wait_for_subscribers_on_topic(test_pub)

        for i in range(5):
            msg = TransformStamped()
            msg.header.stamp = node.get_clock().now().to_msg()
            msg.child_frame_id = "base_link"
            msg.header.frame_id = "odom"

            pos = np.array([float(i) / 5.0, float(i) * float(i) / 25.0, 1.0])

            msg.transform.translation.x = pos[0]
            msg.transform.translation.y = pos[1]
            msg.transform.translation.z = pos[2]

            # Rotation of i radians around (1,1,1)
            a = np.sin(float(i) / 2.0) / np.sqrt(3.0)

            rot = [np.cos(float(i) / 2.0), a, a, a]
            msg.transform.rotation.x = rot[1]
            msg.transform.rotation.y = rot[2]
            msg.transform.rotation.z = rot[3]
            msg.transform.rotation.w = rot[0]

            tf_msg = TFMessage()
            tf_msg.transforms.append(msg)

            test_pub.publish(tf_msg)

            time.sleep(0.005)

            # Wait until pose condition is met or timeout
            def pose_condition():
                _prim = self._stage.GetPrimAtPath("/nova_carter/chassis_link")
                _pos_wp, _rot_wp = xform_utils.get_world_pose(_prim)
                x = _pos_wp.numpy().flatten()
                r = _rot_wp.numpy().flatten()
                pos_match = np.linalg.norm(x - pos) < 1e-5
                rot_match = np.linalg.norm(r - rot) < 1e-5 or np.linalg.norm(r + rot) < 1e-5
                return pos_match and rot_match

            condition_met = await self.simulate_until_condition(pose_condition, max_frames=60)

            self.assertTrue(
                condition_met,
                msg=f"Nova Carter transform tree test failed for iteration {i}: Pose condition not met within time limit",
            )

            _prim = self._stage.GetPrimAtPath("/nova_carter/chassis_link")
            _pos_wp, _rot_wp = xform_utils.get_world_pose(_prim)
            x = _pos_wp.numpy().flatten()
            r = _rot_wp.numpy().flatten()

            # NOTE : a quaterion q and -q represent the same rotation
            self.assertTrue(np.linalg.norm(x - pos) < 1e-5)
            self.assertTrue(np.linalg.norm(r - rot) < 1e-5 or np.linalg.norm(r + rot) < 1e-5)

        self._timeline.stop()
