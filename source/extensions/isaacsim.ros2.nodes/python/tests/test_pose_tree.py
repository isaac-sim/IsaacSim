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


"""Verify ROS 2 pose tree publishing.

Covers duplicate-name handling, frame-name overrides, and
compute-transform-tree graph integration.
"""

import numpy as np
import omni.graph.core as og
import omni.kit.commands
import omni.kit.test
import omni.kit.usd
import usdrt.Sdf
from isaacsim.core.experimental.prims import XformPrim
from isaacsim.core.experimental.utils import stage as stage_utils
from isaacsim.core.nodes.scripts.utils import set_target_prims
from isaacsim.ros2.core.impl.ros2_test_case import ROS2TestCase
from pxr import Gf, Sdf
from usd.schema.isaac import robot_schema

from .common import add_cube, add_franka, get_qos_profile


class TestRos2PoseTree(ROS2TestCase):
    """Verify TFMessage output from ROS 2 pose tree publishing graphs."""

    async def setUp(self) -> None:
        """Create a fresh stage for pose tree publishing tests."""
        await super().setUp()
        await omni.usd.get_context().new_stage_async()
        await omni.kit.app.get_app().next_update_async()

    async def tearDown(self) -> None:
        """Run shared ROS 2 cleanup after pose tree tests."""
        await super().tearDown()

    def _enable_tf_aggregation(self) -> None:
        """Ensure these tests exercise the aggregated TF publisher path."""
        import carb.settings

        carb.settings.get_settings().set_bool("/exts/isaacsim.ros2.nodes/tfAggregation/enabled", True)

    def _find_tf_message_with_children(self, messages: list, child_frames: set[str]) -> object | None:
        """Return the newest TF message containing all requested child frames."""
        for message in reversed(messages):
            transforms_by_child = {transform.child_frame_id: transform for transform in message.transforms}
            if child_frames.issubset(transforms_by_child):
                return message
        return None

    def _assert_same_tf_timestamp(self, message: object) -> None:
        """Check an aggregated TF message stamps all transforms with one time."""
        stamps = {(transform.header.stamp.sec, transform.header.stamp.nanosec) for transform in message.transforms}
        self.assertEqual(len(stamps), 1)

    def _assert_transform(self, transform: object, parent_frame: str, translation: tuple[float, float, float]) -> None:
        """Check the stable fields we expect from the test graph inputs."""
        self.assertEqual(transform.header.frame_id, parent_frame)
        self.assertAlmostEqual(transform.transform.translation.x, translation[0])
        self.assertAlmostEqual(transform.transform.translation.y, translation[1])
        self.assertAlmostEqual(transform.transform.translation.z, translation[2])
        self.assertAlmostEqual(transform.transform.rotation.x, 0.0)
        self.assertAlmostEqual(transform.transform.rotation.y, 0.0)
        self.assertAlmostEqual(transform.transform.rotation.z, 0.0)
        self.assertAlmostEqual(transform.transform.rotation.w, 1.0)

    async def _wait_for_aggregated_tf_message(
        self, node: object, topic_name: str, messages: list, child_frames: set[str], max_frames: int = 120
    ) -> object:
        """Play the graph until one aggregated TF message reaches the ROS subscriber."""
        import rclpy

        def spin() -> None:
            rclpy.spin_once(node, timeout_sec=0.01)

        self._timeline.play()
        await omni.kit.app.get_app().next_update_async()
        await self.wait_for_publishers_on_topic(node, topic_name, timeout_sec=10.0, per_frame_callback=spin)
        self.assertEqual(node.count_publishers(topic_name), 1)
        await self.simulate_until_condition(
            lambda: self._find_tf_message_with_children(messages, child_frames) is not None,
            max_frames=max_frames,
            per_frame_callback=spin,
            failure_message=f"Expected aggregated TF message on {topic_name} containing {sorted(child_frames)}",
        )

        self._timeline.stop()
        await omni.kit.app.get_app().next_update_async()
        spin()
        message = self._find_tf_message_with_children(messages, child_frames)
        self.assertIsNotNone(message)
        return message

    async def test_pose_tree(self) -> None:
        """Test pose tree."""
        import rclpy
        from tf2_msgs.msg import TFMessage

        await add_franka(self._assets_root_path)
        await add_cube("/cube", 0.75, (2.00, 0, 0.75))

        self._tf_data = None
        self._tf_data_prev = None

        def tf_callback(data: TFMessage) -> None:
            self._tf_data = data

        node = self.create_node("tf_tester")
        tf_sub = self.create_subscription(node, TFMessage, "/tf_test", tf_callback, get_qos_profile())

        try:
            og.Controller.edit(
                {"graph_path": "/ActionGraph", "evaluator_name": "execution"},
                {
                    og.Controller.Keys.CREATE_NODES: [
                        ("OnPlaybackTick", "omni.graph.action.OnPlaybackTick"),
                        ("ReadSimTime", "isaacsim.core.nodes.IsaacReadSimulationTime"),
                        ("PublishTF", "isaacsim.ros2.bridge.ROS2PublishTransformTree"),
                    ],
                    og.Controller.Keys.SET_VALUES: [
                        ("PublishTF.inputs:topicName", "/tf_test"),
                        (
                            "PublishTF.inputs:targetPrims",
                            [
                                usdrt.Sdf.Path("/panda"),
                                usdrt.Sdf.Path("/cube"),
                                usdrt.Sdf.Path("/panda/panda_hand/geometry"),
                                usdrt.Sdf.Path("/panda/panda_hand"),
                            ],
                        ),
                    ],
                    og.Controller.Keys.CONNECT: [
                        ("OnPlaybackTick.outputs:tick", "PublishTF.inputs:execIn"),
                        ("ReadSimTime.outputs:simulationTime", "PublishTF.inputs:timeStamp"),
                    ],
                },
            )
        except Exception as e:
            print(e)

        def spin() -> None:
            rclpy.spin_once(node, timeout_sec=0.01)

        self._timeline.play()
        await omni.kit.app.get_app().next_update_async()
        await self.wait_for_publishers_on_topic(node, "/tf_test", timeout_sec=10.0, per_frame_callback=spin)
        await self.simulate_until_condition(
            lambda: self._tf_data is not None and len(self._tf_data.transforms) == 14,
            max_frames=240,
            per_frame_callback=spin,
            failure_message="Timed out waiting for the complete 14-transform pose tree",
        )

        # checks
        self.assertEqual(len(self._tf_data.transforms), 14)  # there are 12 items in the tree.
        self.assertEqual(self._tf_data.transforms[12].header.frame_id, "world")  # check cube's parent is world

        # the pose of panda_hand (a rigid body) should match the pose of the geometry xform (non rigid body) child.
        self.assertEqual(
            self._tf_data.transforms[12].transform.translation, self._tf_data.transforms[13].transform.translation
        )
        # print(self._tf_data.transforms)

        self._timeline.stop()
        await omni.kit.app.get_app().next_update_async()
        spin()
        self._tf_data_prev = self._tf_data
        self._tf_data = None

        # add a parent prim
        set_target_prims(
            primPath="/ActionGraph/PublishTF", inputName="inputs:parentPrim", targetPrimPaths=["/panda/panda_link0"]
        )

        self._timeline.play()
        await omni.kit.app.get_app().next_update_async()
        await self.wait_for_publishers_on_topic(node, "/tf_test", timeout_sec=10.0, per_frame_callback=spin)

        def received_reparented_tree() -> bool:
            if self._tf_data is None:
                return False
            return any(
                transform.header.frame_id == "panda_link0" and transform.child_frame_id == "panda_link1"
                for transform in self._tf_data.transforms
            )

        await self.simulate_until_condition(received_reparented_tree, max_frames=120, per_frame_callback=spin)

        # checks
        reparented_transform = next(
            transform for transform in self._tf_data.transforms if transform.child_frame_id == "panda_link1"
        )
        self.assertEqual(reparented_transform.header.frame_id, "panda_link0")

        self._timeline.stop()
        spin()

    async def test_duplicate_names_tree(self) -> None:
        """Test duplicate names tree."""
        import rclpy
        from tf2_msgs.msg import TFMessage

        await add_franka(self._assets_root_path)

        await add_cube("/cube0/cube", 0.75, (2.00, 0, 0.75))
        await add_cube("/cube1/cube", 0.75, (3.00, 0, 0.75))
        await add_cube("/cube2/cube", 0.75, (4.00, 0, 0.75))

        stage = omni.usd.get_context().get_stage()

        cube2 = stage.GetPrimAtPath("/cube2/cube")

        cube2.CreateAttribute(robot_schema.Attributes.NAME_OVERRIDE.name, Sdf.ValueTypeNames.String, True).Set(
            "Cube_override"
        )

        self._tf_data = None
        self._tf_data_prev = None

        def tf_callback(data: TFMessage) -> None:
            self._tf_data = data

        node = self.create_node("tf_tester")
        tf_sub = self.create_subscription(node, TFMessage, "/tf_test", tf_callback, 10)

        try:
            og.Controller.edit(
                {"graph_path": "/ActionGraph", "evaluator_name": "execution"},
                {
                    og.Controller.Keys.CREATE_NODES: [
                        ("OnPlaybackTick", "omni.graph.action.OnPlaybackTick"),
                        ("ReadSimTime", "isaacsim.core.nodes.IsaacReadSimulationTime"),
                        ("PublishTF", "isaacsim.ros2.bridge.ROS2PublishTransformTree"),
                    ],
                    og.Controller.Keys.SET_VALUES: [
                        ("PublishTF.inputs:topicName", "/tf_test"),
                        (
                            "PublishTF.inputs:targetPrims",
                            [
                                usdrt.Sdf.Path("/panda"),
                                usdrt.Sdf.Path("/cube0/cube"),
                                usdrt.Sdf.Path("/cube1/cube"),
                                usdrt.Sdf.Path("/cube2/cube"),
                            ],
                        ),
                        (
                            "PublishTF.inputs:parentPrim",
                            [usdrt.Sdf.Path("/panda")],
                        ),
                    ],
                    og.Controller.Keys.CONNECT: [
                        ("OnPlaybackTick.outputs:tick", "PublishTF.inputs:execIn"),
                        ("ReadSimTime.outputs:simulationTime", "PublishTF.inputs:timeStamp"),
                    ],
                },
            )
        except Exception as e:
            print(e)

        def spin() -> None:
            rclpy.spin_once(node, timeout_sec=0.01)

        self._timeline.play()
        await omni.kit.app.get_app().next_update_async()
        await self.wait_for_publishers_on_topic(node, "/tf_test", timeout_sec=10.0, per_frame_callback=spin)
        await self.simulate_until_condition(lambda: self._tf_data is not None, max_frames=120, per_frame_callback=spin)

        self._timeline.stop()
        await omni.kit.app.get_app().next_update_async()
        spin()
        self._tf_data_prev = self._tf_data
        self._tf_data = None

        # add a parent prim
        set_target_prims(
            primPath="/ActionGraph/PublishTF", inputName="inputs:parentPrim", targetPrimPaths=["/cube0/cube"]
        )

        self._timeline.play()
        await omni.kit.app.get_app().next_update_async()

        def received_reparented_duplicate_tree() -> bool:
            if self._tf_data is None:
                return False
            tf_by_child = {t.child_frame_id: t for t in self._tf_data.transforms}
            return all(
                child_frame in tf_by_child and tf_by_child[child_frame].header.frame_id == "cube"
                for child_frame in ("cube1_cube", "Cube_override")
            )

        await self.simulate_until_condition(
            received_reparented_duplicate_tree,
            max_frames=120,
            per_frame_callback=spin,
        )

        # checks
        tf_by_child = {t.child_frame_id: t for t in self._tf_data.transforms}
        self.assertEqual(tf_by_child["cube1_cube"].header.frame_id, "cube")
        self.assertEqual(tf_by_child["Cube_override"].header.frame_id, "cube")

        self._timeline.stop()
        spin()

    async def test_frame_name_override(self) -> None:
        """Test frame name override."""
        import rclpy
        from tf2_msgs.msg import TFMessage

        # Create two Franka robots at different paths

        asset_path = self._assets_root_path + "/Isaac/Robots_Multiphysics/FrankaRobotics/FrankaPanda/franka/franka.usda"
        stage_utils.add_reference_to_stage(usd_path=asset_path, path="/World/panda1")
        stage_utils.add_reference_to_stage(usd_path=asset_path, path="/World/panda2")

        stage = omni.usd.get_context().get_stage()

        # Verify robots were created
        panda1 = stage.GetPrimAtPath("/World/panda1")
        panda2 = stage.GetPrimAtPath("/World/panda2")

        self.assertTrue(panda1.IsValid(), "First robot not created successfully")
        self.assertTrue(panda2.IsValid(), "Second robot not created successfully")

        # Set position of second robot
        XformPrim(
            "/World/panda2",
            reset_xform_op_properties=True,
            positions=np.array([[1.5, 0.0, 0.0]]),
        )

        stage = omni.usd.get_context().get_stage()
        self._tf_data = None

        def tf_callback(data: TFMessage) -> None:
            self._tf_data = data

        node = self.create_node("tf_tester")
        self._tf_sub = self.create_subscription(node, TFMessage, "/tf_test", tf_callback, 10)

        try:
            og.Controller.edit(
                {"graph_path": "/ActionGraph", "evaluator_name": "execution"},
                {
                    og.Controller.Keys.CREATE_NODES: [
                        ("OnPlaybackTick", "omni.graph.action.OnPlaybackTick"),
                        ("ReadSimTime", "isaacsim.core.nodes.IsaacReadSimulationTime"),
                        ("PublishTF", "isaacsim.ros2.bridge.ROS2PublishTransformTree"),
                    ],
                    og.Controller.Keys.SET_VALUES: [
                        ("PublishTF.inputs:topicName", "/tf_test"),
                        (
                            "PublishTF.inputs:targetPrims",
                            [
                                usdrt.Sdf.Path("/World/panda1"),
                                usdrt.Sdf.Path("/World/panda2"),
                            ],
                        ),
                    ],
                    og.Controller.Keys.CONNECT: [
                        ("OnPlaybackTick.outputs:tick", "PublishTF.inputs:execIn"),
                        ("ReadSimTime.outputs:simulationTime", "PublishTF.inputs:timeStamp"),
                    ],
                },
            )
        except Exception as e:
            print(e)

        def spin() -> None:
            rclpy.spin_once(node, timeout_sec=0.01)

        franka_links = ["panda_link0", "panda_link1", "panda_link2", "panda_hand"]

        def received_uniquely_named_frames() -> bool:
            if self._tf_data is None:
                return False
            frame_ids = {transform.child_frame_id for transform in self._tf_data.transforms}
            return all(link in frame_ids and f"World_panda2_{link}" in frame_ids for link in franka_links)

        # Run the simulation which will trigger the CARB_LOG_WARN when processing the duplicate "base_link" names
        self._timeline.play()
        await omni.kit.app.get_app().next_update_async()
        await self.simulate_until_condition(
            received_uniquely_named_frames,
            max_frames=240,
            per_frame_callback=spin,
            failure_message="Timed out waiting for original and uniquely renamed transform frames",
        )

        self._timeline.stop()
        spin()

        # Verify transforms were published with unique names
        self.assertIsNotNone(self._tf_data)

        # Creating dict to store all frame IDs from both robots
        original_frames = set()
        renamed_frames = set()

        # Collect frame IDs from TF message
        frame_ids = []
        for transform in self._tf_data.transforms:
            frame_ids.append(transform.child_frame_id)

        # Check for original and renamed frames
        for link in franka_links:
            if link in frame_ids:
                original_frames.add(link)

            renamed_pattern = "World_panda2_" + link
            for frame_id in frame_ids:
                if renamed_pattern in frame_id:
                    renamed_frames.add(frame_id)

        # Verify we found original frames
        self.assertTrue(len(original_frames) > 0, f"No original frames found. All frames: {frame_ids}")

        # Verify we found renamed frames
        self.assertTrue(len(renamed_frames) > 0, f"No renamed frames found. All frames: {frame_ids}")

        # Verify for each original frame, there's a corresponding renamed frame
        for link in franka_links:
            if link in original_frames:
                renamed_exists = False
                expected_renamed = "World_panda2_" + link
                for renamed in renamed_frames:
                    if expected_renamed in renamed:
                        renamed_exists = True
                        break

                self.assertTrue(renamed_exists, f"Original frame {link} should have a renamed frames")

    async def test_tf_aggregation_raw(self) -> None:
        """Two raw TF nodes share one ROS publisher and emit one combined TFMessage."""
        from tf2_msgs.msg import TFMessage

        self._enable_tf_aggregation()

        topic_name = "/tf_aggregation_raw_test"
        expected_children = {"tf_aggregation_base", "tf_aggregation_camera"}
        tf_messages = []

        def tf_callback(data: TFMessage) -> None:
            tf_messages.append(data)

        node = self.create_node("tf_aggregation_raw_tester")
        tf_sub = self.create_subscription(node, TFMessage, topic_name, tf_callback, get_qos_profile())

        og.Controller.edit(
            {"graph_path": "/TfAggregationRawGraph", "evaluator_name": "execution"},
            {
                og.Controller.Keys.CREATE_NODES: [
                    ("OnPlaybackTick", "omni.graph.action.OnPlaybackTick"),
                    ("ReadSimTime", "isaacsim.core.nodes.IsaacReadSimulationTime"),
                    ("RawBase", "isaacsim.ros2.bridge.ROS2PublishRawTransformTree"),
                    ("RawCamera", "isaacsim.ros2.bridge.ROS2PublishRawTransformTree"),
                ],
                og.Controller.Keys.SET_VALUES: [
                    ("RawBase.inputs:topicName", topic_name),
                    ("RawBase.inputs:parentFrameId", "world"),
                    ("RawBase.inputs:childFrameId", "tf_aggregation_base"),
                    ("RawBase.inputs:translation", Gf.Vec3d(1.0, 2.0, 0.25)),
                    ("RawCamera.inputs:topicName", topic_name),
                    ("RawCamera.inputs:parentFrameId", "tf_aggregation_base"),
                    ("RawCamera.inputs:childFrameId", "tf_aggregation_camera"),
                    ("RawCamera.inputs:translation", Gf.Vec3d(0.5, -1.0, 1.5)),
                ],
                og.Controller.Keys.CONNECT: [
                    ("OnPlaybackTick.outputs:tick", "RawBase.inputs:execIn"),
                    ("OnPlaybackTick.outputs:tick", "RawCamera.inputs:execIn"),
                    ("ReadSimTime.outputs:simulationTime", "RawBase.inputs:timeStamp"),
                    ("ReadSimTime.outputs:simulationTime", "RawCamera.inputs:timeStamp"),
                ],
            },
        )

        message = await self._wait_for_aggregated_tf_message(node, topic_name, tf_messages, expected_children)
        self.assertEqual(len(message.transforms), 2)
        self._assert_same_tf_timestamp(message)

        transforms_by_child = {transform.child_frame_id: transform for transform in message.transforms}
        self.assertEqual(set(transforms_by_child), expected_children)
        self._assert_transform(transforms_by_child["tf_aggregation_base"], "world", (1.0, 2.0, 0.25))
        self._assert_transform(transforms_by_child["tf_aggregation_camera"], "tf_aggregation_base", (0.5, -1.0, 1.5))

    async def test_tf_aggregation_mixed(self) -> None:
        """Raw and transform-tree TF nodes aggregate into one combined TFMessage."""
        from tf2_msgs.msg import TFMessage

        self._enable_tf_aggregation()

        topic_name = "/tf_aggregation_mixed_test"
        expected_children = {"tf_aggregation_raw_child", "tf_aggregation_tree_child"}
        tf_messages = []

        def tf_callback(data: TFMessage) -> None:
            tf_messages.append(data)

        node = self.create_node("tf_aggregation_mixed_tester")
        tf_sub = self.create_subscription(node, TFMessage, topic_name, tf_callback, get_qos_profile())

        og.Controller.edit(
            {"graph_path": "/TfAggregationMixedGraph", "evaluator_name": "execution"},
            {
                og.Controller.Keys.CREATE_NODES: [
                    ("OnPlaybackTick", "omni.graph.action.OnPlaybackTick"),
                    ("ReadSimTime", "isaacsim.core.nodes.IsaacReadSimulationTime"),
                    ("RawTF", "isaacsim.ros2.bridge.ROS2PublishRawTransformTree"),
                    ("TreeTF", "isaacsim.ros2.bridge.ROS2PublishTransformTree"),
                ],
                og.Controller.Keys.SET_VALUES: [
                    ("RawTF.inputs:topicName", topic_name),
                    ("RawTF.inputs:parentFrameId", "world"),
                    ("RawTF.inputs:childFrameId", "tf_aggregation_raw_child"),
                    ("RawTF.inputs:translation", Gf.Vec3d(2.0, 0.0, 0.0)),
                    ("TreeTF.inputs:topicName", topic_name),
                    ("TreeTF.inputs:parentFrames", ["world"]),
                    ("TreeTF.inputs:childFrames", ["tf_aggregation_tree_child"]),
                    ("TreeTF.inputs:translations", [Gf.Vec3d(0.0, 3.0, 0.5)]),
                    ("TreeTF.inputs:orientations", [Gf.Vec4d(0.0, 0.0, 0.0, 1.0)]),
                ],
                og.Controller.Keys.CONNECT: [
                    ("OnPlaybackTick.outputs:tick", "RawTF.inputs:execIn"),
                    ("OnPlaybackTick.outputs:tick", "TreeTF.inputs:execIn"),
                    ("ReadSimTime.outputs:simulationTime", "RawTF.inputs:timeStamp"),
                    ("ReadSimTime.outputs:simulationTime", "TreeTF.inputs:timeStamp"),
                ],
            },
        )

        message = await self._wait_for_aggregated_tf_message(node, topic_name, tf_messages, expected_children)
        self.assertEqual(len(message.transforms), 2)
        self._assert_same_tf_timestamp(message)

        transforms_by_child = {transform.child_frame_id: transform for transform in message.transforms}
        self.assertEqual(set(transforms_by_child), expected_children)
        self._assert_transform(transforms_by_child["tf_aggregation_raw_child"], "world", (2.0, 0.0, 0.0))
        self._assert_transform(transforms_by_child["tf_aggregation_tree_child"], "world", (0.0, 3.0, 0.5))

    async def test_tf_aggregation_static(self) -> None:
        """Static raw TF nodes share one transient-local publisher and do not republish unchanged transforms."""
        import rclpy
        from rclpy.qos import QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy
        from tf2_msgs.msg import TFMessage

        self._enable_tf_aggregation()

        topic_name = "/tf_static_aggregation_test"
        expected_children = {"tf_aggregation_static_base", "tf_aggregation_static_sensor"}
        tf_messages = []

        def tf_callback(data: TFMessage) -> None:
            tf_messages.append(data)

        node = self.create_node("tf_aggregation_static_tester")
        tf_static_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10,
        )
        tf_static_sub = self.create_subscription(node, TFMessage, topic_name, tf_callback, tf_static_qos)

        og.Controller.edit(
            {"graph_path": "/TfAggregationStaticGraph", "evaluator_name": "execution"},
            {
                og.Controller.Keys.CREATE_NODES: [
                    ("OnPlaybackTick", "omni.graph.action.OnPlaybackTick"),
                    ("ReadSimTime", "isaacsim.core.nodes.IsaacReadSimulationTime"),
                    ("StaticBase", "isaacsim.ros2.bridge.ROS2PublishRawTransformTree"),
                    ("StaticSensor", "isaacsim.ros2.bridge.ROS2PublishRawTransformTree"),
                ],
                og.Controller.Keys.SET_VALUES: [
                    ("StaticBase.inputs:topicName", topic_name),
                    ("StaticBase.inputs:staticPublisher", True),
                    ("StaticBase.inputs:parentFrameId", "world"),
                    ("StaticBase.inputs:childFrameId", "tf_aggregation_static_base"),
                    ("StaticBase.inputs:translation", Gf.Vec3d(0.25, 0.0, 0.0)),
                    ("StaticSensor.inputs:topicName", topic_name),
                    ("StaticSensor.inputs:staticPublisher", True),
                    ("StaticSensor.inputs:parentFrameId", "tf_aggregation_static_base"),
                    ("StaticSensor.inputs:childFrameId", "tf_aggregation_static_sensor"),
                    ("StaticSensor.inputs:translation", Gf.Vec3d(0.0, 0.0, 1.25)),
                ],
                og.Controller.Keys.CONNECT: [
                    ("OnPlaybackTick.outputs:tick", "StaticBase.inputs:execIn"),
                    ("OnPlaybackTick.outputs:tick", "StaticSensor.inputs:execIn"),
                    ("ReadSimTime.outputs:simulationTime", "StaticBase.inputs:timeStamp"),
                    ("ReadSimTime.outputs:simulationTime", "StaticSensor.inputs:timeStamp"),
                ],
            },
        )

        def spin() -> None:
            rclpy.spin_once(node, timeout_sec=0.01)

        self._timeline.play()
        await omni.kit.app.get_app().next_update_async()
        await self.wait_for_publishers_on_topic(node, topic_name, timeout_sec=10.0, per_frame_callback=spin)
        self.assertEqual(node.count_publishers(topic_name), 1)
        await self.simulate_until_condition(
            lambda: self._find_tf_message_with_children(tf_messages, expected_children) is not None,
            max_frames=120,
            per_frame_callback=spin,
            failure_message=f"Expected aggregated static TF message on {topic_name}",
        )

        message = self._find_tf_message_with_children(tf_messages, expected_children)
        self.assertIsNotNone(message)
        self.assertEqual(len(message.transforms), 2)
        self._assert_same_tf_timestamp(message)

        transforms_by_child = {transform.child_frame_id: transform for transform in message.transforms}
        self.assertEqual(set(transforms_by_child), expected_children)
        self._assert_transform(transforms_by_child["tf_aggregation_static_base"], "world", (0.25, 0.0, 0.0))
        self._assert_transform(
            transforms_by_child["tf_aggregation_static_sensor"], "tf_aggregation_static_base", (0.0, 0.0, 1.25)
        )

        for _ in range(5):
            await omni.kit.app.get_app().next_update_async()
            spin()
        delivered_count = len(tf_messages)
        for _ in range(20):
            await omni.kit.app.get_app().next_update_async()
            spin()
        self.assertEqual(len(tf_messages), delivered_count)

        self._timeline.stop()
        await omni.kit.app.get_app().next_update_async()
        spin()

    async def test_compute_transform_tree_pipeline(self) -> None:
        """Test OgnIsaacComputeTransformTree -> OgnROS2PublishTransformTree external data path."""
        import rclpy
        from tf2_msgs.msg import TFMessage

        await add_franka(self._assets_root_path)

        self._tf_data = None

        def tf_callback(data: TFMessage) -> None:
            self._tf_data = data

        node = self.create_node("tf_pipeline_tester")
        tf_sub = self.create_subscription(node, TFMessage, "/tf_pipeline_test", tf_callback, get_qos_profile())

        try:
            og.Controller.edit(
                {"graph_path": "/ActionGraph", "evaluator_name": "execution"},
                {
                    og.Controller.Keys.CREATE_NODES: [
                        ("OnPlaybackTick", "omni.graph.action.OnPlaybackTick"),
                        ("ReadSimTime", "isaacsim.core.nodes.IsaacReadSimulationTime"),
                        ("ComputeTF", "isaacsim.core.nodes.IsaacComputeTransformTree"),
                        ("PublishTF", "isaacsim.ros2.bridge.ROS2PublishTransformTree"),
                    ],
                    og.Controller.Keys.SET_VALUES: [
                        ("PublishTF.inputs:topicName", "/tf_pipeline_test"),
                        ("ComputeTF.inputs:targetPrims", [usdrt.Sdf.Path("/panda")]),
                    ],
                    og.Controller.Keys.CONNECT: [
                        ("OnPlaybackTick.outputs:tick", "ComputeTF.inputs:execIn"),
                        ("ComputeTF.outputs:execOut", "PublishTF.inputs:execIn"),
                        ("ComputeTF.outputs:parentFrames", "PublishTF.inputs:parentFrames"),
                        ("ComputeTF.outputs:childFrames", "PublishTF.inputs:childFrames"),
                        ("ComputeTF.outputs:translations", "PublishTF.inputs:translations"),
                        ("ComputeTF.outputs:orientations", "PublishTF.inputs:orientations"),
                        ("ReadSimTime.outputs:simulationTime", "PublishTF.inputs:timeStamp"),
                    ],
                },
            )
        except Exception as e:
            print(e)

        def spin() -> None:
            rclpy.spin_once(node, timeout_sec=0.01)

        self._timeline.play()
        await omni.kit.app.get_app().next_update_async()
        await self.simulate_until_condition(lambda: self._tf_data is not None, max_frames=60, per_frame_callback=spin)

        self.assertIsNotNone(self._tf_data, "Expected TF data to be published via compute pipeline")
        self.assertGreater(len(self._tf_data.transforms), 0, "Expected at least one transform")

        # Without a parentPrim on ComputeTF, root link parent frame should be "world"
        root_transforms = [t for t in self._tf_data.transforms if t.header.frame_id == "world"]
        self.assertGreater(len(root_transforms), 0, "Expected at least one transform with 'world' as parent frame")

        # Franka has 11 rigid body links detected via UsdPhysicsRigidBodyAPI traversal
        self.assertEqual(len(self._tf_data.transforms), 11, "Expected 11 transforms for Franka articulation")

        self._timeline.stop()
        spin()
