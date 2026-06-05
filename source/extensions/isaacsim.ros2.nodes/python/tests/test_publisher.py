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

"""Tests for ROS 2 generic publisher OmniGraph node."""

import json

import numpy as np
import omni.graph.core as og
import omni.kit.test
from isaacsim.core.experimental.utils import stage as stage_utils
from isaacsim.ros2.core.impl.ros2_test_case import ROS2TestCase


class TestRos2Publisher(ROS2TestCase):
    """Test suite for ros2 publisher."""

    async def setUp(self):
        """Set up test fixtures."""
        await super().setUp()

        await stage_utils.create_new_stage_async()

    async def tearDown(self):
        """Tear down test fixtures."""
        await super().tearDown()

    # ----------------------------------------------------------------------
    def _callback(self, msg):
        self._ros_message = msg
        # print("  |--", msg)

    async def test_publisher(self):
        """Test publisher."""
        import builtin_interfaces.msg
        import geometry_msgs.msg
        import rclpy
        import shape_msgs.msg
        import std_msgs.msg
        import tf2_msgs.msg

        # define graph
        test_graph, new_nodes, _, _ = og.Controller.edit(
            {"graph_path": "/ActionGraph", "evaluator_name": "execution"},
            {
                og.Controller.Keys.CREATE_NODES: [
                    ("OnPlaybackTick", "omni.graph.action.OnPlaybackTick"),
                    ("Publisher", "isaacsim.ros2.bridge.ROS2Publisher"),
                ],
                og.Controller.Keys.SET_VALUES: [
                    ("Publisher.inputs:topicName", "custom_topic"),
                ],
                og.Controller.Keys.CONNECT: [
                    ("OnPlaybackTick.outputs:tick", "Publisher.inputs:execIn"),
                ],
            },
        )
        ogn_node = new_nodes[-1]

        ros2_subscriber = None
        ros2_node = self.create_node("isaac_sim_test_publisher")
        qos_profile = rclpy.qos.QoSPresetProfiles.SYSTEM_DEFAULT.value

        # define messages
        messages = []
        # - std_msgs
        _layout = std_msgs.msg.MultiArrayLayout()
        _layout.data_offset = 100
        _layout.dim = [
            std_msgs.msg.MultiArrayDimension(label="dim0", size=1, stride=2),
            std_msgs.msg.MultiArrayDimension(label="dim1", size=2, stride=3),
        ]
        messages += [
            ("std_msgs.msg.Bool", std_msgs.msg.Bool(data=True)),
            ("std_msgs.msg.Byte", std_msgs.msg.Byte(data=b"a")),
            ("std_msgs.msg.ByteMultiArray", std_msgs.msg.ByteMultiArray(layout=_layout, data=[b"a", b"0"])),
            ("std_msgs.msg.Char", std_msgs.msg.Char(data=ord("b"))),
            # ("std_msgs.msg.Empty", None),
            ("std_msgs.msg.Float32", std_msgs.msg.Float32(data=1.0e-38)),
            (
                "std_msgs.msg.Float32MultiArray",
                std_msgs.msg.Float32MultiArray(layout=_layout, data=[-1.0e-38, 0.0, 1.0e-38]),
            ),
            # ("std_msgs.msg.Float64", 1.0e-38),
            # ("std_msgs.msg.Float64MultiArray", ),
            # ("std_msgs.msg.Header", ),
            ("std_msgs.msg.Int16", std_msgs.msg.Int16(data=-(2**15))),
            (
                "std_msgs.msg.Int16MultiArray",
                std_msgs.msg.Int16MultiArray(layout=_layout, data=[-(2**15), 0, 2**15 - 1]),
            ),
            ("std_msgs.msg.Int32", std_msgs.msg.Int32(data=-(2**31))),
            (
                "std_msgs.msg.Int32MultiArray",
                std_msgs.msg.Int32MultiArray(layout=_layout, data=[-(2**31), 0, 2**31 - 1]),
            ),
            # ("std_msgs.msg.Int64", -(2**63)),
            # ("std_msgs.msg.Int64MultiArray", ),
            ("std_msgs.msg.Int8", std_msgs.msg.Int8(data=-(2**7))),
            (
                "std_msgs.msg.Int8MultiArray",
                std_msgs.msg.Int8MultiArray(layout=_layout, data=[-(2**7), 0, 2**7 - 1]),
            ),
            # ("std_msgs.msg.MultiArrayDimension", ),
            # ("std_msgs.msg.MultiArrayLayout", ),
            ("std_msgs.msg.String", std_msgs.msg.String(data="abc")),
            ("std_msgs.msg.UInt16", std_msgs.msg.UInt16(data=2**16 - 1)),
            ("std_msgs.msg.UInt16MultiArray", std_msgs.msg.UInt16MultiArray(layout=_layout, data=[0, 2**16 - 1])),
            ("std_msgs.msg.UInt32", std_msgs.msg.UInt32(data=2**32 - 1)),
            ("std_msgs.msg.UInt32MultiArray", std_msgs.msg.UInt32MultiArray(layout=_layout, data=[0, 2**32 - 1])),
            # ("std_msgs.msg.UInt64", 2**64 - 1),
            # ("std_msgs.msg.UInt64MultiArray", ),
            ("std_msgs.msg.UInt8", std_msgs.msg.UInt8(data=2**8 - 1)),
            ("std_msgs.msg.UInt8MultiArray", std_msgs.msg.UInt8MultiArray(layout=_layout, data=[0, 2**8 - 1])),
        ]
        # - shape_msgs
        messages += [
            ("shape_msgs.msg.MeshTriangle", shape_msgs.msg.MeshTriangle(vertex_indices=[10, 20, 30])),
        ]
        # - tf2_msgs
        _transforms = [
            geometry_msgs.msg.TransformStamped(
                header=std_msgs.msg.Header(
                    frame_id=f"header_{i}", stamp=builtin_interfaces.msg.Time(sec=i, nanosec=int(2 * i))
                ),
                child_frame_id=f"child_{i}",
                transform=geometry_msgs.msg.Transform(
                    translation=geometry_msgs.msg.Vector3(x=float(i), y=float(i + 1), z=float(i + 2)),
                    rotation=geometry_msgs.msg.Quaternion(x=float(i), y=float(i - 1), z=float(i - 2), w=float(i - 3)),
                ),
            )
            for i in range(2**12)
        ]
        messages += [
            (
                "tf2_msgs.msg.TFMessage",
                tf2_msgs.msg.TFMessage(transforms=_transforms),
            ),
        ]

        for message_type, message_value in messages:
            print(message_type)
            # create subscriber
            if ros2_subscriber:
                self.destroy_subscription(ros2_node, ros2_subscriber)
                ros2_subscriber = None
            ros2_subscriber = self.create_subscription(
                ros2_node, eval(message_type), "custom_topic", self._callback, qos_profile
            )

            # change message type
            await omni.kit.app.get_app().next_update_async()
            og.Controller.attribute("inputs:messageName", ogn_node).set("")
            await omni.kit.app.get_app().next_update_async()
            message_package = message_type.split(".")[0]
            message_subfolder = message_type.split(".")[1]
            message_name = message_type.split(".")[2]
            og.Controller.attribute("inputs:messagePackage", ogn_node).set(message_package)
            og.Controller.attribute("inputs:messageSubfolder", ogn_node).set(message_subfolder)
            og.Controller.attribute("inputs:messageName", ogn_node).set(message_name)

            # set values to be published
            # - shape_msgs
            if message_type == "shape_msgs.msg.MeshTriangle":
                og.Controller.attribute("inputs:vertex_indices", ogn_node).set(list(message_value.vertex_indices))
            # - tf2_msgs
            elif message_type.startswith("tf2_msgs"):
                og.Controller.attribute("inputs:transforms", ogn_node).set(
                    [
                        f'{{"header": {{"frame_id": "{t.header.frame_id}", "stamp": {{"sec": {t.header.stamp.sec}, "nanosec": {t.header.stamp.nanosec}}}}}, '
                        f'"child_frame_id": "{t.child_frame_id}", '
                        f'"transform": {{"translation": {{"x": {t.transform.translation.x}, "y": {t.transform.translation.y}, "z": {t.transform.translation.z}}}, '
                        f'"rotation": {{"x": {t.transform.rotation.x}, "y": {t.transform.rotation.y}, "z": {t.transform.rotation.z}, "w": {t.transform.rotation.w}}}}}}}'
                        for t in message_value.transforms
                    ]
                )
            # - array
            elif message_type.endswith("Array"):
                if message_type == "std_msgs.msg.ByteMultiArray":
                    og.Controller.attribute("inputs:data", ogn_node).set([ord(d) for d in message_value.data])
                else:
                    og.Controller.attribute("inputs:data", ogn_node).set(np.array(message_value.data).tolist())
                og.Controller.attribute("inputs:layout:data_offset", ogn_node).set(message_value.layout.data_offset)
                og.Controller.attribute("inputs:layout:dim", ogn_node).set(
                    [
                        json.dumps({"label": dim.label, "size": dim.size, "stride": dim.stride})
                        for dim in message_value.layout.dim
                    ]
                )
            # - single value
            else:
                if message_type == "std_msgs.msg.Byte":
                    og.Controller.attribute("inputs:data", ogn_node).set(ord(message_value.data.decode()))
                else:
                    og.Controller.attribute("inputs:data", ogn_node).set(message_value.data)

            self._ros_message = None
            self._timeline.play()

            def spin_callback():
                rclpy.spin_once(ros2_node, timeout_sec=0)

            await self.wait_for_publishers_on_topic(ros2_node, "custom_topic", per_frame_callback=spin_callback)

            condition_met = await self.simulate_until_condition(
                condition_func=lambda: self._ros_message is not None,
                max_frames=300,
                per_frame_callback=spin_callback,
            )
            self.assertTrue(condition_met, f"Timed out waiting for message: {message_type}")

            # check node implementation
            # - shape_msgs
            if message_type == "shape_msgs.msg.MeshTriangle":
                vertex_indices = [*message_value.vertex_indices + [0] * 3][:3]  # default is 0 if not set
                np.testing.assert_array_equal(self._ros_message.vertex_indices, vertex_indices)
            # - tf2_msgs
            elif message_type.startswith("tf2_msgs"):
                transforms = self._ros_message.transforms

                self.assertEqual(len(message_value.transforms), len(transforms))

                expected_frame_ids = [t.header.frame_id for t in message_value.transforms]
                received_frame_ids = [t.header.frame_id for t in transforms]
                self.assertListEqual(expected_frame_ids, received_frame_ids)

                expected_child_ids = [t.child_frame_id for t in message_value.transforms]
                received_child_ids = [t.child_frame_id for t in transforms]
                self.assertListEqual(expected_child_ids, received_child_ids)

                expected_nums = np.array(
                    [
                        [
                            t.header.stamp.sec,
                            t.header.stamp.nanosec,
                            t.transform.translation.x,
                            t.transform.translation.y,
                            t.transform.translation.z,
                            t.transform.rotation.x,
                            t.transform.rotation.y,
                            t.transform.rotation.z,
                            t.transform.rotation.w,
                        ]
                        for t in message_value.transforms
                    ]
                )
                received_nums = np.array(
                    [
                        [
                            t.header.stamp.sec,
                            t.header.stamp.nanosec,
                            t.transform.translation.x,
                            t.transform.translation.y,
                            t.transform.translation.z,
                            t.transform.rotation.x,
                            t.transform.rotation.y,
                            t.transform.rotation.z,
                            t.transform.rotation.w,
                        ]
                        for t in transforms
                    ]
                )
                np.testing.assert_array_equal(expected_nums, received_nums)
            # - array
            elif message_type.endswith("Array"):
                data = self._ros_message.data
                layout_data_offset = self._ros_message.layout.data_offset
                layout_dim = self._ros_message.layout.dim

                self.assertEqual(len(message_value.data), len(data))
                for md, d in zip(message_value.data, data):
                    if message_type == "std_msgs.msg.Float32MultiArray":
                        self.assertAlmostEqual(md, d)
                    else:
                        self.assertEqual(md, d)
                self.assertEqual(message_value.layout.data_offset, layout_data_offset)
                self.assertEqual(len(message_value.layout.dim), len(layout_dim))
                for md, dim in zip(message_value.layout.dim, layout_dim):
                    self.assertEqual(md.label, dim.label)
                    self.assertEqual(md.size, dim.size)
                    self.assertEqual(md.stride, dim.stride)
            # - single value
            else:
                data = self._ros_message.data

                if message_type == "std_msgs.msg.Byte":
                    self.assertEqual(ord(message_value.data.decode()), ord(data.decode()))
                elif message_type == "std_msgs.msg.Float32":
                    self.assertAlmostEqual(message_value.data, data)
                else:
                    self.assertEqual(message_value.data, data)

            self._timeline.stop()
            await omni.kit.app.get_app().next_update_async()
