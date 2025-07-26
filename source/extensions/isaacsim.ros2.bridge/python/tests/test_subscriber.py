# SPDX-FileCopyrightText: Copyright (c) 2018-2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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


import json

import omni.graph.core as og
import omni.kit.test
from isaacsim.core.utils.physics import simulate_async
from isaacsim.core.utils.stage import create_new_stage_async

from .common import ROS2TestCase


class TestRos2Subscriber(ROS2TestCase):
    # Before running each test
    async def setUp(self):
        await super().setUp()

        await create_new_stage_async()

    # After running each test
    async def tearDown(self):
        await super().tearDown()

    # ----------------------------------------------------------------------
    async def test_subscriber(self):
        import builtin_interfaces.msg
        import rclpy

        # define graph
        (test_graph, new_nodes, _, _) = og.Controller.edit(
            {"graph_path": "/ActionGraph", "evaluator_name": "execution"},
            {
                og.Controller.Keys.CREATE_NODES: [
                    ("OnPlaybackTick", "omni.graph.action.OnPlaybackTick"),
                    ("Subscriber", "isaacsim.ros2.bridge.ROS2Subscriber"),
                ],
                og.Controller.Keys.SET_VALUES: [
                    ("Subscriber.inputs:topicName", "custom_topic"),
                ],
                og.Controller.Keys.CONNECT: [
                    ("OnPlaybackTick.outputs:tick", "Subscriber.inputs:execIn"),
                ],
            },
        )
        subscriber_node = new_nodes[-1]

        ros2_publisher = None
        ros2_node = rclpy.create_node("isaac_sim_test_subscriber")

        messages = []

        # std_msgs
        try:
            import std_msgs.msg
        except ModuleNotFoundError:
            pass
        else:
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
        # trajectory_msgs
        try:
            import trajectory_msgs.msg
        except ModuleNotFoundError:
            pass
        else:
            _duration = builtin_interfaces.msg.Duration(sec=10, nanosec=20)
            _points = [
                trajectory_msgs.msg.JointTrajectoryPoint(
                    positions=[1.0, 2.0, 3.0], velocities=[-1.0, -2.0, -3.0], time_from_start=_duration
                ),
                trajectory_msgs.msg.JointTrajectoryPoint(
                    accelerations=[4.0, 5.0, 6.0], effort=[-4.0, -5.0, -6.0], time_from_start=_duration
                ),
            ]
            messages += [
                (
                    "trajectory_msgs.msg.JointTrajectory",
                    trajectory_msgs.msg.JointTrajectory(joint_names=["abc", "def"], points=_points),
                ),
            ]

        for message_type, message_value in messages:
            print(message_type)
            # create publisher
            if ros2_publisher:
                ros2_node.destroy_publisher(ros2_publisher)
                ros2_publisher = None
            ros2_publisher = ros2_node.create_publisher(eval(message_type), "custom_topic", 10)

            # change message type
            await omni.kit.app.get_app().next_update_async()
            og.Controller.attribute("inputs:messageName", subscriber_node).set("")
            await omni.kit.app.get_app().next_update_async()
            message_package = message_type.split(".")[0]
            message_subfolder = message_type.split(".")[1]
            message_name = message_type.split(".")[2]
            og.Controller.attribute("inputs:messagePackage", subscriber_node).set(message_package)
            og.Controller.attribute("inputs:messageSubfolder", subscriber_node).set(message_subfolder)
            og.Controller.attribute("inputs:messageName", subscriber_node).set(message_name)

            self._timeline.play()
            await simulate_async(0.5)

            # publish value
            ros2_publisher.publish(message_value)
            await simulate_async(0.5)

            # check node output
            if message_type.startswith("trajectory_msgs"):
                joint_names = og.Controller.attribute("outputs:joint_names", subscriber_node).get()
                points = og.Controller.attribute("outputs:points", subscriber_node).get()

                for md, d in zip(message_value.joint_names, joint_names):
                    self.assertEqual(md, d)
                self.assertEqual(len(message_value.points), len(points))
                for md, d in zip(message_value.points, points):
                    point = json.loads(d)
                    for mv, v in zip(md.positions, point["positions"]):
                        self.assertAlmostEqual(mv, v)
                    for mv, v in zip(md.velocities, point["velocities"]):
                        self.assertAlmostEqual(mv, v)
                    for mv, v in zip(md.accelerations, point["accelerations"]):
                        self.assertAlmostEqual(mv, v)
                    for mv, v in zip(md.effort, point["effort"]):
                        self.assertAlmostEqual(mv, v)
                    time_from_start = point["time_from_start"]
                    self.assertEqual(md.time_from_start.sec, time_from_start["sec"])
                    self.assertEqual(md.time_from_start.nanosec, time_from_start["nanosec"])

            # array
            elif message_type.endswith("Array"):
                data = og.Controller.attribute("outputs:data", subscriber_node).get()
                layout_data_offset = og.Controller.attribute("outputs:layout:data_offset", subscriber_node).get()
                layout_dim = og.Controller.attribute("outputs:layout:dim", subscriber_node).get()

                for md, d in zip(message_value.data, data):
                    if message_type == "std_msgs.msg.ByteMultiArray":
                        self.assertEqual(ord(md.decode()), d)
                    elif message_type == "std_msgs.msg.Float32MultiArray":
                        self.assertAlmostEqual(md, d)
                    else:
                        self.assertEqual(md, d)
                self.assertEqual(message_value.layout.data_offset, layout_data_offset)
                self.assertEqual(len(message_value.layout.dim), len(layout_dim))
                for md, d in zip(message_value.layout.dim, layout_dim):
                    dim = json.loads(d)
                    self.assertEqual(md.label, dim["label"])
                    self.assertEqual(md.size, dim["size"])
                    self.assertEqual(md.stride, dim["stride"])

            # single value
            else:
                data = og.Controller.attribute("outputs:data", subscriber_node).get()

                if message_type == "std_msgs.msg.Byte":
                    self.assertEqual(ord(message_value.data.decode()), data)
                elif message_type == "std_msgs.msg.Float32":
                    self.assertAlmostEqual(message_value.data, data)
                else:
                    self.assertEqual(message_value.data, data)
