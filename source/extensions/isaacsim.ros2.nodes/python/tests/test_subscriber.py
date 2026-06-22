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

"""Verifies generic ROS 2 subscriber OmniGraph node behavior in the Isaac Sim ROS 2 test environment."""

import json
from typing import Any

import numpy as np
import omni.graph.core as og
import omni.kit.test
from isaacsim.core.experimental.utils import stage as stage_utils
from isaacsim.ros2.core.impl.ros2_test_case import ROS2TestCase


class TestRos2Subscriber(ROS2TestCase):
    """Verify generic ROS 2 subscriber OmniGraph node output attributes."""

    async def setUp(self) -> None:
        """Create a fresh stage for generic subscriber graph tests."""
        await super().setUp()

        await stage_utils.create_new_stage_async()

    async def tearDown(self) -> None:
        """Run shared ROS 2 cleanup after generic subscriber graph tests."""
        await super().tearDown()

    # ----------------------------------------------------------------------
    async def test_subscriber(self) -> Any:
        """Test subscriber.

        Returns:
            None.
        """
        import math

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
        ros2_node = self.create_node("isaac_sim_test_subscriber")

        def spin() -> None:
            # Keep ROS communications responsive while the timeline is running.
            try:
                rclpy.spin_once(ros2_node, timeout_sec=0.0)
            except Exception:
                # Spin is a best-effort helper here; discovery may still succeed without it.
                pass

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
            # create publisher
            if ros2_publisher:
                self.destroy_publisher(ros2_node, ros2_publisher)
                ros2_publisher = None
            ros2_publisher = self.create_publisher(ros2_node, eval(message_type), "custom_topic", 10)

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

            # Wait until the publisher sees an active subscription before publishing.
            # This helps avoid dropping the message due to DDS discovery timing.
            subscription_ready = await self.simulate_until_condition(
                lambda: ros2_publisher.get_subscription_count() > 0,
                max_frames=300,
                per_frame_callback=spin,
            )
            self.assertTrue(
                subscription_ready,
                f"Timed out waiting for ROS2 subscription discovery for {message_type}",
            )

            # publish value
            ros2_publisher.publish(message_value)

            # Wait for node output to update (instead of fixed sleeps).
            if message_type == "shape_msgs.msg.MeshTriangle":

                def condition() -> Any:
                    vertex_indices = og.Controller.attribute("outputs:vertex_indices", subscriber_node).get()
                    vertex_indices = [*list(vertex_indices) + [0] * 3][:3]
                    return vertex_indices is not None and np.array_equal(vertex_indices, message_value.vertex_indices)

                condition_met = await self.simulate_until_condition(condition, max_frames=600, per_frame_callback=spin)
            elif message_type.startswith("tf2_msgs"):

                def condition() -> Any:
                    transforms = og.Controller.attribute("outputs:transforms", subscriber_node).get()
                    return transforms is not None and len(transforms) == len(message_value.transforms)

                condition_met = await self.simulate_until_condition(condition, max_frames=900, per_frame_callback=spin)
            elif message_type.endswith("Array"):

                def condition() -> Any:
                    data = og.Controller.attribute("outputs:data", subscriber_node).get()
                    layout_data_offset = og.Controller.attribute("outputs:layout:data_offset", subscriber_node).get()
                    layout_dim = og.Controller.attribute("outputs:layout:dim", subscriber_node).get()
                    return (
                        data is not None
                        and layout_dim is not None
                        and len(data) == len(message_value.data)
                        and layout_data_offset == message_value.layout.data_offset
                        and len(layout_dim) == len(message_value.layout.dim)
                    )

                condition_met = await self.simulate_until_condition(condition, max_frames=600, per_frame_callback=spin)
            else:

                def condition() -> Any:
                    data = og.Controller.attribute("outputs:data", subscriber_node).get()
                    if message_type == "std_msgs.msg.Byte":
                        return data == ord(message_value.data.decode())
                    if message_type == "std_msgs.msg.Float32":
                        return data is not None and math.isfinite(data) and abs(message_value.data - data) < 1e-6
                    return data == message_value.data

                condition_met = await self.simulate_until_condition(condition, max_frames=600, per_frame_callback=spin)

            self.assertTrue(condition_met, f"Timed out waiting for subscriber output for {message_type}")

            # check node output
            # - shape_msgs
            if message_type == "shape_msgs.msg.MeshTriangle":
                vertex_indices = og.Controller.attribute("outputs:vertex_indices", subscriber_node).get()
                np.testing.assert_array_equal(vertex_indices, message_value.vertex_indices)
            # - tf2_msgs
            elif message_type.startswith("tf2_msgs"):
                transforms = og.Controller.attribute("outputs:transforms", subscriber_node).get()
                self.assertEqual(len(message_value.transforms), len(transforms))

                parsed = [json.loads(d) for d in transforms]

                expected_frame_ids = [t.header.frame_id for t in message_value.transforms]
                received_frame_ids = [p["header"]["frame_id"] for p in parsed]
                self.assertListEqual(expected_frame_ids, received_frame_ids)

                expected_child_ids = [t.child_frame_id for t in message_value.transforms]
                received_child_ids = [p["child_frame_id"] for p in parsed]
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
                            p["header"]["stamp"]["sec"],
                            p["header"]["stamp"]["nanosec"],
                            p["transform"]["translation"]["x"],
                            p["transform"]["translation"]["y"],
                            p["transform"]["translation"]["z"],
                            p["transform"]["rotation"]["x"],
                            p["transform"]["rotation"]["y"],
                            p["transform"]["rotation"]["z"],
                            p["transform"]["rotation"]["w"],
                        ]
                        for p in parsed
                    ]
                )
                np.testing.assert_array_equal(expected_nums, received_nums)
            # - array
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
            # - single value
            else:
                data = og.Controller.attribute("outputs:data", subscriber_node).get()

                if message_type == "std_msgs.msg.Byte":
                    self.assertEqual(ord(message_value.data.decode()), data)
                elif message_type == "std_msgs.msg.Float32":
                    self.assertAlmostEqual(message_value.data, data)
                else:
                    self.assertEqual(message_value.data, data)
