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

import asyncio
import json

import numpy as np
import omni.graph.core as og
import omni.kit.test
import omni.usd
from isaacsim.core.utils.physics import simulate_async
from isaacsim.core.utils.stage import create_new_stage_async
from pxr import Sdf

from .common import ROS2TestCase


class TestRos2ServicePrim(ROS2TestCase):
    # Before running each test
    async def setUp(self):
        await super().setUp()
        await create_new_stage_async()

    # After running each test
    async def tearDown(self):
        await super().tearDown()

    def createAttributes(self, prim_path):
        def rand(size, dtype="float", as_list=False):
            # list
            if as_list:
                return [rand(size, dtype, False) for _ in range(np.random.randint(1, 6))]
            # single value
            if dtype in ["int", "uint"]:
                value = np.random.randint(-127 if dtype == "int" else 0, 127, size).tolist()
                return value[0] if size == (1,) else value
            elif dtype in ["half", "float", "double"]:
                value = (np.round(np.random.uniform(-100, 100, size), 2)).tolist()
                return value[0] if size == (1,) else value
            else:
                raise ValueError

        specs = [
            (Sdf.ValueTypeNames.Asset, json.dumps("./abc")),
            (Sdf.ValueTypeNames.AssetArray, json.dumps(["./abc", "./def"])),
            (Sdf.ValueTypeNames.Bool, json.dumps(True)),
            (Sdf.ValueTypeNames.BoolArray, json.dumps([True, False])),
            (Sdf.ValueTypeNames.Color3d, json.dumps(rand((3,), "double"))),
            (Sdf.ValueTypeNames.Color3dArray, json.dumps(rand((3,), "double", True))),
            (Sdf.ValueTypeNames.Color3f, json.dumps(rand((3,), "float"))),
            (Sdf.ValueTypeNames.Color3fArray, json.dumps(rand((3,), "float", True))),
            (Sdf.ValueTypeNames.Color3h, json.dumps(rand((3,), "half"))),
            (Sdf.ValueTypeNames.Color3hArray, json.dumps(rand((3,), "half", True))),
            (Sdf.ValueTypeNames.Color4d, json.dumps(rand((4,), "double"))),
            (Sdf.ValueTypeNames.Color4dArray, json.dumps(rand((4,), "double", True))),
            (Sdf.ValueTypeNames.Color4f, json.dumps(rand((4,), "float"))),
            (Sdf.ValueTypeNames.Color4fArray, json.dumps(rand((4,), "float", True))),
            (Sdf.ValueTypeNames.Color4h, json.dumps(rand((4,), "half"))),
            (Sdf.ValueTypeNames.Color4hArray, json.dumps(rand((4,), "half", True))),
            (Sdf.ValueTypeNames.Double, json.dumps(rand((1,), "double"))),
            (Sdf.ValueTypeNames.Double2, json.dumps(rand((2,), "double"))),
            (Sdf.ValueTypeNames.Double2Array, json.dumps(rand((2,), "double", True))),
            (Sdf.ValueTypeNames.Double3, json.dumps(rand((3,), "double"))),
            (Sdf.ValueTypeNames.Double3Array, json.dumps(rand((3,), "double", True))),
            (Sdf.ValueTypeNames.Double4, json.dumps(rand((4,), "double"))),
            (Sdf.ValueTypeNames.Double4Array, json.dumps(rand((4,), "double", True))),
            (Sdf.ValueTypeNames.DoubleArray, json.dumps(rand((1,), "double", True))),
            (Sdf.ValueTypeNames.Float, json.dumps(rand((1,), "float"))),
            (Sdf.ValueTypeNames.Float2, json.dumps(rand((2,), "float"))),
            (Sdf.ValueTypeNames.Float2Array, json.dumps(rand((2,), "float", True))),
            (Sdf.ValueTypeNames.Float3, json.dumps(rand((3,), "float"))),
            (Sdf.ValueTypeNames.Float3Array, json.dumps(rand((3,), "float", True))),
            (Sdf.ValueTypeNames.Float4, json.dumps(rand((4,), "float"))),
            (Sdf.ValueTypeNames.Float4Array, json.dumps(rand((4,), "float", True))),
            (Sdf.ValueTypeNames.FloatArray, json.dumps(rand((1,), "float", True))),
            (Sdf.ValueTypeNames.Frame4d, json.dumps(rand((4, 4), "double"))),
            (Sdf.ValueTypeNames.Frame4dArray, json.dumps(rand((4, 4), "double", True))),
            (Sdf.ValueTypeNames.Half, json.dumps(rand((1,), "half"))),
            (Sdf.ValueTypeNames.Half2, json.dumps(rand((2,), "half"))),
            (Sdf.ValueTypeNames.Half2Array, json.dumps(rand((2,), "half", True))),
            (Sdf.ValueTypeNames.Half3, json.dumps(rand((3,), "half"))),
            (Sdf.ValueTypeNames.Half3Array, json.dumps(rand((3,), "half", True))),
            (Sdf.ValueTypeNames.Half4, json.dumps(rand((4,), "half"))),
            (Sdf.ValueTypeNames.Half4Array, json.dumps(rand((4,), "half", True))),
            (Sdf.ValueTypeNames.HalfArray, json.dumps(rand((1,), "half", True))),
            (Sdf.ValueTypeNames.Int, json.dumps(rand((1,), "int"))),
            (Sdf.ValueTypeNames.Int2, json.dumps(rand((2,), "int"))),
            (Sdf.ValueTypeNames.Int2Array, json.dumps(rand((2,), "int", True))),
            (Sdf.ValueTypeNames.Int3, json.dumps(rand((3,), "int"))),
            (Sdf.ValueTypeNames.Int3Array, json.dumps(rand((3,), "int", True))),
            (Sdf.ValueTypeNames.Int4, json.dumps(rand((4,), "int"))),
            (Sdf.ValueTypeNames.Int4Array, json.dumps(rand((4,), "int", True))),
            (Sdf.ValueTypeNames.Int64, json.dumps(rand((1,), "int"))),
            (Sdf.ValueTypeNames.Int64Array, json.dumps(rand((1,), "int", True))),
            (Sdf.ValueTypeNames.IntArray, json.dumps(rand((1,), "int", True))),
            (Sdf.ValueTypeNames.Matrix2d, json.dumps(rand((2, 2), "double"))),
            (Sdf.ValueTypeNames.Matrix2dArray, json.dumps(rand((2, 2), "double", True))),
            (Sdf.ValueTypeNames.Matrix3d, json.dumps(rand((3, 3), "double"))),
            (Sdf.ValueTypeNames.Matrix3dArray, json.dumps(rand((3, 3), "double", True))),
            (Sdf.ValueTypeNames.Matrix4d, json.dumps(rand((4, 4), "double"))),
            (Sdf.ValueTypeNames.Matrix4dArray, json.dumps(rand((4, 4), "double", True))),
            (Sdf.ValueTypeNames.Normal3d, json.dumps(rand((3,), "double"))),
            (Sdf.ValueTypeNames.Normal3dArray, json.dumps(rand((3,), "double", True))),
            (Sdf.ValueTypeNames.Normal3f, json.dumps(rand((3,), "float"))),
            (Sdf.ValueTypeNames.Normal3fArray, json.dumps(rand((3,), "float", True))),
            (Sdf.ValueTypeNames.Normal3h, json.dumps(rand((3,), "half"))),
            (Sdf.ValueTypeNames.Normal3hArray, json.dumps(rand((3,), "half", True))),
            (Sdf.ValueTypeNames.Point3d, json.dumps(rand((3,), "double"))),
            (Sdf.ValueTypeNames.Point3dArray, json.dumps(rand((3,), "double", True))),
            (Sdf.ValueTypeNames.Point3f, json.dumps(rand((3,), "float"))),
            (Sdf.ValueTypeNames.Point3fArray, json.dumps(rand((3,), "float", True))),
            (Sdf.ValueTypeNames.Point3h, json.dumps(rand((3,), "half"))),
            (Sdf.ValueTypeNames.Point3hArray, json.dumps(rand((3,), "half", True))),
            (Sdf.ValueTypeNames.Quatd, json.dumps(rand((4,), "double"))),
            (Sdf.ValueTypeNames.QuatdArray, json.dumps(rand((4,), "double", True))),
            (Sdf.ValueTypeNames.Quatf, json.dumps(rand((4,), "float"))),
            (Sdf.ValueTypeNames.QuatfArray, json.dumps(rand((4,), "float", True))),
            (Sdf.ValueTypeNames.Quath, json.dumps(rand((4,), "half"))),
            (Sdf.ValueTypeNames.QuathArray, json.dumps(rand((4,), "half", True))),
            (Sdf.ValueTypeNames.String, json.dumps("abcd")),
            (Sdf.ValueTypeNames.StringArray, json.dumps(["abcd", "efgh"])),
            (Sdf.ValueTypeNames.TexCoord2d, json.dumps(rand((2,), "double"))),
            (Sdf.ValueTypeNames.TexCoord2dArray, json.dumps(rand((2,), "double", True))),
            (Sdf.ValueTypeNames.TexCoord2f, json.dumps(rand((2,), "float"))),
            (Sdf.ValueTypeNames.TexCoord2fArray, json.dumps(rand((2,), "float", True))),
            (Sdf.ValueTypeNames.TexCoord2h, json.dumps(rand((2,), "half"))),
            (Sdf.ValueTypeNames.TexCoord2hArray, json.dumps(rand((2,), "half", True))),
            (Sdf.ValueTypeNames.TexCoord3d, json.dumps(rand((3,), "double"))),
            (Sdf.ValueTypeNames.TexCoord3dArray, json.dumps(rand((3,), "double", True))),
            (Sdf.ValueTypeNames.TexCoord3f, json.dumps(rand((3,), "float"))),
            (Sdf.ValueTypeNames.TexCoord3fArray, json.dumps(rand((3,), "float", True))),
            (Sdf.ValueTypeNames.TexCoord3h, json.dumps(rand((3,), "half"))),
            (Sdf.ValueTypeNames.TexCoord3hArray, json.dumps(rand((3,), "half", True))),
            (Sdf.ValueTypeNames.TimeCode, json.dumps(rand((1,), "double"))),
            (Sdf.ValueTypeNames.TimeCodeArray, json.dumps(rand((1,), "double", True))),
            (Sdf.ValueTypeNames.Token, json.dumps("abcde")),
            (Sdf.ValueTypeNames.TokenArray, json.dumps(["abcde", "fghij"])),
            (Sdf.ValueTypeNames.UChar, json.dumps(rand((1,), "uint"))),
            (Sdf.ValueTypeNames.UCharArray, json.dumps(rand((1,), "uint", True))),
            (Sdf.ValueTypeNames.UInt, json.dumps(rand((1,), "uint"))),
            (Sdf.ValueTypeNames.UInt64, json.dumps(rand((1,), "uint"))),
            (Sdf.ValueTypeNames.UInt64Array, json.dumps(rand((1,), "uint", True))),
            (Sdf.ValueTypeNames.UIntArray, json.dumps(rand((1,), "uint", True))),
            (Sdf.ValueTypeNames.Vector3d, json.dumps(rand((3,), "double"))),
            (Sdf.ValueTypeNames.Vector3dArray, json.dumps(rand((3,), "double", True))),
            (Sdf.ValueTypeNames.Vector3f, json.dumps(rand((3,), "float"))),
            (Sdf.ValueTypeNames.Vector3fArray, json.dumps(rand((3,), "float", True))),
            (Sdf.ValueTypeNames.Vector3h, json.dumps(rand((3,), "half"))),
            (Sdf.ValueTypeNames.Vector3hArray, json.dumps(rand((3,), "half", True))),
        ]

        stage = omni.usd.get_context().get_stage()
        prim = stage.DefinePrim(prim_path)
        attributes = []
        for i, spec in enumerate(specs):
            name = f"attr_{i}"
            prim.CreateAttribute(name, spec[0])
            attributes.append((name, spec[1]))
        return attributes

    def checkValues(self, a, b):
        a = json.loads(a)
        b = json.loads(b)
        try:
            self.assertTrue(np.allclose(np.array(a), np.array(b), atol=0.1))
        except TypeError:
            self.assertEqual(a, b)

    # ----------------------------------------------------------------------
    async def test_service_get_prims(self):
        try:
            import isaac_ros2_messages.srv
        except ImportError as e:
            print("Skipping test because the ROS2 isaac_ros2_messages package is not available")
            return

        import rclpy

        # define graph
        (test_graph, new_nodes, _, _) = og.Controller.edit(
            {"graph_path": "/ActionGraph", "evaluator_name": "execution"},
            {
                og.Controller.Keys.CREATE_NODES: [
                    ("OnPlaybackTick", "omni.graph.action.OnPlaybackTick"),
                    ("ServicePrim", "isaacsim.ros2.bridge.ROS2ServicePrim"),
                ],
                og.Controller.Keys.CONNECT: [
                    ("OnPlaybackTick.outputs:tick", "ServicePrim.inputs:execIn"),
                ],
            },
        )

        # node and client
        ros2_node = rclpy.create_node("isaac_sim_test_service")
        client = ros2_node.create_client(isaac_ros2_messages.srv.GetPrims, "/get_prims")

        self._timeline.play()
        await asyncio.sleep(1.0)

        request = isaac_ros2_messages.srv.GetPrims.Request()
        request.path = "/ActionGraph"
        future = client.call_async(request)

        await asyncio.sleep(1.0)
        rclpy.spin_until_future_complete(ros2_node, future, timeout_sec=0)
        result = future.result()

        print("request:", request)
        print("result:", result)

        self.assertIsNotNone(result)
        self.assertTrue(result.success)
        self.assertEqual(result.message, "")
        self.assertIn("/ActionGraph", result.paths)
        self.assertIn("/ActionGraph/OnPlaybackTick", result.paths)
        self.assertIn("/ActionGraph/ServicePrim", result.paths)

    # ----------------------------------------------------------------------
    async def test_service_get_prim_attributes(self):
        try:
            import isaac_ros2_messages.srv
        except ImportError as e:
            print("Skipping test because the ROS2 isaac_ros2_messages package is not available")
            return

        import rclpy

        # define graph
        (test_graph, new_nodes, _, _) = og.Controller.edit(
            {"graph_path": "/ActionGraph", "evaluator_name": "execution"},
            {
                og.Controller.Keys.CREATE_NODES: [
                    ("OnPlaybackTick", "omni.graph.action.OnPlaybackTick"),
                    ("ServicePrim", "isaacsim.ros2.bridge.ROS2ServicePrim"),
                ],
                og.Controller.Keys.CONNECT: [
                    ("OnPlaybackTick.outputs:tick", "ServicePrim.inputs:execIn"),
                ],
            },
        )

        # node and client
        ros2_node = rclpy.create_node("isaac_sim_test_service")
        client = ros2_node.create_client(isaac_ros2_messages.srv.GetPrimAttributes, "/get_prim_attributes")

        self._timeline.play()
        await asyncio.sleep(1.0)

        request = isaac_ros2_messages.srv.GetPrimAttributes.Request()
        request.path = "/ActionGraph"
        future = client.call_async(request)

        await asyncio.sleep(1.0)
        rclpy.spin_until_future_complete(ros2_node, future, timeout_sec=0)
        result = future.result()

        print("request:", request)
        print("result:", result)

        self.assertIsNotNone(result)
        self.assertTrue(result.success)
        self.assertEqual(result.message, "")
        self.assertIn("evaluationMode", result.names)
        self.assertIn("token", result.types)

    # ----------------------------------------------------------------------
    async def test_service_get_set_prim_attribute(self):
        try:
            import isaac_ros2_messages.srv
        except ImportError as e:
            print("Skipping test because the ROS2 isaac_ros2_messages package is not available")
            return

        import rclpy

        # define graph
        (test_graph, new_nodes, _, _) = og.Controller.edit(
            {"graph_path": "/ActionGraph", "evaluator_name": "execution"},
            {
                og.Controller.Keys.CREATE_NODES: [
                    ("OnPlaybackTick", "omni.graph.action.OnPlaybackTick"),
                    ("ServicePrim", "isaacsim.ros2.bridge.ROS2ServicePrim"),
                ],
                og.Controller.Keys.CONNECT: [
                    ("OnPlaybackTick.outputs:tick", "ServicePrim.inputs:execIn"),
                ],
            },
        )

        # create attributes for testing
        prim_path = "/Prim"
        specs = self.createAttributes(prim_path)

        # node and client
        ros2_node = rclpy.create_node("isaac_sim_test_service")
        client_get = ros2_node.create_client(isaac_ros2_messages.srv.GetPrimAttribute, "/get_prim_attribute")
        client_set = ros2_node.create_client(isaac_ros2_messages.srv.SetPrimAttribute, "/set_prim_attribute")

        self._timeline.play()
        await asyncio.sleep(1.0)

        for spec in specs:
            print("---")

            # set the attribute
            request_set = isaac_ros2_messages.srv.SetPrimAttribute.Request()
            request_set.path = prim_path
            request_set.attribute = spec[0]
            request_set.value = spec[1]

            future = client_set.call_async(request_set)

            for _ in range(2):
                await omni.kit.app.get_app().next_update_async()
                rclpy.spin_once(ros2_node, timeout_sec=0)
                rclpy.spin_until_future_complete(ros2_node, future, timeout_sec=0)
            result_set = future.result()

            print("(set) request:", request_set)
            print("(set) result:", result_set)

            # get the attribute
            request_get = isaac_ros2_messages.srv.GetPrimAttribute.Request()
            request_get.path = prim_path
            request_get.attribute = spec[0]

            future = client_get.call_async(request_get)

            for _ in range(2):
                await omni.kit.app.get_app().next_update_async()
                rclpy.spin_once(ros2_node, timeout_sec=0)
                rclpy.spin_until_future_complete(ros2_node, future, timeout_sec=0)
            result_get = future.result()

            print("(get) request:", request_get)
            print("(get) result:", result_get)

            # check
            self.assertIsNotNone(result_set)
            self.assertTrue(result_set.success)
            self.assertEqual(result_set.message, "")

            self.assertIsNotNone(result_get)
            self.assertTrue(result_get.success)
            self.assertEqual(result_get.message, "")
            json.loads(result_get.value)

            self.checkValues(spec[1], result_get.value)
