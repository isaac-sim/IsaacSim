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

"""Tests for ROS 2 service OmniGraph nodes."""

import importlib

import omni.graph.core as og
import omni.kit.test
from isaacsim.core.experimental.utils import stage as stage_utils
from isaacsim.ros2.core.impl.ros2_test_case import ROS2TestCase

# Each test case exercises writeNodeAttributeFromMessage for specific primitive types
# by creating a service client/server pair and verifying that request fields are correctly
# written to the server's output attributes using the full prefixed path.
#
# Types covered:
#   eBool   - std_srvs/SetBool          (data: bool)
#   eInt    - nav_msgs/GetPlan           (start:header:stamp:sec: int32)
#   eUInt   - nav_msgs/GetPlan           (start:header:stamp:nanosec: uint32)
#   eInt64  - example_interfaces/AddTwoInts (a, b: int64)
#   eUInt64 - rcl_interfaces/ListParameters (depth: uint64)
#   eFloat  - nav_msgs/GetPlan           (tolerance: float32)
#   eDouble - nav_msgs/GetPlan           (start:pose:position:x: float64)
#   eToken  - nav_msgs/GetPlan           (start:header:frame_id: string)
#
# Not covered (no standard service exposes these as flat request fields):
#   eUChar   - uint8 scalars/arrays
#   eUnknown - nested message arrays (requires response-direction testing)
SERVICE_FIELD_TYPE_CASES = [
    # (label, package, subfolder, message, fields_to_test)
    # fields_to_test: list of (field_path, test_value, is_float)
    (
        "AddTwoInts",
        "example_interfaces",
        "srv",
        "AddTwoInts",
        [
            ("a", 42, False),
            ("b", -7, False),
        ],
    ),
    (
        "SetBool",
        "std_srvs",
        "srv",
        "SetBool",
        [
            ("data", True, False),
        ],
    ),
    (
        "GetPlan",
        "nav_msgs",
        "srv",
        "GetPlan",
        [
            ("tolerance", 3.14, True),
            ("start:header:stamp:sec", 42, False),
            ("start:header:stamp:nanosec", 123456, False),
            ("start:header:frame_id", "test_frame", False),
            ("start:pose:position:x", 1.5, True),
            ("start:pose:position:y", -2.7, True),
        ],
    ),
    (
        "ListParameters",
        "rcl_interfaces",
        "srv",
        "ListParameters",
        [
            ("depth", 5, False),
        ],
    ),
]


class TestRos2Service(ROS2TestCase):
    """Test suite for ros2 service."""

    async def setUp(self):
        """Set up test fixtures."""
        await super().setUp()
        await stage_utils.create_new_stage_async()

    async def tearDown(self):
        """Tear down test fixtures."""
        await super().tearDown()

    def _create_service_graph(self, graph_path, service_name, package, subfolder, message):
        """Create a service client/server action graph and return (graph, server_req_node, client_node)."""
        test_graph, new_nodes, _, _ = og.Controller.edit(
            {"graph_path": graph_path, "evaluator_name": "execution"},
            {
                og.Controller.Keys.CREATE_NODES: [
                    ("OnPlaybackTick", "omni.graph.action.OnPlaybackTick"),
                    ("ServerRequest", "isaacsim.ros2.bridge.OgnROS2ServiceServerRequest"),
                    ("ServerResponse", "isaacsim.ros2.bridge.OgnROS2ServiceServerResponse"),
                    ("Client", "isaacsim.ros2.bridge.OgnROS2ServiceClient"),
                ],
                og.Controller.Keys.SET_VALUES: [
                    ("ServerRequest.inputs:serviceName", service_name),
                    ("ServerRequest.inputs:messagePackage", package),
                    ("ServerRequest.inputs:messageSubfolder", subfolder),
                    ("ServerRequest.inputs:messageName", message),
                    ("ServerResponse.inputs:messagePackage", package),
                    ("ServerResponse.inputs:messageSubfolder", subfolder),
                    ("ServerResponse.inputs:messageName", message),
                    ("Client.inputs:serviceName", service_name),
                    ("Client.inputs:messagePackage", package),
                    ("Client.inputs:messageSubfolder", subfolder),
                    ("Client.inputs:messageName", message),
                ],
                og.Controller.Keys.CONNECT: [
                    ("OnPlaybackTick.outputs:tick", "Client.inputs:execIn"),
                    ("OnPlaybackTick.outputs:tick", "ServerRequest.inputs:execIn"),
                    ("ServerRequest.outputs:onReceived", "ServerResponse.inputs:onReceived"),
                    ("ServerRequest.outputs:serverHandle", "ServerResponse.inputs:serverHandle"),
                ],
            },
        )
        server_req_node = new_nodes[1]
        server_res_node = new_nodes[2]
        client_node = new_nodes[3]
        return test_graph, server_req_node, server_res_node, client_node

    # ----------------------------------------------------------------------
    async def test_service(self):
        """Test service."""
        self._timeline.play()
        await omni.kit.app.get_app().next_update_async()

        test_graph, server_req_node, server_res_node, client_node = self._create_service_graph(
            "/ActionGraph", "/custom_service", "example_interfaces", "srv", "AddTwoInts"
        )

        await og.Controller.evaluate(test_graph)
        await omni.kit.app.get_app().next_update_async()

        og.Controller.attribute("inputs:Request:a", client_node).set(11)
        og.Controller.attribute("inputs:Request:b", client_node).set(10)

        # wait for the client to executes and send the request
        await omni.kit.app.get_app().next_update_async()
        await og.Controller.evaluate(test_graph)
        a = og.Controller.attribute("outputs:Request:a", server_req_node).get()
        b = og.Controller.attribute("outputs:Request:b", server_req_node).get()

        await omni.kit.app.get_app().next_update_async()
        server_result = a + b
        og.Controller.attribute("inputs:Response:sum", server_res_node).set(server_result)

        # wait for the server to executes and send the response
        await omni.kit.app.get_app().next_update_async()
        await omni.kit.app.get_app().next_update_async()

        client_result = og.Controller.attribute("outputs:Response:sum", client_node).get()
        print("server response = ", server_result)
        print("client response = ", client_result)
        self.assertEqual(client_result, 21)
        self._timeline.stop()

    # ----------------------------------------------------------------------
    async def test_service_field_types(self):
        """Verify writeNodeAttributeFromMessage correctly prefixes attribute paths for all types.

        Uses multiple ROS 2 service types whose request fields collectively exercise every
        primitive data type in the writeNodeAttributeFromMessage switch statement.
        For each service, sets values on the client's request inputs, evaluates the graph,
        and checks that the server's request outputs received the correct values.
        """
        self._timeline.play()
        await omni.kit.app.get_app().next_update_async()

        ran_any = False
        for idx, (label, package, subfolder, message, fields) in enumerate(SERVICE_FIELD_TYPE_CASES):
            try:
                importlib.import_module(f"{package}.{subfolder}")
            except (ImportError, ModuleNotFoundError):
                print(f"Skipping {label}: {package}.{subfolder} not available")
                continue
            ran_any = True

            service_name = f"/test_field_types_{idx}"
            graph_path = f"/ActionGraph_types_{idx}"

            test_graph, server_req_node, _, client_node = self._create_service_graph(
                graph_path, service_name, package, subfolder, message
            )

            await og.Controller.evaluate(test_graph)
            await omni.kit.app.get_app().next_update_async()

            for field_path, value, _ in fields:
                og.Controller.attribute(f"inputs:Request:{field_path}", client_node).set(value)

            await omni.kit.app.get_app().next_update_async()
            await og.Controller.evaluate(test_graph)

            for field_path, expected, is_float in fields:
                actual = og.Controller.attribute(f"outputs:Request:{field_path}", server_req_node).get()
                if is_float:
                    self.assertAlmostEqual(
                        actual,
                        expected,
                        places=2,
                        msg=f"[{label}] field '{field_path}': expected {expected}, got {actual}",
                    )
                elif isinstance(expected, bool):
                    self.assertEqual(
                        actual,
                        expected,
                        msg=f"[{label}] field '{field_path}': expected {expected}, got {actual}",
                    )
                else:
                    self.assertEqual(
                        actual,
                        expected,
                        msg=f"[{label}] field '{field_path}': expected {expected}, got {actual}",
                    )

        self.assertTrue(ran_any, "No ROS 2 service packages available")
        self._timeline.stop()
