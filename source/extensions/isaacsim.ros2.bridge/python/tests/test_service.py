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

import omni.graph.core as og
import omni.kit.test
from isaacsim.core.utils.stage import create_new_stage_async

from .common import ROS2TestCase


class TestRos2Service(ROS2TestCase):
    # Before running each test
    async def setUp(self):
        await super().setUp()
        await create_new_stage_async()

    # After running each test
    async def tearDown(self):
        await super().tearDown()

    # ----------------------------------------------------------------------
    async def test_service(self):
        import builtin_interfaces.msg
        import rclpy

        self._timeline.play()
        await omni.kit.app.get_app().next_update_async()

        # define graph
        (test_graph, new_nodes, _, _) = og.Controller.edit(
            {"graph_path": "/ActionGraph", "evaluator_name": "execution"},
            {
                og.Controller.Keys.CREATE_NODES: [
                    ("OnPlaybackTick", "omni.graph.action.OnPlaybackTick"),
                    ("ServerRequest", "isaacsim.ros2.bridge.OgnROS2ServiceServerRequest"),
                    ("ServerResponse", "isaacsim.ros2.bridge.OgnROS2ServiceServerResponse"),
                    ("Client", "isaacsim.ros2.bridge.OgnROS2ServiceClient"),
                ],
                og.Controller.Keys.SET_VALUES: [
                    ("ServerRequest.inputs:serviceName", "/custom_service"),
                    ("ServerRequest.inputs:messagePackage", "example_interfaces"),
                    ("ServerRequest.inputs:messageSubfolder", "srv"),
                    ("ServerRequest.inputs:messageName", "AddTwoInts"),
                    ("ServerResponse.inputs:messagePackage", "example_interfaces"),
                    ("ServerResponse.inputs:messageSubfolder", "srv"),
                    ("ServerResponse.inputs:messageName", "AddTwoInts"),
                    ("Client.inputs:serviceName", "/custom_service"),
                    ("Client.inputs:messagePackage", "example_interfaces"),
                    ("Client.inputs:messageSubfolder", "srv"),
                    ("Client.inputs:messageName", "AddTwoInts"),
                ],
                og.Controller.Keys.CONNECT: [
                    ("OnPlaybackTick.outputs:tick", "Client.inputs:execIn"),
                    ("OnPlaybackTick.outputs:tick", "ServerRequest.inputs:execIn"),
                    ("ServerRequest.outputs:onReceived", "ServerResponse.inputs:onReceived"),
                    ("ServerRequest.outputs:serverHandle", "ServerResponse.inputs:serverHandle"),
                ],
            },
        )

        await og.Controller.evaluate(test_graph)
        await omni.kit.app.get_app().next_update_async()
        server_req_node = new_nodes[1]
        server_res_node = new_nodes[2]
        client_node = new_nodes[3]

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
