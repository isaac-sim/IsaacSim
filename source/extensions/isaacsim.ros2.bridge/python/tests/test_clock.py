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


import carb
import omni.graph.core as og

# Import extension python module we are testing with absolute import path, as if we are external user (other extension)
import omni.kit.commands

# NOTE:
#   omni.kit.test - std python's unittest module with additional wrapping to add support for async/await tests
#   For most things refer to unittest docs: https://docs.python.org/3/library/unittest.html
import omni.kit.test
import omni.kit.usd
from isaacsim.core.utils.physics import simulate_async

from .common import ROS2TestCase, get_qos_profile


# Having a test class derived from omni.kit.test.AsyncTestCase declared on the root of module will make it auto-discoverable by omni.kit.test
class TestRos2BridgeCommands(ROS2TestCase):
    # Before running each test
    async def setUp(self):
        await super().setUp()
        await omni.usd.get_context().new_stage_async()
        self._stage = omni.usd.get_context().get_stage()

        await omni.kit.app.get_app().next_update_async()
        self._stage = omni.usd.get_context().get_stage()

        await omni.kit.app.get_app().next_update_async()
        pass

    # After running each test
    async def tearDown(self):

        self._stage = None
        await super().tearDown()

    async def test_sim_clock(self):
        import rclpy
        from rosgraph_msgs.msg import Clock

        keys = og.Controller.Keys
        (graph, nodes, _, _) = og.Controller.edit(
            {"graph_path": "/controller_graph", "evaluator_name": "execution"},
            {
                keys.CREATE_NODES: [
                    ("OnTick", "omni.graph.action.OnTick"),
                    ("IsaacClock", "isaacsim.core.nodes.IsaacReadSimulationTime"),
                    ("RosPublisher", "isaacsim.ros2.bridge.ROS2PublishClock"),
                ],
                keys.CONNECT: [
                    ("OnTick.outputs:tick", "RosPublisher.inputs:execIn"),
                    ("IsaacClock.outputs:simulationTime", "RosPublisher.inputs:timeStamp"),
                ],
            },
        )

        self._time_sec = 0

        def clock_callback(data):
            self._time_sec = data.clock.sec + data.clock.nanosec / 1.0e9

        node = rclpy.create_node("test_sim_clock")
        clock_sub = node.create_subscription(Clock, "clock", clock_callback, get_qos_profile())

        def spin():
            rclpy.spin_once(node, timeout_sec=0.1)

        self._timeline.play()

        await simulate_async(2.1, callback=spin)
        self._timeline.stop()
        self.assertGreater(self._time_sec, 2.0)
        spin()
        pass

    async def test_sim_clock_manual(self):
        import rclpy
        from rosgraph_msgs.msg import Clock

        keys = og.Controller.Keys
        (graph, nodes, _, _) = og.Controller.edit(
            {"graph_path": "/controller_graph", "evaluator_name": "execution"},
            {
                keys.CREATE_NODES: [
                    ("Impulse", "omni.graph.action.OnImpulseEvent"),
                    ("IsaacClock", "isaacsim.core.nodes.IsaacReadSimulationTime"),
                    ("RosPublisher", "isaacsim.ros2.bridge.ROS2PublishClock"),
                ],
                keys.SET_VALUES: [("IsaacClock.inputs:resetOnStop", True)],
                keys.CONNECT: [
                    ("Impulse.outputs:execOut", "RosPublisher.inputs:execIn"),
                    ("IsaacClock.outputs:simulationTime", "RosPublisher.inputs:timeStamp"),
                ],
            },
        )

        self._time_sec = 0

        def clock_callback(data):
            self._time_sec = data.clock.sec + data.clock.nanosec / 1.0e9

        node = rclpy.create_node("test_sim_clock")
        clock_sub = node.create_subscription(Clock, "clock", clock_callback, get_qos_profile())

        def spin():
            rclpy.spin_once(node, timeout_sec=0.1)

        await simulate_async(0.1, callback=spin)
        self._timeline.play()

        await omni.kit.app.get_app().next_update_async()
        self.assertEqual(self._time_sec, 0.0)
        og.Controller.attribute("/controller_graph/Impulse.state:enableImpulse").set(True)
        # after first step we need to wait for ros node to initialize
        await simulate_async(0.1, callback=spin)

        og.Controller.attribute("/controller_graph/Impulse.state:enableImpulse").set(True)
        # wait for message
        await simulate_async(0.1, callback=spin)
        self.assertGreater(self._time_sec, 0.0)

        self._timeline.stop()
        spin()
        pass

    async def test_system_clock(self):
        import time

        import rclpy
        from rosgraph_msgs.msg import Clock

        keys = og.Controller.Keys
        (graph, nodes, _, _) = og.Controller.edit(
            {"graph_path": "/controller_graph", "evaluator_name": "execution"},
            {
                keys.CREATE_NODES: [
                    ("OnTick", "omni.graph.action.OnTick"),
                    ("IsaacClock", "isaacsim.core.nodes.IsaacReadSystemTime"),
                    ("RosPublisher", "isaacsim.ros2.bridge.ROS2PublishClock"),
                ],
                keys.CONNECT: [
                    ("OnTick.outputs:tick", "RosPublisher.inputs:execIn"),
                    ("IsaacClock.outputs:systemTime", "RosPublisher.inputs:timeStamp"),
                ],
            },
        )
        self._time_sec = 0

        def clock_callback(data):
            self._time_sec = data.clock.sec + data.clock.nanosec / 1.0e9

        node = rclpy.create_node("test_sim_clock")
        clock_sub = node.create_subscription(Clock, "clock", clock_callback, get_qos_profile())

        def spin():
            rclpy.spin_once(node, timeout_sec=0.1)

        self._timeline.play()

        await simulate_async(0.1, callback=spin)
        self.assertAlmostEqual(self._time_sec, time.time(), delta=0.5)
        self._timeline.stop()
        spin()
        pass
