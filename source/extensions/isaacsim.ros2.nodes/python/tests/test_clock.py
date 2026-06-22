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

"""Verifies ROS 2 clock publishing for simulation time, physics-step updates, manual clock control, and system time."""

from typing import Any

import omni.graph.core as og

# Import extension python module we are testing with absolute import path, as if we are external user (other extension)
import omni.kit.commands

# NOTE:
#   omni.kit.test - std python's unittest module with additional wrapping to add support for async/await tests
#   For most things refer to unittest docs: https://docs.python.org/3/library/unittest.html
import omni.kit.test
import omni.kit.usd
from isaacsim.ros2.core.impl.ros2_test_case import ROS2TestCase

from .common import get_qos_profile, simulate_async


class TestRos2NodeCommands(ROS2TestCase):
    """Verify ROS 2 clock publisher node command graphs."""

    async def setUp(self) -> None:
        """Cache the current USD stage for clock graph construction."""
        await super().setUp()
        self._stage = omni.usd.get_context().get_stage()

    async def tearDown(self) -> None:
        """Drop the cached stage after clock publishing tests."""
        self._stage = None
        await super().tearDown()

    async def test_sim_clock(self) -> None:
        """Test sim clock."""
        import rclpy
        from rosgraph_msgs.msg import Clock

        keys = og.Controller.Keys
        graph, nodes, _, _ = og.Controller.edit(
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

        def clock_callback(data: Any) -> None:
            self._time_sec = data.clock.sec + data.clock.nanosec / 1.0e9

        node = self.create_node("test_sim_clock")
        clock_sub = self.create_subscription(node, Clock, "clock", clock_callback, get_qos_profile())

        def spin() -> None:
            rclpy.spin_once(node, timeout_sec=0.01)

        self._timeline.play()

        await self.simulate_until_condition(lambda: self._time_sec > 2.0, max_frames=150, per_frame_callback=spin)
        self._timeline.stop()
        self.assertGreater(self._time_sec, 2.0)
        spin()

    async def test_sim_clock_physics_step(self) -> None:
        """Test sim clock published from an OnPhysicsStep-driven on-demand graph."""
        import rclpy
        from rosgraph_msgs.msg import Clock

        keys = og.Controller.Keys
        graph, nodes, _, _ = og.Controller.edit(
            {
                "graph_path": "/physics_step_clock_graph",
                "pipeline_stage": og.GraphPipelineStage.GRAPH_PIPELINE_STAGE_ONDEMAND,
            },
            {
                keys.CREATE_NODES: [
                    ("OnPhysicsStep", "isaacsim.core.nodes.OnPhysicsStep"),
                    ("IsaacClock", "isaacsim.core.nodes.IsaacReadSimulationTime"),
                    ("RosPublisher", "isaacsim.ros2.bridge.ROS2PublishClock"),
                ],
                keys.CONNECT: [
                    ("OnPhysicsStep.outputs:step", "RosPublisher.inputs:execIn"),
                    ("IsaacClock.outputs:simulationTime", "RosPublisher.inputs:timeStamp"),
                ],
            },
        )

        self._time_sec = 0

        def clock_callback(data: Any) -> None:
            self._time_sec = data.clock.sec + data.clock.nanosec / 1.0e9

        node = self.create_node("test_sim_clock_physics_step")
        clock_sub = self.create_subscription(node, Clock, "clock", clock_callback, get_qos_profile())

        def spin() -> None:
            rclpy.spin_once(node, timeout_sec=0.01)

        self._timeline.play()

        await simulate_async(2.1, callback=spin)
        self._timeline.stop()
        self.assertGreater(self._time_sec, 2.0)
        spin()

    async def test_sim_clock_manual(self) -> None:
        """Test sim clock manual."""
        import rclpy
        from rosgraph_msgs.msg import Clock

        keys = og.Controller.Keys
        graph, nodes, _, _ = og.Controller.edit(
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

        def clock_callback(data: Any) -> None:
            self._time_sec = data.clock.sec + data.clock.nanosec / 1.0e9

        node = self.create_node("test_sim_clock")
        clock_sub = self.create_subscription(node, Clock, "clock", clock_callback, get_qos_profile())

        def spin() -> None:
            rclpy.spin_once(node, timeout_sec=0.01)

        await self.simulate_until_condition(lambda: False, max_frames=10, per_frame_callback=spin)
        self._timeline.play()

        await omni.kit.app.get_app().next_update_async()
        self.assertEqual(self._time_sec, 0.0)
        og.Controller.attribute("/controller_graph/Impulse.state:enableImpulse").set(True)
        # after first step we need to wait for ros node to initialize
        await self.simulate_until_condition(lambda: False, max_frames=10, per_frame_callback=spin)

        og.Controller.attribute("/controller_graph/Impulse.state:enableImpulse").set(True)
        # wait for message
        await self.simulate_until_condition(lambda: self._time_sec > 0.0, max_frames=30, per_frame_callback=spin)
        self.assertGreater(self._time_sec, 0.0)

        self._timeline.stop()
        spin()

    async def test_system_clock(self) -> None:
        """Test system clock."""
        import time

        import rclpy
        from rosgraph_msgs.msg import Clock

        keys = og.Controller.Keys
        graph, nodes, _, _ = og.Controller.edit(
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

        def clock_callback(data: Any) -> None:
            self._time_sec = data.clock.sec + data.clock.nanosec / 1.0e9

        node = self.create_node("test_sim_clock")
        clock_sub = self.create_subscription(node, Clock, "clock", clock_callback, get_qos_profile())

        def spin() -> None:
            rclpy.spin_once(node, timeout_sec=0.1)

        self._timeline.play()

        await self.simulate_until_condition(lambda: self._time_sec > 0, max_frames=30, per_frame_callback=spin)
        self.assertAlmostEqual(self._time_sec, time.time(), delta=0.5)
        self._timeline.stop()
        spin()
