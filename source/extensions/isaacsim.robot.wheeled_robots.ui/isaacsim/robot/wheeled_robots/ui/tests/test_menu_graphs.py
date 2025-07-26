# SPDX-FileCopyrightText: Copyright (c) 2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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

import omni.graph.core as og
import omni.kit.test
import omni.kit.ui_test as ui_test
import omni.timeline
import omni.usd
from isaacsim.core.api import SimulationContext
from isaacsim.core.utils.physics import simulate_async
from isaacsim.core.utils.stage import update_stage_async
from isaacsim.storage.native import get_assets_root_path_async
from omni.kit.ui_test.menu import *
from omni.kit.ui_test.query import *
from omni.ui.tests.test_base import OmniUiTest


class TestDifferentialRobotGraph(OmniUiTest):
    async def setUp(self):
        await omni.usd.get_context().new_stage_async()
        await omni.kit.app.get_app().next_update_async()
        self._stage = omni.usd.get_context().get_stage()
        self._timeline = omni.timeline.get_timeline_interface()
        self._simulation_context = SimulationContext()
        await self._simulation_context.initialize_simulation_context_async()

        # add robot to stage (Nova Carter)
        self._robot_path = "/World/test_robot"
        robot_prim = self._stage.DefinePrim(self._robot_path, "Xform")
        assets_root_path = await get_assets_root_path_async()
        if assets_root_path is None:
            carb.log_error("Could not find Isaac Sim assets folder")
            return
        robot_prim.GetReferences().AddReference(assets_root_path + "/Isaac/Robots/NVIDIA/Jetbot/jetbot.usd")
        await update_stage_async()

        while omni.usd.get_context().get_stage_loading_status()[2] > 0:
            print("Loading...")
            await asyncio.sleep(1.0)
            await omni.kit.app.get_app().next_update_async()

    # After running each test
    async def tearDown(self):
        self._timeline.stop()
        while omni.usd.get_context().get_stage_loading_status()[2] > 0:
            print("tearDown, assets still loading, waiting to finish...")
            await asyncio.sleep(1.0)
            await omni.kit.app.get_app().next_update_async()
        pass

    async def test_basic_graph_creation(self):
        """Test creation of basic differential drive graph structure"""
        # Click through the menu to create the graph
        await menu_click("Tools/Robotics/OmniGraph Controllers/Differential Controller")
        await omni.kit.app.get_app().next_update_async()

        # Wait for and interact with parameter window
        # Find parameter window
        window_name = "Differential Controller"
        param_window = ui_test.find(window_name)
        self.assertIsNotNone(param_window, "Parameter window not found")

        # Find and set the graph root prim
        graph_test_path = "/World/test_graph"
        root_widget_path = f"{window_name}//Frame/VStack[0]"
        graph_root_prim = ui_test.find(root_widget_path + "/HStack[1]/StringField[0]")
        graph_root_prim.model.set_value(graph_test_path)

        # add robot prim to graph
        robot_prim = ui_test.find(root_widget_path + "/HStack[2]/StringField[0]")
        robot_prim.model.set_value(self._robot_path)

        # Find and set wheel radius parameter
        wheel_radius = ui_test.find(root_widget_path + "/HStack[3]/FloatField[0]")
        wheel_radius.model.set_value("0.0325")

        # Find and set wheel distance parameter
        wheel_distance = ui_test.find(root_widget_path + "/HStack[4]/FloatField[0]")
        wheel_distance.model.set_value("0.118")

        # Find and set joint names, right and left
        right_joint_name = ui_test.find(root_widget_path + "/VStack[0]/HStack[0]/StringField[0]")
        right_joint_name.model.set_value("right_wheel_joint")
        left_joint_name = ui_test.find(root_widget_path + "/VStack[0]/HStack[1]/StringField[0]")
        left_joint_name.model.set_value("left_wheel_joint")

        await omni.kit.app.get_app().next_update_async()

        # Click OK button
        ok_button = ui_test.find(root_widget_path + "/HStack[6]/Button[0]")
        self.assertIsNotNone(ok_button, "OK button not found")
        await ok_button.click()

        await omni.kit.app.get_app().next_update_async()

        # Get the created graph at default path
        graph = og.get_graph_by_path(graph_test_path)
        self.assertIsNotNone(graph, "Graph was not created")

        # Verify essential nodes exist
        nodes = graph.get_nodes()
        node_types = {node.get_type_name() for node in nodes}

        expected_nodes = {
            "omni.graph.action.OnPlaybackTick",
            "isaacsim.robot.wheeled_robots.DifferentialController",
            "isaacsim.core.nodes.IsaacArticulationController",
            "omni.graph.nodes.ConstructArray",
        }

        for expected in expected_nodes:
            self.assertIn(expected, node_types, f"Missing expected node type: {expected}")

        # Add check for unexpected nodes
        self.assertEqual(
            expected_nodes, node_types, f"Found unexpected node types. Expected: {expected_nodes}, Got: {node_types}"
        )

    async def test_add_to_existing_graph(self):
        """Test adding differential drive nodes to an existing graph"""
        # First create a base graph with just a tick node
        graph_path = "/World/test_graph"
        graph = og.Controller.create_graph({"graph_path": graph_path, "evaluator_name": "execution"})
        og.Controller.create_node(graph_path + "/OnPlaybackTick", "omni.graph.action.OnPlaybackTick")
        # Open UI and set to add to existing graph

        # Create graph. Check for parameter window before continuing.
        # If the parameter window is not found, retry with increasing delays
        window_name = "Differential Controller"
        delays = [5, 50, 100]
        for delay in delays:
            try:
                await menu_click(
                    "Tools/Robotics/OmniGraph Controllers/Differential Controller", human_delay_speed=delay
                )
                if (param_window := ui_test.find(window_name)) is not None:
                    break

            except AttributeError as e:
                if "NoneType' object has no attribute 'center'" in str(e) and delay != delays[-1]:
                    continue
                raise
        for _ in range(10):
            await update_stage_async()

        # Find and interact with parameter window
        self.assertIsNotNone(param_window, "Parameter window not found")

        # Find and check the "Add to Existing Graph" checkbox
        root_widget_path = f"{window_name}//Frame/VStack[0]"
        add_to_graph_checkbox = ui_test.find(root_widget_path + "/HStack[0]/HStack[0]/VStack[0]/ToolButton[0]")
        self.assertIsNotNone(add_to_graph_checkbox, "Add to existing graph checkbox not found")
        await add_to_graph_checkbox.click()
        for _ in range(10):
            await update_stage_async()
        # Set the existing graph path
        graph_root_prim = ui_test.find(root_widget_path + "/HStack[1]/StringField[0]")
        self.assertIsNotNone(graph_root_prim, "Graph root prim not found")
        graph_root_prim.model.set_value(graph_path)

        # add robot prim to graph
        robot_prim = ui_test.find(root_widget_path + "/HStack[2]/StringField[0]")
        robot_prim.model.set_value(self._robot_path)

        # Find and set wheel radius parameter
        wheel_radius = ui_test.find(root_widget_path + "/HStack[3]/FloatField[0]")
        wheel_radius.model.set_value("0.0325")

        # Find and set wheel distance parameter
        wheel_distance = ui_test.find(root_widget_path + "/HStack[4]/FloatField[0]")
        wheel_distance.model.set_value("0.118")

        for _ in range(10):
            await update_stage_async()

        # Click OK button
        ok_button = ui_test.find(root_widget_path + "/HStack[6]/Button[0]")
        self.assertIsNotNone(ok_button, "OK button not found")
        await ok_button.click()

        for _ in range(10):
            await update_stage_async()

        # Verify nodes were added to existing graph
        graph = og.get_graph_by_path(graph_path)
        self.assertIsNotNone(graph, "Graph not found")

        nodes = graph.get_nodes()
        node_types = {node.get_type_name() for node in nodes}

        expected_nodes = {
            "omni.graph.action.OnPlaybackTick",
            "isaacsim.robot.wheeled_robots.DifferentialController",
            "isaacsim.core.nodes.IsaacArticulationController",
        }

        for expected in expected_nodes:
            self.assertIn(expected, node_types, f"Missing expected node type: {expected}")

        # Add check for unexpected nodes
        self.assertEqual(
            expected_nodes, node_types, f"Found unexpected node types. Expected: {expected_nodes}, Got: {node_types}"
        )

        # Verify we have exactly one tick node and it's connected
        tick_nodes = [n for n in nodes if n.get_type_name() == "omni.graph.action.OnPlaybackTick"]
        self.assertEqual(len(tick_nodes), 1, "Should only have one tick node")

        # Check tick node is connected to new node
        tick_node = tick_nodes[0]
        attr = og.ObjectLookup.attribute(("outputs:tick", tick_node))
        output_ports = attr.get_downstream_connections()
        self.assertTrue(len(output_ports) > 0, "Tick node is not connected to new nodes")

    async def test_keyboard_control(self):
        """Test creation of basic differential drive graph structure"""
        # Click through the menu to create the graph
        await menu_click("Tools/Robotics/OmniGraph Controllers/Differential Controller")
        await omni.kit.app.get_app().next_update_async()

        # Wait for and interact with parameter window
        # Find parameter window
        window_name = "Differential Controller"
        param_window = ui_test.find(window_name)
        self.assertIsNotNone(param_window, "Parameter window not found")

        # Find and set the graph root prim
        root_widget_path = f"{window_name}//Frame/VStack[0]"

        # add robot prim to graph
        robot_prim = ui_test.find(root_widget_path + "/HStack[2]/StringField[0]")
        robot_prim.model.set_value(self._robot_path)

        # Find and set wheel radius parameter
        wheel_radius = ui_test.find(root_widget_path + "/HStack[3]/FloatField[0]")
        wheel_radius.model.set_value("0.0325")

        # Find and set wheel distance parameter
        wheel_distance = ui_test.find(root_widget_path + "/HStack[4]/FloatField[0]")
        wheel_distance.model.set_value("0.118")

        # Enable keyboard control
        keyboard_checkbox = ui_test.find(root_widget_path + "/HStack[5]/HStack[0]/VStack[0]/ToolButton[0]")
        keyboard_checkbox.model.set_value(True)

        await omni.kit.app.get_app().next_update_async()

        # Click OK button
        ok_button = ui_test.find(root_widget_path + "/HStack[6]/Button[0]")
        self.assertIsNotNone(ok_button, "OK button not found")
        await ok_button.click()

        await omni.kit.app.get_app().next_update_async()

        # Test presence of keyboard nodes
        graph = og.get_graph_by_path("/Graphs/differential_controller")
        self.assertIsNotNone(graph, "Graph was not created")

        nodes = graph.get_nodes()
        node_types = {node.get_type_name() for node in nodes}

        keyboard_nodes = {
            "omni.graph.nodes.ReadKeyboardState",
            "omni.graph.nodes.ToDouble",
            "omni.graph.nodes.Multiply",
            "omni.graph.nodes.Add",
            "omni.graph.nodes.ConstantDouble",
            "omni.graph.nodes.ConstantInt",
        }

        for node_type in keyboard_nodes:
            self.assertIn(node_type, node_types, f"Missing keyboard control node: {node_type}")

        # Find key nodes and differential controller
        w_key = None
        s_key = None
        a_key = None
        d_key = None
        diff_node = None

        for node in nodes:
            if node.get_type_name() == "omni.graph.nodes.ReadKeyboardState":
                key_value = og.ObjectLookup.attribute("inputs:key", node).get()
                if key_value == "W":
                    w_key = node
                elif key_value == "S":
                    s_key = node
                elif key_value == "A":
                    a_key = node
                elif key_value == "D":
                    d_key = node
            elif node.get_type_name() == "isaacsim.robot.wheeled_robots.DifferentialController":
                diff_node = node

        self.assertIsNotNone(w_key, "W key node not found")
        self.assertIsNotNone(s_key, "S key node not found")
        self.assertIsNotNone(a_key, "A key node not found")
        self.assertIsNotNone(d_key, "D key node not found")
        self.assertIsNotNone(diff_node, "Differential controller node not found")

        # Test keyboard scaling parameters
        scale_linear = None
        scale_angular = None

        for node in nodes:
            if node.get_type_name() == "omni.graph.nodes.ConstantDouble":
                value = og.ObjectLookup.attribute("inputs:value", node).get()
                if value == 5.0:
                    scale_linear = node
                elif value == 6.0:
                    scale_angular = node

        self.assertIsNotNone(scale_linear, "Linear velocity scaling node not found")
        self.assertIsNotNone(scale_angular, "Angular velocity scaling node not found")

    async def test_differential_drive_golden(self):
        """Test differential drive computation with golden values"""

        # add a ground plane
        import numpy as np
        from isaacsim.core.prims import Articulation
        from omni.physx.scripts.physicsUtils import add_ground_plane
        from pxr import Gf

        # add a ground plane
        add_ground_plane(self._stage, "/World/groundPlane", "Z", 100, Gf.Vec3f(0, 0, 0), Gf.Vec3f(1.0))

        # check the robot is at the origin
        robot = Articulation(prim_paths_expr=self._robot_path, name="test_robot")

        # start the simulation so physics is enabled
        self._timeline.play()
        robot_position = robot.get_world_poses()[0]
        self.assertTrue((robot_position == np.array([0, 0, 0])).all())
        self._timeline.stop()

        # Create a differential drive graph using the menu
        await menu_click("Tools/Robotics/OmniGraph Controllers/Differential Controller")
        await omni.kit.app.get_app().next_update_async()
        # Wait for and interact with parameter window
        # Find parameter window
        window_name = "Differential Controller"
        param_window = ui_test.find(window_name)
        self.assertIsNotNone(param_window, "Parameter window not found")

        # Find and set the graph root prim
        root_widget_path = f"{window_name}//Frame/VStack[0]"
        graph_path = "/Graphs/differential_controller"

        # add robot prim to graph
        robot_param = ui_test.find(root_widget_path + "/HStack[2]/StringField[0]")
        robot_param.model.set_value(self._robot_path)

        # Find and set wheel radius parameter
        wheel_radius = ui_test.find(root_widget_path + "/HStack[3]/FloatField[0]")
        wheel_radius.model.set_value("0.0325")

        # Find and set wheel distance parameter
        wheel_distance = ui_test.find(root_widget_path + "/HStack[4]/FloatField[0]")
        wheel_distance.model.set_value("0.118")

        # Find and set joint names, right and left
        right_joint_name = ui_test.find(root_widget_path + "/VStack[0]/HStack[0]/StringField[0]")
        right_joint_name.model.set_value("right_wheel_joint")
        left_joint_name = ui_test.find(root_widget_path + "/VStack[0]/HStack[1]/StringField[0]")
        left_joint_name.model.set_value("left_wheel_joint")

        await omni.kit.app.get_app().next_update_async()

        # Click OK button
        ok_button = ui_test.find(root_widget_path + "/HStack[6]/Button[0]")
        await ok_button.click()

        await omni.kit.app.get_app().next_update_async()

        # find the differential controller node
        diff_node = og.get_node_by_path(graph_path + "/DifferentialController")
        self.assertIsNotNone(diff_node, "Differential controller node not found")

        # set the desired linear and angular velocities in the differential controller node
        linear_velocity = 0.3
        angular_velocity = 1.0

        og.ObjectLookup.attribute("inputs:linearVelocity", diff_node).set(linear_velocity)
        og.ObjectLookup.attribute("inputs:angularVelocity", diff_node).set(angular_velocity)

        await omni.kit.app.get_app().next_update_async()

        # run the graph for 3 seconds
        self._timeline.play()
        await simulate_async(3.0, 60)

        # find how much the robot has travelled
        robot_position = robot.get_world_poses()[0][0]
        self._timeline.stop()

        self.assertAlmostEqual(robot_position[0], 0.06, delta=0.02)
        self.assertAlmostEqual(robot_position[1], -0.61, delta=0.02)
        self.assertAlmostEqual(robot_position[2], 0.033, delta=0.02)
