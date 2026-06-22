# SPDX-FileCopyrightText: Copyright (c) 2025-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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

"""Tests for the differential controller graph creation UI."""

from __future__ import annotations

import isaacsim.core.experimental.utils.app as app_utils
import omni.graph.core as og
import omni.kit.test  # noqa: F401
import omni.kit.ui_test as ui_test
from isaacsim.core.simulation_manager import SimulationManager
from isaacsim.storage.native import get_assets_root_path_async
from isaacsim.test.utils import MenuUITestCase


class TestDifferentialRobotGraph(MenuUITestCase):
    """Test cases for the differential controller graph creation menu UI."""

    async def setUp(self) -> None:
        """Set up the test environment with a Jetbot robot on stage."""
        await super().setUp()
        SimulationManager.setup_simulation()

        self._robot_path = "/World/test_robot"
        robot_prim = self._stage.DefinePrim(self._robot_path, "Xform")
        assets_root_path = await get_assets_root_path_async()
        if assets_root_path is None:
            import carb

            carb.log_error("Could not find Isaac Sim assets folder")
            return
        robot_prim.GetReferences().AddReference(assets_root_path + "/Isaac/Robots/NVIDIA/Jetbot/jetbot.usd")
        await app_utils.update_app_async()
        await self.wait_for_stage_loading()

    async def tearDown(self) -> None:
        """Stop simulation and clean up."""
        app_utils.stop()
        await self.wait_for_stage_loading()
        await super().tearDown()

    async def test_basic_graph_creation(self) -> None:
        """Test creation of basic differential drive graph structure."""
        # Click through the menu to create the graph
        window_name = "Differential Controller"
        param_window = await self.menu_click_with_retry(
            "Tools/Robotics/OmniGraph Controllers/Differential Controller", window_name=window_name
        )
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

        # Find and set joint names in Differential Controller output order: left, then right
        left_joint_name = ui_test.find(root_widget_path + "/VStack[0]/HStack[0]/StringField[0]")
        left_joint_name.model.set_value("left_wheel_joint")
        right_joint_name = ui_test.find(root_widget_path + "/VStack[0]/HStack[1]/StringField[0]")
        right_joint_name.model.set_value("right_wheel_joint")

        await app_utils.update_app_async()

        # Click OK button
        ok_button = ui_test.find(root_widget_path + "/HStack[6]/Button[0]")
        self.assertIsNotNone(ok_button, "OK button not found")
        await ok_button.click()

        await app_utils.update_app_async()

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

        array_node_path = graph_test_path + "/ArrayNames"
        self.assertEqual(
            og.Controller(og.Controller.attribute(array_node_path + ".inputs:input0")).get(), "left_wheel_joint"
        )
        self.assertEqual(
            og.Controller(og.Controller.attribute(array_node_path + ".inputs:input1")).get(), "right_wheel_joint"
        )

    async def test_joint_indices_use_left_right_order(self) -> None:
        """Test generated joint index arrays support index 0 and match Differential Controller output order."""
        window_name = "Differential Controller"
        param_window = await self.menu_click_with_retry(
            "Tools/Robotics/OmniGraph Controllers/Differential Controller", window_name=window_name
        )
        self.assertIsNotNone(param_window, "Parameter window not found")

        graph_test_path = "/World/test_graph_indices"
        root_widget_path = f"{window_name}//Frame/VStack[0]"
        graph_root_prim = ui_test.find(root_widget_path + "/HStack[1]/StringField[0]")
        graph_root_prim.model.set_value(graph_test_path)

        robot_prim = ui_test.find(root_widget_path + "/HStack[2]/StringField[0]")
        robot_prim.model.set_value(self._robot_path)

        wheel_radius = ui_test.find(root_widget_path + "/HStack[3]/FloatField[0]")
        wheel_radius.model.set_value("0.0325")

        wheel_distance = ui_test.find(root_widget_path + "/HStack[4]/FloatField[0]")
        wheel_distance.model.set_value("0.118")

        left_joint_index = ui_test.find(root_widget_path + "/VStack[1]/HStack[0]/IntField[0]")
        left_joint_index.model.set_value(0)
        right_joint_index = ui_test.find(root_widget_path + "/VStack[1]/HStack[1]/IntField[0]")
        right_joint_index.model.set_value(1)

        await app_utils.update_app_async()

        ok_button = ui_test.find(root_widget_path + "/HStack[6]/Button[0]")
        self.assertIsNotNone(ok_button, "OK button not found")
        await ok_button.click()

        await app_utils.update_app_async()

        graph = og.get_graph_by_path(graph_test_path)
        self.assertIsNotNone(graph, "Graph was not created")

        array_node_path = graph_test_path + "/ArrayNames"
        self.assertEqual(og.Controller(og.Controller.attribute(array_node_path + ".inputs:input0")).get(), 0)
        self.assertEqual(og.Controller(og.Controller.attribute(array_node_path + ".inputs:input1")).get(), 1)

    async def test_add_to_existing_graph(self) -> None:
        """Test adding differential drive nodes to an existing graph."""
        # First create a base graph with just a tick node
        graph_path = "/World/test_graph"
        graph = og.Controller.create_graph({"graph_path": graph_path, "evaluator_name": "execution"})
        og.Controller.create_node(graph_path + "/OnPlaybackTick", "omni.graph.action.OnPlaybackTick")
        # Open UI and set to add to existing graph
        window_name = "Differential Controller"
        param_window = await self.menu_click_with_retry(
            "Tools/Robotics/OmniGraph Controllers/Differential Controller",
            delays=[10, 100, 200],
            window_name=window_name,
            wait_n_frames=20,
        )
        self.assertIsNotNone(param_window, "Parameter window not found")

        # Find and check the "Add to Existing Graph" checkbox
        root_widget_path = f"{window_name}//Frame/VStack[0]"
        add_to_graph_checkbox = ui_test.find(root_widget_path + "/HStack[0]/HStack[0]/VStack[0]/ToolButton[0]")
        self.assertIsNotNone(add_to_graph_checkbox, "Add to existing graph checkbox not found")
        await add_to_graph_checkbox.click()
        await app_utils.update_app_async(steps=10)
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

        await app_utils.update_app_async(steps=10)

        # Click OK button
        ok_button = ui_test.find(root_widget_path + "/HStack[6]/Button[0]")
        self.assertIsNotNone(ok_button, "OK button not found")
        await ok_button.click()

        await app_utils.update_app_async(steps=10)

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

    async def test_keyboard_control(self) -> None:
        """Test keyboard control nodes are created in the differential drive graph."""
        # Click through the menu to create the graph
        window_name = "Differential Controller"
        param_window = await self.menu_click_with_retry(
            "Tools/Robotics/OmniGraph Controllers/Differential Controller", window_name=window_name
        )
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

        await app_utils.update_app_async()

        # Click OK button
        ok_button = ui_test.find(root_widget_path + "/HStack[6]/Button[0]")
        self.assertIsNotNone(ok_button, "OK button not found")
        await ok_button.click()

        await app_utils.update_app_async()

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

    async def test_differential_drive_golden(self) -> None:
        """Test differential drive computation with golden values."""
        from isaacsim.core.experimental.objects import GroundPlane
        from isaacsim.core.experimental.prims import Articulation

        GroundPlane("/World/groundPlane")

        robot = Articulation(paths=self._robot_path)

        app_utils.play()
        robot_position = robot.get_world_poses()[0].numpy()[0]
        self.assertAlmostEqual(robot_position[0], 0.0, delta=0.05)
        self.assertAlmostEqual(robot_position[1], 0.0, delta=0.05)
        self.assertAlmostEqual(robot_position[2], 0.033, delta=0.01)
        app_utils.stop()

        window_name = "Differential Controller"
        param_window = await self.menu_click_with_retry(
            "Tools/Robotics/OmniGraph Controllers/Differential Controller", window_name=window_name
        )
        self.assertIsNotNone(param_window, "Parameter window not found")

        root_widget_path = f"{window_name}//Frame/VStack[0]"
        graph_path = "/Graphs/differential_controller"

        robot_param = ui_test.find(root_widget_path + "/HStack[2]/StringField[0]")
        robot_param.model.set_value(self._robot_path)

        wheel_radius_value = 0.0325
        wheel_radius = ui_test.find(root_widget_path + "/HStack[3]/FloatField[0]")
        wheel_radius.model.set_value(str(wheel_radius_value))

        wheel_distance_value = 0.118
        wheel_distance = ui_test.find(root_widget_path + "/HStack[4]/FloatField[0]")
        wheel_distance.model.set_value(str(wheel_distance_value))

        left_joint_name = ui_test.find(root_widget_path + "/VStack[0]/HStack[0]/StringField[0]")
        left_joint_name.model.set_value("left_wheel_joint")
        right_joint_name = ui_test.find(root_widget_path + "/VStack[0]/HStack[1]/StringField[0]")
        right_joint_name.model.set_value("right_wheel_joint")

        await app_utils.update_app_async()

        ok_button = ui_test.find(root_widget_path + "/HStack[6]/Button[0]")
        await ok_button.click()

        await app_utils.update_app_async()

        diff_node = og.get_node_by_path(graph_path + "/DifferentialController")
        self.assertIsNotNone(diff_node, "Differential controller node not found")

        linear_velocity = 0.3
        angular_velocity = 1.0

        og.ObjectLookup.attribute("inputs:linearVelocity", diff_node).set(linear_velocity)
        og.ObjectLookup.attribute("inputs:angularVelocity", diff_node).set(angular_velocity)

        await app_utils.update_app_async()

        app_utils.play()
        await app_utils.update_app_async()

        velocity_command = og.Controller(og.Controller.attribute("outputs:velocityCommand", diff_node)).get()
        expected_left_velocity = ((2 * linear_velocity) - (angular_velocity * wheel_distance_value)) / (
            2 * wheel_radius_value
        )
        expected_right_velocity = ((2 * linear_velocity) + (angular_velocity * wheel_distance_value)) / (
            2 * wheel_radius_value
        )
        self.assertAlmostEqual(velocity_command[0], expected_left_velocity, delta=1e-6)
        self.assertAlmostEqual(velocity_command[1], expected_right_velocity, delta=1e-6)
        self.assertLess(
            velocity_command[0], velocity_command[1], "Positive angular velocity should command right wheel faster"
        )

        await app_utils.update_app_async(steps=179)

        robot_position = robot.get_world_poses()[0].numpy()[0]
        app_utils.stop()

        self.assertAlmostEqual(robot_position[0], 0.06, delta=0.02)
        # Positive angular velocity with DifferentialController's left/right output order turns toward +Y.
        self.assertAlmostEqual(robot_position[1], 0.61, delta=0.02)
        self.assertAlmostEqual(robot_position[2], 0.033, delta=0.02)
