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

import numpy as np
import omni.graph.core as og
import omni.kit.test
import omni.kit.ui_test as ui_test
import omni.timeline
import omni.usd
from isaacsim.core.prims import Articulation
from isaacsim.core.utils.physics import simulate_async
from isaacsim.core.utils.stage import update_stage_async
from isaacsim.storage.native import get_assets_root_path_async
from omni.kit.ui_test.menu import *
from omni.kit.ui_test.query import *
from omni.physx.scripts.physicsUtils import add_ground_plane
from omni.ui.tests.test_base import OmniUiTest
from pxr import Gf, UsdPhysics


class TestArticulationGraphs(OmniUiTest):
    async def setUp(self):
        await omni.usd.get_context().new_stage_async()
        await update_stage_async()
        self._stage = omni.usd.get_context().get_stage()
        self._timeline = omni.timeline.get_timeline_interface()

    async def tearDown(self):
        self._timeline.stop()
        while omni.usd.get_context().get_stage_loading_status()[2] > 0:
            print("tearDown, assets still loading, waiting to finish...")
            await asyncio.sleep(1.0)
        await update_stage_async()

    async def test_position_graph_creation(self):
        """Test creation of articulation position controller graph"""

        # Add robot to stage (Franka)
        robot_path = "/World/test_robot"
        robot_prim = self._stage.DefinePrim(robot_path, "Xform")
        assets_root_path = await get_assets_root_path_async()
        if assets_root_path is None:
            carb.log_error("Could not find Isaac Sim assets folder")
            return
        robot_prim.GetReferences().AddReference(
            assets_root_path + "/Isaac/Robots/FrankaRobotics/FrankaPanda/franka.usd"
        )
        await update_stage_async()

        # Create position controller graph
        await menu_click("Tools/Robotics/OmniGraph Controllers/Joint Position")
        await update_stage_async()

        window_name = "Articulation Position Controller"
        param_window = ui_test.find(window_name)
        self.assertIsNotNone(param_window, "Parameter window not found")

        root_widget_path = f"{window_name}//Frame/VStack[0]"
        robot_prim = ui_test.find(root_widget_path + "/HStack[1]/StringField[0]")
        robot_prim.model.set_value(robot_path)

        await update_stage_async()

        ok_button = ui_test.find(root_widget_path + "/HStack[4]/Button[0]")
        await ok_button.click()

        await update_stage_async()

        # Verify graph creation
        graph = og.get_graph_by_path("/Graphs/Position_Controller")
        self.assertIsNotNone(graph, "Graph was not created")

        # Check for essential nodes
        nodes = graph.get_nodes()
        node_types = {node.get_type_name() for node in nodes}

        expected_nodes = {
            "omni.graph.action.OnPlaybackTick",
            "isaacsim.core.nodes.IsaacArticulationController",
            "omni.graph.nodes.ConstructArray",
        }

        for expected in expected_nodes:
            self.assertIn(expected, node_types, f"Missing expected node type: {expected}")

    async def test_position_control_golden(self):
        """Test position control with golden values"""

        # Add robot to stage (Franka)
        robot_path = "/World/test_robot"
        robot_prim = self._stage.DefinePrim(robot_path, "Xform")
        assets_root_path = await get_assets_root_path_async()
        if assets_root_path is None:
            carb.log_error("Could not find Isaac Sim assets folder")
            return
        robot_prim.GetReferences().AddReference(
            assets_root_path + "/Isaac/Robots/FrankaRobotics/FrankaPanda/franka.usd"
        )
        await update_stage_async()

        # First create a base graph with just a tick node
        graph_path = "/World/test_graph"
        graph = og.Controller.create_graph({"graph_path": graph_path, "evaluator_name": "execution"})
        og.Controller.create_node(graph_path + "/OnPlaybackTick", "omni.graph.action.OnPlaybackTick")

        # Create position controller graph
        await menu_click("Tools/Robotics/OmniGraph Controllers/Joint Position")
        await update_stage_async()

        window_name = "Articulation Position Controller"
        param_window = ui_test.find(window_name)
        self.assertIsNotNone(param_window, "Parameter window not found")

        # Enable "Add to existing graph"
        root_widget_path = f"{window_name}//Frame/VStack[0]"
        add_to_graph_checkbox = ui_test.find(root_widget_path + "/HStack[0]/HStack[0]/VStack[0]/ToolButton[0]")
        await add_to_graph_checkbox.click()

        # Set robot prim
        robot_prim = ui_test.find(root_widget_path + "/HStack[1]/StringField[0]")
        robot_prim.model.set_value(robot_path)
        # Set the existing graph path
        graph_field = ui_test.find(root_widget_path + "/HStack[2]/StringField[0]")
        graph_field.model.set_value(graph_path)

        await update_stage_async()

        ok_button = ui_test.find(root_widget_path + "/HStack[4]/Button[0]")
        await ok_button.click()

        await update_stage_async()

        # setup jointNames and Desired Joint Positions
        joint_names = [
            "panda_joint1",
            "panda_joint2",
            "panda_joint3",
            "panda_joint4",
            "panda_joint5",
            "panda_joint6",
            "panda_joint7",
            "panda_finger_joint1",
        ]  # note that panda_finger_joint2 is a mimic joint
        desired_joint_positions = [0.0, np.pi / 4, 0.0, -np.pi / 4, 0.0, np.pi / 2, 0.0, 0.02]

        # Get graph and set joint positions
        joint_name_array = og.get_node_by_path(f"{graph_path}/JointNameArray")
        joint_command_array = og.get_node_by_path(f"{graph_path}/JointCommandArray")

        # Set first joint to 45 degrees (π/4 radians)
        for i in range(len(joint_names)):
            og.ObjectLookup.attribute("inputs:input" + str(i), joint_name_array).set(joint_names[i])
            og.ObjectLookup.attribute("inputs:input" + str(i), joint_command_array).set(desired_joint_positions[i])

        # Run simulation and verify
        self._timeline.play()
        robot = Articulation(prim_paths_expr=robot_path, name="test_robot")
        await simulate_async(1.0, 60)
        joint_positions = robot.get_joint_positions()[0]

        self._timeline.stop()
        # Verify first joint reached target position
        self.assertAlmostEqual(
            joint_positions[0], desired_joint_positions[0], places=1, msg="Joint[0] did not reach target position"
        )
        self.assertAlmostEqual(
            joint_positions[1], desired_joint_positions[1], places=1, msg="Joint[1] did not reach target position"
        )
        self.assertAlmostEqual(
            joint_positions[2], desired_joint_positions[2], places=1, msg="Joint[2] did not reach target position"
        )
        self.assertAlmostEqual(
            joint_positions[3], desired_joint_positions[3], places=1, msg="Joint[3] did not reach target position"
        )
        self.assertAlmostEqual(
            joint_positions[4], desired_joint_positions[4], places=1, msg="Joint[4] did not reach target position"
        )
        self.assertAlmostEqual(
            joint_positions[5], desired_joint_positions[5], places=1, msg="Joint[5] did not reach target position"
        )
        self.assertAlmostEqual(
            joint_positions[6], desired_joint_positions[6], places=1, msg="Joint[6] did not reach target position"
        )
        self.assertAlmostEqual(
            joint_positions[7], desired_joint_positions[7], places=1, msg="Joint[7] did not reach target position"
        )

    async def test_velocity_graph_creation(self):
        """Test creation of articulation velocity controller graph"""

        # Add robot to stage (Jetbot)
        robot_path = "/World/test_robot"
        robot_prim = self._stage.DefinePrim(robot_path, "Xform")
        assets_root_path = await get_assets_root_path_async()
        if assets_root_path is None:
            carb.log_error("Could not find Isaac Sim assets folder")
            return
        robot_prim.GetReferences().AddReference(assets_root_path + "/Isaac/Robots/NVIDIA/Jetbot/jetbot.usd")
        await update_stage_async()

        # Create velocity controller graph
        await menu_click("Tools/Robotics/OmniGraph Controllers/Joint Velocity")
        await update_stage_async()

        window_name = "Articulation Velocity Controller"
        param_window = ui_test.find(window_name)
        self.assertIsNotNone(param_window, "Parameter window not found")

        root_widget_path = f"{window_name}//Frame/VStack[0]"
        robot_prim = ui_test.find(root_widget_path + "/HStack[1]/StringField[0]")
        robot_prim.model.set_value(robot_path)

        await update_stage_async()

        ok_button = ui_test.find(root_widget_path + "/HStack[4]/Button[0]")
        await ok_button.click()

        await update_stage_async()

        # Verify graph creation
        graph = og.get_graph_by_path("/Graphs/Velocity_Controller")
        self.assertIsNotNone(graph, "Graph was not created")

        # Check for essential nodes
        nodes = graph.get_nodes()
        node_types = {node.get_type_name() for node in nodes}

        expected_nodes = {
            "omni.graph.action.OnPlaybackTick",
            "isaacsim.core.nodes.IsaacArticulationController",
            "omni.graph.nodes.ConstructArray",
        }

        for expected in expected_nodes:
            self.assertIn(expected, node_types, f"Missing expected node type: {expected}")

    async def test_velocity_control_golden(self):
        """Test velocity control with golden values"""
        # Add robot to stage (Jetbot)
        robot_path = "/World/test_robot"
        robot_prim = self._stage.DefinePrim(robot_path, "Xform")
        assets_root_path = await get_assets_root_path_async()
        if assets_root_path is None:
            carb.log_error("Could not find Isaac Sim assets folder")
            return
        robot_prim.GetReferences().AddReference(assets_root_path + "/Isaac/Robots/NVIDIA/Jetbot/jetbot.usd")
        # add a ground plane
        await update_stage_async()

        add_ground_plane(self._stage, "/World/groundPlane", "Z", 100, Gf.Vec3f(0, 0, 0), Gf.Vec3f(1.0))

        await update_stage_async()
        # First create a base graph with just a tick node
        graph_path = "/World/test_graph"
        graph = og.Controller.create_graph({"graph_path": graph_path, "evaluator_name": "execution"})
        og.Controller.create_node(graph_path + "/OnPlaybackTick", "omni.graph.action.OnPlaybackTick")

        # Create velocity controller graph
        await menu_click("Tools/Robotics/OmniGraph Controllers/Joint Velocity")
        await update_stage_async()

        window_name = "Articulation Velocity Controller"
        param_window = ui_test.find(window_name)
        self.assertIsNotNone(param_window, "Parameter window not found")

        # Enable "Add to existing graph"
        root_widget_path = f"{window_name}//Frame/VStack[0]"
        add_to_graph_checkbox = ui_test.find(root_widget_path + "/HStack[0]/HStack[0]/VStack[0]/ToolButton[0]")
        await add_to_graph_checkbox.click()

        # Set robot prim
        robot_prim = ui_test.find(root_widget_path + "/HStack[1]/StringField[0]")
        robot_prim.model.set_value(robot_path)
        # Set the existing graph path
        graph_field = ui_test.find(root_widget_path + "/HStack[2]/StringField[0]")
        graph_field.model.set_value(graph_path)
        await update_stage_async()
        ok_button = ui_test.find(root_widget_path + "/HStack[4]/Button[0]")
        await ok_button.click()

        await update_stage_async()

        # setup jointNames and Desired Joint Positions
        joint_names = ["left_wheel_joint", "right_wheel_joint"]
        desired_joint_velocities = [2.0, 5.0]

        # Get graph and set joint positions
        joint_name_array = og.get_node_by_path(f"{graph_path}/JointNameArray")
        joint_command_array = og.get_node_by_path(f"{graph_path}/JointCommandArray")

        # Set first joint to 45 degrees (π/4 radians)
        for i in range(len(joint_names)):
            og.ObjectLookup.attribute("inputs:input" + str(i), joint_name_array).set(joint_names[i])
            og.ObjectLookup.attribute("inputs:input" + str(i), joint_command_array).set(desired_joint_velocities[i])

        # Run simulation and verify
        self._timeline.play()
        robot = Articulation(prim_paths_expr=robot_path, name="test_robot")
        await simulate_async(2.0, 60)
        robot_position = robot.get_world_poses()[0][0]
        self._timeline.stop()

        print(robot_position)
        golden_final_positions = [0.13956, 0.153, 0.0335]

        # find how much the robot has travelled
        self.assertAlmostEqual(robot_position[0], golden_final_positions[0], delta=0.02)
        self.assertAlmostEqual(robot_position[1], golden_final_positions[1], delta=0.02)
        self.assertAlmostEqual(robot_position[2], golden_final_positions[2], delta=0.02)

    async def test_gripper_graph_creation(self):
        """Test creation of gripper controller graph"""
        # Add robot to stage (Franka)
        robot_path = "/World/test_robot"
        robot_prim = self._stage.DefinePrim(robot_path, "Xform")
        assets_root_path = await get_assets_root_path_async()
        if assets_root_path is None:
            carb.log_error("Could not find Isaac Sim assets folder")
            return
        robot_prim.GetReferences().AddReference(
            assets_root_path + "/Isaac/Robots/FrankaRobotics/FrankaPanda/franka.usd"
        )
        await update_stage_async()
        # Create gripper controller graph
        await menu_click("Tools/Robotics/OmniGraph Controllers/Open Loop Gripper")
        await update_stage_async()

        window_name = "Gripper Controller"
        param_window = ui_test.find(window_name)
        self.assertIsNotNone(param_window, "Parameter window not found")

        root_widget_path = f"{window_name}//Frame/VStack[0]"

        # Set robot and gripper prims
        parent_robot = ui_test.find(root_widget_path + "/HStack[1]/StringField[0]")
        parent_robot.model.set_value(robot_path)

        gripper_root = ui_test.find(root_widget_path + "/HStack[2]/StringField[0]")
        gripper_root.model.set_value(robot_path + "/panda_hand")

        # Set gripper parameters
        speed_input = ui_test.find(root_widget_path + "/HStack[4]/FloatField[0]")
        speed_input.model.set_value(0.005)

        # Set joint names for gripper
        joint_names = ui_test.find(root_widget_path + "/HStack[5]/StringField[0]")
        joint_names.model.set_value("panda_finger_joint1, panda_finger_joint2")

        # Enable keyboard control
        keyboard_checkbox = ui_test.find(root_widget_path + "/HStack[8]/HStack[0]/VStack[0]/ToolButton[0]")
        keyboard_checkbox.model.set_value(True)

        await update_stage_async()

        ok_button = ui_test.find(root_widget_path + "/HStack[9]/Button[0]")
        await ok_button.click()

        await update_stage_async()

        # Verify graph creation
        graph = og.get_graph_by_path("/Graphs/Gripper_Controller")
        self.assertIsNotNone(graph, "Graph was not created")

        # Check for essential nodes
        nodes = graph.get_nodes()
        node_types = {node.get_type_name() for node in nodes}

        expected_nodes = {
            "omni.graph.action.OnPlaybackTick",
            "isaacsim.robot.manipulators.IsaacGripperController",
            "omni.graph.nodes.ConstructArray",
            "omni.graph.nodes.ConstantDouble",
            "omni.graph.action.OnKeyboardInput",
        }

        for expected in expected_nodes:
            self.assertIn(expected, node_types, f"Missing expected node type: {expected}")

    async def test_gripper_control_golden(self):
        """Test gripper control with golden values"""

        # Add robot to stage (Franka)
        robot_path = "/World/test_robot"
        robot_prim = self._stage.DefinePrim(robot_path, "Xform")
        assets_root_path = await get_assets_root_path_async()
        if assets_root_path is None:
            carb.log_error("Could not find Isaac Sim assets folder")
            return
        robot_prim.GetReferences().AddReference(
            assets_root_path + "/Isaac/Robots/FrankaRobotics/FrankaPanda/franka.usd"
        )
        await update_stage_async()

        # First create a base graph with just a tick node
        graph_path = "/World/test_graph"
        graph = og.Controller.create_graph({"graph_path": graph_path, "evaluator_name": "execution"})
        og.Controller.create_node(graph_path + "/OnPlaybackTick", "omni.graph.action.OnPlaybackTick")

        for _ in range(10):
            await update_stage_async()

        # Create gripper controller graph. Check for parameter window before continuing.
        # If the parameter window is not found, retry with increasing delays
        window_name = "Gripper Controller"
        delays = [5, 50, 100]
        for delay in delays:
            try:
                await menu_click("Tools/Robotics/OmniGraph Controllers/Open Loop Gripper", human_delay_speed=delay)
                if (param_window := ui_test.find(window_name)) is not None:
                    break
            except AttributeError as e:
                if "NoneType' object has no attribute 'center'" in str(e) and delay != delays[-1]:
                    continue
                raise
        for _ in range(10):
            await update_stage_async()

        self.assertIsNotNone(param_window, "Parameter window not found")

        # Enable "Add to existing graph"
        root_widget_path = f"{window_name}//Frame/VStack[0]"
        add_to_graph_checkbox = ui_test.find(root_widget_path + "/HStack[0]/HStack[0]/VStack[0]/ToolButton[0]")
        self.assertIsNotNone(add_to_graph_checkbox, "Add to existing graph checkbox not found")
        await add_to_graph_checkbox.click()
        for _ in range(10):
            await update_stage_async()
        # Set the existing graph path
        graph_field = ui_test.find(root_widget_path + "/HStack[3]/StringField[0]")
        graph_field.model.set_value(graph_path)

        # Set robot and gripper prims
        parent_robot = ui_test.find(root_widget_path + "/HStack[1]/StringField[0]")
        parent_robot.model.set_value(robot_path)

        gripper_root = ui_test.find(root_widget_path + "/HStack[2]/StringField[0]")
        gripper_root.model.set_value(robot_path + "/panda_hand")

        # Set gripper parameters
        speed_input = ui_test.find(root_widget_path + "/HStack[4]/FloatField[0]")
        speed_input.model.set_value(0.005)

        # Set joint names for gripper
        joint_names = ui_test.find(root_widget_path + "/HStack[5]/StringField[0]")
        joint_names.model.set_value("panda_finger_joint1, panda_finger_joint2")

        for _ in range(10):
            await update_stage_async()

        ok_button = ui_test.find(root_widget_path + "/HStack[9]/Button[0]")
        self.assertIsNotNone(ok_button, "OK button not found")
        await ok_button.click()

        for _ in range(10):
            await update_stage_async()

        # get the gripper controller node
        gripper_controller = og.get_node_by_path(f"{graph_path}/GripperController")
        open_attr = og.Controller.attribute("inputs:open", gripper_controller)
        close_attr = og.Controller.attribute("inputs:close", gripper_controller)
        stop_attr = og.Controller.attribute("inputs:stop", gripper_controller)

        # Run simulation and verify
        self._timeline.play()
        robot = Articulation(prim_paths_expr=robot_path, name="test_robot")
        # manually trigger the gripper to close
        close_attr.set(og.ExecutionAttributeState.ENABLED)
        open_attr.set(og.ExecutionAttributeState.DISABLED)
        await og.Controller.evaluate()
        await simulate_async(1.0, 60)

        # get the joint positions
        gripper_closed_position = robot.get_joint_positions()[0][-2:]

        # manually trigger the gripper to open
        close_attr.set(og.ExecutionAttributeState.DISABLED)
        open_attr.set(og.ExecutionAttributeState.ENABLED)
        await og.Controller.evaluate()

        await simulate_async(1.0, 60)
        gripper_open_position = robot.get_joint_positions()[0][-2:]

        # trigger to close again, but trigger stop shortly after moving
        close_attr.set(og.ExecutionAttributeState.ENABLED)
        open_attr.set(og.ExecutionAttributeState.DISABLED)
        await og.Controller.evaluate()
        await simulate_async(0.2, 60)
        gripper_stop_position = robot.get_joint_positions()[0][-2:]
        self._timeline.stop()

        # verify the gripper closed position
        self.assertAlmostEqual(gripper_closed_position[0], 0.0, delta=0.02)
        self.assertAlmostEqual(gripper_closed_position[1], 0.0, delta=0.02)
        self.assertAlmostEqual(gripper_open_position[0], 0.04, delta=0.02)
        self.assertAlmostEqual(gripper_open_position[1], 0.04, delta=0.02)
        self.assertAlmostEqual(gripper_stop_position[0], 0.01, delta=0.02)
        self.assertAlmostEqual(gripper_stop_position[1], 0.01, delta=0.02)
