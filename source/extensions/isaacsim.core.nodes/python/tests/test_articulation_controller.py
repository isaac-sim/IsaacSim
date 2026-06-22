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

"""Verifies the articulation controller OmniGraph node applies valid joint commands and rejects invalid ones safely. Covers name and index selection, command shape mismatches, NaN filtering, partial target prevention, and actionable initialization errors."""

import asyncio
from typing import Any

import carb
import isaacsim.core.experimental.utils.app as app_utils
import isaacsim.core.experimental.utils.stage as stage_utils
import numpy as np
import omni.graph.core as og
import omni.graph.core.tests as ogts
import omni.kit.test
from isaacsim.core.experimental.prims import Articulation
from isaacsim.core.nodes.ogn.python.nodes.OgnIsaacArticulationController import (
    OgnIsaacArticulationController,
    OgnIsaacArticulationControllerInternalState,
)
from isaacsim.storage.native import get_assets_root_path_async


class TestArticulationControllerWrapper(omni.kit.test.AsyncTestCase):
    """Verify controller compute errors without building a full OmniGraph."""

    # ----------------------------------------------------------------------
    async def test_initialization_exception_logs_error_with_prim_path(self) -> None:
        """Test that initialization errors include the prim path."""

        class FakeInputs:
            robotPath = "/BadRobot"
            targetPrim = []
            jointNames = []
            jointIndices = []
            positionCommand = []
            velocityCommand = []
            effortCommand = []

        class FakeState:
            initialized = False
            prim_path = None

            def initialize_controller(self) -> None:
                raise RuntimeError("init failed")

        class FakeDb:
            def __init__(self) -> None:
                self.inputs = FakeInputs()
                self.per_instance_state = FakeState()
                self.errors = []
                self.warnings = []

            def log_error(self, message: Any) -> None:
                self.errors.append(message)

            def log_warn(self, message: Any) -> None:
                self.warnings.append(message)

        db = FakeDb()

        self.assertFalse(OgnIsaacArticulationController.compute(db))
        self.assertEqual(db.warnings, [])
        self.assertEqual(len(db.errors), 1)
        self.assertIn("/BadRobot", db.errors[0])
        self.assertIn("init failed", db.errors[0])

    # ----------------------------------------------------------------------
    async def test_command_mismatch_logs_actionable_error_and_does_not_apply_partial_targets(self) -> None:
        """Verify a width mismatch rejects the whole command instead of applying partial targets."""

        class FakeInputs:
            robotPath = "/Robot"
            targetPrim = []
            jointNames = []
            jointIndices = []
            positionCommand = [0.1, 0.2]
            velocityCommand = []
            effortCommand = [0.3]

        class FakeArticulation:
            def __init__(self) -> None:
                self.position_targets = []
                self.velocity_targets = []
                self.efforts = []

            def set_dof_position_targets(self, *args: Any, **kwargs: Any) -> None:
                self.position_targets.append((args, kwargs))

            def set_dof_velocity_targets(self, *args: Any, **kwargs: Any) -> None:
                self.velocity_targets.append((args, kwargs))

            def set_dof_efforts(self, *args: Any, **kwargs: Any) -> None:
                self.efforts.append((args, kwargs))

        class FakeDb:
            def __init__(self, state: Any) -> None:
                self.inputs = FakeInputs()
                self.per_instance_state = state
                self.errors = []
                self.warnings = []

            def log_error(self, message: Any) -> None:
                self.errors.append(message)

            def log_warn(self, message: Any) -> None:
                self.warnings.append(message)

        state = OgnIsaacArticulationControllerInternalState.__new__(OgnIsaacArticulationControllerInternalState)
        state.initialized = True
        state.prim_path = "/Robot"
        state.joint_names = None
        state.joint_indices = [1, 2]
        state.joint_picked = True
        state.articulation = FakeArticulation()
        db = FakeDb(state)

        self.assertFalse(OgnIsaacArticulationController.compute(db))
        self.assertEqual(db.warnings, [])
        self.assertEqual(len(db.errors), 1)
        self.assertIn("/Robot", db.errors[0])
        self.assertIn("effortCommand", db.errors[0])
        self.assertIn("1", db.errors[0])
        self.assertIn("2", db.errors[0])
        self.assertEqual(state.articulation.position_targets, [])
        self.assertEqual(state.articulation.velocity_targets, [])
        self.assertEqual(state.articulation.efforts, [])


class TestArticulationControllerNode(ogts.OmniGraphTestCase):
    """Verify articulation controller graph behavior on a loaded Franka articulation."""

    async def setUp(self) -> None:
        """Set up  test environment, to be torn down when done."""
        await omni.usd.get_context().new_stage_async()
        # add franka robot for test
        assets_root_path = await get_assets_root_path_async()
        if assets_root_path is None:
            carb.log_error("Could not find Isaac Sim assets folder")
            return
        await stage_utils.open_stage_async(assets_root_path + "/Isaac/Robots/FrankaRobotics/FrankaPanda/franka.usd")

    # ----------------------------------------------------------------------
    async def tearDown(self) -> None:
        """Get rid of temporary data used by the test."""
        await omni.kit.app.get_app().next_update_async()
        while omni.usd.get_context().get_stage_loading_status()[2] > 0:
            print("tearDown, assets still loading, waiting to finish...")
            await asyncio.sleep(1.0)
        await omni.kit.app.get_app().next_update_async()
        return

    # ----------------------------------------------------------------------
    async def test_joint_name_ogn(self) -> None:
        """Verify position commands are applied to joints selected by name."""
        test_graph, new_nodes, _, _ = og.Controller.edit(
            {"graph_path": "/ActionGraph", "evaluator_name": "execution"},
            {
                og.Controller.Keys.CREATE_NODES: [
                    ("OnPlaybackTick", "omni.graph.action.OnPlaybackTick"),
                    ("JointNameArray", "omni.graph.nodes.ConstructArray"),
                    ("JointCommandArray", "omni.graph.nodes.ConstructArray"),
                    ("ArticulationController", "isaacsim.core.nodes.IsaacArticulationController"),
                ],
                og.Controller.Keys.CREATE_ATTRIBUTES: [
                    ("JointCommandArray.inputs:input1", "double"),
                ],
                og.Controller.Keys.SET_VALUES: [
                    ("JointNameArray.inputs:arraySize", 2),
                    ("JointNameArray.inputs:arrayType", "token[]"),
                    ("JointNameArray.inputs:input0", "panda_joint2"),
                    ("JointNameArray.inputs:input1", "panda_joint3"),
                    ("JointCommandArray.inputs:arraySize", 2),
                    ("JointCommandArray.inputs:arrayType", "double[]"),
                    ("JointCommandArray.inputs:input0", -1.0),
                    ("JointCommandArray.inputs:input1", 1.2),
                    ("ArticulationController.inputs:robotPath", "/panda"),
                ],
                og.Controller.Keys.CREATE_ATTRIBUTES: [
                    ("JointNameArray.inputs:input1", "token"),
                    ("JointCommandArray.inputs:input1", "double"),
                ],
                og.Controller.Keys.CONNECT: [
                    ("OnPlaybackTick.outputs:tick", "ArticulationController.inputs:execIn"),
                    ("JointNameArray.outputs:array", "ArticulationController.inputs:jointNames"),
                    ("JointCommandArray.outputs:array", "ArticulationController.inputs:positionCommand"),
                ],
            },
        )

        await og.Controller.evaluate(test_graph)

        # check where the joints are after evaluate
        robot = Articulation("/panda")
        app_utils.play()
        await app_utils.update_app_async(steps=120)

        self.assertAlmostEqual(robot.get_dof_positions().numpy()[0, 1], -1.0, delta=0.001)
        self.assertAlmostEqual(robot.get_dof_positions().numpy()[0, 2], 1.2, delta=0.002)

    # ----------------------------------------------------------------------
    async def test_joint_index_ogn(self) -> None:
        """Verify position commands are applied to joints selected by index."""
        test_graph, new_nodes, _, _ = og.Controller.edit(
            {"graph_path": "/ActionGraph", "evaluator_name": "execution"},
            {
                og.Controller.Keys.CREATE_NODES: [
                    ("OnPlaybackTick", "omni.graph.action.OnPlaybackTick"),
                    ("Joint1Index", "omni.graph.nodes.ConstantInt"),
                    ("Joint2Index", "omni.graph.nodes.ConstantInt"),
                    ("JointIndexArray", "omni.graph.nodes.ConstructArray"),
                    ("Joint1Position", "omni.graph.nodes.ConstantDouble"),
                    ("Joint2Position", "omni.graph.nodes.ConstantDouble"),
                    ("JointCommandArray", "omni.graph.nodes.ConstructArray"),
                    ("ArticulationController", "isaacsim.core.nodes.IsaacArticulationController"),
                ],
                og.Controller.Keys.CREATE_ATTRIBUTES: [
                    ("JointIndexArray.inputs:input1", "int"),
                    ("JointCommandArray.inputs:input1", "double"),
                ],
                og.Controller.Keys.SET_VALUES: [
                    ("Joint1Index.inputs:value", 1),
                    ("Joint2Index.inputs:value", 2),
                    ("Joint1Position.inputs:value", -1.0),
                    ("Joint2Position.inputs:value", 1.2),
                    ("JointIndexArray.inputs:arraySize", 2),
                    ("JointCommandArray.inputs:arraySize", 2),
                    ("ArticulationController.inputs:robotPath", "/panda"),
                ],
                og.Controller.Keys.CONNECT: [
                    ("OnPlaybackTick.outputs:tick", "ArticulationController.inputs:execIn"),
                    ("Joint1Index.inputs:value", "JointIndexArray.inputs:input0"),
                    ("Joint2Index.inputs:value", "JointIndexArray.inputs:input1"),
                    ("JointIndexArray.outputs:array", "ArticulationController.inputs:jointIndices"),
                    ("Joint1Position.inputs:value", "JointCommandArray.inputs:input0"),
                    ("Joint2Position.inputs:value", "JointCommandArray.inputs:input1"),
                    ("JointCommandArray.outputs:array", "ArticulationController.inputs:positionCommand"),
                ],
            },
        )

        await og.Controller.evaluate(test_graph)

        # check where the joints are after evaluate
        robot = Articulation("/panda")
        app_utils.play()
        await app_utils.update_app_async(steps=120)

        self.assertAlmostEqual(robot.get_dof_positions().numpy()[0, 1], -1.0, delta=0.002)
        self.assertAlmostEqual(robot.get_dof_positions().numpy()[0, 2], 1.2, delta=0.002)

    # ----------------------------------------------------------------------
    async def test_short_command_fails_for_explicit_joint_selection_mismatch(self) -> None:
        """Verify a command shorter than the explicit joint selection fails with a mismatch error."""
        state = OgnIsaacArticulationControllerInternalState.__new__(OgnIsaacArticulationControllerInternalState)
        state.initialized = True
        state.joint_indices = [1, 2]
        command = [0.0]

        command_valid, resolved_indices = state._resolve_command_indices(command)

        self.assertFalse(command_valid)
        self.assertEqual(resolved_indices, state.joint_indices)
        self.assertFalse(state.apply_action(command, [], []))

    # ----------------------------------------------------------------------
    async def test_invalid_command_does_not_apply_partial_targets(self) -> None:
        """Verify an invalid command input prevents other command targets from being written."""

        class FakeArticulation:
            def __init__(self) -> None:
                self.position_targets = []
                self.velocity_targets = []
                self.efforts = []

            def set_dof_position_targets(self, *args: Any, **kwargs: Any) -> None:
                self.position_targets.append((args, kwargs))

            def set_dof_velocity_targets(self, *args: Any, **kwargs: Any) -> None:
                self.velocity_targets.append((args, kwargs))

            def set_dof_efforts(self, *args: Any, **kwargs: Any) -> None:
                self.efforts.append((args, kwargs))

        state = OgnIsaacArticulationControllerInternalState.__new__(OgnIsaacArticulationControllerInternalState)
        state.initialized = True
        state.joint_indices = [1, 2]
        state.articulation = FakeArticulation()

        self.assertFalse(state.apply_action([0.1, 0.2], [], [0.3]))
        self.assertEqual(state.articulation.position_targets, [])
        self.assertEqual(state.articulation.velocity_targets, [])
        self.assertEqual(state.articulation.efforts, [])

    # ----------------------------------------------------------------------
    async def test_nan_command_filters_to_finite_entries(self) -> None:
        """Verify NaN command entries are skipped while finite entries are applied."""
        state = OgnIsaacArticulationControllerInternalState.__new__(OgnIsaacArticulationControllerInternalState)
        command = np.array([0.1, np.nan, 0.3])

        resolved_command, resolved_indices = state._filter_finite_command(command, [1, 2, 3])

        self.assertTrue(np.array_equal(resolved_command, np.array([0.1, 0.3])))
        self.assertTrue(np.array_equal(resolved_indices, np.array([1, 3])))

    # ----------------------------------------------------------------------
    async def test_full_array_no_index_ogn(self) -> None:
        """Verify full-width commands apply to all DOFs when no joint selection is provided."""
        test_graph, new_nodes, _, _ = og.Controller.edit(
            {"graph_path": "/ActionGraph", "evaluator_name": "execution"},
            {
                og.Controller.Keys.CREATE_NODES: [
                    ("OnPlaybackTick", "omni.graph.action.OnPlaybackTick"),
                    ("Joint1Position", "omni.graph.nodes.ConstantDouble"),
                    ("Joint2Position", "omni.graph.nodes.ConstantDouble"),
                    ("JointCommandArray", "omni.graph.nodes.ConstructArray"),
                    ("ArticulationController", "isaacsim.core.nodes.IsaacArticulationController"),
                ],
                og.Controller.Keys.CREATE_ATTRIBUTES: [
                    ("JointCommandArray.inputs:input1", "double"),
                    ("JointCommandArray.inputs:input2", "double"),
                ],
                og.Controller.Keys.SET_VALUES: [
                    ("Joint1Position.inputs:value", -1.0),
                    ("Joint2Position.inputs:value", 1.2),
                    ("JointCommandArray.inputs:arraySize", 9),
                    ("ArticulationController.inputs:robotPath", "/panda"),
                ],
                og.Controller.Keys.CONNECT: [
                    ("OnPlaybackTick.outputs:tick", "ArticulationController.inputs:execIn"),
                    ("Joint1Position.inputs:value", "JointCommandArray.inputs:input1"),
                    ("Joint2Position.inputs:value", "JointCommandArray.inputs:input2"),
                    ("JointCommandArray.outputs:array", "ArticulationController.inputs:positionCommand"),
                ],
            },
        )

        await og.Controller.evaluate(test_graph)

        # check where the joints are after evaluate
        robot = Articulation("/panda")
        app_utils.play()
        await app_utils.update_app_async(steps=120)

        self.assertAlmostEqual(robot.get_dof_positions().numpy()[0, 1], -1.0, delta=0.01)
        self.assertAlmostEqual(robot.get_dof_positions().numpy()[0, 2], 1.2, delta=0.003)

    # ----------------------------------------------------------------------
    async def test_single_joint_name_ogn(self) -> None:
        """Verify a one-element command applies to a single joint selected by name."""
        test_graph, new_nodes, _, _ = og.Controller.edit(
            {"graph_path": "/ActionGraph", "evaluator_name": "execution"},
            {
                og.Controller.Keys.CREATE_NODES: [
                    ("OnPlaybackTick", "omni.graph.action.OnPlaybackTick"),
                    ("Joint1Name", "omni.graph.nodes.ConstantToken"),
                    ("JointNameArray", "omni.graph.nodes.ConstructArray"),
                    ("Joint1Position", "omni.graph.nodes.ConstantDouble"),
                    ("JointCommandArray", "omni.graph.nodes.ConstructArray"),
                    ("ArticulationController", "isaacsim.core.nodes.IsaacArticulationController"),
                ],
                og.Controller.Keys.SET_VALUES: [
                    ("Joint1Name.inputs:value", "panda_joint3"),
                    ("Joint1Position.inputs:value", 1.7),
                    ("JointNameArray.inputs:arraySize", 1),
                    ("JointCommandArray.inputs:arraySize", 1),
                    ("ArticulationController.inputs:robotPath", "/panda"),
                ],
                og.Controller.Keys.CONNECT: [
                    ("OnPlaybackTick.outputs:tick", "ArticulationController.inputs:execIn"),
                    ("Joint1Name.inputs:value", "JointNameArray.inputs:input0"),
                    ("JointNameArray.outputs:array", "ArticulationController.inputs:jointNames"),
                    ("Joint1Position.inputs:value", "JointCommandArray.inputs:input0"),
                    ("JointCommandArray.outputs:array", "ArticulationController.inputs:positionCommand"),
                ],
            },
        )

        await og.Controller.evaluate(test_graph)

        # check where the joints are after evaluate
        robot = Articulation("/panda")
        app_utils.play()
        await app_utils.update_app_async(steps=120)

        self.assertAlmostEqual(robot.get_dof_positions().numpy()[0, 2], 1.7, delta=0.002)
        self.assertGreater(abs(robot.get_dof_positions().numpy()[0, 3] - 1.7), 0.1)

    # ----------------------------------------------------------------------
    async def test_single_joint_index_ogn(self) -> None:
        """Verify a one-element command applies to a single joint selected by index."""
        test_graph, new_nodes, _, _ = og.Controller.edit(
            {"graph_path": "/ActionGraph", "evaluator_name": "execution"},
            {
                og.Controller.Keys.CREATE_NODES: [
                    ("OnPlaybackTick", "omni.graph.action.OnPlaybackTick"),
                    ("Joint1Index", "omni.graph.nodes.ConstantInt"),
                    ("JointIndexArray", "omni.graph.nodes.ConstructArray"),
                    ("Joint1Position", "omni.graph.nodes.ConstantDouble"),
                    ("JointCommandArray", "omni.graph.nodes.ConstructArray"),
                    ("ArticulationController", "isaacsim.core.nodes.IsaacArticulationController"),
                ],
                og.Controller.Keys.SET_VALUES: [
                    ("Joint1Index.inputs:value", 2),
                    ("Joint1Position.inputs:value", 1.7),
                    ("JointIndexArray.inputs:arraySize", 1),
                    ("JointCommandArray.inputs:arraySize", 1),
                    ("ArticulationController.inputs:robotPath", "/panda"),
                ],
                og.Controller.Keys.CONNECT: [
                    ("OnPlaybackTick.outputs:tick", "ArticulationController.inputs:execIn"),
                    ("Joint1Index.inputs:value", "JointIndexArray.inputs:input0"),
                    ("JointIndexArray.outputs:array", "ArticulationController.inputs:jointIndices"),
                    ("Joint1Position.inputs:value", "JointCommandArray.inputs:input0"),
                    ("JointCommandArray.outputs:array", "ArticulationController.inputs:positionCommand"),
                ],
            },
        )

        await og.Controller.evaluate(test_graph)

        # check where the joints are after evaluate
        robot = Articulation("/panda")
        app_utils.play()
        await app_utils.update_app_async(steps=120)

        self.assertAlmostEqual(robot.get_dof_positions().numpy()[0, 2], 1.7, delta=0.002)
        self.assertGreater(abs(robot.get_dof_positions().numpy()[0, 3] - 1.7), 0.002)

    # ----------------------------------------------------------------------
    async def test_joint_indices_different_shape(self) -> None:
        """Verify differently shaped joint index inputs still select the intended DOFs."""
        test_graph, new_nodes, _, _ = og.Controller.edit(
            {"graph_path": "/ActionGraph", "evaluator_name": "execution"},
            {
                og.Controller.Keys.CREATE_NODES: [
                    ("OnPlaybackTick", "omni.graph.action.OnPlaybackTick"),
                    ("Joint1Index", "omni.graph.nodes.ConstantInt"),
                    ("JointIndexArray", "omni.graph.nodes.ConstructArray"),
                    ("Joint1Position", "omni.graph.nodes.ConstantDouble"),
                    ("JointCommandArray", "omni.graph.nodes.ConstructArray"),
                    ("ArticulationController", "isaacsim.core.nodes.IsaacArticulationController"),
                ],
                og.Controller.Keys.SET_VALUES: [
                    ("Joint1Index.inputs:value", 2),
                    ("Joint1Position.inputs:value", 1.7),
                    ("JointIndexArray.inputs:arraySize", 1),
                    ("JointCommandArray.inputs:arraySize", 1),
                    ("ArticulationController.inputs:robotPath", "/panda"),
                ],
                og.Controller.Keys.CONNECT: [
                    ("OnPlaybackTick.outputs:tick", "ArticulationController.inputs:execIn"),
                    ("Joint1Index.inputs:value", "JointIndexArray.inputs:input0"),
                    ("JointIndexArray.outputs:array", "ArticulationController.inputs:jointIndices"),
                    ("Joint1Position.inputs:value", "JointCommandArray.inputs:input0"),
                    ("JointCommandArray.outputs:array", "ArticulationController.inputs:positionCommand"),
                ],
            },
        )

        # Initialize robot
        robot = Articulation("/panda")
        app_utils.play()
        await og.Controller.evaluate(test_graph)
        await app_utils.update_app_async(steps=120)

        # Store initial joint positions
        initial_position = robot.get_dof_positions().numpy()[0, 2]

        # Access nodes directly by their paths
        articulation_controller_node = "/ActionGraph/ArticulationController"
        joint1_position_node = "/ActionGraph/Joint1Position"

        # Change the joint position value directly
        og.Controller.attribute("inputs:value", joint1_position_node).set(0.5)

        # Update the joint indices to have a different shape but same content
        current_indices = og.Controller.attribute("inputs:jointIndices", articulation_controller_node).get()
        # Reshape to 1D if it's not already
        reshaped_indices = current_indices.reshape(-1)
        og.Controller.attribute("inputs:jointIndices", articulation_controller_node).set(reshaped_indices)

        # Evaluate the graph again with the updated values
        await og.Controller.evaluate(test_graph)
        await app_utils.update_app_async(steps=120)

        new_position = robot.get_dof_positions().numpy()[0, 2]

        self.assertNotAlmostEqual(initial_position, new_position, delta=0.01)
        self.assertAlmostEqual(new_position, 0.5, delta=0.01)
