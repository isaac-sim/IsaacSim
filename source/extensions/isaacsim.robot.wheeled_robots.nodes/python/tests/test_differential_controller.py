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

"""Unit tests for the DifferentialController OmniGraph node."""

import carb
import isaacsim.core.experimental.utils.app as app_utils
import isaacsim.core.experimental.utils.stage as stage_utils
import omni.graph.core as og
import omni.graph.core.tests as ogts
import omni.kit.test
from isaacsim.core.experimental.prims import Articulation
from isaacsim.core.simulation_manager import SimulationManager
from isaacsim.storage.native import get_assets_root_path_async


class TestDifferentialControllerNode(ogts.OmniGraphTestCase):
    """Test class for the differential controller node in Omni Graph.

    This test class validates the behavior and functionality of the DifferentialController node
    within the Omni Graph framework. It tests the node's ability to convert linear and angular
    velocity commands into individual wheel velocity commands for differential drive robots.

    The tests cover various scenarios including basic differential drive calculations,
    acceleration limits, reset functionality, and integration with robot articulations.
    Each test creates an action graph with the DifferentialController node and verifies
    the output velocity commands match expected values based on input parameters like
    wheel radius, wheel distance, and desired velocities.
    """

    async def setUp(self) -> None:
        """Set up test environment, to be torn down when done."""
        await stage_utils.create_new_stage_async()
        stage_utils.set_stage_up_axis("Z")
        stage_utils.set_stage_units(meters_per_unit=1.0)
        await app_utils.update_app_async()
        SimulationManager.setup_simulation(dt=1.0 / 60.0)

    # ----------------------------------------------------------------------
    async def tearDown(self) -> None:
        """Get rid of temporary data used by the test."""
        app_utils.stop()
        await app_utils.update_app_async()
        await omni.kit.stage_templates.new_stage_async()

    # ----------------------------------------------------------------------
    async def test_differential_controller_node(self) -> None:
        """Test basic differential controller node functionality with velocity commands."""
        test_diff_graph, [play_node, diff_node], _, _ = og.Controller.edit(
            {"graph_path": "/ActionGraph", "evaluator_name": "execution"},
            {
                og.Controller.Keys.CREATE_NODES: [
                    ("OnPlaybackTick", "omni.graph.action.OnPlaybackTick"),
                    ("DifferentialController", "isaacsim.robot.wheeled_robots.DifferentialController"),
                ],
                og.Controller.Keys.CONNECT: [
                    ("OnPlaybackTick.outputs:tick", "DifferentialController.inputs:execIn"),
                ],
                og.Controller.Keys.SET_VALUES: [
                    ("DifferentialController.inputs:wheelRadius", 0.03),
                    ("DifferentialController.inputs:wheelDistance", 0.1125),
                    ("DifferentialController.inputs:linearVelocity", 0.3),
                    ("DifferentialController.inputs:angularVelocity", 1.0),
                ],
            },
        )

        app_utils.play()
        await app_utils.update_app_async()
        await app_utils.update_app_async(steps=3)
        self.assertEqual(og.Controller(og.Controller.attribute("outputs:velocityCommand", diff_node)).get()[0], 8.125)
        self.assertEqual(og.Controller(og.Controller.attribute("outputs:velocityCommand", diff_node)).get()[1], 11.875)

    async def test_differential_controller_node_acceleration_limits(self) -> None:
        """Test differential controller node with acceleration and deceleration limits applied."""
        test_diff_graph, [play_node, diff_node], _, _ = og.Controller.edit(
            {"graph_path": "/ActionGraph", "evaluator_name": "execution"},
            {
                og.Controller.Keys.CREATE_NODES: [
                    ("OnPlaybackTick", "omni.graph.action.OnPlaybackTick"),
                    ("DifferentialController", "isaacsim.robot.wheeled_robots.DifferentialController"),
                ],
                og.Controller.Keys.CONNECT: [
                    ("OnPlaybackTick.outputs:tick", "DifferentialController.inputs:execIn"),
                ],
                og.Controller.Keys.SET_VALUES: [
                    ("DifferentialController.inputs:wheelRadius", 0.03),
                    ("DifferentialController.inputs:wheelDistance", 0.1125),
                    ("DifferentialController.inputs:linearVelocity", 0.3),
                    ("DifferentialController.inputs:angularVelocity", 1.0),
                    ("DifferentialController.inputs:maxAcceleration", 1.0),
                    ("DifferentialController.inputs:maxDeceleration", 1.0),
                    ("DifferentialController.inputs:dt", 1 / 60),
                ],
            },
        )

        app_utils.play()
        await app_utils.update_app_async()
        await app_utils.update_app_async(steps=3)

        self.assertLess(og.Controller(og.Controller.attribute("outputs:velocityCommand", diff_node)).get()[0], 8.125)
        self.assertLess(og.Controller(og.Controller.attribute("outputs:velocityCommand", diff_node)).get()[1], 11.875)

    async def test_differential_controller_node_reset(self) -> None:
        """Test differential controller node reset behavior when timeline is stopped and restarted."""
        test_diff_graph, [play_node, diff_node], _, _ = og.Controller.edit(
            {"graph_path": "/ActionGraph", "evaluator_name": "execution"},
            {
                og.Controller.Keys.CREATE_NODES: [
                    ("OnPlaybackTick", "omni.graph.action.OnPlaybackTick"),
                    ("DifferentialController", "isaacsim.robot.wheeled_robots.DifferentialController"),
                ],
                og.Controller.Keys.CONNECT: [
                    ("OnPlaybackTick.outputs:tick", "DifferentialController.inputs:execIn"),
                ],
                og.Controller.Keys.SET_VALUES: [
                    ("DifferentialController.inputs:wheelRadius", 0.03),
                    ("DifferentialController.inputs:wheelDistance", 0.1125),
                    ("DifferentialController.inputs:linearVelocity", 0.3),
                    ("DifferentialController.inputs:angularVelocity", 1.0),
                ],
            },
        )

        app_utils.play()
        await app_utils.update_app_async()
        await app_utils.update_app_async(steps=3)

        self.assertEqual(og.Controller(og.Controller.attribute("outputs:velocityCommand", diff_node)).get()[0], 8.125)
        self.assertEqual(og.Controller(og.Controller.attribute("outputs:velocityCommand", diff_node)).get()[1], 11.875)

        app_utils.stop()
        await app_utils.update_app_async()
        await app_utils.update_app_async()

        await app_utils.update_app_async(steps=3)

        self.assertEqual(og.Controller(og.Controller.attribute("inputs:linearVelocity", diff_node)).get(), 0.0)
        self.assertEqual(og.Controller(og.Controller.attribute("inputs:angularVelocity", diff_node)).get(), 0.0)

    async def test_differential_controller_reset_with_robot(self) -> None:
        """Test differential controller reset functionality with a Jetbot robot integration."""
        await stage_utils.create_new_stage_async()
        stage_utils.set_stage_up_axis("Z")
        stage_utils.set_stage_units(meters_per_unit=1.0)
        await app_utils.update_app_async()
        SimulationManager.setup_simulation(dt=1.0 / 60.0)

        assets_root_path = await get_assets_root_path_async()
        if assets_root_path is None:
            carb.log_error("Could not find Isaac Sim assets folder")
            return
        await stage_utils.open_stage_async(assets_root_path + "/Isaac/Robots/NVIDIA/Jetbot/jetbot.usd")

        test_graph, [play_node, diff_node, art_node], _, _ = og.Controller.edit(
            {"graph_path": "/ActionGraph", "evaluator_name": "execution"},
            {
                og.Controller.Keys.CREATE_NODES: [
                    ("OnPlaybackTick", "omni.graph.action.OnPlaybackTick"),
                    ("DifferentialController", "isaacsim.robot.wheeled_robots.DifferentialController"),
                    ("ArticulationController", "isaacsim.core.nodes.IsaacArticulationController"),
                ],
                og.Controller.Keys.CONNECT: [
                    ("OnPlaybackTick.outputs:tick", "DifferentialController.inputs:execIn"),
                    ("OnPlaybackTick.outputs:tick", "ArticulationController.inputs:execIn"),
                    ("DifferentialController.outputs:velocityCommand", "ArticulationController.inputs:velocityCommand"),
                ],
                og.Controller.Keys.SET_VALUES: [
                    ("DifferentialController.inputs:wheelRadius", 0.03),
                    ("DifferentialController.inputs:wheelDistance", 0.1125),
                    ("DifferentialController.inputs:linearVelocity", 0.3),
                    ("DifferentialController.inputs:angularVelocity", 1.0),
                    ("ArticulationController.inputs:robotPath", "/jetbot"),
                ],
            },
        )

        articulation = Articulation("/jetbot")
        app_utils.play()
        await app_utils.update_app_async(steps=60)
        await app_utils.update_app_async(steps=60)
        joint_vel_1 = articulation.get_dof_velocities().numpy()[0]

        self.assertNotEqual(joint_vel_1[0], 0)
        self.assertNotEqual(joint_vel_1[1], 0)

        app_utils.stop()
        await app_utils.update_app_async(steps=60)

        app_utils.play()
        await app_utils.update_app_async(steps=60)
        await app_utils.update_app_async(steps=60)

        joint_vel_3 = articulation.get_dof_velocities().numpy()[0]
        self.assertAlmostEqual(joint_vel_3[0], 0, delta=0.01)
        self.assertAlmostEqual(joint_vel_3[1], 0, delta=0.01)
