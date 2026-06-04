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

"""Tests for the HolonomicController OmniGraph node."""

import isaacsim.core.experimental.utils.app as app_utils
import isaacsim.core.experimental.utils.stage as stage_utils
import omni.graph.core as og
import omni.graph.core.tests as ogts
import omni.kit.test
from isaacsim.core.simulation_manager import SimulationManager


class TestHolonomicControllerOgn(ogts.OmniGraphTestCase):
    """Test class for validating the HolonomicController OmniGraph node.

    This class contains test cases that verify the functionality of the HolonomicController node
    within OmniGraph. It tests the node's ability to compute joint velocity commands for holonomic
    drive systems based on input velocity commands and wheel configuration parameters.

    The tests validate that the HolonomicController node correctly processes wheel radius, positions,
    orientations, and mecanum angles to generate appropriate joint velocity outputs. It also verifies
    the node's reset behavior when playback stops.
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
    async def test_holonomic_drive_ogn(self) -> None:
        """Verify the HolonomicController node computes correct joint velocity outputs from velocity inputs."""
        test_holo_graph, [holo_node, _, _, _, array_node], _, _ = og.Controller.edit(
            {"graph_path": "/ActionGraph"},
            {
                og.Controller.Keys.CREATE_NODES: [
                    ("HolonomicController", "isaacsim.robot.wheeled_robots.HolonomicController"),
                    ("XVelocity", "omni.graph.nodes.ConstantDouble"),
                    ("YVelocity", "omni.graph.nodes.ConstantDouble"),
                    ("Rotation", "omni.graph.nodes.ConstantDouble"),
                    ("VelocityCommands", "omni.graph.nodes.MakeVector3"),
                ],
                og.Controller.Keys.SET_VALUES: [
                    ("HolonomicController.inputs:wheelRadius", [0.04, 0.04, 0.04]),
                    (
                        "HolonomicController.inputs:wheelPositions",
                        [
                            [-0.0980432, 0.000636773, -0.050501],
                            [0.0493475, -0.084525, -0.050501],
                            [0.0495291, 0.0856937, -0.050501],
                        ],
                    ),
                    (
                        "HolonomicController.inputs:wheelOrientations",
                        [[0, 0, 0, 1], [0.866, 0, 0, -0.5], [0.866, 0, 0, 0.5]],
                    ),
                    ("HolonomicController.inputs:mecanumAngles", [90, 90, 90]),
                    ("XVelocity.inputs:value", 1.0),
                    ("YVelocity.inputs:value", 1.0),
                    ("Rotation.inputs:value", 0.1),
                ],
                og.Controller.Keys.CONNECT: [
                    ("XVelocity.inputs:value", "VelocityCommands.inputs:x"),
                    ("YVelocity.inputs:value", "VelocityCommands.inputs:y"),
                    ("Rotation.inputs:value", "VelocityCommands.inputs:z"),
                    ("VelocityCommands.outputs:tuple", "HolonomicController.inputs:inputVelocity"),
                ],
            },
        )

        await og.Controller.evaluate(test_holo_graph)
        vel = og.Controller(og.Controller.attribute("outputs:jointVelocityCommand", holo_node)).get()
        self.assertAlmostEqual(vel[0], -25.1053, delta=0.01)
        self.assertAlmostEqual(vel[1], 14.3182, delta=0.01)
        self.assertAlmostEqual(vel[2], -14.5417, delta=0.01)

    # ----------------------------------------------------------------------
    async def test_holonomic_drive_ogn_reset(self) -> None:
        """Verify joint velocities reset to zero when timeline stops."""
        test_holo_graph, [holo_node, _], _, _ = og.Controller.edit(
            {"graph_path": "/ActionGraph"},
            {
                og.Controller.Keys.CREATE_NODES: [
                    ("HolonomicController", "isaacsim.robot.wheeled_robots.HolonomicController"),
                    ("OnPlaybackTick", "omni.graph.action.OnPlaybackTick"),
                ],
                og.Controller.Keys.SET_VALUES: [
                    ("HolonomicController.inputs:wheelRadius", [0.04, 0.04, 0.04]),
                    (
                        "HolonomicController.inputs:wheelPositions",
                        [
                            [-0.0980432, 0.000636773, -0.050501],
                            [0.0493475, -0.084525, -0.050501],
                            [0.0495291, 0.0856937, -0.050501],
                        ],
                    ),
                    (
                        "HolonomicController.inputs:wheelOrientations",
                        [[0, 0, 0, 1], [0.866, 0, 0, -0.5], [0.866, 0, 0, 0.5]],
                    ),
                    ("HolonomicController.inputs:mecanumAngles", [90, 90, 90]),
                    ("HolonomicController.inputs:inputVelocity", [1.0, 1.0, 0.1]),
                ],
                og.Controller.Keys.CONNECT: [
                    ("OnPlaybackTick.outputs:tick", "HolonomicController.inputs:execIn"),
                ],
            },
        )

        app_utils.play()
        await app_utils.update_app_async()

        vel = og.Controller(og.Controller.attribute("outputs:jointVelocityCommand", holo_node)).get()
        self.assertAlmostEqual(vel[0], -25.1053, delta=0.01)
        self.assertAlmostEqual(vel[1], 14.3182, delta=0.01)
        self.assertAlmostEqual(vel[2], -14.5417, delta=0.01)

        app_utils.stop()
        await app_utils.update_app_async()

        self.assertEqual(
            og.Controller(og.Controller.attribute("outputs:jointVelocityCommand", holo_node)).get()[0], 0.0
        )
        self.assertEqual(
            og.Controller(og.Controller.attribute("outputs:jointVelocityCommand", holo_node)).get()[1], 0.0
        )
        self.assertEqual(
            og.Controller(og.Controller.attribute("outputs:jointVelocityCommand", holo_node)).get()[2], 0.0
        )
