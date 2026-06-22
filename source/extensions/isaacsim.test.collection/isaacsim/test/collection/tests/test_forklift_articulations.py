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

"""Test module for forklift robot articulation and movement functionality."""

import carb
import isaacsim.core.experimental.utils.app as app_utils
import isaacsim.core.experimental.utils.stage as stage_utils
import omni.graph.core as og
import omni.kit.test
import omni.timeline
import usdrt.Sdf
from isaacsim.core.experimental.prims import RigidPrim
from isaacsim.core.experimental.utils.prim import get_prim_at_path
from isaacsim.storage.native import get_assets_root_path_async


class TestForkliftArticulations(omni.kit.test.AsyncTestCase):
    """Tests for forklift robot articulation and movement."""

    # Before running each test
    async def setUp(self) -> None:
        """Set up test environment with forklift robot and action graph."""
        await stage_utils.create_new_stage_async()
        self._timeline = omni.timeline.get_timeline_interface()

        self._assets_root_path = await get_assets_root_path_async()
        if self._assets_root_path is None:
            carb.log_error("Could not find Isaac Sim assets folder")
            return

        self.usd_path = self._assets_root_path + "/Isaac/Robots/IsaacSim/ForkliftC/forklift_c.usd"
        stage_utils.add_reference_to_stage(self.usd_path, "/World/Forklift")
        self.stage = omni.usd.get_context().get_stage()

        usd_path = self._assets_root_path + "/Isaac/Environments/Grid/default_environment.usd"
        stage_utils.add_reference_to_stage(usd_path, "/World/defaultGroundPlane")

        await omni.kit.app.get_app().next_update_async()

        # Set stage units
        stage_utils.set_stage_units(meters_per_unit=1.0)
        await app_utils.update_app_async()

        self.graph_path = "/ActionGraph"

        if get_prim_at_path(self.graph_path):
            stage_utils.delete_prim(self.graph_path)

        keys = og.Controller.Keys
        og.Controller.edit(
            {"graph_path": self.graph_path, "evaluator_name": "execution"},
            {
                keys.CREATE_NODES: [
                    ("OnPlaybackTick", "omni.graph.action.OnPlaybackTick"),
                    ("ArticulationController", "isaacsim.core.nodes.IsaacArticulationController"),
                    ("WritePrimAttributeLeft", "omni.graph.nodes.WritePrimAttribute"),
                    ("WritePrimAttributeRight", "omni.graph.nodes.WritePrimAttribute"),
                    ("WritePrimAttributeLift", "omni.graph.nodes.WritePrimAttribute"),
                    ("SteeringAngle", "omni.graph.nodes.ConstantDouble"),
                    ("LiftPosition", "omni.graph.nodes.ConstantDouble"),
                ],
                keys.CONNECT: [
                    ("OnPlaybackTick.outputs:tick", "ArticulationController.inputs:execIn"),
                    ("OnPlaybackTick.outputs:tick", "WritePrimAttributeLeft.inputs:execIn"),
                    ("OnPlaybackTick.outputs:tick", "WritePrimAttributeRight.inputs:execIn"),
                    ("OnPlaybackTick.outputs:tick", "WritePrimAttributeLift.inputs:execIn"),
                    ("SteeringAngle.inputs:value", "WritePrimAttributeLeft.inputs:value"),
                    ("SteeringAngle.inputs:value", "WritePrimAttributeRight.inputs:value"),
                    ("LiftPosition.inputs:value", "WritePrimAttributeLift.inputs:value"),
                ],
                keys.SET_VALUES: [
                    ("SteeringAngle.inputs:value", 0.0),
                    ("LiftPosition.inputs:value", 0.0),
                    (
                        "WritePrimAttributeLeft.inputs:prim",
                        [usdrt.Sdf.Path("/World/Forklift/left_rotator_joint")],
                    ),
                    ("WritePrimAttributeLeft.inputs:name", "drive:angular:physics:targetPosition"),
                    (
                        "WritePrimAttributeRight.inputs:prim",
                        [usdrt.Sdf.Path("/World/Forklift/right_rotator_joint")],
                    ),
                    ("WritePrimAttributeRight.inputs:name", "drive:angular:physics:targetPosition"),
                    (
                        "WritePrimAttributeLift.inputs:prim",
                        [usdrt.Sdf.Path("/World/Forklift/lift_joint")],
                    ),
                    ("WritePrimAttributeLift.inputs:name", "drive:linear:physics:targetPosition"),
                    ("ArticulationController.inputs:robotPath", "/World/Forklift"),
                    ("ArticulationController.inputs:velocityCommand", [0.0, 0.0]),
                    (
                        "ArticulationController.inputs:jointNames",
                        [
                            "left_back_wheel_joint",
                            "right_back_wheel_joint",
                        ],
                    ),
                ],
            },
        )

    # After running each test
    async def tearDown(self) -> None:
        """Clean up test environment and stop timeline."""
        self._timeline.stop()
        await omni.kit.app.get_app().next_update_async()
        while omni.usd.get_context().get_stage_loading_status()[2] > 0:
            await omni.kit.app.get_app().next_update_async()

    async def test_forklift_forward(self) -> None:
        """Test forklift moves forward when positive wheel velocity is applied."""
        body_prim = RigidPrim("/World/Forklift/body")

        og.Controller.attribute(self.graph_path + "/ArticulationController.inputs:velocityCommand").set([5.0, 5.0])

        # start the timeline
        self._timeline.play()
        await omni.kit.app.get_app().next_update_async()
        positions, _ = body_prim.get_world_poses()
        pos = positions.numpy()[0]

        # wait for 200 frames
        for _ in range(200):
            await omni.kit.app.get_app().next_update_async()

        new_positions, _ = body_prim.get_world_poses()
        new_pos = new_positions.numpy()[0]

        # check if the forklift moved
        self.assertNotAlmostEqual(pos[0], new_pos[0], delta=1)
        self.assertGreater(new_pos[0], pos[0])
        self.assertAlmostEqual(pos[1], new_pos[1], delta=1)
        self.assertAlmostEqual(pos[2], new_pos[2], delta=1)

    async def test_forklift_reverse(self) -> None:
        """Test forklift moves backward when negative wheel velocity is applied."""
        body_prim = RigidPrim("/World/Forklift/body")

        og.Controller.attribute(self.graph_path + "/ArticulationController.inputs:velocityCommand").set([-5.0, -5.0])

        # start the timeline
        self._timeline.play()
        await omni.kit.app.get_app().next_update_async()
        positions, _ = body_prim.get_world_poses()
        pos = positions.numpy()[0]

        # wait for 200 frames
        for _ in range(200):
            await omni.kit.app.get_app().next_update_async()

        new_positions, _ = body_prim.get_world_poses()
        new_pos = new_positions.numpy()[0]

        # check if the forklift moved
        self.assertNotAlmostEqual(pos[0], new_pos[0], delta=1)
        self.assertLess(new_pos[0], pos[0])
        self.assertAlmostEqual(pos[1], new_pos[1], delta=1)
        self.assertAlmostEqual(pos[2], new_pos[2], delta=1)

    async def test_forklift_reverse_turn(self) -> None:
        """Test forklift turns while reversing with steering angle applied."""
        body_prim = RigidPrim("/World/Forklift/body")

        og.Controller.attribute(self.graph_path + "/ArticulationController.inputs:velocityCommand").set([-5.0, -5.0])
        og.Controller.attribute(self.graph_path + "/SteeringAngle.inputs:value").set(20.0)

        # start the timeline
        self._timeline.play()
        await omni.kit.app.get_app().next_update_async()
        positions, _ = body_prim.get_world_poses()
        pos = positions.numpy()[0]

        # wait for 200 frames
        for _ in range(200):
            await omni.kit.app.get_app().next_update_async()

        new_positions, _ = body_prim.get_world_poses()
        new_pos = new_positions.numpy()[0]

        # check if the forklift moved
        self.assertNotAlmostEqual(pos[0], new_pos[0], delta=1)
        self.assertNotAlmostEqual(pos[1], new_pos[1], delta=1)
        self.assertAlmostEqual(pos[2], new_pos[2], delta=1)

    async def test_forklift_lift(self) -> None:
        """Test forklift lift mechanism raises when position target is set."""
        lift_prim = RigidPrim("/World/Forklift/lift")

        og.Controller.attribute(self.graph_path + "/LiftPosition.inputs:value").set(1.0)

        # start the timeline
        self._timeline.play()
        await omni.kit.app.get_app().next_update_async()
        positions, _ = lift_prim.get_world_poses()
        pos = positions.numpy()[0]

        # wait for 60 frames
        for _ in range(60):
            await omni.kit.app.get_app().next_update_async()

        new_positions, _ = lift_prim.get_world_poses()
        new_pos = new_positions.numpy()[0]

        # check if the forklift moved
        self.assertAlmostEqual(pos[0], new_pos[0], delta=1)
        self.assertAlmostEqual(pos[1], new_pos[1], delta=1)
        self.assertNotAlmostEqual(pos[2], new_pos[2], delta=0.5)
