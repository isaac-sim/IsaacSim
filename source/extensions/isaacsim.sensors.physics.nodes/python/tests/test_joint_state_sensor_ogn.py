# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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

import isaacsim.core.experimental.utils.prim as prim_utils
import isaacsim.core.experimental.utils.stage as stage_utils
import omni.graph.core as og
import omni.kit.test
import omni.timeline
import usdrt.Sdf
from isaacsim.core.simulation_manager import SimulationManager
from isaacsim.storage.native import get_assets_root_path

from .common import step_simulation


class TestJointStateSensorOgn(omni.kit.test.AsyncTestCase):
    async def setUp(self):
        await stage_utils.create_new_stage_async()
        physics_rate = 60
        SimulationManager.setup_simulation(dt=1.0 / physics_rate)
        self._timeline = omni.timeline.get_timeline_interface()
        await self.setUp_environment()
        await self.setup_ogn()

    async def tearDown(self):
        if self._timeline.is_playing():
            self._timeline.stop()
        SimulationManager.invalidate_physics()
        await omni.kit.app.get_app().next_update_async()
        while omni.usd.get_context().get_stage_loading_status()[2] > 0:
            await asyncio.sleep(1.0)
        await omni.kit.app.get_app().next_update_async()

    async def setUp_environment(self):
        assets_root_path = get_assets_root_path()
        asset_path = assets_root_path + "/Isaac/Robots/IsaacSim/SimpleArticulation/simple_articulation.usd"
        stage_utils.add_reference_to_stage(usd_path=asset_path, path="/Articulation")

    async def setup_ogn(self):
        self.graph_path = "/TestGraph"
        self._node_name = "ReadJointStateNode"

        if prim_utils.get_prim_at_path(self.graph_path).IsValid():
            stage_utils.delete_prim(self.graph_path)

        keys = og.Controller.Keys
        try:
            _, new_nodes, _, _ = og.Controller.edit(
                {"graph_path": self.graph_path, "evaluator_name": "execution"},
                {
                    keys.CREATE_NODES: [
                        ("OnPlaybackTick", "omni.graph.action.OnPlaybackTick"),
                        (self._node_name, "isaacsim.sensors.physics.IsaacReadJointState"),
                    ],
                    keys.CONNECT: [
                        ("OnPlaybackTick.outputs:tick", self._node_name + ".inputs:execIn"),
                    ],
                },
            )
            # Use actual node path from Controller.edit so attribute lookup works (handles template naming etc.)
            if new_nodes and len(new_nodes) >= 2:
                second = new_nodes[1]
                self._node_path = second.path if hasattr(second, "path") else str(second)
            else:
                self._node_path = self.graph_path + "/" + self._node_name
        except Exception as e:
            print(e)
            self._node_path = self.graph_path + "/" + self._node_name

    async def test_valid_joint_state_sensor_ogn(self):
        """With articulation prim set, outputs have valid joint names, positions, velocities, efforts, dof types, and sensor time."""
        og.Controller.set(
            og.Controller.attribute(self.graph_path + "/ReadJointStateNode.inputs:prim"),
            [usdrt.Sdf.Path("/Articulation")],
        )
        self._timeline.play()
        await omni.kit.app.get_app().next_update_async()
        await step_simulation(0.5)

        joint_names = og.Controller.attribute(self.graph_path + "/ReadJointStateNode.outputs:jointNames").get()
        joint_positions = og.Controller.attribute(self.graph_path + "/ReadJointStateNode.outputs:jointPositions").get()
        joint_velocities = og.Controller.attribute(
            self.graph_path + "/ReadJointStateNode.outputs:jointVelocities"
        ).get()
        joint_efforts = og.Controller.attribute(self.graph_path + "/ReadJointStateNode.outputs:jointEfforts").get()
        joint_dof_types = og.Controller.attribute(self.graph_path + "/ReadJointStateNode.outputs:jointDofTypes").get()
        stage_meters_per_unit = og.Controller.attribute(
            self.graph_path + "/ReadJointStateNode.outputs:stageMetersPerUnit"
        ).get()
        sensor_time = og.Controller.attribute(self.graph_path + "/ReadJointStateNode.outputs:sensorTime").get()

        self.assertGreater(len(joint_names), 0, "Should have at least one joint name")
        n = len(joint_names)
        self.assertEqual(len(joint_positions), n, "jointPositions length should match joint count")
        self.assertEqual(len(joint_velocities), n, "jointVelocities length should match joint count")
        self.assertEqual(len(joint_efforts), n, "jointEfforts length should match joint count")
        self.assertEqual(len(joint_dof_types), n, "jointDofTypes length should match joint count")
        self.assertGreater(sensor_time, 0.0, "sensorTime should be positive after stepping")
        self.assertGreater(stage_meters_per_unit, 0.0, "stageMetersPerUnit should be positive")

    async def test_invalid_joint_state_sensor_ogn(self):
        """Outputs are empty and sensorTime is zero when no prim is set."""
        self._timeline.play()
        await step_simulation(0.5)

        joint_names = og.Controller.attribute(self.graph_path + "/ReadJointStateNode.outputs:jointNames").get()
        sensor_time = og.Controller.attribute(self.graph_path + "/ReadJointStateNode.outputs:sensorTime").get()

        self.assertEqual(len(joint_names), 0, "Should have no joint names when prim is not set")
        self.assertEqual(sensor_time, 0.0, "sensorTime should be zero when prim is not set")
