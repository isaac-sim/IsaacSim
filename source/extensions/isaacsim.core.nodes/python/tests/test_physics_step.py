# SPDX-FileCopyrightText: Copyright (c) 2024-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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
import omni.kit.test
from isaacsim.core.simulation_manager import SimulationManager
from pxr import Sdf


class TestPhysicsStep(omni.kit.test.AsyncTestCase):
    async def setUp(self):
        """Set up  test environment, to be torn down when done."""
        self._timeline = omni.timeline.get_timeline_interface()
        await omni.usd.get_context().new_stage_async()

    # ----------------------------------------------------------------------
    async def tearDown(self):
        """Get rid of temporary data used by the test."""
        await omni.kit.stage_templates.new_stage_async()

    async def test_physics_step_node(self):
        carb.settings.get_settings().set_bool("/app/player/useFixedTimeStepping", True)

        stage = omni.usd.get_context().get_stage()

        stage.DefinePrim("/Cube", "Cube")
        keys = og.Controller.Keys
        self._clock_graph, _, _, _ = og.Controller.edit(
            {
                "graph_path": "/physics_step",
                "pipeline_stage": og.GraphPipelineStage.GRAPH_PIPELINE_STAGE_ONDEMAND,
            },
            {
                keys.CREATE_NODES: [
                    ("physics_step", "isaacsim.core.nodes.OnPhysicsStep"),
                    ("read_prim_attr", "omni.graph.nodes.ReadPrimAttribute"),
                    ("constant_float", "omni.graph.nodes.ConstantFloat"),
                    ("add", "omni.graph.nodes.Add"),
                    ("write_prim_attr", "omni.graph.nodes.WritePrimAttribute"),
                ],
                keys.CONNECT: [
                    ("physics_step.outputs:step", "write_prim_attr.inputs:execIn"),
                    ("read_prim_attr.outputs:value", "add.inputs:a"),
                    ("constant_float.inputs:value", "add.inputs:b"),
                    ("add.outputs:sum", "write_prim_attr.inputs:value"),
                ],
                keys.SET_VALUES: [
                    ("read_prim_attr.inputs:prim", Sdf.Path("/Cube")),
                    ("read_prim_attr.inputs:name", "size"),
                    ("write_prim_attr.inputs:prim", Sdf.Path("/Cube")),
                    ("write_prim_attr.inputs:name", "size"),
                    ("constant_float.inputs:value", 1.0),
                ],
            },
        )
        steps = SimulationManager.get_num_physics_steps()
        self.assertEqual(steps, 0)
        self._timeline.play()
        for i in range(10):
            await omni.kit.app.get_app().next_update_async()
            # Check according to the SimulationManager.get_num_physics_steps that the cube size grew by the same size as the number of steps.
            steps = SimulationManager.get_num_physics_steps()
            cube_size = stage.GetAttributeAtPath("/Cube.size").Get()
            self.assertEqual(cube_size, 2.0 + steps)

        self._timeline.stop()

    async def test_physics_step_two_nodes(self):
        """Verify two OnPhysicsStep nodes in one graph share a single subscription and both fire."""
        carb.settings.get_settings().set_bool("/app/player/useFixedTimeStepping", True)

        stage = omni.usd.get_context().get_stage()
        stage.DefinePrim("/CubeA", "Cube")
        stage.DefinePrim("/CubeB", "Cube")

        keys = og.Controller.Keys
        og.Controller.edit(
            {
                "graph_path": "/physics_step_dual",
                "pipeline_stage": og.GraphPipelineStage.GRAPH_PIPELINE_STAGE_ONDEMAND,
            },
            {
                keys.CREATE_NODES: [
                    ("physics_step_a", "isaacsim.core.nodes.OnPhysicsStep"),
                    ("read_a", "omni.graph.nodes.ReadPrimAttribute"),
                    ("const_a", "omni.graph.nodes.ConstantFloat"),
                    ("add_a", "omni.graph.nodes.Add"),
                    ("write_a", "omni.graph.nodes.WritePrimAttribute"),
                    ("physics_step_b", "isaacsim.core.nodes.OnPhysicsStep"),
                    ("read_b", "omni.graph.nodes.ReadPrimAttribute"),
                    ("const_b", "omni.graph.nodes.ConstantFloat"),
                    ("add_b", "omni.graph.nodes.Add"),
                    ("write_b", "omni.graph.nodes.WritePrimAttribute"),
                ],
                keys.CONNECT: [
                    ("physics_step_a.outputs:step", "write_a.inputs:execIn"),
                    ("read_a.outputs:value", "add_a.inputs:a"),
                    ("const_a.inputs:value", "add_a.inputs:b"),
                    ("add_a.outputs:sum", "write_a.inputs:value"),
                    ("physics_step_b.outputs:step", "write_b.inputs:execIn"),
                    ("read_b.outputs:value", "add_b.inputs:a"),
                    ("const_b.inputs:value", "add_b.inputs:b"),
                    ("add_b.outputs:sum", "write_b.inputs:value"),
                ],
                keys.SET_VALUES: [
                    ("read_a.inputs:prim", Sdf.Path("/CubeA")),
                    ("read_a.inputs:name", "size"),
                    ("write_a.inputs:prim", Sdf.Path("/CubeA")),
                    ("write_a.inputs:name", "size"),
                    ("const_a.inputs:value", 1.0),
                    ("read_b.inputs:prim", Sdf.Path("/CubeB")),
                    ("read_b.inputs:name", "size"),
                    ("write_b.inputs:prim", Sdf.Path("/CubeB")),
                    ("write_b.inputs:name", "size"),
                    ("const_b.inputs:value", 1.0),
                ],
            },
        )

        self._timeline.play()
        for _ in range(10):
            await omni.kit.app.get_app().next_update_async()

        steps = SimulationManager.get_num_physics_steps()
        size_a = stage.GetAttributeAtPath("/CubeA.size").Get()
        size_b = stage.GetAttributeAtPath("/CubeB.size").Get()
        self.assertEqual(size_a, 2.0 + steps)
        self.assertEqual(size_b, 2.0 + steps)

        self._timeline.stop()
