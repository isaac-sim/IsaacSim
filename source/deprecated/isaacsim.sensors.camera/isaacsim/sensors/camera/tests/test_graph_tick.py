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

"""Tests for graph tick and SDG pipeline conflict handling."""

import omni.graph.core as og
import omni.kit.test
import omni.timeline
import omni.usd
from isaacsim.sensors.camera import Camera
from pxr import Gf, Sdf, UsdGeom


class TestGraphSDGPipelineConflict(omni.kit.test.AsyncTestCase):
    """Test cases for GraphSDGPipelineConflict."""

    async def setUp(self) -> None:
        """Set up test fixtures."""
        self._timeline = omni.timeline.get_timeline_interface()
        await omni.usd.get_context().new_stage_async()

    async def tearDown(self) -> None:
        """Tear down test fixtures."""
        self._timeline.stop()

    async def _create_and_initialize_camera(self, stage: object) -> "Camera":
        cam_path = "/World/Cam"
        camera_prim = stage.DefinePrim(cam_path, "Camera")
        xform = UsdGeom.XformCommonAPI(camera_prim)
        xform.SetTranslate(Gf.Vec3d(0, 0, 5))
        xform.SetRotate((0, 0, 0), UsdGeom.XformCommonAPI.RotationOrderXYZ)

        camera = Camera(prim_path=cam_path, name="cam", resolution=(64, 64))
        camera.set_focal_length(24.0)
        camera.set_focus_distance(400.0)
        camera.initialize()  # Attach annotator to camera
        return camera

    async def test_sdg_pipeline_conflict(self) -> None:
        """Test sdg pipeline conflict."""
        stage = omni.usd.get_context().get_stage()

        # Define test cube
        stage.DefinePrim("/Cube", "Cube")

        def make_graph(graph_path: str, cube_path: str) -> object:
            """Execute make_graph.

            Args:
                graph_path: The USD path for the OmniGraph to create.
                cube_path: The USD path to the cube prim used in the graph.

            Returns:
                The result of the OmniGraph controller edit operation.
            """
            return og.Controller.edit(
                {"graph_path": graph_path, "evaluator_name": "execution"},
                {
                    og.Controller.Keys.CREATE_NODES: [
                        ("on_tick", "omni.graph.action.OnTick"),
                        ("read_prim_attr", "omni.graph.nodes.ReadPrimAttribute"),
                        ("add", "omni.graph.nodes.Add"),
                        ("constant_float", "omni.graph.nodes.ConstantFloat"),
                        ("write_prim_attr", "omni.graph.nodes.WritePrimAttribute"),
                    ],
                    og.Controller.Keys.CONNECT: [
                        ("on_tick.outputs:tick", "write_prim_attr.inputs:execIn"),
                        ("read_prim_attr.outputs:value", "add.inputs:a"),
                        ("constant_float.inputs:value", "add.inputs:b"),
                        ("add.outputs:sum", "write_prim_attr.inputs:value"),
                    ],
                    og.Controller.Keys.SET_VALUES: [
                        ("on_tick.inputs:onlyPlayback", True),
                        ("read_prim_attr.inputs:prim", Sdf.Path(cube_path)),
                        ("read_prim_attr.inputs:name", "size"),
                        ("write_prim_attr.inputs:prim", Sdf.Path(cube_path)),
                        ("write_prim_attr.inputs:name", "size"),
                        ("constant_float.inputs:value", 5.0),
                    ],
                },
            )

        make_graph("/TestGraph", "/Cube")

        self._timeline.play()

        # Test one graph trigger
        await omni.kit.app.get_app().next_update_async()
        cube_val = stage.GetAttributeAtPath("/Cube.size").Get()
        self.assertEqual(cube_val, 7.0, f"Expected Cube.size = 7.0, got {cube_val}")
        # Create and initialize camera
        await self._create_and_initialize_camera(stage)
        # For each app update we expect the cube size to increase by 5
        for i in range(3):
            await omni.kit.app.get_app().next_update_async()
            print(f"Iteration {i+1}")
            cube_val_after = stage.GetAttributeAtPath("/Cube.size").Get()
            self.assertEqual(
                cube_val_after,
                7.0 + 5 * (i + 1),
                f"Expected Cube.size to stay at {7.0+5*(i+1)}, but got {cube_val_after}",
            )
