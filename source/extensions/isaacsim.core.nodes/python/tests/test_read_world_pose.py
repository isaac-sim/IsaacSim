# SPDX-FileCopyrightText: Copyright (c) 2018-2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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

import carb
import omni.graph.core as og
import omni.graph.core.tests as ogts
import omni.kit.test
import omni.usd
from pxr import Gf, UsdGeom
from usdrt import Sdf


class TestIsaacReadWorldPose(ogts.OmniGraphTestCase):

    async def setUp(self):
        await omni.usd.get_context().new_stage_async()
        self._stage = omni.usd.get_context().get_stage()
        self._timeline = omni.timeline.get_timeline_interface()

        # Create cube and translate
        self._cube_path = "/World/Cube"
        cube = UsdGeom.Cube.Define(self._stage, self._cube_path)
        # Set world transform
        translate_op = cube.AddTranslateOp()
        translate_op.Set(Gf.Vec3d(1.0, 2.0, 3.0))
        await omni.kit.app.get_app().next_update_async()

    async def tearDown(self):
        await omni.kit.app.get_app().next_update_async()
        while omni.usd.get_context().get_stage_loading_status()[2] > 0:
            print("tearDown, assets still loading, waiting to finish...")
            await asyncio.sleep(1.0)
        await omni.kit.app.get_app().next_update_async()
        return

    async def test_pose_outputs_correct_values(self):
        graph_path = "/ActionGraph"
        node_name = "readWorldPoseNode"

        (graph, nodes, _, _) = og.Controller.edit(
            {"graph_path": graph_path, "evaluator_name": "push"},
            {
                og.Controller.Keys.CREATE_NODES: [
                    (node_name, "isaacsim.core.nodes.IsaacReadWorldPose"),
                ],
                og.Controller.Keys.SET_VALUES: [
                    (f"{node_name}.inputs:prim", [Sdf.Path(self._cube_path)]),
                ],
            },
        )

        await omni.kit.app.get_app().next_update_async()
        await og.Controller.evaluate(graph_path)

        # Run simulation
        self._timeline.play()
        await omni.kit.app.get_app().next_update_async()
        await omni.kit.app.get_app().next_update_async()

        # Read outputs
        translation = og.Controller.get(f"{graph_path}/{node_name}.outputs:translation")
        orientation = og.Controller.get(f"{graph_path}/{node_name}.outputs:orientation")
        scale = og.Controller.get(f"{graph_path}/{node_name}.outputs:scale")

        # Check translation
        expected_translation = [1.0, 2.0, 3.0]
        for a, b in zip(translation, expected_translation):
            self.assertAlmostEqual(a, b, places=3, msg=f"Translation mismatch: {translation} vs {expected_translation}")

        # Check orientation is identity quaternion
        expected_orientation = [0.0, 0.0, 0.0, 1.0]
        for a, b in zip(orientation, expected_orientation):
            self.assertAlmostEqual(a, b, places=3, msg=f"Orientation mismatch: {orientation} vs {expected_orientation}")

        # Check scale is default 1.0x1.0x1.0 for Cube
        expected_scale = [1.0, 1.0, 1.0]
        for a, b in zip(scale, expected_scale):
            self.assertAlmostEqual(a, b, places=3, msg=f"Scale mismatch: {scale} vs {expected_scale}")
