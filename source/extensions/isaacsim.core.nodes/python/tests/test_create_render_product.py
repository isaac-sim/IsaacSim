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

import carb
import isaacsim.core.experimental.utils.stage as stage_utils
import omni.graph.core as og
import omni.graph.core.tests as ogts
import omni.kit.test
import omni.replicator.core as rep
from isaacsim.storage.native import get_assets_root_path_async
from pxr import UsdRender


class TestCreateRenderProduct(ogts.OmniGraphTestCase):
    async def setUp(self):
        """Set up  test environment, to be torn down when done."""
        await omni.usd.get_context().new_stage_async()
        self._timeline = omni.timeline.get_timeline_interface()
        # add franka robot for test
        assets_root_path = await get_assets_root_path_async()
        if assets_root_path is None:
            carb.log_error("Could not find Isaac Sim assets folder")
            return
        await stage_utils.open_stage_async(assets_root_path + "/Isaac/Robots/FrankaRobotics/FrankaPanda/franka.usd")

    # ----------------------------------------------------------------------
    async def tearDown(self):
        """Get rid of temporary data used by the test."""
        # await omni.kit.stage_templates.new_stage_async()

    # ----------------------------------------------------------------------
    async def test_create_render_product(self):
        rp_3 = rep.create.render_product("/OmniverseKit_Persp", (512, 512), name="RP3")

        test_graph, new_nodes, _, _ = og.Controller.edit(
            {"graph_path": "/ActionGraph", "evaluator_name": "execution"},
            {
                og.Controller.Keys.CREATE_NODES: [
                    ("OnTick", "omni.graph.action.OnTick"),
                    ("createRP1", "isaacsim.core.nodes.IsaacCreateRenderProduct"),
                    ("createRP2", "isaacsim.core.nodes.IsaacCreateRenderProduct"),
                    ("createRP3", "isaacsim.core.nodes.IsaacCreateRenderProduct"),
                ],
                og.Controller.Keys.CONNECT: [
                    ("OnTick.outputs:tick", "createRP1.inputs:execIn"),
                    ("OnTick.outputs:tick", "createRP2.inputs:execIn"),
                    ("OnTick.outputs:tick", "createRP3.inputs:execIn"),
                ],
                og.Controller.Keys.SET_VALUES: [
                    ("createRP1.inputs:width", 512),
                    ("createRP1.inputs:cameraPrim", "/OmniverseKit_Persp"),
                    ("createRP2.inputs:cameraPrim", "/OmniverseKit_Persp"),
                    ("createRP2.inputs:enabled", False),
                    ("createRP3.inputs:cameraPrim", "/OmniverseKit_Persp"),
                    ("createRP3.inputs:renderProductPrim", [rp_3.path]),
                ],
            },
        )
        self._stage = stage_utils.get_current_stage()
        self._timeline.play()
        await omni.kit.app.get_app().next_update_async()
        await omni.kit.app.get_app().next_update_async()
        await omni.kit.app.get_app().next_update_async()
        rp_prefix = carb.settings.get_settings().get_as_string("/exts/omni.kit.hydra_texture/renderProduct/path/prefix")
        rp_1 = UsdRender.Product(self._stage.GetPrimAtPath("/Render/" + rp_prefix + "Replicator"))
        rp_2 = UsdRender.Product(self._stage.GetPrimAtPath("/Render/" + rp_prefix + "Replicator_01"))
        self.assertTrue(rp_1)
        self.assertFalse(rp_2)

        og.Controller.attribute("inputs:width", new_nodes[1]).set(700)
        await omni.kit.app.get_app().next_update_async()
        await omni.kit.app.get_app().next_update_async()
        self.assertEqual(rp_1.GetResolutionAttr().Get(), (700, 720))

        og.Controller.attribute("inputs:cameraPrim", new_nodes[1]).set(["/OmniverseKit_Top"])
        await omni.kit.app.get_app().next_update_async()
        await omni.kit.app.get_app().next_update_async()
        self.assertEqual(rp_1.GetCameraRel().GetTargets()[0], "/OmniverseKit_Top")

        self._timeline.stop()
        await omni.kit.app.get_app().next_update_async()

        # Test that RP3 re-uses the existing render product
        rp_3_node_rp_out = og.Controller.attribute("outputs:renderProductPath", new_nodes[3]).get()
        self.assertEqual(rp_3_node_rp_out, rp_3.path)
