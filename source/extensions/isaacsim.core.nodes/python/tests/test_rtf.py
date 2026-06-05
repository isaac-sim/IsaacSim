# SPDX-FileCopyrightText: Copyright (c) 2021-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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
from isaacsim.core.nodes.bindings import _isaacsim_core_nodes
from isaacsim.storage.native import get_assets_root_path


class TestRealTimeFactor(ogts.OmniGraphTestCase):
    async def setUp(self):
        """Set up  test environment, to be torn down when done."""
        await omni.usd.get_context().new_stage_async()
        self._timeline = omni.timeline.get_timeline_interface()
        self._core_nodes = _isaacsim_core_nodes.acquire_interface()

        # add franka robot for test
        assets_root_path = get_assets_root_path()
        if assets_root_path is None:
            carb.log_error("Could not find Isaac Sim assets folder")
            return
        await stage_utils.open_stage_async(assets_root_path + "/Isaac/Robots/FrankaRobotics/FrankaPanda/franka.usd")

    # ----------------------------------------------------------------------
    async def tearDown(self):
        """Get rid of temporary data used by the test."""
        await omni.kit.stage_templates.new_stage_async()

    # ----------------------------------------------------------------------
    async def test_rtf(self):
        graph_path = "/ActionGraph"
        nodeName = "isaac_test_node"

        test_graph, new_nodes, _, _ = og.Controller.edit(
            {"graph_path": graph_path, "evaluator_name": "execution"},
            {
                og.Controller.Keys.CREATE_NODES: [
                    ("OnPlaybackTick", "omni.graph.action.OnPlaybackTick"),
                    (nodeName, "isaacsim.core.nodes.IsaacTestNode"),
                    ("toString", "omni.graph.nodes.ToString"),
                    ("rtfNode", "isaacsim.core.nodes.IsaacRealTimeFactor"),
                ],
                og.Controller.Keys.CONNECT: [
                    ("OnPlaybackTick.outputs:tick", nodeName + ".inputs:execIn"),
                    ("rtfNode.outputs:rtf", "toString.inputs:value"),
                    ("toString.outputs:converted", nodeName + ".inputs:input"),
                ],
            },
        )

        node_attribute_path = graph_path + "/" + nodeName + ".outputs:output"

        # check for valid RTF after evaluate
        self._timeline.play()
        await omni.kit.app.get_app().next_update_async()
        await omni.kit.app.get_app().next_update_async()
        await omni.kit.app.get_app().next_update_async()

        self.assertGreater(float(og.Controller.get(og.Controller.attribute(node_attribute_path))), 0.0)
