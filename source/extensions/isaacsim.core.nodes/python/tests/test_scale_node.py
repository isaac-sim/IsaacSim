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

import omni.graph.core as og
import omni.kit.test


class TestScaleFromStageUnit(omni.kit.test.AsyncTestCase):
    async def setUp(self):
        """Set up  test environment, to be torn down when done."""
        await omni.usd.get_context().new_stage_async()
        await omni.kit.app.get_app().next_update_async()

    # ----------------------------------------------------------------------
    async def tearDown(self):
        pass

    # ----------------------------------------------------------------------
    async def test_create_scale_node(self):
        graph_path = "/ActionGraph"
        nodeName = "isaac_test_node"

        test_graph, new_nodes, _, _ = og.Controller.edit(
            {"graph_path": graph_path, "evaluator_name": "execution"},
            {
                og.Controller.Keys.CREATE_NODES: [
                    (nodeName, "isaacsim.core.nodes.OgnIsaacScaleToFromStageUnit"),
                ],
            },
        )

        await omni.kit.app.get_app().next_update_async()
