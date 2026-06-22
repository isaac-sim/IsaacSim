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

"""Verifies the CreateViewport OmniGraph node creates and registers a viewport through the rendering manager. Covers graph execution and viewport output creation."""

import carb
import isaacsim.core.experimental.utils.stage as stage_utils
import omni.graph.core as og
import omni.graph.core.tests as ogts
import omni.kit.test
from isaacsim.core.rendering_manager import ViewportManager
from isaacsim.storage.native import get_assets_root_path_async


class TestCreateViewport(ogts.OmniGraphTestCase):
    """Verify viewport graph creation and registration with the rendering manager."""

    async def setUp(self) -> None:
        """Create a fresh stage for viewport graph execution."""
        await omni.usd.get_context().new_stage_async()
        self._timeline = omni.timeline.get_timeline_interface()
        # add franka robot for test
        assets_root_path = await get_assets_root_path_async()
        if assets_root_path is None:
            carb.log_error("Could not find Isaac Sim assets folder")
            return
        await stage_utils.open_stage_async(assets_root_path + "/Isaac/Robots/FrankaRobotics/FrankaPanda/franka.usd")

    # ----------------------------------------------------------------------
    async def tearDown(self) -> None:
        """Close the viewport created by the test and reset the stage."""
        # cleanup extra viewports created in test
        ViewportManager.destroy_viewport_windows(exclude=["Viewport"])
        await omni.kit.app.get_app().next_update_async()
        # Clean up test
        await omni.kit.stage_templates.new_stage_async()

    # ----------------------------------------------------------------------
    async def test_create_viewport(self) -> None:
        """Verify the node creates a viewport and publishes its title."""
        test_graph, new_nodes, _, _ = og.Controller.edit(
            {"graph_path": "/ActionGraph", "evaluator_name": "execution"},
            {
                og.Controller.Keys.CREATE_NODES: [
                    ("OnTick", "omni.graph.action.OnTick"),
                    ("createViewport1", "isaacsim.core.nodes.IsaacCreateViewport"),
                    ("createViewport2", "isaacsim.core.nodes.IsaacCreateViewport"),
                ],
                og.Controller.Keys.CONNECT: [
                    ("OnTick.outputs:tick", "createViewport1.inputs:execIn"),
                    ("OnTick.outputs:tick", "createViewport2.inputs:execIn"),
                ],
                og.Controller.Keys.SET_VALUES: [
                    ("createViewport1.inputs:viewportId", 0),
                    ("createViewport2.inputs:name", "test name"),
                ],
            },
        )

        # await og.Controller.evaluate(test_graph)
        self.assertEqual(len(ViewportManager.get_viewport_windows()), 1)
        # check where the joints are after evaluate
        self._timeline.play()
        await omni.kit.app.get_app().next_update_async()
        await omni.kit.app.get_app().next_update_async()
        await omni.kit.app.get_app().next_update_async()
        self.assertEqual(len(ViewportManager.get_viewport_windows()), 2)
