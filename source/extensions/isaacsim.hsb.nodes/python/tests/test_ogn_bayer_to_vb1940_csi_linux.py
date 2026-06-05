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

"""Unit tests for RGBToVB1940 node with outputMode=vb1940_csi_linux (merged Bayer+CSI path)."""

import numpy as np
import omni.graph.core as og
import omni.kit.test
import omni.timeline


class TestRGBToVB1940VB1940CSILinux(omni.kit.test.AsyncTestCase):
    """Test RGBToVB1940 when outputMode is vb1940_csi_linux (RGB -> CSI Linux frame in one node)."""

    async def setUp(self) -> None:
        """Set up test environment."""
        await omni.usd.get_context().new_stage_async()
        await omni.kit.app.get_app().next_update_async()

    async def tearDown(self) -> None:
        """Clean up test environment."""
        await omni.kit.stage_templates.new_stage_async()

    async def test_node_creation(self) -> None:
        """Test that the node can be created (same RGBToVB1940 type)."""
        graph_path = "/TestGraph"

        test_graph, new_nodes, _, _ = og.Controller.edit(
            {"graph_path": graph_path, "evaluator_name": "execution"},
            {
                og.Controller.Keys.CREATE_NODES: [
                    ("BayerNode", "isaacsim.hsb.nodes.RGBToVB1940"),
                ],
            },
        )

        await omni.kit.app.get_app().next_update_async()

        self.assertIsNotNone(test_graph)
        self.assertEqual(len(new_nodes), 1)

    async def test_node_vb1940_csi_linux_output(self) -> None:
        """Test node with RGB input and outputMode=vb1940_csi_linux produces CSI frame and bufferSize."""
        graph_path = "/TestGraph"
        width, height = 64, 48

        rgb_data = np.random.randint(0, 255, (height * width * 3,), dtype=np.uint8)

        og.Controller.edit(
            {"graph_path": graph_path, "evaluator_name": "execution"},
            {
                og.Controller.Keys.CREATE_NODES: [
                    ("OnImpulse", "omni.graph.action.OnImpulseEvent"),
                    ("BayerNode", "isaacsim.hsb.nodes.RGBToVB1940"),
                ],
                og.Controller.Keys.SET_VALUES: [
                    ("BayerNode.inputs:width", width),
                    ("BayerNode.inputs:height", height),
                    ("BayerNode.inputs:encoding", "rgb8"),
                    ("BayerNode.inputs:outputMode", "vb1940_csi_linux"),
                    ("BayerNode.inputs:cudaDeviceIndex", -1),
                    ("BayerNode.inputs:dataPtr", 0),
                    ("BayerNode.inputs:bufferSize", 0),
                ],
                og.Controller.Keys.CONNECT: [
                    ("OnImpulse.outputs:execOut", "BayerNode.inputs:execIn"),
                ],
            },
        )

        await omni.kit.app.get_app().next_update_async()

        og.Controller.attribute(f"{graph_path}/BayerNode.inputs:data").set(rgb_data.tolist())

        timeline = omni.timeline.get_timeline_interface()
        timeline.play()

        og.Controller.attribute(f"{graph_path}/OnImpulse.state:enableImpulse").set(True)
        await omni.kit.app.get_app().next_update_async()

        timeline.stop()

        output_data = og.Controller.attribute(f"{graph_path}/BayerNode.outputs:data").get()

        self.assertIsInstance(output_data, (list, tuple, np.ndarray))
        line_bytes = ((width + 3) // 4 * 5 + 7) // 8 * 8
        expected_size = line_bytes * (1 + height + 2)
        self.assertEqual(len(output_data), expected_size)

    async def test_node_invalid_dimensions(self) -> None:
        """Test node with zero dimensions does not crash (bufferSize remains 0)."""
        graph_path = "/TestGraph"

        og.Controller.edit(
            {"graph_path": graph_path, "evaluator_name": "execution"},
            {
                og.Controller.Keys.CREATE_NODES: [
                    ("OnImpulse", "omni.graph.action.OnImpulseEvent"),
                    ("BayerNode", "isaacsim.hsb.nodes.RGBToVB1940"),
                ],
                og.Controller.Keys.SET_VALUES: [
                    ("BayerNode.inputs:width", 0),
                    ("BayerNode.inputs:height", 0),
                    ("BayerNode.inputs:outputMode", "vb1940_csi_linux"),
                ],
                og.Controller.Keys.CONNECT: [
                    ("OnImpulse.outputs:execOut", "BayerNode.inputs:execIn"),
                ],
            },
        )

        await omni.kit.app.get_app().next_update_async()

        timeline = omni.timeline.get_timeline_interface()
        timeline.play()

        og.Controller.attribute(f"{graph_path}/OnImpulse.state:enableImpulse").set(True)
        await omni.kit.app.get_app().next_update_async()

        timeline.stop()

        output_data = og.Controller.attribute(f"{graph_path}/BayerNode.outputs:data").get()
        self.assertEqual(len(output_data), 0)
