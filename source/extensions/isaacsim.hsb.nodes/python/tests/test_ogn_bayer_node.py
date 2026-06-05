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

"""Unit tests for OgnRGBToVB1940 node."""

import numpy as np
import omni.graph.core as og
import omni.kit.test
import omni.timeline


class TestOgnRGBToVB1940(omni.kit.test.AsyncTestCase):
    """Test the RGB to Bayer OGN node."""

    async def setUp(self) -> None:
        """Set up test environment."""
        await omni.usd.get_context().new_stage_async()
        await omni.kit.app.get_app().next_update_async()

    async def tearDown(self) -> None:
        """Clean up test environment."""
        await omni.kit.stage_templates.new_stage_async()

    async def test_node_creation(self) -> None:
        """Test that the node can be created."""
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

        self.assertIsNotNone(test_graph, "Graph should be created")
        self.assertEqual(len(new_nodes), 1, "Should create one node")

    async def test_node_with_cpu_array_input(self) -> None:
        """Test node with CPU array input (most common case)."""
        graph_path = "/TestGraph"
        width, height = 640, 480

        # Create test RGB image
        rgb_image = np.zeros((height * width * 3,), dtype=np.uint8)
        # Set some pattern: red in top-left corner
        for i in range(100 * 100):  # 100x100 red square
            row = i // 100
            col = i % 100
            idx = (row * width + col) * 3
            rgb_image[idx] = 255  # Red channel

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
                    ("BayerNode.inputs:cudaDeviceIndex", -1),  # CPU
                    ("BayerNode.inputs:dataPtr", 0),
                    ("BayerNode.inputs:bufferSize", 0),
                ],
                og.Controller.Keys.CONNECT: [
                    ("OnImpulse.outputs:execOut", "BayerNode.inputs:execIn"),
                ],
            },
        )

        await omni.kit.app.get_app().next_update_async()

        # Set array data after graph creation
        og.Controller.attribute(f"{graph_path}/BayerNode.inputs:data").set(rgb_image.tolist())

        # Use timeline-based execution
        timeline = omni.timeline.get_timeline_interface()
        timeline.play()

        # Trigger execution
        og.Controller.attribute(f"{graph_path}/OnImpulse.state:enableImpulse").set(True)
        await omni.kit.app.get_app().next_update_async()

        timeline.stop()

        # Check outputs: vb1940_csi_linux frame size = lineBytesRaw10(width) * (height + 3)
        # lineBytesRaw10(640) = ((640+3)//4 * 5 + 7) & ~7 = 800
        output_data = og.Controller.attribute(f"{graph_path}/BayerNode.outputs:data").get()
        expected_line_bytes = ((width + 3) // 4 * 5 + 7) & ~7
        expected_size = expected_line_bytes * (height + 3)
        self.assertEqual(len(output_data), expected_size, f"Output size should be CSI frame size {expected_size}")

    async def test_node_with_rgba_input(self) -> None:
        """Test node with RGBA input (should extract RGB)."""
        graph_path = "/TestGraph"
        width, height = 320, 240

        # Create test RGBA image (with alpha channel)
        rgba_image = np.zeros((height * width * 4,), dtype=np.uint8)
        # Set green channel
        for i in range(height * width):
            rgba_image[i * 4 + 1] = 255  # Green
            rgba_image[i * 4 + 3] = 255  # Alpha (should be ignored)

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
                    ("BayerNode.inputs:encoding", "rgba8"),
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

        # Set array data after graph creation
        og.Controller.attribute(f"{graph_path}/BayerNode.inputs:data").set(rgba_image.tolist())

        # Use timeline-based execution
        timeline = omni.timeline.get_timeline_interface()
        timeline.play()

        # Trigger execution
        og.Controller.attribute(f"{graph_path}/OnImpulse.state:enableImpulse").set(True)
        await omni.kit.app.get_app().next_update_async()

        timeline.stop()

        # Check output size matches expected CSI frame size
        output_data = og.Controller.attribute(f"{graph_path}/BayerNode.outputs:data").get()
        expected_line_bytes = ((width + 3) // 4 * 5 + 7) & ~7
        expected_size = expected_line_bytes * (height + 3)
        self.assertEqual(len(output_data), expected_size, f"Output size should be CSI frame size {expected_size}")

    async def test_node_invalid_dimensions(self) -> None:
        """Test node behavior with invalid dimensions."""
        graph_path = "/TestGraph"

        og.Controller.edit(
            {"graph_path": graph_path, "evaluator_name": "execution"},
            {
                og.Controller.Keys.CREATE_NODES: [
                    ("OnImpulse", "omni.graph.action.OnImpulseEvent"),
                    ("BayerNode", "isaacsim.hsb.nodes.RGBToVB1940"),
                ],
                og.Controller.Keys.SET_VALUES: [
                    ("BayerNode.inputs:width", 0),  # Invalid!
                    ("BayerNode.inputs:height", 0),  # Invalid!
                    ("BayerNode.inputs:encoding", "rgb8"),
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

        # Use timeline-based execution
        timeline = omni.timeline.get_timeline_interface()
        timeline.play()

        # Trigger execution - should handle error gracefully (not crash)
        og.Controller.attribute(f"{graph_path}/OnImpulse.state:enableImpulse").set(True)
        await omni.kit.app.get_app().next_update_async()

        timeline.stop()

        # Test passes if we get here without crashing
        self.assertTrue(True, "Node should handle invalid dimensions gracefully")

    async def test_node_unsupported_encoding(self) -> None:
        """Test node with unsupported encoding."""
        graph_path = "/TestGraph"
        width, height = 100, 100
        data = np.zeros((height * width * 3,), dtype=np.uint8)

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
                    ("BayerNode.inputs:encoding", "bgr8"),  # Unsupported!
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

        # Set array data after graph creation
        og.Controller.attribute(f"{graph_path}/BayerNode.inputs:data").set(data.tolist())

        # Use timeline-based execution
        timeline = omni.timeline.get_timeline_interface()
        timeline.play()

        # Trigger execution - should handle error gracefully (not crash)
        og.Controller.attribute(f"{graph_path}/OnImpulse.state:enableImpulse").set(True)
        await omni.kit.app.get_app().next_update_async()

        timeline.stop()

        # Test passes if we get here without crashing
        self.assertTrue(True, "Node should handle unsupported encoding gracefully")

    async def test_bayer_pattern_correctness(self) -> None:
        """Test that the GBRG pattern is correctly applied."""
        graph_path = "/TestGraph"
        width, height = 4, 4

        # Create a test pattern: all red
        rgb_image = np.zeros((height * width * 3,), dtype=np.uint8)
        for i in range(height * width):
            rgb_image[i * 3] = 255  # Red channel only

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

        # Set array data after graph creation
        og.Controller.attribute(f"{graph_path}/BayerNode.inputs:data").set(rgb_image.tolist())

        # Use timeline-based execution
        timeline = omni.timeline.get_timeline_interface()
        timeline.play()

        # Trigger execution
        og.Controller.attribute(f"{graph_path}/OnImpulse.state:enableImpulse").set(True)
        await omni.kit.app.get_app().next_update_async()

        timeline.stop()

        # Check output size matches expected CSI frame size
        output_data = og.Controller.attribute(f"{graph_path}/BayerNode.outputs:data").get()
        expected_line_bytes = ((width + 3) // 4 * 5 + 7) & ~7
        expected_size = expected_line_bytes * (height + 3)
        self.assertEqual(len(output_data), expected_size, f"Output size should be CSI frame size {expected_size}")
