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

"""Unit tests for OgnHSBSend node."""

import numpy as np
import omni.graph.core as og
import omni.kit.test
import omni.timeline


class TestOgnHSBSend(omni.kit.test.AsyncTestCase):
    """Test the HSB Send Image OGN node."""

    async def setUp(self) -> None:
        """Set up test environment."""
        await omni.usd.get_context().new_stage_async()
        await omni.kit.app.get_app().next_update_async()

    async def tearDown(self) -> None:
        """Clean up test environment."""
        await omni.kit.stage_templates.new_stage_async()

    async def test_node_creation(self) -> None:
        """Test that the HSB sender node can be created."""
        graph_path = "/TestGraph"

        test_graph, new_nodes, _, _ = og.Controller.edit(
            {"graph_path": graph_path, "evaluator_name": "execution"},
            {
                og.Controller.Keys.CREATE_NODES: [
                    ("HSBNode", "isaacsim.hsb.nodes.HSBSend"),
                ],
            },
        )

        await omni.kit.app.get_app().next_update_async()

        self.assertIsNotNone(test_graph, "Graph should be created")
        self.assertEqual(len(new_nodes), 1, "Should create one node")

    async def test_node_default_values(self) -> None:
        """Test that node has correct default values."""
        graph_path = "/TestGraph"

        og.Controller.edit(
            {"graph_path": graph_path, "evaluator_name": "execution"},
            {
                og.Controller.Keys.CREATE_NODES: [
                    ("HSBNode", "isaacsim.hsb.nodes.HSBSend"),
                ],
            },
        )

        await omni.kit.app.get_app().next_update_async()

        # Check default values
        ip_address = og.Controller.attribute(f"{graph_path}/HSBNode.inputs:ipAddress").get()
        data_plane_id = og.Controller.attribute(f"{graph_path}/HSBNode.inputs:dataPlaneId").get()
        sensor_id = og.Controller.attribute(f"{graph_path}/HSBNode.inputs:sensorId").get()
        self.assertEqual(ip_address, "127.0.0.1", "Default IP should be localhost")
        self.assertEqual(data_plane_id, 0, "Default data plane ID should be 0")
        self.assertEqual(sensor_id, 0, "Default sensor ID should be 0")

    async def test_node_with_cpu_data(self) -> None:
        """Test node with CPU array data input."""
        graph_path = "/TestGraph"
        width, height = 640, 480

        # Create test Bayer image data
        bayer_data = np.random.randint(0, 255, (height * width,), dtype=np.uint8)

        og.Controller.edit(
            {"graph_path": graph_path, "evaluator_name": "execution"},
            {
                og.Controller.Keys.CREATE_NODES: [
                    ("OnImpulse", "omni.graph.action.OnImpulseEvent"),
                    ("HSBNode", "isaacsim.hsb.nodes.HSBSend"),
                ],
                og.Controller.Keys.SET_VALUES: [
                    ("HSBNode.inputs:ipAddress", "127.0.0.1"),
                    ("HSBNode.inputs:dataPlaneId", 0),
                    ("HSBNode.inputs:sensorId", 0),
                ],
                og.Controller.Keys.CONNECT: [
                    ("OnImpulse.outputs:execOut", "HSBNode.inputs:execIn"),
                ],
            },
        )

        await omni.kit.app.get_app().next_update_async()

        # Set array data after graph creation
        og.Controller.attribute(f"{graph_path}/HSBNode.inputs:data").set(bayer_data.tolist())

        # Use timeline-based execution
        timeline = omni.timeline.get_timeline_interface()
        timeline.play()

        # Note: We can't fully test HSB transmission without a receiver,
        # but we can verify the node doesn't crash and completes execution
        og.Controller.attribute(f"{graph_path}/OnImpulse.state:enableImpulse").set(True)
        await omni.kit.app.get_app().next_update_async()

        timeline.stop()

        # Test passes if we get here without crashing
        self.assertTrue(True, "Graph evaluation should complete without crashing")

    async def test_pipeline_bayer_to_hsb(self) -> None:
        """Test complete pipeline: Bayer conversion -> HSB publish."""
        graph_path = "/TestGraph"
        width, height = 320, 240

        # Create test RGB image
        rgb_image = np.zeros((height * width * 3,), dtype=np.uint8)
        # Create a pattern: red gradient
        for row in range(height):
            for col in range(width):
                idx = (row * width + col) * 3
                rgb_image[idx] = int(255 * col / width)  # Red gradient

        og.Controller.edit(
            {"graph_path": graph_path, "evaluator_name": "execution"},
            {
                og.Controller.Keys.CREATE_NODES: [
                    ("OnImpulse", "omni.graph.action.OnImpulseEvent"),
                    ("BayerConvert", "isaacsim.hsb.nodes.RGBToVB1940"),
                    ("HSBSend", "isaacsim.hsb.nodes.HSBSend"),
                ],
                og.Controller.Keys.SET_VALUES: [
                    # Bayer converter inputs
                    ("BayerConvert.inputs:width", width),
                    ("BayerConvert.inputs:height", height),
                    ("BayerConvert.inputs:encoding", "rgb8"),
                    ("BayerConvert.inputs:outputMode", "vb1940_csi_linux"),
                    ("BayerConvert.inputs:cudaDeviceIndex", -1),
                    ("BayerConvert.inputs:dataPtr", 0),
                    ("BayerConvert.inputs:bufferSize", 0),
                    # HSB publisher inputs
                    ("HSBSend.inputs:ipAddress", "127.0.0.1"),
                    ("HSBSend.inputs:dataPlaneId", 0),
                    ("HSBSend.inputs:sensorId", 1),
                ],
                og.Controller.Keys.CONNECT: [
                    # Pipeline connections
                    ("OnImpulse.outputs:execOut", "BayerConvert.inputs:execIn"),
                    ("BayerConvert.outputs:execOut", "HSBSend.inputs:execIn"),
                    ("BayerConvert.outputs:data", "HSBSend.inputs:data"),
                ],
            },
        )

        await omni.kit.app.get_app().next_update_async()

        # Set array data after graph creation
        og.Controller.attribute(f"{graph_path}/BayerConvert.inputs:data").set(rgb_image.tolist())

        # Use timeline-based execution
        timeline = omni.timeline.get_timeline_interface()
        timeline.play()

        # Trigger the pipeline
        og.Controller.attribute(f"{graph_path}/OnImpulse.state:enableImpulse").set(True)
        await omni.kit.app.get_app().next_update_async()

        timeline.stop()

        # Verify data flowed through the pipeline
        output_data = og.Controller.attribute(f"{graph_path}/BayerConvert.outputs:data").get()

        expected_line_bytes = ((width + 3) // 4 * 5 + 7) & ~7
        expected_size = expected_line_bytes * (height + 3)
        self.assertEqual(len(output_data), expected_size, "Output should be VB1940 CSI frame size")

    async def test_node_with_invalid_dimensions(self) -> None:
        """Test node behavior with invalid image dimensions."""
        graph_path = "/TestGraph"

        og.Controller.edit(
            {"graph_path": graph_path, "evaluator_name": "execution"},
            {
                og.Controller.Keys.CREATE_NODES: [
                    ("OnImpulse", "omni.graph.action.OnImpulseEvent"),
                    ("HSBNode", "isaacsim.hsb.nodes.HSBSend"),
                ],
                og.Controller.Keys.SET_VALUES: [],
                og.Controller.Keys.CONNECT: [
                    ("OnImpulse.outputs:execOut", "HSBNode.inputs:execIn"),
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

    async def test_node_configuration_parameters(self) -> None:
        """Test that configuration parameters can be set."""
        graph_path = "/TestGraph"

        og.Controller.edit(
            {"graph_path": graph_path, "evaluator_name": "execution"},
            {
                og.Controller.Keys.CREATE_NODES: [
                    ("HSBNode", "isaacsim.hsb.nodes.HSBSend"),
                ],
                og.Controller.Keys.SET_VALUES: [
                    ("HSBNode.inputs:ipAddress", "192.168.1.100"),
                    ("HSBNode.inputs:dataPlaneId", 5),
                    ("HSBNode.inputs:sensorId", 10),
                ],
            },
        )

        await omni.kit.app.get_app().next_update_async()

        # Verify parameters were set
        ip_address = og.Controller.attribute(f"{graph_path}/HSBNode.inputs:ipAddress").get()
        data_plane_id = og.Controller.attribute(f"{graph_path}/HSBNode.inputs:dataPlaneId").get()
        sensor_id = og.Controller.attribute(f"{graph_path}/HSBNode.inputs:sensorId").get()
        self.assertEqual(ip_address, "192.168.1.100", "Custom IP should be set")
        self.assertEqual(data_plane_id, 5, "Custom data plane ID should be set")
        self.assertEqual(sensor_id, 10, "Custom sensor ID should be set")

    async def test_data_plane_type_coe(self) -> None:
        """Test that dataPlaneType can be set to 'coe' and is readable."""
        graph_path = "/TestGraph"

        og.Controller.edit(
            {"graph_path": graph_path, "evaluator_name": "execution"},
            {
                og.Controller.Keys.CREATE_NODES: [
                    ("HSBNode", "isaacsim.hsb.nodes.HSBSend"),
                ],
                og.Controller.Keys.SET_VALUES: [
                    ("HSBNode.inputs:dataPlaneType", "coe"),
                ],
            },
        )

        await omni.kit.app.get_app().next_update_async()

        data_plane_type = og.Controller.attribute(f"{graph_path}/HSBNode.inputs:dataPlaneType").get()
        self.assertEqual(data_plane_type, "coe", "dataPlaneType should be 'coe'")

    async def test_multiple_sensors(self) -> None:
        """Test publishing from multiple sensors with different IDs."""
        graph_path = "/TestGraph"
        width, height = 160, 120

        # Create test data
        bayer_data1 = np.full((height * width,), 100, dtype=np.uint8)
        bayer_data2 = np.full((height * width,), 200, dtype=np.uint8)

        og.Controller.edit(
            {"graph_path": graph_path, "evaluator_name": "execution"},
            {
                og.Controller.Keys.CREATE_NODES: [
                    ("OnImpulse", "omni.graph.action.OnImpulseEvent"),
                    ("HSBSensor1", "isaacsim.hsb.nodes.HSBSend"),
                    ("HSBSensor2", "isaacsim.hsb.nodes.HSBSend"),
                ],
                og.Controller.Keys.SET_VALUES: [
                    ("HSBSensor1.inputs:sensorId", 1),
                    ("HSBSensor2.inputs:sensorId", 2),
                ],
                og.Controller.Keys.CONNECT: [
                    ("OnImpulse.outputs:execOut", "HSBSensor1.inputs:execIn"),
                    ("OnImpulse.outputs:execOut", "HSBSensor2.inputs:execIn"),
                ],
            },
        )

        await omni.kit.app.get_app().next_update_async()

        # Set array data after graph creation
        og.Controller.attribute(f"{graph_path}/HSBSensor1.inputs:data").set(bayer_data1.tolist())
        og.Controller.attribute(f"{graph_path}/HSBSensor2.inputs:data").set(bayer_data2.tolist())

        # Use timeline-based execution
        timeline = omni.timeline.get_timeline_interface()
        timeline.play()

        # Both nodes should be able to execute
        og.Controller.attribute(f"{graph_path}/OnImpulse.state:enableImpulse").set(True)
        await omni.kit.app.get_app().next_update_async()

        timeline.stop()

        # Test passes if we get here without crashing
        self.assertTrue(True, "Multiple sensor nodes should execute without crashing")

    async def test_pipeline_vb1940_csi_linux_to_hsb(self) -> None:
        """Test pipeline: RGBToVB1940 (outputMode=vb1940_csi_linux) -> HSBSend (buffer mode)."""
        graph_path = "/TestGraph"
        width, height = 64, 48

        rgb_image = np.zeros((height * width * 3,), dtype=np.uint8)
        for i in range(height * width):
            idx = i * 3
            rgb_image[idx] = i % 256
            rgb_image[idx + 1] = (i * 2) % 256
            rgb_image[idx + 2] = (i * 3) % 256

        og.Controller.edit(
            {"graph_path": graph_path, "evaluator_name": "execution"},
            {
                og.Controller.Keys.CREATE_NODES: [
                    ("OnImpulse", "omni.graph.action.OnImpulseEvent"),
                    ("BayerConvert", "isaacsim.hsb.nodes.RGBToVB1940"),
                    ("HSBSend", "isaacsim.hsb.nodes.HSBSend"),
                ],
                og.Controller.Keys.SET_VALUES: [
                    ("BayerConvert.inputs:width", width),
                    ("BayerConvert.inputs:height", height),
                    ("BayerConvert.inputs:encoding", "rgb8"),
                    ("BayerConvert.inputs:outputMode", "vb1940_csi_linux"),
                    ("BayerConvert.inputs:cudaDeviceIndex", -1),
                    ("BayerConvert.inputs:dataPtr", 0),
                    ("BayerConvert.inputs:bufferSize", 0),
                    ("HSBSend.inputs:ipAddress", "127.0.0.1"),
                ],
                og.Controller.Keys.CONNECT: [
                    ("OnImpulse.outputs:execOut", "BayerConvert.inputs:execIn"),
                    ("BayerConvert.outputs:execOut", "HSBSend.inputs:execIn"),
                    ("BayerConvert.outputs:data", "HSBSend.inputs:data"),
                ],
            },
        )

        await omni.kit.app.get_app().next_update_async()

        og.Controller.attribute(f"{graph_path}/BayerConvert.inputs:data").set(rgb_image.tolist())

        timeline = omni.timeline.get_timeline_interface()
        timeline.play()

        og.Controller.attribute(f"{graph_path}/OnImpulse.state:enableImpulse").set(True)
        await omni.kit.app.get_app().next_update_async()

        timeline.stop()

        output_data = og.Controller.attribute(f"{graph_path}/BayerConvert.outputs:data").get()
        self.assertGreater(len(output_data), 0)

    async def test_pipeline_vb1940_csi_coe_to_hsb(self) -> None:
        """Test pipeline: RGBToVB1940 (outputMode=vb1940_csi_coe) -> HSBSend (dataPlaneType=coe, buffer)."""
        graph_path = "/TestGraph"
        width, height = 64, 48

        rgb_image = np.zeros((height * width * 3,), dtype=np.uint8)
        for i in range(height * width):
            idx = i * 3
            rgb_image[idx] = i % 256
            rgb_image[idx + 1] = (i * 2) % 256
            rgb_image[idx + 2] = (i * 3) % 256

        og.Controller.edit(
            {"graph_path": graph_path, "evaluator_name": "execution"},
            {
                og.Controller.Keys.CREATE_NODES: [
                    ("OnImpulse", "omni.graph.action.OnImpulseEvent"),
                    ("BayerConvert", "isaacsim.hsb.nodes.RGBToVB1940"),
                    ("HSBSend", "isaacsim.hsb.nodes.HSBSend"),
                ],
                og.Controller.Keys.SET_VALUES: [
                    ("BayerConvert.inputs:width", width),
                    ("BayerConvert.inputs:height", height),
                    ("BayerConvert.inputs:encoding", "rgb8"),
                    ("BayerConvert.inputs:outputMode", "vb1940_csi_coe"),
                    ("BayerConvert.inputs:cudaDeviceIndex", -1),
                    ("BayerConvert.inputs:dataPtr", 0),
                    ("BayerConvert.inputs:bufferSize", 0),
                    ("HSBSend.inputs:ipAddress", "127.0.0.1"),
                    ("HSBSend.inputs:dataPlaneType", "coe"),
                ],
                og.Controller.Keys.CONNECT: [
                    ("OnImpulse.outputs:execOut", "BayerConvert.inputs:execIn"),
                    ("BayerConvert.outputs:execOut", "HSBSend.inputs:execIn"),
                    ("BayerConvert.outputs:data", "HSBSend.inputs:data"),
                ],
            },
        )

        await omni.kit.app.get_app().next_update_async()

        og.Controller.attribute(f"{graph_path}/BayerConvert.inputs:data").set(rgb_image.tolist())

        timeline = omni.timeline.get_timeline_interface()
        timeline.play()
        og.Controller.attribute(f"{graph_path}/OnImpulse.state:enableImpulse").set(True)
        await omni.kit.app.get_app().next_update_async()
        timeline.stop()

        # lineBytesRaw10Coe: (width+2)//3*4, 64-byte aligned
        expected_line_bytes = ((width + 2) // 3 * 4 + 63) & ~63
        expected_coe_size = expected_line_bytes * (height + 3)
        output_data = og.Controller.attribute(f"{graph_path}/BayerConvert.outputs:data").get()
        self.assertEqual(len(output_data), expected_coe_size, "COE output data length should match 3p4b frame size")
