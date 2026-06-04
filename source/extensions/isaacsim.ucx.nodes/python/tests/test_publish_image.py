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

"""Test UCX image publishing node functionality."""

import isaacsim.core.experimental.utils.app as app_utils
import numpy as np
import omni
import omni.graph.core as og
import ucxx._lib.libucxx as ucx_api
from isaacsim.ucx.nodes.tests.common import UCXTestCase, unpack_image_message
from ucxx._lib.arr import Array


class TestUCXPublishImage(UCXTestCase):
    """Test UCX image publishing."""

    async def setUp(self) -> None:
        """Set up a new stage for image publishing tests."""
        await super().setUp()
        await omni.usd.get_context().new_stage_async()
        await omni.kit.app.get_app().next_update_async()

    async def setup_ucx_client_with_listener(self) -> None:
        """Setup UCX client."""
        for _ in range(5):
            await omni.kit.app.get_app().next_update_async()
        self.create_ucx_client(self.port)
        for _ in range(10):
            await omni.kit.app.get_app().next_update_async()

    async def receive_image_message(self, tag: int = 10, timeout_frames: int = 1000) -> tuple:
        """Receive and unpack an image message.

        Args:
            tag: UCX tag to receive on.
            timeout_frames: Maximum number of frames to wait.

        Returns:
            Tuple of unpacked image message data.
        """
        # Allocate buffer for image (640x480 RGB = ~1MB to be safe)
        max_buffer_size = 1024 * 1024
        buffer = np.empty(max_buffer_size, dtype=np.uint8)

        request = self.client_endpoint.tag_recv(Array(buffer), tag=ucx_api.UCXXTag(tag))

        import time

        for _ in range(timeout_frames):
            if request.completed:
                break
            time.sleep(0.001)
            await omni.kit.app.get_app().next_update_async()

        self.assertTrue(request.completed, "Did not receive image message")
        request.check_error()

        return unpack_image_message(buffer)

    async def test_image_basic_data(self) -> None:
        """Test basic image publishing with raw data input."""
        # Create a simple test image (10x10 RGB)
        test_width = 10
        test_height = 10
        test_channels = 3
        test_image = np.zeros((test_height, test_width, test_channels), dtype=np.uint8)
        # Create a simple pattern (red on left, blue on right)
        test_image[:, :5, 0] = 255  # Red on left
        test_image[:, 5:, 2] = 255  # Blue on right

        # Flatten to 1D array for OmniGraph
        test_data = test_image.flatten().tolist()

        og.Controller.edit(
            {"graph_path": "/ActionGraph", "evaluator_name": "execution"},
            {
                og.Controller.Keys.CREATE_NODES: [
                    ("OnPlaybackTick", "omni.graph.action.OnPlaybackTick"),
                    ("PublishImage", "isaacsim.ucx.nodes.UCXPublishImage"),
                    ("ReadSimTime", "isaacsim.core.nodes.IsaacReadSimulationTime"),
                ],
                og.Controller.Keys.SET_VALUES: [
                    ("PublishImage.inputs:port", self.port),
                    ("PublishImage.inputs:tag", 10),
                    ("PublishImage.inputs:data", test_data),
                    ("PublishImage.inputs:width", test_width),
                    ("PublishImage.inputs:height", test_height),
                    ("PublishImage.inputs:encoding", "rgb8"),
                ],
                og.Controller.Keys.CONNECT: [
                    ("OnPlaybackTick.outputs:tick", "PublishImage.inputs:execIn"),
                    ("ReadSimTime.outputs:simulationTime", "PublishImage.inputs:timeStamp"),
                ],
            },
        )

        timeline = omni.timeline.get_timeline_interface()
        timeline.play()

        await app_utils.update_app_async()

        await self.setup_ucx_client_with_listener()
        timestamp, width, height, encoding, step, image_data = await self.receive_image_message()

        # Verify metadata
        self.assertGreater(timestamp, 0.0)
        self.assertEqual(width, test_width)
        self.assertEqual(height, test_height)
        self.assertEqual(encoding, "rgb8")
        self.assertEqual(step, test_width * test_channels)

        # Verify data size
        expected_size = test_height * step
        self.assertEqual(len(image_data), expected_size)
