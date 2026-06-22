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

"""Verify ROS 2 image-to-buffer conversion.

Covers color, mono, 16-bit, floating-point, YUV, padded-stride, copy, squeeze,
endian, and invalid-message cases.
"""

import os
import tempfile
import time
from typing import Any

import isaacsim.core.experimental.utils.stage as stage_utils
import numpy as np
import rclpy
from isaacsim.ros2.core.impl.ros2_image_test_utils import create_image, ros2_image_to_buffer
from isaacsim.ros2.core.impl.ros2_test_case import ROS2TestCase
from isaacsim.test.utils.image_io import read_image_as_array, save_depth_image, save_rgb_image
from sensor_msgs.msg import Image

from .common import get_qos_profile


class TestRos2ImageToBuffer(ROS2TestCase):
    """Verify conversion from sensor_msgs/Image messages to NumPy buffers."""

    # Configuration for each test's golden image
    # Maps test method name to (height, width, encoding, channels, dtype, extra_config)
    _TEST_IMAGE_CONFIG = {
        "test_bgr8_normalization_and_no_normalization": (8, 8, "bgr8", 3, np.uint8, {}),
        "test_bgra8_alpha_preserved": (8, 8, "bgra8", 4, np.uint8, {}),
        "test_mono8_squeeze_toggle": (8, 8, "mono8", 1, np.uint8, {}),
        "test_rgb16_big_endian_byteswap": (8, 8, "rgb16", 3, np.uint16, {}),
        "test_stride_padding_rows": (8, 8, "rgb8", 3, np.uint8, {"padding": 4}),
        "test_opencv_16sc3": (8, 8, "16SC3", 3, np.int16, {}),
        "test_opencv_32fc1_squeeze_toggle": (8, 8, "32FC1", 1, np.float32, {}),
        "test_yuv422_shape": (8, 8, "uyvy", 2, np.uint8, {}),
    }

    async def setUp(self) -> None:
        """Create a fresh stage and temporary image-buffer output directory."""
        await super().setUp()

        await stage_utils.create_new_stage_async()

        # Create temp directory for golden images
        self._temp_dir = tempfile.mkdtemp(prefix="ros2_image_test_")
        self._golden_files = []

        # Generate golden image for the current test if applicable
        test_name = self._testMethodName
        if test_name in self._TEST_IMAGE_CONFIG:
            self._create_golden_image(test_name)

        # Initialize received image storage
        self._received_image = None

    async def tearDown(self) -> None:
        """Remove temporary image-buffer files after each conversion test."""
        # Delete all golden images created during this test
        for filepath in self._golden_files:
            if os.path.exists(filepath):
                os.remove(filepath)

        # Remove temp directory if empty
        if hasattr(self, "_temp_dir") and os.path.exists(self._temp_dir):
            try:
                os.rmdir(self._temp_dir)
            except OSError:
                pass  # Directory not empty, leave it

        await super().tearDown()

    def _create_golden_image(self, test_name: Any) -> None:
        """Create and save a random golden image for the given test.

        Args:
            test_name: Name of the image conversion test.
        """
        h, w, encoding, channels, dtype, extra_config = self._TEST_IMAGE_CONFIG[test_name]

        # Use a fixed seed based on test name for reproducibility
        seed = hash(test_name) % (2**32)
        rng = np.random.default_rng(seed)

        # Generate random image data based on dtype
        if dtype == np.float32:
            # Float data: random values in range [-100, 100]
            golden_data = rng.uniform(-100, 100, (h, w, channels)).astype(dtype)
        elif dtype == np.int16:
            # Signed int16: random values in valid range
            golden_data = rng.integers(-32768, 32767, (h, w, channels), dtype=dtype)
        elif dtype == np.uint16:
            # Unsigned int16: random values in valid range
            golden_data = rng.integers(0, 65535, (h, w, channels), dtype=dtype)
        else:
            # uint8: random values 0-255
            golden_data = rng.integers(0, 255, (h, w, channels), dtype=dtype)

        # Squeeze single channel for mono formats
        if channels == 1:
            golden_data = golden_data.squeeze(axis=-1)

        # Save golden data based on dtype
        if dtype == np.float32 and channels == 1:
            # Use save_depth_image with TIFF for lossless float32 storage
            file_name = f"{test_name}_golden.tiff"
            golden_path = os.path.join(self._temp_dir, file_name)
            save_depth_image(golden_data, self._temp_dir, file_name)
        elif dtype == np.uint8:
            # Save as PNG for uint8 data
            file_name = f"{test_name}_golden.png"
            golden_path = os.path.join(self._temp_dir, file_name)
            save_rgb_image(golden_data, self._temp_dir, file_name)
        else:
            # For other non-uint8 types (int16, uint16), store original data directly
            # since PNG can't preserve the dtype
            file_name = f"{test_name}_golden.png"
            golden_path = os.path.join(self._temp_dir, file_name)
            if dtype == np.uint16:
                save_data = (golden_data / 256).astype(np.uint8)
            elif dtype == np.int16:
                save_data = ((golden_data.astype(np.int32) + 32768) / 256).astype(np.uint8)
            else:
                save_data = golden_data.astype(np.uint8)
            save_rgb_image(save_data, self._temp_dir, file_name)
            self._original_golden_data = golden_data

        self._golden_files.append(golden_path)

        # Store config for the test to use
        self._current_golden_path = golden_path
        self._current_config = (h, w, encoding, channels, dtype, extra_config)

    def _load_golden_image(self) -> Any:
        """Load the golden image and adjust for the expected format.

        Returns:
            Golden image data in the expected format.
        """
        h, w, encoding, channels, dtype, extra_config = self._current_config

        # For float32 single-channel, load from TIFF (lossless)
        if dtype == np.float32 and channels == 1:
            img = read_image_as_array(self._current_golden_path, squeeze_singleton_channel=True)
            return img

        # For other non-uint8 types (int16, uint16), use stored original data
        if dtype != np.uint8:
            return self._original_golden_data

        # Load uint8 from PNG
        img = read_image_as_array(self._current_golden_path, squeeze_singleton_channel=False)

        # Strip alpha channel if added by save_rgb_image (PNG adds alpha)
        if img.ndim == 3 and img.shape[2] == 4 and channels < 4:
            img = img[:, :, :channels]

        # Handle mono images (saved as RGB, need first channel)
        if channels == 1 and img.ndim == 3 and img.shape[2] > 1:
            img = img[:, :, 0]

        return img

    def _image_callback(self, msg: Any) -> None:
        """Handle received ROS2 Image messages.

        Args:
            msg: Received ROS 2 image message.
        """
        self._received_image = msg

    async def _publish_and_receive_image(self, ros2_image_msg: Any, topic_name: Any = "/test_image") -> Any:
        """Publish an image and wait for subscriber to receive it.

        Args:
            ros2_image_msg: ROS 2 image message to publish.
            topic_name: Topic to publish and subscribe.

        Returns:
            Received ROS 2 image message, or None if no message arrives.
        """
        # Create node, publisher, and subscriber
        node = self.create_node("image_test_node")
        publisher = self.create_publisher(node, Image, topic_name, get_qos_profile())
        self.create_subscription(node, Image, topic_name, self._image_callback, get_qos_profile())

        def spin() -> None:
            rclpy.spin_once(node, timeout_sec=0.1)

        # Reset received image
        self._received_image = None

        # Publish the image multiple times to ensure delivery

        for _ in range(5):
            publisher.publish(ros2_image_msg)
            time.sleep(0.01)

        await self.simulate_until_condition(
            lambda: self._received_image is not None,
            max_frames=10,
            per_frame_callback=spin,
        )

        self.assertIsNotNone(self._received_image, "Failed to receive image from topic")
        return self._received_image

    async def test_bgr8_normalization_and_no_normalization(self) -> None:
        """Test bgr8 normalization and no normalization."""
        h, w, encoding, channels, dtype, _ = self._current_config
        bgr = self._load_golden_image()

        buf = bgr.tobytes()
        ros2_img = create_image(h, w, encoding, w * channels, buf)

        # Publish and receive via ROS2
        received_msg = await self._publish_and_receive_image(ros2_img)

        # Act: normalize to RGB
        rgb = ros2_image_to_buffer(received_msg, normalize_color_order=True)
        # Assert: channels reversed
        self.assertEqual(rgb.shape, (h, w, 3))
        np.testing.assert_array_equal(rgb[..., 0], bgr[..., 2])  # R == original B
        np.testing.assert_array_equal(rgb[..., 1], bgr[..., 1])  # G == original G
        np.testing.assert_array_equal(rgb[..., 2], bgr[..., 0])  # B == original R

        # Act: keep BGR
        bgr_out = ros2_image_to_buffer(received_msg, normalize_color_order=False)
        np.testing.assert_array_equal(bgr_out, bgr)

    async def test_bgra8_alpha_preserved(self) -> None:
        """Test bgra8 alpha preserved."""
        h, w, encoding, channels, dtype, _ = self._current_config
        bgra = self._load_golden_image()

        ros2_img = create_image(h, w, encoding, w * channels, bgra.tobytes())

        # Publish and receive via ROS2
        received_msg = await self._publish_and_receive_image(ros2_img)

        out = ros2_image_to_buffer(received_msg, normalize_color_order=True)
        self.assertEqual(out.shape, (h, w, 4))
        # RGB reversed, alpha preserved
        np.testing.assert_array_equal(out[..., :3], bgra[..., :3][:, :, ::-1])
        np.testing.assert_array_equal(out[..., 3], bgra[..., 3])

    async def test_mono8_squeeze_toggle(self) -> None:
        """Test mono8 squeeze toggle."""
        h, w, encoding, channels, dtype, _ = self._current_config
        mono = self._load_golden_image()

        # Ensure 2D for mono
        if mono.ndim == 3:
            mono = mono[:, :, 0]

        ros2_img = create_image(h, w, encoding, w * 1, mono.tobytes())

        # Publish and receive via ROS2
        received_msg = await self._publish_and_receive_image(ros2_img)

        # Squeezed
        arr = ros2_image_to_buffer(received_msg, squeeze_singleton_channel=True)
        self.assertEqual(arr.shape, (h, w))
        np.testing.assert_array_equal(arr, mono)
        # Not squeezed
        arr2 = ros2_image_to_buffer(received_msg, squeeze_singleton_channel=False)
        self.assertEqual(arr2.shape, (h, w, 1))
        np.testing.assert_array_equal(arr2[..., 0], mono)

    async def test_rgb16_big_endian_byteswap(self) -> None:
        """Test rgb16 big endian byteswap."""
        h, w, encoding, channels, dtype, extra_config = self._current_config
        rgb16 = self._load_golden_image()

        # Ensure 3D shape
        if rgb16.ndim == 2:
            rgb16 = rgb16[:, :, np.newaxis]

        # Convert to big-endian for the test
        rgb16_be = rgb16.astype(">u2")
        ros2_img = create_image(h, w, encoding, w * channels * 2, rgb16_be.tobytes(), is_bigendian=1)

        # Publish and receive via ROS2
        received_msg = await self._publish_and_receive_image(ros2_img)

        out = ros2_image_to_buffer(received_msg)
        self.assertEqual(out.dtype, np.uint16)
        # Expect native-endian values matching original
        np.testing.assert_array_equal(out, rgb16)

    async def test_stride_padding_rows(self) -> None:
        """Test stride padding rows."""
        h, w, encoding, channels, dtype, extra_config = self._current_config
        padding = extra_config.get("padding", 4)
        golden = self._load_golden_image()

        expected_row_bytes = w * channels * np.dtype(dtype).itemsize
        step = expected_row_bytes + padding

        # Build buffer row by row with padding
        buf = bytearray()
        for row_idx in range(h):
            row_data = golden[row_idx].tobytes()
            buf += row_data
            buf += b"\x00" * padding

        ros2_img = create_image(h, w, encoding, step, bytes(buf))

        # Publish and receive via ROS2
        received_msg = await self._publish_and_receive_image(ros2_img)

        out = ros2_image_to_buffer(received_msg)
        self.assertEqual(out.shape, (h, w, channels))
        np.testing.assert_array_equal(out, golden)

    async def test_opencv_16sc3(self) -> None:
        """Test opencv 16sc3."""
        h, w, encoding, channels, dtype, _ = self._current_config
        arr = self._load_golden_image()

        # Ensure 3D shape
        if arr.ndim == 2:
            arr = arr[:, :, np.newaxis]

        ros2_img = create_image(h, w, encoding, w * channels * 2, arr.tobytes())

        # Publish and receive via ROS2
        received_msg = await self._publish_and_receive_image(ros2_img)

        out = ros2_image_to_buffer(received_msg)
        self.assertEqual(out.shape, (h, w, channels))
        self.assertEqual(out.dtype, np.int16)
        np.testing.assert_array_equal(out, arr)

    async def test_opencv_32fc1_squeeze_toggle(self) -> None:
        """Test opencv 32fc1 squeeze toggle."""
        h, w, encoding, channels, dtype, _ = self._current_config
        float_data = self._load_golden_image()

        # Ensure 2D for single channel
        if float_data.ndim == 3:
            float_data = float_data[:, :, 0]

        ros2_img = create_image(h, w, encoding, w * 4, float_data.tobytes())

        # Publish and receive via ROS2
        received_msg = await self._publish_and_receive_image(ros2_img)

        # Squeezed (single channel becomes 2D)
        arr = ros2_image_to_buffer(received_msg, squeeze_singleton_channel=True)
        self.assertEqual(arr.shape, (h, w))
        self.assertEqual(arr.dtype, np.float32)
        np.testing.assert_array_equal(arr, float_data)
        # Not squeezed (single channel becomes 3D with C=1)
        arr2 = ros2_image_to_buffer(received_msg, squeeze_singleton_channel=False)
        self.assertEqual(arr2.shape, (h, w, 1))
        self.assertEqual(arr2.dtype, np.float32)
        np.testing.assert_array_equal(arr2[..., 0], float_data)

    async def test_yuv422_shape(self) -> None:
        """Test yuv422 shape."""
        h, w, encoding, channels, dtype, _ = self._current_config
        data = self._load_golden_image()

        # YUV422 has 2 channels
        if data.ndim == 3 and data.shape[2] > channels:
            data = data[:, :, :channels]
        elif data.ndim == 2:
            # Need to generate proper 2-channel data
            data = np.stack([data, data], axis=-1)

        ros2_img = create_image(h, w, encoding, w * channels, data.tobytes())

        # Publish and receive via ROS2
        received_msg = await self._publish_and_receive_image(ros2_img)

        out = ros2_image_to_buffer(received_msg)
        self.assertEqual(out.shape, (h, w, channels))
        self.assertEqual(out.dtype, np.uint8)

    def test_unsupported_and_unknown_encodings_raise(self) -> None:
        """Test unsupported and unknown encodings raise."""
        h, w = 1, 1
        # nv21 not supported
        with self.assertRaises(ValueError):
            ros2_image_to_buffer(create_image(h, w, "nv21", 2, b"\x00\x00"))
        # unknown encoding
        with self.assertRaises(ValueError):
            ros2_image_to_buffer(create_image(h, w, "foo", 3, b"\x00\x00\x00"))

    def test_invalid_step_too_small_raises(self) -> None:
        """Test invalid step too small raises."""
        h, w = 1, 2
        # rgb8 expected row bytes = 2*3 = 6, provide step smaller
        with self.assertRaises(ValueError):
            ros2_image_to_buffer(create_image(h, w, "rgb8", 5, b"\x00" * 6))

    def test_buffer_size_mismatch_raises(self) -> None:
        """Test buffer size mismatch raises."""
        h, w = 2, 2
        # rgb8 needs 2*2*3 = 12 bytes; give 10
        with self.assertRaises(ValueError):
            ros2_image_to_buffer(create_image(h, w, "rgb8", 6, b"\x00" * 10))

    def test_row_length_mismatch_raises(self) -> None:
        """Test row length mismatch raises."""
        h, w = 2, 2
        channels = 3
        expected_row_bytes = w * channels
        step = expected_row_bytes + 2
        # Build only first row complete, second row truncated before expected bytes
        row0 = bytes([1] * expected_row_bytes) + b"\x00\x00"
        row1_truncated = bytes([2] * (expected_row_bytes - 1))  # Truncated row without padding
        data = row0 + row1_truncated
        with self.assertRaises(ValueError):
            ros2_image_to_buffer(create_image(h, w, "rgb8", step, data))

    def test_copy_behavior(self) -> None:
        """Test copy behavior."""
        h, w = 1, 3
        orig = bytearray([1, 2, 3, 4, 5, 6, 7, 8, 9])  # rgb8, 3 pixels
        img = create_image(h, w, "rgb8", w * 3, orig)

        # With copy=False, the array should not own its data (view into internal buffer)
        view_arr = ros2_image_to_buffer(img, copy=False)
        self.assertFalse(view_arr.flags["OWNDATA"])
        self.assertEqual(view_arr.shape, (h, w, 3))
        self.assertEqual(view_arr[0, 0, 0], 1)

        # With copy=True, the array should own its data (independent copy)
        copied = ros2_image_to_buffer(img, copy=True)
        self.assertTrue(copied.flags["OWNDATA"])
        self.assertEqual(copied.shape, (h, w, 3))
        self.assertEqual(copied[0, 0, 0], 1)
