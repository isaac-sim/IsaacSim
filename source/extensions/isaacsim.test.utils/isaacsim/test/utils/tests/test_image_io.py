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

"""Test suite for image I/O utilities in Isaac Sim."""

from __future__ import annotations

import io
import os
import tempfile
from unittest.mock import patch

import numpy as np
from isaacsim.test.utils.image_io import (
    read_image_as_array,
    save_depth_image,
    save_rgb_image,
)
from isaacsim.test.utils.timed_async_test import TimedAsyncTestCase
from PIL import Image


class TestImageIO(TimedAsyncTestCase):
    """Test suite for image I/O utilities."""

    async def setUp(self) -> None:
        """Set up test fixtures."""
        await super().setUp()
        self.test_dir = tempfile.mkdtemp()

    async def tearDown(self) -> None:
        """Clean up test fixtures."""
        # Clean up temporary files
        import shutil

        if os.path.exists(self.test_dir):
            shutil.rmtree(self.test_dir)
        await super().tearDown()

    def test_save_rgb_image_basic_functionality(self) -> None:
        """Test save_rgb_image with basic functionality and directory creation."""
        # Arrange RGB data and save to disk; verify file path and mode.
        resolution = (480, 320)
        rgb_data = np.random.randint(0, 255, (resolution[1], resolution[0], 3), dtype=np.uint8)
        file_name = "test_image.png"

        save_rgb_image(rgb_data, self.test_dir, file_name)

        file_path = os.path.join(self.test_dir, file_name)
        self.assertTrue(os.path.exists(file_path))

        saved_image = Image.open(file_path)
        self.assertEqual(saved_image.mode, "RGBA")  # PNG converts to RGBA

        # Verify nested directory creation when the path does not yet exist.
        nested_dir = os.path.join(self.test_dir, "nested", "subdir")
        nested_file = "nested_test.png"

        save_rgb_image(rgb_data, nested_dir, nested_file)

        self.assertTrue(os.path.exists(nested_dir))
        nested_path = os.path.join(nested_dir, nested_file)
        self.assertTrue(os.path.exists(nested_path))

    def test_save_rgb_image_formats(self) -> None:
        """Test save_rgb_image with different data types and file formats."""
        # Test PNG/JPG/JPEG writers with 3-channel data.
        resolution = (480, 320)
        rgb_data = np.random.randint(0, 255, (resolution[1], resolution[0], 3), dtype=np.uint8)
        formats = ["test.png", "test.jpg", "test.jpeg"]

        for file_name in formats:
            save_rgb_image(rgb_data, self.test_dir, file_name)

            file_path = os.path.join(self.test_dir, file_name)
            self.assertTrue(os.path.exists(file_path))
            saved_image = Image.open(file_path)
            self.assertIsNotNone(saved_image)

        # RGBA must be saved as PNG (alpha channel supported).
        rgba_data = np.random.randint(0, 255, (resolution[1], resolution[0], 4), dtype=np.uint8)
        save_rgb_image(rgba_data, self.test_dir, "rgba_test.png")

        rgba_path = os.path.join(self.test_dir, "rgba_test.png")
        self.assertTrue(os.path.exists(rgba_path))
        saved_rgba = Image.open(rgba_path)
        self.assertEqual(saved_rgba.mode, "RGBA")

        # JPEG cannot encode alpha; expect ValueError for RGBA inputs.
        for ext in ["jpg", "jpeg"]:
            with self.assertRaises(ValueError) as context:
                save_rgb_image(rgba_data, self.test_dir, f"rgba_test.{ext}")

            self.assertIn("Cannot save RGBA data", str(context.exception))
            self.assertIn("JPEG format doesn't support transparency", str(context.exception))

    def test_save_rgb_image_file_operations(self) -> None:
        """Test save_rgb_image with existing directories and file overwriting."""
        resolution = (480, 320)
        rgb_data = np.random.randint(0, 255, (resolution[1], resolution[0], 3), dtype=np.uint8)

        # Test saving to existing directory
        file_name = "existing_dir_test.png"
        save_rgb_image(rgb_data, self.test_dir, file_name)

        file_path = os.path.join(self.test_dir, file_name)
        self.assertTrue(os.path.exists(file_path))

        # Test file overwriting with different data
        black_data = np.zeros((resolution[1], resolution[0], 3), dtype=np.uint8)  # All black
        white_data = np.ones((resolution[1], resolution[0], 3), dtype=np.uint8) * 255  # All white
        overwrite_file = "overwrite_test.png"
        overwrite_path = os.path.join(self.test_dir, overwrite_file)

        # Save first image (black)
        save_rgb_image(black_data, self.test_dir, overwrite_file)

        self.assertTrue(os.path.exists(overwrite_path))
        first_image = Image.open(overwrite_path)
        first_array = np.array(first_image.convert("RGB"))

        # Save second image (white) - should overwrite
        save_rgb_image(white_data, self.test_dir, overwrite_file)

        second_image = Image.open(overwrite_path)
        second_array = np.array(second_image.convert("RGB"))

        # Verify images are different (black vs white)
        self.assertFalse(np.array_equal(first_array, second_array))

    def test_save_depth_image_grayscale(self) -> None:
        """Test save_depth_image with grayscale output."""
        # Test basic float32 depth data normalization
        resolution = (480, 320)
        depth_data = np.random.rand(resolution[1], resolution[0]).astype(np.float32) * 10.0
        file_name = "test_depth_gray.png"

        save_depth_image(depth_data, self.test_dir, file_name, normalize=True)

        file_path = os.path.join(self.test_dir, file_name)
        self.assertTrue(os.path.exists(file_path))

        # Check saved image
        saved_image = Image.open(file_path)
        self.assertEqual(saved_image.mode, "L")  # Grayscale
        self.assertEqual(saved_image.size, (resolution[0], resolution[1]))

    def test_save_depth_image_without_normalization(self) -> None:
        """Test save_depth_image without normalization."""
        # Test with uint8 data that doesn't need normalization
        resolution = (480, 320)
        depth_data = np.random.randint(0, 255, (resolution[1], resolution[0]), dtype=np.uint8)
        file_name = "test_depth_no_norm.png"

        save_depth_image(depth_data, self.test_dir, file_name, normalize=False)

        file_path = os.path.join(self.test_dir, file_name)
        self.assertTrue(os.path.exists(file_path))

        saved_image = Image.open(file_path)
        self.assertEqual(saved_image.mode, "L")  # Always L mode for non-TIFF

        # Test with float data in 0-1 range
        depth_data_float = np.random.rand(resolution[1], resolution[0]).astype(np.float32)
        file_name_float = "test_depth_float_no_norm.png"

        save_depth_image(depth_data_float, self.test_dir, file_name_float, normalize=False)

        file_path_float = os.path.join(self.test_dir, file_name_float)
        self.assertTrue(os.path.exists(file_path_float))

    def test_save_depth_image_tiff_with_normalize_warning(self) -> None:
        """Test that TIFF with normalize=True ignores normalize and saves as float32 TIFF."""
        resolution = (480, 320)
        depth_data = np.random.rand(resolution[1], resolution[0]).astype(np.float32) * 10.0
        tiff_file_name = "test_depth_normalize_ignored.tiff"

        # This should save as TIFF (ignoring normalize=True) and print a warning
        stdout_capture = io.StringIO()
        with patch("sys.stdout", stdout_capture):
            save_depth_image(depth_data, self.test_dir, tiff_file_name, normalize=True)
        captured_output = stdout_capture.getvalue()
        self.assertIn(
            "Warning: TIFF format requested with normalize=True",
            captured_output,
            "Should warn about normalize being ignored for TIFF",
        )

        # Check that TIFF was saved (not PNG)
        tiff_path = os.path.join(self.test_dir, tiff_file_name)
        self.assertTrue(os.path.exists(tiff_path), "TIFF file should exist")

        # Verify the saved TIFF is float32 format
        saved_image = Image.open(tiff_path)
        self.assertEqual(saved_image.mode, "F")  # F mode = 32-bit float

        # Verify data integrity - should be the same as original
        tiff_data = np.array(saved_image)
        self.assertEqual(tiff_data.shape, depth_data.shape, "TIFF shape should match original data")
        is_close = np.allclose(tiff_data, depth_data, rtol=1e-5, atol=1e-5, equal_nan=True)
        self.assertTrue(is_close, "TIFF should contain original float32 data, not normalized")

    def test_read_image_as_array_basic_functionality(self) -> None:
        """Test read_image_as_array with basic functionality."""
        # Create a test image first
        resolution = (480, 320)
        rgb_data = np.random.randint(0, 255, (resolution[1], resolution[0], 3), dtype=np.uint8)
        file_name = "test_read_image.png"
        file_path = os.path.join(self.test_dir, file_name)

        save_rgb_image(rgb_data, self.test_dir, file_name)

        # Read the image back as array
        read_array = read_image_as_array(file_path)

        # Verify the array properties
        self.assertIsInstance(read_array, np.ndarray)
        self.assertEqual(len(read_array.shape), 3)  # Should be 3D for RGBA
        self.assertEqual(read_array.shape[:2], (resolution[1], resolution[0]))  # Height, Width
        self.assertEqual(read_array.shape[2], 4)  # RGBA channels

    def test_read_image_as_array_channel_handling(self) -> None:
        """Test read_image_as_array with different image types and channel handling."""
        resolution = (480, 320)

        # Test with RGB image (should always be 3D)
        rgb_data = np.random.randint(0, 255, (resolution[1], resolution[0], 3), dtype=np.uint8)
        rgb_file = "test_rgb.png"
        rgb_path = os.path.join(self.test_dir, rgb_file)
        save_rgb_image(rgb_data, self.test_dir, rgb_file)

        rgb_array = read_image_as_array(rgb_path)
        self.assertEqual(len(rgb_array.shape), 3)  # RGB should be 3D
        self.assertEqual(rgb_array.shape[2], 4)  # RGBA after PNG conversion

        # Test with grayscale image (PIL loads as 2D)
        depth_data = np.random.rand(resolution[1], resolution[0]).astype(np.float32)
        gray_file = "test_gray.png"
        gray_path = os.path.join(self.test_dir, gray_file)
        save_depth_image(depth_data, self.test_dir, gray_file, normalize=True)

        # Both squeeze settings should give same result for grayscale (PIL loads as 2D)
        gray_array_squeezed = read_image_as_array(gray_path, squeeze_singleton_channel=True)
        gray_array_unsqueezed = read_image_as_array(gray_path, squeeze_singleton_channel=False)

        self.assertEqual(len(gray_array_squeezed.shape), 2)  # Grayscale is 2D
        self.assertEqual(len(gray_array_unsqueezed.shape), 2)  # Still 2D (PIL behavior)
        self.assertEqual(gray_array_squeezed.shape, (resolution[1], resolution[0]))

        # Test the squeezing functionality by manually creating a 3D array scenario
        # We'll test this by directly manipulating arrays rather than relying on PIL's loading behavior
        test_array_3d = np.random.randint(0, 255, (100, 100, 1), dtype=np.uint8)
        test_array_2d = test_array_3d[:, :, 0]  # Manually squeeze

        # Verify our squeezing logic would work
        if test_array_3d.ndim == 3 and test_array_3d.shape[2] == 1:
            manually_squeezed = test_array_3d[:, :, 0]
            self.assertTrue(np.array_equal(manually_squeezed, test_array_2d))
            self.assertEqual(len(manually_squeezed.shape), 2)

    def test_read_image_as_array_error_handling(self) -> None:
        """Test read_image_as_array error handling."""
        # Test with non-existent file
        non_existent_path = os.path.join(self.test_dir, "non_existent.png")
        with self.assertRaises(FileNotFoundError) as context:
            read_image_as_array(non_existent_path)
        self.assertIn("Image file not found", str(context.exception))

        # Test with invalid file (create a text file with .png extension)
        invalid_image_path = os.path.join(self.test_dir, "invalid.png")
        with open(invalid_image_path, "w") as f:
            f.write("This is not an image file")

        with self.assertRaises(IOError) as context:
            read_image_as_array(invalid_image_path)
        self.assertIn("Failed to read image file", str(context.exception))
