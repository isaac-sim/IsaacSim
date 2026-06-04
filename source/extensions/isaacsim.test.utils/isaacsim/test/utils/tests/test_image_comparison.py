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

"""Tests for image comparison functionality in Isaac Sim testing utilities."""

from __future__ import annotations

import io
import os
import tempfile
from unittest.mock import patch

import numpy as np
import omni.kit.app
from isaacsim.test.utils.image_comparison import (
    compare_arrays_within_tolerances,
    compare_images_in_directories,
    compare_images_within_tolerances,
)
from isaacsim.test.utils.timed_async_test import TimedAsyncTestCase
from PIL import Image


class TestImageComparison(TimedAsyncTestCase):
    """Test suite for image comparison utilities."""

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

    def get_image_comparison_golden_dir(self) -> str:
        """Resolve path to golden assets for image comparison tests.

        Returns:
            Path to the golden image comparison directory.
        """
        # Resolve path to golden assets for image comparison tests.
        ext_manager = omni.kit.app.get_app().get_extension_manager()
        ext_id = ext_manager.get_enabled_extension_id("isaacsim.test.utils")
        extension_path = ext_manager.get_extension_path(ext_id)
        return os.path.join(extension_path, "data", "golden", "image_comparison")

    async def _test_golden_pair(
        self,
        img1_name: str,
        img2_name: str,
        mean_tol: float,
        abs_tol: float,
        perc: tuple[float, float],
        description: str,
    ) -> None:
        """Helper to test golden image pairs with given tolerances.

        Args:
            img1_name: File name of the first image under the golden directory.
            img2_name: File name of the second image under the golden directory.
            mean_tol: Mean difference threshold.
            abs_tol: Absolute difference threshold.
            perc: Percentile threshold tuple as (percentile, tolerance).
            description: Human-readable description of the pair.

        Raises:
            AssertionError: If the similarity check fails or images are missing.
        """
        golden_dir = self.get_image_comparison_golden_dir()
        img1 = os.path.join(golden_dir, img1_name)
        img2 = os.path.join(golden_dir, img2_name)

        if not (os.path.exists(img1) and os.path.exists(img2)):
            self.fail(f"Missing golden images for {description}. Expected: {img1} and {img2}")

        stdout_capture = io.StringIO()
        with patch("sys.stdout", stdout_capture):
            res = compare_images_within_tolerances(
                golden_file_path=img1,
                test_file_path=img2,
                allclose_rtol=None,
                allclose_atol=None,
                mean_tolerance=mean_tol,
                absolute_tolerance=abs_tol,
                percentile_tolerance=perc,
                print_all_stats=True,
            )
        is_similar = bool(res["passed"])
        captured_stdout = stdout_capture.getvalue()
        self.assertTrue(
            is_similar,
            f"{description} exceeded thresholds: mean<={mean_tol}, abs<={abs_tol}, {perc[0]}th<={perc[1]}.\n"
            f"Captured stdout:\n{captured_stdout}",
        )

    async def test_are_arrays_similar_identical(self) -> None:
        """Test comparing identical arrays using default tolerances."""
        # Arrange identical arrays and expect equality under defaults.
        resolution = (100, 100, 3)

        golden = np.zeros(resolution, dtype=np.uint8)
        test = np.zeros(resolution, dtype=np.uint8)

        # Should pass with default numpy allclose tolerances.
        res = compare_arrays_within_tolerances(golden, test)
        is_similar = bool(res["passed"])
        self.assertTrue(
            is_similar, "Identical arrays should be similar with default tolerances (rtol=1e-05, atol=1e-08)"
        )

    async def test_are_arrays_similar_different_resolutions(self) -> None:
        """Test arrays with different resolutions should raise a shape mismatch error."""
        golden = np.zeros((100, 100, 3), dtype=np.uint8)
        test = np.zeros((120, 100, 3), dtype=np.uint8)

        with self.assertRaises(ValueError) as context:
            compare_arrays_within_tolerances(golden, test)

        self.assertIn("shape", str(context.exception).lower())

    async def test_are_arrays_similar_with_small_differences(self) -> None:
        """Test comparing arrays with small differences."""
        # Arrange nearly equal arrays; then relax tolerances to pass.
        resolution = (100, 100, 3)
        base_value = 50
        diff_value = 1
        rtol = 0.1
        atol = 2.0
        mean_tol = 2.0

        golden = np.ones(resolution, dtype=np.uint8) * base_value
        test = np.ones(resolution, dtype=np.uint8) * (base_value + diff_value)

        # Default tolerances should fail.
        res = compare_arrays_within_tolerances(golden, test)
        is_similar = bool(res["passed"])
        self.assertFalse(
            is_similar,
            f"Arrays with difference of {diff_value} ({base_value} vs {base_value + diff_value}) should fail default tolerances",
        )

        # Relaxed tolerances should pass.
        res = compare_arrays_within_tolerances(
            golden,
            test,
            allclose_rtol=rtol,
            allclose_atol=atol,
            mean_tolerance=mean_tol,
            print_all_stats=True,
        )
        is_similar = bool(res["passed"])
        self.assertTrue(
            is_similar,
            f"Arrays with difference of {diff_value} should pass with rtol={rtol}, atol={atol}, mean_tol={mean_tol}",
        )

    async def test_are_arrays_similar_disable_allclose(self) -> None:
        """Test disabling allclose and using only mean tolerance."""
        # Disable allclose to rely only on mean tolerance.
        resolution = (100, 100, 3)
        diff_value = 5
        mean_tol = 10.0

        golden = np.zeros(resolution, dtype=np.uint8)
        test = np.ones(resolution, dtype=np.uint8) * diff_value

        # Default: allclose will fail.
        res = compare_arrays_within_tolerances(golden, test)
        is_similar = bool(res["passed"])
        self.assertFalse(
            is_similar,
            f"Arrays with difference of {diff_value} (0 vs {diff_value}) should fail default allclose tolerances",
        )

        # Disable allclose; rely only on mean tolerance.
        res = compare_arrays_within_tolerances(
            golden,
            test,
            allclose_rtol=None,
            allclose_atol=None,
            mean_tolerance=mean_tol,
            print_all_stats=True,
        )
        is_similar = bool(res["passed"])
        self.assertTrue(
            is_similar,
            f"Arrays with mean difference of {diff_value} should pass when allclose disabled and mean_tolerance={mean_tol}",
        )

    async def test_are_arrays_similar_multiple_criteria(self) -> None:
        """Test using multiple comparison criteria."""
        # Combine multiple criteria and exercise both pass and fail.
        resolution = (100, 100, 3)
        diff_value = 10
        rtol = 0.2
        atol = 15.0
        mean_tol = 15.0
        max_tol = 15.0
        rmse_tol_pass = 12.0
        rmse_tol_fail = 8.0

        golden = np.zeros(resolution, dtype=np.uint8)
        test = np.ones(resolution, dtype=np.uint8) * diff_value

        # Multiple criteria should pass.
        res = compare_arrays_within_tolerances(
            golden,
            test,
            allclose_rtol=rtol,
            allclose_atol=atol,
            mean_tolerance=mean_tol,
            max_tolerance=max_tol,
            rmse_tolerance=rmse_tol_pass,
            print_all_stats=True,
        )
        is_similar = bool(res["passed"])
        self.assertTrue(
            is_similar,
            f"Arrays with diff={diff_value} should pass all criteria: rtol={rtol}, atol={atol}, mean={mean_tol}, max={max_tol}, rmse={rmse_tol_pass}",
        )

        # Tighten RMSE to force a failure.
        res = compare_arrays_within_tolerances(
            golden,
            test,
            allclose_rtol=rtol,
            allclose_atol=atol,
            mean_tolerance=mean_tol,
            max_tolerance=max_tol,
            rmse_tolerance=rmse_tol_fail,  # Too tight
            print_all_stats=True,
        )
        is_similar = bool(res["passed"])
        self.assertFalse(
            is_similar,
            f"Arrays with diff={diff_value} should fail when RMSE tolerance tightened to {rmse_tol_fail} (actual RMSE={diff_value})",
        )

    async def test_are_arrays_similar_shape_mismatch(self) -> None:
        """Test comparing arrays with different shapes."""
        golden = np.zeros((100, 100, 3), dtype=np.uint8)
        test = np.zeros((50, 50, 3), dtype=np.uint8)  # Different shape

        # Should raise ValueError
        with self.assertRaises(ValueError) as context:
            compare_arrays_within_tolerances(golden, test)

        self.assertIn(
            "shape",
            str(context.exception).lower(),
            "Shape mismatch (100x100x3 vs 50x50x3) should raise ValueError mentioning 'shape'",
        )

    async def test_are_arrays_similar_percentile_tolerance(self) -> None:
        """Test percentile-based comparison."""
        # Introduce a small outlier region and test percentile thresholds.
        resolution = (100, 100)
        outlier_region = (5, 5)  # 5x5 region
        outlier_value = 255
        percentile_low = 95
        percentile_high = 99.9
        tolerance = 1.0
        total_pixels = resolution[0] * resolution[1]
        outlier_pixels = outlier_region[0] * outlier_region[1]

        # Create test data where most pixels are similar but a few are very different
        golden = np.zeros(resolution, dtype=np.uint8)
        test = np.zeros(resolution, dtype=np.uint8)
        test[0 : outlier_region[0], 0 : outlier_region[1]] = outlier_value

        # 95th percentile should be 0 (most pixels are identical).
        res = compare_arrays_within_tolerances(
            golden,
            test,
            allclose_rtol=None,
            allclose_atol=None,
            percentile_tolerance=(percentile_low, tolerance),
        )
        is_similar = bool(res["passed"])
        self.assertTrue(
            is_similar,
            f"{percentile_low}th percentile=0 ({total_pixels-outlier_pixels}/{total_pixels} pixels identical) should pass with tolerance={tolerance}",
        )

        # 99.9th percentile should catch the outliers.
        res = compare_arrays_within_tolerances(
            golden,
            test,
            allclose_rtol=None,
            allclose_atol=None,
            percentile_tolerance=(percentile_high, tolerance),
        )
        is_similar = bool(res["passed"])
        self.assertFalse(
            is_similar,
            f"{percentile_high}th percentile={outlier_value} ({outlier_pixels}/{total_pixels} pixels={outlier_value}) should fail with tolerance={tolerance}",
        )

    async def test_are_images_similar_identical_files(self) -> None:
        """Test comparing identical image files."""
        # Create identical images and expect equality under defaults.
        resolution = (100, 100, 3)

        # Create identical test images
        image_data = np.random.randint(0, 255, resolution, dtype=np.uint8)
        golden_path = os.path.join(self.test_dir, "golden.png")
        test_path = os.path.join(self.test_dir, "test.png")

        Image.fromarray(image_data).save(golden_path)
        Image.fromarray(image_data).save(test_path)

        # Capture stdout to keep CI logs concise.
        stdout_capture = io.StringIO()
        with patch("sys.stdout", stdout_capture):
            res = compare_images_within_tolerances(golden_path, test_path, print_all_stats=True)
        captured_stdout = stdout_capture.getvalue()
        is_similar = bool(res["passed"])
        self.assertTrue(
            is_similar,
            "Identical image files should be similar with default tolerances.\n" f"Captured stdout:\n{captured_stdout}",
        )

    async def test_are_images_similar_different_files(self) -> None:
        """Test comparing different image files."""
        # Create dissimilar images; then relax tolerances to pass.
        resolution = (100, 100, 3)
        diff_value = 100
        mean_tol = 110.0
        rtol = 0.5
        atol = 150.0

        # Create different test images
        golden_data = np.zeros(resolution, dtype=np.uint8)
        test_data = np.ones(resolution, dtype=np.uint8) * diff_value

        golden_path = os.path.join(self.test_dir, "golden.png")
        test_path = os.path.join(self.test_dir, "test.png")

        Image.fromarray(golden_data).save(golden_path)
        Image.fromarray(test_data).save(test_path)

        # Default tolerances should fail; capture stdout.
        stdout_capture = io.StringIO()
        with patch("sys.stdout", stdout_capture):
            res = compare_images_within_tolerances(golden_path, test_path, print_all_stats=True)
        captured_stdout = stdout_capture.getvalue()
        is_similar = bool(res["passed"])
        self.assertFalse(
            is_similar, f"Images with difference of {diff_value} (0 vs {diff_value}) should fail default tolerances"
        )

        # Relaxed tolerances should pass; capture stdout.
        stdout_capture = io.StringIO()
        with patch("sys.stdout", stdout_capture):
            res = compare_images_within_tolerances(
                golden_path,
                test_path,
                mean_tolerance=mean_tol,
                allclose_rtol=rtol,
                allclose_atol=atol,
                print_all_stats=True,
            )
        captured_stdout = stdout_capture.getvalue()
        is_similar = bool(res["passed"])
        self.assertTrue(
            is_similar,
            f"Images with diff={diff_value} should pass with mean_tol={mean_tol}, rtol={rtol}, atol={atol}.\n"
            f"Captured stdout:\n{captured_stdout}",
        )

    async def test_are_images_similar_missing_file(self) -> None:
        """Test comparing when one image file doesn't exist."""
        # Only create the golden image; expect FileNotFoundError for test image.
        resolution = (100, 100, 3)

        golden_path = os.path.join(self.test_dir, "golden.png")
        test_path = os.path.join(self.test_dir, "nonexistent.png")

        # Create only the golden image
        golden_data = np.zeros(resolution, dtype=np.uint8)
        Image.fromarray(golden_data).save(golden_path)

        # Should raise FileNotFoundError
        with self.assertRaises(FileNotFoundError) as context:
            compare_images_within_tolerances(golden_path, test_path)

        self.assertIn(
            "test image file not found",
            str(context.exception).lower(),
            "Missing test image file should raise FileNotFoundError with message about 'test image file not found'",
        )

    async def test_are_images_similar_tiff_format(self) -> None:
        """Test comparing TIFF images with float32 data."""
        # Compare float32 TIFFs with small Gaussian noise under tight tolerances.
        resolution = (100, 100)
        data_scale = 10.0
        noise_mean = 0
        noise_std = 0.001
        rtol = 1e-3
        atol = 5e-3

        # Create float32 depth data
        golden_data = np.random.rand(*resolution).astype(np.float32) * data_scale
        test_data = golden_data + np.random.normal(noise_mean, noise_std, golden_data.shape).astype(np.float32)

        golden_path = os.path.join(self.test_dir, "golden.tiff")
        test_path = os.path.join(self.test_dir, "test.tiff")

        # Save arrays as float32 TIFFs.
        Image.fromarray(golden_data, mode="F").save(golden_path)
        Image.fromarray(test_data, mode="F").save(test_path)

        # Should pass with appropriate tolerances for float data
        # Capture stdout to keep CI logs concise.
        stdout_capture = io.StringIO()
        with patch("sys.stdout", stdout_capture):
            res = compare_images_within_tolerances(
                golden_path,
                test_path,
                allclose_rtol=rtol,
                allclose_atol=atol,
                print_all_stats=True,
            )
        captured_stdout = stdout_capture.getvalue()
        is_similar = bool(res["passed"])
        self.assertTrue(
            is_similar,
            f"TIFF float32 images with small noise (std={noise_std}) should pass with rtol={rtol}, atol={atol}.\n"
            f"Captured stdout:\n{captured_stdout}",
        )

    async def test_compare_rotated_drill_pair(self) -> None:
        """Compare rotated drill RGB images using fixed thresholds and print stats."""
        await self._test_golden_pair(
            "similar_rgb_rotated_drill_1.png",
            "similar_rgb_rotated_drill_2.png",
            mean_tol=4.0,
            abs_tol=240.0,
            perc=(99, 110.0),
            description="Rotated drill pair",
        )

    async def test_compare_shifted_warehouse_pair(self) -> None:
        """Compare shifted warehouse RGB images using fixed thresholds and print stats."""
        await self._test_golden_pair(
            "similar_rgb_shifted_warehouse_1.png",
            "similar_rgb_shifted_warehouse_2.png",
            mean_tol=10.0,
            abs_tol=250.0,
            perc=(99, 125.0),
            description="Shifted warehouse pair",
        )

    async def test_compare_depth_8bit_pair(self) -> None:
        """Compare 8-bit depth visualization image pair using fixed thresholds and print stats."""
        await self._test_golden_pair(
            "similar_depth_8bit_1.png",
            "similar_depth_8bit_2.png",
            mean_tol=5.0,
            abs_tol=180.0,
            perc=(99, 130.0),
            description="Depth 8-bit pair",
        )

    async def test_compare_different_extension_pair(self) -> None:
        """Compare RGB images with different file extensions (JPG vs PNG) and print stats."""
        await self._test_golden_pair(
            "different_extension_rgb_1.jpg",
            "different_extension_rgb_2.png",
            mean_tol=2.0,
            abs_tol=95.0,
            perc=(99, 10.0),
            description="Different extension pair",
        )

    async def test_different_resolution_image_pair_raises(self) -> None:
        """Different resolution image pair should raise a shape mismatch error."""
        golden_dir = self.get_image_comparison_golden_dir()
        img1 = os.path.join(golden_dir, "different_resolution_rgb_1.png")
        img2 = os.path.join(golden_dir, "different_resolution_rgb_2.png")

        if not (os.path.exists(img1) and os.path.exists(img2)):
            self.fail(f"Missing golden images for different resolution pair. Expected: {img1} and {img2}")

        with self.assertRaises(ValueError) as context:
            compare_images_within_tolerances(
                golden_file_path=img1,
                test_file_path=img2,
            )

        self.assertIn("shape", str(context.exception).lower())

    async def test_compare_forklift_rt16_vs_rt64(self) -> None:
        """Compare forklift RGB images (rt16 vs rt64) using fixed thresholds and print stats."""
        await self._test_golden_pair(
            "similar_rgb_forklift_rt16.png",
            "similar_rgb_forklift_rt64.png",
            mean_tol=3.0,
            abs_tol=130.0,
            perc=(99, 25.0),
            description="Forklift rt16 vs rt64",
        )

    async def test_compare_semantic_segmentation_pair(self) -> None:
        """Compare semantic segmentation RGB images using fixed thresholds and print stats."""
        await self._test_golden_pair(
            "similar_rgb_semantic_segmentation_1.png",
            "similar_rgb_semantic_segmentation_2.png",
            mean_tol=8.0,
            abs_tol=235.0,
            perc=(95, 60.0),
            description="Semantic segmentation pair",
        )

    async def test_compare_images_in_directories_identical_files(self) -> None:
        """Test comparing identical files in two directories."""
        golden_dir = os.path.join(self.test_dir, "golden")
        test_dir = os.path.join(self.test_dir, "test")
        os.makedirs(golden_dir)
        os.makedirs(test_dir)

        resolution = (100, 100, 3)
        for i in range(3):
            image_data = np.random.randint(0, 255, resolution, dtype=np.uint8)
            golden_path = os.path.join(golden_dir, f"image_{i:03d}.png")
            test_path = os.path.join(test_dir, f"image_{i:03d}.png")
            Image.fromarray(image_data).save(golden_path)
            Image.fromarray(image_data).save(test_path)

        stdout_capture = io.StringIO()
        with patch("sys.stdout", stdout_capture):
            result = compare_images_in_directories(
                golden_dir=golden_dir, test_dir=test_dir, path_pattern=None, print_per_file_results=True
            )

        self.assertTrue(result["all_passed"], "Identical files in both directories should pass")
        self.assertEqual(result["passed_count"], 3, "All 3 files should pass")
        self.assertEqual(result["failed_count"], 0, "No files should fail")
        self.assertTrue(result["file_list_match"], "File lists should match")
        self.assertEqual(len(result["golden_only_files"]), 0, "No golden-only files")
        self.assertEqual(len(result["test_only_files"]), 0, "No test-only files")

    async def test_compare_images_in_directories_with_pattern(self) -> None:
        """Test comparing files with regex pattern matching."""
        golden_dir = os.path.join(self.test_dir, "golden")
        test_dir = os.path.join(self.test_dir, "test")
        os.makedirs(golden_dir)
        os.makedirs(test_dir)

        resolution = (100, 100, 3)
        for i in range(2):
            image_data = np.random.randint(0, 255, resolution, dtype=np.uint8)
            Image.fromarray(image_data).save(os.path.join(golden_dir, f"rgb_{i:03d}.png"))
            Image.fromarray(image_data).save(os.path.join(test_dir, f"rgb_{i:03d}.png"))
            Image.fromarray(image_data).save(os.path.join(golden_dir, f"depth_{i:03d}.png"))
            Image.fromarray(image_data).save(os.path.join(test_dir, f"depth_{i:03d}.png"))

        stdout_capture = io.StringIO()
        with patch("sys.stdout", stdout_capture):
            result = compare_images_in_directories(
                golden_dir=golden_dir,
                test_dir=test_dir,
                path_pattern=r"^rgb_.*\.png$",
                print_per_file_results=True,
            )

        self.assertTrue(result["all_passed"], "RGB files should pass")
        self.assertEqual(result["passed_count"], 2, "Only 2 RGB files should be compared")
        self.assertEqual(len(result["file_results"]), 2, "Should have 2 file results")
        self.assertIn("rgb_000.png", result["file_results"])
        self.assertIn("rgb_001.png", result["file_results"])
        self.assertNotIn("depth_000.png", result["file_results"])

    async def test_compare_images_in_directories_golden_has_extra_files(self) -> None:
        """Test when golden directory has more files than test directory."""
        golden_dir = os.path.join(self.test_dir, "golden")
        test_dir = os.path.join(self.test_dir, "test")
        os.makedirs(golden_dir)
        os.makedirs(test_dir)

        resolution = (100, 100, 3)
        image_data = np.random.randint(0, 255, resolution, dtype=np.uint8)

        Image.fromarray(image_data).save(os.path.join(golden_dir, "image_000.png"))
        Image.fromarray(image_data).save(os.path.join(golden_dir, "image_001.png"))
        Image.fromarray(image_data).save(os.path.join(golden_dir, "image_002.png"))
        Image.fromarray(image_data).save(os.path.join(test_dir, "image_000.png"))
        Image.fromarray(image_data).save(os.path.join(test_dir, "image_001.png"))

        stdout_capture = io.StringIO()
        with patch("sys.stdout", stdout_capture):
            result = compare_images_in_directories(
                golden_dir=golden_dir, test_dir=test_dir, path_pattern=None, print_per_file_results=True
            )

        captured_output = stdout_capture.getvalue()
        self.assertFalse(result["all_passed"], "Should fail when file lists don't match")
        self.assertFalse(result["file_list_match"], "File lists should not match")
        self.assertEqual(result["passed_count"], 2, "Common files should pass")
        self.assertEqual(len(result["golden_only_files"]), 1, "Should have 1 golden-only file")
        self.assertEqual(len(result["test_only_files"]), 0, "Should have 0 test-only files")
        self.assertIn("image_002.png", result["golden_only_files"])
        self.assertIn("WARNING", captured_output)

    async def test_compare_images_in_directories_test_has_extra_files(self) -> None:
        """Test when test directory has more files than golden directory."""
        golden_dir = os.path.join(self.test_dir, "golden")
        test_dir = os.path.join(self.test_dir, "test")
        os.makedirs(golden_dir)
        os.makedirs(test_dir)

        resolution = (100, 100, 3)
        image_data = np.random.randint(0, 255, resolution, dtype=np.uint8)

        Image.fromarray(image_data).save(os.path.join(golden_dir, "image_000.png"))
        Image.fromarray(image_data).save(os.path.join(golden_dir, "image_001.png"))
        Image.fromarray(image_data).save(os.path.join(test_dir, "image_000.png"))
        Image.fromarray(image_data).save(os.path.join(test_dir, "image_001.png"))
        Image.fromarray(image_data).save(os.path.join(test_dir, "image_002.png"))

        stdout_capture = io.StringIO()
        with patch("sys.stdout", stdout_capture):
            result = compare_images_in_directories(
                golden_dir=golden_dir, test_dir=test_dir, path_pattern=None, print_per_file_results=True
            )

        captured_output = stdout_capture.getvalue()
        self.assertFalse(result["all_passed"], "Should fail when file lists don't match")
        self.assertFalse(result["file_list_match"], "File lists should not match")
        self.assertEqual(result["passed_count"], 2, "Common files should pass")
        self.assertEqual(len(result["golden_only_files"]), 0, "Should have 0 golden-only files")
        self.assertEqual(len(result["test_only_files"]), 1, "Should have 1 test-only file")
        self.assertIn("image_002.png", result["test_only_files"])
        self.assertIn("WARNING", captured_output)

    async def test_compare_images_in_directories_no_common_files(self) -> None:
        """Test when directories have no common files."""
        golden_dir = os.path.join(self.test_dir, "golden")
        test_dir = os.path.join(self.test_dir, "test")
        os.makedirs(golden_dir)
        os.makedirs(test_dir)

        resolution = (100, 100, 3)
        image_data = np.random.randint(0, 255, resolution, dtype=np.uint8)

        Image.fromarray(image_data).save(os.path.join(golden_dir, "golden_image.png"))
        Image.fromarray(image_data).save(os.path.join(test_dir, "test_image.png"))

        stdout_capture = io.StringIO()
        with patch("sys.stdout", stdout_capture):
            result = compare_images_in_directories(
                golden_dir=golden_dir, test_dir=test_dir, path_pattern=None, print_per_file_results=True
            )

        captured_output = stdout_capture.getvalue()
        self.assertFalse(result["all_passed"], "Should fail when no common files")
        self.assertFalse(result["file_list_match"], "File lists should not match")
        self.assertEqual(result["passed_count"], 0, "No files should pass")
        self.assertEqual(result["failed_count"], 0, "No files should fail")
        self.assertEqual(len(result["file_results"]), 0, "Should have no file results")
        self.assertIn("No common files", captured_output)

    async def test_compare_images_in_directories_with_differences(self) -> None:
        """Test comparing files with differences using mean tolerance."""
        golden_dir = os.path.join(self.test_dir, "golden")
        test_dir = os.path.join(self.test_dir, "test")
        os.makedirs(golden_dir)
        os.makedirs(test_dir)

        resolution = (100, 100, 3)
        diff_value = 5

        for i in range(2):
            golden_data = np.ones(resolution, dtype=np.uint8) * 100
            test_data = np.ones(resolution, dtype=np.uint8) * (100 + diff_value)
            Image.fromarray(golden_data).save(os.path.join(golden_dir, f"image_{i}.png"))
            Image.fromarray(test_data).save(os.path.join(test_dir, f"image_{i}.png"))

        stdout_capture = io.StringIO()
        with patch("sys.stdout", stdout_capture):
            result = compare_images_in_directories(
                golden_dir=golden_dir,
                test_dir=test_dir,
                path_pattern=None,
                allclose_rtol=None,
                allclose_atol=None,
                mean_tolerance=10.0,
                print_per_file_results=True,
            )

        self.assertTrue(result["all_passed"], f"Files with mean diff {diff_value} should pass with tolerance 10.0")
        self.assertEqual(result["passed_count"], 2, "Both files should pass")
        self.assertEqual(result["failed_count"], 0, "No files should fail")

        stdout_capture = io.StringIO()
        with patch("sys.stdout", stdout_capture):
            result = compare_images_in_directories(
                golden_dir=golden_dir,
                test_dir=test_dir,
                path_pattern=None,
                allclose_rtol=None,
                allclose_atol=None,
                mean_tolerance=2.0,
                print_per_file_results=True,
            )

        self.assertFalse(result["all_passed"], f"Files with mean diff {diff_value} should fail with tolerance 2.0")
        self.assertEqual(result["passed_count"], 0, "No files should pass")
        self.assertEqual(result["failed_count"], 2, "Both files should fail")

    async def test_compare_images_in_directories_missing_directory(self) -> None:
        """Test when one of the directories doesn't exist."""
        golden_dir = os.path.join(self.test_dir, "golden")
        test_dir = os.path.join(self.test_dir, "nonexistent")

        os.makedirs(golden_dir)

        with self.assertRaises(FileNotFoundError) as context:
            compare_images_in_directories(golden_dir=golden_dir, test_dir=test_dir, path_pattern=None)

        self.assertIn("test directory not found", str(context.exception).lower())

    async def test_compare_images_in_directories_pattern_none_matches_all(self) -> None:
        """Test that pattern=None matches all files including different extensions."""
        golden_dir = os.path.join(self.test_dir, "golden")
        test_dir = os.path.join(self.test_dir, "test")
        os.makedirs(golden_dir)
        os.makedirs(test_dir)

        resolution = (100, 100, 3)
        image_data = np.random.randint(0, 255, resolution, dtype=np.uint8)

        Image.fromarray(image_data).save(os.path.join(golden_dir, "image_001.png"))
        Image.fromarray(image_data).save(os.path.join(test_dir, "image_001.png"))
        Image.fromarray(image_data).save(os.path.join(golden_dir, "image_002.jpg"))
        Image.fromarray(image_data).save(os.path.join(test_dir, "image_002.jpg"))

        stdout_capture = io.StringIO()
        with patch("sys.stdout", stdout_capture):
            result = compare_images_in_directories(
                golden_dir=golden_dir, test_dir=test_dir, path_pattern=None, print_per_file_results=True
            )

        self.assertTrue(result["all_passed"], "Both PNG and JPG files should be compared")
        self.assertEqual(result["passed_count"], 2, "Both files should pass")
        self.assertIn("image_001.png", result["file_results"])
        self.assertIn("image_002.jpg", result["file_results"])

    async def test_compare_images_in_directories_complex_pattern(self) -> None:
        """Test with complex regex pattern matching specific frame numbers."""
        golden_dir = os.path.join(self.test_dir, "golden")
        test_dir = os.path.join(self.test_dir, "test")
        os.makedirs(golden_dir)
        os.makedirs(test_dir)

        resolution = (100, 100, 3)
        image_data = np.random.randint(0, 255, resolution, dtype=np.uint8)

        Image.fromarray(image_data).save(os.path.join(golden_dir, "frame_000000.png"))
        Image.fromarray(image_data).save(os.path.join(test_dir, "frame_000000.png"))
        Image.fromarray(image_data).save(os.path.join(golden_dir, "frame_000001.png"))
        Image.fromarray(image_data).save(os.path.join(test_dir, "frame_000001.png"))
        Image.fromarray(image_data).save(os.path.join(golden_dir, "other_file.png"))
        Image.fromarray(image_data).save(os.path.join(test_dir, "other_file.png"))

        stdout_capture = io.StringIO()
        with patch("sys.stdout", stdout_capture):
            result = compare_images_in_directories(
                golden_dir=golden_dir,
                test_dir=test_dir,
                path_pattern=r"^frame_\d{6}\.png$",
                print_per_file_results=True,
            )

        self.assertTrue(result["all_passed"], "Frame files should pass")
        self.assertEqual(result["passed_count"], 2, "Only 2 frame files should be compared")
        self.assertIn("frame_000000.png", result["file_results"])
        self.assertIn("frame_000001.png", result["file_results"])
        self.assertNotIn("other_file.png", result["file_results"])

    async def test_compare_images_in_directories_print_options(self) -> None:
        """Test different printing options."""
        golden_dir = os.path.join(self.test_dir, "golden")
        test_dir = os.path.join(self.test_dir, "test")
        os.makedirs(golden_dir)
        os.makedirs(test_dir)

        resolution = (100, 100, 3)
        image_data = np.random.randint(0, 255, resolution, dtype=np.uint8)
        Image.fromarray(image_data).save(os.path.join(golden_dir, "image.png"))
        Image.fromarray(image_data).save(os.path.join(test_dir, "image.png"))

        stdout_capture = io.StringIO()
        with patch("sys.stdout", stdout_capture):
            result = compare_images_in_directories(
                golden_dir=golden_dir, test_dir=test_dir, path_pattern=None, print_per_file_results=False
            )
        captured_no_per_file = stdout_capture.getvalue()

        stdout_capture = io.StringIO()
        with patch("sys.stdout", stdout_capture):
            result = compare_images_in_directories(
                golden_dir=golden_dir, test_dir=test_dir, path_pattern=None, print_per_file_results=True
            )
        captured_with_per_file = stdout_capture.getvalue()

        self.assertLess(len(captured_no_per_file), len(captured_with_per_file), "Per-file results should add output")
        self.assertIn("PASSED", captured_with_per_file)
