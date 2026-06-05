# SPDX-FileCopyrightText: Copyright (c) 2024-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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

"""Test suite for file validation utilities."""

from __future__ import annotations

import os
import tempfile
from pathlib import Path

from isaacsim.test.utils.file_validation import get_folder_file_summary, validate_file_list, validate_folder_contents
from isaacsim.test.utils.timed_async_test import TimedAsyncTestCase


class TestFileValidation(TimedAsyncTestCase):
    """Test suite for file validation utilities."""

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

    def create_test_files(self, file_specs: dict[str, str | int]) -> dict[str, str]:
        """Create test files with specified content or size.

        Args:
            file_specs: Dictionary mapping filename to content (str) or size (int).

        Returns:
            Dictionary mapping filename to full file path.
        """
        file_paths = {}
        for filename, content_or_size in file_specs.items():
            file_path = os.path.join(self.test_dir, filename)

            # Create parent directories if needed
            os.makedirs(os.path.dirname(file_path), exist_ok=True)

            if isinstance(content_or_size, str):
                with open(file_path, "w") as f:
                    f.write(content_or_size)
            elif isinstance(content_or_size, int):
                # Create file with specific size
                with open(file_path, "wb") as f:
                    f.write(b"x" * content_or_size)

            file_paths[filename] = file_path

        return file_paths

    # Tests for validate_folder_contents function
    async def test_validate_folder_contents_basic(self) -> None:
        """Test basic folder validation functionality."""
        # Create test files
        self.create_test_files(
            {
                "image1.png": "png content",
                "image2.png": "more png content",
                "data.json": '{"test": true}',
            }
        )

        # Test exact match (should pass)
        result = validate_folder_contents(self.test_dir, {"png": 2, "json": 1})
        self.assertTrue(result, "Should validate correct file counts")

        # Test wrong counts (should fail)
        result = validate_folder_contents(self.test_dir, {"png": 3, "json": 1})
        self.assertFalse(result, "Should fail with incorrect PNG count")

        result = validate_folder_contents(self.test_dir, {"png": 2, "json": 2})
        self.assertFalse(result, "Should fail with incorrect JSON count")

    async def test_validate_folder_contents_nonexistent_directory(self) -> None:
        """Test validation with nonexistent directory."""
        nonexistent_path = os.path.join(self.test_dir, "nonexistent")
        result = validate_folder_contents(nonexistent_path, {"png": 1})
        self.assertFalse(result, "Should fail for nonexistent directory")

    async def test_validate_folder_contents_empty_directory(self) -> None:
        """Test validation with empty directory."""
        empty_dir = os.path.join(self.test_dir, "empty")
        os.makedirs(empty_dir)

        result = validate_folder_contents(empty_dir, {"png": 0})
        self.assertTrue(result, "Should pass for zero expected files")

        result = validate_folder_contents(empty_dir, {"png": 1})
        self.assertFalse(result, "Should fail when expecting files in empty directory")

    async def test_validate_folder_contents_recursive(self) -> None:
        """Test recursive directory traversal."""
        # Create nested structure
        self.create_test_files(
            {
                "top.txt": "top level",
                "subdir/nested.png": "nested image",
                "subdir/deep/deeper.json": '{"deep": true}',
            }
        )

        # Non-recursive should only find top-level files
        result = validate_folder_contents(self.test_dir, {"txt": 1}, recursive=False)
        self.assertTrue(result, "Should find top-level txt file")

        result = validate_folder_contents(self.test_dir, {"png": 1}, recursive=False)
        self.assertFalse(result, "Should not find nested PNG without recursive")

        # Recursive should find all files
        result = validate_folder_contents(self.test_dir, {"txt": 1, "png": 1, "json": 1}, recursive=True)
        self.assertTrue(result, "Should find all files recursively")

    async def test_validate_folder_contents_empty_files(self) -> None:
        """Test empty file detection."""
        # Create files with different sizes
        self.create_test_files(
            {
                "empty.txt": 0,  # 0 bytes
                "small.txt": 5,  # 5 bytes
                "normal.txt": "normal content",
            }
        )

        # Should pass without empty file check
        result = validate_folder_contents(self.test_dir, {"txt": 3}, fail_on_empty_files=False)
        self.assertTrue(result, "Should pass when not checking for empty files")

        # Should fail with empty file check
        result = validate_folder_contents(self.test_dir, {"txt": 3}, fail_on_empty_files=True)
        self.assertFalse(result, "Should fail when empty file detected")

    async def test_validate_folder_contents_min_file_size(self) -> None:
        """Test minimum file size validation."""
        self.create_test_files(
            {
                "tiny.txt": 1,  # 1 byte
                "small.txt": 10,  # 10 bytes
                "large.txt": 100,  # 100 bytes
            }
        )

        # All files should pass with min size 1
        result = validate_folder_contents(self.test_dir, {"txt": 3}, min_file_size_bytes=1)
        self.assertTrue(result, "Should pass with min size 1")

        # Should fail with min size 50 (tiny and small files too small)
        result = validate_folder_contents(self.test_dir, {"txt": 3}, min_file_size_bytes=50)
        self.assertFalse(result, "Should fail with min size 50")

    async def test_validate_folder_contents_exact_match_false(self) -> None:
        """Test non-exact matching (at least the specified counts)."""
        self.create_test_files(
            {
                "file1.png": "content1",
                "file2.png": "content2",
                "file3.png": "content3",
                "data.json": "{}",
            }
        )

        # Should pass with exact_match=False even with extra files
        result = validate_folder_contents(self.test_dir, {"png": 2}, exact_match=False)
        self.assertTrue(result, "Should pass with at least 2 PNG files")

        # Should fail if not enough files
        result = validate_folder_contents(self.test_dir, {"png": 5}, exact_match=False)
        self.assertFalse(result, "Should fail with fewer than 5 PNG files")

    async def test_validate_folder_contents_allowed_extensions(self) -> None:
        """Test allowed extra extensions filtering."""
        self.create_test_files(
            {
                "image.png": "image",
                "data.json": "{}",
                "temp.tmp": "temporary",
                "log.txt": "log content",
            }
        )

        # Should pass when tmp and txt are allowed
        result = validate_folder_contents(self.test_dir, {"png": 1, "json": 1}, allowed_extra_extensions={"tmp", "txt"})
        self.assertTrue(result, "Should pass with allowed extra extensions")

        # Should fail when tmp is not allowed
        result = validate_folder_contents(self.test_dir, {"png": 1, "json": 1}, allowed_extra_extensions={"txt"})
        self.assertFalse(result, "Should fail with disallowed tmp extension")

        # Should pass when None (any extra extensions allowed)
        result = validate_folder_contents(self.test_dir, {"png": 1, "json": 1}, allowed_extra_extensions=None)
        self.assertTrue(result, "Should pass with None (any extensions allowed)")

    async def test_validate_folder_contents_case_insensitive(self) -> None:
        """Test case-insensitive extension matching."""
        self.create_test_files(
            {
                "image.PNG": "uppercase extension",
                "data.Json": "mixed case extension",
                "text.TXT": "text file",
            }
        )

        # Should match regardless of case
        result = validate_folder_contents(self.test_dir, {"png": 1, "json": 1, "txt": 1})
        self.assertTrue(result, "Should match extensions case-insensitively")

    # Tests for get_folder_file_summary function
    async def test_get_folder_file_summary_basic(self) -> None:
        """Test basic folder summary functionality."""
        self.create_test_files(
            {
                "file1.png": "content1",
                "file2.png": "content2",
                "data.json": "{}",
                "noext": "no extension",
            }
        )

        summary = get_folder_file_summary(self.test_dir)

        self.assertEqual(summary["total_files"], 4, "Should count all files")
        self.assertEqual(summary["extension_counts"]["png"], 2, "Should count PNG files")
        self.assertEqual(summary["extension_counts"]["json"], 1, "Should count JSON files")
        self.assertNotIn("", summary["extension_counts"], "Should not include files without extensions")

    async def test_get_folder_file_summary_with_file_sizes(self) -> None:
        """Test folder summary with file size details."""
        files = self.create_test_files(
            {
                "small.txt": "hi",  # 2 bytes
                "large.txt": "a" * 100,  # 100 bytes
            }
        )

        summary = get_folder_file_summary(self.test_dir, include_file_sizes=True)

        self.assertIn("file_details", summary)
        self.assertEqual(len(summary["file_details"]["txt"]), 2, "Should have 2 txt files")

        # Check file details structure
        for file_detail in summary["file_details"]["txt"]:
            self.assertIn("path", file_detail)
            self.assertIn("size_bytes", file_detail)
            self.assertGreater(file_detail["size_bytes"], 0)

    async def test_get_folder_file_summary_recursive(self) -> None:
        """Test recursive folder summary."""
        self.create_test_files(
            {
                "top.txt": "top",
                "subdir/nested.png": "nested",
                "subdir/deep/file.json": "{}",
            }
        )

        # Non-recursive
        summary = get_folder_file_summary(self.test_dir, recursive=False)
        self.assertEqual(summary["total_files"], 1, "Should only count top-level files")

        # Recursive
        summary = get_folder_file_summary(self.test_dir, recursive=True)
        self.assertEqual(summary["total_files"], 3, "Should count all files recursively")
        self.assertEqual(summary["extension_counts"]["txt"], 1)
        self.assertEqual(summary["extension_counts"]["png"], 1)
        self.assertEqual(summary["extension_counts"]["json"], 1)

    async def test_get_folder_file_summary_nonexistent(self) -> None:
        """Test folder summary for nonexistent directory."""
        nonexistent = os.path.join(self.test_dir, "nonexistent")
        summary = get_folder_file_summary(nonexistent)

        self.assertEqual(summary["total_files"], 0)
        self.assertEqual(summary["extension_counts"], {})

    # Tests for validate_file_list function
    async def test_validate_file_list_basic(self) -> None:
        """Test basic file list validation."""
        file_paths = self.create_test_files(
            {
                "exists1.txt": "content1",
                "exists2.txt": "content2",
            }
        )

        existing_files = list(file_paths.values())
        missing_file = os.path.join(self.test_dir, "missing.txt")
        all_files = existing_files + [missing_file]

        # Should pass for existing files
        result = validate_file_list(existing_files)
        self.assertTrue(result["passed"], "Should pass for existing files")
        self.assertEqual(len(result["missing_files"]), 0, "Should have no missing files")

        # Should fail with missing file
        result = validate_file_list(all_files, fail_on_missing=True)
        self.assertFalse(result["passed"], "Should fail with missing file")
        self.assertEqual(len(result["missing_files"]), 1, "Should report 1 missing file")
        self.assertIn(missing_file, result["missing_files"])

    async def test_validate_file_list_empty_files(self) -> None:
        """Test file list validation with empty files."""
        file_paths = self.create_test_files(
            {
                "empty.txt": 0,  # 0 bytes
                "normal.txt": "content",
            }
        )

        all_files = list(file_paths.values())

        # Should pass without empty file check
        result = validate_file_list(all_files, fail_on_empty_files=False)
        self.assertTrue(result["passed"], "Should pass without empty file check")

        # Should fail with empty file check
        result = validate_file_list(all_files, fail_on_empty_files=True)
        self.assertFalse(result["passed"], "Should fail with empty file check")
        self.assertEqual(len(result["empty_files"]), 1, "Should report 1 empty file")

    async def test_validate_file_list_min_size(self) -> None:
        """Test file list validation with minimum size requirement."""
        file_paths = self.create_test_files(
            {
                "tiny.txt": 5,  # 5 bytes
                "large.txt": 50,  # 50 bytes
            }
        )

        all_files = list(file_paths.values())

        # Should pass with min size 5
        result = validate_file_list(all_files, min_file_size_bytes=5)
        self.assertTrue(result["passed"], "Should pass with min size 5")

        # Should fail with min size 10
        result = validate_file_list(all_files, min_file_size_bytes=10)
        self.assertFalse(result["passed"], "Should fail with min size 10")
        self.assertEqual(len(result["undersized_files"]), 1, "Should report 1 undersized file")

    async def test_validate_file_list_directories_as_files(self) -> None:
        """Test that directories are treated as missing files."""
        # Create a directory
        subdir = os.path.join(self.test_dir, "subdir")
        os.makedirs(subdir)

        result = validate_file_list([subdir], fail_on_missing=True)
        self.assertFalse(result["passed"], "Should fail when directory passed as file")
        self.assertEqual(len(result["missing_files"]), 1, "Should report directory as missing file")

    async def test_validate_file_list_path_types(self) -> None:
        """Test file list validation with different path types."""
        file_paths = self.create_test_files(
            {
                "test.txt": "content",
            }
        )

        file_path = list(file_paths.values())[0]

        # Test with string path
        result = validate_file_list([file_path])
        self.assertTrue(result["passed"], "Should work with string paths")

        # Test with Path object
        result = validate_file_list([Path(file_path)])
        self.assertTrue(result["passed"], "Should work with Path objects")

    async def test_validate_file_list_complex_scenario(self) -> None:
        """Test file list validation with multiple criteria."""
        file_paths = self.create_test_files(
            {
                "good.txt": "enough content",  # Good file
                "empty.txt": 0,  # Empty file
                "tiny.txt": 3,  # Too small
            }
        )

        missing_file = os.path.join(self.test_dir, "missing.txt")
        all_files = list(file_paths.values()) + [missing_file]

        result = validate_file_list(all_files, fail_on_missing=True, fail_on_empty_files=True, min_file_size_bytes=5)

        self.assertFalse(result["passed"], "Should fail with multiple issues")
        self.assertEqual(len(result["missing_files"]), 1, "Should report missing file")
        self.assertEqual(len(result["empty_files"]), 1, "Should report empty file")
        self.assertEqual(len(result["undersized_files"]), 2, "Should report 2 undersized files (empty and tiny)")

    async def test_edge_cases(self) -> None:
        """Test various edge cases."""
        # Test with files without extensions
        self.create_test_files(
            {
                "noext": "no extension",
                "file.txt": "with extension",
            }
        )

        # validate_folder_contents should ignore files without extensions
        result = validate_folder_contents(self.test_dir, {"txt": 1})
        self.assertTrue(result, "Should ignore files without extensions")

        # get_folder_file_summary should count all files but only extensions
        summary = get_folder_file_summary(self.test_dir)
        self.assertEqual(summary["total_files"], 2, "Should count all files")
        self.assertEqual(len(summary["extension_counts"]), 1, "Should only count files with extensions")

        # Test empty expected_counts
        result = validate_folder_contents(self.test_dir, {})
        self.assertTrue(result, "Should pass with empty expected counts")
