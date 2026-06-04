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

"""Test suite for Isaac Sim storage native extension functionality."""

import asyncio

import carb
import omni.kit.commands
import omni.kit.test

# import omni.kit.usd
from isaacsim.storage.native import (
    find_filtered_files_async,
    get_assets_root_path,
    get_assets_root_path_async,
    is_local_path,
    path_join,
    resolve_asset_path,
)
from isaacsim.storage.native.impl.file_utils import _URL_SCHEMES


class TestPathJoin(omni.kit.test.AsyncTestCase):
    """Tests for path_join covering all URL schemes and local paths."""

    def _base(self, scheme):
        return f"{scheme}server/folder"

    def test_basic_join_all_schemes(self):
        """Forward-slash join for every URL scheme — no backslashes on Windows."""
        for scheme in _URL_SCHEMES:
            base = self._base(scheme)
            result = path_join(base, "file.usd")
            self.assertEqual(result, f"{base}/file.usd", f"Failed for scheme: {scheme}")

    def test_trailing_slash_stripped(self):
        """Trailing slash on base is stripped before joining."""
        for scheme in _URL_SCHEMES:
            base = self._base(scheme) + "/"
            result = path_join(base, "file.usd")
            self.assertNotIn("//file.usd", result, f"Double slash for scheme: {scheme}")
            self.assertTrue(result.endswith("/file.usd"), f"Failed for scheme: {scheme}")

    def test_dot_slash_prefix_stripped(self):
        """Leading ./ on name is stripped."""
        for scheme in _URL_SCHEMES:
            base = self._base(scheme)
            result = path_join(base, "./sub/file.usd")
            self.assertEqual(result, f"{base}/sub/file.usd", f"Failed for scheme: {scheme}")

    def test_parent_traversal(self):
        """../ traversal uses forward-slash split, not os.path.dirname."""
        for scheme in _URL_SCHEMES:
            base = f"{scheme}server/a/b"
            result = path_join(base, "../file.usd")
            self.assertEqual(result, f"{scheme}server/a/file.usd", f"Failed for scheme: {scheme}")
            self.assertNotIn("\\", result, f"Backslash found for scheme: {scheme}")

    def test_local_path_uses_os_join(self):
        """Local paths fall through to os.path.join."""
        import os

        result = path_join("/local/path", "file.usd")
        self.assertEqual(result, os.path.join("/local/path", "file.usd"))


class TestIsLocalPath(omni.kit.test.AsyncTestCase):
    """Tests for is_local_path covering all URL schemes and local paths."""

    def test_url_schemes_are_not_local(self):
        """Every URL scheme must return False."""
        for scheme in _URL_SCHEMES:
            path = f"{scheme}server/path/file.usd"
            self.assertFalse(is_local_path(path), f"Expected False for scheme: {scheme}")

    def test_local_paths_are_local(self):
        """Absolute and relative local paths return True."""
        for path in ("/home/user/file.usd", "relative/file.usd", "/mnt/data/file.usd"):
            self.assertTrue(is_local_path(path), f"Expected True for: {path}")

    def test_empty_path_is_local(self):
        """Empty string is treated as local."""
        self.assertTrue(is_local_path(""))


class TestStorageNative(omni.kit.test.AsyncTestCase):
    """Test suite for Isaac Sim storage native extension functionality.

    Tests cover asset path resolution, file discovery, filtering, and Nucleus server
    connectivity functions.
    """

    async def setUp(self):
        """Set up test fixtures before each test method."""
        await omni.kit.app.get_app().next_update_async()

    async def tearDown(self):
        """Clean up after each test method."""
        await omni.kit.app.get_app().next_update_async()

    async def test_get_assets_root_path(self):
        """Test asset root path retrieval with various settings configurations.

        Verifies that get_assets_root_path and get_assets_root_path_async correctly
        return the configured asset root path, handle unset settings, and support
        the skip_check parameter for bypassing validation.
        """
        default_assets_url = carb.settings.get_settings().get("/persistent/isaac/asset_root/default")

        # including checking step
        self.assertEqual(await get_assets_root_path_async(), default_assets_url)
        self.assertEqual(get_assets_root_path(), default_assets_url)

        # check unset setting
        carb.settings.get_settings().set("/persistent/isaac/asset_root/default", 0)
        with self.assertRaises(RuntimeError) as context:
            await get_assets_root_path_async()
        self.assertIn("setting is not set", str(context.exception))
        with self.assertRaises(RuntimeError) as context:
            get_assets_root_path()
        self.assertIn("setting is not set", str(context.exception))

        # skipping checking step
        # - define invalid setting
        invalid_assets_url = "https://some-invalid-path"
        carb.settings.get_settings().set("/persistent/isaac/asset_root/default", invalid_assets_url)
        # - check for exception raised due to invalid setting
        with self.assertRaises(RuntimeError) as context:
            await get_assets_root_path_async()
        self.assertIn("Could not find assets root folder:", str(context.exception))
        with self.assertRaises(RuntimeError) as context:
            get_assets_root_path()
        self.assertIn("Could not find assets root folder:", str(context.exception))
        # - check for path (valid or not) due to skipping checking step
        self.assertEqual(await get_assets_root_path_async(skip_check=True), invalid_assets_url)
        self.assertEqual(get_assets_root_path(skip_check=True), invalid_assets_url)

        # reset settings
        carb.settings.get_settings().set("/persistent/isaac/asset_root/default", default_assets_url)

    async def test_find_filtered_files_async_basic_discovery(self):
        """Test basic USD file discovery without filters.

        This test verifies that the find_filtered_files_async function can discover
        USD files in the Isaac Sim Simple Room environment and returns them as a set.
        """
        simple_room_path = await get_assets_root_path_async()
        simple_room_path += "/Isaac/Environments/Simple_Room/"

        result = await find_filtered_files_async(
            root_path=simple_room_path,
        )

        self.assertIsNotNone(result)
        self.assertIsInstance(result, set)
        self.assertGreater(len(result), 0, "Should find at least some USD files in Simple Room")

        # Verify all results are valid USD files
        valid_extensions = [".usd", ".usda", ".usdc", ".usdz"]
        for file_path in result:
            self.assertTrue(
                any(file_path.lower().endswith(ext) for ext in valid_extensions),
                f"File should have valid USD extension: {file_path}",
            )

    async def test_find_filtered_files_async_pattern_filtering_any_mode(self):
        """Test regex pattern filtering with match_all=False (any pattern matches).

        This test verifies that when multiple patterns are provided and match_all=False,
        files matching ANY of the patterns are included in the results.
        """
        simple_room_path = await get_assets_root_path_async()
        simple_room_path += "/Isaac/Environments/Simple_Room/"

        # Test with multiple patterns - should find files matching ANY pattern
        result = await find_filtered_files_async(
            root_path=simple_room_path,
            filter_patterns=["room", "Props"],
            match_all=False,  # ANY mode (default)
        )

        self.assertIsNotNone(result)
        self.assertIsInstance(result, set)

        # If results found, verify each matches at least one pattern
        if len(result) > 0:
            for file_path in result:
                path_lower = file_path.lower()
                has_room = "room" in path_lower
                has_props = "Props" in file_path
                self.assertTrue(
                    has_room or has_props,
                    f"File '{file_path}' should match at least one pattern ['room', 'Props'] in ANY mode",
                )

    async def test_find_filtered_files_async_pattern_filtering_all_mode(self):
        """Test regex pattern filtering with match_all=True (all patterns must match).

        This test verifies that when match_all=True, only files matching ALL patterns
        are included in the results.
        """
        simple_room_path = await get_assets_root_path_async()
        simple_room_path += "/Isaac/Environments/Simple_Room/"

        # Test with multiple patterns - should find files matching ALL patterns
        result = await find_filtered_files_async(
            root_path=simple_room_path,
            filter_patterns=["Towel", "Room"],
            match_all=True,  # ALL mode
        )

        self.assertIsNotNone(result)
        self.assertIsInstance(result, set)

        # If results found, verify each matches ALL patterns
        if len(result) > 0:
            for file_path in result:

                has_towel = "Towel" in file_path
                has_room = "Room" in file_path
                self.assertTrue(
                    has_towel and has_room,
                    f"File '{file_path}' should match ALL patterns ['Towel', 'Room'] in ALL mode",
                )

    async def test_find_filtered_files_async_filepath_exclusion(self):
        """Test filepath exclusion functionality.

        This test verifies that files containing excluded substrings are filtered out
        from the results, even if they would otherwise match the criteria.
        """
        simple_room_path = await get_assets_root_path_async()
        simple_room_path += "/Isaac/Environments/Simple_Room/"

        # First get all files without exclusion
        all_files = await find_filtered_files_async(
            root_path=simple_room_path,
        )

        # Then get files with exclusion (exclude files containing "Props")
        filtered_files = await find_filtered_files_async(
            root_path=simple_room_path,
            filepath_excludes=["Props"],
        )

        self.assertIsNotNone(all_files)
        self.assertIsNotNone(filtered_files)

        # Filtered results should be subset of or equal to all results
        self.assertLessEqual(len(filtered_files), len(all_files))
        self.assertTrue(set(filtered_files).issubset(set(all_files)))

        # Verify no filtered files contain excluded substring
        for file_path in filtered_files:
            self.assertNotIn("Props", file_path, f"Excluded file should not contain 'Props': {file_path}")

    async def test_find_filtered_files_async_max_depth_limiting(self):
        """Test max_depth parameter functionality.

        This test verifies that the max_depth parameter correctly limits the directory
        traversal depth, and that deeper searches find more or equal files.
        """
        simple_room_path = await get_assets_root_path_async()
        simple_room_path += "/Isaac/Environments/Simple_Room/"

        # Test with different depth limits
        shallow_result = await find_filtered_files_async(
            root_path=simple_room_path, max_depth=0  # Only immediate directory
        )

        deeper_result = await find_filtered_files_async(
            root_path=simple_room_path, max_depth=1  # Deeper traversal - should find more files
        )

        self.assertIsNotNone(shallow_result)
        self.assertIsNotNone(deeper_result)

        # Deeper searches should find same or more files
        self.assertLessEqual(len(shallow_result), len(deeper_result))
        self.assertTrue(set(shallow_result).issubset(set(deeper_result)))

    async def test_find_filtered_files_async_async_behavior(self):
        """Test that the function properly executes asynchronously.

        This test verifies that find_filtered_files_async doesn't block the event loop
        and can be cancelled or run concurrently with other async operations.
        """
        simple_room_path = await get_assets_root_path_async()
        simple_room_path += "/Isaac/Environments/Simple_Room/"

        # Test concurrent execution of multiple async calls
        tasks = [find_filtered_files_async(root_path=simple_room_path, max_depth=2) for _ in range(3)]

        # All tasks should complete successfully
        results = await asyncio.gather(*tasks)

        # Verify all results are valid and consistent
        for i, result in enumerate(results):
            self.assertIsNotNone(result, f"Task {i} should return valid result")
            self.assertIsInstance(result, set, f"Task {i} should return a set")
            # All concurrent calls with same parameters should return same results
            self.assertEqual(result, results[0], f"Task {i} should return consistent results")

    async def test_find_filtered_files_async_error_handling(self):
        """Test error handling for invalid paths and edge cases.

        This test verifies that the function gracefully handles invalid paths,
        non-existent directories, and other error conditions.
        """
        # Test with non-existent path
        try:
            result = await find_filtered_files_async(root_path="/invalid/nonexistent/path", max_depth=1)
            # Should return empty set for non-existent paths
            self.assertIsInstance(result, set)
            self.assertEqual(len(result), 0, "Non-existent path should return empty set")
        except Exception as e:
            # Or might raise an exception - either behavior is acceptable
            self.assertIsNotNone(e)

        # Test with empty root path
        try:
            result = await find_filtered_files_async(root_path="", max_depth=1)
            self.assertIsInstance(result, set)
        except Exception as e:
            # Empty path might raise an exception
            self.assertIsNotNone(e)

        # Test with invalid regex patterns (should handle gracefully)
        simple_room_path = await get_assets_root_path_async()
        simple_room_path += "/Isaac/Environments/Simple_Room/"

        # Test with invalid regex - function should handle gracefully
        result = await find_filtered_files_async(
            root_path=simple_room_path, filter_patterns=["[invalid_regex"], max_depth=1  # Unclosed bracket
        )
        self.assertIsInstance(result, set)
        # Should still work, just might not apply the invalid pattern

    async def test_find_filtered_files_async_combined_parameters(self):
        """Test combining multiple parameters together.

        This test verifies that all parameters work correctly when used together:
        patterns, exclusions, depth limiting, and match mode.
        It also demonstrates that more restrictive filtering returns fewer files.
        """
        warehouse_path = await get_assets_root_path_async()
        warehouse_path += "/Isaac/Environments/Simple_Warehouse/"

        # Test 1: Less restrictive filtering - single pattern
        broad_result = await find_filtered_files_async(
            root_path=warehouse_path,
            filter_patterns=["warehouse"],
            match_all=False,  # ANY pattern matching
            filepath_excludes=["Props", "Stage"],
            max_depth=2,
        )

        # Test 2: More restrictive filtering - multiple patterns with match_all=True
        restrictive_result = await find_filtered_files_async(
            root_path=warehouse_path,
            filter_patterns=["warehouse", "shelves"],
            match_all=True,  # ALL patterns must match
            filepath_excludes=["Props", "Stage"],
            max_depth=2,
        )

        self.assertIsNotNone(broad_result)
        self.assertIsInstance(broad_result, set)
        self.assertIsNotNone(restrictive_result)
        self.assertIsInstance(restrictive_result, set)

        # More restrictive filtering should return fewer or equal files
        self.assertLessEqual(
            len(restrictive_result),
            len(broad_result),
            f"Restrictive filter should return fewer files. Broad: {len(broad_result)}, Restrictive: {len(restrictive_result)}",
        )

        # Verify we found some files in broad search
        self.assertGreater(len(broad_result), 0, "Broad search should find at least some warehouse files")

        # Verify broad results match expected criteria
        for file_path in broad_result:
            # Should match "warehouse" pattern
            path_lower = file_path.lower()
            has_warehouse = "warehouse" in path_lower
            self.assertTrue(has_warehouse, f"File '{file_path}' should match pattern 'warehouse'")

            # Should not contain excluded substrings
            self.assertNotIn("Props", file_path, f"File should not contain exclusion: {file_path}")
            self.assertNotIn("Stage", file_path, f"File should not contain exclusion: {file_path}")

            # Should be a valid USD file
            valid_extensions = [".usd", ".usda", ".usdc", ".usdz"]
            self.assertTrue(
                any(file_path.lower().endswith(ext) for ext in valid_extensions),
                f"File should have valid USD extension: {file_path}",
            )

        # Verify restrictive results match ALL patterns
        for file_path in restrictive_result:
            path_lower = file_path.lower()
            has_warehouse = "warehouse" in path_lower
            has_shelves = "shelves" in path_lower
            self.assertTrue(
                has_warehouse and has_shelves, f"File '{file_path}' should match ALL patterns ['warehouse', 'shelves']"
            )

    async def test_resolve_asset_path_resolves_with_assets_root(self):
        """Test resolve_asset_path resolves using assets root when original is bare path.

        Verifies that when given a relative path like '/Isaac/...', the function
        correctly prefixes it with the assets root path.
        """
        assets_root = get_assets_root_path()
        original_relative = "/Isaac/Environments/Grid/default_environment.usd"

        # Expect resolve_asset_path function to prefix with assets root and resolve
        resolved = resolve_asset_path(original_relative)
        self.assertIsNotNone(resolved)
        self.assertEqual(resolved, assets_root + original_relative)

    async def test_resolve_asset_path_returns_original_if_exists(self):
        """Test resolve_asset_path returns original when full path already exists.

        Verifies that when given a complete, valid path, the function returns
        that path unchanged without attempting to modify it.
        """
        assets_root = get_assets_root_path()
        full_path = assets_root + "/Isaac/Environments/Grid/default_environment.usd"

        # Expect the function to return the given full path if it exists
        resolved = resolve_asset_path(full_path)
        self.assertIsNotNone(resolved)
        self.assertEqual(resolved, full_path)

    async def test_resolve_asset_path_returns_none_if_missing(self):
        """Test resolve_asset_path returns None when neither path exists.

        Verifies that when given a path that does not exist in either its
        original form or when prefixed with the assets root, the function
        returns None.
        """
        # Expect the function to return None for a non-existent path
        original = "/nonexistent/path/definitely_missing.usd"
        resolved = resolve_asset_path(original)
        self.assertIsNone(resolved)
