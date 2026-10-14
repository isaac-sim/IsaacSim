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
import os
import threading
import time
from collections.abc import Callable
from unittest.mock import AsyncMock, Mock, call, patch

import carb
import omni.kit.commands
import omni.kit.test

# import omni.kit.usd
from isaacsim.storage.native import (
    Version,
    find_filtered_files_async,
    get_assets_root_path,
    get_assets_root_path_async,
    is_local_path,
    path_join,
    resolve_asset_path,
    verify_asset_root_path,
)
from isaacsim.storage.native.impl.extension import (
    ASSET_REGION_PROFILE_ENV_VAR,
    ASSET_REGION_PROFILES_SETTING,
    ASSET_ROOT_ENV_VAR,
    ASSET_ROOT_SETTING,
    Extension,
)
from isaacsim.storage.native.impl.file_utils import _URL_SCHEMES
from isaacsim.storage.native.nucleus import check_server
from pxr import UsdUtils


def _stat_with_callback(stat: Callable) -> Callable:
    """Adapt a metadata response fixture to the client's callback API."""

    def complete(url: str, callback: Callable) -> Mock:
        callback(*stat(url))
        return Mock()

    return complete


class TestServerCheck(omni.kit.test.AsyncTestCase):
    """Check that synchronous metadata requests finish within their timeout."""

    async def test_check_server_cancels_unresponsive_request(self) -> None:
        """Cancel a metadata request that never completes within its timeout."""
        request = Mock()
        with patch.object(omni.client, "stat_with_callback", return_value=request):
            start = time.monotonic()
            self.assertFalse(check_server("omniverse://unresponsive.example", "/Isaac", timeout=0.02))
            self.assertLess(time.monotonic() - start, 1.0)
        request.stop.assert_called_once_with()

    async def test_check_server_completes_from_client_thread(self) -> None:
        """Accept a successful metadata response from the client's worker thread."""
        request = Mock()
        timer = None

        def stat(url: str, callback: Callable) -> Mock:
            nonlocal timer
            timer = threading.Timer(0.01, callback, args=(omni.client.Result.OK, None))
            timer.start()
            return request

        try:
            with patch.object(omni.client, "stat_with_callback", side_effect=stat):
                self.assertTrue(check_server("omniverse://responsive.example", "/Isaac", timeout=1.0))
            request.stop.assert_not_called()
        finally:
            if timer is not None:
                timer.join()


class TestPathJoin(omni.kit.test.AsyncTestCase):
    """Tests for path_join covering all URL schemes and local paths."""

    def _base(self, scheme: str) -> str:
        return f"{scheme}server/folder"

    def test_basic_join_all_schemes(self) -> None:
        """Forward-slash join for every URL scheme — no backslashes on Windows."""
        for scheme in _URL_SCHEMES:
            base = self._base(scheme)
            result = path_join(base, "file.usd")
            self.assertEqual(result, f"{base}/file.usd", f"Failed for scheme: {scheme}")

    def test_trailing_slash_stripped(self) -> None:
        """Trailing slash on base is stripped before joining."""
        for scheme in _URL_SCHEMES:
            base = self._base(scheme) + "/"
            result = path_join(base, "file.usd")
            self.assertNotIn("//file.usd", result, f"Double slash for scheme: {scheme}")
            self.assertTrue(result.endswith("/file.usd"), f"Failed for scheme: {scheme}")

    def test_dot_slash_prefix_stripped(self) -> None:
        """Leading ./ on name is stripped."""
        for scheme in _URL_SCHEMES:
            base = self._base(scheme)
            result = path_join(base, "./sub/file.usd")
            self.assertEqual(result, f"{base}/sub/file.usd", f"Failed for scheme: {scheme}")

    def test_parent_traversal(self) -> None:
        """../ traversal uses forward-slash split, not os.path.dirname."""
        for scheme in _URL_SCHEMES:
            base = f"{scheme}server/a/b"
            result = path_join(base, "../file.usd")
            self.assertEqual(result, f"{scheme}server/a/file.usd", f"Failed for scheme: {scheme}")
            self.assertNotIn("\\", result, f"Backslash found for scheme: {scheme}")

    def test_local_path_uses_os_join(self) -> None:
        """Local paths fall through to os.path.join."""
        import os

        result = path_join("/local/path", "file.usd")
        self.assertEqual(result, os.path.join("/local/path", "file.usd"))


class TestIsLocalPath(omni.kit.test.AsyncTestCase):
    """Tests for is_local_path covering all URL schemes and local paths."""

    def test_url_schemes_are_not_local(self) -> None:
        """Every URL scheme must return False."""
        for scheme in _URL_SCHEMES:
            path = f"{scheme}server/path/file.usd"
            self.assertFalse(is_local_path(path), f"Expected False for scheme: {scheme}")

    def test_local_paths_are_local(self) -> None:
        """Absolute and relative local paths return True."""
        for path in ("/home/user/file.usd", "relative/file.usd", "/mnt/data/file.usd"):
            self.assertTrue(is_local_path(path), f"Expected True for: {path}")

    def test_empty_path_is_local(self) -> None:
        """Empty string is treated as local."""
        self.assertTrue(is_local_path(""))


class TestAssetRegionProfile(omni.kit.test.AsyncTestCase):
    """Tests for selecting and applying asset region profiles."""

    def test_startup_reads_asset_region_profile_environment(self) -> None:
        """Startup applies the profile selected by the public environment variable."""
        extension = Extension()
        environment = {ASSET_REGION_PROFILE_ENV_VAR: "china"}

        with (
            patch("isaacsim.storage.native.impl.extension.os.getenv", side_effect=environment.get),
            patch.object(extension, "_apply_asset_region_profile") as apply_mock,
        ):
            extension.on_startup("isaacsim.storage.native")

        apply_mock.assert_called_once_with("china")

    def test_default_profile_sets_asset_root_without_s3_configuration(self) -> None:
        """The default profile restores the default root without changing S3 routing."""
        asset_root = "https://example.com/Assets/Isaac/6.1"
        settings = Mock()
        settings.get.return_value = {"asset_root": asset_root}
        extension = Extension()
        extension._profile_headers = []

        with (
            patch("isaacsim.storage.native.impl.extension.carb.settings.get_settings", return_value=settings),
            patch.object(omni.client, "set_s3_configuration") as configure_mock,
        ):
            extension._apply_asset_region_profile("us")

        settings.get.assert_called_once_with(f"{ASSET_REGION_PROFILES_SETTING}/us")
        settings.set_string.assert_called_once_with("/persistent/isaac/asset_root/default", asset_root)
        configure_mock.assert_not_called()

    def test_regional_profile_applies_storage_routing_and_roots(self) -> None:
        """A regional profile configures storage routing, headers, and resolved roots."""
        profile = {
            "endpoint": "storage.example.com",
            "bucket": "assets",
            "region": "example-region",
            "cdn_url": "https://cdn.example.com/",
            "cdn_for_list": False,
            "http_headers": {"x-example": "value"},
            "asset_root": "https://storage.example.com/Assets/Isaac/6.1/",
            "usd_search_endpoint": "https://search.example.com/",
        }
        settings = Mock()
        settings.get.return_value = profile
        extension = Extension()
        extension._profile_headers = []

        with (
            patch("isaacsim.storage.native.impl.extension.carb.settings.get_settings", return_value=settings),
            patch.object(omni.client, "set_s3_configuration", return_value=omni.client.Result.OK) as configure_mock,
            patch.object(omni.client, "set_http_header") as header_mock,
        ):
            extension._apply_asset_region_profile("china")

        settings.get.assert_called_once_with(f"{ASSET_REGION_PROFILES_SETTING}/china")
        configure_mock.assert_called_once_with(
            url="storage.example.com",
            bucket="assets",
            region="example-region",
            cloudfrontUrl="https://cdn.example.com/",
            cloudfrontForList=False,
            writeConfig=False,
        )
        header_mock.assert_called_once_with("x-example", "value")
        self.assertEqual(extension._profile_headers, ["x-example"])
        self.assertEqual(
            settings.set_string.call_args_list,
            [
                call(ASSET_ROOT_SETTING, "https://storage.example.com/Assets/Isaac/6.1"),
                call("/exts/omni.simready.content.browser/usd_search_endpoint", "https://search.example.com/"),
            ],
        )

    def test_asset_root_environment_overrides_selected_profile(self) -> None:
        """An explicit asset root wins after the selected profile is applied."""
        profile_root = "https://example.com/Assets/Isaac/6.1"
        environment_root = "https://override.example.com/assets/"
        settings = Mock()
        settings.get.return_value = {"asset_root": profile_root}
        extension = Extension()
        environment = {
            ASSET_REGION_PROFILE_ENV_VAR: "us",
            ASSET_ROOT_ENV_VAR: environment_root,
        }

        with (
            patch("isaacsim.storage.native.impl.extension.os.getenv", side_effect=environment.get),
            patch("isaacsim.storage.native.impl.extension.carb.settings.get_settings", return_value=settings),
        ):
            extension.on_startup("isaacsim.storage.native")

        self.assertEqual(
            settings.set_string.call_args_list,
            [
                call(ASSET_ROOT_SETTING, profile_root),
                call(ASSET_ROOT_SETTING, environment_root.rstrip("/")),
            ],
        )


class TestAssetRegionProfileAvailability(omni.kit.test.AsyncTestCase):
    """Live checks for assets required from a selected region profile."""

    _REQUIRED_ASSETS = (
        "Isaac/Environments/Hospital/Props/Cube.usd",
        "Isaac/Environments/Simple_Room/simple_room.usd",
        "Isaac/Robots_Multiphysics/FrankaRobotics/FrankaPanda/franka/franka.usda",
        "Isaac/Robots/NVIDIA/NovaCarter/nova_carter.usd",
        "NVIDIA/Materials/Base/Wood/Oak/Oak_BaseColor.png",
    )
    _DEPENDENCY_CANARY = "Isaac/Environments/Hospital/Props/Cube.usd"
    _MIN_DEPENDENCY_LAYERS = 1
    _MIN_DEPENDENCY_ASSETS = 1

    async def test_selected_profile_required_assets_are_available(self) -> None:
        """Resolve stable asset entry points and one dependency closure through the selected profile."""
        profile_name = os.getenv(ASSET_REGION_PROFILE_ENV_VAR)
        if not profile_name:
            self.skipTest(f"Set {ASSET_REGION_PROFILE_ENV_VAR} to run the live profile check")

        settings = carb.settings.get_settings()
        profile = settings.get(f"{ASSET_REGION_PROFILES_SETTING}/{profile_name}")
        self.assertTrue(profile, f"Unknown asset region profile: {profile_name}")

        profile_root = str(profile.get("asset_root") or "").rstrip("/")
        self.assertTrue(profile_root, f"Asset region profile '{profile_name}' has no asset root")
        self.assertEqual(await get_assets_root_path_async(), profile_root)

        failures: list[str] = []
        for relative_path in self._REQUIRED_ASSETS:
            result, _, content = await omni.client.read_file_async(path_join(profile_root, relative_path))
            if result != omni.client.Result.OK:
                failures.append(f"Required asset could not be read: {relative_path}: {result}")
            elif not content:
                failures.append(f"Required asset is empty: {relative_path}")

        canary_path = path_join(profile_root, self._DEPENDENCY_CANARY)
        layers, assets, unresolved = await asyncio.get_running_loop().run_in_executor(
            None, UsdUtils.ComputeAllDependencies, canary_path
        )
        if len(layers) < self._MIN_DEPENDENCY_LAYERS:
            failures.append(
                f"Expected at least {self._MIN_DEPENDENCY_LAYERS} USD layer for {self._DEPENDENCY_CANARY}, "
                f"resolved {len(layers)}"
            )
        if len(assets) < self._MIN_DEPENDENCY_ASSETS:
            failures.append(
                f"Expected at least {self._MIN_DEPENDENCY_ASSETS} USD asset for {self._DEPENDENCY_CANARY}, "
                f"resolved {len(assets)}"
            )
        if unresolved:
            failures.append(
                f"Unresolved dependencies for {self._DEPENDENCY_CANARY}: {[str(path) for path in unresolved]}"
            )

        self.assertFalse(failures, "\n".join(failures))


class TestVerifyAssetRootPath(omni.kit.test.AsyncTestCase):
    """Tests for asset-root version verification across storage providers."""

    def _verify_with_mocks(
        self,
        path: str,
        read_result: omni.client.Result,
        file_content: bytes = b"",
        stat_results: list[tuple[omni.client.Result, None]] | None = None,
    ) -> tuple[omni.client.Result, Version | str, Mock]:
        """Run version verification with deterministic application and storage responses."""
        if stat_results is None:
            stat_results = []
        stat_mock = Mock(side_effect=stat_results)
        with (
            patch(
                "isaacsim.storage.native.nucleus.get_version",
                return_value=("6.1.0", "", "", "", "", "", "", ""),
            ),
            patch.object(omni.client, "set_hang_detection_time_ms"),
            patch.object(omni.client, "push_base_url"),
            patch.object(omni.client, "combine_with_base_url", return_value=f"{path}/version.txt"),
            patch.object(omni.client, "read_file", return_value=(read_result, None, file_content)),
            patch.object(omni.client, "stat_with_callback", side_effect=_stat_with_callback(stat_mock)),
        ):
            result, version = verify_asset_root_path(path)
        return result, version, stat_mock

    def test_version_file_remains_preferred(self) -> None:
        """A readable version file is used without probing asset directories."""
        result, version, stat_mock = self._verify_with_mocks(
            "https://example.com/Assets/Isaac/6.1",
            omni.client.Result.OK,
            b"6.1.0",
        )

        self.assertEqual(result, omni.client.Result.OK)
        self.assertEqual(version, Version("6.1.0"))
        stat_mock.assert_not_called()

    def test_missing_version_file_accepts_isaac_only_http_root(self) -> None:
        """A versioned HTTP root with only Isaac assets is accepted when version.txt is absent."""
        result, version, stat_mock = self._verify_with_mocks(
            "https://example.com/Assets/Isaac/6.1",
            omni.client.Result.ERROR_NOT_FOUND,
            stat_results=[(omni.client.Result.OK, None)],
        )

        self.assertEqual(result, omni.client.Result.OK)
        self.assertEqual(version, Version("6.1.0"))
        stat_mock.assert_called_once_with("https://example.com/Assets/Isaac/6.1/Isaac")

    def test_missing_version_file_uses_three_component_http_root(self) -> None:
        """A three-component version in an HTTP root is preserved."""
        result, version, stat_mock = self._verify_with_mocks(
            "https://example.com/Assets/Isaac/6.1.2",
            omni.client.Result.ERROR_NOT_FOUND,
            stat_results=[(omni.client.Result.OK, None)],
        )

        self.assertEqual(result, omni.client.Result.OK)
        self.assertEqual(version, Version("6.1.2"))
        self.assertEqual(stat_mock.call_count, 1)

    def test_missing_version_file_accepts_nvidia_only_http_root(self) -> None:
        """A versioned HTTP root with only NVIDIA assets is accepted when version.txt is absent."""
        result, version, stat_mock = self._verify_with_mocks(
            "https://example.com/Assets/Isaac/6.1",
            omni.client.Result.ERROR_NOT_FOUND,
            stat_results=[(omni.client.Result.ERROR_NOT_FOUND, None), (omni.client.Result.OK, None)],
        )

        self.assertEqual(result, omni.client.Result.OK)
        self.assertEqual(version, Version("6.1.0"))
        self.assertEqual(stat_mock.call_count, 2)

    def test_missing_version_file_rejects_root_without_asset_directories(self) -> None:
        """A root without either recognized asset directory remains unverified."""
        result, version, stat_mock = self._verify_with_mocks(
            "https://example.com/Assets/Isaac/6.1",
            omni.client.Result.ERROR_NOT_FOUND,
            stat_results=[
                (omni.client.Result.ERROR_NOT_FOUND, None),
                (omni.client.Result.ERROR_NOT_FOUND, None),
            ],
        )

        self.assertEqual(result, omni.client.Result.ERROR_NOT_FOUND)
        self.assertEqual(version, "")
        self.assertEqual(
            [call.args[0] for call in stat_mock.call_args_list],
            [
                "https://example.com/Assets/Isaac/6.1/Isaac",
                "https://example.com/Assets/Isaac/6.1/NVIDIA",
            ],
        )

    def test_missing_version_file_rejects_unsupported_directory_metadata(self) -> None:
        """Unsupported directory metadata does not verify an HTTP asset root."""
        result, version, stat_mock = self._verify_with_mocks(
            "https://example.com/Assets/Isaac/6.1",
            omni.client.Result.ERROR_NOT_FOUND,
            stat_results=[
                (omni.client.Result.ERROR_NOT_SUPPORTED, None),
                (omni.client.Result.ERROR_NOT_SUPPORTED, None),
            ],
        )

        self.assertEqual(result, omni.client.Result.ERROR_NOT_FOUND)
        self.assertEqual(version, "")
        self.assertEqual(stat_mock.call_count, 2)

    def test_missing_version_file_rejects_unversioned_root(self) -> None:
        """An HTTP root without a semantic version path remains unverified."""
        result, version, stat_mock = self._verify_with_mocks(
            "https://example.com/Assets/Isaac/latest",
            omni.client.Result.ERROR_NOT_FOUND,
        )

        self.assertEqual(result, omni.client.Result.ERROR_NOT_FOUND)
        self.assertEqual(version, "")
        stat_mock.assert_not_called()

    def test_missing_version_file_rejects_nonnumeric_version_without_parse_error(self) -> None:
        """A nonnumeric root version is rejected without blaming version.txt content."""
        with patch.object(carb, "log_info") as log_mock:
            result, version, stat_mock = self._verify_with_mocks(
                "https://example.com/Assets/Isaac/6.invalid",
                omni.client.Result.ERROR_NOT_FOUND,
            )

        self.assertEqual(result, omni.client.Result.ERROR_NOT_FOUND)
        self.assertEqual(version, "")
        stat_mock.assert_not_called()
        self.assertFalse(
            any("Unable to parse version file" in call.args[0] for call in log_mock.call_args_list if call.args)
        )

    def test_missing_version_file_preserves_version_mismatch(self) -> None:
        """A valid root for a different Isaac Sim minor version is rejected."""
        result, version, _ = self._verify_with_mocks(
            "https://example.com/Assets/Isaac/6.0",
            omni.client.Result.ERROR_NOT_FOUND,
            stat_results=[(omni.client.Result.OK, None)],
        )

        self.assertEqual(result, omni.client.Result.ERROR_BAD_VERSION)
        self.assertEqual(version, Version("6.0.0"))

    def test_non_not_found_version_error_does_not_fallback(self) -> None:
        """Access and transport errors do not trust the root path version."""
        result, version, stat_mock = self._verify_with_mocks(
            "https://example.com/Assets/Isaac/6.1",
            omni.client.Result.ERROR_ACCESS_DENIED,
        )

        self.assertEqual(result, omni.client.Result.ERROR_NOT_FOUND)
        self.assertEqual(version, "")
        stat_mock.assert_not_called()

    def test_malformed_version_file_does_not_fallback(self) -> None:
        """Malformed version-file content is not replaced with the root path version."""
        result, version, stat_mock = self._verify_with_mocks(
            "https://example.com/Assets/Isaac/6.1",
            omni.client.Result.OK,
            b"invalid",
        )

        self.assertEqual(result, omni.client.Result.ERROR_NOT_FOUND)
        self.assertEqual(version, "")
        stat_mock.assert_not_called()


class TestStorageNative(omni.kit.test.AsyncTestCase):
    """Test suite for Isaac Sim storage native extension functionality.

    Tests cover asset path resolution, file discovery, filtering, and Nucleus server
    connectivity functions.
    """

    async def setUp(self) -> None:
        """Set up test fixtures before each test method."""
        await omni.kit.app.get_app().next_update_async()

    async def tearDown(self) -> None:
        """Clean up after each test method."""
        await omni.kit.app.get_app().next_update_async()

    async def test_get_assets_root_path(self) -> None:
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

    async def test_get_assets_root_path_accepts_isaac_only_profile_root(self) -> None:
        """Accept an asset-region-profile root when its mirrored subset contains only Isaac assets."""
        settings = carb.settings.get_settings()
        default_assets_url = settings.get("/persistent/isaac/asset_root/default")
        profile_assets_url = settings.get("/exts/isaacsim.storage.native/asset_region_profiles/china/asset_root")

        def stat_asset_directory(url: str) -> tuple[omni.client.Result, None]:
            result = omni.client.Result.OK if url.endswith("/Isaac") else omni.client.Result.ERROR_NOT_FOUND
            return result, None

        settings.set("/persistent/isaac/asset_root/default", profile_assets_url)
        try:
            with (
                patch.object(omni.client, "stat_with_callback", side_effect=_stat_with_callback(stat_asset_directory)),
                patch.object(omni.client, "stat_async", new=AsyncMock(side_effect=stat_asset_directory)),
            ):
                self.assertEqual(get_assets_root_path(), profile_assets_url)
                self.assertEqual(await get_assets_root_path_async(), profile_assets_url)
        finally:
            settings.set("/persistent/isaac/asset_root/default", default_assets_url)

    async def test_get_assets_root_path_accepts_unsupported_metadata(self) -> None:
        """Configured asset roots remain usable when the provider cannot stat directories."""
        settings = carb.settings.get_settings()
        default_assets_url = settings.get("/persistent/isaac/asset_root/default")
        storage_assets_url = "omniverse://storage.example.com"
        settings.set("/persistent/isaac/asset_root/default", storage_assets_url)

        try:
            with (
                patch.object(
                    omni.client,
                    "stat_with_callback",
                    side_effect=_stat_with_callback(lambda url: (omni.client.Result.ERROR_NOT_SUPPORTED, None)),
                ) as stat_mock,
                patch.object(
                    omni.client,
                    "stat_async",
                    new=AsyncMock(return_value=(omni.client.Result.ERROR_NOT_SUPPORTED, None)),
                ) as stat_async_mock,
            ):
                self.assertEqual(get_assets_root_path(), storage_assets_url)
                self.assertEqual(await get_assets_root_path_async(), storage_assets_url)

            self.assertEqual(stat_mock.call_count, 1)
            self.assertEqual(stat_async_mock.await_count, 1)
        finally:
            settings.set("/persistent/isaac/asset_root/default", default_assets_url)

    async def test_get_assets_root_path_rejects_access_denied_metadata(self) -> None:
        """Permission failures are not mistaken for unsupported directory metadata."""
        settings = carb.settings.get_settings()
        default_assets_url = settings.get("/persistent/isaac/asset_root/default")
        storage_assets_url = "omniverse://storage.example.com"
        settings.set("/persistent/isaac/asset_root/default", storage_assets_url)

        try:
            with patch.object(
                omni.client,
                "stat_with_callback",
                side_effect=_stat_with_callback(lambda url: (omni.client.Result.ERROR_ACCESS_DENIED, None)),
            ):
                with self.assertRaises(RuntimeError):
                    get_assets_root_path()
            with patch.object(
                omni.client,
                "stat_async",
                new=AsyncMock(return_value=(omni.client.Result.ERROR_ACCESS_DENIED, None)),
            ):
                with self.assertRaises(RuntimeError):
                    await get_assets_root_path_async()
        finally:
            settings.set("/persistent/isaac/asset_root/default", default_assets_url)

    async def test_get_assets_root_path_can_require_supported_metadata(self) -> None:
        """Strict callers can reject providers that cannot stat asset directories."""
        settings = carb.settings.get_settings()
        default_assets_url = settings.get("/persistent/isaac/asset_root/default")
        storage_assets_url = "omniverse://storage.example.com"
        settings.set("/persistent/isaac/asset_root/default", storage_assets_url)

        try:
            with patch.object(
                omni.client,
                "stat_with_callback",
                side_effect=_stat_with_callback(lambda url: (omni.client.Result.ERROR_NOT_SUPPORTED, None)),
            ):
                with self.assertRaises(RuntimeError):
                    get_assets_root_path(accept_unsupported=False)
            with patch.object(
                omni.client,
                "stat_async",
                new=AsyncMock(return_value=(omni.client.Result.ERROR_NOT_SUPPORTED, None)),
            ):
                with self.assertRaises(RuntimeError):
                    await get_assets_root_path_async(accept_unsupported=False)
        finally:
            settings.set("/persistent/isaac/asset_root/default", default_assets_url)

    async def test_find_filtered_files_async_basic_discovery(self) -> None:
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

    async def test_find_filtered_files_async_pattern_filtering_any_mode(self) -> None:
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

    async def test_find_filtered_files_async_pattern_filtering_all_mode(self) -> None:
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

    async def test_find_filtered_files_async_filepath_exclusion(self) -> None:
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

    async def test_find_filtered_files_async_max_depth_limiting(self) -> None:
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

    async def test_find_filtered_files_async_async_behavior(self) -> None:
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

    async def test_find_filtered_files_async_error_handling(self) -> None:
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

    async def test_find_filtered_files_async_combined_parameters(self) -> None:
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

    async def test_resolve_asset_path_resolves_with_assets_root(self) -> None:
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

    async def test_resolve_asset_path_returns_original_if_exists(self) -> None:
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

    async def test_resolve_asset_path_returns_none_if_missing(self) -> None:
        """Test resolve_asset_path returns None when neither path exists.

        Verifies that when given a path that does not exist in either its
        original form or when prefixed with the assets root, the function
        returns None.
        """
        # Expect the function to return None for a non-existent path
        original = "/nonexistent/path/definitely_missing.usd"
        resolved = resolve_asset_path(original)
        self.assertIsNone(resolved)
