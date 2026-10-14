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

"""Test module artifact staging for Kit extensions."""

from __future__ import annotations

import importlib.util
import json
import os
import sys
import tempfile
import unittest
from pathlib import Path
from unittest import mock

STAGE_TOOL_PATH = Path(__file__).resolve().parents[1] / "stage_extension_wheel.py"
STAGE_TOOL_SPEC = importlib.util.spec_from_file_location("isaacsim_module_stage_extension_wheel", STAGE_TOOL_PATH)
if STAGE_TOOL_SPEC is None or STAGE_TOOL_SPEC.loader is None:
    raise RuntimeError(f"Cannot load wheel staging tool from {STAGE_TOOL_PATH}")
stage_tool = importlib.util.module_from_spec(STAGE_TOOL_SPEC)
sys.modules[STAGE_TOOL_SPEC.name] = stage_tool
STAGE_TOOL_SPEC.loader.exec_module(stage_tool)


class LegacyManifestTests(unittest.TestCase):
    """Cover old manifests throughout discovery and metadata readers."""

    def test_legacy_projection_and_dependency_metadata(self) -> None:
        """Keep projected package ownership and provider mappings in old manifests."""
        with tempfile.TemporaryDirectory() as temporary_directory:
            root = Path(temporary_directory)
            extension = root / "source/extensions/acme.runtime"
            extension.mkdir(parents=True)
            (extension / "module-carrier.toml").write_text(
                '[carrier]\ndistribution="acme_runtime"\npython-modules=["acme.runtime"]\n'
                '[carrier.dependency-extensions]\nacme_shared=["acme.shared"]\n',
                encoding="utf-8",
            )
            current_path = extension / "wheel-staging.toml"
            self.assertEqual(stage_tool._read_wheel_distribution(current_path), "acme_runtime")
            self.assertEqual(stage_tool._read_wheel_python_modules(current_path), ("acme.runtime",))
            self.assertEqual(
                stage_tool._read_wheel_dependency_extensions(current_path), {"acme_shared": ("acme.shared",)}
            )
            self.assertEqual(stage_tool._staging_extensions_by_distribution(root), {"acme_runtime": ("acme.runtime",)})

    def test_matching_dual_manifests_share_metadata(self) -> None:
        """Permit a transition retaining both equivalent source spellings."""
        with tempfile.TemporaryDirectory() as temporary_directory:
            root = Path(temporary_directory)
            for filename, table in (("wheel-staging.toml", "wheel"), ("module-carrier.toml", "carrier")):
                (root / filename).write_text(f'[{table}]\ndistribution="acme_runtime"\n', encoding="utf-8")
            self.assertEqual(stage_tool._read_wheel_distribution(root / "wheel-staging.toml"), "acme_runtime")

    def test_conflicting_dual_manifests_fail(self) -> None:
        """Do not silently select a different distribution from conflicting files."""
        with tempfile.TemporaryDirectory() as temporary_directory:
            root = Path(temporary_directory)
            current = root / "wheel-staging.toml"
            current.write_text('[wheel]\ndistribution="acme_new"\n', encoding="utf-8")
            (root / "module-carrier.toml").write_text('[carrier]\ndistribution="acme_old"\n', encoding="utf-8")
            with self.assertRaisesRegex(RuntimeError, "Conflicting staging manifests"):
                stage_tool._read_wheel_distribution(current)


class WheelStagingManifestTests(unittest.TestCase):
    """Test the source-side wheel staging contract."""

    def test_manifest_selects_distribution(self) -> None:
        """Read the distribution staged in an extension."""
        with tempfile.TemporaryDirectory() as temporary_directory:
            root = Path(temporary_directory)
            extension = root / "source" / "extensions" / "acme.runtime"
            extension.mkdir(parents=True)
            (extension / "wheel-staging.toml").write_text('[wheel]\ndistribution = "acme_runtime"\n', encoding="utf-8")
            arguments = stage_tool._Arguments(
                repo_root=root,
                extension="acme.runtime",
                artifacts_directory=root / "artifacts",
                build_directory=root / "build",
                configuration="release",
                cmake=root / "cmake",
            )

            extension_root, distribution = stage_tool._load_wheel_staging_manifest(arguments)

            self.assertEqual(extension_root, extension)
            self.assertEqual(distribution, "acme_runtime")

    def test_wheel_variant_is_explicit(self) -> None:
        """Read the wheel payload variant selected by an extension."""
        with tempfile.TemporaryDirectory() as temporary_directory:
            manifest = Path(temporary_directory) / "wheel-staging.toml"
            manifest.write_text(
                '[wheel]\ndistribution = "acme_schema"\n' 'wheel-variant = "usd2511"\n',
                encoding="utf-8",
            )

            variant = stage_tool._read_wheel_variant(manifest)

            self.assertEqual(variant, "usd2511")

    def test_invalid_wheel_variant_is_rejected(self) -> None:
        """Reject wheel variants outside the build tool's identifier contract."""
        with tempfile.TemporaryDirectory() as temporary_directory:
            manifest = Path(temporary_directory) / "wheel-staging.toml"
            manifest.write_text(
                '[wheel]\ndistribution = "acme_schema"\nwheel-variant = "usd.2511"\n',
                encoding="utf-8",
            )

            with self.assertRaisesRegex(RuntimeError, "must be a lowercase identifier"):
                stage_tool._read_wheel_variant(manifest)

    def test_manifest_selects_python_module_projection(self) -> None:
        """Read the leaf packages projected from a shared distribution."""
        with tempfile.TemporaryDirectory() as temporary_directory:
            manifest = Path(temporary_directory) / "wheel-staging.toml"
            manifest.write_text(
                '[wheel]\ndistribution = "acme_runtime"\npython-modules = ["acme.runtime"]\n',
                encoding="utf-8",
            )

            python_modules = stage_tool._read_wheel_python_modules(manifest)

            self.assertEqual(python_modules, ("acme.runtime",))

    def test_overlapping_python_module_projection_is_rejected(self) -> None:
        """Reject parent and child packages in one projection."""
        with tempfile.TemporaryDirectory() as temporary_directory:
            manifest = Path(temporary_directory) / "wheel-staging.toml"
            manifest.write_text(
                '[wheel]\ndistribution = "acme_runtime"\n' 'python-modules = ["acme.runtime", "acme.runtime.tools"]\n',
                encoding="utf-8",
            )

            with self.assertRaisesRegex(RuntimeError, "overlapping Python module projections"):
                stage_tool._read_wheel_python_modules(manifest)

    def test_manifest_selects_current_platform_wheel_paths(self) -> None:
        """Add native payloads that exist only in the selected platform wheel."""
        with tempfile.TemporaryDirectory() as temporary_directory:
            manifest = Path(temporary_directory) / "wheel-staging.toml"
            manifest.write_text(
                '[wheel]\ndistribution = "acme_runtime"\nwheel-paths = ["_acme*"]\n'
                "[wheel.platform-wheel-paths]\n"
                'win_amd64 = ["acme.dll"]\n'
                'linux_x86_64 = ["libacme.so"]\n',
                encoding="utf-8",
            )

            wheel_paths = stage_tool._read_wheel_paths(manifest, platform_tag="win_amd64")

            self.assertEqual(wheel_paths, ("_acme*", "acme.dll"))

    def test_invalid_platform_wheel_paths_are_rejected(self) -> None:
        """Validate every platform payload declaration, including inactive ones."""
        with tempfile.TemporaryDirectory() as temporary_directory:
            manifest = Path(temporary_directory) / "wheel-staging.toml"
            manifest.write_text(
                '[wheel]\ndistribution = "acme_runtime"\n'
                "[wheel.platform-wheel-paths]\n"
                'win_amd64 = ["../acme.dll"]\n',
                encoding="utf-8",
            )

            with self.assertRaisesRegex(RuntimeError, "must be safe relative paths"):
                stage_tool._read_wheel_paths(manifest, platform_tag="linux_x86_64")

    def test_manifest_selects_dependency_extension_providers(self) -> None:
        """Read Kit extensions that provide a dependency without a staged wheel."""
        with tempfile.TemporaryDirectory() as temporary_directory:
            manifest = Path(temporary_directory) / "wheel-staging.toml"
            manifest.write_text(
                '[wheel]\ndistribution = "acme_feature"\n'
                "[wheel.dependency-extensions]\n"
                'acme_core = ["acme.core", "acme.core.tools"]\n',
                encoding="utf-8",
            )

            dependency_extensions = stage_tool._read_wheel_dependency_extensions(manifest)

            self.assertEqual(dependency_extensions, {"acme_core": ("acme.core", "acme.core.tools")})

    def test_invalid_dependency_extension_providers_are_rejected(self) -> None:
        """Reject empty provider lists that cannot satisfy a package dependency."""
        with tempfile.TemporaryDirectory() as temporary_directory:
            manifest = Path(temporary_directory) / "wheel-staging.toml"
            manifest.write_text(
                '[wheel]\ndistribution = "acme_feature"\n' "[wheel.dependency-extensions]\n" "acme_core = []\n",
                encoding="utf-8",
            )

            with self.assertRaisesRegex(RuntimeError, "must map lowercase underscore-separated"):
                stage_tool._read_wheel_dependency_extensions(manifest)

    def test_extension_path_traversal_is_rejected(self) -> None:
        """Keep extension selection within the repository extension root."""
        with tempfile.TemporaryDirectory() as temporary_directory:
            root = Path(temporary_directory)
            arguments = stage_tool._Arguments(
                repo_root=root,
                extension="../outside",
                artifacts_directory=root / "artifacts",
                build_directory=root / "build",
                configuration="release",
                cmake=root / "cmake",
            )

            with self.assertRaisesRegex(RuntimeError, "Invalid extension name"):
                stage_tool._load_wheel_staging_manifest(arguments)


class VersionMetadataTests(unittest.TestCase):
    """Test shared release metadata validation."""

    def test_package_manifest_requires_expected_release_version(self) -> None:
        """Reject package metadata from another release."""
        with tempfile.TemporaryDirectory() as temporary_directory:
            path = Path(temporary_directory) / "package.json"
            path.write_text(
                json.dumps(
                    {
                        "name": "acme_runtime",
                        "version": "7.0.1",
                        "modules": ["acme.runtime.logging"],
                        "python_imports": [],
                        "dependencies": {},
                        "complete": True,
                    }
                ),
                encoding="utf-8",
            )

            with self.assertRaisesRegex(RuntimeError, "does not match"):
                stage_tool._load_package_manifest(path, "acme_runtime", "7.0.0.dev0")


class WheelStagingDependencyTests(unittest.TestCase):
    """Test the package-to-extension dependency mirror."""

    @staticmethod
    def _write_wheel_staging_manifest(
        root: Path,
        extension_name: str,
        distribution: str,
        dependencies: str = "",
        python_modules: tuple[str, ...] = (),
    ) -> Path:
        """Write one minimal wheel staging manifest and extension configuration.

        Args:
            root: Root directory to process.
            extension_name: Extension name.
            distribution: Python distribution name.
            dependencies: Package dependency names.
            python_modules: Python module names supplied by the package.

        Returns:
            The resulting value.
        """
        extension = root / "source" / "extensions" / extension_name
        (extension / "config").mkdir(parents=True)
        projection = ""
        if python_modules:
            quoted_modules = ", ".join(f'"{module}"' for module in python_modules)
            projection = f"python-modules = [{quoted_modules}]\n"
        (extension / "wheel-staging.toml").write_text(
            f'[wheel]\ndistribution = "{distribution}"\n{projection}',
            encoding="utf-8",
        )
        (extension / "config" / "extension.toml").write_text(
            f'[package]\nversion = "1.2.3"\n{dependencies}',
            encoding="utf-8",
        )
        return extension

    def test_internal_package_dependency_requires_registered_extension_dependency(self) -> None:
        """Accept an internal dependency mirrored by its Kit extension."""
        with tempfile.TemporaryDirectory() as temporary_directory:
            root = Path(temporary_directory)
            extension = self._write_wheel_staging_manifest(
                root,
                "acme.feature",
                "acme_feature",
                '\n[dependencies]\n"acme.core" = {}\n',
            )
            self._write_wheel_staging_manifest(root, "acme.core", "acme_core")

            stage_tool._validate_extension_dependencies(
                root,
                extension,
                {"dependencies": {"acme_core": "==1.2.3"}},
            )

    def test_missing_extension_dependency_is_rejected(self) -> None:
        """Reject an extension missing its package dependency's Kit extension."""
        with tempfile.TemporaryDirectory() as temporary_directory:
            root = Path(temporary_directory)
            extension = self._write_wheel_staging_manifest(root, "acme.feature", "acme_feature")
            self._write_wheel_staging_manifest(root, "acme.core", "acme_core")

            with self.assertRaisesRegex(RuntimeError, "must declare staged wheel dependency 'acme.core'"):
                stage_tool._validate_extension_dependencies(
                    root,
                    extension,
                    {"dependencies": {"acme_core": "==1.2.3"}},
                )

    def test_package_dependency_without_staged_wheel_or_provider_is_rejected(self) -> None:
        """Reject an internal package dependency with no declared Kit provider."""
        with tempfile.TemporaryDirectory() as temporary_directory:
            root = Path(temporary_directory)
            extension = self._write_wheel_staging_manifest(root, "acme.feature", "acme_feature")

            with self.assertRaisesRegex(RuntimeError, "has no extension that stages its wheel"):
                stage_tool._validate_extension_dependencies(
                    root,
                    extension,
                    {"dependencies": {"acme_core": "==1.2.3"}},
                )

    def test_dependency_extension_providers_are_accepted(self) -> None:
        """Accept existing Kit extensions declared as package dependency providers."""
        with tempfile.TemporaryDirectory() as temporary_directory:
            root = Path(temporary_directory)
            extension = self._write_wheel_staging_manifest(
                root,
                "acme.feature",
                "acme_feature",
                '\n[dependencies]\n"acme.core" = {}\n"acme.core.tools" = {}\n',
            )
            with (extension / "wheel-staging.toml").open("a", encoding="utf-8") as stream:
                stream.write('[wheel.dependency-extensions]\nacme_core = ["acme.core", "acme.core.tools"]\n')

            stage_tool._validate_extension_dependencies(
                root,
                extension,
                {"dependencies": {"acme_core": "==1.2.3"}},
            )

    def test_missing_dependency_extension_provider_is_rejected(self) -> None:
        """Require every declared provider to be an extension dependency."""
        with tempfile.TemporaryDirectory() as temporary_directory:
            root = Path(temporary_directory)
            extension = self._write_wheel_staging_manifest(
                root,
                "acme.feature",
                "acme_feature",
                '\n[dependencies]\n"acme.core" = {}\n',
            )
            with (extension / "wheel-staging.toml").open("a", encoding="utf-8") as stream:
                stream.write('[wheel.dependency-extensions]\nacme_core = ["acme.core", "acme.core.tools"]\n')

            with self.assertRaisesRegex(RuntimeError, "must declare provider dependency 'acme.core.tools'"):
                stage_tool._validate_extension_dependencies(
                    root,
                    extension,
                    {"dependencies": {"acme_core": "==1.2.3"}},
                )

    def test_duplicate_full_distribution_stages_are_rejected(self) -> None:
        """Require projections when more than one extension shares a distribution."""
        with tempfile.TemporaryDirectory() as temporary_directory:
            root = Path(temporary_directory)
            self._write_wheel_staging_manifest(root, "acme.first", "acme_core")
            self._write_wheel_staging_manifest(root, "acme.second", "acme_core")

            with self.assertRaisesRegex(RuntimeError, "every staging extension must declare"):
                stage_tool._staging_extensions_by_distribution(root)

    def test_disjoint_distribution_projections_are_accepted(self) -> None:
        """Allow multiple extensions to project separate packages from one wheel."""
        with tempfile.TemporaryDirectory() as temporary_directory:
            root = Path(temporary_directory)
            self._write_wheel_staging_manifest(root, "acme.first", "acme_core", python_modules=("acme.first",))
            self._write_wheel_staging_manifest(root, "acme.second", "acme_core", python_modules=("acme.second",))

            staged_extensions = stage_tool._staging_extensions_by_distribution(root)

            self.assertEqual(staged_extensions["acme_core"], ("acme.first", "acme.second"))

    def test_overlapping_distribution_projections_are_rejected(self) -> None:
        """Keep each package subtree owned by exactly one extension."""
        with tempfile.TemporaryDirectory() as temporary_directory:
            root = Path(temporary_directory)
            self._write_wheel_staging_manifest(root, "acme.first", "acme_core", python_modules=("acme.runtime",))
            self._write_wheel_staging_manifest(root, "acme.second", "acme_core", python_modules=("acme.runtime.tools",))

            with self.assertRaisesRegex(RuntimeError, "Overlapping Python modules"):
                stage_tool._staging_extensions_by_distribution(root)

    def test_package_dependency_requires_every_distribution_projection(self) -> None:
        """Mirror every extension needed to make a package dependency complete in Kit."""
        with tempfile.TemporaryDirectory() as temporary_directory:
            root = Path(temporary_directory)
            extension = self._write_wheel_staging_manifest(
                root,
                "acme.feature",
                "acme_feature",
                '\n[dependencies]\n"acme.core.first" = {}\n',
            )
            self._write_wheel_staging_manifest(
                root, "acme.core.first", "acme_core", python_modules=("acme.core.first",)
            )
            self._write_wheel_staging_manifest(
                root, "acme.core.second", "acme_core", python_modules=("acme.core.second",)
            )

            with self.assertRaisesRegex(RuntimeError, "must declare staged wheel dependency 'acme.core.second'"):
                stage_tool._validate_extension_dependencies(
                    root,
                    extension,
                    {"dependencies": {"acme_core": "==1.2.3"}},
                )


class NativeLibraryTests(unittest.TestCase):
    """Test validation of native libraries installed by a staged wheel."""

    def _write_extension_configuration(self, extension: Path, native_library: str) -> None:
        """Write a minimal extension configuration with one native library.

        Args:
            extension: Extension values.
            native_library: Native library name.
        """
        (extension / "config").mkdir(parents=True)
        (extension / "config" / "extension.toml").write_text(native_library, encoding="utf-8")

    def test_platform_tokens_resolve_staged_library(self) -> None:
        """Accept a tokenized path when it resolves to a staged library."""
        with tempfile.TemporaryDirectory() as temporary_directory:
            root = Path(temporary_directory)
            extension = root / "extension"
            stage = root / "stage"
            library = stage / "pip_prebundle" / "isaacsim" / "lib" / "libacme.so.3"
            library.parent.mkdir(parents=True)
            library.touch()
            self._write_extension_configuration(
                extension,
                '[[native.library]]\npath = "pip_prebundle/isaacsim/lib/${lib_prefix}acme${lib_ext}.3"\n',
            )
            tokens = ("linux-x86_64", {"${lib_prefix}": "lib", "${lib_ext}": ".so"})

            with mock.patch.object(stage_tool, "_native_library_tokens", return_value=tokens):
                stage_tool._validate_staged_native_libraries(extension, stage, "release")

    def test_missing_staged_library_is_rejected(self) -> None:
        """Reject a manifest whose active native-library path matches nothing."""
        with tempfile.TemporaryDirectory() as temporary_directory:
            root = Path(temporary_directory)
            extension = root / "extension"
            self._write_extension_configuration(
                extension,
                '[[native.library]]\npath = "pip_prebundle/isaacsim/lib/${lib_prefix}missing${lib_ext}.3"\n',
            )
            tokens = ("linux-x86_64", {"${lib_prefix}": "lib", "${lib_ext}": ".so"})

            with mock.patch.object(stage_tool, "_native_library_tokens", return_value=tokens):
                with self.assertRaisesRegex(RuntimeError, "is missing"):
                    stage_tool._validate_staged_native_libraries(extension, root / "stage", "release")

    def test_missing_wheel_alias_is_materialized_from_sdk(self) -> None:
        """Restore a native runtime alias omitted from a Python wheel."""
        with tempfile.TemporaryDirectory() as temporary_directory:
            root = Path(temporary_directory)
            extension = root / "extension"
            stage = root / "stage"
            sdk_library = root / "sdk" / "lib" / "libacme.so"
            sdk_library.parent.mkdir(parents=True)
            sdk_library.write_bytes(b"native-library")
            self._write_extension_configuration(
                extension,
                '[[native.library]]\npath = "pip_prebundle/isaacsim/lib/${lib_prefix}acme${lib_ext}"\n',
            )
            tokens = ("linux-x86_64", {"${lib_prefix}": "lib", "${lib_ext}": ".so"})

            with mock.patch.object(stage_tool, "_native_library_tokens", return_value=tokens):
                stage_tool._materialize_native_library_aliases(extension, stage, root / "sdk", "release")
                stage_tool._validate_staged_native_libraries(extension, stage, "release")

            staged_library = stage / "pip_prebundle" / "isaacsim" / "lib" / "libacme.so"
            self.assertEqual(staged_library.read_bytes(), b"native-library")

    def test_native_library_wildcard_is_rejected(self) -> None:
        """Reject a wildcard that the supported Kit runtime would treat literally."""
        with tempfile.TemporaryDirectory() as temporary_directory:
            root = Path(temporary_directory)
            extension = root / "extension"
            stage = root / "stage"
            library_directory = stage / "pip_prebundle" / "isaacsim" / "lib"
            library_directory.mkdir(parents=True)
            (library_directory / "libacme.so.3").touch()
            self._write_extension_configuration(
                extension,
                '[[native.library]]\npath = "pip_prebundle/isaacsim/lib/${lib_prefix}acme${lib_ext}*"\n',
            )
            tokens = ("linux-x86_64", {"${lib_prefix}": "lib", "${lib_ext}": ".so"})

            with mock.patch.object(stage_tool, "_native_library_tokens", return_value=tokens):
                with self.assertRaisesRegex(RuntimeError, "must not contain wildcards"):
                    stage_tool._validate_staged_native_libraries(extension, stage, "release")

    def test_platform_filter_overrides_default_path(self) -> None:
        """Validate the active platform override rather than the default path."""
        with tempfile.TemporaryDirectory() as temporary_directory:
            root = Path(temporary_directory)
            extension = root / "extension"
            stage = root / "stage"
            library = stage / "pip_prebundle" / "acme.dll"
            library.parent.mkdir(parents=True)
            library.touch()
            self._write_extension_configuration(
                extension,
                "[[native.library]]\n"
                'path = "pip_prebundle/libacme.so*"\n'
                '"filter:platform"."windows-x86_64".path = "pip_prebundle/acme${lib_ext}"\n',
            )
            tokens = ("windows-x86_64", {"${lib_ext}": ".dll"})

            with mock.patch.object(stage_tool, "_native_library_tokens", return_value=tokens):
                stage_tool._validate_staged_native_libraries(extension, stage, "release")


class ArtifactTests(unittest.TestCase):
    """Test deterministic artifact selection and standards-based installation."""

    def test_windows_alias_passes_literal_paths_to_junction_api(self) -> None:
        """Keep shell metacharacters literal when creating a Windows junction."""
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory) / "path&%TEMP%(literal)"
            current = root / "extension-wheel-stages/release/acme.runtime"
            legacy = root / "module carriers/release/acme.runtime"
            current.mkdir(parents=True)
            temporary = legacy.with_name(".acme.runtime-test-junction")

            def create_junction(source: str, destination: str) -> None:
                self.assertEqual(source, str(current.resolve()))
                self.assertEqual(destination, str(temporary))
                Path(destination).mkdir()

            winapi = mock.Mock(CreateJunction=mock.Mock(side_effect=create_junction))
            with (
                mock.patch.object(stage_tool._PATHS.sys, "platform", "win32"),
                mock.patch.object(stage_tool._PATHS.uuid, "uuid4", return_value=mock.Mock(hex="test-junction")),
                mock.patch.dict(sys.modules, {"_winapi": winapi}),
            ):
                stage_tool._PATHS.ensure_directory_alias(legacy, current)

            winapi.CreateJunction.assert_called_once_with(str(current.resolve()), str(temporary))
            self.assertTrue(legacy.is_dir())
            self.assertFalse(temporary.exists())

    def test_legacy_stage_alias_tracks_repeated_publication(self) -> None:
        """Replace only the selected old stage after a new payload is available."""
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            extension = root / "source/extensions/acme.runtime"
            extension.mkdir(parents=True)
            (extension / "wheel-staging.toml").write_text('[wheel]\ndistribution="acme_runtime"\n', encoding="utf-8")
            arguments = stage_tool._Arguments(
                root, "acme.runtime", root / "artifacts", root / "build", "release", root / "cmake"
            )
            current = stage_tool._wheel_stage_directory(arguments)
            legacy = root / "_cmake_build/module-carriers/release/acme.runtime"
            sibling = legacy.parent / "acme.other"
            sibling.mkdir(parents=True)
            legacy.mkdir(parents=True)
            (legacy / "old").touch()

            payload = "first"

            def install_wheel(wheel: Path, destination: Path, distribution: str, version: str) -> None:
                destination.mkdir(parents=True)
                (destination / payload).touch()

            def install_sdk(arguments: stage_tool._Arguments, destination: Path, distribution: str) -> None:
                (destination / "include").mkdir(parents=True)
                (destination / "include/header").write_text(payload, encoding="utf-8")

            with (
                mock.patch.object(stage_tool, "_read_version", return_value="1.0"),
                mock.patch.object(stage_tool, "_select_wheel", return_value=root / "runtime.whl"),
                mock.patch.object(stage_tool, "_install_wheel", side_effect=install_wheel),
                mock.patch.object(stage_tool, "_install_sdk", side_effect=install_sdk),
                mock.patch.object(stage_tool, "_load_package_manifest", return_value={}),
                mock.patch.object(stage_tool, "_validate_matching_package_licenses"),
                mock.patch.object(stage_tool, "_validate_extension_dependencies"),
                mock.patch.object(stage_tool, "_materialize_native_library_aliases"),
                mock.patch.object(stage_tool, "_validate_staged_native_libraries") as validate,
            ):
                validate.side_effect = RuntimeError("invalid payload")
                with self.assertRaisesRegex(RuntimeError, "invalid payload"):
                    stage_tool._stage(arguments)
                self.assertTrue((legacy / "old").is_file())
                validate.side_effect = None

                stage_tool._stage(arguments)
                self.assertTrue((legacy / "pip_prebundle/first").samefile(current / "pip_prebundle/first"))
                self.assertEqual((legacy / "sdk/include/header").read_text(encoding="utf-8"), "first")
                self.assertFalse((legacy / "old").exists())

                payload = "second"
                stage_tool._stage(arguments)
                self.assertTrue((legacy / "pip_prebundle/second").samefile(current / "pip_prebundle/second"))
                self.assertEqual((legacy / "sdk/include/header").read_text(encoding="utf-8"), "second")
                self.assertFalse((legacy / "pip_prebundle/first").exists())
                self.assertTrue(sibling.is_dir())

    @unittest.skipIf(os.name == "nt", "This symlink failure fixture requires Unix filesystem semantics")
    def test_legacy_stage_survives_alias_creation_failure(self) -> None:
        """Do not discard a populated old stage when a new link cannot be made."""
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            current = root / "extension-wheel-stages/release/acme.runtime"
            legacy = root / "module-carriers/release/acme.runtime"
            current.mkdir(parents=True)
            legacy.mkdir(parents=True)
            payload = legacy / "old"
            payload.touch()
            with mock.patch.object(Path, "symlink_to", side_effect=OSError("links unavailable")):
                with self.assertRaisesRegex(OSError, "links unavailable"):
                    stage_tool._PATHS.ensure_directory_alias(legacy, current)
            self.assertTrue(payload.exists())

    def test_wheel_stage_is_owned_by_library_build_root(self) -> None:
        """Keep the wheel staging directory alive when Repo cleans its application output."""
        arguments = stage_tool._Arguments(
            repo_root=Path("/repo"),
            extension="acme.runtime",
            artifacts_directory=Path("/artifacts"),
            build_directory=Path("/build"),
            configuration="release",
            cmake=Path("/cmake"),
        )

        self.assertEqual(
            stage_tool._wheel_stage_directory(arguments),
            Path("/repo/_cmake_build/extension-wheel-stages/release/acme.runtime"),
        )

    def test_select_single_rejects_ambiguous_artifacts(self) -> None:
        """Reject stale duplicate artifacts instead of choosing one implicitly."""
        with tempfile.TemporaryDirectory() as temporary_directory:
            directory = Path(temporary_directory)
            (directory / "first.whl").touch()
            (directory / "second.whl").touch()

            with self.assertRaisesRegex(RuntimeError, "found 2"):
                stage_tool._select_single(directory, "*.whl", "wheel")

    def test_select_wheel_accepts_platform_independent_artifact(self) -> None:
        """Accept a pure-Python wheel for a distribution without an explicit variant."""
        with tempfile.TemporaryDirectory() as temporary_directory:
            directory = Path(temporary_directory)
            wheel = directory / "acme_runtime-1.2.3-py3-none-any.whl"
            wheel.touch()

            selected = stage_tool._select_wheel(directory, "acme_runtime", "1.2.3", None)

            self.assertEqual(selected, wheel)

    def test_select_wheel_rejects_wrong_native_platform(self) -> None:
        """Do not stage a native wheel built for another platform."""
        with tempfile.TemporaryDirectory() as temporary_directory:
            directory = Path(temporary_directory)
            (directory / "acme_runtime-1.2.3-cp312-abi3-win_amd64.whl").touch()

            with (
                mock.patch.object(stage_tool, "_platform_tag", return_value="linux_x86_64"),
                self.assertRaisesRegex(RuntimeError, "found 0"),
            ):
                stage_tool._select_wheel(directory, "acme_runtime", "1.2.3", None)

    def test_wheel_install_uses_pip_index_semantics(self) -> None:
        """Ask pip to install the selected distribution without recording a direct URL."""
        with tempfile.TemporaryDirectory() as temporary_directory:
            root = Path(temporary_directory)
            wheel = root / "acme_runtime-1.2.3-py3-none-any.whl"
            wheel.touch()
            with mock.patch.object(stage_tool.subprocess, "run") as run:
                stage_tool._install_wheel(wheel, root / "stage", "acme_runtime", "1.2.3")

        command = run.call_args.args[0]
        self.assertIn("--no-index", command)
        self.assertIn("--find-links", command)
        self.assertIn("acme_runtime==1.2.3", command)
        self.assertNotIn(str(wheel), command)

    def test_python_projection_copies_only_selected_leaf_package(self) -> None:
        """Exclude sibling metadata while preserving required dependency notices."""
        with tempfile.TemporaryDirectory() as temporary_directory:
            root = Path(temporary_directory)
            installed = root / "installed"
            selected = installed / "acme" / "first"
            selected.mkdir(parents=True)
            (selected / "__init__.py").touch()
            sibling = installed / "acme" / "second"
            sibling.mkdir(parents=True)
            (sibling / "__init__.py").touch()
            dist_info = installed / "acme_core-1.2.3.dist-info"
            dist_info.mkdir()
            (dist_info / "METADATA").touch()
            license_path = installed / "share" / "licenses" / "acme_core" / stage_tool._PIP_LICENSE_FILENAME
            license_path.parent.mkdir(parents=True)
            license_path.write_bytes(b"acme dependency license\n")
            destination = root / "pip_prebundle"

            stage_tool._project_python_modules(
                installed,
                destination,
                ("acme.first",),
                {"name": "acme_core", "python_imports": ["acme.first", "acme.second"]},
                b"acme dependency license\n",
            )

            self.assertTrue((destination / "acme" / "first" / "__init__.py").is_file())
            self.assertFalse((destination / "acme" / "second").exists())
            self.assertFalse((destination / dist_info.name).exists())
            self.assertEqual(
                (destination / "share" / "licenses" / "acme_core" / stage_tool._PIP_LICENSE_FILENAME).read_text(
                    encoding="utf-8"
                ),
                "acme dependency license\n",
            )

    def test_python_projection_rejects_module_absent_from_manifest(self) -> None:
        """Require projections to be declared by the wheel package manifest."""
        with tempfile.TemporaryDirectory() as temporary_directory:
            root = Path(temporary_directory)

            with self.assertRaisesRegex(RuntimeError, "absent from the installed distribution"):
                stage_tool._project_python_modules(
                    root / "installed",
                    root / "pip_prebundle",
                    ("acme.missing",),
                    {"name": "acme_runtime", "python_imports": ["acme.runtime"]},
                    b"acme dependency license\n",
                )

    def test_python_projection_requires_license_aggregate(self) -> None:
        """Reject a projection that would drop its wheel's dependency notices."""
        with tempfile.TemporaryDirectory() as temporary_directory:
            root = Path(temporary_directory)
            installed = root / "installed"
            selected = installed / "acme" / "runtime"
            selected.mkdir(parents=True)
            (selected / "__init__.py").touch()

            with self.assertRaisesRegex(RuntimeError, "missing the package license aggregate"):
                stage_tool._project_python_modules(
                    installed,
                    root / "pip_prebundle",
                    ("acme.runtime",),
                    {"name": "acme_runtime", "python_imports": ["acme.runtime"]},
                    b"acme dependency license\n",
                )

    def test_python_projection_verifies_copied_license_aggregate(self) -> None:
        """Reject a projection when the copied dependency notices change."""
        with tempfile.TemporaryDirectory() as temporary_directory:
            root = Path(temporary_directory)
            installed = root / "installed"
            selected = installed / "acme" / "runtime"
            selected.mkdir(parents=True)
            (selected / "__init__.py").touch()
            license_path = installed / "share" / "licenses" / "acme_runtime" / stage_tool._PIP_LICENSE_FILENAME
            license_path.parent.mkdir(parents=True)
            license_path.write_bytes(b"expected license\n")

            def corrupt_copy(_source: Path, destination: Path) -> None:
                destination.write_bytes(b"corrupted license\n")

            with (
                mock.patch.object(stage_tool.shutil, "copy2", side_effect=corrupt_copy),
                self.assertRaisesRegex(RuntimeError, "does not match the wheel"),
            ):
                stage_tool._project_python_modules(
                    installed,
                    root / "pip_prebundle",
                    ("acme.runtime",),
                    {"name": "acme_runtime", "python_imports": ["acme.runtime"]},
                    b"expected license\n",
                )

    def test_wheel_and_sdk_license_aggregates_must_match(self) -> None:
        """Reject a wheel stage assembled from inconsistent redistributed notices."""
        with tempfile.TemporaryDirectory() as temporary_directory:
            root = Path(temporary_directory)
            for prefix, payload in ((root / "wheel", "wheel\n"), (root / "sdk", "sdk\n")):
                path = prefix / "share" / "licenses" / "acme_runtime" / stage_tool._PIP_LICENSE_FILENAME
                path.parent.mkdir(parents=True)
                path.write_text(payload, encoding="utf-8")

            with self.assertRaisesRegex(RuntimeError, "license aggregates do not match"):
                stage_tool._validate_matching_package_licenses(root / "wheel", root / "sdk", "acme_runtime")

    def test_wheel_path_projection_copies_matching_payloads(self) -> None:
        """Copy explicitly selected native and data payloads into a projected prebundle."""
        with tempfile.TemporaryDirectory() as temporary_directory:
            root = Path(temporary_directory)
            installed = root / "installed"
            native_library = installed / "_acme.cpython-312-x86_64-linux-gnu.so"
            native_library.parent.mkdir(parents=True)
            native_library.touch()
            private_libraries = installed / "_acme_libs"
            private_libraries.mkdir()
            (private_libraries / "libacme.so").touch()
            destination = root / "pip_prebundle"

            stage_tool._project_wheel_paths(installed, destination, ("_acme*",))

            self.assertTrue((destination / native_library.name).is_file())
            self.assertTrue((destination / "_acme_libs" / "libacme.so").is_file())

    def test_wheel_path_projection_requires_a_match(self) -> None:
        """Reject stale projections that no longer match wheel contents."""
        with tempfile.TemporaryDirectory() as temporary_directory:
            root = Path(temporary_directory)

            with self.assertRaisesRegex(RuntimeError, "did not match"):
                stage_tool._project_wheel_paths(root / "installed", root / "pip_prebundle", ("_missing*",))

    def test_sdk_install_uses_runtime_and_development_components(self) -> None:
        """Install the SDK directly from the distribution's CMake components."""
        with tempfile.TemporaryDirectory() as temporary_directory:
            root = Path(temporary_directory)
            arguments = stage_tool._Arguments(
                repo_root=root,
                extension="acme.runtime",
                artifacts_directory=root / "artifacts",
                build_directory=root / "build",
                configuration="release",
                cmake=root / "cmake",
            )
            with mock.patch.object(stage_tool.subprocess, "run") as run:
                stage_tool._install_sdk(arguments, root / "stage", "acme_runtime")

        self.assertEqual(run.call_count, 2)
        commands = [call.args[0] for call in run.call_args_list]
        self.assertIn("acme_runtime-runtime", commands[0])
        self.assertIn("acme_runtime-development", commands[1])
        self.assertIn("Release", commands[0])


if __name__ == "__main__":
    unittest.main()
