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

"""Test module wheel assembly without compiling module targets."""

from __future__ import annotations

import importlib.util
import json
import os
import sys
import tempfile
import unittest
import zipfile
from pathlib import Path
from types import ModuleType
from typing import Any
from unittest import mock


def _load_wheel_module() -> ModuleType:
    """Load the wheel tool without conflicting with the third-party wheel package.

    Returns:
        The resulting value.
    """
    path = Path(__file__).parents[1] / "wheel.py"
    spec = importlib.util.spec_from_file_location("isaacsim_module_wheel_tool", path)
    if spec is None or spec.loader is None:
        raise RuntimeError(f"Unable to load wheel tool: {path}")
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


_WHEEL = _load_wheel_module()


def _write_pyproject(path: Path, name: str) -> None:
    """Write the project metadata needed by package discovery.

    Args:
        path: Filesystem path to process.
        name: Name of the item.
    """
    path.parent.mkdir(parents=True)
    path.write_text(f"[project]\nname = {name!r}\n", encoding="utf-8")


def _write_test_wheel(
    path: Path,
    group: str,
    version: str,
    import_name: str,
    dependencies: dict[str, str] | None = None,
    requirements: tuple[str, ...] = (),
    modules: tuple[str, ...] | None = None,
    license_text: str | None = "acme dependency license\n",
) -> None:
    """Write a minimal valid wheel archive for content validation tests.

    Args:
        path: Filesystem path to process.
        group: Wheel distribution group.
        version: Package version.
        import_name: Top-level Python import name.
        dependencies: Package dependency names.
        requirements: Python requirement specifications.
        modules: Python modules included in the wheel.
        license_text: Package license contents.
    """
    distribution = group.replace("-", "_")
    manifest_path = f"{distribution}-{version}.data/data/share/isaacsim/packages/{group}/package.json"
    metadata_path = f"{distribution}-{version}.dist-info/METADATA"
    manifest = {
        "name": group,
        "version": version,
        "complete": True,
        "dependencies": dependencies or {},
        "python_imports": [import_name],
    }
    if modules is not None:
        manifest["modules"] = list(modules)
    requirement_metadata = "".join(f"Requires-Dist: {requirement}\n" for requirement in requirements)
    with zipfile.ZipFile(path, "w") as archive:
        archive.writestr(import_name.replace(".", "/") + "/__init__.py", "")
        archive.writestr(import_name.replace(".", "/") + "/py.typed", "")
        archive.writestr(manifest_path, json.dumps(manifest))
        if license_text is not None:
            license_path = f"{distribution}-{version}.data/data/share/licenses/{group}/{_WHEEL._PIP_LICENSE_FILENAME}"
            archive.writestr(license_path, license_text)
        archive.writestr(
            metadata_path,
            f"Metadata-Version: 2.2\nName: {group}\nVersion: {version}\n{requirement_metadata}",
        )


class WheelToolTests(unittest.TestCase):
    """Tests for package discovery, build validation, and wheel assembly."""

    def test_discover_groups_accepts_any_two_component_package_root(self) -> None:
        """Test discover groups accepts any two component package root."""
        with tempfile.TemporaryDirectory() as temporary_directory:
            repo_root = Path(temporary_directory)
            package_root = repo_root / "source" / "libraries" / "acme" / "robotics"
            _write_pyproject(package_root / "pyproject.toml", "acme-robotics")

            self.assertEqual(_WHEEL._discover_groups(repo_root), {"acme_robotics": package_root})

    def test_discover_groups_accepts_flat_package_root(self) -> None:
        """Test discover groups accepts flat package root."""
        with tempfile.TemporaryDirectory() as temporary_directory:
            repo_root = Path(temporary_directory)
            package_root = repo_root / "source" / "libraries" / "acme_robotics"
            _write_pyproject(package_root / "pyproject.toml", "acme-robotics")

            self.assertEqual(_WHEEL._discover_groups(repo_root), {"acme_robotics": package_root})

    def test_discover_groups_rejects_duplicate_flat_and_two_component_roots(self) -> None:
        """Test discover groups rejects duplicate flat and two component roots."""
        with tempfile.TemporaryDirectory() as temporary_directory:
            repo_root = Path(temporary_directory)
            flat_root = repo_root / "source" / "libraries" / "acme_robotics"
            nested_root = repo_root / "source" / "libraries" / "acme" / "robotics"
            _write_pyproject(flat_root / "pyproject.toml", "acme-robotics")
            _write_pyproject(nested_root / "pyproject.toml", "acme-robotics")

            with self.assertRaisesRegex(RuntimeError, "Duplicate module package group"):
                _WHEEL._discover_groups(repo_root)

    def test_discover_groups_uses_project_name_when_namespace_differs_from_distribution(
        self,
    ) -> None:
        """Test discover groups uses project name when namespace differs from distribution."""
        with tempfile.TemporaryDirectory() as temporary_directory:
            repo_root = Path(temporary_directory)
            pyproject = repo_root / "source" / "libraries" / "acme" / "robotics" / "pyproject.toml"
            _write_pyproject(pyproject, "isaacsim-robot-schema")

            self.assertEqual(
                _WHEEL._discover_groups(repo_root),
                {"isaacsim_robot_schema": pyproject.parent},
            )

    def test_discover_groups_rejects_invalid_project_name(self) -> None:
        """Test discover groups rejects invalid project name."""
        with tempfile.TemporaryDirectory() as temporary_directory:
            repo_root = Path(temporary_directory)
            package_root = repo_root / "source" / "libraries" / "acme_schemas" / "robot"
            _write_pyproject(package_root / "pyproject.toml", "123-invalid")

            with self.assertRaisesRegex(RuntimeError, "invalid distribution name"):
                _WHEEL._discover_groups(repo_root)

    def test_validate_cmake_build_checks_configuration_and_python(self) -> None:
        """Test validate cmake build checks configuration and python."""
        with tempfile.TemporaryDirectory() as temporary_directory:
            repo_root = Path(temporary_directory)
            build_directory = repo_root / "build"
            build_directory.mkdir()
            source_directory = repo_root / "source" / "libraries"
            source_directory.mkdir(parents=True)
            (build_directory / "CMakeCache.txt").write_text(
                "\n".join(
                    (
                        f"CMAKE_HOME_DIRECTORY:INTERNAL={source_directory.as_posix()}",
                        "CMAKE_BUILD_TYPE:STRING=Release",
                        "ISAACSIM_ENABLE_PYTHON:BOOL=ON",
                        "ISAACSIM_ENABLE_PYTHON_BINDINGS:BOOL=ON",
                        "ISAACSIM_ENFORCE_COMPLETE_PACKAGES:BOOL=ON",
                        f"Python_EXECUTABLE:FILEPATH={sys.executable}",
                    )
                ),
                encoding="utf-8",
            )
            build_dependencies = repo_root / "wheel-build"
            arguments = _WHEEL._WheelArguments(
                repo_root,
                build_directory,
                repo_root / "dist",
                "release",
                (),
                "cmake",
                build_dependencies,
            )

            _WHEEL._validate_cmake_build(arguments)
            incompatible = _WHEEL._WheelArguments(
                repo_root,
                build_directory,
                repo_root / "dist",
                "debug",
                (),
                "cmake",
                build_dependencies,
            )
            with self.assertRaisesRegex(RuntimeError, "CMAKE_BUILD_TYPE"):
                _WHEEL._validate_cmake_build(incompatible)

    def test_build_environment_disables_cmake_and_uses_unique_stage(self) -> None:
        """Test build environment disables cmake and uses unique stage."""
        with tempfile.TemporaryDirectory() as temporary_directory:
            repo_root = Path(temporary_directory)
            backend_path = repo_root / "wheel-build"
            backend_path.mkdir(parents=True)
            stage = repo_root / "stage"
            (stage / "python").mkdir(parents=True)
            (stage / "share").mkdir()
            arguments = _WHEEL._WheelArguments(
                repo_root,
                repo_root / "build",
                repo_root / "dist",
                "release",
                (),
                "cmake",
                backend_path,
            )

            with mock.patch.dict(os.environ, {"PYTHONPATH": "existing"}, clear=True):
                environment = _WHEEL._build_environment(arguments, stage)

            self.assertEqual(environment["SKBUILD_WHEEL_CMAKE"], "false")
            self.assertEqual(
                environment["SKBUILD_WHEEL_FORCE_INCLUDE"],
                f"{stage / 'python'}=.;{stage / 'share'}=${{SKBUILD_DATA_DIR}}/share",
            )
            self.assertEqual(environment["PYTHONPATH"], str(backend_path))
            self.assertEqual(environment["PYTHONDONTWRITEBYTECODE"], "1")

    def test_schema_variant_selects_component_and_pure_wheel_build_tag(self) -> None:
        """Keep USD-specific schema payloads distinguishable without making them native wheels."""
        with tempfile.TemporaryDirectory() as temporary_directory:
            root = Path(temporary_directory)
            project = root / "project"
            _write_pyproject(project / "pyproject.toml", "acme-schema")
            with (project / "pyproject.toml").open("a", encoding="utf-8") as stream:
                stream.write(
                    "\n[tool.isaacsim-library]\n"
                    'default-wheel-variant = "usd2505"\n'
                    'wheel-variants = ["usd2505", "usd2511"]\n'
                )
            arguments = _WHEEL._WheelArguments(
                root,
                root / "build",
                root / "dist",
                "release",
                (),
                "cmake",
                root / "wheel-build",
                variants=(("acme_schema", "usd2511"),),
            )

            variant = _WHEEL._select_wheel_variant(arguments, "acme_schema", project)
            with mock.patch.object(_WHEEL.subprocess, "run") as run:
                _WHEEL._run_install(arguments, "acme_schema", root / "stage", variant)

            self.assertEqual(variant, "usd2511")
            self.assertIn("acme_schema-python-usd2511", run.call_args.args[0])

    def test_strip_release_elf_files_uses_configured_toolchain(self) -> None:
        """Strip ELF payloads in release stages while leaving ordinary data alone."""
        with tempfile.TemporaryDirectory() as temporary_directory:
            root = Path(temporary_directory)
            build_directory = root / "build"
            build_directory.mkdir()
            strip = root / "toolchain" / "strip"
            strip.parent.mkdir()
            strip.touch()
            (build_directory / "CMakeCache.txt").write_text(f"CMAKE_STRIP:FILEPATH={strip}\n", encoding="utf-8")
            stage = root / "stage"
            stage.mkdir()
            elf = stage / "module.so"
            elf.write_bytes(b"\x7fELFpayload")
            (stage / "module.py").write_text("value = 1\n", encoding="utf-8")
            arguments = _WHEEL._WheelArguments(
                root,
                build_directory,
                root / "dist",
                "release",
                (),
                "cmake",
                root / "wheel-build",
            )

            with mock.patch.object(_WHEEL.subprocess, "run") as run:
                stripped = _WHEEL._strip_release_elf_files(arguments, stage)

            self.assertEqual(stripped, (elf,))
            run.assert_called_once_with([str(strip), "--strip-unneeded", str(elf)], check=True)

    def test_strip_release_elf_files_preserves_debug_stage(self) -> None:
        """Keep ELF symbols when assembling a debug wheel."""
        with tempfile.TemporaryDirectory() as temporary_directory:
            root = Path(temporary_directory)
            stage = root / "stage"
            stage.mkdir()
            (stage / "module.so").write_bytes(b"\x7fELFpayload")
            arguments = _WHEEL._WheelArguments(
                root,
                root / "build",
                root / "dist",
                "debug",
                (),
                "cmake",
                root / "wheel-build",
            )

            with mock.patch.object(_WHEEL.subprocess, "run") as run:
                stripped = _WHEEL._strip_release_elf_files(arguments, stage)

            self.assertEqual(stripped, ())
            run.assert_not_called()

    def test_select_groups_skips_native_only_packages_unless_requested(self) -> None:
        """Test select groups skips native only packages unless requested."""
        with tempfile.TemporaryDirectory() as temporary_directory:
            repo_root = Path(temporary_directory)
            build_directory = repo_root / "build"
            discovered_groups: dict[str, Path] = {}
            for group, imports in (
                ("acme_native", []),
                ("acme_python", ["acme.python"]),
            ):
                package_root = repo_root / "source" / "libraries" / Path(*group.split("_", 1))
                discovered_groups[group] = package_root
                manifest_path = build_directory / "packages" / group / "package.json"
                manifest_path.parent.mkdir(parents=True)
                manifest_path.write_text(
                    json.dumps(
                        {
                            "name": group,
                            "version": "1.2.3",
                            "complete": True,
                            "modules": [f"acme.{group.removeprefix('acme_')}"],
                            "dependencies": {},
                            "python_imports": imports,
                        }
                    ),
                    encoding="utf-8",
                )
            build_dependencies = repo_root / "wheel-build"
            arguments = _WHEEL._WheelArguments(
                repo_root,
                build_directory,
                repo_root / "dist",
                "release",
                (),
                "cmake",
                build_dependencies,
            )

            groups = _WHEEL._select_groups(arguments, discovered_groups)
            self.assertEqual(groups, ("acme_python",))

            explicit = _WHEEL._WheelArguments(
                repo_root,
                build_directory,
                repo_root / "dist",
                "release",
                ("acme_native",),
                "cmake",
                build_dependencies,
            )
            with self.assertRaisesRegex(RuntimeError, "does not produce a wheel"):
                _WHEEL._select_groups(explicit, discovered_groups)

    def test_validate_wheel_rejects_tests(self) -> None:
        """Test validate wheel rejects tests."""
        with tempfile.TemporaryDirectory() as temporary_directory:
            wheel_path = Path(temporary_directory) / "acme_robotics-1.2.3-cp312-abi3-any.whl"
            _write_test_wheel(wheel_path, "acme_robotics", "1.2.3", "acme.robotics")
            with zipfile.ZipFile(wheel_path, "a") as archive:
                archive.writestr("acme/robotics/tests/test_api.py", "")
            manifest = {
                "name": "acme_robotics",
                "version": "1.2.3",
                "complete": True,
                "dependencies": {},
                "python_imports": ["acme.robotics"],
            }

            with self.assertRaisesRegex(RuntimeError, "forbidden"):
                _WHEEL._validate_wheel(wheel_path, "acme_robotics", manifest)

    def test_validate_wheel_requires_nonempty_license_aggregate(self) -> None:
        """Reject a wheel that omits or empties its redistributed dependency notices."""
        manifest = {
            "name": "acme_robotics",
            "version": "1.2.3",
            "complete": True,
            "dependencies": {},
            "python_imports": ["acme.robotics"],
        }
        with tempfile.TemporaryDirectory() as temporary_directory:
            root = Path(temporary_directory)
            missing = root / "acme_robotics-1.2.3-cp312-abi3-any.whl"
            _write_test_wheel(missing, "acme_robotics", "1.2.3", "acme.robotics", license_text=None)
            with self.assertRaisesRegex(RuntimeError, "must contain exactly one.*PIP-LICENSES"):
                _WHEEL._validate_wheel(missing, "acme_robotics", manifest)

            empty = root / "acme_robotics-1.2.3-cp312-abi3-empty.whl"
            _write_test_wheel(empty, "acme_robotics", "1.2.3", "acme.robotics", license_text="")
            with self.assertRaisesRegex(RuntimeError, "empty license aggregate"):
                _WHEEL._validate_wheel(empty, "acme_robotics", manifest)

    def test_validate_wheel_accepts_exact_internal_dependency(self) -> None:
        """Test validate wheel accepts exact internal dependency."""
        with tempfile.TemporaryDirectory() as temporary_directory:
            wheel_path = Path(temporary_directory) / "acme_robotics-1.2.3-cp312-abi3-any.whl"
            dependencies = {"acme_core": "==1.2.3"}
            _write_test_wheel(
                wheel_path,
                "acme_robotics",
                "1.2.3",
                "acme.robotics",
                dependencies,
                ("acme-core==1.2.3",),
            )
            manifest = {
                "name": "acme_robotics",
                "version": "1.2.3",
                "complete": True,
                "dependencies": dependencies,
                "python_imports": ["acme.robotics"],
            }

            _WHEEL._validate_wheel(wheel_path, "acme_robotics", manifest)

    def test_validate_wheel_accepts_legacy_import_owned_by_compatibility_distribution(self) -> None:
        """Test validate wheel accepts legacy import owned by compatibility distribution."""
        with tempfile.TemporaryDirectory() as temporary_directory:
            wheel_path = Path(temporary_directory) / "acme_deprecated-1.2.3-cp312-abi3-any.whl"
            _write_test_wheel(wheel_path, "acme_deprecated", "1.2.3", "acme.core.legacy")
            manifest = {
                "name": "acme_deprecated",
                "version": "1.2.3",
                "complete": True,
                "dependencies": {},
                "python_imports": ["acme.core.legacy"],
            }

            _WHEEL._validate_wheel(wheel_path, "acme_deprecated", manifest)

    def test_validate_wheel_accepts_omitted_native_only_dependency(self) -> None:
        """Test validate wheel accepts omitted native only dependency."""
        with tempfile.TemporaryDirectory() as temporary_directory:
            wheel_path = Path(temporary_directory) / "acme_robotics-1.2.3-cp312-abi3-any.whl"
            dependencies = {"acme_core": "==1.2.3"}
            _write_test_wheel(
                wheel_path,
                "acme_robotics",
                "1.2.3",
                "acme.robotics",
                dependencies,
            )
            manifest = {
                "name": "acme_robotics",
                "version": "1.2.3",
                "complete": True,
                "dependencies": dependencies,
                "python_imports": ["acme.robotics"],
            }

            _WHEEL._validate_wheel(wheel_path, "acme_robotics", manifest)

    def test_validate_wheel_rejects_different_internal_dependency_version(self) -> None:
        """Test validate wheel rejects different internal dependency version."""
        with tempfile.TemporaryDirectory() as temporary_directory:
            wheel_path = Path(temporary_directory) / "acme_robotics-1.2.3-cp312-abi3-any.whl"
            dependencies = {"acme_core": "==1.2.3"}
            _write_test_wheel(
                wheel_path,
                "acme_robotics",
                "1.2.3",
                "acme.robotics",
                dependencies,
                ("acme-core==1.2.2",),
            )
            manifest = {
                "name": "acme_robotics",
                "version": "1.2.3",
                "complete": True,
                "dependencies": dependencies,
                "python_imports": ["acme.robotics"],
            }

            with self.assertRaisesRegex(RuntimeError, "must require acme_core==1.2.3"):
                _WHEEL._validate_wheel(wheel_path, "acme_robotics", manifest)

    def test_validate_wheel_rejects_ranged_internal_dependency_manifest(self) -> None:
        """Test validate wheel rejects ranged internal dependency manifest."""
        with tempfile.TemporaryDirectory() as temporary_directory:
            wheel_path = Path(temporary_directory) / "acme_robotics-1.2.3-cp312-abi3-any.whl"
            dependencies = {"acme_core": ">=1.2,<2"}
            _write_test_wheel(
                wheel_path,
                "acme_robotics",
                "1.2.3",
                "acme.robotics",
                dependencies,
                ("acme-core>=1.2,<2",),
            )
            manifest = {
                "name": "acme_robotics",
                "version": "1.2.3",
                "complete": True,
                "dependencies": dependencies,
                "python_imports": ["acme.robotics"],
            }

            with self.assertRaisesRegex(RuntimeError, "must require acme_core==1.2.3"):
                _WHEEL._validate_wheel(wheel_path, "acme_robotics", manifest)

    def test_validate_wheel_rejects_conditional_internal_dependency(self) -> None:
        """Test validate wheel rejects conditional internal dependency."""
        with tempfile.TemporaryDirectory() as temporary_directory:
            wheel_path = Path(temporary_directory) / "acme_robotics-1.2.3-cp312-abi3-any.whl"
            dependencies = {"acme_core": "==1.2.3"}
            _write_test_wheel(
                wheel_path,
                "acme_robotics",
                "1.2.3",
                "acme.robotics",
                dependencies,
                ("acme-core==1.2.3; sys_platform == 'linux'",),
            )
            manifest = {
                "name": "acme_robotics",
                "version": "1.2.3",
                "complete": True,
                "dependencies": dependencies,
                "python_imports": ["acme.robotics"],
            }

            with self.assertRaisesRegex(RuntimeError, "must be unconditional"):
                _WHEEL._validate_wheel(wheel_path, "acme_robotics", manifest)

    def test_validate_wheel_rejects_undeclared_internal_requirement(self) -> None:
        """Test validate wheel rejects undeclared internal requirement."""
        with tempfile.TemporaryDirectory() as temporary_directory:
            wheel_path = Path(temporary_directory) / "acme_robotics-1.2.3-cp312-abi3-any.whl"
            _write_test_wheel(
                wheel_path,
                "acme_robotics",
                "1.2.3",
                "acme.robotics",
                requirements=("acme-core==1.2.3",),
            )
            manifest = {
                "name": "acme_robotics",
                "version": "1.2.3",
                "complete": True,
                "dependencies": {},
                "python_imports": ["acme.robotics"],
            }

            with self.assertRaisesRegex(RuntimeError, "native package manifest does not"):
                _WHEEL._validate_wheel(
                    wheel_path,
                    "acme_robotics",
                    manifest,
                    frozenset(("acme_core", "acme_robotics")),
                )

    def test_assemble_group_uses_temporary_stage_and_publishes_one_wheel(self) -> None:
        """Test assemble group uses temporary stage and publishes one wheel."""
        with tempfile.TemporaryDirectory() as temporary_directory:
            repo_root = Path(temporary_directory)
            build_dependencies = repo_root / "wheel-build"
            build_dependencies.mkdir()
            output_directory = repo_root / "dist"
            project_directory = repo_root / "source" / "libraries" / "acme" / "robotics"
            _write_pyproject(project_directory / "pyproject.toml", "acme-robotics")
            arguments = _WHEEL._WheelArguments(
                repo_root,
                repo_root / "build",
                output_directory,
                "release",
                (),
                "cmake",
                build_dependencies,
            )

            def install_component(
                _arguments: Any,
                group: str,
                stage: Path,
                _variant: str | None = None,
            ) -> None:
                (stage / "python" / "acme" / "robotics").mkdir(parents=True)
                (stage / "share" / "isaacsim" / "packages" / group).mkdir(parents=True)
                manifest = {
                    "name": group,
                    "version": "1.2.3",
                    "complete": True,
                    "modules": ["acme.robotics"],
                    "dependencies": {},
                    "python_imports": ["acme.robotics"],
                }
                (stage / "share" / "isaacsim" / "packages" / group / "package.json").write_text(
                    json.dumps(manifest), encoding="utf-8"
                )
                license_path = stage / "share" / "licenses" / group / "isaacsim-libraries-PIP-LICENSES.txt"
                license_path.parent.mkdir(parents=True)
                license_path.write_text("acme dependency license\n", encoding="utf-8")

            def build_wheel(command: list[str], *, check: bool, env: dict[str, str]) -> None:
                self.assertTrue(check)
                self.assertEqual(env["SKBUILD_WHEEL_CMAKE"], "false")
                output = Path(command[command.index("--outdir") + 1])
                _write_test_wheel(
                    output / "acme_robotics-1.2.3-cp312-abi3-any.whl",
                    "acme_robotics",
                    "1.2.3",
                    "acme.robotics",
                    modules=("acme.robotics",),
                )

            with (
                mock.patch.object(_WHEEL, "_run_install", side_effect=install_component),
                mock.patch.object(_WHEEL.subprocess, "run", side_effect=build_wheel),
            ):
                wheel = _WHEEL._assemble_group(arguments, "acme_robotics", project_directory)

            self.assertEqual(wheel, output_directory / "acme_robotics-1.2.3-cp312-abi3-any.whl")
            self.assertTrue(wheel.is_file())
            self.assertEqual(list((repo_root / "_cmake_build" / "isaacsim-libraries-wheel-stage").iterdir()), [])

    def test_assemble_group_removes_temporary_stage_after_failure(self) -> None:
        """Test assemble group removes temporary stage after failure."""
        with tempfile.TemporaryDirectory() as temporary_directory:
            repo_root = Path(temporary_directory)
            project_directory = repo_root / "source" / "libraries" / "acme" / "robotics"
            _write_pyproject(project_directory / "pyproject.toml", "acme-robotics")
            arguments = _WHEEL._WheelArguments(
                repo_root,
                repo_root / "build",
                repo_root / "dist",
                "release",
                (),
                "cmake",
                repo_root / "wheel-build",
            )

            with (
                mock.patch.object(_WHEEL, "_run_install", side_effect=RuntimeError("install failed")),
                self.assertRaisesRegex(RuntimeError, "install failed"),
            ):
                _WHEEL._assemble_group(arguments, "acme_robotics", project_directory)

            self.assertEqual(list((repo_root / "_cmake_build" / "isaacsim-libraries-wheel-stage").iterdir()), [])


if __name__ == "__main__":
    unittest.main()
