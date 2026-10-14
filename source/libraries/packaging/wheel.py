#!/usr/bin/env python3
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

"""Assemble module wheels from an existing CMake build."""

from __future__ import annotations

import argparse
import email.parser
import importlib.util
import json
import os
import re
import shutil
import stat
import subprocess
import sys
import tempfile
import tomllib
import uuid
import zipfile
from collections import defaultdict
from dataclasses import dataclass
from pathlib import Path, PurePosixPath

from packaging.requirements import InvalidRequirement, Requirement

_MANIFEST_CONTRACT_PATH = Path(__file__).resolve().parents[1] / "package_manifest.py"
_MANIFEST_CONTRACT_SPEC = importlib.util.spec_from_file_location("isaacsim_package_manifest", _MANIFEST_CONTRACT_PATH)
if _MANIFEST_CONTRACT_SPEC is None or _MANIFEST_CONTRACT_SPEC.loader is None:
    raise RuntimeError(f"Cannot load package manifest contract: {_MANIFEST_CONTRACT_PATH}")
_MANIFEST_CONTRACT = importlib.util.module_from_spec(_MANIFEST_CONTRACT_SPEC)
_MANIFEST_CONTRACT_SPEC.loader.exec_module(_MANIFEST_CONTRACT)

_DISTRIBUTION_NAME_PATTERN = re.compile(r"[a-z][a-z0-9_]*")
_PIP_LICENSE_FILENAME = "isaacsim-libraries-PIP-LICENSES.txt"
_WHEEL_SIZE_MANIFEST_FILENAME = "wheel_sizes.json"
_MEBIBYTE = 1024 * 1024


@dataclass(frozen=True, slots=True)
class _WheelArguments:
    """Validated wheel assembly arguments."""

    repo_root: Path
    build_directory: Path
    output_directory: Path
    configuration: str
    groups: tuple[str, ...]
    cmake: str
    build_dependencies: Path
    variants: tuple[tuple[str, str], ...] = ()


def _parse_arguments() -> _WheelArguments:
    """Parse wheel assembly arguments.

    Returns:
        The resulting value.
    """
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--repo-root", type=Path, required=True)
    parser.add_argument("--build-dir", type=Path, required=True)
    parser.add_argument("--output-dir", type=Path, required=True)
    parser.add_argument("-c", "--config", default="release", choices=("debug", "release"))
    parser.add_argument("--group", action="append", default=[])
    parser.add_argument("--cmake", default="cmake")
    parser.add_argument("--build-deps-dir", type=Path, required=True)
    parser.add_argument(
        "--variant",
        action="append",
        default=[],
        metavar="GROUP=VARIANT",
        help="Select a declared wheel payload variant for one distribution.",
    )
    parsed = parser.parse_args()
    variants: list[tuple[str, str]] = []
    for value in parsed.variant:
        group, separator, variant = value.partition("=")
        if (
            not separator
            or _DISTRIBUTION_NAME_PATTERN.fullmatch(group) is None
            or re.fullmatch(r"[a-z][a-z0-9]*", variant) is None
        ):
            parser.error(f"invalid --variant value: {value!r}; expected GROUP=VARIANT")
        if any(existing_group == group for existing_group, _ in variants):
            parser.error(f"duplicate --variant group: {group}")
        variants.append((group, variant))
    return _WheelArguments(
        repo_root=parsed.repo_root.resolve(),
        build_directory=parsed.build_dir.resolve(),
        output_directory=parsed.output_dir.resolve(),
        configuration=parsed.config,
        groups=tuple(parsed.group),
        cmake=parsed.cmake,
        build_dependencies=parsed.build_deps_dir.resolve(),
        variants=tuple(variants),
    )


def _canonicalize_distribution_name(name: str) -> str:
    """Return the normalized form of a Python distribution name.

    Args:
        name: Name of the item.

    Returns:
        The resulting value.
    """
    return re.sub(r"[-_.]+", "-", name).lower()


def _read_requirements(requirements: list[str]) -> dict[str, Requirement]:
    """Parse unconditional distribution requirements by normalized name.

    Args:
        requirements: Python requirement specifications.

    Returns:
        The resulting value.
    """
    parsed_requirements: dict[str, Requirement] = {}
    for value in requirements:
        try:
            requirement = Requirement(value)
        except InvalidRequirement as error:
            raise RuntimeError(f"Wheel contains an invalid requirement: {value}") from error
        name = _canonicalize_distribution_name(requirement.name)
        if name in parsed_requirements:
            raise RuntimeError(f"Wheel contains duplicate requirements for {requirement.name}")
        parsed_requirements[name] = requirement
    return parsed_requirements


def _discover_groups(repo_root: Path) -> dict[str, Path]:
    """Discover distribution groups from flat and two-component CMake package roots.

    The authoritative distribution name comes from each ``[project].name`` rather
    than the source directory, so schema distributions whose package boundary
    intentionally differs from their source path (for example
    ``isaacsim_usd_schemas/robot`` -> ``isaacsim-robot-schema``) resolve correctly.

    Args:
        repo_root: Repository root directory.

    Returns:
        The resulting value.
    """
    modules_root = repo_root / "source" / "libraries"
    groups: dict[str, Path] = {}
    normalized_groups: dict[str, str] = {}
    pyprojects = (
        *modules_root.glob("*/pyproject.toml"),
        *modules_root.glob("*/*/pyproject.toml"),
    )
    for pyproject in sorted(pyprojects):
        metadata = tomllib.loads(pyproject.read_text(encoding="utf-8"))
        project_name = metadata.get("project", {}).get("name")
        if not isinstance(project_name, str) or not project_name:
            raise RuntimeError(f"Missing [project].name in {pyproject}")

        package_root = pyproject.parent
        group = re.sub(r"[-.]+", "_", project_name).lower()
        if re.fullmatch(r"[a-z][a-z0-9_]*", group) is None:
            raise RuntimeError(f"Python project {project_name!r} in {pyproject} has an invalid distribution name")
        if group in groups:
            raise RuntimeError(f"Duplicate module package group: {group}")
        normalized_group = _canonicalize_distribution_name(group)
        if normalized_group in normalized_groups:
            raise RuntimeError(
                f"Duplicate normalized module package group: {normalized_groups[normalized_group]} and {group}"
            )
        groups[group] = package_root
        normalized_groups[normalized_group] = group
    return groups


def _read_cmake_cache(build_directory: Path) -> dict[str, str]:
    """Read scalar entries from a CMake cache.

    Args:
        build_directory: Directory containing the CMake build.

    Returns:
        The resulting value.
    """
    path = build_directory / "CMakeCache.txt"
    if not path.is_file():
        raise RuntimeError(f"CMake build is missing: {path}. Build the modules before assembling wheels.")

    entries: dict[str, str] = {}
    for line in path.read_text(encoding="utf-8").splitlines():
        if not line or line.startswith(("#", "//")) or "=" not in line:
            continue
        key_and_type, value = line.split("=", 1)
        key = key_and_type.split(":", 1)[0]
        entries[key] = value
    return entries


def _paths_match(left: str | Path, right: str | Path) -> bool:
    """Return whether two paths identify the same normalized location.

    Args:
        left: Left-hand version component.
        right: Right-hand version component.

    Returns:
        The resulting value.
    """
    return os.path.normcase(str(Path(left).resolve())) == os.path.normcase(str(Path(right).resolve()))


def _validate_cmake_build(arguments: _WheelArguments) -> None:
    """Validate that the selected CMake tree can provide Python components.

    Args:
        arguments: Parsed command-line arguments.
    """
    cache = _read_cmake_cache(arguments.build_directory)
    expected_source = arguments.repo_root / "source" / "libraries"
    checks = {
        "CMAKE_BUILD_TYPE": arguments.configuration.capitalize(),
        "ISAACSIM_ENABLE_PYTHON": "ON",
        "ISAACSIM_ENABLE_PYTHON_BINDINGS": "ON",
        "ISAACSIM_ENFORCE_COMPLETE_PACKAGES": "ON",
    }
    mismatches = [
        f"{key}={cache.get(key)!r}, expected {expected!r}"
        for key, expected in checks.items()
        if cache.get(key) != expected
    ]
    cmake_home_directory = cache.get("CMAKE_HOME_DIRECTORY")
    if cmake_home_directory is None or not _paths_match(cmake_home_directory, expected_source):
        mismatches.append(f"CMAKE_HOME_DIRECTORY={cmake_home_directory!r}, expected {str(expected_source)!r}")
    python_executable = cache.get("Python_EXECUTABLE")
    if python_executable is None or not _paths_match(python_executable, sys.executable):
        mismatches.append(f"Python_EXECUTABLE={python_executable!r}, expected {sys.executable!r}")
    if mismatches:
        details = "\n  ".join(mismatches)
        raise RuntimeError(f"CMake build is incompatible with wheel assembly:\n  {details}")


def _run_install(arguments: _WheelArguments, group: str, stage: Path, variant: str | None = None) -> None:
    """Install one package's Python component into a clean staging prefix.

    Args:
        arguments: Parsed command-line arguments.
        group: Wheel distribution group.
        stage: Stage used by the test.
        variant: Variant values.
    """
    component = f"{group}-python" if variant is None else f"{group}-python-{variant}"
    command = [
        arguments.cmake,
        "--install",
        str(arguments.build_directory),
        "--prefix",
        str(stage),
        "--component",
        component,
        "--config",
        arguments.configuration.capitalize(),
    ]
    subprocess.run(command, check=True)


def _strip_release_elf_files(arguments: _WheelArguments, stage: Path) -> tuple[Path, ...]:
    """Remove debug and unused symbols from staged release-wheel ELF files.

    Debug builds retain their symbols. The CMake-selected strip executable keeps
    cross-compiled wheel assembly on the configured toolchain rather than using
    an arbitrary host utility.

    Args:
        arguments: Parsed command-line arguments.
        stage: CMake installation stage to optimize.

    Returns:
        Files that were stripped.
    """
    if arguments.configuration != "release":
        return ()

    elf_files: list[Path] = []
    for path in sorted(stage.rglob("*")):
        if path.is_symlink() or not path.is_file():
            continue
        with path.open("rb") as stream:
            if stream.read(4) == b"\x7fELF":
                elf_files.append(path)
    if not elf_files:
        return ()

    strip_value = _read_cmake_cache(arguments.build_directory).get("CMAKE_STRIP")
    if not strip_value:
        raise RuntimeError("Release wheel assembly requires CMAKE_STRIP in the configured build")
    strip = Path(strip_value)
    if not strip.is_file():
        raise RuntimeError(f"Release wheel strip executable does not exist: {strip}")

    for path in elf_files:
        subprocess.run([str(strip), "--strip-unneeded", str(path)], check=True)
    return tuple(elf_files)


def _load_package_manifest(path: Path, group: str) -> dict[str, object]:
    """Load and validate a generated or installed package manifest.

    Args:
        path: Filesystem path to process.
        group: Wheel distribution group.

    Returns:
        The resulting value.
    """
    return _MANIFEST_CONTRACT.load_package_manifest(path, group)


def _select_groups(arguments: _WheelArguments, discovered_groups: dict[str, Path]) -> tuple[str, ...]:
    """Select requested packages that own Python import modules.

    Args:
        arguments: Parsed command-line arguments.
        discovered_groups: Wheel groups discovered from package metadata.

    Returns:
        The resulting value.
    """
    if not discovered_groups:
        raise RuntimeError("No module package pyproject.toml files were discovered")
    requested_groups = tuple(dict.fromkeys(arguments.groups)) if arguments.groups else tuple(discovered_groups)
    unknown_groups = sorted(set(requested_groups) - set(discovered_groups))
    if unknown_groups:
        raise RuntimeError(f"Unknown module package groups: {', '.join(unknown_groups)}")
    unused_variant_groups = sorted(set(dict(arguments.variants)) - set(requested_groups))
    if unused_variant_groups:
        raise RuntimeError(f"Wheel variants selected for unrequested groups: {', '.join(unused_variant_groups)}")

    selected_groups: list[str] = []
    for group in requested_groups:
        manifest_path = arguments.build_directory / "packages" / group / "package.json"
        manifest = _load_package_manifest(manifest_path, group)
        if manifest["python_imports"]:
            selected_groups.append(group)
        elif arguments.groups:
            raise RuntimeError(f"Module package {group} has no Python imports and does not produce a wheel")
    return tuple(selected_groups)


def _load_staged_manifest(stage: Path, group: str) -> dict[str, object]:
    """Load and validate the package manifest installed by CMake.

    Args:
        stage: Stage used by the test.
        group: Wheel distribution group.

    Returns:
        The resulting value.
    """
    path = stage / "share" / "isaacsim" / "packages" / group / "package.json"
    manifest = _load_package_manifest(path, group)
    if not (stage / "python").is_dir():
        raise RuntimeError(f"CMake Python component for {group} did not install a payload")
    license_path = stage / "share" / "licenses" / group / _PIP_LICENSE_FILENAME
    if not license_path.is_file() or not license_path.read_bytes().strip():
        raise RuntimeError(f"CMake Python component for {group} did not install its license aggregate: {license_path}")
    return manifest


def _build_environment(
    arguments: _WheelArguments,
    stage: Path,
    variant: str | None = None,
) -> dict[str, str]:
    """Create the isolated backend environment for a staged wheel payload.

    Args:
        arguments: Parsed command-line arguments.
        stage: Stage used by the test.
        variant: Variant values.

    Returns:
        The resulting value.
    """
    environment = os.environ.copy()
    backend_path = arguments.build_dependencies
    if not backend_path.is_dir():
        raise RuntimeError(f"Python wheel build dependencies are missing: {backend_path}")

    include_sources = (stage / "python", stage / "share")
    if any(";" in str(path) or "=" in str(path) for path in include_sources):
        raise RuntimeError("Wheel staging paths must not contain ';' or '=' characters")
    environment["SKBUILD_WHEEL_CMAKE"] = "false"
    environment["SKBUILD_WHEEL_FORCE_INCLUDE"] = ";".join(
        (
            f"{stage / 'python'}=.",
            f"{stage / 'share'}=${{SKBUILD_DATA_DIR}}/share",
        )
    )
    environment["PYTHONPATH"] = str(backend_path)
    environment["PYTHONDONTWRITEBYTECODE"] = "1"
    if variant is not None:
        environment["SKBUILD_WHEEL_BUILD_TAG"] = f"1{variant}"
    return environment


def _select_wheel_variant(arguments: _WheelArguments, group: str, project_directory: Path) -> str | None:
    """Select and validate a package's optional wheel payload variant.

    Args:
        arguments: Parsed command-line arguments.
        group: Wheel distribution group.
        project_directory: Python project directory.

    Returns:
        The resulting value.
    """
    metadata = tomllib.loads((project_directory / "pyproject.toml").read_text(encoding="utf-8"))
    tool = metadata.get("tool", {})
    isaacsim_library = tool.get("isaacsim-library", {}) if isinstance(tool, dict) else {}
    if not isinstance(isaacsim_library, dict):
        raise RuntimeError(f"[tool.isaacsim-library] must be a table in {project_directory / 'pyproject.toml'}")
    declared = isaacsim_library.get("wheel-variants", [])
    default = isaacsim_library.get("default-wheel-variant")
    if not isinstance(declared, list) or not all(
        isinstance(value, str) and re.fullmatch(r"[a-z][a-z0-9]*", value) for value in declared
    ):
        raise RuntimeError(f"Invalid wheel-variants in {project_directory / 'pyproject.toml'}")
    if default is not None and (not isinstance(default, str) or default not in declared):
        raise RuntimeError(f"Invalid default-wheel-variant in {project_directory / 'pyproject.toml'}")
    requested = dict(arguments.variants).get(group, default)
    if requested is not None and requested not in declared:
        raise RuntimeError(f"Wheel variant {requested!r} is not declared for {group}")
    return requested


def _validate_wheel(
    path: Path,
    group: str,
    manifest: dict[str, object],
    internal_groups: frozenset[str] = frozenset(),
    variant: str | None = None,
) -> None:
    """Validate wheel metadata and package contents.

    Args:
        path: Filesystem path to process.
        group: Wheel distribution group.
        manifest: Wheel manifest metadata.
        internal_groups: Internal wheel groups.
        variant: Variant values.
    """
    version = manifest.get("version")
    if variant is not None:
        expected_fragment = f"-{version}-1{variant}-py3-none-any.whl"
        if not path.name.endswith(expected_fragment):
            raise RuntimeError(f"Variant wheel must end with {expected_fragment!r}, found {path.name!r}")
    with zipfile.ZipFile(path) as wheel:
        entries = wheel.infolist()
        names = [entry.filename for entry in entries]
        forbidden_directory_names = {
            "__pycache__",
            ".pytest_cache",
            ".mypy_cache",
            ".ruff_cache",
            "tests",
        }
        forbidden = [
            name
            for name in names
            if forbidden_directory_names.intersection(PurePosixPath(name).parts) or name.endswith((".pyc", ".pyo"))
        ]
        symlinks = [entry.filename for entry in entries if stat.S_ISLNK(entry.external_attr >> 16)]
        if forbidden:
            raise RuntimeError(f"Wheel contains forbidden generated/test files: {forbidden[:5]}")
        if symlinks:
            raise RuntimeError(f"Wheel contains symbolic links: {symlinks[:5]}")

        manifest_suffix = f"share/isaacsim/packages/{group}/package.json"
        manifest_paths = [name for name in names if name.endswith(manifest_suffix)]
        if len(manifest_paths) != 1:
            raise RuntimeError(f"Wheel is missing {manifest_suffix}")
        wheel_manifest = json.loads(wheel.read(manifest_paths[0]))
        if wheel_manifest != manifest:
            raise RuntimeError("Wheel package manifest does not match the staged manifest")

        license_suffix = f"share/licenses/{group}/{_PIP_LICENSE_FILENAME}"
        license_paths = [name for name in names if name.endswith(license_suffix)]
        if len(license_paths) != 1:
            raise RuntimeError(f"Wheel must contain exactly one {license_suffix}, found {len(license_paths)}")
        if not wheel.read(license_paths[0]).strip():
            raise RuntimeError(f"Wheel contains an empty license aggregate: {license_paths[0]}")

        imports = manifest.get("python_imports", [])
        if not isinstance(imports, list):
            raise RuntimeError("Staged package manifest has invalid python_imports")
        for import_name in imports:
            if not isinstance(import_name, str):
                raise RuntimeError("Staged package manifest has a non-string Python import")
            import_path = import_name.replace(".", "/") + "/"
            if not any(name.startswith(import_path) for name in names):
                raise RuntimeError(f"Wheel is missing registered Python package {import_name}")
            if f"{import_path}py.typed" not in names:
                raise RuntimeError(f"Wheel is missing the typing marker for {import_name}")
            import_parts = import_name.split(".")
            namespace_initializers = [
                "/".join(import_parts[:depth]) + "/__init__.py" for depth in range(1, len(import_parts))
            ]
            unexpected_initializers = [name for name in namespace_initializers if name in names]
            if unexpected_initializers:
                raise RuntimeError(f"Wheel contains namespace package initializers: {unexpected_initializers}")

        metadata_paths = [name for name in names if name.endswith(".dist-info/METADATA")]
        if len(metadata_paths) != 1:
            raise RuntimeError(f"Wheel must contain exactly one METADATA file, found {len(metadata_paths)}")
        metadata = email.parser.BytesParser().parsebytes(wheel.read(metadata_paths[0]))
        if _canonicalize_distribution_name(metadata.get("Name", "")) != _canonicalize_distribution_name(group):
            raise RuntimeError(f"Wheel metadata has the wrong distribution name: {metadata.get('Name')!r}")
        if not isinstance(version, str) or metadata.get("Version") != version:
            raise RuntimeError(f"Wheel metadata version {metadata.get('Version')!r} does not match {version!r}")

        requirements = _read_requirements(metadata.get_all("Requires-Dist", []))
        dependencies = manifest["dependencies"]
        if not isinstance(dependencies, dict):
            raise RuntimeError("Staged package manifest has invalid dependencies")
        declared_dependency_names = {_canonicalize_distribution_name(name) for name in dependencies}
        for internal_group in internal_groups:
            normalized_group = _canonicalize_distribution_name(internal_group)
            if (
                normalized_group != _canonicalize_distribution_name(group)
                and normalized_group in requirements
                and normalized_group not in declared_dependency_names
            ):
                raise RuntimeError(
                    f"Wheel requires internal package {internal_group}, but the native package manifest does not"
                )
        for dependency_name, dependency_specifier in dependencies.items():
            if not isinstance(dependency_name, str) or not isinstance(dependency_specifier, str):
                raise RuntimeError("Staged package manifest has an invalid dependency")
            if dependency_specifier != f"=={version}":
                raise RuntimeError(f"Staged package manifest must require {dependency_name}=={version}")
            requirement = requirements.get(_canonicalize_distribution_name(dependency_name))
            if requirement is None:
                continue
            if requirement.extras or requirement.marker is not None or requirement.url is not None:
                raise RuntimeError(f"Wheel requirement for {dependency_name} must be unconditional")
            try:
                expected = Requirement(f"{dependency_name}{dependency_specifier}")
            except InvalidRequirement as error:
                raise RuntimeError(
                    f"Staged package manifest has an invalid dependency: {dependency_name}{dependency_specifier}"
                ) from error
            if requirement.specifier != expected.specifier:
                raise RuntimeError(
                    f"Wheel metadata must require {dependency_name}{expected.specifier}, "
                    f"found {requirement.name}{requirement.specifier}"
                )


def _publish_wheel(source: Path, output_directory: Path) -> Path:
    """Atomically publish a completed wheel to the requested directory.

    Args:
        source: Source path.
        output_directory: Directory where artifacts are written.

    Returns:
        The resulting value.
    """
    output_directory.mkdir(parents=True, exist_ok=True)
    destination = output_directory / source.name
    temporary = output_directory / f".{source.name}.{uuid.uuid4().hex}.tmp"
    try:
        shutil.copyfile(source, temporary)
        temporary.replace(destination)
    finally:
        temporary.unlink(missing_ok=True)
    return destination


def _build_wheel_size_manifest(output_directory: Path, configuration: str) -> dict[str, object]:
    """Build size metadata from the completed wheel artifacts.

    Args:
        output_directory: Directory containing wheel artifacts.
        configuration: Wheel build configuration.

    Returns:
        Versioned wheel size manifest.

    Raises:
        RuntimeError: If the output directory contains no wheels.
    """
    wheels = sorted(output_directory.glob("*.whl"))
    if not wheels:
        raise RuntimeError(f"No wheels found in {output_directory}")

    rows: list[dict[str, int | str]] = []
    content_groups: dict[tuple[int, int], list[int]] = defaultdict(list)
    for path in wheels:
        with zipfile.ZipFile(path) as wheel:
            entries = [entry for entry in wheel.infolist() if not entry.is_dir()]
        rows.append(
            {
                "filename": path.name,
                "size_bytes": path.stat().st_size,
                "installed_size_bytes": sum(entry.file_size for entry in entries),
                "file_count": len(entries),
            }
        )
        for entry in entries:
            content_groups[(entry.file_size, entry.CRC)].append(entry.compress_size)

    duplicate_compressed_size = sum(sum(sizes) - min(sizes) for sizes in content_groups.values() if len(sizes) > 1)
    return {
        "schema_version": 1,
        "configuration": configuration,
        "wheels": rows,
        "totals": {
            "size_bytes": sum(int(row["size_bytes"]) for row in rows),
            "installed_size_bytes": sum(int(row["installed_size_bytes"]) for row in rows),
            "file_count": sum(int(row["file_count"]) for row in rows),
            "duplicate_compressed_size_bytes": duplicate_compressed_size,
        },
    }


def _write_wheel_size_manifest(output_directory: Path, configuration: str) -> dict[str, object]:
    """Write wheel size metadata beside the artifacts.

    Args:
        output_directory: Directory containing wheel artifacts.
        configuration: Wheel build configuration.

    Returns:
        Written wheel size manifest.
    """
    manifest = _build_wheel_size_manifest(output_directory, configuration)
    destination = output_directory / _WHEEL_SIZE_MANIFEST_FILENAME
    temporary = destination.with_suffix(f"{destination.suffix}.tmp")
    temporary.write_text(json.dumps(manifest, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    temporary.replace(destination)
    return manifest


def _print_wheel_sizes(manifest: dict[str, object]) -> None:
    """Print the wheel size manifest in a human-readable table.

    Args:
        manifest: Wheel size manifest to print.
    """
    wheels = manifest["wheels"]
    totals = manifest["totals"]
    if not isinstance(wheels, list) or not isinstance(totals, dict):
        raise RuntimeError("Wheel size manifest has an invalid structure")
    print(f"{'Wheel':48} {'Compressed':>14} {'Installed':>14} {'Files':>7}")
    for row in sorted(wheels, key=lambda item: int(item["size_bytes"]), reverse=True):
        print(
            f"{str(row['filename']):48} "
            f"{int(row['size_bytes']) / _MEBIBYTE:>11.2f} MiB "
            f"{int(row['installed_size_bytes']) / _MEBIBYTE:>11.2f} MiB "
            f"{int(row['file_count']):>7}"
        )
    print(f"Total compressed: {int(totals['size_bytes']) / _MEBIBYTE:.2f} MiB")
    print(f"Total installed: {int(totals['installed_size_bytes']) / _MEBIBYTE:.2f} MiB")
    print("Duplicate compressed content: " f"{int(totals['duplicate_compressed_size_bytes']) / _MEBIBYTE:.2f} MiB")


def _assemble_group(
    arguments: _WheelArguments,
    group: str,
    project_directory: Path,
    internal_groups: frozenset[str] = frozenset(),
) -> Path:
    """Stage and assemble one package without compiling CMake targets.

    Args:
        arguments: Parsed command-line arguments.
        group: Wheel distribution group.
        project_directory: Python project directory.
        internal_groups: Internal wheel groups.

    Returns:
        The resulting value.
    """
    work_root = arguments.repo_root / "_cmake_build" / "isaacsim-libraries-wheel-stage"
    work_root.mkdir(parents=True, exist_ok=True)
    with tempfile.TemporaryDirectory(prefix=f"{group}-", dir=work_root) as temporary_directory:
        temporary_root = Path(temporary_directory)
        stage = temporary_root / "install"
        backend_output = temporary_root / "wheel"
        stage.mkdir()
        backend_output.mkdir()

        variant = _select_wheel_variant(arguments, group, project_directory)
        _run_install(arguments, group, stage, variant)
        _strip_release_elf_files(arguments, stage)
        manifest = _load_staged_manifest(stage, group)
        command = [
            sys.executable,
            "-s",
            "-m",
            "build",
            "--wheel",
            "--no-isolation",
            "--outdir",
            str(backend_output),
            str(project_directory),
        ]
        subprocess.run(command, check=True, env=_build_environment(arguments, stage, variant))
        wheels = list(backend_output.glob("*.whl"))
        if len(wheels) != 1:
            raise RuntimeError(f"Wheel backend produced {len(wheels)} wheels for {group}, expected one")
        _validate_wheel(wheels[0], group, manifest, internal_groups, variant)
        return _publish_wheel(wheels[0], arguments.output_directory)


def _main() -> int:
    """Assemble the requested wheels.

    Returns:
        The resulting value.
    """
    arguments = _parse_arguments()
    _validate_cmake_build(arguments)
    discovered_groups = _discover_groups(arguments.repo_root)
    groups = _select_groups(arguments, discovered_groups)
    internal_groups = frozenset(discovered_groups)
    for group in groups:
        print(_assemble_group(arguments, group, discovered_groups[group], internal_groups))
    manifest = _write_wheel_size_manifest(arguments.output_directory, arguments.configuration)
    _print_wheel_sizes(manifest)
    return 0


if __name__ == "__main__":
    raise SystemExit(_main())
