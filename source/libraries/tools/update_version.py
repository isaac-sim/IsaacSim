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

"""Update the shared standalone library version."""

from __future__ import annotations

import argparse
import re
import tomllib
from pathlib import Path

VERSION_PATTERN = re.compile(
    r"^(0|[1-9][0-9]*)\.(0|[1-9][0-9]*)\.(0|[1-9][0-9]*)"
    r"((a|b|rc)(0|[1-9][0-9]*))?(\.post(0|[1-9][0-9]*))?(\.dev(0|[1-9][0-9]*))?"
    r"(\+([0-9]*[a-z][a-z0-9]*|(0|[1-9][0-9]*))(\.([0-9]*[a-z][a-z0-9]*|(0|[1-9][0-9]*)))*)?$"
)
REQUIREMENT_NAME_PATTERN = re.compile(r"^(?P<name>[A-Za-z0-9_.-]+)")
SHARED_VERSION_PROVIDER_MODULE = "isaacsim_libraries_metadata"


class VersionError(RuntimeError):
    """Report invalid or inconsistent standalone library versions."""


def _normalize_distribution_name(name: str) -> str:
    """Normalize a Python distribution name for identity comparisons.

    Args:
        name: Name of the item.

    Returns:
        The resulting value.
    """
    return re.sub(r"[-_.]+", "-", name).lower()


def _find_pyprojects(libraries_root: Path) -> tuple[Path, ...]:
    """Find every standalone distribution project in deterministic order.

    Args:
        libraries_root: Root directory containing the libraries.

    Returns:
        The resulting value.
    """
    return tuple(sorted(libraries_root.glob("**/pyproject.toml")))


def _read_pyproject(path: Path) -> dict:
    """Read one Python project document.

    Args:
        path: Python project file to read.

    Returns:
        Parsed TOML document.
    """
    try:
        with path.open("rb") as stream:
            document = tomllib.load(stream)
    except (OSError, tomllib.TOMLDecodeError) as error:
        raise VersionError(f"Cannot read Python project metadata from {path}: {error}") from error
    if not isinstance(document, dict):
        raise VersionError(f"Python project metadata must be a table: {path}")
    return document


def _read_project(path: Path) -> dict:
    """Read one Python project table.

    Args:
        path: Filesystem path to process.

    Returns:
        The resulting value.
    """
    try:
        project = _read_pyproject(path)["project"]
    except KeyError as error:
        raise VersionError(f"Cannot read Python project metadata from {path}: missing project table") from error
    if not isinstance(project, dict):
        raise VersionError(f"Python project metadata must be a table: {path}")
    return project


def _read_version(libraries_root: Path) -> str:
    """Read and validate the authoritative standalone library version.

    Args:
        libraries_root: Root directory containing the libraries.

    Returns:
        The resulting value.
    """
    version_path = libraries_root / "VERSION"
    try:
        version = version_path.read_text(encoding="utf-8").strip()
    except OSError as error:
        raise VersionError(f"Cannot read {version_path}: {error}") from error
    if VERSION_PATTERN.fullmatch(version) is None:
        raise VersionError(f"Invalid canonical PEP 440 standalone library version: {version}")
    return version


def _read_distribution_names(pyprojects: tuple[Path, ...]) -> set[str]:
    """Read the normalized names of every standalone distribution.

    Args:
        pyprojects: Discovered Python project files.

    Returns:
        The resulting value.
    """
    names: set[str] = set()
    for path in pyprojects:
        name = _read_project(path).get("name")
        if not isinstance(name, str) or not name:
            raise VersionError(f"Python project name must be a non-empty string: {path}")
        normalized_name = _normalize_distribution_name(name)
        if normalized_name in names:
            raise VersionError(f"Duplicate standalone distribution name: {name}")
        names.add(normalized_name)
    return names


def _read_dynamic_internal_dependencies(path: Path, libraries_root: Path) -> tuple[str, ...]:
    """Read internal dependency names supplied by the shared-version provider.

    Args:
        path: Python project file to inspect.
        libraries_root: Root directory containing the libraries.

    Returns:
        Internal distribution names declared by the provider.
    """
    document = _read_pyproject(path)
    project = document.get("project")
    tool = document.get("tool", {})
    entries = tool.get("dynamic-metadata", []) if isinstance(tool, dict) else []
    if not isinstance(project, dict) or not isinstance(entries, list):
        raise VersionError(f"Invalid dynamic Python project metadata: {path}")
    names: list[str] = []
    for entry in entries:
        if not isinstance(entry, dict):
            raise VersionError(f"Dynamic metadata entries must be tables: {path}")
        provider = entry.get("provider")
        if not isinstance(provider, dict) or provider.get("module") != SHARED_VERSION_PROVIDER_MODULE:
            continue
        provider_path = provider.get("path")
        expected_path = (libraries_root / "packaging").resolve()
        if not isinstance(provider_path, str) or (path.parent / provider_path).resolve() != expected_path:
            raise VersionError(f"Shared-version metadata provider must use {expected_path}: {path}")
        if entry.get("field") != "dependencies":
            raise VersionError(f"Shared-version metadata provider must target dependencies: {path}")
        entry_names = entry.get("names")
        if (
            not isinstance(entry_names, list)
            or not entry_names
            or not all(isinstance(name, str) for name in entry_names)
        ):
            raise VersionError(f"Shared-version metadata provider names must be nonempty strings: {path}")
        names.extend(entry_names)
    dynamic_fields = project.get("dynamic", [])
    if names and (not isinstance(dynamic_fields, list) or "dependencies" not in dynamic_fields):
        raise VersionError(f"Shared-version dependencies must be listed as dynamic: {path}")
    normalized_names = [_normalize_distribution_name(name) for name in names]
    if len(normalized_names) != len(set(normalized_names)):
        raise VersionError(f"Duplicate shared-version dependency in {path}")
    return tuple(names)


def check_version_sync(libraries_root: Path) -> str:
    """Require every internal Python dependency to use shared-version metadata.

    Args:
        libraries_root: Root directory containing the libraries.

    Returns:
        The resulting value.
    """
    version = _read_version(libraries_root)
    pyprojects = _find_pyprojects(libraries_root)
    distribution_names = _read_distribution_names(pyprojects)
    for path in pyprojects:
        project = _read_project(path)
        project_name = project.get("name")
        dependencies = project.get("dependencies", [])
        if not isinstance(dependencies, list) or not all(isinstance(item, str) for item in dependencies):
            raise VersionError(f"Python project dependencies must be strings: {path}")
        for dependency in dependencies:
            match = REQUIREMENT_NAME_PATTERN.match(dependency)
            if match is None:
                continue
            if _normalize_distribution_name(match.group("name")) in distribution_names:
                raise VersionError(
                    f"Internal dependency {dependency} in {path} must use the shared-version metadata provider"
                )
        for dependency_name in _read_dynamic_internal_dependencies(path, libraries_root):
            normalized_name = _normalize_distribution_name(dependency_name)
            if normalized_name not in distribution_names:
                raise VersionError(f"Unknown internal dependency {dependency_name} in {path}")
            if isinstance(project_name, str) and normalized_name == _normalize_distribution_name(project_name):
                raise VersionError(f"Python project cannot depend on itself through shared-version metadata: {path}")
    return version


def update_version(libraries_root: Path, version: str) -> None:
    """Write a new shared version used by package and dependency metadata.

    Args:
        libraries_root: Root directory containing the libraries.
        version: Package version.
    """
    if VERSION_PATTERN.fullmatch(version) is None:
        raise VersionError(f"Invalid canonical PEP 440 standalone library version: {version}")
    (libraries_root / "VERSION").write_text(f"{version}\n", encoding="utf-8")
    check_version_sync(libraries_root)


def main() -> int:
    """Run the standalone version update or consistency check.

    Returns:
        The resulting value.
    """
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("version", nargs="?", help="New canonical PEP 440 version")
    parser.add_argument("--check", action="store_true", help="Check the version and internal dependency metadata")
    arguments = parser.parse_args()
    if arguments.check:
        if arguments.version is not None:
            parser.error("version cannot be combined with --check")
        print(f"Standalone library version is synchronized: {check_version_sync(Path(__file__).parents[1])}")
        return 0
    if arguments.version is None:
        parser.error("version is required unless --check is used")
    update_version(Path(__file__).parents[1], arguments.version)
    print(f"Updated standalone library version to {arguments.version}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
