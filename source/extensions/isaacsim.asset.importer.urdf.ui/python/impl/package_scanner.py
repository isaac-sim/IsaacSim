# SPDX-FileCopyrightText: Copyright (c) 2023-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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

"""Light URDF scanner that extracts package:// references and resolves paths."""

from __future__ import annotations

import pathlib
import re

import carb

_PACKAGE_URI_RE = re.compile(r'package://([^/"\s]+)(/[^"\s]*)?')

_MAX_PARENT_LEVELS = 10


def scan_urdf_packages(urdf_path: str) -> list[tuple[str, str]]:
    """Scan a URDF file for ``package://`` URIs and attempt to resolve each package root.

    Resolution order per package name:

    1. ``ament_index_python`` (ROS 2) -- if importable.
    2. Directory walk -- walk up from the URDF directory checking whether
       ``{dir}/{relative_path}`` exists for any mesh/texture reference.
    3. Meshes-folder heuristic -- look for a ``meshes/`` subfolder near the URDF.

    Args:
        urdf_path: Absolute path to the URDF file.

    Returns:
        List of ``(package_name, resolved_path)`` tuples.  ``resolved_path`` is
        an empty string when no location could be determined.

    """
    path = pathlib.Path(urdf_path)
    if not path.is_file():
        return []

    try:
        content = path.read_text(encoding="utf-8", errors="replace")
    except OSError:
        carb.log_warn(f"package_scanner: unable to read {urdf_path}")
        return []

    matches = _PACKAGE_URI_RE.findall(content)
    if not matches:
        return []

    packages: dict[str, list[str]] = {}
    for pkg_name, rel_path in matches:
        if pkg_name not in packages:
            packages[pkg_name] = []
        if rel_path:
            cleaned = rel_path.lstrip("/")
            if cleaned and cleaned not in packages[pkg_name]:
                packages[pkg_name].append(cleaned)

    urdf_dir = path.parent

    result: list[tuple[str, str]] = []
    for pkg_name, rel_paths in packages.items():
        resolved = (
            _try_ament_resolve(pkg_name, rel_paths)
            or _try_directory_walk(urdf_dir, rel_paths)
            or _try_meshes_folder(urdf_dir, pkg_name, rel_paths)
        )
        result.append((pkg_name, str(resolved) if resolved else ""))

    return result


def _any_file_exists(base: pathlib.Path, rel_paths: list[str]) -> bool:
    """Return True if at least one *rel_paths* entry resolves to an existing file under *base*.

    When *rel_paths* is empty the check cannot be performed and True is returned
    so callers degrade gracefully.

    Args:
        base: Directory to resolve each relative path against.
        rel_paths: Relative file paths to test under *base*.

    Returns:
        True if *rel_paths* is empty or any ``base / rel`` exists as a file.
    """
    if not rel_paths:
        return True
    return any((base / rel).exists() for rel in rel_paths)


def _try_ament_resolve(pkg_name: str, rel_paths: list[str]) -> str | None:
    """Attempt to resolve a ROS 2 package via ``ament_index_python``.

    The resolved share directory is only accepted when at least one of the
    URDF-referenced files is actually present under it.

    Args:
        pkg_name: ROS 2 package name from a ``package://`` URI.
        rel_paths: Relative paths referenced by the URDF for that package.

    Returns:
        Absolute path to the package share directory, or None if unavailable
        or no referenced file exists there.
    """
    try:
        from ament_index_python.packages import get_package_share_directory  # type: ignore[import-untyped]

        share_dir = pathlib.Path(get_package_share_directory(pkg_name))
        if share_dir.is_dir() and _any_file_exists(share_dir, rel_paths):
            return str(share_dir)
    except Exception:
        pass
    return None


def _try_directory_walk(urdf_dir: pathlib.Path, rel_paths: list[str]) -> str | None:
    """Walk up from *urdf_dir* looking for a parent where any *rel_paths* entry exists.

    Args:
        urdf_dir: Directory containing the URDF file (starting search point).
        rel_paths: Relative paths from the URDF that should exist under a candidate root.

    Returns:
        Absolute path to the first ancestor directory where a referenced file
        exists, or None if none found within the walk limit.
    """
    if not rel_paths:
        return None

    current = urdf_dir
    for _ in range(_MAX_PARENT_LEVELS):
        for rel in rel_paths:
            if (current / rel).exists():
                return str(current)
        parent = current.parent
        if parent == current:
            break
        current = parent
    return None


def _try_meshes_folder(urdf_dir: pathlib.Path, pkg_name: str, rel_paths: list[str]) -> str | None:
    """Heuristic: walk up looking for a directory that resolves URDF-referenced files.

    Checks each ancestor (and its ``{pkg_name}/`` subdirectory) as a
    candidate package root.  A candidate is accepted when at least one
    of the URDF-referenced relative paths resolves to an existing file.

    Args:
        urdf_dir: Directory containing the URDF file (starting search point).
        pkg_name: ROS package name; used to test a ``{pkg_name}/`` subfolder.
        rel_paths: Relative paths from the URDF that should exist under a candidate root.

    Returns:
        Absolute path to an accepted candidate directory, or None if none found.
    """
    if not rel_paths:
        return None

    current = urdf_dir
    for _ in range(_MAX_PARENT_LEVELS):
        if _any_file_exists(current, rel_paths):
            return str(current)

        pkg_sub = current / pkg_name
        if pkg_sub.is_dir() and _any_file_exists(pkg_sub, rel_paths):
            return str(pkg_sub)

        parent = current.parent
        if parent == current:
            break
        current = parent
    return None
