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

"""Install and validate a completed source-library wheelhouse."""

from __future__ import annotations

import argparse
import importlib.metadata
import json
import os
import re
import subprocess
import sys
import tempfile
import time
from pathlib import Path

_IMPORT_MODULE_CODE = """\
import importlib
import sys

sys.path.insert(0, sys.argv[1])
importlib.import_module(sys.argv[2])
"""
_PYPI_INDEX_URL = "https://pypi.org/simple"
_NVIDIA_PYPI_INDEX_URL = "https://pypi.nvidia.com"


def _parse_arguments() -> argparse.Namespace:
    """Parse command-line arguments.

    Returns:
        Parsed arguments.
    """
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("wheel_directory", type=Path, help="Directory containing wheel artifacts.")
    parser.add_argument(
        "--metrics-output",
        type=Path,
        required=True,
        help="Path for install metrics JSON.",
    )
    return parser.parse_args()


def _directory_size(path: Path) -> tuple[int, int]:
    """Measure regular files in a directory tree.

    Args:
        path: Directory to measure.

    Returns:
        Total file size and file count.
    """
    files = [candidate for candidate in path.rglob("*") if candidate.is_file() and not candidate.is_symlink()]
    return sum(candidate.stat().st_size for candidate in files), len(files)


def _canonical_distribution_name(name: str) -> str:
    """Normalize a Python distribution name for comparison.

    Args:
        name: Distribution name.

    Returns:
        PEP 503-style normalized distribution name.
    """
    return re.sub(r"[-_.]+", "-", name).lower()


def _wheel_distribution_name(wheel: Path) -> str:
    """Extract the normalized distribution name encoded in a wheel filename.

    Args:
        wheel: Wheel artifact path.

    Returns:
        Normalized distribution name.
    """
    return _canonical_distribution_name(wheel.name.split("-", 1)[0])


def _distribution_size(install_directory: Path, name: str) -> tuple[int, int]:
    """Measure files owned by one installed distribution.

    Args:
        install_directory: Clean installation root.
        name: Normalized distribution name.

    Returns:
        Installed file size and file count.

    Raises:
        RuntimeError: If the requested distribution or its file manifest is missing.
    """
    matches = [
        distribution
        for distribution in importlib.metadata.distributions(path=[str(install_directory)])
        if _canonical_distribution_name(distribution.metadata["Name"] or "") == name
    ]
    if len(matches) != 1:
        raise RuntimeError(f"Expected one installed {name} distribution, found {len(matches)}")
    files = matches[0].files
    if files is None:
        raise RuntimeError(f"Installed distribution {name} has no file manifest")

    root = install_directory.resolve()
    owned_files: set[Path] = set()
    for relative_path in files:
        path = Path(matches[0].locate_file(relative_path)).resolve()
        if not path.is_file() or path.is_symlink():
            continue
        try:
            path.relative_to(root)
        except ValueError as error:
            raise RuntimeError(f"Installed distribution {name} owns a file outside the target: {path}") from error
        owned_files.add(path)
    return sum(path.stat().st_size for path in owned_files), len(owned_files)


def _distribution_count(install_directory: Path) -> int:
    """Count distributions installed into a clean target.

    Args:
        install_directory: Clean installation root.

    Returns:
        Installed distribution count.
    """
    return sum(1 for _ in importlib.metadata.distributions(path=[str(install_directory)]))


def _measure_installed_distributions(
    install_directory: Path, root_distribution_names: set[str]
) -> list[dict[str, object]]:
    """Measure installed dependency distributions.

    Args:
        install_directory: Clean installation root.
        root_distribution_names: Normalized names of the explicitly installed wheels.

    Returns:
        Dependency distribution names, versions, installed sizes, and file counts.
    """
    rows = []
    for distribution in importlib.metadata.distributions(path=[str(install_directory)]):
        name = distribution.metadata["Name"] or ""
        normalized_name = _canonical_distribution_name(name)
        if not normalized_name or normalized_name in root_distribution_names:
            continue
        size, file_count = _distribution_size(install_directory, normalized_name)
        rows.append(
            {
                "name": name,
                "version": distribution.version,
                "installed_size_bytes": size,
                "installed_file_count": file_count,
            }
        )
    return sorted(rows, key=lambda row: (-int(row["installed_size_bytes"]), str(row["name"]).lower()))


def _download_wheel_dependencies(
    wheel_directory: Path,
    download_directory: Path,
    wheels: list[Path],
    cache_directory: Path,
    *,
    use_indexes: bool = True,
) -> float:
    """Download a resolved wheelhouse for the requested wheels.

    Args:
        wheel_directory: Local wheelhouse used to resolve source-library dependencies.
        download_directory: Destination for the resolved wheel archives.
        wheels: Explicit root wheels whose dependencies should be downloaded.
        cache_directory: Shared pip download cache.
        use_indexes: Whether to resolve dependencies from configured network sources.

    Returns:
        Dependency resolution and download duration in seconds.
    """
    if use_indexes:
        index_arguments = [
            "--index-url",
            _PYPI_INDEX_URL,
            "--extra-index-url",
            _NVIDIA_PYPI_INDEX_URL,
        ]
    else:
        index_arguments = ["--no-index"]
    download_directory.mkdir(parents=True, exist_ok=True)
    start = time.perf_counter()
    subprocess.run(
        [
            sys.executable,
            "-s",
            "-m",
            "pip",
            "download",
            "--disable-pip-version-check",
            *index_arguments,
            "--find-links",
            str(wheel_directory),
            "--cache-dir",
            str(cache_directory),
            "--dest",
            str(download_directory),
            *(str(wheel) for wheel in wheels),
        ],
        check=True,
        env={**os.environ, "PYTHONNOUSERSITE": "1"},
    )
    return time.perf_counter() - start


def _measure_downloaded_distributions(
    download_directory: Path, root_distribution_names: set[str]
) -> list[dict[str, object]]:
    """Measure downloaded dependency wheel archives.

    Args:
        download_directory: Directory containing the resolved wheel archives.
        root_distribution_names: Normalized names of the explicitly requested wheels.

    Returns:
        Dependency distribution names, versions, filenames, and archive sizes.
    """
    rows = []
    for wheel in download_directory.glob("*.whl"):
        name = _wheel_distribution_name(wheel)
        if name in root_distribution_names:
            continue
        filename_parts = wheel.name.split("-")
        rows.append(
            {
                "name": name,
                "version": filename_parts[1] if len(filename_parts) > 1 else None,
                "filename": wheel.name,
                "archive_size_bytes": wheel.stat().st_size,
            }
        )
    return sorted(rows, key=lambda row: (-int(row["archive_size_bytes"]), str(row["name"])))


def _install_wheels(
    wheel_directory: Path,
    install_directory: Path,
    wheels: list[Path],
    cache_directory: Path,
    *,
    use_indexes: bool = True,
    dependency_directory: Path | None = None,
) -> float:
    """Install wheels and their dependencies into a clean target.

    Args:
        wheel_directory: Local wheelhouse used to resolve source-library dependencies.
        install_directory: Clean installation root.
        wheels: Explicit root wheels to install.
        cache_directory: Shared pip download cache.
        use_indexes: Whether to resolve dependencies from configured network sources.
        dependency_directory: Optional local directory containing resolved dependency archives.

    Returns:
        Install duration in seconds.
    """
    start = time.perf_counter()
    if use_indexes:
        index_arguments = [
            "--index-url",
            _PYPI_INDEX_URL,
            "--extra-index-url",
            _NVIDIA_PYPI_INDEX_URL,
        ]
    else:
        index_arguments = ["--no-index"]

    subprocess.run(
        [
            sys.executable,
            "-s",
            "-m",
            "pip",
            "install",
            "--disable-pip-version-check",
            "--ignore-installed",
            *index_arguments,
            "--find-links",
            str(wheel_directory),
            *(["--find-links", str(dependency_directory)] if dependency_directory is not None else []),
            "--cache-dir",
            str(cache_directory),
            "--target",
            str(install_directory),
            *(str(wheel) for wheel in wheels),
        ],
        check=True,
        env={**os.environ, "PYTHONNOUSERSITE": "1"},
    )
    return time.perf_counter() - start


def _measure_wheel_dependency_closures(
    wheel_directory: Path,
    wheels: list[Path],
    cache_directory: Path,
    *,
    use_indexes: bool = True,
) -> list[dict[str, object]]:
    """Measure an independent clean installation for every wheel.

    Args:
        wheel_directory: Local wheelhouse used for dependency resolution.
        wheels: Wheels to measure.
        cache_directory: Shared pip download cache.
        use_indexes: Whether to resolve dependencies from configured network sources.

    Returns:
        Per-wheel installed size metrics.
    """
    rows = []
    for wheel in wheels:
        with tempfile.TemporaryDirectory(prefix="isaacsim-wheel-closure-") as temporary_directory:
            install_directory = Path(temporary_directory) / "site-packages"
            install_duration = _install_wheels(
                wheel_directory,
                install_directory,
                [wheel],
                cache_directory,
                use_indexes=use_indexes,
            )
            total_size, total_file_count = _directory_size(install_directory)
            own_size, own_file_count = _distribution_size(install_directory, _wheel_distribution_name(wheel))
            rows.append(
                {
                    "filename": wheel.name,
                    "installed_size_bytes": own_size,
                    "installed_file_count": own_file_count,
                    "dependency_install_size_bytes": total_size,
                    "dependency_install_file_count": total_file_count,
                    "dependency_distribution_count": _distribution_count(install_directory),
                    "dependency_install_duration_seconds": round(install_duration, 6),
                }
            )
    return rows


def _load_installed_manifests(install_directory: Path) -> list[dict[str, object]]:
    """Load installed source-library package manifests.

    Args:
        install_directory: Clean wheel installation directory.

    Returns:
        Installed package manifests.

    Raises:
        RuntimeError: If no package manifests are installed or a manifest is invalid.
    """
    paths = sorted((install_directory / "share" / "isaacsim" / "packages").glob("*/package.json"))
    if not paths:
        raise RuntimeError("Installed wheelhouse contains no source-library package manifests")
    manifests = []
    for path in paths:
        document = json.loads(path.read_text(encoding="utf-8"))
        if not isinstance(document, dict):
            raise RuntimeError(f"Installed package manifest is invalid: {path}")
        manifests.append(document)
    return manifests


def _validate_imports(install_directory: Path, manifests: list[dict[str, object]]) -> int:
    """Import every declared module in an isolated process.

    Args:
        install_directory: Clean wheel installation directory.
        manifests: Installed package manifests.

    Returns:
        Number of imported modules.

    Raises:
        RuntimeError: If a package manifest contains invalid metadata.
    """
    imported = 0
    for manifest in manifests:
        name = manifest.get("name")
        version = manifest.get("version")
        imports = manifest.get("python_imports")
        if not isinstance(name, str) or not isinstance(version, str) or not isinstance(imports, list):
            raise RuntimeError("Installed package manifest has invalid name, version, or imports")
        sys.path.insert(0, str(install_directory))
        try:
            installed_version = importlib.metadata.version(name)
        finally:
            sys.path.pop(0)
        if installed_version != version:
            raise RuntimeError(f"Installed {name} version {installed_version} does not match {version}")
        for module_name in imports:
            if not isinstance(module_name, str):
                raise RuntimeError(f"Installed package {name} has an invalid import name")
            print(f"Validating import {module_name}", flush=True)
            subprocess.run(
                [sys.executable, "-s", "-c", _IMPORT_MODULE_CODE, str(install_directory), module_name],
                check=True,
                cwd=install_directory,
                env={**os.environ, "PYTHONNOUSERSITE": "1"},
            )
            imported += 1
    return imported


def _write_metrics(path: Path, metrics: dict[str, object]) -> None:
    """Write install metrics atomically.

    Args:
        path: Metrics output path.
        metrics: Metrics document to write.
    """
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_suffix(f"{path.suffix}.tmp")
    temporary.write_text(json.dumps(metrics, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    temporary.replace(path)


def _main() -> int:
    """Install the wheelhouse, validate imports, and record metrics.

    Returns:
        Zero after successful validation.
    """
    arguments = _parse_arguments()
    wheel_directory = arguments.wheel_directory.resolve()
    wheels = sorted(wheel_directory.glob("*.whl"))
    if not wheels:
        raise RuntimeError(f"No wheels found in {wheel_directory}")

    with tempfile.TemporaryDirectory(prefix="isaacsim-wheel-metrics-") as temporary_directory:
        temporary_root = Path(temporary_directory)
        cache_directory = temporary_root / "pip-cache"
        download_directory = temporary_root / "downloaded-wheelhouse"
        install_directory = temporary_root / "complete-environment"
        root_distribution_names = {_wheel_distribution_name(wheel) for wheel in wheels}
        download_duration = _download_wheel_dependencies(wheel_directory, download_directory, wheels, cache_directory)
        downloaded_dependency_distributions = _measure_downloaded_distributions(
            download_directory, root_distribution_names
        )
        install_duration = download_duration + _install_wheels(
            wheel_directory,
            install_directory,
            wheels,
            cache_directory,
            use_indexes=False,
            dependency_directory=download_directory,
        )
        installed_size, file_count = _directory_size(install_directory)
        installed_distribution_count = _distribution_count(install_directory)
        dependency_distributions = _measure_installed_distributions(install_directory, root_distribution_names)
        manifests = _load_installed_manifests(install_directory)
        imported_modules = _validate_imports(install_directory, manifests)
        wheel_metrics = _measure_wheel_dependency_closures(wheel_directory, wheels, cache_directory)

    metrics = {
        "schema_version": 4,
        "wheel_count": len(wheels),
        "package_count": len(manifests),
        "import_count": imported_modules,
        "install_duration_seconds": round(install_duration, 6),
        "installed_size_bytes": installed_size,
        "installed_file_count": file_count,
        "installed_distribution_count": installed_distribution_count,
        "downloaded_dependency_distributions": downloaded_dependency_distributions,
        "dependency_distributions": dependency_distributions,
        "wheels": wheel_metrics,
    }
    _write_metrics(arguments.metrics_output.resolve(), metrics)
    print(
        f"Installed {len(wheels)} wheels in {install_duration:.2f} seconds; "
        f"final size {installed_size / (1024 * 1024):.2f} MiB; imported {imported_modules} modules"
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(_main())
