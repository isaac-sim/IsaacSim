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

"""Build standalone pip wheels from packages that have an in-source pyproject.toml.

This is a repo tool that integrates with ``./repo.sh``:

    ./repo.sh build_standalone_wheels              # build all
    ./repo.sh build_standalone_wheels --list       # list eligible packages
    ./repo.sh build_standalone_wheels --ext isaacsim.asset.transformer  # build one

Wheels are output to ``_build/packages/standalone_wheels/``.

The tool uses Kit's bundled Python for building so that the Python version and
ABI tags are consistent with the monolithic ``isaacsim-*`` wheels built by
``./repo.sh python_package``.

Packages whose source layout differs from their import namespace (e.g.
``python/`` dirs, non-standard module names) declare symlinks in their
``pyproject.toml`` under ``[tool.standalone-wheel] symlinks``.  This tool
creates the symlinks before building and removes them afterwards so they
never need to be committed to the repo.
"""

from __future__ import annotations

import glob
import logging
import os
import platform as platform_mod
import re
import shutil
import subprocess
import sys

import omni.repo.man

logger = logging.getLogger(__name__)

# Map repo build platforms to pip wheel platform tags.
PLATFORM_TAGS = {
    "linux-x86_64": "manylinux_2_35_x86_64",
    "linux-aarch64": "manylinux_2_35_aarch64",
    "windows-x86_64": "win_amd64",
}


def setup_repo_tool(parser, config):
    """Register CLI arguments for ``./repo.sh build_standalone_wheels``."""
    parser.prog = "build_standalone_wheels"
    parser.description = "Build standalone pip wheels from packages with in-source pyproject.toml"

    parser.add_argument(
        "--list",
        default=False,
        action="store_true",
        help="List eligible packages and exit.",
    )
    parser.add_argument(
        "--ext",
        dest="extensions",
        nargs="*",
        default=None,
        help="Build only the specified package(s). Default: build all.",
    )
    parser.add_argument(
        "-c",
        "--config",
        dest="build_config",
        default="release",
        help="Build configuration (default: %(default)s).",
    )
    parser.add_argument(
        "--clean",
        default=False,
        action="store_true",
        help="Remove output directory before building.",
    )
    parser.add_argument(
        "--test",
        default=False,
        action="store_true",
        help="After building, install into a temp venv and run standalone_tests/.",
    )
    return run_repo_tool


def _detect_platform() -> str:
    """Return the repo-style platform string for the current host."""
    system = platform_mod.system().lower()
    machine = platform_mod.machine().lower()
    if system == "linux":
        if machine in ("x86_64", "amd64"):
            return "linux-x86_64"
        if machine in ("aarch64", "arm64"):
            return "linux-aarch64"
    elif system == "windows":
        return "windows-x86_64"
    raise RuntimeError(f"Unsupported platform: {system}-{machine}")


def _find_kit_python(repo_root: str, build_platform: str, build_config: str) -> str:
    """Locate Kit's bundled Python executable."""
    if build_platform.startswith("windows"):
        name = "python.exe"
    else:
        name = "python3"
    path = os.path.join(repo_root, "_build", build_platform, build_config, "kit", "python", name)
    if not os.path.isfile(path):
        raise FileNotFoundError(f"Kit Python not found at {path}. Run build.sh first to fetch Kit dependencies.")
    return path


def _find_eligible_packages(repo_root: str) -> list[str]:
    """Return package directories eligible for direct standalone-wheel builds.

    Extension packages retain the historical behavior: every top-level
    extension with a ``pyproject.toml`` is eligible. Library packages must
    explicitly opt in with a ``[tool.standalone-wheel]`` table because many
    native libraries use ``pyproject.toml`` only as metadata for their
    CMake-driven wheel build.
    """
    results = []

    extensions_dir = os.path.join(repo_root, "source", "extensions")
    for entry in sorted(os.listdir(extensions_dir)):
        package_dir = os.path.join(extensions_dir, entry)
        if os.path.isdir(package_dir) and os.path.isfile(os.path.join(package_dir, "pyproject.toml")):
            results.append(package_dir)

    libraries_dir = os.path.join(repo_root, "source", "libraries")
    if os.path.isdir(libraries_dir):
        for current_dir, directory_names, file_names in os.walk(libraries_dir):
            directory_names.sort()
            if "pyproject.toml" in file_names and _has_standalone_wheel_config(current_dir):
                results.append(current_dir)

    return sorted(results)


def _get_package_metadata(package_dir: str) -> dict:
    """Read a package's ``pyproject.toml``."""
    try:
        import tomllib
    except ModuleNotFoundError:
        import tomli as tomllib  # type: ignore[no-redef]

    with open(os.path.join(package_dir, "pyproject.toml"), "rb") as file_handle:
        return tomllib.load(file_handle)


def _has_standalone_wheel_config(package_dir: str) -> bool:
    """Return whether a package explicitly opts into the standalone builder."""
    metadata = _get_package_metadata(package_dir)
    return "standalone-wheel" in metadata.get("tool", {})


def _get_package_name(package_dir: str) -> str:
    """Read the normalized project name from a package's ``pyproject.toml``."""
    return _get_package_metadata(package_dir)["project"]["name"]


def _get_package_id(package_dir: str) -> str:
    """Return the dotted package identifier accepted by ``--ext``."""
    metadata = _get_package_metadata(package_dir)
    package_id = metadata.get("tool", {}).get("standalone-wheel", {}).get("package-id")
    return package_id or metadata["project"]["name"].replace("-", ".")


def _is_native_package(ext_dir: str) -> bool:
    """Check if pyproject.toml declares [tool.standalone-wheel] native = true."""
    pyproject_path = os.path.join(ext_dir, "pyproject.toml")
    try:
        with open(pyproject_path, "r") as f:
            for line in f:
                stripped = line.strip()
                if stripped.startswith("native") and "=" in stripped and "true" in stripped.lower():
                    return True
    except OSError:
        pass
    return False


def _parse_symlinks(ext_dir: str) -> list[tuple[str, str]]:
    """Parse [tool.standalone-wheel] symlinks from pyproject.toml.

    Returns a list of (link_path, target) tuples where paths are relative to
    the package directory.  The format in pyproject.toml is::

        [tool.standalone-wheel]
        symlinks = [
            {link = "isaacsim/asset/importer/urdf/__init__.py", target = "python/__init__.py"},
            {link = "isaacsim/asset/importer/urdf/impl", target = "python/impl"},
        ]
    """
    pyproject_path = os.path.join(ext_dir, "pyproject.toml")
    try:
        # Use tomllib (Python 3.11+) or fall back to a simple parser.
        try:
            import tomllib
        except ModuleNotFoundError:
            import tomli as tomllib  # type: ignore[no-redef]

        with open(pyproject_path, "rb") as f:
            data = tomllib.load(f)

        entries = data.get("tool", {}).get("standalone-wheel", {}).get("symlinks", [])
        return [(e["link"], e["target"]) for e in entries if "link" in e and "target" in e]
    except Exception:
        return []


def _create_symlinks(ext_dir: str, symlinks: list[tuple[str, str]]) -> list[str]:
    """Create symlinks for a package.  Returns list of created paths for cleanup."""
    created = []
    for link_rel, target_rel in symlinks:
        link_path = os.path.join(ext_dir, link_rel)
        # Compute the relative target from the link's parent directory.
        link_parent = os.path.dirname(link_path)
        target_abs = os.path.normpath(os.path.join(ext_dir, target_rel))
        target_relative = os.path.relpath(target_abs, link_parent)

        os.makedirs(link_parent, exist_ok=True)
        if os.path.islink(link_path) or os.path.exists(link_path):
            # Already exists (e.g. from a previous interrupted build).
            if os.path.islink(link_path):
                os.unlink(link_path)
            else:
                continue  # Don't overwrite real files.
        os.symlink(target_relative, link_path)
        created.append(link_path)
    return created


def _cleanup_symlinks(ext_dir: str, symlinks: list[tuple[str, str]]) -> None:
    """Remove symlinks and any empty parent directories created for them."""
    for link_rel, _ in symlinks:
        link_path = os.path.join(ext_dir, link_rel)
        if os.path.islink(link_path):
            os.unlink(link_path)

    # Walk unique parent dirs from deepest to shallowest; remove if empty.
    dirs_to_check = set()
    for link_rel, _ in symlinks:
        parts = link_rel.split("/")
        for i in range(len(parts) - 1, 0, -1):
            dirs_to_check.add(os.path.join(ext_dir, *parts[:i]))

    for d in sorted(dirs_to_check, key=len, reverse=True):
        try:
            if os.path.isdir(d) and not os.listdir(d):
                os.rmdir(d)
        except OSError:
            pass


def _build_wheel(python: str, ext_dir: str, output_dir: str, plat_tag: str | None = None) -> bool:
    """Build a wheel for a single package using Kit's Python."""
    ext_name = _get_package_id(ext_dir)
    omni.repo.man.print_log(f"Building standalone wheel: {ext_name}", logging.INFO)

    # Create temporary symlinks declared in pyproject.toml.
    symlinks = _parse_symlinks(ext_dir)
    created_links = _create_symlinks(ext_dir, symlinks) if symlinks else []

    try:
        cmd = [
            python,
            "-s",  # Skip user site-packages to avoid pollution.
            "-m",
            "pip",
            "wheel",
            "--no-deps",
            "-w",
            output_dir,
        ]
        if plat_tag:
            cmd.extend(["--plat-name", plat_tag])
        cmd.append(ext_dir)

        ret = omni.repo.man.run_process(cmd, exit_on_error=False)
    finally:
        # Always clean up symlinks, even on failure.
        if symlinks:
            _cleanup_symlinks(ext_dir, symlinks)

        # Clean build artifacts from source tree.
        for artifact in ("build", f"{ext_name}.egg-info"):
            artifact_path = os.path.join(ext_dir, artifact)
            if os.path.isdir(artifact_path):
                shutil.rmtree(artifact_path, ignore_errors=True)
        for egg_info in glob.glob(os.path.join(ext_dir, "*.egg-info")):
            shutil.rmtree(egg_info, ignore_errors=True)

    if ret != 0:
        omni.repo.man.print_log(f"FAILED: {ext_name}", logging.ERROR)
        return False

    omni.repo.man.print_log(f"OK: {ext_name}", logging.INFO)
    return True


def _find_built_wheel(package_dir: str, output_dir: str) -> str:
    """Return the single wheel matching a package's declared name and version."""
    metadata = _get_package_metadata(package_dir)
    project = metadata.get("project", {})
    name = project.get("name")
    version = project.get("version")
    if not isinstance(name, str) or not isinstance(version, str):
        raise RuntimeError(f"Package must declare string project.name and project.version: {package_dir}")
    normalized_name = re.sub(r"[-_.]+", "_", name).lower()
    matches = sorted(glob.glob(os.path.join(output_dir, f"{normalized_name}-{version}-*.whl")))
    if len(matches) != 1:
        raise RuntimeError(f"Expected one built wheel for {name}=={version}, found {len(matches)}")
    return matches[0]


def _find_standalone_tests(eligible: list[str]) -> list[tuple[str, str]]:
    """Find packages that have standalone_tests/ directories.

    Returns list of (package_name, tests_dir) tuples.
    """
    results = []
    for ext_dir in eligible:
        tests_dir = os.path.join(ext_dir, "standalone_tests")
        if os.path.isdir(tests_dir):
            results.append((_get_package_id(ext_dir), tests_dir))
    return results


def _run_standalone_tests(
    python: str,
    repo_root: str,
    output_dir: str,
    build_config: str,
    eligible: list[str],
) -> int:
    """Install wheels into a temp venv and run standalone_tests/ for each package.

    Returns 0 on success, 1 on failure.
    """
    import tempfile as _tempfile

    test_packages = _find_standalone_tests(eligible)
    if not test_packages:
        omni.repo.man.print_log("No standalone_tests/ directories found.", logging.WARN)
        return 0

    omni.repo.man.print_log(f"Running standalone tests for {len(test_packages)} package(s)...", logging.INFO)

    venv_dir = _tempfile.mkdtemp(prefix="isaacsim_standalone_test_")
    try:
        # Create venv.
        omni.repo.man.print_log(f"  Creating temp venv: {venv_dir}", logging.INFO)
        ret = omni.repo.man.run_process([python, "-s", "-m", "venv", venv_dir], exit_on_error=False)
        if ret != 0:
            omni.repo.man.print_log("Failed to create venv.", logging.ERROR)
            return 1

        venv_python = os.path.join(venv_dir, "bin", "python3")
        if not os.path.isfile(venv_python):
            venv_python = os.path.join(venv_dir, "Scripts", "python.exe")

        # Install the selected wheels into the clean venv with normal dependency
        # resolution. The local wheelhouses satisfy isaacsim-* inter-package
        # requirements while PyPI/index configuration supplies external deps.
        try:
            wheels = sorted(_find_built_wheel(package_dir, output_dir) for package_dir in eligible)
        except RuntimeError as error:
            omni.repo.man.print_log(str(error), logging.ERROR)
            return 1

        omni.repo.man.print_log(f"  Installing {len(wheels)} wheel(s) with dependency resolution...", logging.INFO)
        install_command = [
            venv_python,
            "-s",
            "-m",
            "pip",
            "install",
            "--quiet",
            "--find-links",
            output_dir,
        ]
        extension_wheelhouse = os.path.join(repo_root, "_cmake_build", "extension-wheel-artifacts", build_config)
        if os.path.isdir(extension_wheelhouse):
            install_command.extend(["--find-links", extension_wheelhouse])
        ret = omni.repo.man.run_process(install_command + wheels, exit_on_error=False)
        if ret != 0:
            omni.repo.man.print_log("Failed to install wheels.", logging.ERROR)
            return 1

        omni.repo.man.print_log("  Checking installed wheel dependencies...", logging.INFO)
        ret = omni.repo.man.run_process(
            [venv_python, "-s", "-m", "pip", "check"],
            exit_on_error=False,
        )
        if ret != 0:
            omni.repo.man.print_log("Dependency check failed.", logging.ERROR)
            return 1

        # Run tests for each package.
        passed = 0
        test_failed = 0
        for package_name, tests_dir in test_packages:
            omni.repo.man.print_log(f"  Testing: {package_name}", logging.INFO)
            ret = omni.repo.man.run_process(
                [venv_python, "-s", "-m", "unittest", "discover", "-s", tests_dir, "-p", "test_*.py", "-v"],
                exit_on_error=False,
            )
            if ret == 0:
                passed += 1
            else:
                test_failed += 1

        omni.repo.man.print_log(f"Standalone tests: {passed} passed, {test_failed} failed", logging.INFO)
        return 1 if test_failed > 0 else 0

    finally:
        shutil.rmtree(venv_dir, ignore_errors=True)


def run_repo_tool(options, config):
    """Entry point called by ``repo.sh``."""
    repo_root = config["repo"]["folders"]["root"]
    build_config = options.build_config
    build_platform = _detect_platform()
    plat_tag = PLATFORM_TAGS.get(build_platform)
    output_dir = os.path.join(repo_root, "_build", "packages", "standalone_wheels")

    # Find Kit Python.
    try:
        python = _find_kit_python(repo_root, build_platform, build_config)
    except FileNotFoundError as e:
        omni.repo.man.print_log(str(e), logging.ERROR)
        return 1

    # Find eligible packages.
    eligible = _find_eligible_packages(repo_root)
    if not eligible:
        omni.repo.man.print_log("No packages with pyproject.toml found.", logging.WARN)
        return 0

    # --list mode: print and exit.
    if options.list:
        omni.repo.man.print_log("Eligible packages for standalone packaging:", logging.INFO)
        for ext_dir in eligible:
            native = " (native)" if _is_native_package(ext_dir) else ""
            symlinks = _parse_symlinks(ext_dir)
            sym = f" ({len(symlinks)} symlinks)" if symlinks else ""
            tests = " (has tests)" if os.path.isdir(os.path.join(ext_dir, "standalone_tests")) else ""
            print(f"  {_get_package_id(ext_dir)}{native}{sym}{tests}")
        return 0

    # Filter to requested packages.
    if options.extensions:
        requested = set(options.extensions)
        filtered = [
            package_dir
            for package_dir in eligible
            if _get_package_id(package_dir) in requested or _get_package_name(package_dir) in requested
        ]
        matched = {_get_package_id(package_dir) for package_dir in filtered}
        matched.update(_get_package_name(package_dir) for package_dir in filtered)
        not_found = requested - matched
        if not_found:
            omni.repo.man.print_log(
                f"Packages not found or missing pyproject.toml: {', '.join(sorted(not_found))}",
                logging.ERROR,
            )
            return 1
        eligible = filtered

    # Clean output directory if requested.
    if options.clean and os.path.isdir(output_dir):
        shutil.rmtree(output_dir)
    os.makedirs(output_dir, exist_ok=True)

    # Build.
    python_version = subprocess.check_output([python, "--version"], stderr=subprocess.STDOUT).decode().strip()

    omni.repo.man.print_log(f"Building standalone wheels...", logging.INFO)
    omni.repo.man.print_log(f"  Python:   {python} ({python_version})", logging.INFO)
    omni.repo.man.print_log(f"  Platform: {build_platform} ({plat_tag})", logging.INFO)
    omni.repo.man.print_log(f"  Config:   {build_config}", logging.INFO)
    omni.repo.man.print_log(f"  Output:   {output_dir}", logging.INFO)

    built = 0
    failed = 0
    for ext_dir in eligible:
        native = _is_native_package(ext_dir)
        tag = plat_tag if native else None
        if _build_wheel(python, ext_dir, output_dir, plat_tag=tag):
            built += 1
        else:
            failed += 1

    omni.repo.man.print_log(f"Standalone wheels: {built} built, {failed} failed", logging.INFO)
    omni.repo.man.print_log(f"Output: {output_dir}", logging.INFO)

    if failed > 0:
        return 1

    # Run tests if requested.
    if options.test:
        return _run_standalone_tests(python, repo_root, output_dir, build_config, eligible)

    return 0
