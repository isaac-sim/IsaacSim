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

"""Build, run, and test Isaac Sim examples."""

from __future__ import annotations

import argparse
import contextlib
import hashlib
import importlib.metadata
import importlib.util
import json
import os
import re
import runpy
import shlex
import shutil
import signal
import site
import stat
import subprocess
import sys
import sysconfig
import tempfile
import time
from collections.abc import Iterator
from dataclasses import dataclass
from pathlib import Path, PurePosixPath
from typing import Any

if sys.version_info >= (3, 11):
    import tomllib
else:  # The developer bootstrap may initially run under Python 3.10.
    tomllib = None

from packaging.requirements import InvalidRequirement, Requirement
from packaging.tags import sys_tags
from packaging.utils import canonicalize_name
from packaging.version import InvalidVersion, Version

DEFAULT_TEST_TIMEOUT_SECONDS = 60
PROCESS_TERMINATION_TIMEOUT_SECONDS = 5
DEVELOPER_ENVIRONMENT_VARIABLE = "ISAACSIM_EXAMPLES_DEVELOPER_ENVIRONMENT"
DEVELOPER_ENVIRONMENT_DIRECTORY = "developer-environment"
DEVELOPER_ENVIRONMENT_LOCK_SUFFIX = "developer-environment.lock"
PYTHON_DEPENDENCY_DIRECTORY = "python-dependencies"
DEPENDENCY_STATE_FILE = "dependency-state.json"
CPP_DEPENDENCY_DIRECTORY = "cpp-dependencies"
CPP_PACKAGE_PATTERN = re.compile(r"([a-z0-9][a-z0-9._-]*)==([A-Za-z0-9][A-Za-z0-9._+-]*)")
EXAMPLE_ID_PATTERN = re.compile(r"[a-z][a-z0-9_]*(\.[a-z][a-z0-9_]*)*")
NAME_PATTERN = re.compile(r"[a-z][a-z0-9_]*")
TARGET_PATTERN = re.compile(r"[A-Za-z0-9_][A-Za-z0-9_.+-]*")
ENVIRONMENT_NAME_PATTERN = re.compile(r"[A-Za-z_][A-Za-z0-9_]*")
ALLOWED_EXAMPLE_GROUPS = frozenset({"libraries", "series"})
ALLOWED_LEVELS = frozenset({"beginner", "intermediate", "expert"})
ALLOWED_SERIES_KINDS = frozenset({"collection", "sequence"})
ALLOWED_MODULE_SURFACES = frozenset({"native_sdk", "python"})
SYSTEM_REQUIREMENT_NAMES = frozenset({"c", "cmake", "cpp", "python", "python_development"})
SYSTEM_REQUIREMENTS_FIELDS = frozenset(
    {"c_standard", "cmake_minimum_version", "cmake_policy_maximum_version", "cpp_standard", "python_version"}
)
VERSION_PATTERN = re.compile(r"(0|[1-9][0-9]*)\.(0|[1-9][0-9]*)(\.(0|[1-9][0-9]*))?")
GENERATED_DIRECTORY_NAMES = frozenset(
    {"__pycache__", ".mypy_cache", ".pytest_cache", ".ruff_cache", "_build", "build", "CMakeFiles", "dist"}
)
GENERATED_FILE_NAMES = frozenset(
    {
        "CMakeCache.txt",
        "Makefile",
        ".ninja_deps",
        ".ninja_log",
        "build.ninja",
        "cmake_install.cmake",
        "compile_commands.json",
    }
)
GENERATED_FILE_SUFFIXES = frozenset({".a", ".dll", ".dylib", ".exe", ".lib", ".o", ".obj", ".pyc", ".pyd", ".so"})


class ExampleError(RuntimeError):
    """Error raised for an invalid example or failed example operation."""


def _initialize_windows_native_toolchain() -> None:
    """Initialize MSVC for native examples in an Isaac Sim source checkout.

    Raises:
        ExampleError: If the source library build tool cannot initialize MSVC.
    """
    if sys.platform != "win32":
        return

    build_tool_path = Path(__file__).resolve().parents[1] / "libraries" / "tools" / "build.py"
    if not build_tool_path.is_file():
        return
    try:
        build_tool = runpy.run_path(str(build_tool_path))
        build_tool["ensure_windows_msvc_environment"]()
    except Exception as error:
        raise ExampleError(f"Cannot initialize the Windows C++ toolchain: {error}") from error


@dataclass(frozen=True)
class _BuildConfiguration:
    """Structured build configuration for an example."""

    adapter: str
    targets: tuple[str, ...] = ()


@dataclass(frozen=True)
class _RunConfiguration:
    """Structured run configuration for an example."""

    adapter: str
    target: str | None = None
    path: Path | None = None
    arguments: tuple[str, ...] = ()


@dataclass(frozen=True)
class _TestConfiguration:
    """Named automated-test configuration for an example."""

    name: str
    path: Path | None
    arguments: tuple[str, ...] | None
    timeout_seconds: int
    expected_exit_code: int
    stdout_contains: tuple[str, ...]
    stderr_contains: tuple[str, ...]
    environment: dict[str, str]


@dataclass(frozen=True)
class _ModuleRequirement:
    """One direct Isaac Sim module distribution requirement."""

    name: str
    surfaces: tuple[str, ...]


@dataclass(frozen=True)
class _PythonPackageRequirement:
    """One exact external Python distribution requirement."""

    name: str
    version: str

    @property
    def specifier(self) -> str:
        """Return the canonical PEP 508 requirement string.

        Returns:
            Exact distribution requirement.
        """
        return f"{self.name}=={self.version}"


@dataclass(frozen=True)
class _CppPackageRequirement:
    """One exact external Pixi package requirement."""

    name: str
    version: str

    @property
    def specifier(self) -> str:
        """Return the exact Pixi package pin.

        Returns:
            Exact package requirement.
        """
        return f"{self.name}=={self.version}"


@dataclass(frozen=True)
class _CppDependencyEnvironment:
    """Installed Pixi workspace used by external C++ dependencies."""

    pixi: Path
    manifest: Path


@dataclass(frozen=True)
class _Requirements:
    """Validated module, external package, and system requirements."""

    modules: tuple[_ModuleRequirement, ...]
    python_packages: tuple[_PythonPackageRequirement, ...]
    cpp_packages: tuple[_CppPackageRequirement, ...]
    system: tuple[str, ...]


@dataclass(frozen=True)
class _SystemRequirements:
    """Release-wide system toolchain requirements."""

    cmake_minimum_version: str
    cmake_policy_maximum_version: str
    c_standard: int
    cpp_standard: int
    python_version: str


@dataclass(frozen=True)
class _Example:
    """Validated example definition."""

    id: str
    title: str
    summary: str
    owners: tuple[str, ...]
    categories: tuple[str, ...]
    topics: tuple[str, ...]
    root: Path
    build: _BuildConfiguration
    run: _RunConfiguration
    requirements: _Requirements
    tests: tuple[_TestConfiguration, ...]


@dataclass(frozen=True)
class _SeriesStep:
    """One validated entry in an example series."""

    example_id: str
    level: str


@dataclass(frozen=True)
class _Series:
    """Validated example series."""

    id: str
    title: str
    summary: str
    kind: str
    root: Path
    steps: tuple[_SeriesStep, ...]


@dataclass(frozen=True)
class _CommandResult:
    """Captured command result."""

    returncode: int
    stdout: str
    stderr: str
    timed_out: bool = False


@dataclass(frozen=True)
class _DeveloperBuild:
    """Configured library build used to create a developer environment."""

    library_build_dir: Path
    cmake: Path
    python: Path
    python_install_dir: str
    python_runtime_dependencies: Path
    developer_environment: Path
    generator: str
    generator_platform: str | None
    generator_toolset: str | None
    make_program: str | None


def _validate_keys(
    table: dict[str, Any],
    required: set[str],
    optional: set[str],
    context: str,
) -> None:
    """Validate the exact set of keys accepted by a TOML table.

    Args:
        table: Table to validate.
        required: Required field names.
        optional: Optional field names.
        context: User-facing location of the table.
    """
    missing = required - table.keys()
    unknown = table.keys() - required - optional
    if missing:
        raise ExampleError(f"{context} is missing fields: {', '.join(sorted(missing))}")
    if unknown:
        raise ExampleError(f"{context} has unknown fields: {', '.join(sorted(unknown))}")


def _read_toml(path: Path) -> dict[str, Any]:
    """Read a TOML document and report parse failures with its path.

    Args:
        path: TOML document path.

    Returns:
        Parsed top-level table.
    """
    if tomllib is None:
        raise ExampleError(f"Reading {path} requires Python 3.11 or newer")
    try:
        value = tomllib.loads(path.read_text(encoding="utf-8"))
    except (OSError, tomllib.TOMLDecodeError) as error:
        raise ExampleError(f"Cannot read {path}: {error}") from error
    if not isinstance(value, dict):
        raise ExampleError(f"{path} must contain a TOML table")
    return value


def _read_string(table: dict[str, Any], field: str, context: str) -> str:
    """Read a required non-empty string.

    Args:
        table: Table containing the field.
        field: Field name to read.
        context: User-facing location of the table.

    Returns:
        Validated string value.
    """
    value = table.get(field)
    if not isinstance(value, str) or not value.strip():
        raise ExampleError(f"{context}.{field} must be a non-empty string")
    return value


def _read_string_list(
    table: dict[str, Any],
    field: str,
    context: str,
    *,
    required: bool = True,
    allow_empty: bool = False,
) -> tuple[str, ...]:
    """Read a list of unique strings.

    Args:
        table: Table containing the field.
        field: Field name to read.
        context: User-facing location of the table.
        required: Whether the field must be present.
        allow_empty: Whether an empty list is valid.

    Returns:
        Validated string values.
    """
    value = table.get(field)
    if value is None and not required:
        return ()
    if not isinstance(value, list) or any(not isinstance(item, str) or not item for item in value):
        raise ExampleError(f"{context}.{field} must be a list of non-empty strings")
    if not allow_empty and not value:
        raise ExampleError(f"{context}.{field} must not be empty")
    if len(value) != len(set(value)):
        raise ExampleError(f"{context}.{field} must not contain duplicates")
    return tuple(value)


def _read_argument_list(
    table: dict[str, Any],
    field: str,
    context: str,
    *,
    required: bool = True,
) -> tuple[str, ...]:
    """Read an ordered command-line argument list.

    Args:
        table: Table containing the field.
        field: Field name to read.
        context: User-facing location of the table.
        required: Whether the field must be present.

    Returns:
        Argument values exactly as authored.
    """
    value = table.get(field)
    if value is None and not required:
        return ()
    if not isinstance(value, list) or any(not isinstance(item, str) for item in value):
        raise ExampleError(f"{context}.{field} must be a list of strings")
    return tuple(value)


def _load_system_requirements(path: Path) -> _SystemRequirements:
    """Load the release-wide system toolchain requirements.

    Args:
        path: Requirements file shipped beside the modules.

    Returns:
        Validated system requirements.
    """
    table = _read_toml(path)
    context = str(path)
    _validate_keys(table, set(SYSTEM_REQUIREMENTS_FIELDS), set(), context)
    for field in ("cmake_minimum_version", "cmake_policy_maximum_version", "python_version"):
        value = _read_string(table, field, context)
        if VERSION_PATTERN.fullmatch(value) is None:
            raise ExampleError(f"{context}.{field} must be a two- or three-component version")
    if str(table["python_version"]).count(".") != 1:
        raise ExampleError(f"{context}.python_version must identify a Python major and minor version")
    minimum_cmake_parts = tuple(int(component) for component in str(table["cmake_minimum_version"]).split("."))
    maximum_cmake_parts = tuple(int(component) for component in str(table["cmake_policy_maximum_version"]).split("."))
    minimum_cmake = minimum_cmake_parts + (0,) * (3 - len(minimum_cmake_parts))
    maximum_cmake = maximum_cmake_parts + (0,) * (3 - len(maximum_cmake_parts))
    if maximum_cmake < minimum_cmake:
        raise ExampleError(f"{context}.cmake_policy_maximum_version must not precede cmake_minimum_version")
    for field in ("c_standard", "cpp_standard"):
        value = table[field]
        if type(value) is not int or value <= 0:
            raise ExampleError(f"{context}.{field} must be a positive integer")
    return _SystemRequirements(
        cmake_minimum_version=str(table["cmake_minimum_version"]),
        cmake_policy_maximum_version=str(table["cmake_policy_maximum_version"]),
        c_standard=int(table["c_standard"]),
        cpp_standard=int(table["cpp_standard"]),
        python_version=str(table["python_version"]),
    )


def _validate_python_version(requirements: _SystemRequirements) -> None:
    """Require the runner's Python to match the release-wide version.

    Args:
        requirements: Release-wide system requirements.
    """
    actual_version = f"{sys.version_info.major}.{sys.version_info.minor}"
    if actual_version != requirements.python_version:
        raise ExampleError(
            f"The examples require Python {requirements.python_version}, "
            f"but {sys.executable} is Python {actual_version}"
        )


def _read_cmake_cache(path: Path) -> dict[str, str]:
    """Read string values from a configured CMake cache.

    Args:
        path: CMake cache path.

    Returns:
        Cache values indexed by variable name.
    """
    try:
        lines = path.read_text(encoding="utf-8").splitlines()
    except OSError as error:
        raise ExampleError(f"Cannot read the configured library build {path}: {error}") from error

    values: dict[str, str] = {}
    for line in lines:
        if not line or line.startswith(("#", "//")) or "=" not in line or ":" not in line.partition("=")[0]:
            continue
        key_and_type, _, value = line.partition("=")
        key, _, _ = key_and_type.partition(":")
        values[key] = value
    return values


def _library_source_digest(root: Path) -> str:
    """Fingerprint authored library sources using the build tool's exclusions.

    Args:
        root: Root directory to process.

    Returns:
        The resulting value.
    """
    excluded = frozenset({"__pycache__", ".mypy_cache", ".pytest_cache", ".ruff_cache", "dist"})
    digest = hashlib.sha256()
    root_depth = len(root.parts)
    for path in sorted(root.rglob("*")):
        relative_parts = path.parts[root_depth:]
        if any(part in excluded for part in relative_parts) or not path.is_file():
            continue
        digest.update("/".join(relative_parts).encode())
        digest.update(b"\0")
        with path.open("rb") as stream:
            for chunk in iter(lambda: stream.read(1024 * 1024), b""):
                digest.update(chunk)
    return digest.hexdigest()


def _validate_developer_artifact_state(
    repository_root: Path, library_build_dir: Path, config: str, *, validate_sources: bool = True
) -> Path:
    """Validate successful full-build state and return its developer environment.

    Args:
        repository_root: Repository root directory.
        library_build_dir: Directory containing built Isaac Sim libraries.
        config: Build configuration name.
        validate_sources: Whether to compare library contents with the recorded source digest.

    Returns:
        The resulting value.
    """
    artifact_path = library_build_dir / "artifact-state.json"
    try:
        artifact_state = json.loads(artifact_path.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError) as error:
        raise ExampleError(
            f"Developer artifacts are not current at {library_build_dir}; "
            f"run the library build before examples: {error}"
        ) from error
    if not isinstance(artifact_state, dict):
        raise ExampleError(f"Developer build state is invalid at {library_build_dir}")
    if (
        artifact_state.get("schema_version") != 1
        or artifact_state.get("configuration") != config
        or artifact_state.get("profile") != "standard"
        or artifact_state.get("dependency_profile") != "locked"
        or artifact_state.get("targets") != ["all"]
    ):
        raise ExampleError(f"Developer artifacts are stale or partial at {library_build_dir}; rebuild source/libraries")
    inputs = artifact_state.get("inputs")
    if not isinstance(inputs, dict) or not inputs:
        raise ExampleError(f"Developer artifact inputs are invalid at {artifact_path}")
    for relative_name, expected_digest in inputs.items():
        if not isinstance(relative_name, str) or not isinstance(expected_digest, str):
            raise ExampleError(f"Developer artifact inputs are invalid at {artifact_path}")
        relative_path = Path(relative_name)
        candidate = (repository_root / relative_path).resolve()
        if (
            relative_path.is_absolute()
            or not candidate.is_relative_to(repository_root.resolve())
            or not candidate.is_file()
        ):
            raise ExampleError(f"Developer configure input is unsafe or missing: {relative_name}")
        digest = hashlib.sha256(candidate.read_bytes()).hexdigest()
        if digest != expected_digest:
            raise ExampleError(f"Developer configure input changed since the build: {relative_name}")
    expected_source_digest = artifact_state.get("source_digest")
    source_root = repository_root / "source" / "libraries"
    if (
        not isinstance(expected_source_digest, str)
        or re.fullmatch(r"[0-9a-f]{64}", expected_source_digest) is None
        or (validate_sources and _library_source_digest(source_root) != expected_source_digest)
    ):
        raise ExampleError("Developer library sources changed since the build; rebuild source/libraries")
    developer_environment = artifact_state.get("developer_environment")
    if not isinstance(developer_environment, dict) or set(developer_environment) != {"components", "path"}:
        raise ExampleError("Developer environment state is missing; rebuild source/libraries")
    relative_path = developer_environment["path"]
    components = developer_environment["components"]
    if (
        relative_path != DEVELOPER_ENVIRONMENT_DIRECTORY
        or not isinstance(components, list)
        or not components
        or any(not isinstance(component, str) or not component for component in components)
    ):
        raise ExampleError("Developer environment state is invalid; rebuild source/libraries")
    environment_path = library_build_dir / relative_path
    is_junction = getattr(environment_path, "is_junction", None)
    if not environment_path.is_dir() or environment_path.is_symlink() or (is_junction is not None and is_junction()):
        raise ExampleError(f"Developer environment is missing or redirected: {environment_path}")
    return environment_path


@contextlib.contextmanager
def _path_lock(lock_path: Path, *, exclusive: bool) -> Iterator[None]:
    """Hold a cross-platform advisory lock on one filesystem path.

    Args:
        lock_path: File used for lock coordination.
        exclusive: Whether to acquire an exclusive lock.

    Yields:
        Control while the lock is held.
    """
    lock_path.parent.mkdir(parents=True, exist_ok=True)
    with lock_path.open("a+b") as lock_file:
        if sys.platform == "win32":
            import ctypes
            import msvcrt
            from ctypes import wintypes

            class Overlapped(ctypes.Structure):
                _fields_ = (
                    ("internal", ctypes.c_size_t),
                    ("internal_high", ctypes.c_size_t),
                    ("offset", wintypes.DWORD),
                    ("offset_high", wintypes.DWORD),
                    ("event", wintypes.HANDLE),
                )

            lock_file.seek(0, os.SEEK_END)
            if lock_file.tell() == 0:
                lock_file.write(b"\0")
                lock_file.flush()

            kernel32 = ctypes.WinDLL("kernel32", use_last_error=True)
            lock_file_ex = kernel32.LockFileEx
            lock_file_ex.argtypes = (
                wintypes.HANDLE,
                wintypes.DWORD,
                wintypes.DWORD,
                wintypes.DWORD,
                wintypes.DWORD,
                ctypes.POINTER(Overlapped),
            )
            lock_file_ex.restype = wintypes.BOOL
            unlock_file_ex = kernel32.UnlockFileEx
            unlock_file_ex.argtypes = (
                wintypes.HANDLE,
                wintypes.DWORD,
                wintypes.DWORD,
                wintypes.DWORD,
                ctypes.POINTER(Overlapped),
            )
            unlock_file_ex.restype = wintypes.BOOL

            overlapped = Overlapped()
            flags = 0x2 if exclusive else 0  # `LOCKFILE_EXCLUSIVE_LOCK`.
            handle = msvcrt.get_osfhandle(lock_file.fileno())
            if not lock_file_ex(handle, flags, 0, 1, 0, ctypes.byref(overlapped)):
                raise ctypes.WinError(ctypes.get_last_error())
            try:
                yield
            finally:
                if not unlock_file_ex(handle, 0, 1, 0, ctypes.byref(overlapped)):
                    raise ctypes.WinError(ctypes.get_last_error())
        else:
            import fcntl

            mode = fcntl.LOCK_EX if exclusive else fcntl.LOCK_SH
            fcntl.flock(lock_file.fileno(), mode)
            try:
                yield
            finally:
                fcntl.flock(lock_file.fileno(), fcntl.LOCK_UN)


def _developer_environment_lock(build_directory: Path, *, exclusive: bool) -> contextlib.AbstractContextManager[None]:
    """Coordinate developer-environment readers with replacement and cleanup.

    Args:
        build_directory: Library build directory that owns the developer environment.
        exclusive: Whether to acquire an exclusive lock.

    Returns:
        Context manager holding the requested lock.
    """
    lock_path = build_directory.parent / f".{build_directory.name}-{DEVELOPER_ENVIRONMENT_LOCK_SUFFIX}"
    return _path_lock(lock_path, exclusive=exclusive)


def _read_developer_build(repository_root: Path, config: str, *, validate_sources: bool = True) -> _DeveloperBuild:
    """Read the configured library build selected for developer execution.

    Args:
        repository_root: Isaac Sim repository root.
        config: Requested build configuration.
        validate_sources: Whether to compare library contents with the recorded source digest.

    Returns:
        Configured build tools and paths.
    """
    library_build_dir = repository_root / "_cmake_build" / f"isaacsim-libraries-{config.lower()}"
    cache_path = library_build_dir / "CMakeCache.txt"
    if not cache_path.is_file():
        build_command = "source\\libraries\\build.bat" if os.name == "nt" else "source/libraries/build.sh"
        raise ExampleError(
            f"Developer mode requires a configured {config} library build at {library_build_dir}. "
            f"Run {build_command} {'-d' if config.lower() == 'debug' else '-r'} first."
        )
    cache = _read_cmake_cache(cache_path)
    configured_type = cache.get("CMAKE_BUILD_TYPE", "")
    if configured_type.lower() != config.lower():
        raise ExampleError(
            f"Developer build {library_build_dir} is configured as {configured_type or 'an unknown configuration'}, "
            f"not {config}."
        )

    required_values = (
        "CMAKE_COMMAND",
        "CMAKE_GENERATOR",
        "ISAACSIM_PYTHON_INSTALL_DIR",
        "ISAACSIM_PYTHON_RUNTIME_DEPS_DIR",
        "Python_EXECUTABLE",
    )
    missing_values = [name for name in required_values if not cache.get(name)]
    if missing_values:
        raise ExampleError(f"Developer build cache is missing values: {', '.join(missing_values)}")

    cmake = Path(cache["CMAKE_COMMAND"])
    python = Path(cache["Python_EXECUTABLE"])
    python_runtime_dependencies = Path(cache["ISAACSIM_PYTHON_RUNTIME_DEPS_DIR"])
    python_install_dir = Path(cache["ISAACSIM_PYTHON_INSTALL_DIR"])
    if python_install_dir.is_absolute() or any(part in {".", ".."} for part in python_install_dir.parts):
        raise ExampleError("Developer build cache has an invalid ISAACSIM_PYTHON_INSTALL_DIR")
    missing_paths = [path for path in (cmake, python) if not path.is_file()]
    if not python_runtime_dependencies.is_dir():
        missing_paths.append(python_runtime_dependencies)
    if missing_paths:
        raise ExampleError(
            "Developer build dependencies are incomplete; rebuild source/libraries. Missing: "
            + ", ".join(str(path) for path in missing_paths)
        )

    developer_environment = _validate_developer_artifact_state(
        repository_root, library_build_dir, config, validate_sources=validate_sources
    )
    for required_directory in (
        developer_environment / "lib" / "cmake",
        developer_environment / python_install_dir,
    ):
        if not required_directory.is_dir():
            raise ExampleError(f"Developer environment is incomplete; rebuild source/libraries: {required_directory}")

    return _DeveloperBuild(
        library_build_dir=library_build_dir,
        cmake=cmake,
        python=python,
        python_install_dir=str(python_install_dir),
        python_runtime_dependencies=python_runtime_dependencies,
        developer_environment=developer_environment,
        generator=cache["CMAKE_GENERATOR"],
        generator_platform=cache.get("CMAKE_GENERATOR_PLATFORM") or None,
        generator_toolset=cache.get("CMAKE_GENERATOR_TOOLSET") or None,
        make_program=cache.get("CMAKE_MAKE_PROGRAM") or None,
    )


def _resolve_example_file(example_root: Path, value: str, context: str) -> Path:
    """Resolve a manifest file path within an example root.

    Args:
        example_root: Root containing the example.
        value: POSIX-style manifest path.
        context: User-facing location of the field.

    Returns:
        Resolved path beneath the example root.
    """
    relative_path = PurePosixPath(value)
    if (
        relative_path.is_absolute()
        or not relative_path.parts
        or any(part in {".", ".."} for part in relative_path.parts)
    ):
        raise ExampleError(f"{context} must be a relative path within the example")
    path = (example_root / Path(*relative_path.parts)).resolve()
    try:
        path.relative_to(example_root.resolve())
    except ValueError as error:
        raise ExampleError(f"{context} escapes the example root") from error
    return path


def _load_build_configuration(table: Any, example_root: Path, context: str) -> _BuildConfiguration:
    """Load and validate a build adapter.

    Args:
        table: Build adapter table.
        example_root: Root containing the example.
        context: User-facing location of the table.

    Returns:
        Validated build configuration.
    """
    if not isinstance(table, dict):
        raise ExampleError(f"{context} must be a table")
    adapter = _read_string(table, "adapter", context)
    if adapter == "none":
        _validate_keys(table, {"adapter"}, set(), context)
        return _BuildConfiguration(adapter=adapter)
    if adapter != "cmake":
        raise ExampleError(f"{context}.adapter is unsupported: {adapter}")

    _validate_keys(table, {"adapter", "targets"}, set(), context)
    targets = _read_string_list(table, "targets", context)
    if any(TARGET_PATTERN.fullmatch(target) is None for target in targets):
        raise ExampleError(f"{context}.targets contains an invalid CMake target")
    if not (example_root / "CMakeLists.txt").is_file():
        raise ExampleError(f"{context} requires {example_root / 'CMakeLists.txt'}")
    return _BuildConfiguration(adapter=adapter, targets=targets)


def _load_run_configuration(table: Any, example_root: Path, context: str) -> _RunConfiguration:
    """Load and validate a run adapter.

    Args:
        table: Run adapter table.
        example_root: Root containing the example.
        context: User-facing location of the table.

    Returns:
        Validated run configuration.
    """
    if not isinstance(table, dict):
        raise ExampleError(f"{context} must be a table")
    adapter = _read_string(table, "adapter", context)
    arguments = _read_argument_list(table, "arguments", context, required=False)
    if adapter == "executable":
        _validate_keys(table, {"adapter", "target"}, {"arguments"}, context)
        target = _read_string(table, "target", context)
        if TARGET_PATTERN.fullmatch(target) is None:
            raise ExampleError(f"{context}.target is invalid: {target}")
        return _RunConfiguration(adapter=adapter, target=target, arguments=arguments)
    if adapter == "python":
        _validate_keys(table, {"adapter", "path"}, {"arguments"}, context)
        script_path = _resolve_example_file(example_root, _read_string(table, "path", context), f"{context}.path")
        if not script_path.is_file():
            raise ExampleError(f"{context}.path does not exist: {script_path}")
        return _RunConfiguration(adapter=adapter, path=script_path, arguments=arguments)
    raise ExampleError(f"{context}.adapter is unsupported: {adapter}")


def _load_test_configurations(
    value: Any, example_root: Path, run: _RunConfiguration, context: str
) -> tuple[_TestConfiguration, ...]:
    """Load and validate test overlays.

    Args:
        value: Test array value.
        example_root: Root containing the example manifest.
        run: Normal run configuration that each test overlays.
        context: User-facing location of the array.

    Returns:
        Validated test configurations.
    """
    if value is None:
        return ()
    if not isinstance(value, list):
        raise ExampleError(f"{context} must be an array of tables")

    tests: list[_TestConfiguration] = []
    names: set[str] = set()
    optional_fields = {
        "arguments",
        "environment",
        "expected_exit_code",
        "path",
        "stderr_contains",
        "stdout_contains",
        "timeout_seconds",
    }
    for index, table in enumerate(value):
        test_context = f"{context}[{index}]"
        if not isinstance(table, dict):
            raise ExampleError(f"{test_context} must be a table")
        _validate_keys(table, {"name"}, optional_fields, test_context)
        name = _read_string(table, "name", test_context)
        if NAME_PATTERN.fullmatch(name) is None:
            raise ExampleError(f"{test_context}.name is invalid: {name}")
        if name in names:
            raise ExampleError(f"{context} contains duplicate test name: {name}")
        names.add(name)

        path = None
        if "path" in table:
            if run.adapter != "python":
                raise ExampleError(f"{test_context}.path is supported only by the Python run adapter")
            path = _resolve_example_file(
                example_root, _read_string(table, "path", test_context), f"{test_context}.path"
            )
            if not path.is_file():
                raise ExampleError(f"{test_context}.path does not exist: {path}")

        arguments = None
        if "arguments" in table:
            arguments = _read_argument_list(table, "arguments", test_context)
        timeout_seconds = table.get("timeout_seconds", DEFAULT_TEST_TIMEOUT_SECONDS)
        if type(timeout_seconds) is not int or timeout_seconds <= 0:
            raise ExampleError(f"{test_context}.timeout_seconds must be a positive integer")
        expected_exit_code = table.get("expected_exit_code", 0)
        if type(expected_exit_code) is not int:
            raise ExampleError(f"{test_context}.expected_exit_code must be an integer")
        stdout_contains = _read_string_list(table, "stdout_contains", test_context, required=False, allow_empty=True)
        stderr_contains = _read_string_list(table, "stderr_contains", test_context, required=False, allow_empty=True)
        environment = table.get("environment", {})
        if not isinstance(environment, dict) or any(
            not isinstance(key, str) or ENVIRONMENT_NAME_PATTERN.fullmatch(key) is None or not isinstance(item, str)
            for key, item in environment.items()
        ):
            raise ExampleError(f"{test_context}.environment must contain portable variable names and string values")
        if "PYTHONDONTWRITEBYTECODE" in {key.upper() for key in environment}:
            raise ExampleError(f"{test_context}.environment must not override PYTHONDONTWRITEBYTECODE")
        tests.append(
            _TestConfiguration(
                name=name,
                path=path,
                arguments=arguments,
                timeout_seconds=timeout_seconds,
                expected_exit_code=expected_exit_code,
                stdout_contains=stdout_contains,
                stderr_contains=stderr_contains,
                environment=dict(environment),
            )
        )
    return tuple(tests)


def _load_requirements(value: Any, context: str) -> _Requirements:
    """Validate module, external package, and system requirements.

    Args:
        value: Requirements table.
        context: User-facing location of the table.

    Returns:
        Validated requirements.
    """
    if not isinstance(value, dict):
        raise ExampleError(f"{context} must be a table")
    _validate_keys(value, {"modules"}, {"cpp_packages", "python_packages", "system"}, context)
    modules = value["modules"]
    if not isinstance(modules, list) or not modules:
        raise ExampleError(f"{context}.modules must be a non-empty array of tables")
    names: set[str] = set()
    module_requirements: list[_ModuleRequirement] = []
    for index, module in enumerate(modules):
        module_context = f"{context}.modules[{index}]"
        if not isinstance(module, dict):
            raise ExampleError(f"{module_context} must be a table")
        _validate_keys(module, {"name", "surfaces"}, set(), module_context)
        name = _read_string(module, "name", module_context)
        if NAME_PATTERN.fullmatch(name) is None or name in names:
            raise ExampleError(f"{module_context}.name is invalid or duplicated: {name}")
        names.add(name)
        surfaces = _read_string_list(module, "surfaces", module_context)
        unknown_surfaces = set(surfaces) - ALLOWED_MODULE_SURFACES
        if unknown_surfaces:
            raise ExampleError(f"{module_context}.surfaces is unsupported: {', '.join(sorted(unknown_surfaces))}")
        module_requirements.append(_ModuleRequirement(name=name, surfaces=surfaces))

    python_packages: list[_PythonPackageRequirement] = []
    package_names: set[str] = set()
    for index, requirement_text in enumerate(_read_string_list(value, "python_packages", context, required=False)):
        package_context = f"{context}.python_packages[{index}]"
        try:
            requirement = Requirement(requirement_text)
        except InvalidRequirement as error:
            raise ExampleError(f"{package_context} is not a valid PEP 508 requirement: {error}") from error
        if requirement.url is not None:
            raise ExampleError(f"{package_context} must not use a direct URL or path")
        if requirement.marker is not None:
            raise ExampleError(f"{package_context} must not use an environment marker")
        if requirement.extras:
            raise ExampleError(f"{package_context} must not request extras")
        specifiers = list(requirement.specifier)
        if len(specifiers) != 1 or specifiers[0].operator != "==" or "*" in specifiers[0].version:
            raise ExampleError(f"{package_context} must use one exact == version")
        try:
            version = str(Version(specifiers[0].version))
        except InvalidVersion as error:
            raise ExampleError(f"{package_context} has an invalid PEP 440 version: {error}") from error
        name = canonicalize_name(requirement.name)
        if name == "isaacsim" or name.startswith("isaacsim-"):
            raise ExampleError(f"{package_context} must declare Isaac Sim distributions through requirements.modules")
        if name in package_names:
            raise ExampleError(f"{context}.python_packages contains duplicate distribution: {name}")
        package_names.add(name)
        python_packages.append(_PythonPackageRequirement(name=name, version=version))

    cpp_packages: list[_CppPackageRequirement] = []
    cpp_package_names: set[str] = set()
    for index, requirement_text in enumerate(_read_string_list(value, "cpp_packages", context, required=False)):
        package_context = f"{context}.cpp_packages[{index}]"
        match = CPP_PACKAGE_PATTERN.fullmatch(requirement_text)
        if match is None:
            raise ExampleError(f"{package_context} must use a lowercase package name and one exact == version")
        name, version = match.groups()
        if name == "isaacsim" or name.startswith("isaacsim-"):
            raise ExampleError(f"{package_context} must declare Isaac Sim distributions through requirements.modules")
        if name in cpp_package_names:
            raise ExampleError(f"{context}.cpp_packages contains duplicate package: {name}")
        cpp_package_names.add(name)
        cpp_packages.append(_CppPackageRequirement(name=name, version=version))

    system = _read_string_list(value, "system", context, required=False, allow_empty=True)
    unknown_system_requirements = set(system) - SYSTEM_REQUIREMENT_NAMES
    if unknown_system_requirements:
        raise ExampleError(
            f"{context}.system contains unknown requirements: {', '.join(sorted(unknown_system_requirements))}"
        )
    if "python_development" in system and "python" not in system:
        raise ExampleError(f"{context}.system python_development requires python")
    return _Requirements(
        modules=tuple(module_requirements),
        python_packages=tuple(sorted(python_packages, key=lambda requirement: requirement.name)),
        cpp_packages=tuple(sorted(cpp_packages, key=lambda requirement: requirement.name)),
        system=system,
    )


def _validate_requirement_coherence(
    build: _BuildConfiguration,
    run: _RunConfiguration,
    requirements: _Requirements,
    context: str,
) -> None:
    """Validate that adapters and declared requirements agree.

    Args:
        build: Structured build configuration.
        run: Structured run configuration.
        requirements: Module and system requirements.
        context: User-facing location of the manifest.
    """
    surfaces = {surface for module in requirements.modules for surface in module.surfaces}
    system = set(requirements.system)
    if build.adapter == "cmake":
        if "native_sdk" not in surfaces:
            raise ExampleError(f"{context} uses the CMake adapter but declares no native_sdk module surface")
        if "cmake" not in system:
            raise ExampleError(f"{context} uses the CMake adapter but does not require cmake")
        if not {"c", "cpp"} & system:
            raise ExampleError(f"{context} uses the CMake adapter but does not require a C or C++ compiler")

    if run.adapter == "python":
        if "python" not in system:
            raise ExampleError(f"{context} uses the Python run adapter but does not require python")
        if build.adapter == "cmake" and "python_development" not in system:
            raise ExampleError(
                f"{context} uses mixed CMake/Python adapters but does not require Python development headers"
            )
    if requirements.python_packages and (run.adapter != "python" or "python" not in system):
        raise ExampleError(f"{context} declares Python packages without a Python run adapter and capability")
    if requirements.cpp_packages and (build.adapter != "cmake" or "cpp" not in system):
        raise ExampleError(f"{context} declares C++ packages without a CMake build adapter and C++ capability")


def _load_example(example_root: Path) -> _Example:
    """Load and validate one example manifest.

    Args:
        example_root: Root containing the example manifest.

    Returns:
        Validated example definition.
    """
    manifest_path = example_root / "example.toml"
    if not (example_root / "README.md").is_file():
        raise ExampleError(f"Example root has no README.md: {example_root}")
    table = _read_toml(manifest_path)
    context = str(manifest_path)
    _validate_keys(
        table,
        {"build", "id", "owners", "requirements", "run", "summary", "title"},
        {"categories", "published", "test", "topics"},
        context,
    )
    published = table.get("published", True)
    if type(published) is not bool:
        raise ExampleError(f"{context}.published must be a boolean")
    example_id = _read_string(table, "id", context)
    if EXAMPLE_ID_PATTERN.fullmatch(example_id) is None:
        raise ExampleError(f"{context}.id is invalid: {example_id}")
    title = _read_string(table, "title", context)
    summary = _read_string(table, "summary", context)
    owners = _read_string_list(table, "owners", context)
    if any(EXAMPLE_ID_PATTERN.fullmatch(owner) is None for owner in owners):
        raise ExampleError(f"{context}.owners contains an invalid API owner")
    categories = _read_string_list(table, "categories", context, required=False, allow_empty=True)
    topics = _read_string_list(table, "topics", context, required=False, allow_empty=True)
    for field, values in (("categories", categories), ("topics", topics)):
        if any(NAME_PATTERN.fullmatch(item) is None for item in values):
            raise ExampleError(f"{context}.{field} contains an invalid name")
    build = _load_build_configuration(table["build"], example_root, f"{context}.build")
    run = _load_run_configuration(table["run"], example_root, f"{context}.run")
    if run.adapter == "executable" and (build.adapter != "cmake" or run.target not in build.targets):
        raise ExampleError(f"{context}.run.target must name a declared CMake build target")
    requirements = _load_requirements(table["requirements"], f"{context}.requirements")
    _validate_requirement_coherence(build, run, requirements, context)
    tests = _load_test_configurations(table.get("test"), example_root, run, f"{context}.test")
    return _Example(
        id=example_id,
        title=title,
        summary=summary,
        owners=owners,
        categories=categories,
        topics=topics,
        root=example_root,
        build=build,
        run=run,
        requirements=requirements,
        tests=tests,
    )


def _load_series(
    series_root: Path,
    examples_by_id: dict[str, _Example],
) -> _Series:
    """Load and validate a series manifest and its co-located steps.

    Args:
        series_root: Root containing the series manifest and steps.
        examples_by_id: Examples indexed by stable ID.

    Returns:
        Validated series definition.
    """
    manifest_path = series_root / "series.toml"
    if not (series_root / "README.md").is_file():
        raise ExampleError(f"Series root has no README.md: {series_root}")
    table = _read_toml(manifest_path)
    context = str(manifest_path)
    _validate_keys(table, {"id", "kind", "step", "summary", "title"}, set(), context)
    series_id = _read_string(table, "id", context)
    if EXAMPLE_ID_PATTERN.fullmatch(series_id) is None:
        raise ExampleError(f"{context}.id is invalid: {series_id}")
    title = _read_string(table, "title", context)
    summary = _read_string(table, "summary", context)
    kind = _read_string(table, "kind", context)
    if kind not in ALLOWED_SERIES_KINDS:
        raise ExampleError(f"{context}.kind is unsupported: {kind}")
    steps = table["step"]
    if not isinstance(steps, list) or not steps:
        raise ExampleError(f"{context}.step must be a non-empty array of tables")

    step_ids: set[str] = set()
    validated_steps: list[_SeriesStep] = []
    for index, step in enumerate(steps):
        step_context = f"{context}.step[{index}]"
        if not isinstance(step, dict):
            raise ExampleError(f"{step_context} must be a table")
        _validate_keys(step, {"example", "level"}, set(), step_context)
        example_id = _read_string(step, "example", step_context)
        level = _read_string(step, "level", step_context)
        if example_id in step_ids:
            raise ExampleError(f"{context} contains duplicate step: {example_id}")
        if example_id not in examples_by_id:
            raise ExampleError(f"{step_context}.example is not published: {example_id}")
        if level not in ALLOWED_LEVELS:
            raise ExampleError(f"{step_context}.level is unsupported: {level}")
        if not examples_by_id[example_id].root.is_relative_to(series_root):
            raise ExampleError(f"{step_context}.example must be physically located under {series_root}")
        step_ids.add(example_id)
        validated_steps.append(_SeriesStep(example_id=example_id, level=level))

    co_located_ids = {example.id for example in examples_by_id.values() if example.root.is_relative_to(series_root)}
    if co_located_ids != step_ids:
        missing = sorted(co_located_ids - step_ids)
        raise ExampleError(f"{context} does not list its co-located examples: {', '.join(missing)}")

    return _Series(
        id=series_id,
        title=title,
        summary=summary,
        kind=kind,
        root=series_root,
        steps=tuple(validated_steps),
    )


def _validate_source_tree(examples_dir: Path) -> None:
    """Reject source symlinks and generated output.

    Args:
        examples_dir: Root of the examples collection.
    """
    for path in examples_dir.rglob("*"):
        if path.is_symlink():
            raise ExampleError(f"Example source must not contain symlinks: {path}")
        if path.is_dir() and path.name in GENERATED_DIRECTORY_NAMES:
            raise ExampleError(f"Example source contains generated output: {path}")
        if path.is_file() and (path.name in GENERATED_FILE_NAMES or path.suffix.lower() in GENERATED_FILE_SUFFIXES):
            raise ExampleError(f"Example source contains generated output: {path}")


def _validate_build_root(build_root: Path, examples_dir: Path) -> None:
    """Require generated build output to remain outside source/examples.

    Args:
        build_root: Requested root for isolated builds.
        examples_dir: Root of the canonical or materialized examples collection.
    """
    if build_root == examples_dir or build_root.is_relative_to(examples_dir):
        raise ExampleError(f"Build root must remain outside source/examples: {build_root}")


def _discover_published_example_roots(examples_dir: Path) -> tuple[Path, ...]:
    """Discover example roots beneath the examples collection.

    Args:
        examples_dir: Root of the examples collection.

    Returns:
        Example roots in deterministic path order.
    """
    examples_root = examples_dir.resolve()
    roots: list[Path] = []
    for manifest_path in sorted(examples_dir.rglob("example.toml")):
        table = _read_toml(manifest_path)
        context = str(manifest_path)
        published = table.get("published", True)
        if type(published) is not bool:
            raise ExampleError(f"{context}.published must be a boolean")
        if not published:
            continue
        root = manifest_path.parent.resolve()
        relative_parts = root.relative_to(examples_root).parts
        if not relative_parts or relative_parts[0] not in ALLOWED_EXAMPLE_GROUPS:
            raise ExampleError(f"Example must start with libraries or series: {root}")
        roots.append(root)
    return tuple(roots)


def _load_collection(examples_dir: Path) -> tuple[tuple[_Example, ...], tuple[_Series, ...]]:
    """Load and validate every example and series.

    Args:
        examples_dir: Root of the examples collection.

    Returns:
        Examples and series in deterministic path order.
    """
    _validate_source_tree(examples_dir)
    examples = tuple(_load_example(root) for root in _discover_published_example_roots(examples_dir))
    examples_by_id = {example.id: example for example in examples}
    if len(examples_by_id) != len(examples):
        raise ExampleError("Examples contain duplicate stable example IDs")
    # Reject collection-wide pin conflicts before loading dependent series metadata.
    _collect_python_packages(examples)
    _collect_cpp_packages(examples)

    resolved_series_roots: list[Path] = []
    series_ids: set[str] = set()
    series: list[_Series] = []
    for manifest_path in sorted(examples_dir.rglob("series.toml")):
        path = manifest_path.parent.resolve()
        if not any(example.root.is_relative_to(path) for example in examples):
            continue
        relative_parts = path.relative_to(examples_dir.resolve()).parts
        if len(relative_parts) != 2 or relative_parts[0] != "series":
            raise ExampleError(f"Series root must be located at series/<series-name>: {path}")
        series_entry = _load_series(path, examples_by_id)
        if series_entry.id in series_ids:
            raise ExampleError(f"Series contain duplicate stable series ID: {series_entry.id}")
        series_ids.add(series_entry.id)
        series.append(series_entry)
        resolved_series_roots.append(path)

    for example in examples:
        if example.root.relative_to(examples_dir.resolve()).parts[0] == "series" and not any(
            example.root.is_relative_to(series_root) for series_root in resolved_series_roots
        ):
            raise ExampleError(f"Example under series is not declared by a series manifest: {example.root}")
    return examples, tuple(series)


def _load_examples(examples_dir: Path) -> tuple[_Example, ...]:
    """Load and validate every example.

    Args:
        examples_dir: Root directory of the examples collection.

    Returns:
        The resulting value.
    """
    examples, _ = _load_collection(examples_dir)
    return examples


def _load_library_catalog(path: Path) -> tuple[str, set[str], set[str]]:
    """Load the release version and valid public library identities.

    Args:
        path: Filesystem path to process.

    Returns:
        The resulting value.
    """
    try:
        document = json.loads(path.read_text(encoding="utf-8"))
        if not isinstance(document, dict) or document.get("schema_version") != 1:
            raise ExampleError(f"Unsupported library documentation catalog schema: {path}")
        release_version = document["release_version"]
        if not isinstance(release_version, str) or not release_version:
            raise TypeError("release_version must be a non-empty string")
        distributions = document["distributions"]
        if not isinstance(distributions, list):
            raise TypeError("distributions must be a list")
        published_distributions = [entry for entry in distributions if entry.get("complete", False)]
        distribution_names = {entry["name"] for entry in published_distributions}
        module_names = {module["name"] for entry in published_distributions for module in entry["modules"]}
    except (AttributeError, KeyError, OSError, TypeError, json.JSONDecodeError) as error:
        raise ExampleError(f"Cannot read library documentation catalog {path}: {error}") from error
    return release_version, distribution_names, module_names


def _catalog_languages(example: _Example) -> list[str]:
    """Derive documented languages from structured build and run adapters.

    Args:
        example: Example metadata entry.

    Returns:
        The resulting value.
    """
    languages: list[str] = []
    system = set(example.requirements.system)
    for language in ("c", "cpp"):
        if language in system:
            languages.append(language)
    if example.run.adapter == "python":
        languages.append("python")
    return languages


def _format_examples_table(examples: tuple[_Example, ...]) -> str:
    """Format examples as an aligned table.

    Args:
        examples: Examples to include.

    Returns:
        Table containing each example's stable ID and title.
    """
    id_header = "ID"
    title_header = "TITLE"
    id_width = max([len(id_header), *(len(example.id) for example in examples)])
    title_width = max([len(title_header), *(len(example.title) for example in examples)])
    lines = [
        f"{id_header:<{id_width}} | {title_header}",
        f"{'-' * id_width}-+-{'-' * title_width}",
    ]
    lines.extend(f"{example.id:<{id_width}} | {example.title}" for example in examples)
    return "\n".join(lines)


def _build_catalog(examples_dir: Path, library_catalog: Path) -> dict[str, Any]:
    """Build deterministic documentation data from the runner's validated manifests.

    Args:
        examples_dir: Root directory of the examples collection.
        library_catalog: Library catalog metadata.

    Returns:
        The resulting value.
    """
    examples, series = _load_collection(examples_dir)
    release_version, distribution_names, module_names = _load_library_catalog(library_catalog)
    series_membership: dict[str, tuple[str, int, str]] = {}
    serialized_series: list[dict[str, Any]] = []
    for series_entry in series:
        steps = [step.example_id for step in series_entry.steps]
        for position, step in enumerate(series_entry.steps, start=1):
            series_membership[step.example_id] = (series_entry.id, position, step.level)
        serialized_series.append(
            {
                "id": series_entry.id,
                "title": series_entry.title,
                "summary": series_entry.summary,
                "kind": series_entry.kind,
                "readme_path": (series_entry.root / "README.md").relative_to(examples_dir).as_posix(),
                "steps": steps,
            }
        )

    serialized_examples: list[dict[str, Any]] = []
    for example in sorted(examples, key=lambda item: item.id):
        requirements = [requirement.name for requirement in example.requirements.modules]
        unknown_distributions = sorted(set(requirements) - distribution_names)
        unknown_owners = sorted(set(example.owners) - module_names)
        if unknown_distributions:
            raise ExampleError(
                f"Example {example.id} requires unknown distributions: {', '.join(unknown_distributions)}"
            )
        if unknown_owners:
            raise ExampleError(f"Example {example.id} has unknown public API owners: {', '.join(unknown_owners)}")
        relative_root = example.root.relative_to(examples_dir).as_posix()
        relative_parts = example.root.relative_to(examples_dir).parts
        group = {"libraries": "library", "series": "series"}[relative_parts[0]]
        membership = series_membership.get(example.id)
        entry_point = (
            example.run.path.relative_to(example.root).as_posix()
            if example.run.path is not None
            else str(example.run.target)
        )
        serialized_examples.append(
            {
                "id": example.id,
                "title": example.title,
                "summary": example.summary,
                "source_path": relative_root,
                "readme_path": f"{relative_root}/README.md",
                "owners": list(example.owners),
                "topics": list(example.topics),
                "categories": list(example.categories),
                "group": group,
                "requirements": {
                    "distributions": requirements,
                    "python_packages": [requirement.specifier for requirement in example.requirements.python_packages],
                    "cpp_packages": [requirement.specifier for requirement in example.requirements.cpp_packages],
                    "capabilities": list(example.requirements.system),
                },
                "entry_point": entry_point,
                "languages": _catalog_languages(example),
                "commands": {
                    "build": f"python source/examples/examples.py build {example.id}",
                    "run": f"python source/examples/examples.py run {example.id}",
                    "test": f"python source/examples/examples.py test {example.id}",
                },
                "series_id": membership[0] if membership else None,
                "series_position": membership[1] if membership else None,
                "level": membership[2] if membership else None,
            }
        )
    return {
        "schema_version": 4,
        "release_version": release_version,
        "examples": serialized_examples,
        "series": serialized_series,
    }


def _render_command(command: list[str]) -> str:
    """Render a command for user-facing output.

    Args:
        command: Command and arguments.

    Returns:
        Platform-appropriate command string.
    """
    return subprocess.list2cmdline(command) if os.name == "nt" else shlex.join(command)


def _run_checked(command: list[str], working_directory: Path, environment: dict[str, str]) -> None:
    """Run a build command and raise an actionable error when it fails.

    Args:
        command: Command and arguments.
        working_directory: Directory in which to run the command.
        environment: Process environment.
    """
    print(f"Running: {_render_command(command)}")
    result = subprocess.run(command, cwd=working_directory, env=environment, check=False)
    if result.returncode != 0:
        raise ExampleError(f"Command failed with exit code {result.returncode}: {_render_command(command)}")


def _collect_python_packages(examples: tuple[_Example, ...]) -> tuple[_PythonPackageRequirement, ...]:
    """Collect the globally compatible external requirements for an examples payload.

    Args:
        examples: Examples sharing one external dependency directory.

    Returns:
        Combined external package requirements in deterministic name order.
    """
    packages: dict[str, tuple[str, str]] = {}
    for example in examples:
        for requirement in example.requirements.python_packages:
            previous = packages.get(requirement.name)
            candidate = (requirement.version, example.id)
            if previous is not None:
                if Version(previous[0]) != Version(requirement.version):
                    raise ExampleError(
                        f"Examples have conflicting Python package requirements for {requirement.name}: "
                        f"{previous[1]} requires {previous[0]}, but {example.id} requires {requirement.version}"
                    )
                candidate = min(previous, candidate)
            packages[requirement.name] = candidate
    return tuple(
        _PythonPackageRequirement(name=name, version=version) for name, (version, _) in sorted(packages.items())
    )


def _collect_cpp_packages(examples: tuple[_Example, ...]) -> tuple[_CppPackageRequirement, ...]:
    """Collect the globally compatible external C++ requirements for an examples payload.

    Args:
        examples: Examples sharing one external dependency directory.

    Returns:
        Combined external package requirements in deterministic name order.
    """
    packages: dict[str, tuple[str, str]] = {}
    for example in examples:
        for requirement in example.requirements.cpp_packages:
            previous = packages.get(requirement.name)
            if previous is not None and previous[0] != requirement.version:
                raise ExampleError(
                    f"Examples have conflicting C++ package requirements for {requirement.name}: "
                    f"{previous[1]} requires {previous[0]}, but {example.id} requires {requirement.version}"
                )
            packages[requirement.name] = (requirement.version, example.id)
    return tuple(_CppPackageRequirement(name=name, version=version) for name, (version, _) in sorted(packages.items()))


def _python_dependency_state(requirements: tuple[_PythonPackageRequirement, ...]) -> dict[str, Any]:
    """Return the marker for the shared dependency directory.

    Args:
        requirements: External package requirements represented by the marker.

    Returns:
        Runtime and requirement state used to validate the directory.
    """
    python_tag = next(sys_tags())
    return {
        "schema_version": 1,
        "requirements": [requirement.specifier for requirement in requirements],
        "python_implementation": sys.implementation.name,
        "python_version": f"{sys.version_info.major}.{sys.version_info.minor}",
        "python_abi": python_tag.abi,
        "platform": sysconfig.get_platform(),
    }


def _python_dependency_path(build_root: Path) -> Path:
    """Return the shared dependency directory for all examples.

    Args:
        build_root: Shared examples build root.

    Returns:
        Path to the external dependency directory.
    """
    return build_root / PYTHON_DEPENDENCY_DIRECTORY


def _python_dependency_lock(build_root: Path, *, exclusive: bool) -> contextlib.AbstractContextManager[None]:
    """Coordinate dependency publishers and command-lifetime consumers.

    Args:
        build_root: Shared examples build root.
        exclusive: Whether to acquire an exclusive lock.

    Returns:
        Context manager holding the requested lock.
    """
    return _path_lock(build_root / f".{PYTHON_DEPENDENCY_DIRECTORY}.lock", exclusive=exclusive)


def _is_redirected_directory(path: Path) -> bool:
    """Return whether a path is a symlink or Windows junction.

    Args:
        path: Path to inspect.

    Returns:
        Whether the path redirects to another filesystem location.
    """
    is_junction = getattr(path, "is_junction", None)
    return path.is_symlink() or (is_junction is not None and is_junction())


def _dependency_state_matches(
    dependency_dir: Path,
    expected_state: dict[str, Any],
) -> bool:
    """Check whether a published dependency directory has the expected state.

    Args:
        dependency_dir: Prepared dependency directory.
        expected_state: Marker selected for the current invocation.

    Returns:
        Whether the directory is safe and matches the expected state.
    """
    if not dependency_dir.is_dir() or _is_redirected_directory(dependency_dir):
        return False
    try:
        state = json.loads((dependency_dir / DEPENDENCY_STATE_FILE).read_text(encoding="utf-8"))
        return state == expected_state
    except (OSError, TypeError, json.JSONDecodeError):
        return False


def _validate_external_distributions(dependency_dir: Path) -> None:
    """Keep Isaac and conflicting ambient distributions out of the overlay.

    Args:
        dependency_dir: External dependency directory to validate.
    """
    ambient: dict[str, str] = {}
    for distribution in importlib.metadata.distributions(path=_ambient_python_paths()):
        ambient.setdefault(canonicalize_name(distribution.metadata.get("Name", "")), distribution.version)
    for distribution in importlib.metadata.distributions(path=[str(dependency_dir)]):
        name = canonicalize_name(distribution.metadata.get("Name", ""))
        if name == "isaacsim" or name.startswith("isaacsim-"):
            raise ExampleError(f"External Python dependency closure contains Isaac Sim distribution: {name}")
        if name in ambient and Version(distribution.version) != Version(ambient[name]):
            raise ExampleError(
                f"External Python dependency {name}=={distribution.version} does not match {name}=={ambient[name]} "
                "provided by the active Isaac Sim environment. Align the relevant example.toml requirement or its "
                "parent dependency with the Isaac Sim version."
            )


def _validate_installed_python_dependencies(
    requirements: tuple[_PythonPackageRequirement, ...], dependency_dir: Path
) -> None:
    """Verify that pip installed every declared package at its exact version.

    Args:
        requirements: Expected external package requirements.
        dependency_dir: Directory containing the installed packages.
    """
    installed: dict[str, str] = {
        canonicalize_name(distribution.metadata.get("Name", "")): distribution.version
        for distribution in importlib.metadata.distributions(path=[str(dependency_dir)])
    }
    for requirement in requirements:
        version = installed.get(requirement.name)
        try:
            matches = version is not None and Version(version) == Version(requirement.version)
        except InvalidVersion:
            matches = False
        if not matches:
            found = "missing" if version is None else f"version {version}"
            raise ExampleError(
                f"Python dependency installation did not produce {requirement.specifier} in {dependency_dir} "
                f"(found {found}); check the pip configuration"
            )


def _ambient_python_paths() -> list[str]:
    """Return the runner search path excluding user sites disabled for children.

    Returns:
        Search paths visible to example child processes.
    """
    configured_user_sites = site.getusersitepackages()
    user_sites = [configured_user_sites] if isinstance(configured_user_sites, str) else configured_user_sites
    excluded = {os.path.normcase(os.path.abspath(path)) for path in user_sites}
    explicit_paths = [path for path in os.environ.get("PYTHONPATH", "").split(os.pathsep) if path]
    if any(not Path(path).is_absolute() for path in explicit_paths):
        raise ExampleError("PYTHONPATH entries must be absolute when examples use external Python packages")
    explicit = {os.path.normcase(os.path.abspath(path)) for path in explicit_paths}
    return [
        path
        for path in sys.path
        if path
        and (
            os.path.normcase(os.path.abspath(path)) not in excluded
            or os.path.normcase(os.path.abspath(path)) in explicit
        )
    ]


def _remove_dependency_tree(dependency_dir: Path) -> None:
    """Remove a dependency tree without following a redirected root.

    Args:
        dependency_dir: Dependency tree to remove.
    """
    if dependency_dir.is_symlink():
        dependency_dir.unlink()
        return
    is_junction = getattr(dependency_dir, "is_junction", None)
    if is_junction is not None and is_junction():
        os.rmdir(dependency_dir)
        return
    if not dependency_dir.is_dir():
        dependency_dir.unlink()
        return

    def make_writable(function: Any, path: str, _: Any) -> None:
        target = Path(path)
        if not _is_redirected_directory(target):
            target.chmod(stat.S_IMODE(target.stat().st_mode) | stat.S_IWUSR)
        target.parent.chmod(stat.S_IMODE(target.parent.stat().st_mode) | stat.S_IWUSR)
        function(path)

    shutil.rmtree(dependency_dir, onerror=make_writable)


def _install_python_dependencies(
    requirements: tuple[_PythonPackageRequirement, ...],
    destination: Path,
    working_directory: Path,
    environment: dict[str, str],
) -> None:
    """Install external requirements into one unpublished dependency directory.

    Args:
        requirements: Combined external requirements for the examples payload.
        destination: Unpublished target directory.
        working_directory: Directory in which to invoke pip.
        environment: Pip process environment.
    """
    _run_checked(
        [
            sys.executable,
            "-m",
            "pip",
            "install",
            "--disable-pip-version-check",
            "--ignore-installed",
            "--no-compile",
            "--no-input",
            "--target",
            str(destination),
            *(requirement.specifier for requirement in requirements),
        ],
        working_directory,
        environment,
    )


def _prepare_python_dependencies(examples: tuple[_Example, ...], build_root: Path) -> Path | None:
    """Prepare and validate the shared external Python dependencies.

    Args:
        examples: Complete examples payload sharing the dependency directory.
        build_root: Shared examples build root.

    Returns:
        Shared dependency directory, or None when the payload declares no packages.
    """
    requirements = _collect_python_packages(examples)
    if not requirements:
        return None

    if importlib.util.find_spec("pip") is None:
        raise ExampleError(f"Preparing shared Python dependencies requires pip for {sys.executable}")

    dependency_dir = _python_dependency_path(build_root)
    build_root.mkdir(parents=True, exist_ok=True)
    staging_dir = Path(tempfile.mkdtemp(prefix=f".{PYTHON_DEPENDENCY_DIRECTORY}-", dir=build_root))
    backup_dir = staging_dir.with_name(f"{staging_dir.name}-backup")
    environment = os.environ.copy()
    environment["PYTHONDONTWRITEBYTECODE"] = "1"
    environment["PYTHONNOUSERSITE"] = "1"
    install_environment = environment.copy()
    install_environment.pop("PYTHONPATH", None)
    try:
        _install_python_dependencies(requirements, staging_dir, build_root, install_environment)
        _validate_installed_python_dependencies(requirements, staging_dir)
        _validate_external_distributions(staging_dir)
        check_environment = environment.copy()
        _prepend_environment_path(check_environment, "PYTHONPATH", [staging_dir])
        _run_checked([sys.executable, "-m", "pip", "check"], build_root, check_environment)
        state = _python_dependency_state(requirements)
        (staging_dir / DEPENDENCY_STATE_FILE).write_text(
            json.dumps(state, indent=2, sort_keys=True) + "\n", encoding="utf-8"
        )
        try:
            if dependency_dir.exists() or _is_redirected_directory(dependency_dir):
                dependency_dir.rename(backup_dir)
            staging_dir.rename(dependency_dir)
        except BaseException:
            if backup_dir.exists() or _is_redirected_directory(backup_dir):
                if dependency_dir.exists() or _is_redirected_directory(dependency_dir):
                    _remove_dependency_tree(dependency_dir)
                backup_dir.rename(dependency_dir)
            raise
        if backup_dir.exists() or _is_redirected_directory(backup_dir):
            _remove_dependency_tree(backup_dir)
        return dependency_dir
    except (OSError, subprocess.SubprocessError) as error:
        raise ExampleError(f"Cannot prepare shared Python dependencies: {error}") from error
    finally:
        if staging_dir.exists():
            _remove_dependency_tree(staging_dir)


def _require_python_dependencies(examples: tuple[_Example, ...], build_root: Path) -> Path | None:
    """Return an existing valid dependency target without installing packages.

    Args:
        examples: Examples sharing one external dependency directory.
        build_root: Shared examples build root.

    Returns:
        Valid dependency directory, or None when no external packages are required.
    """
    requirements = _collect_python_packages(examples)
    if not requirements:
        return None
    dependency_dir = _python_dependency_path(build_root)
    if not _dependency_state_matches(dependency_dir, _python_dependency_state(requirements)):
        raise ExampleError("Shared Python dependencies are not built; run the examples build command first")
    try:
        _validate_installed_python_dependencies(requirements, dependency_dir)
    except ExampleError as error:
        raise ExampleError("Shared Python dependencies are invalid; run the examples build command again") from error
    _validate_external_distributions(dependency_dir)
    return dependency_dir


def _cpp_dependency_platform() -> str:
    """Return the Pixi platform supported by the current examples runner.

    Returns:
        Pixi platform name for the current host.
    """
    platform_name = sysconfig.get_platform().lower().replace("_", "-")
    platforms = {
        "linux-aarch64": "linux-aarch64",
        "linux-x86-64": "linux-64",
        "win-amd64": "win-64",
    }
    try:
        return platforms[platform_name]
    except KeyError as error:
        raise ExampleError(f"External C++ packages are unsupported on platform {platform_name}") from error


def _find_pixi() -> Path:
    """Return the Pixi executable supplied by the launcher or caller.

    Returns:
        Resolved path to the Pixi executable.
    """
    configured = os.environ.get("PIXI_EXE")
    executable = Path(configured).resolve() if configured else None
    if executable is None:
        discovered = shutil.which("pixi")
        executable = Path(discovered).resolve() if discovered else None
    if executable is None or not executable.is_file():
        raise ExampleError("External C++ packages require Pixi through PIXI_EXE or PATH")
    return executable


def _scrub_pixi_update_environment(environment: dict[str, str]) -> None:
    """Remove inherited options that can override the runner's update policy.

    Args:
        environment: Process environment to update.
    """
    update_names = {"PIXI_FROZEN", "PIXI_LOCKED", "PIXI_NO_INSTALL"}
    for name in tuple(environment):
        if name.upper() in update_names:
            environment.pop(name)


def _cpp_dependency_manifest(requirements: tuple[_CppPackageRequirement, ...]) -> str:
    """Render the build-local Pixi manifest for the shared C++ requirements.

    Args:
        requirements: External C++ package requirements.

    Returns:
        Pixi manifest contents.
    """
    dependencies = "".join(
        f"{json.dumps(requirement.name)} = {json.dumps(f'=={requirement.version}')}\n" for requirement in requirements
    )
    return (
        "[workspace]\n"
        'channels = ["conda-forge"]\n'
        f"platforms = [{json.dumps(_cpp_dependency_platform())}]\n\n"
        "[dependencies]\n"
        f"{dependencies}"
    )


def _cpp_dependency_state(requirements: tuple[_CppPackageRequirement, ...]) -> dict[str, Any]:
    """Return the marker for a successfully installed Pixi workspace.

    Args:
        requirements: External C++ package requirements represented by the marker.

    Returns:
        Manifest state used to validate the workspace.
    """
    return {
        "schema_version": 1,
        "manifest": _cpp_dependency_manifest(requirements),
    }


def _cpp_dependency_lock(build_root: Path, *, exclusive: bool) -> contextlib.AbstractContextManager[None]:
    """Coordinate C++ dependency publishers and command-lifetime consumers.

    Args:
        build_root: Shared examples build root.
        exclusive: Whether to acquire an exclusive lock.

    Returns:
        Context manager holding the requested lock.
    """
    return _path_lock(build_root / f".{CPP_DEPENDENCY_DIRECTORY}.lock", exclusive=exclusive)


def _prepare_cpp_dependencies(examples: tuple[_Example, ...], build_root: Path) -> _CppDependencyEnvironment | None:
    """Resolve and install the shared external C++ dependencies with Pixi.

    Args:
        examples: Complete examples payload sharing the dependency workspace.
        build_root: Shared examples build root.

    Returns:
        Prepared dependency environment, or None when the payload declares no packages.
    """
    requirements = _collect_cpp_packages(examples)
    if not requirements:
        return None
    pixi = _find_pixi()
    workspace = build_root / CPP_DEPENDENCY_DIRECTORY
    if _is_redirected_directory(workspace):
        raise ExampleError(f"Shared C++ dependency workspace is redirected: {workspace}")
    workspace.mkdir(parents=True, exist_ok=True)
    manifest = workspace / "pixi.toml"
    state_file = workspace / DEPENDENCY_STATE_FILE
    state_file.unlink(missing_ok=True)
    manifest.write_text(_cpp_dependency_manifest(requirements), encoding="utf-8")
    environment = os.environ.copy()
    _scrub_pixi_update_environment(environment)
    _run_checked(
        [
            str(pixi),
            "install",
            "--manifest-path",
            str(manifest),
            "--no-config",
            "--tls-root-certs",
            "webpki",
        ],
        workspace,
        environment,
    )
    prefix = workspace / ".pixi" / "envs" / "default"
    if not (workspace / "pixi.lock").is_file() or not prefix.is_dir():
        raise ExampleError(f"Pixi did not create the shared C++ dependency environment: {workspace}")
    state_file.write_text(
        json.dumps(_cpp_dependency_state(requirements), indent=2, sort_keys=True) + "\n", encoding="utf-8"
    )
    return _CppDependencyEnvironment(pixi=pixi, manifest=manifest)


def _require_cpp_dependencies(examples: tuple[_Example, ...], build_root: Path) -> _CppDependencyEnvironment | None:
    """Return the existing C++ dependency workspace without installing packages.

    Args:
        examples: Examples sharing one external dependency workspace.
        build_root: Shared examples build root.

    Returns:
        Existing dependency environment, or None when no external packages are required.
    """
    requirements = _collect_cpp_packages(examples)
    if not requirements:
        return None
    workspace = build_root / CPP_DEPENDENCY_DIRECTORY
    manifest = workspace / "pixi.toml"
    prefix = workspace / ".pixi" / "envs" / "default"
    try:
        state_matches = (
            _dependency_state_matches(workspace, _cpp_dependency_state(requirements))
            and manifest.read_text(encoding="utf-8") == _cpp_dependency_manifest(requirements)
            and (workspace / "pixi.lock").is_file()
            and prefix.is_dir()
        )
    except OSError:
        state_matches = False
    if not state_matches:
        raise ExampleError("Shared C++ dependencies are not built; run the examples build command first")
    return _CppDependencyEnvironment(pixi=_find_pixi(), manifest=manifest)


def _wrap_cpp_dependency_command(
    example: _Example, dependencies: _CppDependencyEnvironment | None, command: list[str]
) -> list[str]:
    """Wrap a C++ dependency consumer for the existing Pixi environment.

    Args:
        example: Example that consumes the command.
        dependencies: Prepared external C++ dependency environment.
        command: Command and arguments to wrap.

    Returns:
        Original command or a Pixi-wrapped command when dependencies are required.
    """
    if not example.requirements.cpp_packages:
        return command
    if dependencies is None:
        raise ExampleError(f"Example {example.id} requires an unavailable C++ dependency environment")
    return [
        str(dependencies.pixi),
        "run",
        "--manifest-path",
        str(dependencies.manifest),
        "--no-config",
        "--no-install",
        "--locked",
        "--",
        *command,
    ]


def _restart_in_environment(executable: Path, environment: dict[str, str]) -> int:
    """Restart this command with a prepared execution environment.

    Run the Windows replacement as a child so the invoking terminal waits for
    the example and receives its exit code. Unix can replace the current
    process directly.

    Args:
        executable: Python executable used for the restarted command.
        environment: Complete environment for the restarted command.

    Returns:
        Exit code from the restarted command on Windows. The Unix path replaces
        the current process and does not return.
    """
    command = [str(executable), str(Path(__file__).resolve()), *sys.argv[1:]]
    if os.name == "nt":
        return subprocess.run(command, env=environment, check=False).returncode
    os.execve(executable, command, environment)


def _build_example(
    example: _Example,
    python_dependency_dir: Path | None,
    cpp_dependencies: _CppDependencyEnvironment | None,
    system_requirements: _SystemRequirements,
    build_root: Path,
    cmake: str,
    config: str,
    generator: str | None,
    generator_platform: str | None,
    generator_toolset: str | None,
    make_program: str | None,
) -> Path:
    """Build one example incrementally and return its build directory.

    Args:
        example: Example to build.
        python_dependency_dir: Shared external Python dependency directory.
        cpp_dependencies: Shared external C++ dependency environment.
        system_requirements: Release-wide system toolchain requirements.
        build_root: Root for isolated example build directories.
        cmake: CMake executable.
        config: Build configuration.
        generator: Optional CMake generator.
        generator_platform: Optional generator platform.
        generator_toolset: Optional generator toolset.
        make_program: Optional native build program.

    Returns:
        Isolated example build directory.
    """
    build_dir = build_root / example.id
    if example.build.adapter == "none":
        print(f"Build {example.id}: no build required")
        return build_dir

    _initialize_windows_native_toolchain()
    environment = os.environ.copy()
    if example.requirements.cpp_packages:
        _scrub_pixi_update_environment(environment)
    if python_dependency_dir is not None:
        _prepend_environment_path(environment, "PYTHONPATH", [python_dependency_dir])
    configure_command = [
        cmake,
        "-S",
        str(example.root),
        "-B",
        str(build_dir),
        f"-DCMAKE_BUILD_TYPE={config}",
        f"-DISAACSIM_CMAKE_MINIMUM_VERSION={system_requirements.cmake_minimum_version}",
        f"-DISAACSIM_CMAKE_POLICY_MAXIMUM_VERSION={system_requirements.cmake_policy_maximum_version}",
        f"-DISAACSIM_C_STANDARD={system_requirements.c_standard}",
        f"-DISAACSIM_CXX_STANDARD={system_requirements.cpp_standard}",
        f"-DISAACSIM_PYTHON_VERSION={system_requirements.python_version}",
    ]
    if "python" in example.requirements.system or "python_development" in example.requirements.system:
        configure_command.append(f"-DPython_EXECUTABLE={sys.executable}")
    if generator:
        configure_command.extend(["-G", generator])
    if generator_platform:
        configure_command.extend(["-A", generator_platform])
    if generator_toolset:
        configure_command.extend(["-T", generator_toolset])
    if make_program:
        configure_command.append(f"-DCMAKE_MAKE_PROGRAM={make_program}")
    print(f"Build {example.id}")
    _run_checked(_wrap_cpp_dependency_command(example, cpp_dependencies, configure_command), example.root, environment)
    _run_checked(
        _wrap_cpp_dependency_command(
            example,
            cpp_dependencies,
            [
                cmake,
                "--build",
                str(build_dir),
                "--config",
                config,
                "--target",
                *example.build.targets,
                "--parallel",
            ],
        ),
        example.root,
        environment,
    )
    return build_dir


def _prepend_environment_path(environment: dict[str, str], name: str, paths: list[Path]) -> None:
    """Prepend existing paths to an environment path variable.

    Args:
        environment: Environment to update.
        name: Path variable name.
        paths: Candidate paths to prepend.
    """
    values = [str(path) for path in paths if path.is_dir()]
    current = environment.get(name)
    if current:
        values.append(current)
    if values:
        environment[name] = os.pathsep.join(values)


def _prefix_native_runtime_paths(prefix: Path) -> list[Path]:
    """Return native runtime search paths rooted at an installed prefix.

    Args:
        prefix: Installed SDK or developer environment prefix.

    Returns:
        Native binary and OVStage plugin search paths in lookup order.
    """
    paths = [prefix / "bin"]
    if os.name == "nt":
        plugins = prefix / "bin" / "plugins"
        paths.extend((plugins, plugins / "omni.client.lib", plugins / "omni.usd_resolver"))
    return paths


def _create_developer_environment(
    developer_build: _DeveloperBuild,
) -> dict[str, str]:
    """Create the isolated environment for developer example execution.

    Args:
        developer_build: Configured library build.

    Returns:
        Environment configured for the build's tools and installed surfaces.
    """
    prefix = developer_build.developer_environment
    environment = os.environ.copy()
    environment[DEVELOPER_ENVIRONMENT_VARIABLE] = str(developer_build.library_build_dir.resolve())
    environment["CMAKE_COMMAND"] = str(developer_build.cmake)
    environment["CMAKE_PREFIX_PATH"] = str(prefix)
    environment["PYTHONDONTWRITEBYTECODE"] = "1"
    # Keep the developer environment isolated from incompatible Isaac Sim packages in the caller's Python path.
    environment["PYTHONPATH"] = os.pathsep.join(
        (
            str(prefix / developer_build.python_install_dir),
            str(developer_build.python_runtime_dependencies),
        )
    )
    native_runtime_paths = _prefix_native_runtime_paths(prefix)
    if os.name == "nt":
        native_runtime_paths.extend(
            (
                developer_build.python.parent,
                developer_build.python_runtime_dependencies / "usd_exchange.libs",
            )
        )
    _prepend_environment_path(environment, "PATH", native_runtime_paths)
    if os.name != "nt":
        python_library_dir = developer_build.python.parent.parent / "lib"
        _prepend_environment_path(environment, "LD_LIBRARY_PATH", [python_library_dir, prefix / "lib"])
    return environment


def _create_runtime_environment(
    example: _Example,
    build_dir: Path,
    python_dependency_dir: Path | None = None,
    overrides: dict[str, str] | None = None,
) -> dict[str, str]:
    """Create the inherited runtime environment for an example.

    Args:
        example: Example being executed.
        build_dir: Example's isolated build directory.
        python_dependency_dir: Shared external Python dependency directory.
        overrides: Example-specific environment values.

    Returns:
        Runtime process environment.
    """
    environment = os.environ.copy()
    if overrides is not None:
        environment.update(overrides)
    environment["PYTHONDONTWRITEBYTECODE"] = "1"
    environment["PYTHONNOUSERSITE"] = "1"
    prefix_paths = [Path(value) for value in environment.get("CMAKE_PREFIX_PATH", "").split(os.pathsep) if value]
    native_runtime_paths = [path for prefix in prefix_paths for path in _prefix_native_runtime_paths(prefix)]
    if os.name == "nt":
        native_runtime_paths.append(Path(sys.executable).parent)
        for python_path in environment.get("PYTHONPATH", "").split(os.pathsep):
            if python_path:
                native_runtime_paths.append(Path(python_path) / "usd_exchange.libs")
    _prepend_environment_path(environment, "PATH", native_runtime_paths)
    if os.name != "nt":
        python_library_dir = Path(sys.executable).parent.parent / "lib"
        library_paths = [python_library_dir, *(prefix / "lib" for prefix in prefix_paths)]
        _prepend_environment_path(environment, "LD_LIBRARY_PATH", library_paths)
    if example.run.adapter == "python" and example.build.adapter == "cmake":
        _prepend_environment_path(environment, "PYTHONPATH", [build_dir / "python"])
    if python_dependency_dir is not None:
        _prepend_environment_path(environment, "PYTHONPATH", [python_dependency_dir])
    return environment


def _create_run_command(
    example: _Example, build_dir: Path, arguments: tuple[str, ...], *, python_path: Path | None = None
) -> list[str]:
    """Create the command for an example run adapter.

    Args:
        example: Example being executed.
        build_dir: Example's isolated build directory.
        arguments: Arguments for the normal entry point.
        python_path: Optional test-specific Python entry point.

    Returns:
        Executable command and arguments.
    """
    if example.run.adapter == "python":
        script_path = python_path or example.run.path
        if script_path is None:
            raise ExampleError(f"Python example has no script path: {example.id}")
        return [sys.executable, str(script_path), *arguments]
    if python_path is not None:
        raise ExampleError(f"Executable example cannot use a Python test path: {example.id}")
    if example.run.target is None:
        raise ExampleError(f"Executable example has no target: {example.id}")
    executable = build_dir / "bin" / example.run.target
    if os.name == "nt":
        executable = executable.with_suffix(".exe")
    if not executable.is_file():
        raise ExampleError(f"Built executable does not exist: {executable}")
    return [str(executable), *arguments]


def _terminate_process_tree(process: subprocess.Popen[str]) -> None:
    """Terminate a timed-out process and its descendants.

    Args:
        process: Root process to terminate.
    """
    if os.name == "nt":
        subprocess.run(
            ["taskkill", "/PID", str(process.pid), "/T", "/F"],
            capture_output=True,
            check=False,
            text=True,
        )
        try:
            process.wait(timeout=PROCESS_TERMINATION_TIMEOUT_SECONDS)
        except subprocess.TimeoutExpired:
            process.kill()
            process.wait(timeout=PROCESS_TERMINATION_TIMEOUT_SECONDS)
        return

    process_group = process.pid
    try:
        os.killpg(process_group, signal.SIGTERM)
    except ProcessLookupError:
        pass
    deadline = time.monotonic() + PROCESS_TERMINATION_TIMEOUT_SECONDS
    while time.monotonic() < deadline:
        process.poll()
        try:
            os.killpg(process_group, 0)
        except ProcessLookupError:
            break
        time.sleep(0.05)
    else:
        try:
            os.killpg(process_group, signal.SIGKILL)
        except ProcessLookupError:
            pass
    try:
        process.wait(timeout=PROCESS_TERMINATION_TIMEOUT_SECONDS)
    except subprocess.TimeoutExpired:
        process.kill()
        process.wait(timeout=PROCESS_TERMINATION_TIMEOUT_SECONDS)


def _run_captured(
    command: list[str],
    working_directory: Path,
    environment: dict[str, str],
    timeout_seconds: int,
) -> _CommandResult:
    """Run and capture a command with process-tree timeout handling.

    Args:
        command: Command and arguments.
        working_directory: Directory in which to run the command.
        environment: Process environment.
        timeout_seconds: Maximum execution duration.

    Returns:
        Captured process result.
    """
    creation_flags = int(getattr(subprocess, "CREATE_NEW_PROCESS_GROUP", 0)) if os.name == "nt" else 0
    process = subprocess.Popen(
        command,
        cwd=working_directory,
        env=environment,
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        text=True,
        start_new_session=os.name != "nt",
        creationflags=creation_flags,
    )
    try:
        stdout, stderr = process.communicate(timeout=timeout_seconds)
    except subprocess.TimeoutExpired:
        _terminate_process_tree(process)
        try:
            stdout, stderr = process.communicate(timeout=PROCESS_TERMINATION_TIMEOUT_SECONDS)
        except subprocess.TimeoutExpired:
            if process.stdout is not None:
                process.stdout.close()
            if process.stderr is not None:
                process.stderr.close()
            stdout, stderr = "", "Timed-out descendants did not close their output streams."
        return _CommandResult(process.returncode, stdout, stderr, timed_out=True)
    return _CommandResult(process.returncode, stdout, stderr)


def _run_example(
    example: _Example,
    build_dir: Path,
    arguments: tuple[str, ...],
    python_dependency_dir: Path | None = None,
    cpp_dependencies: _CppDependencyEnvironment | None = None,
) -> int:
    """Run one example interactively.

    Args:
        example: Example to run.
        build_dir: Example's isolated build directory.
        arguments: Additional user arguments.
        python_dependency_dir: Shared external Python dependency directory.
        cpp_dependencies: Shared external C++ dependency environment.

    Returns:
        Example process exit code.
    """
    command = _wrap_cpp_dependency_command(
        example,
        cpp_dependencies,
        _create_run_command(example, build_dir, example.run.arguments + arguments),
    )
    environment = _create_runtime_environment(example, build_dir, python_dependency_dir)
    if example.requirements.cpp_packages:
        _scrub_pixi_update_environment(environment)
    print(f"Run {example.id}: {_render_command(command)}")
    process = subprocess.Popen(command, cwd=example.root, env=environment)
    try:
        return process.wait()
    except KeyboardInterrupt:
        try:
            process.wait(timeout=PROCESS_TERMINATION_TIMEOUT_SECONDS)
        except (KeyboardInterrupt, subprocess.TimeoutExpired):
            process.terminate()
            try:
                process.wait(timeout=PROCESS_TERMINATION_TIMEOUT_SECONDS)
            except (KeyboardInterrupt, subprocess.TimeoutExpired):
                process.kill()
                while True:
                    try:
                        process.wait()
                        break
                    except KeyboardInterrupt:
                        continue
        return 130


def _test_example(
    example: _Example,
    test: _TestConfiguration,
    build_dir: Path,
    python_dependency_dir: Path | None = None,
    cpp_dependencies: _CppDependencyEnvironment | None = None,
) -> None:
    """Run and validate one named example test.

    Args:
        example: Example to test.
        test: Named test configuration.
        build_dir: Example's isolated build directory.
        python_dependency_dir: Shared external Python dependency directory.
        cpp_dependencies: Shared external C++ dependency environment.
    """
    arguments = example.run.arguments if test.arguments is None else test.arguments
    command = _wrap_cpp_dependency_command(
        example,
        cpp_dependencies,
        _create_run_command(example, build_dir, arguments, python_path=test.path),
    )
    environment = _create_runtime_environment(example, build_dir, python_dependency_dir, test.environment)
    if example.requirements.cpp_packages:
        _scrub_pixi_update_environment(environment)
    print(f"Test {example.id}:{test.name}: {_render_command(command)}")
    result = _run_captured(command, example.root, environment, test.timeout_seconds)
    if result.stdout:
        print(result.stdout, end="" if result.stdout.endswith("\n") else "\n")
    if result.stderr:
        print(result.stderr, file=sys.stderr, end="" if result.stderr.endswith("\n") else "\n")
    if result.timed_out:
        raise ExampleError(f"Test {example.id}:{test.name} timed out after {test.timeout_seconds} seconds")
    if result.returncode != test.expected_exit_code:
        raise ExampleError(
            f"Test {example.id}:{test.name} exited with {result.returncode}, expected {test.expected_exit_code}"
        )
    for expected in test.stdout_contains:
        if expected not in result.stdout:
            raise ExampleError(f"Test {example.id}:{test.name} stdout does not contain: {expected}")
    for expected in test.stderr_contains:
        if expected not in result.stderr:
            raise ExampleError(f"Test {example.id}:{test.name} stderr does not contain: {expected}")
    print(f"Test {example.id}:{test.name}: passed")


def _select_examples(examples: tuple[_Example, ...], selectors: list[str]) -> tuple[_Example, ...]:
    """Select examples by stable ID while preserving discovery order.

    Args:
        examples: Complete example collection.
        selectors: Stable IDs to select.

    Returns:
        Selected examples in discovery order.
    """
    examples_by_id = {example.id: example for example in examples}
    if not selectors:
        return examples
    if len(selectors) != len(set(selectors)):
        raise ExampleError("Example selectors must not contain duplicates")
    unknown = set(selectors) - examples_by_id.keys()
    if unknown:
        raise ExampleError(f"Unknown example ID: {', '.join(sorted(unknown))}")
    selected_ids = set(selectors)
    return tuple(example for example in examples if example.id in selected_ids)


def _select_run_example(examples: tuple[_Example, ...], selector: str, examples_dir: Path) -> _Example:
    """Select one example by stable ID, root path, or declared Python script.

    Args:
        examples: Complete example collection.
        selector: Stable ID or path relative to the examples collection.
        examples_dir: Root of the examples collection.

    Returns:
        Selected example.
    """
    examples_by_id = {example.id: example for example in examples}
    if selector in examples_by_id:
        return examples_by_id[selector]

    selector_path = Path(selector)
    if selector_path.is_absolute():
        raise ExampleError(f"Example path must be relative to source/examples: {selector}")
    examples_root = examples_dir.resolve()
    candidate = (examples_root / selector_path).resolve()
    if not candidate.is_relative_to(examples_root):
        raise ExampleError(f"Example path escapes source/examples: {selector}")
    for example in examples:
        if candidate == example.root or candidate == example.run.path:
            return example
    raise ExampleError(f"Unknown example ID or run path: {selector}")


def _select_tests(
    examples: tuple[_Example, ...],
    selectors: list[str],
) -> tuple[tuple[_Example, _TestConfiguration], ...]:
    """Select all or named test configurations.

    Args:
        examples: Complete example collection.
        selectors: Example or named-test selectors.

    Returns:
        Selected example and test pairs.
    """
    examples_by_id = {example.id: example for example in examples}
    if not selectors:
        return tuple((example, test) for example in examples for test in example.tests)

    selected: list[tuple[_Example, _TestConfiguration]] = []
    selected_keys: set[tuple[str, str]] = set()
    for selector in selectors:
        example_id, separator, test_name = selector.partition(":")
        if example_id not in examples_by_id:
            raise ExampleError(f"Unknown example ID: {example_id}")
        example = examples_by_id[example_id]
        tests = example.tests
        if separator:
            tests = tuple(test for test in tests if test.name == test_name)
            if not tests:
                raise ExampleError(f"Unknown test selector: {selector}")
        for test in tests:
            key = (example.id, test.name)
            if key in selected_keys:
                raise ExampleError(f"Duplicate test selector: {example.id}:{test.name}")
            selected_keys.add(key)
            selected.append((example, test))
    return tuple(selected)


def _add_environment_arguments(parser: argparse.ArgumentParser, destination: str) -> None:
    """Add developer and installed environment overrides to a parser.

    Args:
        parser: Parser that accepts the environment selection.
        destination: Namespace attribute populated by these options.
    """
    group = parser.add_mutually_exclusive_group()
    group.add_argument(
        "--dev",
        dest=destination,
        action="store_const",
        const="dev",
        help="Require and use the configured source/libraries developer build.",
    )
    group.add_argument(
        "--installed",
        dest=destination,
        action="store_const",
        const="installed",
        help="Use only the currently active environment.",
    )


def _create_argument_parser(default_build_root: Path) -> argparse.ArgumentParser:
    """Create the examples command-line parser.

    Args:
        default_build_root: Default root for isolated build directories.

    Returns:
        Configured command-line parser.
    """
    parser = argparse.ArgumentParser(description=__doc__)
    _add_environment_arguments(parser, "root_environment_mode")
    parser.add_argument("--build-root", type=Path, default=default_build_root)
    parser.add_argument("--cmake", default=os.environ.get("CMAKE_COMMAND", "cmake"))
    parser.add_argument("--config", default="Release")
    parser.add_argument("--generator")
    parser.add_argument("--generator-platform")
    parser.add_argument("--generator-toolset")
    parser.add_argument("--make-program")
    subparsers = parser.add_subparsers(dest="command", required=True)

    subparsers.add_parser("list", help="List examples by stable ID.")

    build_parser = subparsers.add_parser("build", help="Build selected examples, or all examples by default.")
    _add_environment_arguments(build_parser, "command_environment_mode")
    build_parser.add_argument("examples", nargs="*", metavar="EXAMPLE_ID")

    run_parser = subparsers.add_parser("run", help="Build and run one example by ID, root path, or Python script path.")
    _add_environment_arguments(run_parser, "command_environment_mode")
    run_parser.add_argument("example", metavar="EXAMPLE_ID_OR_PATH")
    run_parser.add_argument("arguments", nargs=argparse.REMAINDER, metavar="EXAMPLE_ARGUMENT")

    test_parser = subparsers.add_parser("test", help="Build and test selected examples, or all examples by default.")
    _add_environment_arguments(test_parser, "command_environment_mode")
    test_parser.add_argument(
        "--no-build",
        action="store_true",
        help="Test examples already present under --build-root without invoking the build toolchain.",
    )
    test_parser.add_argument("selectors", nargs="*", metavar="EXAMPLE_ID[:TEST_NAME]")

    catalog_parser = subparsers.add_parser("catalog", help="Write the validated example catalog.")
    catalog_parser.add_argument("--format", choices=("json",), default="json")
    catalog_parser.add_argument("--output", type=Path, required=True)
    catalog_parser.add_argument(
        "--library-catalog",
        type=Path,
        default=default_build_root.parents[1]
        / "_cmake_build"
        / "isaacsim-libraries-release"
        / "documentation"
        / "library_catalog.json",
        help="Library documentation catalog used to validate owners and distribution requirements.",
    )
    return parser


def _main() -> int:
    """Execute the examples command-line interface.

    Returns:
        Process exit code.
    """
    examples_dir = Path(__file__).resolve().parent
    repository_root = examples_dir.parents[1]
    parser = _create_argument_parser(repository_root / "_build" / "examples")
    arguments = parser.parse_args()
    resources = contextlib.ExitStack()
    try:
        if arguments.command == "catalog":
            catalog = _build_catalog(examples_dir, arguments.library_catalog)
            arguments.output.parent.mkdir(parents=True, exist_ok=True)
            arguments.output.write_text(json.dumps(catalog, indent=2, sort_keys=True) + "\n", encoding="utf-8")
            return 0
        if arguments.command == "list":
            print(_format_examples_table(_load_examples(examples_dir)))
            return 0
        selected_modes = {arguments.root_environment_mode, arguments.command_environment_mode} - {None}
        if len(selected_modes) > 1:
            raise ExampleError("--dev and --installed cannot be used together")
        build_entry_point = "build.bat" if os.name == "nt" else "build.sh"
        source_checkout = (repository_root / ".git").exists() and (
            repository_root / "source" / "libraries" / build_entry_point
        ).is_file()
        environment_mode = arguments.command_environment_mode or arguments.root_environment_mode
        environment_mode = environment_mode or ("dev" if source_checkout else "installed")
        developer_build = None
        if environment_mode == "dev":
            if not source_checkout:
                raise ExampleError("--dev requires examples.py to be run from an Isaac Sim source checkout")
            library_build_dir = repository_root / "_cmake_build" / f"isaacsim-libraries-{arguments.config.lower()}"
            resources.enter_context(_developer_environment_lock(library_build_dir, exclusive=False))
            expected_developer_build = str(library_build_dir.resolve())
            active_developer_build = os.environ.get(DEVELOPER_ENVIRONMENT_VARIABLE)
            # Hash sources in the final environment while holding its lock. Metadata and
            # path checks still run before the restart needed to select that environment.
            developer_build = _read_developer_build(
                repository_root,
                arguments.config,
                validate_sources=active_developer_build == expected_developer_build,
            )
            if (
                active_developer_build != expected_developer_build
                or Path(sys.executable).resolve() != developer_build.python.resolve()
            ):
                print(f"Examples environment: developer ({developer_build.library_build_dir})", flush=True)
                environment = _create_developer_environment(developer_build)
                return _restart_in_environment(developer_build.python, environment)
        system_requirements = _load_system_requirements(
            repository_root / "source" / "libraries" / "system_requirements.toml"
        )
        if environment_mode == "dev":
            if developer_build is None:
                raise ExampleError("Developer environment selection did not resolve a configured build")
            arguments.cmake = str(developer_build.cmake)
            arguments.generator = arguments.generator or developer_build.generator
            arguments.generator_platform = arguments.generator_platform or developer_build.generator_platform
            arguments.generator_toolset = arguments.generator_toolset or developer_build.generator_toolset
            arguments.make_program = arguments.make_program or developer_build.make_program
        else:
            print("Examples environment: installed", flush=True)
        _validate_python_version(system_requirements)
        examples = _load_examples(examples_dir)
        build_root = arguments.build_root.resolve()
        _validate_build_root(build_root, examples_dir)
        selected_build_examples = _select_examples(examples, arguments.examples) if arguments.command == "build" else ()
        run_example = (
            _select_run_example(examples, arguments.example, examples_dir) if arguments.command == "run" else None
        )
        tests = _select_tests(examples, arguments.selectors) if arguments.command == "test" else ()
        builds_dependencies = arguments.command == "build" or (arguments.command == "test" and not arguments.no_build)
        if _collect_python_packages(examples):
            resources.enter_context(_python_dependency_lock(build_root, exclusive=builds_dependencies))
        if _collect_cpp_packages(examples):
            resources.enter_context(_cpp_dependency_lock(build_root, exclusive=builds_dependencies))
        python_dependency_loader = _prepare_python_dependencies if builds_dependencies else _require_python_dependencies
        cpp_dependency_loader = _prepare_cpp_dependencies if builds_dependencies else _require_cpp_dependencies
        python_dependency_dir = python_dependency_loader(examples, build_root)
        cpp_dependencies = cpp_dependency_loader(examples, build_root)
        common_build_arguments = (
            python_dependency_dir,
            cpp_dependencies,
            system_requirements,
            build_root,
            arguments.cmake,
            arguments.config,
            arguments.generator,
            arguments.generator_platform,
            arguments.generator_toolset,
            arguments.make_program,
        )
        if arguments.command == "build":
            for example in selected_build_examples:
                _build_example(example, *common_build_arguments)
            return 0
        if arguments.command == "run":
            if run_example is None:
                raise ExampleError("Run command did not select an example")
            example = run_example
            build_dir = build_root / example.id
            _build_example(example, *common_build_arguments)
            passthrough = tuple(arguments.arguments)
            if passthrough[:1] == ("--",):
                passthrough = passthrough[1:]
            return _run_example(example, build_dir, passthrough, python_dependency_dir, cpp_dependencies)

        selected_example_ids = {example.id for example, _ in tests}
        if arguments.selectors:
            explicitly_selected_examples = {selector.partition(":")[0] for selector in arguments.selectors}
        else:
            explicitly_selected_examples = {example.id for example in examples}
        for example in examples:
            if example.id in explicitly_selected_examples and not example.tests:
                print(f"Test {example.id}: skipped (no test configurations)")
        build_directories: dict[str, Path] = {}
        for example in examples:
            if example.id in selected_example_ids:
                build_dir = build_root / example.id
                if arguments.no_build:
                    build_directories[example.id] = build_dir
                else:
                    build_directories[example.id] = _build_example(example, *common_build_arguments)
        for example, test in tests:
            _test_example(example, test, build_directories[example.id], python_dependency_dir, cpp_dependencies)
        return 0
    except (ExampleError, OSError) as error:
        print(f"Error: {error}", file=sys.stderr)
        return 1
    finally:
        resources.close()


if __name__ == "__main__":
    raise SystemExit(_main())
