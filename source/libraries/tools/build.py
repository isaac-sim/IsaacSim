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

"""Build and test the standalone libraries under ``source/libraries``."""

from __future__ import annotations

import argparse
import datetime
import hashlib
import importlib.util
import json
import os
import platform
import re
import shutil
import subprocess
import sys
import tempfile
import time
from collections.abc import Iterator, Sequence
from contextlib import contextmanager
from dataclasses import dataclass
from pathlib import Path

_STAGING_MANIFEST_SPEC = importlib.util.spec_from_file_location(
    "isaacsim_wheel_staging_manifest", Path(__file__).with_name("wheel_staging_manifest.py")
)
if _STAGING_MANIFEST_SPEC is None or _STAGING_MANIFEST_SPEC.loader is None:
    raise RuntimeError("Cannot load wheel staging manifest helpers")
_STAGING_MANIFEST = importlib.util.module_from_spec(_STAGING_MANIFEST_SPEC)
_STAGING_MANIFEST_SPEC.loader.exec_module(_STAGING_MANIFEST)

_PATHS_SPEC = importlib.util.spec_from_file_location(
    "isaacsim_wheel_staging_paths", Path(__file__).with_name("wheel_staging_paths.py")
)
if _PATHS_SPEC is None or _PATHS_SPEC.loader is None:
    raise RuntimeError("Cannot load wheel staging path helpers")
_PATHS = importlib.util.module_from_spec(_PATHS_SPEC)
_PATHS_SPEC.loader.exec_module(_PATHS)

LIBRARIES_DIRECTORY = Path(__file__).resolve().parents[1]
REPOSITORY_ROOT = LIBRARIES_DIRECTORY.parents[1]
LIBRARY_BUILD_ROOT = REPOSITORY_ROOT / "_cmake_build"
LIBRARY_ARTIFACT_ROOT = LIBRARY_BUILD_ROOT / "isaacsim-libraries-artifacts"
WINDOWS_TOOLCHAIN_DIRECTORY = REPOSITORY_ROOT / "_build" / "host-deps"
EXTENSION_WHEEL_ARTIFACT_ROOT = LIBRARY_BUILD_ROOT / "extension-wheel-artifacts"
DEPENDENCY_MANIFEST = REPOSITORY_ROOT / "deps" / "isaacsim-libraries.packman.xml"
_EXTENSION_NAME_PATTERN = re.compile(r"^[a-z][a-z0-9_]*(?:\.[a-z][a-z0-9_]*)+$")
_DISTRIBUTION_NAME_PATTERN = re.compile(r"^[a-z][a-z0-9_]*$")
PIXI_BUILD_PREFIX_VARIABLE = "ISAACSIM_PIXI_BUILD_PREFIX"
PIXI_NATIVE_RUNTIME_DEPS_VARIABLE = "ISAACSIM_PIXI_NATIVE_RUNTIME_DEPS"
PIXI_RUNTIME_DEPS_VARIABLE = "ISAACSIM_PIXI_RUNTIME_DEPS"
PIXI_SCHEMA_BUILD_DEPS_VARIABLE = "ISAACSIM_PIXI_SCHEMA_BUILD_DEPS"
PIXI_TEST_DEPS_VARIABLE = "ISAACSIM_PIXI_TEST_DEPS"
PIXI_WHEEL_BUILD_DEPS_VARIABLE = "ISAACSIM_PIXI_WHEEL_BUILD_DEPS"
PIXI_LICENSE_FILE_VARIABLE = "ISAACSIM_PIXI_LICENSE_FILE"
WINDOWS_DEBUG_RUNTIME_PATH_VARIABLE = "ISAACSIM_WINDOWS_DEBUG_RUNTIME_PATH"
_PACKAGED_WINDOWS_TOOLCHAIN_ENVIRONMENT_VARIABLES = (
    "DevEnvDir",
    "ExtensionSdkDir",
    "INCLUDE",
    "LIB",
    "LIBPATH",
    "UCRTVersion",
    "UniversalCRTSdkDir",
    "VCIDEInstallDir",
    "VCINSTALLDIR",
    "VCToolsInstallDir",
    "VCToolsRedistDir",
    "VCToolsVersion",
    "VisualStudioVersion",
    "VSCMD_ARG_app_plat",
    "VSCMD_ARG_HOST_ARCH",
    "VSCMD_ARG_TGT_ARCH",
    "VSCMD_VER",
    "VSINSTALLDIR",
    "WindowsLibPath",
    "WindowsSdkBinPath",
    "WindowsSdkDir",
    "WindowsSDKLibVersion",
    "WindowsSDKVersion",
    "__VSCMD_PREINIT_PATH",
)
ARTIFACT_STATE_FILE = "artifact-state.json"
CONFIGURE_CONTEXT_FILE = "configure-context.json"
DEVELOPER_ENVIRONMENT_DIRECTORY = "developer-environment"
DEVELOPER_ENVIRONMENT_LOCK_SUFFIX = "developer-environment.lock"
DEVELOPER_ENVIRONMENT_SURFACES = ("runtime", "development", "python")
SOURCE_BUILD_METRICS_FILE = "source-build-metrics.json"
ARTIFACT_INPUT_PATHS = (
    Path("pixi.toml"),
    Path("pixi.lock"),
    Path("deps/recipes/doctest/recipe.yaml"),
    Path("deps/isaacsim-libraries.packman.xml"),
    Path("tools/pixi-bootstrap.json"),
    Path("tools/pixi_licenses.py"),
    Path("tools/pixi_run.py"),
)


@dataclass(frozen=True)
class _HostPlatform:
    """Describe the supported host and its Packman selector."""

    name: str
    packman_platform: str
    executable_suffix: str


@dataclass(frozen=True)
class _BuildProfile:
    """CMake feature selection for one supported library build shape."""

    testing_enabled: bool
    python_enabled: bool
    bindings_enabled: bool
    warnings_as_errors: bool = False


def _directory_storage(path: Path) -> dict[str, int]:
    """Measure unique regular files in a directory tree.

    Args:
        path: Directory to measure.

    Returns:
        Logical size, allocated disk size, and unique file count.
    """
    logical_size = 0
    disk_size = 0
    file_count = 0
    seen_files: set[tuple[int, int]] = set()
    for candidate in path.rglob("*"):
        if not candidate.is_file() or candidate.is_symlink():
            continue
        stat = candidate.stat()
        identity = (stat.st_dev, stat.st_ino)
        if identity in seen_files:
            continue
        seen_files.add(identity)
        logical_size += stat.st_size
        disk_size += getattr(stat, "st_blocks", 0) * 512 or stat.st_size
        file_count += 1
    return {
        "logical_size_bytes": logical_size,
        "disk_size_bytes": disk_size,
        "file_count": file_count,
    }


_BUILD_PROFILES = {
    "standard": _BuildProfile(True, True, True),
    "cpp-tests": _BuildProfile(True, False, False),
    "python-modules": _BuildProfile(True, True, False),
    "cpp-library": _BuildProfile(False, False, False),
    "werror": _BuildProfile(True, True, True, True),
}


def _prepend_windows_environment_paths(name: str, paths: Sequence[Path]) -> None:
    """Prepend deterministic toolchain paths to one Windows environment variable.

    Args:
        name: Name of the item.
        paths: Paths values.
    """
    values = [str(path) for path in paths]
    existing = os.environ.get(name)
    if existing:
        values.append(existing)
    os.environ[name] = ";".join(values)


def _activate_packaged_windows_sdk(sdk_root: Path) -> None:
    """Activate the flat Packman Windows SDK layout for an x64 Ninja build.

    Args:
        sdk_root: SDK root directory.
    """
    include_root = sdk_root / "include"
    library_root = sdk_root / "lib"
    binary_root = sdk_root / "bin"
    if not (include_root / "ucrt").is_dir():
        versions = sorted(path for path in include_root.glob("10.*") if path.is_dir())
        if len(versions) != 1:
            raise RuntimeError(f"Packaged Windows SDK has an unsupported include layout: {sdk_root}")
        version = versions[0].name
        include_root = versions[0]
        library_root /= version
        binary_root /= version

    include_paths = (
        include_root / "ucrt",
        include_root / "um",
        include_root / "shared",
    )
    library_paths = (library_root / "ucrt" / "x64", library_root / "um" / "x64")
    binary_path = binary_root / "x64"
    required_paths = (
        *include_paths,
        *library_paths,
        library_root / "um" / "x64" / "kernel32.lib",
        binary_path / "rc.exe",
        binary_path / "mt.exe",
    )
    missing = [str(path) for path in required_paths if not path.exists()]
    if missing:
        raise RuntimeError(f"Packaged Windows SDK is incomplete; missing: {', '.join(missing)}")

    _prepend_windows_environment_paths("PATH", (binary_path,))
    _prepend_windows_environment_paths("INCLUDE", include_paths)
    _prepend_windows_environment_paths("LIB", library_paths)
    os.environ["WindowsSdkDir"] = f"{sdk_root}\\"


def _has_packaged_windows_sdk(sdk_root: Path) -> bool:
    """Return whether a dependency root contains the SDK development files.

    Args:
        sdk_root: SDK root directory.

    Returns:
        The resulting value.
    """
    return (sdk_root / "include").is_dir() and (sdk_root / "lib").is_dir()


def _parse_windows_environment(output: str, base_environment: dict[str, str]) -> dict[str, str]:
    """Apply ``set`` output to an environment while honoring case-insensitive variable names.

    Args:
        output: Output buffer to populate.
        base_environment: Base process environment.

    Returns:
        The resulting value.
    """
    environment = base_environment.copy()
    names = {name.casefold(): name for name in environment}
    for line in output.splitlines():
        name, separator, value = line.partition("=")
        if not separator or not name:
            continue
        previous_name = names.get(name.casefold())
        target_name = previous_name if previous_name is not None else name
        environment[target_name] = value
        names[name.casefold()] = target_name
    return environment


def _windows_environment_value(environment: dict[str, str], variable: str) -> str:
    """Return a value from a case-insensitive Windows environment mapping.

    Args:
        environment: Process environment variables.
        variable: Environment variable name.

    Returns:
        The resulting value.
    """
    variable = variable.casefold()
    return next(
        (value for name, value in environment.items() if name.casefold() == variable),
        "",
    )


def activate_packaged_windows_debug_runtime(toolchain_root: Path, sdk_root: Path) -> None:
    """Expose the non-redistributable MSVC and Windows SDK Debug runtimes.

    Args:
        toolchain_root: Toolchain root directory.
        sdk_root: SDK root directory.
    """
    msvc_runtime_directories = tuple(
        sorted(toolchain_root.glob("VC/Redist/MSVC/*/debug_nonredist/x64/Microsoft.VC*.DebugCRT"))
    )
    if not msvc_runtime_directories:
        raise RuntimeError(f"Packaged MSVC Debug CRT is missing below: {toolchain_root}")
    sdk_runtime_directories = tuple(sorted({path.parent for path in sdk_root.rglob("ucrtbased.dll")}))
    if not sdk_runtime_directories:
        raise RuntimeError(f"Packaged Windows SDK Debug UCRT is missing below: {sdk_root}")
    runtime_directories = (*msvc_runtime_directories, *sdk_runtime_directories)
    print(f"Using packaged Windows Debug runtime directories: {', '.join(map(str, runtime_directories))}")
    _prepend_windows_environment_paths("PATH", runtime_directories)
    os.environ[WINDOWS_DEBUG_RUNTIME_PATH_VARIABLE] = os.pathsep.join(map(str, runtime_directories))


def _detect_host_platform() -> _HostPlatform:
    """Return the supported platform corresponding to the current host.

    Returns:
        The resulting value.

    Raises:
        RuntimeError: If the operating system or processor architecture is unsupported.
    """
    machine = platform.machine().lower()
    if sys.platform == "win32" and machine in {"amd64", "x86_64"}:
        return _HostPlatform("windows-x86_64", "windows-x86_64", ".exe")
    if sys.platform.startswith("linux") and machine in {"amd64", "x86_64"}:
        return _HostPlatform("linux-x86_64", "manylinux_2_35_x86_64", "")
    if sys.platform.startswith("linux") and machine in {"aarch64", "arm64"}:
        return _HostPlatform("linux-aarch64", "manylinux_2_35_aarch64", "")
    raise RuntimeError(f"Unsupported library build host: {sys.platform}/{machine}")


def ensure_windows_msvc_environment(*, require_debug_runtime: bool = False) -> Path | None:
    """Initialize the x64 MSVC environment and return its C/C++ compiler.

    Args:
        require_debug_runtime: Whether to expose the packaged Debug CRT and UCRT DLL directories.

    Returns:
        The resolved ``cl.exe`` path on Windows, or ``None`` on other hosts.

    Raises:
        RuntimeError: If the supported x64 C++ toolchain cannot be located or initialized.
    """
    if sys.platform != "win32":
        return None

    packaged_installation = WINDOWS_TOOLCHAIN_DIRECTORY / "msvc"
    packaged_sdk = WINDOWS_TOOLCHAIN_DIRECTORY / "winsdk"
    packaged_vsdevcmd = packaged_installation / "Common7" / "Tools" / "VsDevCmd.bat"
    original_environment = os.environ.copy()
    ambient_compiler = shutil.which("cl.exe")
    ambient_environment_ready = all(
        _windows_environment_value(original_environment, name) for name in ("INCLUDE", "LIB")
    )
    use_packaged_toolchain = packaged_vsdevcmd.is_file() and _has_packaged_windows_sdk(packaged_sdk)
    if use_packaged_toolchain:
        installation_path = packaged_installation
    else:
        if ambient_compiler is not None and ambient_environment_ready:
            return Path(ambient_compiler).resolve()

        installer_root = Path(os.environ.get("ProgramFiles(x86)", r"C:\Program Files (x86)"))
        vswhere = installer_root / "Microsoft Visual Studio" / "Installer" / "vswhere.exe"
        if not vswhere.is_file():
            raise RuntimeError(f"Visual Studio locator is missing: {vswhere}")
        completed = subprocess.run(
            [
                str(vswhere),
                "-latest",
                "-version",
                "[17.0,18.0)",
                "-products",
                "*",
                "-requires",
                "Microsoft.VisualStudio.Component.VC.Tools.x86.x64",
                "-property",
                "installationPath",
            ],
            cwd=REPOSITORY_ROOT,
            capture_output=True,
            text=True,
            check=True,
        )
        installation_path = completed.stdout.strip()
        if not installation_path:
            raise RuntimeError("Visual Studio 2022 with the x64 C++ toolchain is not installed")

    vsdevcmd = Path(installation_path) / "Common7" / "Tools" / "VsDevCmd.bat"
    if not vsdevcmd.is_file():
        raise RuntimeError(f"Visual Studio developer command file is missing: {vsdevcmd}")

    # The Packman toolchain omits optional IDE components whose initialization scripts may return nonzero.
    # Emit the environment regardless; the compiler lookup below validates whether the required C++ setup succeeded.
    activation_environment = original_environment
    if use_packaged_toolchain:
        scrubbed_names = {name.casefold() for name in _PACKAGED_WINDOWS_TOOLCHAIN_ENVIRONMENT_VARIABLES}
        activation_environment = {
            name: value for name, value in original_environment.items() if name.casefold() not in scrubbed_names
        }
    completed = subprocess.run(
        f'cmd.exe /d /c call "{vsdevcmd}" -arch=x64 -host_arch=x64 -no_logo >nul & set',
        cwd=REPOSITORY_ROOT,
        env=activation_environment,
        capture_output=True,
        text=True,
        check=False,
    )
    environment = _parse_windows_environment(completed.stdout, activation_environment)
    compiler = shutil.which("cl.exe", path=_windows_environment_value(environment, "PATH"))
    if use_packaged_toolchain and compiler is not None:
        resolved_compiler = Path(compiler).resolve()
        has_compiler_paths = all(_windows_environment_value(environment, name) for name in ("INCLUDE", "LIB"))
        if not resolved_compiler.is_relative_to(packaged_installation.resolve()) or not has_compiler_paths:
            compiler = None

    if compiler is None:
        if use_packaged_toolchain and ambient_compiler is not None and ambient_environment_ready:
            resolved_compiler = Path(ambient_compiler).resolve()
            print(
                f"Packaged MSVC initialization returned {completed.returncode} without a usable compiler; "
                f"using the active MSVC compiler: {resolved_compiler}"
            )
            if require_debug_runtime:
                activate_packaged_windows_debug_runtime(packaged_installation, packaged_sdk)
            return resolved_compiler
        raise RuntimeError(
            f"{vsdevcmd} did not provide an x64 MSVC compiler environment "
            f"(initialization exit code: {completed.returncode})"
        )

    resolved_compiler = Path(compiler).resolve()
    if completed.returncode:
        print(
            f"MSVC initialization returned {completed.returncode}; "
            f"continuing with the validated compiler: {resolved_compiler}"
        )

    try:
        os.environ.clear()
        os.environ.update(environment)
        if use_packaged_toolchain:
            _activate_packaged_windows_sdk(packaged_sdk)
        if require_debug_runtime and packaged_vsdevcmd.is_file():
            activate_packaged_windows_debug_runtime(packaged_installation, packaged_sdk)
    except Exception:
        os.environ.clear()
        os.environ.update(original_environment)
        raise
    return resolved_compiler


def _run(
    command: Sequence[object],
    *,
    environment: dict[str, str | None] | None = None,
    quiet: bool = False,
) -> None:
    """Run a command from the repository root and stop on failure.

    Args:
        command: Command and arguments to execute.
        environment: Environment variables to add, replace, or remove when set to ``None``.
        quiet: Whether to emit command output only after a failure.
    """
    command_strings = [str(value) for value in command]
    print(f"+ {subprocess.list2cmdline(command_strings)}", flush=True)
    merged_environment = os.environ.copy()
    if environment:
        for name, value in environment.items():
            if value is None:
                merged_environment.pop(name, None)
            else:
                merged_environment[name] = value
    if not quiet:
        subprocess.run(command_strings, cwd=REPOSITORY_ROOT, env=merged_environment, check=True)
        return
    result = subprocess.run(
        command_strings,
        cwd=REPOSITORY_ROOT,
        env=merged_environment,
        capture_output=True,
        text=True,
        check=False,
    )
    if result.returncode != 0:
        raise RuntimeError(
            f"Command failed with exit code {result.returncode}: {subprocess.list2cmdline(command_strings)}\n"
            f"{result.stdout}{result.stderr}"
        )


def _pull_dependencies(host: _HostPlatform) -> None:
    """Pull the platform's library dependencies through Packman.

    Args:
        host: Host platform selection.
    """
    packman = REPOSITORY_ROOT / "tools" / "packman" / ("packman.cmd" if sys.platform == "win32" else "packman")
    _run(
        [
            packman,
            "pull",
            DEPENDENCY_MANIFEST,
            "-p",
            host.packman_platform,
            "-t",
            f"platform_target_abi={host.packman_platform}",
        ],
        environment={"LD_LIBRARY_PATH": None},
    )


def _find_build_tool(prefix: Path, host: _HostPlatform, name: str) -> Path:
    """Return one pinned executable from the Pixi build-driver prefix.

    Args:
        prefix: Installed Pixi build-driver environment.
        host: Host platform selection.
        name: Executable base name.

    Returns:
        The resulting value.

    Raises:
        RuntimeError: If the locked environment does not contain the tool.
    """
    executable_name = f"{name}{host.executable_suffix}"
    candidates = (
        (prefix / "Library" / "bin" / executable_name, prefix / executable_name)
        if host.name == "windows-x86_64"
        else (prefix / "bin" / executable_name,)
    )
    for path in candidates:
        if path.is_file():
            return _require_executable(path, f"Pinned {name}")
    raise RuntimeError(f"Pinned {name} is missing from the Pixi build-driver environment: {prefix}")


def _require_executable(path: Path, description: str) -> Path:
    """Validate an executable supplied by a locked Pixi environment.

    Args:
        path: Executable file to validate.
        description: Human-readable executable identity for diagnostics.

    Returns:
        The validated path.

    Raises:
        RuntimeError: If the path is missing or is not executable.
    """
    if not path.is_file():
        raise RuntimeError(f"{description} is missing: {path}")
    if sys.platform != "win32" and not os.access(path, os.X_OK):
        raise RuntimeError(f"{description} is not executable: {path}")
    return path


def _read_pixi_dependency_path(variable: str) -> Path:
    """Read and validate one required dependency path selected by the Pixi launcher.

    Args:
        variable: Environment variable containing the dependency path.

    Returns:
        Validated dependency directory.

    Raises:
        RuntimeError: If the path is absent or is not a directory.
    """
    value = os.environ.get(variable)
    if not value:
        raise RuntimeError(f"Pixi launcher did not provide {variable}")
    path = Path(value).resolve()
    if not path.is_dir():
        raise RuntimeError(f"Pixi dependency directory is missing for {variable}: {path}")
    return path


def _read_pixi_license_file() -> Path:
    """Read and validate the aggregate selected by the Pixi launcher.

    Returns:
        The resulting value.
    """
    value = os.environ.get(PIXI_LICENSE_FILE_VARIABLE)
    if not value:
        raise RuntimeError(f"Pixi launcher did not provide {PIXI_LICENSE_FILE_VARIABLE}")
    path = Path(value).resolve()
    if not path.is_file() or not path.read_bytes().strip():
        raise RuntimeError(f"Pixi license aggregate is missing or empty: {path}")
    return path


def _file_digest(path: Path) -> str:
    """Return the SHA-256 digest of a required build input.

    Args:
        path: Filesystem path to process.

    Returns:
        The resulting value.
    """
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def _source_tree_digest(root: Path) -> str:
    """Fingerprint authored library sources while ignoring generated output.

    Args:
        root: Root directory to process.

    Returns:
        The resulting value.
    """
    excluded = frozenset({"__pycache__", ".mypy_cache", ".pytest_cache", ".ruff_cache", "dist"})
    digest = hashlib.sha256()
    for path in sorted(root.rglob("*")):
        relative = path.relative_to(root)
        if any(part in excluded for part in relative.parts) or not path.is_file():
            continue
        digest.update(relative.as_posix().encode())
        digest.update(b"\0")
        with path.open("rb") as stream:
            for chunk in iter(lambda: stream.read(1024 * 1024), b""):
                digest.update(chunk)
    return digest.hexdigest()


def _write_json_atomically(path: Path, document: dict[str, object]) -> None:
    """Publish one state document atomically after its operation succeeds.

    Args:
        path: Filesystem path to process.
        document: Parsed TOML document.
    """
    path.parent.mkdir(parents=True, exist_ok=True)
    descriptor, temporary_name = tempfile.mkstemp(prefix=f".{path.name}-", dir=path.parent)
    temporary = Path(temporary_name)
    try:
        with os.fdopen(descriptor, "w", encoding="utf-8", newline="\n") as output:
            json.dump(document, output, indent=2, sort_keys=True)
            output.write("\n")
            output.flush()
            os.fsync(output.fileno())
        os.replace(temporary, path)
    except Exception:
        temporary.unlink(missing_ok=True)
        raise


def _configure_context(environment: dict[str, str] | None = None) -> dict[str, object]:
    """Describe the environment whose system paths CMake may cache.

    Args:
        environment: Environment to inspect, or None to inspect the current process environment.

    Returns:
        Stable description of the CMake execution environment.
    """
    effective_environment = os.environ if environment is None else environment
    return {
        "schema_version": 1,
        "execution_environment": ("linbuild" if "LINBUILD_EMBEDDED" in effective_environment else "host"),
        "platform": sys.platform,
        "machine": platform.machine().lower(),
        "linux_toolchain": (
            effective_environment.get("ISAACSIM_PIXI_BUILD_PREFIX") if sys.platform == "linux" else None
        ),
        "linux_toolchain_lock": (_file_digest(REPOSITORY_ROOT / "pixi.lock") if sys.platform == "linux" else None),
    }


def _configuration_refresh_reason(build_directory: Path, context: dict[str, object]) -> str | None:
    """Return why an existing CMake cache must be regenerated.

    Args:
        build_directory: CMake binary directory to inspect.
        context: Current CMake execution-environment description.

    Returns:
        Reason to refresh the cache, or None when the existing cache remains valid.
    """
    cache = build_directory / "CMakeCache.txt"
    if not cache.is_file():
        return None

    context_path = build_directory / CONFIGURE_CONTEXT_FILE
    try:
        recorded_context = json.loads(context_path.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError):
        return "its execution environment was not recorded"
    if recorded_context != context:
        return "the execution environment changed"

    missing_paths: list[str] = []
    for line in cache.read_text(encoding="utf-8", errors="replace").splitlines():
        key_and_type, separator, value = line.partition("=")
        if not separator or not key_and_type.endswith(":FILEPATH") or value.endswith("-NOTFOUND"):
            continue
        if Path(value).is_absolute() and not Path(value).exists():
            missing_paths.append(value)
    if missing_paths:
        return f"a cached dependency is missing: {missing_paths[0]}"
    return None


def _artifact_state_inputs(
    *,
    configuration: str,
    profile: str,
    dependency_profile: str,
    repository_root: Path = REPOSITORY_ROOT,
    libraries_directory: Path = LIBRARIES_DIRECTORY,
) -> dict[str, object]:
    """Return the current immutable inputs for one build selection.

    Args:
        configuration: Build configuration name.
        profile: Build feature profile.
        dependency_profile: Dependency version profile.
        repository_root: Repository root directory.
        libraries_directory: Directory containing the libraries.

    Returns:
        The resulting value.
    """
    inputs = {path.as_posix(): _file_digest(repository_root / path) for path in ARTIFACT_INPUT_PATHS}
    return {
        "schema_version": 1,
        "configuration": configuration,
        "profile": profile,
        "dependency_profile": dependency_profile,
        "inputs": inputs,
        "source_digest": _source_tree_digest(libraries_directory),
    }


def _write_artifact_state(
    build_directory: Path,
    *,
    configuration: str,
    profile: str,
    dependency_profile: str,
    target: str | None,
    developer_environment_components: tuple[str, ...] | None = None,
) -> None:
    """Record current inputs only after the native build succeeds.

    Args:
        build_directory: Directory containing the CMake build.
        configuration: Build configuration name.
        profile: Build feature profile.
        dependency_profile: Dependency version profile.
        target: Optional CMake target to build.
        developer_environment_components: Components staged in the developer environment.
    """
    _write_json_atomically(
        build_directory / ARTIFACT_STATE_FILE,
        {
            **_artifact_state_inputs(
                configuration=configuration,
                profile=profile,
                dependency_profile=dependency_profile,
            ),
            "repository_root": REPOSITORY_ROOT.as_posix(),
            "targets": [target] if target is not None else ["all"],
            "developer_environment": (
                {
                    "path": DEVELOPER_ENVIRONMENT_DIRECTORY,
                    "components": list(developer_environment_components),
                }
                if developer_environment_components is not None
                else None
            ),
            "completed_at": datetime.datetime.now(datetime.timezone.utc).isoformat(),
        },
    )


def _read_developer_environment_components(build_directory: Path) -> tuple[str, ...]:
    """Read every complete package's runtime, development, and Python components.

    Args:
        build_directory: Directory containing the CMake build.

    Returns:
        The resulting value.
    """
    package_directory = build_directory / "packages"
    manifest_paths = sorted(package_directory.glob("*/package.json"))
    if not manifest_paths:
        raise RuntimeError(f"Developer environment has no package manifests under {package_directory}")

    components_by_surface: dict[str, list[str]] = {surface: [] for surface in DEVELOPER_ENVIRONMENT_SURFACES}
    for manifest_path in manifest_paths:
        try:
            manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
        except (OSError, json.JSONDecodeError) as error:
            raise RuntimeError(f"Cannot read developer package manifest {manifest_path}: {error}") from error
        if not isinstance(manifest, dict) or manifest.get("complete") is not True:
            raise RuntimeError(f"Developer package manifest is incomplete: {manifest_path}")
        components = manifest.get("components")
        if not isinstance(components, dict):
            raise RuntimeError(f"Developer package manifest has invalid components: {manifest_path}")
        for surface in DEVELOPER_ENVIRONMENT_SURFACES:
            component = components.get(surface)
            if not isinstance(component, str) or not component:
                raise RuntimeError(f"Developer package manifest has no {surface} component: {manifest_path}")
            components_by_surface[surface].append(component)

    ordered_components = [
        component for surface in DEVELOPER_ENVIRONMENT_SURFACES for component in components_by_surface[surface]
    ]
    return tuple(dict.fromkeys(ordered_components))


def _remove_owned_directory(path: Path) -> None:
    """Remove one generated directory without following a redirected root.

    Args:
        path: Filesystem path to process.
    """
    if not path.exists() and not path.is_symlink():
        return
    is_junction = getattr(path, "is_junction", None)
    if path.is_symlink() or (is_junction is not None and is_junction()) or not path.is_dir():
        raise RuntimeError(f"Refusing to remove redirected developer environment path: {path}")
    shutil.rmtree(path)


@contextmanager
def _developer_environment_lock(build_directory: Path, *, exclusive: bool) -> Iterator[None]:
    """Coordinate environment readers with replacement and cleanup.

    Args:
        build_directory: Directory containing the CMake build.
        exclusive: Whether to acquire an exclusive lock.

    Yields:
        Control while the developer-environment lock is held.
    """
    lock_path = build_directory.parent / f".{build_directory.name}-{DEVELOPER_ENVIRONMENT_LOCK_SUFFIX}"
    lock_path.parent.mkdir(parents=True, exist_ok=True)
    with lock_path.open("a+b") as lock_file:
        if os.name == "nt":
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
            flags = 0x2 if exclusive else 0  # ``LOCKFILE_EXCLUSIVE_LOCK``.
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


def _materialize_developer_environment(
    cmake: Path,
    build_directory: Path,
    configuration: str,
) -> tuple[str, ...]:
    """Build one complete, consumer-ready developer environment transactionally.

    Args:
        cmake: Path to the CMake executable.
        build_directory: Directory containing the CMake build.
        configuration: Build configuration name.

    Returns:
        The resulting value.
    """
    components = _read_developer_environment_components(build_directory)
    environment_path = build_directory / DEVELOPER_ENVIRONMENT_DIRECTORY
    is_junction = getattr(environment_path, "is_junction", None)
    if environment_path.is_symlink() or (is_junction is not None and is_junction()):
        raise RuntimeError(f"Refusing to replace redirected developer environment path: {environment_path}")
    if environment_path.exists() and not environment_path.is_dir():
        raise RuntimeError(f"Developer environment path is not a directory: {environment_path}")

    staging_path = Path(tempfile.mkdtemp(prefix=".developer-environment-", dir=build_directory))
    previous_path = staging_path.with_name(f"{staging_path.name}-previous")
    try:
        for component in components:
            _run(
                [
                    cmake,
                    "--install",
                    build_directory,
                    "--prefix",
                    staging_path,
                    "--component",
                    component,
                    "--config",
                    configuration,
                ],
                environment={"CMAKE_INSTALL_MODE": None, "DESTDIR": None},
                quiet=True,
            )
        if not (staging_path / "lib" / "cmake").is_dir() or not (staging_path / "python").is_dir():
            raise RuntimeError(f"Developer environment is incomplete after installation: {staging_path}")

        with _developer_environment_lock(build_directory, exclusive=True):
            if environment_path.exists():
                environment_path.rename(previous_path)
            try:
                staging_path.rename(environment_path)
            except Exception:
                if previous_path.exists() and not environment_path.exists():
                    previous_path.rename(environment_path)
                raise
            _remove_owned_directory(previous_path)
            _remove_owned_directory(build_directory / "examples-dev")
    finally:
        _remove_owned_directory(staging_path)
        if environment_path.exists():
            _remove_owned_directory(previous_path)
    return components


def _ensure_developer_environment(cmake: Path, build_directory: Path, configuration: str) -> None:
    """Recreate an omitted developer environment from retained install rules.

    Args:
        cmake: Path to the CMake executable.
        build_directory: Directory containing the CMake build.
        configuration: CMake build type.
    """
    environment_path = build_directory / DEVELOPER_ENVIRONMENT_DIRECTORY
    is_junction = getattr(environment_path, "is_junction", None)
    if (
        environment_path.is_dir()
        and not environment_path.is_symlink()
        and not (is_junction is not None and is_junction())
        and (environment_path / "lib" / "cmake").is_dir()
        and (environment_path / "python").is_dir()
    ):
        return
    print(f"Rematerializing developer environment from restored build artifact: {environment_path}")
    _materialize_developer_environment(cmake, build_directory, configuration)


def _validate_artifact_state(
    build_directory: Path,
    *,
    configuration: str,
    profile: str,
    dependency_profile: str,
    repository_root: Path = REPOSITORY_ROOT,
    libraries_directory: Path = LIBRARIES_DIRECTORY,
) -> str:
    """Reject a missing, stale, malformed, or partial completed build.

    Args:
        build_directory: Directory containing the CMake build.
        configuration: Build configuration name.
        profile: Build feature profile.
        dependency_profile: Dependency version profile.
        repository_root: Repository root directory.
        libraries_directory: Directory containing the libraries.

    Returns:
        Repository root recorded by the build-producing checkout.
    """
    path = build_directory / ARTIFACT_STATE_FILE
    try:
        state = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError) as error:
        raise RuntimeError(f"Completed build state is missing or invalid at {path}: {error}") from error
    expected = _artifact_state_inputs(
        configuration=configuration,
        profile=profile,
        dependency_profile=dependency_profile,
        repository_root=repository_root,
        libraries_directory=libraries_directory,
    )
    if not isinstance(state, dict) or any(state.get(key) != value for key, value in expected.items()):
        raise RuntimeError(f"Completed build state is stale at {path}; rebuild the selected libraries")
    if state.get("targets") != ["all"]:
        raise RuntimeError(f"Completed build state is partial at {path}; rebuild without --target")
    repository_root = state.get("repository_root")
    if not isinstance(repository_root, str) or not repository_root:
        raise RuntimeError(f"Completed build state has no producer repository root at {path}; rebuild the libraries")
    return repository_root


def _relocate_cmake_metadata(
    build_directory: Path,
    producer_root: str,
    consumer_root: Path,
) -> None:
    """Retarget generated CMake metadata after a CI artifact changes checkout roots.

    CTest and install metadata embed absolute paths to executables, sources,
    dependency environments, build outputs, and child install scripts. GitLab
    executors use a job-specific checkout directory, so a restored build artifact
    must reference the consuming checkout before tests can run it.

    Args:
        build_directory: Directory containing the CMake build.
        producer_root: Producer installation root.
        consumer_root: Consumer build root.
    """
    source = producer_root.replace("\\", "/").rstrip("/")
    destination = consumer_root.as_posix().rstrip("/")
    if not source or source == destination:
        return

    source_prefix = f"{source}/"
    destination_prefix = f"{destination}/"
    native_source_prefix = source_prefix.replace("/", "\\")
    native_destination_prefix = destination_prefix.replace("/", "\\")
    escaped_source_prefix = source_prefix.replace("/", "\\\\")
    escaped_destination_prefix = destination_prefix.replace("/", "\\\\")

    cache = build_directory / "CMakeCache.txt"
    metadata_files = ((cache,) if cache.is_file() else ()) + tuple(
        path
        for filename in (
            "CTestTestfile.cmake",
            "cmake_install.cmake",
            "InstallScripts.json",
        )
        for path in build_directory.rglob(filename)
    )
    if not metadata_files:
        raise RuntimeError(f"Configured module CMake metadata is missing below: {build_directory}")

    relocated = 0
    already_relocated = False
    for path in metadata_files:
        content = path.read_text(encoding="utf-8")
        already_relocated = already_relocated or any(
            prefix in content
            for prefix in (
                destination_prefix,
                native_destination_prefix,
                escaped_destination_prefix,
            )
        )
        updated = content.replace(source_prefix, destination_prefix)
        updated = updated.replace(native_source_prefix, native_destination_prefix)
        updated = updated.replace(escaped_source_prefix, escaped_destination_prefix)
        if updated == content:
            continue
        path.write_text(updated, encoding="utf-8", newline="")
        relocated += 1
    if not relocated and not already_relocated:
        raise RuntimeError(
            f"CMake metadata below {build_directory} does not reference its recorded producer root: {source}"
        )
    if relocated:
        print(f"Relocated {relocated} CMake metadata files from {source} to {destination}")


def _configure(
    cmake: Path,
    ninja: Path,
    python: Path | None,
    build_directory: Path,
    configuration: str,
    profile: _BuildProfile,
    native_runtime_dependencies: Path,
    python_test_dependencies: Path | None,
    python_runtime_dependencies: Path | None,
    python_schema_build_dependencies: Path | None,
    pixi_build_prefix: Path,
    python_license_file: Path,
    coverage_enabled: bool = False,
    msvc_compiler: Path | None = None,
    extension_usd_root: Path | None = None,
    fresh: bool = False,
) -> None:
    """Configure one complete library build.

    Args:
        cmake: Pinned CMake executable.
        ninja: Ninja executable.
        python: Module Python executable for a Python-enabled profile.
        build_directory: CMake binary directory.
        configuration: CMake build type.
        profile: CMake feature selection.
        native_runtime_dependencies: Selected native-support Python environment.
        python_test_dependencies: Selected Python test dependency environment.
        python_runtime_dependencies: Selected Python runtime dependency environment.
        python_schema_build_dependencies: Selected USD schema generator dependency environment.
        pixi_build_prefix: Pixi prefix containing native build tools and public headers.
        python_license_file: Selected locked dependency license aggregate.
        coverage_enabled: Whether to instrument supported native targets for coverage.
        msvc_compiler: Explicit MSVC compiler path on Windows.
        extension_usd_root: Optional Kit OpenUSD root for extension schema generation.
        fresh: Whether to discard an existing CMake cache before configuring.
    """
    command: list[object] = [cmake]
    if fresh:
        command.append("--fresh")
    command.extend(
        [
            "-S",
            LIBRARIES_DIRECTORY,
            "-B",
            build_directory,
            "-G",
            "Ninja",
            f"-DCMAKE_MAKE_PROGRAM={ninja}",
            f"-DCMAKE_BUILD_TYPE={configuration}",
            f"-DBUILD_TESTING={'ON' if profile.testing_enabled else 'OFF'}",
            f"-DISAACSIM_ENABLE_PYTHON={'ON' if profile.python_enabled else 'OFF'}",
            f"-DISAACSIM_ENABLE_PYTHON_BINDINGS={'ON' if profile.bindings_enabled else 'OFF'}",
            f"-DISAACSIM_WARNINGS_AS_ERRORS={'ON' if profile.warnings_as_errors else 'OFF'}",
            "-DISAACSIM_ENFORCE_COMPLETE_PACKAGES=ON",
            f"-DISAACSIM_ENABLE_COVERAGE={'ON' if coverage_enabled else 'OFF'}",
            f"-DISAACSIM_PUBLIC_DEPS_ROOT={pixi_build_prefix}",
            f"-DISAACSIM_NATIVE_RUNTIME_DEPS_DIR={native_runtime_dependencies}",
            f"-DISAACSIM_COLLECTED_PYTHON_LICENSE_FILE={python_license_file}",
        ]
    )
    if msvc_compiler is not None:
        command.extend(
            (
                f"-DCMAKE_C_COMPILER={msvc_compiler}",
                f"-DCMAKE_CXX_COMPILER={msvc_compiler}",
            )
        )
    elif sys.platform == "linux":
        compiler = pixi_build_prefix / "bin" / f"{platform.machine().lower()}-conda-linux-gnu"
        command.extend(
            (
                f"-DCMAKE_C_COMPILER={compiler}-gcc",
                f"-DCMAKE_CXX_COMPILER={compiler}-g++",
                f"-DCMAKE_CUDA_HOST_COMPILER={compiler}-g++",
            )
        )
    command.append(
        "-DISAACSIM_EXTENSION_USD_ROOT="
        if extension_usd_root is None
        else f"-DISAACSIM_EXTENSION_USD_ROOT={extension_usd_root}"
    )
    if profile.python_enabled:
        if python is None:
            raise RuntimeError("A Python-enabled profile requires the module Python executable")
        command.append(f"-DPython_EXECUTABLE={python}")
        if python_test_dependencies is not None:
            command.append(f"-DISAACSIM_PYTHON_TEST_DEPS_DIR={python_test_dependencies}")
        if python_runtime_dependencies is not None:
            command.append(f"-DISAACSIM_PYTHON_RUNTIME_DEPS_DIR={python_runtime_dependencies}")
        if python_schema_build_dependencies is not None:
            command.append(f"-DISAACSIM_PYTHON_SCHEMA_BUILD_DEPS_DIR={python_schema_build_dependencies}")
    _run(command)


def _build(
    cmake: Path,
    build_directory: Path,
    configuration: str,
    target: str | None,
    jobs: int | None,
) -> None:
    """Build one configured library tree.

    Args:
        cmake: Pinned CMake executable.
        build_directory: CMake binary directory.
        configuration: CMake build type.
        target: Optional CMake target.
        jobs: Optional parallel job count.
    """
    command: list[object] = [
        cmake,
        "--build",
        build_directory,
        "--config",
        configuration,
    ]
    if target:
        command.extend(("--target", target))
    if jobs:
        command.extend(("--parallel", jobs))
    _run(command)


def _run_tests(
    cmake: Path,
    build_directory: Path,
    configuration: str,
    junit_output: Path | None,
    timeout: int | None,
    test_jobs: int | None,
    test_regex: str | None,
    exclude_test_regex: str | None,
) -> None:
    """Run CTest with the executable paired with the pinned CMake.

    Args:
        cmake: Pinned CMake executable.
        build_directory: CMake binary directory.
        configuration: CMake build type.
        junit_output: Optional JUnit report path.
        timeout: Optional per-test timeout in seconds.
        test_jobs: Optional parallel test job count.
        test_regex: Optional regular expression selecting CTest tests by name.
        exclude_test_regex: Optional regular expression excluding CTest tests by name.

    Raises:
        RuntimeError: If the build tree or pinned CTest executable is missing.
    """
    if not (build_directory / "CTestTestfile.cmake").is_file():
        raise RuntimeError(
            f"Configured module test build is missing: {build_directory}. "
            "Run the matching profile build before --test-only."
        )
    ctest = cmake.with_name(f"ctest{cmake.suffix}")
    if not ctest.is_file():
        raise RuntimeError(f"Pinned CTest is missing: {ctest}")
    ctest = _require_executable(ctest, "Pinned CTest")
    command: list[object] = [
        ctest,
        "--test-dir",
        build_directory,
        "--output-on-failure",
        "--no-tests=error",
        "-C",
        configuration,
    ]
    if junit_output is not None:
        junit_output = junit_output.resolve()
        junit_output.parent.mkdir(parents=True, exist_ok=True)
        command.extend(("--output-junit", junit_output))
    if timeout is not None:
        command.extend(("--timeout", timeout))
    if test_jobs is not None:
        command.extend(("--parallel", test_jobs))
    if test_regex is not None:
        command.extend(("-R", test_regex))
    if exclude_test_regex is not None:
        command.extend(("-E", exclude_test_regex))
    _run(command)


def _build_wheels(
    cmake: Path,
    python: Path,
    build_directory: Path,
    configuration: str,
    groups: Sequence[str],
    output_directory: Path | None,
    python_build_dependencies: Path,
    variants: dict[str, str] | None = None,
) -> None:
    """Assemble wheels from one completed library build.

    Args:
        cmake: Pinned CMake executable.
        python: Module Python executable.
        build_directory: CMake binary directory.
        configuration: CMake build type.
        groups: Optional distribution names to assemble.
        output_directory: Optional wheel output directory.
        python_build_dependencies: Selected wheel frontend/backend environment.
        variants: Optional wheel payload variant selected for each distribution.
    """
    command: list[object] = [
        python,
        "-s",
        LIBRARIES_DIRECTORY / "packaging" / "wheel.py",
        "--repo-root",
        REPOSITORY_ROOT,
        "--build-dir",
        build_directory,
        "--config",
        configuration.lower(),
        "--cmake",
        cmake,
        "--output-dir",
        (output_directory.resolve() if output_directory else LIBRARY_ARTIFACT_ROOT / configuration.lower()),
        "--build-deps-dir",
        python_build_dependencies,
    ]
    for group in groups:
        command.extend(("--group", group))
    for group, variant in sorted((variants or {}).items()):
        command.extend(("--variant", f"{group}={variant}"))
    _run(command, environment={"PYTHONPATH": str(python_build_dependencies)})


def _validate_wheelhouse(
    python: Path,
    wheel_directory: Path,
    metrics_output: Path,
    runtime_dependencies: Path,
    schema_dependencies: Path,
) -> None:
    """Install and validate a wheelhouse with the pinned module interpreter.

    Args:
        python: Module Python executable.
        wheel_directory: Directory containing wheel artifacts.
        metrics_output: Install metrics output path.
        runtime_dependencies: Locked runtime dependency site-packages.
        schema_dependencies: Locked schema dependency site-packages.
    """
    python_path = os.pathsep.join((str(runtime_dependencies), str(schema_dependencies)))
    _run(
        [
            python,
            "-s",
            LIBRARIES_DIRECTORY / "packaging" / "validate_wheelhouse.py",
            wheel_directory.resolve(),
            "--metrics-output",
            metrics_output.resolve(),
        ],
        environment={"PYTHONPATH": python_path},
    )


def _load_wheel_distribution(extension: str) -> str:
    """Read the module distribution staged in one Kit extension.

    Args:
        extension: Extension directory name below ``source/extensions``.

    Returns:
        Registered module distribution name.
    """
    if _EXTENSION_NAME_PATTERN.fullmatch(extension) is None:
        raise RuntimeError(f"Invalid extension name: {extension}")
    path = REPOSITORY_ROOT / "source" / "extensions" / extension / "wheel-staging.toml"
    document = _STAGING_MANIFEST.read_manifest(path)
    wheel = document.get("wheel")
    distribution = wheel.get("distribution") if isinstance(wheel, dict) else None
    if not isinstance(distribution, str) or _DISTRIBUTION_NAME_PATTERN.fullmatch(distribution) is None:
        raise RuntimeError(f"{path} must define a lowercase underscore-separated [wheel].distribution")
    return distribution


def _load_wheel_variant(extension: str) -> str | None:
    """Read the optional wheel payload variant selected by one Kit extension.

    Args:
        extension: Extension values.

    Returns:
        The resulting value.
    """
    path = REPOSITORY_ROOT / "source" / "extensions" / extension / "wheel-staging.toml"
    document = _STAGING_MANIFEST.read_manifest(path)
    wheel = document.get("wheel")
    variant = wheel.get("wheel-variant") if isinstance(wheel, dict) else None
    if variant is not None and (not isinstance(variant, str) or re.fullmatch(r"[a-z][a-z0-9]*", variant) is None):
        raise RuntimeError(f"{path} [wheel].wheel-variant must be a lowercase identifier")
    return variant


def _stage_extension_wheel(
    cmake: Path,
    python: Path,
    build_directory: Path,
    extension: str,
    artifacts_directory: Path,
    configuration: str,
) -> None:
    """Stage one wheel and directly installed SDK for a Kit extension build.

    Args:
        cmake: Pinned CMake executable.
        python: Module Python executable.
        build_directory: Completed CMake build directory.
        extension: Extension directory name below ``source/extensions``.
        artifacts_directory: Directory containing the wheel.
        configuration: Lowercase build configuration.
    """
    _run(
        [
            python,
            "-s",
            LIBRARIES_DIRECTORY / "tools" / "stage_extension_wheel.py",
            "--repo-root",
            REPOSITORY_ROOT,
            "--extension",
            extension,
            "--artifacts-dir",
            artifacts_directory,
            "--build-dir",
            build_directory,
            "--config",
            configuration,
            "--cmake",
            cmake,
        ]
    )


def _parse_arguments() -> argparse.Namespace:
    """Parse command-line arguments.

    Returns:
        The resulting value.
    """
    parser = argparse.ArgumentParser(
        prog="build.bat" if os.name == "nt" else "build.sh",
        description=__doc__,
    )
    configurations = parser.add_argument_group("configurations")
    configurations.add_argument("-r", "--release", action="store_true", help="Build Release.")
    configurations.add_argument("-d", "--debug", action="store_true", help="Build Debug.")
    clean_group = parser.add_mutually_exclusive_group()
    clean_group.add_argument(
        "-c",
        "--clean",
        action="store_true",
        help="Remove selected build trees and exit.",
    )
    clean_group.add_argument(
        "-x",
        "--rebuild",
        action="store_true",
        help="Remove selected build trees before building.",
    )
    parser.add_argument("-g", "--generate", action="store_true", help="Configure without building.")
    parser.add_argument("-t", "--target", help="Build one CMake target.")
    parser.add_argument("-j", "--jobs", type=int, help="Maximum parallel build jobs.")
    parser.add_argument(
        "--profile",
        choices=tuple(_BUILD_PROFILES),
        default="standard",
        help="Feature profile to configure.",
    )
    parser.add_argument("--test", action="store_true", help="Run CTest after building.")
    parser.add_argument(
        "--coverage",
        action="store_true",
        help="Instrument supported native targets for coverage.",
    )
    parser.add_argument(
        "--test-only",
        action="store_true",
        help="Run CTest from an existing build tree without configuring or building.",
    )
    parser.add_argument(
        "--junit-output",
        type=Path,
        help="Write CTest results as JUnit XML; requires --test or --test-only.",
    )
    parser.add_argument(
        "--test-timeout",
        type=int,
        help="Maximum seconds for each CTest test; requires --test or --test-only.",
    )
    parser.add_argument(
        "--test-jobs",
        type=int,
        help="Maximum parallel CTest jobs; requires --test or --test-only.",
    )
    parser.add_argument(
        "--test-regex",
        help="Run only CTest tests whose names match this regular expression; requires --test or --test-only.",
    )
    parser.add_argument(
        "--exclude-test-regex",
        action="append",
        help="Exclude CTest tests whose names match this regular expression; requires --test or --test-only.",
    )
    parser.add_argument("--wheel", action="store_true", help="Assemble Python wheels after building.")
    parser.add_argument(
        "--validate-wheelhouse",
        type=Path,
        help="Install and validate an existing wheelhouse without configuring or building libraries.",
    )
    parser.add_argument(
        "--install-metrics-output",
        type=Path,
        help="Write metrics from --validate-wheelhouse to this JSON path.",
    )
    parser.add_argument(
        "--group",
        action="append",
        default=[],
        help="Wheel distribution to assemble; repeat as needed.",
    )
    parser.add_argument("--output-dir", type=Path, help="Wheel output directory.")
    parser.add_argument(
        "--stage-wheel",
        "--carrier",
        action="append",
        default=[],
        help="Build and stage a library wheel in a Kit extension; repeat as needed.",
    )
    parser.add_argument(
        "--pull-only",
        action="store_true",
        help="Populate Packman dependencies and every supported Pixi environment, then exit.",
    )
    parser.add_argument(
        "--no-pull",
        action="store_true",
        help="Require current local dependency environments without network access.",
    )
    parser.add_argument(
        "--dependency-profile",
        choices=("locked", "minimum"),
        default="locked",
        help="Select locked or minimum Python build/test dependencies.",
    )
    arguments = parser.parse_args()
    packaging_requested = bool(arguments.wheel or arguments.stage_wheel)
    if arguments.jobs is not None and arguments.jobs < 1:
        parser.error("--jobs must be greater than zero")
    if arguments.test_jobs is not None and arguments.test_jobs < 1:
        parser.error("--test-jobs must be greater than zero")
    if arguments.test_timeout is not None and arguments.test_timeout < 1:
        parser.error("--test-timeout must be greater than zero")
    if arguments.test and arguments.test_only:
        parser.error("--test and --test-only are mutually exclusive")
    if arguments.pull_only and (
        arguments.clean
        or arguments.rebuild
        or arguments.generate
        or arguments.target
        or arguments.test
        or arguments.test_only
        or arguments.coverage
        or packaging_requested
        or arguments.group
        or arguments.output_dir
        or arguments.junit_output
        or arguments.test_timeout
        or arguments.test_jobs
        or arguments.test_regex
        or arguments.exclude_test_regex
        or arguments.jobs
        or arguments.no_pull
        or arguments.validate_wheelhouse
        or arguments.install_metrics_output
    ):
        parser.error("--pull-only cannot be combined with build, clean, packaging, or --no-pull options")
    if arguments.generate and (arguments.test or arguments.test_only or packaging_requested):
        parser.error("--generate cannot be combined with --test, --test-only, --wheel, or --stage-wheel")
    if arguments.clean and (
        arguments.generate
        or arguments.test
        or arguments.test_only
        or arguments.coverage
        or packaging_requested
        or arguments.target
    ):
        parser.error("--clean cannot be combined with build actions")
    if arguments.test_only and (
        arguments.rebuild or arguments.target or arguments.coverage or packaging_requested or arguments.jobs
    ):
        parser.error("--test-only cannot be combined with --rebuild, --target, --coverage, packaging, or --jobs")
    if arguments.group and not arguments.wheel:
        parser.error("--group requires --wheel")
    if arguments.install_metrics_output is not None and arguments.validate_wheelhouse is None:
        parser.error("--install-metrics-output requires --validate-wheelhouse")
    if arguments.validate_wheelhouse is not None and (
        arguments.clean
        or arguments.rebuild
        or arguments.generate
        or arguments.target
        or arguments.test
        or arguments.test_only
        or arguments.coverage
        or packaging_requested
        or arguments.group
        or arguments.output_dir
        or arguments.junit_output
        or arguments.test_timeout
        or arguments.test_jobs
        or arguments.test_regex
        or arguments.exclude_test_regex
        or arguments.jobs
    ):
        parser.error("--validate-wheelhouse cannot be combined with build, test, or packaging actions")
    if arguments.output_dir and not packaging_requested:
        parser.error("--output-dir requires --wheel or --stage-wheel")
    if arguments.release and arguments.debug and packaging_requested:
        parser.error("--wheel and --stage-wheel accept one configuration at a time")
    if arguments.target and packaging_requested:
        parser.error("--target cannot be combined with --wheel or --stage-wheel")
    if packaging_requested and arguments.profile != "standard":
        parser.error("--wheel and --stage-wheel require --profile standard")
    if (arguments.test or arguments.test_only) and not _BUILD_PROFILES[arguments.profile].testing_enabled:
        parser.error(f"--test is unavailable for the {arguments.profile} profile")
    if (
        arguments.junit_output
        or arguments.test_timeout
        or arguments.test_jobs
        or arguments.test_regex
        or arguments.exclude_test_regex
    ) and not (arguments.test or arguments.test_only):
        parser.error(
            "--junit-output, --test-timeout, --test-jobs, --test-regex, and --exclude-test-regex require "
            "--test or --test-only"
        )
    if arguments.dependency_profile != "locked" and not _BUILD_PROFILES[arguments.profile].python_enabled:
        parser.error("--dependency-profile minimum requires a Python-enabled build profile")
    return arguments


def _get_build_directory(profile_name: str, configuration: str, dependency_profile: str = "locked") -> Path:
    """Return the canonical binary directory for a build selection.

    Args:
        profile_name: Supported build profile name.
        configuration: CMake build type.
        dependency_profile: Python dependency compatibility selection.

    Returns:
        Binary directory below the repository's `_cmake_build` root.
    """
    configuration_name = configuration.lower()
    if profile_name == "standard":
        suffix = configuration_name
    elif configuration == "Release":
        suffix = profile_name
    else:
        suffix = f"{profile_name}-{configuration_name}"
    if dependency_profile != "locked":
        suffix = f"{suffix}-{dependency_profile}"
    return LIBRARY_BUILD_ROOT / f"isaacsim-libraries-{suffix}"


def _main() -> int:
    """Build all selected configurations.

    Returns:
        The resulting value.
    """
    arguments = _parse_arguments()
    packaging_requested = bool(arguments.wheel or arguments.stage_wheel)
    exclude_test_regex = None
    if arguments.exclude_test_regex:
        exclude_test_regex = arguments.exclude_test_regex[0]
        if len(arguments.exclude_test_regex) > 1:
            exclude_test_regex = "|".join(f"({pattern})" for pattern in arguments.exclude_test_regex)
    configurations = []
    if arguments.release or not arguments.debug:
        configurations.append("Release")
    if arguments.debug:
        configurations.append("Debug")

    build_directories = {
        configuration: _get_build_directory(arguments.profile, configuration, arguments.dependency_profile)
        for configuration in configurations
    }
    if arguments.clean:
        for build_directory in build_directories.values():
            with _developer_environment_lock(build_directory, exclusive=True):
                if build_directory.exists():
                    print(f"Removing {build_directory}")
                    shutil.rmtree(build_directory)
        return 0

    profile = _BUILD_PROFILES[arguments.profile]
    host = _detect_host_platform()
    if not arguments.no_pull:
        _pull_dependencies(host)
    if arguments.pull_only:
        return 0

    if arguments.rebuild:
        for build_directory in build_directories.values():
            with _developer_environment_lock(build_directory, exclusive=True):
                if build_directory.exists():
                    print(f"Removing {build_directory}")
                    shutil.rmtree(build_directory)

    extension_distributions = {extension: _load_wheel_distribution(extension) for extension in arguments.stage_wheel}
    staged_wheel_variants = {
        extension_distributions[extension]: variant
        for extension in arguments.stage_wheel
        if (variant := _load_wheel_variant(extension)) is not None
    }
    wheel_groups = list(dict.fromkeys((*arguments.group, *extension_distributions.values())))
    pixi_build_prefix = _read_pixi_dependency_path(PIXI_BUILD_PREFIX_VARIABLE)
    cmake = _find_build_tool(pixi_build_prefix, host, "cmake")

    native_runtime_dependencies = _read_pixi_dependency_path(PIXI_NATIVE_RUNTIME_DEPS_VARIABLE)

    if profile.python_enabled:
        python = Path(sys.executable)
        python_runtime_dependencies = _read_pixi_dependency_path(PIXI_RUNTIME_DEPS_VARIABLE)
        python_schema_build_dependencies = _read_pixi_dependency_path(PIXI_SCHEMA_BUILD_DEPS_VARIABLE)
        python_test_dependencies = _read_pixi_dependency_path(PIXI_TEST_DEPS_VARIABLE)
    else:
        python = None
        python_runtime_dependencies = None
        python_schema_build_dependencies = None
        python_test_dependencies = None

    if arguments.validate_wheelhouse is not None:
        if python is None or python_runtime_dependencies is None or python_schema_build_dependencies is None:
            raise RuntimeError("Wheelhouse validation requires the module Python dependency environments")
        metrics_output = arguments.install_metrics_output or (
            arguments.validate_wheelhouse / "wheel_install_metrics.json"
        )
        _validate_wheelhouse(
            python,
            arguments.validate_wheelhouse,
            metrics_output,
            python_runtime_dependencies,
            python_schema_build_dependencies,
        )
        return 0

    python_build_dependencies = None
    if packaging_requested:
        if python is None:
            raise RuntimeError("Wheel assembly requires the module Python executable")
        python_build_dependencies = _read_pixi_dependency_path(PIXI_WHEEL_BUILD_DEPS_VARIABLE)

    msvc_compiler = (
        ensure_windows_msvc_environment(require_debug_runtime="Debug" in configurations)
        if host.name == "windows-x86_64"
        else None
    )

    if arguments.test_only:
        for configuration, build_directory in build_directories.items():
            producer_root = _validate_artifact_state(
                build_directory,
                configuration=configuration,
                profile=arguments.profile,
                dependency_profile=arguments.dependency_profile,
            )
            _relocate_cmake_metadata(build_directory, producer_root, REPOSITORY_ROOT)
            if arguments.profile == "standard" and arguments.dependency_profile == "locked":
                _ensure_developer_environment(cmake, build_directory, configuration)
            junit_output = arguments.junit_output
            if junit_output is not None and len(build_directories) > 1:
                junit_output = junit_output.with_name(
                    f"{junit_output.stem}-{configuration.lower()}{junit_output.suffix}"
                )
            _run_tests(
                cmake,
                build_directory,
                configuration,
                junit_output,
                arguments.test_timeout,
                arguments.test_jobs,
                arguments.test_regex,
                exclude_test_regex,
            )
        return 0

    ninja = _find_build_tool(pixi_build_prefix, host, "ninja")
    python_license_file = _read_pixi_license_file()

    for configuration, build_directory in build_directories.items():
        if arguments.output_dir:
            artifact_directory = arguments.output_dir.resolve()
        elif arguments.stage_wheel:
            artifact_directory = EXTENSION_WHEEL_ARTIFACT_ROOT / configuration.lower()
        else:
            artifact_directory = LIBRARY_ARTIFACT_ROOT / configuration.lower()
        extension_usd_root = None
        if "usd2511" in staged_wheel_variants.values():
            extension_usd_root = REPOSITORY_ROOT / "_build" / "target-deps" / "usd" / configuration.lower()
            if not (extension_usd_root / "lib" / "python" / "pxr" / "Usd" / "usdGenSchema.py").is_file():
                raise RuntimeError(
                    f"Kit OpenUSD schema generator is missing: {extension_usd_root}. "
                    "Run the repository dependency pull before preparing schema wheels."
                )
        context = _configure_context()
        refresh_reason = _configuration_refresh_reason(build_directory, context)
        if refresh_reason:
            print(f"Refreshing CMake configuration because {refresh_reason}.")
        (build_directory / ARTIFACT_STATE_FILE).unlink(missing_ok=True)
        build_started = time.perf_counter()
        configure_started = build_started
        _configure(
            cmake,
            ninja,
            python,
            build_directory,
            configuration,
            profile,
            native_runtime_dependencies,
            python_test_dependencies,
            python_runtime_dependencies,
            python_schema_build_dependencies,
            pixi_build_prefix,
            python_license_file,
            coverage_enabled=arguments.coverage,
            msvc_compiler=msvc_compiler,
            extension_usd_root=extension_usd_root,
            fresh=refresh_reason is not None,
        )
        configure_duration = time.perf_counter() - configure_started
        _write_json_atomically(build_directory / CONFIGURE_CONTEXT_FILE, context)
        if arguments.generate:
            continue
        compile_started = time.perf_counter()
        _build(cmake, build_directory, configuration, arguments.target, arguments.jobs)
        compile_duration = time.perf_counter() - compile_started
        developer_environment_components = None
        materialize_started = time.perf_counter()
        if arguments.profile == "standard" and arguments.dependency_profile == "locked" and arguments.target is None:
            developer_environment_components = _materialize_developer_environment(
                cmake,
                build_directory,
                configuration,
            )
        materialize_duration = time.perf_counter() - materialize_started
        _write_artifact_state(
            build_directory,
            configuration=configuration,
            profile=arguments.profile,
            dependency_profile=arguments.dependency_profile,
            target=arguments.target,
            developer_environment_components=developer_environment_components,
        )
        source_build_metrics = {
            "schema_version": 1,
            "configuration": configuration,
            "profile": arguments.profile,
            "rebuild": arguments.rebuild,
            "configure_duration_seconds": round(configure_duration, 6),
            "compile_duration_seconds": round(compile_duration, 6),
            "materialize_duration_seconds": round(materialize_duration, 6),
            "total_duration_seconds": round(time.perf_counter() - build_started, 6),
            **_directory_storage(build_directory),
        }
        _write_json_atomically(build_directory / SOURCE_BUILD_METRICS_FILE, source_build_metrics)
        print(
            f"Source library build: {source_build_metrics['total_duration_seconds']:.2f} seconds, "
            f"{source_build_metrics['disk_size_bytes'] / (1024 * 1024 * 1024):.2f} GiB on disk"
        )
        if arguments.test:
            junit_output = arguments.junit_output
            if junit_output is not None and len(build_directories) > 1:
                junit_output = junit_output.with_name(
                    f"{junit_output.stem}-{configuration.lower()}{junit_output.suffix}"
                )
            _run_tests(
                cmake,
                build_directory,
                configuration,
                junit_output,
                arguments.test_timeout,
                arguments.test_jobs,
                arguments.test_regex,
                exclude_test_regex,
            )
        if packaging_requested:
            if python is None or python_build_dependencies is None:
                raise RuntimeError("Wheel assembly requires the module Python dependency environment")
            if arguments.stage_wheel and arguments.output_dir is None:
                # Wheel artifacts are an internal handoff to `stage_extension_wheel.py`, not a persistent wheel index.
                # Remove outputs from earlier package shapes so a newly pure-Python (or newly native) wheel cannot
                # be mistaken for an ambiguous current build. Explicit output directories may intentionally collect
                # wheels across invocations and remain caller-owned.
                _PATHS.remove_generated_path(artifact_directory)
            _build_wheels(
                cmake,
                python,
                build_directory,
                configuration,
                wheel_groups,
                artifact_directory,
                python_build_dependencies,
                staged_wheel_variants,
            )
            if arguments.stage_wheel and arguments.output_dir is None:
                legacy_artifacts = LIBRARY_BUILD_ROOT / "module-carrier-artifacts" / configuration.lower()
                _PATHS.ensure_directory_alias(legacy_artifacts, artifact_directory)
        if arguments.stage_wheel:
            if python is None:
                raise RuntimeError("Wheel staging requires the module Python executable")
            for extension in arguments.stage_wheel:
                _stage_extension_wheel(
                    cmake,
                    python,
                    build_directory,
                    extension,
                    artifact_directory,
                    configuration.lower(),
                )
    return 0


if __name__ == "__main__":
    try:
        raise SystemExit(_main())
    except (RuntimeError, subprocess.CalledProcessError) as error:
        print(f"error: {error}", file=sys.stderr)
        raise SystemExit(1) from error
