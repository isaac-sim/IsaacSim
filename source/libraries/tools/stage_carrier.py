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

"""Stage one built module distribution for consumption by a Kit carrier extension."""

from __future__ import annotations

import argparse
import importlib.util
import os
import platform
import re
import shutil
import subprocess
import sys
import sysconfig
import tempfile
import tomllib
from dataclasses import dataclass
from pathlib import Path
from typing import Any

_MANIFEST_CONTRACT_PATH = Path(__file__).resolve().parents[1] / "package_manifest.py"
_MANIFEST_CONTRACT_SPEC = importlib.util.spec_from_file_location("isaacsim_package_manifest", _MANIFEST_CONTRACT_PATH)
if _MANIFEST_CONTRACT_SPEC is None or _MANIFEST_CONTRACT_SPEC.loader is None:
    raise RuntimeError(f"Cannot load package manifest contract: {_MANIFEST_CONTRACT_PATH}")
_MANIFEST_CONTRACT = importlib.util.module_from_spec(_MANIFEST_CONTRACT_SPEC)
_MANIFEST_CONTRACT_SPEC.loader.exec_module(_MANIFEST_CONTRACT)

_EXTENSION_NAME_PATTERN = re.compile(r"^[a-z][a-z0-9_]*(?:\.[a-z][a-z0-9_]*)+$")
_DISTRIBUTION_NAME_PATTERN = re.compile(r"^[a-z][a-z0-9_]*$")
_PYTHON_MODULE_PATTERN = re.compile(r"^[A-Za-z_][A-Za-z0-9_]*(?:\.[A-Za-z_][A-Za-z0-9_]*)*$")
_PIP_LICENSE_FILENAME = "isaacsim-libraries-PIP-LICENSES.txt"


@dataclass(frozen=True, slots=True)
class _Arguments:
    """Validated carrier staging arguments."""

    repo_root: Path
    extension: str
    artifacts_directory: Path
    build_directory: Path
    configuration: str
    cmake: Path


def _read_carrier_distribution(path: Path) -> str:
    """Return the distribution declared by one carrier manifest.

    Args:
        path: Filesystem path to process.

    Returns:
        The resulting value.
    """
    with path.open("rb") as stream:
        document = tomllib.load(stream)
    carrier = document.get("carrier")
    distribution = carrier.get("distribution") if isinstance(carrier, dict) else None
    if not isinstance(distribution, str) or _DISTRIBUTION_NAME_PATTERN.fullmatch(distribution) is None:
        raise RuntimeError(f"{path} must define a lowercase underscore-separated [carrier].distribution")
    return distribution


def _read_carrier_wheel_variant(path: Path) -> str | None:
    """Return the optional wheel payload variant selected by one carrier.

    Args:
        path: Filesystem path to process.

    Returns:
        The resulting value.
    """
    with path.open("rb") as stream:
        document = tomllib.load(stream)
    carrier = document.get("carrier")
    variant = carrier.get("wheel-variant") if isinstance(carrier, dict) else None
    if variant is not None and (not isinstance(variant, str) or re.fullmatch(r"[a-z][a-z0-9]*", variant) is None):
        raise RuntimeError(f"{path} [carrier].wheel-variant must be a lowercase identifier")
    return variant


def _read_carrier_python_modules(path: Path) -> tuple[str, ...]:
    """Return the optional Python module projection declared by a carrier.

    Args:
        path: Filesystem path to process.

    Returns:
        The resulting value.
    """
    with path.open("rb") as stream:
        document = tomllib.load(stream)
    carrier = document.get("carrier")
    python_modules = carrier.get("python-modules") if isinstance(carrier, dict) else None
    if python_modules is None:
        return ()
    if (
        not isinstance(python_modules, list)
        or not python_modules
        or not all(isinstance(module, str) and _PYTHON_MODULE_PATTERN.fullmatch(module) for module in python_modules)
        or len(set(python_modules)) != len(python_modules)
    ):
        raise RuntimeError(f"{path} [carrier].python-modules must be a non-empty list of unique Python module names")
    for index, module in enumerate(python_modules):
        for other in python_modules[index + 1 :]:
            if module.startswith(f"{other}.") or other.startswith(f"{module}."):
                raise RuntimeError(f"{path} contains overlapping Python module projections: {module!r} and {other!r}")
    return tuple(python_modules)


def _read_carrier_dependency_extensions(path: Path) -> dict[str, tuple[str, ...]]:
    """Return Kit extensions declared as providers for uncarried package dependencies.

    Args:
        path: Filesystem path to process.

    Returns:
        The resulting value.
    """
    with path.open("rb") as stream:
        document = tomllib.load(stream)
    carrier = document.get("carrier")
    dependency_extensions = carrier.get("dependency-extensions") if isinstance(carrier, dict) else None
    if dependency_extensions is None:
        return {}
    if not isinstance(dependency_extensions, dict):
        raise RuntimeError(f"{path} [carrier].dependency-extensions must be a table")

    validated: dict[str, tuple[str, ...]] = {}
    for distribution, extensions in dependency_extensions.items():
        if (
            not isinstance(distribution, str)
            or _DISTRIBUTION_NAME_PATTERN.fullmatch(distribution) is None
            or not isinstance(extensions, list)
            or not extensions
            or not all(
                isinstance(extension, str) and _EXTENSION_NAME_PATTERN.fullmatch(extension) for extension in extensions
            )
            or len(set(extensions)) != len(extensions)
        ):
            raise RuntimeError(
                f"{path} [carrier].dependency-extensions entries must map lowercase underscore-separated "
                "distribution names to non-empty lists of unique extension names"
            )
        validated[distribution] = tuple(extensions)
    return validated


def _parse_arguments() -> _Arguments:
    """Parse command-line arguments.

    Returns:
        The resulting value.
    """
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--repo-root", type=Path, required=True)
    parser.add_argument("--extension", required=True)
    parser.add_argument("--artifacts-dir", type=Path, required=True)
    parser.add_argument("--build-dir", type=Path, required=True)
    parser.add_argument("--config", choices=("debug", "release"), required=True)
    parser.add_argument("--cmake", type=Path, required=True)
    parsed = parser.parse_args()
    return _Arguments(
        repo_root=parsed.repo_root.resolve(),
        extension=parsed.extension,
        artifacts_directory=parsed.artifacts_dir.resolve(),
        build_directory=parsed.build_dir.resolve(),
        configuration=parsed.config,
        cmake=parsed.cmake.resolve(),
    )


def _load_carrier_manifest(arguments: _Arguments) -> tuple[Path, str]:
    """Return the carrier extension root and its distribution name.

    Args:
        arguments: Parsed command-line arguments.

    Returns:
        The resulting value.
    """
    if _EXTENSION_NAME_PATTERN.fullmatch(arguments.extension) is None:
        raise RuntimeError(f"Invalid carrier extension name: {arguments.extension}")
    extension_root = arguments.repo_root / "source" / "extensions" / arguments.extension
    path = extension_root / "module-carrier.toml"
    if not path.is_file():
        raise RuntimeError(f"Carrier manifest is missing: {path}")
    return extension_root, _read_carrier_distribution(path)


def _carrier_extensions_by_distribution(repo_root: Path) -> dict[str, tuple[str, ...]]:
    """Return carrier extensions grouped by distribution and validate projections.

    Args:
        repo_root: Repository root directory.

    Returns:
        The resulting value.
    """
    carrier_entries: dict[str, list[tuple[str, tuple[str, ...]]]] = {}
    extension_root = repo_root / "source" / "extensions"
    for path in sorted(extension_root.glob("*/module-carrier.toml")):
        extension_name = path.parent.name
        if _EXTENSION_NAME_PATTERN.fullmatch(extension_name) is None:
            raise RuntimeError(f"Invalid carrier extension name: {extension_name}")
        distribution = _read_carrier_distribution(path)
        carrier_entries.setdefault(distribution, []).append((extension_name, _read_carrier_python_modules(path)))

    extensions_by_distribution: dict[str, tuple[str, ...]] = {}
    for distribution, entries in carrier_entries.items():
        if len(entries) > 1 and any(not python_modules for _, python_modules in entries):
            extensions = ", ".join(repr(extension) for extension, _ in entries)
            raise RuntimeError(
                f"Carrier distribution {distribution!r} is shared by {extensions}; every carrier must declare "
                "[carrier].python-modules"
            )
        projected_modules: dict[str, str] = {}
        for extension, python_modules in entries:
            for module in python_modules:
                for existing_module, existing_extension in projected_modules.items():
                    if (
                        module == existing_module
                        or module.startswith(f"{existing_module}.")
                        or existing_module.startswith(f"{module}.")
                    ):
                        raise RuntimeError(
                            f"Overlapping Python modules {existing_module!r} and {module!r} from distribution "
                            f"{distribution!r} are projected by {existing_extension!r} and {extension!r}"
                        )
                projected_modules[module] = extension
        extensions_by_distribution[distribution] = tuple(extension for extension, _ in entries)
    return extensions_by_distribution


def _validate_extension_dependencies(
    repo_root: Path,
    extension_root: Path,
    package_manifest: dict[str, Any],
) -> None:
    """Require internal package dependencies to have declared Kit extension providers.

    Args:
        repo_root: Repository root directory.
        extension_root: Carrier extension root.
        package_manifest: Package manifest metadata.
    """
    package_dependencies = package_manifest.get("dependencies")
    if not isinstance(package_dependencies, dict) or not all(
        isinstance(name, str) and isinstance(specifier, str) for name, specifier in package_dependencies.items()
    ):
        raise RuntimeError("Installed package manifest contains invalid dependencies")

    configuration_path = extension_root / "config" / "extension.toml"
    with configuration_path.open("rb") as stream:
        configuration = tomllib.load(stream)
    extension_dependencies = configuration.get("dependencies", {})
    if not isinstance(extension_dependencies, dict):
        raise RuntimeError(f"{configuration_path} must define [dependencies] as a table")

    carrier_extensions = _carrier_extensions_by_distribution(repo_root)
    dependency_extensions = _read_carrier_dependency_extensions(extension_root / "module-carrier.toml")
    for distribution in sorted(package_dependencies):
        required_extensions = carrier_extensions.get(distribution)
        dependency_kind = "carrier"
        if required_extensions is None:
            required_extensions = dependency_extensions.get(distribution)
            dependency_kind = "provider"
        if required_extensions is None:
            raise RuntimeError(
                f"Internal package dependency {distribution!r} has no registered carrier extension "
                "or declared dependency extension provider"
            )
        for dependency_extension in required_extensions:
            if dependency_extension not in extension_dependencies:
                raise RuntimeError(
                    f"{configuration_path} must declare {dependency_kind} dependency {dependency_extension!r} "
                    f"for internal package {distribution!r}"
                )


def _native_library_tokens(configuration: str) -> tuple[str, dict[str, str]]:
    """Return the Kit platform name and supported native-library path tokens.

    Args:
        configuration: Build configuration name.

    Returns:
        The resulting value.
    """
    machine = platform.machine().lower()
    if sys.platform == "win32" and machine in {"amd64", "x86_64"}:
        platform_name = "windows-x86_64"
        library_prefix = ""
        library_extension = ".dll"
    elif sys.platform.startswith("linux") and machine in {"amd64", "x86_64"}:
        platform_name = "linux-x86_64"
        library_prefix = "lib"
        library_extension = ".so"
    elif sys.platform.startswith("linux") and machine in {"aarch64", "arm64"}:
        platform_name = "linux-aarch64"
        library_prefix = "lib"
        library_extension = ".so"
    else:
        raise RuntimeError(f"Unsupported carrier host: {sys.platform}/{machine}")

    return platform_name, {
        "${config}": configuration,
        "${lib_ext}": library_extension,
        "${lib_prefix}": library_prefix,
        "${platform}": platform_name,
    }


def _resolve_native_library_path(entry: dict[str, Any], platform_name: str, tokens: dict[str, str]) -> str:
    """Resolve the active platform path and the Kit tokens used by carrier manifests.

    Args:
        entry: Manifest entry to expand.
        platform_name: Target platform name.
        tokens: Manifest substitution tokens.

    Returns:
        The resulting value.
    """
    path = entry.get("path")
    platform_filters = entry.get("filter:platform")
    if isinstance(platform_filters, dict):
        platform_override = platform_filters.get(platform_name)
        if isinstance(platform_override, dict) and "path" in platform_override:
            path = platform_override["path"]
    if not isinstance(path, str) or not path:
        raise RuntimeError("Every [[native.library]] entry must define a non-empty path")
    for token, value in tokens.items():
        path = path.replace(token, value)
    unresolved_tokens = re.findall(r"\$\{[^}]+\}", path)
    if unresolved_tokens:
        raise RuntimeError(f"Carrier native library path contains unsupported tokens {unresolved_tokens}: {path}")
    return path


def _validate_staged_native_libraries(extension_root: Path, stage_root: Path, configuration: str) -> None:
    """Require each carrier-owned native-library path to identify a staged file.

    Args:
        extension_root: Carrier extension root.
        stage_root: Carrier staging root.
        configuration: Build configuration name.
    """
    path = extension_root / "config" / "extension.toml"
    with path.open("rb") as stream:
        document = tomllib.load(stream)
    native = document.get("native")
    libraries = native.get("library", []) if isinstance(native, dict) else []
    if not isinstance(libraries, list):
        raise RuntimeError(f"{path} must define [[native.library]] as an array")

    platform_name, tokens = _native_library_tokens(configuration)
    for entry in libraries:
        if not isinstance(entry, dict):
            raise RuntimeError(f"{path} contains an invalid [[native.library]] entry")
        resolved_path = _resolve_native_library_path(entry, platform_name, tokens)
        relative_path = Path(resolved_path)
        if relative_path.is_absolute() or ".." in relative_path.parts:
            raise RuntimeError(f"Carrier native library path must remain inside the extension: {resolved_path}")
        if not relative_path.parts or relative_path.parts[0] != "pip_prebundle":
            continue
        if any(character in resolved_path for character in "*?["):
            raise RuntimeError(
                "Carrier native library paths must not contain wildcards for the supported Kit runtime: "
                f"{resolved_path}"
            )
        if not (stage_root / relative_path).is_file():
            raise RuntimeError(f"Staged native library is missing for {platform_name}: {resolved_path}")


def _read_version(arguments: _Arguments) -> str:
    """Read the shared module version.

    Args:
        arguments: Parsed command-line arguments.

    Returns:
        The resulting value.
    """
    version = (arguments.repo_root / "source" / "libraries" / "VERSION").read_text(encoding="utf-8").strip()
    if not version:
        raise RuntimeError("source/libraries/VERSION is empty")
    return version


def _platform_tag() -> str:
    """Return the normalized platform tag used by native wheels.

    Returns:
        The resulting value.
    """
    return sysconfig.get_platform().replace("-", "_").replace(".", "_")


def _select_single(directory: Path, pattern: str, description: str) -> Path:
    """Select exactly one artifact matching a versioned platform pattern.

    Args:
        directory: Directory to search.
        pattern: Glob pattern used for the search.
        description: Human-readable artifact description.

    Returns:
        The resulting value.
    """
    matches = sorted(directory.glob(pattern))
    if len(matches) != 1:
        raise RuntimeError(f"Expected one {description} matching {directory / pattern}, found {len(matches)}")
    return matches[0]


def _select_wheel(directory: Path, distribution: str, version: str, wheel_variant: str | None) -> Path:
    """Select the wheel built for this host or a platform-independent wheel.

    Args:
        directory: Directory containing wheel artifacts.
        distribution: Distribution filename prefix.
        version: Package version.
        wheel_variant: Optional pure-Python wheel variant.

    Returns:
        The single matching wheel artifact.
    """
    if wheel_variant is not None:
        patterns = (f"{distribution}-{version}-1{wheel_variant}-py3-none-any.whl",)
    else:
        patterns = (
            f"{distribution}-{version}-*-{_platform_tag()}.whl",
            f"{distribution}-{version}-py3-none-any.whl",
        )
    matches = sorted(path for pattern in patterns for path in directory.glob(pattern))
    if len(matches) != 1:
        expected = " or ".join(str(directory / pattern) for pattern in patterns)
        raise RuntimeError(f"Expected one Python wheel matching {expected}, found {len(matches)}")
    return matches[0]


def _load_package_manifest(path: Path, distribution: str, version: str) -> dict[str, Any]:
    """Load and validate an installed module package manifest.

    Args:
        path: Filesystem path to process.
        distribution: Python distribution name.
        version: Package version.

    Returns:
        The resulting value.
    """
    return _MANIFEST_CONTRACT.load_package_manifest(path, distribution, expected_version=version)


def _load_package_license(prefix: Path, distribution: str, artifact: str) -> bytes:
    """Load one installed package's nonempty Python dependency license aggregate.

    Args:
        prefix: Wheel filename prefix.
        distribution: Python distribution name.
        artifact: Wheel artifact path.

    Returns:
        The resulting value.
    """
    path = prefix / "share" / "licenses" / distribution / _PIP_LICENSE_FILENAME
    if not path.is_file():
        raise RuntimeError(f"{artifact} is missing the package license aggregate: {path}")
    payload = path.read_bytes()
    if not payload.strip():
        raise RuntimeError(f"{artifact} contains an empty package license aggregate: {path}")
    return payload


def _validate_matching_package_licenses(wheel_root: Path, sdk_root: Path, distribution: str) -> bytes:
    """Require the wheel and native SDK to carry the same dependency notices.

    Args:
        wheel_root: Directory containing built wheels.
        sdk_root: SDK root directory.
        distribution: Python distribution name.

    Returns:
        The resulting value.
    """
    wheel_license = _load_package_license(wheel_root, distribution, "Installed wheel")
    sdk_license = _load_package_license(sdk_root, distribution, "Installed native SDK")
    if wheel_license != sdk_license:
        raise RuntimeError("Wheel and native SDK package license aggregates do not match")
    return wheel_license


def _install_sdk(arguments: _Arguments, destination: Path, distribution: str) -> None:
    """Install one distribution's native runtime and development components.

    Args:
        arguments: Parsed command-line arguments.
        destination: Destination path.
        distribution: Python distribution name.
    """
    for component in (f"{distribution}-runtime", f"{distribution}-development"):
        subprocess.run(
            [
                str(arguments.cmake),
                "--install",
                str(arguments.build_directory),
                "--prefix",
                str(destination),
                "--component",
                component,
                "--config",
                arguments.configuration.capitalize(),
            ],
            check=True,
        )


def _install_wheel(wheel_path: Path, destination: Path, distribution: str, version: str) -> None:
    """Install a wheel through pip rather than interpreting the wheel format locally.

    Args:
        wheel_path: Path to the wheel artifact.
        destination: Destination path.
        distribution: Python distribution name.
        version: Package version.
    """
    with tempfile.TemporaryDirectory(prefix=".carrier-wheel-", dir=wheel_path.parent) as temporary_directory:
        wheel_index = Path(temporary_directory)
        indexed_wheel = wheel_index / wheel_path.name
        try:
            os.link(wheel_path, indexed_wheel)
        except OSError:
            shutil.copy2(wheel_path, indexed_wheel)
        subprocess.run(
            [
                sys.executable,
                "-s",
                "-m",
                "pip",
                "install",
                "--disable-pip-version-check",
                "--no-cache-dir",
                "--no-compile",
                "--no-deps",
                "--no-index",
                "--find-links",
                str(wheel_index),
                "--target",
                str(destination),
                f"{distribution}=={version}",
            ],
            check=True,
        )


def _project_python_modules(
    installed_wheel: Path,
    destination: Path,
    python_modules: tuple[str, ...],
    package_manifest: dict[str, Any],
    package_license: bytes,
) -> None:
    """Copy selected leaf packages and required notices into a carrier prebundle.

    Args:
        installed_wheel: Installed wheel metadata.
        destination: Destination path.
        python_modules: Python module names supplied by the package.
        package_manifest: Package manifest metadata.
        package_license: Path to the package license.
    """
    python_imports = package_manifest.get("python_imports")
    if not isinstance(python_imports, list) or not all(isinstance(module, str) for module in python_imports):
        raise RuntimeError("Installed package manifest contains invalid python_imports")
    for module in python_modules:
        if module not in python_imports:
            raise RuntimeError(f"Carrier projects Python module {module!r} absent from the installed distribution")
        relative_path = Path(*module.split("."))
        source = installed_wheel / relative_path
        if not source.is_dir() or not (source / "__init__.py").is_file():
            raise RuntimeError(f"Projected Python module is missing its leaf package: {source}")
        target = destination / relative_path
        target.parent.mkdir(parents=True, exist_ok=True)
        shutil.copytree(source, target)

    distribution = package_manifest.get("name")
    if not isinstance(distribution, str) or _DISTRIBUTION_NAME_PATTERN.fullmatch(distribution) is None:
        raise RuntimeError("Installed package manifest contains an invalid distribution name")
    license_source = installed_wheel / "share" / "licenses" / distribution / _PIP_LICENSE_FILENAME
    if not license_source.is_file():
        raise RuntimeError(f"Installed wheel is missing the package license aggregate: {license_source}")
    license_destination = destination / "share" / "licenses" / distribution / _PIP_LICENSE_FILENAME
    license_destination.parent.mkdir(parents=True, exist_ok=True)
    shutil.copy2(license_source, license_destination)
    projected_license = _load_package_license(destination, distribution, "Carrier Python prebundle")
    if projected_license != package_license:
        raise RuntimeError("Carrier Python prebundle package license aggregate does not match the wheel")


def _carrier_stage_directory(arguments: _Arguments) -> Path:
    """Return the persistent carrier handoff directory owned by the library build.

    Args:
        arguments: Parsed command-line arguments.

    Returns:
        The resulting value.
    """
    return arguments.repo_root / "_cmake_build" / "module-carriers" / arguments.configuration / arguments.extension


def _stage(arguments: _Arguments) -> Path:
    """Create and validate one configuration-specific carrier stage.

    Args:
        arguments: Parsed command-line arguments.

    Returns:
        The resulting value.
    """
    extension_root, distribution = _load_carrier_manifest(arguments)
    version = _read_version(arguments)
    carrier_manifest = extension_root / "module-carrier.toml"
    wheel_variant = _read_carrier_wheel_variant(carrier_manifest)
    python_modules = _read_carrier_python_modules(carrier_manifest)
    wheel = _select_wheel(arguments.artifacts_directory, distribution, version, wheel_variant)
    destination = _carrier_stage_directory(arguments)
    destination.parent.mkdir(parents=True, exist_ok=True)
    with tempfile.TemporaryDirectory(prefix=f".{arguments.extension}-", dir=destination.parent) as temporary:
        temporary_root = Path(temporary)
        pip_prebundle = temporary_root / "pip_prebundle"
        sdk_root = temporary_root / "sdk"
        wheel_install = temporary_root / "wheel-install" if python_modules else pip_prebundle
        _install_wheel(wheel, wheel_install, distribution, version)
        _install_sdk(arguments, sdk_root, distribution)

        manifest_relative = Path("share") / "isaacsim" / "packages" / distribution / "package.json"
        wheel_manifest = _load_package_manifest(wheel_install / manifest_relative, distribution, version)
        sdk_manifest = _load_package_manifest(sdk_root / manifest_relative, distribution, version)
        if wheel_manifest != sdk_manifest:
            raise RuntimeError("Wheel and native SDK package manifests do not match")
        wheel_license = _validate_matching_package_licenses(wheel_install, sdk_root, distribution)
        if python_modules:
            _project_python_modules(wheel_install, pip_prebundle, python_modules, wheel_manifest, wheel_license)
            shutil.rmtree(wheel_install)
        _validate_extension_dependencies(arguments.repo_root, extension_root, wheel_manifest)
        _validate_staged_native_libraries(extension_root, temporary_root, arguments.configuration)

        shutil.rmtree(destination, ignore_errors=True)
        temporary_root.rename(destination)

    print(destination)
    return destination


def _main() -> int:
    """Stage the requested carrier extension.

    Returns:
        The resulting value.
    """
    _stage(_parse_arguments())
    return 0


if __name__ == "__main__":
    try:
        raise SystemExit(_main())
    except (OSError, RuntimeError, subprocess.CalledProcessError) as error:
        print(f"error: {error}", file=sys.stderr)
        raise SystemExit(1) from error
