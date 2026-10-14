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

"""Build and stage library wheels in Kit extensions for the repository build."""

from __future__ import annotations

import importlib.util
import subprocess
import sys
from collections.abc import Sequence
from dataclasses import dataclass
from os import environ
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
LIBRARY_BUILD_LAUNCHER = LIBRARIES_DIRECTORY / ("build.bat" if sys.platform == "win32" else "build.sh")
LIBRARY_BUILD_ROOT = REPOSITORY_ROOT / "_cmake_build"
_REPOSITORY_ENTRY_ARGUMENT = "--repo-entry"
# Repo Build short options that take a value. Argparse binds the rest of a
# clustered token to the first of these, so `-rj8` means `-r -j 8`.
_VALUE_SHORT_FLAGS = frozenset("ejpt")
# Repo Build spells a few short options with more than one letter. They must be
# matched whole, otherwise `-mc` would expand to the unrelated `-m -c`.
_MULTI_CHARACTER_SHORT_OPTIONS = ("-mc", "-mpc")


@dataclass(frozen=True, slots=True)
class _BuildSelection:
    """Wheel staging work implied by one root build invocation."""

    configurations: tuple[str, ...]
    clean_only: bool = False
    dependencies_only: bool = False
    generate_only: bool = False
    rebuild: bool = False
    reuse_dependencies: bool = False
    coverage: bool = False
    skipped: bool = False


def _append_configuration(configurations: list[str], configuration: str) -> None:
    """Append one supported configuration without duplicating it.

    Args:
        configurations: Available build configurations.
        configuration: Build configuration name.
    """
    normalized = configuration.lower()
    if normalized in {"debug", "release"} and normalized not in configurations:
        configurations.append(normalized)


def _is_clustered_short_flags(argument: str) -> bool:
    """Return whether an argument packs several single-character options together.

    Args:
        argument: Command-line argument to locate.

    Returns:
        The resulting value.
    """
    return (
        len(argument) > 2
        and argument.startswith("-")
        and not argument.startswith("--")
        and argument[1].isalpha()
        and argument not in _MULTI_CHARACTER_SHORT_OPTIONS
    )


def _expand_short_flag_clusters(arguments: Sequence[str]) -> tuple[str, ...]:
    """Split clustered single-character options the way argparse tokenizes them.

    Repo Build parses its own command line with argparse, where ``-rx`` means
    ``-r -x``. This tool only inspects that same command line, so it has to
    reproduce the tokenization or it will silently miss options. Letters it does
    not recognize are kept as individual flags rather than rejected, because
    Repo Build and ``repo.toml`` are both free to add options this tool does not
    model.

    Args:
        arguments: Parsed command-line arguments.

    Returns:
        The resulting value.
    """
    expanded: list[str] = []
    for argument in arguments:
        if not _is_clustered_short_flags(argument):
            expanded.append(argument)
            continue
        for character in argument[1:]:
            if not character.isalpha():
                # A non-letter ends the cluster: the rest is an attached value.
                break
            expanded.append(f"-{character}")
            if character in _VALUE_SHORT_FLAGS:
                # Argparse binds the remainder of the token to this option.
                break
    return tuple(expanded)


def _parse_root_build_arguments(arguments: Sequence[str]) -> _BuildSelection:
    """Translate root repo-build arguments into library build behavior.

    Args:
        arguments: Parsed command-line arguments.

    Returns:
        The resulting value.
    """
    arguments = _expand_short_flag_clusters(arguments)
    if any(argument in {"-h", "--help", "-S", "--post-build-only", "--stage-only"} for argument in arguments):
        return _BuildSelection((), skipped=True)
    if "--fetch-only" in arguments:
        return _BuildSelection((), dependencies_only=True)

    configurations: list[str] = []
    index = 0
    while index < len(arguments):
        argument = arguments[index]
        if argument in {"-r", "--release", "--release-only"}:
            _append_configuration(configurations, "release")
        elif argument in {"-d", "--debug", "--debug-only"}:
            _append_configuration(configurations, "debug")
        elif argument == "--config":
            index += 1
            while index < len(arguments) and not arguments[index].startswith("-"):
                for value in arguments[index].replace(",", " ").split():
                    _append_configuration(configurations, value)
                index += 1
            index -= 1
        elif argument.startswith("--config="):
            for value in argument.partition("=")[2].replace(",", " ").split():
                _append_configuration(configurations, value)
        index += 1

    if not configurations:
        # Repo Build falls back to release alone, and only pulls that configuration's
        # dependencies. Preparing debug wheels here would need OpenUSD packages the
        # root build never fetched.
        configurations.append("release")

    clean_only = any(argument in {"-c", "--clean"} for argument in arguments)
    rebuild = any(argument in {"-x", "--rebuild"} for argument in arguments)
    return _BuildSelection(
        configurations=tuple(configurations),
        clean_only=clean_only and not rebuild,
        generate_only=any(argument in {"-g", "--generate"} for argument in arguments),
        rebuild=rebuild,
        reuse_dependencies=any(argument in {"-b", "--build-only"} for argument in arguments),
        coverage="--enable-gcov" in arguments,
    )


def _discover_wheel_staging_extensions(repository_root: Path = REPOSITORY_ROOT) -> tuple[str, ...]:
    """Return every extension that declares a wheel staging manifest.

    Args:
        repository_root: Repository root directory.

    Returns:
        The resulting value.
    """
    extension_root = repository_root / "source" / "extensions"
    return tuple(sorted(path.parent.name for path in _STAGING_MANIFEST.find_manifests(extension_root)))


def _should_prepare_repository_build(
    arguments: Sequence[str],
    *,
    platform_name: str = sys.platform,
    environment: dict[str, str] | None = None,
) -> bool:
    """Return whether this process is already in the repository's effective build environment.

    Args:
        arguments: Parsed command-line arguments.
        platform_name: Target platform name.
        environment: Process environment variables.

    Returns:
        The resulting value.
    """
    if not platform_name.startswith("linux"):
        return True
    effective_environment = environ if environment is None else environment
    return (
        "--no-docker" in arguments
        or "LINBUILD_EMBEDDED" in effective_environment
        or "OMNI_REPO_BUILD_NO_DOCKER" in effective_environment
    )


def _run_library_build(command: Sequence[object]) -> None:
    """Run one library build command from the repository root.

    Args:
        command: Command to execute.
    """
    command_strings = [str(value) for value in command]
    print(f"+ {subprocess.list2cmdline(command_strings)}", flush=True)
    subprocess.run(command_strings, cwd=REPOSITORY_ROOT, check=True)


def _clean_wheel_staging_outputs(configuration: str) -> None:
    """Remove generated wheel staging artifacts owned by one configuration.

    Args:
        configuration: Build configuration name.
    """
    for path in (
        LIBRARY_BUILD_ROOT / "extension-wheel-artifacts" / configuration,
        LIBRARY_BUILD_ROOT / "extension-wheel-stages" / configuration,
        LIBRARY_BUILD_ROOT / "module-carrier-artifacts" / configuration,
        LIBRARY_BUILD_ROOT / "module-carriers" / configuration,
    ):
        _PATHS.remove_generated_path(path)


def _ensure_wheel_stage_link_targets(configuration: str, extensions: Sequence[str]) -> None:
    """Create the wheel stage directories that Kit extensions link against.

    A generate-only build configures the modules without staging wheels, but
    the Kit extensions still declare an unconditional prebuild link into the
    wheel stage. Without these directories the root build's
    staging step fails before generation finishes.

    Args:
        configuration: Build configuration name.
        extensions: Extensions that stage wheels.
    """
    for extension in extensions:
        stage = LIBRARY_BUILD_ROOT / "extension-wheel-stages" / configuration / extension
        for relative_path in ("pip_prebundle", "sdk/include"):
            (stage / relative_path).mkdir(parents=True, exist_ok=True)
        legacy = LIBRARY_BUILD_ROOT / "module-carriers" / configuration / extension
        _PATHS.ensure_directory_alias(legacy, stage, preserve_directory=True)
        for relative_path in ("pip_prebundle", "sdk/include"):
            (legacy / relative_path).mkdir(parents=True, exist_ok=True)


def _prepare(arguments: Sequence[str]) -> None:
    """Build and stage wheels for every registered extension in the root build.

    Args:
        arguments: Parsed command-line arguments.
    """
    selection = _parse_root_build_arguments(arguments)
    if selection.skipped:
        return

    if selection.dependencies_only:
        _run_library_build((LIBRARY_BUILD_LAUNCHER, "--pull-only"))
        return

    extensions = _discover_wheel_staging_extensions()
    if not extensions:
        return

    for configuration in selection.configurations:
        configuration_flag = "-d" if configuration == "debug" else "-r"
        if selection.clean_only:
            _run_library_build((LIBRARY_BUILD_LAUNCHER, configuration_flag, "--clean"))
            _clean_wheel_staging_outputs(configuration)
            continue

        command: list[object] = [LIBRARY_BUILD_LAUNCHER, configuration_flag]
        if selection.rebuild:
            _clean_wheel_staging_outputs(configuration)
            command.append("--rebuild")
        if selection.reuse_dependencies:
            command.append("--no-pull")
        if selection.coverage:
            command.append("--coverage")
        if selection.generate_only:
            command.append("--generate")
            print(f"Configuring Kit-independent modules ({configuration})", flush=True)
        else:
            for extension in extensions:
                command.extend(("--stage-wheel", extension))
            print(f"Preparing extension wheels ({configuration}): {', '.join(extensions)}", flush=True)
        _run_library_build(command)
        if selection.generate_only:
            _ensure_wheel_stage_link_targets(configuration, extensions)


def _main() -> int:
    """Prepare extension wheels selected by the root build arguments.

    Returns:
        The resulting value.
    """
    arguments = sys.argv[1:]
    if arguments and arguments[0] == _REPOSITORY_ENTRY_ARGUMENT:
        arguments = arguments[1:]
        if not _should_prepare_repository_build(arguments):
            return 0
    _prepare(arguments)
    return 0


if __name__ == "__main__":
    try:
        raise SystemExit(_main())
    except (OSError, RuntimeError, subprocess.CalledProcessError) as error:
        print(f"error: {error}", file=sys.stderr)
        raise SystemExit(1) from error
