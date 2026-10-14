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

"""Prepare generated public inputs required by the repository build."""

from __future__ import annotations

import argparse
import importlib
import importlib.util
import subprocess
import sys
from collections.abc import Callable, Sequence
from pathlib import Path
from typing import Any

REPOSITORY_ROOT = Path(__file__).resolve().parents[2]
PREPARE_TOOL = REPOSITORY_ROOT / "source" / "libraries" / "tools" / "prepare_root_build.py"
LIBRARY_BUILD_TOOL = PREPARE_TOOL.with_name("build.py")
WINDOWS_TOOLCHAIN_DIRECTORY = REPOSITORY_ROOT / "_build" / "host-deps"
ASSEMBLE_NOTICE_TOOL = REPOSITORY_ROOT / "tools" / "isaac_build" / "assemble_third_party_notice.py"


def _activate_packaged_windows_debug_runtime() -> None:
    """Expose portable Debug runtimes to Kit tools launched by Repo Build."""
    module_name = "isaacsim_library_build_tool_for_repo"
    module = sys.modules.get(module_name)
    if module is None:
        spec = importlib.util.spec_from_file_location(module_name, LIBRARY_BUILD_TOOL)
        if spec is None or spec.loader is None:
            raise RuntimeError(f"Cannot load library build tool from {LIBRARY_BUILD_TOOL}")
        module = importlib.util.module_from_spec(spec)
        sys.modules[module_name] = module
        spec.loader.exec_module(module)
    module.activate_packaged_windows_debug_runtime(
        WINDOWS_TOOLCHAIN_DIRECTORY / "msvc",
        WINDOWS_TOOLCHAIN_DIRECTORY / "winsdk",
    )


def _get_build_arguments(arguments: Sequence[str] | None = None) -> tuple[str, ...]:
    """Return arguments following the Repo ``build`` command."""
    if arguments is None:
        arguments = sys.argv
    try:
        build_index = arguments.index("build")
    except ValueError as error:
        raise RuntimeError("Cannot locate the Repo build command in the process arguments") from error
    return tuple(arguments[build_index + 1 :])


class _BuildPreparation:
    """Prepare generated build inputs once at the latest safe Repo Build step."""

    def __init__(self, *, prepare_on_host: bool = False) -> None:
        self._prepare_on_host = prepare_on_host
        self._prepared = False

    def run(self) -> None:
        """Stage extension wheels and assemble notices after Repo Build links its toolchain."""
        if self._prepared:
            return

        build_arguments = list(_get_build_arguments())
        if self._prepare_on_host and "--no-docker" not in build_arguments:
            build_arguments.append("--no-docker")
        subprocess.run(
            [sys.executable, "-s", PREPARE_TOOL, "--repo-entry", *build_arguments],
            cwd=REPOSITORY_ROOT,
            check=True,
        )
        if (
            sys.platform == "win32"
            and (WINDOWS_TOOLCHAIN_DIRECTORY / "msvc").is_dir()
            and (WINDOWS_TOOLCHAIN_DIRECTORY / "winsdk").is_dir()
        ):
            _activate_packaged_windows_debug_runtime()
        subprocess.run(
            [sys.executable, "-s", ASSEMBLE_NOTICE_TOOL],
            cwd=REPOSITORY_ROOT,
            check=True,
        )
        self._prepared = True


def setup_repo_tool(
    parser: argparse.ArgumentParser, config: dict[str, Any]
) -> Callable[[argparse.Namespace, dict[str, Any]], None]:
    """Set up Repo Build with generated-input preparation at build lifecycle boundaries."""
    repo_build = importlib.import_module("omni.repo.build.main")

    repo_build.TOOL_CONFIG = config.get("repo_build", {})
    repo_build.setup_argument_parser(parser)

    def run_repo_tool(options: argparse.Namespace, merged_config: dict[str, Any]) -> None:
        repo_folders = merged_config["repo"]["folders"]
        # Repo Build omits `use_docker` when containerized builds are disabled
        # by repository configuration.
        preparation = _BuildPreparation(prepare_on_host=not getattr(options, "use_docker", False))

        if options.clean and not options.rebuild:
            preparation.run()

        def pull_dependencies(**kwargs: Any) -> dict[str, Any]:
            dependencies = repo_build.pull_dependencies(**kwargs)
            if options.fetch_only:
                preparation.run()
            return dependencies

        def stage_files(**kwargs: Any) -> None:
            preparation.run()
            repo_build.stage_files(**kwargs)

        def build(**kwargs: Any) -> None:
            preparation.run()
            repo_build.build(**kwargs)

        repo_build.run_build(
            options,
            repo_folders,
            None,
            merged_config,
            pull_dependencies_fn=pull_dependencies,
            stage_files_fn=stage_files,
            build_fn=build,
        )

    return run_repo_tool
