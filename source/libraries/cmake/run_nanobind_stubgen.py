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

"""Run nanobind stubgen with Windows binding runtime directories enabled."""

from __future__ import annotations

import argparse
import importlib
import os
import runpy
import sys
from pathlib import Path

WINDOWS_DEBUG_RUNTIME_PATH_VARIABLE = "ISAACSIM_WINDOWS_DEBUG_RUNTIME_PATH"


def main() -> None:
    """Configure the import environment and run nanobind stubgen."""
    parser = argparse.ArgumentParser()
    parser.add_argument("--binding-dir", type=Path)
    parser.add_argument("--preload-module", action="append", default=[])
    parser.add_argument("stubgen", type=Path)
    arguments, stubgen_arguments = parser.parse_known_args()

    dll_directory_handles = []
    if sys.platform == "win32":
        if arguments.binding_dir is None:
            parser.error("--binding-dir is required on Windows")
        runtime_directories = [
            arguments.binding_dir,
            arguments.binding_dir / "plugins",
            arguments.binding_dir / "plugins" / "bin" / "deps",
        ]
        runtime_directories.extend(
            Path(value) for value in os.environ.get(WINDOWS_DEBUG_RUNTIME_PATH_VARIABLE, "").split(os.pathsep) if value
        )
        for runtime_directory in runtime_directories:
            if runtime_directory.is_dir():
                dll_directory_handles.append(os.add_dll_directory(runtime_directory.resolve()))

    # Stubgen consumes its import paths only after it starts. Make the same paths
    # available to prerequisite imports that register cross-module nanobind types.
    import_paths = []
    for index, argument in enumerate(stubgen_arguments):
        if argument in {"-i", "--import"} and index + 1 < len(stubgen_arguments):
            import_paths.append(stubgen_arguments[index + 1])
        elif argument.startswith("--import="):
            import_paths.append(argument.partition("=")[2])
    sys.path[:0] = import_paths
    for module_name in arguments.preload_module:
        importlib.import_module(module_name)

    sys.argv = [str(arguments.stubgen), *stubgen_arguments]
    runpy.run_path(str(arguments.stubgen), run_name="__main__")


if __name__ == "__main__":
    main()
