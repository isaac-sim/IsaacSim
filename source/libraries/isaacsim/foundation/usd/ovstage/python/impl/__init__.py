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

import ctypes
import importlib.util
import os
import pathlib
import sys

import isaacsim.foundation.usd.openusd  # noqa: F401 - initialize OpenUSD bindings

__all__ = []
_dll_directory_handles: list[object] = []
_native_library_handles: list[ctypes.CDLL] = []


def _ovstage_root() -> pathlib.Path:
    """Get the installed ``ovstage`` dependency root without importing it."""
    spec = importlib.util.find_spec("ovstage")
    if spec is None or not spec.submodule_search_locations:
        raise RuntimeError("Required Python package 'ovstage' is not installed.")
    return pathlib.Path(next(iter(spec.submodule_search_locations))).resolve()


def _load_ovstage_runtime() -> None:
    """Load the pip-owned OVStage libraries before Isaac's native modules."""
    root = _ovstage_root()
    runtime = root / "bin"
    if sys.platform == "win32":
        directories = [runtime, runtime / "plugins"]
        plugins = runtime / "plugins"
        if plugins.is_dir():
            directories.extend(path for path in plugins.iterdir() if path.is_dir())
        for directory in directories:
            if directory.is_dir():
                _dll_directory_handles.append(os.add_dll_directory(directory))
        names = ("ovstage-dynamic.dll", "ovstage.dll")
    else:
        names = ("libovstage-dynamic.so", "libovstage.so")

    os.environ["OVSTAGE_LIBRARY_PATH_HINT"] = str(runtime)
    for name in names:
        path = runtime / name
        if not path.is_file():
            raise RuntimeError(f"Required native dependency was not found: {path}")
        if sys.platform == "win32":
            library = ctypes.CDLL(str(path))
        else:
            library = ctypes.CDLL(str(path), mode=ctypes.RTLD_GLOBAL)
        _native_library_handles.append(library)


_load_ovstage_runtime()
