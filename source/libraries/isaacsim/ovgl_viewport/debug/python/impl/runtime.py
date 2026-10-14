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

"""Initialize the OmniClient runtime owned by the ``ovstage`` dependency."""

from __future__ import annotations

import ctypes
import importlib
import importlib.util
import os
import sys
from pathlib import Path

_OMNICLIENT_LIBRARY_NAME = "omniclient.dll" if sys.platform == "win32" else "libomniclient.so"
_OMNIVERSE_CONNECTION_LIBRARY_NAME = (
    "omniverse_connection.dll" if sys.platform == "win32" else "libomniverse_connection.so"
)

_is_initialized = False
_dll_directory_handles: list[object] = []
_native_library_handles: list[ctypes.CDLL] = []


def _ovstage_root() -> Path:
    """Get the installed ``ovstage`` package root without importing it.

    Returns:
        Installed ``ovstage`` package directory.

    Raises:
        RuntimeError: If the required package is not installed.
    """
    spec = importlib.util.find_spec("ovstage")
    if spec is None or not spec.submodule_search_locations:
        raise RuntimeError("Required Python package 'ovstage' is not installed.")
    return Path(next(iter(spec.submodule_search_locations))).resolve()


def _load_library(path: Path) -> None:
    """Load and retain one dependency-owned native library.

    Args:
        path: Absolute path to the native library.

    Raises:
        RuntimeError: If the required library is missing.
    """
    if not path.is_file():
        raise RuntimeError(f"Required native dependency was not found: {path}")
    try:
        if sys.platform == "win32":
            library = ctypes.CDLL(str(path))
        else:
            library = ctypes.CDLL(str(path), mode=ctypes.RTLD_GLOBAL)
    except OSError as error:
        raise RuntimeError(f"Failed to load required native dependency: {path}") from error
    _native_library_handles.append(library)


def setup() -> None:
    """Load OVStage and its OmniClient libraries before the viewport binding.

    Raises:
        RuntimeError: If the required OVStage or OmniClient runtime is missing.
    """
    global _is_initialized
    if _is_initialized:
        return

    # The Foundation adapter owns OVStage initialization, including its loader
    # hint and complete Windows plugin search path.
    importlib.import_module("isaacsim.foundation.usd.ovstage")

    omni_client_directory = _ovstage_root() / "bin" / "plugins" / "omni.client.lib"
    if sys.platform == "win32":
        if not omni_client_directory.is_dir():
            raise RuntimeError(f"Required native dependency directory was not found: {omni_client_directory}")
        _dll_directory_handles.append(os.add_dll_directory(omni_client_directory))

    _load_library(omni_client_directory / _OMNIVERSE_CONNECTION_LIBRARY_NAME)
    _load_library(omni_client_directory / _OMNICLIENT_LIBRARY_NAME)
    _is_initialized = True
