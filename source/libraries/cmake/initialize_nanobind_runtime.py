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

"""Initialize platform runtime search paths for an Isaac Sim nanobind package."""

from __future__ import annotations

import os
import sys
from pathlib import Path

__all__ = []

_DLL_DIRECTORY_HANDLES: list[object] = []


def _register_windows_dll_directories(binding_directory: Path) -> None:
    """Keep binding-local Windows DLL directories registered for this process.

    Args:
        binding_directory: Directory containing generated bindings.
    """
    if sys.platform != "win32":
        return

    for runtime_directory in (
        binding_directory,
        binding_directory / "plugins",
        binding_directory.parent / "plugins",
        binding_directory / "plugins" / "bin" / "deps",
    ):
        if runtime_directory.is_dir():
            _DLL_DIRECTORY_HANDLES.append(os.add_dll_directory(runtime_directory.resolve()))


_register_windows_dll_directories(Path(__file__).resolve().parent)
