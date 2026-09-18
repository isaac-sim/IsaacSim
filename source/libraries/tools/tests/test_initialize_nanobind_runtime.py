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

"""Tests for nanobind package runtime initialization."""

from __future__ import annotations

import importlib.util
import tempfile
import unittest
from pathlib import Path
from unittest import mock

RUNTIME_INITIALIZER_PATH = Path(__file__).parents[2] / "cmake" / "initialize_nanobind_runtime.py"
RUNTIME_INITIALIZER_SPEC = importlib.util.spec_from_file_location(
    "initialize_nanobind_runtime",
    RUNTIME_INITIALIZER_PATH,
)
if RUNTIME_INITIALIZER_SPEC is None or RUNTIME_INITIALIZER_SPEC.loader is None:
    raise RuntimeError(f"Unable to load {RUNTIME_INITIALIZER_PATH}")
RUNTIME_INITIALIZER = importlib.util.module_from_spec(RUNTIME_INITIALIZER_SPEC)
RUNTIME_INITIALIZER_SPEC.loader.exec_module(RUNTIME_INITIALIZER)


class NanobindRuntimeInitializationTests(unittest.TestCase):
    """Test binding-local DLL directory registration."""

    def test_register_windows_dll_directories_keeps_existing_directory_handles(self) -> None:
        """Register binding-local and sibling runtime directories."""
        with tempfile.TemporaryDirectory() as temporary_directory:
            binding_directory = Path(temporary_directory) / "module" / "bindings"
            (binding_directory / "plugins" / "bin" / "deps").mkdir(parents=True)
            (binding_directory.parent / "plugins").mkdir()
            handles = [object(), object(), object(), object()]
            RUNTIME_INITIALIZER._DLL_DIRECTORY_HANDLES.clear()

            with (
                mock.patch.object(RUNTIME_INITIALIZER.sys, "platform", "win32"),
                mock.patch.object(
                    RUNTIME_INITIALIZER.os,
                    "add_dll_directory",
                    side_effect=handles,
                    create=True,
                ) as add_dll_directory,
            ):
                RUNTIME_INITIALIZER._register_windows_dll_directories(binding_directory)

        self.assertEqual(
            [call.args[0] for call in add_dll_directory.call_args_list],
            [
                binding_directory.resolve(),
                (binding_directory / "plugins").resolve(),
                (binding_directory.parent / "plugins").resolve(),
                (binding_directory / "plugins" / "bin" / "deps").resolve(),
            ],
        )
        self.assertEqual(RUNTIME_INITIALIZER._DLL_DIRECTORY_HANDLES, handles)

    def test_register_windows_dll_directories_skips_other_platforms(self) -> None:
        """Leave the DLL search path unchanged outside Windows."""
        with tempfile.TemporaryDirectory() as temporary_directory:
            with (
                mock.patch.object(RUNTIME_INITIALIZER.sys, "platform", "linux"),
                mock.patch.object(RUNTIME_INITIALIZER.os, "add_dll_directory", create=True) as add_dll_directory,
            ):
                RUNTIME_INITIALIZER._register_windows_dll_directories(Path(temporary_directory))

        add_dll_directory.assert_not_called()


if __name__ == "__main__":
    unittest.main()
