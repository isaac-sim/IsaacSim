# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0

"""Test the isolated nanobind stub-generation launcher."""

from __future__ import annotations

import importlib.util
import tempfile
import unittest
from pathlib import Path
from unittest import mock

LAUNCHER_PATH = Path(__file__).resolve().parents[2] / "cmake" / "run_nanobind_stubgen.py"
LAUNCHER_SPEC = importlib.util.spec_from_file_location("isaacsim_nanobind_stubgen", LAUNCHER_PATH)
if LAUNCHER_SPEC is None or LAUNCHER_SPEC.loader is None:
    raise RuntimeError(f"Cannot load nanobind stub launcher from {LAUNCHER_PATH}")
launcher = importlib.util.module_from_spec(LAUNCHER_SPEC)
LAUNCHER_SPEC.loader.exec_module(launcher)


class NanobindStubgenTests(unittest.TestCase):
    """Test stub-generation import and runtime setup."""

    def test_preloads_module_from_stubgen_import_path(self) -> None:
        """Register dependency types before stubgen imports the target binding."""
        with tempfile.TemporaryDirectory() as temporary_directory:
            root = Path(temporary_directory)
            import_directory = root / "python"
            import_directory.mkdir()
            stubgen = root / "stubgen.py"

            with (
                mock.patch.object(
                    launcher.sys,
                    "argv",
                    [
                        "launcher",
                        "--binding-dir",
                        str(root),
                        "--preload-module",
                        "example.types",
                        str(stubgen),
                        "-i",
                        str(import_directory),
                        "-q",
                    ],
                ),
                mock.patch.object(launcher.sys, "path", ["existing"]),
                mock.patch.object(launcher.importlib, "import_module") as import_module,
                mock.patch.object(launcher.runpy, "run_path") as run_path,
            ):
                launcher.main()

                self.assertEqual(launcher.sys.path[:2], [str(import_directory), "existing"])

            import_module.assert_called_once_with("example.types")
            run_path.assert_called_once_with(str(stubgen), run_name="__main__")

    def test_windows_debug_runtime_is_registered_before_stubgen(self) -> None:
        """Add the binding and portable Debug CRT directories to the DLL search path."""
        with tempfile.TemporaryDirectory() as temporary_directory:
            root = Path(temporary_directory)
            binding_directory = root / "bindings"
            debug_runtime = root / "debug-runtime"
            binding_directory.mkdir()
            debug_runtime.mkdir()
            stubgen = root / "stubgen.py"

            with (
                mock.patch.object(launcher.sys, "platform", "win32"),
                mock.patch.object(
                    launcher.sys,
                    "argv",
                    ["launcher", "--binding-dir", str(binding_directory), str(stubgen), "-q"],
                ),
                mock.patch.dict(
                    launcher.os.environ,
                    {launcher.WINDOWS_DEBUG_RUNTIME_PATH_VARIABLE: str(debug_runtime)},
                    clear=False,
                ),
                mock.patch.object(launcher.os, "add_dll_directory", create=True) as add_dll_directory,
                mock.patch.object(launcher.runpy, "run_path") as run_path,
            ):
                launcher.main()

            self.assertEqual(
                [call.args[0] for call in add_dll_directory.call_args_list],
                [binding_directory.resolve(), debug_runtime.resolve()],
            )
            run_path.assert_called_once_with(str(stubgen), run_name="__main__")


if __name__ == "__main__":
    unittest.main()
