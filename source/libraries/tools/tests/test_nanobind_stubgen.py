# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0

"""Test the isolated nanobind stub-generation launcher."""

from __future__ import annotations

import importlib.util
import io
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
                        launcher.WINDOWS_ISOLATED_FLAG,
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
                    [
                        "launcher",
                        launcher.WINDOWS_ISOLATED_FLAG,
                        "--binding-dir",
                        str(binding_directory),
                        str(stubgen),
                        "-q",
                    ],
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

    def test_windows_runs_stubgen_with_private_standard_handles(self) -> None:
        """Delegate to a child process so a spawned daemon cannot hold the caller's pipes."""

        def run_child(command: list[str], **kwargs: object) -> mock.Mock:
            self.assertIs(kwargs["stdin"], launcher.subprocess.DEVNULL)
            for stream_name in ("stdout", "stderr"):
                self.assertIsInstance(kwargs[stream_name].fileno(), int)
            kwargs["stdout"].write(b"stub output\n")
            kwargs["stderr"].write(b"stub warning\n")
            return mock.Mock(returncode=3)

        stdout = io.TextIOWrapper(io.BytesIO(), encoding="utf-8")
        stderr = io.TextIOWrapper(io.BytesIO(), encoding="utf-8")
        with (
            mock.patch.object(launcher.sys, "platform", "win32"),
            mock.patch.object(launcher.sys, "argv", ["launcher", "--binding-dir", "bindings", "stubgen.py"]),
            mock.patch.object(launcher.sys, "stdout", stdout),
            mock.patch.object(launcher.sys, "stderr", stderr),
            mock.patch.object(launcher.subprocess, "run", side_effect=run_child) as run,
            mock.patch.object(launcher.runpy, "run_path") as run_path,
        ):
            with self.assertRaises(SystemExit) as raised:
                launcher.main()

        self.assertEqual(raised.exception.code, 3)
        run_path.assert_not_called()

        command = run.call_args.args[0]
        self.assertEqual(command[-4:], [launcher.WINDOWS_ISOLATED_FLAG, "--binding-dir", "bindings", "stubgen.py"])
        self.assertEqual(stdout.buffer.getvalue(), b"stub output\n")
        self.assertEqual(stderr.buffer.getvalue(), b"stub warning\n")

    def test_relays_output_to_streams_without_binary_buffer(self) -> None:
        """Decode captured output for text-only streams and skip missing streams."""
        with tempfile.TemporaryFile() as source:
            source.write(b"stub output\n")
            text_stream = io.StringIO()

            launcher._relay_output(source, text_stream)
            launcher._relay_output(source, None)

        self.assertEqual(text_stream.getvalue(), "stub output\n")

    def test_isolated_flag_is_recognized_only_as_first_argument(self) -> None:
        """Forward a later occurrence of the private flag to stubgen unchanged."""
        with (
            mock.patch.object(launcher.sys, "platform", "linux"),
            mock.patch.object(
                launcher.sys,
                "argv",
                ["launcher", "stubgen.py", launcher.WINDOWS_ISOLATED_FLAG],
            ),
            mock.patch.object(launcher.runpy, "run_path"),
        ):
            launcher.main()

            self.assertEqual(launcher.sys.argv, ["stubgen.py", launcher.WINDOWS_ISOLATED_FLAG])


if __name__ == "__main__":
    unittest.main()
