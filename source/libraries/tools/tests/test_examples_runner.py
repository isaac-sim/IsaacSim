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

"""Test developer environment handling in the examples runner."""

from __future__ import annotations

import ast
import hashlib
import importlib.machinery
import importlib.util
import json
import os
import shutil
import stat
import subprocess
import sys
import tempfile
import unittest
from dataclasses import replace
from pathlib import Path
from types import SimpleNamespace
from unittest import mock

EXAMPLES_RUNNER_PATH = Path(__file__).resolve().parents[3] / "examples" / "examples.py"
EXAMPLES_RUNNER_SPEC = importlib.util.spec_from_file_location("isaacsim_examples_runner", EXAMPLES_RUNNER_PATH)
if EXAMPLES_RUNNER_SPEC is None or EXAMPLES_RUNNER_SPEC.loader is None:
    raise RuntimeError(f"Cannot load examples runner from {EXAMPLES_RUNNER_PATH}")
examples_runner = importlib.util.module_from_spec(EXAMPLES_RUNNER_SPEC)
sys.modules[EXAMPLES_RUNNER_SPEC.name] = examples_runner
previous_bytecode_setting = sys.dont_write_bytecode
try:
    sys.dont_write_bytecode = True
    EXAMPLES_RUNNER_SPEC.loader.exec_module(examples_runner)
finally:
    sys.dont_write_bytecode = previous_bytecode_setting

EXAMPLES_INTEGRATION_PATH = Path(__file__).resolve().parents[2] / "testing" / "examples" / "run_examples_test.py"
examples_integration = None
if sys.version_info >= (3, 11):
    EXAMPLES_INTEGRATION_SPEC = importlib.util.spec_from_file_location(
        "isaacsim_examples_integration", EXAMPLES_INTEGRATION_PATH
    )
    if EXAMPLES_INTEGRATION_SPEC is None or EXAMPLES_INTEGRATION_SPEC.loader is None:
        raise RuntimeError(f"Cannot load examples integration runner from {EXAMPLES_INTEGRATION_PATH}")
    examples_integration = importlib.util.module_from_spec(EXAMPLES_INTEGRATION_SPEC)
    sys.modules[EXAMPLES_INTEGRATION_SPEC.name] = examples_integration
    EXAMPLES_INTEGRATION_SPEC.loader.exec_module(examples_integration)


class ExamplesCommandTests(unittest.TestCase):
    """Test machine-readable discovery and execution commands."""

    @unittest.skipUnless(
        os.name != "nt" and all(shutil.which(tool) for tool in ("cmake", "cc", "make")),
        "Requires a POSIX CMake/C toolchain",
    )
    def test_readiness_recovers_after_successful_incremental_build(self) -> None:
        """Accept no-op rebuilds while rejecting edits, failed builds, and missing outputs."""
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            source = root / "source"
            source.mkdir()
            script = source / "main.c"
            script.write_text("int main(void) { return 0; }\n", encoding="utf-8")
            readme = source / "README.md"
            readme.write_text("Documentation\n", encoding="utf-8")
            (source / "CMakeLists.txt").write_text(
                "cmake_minimum_required(VERSION 3.16)\nproject(probe C)\n"
                'set(CMAKE_RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/bin")\n'
                "add_executable(demo main.c)\n",
                encoding="utf-8",
            )
            example = SimpleNamespace(
                id="demo",
                root=source,
                requirements=SimpleNamespace(python_packages=(), cpp_packages=(), system=()),
                build=SimpleNamespace(adapter="cmake", targets=("demo",)),
                run=SimpleNamespace(adapter="executable", target="demo", path=None),
            )
            requirements = SimpleNamespace(
                cmake_minimum_version="3.16",
                cmake_policy_maximum_version="3.30",
                c_standard="11",
                cpp_standard="17",
                python_version="3.11",
            )
            build_root = root / "build"
            library_build = root / "library-build"
            library_build.mkdir()
            (library_build / "CMakeCache.txt").write_text(
                f"CMAKE_C_COMPILER:FILEPATH={shutil.which('cc')}\n"
                f"CMAKE_CXX_COMPILER:FILEPATH={shutil.which('c++')}\n",
                encoding="utf-8",
            )

            def build(sdk_digest: str = "sdk-one") -> Path:
                return examples_runner._build_example(
                    example,
                    None,
                    None,
                    requirements,
                    build_root,
                    shutil.which("cmake"),
                    "Release",
                    "Unix Makefiles",
                    None,
                    None,
                    None,
                    developer_build=SimpleNamespace(library_build_dir=library_build, artifact_digest=sdk_digest),
                )

            def check(sdk_digest: str = "sdk-one") -> list[str]:
                return examples_runner._find_runnable_examples((example,), build_root, "Release", sdk_digest=sdk_digest)

            build_dir = build()
            executable = build_dir / "bin/demo"
            self.assertEqual(check(), ["demo"])
            self.assertEqual(check("sdk-two"), [])
            build("sdk-two")
            self.assertEqual(check("sdk-two"), ["demo"])
            build()
            original_time = executable.stat().st_mtime_ns
            readme.write_text("Updated documentation\n", encoding="utf-8")
            os.utime(readme, ns=(original_time + 1_000_000, original_time + 1_000_000))
            self.assertEqual(check(), [])
            build()
            self.assertEqual(executable.stat().st_mtime_ns, original_time)
            self.assertEqual(check(), ["demo"])

            # Input contents, rather than timestamps alone, invalidate the record.
            script.write_text("int main(void) { return 1; }\n", encoding="utf-8")
            self.assertEqual(check(), [])
            build()
            self.assertEqual(check(), ["demo"])
            with mock.patch.object(
                examples_runner, "_run_checked", side_effect=[None, examples_runner.ExampleError("failed")]
            ):
                with self.assertRaisesRegex(examples_runner.ExampleError, "failed"):
                    build()
            self.assertEqual(check(), [])
            build()
            self.assertEqual(check(), ["demo"])

            # An edit during compilation must not certify the changed inputs.
            run_checked = examples_runner._run_checked

            def change_during_build(command: list[str], working_directory: Path, environment: dict[str, str]) -> None:
                run_checked(command, working_directory, environment)
                if "--build" in command:
                    readme.write_text("Changed while building\n", encoding="utf-8")

            with mock.patch.object(examples_runner, "_run_checked", side_effect=change_during_build):
                build()
            self.assertEqual(check(), [])
            build()
            self.assertEqual(check(), ["demo"])
            executable.unlink()
            self.assertEqual(check(), [])

    def test_readiness_reuses_existing_outputs_and_rejects_stale_or_missing_builds(self) -> None:
        """Discover disk builds without an in-memory record or running build tools."""
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            source = root / "source"
            source.mkdir()
            script = source / "main.py"
            script.write_text("print('hello')", encoding="utf-8")
            example = SimpleNamespace(
                id="demo",
                requirements=SimpleNamespace(cpp_packages=()),
                root=source,
                build=SimpleNamespace(adapter="cmake", targets=("demo",)),
                run=SimpleNamespace(adapter="executable", target="demo", path=None),
            )
            build_root = root / "build"
            build_dir = build_root / "demo"
            (build_dir / "bin").mkdir(parents=True)
            cache = build_dir / "CMakeCache.txt"
            cache.write_text(
                f"CMAKE_BUILD_TYPE:STRING=Release\nCMAKE_HOME_DIRECTORY:INTERNAL={source}\n",
                encoding="utf-8",
            )
            executable = build_dir / "bin" / ("demo.exe" if os.name == "nt" else "demo")
            with (
                mock.patch.object(examples_runner, "_require_python_dependencies"),
                mock.patch.object(
                    examples_runner, "_run_checked", side_effect=AssertionError("Readiness must not build")
                ),
            ):
                check = lambda config="Release": examples_runner._find_runnable_examples((example,), build_root, config)
                self.assertEqual(check(), [])
                executable.touch()
                os.utime(script, ns=(1, 1))
                self.assertEqual(check(), ["demo"])
                self.assertEqual(
                    examples_runner._find_runnable_examples(
                        (example,),
                        build_root,
                        "Release",
                        sdk_digest="sdk",
                        sdk_mtime_ns=executable.stat().st_mtime_ns + 1,
                    ),
                    [],
                )
                self.assertEqual(check("Debug"), [])
                newer = executable.stat().st_mtime_ns + 1_000_000
                os.utime(script, ns=(newer, newer))
                self.assertEqual(check(), [])
                os.utime(script, ns=(1, 1))
                executable.unlink()
                self.assertEqual(check(), [])
                example.run.adapter = "python"
                example.run.path = script
                self.assertEqual(check(), [])
                (build_dir / "python").mkdir()
                (build_dir / "python" / f"demo.incompatible{'.pyd' if os.name == 'nt' else '.so'}").touch()
                self.assertEqual(check(), [])
                (build_dir / "python" / f"demo{importlib.machinery.EXTENSION_SUFFIXES[0]}").touch()
                self.assertEqual(check(), ["demo"])
                example.build.targets = ("demo", "support")
                self.assertEqual(check(), [])
                (build_dir / "python" / f"support{importlib.machinery.EXTENSION_SUFFIXES[0]}").touch()
                self.assertEqual(check(), ["demo"])
                example.build.adapter = "none"
                self.assertEqual(check(), ["demo"])
                script.unlink()
                self.assertEqual(check(), [])
            with mock.patch.object(
                examples_runner, "_require_python_dependencies", side_effect=examples_runner.ExampleError("Missing")
            ):
                self.assertEqual(examples_runner._find_runnable_examples((example,), build_root, "Release"), [])

    def test_readiness_requires_existing_cpp_dependencies(self) -> None:
        """Require a matching on-disk C++ environment before offering an otherwise runnable example."""
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            source = root / "source"
            source.mkdir()
            script = source / "main.cpp"
            script.write_text("int main() { return 0; }", encoding="utf-8")
            os.utime(script, ns=(1, 1))
            packages = (examples_runner._CppPackageRequirement("fmt", "11.1.4"),)
            example = SimpleNamespace(
                id="demo",
                root=source,
                requirements=SimpleNamespace(python_packages=(), cpp_packages=packages),
                build=SimpleNamespace(adapter="cmake", targets=("demo",)),
                run=SimpleNamespace(adapter="executable", path=None, target="demo", arguments=()),
            )
            build_root = root / "build"
            build_dir = build_root / "demo"
            (build_dir / "bin").mkdir(parents=True)
            (build_dir / "bin" / ("demo.exe" if os.name == "nt" else "demo")).touch()
            (build_dir / "CMakeCache.txt").write_text(
                f"CMAKE_BUILD_TYPE:STRING=Release\nCMAKE_HOME_DIRECTORY:INTERNAL={source}\n", encoding="utf-8"
            )
            workspace = build_root / examples_runner.CPP_DEPENDENCY_DIRECTORY
            with mock.patch.object(examples_runner, "_find_pixi", return_value=Path(sys.executable)):
                self.assertEqual(examples_runner._find_runnable_examples((example,), build_root, "Release"), [])
                (workspace / ".pixi/envs/default").mkdir(parents=True)
                (workspace / "pixi.toml").write_text(
                    examples_runner._cpp_dependency_manifest(packages), encoding="utf-8"
                )
                (workspace / "pixi.lock").touch()
                (workspace / examples_runner.DEPENDENCY_STATE_FILE).write_text(
                    json.dumps(examples_runner._cpp_dependency_state(packages)), encoding="utf-8"
                )
                self.assertEqual(examples_runner._find_runnable_examples((example,), build_root, "Release"), ["demo"])
                example.requirements.cpp_packages = (examples_runner._CppPackageRequirement("fmt", "12.0.0"),)
                self.assertEqual(examples_runner._find_runnable_examples((example,), build_root, "Release"), [])

    def test_run_no_build_skips_compilation_and_preserves_arguments(self) -> None:
        """Keep ordinary Run behavior while allowing frontends to reuse prior outputs."""
        example = SimpleNamespace(id="demo", requirements=SimpleNamespace(cpp_packages=()))
        for no_build in (False, True):
            with self.subTest(no_build=no_build):
                arguments = ["examples.py", "--installed", "run"]
                arguments.extend(["--no-build"] if no_build else [])
                arguments.extend(["demo", "--", "--name", "two words"])
                with (
                    mock.patch.object(sys, "argv", arguments),
                    mock.patch.object(examples_runner, "_load_examples", return_value=(example,)),
                    mock.patch.object(examples_runner, "_select_run_example", return_value=example),
                    mock.patch.object(examples_runner, "_load_system_requirements"),
                    mock.patch.object(examples_runner, "_validate_python_version"),
                    mock.patch.object(examples_runner, "_collect_python_packages", return_value=()),
                    mock.patch.object(examples_runner, "_require_python_dependencies", return_value=None),
                    mock.patch.object(examples_runner, "_build_example") as build,
                    mock.patch.object(examples_runner, "_find_runnable_examples", return_value=["demo"]) as find,
                    mock.patch.object(examples_runner, "_run_example", return_value=0) as run,
                ):
                    self.assertEqual(examples_runner._main(), 0)
                    self.assertEqual(build.call_count, 0 if no_build else 1)
                    self.assertEqual(find.call_count, 1 if no_build else 0)
                    self.assertEqual(run.call_args.args[2], ("--name", "two words"))

    def test_run_no_build_rejects_stale_selected_example(self) -> None:
        """Recheck the selected artifact in the execution process before launching it."""
        example = SimpleNamespace(id="demo", requirements=SimpleNamespace(cpp_packages=()))
        with (
            mock.patch.object(sys, "argv", ["examples.py", "--installed", "run", "--no-build", "demo"]),
            mock.patch.object(examples_runner, "_load_examples", return_value=(example,)),
            mock.patch.object(examples_runner, "_select_run_example", return_value=example),
            mock.patch.object(examples_runner, "_load_system_requirements"),
            mock.patch.object(examples_runner, "_validate_python_version"),
            mock.patch.object(examples_runner, "_collect_python_packages", return_value=()),
            mock.patch.object(examples_runner, "_require_python_dependencies", return_value=None),
            mock.patch.object(examples_runner, "_find_runnable_examples", return_value=[]) as find,
            mock.patch.object(examples_runner, "_run_example") as run,
            mock.patch("builtins.print"),
        ):
            self.assertEqual(examples_runner._main(), 1)
        find.assert_called_once()
        self.assertEqual(find.call_args.kwargs["selected_examples"], (example,))
        run.assert_not_called()

    def test_structured_listing_needs_no_built_library_catalog(self) -> None:
        """Expose validated local examples and ordered series without SDK output."""
        with mock.patch.object(
            examples_runner, "_load_library_catalog", side_effect=AssertionError("SDK catalog read")
        ):
            listing = examples_runner._serialize_listing(EXAMPLES_RUNNER_PATH.parent)
        self.assertEqual(listing["schema_version"], 1)
        example_ids = {item["id"] for item in listing["examples"]}
        self.assertIn("hello_world.python", example_ids)
        self.assertTrue(listing["series"])
        for series in listing["series"]:
            self.assertTrue(all(step["example"] in example_ids for step in series["steps"]))

    def test_check_validates_sdk_once_in_each_environment(self) -> None:
        """Defer source hashing until the selected interpreter and retain shared locks."""
        repository_root = EXAMPLES_RUNNER_PATH.parents[2]
        library_build_dir = repository_root / "_cmake_build" / "isaacsim-libraries-release"
        build = SimpleNamespace(
            library_build_dir=library_build_dir,
            python=Path(sys.executable),
            artifact_digest="sdk-digest",
            artifact_mtime_ns=1,
        )
        for active in (False, True):
            with (
                self.subTest(active=active),
                mock.patch.object(sys, "argv", ["examples.py", "check"]),
                mock.patch.dict(
                    os.environ,
                    {
                        examples_runner.DEVELOPER_ENVIRONMENT_VARIABLE: (
                            str(library_build_dir.resolve()) if active else ""
                        )
                    },
                ),
                mock.patch.object(examples_runner, "_developer_environment_lock") as sdk_lock,
                mock.patch.object(examples_runner, "_python_dependency_lock") as python_lock,
                mock.patch.object(examples_runner, "_cpp_dependency_lock") as cpp_lock,
                mock.patch.object(examples_runner, "_read_developer_build", return_value=build) as read,
                mock.patch.object(examples_runner, "_create_developer_environment", return_value={}),
                mock.patch.object(examples_runner, "_restart_in_environment", return_value=0) as restart,
                mock.patch.object(examples_runner, "_load_collection", return_value=((), ())),
                mock.patch.object(examples_runner, "_collect_python_packages", return_value=("package",)),
                mock.patch.object(examples_runner, "_collect_cpp_packages", return_value=("package",)),
                mock.patch.object(examples_runner, "_find_runnable_examples", return_value=["demo"]) as find,
                mock.patch("builtins.print") as output,
            ):
                self.assertEqual(examples_runner._main(), 0)
                read.assert_called_once_with(repository_root, "Release", validate_sources=active)
                sdk_lock.assert_called_once_with(library_build_dir, exclusive=False)
                if active:
                    restart.assert_not_called()
                    self.assertEqual(json.loads(output.call_args.args[0])["runnable_examples"], ["demo"])
                    for lock in (python_lock, cpp_lock):
                        lock.assert_called_once_with(find.call_args.args[1], exclusive=False)
                        lock.return_value.__enter__.assert_called_once()
                        lock.return_value.__exit__.assert_called_once()
                else:
                    restart.assert_called_once_with(build.python, {})
                    find.assert_not_called()
                    output.assert_not_called()


class ExamplesListFormattingTests(unittest.TestCase):
    """Test user-facing example list formatting."""

    def test_format_examples_table_aligns_ids_and_titles(self) -> None:
        """Align table columns to the longest values."""
        examples = (
            SimpleNamespace(id="short", title="A longer title"),
            SimpleNamespace(id="example.with_a_long_id", title="Title"),
        )

        table = examples_runner._format_examples_table(examples)

        self.assertEqual(
            table,
            "\n".join(
                (
                    "ID                     | TITLE",
                    "-----------------------+---------------",
                    "short                  | A longer title",
                    "example.with_a_long_id | Title",
                )
            ),
        )


@unittest.skipIf(examples_integration is None, "Examples integration requires Python 3.11 or newer")
class ExamplesBuildMetricsTests(unittest.TestCase):
    """Test example build storage measurements."""

    @unittest.skipIf(os.name == "nt", "Symlink identity test requires Unix filesystem semantics")
    def test_directory_storage_counts_unique_regular_files(self) -> None:
        """Exclude symlinks and count hard-linked content once."""
        with tempfile.TemporaryDirectory() as temporary_directory:
            root = Path(temporary_directory)
            original = root / "original.bin"
            original.write_bytes(b"x" * 100)
            os.link(original, root / "hard-link.bin")
            (root / "symbolic-link.bin").symlink_to(original)

            metrics = examples_integration._directory_storage(root)

        self.assertEqual(metrics["logical_size_bytes"], 100)
        self.assertGreaterEqual(metrics["disk_size_bytes"], 100)
        self.assertEqual(metrics["file_count"], 1)


class PythonExampleInterruptHandlingTests(unittest.TestCase):
    """Require published Python examples to exit cleanly when interrupted."""

    def test_interactive_runner_maps_keyboard_interrupt_to_exit_130(self) -> None:
        """Wait for an interrupted child to finish cleaning up without a traceback."""
        example = SimpleNamespace(
            id="demo",
            root=Path("/example"),
            run=SimpleNamespace(arguments=()),
            requirements=SimpleNamespace(cpp_packages=()),
        )
        process = mock.Mock()
        process.wait.side_effect = [KeyboardInterrupt, 0]
        with (
            mock.patch.object(examples_runner, "_create_run_command", return_value=["python", "main.py"]),
            mock.patch.object(examples_runner, "_create_runtime_environment", return_value={}),
            mock.patch.object(examples_runner.subprocess, "Popen", return_value=process),
            mock.patch("builtins.print"),
        ):
            result = examples_runner._run_example(example, Path("/build"), ())

        self.assertEqual(result, 130)
        self.assertEqual(
            process.wait.call_args_list,
            [mock.call(), mock.call(timeout=examples_runner.PROCESS_TERMINATION_TIMEOUT_SECONDS)],
        )
        process.terminate.assert_not_called()

    def test_interactive_runner_terminates_child_that_does_not_handle_interrupt(self) -> None:
        """Do not orphan a child when only the runner receives the interrupt."""
        example = SimpleNamespace(
            id="demo",
            root=Path("/example"),
            run=SimpleNamespace(arguments=()),
            requirements=SimpleNamespace(cpp_packages=()),
        )
        process = mock.Mock()
        process.wait.side_effect = [
            KeyboardInterrupt,
            examples_runner.subprocess.TimeoutExpired(
                ["python", "main.py"], examples_runner.PROCESS_TERMINATION_TIMEOUT_SECONDS
            ),
            0,
        ]
        with (
            mock.patch.object(examples_runner, "_create_run_command", return_value=["python", "main.py"]),
            mock.patch.object(examples_runner, "_create_runtime_environment", return_value={}),
            mock.patch.object(examples_runner.subprocess, "Popen", return_value=process),
            mock.patch("builtins.print"),
        ):
            result = examples_runner._run_example(example, Path("/build"), ())

        self.assertEqual(result, 130)
        process.terminate.assert_called_once_with()
        process.kill.assert_not_called()

    def test_python_entry_points_swallow_keyboard_interrupt(self) -> None:
        """Keep `KeyboardInterrupt` tracebacks out of directly executed examples."""
        examples = examples_runner._load_examples(EXAMPLES_RUNNER_PATH.parent)
        for example in examples:
            if example.run.adapter != "python":
                continue
            with self.subTest(example=example.id):
                tree = ast.parse(example.run.path.read_text(encoding="utf-8"), filename=str(example.run.path))
                main_guards = [
                    node
                    for node in tree.body
                    if isinstance(node, ast.If)
                    and isinstance(node.test, ast.Compare)
                    and isinstance(node.test.left, ast.Name)
                    and node.test.left.id == "__name__"
                    and len(node.test.ops) == 1
                    and isinstance(node.test.ops[0], ast.Eq)
                    and len(node.test.comparators) == 1
                    and isinstance(node.test.comparators[0], ast.Constant)
                    and node.test.comparators[0].value == "__main__"
                ]
                self.assertEqual(len(main_guards), 1)
                handlers = [
                    handler
                    for statement in main_guards[0].body
                    if isinstance(statement, ast.Try)
                    for handler in statement.handlers
                    if isinstance(handler.type, ast.Name) and handler.type.id == "KeyboardInterrupt"
                ]
                self.assertEqual(len(handlers), 1)
                self.assertEqual(len(handlers[0].body), 1)
                handler = handlers[0].body[0]
                if example.id.startswith("benchmarks."):
                    self.assertEqual(ast.unparse(handler), "raise SystemExit(130)")
                else:
                    self.assertIsInstance(handler, ast.Pass)


class RuntimeEnvironmentTests(unittest.TestCase):
    """Test environment construction for each example child process."""

    def test_runtime_environment_adds_windows_ovstage_search_paths(self) -> None:
        """Prepend every installed prefix's OVStage runtime directories."""
        with tempfile.TemporaryDirectory() as temporary_directory:
            root = Path(temporary_directory)
            prefix = root / "sdk"
            python_directory = root / "python"
            inherited = root / "inherited"
            plugins = prefix / "bin" / "plugins"
            prefix_paths = (
                prefix / "bin",
                prefix / "plugins",
                plugins,
                plugins / "omni.client.lib",
                plugins / "omni.usd_resolver",
            )
            for directory in (*prefix_paths, python_directory):
                directory.mkdir(parents=True, exist_ok=True)
            example = SimpleNamespace(
                run=SimpleNamespace(adapter="executable"),
                build=SimpleNamespace(adapter="none"),
            )
            environment = {"CMAKE_PREFIX_PATH": str(prefix), "PATH": str(inherited)}

            with (
                mock.patch.dict(examples_runner.os.environ, environment, clear=True),
                mock.patch.object(examples_runner.os, "name", "nt"),
                mock.patch.object(examples_runner, "Path", type(root)),
                mock.patch.object(examples_runner.sys, "executable", str(python_directory / "python.exe")),
            ):
                actual = examples_runner._create_runtime_environment(example, root / "build")

            self.assertEqual(
                actual["PATH"].split(os.pathsep),
                [*map(str, prefix_paths), str(python_directory), str(inherited)],
            )

    @unittest.skipIf(os.name == "nt", "Windows resolves native dependencies through PATH.")
    def test_python_native_libraries_precede_cmake_prefixes(self) -> None:
        """Preserve the native dependency family selected by the Python interpreter."""
        with tempfile.TemporaryDirectory() as temporary_directory:
            root = Path(temporary_directory)
            python = root / "python-environment" / "bin" / "python"
            prefix_a = root / "prefix-a"
            prefix_b = root / "prefix-b"
            inherited = root / "inherited"
            for directory in (python.parent.parent / "lib", prefix_a / "lib", prefix_b / "lib"):
                directory.mkdir(parents=True)
            environment = {
                "CMAKE_PREFIX_PATH": os.pathsep.join(map(str, (prefix_a, prefix_b))),
                "LD_LIBRARY_PATH": str(inherited),
            }
            example = SimpleNamespace(
                run=SimpleNamespace(adapter="python"),
                build=SimpleNamespace(adapter="none"),
            )

            with (
                mock.patch.dict(examples_runner.os.environ, environment, clear=True),
                mock.patch.object(examples_runner.sys, "executable", str(python)),
            ):
                actual = examples_runner._create_runtime_environment(example, root / "build")

            self.assertEqual(
                actual["LD_LIBRARY_PATH"].split(os.pathsep),
                [str(python.parent.parent / "lib"), str(prefix_a / "lib"), str(prefix_b / "lib"), str(inherited)],
            )


class NamedRunCommandTests(unittest.TestCase):
    """Test compatible default runs and foreground named commands."""

    def setUp(self) -> None:
        """Load the gRPC example with a default client and named server."""
        self.example = examples_runner._load_example(
            EXAMPLES_RUNNER_PATH.parent / "series" / "falling_cube_ovsim" / "simulate_cube_grpc"
        )

    def test_bare_run_and_named_server_select_independently(self) -> None:
        """Keep the client as the default without mutating its loaded configuration."""
        default = examples_runner._select_run_example((self.example,), self.example.id, EXAMPLES_RUNNER_PATH.parent)
        server = examples_runner._select_run_example(
            (self.example,), self.example.id + ":server", EXAMPLES_RUNNER_PATH.parent
        )
        self.assertIs(default, self.example)
        self.assertEqual(default.run.adapter, "python")
        self.assertEqual(server.run.adapter, "command")
        self.assertEqual(server.run.command, "isaacsim-ovsim-grpc-server")
        self.assertEqual(server.run.arguments, ("--listen-address", "127.0.0.1:50051"))
        self.assertEqual(server.id, default.id)
        self.assertEqual(server.tests, default.tests)
        for name in ("", "missing", "server:other"):
            with self.subTest(name=name), self.assertRaisesRegex(examples_runner.ExampleError, "Unknown run command"):
                examples_runner._select_run_example(
                    (self.example,), self.example.id + ":" + name, EXAMPLES_RUNNER_PATH.parent
                )

    def test_legacy_relative_run_selectors_still_work(self) -> None:
        """Preserve root and script path selection alongside named commands."""
        for path in (self.example.root, self.example.run.path):
            relative = path.relative_to(EXAMPLES_RUNNER_PATH.parent).as_posix()
            self.assertIs(
                examples_runner._select_run_example((self.example,), relative, EXAMPLES_RUNNER_PATH.parent),
                self.example,
            )
            selected = examples_runner._select_run_example(
                (self.example,), relative + ":server", EXAMPLES_RUNNER_PATH.parent
            )
            self.assertEqual(selected.run.command, "isaacsim-ovsim-grpc-server")

    def test_named_python_run_has_its_own_arguments(self) -> None:
        """Named commands use independent configurations and validated local scripts."""
        run, named = examples_runner._load_run_configurations(
            {
                "adapter": "python",
                "path": "main.py",
                "arguments": ["default"],
                "alternate": {"adapter": "python", "path": "test.py", "arguments": ["alternate"]},
            },
            self.example.root,
            "run",
        )
        self.assertEqual(run.arguments, ("default",))
        self.assertEqual(named[0][1].arguments, ("alternate",))
        self.assertEqual(named[0][1].path, self.example.root / "test.py")
        selected = replace(self.example, run=named[0][1])
        self.assertEqual(
            examples_runner._create_run_command(selected, Path("build"), ("one",)),
            [sys.executable, str(self.example.root / "test.py"), "one"],
        )

    def test_rejects_malformed_runs_and_command_names(self) -> None:
        """Reject unknown fields, shell fragments, paths, and named-only manifests."""
        for command in ("../server", "/bin/server", "server --flag", "a;b", "$(server)", "-server", "C:\\server.exe"):
            with self.subTest(command=command), self.assertRaisesRegex(examples_runner.ExampleError, "executable name"):
                examples_runner._load_run_configuration(
                    {"adapter": "command", "command": command}, self.example.root, "run"
                )
        default = {"adapter": "python", "path": "main.py"}
        bad_tables = (
            {**default, "typo": "ignored"},
            {**default, "bad-name": {"adapter": "command", "command": "server"}},
            {**default, "server": {"adapter": "command", "command": "server", "background": True}},
            {"server": {"adapter": "command", "command": "server"}},
            {**default, "server": {"adapter": "python", "path": "../main.py"}},
            {**default, "server": {"adapter": "python", "path": "missing.py"}},
        )
        for table in bad_tables:
            with self.subTest(table=table), self.assertRaises(examples_runner.ExampleError):
                examples_runner._load_run_configurations(table, self.example.root, "run")

    def test_command_lookup_uses_prepared_environment(self) -> None:
        """Resolve an installed executable even when it is absent from the parent PATH."""
        selected = replace(
            self.example, run=examples_runner._RunConfiguration("command", command=Path(sys.executable).name)
        )
        with mock.patch.dict(os.environ, {"PATH": ""}):
            command = examples_runner._create_run_command(
                selected, Path("build"), ("--version",), environment={"PATH": str(Path(sys.executable).parent)}
            )
        self.assertEqual(Path(command[0]), Path(sys.executable).absolute())
        self.assertEqual(command[1:], ["--version"])
        with (
            tempfile.TemporaryDirectory() as empty_path,
            self.assertRaisesRegex(examples_runner.ExampleError, "unavailable in the prepared environment"),
        ):
            examples_runner._create_run_command(selected, Path("build"), (), environment={"PATH": empty_path})

    @unittest.skipIf(os.name == "nt", "Creating executable symlinks requires Windows privileges")
    def test_command_lookup_preserves_environment_symlinks(self) -> None:
        """Keep the selected executable path so virtual environments remain active."""
        with tempfile.TemporaryDirectory() as temporary_directory:
            executable = Path(temporary_directory) / "environment-python"
            executable.symlink_to(sys.executable)
            selected = replace(self.example, run=examples_runner._RunConfiguration("command", command=executable.name))
            actual = examples_runner._create_run_command(
                selected, Path("build"), (), environment={"PATH": temporary_directory}
            )
            self.assertEqual(actual, [str(executable)])

    def test_named_command_runs_in_foreground_and_preserves_arguments(self) -> None:
        """Wait for a real child, preserve literal arguments, and return its exit status."""
        with tempfile.TemporaryDirectory() as temporary_directory:
            root = Path(temporary_directory)
            script = root / "record.py"
            script.write_text(
                'import json, pathlib, sys\npathlib.Path("result.json").write_text(json.dumps(sys.argv[1:]))\nsys.exit(7)\n',
                encoding="utf-8",
            )
            selected = replace(
                self.example,
                root=root,
                run=examples_runner._RunConfiguration(
                    "command", command=Path(sys.executable).name, arguments=(str(script), "default argument")
                ),
            )
            environment = dict(os.environ, PATH=str(Path(sys.executable).parent))
            with mock.patch.object(examples_runner, "_create_runtime_environment", return_value=environment):
                result = examples_runner._run_example(selected, root / "build", ("two words", "$(not-a-shell)", "--"))
            self.assertEqual(result, 7)
            self.assertEqual(
                json.loads((root / "result.json").read_text()),
                ["default argument", "two words", "$(not-a-shell)", "--"],
            )

    def test_default_command_adapter_and_named_python_share_requirements(self) -> None:
        """Allow shared Python dependencies when a named Python run consumes them."""
        command = examples_runner._RunConfiguration("command", command="server")
        requirements = replace(
            self.example.requirements, python_packages=(examples_runner._PythonPackageRequirement("demo", "1.0"),)
        )
        examples_runner._validate_requirement_coherence(
            self.example.build, command, requirements, "example", (self.example.run,)
        )
        with self.assertRaisesRegex(examples_runner.ExampleError, "without a Python run adapter"):
            examples_runner._validate_requirement_coherence(self.example.build, command, requirements, "example")

    def test_named_executable_must_match_build_targets(self) -> None:
        """Reject invalid native targets before invoking CMake or launching a process."""
        with tempfile.TemporaryDirectory() as temporary_directory:
            root = Path(temporary_directory)
            (root / "main.py").touch()
            (root / "README.md").write_text("# Fixture\n")
            manifest = (
                (self.example.root / "example.toml")
                .read_text()
                .replace(
                    'adapter = "command"\ncommand = "isaacsim-ovsim-grpc-server"',
                    'adapter = "executable"\ntarget = "server"',
                )
            )
            # This fixture has no tests; only native command validation should fail.
            (root / "example.toml").write_text(manifest.split("[[test]]")[0], encoding="utf-8")
            with self.assertRaisesRegex(
                examples_runner.ExampleError, "run.server.target must name a declared CMake build target"
            ):
                examples_runner._load_example(root)

    def test_cli_selects_named_command_without_changing_default(self) -> None:
        """Exercise selection through the real CLI flow in installed mode."""
        for suffix, adapter in (("", "python"), (":server", "command")):
            with (
                mock.patch.object(
                    sys,
                    "argv",
                    ["examples.py", "--installed", "run", self.example.id + suffix, "--", "literal argument"],
                ),
                mock.patch.object(examples_runner, "_load_system_requirements"),
                mock.patch.object(examples_runner, "_validate_python_version"),
                mock.patch.object(examples_runner, "_load_examples", return_value=(self.example,)),
                mock.patch.object(examples_runner, "_build_example"),
                mock.patch.object(examples_runner, "_run_example", return_value=9) as run,
            ):
                self.assertEqual(examples_runner._main(), 9)
            self.assertEqual(run.call_args.args[0].run.adapter, adapter)
            self.assertEqual(run.call_args.args[2], ("literal argument",))


class PythonTestEntryPointTests(unittest.TestCase):
    """Test Python-specific entry points in example test configurations."""

    def test_loads_test_path_relative_to_example(self) -> None:
        """Resolve an explicit test script without changing the normal run path."""
        with tempfile.TemporaryDirectory() as temporary_directory:
            root = Path(temporary_directory)
            main_path = root / "main.py"
            test_path = root / "test.py"
            main_path.touch()
            test_path.touch()
            run = examples_runner._RunConfiguration(adapter="python", path=main_path)

            tests = examples_runner._load_test_configurations(
                [{"name": "headless", "path": "test.py"}], root, run, "example.toml.test"
            )

            self.assertEqual(tests[0].path, test_path)
            example = SimpleNamespace(id="demo", run=run)
            self.assertEqual(
                examples_runner._create_run_command(example, root / "build", (), python_path=tests[0].path),
                [sys.executable, str(test_path)],
            )

    def test_rejects_python_test_path_for_executable_example(self) -> None:
        """Keep executable tests on their declared built target."""
        with tempfile.TemporaryDirectory() as temporary_directory:
            root = Path(temporary_directory)
            (root / "test.py").touch()
            run = examples_runner._RunConfiguration(adapter="executable", target="demo")

            with self.assertRaisesRegex(examples_runner.ExampleError, "Python run adapter"):
                examples_runner._load_test_configurations(
                    [{"name": "headless", "path": "test.py"}], root, run, "example.toml.test"
                )


@unittest.skipIf(examples_integration is None, "Examples integration tests require Python 3.11 or newer")
class ExamplesIntegrationPackageRequirementTests(unittest.TestCase):
    """Test installed package dependency resolution for examples."""

    def _write_example(
        self,
        examples_dir: Path,
        name: str,
        requirements: tuple[tuple[str, tuple[str, ...]], ...],
    ) -> None:
        """Write one minimal example with direct package requirements.

        Args:
            examples_dir: Root of the examples collection.
            name: Example directory name.
            requirements: Direct package names and surfaces.
        """
        example_root = examples_dir / name
        example_root.mkdir(parents=True)
        lines = ["published = true", "", "[requirements]"]
        for package_name, surfaces in requirements:
            lines.extend(
                (
                    "",
                    "[[requirements.modules]]",
                    f'name = "{package_name}"',
                    f"surfaces = {json.dumps(surfaces)}",
                )
            )
        (example_root / "example.toml").write_text("\n".join(lines) + "\n", encoding="utf-8")

    def _write_package(self, build_dir: Path, name: str, dependencies: tuple[str, ...] = ()) -> None:
        """Write one canonical generated package manifest.

        Args:
            build_dir: Library build directory.
            name: Package group name.
            dependencies: Direct internal package dependencies.
        """
        manifest_path = build_dir / "packages" / name / "package.json"
        manifest_path.parent.mkdir(parents=True)
        manifest_path.write_text(
            json.dumps(
                {
                    "name": name,
                    "version": "1.0.0",
                    "version_scheme": "pep440",
                    "complete": True,
                    "dependencies": dict.fromkeys(dependencies, "==1.0.0"),
                    "modules": [f"isaacsim.{name}"],
                    "python_imports": [],
                    "components": {
                        "runtime": f"{name}-runtime",
                        "development": f"{name}-development",
                        "python": f"{name}-python",
                        "documentation": f"{name}-documentation",
                    },
                }
            ),
            encoding="utf-8",
        )

    def _arguments(self, build_dir: Path, package_groups: tuple[str, ...]) -> SimpleNamespace:
        """Create package installation arguments.

        Args:
            build_dir: Library build directory.
            package_groups: Available internal package groups.

        Returns:
            Minimal arguments consumed by package resolution.
        """
        return SimpleNamespace(
            cmake=Path("/cmake"),
            library_build_dir=build_dir,
            package_group=list(package_groups),
            config="Release",
        )

    def test_install_declared_requirements_expands_recursive_dependency_closure(self) -> None:
        """Install transitive internal dependencies from generated manifests."""
        with tempfile.TemporaryDirectory() as temporary_directory:
            root = Path(temporary_directory)
            examples_dir = root / "examples"
            build_dir = root / "build"
            sdk_prefix = root / "sdk"
            python_prefix = root / "python"
            self._write_example(examples_dir, "demo", (("root_package", ("native_sdk",)),))
            self._write_package(build_dir, "root_package", ("middle_package",))
            self._write_package(build_dir, "middle_package", ("leaf_package",))
            self._write_package(build_dir, "leaf_package")
            arguments = self._arguments(build_dir, ("root_package", "middle_package", "leaf_package"))

            with mock.patch.object(examples_integration, "_install_component") as install_component:
                examples_integration._install_declared_requirements(
                    arguments,
                    examples_dir,
                    sdk_prefix,
                    python_prefix,
                )

            self.assertEqual(
                install_component.call_args_list,
                [
                    mock.call(arguments, sdk_prefix, "leaf_package-runtime"),
                    mock.call(arguments, sdk_prefix, "leaf_package-development"),
                    mock.call(arguments, sdk_prefix, "middle_package-runtime"),
                    mock.call(arguments, sdk_prefix, "middle_package-development"),
                    mock.call(arguments, sdk_prefix, "root_package-runtime"),
                    mock.call(arguments, sdk_prefix, "root_package-development"),
                ],
            )

    def test_install_declared_requirements_unions_surfaces_and_installs_each_component_once(self) -> None:
        """Union surfaces propagated to a shared dependency without duplicate installs."""
        with tempfile.TemporaryDirectory() as temporary_directory:
            root = Path(temporary_directory)
            examples_dir = root / "examples"
            build_dir = root / "build"
            sdk_prefix = root / "sdk"
            python_prefix = root / "python"
            self._write_example(examples_dir, "native", (("native_root", ("native_sdk",)),))
            self._write_example(examples_dir, "python", (("python_root", ("python",)),))
            self._write_package(build_dir, "native_root", ("shared_dependency",))
            self._write_package(build_dir, "python_root", ("shared_dependency",))
            self._write_package(build_dir, "shared_dependency")
            arguments = self._arguments(build_dir, ("native_root", "python_root", "shared_dependency"))

            with mock.patch.object(examples_integration, "_install_component") as install_component:
                examples_integration._install_declared_requirements(
                    arguments,
                    examples_dir,
                    sdk_prefix,
                    python_prefix,
                )

            self.assertEqual(
                install_component.call_args_list,
                [
                    mock.call(arguments, sdk_prefix, "shared_dependency-runtime"),
                    mock.call(arguments, sdk_prefix, "shared_dependency-development"),
                    mock.call(arguments, python_prefix, "shared_dependency-python"),
                    mock.call(arguments, sdk_prefix, "native_root-runtime"),
                    mock.call(arguments, sdk_prefix, "native_root-development"),
                    mock.call(arguments, sdk_prefix, "python_root-runtime"),
                    mock.call(arguments, python_prefix, "python_root-python"),
                ],
            )

    def test_install_declared_requirements_rejects_missing_dependency_before_install(self) -> None:
        """Reject a dependency absent from the available package groups before installing."""
        with tempfile.TemporaryDirectory() as temporary_directory:
            root = Path(temporary_directory)
            examples_dir = root / "examples"
            build_dir = root / "build"
            self._write_example(examples_dir, "demo", (("root_package", ("python",)),))
            self._write_package(build_dir, "root_package", ("missing_package",))
            arguments = self._arguments(build_dir, ("root_package",))

            with (
                mock.patch.object(examples_integration, "_install_component") as install_component,
                self.assertRaisesRegex(ValueError, "root_package -> missing_package"),
            ):
                examples_integration._install_declared_requirements(
                    arguments,
                    examples_dir,
                    root / "sdk",
                    root / "python",
                )

            install_component.assert_not_called()

    def test_install_declared_requirements_rejects_cycle_before_install(self) -> None:
        """Reject an internal package dependency cycle before installing."""
        with tempfile.TemporaryDirectory() as temporary_directory:
            root = Path(temporary_directory)
            examples_dir = root / "examples"
            build_dir = root / "build"
            self._write_example(examples_dir, "demo", (("package_a", ("native_sdk",)),))
            self._write_package(build_dir, "package_a", ("package_b",))
            self._write_package(build_dir, "package_b", ("package_a",))
            arguments = self._arguments(build_dir, ("package_a", "package_b"))

            with (
                mock.patch.object(examples_integration, "_install_component") as install_component,
                self.assertRaisesRegex(RuntimeError, "package_a -> package_b -> package_a"),
            ):
                examples_integration._install_declared_requirements(
                    arguments,
                    examples_dir,
                    root / "sdk",
                    root / "python",
                )

            install_component.assert_not_called()


@unittest.skipIf(examples_integration is None, "Examples integration tests require Python 3.11 or newer")
class ExamplesIntegrationCleanupTests(unittest.TestCase):
    """Test retention of examples integration diagnostics."""

    def test_remove_test_tree_handles_legacy_read_only_dependencies(self) -> None:
        """Remove dependency directories hardened by the previous layout."""
        with tempfile.TemporaryDirectory() as temporary_directory:
            test_root = Path(temporary_directory) / "examples-test"
            package_dir = test_root / "python-dependencies" / "key" / "package"
            package_dir.mkdir(parents=True)
            (package_dir / "dependency.py").touch()
            package_dir.chmod(stat.S_IMODE(package_dir.stat().st_mode) & ~0o222)

            examples_integration._remove_test_tree(test_root)

            self.assertFalse(test_root.exists())

    @unittest.skipIf(os.name == "nt", "Symbolic-link creation requires additional privileges on Windows.")
    def test_remove_test_tree_does_not_chmod_symlink_target(self) -> None:
        """Remove a nested symlink without changing its external target."""
        with tempfile.TemporaryDirectory() as temporary_directory:
            root = Path(temporary_directory)
            target = root / "keep.py"
            target.touch(mode=0o400)
            test_root = root / "examples-test"
            test_root.mkdir()
            (test_root / "redirect.py").symlink_to(target)
            test_root.chmod(0o500)

            examples_integration._remove_test_tree(test_root)

            self.assertEqual(stat.S_IMODE(target.stat().st_mode), 0o400)

    def _run_integration(self, *, error: Exception | None = None) -> bool:
        """Run a stubbed integration and report whether its test root remains.

        Args:
            error: Exception raised by the example.

        Returns:
            The resulting value.
        """
        with tempfile.TemporaryDirectory() as temporary_directory:
            test_root = Path(temporary_directory) / "examples-test"
            arguments = SimpleNamespace(test_root=test_root)

            def run_integration(_arguments: object) -> None:
                test_root.mkdir()
                (test_root / "diagnostic.txt").touch()
                if error is not None:
                    raise error

            with (
                mock.patch.object(examples_integration, "_parse_arguments", return_value=arguments),
                mock.patch.object(examples_integration, "_run_full_integration", side_effect=run_integration),
            ):
                if error is None:
                    self.assertEqual(examples_integration.main(), 0)
                else:
                    with self.assertRaises(type(error)):
                        examples_integration.main()
            return test_root.exists()

    def test_main_removes_successful_full_integration_tree(self) -> None:
        """Remove scratch output after the complete integration phase succeeds."""
        self.assertFalse(self._run_integration())

    def test_main_preserves_failed_integration_tree(self) -> None:
        """Keep scratch output when the integration fails so it can be diagnosed."""
        self.assertTrue(self._run_integration(error=RuntimeError("failure")))

    @unittest.skipIf(os.name == "nt", "Windows resolves native dependencies through PATH.")
    def test_runtime_environment_keeps_python_native_libraries_first(self) -> None:
        """Keep the native dependency family selected by the Python interpreter coherent."""
        with tempfile.TemporaryDirectory() as temporary_directory:
            root = Path(temporary_directory)
            python = root / "python-environment" / "bin" / "python"
            sdk_prefix = root / "sdk"
            python_prefix = root / "python-package"
            runtime_dependencies = root / "runtime-dependencies"
            arguments = SimpleNamespace(
                python=python,
                python_install_dir="python",
                python_runtime_deps_dir=runtime_dependencies,
            )
            for inherited_paths in ((), (root / "inherited-a", root / "inherited-b")):
                with self.subTest(inherited=bool(inherited_paths)):
                    inherited_environment = {}
                    if inherited_paths:
                        inherited_environment["LD_LIBRARY_PATH"] = os.pathsep.join(map(str, inherited_paths))
                    with mock.patch.dict(examples_integration.os.environ, inherited_environment, clear=True):
                        environment = examples_integration._runtime_environment(arguments, sdk_prefix, python_prefix)

                    self.assertEqual(
                        environment["LD_LIBRARY_PATH"].split(os.pathsep),
                        [str(python.parent.parent / "lib"), str(sdk_prefix / "lib"), *map(str, inherited_paths)],
                    )


class DeveloperArtifactStateTests(unittest.TestCase):
    """Test rejection of absent, partial, and changed developer artifacts."""

    def test_source_digest_matches_original_path_semantics(self) -> None:
        """Preserve fingerprint bytes for relative roots, exclusions, Unicode, and links."""
        excluded = {"__pycache__", ".mypy_cache", ".pytest_cache", ".ruff_cache", "dist"}

        def original_digest(root: Path) -> str:
            """Compute the build-compatible reference fingerprint."""
            digest = hashlib.sha256()
            for path in sorted(root.rglob("*")):
                relative = path.relative_to(root)
                if any(part in excluded for part in relative.parts) or not path.is_file():
                    continue
                digest.update(relative.as_posix().encode())
                digest.update(b"\0")
                with path.open("rb") as stream:
                    for chunk in iter(lambda: stream.read(1024 * 1024), b""):
                        digest.update(chunk)
            return digest.hexdigest()

        with tempfile.TemporaryDirectory() as temporary_directory:
            root = Path(temporary_directory) / "libraries"
            for name in ("root.py", "nested/a.cpp", "nested/deeper/cube.c", "nested/日本語.py", "file/dist"):
                path = root / name
                path.parent.mkdir(parents=True, exist_ok=True)
                path.write_text(f"Contents for {name}\n", encoding="utf-8")
            for name in excluded:
                path = root / "generated" / name / "ignored.py"
                path.parent.mkdir(parents=True, exist_ok=True)
                path.write_text("Generated output\n", encoding="utf-8")
            if os.name != "nt":
                (root / "linked-file.py").symlink_to(root / "root.py")
                (root / "linked-directory").symlink_to(root / "nested", target_is_directory=True)
            expected = original_digest(root)
            for candidate in (root, Path(os.path.relpath(root))):
                with self.subTest(root=candidate):
                    self.assertEqual(original_digest(candidate), expected)
                    self.assertEqual(examples_runner._source_digest(candidate), expected)

    def _write_state(self, repository_root: Path, build_dir: Path, *, targets: list[str]) -> Path:
        """Write matching completed-build state and return its tracked input.

        Args:
            repository_root: Repository root directory.
            build_dir: Directory containing built examples.
            targets: Selected example targets.

        Returns:
            The resulting value.
        """
        source_root = repository_root / "source" / "libraries"
        source_root.mkdir(parents=True, exist_ok=True)
        manifest = repository_root / "pixi.toml"
        manifest.write_text("locked\n", encoding="utf-8")
        developer_environment = build_dir / examples_runner.DEVELOPER_ENVIRONMENT_DIRECTORY
        developer_environment.mkdir(exist_ok=True)
        (build_dir / "artifact-state.json").write_text(
            json.dumps(
                {
                    "schema_version": 1,
                    "configuration": "Release",
                    "profile": "standard",
                    "dependency_profile": "locked",
                    "inputs": {"pixi.toml": hashlib.sha256(manifest.read_bytes()).hexdigest()},
                    "source_digest": examples_runner._source_digest(source_root),
                    "targets": targets,
                    "developer_environment": {
                        "path": examples_runner.DEVELOPER_ENVIRONMENT_DIRECTORY,
                        "components": ["isaacsim_common-runtime"],
                    },
                }
            ),
            encoding="utf-8",
        )
        return manifest

    def test_validate_developer_artifact_state_accepts_matching_full_build(self) -> None:
        """Accept a full build whose locked inputs have not changed."""
        with tempfile.TemporaryDirectory() as temporary_directory:
            repository_root = Path(temporary_directory)
            build_dir = repository_root / "build"
            build_dir.mkdir()
            self._write_state(repository_root, build_dir, targets=["all"])

            actual_environment = examples_runner._validate_developer_artifact_state(
                repository_root, build_dir, "Release"
            )

            self.assertEqual(actual_environment, build_dir / examples_runner.DEVELOPER_ENVIRONMENT_DIRECTORY)

    def test_artifact_state_digest_tracks_sdk_identity_without_build_time(self) -> None:
        """Invalidate examples for changed SDK inputs without invalidating identical rebuilds."""
        with tempfile.TemporaryDirectory() as temporary_directory:
            repository_root = Path(temporary_directory)
            build_dir = repository_root / "build"
            build_dir.mkdir()
            self._write_state(repository_root, build_dir, targets=["all"])
            artifact_path = build_dir / "artifact-state.json"
            original = examples_runner._artifact_state_digest(artifact_path)
            state = json.loads(artifact_path.read_text(encoding="utf-8"))
            state["completed_at"] = "later"
            artifact_path.write_text(json.dumps(state), encoding="utf-8")
            self.assertEqual(examples_runner._artifact_state_digest(artifact_path), original)
            state["source_digest"] = "0" * 64
            artifact_path.write_text(json.dumps(state), encoding="utf-8")
            self.assertNotEqual(examples_runner._artifact_state_digest(artifact_path), original)

    def test_validate_developer_artifact_state_rejects_changed_input(self) -> None:
        """Require a rebuild after a locked manifest changes."""
        with tempfile.TemporaryDirectory() as temporary_directory:
            repository_root = Path(temporary_directory)
            build_dir = repository_root / "build"
            build_dir.mkdir()
            manifest = self._write_state(repository_root, build_dir, targets=["all"])
            manifest.write_text("changed\n", encoding="utf-8")

            with self.assertRaisesRegex(examples_runner.ExampleError, "changed since the build"):
                examples_runner._validate_developer_artifact_state(repository_root, build_dir, "Release")

    def test_validate_developer_artifact_state_rejects_partial_build(self) -> None:
        """Require a full library build before materializing developer examples."""
        with tempfile.TemporaryDirectory() as temporary_directory:
            repository_root = Path(temporary_directory)
            build_dir = repository_root / "build"
            build_dir.mkdir()
            self._write_state(repository_root, build_dir, targets=["isaacsim_common"])

            with self.assertRaisesRegex(examples_runner.ExampleError, "stale or partial"):
                examples_runner._validate_developer_artifact_state(repository_root, build_dir, "Release")

    def test_validate_developer_artifact_state_rejects_changed_library_sources(self) -> None:
        """Reject changed Python, C, and C++ library sources until the libraries are rebuilt."""
        for source_name in ("Module.py", "Module.c", "Module.cpp"):
            with self.subTest(source_name=source_name), tempfile.TemporaryDirectory() as temporary_directory:
                repository_root = Path(temporary_directory)
                build_dir = repository_root / "build"
                source_root = repository_root / "source" / "libraries"
                build_dir.mkdir()
                source_root.mkdir(parents=True)
                source_path = source_root / source_name
                source_path.write_text("original\n", encoding="utf-8")
                self._write_state(repository_root, build_dir, targets=["all"])
                source_path.write_text("changed\n", encoding="utf-8")

                with self.assertRaisesRegex(examples_runner.ExampleError, "sources changed since the build"):
                    examples_runner._validate_developer_artifact_state(repository_root, build_dir, "Release")

    def test_validate_developer_artifact_state_accepts_changed_example_sources(self) -> None:
        """Keep the environment valid after Python, C, and C++ example changes."""
        with tempfile.TemporaryDirectory() as temporary_directory:
            repository_root = Path(temporary_directory)
            build_dir = repository_root / "build"
            example_root = repository_root / "source" / "examples"
            build_dir.mkdir()
            example_root.mkdir(parents=True)
            source_paths = [example_root / name for name in ("main.py", "Main.c", "Main.cpp")]
            for source_path in source_paths:
                source_path.write_text("original\n", encoding="utf-8")
            self._write_state(repository_root, build_dir, targets=["all"])

            for source_path in source_paths:
                source_path.write_text("changed\n", encoding="utf-8")
                examples_runner._validate_developer_artifact_state(repository_root, build_dir, "Release")

    def test_bootstrap_preserves_metadata_checks_when_source_hash_is_deferred(self) -> None:
        """Reject malformed source metadata and changed configure inputs before restarting."""
        with tempfile.TemporaryDirectory() as temporary_directory:
            root = Path(temporary_directory)
            build_dir = root / "build"
            build_dir.mkdir()
            manifest = self._write_state(root, build_dir, targets=["all"])
            artifact_path = build_dir / "artifact-state.json"
            state = json.loads(artifact_path.read_text(encoding="utf-8"))
            for digest in (None, "not-a-digest", 42):
                with self.subTest(digest=digest):
                    artifact_path.write_text(json.dumps({**state, "source_digest": digest}), encoding="utf-8")
                    with self.assertRaisesRegex(examples_runner.ExampleError, "sources changed"):
                        examples_runner._validate_developer_artifact_state(
                            root, build_dir, "Release", validate_sources=False
                        )
            artifact_path.write_text(json.dumps(state), encoding="utf-8")
            manifest.write_text("changed\n", encoding="utf-8")
            with self.assertRaisesRegex(examples_runner.ExampleError, "configure input changed"):
                examples_runner._validate_developer_artifact_state(root, build_dir, "Release", validate_sources=False)

    def test_deferred_source_hash_is_required_in_final_validation(self) -> None:
        """Allow bootstrap metadata reads but reject changed sources before execution."""
        with tempfile.TemporaryDirectory() as temporary_directory:
            root = Path(temporary_directory)
            build_dir = root / "build"
            build_dir.mkdir()
            self._write_state(root, build_dir, targets=["all"])
            (root / "source" / "libraries" / "new.py").write_text("changed\n", encoding="utf-8")
            examples_runner._validate_developer_artifact_state(root, build_dir, "Release", validate_sources=False)
            with self.assertRaisesRegex(examples_runner.ExampleError, "sources changed"):
                examples_runner._validate_developer_artifact_state(root, build_dir, "Release")


class DeveloperRunnerStartupTests(unittest.TestCase):
    """Test environment selection before version-specific manifest parsing."""

    def test_restart_selects_python_and_environment_before_loading_manifests(self) -> None:
        """Bootstrap directly into the full environment, including from Python 3.10."""
        root = EXAMPLES_RUNNER_PATH.parents[2]
        build_dir = root / "_cmake_build" / "isaacsim-libraries-release"
        developer_build = examples_runner._DeveloperBuild(
            library_build_dir=build_dir,
            cmake=Path("/tools/cmake"),
            python=Path("/tools/python"),
            python_install_dir="python",
            python_runtime_dependencies=Path("/tools/dependencies"),
            developer_environment=build_dir / "developer-environment",
            artifact_digest="sdk-digest",
            artifact_mtime_ns=1,
            generator="Ninja",
            generator_platform=None,
            generator_toolset=None,
            make_program=None,
        )
        for marker, executable in (
            ("", "/tools/python"),
            ("another-build", "/tools/python"),
            (f"bootstrap:{build_dir}", "/tools/python"),
            (str(build_dir), "/tools/python"),
            ("", sys.executable),
        ):
            developer_build = replace(developer_build, python=Path(executable))
            with (
                self.subTest(marker=marker, executable=executable),
                mock.patch.dict(os.environ, {examples_runner.DEVELOPER_ENVIRONMENT_VARIABLE: marker}, clear=True),
                mock.patch.object(sys, "argv", [str(EXAMPLES_RUNNER_PATH), "--dev", "run", "hello_world.c"]),
                mock.patch.object(examples_runner, "tomllib", None),
                mock.patch.object(examples_runner, "_developer_environment_lock"),
                mock.patch.object(examples_runner, "_read_developer_build", return_value=developer_build) as read_build,
                mock.patch.object(examples_runner, "_load_examples") as load_examples,
                mock.patch.object(examples_runner, "_load_system_requirements") as load_requirements,
                mock.patch.object(examples_runner, "_restart_in_environment", return_value=7) as restart,
                mock.patch("builtins.print"),
            ):
                self.assertEqual(examples_runner._main(), 7)
                read_build.assert_called_once_with(root, "Release", validate_sources=marker == str(build_dir))
                restart.assert_called_once()
                executable, environment = restart.call_args.args
                self.assertEqual(executable, developer_build.python)
                self.assertEqual(environment[examples_runner.DEVELOPER_ENVIRONMENT_VARIABLE], str(build_dir))
                self.assertEqual(
                    environment["CMAKE_PREFIX_PATH"],
                    os.pathsep.join(
                        (
                            str(developer_build.developer_environment),
                            str(developer_build.python_runtime_dependencies),
                        )
                    ),
                )
                self.assertEqual(
                    environment["PYTHONPATH"],
                    os.pathsep.join(
                        (
                            str(developer_build.developer_environment / "python"),
                            str(developer_build.python_runtime_dependencies),
                        )
                    ),
                )
                load_examples.assert_not_called()
                load_requirements.assert_not_called()

    def test_active_environment_still_rejects_stale_artifacts(self) -> None:
        """Never treat an inherited environment marker as proof of fresh artifacts."""
        root = EXAMPLES_RUNNER_PATH.parents[2]
        build_dir = root / "_cmake_build" / "isaacsim-libraries-release"
        with (
            mock.patch.dict(os.environ, {examples_runner.DEVELOPER_ENVIRONMENT_VARIABLE: str(build_dir)}),
            mock.patch.object(sys, "argv", [str(EXAMPLES_RUNNER_PATH), "--dev", "run", "hello_world.c"]),
            mock.patch.object(examples_runner, "_developer_environment_lock") as lock,
            mock.patch.object(
                examples_runner, "_read_developer_build", side_effect=examples_runner.ExampleError("Stale artifacts")
            ) as read_build,
            mock.patch.object(examples_runner, "_run_example") as run_example,
            mock.patch("builtins.print"),
        ):
            self.assertEqual(examples_runner._main(), 1)
            lock.assert_called_once_with(build_dir, exclusive=False)
            read_build.assert_called_once_with(root, "Release", validate_sources=True)
            run_example.assert_not_called()

    def test_matching_interpreter_and_environment_do_not_restart(self) -> None:
        """Continue to manifest parsing after validating an already selected environment."""
        root = EXAMPLES_RUNNER_PATH.parents[2]
        build_dir = root / "_cmake_build" / "isaacsim-libraries-release"
        build = SimpleNamespace(
            library_build_dir=build_dir,
            python=Path(sys.executable),
            artifact_digest="sdk-digest",
            artifact_mtime_ns=1,
        )
        with (
            mock.patch.dict(os.environ, {examples_runner.DEVELOPER_ENVIRONMENT_VARIABLE: str(build_dir)}),
            mock.patch.object(sys, "argv", [str(EXAMPLES_RUNNER_PATH), "--dev", "run", "hello_world.c"]),
            mock.patch.object(examples_runner, "_developer_environment_lock"),
            mock.patch.object(examples_runner, "_read_developer_build", return_value=build) as read_build,
            mock.patch.object(
                examples_runner, "_load_system_requirements", side_effect=examples_runner.ExampleError("Stop parsing")
            ) as load_requirements,
            mock.patch.object(examples_runner, "_restart_in_environment") as restart,
            mock.patch("builtins.print"),
        ):
            self.assertEqual(examples_runner._main(), 1)
            read_build.assert_called_once_with(root, "Release", validate_sources=True)
            load_requirements.assert_called_once()
            restart.assert_not_called()

    def test_windows_restart_forwards_arguments_environment_and_exit_code(self) -> None:
        """Wait for the replacement interpreter and return its exit status on Windows."""
        executable = Path("/tools/python")
        environment = {"EXAMPLE": "value"}
        with (
            mock.patch.object(sys, "argv", ["runner.py", "run", "hello_world.c", "--", "argument with spaces"]),
            mock.patch.object(examples_runner, "os", SimpleNamespace(name="nt")),
            mock.patch.object(examples_runner.subprocess, "run", return_value=SimpleNamespace(returncode=9)) as run,
        ):
            self.assertEqual(examples_runner._restart_in_environment(executable, environment), 9)
            run.assert_called_once_with(
                [str(executable), str(EXAMPLES_RUNNER_PATH), "run", "hello_world.c", "--", "argument with spaces"],
                env=environment,
                check=False,
            )

    def test_unix_restart_replaces_process_with_original_arguments(self) -> None:
        """Forward the argument vector and complete environment directly to exec."""
        executable = Path("/tools/python")
        environment = {"EXAMPLE": "value"}
        execve = mock.Mock(side_effect=SystemExit(0))
        with (
            mock.patch.object(sys, "argv", ["runner.py", "run", "hello_world.c"]),
            mock.patch.object(examples_runner, "os", SimpleNamespace(name="posix", execve=execve)),
            self.assertRaises(SystemExit),
        ):
            examples_runner._restart_in_environment(executable, environment)
        execve.assert_called_once_with(
            executable, [str(executable), str(EXAMPLES_RUNNER_PATH), "run", "hello_world.c"], environment
        )


class DeveloperEnvironmentTests(unittest.TestCase):
    """Test read-only use of the complete build-owned developer environment."""

    def test_create_developer_environment_adds_windows_ovstage_search_paths(self) -> None:
        """Prepend developer-prefix OVStage directories before other runtime paths."""
        with tempfile.TemporaryDirectory() as temporary_directory:
            root = Path(temporary_directory)
            prefix = root / "developer-environment"
            python_directory = root / "python-environment"
            runtime_dependencies = root / "runtime-dependencies"
            inherited = root / "inherited"
            plugins = prefix / "bin" / "plugins"
            prefix_paths = (
                prefix / "bin",
                prefix / "plugins",
                plugins,
                plugins / "omni.client.lib",
                plugins / "omni.usd_resolver",
            )
            usd_exchange_libraries = runtime_dependencies / "usd_exchange.libs"
            for directory in (*prefix_paths, python_directory, usd_exchange_libraries):
                directory.mkdir(parents=True, exist_ok=True)
            executable = python_directory / "python.exe"
            executable.touch()
            developer_build = examples_runner._DeveloperBuild(
                library_build_dir=root,
                cmake=root / "cmake.exe",
                python=executable,
                python_install_dir="python",
                python_runtime_dependencies=runtime_dependencies,
                developer_environment=prefix,
                artifact_digest="sdk-digest",
                artifact_mtime_ns=1,
                generator="Ninja",
                generator_platform=None,
                generator_toolset=None,
                make_program=None,
            )

            with (
                mock.patch.dict(examples_runner.os.environ, {"PATH": str(inherited)}, clear=True),
                mock.patch.object(examples_runner.os, "name", "nt"),
            ):
                actual = examples_runner._create_developer_environment(developer_build)

            self.assertEqual(
                actual["PATH"].split(os.pathsep),
                [*map(str, prefix_paths), str(python_directory), str(usd_exchange_libraries), str(inherited)],
            )

    def test_create_developer_environment_uses_one_unified_prefix(self) -> None:
        """Expose native and Python surfaces without invoking installation."""
        with tempfile.TemporaryDirectory() as temporary_directory:
            root = Path(temporary_directory)
            prefix = root / "developer-environment"
            python_prefix = root / "python-environment"
            runtime_dependencies = root / "runtime-dependencies"
            for directory in (
                prefix / "bin",
                prefix / "lib",
                prefix / "python",
                python_prefix / "bin",
                python_prefix / "lib",
                runtime_dependencies,
            ):
                directory.mkdir(parents=True)
            executable = python_prefix / "bin" / "python"
            executable.touch()
            developer_build = examples_runner._DeveloperBuild(
                library_build_dir=root,
                cmake=executable,
                python=executable,
                python_install_dir="python",
                python_runtime_dependencies=runtime_dependencies,
                developer_environment=prefix,
                artifact_digest="sdk-digest",
                artifact_mtime_ns=1,
                generator="Ninja",
                generator_platform=None,
                generator_toolset=None,
                make_program=None,
            )

            with mock.patch.object(examples_runner.subprocess, "run") as run:
                first = examples_runner._create_developer_environment(developer_build)
                second = examples_runner._create_developer_environment(developer_build)

            self.assertEqual(
                first["CMAKE_PREFIX_PATH"].split(examples_runner.os.pathsep),
                [str(prefix), str(runtime_dependencies)],
            )
            self.assertEqual(first["PYTHONPATH"].split(examples_runner.os.pathsep)[0], str(prefix / "python"))
            if os.name != "nt":
                self.assertEqual(
                    first["LD_LIBRARY_PATH"].split(os.pathsep)[:2],
                    [str(python_prefix / "lib"), str(prefix / "lib")],
                )
            self.assertEqual(first, second)
            run.assert_not_called()

    def test_validate_developer_artifact_state_rejects_missing_environment(self) -> None:
        """Reject completed state whose build-owned environment was removed."""
        with tempfile.TemporaryDirectory() as temporary_directory:
            repository_root = Path(temporary_directory)
            build_dir = repository_root / "build"
            build_dir.mkdir()
            fixture = DeveloperArtifactStateTests()
            fixture._write_state(repository_root, build_dir, targets=["all"])
            (build_dir / examples_runner.DEVELOPER_ENVIRONMENT_DIRECTORY).rmdir()

            with self.assertRaisesRegex(examples_runner.ExampleError, "missing or redirected"):
                examples_runner._validate_developer_artifact_state(repository_root, build_dir, "Release")

    def test_validate_developer_artifact_state_rejects_redirected_environment(self) -> None:
        """Reject a developer environment redirected outside its build tree."""
        with tempfile.TemporaryDirectory() as temporary_directory:
            repository_root = Path(temporary_directory)
            build_dir = repository_root / "build"
            external_environment = repository_root / "external"
            build_dir.mkdir()
            external_environment.mkdir()
            fixture = DeveloperArtifactStateTests()
            fixture._write_state(repository_root, build_dir, targets=["all"])
            environment_path = build_dir / examples_runner.DEVELOPER_ENVIRONMENT_DIRECTORY
            environment_path.rmdir()
            try:
                environment_path.symlink_to(external_environment, target_is_directory=True)
            except OSError as error:
                self.skipTest(f"Directory symlinks are unavailable: {error}")

            with self.assertRaisesRegex(examples_runner.ExampleError, "missing or redirected"):
                examples_runner._validate_developer_artifact_state(repository_root, build_dir, "Release")


class ModuleRequirementTests(unittest.TestCase):
    """Test manifest syntax compatibility for module requirements."""

    def test_load_requirements_accepts_equivalent_module_table_syntaxes(self) -> None:
        """Preserve module names and surfaces in inline records and array tables."""
        manifests = (
            """[requirements]
system = ["python"]
modules = [
    { name = "isaacsim_foundation", surfaces = ["python"] },
    { name = "isaacsim_ovsim", surfaces = ["native_sdk", "python"] },
]
""",
            """[requirements]
system = ["python"]

[[requirements.modules]]
name = "isaacsim_foundation"
surfaces = ["python"]

[[requirements.modules]]
name = "isaacsim_ovsim"
surfaces = ["native_sdk", "python"]
""",
        )
        with tempfile.TemporaryDirectory() as temporary_directory:
            manifest_path = Path(temporary_directory) / "example.toml"
            requirements = []
            for manifest in manifests:
                manifest_path.write_text(manifest, encoding="utf-8")
                table = examples_runner._read_toml(manifest_path)
                requirements.append(examples_runner._load_requirements(table["requirements"], "requirements"))

        self.assertEqual(requirements[0], requirements[1])
        self.assertEqual(
            [(module.name, module.surfaces) for module in requirements[0].modules],
            [("isaacsim_foundation", ("python",)), ("isaacsim_ovsim", ("native_sdk", "python"))],
        )


class PythonPackageRequirementTests(unittest.TestCase):
    """Test external Python package parsing and shared preparation."""

    @staticmethod
    def _requirements(python_packages: list[str]) -> dict[str, object]:
        """Create a requirements table with external Python packages.

        Args:
            python_packages: PEP 508 requirement strings.

        Returns:
            Manifest requirements table.
        """
        return {
            "modules": [{"name": "isaacsim_common", "surfaces": ["python"]}],
            "python_packages": python_packages,
            "system": ["python"],
        }

    def _make_example(self, root: Path, example_id: str, package: str) -> object:
        """Create one Python example with an external package requirement.

        Args:
            root: Example source directory.
            example_id: Unique example identifier.
            package: Exact external package requirement.

        Returns:
            Parsed example definition.
        """
        requirements = examples_runner._load_requirements(self._requirements([package]), "requirements")
        return examples_runner._Example(
            id=example_id,
            title=example_id.title(),
            summary="Demo.",
            owners=("isaacsim.common.logging",),
            categories=(),
            topics=(),
            root=root,
            build=examples_runner._BuildConfiguration(adapter="none"),
            run=examples_runner._RunConfiguration(adapter="python", path=root / "main.py"),
            requirements=requirements,
            tests=(),
        )

    @staticmethod
    def _publish_fake_distributions(
        requirements: tuple[object, ...], destination: Path, _working_directory: Path, _environment: dict[str, str]
    ) -> None:
        """Publish minimal distribution metadata for a mocked installation.

        Args:
            requirements: Resolved external package requirements.
            destination: Directory that receives the distribution metadata.
            _working_directory: Unused mocked installer working directory.
            _environment: Unused mocked installer environment.
        """
        for requirement in requirements:
            metadata_dir = destination / f"{requirement.name}-{requirement.version}.dist-info"
            metadata_dir.mkdir()
            metadata_dir.joinpath("METADATA").write_text(
                f"Metadata-Version: 2.1\nName: {requirement.name}\nVersion: {requirement.version}\n",
                encoding="utf-8",
            )

    def test_load_requirements_accepts_and_sorts_exact_pep508_pins(self) -> None:
        """Accept exact pins and normalize their distribution names."""
        requirements = examples_runner._load_requirements(
            self._requirements(["Transitions == 0.9.3", "typing_extensions==4.15.0"]), "requirements"
        )

        self.assertEqual(
            [requirement.specifier for requirement in requirements.python_packages],
            ["transitions==0.9.3", "typing-extensions==4.15.0"],
        )

    def test_load_requirements_rejects_unsupported_pep508_features(self) -> None:
        """Reject non-exact, conditional, extra, direct, and malformed requirements."""
        cases = {
            "range": "demo>=1.0",
            "wildcard": "demo==1.*",
            "marker": 'demo==1.0; python_version > "3.11"',
            "extra": "demo[feature]==1.0",
            "url": "demo @ https://example.invalid/demo.whl",
            "malformed": "not a requirement ==",
        }
        for name, requirement in cases.items():
            with self.subTest(name=name), self.assertRaises(examples_runner.ExampleError):
                examples_runner._load_requirements(self._requirements([requirement]), "requirements")

    def test_load_requirements_rejects_duplicate_normalized_names(self) -> None:
        """Reject spelling variants of the same distribution name."""
        with self.assertRaisesRegex(examples_runner.ExampleError, "duplicate distribution"):
            examples_runner._load_requirements(
                self._requirements(["demo-package==1.0", "demo_package==1.0"]), "requirements"
            )

    def test_load_requirements_rejects_isaac_distribution_as_external(self) -> None:
        """Keep Isaac Sim distributions in the version-matched module contract."""
        with self.assertRaisesRegex(examples_runner.ExampleError, "through requirements.modules"):
            examples_runner._load_requirements(self._requirements(["isaacsim-common==7.0.0"]), "requirements")

    def test_validate_requirement_coherence_requires_python_execution(self) -> None:
        """Reject external packages on a native-only example."""
        requirements = examples_runner._load_requirements(self._requirements(["demo==1.0"]), "requirements")

        with self.assertRaisesRegex(examples_runner.ExampleError, "without a Python run adapter"):
            examples_runner._validate_requirement_coherence(
                examples_runner._BuildConfiguration(adapter="none"),
                examples_runner._RunConfiguration(adapter="executable", target="demo"),
                requirements,
                "example.toml",
            )

    def test_collect_python_packages_accepts_equivalent_versions(self) -> None:
        """Choose equivalent PEP 440 spellings independently of example order."""
        with tempfile.TemporaryDirectory() as temporary_directory:
            root = Path(temporary_directory)
            examples = (
                self._make_example(root, "first", "demo-package==1.0.0"),
                self._make_example(root, "second", "demo-package==1.0"),
            )

            forward = examples_runner._collect_python_packages(examples)
            reverse = examples_runner._collect_python_packages(tuple(reversed(examples)))

            self.assertEqual(forward, reverse)
            self.assertEqual([requirement.specifier for requirement in forward], ["demo-package==1.0"])

    def test_test_environment_rejects_bytecode_override_case_insensitively(self) -> None:
        """Keep tests from enabling bytecode generation."""
        with self.assertRaisesRegex(examples_runner.ExampleError, "must not override PYTHONDONTWRITEBYTECODE"):
            examples_runner._load_test_configurations(
                [{"name": "default", "environment": {"pythondontwritebytecode": "0"}}],
                Path(),
                examples_runner._RunConfiguration(adapter="python", path=Path("main.py")),
                "example.toml.test",
            )

    def test_prepare_python_dependencies_installs_union_into_shared_directory(self) -> None:
        """Install the collection's requirement union into one shared directory."""
        with tempfile.TemporaryDirectory() as temporary_directory:
            root = Path(temporary_directory)
            example_root = root / "example"
            example_root.mkdir()

            examples = (
                self._make_example(example_root, "first", "zebra-package==2.0"),
                self._make_example(example_root, "second", "alpha-package==1.0"),
            )
            build_root = root / "build"
            installed_python_path = str(root / "installed" / "isaac")

            with (
                mock.patch.dict(os.environ, {"PYTHONPATH": installed_python_path}),
                mock.patch.object(importlib.util, "find_spec", return_value=object()),
                mock.patch.object(
                    examples_runner,
                    "_install_python_dependencies",
                    side_effect=self._publish_fake_distributions,
                ) as install_dependencies,
                mock.patch.object(examples_runner, "_run_checked") as run_checked,
            ):
                prepared = examples_runner._prepare_python_dependencies(examples, build_root)
                required = examples_runner._require_python_dependencies(examples, build_root)

            self.assertEqual(prepared, required)
            self.assertEqual(prepared, build_root / examples_runner.PYTHON_DEPENDENCY_DIRECTORY)
            install_dependencies.assert_called_once()
            installed_requirements = install_dependencies.call_args.args[0]
            self.assertEqual(
                [requirement.specifier for requirement in installed_requirements],
                ["alpha-package==1.0", "zebra-package==2.0"],
            )
            self.assertNotIn("PYTHONPATH", install_dependencies.call_args.args[3])
            run_checked.assert_called_once()
            check_environment = run_checked.call_args.args[2]
            self.assertEqual(check_environment["PYTHONPATH"].split(os.pathsep)[1], installed_python_path)
            for example in examples:
                runtime_environment = examples_runner._create_runtime_environment(
                    example,
                    build_root / example.id,
                    prepared,
                    {"PYTHONPATH": str(root / "test-packages")},
                )
                self.assertEqual(
                    runtime_environment["PYTHONPATH"].split(os.pathsep),
                    [str(prepared), str(root / "test-packages")],
                )

    def test_remove_dependency_tree_handles_legacy_and_corrupt_targets(self) -> None:
        """Remove a legacy read-only tree or a regular file at the target path."""
        with tempfile.TemporaryDirectory() as temporary_directory:
            root = Path(temporary_directory)
            dependency_dir = root / "python-dependencies"
            package_dir = dependency_dir / "key" / "package"
            package_dir.mkdir(parents=True)
            (package_dir / "dependency.py").touch()
            package_dir.chmod(stat.S_IMODE(package_dir.stat().st_mode) & ~0o222)

            examples_runner._remove_dependency_tree(dependency_dir)
            dependency_dir.touch()
            examples_runner._remove_dependency_tree(dependency_dir)

            self.assertFalse(dependency_dir.exists())

    @unittest.skipIf(os.name == "nt", "Creating an unprivileged symlink is not portable on Windows")
    def test_remove_dependency_tree_does_not_chmod_symlink_target(self) -> None:
        """Remove a nested symlink without changing its external target."""
        with tempfile.TemporaryDirectory() as temporary_directory:
            root = Path(temporary_directory)
            target = root / "keep.py"
            target.touch(mode=0o400)
            dependency_dir = root / "python-dependencies"
            dependency_dir.mkdir()
            (dependency_dir / "redirect.py").symlink_to(target)
            dependency_dir.chmod(0o500)

            examples_runner._remove_dependency_tree(dependency_dir)

            self.assertEqual(stat.S_IMODE(target.stat().st_mode), 0o400)

    def test_install_python_dependencies_uses_declared_union(self) -> None:
        """Pass the normalized requirement union to pip without compiling bytecode."""
        requirements = (
            examples_runner._PythonPackageRequirement(name="alpha-package", version="1.0"),
            examples_runner._PythonPackageRequirement(name="zebra-package", version="2.0"),
        )
        with mock.patch.object(examples_runner, "_run_checked") as run_checked:
            examples_runner._install_python_dependencies(
                requirements, Path("destination"), Path("working-directory"), {"SAFE": "1"}
            )

        command, working_directory, environment = run_checked.call_args.args
        self.assertEqual(command[-2:], ["alpha-package==1.0", "zebra-package==2.0"])
        self.assertIn("--no-compile", command)
        self.assertEqual(working_directory, Path("working-directory"))
        self.assertEqual(environment, {"SAFE": "1"})

    def test_require_python_dependencies_never_installs_missing_target(self) -> None:
        """Require a build before runtime instead of invoking pip."""
        with tempfile.TemporaryDirectory() as temporary_directory:
            root = Path(temporary_directory)
            example_root = root / "example"
            example_root.mkdir()
            example = examples_runner._Example(
                id="demo",
                title="Demo",
                summary="Demo.",
                owners=("isaacsim.common.logging",),
                categories=(),
                topics=(),
                root=example_root,
                build=examples_runner._BuildConfiguration(adapter="none"),
                run=examples_runner._RunConfiguration(adapter="python", path=example_root / "main.py"),
                requirements=examples_runner._load_requirements(
                    self._requirements(["demo-package==1.2.3"]), "requirements"
                ),
                tests=(),
            )

            with (
                mock.patch.object(examples_runner, "_run_checked") as run_checked,
                self.assertRaisesRegex(examples_runner.ExampleError, "run the examples build command first"),
            ):
                examples_runner._require_python_dependencies((example,), root / "build")
            run_checked.assert_not_called()

    def test_require_python_dependencies_rejects_incomplete_target(self) -> None:
        """Reject a matching state marker when its declared distribution is missing."""
        with tempfile.TemporaryDirectory() as temporary_directory:
            root = Path(temporary_directory)
            example = self._make_example(root, "demo", "demo-package==1.0")
            requirements = examples_runner._collect_python_packages((example,))
            dependency_dir = root / "build" / examples_runner.PYTHON_DEPENDENCY_DIRECTORY
            dependency_dir.mkdir(parents=True)
            dependency_dir.joinpath(examples_runner.DEPENDENCY_STATE_FILE).write_text(
                json.dumps(examples_runner._python_dependency_state(requirements)), encoding="utf-8"
            )

            with self.assertRaisesRegex(examples_runner.ExampleError, "run the examples build command again"):
                examples_runner._require_python_dependencies((example,), root / "build")

    def test_prepare_python_dependencies_replaces_previous_install(self) -> None:
        """Rebuild the shared directory during every build phase."""
        with tempfile.TemporaryDirectory() as temporary_directory:
            root = Path(temporary_directory)
            example_root = root / "example"
            example_root.mkdir()
            example = self._make_example(example_root, "demo", "demo-package==1.0")
            with (
                mock.patch.object(importlib.util, "find_spec", return_value=object()),
                mock.patch.object(
                    examples_runner,
                    "_install_python_dependencies",
                    side_effect=self._publish_fake_distributions,
                ) as install_dependencies,
                mock.patch.object(examples_runner, "_run_checked"),
            ):
                first = examples_runner._prepare_python_dependencies((example,), root / "build")
                second = examples_runner._prepare_python_dependencies((example,), root / "build")

            self.assertEqual(first, second)
            self.assertEqual(install_dependencies.call_count, 2)
            self.assertEqual(list((root / "build").iterdir()), [second])

    def test_prepare_python_dependencies_restores_previous_install_on_publish_failure(self) -> None:
        """Keep the previous shared directory when publishing its replacement fails."""
        failures = (
            (OSError("injected publication failure"), False, examples_runner.ExampleError),
            (KeyboardInterrupt("injected backup interruption"), True, KeyboardInterrupt),
        )
        for failure, interrupt_after_backup, expected_error in failures:
            with self.subTest(failure=type(failure).__name__), tempfile.TemporaryDirectory() as temporary_directory:
                root = Path(temporary_directory)
                build_root = root / "build"
                dependency_dir = build_root / examples_runner.PYTHON_DEPENDENCY_DIRECTORY
                dependency_dir.mkdir(parents=True)
                previous_file = dependency_dir / "previous.txt"
                previous_file.write_text("keep", encoding="utf-8")
                example = self._make_example(root, "demo", "demo-package==1.0")
                real_rename = Path.rename

                def fail_staging_publish(source: Path, target: Path) -> Path:
                    if interrupt_after_backup:
                        result = real_rename(source, target)
                        if source == dependency_dir:
                            raise failure
                        return result
                    if source.joinpath(examples_runner.DEPENDENCY_STATE_FILE).is_file():
                        raise failure
                    return real_rename(source, target)

                with (
                    mock.patch.object(importlib.util, "find_spec", return_value=object()),
                    mock.patch.object(
                        examples_runner,
                        "_install_python_dependencies",
                        side_effect=self._publish_fake_distributions,
                    ),
                    mock.patch.object(examples_runner, "_run_checked"),
                    mock.patch.object(Path, "rename", autospec=True, side_effect=fail_staging_publish),
                    self.assertRaisesRegex(expected_error, str(failure)),
                ):
                    examples_runner._prepare_python_dependencies((example,), build_root)

                self.assertEqual(previous_file.read_text(encoding="utf-8"), "keep")
                self.assertEqual(list(build_root.iterdir()), [dependency_dir])

    def test_prepare_python_dependencies_rejects_dry_run_without_install(self) -> None:
        """Do not publish a marker when pip reports success without installing packages."""
        with tempfile.TemporaryDirectory() as temporary_directory:
            root = Path(temporary_directory)
            example_root = root / "example"
            example_root.mkdir()
            example = self._make_example(example_root, "demo", "demo-package==1.0")
            with (
                mock.patch.dict(os.environ, {"PIP_DRY_RUN": "1"}),
                mock.patch.object(importlib.util, "find_spec", return_value=object()),
                mock.patch.object(examples_runner, "_install_python_dependencies") as install_dependencies,
                mock.patch.object(examples_runner, "_run_checked") as run_checked,
                self.assertRaisesRegex(examples_runner.ExampleError, "did not produce demo-package==1.0"),
            ):
                examples_runner._prepare_python_dependencies((example,), root / "build")

            self.assertEqual(install_dependencies.call_args.args[3]["PIP_DRY_RUN"], "1")
            run_checked.assert_not_called()
            self.assertFalse((root / "build" / examples_runner.PYTHON_DEPENDENCY_DIRECTORY).exists())

    def test_prepare_python_dependencies_requires_pip(self) -> None:
        """Report a clear build-time error when pip is unavailable."""
        with tempfile.TemporaryDirectory() as temporary_directory:
            root = Path(temporary_directory)
            example_root = root / "example"
            example_root.mkdir()
            example = self._make_example(example_root, "demo", "demo-package==1.0")
            with (
                mock.patch.object(importlib.util, "find_spec", return_value=None),
                self.assertRaisesRegex(examples_runner.ExampleError, "requires pip"),
            ):
                examples_runner._prepare_python_dependencies((example,), root / "build")

    def test_require_python_dependencies_does_not_need_pip(self) -> None:
        """Consume a transferred target without invoking or probing pip."""
        with tempfile.TemporaryDirectory() as temporary_directory:
            root = Path(temporary_directory)
            example_root = root / "example"
            example_root.mkdir()
            example = self._make_example(example_root, "demo", "portable-demo-package==1.0")
            with (
                mock.patch.object(importlib.util, "find_spec", return_value=object()),
                mock.patch.object(
                    examples_runner,
                    "_install_python_dependencies",
                    side_effect=self._publish_fake_distributions,
                ),
                mock.patch.object(examples_runner, "_run_checked"),
            ):
                prepared = examples_runner._prepare_python_dependencies((example,), root / "build")

            with (
                mock.patch.object(sys, "executable", "/different/consumer/python"),
                mock.patch.dict(os.environ, {}, clear=True),
                mock.patch.object(importlib.util, "find_spec", return_value=None) as find_spec,
            ):
                required = examples_runner._require_python_dependencies((example,), root / "build")

            self.assertEqual(required, prepared)
            find_spec.assert_not_called()

    def test_ambient_paths_reject_relative_pythonpath_entries(self) -> None:
        """Avoid validating relative paths against a different child working directory."""
        with (
            mock.patch.dict(os.environ, {"PYTHONPATH": "relative/packages"}),
            self.assertRaisesRegex(examples_runner.ExampleError, "PYTHONPATH entries must be absolute"),
        ):
            examples_runner._ambient_python_paths()

    def test_dependency_state_rejects_different_requirements(self) -> None:
        """Reject a shared target prepared for a different requirement union."""
        with tempfile.TemporaryDirectory() as temporary_directory:
            dependency_dir = Path(temporary_directory)
            expected_state = {"schema_version": 1, "requirements": ["demo==1.0"]}
            state = {"schema_version": 1, "requirements": ["demo==2.0"]}
            (dependency_dir / examples_runner.DEPENDENCY_STATE_FILE).write_text(json.dumps(state), encoding="utf-8")

            self.assertFalse(examples_runner._dependency_state_matches(dependency_dir, expected_state))

    def test_dependency_state_ignores_compatible_tag_list_variation(self) -> None:
        """Accept compatible runtimes whose ordered wheel tag lists differ."""
        requirements = (examples_runner._PythonPackageRequirement(name="demo", version="1.0"),)
        with (
            mock.patch.object(
                examples_runner,
                "sys_tags",
                return_value=iter(
                    (
                        SimpleNamespace(interpreter="cp312", abi="cp312", platform="manylinux_2_17_x86_64"),
                        SimpleNamespace(interpreter="cp312", abi="abi3", platform="manylinux_2_17_x86_64"),
                    )
                ),
            ),
            mock.patch.object(examples_runner.sysconfig, "get_platform", return_value="linux-x86_64"),
        ):
            first = examples_runner._python_dependency_state(requirements)
        with (
            mock.patch.object(
                examples_runner,
                "sys_tags",
                return_value=iter(
                    (
                        SimpleNamespace(interpreter="cp312", abi="cp312", platform="manylinux_2_39_x86_64"),
                        SimpleNamespace(interpreter="cp312", abi="abi3", platform="manylinux_2_39_x86_64"),
                        SimpleNamespace(interpreter="py3", abi="none", platform="any"),
                    )
                ),
            ),
            mock.patch.object(examples_runner.sysconfig, "get_platform", return_value="linux-x86_64"),
        ):
            second = examples_runner._python_dependency_state(requirements)

        self.assertEqual(first, second)

    def test_dependency_state_changes_with_abi_or_platform(self) -> None:
        """Reject a transferred dependency directory built for another Python ABI or platform."""
        requirements = (examples_runner._PythonPackageRequirement(name="demo", version="1.0"),)
        with (
            mock.patch.object(examples_runner, "sys_tags", return_value=iter((SimpleNamespace(abi="cp312"),))),
            mock.patch.object(examples_runner.sysconfig, "get_platform", return_value="linux-x86_64"),
        ):
            expected = examples_runner._python_dependency_state(requirements)
        with (
            mock.patch.object(examples_runner, "sys_tags", return_value=iter((SimpleNamespace(abi="cp312t"),))),
            mock.patch.object(examples_runner.sysconfig, "get_platform", return_value="linux-x86_64"),
        ):
            different_abi = examples_runner._python_dependency_state(requirements)
        with (
            mock.patch.object(examples_runner, "sys_tags", return_value=iter((SimpleNamespace(abi="cp312"),))),
            mock.patch.object(examples_runner.sysconfig, "get_platform", return_value="win-amd64"),
        ):
            different_platform = examples_runner._python_dependency_state(requirements)

        self.assertNotEqual(expected, different_abi)
        self.assertNotEqual(expected, different_platform)

    def test_external_closure_rejects_transitive_isaac_distributions(self) -> None:
        """Do not let an external dependency shadow an installed Isaac distribution."""
        with tempfile.TemporaryDirectory() as temporary_directory:
            metadata_dir = Path(temporary_directory) / "isaacsim_common-7.0.dist-info"
            metadata_dir.mkdir()
            metadata_dir.joinpath("METADATA").write_text(
                "Metadata-Version: 2.1\nName: isaacsim-common\nVersion: 7.0\n",
                encoding="utf-8",
            )

            with self.assertRaisesRegex(examples_runner.ExampleError, "contains Isaac Sim distribution"):
                examples_runner._validate_external_distributions(Path(temporary_directory))

    def test_external_closure_ignores_disabled_user_site_distributions(self) -> None:
        """Do not report conflicts from user-site packages hidden from example children."""
        with tempfile.TemporaryDirectory() as temporary_directory:
            root = Path(temporary_directory)
            overlay = root / "overlay"
            user_site = root / "user-site"
            for directory, version in ((overlay, "2.0"), (user_site, "1.0")):
                metadata_dir = directory / f"demo-{version}.dist-info"
                metadata_dir.mkdir(parents=True)
                metadata_dir.joinpath("METADATA").write_text(
                    f"Metadata-Version: 2.1\nName: demo\nVersion: {version}\n",
                    encoding="utf-8",
                )

            with (
                mock.patch.object(sys, "path", [str(user_site)]),
                mock.patch.object(examples_runner.site, "getusersitepackages", return_value=str(user_site)),
                mock.patch.dict(os.environ, {"PYTHONPATH": ""}),
            ):
                examples_runner._validate_external_distributions(overlay)
                with (
                    mock.patch.dict(os.environ, {"PYTHONPATH": str(user_site)}),
                    self.assertRaisesRegex(
                        examples_runner.ExampleError,
                        "demo==1.0 provided by the active Isaac Sim environment.*relevant example.toml requirement",
                    ),
                ):
                    examples_runner._validate_external_distributions(overlay)

    def test_external_closure_uses_first_ambient_distribution(self) -> None:
        """Match Python's first-path-wins import precedence for duplicate distributions."""
        with tempfile.TemporaryDirectory() as temporary_directory:
            root = Path(temporary_directory)
            paths = {name: root / name for name in ("overlay", "first", "second")}
            for name, version in (("overlay", "2.0"), ("first", "1.0"), ("second", "2.0")):
                metadata_dir = paths[name] / f"demo-{version}.dist-info"
                metadata_dir.mkdir(parents=True)
                metadata_dir.joinpath("METADATA").write_text(
                    f"Metadata-Version: 2.1\nName: demo\nVersion: {version}\n",
                    encoding="utf-8",
                )

            with (
                mock.patch.object(sys, "path", [str(paths["first"]), str(paths["second"])]),
                mock.patch.dict(os.environ, {"PYTHONPATH": ""}),
                self.assertRaisesRegex(
                    examples_runner.ExampleError,
                    "demo==1.0 provided by the active Isaac Sim environment.*relevant example.toml requirement",
                ),
            ):
                examples_runner._validate_external_distributions(paths["overlay"])


class CppPackageRequirementTests(unittest.TestCase):
    """Test external C++ package parsing and Pixi preparation."""

    @staticmethod
    def _requirements(cpp_packages: list[str]) -> dict[str, object]:
        """Create native requirements with external C++ packages."""
        return {
            "modules": [{"name": "isaacsim_common", "surfaces": ["native_sdk"]}],
            "cpp_packages": cpp_packages,
            "system": ["cmake", "cpp"],
        }

    def _make_example(self, root: Path, example_id: str, package: str) -> object:
        """Create one native example with an external package requirement."""
        return examples_runner._Example(
            id=example_id,
            title=example_id.title(),
            summary="Demo.",
            owners=("isaacsim.common.logging",),
            categories=(),
            topics=(),
            root=root,
            build=examples_runner._BuildConfiguration(adapter="cmake", targets=("demo",)),
            run=examples_runner._RunConfiguration(adapter="executable", target="demo"),
            requirements=examples_runner._load_requirements(self._requirements([package]), "requirements"),
            tests=(),
        )

    def test_load_requirements_accepts_exact_cpp_packages(self) -> None:
        """Accept exact Pixi pins and sort them by package name."""
        requirements = examples_runner._load_requirements(
            self._requirements(["zlib==1.3.2", "fmt==7.0.3"]), "requirements"
        )

        self.assertEqual(
            [requirement.specifier for requirement in requirements.cpp_packages],
            ["fmt==7.0.3", "zlib==1.3.2"],
        )

    def test_load_requirements_rejects_invalid_cpp_packages(self) -> None:
        """Reject ranges, channels, wildcards, uppercase, and duplicate package names."""
        for requirement in ("fmt>=7.0.3", "conda-forge::fmt==7.0.3", "Fmt==7.0.3", "fmt==7.*"):
            with self.subTest(requirement=requirement), self.assertRaises(examples_runner.ExampleError):
                examples_runner._load_requirements(self._requirements([requirement]), "requirements")
        with self.assertRaisesRegex(examples_runner.ExampleError, "duplicates"):
            examples_runner._load_requirements(self._requirements(["fmt==7.0.3", "fmt==7.0.3"]), "requirements")

    def test_cpp_dependency_platform_maps_supported_hosts(self) -> None:
        """Map supported host platform spellings to Pixi platforms."""
        for host, expected in (
            ("linux-x86_64", "linux-64"),
            ("linux-aarch64", "linux-aarch64"),
            ("win-amd64", "win-64"),
        ):
            with (
                self.subTest(host=host),
                mock.patch.object(examples_runner.sysconfig, "get_platform", return_value=host),
            ):
                self.assertEqual(examples_runner._cpp_dependency_platform(), expected)
        with mock.patch.object(examples_runner.sysconfig, "get_platform", return_value="unsupported"):
            with self.assertRaisesRegex(examples_runner.ExampleError, "unsupported"):
                examples_runner._cpp_dependency_platform()

    def test_collect_cpp_packages_rejects_conflicts(self) -> None:
        """Reject incompatible direct pins before invoking Pixi."""
        with tempfile.TemporaryDirectory() as temporary_directory:
            root = Path(temporary_directory)
            examples = (
                self._make_example(root, "first", "fmt==7.0.3"),
                self._make_example(root, "second", "fmt==8.0.0"),
            )

            with self.assertRaisesRegex(examples_runner.ExampleError, r"conflicting C\+\+ package requirements"):
                examples_runner._collect_cpp_packages(examples)

    def test_validate_requirement_coherence_requires_cmake_cpp(self) -> None:
        """Reject C++ packages without a CMake C++ consumer."""
        requirements = examples_runner._load_requirements(self._requirements(["fmt==7.0.3"]), "requirements")

        with self.assertRaisesRegex(examples_runner.ExampleError, "without a CMake build adapter"):
            examples_runner._validate_requirement_coherence(
                examples_runner._BuildConfiguration(adapter="none"),
                examples_runner._RunConfiguration(adapter="executable", target="demo"),
                requirements,
                "example.toml",
            )

    def test_prepare_cpp_dependencies_installs_union_once(self) -> None:
        """Generate one workspace and install its complete requirement union."""
        with tempfile.TemporaryDirectory() as temporary_directory:
            root = Path(temporary_directory)
            pixi = root / "pixi"
            pixi.touch()
            examples = (
                self._make_example(root, "first", "zlib==1.3.2"),
                self._make_example(root, "second", "fmt==7.0.3"),
            )

            def install(_command: list[str], working_directory: Path, _environment: dict[str, str]) -> None:
                self.assertFalse(
                    {name.upper() for name in _environment} & {"PIXI_FROZEN", "PIXI_LOCKED", "PIXI_NO_INSTALL"}
                )
                working_directory.joinpath("pixi.lock").touch()
                working_directory.joinpath(".pixi", "envs", "default").mkdir(parents=True)

            with (
                mock.patch.object(examples_runner, "_find_pixi", return_value=pixi),
                mock.patch.object(examples_runner, "_cpp_dependency_platform", return_value="linux-64"),
                mock.patch.object(examples_runner, "_run_checked", side_effect=install) as run_checked,
                mock.patch.dict(
                    os.environ,
                    {"PIXI_FROZEN": "1", "pixi_locked": "1", "PiXi_No_Install": "1"},
                ),
            ):
                prepared = examples_runner._prepare_cpp_dependencies(examples, root / "build")
                required = examples_runner._require_cpp_dependencies(examples, root / "build")

            self.assertEqual(prepared, required)
            self.assertIsNotNone(prepared)
            manifest = prepared.manifest.read_text(encoding="utf-8")
            self.assertIn('"fmt" = "==7.0.3"\n"zlib" = "==1.3.2"', manifest)
            run_checked.assert_called_once()
            command = run_checked.call_args.args[0]
            self.assertEqual(command[1], "install")
            self.assertIn("--manifest-path", command)

    def test_require_cpp_dependencies_never_installs(self) -> None:
        """Require a build before runtime without invoking Pixi."""
        with tempfile.TemporaryDirectory() as temporary_directory:
            root = Path(temporary_directory)
            example = self._make_example(root, "demo", "fmt==7.0.3")
            with (
                mock.patch.object(examples_runner, "_find_pixi") as find_pixi,
                mock.patch.object(examples_runner, "_run_checked") as run_checked,
                self.assertRaisesRegex(examples_runner.ExampleError, "run the examples build command first"),
            ):
                examples_runner._require_cpp_dependencies((example,), root / "build")

            find_pixi.assert_not_called()
            run_checked.assert_not_called()

    def test_failed_cpp_dependency_update_invalidates_workspace(self) -> None:
        """Reject an old environment after installation of a changed pin fails."""
        with tempfile.TemporaryDirectory() as temporary_directory:
            root = Path(temporary_directory)
            pixi = root / "pixi"
            pixi.touch()
            old_example = self._make_example(root, "demo", "demo-package==1.0")
            new_example = self._make_example(root, "demo", "demo-package==2.0")

            def install(_command: list[str], working_directory: Path, _environment: dict[str, str]) -> None:
                working_directory.joinpath("pixi.lock").touch()
                working_directory.joinpath(".pixi", "envs", "default").mkdir(parents=True)

            with (
                mock.patch.object(examples_runner, "_find_pixi", return_value=pixi),
                mock.patch.object(examples_runner, "_cpp_dependency_platform", return_value="linux-64"),
                mock.patch.object(examples_runner, "_run_checked", side_effect=install),
            ):
                examples_runner._prepare_cpp_dependencies((old_example,), root / "build")

            with (
                mock.patch.object(examples_runner, "_find_pixi", return_value=pixi),
                mock.patch.object(examples_runner, "_cpp_dependency_platform", return_value="linux-64"),
                mock.patch.object(examples_runner, "_run_checked", side_effect=examples_runner.ExampleError("failed")),
                self.assertRaisesRegex(examples_runner.ExampleError, "failed"),
            ):
                examples_runner._prepare_cpp_dependencies((new_example,), root / "build")

            with (
                mock.patch.object(examples_runner, "_cpp_dependency_platform", return_value="linux-64"),
                self.assertRaisesRegex(examples_runner.ExampleError, "run the examples build command first"),
            ):
                examples_runner._require_cpp_dependencies((new_example,), root / "build")

    def test_wrap_cpp_dependency_command_uses_existing_environment(self) -> None:
        """Use Pixi's locked no-install execution mode for consumers."""
        with tempfile.TemporaryDirectory() as temporary_directory:
            root = Path(temporary_directory)
            example = self._make_example(root, "demo", "fmt==7.0.3")
            dependencies = examples_runner._CppDependencyEnvironment(pixi=root / "pixi", manifest=root / "pixi.toml")

            command = examples_runner._wrap_cpp_dependency_command(example, dependencies, ["demo"])

            self.assertIn("--no-install", command)
            self.assertIn("--locked", command)
            self.assertNotIn("install", command)
            self.assertEqual(command[-2:], ["--", "demo"])


class LinuxCompilerTests(unittest.TestCase):
    """Reuse the library compiler when building examples from source."""

    def test_developer_example_refreshes_only_mismatched_compiler_cache(self) -> None:
        """Migrate old host-compiler caches without discarding matching caches."""
        with tempfile.TemporaryDirectory() as temporary_directory:
            root = Path(temporary_directory)
            gcc, gxx = root / "gcc", root / "g++"
            (root / "CMakeCache.txt").write_text(
                f"CMAKE_C_COMPILER:FILEPATH={gcc}\nCMAKE_CXX_COMPILER:FILEPATH={gxx}\n"
            )
            developer = SimpleNamespace(library_build_dir=root, artifact_digest="sdk-one")
            source = root / "source"
            source.mkdir()
            example = SimpleNamespace(
                id="demo",
                root=source,
                build=SimpleNamespace(adapter="cmake", targets=("demo",)),
                requirements=SimpleNamespace(cpp_packages=(), system=()),
            )
            requirements = SimpleNamespace(
                cmake_minimum_version="3.26",
                cmake_policy_maximum_version="4.3",
                c_standard=11,
                cpp_standard=17,
                python_version="3.12",
            )
            build_dir = root / "demo"
            build_dir.mkdir()
            for cached, fresh in (("/usr/bin/gcc", True), (str(gcc), False)):
                with self.subTest(cached=cached):
                    (build_dir / "CMakeCache.txt").write_text(f"CMAKE_C_COMPILER:FILEPATH={cached}\n")
                    with (
                        mock.patch.object(examples_runner.sys, "platform", "linux"),
                        mock.patch.object(examples_runner, "_run_checked") as run,
                    ):
                        examples_runner._build_example(
                            example,
                            None,
                            None,
                            requirements,
                            root,
                            "cmake",
                            "Release",
                            "Ninja",
                            None,
                            None,
                            "ninja",
                            developer,
                        )
                    command = run.call_args_list[0].args[0]
                    self.assertEqual("--fresh" in command, fresh)
                    self.assertIn(f"-DCMAKE_C_COMPILER={gcc}", command)
                    self.assertIn(f"-DCMAKE_CXX_COMPILER={gxx}", command)
                    self.assertEqual(
                        json.loads((build_dir / ".example-build-state.json").read_text())["sdk_digest"], "sdk-one"
                    )


class GpuProbeTests(unittest.TestCase):
    """Test the host GPU probe that decides whether a GPU example's tests can run."""

    def setUp(self) -> None:
        """Start each case without an answer cached from the host or another case."""
        examples_runner._has_gpu.cache_clear()
        self.addCleanup(examples_runner._has_gpu.cache_clear)

    def test_probe_reports_a_gpu_when_the_driver_lists_one(self) -> None:
        """Report a GPU when the driver utility lists a device."""
        listed = subprocess.CompletedProcess([], 0, stdout="GPU 0: NVIDIA GeForce RTX 5080 (UUID: GPU-0)\n", stderr="")

        with (
            mock.patch.object(examples_runner.shutil, "which", return_value="/usr/bin/nvidia-smi"),
            mock.patch.object(examples_runner.subprocess, "run", return_value=listed),
        ):
            self.assertTrue(examples_runner._has_gpu())

    def test_probe_reports_no_gpu_without_a_driver(self) -> None:
        """Report no GPU when the driver utility is not installed."""
        with mock.patch.object(examples_runner.shutil, "which", return_value=None):
            self.assertFalse(examples_runner._has_gpu())

    def test_probe_reports_no_gpu_when_the_driver_lists_none(self) -> None:
        """Report no GPU when the driver is installed but reports no device."""
        empty = subprocess.CompletedProcess([], 0, stdout="", stderr="")

        with (
            mock.patch.object(examples_runner.shutil, "which", return_value="/usr/bin/nvidia-smi"),
            mock.patch.object(examples_runner.subprocess, "run", return_value=empty),
        ):
            self.assertFalse(examples_runner._has_gpu())

    def test_probe_reports_no_gpu_when_the_driver_is_unusable(self) -> None:
        """Report no GPU when the driver utility is present but fails or hangs."""
        failed = subprocess.CompletedProcess([], 9, stdout="", stderr="couldn't communicate with the driver\n")
        expired = subprocess.TimeoutExpired("nvidia-smi", examples_runner.GPU_PROBE_TIMEOUT_SECONDS)

        for outcome in ({"return_value": failed}, {"side_effect": expired}, {"side_effect": OSError()}):
            with self.subTest(outcome=outcome):
                examples_runner._has_gpu.cache_clear()
                with (
                    mock.patch.object(examples_runner.shutil, "which", return_value="/usr/bin/nvidia-smi"),
                    mock.patch.object(examples_runner.subprocess, "run", **outcome),
                ):
                    self.assertFalse(examples_runner._has_gpu())


class DocumentationCatalogTests(unittest.TestCase):
    """Test documentation catalog generation through the real examples parser."""

    def setUp(self) -> None:
        """Handle setUp."""
        self.temporary_directory = tempfile.TemporaryDirectory()
        self.root = Path(self.temporary_directory.name)
        self.library_catalog = self.root / "libraries.json"
        self.library_catalog.write_text(
            json.dumps(
                {
                    "schema_version": 1,
                    "release_version": "7.0.0.dev0",
                    "distributions": [
                        {
                            "name": "isaacsim_common",
                            "complete": True,
                            "modules": [{"name": "isaacsim.common.logging"}],
                        }
                    ],
                }
            ),
            encoding="utf-8",
        )

    def tearDown(self) -> None:
        """Handle tearDown."""
        self.temporary_directory.cleanup()

    def _write_example(
        self,
        relative_path: str,
        example_id: str,
        *,
        published: bool = True,
        run_path: str = "main.py",
    ) -> None:
        example_root = self.root / relative_path
        example_root.mkdir(parents=True)
        example_root.joinpath("README.md").write_text(f"# {example_id}\n", encoding="utf-8")
        entry_point = example_root / run_path
        entry_point.parent.mkdir(parents=True, exist_ok=True)
        entry_point.write_text("print('ok')\n", encoding="utf-8")
        example_root.joinpath("example.toml").write_text(
            f"""id = "{example_id}"
title = "{example_id}"
summary = "Catalog fixture."
owners = ["isaacsim.common.logging"]
topics = ["python"]
published = {str(published).lower()}

[build]
adapter = "none"

[run]
adapter = "python"
path = "{run_path}"

[requirements]
system = ["python"]

[[requirements.modules]]
name = "isaacsim_common"
surfaces = ["python"]
""",
            encoding="utf-8",
        )

    def _write_minimal_series(self, kind: str | None) -> None:
        """Write one valid series example with an optional series kind."""
        self._write_example("series/learn/first", "learn.first")
        series_root = self.root / "series" / "learn"
        series_root.joinpath("README.md").write_text("# Learn\n", encoding="utf-8")
        kind_field = f'kind = "{kind}"\n' if kind is not None else ""
        series_root.joinpath("series.toml").write_text(
            f"""id = "learn"
title = "Learn"
summary = "Series fixture."
{kind_field}
[[step]]
example = "learn.first"
level = "beginner"
""",
            encoding="utf-8",
        )

    def test_catalog_is_deterministic_and_filters_unpublished_examples(self) -> None:
        """Test catalog is deterministic and filters unpublished examples."""
        self._write_example("libraries/demo/zeta", "zeta")
        self._write_example("libraries/demo/alpha", "alpha")
        self._write_example("libraries/demo/hidden", "hidden", published=False)

        first = examples_runner._build_catalog(self.root, self.library_catalog)
        second = examples_runner._build_catalog(self.root, self.library_catalog)

        self.assertEqual(first, second)
        self.assertEqual(first["release_version"], "7.0.0.dev0")
        self.assertEqual([entry["id"] for entry in first["examples"]], ["alpha", "zeta"])
        self.assertEqual(first["examples"][0]["languages"], ["python"])

    def test_catalog_discovers_benchmarks(self) -> None:
        """Discover nested benchmark workloads without a teaching series."""
        self._write_example("benchmarks/physics/manager/cube", "benchmarks.physics.manager.cube")
        catalog = examples_runner._build_catalog(self.root, self.library_catalog)
        entry = catalog["examples"][0]
        self.assertEqual(entry["group"], "benchmark")
        self.assertIsNone(entry["series_id"])
        self.assertEqual(entry["source_path"], "benchmarks/physics/manager/cube")

    def test_catalog_exposes_external_python_packages(self) -> None:
        """Publish normalized external package requirements for inspection."""
        self._write_example("libraries/demo/packages", "packages")
        manifest = self.root / "libraries" / "demo" / "packages" / "example.toml"
        manifest.write_text(
            manifest.read_text(encoding="utf-8").replace(
                'system = ["python"]', 'system = ["python"]\npython_packages = ["Typing_Extensions==4.15.0"]'
            ),
            encoding="utf-8",
        )

        catalog = examples_runner._build_catalog(self.root, self.library_catalog)

        self.assertEqual(catalog["schema_version"], 4)
        self.assertEqual(catalog["examples"][0]["requirements"]["python_packages"], ["typing-extensions==4.15.0"])

    def test_catalog_exposes_external_cpp_packages(self) -> None:
        """Publish external C++ package pins for installers and documentation."""
        self._write_example("libraries/demo/packages", "packages")
        example_root = self.root / "libraries" / "demo" / "packages"
        manifest = example_root / "example.toml"
        manifest.write_text(
            manifest.read_text(encoding="utf-8")
            .replace('adapter = "none"', 'adapter = "cmake"\ntargets = ["demo"]')
            .replace('adapter = "python"\npath = "main.py"', 'adapter = "executable"\ntarget = "demo"')
            .replace('system = ["python"]', 'system = ["cmake", "cpp"]\ncpp_packages = ["fmt==7.0.3"]')
            .replace('surfaces = ["python"]', 'surfaces = ["native_sdk"]'),
            encoding="utf-8",
        )
        example_root.joinpath("CMakeLists.txt").write_text("cmake_minimum_required(VERSION 3.26)\n", encoding="utf-8")

        catalog = examples_runner._build_catalog(self.root, self.library_catalog)

        self.assertEqual(catalog["schema_version"], 4)
        self.assertEqual(catalog["examples"][0]["requirements"]["cpp_packages"], ["fmt==7.0.3"])

    def test_catalog_includes_named_commands_without_changing_default(self) -> None:
        """Publish named launch commands while retaining schema and default commands."""
        self._write_example("libraries/demo/named", "named")
        manifest = self.root / "libraries" / "demo" / "named" / "example.toml"
        manifest.write_text(
            manifest.read_text() + '\n[run.server]\nadapter="command"\ncommand="server"\n', encoding="utf-8"
        )
        catalog = examples_runner._build_catalog(self.root, self.library_catalog)
        self.assertEqual(catalog["schema_version"], 4)
        self.assertEqual(catalog["examples"][0]["entry_point"], "main.py")
        commands = catalog["examples"][0]["commands"]
        self.assertEqual(commands["run"], "python source/examples/examples.py run named")
        self.assertEqual(commands["run:server"], "python source/examples/examples.py run named:server")

    def test_catalog_preserves_nested_entry_point(self) -> None:
        """Test catalog preserves nested entry point."""
        self._write_example("libraries/demo/nested", "nested", run_path="scripts/main.py")

        catalog = examples_runner._build_catalog(self.root, self.library_catalog)

        self.assertEqual(catalog["examples"][0]["entry_point"], "scripts/main.py")

    def test_catalog_preserves_series_order_and_level(self) -> None:
        """Test catalog preserves series order and level."""
        self._write_example("series/learn/first", "learn.first")
        self._write_example("series/learn/second", "learn.second")
        series_root = self.root / "series" / "learn"
        series_root.joinpath("README.md").write_text("# Learn\n", encoding="utf-8")
        series_root.joinpath("series.toml").write_text(
            """id = "learn"
title = "Learn"
summary = "Ordered learning."
kind = "sequence"

[[step]]
example = "learn.second"
level = "intermediate"

[[step]]
example = "learn.first"
level = "beginner"
""",
            encoding="utf-8",
        )

        catalog = examples_runner._build_catalog(self.root, self.library_catalog)

        self.assertEqual(catalog["series"][0]["steps"], ["learn.second", "learn.first"])
        self.assertEqual(catalog["series"][0]["kind"], "sequence")
        examples = {entry["id"]: entry for entry in catalog["examples"]}
        self.assertEqual(examples["learn.second"]["series_position"], 1)
        self.assertEqual(examples["learn.second"]["level"], "intermediate")

    def test_catalog_serializes_collection_kind(self) -> None:
        """Preserve collection semantics in the generated catalog."""
        self._write_minimal_series("collection")

        catalog = examples_runner._build_catalog(self.root, self.library_catalog)

        self.assertEqual(catalog["series"][0]["kind"], "collection")

    def test_catalog_requires_series_kind(self) -> None:
        """Require authors to choose sequence or collection semantics."""
        self._write_minimal_series(None)

        with self.assertRaisesRegex(examples_runner.ExampleError, "missing fields: kind"):
            examples_runner._build_catalog(self.root, self.library_catalog)

    def test_catalog_rejects_unsupported_series_kind(self) -> None:
        """Reject series kinds outside the documented closed set."""
        self._write_minimal_series("tutorial")

        with self.assertRaisesRegex(examples_runner.ExampleError, r"\.kind is unsupported: tutorial"):
            examples_runner._build_catalog(self.root, self.library_catalog)

    def test_catalog_rejects_conflicting_series_python_packages(self) -> None:
        """Reject incompatible direct pins before preparing a series workspace."""
        self._write_example("series/learn/first", "learn.first")
        self._write_example("series/learn/second", "learn.second")
        series_root = self.root / "series" / "learn"
        series_root.joinpath("README.md").write_text("# Learn\n", encoding="utf-8")
        series_root.joinpath("series.toml").write_text(
            """id = "learn"
title = "Learn"
summary = "Ordered learning."
kind = "sequence"

[[step]]
example = "learn.first"
level = "beginner"

[[step]]
example = "learn.second"
level = "intermediate"
""",
            encoding="utf-8",
        )
        for step, version in (("first", "1.0"), ("second", "2.0")):
            manifest = series_root / step / "example.toml"
            manifest.write_text(
                manifest.read_text(encoding="utf-8").replace(
                    'system = ["python"]', f'system = ["python"]\npython_packages = ["demo=={version}"]'
                ),
                encoding="utf-8",
            )

        with self.assertRaisesRegex(examples_runner.ExampleError, "conflicting Python package requirements"):
            examples_runner._build_catalog(self.root, self.library_catalog)

    def test_catalog_groups_a_benchmark_example(self) -> None:
        """Assign the benchmark group to an example beneath a benchmarks category directory."""
        self._write_example("benchmarks/ovrtx/camera", "benchmark.ovrtx.camera")

        catalog = examples_runner._build_catalog(self.root, self.library_catalog)

        self.assertEqual(catalog["examples"][0]["group"], "benchmark")

    def test_catalog_reports_a_gpu_capability(self) -> None:
        """Carry a declared GPU requirement through to the catalog's capabilities."""
        self._write_example("benchmarks/ovrtx/camera", "benchmark.ovrtx.camera")
        manifest = self.root / "benchmarks" / "ovrtx" / "camera" / "example.toml"
        manifest.write_text(
            manifest.read_text(encoding="utf-8").replace('system = ["python"]', 'system = ["gpu", "python"]'),
            encoding="utf-8",
        )

        catalog = examples_runner._build_catalog(self.root, self.library_catalog)

        self.assertEqual(catalog["examples"][0]["requirements"]["capabilities"], ["gpu", "python"])

    def test_catalog_rejects_unknown_library_cross_references(self) -> None:
        """Test catalog rejects unknown library cross references."""
        self._write_example("libraries/demo/invalid", "invalid")
        manifest = self.root / "libraries" / "demo" / "invalid" / "example.toml"
        manifest.write_text(
            manifest.read_text(encoding="utf-8").replace("isaacsim_common", "missing_distribution"),
            encoding="utf-8",
        )

        with self.assertRaisesRegex(examples_runner.ExampleError, "unknown distributions"):
            examples_runner._build_catalog(self.root, self.library_catalog)

    def test_catalog_rejects_unknown_library_owner(self) -> None:
        """Test catalog rejects unknown library owner."""
        self._write_example("libraries/demo/invalid", "invalid")
        manifest = self.root / "libraries" / "demo" / "invalid" / "example.toml"
        manifest.write_text(
            manifest.read_text(encoding="utf-8").replace("isaacsim.common.logging", "isaacsim.missing.module"),
            encoding="utf-8",
        )

        with self.assertRaisesRegex(examples_runner.ExampleError, "unknown public API owners"):
            examples_runner._build_catalog(self.root, self.library_catalog)

    def test_catalog_rejects_references_to_incomplete_distributions(self) -> None:
        """Test catalog rejects references to incomplete distributions."""
        self._write_example("libraries/demo/invalid", "invalid")
        catalog = json.loads(self.library_catalog.read_text(encoding="utf-8"))
        catalog["distributions"][0]["complete"] = False
        self.library_catalog.write_text(json.dumps(catalog), encoding="utf-8")

        with self.assertRaisesRegex(examples_runner.ExampleError, "unknown distributions"):
            examples_runner._build_catalog(self.root, self.library_catalog)

    def test_catalog_rejects_unsupported_library_catalog_schema(self) -> None:
        """Test catalog rejects unsupported library catalog schema."""
        self._write_example("libraries/demo/invalid", "invalid")
        catalog = json.loads(self.library_catalog.read_text(encoding="utf-8"))
        catalog["schema_version"] = 2
        self.library_catalog.write_text(json.dumps(catalog), encoding="utf-8")

        with self.assertRaisesRegex(examples_runner.ExampleError, "Unsupported library documentation catalog schema"):
            examples_runner._build_catalog(self.root, self.library_catalog)


class NsysTests(unittest.TestCase):
    """Check profiling through the existing runner without Nsight or a GPU."""

    def setUp(self) -> None:
        """Load the command helper independently of workload dependencies."""
        spec = importlib.util.spec_from_file_location("nsys", EXAMPLES_RUNNER_PATH.parent / "nsys.py")
        self.nsys = importlib.util.module_from_spec(spec)
        with mock.patch.object(sys, "dont_write_bytecode", True):
            spec.loader.exec_module(self.nsys)
        self.addCleanup(mock.patch.stopall)
        mock.patch.dict(sys.modules, {"nsys": self.nsys}).start()

    def test_scopes_preserve_arguments_and_prepared_environment(self) -> None:
        """Capture the measured range only when requested, keeping workload arguments verbatim."""
        with tempfile.TemporaryDirectory() as directory:
            for scope in ("benchmark", "full"):
                with self.subTest(scope=scope):
                    environment = {"PATH": directory, "PYTHONPATH": "prepared dependencies"}
                    command = ["python", "a benchmark.py", "--output", "result with spaces.json"]
                    profiler_options = ("sample=none", "session-new=session with spaces")
                    with mock.patch.object(self.nsys.shutil, "which", return_value="/tools/nsys") as which:
                        launched, report = self.nsys.prepare_nsys_command(
                            command,
                            environment,
                            scope=scope,
                            output=Path(directory) / "profile.v1",
                            options=profiler_options,
                        )
                    which.assert_called_once_with("nsys", path=directory)
                    expected_tail = [*(f"--{option}" for option in profiler_options), *command]
                    self.assertEqual(launched[-len(expected_tail) :], expected_tail)
                    self.assertEqual(report, Path(directory) / "profile.v1.nsys-rep")
                    self.assertEqual(environment["PYTHONPATH"], "prepared dependencies")
                    self.assertEqual("--capture-range=nvtx" in launched, scope == "benchmark")
                    if scope == "benchmark":
                        self.assertIn("--nvtx-capture=MEASURE@isaacsim.performance.benchmarking", launched)
                        self.assertEqual(environment["NSYS_NVTX_PROFILER_REGISTER_ONLY"], "0")
                    else:
                        self.assertNotIn("NSYS_NVTX_PROFILER_REGISTER_ONLY", environment)

    def test_custom_trace_replaces_default(self) -> None:
        """Emit exactly one trace option so Nsight accepts custom trace selection."""
        with tempfile.TemporaryDirectory() as directory:
            for options, expected in (
                ((), "--trace=nvtx,cuda"),
                (("trace=nvtx,cuda,vulkan",), "--trace=nvtx,cuda,vulkan"),
            ):
                with (
                    self.subTest(options=options),
                    mock.patch.object(self.nsys.shutil, "which", return_value="/tools/nsys"),
                ):
                    command, _ = self.nsys.prepare_nsys_command(
                        ["workload"], {}, scope="full", output=Path(directory) / "report", options=options
                    )
                self.assertEqual([arg for arg in command if arg.startswith("--trace=")], [expected])

    def test_missing_profiler_and_existing_report_fail_before_launch(self) -> None:
        """Give actionable failures and preserve an existing report."""
        with tempfile.TemporaryDirectory() as directory:
            report = Path(directory) / "capture.nsys-rep"
            with mock.patch.object(self.nsys.shutil, "which", return_value=None):
                with self.assertRaises(FileNotFoundError):
                    self.nsys.prepare_nsys_command(["workload"], {}, scope="full", output=report)
            report.write_text("previous report", encoding="utf-8")
            with mock.patch.object(self.nsys.shutil, "which", return_value="/tools/nsys"):
                with self.assertRaises(FileExistsError):
                    self.nsys.prepare_nsys_command(["workload"], {}, scope="full", output=report)
            self.assertEqual(report.read_text(encoding="utf-8"), "previous report")

    def test_runner_profiles_inside_pixi_and_preserves_exit_status(self) -> None:
        """Use the existing child process, working directory, dependency wrapper, and exit handling."""
        cases = ((0, True, False), (0, True, True), (7, False, False), (0, False, False))
        for returncode, produces_report, default_output in cases:
            with (
                self.subTest(returncode=returncode, report=produces_report, default_output=default_output),
                tempfile.TemporaryDirectory() as directory,
            ):
                root = Path(directory)
                output = None if default_output else root / "profile"
                build_dir = root / "build"
                example = SimpleNamespace(
                    id="demo",
                    root=root,
                    run=SimpleNamespace(arguments=()),
                    requirements=SimpleNamespace(cpp_packages=("dependency",)),
                )
                dependencies = SimpleNamespace(pixi=Path("/tools/pixi"), manifest=root / "pixi.toml")
                environment = {"PATH": directory, "PYTHONPATH": "prepared dependencies"}
                process = mock.Mock()
                process.wait.return_value = returncode

                def launch(command: list[str], **kwargs: object) -> mock.Mock:
                    if produces_report:
                        report = Path(
                            next(value.removeprefix("--output=") for value in command if value.startswith("--output="))
                        )
                        report.write_text("recorded timeline", encoding="utf-8")
                    return process

                with (
                    mock.patch.object(examples_runner, "_create_runtime_environment", return_value=environment),
                    mock.patch.object(examples_runner, "_create_run_command", return_value=["workload", "two words"]),
                    mock.patch.object(self.nsys.shutil, "which", return_value="/tools/nsys"),
                    mock.patch.object(examples_runner.subprocess, "Popen", side_effect=launch) as popen,
                ):
                    if returncode == 0 and not produces_report:
                        with self.assertRaises(examples_runner.ExampleError):
                            examples_runner._run_example(
                                example,
                                build_dir,
                                (),
                                cpp_dependencies=dependencies,
                                nsys_scope="benchmark",
                                nsys_output=output,
                                nsys_options=("sample=none", "cpuctxsw=none"),
                            )
                    else:
                        self.assertEqual(
                            examples_runner._run_example(
                                example,
                                build_dir,
                                (),
                                cpp_dependencies=dependencies,
                                nsys_scope="benchmark",
                                nsys_output=output,
                                nsys_options=("sample=none", "cpuctxsw=none"),
                            ),
                            returncode,
                        )
                popen.assert_called_once()
                launched = popen.call_args.args[0]
                self.assertEqual(launched[0], str(dependencies.pixi))
                self.assertEqual(launched[launched.index("--") + 1], str(Path("/tools/nsys").absolute()))
                self.assertEqual(launched[-4:], ["--sample=none", "--cpuctxsw=none", "workload", "two words"])
                self.assertEqual(popen.call_args.kwargs["cwd"], root)
                self.assertEqual(popen.call_args.kwargs["env"]["PYTHONPATH"], "prepared dependencies")
                process.wait.assert_called_once_with()

    def test_runner_parser_keeps_workload_options_after_separator(self) -> None:
        """Runner flags precede the example; everything after it remains workload input."""
        parser = examples_runner._create_argument_parser(Path.cwd() / "build")
        arguments = parser.parse_args(
            [
                "run",
                "--nsys",
                "--nsys-scope",
                "full",
                "--nsys-output",
                "trace.v1",
                "--nsys-option",
                "sample=none",
                "--nsys-option",
                "session-new=session with spaces",
                "demo",
                "--",
                "--nsys",
            ]
        )
        self.assertTrue(arguments.nsys)
        self.assertEqual(arguments.nsys_scope, "full")
        self.assertEqual(arguments.nsys_output, Path("trace.v1"))
        self.assertEqual(arguments.nsys_option, ["sample=none", "session-new=session with spaces"])
        self.assertEqual(arguments.arguments, ["--nsys"])


if __name__ == "__main__":
    unittest.main()
