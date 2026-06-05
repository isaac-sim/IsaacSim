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

"""Test script that loads doc snippets and checks for errors.

This script:
1. Iterates over Python files in docs/isaacsim/snippets
2. For files that do NOT contain SimulationApp in uncommented lines, loads SimulationApp
3. Loads each file as a module and catches/stores any exceptions
4. Prints any exceptions with the snippet file name and trace
5. Returns appropriate status code (nonzero if exceptions, zero otherwise)
"""

from __future__ import annotations

import argparse
import asyncio
import atexit
import csv
import fnmatch
import importlib.util
import os
import platform
import re
import signal
import sys
import time
import traceback
import unittest
import xml.etree.ElementTree as ET
from collections import defaultdict
from datetime import datetime
from functools import partial
from pathlib import Path

# Note: SimulationApp is imported inside the experience loop to allow fresh imports
# after closing each SimulationApp instance.


def parse_args():
    """Parse command line arguments."""
    parser = argparse.ArgumentParser(description="Test script that loads doc snippets and checks for errors.")
    parser.add_argument(
        "-f",
        "--filter",
        type=str,
        nargs="*",
        default=None,
        help="Filter snippets to test only those in directories matching any of the given substrings.",
    )
    parser.add_argument(
        "--experience-csv",
        type=str,
        default=None,
        help="Path to a CSV file mapping snippet files to Kit app experiences "
        "(relative to the script directory or absolute). "
        "First column is the snippet path (relative to the doc_snippets directory), "
        "second column is the experience name.",
    )
    parser.add_argument(
        "--expected-failures-csv",
        type=str,
        default=None,
        help="Path to a CSV file listing snippets expected to fail. "
        "First column is the snippet path (relative to the doc_snippets directory), "
        "second column is an optional exception message regex pattern.",
    )
    parser.add_argument(
        "--snippet-timeout",
        type=int,
        default=120,
        help="Per-snippet timeout in seconds. If a snippet takes longer than this, "
        "it is aborted and reported as a failure. Default: 120.",
    )
    parser.add_argument(
        "--excluded-snippets-csv",
        type=str,
        default=None,
        help="Path to a CSV file listing snippets to skip entirely (e.g. snippets that "
        "crash or hang the test process). First column is the snippet path (relative to "
        "the snippets directory). Lines starting with '#' are comments.",
    )
    parser.add_argument(
        "--platform-constraints-csv",
        type=str,
        default=None,
        help="Path to a CSV file listing per-snippet platform constraints. "
        "First column is the snippet path (relative to the snippets directory), "
        "subsequent columns are allowed Isaac Sim platform names or fnmatch patterns.",
    )
    parser.add_argument(
        "--junit-xml",
        type=str,
        default=None,
        help="Path to write a JUnit XML report with one testcase per snippet. "
        "When set, the report is written after all tests complete so CI systems "
        "like GitLab can display per-snippet pass/fail rows.",
    )
    parser.add_argument(
        "--asset-root",
        type=str,
        default=None,
        help="Override the /persistent/isaac/asset_root/default carb setting "
        "after SimulationApp starts. Useful when the default Nucleus server is "
        "unreachable and you want to use S3 or a local path instead.",
    )
    return parser.parse_known_args()


def parse_experience_csv(csv_path, base_dir, snippets_root=None):
    """Parse the experience CSV file and return a mapping of absolute file paths to experiences.

    Args:
        csv_path: Path to the CSV file (relative to base_dir or absolute).
        base_dir: Base directory for resolving the CSV file path itself.
        snippets_root: Base directory for resolving snippet paths within the CSV.
            If None, defaults to base_dir.

    Returns:
        Dictionary mapping absolute file paths (as strings) to experience names.
    """
    if snippets_root is None:
        snippets_root = base_dir
    experience_map = {}
    # Resolve CSV path relative to base_dir if not absolute
    csv_file = Path(csv_path)
    root_path = Path(snippets_root)
    if not csv_file.is_absolute():
        csv_file = base_dir / csv_file
    if not csv_file.exists():
        print(f"Warning: Experience CSV file not found: {csv_file}")
        return experience_map

    with open(csv_file, "r", encoding="utf-8") as f:
        reader = csv.reader(f)
        for row in reader:
            if not row or row[0].strip().startswith("#"):
                continue
            if len(row) >= 2:
                snippet_path = row[0].strip()
                if not snippet_path:
                    continue
                experience = row[1].strip()
                abs_path = (root_path / snippet_path).resolve()
                experience_map[str(abs_path)] = experience

    return experience_map


def group_files_by_experience(files, experience_map):
    """Group files by their associated experience.

    Args:
        files: List of file paths to group.
        experience_map: Dictionary mapping file paths to experiences.

    Returns:
        Dictionary mapping experience names to lists of file paths.
    """
    groups = defaultdict(list)
    for file_path in files:
        experience = experience_map.get(str(file_path.resolve()), "")
        groups[experience].append(file_path)
    return groups


def resolve_experience_path(experience):
    """Resolve an experience name from the CSV to a Kit app path."""
    if not experience:
        return ""

    experience_path = Path(experience)
    candidates = []
    if experience_path.is_absolute():
        candidates.append(experience_path)
    else:
        exp_root = os.environ.get("EXP_PATH")
        if exp_root:
            candidates.append(Path(exp_root) / experience_path)
        candidates.append(experience_path)

    for candidate in candidates:
        if candidate.is_file():
            return str(candidate)

    print(f"Warning: Experience file not found for {experience!r}; passing value through unchanged")
    return experience


def get_current_platform_name() -> str:
    """Return the current platform using Isaac Sim platform target naming."""
    machine = platform.machine().lower()
    if machine in {"amd64", "x64"}:
        machine = "x86_64"
    elif machine == "arm64":
        machine = "aarch64"

    if sys.platform.startswith("linux"):
        return f"linux-{machine}"
    if sys.platform.startswith("win"):
        return f"windows-{machine}"
    return f"{sys.platform}-{machine}"


def parse_platform_constraints_csv(
    csv_path: str | Path, base_dir: str | Path, snippets_root: str | Path | None = None
) -> dict[str, tuple[str, ...]]:
    """Parse snippet platform constraints from CSV."""
    if snippets_root is None:
        snippets_root = base_dir

    constraints = {}
    csv_file = Path(csv_path)
    base_path = Path(base_dir)
    root_path = Path(snippets_root)
    if not csv_file.is_absolute():
        csv_file = base_path / csv_file
    if not csv_file.exists():
        print(f"Warning: Platform constraints CSV file not found: {csv_file}")
        return constraints

    with open(csv_file, "r", encoding="utf-8") as f:
        reader = csv.reader(f)
        for row in reader:
            if not row or row[0].strip().startswith("#"):
                continue
            snippet_path = row[0].strip()
            if not snippet_path:
                continue
            allowed_platforms = []
            for field in row[1:]:
                allowed_platforms.extend(platform for platform in re.split(r"[;\s]+", field.strip()) if platform)
            if not allowed_platforms:
                continue

            abs_path = str((root_path / snippet_path).resolve())
            constraints[abs_path] = tuple(allowed_platforms)

    return constraints


def get_platform_skip_reason(
    file_path: str | Path, current_platform: str, platform_constraints: dict[str, tuple[str, ...]]
) -> str | None:
    """Return a unittest skip reason when the snippet is not allowed on the current platform."""
    allowed_platforms = platform_constraints.get(str(Path(file_path).resolve()))
    if not allowed_platforms:
        return None
    if any(fnmatch.fnmatchcase(current_platform, allowed_platform) for allowed_platform in allowed_platforms):
        return None
    return f"Snippet is constrained to platform(s): {', '.join(allowed_platforms)}"


def parse_expected_failures_csv(csv_path, base_dir, snippets_root=None):
    """Parse expected failures CSV and return a list of (abs_path, compiled_pattern|None) tuples.

    Args:
        csv_path: Path to the CSV file (relative to base_dir or absolute).
        base_dir: Base directory for resolving the CSV file path itself.
        snippets_root: Base directory for resolving snippet paths within the CSV.
            If None, defaults to base_dir.

    Returns:
        List of (absolute_path_str, compiled_regex_or_None) tuples.
    """
    if snippets_root is None:
        snippets_root = base_dir
    entries = []
    csv_file = Path(csv_path)
    if not csv_file.is_absolute():
        csv_file = base_dir / csv_file
    if not csv_file.exists():
        print(f"Warning: Expected failures CSV file not found: {csv_file}")
        return entries

    with open(csv_file, "r", encoding="utf-8") as f:
        reader = csv.reader(f)
        for row in reader:
            if not row or row[0].strip().startswith("#"):
                continue
            snippet_path = row[0].strip()
            pattern_str = row[1].strip() if len(row) >= 2 and row[1].strip() else None
            abs_path = str((snippets_root / snippet_path).resolve())
            compiled = re.compile(pattern_str) if pattern_str else None
            entries.append((abs_path, compiled))

    return entries


def parse_excluded_snippets_csv(csv_path, base_dir, snippets_root=None):
    """Parse excluded snippets CSV and return a set of absolute paths to skip.

    These are snippets that should be completely excluded from test discovery
    (e.g. snippets that crash, hang, or kill the test process via timeouts
    that cannot be caught by expected-failure matching).

    Args:
        csv_path: Path to the CSV file (relative to base_dir or absolute).
        base_dir: Base directory for resolving the CSV file path itself.
        snippets_root: Base directory for resolving snippet paths within the CSV.
            If None, defaults to base_dir.

    Returns:
        Set of absolute path strings to exclude.
    """
    if snippets_root is None:
        snippets_root = base_dir
    excluded = set()
    csv_file = Path(csv_path)
    if not csv_file.is_absolute():
        csv_file = base_dir / csv_file
    if not csv_file.exists():
        print(f"Warning: Excluded snippets CSV file not found: {csv_file}")
        return excluded

    with open(csv_file, "r", encoding="utf-8") as f:
        reader = csv.reader(f)
        for row in reader:
            if not row or row[0].strip().startswith("#"):
                continue
            snippet_path = row[0].strip()
            if not snippet_path:
                continue
            abs_path = str((snippets_root / snippet_path).resolve())
            excluded.add(abs_path)

    return excluded


def is_expected_failure(file_path, exception, expected_failures):
    """Return True if this snippet + exception combo matches an expected-failure entry."""
    if not expected_failures:
        return False
    file_path_str = str(Path(file_path).resolve())
    exc_str = f"{type(exception).__name__}: {exception}"
    for expected_path, pattern in expected_failures:
        if file_path_str == expected_path:
            if pattern is None or pattern.search(exc_str):
                return True
    return False


def find_python_files(root_dir):
    """Find all Python files recursively in the given directory."""
    root_path = Path(root_dir)
    return list(root_path.rglob("*.py"))


def file_contains_simulation_app(file_path):
    """Check if a file contains 'SimulationApp' in uncommented lines."""
    try:
        with open(file_path, "r", encoding="utf-8") as f:
            for line in f:
                # Skip comment-only lines
                stripped = line.strip()
                if stripped.startswith("#"):
                    continue
                # Check if SimulationApp appears in the code part (before any inline comment)
                code_part = line.split("#")[0]
                if "SimulationApp" in code_part:
                    return True
        return False
    except Exception as e:
        print(f"Error reading file {file_path}: {type(e).__name__}: {e}")
        raise


def cleanup_before_new_stage(simulation_app, file_path, deadline=None):
    """Clean up the current stage before creating a new one."""
    import omni.timeline
    import omni.usd

    # Stop the timeline if it's playing
    timeline = omni.timeline.get_timeline_interface()
    if timeline.is_playing():
        timeline.stop()
        simulation_app.update()

    # Stop replicator if it's running
    try:
        import omni.replicator.core as rep

        if rep.orchestrator.get_status() not in [rep.orchestrator.Status.STOPPED, rep.orchestrator.Status.STOPPING]:
            rep.orchestrator.stop()
        cleanup_deadline = time.monotonic() + 10.0
        if deadline is not None:
            cleanup_deadline = min(cleanup_deadline, deadline)
        while rep.orchestrator.get_status() != rep.orchestrator.Status.STOPPED:
            if time.monotonic() > cleanup_deadline:
                print(f"Warning: Timed out waiting for replicator cleanup before file: {file_path}")
                break
            simulation_app.update()
        rep.orchestrator.set_capture_on_play(False)
    except Exception:
        pass

    # Run a few update frames to let everything settle
    for _ in range(5):
        simulation_app.update()

    # Close the current stage if possible; treat inability as recoverable.
    context = omni.usd.get_context()
    if context.can_close_stage():
        context.close_stage()
        simulation_app.update()
    else:
        print(f"Warning: Cannot close stage for file: {file_path}, forcing new stage")
        for _ in range(10):
            simulation_app.update()


def _is_path_within(path, root):
    """Return True if path is inside root."""
    try:
        return Path(path).resolve().is_relative_to(Path(root).resolve())
    except Exception:
        return False


def _task_belongs_to_snippets(task, snippets_root):
    """Return True if task coroutine source file is from snippets tree."""
    try:
        coro = task.get_coro()
        code = getattr(coro, "cr_code", None) or getattr(coro, "gi_code", None)
        if code is None:
            return False
        source_file = getattr(code, "co_filename", "")
        return bool(source_file) and _is_path_within(source_file, snippets_root)
    except Exception:
        return False


def _exception_belongs_to_snippets(exception, snippets_root):
    """Return True if any traceback frame for *exception* is from snippets tree."""
    try:
        tb = exception.__traceback__
        while tb is not None:
            source_file = tb.tb_frame.f_code.co_filename
            if source_file and _is_path_within(source_file, snippets_root):
                return True
            tb = tb.tb_next
    except Exception:
        return False
    return False


def _loop_context_belongs_to_snippets(context, snippets_root):
    """Return True if an asyncio loop exception context belongs to the snippet under test."""
    exception = context.get("exception")
    if exception is not None and _exception_belongs_to_snippets(exception, snippets_root):
        return True

    task = context.get("task") or context.get("future")
    if task is not None and _task_belongs_to_snippets(task, snippets_root):
        return True

    return False


def _patch_simulation_context_render_for_fabric_bootstrap():
    """Avoid cached-core Fabric updates before SimulationContext has a PhysicsContext."""
    import omni.kit.app
    from isaacsim.core.api.simulation_context import SimulationContext
    from isaacsim.core.utils.carb import set_carb_setting

    if getattr(SimulationContext, "_doc_snippets_fabric_bootstrap_patch", False):
        return

    original_render = SimulationContext.render
    original_render_async = SimulationContext.render_async

    def render(self):
        if getattr(self, "_physics_context", None) is not None:
            return original_render(self)
        set_carb_setting(self._settings, "/app/player/playSimulations", False)
        self._app.update()
        set_carb_setting(self._settings, "/app/player/playSimulations", True)
        return None

    async def render_async(self):
        if getattr(self, "_physics_context", None) is not None:
            return await original_render_async(self)
        set_carb_setting(self._settings, "/app/player/playSimulations", False)
        await omni.kit.app.get_app().next_update_async()
        set_carb_setting(self._settings, "/app/player/playSimulations", True)
        return None

    SimulationContext.render = render
    SimulationContext.render_async = render_async
    SimulationContext._doc_snippets_fabric_bootstrap_patch = True


class JUnitTestResult(unittest.TextTestResult):
    """TextTestResult subclass that records per-test timing for JUnit XML output.

    When *junit_xml_path* is set, the report is flushed to disk after every test
    so that a partial report survives even if the process is killed mid-run.
    """

    def __init__(self, stream, descriptions, verbosity, junit_xml_path=None):
        super().__init__(stream, descriptions, verbosity)
        self.test_timings = []
        self._test_start = 0.0
        self._junit_xml_path = junit_xml_path

    def _flush_report(self):
        """Write the current (possibly partial) JUnit XML to disk."""
        if self._junit_xml_path:
            try:
                self.write_junit_xml(self._junit_xml_path)
            except Exception:
                pass

    def startTest(self, test):
        super().startTest(test)
        self._test_start = time.monotonic()

    def addSuccess(self, test):
        super().addSuccess(test)
        self.test_timings.append((test, "pass", time.monotonic() - self._test_start, None))
        self._flush_report()

    def addFailure(self, test, err):
        super().addFailure(test, err)
        msg = self._exc_info_to_string(err, test)
        self.test_timings.append((test, "fail", time.monotonic() - self._test_start, msg))
        self._flush_report()

    def addError(self, test, err):
        super().addError(test, err)
        msg = self._exc_info_to_string(err, test)
        self.test_timings.append((test, "error", time.monotonic() - self._test_start, msg))
        self._flush_report()

    def write_junit_xml(self, output_path):
        """Write a JUnit XML report with one <testcase> per snippet."""
        failures = sum(1 for _, s, _, _ in self.test_timings if s == "fail")
        errors_count = sum(1 for _, s, _, _ in self.test_timings if s == "error")
        total_time = sum(e for _, _, e, _ in self.test_timings)

        testsuites = ET.Element("testsuites")
        testsuite = ET.SubElement(
            testsuites,
            "testsuite",
            name="doc_snippets_async",
            tests=str(len(self.test_timings)),
            failures=str(failures),
            errors=str(errors_count),
            time=f"{total_time:.3f}",
            timestamp=datetime.now().isoformat(),
        )

        for test, status, elapsed, msg in self.test_timings:
            name = test.shortDescription() or str(test)
            tc = ET.SubElement(
                testsuite,
                "testcase",
                name=name,
                classname=test.__class__.__name__,
                time=f"{elapsed:.3f}",
            )
            if status == "fail":
                failure = ET.SubElement(tc, "failure", message=f"{name} failed")
                failure.text = _sanitize_xml(msg) if msg else f"{name} failed"
            elif status == "error":
                error_el = ET.SubElement(tc, "error", message=f"{name} error")
                error_el.text = _sanitize_xml(msg) if msg else f"{name} error"

        ET.indent(testsuites, space="  ", level=0)
        tree = ET.ElementTree(testsuites)
        Path(output_path).parent.mkdir(parents=True, exist_ok=True)
        tree.write(output_path, encoding="utf-8", xml_declaration=True)
        print(f"JUnit XML report written to {output_path}")


def _sanitize_xml(text):
    """Remove control characters that are invalid in XML 1.0."""
    if not text:
        return text
    return "".join(ch if (ord(ch) >= 0x20 or ch in "\t\n\r") else "" for ch in text)


class SnippetTimeoutError(Exception):
    """Raised when a snippet exceeds its per-snippet time limit."""


# How long to wait after the initial SIGALRM before force-exiting.  This gives
# the Python-level exception a chance to propagate when possible; if the process
# is stuck inside native code that swallowed the exception, the escalation
# handler terminates the process so CI isn't left waiting for the outer timeout.
_ALARM_ESCALATION_SECONDS = 30


def _force_exit_alarm_handler(signum, frame):
    """Last-resort SIGALRM handler: force-exit when a snippet is stuck in native code."""
    print(
        f"\n[FATAL] Snippet still stuck {_ALARM_ESCALATION_SECONDS}s after timeout. "
        "Partial JUnit report (if any) has been written to disk. Forcing exit.",
        flush=True,
    )
    os._exit(1)


def _wait_for_snippet_tasks(simulation_app, tasks, settle_frames=10, deadline=None):
    """Give snippet-created async tasks a chance to complete."""
    if not tasks:
        return
    for _ in range(settle_frames):
        if all(task.done() for task in tasks):
            break
        if deadline is not None and time.monotonic() > deadline:
            break
        simulation_app.update()


def load_snippet_module(file_path, snippets_root, index, simulation_app, snippet_timeout=120):
    """Load a snippet module and return any exception that occurred.

    Returns:
        Tuple of (file_path_str, exception_or_None, elapsed_seconds).
    """
    import gc

    import isaacsim.core.utils.stage as stage_utils

    unique_module_name = f"_snippet_test_{index}"
    exceptions = []
    module = None
    loop = None
    baseline_tasks = set()
    captured_loop_exceptions = []
    # Snapshot of modules before loading
    modules_before = set(sys.modules.keys())
    start_time = time.monotonic()
    deadline = start_time + snippet_timeout

    def loop_exception_handler(loop, context):
        """Capture unhandled loop exceptions as test failures."""
        captured_loop_exceptions.append(context)

    def _check_deadline(phase):
        """Raise SnippetTimeoutError if the per-snippet deadline has been exceeded."""
        if time.monotonic() > deadline:
            raise SnippetTimeoutError(f"Snippet timed out after {snippet_timeout}s during {phase}: {file_path}")

    # Use SIGALRM as a hard backstop to interrupt blocking C/C++ calls that
    # cannot be interrupted by a Python-level deadline check.
    prev_alarm_handler = None
    prev_alarm_remaining = 0

    def _alarm_handler(signum, frame):
        # Install a second-chance handler: if this raise gets swallowed by
        # native code (e.g. a C callback catches the Python exception at the
        # boundary), the escalation alarm will force-exit the process so CI
        # doesn't hang until the outer timeout.
        signal.signal(signal.SIGALRM, _force_exit_alarm_handler)
        signal.alarm(_ALARM_ESCALATION_SECONDS)
        raise SnippetTimeoutError(f"Snippet timed out after {snippet_timeout}s (SIGALRM): {file_path}")

    if hasattr(signal, "SIGALRM"):
        prev_alarm_handler = signal.signal(signal.SIGALRM, _alarm_handler)
        # Add a few extra seconds so the soft deadline check fires first when possible.
        prev_alarm_remaining = signal.alarm(snippet_timeout + 5)

    try:
        loop = asyncio.get_event_loop()
        baseline_tasks = set(asyncio.all_tasks(loop))
        previous_exception_handler = loop.get_exception_handler()
        loop.set_exception_handler(loop_exception_handler)

        # Clean up the current stage before creating a new one
        cleanup_before_new_stage(simulation_app, file_path, deadline=deadline)
        _check_deadline("cleanup")

        # Open a new stage and wait for it to finish loading
        stage_utils.create_new_stage()
        simulation_app.update()
        while stage_utils.is_stage_loading():
            _check_deadline("stage_loading")
            simulation_app.update()

        # Add the snippets root to sys.path if not already there
        snippets_root_str = str(snippets_root)
        if snippets_root_str not in sys.path:
            sys.path.insert(0, snippets_root_str)

        # Load the module with a unique name
        spec = importlib.util.spec_from_file_location(unique_module_name, file_path)
        if spec is None or spec.loader is None:
            raise ImportError(f"Could not create spec for module {unique_module_name}")

        module = importlib.util.module_from_spec(spec)
        # Execute the module
        spec.loader.exec_module(module)
        _check_deadline("exec_module")

    except SystemExit as e:
        # Snippets must not call sys.exit(); treat as a test failure.
        exceptions.append(RuntimeError(f"Snippet called sys.exit({e.code!r}). Snippets must not call sys.exit()."))
    except KeyboardInterrupt:
        # Don't let a stray KeyboardInterrupt from a snippet kill the whole harness.
        exceptions.append(RuntimeError("KeyboardInterrupt raised during snippet execution."))
    except SnippetTimeoutError as e:
        exceptions.append(e)
    except Exception as e:
        exceptions.append(e)
    finally:
        snippet_tasks = []
        if loop is not None:
            current_tasks = set(asyncio.all_tasks(loop))
            snippet_tasks = [
                task for task in (current_tasks - baseline_tasks) if _task_belongs_to_snippets(task, snippets_root)
            ]

            timed_out = time.monotonic() > deadline

            if not timed_out:
                # Let snippet-created tasks finish naturally first.
                _wait_for_snippet_tasks(simulation_app, snippet_tasks, settle_frames=30, deadline=deadline)

            # Cancel still-pending snippet tasks so they do not leak across snippets.
            pending_snippet_tasks = [task for task in snippet_tasks if not task.done()]
            for task in pending_snippet_tasks:
                task.cancel()
            _wait_for_snippet_tasks(simulation_app, pending_snippet_tasks, settle_frames=5, deadline=deadline)

            # Retrieve task exceptions explicitly so they become deterministic test failures.
            for task in snippet_tasks:
                if not task.done() or task.cancelled():
                    continue
                try:
                    task_exception = task.exception()
                except asyncio.CancelledError:
                    continue
                except Exception as task_exception:
                    exceptions.append(task_exception)
                    continue
                if task_exception is not None:
                    exceptions.append(task_exception)

            # Restore the loop exception handler.
            try:
                loop.set_exception_handler(previous_exception_handler)
            except Exception:
                pass

        # Disarm the SIGALRM backstop.
        if hasattr(signal, "SIGALRM"):
            signal.alarm(0)
            if prev_alarm_handler is not None:
                signal.signal(signal.SIGALRM, prev_alarm_handler)
            if prev_alarm_remaining > 0:
                signal.alarm(prev_alarm_remaining)

        # Promote unhandled loop-level async exceptions to snippet failures.
        # Kit can emit unrelated loop-level exceptions during stage churn; keep
        # the failure attribution scoped to the snippet under test.
        for context in captured_loop_exceptions:
            if not _loop_context_belongs_to_snippets(context, snippets_root):
                continue
            loop_exception = context.get("exception")
            if loop_exception is not None:
                exceptions.append(loop_exception)
            else:
                message = context.get("message", "Unhandled asyncio loop exception.")
                exceptions.append(RuntimeError(message))

        # Some modules monkey-patch global runtime state (e.g. asyncio loop internals).
        # Unloading them can leave patched callables referencing cleared module globals.
        module_cleanup_exclude = {"nest_asyncio"}

        # Unload only snippet-local modules. Do not wipe module dicts to avoid breaking
        # async tasks/callbacks that still reference module globals.
        modules_after = set(sys.modules.keys())
        new_modules = modules_after - modules_before
        for mod_name in new_modules:
            if mod_name in module_cleanup_exclude:
                continue
            module_obj = sys.modules.get(mod_name)
            module_file = getattr(module_obj, "__file__", None) if module_obj is not None else None
            if module_file is None or not _is_path_within(module_file, snippets_root):
                continue
            sys.modules.pop(mod_name, None)

        # Force garbage collection to clean up unreferenced objects
        gc.collect()

    elapsed = time.monotonic() - start_time
    if not exceptions:
        return (str(file_path), None, elapsed)
    if len(exceptions) == 1:
        return (str(file_path), exceptions[0], elapsed)
    return (
        str(file_path),
        ExceptionGroup(f"Multiple exceptions while testing snippet {file_path}", exceptions),
        elapsed,
    )


# Parse command line arguments
args, _ = parse_args()

# Get the snippets directory (canonical location: docs/isaacsim/snippets)
# Walk upward from the script location to find the repo root, so the script
# works regardless of whether it is invoked from source or from a build tree.
script_dir = Path(__file__).resolve().parent
snippets_rel = Path("docs") / "isaacsim" / "snippets"

repo_root = None
for _candidate in [script_dir, *script_dir.parents]:
    if (_candidate / snippets_rel).is_dir():
        repo_root = _candidate
        break

if repo_root is None:
    print(f"Error: Could not locate {snippets_rel} in any ancestor of {script_dir}")
    sys.exit(1)

snippets_dir = repo_root / snippets_rel

# Find all Python files
print(f"Scanning for Python files in {snippets_dir}...")
python_files = find_python_files(snippets_dir)
print(f"Found {len(python_files)} Python files")

# Filter files that don't contain SimulationApp and exclude __init__.py files
files_to_test = []
for file_path in python_files:
    if file_path.name == "__init__.py":
        continue
    if not file_contains_simulation_app(file_path):
        files_to_test.append(file_path)

print(f"Found {len(files_to_test)} files to test")

# Apply directory filter if specified
if args.filter:
    files_to_test = [f for f in files_to_test if any(keyword in str(f) for keyword in args.filter)]
    print(f"After applying filter {args.filter}: {len(files_to_test)} files to test")

# Exclude snippets that crash/hang the test process
excluded_snippets = set()
if args.excluded_snippets_csv:
    excluded_snippets = parse_excluded_snippets_csv(args.excluded_snippets_csv, script_dir, snippets_dir)
    if excluded_snippets:
        before_count = len(files_to_test)
        files_to_test = [f for f in files_to_test if str(f.resolve()) not in excluded_snippets]
        skipped = before_count - len(files_to_test)
        print(f"Excluded {skipped} snippet(s) via {args.excluded_snippets_csv} ({len(files_to_test)} remaining)")

# Parse experience CSV and group files by experience
experience_map = {}
if args.experience_csv:
    experience_map = parse_experience_csv(args.experience_csv, script_dir, snippets_dir)
    print(f"Loaded {len(experience_map)} experience mappings from CSV")

files_by_experience = group_files_by_experience(files_to_test, experience_map)
experience_names = sorted(files_by_experience.keys(), key=lambda x: (x != "", x))  # Default experience first
print(f"Files grouped into {len(experience_names)} experience group(s): {experience_names}")

# Parse expected failures
expected_failures = []
if args.expected_failures_csv:
    expected_failures = parse_expected_failures_csv(args.expected_failures_csv, script_dir, snippets_dir)
    print(f"Loaded {len(expected_failures)} expected failure rules from CSV")

# Parse platform constraints
current_platform = get_current_platform_name()
platform_constraints = {}
if args.platform_constraints_csv:
    platform_constraints = parse_platform_constraints_csv(args.platform_constraints_csv, script_dir, snippets_dir)
    print(
        f"Loaded {len(platform_constraints)} platform constraint rule(s) from CSV "
        f"for current platform: {current_platform}"
    )

# ---------------------------------------------------------------------------
# Dynamic unittest generation -- one test method per snippet file
# ---------------------------------------------------------------------------

_total_snippets = len(files_to_test)


def is_in_expected_failures(file_path, expected_failures):
    """Return True if this snippet path appears in the expected-failure list (regardless of pattern)."""
    if not expected_failures:
        return False
    file_path_str = str(Path(file_path).resolve())
    for expected_path, _pattern in expected_failures:
        if file_path_str == expected_path:
            return True
    return False


def _make_snippet_test(
    file_path,
    snippets_root,
    snippet_index,
    total_count,
    expected_failures_list,
    snippet_timeout,
    platform_constraints_map,
    current_platform_name,
):
    """Create a test method for a single doc snippet."""

    def test_snippet(self):
        platform_skip_reason = get_platform_skip_reason(file_path, current_platform_name, platform_constraints_map)
        if platform_skip_reason:
            self.skipTest(platform_skip_reason)

        result_path, exception, elapsed = load_snippet_module(
            file_path, snippets_root, snippet_index, self.__class__._simulation_app, snippet_timeout=snippet_timeout
        )
        print(f" [{elapsed:.1f}s]", end="", flush=True)
        if exception is None:
            if is_in_expected_failures(result_path, expected_failures_list):
                self.fail(
                    f"Snippet was expected to fail but passed. "
                    f"Remove it from the expected-failures CSV: {result_path}"
                )
            return
        if is_expected_failure(result_path, exception, expected_failures_list):
            return
        if hasattr(exception, "__traceback__") and exception.__traceback__:
            msg = "".join(traceback.format_exception(type(exception), exception, exception.__traceback__))
        else:
            msg = f"{type(exception).__name__}: {exception}"
        self.fail(msg)

    return test_snippet


_simulation_app = None
_test_classes = []
_snippet_counter = 0

for _exp_idx, _experience in enumerate(experience_names):
    _group = files_by_experience[_experience]
    _exp_display = _experience if _experience else "(default)"
    _safe_exp = re.sub(r"[^a-zA-Z0-9]", "_", _exp_display)
    _class_name = f"TestSnippets_{_safe_exp}"

    _base_name = _class_name
    _dedup = 2
    while _class_name in globals():
        _class_name = f"{_base_name}_{_dedup}"
        _dedup += 1

    def _make_class_methods(exp_value, group_files):
        @classmethod
        def setUpClass(cls):
            global _simulation_app
            if _simulation_app is None:
                from isaacsim import SimulationApp

                launch_config = {"headless": True}
                experience_path = resolve_experience_path(exp_value)
                if experience_path:
                    print(f"Launching SimulationApp with experience: {experience_path}")
                _simulation_app = SimulationApp(launch_config=launch_config, experience=experience_path)

                # Override asset root if requested (e.g. when Nucleus is unreachable)
                if args.asset_root:
                    import carb

                    carb.settings.get_settings().set("/persistent/isaac/asset_root/default", args.asset_root)
                    print(f"Asset root overridden to: {args.asset_root}")

                _patch_simulation_context_render_for_fabric_bootstrap()

            cls._simulation_app = _simulation_app

            exp_disp = exp_value if exp_value else "(default)"
            print(f"\n{'=' * 80}")
            print(f"Processing experience group: {exp_disp} ({len(group_files)} files)")
            print("=" * 80)

        @classmethod
        def tearDownClass(cls):
            pass

        return setUpClass, tearDownClass

    _setup, _teardown = _make_class_methods(_experience, _group)

    _TestClass = type(
        _class_name,
        (unittest.TestCase,),
        {
            "maxDiff": None,
            "setUpClass": _setup,
            "tearDownClass": _teardown,
        },
    )

    for _file_path in _group:
        _rel = _file_path.relative_to(snippets_dir)
        _safe_name = re.sub(r"[^a-zA-Z0-9]", "_", str(_rel))
        _test_name = f"test_{_snippet_counter:04d}_{_safe_name}"

        _func = _make_snippet_test(
            _file_path,
            snippets_dir,
            _snippet_counter,
            _total_snippets,
            expected_failures,
            args.snippet_timeout,
            platform_constraints,
            current_platform,
        )
        _func.__name__ = _test_name
        _func.__doc__ = str(_rel)
        setattr(_TestClass, _test_name, _func)
        _snippet_counter += 1

    globals()[_class_name] = _TestClass
    _test_classes.append(_TestClass)

# Build and run the test suite in deterministic order
_suite = unittest.TestSuite()
for _cls in _test_classes:
    _suite.addTests(unittest.TestLoader().loadTestsFromTestCase(_cls))

_test_count = _suite.countTestCases()
print(f"\n{'=' * 40}")
print(f"Running Tests (count: {_test_count}):")
print("=" * 40)

_ResultClass = partial(JUnitTestResult, junit_xml_path=args.junit_xml) if args.junit_xml else JUnitTestResult
_runner = unittest.TextTestRunner(stream=sys.stdout, verbosity=2, resultclass=_ResultClass)
_result = _runner.run(_suite)

# SimulationApp.close() may terminate the process (e.g. fastShutdown), so register
# the summary as an atexit handler to guarantee it prints regardless.
_summary_printed = False


def _print_summary():
    global _summary_printed
    if _summary_printed:
        return
    _summary_printed = True
    print("=" * 40)
    if _result.wasSuccessful():
        print("[ ok ] Test passed.")
    else:
        _fail_count = len(_result.failures) + len(_result.errors)
        print(f"[ FAIL ] Test failed. ({_fail_count} failure(s))")
        if _result.failures:
            print(f"\n{'=' * 40}")
            print("Failed tests:")
            print("=" * 40)
            for _test, _tb in _result.failures:
                print(f"  FAIL: {_test}")
        if _result.errors:
            print(f"\n{'=' * 40}")
            print("Error tests:")
            print("=" * 40)
            for _test, _tb in _result.errors:
                print(f"  ERROR: {_test}")
    sys.stdout.flush()


atexit.register(_print_summary)
_print_summary()

# Close the single SimulationApp after the summary has been printed.
if _simulation_app is not None:
    _simulation_app.close()

if not _result.wasSuccessful():
    sys.exit(1)
