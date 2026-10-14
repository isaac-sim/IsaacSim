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

"""Tests that the headless SysID tools hand their failure status to ``SimulationApp.close``."""

from __future__ import annotations

import contextlib
import importlib.util
import inspect
import io
import sys
from pathlib import Path
from types import ModuleType, SimpleNamespace
from unittest.mock import patch

import omni.kit.test

_TOOLS_DIR = Path(__file__).resolve().parents[4] / "tools"

# Each tool with the minimum argv its parser requires.
_TOOL_ARGV = {
    "headless_sysid_solve": ["--run-spec", "spec.json"],
    "headless_sysid_analyze": ["--run-spec", "spec.json"],
    "headless_sysid_apply_theta_to_stage": [
        "--run-spec",
        "spec.json",
        "--result-json",
        "result.json",
        "--output-stage",
        "out.usda",
    ],
    "headless_sysid_simsim": ["--run-spec", "spec.json", "--output-dir", "out"],
    "headless_sysid_shift_commands": [
        "--run-spec",
        "spec.json",
        "--output-csv",
        "out.csv",
        "--command-delay",
        "0.0",
    ],
}


def _load_tool(name: str) -> ModuleType:
    path = _TOOLS_DIR / f"{name}.py"
    spec = importlib.util.spec_from_file_location(f"_{name}_exit_code_test", path)
    if spec is None or spec.loader is None:
        raise RuntimeError(f"Could not load tool module: {path}")
    module = importlib.util.module_from_spec(spec)
    sys.path.insert(0, str(_TOOLS_DIR))
    try:
        spec.loader.exec_module(module)
    finally:
        sys.path.pop(0)
    return module


class _FailedTask:
    def done(self) -> bool:
        return True

    def result(self) -> None:
        raise RuntimeError("Simulated headless SysID failure.")


class _ImmediateLoop:
    """Event loop stand-in whose tasks have already failed.

    The Kit test runner owns the running loop, so the tool's update pump cannot
    drive a real task to completion from inside a test.
    """

    def create_task(self, coro: object) -> _FailedTask:
        if inspect.iscoroutine(coro):
            coro.close()
        return _FailedTask()

    def is_running(self) -> bool:
        return False

    def run_until_complete(self, coro: object) -> None:
        if inspect.iscoroutine(coro):
            coro.close()


class _FakeSimulationApp:
    instances: list[_FakeSimulationApp] = []

    def __init__(self, *_args: object, **_kwargs: object) -> None:
        self.close_calls: list[dict[str, object]] = []
        _FakeSimulationApp.instances.append(self)

    def update(self) -> None:
        pass

    def close(self, **kwargs: object) -> None:
        self.close_calls.append(kwargs)


class TestHeadlessToolExitCodes(omni.kit.test.AsyncTestCase):
    """A failed headless run must not leave the process exiting with status 0."""

    def test_failed_run_passes_nonzero_exit_code_to_close(self) -> None:
        """Every tool forwards its failure status to ``SimulationApp.close``.

        Under the default fast shutdown, ``close()`` terminates the process with
        its ``exit_code`` argument, so a ``return 1`` after ``close()`` never
        reaches the shell.
        """
        fake_isaacsim = SimpleNamespace(SimulationApp=_FakeSimulationApp)
        fake_asyncio = SimpleNamespace(get_event_loop=_ImmediateLoop)
        for name, argv in _TOOL_ARGV.items():
            with self.subTest(tool=name):
                tool = _load_tool(name)
                _FakeSimulationApp.instances.clear()
                # Kit fails the test run on stderr output, and the tools report failures there.
                stderr = io.StringIO()
                with (
                    patch.dict(sys.modules, {"isaacsim": fake_isaacsim}),
                    patch.object(sys, "argv", [name, *argv]),
                    patch.object(tool, "asyncio", fake_asyncio),
                    contextlib.redirect_stderr(stderr),
                ):
                    code = tool.main()
                self.assertIn("Simulated headless SysID failure.", stderr.getvalue())
                self.assertEqual(code, 1)
                self.assertEqual(len(_FakeSimulationApp.instances), 1)
                self.assertEqual(_FakeSimulationApp.instances[0].close_calls, [{"exit_code": 1}])
