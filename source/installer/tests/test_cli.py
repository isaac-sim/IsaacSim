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

"""Check source detection, explicit checkout selection, and optional TUI dispatch."""

from __future__ import annotations

import sys
from pathlib import Path

import pytest
from isaacsim_installer.cli import main as cli
from isaacsim_installer.core.backend import find_checkout_root
from isaacsim_installer.tui.app import ExamplesApp


def make_checkout(root: Path) -> Path:
    """Create the source layout used to detect an editable installation."""
    package = root / "source/installer"
    package.mkdir(parents=True)
    runner = root / "source/examples/examples.py"
    runner.parent.mkdir(parents=True)
    runner.touch()
    return package


def test_source_detection_does_not_treat_a_wheel_inside_checkout_as_editable(tmp_path: Path) -> None:
    """Only the package's authored location establishes checkout ownership."""
    package = make_checkout(tmp_path)
    assert find_checkout_root(package) == tmp_path
    assert find_checkout_root(tmp_path / ".venv/lib/python3.12/site-packages/isaacsim_installer") is None
    (tmp_path / "source/examples/examples.py").unlink()
    assert find_checkout_root(package) is None


@pytest.mark.parametrize("explicit", [False, True])
def test_tui_uses_detected_checkout_unless_explicitly_overridden(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch, explicit: bool
) -> None:
    """Dispatch the UI using its package checkout or the caller's override, never cwd."""
    detected = tmp_path / "detected"
    package = make_checkout(detected)
    override = tmp_path / "override"
    make_checkout(override)
    monkeypatch.setattr(cli, "__file__", str(package / "cli/main.py"))
    monkeypatch.chdir(tmp_path)
    monkeypatch.setattr(sys.stdin, "isatty", lambda: True)
    monkeypatch.setattr(sys.stdout, "isatty", lambda: True)
    launched = []
    monkeypatch.setattr(ExamplesApp, "run", lambda self: launched.append(self.backend.root))
    args = ["tui", "--source-root", str(override)] if explicit else ["tui"]
    assert cli.main(args) == 0
    assert launched == [override if explicit else detected]


@pytest.mark.parametrize(
    "arguments",
    [
        ["run", "--no-build", "hello_world.c", "--", "--config", "payload", "--source-root", "literal", "--help"],
        ["run", "hello_world.c", "--config", "payload"],
    ],
)
def test_local_commands_preserve_runner_arguments_and_exit_status(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch, arguments: list[str]
) -> None:
    """Dispatch without optional dependencies, preserving runner and example argument boundaries."""
    package = make_checkout(tmp_path)
    monkeypatch.setattr(cli, "__file__", str(package / "cli/main.py"))
    monkeypatch.setattr(cli.importlib.util, "find_spec", lambda name: None)
    calls = []

    def execute(root: Path, selected: str, args: list[str], config: str) -> int:
        calls.append((root, selected, args, config))
        return 17

    monkeypatch.setattr(cli, "execute_terminal", execute)
    assert cli.main([arguments[0], "--config", "Debug", *arguments[1:]]) == 17
    assert calls == [(tmp_path, arguments[0], arguments[1:], "Debug")]


def test_missing_checkout_reports_selected_path(tmp_path: Path, capsys: pytest.CaptureFixture[str]) -> None:
    """Give an actionable CLI error when the selected checkout has no driver."""
    with pytest.raises(SystemExit, match="2"):
        cli.main(["list", "--source-root", str(tmp_path)])
    error = capsys.readouterr().err
    assert str(tmp_path / "tools/pixi_run.py") in error
    assert "Select an Isaac Sim checkout with --source-root" in error
