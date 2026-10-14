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

"""Check real subprocess discovery, argument transport, and cancellation."""

from __future__ import annotations

import asyncio
import json
import os
import sys
from pathlib import Path

import pytest
from isaacsim_installer.core.backend import BackendError, LocalBackend, _decode_catalog

ROOT = Path(__file__).resolve().parents[3]


def create_runner(root: Path, script: str) -> LocalBackend:
    """Create a checkout-shaped subprocess fixture."""
    path = root / "source/examples/examples.py"
    path.parent.mkdir(parents=True)
    path.write_text(script, encoding="utf-8")
    return LocalBackend(root)


async def test_discovery_reports_old_runner_and_bad_schema(tmp_path: Path) -> None:
    """Give an actionable error for older checkouts and unsupported wire versions."""
    backend = create_runner(tmp_path, 'print("ID | TITLE")\n')
    with pytest.raises(BackendError, match="Update the selected checkout"):
        await backend.discover()
    with pytest.raises(BackendError, match="Unsupported"):
        _decode_catalog({"schema_version": 99})


async def test_discovery_timeout_stops_runner(tmp_path: Path, monkeypatch: pytest.MonkeyPatch) -> None:
    """Bound discovery time and report which runner operation stalled."""
    backend = create_runner(tmp_path, "import time\ntime.sleep(60)\n")
    process: asyncio.subprocess.Process | None = None
    start = backend._start

    async def capture_process(command: list[str]) -> asyncio.subprocess.Process:
        nonlocal process
        process = await start(command)
        return process

    monkeypatch.setattr(backend, "_start", capture_process)
    monkeypatch.setattr("isaacsim_installer.core.backend._JSON_TIMEOUT_SECONDS", 0.05)
    with pytest.raises(BackendError, match="Examples list timed out"):
        await backend.discover()
    assert process is not None and process.returncode is not None


async def test_streamed_run_preserves_argv_and_exit_status(tmp_path: Path, monkeypatch: pytest.MonkeyPatch) -> None:
    """Round-trip spaces, option-looking strings, and empty arguments through Python."""
    monkeypatch.setenv("PYTHONIOENCODING", "ascii")
    backend = create_runner(
        tmp_path,
        "import json, os, sys\nfrom pathlib import Path\n"
        'Path("run.json").write_text(json.dumps({"args": sys.argv[1:], "dont_write_bytecode": '
        'os.environ.get("PYTHONDONTWRITEBYTECODE")}))\nprint("[red]café ✓[/red]", flush=True)\nsys.exit(6)\n',
    )
    args = ("--name", "two words", "", "$(literal)")
    output: list[str] = []
    assert await backend.run("demo", args, output.append) == 6
    assert "[red]café ✓[/red]" in "".join(output)
    run = json.loads((tmp_path / "run.json").read_text())
    assert run["args"] == [
        "--config",
        "Release",
        "run",
        "--dev",
        "--no-build",
        "demo",
        "--",
        *args,
    ]
    assert run["dont_write_bytecode"] == "1"


async def test_cancel_reaps_child_process(tmp_path: Path) -> None:
    """Cancel after a child starts and ensure it cannot continue writing."""
    backend = create_runner(
        tmp_path,
        """import subprocess, sys, time
child = subprocess.Popen(
    [sys.executable, "-c", "import time; from pathlib import Path; time.sleep(1); Path('leaked').touch()"]
)
print("ready", flush=True)
try:
    time.sleep(60)
finally:
    child.wait()
""",
    )
    ready = asyncio.Event()
    emit = lambda text: ready.set() if "ready" in text else None
    task = asyncio.create_task(backend.run("demo", (), emit))
    await asyncio.wait_for(ready.wait(), timeout=5)
    task.cancel()
    with pytest.raises(asyncio.CancelledError):
        await task
    await asyncio.sleep(1.1)
    assert not (tmp_path / "leaked").exists()


@pytest.mark.skipif(os.name == "nt", reason="POSIX runner interrupt probe")
async def test_cancel_drains_noisy_shutdown(tmp_path: Path, monkeypatch: pytest.MonkeyPatch) -> None:
    """Finish cancellation even when graceful shutdown exceeds the pipe capacity."""
    backend = create_runner(
        tmp_path,
        """import signal, sys, time
from pathlib import Path
def shutdown(*args):
    sys.stdout.write("x" * 1_000_000)
    sys.stdout.flush()
    Path("shutdown-finished").touch()
    sys.exit(0)
signal.signal(signal.SIGINT, shutdown)
print("ready", flush=True)
time.sleep(60)
""",
    )
    process: asyncio.subprocess.Process | None = None
    start = backend._start

    async def capture_process(command: list[str]) -> asyncio.subprocess.Process:
        nonlocal process
        process = await start(command)
        return process

    monkeypatch.setattr(backend, "_start", capture_process)
    ready = asyncio.Event()
    task = asyncio.create_task(backend.run("demo", (), lambda text: ready.set()))
    try:
        await asyncio.wait_for(ready.wait(), timeout=5)
        task.cancel()
        done, _ = await asyncio.wait({task}, timeout=5)
        assert done, "Cancellation blocked on shutdown output"
        with pytest.raises(asyncio.CancelledError):
            await task
        assert process is not None and process.returncode == 0
        assert (tmp_path / "shutdown-finished").exists()
    finally:
        if not task.done():
            # Reap a broken implementation without leaving the regression hung.
            if process is not None:
                if process.returncode is None:
                    process.kill()
                await process.communicate()
            task.cancel()
            await asyncio.gather(task, return_exceptions=True)


@pytest.mark.skipif(os.name == "nt", reason="POSIX runner interrupt probe")
async def test_cancel_reaps_runner_test_in_separate_session(tmp_path: Path) -> None:
    """Interrupt the real test-capture helper, whose child owns another session."""
    runner_path = ROOT / "source/examples/examples.py"
    script = f"""import importlib.util, os, sys
sys.dont_write_bytecode = True
spec = importlib.util.spec_from_file_location('runner', {str(runner_path)!r})
runner = importlib.util.module_from_spec(spec)
sys.modules['runner'] = runner
spec.loader.exec_module(runner)
runner._run_captured(
    [sys.executable, '-c',
     "from pathlib import Path; import os, time; Path('child.pid').write_text(str(os.getpid())); "
     "Path('ready').touch(); time.sleep(2); Path('leaked').touch()"],
    runner.Path.cwd(), os.environ.copy(), 60,
)
"""
    backend = create_runner(tmp_path, script)
    task = asyncio.create_task(backend.execute("test", "demo", lambda text: None))

    async def wait_for_child() -> None:
        while not (tmp_path / "ready").exists():
            await asyncio.sleep(0.02)

    try:
        await asyncio.wait_for(wait_for_child(), timeout=5)
    finally:
        task.cancel()
        with pytest.raises(asyncio.CancelledError):
            await task
    child_pid = int((tmp_path / "child.pid").read_text())
    with pytest.raises(ProcessLookupError):
        os.kill(child_pid, 0)
    assert not (tmp_path / "leaked").exists()


async def test_missing_developer_build_has_actionable_readiness(tmp_path: Path) -> None:
    """A checkout without a configured SDK reports setup before execution."""
    import shutil

    runner = tmp_path / "source/examples/examples.py"
    runner.parent.mkdir(parents=True)
    shutil.copyfile(ROOT / "source/examples/examples.py", runner)
    readiness = await LocalBackend(tmp_path).check_ready()
    assert not readiness.ready
    assert "Release" in readiness.message
    assert ("build.bat -r" if os.name == "nt" else "build.sh -r") in readiness.instructions
    assert str(tmp_path) in readiness.instructions


@pytest.mark.parametrize("config,flag", [("Release", "-r"), ("Debug", "-d")])
async def test_prepare_uses_repository_build_driver(tmp_path: Path, config: str, flag: str) -> None:
    """Forward SDK configuration to the standard build driver and stream its output."""
    backend = create_runner(tmp_path, "")
    backend.config = config
    driver = tmp_path / "tools/pixi_run.py"
    driver.parent.mkdir()
    driver.write_text(
        "import sys, json; from pathlib import Path; "
        "Path('build-args.json').write_text(json.dumps(sys.argv[1:])); print('SDK build output'); sys.exit(4)"
    )
    output: list[str] = []
    assert await backend.prepare(output.append) == 4
    assert json.loads((tmp_path / "build-args.json").read_text()) == ["libraries", "--", flag]
    assert "SDK build output" in "".join(output)


@pytest.mark.parametrize("use_relative_path", [True, False], ids=["relative-path", "path-command"])
async def test_interpreter_resolution_is_independent_of_checkout(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch, use_relative_path: bool
) -> None:
    """Resolve interpreter paths from the caller while retaining PATH command lookup."""
    checkout = tmp_path / "checkout"
    create_runner(checkout, "import json; print(json.dumps(dict(schema_version=1, examples=[], series=[])))")
    monkeypatch.chdir(tmp_path)
    monkeypatch.setenv("PATH", str(Path(sys.executable).parent) + os.pathsep + os.environ.get("PATH", ""))
    interpreter = os.path.relpath(sys.executable, tmp_path) if use_relative_path else Path(sys.executable).name
    catalog = await LocalBackend(checkout, python=interpreter).discover()
    assert catalog.examples == () and catalog.series == ()
