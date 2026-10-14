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

"""Backend contract and local checkout adapter using the examples runner CLI."""

from __future__ import annotations

import asyncio
import codecs
import contextlib
import json
import os
import signal
import subprocess
import sys
from collections.abc import Callable, Sequence
from pathlib import Path
from typing import Protocol

from isaacsim_installer.core.models import Catalog, Example, ModuleRequirement, Readiness, Series, SeriesStep

_CONFIGURATIONS = frozenset({"Debug", "Release"})
_JSON_TIMEOUT_SECONDS = 30
_PROCESS_SHUTDOWN_SECONDS = 12


class BackendError(RuntimeError):
    """An actionable discovery or execution failure."""


def find_checkout_root(package_directory: Path) -> Path | None:
    """Detect checkout ownership only for a source or editable installation."""
    root = package_directory.parent.parent
    if root / "source/installer" == package_directory and (root / "source/examples/examples.py").is_file():
        return root
    return None


def execute_terminal(root: Path, action: str, arguments: Sequence[str] = (), config: str = "Release") -> int:
    """Run the checkout's examples workflow with the caller's terminal streams.

    The Python entry point is the dispatcher used by `examples.sh` and
    `examples.bat`, avoiding batch-shell interpretation of literal arguments.
    """
    driver = root / "tools/pixi_run.py"
    if not driver.is_file():
        raise BackendError(f"No examples driver at {driver}. Select an Isaac Sim checkout with --source-root.")
    command = [sys.executable, "-B", "-u", str(driver), "examples", "--", "--config", config, action, *arguments]
    try:
        return subprocess.call(command, cwd=root)
    except OSError as error:
        raise BackendError(f"Cannot start examples runner: {error}") from error


class ExamplesBackend(Protocol):
    """UI-facing operations; adapters own discovery, execution, and cleanup.

    Cancellation of an async operation must reap its subprocesses before returning.
    Output callbacks run on the calling event loop and receive bounded literal text.
    """

    label: str

    async def discover(self) -> Catalog:
        """Return the current catalog without installing or building anything."""
        ...

    def capabilities(self, example: Example) -> frozenset[str]:
        """Return supported actions for one record."""
        ...

    async def check_ready(self) -> Readiness:
        """Check execution prerequisites without installing or building anything."""
        ...

    async def prepare(self, emit: Callable[[str], None]) -> int:
        """Perform the preparation advertised by readiness and stream its output."""
        ...

    async def execute(self, action: str, example_id: str, emit: Callable[[str], None]) -> int:
        """Build or test an example, stream output, and return its exit code."""
        ...

    async def run(self, example_id: str, arguments: tuple[str, ...], emit: Callable[[str], None]) -> int:
        """Run an example, stream output, and reap children on cancellation."""
        ...


async def prepare_and_build(
    backend: ExamplesBackend, example_id: str, emit: Callable[[str], None]
) -> tuple[int, Readiness]:
    """Prepare the SDK, then build the example only when prerequisites are ready."""
    code = await backend.prepare(emit)
    readiness = await backend.check_ready()
    if code == 0 and readiness.ready:
        emit(f"\nBuilding example · {example_id}\n")
        code = await backend.execute("build", example_id, emit)
        readiness = await backend.check_ready()
    return code, readiness


def _decode_catalog(document: object) -> Catalog:
    """Translate the runner's versioned wire format into presentation records."""
    if not isinstance(document, dict) or document.get("schema_version") != 1:
        raise BackendError("Unsupported examples listing. Use a checkout with 'list --format json' support.")
    try:
        examples = []
        for item in document["examples"]:
            values = dict(item)
            for field in ("owners", "categories", "topics", "system_requirements", "tests"):
                values[field] = tuple(values[field])
            values["module_requirements"] = tuple(
                ModuleRequirement(entry["name"], tuple(entry["surfaces"])) for entry in values["module_requirements"]
            )
            examples.append(Example(**values))
        series = tuple(
            Series(item["id"], item["title"], item["summary"], tuple(SeriesStep(**step) for step in item["steps"]))
            for item in document["series"]
        )
        catalog = Catalog(tuple(examples), series)
        ids = [item.id for item in (*catalog.examples, *catalog.series)]
        if len(set(ids)) != len(ids):
            raise ValueError("Duplicate record IDs")
        example_ids = {example.id for example in examples}
        for item in series:
            if any(step.example not in example_ids for step in item.steps):
                raise ValueError("Unknown series step")
        return catalog
    except (KeyError, TypeError, ValueError) as error:
        raise BackendError(f"Invalid examples listing: {error}") from error


class _PosixProcess:
    """Own a POSIX process group until explicit reaping after cleanup.

    Args:
        command: Executable and literal arguments for the owned session leader.
        cwd: Working directory for the command.
        env: Environment for the command.
    """

    def __init__(self, command: list[str], *, cwd: Path, env: dict[str, str]) -> None:
        if not hasattr(os, "WNOWAIT") or signal.getsignal(signal.SIGCHLD) == signal.SIG_IGN:
            raise BackendError("Process cleanup requires waitid with retained child exit status")
        self._process = subprocess.Popen(
            command,
            cwd=cwd,
            env=env,
            stdin=subprocess.DEVNULL,
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            start_new_session=True,
        )
        self.pid = self._process.pid
        self.stdout = asyncio.StreamReader()
        self._transport: asyncio.ReadTransport | None = None
        self._returncode: int | None = None
        self._reaped = False

    @property
    def returncode(self) -> int | None:
        """Observe exit without releasing the leader's process-group identity."""
        if not self._reaped and self._returncode is None:
            result = os.waitid(os.P_PID, self.pid, os.WEXITED | os.WNOHANG | os.WNOWAIT)
            if result is not None:
                self._returncode = result.si_status if result.si_code == os.CLD_EXITED else -result.si_status
        return self._returncode

    async def connect_output(self) -> None:
        """Attach bounded asynchronous reading to the command's output pipe."""
        self._transport, _ = await asyncio.get_running_loop().connect_read_pipe(
            lambda: asyncio.StreamReaderProtocol(self.stdout), self._process.stdout
        )

    async def wait(self) -> int:
        """Wait for exit while retaining the leader until reap is called."""
        while (returncode := self.returncode) is None:
            await asyncio.sleep(0.02)
        return returncode

    async def communicate(self) -> tuple[bytes, None]:
        """Drain output and observe exit without releasing process ownership."""
        output = await self.stdout.read()
        await self.wait()
        return output, None

    def signal_group(self, sig: signal.Signals) -> None:
        """Signal only the group whose unreaped leader this object owns."""
        if self._reaped:
            raise RuntimeError("Cannot signal a process group after releasing its leader")
        # No asyncio child watcher owns this `Popen`. Do not call its `poll`,
        # `wait`, or `send_signal` methods before the final group signal: they
        # may reap the leader and permit its numeric group ID to be reused.
        os.waitid(os.P_PID, self.pid, os.WEXITED | os.WNOHANG | os.WNOWAIT)
        os.killpg(self.pid, sig)

    def kill(self) -> None:
        """Force-stop the owned group without reaping its leader."""
        self.signal_group(signal.SIGKILL)

    def reap(self) -> None:
        """Release an exited leader and its pipe after the last group signal."""
        if self._reaped:
            return
        if self.returncode is None:
            raise RuntimeError("Cannot reap a running command")
        self._process.wait()
        self._reaped = True
        if self._transport is not None:
            self._transport.close()
        else:
            self._process.stdout.close()


_Process = asyncio.subprocess.Process | _PosixProcess


async def _stop_process(process: _Process) -> None:
    """Finish owned cleanup even if cancellation is requested repeatedly."""
    cleanup = asyncio.create_task(_drain_and_terminate_process(process))
    cancelled = False
    while not cleanup.done():
        try:
            await asyncio.shield(cleanup)
        except asyncio.CancelledError:
            cancelled = True
    cleanup.result()
    if cancelled:
        raise asyncio.CancelledError


async def _drain_and_terminate_process(process: _Process) -> None:
    """Drain shutdown output while terminating the runner."""

    async def drain_output() -> None:
        """Discard shutdown output in bounded chunks until the pipe closes."""
        while await process.stdout.read(8192):
            pass

    # The streaming/JSON reader has stopped. Without a replacement reader,
    # shutdown output can fill the pipe and block wait() even after SIGKILL.
    drainer = asyncio.create_task(drain_output())
    try:
        await _terminate_process(process)
    finally:
        drainer.cancel()
        with contextlib.suppress(asyncio.CancelledError):
            await drainer


async def _terminate_process(process: _Process) -> None:
    """Terminate the owned process group, escalate, and observe runner exit."""
    if os.name == "nt":
        # Kill the tree while the root still exists; CTRL_BREAK can otherwise
        # remove the root before taskkill has enumerated detached test children.
        killer = await asyncio.create_subprocess_exec(
            "taskkill",
            "/PID",
            str(process.pid),
            "/T",
            "/F",
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
        )
        await killer.wait()
        if process.returncode is None:
            process.kill()
        await process.wait()
        return
    if not isinstance(process, _PosixProcess):
        raise BackendError("POSIX group cleanup requires an owned, unreaped leader")
    try:
        process.signal_group(signal.SIGINT)
    except ProcessLookupError:
        pass
    try:
        # Allow the runner's own detached-test cleanup to finish first.
        await asyncio.wait_for(process.wait(), timeout=_PROCESS_SHUTDOWN_SECONDS)
    except asyncio.TimeoutError:
        pass
    finally:
        try:
            process.signal_group(signal.SIGKILL)
        except ProcessLookupError:
            pass
        await process.wait()


class LocalBackend:
    """Use a selected checkout and its existing runner without importing workspace code.

    Args:
        root: Isaac Sim source checkout selected by the caller.
        python: Interpreter path relative to the caller, or a command on PATH, with runner dependencies installed.
        config: Existing developer build configuration, such as Release or Debug.
    """

    def __init__(self, root: Path, python: str = sys.executable, config: str = "Release") -> None:
        config = config.capitalize()
        if config not in _CONFIGURATIONS:
            raise BackendError(f"Unsupported build configuration: {config}")
        self.root = root.expanduser().resolve()
        # Resolve caller-relative paths before changing cwd; preserve venv symlinks.
        interpreter = Path(python).expanduser()
        self.python = str(interpreter.absolute()) if os.path.dirname(python) or interpreter != Path(python) else python
        self.config = config
        self.runner = self.root / "source/examples/examples.py"
        if not self.runner.is_file():
            raise BackendError(f"No examples runner at {self.runner}. Select an Isaac Sim checkout with --source-root.")
        self.label = f"Local checkout · {self.root} · {config}"

    def capabilities(self, example: Example) -> frozenset[str]:
        """Offer tests only for examples that declare test cases."""
        return frozenset({"build", "run", "test"} if example.tests else {"build", "run"})

    def _command(self, action: str, example_id: str | None = None) -> list[str]:
        """Build runner arguments without invoking a shell or preparing dependencies."""
        command = [self.python, "-u", str(self.runner), "--config", self.config]
        if action in {"list", "check"}:
            return [*command, action, "--format", "json"]
        if action not in {"build", "run", "test"} or not example_id or example_id.startswith("-"):
            raise BackendError("Unsupported example action or ID")
        return [*command, action, "--dev", *(["--no-build"] if action == "run" else []), example_id]

    async def _start(self, command: list[str]) -> _Process:
        """Start a process with piped output and a separately owned process group."""
        environment = {
            **os.environ,
            "PYTHONDONTWRITEBYTECODE": "1",
            "PYTHONUNBUFFERED": "1",
            "PYTHONIOENCODING": "utf-8",
        }
        try:
            if os.name != "nt":
                process = _PosixProcess(command, cwd=self.root, env=environment)
                try:
                    await process.connect_output()
                except BaseException:
                    try:
                        await _stop_process(process)
                    finally:
                        process.reap()
                    raise
                return process
            return await asyncio.create_subprocess_exec(
                *command,
                cwd=self.root,
                env=environment,
                stdin=subprocess.DEVNULL,
                stdout=subprocess.PIPE,
                stderr=subprocess.STDOUT,
                creationflags=subprocess.CREATE_NEW_PROCESS_GROUP,
            )
        except OSError as error:
            raise BackendError(f"Cannot start examples runner: {error}") from error

    async def _read_json(self, action: str) -> object:
        """Read one bounded-time machine-readable response from the runner."""
        process = await self._start(self._command(action))
        try:
            output, _ = await asyncio.wait_for(process.communicate(), timeout=_JSON_TIMEOUT_SECONDS)
        except BaseException as error:
            await _stop_process(process)
            if isinstance(error, asyncio.TimeoutError):
                raise BackendError(f"Examples {action} timed out after {_JSON_TIMEOUT_SECONDS} seconds.") from error
            raise
        finally:
            if isinstance(process, _PosixProcess):
                process.reap()
        if process.returncode:
            details = output.decode("utf-8", errors="replace")[-8000:].strip()
            raise BackendError(details or f"Examples {action} failed with exit code {process.returncode}.")
        try:
            return json.loads(output)
        except ValueError as error:
            raise BackendError("Runner did not return a valid JSON listing. Update the selected checkout.") from error

    async def discover(self) -> Catalog:
        """Read validated metadata without requiring a developer build."""
        return _decode_catalog(await self._read_json("list"))

    async def check_ready(self) -> Readiness:
        """Ask the runner for shared prerequisites and runnable example IDs."""
        document = await self._read_json("check")
        if not isinstance(document, dict) or document.get("schema_version") != 1:
            raise BackendError("Unsupported readiness response. Update the selected checkout.")
        if not isinstance(document.get("ready"), bool) or any(
            not isinstance(document.get(key), str) for key in ("message", "instructions", "details")
        ):
            raise BackendError("Invalid readiness response from the examples runner.")
        runnable = document.get("runnable_examples", [])
        if not isinstance(runnable, list) or any(not isinstance(item, str) for item in runnable):
            raise BackendError("Invalid runnable examples in readiness response.")
        return Readiness(
            document["ready"],
            document["message"],
            document["instructions"],
            document["details"],
            can_prepare=not document["ready"],
            runnable_examples=frozenset(runnable),
        )

    async def prepare(self, emit: Callable[[str], None]) -> int:
        """Use the same build driver as source/libraries/build.sh and build.bat."""
        driver = self.root / "tools/pixi_run.py"
        if not driver.is_file():
            raise BackendError(f"No library build driver at {driver}")
        flag = "-d" if self.config == "Debug" else "-r"
        return await self._stream_command([self.python, "-u", str(driver), "libraries", "--", flag], emit)

    async def execute(self, action: str, example_id: str, emit: Callable[[str], None]) -> int:
        """Stream bounded output chunks and clean up the process tree on cancellation."""
        if action not in {"build", "test"}:
            raise BackendError("Use the streaming run operation for Run.")
        return await self._stream_command(self._command(action, example_id), emit)

    async def _stream_command(self, command: list[str], emit: Callable[[str], None]) -> int:
        """Stream an owned subprocess and reap its process tree on cancellation."""
        process = await self._start(command)
        decoder = codecs.getincrementaldecoder("utf-8")(errors="replace")
        try:
            while chunk := await process.stdout.read(8192):
                if text := decoder.decode(chunk):
                    emit(text)
            if text := decoder.decode(b"", final=True):
                emit(text)
            return await process.wait()
        except BaseException:
            await _stop_process(process)
            raise
        finally:
            if isinstance(process, _PosixProcess):
                process.reap()

    async def run(self, example_id: str, arguments: tuple[str, ...], emit: Callable[[str], None]) -> int:
        """Stream a run through the same owned process lifecycle as builds and tests."""
        return await self._stream_command([*self._command("run", example_id), "--", *arguments], emit)
