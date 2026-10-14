# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0

"""Exercise POSIX group ownership and real subprocess cleanup."""

from __future__ import annotations

import asyncio
import os
import signal
import sys
import tempfile
import unittest
from pathlib import Path
from unittest import mock

from isaacsim_installer.core import backend as module


@unittest.skipIf(os.name == "nt", "POSIX process-group ownership")
class TestProcessGroups(unittest.IsolatedAsyncioTestCase):
    """Check cleanup without permitting a group ID to outlive ownership."""

    async def asyncSetUp(self) -> None:
        """Create a minimal checkout for real subprocess tests."""
        self.temporary = tempfile.TemporaryDirectory(prefix="installer-process-test-")
        self.root = Path(self.temporary.name)
        runner = self.root / "source/examples/examples.py"
        runner.parent.mkdir(parents=True)
        runner.write_text("", encoding="utf-8")
        self.backend = module.LocalBackend(self.root)
        self.processes: list[module._PosixProcess] = []

    async def asyncTearDown(self) -> None:
        """Reap every owned subprocess even if an assertion fails."""
        for process in self.processes:
            if not process._reaped:
                process.kill()
                await process.wait()
                process.reap()
        self.temporary.cleanup()

    async def start(self, source: str) -> module._PosixProcess:
        """Launch a short Python program in its own owned group."""
        process = await self.backend._start([sys.executable, "-u", "-c", source])
        self.processes.append(process)
        return process

    async def test_exit_keeps_leader_waitable_until_last_signal(self) -> None:
        """Observe exit without allowing group-ID reuse before cleanup."""
        process = await self.start("raise SystemExit(7)")
        self.assertEqual(await asyncio.wait_for(process.wait(), 3), 7)
        real_killpg = os.killpg
        observed = []

        def checked_signal(pid: int, sig: signal.Signals) -> None:
            status = os.waitid(os.P_PID, pid, os.WEXITED | os.WNOHANG | os.WNOWAIT)
            self.assertEqual(status.si_pid, process.pid)
            observed.append(sig)
            real_killpg(pid, sig)

        with mock.patch.object(module.os, "killpg", side_effect=checked_signal):
            await module._stop_process(process)
        self.assertEqual(observed, [signal.SIGINT, signal.SIGKILL])
        process.reap()
        with self.assertRaises(ChildProcessError):
            os.waitid(os.P_PID, process.pid, os.WEXITED | os.WNOHANG | os.WNOWAIT)
        with mock.patch.object(module.os, "killpg") as killpg:
            with self.assertRaises(RuntimeError):
                process.signal_group(signal.SIGKILL)
            killpg.assert_not_called()

    async def test_reaped_elsewhere_refuses_group_signal(self) -> None:
        """Fail closed if an external reaper breaks exclusive child ownership."""
        process = await self.start("pass")
        await asyncio.wait_for(process.wait(), 3)
        process._process.wait()
        try:
            with mock.patch.object(module.os, "killpg") as killpg:
                with self.assertRaises(ChildProcessError):
                    process.signal_group(signal.SIGINT)
                killpg.assert_not_called()
        finally:
            process.reap()

    async def test_timeout_escalates_to_owned_group(self) -> None:
        """Kill an unresponsive group after the graceful shutdown timeout."""
        process = await self.start(
            "import signal, time; signal.signal(signal.SIGINT, signal.SIG_IGN); "
            "print('ready', flush=True); time.sleep(30)"
        )
        await asyncio.wait_for(process.stdout.readline(), 3)
        with mock.patch.object(module, "_PROCESS_SHUTDOWN_SECONDS", 0.05):
            await asyncio.wait_for(module._stop_process(process), 3)
        self.assertEqual(process.returncode, -signal.SIGKILL)
        process.reap()

    async def test_exited_leader_still_cleans_surviving_child(self) -> None:
        """Retain ownership when descendants outlive their session leader."""
        child = (
            "import signal, time; from pathlib import Path; "
            "signal.signal(signal.SIGINT, signal.SIG_IGN); "
            "print('child ready', flush=True); time.sleep(2); Path('leaked').touch()"
        )
        process = await self.start(f"import subprocess, sys; subprocess.Popen([sys.executable, '-u', '-c', {child!r}])")
        self.assertEqual(await asyncio.wait_for(process.stdout.readline(), 3), b"child ready\n")
        self.assertEqual(await asyncio.wait_for(process.wait(), 3), 0)
        await asyncio.wait_for(module._stop_process(process), 1)
        self.assertEqual(await asyncio.wait_for(process.stdout.read(), 1), b"")
        self.assertFalse((self.root / "leaked").exists())
        process.reap()

    async def test_noisy_graceful_shutdown_is_drained(self) -> None:
        """Drain more than a pipe buffer while preserving graceful exit."""
        process = await self.start(
            "import signal, sys, time\n"
            "def stop(*args):\n"
            "    sys.stdout.write('x' * 1_000_000)\n"
            "    sys.stdout.flush()\n"
            "    raise SystemExit(0)\n"
            "signal.signal(signal.SIGINT, stop)\n"
            "print('ready', flush=True)\n"
            "time.sleep(30)\n"
        )
        await asyncio.wait_for(process.stdout.readline(), 3)
        await asyncio.wait_for(module._stop_process(process), 3)
        self.assertEqual(process.returncode, 0)
        process.reap()

    async def test_repeated_cancellation_finishes_cleanup(self) -> None:
        """Do not release group ownership when cancellation repeats."""
        process = await self.start(
            "import signal, time; signal.signal(signal.SIGINT, signal.SIG_IGN); "
            "print('ready', flush=True); time.sleep(30)"
        )
        await asyncio.wait_for(process.stdout.readline(), 3)
        with mock.patch.object(module, "_PROCESS_SHUTDOWN_SECONDS", 0.1):
            cleanup = asyncio.create_task(module._stop_process(process))
            await asyncio.sleep(0.02)
            cleanup.cancel()
            await asyncio.sleep(0.02)
            cleanup.cancel()
            with self.assertRaises(asyncio.CancelledError):
                await cleanup
        self.assertEqual(process.returncode, -signal.SIGKILL)
        process.reap()

    async def test_normal_command_returns_output_and_reaps(self) -> None:
        """Preserve streamed output and return status on normal completion."""
        output = []
        original_start = self.backend._start

        async def capture(command: list[str]) -> module._PosixProcess:
            process = await original_start(command)
            self.processes.append(process)
            return process

        with mock.patch.object(self.backend, "_start", side_effect=capture):
            result = await self.backend._stream_command(
                [sys.executable, "-c", "print('complete'); raise SystemExit(4)"], output.append
            )
        self.assertEqual(result, 4)
        self.assertEqual("".join(output), "complete\n")
        self.assertTrue(self.processes[-1]._reaped)

    async def test_output_callback_failure_cleans_and_reaps(self) -> None:
        """Keep cleanup effective when an output callback raises."""
        original_start = self.backend._start

        async def capture(command: list[str]) -> module._PosixProcess:
            process = await original_start(command)
            self.processes.append(process)
            return process

        def fail(text: str) -> None:
            raise ValueError("callback failed")

        with mock.patch.object(self.backend, "_start", side_effect=capture):
            with self.assertRaisesRegex(ValueError, "callback failed"):
                await self.backend._stream_command(
                    [sys.executable, "-u", "-c", "import time; print('ready'); time.sleep(30)"], fail
                )
        self.assertTrue(self.processes[-1]._reaped)

    async def test_output_connection_failure_cleans_and_reaps(self) -> None:
        """Do not leak the subprocess if pipe registration fails."""
        captured = []

        async def fail(process: module._PosixProcess) -> None:
            captured.append(process)
            raise OSError("pipe registration failed")

        with mock.patch.object(module._PosixProcess, "connect_output", fail):
            with self.assertRaises(module.BackendError):
                await self.backend._start([sys.executable, "-c", "import time; time.sleep(30)"])
        self.assertTrue(captured[0]._reaped)

    async def test_output_connection_cancellation_cleans_and_reaps(self) -> None:
        """Retain ownership across cancellation during pipe registration."""
        captured = []

        async def cancel(process: module._PosixProcess) -> None:
            captured.append(process)
            raise asyncio.CancelledError

        with mock.patch.object(module._PosixProcess, "connect_output", cancel):
            with self.assertRaises(asyncio.CancelledError):
                await self.backend._start([sys.executable, "-c", "import time; time.sleep(30)"])
        self.assertTrue(captured[0]._reaped)

    async def test_ignored_child_status_refuses_to_spawn(self) -> None:
        """Do not create a group if the OS would automatically reap its leader."""
        with (
            mock.patch.object(module.signal, "getsignal", return_value=signal.SIG_IGN),
            mock.patch.object(module.subprocess, "Popen") as popen,
        ):
            with self.assertRaises(module.BackendError):
                await self.backend._start([sys.executable, "-c", "pass"])
            popen.assert_not_called()

    async def test_discovery_timeout_cleans_and_reaps(self) -> None:
        """Bound JSON discovery and release the timed-out child."""
        runner = self.root / "source/examples/examples.py"
        runner.write_text("import time; time.sleep(30)", encoding="utf-8")
        original_start = self.backend._start

        async def capture(command: list[str]) -> module._PosixProcess:
            process = await original_start(command)
            self.processes.append(process)
            return process

        with (
            mock.patch.object(self.backend, "_start", side_effect=capture),
            mock.patch.object(module, "_JSON_TIMEOUT_SECONDS", 0.05),
        ):
            with self.assertRaisesRegex(module.BackendError, "timed out"):
                await self.backend.discover()
        self.assertTrue(self.processes[-1]._reaped)

    async def test_json_discovery_returns_successful_output(self) -> None:
        """Preserve the JSON subprocess contract."""
        runner = self.root / "source/examples/examples.py"
        runner.write_text("print('{\"value\": 42}')", encoding="utf-8")
        self.assertEqual(await self.backend._read_json("list"), {"value": 42})

    async def test_cancel_preserves_detached_runner_cleanup(self) -> None:
        """Give the examples runner time to reap a test in another session."""
        runner_path = Path(__file__).resolve().parents[3] / "source/examples/examples.py"
        child = (
            "from pathlib import Path; import os, time; "
            "Path('child.pid').write_text(str(os.getpid())); Path('ready').touch(); "
            "time.sleep(2); Path('leaked').touch()"
        )
        source = (
            "import importlib.util, os, sys\n"
            f"spec = importlib.util.spec_from_file_location('runner', {str(runner_path)!r})\n"
            "runner = importlib.util.module_from_spec(spec)\n"
            "sys.modules['runner'] = runner\n"
            "spec.loader.exec_module(runner)\n"
            f"runner._run_captured([sys.executable, '-c', {child!r}], runner.Path.cwd(), os.environ.copy(), 30)\n"
        )
        output: list[str] = []
        task = asyncio.create_task(self.backend._stream_command([sys.executable, "-u", "-c", source], output.append))

        async def wait_ready() -> None:
            while not (self.root / "ready").exists():
                if task.done():
                    self.fail(f"Runner exited before starting its child: {task.result()}; {''.join(output)}")
                await asyncio.sleep(0.02)

        try:
            await asyncio.wait_for(wait_ready(), 3)
        finally:
            if not task.done():
                task.cancel()
                with self.assertRaises(asyncio.CancelledError):
                    await task
        child_pid = int((self.root / "child.pid").read_text())
        with self.assertRaises(ProcessLookupError):
            os.kill(child_pid, 0)
        self.assertFalse((self.root / "leaked").exists())
