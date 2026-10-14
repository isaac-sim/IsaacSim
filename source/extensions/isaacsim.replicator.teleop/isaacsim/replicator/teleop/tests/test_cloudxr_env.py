# SPDX-FileCopyrightText: Copyright (c) 2025-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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

"""Unit tests for CloudXR readiness helpers."""

from __future__ import annotations

import contextlib
import os
import socket
import tempfile
import types
from collections.abc import Iterator
from pathlib import Path
from unittest import mock

import omni.kit.test
from isaacsim.replicator.teleop import cloudxr_env
from isaacsim.replicator.teleop.cloudxr_env import (
    cloudxr_runtime_ipc_socket,
    is_cloudxr_runtime_ready,
    load_and_apply_cloudxr_env,
    parse_cloudxr_env_file,
    prepare_live_cloudxr_env,
)

_HAS_UNIX_SOCKETS = hasattr(socket, "AF_UNIX")


@contextlib.contextmanager
def _live_runtime(install_dir: Path) -> Iterator[None]:
    """Listen on the IPC path a live CloudXR runtime serves, where the platform has Unix sockets.

    Platforms without Unix domain sockets have no IPC probe, so the readiness sentinel alone
    marks the runtime as live there.
    """
    if not _HAS_UNIX_SOCKETS:
        yield
        return
    with socket.socket(socket.AF_UNIX, socket.SOCK_STREAM) as runtime:
        runtime.bind(str(cloudxr_runtime_ipc_socket(install_dir)))
        runtime.listen()
        yield


class TestCloudXREnvHelpers(omni.kit.test.AsyncTestCase):
    """Parse, apply, and readiness checks for CloudXR env files."""

    async def test_parse_and_apply_cloudxr_env(self) -> None:
        """Run the parse and apply cloudxr env test."""
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            run_dir = root / "run"
            run_dir.mkdir()
            env_path = run_dir / "cloudxr.env"
            env_path.write_text(
                "export XR_RUNTIME_JSON=/tmp/test/openxr_cloudxr.json\nexport XRT_NO_STDIN=true\n",
                encoding="utf-8",
            )
            values = parse_cloudxr_env_file(env_path)
            self.assertEqual(values["XR_RUNTIME_JSON"], "/tmp/test/openxr_cloudxr.json")
            self.assertEqual(values["XRT_NO_STDIN"], "true")

            with self.subTest("load_and_apply"), mock.patch.dict(os.environ, {}, clear=False):
                loaded = load_and_apply_cloudxr_env(root)
                self.assertEqual(loaded, env_path)
                self.assertEqual(os.environ["XR_RUNTIME_JSON"], "/tmp/test/openxr_cloudxr.json")

    async def test_is_cloudxr_runtime_ready(self) -> None:
        """A runtime is ready only once its sentinel exists and it serves its IPC socket."""
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            self.assertFalse(is_cloudxr_runtime_ready(root))
            run_dir = root / "run"
            run_dir.mkdir()
            self.assertFalse(is_cloudxr_runtime_ready(root))
            (run_dir / "runtime_started").write_text("", encoding="utf-8")
            with _live_runtime(root):
                self.assertTrue(is_cloudxr_runtime_ready(root))

    async def test_stale_sentinel_is_not_ready(self) -> None:
        """A readiness sentinel without a listening runtime does not count as ready."""
        if not _HAS_UNIX_SOCKETS:
            self.skipTest("Unix domain sockets are unavailable on this platform")
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            run_dir = root / "run"
            run_dir.mkdir()
            (run_dir / "runtime_started").write_text("", encoding="utf-8")
            self.assertFalse(is_cloudxr_runtime_ready(root))

    async def test_sentinel_decides_readiness_without_unix_sockets(self) -> None:
        """Platforms without Unix domain sockets fall back to the readiness sentinel."""
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            run_dir = root / "run"
            run_dir.mkdir()
            with mock.patch.object(cloudxr_env, "socket", types.SimpleNamespace()):
                self.assertFalse(is_cloudxr_runtime_ready(root))
                (run_dir / "runtime_started").write_text("", encoding="utf-8")
                self.assertTrue(is_cloudxr_runtime_ready(root))

    async def test_prepare_live_cloudxr_env(self) -> None:
        """prepare_live_cloudxr_env applies env vars when the runtime is ready."""
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            run_dir = root / "run"
            run_dir.mkdir()
            (run_dir / "cloudxr.env").write_text(
                "export XR_RUNTIME_JSON=/tmp/openxr.json\n",
                encoding="utf-8",
            )

            with mock.patch.dict(os.environ, {}, clear=False):
                ok, message = prepare_live_cloudxr_env(root)
                self.assertFalse(ok)
                self.assertEqual(message, "CloudXR not running")

                (run_dir / "runtime_started").write_text("", encoding="utf-8")
                with _live_runtime(root):
                    ok, message = prepare_live_cloudxr_env(root)
                    self.assertTrue(ok)
                    self.assertEqual(message, "")
                    self.assertEqual(os.environ["XR_RUNTIME_JSON"], "/tmp/openxr.json")

    async def test_parse_cloudxr_env_empty_and_quoted_values(self) -> None:
        """The env parser handles empty assignments and quoted spaces."""
        with tempfile.TemporaryDirectory() as tmp:
            env_path = Path(tmp) / "cloudxr.env"
            env_path.write_text('EMPTY=\nQUOTED="value with spaces"\nINVALID-KEY=nope\n', encoding="utf-8")
            values = parse_cloudxr_env_file(env_path)
            self.assertEqual(values["EMPTY"], "")
            self.assertEqual(values["QUOTED"], "value with spaces")
            self.assertNotIn("INVALID-KEY", values)

    async def test_prepare_rejects_malformed_cloudxr_env(self) -> None:
        """Malformed shell quoting is reported as a readiness error."""
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            run_dir = root / "run"
            run_dir.mkdir()
            (run_dir / "runtime_started").write_text("", encoding="utf-8")
            (run_dir / "cloudxr.env").write_text('BROKEN="unterminated\n', encoding="utf-8")
            with _live_runtime(root):
                ok, message = prepare_live_cloudxr_env(root)
                self.assertFalse(ok)
                self.assertIn("CloudXR env invalid", message)
