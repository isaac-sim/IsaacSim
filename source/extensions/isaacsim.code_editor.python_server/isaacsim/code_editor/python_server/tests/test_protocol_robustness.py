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

"""Test TCP protocol robustness: fragmentation handling and async stdout capture."""

from __future__ import annotations

import asyncio
import json

import carb
import omni.kit.test

from ._auth import add_auth_header

_SETTINGS_PREFIX = "/exts/isaacsim.code_editor.python_server"
_HOST = "127.0.0.1"


async def _send_and_receive(port: int, source: str) -> dict:
    """Send Python source to the server and return the parsed JSON response.

    Args:
        port: The TCP port to connect to.
        source: The Python source code to send.

    Returns:
        The parsed JSON response dictionary.
    """
    reader, writer = await asyncio.open_connection(_HOST, port)
    writer.write(add_auth_header(source).encode())
    writer.write_eof()
    data = await asyncio.wait_for(reader.read(), timeout=30.0)
    writer.close()
    return json.loads(data.decode())


async def _send_fragmented_and_receive(port: int, source: str, chunk_size: int = 10) -> dict:
    """Send Python source in small chunks to simulate TCP fragmentation.

    Args:
        port: The TCP port to connect to.
        source: The Python source code to send.
        chunk_size: Number of bytes per chunk.

    Returns:
        The parsed JSON response dictionary.
    """
    reader, writer = await asyncio.open_connection(_HOST, port)
    encoded = add_auth_header(source).encode()
    for i in range(0, len(encoded), chunk_size):
        writer.write(encoded[i : i + chunk_size])
        await writer.drain()
        await asyncio.sleep(0.01)
    writer.write_eof()
    data = await asyncio.wait_for(reader.read(), timeout=30.0)
    writer.close()
    return json.loads(data.decode())


class TestTcpFragmentation(omni.kit.test.AsyncTestCase):
    """Test that the TCP server correctly handles fragmented payloads.

    TCP does not guarantee that a single ``send()`` from the client arrives as a single
    ``data_received()`` on the server.  These tests verify that the server buffers
    incoming data and only processes the complete payload after the client signals EOF.
    """

    async def setUp(self) -> None:
        """Read socket configuration from Carbonite settings."""
        settings = carb.settings.get_settings()
        self._port: int = settings.get(f"{_SETTINGS_PREFIX}/port")

    async def tearDown(self) -> None:
        """Clean up after each test."""

    async def test_fragmented_print(self) -> None:
        """Verify that a print statement sent in small TCP chunks executes correctly."""
        source = 'print("fragmented hello")'
        data = await _send_fragmented_and_receive(self._port, source, chunk_size=5)
        print("response:", data)
        self.assertEqual("ok", data.get("status"))
        self.assertEqual("fragmented hello", data.get("output"))

    async def test_fragmented_multiline(self) -> None:
        """Verify that multiline code sent in fragments produces correct output."""
        source = "results = []\nfor i in range(5):\n    results.append(i * 2)\nprint(','.join(str(r) for r in results))"
        data = await _send_fragmented_and_receive(self._port, source, chunk_size=8)
        print("response:", data)
        self.assertEqual("ok", data.get("status"))
        self.assertEqual("0,2,4,6,8", data.get("output"))

    async def test_fragmented_large_payload(self) -> None:
        """Verify that a large code block sent in small fragments executes correctly."""
        # Generate a code block larger than typical TCP segment sizes
        lines = [f"x_{i} = {i}" for i in range(100)]
        lines.append("print(x_0 + x_99)")
        source = "\n".join(lines)
        self.assertGreater(len(source), 500, "Payload should be large enough to fragment")

        data = await _send_fragmented_and_receive(self._port, source, chunk_size=64)
        print("response:", data)
        self.assertEqual("ok", data.get("status"))
        self.assertEqual("99", data.get("output"))

    async def test_fragmented_expression(self) -> None:
        """Verify that an expression sent in fragments returns the correct result."""
        source = "sum(range(100))"
        data = await _send_fragmented_and_receive(self._port, source, chunk_size=3)
        print("response:", data)
        self.assertEqual("ok", data.get("status"))
        self.assertEqual(4950, data.get("result"))


class TestAsyncStdout(omni.kit.test.AsyncTestCase):
    """Test that print() output from async (coroutine) code is captured in the response.

    The server uses ``contextlib.redirect_stdout`` to capture ``print()`` output.
    For synchronous code this works, but for coroutines the actual execution happens
    later in ``_await_and_reply``.  These tests verify that stdout from awaited code
    is included in the JSON response ``output`` field.
    """

    async def setUp(self) -> None:
        """Read socket configuration from Carbonite settings."""
        settings = carb.settings.get_settings()
        self._port: int = settings.get(f"{_SETTINGS_PREFIX}/port")

    async def tearDown(self) -> None:
        """Clean up after each test."""

    async def test_async_print_single(self) -> None:
        """Verify that a single print() inside async code appears in output."""
        source = 'import asyncio\nawait asyncio.sleep(0)\nprint("async hello")'
        data = await _send_and_receive(self._port, source)
        print("response:", data)
        self.assertEqual("ok", data.get("status"))
        self.assertIn("async hello", data.get("output", ""))

    async def test_async_print_multiple(self) -> None:
        """Verify that multiple print() calls inside async code all appear in output."""
        source = (
            "import asyncio\n"
            'print("line1")\n'
            "await asyncio.sleep(0)\n"
            'print("line2")\n'
            "await asyncio.sleep(0)\n"
            'print("line3")'
        )
        data = await _send_and_receive(self._port, source)
        print("response:", data)
        self.assertEqual("ok", data.get("status"))
        output = data.get("output", "")
        self.assertIn("line1", output)
        self.assertIn("line2", output)
        self.assertIn("line3", output)

    async def test_async_print_with_computation(self) -> None:
        """Verify that print() with computed values in async code appears in output."""
        source = (
            "import asyncio\n" "await asyncio.sleep(0)\n" "result = sum(range(10))\n" 'print(f"computed: {result}")'
        )
        data = await _send_and_receive(self._port, source)
        print("response:", data)
        self.assertEqual("ok", data.get("status"))
        self.assertIn("computed: 45", data.get("output", ""))

    async def test_sync_print_still_works(self) -> None:
        """Verify that synchronous print() is unaffected (regression check)."""
        source = 'print("sync hello")'
        data = await _send_and_receive(self._port, source)
        print("response:", data)
        self.assertEqual("ok", data.get("status"))
        self.assertEqual("sync hello", data.get("output"))
