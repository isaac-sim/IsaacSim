# SPDX-FileCopyrightText: Copyright (c) 2018-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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

"""Test TCP and UDP socket communication for the Python server extension."""

from __future__ import annotations

import asyncio
import json

import carb
import omni.kit.test

from ._auth import add_auth_header

_SETTINGS_PREFIX = "/exts/isaacsim.code_editor.python_server"
_HOST = "127.0.0.1"
_MESSAGE = "Hello World!"


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


async def _send_raw_payload(port: int, payload: str) -> dict:
    """Send a payload exactly as provided and return the parsed JSON response.

    Args:
        port: The TCP port to connect to.
        payload: The raw payload to send without adding authentication.

    Returns:
        The parsed JSON response dictionary.
    """
    reader, writer = await asyncio.open_connection(_HOST, port)
    writer.write(payload.encode())
    writer.write_eof()
    data = await asyncio.wait_for(reader.read(), timeout=30.0)
    writer.close()
    return json.loads(data.decode())


class TestSockets(omni.kit.test.AsyncTestCase):
    """Test TCP and UDP socket communication for the Python server extension."""

    async def setUp(self) -> None:
        """Read socket configuration from Carbonite settings."""
        settings = carb.settings.get_settings()
        self._socket_port: int = settings.get(f"{_SETTINGS_PREFIX}/port")
        self._publish_carb_logs: bool = settings.get(f"{_SETTINGS_PREFIX}/carb_logs")

    async def tearDown(self) -> None:
        """Clean up after each test."""

    async def test_tcp_socket(self) -> None:
        """Verify that a print statement returns output via TCP."""
        data = await _send_and_receive(self._socket_port, f'print("{_MESSAGE}")')
        print("response:", data)
        self.assertEqual("ok", data.get("status"))
        self.assertEqual(_MESSAGE, data.get("output"))

    async def test_tcp_socket_missing_auth_token(self) -> None:
        """Verify that unauthenticated requests are rejected before execution."""
        data = await _send_raw_payload(self._socket_port, f'print("{_MESSAGE}")')
        print("response:", data)
        self.assertEqual("error", data.get("status"))
        self.assertEqual("AuthenticationError", data.get("ename"))

    async def test_tcp_socket_invalid_auth_token(self) -> None:
        """Verify that requests with an invalid token are rejected before execution."""
        data = await _send_raw_payload(self._socket_port, "# isaacsim-python-server-token: invalid\nprint('bad')")
        print("response:", data)
        self.assertEqual("error", data.get("status"))
        self.assertEqual("AuthenticationError", data.get("ename"))

    async def test_tcp_socket_error(self) -> None:
        """Verify that a syntax error returns error status via TCP."""
        data = await _send_and_receive(self._socket_port, "def")
        print("response:", data)
        self.assertEqual("error", data.get("status"))
        self.assertIn("ename", data)
        self.assertIn("evalue", data)
        self.assertIn("traceback", data)
        self.assertEqual("SyntaxError", data["ename"])

    async def test_tcp_socket_expression(self) -> None:
        """Verify that an expression returns its evaluated result."""
        data = await _send_and_receive(self._socket_port, "1 + 1")
        print("response:", data)
        self.assertEqual("ok", data.get("status"))
        self.assertIn("result", data)
        self.assertEqual(2, data["result"])

    async def test_tcp_socket_expression_non_json(self) -> None:
        """Verify that a non-JSON-serializable expression returns a repr string."""
        data = await _send_and_receive(self._socket_port, "object()")
        print("response:", data)
        self.assertEqual("ok", data.get("status"))
        self.assertIn("result", data)
        self.assertIsInstance(data["result"], str)
        self.assertTrue(data["result"].startswith("<object object at"))

    async def test_sync_code_not_in_asyncio_task(self) -> None:
        """Verify that synchronous code does not execute inside an asyncio Task.

        Running user code inside a Task causes ``RuntimeError: Cannot enter
        into task … while another task … is being executed`` when the code
        pumps the Kit event loop (e.g. ``update_app``) and pending tasks try
        to wake up.  This test ensures the fix stays in place.
        """
        data = await _send_and_receive(self._socket_port, "__import__('asyncio').current_task()")
        print("response:", data)
        self.assertEqual("ok", data.get("status"))
        self.assertIsNone(data["result"], "Synchronous code must not execute inside an asyncio Task")

    async def test_tcp_socket_multiline(self) -> None:
        """Verify that multiline code returns combined output."""
        source = "for i in range(3):\n    print(i)"
        data = await _send_and_receive(self._socket_port, source)
        print("response:", data)
        self.assertEqual("ok", data.get("status"))
        self.assertEqual("0\n1\n2", data.get("output"))

    async def test_udp_socket(self) -> None:
        """Verify that carb log messages are broadcast to UDP clients."""

        class _ClientProtocol(asyncio.Protocol):
            def connection_made(self, transport: asyncio.BaseTransport) -> None:
                self.transport = transport
                self._data: list[str] = []

            def datagram_received(self, data: bytes, addr: tuple[str, int]) -> None:
                self._data.append(data.decode())

            def get_data(self) -> str:
                return " ".join(self._data)

        if not self._publish_carb_logs:
            carb.log_warn("Carb log publishing is disabled")
            return

        transport, protocol = await asyncio.get_event_loop().create_datagram_endpoint(
            lambda: _ClientProtocol(), remote_addr=(_HOST, self._socket_port)
        )
        transport.sendto(b"*")

        await _send_and_receive(self._socket_port, f'carb.log_info("{_MESSAGE}")')

        for _ in range(10):
            await asyncio.sleep(0.1)

        transport.close()
        print("logs:", protocol.get_data())
        self.assertIn(
            f"[Info][isaacsim.code_editor.python_server.extension] {_MESSAGE}",
            protocol.get_data(),
        )
