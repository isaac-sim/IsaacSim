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

"""Tests for Jupyter socket communication."""

import asyncio
import json
import os

import carb
import isaacsim.core.experimental.utils.app as app_utils
import omni.kit.test

message = "Hello World!"


def _read_token() -> str:
    """Read the authentication token written by the extension at startup."""
    ext_path = app_utils.get_extension_path("isaacsim.code_editor.jupyter")
    with open(os.path.join(ext_path, "data", "launchers", "token.txt")) as f:
        return f.read().strip()


class TestSockets(omni.kit.test.AsyncTestCase):
    """Test suite for Jupyter socket communication."""

    # Before running each test
    async def setUp(self) -> None:
        """Set up test fixtures before each test."""
        settings = carb.settings.get_settings()
        self._socket_port = settings.get("/exts/isaacsim.code_editor.jupyter/port")

    # After running each test
    async def tearDown(self) -> None:
        """Tear down test fixtures after each test."""

    async def test_tcp_socket(self) -> None:
        """Test TCP socket code execution and response parsing."""
        # open TCP socket (code execution)
        reader, writer = await asyncio.open_connection("127.0.0.1", self._socket_port)
        writer.write((_read_token() + f'print("{message}")').encode())
        # wait for code execution
        for _ in range(10):
            await asyncio.sleep(0.1)
        # parse response
        data = await reader.read()
        data = json.loads(data.decode())
        # validate output
        print("response:", data)
        self.assertEqual("ok", data.get("status", None))
        self.assertEqual(message, data.get("output", None))
