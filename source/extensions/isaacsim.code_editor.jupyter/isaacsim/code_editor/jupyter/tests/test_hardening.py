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

"""Hardening tests for the Jupyter extension's code execution server.

Tests cover:
- Basic code execution (expressions, statements, multiline, errors)
- Dangerous exceptions (SystemExit, BaseException, GeneratorExit)
- Async code execution
- Unicode and special characters
- Error message quality and response format
- Global state mutation resilience
"""

from __future__ import annotations

import asyncio
import json
import os
import unittest

import carb
import isaacsim.core.experimental.utils.app as app_utils
import omni.kit.test

_SETTINGS_PREFIX = "/exts/isaacsim.code_editor.jupyter"
_HOST = "127.0.0.1"


def _read_token() -> str:
    """Read the authentication token written by the extension at startup."""
    ext_path = app_utils.get_extension_path("isaacsim.code_editor.jupyter")
    with open(os.path.join(ext_path, "data", "launchers", "token.txt")) as f:
        return f.read().strip()


async def _send_and_receive(port: int, source: str, timeout: float = 10.0, token: str | None = None) -> dict:
    """Send source to the Jupyter server and return the parsed JSON response.

    The Jupyter extension processes code on ``data_received`` (not ``eof_received``),
    so a single write followed by a read is the expected pattern. We do NOT call
    ``write_eof()`` here because the Jupyter protocol does not require it and the
    server closes the transport after sending the reply.

    Args:
        port: The TCP port to connect to.
        source: Raw Python source string.
        timeout: Maximum seconds to wait for a response.
        token: Authentication token to prepend to ``source``. When ``None`` (the
            default) the helper reads the token from the extension's token file
            so that callers do not need to manage it explicitly. Pass an explicit
            string (including an invalid one) to exercise the auth check.

    Returns:
        The parsed JSON response dictionary.
    """
    if token is None:
        token = _read_token()
    reader, writer = await asyncio.open_connection(_HOST, port)
    writer.write((token + source).encode())
    # The Jupyter server processes on data_received and closes the transport.
    # Give it a moment to execute, then read the response.
    data = await asyncio.wait_for(reader.read(), timeout=timeout)
    writer.close()
    return json.loads(data.decode())


# ---------------------------------------------------------------------------
# 1. Basic execution
# ---------------------------------------------------------------------------


class TestBasicExecution(omni.kit.test.AsyncTestCase):
    """Test basic code execution patterns."""

    async def setUp(self) -> None:
        """Read socket configuration from Carbonite settings."""
        settings = carb.settings.get_settings()
        self._port: int = settings.get(f"{_SETTINGS_PREFIX}/port")

    async def tearDown(self) -> None:
        """Clean up after each test."""

    async def test_print_statement(self) -> None:
        """A print statement should return output with ok status."""
        data = await _send_and_receive(self._port, "print('hello')")
        self.assertEqual("ok", data.get("status"))
        self.assertEqual("hello", data.get("output"))

    async def test_syntax_error(self) -> None:
        """A syntax error should return error status with SyntaxError ename."""
        data = await _send_and_receive(self._port, "def")
        self.assertEqual("error", data.get("status"))
        self.assertEqual("SyntaxError", data.get("ename"))

    async def test_multiline_code(self) -> None:
        """Multiline code should execute correctly."""
        code = "for i in range(3):\n    print(i)"
        data = await _send_and_receive(self._port, code)
        self.assertEqual("ok", data.get("status"))
        self.assertEqual("0\n1\n2", data.get("output"))

    async def test_runtime_error(self) -> None:
        """A runtime error should return error status with proper ename."""
        data = await _send_and_receive(self._port, "1/0")
        self.assertEqual("error", data.get("status"))
        self.assertEqual("ZeroDivisionError", data.get("ename"))

    async def test_name_error(self) -> None:
        """An undefined variable should produce NameError."""
        data = await _send_and_receive(self._port, "undefined_xyz")
        self.assertEqual("error", data.get("status"))
        self.assertEqual("NameError", data.get("ename"))
        self.assertIn("undefined_xyz", data.get("evalue", ""))

    async def test_import_and_use(self) -> None:
        """Importing a module and using it should work."""
        data = await _send_and_receive(self._port, "import math; print(math.pi)")
        self.assertEqual("ok", data.get("status"))
        self.assertIn("3.14159", data.get("output", ""))

    async def test_pass_statement(self) -> None:
        """A `pass` statement should return ok status with empty output."""
        data = await _send_and_receive(self._port, "pass")
        self.assertEqual("ok", data.get("status"))
        self.assertEqual("", data.get("output"))

    async def test_trailing_newline_stripped(self) -> None:
        """Output trailing newline should be stripped (Jupyter convention)."""
        data = await _send_and_receive(self._port, "print('test')")
        self.assertEqual("ok", data.get("status"))
        # The extension strips trailing newline from output
        self.assertFalse(data.get("output", "").endswith("\n"))


# ---------------------------------------------------------------------------
# 2. Dangerous exceptions
# ---------------------------------------------------------------------------


@unittest.skipIf(os.getenv("CI"), "Skipped in CI — SystemExit/BaseException tests can destabilize the Kit process")
class TestDangerousExceptions(omni.kit.test.AsyncTestCase):
    """Test that SystemExit, BaseException, etc. don't crash Kit."""

    async def setUp(self) -> None:
        """Read socket configuration from Carbonite settings."""
        settings = carb.settings.get_settings()
        self._port: int = settings.get(f"{_SETTINGS_PREFIX}/port")

    async def tearDown(self) -> None:
        """Clean up after each test."""

    async def test_dangerous_exceptions(self) -> None:
        """All dangerous exception variants must be caught.

        Consolidated into a single test to avoid cumulative state issues.
        """
        # sys.exit(0)
        data = await _send_and_receive(self._port, "import sys; sys.exit(0)")
        self.assertEqual("error", data.get("status"), "sys.exit(0) not caught")

        # sys.exit(1)
        data = await _send_and_receive(self._port, "import sys; sys.exit(1)")
        self.assertEqual("error", data.get("status"), "sys.exit(1) not caught")

        # raise SystemExit
        data = await _send_and_receive(self._port, "raise SystemExit('bye')")
        self.assertEqual("error", data.get("status"), "raise SystemExit not caught")

        # Custom BaseException subclass
        code = "class _Boom(BaseException): pass\nraise _Boom('boom')"
        data = await _send_and_receive(self._port, code)
        self.assertEqual("error", data.get("status"), "BaseException subclass not caught")

        # GeneratorExit
        data = await _send_and_receive(self._port, "raise GeneratorExit()")
        self.assertEqual("error", data.get("status"), "GeneratorExit not caught")

        # Liveness check
        data = await _send_and_receive(self._port, "print('survived')")
        self.assertEqual("ok", data.get("status"), "Server not alive after dangerous exceptions")
        self.assertEqual("survived", data.get("output"))


# ---------------------------------------------------------------------------
# 3. Async code execution
# ---------------------------------------------------------------------------


class TestAsyncExecution(omni.kit.test.AsyncTestCase):
    """Test that async/await code works in the Jupyter executor."""

    async def setUp(self) -> None:
        """Read socket configuration from Carbonite settings."""
        settings = carb.settings.get_settings()
        self._port: int = settings.get(f"{_SETTINGS_PREFIX}/port")

    async def tearDown(self) -> None:
        """Clean up after each test."""

    async def test_simple_await(self) -> None:
        """Simple await expression should work."""
        code = "import asyncio; await asyncio.sleep(0); print('awaited')"
        data = await _send_and_receive(self._port, code)
        self.assertEqual("ok", data.get("status"))
        self.assertIn("awaited", data.get("output", ""))

    async def test_async_exception(self) -> None:
        """Exception inside async code should be captured."""
        code = "import asyncio\nawait asyncio.sleep(0)\nraise ValueError('async_error')"
        data = await _send_and_receive(self._port, code)
        self.assertEqual("error", data.get("status"))
        self.assertEqual("ValueError", data.get("ename"))
        self.assertIn("async_error", data.get("evalue", ""))

    async def test_nested_async(self) -> None:
        """Nested async function calls should work."""
        code = (
            "import asyncio\n"
            "async def inner():\n"
            "    await asyncio.sleep(0)\n"
            "    return 42\n"
            "result = await inner()\n"
            "print(result)"
        )
        data = await _send_and_receive(self._port, code)
        self.assertEqual("ok", data.get("status"))
        self.assertIn("42", data.get("output", ""))


# ---------------------------------------------------------------------------
# 4. Unicode and special characters
# ---------------------------------------------------------------------------


class TestUnicodeHandling(omni.kit.test.AsyncTestCase):
    """Test that unicode in code and output works correctly."""

    async def setUp(self) -> None:
        """Read socket configuration from Carbonite settings."""
        settings = carb.settings.get_settings()
        self._port: int = settings.get(f"{_SETTINGS_PREFIX}/port")

    async def tearDown(self) -> None:
        """Clean up after each test."""

    async def test_unicode_output(self) -> None:
        """Unicode characters in print output should be preserved."""
        data = await _send_and_receive(self._port, 'print("Hello 世界 🌍")')
        self.assertEqual("ok", data.get("status"))
        self.assertEqual("Hello 世界 🌍", data.get("output"))

    async def test_unicode_variable(self) -> None:
        """Unicode variable names and values should work."""
        data = await _send_and_receive(self._port, 'café = "☕"; print(café)')
        self.assertEqual("ok", data.get("status"))
        self.assertEqual("☕", data.get("output"))

    async def test_newlines_in_output(self) -> None:
        """Embedded newlines should be preserved."""
        data = await _send_and_receive(self._port, r'print("line1\nline2\nline3")')
        self.assertEqual("ok", data.get("status"))
        self.assertEqual("line1\nline2\nline3", data.get("output"))


# ---------------------------------------------------------------------------
# 5. Error messages and response format
# ---------------------------------------------------------------------------


class TestErrorMessages(omni.kit.test.AsyncTestCase):
    """Test that error responses contain useful diagnostic information."""

    async def setUp(self) -> None:
        """Read socket configuration from Carbonite settings."""
        settings = carb.settings.get_settings()
        self._port: int = settings.get(f"{_SETTINGS_PREFIX}/port")

    async def tearDown(self) -> None:
        """Clean up after each test."""

    async def test_error_response_shape(self) -> None:
        """Error responses must include status, ename, evalue, and traceback."""
        data = await _send_and_receive(self._port, "1/0")
        self.assertIn("status", data)
        self.assertIn("ename", data)
        self.assertIn("evalue", data)
        self.assertIn("traceback", data)
        self.assertEqual("error", data["status"])

    async def test_success_response_shape(self) -> None:
        """Success responses must include status and output."""
        data = await _send_and_receive(self._port, "print('ok')")
        self.assertIn("status", data)
        self.assertIn("output", data)
        self.assertEqual("ok", data["status"])

    async def test_traceback_present(self) -> None:
        """Error responses should include traceback text."""
        data = await _send_and_receive(self._port, "1/0")
        self.assertIn("traceback", data)
        traceback_str = data.get("traceback", "")
        if isinstance(traceback_str, list):
            traceback_str = "\n".join(traceback_str)
        self.assertIn("ZeroDivisionError", traceback_str)

    async def test_indentation_error(self) -> None:
        """IndentationError should be reported correctly."""
        data = await _send_and_receive(self._port, "if True:\nprint('bad')")
        self.assertEqual("error", data.get("status"))
        self.assertIn(data.get("ename", ""), ("IndentationError", "SyntaxError"))

    async def test_recursion_error(self) -> None:
        """RecursionError from infinite recursion should be caught."""
        data = await _send_and_receive(self._port, "def f(): f()\nf()")
        self.assertEqual("error", data.get("status"))
        self.assertEqual("RecursionError", data.get("ename"))

        # Server must survive
        data = await _send_and_receive(self._port, "print('after_recursion')")
        self.assertEqual("ok", data.get("status"))


# ---------------------------------------------------------------------------
# 6. Global state resilience
# ---------------------------------------------------------------------------


class TestGlobalState(omni.kit.test.AsyncTestCase):
    """Test that user code mutating globals doesn't break subsequent requests."""

    async def setUp(self) -> None:
        """Read socket configuration from Carbonite settings."""
        settings = carb.settings.get_settings()
        self._port: int = settings.get(f"{_SETTINGS_PREFIX}/port")

    async def tearDown(self) -> None:
        """Clean up after each test."""

    async def test_variable_persistence(self) -> None:
        """Variables set in one request should persist in the next.

        The Jupyter extension uses a single shared global namespace.
        """
        data = await _send_and_receive(self._port, "jupyter_test_var = 42")
        self.assertEqual("ok", data.get("status"))

        data = await _send_and_receive(self._port, "print(jupyter_test_var)")
        self.assertEqual("ok", data.get("status"))
        self.assertEqual("42", data.get("output"))

        # Clean up
        await _send_and_receive(self._port, "del jupyter_test_var")

    async def test_import_persistence(self) -> None:
        """Imports should persist across requests."""
        data = await _send_and_receive(self._port, "import os as _jupyter_test_os")
        self.assertEqual("ok", data.get("status"))

        data = await _send_and_receive(self._port, "print(_jupyter_test_os.sep)")
        self.assertEqual("ok", data.get("status"))
        self.assertIn(data.get("output", ""), ("/", "\\"))

        # Clean up
        await _send_and_receive(self._port, "del _jupyter_test_os")

    async def test_override_print_recovers(self) -> None:
        """Overriding print should not permanently break the server.

        Since the Jupyter extension shares a single namespace, overriding print
        WILL affect subsequent calls. But builtins.print should still work.
        """
        data = await _send_and_receive(self._port, "print = lambda *a: None\nprint('silent')")
        self.assertEqual("ok", data.get("status"))
        # Output should be empty since print was overridden
        self.assertEqual("", data.get("output"))

        # Restore print
        data = await _send_and_receive(self._port, "import builtins; print = builtins.print")
        self.assertEqual("ok", data.get("status"))

        # Now print should work again
        data = await _send_and_receive(self._port, "print('restored')")
        self.assertEqual("ok", data.get("status"))
        self.assertEqual("restored", data.get("output"))


# ---------------------------------------------------------------------------
# 7. Completion protocol
# ---------------------------------------------------------------------------
# NOTE: Jedi completion tests are skipped in environments where pip dependencies
# (jupyterlab, python-language-server) fail to install. The completion protocol
# uses %!c and %!i prefixes handled by Jedi, which requires a fully configured
# project. When these tests are needed, ensure `jedi` is installed in the Kit
# Python environment.


# ---------------------------------------------------------------------------
# 8. Rapid sequential requests
# ---------------------------------------------------------------------------


class TestSequentialRequests(omni.kit.test.AsyncTestCase):
    """Test rapid sequential requests don't break the server."""

    async def setUp(self) -> None:
        """Read socket configuration from Carbonite settings."""
        settings = carb.settings.get_settings()
        self._port: int = settings.get(f"{_SETTINGS_PREFIX}/port")

    async def tearDown(self) -> None:
        """Clean up after each test."""

    async def test_rapid_sequential(self) -> None:
        """10 rapid sequential requests should all get valid responses."""
        for i in range(10):
            data = await _send_and_receive(self._port, f"print({i})")
            self.assertEqual("ok", data.get("status"))
            self.assertEqual(str(i), data.get("output"))


# ---------------------------------------------------------------------------
# 9. Authentication
# ---------------------------------------------------------------------------


class TestAuthentication(omni.kit.test.AsyncTestCase):
    """Verify that the token-based authentication gate cannot be bypassed."""

    async def setUp(self) -> None:
        """Read socket configuration from Carbonite settings."""
        settings = carb.settings.get_settings()
        self._port: int = settings.get(f"{_SETTINGS_PREFIX}/port")

    async def tearDown(self) -> None:
        """Clean up after each test."""

    async def test_invalid_token_rejected(self) -> None:
        """A request with a wrong token of the correct length must be rejected."""
        # Same length so the server slices the prefix in the same place; only the bytes differ.
        bogus = "0" * len(_read_token())
        data = await _send_and_receive(self._port, "print('should not run')", token=bogus)
        self.assertEqual("error", data.get("status"))
        self.assertEqual("AuthenticationError", data.get("ename"))
        self.assertEqual("", data.get("output"))

    async def test_missing_token_rejected(self) -> None:
        """A request with no token prefix must be rejected, not executed."""
        data = await _send_and_receive(self._port, "print('should not run')", token="")
        self.assertEqual("error", data.get("status"))
        self.assertEqual("AuthenticationError", data.get("ename"))

    async def test_server_alive_after_rejected_request(self) -> None:
        """A rejected request must not destabilize the server for subsequent valid traffic."""
        bogus = "0" * len(_read_token())
        await _send_and_receive(self._port, "print('nope')", token=bogus)
        # Valid token should succeed immediately after.
        data = await _send_and_receive(self._port, "print('alive')")
        self.assertEqual("ok", data.get("status"))
        self.assertEqual("alive", data.get("output"))
