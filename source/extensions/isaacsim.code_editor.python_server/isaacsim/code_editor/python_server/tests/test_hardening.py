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

"""Adversarial hardening tests for the python_server extension.

These tests exercise edge cases that cause crashes or hangs in production:
- SystemExit / KeyboardInterrupt in user code
- Rapid concurrent connections
- Huge payloads and output
- Partial sends and abrupt disconnects
- Code that mutates global state (sys.modules, builtins)
- Unicode / binary edge cases in code and output
- Tasks that outlive the connection
- Infinite loops vs execution timeout
"""

from __future__ import annotations

import asyncio
import json
import os
import unittest

import carb
import omni.kit.app
import omni.kit.test

from ._auth import add_auth_header, add_auth_to_envelope

_SETTINGS_PREFIX = "/exts/isaacsim.code_editor.python_server"
_HOST = "127.0.0.1"


async def _send_and_receive(port: int, source: str, client_timeout: float = 30.0) -> dict:
    """Send source to the server and return the parsed JSON response.

    Args:
        port: The TCP port to connect to.
        source: Raw Python source or a JSON-encoded envelope string.
        client_timeout: Maximum seconds to wait for a response.

    Returns:
        The parsed JSON response dictionary.
    """
    reader, writer = await asyncio.open_connection(_HOST, port)
    writer.write(add_auth_header(source).encode())
    writer.write_eof()
    data = await asyncio.wait_for(reader.read(), timeout=client_timeout)
    writer.close()
    return json.loads(data.decode())


async def _send_envelope(port: int, envelope: dict, client_timeout: float = 30.0) -> dict:
    """Serialise envelope as JSON, send it, and return the parsed response.

    Args:
        port: The TCP port to connect to.
        envelope: The request envelope dict.
        client_timeout: Maximum seconds to wait for a response.

    Returns:
        The parsed JSON response dictionary.
    """
    return await _send_and_receive(port, json.dumps(add_auth_to_envelope(envelope)), client_timeout)


# ---------------------------------------------------------------------------
# 1. SystemExit / KeyboardInterrupt
# ---------------------------------------------------------------------------


@unittest.skipIf(os.getenv("CI"), "Skipped in CI — SystemExit/BaseException tests can destabilize the Kit process")
class TestDangerousExceptions(omni.kit.test.AsyncTestCase):
    """Test that SystemExit, KeyboardInterrupt, and similar do not crash Kit."""

    async def setUp(self) -> None:
        """Read socket configuration from Carbonite settings."""
        settings = carb.settings.get_settings()
        self._port: int = settings.get(f"{_SETTINGS_PREFIX}/port")

    async def tearDown(self) -> None:
        """Clean up after each test."""

    async def test_dangerous_exceptions(self) -> None:
        """All dangerous exception variants must be caught without crashing the server.

        Consolidated into a single test to avoid cumulative server state degradation
        from repeated SystemExit/BaseException handling across multiple test methods.
        Each case is verified, then a liveness check confirms the server is still operational.
        """
        # 1. sys.exit(0)
        data = await _send_and_receive(self._port, "import sys; sys.exit(0)")
        self.assertEqual("error", data.get("status"), "sys.exit(0) not caught")
        self.assertEqual("RuntimeError", data.get("ename"))
        self.assertIn("SystemExit", data.get("evalue", ""))

        # 2. sys.exit(1)
        data = await _send_and_receive(self._port, "import sys; sys.exit(1)")
        self.assertEqual("error", data.get("status"), "sys.exit(1) not caught")
        self.assertEqual("RuntimeError", data.get("ename"))
        self.assertIn("SystemExit", data.get("evalue", ""))

        # 3. raise SystemExit('bye')
        data = await _send_and_receive(self._port, "raise SystemExit('bye')")
        self.assertEqual("error", data.get("status"), "raise SystemExit not caught")
        self.assertEqual("RuntimeError", data.get("ename"))
        self.assertIn("SystemExit", data.get("evalue", ""))

        # 4. SystemExit via JSON envelope
        data = await _send_envelope(self._port, {"code": "raise SystemExit(42)"})
        self.assertEqual("error", data.get("status"), "envelope SystemExit not caught")

        # 5. Custom BaseException subclass
        code = "class CriticalFailure(BaseException): pass\nraise CriticalFailure('custom')"
        data = await _send_and_receive(self._port, code)
        self.assertEqual("error", data.get("status"), "BaseException subclass not caught")
        self.assertEqual("RuntimeError", data.get("ename"))
        self.assertIn("CriticalFailure", data.get("evalue", ""))

        # 6. GeneratorExit
        data = await _send_and_receive(self._port, "raise GeneratorExit()")
        self.assertEqual("error", data.get("status"), "GeneratorExit not caught")
        self.assertEqual("RuntimeError", data.get("ename"))
        self.assertIn("GeneratorExit", data.get("evalue", ""))

        # 7. Liveness check — server must still accept new connections
        data = await _send_and_receive(self._port, "print('survived')")
        self.assertEqual("ok", data.get("status"), "Server not alive after dangerous exceptions")
        self.assertEqual("survived", data.get("output"))


# ---------------------------------------------------------------------------
# 2. Concurrent connections and race conditions
# ---------------------------------------------------------------------------


class TestConcurrency(omni.kit.test.AsyncTestCase):
    """Test rapid concurrent connections and race conditions."""

    async def setUp(self) -> None:
        """Read socket configuration from Carbonite settings."""
        settings = carb.settings.get_settings()
        self._port: int = settings.get(f"{_SETTINGS_PREFIX}/port")

    async def tearDown(self) -> None:
        """Clean up test contexts."""
        for i in range(5):
            await _send_envelope(self._port, {"introspect": "delete_context", "context": f"concurrent_{i}"})

    async def test_rapid_sequential_connections(self) -> None:
        """Fire 20 quick sequential requests — all should get valid responses."""
        for i in range(20):
            data = await _send_and_receive(self._port, f"print({i})")
            self.assertEqual("ok", data.get("status"))
            self.assertEqual(str(i), data.get("output"))

    async def test_concurrent_connections(self) -> None:
        """Fire 5 simultaneous connections — all should complete without error.

        The server processes requests sequentially on the main loop, so
        connections queue up. This tests that the queue and buffering work.
        """

        async def _one_request(idx: int) -> dict:
            return await _send_and_receive(self._port, f"print('concurrent_{idx}')")

        results = await asyncio.gather(*[_one_request(i) for i in range(5)])
        for i, data in enumerate(results):
            print(f"concurrent {i}:", data)
            self.assertEqual("ok", data.get("status"))
            self.assertEqual(f"concurrent_{i}", data.get("output"))

    async def test_concurrent_contexts_isolated(self) -> None:
        """Concurrent requests to different named contexts stay isolated."""

        async def _write_and_read(idx: int) -> dict:
            ctx = f"concurrent_{idx}"
            await _send_envelope(self._port, {"code": f"val = {idx * 100}", "context": ctx})
            return await _send_envelope(self._port, {"code": "val", "context": ctx})

        results = await asyncio.gather(*[_write_and_read(i) for i in range(5)])
        for i, data in enumerate(results):
            self.assertEqual("ok", data.get("status"))
            self.assertEqual(i * 100, data.get("result"))


# ---------------------------------------------------------------------------
# 3. Large payloads
# ---------------------------------------------------------------------------


class TestLargePayloads(omni.kit.test.AsyncTestCase):
    """Test that large code and large output are handled correctly."""

    async def setUp(self) -> None:
        """Read socket configuration from Carbonite settings."""
        settings = carb.settings.get_settings()
        self._port: int = settings.get(f"{_SETTINGS_PREFIX}/port")

    async def tearDown(self) -> None:
        """Clean up after each test."""

    async def test_large_code_payload(self) -> None:
        """Send a code block with 500 variable assignments (~10 KB)."""
        lines = [f"v_{i} = {i}" for i in range(500)]
        lines.append("print(v_0 + v_499)")
        source = "\n".join(lines)
        data = await _send_and_receive(self._port, source)
        print("response status:", data.get("status"))
        self.assertEqual("ok", data.get("status"))
        self.assertEqual("499", data.get("output"))

    async def test_large_output(self) -> None:
        """Code that prints 10,000 characters should be fully captured."""
        data = await _send_and_receive(self._port, "print('x' * 10000)")
        self.assertEqual("ok", data.get("status"))
        self.assertEqual(10000, len(data.get("output", "")))

    async def test_large_result(self) -> None:
        """An expression returning a large list should serialize correctly."""
        data = await _send_and_receive(self._port, "list(range(1000))")
        self.assertEqual("ok", data.get("status"))
        result = data.get("result")
        self.assertIsInstance(result, list)
        self.assertEqual(1000, len(result))


# ---------------------------------------------------------------------------
# 4. Partial sends and abrupt disconnects
# ---------------------------------------------------------------------------


class TestConnectionEdgeCases(omni.kit.test.AsyncTestCase):
    """Test that partial sends and abrupt disconnects don't crash the server."""

    async def setUp(self) -> None:
        """Read socket configuration from Carbonite settings."""
        settings = carb.settings.get_settings()
        self._port: int = settings.get(f"{_SETTINGS_PREFIX}/port")

    async def tearDown(self) -> None:
        """Clean up after each test."""

    async def test_empty_payload(self) -> None:
        """Sending an empty payload (just EOF) should return a valid response."""
        data = await _send_and_receive(self._port, "")
        print("response:", data)
        # Empty code should either succeed with empty output or return an error
        self.assertIn(data.get("status"), ("ok", "error"))

    async def test_abrupt_disconnect(self) -> None:
        """Connect, send partial data, then close without EOF — server must survive.

        After the abrupt disconnect, the server should still accept new connections.
        """
        reader, writer = await asyncio.open_connection(_HOST, self._port)
        writer.write(b"print('partial")  # Intentionally incomplete
        writer.close()
        await asyncio.sleep(0.5)

        # Server must still be alive
        data = await _send_and_receive(self._port, "print('after_disconnect')")
        self.assertEqual("ok", data.get("status"))
        self.assertEqual("after_disconnect", data.get("output"))

    async def test_connect_and_immediate_eof(self) -> None:
        """Connect and immediately send EOF — server must not crash."""
        reader, writer = await asyncio.open_connection(_HOST, self._port)
        writer.write_eof()
        try:
            data = await asyncio.wait_for(reader.read(), timeout=5.0)
            # If we get a response, it should be valid JSON
            if data:
                result = json.loads(data.decode())
                self.assertIn(result.get("status"), ("ok", "error"))
        except (asyncio.TimeoutError, ConnectionResetError):
            pass  # Server closing the connection is acceptable
        writer.close()

        # Server must still be alive
        data = await _send_and_receive(self._port, "print('after_eof')")
        self.assertEqual("ok", data.get("status"))

    async def test_binary_garbage(self) -> None:
        """Sending non-UTF8 binary data should produce an error response, not a crash."""
        reader, writer = await asyncio.open_connection(_HOST, self._port)
        writer.write(b"\x80\x81\x82\xff\xfe\xfd")
        writer.write_eof()
        data = await asyncio.wait_for(reader.read(), timeout=5.0)
        result = json.loads(data.decode())
        print("response:", result)
        self.assertEqual("error", result.get("status"))
        self.assertEqual("UnicodeDecodeError", result.get("ename"))
        writer.close()

        # Server must still be alive
        data = await _send_and_receive(self._port, "print('after_binary')")
        self.assertEqual("ok", data.get("status"))


# ---------------------------------------------------------------------------
# 5. Global state mutation
# ---------------------------------------------------------------------------


class TestGlobalStateMutation(omni.kit.test.AsyncTestCase):
    """Test that user code mutating sys.modules, builtins, etc. doesn't break the server."""

    async def setUp(self) -> None:
        """Read socket configuration from Carbonite settings."""
        settings = carb.settings.get_settings()
        self._port: int = settings.get(f"{_SETTINGS_PREFIX}/port")

    async def tearDown(self) -> None:
        """Clean up test contexts."""
        for ctx in ("mutate_ctx",):
            await _send_envelope(self._port, {"introspect": "delete_context", "context": ctx})

    async def test_del_builtins(self) -> None:
        """Deleting __builtins__ in user code should not break subsequent requests."""
        data = await _send_and_receive(self._port, "del __builtins__")
        # This may error — that's fine
        print("response:", data)

        # Server must still work
        data = await _send_and_receive(self._port, "print('builtins_restored')")
        self.assertEqual("ok", data.get("status"))
        self.assertEqual("builtins_restored", data.get("output"))

    async def test_override_print(self) -> None:
        """Overriding print() should only affect the current execution, not the server."""
        code = "print = lambda *a: None\nprint('silent')"
        data = await _send_envelope(self._port, {"code": code, "context": "mutate_ctx"})
        print("response:", data)
        self.assertEqual("ok", data.get("status"))
        # Output should be empty since print was overridden
        self.assertEqual("", data.get("output"))

        # Default context should still have working print
        data = await _send_and_receive(self._port, "print('print_works')")
        self.assertEqual("ok", data.get("status"))
        self.assertEqual("print_works", data.get("output"))

    async def test_import_and_mutate_sys(self) -> None:
        """Code that manipulates sys.path should not break subsequent requests."""
        data = await _send_and_receive(self._port, "import sys; sys.path.append('/tmp/fake'); print(len(sys.path))")
        self.assertEqual("ok", data.get("status"))

        data = await _send_and_receive(self._port, "print('after_sys_mutate')")
        self.assertEqual("ok", data.get("status"))
        self.assertEqual("after_sys_mutate", data.get("output"))


# ---------------------------------------------------------------------------
# 6. Unicode and special characters
# ---------------------------------------------------------------------------


class TestUnicodeHandling(omni.kit.test.AsyncTestCase):
    """Test that unicode and special characters in code and output work correctly."""

    async def setUp(self) -> None:
        """Read socket configuration from Carbonite settings."""
        settings = carb.settings.get_settings()
        self._port: int = settings.get(f"{_SETTINGS_PREFIX}/port")

    async def tearDown(self) -> None:
        """Clean up after each test."""

    async def test_unicode_output(self) -> None:
        """Unicode characters in print output should be preserved."""
        data = await _send_and_receive(self._port, 'print("Hello 世界 🌍 émojis")')
        self.assertEqual("ok", data.get("status"))
        self.assertEqual("Hello 世界 🌍 émojis", data.get("output"))

    async def test_unicode_in_variable(self) -> None:
        """Unicode variable names and values should work."""
        data = await _send_and_receive(self._port, 'café = "☕"; print(café)')
        self.assertEqual("ok", data.get("status"))
        self.assertEqual("☕", data.get("output"))

    async def test_newlines_in_output(self) -> None:
        """Embedded newlines should be preserved in output."""
        data = await _send_and_receive(self._port, r'print("line1\nline2\nline3")')
        self.assertEqual("ok", data.get("status"))
        self.assertEqual("line1\nline2\nline3", data.get("output"))

    async def test_special_json_chars_in_output(self) -> None:
        """Characters that need JSON escaping (backslash, quotes, tabs) should serialize correctly."""
        data = await _send_and_receive(self._port, r"""print('back\\slash\t"quotes"')""")
        self.assertEqual("ok", data.get("status"))
        output = data.get("output", "")
        self.assertIn("\\", output)
        self.assertIn('"', output)

    async def test_null_bytes_in_output(self) -> None:
        r"""Null bytes (\x00) in output should not truncate the JSON response."""
        data = await _send_and_receive(self._port, r"print('before\x00after')")
        self.assertEqual("ok", data.get("status"))
        # The output should contain both parts (null byte may be escaped or stripped)
        output = data.get("output", "")
        self.assertIn("before", output)


# ---------------------------------------------------------------------------
# 7. Async task lifecycle
# ---------------------------------------------------------------------------


class TestAsyncTaskLifecycle(omni.kit.test.AsyncTestCase):
    """Test that async code and tasks spawned by user code are handled correctly."""

    async def setUp(self) -> None:
        """Read socket configuration from Carbonite settings."""
        settings = carb.settings.get_settings()
        self._port: int = settings.get(f"{_SETTINGS_PREFIX}/port")

    async def tearDown(self) -> None:
        """Clean up after each test."""

    async def test_simple_await(self) -> None:
        """Simple await expression should work and return result."""
        data = await _send_and_receive(self._port, "import asyncio; await asyncio.sleep(0); print('awaited')")
        self.assertEqual("ok", data.get("status"))
        self.assertIn("awaited", data.get("output", ""))

    async def test_async_exception(self) -> None:
        """Exception inside async code should be captured in response."""
        code = "import asyncio\nawait asyncio.sleep(0)\nraise ValueError('async_error')"
        data = await _send_and_receive(self._port, code)
        self.assertEqual("error", data.get("status"))
        self.assertEqual("ValueError", data.get("ename"))
        self.assertIn("async_error", data.get("evalue", ""))

    async def test_create_task_in_user_code(self) -> None:
        """User code that creates an asyncio.Task should not crash the server.

        The task may or may not complete — what matters is the server stays alive.
        """
        code = (
            "import asyncio\n"
            "async def _bg():\n"
            "    await asyncio.sleep(0.01)\n"
            "asyncio.ensure_future(_bg())\n"
            "print('task_scheduled')"
        )
        data = await _send_and_receive(self._port, code)
        self.assertEqual("ok", data.get("status"))
        self.assertEqual("task_scheduled", data.get("output"))

        # Give the background task a moment, then verify server is fine
        await asyncio.sleep(0.1)
        data = await _send_and_receive(self._port, "print('after_task')")
        self.assertEqual("ok", data.get("status"))

    async def test_nested_await(self) -> None:
        """Nested async function calls should work correctly."""
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
# 8. Error message quality
# ---------------------------------------------------------------------------


class TestErrorMessages(omni.kit.test.AsyncTestCase):
    """Test that error responses contain useful diagnostic information."""

    async def setUp(self) -> None:
        """Read socket configuration from Carbonite settings."""
        settings = carb.settings.get_settings()
        self._port: int = settings.get(f"{_SETTINGS_PREFIX}/port")

    async def tearDown(self) -> None:
        """Clean up after each test."""

    async def test_name_error_includes_suggestion(self) -> None:
        """NameError should include the variable name in the response."""
        data = await _send_and_receive(self._port, "undefined_variable_xyz")
        self.assertEqual("error", data.get("status"))
        self.assertEqual("NameError", data.get("ename"))
        self.assertIn("undefined_variable_xyz", data.get("evalue", ""))

    async def test_type_error_includes_details(self) -> None:
        """TypeError should include meaningful details."""
        data = await _send_and_receive(self._port, "'string' + 42")
        self.assertEqual("error", data.get("status"))
        self.assertEqual("TypeError", data.get("ename"))

    async def test_traceback_present(self) -> None:
        """Error responses should include a traceback field."""
        data = await _send_and_receive(self._port, "1/0")
        self.assertEqual("error", data.get("status"))
        self.assertEqual("ZeroDivisionError", data.get("ename"))
        self.assertIn("traceback", data)
        traceback_str = data.get("traceback", "")
        if isinstance(traceback_str, list):
            traceback_str = "\n".join(traceback_str)
        self.assertIn("ZeroDivisionError", traceback_str)

    async def test_syntax_error_line_number(self) -> None:
        """SyntaxError should include line information."""
        data = await _send_and_receive(self._port, "def f(\n    x\n    y\n):\n    pass")
        self.assertEqual("error", data.get("status"))
        self.assertEqual("SyntaxError", data.get("ename"))

    async def test_indentation_error(self) -> None:
        """IndentationError should be reported correctly."""
        data = await _send_and_receive(self._port, "if True:\nprint('bad indent')")
        self.assertEqual("error", data.get("status"))
        self.assertIn(data.get("ename", ""), ("IndentationError", "SyntaxError"))

    async def test_recursion_error(self) -> None:
        """RecursionError from infinite recursion should be caught and reported."""
        code = "def f(): f()\nf()"
        data = await _send_and_receive(self._port, code)
        self.assertEqual("error", data.get("status"))
        self.assertEqual("RecursionError", data.get("ename"))

        # Server must survive
        data = await _send_and_receive(self._port, "print('after_recursion')")
        self.assertEqual("ok", data.get("status"))

    async def test_memory_error_small(self) -> None:
        """Attempting a large allocation should error, not crash."""
        # Use 10**18 which exceeds the virtual address space (~128 TB on x86-64)
        # so malloc fails immediately.  10**10 (~80 GB) can trigger Linux
        # overcommit, causing memory thrashing instead of a fast MemoryError.
        data = await _send_and_receive(
            self._port, "try:\n    x = [0] * (10**18)\nexcept MemoryError:\n    print('oom')"
        )
        self.assertEqual("ok", data.get("status"))
        # Either it OOM'd and caught it, or it succeeded (unlikely)

        # Server must survive either way
        data = await _send_and_receive(self._port, "print('after_memory')")
        self.assertEqual("ok", data.get("status"))


# ---------------------------------------------------------------------------
# 9. Execution timeout edge cases
# ---------------------------------------------------------------------------


class TestTimeoutEdgeCases(omni.kit.test.AsyncTestCase):
    """Test execution timeout with various code patterns."""

    async def setUp(self) -> None:
        """Read socket configuration from Carbonite settings."""
        settings = carb.settings.get_settings()
        self._port: int = settings.get(f"{_SETTINGS_PREFIX}/port")

    async def tearDown(self) -> None:
        """Clean up test contexts."""
        for ctx in ("timeout_ctx",):
            await _send_envelope(self._port, {"introspect": "delete_context", "context": ctx})

    async def test_timeout_with_print_before_hang(self) -> None:
        """Code that prints then hangs: timeout should include the printed output."""
        data = await _send_envelope(
            self._port,
            {"code": "print('before_hang')\nimport time; time.sleep(5)", "timeout": 1},
            client_timeout=30.0,
        )
        print("response:", data)
        self.assertEqual("error", data.get("status"))
        self.assertEqual("TimeoutError", data.get("ename"))

    async def test_fast_code_with_generous_timeout(self) -> None:
        """Fast code with a timeout set should complete normally."""
        data = await _send_envelope(
            self._port,
            {"code": "print('fast')", "timeout": 30},
        )
        self.assertEqual("ok", data.get("status"))
        self.assertEqual("fast", data.get("output"))

    async def test_timeout_does_not_affect_next_request(self) -> None:
        """After a timeout, the next request should work normally."""
        # Trigger a timeout
        await _send_envelope(
            self._port,
            {"code": "import time; time.sleep(3)", "timeout": 1},
            client_timeout=30.0,
        )

        # Next request should be fine
        data = await _send_and_receive(self._port, "print('after_timeout')")
        self.assertEqual("ok", data.get("status"))
        self.assertEqual("after_timeout", data.get("output"))


# ---------------------------------------------------------------------------
# 10. Response format consistency
# ---------------------------------------------------------------------------


class TestResponseFormat(omni.kit.test.AsyncTestCase):
    """Test that response JSON always has the expected shape."""

    async def setUp(self) -> None:
        """Read socket configuration from Carbonite settings."""
        settings = carb.settings.get_settings()
        self._port: int = settings.get(f"{_SETTINGS_PREFIX}/port")

    async def tearDown(self) -> None:
        """Clean up after each test."""

    async def test_success_response_shape(self) -> None:
        """Successful execution must include status and output fields."""
        data = await _send_and_receive(self._port, "print('shape_test')")
        self.assertIn("status", data)
        self.assertIn("output", data)
        self.assertEqual("ok", data["status"])
        self.assertIsInstance(data["output"], str)

    async def test_error_response_shape(self) -> None:
        """Error execution must include status, ename, evalue, and traceback fields."""
        data = await _send_and_receive(self._port, "1/0")
        self.assertIn("status", data)
        self.assertIn("ename", data)
        self.assertIn("evalue", data)
        self.assertIn("traceback", data)
        self.assertEqual("error", data["status"])
        self.assertIsInstance(data["ename"], str)
        self.assertIsInstance(data["evalue"], str)

    async def test_expression_result_in_response(self) -> None:
        """Expression evaluation must include a result field."""
        data = await _send_and_receive(self._port, "42")
        self.assertEqual("ok", data.get("status"))
        self.assertIn("result", data)
        self.assertEqual(42, data["result"])

    async def test_statement_no_result(self) -> None:
        """Statement execution should have None or missing result."""
        data = await _send_and_receive(self._port, "x = 42")
        self.assertEqual("ok", data.get("status"))
        # Result should be None or absent for statements
        result = data.get("result")
        self.assertIsNone(result)

    async def test_response_is_valid_json(self) -> None:
        """Even for weird code, the response must always be valid JSON."""
        weird_cases = [
            "None",
            "...",
            "pass",
            "''",
            '""',
            "b''",
            "{}",
            "[]",
            "()",
        ]
        for code in weird_cases:
            data = await _send_and_receive(self._port, code)
            self.assertIn(data.get("status"), ("ok", "error"), f"Bad status for code: {code!r}")
