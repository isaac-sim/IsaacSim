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

"""Test JSON envelope protocol, named contexts, timeouts, fire-and-forget, and introspection."""

from __future__ import annotations

import asyncio
import json

import carb
import omni.kit.test

from ._auth import add_auth_header, add_auth_to_envelope

_SETTINGS_PREFIX = "/exts/isaacsim.code_editor.python_server"
_HOST = "127.0.0.1"


async def _send_and_receive(port: int, source: str, client_timeout: float = 30.0) -> dict:
    """Send *source* to the server and return the parsed JSON response.

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
    """Serialise *envelope* as JSON, send it, and return the parsed response.

    Args:
        port: The TCP port to connect to.
        envelope: The request envelope dict.
        client_timeout: Maximum seconds to wait for a response.

    Returns:
        The parsed JSON response dictionary.
    """
    return await _send_and_receive(port, json.dumps(add_auth_to_envelope(envelope)), client_timeout)


class TestJsonEnvelope(omni.kit.test.AsyncTestCase):
    """Test JSON envelope protocol features."""

    async def setUp(self) -> None:
        """Read socket configuration from Carbonite settings."""
        settings = carb.settings.get_settings()
        self._port: int = settings.get(f"{_SETTINGS_PREFIX}/port")

    async def tearDown(self) -> None:
        """Clean up test contexts."""
        for ctx in ("args_test_ctx", "envelope_empty_code_ctx"):
            await _send_envelope(self._port, {"introspect": "delete_context", "context": ctx})

    async def test_json_envelope_basic(self) -> None:
        """Verify that a JSON envelope with a code field executes and returns output."""
        data = await _send_envelope(self._port, {"code": 'print("envelope_hello")'})
        print("response:", data)
        self.assertEqual("ok", data.get("status"))
        self.assertEqual("envelope_hello", data.get("output"))

    async def test_json_envelope_with_args(self) -> None:
        """Verify that args dict values are injected into the execution namespace."""
        data = await _send_envelope(
            self._port,
            {
                "code": "print(env_injected_val)",
                "args": {"env_injected_val": "hello_from_args"},
                "context": "args_test_ctx",
            },
        )
        print("response:", data)
        self.assertEqual("ok", data.get("status"))
        self.assertEqual("hello_from_args", data.get("output"))

    async def test_json_envelope_with_args_types(self) -> None:
        """Verify that args preserve Python types (int, float, list, dict, None)."""
        data = await _send_envelope(
            self._port,
            {
                "code": "import json\nprint(json.dumps([type(a).__name__, type(b).__name__, type(c).__name__, type(d).__name__, type(e).__name__]))",
                "args": {"a": 42, "b": 3.14, "c": [1, 2], "d": {"k": "v"}, "e": None},
                "context": "args_test_ctx",
            },
        )
        print("response:", data)
        self.assertEqual("ok", data.get("status"))
        types = json.loads(data.get("output", "[]"))
        self.assertEqual(["int", "float", "list", "dict", "NoneType"], types)

    async def test_json_envelope_fallback(self) -> None:
        """Verify that raw Python code (not JSON) still executes correctly."""
        data = await _send_and_receive(self._port, 'print("raw_fallback")')
        print("response:", data)
        self.assertEqual("ok", data.get("status"))
        self.assertEqual("raw_fallback", data.get("output"))

    async def test_json_envelope_invalid_json(self) -> None:
        """Verify that a '{'-prefixed string that is not valid JSON falls back to raw execution.

        ``{invalid`` starts with ``{``, fails JSON parsing, so the server
        treats it as raw Python — an unclosed brace that produces a SyntaxError.
        """
        data = await _send_and_receive(self._port, "{invalid")
        print("response:", data)
        self.assertEqual("error", data.get("status"))
        self.assertEqual("SyntaxError", data.get("ename"))

    async def test_json_envelope_empty_code(self) -> None:
        """Verify that an envelope with empty code executes without error."""
        data = await _send_envelope(self._port, {"code": "", "context": "envelope_empty_code_ctx"})
        print("response:", data)
        self.assertEqual("ok", data.get("status"))
        self.assertEqual("", data.get("output"))


class TestNamedContexts(omni.kit.test.AsyncTestCase):
    """Test named execution context isolation and persistence."""

    async def setUp(self) -> None:
        """Read socket configuration from Carbonite settings."""
        settings = carb.settings.get_settings()
        self._port: int = settings.get(f"{_SETTINGS_PREFIX}/port")

    async def tearDown(self) -> None:
        """Clean up test contexts via introspection."""
        for ctx in ("ctx_iso_A", "ctx_iso_B", "persist_ctx", "default_compat_ctx", "seed_test_ctx"):
            await _send_envelope(self._port, {"introspect": "delete_context", "context": ctx})

    async def test_named_context_isolation(self) -> None:
        """Verify that variables in context A are not visible in context B."""
        await _send_envelope(self._port, {"code": "ctx_iso_var = 999", "context": "ctx_iso_A"})
        data = await _send_envelope(self._port, {"code": "ctx_iso_var", "context": "ctx_iso_B"})
        print("response:", data)
        self.assertEqual("error", data.get("status"))
        self.assertEqual("NameError", data.get("ename"))

    async def test_named_context_persistence(self) -> None:
        """Verify that a variable set in a named context persists across calls."""
        await _send_envelope(self._port, {"code": "persist_x = 42", "context": "persist_ctx"})
        data = await _send_envelope(self._port, {"code": "persist_x", "context": "persist_ctx"})
        print("response:", data)
        self.assertEqual("ok", data.get("status"))
        self.assertEqual(42, data.get("result"))

    async def test_default_context_backwards_compat(self) -> None:
        """Verify that omitting the context field uses the default context."""
        # JSON envelope without "context" key
        data = await _send_envelope(self._port, {"code": 'print("default_ctx_test")'})
        self.assertEqual("ok", data.get("status"))
        self.assertEqual("default_ctx_test", data.get("output"))

        # Raw protocol (no envelope at all) — also uses default context
        data = await _send_and_receive(self._port, 'print("raw_default_ctx_test")')
        self.assertEqual("ok", data.get("status"))
        self.assertEqual("raw_default_ctx_test", data.get("output"))

    async def test_named_context_clean_seed(self) -> None:
        """Verify that named contexts start with a clean namespace.

        Named contexts should have ``__builtins__`` (so ``print``, ``len`` etc. work)
        but should NOT contain extension-internal symbols like ``threading``, ``uuid``,
        ``OrderedDict``, or ``Extension``.
        """
        data = await _send_envelope(
            self._port,
            {
                "code": (
                    "import json\n"
                    "names = sorted(k for k in dir() if not k.startswith('__'))\n"
                    "has_builtins = '__builtins__' in dir()\n"
                    "can_print = callable(print)\n"
                    "print(json.dumps({'names': names, 'has_builtins': has_builtins, 'can_print': can_print}))"
                ),
                "context": "seed_test_ctx",
            },
        )
        print("response:", data)
        self.assertEqual("ok", data.get("status"))
        result = json.loads(data.get("output", "{}"))
        # Builtins should be available
        self.assertTrue(result.get("has_builtins"), "Named context should have __builtins__")
        self.assertTrue(result.get("can_print"), "print() should work in named contexts")
        # Extension internals should NOT be present
        internal_symbols = {"threading", "uuid", "OrderedDict", "Extension", "_SETTINGS_PREFIX", "_MAX_COMPLETED_TASKS"}
        leaked = internal_symbols & set(result.get("names", []))
        self.assertEqual(set(), leaked, f"Extension internals leaked into named context: {leaked}")

    async def test_named_context_builtins_work(self) -> None:
        """Verify that common builtins (print, len, range, import) work in named contexts."""
        data = await _send_envelope(
            self._port,
            {
                "code": "import json\nprint(json.dumps({'a': len(list(range(5)))}))",
                "context": "seed_test_ctx",
            },
        )
        print("response:", data)
        self.assertEqual("ok", data.get("status"))
        self.assertEqual('{"a": 5}', data.get("output"))


class TestExecutionTimeout(omni.kit.test.AsyncTestCase):
    """Test per-request execution timeouts for sync and async code."""

    async def setUp(self) -> None:
        """Read socket configuration from Carbonite settings."""
        settings = carb.settings.get_settings()
        self._port: int = settings.get(f"{_SETTINGS_PREFIX}/port")

    async def tearDown(self) -> None:
        """Clean up test contexts."""
        for ctx in ("no_timeout_ctx", "timeout_zero_ctx"):
            await _send_envelope(self._port, {"introspect": "delete_context", "context": ctx})

    async def test_execution_timeout(self) -> None:
        """Verify that sync code exceeding the timeout produces a TimeoutError response.

        The sync code sleeps for 3 s while the timeout is 1 s.  The event loop
        is blocked for ~3 s (sync code cannot be interrupted) but the watchdog
        timer fires from a background thread and queues the error reply.  The
        client receives it once the event loop resumes, so we need a generous
        client_timeout.
        """
        data = await _send_envelope(
            self._port,
            {"code": "import time; time.sleep(3)", "timeout": 1},
            client_timeout=30.0,
        )
        print("response:", data)
        self.assertEqual("error", data.get("status"))
        self.assertEqual("TimeoutError", data.get("ename"))
        self.assertIn("timed out", data.get("evalue", ""))

    async def test_execution_timeout_async(self) -> None:
        """Verify that async code exceeding the timeout produces a TimeoutError response.

        ``asyncio.wait_for`` cancels the coroutine after 1 s so this test
        completes quickly despite the 10 s sleep in the user code.
        """
        data = await _send_envelope(
            self._port,
            {"code": "import asyncio\nawait asyncio.sleep(10)", "timeout": 1},
            client_timeout=10.0,
        )
        print("response:", data)
        self.assertEqual("error", data.get("status"))
        self.assertEqual("TimeoutError", data.get("ename"))
        self.assertIn("timed out", data.get("evalue", ""))

    async def test_no_timeout_default(self) -> None:
        """Verify that code without a timeout setting runs to completion."""
        data = await _send_envelope(
            self._port,
            {"code": "import time; time.sleep(2)", "context": "no_timeout_ctx"},
            client_timeout=10.0,
        )
        print("response:", data)
        self.assertEqual("ok", data.get("status"))

    async def test_timeout_zero_overrides_global(self) -> None:
        """Verify that ``timeout: 0`` in the envelope means 'no timeout'.

        Even if the server had a global ``execution_timeout`` configured,
        an explicit ``timeout: 0`` in the envelope should disable the timeout
        for that request and let the code run to completion.
        """
        data = await _send_envelope(
            self._port,
            {
                "code": "import time; time.sleep(1); print('completed')",
                "timeout": 0,
                "context": "timeout_zero_ctx",
            },
            client_timeout=10.0,
        )
        print("response:", data)
        self.assertEqual("ok", data.get("status"))
        self.assertEqual("completed", data.get("output"))


class TestFireAndForget(omni.kit.test.AsyncTestCase):
    """Test fire-and-forget execution mode."""

    async def setUp(self) -> None:
        """Read socket configuration from Carbonite settings."""
        settings = carb.settings.get_settings()
        self._port: int = settings.get(f"{_SETTINGS_PREFIX}/port")

    async def tearDown(self) -> None:
        """Clean up test contexts."""
        for ctx in ("ff_error_ctx", "ff_async_ctx"):
            await _send_envelope(self._port, {"introspect": "delete_context", "context": ctx})

    async def test_fire_and_forget(self) -> None:
        """Verify that a fire-and-forget request receives an immediate ACK with a task_id."""
        data = await _send_envelope(
            self._port,
            {"code": "import time; time.sleep(0.05)", "fire_and_forget": True},
        )
        print("response:", data)
        self.assertEqual("ok", data.get("status"))
        self.assertTrue(data.get("fire_and_forget"))
        self.assertIsNotNone(data.get("task_id"))
        self.assertIsInstance(data.get("task_id"), str)
        self.assertEqual("", data.get("output"))

    async def test_fire_and_forget_result_query(self) -> None:
        """Verify that a completed fire-and-forget task result is retrievable via introspection."""
        ack = await _send_envelope(
            self._port,
            {"code": "ff_stored_result = 1 + 1", "fire_and_forget": True},
        )
        self.assertEqual("ok", ack.get("status"))
        task_id: str = ack.get("task_id", "")
        self.assertTrue(task_id)

        # Poll until the background task completes (should be near-instant for sync code)
        result = None
        for _ in range(20):
            await asyncio.sleep(0.1)
            data = await _send_envelope(self._port, {"introspect": "task", "task_id": task_id})
            if data.get("result") is not None:
                result = data.get("result")
                break

        print("task result:", result)
        self.assertIsNotNone(result, "Background task result should be available via introspection")
        self.assertEqual("ok", result.get("status"))

    async def test_fire_and_forget_error_stored(self) -> None:
        """Verify that a fire-and-forget task that raises an error stores the error result."""
        ack = await _send_envelope(
            self._port,
            {"code": "raise ValueError('ff_test_error')", "fire_and_forget": True, "context": "ff_error_ctx"},
        )
        self.assertEqual("ok", ack.get("status"))
        task_id: str = ack.get("task_id", "")
        self.assertTrue(task_id)

        # Poll until the background task completes
        result = None
        for _ in range(20):
            await asyncio.sleep(0.1)
            data = await _send_envelope(self._port, {"introspect": "task", "task_id": task_id})
            if data.get("result") is not None:
                result = data.get("result")
                break

        print("error task result:", result)
        self.assertIsNotNone(result, "Failed background task result should be stored")
        self.assertEqual("error", result.get("status"))
        self.assertEqual("ValueError", result.get("ename"))
        self.assertIn("ff_test_error", result.get("evalue", ""))

    async def test_fire_and_forget_nonexistent_task(self) -> None:
        """Verify that querying a nonexistent task_id returns null result."""
        data = await _send_envelope(self._port, {"introspect": "task", "task_id": "nonexistent-uuid"})
        print("response:", data)
        self.assertEqual("ok", data.get("status"))
        self.assertIsNone(data.get("result"))

    async def test_fire_and_forget_async_code(self) -> None:
        """Verify that fire-and-forget works with async (coroutine) code.

        This exercises the _await_and_store path which drives the coroutine
        to completion in the background. Previously, an indentation bug in the
        success branch caused an UnboundLocalError when async code completed
        without raising an exception.
        """
        code = "import asyncio; await asyncio.sleep(0.05); ff_async_result = 'async_done'"
        ack = await _send_envelope(
            self._port,
            {"code": code, "fire_and_forget": True, "context": "ff_async_ctx"},
        )
        self.assertEqual("ok", ack.get("status"))
        task_id: str = ack.get("task_id", "")
        self.assertTrue(task_id)

        # Poll until the background task completes
        result = None
        for _ in range(40):
            await asyncio.sleep(0.1)
            data = await _send_envelope(self._port, {"introspect": "task", "task_id": task_id})
            if data.get("result") is not None:
                result = data.get("result")
                break

        print("async fire-and-forget result:", result)
        self.assertIsNotNone(result, "Async fire-and-forget task result should be stored")
        self.assertEqual("ok", result.get("status"))

    async def test_fire_and_forget_async_error(self) -> None:
        """Verify that fire-and-forget stores errors from async code that raises."""
        code = "import asyncio; await asyncio.sleep(0.05); raise RuntimeError('ff_async_err')"
        ack = await _send_envelope(
            self._port,
            {"code": code, "fire_and_forget": True, "context": "ff_async_ctx"},
        )
        self.assertEqual("ok", ack.get("status"))
        task_id: str = ack.get("task_id", "")
        self.assertTrue(task_id)

        result = None
        for _ in range(40):
            await asyncio.sleep(0.1)
            data = await _send_envelope(self._port, {"introspect": "task", "task_id": task_id})
            if data.get("result") is not None:
                result = data.get("result")
                break

        print("async fire-and-forget error result:", result)
        self.assertIsNotNone(result, "Async fire-and-forget error result should be stored")
        self.assertEqual("error", result.get("status"))
        self.assertEqual("RuntimeError", result.get("ename"))
        self.assertIn("ff_async_err", result.get("evalue", ""))


class TestIntrospection(omni.kit.test.AsyncTestCase):
    """Test server introspection endpoints."""

    async def setUp(self) -> None:
        """Read socket configuration from Carbonite settings."""
        settings = carb.settings.get_settings()
        self._port: int = settings.get(f"{_SETTINGS_PREFIX}/port")

    async def tearDown(self) -> None:
        """Clean up test contexts."""
        for ctx in ("introspect_ctx_A", "introspect_ctx_B", "del_test_ctx", "ctx_vars_test"):
            await _send_envelope(self._port, {"introspect": "delete_context", "context": ctx})

    async def test_introspect_contexts(self) -> None:
        """Verify that introspect/contexts lists all created context names."""
        await _send_envelope(self._port, {"code": "introspect_A_var = 1", "context": "introspect_ctx_A"})
        await _send_envelope(self._port, {"code": "introspect_B_var = 2", "context": "introspect_ctx_B"})

        data = await _send_envelope(self._port, {"introspect": "contexts"})
        print("response:", data)
        self.assertEqual("ok", data.get("status"))
        result = data.get("result", {})
        self.assertIsInstance(result, dict)
        self.assertIn("introspect_ctx_A", result)
        self.assertIn("introspect_ctx_B", result)
        # Default context is always present
        self.assertIn("", result)

    async def test_introspect_status(self) -> None:
        """Verify that introspect/status returns valid server status fields."""
        data = await _send_envelope(self._port, {"introspect": "status"})
        print("response:", data)
        self.assertEqual("ok", data.get("status"))
        result = data.get("result", {})
        self.assertIsInstance(result, dict)
        self.assertGreater(result.get("uptime_seconds", -1), 0)
        self.assertGreaterEqual(result.get("active_connections", -1), 0)
        self.assertGreaterEqual(result.get("completed_tasks", -1), 0)
        self.assertIn("contexts", result)
        self.assertIn("", result["contexts"])  # Default context always present

    async def test_introspect_delete_context(self) -> None:
        """Verify that a named context can be deleted via introspection."""
        await _send_envelope(self._port, {"code": "del_test_x = 7", "context": "del_test_ctx"})

        # Confirm it exists
        data = await _send_envelope(self._port, {"introspect": "contexts"})
        self.assertIn("del_test_ctx", data.get("result", {}))

        # Delete it
        data = await _send_envelope(self._port, {"introspect": "delete_context", "context": "del_test_ctx"})
        print("delete response:", data)
        self.assertEqual("ok", data.get("status"))

        # Confirm it is gone
        data = await _send_envelope(self._port, {"introspect": "contexts"})
        self.assertNotIn("del_test_ctx", data.get("result", {}))

    async def test_introspect_delete_default_context_rejected(self) -> None:
        """Verify that deleting the default context is rejected."""
        data = await _send_envelope(self._port, {"introspect": "delete_context", "context": ""})
        print("response:", data)
        self.assertEqual("error", data.get("status"))
        self.assertIn("default", data.get("result", "").lower())

    async def test_introspect_unknown_command(self) -> None:
        """Verify that an unknown introspect command returns an error."""
        data = await _send_envelope(self._port, {"introspect": "nonexistent_command"})
        print("response:", data)
        self.assertEqual("error", data.get("status"))
        self.assertIn("nonexistent_command", data.get("result", ""))

    async def test_introspect_context_variables(self) -> None:
        """Verify that introspect/context lists variable names and their types."""
        await _send_envelope(
            self._port,
            {"code": "my_int = 42; my_str = 'hello'; my_list = [1, 2, 3]", "context": "ctx_vars_test"},
        )

        data = await _send_envelope(self._port, {"introspect": "context", "context": "ctx_vars_test"})
        print("response:", data)
        self.assertEqual("ok", data.get("status"))
        result = data.get("result", {})
        self.assertEqual("int", result.get("my_int"))
        self.assertEqual("str", result.get("my_str"))
        self.assertEqual("list", result.get("my_list"))

    async def test_introspect_nonexistent_context(self) -> None:
        """Verify that inspecting a nonexistent context returns an error."""
        data = await _send_envelope(self._port, {"introspect": "context", "context": "does_not_exist"})
        print("response:", data)
        self.assertEqual("error", data.get("status"))
        self.assertIn("not found", data.get("result", "").lower())
