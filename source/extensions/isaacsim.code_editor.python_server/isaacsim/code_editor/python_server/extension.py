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

"""TCP socket server extension for remote Python code execution in Isaac Sim."""

from __future__ import annotations

__all__ = []

import asyncio
import contextlib
import hmac
import io
import json
import secrets
import socket
import sys
import threading
import time
import traceback
import uuid
from collections import OrderedDict

import carb
import omni.ext

from .executor import ExecutionResult, Executor

_SETTINGS_PREFIX = "/exts/isaacsim.code_editor.python_server"
_AUTH_HEADER_PREFIX = "# isaacsim-python-server-token:"

#: Maximum number of completed fire-and-forget task results to retain (FIFO eviction).
_MAX_COMPLETED_TASKS = 100


def _drive_coroutine(coro: object, callback: object = None) -> None:
    """Drive a coroutine to completion without creating an asyncio Task.

    This avoids the ``RuntimeError: Cannot enter into task`` that occurs on
    Python 3.12+ when a coroutine running inside a Task yields to the event
    loop and other pending tasks try to wake up.  By stepping the coroutine
    manually via ``coro.send()`` and scheduling continuations with
    ``loop.call_soon()``, the coroutine never becomes the *current task*,
    so other tasks are free to run concurrently.

    Args:
        coro: The coroutine to drive.
        callback: Optional callable invoked with the coroutine's return value
            (or ``None`` on exception) when it finishes.
    """
    loop = _get_event_loop()

    def _step(value: object = None, exc: BaseException | None = None) -> None:
        try:
            if exc is not None:
                result = coro.throw(exc)
            else:
                result = coro.send(value)
        except StopIteration as stop:
            if callback is not None:
                callback(stop.value)
            return
        except BaseException:
            if callback is not None:
                callback(None)
            return

        if isinstance(result, asyncio.Future):
            result.add_done_callback(_on_future_done)
        else:
            # Yielded something else (e.g. None) — continue on next loop tick
            loop.call_soon(_step)

    def _on_future_done(fut: asyncio.Future) -> None:
        try:
            result = fut.result()
        except BaseException as e:
            loop.call_soon(_step, None, e)
        else:
            loop.call_soon(_step, result)

    loop.call_soon(_step)


def _get_event_loop() -> asyncio.AbstractEventLoop:
    """Retrieve the current event loop with a fallback for older Python runtimes.

    Returns:
        The current asyncio event loop.
    """
    try:
        return asyncio.get_event_loop()
    except RuntimeError:
        return asyncio.get_event_loop_policy().get_event_loop()


def _serialize_result(value: object) -> object:
    """Attempt to convert *value* into a JSON-native representation.

    Falls back to `repr` for objects that are not directly JSON-serializable.

    Args:
        value: The Python object to serialize.

    Returns:
        A JSON-compatible representation of *value*.
    """
    if value is None or isinstance(value, (bool, int, float, str)):
        return value
    if isinstance(value, (list, tuple)):
        return [_serialize_result(item) for item in value]
    if isinstance(value, dict):
        return {str(k): _serialize_result(v) for k, v in value.items()}
    return repr(value)


class Extension(omni.ext.IExt):
    """TCP socket server for remote Python code execution in Isaac Sim.

    The extension creates a TCP server that accepts Python source code from any
    client (VS Code, LLM agents, custom tools), executes it within Isaac Sim's
    Python environment, and returns structured JSON responses.

    Requests may be sent as raw Python source or as a JSON envelope that
    enables named execution contexts, per-request timeouts, fire-and-forget
    execution, and server introspection.

    Optionally, Carbonite log messages can be broadcast over UDP to connected
    clients.

    Configuration is managed through Carbonite settings under
    ``/exts/isaacsim.code_editor.python_server/``.
    """

    def on_startup(self, ext_id: str) -> None:
        """Initialize the TCP server and optional UDP log broadcaster.

        Args:
            ext_id: The extension identifier.
        """
        #: Monotonic timestamp recorded when the extension started.
        self._start_time: float = time.monotonic()

        # Named execution contexts; "" (empty string) is the default context and
        # is the shared default namespace.
        self._contexts: dict[str, dict] = {"": {**globals()}}

        # Completed fire-and-forget task results, keyed by task UUID.
        # Bounded to _MAX_COMPLETED_TASKS entries with FIFO eviction.
        self._completed_tasks: OrderedDict[str, dict] = OrderedDict()

        #: Number of currently active TCP connections.
        self._active_connections: int = 0

        settings = carb.settings.get_settings()
        self._socket_host: str = settings.get(f"{_SETTINGS_PREFIX}/host")
        self._socket_port: int = settings.get(f"{_SETTINGS_PREFIX}/port")
        self._require_auth: bool = bool(settings.get(f"{_SETTINGS_PREFIX}/require_auth"))
        self._auth_token: str = self._get_or_create_auth_token(settings) if self._require_auth else ""
        self._publish_carb_logs: bool = settings.get(f"{_SETTINGS_PREFIX}/carb_logs")
        self._execution_timeout: float = float(settings.get(f"{_SETTINGS_PREFIX}/execution_timeout") or 0.0)
        self._keepalive_interval: float = float(settings.get(f"{_SETTINGS_PREFIX}/keepalive_interval") or 0.0)

        if self._publish_carb_logs:
            self._logging = carb.logging.acquire_logging()
            self._logger_handle = self._logging.add_logger(self._carb_logging)
            self._logging_levels: dict[int, str] = {
                carb.logging.LEVEL_INFO: "Info",
                carb.logging.LEVEL_WARN: "Warning",
                carb.logging.LEVEL_ERROR: "Error",
                carb.logging.LEVEL_FATAL: "Fatal",
            }

            self._udp_server: socket.socket | None = None
            self._udp_clients: list[tuple[str, int]] = []
            self._udp_server_running = False
            threading.Thread(target=self._create_udp_socket).start()

        self._server: asyncio.AbstractServer | None = None
        _get_event_loop().create_task(self._create_socket())

    def on_shutdown(self) -> None:
        """Shut down the TCP server and clean up resources."""
        if self._server is not None:
            self._server.close()
            asyncio.run_coroutine_threadsafe(self._server.wait_closed(), _get_event_loop())
            self._server = None

        self._contexts.clear()
        self._completed_tasks.clear()

        if self._publish_carb_logs:
            self._logging.remove_logger(self._logger_handle)
            self._logging = None
            self._logger_handle = None
            self._udp_server = None
            self._udp_clients = []
            trial_count = 0
            while self._udp_server_running:
                time.sleep(0.1)
                trial_count += 1
                if trial_count > 10:
                    break
            self._udp_server_running = False

    # ------------------------------------------------------------------
    # UDP carb log broadcasting
    # ------------------------------------------------------------------

    def _carb_logging(self, source: str, level: int, filename: str, line_number: int, message: str) -> None:
        """Broadcast a carb log message to all registered UDP clients.

        Args:
            source: The source of the log message.
            level: The logging level.
            filename: The source filename.
            line_number: The line number in the source file.
            message: The log message content.
        """
        if level in self._logging_levels and self._udp_server and self._udp_clients:
            data = f"[{self._logging_levels[level]}][{source}] {message}".encode()
            for client in self._udp_clients:
                try:
                    self._udp_server.sendto(data, client)
                except Exception as exc:
                    carb.log_error(f"{exc} len:{len(data)}")

    def _create_udp_socket(self) -> None:
        """Create a UDP socket for broadcasting carb log messages."""
        self._udp_clients = []
        self._udp_server_running = True
        with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as server:
            try:
                server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
                server.bind((self._socket_host, self._socket_port))
                server.setblocking(False)
                server.settimeout(0.25)
            except Exception as exc:
                self._udp_server = None
                self._udp_clients = []
                carb.log_error(str(exc))
                self._udp_server_running = False
                return

            self._udp_server = server
            while self._udp_server:
                try:
                    _, addr = server.recvfrom(1024)
                    if addr not in self._udp_clients:
                        self._udp_clients.append(addr)
                except socket.timeout:
                    pass
                except Exception as exc:
                    carb.log_warn(f"UDP server error: {exc}")
                    break
            self._udp_server = None
            self._udp_clients = []
            self._udp_server_running = False

    # ------------------------------------------------------------------
    # Context management
    # ------------------------------------------------------------------

    def _get_context(self, name: str) -> dict:
        """Get or create a named execution context.

        Each context is an independent globals dict.  The default context
        (``name=""``) is created at startup with module-level globals for
        backwards compatibility.  Named contexts start with a minimal seed
        containing only ``__builtins__`` so user code has access to built-in
        functions without inheriting internal extension symbols.

        Args:
            name: Context name; empty string selects the default context.

        Returns:
            The globals dict for the named context.
        """
        if name not in self._contexts:
            self._contexts[name] = {"__builtins__": __builtins__}
        return self._contexts[name]

    def _store_completed_task(self, task_id: str, result: dict) -> None:
        """Store a completed task result, evicting the oldest entry if at capacity.

        Args:
            task_id: The unique task identifier.
            result: The JSON-serializable result dict.
        """
        if len(self._completed_tasks) >= _MAX_COMPLETED_TASKS:
            self._completed_tasks.popitem(last=False)
        self._completed_tasks[task_id] = result

    # ------------------------------------------------------------------
    # TCP code execution server
    # ------------------------------------------------------------------

    def _get_or_create_auth_token(self, settings: object) -> str:
        """Read the configured auth token or generate one for this settings store.

        Args:
            settings: The Carbonite settings interface used to read and store the token.

        Returns:
            The configured authentication token, or an empty string when authentication is disabled.
        """
        if not self._require_auth:
            return ""
        token = settings.get_as_string(f"{_SETTINGS_PREFIX}/auth_token")
        if not token:
            token = secrets.token_urlsafe(32)
            settings.set_string(f"{_SETTINGS_PREFIX}/auth_token", token)
        carb.log_info(f"Python server authentication token: {token}")
        print(f"Python server authentication token: {token}")
        return token

    async def _create_socket(self) -> None:
        """Create the async TCP server and begin accepting connections."""

        class _ServerProtocol(asyncio.Protocol):
            """Handle individual TCP connections from clients.

            Incoming data is buffered until the client signals EOF (half-close),
            ensuring that TCP-fragmented payloads are fully reassembled before
            execution.

            Args:
                parent: The owning Extension instance.
            """

            def __init__(self, parent: Extension) -> None:
                super().__init__()
                self._parent = parent
                self._buffer = bytearray()

            def connection_made(self, transport: asyncio.BaseTransport) -> None:
                carb.log_info(f"Connection from {transport.get_extra_info('peername')}")
                self.transport = transport
                self._parent._active_connections += 1

            def connection_lost(self, exc: Exception | None) -> None:
                self._parent._active_connections = max(0, self._parent._active_connections - 1)

            def data_received(self, data: bytes) -> None:
                self._buffer.extend(data)

            def eof_received(self) -> bool:
                try:
                    code = self._buffer.decode()
                except UnicodeDecodeError as exc:
                    # Non-UTF8 binary data — send an error response and close.
                    self._buffer.clear()
                    error_reply = json.dumps(
                        {
                            "status": "error",
                            "output": "",
                            "ename": "UnicodeDecodeError",
                            "evalue": str(exc),
                            "traceback": [],
                        }
                    )
                    self.transport.write(error_reply.encode())
                    self.transport.close()
                    return True
                self._buffer.clear()
                # Schedule execution outside the transport's _read_ready
                # callback so that its contextvars.Context is no longer
                # entered when user code pumps the event loop (e.g.
                # update_app).  This prevents "cannot enter context" errors
                # on Python 3.12+.
                loop = _get_event_loop()
                if loop is not None and loop.is_running():
                    loop.call_soon(self._parent._process_code, code, self.transport)
                else:
                    carb.log_warn("Event loop unavailable; dropping python_server command")
                return True

        try:
            self._server = await _get_event_loop().create_server(
                protocol_factory=lambda: _ServerProtocol(self),
                host=self._socket_host,
                port=self._socket_port,
                family=socket.AF_INET,
                reuse_port=None if sys.platform == "win32" else True,
            )
            carb.log_info(f"Serving at {self._socket_host}:{self._socket_port}")
            await self._server.start_serving()
        except Exception as exc:
            carb.log_error(str(exc))
            self._server = None

    def _strip_auth_header(self, source: str) -> tuple[str | None, str]:
        """Extract the optional raw-source token header from *source*.

        Args:
            source: The raw incoming string from the TCP connection.

        Returns:
            A ``(token, source)`` tuple where *token* is the header token if present,
            and *source* is the remaining Python source.
        """
        if not source.startswith(_AUTH_HEADER_PREFIX):
            return None, source
        header, separator, remainder = source.partition("\n")
        token = header[len(_AUTH_HEADER_PREFIX) :].strip()
        return token, remainder if separator else ""

    def _parse_envelope(self, source: str) -> tuple[str, dict]:
        """Parse the incoming request as a JSON envelope or raw Python code.

        If *source* starts with ``{`` and is valid JSON containing a ``dict``,
        the envelope is returned.  Otherwise the source is treated as raw Python
        and an empty envelope dict is returned (raw code fallback).

        Args:
            source: The raw incoming string from the TCP connection.

        Returns:
            A ``(code, envelope)`` tuple where *code* is the Python source to
            execute and *envelope* contains any extra request metadata.
        """
        header_token, source = self._strip_auth_header(source)
        if source.lstrip().startswith("{"):
            try:
                parsed = json.loads(source)
                if isinstance(parsed, dict):
                    if header_token is not None:
                        parsed.setdefault("auth_token", header_token)
                    return parsed.get("code", ""), parsed
            except json.JSONDecodeError:
                pass
        if header_token is not None:
            return source, {"auth_token": header_token}
        return source, {}

    def _is_authenticated(self, envelope: dict) -> bool:
        """Return whether the request envelope contains a valid authentication token.

        Args:
            envelope: The parsed request envelope.

        Returns:
            True if authentication is disabled or the envelope token is valid; otherwise False.
        """
        if not self._require_auth:
            return True
        token = envelope.get("auth_token")
        if not token or not isinstance(token, str):
            return False
        return hmac.compare_digest(token, self._auth_token)

    def _process_code(self, source: str, transport: asyncio.Transport) -> None:
        """Execute Python source and send a JSON reply back to the client.

        Parses the incoming *source* as a JSON envelope (if it starts with
        ``{``) or as raw Python code (raw code fallback).  Handles
        introspection requests, fire-and-forget mode, named contexts,
        per-request timeouts, and sync-code watchdog timers.

        For async code, the coroutine is driven to completion without creating
        an asyncio Task (see ``_drive_coroutine``).  For sync code, execution
        runs directly.  In both cases user code never runs inside a Task, which
        prevents ``RuntimeError: Cannot enter into task`` on Python 3.12+ when
        user code pumps the application event loop (e.g. ``update_app``).

        Args:
            source: The raw incoming string from the TCP connection.
            transport: The asyncio transport for sending the response.
        """
        try:
            self._process_code_inner(source, transport)
        except Exception as exc:
            carb.log_error(f"python_server internal error: {exc}")
            reply: dict[str, object] = {
                "status": "error",
                "output": "",
                "ename": type(exc).__name__,
                "evalue": f"Internal server error: {exc}",
                "traceback": [],
            }
            self._send_raw_reply(reply, transport)

    def _process_code_inner(self, source: str, transport: asyncio.Transport) -> None:
        """Inner implementation of ``_process_code``, wrapped by a safety-net handler.

        Args:
            source: The raw incoming string from the TCP connection.
            transport: The asyncio transport for sending the response.
        """
        code, envelope = self._parse_envelope(source)
        if not self._is_authenticated(envelope):
            self._send_raw_reply(
                {
                    "status": "error",
                    "output": "",
                    "ename": "AuthenticationError",
                    "evalue": "Missing or invalid authentication token",
                    "traceback": ["AuthenticationError: Missing or invalid authentication token"],
                },
                transport,
            )
            return

        # Introspection shortcut — no code execution needed
        if "introspect" in envelope:
            reply = self._handle_introspect(envelope)
            transport.write(json.dumps(reply, separators=(",", ":")).encode())
            transport.close()
            return

        context_name: str = envelope.get("context", "")
        raw_timeout = envelope.get("timeout")
        timeout: float = float(raw_timeout if raw_timeout is not None else (self._execution_timeout or 0.0))
        fire_and_forget: bool = bool(envelope.get("fire_and_forget", False))
        args: dict = envelope.get("args") or {}

        ctx_globals = self._get_context(context_name)
        if args:
            ctx_globals.update(args)

        if fire_and_forget:
            task_id = str(uuid.uuid4())
            ack: dict[str, object] = {
                "status": "ok",
                "output": "",
                "fire_and_forget": True,
                "task_id": task_id,
            }
            transport.write(json.dumps(ack, separators=(",", ":")).encode())
            transport.close()
            _get_event_loop().call_soon(self._execute_background, code, ctx_globals, task_id, timeout)
            return

        # Watchdog timer for sync-code timeout.
        # Threading.Timer fires from a background thread even while sync code
        # blocks the event loop, queuing the error reply via call_soon_threadsafe.
        # The sync code continues running but the client gets notified.
        timeout_sent = threading.Event()
        timer_handle: threading.Timer | None = None

        if timeout > 0:
            # Capture the loop reference NOW (main thread) — the Timer fires
            # in a background thread where _get_event_loop() may fail on
            # Python 3.12+ ("no current event loop in thread").
            loop = _get_event_loop()

            def _timeout_watchdog() -> None:
                timeout_sent.set()
                watchdog_reply: dict[str, object] = {
                    "status": "error",
                    "output": "",
                    "ename": "TimeoutError",
                    "evalue": f"Execution timed out after {timeout}s",
                    "traceback": [],
                }
                loop.call_soon_threadsafe(self._send_raw_reply, watchdog_reply, transport)

            timer_handle = threading.Timer(timeout, _timeout_watchdog)
            timer_handle.start()

        start_time = time.monotonic()
        executor = Executor(ctx_globals, ctx_globals)
        exec_result: ExecutionResult = executor.execute(code)
        elapsed = time.monotonic() - start_time

        if timer_handle is not None:
            timer_handle.cancel()

        if timeout_sent.is_set():
            # Watchdog already queued the error reply — discard the sync result
            if exec_result.exception is None and asyncio.iscoroutine(exec_result.result):
                exec_result.result.close()
            return

        if exec_result.exception is None and asyncio.iscoroutine(exec_result.result):
            remaining = max(0.0, timeout - elapsed) if timeout > 0 else 0.0
            _drive_coroutine(self._await_and_reply(exec_result, transport, remaining, start_time, timeout))
            return

        reply = self._build_reply_dict(exec_result)
        if self._keepalive_interval > 0 and elapsed >= self._keepalive_interval:
            reply["elapsed_seconds"] = elapsed
        self._send_raw_reply(reply, transport)

    def _execute_background(self, code: str, ctx_globals: dict, task_id: str, timeout: float) -> None:
        """Execute code in the background for a fire-and-forget request.

        Stores the result in ``self._completed_tasks`` when done.  If the
        compiled code is a coroutine, a Task is created to await it.

        Args:
            code: The Python source code to execute.
            ctx_globals: The execution namespace (named context globals).
            task_id: The unique identifier for this background task.
            timeout: Per-request execution timeout in seconds (0 = no limit).
        """
        executor = Executor(ctx_globals, ctx_globals)
        exec_result: ExecutionResult = executor.execute(code)

        if exec_result.exception is None and asyncio.iscoroutine(exec_result.result):
            _drive_coroutine(self._await_and_store(exec_result, task_id, timeout))
            return

        self._store_completed_task(task_id, self._build_reply_dict(exec_result))

    # ------------------------------------------------------------------
    # Introspection
    # ------------------------------------------------------------------

    def _handle_introspect(self, envelope: dict) -> dict[str, object]:
        """Handle an introspection request and return a JSON-serializable reply.

        Args:
            envelope: The parsed JSON envelope containing the ``introspect`` command.

        Returns:
            A reply dict with ``status`` and ``result`` fields.
        """
        command: str = envelope.get("introspect", "")

        if command == "contexts":
            return {
                "status": "ok",
                "result": {name: len(ctx) for name, ctx in self._contexts.items()},
            }

        if command == "context":
            name: str = envelope.get("context", "")
            if name not in self._contexts:
                return {"status": "error", "result": f"Context '{name}' not found"}
            return {
                "status": "ok",
                "result": {k: type(v).__name__ for k, v in self._contexts[name].items()},
            }

        if command == "tasks":
            completed = {tid: res.get("status") for tid, res in self._completed_tasks.items()}
            return {"status": "ok", "result": {"completed": completed}}

        if command == "task":
            task_id: str = envelope.get("task_id", "")
            result = self._completed_tasks.get(task_id)
            return {"status": "ok", "result": result}

        if command == "delete_context":
            name = envelope.get("context", "")
            if name == "":
                return {"status": "error", "result": "Cannot delete the default context"}
            if name in self._contexts:
                del self._contexts[name]
                return {"status": "ok", "result": f"Context '{name}' deleted"}
            return {"status": "ok", "result": f"Context '{name}' not found"}

        if command == "status":
            uptime = time.monotonic() - self._start_time
            return {
                "status": "ok",
                "result": {
                    "uptime_seconds": uptime,
                    "active_connections": self._active_connections,
                    "completed_tasks": len(self._completed_tasks),
                    "contexts": list(self._contexts.keys()),
                },
            }

        return {"status": "error", "result": f"Unknown introspect command: '{command}'"}

    # ------------------------------------------------------------------
    # Reply helpers
    # ------------------------------------------------------------------

    def _build_reply_dict(self, exec_result: ExecutionResult) -> dict[str, object]:
        """Build the JSON reply dict from an execution result.

        Args:
            exec_result: The completed execution result.

        Returns:
            A JSON-serializable reply dict.
        """
        output = exec_result.output
        if output.endswith("\n"):
            output = output[:-1]

        reply: dict[str, object] = {
            "status": "ok" if exec_result.exception is None else "error",
            "output": output,
        }

        if exec_result.exception is None and exec_result.is_expression:
            reply["result"] = _serialize_result(exec_result.result)

        if exec_result.exception is not None:
            reply["traceback"] = [exec_result.traceback_str]
            reply["ename"] = type(exec_result.exception).__name__
            reply["evalue"] = str(exec_result.exception)

        return reply

    def _send_raw_reply(self, reply: dict[str, object], transport: asyncio.Transport) -> None:
        """Serialize *reply* and send it over *transport*, then close the connection.

        Silently skips writing if the transport is already closing (e.g. when a
        watchdog timer fires after the connection was closed normally).

        Args:
            reply: The JSON-serializable reply dict.
            transport: The asyncio transport for sending the response.
        """
        if not transport.is_closing():
            transport.write(json.dumps(reply, separators=(",", ":")).encode())
            transport.close()

    async def _await_and_reply(
        self,
        exec_result: ExecutionResult,
        transport: asyncio.Transport,
        timeout: float = 0.0,
        start_time: float | None = None,
        requested_timeout: float | None = None,
    ) -> None:
        """Await a coroutine result produced by user code and send the reply.

        Stdout is redirected during the await so that ``print()`` calls inside
        the coroutine are captured in the JSON response ``output`` field.

        Args:
            exec_result: The execution result whose ``result`` is a coroutine.
            transport: The asyncio transport for sending the response.
            timeout: Maximum seconds to wait for the coroutine (0 = no limit).
                This may be less than *requested_timeout* when sync compilation
                consumed part of the budget.
            start_time: Monotonic timestamp from before sync compilation, used
                to calculate total elapsed time for keepalive tracking.
            requested_timeout: The original user-facing timeout value, used in
                the error message.  Falls back to *timeout* if not provided.
        """
        display_timeout = requested_timeout if requested_timeout is not None else timeout
        _start = start_time if start_time is not None else time.monotonic()
        coro = exec_result.result
        async_output = io.StringIO()

        # Use a threading.Timer for async timeout instead of asyncio.wait_for,
        # because _drive_coroutine runs outside a Task context and
        # asyncio.timeout() requires being inside a Task on Python 3.12+.
        timeout_fired = threading.Event()
        timer_handle: threading.Timer | None = None

        if timeout > 0:
            loop = _get_event_loop()

            def _async_timeout_watchdog() -> None:
                timeout_fired.set()
                combined_output = exec_result.output + async_output.getvalue()
                if combined_output.endswith("\n"):
                    combined_output = combined_output[:-1]
                reply: dict[str, object] = {
                    "status": "error",
                    "output": combined_output,
                    "ename": "TimeoutError",
                    "evalue": f"Execution timed out after {display_timeout}s",
                    "traceback": [],
                }
                loop.call_soon_threadsafe(self._send_raw_reply, reply, transport)

            timer_handle = threading.Timer(timeout, _async_timeout_watchdog)
            timer_handle.start()

        try:
            with contextlib.redirect_stdout(async_output):
                awaited = await coro
        except Exception as exc:
            if timer_handle is not None:
                timer_handle.cancel()
            if timeout_fired.is_set():
                return
            combined_output = exec_result.output + async_output.getvalue()
            exec_result = ExecutionResult(
                output=combined_output,
                exception=exc,
                traceback_str=traceback.format_exc(),
            )
        else:
            if timer_handle is not None:
                timer_handle.cancel()
            if timeout_fired.is_set():
                return
            combined_output = exec_result.output + async_output.getvalue()
            exec_result = ExecutionResult(output=combined_output, result=awaited)

        elapsed = time.monotonic() - _start
        reply = self._build_reply_dict(exec_result)
        if self._keepalive_interval > 0 and elapsed >= self._keepalive_interval:
            reply["elapsed_seconds"] = elapsed
        self._send_raw_reply(reply, transport)

    async def _await_and_store(
        self,
        exec_result: ExecutionResult,
        task_id: str,
        timeout: float = 0.0,
    ) -> None:
        """Await a coroutine from a fire-and-forget request and store the result.

        Args:
            exec_result: The execution result whose ``result`` is a coroutine.
            task_id: The unique identifier for this background task.
            timeout: Maximum seconds to wait for the coroutine (0 = no limit).
        """
        coro = exec_result.result
        async_output = io.StringIO()

        # Use threading.Timer for timeout (same as _await_and_reply) to avoid
        # asyncio.wait_for which requires a Task context on Python 3.12+.
        timeout_fired = threading.Event()
        timer_handle: threading.Timer | None = None

        if timeout > 0:

            def _ff_timeout_watchdog() -> None:
                timeout_fired.set()

            timer_handle = threading.Timer(timeout, _ff_timeout_watchdog)
            timer_handle.start()

        try:
            with contextlib.redirect_stdout(async_output):
                awaited = await coro
        except Exception as exc:
            if timer_handle is not None:
                timer_handle.cancel()
            if timeout_fired.is_set():
                combined_output = exec_result.output + async_output.getvalue()
                if combined_output.endswith("\n"):
                    combined_output = combined_output[:-1]
                result: dict[str, object] = {
                    "status": "error",
                    "output": combined_output,
                    "ename": "TimeoutError",
                    "evalue": f"Execution timed out after {timeout}s",
                    "traceback": [],
                }
            else:
                combined_output = exec_result.output + async_output.getvalue()
                inner = ExecutionResult(
                    output=combined_output,
                    exception=exc,
                    traceback_str=traceback.format_exc(),
                )
                result = self._build_reply_dict(inner)
        else:
            if timer_handle is not None:
                timer_handle.cancel()
            if timeout_fired.is_set():
                combined_output = exec_result.output + async_output.getvalue()
                if combined_output.endswith("\n"):
                    combined_output = combined_output[:-1]
                result = {
                    "status": "error",
                    "output": combined_output,
                    "ename": "TimeoutError",
                    "evalue": f"Execution timed out after {timeout}s",
                    "traceback": [],
                }
            else:
                combined_output = exec_result.output + async_output.getvalue()
                inner = ExecutionResult(output=combined_output, result=awaited)
                result = self._build_reply_dict(inner)

        self._store_completed_task(task_id, result)
