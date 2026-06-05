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

"""Provide a Python code executor for running statements and expressions within Isaac Sim."""

from __future__ import annotations  # isort: skip
import __future__ as _future_module  # isort: skip

import contextlib
import dis
import io
import traceback
from dataclasses import dataclass, field

try:
    from ast import PyCF_ALLOW_TOP_LEVEL_AWAIT
except ImportError:
    PyCF_ALLOW_TOP_LEVEL_AWAIT = 0

_SENTINEL = object()


def _safe_getvalue(sio: io.StringIO) -> str:
    """Read captured output, returning an empty string on failure."""
    try:
        return sio.getvalue()
    except Exception:
        return ""


def _safe_format_exc(exc: BaseException) -> str:
    """Format the current traceback, falling back to a minimal representation.

    After RecursionError or MemoryError the normal formatting helpers may
    themselves raise; this wrapper ensures we always return *something*.
    """
    try:
        return traceback.format_exc()
    except Exception:
        try:
            return f"{type(exc).__name__}: {exc}\n"
        except Exception:
            return "Traceback unavailable\n"


@dataclass
class ExecutionResult:
    """Hold the outcome of a single code execution.

    Attributes are populated by `Executor.execute` and consumed by the server
    to build the JSON reply.
    """

    output: str = ""
    #: Captured standard output.

    result: object = field(default=_SENTINEL)
    #: Evaluated expression value, or `_SENTINEL` when the code was a statement.

    exception: Exception | None = None
    #: Exception raised during execution, if any.

    traceback_str: str = ""
    #: Formatted traceback string when an exception occurred.

    @property
    def is_expression(self) -> bool:
        """Return whether the executed code was an expression with a value."""
        return self.result is not _SENTINEL


class Executor:
    """Execute Python statements or expressions from strings.

    Args:
        globals: Global namespace for code execution.
        locals: Local namespace for code execution.
    """

    def __init__(self, globals: dict | None = None, locals: dict | None = None) -> None:
        self._globals: dict = globals if globals is not None else {}
        self._locals: dict = locals if locals is not None else {}
        self._compiler_flags = self._get_compiler_flags()
        self._coroutine_flag = self._get_coroutine_flag()

    def _get_compiler_flags(self) -> int:
        """Collect current Python compiler flags including future features.

        Returns:
            The combined compiler flags.
        """
        flags = 0
        for value in globals().values():
            try:
                if isinstance(value, _future_module._Feature):
                    flags |= value.compiler_flag
            except BaseException:
                pass
        return flags | PyCF_ALLOW_TOP_LEVEL_AWAIT

    def _get_coroutine_flag(self) -> int:
        """Look up the compiler flag value for coroutines.

        Returns:
            The coroutine compiler flag, or -1 if not found.
        """
        for k, v in dis.COMPILER_FLAG_NAMES.items():
            if v == "COROUTINE":
                return k
        return -1

    def execute(self, source: str) -> ExecutionResult:
        """Execute *source* in the configured Python scope.

        The method first attempts to compile the source as an expression
        (``eval`` mode).  If that fails with a `SyntaxError`, it falls back to
        ``exec`` mode.  For expressions the evaluated value is captured in
        `ExecutionResult.result`.

        When the compiled code is a coroutine (contains top-level ``await``),
        the unawaited coroutine is stored in ``result`` so the caller can
        schedule it on the event loop.  This avoids running user code inside an
        asyncio Task, which would prevent other pending tasks from being woken.

        Args:
            source: Python statement or expression to execute.

        Returns:
            An `ExecutionResult` with captured output, result, and error info.
        """
        output = io.StringIO()
        result = _SENTINEL
        try:
            with contextlib.redirect_stdout(output):
                is_exec = True
                try:
                    code = compile(source, "<string>", "eval", flags=self._compiler_flags, dont_inherit=True)
                except SyntaxError:
                    pass
                else:
                    result = eval(code, self._globals, self._locals)  # noqa: S307
                    is_exec = False

                if is_exec:
                    code = compile(source, "<string>", "exec", flags=self._compiler_flags, dont_inherit=True)
                    result = eval(code, self._globals, self._locals)  # noqa: S307

                is_coroutine = self._coroutine_flag != -1 and bool(code.co_flags & self._coroutine_flag)
                if not is_coroutine and is_exec:
                    result = _SENTINEL
        except SystemExit as exc:
            return ExecutionResult(
                output=_safe_getvalue(output),
                exception=RuntimeError(f"SystemExit({exc.code}) intercepted — use raise RuntimeError() instead"),
                traceback_str=f"SystemExit({exc.code}): The python_server caught SystemExit to prevent application shutdown.\n",
            )
        except Exception as exc:
            return ExecutionResult(
                output=_safe_getvalue(output),
                exception=exc,
                traceback_str=_safe_format_exc(exc),
            )
        except BaseException as exc:
            # Catch remaining BaseException subclasses (GeneratorExit, KeyboardInterrupt,
            # custom subclasses) to prevent them from crashing the host application.
            return ExecutionResult(
                output=_safe_getvalue(output),
                exception=RuntimeError(
                    f"{type(exc).__name__}({exc}) intercepted — "
                    "BaseException subclasses must not propagate out of user code"
                ),
                traceback_str=_safe_format_exc(exc),
            )

        return ExecutionResult(output=output.getvalue(), result=result)
