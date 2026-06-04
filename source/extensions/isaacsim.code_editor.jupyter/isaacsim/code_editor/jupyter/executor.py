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

"""Provides functionality for executing Python statements and expressions from strings with customizable namespaces."""

from __future__ import annotations  # isort:skip — must be first import
import __future__ as __future_module__  # isort:skip — needed for _Feature check below

import contextlib
import dis
import io
import traceback

try:
    from ast import PyCF_ALLOW_TOP_LEVEL_AWAIT
except ImportError:
    PyCF_ALLOW_TOP_LEVEL_AWAIT = 0


class Executor:
    """Execute Python statements or expressions from strings.

    Args:
        globals: Global namespace.
        locals: Local namespace.
    """

    def __init__(self, globals: dict | None = None, locals: dict | None = None) -> None:
        self._globals = globals if globals is not None else {}
        self._locals = locals if locals is not None else {}
        self._compiler_flags = self._get_compiler_flags()
        self._coroutine_flag = self._get_coroutine_flag()

    def _get_compiler_flags(self) -> int:
        """Get current Python version compiler flags.

        Returns:
            The compiler flags for the current Python version.
        """
        flags = 0
        for value in globals().values():
            try:
                if isinstance(value, __future_module__._Feature):
                    flags |= value.compiler_flag
            except BaseException:
                pass
        return flags | PyCF_ALLOW_TOP_LEVEL_AWAIT

    def _get_coroutine_flag(self) -> int:
        """Get current Python version coroutine flag.

        Returns:
            The coroutine flag for the current Python version, or -1 if not found.
        """
        for k, v in dis.COMPILER_FLAG_NAMES.items():
            if v == "COROUTINE":
                return k
        return -1

    async def execute(self, source: str) -> tuple[str, Exception, str]:
        """Execute source in the Python scope.

        Args:
            source: Statement or expression.

        Returns:
            A tuple of (standard output, exception thrown (or None if not thrown), exception trace).
        """
        output = io.StringIO()
        try:
            with contextlib.redirect_stdout(output):
                do_exec_step = True
                # try 'eval' first
                try:
                    code = compile(source, "<string>", "eval", flags=self._compiler_flags, dont_inherit=True)
                except SyntaxError:
                    pass
                else:
                    result = eval(code, self._globals, self._locals)
                    do_exec_step = False
                # if 'eval' fails, try 'exec'
                if do_exec_step:
                    code = compile(source, "<string>", "exec", flags=self._compiler_flags, dont_inherit=True)
                    result = eval(code, self._globals, self._locals)
                # await the result if it is a coroutine
                if self._coroutine_flag != -1 and bool(code.co_flags & self._coroutine_flag):
                    result = await result
        except SystemExit as exc:
            error = RuntimeError(
                f"SystemExit({exc.code}): The Jupyter executor caught SystemExit to prevent application shutdown."
            )
            return output.getvalue(), error, traceback.format_exc()
        except Exception as e:
            return output.getvalue(), e, traceback.format_exc()
        except BaseException as exc:
            # Catch remaining BaseException subclasses (GeneratorExit, KeyboardInterrupt,
            # custom subclasses) to prevent them from crashing Kit.
            error = RuntimeError(
                f"{type(exc).__name__}({exc}): The Jupyter executor caught this to prevent application shutdown."
            )
            return output.getvalue(), error, traceback.format_exc()
        return output.getvalue(), None, ""
