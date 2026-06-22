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

"""Async testing framework with doctest support for validating code examples in docstrings."""

from __future__ import annotations

import doctest
import sys

import isaacsim
import omni
import omni.kit.test

from . import _doctest


class AsyncDocTestCase(omni.kit.test.AsyncTestCase):
    """Base class for all async test cases with doctest support for checking docstrings examples.

    This class provides methods to validate that code examples in docstrings execute correctly.
    It is designed for Kit-based async test environments and inherits from ``omni.kit.test.AsyncTestCase``.

    .. note::

        Derive from it to make the tests auto discoverable.
        Test methods must start with the ``test_`` prefix.

    This class adds the following methods:

    * ``assertDocTest``: Check that the examples in docstrings pass for a class/module member.
    * ``assertDocTests``: Check that the examples in docstrings pass for all class/module's members.

    **Available Doctest Directives:**

    Use these directives in docstring examples to control test behavior:

    * ``# doctest: +NO_CHECK``: Run the example but skip output verification.
    * ``# doctest: +SKIP``: Skip this example entirely (don't run it).
    * ``# doctest: +ELLIPSIS``: Allow ``...`` in expected output to match any substring.
    * ``# doctest: +NORMALIZE_WHITESPACE``: Ignore whitespace differences.

    **Creating a Test File:**

    To test docstrings for an extension module or class, create a test file like this:

    .. code-block:: python

        # test_docstrings.py
        from isaacsim.test.docstring import AsyncDocTestCase
        import my_extension_module

        class TestDocstrings(AsyncDocTestCase):
            async def test_my_module_docstrings(self):
                # Test all members of a module
                await self.assertDocTests(my_extension_module)

            async def test_single_function(self):
                # Test a specific function (sync method)
                self.assertDocTest(my_extension_module.some_function)

            async def test_with_options(self):
                # Test with custom options
                await self.assertDocTests(
                    my_extension_module.MyClass,
                    exclude=[my_extension_module.MyClass.internal_method],
                    stop_on_failure=True,  # Stop at first failure
                    await_update=True,  # Await Kit update between tests
                )

    Example:

    .. code-block:: python

        >>> import isaacsim.test.docstring
        >>> tester = isaacsim.test.docstring.AsyncDocTestCase()
        >>> tester.__class__.__name__
        'AsyncDocTestCase'

    Args:
        *args: Variable positional arguments passed to the parent class.
        **kwargs: Additional keyword arguments passed to the parent class.
    """

    def __init__(self, *args: object, **kwargs: object) -> None:
        super().__init__(*args, **kwargs)
        self._doctest_checker = _doctest.DocTest()

    def _assert_doc_test_impl(
        self,
        expr: object,
        msg: str = "",
        flags: int = doctest.NORMALIZE_WHITESPACE | doctest.ELLIPSIS | doctest.FAIL_FAST,
    ) -> None:
        """Check that the examples in docstrings pass for a class/module member.

        Args:
            expr: Module function or class definition, property or method to check docstrings examples for.
            msg: Custom message to display when failing.
            flags: Doctest option flags.

        Example:

        .. code-block:: python

            >>> from isaacsim.test.docstring import AsyncDocTestCase
            >>>
            >>> tester.assertDocTest(AsyncDocTestCase)
        """
        result = self._doctest_checker.checkDocTest(expr, flags)
        if not result:
            sys.tracebacklimit = 0  # don't show test file tracing, only doctest
            self.fail("Examples in docstrings failed" if not msg else msg)

    assertDocTest = _assert_doc_test_impl

    async def _assert_doc_tests_impl(
        self,
        expr: object,
        msg: str = "",
        flags: int = doctest.NORMALIZE_WHITESPACE | doctest.ELLIPSIS | doctest.FAIL_FAST,
        order: list[tuple[object, int]] | None = None,
        exclude: list[object] | None = None,
        stop_on_failure: bool = False,
        await_update: bool = True,
    ) -> None:
        """Check that the examples in docstrings pass for all class/module's members (names).

        Args:
            expr: Module or class definition to check members' docstrings examples for.
            msg: Custom message to display when failing.
            flags: Doctest option flags.
            order: List of pair (name, index) to modify the examples execution order.
            exclude: List of class/module names to exclude for testing.
            stop_on_failure: Stop testing docstrings example at first encountered failure.
            await_update: Await next kit application update async before running each docstrings example.

        Example:

        .. code-block:: python

            >>> from isaacsim.test.docstring import AsyncDocTestCase
            >>> from omni.kit.async_engine import run_coroutine
            >>>
            >>> async def task():
            ...     await tester.assertDocTests(AsyncDocTestCase, exclude=[AsyncDocTestCase.assertDocTests])
            ...
            >>> run_coroutine(task())  # doctest: +NO_CHECK
        """
        if order is None:
            order = []
        if exclude is None:
            exclude = []
        objects = self._doctest_checker.get_members(expr, order, exclude, {"omni": omni, "isaacsim": isaacsim})
        print(f"class/module members to check: {len(objects)}")
        # test docstrings examples
        failures = 0
        for obj in objects:
            name = getattr(obj, "__name__", None)
            if name is None:
                name = obj.fget.__name__
            print(f"  |-- checking: {name}")
            if await_update:
                await omni.kit.app.get_app().next_update_async()
            try:
                self.assertDocTest(obj, msg=msg, flags=flags)
            except AssertionError as e:
                failures += 1
                if stop_on_failure:
                    raise e
        if failures:
            self.fail(
                f"Some tests failed: failed ({failures}), successful ({len(objects) - failures}), total ({len(objects)})"
            )

    assertDocTests = _assert_doc_tests_impl
