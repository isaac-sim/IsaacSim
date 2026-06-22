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

"""Module providing custom doctest functionality with extended output checking capabilities."""

from __future__ import annotations

import doctest
import inspect

NO_CHECK = doctest.register_optionflag("NO_CHECK")
"""Custom doctest directive to run an example but ignore its output.

When specified, the example code is executed but no output comparison is performed,
regardless of whether the example produces output or not. This is useful for examples
that have non-deterministic output (e.g., memory addresses, timestamps, random values)
or when you only want to verify that code runs without errors.

The following directives are available for use in docstring examples:

**Custom Directives (this extension):**

* ``# doctest: +NO_CHECK``: Run the example but skip output verification.

**Standard Python doctest Directives:**

* ``# doctest: +SKIP``: Skip this example entirely (don't run it).
* ``# doctest: +ELLIPSIS``: Allow ``...`` in expected output to match any substring.
* ``# doctest: +NORMALIZE_WHITESPACE``: Ignore whitespace differences when comparing output.
* ``# doctest: +IGNORE_EXCEPTION_DETAIL``: Ignore exception message details, only check type.

**Default Flags:**

The ``assertDocTest`` and ``assertDocTests`` methods use these flags by default:
``doctest.NORMALIZE_WHITESPACE | doctest.ELLIPSIS | doctest.FAIL_FAST``

Example:

.. code-block:: python

    # Run but don't check output (useful for non-deterministic results)
    >>> print(list(range(20)))  # doctest: +NO_CHECK

    # Skip this example entirely
    >>> some_dangerous_operation()  # doctest: +SKIP

    # Use ellipsis to match variable parts of output
    >>> print({"key": "value", "id": 12345})
    {...'key': 'value'...}

    # Combine multiple directives
    >>> print(complex_output())  # doctest: +ELLIPSIS, +NORMALIZE_WHITESPACE
    ...output pattern...
"""


class _Checker(doctest.OutputChecker):
    """Custom doctest's output checker to support the NO_CHECK option."""

    def check_output(self, want: str, got: str, optionflags: int) -> bool:
        """Check if the actual output matches the expected output.

        Extends the default doctest output checking by supporting the NO_CHECK flag.
        When NO_CHECK is set in optionflags, the method returns True regardless of
        whether the actual output matches the expected output.

        Args:
            want: The expected output string.
            got: The actual output string.
            optionflags: Bitwise OR of option flags controlling comparison behavior.

        Returns:
            True if the outputs match or NO_CHECK flag is set, False otherwise.
        """
        result = super().check_output(want, got, optionflags)
        if not result:
            if optionflags & NO_CHECK:
                return True
        return result


class DocTest:
    """A doctest runner for validating docstring examples in Python modules and classes.

    This class provides functionality to run doctest examples found in docstrings of classes, modules,
    functions, and methods. It supports custom doctest directives including a NO_CHECK flag for running
    examples without output verification, which is useful for non-deterministic output.

    The class automatically discovers and tests all public members of a given object, with support for
    both regular Python modules and pybind11 binary extension modules. It maintains a global namespace
    for example execution and uses a custom output checker to handle the NO_CHECK directive.

    Args:
        *args: Variable length argument list passed to the doctest runner.
        **kwargs: Arbitrary keyword arguments for configuring the doctest runner.
    """

    def __init__(self, *args: object, **kwargs: object) -> None:
        self._globs = {"__name__": "__main__"}
        self._checker = _Checker()

    def _get_names(self, obj: object, privates: bool = False) -> list[str]:
        """Get class/module names without including special methods.

        Args:
            obj: The class or module object to get names from.
            privates: Whether to include private names (starting with underscore).

        Returns:
            List of qualified names in the format 'obj_name.member_name'.
        """
        names = []
        is_pybind11_mod = self._is_pybind11_module(obj)

        for name, value in inspect.getmembers(obj):
            if self._is_function_like(value):
                # ignore special names, e.g., __str__
                if name.startswith("__"):
                    continue
                # ignore private names unless specified
                elif name.startswith("_") and not privates:
                    continue
                # ignore functions not defined in current module
                elif inspect.ismodule(obj) and not is_pybind11_mod and inspect.getmodule(value) != obj:
                    continue
                # for pybind11 modules, skip the module check since getmodule returns None
                elif inspect.ismodule(obj) and is_pybind11_mod:
                    # accept all functions in pybind11 modules since getmodule doesn't work
                    pass
                # ignore properties/methods not defined in current class
                elif inspect.isclass(obj) and not obj.__dict__.get(name):
                    continue
                names.append(f"{obj.__name__}.{name}")
        # add class name to list
        if inspect.isclass(obj):
            names.insert(0, obj.__name__)
        # add module name to list
        if inspect.ismodule(obj):
            names.insert(0, obj.__name__)
        # order methods
        if "initialize" in names:
            names.remove("initialize")
            names.insert(1, "initialize")
        return sorted(names)

    def get_members(
        self,
        expr: object,
        order: list[tuple[object, int]] | None = None,
        exclude: list[object] | None = None,
        _globals: dict | None = None,
    ) -> list[object]:
        """Get class/module members (names).

        Args:
            expr: module function or class definition, property or method to check docstrings examples for
            order: list of pair (name, index) to modify the examples execution order
            exclude: list of class/module names to exclude for testing
            _globals: current namespace

        Returns:
            list of class/module members
        """
        if order is None:
            order = []
        if exclude is None:
            exclude = []
        if _globals is None:
            _globals = {}
        _globals.update({expr.__name__: expr})
        members = [eval(name, _globals) for name in self._get_names(expr)]
        # remove exclude items
        members = [member for member in members if not member in exclude]
        # order names
        for member, index in order:
            if member in members:
                index = len(members) + index if index < 0 else index
                members.insert(index, members.pop(members.index(member)))
        return members

    def _check_doc_test_impl(
        self,
        expr: object,
        flags: int = doctest.NORMALIZE_WHITESPACE | doctest.ELLIPSIS | doctest.FAIL_FAST,
    ) -> bool:
        """Check that the examples in docstrings pass for a class/module member.

        Args:
            expr: module function or class definition, property or method to check docstrings examples for
            flags: doctest's option flags

        Returns:
            whether the test passes or fails
        """
        # implement doctest.run_docstring_examples with execution checking
        testFinder = doctest.DocTestFinder(verbose=False, recurse=False)
        testRunner = doctest.DocTestRunner(checker=self._checker, verbose=False, optionflags=flags)
        for test in testFinder.find(expr, name="module"):
            test.globs = self._globs
            try:
                status = testRunner.run(test, clear_globs=False)
            except SystemExit as exc:
                import carb

                carb.log_error(
                    f"SystemExit({exc.code}) raised during doctest for {expr!r}. "
                    "Docstring examples must not call sys.exit()."
                )
                return False
            if status.failed:
                return False
        return True

    checkDocTest = _check_doc_test_impl

    def _is_pybind11_module(self, obj: object) -> bool:
        """Check if this is a pybind11 module.

        Args:
            obj: The object to check.

        Returns:
            True if the object is a pybind11 module.
        """
        if not inspect.ismodule(obj):
            return False

        # check if it's a binary extension module
        if hasattr(obj, "__file__") and obj.__file__:
            import os

            _, ext = os.path.splitext(obj.__file__)
            if ext in [".so", ".pyd", ".dll"]:
                return True

        return False

    def _is_function_like(self, obj: object) -> bool:
        """Check if object is function-like (including pybind11 functions).

        Args:
            obj: The object to check.

        Returns:
            True if the object is function-like.
        """
        return (
            inspect.isfunction(obj)
            or inspect.isdatadescriptor(obj)
            or (callable(obj) and hasattr(obj, "__doc__") and not inspect.isclass(obj) and not inspect.ismodule(obj))
        )
