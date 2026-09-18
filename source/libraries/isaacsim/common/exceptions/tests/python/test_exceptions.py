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

"""Test the Python exceptions facade."""

from __future__ import annotations

import isaacsim.common.exceptions as exceptions
import pytest
from isaacsim.common.exceptions.bindings import _bindings


def test_every_exception_is_an_exception_subclass() -> None:
    """Verify each exported type is usable as a Python exception."""
    for name in exceptions.__all__:
        assert issubclass(getattr(exceptions, name), Exception)


def test_every_exception_derives_from_the_common_base() -> None:
    """Verify that public exception types derive from ``IsaacSimException``."""
    assert issubclass(exceptions.PrimPathError, exceptions.IsaacSimException)
    assert issubclass(exceptions.PrimPathStringError, exceptions.IsaacSimException)
    assert issubclass(exceptions.AttributeNameError, exceptions.IsaacSimException)
    assert issubclass(exceptions.ValueTypeError, exceptions.IsaacSimException)


def test_prim_path_error_carries_the_offending_path() -> None:
    """Verify PrimPathError exposes the offending path and a formatted message."""
    with pytest.raises(exceptions.PrimPathError) as excinfo:
        _bindings._raise_prim_path_error("/World/does_not_exist")

    assert excinfo.value.prim_path == "/World/does_not_exist"
    assert str(excinfo.value) == "Invalid prim at path: '/World/does_not_exist'"
    # Catchable through the common base.
    assert isinstance(excinfo.value, exceptions.IsaacSimException)


def test_prim_path_string_error_carries_the_offending_string() -> None:
    """Verify PrimPathStringError exposes the offending string and a formatted message."""
    with pytest.raises(exceptions.PrimPathStringError) as excinfo:
        _bindings._raise_prim_path_string_error("not a path")

    assert excinfo.value.prim_path == "not a path"
    assert str(excinfo.value) == "Invalid prim path string: 'not a path'"
    assert isinstance(excinfo.value, exceptions.IsaacSimException)


def test_attribute_name_error_carries_the_offending_name() -> None:
    """Verify AttributeNameError exposes the offending attribute name and a formatted message."""
    with pytest.raises(exceptions.AttributeNameError) as excinfo:
        _bindings._raise_attribute_name_error("not_an_attribute")

    assert excinfo.value.attribute_name == "not_an_attribute"
    assert excinfo.value.valid_attribute_names == []
    assert str(excinfo.value) == "Invalid attribute name: 'not_an_attribute'"
    assert isinstance(excinfo.value, exceptions.IsaacSimException)


def test_attribute_name_error_carries_the_valid_attribute_names() -> None:
    """Verify AttributeNameError exposes the valid attribute names when provided."""
    with pytest.raises(exceptions.AttributeNameError) as excinfo:
        _bindings._raise_attribute_name_error("not_an_attribute", ["scale", "translate"])

    assert excinfo.value.attribute_name == "not_an_attribute"
    assert excinfo.value.valid_attribute_names == ["scale", "translate"]
    assert (
        str(excinfo.value) == "Invalid attribute name: 'not_an_attribute'. Valid attribute names: 'scale', 'translate'"
    )
    assert isinstance(excinfo.value, exceptions.IsaacSimException)


def test_value_type_error_carries_the_expected_and_actual_types() -> None:
    """Verify ValueTypeError exposes the attribute, expected, and actual types and a formatted message."""
    with pytest.raises(exceptions.ValueTypeError) as excinfo:
        _bindings._raise_value_type_error("scale", "float", "str")

    assert excinfo.value.attribute_name == "scale"
    assert excinfo.value.expected_type == "float"
    assert excinfo.value.actual_type == "str"
    assert str(excinfo.value) == "Invalid value type for attribute 'scale': expected float, got str"
    assert isinstance(excinfo.value, exceptions.IsaacSimException)


def test_every_exception_is_catchable_through_the_common_base() -> None:
    """Verify all three exceptions can be caught generically through IsaacSimException."""
    for raise_callable, args in (
        (_bindings._raise_prim_path_error, ("/World/foo",)),
        (_bindings._raise_prim_path_string_error, ("not a path",)),
        (_bindings._raise_attribute_name_error, ("foo",)),
        (_bindings._raise_value_type_error, ("foo", "int", "str")),
    ):
        with pytest.raises(exceptions.IsaacSimException):
            raise_callable(*args)
