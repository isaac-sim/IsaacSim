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

"""Pin the diagnostics the nanobind Array caster emits when a conversion fails.

A caster reports failure by returning false, which nanobind renders as a generic
"incompatible function arguments" error naming only the expected signature. That text is
identical whether the argument was the wrong type entirely or was an array the caster
tried and failed to import, so the specific cause used to be unrecoverable. The cause now
travels through ``sys.unraisablehook``, which is the one channel that survives overload
resolution, and these tests pin that it is emitted when -- and only when -- a conversion
was actually attempted and failed.

``EntityView`` is used as the vehicle simply because it is the reachable binding that
takes an ``Array``; nothing here depends on a physics engine or a running simulation.
"""

from __future__ import annotations

import sys
from collections.abc import Iterator

import _isaacsim_tensor_native_fixture as native
import isaacsim.physics.manager.impl.tensors as t
import numpy as np
import pytest
import warp as wp


@pytest.fixture
def unraisable(monkeypatch: pytest.MonkeyPatch) -> Iterator[list]:
    """Capture exceptions reported through ``sys.unraisablehook``.

    Replacing the hook also keeps the reports away from pytest's own unraisable plugin,
    which would otherwise turn each one into a ``PytestUnraisableExceptionWarning``.
    """
    captured: list = []
    monkeypatch.setattr(sys, "unraisablehook", captured.append)
    try:
        yield captured
    finally:
        # Tracebacks retain the test frame, including this capture list and native objects.
        captured.clear()


def _causes(captured: list) -> list[str]:
    return [str(entry.exc_value) for entry in captured]


class TestArrayCasterDiagnostics:
    """Verify a failed Array conversion reports why it failed."""

    def test_unsupported_dtype_reports_its_cause(self, unraisable: list) -> None:
        """Verify an array of an unrepresentable dtype names the dtype that was refused."""
        view = t.EntityView()
        with pytest.raises(TypeError):
            view.set_array_data("any-op", wp.zeros((2, 3), dtype=wp.float16, device="cpu"))
        assert len(unraisable) == 1
        assert "unsupported dtype" in _causes(unraisable)[0]

    def test_ragged_sequence_reports_its_cause(self, unraisable: list) -> None:
        """Verify a list that numpy refuses to rectangularize reports numpy's own complaint.

        The cause here is a Python exception, which is handed back to the interpreter rather
        than flattened to a string, so it keeps numpy's own type and wording.
        """
        view = t.EntityView()
        with pytest.raises(TypeError):
            view.set_array_data("any-op", [[1, [2]], [3, [4]]])
        assert len(unraisable) == 1
        assert "inhomogeneous" in _causes(unraisable)[0]

    def test_out_of_range_scalar_reports_its_cause(self, unraisable: list) -> None:
        """Verify an integer too large for int64 is reported rather than silently refused.

        The cause is nanobind's own cast failure, whose wording does not mention int64; what
        is pinned is that a cause is reported at all, and that it names the conversion.
        """
        view = t.EntityView()
        with pytest.raises(TypeError):
            view.set_array_data("any-op", 2**200)
        assert len(unraisable) == 1
        assert "isaacsim::common::array::Array" in _causes(unraisable)[0]

    def test_convertible_array_reports_nothing(self, unraisable: list) -> None:
        """Verify a supported array stays silent, so the hook marks real failures only.

        The write still fails -- no set-implementation is registered -- but it fails inside
        the binding, which means the argument converted cleanly. ``IndexError`` is how
        nanobind surfaces the ``std::out_of_range`` the view raises for an unknown operation.
        """
        view = t.EntityView()
        source = np.arange(6, dtype=np.int32)
        for array in (
            wp.zeros((2, 3), dtype=wp.float32, device="cpu"),
            source[:1][::-1],
            np.lib.stride_tricks.as_strided(source, shape=(0, 3), strides=(-12, -4)),
            source[::2],
        ):
            with pytest.raises(IndexError, match="not registered"):
                view.set_array_data("any-op", array)
        assert unraisable == []

    def test_unrelated_type_reports_nothing(self, unraisable: list) -> None:
        """Verify an argument that is not array-like is refused without a report.

        Nothing was attempted here, so there is no cause to preserve; reporting one would
        make ordinary overload resolution noisy.
        """
        view = t.EntityView()
        with pytest.raises(TypeError):
            view.set_array_data("any-op", object())
        assert unraisable == []


@pytest.mark.parametrize("cuda", [False, True])
@pytest.mark.parametrize("mutate", [False, True])
def test_export_retains_owner_without_blocking_mutation(cuda: bool, mutate: bool) -> None:
    """A Python alias survives its local Array and observes a subsequent native set."""
    if cuda and not wp.is_cuda_available():
        pytest.skip("CUDA is unavailable")
    exported = native.export_local_array(cuda, mutate)
    np.testing.assert_array_equal(exported.numpy(), [4, 5, 6] if mutate else [1, 2, 3])


def test_reversed_array_is_rejected(unraisable: list) -> None:
    """A meaningful negative stride still fails the contiguous compatibility contract."""
    with pytest.raises(TypeError):
        t.EntityView().set_array_data("any-op", np.arange(3, dtype=np.int32)[::-1])
    assert len(unraisable) == 1
    assert "C-contiguous" in _causes(unraisable)[0]
