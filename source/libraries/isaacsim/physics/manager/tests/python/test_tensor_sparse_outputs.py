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

"""Verify legacy optional destinations with modern multi-buffer providers."""

import gc
import threading
import weakref

import numpy as np
import pytest
import warp as wp
from isaacsim.physics.manager.impl import tensors as t


@pytest.mark.parametrize("device", ["cpu", "cuda:0"])
@pytest.mark.parametrize("mode", ["default", "unspecified", "mixed", "complete"])
def test_sparse_outputs_preserve_destinations(device: str, mode: str) -> None:
    """Preserve supplied storage and owners across the allocation fallback."""
    wp.init()
    if device != "cpu" and not wp.is_cuda_available():
        pytest.skip("CUDA device unavailable")
    source = [np.arange(3, dtype=np.float32), np.arange(4, dtype=np.float32)]
    calls = []

    def getter(indices: object, out: object) -> object:
        calls.append(out is None)
        return source if out is None else out

    view = t.EntityView()
    view._register_multi_impl("read", t.ImplKind.Get, getter, t.ArrayTensorSpec(dtype=wp.float32))
    first = wp.array(np.zeros_like(source[0]) if mode == "mixed" else source[0], device=device)
    second = wp.array(source[1], device=device)
    owner = weakref.ref(first)
    released = threading.Event()
    weakref.finalize(first, released.set)
    pointer = first.ptr
    outputs = {
        "default": [],
        "unspecified": [None, None],
        "mixed": [first, None],
        "complete": [first, second],
    }[mode]
    result = [wp.from_dlpack(value) for value in view.get_data_multi("read", out=outputs)]
    assert calls == [mode != "complete"]
    del outputs, first, second
    gc.collect()
    for index, value in enumerate(result):
        np.testing.assert_array_equal(value.numpy(), source[index])
    del value
    if mode in ("mixed", "complete"):
        assert result[0].ptr == pointer
        assert str(result[0].device) == device
        assert owner() is not None
    result.clear()
    wp.synchronize()
    gc.collect()
    assert released.wait(timeout=5)
    assert owner() is None


@pytest.mark.parametrize("mismatch", ["shape", "dtype", "count", "layout"])
def test_sparse_outputs_validate_before_writing(mismatch: str) -> None:
    """Leave every supplied buffer unchanged when a later slot is invalid."""
    view = t.EntityView()
    values = [np.ones(3, dtype=np.float32) for _ in range(3)]
    if mismatch == "count":
        values.pop()
    view._register_multi_impl(
        "read",
        t.ImplKind.Get,
        lambda indices, out: values,
        t.ArrayTensorSpec(dtype=wp.float32),
    )
    first = np.full(3, 17, dtype=np.float32)
    last = np.zeros(
        4 if mismatch == "shape" else 3,
        dtype=np.float64 if mismatch == "dtype" else np.float32,
    )
    if mismatch == "layout":
        last = np.lib.stride_tricks.as_strided(last, shape=(3,), strides=(0,))
    with pytest.raises(ValueError, match="sparse output"):
        view.get_data_multi("read", out=[first, None, last])
    np.testing.assert_array_equal(first, np.full(3, 17, dtype=np.float32))


def test_sparse_outputs_propagate_provider_error() -> None:
    """Preserve provider failures without modifying supplied destinations."""

    def getter(indices: object, out: object) -> object:
        raise ValueError("provider allocation failed")

    view = t.EntityView()
    view._register_multi_impl("read", t.ImplKind.Get, getter, t.ArrayTensorSpec(dtype=wp.float32))
    output = np.full(3, 17, dtype=np.float32)
    with pytest.raises(ValueError, match="provider allocation failed"):
        view.get_data_multi("read", out=[output, None])
    np.testing.assert_array_equal(output, np.full(3, 17, dtype=np.float32))


@pytest.mark.parametrize("empty", [False, True])
def test_sparse_outputs_accept_strided_and_empty_buffers(empty: bool) -> None:
    """Copy into valid strided or zero-length destinations without substitution."""
    source = np.arange(0 if empty else 3, dtype=np.float32)
    storage = np.full(6, 17, dtype=np.float32)
    destination = storage[:0] if empty else storage[::2]
    view = t.EntityView()
    view._register_multi_impl(
        "read",
        t.ImplKind.Get,
        lambda indices, out: [source, source],
        t.ArrayTensorSpec(dtype=wp.float32),
    )
    result = view.get_data_multi("read", out=[destination, None])
    np.testing.assert_array_equal(destination, source)
    np.testing.assert_array_equal(storage[1::2], np.full(3, 17, dtype=np.float32))
    assert len(result) == 2


def test_sparse_outputs_snapshot_cross_slot_aliases() -> None:
    """Protect later and omitted results from overlapping destination writes."""
    first = np.array([1, 2, 3], dtype=np.float32)
    last = np.array([4, 5, 6], dtype=np.float32)
    view = t.EntityView()
    view._register_multi_impl(
        "read",
        t.ImplKind.Get,
        lambda indices, out: [first, last, last],
        t.ArrayTensorSpec(dtype=wp.float32),
    )
    result = [wp.from_dlpack(value) for value in view.get_data_multi("read", out=[last, None, first])]
    np.testing.assert_array_equal(last, [1, 2, 3])
    np.testing.assert_array_equal(first, [4, 5, 6])
    np.testing.assert_array_equal(result[1].numpy(), [4, 5, 6])


def test_sparse_outputs_allow_omitted_trailing_destinations() -> None:
    """Treat missing trailing slots as unspecified without dropping supplied buffers."""
    source = [np.arange(3, dtype=np.float32) for _ in range(3)]
    view = t.EntityView()
    view._register_multi_impl(
        "read",
        t.ImplKind.Get,
        lambda indices, out: source,
        t.ArrayTensorSpec(dtype=wp.float32),
    )
    destination = np.zeros(3, dtype=np.float32)
    result = [wp.from_dlpack(value) for value in view.get_data_multi("read", out=[destination, None])]
    assert len(result) == 3
    assert result[0].ptr == destination.ctypes.data
    for value in result:
        np.testing.assert_array_equal(value.numpy(), source[0])
