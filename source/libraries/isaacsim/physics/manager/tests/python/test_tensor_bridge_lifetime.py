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

"""Check storage ownership across descriptor providers and Array callers."""

from __future__ import annotations

import gc
import threading
import weakref

import numpy as np
import pytest
import warp as wp
from isaacsim.physics.manager.impl import tensors as t


def test_array_get_retains_legacy_callback_output_owner() -> None:
    """Retain caller storage after crossing the legacy callback bridge."""
    wp.init()
    view = t.EntityView()
    view._register_impl("echo", t.ImplKind.Get, lambda indices, out: out, t.TensorSpec())
    output = np.arange(4, dtype=np.float32)
    owner = weakref.ref(output)
    result = view.get_array_data("echo", out=output)
    assert result.ptr == output.ctypes.data
    del output
    gc.collect()
    assert owner() is not None
    np.testing.assert_array_equal(result.numpy(), np.arange(4, dtype=np.float32))
    del result
    gc.collect()
    assert owner() is None


def test_array_multi_get_retains_each_legacy_output_owner() -> None:
    """Keep each supplied output alive independently of the temporary list."""
    wp.init()
    view = t.EntityView()
    view._register_multi_impl("echo", t.ImplKind.Get, lambda indices, out: out, t.TensorSpec())
    outputs = [np.arange(3, dtype=np.float32), np.arange(5, dtype=np.float32)]
    owners = [weakref.ref(value) for value in outputs]
    result = view.get_array_data_multi("echo", out=outputs)
    del outputs
    gc.collect()
    assert all(owner() is not None for owner in owners)
    np.testing.assert_array_equal(result[1].numpy(), np.arange(5, dtype=np.float32))
    del result
    gc.collect()
    assert all(owner() is None for owner in owners)


@pytest.mark.parametrize("layout", ["contiguous", "sliced", "transposed"])
def test_legacy_callback_can_retain_array_input(layout: str) -> None:
    """Retain input storage when a provider keeps its callback argument."""
    wp.init()
    retained = []
    view = t.EntityView()
    view._register_impl(
        "capture",
        t.ImplKind.Set,
        lambda data, indices: retained.append(wp.from_dlpack(data)),
        t.TensorSpec(),
    )
    storage = np.arange(12, dtype=np.float32)
    source = {
        "contiguous": storage,
        "sliced": storage[1::2],
        "transposed": storage.reshape(3, 4).T,
    }[layout]
    expected = source.copy()
    expected_strides = source.strides
    owner = weakref.ref(storage)
    view.set_array_data("capture", source)
    del source, storage
    gc.collect()
    assert owner() is not None
    assert retained[0].strides == expected_strides
    np.testing.assert_array_equal(retained[0].numpy(), expected)
    retained.clear()
    gc.collect()
    assert owner() is None


def test_unknown_legacy_spec_rejects_array_metadata_query() -> None:
    """Keep Unknown raw metadata without advertising Float32 to Array callers."""
    view = t.EntityView()
    view._register_impl(
        "unknown",
        t.ImplKind.Get,
        lambda indices, out: out,
        t.TensorSpec(dtype=t.DType.UNKNOWN),
    )
    assert view.get_impl_spec("unknown", t.ImplKind.Get).dtype == t.DType.UNKNOWN
    with pytest.raises(ValueError, match="unspecified tensor dtype"):
        view.get_array_impl_spec("unknown", t.ImplKind.Get)


def test_legacy_callback_preserves_negative_strides() -> None:
    """Keep signed strides and storage ownership on the raw descriptor path."""

    class CapsuleSource:
        """Adapt the raw callback capsule to the NumPy DLPack protocol."""

        def __init__(self, capsule: object) -> None:
            self.capsule = capsule

        def __dlpack__(self) -> object:
            return self.capsule

    retained = []
    view = t.EntityView()
    view._register_impl(
        "capture",
        t.ImplKind.Set,
        lambda data, indices: retained.append(np.from_dlpack(CapsuleSource(data))),
        t.TensorSpec(),
    )
    storage = np.arange(6, dtype=np.float32)
    source = storage[::-1]
    owner = weakref.ref(storage)
    view.set_data("capture", source)
    assert retained[0].strides == source.strides
    assert retained[0].ctypes.data == source.ctypes.data
    del source, storage
    gc.collect()
    assert owner() is not None
    np.testing.assert_array_equal(retained[0], np.arange(6, dtype=np.float32)[::-1])
    retained.clear()
    gc.collect()
    assert owner() is None


@pytest.mark.parametrize("operation", ["get", "set"])
def test_cuda_bridge_retains_owner(operation: str) -> None:
    """Retain real CUDA allocations across descriptor callback boundaries."""
    wp.init()
    if not wp.is_cuda_available():
        pytest.skip("CUDA device unavailable")
    view = t.EntityView()
    retained = []
    if operation == "get":
        view._register_impl("echo", t.ImplKind.Get, lambda indices, out: out, t.TensorSpec())
    else:
        view._register_impl(
            "capture",
            t.ImplKind.Set,
            lambda data, indices: retained.append(wp.from_dlpack(data)),
            t.TensorSpec(),
        )
    source = wp.array(np.arange(12, dtype=np.float32), device="cuda:0")
    released = threading.Event()
    weakref.finalize(source, released.set)
    owner = weakref.ref(source)
    pointer = source.ptr
    if operation == "get":
        retained.append(view.get_array_data("echo", out=source))
    else:
        view.set_array_data("capture", source)
    del source
    gc.collect()
    assert owner() is not None
    assert retained[0].device.is_cuda
    assert retained[0].ptr == pointer
    np.testing.assert_array_equal(retained[0].numpy(), np.arange(12, dtype=np.float32))
    retained.clear()
    wp.synchronize()
    gc.collect()
    # Device synchronization does not join the native allocation retirement worker.
    assert released.wait(timeout=5), "CUDA allocation owner was not released"
    assert owner() is None
