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

"""Exercise the M1 public tensor contract alongside the Array transport."""

from __future__ import annotations

import gc

import numpy as np
import pytest
import warp as wp
from isaacsim.physics.manager.impl import tensors as t


def test_legacy_specification_and_device_properties() -> None:
    """Preserve enum-valued specifications and legacy device ordinals."""
    spec = t.TensorSpec(dtype=t.DType.FLOAT32, shape_hint=[3], device_kind=t.DeviceKind.CPU)
    view = t.EntityView(["/World"])
    view.device_ordinal = -1
    view._register_impl("legacy", t.ImplKind.Get, lambda indices, out: np.arange(3, dtype=np.float32), spec)
    returned = view.get_impl_spec("legacy", t.ImplKind.Get)
    assert returned.dtype == t.DType.FLOAT32
    assert returned.device_kind == t.DeviceKind.CPU
    assert view.device == "cpu"
    view.device = "cuda:0"
    assert view.device_ordinal == 0


def test_legacy_descriptor_retains_temporary_owner() -> None:
    """Keep descriptor-backed arrays alive after their temporary owner disappears."""
    wp.init()
    descriptor = t.TensorDesc(np.arange(4, dtype=np.float32))
    result = wp.from_dlpack(descriptor.to_array())
    del descriptor
    gc.collect()
    np.testing.assert_array_equal(result.numpy(), np.arange(4, dtype=np.float32))


def test_legacy_get_preserves_caller_output() -> None:
    """Write through and retain the caller's output buffer on the raw binding path."""
    wp.init()
    view = t.EntityView(["/World"])
    view._register_impl("echo", t.ImplKind.Get, lambda indices, out: out, t.TensorSpec())
    output = np.arange(4, dtype=np.float32)
    result = wp.from_dlpack(view.get_data("echo", out=output))
    assert result.ptr == output.ctypes.data
    output[0] = 19
    assert result.numpy()[0] == 19
    del output
    gc.collect()
    assert result.numpy()[0] == 19


def test_legacy_and_array_specifications_share_capability_queries() -> None:
    """Expose old specification queries for an explicitly modern provider."""
    view = t.EntityView(["/World"])
    view._register_impl(
        "modern",
        t.ImplKind.Get,
        lambda indices, out: wp.zeros(3, dtype=wp.float32, device="cpu"),
        t.ArrayTensorSpec(dtype=wp.float32, device="cpu", shape_hint=[3]),
    )
    assert view.get_impl_spec("modern", t.ImplKind.Get).dtype == t.DType.FLOAT32
    assert view.get_array_impl_spec("modern", t.ImplKind.Get).dtype == wp.float32


def test_create_entity_rejects_conflicting_device_arguments() -> None:
    """Reject conflicting old and new device arguments before factory lookup."""
    with pytest.raises(ValueError, match="device_ordinal or device"):
        t.create_entity("unused", "unused", ["/World"], device_ordinal=-1, device="cpu")
