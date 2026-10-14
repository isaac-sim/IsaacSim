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

"""Keep shaped empty tensors distinct from omitted buffers at native callbacks."""

import pytest
import warp as wp
from isaacsim.physics.manager.impl import tensors as t


@pytest.mark.parametrize("multi", [False, True])
def test_native_callbacks_preserve_empty_tensor_shapes(multi: bool) -> None:
    """Raw binding calls preserve zero-row selectors, zero-DOF data, and output slots."""
    view = t.EntityView()
    spec = t.TensorSpec(
        supports_indexed_read=True,
        supports_indexed_write=True,
    )
    indices = wp.zeros(0, dtype=wp.int32, device="cpu")
    data = wp.zeros((2, 0), dtype=wp.float32, device="cpu")
    calls = []

    def read(selection: object, output: object) -> object:
        calls.append("get")
        assert selection is not None
        assert wp.from_dlpack(selection).shape == (0,)
        if multi:
            assert output[0] is None
            assert output[1] is not None
            value = wp.from_dlpack(output[1])
            assert value.shape == (2, 0)
            return [t.TensorDesc(), value]
        assert output is not None
        value = wp.from_dlpack(output)
        assert value.shape == (2, 0)
        return value

    def write(value: object, selection: object) -> None:
        calls.append("set")
        assert selection is not None and wp.from_dlpack(selection).shape == (0,)
        if multi:
            assert len(value) == 2
            assert all(item is not None and wp.from_dlpack(item).shape == (2, 0) for item in value)
        else:
            assert value is not None and wp.from_dlpack(value).shape == (2, 0)

    register = view._register_multi_impl if multi else view._register_impl
    register("empty", t.ImplKind.Get, read, spec)
    register("empty", t.ImplKind.Set, write, spec)
    # Class methods bypass the public Warp dispatch's empty-write no-op.
    if multi:
        result = t.EntityView.get_data_multi(view, "empty", indices, [None, data])
        assert result[0] is None
        assert wp.from_dlpack(result[1]).shape == (2, 0)
        t.EntityView.set_data_multi(view, "empty", [data, data], indices)
    else:
        result = t.EntityView.get_data(view, "empty", indices, data)
        assert wp.from_dlpack(result).shape == (2, 0)
        t.EntityView.set_data(view, "empty", data, indices)
    assert calls == ["get", "set"]


def test_modern_multi_callback_receives_complete_empty_outputs() -> None:
    """A present zero-sized destination selects the provider's supplied-output route."""
    view = t.EntityView()
    output = wp.zeros((2, 0), dtype=wp.float32, device="cpu")
    calls = []

    def read(indices: object, outputs: object) -> object:
        assert indices is None
        assert outputs is not None
        assert len(outputs) == 1
        assert tuple(outputs[0].shape) == (2, 0)
        calls.append("supplied")
        return outputs

    view._register_multi_impl(
        "empty",
        t.ImplKind.Get,
        read,
        t.ArrayTensorSpec(dtype=wp.float32, shape_hint=[2, 0], device="cpu"),
    )
    result = t.EntityView.get_data_multi(view, "empty", None, [output])
    assert len(result) == 1
    assert wp.from_dlpack(result[0]).shape == (2, 0)
    assert calls == ["supplied"]
