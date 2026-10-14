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

"""Collect native callback cycles without weakening live registrations."""

import gc
import weakref

import numpy as np
import pytest
import warp as wp
from isaacsim.physics.manager.impl import tensors as t


class Sentinel:
    """Expose view lifetime through a weak-referenceable owned marker."""


class DerivedView(t.EntityView):
    """Exercise inherited instance dictionaries and bound method cycles."""

    def read(self, indices: object, output: object) -> object:
        """Return a tensor after consulting this bound view."""
        return np.full(1, self.count, dtype=np.float32)


@pytest.mark.parametrize("modern", [False, True])
@pytest.mark.parametrize("derived", [False, True])
def test_callback_cycles_are_collected(modern: bool, derived: bool) -> None:
    """Collect both duplicated legacy owners and single modern callback owners."""

    def make_cycle() -> object:
        view = DerivedView() if derived else t.EntityView()
        view.marker = Sentinel()
        marker = weakref.ref(view.marker)
        callback = view.read if derived else lambda indices, out: np.full(1, view.count, dtype=np.float32)
        spec = t.ArrayTensorSpec(dtype=wp.float32) if modern else t.TensorSpec()
        view._register_impl("read", t.ImplKind.Get, callback, spec)
        view.count = 7
        result = wp.from_dlpack(view.get_data("read"))
        np.testing.assert_array_equal(result.numpy(), [7])
        return marker

    marker = make_cycle()
    gc.collect()
    assert marker() is None


def test_external_callback_reference_keeps_view_live() -> None:
    """Keep captured views usable while their callback has an external root."""

    def make_callback() -> tuple[object, object]:
        view = t.EntityView()
        view.marker = Sentinel()
        callback = lambda indices, out: np.full(1, view.count, dtype=np.float32)
        view._register_impl("read", t.ImplKind.Get, callback, t.TensorSpec())
        view.count = 9
        return callback, weakref.ref(view.marker)

    callback, marker = make_callback()
    gc.collect()
    assert marker() is not None
    np.testing.assert_array_equal(callback(None, None), [9])
    del callback
    gc.collect()
    assert marker() is None


def test_callback_replacement_releases_previous_owner() -> None:
    """Drop replaced native and mirrored references without affecting new callbacks."""
    view = DerivedView()
    callback = view.read
    previous = weakref.ref(callback)
    view._register_impl("read", t.ImplKind.Get, callback, t.TensorSpec())
    del callback
    view._register_impl(
        "read",
        t.ImplKind.Get,
        lambda indices, out: np.ones(1, dtype=np.float32),
        t.ArrayTensorSpec(dtype=wp.float32),
    )
    gc.collect()
    assert previous() is None
    np.testing.assert_array_equal(wp.from_dlpack(view.get_data("read")).numpy(), [1])


def test_metadata_callback_cycle_is_collected() -> None:
    """Include metadata callbacks in native-owner traversal."""

    def make_cycle() -> object:
        view = t.EntityView()
        view.marker = Sentinel()
        marker = weakref.ref(view.marker)
        view._register_metadata("count", lambda: view.count)
        assert view.get_metadata("count") == 0
        return marker

    marker = make_cycle()
    gc.collect()
    assert marker() is None


@pytest.mark.parametrize("replace", [False, True])
def test_callback_gc_survives_dictionary_mutation(replace: bool) -> None:
    """Keep native callback edges visible after callers clear or replace __dict__."""

    def make_cycle() -> object:
        view = t.EntityView()
        callback = lambda: view.count
        reference = weakref.ref(callback)
        view._register_metadata("count", callback)
        if replace:
            view.__dict__ = {}
        else:
            view.__dict__.clear()
        assert view.get_metadata("count") == 0
        return reference

    reference = make_cycle()
    gc.collect()
    assert reference() is None
