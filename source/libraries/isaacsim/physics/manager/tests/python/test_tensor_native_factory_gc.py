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

"""Verify collection of actual C++ factory views and externally retained owners."""

import gc
import weakref

import _isaacsim_tensor_native_fixture as native
import pytest
from isaacsim.physics.manager.bindings._bindings import PhysicsManager
from isaacsim.physics.manager.impl import tensors as t


@pytest.mark.parametrize("derived", [False, True])
@pytest.mark.parametrize("manager_route", [False, True])
@pytest.mark.parametrize("retain_native", [False, True])
def test_native_factory_callback_ownership(manager_route: bool, retain_native: bool, derived: bool) -> None:
    """Collect wrapper-only cycles after preserving any real external native root."""

    def make_cycle() -> object:
        native.install(retain_native, derived)
        if manager_route:
            view = PhysicsManager.get_instance().create_entity(
                engine="native_factory_gc_test", entity="view", paths=["/World"]
            )
        else:
            view = t.create_entity("native_factory_gc_test", "view", ["/World"])
        assert isinstance(view, t.EntityView)
        if retain_native:
            repeated = t.create_entity("native_factory_gc_test", "view", ["/World"])
            assert repeated is view
        view.count = 31
        callback = lambda: view.count
        reference = weakref.ref(callback)
        view._register_metadata("count", callback)
        assert view.get_metadata("count") == 31
        assert native.owner_count() == (2 if retain_native else 1)
        return reference

    try:
        reference = make_cycle()
        gc.collect()
        if retain_native:
            assert reference() is not None
            assert native.owner_count() == 2
            assert native.read_metadata() == 31
            native.release()
            gc.collect()
        assert reference() is None
        assert native.owner_count() == 0
    finally:
        native.reset()
        gc.collect()
