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

"""Verify tensor-registry isolation with both backends loaded.

Verifies that loading both `isaacsim.physics_engines.ovnewton.impl.tensors` and
`isaacsim.physics_engines.ovphysx.tensors` into the same process produces two
independent (engine, entity-name) → factory maps with no cross-talk.
"""

from __future__ import annotations

import _physics_setup  # noqa: F401  -- auto-registers ovphysx via OvPhysxUmbrellaStartup
import isaacsim.physics.manager.impl.tensors as t
import isaacsim.physics_engines.ovnewton.impl.tensors  # noqa: F401  (registers)


class TestMultiBackendRegistryIsolation:
    """Verify tensor factory isolation between Newton and OvPhysX."""

    def test_both_engines_present(self) -> None:
        """Verify that both tensor engines appear in registry enumeration."""
        engines = set(t.get_registry().list_engines())
        assert "newton" in engines
        assert "ovphysx" in engines

    def test_per_engine_factories_are_distinct(self) -> None:
        """Verify that equal entity names create independent backend views."""
        # The same entity name resolves through different factories per
        # engine; the resulting views are independent objects.
        newton_view = t.create_entity("newton", "rigid-body", ["/World/X"])
        ovphysx_view = t.create_entity("ovphysx", "rigid-body", ["/World/X"])
        assert newton_view is not ovphysx_view

    def test_unregistering_one_engine_doesnt_affect_the_other(self) -> None:
        """Verify that unregistering a Newton factory leaves OvPhysX unchanged."""
        # Take a snapshot first.
        before_newton = set(t.get_registry().list_entities("newton"))
        before_ovphysx = set(t.get_registry().list_entities("ovphysx"))

        # Pretend to drop one entity from newton, then restore it.
        assert t.get_registry().unregister_entity("newton", "rigid-body")
        try:
            assert "rigid-body" not in set(t.get_registry().list_entities("newton"))
            assert "rigid-body" in set(t.get_registry().list_entities("ovphysx"))
        finally:
            # Restore newton's real factory so other tests see the registry they expect, not a stub that
            # would answer every later create_entity call for this entity.
            from isaacsim.physics_engines.ovnewton.impl.tensors.entity_factories import _make_rigid_body_factory

            t.get_registry().register_entity("newton", "rigid-body", _make_rigid_body_factory("newton"))

        # Sanity — both engines back to their pre-test entity sets.
        assert set(t.get_registry().list_entities("newton")) == before_newton
        assert set(t.get_registry().list_entities("ovphysx")) == before_ovphysx
