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

"""Validate OvPhysX tensor registration and capability reporting.

The shared test setup registers the backend and its simulation-view factory
before these tests inspect the tensor registry.
"""

from __future__ import annotations

import _physics_setup  # noqa: F401  -- loads OvPhysxBackend + calls Startup
import isaacsim.physics.manager.impl.tensors as t
import pytest


class TestOvPhysxTensorsRegistration:
    """Validate OvPhysX engine, entity, and operation registration."""

    @classmethod
    def setup_class(cls) -> None:
        """Retain the registry populated by the OvPhysX test setup."""
        cls._registry = t.get_registry()

    def test_ovphysx_engine_registered(self) -> None:
        """Require ``ovphysx`` in the registered engine names."""
        assert "ovphysx" in self._registry.list_engines()

    def test_all_seven_entity_types_registered(self) -> None:
        """Require the exact seven OvPhysX entity types."""
        entities = sorted(self._registry.list_entities("ovphysx"))
        assert entities == [
            "articulation",
            "deformable-material",
            "rigid-body",
            "rigid-contact",
            "sdf-shape",
            "surface-deformable-body",
            "volume-deformable-body",
        ]

    @pytest.mark.parametrize(
        "entity",
        (
            "sdf-shape",
            "volume-deformable-body",
            "surface-deformable-body",
            "deformable-material",
        ),
    )
    def test_unsupported_views_report_supports_false(self, entity: str) -> None:
        """Require named read operations without callable implementations.

        Args:
            entity: Entity type whose registered read operations are unavailable.

        """
        view = t.create_entity("ovphysx", entity, ["/World/X"])
        impls = view.list_impls(t.ImplKind.Get)
        assert len(impls) > 0
        for impl in impls:
            assert not view.has_impl(impl, t.ImplKind.Get)

    def test_contact_entity_view_creatable(self) -> None:
        """Require a standalone contact view with named read operations."""
        # With no live simulation ovphysx returns a placeholder rigid-contact view. The real contact
        # tensors (net-contact-forces, contact-data, friction-data, ...) need a resolved handle; here
        # we just confirm the entity type resolves and registers its impl.
        view = t.create_entity("ovphysx", "rigid-contact", ["/World/X"])
        assert view is not None
        assert len(view.list_impls(t.ImplKind.Get)) > 0


class TestMultiEngineCoexistence:
    """Validate Newton and OvPhysX coexistence in one tensor registry."""

    @classmethod
    def setup_class(cls) -> None:
        """Register Newton and retain the registry already containing OvPhysX."""
        import isaacsim.physics_engines.ovnewton.impl.tensors  # noqa: F401  -- registers Newton factory

        cls._registry = t.get_registry()

    def test_both_engines_registered(self) -> None:
        """Require both backend names in the shared registry."""
        engines = set(self._registry.list_engines())
        assert "newton" in engines
        assert "ovphysx" in engines

    def test_per_engine_entity_lookup(self) -> None:
        """Require distinct Newton and OvPhysX SDF entity views."""
        n_view = t.create_entity("newton", "sdf-shape", ["/World/X"])
        o_view = t.create_entity("ovphysx", "sdf-shape", ["/World/X"])
        assert n_view is not o_view
