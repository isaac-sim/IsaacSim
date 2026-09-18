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

"""Exercise core tensor-registry behavior across backends.

Each test class is a mixin that
``backends/test_<engine>_tensors.py`` subclasses with the engine selected
via ``backend`` / ``frontend`` class attributes.

This module covers registry round-trips, entity registration, the supported
view matrix, and compatibility method delegation. Detailed operation contracts
are exercised by the scenario modules under ``common``.

"""

from __future__ import annotations

import isaacsim.physics.manager.impl.tensors as t
import pytest


class TensorRegistryBasicsMixin:
    """Backend-agnostic tests for entity-view creation and registry queries."""

    backend: str = "<override>"
    # Entity types that should report supports=False for this backend.
    # Override in subclasses as each engine adds real implementations.
    # Default unsupported list for Newton; ovphysx overrides to empty.
    unsupported_entity_types: tuple[str, ...] = (
        "sdf-shape",
        "volume-deformable-body",
        "surface-deformable-body",
        "deformable-material",
    )

    def test_engine_registered(self) -> None:
        """Verify that registry enumeration contains the selected backend."""
        engines = set(t.get_registry().list_engines())
        assert self.backend in engines

    def test_all_seven_entity_types_registered(self) -> None:
        """Verify that the backend registers all seven tensor entity types."""
        entities = set(t.get_registry().list_entities(self.backend))
        assert entities == {
            "articulation",
            "deformable-material",
            "rigid-body",
            "rigid-contact",
            "sdf-shape",
            "surface-deformable-body",
            "volume-deformable-body",
        }

    def test_unsupported_views_report_supports_false(self) -> None:
        """Verify that unsupported entity operations explicitly report no support."""
        for entity in self.unsupported_entity_types:
            view = t.create_entity(self.backend, entity, ["/World/X"])
            impls = view.list_impls(t.ImplKind.Get)
            assert len(impls) > 0, f"{self.backend}/{entity} did not register a get implementation"
            for impl in impls:
                assert not view.has_impl(
                    impl, t.ImplKind.Get
                ), f"{self.backend}/{entity} unexpectedly reports support for {impl!r}"

    def test_create_entity_rejects_empty_paths(self) -> None:
        """Verify that an empty pattern list is refused rather than resolved as an empty pattern."""
        with pytest.raises(ValueError):
            t.create_entity(self.backend, "articulation", [])

    def test_create_entity_rejects_unregistered_entity(self) -> None:
        """Verify that an unregistered entity name is refused rather than silently returning no view."""
        with pytest.raises(IndexError):
            t.create_entity(self.backend, "not-an-entity", ["/World/X"])


class LegacyMethodDelegationMixin:
    """Verify compatibility-method delegation on ``_LegacyAdapter`` subclasses.

    The wrapped adapter exposes explicit compatibility method names
    (`view.get_dof_positions()`, `view.set_dof_positions(data, indices)`,
    and similar operations). Its ``__getattr__`` implementation maps those
    calls to the string-keyed tensor API, such as
    ``view.get_data("dof-positions")``.
    """

    backend: str = "<override>"
    # Engine module that exposes a `_LegacyAdapter` for inspection.
    adapter_module: str = "<override>"

    def test_adapter_has_getattr_delegation(self) -> None:
        """Verify that the backend adapter defines compatibility-method delegation."""
        import importlib

        mod = importlib.import_module(self.adapter_module)
        adapter_cls = getattr(mod, "_LegacyAdapter")
        assert "__getattr__" in adapter_cls.__dict__, (
            f"{adapter_cls.__name__} must define __getattr__ to delegate "
            f"unknown attributes to the wrapped compatibility view"
        )
