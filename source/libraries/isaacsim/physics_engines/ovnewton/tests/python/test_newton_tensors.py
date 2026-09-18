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

"""Validate Newton tensor registration, dispatch, and shape guards.

Importing ``isaacsim.physics_engines.ovnewton.impl.tensors`` must register all
seven entity types. Unsupported entity operations expose no implementations,
while the supported adapters enforce their padded tensor-shape contracts. The
CPU and GPU data-operation matrix is covered by ``test_get_set_contract.py``.
"""

from __future__ import annotations

from types import SimpleNamespace

import _physics_setup  # noqa: F401
import isaacsim.physics.manager.impl.tensors as t
import pytest
import warp as wp


class TestNewtonTensorsRegistration:
    """Validate Newton entity registration and capability reporting."""

    @classmethod
    def setup_class(cls) -> None:
        """Load the Newton tensor adapters and retain the populated registry."""
        # Import as a side effect; registration happens at module load.
        import isaacsim.physics_engines.ovnewton.impl.tensors  # noqa: F401

        cls._registry = t.get_registry()

    def test_newton_engine_registered(self) -> None:
        """Verify that the Newton engine is registered."""
        assert "newton" in self._registry.list_engines()

    def test_all_seven_entity_types_registered(self) -> None:
        """Verify that Newton registers the exact seven public entity types."""
        entities = sorted(self._registry.list_entities("newton"))
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
    def test_unported_views_report_no_support(self, entity: str) -> None:
        """Verify that unsupported entity views say so rather than coming back empty.

        An empty view is indistinguishable from a broken registration, so these register a read
        implementation that reports ``supports=False``.

        Args:
            entity: Registered entity type without Newton tensor operations.

        """
        view = t.create_entity("newton", entity, ["/World/X"])
        implementations = view.list_impls(t.ImplKind.Get)
        assert implementations, f"newton/{entity} registered no get implementation"
        for implementation in implementations:
            assert not view.get_impl_spec(implementation, t.ImplKind.Get).supports


class TestNewtonArticulationShapeContracts:
    """Validate Newton articulation padding and variable-width shape contracts."""

    def test_max_links_uses_padded_backend_width(self) -> None:
        """Verify that ``max_links`` reports the padded backend map width."""
        from isaacsim.physics_engines.ovnewton.impl.tensors.backend import ArticulationSet

        backend = object.__new__(ArticulationSet)
        backend.link_indices = SimpleNamespace(shape=(2, 4))
        backend.meta_types = [SimpleNamespace(link_count=2), SimpleNamespace(link_count=4)]

        assert backend.max_links == 4

    def test_variable_dof_width_does_not_advertise_dense_row_operations(self) -> None:
        """Verify heterogeneous DOF counts disable dense DOF-row operations.

        Link-level inverse-mass reads remain available because their shape does
        not depend on a uniform DOF width.
        """
        from isaacsim.physics_engines.ovnewton.impl.tensors.entity_factories import _NewtonArticulationViewAdapter

        device = wp.get_device("cpu")
        legacy = SimpleNamespace(
            count=2,
            max_dofs=3,
            max_links=4,
            prim_paths=["/World/A", "/World/B"],
            _model=SimpleNamespace(device=device, max_joints_per_articulation=3, max_dofs_per_articulation=3),
            _backend=SimpleNamespace(
                meta_types=[SimpleNamespace(dof_count=1), SimpleNamespace(dof_count=3)],
                dof_position_indices=SimpleNamespace(shape=(4,)),
                dof_velocity_indices=SimpleNamespace(shape=(4,)),
                dof_axis_indices=SimpleNamespace(shape=(4,)),
            ),
            get_dof_positions=lambda: None,
            set_dof_positions=lambda data, indices: None,
            get_dof_velocities=lambda: None,
            set_dof_velocities=lambda data, indices: None,
            get_dof_stiffnesses=lambda: None,
            set_dof_stiffnesses=lambda data, indices: None,
            get_dof_dampings=lambda: None,
            set_dof_dampings=lambda data, indices: None,
            get_dof_armatures=lambda: None,
            set_dof_armatures=lambda data, indices: None,
            get_dof_max_forces=lambda: None,
            set_dof_max_forces=lambda data, indices: None,
            get_dof_limits=lambda: None,
            set_dof_limits=lambda data, indices: None,
            get_inv_masses=lambda: None,
            get_root_velocities=lambda: None,
            set_root_velocities=lambda data, indices: None,
            get_masses=lambda: None,
            set_masses=lambda data, indices: None,
            get_coms=lambda: None,
            set_coms=lambda data, indices: None,
            get_inertias=lambda: None,
            set_inertias=lambda data, indices: None,
            get_inv_inertias=lambda: None,
            get_link_transforms=lambda: None,
            get_link_velocities=lambda: None,
            get_jacobians=lambda: None,
            get_generalized_mass_matrices=lambda: None,
            get_root_transforms=lambda: None,
            set_root_transforms=lambda data, indices: None,
            _newton_stage=SimpleNamespace(solver_is_maximal=True),
        )

        view = _NewtonArticulationViewAdapter(legacy, legacy.prim_paths)

        assert view._warp_dispatch_attached
        assert not (view.has_impl("dof-positions", t.ImplKind.Get))
        assert not (view.has_impl("dof-velocities", t.ImplKind.Get))
        assert not (view.has_impl("dof-stiffnesses", t.ImplKind.Get))
        assert view.has_impl("inv-masses", t.ImplKind.Get)

    def test_each_dense_dof_operation_checks_its_actual_map_width(self) -> None:
        """Verify each dense DOF operation checks its own index-map width.

        An oversized position map disables position access without suppressing
        velocity, stiffness, damping, armature, or force-limit operations whose
        maps remain dense.
        """
        from isaacsim.physics_engines.ovnewton.impl.tensors.entity_factories import _NewtonArticulationViewAdapter

        device = wp.get_device("cpu")
        legacy = SimpleNamespace(
            count=2,
            max_dofs=3,
            max_links=4,
            prim_paths=["/World/A", "/World/B"],
            _model=SimpleNamespace(device=device, max_joints_per_articulation=3, max_dofs_per_articulation=3),
            _backend=SimpleNamespace(
                meta_types=[SimpleNamespace(dof_count=3), SimpleNamespace(dof_count=3)],
                dof_position_indices=SimpleNamespace(shape=(8,)),
                dof_velocity_indices=SimpleNamespace(shape=(6,)),
                dof_axis_indices=SimpleNamespace(shape=(6,)),
            ),
            get_dof_positions=lambda: None,
            set_dof_positions=lambda data, indices: None,
            get_dof_velocities=lambda: None,
            set_dof_velocities=lambda data, indices: None,
            get_dof_position_targets=lambda: None,
            set_dof_position_targets=lambda data, indices: None,
            get_dof_velocity_targets=lambda: None,
            set_dof_velocity_targets=lambda data, indices: None,
            get_dof_actuation_forces=lambda: None,
            set_dof_actuation_forces=lambda data, indices: None,
            get_dof_stiffnesses=lambda: None,
            set_dof_stiffnesses=lambda data, indices: None,
            get_dof_dampings=lambda: None,
            set_dof_dampings=lambda data, indices: None,
            get_dof_armatures=lambda: None,
            set_dof_armatures=lambda data, indices: None,
            get_dof_max_forces=lambda: None,
            set_dof_max_forces=lambda data, indices: None,
            get_dof_limits=lambda: None,
            set_dof_limits=lambda data, indices: None,
            get_inv_masses=lambda: None,
            get_root_velocities=lambda: None,
            set_root_velocities=lambda data, indices: None,
            get_masses=lambda: None,
            set_masses=lambda data, indices: None,
            get_coms=lambda: None,
            set_coms=lambda data, indices: None,
            get_inertias=lambda: None,
            set_inertias=lambda data, indices: None,
            get_inv_inertias=lambda: None,
            get_link_transforms=lambda: None,
            get_link_velocities=lambda: None,
            get_jacobians=lambda: None,
            get_generalized_mass_matrices=lambda: None,
            get_root_transforms=lambda: None,
            set_root_transforms=lambda data, indices: None,
            _newton_stage=SimpleNamespace(solver_is_maximal=True),
        )

        view = _NewtonArticulationViewAdapter(legacy, legacy.prim_paths)

        assert not (view.has_impl("dof-positions", t.ImplKind.Get))
        assert view.has_impl("dof-velocities", t.ImplKind.Get)
        assert view.has_impl("dof-stiffnesses", t.ImplKind.Get)
        assert view.has_impl("dof-dampings", t.ImplKind.Get)
        assert view.has_impl("dof-armatures", t.ImplKind.Get)
        assert view.has_impl("dof-max-forces", t.ImplKind.Get)

    def test_nondense_velocity_map_gates_only_velocity_operation(self) -> None:
        """Verify a non-dense velocity map disables only velocity access."""
        from isaacsim.physics_engines.ovnewton.impl.tensors.entity_factories import _NewtonArticulationViewAdapter

        device = wp.get_device("cpu")
        legacy = SimpleNamespace(
            count=2,
            max_dofs=3,
            max_links=4,
            prim_paths=["/World/A", "/World/B"],
            _model=SimpleNamespace(device=device, max_joints_per_articulation=3, max_dofs_per_articulation=3),
            _backend=SimpleNamespace(
                meta_types=[SimpleNamespace(dof_count=3), SimpleNamespace(dof_count=3)],
                dof_position_indices=SimpleNamespace(shape=(6,)),
                dof_velocity_indices=SimpleNamespace(shape=(4,)),
                dof_axis_indices=SimpleNamespace(shape=(6,)),
            ),
            get_dof_positions=lambda: None,
            set_dof_positions=lambda data, indices: None,
            get_dof_velocities=lambda: None,
            set_dof_velocities=lambda data, indices: None,
            get_dof_position_targets=lambda: None,
            set_dof_position_targets=lambda data, indices: None,
            get_dof_velocity_targets=lambda: None,
            set_dof_velocity_targets=lambda data, indices: None,
            get_dof_actuation_forces=lambda: None,
            set_dof_actuation_forces=lambda data, indices: None,
            get_dof_stiffnesses=lambda: None,
            set_dof_stiffnesses=lambda data, indices: None,
            get_dof_dampings=lambda: None,
            set_dof_dampings=lambda data, indices: None,
            get_dof_armatures=lambda: None,
            set_dof_armatures=lambda data, indices: None,
            get_dof_max_forces=lambda: None,
            set_dof_max_forces=lambda data, indices: None,
            get_dof_limits=lambda: None,
            set_dof_limits=lambda data, indices: None,
            get_inv_masses=lambda: None,
            get_root_velocities=lambda: None,
            set_root_velocities=lambda data, indices: None,
            get_masses=lambda: None,
            set_masses=lambda data, indices: None,
            get_coms=lambda: None,
            set_coms=lambda data, indices: None,
            get_inertias=lambda: None,
            set_inertias=lambda data, indices: None,
            get_inv_inertias=lambda: None,
            get_link_transforms=lambda: None,
            get_link_velocities=lambda: None,
            get_jacobians=lambda: None,
            get_generalized_mass_matrices=lambda: None,
            get_root_transforms=lambda: None,
            set_root_transforms=lambda data, indices: None,
            _newton_stage=SimpleNamespace(solver_is_maximal=True),
        )

        view = _NewtonArticulationViewAdapter(legacy, legacy.prim_paths)

        assert view.has_impl("dof-positions", t.ImplKind.Get)
        assert not (view.has_impl("dof-velocities", t.ImplKind.Get))
        assert view.has_impl("dof-stiffnesses", t.ImplKind.Get)


class TestNamedSimulationInstances:
    """Validate that distinctly named Newton simulations register and resolve independently."""

    def test_named_simulations_register_their_own_factories(self) -> None:
        """Verify each named simulation gets its own factories and resolves without ambiguity.

        Registering a distinct name per simulation is what allows several to coexist: the entity factories
        resolve their stage by exact name match, so nothing is inferred and nothing is guessed between them.
        """
        import isaacsim.physics.registration as physics_registration
        import isaacsim.physics_engines.ovnewton.impl as newton_backend

        first_id = newton_backend.register(simulation_name="newton-a")
        try:
            second_id = newton_backend.register(simulation_name="newton-b")
            try:
                registry = t.get_registry()
                simulations = set(registry.list_simulations())
                assert {"newton-a", "newton-b"} <= simulations, f"named simulations missing from {sorted(simulations)}"

                # Engines and simulations are separate: naming simulations does not invent engines, and
                # newton stays one engine however many of its simulations are running.
                engines = set(registry.list_engines())
                assert "newton" in engines
                assert not ({"newton-a", "newton-b"} & engines), f"simulations leaked into engines: {sorted(engines)}"

                # Both are active at once, and each name resolves to its own simulation rather than to
                # whichever the registry happened to yield first.
                physics_registration.activate_simulation(first_id)
                physics_registration.activate_simulation(second_id)
                assert physics_registration.get_active_simulation_id("newton-a") == first_id
                assert physics_registration.get_active_simulation_id("newton-b") == second_id

                # Each name carries the full entity set, and create_entity accepts the instance name.
                # sdf-shape is used because it needs no model: these simulations are registered but never
                # started, and Newton's implemented views require an initialized model to build.
                for name in ("newton-a", "newton-b"):
                    assert len(registry.list_entities(name)) == 7, f"{name} did not register all entity types"
                    view = t.create_entity(name, "sdf-shape", ["/World/X"])
                    assert view.list_impls(t.ImplKind.Get), f"create_entity({name!r}) returned an unusable view"
            finally:
                newton_backend.unregister(second_id)
        finally:
            newton_backend.unregister(first_id)

        # Teardown removes what registration added, so dead names do not accumulate.
        remaining = set(t.get_registry().list_simulations())
        assert not ({"newton-a", "newton-b"} & remaining), f"factories outlived their simulations: {sorted(remaining)}"

    def test_registering_under_the_default_name_leaves_it_intact(self) -> None:
        """Verify a simulation never tears down factories it did not register.

        Passing the default name explicitly is the same request as passing nothing, and the factories for
        that name are registered at import by whoever imported the module. A simulation that did not create
        them must not remove them on teardown, or ``create_entity`` would stop working process-wide for
        everyone once that simulation ended.
        """
        import isaacsim.physics_engines.ovnewton.impl as newton_backend
        from isaacsim.physics_engines.ovnewton.impl.tensors import DEFAULT_SIMULATION_NAME

        simulation_id = newton_backend.register(simulation_name=DEFAULT_SIMULATION_NAME)
        newton_backend.unregister(simulation_id)

        registry = t.get_registry()
        assert DEFAULT_SIMULATION_NAME in set(registry.list_simulations())
        assert len(registry.list_entities(DEFAULT_SIMULATION_NAME)) == 7

        # And the path a consumer actually uses still resolves.
        view = t.create_entity(DEFAULT_SIMULATION_NAME, "articulation", ["/World/X"])
        assert view.list_impls(t.ImplKind.Get), "createEntity stopped working after an unrelated teardown"
