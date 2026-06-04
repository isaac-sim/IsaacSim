# SPDX-FileCopyrightText: Copyright (c) 2021-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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

"""Implements a wrapper for PhysX particle system functionality in Isaac Sim."""

from __future__ import annotations

from collections.abc import Sequence

import carb
import isaacsim.core.utils.prims as prim_utils
import isaacsim.core.utils.stage as stage_utils
from pxr import PhysxSchema, Usd

from .particle_system import ParticleSystem


class SingleParticleSystem:
    """A wrapper around PhysX particle system.

    PhysX uses GPU-accelerated position-based-dynamics (PBD) particle simulation [1]. The particle system
    can be used to simulate fluids, cloth and inflatables [2].

    The wrapper is useful for creating and setting solver parameters common to the particle objects
    associated with the system. The particle system's solver parameters cannot be changed once the scene
    is playing.

    Initializes and Applies PhysxSchema.PhysxParticleSystem to the prim at prim_path.

    All arguments are accepted as :obj:`None`. In this case, they either have the default values from
    `PhysxParticleSystem` schema (in case a new particle system is created), or the values present in the
    existing particle system.

    Note:
        CPU simulation of particles is not supported. PhysX must be simulated with GPU enabled.

    Reference:
        [1] https://mmacklin.com/pbf_sig_preprint.pdf
        [2] https://docs.omniverse.nvidia.com/prod_extensions/prod_extensions/ext_physics.html#particle-simulation

    Args:
        prim_path: The path to the particle system.
        name: Name given to the prim when instantiating it.
        particle_system_enabled: Whether to enable or disable the particle system.
        simulation_owner: Single PhysicsScene that simulates this particle system.
        contact_offset: Contact offset used for collisions with non-particle objects such as rigid or deformable
            bodies.
        rest_offset: Rest offset used for collisions with non-particle objects such as rigid or deformable bodies.
        particle_contact_offset: Contact offset used for interactions between particles.
            Must be larger than solid and fluid rest offsets.
        solid_rest_offset: Rest offset used for solid-solid or solid-fluid particle interactions.
            Must be smaller than particle contact offset.
        fluid_rest_offset: Rest offset used for fluid-fluid particle interactions.
            Must be smaller than particle contact offset.
        enable_ccd: Enable continuous collision detection for particles to help avoid tunneling effects.
        solver_position_iteration_count: Number of solver iterations for position.
        max_depenetration_velocity: The maximum velocity permitted to be introduced by the solver to depenetrate
            intersecting particles.
        wind: The wind applied to the current particle system.
        max_neighborhood: The particle neighborhood size.
        max_velocity: Maximum particle velocity.
        global_self_collision_enabled: If True, self collisions follow particle-object-specific settings.
            If False, all particle self collisions are disabled, regardless of any other settings.
            Improves performance if self collisions are not needed.
        non_particle_collision_enabled: Enable or disable particle collision with non-particle objects for all
            particles in the system. Improves performance if non-particle collisions are not needed.
    """

    def __init__(
        self,
        prim_path: str,
        name: str | None = "particle_system",
        particle_system_enabled: bool | None = None,
        simulation_owner: str | None = None,
        contact_offset: float | None = None,
        rest_offset: float | None = None,
        particle_contact_offset: float | None = None,
        solid_rest_offset: float | None = None,
        fluid_rest_offset: float | None = None,
        enable_ccd: bool | None = None,
        solver_position_iteration_count: float | None = None,
        max_depenetration_velocity: float | None = None,
        wind: Sequence[float] = None,
        max_neighborhood: int | None = None,
        max_velocity: float | None = None,
        global_self_collision_enabled: bool | None = None,
        non_particle_collision_enabled: bool | None = None,
    ) -> None:
        # store constants
        from isaacsim.core.simulation_manager import SimulationManager

        self._backend = SimulationManager.get_backend()
        self._device = SimulationManager.get_physics_sim_device()
        self._backend_utils = SimulationManager._get_backend_utils()

        self._name = name
        self._prim_path = prim_path
        # check whether to create or apply particle system
        if prim_utils.is_prim_path_valid(prim_path):
            carb.log_info(f"Physics particle system prim is already defined at path {self._prim_path}.")
            self._prim = prim_utils.get_prim_at_path(prim_path)
            self._particle_system = PhysxSchema.PhysxParticleSystem(self._prim)
        else:
            stage = stage_utils.get_current_stage()
            self._particle_system = PhysxSchema.PhysxParticleSystem.Define(stage, self._prim_path)
            self._prim = prim_utils.get_prim_at_path(prim_path)

        if particle_system_enabled is not None:
            particle_system_enabled = self._backend_utils.create_tensor_from_list(
                [particle_system_enabled], dtype="bool", device=self._device
            )
        if simulation_owner is not None:
            simulation_owner = [simulation_owner]
        if contact_offset is not None:
            contact_offset = self._backend_utils.create_tensor_from_list(
                [contact_offset], dtype="float32", device=self._device
            )
        if rest_offset is not None:
            rest_offset = self._backend_utils.create_tensor_from_list(
                [rest_offset], dtype="float32", device=self._device
            )
        if particle_contact_offset is not None:
            particle_contact_offset = self._backend_utils.create_tensor_from_list(
                [particle_contact_offset], dtype="float32", device=self._device
            )
        if solid_rest_offset is not None:
            solid_rest_offset = self._backend_utils.create_tensor_from_list(
                [solid_rest_offset], dtype="float32", device=self._device
            )
        if fluid_rest_offset is not None:
            fluid_rest_offset = self._backend_utils.create_tensor_from_list(
                [fluid_rest_offset], dtype="float32", device=self._device
            )
        if enable_ccd is not None:
            enable_ccd = self._backend_utils.create_tensor_from_list([enable_ccd], dtype="bool", device=self._device)
        if solver_position_iteration_count is not None:
            solver_position_iteration_count = self._backend_utils.create_tensor_from_list(
                [solver_position_iteration_count], dtype="int32", device=self._device
            )
        if max_depenetration_velocity is not None:
            max_depenetration_velocity = self._backend_utils.create_tensor_from_list(
                [max_depenetration_velocity], dtype="float32", device=self._device
            )
        if wind is not None:
            wind = self._backend_utils.create_tensor_from_list([wind], dtype="float32", device=self._device)
        if max_neighborhood is not None:
            max_neighborhood = self._backend_utils.create_tensor_from_list(
                [max_neighborhood], dtype="int32", device=self._device
            )
        if max_velocity is not None:
            max_velocity = self._backend_utils.create_tensor_from_list(
                [max_velocity], dtype="float32", device=self._device
            )
        if global_self_collision_enabled is not None:
            global_self_collision_enabled = self._backend_utils.create_tensor_from_list(
                [global_self_collision_enabled], dtype="bool", device=self._device
            )
        if non_particle_collision_enabled is not None:
            non_particle_collision_enabled = self._backend_utils.create_tensor_from_list(
                [non_particle_collision_enabled], dtype="bool", device=self._device
            )

        self._particle_system_view = ParticleSystem(
            prim_paths_expr=prim_path,
            particle_systems_enabled=particle_system_enabled,
            simulation_owners=simulation_owner,
            contact_offsets=contact_offset,
            rest_offsets=rest_offset,
            particle_contact_offsets=particle_contact_offset,
            solid_rest_offsets=solid_rest_offset,
            fluid_rest_offsets=fluid_rest_offset,
            enable_ccds=enable_ccd,
            solver_position_iteration_counts=solver_position_iteration_count,
            max_depenetration_velocities=max_depenetration_velocity,
            winds=wind,
            max_neighborhoods=max_neighborhood,
            max_velocities=max_velocity,
            global_self_collisions_enabled=global_self_collision_enabled,
        )

    """
    Properties.
    """

    @property
    def prim_path(self) -> str:
        """Stage path to the particle system.

        Returns:
            The stage path to the particle system.
        """
        return self._prim_path

    @property
    def prim(self) -> Usd.Prim:
        """USD prim of the particle system.

        Returns:
            The USD prim present.
        """
        return self._prim

    @property
    def particle_system(self) -> PhysxSchema.PhysxParticleSystem:
        """PhysX particle system schema.

        Returns:
            The particle system.
        """
        return self._particle_system

    @property
    def name(self) -> str | None:
        """Name given to the prim when instantiating it.

        Returns:
            Name given to the prim when instantiating it. Otherwise None.
        """
        return self._name

    def initialize(self, physics_sim_view: object = None) -> None:
        """Initializes the particle system.

        Args:
            physics_sim_view: Physics simulation view to initialize with.
        """
        self._particle_system_view.initialize(physics_sim_view=physics_sim_view)
        return

    def is_valid(self) -> bool:
        """Checks if the particle system prim is valid.

        Returns:
            True if the current prim path corresponds to a valid prim in stage. False otherwise.
        """
        return self._particle_system_view.is_valid()

    def post_reset(self) -> None:
        """Resets the particle system to its initial state."""
        self._particle_system_view.post_reset()
        return

    def apply_particle_material(self, particle_materials: "ParticleMaterial") -> None:
        """Applies particle material to the particle system.

        Args:
            particle_materials: The particle material to apply.
        """
        self._particle_system_view.apply_particle_materials(particle_materials)

    def get_applied_particle_material(self) -> "ParticleMaterial":
        """Gets the applied particle material from the particle system.

        Returns:
            The applied particle material.
        """
        return self._particle_system_view.get_applied_particle_materials()[0]

    """
    Operations - Setters.
    """

    def set_particle_system_enabled(self, value: bool) -> None:
        """Set enabling of the particle system.

        Args:
            value: Whether to enable or disable.
        """
        value = self._backend_utils.convert(value, device=self._device)
        value = self._backend_utils.expand_dims(value, 0)
        self._particle_system_view.set_particle_systems_enabled(value)

    def set_simulation_owner(self, value: str) -> None:
        """Set the PhysicsScene that simulates this particle system.

        Args:
            value: The prim path to the physics scene.
        """
        self._particle_system_view.set_simulation_owners([value])

    def set_contact_offset(self, value: float) -> None:
        """Set the contact offset used for collisions with non-particle objects such as rigid or deformable bodies.

        Args:
            value: The contact offset.
        """
        value = self._backend_utils.convert(value, device=self._device)
        value = self._backend_utils.expand_dims(value, 0)
        self._particle_system_view.set_contact_offsets(value)

    def set_rest_offset(self, value: float) -> None:
        """Set the rest offset used for collisions with non-particle objects such as rigid or deformable bodies.

        Args:
            value: The rest offset.
        """
        value = self._backend_utils.convert(value, device=self._device)
        value = self._backend_utils.expand_dims(value, 0)
        self._particle_system_view.set_rest_offsets(value)

    def set_particle_contact_offset(self, value: float) -> None:
        """Set the contact offset used for interactions between particles.

        Note:
            Must be larger than solid and fluid rest offsets.

        Args:
            value: The contact offset.
        """
        value = self._backend_utils.convert(value, device=self._device)
        value = self._backend_utils.expand_dims(value, 0)
        self._particle_system_view.set_particle_contact_offsets(value)

    def set_solid_rest_offset(self, value: float) -> None:
        """Set the rest offset used for solid-solid or solid-fluid particle interactions.

        Note:
            Must be smaller than particle contact offset.

        Args:
            value: The rest offset.
        """
        value = self._backend_utils.convert(value, device=self._device)
        value = self._backend_utils.expand_dims(value, 0)
        self._particle_system_view.set_solid_rest_offsets(value)

    def set_fluid_rest_offset(self, value: float) -> None:
        """Set the rest offset used for fluid-fluid particle interactions.

        Note:
            Must be smaller than particle contact offset.

        Args:
            value: The rest offset.
        """
        value = self._backend_utils.convert(value, device=self._device)
        value = self._backend_utils.expand_dims(value, 0)
        self._particle_system_view.set_fluid_rest_offsets(value)

    def set_enable_ccd(self, value: bool) -> None:
        """Enable continuous collision detection for particles.

        Args:
            value: Whether to enable or disable.
        """
        value = self._backend_utils.convert(value, device=self._device)
        value = self._backend_utils.expand_dims(value, 0)
        self._particle_system_view.set_enable_ccds(value)

    def set_solver_position_iteration_count(self, value: int) -> None:
        """Set the number of solver iterations for position.

        Args:
            value: Number of solver iterations.
        """
        value = self._backend_utils.convert(value, device=self._device)
        value = self._backend_utils.expand_dims(value, 0)
        self._particle_system_view.set_solver_position_iteration_counts(value)

    def set_max_depenetration_velocity(self, value: float) -> None:
        """Set the maximum velocity permitted to be introduced by the solver to.

            depenetrate intersecting particles.

        Args:
            value: The maximum depenetration velocity.
        """
        value = self._backend_utils.convert(value, device=self._device)
        value = self._backend_utils.expand_dims(value, 0)
        self._particle_system_view.set_max_depenetration_velocities(value)

    def set_wind(self, value: Sequence[float]) -> None:
        """Set the wind velocity applied to the current particle system.

        Args:
            value: The wind applied to the current particle system.
        """
        value = self._backend_utils.convert(value, device=self._device)
        value = self._backend_utils.expand_dims(value, 0)
        self._particle_system_view.set_winds(value)

    def set_max_neighborhood(self, value: int) -> None:
        """Set the particle neighborhood size.

        Args:
            value: The neighborhood size.
        """
        value = self._backend_utils.convert(value, device=self._device)
        value = self._backend_utils.expand_dims(value, 0)
        self._particle_system_view.set_max_neighborhoods(value)

    def set_max_velocity(self, value: float) -> None:
        """Set the maximum particle velocity.

        Args:
            value: The maximum velocity.
        """
        value = self._backend_utils.convert(value, device=self._device)
        value = self._backend_utils.expand_dims(value, 0)
        self._particle_system_view.set_max_velocities(value)

    def set_global_self_collision_enabled(self, value: bool) -> None:
        """Enable self collisions to follow particle-object-specific settings.

        If True, self collisions follow particle-object-specific settings. If False,
        all particle self collisions are disabled, regardless of any other settings.

        Note: Improves performance if self collisions are not needed.

        Args:
            value: Whether to enable or disable.
        """
        value = self._backend_utils.convert(value, device=self._device)
        value = self._backend_utils.expand_dims(value, 0)
        self._particle_system_view.set_global_self_collisions_enabled(value)

    """
    Operations - Getters.
    """

    def get_particle_system_enabled(self) -> bool:
        """Whether particle system is enabled.

        Returns:
            Whether particle system is enabled or not.
        """
        return self._particle_system_view.get_particle_systems_enabled()[0]

    def get_simulation_owner(self) -> Usd.Prim:
        """The physics scene prim attached to particle system.

        Returns:
            The physics scene prim attached to particle system.
        """
        return self._particle_system_view.get_simulation_owners()[0]

    def get_contact_offset(self) -> float:
        """The contact offset used for collisions with non-particle objects.

        Returns:
            The contact offset used for collisions with non-particle objects.
        """
        return self._particle_system_view.get_contact_offsets()[0]

    def get_rest_offset(self) -> float:
        """The rest offset used for collisions with non-particle objects.

        Returns:
            The rest offset used for collisions with non-particle objects.
        """
        return self._particle_system_view.get_rest_offsets()[0]

    def get_particle_contact_offset(self) -> float:
        """The contact offset used for interactions between particles.

        Returns:
            The contact offset used for interactions between particles.
        """
        return self._particle_system_view.get_particle_contact_offsets()[0]

    def get_solid_rest_offset(self) -> float:
        """The rest offset used for solid-solid or solid-fluid particle interactions.

        Returns:
            The rest offset used for solid-solid or solid-fluid particle interactions.
        """
        return self._particle_system_view.get_solid_rest_offsets()[0]

    def get_fluid_rest_offset(self) -> float:
        """The rest offset used for fluid-fluid particle interactions.

        Returns:
            The rest offset used for fluid-fluid particle interactions.
        """
        return self._particle_system_view.get_fluid_rest_offsets()[0]

    def get_enable_ccd(self) -> bool:
        """Whether continuous collision detection for particles is enabled or disabled.

        Returns:
            Whether continuous collision detection for particles is enabled or disabled.
        """
        return self._particle_system_view.get_enable_ccds()[0]

    def get_solver_position_iteration_count(self) -> int:
        """The number of solver iterations for positions.

        Returns:
            The number of solver iterations for positions.
        """
        return self._particle_system_view.get_solver_position_iteration_counts()[0]

    def get_max_depenetration_velocity(self) -> float:
        """The maximum velocity permitted between intersecting particles.

        Returns:
            The maximum velocity permitted between intersecting particles.
        """
        return self._particle_system_view.get_max_depenetration_velocities()[0]

    def get_wind(self) -> Sequence[float]:
        """The wind applied to the current particle system.

        Returns:
            The wind applied to the current particle system.
        """
        return self._particle_system_view.get_winds()[0].tolist()

    def get_max_neighborhood(self) -> int:
        """The particle neighborhood size.

        Returns:
            The particle neighborhood size.
        """
        return self._particle_system_view.get_max_neighborhoods()[0]

    def get_max_velocity(self) -> float:
        """The maximum particle velocity.

        Returns:
            The maximum particle velocity.
        """
        return self._particle_system_view.get_max_velocities()[0]

    def get_global_self_collision_enabled(self) -> bool:
        """Whether self collisions to follow particle-object-specific settings is enabled or disabled.

        Returns:
            Whether self collisions to follow particle-object-specific settings
                is enabled or disabled.
        """
        return self._particle_system_view.get_global_self_collisions_enabled()[0]

    """
    Operations - WIP.
    """

    def apply_particle_anisotropy(self) -> PhysxSchema.PhysxParticleAnisotropyAPI:
        """Applies anisotropy to the particle system.

        This is used to compute anisotropic scaling of particles in a post-processing step.
        It only affects the rendering output including iso-surface generation.

        Returns:
            The applied anisotropy API schema.
        """
        return PhysxSchema.PhysxParticleAnisotropyAPI.Apply(self._prim)

    def apply_particle_smoothing(self) -> PhysxSchema.PhysxParticleSmoothingAPI:
        """Applies smoothing to the simulated particle system.

        This is used to control smoothing of particles in a post-processing step.
        It only affects the rendering output including iso-surface generation.

        Returns:
            The applied smoothing API schema.
        """
        return PhysxSchema.PhysxParticleSmoothingAPI.Apply(self._prim)

    def apply_particle_isotropy(self) -> PhysxSchema.PhysxParticleAnisotropyAPI:
        """Applies iso-surface extraction to the particle system.

        This is used to define settings to extract an iso-surface from the particles
        in a post-processing step. It only affects the rendering output including iso-surface generation.

        Returns:
            The applied anisotropy API schema.
        """
        return PhysxSchema.PhysxParticleAnisotropyAPI.Apply(self._prim)
