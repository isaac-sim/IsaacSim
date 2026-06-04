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

"""Provides high-level functionality for managing particle systems in Isaac Sim."""

from __future__ import annotations

from collections.abc import Sequence
from typing import TYPE_CHECKING

import carb
import carb.eventdispatcher
import numpy as np

# isaac-core
import omni.kit.app
import omni.timeline
from isaacsim.core.deprecation_manager import import_module
from isaacsim.core.utils.prims import find_matching_prim_paths, get_prim_at_path, is_prim_path_valid

if TYPE_CHECKING:
    from isaacsim.core.api.materials.particle_material import ParticleMaterial

# omniverse
from pxr import UsdShade

torch = import_module("torch")


class ParticleSystem:
    """Provides high level functions to deal with particle systems (1 or more particle systems) as well as its attributes/ properties.

    This object wraps all matching particle systems found at the regex provided at the prim_paths_expr.
    Note: not all the attributes of the PhysxSchema.PhysxParticleSystem is currently controlled with this view class
    Tensor API support will be added in the future to extend the functionality of this class to applications beyond cloth.

    Args:
        prim_paths_expr: Prim paths regex to encapsulate all prims that match it.
        name: Shortname to be used as a key by Scene class.
        particle_systems_enabled: Whether to enable or disable the particle system.
        simulation_owners: Single PhysicsScene that simulates this particle system.
        contact_offsets: Contact offset used for collisions with non-particle objects such as rigid or deformable
            bodies.
        rest_offsets: Rest offset used for collisions with non-particle objects such as rigid or deformable bodies.
        particle_contact_offsets: Contact offset used for interactions between particles.
            Must be larger than solid and fluid rest offsets.
        solid_rest_offsets: Rest offset used for solid-solid or solid-fluid particle interactions.
            Must be smaller than particle contact offset.
        fluid_rest_offsets: Rest offset used for fluid-fluid particle interactions.
            Must be smaller than particle contact offset.
        enable_ccds: Enable continuous collision detection for particles to help avoid tunneling effects.
        solver_position_iteration_counts: Number of solver iterations for position.
        max_depenetration_velocities: The maximum velocity permitted to be introduced by the solver to depenetrate
            intersecting particles.
        winds: The wind applied to the current particle system.
        max_neighborhoods: The particle neighborhood size.
        max_velocities: Maximum particle velocity.
        global_self_collisions_enabled: If True, self collisions follow particle-object-specific settings.
            If False, all particle self collisions are disabled, regardless of any other settings.
            Improves performance if self collisions are not needed.
    """

    def __init__(
        self,
        prim_paths_expr: str,
        name: str = "particle_system_view",
        particle_systems_enabled: np.ndarray | torch.Tensor | None = None,
        simulation_owners: Sequence[str] | None = None,
        contact_offsets: np.ndarray | torch.Tensor | None = None,
        rest_offsets: np.ndarray | torch.Tensor | None = None,
        particle_contact_offsets: np.ndarray | torch.Tensor | None = None,
        solid_rest_offsets: np.ndarray | torch.Tensor | None = None,
        fluid_rest_offsets: np.ndarray | torch.Tensor | None = None,
        enable_ccds: np.ndarray | torch.Tensor | None = None,
        solver_position_iteration_counts: np.ndarray | torch.Tensor | None = None,
        max_depenetration_velocities: np.ndarray | torch.Tensor | None = None,
        winds: np.ndarray | torch.Tensor | None = None,
        max_neighborhoods: int | None = None,
        max_velocities: np.ndarray | torch.Tensor | None = None,
        global_self_collisions_enabled: np.ndarray | torch.Tensor | None = None,
    ) -> None:
        self._name = name
        self._physics_view = None
        self._prim_paths = find_matching_prim_paths(prim_paths_expr)
        if len(self._prim_paths) == 0:
            raise Exception(
                f"Prim path expression {prim_paths_expr} is invalid, a prim matching the expression needs to created before wrapping it as view"
            )
        self._count = len(self._prim_paths)
        self._prims = []
        self._regex_prim_paths = prim_paths_expr
        for prim_path in self._prim_paths:
            self._prims.append(get_prim_at_path(prim_path))

        self._applied_particle_materials = [None] * self._count
        self._binding_apis = [None] * self._count

        from isaacsim.core.simulation_manager import SimulationManager

        self._backend = SimulationManager.get_backend()
        self._device = SimulationManager.get_physics_sim_device()
        self._backend_utils = SimulationManager._get_backend_utils()
        if self._device != "cpu":
            carb.log_warn(
                f"ParticleSystem view requested device '{self._device}' but only 'cpu' is currently supported. "
                "Falling back to 'cpu'."
            )
            self._device = "cpu"

        # set properties
        if particle_systems_enabled is not None:
            self.set_particle_systems_enabled(particle_systems_enabled)
        if simulation_owners is not None:
            self.set_simulation_owners(simulation_owners)
        if contact_offsets is not None:
            self.set_contact_offsets(contact_offsets)
        if rest_offsets is not None:
            self.set_rest_offsets(rest_offsets)
        if particle_contact_offsets is not None:
            self.set_particle_contact_offsets(particle_contact_offsets)
        if solid_rest_offsets is not None:
            self.set_solid_rest_offsets(solid_rest_offsets)
        if fluid_rest_offsets is not None:
            self.set_fluid_rest_offsets(fluid_rest_offsets)
        if enable_ccds is not None:
            self.set_enable_ccds(enable_ccds)
        if solver_position_iteration_counts is not None:
            self.set_solver_position_iteration_counts(solver_position_iteration_counts)
        if max_depenetration_velocities is not None:
            self.set_max_depenetration_velocities(max_depenetration_velocities)
        if winds is not None:
            self.set_winds(winds)
        if max_neighborhoods is not None:
            self.set_max_neighborhoods(max_neighborhoods)
        if max_velocities is not None:
            self.set_max_velocities(max_velocities)
        if global_self_collisions_enabled is not None:
            self.set_global_self_collisions_enabled(global_self_collisions_enabled)

        self._invalidate_physics_handle_event = carb.eventdispatcher.get_eventdispatcher().observe_event(
            event_name=omni.timeline.GLOBAL_EVENT_STOP,
            on_event=self._invalidate_physics_handle_callback,
            observer_name="isaacsim.core.prims.ParticleSystem.initialize._invalidate_physics_handle_callback",
        )

    def __del__(self) -> None:
        """Cleans up the particle system view by releasing physics resources."""
        if hasattr(self, "_physics_view"):
            del self._physics_view
        self._invalidate_physics_handle_event = None
        return

    def _apply_material_binding_api(self, index: int) -> None:
        """Applies the UsdShade MaterialBindingAPI to the prim at the specified index.

        Args:
            index: Index of the prim to apply the MaterialBindingAPI to.
        """
        if self._binding_apis[index] is None:
            if self._prims[index].HasAPI(UsdShade.MaterialBindingAPI):
                self._binding_apis[index] = UsdShade.MaterialBindingAPI(self._prims[index])
            else:
                self._binding_apis[index] = UsdShade.MaterialBindingAPI.Apply(self._prims[index])

    """
    Properties.
    """

    @property
    def count(self) -> int:
        """Number of particle systems in the view.

        Returns:
            Number of particle systems for the prims in the view.
        """
        return self._count

    @property
    def name(self) -> str:
        """Name given to the view when instantiating it.

        Returns:
            Name given to the view when instantiating it.
        """
        return self._name

    def is_physics_handle_valid(self) -> bool:
        """Checks whether the physics handle of the view is valid.

        Returns:
            True if the physics handle of the view is valid (i.e physics is initialized for the view). Otherwise False.
        """
        return self._physics_view is not None

    def initialize(self, physics_sim_view: omni.physics.tensors.SimulationView = None) -> None:
        """Create a physics simulation view if not passed and creates a Particle System View.

        Args:
            physics_sim_view: Current physics simulation view.
        """
        if physics_sim_view is None:
            physics_sim_view = omni.physics.tensors.create_simulation_view(self._backend)
            physics_sim_view.set_subspace_roots("/")
        carb.log_info(f"initializing view for {self._name}")
        if not carb.settings.get_settings().get_as_bool("/physics/suppressReadback"):
            carb.log_error(
                "Using particle system view requires the gpu pipeline or (a World initialized with a cuda device)"
            )
        self._physics_sim_view = physics_sim_view
        # Particle system views were removed from the physics tensor API.
        # Keep count from __init__ so USD attribute access (stopped-timeline) still works.
        carb.log_warn("create_particle_system_view is no longer available; using USD attribute access only.")
        self._physics_view = None
        carb.log_info(f"Particle System View Device: {self._device}")
        return

    def _invalidate_physics_handle_callback(self, event: object) -> None:
        """Invalidates the physics handle when timeline stops.

        Args:
            event: The timeline stop event.
        """
        self._physics_view = None
        return

    def is_valid(self, indices: np.ndarray | list | torch.Tensor | None = None) -> bool:
        """Checks whether all prim paths in the view correspond to valid prims in the stage.

        Args:
            indices: Indices to specify which prims to query. Shape (M,).
                Where M <= size of the encapsulated prims in the view.

        Returns:
            True if all prim paths specified in the view correspond to a valid prim in stage. False otherwise.
        """
        indices = self._backend_utils.resolve_indices(indices, self.count, self._device)
        result = True
        for index in indices:
            result = result and is_prim_path_valid(self._prim_paths[index.tolist()])
        return result

    def post_reset(self) -> None:
        """Resets the particles to their initial states."""
        return

    def apply_particle_materials(
        self,
        particle_materials: "ParticleMaterial" | list["ParticleMaterial"],
        indices: np.ndarray | list | torch.Tensor | None = None,
    ) -> None:
        """Used to apply particle material to prims in the view.

        Args:
            particle_materials: Particle materials to be applied to prims in the view.
                Note: if a physics material is not defined, the defaults will be used from PhysX.
                If a list is provided then its size has to be equal the view's size or indices size.
                If one material is provided it will be applied to all prims in the view.
            indices: Indices to specify which prims to manipulate. Shape (M,).
                Where M <= size of the encapsulated prims in the view.

        Raises:
            Exception: length of physics materials != length of prims indexed
        """
        indices = self._backend_utils.resolve_indices(indices, self.count, self._device)
        if isinstance(particle_materials, list):
            if indices.shape[0] != len(particle_materials):
                raise Exception("length of particle materials != length of prims indexed")
        read_idx = 0
        for i in indices:
            self._apply_material_binding_api(i.tolist())
            material = particle_materials[read_idx] if isinstance(particle_materials, list) else particle_materials
            self._binding_apis[i.tolist()].Bind(
                material.material, bindingStrength=UsdShade.Tokens.weakerThanDescendants, materialPurpose="physics"
            )
            self._applied_particle_materials[i.tolist()] = material
            read_idx += 1

    def get_applied_particle_materials(
        self, indices: np.ndarray | list | torch.Tensor | None = None
    ) -> list[ParticleMaterial]:
        """Gets the applied particle material to prims in the view.

        Args:
            indices: indices to specify which prims to query. Shape (M,).
                Where M <= size of the encapsulated prims in the view.

        Returns:
            The current applied particle materials for prims in the view.
        """
        indices = self._backend_utils.resolve_indices(indices, self.count, self._device)
        result = [None] * indices.shape[0]
        write_idx = 0
        for i in indices:
            self._apply_material_binding_api(i.tolist())
            if self._applied_particle_materials[i.tolist()] is not None:
                result[write_idx] = self._applied_particle_materials[i.tolist()]
                write_idx += 1
            else:
                physics_binding = self._binding_apis[i.tolist()].GetDirectBinding(materialPurpose="physics")
                material_path = physics_binding.GetMaterialPath()
                if material_path == "":
                    result[write_idx] = None
                else:
                    from isaacsim.core.api.materials.particle_material import ParticleMaterial

                    self._applied_particle_materials[i.tolist()] = ParticleMaterial(prim_path=material_path)
                    result[write_idx] = self._applied_particle_materials[i.tolist()]
                write_idx += 1
        return result

    """
    Operations - Setters.
    """

    def set_particle_contact_offsets(
        self, values: np.ndarray | torch.Tensor, indices: np.ndarray | list | torch.Tensor | None = None
    ) -> None:
        """Set the contact offset used for interactions between particles.

        Note: Must be larger than solid and fluid rest offsets.

        Args:
            values: The contact offset.
            indices: indices to specify which prims to manipulate. Shape (M,).
                Where M <= size of the encapsulated prims in the view.
        """
        indices = self._backend_utils.resolve_indices(indices, self.count, self._device)
        if not omni.timeline.get_timeline_interface().is_stopped() and self._physics_view is not None:
            new_values = self._backend_utils.clone_tensor(
                self._physics_view.get_particle_contact_offsets(), device=self._device
            )
            new_values[indices] = self._backend_utils.move_data(values, self._device)
            self._physics_view.set_particle_contact_offsets(new_values, indices)
        else:
            idx_count = 0
            for i in indices:
                if "particleContactOffset" not in self._prims[i.tolist()].GetPropertyNames():
                    carb.log_error(
                        f"particleContactOffset property needs to be set for {self.name} before setting its value"
                    )
                    idx_count += 1
                    continue
                self._prims[i.tolist()].GetAttribute("particleContactOffset").Set(values[idx_count].tolist())
                idx_count += 1

    def set_solid_rest_offsets(
        self, values: np.ndarray | torch.Tensor, indices: np.ndarray | list | torch.Tensor | None = None
    ) -> None:
        """Set the rest offset used for solid-solid or solid-fluid particle interactions.

        Note: Must be smaller than particle contact offset.

        Args:
            values: solid rest offset to set particle systems to. shape is (M, ).
            indices: indices to specify which prims to manipulate. Shape (M,).
                Where M <= size of the encapsulated prims in the view.
        """
        indices = self._backend_utils.resolve_indices(indices, self.count, self._device)
        if not omni.timeline.get_timeline_interface().is_stopped() and self._physics_view is not None:
            new_values = self._backend_utils.clone_tensor(
                self._physics_view.get_solid_rest_offsets(), device=self._device
            )
            new_values[indices] = self._backend_utils.move_data(values, self._device)
            self._physics_view.set_solid_rest_offsets(new_values, indices)
        else:
            idx_count = 0
            for i in indices:
                if "solidRestOffset" not in self._prims[i.tolist()].GetPropertyNames():
                    carb.log_error(f"solidRestOffset property needs to be set for {self.name} before setting its value")
                    idx_count += 1
                    continue
                self._prims[i.tolist()].GetAttribute("solidRestOffset").Set(values[idx_count].tolist())
                idx_count += 1

    def set_fluid_rest_offsets(
        self, values: np.ndarray | torch.Tensor, indices: np.ndarray | list | torch.Tensor | None = None
    ) -> None:
        """Set the rest offset used for fluid-fluid particle interactions.

        Note: Must be smaller than particle contact offset.

        Args:
            values: fluid rest offset to set particle systems to. shape is (M, ).
            indices: indices to specify which prims to manipulate. Shape (M,).
                Where M <= size of the encapsulated prims in the view.
        """
        indices = self._backend_utils.resolve_indices(indices, self.count, self._device)
        if not omni.timeline.get_timeline_interface().is_stopped() and self._physics_view is not None:
            new_values = self._backend_utils.clone_tensor(
                self._physics_view.get_fluid_rest_offsets(), device=self._device
            )
            new_values[indices] = self._backend_utils.move_data(values, self._device)
            self._physics_view.set_fluid_rest_offsets(new_values, indices)
        else:
            idx_count = 0
            for i in indices:
                if "fluidRestOffset" not in self._prims[i.tolist()].GetPropertyNames():
                    carb.log_error(f"fluidRestOffset property needs to be set for {self.name} before setting its value")
                    idx_count += 1
                    continue
                self._prims[i.tolist()].GetAttribute("fluidRestOffset").Set(values[idx_count].tolist())
                idx_count += 1

    def set_winds(
        self, values: np.ndarray | torch.Tensor, indices: np.ndarray | list | torch.Tensor | None = None
    ) -> None:
        """Set the winds velocities applied to the current particle system.

        Args:
            values: The wind applied to the current particle system. shape is (M, 3).
            indices: indices to specify which prims to manipulate. Shape (M,).
                Where M <= size of the encapsulated prims in the view.
        """
        indices = self._backend_utils.resolve_indices(indices, self.count, self._device)
        if not omni.timeline.get_timeline_interface().is_stopped() and self._physics_view is not None:
            new_values = self._backend_utils.clone_tensor(self._physics_view.get_wind(), device=self._device)
            new_values[indices] = self._backend_utils.move_data(values, self._device)
            self._physics_view.set_wind(new_values, indices)
        else:
            idx_count = 0
            for i in indices:
                if "wind" not in self._prims[i.tolist()].GetPropertyNames():
                    carb.log_error(f"wind property needs to be set for {self.name} before setting its value")
                    idx_count += 1
                    continue
                self._prims[i.tolist()].GetAttribute("wind").Set(tuple(values[idx_count].tolist()))
                idx_count += 1

    def set_max_velocities(
        self, values: np.ndarray | torch.Tensor, indices: np.ndarray | list | torch.Tensor | None = None
    ) -> None:
        """Set the maximum particle velocity for particle systems.

        Args:
            values: maximum particle velocity tensor to set particle systems to. shape is (M, ).
            indices: indices to specify which prims to manipulate. Shape (M,).
                Where M <= size of the encapsulated prims in the view.
        """
        indices = self._backend_utils.resolve_indices(indices, self.count, self._device)
        idx_count = 0
        for i in indices:
            if "maxVelocity" not in self._prims[i.tolist()].GetPropertyNames():
                carb.log_error(f"maxVelocity property needs to be set for {self.name} before setting its value")
                idx_count += 1
                continue
            self._prims[i.tolist()].GetAttribute("maxVelocity").Set(values[idx_count].tolist())
            idx_count += 1

    def set_max_depenetration_velocities(
        self, values: np.ndarray | torch.Tensor, indices: np.ndarray | list | torch.Tensor | None = None
    ) -> None:
        """Set the maximum velocity permitted to be introduced by the solver to depenetrate intersecting particles for particle systems.

        Args:
            values: maximum particle velocity tensor to set particle systems to. shape is (M, ).
            indices: indices to specify which prims to manipulate. Shape (M,).
                Where M <= size of the encapsulated prims in the view.
        """
        indices = self._backend_utils.resolve_indices(indices, self.count, self._device)
        idx_count = 0
        for i in indices:
            if "maxDepenetrationVelocity" not in self._prims[i.tolist()].GetPropertyNames():
                carb.log_error(
                    f"maxDepenetrationVelocity property needs to be set for {self.name} before setting its value"
                )
                idx_count += 1
                continue
            self._prims[i.tolist()].GetAttribute("maxDepenetrationVelocity").Set(values[idx_count].tolist())
            idx_count += 1

    def set_rest_offsets(
        self, values: np.ndarray | torch.Tensor, indices: np.ndarray | list | torch.Tensor | None = None
    ) -> None:
        """Set the rest offset used for collisions with non-particle objects such as rigid or deformable bodies for particle systems.

        Args:
            values: rest offset tensor to set particle systems to. shape is (M, ).
            indices: indices to specify which prims to manipulate. Shape (M,).
                Where M <= size of the encapsulated prims in the view.
        """
        indices = self._backend_utils.resolve_indices(indices, self.count, self._device)
        idx_count = 0
        for i in indices:
            if "restOffset" not in self._prims[i.tolist()].GetPropertyNames():
                carb.log_error(f"restOffset property needs to be set for {self.name} before setting its value")
                idx_count += 1
                continue
            self._prims[i.tolist()].GetAttribute("restOffset").Set(values[idx_count].tolist())
            idx_count += 1

    def set_contact_offsets(
        self, values: np.ndarray | torch.Tensor, indices: np.ndarray | list | torch.Tensor | None = None
    ) -> None:
        """Set the contact offset used for collisions with non-particle objects such as rigid or deformable bodies for particle systems.

        Args:
            values: contact offset tensor to set particle systems to. shape is (M, ).
            indices: indices to specify which prims to manipulate. Shape (M,).
                Where M <= size of the encapsulated prims in the view.
        """
        indices = self._backend_utils.resolve_indices(indices, self.count, self._device)
        idx_count = 0
        for i in indices:
            if "contactOffset" not in self._prims[i.tolist()].GetPropertyNames():
                carb.log_error(f"contactOffset property needs to be set for {self.name} before setting its value")
                idx_count += 1
                continue
            self._prims[i.tolist()].GetAttribute("contactOffset").Set(values[idx_count].tolist())
            idx_count += 1

    def set_solver_position_iteration_counts(
        self, values: np.ndarray | torch.Tensor, indices: np.ndarray | list | torch.Tensor | None = None
    ) -> None:
        """Set the number of solver iterations for position for particle systems.

        Args:
            values: solver position iteration count tensor to set particle systems to. shape is (M, ).
            indices: indices to specify which prims to manipulate. Shape (M,).
                Where M <= size of the encapsulated prims in the view.
        """
        indices = self._backend_utils.resolve_indices(indices, self.count, self._device)
        idx_count = 0
        for i in indices:
            if "solverPositionIterationCount" not in self._prims[i.tolist()].GetPropertyNames():
                carb.log_error(
                    f"solverPositionIteration property needs to be set for {self.name} before setting its value"
                )
                idx_count += 1
                continue
            self._prims[i.tolist()].GetAttribute("solverPositionIterationCount").Set(values[idx_count].tolist())
            idx_count += 1

    def set_max_neighborhoods(
        self, values: np.ndarray | torch.Tensor, indices: np.ndarray | list | torch.Tensor | None = None
    ) -> None:
        """Set the particle neighborhood size for particle systems.

        Args:
            values: Particle neighborhood size tensor to set particle systems to. shape is (M, ).
            indices: indices to specify which prims
                to manipulate. Shape (M,).
                Where M <= size of the encapsulated prims in the view.
                Defaults to None (i.e: all prims in the view).
        """
        indices = self._backend_utils.resolve_indices(indices, self.count, self._device)
        idx_count = 0
        for i in indices:
            if "maxNeighborhood" not in self._prims[i.tolist()].GetPropertyNames():
                carb.log_error(f"maxNeighborhood property needs to be set for {self.name} before setting its value")
                idx_count += 1
                continue
            self._prims[i.tolist()].GetAttribute("maxNeighborhood").Set(values[idx_count].tolist())
            idx_count += 1

    def set_global_self_collisions_enabled(
        self, values: np.ndarray | torch.Tensor, indices: np.ndarray | list | torch.Tensor | None = None
    ) -> None:
        """Enable self collisions to follow particle-object-specific settings for particle systems.

        Args:
            values: Whether to enable global self collisions tensor to set particle systems to. shape is (M, ).
            indices: indices to specify which prims
                to manipulate. Shape (M,).
                Where M <= size of the encapsulated prims in the view.
                Defaults to None (i.e: all prims in the view).
        """
        indices = self._backend_utils.resolve_indices(indices, self.count, self._device)
        idx_count = 0
        for i in indices:
            if "globalSelfCollisionEnabled" not in self._prims[i.tolist()].GetPropertyNames():
                carb.log_error(
                    f"globalSelfCollisionEnabled property needs to be set for {self.name} before setting its value"
                )
            self._prims[i.tolist()].GetAttribute("globalSelfCollisionEnabled").Set(values[idx_count].tolist())
            idx_count += 1

    def set_enable_ccds(
        self, values: np.ndarray | torch.Tensor, indices: np.ndarray | list | torch.Tensor | None = None
    ) -> None:
        """Enable continuous collision detection for particles for particle systems.

        Args:
            values: Whether to enable continuous collision detection tensor to set particle systems to. shape is (M, ).
            indices: indices to specify which prims
                to manipulate. Shape (M,).
                Where M <= size of the encapsulated prims in the view.
                Defaults to None (i.e: all prims in the view).
        """
        indices = self._backend_utils.resolve_indices(indices, self.count, self._device)
        idx_count = 0
        for i in indices:
            if "enableCCD" not in self._prims[i.tolist()].GetPropertyNames():
                carb.log_error(f"enableCCD property needs to be set for {self.name} before setting its value")
            self._prims[i.tolist()].GetAttribute("enableCCD").Set(values[idx_count].tolist())
            idx_count += 1

    def set_particle_systems_enabled(
        self, values: np.ndarray | torch.Tensor, indices: np.ndarray | list | torch.Tensor | None = None
    ) -> None:
        """Set enabling of the particle systems.

        Args:
            values: Whether to enable particle system tensor to set particle systems to. shape is (M, ).
            indices: indices to specify which prims
                to manipulate. Shape (M,).
                Where M <= size of the encapsulated prims in the view.
                Defaults to None (i.e: all prims in the view).
        """
        indices = self._backend_utils.resolve_indices(indices, self.count, self._device)
        idx_count = 0
        for i in indices:
            if "particleSystemEnabled" not in self._prims[i.tolist()].GetPropertyNames():
                carb.log_error(
                    f"particleSystemEnabled property needs to be set for {self.name} before setting its value"
                )
            self._prims[i.tolist()].GetAttribute("particleSystemEnabled").Set(values[idx_count].tolist())
            idx_count += 1

    def set_simulation_owners(
        self, values: Sequence[str], indices: np.ndarray | list | torch.Tensor | None = None
    ) -> None:
        """Set the PhysicsScene that simulates particle systems.

        Args:
            values: PhysicsScene list to set particle systems to. shape is (M, ).
            indices: indices to specify which prims
                to manipulate. Shape (M,).
                Where M <= size of the encapsulated prims in the view.
                Defaults to None (i.e: all prims in the view).
        """
        indices = self._backend_utils.resolve_indices(indices, self.count, self._device)
        idx_count = 0
        for i in indices:
            if "simulationOwner" not in self._prims[i.tolist()].GetPropertyNames():
                carb.log_error(f"simulationOwner property needs to be set for {self.name} before setting its value")
            self._prims[i.tolist()].GetRelationship("simulationOwner").SetTargets([values[idx_count]])
            idx_count += 1

    """
    Operations - Getters.
    """

    def get_particle_contact_offsets(
        self, indices: np.ndarray | list | torch.Tensor | None = None, clone: bool = True
    ) -> np.ndarray | torch.Tensor:
        """The contact offset used for interactions between particles in the view concatenated.

        Args:
            indices: indices to specify which prims
                to query. Shape (M,).
                Where M <= size of the encapsulated prims in the view.
                Defaults to None (i.e: all prims in the view)
            clone: True to return a clone of the internal buffer. Otherwise False.

        Returns:
            The contact offset used for interactions between particles in the view concatenated. shape is (M, ).
        """
        indices = self._backend_utils.resolve_indices(indices, self.count, self._device)
        if not omni.timeline.get_timeline_interface().is_stopped() and self._physics_view is not None:
            results = self._physics_view.get_particle_contact_offsets()
            if not clone:
                return results[indices]
            else:
                return self._backend_utils.clone_tensor(results, device=self._device)[indices]
        else:
            results = self._backend_utils.create_zeros_tensor([indices.shape[0]], dtype="float32", device=self._device)
            write_idx = 0
            for i in indices:
                results[write_idx] = self._prims[i.tolist()].GetAttribute("particleContactOffset").Get()
                write_idx += 1
            return results

    def get_solid_rest_offsets(
        self, indices: np.ndarray | list | torch.Tensor | None = None, clone: bool = True
    ) -> np.ndarray | torch.Tensor:
        """The rest offset used for solid-solid or solid-fluid particle interactions.

        Args:
            indices: indices to specify which prims
                to query. Shape (M,).
                Where M <= size of the encapsulated prims in the view.
                Defaults to None (i.e: all prims in the view)
            clone: True to return a clone of the internal buffer. Otherwise False.

        Returns:
            The rest offset used for solid-solid or solid-fluid particle interactions. shape is (M, ).
        """
        indices = self._backend_utils.resolve_indices(indices, self.count, self._device)
        if not omni.timeline.get_timeline_interface().is_stopped() and self._physics_view is not None:
            results = self._physics_view.get_solid_rest_offsets()
            if not clone:
                return results[indices]
            else:
                return self._backend_utils.clone_tensor(results, device=self._device)[indices]
        else:
            results = self._backend_utils.create_zeros_tensor([indices.shape[0]], dtype="float32", device=self._device)
            write_idx = 0
            for i in indices:
                results[write_idx] = self._prims[i.tolist()].GetAttribute("solidRestOffset").Get()
                write_idx += 1
            return results

    def get_fluid_rest_offsets(
        self, indices: np.ndarray | list | torch.Tensor | None = None, clone: bool = True
    ) -> np.ndarray | torch.Tensor:
        """The rest offset used for fluid-fluid particle interactions.

        Args:
            indices: indices to specify which prims
                to query. Shape (M,).
                Where M <= size of the encapsulated prims in the view.
                Defaults to None (i.e: all prims in the view)
            clone: True to return a clone of the internal buffer. Otherwise False.

        Returns:
            The rest offset used for fluid-fluid particle interactions. shape is (M, ).
        """
        indices = self._backend_utils.resolve_indices(indices, self.count, self._device)
        if not omni.timeline.get_timeline_interface().is_stopped() and self._physics_view is not None:
            results = self._physics_view.get_fluid_rest_offsets()
            if not clone:
                return results[indices]
            else:
                return self._backend_utils.clone_tensor(results, device=self._device)[indices]
        else:
            results = self._backend_utils.create_zeros_tensor([indices.shape[0]], dtype="float32", device=self._device)
            write_idx = 0
            for i in indices:
                results[write_idx] = self._prims[i.tolist()].GetAttribute("fluidRestOffset").Get()
                write_idx += 1
            return results

    def get_winds(
        self, indices: np.ndarray | list | torch.Tensor | None = None, clone: bool = True
    ) -> np.ndarray | torch.Tensor:
        """The winds applied to the current particle system.

        Args:
            indices: indices to specify which prims
                to query. Shape (M,).
                Where M <= size of the encapsulated prims in the view.
                Defaults to None (i.e: all prims in the view)
            clone: True to return a clone of the internal buffer. Otherwise False.

        Returns:
            The winds applied to the current particle system. shape is (M, 3).
        """
        indices = self._backend_utils.resolve_indices(indices, self.count, self._device)
        if not omni.timeline.get_timeline_interface().is_stopped() and self._physics_view is not None:
            self._physics_sim_view.enable_warnings(False)
            results = self._physics_view.get_wind()
            if not clone:
                return results[indices]
            else:
                return self._backend_utils.clone_tensor(results, device=self._device)[indices]
        else:
            results = self._backend_utils.create_zeros_tensor(
                [indices.shape[0], 3], dtype="float32", device=self._device
            )
            write_idx = 0
            for i in indices:
                results[write_idx] = self._backend_utils.create_tensor_from_list(
                    self._prims[i.tolist()].GetAttribute("wind").Get(), dtype="float32", device=self._device
                )
                write_idx += 1
            return results

    def get_max_velocities(self, indices: np.ndarray | list | torch.Tensor | None = None) -> np.ndarray | torch.Tensor:
        """The maximum particle velocities for each particle system.

        Args:
            indices: indices to specify which prims
                to query. Shape (M,).
                Where M <= size of the encapsulated prims in the view.
                Defaults to None (i.e: all prims in the view)

        Returns:
            The maximum particle velocities for each particle system. shape is (M, ).
        """
        indices = self._backend_utils.resolve_indices(indices, self.count, self._device)
        results = self._backend_utils.create_zeros_tensor([indices.shape[0]], dtype="float32", device=self._device)
        write_idx = 0
        for i in indices:
            results[write_idx] = self._prims[i.tolist()].GetAttribute("maxVelocity").Get()
            write_idx += 1
        return results

    def get_max_depenetration_velocities(
        self, indices: np.ndarray | list | torch.Tensor | None = None
    ) -> np.ndarray | torch.Tensor:
        """The maximum velocity permitted to be introduced by the solver to depenetrate intersecting particles for particle systems for each particle system.

        Args:
            indices: Indices to specify which prims to query. Shape (M,).
                Where M <= size of the encapsulated prims in the view.

        Returns:
            The maximum velocity permitted to be introduced by the solver to
            depenetrate intersecting particles for particle systems for each particle system. shape is (M, ).
        """
        indices = self._backend_utils.resolve_indices(indices, self.count, self._device)
        results = self._backend_utils.create_zeros_tensor([indices.shape[0]], dtype="float32", device=self._device)
        write_idx = 0
        for i in indices:
            results[write_idx] = self._prims[i.tolist()].GetAttribute("maxDepenetrationVelocity").Get()
            write_idx += 1
        return results

    def get_rest_offsets(self, indices: np.ndarray | list | torch.Tensor | None = None) -> np.ndarray | torch.Tensor:
        """The rest offset used for collisions with non-particle objects for each particle system.

        Args:
            indices: Indices to specify which prims to query. Shape (M,).
                Where M <= size of the encapsulated prims in the view.

        Returns:
            The rest offset used for collisions with non-particle objects for each particle system. shape is (M, ).
        """
        indices = self._backend_utils.resolve_indices(indices, self.count, self._device)
        results = self._backend_utils.create_zeros_tensor([indices.shape[0]], dtype="float32", device=self._device)
        write_idx = 0
        for i in indices:
            results[write_idx] = self._prims[i.tolist()].GetAttribute("restOffset").Get()
            write_idx += 1
        return results

    def get_contact_offsets(self, indices: np.ndarray | list | torch.Tensor | None = None) -> np.ndarray | torch.Tensor:
        """The contact offset used for collisions with non-particle objects for each particle system.

        Args:
            indices: Indices to specify which prims to query. Shape (M,).
                Where M <= size of the encapsulated prims in the view.

        Returns:
            The contact offset  used for collisions with non-particle objects for each particle system. shape is (M, ).
        """
        indices = self._backend_utils.resolve_indices(indices, self.count, self._device)
        results = self._backend_utils.create_zeros_tensor([indices.shape[0]], dtype="float32", device=self._device)
        write_idx = 0
        for i in indices:
            results[write_idx] = self._prims[i.tolist()].GetAttribute("contactOffset").Get()
            write_idx += 1
        return results

    def get_solver_position_iteration_counts(
        self, indices: np.ndarray | list | torch.Tensor | None = None
    ) -> np.ndarray | torch.Tensor:
        """The number of solver iterations for positions for each particle system.

        Args:
            indices: Indices to specify which prims to query. Shape (M,).
                Where M <= size of the encapsulated prims in the view.

        Returns:
            The number of solver iterations for positions for each particle system. shape is (M, ).
        """
        indices = self._backend_utils.resolve_indices(indices, self.count, self._device)
        results = self._backend_utils.create_zeros_tensor([indices.shape[0]], dtype="int32", device=self._device)
        write_idx = 0
        for i in indices:
            results[write_idx] = self._prims[i.tolist()].GetAttribute("solverPositionIterationCount").Get()
            write_idx += 1
        return results

    def get_max_neighborhoods(
        self, indices: np.ndarray | list | torch.Tensor | None = None
    ) -> np.ndarray | torch.Tensor:
        """The particle neighborhood size for each particle system.

        Args:
            indices: Indices to specify which prims to query. Shape (M,).
                Where M <= size of the encapsulated prims in the view.

        Returns:
            The particle neighborhood size for each particle system. shape is (M, ).
        """
        indices = self._backend_utils.resolve_indices(indices, self.count, self._device)
        results = self._backend_utils.create_zeros_tensor([indices.shape[0]], dtype="int32", device=self._device)
        write_idx = 0
        for i in indices:
            results[write_idx] = self._prims[i.tolist()].GetAttribute("maxNeighborhood").Get()
            write_idx += 1
        return results

    def get_global_self_collisions_enabled(
        self, indices: np.ndarray | list | torch.Tensor | None = None
    ) -> np.ndarray | torch.Tensor:
        """Whether self collisions to follow particle-object-specific settings is enabled or disabled for each particle system.

        Args:
            indices: Indices to specify which prims to query. Shape (M,).
                Where M <= size of the encapsulated prims in the view.

        Returns:
            Whether self collisions to follow particle-object-specific settings
            is enabled or disabled. for each particle system. shape is (M, ).
        """
        indices = self._backend_utils.resolve_indices(indices, self.count, self._device)
        results = self._backend_utils.create_zeros_tensor([indices.shape[0]], dtype="bool", device=self._device)
        write_idx = 0
        for i in indices:
            results[write_idx] = self._prims[i.tolist()].GetAttribute("globalSelfCollisionEnabled").Get()
            write_idx += 1
        return results

    def get_enable_ccds(self, indices: np.ndarray | list | torch.Tensor | None = None) -> np.ndarray | torch.Tensor:
        """Whether continuous collision detection for particles is enabled or disabled for each particle system.

        Args:
            indices: Indices to specify which prims to query. Shape (M,).
                Where M <= size of the encapsulated prims in the view.

        Returns:
            Whether continuous collision detection for particles is enabled or disabled for each particle system. shape is (M, ).
        """
        indices = self._backend_utils.resolve_indices(indices, self.count, self._device)
        results = self._backend_utils.create_zeros_tensor([indices.shape[0]], dtype="bool", device=self._device)
        write_idx = 0
        for i in indices:
            results[write_idx] = self._prims[i.tolist()].GetAttribute("enableCCD").Get()
            write_idx += 1
        return results

    def get_particle_systems_enabled(
        self, indices: np.ndarray | list | torch.Tensor | None = None
    ) -> np.ndarray | torch.Tensor:
        """Whether particle system is enabled or not for each particle system.

        Args:
            indices: Indices to specify which prims to query. Shape (M,).
                Where M <= size of the encapsulated prims in the view.

        Returns:
            Whether particle system is enabled or not for each particle system. shape is (M, ).
        """
        indices = self._backend_utils.resolve_indices(indices, self.count, self._device)
        results = self._backend_utils.create_zeros_tensor([indices.shape[0]], dtype="bool", device=self._device)
        write_idx = 0
        for i in indices:
            results[write_idx] = self._prims[i.tolist()].GetAttribute("particleSystemEnabled").Get()
            write_idx += 1
        return results

    def get_simulation_owners(self, indices: np.ndarray | list | torch.Tensor | None = None) -> Sequence[str]:
        """The physics scene prim path attached to particle system.

        Args:
            indices: Indices to specify which prims to query. Shape (M,).
                Where M <= size of the encapsulated prims in the view.

        Returns:
            The physics scene prim path attached to particle system. shape is (M, ).
        """
        indices = self._backend_utils.resolve_indices(indices, self.count, self._device)
        results = []
        for i in indices:
            results.append(self._prims[i.tolist()].GetRelationship("simulationOwner").Get())
        return results
