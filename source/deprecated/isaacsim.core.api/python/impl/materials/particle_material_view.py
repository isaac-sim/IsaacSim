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

"""Provides a view class for managing particle material prims and their physics properties in bulk operations."""

from __future__ import annotations

# omniverse
import carb
import numpy as np

# isaac-core
import omni.kit.app
from isaacsim.core.api.simulation_context.simulation_context import SimulationContext
from isaacsim.core.deprecation_manager import import_module
from isaacsim.core.utils.prims import find_matching_prim_paths, get_prim_at_path, is_prim_path_valid
from pxr import PhysxSchema

torch = import_module("torch")


class ParticleMaterialView:
    """The view class to deal with particle material prims.

    Provides high level functions to deal with particle material (1 or more particle materials)
    as well as its attributes/ properties. This object wraps all matching materials found at the regex provided at the prim_paths_expr.
    This object wraps all matching materials Prims found at the regex provided at the prim_paths_expr.

    Args:
        prim_paths_expr: Prim paths regex to encapsulate all prims that match it.
        name: Shortname to be used as a key by Scene class.
        frictions: The friction coefficient tensor, shape is (N, ).
        particle_friction_scales: The coefficient that scales friction for
            solid particle-particle interactions, shape is (N, ).
        dampings: The global velocity damping tensor, shape is (N, ).
        viscosities: The viscosity tensor of fluid particles, shape is (N, ).
        vorticity_confinements: The vorticity confinement tensor for fluid particles, shape is (N, ).
        surface_tensions: The surface tension tensor, shape is (N, ).
        cohesions: The cohesion tensor for interaction between fluid particles, shape is (N, ).
        adhesions: The adhesion tensor for interaction between particles (solid or fluid),
            and rigid or deformable objects, shape is (N, ).
        particle_adhesion_scales: The coefficient tensor that scales adhesion for solid
            particle-particle iterations, shape is (N, ).
        adhesion_offset_scales: The offset scale tensor defines at which adhesion ceases
            to take effect, shape is (N, ).
        gravity_scales: The gravitational acceleration scaling tensor. It can be used
            to approximate lighter-than-air inflatables, shape is (N, ).
        lifts: The lift coefficient tensor for cloth and inflatable particle objects, shape is (N, ).
        drags: The drag coefficient tensor for cloth and inflatable particle objects, shape is (N, ).

    """

    def __init__(
        self,
        prim_paths_expr: str,
        name: str = "particle_material_view",
        frictions: np.ndarray | torch.Tensor | None = None,
        particle_friction_scales: np.ndarray | torch.Tensor | None = None,
        dampings: np.ndarray | torch.Tensor | None = None,
        viscosities: np.ndarray | torch.Tensor | None = None,
        vorticity_confinements: np.ndarray | torch.Tensor | None = None,
        surface_tensions: np.ndarray | torch.Tensor | None = None,
        cohesions: np.ndarray | torch.Tensor | None = None,
        adhesions: np.ndarray | torch.Tensor | None = None,
        particle_adhesion_scales: np.ndarray | torch.Tensor | None = None,
        adhesion_offset_scales: np.ndarray | torch.Tensor | None = None,
        gravity_scales: np.ndarray | torch.Tensor | None = None,
        lifts: np.ndarray | torch.Tensor | None = None,
        drags: np.ndarray | torch.Tensor | None = None,
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

        if SimulationContext.instance() is not None:
            self._backend = SimulationContext.instance().backend
            self._backend_utils = SimulationContext.instance().backend_utils
        else:
            import isaacsim.core.utils.numpy as np_utils

            self._backend = "numpy"
            self._backend_utils = np_utils

        # NOTE: particleSystemView is only supported on the host
        self._device = "cpu"
        self._material_apis = [None] * self._count

        # set properties
        if frictions is not None:
            self.set_frictions(frictions)
        if particle_friction_scales is not None:
            self.set_particle_friction_scales(particle_friction_scales)
        if dampings is not None:
            self.set_dampings(dampings)
        if viscosities is not None:
            self.set_viscosities(viscosities)
        if vorticity_confinements is not None:
            self.set_vorticity_confinements(vorticity_confinements)
        if surface_tensions is not None:
            self.set_surface_tensions(surface_tensions)
        if cohesions is not None:
            self.set_cohesions(cohesions)
        if adhesions is not None:
            self.set_adhesions(adhesions)
        if particle_adhesion_scales is not None:
            self.set_particle_adhesion_scales(particle_adhesion_scales)
        if adhesion_offset_scales is not None:
            self.set_adhesion_offset_scales(adhesion_offset_scales)
        if gravity_scales is not None:
            self.set_gravity_scales(gravity_scales)
        if lifts is not None:
            self.set_lifts(lifts)
        if drags is not None:
            self.set_drags(drags)

    """
    Properties.
    """

    @property
    def count(self) -> int:
        """Number of particle material prims in the view.

        Returns:
            Number of particle material prims in the view.

        """
        return self._count

    @property
    def name(self) -> str:
        """Name given to the view when instantiating it.

        Returns:
            Name given to the view when instantiating it.

        """
        return self._name

    def _apply_material_api(self, index: int) -> None:
        """Applies the PhysX PBD material API to the prim at the specified index.

        Args:
            index: Index of the prim to apply the material API to.

        """
        if self._material_apis[index] is None:
            if self._prims[index].HasAPI(PhysxSchema.PhysxPBDMaterialAPI):
                material_api = PhysxSchema.PhysxPBDMaterialAPI(self._prims[index])
            else:
                material_api = PhysxSchema.PhysxPBDMaterialAPI.Apply(self._prims[index])
            self._material_apis[index] = material_api

    def is_physics_handle_valid(self) -> bool:
        """Checks if the physics handle of the view is valid.

        Returns:
            True if the physics handle of the view is valid (i.e physics is initialized for the view). Otherwise False.

        """
        return self._physics_view is not None

    def is_valid(self, indices: np.ndarray | list | torch.Tensor | None = None) -> bool:
        """Checks if all prim paths specified in the view correspond to valid prims in the stage.

        Args:
            indices: indices to specify which prims to query. Shape (M,).
                Where M <= size of the encapsulated prims in the view.
                Defaults to None (i.e: all prims in the view).

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

    def initialize(self, physics_sim_view: omni.physics.tensors.SimulationView = None) -> None:
        """Create a physics simulation view if not passed and creates a rigid body view in physX.

        Args:
            physics_sim_view: current physics simulation view.

        """
        if physics_sim_view is None:
            physics_sim_view = omni.physics.tensors.create_simulation_view(self._backend)
            physics_sim_view.set_subspace_roots("/")
        carb.log_info(f"initializing view for {self._name}")
        self._physics_sim_view = physics_sim_view
        # Particle material views were removed from the physics tensor API.
        # Keep count from __init__ so USD attribute access (stopped-timeline) still works.
        carb.log_warn("create_particle_material_view is no longer available; using USD attribute access only.")
        self._physics_view = None
        carb.log_info(f"Particle material View Device: {self._device}")
        return

    def set_frictions(
        self,
        values: np.ndarray | torch.Tensor | None,
        indices: np.ndarray | list | torch.Tensor | None = None,
    ) -> None:
        """Sets the friction for the material prims indicated by the indices.

        Args:
            values: material friction tensor with the shape (M, ).
            indices: indices to specify which material prims to manipulate. Shape (M,).
                Where M <= size of the encapsulated prims in the view.
                Defaults to None (i.e: all prims in the view).

        """
        indices = self._backend_utils.resolve_indices(indices, self.count, device=self._device)
        if not omni.timeline.get_timeline_interface().is_stopped() and self._physics_view is not None:
            new_values = self._backend_utils.move_data(values, self._device)
            current_values = self.get_frictions(clone=False)
            current_values[indices] = new_values
            self._physics_view.set_friction(current_values, indices)
            self._physics_sim_view.enable_warnings(True)
        else:
            idx_count = 0
            for i in indices:
                self._apply_material_api(i.tolist())
                if "physxPBDMaterial:friction" not in self._prims[i.tolist()].GetPropertyNames():
                    self._material_apis[i.tolist()].CreateFrictionAttr().Set(values[idx_count].tolist())
                else:
                    self._material_apis[i.tolist()].GetFrictionAttr().Set(values[idx_count].tolist())

                idx_count += 1

    def get_frictions(
        self, indices: np.ndarray | list | torch.Tensor | None = None, clone: bool = True
    ) -> np.ndarray | torch.Tensor:
        """Gets the friction of materials indicated by the indices.

        Args:
            indices: indices to specify which material prims to query. Shape (M,).
                Where M <= size of the encapsulated prims in the view.
                Defaults to None (i.e: all prims in the view).
            clone: True to return a clone of the internal buffer. Otherwise False.

        Returns:
            Friction tensor with shape (M, ).

        """
        indices = self._backend_utils.resolve_indices(indices, self.count, self._device)
        if not omni.timeline.get_timeline_interface().is_stopped() and self._physics_view is not None:
            current_values = self._physics_view.get_friction()
            self._physics_sim_view.enable_warnings(True)
            if not clone:
                return current_values[indices]
            else:
                return self._backend_utils.clone_tensor(current_values[indices], device=self._device)
        else:
            result = self._backend_utils.create_zeros_tensor([indices.shape[0]], dtype="float32", device=self._device)
            write_idx = 0
            for i in indices:
                self._apply_material_api(i.tolist())
                if "physxPBDMaterial:friction" not in self._prims[i.tolist()].GetPropertyNames():
                    result[write_idx] = self._material_apis[i.tolist()].CreateFrictionAttr().Get()
                else:
                    result[write_idx] = self._material_apis[i.tolist()].GetFrictionAttr().Get()
                write_idx += 1
            return result

    def set_dampings(
        self,
        values: np.ndarray | torch.Tensor | None,
        indices: np.ndarray | list | torch.Tensor | None = None,
    ) -> None:
        """Sets the dampings for the material prims indicated by the indices.

        Args:
            values: material damping tensor with the shape (M, ).
            indices: indices to specify which material prims to manipulate. Shape (M,).
                Where M <= size of the encapsulated prims in the view.
                Defaults to None (i.e: all prims in the view).

        """
        indices = self._backend_utils.resolve_indices(indices, self.count, device=self._device)
        if not omni.timeline.get_timeline_interface().is_stopped() and self._physics_view is not None:
            new_values = self._backend_utils.move_data(values, self._device)
            current_values = self.get_dampings(clone=False)
            current_values[indices] = new_values
            self._physics_view.set_damping(current_values, indices)
            self._physics_sim_view.enable_warnings(True)
        else:
            idx_count = 0
            for i in indices:
                self._apply_material_api(i.tolist())
                if "physxPBDMaterial:damping" not in self._prims[i.tolist()].GetPropertyNames():
                    self._material_apis[i.tolist()].CreateDampingAttr().Set(values[idx_count].tolist())
                else:
                    self._material_apis[i.tolist()].GetDampingAttr().Set(values[idx_count].tolist())

                idx_count += 1

    def get_dampings(
        self, indices: np.ndarray | list | torch.Tensor | None = None, clone: bool = True
    ) -> np.ndarray | torch.Tensor:
        """Gets the dampings of materials indicated by the indices.

        Args:
            indices: indices to specify which material prims to query. Shape (M,).
                Where M <= size of the encapsulated prims in the view.
            clone: True to return a clone of the internal buffer. Otherwise False.

        Returns:
            Dampings tensor with shape (M, ).

        """
        indices = self._backend_utils.resolve_indices(indices, self.count, self._device)
        if not omni.timeline.get_timeline_interface().is_stopped() and self._physics_view is not None:
            current_values = self._physics_view.get_damping()
            self._physics_sim_view.enable_warnings(True)
            if not clone:
                return current_values[indices]
            else:
                return self._backend_utils.clone_tensor(current_values[indices], device=self._device)
        else:
            result = self._backend_utils.create_zeros_tensor([indices.shape[0]], dtype="float32", device=self._device)
            write_idx = 0
            for i in indices:
                self._apply_material_api(i.tolist())
                if "physxPBDMaterial:damping" not in self._prims[i.tolist()].GetPropertyNames():
                    result[write_idx] = self._material_apis[i.tolist()].CreateDampingAttr().Get()
                else:
                    result[write_idx] = self._material_apis[i.tolist()].GetDampingAttr().Get()
                write_idx += 1
            return result

    def set_gravity_scales(
        self,
        values: np.ndarray | torch.Tensor | None,
        indices: np.ndarray | list | torch.Tensor | None = None,
    ) -> None:
        """Sets the gravity scale for the material prims indicated by the indices.

        Args:
            values: material gravity scale tensor with the shape (M, ).
            indices: indices to specify which material prims to manipulate. Shape (M,).
                Where M <= size of the encapsulated prims in the view.

        """
        indices = self._backend_utils.resolve_indices(indices, self.count, device=self._device)
        if not omni.timeline.get_timeline_interface().is_stopped() and self._physics_view is not None:
            new_values = self._backend_utils.move_data(values, self._device)
            current_values = self.get_gravity_scales(clone=False)
            current_values[indices] = new_values
            self._physics_view.set_gravity_scale(current_values, indices)
            self._physics_sim_view.enable_warnings(True)
        else:
            idx_count = 0
            for i in indices:
                self._apply_material_api(i.tolist())
                if "physxPBDMaterial:gravityScale" not in self._prims[i.tolist()].GetPropertyNames():
                    self._material_apis[i.tolist()].CreateGravityScaleAttr().Set(values[idx_count].tolist())
                else:
                    self._material_apis[i.tolist()].GetGravityScaleAttr().Set(values[idx_count].tolist())
                idx_count += 1

    def get_gravity_scales(
        self, indices: np.ndarray | list | torch.Tensor | None = None, clone: bool = True
    ) -> np.ndarray | torch.Tensor:
        """Gets the gravity scale of materials indicated by the indices.

        Args:
            indices: indices to specify which material prims to query. Shape (M,).
                Where M <= size of the encapsulated prims in the view.
            clone: True to return a clone of the internal buffer. Otherwise False.

        Returns:
            Gravity scale tensor with shape (M, ).

        """
        indices = self._backend_utils.resolve_indices(indices, self.count, self._device)
        if not omni.timeline.get_timeline_interface().is_stopped() and self._physics_view is not None:
            current_values = self._physics_view.get_gravity_scale()
            self._physics_sim_view.enable_warnings(True)
            if not clone:
                return current_values[indices]
            else:
                return self._backend_utils.clone_tensor(current_values[indices], device=self._device)
        else:
            result = self._backend_utils.create_zeros_tensor([indices.shape[0]], dtype="float32", device=self._device)
            write_idx = 0
            for i in indices:
                self._apply_material_api(i.tolist())
                if "physxPBDMaterial:gravityScale" not in self._prims[i.tolist()].GetPropertyNames():
                    result[write_idx] = self._material_apis[i.tolist()].CreateGravityScaleAttr().Get()
                else:
                    result[write_idx] = self._material_apis[i.tolist()].GetGravityScaleAttr().Get()
                write_idx += 1
            return result

    def set_lifts(
        self,
        values: np.ndarray | torch.Tensor | None,
        indices: np.ndarray | list | torch.Tensor | None = None,
    ) -> None:
        """Sets the lifts for the material prims indicated by the indices.

        Args:
            values: material lift tensor with the shape (M, ).
            indices: indices to specify which material prims to manipulate. Shape (M,).
                Where M <= size of the encapsulated prims in the view.

        """
        # lift attribute was removed from PhysxPBDMaterialAPI — no-op for compatibility
        carb.log_warn("physxPBDMaterial:lift has been removed; set_lifts is a no-op.")

    def get_lifts(
        self, indices: np.ndarray | list | torch.Tensor | None = None, clone: bool = True
    ) -> np.ndarray | torch.Tensor:
        """Gets the lifts of materials indicated by the indices.

        Note: lift attribute was removed from PhysxPBDMaterialAPI. Always returns zeros.

        Args:
            indices: Indices to specify which material prims to query.
                Defaults to None (i.e: all prims in the view).
            clone: True to return a clone of the internal buffer. Otherwise False.

        Returns:
            Tensor of zeros with shape (M, ).

        """
        indices = self._backend_utils.resolve_indices(indices, self.count, self._device)
        return self._backend_utils.create_zeros_tensor([indices.shape[0]], dtype="float32", device=self._device)

    def set_drags(
        self,
        values: np.ndarray | torch.Tensor | None,
        indices: np.ndarray | list | torch.Tensor | None = None,
    ) -> None:
        """Sets the drags for the material prims indicated by the indices.

        Note: drag attribute was removed from PhysxPBDMaterialAPI. This is a no-op.

        Args:
            values: Material drag tensor with the shape (M, ) (ignored).
            indices: Indices to specify which material prims to manipulate.
                Defaults to None (i.e: all prims in the view).

        """
        # drag attribute was removed from PhysxPBDMaterialAPI — no-op for compatibility
        carb.log_warn("physxPBDMaterial:drag has been removed; set_drags is a no-op.")

    def get_drags(
        self, indices: np.ndarray | list | torch.Tensor | None = None, clone: bool = True
    ) -> np.ndarray | torch.Tensor:
        """Gets the drags of materials indicated by the indices.

        Note: drag attribute was removed from PhysxPBDMaterialAPI. Always returns zeros.

        Args:
            indices: Indices to specify which material prims to query.
                Defaults to None (i.e: all prims in the view).
            clone: True to return a clone of the internal buffer. Otherwise False.

        Returns:
            Tensor of zeros with shape (M, ).

        """
        indices = self._backend_utils.resolve_indices(indices, self.count, self._device)
        return self._backend_utils.create_zeros_tensor([indices.shape[0]], dtype="float32", device=self._device)

    def set_viscosities(
        self,
        values: np.ndarray | torch.Tensor | None,
        indices: np.ndarray | list | torch.Tensor | None = None,
    ) -> None:
        """Sets the particle viscosity for the material prims indicated by the indices.

        Args:
            values: material particle viscosity scale tensor with the shape (M, ).
            indices: indices to specify which material prims to manipulate. Shape (M,).
                Where M <= size of the encapsulated prims in the view.

        """
        indices = self._backend_utils.resolve_indices(indices, self.count, device=self._device)
        idx_count = 0
        for i in indices:
            self._apply_material_api(i.tolist())
            if "physxPBDMaterial:viscosity" not in self._prims[i.tolist()].GetPropertyNames():
                self._material_apis[i.tolist()].CreateViscosityAttr().Set(values[idx_count].tolist())
            else:
                self._material_apis[i.tolist()].GetViscosityAttr().Set(values[idx_count].tolist())
            idx_count += 1

    def get_viscosities(
        self, indices: np.ndarray | list | torch.Tensor | None = None, clone: bool = True
    ) -> np.ndarray | torch.Tensor:
        """Gets the viscosity of materials indicated by the indices.

        Args:
            indices: indices to specify which material prims to query. Shape (M,).
                Where M <= size of the encapsulated prims in the view.
            clone: True to return a clone of the internal buffer. Otherwise False.

        Returns:
            Viscosity tensor with shape (M, ).

        """
        indices = self._backend_utils.resolve_indices(indices, self.count, self._device)
        result = self._backend_utils.create_zeros_tensor([indices.shape[0]], dtype="float32", device=self._device)
        write_idx = 0
        for i in indices:
            self._apply_material_api(i.tolist())
            if "physxPBDMaterial:viscosity" not in self._prims[i.tolist()].GetPropertyNames():
                result[write_idx] = self._material_apis[i.tolist()].CreateViscosityAttr().Get()
            else:
                result[write_idx] = self._material_apis[i.tolist()].GetViscosityAttr().Get()
            write_idx += 1
        return result

    def set_cohesions(
        self,
        values: np.ndarray | torch.Tensor | None,
        indices: np.ndarray | list | torch.Tensor | None = None,
    ) -> None:
        """Sets the particle cohesion for the material prims indicated by the indices.

        Args:
            values: material particle cohesion scale tensor with the shape (M, ).
            indices: indices to specify which material prims to manipulate. Shape (M,).
                Where M <= size of the encapsulated prims in the view.

        """
        indices = self._backend_utils.resolve_indices(indices, self.count, device=self._device)
        idx_count = 0
        for i in indices:
            self._apply_material_api(i.tolist())
            if "physxPBDMaterial:cohesion" not in self._prims[i.tolist()].GetPropertyNames():
                self._material_apis[i.tolist()].CreateCohesionAttr().Set(values[idx_count].tolist())
            else:
                self._material_apis[i.tolist()].GetCohesionAttr().Set(values[idx_count].tolist())
            idx_count += 1

    def get_cohesions(
        self, indices: np.ndarray | list | torch.Tensor | None = None, clone: bool = True
    ) -> np.ndarray | torch.Tensor:
        """Gets the cohesion of materials indicated by the indices.

        Args:
            indices: indices to specify which material prims to query. Shape (M,).
                Where M <= size of the encapsulated prims in the view.
                Defaults to None (i.e: all prims in the view).
            clone: True to return a clone of the internal buffer. Otherwise False.

        Returns:
            cohesion tensor with shape (M, )

        """
        indices = self._backend_utils.resolve_indices(indices, self.count, self._device)
        result = self._backend_utils.create_zeros_tensor([indices.shape[0]], dtype="float32", device=self._device)
        write_idx = 0
        for i in indices:
            self._apply_material_api(i.tolist())
            if "physxPBDMaterial:cohesion" not in self._prims[i.tolist()].GetPropertyNames():
                result[write_idx] = self._material_apis[i.tolist()].CreateCohesionAttr().Get()
            else:
                result[write_idx] = self._material_apis[i.tolist()].GetCohesionAttr().Get()
            write_idx += 1
        return result

    def set_adhesions(
        self,
        values: np.ndarray | torch.Tensor | None,
        indices: np.ndarray | list | torch.Tensor | None = None,
    ) -> None:
        """Sets the particle adhesion for the material prims indicated by the indices.

        Args:
            values: material particle adhesion scale tensor with the shape (M, ).
            indices: indices to specify which material prims to manipulate. Shape (M,).
                Where M <= size of the encapsulated prims in the view.
                Defaults to None (i.e: all prims in the view).

        """
        indices = self._backend_utils.resolve_indices(indices, self.count, device=self._device)
        idx_count = 0
        for i in indices:
            self._apply_material_api(i.tolist())
            if "physxPBDMaterial:adhesion" not in self._prims[i.tolist()].GetPropertyNames():
                self._material_apis[i.tolist()].CreateAdhesionAttr().Set(values[idx_count].tolist())
            else:
                self._material_apis[i.tolist()].GetAdhesionAttr().Set(values[idx_count].tolist())
            idx_count += 1

    def get_adhesions(
        self, indices: np.ndarray | list | torch.Tensor | None = None, clone: bool = True
    ) -> np.ndarray | torch.Tensor:
        """Gets the adhesion of materials indicated by the indices.

        Args:
            indices: indices to specify which material prims to query. Shape (M,).
                Where M <= size of the encapsulated prims in the view.
                Defaults to None (i.e: all prims in the view).
            clone: True to return a clone of the internal buffer. Otherwise False.

        Returns:
            adhesion tensor with shape (M, )

        """
        indices = self._backend_utils.resolve_indices(indices, self.count, self._device)
        result = self._backend_utils.create_zeros_tensor([indices.shape[0]], dtype="float32", device=self._device)
        write_idx = 0
        for i in indices:
            self._apply_material_api(i.tolist())
            if "physxPBDMaterial:adhesion" not in self._prims[i.tolist()].GetPropertyNames():
                result[write_idx] = self._material_apis[i.tolist()].CreateAdhesionAttr().Get()
            else:
                result[write_idx] = self._material_apis[i.tolist()].GetAdhesionAttr().Get()
            write_idx += 1
        return result

    def set_particle_adhesion_scales(
        self,
        values: np.ndarray | torch.Tensor | None,
        indices: np.ndarray | list | torch.Tensor | None = None,
    ) -> None:
        """Sets the particle adhesion for the material prims indicated by the indices.

        Args:
            values: material particle adhesion scale tensor with the shape (M, ).
            indices: indices to specify which material prims to manipulate. Shape (M,).
                Where M <= size of the encapsulated prims in the view.
                Defaults to None (i.e: all prims in the view).

        """
        indices = self._backend_utils.resolve_indices(indices, self.count, device=self._device)
        idx_count = 0
        for i in indices:
            self._apply_material_api(i.tolist())
            if "physxPBDMaterial:particleAdhesionScale" not in self._prims[i.tolist()].GetPropertyNames():
                self._material_apis[i.tolist()].CreateParticleAdhesionScaleAttr().Set(values[idx_count].tolist())
            else:
                self._material_apis[i.tolist()].GetParticleAdhesionScaleAttr().Set(values[idx_count].tolist())
            idx_count += 1

    def get_particle_adhesion_scales(
        self, indices: np.ndarray | list | torch.Tensor | None = None, clone: bool = True
    ) -> np.ndarray | torch.Tensor:
        """Gets the adhesion scale of materials indicated by the indices.

        Args:
            indices: indices to specify which material prims to query. Shape (M,).
                Where M <= size of the encapsulated prims in the view.
                Defaults to None (i.e: all prims in the view).
            clone: True to return a clone of the internal buffer. Otherwise False.

        Returns:
            adhesion scale tensor with shape (M, )

        """
        indices = self._backend_utils.resolve_indices(indices, self.count, self._device)
        result = self._backend_utils.create_zeros_tensor([indices.shape[0]], dtype="float32", device=self._device)
        write_idx = 0
        for i in indices:
            self._apply_material_api(i.tolist())
            if "physxPBDMaterial:particleAdhesionScale" not in self._prims[i.tolist()].GetPropertyNames():
                result[write_idx] = self._material_apis[i.tolist()].CreateParticleAdhesionScaleAttr().Get()
            else:
                result[write_idx] = self._material_apis[i.tolist()].GetParticleAdhesionScaleAttr().Get()
            write_idx += 1
        return result

    def set_adhesion_offset_scales(
        self,
        values: np.ndarray | torch.Tensor | None,
        indices: np.ndarray | list | torch.Tensor | None = None,
    ) -> None:
        """Sets the adhesion offset scale for the material prims indicated by the indices.

        Args:
            values: material adhesion offset scale tensor with the shape (M, ).
            indices: indices to specify which material prims to manipulate. Shape (M,).
                Where M <= size of the encapsulated prims in the view.
                Defaults to None (i.e: all prims in the view).

        """
        indices = self._backend_utils.resolve_indices(indices, self.count, device=self._device)
        idx_count = 0
        for i in indices:
            self._apply_material_api(i.tolist())
            if "physxPBDMaterial:adhesionOffsetScale" not in self._prims[i.tolist()].GetPropertyNames():
                self._material_apis[i.tolist()].CreateAdhesionOffsetScaleAttr().Set(values[idx_count].tolist())
            else:
                self._material_apis[i.tolist()].GetAdhesionOffsetScaleAttr().Set(values[idx_count].tolist())
            idx_count += 1

    def get_adhesion_offset_scales(
        self, indices: np.ndarray | list | torch.Tensor | None = None, clone: bool = True
    ) -> np.ndarray | torch.Tensor:
        """Gets the adhesion offset scale of materials indicated by the indices.

        Args:
            indices: indices to specify which material prims to query. Shape (M,).
                Where M <= size of the encapsulated prims in the view.
                Defaults to None (i.e: all prims in the view).
            clone: True to return a clone of the internal buffer. Otherwise False.

        Returns:
            adhesion offset scale tensor with shape (M, )

        """
        indices = self._backend_utils.resolve_indices(indices, self.count, self._device)
        result = self._backend_utils.create_zeros_tensor([indices.shape[0]], dtype="float32", device=self._device)
        write_idx = 0
        for i in indices:
            self._apply_material_api(i.tolist())
            if "physxPBDMaterial:adhesionOffsetScale" not in self._prims[i.tolist()].GetPropertyNames():
                result[write_idx] = self._material_apis[i.tolist()].CreateAdhesionOffsetScaleAttr().Get()
            else:
                result[write_idx] = self._material_apis[i.tolist()].GetAdhesionOffsetScaleAttr().Get()
            write_idx += 1
        return result

    def set_surface_tensions(
        self,
        values: np.ndarray | torch.Tensor | None,
        indices: np.ndarray | list | torch.Tensor | None = None,
    ) -> None:
        """Sets the particle surface tension for the material prims indicated by the indices.

        Args:
            values: material particle surface tension scale tensor with the shape (M, ).
            indices: indices to specify which material prims to manipulate. Shape (M,).
                Where M <= size of the encapsulated prims in the view.
                Defaults to None (i.e: all prims in the view).

        """
        indices = self._backend_utils.resolve_indices(indices, self.count, device=self._device)
        idx_count = 0
        for i in indices:
            self._apply_material_api(i.tolist())
            if "physxPBDMaterial:surfaceTension" not in self._prims[i.tolist()].GetPropertyNames():
                self._material_apis[i.tolist()].CreateSurfaceTensionAttr().Set(values[idx_count].tolist())
            else:
                self._material_apis[i.tolist()].GetSurfaceTensionAttr().Set(values[idx_count].tolist())
            idx_count += 1

    def get_surface_tensions(
        self, indices: np.ndarray | list | torch.Tensor | None = None, clone: bool = True
    ) -> np.ndarray | torch.Tensor:
        """Gets the surface tension of materials indicated by the indices.

        Args:
            indices: indices to specify which material prims to query. Shape (M,).
                Where M <= size of the encapsulated prims in the view.
                Defaults to None (i.e: all prims in the view).
            clone: True to return a clone of the internal buffer. Otherwise False.

        Returns:
            surface tension tensor with shape (M, )

        """
        indices = self._backend_utils.resolve_indices(indices, self.count, self._device)
        result = self._backend_utils.create_zeros_tensor([indices.shape[0]], dtype="float32", device=self._device)
        write_idx = 0
        for i in indices:
            self._apply_material_api(i.tolist())
            if "physxPBDMaterial:surfaceTension" not in self._prims[i.tolist()].GetPropertyNames():
                result[write_idx] = self._material_apis[i.tolist()].CreateSurfaceTensionAttr().Get()
            else:
                result[write_idx] = self._material_apis[i.tolist()].GetSurfaceTensionAttr().Get()
            write_idx += 1
        return result

    def set_vorticity_confinements(
        self,
        values: np.ndarray | torch.Tensor | None,
        indices: np.ndarray | list | torch.Tensor | None = None,
    ) -> None:
        """Sets the vorticity confinement for the material prims indicated by the indices.

        Args:
            values: material particle vorticity confinement scale tensor with the shape (M, ).
            indices: indices to specify which material prims to manipulate. Shape (M,).
                Where M <= size of the encapsulated prims in the view.
                Defaults to None (i.e: all prims in the view).

        """
        indices = self._backend_utils.resolve_indices(indices, self.count, device=self._device)
        idx_count = 0
        for i in indices:
            self._apply_material_api(i.tolist())
            if "physxPBDMaterial:vorticityConfinement" not in self._prims[i.tolist()].GetPropertyNames():
                self._material_apis[i.tolist()].CreatetVorticityConfinementAttr().Set(values[idx_count].tolist())
            else:
                self._material_apis[i.tolist()].GetVorticityConfinementAttr().Set(values[idx_count].tolist())
            idx_count += 1

    def get_vorticity_confinements(
        self, indices: np.ndarray | list | torch.Tensor | None = None, clone: bool = True
    ) -> np.ndarray | torch.Tensor:
        """Gets the vorticity confinement of materials indicated by the indices.

        Args:
            indices: indices to specify which material prims to query. Shape (M,).
                Where M <= size of the encapsulated prims in the view.
            clone: True to return a clone of the internal buffer. Otherwise False.

        Returns:
            vorticity confinement tensor with shape (M, )

        """
        indices = self._backend_utils.resolve_indices(indices, self.count, self._device)
        result = self._backend_utils.create_zeros_tensor([indices.shape[0]], dtype="float32", device=self._device)
        write_idx = 0
        for i in indices:
            self._apply_material_api(i.tolist())
            if "physxPBDMaterial:vorticityConfinement" not in self._prims[i.tolist()].GetPropertyNames():
                result[write_idx] = self._material_apis[i.tolist()].CreateVorticityConfinementAttr().Get()
            else:
                result[write_idx] = self._material_apis[i.tolist()].GetVorticityConfinementAttr().Get()
            write_idx += 1
        return result

    def set_particle_friction_scales(
        self,
        values: np.ndarray | torch.Tensor | None,
        indices: np.ndarray | list | torch.Tensor | None = None,
    ) -> None:
        """Sets the particle friction scale for the material prims indicated by the indices.

        Args:
            values: material particle friction scale tensor with the shape (M, ).
            indices: indices to specify which material prims to manipulate. Shape (M,).
                Where M <= size of the encapsulated prims in the view.

        """
        indices = self._backend_utils.resolve_indices(indices, self.count, device=self._device)
        idx_count = 0
        for i in indices:
            self._apply_material_api(i.tolist())
            if "physxPBDMaterial:particleFrictionScale" not in self._prims[i.tolist()].GetPropertyNames():
                self._material_apis[i.tolist()].CreateParticleFrictionScaleAttr().Set(values[idx_count].tolist())
            else:
                self._material_apis[i.tolist()].GetParticleFrictionScaleAttr().Set(values[idx_count].tolist())
            idx_count += 1

    def get_particle_friction_scales(
        self, indices: np.ndarray | list | torch.Tensor | None = None, clone: bool = True
    ) -> np.ndarray | torch.Tensor:
        """Gets the particle friction scale of materials indicated by the indices.

        Args:
            indices: indices to specify which material prims to query. Shape (M,).
                Where M <= size of the encapsulated prims in the view.
            clone: True to return a clone of the internal buffer. Otherwise False.

        Returns:
            particle friction scale tensor with shape (M, )

        """
        indices = self._backend_utils.resolve_indices(indices, self.count, self._device)
        result = self._backend_utils.create_zeros_tensor([indices.shape[0]], dtype="float32", device=self._device)
        write_idx = 0
        for i in indices:
            self._apply_material_api(i.tolist())
            if "physxPBDMaterial:particleFrictionScale" not in self._prims[i.tolist()].GetPropertyNames():
                result[write_idx] = self._material_apis[i.tolist()].CreateParticleFrictionScaleAttr().Get()
            else:
                result[write_idx] = self._material_apis[i.tolist()].GetParticleFrictionScaleAttr().Get()
            write_idx += 1
        return result
