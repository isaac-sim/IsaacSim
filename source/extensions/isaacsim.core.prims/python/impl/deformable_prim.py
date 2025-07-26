# SPDX-FileCopyrightText: Copyright (c) 2021-2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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

from typing import List, Optional, Union

import carb
import numpy as np
import omni.kit.app
import torch
from pxr import PhysxSchema, UsdShade

from .xform_prim import XFormPrim


class DeformablePrim(XFormPrim):
    """The view class for deformable prims."""

    def __init__(
        self,
        prim_paths_expr: str,
        deformable_materials: Optional[Union[np.ndarray, torch.Tensor]] = None,
        name: str = "deformable_prim_view",
        reset_xform_properties: bool = True,
        positions: Optional[Union[np.ndarray, torch.Tensor]] = None,
        translations: Optional[Union[np.ndarray, torch.Tensor]] = None,
        orientations: Optional[Union[np.ndarray, torch.Tensor]] = None,
        scales: Optional[Union[np.ndarray, torch.Tensor]] = None,
        visibilities: Optional[Union[np.ndarray, torch.Tensor]] = None,
        vertex_velocity_dampings: Optional[Union[np.ndarray, torch.Tensor]] = None,
        sleep_dampings: Optional[Union[np.ndarray, torch.Tensor]] = None,
        sleep_thresholds: Optional[Union[np.ndarray, torch.Tensor]] = None,
        settling_thresholds: Optional[Union[np.ndarray, torch.Tensor]] = None,
        self_collisions: Optional[Union[np.ndarray, torch.Tensor]] = None,
        self_collision_filter_distances: Optional[Union[np.ndarray, torch.Tensor]] = None,
        solver_position_iteration_counts: Optional[Union[np.ndarray, torch.Tensor]] = None,
    ):
        """
        Provides high level functions to deal with deformable bodies (1 or more deformable bodies)
        as well as its attributes/ properties. This object wraps all matching deformable bodies found at the regex provided at the prim_paths_expr.

        Note: - if the underlying UsdGeom.Mesh.Get does not already have appropriate USD deformable body apis applied to it before init, this class will apply it.
        Args:
            prim_paths_expr (str): Prim paths regex to encapsulate all prims that match it.
            name (str): Shortname to be used as a key by Scene class.
            positions (Union[np.ndarray, torch.Tensor], optional): Default positions in the world frame of the prim. shape is (N, 3).
            translations (Union[np.ndarray, torch.Tensor], optional): Default translations in the local frame of the
                                                                        prims (with respect to its parent prims). shape is (N, 3).
            orientations (Union[np.ndarray, torch.Tensor], optional): Default quaternion orientations in the world/
                                                                        local frame of the prim (depends if translation or position is specified).
                                                                        quaternion is scalar-first (w, x, y, z). shape is (N, 4).
            scales (Union[np.ndarray, torch.Tensor], optional): Local scales to be applied to the prim's dimensions. shape is (N, 3).
            visibilities (Union[np.ndarray, torch.Tensor], optional): Set to false for an invisible prim in the stage while rendering. shape is (N,).
            vertex_velocity_dampings (Union[np.ndarray, torch.Tensor], optional): Velocity damping parameter controlling how much after every time step the nodal velocity is reduced
            sleep_dampings (Union[np.ndarray, torch.Tensor], optional): Damping value that damps the motion of bodies that move slow enough to be candidates for sleeping (see sleep_threshold)
            sleep_thresholds (Union[np.ndarray, torch.Tensor], optional): Threshold that defines the maximal magnitude of the linear motion a soft body can move in one second such that it can go to sleep in the next frame
            settling_thresholds (Union[np.ndarray, torch.Tensor], optional): Threshold that defines the maximal magnitude of the linear motion a fem body can move in one second before it becomes a candidate for sleeping
            self_collisions (Union[np.ndarray, torch.Tensor], optional): Enables the self collision for the deformable body based on the rest position distances.
            self_collision_filter_distances (Union[np.ndarray, torch.Tensor], optional): Penetration value that needs to get exceeded before contacts for self collision are generated. Will only have an effect if self collisions are enabled based on the rest position distances.
            solver_position_iteration_counts (Union[np.ndarray, torch.Tensor], optional): Number of the solver's positional iteration counts
        """

        self._physics_view = None
        self._device = None
        self._name = name
        XFormPrim.__init__(
            self,
            prim_paths_expr=prim_paths_expr,
            name=name,
            positions=positions,
            translations=translations,
            orientations=orientations,
            scales=scales,
            visibilities=visibilities,
            reset_xform_properties=reset_xform_properties,
        )
        self._deformable_body_apis = [None] * self._count
        self._deformable_apis = [None] * self._count
        self._mass_apis = [None] * self._count
        self._applied_deformable_materials = [None] * self._count
        self._binding_apis = [None] * self._count

        if vertex_velocity_dampings is not None:
            self.set_vertex_velocity_dampings(vertex_velocity_dampings)
        if sleep_dampings is not None:
            self.set_sleep_dampings(sleep_dampings)
        if sleep_thresholds is not None:
            self.set_sleep_thresholds(sleep_thresholds)
        if settling_thresholds is not None:
            self.set_settling_thresholds(settling_thresholds)
        if self_collisions is not None:
            self.set_self_collisions(self_collisions)
        if self_collision_filter_distances is not None:
            self.set_self_collision_filter_distances(self_collision_filter_distances)
        if solver_position_iteration_counts is not None:
            self.set_solver_position_iteration_counts(solver_position_iteration_counts)

        timeline = omni.timeline.get_timeline_interface()
        self._invalidate_physics_handle_event = timeline.get_timeline_event_stream().create_subscription_to_pop(
            self._invalidate_physics_handle_callback
        )
        carb.log_warn(
            "Please note that support for deformable prims in the current form is now deprecated. Some features including stress/strain APIs may be removed in the future."
        )

    """
    Properties.
    """

    @property
    def count(self) -> int:
        """
        Returns:
            int: deformable counts.
        """
        return self._count

    @property
    def max_simulation_mesh_elements_per_body(self) -> int:
        """
        Returns:
            int: maximum number of simulation mesh elements per deformable body.
        """
        return self._max_simulation_mesh_elements_per_body

    @property
    def max_simulation_mesh_vertices_per_body(self) -> int:
        """
        Returns:
            int: maximum number of simulation mesh vertices per deformable body.
        """
        return self._max_simulation_mesh_vertices_per_body

    @property
    def max_collision_mesh_elements_per_body(self) -> int:
        """
        Returns:
            int: maximum number of collision mesh elements per deformable body.
        """
        return self._max_collision_mesh_elements_per_body

    @property
    def max_collision_mesh_vertices_per_body(self) -> int:
        """
        Returns:
            int: maximum number of collision mesh vertices per deformable body.
        """
        return self._max_collision_mesh_vertices_per_body

    def is_physics_handle_valid(self) -> bool:
        """
        Returns:
            bool: True if the physics handle of the view is valid (i.e physics is initialized for the view). Otherwise False.
        """
        return self._physics_view is not None

    def initialize(self, physics_sim_view: omni.physics.tensors.SimulationView = None) -> None:
        """Create a physics simulation view if not passed and creates a deformable body view in physX.

        Args:
            physics_sim_view (omni.physics.tensors.SimulationView, optional): current physics simulation view. Defaults to None.
        """

        if physics_sim_view is None:
            physics_sim_view = omni.physics.tensors.create_simulation_view(self._backend)
            physics_sim_view.set_subspace_roots("/")
        carb.log_info("initializing view for {}".format(self._name))
        if not carb.settings.get_settings().get_as_bool("/physics/suppressReadback"):
            carb.log_error(
                "Using Deformable body requires the gpu pipeline or (a World initialized with a cuda device)"
            )
        self._physics_sim_view = physics_sim_view
        self._physics_view = self._physics_sim_view.create_soft_body_view(self._regex_prim_paths[0].replace(".*", "*"))
        self._count = self._physics_view.count
        self._max_collision_mesh_elements_per_body = self._physics_view.max_elements_per_body
        self._max_collision_mesh_vertices_per_body = self._physics_view.max_vertices_per_body
        self._max_simulation_mesh_elements_per_body = self._physics_view.max_sim_elements_per_body
        self._max_simulation_mesh_vertices_per_body = self._physics_view.max_sim_vertices_per_body
        carb.log_info("Deformable Prim View Device: {}".format(self._device))
        return

    def _invalidate_physics_handle_callback(self, event):
        if event.type == int(omni.timeline.TimelineEventType.STOP):
            self._physics_view = None
        return

    def _apply_deformable_body_api(self, index):
        if self._deformable_body_apis[index] is None:
            if self._prims[index].HasAPI(PhysxSchema.PhysxDeformableBodyAPI):
                api = PhysxSchema.PhysxDeformableBodyAPI(self._prims[index])
            else:
                api = PhysxSchema.PhysxDeformableBodyAPI.Apply(self._prims[index])
            self._deformable_body_apis[index] = api

    def _apply_deformable_api(self, index):
        self._apply_deformable_body_api(index)
        if self._deformable_apis[index] is None:
            if self._prims[index].HasAPI(PhysxSchema.PhysxDeformableAPI):
                api = PhysxSchema.PhysxDeformableAPI(self._deformable_body_apis[index])
            else:
                api = PhysxSchema.PhysxDeformableAPI.Apply(self._deformable_body_apis[index])
            self._deformable_apis[index] = api

    def _apply_material_binding_api(self, index):
        if self._binding_apis[index] is None:
            if self._prims[index].HasAPI(UsdShade.MaterialBindingAPI):
                self._binding_apis[index] = UsdShade.MaterialBindingAPI(self._prims[index])
            else:
                self._binding_apis[index] = UsdShade.MaterialBindingAPI.Apply(self._prims[index])

    def apply_deformable_materials(
        self,
        deformable_materials: Union["DeformableMaterial", List["DeformableMaterial"]],
        indices: Optional[Union[np.ndarray, list, torch.Tensor]] = None,
    ) -> None:
        """Used to apply deformable material to prims in the view.

        Args:
            deformable_materials (Union[DeformableMaterial, List[DeformableMaterial]]): deformable materials to be applied to prims in the view.
                                                                                Note: if a physics material is not defined,
                                                                                the defaults will be used from PhysX.
                                                                                If a list is provided then its size has to be equal
                                                                                the view's size or indices size.
                                                                                If one material is provided it will be applied to all prims in the view.
            indices (Optional[Union[np.ndarray, list, torch.Tensor]], optional): indices to specify which prims
                                                                                to manipulate. Shape (M,).
                                                                                Where M <= size of the encapsulated prims in the view.
                                                                                Defaults to None (i.e: all prims in the view).
        Raises:
            Exception: length of physics materials != length of prims indexed
        """
        indices = self._backend_utils.resolve_indices(indices, self.count, self._device)
        if isinstance(deformable_materials, list):
            if indices.shape[0] != len(deformable_materials):
                raise Exception("length of deformable materials != length of prims indexed")
        read_idx = 0
        for i in indices:
            self._apply_material_binding_api(i.tolist())
            material = (
                deformable_materials[read_idx] if isinstance(deformable_materials, list) else deformable_materials
            )
            self._binding_apis[i.tolist()].Bind(
                material.material, bindingStrength=UsdShade.Tokens.weakerThanDescendants, materialPurpose="physics"
            )
            self._applied_deformable_materials[i.tolist()] = material
            read_idx += 1

    def get_applied_deformable_materials(
        self, indices: Optional[Union[np.ndarray, list, torch.Tensor]] = None
    ) -> List["DeformableMaterial"]:
        """Gets the applied deformable material to prims in the view.

        Args:
            indices (Optional[Union[np.ndarray, list, torch.Tensor]], optional): indices to specify which prims
                                                                                to query. Shape (M,).
                                                                                Where M <= size of the encapsulated prims in the view.
                                                                                Defaults to None (i.e: all prims in the view).

        Returns:
            List[DeformableMaterial]: the current applied deformable materials for prims in the view.
        """
        indices = self._backend_utils.resolve_indices(indices, self.count, self._device)
        result = [None] * indices.shape[0]
        write_idx = 0
        for i in indices:
            self._apply_material_binding_api(i.tolist())
            if self._applied_deformable_materials[i.tolist()] is not None:
                result[write_idx] = self._applied_deformable_materials[i.tolist()]
                write_idx += 1
            else:
                physics_binding = self._binding_apis[i.tolist()].GetDirectBinding(materialPurpose="physics")
                material_path = physics_binding.GetMaterialPath()
                if material_path == "":
                    result[write_idx] = None
                else:
                    from isaacsim.core.api.materials.deformable_material import DeformableMaterial

                    self._applied_deformable_materials[i.tolist()] = DeformableMaterial(prim_path=material_path)
                    result[write_idx] = self._applied_deformable_materials[i.tolist()]
                write_idx += 1
        return result

    def set_simulation_mesh_nodal_positions(
        self,
        positions: Optional[Union[np.ndarray, torch.Tensor]],
        indices: Optional[Union[np.ndarray, list, torch.Tensor]] = None,
    ) -> None:
        """Sets the nodal positions of the simulation mesh for the deformable bodies indicated by the indices.

        Args:
            positions (Union[np.ndarray, torch.Tensor]): nodal positions with the shape (M, max_simulation_mesh_vertices_per_body, 3).
            indices (Optional[Union[np.ndarray, list, torch.Tensor]], optional): indices to specify which deformable prims to manipulate. Shape (M,).
                                                                                 Where M <= size of the encapsulated prims in the view.
                                                                                 Defaults to None (i.e: all prims in the view).
        """
        indices = self._backend_utils.resolve_indices(indices, self.count, device=self._device)
        if not omni.timeline.get_timeline_interface().is_stopped() and self._physics_view is not None:
            new_positions = self._backend_utils.move_data(positions, self._device)
            current_positions = self.get_simulation_mesh_nodal_positions(clone=False)
            current_positions[indices] = new_positions
            self._physics_view.set_sim_nodal_positions(current_positions, indices)
        else:
            idx_count = 0
            for i in indices:
                self._apply_deformable_body_api(i.tolist())
                if "physxDeformable:simulationPoints" not in self._prims[i.tolist()].GetPropertyNames():
                    carb.log_warn(
                        f"physxDeformable:simulationPoints is not defined on the deformable prim {self.name}."
                    )
                    self._deformable_body_apis[i.tolist()].CreateSimulationPointsAttr().Set(
                        positions[idx_count].tolist()
                    )
                else:
                    self._deformable_body_apis[i.tolist()].GetSimulationPointsAttr().Set(positions[idx_count].tolist())
                idx_count += 1
        return

    def set_simulation_mesh_nodal_velocities(
        self,
        velocities: Optional[Union[np.ndarray, torch.Tensor]],
        indices: Optional[Union[np.ndarray, list, torch.Tensor]] = None,
    ) -> None:
        """Sets the vertex velocities for the deformable bodies indicated by the indices.

        Args:
            velocities (Union[np.ndarray, torch.Tensor]): vertex velocities with the shape
                                                                                (M, max_simulation_mesh_vertices_per_body, 3).
            indices (Optional[Union[np.ndarray, list, torch.Tensor]], optional): indices to specify which deformable prims to manipulate. Shape (M,).
                                                                                 Where M <= size of the encapsulated prims in the view.
                                                                                 Defaults to None (i.e: all prims in the view).
        """
        indices = self._backend_utils.resolve_indices(indices, self.count, device=self._device)
        if not omni.timeline.get_timeline_interface().is_stopped() and self._physics_view is not None:
            new_velocities = self._backend_utils.move_data(velocities, self._device)
            current_velocities = self.get_simulation_mesh_nodal_velocities(clone=False)
            current_velocities[indices] = new_velocities
            self._physics_view.set_sim_nodal_velocities(current_velocities, indices)
        else:
            idx_count = 0
            for i in indices:
                self._apply_deformable_api(i.tolist())
                if "physxDeformable:simulationVelocities" not in self._prims[i.tolist()].GetPropertyNames():
                    carb.log_warn(
                        f"physxDeformable:simulationVelocities is not defined on the deformable prim {self.name}."
                    )
                    self._deformable_apis[i.tolist()].CreateSimulationVelocitiesAttr().Set(
                        velocities[idx_count].tolist()
                    )
                else:
                    self._deformable_apis[i.tolist()].GetSimulationVelocitiesAttr().Set(velocities[idx_count].tolist())
                idx_count += 1

    def set_simulation_mesh_kinematic_targets(
        self,
        positions: Optional[Union[np.ndarray, torch.Tensor]],
        indices: Optional[Union[np.ndarray, list, torch.Tensor]] = None,
    ) -> None:
        """Sets the kinematic targets of the simulation mesh for the deformable bodies indicated by the indices.

        Args:
            positions (Union[np.ndarray, torch.Tensor]): kinematic targets with the shape (M, max_simulation_mesh_vertices_per_body, 4).
            indices (Optional[Union[np.ndarray, list, torch.Tensor]], optional): indices to specify which deformable prims to manipulate. Shape (M,).
                                                                                 Where M <= size of the encapsulated prims in the view.
                                                                                 Defaults to None (i.e: all prims in the view).
        """
        indices = self._backend_utils.resolve_indices(indices, self.count, device=self._device)
        if not omni.timeline.get_timeline_interface().is_stopped() and self._physics_view is not None:
            new_positions = self._backend_utils.move_data(positions, self._device)
            current_positions = self.get_simulation_mesh_kinematic_targets(clone=False)
            current_positions[indices] = new_positions
            self._physics_view.set_sim_kinematic_targets(current_positions, indices)
        else:
            carb.log_warn(
                "Physics Simulation View is not created yet in order to use set_simulation_mesh_kinematic_targets"
            )
            return

    def set_simulation_mesh_indices(
        self,
        values: Optional[Union[np.ndarray, torch.Tensor]],
        indices: Optional[Union[np.ndarray, list, torch.Tensor]] = None,
    ) -> None:
        """Sets the simulation mesh element indices of the deformable bodies indicated by the indices.

        Args:
            values (Union[np.ndarray, torch.Tensor]): element indices with the shape  (M, ).
            indices (Optional[Union[np.ndarray, list, torch.Tensor]], optional): indices to specify which deformable prims to manipulate. Shape (M,).
                                                                                 Where M <= size of the encapsulated prims in the view.
                                                                                 Defaults to None (i.e: all prims in the view).
        """
        indices = self._backend_utils.resolve_indices(indices, self.count, device=self._device)
        idx_count = 0
        for i in indices:
            self._apply_deformable_api(i.tolist())
            if "physxDeformable:simulationIndices" not in self._prims[i.tolist()].GetPropertyNames():
                carb.log_warn(f"physxDeformable:simulationIndices is not defined on the deformable prim {self.name}.")
                self._deformable_apis[i.tolist()].CreateSimulationIndicesAttr().Set(values[idx_count].tolist())
            else:
                self._deformable_apis[i.tolist()].GetSimulationIndicesAttr().Set(values[idx_count].tolist())

            idx_count += 1

    def set_collision_mesh_indices(
        self,
        values: Optional[Union[np.ndarray, torch.Tensor]],
        indices: Optional[Union[np.ndarray, list, torch.Tensor]] = None,
    ) -> None:
        """Sets the collision mesh element indices of the deformable bodies indicated by the indices.

        Args:
            values (Union[np.ndarray, torch.Tensor]): element indices with the shape  (M, ).
            indices (Optional[Union[np.ndarray, list, torch.Tensor]], optional): indices to specify which deformable prims to manipulate. Shape (M,).
                                                                                 Where M <= size of the encapsulated prims in the view.
                                                                                 Defaults to None (i.e: all prims in the view).
        """
        indices = self._backend_utils.resolve_indices(indices, self.count, device=self._device)
        idx_count = 0
        for i in indices:
            self._apply_deformable_body_api(i.tolist())
            if "physxDeformable:collisionIndices" not in self._prims[i.tolist()].GetPropertyNames():
                carb.log_warn(f"physxDeformable:collisionIndices is not defined on the deformable prim {self.name}.")
                self._deformable_body_apis[i.tolist()].CreateCollisionIndicesAttr().Set(values[idx_count].tolist())
            else:
                self._deformable_body_apis[i.tolist()].GetCollisionIndicesAttr().Set(values[idx_count].tolist())

            idx_count += 1

    def set_collision_mesh_rest_points(
        self,
        values: Optional[Union[np.ndarray, torch.Tensor]],
        indices: Optional[Union[np.ndarray, list, torch.Tensor]] = None,
    ) -> None:
        """Sets the collision mesh vertices rest positions of the deformable bodies indicated by the indices.

        Args:
            values (Union[np.ndarray, torch.Tensor]): vertices rest positions values with the shape  (M, ).
            indices (Optional[Union[np.ndarray, list, torch.Tensor]], optional): indices to specify which deformable prims to manipulate. Shape (M,).
                                                                                 Where M <= size of the encapsulated prims in the view.
                                                                                 Defaults to None (i.e: all prims in the view).
        """
        indices = self._backend_utils.resolve_indices(indices, self.count, device=self._device)
        idx_count = 0
        for i in indices:
            self._apply_deformable_body_api(i.tolist())
            if "physxDeformable:collisionRestPoints" not in self._prims[i.tolist()].GetPropertyNames():
                carb.log_warn(f"physxDeformable:collisionRestPoints is not defined on the deformable prim {self.name}.")
                self._deformable_body_apis[i.tolist()].CreateCollisionRestPointsAttr().Set(values[idx_count].tolist())
            else:
                self._deformable_body_apis[i.tolist()].GetCollisionRestPointsAttr().Set(values[idx_count].tolist())

            idx_count += 1

    def set_simulation_mesh_rest_points(
        self,
        values: Optional[Union[np.ndarray, torch.Tensor]],
        indices: Optional[Union[np.ndarray, list, torch.Tensor]] = None,
    ) -> None:
        """Sets the simulation mesh vertices rest positions of the deformable bodies indicated by the indices.

        Args:
            values (Union[np.ndarray, torch.Tensor]): vertices rest positions values with the shape  (M, ).
            indices (Optional[Union[np.ndarray, list, torch.Tensor]], optional): indices to specify which deformable prims to manipulate. Shape (M,).
                                                                                 Where M <= size of the encapsulated prims in the view.
                                                                                 Defaults to None (i.e: all prims in the view).
        """
        indices = self._backend_utils.resolve_indices(indices, self.count, device=self._device)
        idx_count = 0
        for i in indices:
            self._apply_deformable_body_api(i.tolist())
            if "physxDeformable:simulationRestPoints" not in self._prims[i.tolist()].GetPropertyNames():
                carb.log_warn(
                    f"physxDeformable:simulationRestPoints is not defined on the deformable prim {self.name}."
                )
                self._deformable_body_apis[i.tolist()].CreateSimulationRestPointsAttr().Set(values[idx_count].tolist())
            else:
                self._deformable_body_apis[i.tolist()].GetSimulationRestPointsAttr().Set(values[idx_count].tolist())

            idx_count += 1

    def set_sleep_dampings(
        self,
        values: Optional[Union[np.ndarray, torch.Tensor]],
        indices: Optional[Union[np.ndarray, list, torch.Tensor]] = None,
    ) -> None:
        """Sets the sleep dampings values for deformable bodies indicated by the indices.

        Args:
            values (Union[np.ndarray, torch.Tensor]): solver position iteration counts values with the shape  (M, ).
            indices (Optional[Union[np.ndarray, list, torch.Tensor]], optional): indices to specify which deformable prims to manipulate. Shape (M,).
                                                                                 Where M <= size of the encapsulated prims in the view.
                                                                                 Defaults to None (i.e: all prims in the view).
        """
        indices = self._backend_utils.resolve_indices(indices, self.count, device=self._device)
        idx_count = 0
        for i in indices:
            # if values[idx_count] < 0:
            #     carb.log_error("The range of solver position iteration counts is [0. inf). Incorrect value for index ", idx_count)
            self._apply_deformable_api(i.tolist())
            if "physxDeformable:sleepDampings" not in self._prims[i.tolist()].GetPropertyNames():
                carb.log_warn(f"physxDeformable:sleepDampings is not defined on the deformable prim {self.name}.")
                self._deformable_apis[i.tolist()].CreateSleepDampingAttr().Set(values[idx_count].tolist())
            else:
                self._deformable_apis[i.tolist()].GetSleepDampingAttr().Set(values[idx_count].tolist())

            idx_count += 1

    def set_sleep_thresholds(
        self,
        values: Optional[Union[np.ndarray, torch.Tensor]],
        indices: Optional[Union[np.ndarray, list, torch.Tensor]] = None,
    ) -> None:
        """Sets the sleep threshold values for deformable bodies indicated by the indices.

        Args:
            values (Union[np.ndarray, torch.Tensor]): solver position iteration counts values with the shape  (M, ).
            indices (Optional[Union[np.ndarray, list, torch.Tensor]], optional): indices to specify which deformable prims to manipulate. Shape (M,).
                                                                                 Where M <= size of the encapsulated prims in the view.
                                                                                 Defaults to None (i.e: all prims in the view).
        """
        indices = self._backend_utils.resolve_indices(indices, self.count, device=self._device)
        idx_count = 0
        for i in indices:
            # if values[idx_count] < 0:
            #     carb.log_error("The range of solver position iteration counts is [0. inf). Incorrect value for index ", idx_count)
            self._apply_deformable_api(i.tolist())
            if "physxDeformable:sleepThreshold" not in self._prims[i.tolist()].GetPropertyNames():
                carb.log_warn(f"physxDeformable:sleepThreshold is not defined on the deformable prim {self.name}.")
                self._deformable_apis[i.tolist()].CreateSleepThresholdAttr().Set(values[idx_count].tolist())
            else:
                self._deformable_apis[i.tolist()].GetSleepThresholdAttr().Set(values[idx_count].tolist())

            idx_count += 1

    def set_settling_thresholds(
        self,
        values: Optional[Union[np.ndarray, torch.Tensor]],
        indices: Optional[Union[np.ndarray, list, torch.Tensor]] = None,
    ) -> None:
        """Sets the settling threshold values for deformable bodies indicated by the indices.

        Args:
            values (Union[np.ndarray, torch.Tensor]): solver position iteration counts values with the shape  (M, ).
            indices (Optional[Union[np.ndarray, list, torch.Tensor]], optional): indices to specify which deformable prims to manipulate. Shape (M,).
                                                                                 Where M <= size of the encapsulated prims in the view.
                                                                                 Defaults to None (i.e: all prims in the view).
        """
        indices = self._backend_utils.resolve_indices(indices, self.count, device=self._device)
        idx_count = 0
        for i in indices:
            # if values[idx_count] < 0:
            #     carb.log_error("The range of solver position iteration counts is [0. inf). Incorrect value for index ", idx_count)
            self._apply_deformable_api(i.tolist())
            if "physxDeformable:settlingThreshold" not in self._prims[i.tolist()].GetPropertyNames():
                carb.log_warn(f"physxDeformable:settlingThreshold is not defined on the deformable prim {self.name}.")
                self._deformable_apis[i.tolist()].CreateSettlingThresholdAttr().Set(values[idx_count].tolist())
            else:
                self._deformable_apis[i.tolist()].GetSettlingThresholdAttr().Set(values[idx_count].tolist())

            idx_count += 1

    def set_self_collision_filter_distances(
        self,
        values: Optional[Union[np.ndarray, torch.Tensor]],
        indices: Optional[Union[np.ndarray, list, torch.Tensor]] = None,
    ) -> None:
        """Sets the self collisions filter distance values for deformable bodies indicated by the indices.

        Args:
            values (Union[np.ndarray, torch.Tensor]): solver position iteration counts values with the shape  (M, ).
            indices (Optional[Union[np.ndarray, list, torch.Tensor]], optional): indices to specify which deformable prims to manipulate. Shape (M,).
                                                                                 Where M <= size of the encapsulated prims in the view.
                                                                                 Defaults to None (i.e: all prims in the view).
        """
        indices = self._backend_utils.resolve_indices(indices, self.count, device=self._device)
        idx_count = 0
        for i in indices:
            if values[idx_count] < 0:
                carb.log_error(
                    "The range of self collision filter distances is [0. inf). Incorrect value for index ", idx_count
                )
            self._apply_deformable_api(i.tolist())
            if "physxDeformable:selfCollisionFilterDistance" not in self._prims[i.tolist()].GetPropertyNames():
                carb.log_warn(
                    f"physxDeformable:selfCollisionFilterDistance is not defined on the deformable prim {self.name}."
                )
                self._deformable_apis[i.tolist()].CreateSelfCollisionFilterDistanceAttr().Set(
                    values[idx_count].tolist()
                )
            else:
                self._deformable_apis[i.tolist()].GetSelfCollisionFilterDistanceAttr().Set(values[idx_count].tolist())

            idx_count += 1

    def set_self_collisions(
        self,
        values: Optional[Union[np.ndarray, torch.Tensor]],
        indices: Optional[Union[np.ndarray, list, torch.Tensor]] = None,
    ) -> None:
        """Sets the self collisions values for deformable bodies indicated by the indices.

        Args:
            values (Union[np.ndarray, torch.Tensor]): solver position iteration counts values with the shape  (M, ).
            indices (Optional[Union[np.ndarray, list, torch.Tensor]], optional): indices to specify which deformable prims to manipulate. Shape (M,).
                                                                                 Where M <= size of the encapsulated prims in the view.
                                                                                 Defaults to None (i.e: all prims in the view).
        """
        indices = self._backend_utils.resolve_indices(indices, self.count, device=self._device)
        idx_count = 0
        for i in indices:
            self._apply_deformable_api(i.tolist())
            if "physxDeformable:selfCollision" not in self._prims[i.tolist()].GetPropertyNames():
                carb.log_warn(f"physxDeformable:selfCollision is not defined on the deformable prim {self.name}.")
                self._deformable_apis[i.tolist()].CreateSelfCollisionAttr().Set(values[idx_count].tolist())
            else:
                self._deformable_apis[i.tolist()].GetSelfCollisionAttr().Set(values[idx_count].tolist())

            idx_count += 1

    def set_vertex_velocity_dampings(
        self,
        values: Optional[Union[np.ndarray, torch.Tensor]],
        indices: Optional[Union[np.ndarray, list, torch.Tensor]] = None,
    ) -> None:
        """Sets values of the vertex velocity damping values to deformable bodies indicated by the indices.

        Args:
            values (Union[np.ndarray, torch.Tensor]): solver position iteration counts values with the shape  (M, ).
            indices (Optional[Union[np.ndarray, list, torch.Tensor]], optional): indices to specify which deformable prims to manipulate. Shape (M,).
                                                                                 Where M <= size of the encapsulated prims in the view.
                                                                                 Defaults to None (i.e: all prims in the view).
        """
        indices = self._backend_utils.resolve_indices(indices, self.count, device=self._device)
        idx_count = 0
        for i in indices:
            if values[idx_count] < 0:
                carb.log_error(
                    "The range of vertex velocity damping is [0. inf). Incorrect value for index ", idx_count
                )
            self._apply_deformable_api(i.tolist())
            if "physxDeformable:vertexVelocityDamping" not in self._prims[i.tolist()].GetPropertyNames():
                carb.log_warn(
                    f"physxDeformable:vertexVelocityDamping is not defined on the deformable prim {self.name}."
                )
                self._deformable_apis[i.tolist()].CreateVertexVelocityDampingAttr().Set(values[idx_count].tolist())
            else:
                self._deformable_apis[i.tolist()].GetVertexVelocityDampingAttr().Set(values[idx_count].tolist())

            idx_count += 1

    def set_solver_position_iteration_counts(
        self,
        values: Optional[Union[np.ndarray, torch.Tensor]],
        indices: Optional[Union[np.ndarray, list, torch.Tensor]] = None,
    ) -> None:
        """Sets values of the solver position iteration counts to deformable bodies indicated by the indices.

        Args:
            values (Union[np.ndarray, torch.Tensor]): solver position iteration counts values with the shape  (M, ).
            indices (Optional[Union[np.ndarray, list, torch.Tensor]], optional): indices to specify which deformable prims to manipulate. Shape (M,).
                                                                                 Where M <= size of the encapsulated prims in the view.
                                                                                 Defaults to None (i.e: all prims in the view).
        """
        indices = self._backend_utils.resolve_indices(indices, self.count, device=self._device)
        idx_count = 0
        for i in indices:
            if values[idx_count] < 0:
                carb.log_error(
                    "The range of solver position iteration counts is [0. inf). Incorrect value for index ", idx_count
                )
            self._apply_deformable_api(i.tolist())
            if "physxDeformable:solverPositionIterationCount" not in self._prims[i.tolist()].GetPropertyNames():
                carb.log_warn(
                    f"physxDeformable:solverPositionIterationCount is not defined on the deformable prim {self.name}."
                )
                self._deformable_apis[i.tolist()].CreateSolverPositionIterationCountAttr().Set(
                    values[idx_count].tolist()
                )
            else:
                self._deformable_apis[i.tolist()].GetSolverPositionIterationCountAttr().Set(values[idx_count].tolist())

            idx_count += 1

    def get_sleep_dampings(
        self, indices: Optional[Union[np.ndarray, list, torch.Tensor]] = None, clone: bool = True
    ) -> Union[np.ndarray, torch.Tensor]:
        """Gets the sleep damping for the deformable bodies indicated by the indices.

        Args:
            indices (Optional[Union[np.ndarray, list, torch.Tensor]], optional): indices to specify which deformable prims to query. Shape (M,).
                                                                                 Where M <= size of the encapsulated prims in the view.
                                                                                 Defaults to None (i.e: all prims in the view).
            clone (float, optional): True to return a clone of the internal buffer. Otherwise False. Defaults to True.

        Returns:
            Union[Tuple[np.ndarray, np.ndarray], Tuple[torch.Tensor, torch.Tensor]]: the sleep damping tensor with shape (M, )
        """
        indices = self._backend_utils.resolve_indices(indices, self.count, self._device)
        results = self._backend_utils.create_zeros_tensor([indices.shape[0]], dtype="float32", device=self._device)
        write_idx = 0
        for i in indices:
            self._apply_deformable_api(i.tolist())
            if "physxDeformable:sleepDamping" not in self._prims[i.tolist()].GetPropertyNames():
                carb.log_warn(f"physxDeformable:sleepDamping is not defined on the deformable prim {self.name}.")
                results[write_idx] = self._deformable_apis[i.tolist()].CreateSleepDampingAttr().Get()
            else:
                results[write_idx] = self._deformable_apis[i.tolist()].GetSleepDampingAttr().Get()
            write_idx += 1
        return results

    def get_sleep_thresholds(
        self, indices: Optional[Union[np.ndarray, list, torch.Tensor]] = None, clone: bool = True
    ) -> Union[np.ndarray, torch.Tensor]:
        """Gets the sleep threshold for the deformable bodies indicated by the indices.

        Args:
            indices (Optional[Union[np.ndarray, list, torch.Tensor]], optional): indices to specify which deformable prims to query. Shape (M,).
                                                                                 Where M <= size of the encapsulated prims in the view.
                                                                                 Defaults to None (i.e: all prims in the view).
            clone (float, optional): True to return a clone of the internal buffer. Otherwise False. Defaults to True.

        Returns:
            Union[Tuple[np.ndarray, np.ndarray], Tuple[torch.Tensor, torch.Tensor]]: the sleep threshold tensor with shape (M, )
        """
        indices = self._backend_utils.resolve_indices(indices, self.count, self._device)
        results = self._backend_utils.create_zeros_tensor([indices.shape[0]], dtype="float32", device=self._device)
        write_idx = 0
        for i in indices:
            self._apply_deformable_api(i.tolist())
            if "physxDeformable:sleepThreshold" not in self._prims[i.tolist()].GetPropertyNames():
                carb.log_warn(f"physxDeformable:sleepThreshold is not defined on the deformable prim {self.name}.")
                results[write_idx] = self._deformable_apis[i.tolist()].CreateSleepThresholdAttr().Get()
            else:
                results[write_idx] = self._deformable_apis[i.tolist()].GetSleepThresholdAttr().Get()
            write_idx += 1
        return results

    def get_settling_thresholds(
        self, indices: Optional[Union[np.ndarray, list, torch.Tensor]] = None, clone: bool = True
    ) -> Union[np.ndarray, torch.Tensor]:
        """Gets the settling threshold for the deformable bodies indicated by the indices.

        Args:
            indices (Optional[Union[np.ndarray, list, torch.Tensor]], optional): indices to specify which deformable prims to query. Shape (M,).
                                                                                 Where M <= size of the encapsulated prims in the view.
                                                                                 Defaults to None (i.e: all prims in the view).
            clone (float, optional): True to return a clone of the internal buffer. Otherwise False. Defaults to True.

        Returns:
            Union[Tuple[np.ndarray, np.ndarray], Tuple[torch.Tensor, torch.Tensor]]: the settling threshold tensor with shape (M, )
        """
        indices = self._backend_utils.resolve_indices(indices, self.count, self._device)
        results = self._backend_utils.create_zeros_tensor([indices.shape[0]], dtype="float32", device=self._device)
        write_idx = 0
        for i in indices:
            self._apply_deformable_api(i.tolist())
            if "physxDeformable:settlingThreshold" not in self._prims[i.tolist()].GetPropertyNames():
                carb.log_warn(f"physxDeformable:settlingThreshold is not defined on the deformable prim {self.name}.")
                results[write_idx] = self._deformable_apis[i.tolist()].CreateSettlingThresholdAttr().Get()
            else:
                results[write_idx] = self._deformable_apis[i.tolist()].GetSettlingThresholdAttr().Get()
            write_idx += 1
        return results

    def get_self_collision_filter_distances(
        self, indices: Optional[Union[np.ndarray, list, torch.Tensor]] = None, clone: bool = True
    ) -> Union[np.ndarray, torch.Tensor]:
        """Gets the self collision filter distance for the deformable bodies indicated by the indices.

        Args:
            indices (Optional[Union[np.ndarray, list, torch.Tensor]], optional): indices to specify which deformable prims to query. Shape (M,).
                                                                                 Where M <= size of the encapsulated prims in the view.
                                                                                 Defaults to None (i.e: all prims in the view).
            clone (float, optional): True to return a clone of the internal buffer. Otherwise False. Defaults to True.

        Returns:
            Union[Tuple[np.ndarray, np.ndarray], Tuple[torch.Tensor, torch.Tensor]]: the self collision filter distance tensor with shape (M, )
        """
        indices = self._backend_utils.resolve_indices(indices, self.count, self._device)
        results = self._backend_utils.create_zeros_tensor([indices.shape[0]], dtype="float32", device=self._device)
        write_idx = 0
        for i in indices:
            self._apply_deformable_api(i.tolist())
            if "physxDeformable:selfCollisionFilterDistance" not in self._prims[i.tolist()].GetPropertyNames():
                carb.log_warn(
                    f"physxDeformable:selfCollisionFilterDistance is not defined on the deformable prim {self.name}."
                )
                results[write_idx] = self._deformable_apis[i.tolist()].CreateSelfCollisionFilterDistanceAttr().Get()
            else:
                results[write_idx] = self._deformable_apis[i.tolist()].GetSelfCollisionFilterDistanceAttr().Get()
            write_idx += 1
        return results

    def get_self_collisions(
        self, indices: Optional[Union[np.ndarray, list, torch.Tensor]] = None, clone: bool = True
    ) -> Union[np.ndarray, torch.Tensor]:
        """Gets the self collision parameters for the deformable bodies indicated by the indices.

        Args:
            indices (Optional[Union[np.ndarray, list, torch.Tensor]], optional): indices to specify which deformable prims to query. Shape (M,).
                                                                                 Where M <= size of the encapsulated prims in the view.
                                                                                 Defaults to None (i.e: all prims in the view).
            clone (bool, optional): True to return a clone of the internal buffer. Otherwise False. Defaults to True.

        Returns:
            Union[Tuple[np.ndarray, np.ndarray], Tuple[torch.Tensor, torch.Tensor]]: the self collision tensor with shape (M, )
        """
        indices = self._backend_utils.resolve_indices(indices, self.count, self._device)
        self_collisions = self._backend_utils.create_zeros_tensor(
            [indices.shape[0]], dtype="float32", device=self._device
        )
        write_idx = 0
        for i in indices:
            self._apply_deformable_api(i.tolist())
            if "physxDeformable:selfCollision" not in self._prims[i.tolist()].GetPropertyNames():
                carb.log_warn(f"physxDeformable:selfCollision is not defined on the deformable prim {self.name}.")
                self_collisions[write_idx] = self._deformable_apis[i.tolist()].CreateSelfCollisionAttr().Get()
            else:
                self_collisions[write_idx] = self._deformable_apis[i.tolist()].GetSelfCollisionAttr().Get()
            write_idx += 1
        return self_collisions

    def get_vertex_velocity_dampings(
        self, indices: Optional[Union[np.ndarray, list, torch.Tensor]] = None, clone: bool = True
    ) -> Union[np.ndarray, torch.Tensor]:
        """Gets the vertex velocity dampings of the deformable bodies indicated by the indices.

        Args:
            indices (Optional[Union[np.ndarray, list, torch.Tensor]], optional): indices to specify which deformable prims to query. Shape (M,).
                                                                                 Where M <= size of the encapsulated prims in the view.
                                                                                 Defaults to None (i.e: all prims in the view).
            clone (float, optional): True to return a clone of the internal buffer. Otherwise False. Defaults to True.

        Returns:
            Union[Tuple[np.ndarray, np.ndarray], Tuple[torch.Tensor, torch.Tensor]]: deformable bodies vertex velocity dampings with shape (M, ).
        """
        indices = self._backend_utils.resolve_indices(indices, self.count, self._device)
        results = self._backend_utils.create_zeros_tensor([indices.shape[0]], dtype="float32", device=self._device)
        write_idx = 0
        for i in indices:
            self._apply_deformable_api(i.tolist())
            if "physxDeformable:vertexVelocityDamping" not in self._prims[i.tolist()].GetPropertyNames():
                carb.log_warn(
                    f"physxDeformable:vertexVelocityDamping is not defined on the deformable prim {self.name}."
                )
                results[write_idx] = self._deformable_apis[i.tolist()].CreateVertexVelocityDampingAttr().Get()
            else:
                results[write_idx] = self._deformable_apis[i.tolist()].GetVertexVelocityDampingAttr().Get()
            write_idx += 1
        return results

    def get_solver_position_iteration_counts(
        self, indices: Optional[Union[np.ndarray, list, torch.Tensor]] = None, clone: bool = True
    ) -> Union[np.ndarray, torch.Tensor]:
        """Gets the solver's positional iteration counts of the deformable bodies indicated by the indices.

        Args:
            indices (Optional[Union[np.ndarray, list, torch.Tensor]], optional): indices to specify which deformable prims to query. Shape (M,).
                                                                                 Where M <= size of the encapsulated prims in the view.
                                                                                 Defaults to None (i.e: all prims in the view).
            clone (int, optional): True to return a clone of the internal buffer. Otherwise False. Defaults to True.

        Returns:
            Union[Tuple[np.ndarray, np.ndarray], Tuple[torch.Tensor, torch.Tensor]]: solver's positional iteration counts with shape (M, ).
        """
        indices = self._backend_utils.resolve_indices(indices, self.count, self._device)
        result = self._backend_utils.create_zeros_tensor([indices.shape[0]], dtype="int32", device=self._device)
        write_idx = 0
        for i in indices:
            self._apply_deformable_body_api(i.tolist())
            if "physxDeformable:solverPositionIterationCount" not in self._prims[i.tolist()].GetPropertyNames():
                carb.log_warn(
                    f"physxDeformable:solverPositionIterationCount is not defined on the deformable prim {self.name}."
                )
                result[write_idx] = self._deformable_apis[i.tolist()].CreateSolverPositionIterationCountAttr().Get()
            else:
                result[write_idx] = self._deformable_apis[i.tolist()].GetSolverPositionIterationCountAttr().Get()
            write_idx += 1
        return result

    def get_simulation_mesh_nodal_positions(
        self, indices: Optional[Union[np.ndarray, list, torch.Tensor]] = None, clone: bool = True
    ) -> Union[np.ndarray, torch.Tensor]:
        """Gets the nodal positions of the simulation mesh for the deformable bodies indicated by the indices.

        Args:
            indices (Optional[Union[np.ndarray, list, torch.Tensor]], optional): indices to specify which deformable prims to query. Shape (M,).
                                                                                 Where M <= size of the encapsulated prims in the view.
                                                                                 Defaults to None (i.e: all prims in the view).
            clone (bool, optional): True to return a clone of the internal buffer. Otherwise False. Defaults to True.

        Returns:
            Union[Tuple[np.ndarray, np.ndarray], Tuple[torch.Tensor, torch.Tensor]]: position tensor with shape (M, max_simulation_mesh_vertices_per_body, 3)
        """
        indices = self._backend_utils.resolve_indices(indices, self.count, self._device)
        if not omni.timeline.get_timeline_interface().is_stopped() and self._physics_view is not None:
            positions = self._physics_view.get_sim_nodal_positions()
            if not clone:
                return positions[indices].reshape(len(indices), -1, 3)
            else:
                return self._backend_utils.clone_tensor(
                    positions[indices].reshape(len(indices), -1, 3), device=self._device
                )
        else:
            positions = self._backend_utils.create_zeros_tensor(
                [indices.shape[0], self.max_simulation_mesh_vertices_per_body, 3], dtype="float32", device=self._device
            )
            write_idx = 0
            for i in indices:
                self._apply_deformable_body_api(i.tolist())
                points = self._deformable_body_apis[i.tolist()].GetSimulationPointsAttr().Get()
                if points is None:
                    raise Exception(f"The prim {self.name} does not have points attribute.")
                positions[write_idx] = self._backend_utils.create_tensor_from_list(
                    points, dtype="float32", device=self._device
                ).view(self.max_simulation_mesh_vertices_per_body, 3)
                write_idx += 1
            return positions

    def get_simulation_mesh_nodal_velocities(
        self, indices: Optional[Union[np.ndarray, list, torch.Tensor]] = None, clone: bool = True
    ) -> Union[np.ndarray, torch.Tensor]:
        """Gets the vertex velocities for the deformable bodies indicated by the indices.

        Args:
            indices (Optional[Union[np.ndarray, list, torch.Tensor]], optional): indices to specify which deformable prims to query. Shape (M,).
                                                                                 Where M <= size of the encapsulated prims in the view.
                                                                                 Defaults to None (i.e: all prims in the view).
            clone (bool, optional): True to return a clone of the internal buffer. Otherwise False. Defaults to True.

        Returns:
            Union[Tuple[np.ndarray, np.ndarray], Tuple[torch.Tensor, torch.Tensor]]: velocity tensor with shape (M, max_simulation_mesh_vertices_per_body, 3)
        """
        indices = self._backend_utils.resolve_indices(indices, self.count, self._device)
        if not omni.timeline.get_timeline_interface().is_stopped() and self._physics_view is not None:
            velocities = self._physics_view.get_sim_nodal_velocities()
            if not clone:
                return velocities[indices].reshape(len(indices), -1, 3)
            else:
                return self._backend_utils.clone_tensor(
                    velocities[indices].reshape(len(indices), -1, 3), device=self._device
                )
        else:
            velocities = self._backend_utils.create_zeros_tensor(
                [indices.shape[0], self.max_simulation_mesh_vertices_per_body, 3], dtype="float32", device=self._device
            )
            write_idx = 0
            for i in indices:
                self._apply_deformable_api(i.tolist())
                point_velocities = self._deformable_apis[i.tolist()].GetSimulationVelocitiesAttr().Get()
                if point_velocities is None:
                    raise Exception(f"The prim {self.name} does not have velocities attribute.")
                velocities[write_idx] = self._backend_utils.create_tensor_from_list(
                    point_velocities, dtype="float32", device=self._device
                ).view(self.max_simulation_mesh_vertices_per_body, 3)
                write_idx += 1
            return velocities

    def get_simulation_mesh_kinematic_targets(
        self, indices: Optional[Union[np.ndarray, list, torch.Tensor]] = None, clone: bool = True
    ) -> Union[np.ndarray, torch.Tensor]:
        """Gets the nodal kinematic targets of the simulation mesh for the deformable bodies indicated by the indices.

        Args:
            indices (Optional[Union[np.ndarray, list, torch.Tensor]], optional): indices to specify which deformable prims to query. Shape (M,).
                                                                                 Where M <= size of the encapsulated prims in the view.
                                                                                 Defaults to None (i.e: all prims in the view).
            clone (bool, optional): True to return a clone of the internal buffer. Otherwise False. Defaults to True.

        Returns:
            Union[Tuple[np.ndarray, np.ndarray], Tuple[torch.Tensor, torch.Tensor]]: kinematic targets tensor,
                                                                                    with shape (M, max_simulation_mesh_vertices_per_body, 4)
                                                                                    the first three components are the position targets and the last value (0 or 1)
                                                                                    indicate whether the node is kinematically driven or not.
        """
        indices = self._backend_utils.resolve_indices(indices, self.count, self._device)
        if not omni.timeline.get_timeline_interface().is_stopped() and self._physics_view is not None:
            positions = self._physics_view.get_sim_kinematic_targets()
            if not clone:
                return positions[indices].reshape(len(indices), -1, 4)
            else:
                return self._backend_utils.clone_tensor(
                    positions[indices].reshape(len(indices), -1, 4), device=self._device
                )
        else:
            carb.log_warn(
                "Physics Simulation View is not created yet in order to use get_simulation_mesh_kinematic_targets"
            )
            return None

    def get_collision_mesh_nodal_positions(
        self, indices: Optional[Union[np.ndarray, list, torch.Tensor]] = None, clone: bool = True
    ) -> Union[np.ndarray, torch.Tensor]:
        """Gets the nodal positions of the collision mesh for the deformable bodies indicated by the indices.

        Args:
            indices (Optional[Union[np.ndarray, list, torch.Tensor]], optional): indices to specify which deformable prims to query. Shape (M,).
                                                                                 Where M <= size of the encapsulated prims in the view.
                                                                                 Defaults to None (i.e: all prims in the view).
            clone (bool, optional): True to return a clone of the internal buffer. Otherwise False. Defaults to True.

        Returns:
            Union[Tuple[np.ndarray, np.ndarray], Tuple[torch.Tensor, torch.Tensor]]: position tensor with shape (M, max_collision_mesh_vertices_per_body, 3)
        """
        indices = self._backend_utils.resolve_indices(indices, self.count, self._device)
        if not omni.timeline.get_timeline_interface().is_stopped() and self._physics_view is not None:
            positions = self._physics_view.get_nodal_positions()
            if not clone:
                return positions[indices].reshape(len(indices), -1, 3)
            else:
                return self._backend_utils.clone_tensor(
                    positions[indices].reshape(len(indices), -1, 3), device=self._device
                )
        else:
            positions = self._backend_utils.create_zeros_tensor(
                [indices.shape[0], self.max_collision_mesh_vertices_per_body, 3], dtype="float32", device=self._device
            )
            write_idx = 0
            for i in indices:
                self._apply_deformable_body_api(i.tolist())
                points = self._prim[i.tolist()].GetPointsAttr().Get()
                if points is None:
                    raise Exception(f"The prim {self.name} does not have points attribute.")
                positions[write_idx] = self._backend_utils.create_tensor_from_list(
                    points, dtype="float32", device=self._device
                ).view(self.max_collision_mesh_vertices_per_body, 3)
                write_idx += 1
            return positions

    def get_simulation_mesh_indices(
        self, indices: Optional[Union[np.ndarray, list, torch.Tensor]] = None, clone: bool = True
    ) -> Union[np.ndarray, torch.Tensor]:
        """Gets the simulation mesh element indices of the deformable bodies indicated by the indices.

        Args:
            indices (Optional[Union[np.ndarray, list, torch.Tensor]], optional): indices to specify which deformable prims to query. Shape (M,).
                                                                                 Where M <= size of the encapsulated prims in the view.
                                                                                 Defaults to None (i.e: all prims in the view).
            clone (float, optional): True to return a clone of the internal buffer. Otherwise False. Defaults to True.

        Returns:
            Union[Tuple[np.ndarray, np.ndarray], Tuple[torch.Tensor, torch.Tensor]]: deformable bodies simulation mesh element indices
                                                                         with shape (M, self.max_simulation_mesh_elements_per_body, 4).
        """
        indices = self._backend_utils.resolve_indices(indices, self.count, self._device)
        if not omni.timeline.get_timeline_interface().is_stopped() and self._physics_view is not None:
            sim_indices = self._physics_view.get_sim_element_indices()
            if not clone:
                return sim_indices[indices].reshape(len(indices), -1, 4)
            else:
                return self._backend_utils.clone_tensor(
                    sim_indices[indices].reshape(len(indices), -1, 4), device=self._device
                )
        else:
            results = self._backend_utils.create_zeros_tensor(
                [indices.shape[0], self.max_simulation_mesh_elements_per_body, 4], dtype="int32", device=self._device
            )
            write_idx = 0
            for i in indices:
                self._apply_deformable_api(i.tolist())
                if "physxDeformable:simulationIndices" not in self._prims[i.tolist()].GetPropertyNames():
                    carb.log_warn(
                        f"physxDeformable:simulationIndices is not defined on the deformable prim {self.name}."
                    )
                    indices = self._deformable_apis[i.tolist()].CreateSimulationIndicesAttr().Get()
                else:
                    indices = self._deformable_apis[i.tolist()].GetSimulationIndicesAttr().Get()

                results[write_idx] = self._backend_utils.create_tensor_from_list(
                    indices, dtype="int32", device=self._device
                ).view(self.max_simulation_mesh_elements_per_body, 4)
                write_idx += 1
            return results

    def get_collision_mesh_indices(
        self, indices: Optional[Union[np.ndarray, list, torch.Tensor]] = None, clone: bool = True
    ) -> Union[np.ndarray, torch.Tensor]:
        """Gets the collision mesh element indices of the deformable bodies indicated by the indices.

        Args:
            indices (Optional[Union[np.ndarray, list, torch.Tensor]], optional): indices to specify which deformable prims to query. Shape (M,).
                                                                                 Where M <= size of the encapsulated prims in the view.
                                                                                 Defaults to None (i.e: all prims in the view).
            clone (float, optional): True to return a clone of the internal buffer. Otherwise False. Defaults to True.

        Returns:
            Union[Tuple[np.ndarray, np.ndarray], Tuple[torch.Tensor, torch.Tensor]]: deformable bodies collision mesh element indices
                                                                        with shape (M,  self.max_collision_mesh_elements_per_body, 4).
        """
        indices = self._backend_utils.resolve_indices(indices, self.count, self._device)
        if not omni.timeline.get_timeline_interface().is_stopped() and self._physics_view is not None:
            collision_indices = self._physics_view.get_element_indices()
            if not clone:
                return collision_indices[indices].reshape(len(indices), -1, 4)
            else:
                return self._backend_utils.clone_tensor(
                    collision_indices[indices].reshape(len(indices), -1, 4), device=self._device
                )
        else:
            carb.log_warn("Physics Simulation View is not created yet in order to use get_collision_mesh_indices")
            return None
            # TODO: there is discrepancy between max_collision_mesh_vertices_per_body and size of GetCollisionRestPointsAttr().Get()
            # the USD indices GetCollisionIndicesAttr().Get() are also different from the tensor API indices so the following should be revisited
            # results = self._backend_utils.create_zeros_tensor(
            #                     [indices.shape[0], self.max_collision_mesh_elements_per_body, 4], dtype="int32", device=self._device
            #                     )
            # write_idx = 0
            # for i in indices:
            #     self._apply_deformable_body_api(i.tolist())
            #     if "physxDeformable:collisionIndices" not in self._prims[i.tolist()].GetPropertyNames():
            #         carb.log_error(f"collisionIndices is not defined on the deformable prim: {self.name}.")
            #     else:
            #         indices = self._deformable_body_apis[i.tolist()].GetCollisionIndicesAttr().Get()
            #         results[write_idx] = self._backend_utils.create_tensor_from_list( indices, dtype="int32",
            #             device=self._device).view(self.max_collision_mesh_elements_per_body, 4)
            #         write_idx += 1
            # return results

    def get_simulation_mesh_rest_points(
        self, indices: Optional[Union[np.ndarray, list, torch.Tensor]] = None, clone: bool = True
    ) -> Union[np.ndarray, torch.Tensor]:
        """Gets the simulation mesh rest points of the deformable bodies indicated by the indices.
            rest point are the nodal positions with respect to the local prim transform, while the values returned by get_simulation_mesh_nodal_positions
            are the nodal positions with respect to the origin

        Args:
            indices (Optional[Union[np.ndarray, list, torch.Tensor]], optional): indices to specify which deformable prims to query. Shape (M,).
                                                                                 Where M <= size of the encapsulated prims in the view.
                                                                                 Defaults to None (i.e: all prims in the view).
            clone (float, optional): True to return a clone of the internal buffer. Otherwise False. Defaults to True.

        Returns:
            Union[Tuple[np.ndarray, np.ndarray], Tuple[torch.Tensor, torch.Tensor]]: deformable bodies simulation mesh rest points
                                                                                    with shape (M, self.max_simulation_mesh_vertices_per_body, 3).
        """
        indices = self._backend_utils.resolve_indices(indices, self.count, self._device)
        results = self._backend_utils.create_zeros_tensor(
            [indices.shape[0], self.max_simulation_mesh_vertices_per_body, 3], dtype="float32", device=self._device
        )
        write_idx = 0
        for i in indices:
            self._apply_deformable_body_api(i.tolist())
            if "physxDeformable:simulationRestPoints" not in self._prims[i.tolist()].GetPropertyNames():
                carb.log_warn(f"simulationRestPoints is not defined on the deformable prim: {self.name}.")
                points = self._deformable_body_apis[i.tolist()].CreateSimulationRestPointsAttr().Get()
            else:
                points = self._deformable_body_apis[i.tolist()].GetSimulationRestPointsAttr().Get()
            results[write_idx] = self._backend_utils.create_tensor_from_list(
                points, dtype="float32", device=self._device
            ).view(self.max_simulation_mesh_vertices_per_body, 3)
            write_idx += 1

        return results

    # def get_collision_mesh_rest_points(
    #     self, indices: Optional[Union[np.ndarray, list, torch.Tensor]] = None, clone: bool = True
    # ) -> Union[np.ndarray, torch.Tensor]:
    #     """Gets the collision mesh rest points of the deformable bodies indicated by the indices.
    #     TODO: there is discrepancy between max_collision_mesh_vertices_per_body and size of GetCollisionRestPointsAttr().Get()

    #     Args:
    #         indices (Optional[Union[np.ndarray, list, torch.Tensor]], optional): indices to specify which deformable prims to query. Shape (M,).
    #                                                                              Where M <= size of the encapsulated prims in the view.
    #                                                                              Defaults to None (i.e: all prims in the view).
    #         clone (float, optional): True to return a clone of the internal buffer. Otherwise False. Defaults to True.

    #     Returns:
    #         Union[Tuple[np.ndarray, np.ndarray], Tuple[torch.Tensor, torch.Tensor]]: deformable bodies collision mesh rest points
    #                                                                                 with shape (M, self.max_collision_mesh_vertices_per_body, 3).
    #     """
    #     indices = self._backend_utils.resolve_indices(indices, self.count, self._device)
    #     results = self._backend_utils.create_zeros_tensor([indices.shape[0], self.max_collision_mesh_vertices_per_body, 3],
    #                                                         dtype="float32", device=self._device)
    #     write_idx = 0
    #     for i in indices:
    #         self._apply_deformable_body_api(i.tolist())
    #         if "physxDeformable:collisionRestPoints" not in self._prims[i.tolist()].GetPropertyNames():
    #             carb.log_error(f"collisionRestPoints is not defined on the deformable prim: {self.name}.")
    #         else:
    #             points =  self._deformable_body_apis[i.tolist()].GetCollisionRestPointsAttr().Get()
    #             print(len(points), self.max_collision_mesh_vertices_per_body, self.max_simulation_mesh_vertices_per_body)
    #             results[write_idx] = self._backend_utils.create_tensor_from_list(points, dtype="float32",
    #                                 device=self._device).view(self.max_collision_mesh_vertices_per_body, 3)
    #         write_idx += 1
    #     return results

    def get_simulation_mesh_element_rest_poses(
        self, indices: Optional[Union[np.ndarray, list, torch.Tensor]] = None, clone: bool = True
    ) -> Union[np.ndarray, torch.Tensor]:
        """Gets the simulation mesh rest poses for the deformable bodies indicated by the indices.
        This method will return the 3x3 matrix inv([x1-x0, x2-x0, x3-x0]) where x0, x1, x2, x3 are the rest points of the simulation mesh elements

        Args:
            indices (Optional[Union[np.ndarray, list, torch.Tensor]], optional): indices to specify which deformable prims to query. Shape (M,).
                                                                                 Where M <= size of the encapsulated prims in the view.
                                                                                 Defaults to None (i.e: all prims in the view).
            clone (bool, optional): True to return a clone of the internal buffer. Otherwise False. Defaults to True.

        Returns:
            Union[Tuple[np.ndarray, np.ndarray], Tuple[torch.Tensor, torch.Tensor]]: simulation mesh rest poses with
                                                                                    shape (M, max_simulation_mesh_elements_per_body, 3, 3)
        """
        indices = self._backend_utils.resolve_indices(indices, self.count, self._device)
        if not omni.timeline.get_timeline_interface().is_stopped() and self._physics_view is not None:
            results = self._physics_view.get_sim_element_rest_poses()
            if not clone:
                return results[indices].reshape(len(indices), -1, 3, 3)
            else:
                return self._backend_utils.clone_tensor(
                    results[indices].reshape(len(indices), -1, 3, 3), device=self._device
                )
        else:
            carb.log_warn(
                "Physics Simulation View is not created yet in order to use get_simulation_mesh_element_rest_poses"
            )
            return None

    def get_collision_mesh_element_rest_poses(
        self, indices: Optional[Union[np.ndarray, list, torch.Tensor]] = None, clone: bool = True
    ) -> Union[np.ndarray, torch.Tensor]:
        """Gets the collision mesh rest poses for the deformable bodies indicated by the indices.
        This method will return the 3x3 matrix inv([x1-x0, x2-x0, x3-x0]) where x0, x1, x2, x3 are the rest points of collision mesh elements

        Args:
            indices (Optional[Union[np.ndarray, list, torch.Tensor]], optional): indices to specify which deformable prims to query. Shape (M,).
                                                                                 Where M <= size of the encapsulated prims in the view.
                                                                                 Defaults to None (i.e: all prims in the view).
            clone (bool, optional): True to return a clone of the internal buffer. Otherwise False. Defaults to True.

        Returns:
            Union[Tuple[np.ndarray, np.ndarray], Tuple[torch.Tensor, torch.Tensor]]: collision mesh rest poses with shape (M, max_collision_mesh_elements_per_body, 3, 3)
        """
        indices = self._backend_utils.resolve_indices(indices, self.count, self._device)
        if not omni.timeline.get_timeline_interface().is_stopped() and self._physics_view is not None:
            results = self._physics_view.get_element_rest_poses()
            if not clone:
                return results[indices].reshape(len(indices), -1, 3, 3)
            else:
                return self._backend_utils.clone_tensor(
                    results[indices].reshape(len(indices), -1, 3, 3), device=self._device
                )
        else:
            carb.log_warn(
                "Physics Simulation View is not created yet in order to use get_collision_mesh_element_rest_poses"
            )
            return None

    def get_simulation_mesh_element_rotations(
        self, indices: Optional[Union[np.ndarray, list, torch.Tensor]] = None, clone: bool = True
    ) -> Union[np.ndarray, torch.Tensor]:
        """Gets the simulation mesh element-wise rotations as quaternions for the deformable bodies indicated by the indices.

        Args:
            indices (Optional[Union[np.ndarray, list, torch.Tensor]], optional): indices to specify which deformable prims to query. Shape (M,).
                                                                                 Where M <= size of the encapsulated prims in the view.
                                                                                 Defaults to None (i.e: all prims in the view).
            clone (bool, optional): True to return a clone of the internal buffer. Otherwise False. Defaults to True.

        Returns:
            Union[Tuple[np.ndarray, np.ndarray], Tuple[torch.Tensor, torch.Tensor]]:  simulation mesh element-wise rotations with shape (M, max_simulation_mesh_elements_per_body, 4)
        """
        indices = self._backend_utils.resolve_indices(indices, self.count, self._device)
        if not omni.timeline.get_timeline_interface().is_stopped() and self._physics_view is not None:
            results = self._physics_view.get_sim_element_rotations()
            if not clone:
                return results[indices].reshape(len(indices), -1, 4)
            else:
                return self._backend_utils.clone_tensor(
                    results[indices].reshape(len(indices), -1, 4), device=self._device
                )
        else:
            carb.log_warn("Physics Simulation View is not created yet in order to use get_simulation_mesh_rotations")
            return None

    def get_collision_mesh_element_rotations(
        self, indices: Optional[Union[np.ndarray, list, torch.Tensor]] = None, clone: bool = True
    ) -> Union[np.ndarray, torch.Tensor]:
        """Gets the collision mesh element-wise rotations as quaternions for the deformable bodies indicated by the indices.

        Args:
            indices (Optional[Union[np.ndarray, list, torch.Tensor]], optional): indices to specify which deformable prims to query. Shape (M,).
                                                                                 Where M <= size of the encapsulated prims in the view.
                                                                                 Defaults to None (i.e: all prims in the view).
            clone (bool, optional): True to return a clone of the internal buffer. Otherwise False. Defaults to True.

        Returns:
            Union[Tuple[np.ndarray, np.ndarray], Tuple[torch.Tensor, torch.Tensor]]: collision mesh rotations with shape (M, max_collision_mesh_elements_per_body, 4)
        """
        indices = self._backend_utils.resolve_indices(indices, self.count, self._device)
        if not omni.timeline.get_timeline_interface().is_stopped() and self._physics_view is not None:
            results = self._physics_view.get_element_rotations()
            if not clone:
                return results[indices].reshape(len(indices), -1, 4)
            else:
                return self._backend_utils.clone_tensor(
                    results[indices].reshape(len(indices), -1, 4), device=self._device
                )
        else:
            carb.log_warn(
                "Physics Simulation View is not created yet in order to use get_collision_mesh_element_rotations"
            )
            return None

    def get_simulation_mesh_element_deformation_gradients(
        self, indices: Optional[Union[np.ndarray, list, torch.Tensor]] = None, clone: bool = True
    ) -> Union[np.ndarray, torch.Tensor]:
        """Gets the simulation mesh element-wise second-order deformation gradient tensors for the deformable bodies indicated by the indices.
        This method will return the simulation mesh element-wise deformation gradient of the deformable bodies

        Args:
            indices (Optional[Union[np.ndarray, list, torch.Tensor]], optional): indices to specify which deformable prims to query. Shape (M,).
                                                                                 Where M <= size of the encapsulated prims in the view.
                                                                                 Defaults to None (i.e: all prims in the view).
            clone (bool, optional): True to return a clone of the internal buffer. Otherwise False. Defaults to True.

        Returns:
            Union[Tuple[np.ndarray, np.ndarray], Tuple[torch.Tensor, torch.Tensor]]: simulation mesh element-wise deformation gradients with shape (M, max_simulation_mesh_elements_per_body, 3, 3)
        """
        indices = self._backend_utils.resolve_indices(indices, self.count, self._device)
        if not omni.timeline.get_timeline_interface().is_stopped() and self._physics_view is not None:
            results = self._physics_view.get_sim_element_deformation_gradients()
            if not clone:
                return results[indices].reshape(len(indices), -1, 3, 3)
            else:
                return self._backend_utils.clone_tensor(
                    results[indices].reshape(len(indices), -1, 3, 3), device=self._device
                )
        else:
            carb.log_warn(
                "Physics Simulation View is not created yet in order to use get_simulation_mesh_element_deformation_gradients"
            )
            return None

    def get_collision_mesh_element_deformation_gradients(
        self, indices: Optional[Union[np.ndarray, list, torch.Tensor]] = None, clone: bool = True
    ) -> Union[np.ndarray, torch.Tensor]:
        """Gets the collision mesh element-wise second-order deformation gradient tensors for the deformable bodies indicated by the indices.

        Args:
            indices (Optional[Union[np.ndarray, list, torch.Tensor]], optional): indices to specify which deformable prims to query. Shape (M,).
                                                                                 Where M <= size of the encapsulated prims in the view.
                                                                                 Defaults to None (i.e: all prims in the view).
            clone (bool, optional): True to return a clone of the internal buffer. Otherwise False. Defaults to True.

        Returns:
            Union[Tuple[np.ndarray, np.ndarray], Tuple[torch.Tensor, torch.Tensor]]: collision mesh deformation gradients with shape (M, max_collision_mesh_elements_per_body, 3, 3)
        """
        indices = self._backend_utils.resolve_indices(indices, self.count, self._device)
        if not omni.timeline.get_timeline_interface().is_stopped() and self._physics_view is not None:
            results = self._physics_view.get_element_deformation_gradients()
            if not clone:
                return results[indices].reshape(len(indices), -1, 3, 3)
            else:
                return self._backend_utils.clone_tensor(
                    results[indices].reshape(len(indices), -1, 3, 3), device=self._device
                )
        else:
            carb.log_warn(
                "Physics Simulation View is not created yet in order to use get_collision_mesh_element_deformation_gradients"
            )
            return None

    def get_simulation_mesh_element_stresses(
        self, indices: Optional[Union[np.ndarray, list, torch.Tensor]] = None, clone: bool = True
    ) -> Union[np.ndarray, torch.Tensor]:
        """Gets the simulation mesh element-wise second-order stress tensors for the deformable bodies indicated by the indices.

        Args:
            indices (Optional[Union[np.ndarray, list, torch.Tensor]], optional): indices to specify which deformable prims to query. Shape (M,).
                                                                                 Where M <= size of the encapsulated prims in the view.
                                                                                 Defaults to None (i.e: all prims in the view).
            clone (bool, optional): True to return a clone of the internal buffer. Otherwise False. Defaults to True.

        Returns:
            Union[Tuple[np.ndarray, np.ndarray], Tuple[torch.Tensor, torch.Tensor]]:
            simulation mesh element-wise stresses with shape (M, max_simulation_mesh_elements_per_body, 3, 3)
        """
        indices = self._backend_utils.resolve_indices(indices, self.count, self._device)
        if not omni.timeline.get_timeline_interface().is_stopped() and self._physics_view is not None:
            results = self._physics_view.get_sim_element_stresses()
            if not clone:
                return results[indices].reshape(len(indices), -1, 3, 3)
            else:
                return self._backend_utils.clone_tensor(
                    results[indices].reshape(len(indices), -1, 3, 3), device=self._device
                )
        else:
            carb.log_warn(
                "Physics Simulation View is not created yet in order to use get_simulation_mesh_element_stresses"
            )
            return None

    def get_collision_mesh_element_stresses(
        self, indices: Optional[Union[np.ndarray, list, torch.Tensor]] = None, clone: bool = True
    ) -> Union[np.ndarray, torch.Tensor]:
        """Gets the collision mesh element-wise second-order stress tensors for bodies indicated by the indices.
        This method will return the collision mesh element-wise stresses of the deformable bodies

        Args:
            indices (Optional[Union[np.ndarray, list, torch.Tensor]], optional): indices to specify which deformable prims to query. Shape (M,).
                                                                                 Where M <= size of the encapsulated prims in the view.
                                                                                 Defaults to None (i.e: all prims in the view).
            clone (bool, optional): True to return a clone of the internal buffer. Otherwise False. Defaults to True.

        Returns:
            Union[Tuple[np.ndarray, np.ndarray], Tuple[torch.Tensor, torch.Tensor]]: collision mesh stresses with shape (M, max_collision_mesh_elements_per_body, 3, 3)
        """
        indices = self._backend_utils.resolve_indices(indices, self.count, self._device)
        if not omni.timeline.get_timeline_interface().is_stopped() and self._physics_view is not None:
            results = self._physics_view.get_element_stresses()
            if not clone:
                return results[indices].reshape(len(indices), -1, 3, 3)
            else:
                return self._backend_utils.clone_tensor(
                    results[indices].reshape(len(indices), -1, 3, 3), device=self._device
                )
        else:
            carb.log_warn(
                "Physics Simulation View is not created yet in order to use get_collision_mesh_element_stresses"
            )
            return None
