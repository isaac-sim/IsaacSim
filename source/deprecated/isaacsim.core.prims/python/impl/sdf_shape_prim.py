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

"""Provides high-level functionality for handling geometry prims that provide their Signed Distance Field (SDF)."""

from __future__ import annotations

import carb
import numpy as np
import omni.kit.app
import warp as wp
from isaacsim.core.deprecation_manager import import_module
from isaacsim.core.utils.prims import find_matching_prim_paths, get_prim_at_path
from pxr import PhysxSchema, UsdGeom, UsdPhysics

from .geometry_prim import GeometryPrim

torch = import_module("torch")


class SdfShapePrim(GeometryPrim):
    """High level functions to deal with geometry prims that provide their Signed Distance Field (SDF).

    This object wraps all matching mesh geometry prims found at the regex provided at the prim_paths_expr.

    Args:
        prim_paths_expr: Prim paths regex to encapsulate all prims that match it.
            Example: "/World/Env[1-5]/Microwave" will match /World/Env1/Microwave,
            /World/Env2/Microwave..etc.
            (a non regex prim path can also be used to encapsulate one XForm).
        num_query_points: Number of points queried by this view object.
        prepare_sdf_schemas: Apply PhysxSDFMeshCollisionAPI to prims in prim_paths_expr.
        name: Shortname to be used as a key by Scene class.
            Note: needs to be unique if the object is added to the Scene.
        positions: Default positions in the world frame of the prim.
            Shape is (N, 3).
        translations: Default translations in the local frame of the prims
            (with respect to its parent prims). Shape is (N, 3).
        orientations: Default quaternion orientations in the world/ local frame of the prim
            (depends if translation or position is specified).
            Quaternion is scalar-first (w, x, y, z). Shape is (N, 4).
        scales: Local scales to be applied to the prim's dimensions. Shape is (N, 3).
        visibilities: Set to false for an invisible prim in the stage while rendering. Shape is (N,).
        reset_xform_properties: True if the prims don't have the right set of xform properties
            (i.e: translate, orient and scale) ONLY and in that order.
            Set this parameter to False if the object were cloned using using
            the cloner api in isaacsim.core.cloner.
        collisions: Set to True if the geometry already have/
            should have a collider (i.e not only a visual geometry). Shape is (N,).
        track_contact_forces: If enabled, the view will track the net contact forces on each geometry prim
            in the view. Note that the collision flag should be set to True to report
            contact forces.
        prepare_contact_sensors: Applies contact reporter API to the prim if it already does not have one.
        disable_stablization: Disables the contact stabilization parameter in the physics context.
        contact_filter_prim_paths_expr: A list of filter expressions which allows for tracking
            contact forces between the geometry prim and this subset
            through get_contact_force_matrix().
    """

    def __init__(
        self,
        prim_paths_expr: str,
        num_query_points: int,
        prepare_sdf_schemas: bool = True,
        name: str = "sdf_shape_view",
        positions: np.ndarray | torch.Tensor | wp.array | None = None,
        translations: np.ndarray | torch.Tensor | wp.array | None = None,
        orientations: np.ndarray | torch.Tensor | wp.array | None = None,
        scales: np.ndarray | torch.Tensor | wp.array | None = None,
        visibilities: np.ndarray | torch.Tensor | wp.array | None = None,
        reset_xform_properties: bool = True,
        collisions: np.ndarray | torch.Tensor | wp.array | None = None,
        track_contact_forces: bool = False,
        prepare_contact_sensors: bool = False,
        disable_stablization: bool = True,
        contact_filter_prim_paths_expr: list[str] | None = None,
    ) -> None:
        if contact_filter_prim_paths_expr is None:
            contact_filter_prim_paths_expr = []
        GeometryPrim.__init__(
            self,
            prim_paths_expr=prim_paths_expr,
            name=name,
            positions=positions,
            translations=translations,
            orientations=orientations,
            scales=scales,
            visibilities=visibilities,
            reset_xform_properties=reset_xform_properties,
            collisions=collisions,
            track_contact_forces=track_contact_forces,
            prepare_contact_sensors=prepare_contact_sensors,
            disable_stablization=disable_stablization,
            contact_filter_prim_paths_expr=contact_filter_prim_paths_expr,
        )

        self._num_query_points = num_query_points
        self._physics_view = None

        if prepare_sdf_schemas:
            self._prim_paths = find_matching_prim_paths(prim_paths_expr)
            for path in self._prim_paths:
                prim = get_prim_at_path(path)
                if not prim.IsA(UsdGeom.Mesh):
                    carb.log_error(f"prim at path'{path}' is not a UsdGeom.Mesh and cannot provide sdf information!")
                else:
                    self._apply_sdf_schema(get_prim_at_path(path))

        self._sdf_collision_apis = [None] * self._count

        return

    @property
    def num_query_points(self) -> int:
        """Number of points queried by this view object.

        Returns:
            Number of points queried by this view object.
        """
        return self._num_query_points

    def _apply_sdf_schema(self, prim_at_path: object) -> None:
        """Apply appropriate sdf schemas to prims.

        Args:
            prim_at_path: The prim to apply SDF schema to.
        """
        if not prim_at_path.HasAPI(UsdPhysics.CollisionAPI):
            UsdPhysics.CollisionAPI.Apply(prim_at_path)
        if not prim_at_path.HasAPI(UsdPhysics.MeshCollisionAPI):
            meshcollisionAPI = UsdPhysics.MeshCollisionAPI.Apply(prim_at_path)
        else:
            meshcollisionAPI = UsdPhysics.MeshCollisionAPI(prim_at_path)
        meshcollisionAPI.CreateApproximationAttr().Set("sdf")

        if not prim_at_path.HasAPI(PhysxSchema.PhysxSDFMeshCollisionAPI):
            PhysxSchema.PhysxSDFMeshCollisionAPI.Apply(prim_at_path)

    def is_physics_handle_valid(self) -> bool:
        """Whether the physics handle of the view is valid.

        Returns:
            True if the physics handle of the view is valid (i.e physics is initialized for the view).
            Otherwise False.
        """
        return self._physics_view is not None

    def initialize(self, physics_sim_view: omni.physics.tensors.SimulationView = None) -> None:
        """Create a physics simulation view if not passed and creates a sdf shape view in physX.

        Args:
            physics_sim_view: Current physics simulation view.
        """
        if physics_sim_view is None:
            physics_sim_view = omni.physics.tensors.create_simulation_view(self._backend)
            physics_sim_view.set_subspace_roots("/")
        carb.log_info(f"initializing view for {self._name}")
        self._physics_sim_view = physics_sim_view
        self._physics_view = physics_sim_view.create_sdf_shape_view(
            self._regex_prim_paths[0].replace(".*", "*"), self._num_query_points
        )
        if not carb.settings.get_settings().get_as_bool("/physics/suppressReadback"):
            carb.log_error("Using SDFShapeView requires the gpu pipeline or (a World initialized with a cuda device)")
        carb.log_info(f"SDF Shape View Device: {self._device}")
        self._num_shapes = self._physics_view.count
        self._num_query_points = self._physics_view.max_num_points
        return

    def get_sdf_and_gradients(
        self,
        points: np.ndarray | torch.Tensor,
        indices: np.ndarray | torch.Tensor | None = None,
        clone: bool = True,
    ) -> np.ndarray | torch.Tensor:
        """Get the SDF values and gradients of the query points.

        Args:
            points: Points (represented in the local frames of meshes) to be queried for sdf and gradients.
                Shape is (self.num_shapes, self.num_query_points, 3).
            indices: Indices to specify which prims to query. Shape (M,).
                Where M <= size of the encapsulated prims in the view.
            clone: True to return a clone of the internal buffer. Otherwise False.

        Returns:
            SDF values and gradients of points for prims with shape (self.num_shapes, self.num_query_points, 4).
            The first component is the SDF value while the last three represent the gradient
        """
        if not omni.timeline.get_timeline_interface().is_stopped() and self._physics_view is not None:
            indices = self._backend_utils.resolve_indices(indices, self._num_shapes, self._device)
            points = self._backend_utils.move_data(points, self._device)
            sdf_and_gradients = self._physics_view.get_sdf_and_gradients(points)
            if not clone:
                return sdf_and_gradients[indices]
            else:
                return self._backend_utils.clone_tensor(sdf_and_gradients[indices], device=self._device)
        else:
            carb.log_warn("Physics Simulation View is not created yet to use the SdfShapePrim")
            return None

    def get_sdf_margins(
        self, indices: np.ndarray | list | torch.Tensor | None = None, clone: bool = True
    ) -> np.ndarray | torch.Tensor:
        """Gets sdf margin values.

        Args:
            indices: Indices to specify which prims to query. Shape (M,).
                Where M <= size of the encapsulated prims in the view.
            clone: True to return a clone of the internal buffer. Otherwise False.

        Returns:
            Margins of the sdf collision apis for prims in the view. shape is (M,).
        """
        indices = self._backend_utils.resolve_indices(indices, self.count, self._device)
        values = np.zeros(indices.shape[0], dtype="float32")
        write_idx = 0
        indices = self._backend_utils.to_list(indices)
        for i in indices:
            if self._sdf_collision_apis[i] is None:
                if self._prims[i].HasAPI(PhysxSchema.PhysxSDFMeshCollisionAPI):
                    self._sdf_collision_apis[i] = PhysxSchema.PhysxSDFMeshCollisionAPI(self._prims[i])
                else:
                    self._sdf_collision_apis[i] = PhysxSchema.PhysxSDFMeshCollisionAPI.Apply(self._prims[i])
            values[write_idx] = self._sdf_collision_apis[i].GetSdfMarginAttr().Get()
            write_idx += 1
        values = self._backend_utils.convert(values, dtype="float32", device=self._device, indexed=True)

        return values

    def get_sdf_narrow_band_thickness(
        self, indices: np.ndarray | list | torch.Tensor | None = None, clone: bool = True
    ) -> np.ndarray | torch.Tensor:
        """Gets sdf collision narrow band thickness values.

        Args:
            indices: Indices to specify which prims to query. Shape (M,).
                Where M <= size of the encapsulated prims in the view.
            clone: True to return a clone of the internal buffer. Otherwise False.

        Returns:
            Narrow band thickness of the sdf collision apis for prims in the view. shape is (M,).
        """
        indices = self._backend_utils.resolve_indices(indices, self.count, self._device)
        values = np.zeros(indices.shape[0], dtype="float32")
        write_idx = 0
        indices = self._backend_utils.to_list(indices)
        for i in indices:
            if self._sdf_collision_apis[i] is None:
                if self._prims[i].HasAPI(PhysxSchema.PhysxSDFMeshCollisionAPI):
                    self._sdf_collision_apis[i] = PhysxSchema.PhysxSDFMeshCollisionAPI(self._prims[i])
                else:
                    self._sdf_collision_apis[i] = PhysxSchema.PhysxSDFMeshCollisionAPI.Apply(self._prims[i])
            values[write_idx] = self._sdf_collision_apis[i].GetSdfNarrowBandThicknessAttr().Get()
            write_idx += 1
        values = self._backend_utils.convert(values, dtype="float32", device=self._device, indexed=True)

        return values

    def get_sdf_subgrid_resolution(
        self, indices: np.ndarray | list | torch.Tensor | None = None, clone: bool = True
    ) -> np.ndarray | torch.Tensor:
        """Gets sdf collision subgrid resolution values.

        Args:
            indices: Indices to specify which prims to query. Shape (M,).
                Where M <= size of the encapsulated prims in the view.
            clone: True to return a clone of the internal buffer. Otherwise False.

        Returns:
            Subgrid resolutions of the sdf collision apis for prims in the view. shape is (M,).
        """
        indices = self._backend_utils.resolve_indices(indices, self.count, self._device)
        values = np.zeros(indices.shape[0], dtype="int32")
        write_idx = 0
        indices = self._backend_utils.to_list(indices)
        for i in indices:
            if self._sdf_collision_apis[i] is None:
                if self._prims[i].HasAPI(PhysxSchema.PhysxSDFMeshCollisionAPI):
                    self._sdf_collision_apis[i] = PhysxSchema.PhysxSDFMeshCollisionAPI(self._prims[i])
                else:
                    self._sdf_collision_apis[i] = PhysxSchema.PhysxSDFMeshCollisionAPI.Apply(self._prims[i])
            values[write_idx] = self._sdf_collision_apis[i].GetSdfSubgridResolutionAttr().Get()
            write_idx += 1
        values = self._backend_utils.convert(values, dtype="int32", device=self._device, indexed=True)

        return values

    def get_sdf_resolution(
        self, indices: np.ndarray | list | torch.Tensor | None = None, clone: bool = True
    ) -> np.ndarray | torch.Tensor:
        """Gets sdf collision resolution values.

        Args:
            indices: Indices to specify which prims to query. Shape (M,).
                Where M <= size of the encapsulated prims in the view.
            clone: True to return a clone of the internal buffer. Otherwise False.

        Returns:
            Resolutions of the sdf collision apis for prims in the view. shape is (M,).
        """
        indices = self._backend_utils.resolve_indices(indices, self.count, self._device)
        values = np.zeros(indices.shape[0], dtype="float32")
        write_idx = 0
        indices = self._backend_utils.to_list(indices)
        for i in indices:
            if self._sdf_collision_apis[i] is None:
                if self._prims[i].HasAPI(PhysxSchema.PhysxSDFMeshCollisionAPI):
                    self._sdf_collision_apis[i] = PhysxSchema.PhysxSDFMeshCollisionAPI(self._prims[i])
                else:
                    self._sdf_collision_apis[i] = PhysxSchema.PhysxSDFMeshCollisionAPI.Apply(self._prims[i])
            values[write_idx] = self._sdf_collision_apis[i].GetSdfResolutionAttr().Get()
            write_idx += 1
        values = self._backend_utils.convert(values, dtype="float32", device=self._device, indexed=True)

        return values

    def set_sdf_margins(
        self, values: np.ndarray | torch.Tensor, indices: np.ndarray | list | torch.Tensor | None = None
    ) -> None:
        """Sets signed distance field margins for prims in the view.

        Args:
            values: Sdf margins to be set. shape (M,).
            indices: Indices to specify which prims to manipulate. Shape (M,).
                Where M <= size of the encapsulated prims in the view.
        """
        indices = self._backend_utils.resolve_indices(indices, self.count, self._device)
        indices = self._backend_utils.to_list(indices)
        read_idx = 0
        for i in indices:
            if self._sdf_collision_apis[i] is None:
                if self._prims[i].HasAPI(PhysxSchema.PhysxSDFMeshCollisionAPI):
                    self._sdf_collision_apis[i] = PhysxSchema.PhysxSDFMeshCollisionAPI(self._prims[i])
                else:
                    self._sdf_collision_apis[i] = PhysxSchema.PhysxSDFMeshCollisionAPI.Apply(self._prims[i])
            self._sdf_collision_apis[i].GetSdfMarginAttr().Set(values[read_idx].tolist())
            read_idx += 1

    def set_sdf_narrow_band_thickness(
        self, values: np.ndarray | torch.Tensor, indices: np.ndarray | list | torch.Tensor | None = None
    ) -> None:
        """Sets signed distance field narrow band thicknesses for prims in the view.

        Args:
            values: Sdf narrow band thicknesses to be set. shape (M,).
            indices: Indices to specify which prims to manipulate. Shape (M,).
                Where M <= size of the encapsulated prims in the view.
        """
        indices = self._backend_utils.resolve_indices(indices, self.count, self._device)
        indices = self._backend_utils.to_list(indices)
        read_idx = 0
        for i in indices:
            if self._sdf_collision_apis[i] is None:
                if self._prims[i].HasAPI(PhysxSchema.PhysxSDFMeshCollisionAPI):
                    self._sdf_collision_apis[i] = PhysxSchema.PhysxSDFMeshCollisionAPI(self._prims[i])
                else:
                    self._sdf_collision_apis[i] = PhysxSchema.PhysxSDFMeshCollisionAPI.Apply(self._prims[i])
            self._sdf_collision_apis[i].GetSdfNarrowBandThicknessAttr().Set(values[read_idx].tolist())
            read_idx += 1

    def set_sdf_subgrid_resolution(
        self, values: np.ndarray | torch.Tensor, indices: np.ndarray | list | torch.Tensor | None = None
    ) -> None:
        """Sets signed distance field subgrid resolutions for prims in the view.

        Args:
            values: Sdf subgrid resolutions to be set. shape (M,).
            indices: Indices to specify which prims to manipulate. Shape (M,).
                Where M <= size of the encapsulated prims in the view.
        """
        indices = self._backend_utils.resolve_indices(indices, self.count, self._device)
        indices = self._backend_utils.to_list(indices)
        read_idx = 0
        for i in indices:
            if self._sdf_collision_apis[i] is None:
                if self._prims[i].HasAPI(PhysxSchema.PhysxSDFMeshCollisionAPI):
                    self._sdf_collision_apis[i] = PhysxSchema.PhysxSDFMeshCollisionAPI(self._prims[i])
                else:
                    self._sdf_collision_apis[i] = PhysxSchema.PhysxSDFMeshCollisionAPI.Apply(self._prims[i])
            self._sdf_collision_apis[i].GetSdfSubgridResolutionAttr().Set(values[read_idx].tolist())
            read_idx += 1

    def set_sdf_resolution(
        self, values: np.ndarray | torch.Tensor, indices: np.ndarray | list | torch.Tensor | None = None
    ) -> None:
        """Sets signed distance field resolutions for prims in the view.

        Args:
            values: Sdf resolutions to be set. shape (M,).
            indices: Indices to specify which prims to manipulate. Shape (M,).
                Where M <= size of the encapsulated prims in the view.
        """
        indices = self._backend_utils.resolve_indices(indices, self.count, self._device)
        indices = self._backend_utils.to_list(indices)
        read_idx = 0
        for i in indices:
            if self._sdf_collision_apis[i] is None:
                if self._prims[i].HasAPI(PhysxSchema.PhysxSDFMeshCollisionAPI):
                    self._sdf_collision_apis[i] = PhysxSchema.PhysxSDFMeshCollisionAPI(self._prims[i])
                else:
                    self._sdf_collision_apis[i] = PhysxSchema.PhysxSDFMeshCollisionAPI.Apply(self._prims[i])
            self._sdf_collision_apis[i].GetSdfResolutionAttr().Set(values[read_idx].tolist())
            read_idx += 1
