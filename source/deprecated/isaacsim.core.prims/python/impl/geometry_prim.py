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

"""High level wrapper to deal with geom prims (one or many) as well as their attributes/properties."""

from __future__ import annotations

from typing import TYPE_CHECKING

import carb
import numpy as np
import omni.kit.app
import warp as wp
from isaacsim.core.deprecation_manager import import_module
from pxr import PhysxSchema, UsdGeom, UsdPhysics, UsdShade

if TYPE_CHECKING:
    from isaacsim.core.api.materials.physics_material import PhysicsMaterial

from .xform_prim import XFormPrim

torch = import_module("torch")


class GeometryPrim(XFormPrim):
    """High level wrapper to deal with geom prims (one or many) as well as their attributes/properties.

    This class wraps all matching geom prims found at the regex provided at the ``prim_paths_expr`` argument

    .. note::

        Each prim will have ``xformOp:orient``, ``xformOp:translate`` and ``xformOp:scale`` only post-init,
        unless it is a non-root articulation link.

    .. warning::

        The geometry prim view object must be initialized in order to be able to operate on it.
        See the ``initialize`` method for more details.

    .. warning::

        Some methods require the prims to have the Physx Collision API. Instantiate the class with the ``collision``
        parameter to a list of True values to apply the collision API.

    Args:
        prim_paths_expr: prim paths regex to encapsulate all prims that match it.
            example: "/World/Env[1-5]/Microwave" will match /World/Env1/Microwave,
            /World/Env2/Microwave..etc.
            (a non regex prim path can also be used to encapsulate one XForm).
        name: shortname to be used as a key by Scene class.
            Note: needs to be unique if the object is added to the Scene.
        positions: default positions in the world frame of the prim.
            shape is (N, 3).
        translations: default translations in the local frame of the prims
            (with respect to its parent prims). shape is (N, 3).
        orientations: default quaternion orientations in the world/ local frame of the prim
            (depends if translation or position is specified).
            quaternion is scalar-first (w, x, y, z). shape is (N, 4).
        scales: local scales to be applied to
            the prim's dimensions. shape is (N, 3).
        visibilities: set to false for an invisible prim in
            the stage while rendering. shape is (N,).
        reset_xform_properties: True if the prims don't have the right set of xform properties
            (i.e: translate, orient and scale) ONLY and in that order.
            Set this parameter to False if the object were cloned using using
            the cloner api in isaacsim.core.cloner.
        collisions: Set to True if the geometry already have/
            should have a collider (i.e not only a visual geometry). shape is (N,).
        track_contact_forces: if enabled, the view will track the net contact forces on each geometry prim
            in the view. Note that the collision flag should be set to True to report
            contact forces.
        prepare_contact_sensors: applies contact reporter API to the prim if it already does not have one.
        disable_stablization: disables the contact stabilization parameter in the physics context.
        contact_filter_prim_paths_expr: a list of filter expressions which allows for tracking
            contact forces between the geometry prim and this subset
            through get_contact_force_matrix().
        max_contact_count: maximum number of contact data to report when detailed contact information is needed

    Example:

    .. code-block:: python

        >>> import isaacsim.core.utils.stage as stage_utils
        >>> from isaacsim.core.cloner import GridCloner
        >>> from isaacsim.core.prims import GeometryPrim
        >>> from pxr import UsdGeom
        >>>
        >>> env_zero_path = "/World/envs/env_0"
        >>> num_envs = 5
        >>>
        >>> # clone the environment (num_envs)
        >>> cloner = GridCloner(spacing=1.5)
        >>> cloner.define_base_env(env_zero_path)
        >>> UsdGeom.Xform.Define(stage_utils.get_current_stage(), env_zero_path)
        >>> stage_utils.get_current_stage().DefinePrim(f"{env_zero_path}/Xform", "Xform")
        >>> stage_utils.get_current_stage().DefinePrim(f"{env_zero_path}/Xform/Cube", "Cube")
        >>> env_pos = cloner.clone(
        ...     source_prim_path=env_zero_path,
        ...     prim_paths=cloner.generate_paths("/World/envs/env", num_envs),
        ...     copy_from_source=True
        ... )
        >>>
        >>> # wrap the prims
        >>> prims = GeometryPrim(
        ...     prim_paths_expr="/World/envs/env.*/Xform",
        ...     name="geometry_prim_view",
        ...     collisions=[True] * num_envs
        ... )
        >>> prims
        <isaacsim.core.prims.geometry_prim.GeometryPrim object at 0x7f372bb21630>
    """

    def __init__(
        self,
        prim_paths_expr: str,
        name: str = "geometry_prim_view",
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
        max_contact_count: int = 0,
    ) -> None:
        if contact_filter_prim_paths_expr is None:
            contact_filter_prim_paths_expr = []
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
        self._geoms = [None] * self._count
        self._collision_apis = [None] * self._count
        self._mesh_collision_apis = [None] * self._count
        self._physx_collision_apis = [None] * self._count
        if collisions is not None:
            collisions = self._backend_utils.to_list(collisions)
        for i in range(self.count):
            prim = self._prims[i]
            if prim.IsA(UsdGeom.Cube):
                self._geoms[i] = UsdGeom.Cube(prim)
            elif prim.IsA(UsdGeom.Capsule):
                self._geoms[i] = UsdGeom.Capsule(prim)
            elif prim.IsA(UsdGeom.Cone):
                self._geoms[i] = UsdGeom.Cone(prim)
            elif prim.IsA(UsdGeom.Cylinder):
                self._geoms[i] = UsdGeom.Cylinder(prim)
            elif prim.IsA(UsdGeom.Sphere):
                self._geoms[i] = UsdGeom.Sphere(prim)
            elif prim.IsA(UsdGeom.Mesh):
                self._geoms[i] = UsdGeom.Mesh(prim)
            else:
                self._geoms[i] = UsdGeom.Gprim(prim)

            if collisions is not None and i < len(collisions):
                if collisions[i]:
                    self.apply_collision_apis([i])

        self._applied_physics_materials = [None] * self._count
        self._binding_apis = [None] * self._count

        self._track_contact_forces = track_contact_forces or len(contact_filter_prim_paths_expr) != 0
        self._contact_filter_prim_paths_expr = contact_filter_prim_paths_expr
        if self._track_contact_forces:
            from isaacsim.core.api.sensors.rigid_contact_view import RigidContactView

            self._contact_view = RigidContactView(
                prim_paths_expr=prim_paths_expr,
                filter_paths_expr=contact_filter_prim_paths_expr,
                name=name + "_contact",
                prepare_contact_sensors=prepare_contact_sensors,
                disable_stablization=disable_stablization,
                max_contact_count=max_contact_count,
            )
        return

    @property
    def geoms(self) -> list[UsdGeom.Gprim]:
        """USD geom objects encapsulated.

        Returns:
            USD geom objects encapsulated.

        Example:

        .. code-block:: python

            >>> prims.geoms
            [UsdGeom.Gprim(Usd.Prim(</World/envs/env_0/Xform>)), UsdGeom.Gprim(Usd.Prim(</World/envs/env_1/Xform>)),
             UsdGeom.Gprim(Usd.Prim(</World/envs/env_2/Xform>)), UsdGeom.Gprim(Usd.Prim(</World/envs/env_3/Xform>)),
             UsdGeom.Gprim(Usd.Prim(</World/envs/env_4/Xform>))]
        """
        return self._geoms

    def initialize(self, physics_sim_view: omni.physics.tensors.SimulationView | None = None) -> None:
        """Create a physics simulation view if not passed and set other properties using the PhysX tensor API.

        .. note::

            If the rigid prim view has been added to the world scene (e.g., ``world.scene.add(prims)``),
            it will be automatically initialized when the world is reset (e.g., ``world.reset()``).

        .. warning::

            This method needs to be called after each hard reset (e.g., Stop + Play on the timeline)
            before interacting with any other class method.

        Args:
            physics_sim_view: Current physics simulation view.

        Example:

        .. code-block:: python

            >>> prims.initialize()
        """
        if self._track_contact_forces:
            self._contact_view.initialize(physics_sim_view)
        return

    def set_contact_offsets(
        self,
        offsets: np.ndarray | torch.Tensor | wp.array,
        indices: np.ndarray | list | torch.Tensor | wp.array | None = None,
    ) -> None:
        """Set contact offsets for prims in the view.

        Shapes whose distance is less than the sum of their contact offset values will generate contacts

        Search for *Advanced Collision Detection* in |physx_docs| for more details

        Args:
            offsets: Contact offsets of the collision shapes. Allowed range [maximum(0, rest_offset), 0].
                Default value is -inf, means default is picked by simulation based on the shape extent.
                Shape (M,).
            indices: Indices to specify which prims
                to manipulate. Shape (M,).
                Where M <= size of the encapsulated prims in the view.

        Example:

        .. code-block:: python

            >>> # set the contact offset for all the prims to the specified values.
            >>> prims.set_contact_offsets(np.full(num_envs, 0.02))
            >>>
            >>> # set the contact offset for the first, middle and last of the 5 envs
            >>> prims.set_contact_offsets(np.full(3, 0.02), indices=np.array([0, 2, 4]))
        """
        indices = self._backend_utils.resolve_indices(indices, self.count, self._device)
        read_idx = 0
        indices = self._backend_utils.to_list(indices)
        offsets = self._backend_utils.to_list(offsets)
        for i in indices:
            if self._physx_collision_apis[i] is None:
                if self._prims[i].HasAPI(PhysxSchema.PhysxCollisionAPI):
                    collision_api = PhysxSchema.PhysxCollisionAPI(self.prims[i])
                else:
                    collision_api = PhysxSchema.PhysxCollisionAPI.Apply(self._prims[i])
                self._physx_collision_apis[i] = collision_api
            self._physx_collision_apis[i].GetContactOffsetAttr().Set(offsets[read_idx])
            read_idx += 1
        return

    def get_contact_offsets(
        self, indices: np.ndarray | list | torch.Tensor | wp.array | None = None
    ) -> np.ndarray | torch.Tensor | wp.indexedarray:
        """Get contact offsets for prims in the view.

        Shapes whose distance is less than the sum of their contact offset values will generate contacts

        Search for *Advanced Collision Detection* in |physx_docs| for more details

        Args:
            indices: Indices to specify which prims
                to query. Shape (M,).
                Where M <= size of the encapsulated prims in the view.

        Returns:
            Contact offsets of the collision shapes. Shape is (M,).

        Example:

        .. code-block:: python

            >>> # get the contact offsets of all prims. Returned shape is (5,).
            >>> prims.get_contact_offsets()
            [-inf -inf -inf -inf -inf]
            >>>
            >>> # get the contact offsets of the prims for the first, middle and last of the 5 envs
            >>> prims.get_contact_offsets(indices=np.array([0, 2, 4]))
            [-inf -inf -inf]
        """
        indices = self._backend_utils.resolve_indices(indices, self.count, self._device)
        offsets = np.zeros(shape=indices.shape[0], dtype=np.float32)
        write_idx = 0
        indices = self._backend_utils.to_list(indices)
        for i in indices:
            if self._physx_collision_apis[i] is None:
                if self._prims[i].HasAPI(PhysxSchema.PhysxCollisionAPI):
                    self._physx_collision_apis[i] = PhysxSchema.PhysxCollisionAPI(self.prims[i])
            if self._physx_collision_apis[i] is not None:
                offsets[write_idx] = self._physx_collision_apis[i].GetContactOffsetAttr().Get()
            write_idx += 1
        offsets = self._backend_utils.convert(offsets, dtype="float32", device=self._device, indexed=True)
        return offsets

    def set_rest_offsets(
        self,
        offsets: np.ndarray | torch.Tensor | wp.array,
        indices: np.ndarray | list | torch.Tensor | wp.array | None = None,
    ) -> None:
        """Set rest offsets for prims in the view.

        Two shapes will come to rest at a distance equal to the sum of their rest offset values.
        If the rest offset is 0, they should converge to touching exactly

        Search for *Advanced Collision Detection* in |physx_docs| for more details

        .. warning::

            The contact offset must be positive and greater than the rest offset

        Args:
            offsets: Rest offset of a collision shape. Allowed range [-max_float, contact_offset].
                Default value is -inf, means default is picked by simulation.
                For rigid bodies its zero. Shape (M,).
            indices: Indices to specify which prims
                to manipulate. Shape (M,).
                Where M <= size of the encapsulated prims in the view.

        Example:

        .. code-block:: python

            >>> # set the rest offset for all the prims to the specified values.
            >>> prims.set_rest_offsets(np.full(num_envs, 0.01))
            >>>
            >>> # set the rest offset for the first, middle and last of the 5 envs
            >>> prims.set_rest_offsets(np.full(3, 0.01), indices=np.array([0, 2, 4]))
        """
        indices = self._backend_utils.resolve_indices(indices, self.count, self._device)
        read_idx = 0
        indices = self._backend_utils.to_list(indices)
        offsets = self._backend_utils.to_list(offsets)
        for i in indices:
            if self._physx_collision_apis[i] is None:
                if self._prims[i].HasAPI(PhysxSchema.PhysxCollisionAPI):
                    collision_api = PhysxSchema.PhysxCollisionAPI(self.prims[i])
                else:
                    collision_api = PhysxSchema.PhysxCollisionAPI.Apply(self._prims[i])
                self._physx_collision_apis[i] = collision_api
            self._physx_collision_apis[i].GetRestOffsetAttr().Set(offsets[read_idx])
            read_idx += 1
        return

    def get_rest_offsets(
        self, indices: np.ndarray | list | torch.Tensor | wp.array | None = None
    ) -> np.ndarray | torch.Tensor | wp.indexedarray:
        """Get rest offsets for prims in the view.

        Two shapes will come to rest at a distance equal to the sum of their rest offset values.
        If the rest offset is 0, they should converge to touching exactly

        Search for *Advanced Collision Detection* in |physx_docs| for more details

        Args:
            indices: Indices to specify which prims
                to query. Shape (M,).
                Where M <= size of the encapsulated prims in the view.

        Returns:
            Rest offsets of the collision shapes. Shape is (M,).

        Example:

        .. code-block:: python

            >>> # get the rest offsets of all prims. Returned shape is (5,).
            >>> prims.get_rest_offsets()
            [-inf -inf -inf -inf -inf]
            >>>
            >>> # get the rest offsets of the prims for the first, middle and last of the 5 envs
            >>> prims.get_rest_offsets(indices=np.array([0, 2, 4]))
            [-inf -inf -inf]
        """
        indices = self._backend_utils.resolve_indices(indices, self.count, self._device)
        offsets = np.zeros(indices.shape[0], dtype=np.float32)
        write_idx = 0
        indices = self._backend_utils.to_list(indices)
        for i in indices:
            if self._physx_collision_apis[i] is None:
                if self._prims[i].HasAPI(PhysxSchema.PhysxCollisionAPI):
                    self._physx_collision_apis[i] = PhysxSchema.PhysxCollisionAPI(self.prims[i])
            if self._physx_collision_apis[i] is not None:
                offsets[write_idx] = self._physx_collision_apis[i].GetRestOffsetAttr().Get()
            write_idx += 1
        offsets = self._backend_utils.convert(offsets, dtype="float32", device=self._device, indexed=True)
        return offsets

    def set_torsional_patch_radii(
        self,
        radii: np.ndarray | torch.Tensor | wp.array,
        indices: np.ndarray | list | torch.Tensor | wp.array | None = None,
    ) -> None:
        """Set torsional patch radii for prims in the view.

        Search for *"Torsional Patch Radius"* in |physx_docs| for more details

        Args:
            radii: Radius of the contact patch used to apply torsional friction. Allowed range [0, max_float].
                Shape is (M,).
            indices: Indices to specify which prims
                to manipulate. Shape (M,).
                Where M <= size of the encapsulated prims in the view.

        Example:

        .. code-block:: python

            >>> # set the torsional patch radius for all the prims to the specified values.
            >>> prims.set_torsional_patch_radii(np.full(num_envs, 0.1))
            >>>
            >>> # set the torsional patch radius for the first, middle and last of the 5 envs
            >>> prims.set_torsional_patch_radii(np.full(3, 0.1), indices=np.array([0, 2, 4]))
        """
        indices = self._backend_utils.resolve_indices(indices, self.count, self._device)
        read_idx = 0
        indices = self._backend_utils.to_list(indices)
        radii = self._backend_utils.to_list(radii)
        for i in indices:
            if self._physx_collision_apis[i] is None:
                if self._prims[i].HasAPI(PhysxSchema.PhysxCollisionAPI):
                    collision_api = PhysxSchema.PhysxCollisionAPI(self.prims[i])
                else:
                    collision_api = PhysxSchema.PhysxCollisionAPI.Apply(self._prims[i])
                self._physx_collision_apis[i] = collision_api
            self._physx_collision_apis[i].GetTorsionalPatchRadiusAttr().Set(radii[read_idx])
            read_idx += 1
        return

    def get_torsional_patch_radii(
        self, indices: np.ndarray | list | torch.Tensor | wp.array | None = None
    ) -> np.ndarray | torch.Tensor | wp.indexedarray:
        """Get torsional patch radii for prims in the view.

        Search for *"Torsional Patch Radius"* in |physx_docs| for more details

        Args:
            indices: Indices to specify which prims
                to query. Shape (M,).
                Where M <= size of the encapsulated prims in the view.

        Returns:
            Radius of the contact patch used to apply torsional friction. Shape is (M,).

        Example:

        .. code-block:: python

            >>> # get the torsional patch radius of all prims. Returned shape is (5,).
            >>> prims.get_torsional_patch_radii()
            [0. 0. 0. 0. 0.]
            >>>
            >>> # get the torsional patch radius of the prims for the first, middle and last of the 5 envs
            >>> prims.get_torsional_patch_radii(indices=np.array([0, 2, 4]))
            [0. 0. 0.]
        """
        indices = self._backend_utils.resolve_indices(indices, self.count, self._device)
        radii = np.zeros(indices.shape[0], dtype=np.float32)
        write_idx = 0
        indices = self._backend_utils.to_list(indices)
        for i in indices:
            if self._physx_collision_apis[i] is None:
                if self._prims[i].HasAPI(PhysxSchema.PhysxCollisionAPI):
                    self._physx_collision_apis[i] = PhysxSchema.PhysxCollisionAPI(self.prims[i])
            if self._physx_collision_apis[i] is not None:
                radii[write_idx] = self._physx_collision_apis[i].GetTorsionalPatchRadiusAttr().Get()
            write_idx += 1
        radii = self._backend_utils.convert(radii, dtype="float32", device=self._device, indexed=True)
        return radii

    def set_min_torsional_patch_radii(
        self,
        radii: np.ndarray | torch.Tensor | wp.array,
        indices: np.ndarray | list | torch.Tensor | wp.array | None = None,
    ) -> None:
        """Set minimum torsional patch radii for prims in the view.

        Search for *"Torsional Patch Radius"* in |physx_docs| for more details

        Args:
            radii: Minimum radius of the contact patch used to apply torsional friction.
                Allowed range [0, max_float]. Shape is (M, ).
            indices: Indices to specify which prims
                to manipulate. Shape (M,).
                Where M <= size of the encapsulated prims in the view.

        Example:

        .. code-block:: python

            >>> # set the minimum torsional patch radius for all the prims to the specified values.
            >>> prims.set_min_torsional_patch_radii(np.full(num_envs, 0.05))
            >>>
            >>> # set the minimum torsional patch radius for the first, middle and last of the 5 envs
            >>> prims.set_min_torsional_patch_radii(np.full(3, 0.05), indices=np.array([0, 2, 4]))
        """
        indices = self._backend_utils.resolve_indices(indices, self.count, self._device)
        read_idx = 0
        indices = self._backend_utils.to_list(indices)
        radii = self._backend_utils.to_list(radii)
        for i in indices:
            if self._physx_collision_apis[i] is None:
                if self._prims[i].HasAPI(PhysxSchema.PhysxCollisionAPI):
                    collision_api = PhysxSchema.PhysxCollisionAPI(self.prims[i])
                else:
                    collision_api = PhysxSchema.PhysxCollisionAPI.Apply(self._prims[i])
                self._physx_collision_apis[i] = collision_api
            self._physx_collision_apis[i].GetMinTorsionalPatchRadiusAttr().Set(radii[read_idx])
            read_idx += 1
        return

    def get_min_torsional_patch_radii(
        self, indices: np.ndarray | list | torch.Tensor | None = None
    ) -> np.ndarray | torch.Tensor:
        """Get minimum torsional patch radii for prims in the view.

        Search for *"Torsional Patch Radius"* in |physx_docs| for more details

        Args:
            indices: Indices to specify which prims
                to query. Shape (M,).
                Where M <= size of the encapsulated prims in the view.

        Returns:
            Minimum radius of the contact patch used to apply torsional friction. Shape is (M,).

        Example:

        .. code-block:: python

            >>> # get the minimum torsional patch radius of all prims. Returned shape is (5,).
            >>> prims.get_min_torsional_patch_radii()
            [0. 0. 0. 0. 0.]
            >>>
            >>> # get the minimum torsional patch radius of the prims for the first, middle and last of the 5 envs
            >>> prims.get_min_torsional_patch_radii(indices=np.array([0, 2, 4]))
            [0. 0. 0.]
        """
        indices = self._backend_utils.resolve_indices(indices, self.count, self._device)
        radii = np.zeros(indices.shape[0], dtype=np.float32)
        write_idx = 0
        indices = self._backend_utils.to_list(indices)
        for i in indices:
            if self._physx_collision_apis[i] is None:
                if self._prims[i].HasAPI(PhysxSchema.PhysxCollisionAPI):
                    self._physx_collision_apis[i] = PhysxSchema.PhysxCollisionAPI(self.prims[i])
            if self._physx_collision_apis[i] is not None:
                radii[write_idx] = self._physx_collision_apis[i].GetMinTorsionalPatchRadiusAttr().Get()
            write_idx += 1
        radii = self._backend_utils.convert(radii, dtype="float32", device=self._device, indexed=True)
        return radii

    def set_collision_approximations(
        self, approximation_types: list[str], indices: np.ndarray | list | torch.Tensor | wp.array | None = None
    ) -> None:
        """Set collision approximation types for prims in the view.

        .. list-table::
            :header-rows: 1

            * - Approximation
              - Full name
              - Description
            * - ``"none"``
              - Triangle Mesh
              - The mesh geometry is used directly as a collider without any approximation
            * - ``"convexDecomposition"``
              - Convex Decomposition
              - A convex mesh decomposition is performed. This results in a set of convex mesh colliders
            * - ``"convexHull"``
              - Convex Hull
              - A convex hull of the mesh is generated and used as the collider
            * - ``"boundingSphere"``
              - Bounding Sphere
              - A bounding sphere is computed around the mesh and used as a collider
            * - ``"boundingCube"``
              - Bounding Cube
              - An optimally fitting box collider is computed around the mesh
            * - ``"meshSimplification"``
              - Mesh Simplification
              - A mesh simplification step is performed, resulting in a simplified triangle mesh collider
            * - ``"sdf"``
              - SDF Mesh
              - SDF (Signed-Distance-Field) use high-detail triangle meshes as collision shape
            * - ``"sphereFill"``
              - Sphere Approximation
              - A sphere mesh decomposition is performed. This results in a set of sphere colliders

        Args:
            approximation_types: approximations used for collision. List size == M or the size of the view.
            indices: indices to specify which prims
                                                                                 to manipulate. Shape (M,).
                                                                                 Where M <= size of the encapsulated prims in the view.

        Example:

        .. code-block:: python

            >>> # set the collision approximations for all the prims to the specified values.
            >>> prims.set_collision_approximations(["convexDecomposition"] * num_envs)
            >>>
            >>> # set the collision approximations for the first, middle and last of the 5 envs
            >>> types = ["convexDecomposition", "convexHull", "meshSimplification"]
            >>> prims.set_collision_approximations(types, indices=np.array([0, 2, 4]))
        """
        indices = self._backend_utils.resolve_indices(indices, self.count, self._device)
        read_idx = 0
        indices = self._backend_utils.to_list(indices)
        for i in indices:
            if self._mesh_collision_apis[i] is None:
                if self._prims[i].HasAPI(UsdPhysics.MeshCollisionAPI):
                    collision_api = UsdPhysics.MeshCollisionAPI(self.prims[i])
                else:
                    collision_api = UsdPhysics.MeshCollisionAPI.Apply(self._prims[i])
                self._mesh_collision_apis[i] = collision_api
            self._mesh_collision_apis[i].GetApproximationAttr().Set(approximation_types[read_idx])
            read_idx += 1
        return

    def get_collision_approximations(
        self, indices: np.ndarray | list | torch.Tensor | wp.array | None = None
    ) -> list[str]:
        """Get collision approximation types for prims in the view.

        .. list-table::
            :header-rows: 1

            * - Approximation
              - Full name
              - Description
            * - ``"none"``
              - Triangle Mesh
              - The mesh geometry is used directly as a collider without any approximation
            * - ``"convexDecomposition"``
              - Convex Decomposition
              - A convex mesh decomposition is performed. This results in a set of convex mesh colliders
            * - ``"convexHull"``
              - Convex Hull
              - A convex hull of the mesh is generated and used as the collider
            * - ``"boundingSphere"``
              - Bounding Sphere
              - A bounding sphere is computed around the mesh and used as a collider
            * - ``"boundingCube"``
              - Bounding Cube
              - An optimally fitting box collider is computed around the mesh
            * - ``"meshSimplification"``
              - Mesh Simplification
              - A mesh simplification step is performed, resulting in a simplified triangle mesh collider
            * - ``"sdf"``
              - SDF Mesh
              - SDF (Signed-Distance-Field) use high-detail triangle meshes as collision shape
            * - ``"sphereFill"``
              - Sphere Approximation
              - A sphere mesh decomposition is performed. This results in a set of sphere colliders

        Args:
            indices: indices to specify which prims
                                                                                 to query. Shape (M,).
                                                                                 Where M <= size of the encapsulated prims in the view.

        Returns:
            approximations used for collision. size == M or size of the view.

        Example:

        .. code-block:: python

            >>> # get the collision approximation of all prims. Returned size is (5,).
            >>> prims.get_collision_approximations()
            ['none', 'none', 'none', 'none', 'none']
            >>>
            >>> # get the collision approximation of the prims for the first, middle and last of the 5 envs
            >>> prims.get_collision_approximations(indices=np.array([0, 2, 4]))
            ['none', 'none', 'none']
        """
        indices = self._backend_utils.resolve_indices(indices, self.count, self._device)
        approximation_types = [None] * indices.shape[0]
        write_idx = 0
        indices = self._backend_utils.to_list(indices)
        for i in indices:
            if self._mesh_collision_apis[i] is None:
                if self._prims[i].HasAPI(UsdPhysics.MeshCollisionAPI):
                    self._mesh_collision_apis[i] = UsdPhysics.MeshCollisionAPI(self.prims[i])
            if self._mesh_collision_apis[i] is not None:
                approximation_types[write_idx] = self._mesh_collision_apis[i].GetApproximationAttr().Get()
            else:
                approximation_types[write_idx] = "none"
            write_idx += 1
        return approximation_types

    def enable_collision(self, indices: np.ndarray | list | torch.Tensor | wp.array | None = None) -> None:
        """Enables collision on prims in the view.

        Args:
            indices: indices to specify which prims
                                                                                 to manipulate. Shape (M,).
                                                                                 Where M <= size of the encapsulated prims in the view.

        Example:

        .. code-block:: python

            >>> # enable the collision API for all prims
            >>> prims.enable_collision()
            >>>
            >>> # enable the collision API for the prims for the first, middle and last of the 5 envs
            >>> prims.enable_collision(indices=np.array([0, 2, 4]))
        """
        indices = self._backend_utils.resolve_indices(indices, self.count, self._device)
        indices = self._backend_utils.to_list(indices)
        for i in indices:
            if self._collision_apis[i] is None:
                self.apply_collision_apis([i])
            self._collision_apis[i].GetCollisionEnabledAttr().Set(True)
        return

    def disable_collision(self, indices: np.ndarray | list | torch.Tensor | wp.array | None = None) -> None:
        """Disables collision on prims in the view.

        Args:
            indices: indices to specify which prims
                                                                                 to manipulate. Shape (M,).
                                                                                 Where M <= size of the encapsulated prims in the view.

        Example:

        .. code-block:: python

            >>> # disable the collision API for all prims
            >>> prims.disable_collision()
            >>>
            >>> # disable the collision API for the prims for the first, middle and last of the 5 envs
            >>> prims.disable_collision(indices=np.array([0, 2, 4]))
        """
        indices = self._backend_utils.resolve_indices(indices, self.count, self._device)
        indices = self._backend_utils.to_list(indices)
        for i in indices:
            if self._collision_apis[i] is None:
                if self.prims[i].HasAPI(UsdPhysics.CollisionAPI):
                    self._collision_apis[i] = UsdPhysics.CollisionAPI(self.prims[i])
                else:
                    continue
            self._collision_apis[i].GetCollisionEnabledAttr().Set(False)
        return

    def is_collision_enabled(
        self, indices: np.ndarray | list | torch.Tensor | wp.array | None = None
    ) -> np.ndarray | torch.Tensor | wp.indexedarray:
        """Queries if collision is enabled on prims in the view.

        Args:
            indices: indices to specify which prims
                                                                                 to query. Shape (M,).
                                                                                 Where M <= size of the encapsulated prims in the view.

        Returns:
            True if collision is enabled. Shape is (M,).

        Example:

        .. code-block:: python

            >>> # check if the collision is enabled for all prims. Returned size is (5,).
            >>> prims.is_collision_enabled()
            [ True  True  True  True  True]
            >>>
            >>> # check if the collision is enabled for the first, middle and last of the 5 envs
            >>> prims.is_collision_enabled(indices=np.array([0, 2, 4]))
            [ True  True  True]
        """
        indices = self._backend_utils.resolve_indices(indices, self.count, self._device)
        collisions = np.zeros(shape=indices.shape[0], dtype=bool)
        write_idx = 0
        indices = self._backend_utils.to_list(indices)
        for i in indices:
            if self._collision_apis[i] is None:
                if self.prims[i].HasAPI(UsdPhysics.CollisionAPI):
                    self._collision_apis[i] = UsdPhysics.CollisionAPI(self.prims[i])
                    collisions[write_idx] = self._collision_apis[i].GetCollisionEnabledAttr().Get()
                else:
                    collisions[write_idx] = False
            else:
                collisions[write_idx] = self._collision_apis[i].GetCollisionEnabledAttr().Get()
            write_idx += 1
        collisions = self._backend_utils.convert(collisions, dtype="bool", device=self._device, indexed=True)
        return collisions

    def apply_collision_apis(self, indices: np.ndarray | list | torch.Tensor | wp.array | None = None) -> None:
        """Apply the collision API to prims in the view and update internal variables.

        Args:
            indices: indices to specify which prims
                                                                                 to manipulate. Shape (M,).
                                                                                 Where M <= size of the encapsulated prims in the view.

        Example:

        .. code-block:: python

            >>> # apply the collision API for all prims
            >>> prims.apply_collision_apis()
            >>>
            >>> # apply the collision API for the first, middle and last of the 5 envs
            >>> prims.apply_collision_apis(indices=np.array([0, 2, 4]))
        """
        indices = self._backend_utils.to_list(self._backend_utils.resolve_indices(indices, self.count, self._device))
        for i in indices:
            if self.prims[i].HasAPI(UsdPhysics.CollisionAPI):
                self._collision_apis[i] = UsdPhysics.CollisionAPI(self.prims[i])
            else:
                self._collision_apis[i] = UsdPhysics.CollisionAPI.Apply(self.prims[i])
            if self.prims[i].HasAPI(UsdPhysics.MeshCollisionAPI):
                self._mesh_collision_apis[i] = UsdPhysics.MeshCollisionAPI(self.prims[i])
            else:
                self._mesh_collision_apis[i] = UsdPhysics.MeshCollisionAPI.Apply(self.prims[i])
            if self.prims[i].HasAPI(PhysxSchema.PhysxCollisionAPI):
                self._physx_collision_apis[i] = PhysxSchema.PhysxCollisionAPI(self.prims[i])
            else:
                self._physx_collision_apis[i] = PhysxSchema.PhysxCollisionAPI.Apply(self.prims[i])
        return

    def apply_physics_materials(
        self,
        physics_materials: PhysicsMaterial | list[PhysicsMaterial],
        weaker_than_descendants: bool | list[bool] | None = None,
        indices: np.ndarray | list | torch.Tensor | wp.array | None = None,
    ) -> None:
        """Used to apply physics material to prims in the view and optionally its descendants.

        Args:
            physics_materials: physics materials to be applied to prims in the view.
                                                                            Physics material can be used to define friction, restitution..etc.
                                                                            Note: if a physics material is not defined, the defaults will be used
                                                                            from PhysX. If a list is provided then its size has to be equal
                                                                            the view's size or indices size.
                                                                            If one material is provided it will be applied to all prims in the view.
            weaker_than_descendants: True if the material shouldn't override the descendants
                                                                                    materials, otherwise False.
                                                                                    If a list of visual materials is provided then a list
                                                                                    has to be provided with the same size for this arg as well.
            indices: indices to specify which prims
                                                                                 to manipulate. Shape (M,).
                                                                                 Where M <= size of the encapsulated prims in the view.

        Raises:
            Exception: length of physics materials != length of prims indexed
            Exception: length of physics materials != length of weaker descendants arg

        Example:

        .. code-block:: python

            >>> from isaacsim.core.api.materials import PhysicsMaterial
            >>>
            >>> # create a rigid body physical material
            >>> material = PhysicsMaterial(
            ...     prim_path="/World/physics_material/aluminum",  # path to the material prim to create
            ...     dynamic_friction=0.4,
            ...     static_friction=1.1,
            ...     restitution=0.1
            ... )
            >>>
            >>> # apply the material to all prims
            >>> prims.apply_physics_materials(material)  # or [material] * num_envs
            >>>
            >>> # apply the collision API for the first, middle and last of the 5 envs
            >>> prims.apply_physics_materials(material, indices=np.array([0, 2, 4]))
        """
        indices = self._backend_utils.resolve_indices(indices, self.count, self._device)
        indices = self._backend_utils.to_list(indices)
        if isinstance(physics_materials, list):
            if len(indices) != len(physics_materials):
                raise Exception("length of physics materials != length of prims indexed")
            if weaker_than_descendants is None:
                weaker_than_descendants = [False] * len(physics_materials)
            if len(physics_materials) != len(weaker_than_descendants):
                raise Exception("length of physics materials != length of weaker descendants bools arg")
        if isinstance(physics_materials, list):
            read_idx = 0
            for i in indices:
                if self._binding_apis[i] is None:
                    if self._prims[i].HasAPI(UsdShade.MaterialBindingAPI):
                        self._binding_apis[i] = UsdShade.MaterialBindingAPI(self._prims[i])
                    else:
                        self._binding_apis[i] = UsdShade.MaterialBindingAPI.Apply(self._prims[i])
                if weaker_than_descendants[read_idx]:
                    self._binding_apis[i].Bind(
                        physics_materials[read_idx].material,
                        bindingStrength=UsdShade.Tokens.weakerThanDescendants,
                        materialPurpose="physics",
                    )
                else:
                    self._binding_apis[i].Bind(
                        physics_materials[read_idx].material,
                        bindingStrength=UsdShade.Tokens.strongerThanDescendants,
                        materialPurpose="physics",
                    )
                self._applied_physics_materials[i] = physics_materials[read_idx]
                read_idx += 1
            return
        else:
            if weaker_than_descendants is None:
                weaker_than_descendants = False
            for i in indices:
                if self._binding_apis[i] is None:
                    if self._prims[i].HasAPI(UsdShade.MaterialBindingAPI):
                        self._binding_apis[i] = UsdShade.MaterialBindingAPI(self._prims[i])
                    else:
                        self._binding_apis[i] = UsdShade.MaterialBindingAPI.Apply(self._prims[i])
                if weaker_than_descendants:
                    self._binding_apis[i].Bind(
                        physics_materials.material,
                        bindingStrength=UsdShade.Tokens.weakerThanDescendants,
                        materialPurpose="physics",
                    )
                else:
                    self._binding_apis[i].Bind(
                        physics_materials.material,
                        bindingStrength=UsdShade.Tokens.strongerThanDescendants,
                        materialPurpose="physics",
                    )
                self._applied_physics_materials[i] = physics_materials
        return

    def get_applied_physics_materials(
        self, indices: np.ndarray | list | torch.Tensor | wp.array | None = None
    ) -> list[PhysicsMaterial]:
        """Get the applied physics material to prims in the view.

        Args:
            indices: indices to specify which prims
                                                                                 to query. Shape (M,).
                                                                                 Where M <= size of the encapsulated prims in the view.

        Returns:
            the current applied physics materials for prims in the view.

        Example:

        .. code-block:: python

            >>> # get the applied material for all prims
            >>> prims.get_applied_physics_materials()
            [<isaacsim.core.api.materials.physics_material.PhysicsMaterial object at 0x7f720859ece0>,
             <isaacsim.core.api.materials.physics_material.PhysicsMaterial object at 0x7f720859ece0>,
             <isaacsim.core.api.materials.physics_material.PhysicsMaterial object at 0x7f720859ece0>,
             <isaacsim.core.api.materials.physics_material.PhysicsMaterial object at 0x7f720859ece0>,
             <isaacsim.core.api.materials.physics_material.PhysicsMaterial object at 0x7f720859ece0>]
            >>>
            >>> # get the applied material for the first, middle and last of the 5 envs
            >>> prims.get_applied_physics_materials(indices=np.array([0, 2, 4]))
            [<isaacsim.core.api.materials.physics_material.PhysicsMaterial object at 0x7f720859ece0>,
             <isaacsim.core.api.materials.physics_material.PhysicsMaterial object at 0x7f720859ece0>,
             <isaacsim.core.api.materials.physics_material.PhysicsMaterial object at 0x7f720859ece0>]
        """
        indices = self._backend_utils.resolve_indices(indices, self.count, self._device)
        result = [None] * indices.shape[0]
        write_idx = 0
        indices = self._backend_utils.to_list(indices)
        for i in indices:
            if self._binding_apis[i] is None:
                if self._prims[i].HasAPI(UsdShade.MaterialBindingAPI):
                    self._binding_apis[i] = UsdShade.MaterialBindingAPI(self._prims[i])
                else:
                    result[write_idx] = None
                    write_idx += 1
                    continue
            if self._applied_physics_materials[i] is not None:
                result[write_idx] = self._applied_physics_materials[i]
                write_idx += 1
            else:
                physics_binding = self._binding_apis[i].GetDirectBinding(materialPurpose="physics")
                material_path = physics_binding.GetMaterialPath()
                if material_path == "":
                    result[write_idx] = None
                else:
                    from isaacsim.core.api.materials.physics_material import PhysicsMaterial

                    self._applied_physics_materials[i] = PhysicsMaterial(prim_path=material_path)
                    result[write_idx] = self._applied_physics_materials[i]
                write_idx += 1
        return result

    def get_net_contact_forces(
        self,
        indices: np.ndarray | list | torch.Tensor | wp.array | None = None,
        clone: bool = True,
        dt: float = 1.0,
    ) -> np.ndarray | torch.Tensor | wp.indexedarray | None:
        """If contact forces of the prims in the view are tracked, this method returns the net contact forces on prims.

        i.e., a matrix of dimension (self.count, 3).

        Args:
            indices: indices to specify which prims
                                                                                 to query. Shape (M,).
                                                                                 Where M <= size of the encapsulated prims in the view.
            clone: True to return a clone of the internal buffer. Otherwise False.
            dt: time step multiplier to convert the underlying impulses to forces. If the default value is used then the forces are in fact contact impulses

        Returns:
            Net contact forces of the prims with shape (M,3). None if contact tracking is not enabled.
        """
        if self._track_contact_forces:
            return self._contact_view.get_net_contact_forces(indices, clone, dt)
        else:
            carb.log_warn(
                "contact forces cannot be retrieved with this API unless the GeometryPrim is initialized with track_contact_forces = True or a list of contact filters is provided via contact_filter_prim_paths_expr"
            )
            return None

    def get_contact_force_matrix(
        self,
        indices: np.ndarray | list | torch.Tensor | wp.array | None = None,
        clone: bool = True,
        dt: float = 1.0,
    ) -> np.ndarray | torch.Tensor | wp.indexedarray | None:
        """If the object is initialized with filter_paths_expr list, this method returns the contact forces between the prims.

        in the view and the filter prims. i.e., a matrix of dimension (self.count, self._contact_view.num_filters, 3)
        where num_filters is the determined according to the filter_paths_expr parameter.

        Args:
            indices: indices to specify which prims
                                                                                 to query. Shape (M,).
                                                                                 Where M <= size of the encapsulated prims in the view.
            clone: True to return a clone of the internal buffer. Otherwise False.
            dt: time step multiplier to convert the underlying impulses to forces. If the default value is used then the forces are in fact contact impulses

        Returns:
            Net contact forces of the prims with shape (M, self._contact_view.num_filters, 3). None if no contact filter is specified.
        """
        if len(self._contact_filter_prim_paths_expr) != 0:
            return self._contact_view.get_contact_force_matrix(indices, clone, dt)
        else:
            carb.log_warn(
                "No filter is specified for get_contact_force_matrix. Initialize the GeometryPrim with the contact_filter_prim_paths_expr and specify a list of filters."
            )
            return None

    def get_contact_force_data(
        self,
        indices: np.ndarray | list | torch.Tensor | wp.array | None = None,
        clone: bool = True,
        dt: float = 1.0,
    ) -> (
        tuple[
            np.ndarray | torch.Tensor | wp.indexedarray,
            np.ndarray | torch.Tensor | wp.indexedarray,
            np.ndarray | torch.Tensor | wp.indexedarray,
            np.ndarray | torch.Tensor | wp.indexedarray,
            np.ndarray | torch.Tensor | wp.indexedarray,
            np.ndarray | torch.Tensor | wp.indexedarray,
        ]
        | None
    ):
        """Get more detailed contact information between the prims in the view and the filter prims. Specifically, this method provides individual.

        contact normals, contact points, contact separations as well as contact forces for each pair
        (the sum of which equals the forces that the get_contact_force_matrix method provides as the force aggregate of a pair)
        Given to the dynamic nature of collision between bodies, this method will provide buffers of contact data which are arranged sequentially for each pair.
        The starting index and the number of contact data points for each pair in this stream can be realized from pair_contacts_start_indices,
        and pair_contacts_count tensors. They both have a dimension of (self.num_shapes, self.num_filters) where filter_count is determined
        according to the filter_paths_expr parameter.

        Args:
            indices: indices to specify which prims to query. Shape (M,).
                Where M <= size of the encapsulated prims in the view.
                Defaults to None (i.e: all prims in the view).
            clone: True to return a clone of the internal buffer. Otherwise False.
            dt: time step multiplier to convert the underlying impulses to forces. If the default value is used then the forces are in fact contact impulses

        Returns:
            A set of buffers for normal forces with shape (max_contact_count, 1), points with shape (max_contact_count, 3), normals with shape (max_contact_count, 3),
            and distances with shape (max_contact_count, 1), as well as two tensors with shape (M, self.num_filters)
            to indicate the starting index and the number of contact data points per pair in the aforementioned buffers.
        """
        if len(self._contact_filter_prim_paths_expr) != 0:
            return self._contact_view.get_contact_force_data(indices, clone, dt)
        else:
            carb.log_warn(
                "No filter is specified for get_contact_force_data. Initialize the GeometryPrim with the contact_filter_prim_paths_expr and specify a list of filters."
            )
            return None

    def get_friction_data(
        self,
        indices: np.ndarray | list | torch.Tensor | wp.array | None = None,
        clone: bool = True,
        dt: float = 1.0,
    ) -> (
        tuple[
            np.ndarray | torch.Tensor | wp.indexedarray,
            np.ndarray | torch.Tensor | wp.indexedarray,
            np.ndarray | torch.Tensor | wp.indexedarray,
            np.ndarray | torch.Tensor | wp.indexedarray,
        ]
        | None
    ):
        """Gets friction data between the prims in the view and the filter prims. Specifically, this method provides frictional contact forces,.

        and points. The data in reported for number of anchor points that includes tangential forces in a single tangent direction to contact normal.
        Given to the dynamic nature of collision between bodies, this method will provide buffers of friction data arranged sequentially for each pair.
        The starting index and the number of contact data points for each pair in this stream can be realized from pair_contacts_start_indices,
        and pair_contacts_count tensors. They both have a dimension of (self.num_shapes, self.num_filters) where filter_count is determined
        according to the filter_paths_expr parameter.

        Args:
            indices: indicies to specify which prims to query. Shape (M,).
                Where M <= size of the encapsulated prims in the view.
                Defaults to None (i.e: all prims in the view).
            clone: True to return a clone of the internal buffer. Otherwise False.
            dt: time step multiplier to convert the underlying impulses to forces. If the default value is used then the forces are in fact contact impulses

        Returns:
            A set of buffers for tangential forces per patch (at number of anchor points, each in a single directions)
            with shape (max_contact_count, 3), points with shape (max_contact_count, 3),
            as well as two tensors with shape (M, self.num_filters) to indicate the starting index and the number of
            contact data points per pair in the aforementioned buffers.
        """
        if len(self._contact_filter_prim_paths_expr) != 0:
            return self._contact_view.get_friction_data(indices, clone, dt)
        else:
            carb.log_warn(
                "No filter is specified for get_friction_data. Initialize the GeometryPrim with the contact_filter_prim_paths_expr and specify a list of filters."
            )
            return None
