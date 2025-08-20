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

from __future__ import annotations

import carb
import isaacsim.core.experimental.utils.ops as ops_utils
import numpy as np
import warp as wp
from pxr import PhysxSchema, UsdGeom, UsdPhysics, UsdShade

from .prim import _MSG_PRIM_NOT_VALID
from .xform_prim import XformPrim


class GeomPrim(XformPrim):
    """High level wrapper for manipulating geometric prims and their attributes.

    This class is a wrapper over one or more USD geometric prims in the stage to provide
    high-level functionality for manipulating collision properties, and other attributes.
    The prims are specified using paths that can include regular expressions for matching multiple prims.

    Args:
        paths: Single path or list of paths to USD prims. Can include regular expressions for matching multiple prims.
        resolve_paths: Whether to resolve the given paths (true) or use them as is (false).
        positions: Positions in the world frame (shape ``(N, 3)``).
            If the input shape is smaller than expected, data will be broadcasted (following NumPy broadcast rules).
        translations: Translations in the local frame (shape ``(N, 3)``).
            If the input shape is smaller than expected, data will be broadcasted (following NumPy broadcast rules).
        orientations: Orientations in the world frame (shape ``(N, 4)``, quaternion ``wxyz``).
            If the input shape is smaller than expected, data will be broadcasted (following NumPy broadcast rules).
        scales: Scales to be applied to the prims (shape ``(N, 3)``).
            If the input shape is smaller than expected, data will be broadcasted (following NumPy broadcast rules).
        reset_xform_op_properties: Whether to reset the transformation operation attributes of the prims to a standard set.
            See :py:meth:`reset_xform_op_properties` for more details.
        apply_collision_apis: Whether to apply collision APIs to the prims during initialization.

    Raises:
        ValueError: If no prims are found matching the specified path(s).
        AssertionError: If both positions and translations are specified.

    Example:

    .. code-block:: python

        >>> import numpy as np
        >>> from isaacsim.core.experimental.prims import GeomPrim
        >>>
        >>> # given a USD stage with the prims: /World/prim_0, /World/prim_1, and /World/prim_2
        >>> # - create wrapper over single prim (with collision APIs enabled)
        >>> prim = GeomPrim("/World/prim_0", apply_collision_apis=True)  # doctest: +NO_CHECK
        >>> # - create wrapper over multiple prims using regex (with collision APIs enabled)
        >>> prims = GeomPrim("/World/prim_.*", apply_collision_apis=True)  # doctest: +NO_CHECK
    """

    def __init__(
        self,
        paths: str | list[str],
        *,
        # Prim
        resolve_paths: bool = True,
        # XformPrim
        positions: list | np.ndarray | wp.array | None = None,
        translations: list | np.ndarray | wp.array | None = None,
        orientations: list | np.ndarray | wp.array | None = None,
        scales: list | np.ndarray | wp.array | None = None,
        reset_xform_op_properties: bool = False,
        # GeomPrim
        apply_collision_apis: bool = False,
    ) -> None:
        # define properties
        self._geoms = []
        # initialize base class
        super().__init__(
            paths,
            resolve_paths=resolve_paths,
            positions=positions,
            translations=translations,
            orientations=orientations,
            scales=scales,
            reset_xform_op_properties=reset_xform_op_properties,
        )
        # get geometric primitives
        for prim in self.prims:
            # shapes
            if prim.IsA(UsdGeom.Capsule):
                self._geoms.append(UsdGeom.Capsule(prim))
            elif prim.IsA(UsdGeom.Cone):
                self._geoms.append(UsdGeom.Cone(prim))
            elif prim.IsA(UsdGeom.Cube):
                self._geoms.append(UsdGeom.Cube(prim))
            elif prim.IsA(UsdGeom.Cylinder):
                self._geoms.append(UsdGeom.Cylinder(prim))
            elif prim.IsA(UsdGeom.Sphere):
                self._geoms.append(UsdGeom.Sphere(prim))
            # meshes
            elif prim.IsA(UsdGeom.Mesh):
                self._geoms.append(UsdGeom.Mesh(prim))
            # generic
            else:
                self._geoms.append(UsdGeom.Gprim(prim))
        # initialize instance from arguments
        if apply_collision_apis:
            self.apply_collision_apis()

    """
    Properties.
    """

    @property
    def geoms(self) -> list[UsdGeom.Gprim]:
        """USD geometric primitives encapsulated by the wrapper.

        Returns:
            List of USD geometric primitives.

        Example:

        .. code-block:: python

            >>> prims.geoms
            [UsdGeom.Gprim(Usd.Prim(</World/prim_0>)),
             UsdGeom.Gprim(Usd.Prim(</World/prim_1>)),
             UsdGeom.Gprim(Usd.Prim(</World/prim_2>))]
        """
        return self._geoms

    """
    Methods.
    """

    def set_offsets(
        self,
        contact_offsets: list | np.ndarray | wp.array = None,
        rest_offsets: list | np.ndarray | wp.array = None,
        *,
        indices: list | np.ndarray | wp.array | None = None,
    ) -> None:
        """Set the contact and rest offsets for collision detection between prims.

        Backends: :guilabel:`usd`.

        Shapes whose distance is less than the sum of their contact offset values will generate contacts.
        The rest offset determines the distance at which two shapes will come to rest.
        Search for *Advanced Collision Detection* in |physx_docs| for more details.

        .. warning::

            The contact offset must be positive and greater than the rest offset.

        Args:
            contact_offsets: Contact offsets of the collision shapes (shape ``(N, 1)``).
                If the input shape is smaller than expected, data will be broadcasted (following NumPy broadcast rules).
            rest_offsets: Rest offsets of the collision shapes (shape ``(N, 1)``).
                If the input shape is smaller than expected, data will be broadcasted (following NumPy broadcast rules).
            indices: Indices of prims to process (shape ``(N,)``). If not defined, all wrapped prims are processed.

        Raises:
            AssertionError: If neither contact_offsets nor rest_offsets are specified.
            AssertionError: Wrapped prims are not valid.

        Example:

        .. code-block:: python

            >>> # set same offsets (contact: 0.005, rest: 0.001) for all prims
            >>> prims.set_offsets(contact_offsets=[0.005], rest_offsets=[0.001])
            >>>
            >>> # set only the rest offsets for the second prim
            >>> prims.set_offsets(rest_offsets=[0.002], indices=[1])
        """
        assert (
            contact_offsets is not None or rest_offsets is not None
        ), "Both 'contact_offsets' and 'rest_offsets' are not defined. Define at least one of them"
        assert self.valid, _MSG_PRIM_NOT_VALID
        # USD API
        indices = ops_utils.resolve_indices(indices, count=len(self), device="cpu")
        if contact_offsets is not None:
            contact_offsets = ops_utils.place(contact_offsets, device="cpu").numpy().reshape((-1, 1))
        if rest_offsets is not None:
            rest_offsets = ops_utils.place(rest_offsets, device="cpu").numpy().reshape((-1, 1))
        for i, index in enumerate(indices.numpy()):
            physx_collision_api = GeomPrim.ensure_api([self.prims[index]], PhysxSchema.PhysxCollisionAPI)[0]
            if contact_offsets is not None:
                physx_collision_api.GetContactOffsetAttr().Set(
                    contact_offsets[0 if contact_offsets.shape[0] == 1 else i].item()
                )
            if rest_offsets is not None:
                physx_collision_api.GetRestOffsetAttr().Set(rest_offsets[0 if rest_offsets.shape[0] == 1 else i].item())

    def get_offsets(self, *, indices: list | np.ndarray | wp.array | None = None) -> tuple[wp.array, wp.array]:
        """Get the contact and rest offsets for collision detection between prims.

        Backends: :guilabel:`usd`.

        Shapes whose distance is less than the sum of their contact offset values will generate contacts.
        The rest offset determines the distance at which two shapes will come to rest.
        Search for *Advanced Collision Detection* in |physx_docs| for more details.

        Args:
            indices: Indices of prims to process (shape ``(N,)``). If not defined, all wrapped prims are processed.

        Returns:
            Two-elements tuple. 1) The contact offsets (shape ``(N, 1)``). 2) The rest offsets (shape ``(N, 1)``).

        Raises:
            AssertionError: Wrapped prims are not valid.

        Example:

        .. code-block:: python

            >>> # get the offsets of all prims
            >>> contact_offsets, rest_offsets = prims.get_offsets()
            >>> contact_offsets.shape, rest_offsets.shape
            ((3, 1), (3, 1))
            >>>
            >>> # get the offsets of the second prim
            >>> contact_offsets, rest_offsets = prims.get_offsets(indices=[1])
            >>> contact_offsets.shape, rest_offsets.shape
            ((1, 1), (1, 1))
        """
        assert self.valid, _MSG_PRIM_NOT_VALID
        # USD API
        indices = ops_utils.resolve_indices(indices, count=len(self), device="cpu")
        contact_offsets = np.zeros((indices.shape[0], 1), dtype=np.float32)
        rest_offsets = np.zeros((indices.shape[0], 1), dtype=np.float32)
        for i, index in enumerate(indices.numpy()):
            physx_collision_api = GeomPrim.ensure_api([self.prims[index]], PhysxSchema.PhysxCollisionAPI)[0]
            contact_offsets[i] = physx_collision_api.GetContactOffsetAttr().Get()
            rest_offsets[i] = physx_collision_api.GetRestOffsetAttr().Get()
        return ops_utils.place(contact_offsets, device=self._device), ops_utils.place(rest_offsets, device=self._device)

    def set_torsional_patch_radii(
        self,
        radii: list | np.ndarray | wp.array,
        *,
        indices: list | np.ndarray | wp.array | None = None,
        minimum: bool = False,
    ) -> None:
        """Set the torsional patch radii of the contact patches used to apply torsional frictions.

        Backends: :guilabel:`usd`.

        Search for *Torsional Patch Radius* in |physx_docs| for more details.

        Args:
            radii: Torsional patch radii (shape ``(N, 1)``).
                If the input shape is smaller than expected, data will be broadcasted (following NumPy broadcast rules).
            indices: Indices of prims to process (shape ``(N,)``). If not defined, all wrapped prims are processed.
            minimum: Whether to set the minimum torsional patch radii instead of the standard ones.

        Raises:
            AssertionError: Wrapped prims are not valid.

        Example:

        .. code-block:: python

            >>> # set the torsional patch radii for all prims
            >>> prims.set_torsional_patch_radii([0.1])
            >>>
            >>> # set the torsional patch radii for the first and last prims
            >>> prims.set_torsional_patch_radii([0.2], indices=np.array([0, 2]))
        """
        assert self.valid, _MSG_PRIM_NOT_VALID
        # USD API
        indices = ops_utils.resolve_indices(indices, count=len(self), device="cpu")
        radii = ops_utils.place(radii, device="cpu").numpy().reshape((-1, 1))
        for i, index in enumerate(indices.numpy()):
            radius = radii[0 if radii.shape[0] == 1 else i].item()
            physx_collision_api = GeomPrim.ensure_api([self.prims[index]], PhysxSchema.PhysxCollisionAPI)[0]
            if minimum:
                physx_collision_api.GetMinTorsionalPatchRadiusAttr().Set(radius)
            else:
                physx_collision_api.GetTorsionalPatchRadiusAttr().Set(radius)

    def get_torsional_patch_radii(
        self,
        *,
        indices: list | np.ndarray | wp.array | None = None,
        minimum: bool = False,
    ) -> wp.array:
        """Get the torsional patch radii of the contact patches used to apply torsional frictions.

        Backends: :guilabel:`usd`.

        Search for *Torsional Patch Radius* in |physx_docs| for more details.

        Args:
            indices: Indices of prims to process (shape ``(N,)``). If not defined, all wrapped prims are processed.
            minimum: Whether to get the minimum torsional patch radii instead of the standard ones.

        Returns:
            The (minimum) torsional patch radii (shape ``(N, 1)``).

        Raises:
            AssertionError: Wrapped prims are not valid.

        Example:

        .. code-block:: python

            >>> # get the torsional patch radii of all prims
            >>> radii = prims.get_torsional_patch_radii()
            >>> radii.shape
            (3, 1)
            >>>
            >>> # get the torsional patch radii of second prim
            >>> radii = prims.get_torsional_patch_radii(indices=[1])
            >>> radii.shape
            (1, 1)
        """
        assert self.valid, _MSG_PRIM_NOT_VALID
        # USD API
        indices = ops_utils.resolve_indices(indices, count=len(self), device="cpu")
        radii = np.zeros((indices.shape[0], 1), dtype=np.float32)
        for i, index in enumerate(indices.numpy()):
            physx_collision_api = GeomPrim.ensure_api([self.prims[index]], PhysxSchema.PhysxCollisionAPI)[0]
            if minimum:
                radii[i] = physx_collision_api.GetMinTorsionalPatchRadiusAttr().Get()
            else:
                radii[i] = physx_collision_api.GetTorsionalPatchRadiusAttr().Get()
        return ops_utils.place(radii, device=self._device)

    def set_collision_approximations(
        self, approximations: str | list[str], *, indices: list | np.ndarray | wp.array | None = None
    ) -> None:
        """Set the collision approximation types for mesh collision detection.

        Backends: :guilabel:`usd`.

        The collision approximation type determines how the mesh geometry is approximated for collision detection.
        Different approximations offer trade-offs between accuracy and performance.

        .. list-table::
            :header-rows: 1

            * - Approximation
              - Full name
              - Description
            * - ``"none"``
              - Triangle Mesh
              - The mesh geometry is used directly as a collider without any approximation.
            * - ``"convexDecomposition"``
              - Convex Decomposition
              - A convex mesh decomposition is performed. This results in a set of convex mesh colliders.
            * - ``"convexHull"``
              - Convex Hull
              - A convex hull of the mesh is generated and used as the collider.
            * - ``"boundingSphere"``
              - Bounding Sphere
              - A bounding sphere is computed around the mesh and used as a collider.
            * - ``"boundingCube"``
              - Bounding Cube
              - An optimally fitting box collider is computed around the mesh.
            * - ``"meshSimplification"``
              - Mesh Simplification
              - A mesh simplification step is performed, resulting in a simplified triangle mesh collider.
            * - ``"sdf"``
              - SDF Mesh
              - SDF (Signed-Distance-Field) use high-detail triangle meshes as collision shape.
            * - ``"sphereFill"``
              - Sphere Approximation
              - A sphere mesh decomposition is performed. This results in a set of sphere colliders.

        Args:
            approximations: Approximation types to use for collision detection (shape ``(N, 1)``).
                If the input shape is smaller than expected, data will be broadcasted (following NumPy broadcast rules).
            indices: Indices of prims to process (shape ``(N,)``). If not defined, all wrapped prims are processed.

        Raises:
            AssertionError: Wrapped prims are not valid.

        Example:

        .. code-block:: python

            >>> # set the collision approximations for all prims to 'convexDecomposition'
            >>> prims.set_collision_approximations(["convexDecomposition"])
        """
        assert self.valid, _MSG_PRIM_NOT_VALID
        # USD API
        indices = ops_utils.resolve_indices(indices, count=len(self), device="cpu")
        approximations = [approximations] if isinstance(approximations, str) else approximations
        broadcast = len(approximations) == 1
        for i, index in enumerate(indices.numpy()):
            mesh_collision_api = GeomPrim.ensure_api([self.prims[index]], UsdPhysics.MeshCollisionAPI)[0]
            mesh_collision_api.GetApproximationAttr().Set(approximations[0 if broadcast else i])

    def get_collision_approximations(self, *, indices: list | np.ndarray | wp.array | None = None) -> list[str]:
        """Get the collision approximation types for mesh collision detection.

        Backends: :guilabel:`usd`.

        The collision approximation type determines how the mesh geometry is approximated for collision detection.
        Different approximations offer trade-offs between accuracy and performance.

        .. list-table::
            :header-rows: 1

            * - Approximation
              - Full name
              - Description
            * - ``"none"``
              - Triangle Mesh
              - The mesh geometry is used directly as a collider without any approximation.
            * - ``"convexDecomposition"``
              - Convex Decomposition
              - A convex mesh decomposition is performed. This results in a set of convex mesh colliders.
            * - ``"convexHull"``
              - Convex Hull
              - A convex hull of the mesh is generated and used as the collider.
            * - ``"boundingSphere"``
              - Bounding Sphere
              - A bounding sphere is computed around the mesh and used as a collider.
            * - ``"boundingCube"``
              - Bounding Cube
              - An optimally fitting box collider is computed around the mesh.
            * - ``"meshSimplification"``
              - Mesh Simplification
              - A mesh simplification step is performed, resulting in a simplified triangle mesh collider.
            * - ``"sdf"``
              - SDF Mesh
              - SDF (Signed-Distance-Field) use high-detail triangle meshes as collision shape.
            * - ``"sphereFill"``
              - Sphere Approximation
              - A sphere mesh decomposition is performed. This results in a set of sphere colliders.

        Args:
            indices: Indices of prims to process (shape ``(N,)``). If not defined, all wrapped prims are processed.

        Returns:
            List of collision approximation types (shape ``(N,)``).

        Raises:
            AssertionError: Wrapped prims are not valid.

        Example:

        .. code-block:: python

            >>> # get the collision approximation of all prims after setting them to different types
            >>> prims.set_collision_approximations(["convexDecomposition", "convexHull", "boundingSphere"])
            >>> prims.get_collision_approximations()
            ['convexDecomposition', 'convexHull', 'boundingSphere']
        """
        assert self.valid, _MSG_PRIM_NOT_VALID
        # USD API
        indices = ops_utils.resolve_indices(indices, count=len(self), device="cpu")
        approximations = []
        for index in indices.numpy():
            mesh_collision_api = GeomPrim.ensure_api([self.prims[index]], UsdPhysics.MeshCollisionAPI)[0]
            approximations.append(mesh_collision_api.GetApproximationAttr().Get())
        return approximations

    def set_enabled_collisions(
        self,
        enabled: list | np.ndarray | wp.array,
        *,
        indices: list | np.ndarray | wp.array | None = None,
    ) -> None:
        """Enable or disable the collision API of the prims.

        Backends: :guilabel:`usd`.

        When disabled, the prims will not participate in collision detection.

        Args:
            enabled: Boolean flags to enable/disable collision APIs (shape ``(N, 1)``).
                If the input shape is smaller than expected, data will be broadcasted (following NumPy broadcast rules).
            indices: Indices of prims to process (shape ``(N,)``). If not defined, all wrapped prims are processed.

        Raises:
            AssertionError: Wrapped prims are not valid.

        Example:

        .. code-block:: python

            >>> # enable the collision API for all prims
            >>> prims.set_enabled_collisions([True])
            >>>
            >>> # disable the collision API for the first and last prims
            >>> prims.set_enabled_collisions([False], indices=[0, 2])
        """
        assert self.valid, _MSG_PRIM_NOT_VALID
        # USD API
        indices = ops_utils.resolve_indices(indices, count=len(self), device="cpu")
        enabled = ops_utils.place(enabled, device="cpu").numpy().reshape((-1, 1))
        for i, index in enumerate(indices.numpy()):
            prim = self.prims[index]
            value = bool(enabled[0 if enabled.shape[0] == 1 else i].item())
            if value:
                collision_api = GeomPrim.ensure_api([prim], UsdPhysics.CollisionAPI)[0]
                GeomPrim.ensure_api([prim], UsdPhysics.MeshCollisionAPI)
                GeomPrim.ensure_api([prim], PhysxSchema.PhysxCollisionAPI)
            elif prim.HasAPI(UsdPhysics.CollisionAPI):
                collision_api = UsdPhysics.CollisionAPI(prim)
            else:
                continue
            collision_api.GetCollisionEnabledAttr().Set(value)

    def get_enabled_collisions(
        self,
        *,
        indices: list | np.ndarray | wp.array | None = None,
    ) -> wp.array:
        """Get the enabled state of the collision API of the prims.

        Backends: :guilabel:`usd`.

        Args:
            indices: Indices of prims to process (shape ``(N,)``). If not defined, all wrapped prims are processed.

        Returns:
            Boolean flags indicating if the collision API is enabled (shape ``(N, 1)``).

        Raises:
            AssertionError: Wrapped prims are not valid.

        Example:

        .. code-block:: python

            >>> # get the collision enabled state of all prims after disabling it for the second prim
            >>> prims.set_enabled_collisions([False], indices=[1])
            >>> print(prims.get_enabled_collisions())
            [[ True]
             [False]
             [ True]]
        """
        assert self.valid, _MSG_PRIM_NOT_VALID
        # USD API
        indices = ops_utils.resolve_indices(indices, count=len(self), device="cpu")
        enabled = np.zeros((indices.shape[0], 1), dtype=np.bool_)
        for i, index in enumerate(indices.numpy()):
            prim = self.prims[index]
            if prim.HasAPI(UsdPhysics.CollisionAPI):
                collision_api = UsdPhysics.CollisionAPI(prim)
                enabled[i] = collision_api.GetCollisionEnabledAttr().Get()
        return ops_utils.place(enabled, device=self._device)

    def apply_collision_apis(self, *, indices: list | np.ndarray | wp.array | None = None) -> None:
        """Apply collision APIs to enable collision detection for prims.

        Backends: :guilabel:`usd`.

        This method applies the following APIs to enable collision detection:

        - USD: ``UsdPhysics.CollisionAPI`` and ``UsdPhysics.MeshCollisionAPI``
        - PhysX: ``PhysxSchema.PhysxCollisionAPI``

        .. note::

            If a prim in the parent hierarchy has the ``RigidBodyAPI`` applied, the collider is a part of that body.
            If there is no body in the parent hierarchy, the collider is considered to be static.

        Args:
            indices: Indices of prims to process (shape ``(N,)``). If not defined, all wrapped prims are processed.

        Example:

        .. code-block:: python

            >>> # apply the collision API to all prims
            >>> prims.apply_collision_apis()
        """
        # USD API
        indices = ops_utils.resolve_indices(indices, count=len(self), device="cpu")
        for index in indices.numpy():
            GeomPrim.ensure_api([self.prims[index]], UsdPhysics.CollisionAPI)
            GeomPrim.ensure_api([self.prims[index]], UsdPhysics.MeshCollisionAPI)
            GeomPrim.ensure_api([self.prims[index]], PhysxSchema.PhysxCollisionAPI)

    def apply_physics_materials(
        self,
        materials: type["PhysicsMaterial"] | list[type["PhysicsMaterial"]],
        *,
        weaker_than_descendants: list | np.ndarray | wp.array | None = None,
        indices: list | np.ndarray | wp.array | None = None,
    ) -> None:
        """Apply physics materials to the prims, and optionally, to their descendants.

        Backends: :guilabel:`usd`.

        Physics materials define properties like friction and restitution that affect how objects interact during collisions.
        If no physics material is defined, default values from Physics will be used.

        Args:
            materials: Physics materials to be applied to the prims (shape ``(N)``).
                If the input shape is smaller than expected, data will be broadcasted (following NumPy broadcast rules).
            weaker_than_descendants: Boolean flags to indicate whether descendant materials should be overridden (shape ``(N, 1)``).
                If the input shape is smaller than expected, data will be broadcasted (following NumPy broadcast rules).
            indices: Indices of prims to process (shape ``(N,)``). If not defined, all wrapped prims are processed.

        Raises:
            AssertionError: Wrapped prims are not valid.

        Example:

        .. code-block:: python

            >>> from isaacsim.core.experimental.materials import RigidBodyMaterial
            >>>
            >>> # create a rigid body physics material
            >>> material = RigidBodyMaterial(
            ...     "/World/physics_material/aluminum",
            ...     static_frictions=[1.1],
            ...     dynamic_frictions=[0.4],
            ...     restitutions=[0.1],
            ... )
            >>>
            >>> # apply the material to all prims
            >>> prims.apply_physics_materials(material)  # or [material]
        """
        assert self.valid, _MSG_PRIM_NOT_VALID
        # USD API
        indices = ops_utils.resolve_indices(indices, count=len(self), device="cpu")
        # accommodate values and determine broadcast status
        if not isinstance(materials, (list, tuple)):
            materials = [materials]
        broadcast_materials = len(materials) == 1
        if weaker_than_descendants is None:
            weaker_than_descendants = [False]
        weaker_than_descendants = ops_utils.place(weaker_than_descendants, device="cpu").numpy().reshape((-1, 1))
        broadcast_weaker_than_descendants = weaker_than_descendants.shape[0] == 1
        # set values
        for i, index in enumerate(indices.numpy()):
            material_binding_api = GeomPrim.ensure_api([self.prims[index]], UsdShade.MaterialBindingAPI)[0]
            binding_strength = (
                UsdShade.Tokens.weakerThanDescendants
                if weaker_than_descendants[0 if broadcast_weaker_than_descendants else i].item()
                else UsdShade.Tokens.strongerThanDescendants
            )
            material_binding_api.Bind(
                materials[0 if broadcast_materials else i].materials[0],
                bindingStrength=binding_strength,
                materialPurpose="physics",
            )

    def get_applied_physics_materials(
        self, *, indices: list | np.ndarray | wp.array | None = None
    ) -> list[type["PhysicsMaterial"] | None]:
        """Get the applied physics materials.

        Backends: :guilabel:`usd`.

        Args:
            indices: Indices of prims to process (shape ``(N,)``). If not defined, all wrapped prims are processed.

        Returns:
            List of applied physics materials (shape ``(N,)``). If a prim does not have a material, ``None`` is returned.

        Raises:
            AssertionError: Wrapped prims are not valid.

        Example:

        .. code-block:: python

            >>> # get the applied material path of the first prim
            >>> physics_material = prims.get_applied_physics_materials(indices=[0])[0]
            >>> physics_material.paths[0]
            '/World/physics_material/aluminum'
        """
        assert self.valid, _MSG_PRIM_NOT_VALID
        # USD API
        from isaacsim.core.experimental.materials import PhysicsMaterial  # defer imports to avoid circular dependencies

        indices = ops_utils.resolve_indices(indices, count=len(self), device="cpu")
        materials = []
        for index in indices.numpy():
            material_binding_api = GeomPrim.ensure_api([self.prims[index]], UsdShade.MaterialBindingAPI)[0]
            material_path = str(material_binding_api.GetDirectBinding(materialPurpose="physics").GetMaterialPath())
            material = None
            if material_path:
                material = PhysicsMaterial.fetch_instances([material_path])[0]
                if material is None:
                    carb.log_warn(f"Unsupported physics material ({material_path}): {self.paths[index]}")
            materials.append(material)
        return materials
