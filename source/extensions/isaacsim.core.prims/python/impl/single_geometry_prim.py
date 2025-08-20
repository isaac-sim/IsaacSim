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

from typing import List, Optional, Sequence, Union

import numpy as np
import torch
from pxr import UsdGeom

from .geometry_prim import GeometryPrim
from .single_prim_wrapper import _SinglePrimWrapper


class SingleGeometryPrim(_SinglePrimWrapper):
    """High level wrapper to deal with a Geom prim (only one geometry prim) and its attributes/properties.

    The ``prim_path`` should correspond to type UsdGeom.Cube, UsdGeom.Capsule, UsdGeom.Cone, UsdGeom.Cylinder,
    UsdGeom.Sphere or UsdGeom.Mesh.

    .. warning::

        The geometry object must be initialized in order to be able to operate on it.
        See the ``initialize`` method for more details.

    .. warning::

        Some methods require the prim to have the Physx Collision API. Instantiate the class with the ``collision``
        parameter to True to apply the collision API.

    Args:
        prim_path (str): prim path of the Prim to encapsulate or create.
        name (str, optional): shortname to be used as a key by Scene class.
                                Note: needs to be unique if the object is added to the Scene.
                                Defaults to "xform_prim".
        position (Optional[Sequence[float]], optional): position in the world frame of the prim. shape is (3, ).
                                                    Defaults to None, which means left unchanged.
        translation (Optional[Sequence[float]], optional): translation in the local frame of the prim
                                                        (with respect to its parent prim). shape is (3, ).
                                                        Defaults to None, which means left unchanged.
        orientation (Optional[Sequence[float]], optional): quaternion orientation in the world/ local frame of the prim
                                                        (depends if translation or position is specified).
                                                        quaternion is scalar-first (w, x, y, z). shape is (4, ).
                                                        Defaults to None, which means left unchanged.
        scale (Optional[Sequence[float]], optional): local scale to be applied to the prim's dimensions. shape is (3, ).
                                                Defaults to None, which means left unchanged.
        visible (bool, optional): set to false for an invisible prim in the stage while rendering. Defaults to True.
        reset_xform_properties (bool, optional): True if the prims don't have the right set of xform properties
                                                (i.e: translate, orient and scale) ONLY and in that order.
                                                Set this parameter to False if the object were cloned using using
                                                the cloner api in isaacsim.core.cloner. Defaults to True.
        collision (bool, optional): Set to True if the geometry should have a collider (i.e not only a visual geometry).
                                    Defaults to False.
        track_contact_forces (bool, Optional) : if enabled, the view will track the net contact forces on each geometry prim in the view.
                                                Note that the collision flag should be set to True to report contact forces. Defaults to False.
        prepare_contact_sensors (bool, Optional): applies contact reporter API to the prim if it already does not have one. Defaults to False.
        disable_stablization (bool, optional): disables the contact stabilization parameter in the physics context. Defaults to True.
        contact_filter_prim_paths_expr (Optional[List[str]], Optional): a list of filter expressions which allows for tracking contact forces
                                                                between the geometry prim and this subset through get_contact_force_matrix().

    Example:

    .. code-block:: python

        >>> import isaacsim.core.utils.stage as stage_utils
        >>> from isaacsim.core.prims import SingleGeometryPrim
        >>>
        >>> # create a Cube at the given path
        >>> stage_utils.get_current_stage().DefinePrim("/World/Xform", "Xform")
        >>> stage_utils.get_current_stage().DefinePrim("/World/Xform/Cube", "Cube")
        >>>
        >>> # wrap the prim as geometry prim
        >>> prim = SingleGeometryPrim("/World/Xform", collision=True)
        >>> prim
        <isaacsim.core.prims.single_geometry_prim.SingleGeometryPrim object at 0x7fe960247400>
    """

    def __init__(
        self,
        prim_path: str,
        name: str = "geometry_prim",
        position: Optional[Sequence[float]] = None,
        translation: Optional[Sequence[float]] = None,
        orientation: Optional[Sequence[float]] = None,
        scale: Optional[Sequence[float]] = None,
        visible: Optional[bool] = None,
        reset_xform_properties: bool = True,
        collision: bool = False,
        track_contact_forces: bool = False,
        prepare_contact_sensor: bool = False,
        disable_stablization: bool = True,
        contact_filter_prim_paths_expr: Optional[List[str]] = [],
    ) -> None:
        from isaacsim.core.simulation_manager import SimulationManager

        self._backend = SimulationManager.get_backend()
        self._device = SimulationManager.get_physics_sim_device()
        self._backend_utils = SimulationManager._get_backend_utils()
        if position is not None:
            position = self._backend_utils.convert(position, self._device)
            position = self._backend_utils.expand_dims(position, 0)
        if translation is not None:
            translation = self._backend_utils.convert(translation, self._device)
            translation = self._backend_utils.expand_dims(translation, 0)
        if orientation is not None:
            orientation = self._backend_utils.convert(orientation, self._device)
            orientation = self._backend_utils.expand_dims(orientation, 0)
        if scale is not None:
            scale = self._backend_utils.convert(scale, self._device)
            scale = self._backend_utils.expand_dims(scale, 0)
        if visible is not None:
            visible = self._backend_utils.create_tensor_from_list([visible], dtype="bool", device=self._device)
        collision = self._backend_utils.create_tensor_from_list([collision], dtype="bool", device=self._device)
        self._geometry_prim_view = GeometryPrim(
            prim_paths_expr=prim_path,
            name=name,
            positions=position,
            translations=translation,
            orientations=orientation,
            scales=scale,
            visibilities=visible,
            reset_xform_properties=reset_xform_properties,
            collisions=collision,
            track_contact_forces=track_contact_forces,
            prepare_contact_sensors=prepare_contact_sensor,
            disable_stablization=disable_stablization,
            contact_filter_prim_paths_expr=contact_filter_prim_paths_expr,
        )
        _SinglePrimWrapper.__init__(self, view=self._geometry_prim_view)

    @property
    def geom(self) -> UsdGeom.Gprim:
        """
        Returns:
            UsdGeom.Gprim: USD geometry object encapsulated.
        """
        return self._geometry_prim_view.geoms[0]

    def set_contact_offset(self, offset: float) -> None:
        """Set the contact offset

        Shapes whose distance is less than the sum of their contact offset values will generate contacts

        Search for *Advanced Collision Detection* in |physx_docs| for more details

        .. warning::

            The contact offset must be positive and greater than the rest offset

        Args:
            offset (float): Contact offset of a collision shape. Allowed range [maximum(0, rest_offset), 0].
                            Default value is -inf, means default is picked by simulation based on the shape extent.

        Example:

        .. code-block:: python

            >>> prim.set_contact_offset(0.02)
        """
        offset = self._backend_utils.create_tensor_from_list([offset], dtype="float32", device=self._device)
        self._geometry_prim_view.set_contact_offsets(offsets=offset)
        return

    def get_contact_offset(self) -> float:
        """Get the contact offset

        Shapes whose distance is less than the sum of their contact offset values will generate contacts

        Search for *Advanced Collision Detection* in |physx_docs| for more details

        Returns:
            float: contact offset of the collision shape. Default value is -inf, means default is picked by simulation.

        Example:

        .. code-block:: python

            >>> prim.get_contact_offset()
            -inf
        """
        return self._geometry_prim_view.get_contact_offsets()[0]

    def set_rest_offset(self, offset: float) -> None:
        """Set the rest offset

        Two shapes will come to rest at a distance equal to the sum of their rest offset values.
        If the rest offset is 0, they should converge to touching exactly

        Search for *Advanced Collision Detection* in |physx_docs| for more details

        .. warning::

            The contact offset must be positive and greater than the rest offset

        Args:
            offset (float): Rest offset of a collision shape. Allowed range [-max_float, contact_offset.
                            Default value is -inf, means default is picked by simulation. For rigid bodies its zero.

        Example:

        .. code-block:: python

            >>> prim.set_rest_offset(0.01)
        """
        offset = self._backend_utils.create_tensor_from_list([offset], dtype="float32", device=self._device)
        self._geometry_prim_view.set_rest_offsets(offsets=offset)
        return

    def get_rest_offset(self) -> float:
        """Get the rest offset

        Two shapes will come to rest at a distance equal to the sum of their rest offset values.
        If the rest offset is 0, they should converge to touching exactly

        Search for *Advanced Collision Detection* in |physx_docs| for more details

        Returns:
            float: rest offset of the collision shape.

        Example:

        .. code-block:: python

            >>> prim.get_rest_offset()
            -inf
        """
        return self._geometry_prim_view.get_rest_offsets()[0]

    def set_torsional_patch_radius(self, radius: float) -> None:
        """Set the radius of the contact patch used to apply torsional friction

        Search for *"Torsional Patch Radius"* in |physx_docs| for more details

        Args:
            radius (float): radius of the contact patch used to apply torsional friction. Allowed range [0, max_float].

        Example:

        .. code-block:: python

            >>> prim.set_torsional_patch_radius(0.1)
        """
        radius = self._backend_utils.create_tensor_from_list([radius], dtype="float32", device=self._device)
        self._geometry_prim_view.set_torsional_patch_radii(radii=radius)
        return

    def get_torsional_patch_radius(self) -> float:
        """Get the radius of the contact patch used to apply torsional friction

        Search for *"Torsional Patch Radius"* in |physx_docs| for more details

        Returns:
            float: radius of the contact patch used to apply torsional friction. Allowed range [0, max_float].

        Example:

        .. code-block:: python

            >>> prim.get_torsional_patch_radius()
            0.0
        """
        return self._geometry_prim_view.get_torsional_patch_radii()[0]

    def set_min_torsional_patch_radius(self, radius: float) -> None:
        """Set the minimum radius of the contact patch used to apply torsional friction

        Search for *"Torsional Patch Radius"* in |physx_docs| for more details

        Args:
            radius (float): minimum radius of the contact patch used to apply torsional friction. Allowed range [0, max_float].

        Example:

        .. code-block:: python

            >>> prim.set_min_torsional_patch_radius(0.05)
        """
        radius = self._backend_utils.create_tensor_from_list([radius], dtype="float32", device=self._device)
        self._geometry_prim_view.set_min_torsional_patch_radii(radii=radius)
        return

    def get_min_torsional_patch_radius(self) -> float:
        """Get the minimum radius of the contact patch used to apply torsional friction

        Search for *"Torsional Patch Radius"* in |physx_docs| for more details

        Returns:
            float: minimum radius of the contact patch used to apply torsional friction. Allowed range [0, max_float].

        Example:

        .. code-block:: python

            >>> prim.get_min_torsional_patch_radius()
            0.0
        """
        return self._geometry_prim_view.get_min_torsional_patch_radii()[0]

    def set_collision_approximation(self, approximation_type: str) -> None:
        """Set the collision approximation

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

        .. note ::

            Use Convex Decomposition or SDF (Signed-Distance-Field) tri-meshes to capture details better

        .. warning::

            Switching to Convex Decomposition or SDF (Signed-Distance-Field) will have a simulation performance
            impact due to higher computational cost

        Args:
            approximation_type (str): approximation used for collision

        Example:

        .. code-block:: python

            >>> prim.set_collision_approximation("convexDecomposition")
        """
        self._geometry_prim_view.set_collision_approximations([approximation_type])
        return

    def get_collision_approximation(self) -> str:
        """Get the collision approximation

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

        Returns:
            str: approximation used for collision

        Example:

        .. code-block:: python

            >>> prim.get_collision_approximation()
            none
        """
        return self._geometry_prim_view.get_collision_approximations()[0]

    def set_collision_enabled(self, enabled: bool) -> None:
        """Enable/disable the Collision API

        Args:
            enabled (bool): Whether to enable or disable the Collision API

        Example:

        .. code-block:: python

            >>> # disable collisions
            >>> prim.set_collision_enabled(False)
        """
        if enabled:
            self._geometry_prim_view.enable_collision()
        else:
            self._geometry_prim_view.disable_collision()
        return

    def get_collision_enabled(self) -> bool:
        """Check if the Collision API is enabled

        Returns:
            bool: True if the Collision API is enabled. Otherwise False

        Example:

        .. code-block:: python

            >>> prim.get_collision_enabled()
            True
        """
        return self._geometry_prim_view.is_collision_enabled()[0]

    def apply_physics_material(self, physics_material: "PhysicsMaterial", weaker_than_descendants: bool = False):
        """Used to apply physics material to the held prim and optionally its descendants.

        Args:
            physics_material (PhysicsMaterial): physics material to be applied to the held prim. This where you want to
                                                define friction, restitution..etc. Note: if a physics material is not
                                                defined, the defaults will be used from PhysX.
            weaker_than_descendants (bool, optional): True if the material shouldn't override the descendants
                                                      materials, otherwise False. Defaults to False.

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
            >>> prim.apply_physics_material(material)
        """
        self._geometry_prim_view.apply_physics_materials(
            physics_materials=physics_material, weaker_than_descendants=weaker_than_descendants
        )
        return

    def get_applied_physics_material(self) -> "PhysicsMaterial":
        """Return the current applied physics material in case it was applied using apply_physics_material or not.

        Returns:
            PhysicsMaterial: the current applied physics material.

        Example:

        .. code-block:: python

            >>> # given a physics material applied
            >>> prim.get_applied_physics_material()
            <isaacsim.core.api.materials.physics_material.PhysicsMaterial object at 0x7fb66c30cd30>
        """
        return self._geometry_prim_view.get_applied_physics_materials()[0]

    def get_net_contact_forces(self, dt: float = 1.0) -> Union[np.ndarray, torch.Tensor]:
        """
        If contact forces of the prims in the view are tracked, this method returns the net contact forces on prims.
        i.e., a matrix of dimension (1, 3)

        Args:
            dt (float): time step multiplier to convert the underlying impulses to forces. If the default value is used then the forces are in fact contact impulses

        Returns:
            Union[np.ndarray, torch.Tensor]: Net contact forces of the prims with shape (3).
        """
        return self._geometry_prim_view.get_net_contact_forces(dt=dt)[0]

    def get_contact_force_matrix(self, dt: float = 1.0) -> Union[np.ndarray, torch.Tensor]:
        """
        If the object is initialized with filter_paths_expr list, this method returns the contact forces between the prims
        in the view and the filter prims. i.e., a matrix of dimension (self._contact_view.num_filters, 3)
        where num_filters is the determined according to the filter_paths_expr parameter.

        Args:
            dt (float): time step multiplier to convert the underlying impulses to forces. If the default value is used then the forces are in fact contact impulses

        Returns:
            Union[np.ndarray, torch.Tensor]: Net contact forces of the prims with shape (self._geometry_prim_view._contact_view.num_filters, 3).
        """
        return self._geometry_prim_view._contact_view.get_contact_force_matrix(dt=dt)[0]

    def get_contact_force_data(self, dt: float = 1.0) -> Union[np.ndarray, torch.Tensor]:
        """
        If the object is initialized with filter_paths_expr list, this method returns the detailed contact forces between the prims
        in the view and the filter prims including the normal contact forces, normal directions, contact points, separations.
        The number of contacts per pair is determined from a static tensor of dimension (self._contact_view.num_filters) while
        the starting index of the associated contact in the above tensors is determined from another static tensor of dimension (self._contact_view.num_filters).

        Args:
            dt (float): time step multiplier to convert the underlying impulses to forces. If the default value is used then the forces are in fact contact impulses

        Returns:
            Tuple[Union[np.ndarray, torch.Tensor, wp.indexedarray], Union[np.ndarray, torch.Tensor, wp.indexedarray],
            Union[np.ndarray, torch.Tensor, wp.indexedarray], Union[np.ndarray, torch.Tensor, wp.indexedarray],
            Union[np.ndarray, torch.Tensor, wp.indexedarray], Union[np.ndarray, torch.Tensor, wp.indexedarray]]:
            A set of buffers for normal forces with shape (max_contact_count, 1), points with shape (max_contact_count, 3), normals with shape (max_contact_count, 3),
            and distances with shape (max_contact_count, 1), as well as two tensors with shape (self.num_filters)
            to indicate the starting index and the number of contact data points per pair in the aforementioned buffers.
        """
        return self._geometry_prim_view._contact_view.get_contact_force_data(dt=dt)[0]

    def get_friction_data(self, dt: float = 1.0) -> Union[np.ndarray, torch.Tensor]:
        """
        If the object is initialized with filter_paths_expr list, this method returns the detailed friction forces between the prims
        in the view and the filter prims including the tangential forces, and points.
        The number of points per pair is determined from a static tensor of dimension (self._contact_view.num_filters) while
        the starting index of the associated contact in the above tensors is determined from another static tensor of dimension (self._contact_view.num_filters).

        Args:
            dt (float): time step multiplier to convert the underlying impulses to forces. If the default value is used then the forces are in fact contact impulses

        Returns:
            Tuple[Union[np.ndarray, torch.Tensor, wp.indexedarray], Union[np.ndarray, torch.Tensor, wp.indexedarray],
            Union[np.ndarray, torch.Tensor, wp.indexedarray], Union[np.ndarray, torch.Tensor, wp.indexedarray]]:
            A set of buffers for normal forces with shape (max_contact_count, 1), points with shape (max_contact_count, 3),
            as well as two tensors with shape (self.num_filters) to indicate the starting index and the number of
            contact data points per pair in the aforementioned buffers.
        """
        return self._geometry_prim_view._contact_view.get_friction_data(dt=dt)[0]
