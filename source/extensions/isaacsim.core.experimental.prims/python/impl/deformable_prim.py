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

import weakref
from typing import Literal

import carb
import isaacsim.core.experimental.utils.backend as backend_utils
import isaacsim.core.experimental.utils.ops as ops_utils
import isaacsim.core.experimental.utils.prim as prim_utils
import isaacsim.core.experimental.utils.stage as stage_utils
import numpy as np
import omni.physics.tensors
import warp as wp
from isaacsim.core.simulation_manager import SimulationManager
from omni.physx import get_physx_cooking_interface
from omni.physx.bindings import _physx as physx_bindings
from omni.physx.scripts import deformableUtils
from pxr import Usd, UsdGeom, UsdShade

from .prim import _MSG_PHYSICS_TENSOR_ENTITY_NOT_VALID, _MSG_PRIM_NOT_VALID
from .xform_prim import XformPrim


def _check_or_apply_deformable_schema(
    stage: Usd.Stage, path: str, deformable_type: Literal["surface", "volume"] | None
) -> tuple[Literal["wrap", "create"], Literal["surface", "volume"]]:
    prim = stage.GetPrimAtPath(path)
    # check for conflicting APIs
    if physx_bindings.hasconflictingapis_DeformableBodyAPI(prim, False):  # don't check itself (deformable schema)
        raise ValueError(
            f"The prim at path {path} has conflicting APIs with the Deformable schema: {prim.GetAppliedSchemas()}"
        )
    # Mesh (surface)
    if prim.IsA(UsdGeom.Mesh):
        assert (
            deformable_type is None or deformable_type == "surface"
        ), f"The prim at path {path} is a Mesh (surface), but the deformable type is set to '{deformable_type}'"
        # validate mesh
        mesh = UsdGeom.Mesh(prim)
        face_vertex_counts = mesh.GetFaceVertexCountsAttr().Get()
        if not face_vertex_counts or not len(face_vertex_counts):
            raise ValueError(f"The Mesh prim at path {path} is empty or has non-triangular faces")
        elif not all(count == 3 for count in face_vertex_counts):
            raise ValueError(
                f"The Mesh prim at path {path} has non-triangular faces. "
                "Nest mesh under a Xform and create surface deformable body on it"
            )
        # check or apply schemas
        if prim_utils.has_api(prim, ["OmniPhysicsDeformableBodyAPI", "OmniPhysicsSurfaceDeformableSimAPI"]):
            return "wrap", "surface"
        if deformableUtils.set_physics_surface_deformable_body(stage, path):
            return "create", "surface"
        raise ValueError(f"Failed to setup a surface deformable body for the Mesh prim at path {path}")
    # TetMesh (volume)
    elif prim.IsA(UsdGeom.TetMesh):
        assert (
            deformable_type is None or deformable_type == "volume"
        ), f"The prim at path {path} is a TetMesh (volume), but the deformable type is set to '{deformable_type}'"
        # check or apply schemas
        if prim_utils.has_api(prim, ["OmniPhysicsDeformableBodyAPI", "OmniPhysicsVolumeDeformableSimAPI"]):
            return "wrap", "volume"
        if deformableUtils.set_physics_volume_deformable_body(stage, path):
            return "create", "volume"
        raise ValueError(f"Failed to setup a volume deformable body for the TetMesh prim at path {path}")
    # Xform with nested mesh (surface or volume)
    elif prim.IsA(UsdGeom.Imageable) and not prim.IsA(UsdGeom.Gprim):
        # check for deformable body
        if prim_utils.has_api(
            prim, ["OmniPhysicsBodyAPI", "OmniPhysicsDeformableBodyAPI", "PhysxAutoDeformableBodyAPI"]
        ):
            # - surface
            predicate = lambda prim, path: prim_utils.has_api(prim, "OmniPhysicsSurfaceDeformableSimAPI")
            if prim_utils.get_first_matching_child_prim(prim, predicate=predicate) is not None:
                assert (
                    deformable_type is None or deformable_type == "surface"
                ), f"The prim at path {path} is a surface, but the deformable type is set to '{deformable_type}'"
                get_physx_cooking_interface().cook_auto_deformable_body(path)
                return "wrap", "surface"
            # - volume
            predicate = lambda prim, path: prim_utils.has_api(prim, "OmniPhysicsVolumeDeformableSimAPI")
            if prim_utils.get_first_matching_child_prim(prim, predicate=predicate) is not None:
                assert (
                    deformable_type is None or deformable_type == "volume"
                ), f"The prim at path {path} is a volume, but the deformable type is set to '{deformable_type}'"
                get_physx_cooking_interface().cook_auto_deformable_body(path)
                return "wrap", "volume"
            raise ValueError(
                f"The prim at path {path} has the Deformable schema applied, "
                "but its children prims do not have the required surface/volume deformable schema"
            )
        # create deformable body
        # - surface
        if deformable_type == "surface":
            cooking_mesh = prim_utils.get_first_matching_child_prim(
                prim, predicate=lambda prim, path: prim.IsA(UsdGeom.Mesh)
            )
            if cooking_mesh is None:
                raise ValueError(f"Failed to find a (cooking) child Mesh for the prim at path {path}")
            if deformableUtils.create_auto_surface_deformable_hierarchy(
                stage,
                path,
                simulation_mesh_path=f"{path}/simulation_mesh",
                cooking_src_mesh_path=prim_utils.get_prim_path(cooking_mesh),
                cooking_src_simplification_enabled=False,
            ):
                get_physx_cooking_interface().cook_auto_deformable_body(path)
                return "create", "surface"
            raise ValueError(f"Failed to setup a surface deformable body for the prim at path {path}")
        # - volume
        elif deformable_type == "volume":
            cooking_mesh = prim_utils.get_first_matching_child_prim(
                prim, predicate=lambda prim, path: prim.IsA(UsdGeom.Mesh)
            )
            if cooking_mesh is None:
                raise ValueError(f"Failed to find a (cooking) child Mesh for the prim at path {path}")
            if deformableUtils.create_auto_volume_deformable_hierarchy(
                stage,
                path,
                simulation_tetmesh_path=f"{path}/simulation_mesh",
                collision_tetmesh_path=f"{path}/collision_mesh",
                cooking_src_mesh_path=prim_utils.get_prim_path(cooking_mesh),
                simulation_hex_mesh_enabled=True,
                cooking_src_simplification_enabled=False,
            ):
                get_physx_cooking_interface().cook_auto_deformable_body(path)
                return "create", "volume"
            raise ValueError(f"Failed to setup a volume deformable body for the prim at path {path}")
        else:
            raise ValueError(
                f"The prim at path {path} is a UsdGeom.Imageable, but the 'deformable_type' argument is not set. "
                "Set the 'deformable_type' argument to 'surface' or 'volume' in order to apply the Deformable schema."
            )
    else:
        raise ValueError(f"Failed to check or apply the Deformable schema for the prim at path {path}")


class DeformablePrim(XformPrim):
    """High level wrapper for manipulating prims (that have Deformable Schema applied) and their attributes.

    .. warning::

        The deformable prims require the Deformable feature (beta) to be enabled.
        Deformable feature (beta) can be enabled in *apps/.kit* experience files by setting
        ``physics.enableDeformableBeta = true`` under the ``[settings.persistent]`` section.

    .. warning::

        The current ``omni.physics.tensors`` implementation does not support ``list[str]``
        as input for deformable body paths. This limitation will be fixed in the future releases.
        An error message will be logged if a list of paths is provided.

    This class is a wrapper over one or more USD prims in the stage to provide
    high-level functionality for manipulating deformable body properties, and other attributes.
    The prims are specified using paths that can include regular expressions for matching multiple prims.

    .. note::

        If the prims do not already have the Deformable Schema applied to them, it will be applied.

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
        deformable_type: Type of deformable body.
            If not defined, the type will be inferred from the prims.

    Raises:
        RuntimeError: If the Deformable feature (beta) is disabled.
        ValueError: If no prims are found matching the specified path(s).
        AssertionError: If both positions and translations are specified.

    Example:

    .. code-block:: python

        >>> import numpy as np
        >>> import omni.timeline
        >>> from isaacsim.core.experimental.prims import DeformablePrim
        >>>
        >>> # given a USD stage with the tetrahedral mesh prims: /World/prim_0, /World/prim_1, and /World/prim_2
        >>> # - create wrapper over single prim (volume deformable body)
        >>> prim = DeformablePrim("/World/prim_0", deformable_type="volume")  # doctest: +NO_CHECK
        >>> # - create wrapper over multiple prims using regex (volume deformable body)
        >>> prims = DeformablePrim("/World/prim_.*", deformable_type="volume")  # doctest: +NO_CHECK
        >>>
        >>> # play the simulation so that the Physics tensor entity becomes valid
        >>> omni.timeline.get_timeline_interface().play()
    """

    def __init__(
        self,
        paths: str | list[str],
        *,
        # Prim
        resolve_paths: bool = False,
        # XformPrim
        positions: list | np.ndarray | wp.array | None = None,
        translations: list | np.ndarray | wp.array | None = None,
        orientations: list | np.ndarray | wp.array | None = None,
        scales: list | np.ndarray | wp.array | None = None,
        reset_xform_op_properties: bool = False,
        # DeformablePrim
        deformable_type: Literal["surface", "volume"] | None = None,
    ) -> None:
        # check for deformable feature (beta)
        setting_name = physx_bindings.SETTING_ENABLE_DEFORMABLE_BETA
        enabled = carb.settings.get_settings().get(setting_name)
        if not enabled:
            setting_name = (setting_name[1:] if setting_name.startswith("/") else setting_name).replace("/", ".")
            raise RuntimeError(
                "Deformable bodies require Deformable feature (beta) to be enabled. "
                f"Enable it in .kit experience settings ('{setting_name} = true') to use them."
            )
        # get  prims
        stage = stage_utils.get_current_stage(backend="usd")
        existent_paths, nonexistent_paths = self.resolve_paths(paths)
        assert (
            not nonexistent_paths
        ), f"Specified paths must correspond to existing prims: {', '.join(nonexistent_paths)}"
        # check for deformable compatibility
        deformable_types = set()
        for path in existent_paths:
            deformable_types.add(_check_or_apply_deformable_schema(stage, path, deformable_type)[1])
        assert len(deformable_types) == 1, f"Multiple deformable types are not supported: {deformable_types}"
        # define properties
        self._deformable_type = deformable_types.pop()
        # - physics tensor entity properties
        self._num_nodes_per_element = None
        # -- simulation mesh
        self._simulation_mesh_paths = None
        self._simulation_nodes_per_body = None
        self._simulation_elements_per_body = None
        # -- collision mesh
        self._collision_mesh_paths = None
        self._collision_nodes_per_body = None
        self._collision_elements_per_body = None
        # -- rest mesh
        self._rest_nodes_per_body = None
        self._rest_elements_per_body = None
        # -- deformable body physics view
        self._physics_deformable_body_view_paths = paths  # TODO: remove by `self.paths` when `list[str]` is supported
        self._physics_deformable_body_view = None
        self._physics_tensor_entity_initialized = False
        # initialize base class
        super().__init__(
            existent_paths,
            resolve_paths=resolve_paths,
            positions=positions,
            translations=translations,
            orientations=orientations,
            scales=scales,
            reset_xform_op_properties=reset_xform_op_properties,
        )
        # setup subscriptions
        self._subscription_to_timeline_stop_event = (
            SimulationManager._timeline.get_timeline_event_stream().create_subscription_to_pop_by_type(
                int(omni.timeline.TimelineEventType.STOP),
                lambda event, obj=weakref.proxy(self): obj._on_timeline_stop(event),
            )
        )
        # setup physics-related configuration if simulation is running
        if SimulationManager._physics_sim_view__warp is not None:
            SimulationManager._physx_sim_interface.flush_changes()
            self._on_physics_ready(None)

    def __del__(self):
        super().__del__()
        self._subscription_to_timeline_stop_event = None
        if hasattr(self, "_physics_deformable_body_view"):
            del self._physics_deformable_body_view

    """
    Properties.
    """

    @property
    def deformable_type(self) -> Literal["surface", "volume"]:
        """Type of deformable body.

        Backends: :guilabel:`usd`, :guilabel:`tensor`. |br|
        Deformable type: :guilabel:`surface`, :guilabel:`volume`.

        Returns:
            Type of deformable body.

        Example:

        .. code-block:: python

            >>> prims.deformable_type
            'volume'
        """
        return self._deformable_type

    @property
    def simulation_mesh_paths(self) -> list[str]:
        """Simulation mesh paths of the prims.

        Backends: :guilabel:`usd`, :guilabel:`tensor`. |br|
        Deformable type: :guilabel:`surface`, :guilabel:`volume`.

        Returns:
            Ordered list of simulation mesh paths.

        Example:

        .. code-block:: python

            >>> prims.simulation_mesh_paths
            ['/World/prim_0', '/World/prim_1', '/World/prim_2']
        """
        if self._simulation_mesh_paths is None:
            self._query_deformable_properties()
        return self._simulation_mesh_paths

    @property
    def collision_mesh_paths(self) -> list[str]:
        """Collision mesh paths of the prims.

        Backends: :guilabel:`usd`, :guilabel:`tensor`. |br|
        Deformable type: :guilabel:`surface`, :guilabel:`volume`.

        Returns:
            Ordered list of collision mesh paths.

        Example:

        .. code-block:: python

            >>> prims.collision_mesh_paths
            ['/World/prim_0', '/World/prim_1', '/World/prim_2']
        """
        if self._collision_mesh_paths is None:
            self._query_deformable_properties()
        return self._collision_mesh_paths

    @property
    def num_nodes_per_element(self) -> int:
        """Number of nodes (mesh points) per element (triangular face for surface, tetrahedron for volume).

        Backends: :guilabel:`usd`, :guilabel:`tensor`. |br|
        Deformable type: :guilabel:`surface`, :guilabel:`volume`.

        The number of nodes per element is 3 (triangle) for surface and 4 (tetrahedron) for volume.

        Returns:
            Number of nodes per element.

        Example:

        .. code-block:: python

            >>> prims.num_nodes_per_element
            4
        """
        if self._num_nodes_per_element is None:
            self._query_deformable_properties()
        return self._num_nodes_per_element

    @property
    def num_nodes_per_body(self) -> tuple[int, int, int]:
        """Number of nodes (mesh points) per body (mesh prim).

        Backends: :guilabel:`usd`, :guilabel:`tensor`. |br|
        Deformable type: :guilabel:`surface`, :guilabel:`volume`.

        Returns:
            Tuple of three elements. 1) Number of nodes per body (simulation mesh).
            2) Number of nodes per body (collision mesh).
            3) Number of nodes per body (rest mesh).

        Example:

        .. code-block:: python

            >>> prims.num_nodes_per_body
            (4, 4, 4)
        """
        if self._simulation_nodes_per_body is None:
            self._query_deformable_properties()
        return self._simulation_nodes_per_body, self._collision_nodes_per_body, self._rest_nodes_per_body

    @property
    def num_elements_per_body(self) -> tuple[int, int, int]:
        """Number of elements (triangular faces for surface, tetrahedrons for volume) per body (mesh prim).

        Backends: :guilabel:`usd`, :guilabel:`tensor`. |br|
        Deformable type: :guilabel:`surface`, :guilabel:`volume`.

        .. note::

            In the current implementation, the rest mesh is limited to the same topology as the simulation mesh.
            This means that the number of elements per body is the same for the simulation and rest meshes.

        Returns:
            Tuple of three elements. 1) Number of elements per body (simulation mesh).
            2) Number of elements per body (collision mesh).
            3) Number of elements per body (rest mesh).

        Example:

        .. code-block:: python

            >>> prims.num_elements_per_body
            (1, 1, 1)
        """
        if self._simulation_elements_per_body is None:
            self._query_deformable_properties()
        return self._simulation_elements_per_body, self._collision_elements_per_body, self._rest_elements_per_body

    """
    Methods.
    """

    def is_physics_tensor_entity_valid(self) -> bool:
        """Check if the physics tensor entity is valid.

        Returns:
            Whether the physics tensor entity is valid.
        """
        return SimulationManager._physics_sim_view__warp is not None and self._physics_deformable_body_view is not None

    def get_element_indices(
        self, *, indices: list | np.ndarray | wp.array | None = None
    ) -> tuple[wp.array, wp.array, wp.array]:
        """Get the element (triangular faces for surface, tetrahedrons for volume) indices
        of the simulation, collision and rest meshes of the prims.

        Backends: :guilabel:`tensor`. |br|
        Deformable type: :guilabel:`surface`, :guilabel:`volume`.

        Args:
            indices: Indices of prims to process (shape ``(N,)``). If not defined, all wrapped prims are processed.

        Returns:
            Three-elements tuple.
            1) The simulation mesh's element indices (shape ``(N, num_elements_per_body, num_nodes_per_element)``).
            2) The collision mesh's element indices (shape ``(N, num_elements_per_body, num_nodes_per_element)``).
            3) The rest mesh's element indices (shape ``(N, num_elements_per_body, num_nodes_per_element)``).

        Raises:
            AssertionError: Wrapped prims are not valid.
            AssertionError: Physics tensor entity is not valid.

        Example:

        .. code-block:: python

            >>> simulation_indices, collision_indices, rest_indices = prims.get_element_indices()
            >>> print(simulation_indices)
            [[[0 1 2 3]]
             [[0 1 2 3]]
             [[0 1 2 3]]]
            >>> print(collision_indices)
            [[[0 1 2 3]]
             [[0 1 2 3]]
             [[0 1 2 3]]]
            >>> print(rest_indices)
            [[[0 1 2 3]]
             [[0 1 2 3]]
             [[0 1 2 3]]]
        """
        assert self.valid, _MSG_PRIM_NOT_VALID
        assert self.is_physics_tensor_entity_valid(), _MSG_PHYSICS_TENSOR_ENTITY_NOT_VALID
        # Tensor API
        simulation_indices = self._physics_deformable_body_view.get_simulation_element_indices()  # shape: (N, EpB, NpE)
        collision_indices = self._physics_deformable_body_view.get_collision_element_indices()  # shape: (N, EpB, NpE)
        rest_indices = self._physics_deformable_body_view.get_rest_element_indices()  # shape: (N, EpB, NpE)
        indices = ops_utils.resolve_indices(indices, count=len(self), device=simulation_indices.device)
        return (
            simulation_indices[indices].contiguous().to(self._device),
            collision_indices[indices].contiguous().to(self._device),
            rest_indices[indices].contiguous().to(self._device),
        )

    def get_nodal_positions(
        self, *, indices: list | np.ndarray | wp.array | None = None
    ) -> tuple[wp.array, wp.array, wp.array]:
        """Get the nodal (mesh points) positions of the simulation, collision and rest meshes of the prims.

        Backends: :guilabel:`tensor`. |br|
        Deformable type: :guilabel:`surface`, :guilabel:`volume`.

        Args:
            indices: Indices of prims to process (shape ``(N,)``). If not defined, all wrapped prims are processed.

        Returns:
            Three-elements tuple.
            1) The simulation mesh's nodal positions (shape ``(N, num_nodes_per_body, 3)``).
            2) The collision mesh's nodal positions (shape ``(N, num_nodes_per_body, 3)``).
            3) The rest mesh's nodal positions (shape ``(N, num_nodes_per_body, 3)``).

        Raises:
            AssertionError: Wrapped prims are not valid.
            AssertionError: Physics tensor entity is not valid.

        Example:

        .. code-block:: python

            >>> # get the nodal positions of all prims
            >>> simulation_positions, collision_positions, rest_positions = prims.get_nodal_positions()
            >>> simulation_positions.shape, collision_positions.shape, rest_positions.shape
            ((3, 4, 3), (3, 4, 3), (3, 4, 3))
            >>>
            >>> # get the nodal positions of the first and last prims
            >>> simulation_positions, collision_positions, rest_positions = prims.get_nodal_positions(indices=[0, 2])
            >>> simulation_positions.shape, collision_positions.shape, rest_positions.shape
            ((2, 4, 3), (2, 4, 3), (2, 4, 3))
        """
        assert self.valid, _MSG_PRIM_NOT_VALID
        assert self.is_physics_tensor_entity_valid(), _MSG_PHYSICS_TENSOR_ENTITY_NOT_VALID
        # Tensor API
        simulation_positions = self._physics_deformable_body_view.get_simulation_nodal_positions()  # shape: (N, NpB, 3)
        collision_positions = self._physics_deformable_body_view.get_collision_nodal_positions()  # shape: (N, NpB, 3)
        rest_positions = self._physics_deformable_body_view.get_rest_nodal_positions()  # shape: (N, NpB, 3)
        indices = ops_utils.resolve_indices(indices, count=len(self), device=simulation_positions.device)
        return (
            simulation_positions[indices].contiguous().to(self._device),
            collision_positions[indices].contiguous().to(self._device),
            rest_positions[indices].contiguous().to(self._device),
        )

    def set_nodal_positions(
        self,
        positions: list | np.ndarray | wp.array | None = None,
        *,
        indices: list | np.ndarray | wp.array | None = None,
    ) -> None:
        """Set the nodal (mesh points) positions of the simulation mesh of the prims.

        Backends: :guilabel:`tensor`. |br|
        Deformable type: :guilabel:`surface`, :guilabel:`volume`.

        Args:
            positions: Nodal positions (shape ``(N, num_nodes_per_body, 3)``).
                If the input shape is smaller than expected, data will be broadcasted (following NumPy broadcast rules).
            indices: Indices of prims to process (shape ``(N,)``). If not defined, all wrapped prims are processed.

        Raises:
            AssertionError: Wrapped prims are not valid.
            AssertionError: Physics tensor entity is not valid.

        Example:

        .. code-block:: python

            >>> # set random nodal positions for all prims
            >>> positions = np.random.uniform(low=-1, high=1, size=(3, 4, 3))
            >>> prims.set_nodal_positions(positions)
        """
        assert self.valid, _MSG_PRIM_NOT_VALID
        assert self.is_physics_tensor_entity_valid(), _MSG_PHYSICS_TENSOR_ENTITY_NOT_VALID
        # Tensor API
        data = self._physics_deformable_body_view.get_simulation_nodal_positions()  # shape: (N, NpB, 3)
        indices = ops_utils.resolve_indices(indices, count=len(self), device=data.device)
        positions = ops_utils.broadcast_to(
            positions,
            shape=(indices.shape[0], self.num_nodes_per_body[0], 3),
            dtype=wp.float32,
            device=data.device,
        )
        wp.copy(data[indices], positions)
        self._physics_deformable_body_view.set_simulation_nodal_positions(data, indices)

    def get_nodal_velocities(self, *, indices: list | np.ndarray | wp.array | None = None) -> wp.array:
        """Get the nodal (mesh points) velocities of the simulation mesh of the prims.

        Backends: :guilabel:`tensor`. |br|
        Deformable type: :guilabel:`surface`, :guilabel:`volume`.

        Args:
            indices: Indices of prims to process (shape ``(N,)``). If not defined, all wrapped prims are processed.

        Returns:
            The simulation mesh's nodal velocities (shape ``(N, num_nodes_per_body, 3)``).

        Raises:
            AssertionError: Wrapped prims are not valid.
            AssertionError: Physics tensor entity is not valid.

        Example:

        .. code-block:: python

            >>> # get the nodal velocities of all prims
            >>> simulation_velocities = prims.get_nodal_velocities()
            >>> simulation_velocities.shape
            (3, 4, 3)
            >>>
            >>> # get the nodal velocities of the first and last prims
            >>> simulation_velocities = prims.get_nodal_velocities(indices=[0, 2])
            >>> simulation_velocities.shape
            (2, 4, 3)
        """
        assert self.valid, _MSG_PRIM_NOT_VALID
        assert self.is_physics_tensor_entity_valid(), _MSG_PHYSICS_TENSOR_ENTITY_NOT_VALID
        # Tensor API
        data = self._physics_deformable_body_view.get_simulation_nodal_velocities()  # shape: (N, NpB, 3)
        indices = ops_utils.resolve_indices(indices, count=len(self), device=data.device)
        return data[indices].contiguous().to(self._device)

    def set_nodal_velocities(
        self,
        velocities: list | np.ndarray | wp.array | None = None,
        *,
        indices: list | np.ndarray | wp.array | None = None,
    ) -> None:
        """Set the nodal (mesh points) velocities of the simulation mesh of the prims.

        Backends: :guilabel:`tensor`. |br|
        Deformable type: :guilabel:`surface`, :guilabel:`volume`.

        Args:
            velocities: Nodal velocities (shape ``(N, num_nodes_per_body, 3)``).
                If the input shape is smaller than expected, data will be broadcasted (following NumPy broadcast rules).
            indices: Indices of prims to process (shape ``(N,)``). If not defined, all wrapped prims are processed.

        Raises:
            AssertionError: Wrapped prims are not valid.
            AssertionError: Physics tensor entity is not valid.

        Example:

        .. code-block:: python

            >>> # set random nodal velocities for all prims
            >>> velocities = np.random.uniform(low=-1, high=1, size=(3, 4, 3))
            >>> prims.set_nodal_velocities(velocities)
        """
        assert self.valid, _MSG_PRIM_NOT_VALID
        assert self.is_physics_tensor_entity_valid(), _MSG_PHYSICS_TENSOR_ENTITY_NOT_VALID
        # Tensor API
        data = self._physics_deformable_body_view.get_simulation_nodal_velocities()  # shape: (N, NpB, 3)
        indices = ops_utils.resolve_indices(indices, count=len(self), device=data.device)
        velocities = ops_utils.broadcast_to(
            velocities,
            shape=(indices.shape[0], self.num_nodes_per_body[0], 3),
            dtype=wp.float32,
            device=data.device,
        )
        wp.copy(data[indices], velocities)
        self._physics_deformable_body_view.set_simulation_nodal_velocities(data, indices)

    def get_nodal_kinematic_position_targets(
        self, *, indices: list | np.ndarray | wp.array | None = None
    ) -> tuple[wp.array, wp.array]:
        """Get the nodal (mesh points) kinematic position targets of the simulation mesh of the prims.

        Backends: :guilabel:`tensor`. |br|
        Deformable type: :guilabel:`volume`.

        .. note::

            This method is only supported for volume deformable bodies.

        Args:
            indices: Indices of prims to process (shape ``(N,)``). If not defined, all wrapped prims are processed.

        Returns:
            Two-elements tuple.
            1) The simulation mesh's nodal kinematic position targets (shape ``(N, num_nodes_per_body, 3)``).
            2) Boolean flags indicating whether the kinematic (true) or dynamic (false) control
            is enabled (shape ``(N, num_nodes_per_body, 1)``).

        Raises:
            AssertionError: When the deformable body is not a volume.
            AssertionError: Wrapped prims are not valid.
            AssertionError: Physics tensor entity is not valid.

        Example:

        .. code-block:: python

            >>> # get the nodal kinematic position targets of all prims
            >>> positions, enabled = prims.get_nodal_kinematic_position_targets()
            >>> positions.shape, enabled.shape
            ((3, 4, 3), (3, 4, 1))
            >>>
            >>> # get the nodal kinematic position targets of the first and last prims
            >>> positions, enabled = prims.get_nodal_kinematic_position_targets(indices=[0, 2])
            >>> positions.shape, enabled.shape
            ((2, 4, 3), (2, 4, 1))
        """
        assert self.deformable_type == "volume", "Nodal kinematic target is only supported for volume deformable bodies"
        assert self.valid, _MSG_PRIM_NOT_VALID
        assert self.is_physics_tensor_entity_valid(), _MSG_PHYSICS_TENSOR_ENTITY_NOT_VALID
        # Tensor API
        data = self._physics_deformable_body_view.get_simulation_nodal_kinematic_targets()  # shape: (N, NpB, 4)
        indices = ops_utils.resolve_indices(indices, count=len(self), device=data.device)
        positions = data[indices, :, :3].contiguous().to(self._device)
        disabled = data[indices, :, 3:].contiguous().to(self._device)
        enabled = np.logical_not(disabled.numpy()).astype(np.bool_)  # negate values
        return positions, ops_utils.place(enabled, device=data.device)

    def set_nodal_kinematic_position_targets(
        self,
        positions: list | np.ndarray | wp.array | None = None,
        enabled: list | np.ndarray | wp.array | None = None,
        *,
        indices: list | np.ndarray | wp.array | None = None,
    ) -> None:
        """Set the nodal (mesh points) kinematic position targets of the simulation mesh of the prims.

        Backends: :guilabel:`tensor`. |br|
        Deformable type: :guilabel:`volume`.

        .. note::

            This method is only supported for volume deformable bodies.

        Args:
            positions: Nodal positions (shape ``(N, num_nodes_per_body, 3)``).
                If the input shape is smaller than expected, data will be broadcasted (following NumPy broadcast rules).
            enabled: Boolean flags to enable the kinematic (true) or dynamic (false) control (shape ``(N, num_nodes_per_body, 1)``).
            indices: Indices of prims to process (shape ``(N,)``). If not defined, all wrapped prims are processed.

        Raises:
            AssertionError: When the deformable body is not a volume.
            AssertionError: Wrapped prims are not valid.
            AssertionError: Physics tensor entity is not valid.

        Example:

        .. code-block:: python

            >>> # set random nodal kinematic position targets for the first and last prims
            >>> positions = np.random.uniform(low=-1, high=1, size=(2, 4, 3))
            >>> prims.set_nodal_kinematic_position_targets(positions, enabled=[True], indices=[0, 2])
        """
        assert self.deformable_type == "volume", "Nodal kinematic target is only supported for volume deformable bodies"
        assert self.valid, _MSG_PRIM_NOT_VALID
        assert self.is_physics_tensor_entity_valid(), _MSG_PHYSICS_TENSOR_ENTITY_NOT_VALID
        # Tensor API
        data = self._physics_deformable_body_view.get_simulation_nodal_kinematic_targets()  # shape: (N, NpB, 3)
        indices = ops_utils.resolve_indices(indices, count=len(self), device=data.device)
        if positions is not None:
            positions = ops_utils.broadcast_to(
                positions, shape=(indices.shape[0], self.num_nodes_per_body[0], 3), dtype=wp.float32, device=data.device
            )
            wp.copy(data[indices, :, :3], positions)
        if enabled is not None:
            enabled = ops_utils.place(enabled, dtype=wp.uint8, device=data.device)
            disabled = np.logical_not(enabled.numpy()).astype(np.uint8)  # negate values
            disabled = ops_utils.broadcast_to(
                disabled, shape=(indices.shape[0], self.num_nodes_per_body[0], 1), dtype=wp.float32, device=data.device
            )
            wp.copy(data[indices, :, 3:], disabled)
        self._physics_deformable_body_view.set_simulation_nodal_kinematic_targets(data, indices)

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
            materials: Physics materials to be applied to the prims (shape ``(N,)``).
                If the input shape is smaller than expected, data will be broadcasted (following NumPy broadcast rules).
            weaker_than_descendants: Boolean flags to indicate whether descendant materials should be overridden (shape ``(N, 1)``).
                If the input shape is smaller than expected, data will be broadcasted (following NumPy broadcast rules).
            indices: Indices of prims to process (shape ``(N,)``). If not defined, all wrapped prims are processed.

        Raises:
            AssertionError: Wrapped prims are not valid.

        Example:

        .. code-block:: python

            >>> from isaacsim.core.experimental.materials import VolumeDeformableMaterial
            >>>
            >>> # create a volume deformable physics material
            >>> material = VolumeDeformableMaterial(
            ...     "/World/physics_material/deformable_material",
            ...     static_frictions=[1.1],
            ...     dynamic_frictions=[0.4],
            ...     poissons_ratios=[0.1],
            ...     youngs_moduli=[1000000.0],
            ... )
            >>>
            >>> # apply the material to all prims
            >>> prims.apply_physics_materials(material)  # or [material]  # doctest: +SKIP
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
            material_binding_api = DeformablePrim.ensure_api([self.prims[index]], UsdShade.MaterialBindingAPI)[0]
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
            >>> physics_material.paths[0]  # doctest: +SKIP
            '/World/physics_material/deformable_material'
        """
        assert self.valid, _MSG_PRIM_NOT_VALID
        # USD API
        from isaacsim.core.experimental.materials import PhysicsMaterial  # defer imports to avoid circular dependencies

        indices = ops_utils.resolve_indices(indices, count=len(self), device="cpu")
        materials = []
        for index in indices.numpy():
            material_binding_api = DeformablePrim.ensure_api([self.prims[index]], UsdShade.MaterialBindingAPI)[0]
            material_path = str(material_binding_api.GetDirectBinding(materialPurpose="physics").GetMaterialPath())
            material = None
            if material_path:
                material = PhysicsMaterial.fetch_instances([material_path])[0]
                if material is None:
                    carb.log_warn(f"Unsupported physics material ({material_path}): {self.paths[index]}")
            materials.append(material)
        return materials

    """
    Internal methods.
    """

    def _check_for_tensor_backend(self, backend: str, *, fallback_backend: str = "usd") -> str:
        """Check if the tensor backend is valid."""
        if backend == "tensor" and not self.is_physics_tensor_entity_valid():
            if backend_utils.is_backend_set():
                if backend_utils.should_raise_on_fallback():
                    raise RuntimeError(
                        f"Physics tensor entity is not valid. Fallback set to '{fallback_backend}' backend."
                    )
                carb.log_warn(
                    f"Physics tensor entity is not valid for use with 'tensor' backend. Falling back to '{fallback_backend}' backend."
                )
            return fallback_backend
        return backend

    def _query_deformable_properties(self) -> None:
        """Query deformable properties."""
        stage = stage_utils.get_current_stage(backend="usd")

        simulation_mesh_paths, collision_mesh_paths = [], []
        simulation_nodes_per_bodies, collision_nodes_per_bodies, rest_nodes_per_bodies = [], [], []
        simulation_elements_per_bodies, collision_elements_per_bodies, rest_elements_per_bodies = [], [], []

        # query data
        for prim in self.prims:
            # - surface
            if prim.IsA(UsdGeom.Mesh):
                mesh = UsdGeom.Mesh(prim)
                # -- paths
                path = prim_utils.get_prim_path(prim)
                simulation_mesh_paths.append(path)
                collision_mesh_paths.append(path)
                # -- nodes per body
                num_points = len(mesh.GetPointsAttr().Get())
                simulation_nodes_per_bodies.append(num_points)
                collision_nodes_per_bodies.append(num_points)
                rest_nodes_per_bodies.append(num_points)
                # -- elements per body
                num_faces = len(mesh.GetFaceVertexCountsAttr().Get())
                simulation_elements_per_bodies.append(num_faces)
                collision_elements_per_bodies.append(num_faces)
                rest_elements_per_bodies.append(num_faces)
            # - volume
            elif prim.IsA(UsdGeom.TetMesh):
                mesh = UsdGeom.TetMesh(prim)
                # -- paths
                path = prim_utils.get_prim_path(prim)
                simulation_mesh_paths.append(path)
                collision_mesh_paths.append(path)
                # -- nodes per body
                num_points = len(mesh.GetPointsAttr().Get())
                simulation_nodes_per_bodies.append(num_points)
                collision_nodes_per_bodies.append(num_points)
                rest_nodes_per_bodies.append(num_points)
                # -- elements per body
                num_tetrahedra = len(mesh.GetTetVertexIndicesAttr().Get())
                simulation_elements_per_bodies.append(num_tetrahedra)
                collision_elements_per_bodies.append(num_tetrahedra)
                rest_elements_per_bodies.append(num_tetrahedra)
            # - auto-surface/volume
            elif prim.IsA(UsdGeom.Imageable) and not prim.IsA(UsdGeom.Gprim):
                # - auto-surface
                if self._deformable_type == "surface":
                    predicate = lambda prim, path: prim_utils.has_api(prim, "OmniPhysicsSurfaceDeformableSimAPI")
                    simulation_mesh_prim = prim_utils.get_first_matching_child_prim(prim, predicate=predicate)
                    if simulation_mesh_prim is None or not simulation_mesh_prim.IsA(UsdGeom.Mesh):
                        raise RuntimeError(f"Unable to find simulation mesh for auto-surface deformable body: {prim}")
                    mesh = UsdGeom.Mesh(simulation_mesh_prim)
                    if mesh.GetPointsAttr().Get() is None:
                        raise RuntimeError(
                            "An app update step is required between creating the deformable body and "
                            f"querying its properties for the simulation mesh to be populated: {mesh}"
                        )
                    # -- paths
                    path = prim_utils.get_prim_path(simulation_mesh_prim)
                    simulation_mesh_paths.append(path)
                    collision_mesh_paths.append(path)
                    # -- nodes per body
                    num_points = len(mesh.GetPointsAttr().Get())
                    simulation_nodes_per_bodies.append(num_points)
                    collision_nodes_per_bodies.append(num_points)
                    rest_nodes_per_bodies.append(num_points)
                    # -- elements per body
                    num_faces = len(mesh.GetFaceVertexCountsAttr().Get())
                    simulation_elements_per_bodies.append(num_faces)
                    collision_elements_per_bodies.append(num_faces)
                    rest_elements_per_bodies.append(num_faces)
                # - auto-volume
                elif self._deformable_type == "volume":
                    predicate = lambda prim, path: prim_utils.has_api(prim, "OmniPhysicsVolumeDeformableSimAPI")
                    simulation_mesh_prim = prim_utils.get_first_matching_child_prim(prim, predicate=predicate)
                    if simulation_mesh_prim is None or not simulation_mesh_prim.IsA(UsdGeom.TetMesh):
                        raise RuntimeError(f"Unable to find simulation mesh for auto-volume deformable body: {prim}")
                    predicate = lambda prim, path: prim_utils.has_api(prim, "PhysicsCollisionAPI")
                    collision_mesh_prim = prim_utils.get_first_matching_child_prim(prim, predicate=predicate)
                    if collision_mesh_prim is None or not collision_mesh_prim.IsA(UsdGeom.PointBased):
                        raise RuntimeError(f"Unable to find collision mesh for auto-volume deformable body: {prim}")
                    simulation_mesh = UsdGeom.TetMesh(simulation_mesh_prim)
                    collision_mesh = (
                        UsdGeom.Mesh(collision_mesh_prim)
                        if collision_mesh_prim.IsA(UsdGeom.Mesh)
                        else UsdGeom.TetMesh(collision_mesh_prim)
                    )
                    if simulation_mesh.GetPointsAttr().Get() is None:
                        raise RuntimeError(
                            "An app update step is required between creating the deformable body and "
                            f"querying its properties for the simulation mesh to be populated: {simulation_mesh}"
                        )
                    # -- paths
                    simulation_mesh_paths.append(prim_utils.get_prim_path(simulation_mesh_prim))
                    collision_mesh_paths.append(prim_utils.get_prim_path(collision_mesh_prim))
                    # -- nodes per body
                    simulation_num_points = len(simulation_mesh.GetPointsAttr().Get())
                    collision_num_points = len(collision_mesh.GetPointsAttr().Get())
                    simulation_nodes_per_bodies.append(simulation_num_points)
                    collision_nodes_per_bodies.append(collision_num_points)
                    rest_nodes_per_bodies.append(simulation_num_points)
                    # -- elements per body
                    simulation_num_tetrahedra = len(simulation_mesh.GetTetVertexIndicesAttr().Get())
                    collision_num_tetrahedra = len(
                        collision_mesh.GetFaceVertexCountsAttr().Get()
                        if collision_mesh_prim.IsA(UsdGeom.Mesh)
                        else collision_mesh.GetTetVertexIndicesAttr().Get()
                    )
                    simulation_elements_per_bodies.append(simulation_num_tetrahedra)
                    collision_elements_per_bodies.append(collision_num_tetrahedra)
                    rest_elements_per_bodies.append(simulation_num_tetrahedra)
            else:
                raise RuntimeError(f"Unsupported prim: {prim}")

        # update properties
        self._num_nodes_per_element = 3 if self._deformable_type == "surface" else 4
        # - simulation mesh
        self._simulation_mesh_paths = simulation_mesh_paths
        self._simulation_nodes_per_body = max(simulation_nodes_per_bodies)
        self._simulation_elements_per_body = max(simulation_elements_per_bodies)
        # - collision mesh
        self._collision_mesh_paths = collision_mesh_paths
        self._collision_nodes_per_body = max(collision_nodes_per_bodies)
        self._collision_elements_per_body = max(collision_elements_per_bodies)
        # - rest mesh
        self._rest_nodes_per_body = max(rest_nodes_per_bodies)
        self._rest_elements_per_body = max(rest_elements_per_bodies)

    """
    Internal callbacks.
    """

    def _on_physics_ready(self, event) -> None:
        """Handle physics ready event."""
        super()._on_physics_ready(event)
        # get physics simulation view
        physics_simulation_view = SimulationManager._physics_sim_view__warp
        if physics_simulation_view is None or not physics_simulation_view.is_valid:
            carb.log_warn(f"Invalid physics simulation view. DeformablePrim ({self.paths}) will not be initialized")
            return
        # create deformable body view
        if not isinstance(self._physics_deformable_body_view_paths, str):
            carb.log_error(
                f"Current physics tensor implementation does not support `list[str]` as input for deformable body paths"
            )
            return
        pattern = self._physics_deformable_body_view_paths.replace(".*", "*")
        if self._deformable_type == "surface":
            self._physics_deformable_body_view = physics_simulation_view.create_surface_deformable_body_view(pattern)
        elif self._deformable_type == "volume":
            self._physics_deformable_body_view = physics_simulation_view.create_volume_deformable_body_view(pattern)
        else:
            raise RuntimeError(f"Invalid deformable type: {self._deformable_type}")
        # validate deformable body view
        if self._physics_deformable_body_view is None:
            carb.log_warn(f"Unable to create deformable body view for {self.paths}")
            return
        # TODO: it is not working... uncomment when fixed
        # if not self._physics_deformable_body_view.check():
        #     carb.log_warn(
        #         f"Unable to create deformable body view for {self.paths}. Underlying physics objects are not valid"
        #     )
        #     self._physics_deformable_body_view = None
        # get internal properties
        if not getattr(
            self, "_physics_tensor_entity_initialized", False
        ):  # HACK: make sure attribute exists if callback is called first
            self._physics_tensor_entity_initialized = True
            # deformable body properties
            self._num_nodes_per_element = self._physics_deformable_body_view.num_nodes_per_element
            # - simulation mesh
            self._simulation_mesh_paths = self._physics_deformable_body_view.simulation_mesh_prim_paths
            self._simulation_nodes_per_body = self._physics_deformable_body_view.max_simulation_nodes_per_body
            self._simulation_elements_per_body = self._physics_deformable_body_view.max_simulation_elements_per_body
            # - collision mesh
            self._collision_mesh_paths = self._physics_deformable_body_view.collision_mesh_prim_paths
            self._collision_nodes_per_body = self._physics_deformable_body_view.max_collision_nodes_per_body
            self._collision_elements_per_body = self._physics_deformable_body_view.max_collision_elements_per_body
            # - rest mesh (same topology as the simulation mesh)
            self._rest_nodes_per_body = self._physics_deformable_body_view.max_rest_nodes_per_body
            self._rest_elements_per_body = self._physics_deformable_body_view.max_simulation_elements_per_body

    def _on_timeline_stop(self, event) -> None:
        """Handle timeline stop event."""
        # invalidate deformable body view
        self._physics_deformable_body_view = None
