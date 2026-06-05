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

"""High level wrapper for manipulating prims that have Deformable Schema applied and their attributes."""

from __future__ import annotations

import weakref
from typing import Any, Literal

import carb
import carb.eventdispatcher
import isaacsim.core.experimental.utils.backend as backend_utils
import isaacsim.core.experimental.utils.ops as ops_utils
import isaacsim.core.experimental.utils.prim as prim_utils
import isaacsim.core.experimental.utils.stage as stage_utils
import numpy as np
import omni.physics.tensors
import omni.timeline
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
    """Check if the deformable schema exists on a prim and apply it if needed.

    Validates that the prim is suitable for deformable body physics and either wraps an existing
    deformable schema or creates a new one. Handles Mesh (surface), TetMesh (volume), and Xform
    prims with nested meshes.

    Args:
        stage: The USD stage containing the prim.
        path: Path to the prim to check or modify.
        deformable_type: Type of deformable body to create if schema doesn't exist.
            If None, inferred from prim type.

    Returns:
        A tuple of (action, deformable_type) where action is "wrap" if existing schema was found
        or "create" if new schema was applied.

    Raises:
        ValueError: If the prim has conflicting APIs, invalid geometry, or schema setup fails.
        AssertionError: If deformable_type conflicts with prim type.
    """
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
        # get prims
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
        # - volume-specific properties
        self._simulation_rotations = None
        self._simulation_stresses = None
        self._rest_pose_inverse_matrices = None
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
        def safe_timeline_stop_callback(event: Any, obj: Any = weakref.proxy(self)) -> None:
            try:
                obj._on_timeline_stop(event)
            except ReferenceError:
                # Object has been garbage collected, ignore the event
                pass

        self._subscription_to_timeline_stop_event = carb.eventdispatcher.get_eventdispatcher().observe_event(
            event_name=omni.timeline.GLOBAL_EVENT_STOP,
            on_event=safe_timeline_stop_callback,
            observer_name="isaacsim.core.experimental.prims.DeformablePrim._on_timeline_stop",
        )
        # setup physics-related configuration if simulation is running
        if SimulationManager._physics_sim_view__warp is not None:
            SimulationManager._physics_sim_interface.flush_changes()
            self._on_physics_ready(None)

    def __del__(self) -> None:
        """Clean up resources when the object is destroyed."""
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
        self, *, indices: int | list | np.ndarray | wp.array | None = None
    ) -> tuple[wp.array, wp.array, wp.array]:
        """Get the element (triangular faces for surface, tetrahedrons for volume) indices.

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
        self, *, indices: int | list | np.ndarray | wp.array | None = None
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
        indices: int | list | np.ndarray | wp.array | None = None,
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

    def get_nodal_velocities(self, *, indices: int | list | np.ndarray | wp.array | None = None) -> wp.array:
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
        indices: int | list | np.ndarray | wp.array | None = None,
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
        self, *, indices: int | list | np.ndarray | wp.array | None = None
    ) -> tuple[wp.array, wp.array]:
        """Get the nodal (mesh points) kinematic position targets of the simulation mesh of the prims.

        Backends: :guilabel:`tensor`. |br|
        Deformable type: :guilabel:`volume`.

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
        enabled: bool | list | np.ndarray | wp.array | None = None,
        *,
        indices: int | list | np.ndarray | wp.array | None = None,
    ) -> None:
        """Set the nodal (mesh points) kinematic position targets of the simulation mesh of the prims.

        Backends: :guilabel:`tensor`. |br|
        Deformable type: :guilabel:`volume`.

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
        weaker_than_descendants: bool | list | np.ndarray | wp.array | None = None,
        indices: int | list | np.ndarray | wp.array | None = None,
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
        self, *, indices: int | list | np.ndarray | wp.array | None = None
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

    def get_nodal_rotations(self, *, indices: int | list | np.ndarray | wp.array | None = None) -> wp.array:
        """Get the nodal (tetrahedrons) rotations of the simulation mesh of the prims.

        Backends: :guilabel:`tensor`. |br|
        Deformable type: :guilabel:`volume`.

        Args:
            indices: Indices of prims to process (shape ``(N,)``). If not defined, all wrapped prims are processed.

        Returns:
            The simulation mesh's nodal rotations (shape ``(N, num_nodes_per_body, 3)``).

        Raises:
            AssertionError: When the deformable body is not a volume.
            AssertionError: Wrapped prims are not valid.
            AssertionError: Physics tensor entity is not valid.

        Example:

        .. code-block:: python

            >>> # get the nodal rotations of all prims
            >>> simulation_rotations = prims.get_nodal_rotations()  # doctest: +NO_CHECK
            >>> simulation_rotations.shape
            (3, 1, 4)
            >>>
            >>> # get the nodal rotations of the first and last prims
            >>> simulation_rotations = prims.get_nodal_rotations(indices=[0, 2])  # doctest: +NO_CHECK
            >>> simulation_rotations.shape
            (2, 1, 4)
        """
        assert self.deformable_type == "volume", "Nodal rotations is only supported for volume deformable bodies"
        assert self.valid, _MSG_PRIM_NOT_VALID
        assert self.is_physics_tensor_entity_valid(), _MSG_PHYSICS_TENSOR_ENTITY_NOT_VALID
        # Tensor API
        simulation_indices = self._physics_deformable_body_view.get_simulation_element_indices()  # shape: (N, EpB, NpE)
        simulation_positions = self._physics_deformable_body_view.get_simulation_nodal_positions()  # shape: (N, NpB, 3)
        rest_positions = self._physics_deformable_body_view.get_rest_nodal_positions()  # shape: (N, NpB, 3)
        indices = ops_utils.resolve_indices(indices, count=len(self), device=simulation_positions.device)
        # - precompute rotations
        precompute_rotations = self._simulation_rotations is None
        if precompute_rotations:
            device = simulation_indices.device
            shape = simulation_indices.shape[:2]
            self._simulation_stresses = wp.zeros(shape=shape, dtype=wp.mat33, device=device)
            self._simulation_gradients = wp.zeros(shape=shape, dtype=wp.mat33, device=device)
            self._simulation_rotations = wp.zeros(shape=shape + (4,), dtype=wp.float32, device=device)
            self._rest_pose_inverse_matrices = wp.zeros(shape=shape, dtype=wp.mat33, device=device)
        # - compute rotations
        wp.launch(
            kernel=_wk_compute_deformable_rotation,
            dim=simulation_indices.shape[:2],
            inputs=[
                simulation_indices,
                simulation_positions,
                rest_positions,
                self._rest_pose_inverse_matrices,
                self._simulation_rotations,
                precompute_rotations,
            ],
            device=simulation_indices.device,
        )
        wp.synchronize()
        return self._simulation_rotations[indices].contiguous().to(self._device)

    def get_nodal_gradients(self, *, indices: int | list | np.ndarray | wp.array | None = None) -> wp.array:
        """Get the nodal (tetrahedrons) gradients of the simulation mesh of the prims.

        Backends: :guilabel:`tensor`. |br|
        Deformable type: :guilabel:`volume`.

        Args:
            indices: Indices of prims to process (shape ``(N,)``). If not defined, all wrapped prims are processed.

        Returns:
            The simulation mesh's nodal gradients (shape ``(N, num_elements_per_body)``, where items are 3x3 matrices).

        Raises:
            AssertionError: When the deformable body is not a volume.
            AssertionError: Wrapped prims are not valid.
            AssertionError: Physics tensor entity is not valid.

        Example:

        .. code-block:: python

            >>> # get the nodal gradients of all prims
            >>> simulation_gradients = prims.get_nodal_gradients()  # doctest: +NO_CHECK
            >>> simulation_gradients.shape
            (3, 1)
            >>>
            >>> # get the nodal gradients of the first and last prims
            >>> simulation_gradients = prims.get_nodal_gradients(indices=[0, 2])  # doctest: +NO_CHECK
            >>> simulation_gradients.shape
            (2, 1)
        """
        assert self.deformable_type == "volume", "Nodal gradients is only supported for volume deformable bodies"
        assert self.valid, _MSG_PRIM_NOT_VALID
        assert self.is_physics_tensor_entity_valid(), _MSG_PHYSICS_TENSOR_ENTITY_NOT_VALID
        # Tensor API
        simulation_indices = self._physics_deformable_body_view.get_simulation_element_indices()  # shape: (N, EpB, NpE)
        simulation_positions = self._physics_deformable_body_view.get_simulation_nodal_positions()  # shape: (N, NpB, 3)
        rest_positions = self._physics_deformable_body_view.get_rest_nodal_positions()  # shape: (N, NpB, 3)
        indices = ops_utils.resolve_indices(indices, count=len(self), device=simulation_positions.device)
        # - precompute rotations
        precompute_rotations = self._simulation_rotations is None
        if precompute_rotations:
            device = simulation_indices.device
            shape = simulation_indices.shape[:2]
            self._simulation_stresses = wp.zeros(shape=shape, dtype=wp.mat33, device=device)
            self._simulation_gradients = wp.zeros(shape=shape, dtype=wp.mat33, device=device)
            self._simulation_rotations = wp.zeros(shape=shape + (4,), dtype=wp.float32, device=device)
            self._rest_pose_inverse_matrices = wp.zeros(shape=shape, dtype=wp.mat33, device=device)
        # - compute rotations and gradients
        wp.launch(
            kernel=_wk_compute_deformable_rotation,
            dim=simulation_indices.shape[:2],
            inputs=[
                simulation_indices,
                simulation_positions,
                rest_positions,
                self._rest_pose_inverse_matrices,
                self._simulation_rotations,
                precompute_rotations,
            ],
            device=simulation_indices.device,
        )
        wp.launch(
            kernel=_wk_compute_deformable_gradient,
            dim=simulation_indices.shape[:2],
            inputs=[
                simulation_indices,
                simulation_positions,
                self._rest_pose_inverse_matrices,
                self._simulation_rotations,
                self._simulation_gradients,
            ],
            device=simulation_indices.device,
        )
        wp.synchronize()
        return self._simulation_gradients[indices].contiguous().to(self._device)

    def get_nodal_stresses(self, *, indices: int | list | np.ndarray | wp.array | None = None) -> wp.array:
        """Get the nodal (tetrahedrons) stresses of the simulation mesh of the prims.

        Backends: :guilabel:`tensor`. |br|
        Deformable type: :guilabel:`volume`.

        Args:
            indices: Indices of prims to process (shape ``(N,)``). If not defined, all wrapped prims are processed.

        Returns:
            The simulation mesh's nodal stresses (shape ``(N, num_elements_per_body)``, where items are 3x3 matrices).

        Raises:
            AssertionError: When the deformable body is not a volume.
            AssertionError: Wrapped prims are not valid.
            AssertionError: Physics tensor entity is not valid.

        Example:

        .. code-block:: python

            >>> # get the nodal stresses of all prims
            >>> simulation_stresses = prims.get_nodal_stresses()  # doctest: +SKIP
            >>> simulation_stresses.shape  # doctest: +SKIP
            (3, 1)
            >>>
            >>> # get the nodal stresses of the first and last prims
            >>> simulation_stresses = prims.get_nodal_stresses(indices=[0, 2])  # doctest: +SKIP
            >>> simulation_stresses.shape  # doctest: +SKIP
            (2, 1)
        """
        assert self.deformable_type == "volume", "Nodal stresses is only supported for volume deformable bodies"
        assert self.valid, _MSG_PRIM_NOT_VALID
        assert self.is_physics_tensor_entity_valid(), _MSG_PHYSICS_TENSOR_ENTITY_NOT_VALID
        # Tensor API
        simulation_indices = self._physics_deformable_body_view.get_simulation_element_indices()  # shape: (N, EpB, NpE)
        simulation_positions = self._physics_deformable_body_view.get_simulation_nodal_positions()  # shape: (N, NpB, 3)
        rest_positions = self._physics_deformable_body_view.get_rest_nodal_positions()  # shape: (N, NpB, 3)
        indices = ops_utils.resolve_indices(indices, count=len(self), device=simulation_positions.device)
        # - get material properties
        E = np.empty((0, 1))
        nu = np.empty((0, 1))
        with backend_utils.use_backend("usd"):
            applied_physics_materials = self.get_applied_physics_materials()
        for physics_material in applied_physics_materials:
            if physics_material is not None:
                E = np.append(E, physics_material.get_youngs_moduli().numpy(), axis=0)
                nu = np.append(nu, physics_material.get_poissons_ratios().numpy(), axis=0)
            else:
                E = np.append(E, np.zeros((1, 1)), axis=0)
                nu = np.append(nu, np.zeros((1, 1)), axis=0)
        youngs_moduli = wp.from_numpy(E, dtype=wp.float32, device=simulation_indices.device)
        poissons_ratios = wp.from_numpy(nu, dtype=wp.float32, device=simulation_indices.device)
        # - precompute rotations
        precompute_rotations = self._simulation_rotations is None
        if precompute_rotations:
            device = simulation_indices.device
            shape = simulation_indices.shape[:2]
            self._simulation_stresses = wp.zeros(shape=shape, dtype=wp.mat33, device=device)
            self._simulation_gradients = wp.zeros(shape=shape, dtype=wp.mat33, device=device)
            self._simulation_rotations = wp.zeros(shape=shape + (4,), dtype=wp.float32, device=device)
            self._rest_pose_inverse_matrices = wp.zeros(shape=shape, dtype=wp.mat33, device=device)
        # - compute rotations and stresses
        wp.launch(
            kernel=_wk_compute_deformable_rotation,
            dim=simulation_indices.shape[:2],
            inputs=[
                simulation_indices,
                simulation_positions,
                rest_positions,
                self._rest_pose_inverse_matrices,
                self._simulation_rotations,
                precompute_rotations,
            ],
            device=simulation_indices.device,
        )
        wp.launch(
            kernel=_wk_compute_deformable_stress,
            dim=simulation_indices.shape[:2],
            inputs=[
                simulation_indices,
                simulation_positions,
                self._rest_pose_inverse_matrices,
                self._simulation_rotations,
                youngs_moduli,
                poissons_ratios,
                self._simulation_stresses,
            ],
            device=simulation_indices.device,
        )
        wp.synchronize()
        return self._simulation_stresses[indices].contiguous().to(self._device)

    """
    Internal methods.
    """

    def _check_for_tensor_backend(self, backend: str, *, fallback_backend: str = "usd") -> str:
        """Check if the tensor backend is valid.

        Args:
            backend: The backend to check.
            fallback_backend: The backend to use if the tensor backend is not valid.

        Returns:
            The validated backend name.
        """
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

    def _on_physics_ready(self, event: object) -> None:
        """Handle physics ready event.

        Args:
            event: The physics ready event.
        """
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

        # precompute the rest pose inverse matrices for rotations
        self._precompute_rotations = True

    def _on_timeline_stop(self, event: object) -> None:
        """Handle timeline stop event.

        Args:
            event: The timeline stop event.
        """
        # invalidate deformable body view
        self._physics_deformable_body_view = None


"""
Warp kernels
"""


@wp.func
def _wf_get_matrix_column(m: wp.mat33, i: int) -> wp.vec3:
    """Extract a column from a 3x3 matrix.

    Args:
        m: The 3x3 matrix to extract from.
        i: Column index (0, 1, or 2).

    Returns:
        The specified column as a 3D vector.
    """
    return wp.vec3(m[0][i], m[1][i], m[2][i])


@wp.func
def _wf_quaternion_from_axis_angle(angle_radians: float, axis: wp.vec3) -> wp.quat:
    """Create a quaternion from an axis-angle representation.

    Converts a rotation specified by an axis vector and angle into a quaternion.

    Args:
        angle_radians: Rotation angle in radians.
        axis: The rotation axis as a normalized 3D vector.

    Returns:
        The resulting quaternion representing the rotation.
    """
    half_angle = angle_radians * 0.5
    s = wp.sin(half_angle)
    w = wp.cos(half_angle)
    x = s * axis[0]
    y = s * axis[1]
    z = s * axis[2]
    return wp.quat(x, y, z, w)


@wp.func
def _wf_compute_deformation_gradient(
    Q_inverse: wp.mat33,
    rotation: wp.quat,
    indices: wp.array3d(dtype=wp.uint32),
    positions: wp.array3d(dtype=wp.float32),
    body_index: int,
    tetrahedron_index: int,
) -> wp.mat33:
    """Compute the deformation gradient for a tetrahedron element.

    Calculates F = R^T * P * Q^-1 where R is the rotation matrix, P is the current
    deformation matrix, and Q^-1 is the inverse rest pose matrix.

    Args:
        Q_inverse: Inverse rest pose matrix for the tetrahedron.
        rotation: Current rotation as a quaternion.
        indices: Element connectivity indices for all bodies and tetrahedra.
        positions: Current nodal positions for all bodies.
        body_index: Index of the deformable body.
        tetrahedron_index: Index of the tetrahedron within the body.

    Returns:
        The computed deformation gradient matrix.
    """
    R = wp.quat_to_matrix(rotation)
    P = _wf_compute_deformation_matrix(indices, positions, body_index, tetrahedron_index, False)
    F = P * Q_inverse
    return wp.transpose(R) * F


@wp.func
def _wf_compute_deformation_matrix(
    indices: wp.array3d(dtype=wp.uint32),
    positions: wp.array3d(dtype=wp.float32),
    body_index: int,
    tetrahedron_index: int,
    inverse: bool,
) -> wp.mat33:
    """Compute the deformation matrix for a tetrahedron element.

    Constructs a 3x3 matrix from the displacement vectors of the tetrahedron vertices.
    Optionally computes the inverse matrix for rest pose calculations.

    Args:
        indices: Element connectivity indices for all bodies and tetrahedra.
        positions: Nodal positions for all bodies.
        body_index: Index of the deformable body.
        tetrahedron_index: Index of the tetrahedron within the body.
        inverse: Whether to return the inverse matrix.

    Returns:
        The deformation matrix or its inverse if requested.
    """
    tetrahedron_indices = indices[body_index][tetrahedron_index]
    v0 = positions[body_index][wp.int32(tetrahedron_indices[0])]
    v1 = positions[body_index][wp.int32(tetrahedron_indices[1])]
    v2 = positions[body_index][wp.int32(tetrahedron_indices[2])]
    v3 = positions[body_index][wp.int32(tetrahedron_indices[3])]
    # compute displacement field
    u1 = wp.vec3(v1[0] - v0[0], v1[1] - v0[1], v1[2] - v0[2])
    u2 = wp.vec3(v2[0] - v0[0], v2[1] - v0[1], v2[2] - v0[2])
    u3 = wp.vec3(v3[0] - v0[0], v3[1] - v0[1], v3[2] - v0[2])
    # current configuration matrix
    m = wp.matrix_from_cols(u1, u2, u3)
    if inverse:
        det = wp.determinant(m)
        volume = det / 6.0
        if volume < 1e-9:
            m = wp.matrix_from_rows(wp.vec3(0.0, 0.0, 0.0), wp.vec3(0.0, 0.0, 0.0), wp.vec3(0.0, 0.0, 0.0))
        else:
            m = wp.inverse(m)
    return m


@wp.func
def _wf_extract_rotation(A: wp.mat33, q: wp.quat, max_iterations: int, eps: float = 1e-6) -> wp.quat:
    """Extract rotation from a deformation matrix using iterative algorithm.

    Uses an iterative method to find the rotation component that best aligns with the
    given deformation matrix, starting from an initial quaternion guess.

    Args:
        A: The deformation matrix to extract rotation from.
        q: Initial quaternion guess for the rotation.
        max_iterations: Maximum number of iterations to perform.
        eps: Convergence tolerance for early termination.

    Returns:
        The extracted rotation as a normalized quaternion.
    """
    for _ in range(max_iterations):
        # convert quaternion q to rotation matrix R
        R = wp.quat_to_matrix(q)
        # compute omega vector as the sum of cross products of corresponding columns of R and A
        R0 = _wf_get_matrix_column(R, 0)
        R1 = _wf_get_matrix_column(R, 1)
        R2 = _wf_get_matrix_column(R, 2)
        A0 = _wf_get_matrix_column(A, 0)
        A1 = _wf_get_matrix_column(A, 1)
        A2 = _wf_get_matrix_column(A, 2)
        omega = wp.cross(R0, A0) + wp.cross(R1, A1) + wp.cross(R2, A2)
        # compute normalization denominator: sum of dot products plus epsilon
        denominator = wp.abs(wp.dot(R0, A0) + wp.dot(R1, A1) + wp.dot(R2, A2)) + eps
        # normalize omega
        omega = omega * (1.0 / denominator)
        # magnitude of omega
        w = wp.length(omega)
        # normalize omega vector to unit length if possible
        omega = wp.normalize(omega)
        # update current estimate quaternion
        q = _wf_quaternion_from_axis_angle(w, omega) * q
        # normalize quaternion to keep it a valid rotation quaternion
        q = wp.normalize(q)
        # early exit
        if w < eps:
            break
    return q


@wp.kernel(enable_backward=False)
def _wk_compute_deformable_rotation(
    simulation_indices: wp.array3d(dtype=wp.uint32),
    simulation_positions: wp.array3d(dtype=wp.float32),
    rest_positions: wp.array3d(dtype=wp.float32),
    rest_pose_inverse_matrices: wp.array2d(dtype=wp.mat33),
    simulation_rotations: wp.array3d(dtype=wp.float32),
    precompute: bool,
) -> None:
    """Warp kernel to compute rotations for deformable tetrahedron elements.

    Extracts rotation components from deformation gradients for each tetrahedron element.
    Optionally precomputes rest pose inverse matrices on first call.

    Args:
        simulation_indices: Element connectivity indices for all bodies and tetrahedra.
        simulation_positions: Current nodal positions for all bodies.
        rest_positions: Rest pose nodal positions for all bodies.
        rest_pose_inverse_matrices: Storage for precomputed inverse rest pose matrices.
        simulation_rotations: Storage for computed rotations as quaternions.
        precompute: Whether to precompute rest pose inverse matrices and initialize rotations.
    """
    body_index, tetrahedron_index = wp.tid()
    if precompute:
        rest_pose_inverse_matrices[body_index][tetrahedron_index] = _wf_compute_deformation_matrix(
            simulation_indices, rest_positions, body_index, tetrahedron_index, True
        )
        simulation_rotations[body_index][tetrahedron_index][0] = 0.0
        simulation_rotations[body_index][tetrahedron_index][1] = 0.0
        simulation_rotations[body_index][tetrahedron_index][2] = 0.0
        simulation_rotations[body_index][tetrahedron_index][3] = 1.0
    # compute P and Q_inverse and deformation gradient
    P = _wf_compute_deformation_matrix(simulation_indices, simulation_positions, body_index, tetrahedron_index, False)
    Q_inverse = rest_pose_inverse_matrices[body_index][tetrahedron_index]
    F = P * Q_inverse
    # current rotation
    tetrahedron_rotation = simulation_rotations[body_index][tetrahedron_index]
    q = wp.quat(tetrahedron_rotation[0], tetrahedron_rotation[1], tetrahedron_rotation[2], tetrahedron_rotation[3])
    # extract rotation
    quaternion = _wf_extract_rotation(F, q, 100)
    simulation_rotations[body_index][tetrahedron_index][0] = quaternion.x
    simulation_rotations[body_index][tetrahedron_index][1] = quaternion.y
    simulation_rotations[body_index][tetrahedron_index][2] = quaternion.z
    simulation_rotations[body_index][tetrahedron_index][3] = quaternion.w


@wp.kernel(enable_backward=False)
def _wk_compute_deformable_gradient(
    simulation_indices: wp.array3d(dtype=wp.uint32),
    simulation_positions: wp.array3d(dtype=wp.float32),
    rest_pose_inverse_matrices: wp.array2d(dtype=wp.mat33),
    simulation_rotations: wp.array3d(dtype=wp.float32),
    simulation_gradients: wp.array2d(dtype=wp.mat33),
) -> None:
    """Warp kernel to compute deformation gradients for all tetrahedron elements.

    Processes each tetrahedron element across all deformable bodies to compute their
    deformation gradients using current positions, rotations, and rest pose data.

    Args:
        simulation_indices: Element connectivity indices for all bodies and tetrahedra.
        simulation_positions: Current nodal positions for all bodies.
        rest_pose_inverse_matrices: Precomputed inverse rest pose matrices.
        simulation_rotations: Current rotations for all tetrahedra as quaternions.
        simulation_gradients: Output array for computed deformation gradients.
    """
    body_index, tetrahedron_index = wp.tid()
    rest_pose = rest_pose_inverse_matrices[body_index][tetrahedron_index]
    tetrahedron_rotation = simulation_rotations[body_index][tetrahedron_index]
    q = wp.quat(tetrahedron_rotation[0], tetrahedron_rotation[1], tetrahedron_rotation[2], tetrahedron_rotation[3])

    simulation_gradients[body_index][tetrahedron_index] = _wf_compute_deformation_gradient(
        rest_pose, q, simulation_indices, simulation_positions, body_index, tetrahedron_index
    )


@wp.kernel(enable_backward=False)
def _wk_compute_deformable_stress(
    simulation_indices: wp.array3d(dtype=wp.uint32),
    simulation_positions: wp.array3d(dtype=wp.float32),
    rest_pose_inverse_matrices: wp.array2d(dtype=wp.mat33),
    simulation_rotations: wp.array3d(dtype=wp.float32),
    youngs_modulus: wp.array2d(dtype=wp.float32),
    poissons_ratio: wp.array2d(dtype=wp.float32),
    simulation_stresses: wp.array2d(dtype=wp.mat33),
) -> None:
    """Warp kernel to compute stress tensors for deformable tetrahedron elements.

    Calculates Cauchy stress tensors using linear elasticity theory based on deformation
    gradients and material properties (Young's modulus and Poisson's ratio).

    Args:
        simulation_indices: Element connectivity indices for all bodies and tetrahedra.
        simulation_positions: Current nodal positions for all bodies.
        rest_pose_inverse_matrices: Precomputed inverse rest pose matrices.
        simulation_rotations: Current rotations for all tetrahedra as quaternions.
        youngs_modulus: Young's modulus values for each deformable body.
        poissons_ratio: Poisson's ratio values for each deformable body.
        simulation_stresses: Output array for computed stress tensors.
    """
    body_index, tetrahedron_index = wp.tid()
    rest_pose = rest_pose_inverse_matrices[body_index][tetrahedron_index]
    tetrahedron_rotation = simulation_rotations[body_index][tetrahedron_index]
    q = wp.quat(tetrahedron_rotation[0], tetrahedron_rotation[1], tetrahedron_rotation[2], tetrahedron_rotation[3])

    F = _wf_compute_deformation_gradient(
        rest_pose, q, simulation_indices, simulation_positions, body_index, tetrahedron_index
    )
    E = youngs_modulus[body_index][0]
    nu = poissons_ratio[body_index][0]
    mu = E / (1.0 + nu) / 2.0
    lmbda = E * nu / (1.0 + nu) / (1.0 - 2.0 * nu)
    identity = wp.mat33(1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0)
    eps = (wp.transpose(F) + F) * 0.5 - identity
    J = wp.determinant(F)
    trace = eps[0][0] + eps[1][1] + eps[2][2]
    P = wp.quat_to_matrix(q) * eps * (2.0 * mu) + identity * (lmbda * trace)

    simulation_stresses[body_index][tetrahedron_index] = (1.0 / J) * P * wp.transpose(F)
