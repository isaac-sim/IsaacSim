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

"""Synchronize USD prims with motion-generation planning world interfaces."""

from __future__ import annotations

from typing import Generic, TypeVar

import isaacsim.core.experimental.utils.backend as backend_utils
import isaacsim.core.experimental.utils.prim as prim_utils
import numpy as np
import warp as wp
from isaacsim.core.experimental.objects import (
    Capsule,
    Cone,
    Cube,
    Cylinder,
    Mesh,
    Plane,
    Sphere,
)
from isaacsim.core.experimental.prims import (
    GeomPrim,
    XformPrim,
)
from isaacsim.robot_motion.schema import MOTION_PLANNING_API_NAME, MOTION_PLANNING_ENABLED_ATTR

from .obstacle_strategy import (
    ObstacleConfiguration,
    ObstacleRepresentation,
    ObstacleStrategy,
)
from .trackable_api import TrackableApi
from .utils import collision_approximation, scene_validation
from .world_interface import WorldInterface

_SUPPORTED_COLLISION_APIS = frozenset(
    {
        TrackableApi.PHYSICS_COLLISION,
        TrackableApi.MOTION_GENERATION_COLLISION,
    }
)


def _add_sphere_from_prim(
    prim_path: str,
    world_interface: WorldInterface,
    obstacle_configuration: ObstacleConfiguration,
    collision_api: TrackableApi,
) -> None:
    """Add a sphere prim to the planning world interface.

    Args:
        prim_path: Path to the sphere prim.
        world_interface: Planning world interface to populate.
        obstacle_configuration: Configuration for the obstacle representation.
        collision_api: Collision API which is used to signal that the obstacle is enabled/disabled.
    """
    # Given that this object is being added directly as a sphere,
    # we can get its data from the core API:
    isaac_core_object = Sphere(paths=prim_path)

    world_interface.add_spheres(
        prim_paths=[prim_path],
        radii=isaac_core_object.get_radii(),
        scales=isaac_core_object.get_local_scales(),
        safety_tolerances=wp.array([[obstacle_configuration.safety_tolerance]]),
        poses=isaac_core_object.get_world_poses(),
        enabled_array=_get_collision_enabled_values([prim_path], collision_api),
    )


def _add_cube_from_prim(
    prim_path: str,
    world_interface: WorldInterface,
    obstacle_configuration: ObstacleConfiguration,
    collision_api: TrackableApi,
) -> None:
    """Add a cube prim to the planning world interface.

    Args:
        prim_path: Path to the cube prim.
        world_interface: Planning world interface to populate.
        obstacle_configuration: Configuration for the obstacle representation.
        collision_api: Collision API which is used to signal that the obstacle is enabled/disabled.
    """
    # Given that this object is being added directly as a cube,
    # we can get its data from the core API:
    isaac_core_object = Cube(paths=prim_path)

    world_interface.add_cubes(
        prim_paths=[prim_path],
        sizes=isaac_core_object.get_sizes(),
        scales=isaac_core_object.get_local_scales(),
        safety_tolerances=wp.array([[obstacle_configuration.safety_tolerance]]),
        poses=isaac_core_object.get_world_poses(),
        enabled_array=_get_collision_enabled_values([prim_path], collision_api),
    )


def _add_cone_from_prim(
    prim_path: str,
    world_interface: WorldInterface,
    obstacle_configuration: ObstacleConfiguration,
    collision_api: TrackableApi,
) -> None:
    """Add a cone prim to the planning world interface.

    Args:
        prim_path: Path to the cone prim.
        world_interface: Planning world interface to populate.
        obstacle_configuration: Configuration for the obstacle representation.
        collision_api: Collision API which is used to signal that the obstacle is enabled/disabled.
    """
    # Given that this object is being added directly as a cone,
    # we can get its data from the core API:
    isaac_core_object = Cone(paths=prim_path)
    world_interface.add_cones(
        prim_paths=[prim_path],
        axes=isaac_core_object.get_axes(),
        radii=isaac_core_object.get_radii(),
        lengths=isaac_core_object.get_heights(),
        scales=isaac_core_object.get_local_scales(),
        safety_tolerances=wp.array([[obstacle_configuration.safety_tolerance]]),
        poses=isaac_core_object.get_world_poses(),
        enabled_array=_get_collision_enabled_values([prim_path], collision_api),
    )


def _add_plane_from_prim(
    prim_path: str,
    world_interface: WorldInterface,
    obstacle_configuration: ObstacleConfiguration,
    collision_api: TrackableApi,
) -> None:
    """Add a plane prim to the planning world interface.

    Args:
        prim_path: Path to the plane prim.
        world_interface: Planning world interface to populate.
        obstacle_configuration: Configuration for the obstacle representation.
        collision_api: Collision API which is used to signal that the obstacle is enabled/disabled.
    """
    # Given that this object is being added directly as a plane,
    # we can get its data from the core API:
    isaac_core_object = Plane(paths=prim_path)
    world_interface.add_planes(
        prim_paths=[prim_path],
        axes=isaac_core_object.get_axes(),
        lengths=isaac_core_object.get_lengths(),
        widths=isaac_core_object.get_widths(),
        scales=isaac_core_object.get_local_scales(),
        safety_tolerances=wp.array([[obstacle_configuration.safety_tolerance]]),
        poses=isaac_core_object.get_world_poses(),
        enabled_array=_get_collision_enabled_values([prim_path], collision_api),
    )


def _add_capsule_from_prim(
    prim_path: str,
    world_interface: WorldInterface,
    obstacle_configuration: ObstacleConfiguration,
    collision_api: TrackableApi,
) -> None:
    """Add a capsule prim to the planning world interface.

    Args:
        prim_path: Path to the capsule prim.
        world_interface: Planning world interface to populate.
        obstacle_configuration: Configuration for the obstacle representation.
        collision_api: Collision API which is used to signal that the obstacle is enabled/disabled.
    """
    # Given that this object is being added directly as a capsule,
    # we can get its data from the core API:
    isaac_core_object = Capsule(paths=prim_path)
    world_interface.add_capsules(
        prim_paths=[prim_path],
        axes=isaac_core_object.get_axes(),
        radii=isaac_core_object.get_radii(),
        lengths=isaac_core_object.get_heights(),
        scales=isaac_core_object.get_local_scales(),
        safety_tolerances=wp.array([[obstacle_configuration.safety_tolerance]]),
        poses=isaac_core_object.get_world_poses(),
        enabled_array=_get_collision_enabled_values([prim_path], collision_api),
    )


def _add_cylinder_from_prim(
    prim_path: str,
    world_interface: WorldInterface,
    obstacle_configuration: ObstacleConfiguration,
    collision_api: TrackableApi,
) -> None:
    """Add a cylinder prim to the planning world interface.

    Args:
        prim_path: Path to the cylinder prim.
        world_interface: Planning world interface to populate.
        obstacle_configuration: Configuration for the obstacle representation.
        collision_api: Collision API which is used to signal that the obstacle is enabled/disabled.
    """
    # Given that this object is being added directly as a cylinder,
    # we can get its data from the core API:
    isaac_core_object = Cylinder(paths=prim_path)
    world_interface.add_cylinders(
        prim_paths=[prim_path],
        axes=isaac_core_object.get_axes(),
        radii=isaac_core_object.get_radii(),
        lengths=isaac_core_object.get_heights(),
        scales=isaac_core_object.get_local_scales(),
        safety_tolerances=wp.array([[obstacle_configuration.safety_tolerance]]),
        poses=isaac_core_object.get_world_poses(),
        enabled_array=_get_collision_enabled_values([prim_path], collision_api),
    )


def _add_mesh_from_prim(
    prim_path: str,
    world_interface: WorldInterface,
    obstacle_configuration: ObstacleConfiguration,
    collision_api: TrackableApi,
) -> None:
    """Add a mesh prim to the planning world interface.

    Args:
        prim_path: Path to the mesh prim.
        world_interface: Planning world interface to populate.
        obstacle_configuration: Configuration for the obstacle representation.
        collision_api: Collision API which is used to signal that the obstacle is enabled/disabled.
    """
    # Given that this object is being added directly as a mesh,
    # we can get its data from the core API:
    isaac_core_object = Mesh(paths=prim_path)
    face_indices, face_counts, _, _ = isaac_core_object.get_face_specs()

    world_interface.add_meshes(
        prim_paths=[prim_path],
        points=isaac_core_object.get_points(),
        face_vertex_indices=face_indices,
        face_vertex_counts=face_counts,
        normals=isaac_core_object.get_normals(),
        scales=isaac_core_object.get_local_scales(),
        safety_tolerances=wp.array([[obstacle_configuration.safety_tolerance]]),
        poses=isaac_core_object.get_world_poses(),
        enabled_array=_get_collision_enabled_values([prim_path], collision_api),
    )


def _add_triangulated_mesh_from_prim(
    prim_path: str,
    world_interface: WorldInterface,
    obstacle_configuration: ObstacleConfiguration,
    collision_api: TrackableApi,
) -> None:
    """Add a triangulated mesh prim to the planning world interface.

    Args:
        prim_path: Path to the mesh prim.
        world_interface: Planning world interface to populate.
        obstacle_configuration: Configuration for the obstacle representation.
        collision_api: Collision API which is used to signal that the obstacle is enabled/disabled.
    """
    # Given that this object is being added directly as a triangulated mesh,
    # we can get its data from the core API:
    isaac_core_object = Mesh(paths=prim_path)

    # triangulate the mesh:
    all_triangulated_mesh_indices = collision_approximation.triangulate_mesh(isaac_core_object)

    if len(all_triangulated_mesh_indices) < 1:
        raise ValueError("collision_approximation.triangulate_mesh failed to triangulate any meshes.")
    triangulated_mesh_indices = all_triangulated_mesh_indices[0]

    # reshape to something more natural, pass as a warp array to match other inputs.
    triangulated_mesh_indices = wp.from_numpy(
        triangulated_mesh_indices,
        dtype=wp.int32,
    )

    world_interface.add_triangulated_meshes(
        prim_paths=[prim_path],
        points=isaac_core_object.get_points(),
        face_vertex_indices=[triangulated_mesh_indices],
        scales=isaac_core_object.get_local_scales(),
        safety_tolerances=wp.array([[obstacle_configuration.safety_tolerance]]),
        poses=isaac_core_object.get_world_poses(),
        enabled_array=_get_collision_enabled_values([prim_path], collision_api),
    )


def _add_oriented_bounding_box_from_prim(
    prim_path: str,
    world_interface: WorldInterface,
    obstacle_configuration: ObstacleConfiguration,
    collision_api: TrackableApi,
) -> None:
    """Add an oriented bounding box representation to the planning world interface.

    Args:
        prim_path: Path to the prim.
        world_interface: Planning world interface to populate.
        obstacle_configuration: Configuration for the obstacle representation.
        collision_api: Collision API which is used to signal that the obstacle is enabled/disabled.
    """
    # Create a fresh bounding box cache bound to the current stage.
    bbox_cache = collision_approximation.create_bbox_cache()

    # compute the oriented bounding box of the prim:
    obb = collision_approximation.compute_obb(
        bbox_cache=bbox_cache,
        prim_path=prim_path,
    )

    isaac_core_object = XformPrim(paths=prim_path)

    # Stack arrays to match expected format: (N, 3) for centers, (N, 4) for rotations, (N, 3) for half_side_lengths
    centers = wp.from_numpy(obb.center.reshape(1, 3), dtype=wp.float32)
    rotations = wp.from_numpy(obb.rotation.reshape(1, 4), dtype=wp.float32)
    half_side_lengths = wp.from_numpy(obb.half_side_lengths.reshape(1, 3), dtype=wp.float32)

    world_interface.add_oriented_bounding_boxes(
        prim_paths=[prim_path],
        centers=centers,
        rotations=rotations,
        half_side_lengths=half_side_lengths,
        scales=isaac_core_object.get_local_scales(),
        safety_tolerances=wp.array([[obstacle_configuration.safety_tolerance]]),
        poses=isaac_core_object.get_world_poses(),
        enabled_array=_get_collision_enabled_values([prim_path], collision_api),
    )


_ADD_OBJECT_CALLBACK_MAP = {
    ObstacleRepresentation.SPHERE: _add_sphere_from_prim,
    ObstacleRepresentation.CUBE: _add_cube_from_prim,
    ObstacleRepresentation.CONE: _add_cone_from_prim,
    ObstacleRepresentation.PLANE: _add_plane_from_prim,
    ObstacleRepresentation.CAPSULE: _add_capsule_from_prim,
    ObstacleRepresentation.CYLINDER: _add_cylinder_from_prim,
    ObstacleRepresentation.MESH: _add_mesh_from_prim,
    ObstacleRepresentation.TRIANGULATED_MESH: _add_triangulated_mesh_from_prim,
    ObstacleRepresentation.OBB: _add_oriented_bounding_box_from_prim,
    # TODO:
    # SIGNED_DISTANCE_FIELD:
    # CONVEX_HULL:
    # BOUNDING_SPHERE:
    # CONVEX_DECOMPOSITION:
}


def _get_collision_enabled_values(prim_paths: list[str], collision_api: TrackableApi) -> wp.array:
    """Get collision enabled values for prims based on the specified API.

    Args:
        prim_paths: List of prim paths to query.
        collision_api: Collision API to use for reading enabled state.

    Returns:
        Boolean array indicating collision enabled state (shape ``(N, 1)``).
    """
    if collision_api == TrackableApi.PHYSICS_COLLISION:
        collision_object = GeomPrim(paths=prim_paths)
        return collision_object.get_enabled_collisions()
    elif collision_api == TrackableApi.MOTION_GENERATION_COLLISION:
        enabled = np.zeros((len(prim_paths), 1), dtype=np.bool_)
        for i, prim_path in enumerate(prim_paths):
            prim = prim_utils.get_prim_at_path(prim_path)
            if not prim.IsValid():
                raise RuntimeError(f"Prim {prim_path} is invalid or does not exist.")
            if MOTION_PLANNING_ENABLED_ATTR not in prim_utils.get_prim_attribute_names(prim):
                raise RuntimeError(
                    f"Prim {prim_path} is missing the {MOTION_PLANNING_ENABLED_ATTR} attribute. "
                    f"This should not happen if the prim was properly validated during WorldBinding.initialize()."
                )
            enabled[i] = prim_utils.get_prim_attribute_value(prim, MOTION_PLANNING_ENABLED_ATTR)
        return wp.from_numpy(enabled, dtype=wp.bool)
    else:
        raise ValueError(f"Unsupported collision API: {collision_api}")


TWorldInterface = TypeVar("TWorldInterface", bound=WorldInterface)


class WorldBinding(Generic[TWorldInterface]):
    """Binding that mirrors tracked USD prims into a planning world interface.

    This class populates a planning world implementation from specified USD prims
    and synchronizes their transforms.

    Args:
        world_interface: World implementation to populate and update.
        obstacle_strategy: Strategy used to select obstacle representations per prim.
        tracked_prims: Prim paths to track in the USD stage.
        tracked_collision_api: Collision API used to read the initial enabled state.

    Raises:
        ValueError: If tracked_collision_api is not a supported collision API.

    Example:

    .. code-block:: python

        from isaacsim.robot_motion.experimental.motion_generation import (
            ObstacleStrategy,
            TrackableApi,
            WorldBinding,
        )
        from isaacsim.robot_motion.experimental.motion_generation.tests.mirror_world_interface import (
            MirrorWorldInterface, # your planning world interface goes here!
        )

        world_binding = WorldBinding(
            world_interface=MirrorWorldInterface(),
            obstacle_strategy=ObstacleStrategy(),
            tracked_prims=["/World/Sphere"],
            tracked_collision_api=TrackableApi.PHYSICS_COLLISION,
        )
    """

    def __init__(
        self,
        *,
        world_interface: TWorldInterface,
        obstacle_strategy: ObstacleStrategy,
        tracked_prims: list[str],
        tracked_collision_api: TrackableApi,
    ) -> None:
        if tracked_collision_api not in _SUPPORTED_COLLISION_APIS:
            raise ValueError(
                f"Unsupported collision API: {tracked_collision_api}. "
                f"Supported APIs: {sorted(_SUPPORTED_COLLISION_APIS)}"
            )

        self._tracked_collision_api = tracked_collision_api
        self._world_interface = world_interface
        self._obstacle_strategy = obstacle_strategy
        self._tracked_prims = tracked_prims
        self._initialized = False

    def initialize(self) -> None:
        """Initialize and populate the planning world from tracked prims.

        Raises:
            RuntimeError: If already initialized.
            RuntimeError: If any tracked prim lacks the CollisionAPI.
            RuntimeError: If any tracked prim path is invalid in the stage.
            AssertionError: If any ancestor prims of the tracked prims have non-unity
                scaling, which would cause issues with world-space operations.

        Example:

        .. code-block:: python

            from isaacsim.robot_motion.experimental.motion_generation import (
                ObstacleStrategy,
                TrackableApi,
                WorldBinding,
            )
            from isaacsim.robot_motion.experimental.motion_generation.tests.mirror_world_interface import (
                MirrorWorldInterface,
            )

            world_binding = WorldBinding(
                world_interface=MirrorWorldInterface(),
                obstacle_strategy=ObstacleStrategy(),
                tracked_prims=["/World/Sphere"],
                tracked_collision_api=TrackableApi.PHYSICS_COLLISION,
            )
            world_binding.initialize()
        """
        if self._initialized:
            raise RuntimeError(
                "WorldBinding is already initialized and does not support re-initialization. "
                "Create a new WorldBinding instance to track new prims."
            )

        if len(self._tracked_prims) == 0:
            self._initialized = True
            return

        with backend_utils.use_backend("usd"):
            prims = [prim_utils.get_prim_at_path(p) for p in self._tracked_prims]

        # Check that all of the tracked prims are valid:
        valid_prims = [prim.IsValid() for prim in prims]
        if not all(valid_prims):
            invalid_prims = [prim_path for prim_path, valid in zip(self._tracked_prims, valid_prims) if not valid]
            raise RuntimeError(f"The following paths do not correspond to valid prims in the stage: {invalid_prims}")

        # Check that all of the tracked prims have the necessary APIs applied:
        if not all(prim_utils.has_api(p, self._tracked_collision_api) for p in self._tracked_prims):
            prims_without_collision_api = [
                p for p in self._tracked_prims if not prim_utils.has_api(p, self._tracked_collision_api)
            ]
            raise RuntimeError(
                f"The following prims do not have {self._tracked_collision_api} applied: {prims_without_collision_api}"
            )

        # We need to check that all ancestor prims have NO scaling about any axes.
        # This guarantees that no shearing can occur and ensures local_scale == world_scale.
        invalid_ancestors = scene_validation.find_all_invalid_ancestors(prim_paths=self._tracked_prims)
        if len(invalid_ancestors) != 0:
            raise AssertionError(f"The following ancestor prims have non-unity scaling.\n{invalid_ancestors}")

        # For motion planning API, validate that all prims have the enabled attribute.
        if self._tracked_collision_api == TrackableApi.MOTION_GENERATION_COLLISION:
            prims_without_attr = [
                prim_path
                for prim, prim_path in zip(prims, self._tracked_prims)
                if MOTION_PLANNING_ENABLED_ATTR not in prim_utils.get_prim_attribute_names(prim)
            ]
            if prims_without_attr:
                raise RuntimeError(
                    f"The following prims have {MOTION_PLANNING_API_NAME} applied but are missing the "
                    f"{MOTION_PLANNING_ENABLED_ATTR} attribute: {prims_without_attr}"
                )

        for prim_path in self._tracked_prims:
            obstacle_configuration = self._obstacle_strategy.get_obstacle_configuration(prim_path)
            _ADD_OBJECT_CALLBACK_MAP[obstacle_configuration.representation](
                prim_path=prim_path,
                world_interface=self._world_interface,
                obstacle_configuration=obstacle_configuration,
                collision_api=self._tracked_collision_api,
            )

        self._xform = XformPrim(paths=self._tracked_prims)

        self._initialized = True

    def synchronize_transforms(self) -> None:
        """Synchronize tracked prim transforms into the planning world.

        Update the world poses of tracked obstacles.

        Raises:
            RuntimeError: If the world binding has not been initialized.

        Example:

        .. code-block:: python

            >>> world_binding.synchronize_transforms()
        """
        if not self._initialized:
            raise RuntimeError("WorldBinding is not initialized. Call initialize() first.")

        if len(self._tracked_prims) == 0:
            return

        self._world_interface.update_obstacle_transforms(self._tracked_prims, self._xform.get_world_poses())

    def get_world_interface(self) -> WorldInterface:
        """Return the planning world interface instance.

        Returns:
            The world interface used by this binding.

        Example:

        .. code-block:: python

            from isaacsim.robot_motion.experimental.motion_generation import (
                ObstacleStrategy,
                TrackableApi,
                WorldBinding,
            )
            from isaacsim.robot_motion.experimental.motion_generation.tests.mirror_world_interface import (
                MirrorWorldInterface,
            )

            world_binding = WorldBinding(
                world_interface=MirrorWorldInterface(),
                obstacle_strategy=ObstacleStrategy(),
                tracked_prims=[],
                tracked_collision_api=TrackableApi.PHYSICS_COLLISION,
            )
            world_interface = world_binding.get_world_interface()
        """
        return self._world_interface
