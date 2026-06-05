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

"""cuMotion world interface for collision checking and motion planning in Isaac Sim environments."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Any, Literal

import cumotion
import isaacsim.robot_motion.experimental.motion_generation as mg
import numpy as np
import warp as wp
from isaacsim.core.experimental.materials import OmniPbrMaterial
from isaacsim.core.experimental.objects import Capsule, Cube, Sphere
from isaacsim.core.experimental.prims import XformPrim

from .utils import (
    ColliderBatchTransformOutput,
    batch_compute_collider_transforms,
    compute_collider_transforms_cpu,
    cumotion_to_isaac_sim_pose,
    isaac_sim_to_cumotion_pose,
)


@dataclass
class _CumotionCollider:
    """Collision geometry representation for cuMotion.

    Represents a single collision primitive in the cuMotion world. Note that a full
    parent object can be modeled by several smaller collision primitives. Each collider has its own
    transform relative to the parent object origin.

    Attributes:
        obstacle_handle: Handle to the collision obstacle in cuMotion's C++ memory.
        transform_object_to_collider: Transform from the parent object frame to this
            collider's frame. Defaults to identity.
        debug_prim_path: USD prim path for debug visualization. Defaults to None.

    Note:
        The world-to-collider transform is computed as:
        transform_world_to_collider = transform_world_to_object * transform_object_to_collider
    """

    obstacle_handle: cumotion.World.ObstacleHandle

    # Which indices are these particular transforms stored at?
    transform_object_to_collider_index: int

    # A prim path. This is only used if the user has enabled debugging visualizations.
    debug_prim_path: str | None = None


def _vectorize_cumotion_collider_data(
    cumotion_colliders: list[_CumotionCollider],
) -> tuple[list[cumotion.World.ObstacleHandle], list[str | None], int]:
    """Extract obstacle handles and debug prim paths from a list of _CumotionCollider objects.

    Args:
        cumotion_colliders: List of _CumotionCollider objects representing collision geometries.

    Returns:
        Tuple containing:
            - List of obstacle handles
            - List of debug prim paths (may contain None values)
            - Number of colliders
    """
    # The list of geometries which collectively model this obstacle:
    obstacle_handles = [c.obstacle_handle for c in cumotion_colliders]

    # The transforms from object frame --> collider geometry frame for each
    # geometry which model the obstacle.
    debug_prim_paths = [c.debug_prim_path for c in cumotion_colliders]
    n_colliders = len(cumotion_colliders)
    return obstacle_handles, debug_prim_paths, n_colliders


class CollisionData:
    """Container for collision data associated with a USD prim.

    Stores the transform and collision geometries for a single USD prim that has
    been added to the cuMotion world for collision checking.

    Args:
        transform_world_to_object_index: Index into the world-to-object transform array.
        obstacle_handle_starting_index: Starting index into the obstacle handles array.
        n_colliders: Number of collision primitives representing this object.
    """

    def __init__(
        self,
        transform_world_to_object_index: int,
        obstacle_handle_starting_index: int,
        n_colliders: int,
    ) -> None:
        # Store the indices of where the data for this object begins:
        self.transform_world_to_object_index = transform_world_to_object_index
        self.obstacle_handle_starting_index = obstacle_handle_starting_index
        self.n_colliders = n_colliders
        self.obstacle_handle_ending_index = obstacle_handle_starting_index + n_colliders


@dataclass
class _TransformBatchCache:
    """Pre-allocated warp arrays for hot-path batch transform computation.

    All fields are constant for a given set of prim paths and obstacle geometry.
    The cache is keyed on ``prim_paths_key``; it is rebuilt whenever the key changes
    (i.e. a different subset of prims is updated) or when the obstacle structure
    changes (e.g. a new obstacle is added via ``_extend_collider_data``).
    """

    prim_paths_key: tuple
    """Tuple of prim paths that this cache was built for."""
    transform_world_to_object_indices: np.ndarray
    """Indices into ``_position_world_to_objects`` for state-array updates, shape (N,)."""
    positions_obj_to_colliders_wp: wp.array
    """Object-to-collider position offsets as a warp array, shape (M, 3). Constant."""
    quaternions_obj_to_colliders_wp: wp.array
    """Object-to-collider quaternion offsets as a warp array, shape (M, 4). Constant."""
    collider_to_obj_indices_wp: wp.array
    """Per-collider index of the parent object, shape (M,). Constant."""
    positions_obj_to_colliders_np: np.ndarray
    """Host mirror of ``positions_obj_to_colliders_wp`` for the CPU compute path."""
    quaternions_obj_to_colliders_np: np.ndarray
    """Host mirror of ``quaternions_obj_to_colliders_wp`` for the CPU compute path."""
    collider_to_obj_indices_np: np.ndarray
    """Host mirror of ``collider_to_obj_indices_wp`` for the CPU compute path."""
    n_colliders_per_object: list
    """Number of colliders per object, length N. Constant."""
    obstacle_handles: list
    """Flat list of obstacle handles in collider order, length M. Constant."""
    output: ColliderBatchTransformOutput
    """Pre-allocated output struct; reused each frame to avoid per-frame warp allocs."""


class CumotionWorldInterface(mg.WorldInterface):
    """World interface for cuMotion collision checking and planning.

    This class provides the bridge between Isaac Sim's USD scene representation and
    cuMotion's collision world. It manages obstacle registration, updates, and
    coordinate frame transformations between Isaac Sim (world frame) and cuMotion
    (robot base frame).

    Args:
        world_to_robot_base: Transform from world frame to robot base frame. Defaults
            to None (identity transform). This is a tuple of position and quaternion,
            where the quaternion is in the form (w, x, y, z).
        visualize_debug_prims: Whether to create visual debug primitives for collision
            geometry. Defaults to False.
        visual_debug_enabled_prim_rgb: RGB color for enabled obstacle debug visualization.
            Defaults to None (red).
        visual_debug_disabled_prim_rgb: RGB color for disabled obstacle debug visualization.
            Defaults to None (green).
        visual_debug_prim_alpha: Alpha transparency for debug visualization. Defaults to 0.3.
        device: Device used by the interface for internally-allocated warp
            arrays and for the per-frame collider transform composition.
            Accepts the same values as :func:`wp.get_device` (``None``,
            ``"cpu"``, ``"cuda"``, ``"cuda:0"``, or a :class:`wp.Device`);
            ``None`` resolves to warp's current default device. CPU devices
            run the transform composition in vectorized NumPy on the host;
            CUDA devices run the Warp kernel path on the GPU.

    Attributes:
        world_view: cuMotion world view for collision queries.

    Example:

        .. code-block:: python

            world_interface = CumotionWorldInterface(
                visualize_debug_prims=True,
                visual_debug_enabled_prim_rgb=[0.0, 1.0, 0.0],
                visual_debug_prim_alpha=0.5,
            )
    """

    def __init__(
        self,
        world_to_robot_base: tuple[wp.array, wp.array] | None = None,
        visualize_debug_prims: bool = False,
        visual_debug_enabled_prim_rgb: list[float] | None = None,
        visual_debug_disabled_prim_rgb: list[float] | None = None,
        visual_debug_prim_alpha: float = 0.3,
        device: wp.DeviceLike = None,
    ) -> None:
        self._device: wp.Device = wp.get_device(device)

        self._world: cumotion.World = cumotion.create_world()
        self.__world_view = self._world.add_world_view()
        self.__world_inspector = cumotion.create_world_inspector(self.__world_view)
        # want to be able to manipulate our internal world objects according to changes
        # which occur to each prim:
        self._prim_path_to_collision_data: dict[str, CollisionData] = {}

        with wp.ScopedDevice(self._device):
            # Transform of the robot Base Frame --> World Frame. It is useful to store
            # in this way so we don't have to do as much matrix inversion. Assume identity.
            self._position_base_to_world: wp.array = wp.zeros(
                shape=[
                    3,
                ],
                dtype=wp.float32,
            )
            self._quaternion_base_to_world: wp.array = wp.array([1.0, 0.0, 0.0, 0.0], dtype=wp.float32)
            # Host mirrors of the base-to-world transform used by the CPU compute
            # path. Reset to None whenever the wp.arrays above are reassigned.
            self._position_base_to_world_np: np.ndarray | None = None
            self._quaternion_base_to_world_np: np.ndarray | None = None

            if world_to_robot_base is not None:
                world_to_robot_base_cumotion = isaac_sim_to_cumotion_pose(
                    world_to_robot_base[0], world_to_robot_base[1]
                )
                base_to_world_cumotion = world_to_robot_base_cumotion.inverse()
                self._position_base_to_world, self._quaternion_base_to_world = cumotion_to_isaac_sim_pose(
                    base_to_world_cumotion
                )

        # Memoized output of get_world_to_robot_base_transform; stays valid until
        # the base-frame pose is updated via _update_world_to_robot_root_transforms.
        self._cached_world_to_robot_base_transform: tuple[wp.array, wp.array] | None = None

        # Storing Transforms from world frame --> object frames.
        self._position_world_to_objects: np.ndarray = np.empty((0, 3))
        self._quaternion_world_to_objects: np.ndarray = np.empty((0, 4))

        # Storing Transforms from object frames --> collision geometry frames.
        self._position_objects_to_colliders: np.ndarray = np.empty((0, 3))
        self._quaternion_objects_to_colliders: np.ndarray = np.empty((0, 4))

        # storing all of the obstacle data in pre-built arrays:
        self._all_obstacle_handles: list[cumotion.World.ObstacleHandle] = []
        self._all_debug_prim_paths: list[str] = []
        self._n_colliders_in_each_object: np.ndarray = np.empty([0], dtype=np.int32)

        # Cache of pre-allocated warp arrays for _update_prim_world_to_object_transforms.
        # Set to None when the obstacle structure changes so it is rebuilt on next call.
        self._transform_batch_cache: _TransformBatchCache | None = None

        # if we want to do debug visuals, set those up here:
        # Do we want to draw debug sphere for mesh files?
        self._visualize_debug_prims = visualize_debug_prims

        if not visualize_debug_prims:
            return

        # Set up the "disabled obstacle" material
        if visual_debug_disabled_prim_rgb is None:
            visual_debug_disabled_prim_rgb = [0.0, 1.0, 0.0]
        self._visual_debug_disabled_prim_material = OmniPbrMaterial(
            paths="/CumotionDebug/DisabledMaterial",
        )
        self._visual_debug_disabled_prim_material.set_input_values(
            "diffuse_color_constant", visual_debug_disabled_prim_rgb
        )
        self._visual_debug_disabled_prim_material.set_input_values("enable_opacity", [True])
        self._visual_debug_disabled_prim_material.set_input_values("opacity_constant", [visual_debug_prim_alpha])

        # Set up the "enabled obstacle" material
        if visual_debug_enabled_prim_rgb is None:
            visual_debug_enabled_prim_rgb = [1.0, 0.0, 0.0]
        self._visual_debug_enabled_prim_material = OmniPbrMaterial(
            paths="/CumotionDebug/EnabledMaterial",
        )
        self._visual_debug_enabled_prim_material.set_input_values(
            "diffuse_color_constant", visual_debug_enabled_prim_rgb
        )
        self._visual_debug_enabled_prim_material.set_input_values("enable_opacity", [True])
        self._visual_debug_enabled_prim_material.set_input_values("opacity_constant", [visual_debug_prim_alpha])

    def _set_debug_material(self, debug_visual: Any, enabled: bool) -> None:
        """Set the debug material for a visual prim based on enabled state.

        Args:
            debug_visual: Visual prim object to apply material to.
            enabled: Whether the obstacle is enabled (True) or disabled (False).
        """
        if enabled:
            debug_visual.apply_visual_materials(self._visual_debug_enabled_prim_material)
            return
        debug_visual.apply_visual_materials(self._visual_debug_disabled_prim_material)

    @property
    def world_view(self) -> cumotion.WorldView:
        """Get the world_view associated with the cumotion.World object."""
        return self.__world_view

    def add_spheres(
        self,
        prim_paths: list[str],
        radii: wp.array,
        scales: wp.array,
        safety_tolerances: wp.array,
        poses: tuple[wp.array, wp.array],
        enabled_array: wp.array,
    ) -> None:
        """Add sphere collision obstacles to the cuMotion world.

        Args:
            prim_paths: USD prim paths for each sphere.
            radii: Sphere radii.
            scales: Local scale factors for each sphere.
            safety_tolerances: Safety margin around each sphere.
            poses: Tuple of (positions, orientations) arrays.
            enabled_array: Collision enabled state for each sphere.
        """
        positions, quaternions = poses
        positions_np = positions.numpy()
        quaternions_np = quaternions.numpy()
        scales_np = scales.numpy()
        radii_np = radii.numpy()
        safety_tolerances_np = safety_tolerances.numpy()
        enabled_np = enabled_array.numpy()

        if np.any(radii_np <= 0.0):
            raise ValueError(f"Radius values must be positive in cuMotion.")

        # For accuracy, we need to make sure that the scales are uniform. Otherwise,
        # these are ellipsoids, and should not be included in cumotion as a sphere.
        for scale in scales_np:
            if not np.allclose(scale, scale[0]) or np.any(scale <= 0.0):
                raise ValueError(f"Scale on sphere objects must be uniform and positive in cuMotion.")

        if not (
            len(prim_paths)
            == len(radii_np)
            == len(positions_np)
            == len(quaternions_np)
            == len(scales_np)
            == len(safety_tolerances_np)
            == len(enabled_np)
        ):
            raise ValueError(f"All input arrays must have the same length in cuMotion.")

        if len(prim_paths) == 0:
            raise ValueError(f"No prim paths provided to add_sphere call.")

        prim_set = set(prim_paths)
        if not prim_set.isdisjoint(set(self._prim_path_to_collision_data.keys())):
            raise ValueError(f"Tried to add a sphere with a prim path that already exists in the world.")

        if len(prim_paths) != len(prim_set):
            raise ValueError(f"Attempted to add duplicate prim paths in cuMotion.")

        for i in range(len(prim_paths)):
            prim_path = prim_paths[i]
            radius = radii_np[i]
            position = positions_np[i]
            quaternion = quaternions_np[i]
            scale = scales_np[i]
            safety_tolerance = safety_tolerances_np[i]
            enabled = enabled_np[i]

            # Makes sure that the entire sphere is always covered.
            radius = float(np.max(scale) * radius.item() + safety_tolerance.item())

            # use a sphere object:
            obstacle: cumotion.Obstacle = cumotion.create_obstacle(cumotion.Obstacle.Type.SPHERE)
            obstacle.set_attribute(cumotion.Obstacle.Attribute.RADIUS, radius)

            debug_prim_path = None
            if self._visualize_debug_prims:
                debug_prim_path = self._debug_collision_prim_name_generate(original_prim_name=prim_path, i_geometry=0)
                sphere = Sphere(paths=debug_prim_path, radii=radius)
                self._set_debug_material(sphere, enabled.item())

            transform_world_to_object_index = self._extend_world_to_objects_matrix(position, quaternion)

            # Set the obstacle in the world:
            obstacle_handle = self._world.add_obstacle(obstacle)
            collision_data = self._extend_collider_data(
                transform_world_to_object_index=transform_world_to_object_index,
                cumotion_colliders=[
                    _CumotionCollider(
                        obstacle_handle=obstacle_handle,
                        debug_prim_path=debug_prim_path,
                        transform_object_to_collider_index=self._extend_objects_to_colliders_matrix(
                            position=np.array([0.0, 0.0, 0.0]), quaternion=np.array([1.0, 0.0, 0.0, 0.0])
                        ),
                    )
                ],
            )
            self._prim_path_to_collision_data[prim_path] = collision_data

            # cuMotion will throw an error if you try to enable a collider
            # which is already enabled.
            if not enabled.item():
                self._update_prim_enabled_value(prim_path=prim_path, enabled=False)

        self._update_prim_world_to_object_transforms(
            prim_paths=prim_paths,
            poses=(positions, quaternions),
        )

    def add_cubes(
        self,
        prim_paths: list[str],
        sizes: wp.array,
        scales: wp.array,
        safety_tolerances: wp.array,
        poses: tuple[wp.array, wp.array],
        enabled_array: wp.array,
    ) -> None:
        """Add cuboid collision obstacles to the cuMotion world.

        Args:
            prim_paths: USD prim paths for each cube.
            sizes: Cube side lengths.
            scales: Local scale factors for each cube.
            safety_tolerances: Safety margin around each cube.
            poses: Tuple of (positions, orientations) arrays.
            enabled_array: Collision enabled state for each cube.
        """
        positions, quaternions = poses
        positions_np = positions.numpy()
        quaternions_np = quaternions.numpy()
        scales_np = scales.numpy()
        sizes_np = sizes.numpy()
        safety_tolerances_np = safety_tolerances.numpy()
        enabled_np = enabled_array.numpy()

        if np.any(sizes_np < 0.0):
            raise ValueError(f"Size values must be non-negative in cuMotion.")

        if np.any(scales_np <= 0.0):
            raise ValueError(f"Scale values for cube must be positive in cuMotion.")

        if not (
            len(prim_paths)
            == len(sizes_np)
            == len(positions_np)
            == len(quaternions_np)
            == len(scales_np)
            == len(safety_tolerances_np)
            == len(enabled_np)
        ):
            raise ValueError(f"All input arrays must have the same length in cuMotion.")

        if len(prim_paths) == 0:
            raise ValueError(f"No prim paths provided to add_cube call.")

        prim_set = set(prim_paths)
        if not prim_set.isdisjoint(set(self._prim_path_to_collision_data.keys())):
            raise ValueError(f"Tried to add a cube with a prim path that already exists in the world.")

        if len(prim_paths) != len(prim_set):
            raise ValueError(f"Attempted to add duplicate prim paths in cuMotion.")

        for i in range(len(prim_paths)):
            prim_path = prim_paths[i]
            size = sizes_np[i]
            position = positions_np[i]
            quaternion = quaternions_np[i]
            scale = scales_np[i]
            safety_tolerance = safety_tolerances_np[i]
            enabled = enabled_np[i]

            transform_world_to_object_index = self._extend_world_to_objects_matrix(position, quaternion)

            obstacle: cumotion.Obstacle = cumotion.create_obstacle(cumotion.Obstacle.Type.CUBOID)
            side_lengths = np.array(
                [
                    float(scale[0] * size.item() + safety_tolerance.item()),
                    float(scale[1] * size.item() + safety_tolerance.item()),
                    float(scale[2] * size.item() + safety_tolerance.item()),
                ]
            )
            obstacle.set_attribute(
                cumotion.Obstacle.Attribute.SIDE_LENGTHS,
                side_lengths,
            )

            debug_prim_path = None
            if self._visualize_debug_prims:
                # create the debug visual prim:
                debug_prim_path = self._debug_collision_prim_name_generate(
                    original_prim_name=prim_path,
                    i_geometry=0,
                )
                visual_cube = Cube(
                    paths=debug_prim_path,
                    sizes=1.0,
                    scales=side_lengths,
                )
                self._set_debug_material(visual_cube, enabled.item())

            # Set the obstacle in the world:
            obstacle_handle = self._world.add_obstacle(obstacle)
            collision_data = self._extend_collider_data(
                transform_world_to_object_index=transform_world_to_object_index,
                cumotion_colliders=[
                    _CumotionCollider(
                        obstacle_handle=obstacle_handle,
                        debug_prim_path=debug_prim_path,
                        transform_object_to_collider_index=self._extend_objects_to_colliders_matrix(
                            position=np.array([0.0, 0.0, 0.0]), quaternion=np.array([1.0, 0.0, 0.0, 0.0])
                        ),
                    )
                ],
            )
            self._prim_path_to_collision_data[prim_path] = collision_data

            # cuMotion will throw an error if you try to enable a collider
            # which is already enabled.
            if not enabled.item():
                self._update_prim_enabled_value(prim_path=prim_path, enabled=False)

        self._update_prim_world_to_object_transforms(
            prim_paths=prim_paths,
            poses=(positions, quaternions),
        )

    def add_triangulated_meshes(
        self,
        prim_paths: list[str],
        points: list[wp.array],
        face_vertex_indices: list[wp.array],
        scales: wp.array,
        safety_tolerances: wp.array,
        poses: tuple[wp.array, wp.array],
        enabled_array: wp.array,
    ) -> None:
        """Add triangulated mesh collision obstacles to the cuMotion world using sphere decomposition.

        Args:
            prim_paths: USD prim paths for each mesh.
            points: List of vertex position arrays for each mesh.
            face_vertex_indices: List of triangle index arrays for each mesh.
            scales: Local scale factors for each mesh.
            safety_tolerances: Safety margin around each mesh.
            poses: Tuple of (positions, orientations) arrays.
            enabled_array: Collision enabled state for each mesh.
        """

        def _to_cumotion_sphere_collider(
            prim_name: str, i_sphere: int, sphere: Any, enabled: bool
        ) -> _CumotionCollider:
            """Convert a collision sphere to a _CumotionCollider object.

            Args:
                prim_name: Name of the prim this collider belongs to.
                i_sphere: Index of this sphere within the mesh decomposition.
                sphere: Collision sphere object with radius and center attributes.
                enabled: Whether the collider should be enabled initially.

            Returns:
                _CumotionCollider object representing the sphere collider.
            """
            # use a sphere object:
            obstacle: cumotion.Obstacle = cumotion.create_obstacle(cumotion.Obstacle.Type.SPHERE)
            obstacle.set_attribute(cumotion.Obstacle.Attribute.RADIUS, sphere.radius)

            # Set the obstacle in the world:
            obstacle_handle = self._world.add_obstacle(obstacle)

            # Store the object-to-collider transform (sphere center offset)
            transform_object_to_collider_index = self._extend_objects_to_colliders_matrix(
                position=sphere.center, quaternion=np.array([1.0, 0.0, 0.0, 0.0])
            )

            # For debugging - we can optionally draw collision spheres:
            debug_prim_path = None
            if self._visualize_debug_prims:
                # Get a unique prim name for this sphere:
                debug_prim_path = self._debug_collision_prim_name_generate(
                    original_prim_name=prim_name,
                    i_geometry=i_sphere,
                )

                # Create the sphere:
                sphere_core_object = Sphere(paths=debug_prim_path)
                sphere_core_object.set_radii(sphere.radius, indices=0)
                self._set_debug_material(sphere_core_object, enabled)

            return _CumotionCollider(
                obstacle_handle=obstacle_handle,
                transform_object_to_collider_index=transform_object_to_collider_index,
                debug_prim_path=debug_prim_path,
            )

        positions, quaternions = poses
        positions_np = positions.numpy()
        quaternions_np = quaternions.numpy()
        enabled_array_np = enabled_array.numpy()
        safety_tolerances_np = safety_tolerances.numpy()
        scales_np = scales.numpy()

        if not (
            len(prim_paths)
            == len(points)
            == len(face_vertex_indices)
            == len(positions_np)
            == len(quaternions_np)
            == len(scales)
            == len(safety_tolerances_np)
            == len(enabled_array_np)
        ):
            raise ValueError(f"All input arrays must have the same length in cuMotion.")

        if len(prim_paths) == 0:
            raise ValueError(f"No prim paths provided to add_triangulated_meshes call.")

        prim_set = set(prim_paths)
        if not prim_set.isdisjoint(set(self._prim_path_to_collision_data.keys())):
            raise ValueError(f"Tried to add a triangulated mesh with a prim path that already exists in the world.")

        if len(prim_paths) != len(prim_set):
            raise ValueError(f"Attempted to add duplicate prim paths in cuMotion.")

        for i in range(len(prim_paths)):
            prim_path = prim_paths[i]
            points_array = points[i]
            face_vertex_indices_array = face_vertex_indices[i]
            position = positions_np[i]
            quaternion = quaternions_np[i]
            scale = scales_np[i]
            safety_tolerance = safety_tolerances_np[i]
            enabled = enabled_array_np[i]

            if points_array.shape[1] != 3:
                raise ValueError(
                    f"Points must have 3 elements (x, y, z) in add_triangulated_meshes call. Got {points_array.shape[1]} elements."
                )

            if face_vertex_indices_array.shape[1] != 3:
                raise ValueError(
                    f"Face vertex indices must have 3 elements (i1, i2, i3) in add_triangulated_meshes call. Got {face_vertex_indices_array.shape[1]} elements."
                )

            # Apply scaling to the input points:
            points_scaled = (np.diag(scale) @ points_array.numpy().T).T

            transform_world_to_object_index = self._extend_world_to_objects_matrix(position, quaternion)
            collision_spheres = cumotion.generate_collision_spheres(
                vertices=points_scaled,
                triangles=face_vertex_indices_array.numpy(),
                max_overshoot=safety_tolerance.item(),
            )

            collision_data = self._extend_collider_data(
                transform_world_to_object_index=transform_world_to_object_index,
                cumotion_colliders=[
                    _to_cumotion_sphere_collider(prim_path, i_sphere, sphere, enabled.item())
                    for i_sphere, sphere in enumerate(collision_spheres)
                ],
            )

            self._prim_path_to_collision_data[prim_path] = collision_data

            # cuMotion will throw an error if you try to enable a collider
            # which is already enabled.
            if not enabled.item():
                self._update_prim_enabled_value(prim_path=prim_path, enabled=False)

        self._update_prim_world_to_object_transforms(
            prim_paths=prim_paths,
            poses=(positions, quaternions),
        )

    def add_planes(
        self,
        prim_paths: list[str],
        axes: list[Literal["X", "Y", "Z"]],
        lengths: wp.array,
        widths: wp.array,
        scales: wp.array,
        safety_tolerances: wp.array,
        poses: tuple[wp.array, wp.array],
        enabled_array: wp.array,
    ) -> None:
        """Add plane collision obstacles to the cuMotion world as thin cuboids.

        Args:
            prim_paths: USD prim paths for each plane.
            axes: Plane normal axis for each plane.
            lengths: Plane lengths.
            widths: Plane widths.
            scales: Local scale factors for each plane.
            safety_tolerances: Safety margin around each plane.
            poses: Tuple of (positions, orientations) arrays.
            enabled_array: Collision enabled state for each plane.
        """
        positions, quaternions = poses
        positions_np = positions.numpy()
        quaternions_np = quaternions.numpy()
        enabled_array_np = enabled_array.numpy()
        safety_tolerances_np = safety_tolerances.numpy()
        scales_np = scales.numpy()
        if not (
            len(prim_paths)
            == len(axes)
            == len(positions_np)
            == len(quaternions_np)
            == len(scales_np)
            == len(safety_tolerances_np)
            == len(enabled_array_np)
        ):
            raise ValueError(f"All input arrays must have the same length in cuMotion.")

        if len(prim_paths) == 0:
            raise ValueError(f"No prim paths provided to add_planes call.")

        prim_set = set(prim_paths)
        if not prim_set.isdisjoint(set(self._prim_path_to_collision_data.keys())):
            raise ValueError(f"Tried to add a plane with a prim path that already exists in the world.")

        if len(prim_paths) != len(prim_set):
            raise ValueError(f"Attempted to add duplicate prim paths in cuMotion.")

        for i in range(len(prim_paths)):
            prim_path = prim_paths[i]
            axis = axes[i]
            position = positions_np[i]
            quaternion = quaternions_np[i]
            safety_tolerance = safety_tolerances_np[i]
            enabled = enabled_array_np[i]

            transform_world_to_object_index = self._extend_world_to_objects_matrix(position, quaternion)

            # This collider will be a cuboid type in cumotion:
            obstacle: cumotion.Obstacle = cumotion.create_obstacle(cumotion.Obstacle.Type.CUBOID)

            # TODO: a bit arbitrary?
            LARGE_NUMBER = 50_000.0
            SMALL_NUMBER = 1e-6 + safety_tolerance.item()

            side_lengths = np.array([LARGE_NUMBER, LARGE_NUMBER, SMALL_NUMBER])

            obstacle.set_attribute(
                cumotion.Obstacle.Attribute.SIDE_LENGTHS,
                side_lengths,
            )

            # Determine the rotation needed to align the capsule with the specified axis
            # cuMotion capsules are aligned along the Z-axis by default
            # If axis is "Z", use identity transform
            # If axis is "X", rotate 90 degrees about Y-axis
            # If axis is "Y", rotate 90 degrees about X-axis
            if axis == "Z":
                collider_position = np.array([0.0, 0.0, 0.0])
                collider_quaternion = np.array([1.0, 0.0, 0.0, 0.0])
            elif axis == "X":
                # 90 degree rotation about Y-axis: [cos(90/2), 0, sin(90/2), 0] (w, x, y, z)
                collider_position = np.array([0.0, 0.0, 0.0])
                collider_quaternion = np.array([np.cos(np.pi / 4), 0.0, np.sin(np.pi / 4), 0.0])
            elif axis == "Y":
                # 90 degree rotation about X-axis: [cos(90/2), sin(90/2), 0, 0] (w, x, y, z)
                collider_position = np.array([0.0, 0.0, 0.0])
                collider_quaternion = np.array([np.cos(np.pi / 4), np.sin(np.pi / 4), 0.0, 0.0])
            else:
                raise ValueError(f"Invalid axis: {axis}. Expected 'X', 'Y', or 'Z'.")

            debug_prim_path = None
            if self._visualize_debug_prims:
                debug_prim_path = self._debug_collision_prim_name_generate(original_prim_name=prim_path, i_geometry=0)
                visual_box = Cube(paths=debug_prim_path, sizes=1.0, scales=side_lengths)
                self._set_debug_material(visual_box, enabled.item())

            obstacle_handle = self._world.add_obstacle(obstacle)

            collision_data = self._extend_collider_data(
                transform_world_to_object_index=transform_world_to_object_index,
                cumotion_colliders=[
                    _CumotionCollider(
                        obstacle_handle=obstacle_handle,
                        transform_object_to_collider_index=self._extend_objects_to_colliders_matrix(
                            position=collider_position, quaternion=collider_quaternion
                        ),
                        debug_prim_path=debug_prim_path,
                    )
                ],
            )

            self._prim_path_to_collision_data[prim_path] = collision_data

            # cuMotion will throw an error if you try to enable a collider
            # which is already enabled.
            if not enabled.item():
                self._update_prim_enabled_value(
                    prim_path=prim_path,
                    enabled=False,
                )

        self._update_prim_world_to_object_transforms(
            prim_paths=prim_paths,
            poses=(positions, quaternions),
        )

    def add_capsules(
        self,
        prim_paths: list[str],
        axes: list[Literal["X", "Y", "Z"]],
        radii: wp.array,
        lengths: wp.array,
        scales: wp.array,
        safety_tolerances: wp.array,
        poses: tuple[wp.array, wp.array],
        enabled_array: wp.array,
    ) -> None:
        """Add capsule collision obstacles to the cuMotion world.

        Args:
            prim_paths: USD prim paths for each capsule.
            axes: Capsule axis for each capsule.
            radii: Capsule radii.
            lengths: Capsule lengths.
            scales: Local scale factors for each capsule.
            safety_tolerances: Safety margin around each capsule.
            poses: Tuple of (positions, orientations) arrays.
            enabled_array: Collision enabled state for each capsule.
        """
        positions, quaternions = poses
        positions_np = positions.numpy()
        quaternions_np = quaternions.numpy()
        enabled_array_np = enabled_array.numpy()
        safety_tolerances_np = safety_tolerances.numpy()
        scales_np = scales.numpy()
        radii_np = radii.numpy()
        lengths_np = lengths.numpy()

        # For accuracy, we need to make sure that the scales are uniform. Otherwise,
        # these would be ellipsoid capsules, which are not supported in cuMotion.
        for scale in scales_np:
            if not np.allclose(scale, scale[0]) or np.any(scale <= 0.0):
                raise ValueError(f"Scale on capsule objects must be uniform and positive in cuMotion.")

        if np.any(lengths_np <= 0.0):
            raise ValueError(f"Length values must be positive in cuMotion.")

        if np.any(radii_np <= 0.0):
            raise ValueError(f"Radius values must be positive in cuMotion.")

        if not (
            len(prim_paths)
            == len(axes)
            == len(radii_np)
            == len(lengths_np)
            == len(positions_np)
            == len(quaternions_np)
            == len(scales_np)
            == len(safety_tolerances_np)
            == len(enabled_array_np)
        ):
            raise ValueError(f"All input arrays must have the same length in cuMotion.")

        if len(prim_paths) == 0:
            raise ValueError(f"No prim paths provided to add_capsules call.")

        prim_set = set(prim_paths)
        if not prim_set.isdisjoint(set(self._prim_path_to_collision_data.keys())):
            raise ValueError(f"Tried to add a capsule with a prim path that already exists in the world.")

        if len(prim_paths) != len(prim_set):
            raise ValueError(f"Attempted to add duplicate prim paths in cuMotion.")

        for i in range(len(prim_paths)):
            prim_path = prim_paths[i]
            axis = axes[i]
            radius = radii_np[i]
            length = lengths_np[i]
            position = positions_np[i]
            quaternion = quaternions_np[i]
            scale = scales_np[i]
            safety_tolerance = safety_tolerances_np[i]
            enabled = enabled_array_np[i]

            transform_world_to_object_index = self._extend_world_to_objects_matrix(position, quaternion)

            # This collider will be a cylinder-type in cumotion (which is actually a capsule):
            obstacle: cumotion.Obstacle = cumotion.create_obstacle(cumotion.Obstacle.Type.CAPSULE)

            # If the scaling isn't uniform, we cannot treat this as a capsule.
            if not np.allclose(scale, scale[0]):
                raise ValueError(f"Scale on capsule objects must be uniform in cuMotion.")

            # Apply scale and safety tolerance to radius and height
            scaled_radius = float(scale[0] * radius.item() + safety_tolerance.item())
            scaled_height = float(scale[0] * length.item() + safety_tolerance.item())

            obstacle.set_attribute(cumotion.Obstacle.Attribute.RADIUS, scaled_radius)
            obstacle.set_attribute(cumotion.Obstacle.Attribute.HEIGHT, scaled_height)

            # Determine the rotation needed to align the capsule with the specified axis
            # cuMotion capsules are aligned along the Z-axis by default
            # If axis is "Z", use identity transform
            # If axis is "X", rotate 90 degrees about Y-axis
            # If axis is "Y", rotate 90 degrees about X-axis
            if axis == "Z":
                collider_position = np.array([0.0, 0.0, 0.0])
                collider_quaternion = np.array([1.0, 0.0, 0.0, 0.0])
            elif axis == "X":
                # 90 degree rotation about Y-axis: [cos(90/2), 0, sin(90/2), 0] (w, x, y, z)
                collider_position = np.array([0.0, 0.0, 0.0])
                collider_quaternion = np.array([np.cos(np.pi / 4), 0.0, np.sin(np.pi / 4), 0.0])
            elif axis == "Y":
                # 90 degree rotation about X-axis: [cos(90/2), sin(90/2), 0, 0] (w, x, y, z)
                collider_position = np.array([0.0, 0.0, 0.0])
                collider_quaternion = np.array([np.cos(np.pi / 4), np.sin(np.pi / 4), 0.0, 0.0])
            else:
                raise ValueError(f"Invalid axis: {axis}. Expected 'X', 'Y', or 'Z'.")

            debug_prim_path = None
            if self._visualize_debug_prims:
                debug_prim_path = self._debug_collision_prim_name_generate(original_prim_name=prim_path, i_geometry=0)
                visual_capsule = Capsule(
                    paths=debug_prim_path,
                    radii=scaled_radius,
                    heights=scaled_height,
                )
                self._set_debug_material(visual_capsule, enabled.item())

            obstacle_handle = self._world.add_obstacle(obstacle)

            collision_data = self._extend_collider_data(
                transform_world_to_object_index=transform_world_to_object_index,
                cumotion_colliders=[
                    _CumotionCollider(
                        obstacle_handle=obstacle_handle,
                        transform_object_to_collider_index=self._extend_objects_to_colliders_matrix(
                            position=collider_position, quaternion=collider_quaternion
                        ),
                        debug_prim_path=debug_prim_path,
                    )
                ],
            )

            self._prim_path_to_collision_data[prim_path] = collision_data

            # cuMotion will throw an error if you try to enable a collider
            # which is already enabled.
            if not enabled.item():
                self._update_prim_enabled_value(
                    prim_path=prim_path,
                    enabled=False,
                )

        self._update_prim_world_to_object_transforms(
            prim_paths=prim_paths,
            poses=(positions, quaternions),
        )

    def add_oriented_bounding_boxes(
        self,
        prim_paths: list[str],
        centers: wp.array,
        rotations: wp.array,
        half_side_lengths: wp.array,
        scales: wp.array,
        safety_tolerances: wp.array,
        poses: tuple[wp.array, wp.array],
        enabled_array: wp.array,
    ) -> None:
        """Add oriented bounding box collision obstacles to the cuMotion world as cuboids.

        Args:
            prim_paths: USD prim paths for each oriented bounding box.
            centers: OBB centers relative to the object frame.
            rotations: OBB rotation quaternions relative to the object frame.
            half_side_lengths: Half side lengths of each OBB.
            scales: Local scale factors for each OBB.
            safety_tolerances: Safety margin around each OBB.
            poses: Tuple of (positions, orientations) arrays.
            enabled_array: Collision enabled state for each OBB.
        """
        positions, quaternions = poses
        positions_np = positions.numpy()
        quaternions_np = quaternions.numpy()
        enabled_array_np = enabled_array.numpy()
        safety_tolerances_np = safety_tolerances.numpy()
        scales_np = scales.numpy()

        centers_np = centers.numpy()
        rotations_np = rotations.numpy()
        half_side_lengths_np = half_side_lengths.numpy()

        for scale in scales_np:
            if not np.allclose(scale, scale[0]) or np.any(scale <= 0.0):
                raise ValueError(f"Scale on oriented bounding boxes must be uniform and positive in cuMotion.")

        if np.any(half_side_lengths_np <= 0.0):
            raise ValueError(f"Half side length values must be positive in cuMotion.")

        if not (
            len(prim_paths)
            == positions_np.shape[0]
            == quaternions_np.shape[0]
            == centers_np.shape[0]
            == rotations_np.shape[0]
            == half_side_lengths_np.shape[0]
            == scales_np.shape[0]
            == safety_tolerances_np.shape[0]
            == enabled_array_np.shape[0]
        ):
            raise ValueError(f"All input arrays must have the same length in cuMotion.")

        if len(prim_paths) == 0:
            raise ValueError(f"No prim paths provided to add_oriented_bounding_boxes call.")

        prim_set = set(prim_paths)
        if not prim_set.isdisjoint(set(self._prim_path_to_collision_data.keys())):
            raise ValueError(
                f"Tried to add an oriented bounding box with a prim path that already exists in the world."
            )

        if len(prim_paths) != len(prim_set):
            raise ValueError(f"Attempted to add duplicate prim paths in cuMotion.")

        for i in range(len(prim_paths)):
            prim_path = prim_paths[i]
            position = positions_np[i]
            quaternion = quaternions_np[i]
            center = centers_np[i]
            rotation_quaternion = rotations_np[i]
            half_side_length = half_side_lengths_np[i]
            scale = scales_np[i]
            safety_tolerance = safety_tolerances_np[i]
            enabled = enabled_array_np[i]

            transform_world_to_object_index = self._extend_world_to_objects_matrix(position, quaternion)

            # This collider will be a cuboid-type in cumotion:
            obstacle: cumotion.Obstacle = cumotion.create_obstacle(cumotion.Obstacle.Type.CUBOID)
            side_lengths = np.array(
                [
                    float(scale[0] * 2 * half_side_length[0] + safety_tolerance.item()),
                    float(scale[1] * 2 * half_side_length[1] + safety_tolerance.item()),
                    float(scale[2] * 2 * half_side_length[2] + safety_tolerance.item()),
                ]
            )
            obstacle.set_attribute(cumotion.Obstacle.Attribute.SIDE_LENGTHS, side_lengths)

            # Store the constant offset between the OBB-frame and the object-frame:
            # The rotation is already a quaternion (w, x, y, z)
            collider_translation = scale * center

            debug_prim_path = None
            if self._visualize_debug_prims:
                debug_prim_path = self._debug_collision_prim_name_generate(original_prim_name=prim_path, i_geometry=0)
                visual_cube = Cube(
                    paths=debug_prim_path,
                    sizes=1.0,
                    scales=side_lengths,
                )
                self._set_debug_material(visual_cube, enabled.item())

            obstacle_handle = self._world.add_obstacle(obstacle)

            collision_data = self._extend_collider_data(
                transform_world_to_object_index=transform_world_to_object_index,
                cumotion_colliders=[
                    _CumotionCollider(
                        obstacle_handle=obstacle_handle,
                        transform_object_to_collider_index=self._extend_objects_to_colliders_matrix(
                            position=collider_translation, quaternion=rotation_quaternion
                        ),
                        debug_prim_path=debug_prim_path,
                    )
                ],
            )

            self._prim_path_to_collision_data[prim_path] = collision_data

            # cuMotion will throw an error if you try to enable a collider
            # which is already enabled.
            if not enabled.item():
                self._update_prim_enabled_value(
                    prim_path=prim_path,
                    enabled=False,
                )

        self._update_prim_world_to_object_transforms(
            prim_paths=prim_paths,
            poses=(positions, quaternions),
        )

    def update_obstacle_transforms(self, prim_paths: list[str], poses: tuple[wp.array, wp.array]) -> None:
        """Update the world-to-object transforms for tracked obstacles.

        Updates the poses of obstacles that have already been added to the world.

        Args:
            prim_paths: List of prim paths identifying the obstacles to update.
            poses: Tuple of (positions, quaternions) where positions has shape (N, 3)
                and quaternions has shape (N, 4) in (w, x, y, z) format.

        Raises:
            RuntimeError: If any prim paths are not currently tracked.
            ValueError: If input arrays have mismatched lengths or are empty.
        """
        if not self._validate_prim_paths(prim_paths):
            raise RuntimeError("Not all prims being updated are currently tracked by the cumotion world.")

        positions, quaternions = poses
        if not (len(prim_paths) == positions.shape[0] == quaternions.shape[0]):
            raise ValueError(f"All input arrays must have the same length in cuMotion.")

        if len(prim_paths) == 0:
            raise ValueError(f"No prim paths provided to update_obstacle_transforms call.")

        self._update_prim_world_to_object_transforms(prim_paths, poses)

    def update_obstacle_enables(self, prim_paths: list[str], enabled_array: wp.array) -> None:
        """Update the enabled/disabled state of tracked obstacles.

        Enables or disables collision checking for obstacles that have already been
        added to the world. Disabled obstacles are ignored during collision checking.

        Args:
            prim_paths: List of prim paths identifying the obstacles to update.
            enabled_array: Warp array of boolean values indicating enabled state,
                shape (N,) where N is the number of prim paths.

        Raises:
            RuntimeError: If any prim paths are not currently tracked.
            ValueError: If input arrays have mismatched lengths or are empty.
        """
        if not self._validate_prim_paths(prim_paths):
            raise RuntimeError("Not all prims being updated are currently tracked by the cumotion world.")

        if not (len(prim_paths) == enabled_array.shape[0]):
            raise ValueError(f"All input arrays must have the same length in cuMotion.")

        if len(prim_paths) == 0:
            raise ValueError(f"No prim paths provided to update_obstacle_enables call.")

        for i in range(len(prim_paths)):
            prim_path = prim_paths[i]
            enabled = enabled_array.numpy()[i].item()
            self._update_prim_enabled_value(prim_path, enabled)

    def get_world_to_robot_base_transform(self) -> tuple[wp.array, wp.array]:
        """Get the transform from world frame to robot base frame.

        Returns the current transform that maps coordinates from the Isaac Sim
        world frame to the cuMotion robot base frame. The result is memoized:
        the same ``wp.array`` objects are returned on every call until the
        base-frame pose is updated via ``update_world_to_robot_root_transforms``.

        Returns:
            Tuple of (position, quaternion) as warp arrays where position has shape (3,)
            and quaternion has shape (4,) in (w, x, y, z) format.
        """
        if self._cached_world_to_robot_base_transform is None:
            with wp.ScopedDevice(self._device):
                transform_base_to_world = isaac_sim_to_cumotion_pose(
                    self._position_base_to_world, self._quaternion_base_to_world
                )
                self._cached_world_to_robot_base_transform = cumotion_to_isaac_sim_pose(
                    transform_base_to_world.inverse()
                )
        return self._cached_world_to_robot_base_transform

    def update_world_to_robot_root_transforms(self, poses: tuple[wp.array, wp.array]) -> None:
        """Update the transform from world frame to robot base frame.

        cuMotion plans relative to the robot base frame. This method updates the base
        frame pose and recomputes all collider transforms in the base frame.

        Args:
            poses: Tuple of (positions, quaternions) where positions has shape (1, 3)
                and quaternions has shape (1, 4) in (w, x, y, z) format.

        Raises:
            ValueError: If the number of transforms is not exactly 1.
            ValueError: If positions don't have 3 elements.
            ValueError: If quaternions don't have 4 elements.
        """
        positions, quaternions = poses
        if not (positions.shape[0] == quaternions.shape[0] == 1):
            raise ValueError("cuMotion only works with a single robot.")

        if positions.shape[1] != 3:
            raise ValueError(
                f"Positions must have 3 elements (x, y, z) in update_world_to_robot_root_transforms call. Got {positions.shape[1]} elements."
            )

        if quaternions.shape[1] != 4:
            raise ValueError(
                f"Quaternions must have 4 elements (w, x, y, z) in update_world_to_robot_root_transforms call. Got {quaternions.shape[1]} elements."
            )

        self._update_world_to_robot_root_transforms(poses)

    def _rebuild_transform_cache(
        self,
        prim_paths: list[str],
        collision_data_array: list[CollisionData],
    ) -> None:
        """Build (or rebuild) the pre-allocated warp arrays used by the hot path.

        Called whenever ``prim_paths`` changes or the obstacle structure changes.
        All arrays that are constant for the lifetime of a given ``prim_paths`` set
        are converted to warp arrays here so that the hot path never needs to do it.

        Args:
            prim_paths: The ordered list of prim paths being updated.
            collision_data_array: Pre-resolved CollisionData for each prim path.
        """
        n_colliders_per_object = [cd.n_colliders for cd in collision_data_array]
        total_colliders = sum(n_colliders_per_object)

        # --- Assemble constant object-to-collider arrays from the global backing store ---
        position_objects_to_colliders = np.empty((total_colliders, 3), dtype=np.float32)
        quaternion_objects_to_colliders = np.empty((total_colliders, 4), dtype=np.float32)
        obstacle_handles: list = []
        collider_offset = 0
        for cd in collision_data_array:
            s = cd.obstacle_handle_starting_index
            e = cd.obstacle_handle_ending_index
            n = cd.n_colliders
            position_objects_to_colliders[collider_offset : collider_offset + n, :] = (
                self._position_objects_to_colliders[s:e, :]
            )
            quaternion_objects_to_colliders[collider_offset : collider_offset + n, :] = (
                self._quaternion_objects_to_colliders[s:e, :]
            )
            obstacle_handles.extend(self._all_obstacle_handles[s:e])
            collider_offset += n

        # --- Build the collider-to-object index mapping ---
        collider_to_obj_list: list[int] = []
        for obj_idx, n_col in enumerate(n_colliders_per_object):
            collider_to_obj_list.extend([obj_idx] * n_col)
        collider_to_obj_indices_np = np.asarray(collider_to_obj_list, dtype=np.int32)

        # --- Pre-compute world-to-object index array for state updates ---
        transform_world_to_object_indices = np.array(
            [cd.transform_world_to_object_index for cd in collision_data_array],
            dtype=np.int32,
        )

        # --- Convert constant data to warp arrays (done once per cache build) ---
        positions_obj_to_colliders_wp = wp.from_numpy(position_objects_to_colliders, dtype=wp.float32)
        quaternions_obj_to_colliders_wp = wp.from_numpy(quaternion_objects_to_colliders, dtype=wp.float32)
        collider_to_obj_indices_wp = wp.from_numpy(collider_to_obj_indices_np, dtype=wp.int32)

        # --- Pre-allocate output buffers (reused every frame, never reallocated unless size changes) ---
        output = ColliderBatchTransformOutput()
        output.positions_base_to_collider = wp.zeros((total_colliders, 3), dtype=wp.float32)
        output.quaternions_base_to_collider = wp.zeros((total_colliders, 4), dtype=wp.float32)
        output.positions_world_to_collider = wp.zeros((total_colliders, 3), dtype=wp.float32)
        output.quaternions_world_to_collider = wp.zeros((total_colliders, 4), dtype=wp.float32)

        self._transform_batch_cache = _TransformBatchCache(
            prim_paths_key=tuple(prim_paths),
            transform_world_to_object_indices=transform_world_to_object_indices,
            positions_obj_to_colliders_wp=positions_obj_to_colliders_wp,
            quaternions_obj_to_colliders_wp=quaternions_obj_to_colliders_wp,
            collider_to_obj_indices_wp=collider_to_obj_indices_wp,
            positions_obj_to_colliders_np=position_objects_to_colliders,
            quaternions_obj_to_colliders_np=quaternion_objects_to_colliders,
            collider_to_obj_indices_np=collider_to_obj_indices_np,
            n_colliders_per_object=n_colliders_per_object,
            obstacle_handles=obstacle_handles,
            output=output,
        )

    def _get_base_to_world_numpy(self) -> tuple[np.ndarray, np.ndarray]:
        """Return host-resident copies of the base-to-world transform.

        Memoized and invalidated whenever the base-frame transform is updated
        via ``_update_world_to_robot_root_transforms``.
        """
        if self._position_base_to_world_np is None:
            self._position_base_to_world_np = np.asarray(self._position_base_to_world.numpy(), dtype=np.float32)
        if self._quaternion_base_to_world_np is None:
            self._quaternion_base_to_world_np = np.asarray(self._quaternion_base_to_world.numpy(), dtype=np.float32)
        return self._position_base_to_world_np, self._quaternion_base_to_world_np

    def _update_prim_world_to_object_transforms(
        self,
        prim_paths: list[str],
        poses: tuple[wp.array, wp.array],
    ) -> None:
        """Update world-to-object transforms and recompute all collider poses.

        This is an internal method that updates the stored transforms and propagates
        the changes to all colliders. It recomputes collider poses in both base frame
        (for cuMotion collision checking) and world frame (for debug visualization if enabled).
        Input validation should be performed before calling.

        Dispatch is controlled by ``self._device``: CPU devices run the
        composition in vectorized NumPy on the host; CUDA devices run the
        Warp kernel path on the GPU.

        Args:
            prim_paths: List of prim paths identifying the obstacles to update.
            poses: Tuple of (positions, quaternions) where positions has shape (N, 3)
                and quaternions has shape (N, 4) in (w, x, y, z) format.
        """
        # SAFETY: the input data is validated before calling this function.
        collision_data_array = [self._prim_path_to_collision_data[p] for p in prim_paths]

        positions, quaternions = poses

        with wp.ScopedDevice(self._device):
            # Rebuild the cache when the prim_paths key changes or was invalidated.
            cache_key = tuple(prim_paths)
            if self._transform_batch_cache is None or self._transform_batch_cache.prim_paths_key != cache_key:
                self._rebuild_transform_cache(prim_paths, collision_data_array)
            cache = self._transform_batch_cache

            # Mirror the new world-to-object state on the host so that
            # _update_world_to_robot_root_transforms and the add_* methods can
            # read it without going back to the device.
            positions_np = positions.numpy()
            quaternions_np = quaternions.numpy()
            self._position_world_to_objects[cache.transform_world_to_object_indices, :] = positions_np
            self._quaternion_world_to_objects[cache.transform_world_to_object_indices, :] = quaternions_np

            if self._device.is_cpu:
                base_pos_np, base_quat_np = self._get_base_to_world_numpy()
                (
                    positions_base_to_colliders_np,
                    quaternions_base_to_colliders_np,
                    positions_world_to_colliders_np,
                    quaternions_world_to_colliders_np,
                ) = compute_collider_transforms_cpu(
                    position_base_to_world=base_pos_np,
                    quaternion_base_to_world=base_quat_np,
                    positions_world_to_object=positions_np.astype(np.float32, copy=False),
                    quaternions_world_to_object=quaternions_np.astype(np.float32, copy=False),
                    positions_object_to_collider=cache.positions_obj_to_colliders_np,
                    quaternions_object_to_collider=cache.quaternions_obj_to_colliders_np,
                    collider_to_object_indices=cache.collider_to_obj_indices_np,
                )
                if not self._visualize_debug_prims:
                    positions_world_to_colliders_np = None
                    quaternions_world_to_colliders_np = None
            else:
                # Pass pose inputs through verbatim when they already match the
                # interface device and dtype; otherwise marshal once via numpy.
                if positions.device == self._device and positions.dtype == wp.float32:
                    positions_kernel_input = positions
                else:
                    positions_kernel_input = wp.from_numpy(positions_np, dtype=wp.float32)
                if quaternions.device == self._device and quaternions.dtype == wp.float32:
                    quaternions_kernel_input = quaternions
                else:
                    quaternions_kernel_input = wp.from_numpy(quaternions_np, dtype=wp.float32)

                output_batch: ColliderBatchTransformOutput = batch_compute_collider_transforms(
                    position_base_to_world=self._position_base_to_world,
                    quaternion_base_to_world=self._quaternion_base_to_world,
                    positions_world_to_object=positions_kernel_input,
                    quaternions_world_to_object=quaternions_kernel_input,
                    positions_object_to_collider=cache.positions_obj_to_colliders_wp,
                    quaternions_object_to_collider=cache.quaternions_obj_to_colliders_wp,
                    num_colliders_per_object=cache.n_colliders_per_object,
                    collider_to_object_indices=cache.collider_to_obj_indices_wp,
                    output=cache.output,
                )

                positions_base_to_colliders_np = output_batch.positions_base_to_collider.numpy()
                quaternions_base_to_colliders_np = output_batch.quaternions_base_to_collider.numpy()
                # Same four-name contract as the CPU branch: world-frame arrays only when
                # debug visualization is enabled (avoids UnboundLocalError if this block is refactored).
                if self._visualize_debug_prims:
                    positions_world_to_colliders_np = output_batch.positions_world_to_collider.numpy()
                    quaternions_world_to_colliders_np = output_batch.quaternions_world_to_collider.numpy()
                else:
                    positions_world_to_colliders_np = None
                    quaternions_world_to_colliders_np = None

        for i, obstacle_handle in enumerate(cache.obstacle_handles):
            pose = cumotion.Pose3(
                rotation=cumotion.Rotation3(
                    quaternions_base_to_colliders_np[i, 0],
                    quaternions_base_to_colliders_np[i, 1],
                    quaternions_base_to_colliders_np[i, 2],
                    quaternions_base_to_colliders_np[i, 3],
                ),
                translation=positions_base_to_colliders_np[i, :],
            )
            self._world.set_pose(obstacle_handle, pose)

        if (
            self._visualize_debug_prims
            and positions_world_to_colliders_np is not None
            and quaternions_world_to_colliders_np is not None
        ):
            debug_prim_names: list[str] = []
            for cd in collision_data_array:
                debug_prim_names.extend(
                    self._all_debug_prim_paths[cd.obstacle_handle_starting_index : cd.obstacle_handle_ending_index]
                )
            xform_prim = XformPrim(paths=debug_prim_names)
            xform_prim.set_local_poses(
                translations=positions_world_to_colliders_np,
                orientations=quaternions_world_to_colliders_np,
            )

    def _update_world_to_robot_root_transforms(
        self,
        pose: tuple[wp.array, wp.array],
    ) -> None:
        """Update the robot base frame transform and recompute all collider poses.

        This is an internal method that updates the base frame transform and propagates
        the changes to all colliders. Input validation should be performed before calling.

        Args:
            pose: Tuple of (position, quaternion) where position has shape (1, 3)
                and quaternion has shape (1, 4) in (w, x, y, z) format.
        """
        # SAFETY: the input data is validated before calling this function.

        with wp.ScopedDevice(self._device):
            # Store the new values of the positions and quaternions:
            position, quaternion = pose
            pose_world_to_base_cumotion = isaac_sim_to_cumotion_pose(position, quaternion)
            self._position_base_to_world, self._quaternion_base_to_world = cumotion_to_isaac_sim_pose(
                pose_world_to_base_cumotion.inverse()
            )

            # Base transform changed; invalidate the memoized world<->base tuple
            # and the host mirrors of the base-frame transform.
            self._cached_world_to_robot_base_transform = None
            self._position_base_to_world_np = None
            self._quaternion_base_to_world_np = None

            # batch update all of the transforms (world --> collider frames):
            if self._device.is_cpu:
                base_pos_np, base_quat_np = self._get_base_to_world_numpy()
                collider_to_object_indices = np.repeat(
                    np.arange(len(self._n_colliders_in_each_object), dtype=np.int32),
                    self._n_colliders_in_each_object,
                )
                (
                    positions_base_to_colliders_np,
                    quaternions_base_to_colliders_np,
                    _positions_world_to_colliders_np,
                    _quaternions_world_to_colliders_np,
                ) = compute_collider_transforms_cpu(
                    position_base_to_world=base_pos_np,
                    quaternion_base_to_world=base_quat_np,
                    positions_world_to_object=self._position_world_to_objects.astype(np.float32, copy=False),
                    quaternions_world_to_object=self._quaternion_world_to_objects.astype(np.float32, copy=False),
                    positions_object_to_collider=self._position_objects_to_colliders.astype(np.float32, copy=False),
                    quaternions_object_to_collider=self._quaternion_objects_to_colliders.astype(np.float32, copy=False),
                    collider_to_object_indices=collider_to_object_indices,
                )
            else:
                output_batch: ColliderBatchTransformOutput = batch_compute_collider_transforms(
                    position_base_to_world=self._position_base_to_world,
                    quaternion_base_to_world=self._quaternion_base_to_world,
                    positions_world_to_object=wp.from_numpy(self._position_world_to_objects, dtype=wp.float32),
                    quaternions_world_to_object=wp.from_numpy(self._quaternion_world_to_objects, dtype=wp.float32),
                    positions_object_to_collider=wp.from_numpy(self._position_objects_to_colliders, dtype=wp.float32),
                    quaternions_object_to_collider=wp.from_numpy(
                        self._quaternion_objects_to_colliders, dtype=wp.float32
                    ),
                    num_colliders_per_object=self._n_colliders_in_each_object,
                )

                positions_base_to_colliders_np = output_batch.positions_base_to_collider.numpy()
                quaternions_base_to_colliders_np = output_batch.quaternions_base_to_collider.numpy()

        # Write these results to the cumotion colliders:
        for obstacle_handle, transform_index in zip(
            self._all_obstacle_handles, range(positions_base_to_colliders_np.shape[0])
        ):

            pose = cumotion.Pose3(
                rotation=cumotion.Rotation3(*quaternions_base_to_colliders_np[transform_index, :]),
                translation=positions_base_to_colliders_np[transform_index, :],
            )

            # set the base-frame pose:
            self._world.set_pose(obstacle_handle, pose)

    def _update_prim_enabled_value(self, prim_path: str, enabled: bool) -> None:
        """Update the enabled state for all colliders associated with a prim.

        Args:
            prim_path: Prim path identifying the obstacle.
            enabled: Whether to enable (True) or disable (False) the obstacle.
        """
        collision_data = self._prim_path_to_collision_data[prim_path]
        set_enable_function = self._world.enable_obstacle if enabled else self._world.disable_obstacle
        startidx = collision_data.obstacle_handle_starting_index
        endidx = collision_data.obstacle_handle_ending_index
        obstacle_handles = self._all_obstacle_handles[startidx:endidx]

        if self._visualize_debug_prims:
            debug_paths = self._all_debug_prim_paths[startidx:endidx]

        self.__world_view.update()

        for i in range(len(obstacle_handles)):
            obstacle_handle = obstacle_handles[i]
            if self.__world_inspector.is_enabled(obstacle_handle) == enabled:
                # nothing to do, we are already in this enabled state:
                continue
            set_enable_function(obstacle_handle)

            if self._visualize_debug_prims:
                self._set_debug_material(XformPrim(debug_paths[i]), enabled)

    def _debug_collision_prim_name_generate(self, original_prim_name: str, i_geometry: int) -> str:
        """Generate a unique debug prim path for collision visualization.

        Args:
            original_prim_name: Original prim path name.
            i_geometry: Index of the geometry part (for multi-part obstacles).

        Returns:
            Generated debug prim path in the format "/CumotionDebug/{original_name}/Part{i}".
        """
        if original_prim_name.startswith("/"):
            original_prim_name = original_prim_name[1:]
        altered_prim_name = f"/CumotionDebug/{original_prim_name}/Part{i_geometry}"
        return altered_prim_name

    def _validate_prim_paths(self, prim_paths: list[str]) -> bool:
        """Validate that all prim paths are currently tracked.

        Args:
            prim_paths: List of prim paths to validate.

        Returns:
            True if all prim paths are tracked, False otherwise.
        """
        if not set(prim_paths).issubset(self._prim_path_to_collision_data.keys()):
            return False
        return True

    def _extend_world_to_objects_matrix(self, position: np.ndarray, quaternion: np.ndarray) -> int:
        """Add a new world-to-object transform to the internal storage arrays.

        Args:
            position: Position array of shape (3,) or (1, 3).
            quaternion: Quaternion array of shape (4,) or (1, 4) in (w, x, y, z) format.

        Returns:
            Index where the transform is stored in the internal arrays.
        """
        self._position_world_to_objects = np.concatenate(
            [self._position_world_to_objects, np.reshape(position, shape=[-1, 3])]
        )

        self._quaternion_world_to_objects = np.concatenate(
            [self._quaternion_world_to_objects, np.reshape(quaternion, shape=[-1, 4])]
        )

        # Returns the index where this pose is stored:
        return len(self._position_world_to_objects) - 1

    def _extend_objects_to_colliders_matrix(self, position: np.ndarray, quaternion: np.ndarray) -> int:
        """Add a new object-to-collider transform to the internal storage arrays.

        Args:
            position: Position array of shape (3,) or (1, 3).
            quaternion: Quaternion array of shape (4,) or (1, 4) in (w, x, y, z) format.

        Returns:
            Index where the transform is stored in the internal arrays.
        """
        self._position_objects_to_colliders = np.concatenate(
            [self._position_objects_to_colliders, np.reshape(position, shape=[-1, 3])]
        )

        self._quaternion_objects_to_colliders = np.concatenate(
            [self._quaternion_objects_to_colliders, np.reshape(quaternion, shape=[-1, 4])]
        )

        # Returns the index where this pose is stored:
        return len(self._position_objects_to_colliders) - 1

    def _extend_collider_data(
        self, cumotion_colliders: list[_CumotionCollider], transform_world_to_object_index: int
    ) -> CollisionData:
        """Add collision data for a new object to the internal storage.

        Args:
            cumotion_colliders: List of _CumotionCollider objects representing the object's collision geometries.
            transform_world_to_object_index: Index of the world-to-object transform in the internal arrays.

        Returns:
            CollisionData object containing indices and metadata for the added object.
        """
        obstacle_handles, debug_prim_paths, n_colliders = _vectorize_cumotion_collider_data(cumotion_colliders)

        obstacle_handle_starting_index = len(self._all_obstacle_handles)

        self._all_obstacle_handles.extend(obstacle_handles)
        self._all_debug_prim_paths.extend(debug_prim_paths)

        self._n_colliders_in_each_object = np.append(self._n_colliders_in_each_object, n_colliders)

        # Invalidate the batch-transform cache: the obstacle structure has changed.
        self._transform_batch_cache = None

        collision_data = CollisionData(
            transform_world_to_object_index=transform_world_to_object_index,
            obstacle_handle_starting_index=obstacle_handle_starting_index,
            n_colliders=n_colliders,
        )

        return collision_data
