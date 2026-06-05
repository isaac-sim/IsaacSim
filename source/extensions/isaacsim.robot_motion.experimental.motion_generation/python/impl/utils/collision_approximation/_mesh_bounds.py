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

"""Utilities for computing bounding box approximations from mesh geometries."""

import numpy as np
import trimesh
from isaacsim.core.experimental.objects import Mesh
from isaacsim.core.experimental.utils.transform import quaternion_to_rotation_matrix, rotation_matrix_to_quaternion

from .bounding_geometries import AABB, OBB
from .triangulate_mesh import triangulate_mesh


def _core_mesh_to_trimesh(input_mesh: Mesh) -> trimesh.Trimesh:
    """Convert an Isaac Sim mesh into a Trimesh triangle mesh.

    Args:
        input_mesh: Mesh to convert.

    Returns:
        Trimesh triangle mesh created from the input mesh data.

    Raises:
        ValueError: Raised when the input mesh has no triangulated faces or points.
    """
    # triangulate the input mesh:
    triangulated_mesh_lists = triangulate_mesh(input_mesh)
    if len(triangulated_mesh_lists) < 1:
        raise ValueError("Input mesh has no triangulated meshes.")
    triangulated_mesh_indices = triangulated_mesh_lists[0]

    mesh_points_list = input_mesh.get_points(indices=0)
    if len(mesh_points_list) < 1:
        raise ValueError("Input mesh has no points.")

    points_array = np.reshape(mesh_points_list[0].numpy(), shape=(-1, 3))

    _trimesh = trimesh.Trimesh(vertices=points_array, faces=triangulated_mesh_indices, process=False)

    return _trimesh


def _clean_mesh(input_mesh: trimesh.Trimesh) -> None:
    """Clean an Trimesh triangle mesh in-place.

    Args:
        input_mesh: Mesh to clean.
    """
    input_mesh.remove_unreferenced_vertices()
    input_mesh.remove_infinite_values()
    return


def compute_obb_mesh(input_mesh: Mesh) -> OBB:
    """Compute an oriented bounding box from a mesh.

    Args:
        input_mesh: Mesh to compute the bounding box from.

    Returns:
        Oriented bounding box for the input mesh.

    Raises:
        ValueError: Raised when the input mesh has no triangulated faces or points.

    Example:

    .. code-block:: python

        >>> from isaacsim.core.experimental.objects import Mesh
        >>> from isaacsim.robot_motion.experimental.motion_generation.utils.collision_approximation import compute_obb_mesh
        >>> mesh = Mesh("/World/SomeMesh")
        >>> _ = compute_obb_mesh(mesh)  # doctest: +SKIP
    """
    _trimesh = _core_mesh_to_trimesh(input_mesh)

    # have to do some cleaning, to make these methods more robust to the quality
    # of input asset:
    _clean_mesh(_trimesh)

    # compute the mesh OBB:
    transform, extent = trimesh.bounds.oriented_bounds(_trimesh)

    # Extract rotation matrix from transform and convert to quaternion (w, x, y, z)
    rotation_matrix = transform[:3, :3]
    rotation_quaternion = rotation_matrix_to_quaternion(rotation_matrix).numpy()

    return OBB(rotation=rotation_quaternion, half_side_lengths=extent / 2.0, center=transform[:3, 3])


def compute_world_aabb_mesh(input_mesh: Mesh) -> AABB:
    """Compute a world-space axis-aligned bounding box from a mesh.

    Args:
        input_mesh: Mesh to compute the bounding box from.

    Returns:
        Axis-aligned bounding box in world coordinates.

    Raises:
        ValueError: Raised when the input mesh has no points or triangulated faces.

    Example:

    .. code-block:: python

        >>> from isaacsim.core.experimental.objects import Mesh
        >>> from isaacsim.robot_motion.experimental.motion_generation.utils.collision_approximation import compute_world_aabb_mesh
        >>> mesh = Mesh("/World/SomeMesh")
        >>> _ = compute_world_aabb_mesh(mesh)  # doctest: +SKIP
    """
    # First, apply the translation, rotation and scaling to the points.
    mesh_points_list = input_mesh.get_points(indices=0)
    if len(mesh_points_list) < 1:
        raise ValueError("Input mesh has no points.")
    points_vectors = mesh_points_list[0].numpy().T

    scale_matrix = np.diag(
        np.reshape(
            input_mesh.get_local_scales(indices=0).numpy(),
            shape=[
                3,
            ],
        )
    )
    position, quaternion = input_mesh.get_world_poses(indices=0)
    position_np = np.reshape(position.numpy(), shape=[3, 1])
    rotation_matrix_np = np.reshape(quaternion_to_rotation_matrix(quaternion).numpy(), shape=[3, 3])
    transformed_points = (position_np + rotation_matrix_np @ scale_matrix @ points_vectors).T

    triangulated_mesh_lists = triangulate_mesh(input_mesh)
    if len(triangulated_mesh_lists) < 1:
        raise ValueError("Input mesh has no triangulated meshes.")
    triangulated_mesh_indices = triangulated_mesh_lists[0]

    _trimesh = trimesh.Trimesh(
        vertices=transformed_points,
        faces=triangulated_mesh_indices,
        process=False,
    )

    # have to do some cleaning, to make these methods more robust to the quality
    # of input asset:
    _clean_mesh(_trimesh)

    aabb = _trimesh.bounds

    return AABB(
        min_bounds=np.reshape(
            aabb[0, :],
            shape=[
                3,
            ],
        ),
        max_bounds=np.reshape(
            aabb[1, :],
            shape=[
                3,
            ],
        ),
    )


# def compute_mesh_convex_hull(input_mesh: Mesh):
#     """Compute a convex hull from mesh geometry.

#     Args:
#         input_mesh: Mesh to compute the convex hull from.

#     Returns:
#         Convex hull representation for the input mesh.

#     Raises:
#         ValueError: Raised when the input mesh has no triangulated faces or points.

#     Example:

#     .. code-block:: python

#         >>> from isaacsim.core.experimental.objects import Mesh
#         >>> from isaacsim.robot_motion.experimental.motion_generation.utils.collision_approximation import compute_mesh_convex_hull
#         >>> mesh = Mesh("/World/SomeMesh")
#         >>> _ = compute_mesh_convex_hull(mesh)  # doctest: +SKIP
#     """
#     _trimesh = _core_mesh_to_trimesh(input_mesh)
#     _clean_mesh(_trimesh)
#     convex_hull: trimesh.Trimesh = trimesh.convex.convex_hull(_trimesh)

#     return ConvexHull(points=convex_hull.vertices, triangles=convex_hull.faces)
