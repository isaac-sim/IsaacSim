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

"""Abstract interface for planning world implementations with collision obstacle management."""

from __future__ import annotations

from typing import Literal, NoReturn

import warp as wp


class WorldInterface:
    """Abstract interface for planning world implementations.

    This class defines the contract that planning world implementations must
    fulfill to work with WorldBinding. Subclasses should override all methods
    to provide actual collision world functionality.

    Methods are organized into three categories:
    - Add methods: Initialize obstacle representations in the planning world.
    - Update transform methods: Update poses, enables, and scales for existing obstacles.
    - Update property methods: Update shape-specific properties like radii, sizes, etc.
    """

    #######################################################
    ## Add many objects to the planning world in parallel:
    #######################################################

    def add_spheres(
        self,
        prim_paths: list[str],
        radii: wp.array,
        scales: wp.array,
        safety_tolerances: wp.array,
        poses: tuple[wp.array, wp.array],
        enabled_array: wp.array,
    ) -> NoReturn:
        """Add sphere obstacles to the planning world.

        Args:
            prim_paths: USD prim paths for each sphere.
            radii: Sphere radii.
            scales: Local scale factors for each sphere.
            safety_tolerances: Safety margin around each sphere.
            poses: Tuple of (positions, orientations) arrays.
            enabled_array: Collision enabled state for each sphere.

        Raises:
            NotImplementedError: If not overridden by subclass.
        """
        raise NotImplementedError("add_spheres not implemented by the planning world child class.")

    def add_cubes(
        self,
        prim_paths: list[str],
        sizes: wp.array,
        scales: wp.array,
        safety_tolerances: wp.array,
        poses: tuple[wp.array, wp.array],
        enabled_array: wp.array,
    ) -> NoReturn:
        """Add cube obstacles to the planning world.

        Args:
            prim_paths: USD prim paths for each cube.
            sizes: Cube side lengths.
            scales: Local scale factors for each cube.
            safety_tolerances: Safety margin around each cube.
            poses: Tuple of (positions, orientations) arrays.
            enabled_array: Collision enabled state for each cube.

        Raises:
            NotImplementedError: If not overridden by subclass.
        """
        raise NotImplementedError("add_cubes not implemented by the planning world child class.")

    def add_cones(
        self,
        prim_paths: list[str],
        axes: list[Literal["X", "Y", "Z"]],
        radii: wp.array,
        lengths: wp.array,
        scales: wp.array,
        safety_tolerances: wp.array,
        poses: tuple[wp.array, wp.array],
        enabled_array: wp.array,
    ) -> NoReturn:
        """Add cone obstacles to the planning world.

        Args:
            prim_paths: USD prim paths for each cone.
            axes: Orientation axis for each cone.
            radii: Cone base radii.
            lengths: Cone heights.
            scales: Local scale factors for each cone.
            safety_tolerances: Safety margin around each cone.
            poses: Tuple of (positions, orientations) arrays.
            enabled_array: Collision enabled state for each cone.

        Raises:
            NotImplementedError: If not overridden by subclass.
        """
        raise NotImplementedError("add_cones not implemented by the planning world child class.")

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
    ) -> NoReturn:
        """Add plane obstacles to the planning world.

        Args:
            prim_paths: USD prim paths for each plane.
            axes: Normal axis for each plane.
            lengths: Plane lengths.
            widths: Plane widths.
            scales: Local scale factors for each plane.
            safety_tolerances: Safety margin around each plane.
            poses: Tuple of (positions, orientations) arrays.
            enabled_array: Collision enabled state for each plane.

        Raises:
            NotImplementedError: If not overridden by subclass.
        """
        raise NotImplementedError("add_planes not implemented by the planning world child class.")

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
    ) -> NoReturn:
        """Add capsule obstacles to the planning world.

        Args:
            prim_paths: USD prim paths for each capsule.
            axes: Orientation axis for each capsule.
            radii: Capsule radii.
            lengths: Capsule heights (excluding hemispherical caps).
            scales: Local scale factors for each capsule.
            safety_tolerances: Safety margin around each capsule.
            poses: Tuple of (positions, orientations) arrays.
            enabled_array: Collision enabled state for each capsule.

        Raises:
            NotImplementedError: If not overridden by subclass.
        """
        raise NotImplementedError("add_capsules not implemented by the planning world child class.")

    def add_cylinders(
        self,
        prim_paths: list[str],
        axes: list[Literal["X", "Y", "Z"]],
        radii: wp.array,
        lengths: wp.array,
        scales: wp.array,
        safety_tolerances: wp.array,
        poses: tuple[wp.array, wp.array],
        enabled_array: wp.array,
    ) -> NoReturn:
        """Add cylinder obstacles to the planning world.

        Args:
            prim_paths: USD prim paths for each cylinder.
            axes: Orientation axis for each cylinder.
            radii: Cylinder radii.
            lengths: Cylinder heights.
            scales: Local scale factors for each cylinder.
            safety_tolerances: Safety margin around each cylinder.
            poses: Tuple of (positions, orientations) arrays.
            enabled_array: Collision enabled state for each cylinder.

        Raises:
            NotImplementedError: If not overridden by subclass.
        """
        raise NotImplementedError("add_cylinders not implemented by the planning world child class.")

    def add_meshes(
        self,
        prim_paths: list[str],
        points: list[wp.array],
        face_vertex_indices: list[wp.array],
        face_vertex_counts: list[wp.array],
        normals: list[wp.array],
        scales: wp.array,
        safety_tolerances: wp.array,
        poses: tuple[wp.array, wp.array],
        enabled_array: wp.array,
    ) -> NoReturn:
        """Add mesh obstacles to the planning world.

        Args:
            prim_paths: USD prim paths for each mesh.
            points: Vertex positions for each mesh.
            face_vertex_indices: Face vertex indices for each mesh.
            face_vertex_counts: Number of vertices per face for each mesh.
            normals: Face normals for each mesh.
            scales: Local scale factors for each mesh.
            safety_tolerances: Safety margin around each mesh.
            poses: Tuple of (positions, orientations) arrays.
            enabled_array: Collision enabled state for each mesh.

        Raises:
            NotImplementedError: If not overridden by subclass.
        """
        raise NotImplementedError("add_meshes not implemented by the planning world child class.")

    def add_triangulated_meshes(
        self,
        prim_paths: list[str],
        points: list[wp.array],
        face_vertex_indices: list[wp.array],
        scales: wp.array,
        safety_tolerances: wp.array,
        poses: tuple[wp.array, wp.array],
        enabled_array: wp.array,
    ) -> NoReturn:
        """Add triangulated mesh obstacles to the planning world.

        Args:
            prim_paths: USD prim paths for each mesh.
            points: Vertex positions for each mesh.
            face_vertex_indices: Triangle vertex indices for each mesh.
            scales: Local scale factors for each mesh.
            safety_tolerances: Safety margin around each mesh.
            poses: Tuple of (positions, orientations) arrays.
            enabled_array: Collision enabled state for each mesh.

        Raises:
            NotImplementedError: If not overridden by subclass.
        """
        raise NotImplementedError("add_triangulated_meshes not implemented by the planning world child class.")

    # def add_convex_hull_meshes(
    #     self,
    #     prim_paths: list[str],
    #     points: list[wp.array],
    #     face_vertex_indices: list[wp.array],
    #     scales: wp.array,
    #     safety_tolerances: wp.array,
    #     poses: Tuple[wp.array, wp.array],
    #     enabled_array: wp.array,
    # ):
    #     raise NotImplementedError("add_convex_hull_meshes not implemented by the planning world child class.")

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
    ) -> NoReturn:
        """Add oriented bounding box obstacles to the planning world.

        Args:
            prim_paths: USD prim paths for each bounding box.
            centers: Local center positions for each bounding box.
            rotations: Local rotations as quaternions (w, x, y, z) for each bounding box.
            half_side_lengths: Half extents along each axis for each bounding box.
            scales: Local scale factors for each bounding box.
            safety_tolerances: Safety margin around each bounding box.
            poses: Tuple of (positions, orientations) arrays.
            enabled_array: Collision enabled state for each bounding box.

        Raises:
            NotImplementedError: If not overridden by subclass.
        """
        raise NotImplementedError("add_oriented_bounding_boxes not implemented by the planning world child class.")

    ####################################################
    ## Update RT properties of many prims in parallel:
    ####################################################

    def update_obstacle_transforms(
        self,
        prim_paths: list[str],
        poses: tuple[wp.array, wp.array],
    ) -> NoReturn:
        """Update world transforms for existing obstacles.

        Args:
            prim_paths: USD prim paths of obstacles to update.
            poses: Tuple of (positions, orientations) arrays.

        Raises:
            NotImplementedError: If not overridden by subclass.
        """
        raise NotImplementedError("update_obstacle_transforms not implemented by the planning world child class.")

    def update_obstacle_twists(
        self,
        prim_paths: list[str],
        poses: tuple[wp.array, wp.array],
    ) -> NoReturn:
        """Update twist velocities for existing obstacles.

        Args:
            prim_paths: USD prim paths of obstacles to update.
            poses: Tuple of (linear velocities, angular velocities) arrays.

        Raises:
            NotImplementedError: If not overridden by subclass.
        """
        raise NotImplementedError("update_obstacle_twists not implemented by the planning world child class.")

    def update_obstacle_enables(
        self,
        prim_paths: list[str],
        enabled_array: wp.array,
    ) -> NoReturn:
        """Update collision enabled state for existing obstacles.

        Args:
            prim_paths: USD prim paths of obstacles to update.
            enabled_array: New collision enabled states.

        Raises:
            NotImplementedError: If not overridden by subclass.
        """
        raise NotImplementedError("update_obstacle_enables not implemented by the planning world child class.")

    def update_obstacle_scales(
        self,
        prim_paths: list[str],
        scales: wp.array,
    ) -> NoReturn:
        """Update local scales for existing obstacles.

        Args:
            prim_paths: USD prim paths of obstacles to update.
            scales: New local scale factors.

        Raises:
            NotImplementedError: If not overridden by subclass.
        """
        raise NotImplementedError("update_obstacle_scales not implemented by the planning world child class.")

    ####################################################
    ## Update inherent properties of many prims in parallel:
    ####################################################

    def update_sphere_properties(
        self,
        prim_paths: list[str],
        radii: wp.array | None,
    ) -> NoReturn:
        """Update sphere-specific properties for existing obstacles.

        Args:
            prim_paths: USD prim paths of spheres to update.
            radii: New sphere radii. Pass None to skip updating.

        Raises:
            NotImplementedError: If not overridden by subclass.
        """
        raise NotImplementedError("update_sphere_properties not implemented by the planning world child class.")

    def update_cube_properties(
        self,
        prim_paths: list[str],
        sizes: wp.array | None,
    ) -> NoReturn:
        """Update cube-specific properties for existing obstacles.

        Args:
            prim_paths: USD prim paths of cubes to update.
            sizes: New cube side lengths. Pass None to skip updating.

        Raises:
            NotImplementedError: If not overridden by subclass.
        """
        raise NotImplementedError("update_cube_properties not implemented by the planning world child class.")

    def update_cone_properties(
        self,
        prim_paths: list[str],
        axes: list[Literal["X", "Y", "Z"]] | None,
        radii: wp.array | None,
        lengths: wp.array | None,
    ) -> NoReturn:
        """Update cone-specific properties for existing obstacles.

        Args:
            prim_paths: USD prim paths of cones to update.
            axes: New orientation axes. Pass None to skip updating.
            radii: New cone base radii. Pass None to skip updating.
            lengths: New cone heights. Pass None to skip updating.

        Raises:
            NotImplementedError: If not overridden by subclass.
        """
        raise NotImplementedError("update_cone_properties not implemented by the planning world child class.")

    def update_plane_properties(
        self,
        prim_paths: list[str],
        axes: list[Literal["X", "Y", "Z"]] | None,
        lengths: wp.array | None,
        widths: wp.array | None,
    ) -> NoReturn:
        """Update plane-specific properties for existing obstacles.

        Args:
            prim_paths: USD prim paths of planes to update.
            axes: New normal axes. Pass None to skip updating.
            lengths: New plane lengths. Pass None to skip updating.
            widths: New plane widths. Pass None to skip updating.

        Raises:
            NotImplementedError: If not overridden by subclass.
        """
        raise NotImplementedError("update_plane_properties not implemented by the planning world child class.")

    def update_capsule_properties(
        self,
        prim_paths: list[str],
        axes: list[Literal["X", "Y", "Z"]] | None,
        radii: wp.array | None,
        lengths: wp.array | None,
    ) -> NoReturn:
        """Update capsule-specific properties for existing obstacles.

        Args:
            prim_paths: USD prim paths of capsules to update.
            axes: New orientation axes. Pass None to skip updating.
            radii: New capsule radii. Pass None to skip updating.
            lengths: New capsule heights. Pass None to skip updating.

        Raises:
            NotImplementedError: If not overridden by subclass.
        """
        raise NotImplementedError("update_capsule_properties not implemented by the planning world child class.")

    def update_cylinder_properties(
        self,
        prim_paths: list[str],
        axes: list[Literal["X", "Y", "Z"]] | None,
        radii: wp.array | None,
        lengths: wp.array | None,
    ) -> NoReturn:
        """Update cylinder-specific properties for existing obstacles.

        Args:
            prim_paths: USD prim paths of cylinders to update.
            axes: New orientation axes. Pass None to skip updating.
            radii: New cylinder radii. Pass None to skip updating.
            lengths: New cylinder heights. Pass None to skip updating.

        Raises:
            NotImplementedError: If not overridden by subclass.
        """
        raise NotImplementedError("update_cylinder_properties not implemented by the planning world child class.")

    def update_mesh_properties(
        self,
        prim_paths: list[str],
        points: list[wp.array] | None,
        face_vertex_indices: list[wp.array] | None,
        face_vertex_counts: list[wp.array] | None,
        normals: list[wp.array] | None,
    ) -> NoReturn:
        """Update mesh-specific properties for existing obstacles.

        Args:
            prim_paths: USD prim paths of meshes to update.
            points: New vertex positions. Pass None to skip updating.
            face_vertex_indices: New face vertex indices. Pass None to skip updating.
            face_vertex_counts: New face vertex counts. Pass None to skip updating.
            normals: New face normals. Pass None to skip updating.

        Raises:
            NotImplementedError: If not overridden by subclass.
        """
        raise NotImplementedError("update_mesh_properties not implemented by the planning world child class.")

    def update_triangulated_mesh_properties(
        self,
        prim_paths: list[str],
        points: list[wp.array] | None,
        face_vertex_indices: list[wp.array] | None,
    ) -> NoReturn:
        """Update triangulated mesh-specific properties for existing obstacles.

        Args:
            prim_paths: USD prim paths of triangulated meshes to update.
            points: New vertex positions. Pass None to skip updating.
            face_vertex_indices: New triangle vertex indices. Pass None to skip updating.

        Raises:
            NotImplementedError: If not overridden by subclass.
        """
        raise NotImplementedError(
            "update_triangulated_mesh_properties not implemented by the planning world child class."
        )

    def update_oriented_bounding_box_properties(
        self,
        prim_paths: list[str],
        centers: wp.array | None,
        rotations: wp.array | None,
        half_side_lengths: wp.array | None,
    ) -> NoReturn:
        """Update oriented bounding box-specific properties for existing obstacles.

        Args:
            prim_paths: USD prim paths of bounding boxes to update.
            centers: New local center positions. Pass None to skip updating.
            rotations: New local rotations as quaternions (w, x, y, z). Pass None to skip updating.
            half_side_lengths: New half extents. Pass None to skip updating.

        Raises:
            NotImplementedError: If not overridden by subclass.
        """
        raise NotImplementedError(
            "update_oriented_bounding_box_properties not implemented by the planning world child class."
        )
