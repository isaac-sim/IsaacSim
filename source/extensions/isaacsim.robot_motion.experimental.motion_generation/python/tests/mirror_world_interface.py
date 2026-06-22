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

"""Mirror motion-generation collision objects in memory for tests.

WorldBinding tests use this ``WorldInterface`` implementation to verify that
USD geometry, poses, scales, safety tolerances, and enabled flags are translated
correctly without invoking a planner backend.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Any, Literal

import warp as wp
from isaacsim.robot_motion.experimental.motion_generation import WorldInterface


@dataclass
class MirrorSphere:
    """A dataclass representing a spherical collision object for motion generation.

    Args:
        radius: The radius of the sphere.
        scale: The scale factor applied to the sphere.
        safety_tolerance: The safety tolerance for collision detection.
        pose: The position and orientation of the sphere as a tuple of (position, quaternion).
        enabled: Whether the sphere is enabled for collision detection.
    """

    radius: Any
    scale: Any
    safety_tolerance: Any
    pose: tuple[Any, Any]
    enabled: Any


@dataclass
class MirrorCube:
    """A dataclass representing a cube collision object in the mirror world interface.

    Args:
        size: The size dimensions of the cube.
        scale: The scale factor applied to the cube.
        safety_tolerance: The safety tolerance margin for collision detection.
        pose: The pose of the cube as a tuple of position and quaternion.
        enabled: Whether the cube collision object is enabled.
    """

    size: Any
    scale: Any
    safety_tolerance: Any
    pose: tuple[Any, Any]
    enabled: Any


@dataclass
class MirrorCone:
    """A dataclass for representing a cone-shaped mirror object in motion generation.

    Args:
        axis: The axis orientation of the cone.
        radius: The radius of the cone base.
        length: The length of the cone along its axis.
        scale: Scale factor applied to the cone geometry.
        safety_tolerance: Safety tolerance for collision detection.
        pose: Position and orientation as a tuple of (position, quaternion).
        enabled: Whether the cone is enabled for collision detection.
    """

    axis: Any
    radius: Any
    length: Any
    scale: Any
    safety_tolerance: Any
    pose: tuple[Any, Any]
    enabled: Any


@dataclass
class MirrorPlane:
    """A dataclass representing a plane collision object for motion generation.

    Args:
        axis: The orientation axis of the plane.
        length: The length dimension of the plane.
        width: The width dimension of the plane.
        scale: The scaling factor applied to the plane.
        safety_tolerance: The safety buffer distance around the plane.
        pose: The position and orientation of the plane as a tuple of (position, quaternion).
        enabled: Whether the plane collision detection is enabled.
    """

    axis: Any
    length: Any
    width: Any
    scale: Any
    safety_tolerance: Any
    pose: tuple[Any, Any]
    enabled: Any


@dataclass
class MirrorCapsule:
    """Represents a capsule collision geometry for motion generation.

    Args:
        axis: The axis along which the capsule is oriented.
        radius: The radius of the capsule.
        length: The length of the capsule.
        scale: The scale factor applied to the capsule.
        safety_tolerance: The safety tolerance for collision detection.
        pose: The pose of the capsule as a tuple of position and orientation.
        enabled: Whether the capsule is enabled for collision detection.
    """

    axis: Any
    radius: Any
    length: Any
    scale: Any
    safety_tolerance: Any
    pose: tuple[Any, Any]
    enabled: Any


@dataclass
class MirrorCylinder:
    """A dataclass representing a cylindrical collision object for motion generation.

    Args:
        axis: The axis orientation of the cylinder.
        radius: The radius of the cylinder.
        length: The length of the cylinder.
        scale: The scaling factor applied to the cylinder.
        safety_tolerance: The safety tolerance distance for collision avoidance.
        pose: A tuple containing the position and quaternion representing the cylinder's pose.
        enabled: Whether the cylinder collision object is enabled.
    """

    axis: Any
    radius: Any
    length: Any
    scale: Any
    safety_tolerance: Any
    pose: tuple[Any, Any]
    enabled: Any


@dataclass
class MirrorMesh:
    """Dataclass representing a mesh collision object for motion generation.

    Stores mesh geometry data including vertex positions, face connectivity, normals, and transformation properties for collision detection and avoidance in robot motion planning.

    Args:
        points: Vertex positions of the mesh.
        face_vertex_indices: Indices connecting vertices to form faces.
        face_vertex_counts: Number of vertices per face.
        normals: Normal vectors for the mesh faces.
        scale: Scaling factor applied to the mesh.
        safety_tolerance: Additional safety margin around the mesh for collision avoidance.
        pose: Position and orientation as a tuple of (position, quaternion).
        enabled: Whether the mesh collision object is active.
    """

    points: Any
    face_vertex_indices: Any
    face_vertex_counts: Any
    normals: Any
    scale: Any
    safety_tolerance: Any
    pose: tuple[Any, Any]
    enabled: Any


@dataclass
class MirrorTriangulatedMesh:
    """Data structure representing a triangulated mesh for collision detection.

    This dataclass stores the geometric properties and configuration of a triangulated mesh,
    including vertex data, triangle indices, scaling factors, safety parameters, pose information,
    and enablement state.

    Args:
        points: Vertex coordinates of the triangulated mesh.
        face_vertex_indices: Triangle vertex indices defining the mesh faces.
        scale: Scaling factor applied to the mesh geometry.
        safety_tolerance: Safety margin added around the mesh for collision detection.
        pose: Position and orientation of the mesh as (position, quaternion).
        enabled: Whether collision detection is enabled for this mesh.
    """

    points: Any
    face_vertex_indices: Any
    scale: Any
    safety_tolerance: Any
    pose: tuple[Any, Any]
    enabled: Any


@dataclass
class MirrorOrientedBoundingBox:
    """A dataclass representing an oriented bounding box collision object.

    Args:
        center: Center point of the bounding box.
        rotation: Quaternion (w, x, y, z) defining the orientation of the bounding box.
        half_side_length: Half the side length dimensions of the bounding box.
        scale: Scale factor applied to the bounding box.
        safety_tolerance: Safety tolerance margin for collision detection.
        pose: Tuple containing position and quaternion for the bounding box pose.
        enabled: Whether the bounding box collision detection is enabled.
    """

    center: Any
    rotation: Any
    half_side_length: Any
    scale: Any
    safety_tolerance: Any
    pose: tuple[Any, Any]
    enabled: Any


class MirrorWorldInterface(WorldInterface):
    """A mirror implementation of the WorldInterface for collision object management.

    This class provides an in-memory representation of collision objects without directly interfacing with
    a USD stage or simulation environment. It stores collision geometry data in Python data structures,
    making it useful for testing, debugging, or scenarios where you need to track collision objects
    without the overhead of a full simulation.

    The class maintains a dictionary of collision objects indexed by their primitive paths, supporting
    various geometric primitives including spheres, cubes, cones, planes, capsules, cylinders, meshes,
    triangulated meshes, and oriented bounding boxes. Each collision object stores its geometric
    properties, transformation data, scaling factors, safety tolerances, and enabled state.

    All geometric data from Warp arrays is converted to NumPy format for storage, allowing for easy
    inspection and manipulation of the collision object properties. The interface supports both adding
    new collision objects and updating existing ones through dedicated methods for each geometry type.

    This implementation is particularly useful for motion planning applications where you need to
    maintain a lightweight representation of the collision environment that can be easily queried
    and modified without the complexity of managing USD prims or physics simulation state.
    """

    def __init__(self) -> None:
        self.collision_objects: dict[str, Any] = {}

    def add_spheres(
        self,
        prim_paths: list[str],
        radii: wp.array,
        scales: wp.array,
        safety_tolerances: wp.array,
        poses: tuple[wp.array, wp.array],
        enabled_array: wp.array,
    ) -> None:
        """Adds sphere collision objects to the mirror world interface.

        Args:
            prim_paths: List of prim paths for the sphere objects.
            radii: Warp array of sphere radii.
            scales: Warp array of scale factors for the spheres.
            safety_tolerances: Warp array of safety tolerance values.
            poses: Tuple containing position and quaternion arrays for sphere poses.
            enabled_array: Warp array indicating which spheres are enabled.
        """
        radii_np = radii.numpy()
        scales_np = scales.numpy()
        positions_np = poses[0].numpy()
        quaternions_np = poses[1].numpy()
        enabled_np = enabled_array.numpy()
        safety_tolerances_np = safety_tolerances.numpy()
        for index, prim_path in enumerate(prim_paths):
            self.collision_objects[prim_path] = MirrorSphere(
                radius=radii_np[index],
                scale=scales_np[index],
                safety_tolerance=safety_tolerances_np[index],
                pose=(positions_np[index], quaternions_np[index]),
                enabled=enabled_np[index],
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
        """Adds cube collision objects to the mirror world interface.

        Args:
            prim_paths: List of prim paths for the cube objects.
            sizes: Warp array of cube sizes.
            scales: Warp array of scale factors for the cubes.
            safety_tolerances: Warp array of safety tolerance values.
            poses: Tuple containing position and quaternion arrays for cube poses.
            enabled_array: Warp array indicating which cubes are enabled.
        """
        sizes_np = sizes.numpy()
        scales_np = scales.numpy()
        positions_np = poses[0].numpy()
        quaternions_np = poses[1].numpy()
        enabled_np = enabled_array.numpy()
        safety_tolerances_np = safety_tolerances.numpy()
        for index, prim_path in enumerate(prim_paths):
            self.collision_objects[prim_path] = MirrorCube(
                size=sizes_np[index],
                scale=scales_np[index],
                safety_tolerance=safety_tolerances_np[index],
                pose=(positions_np[index], quaternions_np[index]),
                enabled=enabled_np[index],
            )

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
    ) -> None:
        """Adds cone collision objects to the mirror world interface.

        Args:
            prim_paths: List of prim paths for the cone objects.
            axes: List of cone axis orientations (X, Y, or Z).
            radii: Warp array of cone base radii.
            lengths: Warp array of cone heights.
            scales: Warp array of scale factors for the cones.
            safety_tolerances: Warp array of safety tolerance values.
            poses: Tuple containing position and quaternion arrays for cone poses.
            enabled_array: Warp array indicating which cones are enabled.
        """
        radii_np = radii.numpy()
        lengths_np = lengths.numpy()
        scales_np = scales.numpy()
        positions_np = poses[0].numpy()
        quaternions_np = poses[1].numpy()
        enabled_np = enabled_array.numpy()
        safety_tolerances_np = safety_tolerances.numpy()
        for index, prim_path in enumerate(prim_paths):
            self.collision_objects[prim_path] = MirrorCone(
                axis=axes[index],
                radius=radii_np[index],
                length=lengths_np[index],
                scale=scales_np[index],
                safety_tolerance=safety_tolerances_np[index],
                pose=(positions_np[index], quaternions_np[index]),
                enabled=enabled_np[index],
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
        """Adds plane collision objects to the mirror world interface.

        Args:
            prim_paths: List of prim paths for the plane objects.
            axes: List of plane normal axis orientations (X, Y, or Z).
            lengths: Warp array of plane lengths.
            widths: Warp array of plane widths.
            scales: Warp array of scale factors for the planes.
            safety_tolerances: Warp array of safety tolerance values.
            poses: Tuple containing position and quaternion arrays for plane poses.
            enabled_array: Warp array indicating which planes are enabled.
        """
        lengths_np = lengths.numpy()
        widths_np = widths.numpy()
        scales_np = scales.numpy()
        positions_np = poses[0].numpy()
        quaternions_np = poses[1].numpy()
        enabled_np = enabled_array.numpy()
        safety_tolerances_np = safety_tolerances.numpy()
        for index, prim_path in enumerate(prim_paths):
            self.collision_objects[prim_path] = MirrorPlane(
                axis=axes[index],
                length=lengths_np[index],
                width=widths_np[index],
                scale=scales_np[index],
                safety_tolerance=safety_tolerances_np[index],
                pose=(positions_np[index], quaternions_np[index]),
                enabled=enabled_np[index],
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
        """Adds capsule collision objects to the mirror world interface.

        Args:
            prim_paths: List of prim paths for the capsule objects.
            axes: List of capsule axis orientations (X, Y, or Z).
            radii: Warp array of capsule radii.
            lengths: Warp array of capsule lengths.
            scales: Warp array of scale factors for the capsules.
            safety_tolerances: Warp array of safety tolerance values.
            poses: Tuple containing position and quaternion arrays for capsule poses.
            enabled_array: Warp array indicating which capsules are enabled.
        """
        radii_np = radii.numpy()
        lengths_np = lengths.numpy()
        scales_np = scales.numpy()
        positions_np = poses[0].numpy()
        quaternions_np = poses[1].numpy()
        enabled_np = enabled_array.numpy()
        safety_tolerances_np = safety_tolerances.numpy()
        for index, prim_path in enumerate(prim_paths):
            self.collision_objects[prim_path] = MirrorCapsule(
                axis=axes[index],
                radius=radii_np[index],
                length=lengths_np[index],
                scale=scales_np[index],
                safety_tolerance=safety_tolerances_np[index],
                pose=(positions_np[index], quaternions_np[index]),
                enabled=enabled_np[index],
            )

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
    ) -> None:
        """Adds cylinder collision objects to the mirror world interface.

        Args:
            prim_paths: List of prim paths for the cylinder objects.
            axes: List of cylinder axis orientations (X, Y, or Z).
            radii: Warp array of cylinder radii.
            lengths: Warp array of cylinder lengths.
            scales: Warp array of scale factors for the cylinders.
            safety_tolerances: Warp array of safety tolerance values.
            poses: Tuple containing position and quaternion arrays for cylinder poses.
            enabled_array: Warp array indicating which cylinders are enabled.
        """
        radii_np = radii.numpy()
        lengths_np = lengths.numpy()
        scales_np = scales.numpy()
        positions_np = poses[0].numpy()
        quaternions_np = poses[1].numpy()
        enabled_np = enabled_array.numpy()
        safety_tolerances_np = safety_tolerances.numpy()
        for index, prim_path in enumerate(prim_paths):
            self.collision_objects[prim_path] = MirrorCylinder(
                axis=axes[index],
                radius=radii_np[index],
                length=lengths_np[index],
                scale=scales_np[index],
                safety_tolerance=safety_tolerances_np[index],
                pose=(positions_np[index], quaternions_np[index]),
                enabled=enabled_np[index],
            )

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
    ) -> None:
        """Adds mesh collision objects to the mirror world interface.

        Args:
            prim_paths: List of prim paths for the mesh objects.
            points: List of Warp arrays containing vertex points for each mesh.
            face_vertex_indices: List of Warp arrays containing face vertex indices for each mesh.
            face_vertex_counts: List of Warp arrays containing vertex counts per face for each mesh.
            normals: List of Warp arrays containing vertex normals for each mesh.
            scales: Warp array of scale factors for the meshes.
            safety_tolerances: Warp array of safety tolerance values.
            poses: Tuple containing position and quaternion arrays for mesh poses.
            enabled_array: Warp array indicating which meshes are enabled.
        """
        scales_np = scales.numpy()
        positions_np = poses[0].numpy()
        quaternions_np = poses[1].numpy()
        enabled_np = enabled_array.numpy()
        safety_tolerances_np = safety_tolerances.numpy()
        for index, prim_path in enumerate(prim_paths):
            self.collision_objects[prim_path] = MirrorMesh(
                points=points[index],
                face_vertex_indices=face_vertex_indices[index],
                face_vertex_counts=face_vertex_counts[index],
                normals=normals[index],
                scale=scales_np[index],
                safety_tolerance=safety_tolerances_np[index],
                pose=(positions_np[index], quaternions_np[index]),
                enabled=enabled_np[index],
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
        """Adds triangulated mesh collision objects to the mirror world interface.

        Args:
            prim_paths: List of prim paths for the triangulated mesh objects.
            points: List of Warp arrays containing vertex points for each triangulated mesh.
            face_vertex_indices: List of Warp arrays containing triangle vertex indices for each mesh.
            scales: Warp array of scale factors for the meshes.
            safety_tolerances: Warp array of safety tolerance values.
            poses: Tuple containing position and quaternion arrays for mesh poses.
            enabled_array: Warp array indicating which meshes are enabled.
        """
        scales_np = scales.numpy()
        positions_np = poses[0].numpy()
        quaternions_np = poses[1].numpy()
        enabled_np = enabled_array.numpy()
        safety_tolerances_np = safety_tolerances.numpy()
        for index, prim_path in enumerate(prim_paths):
            self.collision_objects[prim_path] = MirrorTriangulatedMesh(
                points=points[index],
                face_vertex_indices=face_vertex_indices[index],
                scale=scales_np[index],
                safety_tolerance=safety_tolerances_np[index],
                pose=(positions_np[index], quaternions_np[index]),
                enabled=enabled_np[index],
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
        """Adds oriented bounding box collision objects to the mirror world interface.

        Args:
            prim_paths: List of prim paths for the oriented bounding box objects.
            centers: Warp array containing center points for each bounding box.
            rotations: Warp array containing quaternions (w, x, y, z) for each bounding box.
            half_side_lengths: Warp array containing half side lengths for each bounding box.
            scales: Warp array of scale factors for the bounding boxes.
            safety_tolerances: Warp array of safety tolerance values.
            poses: Tuple containing position and quaternion arrays for bounding box poses.
            enabled_array: Warp array indicating which bounding boxes are enabled.
        """
        scales_np = scales.numpy()
        positions_np = poses[0].numpy()
        quaternions_np = poses[1].numpy()
        enabled_np = enabled_array.numpy()
        safety_tolerances_np = safety_tolerances.numpy()
        centers_np = centers.numpy()
        rotations_np = rotations.numpy()
        half_side_lengths_np = half_side_lengths.numpy()
        for index, prim_path in enumerate(prim_paths):
            self.collision_objects[prim_path] = MirrorOrientedBoundingBox(
                center=centers_np[index],
                rotation=rotations_np[index],
                half_side_length=half_side_lengths_np[index],
                scale=scales_np[index],
                safety_tolerance=safety_tolerances_np[index],
                pose=(positions_np[index], quaternions_np[index]),
                enabled=enabled_np[index],
            )

    def update_obstacle_transforms(
        self,
        prim_paths: list[str],
        poses: tuple[wp.array, wp.array],
    ) -> None:
        """Updates the transform poses of existing obstacle collision objects.

        Args:
            prim_paths: List of prim paths for the obstacles to update.
            poses: Tuple containing position and quaternion arrays for the new poses.
        """
        positions_np = poses[0].numpy()
        quaternions_np = poses[1].numpy()
        for index, prim_path in enumerate(prim_paths):
            self.collision_objects[prim_path].pose = (positions_np[index], quaternions_np[index])

    def update_obstacle_enables(
        self,
        prim_paths: list[str],
        enabled_array: wp.array,
    ) -> None:
        """Updates the enabled state of collision objects in the mirror world.

        Args:
            prim_paths: List of prim paths identifying the collision objects to update.
            enabled_array: Warp array containing the enabled state for each object.
        """
        enabled_np = enabled_array.numpy()
        for index, prim_path in enumerate(prim_paths):
            self.collision_objects[prim_path].enabled = enabled_np[index]

    def update_obstacle_scales(
        self,
        prim_paths: list[str],
        scales: wp.array,
    ) -> None:
        """Updates the scale values of collision objects in the mirror world.

        Args:
            prim_paths: List of prim paths identifying the collision objects to update.
            scales: Warp array containing the scale values for each object.
        """
        scales_np = scales.numpy()
        for index, prim_path in enumerate(prim_paths):
            self.collision_objects[prim_path].scale = scales_np[index]

    def update_sphere_properties(
        self,
        prim_paths: list[str],
        radii: wp.array | None,
    ) -> None:
        """Updates the radius properties of sphere collision objects.

        Args:
            prim_paths: List of prim paths identifying the sphere objects to update.
            radii: Warp array containing the radius values. If None, radius values are not updated.
        """
        radii_np = radii.numpy() if radii is not None else None
        for index, prim_path in enumerate(prim_paths):
            if radii_np is not None:
                self.collision_objects[prim_path].radius = radii_np[index]

    def update_cube_properties(
        self,
        prim_paths: list[str],
        sizes: wp.array | None,
    ) -> None:
        """Updates the size properties of cube collision objects.

        Args:
            prim_paths: List of prim paths identifying the cube objects to update.
            sizes: Warp array containing the size values. If None, size values are not updated.
        """
        sizes_np = sizes.numpy() if sizes is not None else None
        for index, prim_path in enumerate(prim_paths):
            if sizes_np is not None:
                self.collision_objects[prim_path].size = sizes_np[index]

    def update_cone_properties(
        self,
        prim_paths: list[str],
        axes: list[Literal["X", "Y", "Z"]] | None,
        radii: wp.array | None,
        lengths: wp.array | None,
    ) -> None:
        """Updates the properties of cone collision objects.

        Args:
            prim_paths: List of prim paths identifying the cone objects to update.
            axes: List of axis orientations for each cone. If None, axis values are not updated.
            radii: Warp array containing the radius values. If None, radius values are not updated.
            lengths: Warp array containing the length values. If None, length values are not updated.
        """
        radii_np = radii.numpy() if radii is not None else None
        lengths_np = lengths.numpy() if lengths is not None else None
        for index, prim_path in enumerate(prim_paths):
            if axes is not None:
                self.collision_objects[prim_path].axis = axes[index]
            if radii_np is not None:
                self.collision_objects[prim_path].radius = radii_np[index]
            if lengths_np is not None:
                self.collision_objects[prim_path].length = lengths_np[index]

    def update_plane_properties(
        self,
        prim_paths: list[str],
        axes: list[Literal["X", "Y", "Z"]] | None,
        lengths: wp.array | None,
        widths: wp.array | None,
    ) -> None:
        """Updates the properties of plane collision objects.

        Args:
            prim_paths: List of prim paths identifying the plane objects to update.
            axes: List of axis orientations for each plane. If None, axis values are not updated.
            lengths: Warp array containing the length values. If None, length values are not updated.
            widths: Warp array containing the width values. If None, width values are not updated.
        """
        lengths_np = lengths.numpy() if lengths is not None else None
        widths_np = widths.numpy() if widths is not None else None
        for index, prim_path in enumerate(prim_paths):
            if axes is not None:
                self.collision_objects[prim_path].axis = axes[index]
            if lengths_np is not None:
                self.collision_objects[prim_path].length = lengths_np[index]
            if widths_np is not None:
                self.collision_objects[prim_path].width = widths_np[index]

    def update_capsule_properties(
        self,
        prim_paths: list[str],
        axes: list[Literal["X", "Y", "Z"]] | None,
        radii: wp.array | None,
        lengths: wp.array | None,
    ) -> None:
        """Updates the properties of capsule collision objects.

        Args:
            prim_paths: List of prim paths identifying the capsule objects to update.
            axes: List of axis orientations for each capsule. If None, axis values are not updated.
            radii: Warp array containing the radius values. If None, radius values are not updated.
            lengths: Warp array containing the length values. If None, length values are not updated.
        """
        radii_np = radii.numpy() if radii is not None else None
        lengths_np = lengths.numpy() if lengths is not None else None
        for index, prim_path in enumerate(prim_paths):
            if axes is not None:
                self.collision_objects[prim_path].axis = axes[index]
            if radii_np is not None:
                self.collision_objects[prim_path].radius = radii_np[index]
            if lengths_np is not None:
                self.collision_objects[prim_path].length = lengths_np[index]

    def update_cylinder_properties(
        self,
        prim_paths: list[str],
        axes: list[Literal["X", "Y", "Z"]] | None,
        radii: wp.array | None,
        lengths: wp.array | None,
    ) -> None:
        """Updates the properties of cylinder collision objects.

        Args:
            prim_paths: List of prim paths identifying the cylinder objects to update.
            axes: List of axis orientations for each cylinder. If None, axis values are not updated.
            radii: Warp array containing the radius values. If None, radius values are not updated.
            lengths: Warp array containing the length values. If None, length values are not updated.
        """
        radii_np = radii.numpy() if radii is not None else None
        lengths_np = lengths.numpy() if lengths is not None else None
        for index, prim_path in enumerate(prim_paths):
            if axes is not None:
                self.collision_objects[prim_path].axis = axes[index]
            if radii_np is not None:
                self.collision_objects[prim_path].radius = radii_np[index]
            if lengths_np is not None:
                self.collision_objects[prim_path].length = lengths_np[index]

    def update_mesh_properties(
        self,
        prim_paths: list[str],
        points: list[wp.array] | None,
        face_vertex_indices: list[wp.array] | None,
        face_vertex_counts: list[wp.array] | None,
        normals: list[wp.array] | None,
    ) -> None:
        """Updates the mesh properties of mesh collision objects.

        Args:
            prim_paths: List of prim paths identifying the mesh objects to update.
            points: List of Warp arrays containing vertex points for each mesh. If None, points are not updated.
            face_vertex_indices: List of Warp arrays containing face vertex indices for each mesh.
                If None, face vertex indices are not updated.
            face_vertex_counts: List of Warp arrays containing vertex counts per face for each mesh.
                If None, face vertex counts are not updated.
            normals: List of Warp arrays containing vertex normals for each mesh. If None, normals are not updated.
        """
        for index, prim_path in enumerate(prim_paths):
            if points is not None:
                self.collision_objects[prim_path].points = points[index]
            if face_vertex_indices is not None:
                self.collision_objects[prim_path].face_vertex_indices = face_vertex_indices[index]
            if face_vertex_counts is not None:
                self.collision_objects[prim_path].face_vertex_counts = face_vertex_counts[index]
            if normals is not None:
                self.collision_objects[prim_path].normals = normals[index]

    def update_triangulated_mesh_properties(
        self,
        prim_paths: list[str],
        points: list[wp.array] | None,
        face_vertex_indices: list[wp.array] | None,
    ) -> None:
        """Updates the mesh properties of triangulated mesh collision objects.

        Args:
            prim_paths: List of prim paths identifying the triangulated mesh objects to update.
            points: List of Warp arrays containing vertex points for each mesh. If None, points are not updated.
            face_vertex_indices: List of Warp arrays containing face vertex indices for each mesh.
                If None, face vertex indices are not updated.
        """
        for index, prim_path in enumerate(prim_paths):
            if points is not None:
                self.collision_objects[prim_path].points = points[index]
            if face_vertex_indices is not None:
                self.collision_objects[prim_path].face_vertex_indices = face_vertex_indices[index]

    def update_oriented_bounding_box_properties(
        self,
        prim_paths: list[str],
        centers: wp.array | None,
        rotations: wp.array | None,
        half_side_lengths: wp.array | None,
    ) -> None:
        """Updates the properties of oriented bounding box collision objects.

        Args:
            prim_paths: List of prim paths identifying the oriented bounding box objects to update.
            centers: Warp array containing center positions for each box. If None, centers are not updated.
            rotations: Warp array containing rotation values for each box. If None, rotations are not updated.
            half_side_lengths: Warp array containing half side lengths for each box.
                If None, half side lengths are not updated.
        """
        centers_np = centers.numpy() if centers is not None else None
        rotations_np = rotations.numpy() if rotations is not None else None
        half_side_lengths_np = half_side_lengths.numpy() if half_side_lengths is not None else None
        for index, prim_path in enumerate(prim_paths):
            if centers_np is not None:
                self.collision_objects[prim_path].center = centers_np[index]
            if rotations_np is not None:
                self.collision_objects[prim_path].rotation = rotations_np[index]
            if half_side_lengths_np is not None:
                self.collision_objects[prim_path].half_side_length = half_side_lengths_np[index]
