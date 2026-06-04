# SPDX-FileCopyrightText: Copyright (c) 2024-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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

"""Provide utility functions for object-based synthetic data generation."""

import random

import numpy as np
import omni.replicator.core as rep
from omni.kit.viewport.utility import get_active_viewport
from pxr import Gf, PhysxSchema, Usd, UsdGeom, UsdPhysics


def add_colliders(root_prim: Usd.Prim) -> None:
    """Enable collisions on the asset (without rigid body dynamics the asset will be static)."""
    # Iterate descendant prims (including root) and add colliders to mesh or primitive types
    for desc_prim in Usd.PrimRange(root_prim):
        if desc_prim.IsA(UsdGeom.Mesh) or desc_prim.IsA(UsdGeom.Gprim):
            # Physics
            if not desc_prim.HasAPI(UsdPhysics.CollisionAPI):
                collision_api = UsdPhysics.CollisionAPI.Apply(desc_prim)
            else:
                collision_api = UsdPhysics.CollisionAPI(desc_prim)
            collision_api.CreateCollisionEnabledAttr(True)
            # PhysX
            if not desc_prim.HasAPI(PhysxSchema.PhysxCollisionAPI):
                physx_collision_api = PhysxSchema.PhysxCollisionAPI.Apply(desc_prim)
            else:
                physx_collision_api = PhysxSchema.PhysxCollisionAPI(desc_prim)
            # Set PhysX specific properties
            physx_collision_api.CreateContactOffsetAttr(0.001)
            physx_collision_api.CreateRestOffsetAttr(0.0)

        # Add mesh specific collision properties only to mesh types
        if desc_prim.IsA(UsdGeom.Mesh):
            # Add mesh collision properties to the mesh (e.g. collider aproximation type)
            if not desc_prim.HasAPI(UsdPhysics.MeshCollisionAPI):
                mesh_collision_api = UsdPhysics.MeshCollisionAPI.Apply(desc_prim)
            else:
                mesh_collision_api = UsdPhysics.MeshCollisionAPI(desc_prim)
            mesh_collision_api.CreateApproximationAttr().Set("convexHull")


def create_collision_box_walls(
    stage: Usd.Stage,
    path: str,
    width: float,
    depth: float,
    height: float,
    thickness: float = 0.5,
    visible: bool = False,
) -> None:
    """Create a collision box area wrapping the given working area with origin at (0, 0, 0)."""
    # Define the walls (name, location, size) with thickness towards outside of the working area
    walls = [
        ("floor", (0, 0, (height + thickness) / -2.0), (width, depth, thickness)),
        ("ceiling", (0, 0, (height + thickness) / 2.0), (width, depth, thickness)),
        ("left_wall", ((width + thickness) / -2.0, 0, 0), (thickness, depth, height)),
        ("right_wall", ((width + thickness) / 2.0, 0, 0), (thickness, depth, height)),
        ("front_wall", (0, (depth + thickness) / 2.0, 0), (width, thickness, height)),
        ("back_wall", (0, (depth + thickness) / -2.0, 0), (width, thickness, height)),
    ]
    for name, location, size in walls:
        prim = stage.DefinePrim(f"{path}/{name}", "Cube")
        scale = (size[0] / 2.0, size[1] / 2.0, size[2] / 2.0)
        rep.functional.modify.pose(prim, position_value=location, scale_value=scale)
        add_colliders(prim)
        if not visible:
            UsdGeom.Imageable(prim).MakeInvisible()


def get_random_transform_values(
    loc_min: tuple[float, float, float] = (0, 0, 0),
    loc_max: tuple[float, float, float] = (1, 1, 1),
    rot_min: tuple[float, float, float] = (0, 0, 0),
    rot_max: tuple[float, float, float] = (360, 360, 360),
    scale_min_max: tuple[float, float] = (0.1, 1.0),
) -> tuple[tuple[float, float, float], tuple[float, float, float], tuple[float, float, float]]:
    """Create random transformation values for location, rotation, and scale."""
    location = (
        random.uniform(loc_min[0], loc_max[0]),
        random.uniform(loc_min[1], loc_max[1]),
        random.uniform(loc_min[2], loc_max[2]),
    )
    rotation = (
        random.uniform(rot_min[0], rot_max[0]),
        random.uniform(rot_min[1], rot_max[1]),
        random.uniform(rot_min[2], rot_max[2]),
    )
    scale = tuple([random.uniform(scale_min_max[0], scale_min_max[1])] * 3)
    return location, rotation, scale


def get_random_pose_on_sphere(
    origin: tuple[float, float, float],
    radius: float,
    camera_forward_axis: tuple[float, float, float] = (0, 0, -1),
) -> tuple[Gf.Vec3f, Gf.Quatf]:
    """Generate a random pose on a sphere looking at the origin."""
    origin = Gf.Vec3f(origin)
    camera_forward_axis = Gf.Vec3f(camera_forward_axis)

    # Generate random angles for spherical coordinates
    theta = np.random.uniform(0, 2 * np.pi)
    phi = np.arcsin(np.random.uniform(-1, 1))

    # Spherical to Cartesian conversion
    x = radius * np.cos(theta) * np.cos(phi)
    y = radius * np.sin(phi)
    z = radius * np.sin(theta) * np.cos(phi)

    location = origin + Gf.Vec3f(x, y, z)

    # Calculate direction vector from camera to look_at point
    direction = origin - location
    direction_normalized = direction.GetNormalized()

    # Calculate rotation from forward direction (rotateFrom) to direction vector (rotateTo)
    rotation = Gf.Rotation(Gf.Vec3d(camera_forward_axis), Gf.Vec3d(direction_normalized))
    orientation = Gf.Quatf(rotation.GetQuat())

    return location, orientation


def set_render_products_updates(render_products: list, enabled: bool, include_viewport: bool = False) -> None:
    """Enable or disable the render products and viewport rendering."""
    for rp in render_products:
        rp.hydra_texture.set_updates_enabled(enabled)
    if include_viewport:
        get_active_viewport().updates_enabled = enabled


def apply_velocities_towards_target(
    prims: list[Usd.Prim],
    target: tuple[float, float, float] = (0, 0, 0),
    strength_range: tuple[float, float] = (0.1, 1.0),
) -> None:
    """Apply velocities to prims directing them towards a target point."""
    for prim in prims:
        loc = prim.GetAttribute("xformOp:translate").Get()
        strength = random.uniform(strength_range[0], strength_range[1])
        velocity = ((target[0] - loc[0]) * strength, (target[1] - loc[1]) * strength, (target[2] - loc[2]) * strength)
        prim.GetAttribute("physics:velocity").Set(velocity)


def apply_random_velocities(
    prims: list[Usd.Prim],
    linear_range: tuple[float, float] = (-2.5, 2.5),
    angular_range: tuple[float, float] = (-45, 45),
) -> None:
    """Apply random linear and angular velocities to prims."""
    for prim in prims:
        lin_vel = (
            random.uniform(linear_range[0], linear_range[1]),
            random.uniform(linear_range[0], linear_range[1]),
            random.uniform(linear_range[0], linear_range[1]),
        )
        ang_vel = (
            random.uniform(angular_range[0], angular_range[1]),
            random.uniform(angular_range[0], angular_range[1]),
            random.uniform(angular_range[0], angular_range[1]),
        )
        prim.GetAttribute("physics:velocity").Set(lin_vel)
        prim.GetAttribute("physics:angularVelocity").Set(ang_vel)
