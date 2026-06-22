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

"""Complete example demonstrating scene interaction with the Motion Generation API.

This example shows how to use SceneQuery, ObstacleStrategy, WorldInterface, and WorldBinding
to connect USD scene objects to a motion planning library.

Note: The scene setup function creates a simple example scene, but in practice your scene
could come from anywhere - a USD file, procedurally generated, loaded from a database, etc.
"""

# ============================================================================
# 1. Launch Simulation App
# ============================================================================
# All Isaac Sim imports must come after SimulationApp is instantiated
from isaacsim import SimulationApp

simulation_app = SimulationApp({"headless": False})

from typing import Any

import isaacsim.core.experimental.utils.stage as stage_utils
import isaacsim.robot_motion.experimental.motion_generation as mg
import numpy as np

# Now we can import Isaac Sim modules
import omni.timeline
from isaacsim.core.experimental.objects import Cube, Mesh, Sphere
from isaacsim.core.experimental.prims import GeomPrim, RigidPrim
from isaacsim.core.rendering_manager import ViewportManager
from isaacsim.core.simulation_manager import SimulationManager
from isaacsim.robot_motion.experimental.motion_generation import WorldInterface


# ============================================================================
# 2. Scene Setup
# ============================================================================
# <start-setup-scene-snippet>
def setup_scene() -> None:
    """Create a simple example scene with obstacles.

    The scene is created from the default template which includes a ground plane, lights, etc.
    In practice, your scene could come from:
    - A USD file loaded via open_stage()
    - Procedurally generated content
    - A database or asset library
    - Any other source - the Motion Generation API doesn't care where the scene comes from
    """
    # Create a new stage from default template (includes ground plane, lights, etc.)
    stage_utils.create_new_stage(template="default stage")
    stage_utils.set_stage_units(meters_per_unit=1.0)

    # Add some example obstacles
    # These will be found by SceneQuery and synchronized via WorldBinding
    Cube("/World/Obstacle1", positions=[-1.0, 1.0, 0.6], sizes=0.4)
    Sphere("/World/Obstacle2", positions=[1.0, 0.0, 0.75], radii=0.2)

    # Apply collision APIs to obstacles so they can be found by SceneQuery
    GeomPrim(["/World/Obstacle1", "/World/Obstacle2"], apply_collision_apis=True)

    # Make obstacles rigid bodies so they fall under gravity
    RigidPrim(["/World/Obstacle1", "/World/Obstacle2"], masses=1.0)

    # Add two simple meshes
    # Mesh1: Will use default OBB representation with large safety tolerance
    # we will start with mesh with some initial orientation.
    angle = np.pi / 4
    Mesh(
        "/World/Mesh1",
        primitives="Cube",
        positions=[-1.5, -1.5, 0.5],
        scales=[0.5, 0.5, 0.1],
        # create quaternion:
        orientations=[np.cos(angle / 2), np.sin(angle / 2), 0.0, 0.0],
    )

    # Mesh2: Will be overridden to use TRIANGULATED_MESH with small safety tolerance
    # This mesh might need more faithful representation for interaction
    Mesh("/World/Mesh2", primitives="Sphere", positions=[0.0, 1.5, 0.3], scales=[0.3, 0.3, 0.3])

    # Apply collision APIs to meshes so they can be found by SceneQuery
    mesh_geom = GeomPrim(["/World/Mesh1", "/World/Mesh2"], apply_collision_apis=True)

    # Set collision approximation to convexHull - this is for physics simulation,
    # and is independent of the motion generation library representation!
    mesh_geom.set_collision_approximations(["convexHull", "convexHull"])

    # Make meshes rigid bodies so they fall under gravity
    RigidPrim(["/World/Mesh1", "/World/Mesh2"], masses=[1.0, 1.0])

    # Set camera view
    ViewportManager.set_camera_view("/OmniverseKit_Persp", eye=[3.0, 3.0, 2.0], target=[0.0, 0.0, 0.5])

    print("Scene setup complete")


# <end-setup-scene-snippet>


# ============================================================================
# 3. SceneQuery: Finding Objects
# ============================================================================
# <start-scene-query-snippet>
def demonstrate_scene_query() -> list[str]:
    """Demonstrate using SceneQuery to find objects in the scene."""
    # Create a scene query
    query = mg.SceneQuery()

    # Find all collision objects in a bounding box
    # This searches for prims with PhysicsCollisionAPI in the specified region
    search_origin = [0.0, 0.0, 0.0]  # Center of search region
    search_min = [-2.0, -2.0, -0.5]  # Minimum bounds (relative to origin)
    search_max = [2.0, 2.0, 1.5]  # Maximum bounds (relative to origin)

    collision_objects = query.get_prims_in_aabb(
        search_box_origin=search_origin,
        search_box_minimum=search_min,
        search_box_maximum=search_max,
        tracked_api=mg.TrackableApi.PHYSICS_COLLISION,
    )

    # You can also exclude specific prims (e.g., the robot itself)
    # In this example, we exclude the ground plane
    filtered_objects = query.get_prims_in_aabb(
        search_box_origin=search_origin,
        search_box_minimum=search_min,
        search_box_maximum=search_max,
        tracked_api=mg.TrackableApi.PHYSICS_COLLISION,
        exclude_prim_paths=["/Environment/groundCollider"],  # Exclude ground plane (and children) from results
    )

    # Find all robots in the stage (if any)
    robots = query.get_robots_in_stage()

    return filtered_objects


# <end-scene-query-snippet>


# ============================================================================
# 4. ObstacleStrategy: Representation Management
# ============================================================================
# <start-obstacle-strategy-snippet>
def demonstrate_obstacle_strategy(obstacle_paths: list[str]) -> mg.ObstacleStrategy:
    """Demonstrate configuring obstacle representations with ObstacleStrategy."""
    # Create an obstacle strategy
    strategy = mg.ObstacleStrategy()

    # Set default representation for Mesh shape type to OBB with large safety tolerance
    # of 0.15 meters.
    strategy.set_default_configuration(Mesh, mg.ObstacleConfiguration("obb", 0.15))

    # Set per-object overrides for specific prims
    # Mesh2 needs more faithful representation for interaction, so use triangulated mesh
    # with smaller safety tolerance (1cm)
    overrides = {}
    mesh2_path = "/World/Mesh2"
    if mesh2_path in obstacle_paths:
        overrides[mesh2_path] = mg.ObstacleConfiguration("triangulated_mesh", 0.01)
        strategy.set_configuration_overrides(overrides)

    # Set default safety tolerance for all other shapes
    # This will be overridden for all Mesh types (which will have 15cm padding),
    # And for "/World/Mesh2" (which will have 1cm padding).
    strategy.set_default_safety_tolerance(0.05)  # 5cm padding

    return strategy


# <end-obstacle-strategy-snippet>


# ============================================================================
# 5. WorldInterface: Connecting to Motion Planning Libraries
# ============================================================================
# <start-world-interface-snippet>
class ExampleWorldInterface(WorldInterface):
    """Example implementation of WorldInterface.

    In practice, you would implement this to translate obstacle data into your
    planning library's world representation. The methods receive pure data:
    positions, orientations, radii, sizes, etc. as warp arrays - not USD objects.

    For example, if using cumotion, you would add obstacles to their WorldModel here.
    """

    def __init__(self) -> None:
        """Initialize the planning world representation."""
        # In a real implementation, this would initialize your planning library's world
        # e.g., self.world_model = cumotion.WorldModel()
        self.obstacles = {}

    # <start-add-spheres-snippet>
    def add_spheres(
        self,
        prim_paths: list[str],
        radii: Any,
        scales: Any,
        safety_tolerances: Any,
        poses: tuple[Any, Any],
        enabled_array: Any,
    ) -> None:
        """Add spheres to your planning library using the provided data.

        Args:
            prim_paths: Prim paths (Useful as unique identifiers)
            radii: Sphere radii as warp array (shape [N, 1])
            scales: Scale factors as warp array
            safety_tolerances: Safety margins as warp array (shape [N, 1])
            poses: Tuple of (positions, orientations) as warp arrays
            enabled_array: Enabled flags as warp array (shape [N, 1])
        """
        positions, orientations = poses
        for i, path in enumerate(prim_paths):
            # Extract data from warp arrays (all are shape [N, 1])
            radius = radii.numpy()[i, 0] + safety_tolerances.numpy()[i, 0]
            position = positions.numpy()[i]
            orientation = orientations.numpy()[i]

            # Add to your planning library here!
            # e.g., self.world_model.add_sphere(path, radius, position, orientation)
            self.obstacles[path] = {
                "type": "sphere",
                "radius": radius,
                "position": position,
                "orientation": orientation,
            }

    # <end-add-spheres-snippet>

    def add_cubes(
        self,
        prim_paths: list[str],
        sizes: Any,
        scales: Any,
        safety_tolerances: Any,
        poses: tuple[Any, Any],
        enabled_array: Any,
    ) -> None:
        """Add cubes to your planning library using the provided data.

        Args:
            prim_paths: Prim paths (Useful as unique identifiers)
            sizes: Cube side lengths as warp array (shape [N, 1])
            scales: Scale factors as warp array
            safety_tolerances: Safety margins as warp array (shape [N, 1])
            poses: Tuple of (positions, orientations) as warp arrays
            enabled_array: Enabled flags as warp array (shape [N, 1])
        """
        positions, orientations = poses
        for i, path in enumerate(prim_paths):
            # Extract data from warp arrays (all are shape [N, 1])
            size = sizes.numpy()[i, 0] + safety_tolerances.numpy()[i, 0]
            position = positions.numpy()[i]
            orientation = orientations.numpy()[i]

            # In a real implementation, you would add this to your planning library here
            # e.g., self.world_model.add_cube(path, size, position, orientation)
            self.obstacles[path] = {
                "type": "cube",
                "size": size,
                "position": position,
                "orientation": orientation,
            }

    def add_triangulated_meshes(
        self,
        prim_paths: list[str],
        points: list[Any],
        face_vertex_indices: list[Any],
        scales: Any,
        safety_tolerances: Any,
        poses: tuple[Any, Any],
        enabled_array: Any,
    ) -> None:
        """Add triangulated meshes to your planning library using the provided data.

        Args:
            prim_paths: Prim paths (Useful as unique identifiers)
            points: Vertex positions for each mesh (list of warp arrays)
            face_vertex_indices: Triangle vertex indices for each mesh (list of warp arrays)
            scales: Scale factors as warp array
            safety_tolerances: Safety margins as warp array (shape [N, 1])
            poses: Tuple of (positions, orientations) as warp arrays
            enabled_array: Enabled flags as warp array (shape [N, 1])
        """
        positions, orientations = poses
        for i, path in enumerate(prim_paths):
            # Extract data from warp arrays (all are shape [N, 1])
            position = positions.numpy()[i]
            orientation = orientations.numpy()[i]
            safety_tolerance = safety_tolerances.numpy()[i, 0]
            num_triangles = len(face_vertex_indices[i].numpy()) // 3 if i < len(face_vertex_indices) else 0

            # In a real implementation, you would add this to your planning library here
            # e.g., self.world_model.add_triangulated_mesh(path, points[i], face_vertex_indices[i], position, orientation, safety_tolerance)
            self.obstacles[path] = {
                "type": "triangulated_mesh",
                "position": position,
                "orientation": orientation,
                "safety_tolerance": safety_tolerance,
                "num_triangles": num_triangles,
            }

    def add_oriented_bounding_boxes(
        self,
        prim_paths: list[str],
        centers: Any,
        rotations: Any,
        half_side_lengths: Any,
        scales: Any,
        safety_tolerances: Any,
        poses: tuple[Any, Any],
        enabled_array: Any,
    ) -> None:
        """Add oriented bounding boxes to your planning library using the provided data.

        Args:
            prim_paths: Prim paths (Useful as unique identifiers)
            centers: Local center positions for each bounding box.
            rotations: Local rotations as quaternions (w, x, y, z) for each bounding box.
            half_side_lengths: Half extents along each axis for each bounding box.
            scales: Scale factors as warp array.
            safety_tolerances: Safety margins as warp array.
            poses: Tuple of (positions, orientations) as warp arrays.
            enabled_array: Enabled flags as warp array.
        """
        positions, orientations = poses
        centers_np = centers.numpy()
        rotations_np = rotations.numpy()
        half_side_lengths_np = half_side_lengths.numpy()
        for i, path in enumerate(prim_paths):
            # Extract data from warp arrays
            position = positions.numpy()[i]
            orientation = orientations.numpy()[i]
            safety_tolerance = safety_tolerances.numpy()[i, 0]
            center = centers_np[i]
            rotation = rotations_np[i]
            half_extents = half_side_lengths_np[i]

            # In a real implementation, you would add this to your planning library here
            # e.g., self.world_model.add_obb(path, center, rotation, half_extents, position, orientation, safety_tolerance)
            self.obstacles[path] = {
                "type": "oriented_bounding_box",
                "center": center,
                "rotation": rotation,
                "half_extents": half_extents,
                "position": position,
                "orientation": orientation,
                "safety_tolerance": safety_tolerance,
            }

    def add_planes(
        self,
        prim_paths: list[str],
        axes: list[str],
        lengths: Any,
        widths: Any,
        scales: Any,
        safety_tolerances: Any,
        poses: tuple[Any, Any],
        enabled_array: Any,
    ) -> None:
        """Add planes to your planning library using the provided data.

        Args:
            prim_paths: Prim paths (Useful as unique identifiers)
            axes: Normal axis for each plane (list of "X", "Y", or "Z")
            lengths: Plane lengths as warp array (shape [N, 1])
            widths: Plane widths as warp array (shape [N, 1])
            scales: Scale factors as warp array
            safety_tolerances: Safety margins as warp array (shape [N, 1])
            poses: Tuple of (positions, orientations) as warp arrays
            enabled_array: Enabled flags as warp array (shape [N, 1])
        """
        positions, orientations = poses
        for i, path in enumerate(prim_paths):
            # Extract data from warp arrays (all are shape [N, 1] for scalars)
            position = positions.numpy()[i]
            orientation = orientations.numpy()[i]
            safety_tolerance = safety_tolerances.numpy()[i, 0]
            axis = axes[i] if i < len(axes) else "Z"
            length = lengths.numpy()[i, 0]
            width = widths.numpy()[i, 0]

            # In a real implementation, you would add this to your planning library here
            # e.g., self.world_model.add_plane(path, axis, length, width, position, orientation, safety_tolerance)
            self.obstacles[path] = {
                "type": "plane",
                "axis": axis,
                "length": length,
                "width": width,
                "position": position,
                "orientation": orientation,
                "safety_tolerance": safety_tolerance,
            }

    # <start-update-transforms-snippet>
    def update_obstacle_transforms(self, prim_paths: list[str], poses: tuple[Any, Any]) -> None:
        """Update transforms of existing obstacles in your planning library.

        Called frequently for real-time updates, or just before creating a trajectory plan.

        Args:
            prim_paths: Prim paths of obstacles to update
            poses: Tuple of (positions, orientations) as warp arrays
        """
        positions, orientations = poses

        for i, path in enumerate(prim_paths):
            if path in self.obstacles:
                # Extract positions and orientations from warp arrays
                position = positions.numpy()[i]
                orientation = orientations.numpy()[i]

                # Update transforms in your planning library
                # e.g., self.world_model.update_obstacle_pose(path, position, orientation)
                self.obstacles[path]["position"] = position
                self.obstacles[path]["orientation"] = orientation

    # <end-update-transforms-snippet>

    # <start-update-sphere-properties-snippet>
    def update_sphere_properties(self, prim_paths: list[str], radii: Any) -> None:
        """Update sphere-specific properties for existing obstacles.

        Called when shape properties change (e.g., radius changes).

        Args:
            prim_paths: Prim paths of spheres to update
            radii: New sphere radii as warp array (shape [N, 1]), or None to skip
        """
        if radii is None:
            return

        for i, path in enumerate(prim_paths):
            if path in self.obstacles and self.obstacles[path]["type"] == "sphere":
                # Extract new radius from warp array
                new_radius = radii.numpy()[i, 0]

                # Update sphere properties in your planning library
                # e.g., self.world_model.update_sphere_radius(path, new_radius)
                self.obstacles[path]["radius"] = new_radius

    # <end-update-sphere-properties-snippet>

    # Note: In a real implementation, you might also implement:
    # - add_cones, add_cylinders, add_capsules, add_planes, etc.
    # - update_obstacle_enables, update_obstacle_scales, etc.
    # - All other useful methods required by WorldInterface,
    # - depending on your planning library's needs.


# <end-world-interface-snippet>


# ============================================================================
# 6. WorldBinding: Synchronizing Scene to Your Planning Library
# ============================================================================
# <start-world-binding-snippet>
def demonstrate_world_binding(
    obstacle_paths: list[str], obstacle_strategy: mg.ObstacleStrategy
) -> tuple[mg.WorldBinding, ExampleWorldInterface]:
    """Demonstrate using WorldBinding to synchronize scene to planning library."""
    # Create WorldInterface adapter for your planning library
    world_interface = ExampleWorldInterface()

    # Create world binding
    binding = mg.WorldBinding(
        world_interface=world_interface,
        obstacle_strategy=obstacle_strategy,
        tracked_prims=obstacle_paths,
        tracked_collision_api=mg.TrackableApi.PHYSICS_COLLISION,
    )

    # Initialize the binding (populates your planning library's world)
    binding.initialize()

    # In your simulation loop, you would call:
    # binding.synchronize_transforms()  # Fast: updates only poses (use every frame for moving obstacles)
    # binding.synchronize_properties()  # Slower: updates shape properties (use less frequently)
    # binding.synchronize()  # Convenience: calls both methods above

    return binding, world_interface


# <end-world-binding-snippet>


# ============================================================================
# 7. Complete Workflow
# ============================================================================
# <start-main-snippet>
def main() -> None:
    """Run the complete scene interaction workflow."""
    print("=" * 60)
    print("Motion Generation API - Scene Interaction Example")
    print("=" * 60)

    # Setup scene (could come from anywhere - USD file, procedural, etc.)
    setup_scene()

    # Initialize physics
    SimulationManager.set_physics_dt(1.0 / 60.0)

    # Step 1: Find obstacles using SceneQuery
    obstacle_paths = demonstrate_scene_query()

    if len(obstacle_paths) == 0:
        print("\nNo obstacles found in scene. Exiting.")
        simulation_app.close()
        return

    # Step 2: Configure obstacle representations
    obstacle_strategy = demonstrate_obstacle_strategy(obstacle_paths)

    # Step 3: Create WorldInterface and WorldBinding
    binding, world_interface = demonstrate_world_binding(obstacle_paths, obstacle_strategy)

    # Step 4: Demonstrate updating (simulate scene changes)

    # Pick a target object to print its pose
    target_path = "/World/Mesh1"

    # Start timeline
    timeline = omni.timeline.get_timeline_interface()
    # let objects float for a few seconds:
    for _ in range(500):
        simulation_app.update()

    timeline.play()
    # Run simulation loop
    for step in range(25):  # Run for short period while falling:
        # Update the app (advances timeline, handles rendering, physics, etc.)
        simulation_app.update()

        # Update the binding: use synchronize_transforms for fast updates when only poses change
        # For full synchronization including property changes, use synchronize() instead
        # Note: synchronize_transforms() reads current world poses, which are updated by physics
        binding.synchronize_transforms()

        # Print transforms
        obstacle = world_interface.obstacles[target_path]
        position = obstacle["position"]
        orientation = obstacle["orientation"]
        print(f"  WorldBinding update - {target_path}:")
        print(f"    Position: [{position[0]:.3f}, {position[1]:.3f}, {position[2]:.3f}]")
        print(
            f"    Orientation (quat wxyz): [{orientation[0]:.3f}, {orientation[1]:.3f}, {orientation[2]:.3f}, {orientation[3]:.3f}]"
        )

        if step % 5 == 0:  # Periodically check for property changes (less frequent)
            binding.synchronize_properties()

    print("\n" + "=" * 60)
    print("Summary")
    print("=" * 60)
    print(f"  SceneQuery found {len(obstacle_paths)} obstacles")
    print(f"  WorldInterface translated {len(world_interface.obstacles)} obstacles to planning library")
    print(f"  WorldBinding kept planning library synchronized with scene")
    print("\nComplete workflow demonstrated successfully!")

    # Stop timeline
    timeline.pause()

    # Keep window open for a moment to see results
    print("\nClosing soon...")
    for _ in range(500):
        simulation_app.update()

    simulation_app.close()


# <end-main-snippet>


if __name__ == "__main__":
    main()
