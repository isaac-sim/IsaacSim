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

"""Test suite for CumotionWorldInterface functionality including obstacle management and robot base transforms."""

import isaacsim.core.experimental.utils.stage as stage_utils
import isaacsim.robot_motion.cumotion as cu_mg
import numpy as np
import omni.kit.test
import warp as wp
from omni.kit.app import get_app


class TestCumotionWorldInterface(omni.kit.test.AsyncTestCase):
    """Test suite for CumotionWorldInterface."""

    async def setUp(self) -> None:
        """Set up test environment before each test."""
        await stage_utils.create_new_stage_async()
        await get_app().next_update_async()
        self._timeline = omni.timeline.get_timeline_interface()

    async def tearDown(self) -> None:
        """Clean up after each test."""
        if self._timeline.is_playing():
            self._timeline.stop()
        await get_app().next_update_async()

    async def test_world_interface_initialization(self) -> None:
        """Test basic initialization of CumotionWorldInterface."""
        world_interface = cu_mg.CumotionWorldInterface()

        self.assertIsNotNone(world_interface)
        self.assertIsNotNone(world_interface.world_view)
        self.assertEqual(len(world_interface._prim_path_to_collision_data), 0)

    async def test_world_interface_with_debug_visualization(self) -> None:
        """Test initialization with debug visualization enabled."""
        world_interface = cu_mg.CumotionWorldInterface(
            visualize_debug_prims=True,
            visual_debug_enabled_prim_rgb=[1.0, 0.0, 0.0],
            visual_debug_disabled_prim_rgb=[0.0, 1.0, 0.0],
            visual_debug_prim_alpha=0.5,
        )

        self.assertIsNotNone(world_interface)

    async def test_world_interface_with_robot_base_transform(self) -> None:
        """Test initialization with robot base transform."""
        robot_base_pose = (
            wp.array([1.0, 0.0, 0.0], dtype=wp.float32),
            wp.array([1.0, 0.0, 0.0, 0.0], dtype=wp.float32),
        )

        world_interface = cu_mg.CumotionWorldInterface(world_to_robot_base=robot_base_pose)

        self.assertIsNotNone(world_interface)

    async def test_add_sphere_obstacle(self) -> None:
        """Test adding sphere obstacles directly."""
        world_interface = cu_mg.CumotionWorldInterface()

        # Create sphere data
        prim_paths = ["/World/Sphere"]
        radii = wp.array([[0.1]], dtype=wp.float32)
        scales = wp.array([[1.0, 1.0, 1.0]], dtype=wp.float32)
        safety_tolerances = wp.array([[0.0]], dtype=wp.float32)
        positions = wp.array([[0.5, 0.0, 0.5]], dtype=wp.float32)
        quaternions = wp.array([[1.0, 0.0, 0.0, 0.0]], dtype=wp.float32)  # w, x, y, z
        enabled = wp.array([True], dtype=bool)

        # Add sphere
        world_interface.add_spheres(
            prim_paths=prim_paths,
            radii=radii,
            scales=scales,
            safety_tolerances=safety_tolerances,
            poses=(positions, quaternions),
            enabled_array=enabled,
        )

        # Verify sphere was added
        self.assertIn("/World/Sphere", world_interface._prim_path_to_collision_data)
        collision_data = world_interface._prim_path_to_collision_data["/World/Sphere"]
        self.assertEqual(collision_data.n_colliders, 1)

    async def test_add_cube_obstacle(self) -> None:
        """Test adding cube obstacles directly."""
        world_interface = cu_mg.CumotionWorldInterface()

        # Create cube data
        prim_paths = ["/World/Cube"]
        sizes = wp.array([[1.0]], dtype=wp.float32)
        scales = wp.array([[0.2, 0.2, 0.2]], dtype=wp.float32)
        safety_tolerances = wp.array([[0.0]], dtype=wp.float32)
        positions = wp.array([[0.3, 0.0, 0.3]], dtype=wp.float32)
        quaternions = wp.array([[1.0, 0.0, 0.0, 0.0]], dtype=wp.float32)
        enabled = wp.array([True], dtype=bool)

        # Add cube
        world_interface.add_cubes(
            prim_paths=prim_paths,
            sizes=sizes,
            scales=scales,
            safety_tolerances=safety_tolerances,
            poses=(positions, quaternions),
            enabled_array=enabled,
        )

        # Verify cube was added
        self.assertIn("/World/Cube", world_interface._prim_path_to_collision_data)
        collision_data = world_interface._prim_path_to_collision_data["/World/Cube"]
        self.assertEqual(collision_data.n_colliders, 1)

    async def test_add_capsule_obstacle(self) -> None:
        """Test adding capsule obstacles directly."""
        world_interface = cu_mg.CumotionWorldInterface()

        # Create capsule data
        prim_paths = ["/World/Capsule"]
        axes = ["Z"]
        radii = wp.array([[0.05]], dtype=wp.float32)
        lengths = wp.array([[0.3]], dtype=wp.float32)
        scales = wp.array([[1.0, 1.0, 1.0]], dtype=wp.float32)
        safety_tolerances = wp.array([[0.0]], dtype=wp.float32)
        positions = wp.array([[0.4, 0.2, 0.5]], dtype=wp.float32)
        quaternions = wp.array([[1.0, 0.0, 0.0, 0.0]], dtype=wp.float32)
        enabled = wp.array([True], dtype=bool)

        # Add capsule
        world_interface.add_capsules(
            prim_paths=prim_paths,
            axes=axes,
            radii=radii,
            lengths=lengths,
            scales=scales,
            safety_tolerances=safety_tolerances,
            poses=(positions, quaternions),
            enabled_array=enabled,
        )

        # Verify capsule was added
        self.assertIn("/World/Capsule", world_interface._prim_path_to_collision_data)
        collision_data = world_interface._prim_path_to_collision_data["/World/Capsule"]
        self.assertEqual(collision_data.n_colliders, 1)

    async def test_add_oriented_bounding_box_obstacle(self) -> None:
        """Test adding oriented bounding box obstacles directly."""
        world_interface = cu_mg.CumotionWorldInterface()

        # Create OBB data
        prim_paths = ["/World/OBB"]
        centers = wp.array([[0.0, 0.0, 0.0]], dtype=wp.float32)
        rotations = wp.array([[1.0, 0.0, 0.0, 0.0]], dtype=wp.float32)  # Identity quaternion (w, x, y, z)
        half_side_lengths = wp.array([[0.5, 0.5, 0.5]], dtype=wp.float32)
        scales = wp.array([[1.0, 1.0, 1.0]], dtype=wp.float32)
        safety_tolerances = wp.array([[0.0]], dtype=wp.float32)
        positions = wp.array([[0.5, 0.0, 0.5]], dtype=wp.float32)
        quaternions = wp.array([[1.0, 0.0, 0.0, 0.0]], dtype=wp.float32)
        enabled = wp.array([True], dtype=bool)

        # Add OBB
        world_interface.add_oriented_bounding_boxes(
            prim_paths=prim_paths,
            centers=centers,
            rotations=rotations,
            half_side_lengths=half_side_lengths,
            scales=scales,
            safety_tolerances=safety_tolerances,
            poses=(positions, quaternions),
            enabled_array=enabled,
        )

        # Verify OBB was added
        self.assertIn("/World/OBB", world_interface._prim_path_to_collision_data)
        collision_data = world_interface._prim_path_to_collision_data["/World/OBB"]
        self.assertEqual(collision_data.n_colliders, 1)

    async def test_update_obstacle_transform(self) -> None:
        """Test updating obstacle transforms."""
        world_interface = cu_mg.CumotionWorldInterface()

        # Add a sphere first
        prim_paths = ["/World/Sphere"]
        radii = wp.array([[0.1]], dtype=wp.float32)
        scales = wp.array([[1.0, 1.0, 1.0]], dtype=wp.float32)
        safety_tolerances = wp.array([[0.0]], dtype=wp.float32)
        positions = wp.array([[0.5, 0.0, 0.5]], dtype=wp.float32)
        quaternions = wp.array([[1.0, 0.0, 0.0, 0.0]], dtype=wp.float32)
        enabled = wp.array([True], dtype=bool)

        world_interface.add_spheres(
            prim_paths=prim_paths,
            radii=radii,
            scales=scales,
            safety_tolerances=safety_tolerances,
            poses=(positions, quaternions),
            enabled_array=enabled,
        )

        # Update the position
        new_positions = wp.array([[1.0, 0.5, 0.3]], dtype=wp.float32)
        new_quaternions = wp.array([[1.0, 0.0, 0.0, 0.0]], dtype=wp.float32)

        world_interface.update_obstacle_transforms(prim_paths=prim_paths, poses=(new_positions, new_quaternions))

        # Verify update completed:
        sphere_position_index = world_interface._prim_path_to_collision_data[
            "/World/Sphere"
        ].transform_world_to_object_index
        sphere_position = world_interface._position_world_to_objects[sphere_position_index, :]
        sphere_quaternion = world_interface._quaternion_world_to_objects[sphere_position_index, :]
        self.assertTrue(np.isclose(sphere_position, [1.0, 0.5, 0.3]).all())
        self.assertTrue(np.isclose(sphere_quaternion, [1.0, 0.0, 0.0, 0.0]).all())

    async def test_enable_disable_obstacle(self) -> None:
        """Test enabling and disabling obstacles."""
        world_interface = cu_mg.CumotionWorldInterface()

        # Add a sphere first
        prim_paths = ["/World/Sphere"]
        radii = wp.array([[0.1]], dtype=wp.float32)
        scales = wp.array([[1.0, 1.0, 1.0]], dtype=wp.float32)
        safety_tolerances = wp.array([[0.0]], dtype=wp.float32)
        positions = wp.array([[0.5, 0.0, 0.5]], dtype=wp.float32)
        quaternions = wp.array([[1.0, 0.0, 0.0, 0.0]], dtype=wp.float32)
        enabled = wp.array([True], dtype=bool)

        world_interface.add_spheres(
            prim_paths=prim_paths,
            radii=radii,
            scales=scales,
            safety_tolerances=safety_tolerances,
            poses=(positions, quaternions),
            enabled_array=enabled,
        )

        # Disable the obstacle
        enabled_disabled = wp.array([False], dtype=bool)
        world_interface.update_obstacle_enables(prim_paths=prim_paths, enabled_array=enabled_disabled)

        # Re-enable the obstacle
        enabled_enabled = wp.array([True], dtype=bool)
        world_interface.update_obstacle_enables(prim_paths=prim_paths, enabled_array=enabled_enabled)

        # Verify update completed
        self.assertIn("/World/Sphere", world_interface._prim_path_to_collision_data)

    async def test_multiple_obstacles(self) -> None:
        """Test adding multiple obstacles of different types."""
        world_interface = cu_mg.CumotionWorldInterface()

        # Add a sphere
        world_interface.add_spheres(
            prim_paths=["/World/Sphere"],
            radii=wp.array([[0.1]], dtype=wp.float32),
            scales=wp.array([[1.0, 1.0, 1.0]], dtype=wp.float32),
            safety_tolerances=wp.array([[0.0]], dtype=wp.float32),
            poses=(wp.array([[0.5, 0.0, 0.5]], dtype=wp.float32), wp.array([[1.0, 0.0, 0.0, 0.0]], dtype=wp.float32)),
            enabled_array=wp.array([True], dtype=bool),
        )

        # Add a cube
        world_interface.add_cubes(
            prim_paths=["/World/Cube"],
            sizes=wp.array([[1.0]], dtype=wp.float32),
            scales=wp.array([[0.2, 0.2, 0.2]], dtype=wp.float32),
            safety_tolerances=wp.array([[0.0]], dtype=wp.float32),
            poses=(wp.array([[0.3, 0.0, 0.3]], dtype=wp.float32), wp.array([[1.0, 0.0, 0.0, 0.0]], dtype=wp.float32)),
            enabled_array=wp.array([True], dtype=bool),
        )

        # Add a capsule
        world_interface.add_capsules(
            prim_paths=["/World/Capsule"],
            axes=["Z"],
            radii=wp.array([[0.05]], dtype=wp.float32),
            lengths=wp.array([[0.3]], dtype=wp.float32),
            scales=wp.array([[1.0, 1.0, 1.0]], dtype=wp.float32),
            safety_tolerances=wp.array([[0.0]], dtype=wp.float32),
            poses=(wp.array([[0.4, 0.2, 0.5]], dtype=wp.float32), wp.array([[1.0, 0.0, 0.0, 0.0]], dtype=wp.float32)),
            enabled_array=wp.array([True], dtype=bool),
        )

        # Verify all obstacles are registered
        self.assertIn("/World/Sphere", world_interface._prim_path_to_collision_data)
        self.assertIn("/World/Cube", world_interface._prim_path_to_collision_data)
        self.assertIn("/World/Capsule", world_interface._prim_path_to_collision_data)

    async def test_world_view_update(self) -> None:
        """Test that world view updates properly."""
        world_interface = cu_mg.CumotionWorldInterface()

        # Update should not raise errors
        world_interface.world_view.update()

        self.assertIsNotNone(world_interface.world_view)

    async def test_uniform_scale_requirement_sphere(self) -> None:
        """Test that non-uniform scaling on spheres raises an error."""
        world_interface = cu_mg.CumotionWorldInterface()

        # Create sphere data with non-uniform scale
        prim_paths = ["/World/Sphere"]
        radii = wp.array([[0.1]], dtype=wp.float32)
        scales = wp.array([[1.0, 2.0, 1.0]], dtype=wp.float32)  # Non-uniform
        safety_tolerances = wp.array([[0.0]], dtype=wp.float32)
        positions = wp.array([[0.5, 0.0, 0.5]], dtype=wp.float32)
        quaternions = wp.array([[1.0, 0.0, 0.0, 0.0]], dtype=wp.float32)
        enabled = wp.array([True], dtype=bool)

        # Should raise ValueError
        with self.assertRaises(ValueError):
            world_interface.add_spheres(
                prim_paths=prim_paths,
                radii=radii,
                scales=scales,
                safety_tolerances=safety_tolerances,
                poses=(positions, quaternions),
                enabled_array=enabled,
            )

    async def test_uniform_scale_requirement_capsule(self) -> None:
        """Test that non-uniform scaling on capsules raises an error."""
        world_interface = cu_mg.CumotionWorldInterface()

        # Create capsule data with non-uniform scale
        prim_paths = ["/World/Capsule"]
        axes = ["Z"]
        radii = wp.array([[0.05]], dtype=wp.float32)
        lengths = wp.array([[0.3]], dtype=wp.float32)
        scales = wp.array([[1.0, 2.0, 1.0]], dtype=wp.float32)  # Non-uniform
        safety_tolerances = wp.array([[0.0]], dtype=wp.float32)
        positions = wp.array([[0.4, 0.2, 0.5]], dtype=wp.float32)
        quaternions = wp.array([[1.0, 0.0, 0.0, 0.0]], dtype=wp.float32)
        enabled = wp.array([True], dtype=bool)

        # Should raise ValueError
        with self.assertRaises(ValueError):
            world_interface.add_capsules(
                prim_paths=prim_paths,
                axes=axes,
                radii=radii,
                lengths=lengths,
                scales=scales,
                safety_tolerances=safety_tolerances,
                poses=(positions, quaternions),
                enabled_array=enabled,
            )

    async def test_robot_base_transform_update(self) -> None:
        """Test updating robot base transform."""
        world_interface = cu_mg.CumotionWorldInterface()

        # Add a sphere first to have something to transform
        world_interface.add_spheres(
            prim_paths=["/World/Sphere"],
            radii=wp.array([[0.1]], dtype=wp.float32),
            scales=wp.array([[1.0, 1.0, 1.0]], dtype=wp.float32),
            safety_tolerances=wp.array([[0.0]], dtype=wp.float32),
            poses=(wp.array([[0.5, 0.0, 0.5]], dtype=wp.float32), wp.array([[1.0, 0.0, 0.0, 0.0]], dtype=wp.float32)),
            enabled_array=wp.array([True], dtype=bool),
        )

        # Create positions and orientations for robot base
        positions = wp.array([[1.0, 2.0, 0.0]], dtype=wp.float32)
        quaternions = wp.array([[1.0, 0.0, 0.0, 0.0]], dtype=wp.float32)  # w, x, y, z

        # Update robot base transform
        world_interface.update_world_to_robot_root_transforms((positions, quaternions))

        # Should complete without errors
        self.assertIsNotNone(world_interface)

    async def test_add_plane_obstacle(self) -> None:
        """Test adding plane obstacles directly."""
        world_interface = cu_mg.CumotionWorldInterface()

        # Create plane data
        prim_paths = ["/World/Plane"]
        axes = ["Z"]
        lengths = wp.array([[10.0]], dtype=wp.float32)
        widths = wp.array([[10.0]], dtype=wp.float32)
        scales = wp.array([[1.0, 1.0, 1.0]], dtype=wp.float32)
        safety_tolerances = wp.array([[0.0]], dtype=wp.float32)
        positions = wp.array([[0.0, 0.0, 0.0]], dtype=wp.float32)
        quaternions = wp.array([[1.0, 0.0, 0.0, 0.0]], dtype=wp.float32)
        enabled = wp.array([True], dtype=bool)

        # Add plane
        world_interface.add_planes(
            prim_paths=prim_paths,
            axes=axes,
            lengths=lengths,
            widths=widths,
            scales=scales,
            safety_tolerances=safety_tolerances,
            poses=(positions, quaternions),
            enabled_array=enabled,
        )

        # Verify plane was added
        self.assertIn("/World/Plane", world_interface._prim_path_to_collision_data)

    async def test_add_triangulated_mesh(self) -> None:
        """Test adding triangulated mesh obstacles directly."""
        world_interface = cu_mg.CumotionWorldInterface()

        # Create a simple tetrahedron mesh (4 vertices, 4 triangular faces with volume)
        points = [
            wp.array(
                [
                    [0.0, 0.0, 0.0],  # vertex 0
                    [1.0, 0.0, 0.0],  # vertex 1
                    [0.5, 1.0, 0.0],  # vertex 2
                    [0.5, 0.5, 0.8],  # vertex 3 (apex, gives volume)
                ],
                dtype=wp.float32,
            )
        ]
        face_vertex_indices = [
            wp.array(
                [
                    [0, 1, 2],  # base triangle
                    [0, 1, 3],  # side face 1
                    [1, 2, 3],  # side face 2
                    [2, 0, 3],  # side face 3
                ],
                dtype=wp.int32,
            )
        ]

        prim_paths = ["/World/Mesh"]
        scales = wp.array([[1.0, 1.0, 1.0]], dtype=wp.float32)
        safety_tolerances = wp.array([[0.01]], dtype=wp.float32)
        positions = wp.array([[0.0, 0.0, 0.5]], dtype=wp.float32)
        quaternions = wp.array([[1.0, 0.0, 0.0, 0.0]], dtype=wp.float32)
        enabled = wp.array([True], dtype=bool)

        # Add mesh
        world_interface.add_triangulated_meshes(
            prim_paths=prim_paths,
            points=points,
            face_vertex_indices=face_vertex_indices,
            scales=scales,
            safety_tolerances=safety_tolerances,
            poses=(positions, quaternions),
            enabled_array=enabled,
        )

        # Verify mesh was added (will be represented as collision spheres)
        self.assertIn("/World/Mesh", world_interface._prim_path_to_collision_data)

    async def test_capsule_different_axes(self) -> None:
        """Test adding capsules with different axis orientations."""
        world_interface = cu_mg.CumotionWorldInterface()

        # Test X-axis capsule
        world_interface.add_capsules(
            prim_paths=["/World/CapsuleX"],
            axes=["X"],
            radii=wp.array([[0.05]], dtype=wp.float32),
            lengths=wp.array([[0.3]], dtype=wp.float32),
            scales=wp.array([[1.0, 1.0, 1.0]], dtype=wp.float32),
            safety_tolerances=wp.array([[0.0]], dtype=wp.float32),
            poses=(wp.array([[0.0, 0.0, 0.0]], dtype=wp.float32), wp.array([[1.0, 0.0, 0.0, 0.0]], dtype=wp.float32)),
            enabled_array=wp.array([True], dtype=bool),
        )

        # Test Y-axis capsule
        world_interface.add_capsules(
            prim_paths=["/World/CapsuleY"],
            axes=["Y"],
            radii=wp.array([[0.05]], dtype=wp.float32),
            lengths=wp.array([[0.3]], dtype=wp.float32),
            scales=wp.array([[1.0, 1.0, 1.0]], dtype=wp.float32),
            safety_tolerances=wp.array([[0.0]], dtype=wp.float32),
            poses=(wp.array([[0.5, 0.0, 0.0]], dtype=wp.float32), wp.array([[1.0, 0.0, 0.0, 0.0]], dtype=wp.float32)),
            enabled_array=wp.array([True], dtype=bool),
        )

        # Test Z-axis capsule
        world_interface.add_capsules(
            prim_paths=["/World/CapsuleZ"],
            axes=["Z"],
            radii=wp.array([[0.05]], dtype=wp.float32),
            lengths=wp.array([[0.3]], dtype=wp.float32),
            scales=wp.array([[1.0, 1.0, 1.0]], dtype=wp.float32),
            safety_tolerances=wp.array([[0.0]], dtype=wp.float32),
            poses=(wp.array([[1.0, 0.0, 0.0]], dtype=wp.float32), wp.array([[1.0, 0.0, 0.0, 0.0]], dtype=wp.float32)),
            enabled_array=wp.array([True], dtype=bool),
        )

        # Verify all capsules were added
        self.assertIn("/World/CapsuleX", world_interface._prim_path_to_collision_data)
        self.assertIn("/World/CapsuleY", world_interface._prim_path_to_collision_data)
        self.assertIn("/World/CapsuleZ", world_interface._prim_path_to_collision_data)

    async def test_invalid_capsule_axis(self) -> None:
        """Test that invalid axis for capsule raises an error."""
        world_interface = cu_mg.CumotionWorldInterface()

        # Test with invalid axis string
        with self.assertRaises(ValueError):
            world_interface.add_capsules(
                prim_paths=["/World/Capsule"],
                axes=["W"],  # Invalid axis
                radii=wp.array([[0.05]], dtype=wp.float32),
                lengths=wp.array([[0.3]], dtype=wp.float32),
                scales=wp.array([[1.0, 1.0, 1.0]], dtype=wp.float32),
                safety_tolerances=wp.array([[0.0]], dtype=wp.float32),
                poses=(
                    wp.array([[0.0, 0.0, 0.0]], dtype=wp.float32),
                    wp.array([[1.0, 0.0, 0.0, 0.0]], dtype=wp.float32),
                ),
                enabled_array=wp.array([True], dtype=bool),
            )

    async def test_invalid_plane_axis(self) -> None:
        """Test that invalid axis for plane raises an error."""
        world_interface = cu_mg.CumotionWorldInterface()

        # Test with lowercase axis (should be uppercase)
        with self.assertRaises(ValueError):
            world_interface.add_planes(
                prim_paths=["/World/Plane"],
                axes=["invalid"],  # Invalid axis
                lengths=wp.array([[10.0]], dtype=wp.float32),
                widths=wp.array([[10.0]], dtype=wp.float32),
                scales=wp.array([[1.0, 1.0, 1.0]], dtype=wp.float32),
                safety_tolerances=wp.array([[0.0]], dtype=wp.float32),
                poses=(
                    wp.array([[0.0, 0.0, 0.0]], dtype=wp.float32),
                    wp.array([[1.0, 0.0, 0.0, 0.0]], dtype=wp.float32),
                ),
                enabled_array=wp.array([True], dtype=bool),
            )

    async def test_mismatched_array_lengths(self) -> None:
        """Test that mismatched array lengths raise an error."""
        world_interface = cu_mg.CumotionWorldInterface()

        # Two prim paths but only one radius
        with self.assertRaises((ValueError, RuntimeError)):
            world_interface.add_spheres(
                prim_paths=["/World/Sphere1", "/World/Sphere2"],
                radii=wp.array([[0.1]], dtype=wp.float32),  # Only 1 radius for 2 prims
                scales=wp.array([[1.0, 1.0, 1.0]], dtype=wp.float32),
                safety_tolerances=wp.array([[0.0]], dtype=wp.float32),
                poses=(
                    wp.array([[0.5, 0.0, 0.5]], dtype=wp.float32),
                    wp.array([[1.0, 0.0, 0.0, 0.0]], dtype=wp.float32),
                ),
                enabled_array=wp.array([True], dtype=bool),
            )

    async def test_empty_prim_paths(self) -> None:
        """Test that empty prim paths list is handled gracefully."""
        world_interface = cu_mg.CumotionWorldInterface()

        # trying to add spheres with empty prim paths should raise an error
        with self.assertRaises(ValueError):
            world_interface.add_spheres(
                prim_paths=[],
                radii=wp.array([], dtype=wp.float32),
                scales=wp.array([], dtype=wp.float32),
                safety_tolerances=wp.array([], dtype=wp.float32),
                poses=(wp.array([], dtype=wp.float32), wp.array([], dtype=wp.float32)),
                enabled_array=wp.array([], dtype=bool),
            )

        # trying to add cubes with empty prim paths should raise an error
        with self.assertRaises(ValueError):
            world_interface.add_cubes(
                prim_paths=[],
                sizes=wp.array([], dtype=wp.float32),
                scales=wp.array([], dtype=wp.float32),
                safety_tolerances=wp.array([], dtype=wp.float32),
                poses=(wp.array([], dtype=wp.float32), wp.array([], dtype=wp.float32)),
                enabled_array=wp.array([], dtype=bool),
            )

        # trying to add capsules with empty prim paths should raise an error
        with self.assertRaises(ValueError):
            world_interface.add_capsules(
                prim_paths=[],
                axes=["X"],
                radii=wp.array([], dtype=wp.float32),
                lengths=wp.array([], dtype=wp.float32),
                scales=wp.array([], dtype=wp.float32),
                safety_tolerances=wp.array([], dtype=wp.float32),
                poses=(wp.array([], dtype=wp.float32), wp.array([], dtype=wp.float32)),
                enabled_array=wp.array([], dtype=bool),
            )

        # trying to add planes with empty prim paths should raise an error
        with self.assertRaises(ValueError):
            world_interface.add_planes(
                prim_paths=[],
                axes=["X"],
                lengths=wp.array([], dtype=wp.float32),
                widths=wp.array([], dtype=wp.float32),
                scales=wp.array([], dtype=wp.float32),
                safety_tolerances=wp.array([], dtype=wp.float32),
                poses=(wp.array([], dtype=wp.float32), wp.array([], dtype=wp.float32)),
                enabled_array=wp.array([], dtype=bool),
            )

        # trying to add triangulated meshes with empty prim paths should raise an error
        with self.assertRaises(ValueError):
            world_interface.add_triangulated_meshes(
                prim_paths=[],
                points=[wp.array([], dtype=wp.float32)],
                face_vertex_indices=[wp.array([], dtype=wp.int32)],
                scales=wp.array([], dtype=wp.float32),
                safety_tolerances=wp.array([], dtype=wp.float32),
                poses=(wp.array([], dtype=wp.float32), wp.array([], dtype=wp.float32)),
                enabled_array=wp.array([], dtype=bool),
            )

    async def test_mesh_with_invalid_face_indices(self) -> None:
        """Test that mesh with out-of-bounds face indices raises an error."""
        world_interface = cu_mg.CumotionWorldInterface()

        # Only 3 vertices, but face references vertex 5
        points = [wp.array([[0.0, 0.0, 0.0], [1.0, 0.0, 0.0], [0.5, 1.0, 0.0]], dtype=wp.float32)]
        face_vertex_indices = [wp.array([[0, 1, 5]], dtype=wp.int32)]  # Index 5 is out of bounds

        with self.assertRaises((ValueError, RuntimeError, IndexError)):
            world_interface.add_triangulated_meshes(
                prim_paths=["/World/Mesh"],
                points=points,
                face_vertex_indices=face_vertex_indices,
                scales=wp.array([[1.0, 1.0, 1.0]], dtype=wp.float32),
                safety_tolerances=wp.array([[0.01]], dtype=wp.float32),
                poses=(
                    wp.array([[0.0, 0.0, 0.5]], dtype=wp.float32),
                    wp.array([[1.0, 0.0, 0.0, 0.0]], dtype=wp.float32),
                ),
                enabled_array=wp.array([True], dtype=bool),
            )

    async def test_mesh_with_negative_face_indices(self) -> None:
        """Test that mesh with negative face indices raises an error."""
        world_interface = cu_mg.CumotionWorldInterface()

        # Negative face index
        points = [wp.array([[0.0, 0.0, 0.0], [1.0, 0.0, 0.0], [0.5, 1.0, 0.0]], dtype=wp.float32)]
        face_vertex_indices = [wp.array([[-1, 1, 2]], dtype=wp.int32)]  # Negative index

        with self.assertRaises((ValueError, RuntimeError, IndexError)):
            world_interface.add_triangulated_meshes(
                prim_paths=["/World/Mesh"],
                points=points,
                face_vertex_indices=face_vertex_indices,
                scales=wp.array([[1.0, 1.0, 1.0]], dtype=wp.float32),
                safety_tolerances=wp.array([[0.01]], dtype=wp.float32),
                poses=(
                    wp.array([[0.0, 0.0, 0.5]], dtype=wp.float32),
                    wp.array([[1.0, 0.0, 0.0, 0.0]], dtype=wp.float32),
                ),
                enabled_array=wp.array([True], dtype=bool),
            )

    async def test_mesh_with_non_triangular_faces(self) -> None:
        """Test that mesh with non-triangular faces (quads) raises an error."""
        world_interface = cu_mg.CumotionWorldInterface()

        # 4 vertices per face (quad) instead of 3 (triangle)
        points = [wp.array([[0.0, 0.0, 0.0], [1.0, 0.0, 0.0], [1.0, 1.0, 0.0], [0.0, 1.0, 0.0]], dtype=wp.float32)]
        face_vertex_indices = [wp.array([[0, 1, 2, 3]], dtype=wp.int32)]  # 4 indices (quad)

        with self.assertRaises((ValueError, RuntimeError)):
            world_interface.add_triangulated_meshes(
                prim_paths=["/World/Mesh"],
                points=points,
                face_vertex_indices=face_vertex_indices,
                scales=wp.array([[1.0, 1.0, 1.0]], dtype=wp.float32),
                safety_tolerances=wp.array([[0.01]], dtype=wp.float32),
                poses=(
                    wp.array([[0.0, 0.0, 0.5]], dtype=wp.float32),
                    wp.array([[1.0, 0.0, 0.0, 0.0]], dtype=wp.float32),
                ),
                enabled_array=wp.array([True], dtype=bool),
            )

    async def test_mesh_with_empty_points(self) -> None:
        """Test that mesh with no points raises an error."""
        world_interface = cu_mg.CumotionWorldInterface()

        # Empty points array
        points = [wp.array([], dtype=wp.float32)]
        face_vertex_indices = [wp.array([[0, 1, 2]], dtype=wp.int32)]

        with self.assertRaises((ValueError, RuntimeError, IndexError)):
            world_interface.add_triangulated_meshes(
                prim_paths=["/World/Mesh"],
                points=points,
                face_vertex_indices=face_vertex_indices,
                scales=wp.array([[1.0, 1.0, 1.0]], dtype=wp.float32),
                safety_tolerances=wp.array([[0.01]], dtype=wp.float32),
                poses=(
                    wp.array([[0.0, 0.0, 0.5]], dtype=wp.float32),
                    wp.array([[1.0, 0.0, 0.0, 0.0]], dtype=wp.float32),
                ),
                enabled_array=wp.array([True], dtype=bool),
            )

    async def test_negative_radius(self) -> None:
        """Test that negative radius raises an error."""
        world_interface = cu_mg.CumotionWorldInterface()

        with self.assertRaises(ValueError):
            world_interface.add_spheres(
                prim_paths=["/World/Sphere"],
                radii=wp.array([[-0.1]], dtype=wp.float32),  # Negative radius
                scales=wp.array([[1.0, 1.0, 1.0]], dtype=wp.float32),
                safety_tolerances=wp.array([[0.0]], dtype=wp.float32),
                poses=(
                    wp.array([[0.5, 0.0, 0.5]], dtype=wp.float32),
                    wp.array([[1.0, 0.0, 0.0, 0.0]], dtype=wp.float32),
                ),
                enabled_array=wp.array([True], dtype=bool),
            )

    async def test_negative_capsule_length(self) -> None:
        """Test that negative capsule length raises an error."""
        world_interface = cu_mg.CumotionWorldInterface()

        with self.assertRaises(ValueError):
            world_interface.add_capsules(
                prim_paths=["/World/Capsule"],
                axes=["Z"],
                radii=wp.array([[0.05]], dtype=wp.float32),
                lengths=wp.array([[-0.3]], dtype=wp.float32),  # Negative length
                scales=wp.array([[1.0, 1.0, 1.0]], dtype=wp.float32),
                safety_tolerances=wp.array([[0.0]], dtype=wp.float32),
                poses=(
                    wp.array([[0.4, 0.2, 0.5]], dtype=wp.float32),
                    wp.array([[1.0, 0.0, 0.0, 0.0]], dtype=wp.float32),
                ),
                enabled_array=wp.array([True], dtype=bool),
            )

    async def test_update_nonexistent_obstacle(self) -> None:
        """Test that updating a non-existent obstacle raises an error."""
        world_interface = cu_mg.CumotionWorldInterface()

        # Try to update an obstacle that was never added
        with self.assertRaises((ValueError, KeyError, RuntimeError)):
            world_interface.update_obstacle_transforms(
                prim_paths=["/World/NonExistent"],
                poses=(
                    wp.array([[1.0, 0.0, 0.0]], dtype=wp.float32),
                    wp.array([[1.0, 0.0, 0.0, 0.0]], dtype=wp.float32),
                ),
            )

    async def test_invalid_quaternion_dimensions(self) -> None:
        """Test that quaternions with wrong number of elements raise an error."""
        world_interface = cu_mg.CumotionWorldInterface()

        # Test with 3-element quaternion instead of 4
        with self.assertRaises((ValueError, RuntimeError)):
            world_interface.add_spheres(
                prim_paths=["/World/Sphere"],
                radii=wp.array([[0.1]], dtype=wp.float32),
                scales=wp.array([[1.0, 1.0, 1.0]], dtype=wp.float32),
                safety_tolerances=wp.array([[0.0]], dtype=wp.float32),
                poses=(
                    wp.array([[0.5, 0.0, 0.5]], dtype=wp.float32),
                    wp.array([[1.0, 0.0, 0.0]], dtype=wp.float32),
                ),  # Only 3 elements
                enabled_array=wp.array([True], dtype=bool),
            )

    async def test_invalid_position_dimensions(self) -> None:
        """Test that positions with wrong number of elements raise an error."""
        world_interface = cu_mg.CumotionWorldInterface()

        # Test with 2D position instead of 3D
        with self.assertRaises((ValueError, RuntimeError)):
            world_interface.add_spheres(
                prim_paths=["/World/Sphere"],
                radii=wp.array([[0.1]], dtype=wp.float32),
                scales=wp.array([[1.0, 1.0, 1.0]], dtype=wp.float32),
                safety_tolerances=wp.array([[0.0]], dtype=wp.float32),
                poses=(
                    wp.array([[0.5, 0.5]], dtype=wp.float32),  # Only 2 elements
                    wp.array([[1.0, 0.0, 0.0, 0.0]], dtype=wp.float32),
                ),
                enabled_array=wp.array([True], dtype=bool),
            )

    async def test_non_uniform_scale_on_sphere(self) -> None:
        """Test that non-uniform scale on sphere raises an error."""
        world_interface = cu_mg.CumotionWorldInterface()

        # Spheres must have uniform scale
        with self.assertRaises(ValueError):
            world_interface.add_spheres(
                prim_paths=["/World/Sphere"],
                radii=wp.array([[0.1]], dtype=wp.float32),
                scales=wp.array([[1.0, 2.0, 1.0]], dtype=wp.float32),  # Non-uniform scale
                safety_tolerances=wp.array([[0.0]], dtype=wp.float32),
                poses=(
                    wp.array([[0.5, 0.0, 0.5]], dtype=wp.float32),
                    wp.array([[1.0, 0.0, 0.0, 0.0]], dtype=wp.float32),
                ),
                enabled_array=wp.array([True], dtype=bool),
            )

    async def test_non_uniform_scale_on_capsule(self) -> None:
        """Test that non-uniform scale on capsule raises an error."""
        world_interface = cu_mg.CumotionWorldInterface()

        # Capsules must have uniform scale
        with self.assertRaises(ValueError):
            world_interface.add_capsules(
                prim_paths=["/World/Capsule"],
                axes=["Z"],
                radii=wp.array([[0.05]], dtype=wp.float32),
                lengths=wp.array([[0.3]], dtype=wp.float32),
                scales=wp.array([[1.0, 1.5, 1.0]], dtype=wp.float32),  # Non-uniform scale
                safety_tolerances=wp.array([[0.0]], dtype=wp.float32),
                poses=(
                    wp.array([[0.4, 0.2, 0.5]], dtype=wp.float32),
                    wp.array([[1.0, 0.0, 0.0, 0.0]], dtype=wp.float32),
                ),
                enabled_array=wp.array([True], dtype=bool),
            )

    async def test_duplicate_prim_path_same_type(self) -> None:
        """Test that adding the same prim path twice with same geometry type raises an error."""
        world_interface = cu_mg.CumotionWorldInterface()

        # Add a sphere
        world_interface.add_spheres(
            prim_paths=["/World/Sphere"],
            radii=wp.array([[0.1]], dtype=wp.float32),
            scales=wp.array([[1.0, 1.0, 1.0]], dtype=wp.float32),
            safety_tolerances=wp.array([[0.0]], dtype=wp.float32),
            poses=(wp.array([[0.5, 0.0, 0.5]], dtype=wp.float32), wp.array([[1.0, 0.0, 0.0, 0.0]], dtype=wp.float32)),
            enabled_array=wp.array([True], dtype=bool),
        )

        # Try to add the same prim path again - should raise an error
        with self.assertRaises(ValueError):
            world_interface.add_spheres(
                prim_paths=["/World/Sphere"],
                radii=wp.array([[0.2]], dtype=wp.float32),
                scales=wp.array([[1.0, 1.0, 1.0]], dtype=wp.float32),
                safety_tolerances=wp.array([[0.0]], dtype=wp.float32),
                poses=(
                    wp.array([[0.6, 0.0, 0.6]], dtype=wp.float32),
                    wp.array([[1.0, 0.0, 0.0, 0.0]], dtype=wp.float32),
                ),
                enabled_array=wp.array([True], dtype=bool),
            )

    async def test_duplicate_prim_path_different_types(self) -> None:
        """Test that adding the same prim path with different geometry types raises an error."""
        world_interface = cu_mg.CumotionWorldInterface()

        # Add a sphere
        world_interface.add_spheres(
            prim_paths=["/World/Object"],
            radii=wp.array([[0.1]], dtype=wp.float32),
            scales=wp.array([[1.0, 1.0, 1.0]], dtype=wp.float32),
            safety_tolerances=wp.array([[0.0]], dtype=wp.float32),
            poses=(wp.array([[0.5, 0.0, 0.5]], dtype=wp.float32), wp.array([[1.0, 0.0, 0.0, 0.0]], dtype=wp.float32)),
            enabled_array=wp.array([True], dtype=bool),
        )

        # Try to add a cube with the same prim path - should raise an error
        with self.assertRaises(ValueError):
            world_interface.add_cubes(
                prim_paths=["/World/Object"],
                sizes=wp.array([[1.0]], dtype=wp.float32),
                scales=wp.array([[1.0, 1.0, 1.0]], dtype=wp.float32),
                safety_tolerances=wp.array([[0.0]], dtype=wp.float32),
                poses=(
                    wp.array([[0.3, 0.0, 0.3]], dtype=wp.float32),
                    wp.array([[1.0, 0.0, 0.0, 0.0]], dtype=wp.float32),
                ),
                enabled_array=wp.array([True], dtype=bool),
            )

    async def test_duplicate_prim_path_in_batch(self) -> None:
        """Test that adding duplicate prim paths within a single batch raises an error."""
        world_interface = cu_mg.CumotionWorldInterface()

        # Try to add multiple spheres where prim paths contain duplicates
        with self.assertRaises(ValueError):
            world_interface.add_spheres(
                prim_paths=["/World/Sphere1", "/World/Sphere2", "/World/Sphere1"],  # Duplicate
                radii=wp.array([[0.1], [0.1], [0.1]], dtype=wp.float32),
                scales=wp.array([[1.0, 1.0, 1.0], [1.0, 1.0, 1.0], [1.0, 1.0, 1.0]], dtype=wp.float32),
                safety_tolerances=wp.array([[0.0], [0.0], [0.0]], dtype=wp.float32),
                poses=(
                    wp.array([[0.5, 0.0, 0.5], [1.0, 0.0, 0.5], [1.5, 0.0, 0.5]], dtype=wp.float32),
                    wp.array([[1.0, 0.0, 0.0, 0.0], [1.0, 0.0, 0.0, 0.0], [1.0, 0.0, 0.0, 0.0]], dtype=wp.float32),
                ),
                enabled_array=wp.array([True, True, True], dtype=bool),
            )

    async def test_invalid_robot_base_position_dimensions(self) -> None:
        """Test that invalid robot base position dimensions raise an error."""
        # Invalid position (2D instead of 3D)
        with self.assertRaises((ValueError, RuntimeError)):
            world_interface = cu_mg.CumotionWorldInterface(
                world_to_robot_base=(
                    wp.array([[1.0, 0.0]], dtype=wp.float32),  # Only 2 elements
                    wp.array([[1.0, 0.0, 0.0, 0.0]], dtype=wp.float32),
                )
            )

    async def test_invalid_robot_base_quaternion_dimensions(self) -> None:
        """Test that invalid robot base quaternion dimensions raise an error."""
        # Invalid quaternion (3 elements instead of 4)
        with self.assertRaises((ValueError, RuntimeError)):
            world_interface = cu_mg.CumotionWorldInterface(
                world_to_robot_base=(
                    wp.array([[1.0, 0.0, 0.0]], dtype=wp.float32),
                    wp.array([[1.0, 0.0, 0.0]], dtype=wp.float32),  # Only 3 elements
                )
            )

    async def test_update_robot_base_invalid_positions(self) -> None:
        """Test that updating robot base with invalid position dimensions raises an error."""
        world_interface = cu_mg.CumotionWorldInterface()

        # Try to update with 2D positions instead of 3D
        with self.assertRaises(ValueError):
            world_interface.update_world_to_robot_root_transforms(
                poses=(wp.array([[1.0, 0.0]], dtype=wp.float32), wp.array([[1.0, 0.0, 0.0, 0.0]], dtype=wp.float32))
            )

    async def test_update_robot_base_invalid_quaternions(self) -> None:
        """Test that updating robot base with invalid quaternion dimensions raises an error."""
        world_interface = cu_mg.CumotionWorldInterface()

        # Try to update with 3-element quaternions instead of 4
        with self.assertRaises(ValueError):
            world_interface.update_world_to_robot_root_transforms(
                poses=(
                    wp.array([[1.0, 0.0, 0.0]], dtype=wp.float32),
                    wp.array([[1.0, 0.0, 0.0]], dtype=wp.float32),
                )  # Only 3 elements
            )

    async def test_oriented_bounding_box_non_uniform_scale(self) -> None:
        """Test that non-uniform scale on oriented bounding box raises an error."""
        world_interface = cu_mg.CumotionWorldInterface()

        # OBBs must have uniform scale
        centers = wp.array([[0.0, 0.0, 0.0]], dtype=wp.float32)
        rotations = wp.array([[1.0, 0.0, 0.0, 0.0]], dtype=wp.float32)  # Identity quaternion (w, x, y, z)
        half_side_lengths = wp.array([[0.5, 0.5, 0.5]], dtype=wp.float32)

        with self.assertRaises(ValueError):
            world_interface.add_oriented_bounding_boxes(
                prim_paths=["/World/OBB"],
                centers=centers,
                rotations=rotations,
                half_side_lengths=half_side_lengths,
                scales=wp.array([[1.0, 2.0, 1.0]], dtype=wp.float32),  # Non-uniform scale
                safety_tolerances=wp.array([[0.0]], dtype=wp.float32),
                poses=(
                    wp.array([[0.0, 0.0, 0.5]], dtype=wp.float32),
                    wp.array([[1.0, 0.0, 0.0, 0.0]], dtype=wp.float32),
                ),
                enabled_array=wp.array([True], dtype=bool),
            )

    async def test_oriented_bounding_box_negative_half_side_lengths(self) -> None:
        """Test that negative half side lengths for OBB raise an error."""
        world_interface = cu_mg.CumotionWorldInterface()

        centers = wp.array([[0.0, 0.0, 0.0]], dtype=wp.float32)
        rotations = wp.array([[1.0, 0.0, 0.0, 0.0]], dtype=wp.float32)  # Identity quaternion (w, x, y, z)
        half_side_lengths = wp.array([[-0.5, 0.5, 0.5]], dtype=wp.float32)  # Negative value

        with self.assertRaises(ValueError):
            world_interface.add_oriented_bounding_boxes(
                prim_paths=["/World/OBB"],
                centers=centers,
                rotations=rotations,
                half_side_lengths=half_side_lengths,
                scales=wp.array([[1.0, 1.0, 1.0]], dtype=wp.float32),
                safety_tolerances=wp.array([[0.0]], dtype=wp.float32),
                poses=(
                    wp.array([[0.0, 0.0, 0.5]], dtype=wp.float32),
                    wp.array([[1.0, 0.0, 0.0, 0.0]], dtype=wp.float32),
                ),
                enabled_array=wp.array([True], dtype=bool),
            )

    async def test_mesh_points_with_wrong_dimensions(self) -> None:
        """Test that mesh points with wrong number of coordinates raise an error."""
        world_interface = cu_mg.CumotionWorldInterface()

        # Points with only 2 coordinates instead of 3
        points = [wp.array([[0.0, 0.0], [1.0, 0.0], [0.5, 1.0]], dtype=wp.float32)]
        face_vertex_indices = [wp.array([[0, 1, 2]], dtype=wp.int32)]

        with self.assertRaises(ValueError):
            world_interface.add_triangulated_meshes(
                prim_paths=["/World/Mesh"],
                points=points,
                face_vertex_indices=face_vertex_indices,
                scales=wp.array([[1.0, 1.0, 1.0]], dtype=wp.float32),
                safety_tolerances=wp.array([[0.01]], dtype=wp.float32),
                poses=(
                    wp.array([[0.0, 0.0, 0.5]], dtype=wp.float32),
                    wp.array([[1.0, 0.0, 0.0, 0.0]], dtype=wp.float32),
                ),
                enabled_array=wp.array([True], dtype=bool),
            )

    # ============================================================================
    # device dispatch
    # ============================================================================

    def _add_three_spheres(self, world_interface: "cu_mg.CumotionWorldInterface") -> list[str]:
        """Register three spheres on ``world_interface`` and return their prim paths."""
        prim_paths = ["/World/Sphere0", "/World/Sphere1", "/World/Sphere2"]
        positions = wp.array([[0.5, 0.0, 0.5], [0.0, 0.5, 0.6], [-0.3, 0.2, 0.4]], dtype=wp.float32)
        quaternions = wp.array(
            [
                [1.0, 0.0, 0.0, 0.0],
                [np.cos(np.pi / 6), 0.0, 0.0, np.sin(np.pi / 6)],
                [np.cos(np.pi / 4), 0.0, np.sin(np.pi / 4), 0.0],
            ],
            dtype=wp.float32,
        )
        world_interface.add_spheres(
            prim_paths=prim_paths,
            radii=wp.array([[0.1], [0.05], [0.08]], dtype=wp.float32),
            scales=wp.array([[1.0, 1.0, 1.0]] * 3, dtype=wp.float32),
            safety_tolerances=wp.array([[0.0]] * 3, dtype=wp.float32),
            poses=(positions, quaternions),
            enabled_array=wp.array([True, True, True], dtype=bool),
        )
        return prim_paths

    async def test_device_explicit_cpu_runs_through_update(self) -> None:
        """``device='cpu'`` exercises the NumPy path end-to-end without raising."""
        world_interface = cu_mg.CumotionWorldInterface(device="cpu")
        prim_paths = self._add_three_spheres(world_interface)

        # Drive the full hot path so the CPU dispatch branch runs.
        world_interface.update_obstacle_transforms(
            prim_paths=prim_paths,
            poses=(
                wp.array([[0.6, 0.0, 0.5], [0.1, 0.5, 0.6], [-0.2, 0.2, 0.4]], dtype=wp.float32),
                wp.array([[1.0, 0.0, 0.0, 0.0]] * 3, dtype=wp.float32),
            ),
        )

        # State arrays should reflect the new world-to-object positions.
        for prim_path, expected in zip(prim_paths, [[0.6, 0.0, 0.5], [0.1, 0.5, 0.6], [-0.2, 0.2, 0.4]]):
            idx = world_interface._prim_path_to_collision_data[prim_path].transform_world_to_object_index
            self.assertTrue(np.allclose(world_interface._position_world_to_objects[idx, :], expected, atol=1e-5))

    async def test_device_explicit_cuda_runs_through_update(self) -> None:
        """``device='cuda'`` exercises the Warp kernel path end-to-end."""
        if not wp.is_cuda_available():
            self.skipTest("CUDA device not available")

        world_interface = cu_mg.CumotionWorldInterface(device="cuda")
        prim_paths = self._add_three_spheres(world_interface)

        world_interface.update_obstacle_transforms(
            prim_paths=prim_paths,
            poses=(
                wp.array([[0.6, 0.0, 0.5], [0.1, 0.5, 0.6], [-0.2, 0.2, 0.4]], dtype=wp.float32),
                wp.array([[1.0, 0.0, 0.0, 0.0]] * 3, dtype=wp.float32),
            ),
        )

        for prim_path, expected in zip(prim_paths, [[0.6, 0.0, 0.5], [0.1, 0.5, 0.6], [-0.2, 0.2, 0.4]]):
            idx = world_interface._prim_path_to_collision_data[prim_path].transform_world_to_object_index
            self.assertTrue(np.allclose(world_interface._position_world_to_objects[idx, :], expected, atol=1e-5))

    async def test_device_none_resolves_to_warp_default(self) -> None:
        """``device=None`` (the default) resolves to ``wp.get_device()``."""
        world_interface = cu_mg.CumotionWorldInterface()
        self.assertEqual(world_interface._device, wp.get_device())

    async def test_device_invalid_value_raises(self) -> None:
        """Strings that warp can't resolve to a Device are rejected at construction."""
        for bad in ("auto", "gpu", "", "CPU"):
            with self.assertRaises(ValueError):
                cu_mg.CumotionWorldInterface(device=bad)  # type: ignore[arg-type]

    async def test_cpu_and_cuda_produce_same_obstacle_pose_state(self) -> None:
        """Both backends must agree on the per-obstacle world-frame state they store."""
        if not wp.is_cuda_available():
            self.skipTest("CUDA device not available")

        new_positions = wp.array([[0.6, 0.0, 0.5], [0.1, 0.5, 0.6], [-0.2, 0.2, 0.4]], dtype=wp.float32)
        new_quaternions = wp.array(
            [
                [1.0, 0.0, 0.0, 0.0],
                [np.cos(np.pi / 8), 0.0, np.sin(np.pi / 8), 0.0],
                [np.cos(np.pi / 5), np.sin(np.pi / 5), 0.0, 0.0],
            ],
            dtype=wp.float32,
        )

        cpu_iface = cu_mg.CumotionWorldInterface(device="cpu")
        gpu_iface = cu_mg.CumotionWorldInterface(device="cuda")
        cpu_paths = self._add_three_spheres(cpu_iface)
        gpu_paths = self._add_three_spheres(gpu_iface)

        cpu_iface.update_obstacle_transforms(prim_paths=cpu_paths, poses=(new_positions, new_quaternions))
        gpu_iface.update_obstacle_transforms(prim_paths=gpu_paths, poses=(new_positions, new_quaternions))

        self.assertTrue(
            np.allclose(cpu_iface._position_world_to_objects, gpu_iface._position_world_to_objects, atol=1e-5)
        )
        self.assertTrue(
            np.allclose(
                cpu_iface._quaternion_world_to_objects,
                gpu_iface._quaternion_world_to_objects,
                atol=1e-5,
            )
        )

    async def test_device_cpu_with_debug_visualization(self) -> None:
        """The CPU dispatch branch must work when debug visualization is enabled."""
        world_interface = cu_mg.CumotionWorldInterface(
            device="cpu",
            visualize_debug_prims=True,
            visual_debug_enabled_prim_rgb=[1.0, 0.0, 0.0],
        )
        prim_paths = self._add_three_spheres(world_interface)

        # Should run end-to-end without raising; debug prims may exist on the stage.
        world_interface.update_obstacle_transforms(
            prim_paths=prim_paths,
            poses=(
                wp.array([[0.6, 0.0, 0.5], [0.1, 0.5, 0.6], [-0.2, 0.2, 0.4]], dtype=wp.float32),
                wp.array([[1.0, 0.0, 0.0, 0.0]] * 3, dtype=wp.float32),
            ),
        )

    async def test_device_cuda_with_debug_visualization(self) -> None:
        """The CUDA dispatch branch must still work when debug visualization is enabled."""
        if not wp.is_cuda_available():
            self.skipTest("CUDA device not available")

        world_interface = cu_mg.CumotionWorldInterface(
            device="cuda",
            visualize_debug_prims=True,
            visual_debug_enabled_prim_rgb=[1.0, 0.0, 0.0],
        )
        prim_paths = self._add_three_spheres(world_interface)

        world_interface.update_obstacle_transforms(
            prim_paths=prim_paths,
            poses=(
                wp.array([[0.6, 0.0, 0.5], [0.1, 0.5, 0.6], [-0.2, 0.2, 0.4]], dtype=wp.float32),
                wp.array([[1.0, 0.0, 0.0, 0.0]] * 3, dtype=wp.float32),
            ),
        )

    # ============================================================================
    # get_world_to_robot_base_transform: documented caching contract
    # ============================================================================

    async def test_world_to_robot_base_transform_caches_until_invalidated(self) -> None:
        """``get_world_to_robot_base_transform`` returns the same wp.array tuple until the base moves."""
        world_interface = cu_mg.CumotionWorldInterface()
        first_pos, first_quat = world_interface.get_world_to_robot_base_transform()
        second_pos, second_quat = world_interface.get_world_to_robot_base_transform()
        self.assertIs(first_pos, second_pos)
        self.assertIs(first_quat, second_quat)

        # Updating the base pose must invalidate the cache.
        world_interface.update_world_to_robot_root_transforms(
            poses=(
                wp.array([[1.0, 0.0, 0.0]], dtype=wp.float32),
                wp.array([[1.0, 0.0, 0.0, 0.0]], dtype=wp.float32),
            )
        )
        third_pos, third_quat = world_interface.get_world_to_robot_base_transform()
        self.assertIsNot(third_pos, first_pos)
        self.assertIsNot(third_quat, first_quat)

    # ============================================================================
    # update_world_to_robot_root_transforms: CPU/CUDA smoke coverage
    # ============================================================================

    async def test_root_transform_update_cpu_smoke(self) -> None:
        """The CPU branch of ``_update_world_to_robot_root_transforms`` runs end-to-end."""
        world_interface = cu_mg.CumotionWorldInterface(device="cpu")
        self._add_three_spheres(world_interface)

        # Identity, translated, rotated, and combined base poses should all succeed.
        sqrt2_2 = float(np.sqrt(2.0) / 2.0)
        for pos, quat in (
            ([0.0, 0.0, 0.0], [1.0, 0.0, 0.0, 0.0]),
            ([1.0, 2.0, 0.0], [1.0, 0.0, 0.0, 0.0]),
            ([0.0, 0.0, 0.0], [sqrt2_2, 0.0, 0.0, sqrt2_2]),
            ([0.5, -0.5, 0.25], [sqrt2_2, sqrt2_2, 0.0, 0.0]),
        ):
            world_interface.update_world_to_robot_root_transforms(
                poses=(
                    wp.array([pos], dtype=wp.float32),
                    wp.array([quat], dtype=wp.float32),
                )
            )

    async def test_root_transform_update_cuda_smoke(self) -> None:
        """The CUDA branch of ``update_world_to_robot_root_transforms`` runs end-to-end."""
        if not wp.is_cuda_available():
            self.skipTest("CUDA device not available")

        world_interface = cu_mg.CumotionWorldInterface(device="cuda")
        self._add_three_spheres(world_interface)

        sqrt2_2 = float(np.sqrt(2.0) / 2.0)
        for pos, quat in (
            ([0.0, 0.0, 0.0], [1.0, 0.0, 0.0, 0.0]),
            ([1.0, 2.0, 0.0], [1.0, 0.0, 0.0, 0.0]),
            ([0.0, 0.0, 0.0], [sqrt2_2, 0.0, 0.0, sqrt2_2]),
            ([0.5, -0.5, 0.25], [sqrt2_2, sqrt2_2, 0.0, 0.0]),
        ):
            world_interface.update_world_to_robot_root_transforms(
                poses=(
                    wp.array([pos], dtype=wp.float32),
                    wp.array([quat], dtype=wp.float32),
                )
            )
