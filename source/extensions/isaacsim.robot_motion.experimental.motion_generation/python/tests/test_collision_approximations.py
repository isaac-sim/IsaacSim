# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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

"""Tests for collision approximation utilities used in robot motion generation."""

import numpy as np
import omni.kit.test
import omni.timeline
from isaacsim.core.experimental.objects import Cube, Mesh
from isaacsim.core.experimental.utils.stage import create_new_stage_async
from isaacsim.robot_motion.experimental.motion_generation.impl.utils.collision_approximation import (  # compute_mesh_convex_hull,
    compute_obb,
    compute_world_aabb,
    create_bbox_cache,
    triangulate_mesh,
)
from omni.kit.app import get_app


class TestCollisionApproximations(omni.kit.test.AsyncTestCase):
    """Test suite for collision approximation utilities in motion generation.

    This test class validates the functionality of collision approximation algorithms used in robot motion
    generation, including oriented bounding boxes (OBB), world axis-aligned bounding boxes (AABB), and mesh
    triangulation. The tests ensure that geometric approximations are computed correctly for various mesh and
    geometric primitive configurations.

    The test suite covers:
    - Oriented bounding box computation for meshes with unreferenced vertices
    - World axis-aligned bounding box computation and comparison between cube and mesh primitives
    - Mesh triangulation error handling for malformed geometry
    - Geometric transformations including position, orientation, and scale effects on bounding calculations

    Each test creates stage geometry using Isaac Sim's experimental object system and validates the collision
    approximation results against expected geometric properties.
    """

    # Before running each test
    async def setUp(self):
        """Set up test environment before each test."""
        await create_new_stage_async()

        await get_app().next_update_async()

        # Initialize timeline
        self._timeline = omni.timeline.get_timeline_interface()

    # After running each test
    async def tearDown(self):
        """Clean up test environment after each test."""
        # Stop timeline if running
        if self._timeline.is_playing():
            self._timeline.stop()

        # Clean up physics callbacks if any
        if hasattr(self, "sample") and self.sample:
            self.sample.physics_cleanup()

        await get_app().next_update_async()

    async def test_oriented_bounding_box(self):
        """Test that oriented bounding box computation correctly handles mesh geometry.

        Verifies that the OBB calculation ignores unreferenced vertices and produces correct
        bounding box dimensions for a cube mesh.
        """
        # create a mesh in the scene:
        stage_mesh = Mesh(
            paths="/World/Mesh",
            positions=[1.0, 2.0, 3.0],
            orientations=[1.0, 0.0, 0.0, 0.0],
            scales=[3.0, 2.0, 1.0],
        )

        # NOTE: here, we will put
        # an unreferenced point.
        mesh_points = [
            [
                (-0.5, -0.5, -0.5),
                (0.5, -0.5, -0.5),
                (0.5, 0.5, -0.5),
                (-0.5, 0.5, -0.5),
                (-0.5, -0.5, 0.5),
                (0.5, -0.5, 0.5),
                (0.5, 0.5, 0.5),
                (-0.5, 0.5, 0.5),
                # POINT WHICH IS NOT REFERENCED:
                (0.0, 0.0, 0.0),
            ]
        ]
        mesh_face_counts = [[4, 4, 4, 4, 4, 4]]
        mesh_face_indices = [
            [
                0,
                1,
                2,
                3,
                4,
                5,
                6,
                7,
                0,
                4,
                7,
                3,
                1,
                5,
                6,
                2,
                0,
                1,
                5,
                4,
                3,
                2,
                6,
                7,
            ]
        ]
        mesh_normals = [
            [
                (0.0, 0.0, -1.0),
                (0.0, 0.0, 1.0),
                (-1.0, 0.0, 0.0),
                (1.0, 0.0, 0.0),
                (0.0, -1.0, 0.0),
                (0.0, 1.0, 0.0),
            ]
        ]
        stage_mesh.set_points(mesh_points)
        stage_mesh.set_face_specs(vertex_indices=mesh_face_indices, vertex_counts=mesh_face_counts)
        stage_mesh.set_normals(mesh_normals)

        await get_app().next_update_async()

        # OBB:
        bbox_cache = create_bbox_cache()
        obb = compute_obb(bbox_cache=bbox_cache, prim_path="/World/Mesh")

        # even with the unreferenced point, the bounding box of
        # this mesh should be correct, and unaffected by the
        # position, orientation or scale of the Mesh object:
        self.assertTrue(np.isclose(obb.center, [0.0, 0.0, 0.0]).all())
        self.assertTrue(np.isclose(obb.half_side_lengths, [0.5, 0.5, 0.5]).all())

    async def test_world_axis_aligned_bounding_box(self):
        """Test that world axis-aligned bounding box computation produces consistent results.

        Verifies that AABB calculations for geometrically equivalent Cube and Mesh objects
        with identical transformations produce matching bounding boxes.
        """
        positions = [1.0, 2.0, 3.0]
        q_ = np.array([0.5, 0.1, 0.6, -0.9])
        orientations = q_ / np.linalg.norm(q_)
        scales = [3.0, 2.0, 1.0]
        size = 1.5

        # create a cube in the scene:
        stage_cube = Cube(
            paths="/World/Cube", positions=positions, orientations=orientations, scales=scales, sizes=size
        )

        # create a mesh in the scene
        stage_mesh = Mesh(
            paths="/World/Mesh",
            positions=positions,
            orientations=orientations,
            scales=scales,
        )

        # NOTE: here, we will put
        # an unreferenced point.
        # Otherwise, this mesh and the Cube
        # represent identical geometry.
        mesh_points = [
            [
                (-size / 2, -size / 2, -size / 2),
                (size / 2, -size / 2, -size / 2),
                (size / 2, size / 2, -size / 2),
                (-size / 2, size / 2, -size / 2),
                (-size / 2, -size / 2, size / 2),
                (size / 2, -size / 2, size / 2),
                (size / 2, size / 2, size / 2),
                (-size / 2, size / 2, size / 2),
                # POINT WHICH IS NOT REFERENCED:
                (0.0, 0.0, 0.0),
            ]
        ]
        mesh_face_counts = [[4, 4, 4, 4, 4, 4]]
        mesh_face_indices = [
            [
                0,
                1,
                2,
                3,
                4,
                5,
                6,
                7,
                0,
                4,
                7,
                3,
                1,
                5,
                6,
                2,
                0,
                1,
                5,
                4,
                3,
                2,
                6,
                7,
            ]
        ]
        mesh_normals = [
            [
                (0.0, 0.0, -1.0),
                (0.0, 0.0, 1.0),
                (-1.0, 0.0, 0.0),
                (1.0, 0.0, 0.0),
                (0.0, -1.0, 0.0),
                (0.0, 1.0, 0.0),
            ]
        ]
        stage_mesh.set_points(mesh_points)
        stage_mesh.set_face_specs(vertex_indices=mesh_face_indices, vertex_counts=mesh_face_counts)
        stage_mesh.set_normals(mesh_normals)

        await get_app().next_update_async()

        # AABB:
        bbox_cache = create_bbox_cache()
        aabb_mesh = compute_world_aabb(bbox_cache=bbox_cache, prim_path="/World/Mesh")

        aabb_cube = compute_world_aabb(bbox_cache=bbox_cache, prim_path="/World/Cube")

        # These two AABBs should exactly match:
        self.assertTrue(np.isclose(aabb_cube.min_bounds, aabb_mesh.min_bounds).all())
        self.assertTrue(np.isclose(aabb_cube.max_bounds, aabb_mesh.max_bounds).all())

        print("-" * 100)
        print("COMPUTED AABBs")
        print(f"aabb_cube: {aabb_cube}")
        print(f"aabb_mesh: {aabb_mesh}")
        print("-" * 100)

    # TODO: ADD BACK IN WHEN WE WANT TO ADD CONVEX HULL
    # async def test_mesh_convex_hull(self):
    #     # create a mesh in the scene:
    #     stage_mesh = Mesh(
    #         paths="/World/Mesh",
    #         positions=[1.0, 2.0, 3.0],
    #         orientations=[1.0, 0.0, 0.0, 0.0],
    #         scales=[3.0, 2.0, 1.0],
    #     )

    #     # NOTE: here, we will put
    #     # an unreferenced point.
    #     mesh_points = [
    #         [
    #             (-0.5, -0.5, -0.5),
    #             (0.5, -0.5, -0.5),
    #             (0.5, 0.5, -0.5),
    #             (-0.5, 0.5, -0.5),
    #             (-0.5, -0.5, 0.5),
    #             (0.5, -0.5, 0.5),
    #             (0.5, 0.5, 0.5),
    #             (-0.5, 0.5, 0.5),
    #             # POINT WHICH IS NOT REFERENCED:
    #             (0.0, 0.0, 0.0),
    #         ]
    #     ]
    #     mesh_face_counts = [[4, 4, 4, 4, 4, 4]]
    #     mesh_face_indices = [
    #         [
    #             0,
    #             1,
    #             2,
    #             3,
    #             4,
    #             5,
    #             6,
    #             7,
    #             0,
    #             4,
    #             7,
    #             3,
    #             1,
    #             5,
    #             6,
    #             2,
    #             0,
    #             1,
    #             5,
    #             4,
    #             3,
    #             2,
    #             6,
    #             7,
    #         ]
    #     ]
    #     mesh_normals = [
    #         [
    #             (0.0, 0.0, -1.0),
    #             (0.0, 0.0, 1.0),
    #             (-1.0, 0.0, 0.0),
    #             (1.0, 0.0, 0.0),
    #             (0.0, -1.0, 0.0),
    #             (0.0, 1.0, 0.0),
    #         ]
    #     ]
    #     stage_mesh.set_points(mesh_points)
    #     stage_mesh.set_face_specs(vertex_indices=mesh_face_indices, vertex_counts=mesh_face_counts)
    #     stage_mesh.set_normals(mesh_normals)

    #     await get_app().next_update_async()

    #     # compute the convex hull of this mesh:
    #     convex_hull_data = compute_mesh_convex_hull(stage_mesh)
    #     self.assertIsNotNone(convex_hull_data.points)
    #     self.assertIsNotNone(convex_hull_data.triangles)

    async def test_triangulate_mesh_no_faces_raises(self):
        """Test that triangulate_mesh raises ValueError for meshes without faces.

        Verifies that attempting to triangulate a mesh with no face specifications
        raises the expected ValueError exception.
        """
        stage_mesh = Mesh(paths="/World/EmptyMesh")
        stage_mesh.set_points([[(0.0, 0.0, 0.0)]])
        stage_mesh.set_face_specs(vertex_indices=[[]], vertex_counts=[[]])

        await get_app().next_update_async()

        self.assertRaises(ValueError, triangulate_mesh, stage_mesh)
