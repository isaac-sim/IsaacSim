# SPDX-FileCopyrightText: Copyright (c) 2025-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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

"""Verifies bounding-box utilities for axis-aligned and oriented bounds. Covers cache creation, single and combined AABBs, OBBs, OBB corners, invalid prim handling, child inclusion, and transformed cubes."""

import isaacsim.core.experimental.utils.bounds as bounds_utils
import isaacsim.core.experimental.utils.stage as stage_utils
import numpy as np
import omni.kit.test
from isaacsim.core.experimental.objects import Cube
from pxr import UsdGeom


class TestBounds(omni.kit.test.AsyncTestCase):
    """Test bounds."""

    async def setUp(self) -> None:
        """Method called to prepare the test fixture."""
        super().setUp()
        await stage_utils.create_new_stage_async()
        self.tolerance = 1e-5

    async def tearDown(self) -> None:
        """Method called immediately after the test method has been called."""
        super().tearDown()

    # --------------------------------------------------------------------

    async def test_create_bbox_cache(self) -> None:
        """Test create_bbox_cache returns a valid BBoxCache."""
        cache = bounds_utils.create_bbox_cache()
        self.assertIsInstance(cache, UsdGeom.BBoxCache)

    async def test_create_bbox_cache_custom_time(self) -> None:
        """Test create_bbox_cache with a custom time code."""
        from pxr import Usd

        cache = bounds_utils.create_bbox_cache(time=Usd.TimeCode(1.0))
        self.assertIsInstance(cache, UsdGeom.BBoxCache)

    # -- compute_aabb --

    async def test_compute_aabb_cube_by_path(self) -> None:
        """Test compute_aabb on a unit cube using a string path."""
        Cube("/World/Cube", sizes=1.0)
        aabb = bounds_utils.compute_aabb("/World/Cube")
        self.assertEqual(aabb.shape, (6,))
        expected = np.array([-0.5, -0.5, -0.5, 0.5, 0.5, 0.5])
        np.testing.assert_allclose(aabb, expected, atol=self.tolerance)

    async def test_compute_aabb_cube_by_prim(self) -> None:
        """Test compute_aabb on a unit cube using a prim object."""
        cube = Cube("/World/Cube", sizes=1.0)
        aabb = bounds_utils.compute_aabb(cube.prims[0])
        expected = np.array([-0.5, -0.5, -0.5, 0.5, 0.5, 0.5])
        np.testing.assert_allclose(aabb, expected, atol=self.tolerance)

    async def test_compute_aabb_with_cache(self) -> None:
        """Test compute_aabb with an externally created bbox cache."""
        Cube("/World/Cube", sizes=1.0)
        cache = bounds_utils.create_bbox_cache()
        aabb = bounds_utils.compute_aabb("/World/Cube", bbox_cache=cache)
        expected = np.array([-0.5, -0.5, -0.5, 0.5, 0.5, 0.5])
        np.testing.assert_allclose(aabb, expected, atol=self.tolerance)

    async def test_compute_aabb_invalid_prim(self) -> None:
        """Test compute_aabb raises ValueError for an invalid prim."""
        with self.assertRaises(ValueError):
            bounds_utils.compute_aabb("/World/NonExistent")

    async def test_compute_aabb_translated_cube(self) -> None:
        """Test compute_aabb with a cube offset from the origin."""
        cube = Cube("/World/Cube", sizes=1.0, positions=np.array([[10, 0, 0]]))
        aabb = bounds_utils.compute_aabb(cube.prims[0])
        expected = np.array([9.5, -0.5, -0.5, 10.5, 0.5, 0.5])
        np.testing.assert_allclose(aabb, expected, atol=self.tolerance)

    async def test_compute_aabb_include_children(self) -> None:
        """Test compute_aabb with include_children."""
        parent = stage_utils.define_prim("/World/Parent", "Xform")
        Cube("/World/Parent/CubeA", sizes=1.0)
        Cube("/World/Parent/CubeB", sizes=1.0, positions=np.array([[5, 0, 0]]))

        aabb = bounds_utils.compute_aabb(parent, include_children=True)
        expected = np.array([-0.5, -0.5, -0.5, 5.5, 0.5, 0.5])
        np.testing.assert_allclose(aabb, expected, atol=self.tolerance)

    # -- compute_combined_aabb --

    async def test_compute_combined_aabb_two_cubes(self) -> None:
        """Test compute_combined_aabb with two cubes."""
        Cube("/World/CubeA", sizes=1.0)
        Cube("/World/CubeB", sizes=1.0, positions=np.array([[3, 0, 0]]))

        aabb = bounds_utils.compute_combined_aabb(["/World/CubeA", "/World/CubeB"])
        expected = np.array([-0.5, -0.5, -0.5, 3.5, 0.5, 0.5])
        np.testing.assert_allclose(aabb, expected, atol=self.tolerance)

    async def test_compute_combined_aabb_with_prim_objects(self) -> None:
        """Test compute_combined_aabb using prim objects."""
        cube_a = Cube("/World/CubeA", sizes=1.0)
        cube_b = Cube("/World/CubeB", sizes=1.0, positions=np.array([[0, 4, 0]]))

        aabb = bounds_utils.compute_combined_aabb([cube_a.prims[0], cube_b.prims[0]])
        expected = np.array([-0.5, -0.5, -0.5, 0.5, 4.5, 0.5])
        np.testing.assert_allclose(aabb, expected, atol=self.tolerance)

    async def test_compute_combined_aabb_empty_list(self) -> None:
        """Test compute_combined_aabb raises ValueError for empty list."""
        with self.assertRaises(ValueError):
            bounds_utils.compute_combined_aabb([])

    async def test_compute_combined_aabb_with_cache(self) -> None:
        """Test compute_combined_aabb with an externally created bbox cache."""
        Cube("/World/CubeA", sizes=1.0)
        Cube("/World/CubeB", sizes=1.0)
        cache = bounds_utils.create_bbox_cache()
        aabb = bounds_utils.compute_combined_aabb(["/World/CubeA", "/World/CubeB"], bbox_cache=cache)
        expected = np.array([-0.5, -0.5, -0.5, 0.5, 0.5, 0.5])
        np.testing.assert_allclose(aabb, expected, atol=self.tolerance)

    # -- compute_obb --

    async def test_compute_obb_identity_cube(self) -> None:
        """Test compute_obb on a unit cube at the origin."""
        Cube("/World/Cube", sizes=1.0)
        centroid, axes, half_extent = bounds_utils.compute_obb("/World/Cube")

        self.assertEqual(centroid.shape, (3,))
        self.assertEqual(axes.shape, (3, 3))
        self.assertEqual(half_extent.shape, (3,))

        np.testing.assert_allclose(centroid, [0.0, 0.0, 0.0], atol=self.tolerance)
        np.testing.assert_allclose(axes, np.eye(3), atol=self.tolerance)
        np.testing.assert_allclose(half_extent, [0.5, 0.5, 0.5], atol=self.tolerance)

    async def test_compute_obb_by_prim(self) -> None:
        """Test compute_obb using a prim object."""
        cube = Cube("/World/Cube", sizes=1.0)
        centroid, axes, half_extent = bounds_utils.compute_obb(cube.prims[0])
        np.testing.assert_allclose(centroid, [0.0, 0.0, 0.0], atol=self.tolerance)
        np.testing.assert_allclose(half_extent, [0.5, 0.5, 0.5], atol=self.tolerance)

    async def test_compute_obb_translated_cube(self) -> None:
        """Test compute_obb with a translated cube."""
        cube = Cube("/World/Cube", sizes=1.0, positions=np.array([[5, 10, 15]]))

        centroid, axes, half_extent = bounds_utils.compute_obb(cube.prims[0])
        np.testing.assert_allclose(centroid, [5.0, 10.0, 15.0], atol=self.tolerance)
        np.testing.assert_allclose(half_extent, [0.5, 0.5, 0.5], atol=self.tolerance)

    async def test_compute_obb_rotated_cube(self) -> None:
        """Test compute_obb with a cube rotated 45 degrees around Z."""
        angle = np.radians(45.0)
        quat = np.array([[np.cos(angle / 2), 0, 0, np.sin(angle / 2)]])
        cube = Cube("/World/Cube", sizes=1.0, orientations=quat)

        centroid, axes, half_extent = bounds_utils.compute_obb(cube.prims[0])
        np.testing.assert_allclose(centroid, [0.0, 0.0, 0.0], atol=self.tolerance)
        np.testing.assert_allclose(half_extent, [0.5, 0.5, 0.5], atol=self.tolerance)
        # axes should be a rotation matrix (orthonormal)
        np.testing.assert_allclose(axes @ axes.T, np.eye(3), atol=self.tolerance)

    async def test_compute_obb_with_cache(self) -> None:
        """Test compute_obb with an externally created bbox cache."""
        Cube("/World/Cube", sizes=1.0)
        cache = bounds_utils.create_bbox_cache()
        centroid, axes, half_extent = bounds_utils.compute_obb("/World/Cube", bbox_cache=cache)
        np.testing.assert_allclose(centroid, [0.0, 0.0, 0.0], atol=self.tolerance)
        np.testing.assert_allclose(half_extent, [0.5, 0.5, 0.5], atol=self.tolerance)

    # -- get_obb_corners --

    async def test_get_obb_corners_identity(self) -> None:
        """Test get_obb_corners with identity axes and unit half-extent."""
        centroid = np.array([0.0, 0.0, 0.0])
        axes = np.eye(3)
        half_extent = np.array([0.5, 0.5, 0.5])

        corners = bounds_utils.get_obb_corners(centroid, axes, half_extent)
        self.assertEqual(corners.shape, (8, 3))

        expected = np.array(
            [
                [-0.5, -0.5, -0.5],
                [-0.5, -0.5, 0.5],
                [-0.5, 0.5, -0.5],
                [-0.5, 0.5, 0.5],
                [0.5, -0.5, -0.5],
                [0.5, -0.5, 0.5],
                [0.5, 0.5, -0.5],
                [0.5, 0.5, 0.5],
            ]
        )
        np.testing.assert_allclose(corners, expected, atol=self.tolerance)

    async def test_get_obb_corners_translated(self) -> None:
        """Test get_obb_corners with a non-zero centroid."""
        centroid = np.array([1.0, 2.0, 3.0])
        axes = np.eye(3)
        half_extent = np.array([0.5, 0.5, 0.5])

        corners = bounds_utils.get_obb_corners(centroid, axes, half_extent)
        expected = np.array(
            [
                [0.5, 1.5, 2.5],
                [0.5, 1.5, 3.5],
                [0.5, 2.5, 2.5],
                [0.5, 2.5, 3.5],
                [1.5, 1.5, 2.5],
                [1.5, 1.5, 3.5],
                [1.5, 2.5, 2.5],
                [1.5, 2.5, 3.5],
            ]
        )
        np.testing.assert_allclose(corners, expected, atol=self.tolerance)

    async def test_get_obb_corners_asymmetric_extents(self) -> None:
        """Test get_obb_corners with different half-extent values per axis."""
        centroid = np.array([0.0, 0.0, 0.0])
        axes = np.eye(3)
        half_extent = np.array([1.0, 2.0, 3.0])

        corners = bounds_utils.get_obb_corners(centroid, axes, half_extent)
        self.assertEqual(corners.shape, (8, 3))
        np.testing.assert_allclose(corners[0], [-1.0, -2.0, -3.0], atol=self.tolerance)
        np.testing.assert_allclose(corners[7], [1.0, 2.0, 3.0], atol=self.tolerance)

    # -- compute_obb_corners --

    async def test_compute_obb_corners_identity_cube(self) -> None:
        """Test compute_obb_corners on a unit cube at the origin."""
        Cube("/World/Cube", sizes=1.0)
        corners = bounds_utils.compute_obb_corners("/World/Cube")
        self.assertEqual(corners.shape, (8, 3))

        expected = np.array(
            [
                [-0.5, -0.5, -0.5],
                [-0.5, -0.5, 0.5],
                [-0.5, 0.5, -0.5],
                [-0.5, 0.5, 0.5],
                [0.5, -0.5, -0.5],
                [0.5, -0.5, 0.5],
                [0.5, 0.5, -0.5],
                [0.5, 0.5, 0.5],
            ]
        )
        np.testing.assert_allclose(corners, expected, atol=self.tolerance)

    async def test_compute_obb_corners_by_prim(self) -> None:
        """Test compute_obb_corners using a prim object."""
        cube = Cube("/World/Cube", sizes=1.0)
        corners = bounds_utils.compute_obb_corners(cube.prims[0])
        self.assertEqual(corners.shape, (8, 3))

    async def test_compute_obb_corners_with_cache(self) -> None:
        """Test compute_obb_corners with an externally created bbox cache."""
        Cube("/World/Cube", sizes=1.0)
        cache = bounds_utils.create_bbox_cache()
        corners = bounds_utils.compute_obb_corners("/World/Cube", bbox_cache=cache)
        expected = np.array(
            [
                [-0.5, -0.5, -0.5],
                [-0.5, -0.5, 0.5],
                [-0.5, 0.5, -0.5],
                [-0.5, 0.5, 0.5],
                [0.5, -0.5, -0.5],
                [0.5, -0.5, 0.5],
                [0.5, 0.5, -0.5],
                [0.5, 0.5, 0.5],
            ]
        )
        np.testing.assert_allclose(corners, expected, atol=self.tolerance)

    async def test_compute_obb_corners_translated_cube(self) -> None:
        """Test compute_obb_corners with a translated cube."""
        cube = Cube("/World/Cube", sizes=1.0, positions=np.array([[2, 3, 4]]))

        corners = bounds_utils.compute_obb_corners(cube.prims[0])
        expected = np.array(
            [
                [1.5, 2.5, 3.5],
                [1.5, 2.5, 4.5],
                [1.5, 3.5, 3.5],
                [1.5, 3.5, 4.5],
                [2.5, 2.5, 3.5],
                [2.5, 2.5, 4.5],
                [2.5, 3.5, 3.5],
                [2.5, 3.5, 4.5],
            ]
        )
        np.testing.assert_allclose(corners, expected, atol=self.tolerance)
