# SPDX-FileCopyrightText: Copyright (c) 2018-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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

"""Tests for path planning functionality in the mobility generation module."""

import numpy as np

# NOTE:
#   omni.kit.test - std python's unittest module with additional wrapping to add suport for async/await tests
#   For most things refer to unittest docs: https://docs.python.org/3/library/unittest.html
import omni.kit.test
import omni.usd
from isaacsim.replicator.mobility_gen.impl.path_planner import compress_path, generate_paths


# Having a test class dervived from omni.kit.test.AsyncTestCase declared on the root of module will make it auto-discoverable by omni.kit.test
class TestPathPlanner(omni.kit.test.AsyncTestCase):
    """Test suite for path planning functionality in the mobility generation module.

    This test class validates the correctness of path generation and compression algorithms used for
    mobility simulation. It includes tests for diagonal path generation in open environments,
    L-shaped path navigation around obstacles, and path compression optimization.

    The test cases cover:

    - Diagonal pathfinding in unobstructed environments
    - Navigation around obstacles requiring L-shaped routing
    - Path compression for straight-line segments
    - Path compression for paths with directional changes
    """

    # Before running each test
    async def setUp(self) -> None:
        """Set up test fixtures before running each test method."""

    # After running each test
    async def tearDown(self) -> None:
        """Clean up test fixtures after running each test method."""

    # test to make sure this runs
    async def test_generate_path_diagonal(self) -> None:
        """Test generating a diagonal path from (0,0) to (2,2) in a 3x3 free space grid."""
        start = (0, 0)
        end = (2, 2)

        freespace = np.array([[1, 1, 1], [1, 1, 1], [1, 1, 1]]).astype(np.uint8)

        output = generate_paths(start=start, freespace=freespace)

        path = output.unroll_path(end=end)

        ground_truth = np.array([[0, 0], [1, 1], [2, 2]])

        self.assertTrue(np.allclose(path, ground_truth))

    async def test_generate_path_l_shaped(self) -> None:
        """Test generating an L-shaped path when direct diagonal movement is blocked by obstacles."""
        start = (0, 0)
        end = (2, 2)

        freespace = np.array([[1, 0, 0], [1, 0, 0], [1, 1, 1]]).astype(np.uint8)

        output = generate_paths(start=start, freespace=freespace)

        path = output.unroll_path(end=end)

        ground_truth = np.array(
            [
                [0, 0],
                [1, 0],
                # [2, 0], # skip (2, 0) because path can move diagonally
                [2, 1],
                [2, 2],
            ]
        )

        self.assertTrue(np.allclose(path, ground_truth))

    async def test_compress_path_line(self) -> None:
        """Test compressing a straight line path by removing intermediate collinear points."""
        # 111 -> 1-1
        path = np.array([[0, 0], [0, 1], [0, 2]]).astype(np.float32)

        path_compressed, _ = compress_path(path)

        compressed_path_true = np.array([[0, 0], [0, 2]])

        self.assertTrue(np.allclose(path_compressed, compressed_path_true))

    async def test_compress_path_bend(self) -> None:
        """Test compressing a path with bends by removing intermediate points in straight segments."""
        # 111----      1-1----
        # ---1---   => -------
        # ----111      ----1-1
        path = np.array([[0, 0], [0, 1], [0, 2], [1, 3], [2, 4], [2, 5], [2, 6]]).astype(np.float32)

        path_compressed, _ = compress_path(path)

        compressed_path_true = np.array([[0, 0], [0, 2], [2, 4], [2, 6]])

        self.assertTrue(np.allclose(path_compressed, compressed_path_true))
