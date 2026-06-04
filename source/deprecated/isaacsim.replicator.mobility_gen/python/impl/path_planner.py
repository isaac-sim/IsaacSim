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

"""Path planning utilities for generating and optimizing navigation paths in occupancy grids."""

import random
from dataclasses import dataclass

import numpy as np

from ..bindings import _path_planner


@dataclass
class GeneratePathsOutput:
    """Output data structure from path generation containing navigation information.

    Stores the results of pathfinding operations including visited cells, distances,
    and parent pointers for path reconstruction.

    Args:
        visited: Array marking which cells have been visited during pathfinding.
        distance_to_start: Array containing distances from each cell to the start position.
        prev_i: Array storing the row indices of parent cells for path reconstruction.
        prev_j: Array storing the column indices of parent cells for path reconstruction.
    """

    visited: np.ndarray
    distance_to_start: np.ndarray
    prev_i: np.ndarray
    prev_j: np.ndarray

    def unroll_path(self, end: tuple[int, int]) -> np.ndarray:
        """Unrolls a path from the start point to a specified end point.

        Args:
            end: The (i, j) coordinates of the end point to unroll the path to.

        Returns:
            Array of path coordinates from start to the specified end point.
        """
        end = np.array([end[0], end[1]], dtype=np.int64)
        path = _path_planner.unroll_path(end, self.prev_i, self.prev_j)
        return np.array(path)

    def get_valid_end_points(self) -> tuple:
        """Get coordinates of all valid end points that can be reached from the start.

        Returns:
            Tuple of (i_indices, j_indices) arrays containing coordinates of visited points.
        """
        return np.where(self.visited != 0)

    def sample_random_end_point(self) -> tuple[int, int]:
        """Samples a random end point from all valid reachable points.

        Returns:
            Random (i, j) coordinates of a valid end point.
        """
        i, j = self.get_valid_end_points()
        index = random.randint(0, len(i) - 1)
        return (int(i[index]), int(j[index]))

    def sample_random_path(self) -> np.ndarray:
        """Generate a random path by sampling a random end point and unrolling the path to it.

        Returns:
            Array of path coordinates from start to a randomly selected end point.
        """
        end = self.sample_random_end_point()
        return self.unroll_path(end)


def generate_paths(start: tuple[int, int], freespace: np.ndarray) -> GeneratePathsOutput:
    """Generate shortest paths from a starting position to all reachable points in the freespace.

    Uses pathfinding algorithm to compute distances and predecessor information for path reconstruction
    from the start position to any reachable location in the provided freespace map.

    Args:
        start: Starting position coordinates (row, column) in the freespace grid.
        freespace: Binary occupancy grid where non-zero values represent navigable space.

    Returns:
        Path generation results containing visited nodes, distances, and predecessor information.
    """
    start = np.array([start[0], start[1]], dtype=np.int64)
    freespace = freespace.astype(np.uint8)
    visited = np.zeros(freespace.shape, dtype=np.uint8)
    distance_to_start = np.zeros(freespace.shape, dtype=np.float64)
    prev_i = -np.ones((freespace.shape), dtype=np.int64)
    prev_j = -np.ones((freespace.shape), dtype=np.int64)

    _path_planner.generate_paths(start, freespace, visited, distance_to_start, prev_i, prev_j)

    return GeneratePathsOutput(visited=visited, distance_to_start=distance_to_start, prev_i=prev_i, prev_j=prev_j)


def compress_path(path: np.ndarray, eps: float = 1e-3) -> tuple:
    """Compress a path by removing redundant points that lie approximately on a straight line.

    The function identifies path points that deviate minimally from the line connecting their neighbors
    and removes them to simplify the path while preserving its overall shape.

    Args:
        path: Path coordinates as array of shape (N, 2) or (N, 3).
        eps: Threshold for point deviation. Points with squared deviation below this value are removed.

    Returns:
        A tuple containing the compressed path array and a boolean mask indicating which points were kept.
    """
    pref = path[1:-1]
    pnext = path[2:]
    pprev = path[:-2]

    vnext = pnext - pref
    vprev = pref - pprev

    keepmask = np.ones((path.shape[0],), dtype=bool)  # keep beginning / end by default
    keepmask[1:-1] = np.sum((vnext - vprev) ** 2, axis=-1) > eps

    return path[keepmask], keepmask
