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

"""Path planning utilities using a C++ BFS backend."""

from __future__ import annotations

import random
from dataclasses import dataclass

import numpy as np

from ..bindings import _path_planner


@dataclass
class GeneratePathsOutput:
    """Output of the generate_paths BFS search.

    Args:
        visited: Boolean grid of visited cells.
        distance_to_start: Grid of distances from the start cell.
        prev_i: Grid of previous row indices for path reconstruction.
        prev_j: Grid of previous column indices for path reconstruction.
    """

    visited: np.ndarray
    distance_to_start: np.ndarray
    prev_i: np.ndarray
    prev_j: np.ndarray

    def unroll_path(self, end: tuple[int, int]) -> np.ndarray:
        """Unroll the BFS tree to produce a path from start to the given end cell.

        Args:
            end: The (row, col) index of the end cell.

        Returns:
            An Nx2 array of (row, col) indices from start to end.
        """
        end = np.array([end[0], end[1]], dtype=np.int64)
        path = _path_planner.unroll_path(end, self.prev_i, self.prev_j)
        return np.array(path)

    def get_valid_end_points(self) -> tuple[np.ndarray, ...]:
        """Return the indices of all reachable cells.

        Returns:
            A tuple of arrays of (row, col) indices for all visited cells.
        """
        return np.where(self.visited != 0)

    def sample_random_end_point(self) -> tuple[int, int]:
        """Sample a random reachable end point from the BFS result.

        Returns:
            A (row, col) index tuple for a randomly selected reachable cell.
        """
        i, j = self.get_valid_end_points()
        if len(i) == 0:
            raise RuntimeError("BFS found no reachable cells — start position may be blocked or outside freespace.")
        index = random.randint(0, len(i) - 1)
        return (int(i[index]), int(j[index]))

    def sample_random_path(self) -> np.ndarray:
        """Sample a random path from start to a randomly chosen reachable end point.

        Returns:
            An Nx2 array of (row, col) indices from start to a random end.
        """
        end = self.sample_random_end_point()
        return self.unroll_path(end)


def generate_paths(start: tuple[int, int], freespace: np.ndarray) -> GeneratePathsOutput:
    """Run BFS from start over the freespace grid to generate all reachable paths.

    Args:
        start: The (row, col) starting cell index.
        freespace: A 2D boolean or uint8 array marking traversable cells.

    Returns:
        A GeneratePathsOutput containing BFS results for path reconstruction.
    """
    start = np.array([start[0], start[1]], dtype=np.int64)
    freespace = freespace.astype(np.uint8)
    visited = np.zeros(freespace.shape, dtype=np.uint8)
    distance_to_start = np.zeros(freespace.shape, dtype=np.float64)
    prev_i = -np.ones((freespace.shape), dtype=np.int64)
    prev_j = -np.ones((freespace.shape), dtype=np.int64)

    _path_planner.generate_paths(start, freespace, visited, distance_to_start, prev_i, prev_j)

    return GeneratePathsOutput(visited=visited, distance_to_start=distance_to_start, prev_i=prev_i, prev_j=prev_j)


def compress_path(path: np.ndarray, eps: float = 1e-3) -> tuple[np.ndarray, np.ndarray]:
    """Remove collinear intermediate points from a path.

    Args:
        path: An Nx2 or Nx3 array of path waypoints.
        eps: The squared-difference threshold below which a point is considered collinear.

    Returns:
        A tuple of (compressed_path, keepmask) where compressed_path contains only
        non-collinear points and keepmask is a boolean array marking kept points.
    """
    pref = path[1:-1]
    pnext = path[2:]
    pprev = path[:-2]

    vnext = pnext - pref
    vprev = pref - pprev

    keepmask = np.ones((path.shape[0],), dtype=bool)  # keep beginning / end by default
    keepmask[1:-1] = np.sum((vnext - vprev) ** 2, axis=-1) > eps

    return path[keepmask], keepmask
