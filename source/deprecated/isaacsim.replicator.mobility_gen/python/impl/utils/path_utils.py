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

"""Utilities for path manipulation and querying operations in mobility generation."""

import numpy as np


def nearest_point_on_segment(a: np.ndarray, b: np.ndarray, c: np.ndarray) -> tuple[np.ndarray, float]:
    """Find the nearest point on a line segment to a given point.

    Computes the closest point on the line segment AB to point C, along with the distance
    from A to that closest point along the segment direction.

    Args:
        a: Start point of the line segment.
        b: End point of the line segment.
        c: Query point to find the nearest point on the segment.

    Returns:
        A tuple containing the nearest point on the segment and the distance from point A
        to the nearest point along the segment direction.
    """
    a2b = b - a
    a2c = c - a
    a2b_mag = np.sqrt(np.sum(a2b**2))
    a2b_norm = a2b / (a2b_mag + 1e-6)
    dist = np.dot(a2c, a2b_norm)
    if dist < 0:
        return a, dist
    elif dist > a2b_mag:
        return b, dist
    else:
        return a + a2b_norm * dist, dist


class PathHelper:
    """A utility class for working with paths defined by a sequence of points.

    Provides methods to query path properties, find points at specific distances along the path,
    and locate the nearest point on the path to a given query point. The class automatically
    calculates cumulative distances along the path and supports interpolation between path points.

    Args:
        points: Array of 2D or 3D points defining the path, with shape (N, 2) or (N, 3).
    """

    def __init__(self, points: np.ndarray) -> None:
        self.points = points
        self._init_point_distances()

    def _init_point_distances(self) -> None:
        """Initializes the cumulative distances from the start of the path to each point."""
        self._point_distances = np.zeros(len(self.points))
        length = 0.0
        for i in range(0, len(self.points) - 1):
            self._point_distances[i] = length
            a = self.points[i]
            b = self.points[i + 1]
            dist = np.sqrt(np.sum((a - b) ** 2))
            length += dist
        self._point_distances[-1] = length

    def point_distances(self) -> np.ndarray:
        """Cumulative distances from the start of the path to each point.

        Returns:
            Array of cumulative distances for each point in the path.
        """
        return self._point_distances

    def get_path_length(self) -> float:
        """Calculates the total length of the path.

        Returns:
            The total length of the path.
        """
        length = 0.0
        for i in range(1, len(self.points)):
            a = self.points[i - 1]
            b = self.points[i]
            dist = np.sqrt(np.sum((a - b) ** 2))
            length += dist
        return length

    def points_x(self) -> np.ndarray:
        """X coordinates of all points in the path.

        Returns:
            Array of x coordinates.
        """
        return self.points[:, 0]

    def points_y(self) -> np.ndarray:
        """Y coordinates of all points in the path.

        Returns:
            Array of y coordinates.
        """
        return self.points[:, 1]

    def get_segment_by_distance(self, distance: float) -> tuple[int, int]:
        """Finds the path segment that contains the specified distance along the path.

        Args:
            distance: Distance along the path to locate the segment for.

        Returns:
            Tuple of indices (start_index, end_index) representing the segment.
        """
        for i in range(0, len(self.points) - 1):
            d_a = self._point_distances[i]
            d_b = self._point_distances[i + 1]

            if distance < d_b:
                return (i, i + 1)

        i = len(self.points) - 2

        return (i, i + 1)

    def get_point_by_distance(self, distance: float) -> np.ndarray:
        """Calculates the interpolated point at the specified distance along the path.

        Args:
            distance: Distance along the path to get the point for.

        Returns:
            Interpolated point coordinates at the specified distance.
        """
        a_idx, b_idx = self.get_segment_by_distance(distance)
        a, b = self.points[a_idx], self.points[b_idx]
        a_dist, b_dist = self._point_distances[a_idx], self._point_distances[b_idx]
        u = (distance - a_dist) / ((b_dist - a_dist) + 1e-6)
        u = np.clip(u, 0.0, 1.0)
        return a + u * (b - a)

    def find_nearest(self, point: np.ndarray) -> tuple[np.ndarray, float, tuple[int, int], float]:
        """Finds the nearest point on the path to the given point.

        Args:
            point: Point to find the nearest path point for.

        Returns:
            A tuple of (nearest_point, distance_along_path, segment_indices, distance_to_path).
        """
        min_pt_dist_to_seg = 1e9
        min_pt_seg = None
        min_pt = None
        min_pt_dist_along_path = None

        for a_idx in range(0, len(self.points) - 1):
            b_idx = a_idx + 1
            a = self.points[a_idx]
            b = self.points[b_idx]
            nearest_pt, dist_along_seg = nearest_point_on_segment(a, b, point)
            dist_to_seg = np.sqrt(np.sum((point - nearest_pt) ** 2))

            if dist_to_seg < min_pt_dist_to_seg:
                min_pt_seg = (a_idx, b_idx)
                min_pt_dist_to_seg = dist_to_seg
                min_pt = nearest_pt
                min_pt_dist_along_path = self._point_distances[a_idx] + dist_along_seg

        return min_pt, min_pt_dist_along_path, min_pt_seg, min_pt_dist_to_seg
