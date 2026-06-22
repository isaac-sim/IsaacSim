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

# Epsilon for treating a segment as degenerate (zero length).
_DEGENERATE_SEGMENT_EPS = 1e-10
# Epsilon to avoid division by zero in segment parameter and interpolation.
_SEGMENT_DIV_EPS = 1e-6
# Initial sentinel for minimum distance in find_nearest.
_INIT_MIN_DIST = 1e9


def _nearest_point_on_segment(a: np.ndarray, b: np.ndarray, c: np.ndarray) -> tuple[np.ndarray, float]:
    """Find the nearest point on a line segment to a given point (internal).

    Computes the closest point on the line segment AB to point C, along with the
    distance from A to that closest point along the segment direction.

    Args:
        a: Start point of the line segment.
        b: End point of the line segment.
        c: Query point to find the nearest point on the segment.

    Returns:
        A tuple containing the nearest point on the segment and the distance
        from point A to the nearest point along the segment direction.
    """
    a2b = b - a
    a2c = c - a
    a2b_mag = np.linalg.norm(a2b)
    if a2b_mag < _DEGENERATE_SEGMENT_EPS:
        return a.copy(), 0.0
    a2b_norm = a2b / a2b_mag
    dist = np.dot(a2c, a2b_norm)
    if dist < 0:
        return a.copy(), dist
    if dist > a2b_mag:
        return b.copy(), dist
    return a + a2b_norm * dist, dist


class PathHelper:
    """Utility for querying path length, segment by distance, and nearest point on path.

    Args:
        points: Array of path points with shape ``(N, 2)`` or ``(N, 3)``.
    """

    def __init__(self, points: np.ndarray) -> None:
        self._points = points
        self._init_point_distances()

    def _init_point_distances(self) -> None:
        """Compute cumulative distances from the start of the path to each point."""
        n = len(self._points)
        self._point_distances = np.zeros(n)
        if n < 2:
            return
        diffs = np.diff(self._points, axis=0)
        lengths = np.linalg.norm(diffs, axis=1)
        self._point_distances[1:] = np.cumsum(lengths)

    def get_path_length(self) -> float:
        """Total length of the path. O(1) after initialization.

        Returns:
            Total path length.
        """
        if len(self._points) < 2:
            return 0.0
        return float(self._point_distances[-1])

    def _get_segment_by_distance(self, distance: float) -> tuple[int, int]:
        """Return segment indices (i, i+1) such that distance lies in [d[i], d[i+1]). O(log n).

        Args:
            distance: Distance along the path.

        Returns:
            Segment endpoint indices.
        """
        n = len(self._points)
        if n < 2:
            return (0, 0)
        if distance <= 0:
            return (0, 1)
        path_len = self._point_distances[-1]
        if distance >= path_len:
            return (n - 2, n - 1)
        i = np.searchsorted(self._point_distances, distance, side="right") - 1
        i = max(0, min(i, n - 2))
        return (i, i + 1)

    def get_point_by_distance(self, distance: float) -> np.ndarray:
        """Interpolated point at the given distance along the path.

        Args:
            distance: Distance along the path.

        Returns:
            Interpolated path point.
        """
        a_idx, b_idx = self._get_segment_by_distance(distance)
        a, b = self._points[a_idx], self._points[b_idx]
        a_dist, b_dist = self._point_distances[a_idx], self._point_distances[b_idx]
        seg_len = (b_dist - a_dist) + _SEGMENT_DIV_EPS
        u = np.clip((distance - a_dist) / seg_len, 0.0, 1.0)
        return a + u * (b - a)

    def find_nearest(self, point: np.ndarray) -> tuple[np.ndarray, float, tuple[int, int], float]:
        """Nearest point on the path to the given point.

        Args:
            point: Query point.

        Returns:
            (nearest_point, distance_along_path, segment_indices, distance_to_path).
        """
        if len(self._points) == 0:
            return (
                np.array([], dtype=np.float64),
                0.0,
                (0, 0),
                0.0,
            )
        if len(self._points) == 1:
            pt = self._points[0]
            return pt.copy(), 0.0, (0, 0), float(np.linalg.norm(point - pt))

        min_pt_dist_to_seg = _INIT_MIN_DIST
        min_pt_seg = (0, 1)
        min_pt = self._points[0].copy()
        min_pt_dist_along_path = 0.0

        for a_idx in range(len(self._points) - 1):
            b_idx = a_idx + 1
            a = self._points[a_idx]
            b = self._points[b_idx]
            nearest_pt, dist_along_seg = _nearest_point_on_segment(a, b, point)
            dist_to_seg = np.linalg.norm(point - nearest_pt)

            if dist_to_seg < min_pt_dist_to_seg:
                min_pt_seg = (a_idx, b_idx)
                min_pt_dist_to_seg = dist_to_seg
                min_pt = nearest_pt
                min_pt_dist_along_path = self._point_distances[a_idx] + dist_along_seg

        return min_pt, min_pt_dist_along_path, min_pt_seg, min_pt_dist_to_seg
