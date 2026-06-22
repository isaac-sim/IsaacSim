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

"""Verifies occupancy map utility functions for coordinate conversion, bounds updates, and image generation. Covers unknown, occupied, and free-space pixels, custom colors, empty buffers, performance, and symmetry assumptions."""

from __future__ import annotations

import omni.kit.test
from isaacsim.asset.gen.omap.utils import compute_coordinates, generate_image, update_location


class MockOccupancyMap:
    """Mock occupancy map for testing utility functions.

    Args:
        min_bound: Minimum boundary coordinates.
        max_bound: Maximum boundary coordinates.
        buffer: Occupancy buffer (list of float values).
        dimensions: Map dimensions (x, y, z).
    """

    def __init__(
        self,
        min_bound: tuple[float, float, float] = (0.0, 0.0, 0.0),
        max_bound: tuple[float, float, float] = (1.0, 1.0, 1.0),
        buffer: list[float] | None = None,
        dimensions: tuple[int, int, int] = (10, 10, 1),
    ) -> None:
        self._min_bound = min_bound
        self._max_bound = max_bound
        self._buffer = buffer if buffer is not None else [0.5] * (dimensions[0] * dimensions[1])
        self._dimensions = dimensions
        self._transform_set = False
        self._cell_size = 0.05

    def get_min_bound(self) -> tuple:
        """Minimum boundary coordinates of the occupancy map.

        Returns:
            The minimum boundary coordinates as a tuple.
        """
        return self._min_bound

    def get_max_bound(self) -> tuple:
        """Maximum boundary coordinates of the occupancy map.

        Returns:
            The maximum boundary coordinates as a tuple.
        """
        return self._max_bound

    def get_buffer(self) -> list[float]:
        """Occupancy buffer containing the map data.

        Returns:
            The occupancy buffer as a list of float values.
        """
        return self._buffer

    def get_dimensions(self) -> tuple:
        """Map dimensions of the occupancy map.

        Returns:
            The map dimensions as a tuple (x, y, z).
        """
        return self._dimensions

    def set_transform(
        self,
        origin: tuple[float, float, float],
        min_point: tuple[float, float, float],
        max_point: tuple[float, float, float],
    ) -> None:
        """Set transform parameters for the occupancy map.

        Args:
            origin: Origin coordinates for the transform.
            min_point: Minimum point coordinates.
            max_point: Maximum point coordinates.
        """
        self._transform_set = True
        self._origin = origin
        self._min_point = min_point
        self._max_point = max_point

    def update(self) -> None:
        """Update the map (no-op for mock)."""

    def set_cell_size(self, cell_size: float) -> None:
        """Set cell size for the occupancy map.

        Args:
            cell_size: Size of each cell in the map.
        """
        self._cell_size = cell_size


class TestUtilsFunctions(omni.kit.test.AsyncTestCase):
    """Test suite for occupancy map utility functions."""

    def test_update_location(self) -> None:
        """Test update_location function."""
        mock_om = MockOccupancyMap()
        start_location = (1.0, 2.0, 3.0)
        lower_bound = (-1.0, -1.0, -1.0)
        upper_bound = (1.0, 1.0, 1.0)

        update_location(mock_om, start_location, lower_bound, upper_bound)

        # Verify transform was set
        self.assertTrue(mock_om._transform_set)
        self.assertEqual(mock_om._origin, start_location)
        self.assertEqual(mock_om._min_point, lower_bound)
        self.assertEqual(mock_om._max_point, upper_bound)

    def test_compute_coordinates(self) -> None:
        """Test compute_coordinates function."""
        mock_om = MockOccupancyMap(min_bound=(-5.0, -5.0, 0.0), max_bound=(5.0, 5.0, 2.0))
        cell_size = 0.05

        top_left, top_right, bottom_left, bottom_right, image_coords = compute_coordinates(mock_om, cell_size)

        # Verify corners are computed correctly
        self.assertIsInstance(top_left, tuple)
        self.assertIsInstance(top_right, tuple)
        self.assertIsInstance(bottom_left, tuple)
        self.assertIsInstance(bottom_right, tuple)
        self.assertEqual(len(top_left), 2)
        self.assertEqual(len(top_right), 2)

        # Verify geometric relationships
        self.assertAlmostEqual(top_left[0], mock_om._max_bound[0] - cell_size / 2.0, places=5)
        self.assertAlmostEqual(top_right[0], mock_om._min_bound[0] + cell_size / 2.0, places=5)

    def test_generate_image_all_unknown(self) -> None:
        """Test generate_image with all unknown cells."""
        buffer = [0.5] * 100  # 10x10 grid, all unknown
        mock_om = MockOccupancyMap(buffer=buffer, dimensions=(10, 10, 1))

        occupied_col = [0, 0, 0, 255]
        unknown_col = [127, 127, 127, 255]
        freespace_col = [255, 255, 255, 255]

        image = generate_image(mock_om, occupied_col, unknown_col, freespace_col)

        # Verify image is correct size (10x10 cells * 4 RGBA values)
        self.assertEqual(len(image), 10 * 10 * 4)

        # Verify all pixels are unknown color (gray)
        for i in range(0, len(image), 4):
            self.assertEqual(image[i : i + 4], unknown_col)

    def test_generate_image_mixed_occupancy(self) -> None:
        """Test generate_image with mixed occupancy states."""
        # Create buffer with known pattern: occupied, unknown, freespace
        buffer = [1.0] * 10 + [0.5] * 10 + [0.0] * 10  # 30 cells
        mock_om = MockOccupancyMap(buffer=buffer, dimensions=(30, 1, 1))

        occupied_col = [0, 0, 0, 255]
        unknown_col = [127, 127, 127, 255]
        freespace_col = [255, 255, 255, 255]

        image = generate_image(mock_om, occupied_col, unknown_col, freespace_col)

        # Verify image size
        self.assertEqual(len(image), 30 * 4)

        # Verify first 10 cells are occupied (black)
        for i in range(0, 10 * 4, 4):
            self.assertEqual(image[i : i + 4], occupied_col)

        # Verify next 10 cells are unknown (gray)
        for i in range(10 * 4, 20 * 4, 4):
            self.assertEqual(image[i : i + 4], unknown_col)

        # Verify last 10 cells are freespace (white)
        for i in range(20 * 4, 30 * 4, 4):
            self.assertEqual(image[i : i + 4], freespace_col)

    def test_generate_image_custom_colors(self) -> None:
        """Test generate_image with custom colors."""
        buffer = [1.0]  # Single occupied cell
        mock_om = MockOccupancyMap(buffer=buffer, dimensions=(1, 1, 1))

        # Custom colors
        occupied_col = [255, 0, 0, 255]  # Red
        unknown_col = [0, 255, 0, 255]  # Green
        freespace_col = [0, 0, 255, 255]  # Blue

        image = generate_image(mock_om, occupied_col, unknown_col, freespace_col)

        # Verify occupied cell is red
        self.assertEqual(image, occupied_col)

    def test_generate_image_empty_buffer(self) -> None:
        """Test generate_image with empty buffer."""
        buffer = []
        mock_om = MockOccupancyMap(buffer=buffer, dimensions=(0, 0, 1))

        occupied_col = [0, 0, 0, 255]
        unknown_col = [127, 127, 127, 255]
        freespace_col = [255, 255, 255, 255]

        image = generate_image(mock_om, occupied_col, unknown_col, freespace_col)

        # Empty buffer should produce empty image
        self.assertEqual(len(image), 0)

    def test_generate_image_performance(self) -> None:
        """Test generate_image performance with large map."""
        # Create a large map (1000x1000 = 1M cells)
        size = 1000
        buffer = [0.5] * (size * size)
        mock_om = MockOccupancyMap(buffer=buffer, dimensions=(size, size, 1))

        occupied_col = [0, 0, 0, 255]
        unknown_col = [127, 127, 127, 255]
        freespace_col = [255, 255, 255, 255]

        # This should complete quickly with optimized NumPy implementation
        image = generate_image(mock_om, occupied_col, unknown_col, freespace_col)

        # Verify correct size
        self.assertEqual(len(image), size * size * 4)

    def test_compute_coordinates_symmetry(self) -> None:
        """Test compute_coordinates with symmetric bounds."""
        mock_om = MockOccupancyMap(min_bound=(-10.0, -10.0, 0.0), max_bound=(10.0, 10.0, 2.0))
        cell_size = 0.1

        top_left, top_right, bottom_left, bottom_right, image_coords = compute_coordinates(mock_om, cell_size)

        # For symmetric bounds, verify geometric properties
        # Top left X should equal bottom left X
        self.assertAlmostEqual(top_left[0], bottom_left[0], places=5)
        # Top right X should equal bottom right X
        self.assertAlmostEqual(top_right[0], bottom_right[0], places=5)
        # Top left Y should equal top right Y
        self.assertAlmostEqual(top_left[1], top_right[1], places=5)
        # Bottom left Y should equal bottom right Y
        self.assertAlmostEqual(bottom_left[1], bottom_right[1], places=5)

    def test_update_location_with_zero_bounds(self) -> None:
        """Test update_location with zero-sized bounds."""
        mock_om = MockOccupancyMap()
        start_location = (0.0, 0.0, 0.0)
        lower_bound = (0.0, 0.0, 0.0)
        upper_bound = (0.0, 0.0, 0.0)

        # Should not raise an error
        update_location(mock_om, start_location, lower_bound, upper_bound)
        self.assertTrue(mock_om._transform_set)
