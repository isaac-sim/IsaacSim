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

"""Utility functions for occupancy map generation and visualization."""

import numpy as np


def update_location(
    om: object,
    start_location: tuple[float, float, float],
    lower_bound: tuple[float, float, float],
    upper_bound: tuple[float, float, float],
) -> None:
    """Updates the occupancy map transform and visualization.

    Sets the transformation parameters for the occupancy map including origin and bounds,
    then triggers an update of the visualization.

    Args:
        om: The occupancy map interface object.
        start_location: The origin point in world coordinates as (x, y, z).
        lower_bound: The minimum bounds relative to origin as (x, y, z).
        upper_bound: The maximum bounds relative to origin as (x, y, z).

    Example:

    .. code-block:: python

        >>> update_location(om, (0, 0, 0), (-5, -5, 0), (5, 5, 2))
    """
    om.set_transform(
        (start_location[0], start_location[1], start_location[2]),
        (lower_bound[0], lower_bound[1], lower_bound[2]),
        (upper_bound[0], upper_bound[1], upper_bound[2]),
    )
    om.update()


def compute_coordinates(
    om: object, cell_size: float
) -> tuple[tuple[float, float], tuple[float, float], tuple[float, float], tuple[float, float], np.ndarray]:
    """Computes the corner coordinates and image transformation for the occupancy map.

    Calculates the world coordinates of the four corners of the occupancy map image
    (top-left, top-right, bottom-left, bottom-right) and the transformation matrix
    for converting between world coordinates and image pixel coordinates.

    Args:
        om: The occupancy map interface object.
        cell_size: Size of each cell in meters.

    Returns:
        A tuple containing top_left, top_right, bottom_left, bottom_right corner coordinates,
        and image_coords transformation matrix.

    Example:

    .. code-block:: python

        >>> top_left, top_right, bottom_left, bottom_right, image_coords = compute_coordinates(om, 0.05)
        >>> print(top_left)
        (4.975, -5.025)
    """
    min_b = om.get_min_bound()
    max_b = om.get_max_bound()
    scale = cell_size
    half_w = scale * 0.5
    top_left = (max_b[0] - half_w, min_b[1] + half_w)
    top_right = (min_b[0] + half_w, min_b[1] + half_w)
    bottom_left = (max_b[0] - half_w, max_b[1] - half_w)
    bottom_right = (min_b[0] + half_w, max_b[1] - half_w)

    image_coords = np.array([[0, 1], [-1, 0]]) @ np.array([[-top_left[0]], [-top_left[1]]])

    return top_left, top_right, bottom_left, bottom_right, image_coords


def generate_image(om: object, occupied_col: list[int], unknown_col: list[int], freespace_col: list[int]) -> list[int]:
    """Generates a colored RGBA image from the occupancy map buffer (optimized).

    Creates an image representation of the occupancy map where each cell is colored
    according to its occupancy state. Occupied cells, free space cells, and unknown
    cells are assigned different colors. Uses vectorized NumPy operations for
    improved performance over large maps.

    Args:
        om: The occupancy map interface object.
        occupied_col: RGBA color values (0-255) for occupied cells as [R, G, B, A].
        unknown_col: RGBA color values (0-255) for unknown cells as [R, G, B, A].
        freespace_col: RGBA color values (0-255) for free space cells as [R, G, B, A].

    Returns:
        A flat list of RGBA values representing the image, with 4 values per pixel.

    Example:

    .. code-block:: python

        >>> occupied = [0, 0, 0, 255]
        >>> unknown = [127, 127, 127, 255]
        >>> freespace = [255, 255, 255, 255]
        >>> image = generate_image(om, occupied, unknown, freespace)
    """
    buffer = np.array(om.get_buffer(), dtype=np.float32)
    dims = om.get_dimensions()
    total_cells = dims[0] * dims[1]

    # Initialize with unknown color using NumPy for better performance
    image = np.tile(unknown_col, total_cells).reshape(-1, 4)

    # Vectorized assignment for occupied and free space cells
    occupied_mask = buffer == 1.0
    freespace_mask = buffer == 0.0

    image[occupied_mask] = occupied_col
    image[freespace_mask] = freespace_col

    return image.flatten().tolist()
