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

"""Provides bounding geometry data structures for collision approximation in robot motion generation."""

from dataclasses import dataclass

import numpy as np


@dataclass
class AABB:
    """Axis-aligned bounding box representation.

    Args:
        min_bounds: Minimum bounds of the box in 3D space.
        max_bounds: Maximum bounds of the box in 3D space.

    Example:

    .. code-block:: python

        >>> import numpy as np
        >>> from isaacsim.robot_motion.experimental.motion_generation.utils.collision_approximation import AABB
        >>> bounds = AABB(
        ...     min_bounds=np.array([-1.0, -1.0, -1.0]),
        ...     max_bounds=np.array([1.0, 1.0, 1.0]),
        ... )
        >>> bounds.min_bounds.shape
        (3,)
    """

    min_bounds: np.ndarray
    max_bounds: np.ndarray


@dataclass
class OBB:
    """Oriented bounding box representation.

    Args:
        rotation: Rotation as quaternion (w, x, y, z) of the box orientation.
        half_side_lengths: Half-lengths along each box axis.
        center: Center position of the box in world space.

    Example:

    .. code-block:: python

        >>> import numpy as np
        >>> from isaacsim.robot_motion.experimental.motion_generation.utils.collision_approximation import OBB
        >>> obb = OBB(
        ...     rotation=np.array([1.0, 0.0, 0.0, 0.0]),
        ...     half_side_lengths=np.array([0.5, 1.0, 1.5]),
        ...     center=np.array([0.0, 0.0, 0.0]),
        ... )
        >>> obb.center.shape
        (3,)
    """

    rotation: np.ndarray
    half_side_lengths: np.ndarray
    center: np.ndarray


@dataclass
class ConvexHull:
    """Convex hull representation using vertices and triangle indices.

    Args:
        points: Vertex positions of the convex hull.
        triangles: Triangle indices forming the hull surface.

    Example:

    .. code-block:: python

        >>> import numpy as np
        >>> from isaacsim.robot_motion.experimental.motion_generation.utils.collision_approximation import ConvexHull
        >>> hull = ConvexHull(
        ...     points=np.array([[0.0, 0.0, 0.0], [1.0, 0.0, 0.0], [0.0, 1.0, 0.0]]),
        ...     triangles=np.array([[0, 1, 2]]),
        ... )
        >>> hull.points.shape
        (3, 3)
    """

    points: np.ndarray
    triangles: np.ndarray
