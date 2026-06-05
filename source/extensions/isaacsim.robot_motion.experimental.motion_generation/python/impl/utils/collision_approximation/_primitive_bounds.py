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

"""Compute bounding volumes for primitive geometry.

This module is intended for primitive shapes only. For mesh bounds, ensure meshes
are cleaned and triangulated before computing bounds.
"""

import isaacsim.core.experimental.utils.prim as prim_utils
import isaacsim.core.experimental.utils.transform as transform_utils
import numpy as np
from pxr import Usd, UsdGeom

from .bounding_geometries import AABB, OBB


def compute_world_aabb_primitive(
    bbox_cache: UsdGeom.BBoxCache,
    prim_path: str,
) -> AABB:
    """Compute a world-space axis-aligned bounding box for a primitive.

    Args:
        bbox_cache: Bounding box cache for prim queries.
        prim_path: USD prim path to compute bounds for.

    Returns:
        Axis-aligned bounding box for the primitive in world coordinates.

    Example:

    .. code-block:: python

        >>> from pxr import Usd
        >>> from isaacsim.robot_motion.experimental.motion_generation.utils.collision_approximation import (
        ...     compute_world_aabb_primitive,
        ...     create_bbox_cache,
        ... )
        >>> bbox_cache = create_bbox_cache(Usd.TimeCode.Default())
        >>> prim_path = "/World/SomePrim"
        >>> _ = compute_world_aabb_primitive(bbox_cache=bbox_cache, prim_path=prim_path)  # doctest: +SKIP
    """
    prim = prim_utils.get_prim_at_path(prim_path)

    imageable = UsdGeom.Imageable(prim)
    time = Usd.TimeCode.Default()  # The time at which we compute the bounding box
    bound = imageable.ComputeWorldBound(time, UsdGeom.Tokens.default_)
    bounds_range = bound.ComputeAlignedBox()

    return AABB(min_bounds=np.array([*bounds_range.GetMin()]), max_bounds=np.array([*bounds_range.GetMax()]))


def compute_obb_primitive(
    bbox_cache: UsdGeom.BBoxCache,
    prim_path: str,
) -> OBB:
    """Compute an oriented bounding box for a primitive.

    Args:
        bbox_cache: Bounding box cache for prim queries.
        prim_path: USD prim path to compute bounds for.

    Returns:
        Oriented bounding box for the primitive.

    Example:

    .. code-block:: python

        >>> from pxr import Usd
        >>> from isaacsim.robot_motion.experimental.motion_generation.utils.collision_approximation import (
        ...     compute_obb_primitive,
        ...     create_bbox_cache,
        ... )
        >>> bbox_cache = create_bbox_cache(Usd.TimeCode.Default())
        >>> prim_path = "/World/SomePrim"
        >>> _ = compute_obb_primitive(bbox_cache=bbox_cache, prim_path=prim_path)  # doctest: +SKIP
    """
    prim = prim_utils.get_prim_at_path(prim_path)

    # TODO: what is the difference between ComputeLocalBound and ComputeWorldBound?
    # ComputeUntransformedBound should not include any transforms authored by the prim
    # itself, which should be the correct behaviour.
    bound = bbox_cache.ComputeUntransformedBound(prim)
    centroid = bound.ComputeCentroid()
    rotation_matrix = bound.GetMatrix().ExtractRotationMatrix()
    x_axis = rotation_matrix.GetRow(0)
    y_axis = rotation_matrix.GetRow(1)
    z_axis = rotation_matrix.GetRow(2)
    half_extent = bound.GetRange().GetSize() * 0.5

    # Convert rotation matrix to quaternion (w, x, y, z)
    axes_np = np.array([[*x_axis], [*y_axis], [*z_axis]])
    rotation_quaternion = transform_utils.rotation_matrix_to_quaternion(axes_np).numpy()

    # return the OBB
    center_np = np.array([*centroid])
    half_extent_np = np.array(half_extent)

    return OBB(rotation=rotation_quaternion, center=center_np, half_side_lengths=half_extent_np)
