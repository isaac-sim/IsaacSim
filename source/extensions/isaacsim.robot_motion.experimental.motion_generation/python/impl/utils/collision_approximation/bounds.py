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

"""Provides utilities for computing bounding boxes and collision approximations for USD prims."""

from pxr import Usd, UsdGeom

from ._primitive_bounds import compute_obb_primitive, compute_world_aabb_primitive
from .bounding_geometries import AABB, OBB


def create_bbox_cache(
    time: Usd.TimeCode = Usd.TimeCode.Default(),
    use_extents_hint: bool = True,
) -> UsdGeom.BBoxCache:
    """Create a USD bounding box cache.

    Args:
        time: Timecode used for cache initialization.
        use_extents_hint: Whether to use authored extents to compute bounds.

    Returns:
        Initialized bounding box cache.

    Example:

    .. code-block:: python

        >>> from pxr import Usd, UsdGeom
        >>> from isaacsim.robot_motion.experimental.motion_generation.utils.collision_approximation import create_bbox_cache
        >>> cache = create_bbox_cache(time=Usd.TimeCode.Default())
        >>> isinstance(cache, UsdGeom.BBoxCache)
        True
    """
    return UsdGeom.BBoxCache(time=time, includedPurposes=[UsdGeom.Tokens.default_], useExtentsHint=use_extents_hint)


def compute_obb(
    bbox_cache: UsdGeom.BBoxCache,
    prim_path: str,
) -> OBB:
    """Compute an oriented bounding box for a prim path.

    Args:
        bbox_cache: Bounding box cache for prim queries.
        prim_path: USD prim path to compute bounds for.

    Returns:
        Oriented bounding box for the prim.

    Example:

    .. code-block:: python

        >>> from pxr import Usd
        >>> from isaacsim.robot_motion.experimental.motion_generation.utils.collision_approximation import (
        ...     compute_obb,
        ...     create_bbox_cache,
        ... )
        >>> bbox_cache = create_bbox_cache(Usd.TimeCode.Default())
        >>> prim_path = "/World/SomePrim"
        >>> _ = compute_obb(bbox_cache=bbox_cache, prim_path=prim_path)  # doctest: +SKIP
    """
    # TODO: ADD BACK IN ONCE MESH FILTERING WORKS.
    # # if this is a mesh, compute the mesh OBB:
    # if Mesh.are_of_type(prim_path).numpy().item():
    #     return compute_obb_mesh(Mesh(prim_path))

    # otherwise, compute based on primitive geometries:
    return compute_obb_primitive(bbox_cache=bbox_cache, prim_path=prim_path)


def compute_world_aabb(
    bbox_cache: UsdGeom.BBoxCache,
    prim_path: str,
) -> AABB:
    """Compute a world-space axis-aligned bounding box for a prim path.

    Args:
        bbox_cache: Bounding box cache for prim queries.
        prim_path: USD prim path to compute bounds for.

    Returns:
        Axis-aligned bounding box for the prim in world coordinates.

    Example:

    .. code-block:: python

        >>> from pxr import Usd
        >>> from isaacsim.robot_motion.experimental.motion_generation.utils.collision_approximation import (
        ...     compute_world_aabb,
        ...     create_bbox_cache,
        ... )
        >>> bbox_cache = create_bbox_cache(Usd.TimeCode.Default())
        >>> prim_path = "/World/SomePrim"
        >>> _ = compute_world_aabb(bbox_cache=bbox_cache, prim_path=prim_path)  # doctest: +SKIP
    """
    # TODO: ADD BACK IN ONCE MESH FILTERING WORKS.
    # # if this is a mesh, compute the mesh OBB:
    # if Mesh.are_of_type(prim_path).numpy().item():
    #     return compute_world_aabb_mesh(Mesh(prim_path))

    # otherwise, compute based on primitive geometries:
    return compute_world_aabb_primitive(bbox_cache=bbox_cache, prim_path=prim_path)
