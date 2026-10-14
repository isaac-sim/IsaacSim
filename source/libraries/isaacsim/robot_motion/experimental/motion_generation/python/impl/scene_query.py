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

"""Interface for searching the USD world for objects with a given TrackableApi."""

from __future__ import annotations

import isaacsim.core.experimental.utils.prim as prim_utils
import isaacsim.core.experimental.utils.stage as stage_utils
import numpy as np
import warp as wp
from isaacsim.core.experimental.objects import Plane
from isaacsim.core.experimental.utils.transform import quaternion_to_rotation_matrix
from pxr import Usd

from .trackable_api import TrackableApi
from .utils import collision_approximation as bound_utils


def _iter_stage_prims(stage: Usd.Stage) -> Usd.PrimRange:
    """Traverse a stage, including instance proxies but excluding prototype definitions.

    Args:
        stage: Stage used by the test.

    Returns:
        The resulting value.
    """
    return Usd.PrimRange.Stage(stage, Usd.TraverseInstanceProxies())


def _has_api(prim: Usd.Prim, api_name: str) -> bool:
    """Return whether a prim has a registered or authored API schema.

    Args:
        prim: USD prim to process.
        api_name: Api name values.

    Returns:
        The resulting value.
    """
    if prim_utils.has_api(prim, api_name):
        return True
    authored_api_schemas = prim.GetMetadata("apiSchemas")
    return bool(authored_api_schemas and api_name in authored_api_schemas.GetAddedOrExplicitItems())


def _has_trackable_api(prim: Usd.Prim, tracked_api: TrackableApi) -> bool:
    """Return whether a prim has the requested trackable API.

    Args:
        prim: USD prim to process.
        tracked_api: Tracked api values.

    Returns:
        The resulting value.
    """
    return _has_api(prim, str(tracked_api))


def _do_two_aabbs_overlap(
    first: bound_utils.AABB,
    second: bound_utils.AABB,
) -> bool:
    """Check whether two AABBs overlap.

    Args:
        first: First axis-aligned bounding box.
        second: Second axis-aligned bounding box.

    Returns:
        True if the AABBs overlap, False otherwise.
    """
    # check if boxes overlap in x:
    if first.min_bounds[0] > second.max_bounds[0]:
        return False
    if first.max_bounds[0] < second.min_bounds[0]:
        return False

    # check if boxes overlap in y:
    if first.min_bounds[1] > second.max_bounds[1]:
        return False
    if first.max_bounds[1] < second.min_bounds[1]:
        return False

    # check if boxes overlap in z:
    if first.min_bounds[2] > second.max_bounds[2]:
        return False
    if first.max_bounds[2] < second.min_bounds[2]:
        return False

    # Boxes overlap in all three axes.
    return True


def _does_plane_intersect_aabb(plane: Plane, aabb: bound_utils.AABB) -> bool:
    """Check whether a plane intersects an AABB.

    Args:
        plane: Plane to test for intersection.
        aabb: Axis-aligned bounding box to test against.

    Raises:
        ValueError: If the plane axis is not "X", "Y", or "Z".

    Returns:
        True if the plane intersects the AABB, False otherwise.
    """
    # First, get all 8 points of the AABB:
    aabb_points_world = np.array(
        [
            [aabb.min_bounds[0], aabb.min_bounds[1], aabb.min_bounds[2]],
            [aabb.min_bounds[0], aabb.min_bounds[1], aabb.max_bounds[2]],
            [aabb.min_bounds[0], aabb.max_bounds[1], aabb.min_bounds[2]],
            [aabb.min_bounds[0], aabb.max_bounds[1], aabb.max_bounds[2]],
            [aabb.max_bounds[0], aabb.min_bounds[1], aabb.min_bounds[2]],
            [aabb.max_bounds[0], aabb.min_bounds[1], aabb.max_bounds[2]],
            [aabb.max_bounds[0], aabb.max_bounds[1], aabb.min_bounds[2]],
            [aabb.max_bounds[0], aabb.max_bounds[1], aabb.max_bounds[2]],
        ]
    )

    plane_axis = plane.get_axes(indices=[0])[0]
    plane_center, plane_rotation = plane.get_world_poses(indices=[0])
    plane_center = plane_center.numpy()[0]
    R_world_plane = quaternion_to_rotation_matrix(quaternion=plane_rotation).numpy()[0]

    # put the AABB points into the frame of the plane:
    aabb_points_plane_frame = (R_world_plane.T @ (aabb_points_world - plane_center).T).T

    # Check if points appear on both positive and negative sides of the plane. If so,
    # the plane intersects the aabb. Else it does not.
    if plane_axis == "X":
        search_index = 0
    elif plane_axis == "Y":
        search_index = 1
    elif plane_axis == "Z":
        search_index = 2
    else:
        raise ValueError(f"Invalid plane axis: {plane_axis}")

    negative_found = False
    positive_found = False

    for point in aabb_points_plane_frame:
        if point[search_index] >= 0:
            positive_found = True
        if point[search_index] <= 0:
            negative_found = True
    if positive_found and negative_found:
        # The plane intersects the AABB.
        return True

    # TODO: consider width and length of planes.
    return False


class SceneQuery:
    """Interface for searching the USD world for objects with a given TrackableApi."""

    def __init__(self) -> None:
        self._bb_cache = bound_utils.create_bbox_cache()
        self._stage = stage_utils.get_current_stage(backend="usd")

    def get_prims_in_aabb(
        self,
        search_box_origin: wp.array | list[float] | np.ndarray,
        search_box_minimum: wp.array | list[float] | np.ndarray,
        search_box_maximum: wp.array | list[float] | np.ndarray,
        tracked_api: TrackableApi,
        include_prim_paths: list[str] | str | None = None,
        exclude_prim_paths: list[str] | str | None = None,
    ) -> list[str]:
        """Return prim paths that intersect an AABB and match an applied API filter.

        Args:
            search_box_origin: Origin for the AABB bounds.
            search_box_minimum: Minimum for the AABB bounds, relative to the search_box_origin.
            search_box_maximum: Maximum bounds in the box frame, relative to the search_box_origin.
            tracked_api: API schema filter to apply when searching.
            include_prim_paths: Optional list of prim paths to include (including their children).
                If this is None, then all prims are included.
            exclude_prim_paths: Optional list of prim paths to exclude (including their children).
                If this is None, then no prims are excluded.

        Returns:
            List of prim paths that intersect the AABB and match the API filter.

        Raises:
            ValueError: If the tracked_api is unsupported.
            ValueError: If any search bounds are not size 3.
            ValueError: If minimum bounds are not strictly less than maximum bounds.

        Example:

        .. code-block:: python

            >>> from isaacsim.robot_motion.experimental.motion_generation import SceneQuery, TrackableApi
            >>>
            >>> query = SceneQuery()
            >>> hits = query.get_prims_in_aabb(
            ...     search_box_origin=[0.0, 0.0, 0.0],
            ...     search_box_minimum=[-1.0, -1.0, -1.0],
            ...     search_box_maximum=[1.0, 1.0, 1.0],
            ...     tracked_api=TrackableApi.PHYSICS_COLLISION,
            ... )
        """
        # Refresh the cache once per query so authored transform changes are reflected,
        # while still sharing one cache across all bound computations in this query.
        self._bb_cache = bound_utils.create_bbox_cache()

        # Convert single include path to list if it exists:
        if include_prim_paths is not None:
            if isinstance(include_prim_paths, str):
                include_prim_paths = [include_prim_paths]

        # Convert single exclude path to list if it exists:
        if exclude_prim_paths is not None:
            if isinstance(exclude_prim_paths, str):
                exclude_prim_paths = [exclude_prim_paths]

        if tracked_api not in {
            TrackableApi.PHYSICS_COLLISION,
            TrackableApi.PHYSICS_RIGID_BODY,
            TrackableApi.MOTION_GENERATION_COLLISION,
        }:
            raise ValueError(f"{str(tracked_api)} is not in the list of supported TrackableApi.")

        if isinstance(search_box_origin, wp.array):
            search_box_origin = search_box_origin.numpy()

        if isinstance(search_box_maximum, wp.array):
            search_box_maximum = search_box_maximum.numpy()

        if isinstance(search_box_minimum, wp.array):
            search_box_minimum = search_box_minimum.numpy()

        # if list of floats, or higher dimensional numpy array, flatten to a 1D array.
        search_box_origin = np.array(search_box_origin).flatten()
        search_box_maximum = np.array(search_box_maximum).flatten()
        search_box_minimum = np.array(search_box_minimum).flatten()

        if not (search_box_origin.size == search_box_maximum.size == search_box_minimum.size == 3):
            raise ValueError("Search box origin, maximum and minimum must all be of size 3.")

        if (search_box_minimum >= search_box_maximum).any():
            raise ValueError("All minimum search bounds must be strictly less than all maximum search bounds.")

        # search box, shifted according to the origin of the box:
        search_box_maximum = search_box_maximum + search_box_origin
        search_box_minimum = search_box_minimum + search_box_origin

        search_box = bound_utils.AABB(
            min_bounds=search_box_minimum,
            max_bounds=search_box_maximum,
        )

        paths_to_exclude = tuple(exclude_prim_paths or [])
        paths_to_include = tuple(include_prim_paths or [])

        def _is_at_or_below(path: str, ancestor: str) -> bool:
            return path == ancestor or path.startswith(f"{ancestor.rstrip('/')}/")

        def _search_predicate(prim: Usd.Prim) -> bool:
            if prim.IsInPrototype():
                return False

            prim_path = prim.GetPath().pathString
            if paths_to_include and not any(_is_at_or_below(prim_path, path) for path in paths_to_include):
                return False
            if any(_is_at_or_below(prim_path, path) for path in paths_to_exclude):
                return False

            if prim.GetTypeName() == "Plane":
                return _does_plane_intersect_aabb(Plane(prim_path), search_box)

            body_aabb = bound_utils.compute_world_aabb(self._bb_cache, prim_path)
            return _do_two_aabbs_overlap(body_aabb, search_box)

        return [
            prim.GetPath().pathString
            for prim in _iter_stage_prims(self._stage)
            if _has_trackable_api(prim, tracked_api) and _search_predicate(prim)
        ]

    def get_robots_in_stage(self) -> list[str]:
        """Return robot prim paths in the current stage.

        Returns:
            List of robot prim paths on the current stage.

        Example:

        .. code-block:: python

            >>> from isaacsim.robot_motion.experimental.motion_generation import SceneQuery
            >>>
            >>> query = SceneQuery()
            >>> robots = query.get_robots_in_stage()
        """
        return [
            prim.GetPath().pathString
            for prim in _iter_stage_prims(self._stage)
            if not prim.IsInPrototype() and _has_api(prim, "IsaacRobotAPI")
        ]
