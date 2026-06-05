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

"""Deprecated bounds computation utilities."""

# omniverse
import carb
import numpy as np

# isaacsim
from isaacsim.core.utils.prims import get_prim_at_path
from pxr import Gf, Usd, UsdGeom


def recompute_extents(
    prim: UsdGeom.Boundable, time: Usd.TimeCode = Usd.TimeCode.Default(), include_children: bool = False
) -> None:
    """Recomputes and overwrites the extents attribute for a UsdGeom.Boundable prim.

    Args:
        prim: Input prim to recompute extents for
        time: timecode to use for computing extents
        include_children: include children of specified prim in calculation

    Raises:
        ValueError: If prim is not of UsdGeom.Boundable type

    Example:

    .. code-block:: python

        >>> import isaacsim.core.utils.bounds as bounds_utils
        >>> import isaacsim.core.utils.stage as stage_utils
        >>>
        >>> prim = stage_utils.get_current_stage().GetPrimAtPath("/World/Cube")
        >>> bounds_utils.recompute_extents(prim)
    """

    #
    def update_extents(prim: UsdGeom.Boundable, time: Usd.TimeCode = Usd.TimeCode.Default()) -> None:
        compute_prim = UsdGeom.Boundable(prim)
        if compute_prim:
            bounds = []
            mesh = UsdGeom.Mesh(compute_prim)
            if mesh:
                points = mesh.GetPointsAttr().Get()
                if points is None:
                    return
                bounds = mesh.ComputeExtent(points)
            else:
                bounds = UsdGeom.Boundable.ComputeExtentFromPlugins(compute_prim, time)

            if compute_prim.GetExtentAttr().HasValue():
                compute_prim.GetExtentAttr().Set(bounds)
            else:
                compute_prim.CreateExtentAttr(bounds)
        else:
            raise ValueError(f"Input prim is not of type UsdGeom.Boundable, is instead {type(prim)}")

    if include_children:
        for p in Usd.PrimRange(prim.GetPrim()):
            try:
                update_extents(p, time)
            except ValueError:
                carb.log_warn(f"Skipping {p}, not boundable")
    else:
        update_extents(prim, time)


def create_bbox_cache(time: Usd.TimeCode = Usd.TimeCode.Default(), use_extents_hint: bool = True) -> UsdGeom.BBoxCache:
    """Helper function to create a Bounding Box Cache object that can be used for computations.

    Args:
        time: time at which cache should be initialized
        use_extents_hint: Use existing extents attribute on prim to compute bounding box

    Returns:
        Initialized bbox cache

    Example:

    .. code-block:: python

        >>> import isaacsim.core.utils.bounds as bounds_utils
        >>>
        >>> bounds_utils.create_bbox_cache()
        <pxr.UsdGeom.BBoxCache object at 0x7f6720b8bc90>
    """
    return UsdGeom.BBoxCache(time=time, includedPurposes=[UsdGeom.Tokens.default_], useExtentsHint=use_extents_hint)


def compute_aabb(bbox_cache: UsdGeom.BBoxCache, prim_path: str, include_children: bool = False) -> np.array:
    """Compute an Axis-Aligned Bounding Box (AABB) for a given ``prim_path``.

    A combined AABB is computed if ``include_children`` is True

    Args:
        bbox_cache: Existing Bounding box cache to use for computation
        prim_path: prim path to compute AABB for
        include_children: include children of specified prim in calculation

    Returns:
        Bounding box for this prim, [min x, min y, min z, max x, max y, max z]

    Example:

    .. code-block:: python

        >>> import isaacsim.core.utils.bounds as bounds_utils
        >>>
        >>> # 1 stage unit length cube centered at (0.0, 0.0, 0.0)
        >>> cache = bounds_utils.create_bbox_cache()
        >>> bounds_utils.compute_aabb(cache, prim_path="/World/Cube")
        [-0.5 -0.5 -0.5  0.5  0.5  0.5]
        >>>
        >>> # the same cube rotated 45 degrees around the z-axis
        >>> cache = bounds_utils.create_bbox_cache()
        >>> bounds_utils.compute_aabb(cache, prim_path="/World/Cube")
        [-0.70710678  -0.70710678  -0.5  0.70710678  0.70710678  0.5]
    """
    total_bounds = Gf.BBox3d()
    prim = get_prim_at_path(prim_path)
    if not prim or not prim.IsValid():
        raise ValueError(f"No valid prim at path: {prim_path}")
    if include_children:
        for p in Usd.PrimRange(prim):
            total_bounds = Gf.BBox3d.Combine(
                total_bounds, Gf.BBox3d(bbox_cache.ComputeWorldBound(p).ComputeAlignedRange())
            )
    else:
        total_bounds = Gf.BBox3d(bbox_cache.ComputeWorldBound(prim).ComputeAlignedRange())

    bbox_range = total_bounds.GetRange()
    return np.array([*bbox_range.GetMin(), *bbox_range.GetMax()])


def compute_combined_aabb(bbox_cache: UsdGeom.BBoxCache, prim_paths: list[str]) -> np.array:
    """Computes a combined Axis-Aligned Bounding Box (AABB) given a list of prim paths.

    Args:
        bbox_cache: Existing Bounding box cache to use for computation
        prim_paths: List of prim paths to compute combined AABB for

    Returns:
        Bounding box for input prims, [min x, min y, min z, max x, max y, max z]

    Example:

    .. code-block:: python

        >>> import isaacsim.core.utils.bounds as bounds_utils
        >>>
        >>> # 1 stage unit length cube centered at (0.0, 0.0, 0.0)
        >>> # with a 1 stage unit diameter sphere centered at (-0.5, 0.5, 0.5)
        >>> cache = bounds_utils.create_bbox_cache()
        >>> bounds_utils.compute_combined_aabb(cache, prim_paths=["/World/Cube", "/World/Sphere"])
        [-1.  -0.5 -0.5  0.5  1.   1. ]
    """
    if not prim_paths:
        raise ValueError("prim_paths list is empty.")
    total_bounds = Gf.BBox3d()
    for prim_path in prim_paths:
        prim = get_prim_at_path(prim_path)
        bounds = bbox_cache.ComputeWorldBound(prim)
        total_bounds = Gf.BBox3d.Combine(total_bounds, Gf.BBox3d(bounds.ComputeAlignedRange()))
    bbox_range = total_bounds.GetRange()
    return np.array([*bbox_range.GetMin(), *bbox_range.GetMax()])


def compute_obb(bbox_cache: UsdGeom.BBoxCache, prim_path: str) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """Computes the Oriented Bounding Box (OBB) of a prim.

    .. note::

        * The OBB does not guarantee the smallest possible bounding box, it rotates and scales the default AABB.
        * The rotation matrix incorporates any scale factors applied to the object.
        * The `half_extent` values do not include these scaling effects.

    Args:
        bbox_cache: USD Bounding Box Cache object to use for computation
        prim_path: Prim path to compute OBB for

    Returns:
        A tuple containing the following OBB information:
            - The centroid of the OBB as a NumPy array.
            - The axes of the OBB as a 2D NumPy array, where each row represents a different axis.
            - The half extent of the OBB as a NumPy array.

    Example:

    .. code-block:: python

        >>> import isaacsim.core.utils.bounds as bounds_utils
        >>>
        >>> # 1 stage unit length cube centered at (0.0, 0.0, 0.0)
        >>> cache = bounds_utils.create_bbox_cache()
        >>> centroid, axes, half_extent = bounds_utils.compute_obb(cache, prim_path="/World/Cube")
        >>> centroid
        [0. 0. 0.]
        >>> axes
        [[1. 0. 0.]
         [0. 1. 0.]
         [0. 0. 1.]]
        >>> half_extent
        [0.5 0.5 0.5]
        >>>
        >>> # the same cube rotated 45 degrees around the z-axis
        >>> cache = bounds_utils.create_bbox_cache()
        >>> centroid, axes, half_extent = bounds_utils.compute_obb(cache, prim_path="/World/Cube")
        >>> centroid
        [0. 0. 0.]
        >>> axes
        [[ 0.70710678  0.70710678  0.        ]
         [-0.70710678  0.70710678  0.        ]
         [ 0.          0.          1.        ]]
        >>> half_extent
        [0.5 0.5 0.5]
    """
    # Compute the BBox3d for the prim
    prim = get_prim_at_path(prim_path)
    bound = bbox_cache.ComputeWorldBound(prim)

    # Compute the translated centroid of the world bound
    centroid = bound.ComputeCentroid()

    # Compute the axis vectors of the OBB
    # NOTE: The rotation matrix incorporates the scale factors applied to the object
    rotation_matrix = bound.GetMatrix().ExtractRotationMatrix()
    x_axis = rotation_matrix.GetRow(0)
    y_axis = rotation_matrix.GetRow(1)
    z_axis = rotation_matrix.GetRow(2)

    # Compute the half-lengths of the OBB along each axis
    # NOTE the size/extent values do not include any scaling effects
    half_extent = bound.GetRange().GetSize() * 0.5

    return np.array([*centroid]), np.array([[*x_axis], [*y_axis], [*z_axis]]), np.array(half_extent)


def get_obb_corners(centroid: np.ndarray, axes: np.ndarray, half_extent: np.ndarray) -> np.ndarray:
    """Computes the corners of the Oriented Bounding Box (OBB) from the given OBB information.

    Args:
        centroid: The centroid of the OBB as a NumPy array.
        axes: The axes of the OBB as a 2D NumPy array, where each row represents a different axis.
        half_extent: The half extent of the OBB as a NumPy array.

    Returns:
        NumPy array of shape (8, 3) containing each corner location of the OBB

        :math:`c_0 = (x_{min}, y_{min}, z_{min})`
        |br| :math:`c_1 = (x_{min}, y_{min}, z_{max})`
        |br| :math:`c_2 = (x_{min}, y_{max}, z_{min})`
        |br| :math:`c_3 = (x_{min}, y_{max}, z_{max})`
        |br| :math:`c_4 = (x_{max}, y_{min}, z_{min})`
        |br| :math:`c_5 = (x_{max}, y_{min}, z_{max})`
        |br| :math:`c_6 = (x_{max}, y_{max}, z_{min})`
        |br| :math:`c_7 = (x_{max}, y_{max}, z_{max})`

    Example:

    .. code-block:: python

        >>> import isaacsim.core.utils.bounds as bounds_utils
        >>>
        >>> cache = bounds_utils.create_bbox_cache()
        >>> centroid, axes, half_extent = bounds_utils.compute_obb(cache, prim_path="/World/Cube")
        >>> bounds_utils.get_obb_corners(centroid, axes, half_extent)
        [[-0.5 -0.5 -0.5]
         [-0.5 -0.5  0.5]
         [-0.5  0.5 -0.5]
         [-0.5  0.5  0.5]
         [ 0.5 -0.5 -0.5]
         [ 0.5 -0.5  0.5]
         [ 0.5  0.5 -0.5]
         [ 0.5  0.5  0.5]]
    """
    corners = [
        centroid - axes[0] * half_extent[0] - axes[1] * half_extent[1] - axes[2] * half_extent[2],
        centroid - axes[0] * half_extent[0] - axes[1] * half_extent[1] + axes[2] * half_extent[2],
        centroid - axes[0] * half_extent[0] + axes[1] * half_extent[1] - axes[2] * half_extent[2],
        centroid - axes[0] * half_extent[0] + axes[1] * half_extent[1] + axes[2] * half_extent[2],
        centroid + axes[0] * half_extent[0] - axes[1] * half_extent[1] - axes[2] * half_extent[2],
        centroid + axes[0] * half_extent[0] - axes[1] * half_extent[1] + axes[2] * half_extent[2],
        centroid + axes[0] * half_extent[0] + axes[1] * half_extent[1] - axes[2] * half_extent[2],
        centroid + axes[0] * half_extent[0] + axes[1] * half_extent[1] + axes[2] * half_extent[2],
    ]
    return np.array(corners)


def compute_obb_corners(bbox_cache: UsdGeom.BBoxCache, prim_path: str) -> np.ndarray:
    """Computes the corners of the Oriented Bounding Box (OBB) of a prim.

    Args:
        bbox_cache: Bounding Box Cache object to use for computation
        prim_path: Prim path to compute OBB for

    Returns:
        NumPy array of shape (8, 3) containing each corner location of the OBB

        :math:`c_0 = (x_{min}, y_{min}, z_{min})`
        |br| :math:`c_1 = (x_{min}, y_{min}, z_{max})`
        |br| :math:`c_2 = (x_{min}, y_{max}, z_{min})`
        |br| :math:`c_3 = (x_{min}, y_{max}, z_{max})`
        |br| :math:`c_4 = (x_{max}, y_{min}, z_{min})`
        |br| :math:`c_5 = (x_{max}, y_{min}, z_{max})`
        |br| :math:`c_6 = (x_{max}, y_{max}, z_{min})`
        |br| :math:`c_7 = (x_{max}, y_{max}, z_{max})`

    Example:

    .. code-block:: python

        >>> import isaacsim.core.utils.bounds as bounds_utils
        >>>
        >>> cache = bounds_utils.create_bbox_cache()
        >>> bounds_utils.compute_obb_corners(cache, prim_path="/World/Cube")
        [[-0.5 -0.5 -0.5]
         [-0.5 -0.5  0.5]
         [-0.5  0.5 -0.5]
         [-0.5  0.5  0.5]
         [ 0.5 -0.5 -0.5]
         [ 0.5 -0.5  0.5]
         [ 0.5  0.5 -0.5]
         [ 0.5  0.5  0.5]]
    """
    centroid, axis, half_extent = compute_obb(bbox_cache, prim_path)
    return get_obb_corners(centroid, axis, half_extent)
