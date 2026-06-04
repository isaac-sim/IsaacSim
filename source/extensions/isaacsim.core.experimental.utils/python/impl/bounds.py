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

"""Functions for computing bounding boxes of USD prims."""

from __future__ import annotations

import numpy as np
from pxr import Gf, Usd, UsdGeom

from . import stage as stage_utils


def create_bbox_cache(
    time: Usd.TimeCode = Usd.TimeCode.Default(), *, use_extents_hint: bool = True
) -> UsdGeom.BBoxCache:
    """Create a ``UsdGeom.BBoxCache`` for bounding box computations.

    Backends: :guilabel:`usd`.

    Args:
        time: Time code at which the cache should be initialized.
        use_extents_hint: Use existing ``extents`` attribute on prims to speed up computation.

    Returns:
        Initialized bounding box cache.

    Examples:

    .. code-block:: python

        >>> import isaacsim.core.experimental.utils.bounds as bounds_utils
        >>>
        >>> cache = bounds_utils.create_bbox_cache()
    """
    return UsdGeom.BBoxCache(time=time, includedPurposes=[UsdGeom.Tokens.default_], useExtentsHint=use_extents_hint)


def compute_aabb(
    prim: str | Usd.Prim, *, bbox_cache: UsdGeom.BBoxCache | None = None, include_children: bool = False
) -> np.ndarray:
    """Compute an Axis-Aligned Bounding Box (AABB) for a prim.

    Backends: :guilabel:`usd`.

    Args:
        prim: Prim path or prim instance.
        bbox_cache: Bounding box cache to use. If ``None``, a new one is created.
        include_children: Whether to include children of the prim in the calculation.

    Returns:
        Bounding box as ``[min_x, min_y, min_z, max_x, max_y, max_z]``.

    Raises:
        ValueError: If the prim is not valid.

    Examples:

    .. code-block:: python

        >>> import isaacsim.core.experimental.utils.bounds as bounds_utils
        >>> from isaacsim.core.experimental.objects import Cube
        >>>
        >>> _ = Cube("/World/Cube", sizes=1.0)
        >>>
        >>> bounds_utils.compute_aabb("/World/Cube")
        [-0.5 -0.5 -0.5  0.5  0.5  0.5]
    """
    prim = stage_utils.get_current_stage(backend="usd").GetPrimAtPath(prim) if isinstance(prim, str) else prim
    if not prim or not prim.IsValid():
        raise ValueError(f"No valid prim: {prim}")
    if bbox_cache is None:
        bbox_cache = create_bbox_cache()

    total_bounds = Gf.BBox3d()
    if include_children:
        for p in Usd.PrimRange(prim):
            total_bounds = Gf.BBox3d.Combine(
                total_bounds, Gf.BBox3d(bbox_cache.ComputeWorldBound(p).ComputeAlignedRange())
            )
    else:
        total_bounds = Gf.BBox3d(bbox_cache.ComputeWorldBound(prim).ComputeAlignedRange())

    bbox_range = total_bounds.GetRange()
    return np.array([*bbox_range.GetMin(), *bbox_range.GetMax()])


def compute_combined_aabb(prims: list[str | Usd.Prim], *, bbox_cache: UsdGeom.BBoxCache | None = None) -> np.ndarray:
    """Compute a combined Axis-Aligned Bounding Box (AABB) for multiple prims.

    Backends: :guilabel:`usd`.

    Args:
        prims: List of prim paths or prim instances.
        bbox_cache: Bounding box cache to use. If ``None``, a new one is created.

    Returns:
        Combined bounding box as ``[min_x, min_y, min_z, max_x, max_y, max_z]``.

    Raises:
        ValueError: If the prims list is empty.

    Examples:

    .. code-block:: python

        >>> import isaacsim.core.experimental.utils.bounds as bounds_utils
        >>> from isaacsim.core.experimental.objects import Cube, Sphere
        >>>
        >>> _ = Cube("/World/Cube", sizes=1.0)
        >>> _ = Sphere("/World/Sphere", radii=1.0)
        >>>
        >>> bounds_utils.compute_combined_aabb(["/World/Cube", "/World/Sphere"])
        [-1. -1. -1.  1.  1.  1.]
    """
    if not prims:
        raise ValueError("prims list is empty.")
    if bbox_cache is None:
        bbox_cache = create_bbox_cache()

    stage = stage_utils.get_current_stage(backend="usd")
    total_bounds = Gf.BBox3d()
    for prim in prims:
        prim = stage.GetPrimAtPath(prim) if isinstance(prim, str) else prim
        bounds = bbox_cache.ComputeWorldBound(prim)
        total_bounds = Gf.BBox3d.Combine(total_bounds, Gf.BBox3d(bounds.ComputeAlignedRange()))
    bbox_range = total_bounds.GetRange()
    return np.array([*bbox_range.GetMin(), *bbox_range.GetMax()])


def compute_obb(
    prim: str | Usd.Prim, *, bbox_cache: UsdGeom.BBoxCache | None = None
) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """Compute the Oriented Bounding Box (OBB) of a prim.

    Backends: :guilabel:`usd`.

    .. note::

        * The OBB does not guarantee the smallest possible bounding box; it rotates and scales the default AABB.
        * The rotation matrix (axes) incorporates any scale factors applied to the object.
        * The ``half_extent`` values do not include these scaling effects.

    Args:
        prim: Prim path or prim instance.
        bbox_cache: Bounding box cache to use. If ``None``, a new one is created.

    Returns:
        A tuple of ``(centroid, axes, half_extent)`` where:

        - ``centroid``: Center of the OBB as a shape ``(3,)`` array.
        - ``axes``: Axes of the OBB as a shape ``(3, 3)`` array (one axis per row).
        - ``half_extent``: Half-lengths along each local axis as a shape ``(3,)`` array.

    Examples:

    .. code-block:: python

        >>> import isaacsim.core.experimental.utils.bounds as bounds_utils
        >>> from isaacsim.core.experimental.objects import Cube
        >>>
        >>> _ = Cube("/World/Cube", sizes=1.0)
        >>>
        >>> centroid, axes, half_extent = bounds_utils.compute_obb("/World/Cube")
        >>> centroid
        [0. 0. 0.]
        >>> axes
        [[1. 0. 0.]
         [0. 1. 0.]
         [0. 0. 1.]]
        >>> half_extent
        [0.5 0.5 0.5]
    """
    prim = stage_utils.get_current_stage(backend="usd").GetPrimAtPath(prim) if isinstance(prim, str) else prim
    if bbox_cache is None:
        bbox_cache = create_bbox_cache()

    bound = bbox_cache.ComputeWorldBound(prim)
    centroid = bound.ComputeCentroid()

    rotation_matrix = bound.GetMatrix().ExtractRotationMatrix()
    x_axis = rotation_matrix.GetRow(0)
    y_axis = rotation_matrix.GetRow(1)
    z_axis = rotation_matrix.GetRow(2)

    half_extent = bound.GetRange().GetSize() * 0.5

    return np.array([*centroid]), np.array([[*x_axis], [*y_axis], [*z_axis]]), np.array(half_extent)


def get_obb_corners(centroid: np.ndarray, axes: np.ndarray, half_extent: np.ndarray) -> np.ndarray:
    """Compute the 8 corners of an Oriented Bounding Box (OBB).

    Backends: :guilabel:`usd`.

    Args:
        centroid: Center of the OBB as a shape ``(3,)`` array.
        axes: Axes of the OBB as a shape ``(3, 3)`` array (one axis per row).
        half_extent: Half-lengths along each local axis as a shape ``(3,)`` array.

    Returns:
        Array of shape ``(8, 3)`` with each row being a corner position. Corners are ordered as:

        :math:`c_0 = (x_{min}, y_{min}, z_{min})`
        |br| :math:`c_1 = (x_{min}, y_{min}, z_{max})`
        |br| :math:`c_2 = (x_{min}, y_{max}, z_{min})`
        |br| :math:`c_3 = (x_{min}, y_{max}, z_{max})`
        |br| :math:`c_4 = (x_{max}, y_{min}, z_{min})`
        |br| :math:`c_5 = (x_{max}, y_{min}, z_{max})`
        |br| :math:`c_6 = (x_{max}, y_{max}, z_{min})`
        |br| :math:`c_7 = (x_{max}, y_{max}, z_{max})`

    Examples:

    .. code-block:: python

        >>> import isaacsim.core.experimental.utils.bounds as bounds_utils
        >>> from isaacsim.core.experimental.objects import Cube
        >>>
        >>> _ = Cube("/World/Cube", sizes=1.0)
        >>>
        >>> centroid, axes, half_extent = bounds_utils.compute_obb("/World/Cube")
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


def compute_obb_corners(prim: str | Usd.Prim, *, bbox_cache: UsdGeom.BBoxCache | None = None) -> np.ndarray:
    """Compute the 8 corners of the Oriented Bounding Box (OBB) of a prim.

    Backends: :guilabel:`usd`.

    Convenience function that combines :py:func:`compute_obb` and :py:func:`get_obb_corners`.

    Args:
        prim: Prim path or prim instance.
        bbox_cache: Bounding box cache to use. If ``None``, a new one is created.

    Returns:
        Array of shape ``(8, 3)`` with each row being a corner position (see :py:func:`get_obb_corners`).

    Examples:

    .. code-block:: python

        >>> import isaacsim.core.experimental.utils.bounds as bounds_utils
        >>> from isaacsim.core.experimental.objects import Cube
        >>>
        >>> _ = Cube("/World/Cube", sizes=1.0)
        >>>
        >>> bounds_utils.compute_obb_corners("/World/Cube")
        [[-0.5 -0.5 -0.5]
         [-0.5 -0.5  0.5]
         [-0.5  0.5 -0.5]
         [-0.5  0.5  0.5]
         [ 0.5 -0.5 -0.5]
         [ 0.5 -0.5  0.5]
         [ 0.5  0.5 -0.5]
         [ 0.5  0.5  0.5]]
    """
    centroid, axes, half_extent = compute_obb(prim, bbox_cache=bbox_cache)
    return get_obb_corners(centroid, axes, half_extent)
