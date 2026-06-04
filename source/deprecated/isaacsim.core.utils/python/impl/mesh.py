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

"""Deprecated mesh utility functions."""

import numpy as np

# isaacsim
from isaacsim.core.utils.stage import get_stage_units
from isaacsim.core.utils.transformations import get_relative_transform

# omniverse
from pxr import Usd, UsdGeom


def get_mesh_vertices_relative_to(mesh_prim: UsdGeom.Mesh, coord_prim: Usd.Prim) -> np.ndarray:
    """Get vertices of the mesh prim in the coordinate system of the given prim.

    Args:
        mesh_prim: Mesh prim to get the vertice points.
        coord_prim: Prim used as relative coordinate.

    Returns:
        Vertices of the mesh in the coordinate system of the given prim. Shape is (N, 3).

    Example:

    .. code-block:: python

        >>> import isaacsim.core.utils.mesh as mesh_utils
        >>> import isaacsim.core.utils.stage as stage_utils
        >>>
        >>> # 1 stage unit length cube centered at (0.0, 0.0, 0.0)
        >>> mesh_prim = stage_utils.get_current_stage().GetPrimAtPath("/World/Cube")
        >>> # 1 stage unit diameter sphere centered at (1.0, 1.0, 1.0)
        >>> coord_prim = stage_utils.get_current_stage().GetPrimAtPath("/World/Sphere")
        >>>
        >>> mesh_utils.get_mesh_vertices_relative_to(mesh_prim, coord_prim)
        [[-1.5 -1.5 -0.5]
         [-0.5 -1.5 -0.5]
         [-1.5 -0.5 -0.5]
         [-0.5 -0.5 -0.5]
         [-1.5 -1.5 -1.5]
         [-0.5 -1.5 -1.5]
         [-1.5 -0.5 -1.5]
         [-0.5 -0.5 -1.5]]
    """
    # Vertices of the mesh in the mesh's coordinate system
    vertices_vec3f = UsdGeom.Mesh(mesh_prim).GetPointsAttr().Get()
    if vertices_vec3f is None:
        return np.empty((0, 3), dtype=np.float64)
    vertices = np.array(vertices_vec3f)
    vertices_tf_row_major = np.pad(vertices, ((0, 0), (0, 1)), constant_values=1.0)

    # Transformation matrix from the coordinate system of the mesh to the coordinate system of the prim
    relative_tf_column_major = get_relative_transform(mesh_prim, coord_prim)
    relative_tf_row_major = np.transpose(relative_tf_column_major)

    # Transform points so they are in the coordinate system of the top-level ancestral xform prim
    points_in_relative_coord = vertices_tf_row_major @ relative_tf_row_major

    points_in_meters = points_in_relative_coord[:, :-1] * get_stage_units()

    return points_in_meters
