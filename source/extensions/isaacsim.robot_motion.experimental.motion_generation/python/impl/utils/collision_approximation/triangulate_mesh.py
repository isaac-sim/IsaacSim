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

"""Triangulates mesh faces into triangle index arrays for collision approximation in robot motion generation."""

from typing import List

import numpy as np
from isaacsim.core.experimental.objects import Mesh
from pxr import UsdGeom


def triangulate_mesh(mesh_input: Mesh) -> List[np.ndarray[int]]:
    """Triangulate mesh faces into triangle index arrays.

    Args:
        mesh_input: Mesh to triangulate.

    Returns:
        Triangle index arrays, one per mesh geometry.

    Raises:
        ValueError: Raised when the mesh has no geometry or face data.

    Example:

    .. code-block:: python

        >>> from isaacsim.robot_motion.experimental.motion_generation.utils.collision_approximation import triangulate_mesh
        >>> from isaacsim.core.experimental.objects import Mesh
        >>> mesh = Mesh("/World/SomeMesh")
        >>> _ = triangulate_mesh(mesh)  # doctest: +SKIP
    """
    if len(mesh_input.geoms) == 0:
        raise ValueError("triangulate_mesh was passed a mesh with no geoms.")

    triangle_lists = []
    for geom in mesh_input.geoms:
        mesh = UsdGeom.Mesh(geom)
        # indices and faces converted to triangles
        indices = mesh.GetFaceVertexIndicesAttr().Get()
        faces = mesh.GetFaceVertexCountsAttr().Get()

        if not indices or not faces:
            raise ValueError("triangulate_mesh was passed a mesh geom with no indices or faces.")
        triangles = []
        indices_offset = 0

        for face_count in faces:
            start_index = indices[indices_offset]
            for face_index in range(face_count - 2):
                index1 = indices_offset + face_index + 1
                index2 = indices_offset + face_index + 2
                triangles.append(start_index)
                triangles.append(indices[index1])
                triangles.append(indices[index2])
            indices_offset += face_count

        # reshape to be more natural to work with:
        triangles = np.reshape(np.array(triangles), shape=[-1, 3])
        triangle_lists.append(triangles)
    return triangle_lists
