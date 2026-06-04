# SPDX-FileCopyrightText: Copyright (c) 2022-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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

"""Mesh discovery and link-frame transform utilities for collision sphere generation."""

from __future__ import annotations

from collections import OrderedDict

import carb
import isaacsim.core.experimental.utils.prim as prim_utils
import isaacsim.core.experimental.utils.xform as xform_utils
import numpy as np
from pxr import Usd, UsdGeom


def find_link_meshes(
    stage: Usd.Stage,
    articulation_base_path: str,
    link_names: list[str],
) -> OrderedDict[str, list[str]]:
    """Identify and map all meshes within each link of the selected articulation.

    Walks the prim subtree rooted at ``articulation_base_path`` and groups every
    ``UsdGeom.Mesh`` it finds under the link that contains it (matched by the
    first path component appearing in ``link_names``).

    Instanceable meshes cannot be used for automatic sphere generation, but the
    enclosing link is still recorded (with an empty mesh list) so spheres can be
    authored manually under it.

    Args:
        stage: Active USD stage.
        articulation_base_path: Path to the articulation root.
        link_names: Link names belonging to this articulation.

    Returns:
        Ordered mapping from link subpath (relative to ``articulation_base_path``)
        to the list of mesh subpaths under that link (relative to the link).
    """
    link_to_meshes: OrderedDict[str, list[str]] = OrderedDict()
    link_names_set = set(link_names)

    num_art_path_components = len(articulation_base_path.split("/"))
    art_path_len = len(articulation_base_path)

    for prim in Usd.PrimRange(stage.GetPrimAtPath(articulation_base_path), Usd.TraverseInstanceProxies()):
        path = str(prim.GetPath())
        if not prim_utils.get_prim_at_path(path).IsA(UsdGeom.Xformable):
            continue

        geom_mesh = UsdGeom.Mesh(prim)
        if not geom_mesh.GetPointsAttr().HasValue():
            continue

        is_instanced = prim.IsInstanceProxy()

        # Find the length of the path of the link.
        link_subpath: str | None = None
        link_path_len = art_path_len
        for segment in path.split("/")[num_art_path_components:]:
            link_path_len += 1 + len(segment)
            if segment in link_names_set:
                link_subpath = path[art_path_len:link_path_len]
                break

        if link_subpath is None:
            carb.log_warn(
                f"The mesh at path {path} was not determined to be a part of "
                f"any link in the Articulation {articulation_base_path}"
            )
            continue

        if is_instanced:
            carb.log_warn(
                f"Found instanceable mesh at path {path}.  Instanceable meshes are not fully "
                "compatible with the Robot Description Editor.  They cannot be used to generate spheres "
                "automatically.  You may author spheres by hand or stop the timeline and "
                "uncheck 'Instanceable' in the Properties panel for this path and all parent "
                "paths."
            )
            if link_subpath not in link_to_meshes:
                link_to_meshes[link_subpath] = []
        else:
            mesh_subpath = path[link_path_len:]
            link_to_meshes.setdefault(link_subpath, []).append(mesh_subpath)

    return link_to_meshes


def compute_link_frame_mesh(link_path: str, mesh_path: str) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """Resolve a mesh's points into the local frame of its parent link.

    Uses :func:`isaacsim.core.experimental.utils.xform.get_relative_transform`
    to map the mesh's local points into the link's local frame. The relative
    transform incorporates each prim's full local-to-world matrix (including any
    accumulated scale on parents), which gives cuMotion's collision-sphere
    generator points in real-world dimensions.

    Args:
        link_path: USD path to the parent link.
        mesh_path: USD path to the mesh prim.

    Returns:
        Tuple ``(link_frame_points, face_inds, vert_cts)`` where
        ``link_frame_points`` has shape ``(N, 3)``.
    """
    geom_prim = prim_utils.get_prim_at_path(mesh_path)
    geom_mesh = UsdGeom.Mesh(geom_prim)

    mesh_points = np.array(geom_mesh.GetPointsAttr().Get())  # (N, 3)
    face_inds = np.array(geom_mesh.GetFaceVertexIndicesAttr().Get())
    vert_cts = np.array(geom_mesh.GetFaceVertexCountsAttr().Get())

    # Column-major 4x4 transform mapping mesh-local points to link-local frame.
    relative_transform = xform_utils.get_relative_transform(geom_prim, link_path)

    homogeneous_points = np.concatenate([mesh_points, np.ones((mesh_points.shape[0], 1))], axis=1)  # (N, 4)
    link_frame_points = (relative_transform @ homogeneous_points.T)[:3, :].T  # (N, 3)

    return link_frame_points, face_inds, vert_cts
