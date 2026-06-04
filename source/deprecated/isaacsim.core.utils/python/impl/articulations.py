# SPDX-FileCopyrightText: Copyright (c) 2024-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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

"""Deprecated articulation utility functions."""

from copy import copy

from isaacsim.core.utils.stage import get_current_stage
from pxr import PhysxSchema, Usd, UsdPhysics


def remove_articulation_root(prim: Usd.Prim) -> None:
    """Remove the Articulation Root from `prim` if one exists.

    Args:
        prim: A prim whose Articulation Root will be removed.
    """
    if prim.HasAPI(UsdPhysics.ArticulationRootAPI):
        prim.RemoveAPI(UsdPhysics.ArticulationRootAPI)
        if prim.HasAPI(PhysxSchema.PhysxArticulationAPI):
            prim.RemoveAPI(PhysxSchema.PhysxArticulationAPI)


def add_articulation_root(prim: Usd.Prim) -> None:
    """Add an Articulation Root to `prim`.

    Args:
        prim: A prim to which an Articulation Root will be added.
    """
    prim.ApplyAPI(UsdPhysics.ArticulationRootAPI)
    prim.ApplyAPI(PhysxSchema.PhysxArticulationAPI)


def move_articulation_root(src_prim: Usd.Prim, dst_prim: Usd.Prim) -> None:
    """Move the Articulation Root from `src_prim` to `dst_prim`. If `src_prim` is not an.

    Articulation Root, nothing will happen.

    Args:
        src_prim: A prim from which an Articulation Root will be removed.
        dst_prim: A prim to which an Articulation Root will be added.
    """
    if src_prim.HasAPI(UsdPhysics.ArticulationRootAPI):
        remove_articulation_root(src_prim)
        add_articulation_root(dst_prim)


def find_all_articulation_base_paths() -> list:
    """Find all base Articulation paths on the stage.

    A base path is defined as the maximal path that contains every part of a robot. For example,
    the articulation root in the UR10 robot may be at "/World/ur10/base_link", but the path returned
    by this function would be "/World/ur10".

    An Articulation base path:
      - Contains exactly one Articulation Root in the subtree of prim paths that stem from a base path.
      - Is a parent of every link in an Articulation.

    On a stage with nested articulation roots, only the inner-most root will be listed.

    Returns:
        A list of every Articulation base path on the stage.
    """
    articulation_root_paths = []
    articulation_candidates = set()

    stage = get_current_stage()

    if not stage:
        return articulation_root_paths

    # Find all articulation root paths
    # Find all paths that are the maximal subpath of all prims connected by a fixed joint
    # I.e. a fixed joint connecting /ur10/link1 to /ur10/link0 would result in the path
    # /ur10.  The path /ur10 becomes a candidate Articulation.
    for prim in Usd.PrimRange(stage.GetPrimAtPath("/")):
        if (
            prim.HasAPI(UsdPhysics.ArticulationRootAPI)
            and prim.GetProperty("physxArticulation:articulationEnabled").IsValid()
            and prim.GetProperty("physxArticulation:articulationEnabled").Get()
        ):
            articulation_root_paths.append(tuple(str(prim.GetPath()).split("/")[1:]))
        elif UsdPhysics.Joint(prim):
            bodies = prim.GetProperty("physics:body0").GetTargets()
            bodies.extend(prim.GetProperty("physics:body1").GetTargets())
            if len(bodies) == 1:
                continue
            base_path_split = str(bodies[0]).split("/")[1:]
            for body in bodies[1:]:
                body_path_split = str(body).split("/")[1:]
                common_len = min(len(base_path_split), len(body_path_split))
                for i in range(common_len):
                    if base_path_split[i] != body_path_split[i]:
                        base_path_split = base_path_split[:i]
                        break
                else:
                    base_path_split = base_path_split[:common_len]
            articulation_candidates.add(tuple(base_path_split))

    # Only keep candidates that have exactly one Articulation Root in their subtree.
    tmp = set()
    included_roots = set()
    for c in articulation_candidates:
        subtree_root_count = 0
        matched_root = None
        for root in articulation_root_paths:
            if len(root) >= len(c) and root[: len(c)] == c:
                subtree_root_count += 1
                matched_root = root
        if subtree_root_count == 1:
            tmp.add(c)
            included_roots.add(matched_root)
    articulation_candidates = tmp

    # Only keep candidates whose path is not a subset of another candidate's path
    unique_candidates = []
    for c1 in articulation_candidates:
        is_unique = True
        for c2 in articulation_candidates:
            if c1 == c2:
                continue
            elif c2[: len(c1)] == c1:
                is_unique = False
                break
        if is_unique:
            unique_candidates.append(c1)
    articulation_candidates = copy(unique_candidates)

    # Add any Articulation Root on the stage that is not in the subtree of any valid candidate
    # and does not contain any valid candidate in a subtree.
    for root in articulation_root_paths:
        if root in included_roots:
            continue
        add_to_candidates = True
        for c in unique_candidates:
            if len(root) <= len(c) and c[: len(root)] == root:
                add_to_candidates = False
                break
        if add_to_candidates:
            articulation_candidates.append(root)

    articulation_base_paths = []
    for c in articulation_candidates:
        articulation_path = ""
        for s in c:
            articulation_path += "/" + s
        articulation_base_paths.append(articulation_path)

    return articulation_base_paths
