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

"""Pure stage-graph utilities for articulation discovery and self-collision inference.

This module is deliberately UI-free and depends only on USD/PhysX schema reads,
so it can be exercised in unit tests without an ``omni.ui`` / Kit context.
"""

from __future__ import annotations

import copy

import isaacsim.core.experimental.utils.prim as prim_utils
from pxr import PhysxSchema, Usd, UsdPhysics


def find_all_articulation_base_paths(stage: Usd.Stage | None) -> list[str]:
    """Find every articulation base path on a USD stage.

    A "base path" is the maximal subtree path that contains an entire
    articulation. For example, the articulation root for the UR10 robot may
    live at ``/World/ur10/base_link``, but the base path returned here is
    ``/World/ur10`` — the common ancestor of every link reachable through the
    articulation's joints.

    A returned base path satisfies both conditions:

    - Its subtree contains exactly one prim with ``UsdPhysics.ArticulationRootAPI``
      applied (and the PhysX ``articulationEnabled`` flag set).
    - It is an ancestor of every link referenced by that articulation's joints.

    When articulation roots are nested, only the inner-most root is reported.

    .. note::

        The "enabled" filter reads ``physxArticulation:articulationEnabled``,
        which is PhysX-specific. Articulations authored against
        ``UsdPhysics.ArticulationRootAPI`` but driven by a non-PhysX backend
        are skipped.

    Args:
        stage: USD stage to scan. Pass ``None`` to receive an empty list.

    Returns:
        Articulation base paths on the stage, each formatted as a leading-slash
        prim path string (for example ``"/World/ur10"``).
    """
    articulation_root_paths: list[tuple[str, ...]] = []
    articulation_candidates: set[tuple[str, ...]] = set()

    if not stage:
        return []

    # Single stage traversal collects two things in parallel: every enabled
    # articulation root, and the common-ancestor path of every joint's body
    # targets (a candidate base path for whichever articulation owns it).
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
            # A joint with fewer than two resolved targets cannot define a
            # connected component; skip it (also avoids an `IndexError` when
            # the loop below dereferences `bodies[0]`).
            if len(bodies) < 2:
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

    # Discard joint candidates whose subtree owns zero or multiple articulation
    # roots — the former are not articulations at all and the latter are
    # ambiguous (we want the inner-most root, handled below).
    refined_candidates: set[tuple[str, ...]] = set()
    included_roots: set[tuple[str, ...]] = set()
    for candidate in articulation_candidates:
        subtree_root_count = 0
        matched_root: tuple[str, ...] | None = None
        for root in articulation_root_paths:
            if len(root) >= len(candidate) and root[: len(candidate)] == candidate:
                subtree_root_count += 1
                matched_root = root
        if subtree_root_count == 1 and matched_root is not None:
            refined_candidates.add(candidate)
            included_roots.add(matched_root)
    articulation_candidates = refined_candidates

    # Collapse nested candidates: if one candidate's path is a prefix of
    # another's, the deeper candidate is the one we actually want (inner-most
    # robot subtree), so drop the shallower duplicate.
    unique_candidates: list[tuple[str, ...]] = []
    for c1 in articulation_candidates:
        is_unique = True
        for c2 in articulation_candidates:
            if c1 == c2:
                continue
            if c2[: len(c1)] == c1:
                is_unique = False
                break
        if is_unique:
            unique_candidates.append(c1)
    articulation_candidates = copy.copy(unique_candidates)

    # Backfill articulation roots that were never matched by the joint walk
    # (e.g. articulations with no `UsdPhysics.Joint` prims, or joints that
    # failed the two-target check above), as long as they do not live inside
    # one of the candidates we already picked.
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

    # Convert the (segment, segment, ...) tuples back into "/seg/seg/..." SDF
    # paths for callers, which expect strings.
    base_paths: list[str] = []
    for candidate in articulation_candidates:
        path = ""
        for segment in candidate:
            path += "/" + segment
        base_paths.append(path)
    return base_paths


def find_mimic_joint_names(stage: Usd.Stage | None, articulation_base_path: str | None) -> set[str]:
    """Return the names of every mimic-follower joint under ``articulation_base_path``.

    A "mimic-follower" joint is a ``UsdPhysics.Joint`` prim whose position is
    derived from a reference joint via a gear ratio. Two schemas can express
    this relationship and the function detects both:

    * ``PhysxSchema.PhysxMimicJointAPI`` (multi-apply): the legacy PhysX-side
      schema still present on assets imported before Isaac Sim 5.0.
    * ``NewtonMimicAPI`` (single-apply): the schema authored by the current
      URDF importer (Isaac Sim 5.0+), consumed directly by the Newton runtime.
      Detected by string name because the Python binding may not be loaded in
      all standalone tooling configurations.

    The PhysX articulation treats mimic followers as auxiliary c-space
    coordinates rather than independently actuated DOFs. cuMotion's
    ``load_robot_from_memory`` raises if the XRDF's ``default_joint_positions``
    lists a mimic follower, so the XRDF/Lula exporters use this helper to
    filter them out.

    Args:
        stage: USD stage to scan. Pass ``None`` to receive an empty set.
        articulation_base_path: Stage path of the articulation subtree to scan.
            Pass ``None`` or an empty string to receive an empty set.

    Returns:
        Set of mimic-follower joint prim names (matching ``Articulation.dof_names``).
    """
    if stage is None or not articulation_base_path:
        return set()

    base_prim = stage.GetPrimAtPath(articulation_base_path)
    if not base_prim or not base_prim.IsValid():
        return set()

    mimic_names: set[str] = set()
    for prim in Usd.PrimRange(base_prim):
        if not UsdPhysics.Joint(prim):
            continue
        if prim.HasAPI(PhysxSchema.PhysxMimicJointAPI) or prim.HasAPI("NewtonMimicAPI"):
            mimic_names.add(prim.GetName())
    return mimic_names


def get_ignore_dict(articulation_base_path: str, ordered_links: list[str]) -> dict[str, list[str]]:
    """Build a self-collision ignore mapping from joint connectivity.

    Two links connected directly by a joint are added to each other's ignore
    lists. Additionally, if link A connects to links B, C, and D, then B, C,
    and D are also made to ignore each other — they share a common parent and
    pairwise self-collision checks among them are wasted work in practice.

    Joints whose ``physics:body0`` or ``physics:body1`` does not resolve to
    exactly one target are skipped, as are joints whose endpoints fall outside
    ``ordered_links``.

    Args:
        articulation_base_path: Stage path of the articulation root to scan.
        ordered_links: Link names that belong to this articulation. Only joints
            whose endpoints both land on names in this list contribute to the
            result.

    Returns:
        Mapping of link names to the lists of links each should ignore during
        self-collision checks.
    """
    ignore_dict: dict[str, list[str]] = {}

    for prim in Usd.PrimRange(prim_utils.get_prim_at_path(articulation_base_path)):
        if not UsdPhysics.Joint(prim):
            continue
        body0 = prim.GetProperty("physics:body0").GetTargets()
        body1 = prim.GetProperty("physics:body1").GetTargets()

        if len(body0) != 1 or len(body1) != 1:
            continue

        link0 = str(body0[0]).split("/")[-1]
        link1 = str(body1[0]).split("/")[-1]
        if link0 in ordered_links and link1 in ordered_links:
            if link0 in ignore_dict:
                ignore_dict[link0].append(link1)
            else:
                ignore_dict[link0] = [link1]

    # Sibling-pair expansion: if A connects to B, C, and D in `ignore_dict`,
    # add the pairwise ignores B↔C, B↔D, and C↔D into `extended_ignore_dict`
    # so siblings sharing a common parent do not pay self-collision cost.
    extended_ignore_dict = copy.deepcopy(ignore_dict)
    for _, neighbours in ignore_dict.items():
        for i in range(len(neighbours) - 1):
            for j in range(i + 1, len(neighbours)):
                if neighbours[i] in extended_ignore_dict:
                    extended_ignore_dict[neighbours[i]].append(neighbours[j])
                else:
                    extended_ignore_dict[neighbours[i]] = [neighbours[j]]

    return extended_ignore_dict
