# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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

"""USD articulation graph helpers used by articulation metadata queries."""

from __future__ import annotations

from collections import deque

import carb
import omni.physics.tensors
from pxr import Usd, UsdPhysics


def _query_articulation_metadata_from_usd(
    stage: Usd.Stage, path: str
) -> tuple[list[str], list[str], list[str], list[omni.physics.tensors.DofType]]:
    """Query link, joint, and DOF metadata for an articulation from the USD stage.

    Args:
        stage: USD stage to inspect or populate.
        path: Prim path to inspect.

    Returns:
        Link paths, joint paths, DOF paths, and DOF types discovered from the USD stage.
    """
    root_path = _find_containing_articulation_root_path(stage, path)
    if root_path is None:
        carb.log_warn(f"No articulation root found for {path}")
        return [], [], [], []

    root_prim = stage.GetPrimAtPath(root_path)
    scope_prim = _get_articulation_scope_prim(root_prim)
    root_link = _get_root_link_prim(stage, root_prim, scope_prim)
    if root_link is None or not root_link.IsValid():
        carb.log_warn(f"No articulation root link found for {root_path}")
        return [], [], [], []

    candidate_joints = _collect_candidate_joints(scope_prim, root_path)
    body_to_joints = _build_body_to_joints(candidate_joints)
    link_paths, joint_paths = _walk_connected_articulation(root_link, body_to_joints, root_path)
    dof_paths, dof_types = _discover_dofs(stage, joint_paths)
    return link_paths, joint_paths, dof_paths, dof_types


def _find_containing_articulation_root_path(stage: Usd.Stage, path: str) -> str | None:
    """Find the nearest articulation root that owns the given prim path.

    Args:
        stage: USD stage to inspect or populate.
        path: Prim path to inspect.

    Returns:
        Containing articulation root path, or None when no root owns the prim path.
    """
    prim = stage.GetPrimAtPath(path)
    if prim is None or not prim.IsValid():
        return None

    descendant_root = _find_first_articulation_root(prim)
    if descendant_root is not None:
        return descendant_root.GetPath().pathString

    target_path = prim.GetPath().pathString
    current = prim
    while current is not None and current.IsValid():
        current = current.GetParent()
        if current is None or not current.IsValid():
            break
        roots = _find_articulation_roots(current)
        scoped_roots = [
            root
            for root in roots
            if _path_is_at_or_under(target_path, _get_articulation_scope_prim(root).GetPath().pathString)
        ]
        if scoped_roots:
            containing_roots = [
                root
                for root in scoped_roots
                if _articulation_contains_path(stage, root.GetPath().pathString, target_path)
            ]
            candidates = containing_roots or scoped_roots
            candidates.sort(key=_articulation_root_rank, reverse=True)
            return candidates[0].GetPath().pathString
    return None


def _find_first_articulation_root(prim: Usd.Prim) -> Usd.Prim | None:
    """Return the first articulation root at or below the given prim.

    Args:
        prim: Prim or prim wrapper under test.

    Returns:
        First articulation root prim found at or below the input prim, or None if none exists.
    """
    roots = _find_articulation_roots(prim)
    return roots[0] if roots else None


def _find_articulation_roots(prim: Usd.Prim) -> list[Usd.Prim]:
    """Return articulation roots at or below the given prim.

    Args:
        prim: Prim or prim wrapper under test.

    Returns:
        Articulation root prims found at or below the input prim.
    """
    return [candidate for candidate in Usd.PrimRange(prim) if candidate.HasAPI(UsdPhysics.ArticulationRootAPI)]


def _articulation_contains_path(stage: Usd.Stage, root_path: str, target_path: str) -> bool:
    """Check whether the connected articulation graph rooted at ``root_path`` contains ``target_path``.

    Args:
        stage: USD stage to inspect or populate.
        root_path: Articulation root path.
        target_path: Prim path to search for within the articulation.

    Returns:
        True if the connected articulation graph contains the target path, False otherwise.
    """
    root_prim = stage.GetPrimAtPath(root_path)
    if root_prim is None or not root_prim.IsValid():
        return False
    scope_prim = _get_articulation_scope_prim(root_prim)
    root_link = _get_root_link_prim(stage, root_prim, scope_prim)
    if root_link is None or not root_link.IsValid():
        return _path_is_at_or_under(target_path, root_path)

    candidate_joints = _collect_candidate_joints(scope_prim, root_path)
    body_to_joints = _build_body_to_joints(candidate_joints)
    link_paths, joint_paths = _walk_connected_articulation(root_link, body_to_joints, root_path)
    return any(_path_is_at_or_under(target_path, path) for path in [root_path, *link_paths, *joint_paths])


def _articulation_root_rank(root: Usd.Prim) -> tuple[int, int]:
    """Rank articulation root candidates by their scope and root path specificity.

    Args:
        root: Articulation root candidate to rank.

    Returns:
        Sort key that ranks articulation root candidates by namespace specificity.
    """
    scope_path = _get_articulation_scope_prim(root).GetPath().pathString
    root_path = root.GetPath().pathString
    return (len(scope_path), len(root_path))


def _get_articulation_scope_prim(root_prim: Usd.Prim) -> Usd.Prim:
    """Return the prim whose subtree should be searched for articulation joints.

    Args:
        root_prim: Articulation root prim.

    Returns:
        Prim whose subtree should be searched for articulation joints.
    """
    if root_prim.IsA(UsdPhysics.Joint):
        parent = root_prim.GetParent()
        return parent if parent is not None and parent.IsValid() else root_prim
    return root_prim


def _get_root_link_prim(stage: Usd.Stage, root_prim: Usd.Prim, scope_prim: Usd.Prim) -> Usd.Prim | None:
    """Find the root link used to walk the connected articulation graph.

    Args:
        stage: USD stage to inspect or populate.
        root_prim: Articulation root prim.
        scope_prim: Prim whose subtree is searched for articulation data.

    Returns:
        Root link prim used to walk the articulation graph, or None if none can be found.
    """
    if root_prim.IsA(UsdPhysics.Joint):
        for body_path in _joint_body_paths(root_prim):
            if body_path:
                body_prim = stage.GetPrimAtPath(body_path)
                if body_prim is not None and body_prim.IsValid():
                    return body_prim

    if root_prim.HasAPI(UsdPhysics.RigidBodyAPI):
        return root_prim

    for candidate in Usd.PrimRange(scope_prim):
        if candidate.HasAPI(UsdPhysics.RigidBodyAPI):
            return candidate
    return None


def _collect_candidate_joints(scope_prim: Usd.Prim, root_path: str) -> list[Usd.Prim]:
    """Collect joint prims under the articulation scope, excluding nested articulations.

    Args:
        scope_prim: Prim whose subtree is searched for articulation data.
        root_path: Articulation root path.

    Returns:
        Joint prims in the articulation scope excluding nested articulation roots.
    """
    joints: list[Usd.Prim] = []
    for prim in Usd.PrimRange(scope_prim):
        if not prim.IsA(UsdPhysics.Joint):
            continue
        if _is_under_foreign_articulation_root(prim, scope_prim, root_path):
            continue
        joints.append(prim)
    return joints


def _is_under_foreign_articulation_root(prim: Usd.Prim, scope_prim: Usd.Prim, root_path: str) -> bool:
    """Return whether ``prim`` is scoped by a nested articulation root other than ``root_path``.

    Args:
        prim: Prim or prim wrapper under test.
        scope_prim: Prim whose subtree is searched for articulation data.
        root_path: Articulation root path.

    Returns:
        True if the prim is under a nested foreign articulation root, False otherwise.
    """
    current = prim
    scope_path = scope_prim.GetPath()
    while current is not None and current.IsValid():
        current_path = current.GetPath()
        if current.HasAPI(UsdPhysics.ArticulationRootAPI) and current_path.pathString != root_path:
            return True
        if current_path == scope_path:
            return False
        current = current.GetParent()
    return False


def _build_body_to_joints(joints: list[Usd.Prim]) -> dict[str, list[tuple[Usd.Prim, int]]]:
    """Build a map from body path to connected joints and body slot indices.

    Args:
        joints: Joint prims to index by body path.

    Returns:
        Mapping from each body path to its connected joint prims and body slot indices.
    """
    body_to_joints: dict[str, list[tuple[Usd.Prim, int]]] = {}
    for joint in joints:
        for body_index, body_path in enumerate(_joint_body_paths(joint)):
            if body_path:
                body_to_joints.setdefault(body_path, []).append((joint, body_index))
    return body_to_joints


def _walk_connected_articulation(
    root_link: Usd.Prim,
    body_to_joints: dict[str, list[tuple[Usd.Prim, int]]],
    root_path: str,
) -> tuple[list[str], list[str]]:
    """Walk links and joints reachable from ``root_link`` through USD joint relationships.

    Args:
        root_link: Root link used to start articulation graph traversal.
        body_to_joints: Mapping from body paths to connected joints.
        root_path: Articulation root path.

    Returns:
        Reachable link paths and joint paths from the root link.
    """
    link_paths: list[str] = []
    joint_paths: list[str] = []
    visited_links: set[str] = set()
    visited_joints: set[str] = set()
    queue: deque[Usd.Prim] = deque([root_link])
    stage = root_link.GetStage()

    while queue:
        link_prim = queue.popleft()
        if link_prim is None or not link_prim.IsValid():
            continue
        link_path = link_prim.GetPath().pathString
        if link_path in visited_links:
            continue
        visited_links.add(link_path)

        if link_prim.HasAPI(UsdPhysics.RigidBodyAPI):
            link_paths.append(link_path)

        for joint_prim, body_index in body_to_joints.get(link_path, []):
            joint_path = joint_prim.GetPath().pathString
            if joint_path not in visited_joints:
                joint_paths.append(joint_path)
                visited_joints.add(joint_path)

            other_body_path = _joint_body_paths(joint_prim)[1 - body_index]
            if not other_body_path or other_body_path in visited_links:
                continue
            other_body_prim = stage.GetPrimAtPath(other_body_path)
            if other_body_prim is not None and other_body_prim.IsValid():
                queue.append(other_body_prim)

    if root_path not in visited_joints:
        root_prim = stage.GetPrimAtPath(root_path)
        if root_prim is not None and root_prim.IsValid() and root_prim.IsA(UsdPhysics.Joint):
            joint_paths.append(root_path)
    return link_paths, joint_paths


def _discover_dofs(stage: Usd.Stage, joint_paths: list[str]) -> tuple[list[str], list[omni.physics.tensors.DofType]]:
    """Return commandable DOF paths and types from a list of articulation joint paths.

    Args:
        stage: USD stage to inspect or populate.
        joint_paths: Joint paths to inspect.

    Returns:
        Commandable DOF paths and their tensor DOF types.
    """
    dof_paths: list[str] = []
    dof_types: list[omni.physics.tensors.DofType] = []
    for joint_path in joint_paths:
        joint = stage.GetPrimAtPath(joint_path)
        if joint is None or not joint.IsValid() or joint.IsA(UsdPhysics.FixedJoint):
            continue
        dof_type = _get_dof_type(joint)
        if dof_type == omni.physics.tensors.DofType.Invalid:
            continue
        dof_paths.append(joint_path)
        dof_types.append(dof_type)
    return dof_paths, dof_types


def _get_dof_type(joint: Usd.Prim) -> omni.physics.tensors.DofType:
    """Infer the Isaac tensor DOF type for a USD joint prim.

    Args:
        joint: Joint prim to query.

    Returns:
        Tensor DOF type inferred from the joint prim.
    """
    if joint.IsA(UsdPhysics.RevoluteJoint) or joint.IsA(UsdPhysics.SphericalJoint):
        return omni.physics.tensors.DofType.Rotation
    if joint.IsA(UsdPhysics.PrismaticJoint):
        return omni.physics.tensors.DofType.Translation
    for attr in joint.GetAttributes():
        attr_name = attr.GetName()
        if not attr_name.startswith("drive:"):
            continue
        if attr_name.startswith("drive:linear:"):
            return omni.physics.tensors.DofType.Translation
        if attr_name.startswith("drive:angular:"):
            return omni.physics.tensors.DofType.Rotation
        break
    return omni.physics.tensors.DofType.Invalid


def _joint_body_paths(joint: Usd.Prim) -> tuple[str | None, str | None]:
    """Return the resolved body0 and body1 relationship target paths for a joint.

    Args:
        joint: Joint prim to query.

    Returns:
        Resolved body0 and body1 relationship target paths.
    """
    return (_relationship_target_path(joint, "physics:body0"), _relationship_target_path(joint, "physics:body1"))


def _relationship_target_path(prim: Usd.Prim, rel_name: str) -> str | None:
    """Return the first absolute target path for a USD relationship.

    Args:
        prim: Prim or prim wrapper under test.
        rel_name: Relationship name to query.

    Returns:
        First absolute target path for the relationship, or None if it has no target.
    """
    rel = prim.GetRelationship(rel_name)
    if not rel:
        return None
    targets = rel.GetTargets()
    if not targets:
        return None
    target_path = targets[0]
    if not target_path.IsAbsolutePath():
        target_path = target_path.MakeAbsolutePath(prim.GetPath())
    return target_path.pathString


def _path_is_at_or_under(path: str, prefix: str) -> bool:
    """Return whether ``path`` is equal to or below ``prefix`` in namespace.

    Args:
        path: Prim path to inspect.
        prefix: Namespace prefix to compare against.

    Returns:
        True if the path is equal to or below the namespace prefix, False otherwise.
    """
    if prefix == "/":
        return True
    return path == prefix or path.startswith(f"{prefix}/")
