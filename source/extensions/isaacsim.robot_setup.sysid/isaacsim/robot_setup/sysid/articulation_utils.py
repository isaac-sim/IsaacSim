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

"""USD articulation root discovery for the SysID extension."""

from __future__ import annotations

from collections import deque
from typing import TYPE_CHECKING, Any

import carb

if TYPE_CHECKING:
    from pxr import Usd


def find_articulation_root(stage: Usd.Stage, robot_path: str) -> str | None:
    """Return the articulation root associated with a selected robot prim.

    Articulation roots are read directly from ``UsdPhysics`` because
    ``isaacsim.core.experimental`` offers no equivalent nearest-root query, and the
    caller may pass any descendant of the articulation.

    Args:
        stage: USD stage containing the selected robot.
        robot_path: Robot prim path or a descendant of its articulation.

    Returns:
        Articulation-root prim path, or ``None`` when no unique root can be resolved.
    """
    # `pxr` ships with Kit and with the optional `usd-core` dependency, so it is
    # imported per call to keep the portable package importable without USD.
    from pxr import Sdf, Usd, UsdPhysics

    prim = stage.GetPrimAtPath(robot_path)
    if not prim.IsValid():
        return None

    if prim.HasAPI(UsdPhysics.ArticulationRootAPI):
        return str(prim.GetPath())

    candidates: list[tuple[int, str]] = []
    selected_depth = prim.GetPath().pathElementCount
    for child in Usd.PrimRange(prim):
        if child.HasAPI(UsdPhysics.ArticulationRootAPI):
            path = child.GetPath()
            candidates.append((path.pathElementCount - selected_depth, str(path)))

    if candidates:
        nearest_depth = min(depth for depth, _path in candidates)
        nearest = sorted(path for depth, path in candidates if depth == nearest_depth)
        if len(nearest) > 1:
            carb.log_warn(
                f"SysId: multiple articulation roots are equally close below '{robot_path}': {nearest}. "
                "Select the intended articulation root explicitly."
            )
            return None
        return nearest[0]

    # A selected link may live below an articulation root. Walk the ancestor
    # chain, but inspect only each ancestor itself so sibling robots cannot be
    # selected accidentally.
    parent_path = Sdf.Path(robot_path).GetParentPath()
    while parent_path != Sdf.Path.absoluteRootPath:
        parent = stage.GetPrimAtPath(parent_path)
        if parent.IsValid() and parent.HasAPI(UsdPhysics.ArticulationRootAPI):
            return str(parent_path)
        parent_path = parent_path.GetParentPath()
    return None


def collect_robot_link_and_joint_paths(stage: Usd.Stage, robot_path: str) -> tuple[list[str], list[str]]:
    """Collect rigid links and supported scalar joints below an articulation root.

    Args:
        stage: USD stage containing the robot.
        robot_path: Robot prim path used to resolve the articulation root.

    Returns:
        Rigid-link paths and supported scalar-joint paths below the articulation.
    """
    from pxr import UsdPhysics

    root = find_articulation_root(stage, robot_path) or robot_path
    root_prim = stage.GetPrimAtPath(root)
    if not root_prim or not root_prim.IsValid():
        raise ValueError(f"Robot prim path is not valid: {robot_path}")

    links: list[str] = []
    joints: list[str] = []
    pending = deque([root_prim])
    while pending:
        prim = pending.popleft()
        if prim.HasAPI(UsdPhysics.RigidBodyAPI) or prim.HasAPI(UsdPhysics.MassAPI):
            links.append(str(prim.GetPath()))
        if prim.IsA(UsdPhysics.RevoluteJoint) or prim.IsA(UsdPhysics.PrismaticJoint):
            joints.append(str(prim.GetPath()))
        pending.extend(prim.GetChildren())

    if not links:
        links = [str(root_prim.GetPath())]

    if not joints:
        # Schema-first: an articulation's joints reference its rigid bodies through the
        # physics:body0/body1 relationships, so prefer that authoritative wiring. It finds
        # joints authored in a sibling scope (e.g. ``<robot>/Physics``) regardless of tree
        # layout and never adopts a different robot's joints from a shared parent scope.
        joints = _joints_wired_to_links(stage, robot_path, root_prim, set(links))

    if not joints:
        # Fallback: joints that do not author body relationships cannot be matched by
        # schema, so fall back to tree proximity -- search the selected robot's own subtree
        # first (a sibling joint scope lives under it), then climb ancestors if still empty.
        joints = _joints_by_proximity(stage, robot_path, root_prim)

    return links, joints


def _scalar_joints_in_subtree(scope: Any, usd_physics: Any) -> list[Any]:
    """Collect the revolute/prismatic joint prims in a subtree (breadth-first).

    Args:
        scope: Root prim of the subtree to search.
        usd_physics: The imported ``pxr.UsdPhysics`` module.

    Returns:
        The scalar-joint prims found under ``scope``.
    """
    joints: list[Any] = []
    pending = deque([scope])
    while pending:
        prim = pending.popleft()
        if prim.IsA(usd_physics.RevoluteJoint) or prim.IsA(usd_physics.PrismaticJoint):
            joints.append(prim)
        pending.extend(prim.GetChildren())
    return joints


def _joint_references_links(joint_prim: Any, link_paths: set[str]) -> bool:
    """Whether a joint's ``physics:body0``/``body1`` targets one of ``link_paths``.

    Args:
        joint_prim: The scalar-joint prim to inspect.
        link_paths: Rigid-body link paths that belong to the robot.

    Returns:
        True when a connected body is one of ``link_paths`` (or a descendant of one).
    """
    for rel_name in ("physics:body0", "physics:body1"):
        rel = joint_prim.GetRelationship(rel_name)
        if not rel:
            continue
        for target in rel.GetTargets():
            target_path = str(target)
            if any(target_path == link or target_path.startswith(link + "/") for link in link_paths):
                return True
    return False


def _joints_wired_to_links(stage: Usd.Stage, robot_path: str, root_prim: Any, link_paths: set[str]) -> list[str]:
    """Joints wired to this robot's bodies, searched from the robot up its ancestors.

    Body membership (``physics:body0``/``body1``) makes joint discovery independent of the
    prim-tree layout and keeps a shared parent scope from leaking a different robot's
    joints into the result.

    Args:
        stage: Stage under inspection.
        robot_path: The originally selected robot prim path.
        root_prim: The resolved articulation-root prim.
        link_paths: Rigid-body link paths that belong to this robot.

    Returns:
        The scalar-joint paths in the nearest scope that references the robot's bodies.
    """
    from pxr import UsdPhysics

    scope = stage.GetPrimAtPath(robot_path)
    if not (scope and scope.IsValid()):
        scope = root_prim
    found: list[str] = []
    while scope and scope.IsValid() and not scope.IsPseudoRoot() and not found:
        found = [
            str(joint.GetPath())
            for joint in _scalar_joints_in_subtree(scope, UsdPhysics)
            if _joint_references_links(joint, link_paths)
        ]
        scope = scope.GetParent()
    return found


def _joints_by_proximity(stage: Usd.Stage, robot_path: str, root_prim: Any) -> list[str]:
    """Scalar joints found by tree proximity, for robots whose joints lack body wiring.

    Searches the selected robot's own subtree first (a sibling joint scope lives under it),
    then climbs ancestors, returning the joints in the nearest scope that contains any.

    Args:
        stage: Stage under inspection.
        robot_path: The originally selected robot prim path.
        root_prim: The resolved articulation-root prim.

    Returns:
        The scalar-joint paths in the nearest ancestor scope that contains any.
    """
    from pxr import UsdPhysics

    scope = stage.GetPrimAtPath(robot_path)
    if not (scope and scope.IsValid()):
        scope = root_prim
    found: list[str] = []
    while scope and scope.IsValid() and not scope.IsPseudoRoot() and not found:
        found = [str(joint.GetPath()) for joint in _scalar_joints_in_subtree(scope, UsdPhysics)]
        scope = scope.GetParent()
    return found


def order_joint_paths_for_trajectory(joint_paths: list[str], trajectory: Any) -> list[str]:
    """Order joint paths using telemetry ``joint_names`` metadata when available.

    Every consumer downstream (USD snapshot baselines, all environment bridges)
    assumes ``joint_paths[i]`` corresponds to telemetry column ``i``. The USD
    traversal order is NOT guaranteed to match — on a Kit-composed stage the
    children enumeration can differ from the authored order, silently permuting
    the mapping. Position fitting is self-consistent under such a permutation
    (each wrongly-mapped joint still tracks the command it was given), so only
    physically grounded channels (measured torque, controller feedforward)
    expose it. When the telemetry mapping declares ``joint_names``, use them as
    the authoritative order; unmatched joints (for example grippers) keep
    traversal order at the tail. Ambiguous leaf names (mirrored or multi-arm
    layouts that repeat ``joint1``) fall back to traversal order rather than
    binding telemetry to an arbitrary one of the candidates.

    Args:
        joint_paths: Candidate USD joint prim paths.
        trajectory: Loaded trajectory whose provenance may define joint order.

    Returns:
        Joint paths ordered to match telemetry joint names when available.
    """
    metadata = getattr(trajectory, "metadata", None)
    extra = getattr(metadata, "extra", None)
    names = None
    if isinstance(extra, dict):
        names = extra.get("joint_names")
        if not names:
            for key in ("topic_mapping", "column_mapping"):
                nested = extra.get(key)
                if isinstance(nested, dict) and nested.get("joint_names"):
                    names = nested["joint_names"]
                    break
    if not names:
        return joint_paths

    by_leaf: dict[str, str] = {}
    duplicate_leaves: set[str] = set()
    for path in joint_paths:
        path_text = str(path)
        leaf = path_text.rsplit("/", 1)[-1]
        existing = by_leaf.get(leaf)
        if existing is not None:
            duplicate_leaves.add(leaf)
            carb.log_warn(
                f"SysId: multiple joint prims share leaf name '{leaf}': '{existing}' and '{path_text}'. "
                "Telemetry joint-name ordering is ambiguous."
            )
            continue
        by_leaf[leaf] = path_text
    ordered: list[str] = []
    for name in names:
        if str(name) in duplicate_leaves:
            carb.log_warn(f"SysId: telemetry joint '{name}' matches multiple joint prims; keeping USD traversal order.")
            return joint_paths
        path = by_leaf.get(str(name))
        if path is None:
            carb.log_warn(
                f"SysId: telemetry joint '{name}' has no matching joint prim; keeping USD traversal order "
                "(verify the robot import and topic mapping)."
            )
            return joint_paths
        ordered.append(path)
    ordered_set = set(ordered)
    remainder = [path for path in joint_paths if path not in ordered_set]
    if ordered != joint_paths[: len(ordered)]:
        reordered = [path.rsplit("/", 1)[-1] for path in ordered]
        carb.log_warn(
            f"SysId: reordered joint paths to match telemetry joint_names (USD traversal order differed): {reordered}"
        )
    return ordered + remainder
