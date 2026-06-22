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
"""Robot discovery from USD stage using physics graph."""

from __future__ import annotations

import logging
from dataclasses import dataclass, field

from pxr import Sdf, Usd, UsdGeom, UsdPhysics

from .transform_utils import get_prim_name

_logger = logging.getLogger(__name__)


@dataclass
class SiteInfo:
    """A SiteAPI prim and its parent rigid body link."""

    prim: Usd.Prim = None
    parent_link_prim: Usd.Prim = None


@dataclass
class RobotDescription:
    """Discovered robot structure from a USD stage."""

    name: str = ""
    root_prim: Usd.Prim = None
    root_link: Usd.Prim = None
    is_fixed_base: bool = True
    ordered_links: list[Usd.Prim] = field(default_factory=list)
    ordered_joints: list[Usd.Prim] = field(default_factory=list)
    loop_joints: list[Usd.Prim] = field(default_factory=list)
    sites: list[SiteInfo] = field(default_factory=list)
    joint_parent_child: dict[str, tuple[Sdf.Path, Sdf.Path]] = field(default_factory=dict)


def find_robot(stage: Usd.Stage, root_prim_path: str | None = None) -> RobotDescription:
    """Discover robot structure from a USD stage.

    Discovery priority:
    1. User-specified root prim path
    2. IsaacRobotAPI on default prim or subtree
    3. ArticulationRootAPI on default prim or subtree
    4. Stage-wide search for ArticulationRootAPI

    Args:
        stage: USD stage to search.
        root_prim_path: Optional explicit root prim path.

    Returns:
        RobotDescription with discovered links and joints.

    Raises:
        ValueError: If no articulated robot is found.
    """
    desc = RobotDescription()

    root_prim = _resolve_root_prim(stage, root_prim_path)
    desc.root_prim = root_prim

    _select_physics_variant(root_prim)
    _ensure_robot_schema(stage, root_prim)

    art_root = _find_articulation_root(root_prim)
    if art_root is None:
        art_root = _find_articulation_root_stage_wide(stage)
    if art_root is None:
        raise ValueError(f"No ArticulationRootAPI found under {root_prim.GetPath()} or in the stage")

    links, joints = _discover_articulation(stage, root_prim, art_root)

    if not links:
        raise ValueError(f"No rigid body links found in articulation rooted at {art_root.GetPath()}")

    desc.root_link = links[0]
    desc.ordered_links = links
    desc.ordered_joints = joints
    desc.loop_joints = _collect_loop_joints(stage, root_prim)
    desc.sites = _collect_sites(links)
    desc.name = get_prim_name(root_prim)
    desc.is_fixed_base = _detect_fixed_base(stage, links[0], joints)
    desc.joint_parent_child = _build_joint_body_map(stage, joints + desc.loop_joints)

    return desc


def _resolve_root_prim(stage: Usd.Stage, root_prim_path: str | None) -> Usd.Prim:
    """Resolve the root prim for robot search.

    Args:
        stage: USD stage to read.
        root_prim_path: Optional root prim path.

    Returns:
        Resolved root prim.
    """
    if root_prim_path:
        path = root_prim_path if root_prim_path.startswith("/") else f"/{root_prim_path}"
        prim = stage.GetPrimAtPath(Sdf.Path(path))
        if prim and prim.IsValid():
            return prim
        _logger.warning(f"Specified root prim '{root_prim_path}' not found, falling back to default prim")

    default_prim = stage.GetDefaultPrim()
    if default_prim and default_prim.IsValid():
        return default_prim

    pseudo_root = stage.GetPseudoRoot()
    children = list(pseudo_root.GetChildren())
    if children:
        return children[0]

    raise ValueError("Stage has no prims")


def _select_physics_variant(prim: Usd.Prim) -> None:
    """Ensure a Physics variant is selected if the variant set exists.

    Only sets a selection when none is currently active. Never overrides
    an existing selection.

    Args:
        prim: USD prim to read.
    """
    vsets = prim.GetVariantSets()
    if not vsets.HasVariantSet("Physics"):
        return

    vset = vsets.GetVariantSet("Physics")
    current = vset.GetVariantSelection()
    if current:
        return

    choices = vset.GetVariantNames()
    lower_map = {c.lower(): c for c in choices}
    if "physx" in lower_map:
        vset.SetVariantSelection(lower_map["physx"])
    elif "physics" in lower_map:
        vset.SetVariantSelection(lower_map["physics"])
    elif choices:
        non_none = [c for c in choices if c.lower() != "none"]
        if non_none:
            vset.SetVariantSelection(non_none[0])
        else:
            vset.SetVariantSelection(choices[0])


def _ensure_robot_schema(stage: Usd.Stage, root_prim: Usd.Prim) -> None:
    """Apply IsaacRobotAPI and populate robot schema relationships if not present.

    This ensures robot_schema utilities (GetJointPose, GenerateRobotLinkTree, etc.)
    work correctly on assets that don't already have the schema applied.

    Args:
        stage: USD stage to read.
        root_prim: Robot root prim.
    """
    try:
        from usd.schema.isaac.robot_schema import ApplyRobotAPI, Classes
        from usd.schema.isaac.robot_schema.utils import PopulateRobotSchemaFromArticulation

        if not root_prim.HasAPI(Classes.ROBOT_API.value):
            ApplyRobotAPI(root_prim)
            _logger.info(f"Applied IsaacRobotAPI to {root_prim.GetPath()}")

        rel = root_prim.GetRelationship("isaac:physics:robotLinks")
        if not rel or not rel.GetTargets():
            PopulateRobotSchemaFromArticulation(stage, root_prim)
            _logger.info(f"Populated robot schema relationships for {root_prim.GetPath()}")
    except ImportError:
        _logger.debug("robot_schema not available, skipping schema population")


def _find_articulation_root(prim: Usd.Prim) -> Usd.Prim | None:
    """Find ArticulationRootAPI on prim or in its subtree (including instance proxies).

    Args:
        prim: USD prim to read.

    Returns:
        Articulation root prim, or None if absent.
    """
    if prim.HasAPI(UsdPhysics.ArticulationRootAPI):
        return prim
    for child in Usd.PrimRange(prim, Usd.TraverseInstanceProxies()):
        if child.HasAPI(UsdPhysics.ArticulationRootAPI):
            return child
    return None


def _find_articulation_root_stage_wide(stage: Usd.Stage) -> Usd.Prim | None:
    """Search the entire stage for an ArticulationRootAPI prim (including instance proxies).

    Args:
        stage: USD stage to read.

    Returns:
        Articulation root prim, or None if absent.
    """
    for prim in stage.TraverseAll():
        if prim.HasAPI(UsdPhysics.ArticulationRootAPI):
            return prim
    return None


def _get_joint_body(joint_prim: Usd.Prim, body_index: int) -> Sdf.Path | None:
    """Get the body relationship target for a joint.

    Args:
        joint_prim: USD joint prim to read.
        body_index: Body relationship index to read.

    Returns:
        Joint body target path, or None if absent.
    """
    joint = UsdPhysics.Joint(joint_prim)
    if not joint:
        return None
    exclude_attr = joint.GetExcludeFromArticulationAttr()
    if exclude_attr and exclude_attr.Get():
        return None
    rel = joint.GetBody0Rel() if body_index == 0 else joint.GetBody1Rel()
    if rel:
        targets = rel.GetTargets()
        if targets:
            return targets[0]
    return None


def _discover_articulation(
    stage: Usd.Stage, robot_prim: Usd.Prim, art_root: Usd.Prim
) -> tuple[list[Usd.Prim], list[Usd.Prim]]:
    """Discover all links and joints by BFS through the joint graph.

    Traverses instance proxies so that joints and bodies inside instanced
    subtrees are found.  When the articulation root prim is not itself a
    rigid body, the actual root body is inferred from the joint graph.

    Args:
        stage: USD stage.
        robot_prim: The robot root prim (search scope for joints).
        art_root: The articulation root prim.

    Returns:
        (ordered_links, ordered_joints)
    """
    from collections import deque

    instance_pred = Usd.TraverseInstanceProxies()
    all_joints = [p for p in Usd.PrimRange(robot_prim, instance_pred) if p.IsA(UsdPhysics.Joint)]

    body_to_joints: dict[str, list[tuple[Usd.Prim, int]]] = {}
    all_body_paths: set[str] = set()

    for j in all_joints:
        body0 = _get_joint_body(j, 0)
        body1 = _get_joint_body(j, 1)
        if body0:
            key = str(body0)
            body_to_joints.setdefault(key, []).append((j, 0))
            all_body_paths.add(key)
        if body1:
            key = str(body1)
            body_to_joints.setdefault(key, []).append((j, 1))
            all_body_paths.add(key)

    root_link = art_root
    root_joint = None

    if art_root.IsA(UsdPhysics.Joint):
        root_joint = art_root
        candidate = _get_joint_body(root_joint, 0) or _get_joint_body(root_joint, 1)
        if candidate:
            root_link = stage.GetPrimAtPath(candidate)
    elif not art_root.HasAPI(UsdPhysics.RigidBodyAPI):
        rigid_body_paths: set[str] = set()
        for path in all_body_paths:
            prim = stage.GetPrimAtPath(Sdf.Path(path))
            if prim and prim.HasAPI(UsdPhysics.RigidBodyAPI):
                rigid_body_paths.add(path)

        rigid_child_paths: set[str] = set()
        for j in all_joints:
            body0 = _get_joint_body(j, 0)
            body1 = _get_joint_body(j, 1)
            if not body1:
                continue
            body1_key = str(body1)
            if body1_key not in rigid_body_paths:
                continue
            if not body0:
                continue
            body0_prim = stage.GetPrimAtPath(body0)
            if body0_prim and body0_prim.HasAPI(UsdPhysics.RigidBodyAPI):
                rigid_child_paths.add(body1_key)

        root_candidates = rigid_body_paths - rigid_child_paths
        if not root_candidates:
            root_candidates = rigid_body_paths or all_body_paths

        art_root_path = str(art_root.GetPath())
        best = None
        for c in root_candidates:
            prim = stage.GetPrimAtPath(Sdf.Path(c))
            if prim and prim.HasAPI(UsdPhysics.RigidBodyAPI):
                if best is None or c.startswith(art_root_path):
                    best = prim
        if best:
            root_link = best

    if not root_link:
        return [], [root_joint] if root_joint else []

    root_link_key = str(root_link.GetPath())

    if not root_joint:
        for j, bi in body_to_joints.get(root_link_key, []):
            root_joint = j
            break

    queue: deque[Usd.Prim] = deque()
    visited_links: set[str] = set()
    visited_joints: set[str] = set()
    ordered_links: list[Usd.Prim] = []
    ordered_joints: list[Usd.Prim] = []

    if root_joint:
        ordered_joints.append(root_joint)
        visited_joints.add(str(root_joint.GetPath()))

    queue.append(root_link)

    while queue:
        link = queue.popleft()
        if not link:
            continue
        lk = str(link.GetPath())
        if lk in visited_links:
            continue
        visited_links.add(lk)
        if link.HasAPI(UsdPhysics.RigidBodyAPI):
            ordered_links.append(link)

        for j, bi in body_to_joints.get(lk, []):
            jk = str(j.GetPath())
            if jk not in visited_joints:
                ordered_joints.append(j)
                visited_joints.add(jk)

            other_bi = 1 - bi
            other_path = _get_joint_body(j, other_bi)
            if not other_path:
                continue
            if str(other_path) in visited_links:
                continue
            other_prim = stage.GetPrimAtPath(other_path)
            if other_prim:
                queue.append(other_prim)

    return ordered_links, ordered_joints


def _detect_fixed_base(stage: Usd.Stage, root_link: Usd.Prim, joints: list[Usd.Prim]) -> bool:
    """Determine if the robot has a fixed base.

    A robot is fixed-base if there is a FixedJoint connecting the root link to:
    - A body0 with no target (world)
    - A body0 targeting a non-RigidBody prim (e.g. the default prim / scene root)

    Args:
        stage: USD stage to read.
        root_link: Value to use.
        joints: Joint data or prims to process.

    Returns:
        True if the robot has a fixed base, False otherwise.
    """
    root_path = str(root_link.GetPath())

    for j in joints:
        if not j.IsA(UsdPhysics.FixedJoint):
            continue

        body0 = _get_joint_body(j, 0)
        body1 = _get_joint_body(j, 1)

        target_is_root = False
        other_body = None

        if body1 and str(body1) == root_path:
            target_is_root = True
            other_body = body0
        elif body0 and str(body0) == root_path:
            target_is_root = True
            other_body = body1

        if not target_is_root:
            continue

        if other_body is None:
            return True

        other_prim = stage.GetPrimAtPath(other_body)
        if other_prim is None or not other_prim.IsValid():
            return True
        if not other_prim.HasAPI(UsdPhysics.RigidBodyAPI):
            return True

    return False


def _build_joint_body_map(
    stage: Usd.Stage, joints: list[Usd.Prim]
) -> dict[str, tuple[Sdf.Path | None, Sdf.Path | None]]:
    """Build a map from joint path to (body0_path, body1_path).

    Uses _get_joint_body_unchecked for loop joints (excludeFromArticulation=true)
    so their body relationships are still captured.

    Args:
        stage: USD stage to read.
        joints: Joint data or prims to process.

    Returns:
        Mapping from joint paths to body target paths.
    """
    result = {}
    for j in joints:
        body0 = _get_joint_body(j, 0) or _get_joint_body_unchecked(j, 0)
        body1 = _get_joint_body(j, 1) or _get_joint_body_unchecked(j, 1)
        result[str(j.GetPath())] = (body0, body1)
    return result


def _get_joint_body_unchecked(joint_prim: Usd.Prim, body_index: int) -> Sdf.Path | None:
    """Get body relationship target ignoring excludeFromArticulation.

    Args:
        joint_prim: USD joint prim to read.
        body_index: Body relationship index to read.

    Returns:
        Joint body target path, or None if absent.
    """
    joint = UsdPhysics.Joint(joint_prim)
    if not joint:
        return None
    rel = joint.GetBody0Rel() if body_index == 0 else joint.GetBody1Rel()
    if rel:
        targets = rel.GetTargets()
        if targets:
            return targets[0]
    return None


def _collect_loop_joints(stage: Usd.Stage, robot_prim: Usd.Prim) -> list[Usd.Prim]:
    """Collect joints with physics:excludeFromArticulation = true (loop/closed-chain joints).

    Args:
        stage: USD stage to read.
        robot_prim: Robot root prim.

    Returns:
        Loop joint prims.
    """
    loop_joints = []
    instance_pred = Usd.TraverseInstanceProxies()
    for prim in Usd.PrimRange(robot_prim, instance_pred):
        if not prim.IsA(UsdPhysics.Joint):
            continue
        joint = UsdPhysics.Joint(prim)
        exclude_attr = joint.GetExcludeFromArticulationAttr()
        if exclude_attr and exclude_attr.Get():
            loop_joints.append(prim)
    return loop_joints


def _collect_sites(links: list[Usd.Prim]) -> list[SiteInfo]:
    """Collect SiteAPI and ReferencePointAPI prims under rigid body links.

    These are child Xforms that represent reference frames (sensor mounts,
    end-effector offsets, etc.) and map to ghost links + fixed joints in URDF.

    Args:
        links: Link prims to inspect.

    Returns:
        Site prims and their parent links.
    """
    sites = []
    instance_pred = Usd.TraverseInstanceProxies()
    for link_prim in links:
        for child in link_prim.GetFilteredChildren(instance_pred):
            if not child.IsA(UsdGeom.Xform):
                continue
            is_site = child.HasAPI("IsaacSiteAPI") or child.HasAPI("IsaacReferencePointAPI")
            if not is_site:
                continue
            if child.HasAPI(UsdPhysics.RigidBodyAPI):
                continue
            sites.append(SiteInfo(prim=child, parent_link_prim=link_prim))
    return sites
