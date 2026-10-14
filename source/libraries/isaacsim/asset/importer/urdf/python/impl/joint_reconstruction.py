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
"""Reconstruct original USD joint types from URDF round-trip breadcrumbs.

When the URDF exporter decomposes multi-DOF USD joints (SphericalJoint,
D6Joint) into chains of single-DOF URDF joints with ghost links, it
embeds ``isaac:source_joint`` XML comments containing the original joint
parameters.  This module parses those comments and collapses the chains
back into single multi-DOF USD joints so the round-trip is lossless.
"""

from __future__ import annotations

import json
import logging
import xml.etree.ElementTree as ET
from dataclasses import dataclass, field

from pxr import Gf, Sdf, Usd, UsdPhysics

_logger = logging.getLogger(__name__)

_BREADCRUMB_PREFIX = " isaac:source_joint "


@dataclass
class SourceJointInfo:
    """Parsed breadcrumb for a multi-DOF joint chain."""

    joint_name: str = ""
    original_type: str = ""
    original_name: str = ""
    chain_joints: list[str] = field(default_factory=list)
    ghost_links: list[str] = field(default_factory=list)
    per_axis_limits: dict = field(default_factory=dict)
    per_axis_drives: dict = field(default_factory=dict)
    local_poses: dict = field(default_factory=dict)
    params: dict = field(default_factory=dict)


def parse_source_joint_breadcrumbs(urdf_path: str) -> list[SourceJointInfo]:
    """Parse ``isaac:source_joint`` XML comments from a URDF file.

    Args:
        urdf_path: Path to the URDF file.

    Returns:
        List of parsed breadcrumb records.
    """
    results = _parse_with_comment_builder(urdf_path)
    if not results:
        results = _parse_with_standard_tree(urdf_path)
    return results


def _parse_with_standard_tree(urdf_path: str) -> list[SourceJointInfo]:
    """Try parsing comments from the standard ElementTree (some versions expose them).

    Args:
        urdf_path: Path to the URDF file.

    Returns:
        Parsed ``isaac:source_joint`` breadcrumb records, or an empty list on parse failure.
    """
    try:
        tree = ET.parse(urdf_path)
    except ET.ParseError:
        _logger.warning(f"Failed to parse URDF XML at {urdf_path}")
        return []

    results: list[SourceJointInfo] = []
    root = tree.getroot()

    for joint_elem in root.iter("joint"):
        joint_name = joint_elem.get("name", "")
        for child in joint_elem:
            if not isinstance(child.tag, str) and callable(child.tag):
                comment_text = child.text or ""
                info = _parse_comment(comment_text, joint_name, urdf_path)
                if info:
                    results.append(info)
    return results


def _parse_with_comment_builder(urdf_path: str) -> list[SourceJointInfo]:
    """Parse using a custom TreeBuilder that captures XML comments.

    Args:
        urdf_path: Path to the URDF file.

    Returns:
        Parsed ``isaac:source_joint`` breadcrumb records, or an empty list on parse failure.
    """

    class _CommentedTreeBuilder(ET.TreeBuilder):
        def comment(self, data: str) -> None:  # type: ignore[override]
            self.start(ET.Comment, {})
            self.data(data)
            self.end(ET.Comment)  # type: ignore[arg-type]

    try:
        parser = ET.XMLParser(target=_CommentedTreeBuilder())
        tree = ET.parse(urdf_path, parser=parser)
    except ET.ParseError:
        return []

    results: list[SourceJointInfo] = []
    root = tree.getroot()

    for joint_elem in root.iter("joint"):
        joint_name = joint_elem.get("name", "")
        for child in joint_elem:
            text = child.text or ""
            info = _parse_comment(text, joint_name, urdf_path)
            if info:
                results.append(info)
    return results


def _parse_comment(text: str, joint_name: str, urdf_path: str) -> SourceJointInfo | None:
    """Parse a single comment string for an ``isaac:source_joint`` breadcrumb.

    Args:
        text: Raw XML comment body.
        joint_name: Name of the enclosing URDF joint element.
        urdf_path: Path to the URDF file (for logging).

    Returns:
        Parsed breadcrumb, or ``None`` if the text is not a joint breadcrumb or JSON is invalid.
    """
    if _BREADCRUMB_PREFIX not in text:
        return None
    json_str = text.split(_BREADCRUMB_PREFIX, 1)[1].strip()
    try:
        meta = json.loads(json_str)
    except json.JSONDecodeError:
        _logger.warning(f"Malformed joint breadcrumb JSON in {urdf_path}: {json_str}")
        return None

    info = SourceJointInfo(
        joint_name=joint_name,
        original_type=meta.get("type", ""),
        original_name=meta.get("original_name", ""),
        chain_joints=meta.get("chain_joints", []),
        ghost_links=meta.get("ghost_links", []),
        per_axis_limits=meta.get("per_axis_limits", {}),
        per_axis_drives=meta.get("per_axis_drives", {}),
        params=meta,
    )
    for key in ("local_pos0", "local_rot0", "local_pos1", "local_rot1"):
        if key in meta:
            info.local_poses[key] = meta[key]
    return info


def reconstruct_source_joints(stage: Usd.Stage, breadcrumbs: list[SourceJointInfo]) -> int:
    """Collapse chain joints and ghost links back into original USD joint types.

    Args:
        stage: The USD stage produced by the URDF converter.
        breadcrumbs: Parsed breadcrumb records from the URDF file.

    Returns:
        Number of joints reconstructed.
    """
    if not breadcrumbs:
        return 0

    count = 0
    for bc in breadcrumbs:
        if _reconstruct_one(stage, bc):
            count += 1

    return count


def _find_prim_by_name(stage: Usd.Stage, name: str) -> Usd.Prim | None:
    """Find a prim anywhere on the stage whose GetName() matches *name*.

    Args:
        stage: USD stage to search.
        name: Prim name to match (``GetName()``).

    Returns:
        The first matching prim, or ``None`` if not found.
    """
    for prim in stage.Traverse():
        if prim.GetName() == name:
            return prim
    return None


def _reconstruct_one(stage: Usd.Stage, bc: SourceJointInfo) -> bool:
    """Reconstruct a single multi-DOF joint from its chain representation.

    Args:
        stage: USD stage to modify.
        bc: Parsed joint breadcrumb describing the chain and original joint.

    Returns:
        ``True`` if the chain was collapsed into a reconstructed USD joint.
    """
    if not bc.chain_joints:
        return False

    chain_prims: list[Usd.Prim] = []
    for jname in bc.chain_joints:
        prim = _find_prim_by_name(stage, jname)
        if prim is None:
            _logger.warning(f"Chain joint prim '{jname}' not found for reconstruction of '{bc.original_name}'")
            return False
        chain_prims.append(prim)

    ghost_prims: list[Usd.Prim] = []
    for gname in bc.ghost_links:
        prim = _find_prim_by_name(stage, gname)
        if prim is None:
            _logger.warning(f"Ghost link prim '{gname}' not found for reconstruction of '{bc.original_name}'")
            return False
        ghost_prims.append(prim)

    first_joint = UsdPhysics.Joint(chain_prims[0])
    last_joint = UsdPhysics.Joint(chain_prims[-1])
    if not first_joint or not last_joint:
        return False

    body0_targets = first_joint.GetBody0Rel().GetTargets() if first_joint.GetBody0Rel() else []
    body1_targets = last_joint.GetBody1Rel().GetTargets() if last_joint.GetBody1Rel() else []
    parent_path = body0_targets[0] if body0_targets else None
    child_path = body1_targets[0] if body1_targets else None

    if parent_path is None or child_path is None:
        _logger.warning(f"Could not determine parent/child for joint '{bc.original_name}'")
        return False

    # Determine where to place the new joint
    first_prim_parent = chain_prims[0].GetPath().GetParentPath()
    new_joint_path = first_prim_parent.AppendChild(bc.original_name)

    # Remove chain joints and ghost links
    paths_to_remove: list[Sdf.Path] = []
    for p in chain_prims:
        paths_to_remove.append(p.GetPath())
    for p in ghost_prims:
        paths_to_remove.append(p.GetPath())

    for path in paths_to_remove:
        stage.RemovePrim(path)

    # Define the new joint
    usd_type = bc.original_type
    if usd_type == "PhysicsSphericalJoint":
        new_joint_api = UsdPhysics.SphericalJoint.Define(stage, new_joint_path)
    elif usd_type in ("PhysicsD6Joint", "PhysicsJoint"):
        new_joint_api = UsdPhysics.Joint.Define(stage, new_joint_path)
    else:
        new_joint_api = UsdPhysics.Joint.Define(stage, new_joint_path)

    new_joint_api.CreateBody0Rel().SetTargets([parent_path])
    new_joint_api.CreateBody1Rel().SetTargets([child_path])

    # Restore local poses
    poses = bc.local_poses
    if "local_pos0" in poses:
        p = poses["local_pos0"]
        new_joint_api.CreateLocalPos0Attr().Set(Gf.Vec3f(p[0], p[1], p[2]))
    if "local_rot0" in poses:
        q = poses["local_rot0"]
        new_joint_api.CreateLocalRot0Attr().Set(Gf.Quatf(q[0], q[1], q[2], q[3]))
    if "local_pos1" in poses:
        p = poses["local_pos1"]
        new_joint_api.CreateLocalPos1Attr().Set(Gf.Vec3f(p[0], p[1], p[2]))
    if "local_rot1" in poses:
        q = poses["local_rot1"]
        new_joint_api.CreateLocalRot1Attr().Set(Gf.Quatf(q[0], q[1], q[2], q[3]))

    joint_prim = new_joint_api.GetPrim()

    # Restore per-axis limits
    for axis_token, limits in bc.per_axis_limits.items():
        lim = UsdPhysics.LimitAPI.Apply(joint_prim, axis_token)
        if "low" in limits:
            lim.CreateLowAttr().Set(float(limits["low"]))
        if "high" in limits:
            lim.CreateHighAttr().Set(float(limits["high"]))

    # Restore per-axis drives
    for axis_token, drive_params in bc.per_axis_drives.items():
        drv = UsdPhysics.DriveAPI.Apply(joint_prim, axis_token)
        if "damping" in drive_params:
            drv.CreateDampingAttr().Set(float(drive_params["damping"]))
        if "stiffness" in drive_params:
            drv.CreateStiffnessAttr().Set(float(drive_params["stiffness"]))
        if "max_force" in drive_params:
            drv.CreateMaxForceAttr().Set(float(drive_params["max_force"]))

    _logger.debug(f"Reconstructed {usd_type} '{bc.original_name}' at {new_joint_path}")
    return True
