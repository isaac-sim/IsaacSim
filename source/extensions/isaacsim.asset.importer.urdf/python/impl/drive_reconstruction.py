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
"""Reconstruct joint drive / actuator parameters from URDF round-trip breadcrumbs.

When the URDF exporter writes ``isaac:source_drive`` XML comments containing
the original DriveAPI gains, MjcActuator parameters, or PhysxJointAPI armature,
this module parses those comments and applies them to the imported USD stage so
actuation data survives the round-trip.
"""

from __future__ import annotations

import json
import logging
import xml.etree.ElementTree as ET
from dataclasses import dataclass, field

from isaacsim.asset.importer.utils.impl.physx_types import PhysxAttr, PhysxSchema
from pxr import Sdf, Usd, UsdPhysics

_logger = logging.getLogger(__name__)

_BREADCRUMB_PREFIX = " isaac:source_drive "


@dataclass
class SourceDriveInfo:
    """Parsed breadcrumb for a joint's actuation state."""

    joint_name: str = ""
    source: str = ""
    instance: str = ""
    drive: dict = field(default_factory=dict)
    actuator: dict = field(default_factory=dict)
    armature: float | None = None


def parse_source_drive_breadcrumbs(urdf_path: str) -> list[SourceDriveInfo]:
    """Parse ``isaac:source_drive`` XML comments from a URDF file.

    Args:
        urdf_path: Path to the URDF file.

    Returns:
        List of parsed breadcrumb records.
    """
    results = _parse_with_comment_builder(urdf_path)
    if not results:
        results = _parse_with_standard_tree(urdf_path)
    return results


def _parse_with_standard_tree(urdf_path: str) -> list[SourceDriveInfo]:
    """Try parsing comments from the standard ElementTree.

    Args:
        urdf_path: Path to the URDF file.

    Returns:
        Parsed ``isaac:source_drive`` breadcrumb records, or an empty list on parse failure.
    """
    try:
        tree = ET.parse(urdf_path)
    except ET.ParseError:
        _logger.warning(f"Failed to parse URDF XML at {urdf_path}")
        return []

    results: list[SourceDriveInfo] = []
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


def _parse_with_comment_builder(urdf_path: str) -> list[SourceDriveInfo]:
    """Parse using a custom TreeBuilder that captures XML comments.

    Args:
        urdf_path: Path to the URDF file.

    Returns:
        Parsed ``isaac:source_drive`` breadcrumb records, or an empty list on parse failure.
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

    results: list[SourceDriveInfo] = []
    root = tree.getroot()

    for joint_elem in root.iter("joint"):
        joint_name = joint_elem.get("name", "")
        for child in joint_elem:
            text = child.text or ""
            info = _parse_comment(text, joint_name, urdf_path)
            if info:
                results.append(info)
    return results


def _parse_comment(text: str, joint_name: str, urdf_path: str) -> SourceDriveInfo | None:
    """Parse a single comment string for an ``isaac:source_drive`` breadcrumb.

    Args:
        text: Raw XML comment body.
        joint_name: Name of the enclosing URDF joint element.
        urdf_path: Path to the URDF file (for logging).

    Returns:
        Parsed breadcrumb, or ``None`` if the text is not a drive breadcrumb or JSON is invalid.
    """
    if _BREADCRUMB_PREFIX not in text:
        return None
    json_str = text.split(_BREADCRUMB_PREFIX, 1)[1].strip()
    try:
        meta = json.loads(json_str)
    except json.JSONDecodeError:
        _logger.warning(f"Malformed drive breadcrumb JSON in {urdf_path}: {json_str}")
        return None

    return SourceDriveInfo(
        joint_name=joint_name,
        source=meta.get("source", ""),
        instance=meta.get("instance", ""),
        drive=meta.get("drive", {}),
        actuator=meta.get("actuator", {}),
        armature=meta.get("armature"),
    )


def reconstruct_source_drives(stage: Usd.Stage, breadcrumbs: list[SourceDriveInfo]) -> int:
    """Apply saved drive / actuator parameters back onto the USD stage.

    Must be called **after** ``convert_joints_attributes`` so that the
    breadcrumb values overwrite the default synthesis from URDF custom attrs.

    Args:
        stage: The USD stage produced by the URDF converter.
        breadcrumbs: Parsed breadcrumb records from the URDF file.

    Returns:
        Number of joints updated.
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


def _reconstruct_one(stage: Usd.Stage, bc: SourceDriveInfo) -> bool:
    """Reconstruct actuation data for a single joint.

    Args:
        stage: USD stage to modify.
        bc: Parsed drive breadcrumb for one joint.

    Returns:
        ``True`` if any drive, actuator, or armature data was applied.
    """
    if not bc.joint_name:
        return False

    joint_prim = _find_prim_by_name(stage, bc.joint_name)
    if joint_prim is None:
        _logger.warning(f"Joint prim '{bc.joint_name}' not found for drive reconstruction")
        return False

    updated = False

    if bc.source == "physx" and bc.drive:
        instance = bc.instance
        if not instance:
            if joint_prim.IsA(UsdPhysics.RevoluteJoint):
                instance = "angular"
            else:
                instance = "linear"
        drv = UsdPhysics.DriveAPI.Apply(joint_prim, instance)
        if "stiffness" in bc.drive:
            drv.CreateStiffnessAttr().Set(float(bc.drive["stiffness"]))
        if "damping" in bc.drive:
            drv.CreateDampingAttr().Set(float(bc.drive["damping"]))
        if "max_force" in bc.drive:
            drv.CreateMaxForceAttr().Set(float(bc.drive["max_force"]))
        if "target_position" in bc.drive:
            drv.CreateTargetPositionAttr().Set(float(bc.drive["target_position"]))
        updated = True

    elif bc.source == "mujoco" and bc.actuator:
        default_prim = stage.GetDefaultPrim()
        scope_path = default_prim.GetPath().AppendChild("Physics") if default_prim else Sdf.Path("/Physics")
        actuator_name = f"{bc.joint_name}_actuator"
        actuator_path = scope_path.AppendChild(actuator_name)

        actuator_prim = stage.GetPrimAtPath(actuator_path)
        if not actuator_prim.IsValid():
            actuator_prim = stage.DefinePrim(actuator_path, "MjcActuator")

        actuator_prim.CreateRelationship("mjc:target", custom=False).SetTargets([joint_prim.GetPath()])

        act = bc.actuator
        if "gainPrm" in act:
            actuator_prim.CreateAttribute("mjc:gainPrm", Sdf.ValueTypeNames.FloatArray).Set(
                [float(v) for v in act["gainPrm"]]
            )
        if "biasPrm" in act:
            actuator_prim.CreateAttribute("mjc:biasPrm", Sdf.ValueTypeNames.FloatArray).Set(
                [float(v) for v in act["biasPrm"]]
            )
        if "gainType" in act:
            actuator_prim.CreateAttribute("mjc:gainType", Sdf.ValueTypeNames.Token).Set(act["gainType"])
        if "biasType" in act:
            actuator_prim.CreateAttribute("mjc:biasType", Sdf.ValueTypeNames.Token).Set(act["biasType"])
        if "forceRange_min" in act:
            actuator_prim.CreateAttribute("mjc:forceRange:min", Sdf.ValueTypeNames.Float).Set(
                float(act["forceRange_min"])
            )
        if "forceRange_max" in act:
            actuator_prim.CreateAttribute("mjc:forceRange:max", Sdf.ValueTypeNames.Float).Set(
                float(act["forceRange_max"])
            )
        updated = True

    if bc.armature is not None:
        if not joint_prim.HasAPI(PhysxSchema.JOINT_API):
            joint_prim.ApplyAPI(PhysxSchema.JOINT_API)
        joint_prim.CreateAttribute(PhysxAttr.JOINT_ARMATURE.name, PhysxAttr.JOINT_ARMATURE.type).Set(float(bc.armature))
        updated = True

    if updated:
        _logger.debug(f"Reconstructed drive for joint '{bc.joint_name}' (source={bc.source})")

    return updated
