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
    inferred_mimic_limits: list[str] = field(default_factory=list)
    unauthored_effort: bool = False
    #: Whether verified export metadata requests removal of synthesized effort limits.


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
                info = _parse_comment(comment_text, joint_name, urdf_path, joint_elem)
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
            info = _parse_comment(text, joint_name, urdf_path, joint_elem)
            if info:
                results.append(info)
    return results


def _parse_comment(
    text: str, joint_name: str, urdf_path: str, joint_elem: ET.Element | None = None
) -> SourceDriveInfo | None:
    """Parse a single comment string for an ``isaac:source_drive`` breadcrumb.

    Args:
        text: Raw XML comment body.
        joint_name: Name of the enclosing URDF joint element.
        urdf_path: Path to the URDF file (for logging).
        joint_elem: Current joint XML, used to validate inferred-limit provenance.

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
        inferred_mimic_limits=_validate_inferred_mimic_limits(meta.get("inferred_mimic_limits"), joint_elem),
        unauthored_effort=_validate_unauthored_effort(meta.get("unauthored_effort"), joint_elem),
    )


def _validate_unauthored_effort(metadata: dict | None, joint: ET.Element | None) -> bool:
    """Validate synthesized effort provenance against the current URDF.

    Args:
        metadata: Exporter's unauthored effort record.
        joint: Current joint XML.

    Returns:
        Whether the generated effort and joint relationship are unchanged.
    """
    if not isinstance(metadata, dict) or joint is None:
        return False
    limit = joint.find("limit")
    mimic = joint.find("mimic")
    return (
        limit is not None
        and isinstance(metadata.get("effort"), str)
        and metadata["effort"] == limit.get("effort")
        and metadata.get("joint_type") == joint.get("type")
        and metadata.get("mimic") == (dict(mimic.attrib) if mimic is not None else None)
    )


def restore_unauthored_effort_limits(stage: Usd.Stage, breadcrumbs: list[SourceDriveInfo]) -> None:
    """Remove generated effort opinions after import and physics conversion.

    Args:
        stage: Imported USD stage.
        breadcrumbs: Verified source drive metadata.
    """
    for record in breadcrumbs:
        if not record.unauthored_effort:
            continue
        prim = _find_prim_by_name(stage, record.joint_name)
        if prim is None:
            continue
        for instance in ("angular", "linear"):
            prim.RemoveProperty(f"drive:{instance}:physics:maxForce")
        for name in ("urdf:limit:effort", "mjc:actuatorfrcrange:min", "mjc:actuatorfrcrange:max"):
            prim.RemoveProperty(name)
        for actuator in stage.Traverse():
            target = actuator.GetRelationship("mjc:target")
            if target and prim.GetPath() in target.GetTargets():
                for side in ("min", "max"):
                    actuator.RemoveProperty(f"mjc:forceRange:{side}")


def _validate_inferred_mimic_limits(metadata: dict | None, joint_elem: ET.Element | None) -> list[str]:
    """Accept provenance only for unchanged emitted limits and mimic relationships.

    Args:
        metadata: Inference record embedded by the exporter.
        joint_elem: Current URDF joint element.

    Returns:
        Names of unchanged inferred limits; empty for absent, stale, or malformed metadata.
    """
    if not isinstance(metadata, dict) or joint_elem is None:
        return []
    mimic = joint_elem.find("mimic")
    limit = joint_elem.find("limit")
    if mimic is None or limit is None:
        return []
    if metadata.get("joint_type") != joint_elem.get("type") or metadata.get("mimic") != dict(mimic.attrib):
        return []
    values = metadata.get("limits")
    if not isinstance(values, dict):
        return []
    return [
        name
        for name in ("velocity", "lower", "upper")
        if isinstance(values.get(name), str) and values[name] == limit.get(name)
    ]


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

    # Derived URDF bounds describe ideal coupling, not independent USD stops.
    # Restoring redundant follower speed caps can destabilize the PhysX solver.
    is_mimic = joint_prim.HasAPI("NewtonMimicAPI") or any(
        schema.startswith("PhysxMimicJointAPI:") for schema in joint_prim.GetAppliedSchemas()
    )
    if is_mimic and bc.inferred_mimic_limits:
        if "velocity" in bc.inferred_mimic_limits:
            joint_prim.CreateAttribute("newton:velocityLimit", Sdf.ValueTypeNames.Float).Set(float("inf"))
            joint_prim.CreateAttribute(PhysxAttr.JOINT_MAX_VELOCITY.name, PhysxAttr.JOINT_MAX_VELOCITY.type).Set(
                float("inf")
            )
            updated = True
        if joint_prim.IsA(UsdPhysics.RevoluteJoint) or joint_prim.IsA(UsdPhysics.PrismaticJoint):
            for name, value in (("lower", -float("inf")), ("upper", float("inf"))):
                if name in bc.inferred_mimic_limits:
                    joint_prim.CreateAttribute(f"physics:{name}Limit", Sdf.ValueTypeNames.Float).Set(value)
                    updated = True

    if updated:
        _logger.debug(f"Reconstructed drive for joint '{bc.joint_name}' (source={bc.source})")

    return updated
