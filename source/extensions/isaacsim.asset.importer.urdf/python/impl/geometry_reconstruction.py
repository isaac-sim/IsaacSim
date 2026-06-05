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
"""Reconstruct original USD geometry primitives from URDF round-trip breadcrumbs.

When the URDF exporter converts USD-only primitives (Capsule, Cone) into
URDF-compatible geometry, it embeds ``isaac:source_geometry`` XML comments
containing the original parameters.  This module parses those comments and
replaces the converted geometry on the imported USD stage with the correct
primitive types so the round-trip is lossless.
"""

from __future__ import annotations

import json
import logging
import xml.etree.ElementTree as ET
from collections import defaultdict
from dataclasses import dataclass, field

from pxr import Sdf, Usd, UsdGeom, UsdPhysics

_logger = logging.getLogger(__name__)

_BREADCRUMB_PREFIX = " isaac:source_geometry "
_GEOMETRY_TYPES = {"Mesh", "Cube", "Sphere", "Capsule", "Cylinder", "Cone"}


@dataclass
class SourceGeometryInfo:
    """Parsed breadcrumb for a single visual/collision element."""

    link_name: str = ""
    element_type: str = ""  # "visual" or "collision"
    element_name: str = ""
    original_type: str = ""  # "Capsule" or "Cone"
    role: str = ""  # "body", "top_cap", "bottom_cap", or ""
    params: dict = field(default_factory=dict)


def parse_source_geometry_breadcrumbs(urdf_path: str) -> list[SourceGeometryInfo]:
    """Parse ``isaac:source_geometry`` XML comments from a URDF file.

    Args:
        urdf_path: Path to the URDF file.

    Returns:
        List of parsed breadcrumb records.
    """
    try:
        tree = ET.parse(urdf_path)
    except ET.ParseError:
        _logger.warning(f"Failed to parse URDF XML at {urdf_path}")
        return []

    results: list[SourceGeometryInfo] = []
    root = tree.getroot()

    for link_elem in root.iter("link"):
        link_name = link_elem.get("name", "")

        for tag in ("visual", "collision"):
            for elem in link_elem.iter(tag):
                elem_name = elem.get("name", "")
                for child in elem:
                    if not isinstance(child.tag, str) and callable(child.tag):
                        # XML comment — child.text contains the comment body
                        comment_text = child.text or ""
                        if _BREADCRUMB_PREFIX not in comment_text:
                            continue
                        json_str = comment_text.split(_BREADCRUMB_PREFIX, 1)[1].strip()
                        try:
                            meta = json.loads(json_str)
                        except json.JSONDecodeError:
                            _logger.warning(f"Malformed breadcrumb JSON in {urdf_path}: {json_str}")
                            continue
                        results.append(
                            SourceGeometryInfo(
                                link_name=link_name,
                                element_type=tag,
                                element_name=elem_name,
                                original_type=meta.get("type", ""),
                                role=meta.get("role", ""),
                                params=meta,
                            )
                        )

    # ElementTree exposes comments via iteration only if we use a custom parser.
    # Re-parse with a comment-aware approach if we got nothing.
    if not results:
        results = _parse_breadcrumbs_with_comments(urdf_path)

    return results


def _parse_breadcrumbs_with_comments(urdf_path: str) -> list[SourceGeometryInfo]:
    """Fallback parser that explicitly handles XML comments via TreeBuilder.

    Args:
        urdf_path: Path to the URDF file.

    Returns:
        Parsed ``isaac:source_geometry`` breadcrumb records, or an empty list on parse failure.
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

    results: list[SourceGeometryInfo] = []
    root = tree.getroot()

    for link_elem in root.iter("link"):
        link_name = link_elem.get("name", "")

        for tag in ("visual", "collision"):
            for elem in link_elem.iter(tag):
                elem_name = elem.get("name", "")
                for child in elem:
                    text = child.text or ""
                    if _BREADCRUMB_PREFIX not in text:
                        continue
                    json_str = text.split(_BREADCRUMB_PREFIX, 1)[1].strip()
                    try:
                        meta = json.loads(json_str)
                    except json.JSONDecodeError:
                        _logger.warning(f"Malformed breadcrumb JSON in {urdf_path}: {json_str}")
                        continue
                    results.append(
                        SourceGeometryInfo(
                            link_name=link_name,
                            element_type=tag,
                            element_name=elem_name,
                            original_type=meta.get("type", ""),
                            role=meta.get("role", ""),
                            params=meta,
                        )
                    )

    return results


def reconstruct_source_geometry(stage: Usd.Stage, breadcrumbs: list[SourceGeometryInfo]) -> int:
    """Replace converted geometry with original USD primitives on the stage.

    Args:
        stage: The USD stage produced by the URDF converter.
        breadcrumbs: Parsed breadcrumb records from the URDF file.

    Returns:
        Number of primitives reconstructed.
    """
    if not breadcrumbs:
        return 0

    # Group by (link_name, original_type, source_prim_name)
    groups: dict[tuple[str, str, str], list[SourceGeometryInfo]] = defaultdict(list)
    for bc in breadcrumbs:
        key = (bc.link_name, bc.original_type, bc.params.get("source_prim_name", ""))
        groups[key].append(bc)

    count = 0

    for (link_name, orig_type, source_name), entries in groups.items():
        if orig_type == "Capsule":
            if _reconstruct_capsule(stage, link_name, source_name, entries):
                count += 1
        elif orig_type == "Cone":
            for entry in entries:
                if _reconstruct_cone(stage, link_name, source_name, entry):
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


def _find_geometry_child(prim: Usd.Prim) -> Usd.Prim | None:
    """Find the first geometry-typed child of a prim.

    Args:
        prim: Parent prim whose direct children are searched.

    Returns:
        First child whose type is a known geometry schema, or ``None``.
    """
    for child in prim.GetAllChildren():
        if child.GetTypeName() in _GEOMETRY_TYPES:
            return child
    return None


def _copy_xform(src: Usd.Prim, dst: Usd.Prim) -> None:
    """Copy all XformOps from *src* to *dst*.

    Args:
        src: Prim providing the ordered xform ops and values.
        dst: Prim that receives a cleared op order and duplicated ops.
    """
    src_xf = UsdGeom.Xformable(src)
    dst_xf = UsdGeom.Xformable(dst)
    if not src_xf or not dst_xf:
        return
    dst_xf.ClearXformOpOrder()
    for op in src_xf.GetOrderedXformOps():
        new_op = dst_xf.AddXformOp(op.GetOpType(), op.GetPrecision(), op.GetOpName().replace("xformOp:", ""))
        val = op.Get()
        if val is not None:
            new_op.Set(val)


def _copy_purpose(src: Usd.Prim, dst: Usd.Prim) -> None:
    """Copy the imageable purpose attribute from *src* to *dst*.

    Args:
        src: Imageable prim whose purpose is read (if set).
        dst: Imageable prim whose purpose is written.
    """
    src_img = UsdGeom.Imageable(src)
    dst_img = UsdGeom.Imageable(dst)
    if src_img and dst_img:
        purpose = src_img.GetPurposeAttr().Get()
        if purpose:
            dst_img.GetPurposeAttr().Set(purpose)


def _reconstruct_capsule(stage: Usd.Stage, link_name: str, source_name: str, entries: list[SourceGeometryInfo]) -> bool:
    """Replace cylinder + 2 sphere prims with a single UsdGeom.Capsule.

    Args:
        stage: USD stage to modify.
        link_name: URDF link name for the capsule group.
        source_name: Original USD prim name for the reconstructed capsule.
        entries: Breadcrumb records for all capsule parts (body, caps, etc.).

    Returns:
        ``True`` if a capsule was created and old prims removed successfully.
    """
    roles = {e.role: e for e in entries}
    body_entry = roles.get("body")
    if body_entry is None:
        _logger.warning(f"Capsule group '{source_name}' in link '{link_name}' missing body entry")
        return False

    params = body_entry.params
    radius = params.get("radius", 0.5)
    height = params.get("height", 1.0)
    axis = params.get("axis", "Z")

    # Find all prims to remove
    prims_to_remove: list[Sdf.Path] = []
    body_geom_prim: Usd.Prim | None = None

    for entry in entries:
        wrapper = _find_prim_by_name(stage, entry.element_name)
        if wrapper is None:
            continue
        geom_child = _find_geometry_child(wrapper)
        if geom_child is not None and entry.role == "body":
            body_geom_prim = geom_child
        prims_to_remove.append(wrapper.GetPath())

    if body_geom_prim is None:
        _logger.warning(f"Could not find body geometry prim for capsule '{source_name}'")
        return False

    # Determine where to create the capsule — use the body wrapper's parent
    body_wrapper = _find_prim_by_name(stage, body_entry.element_name)
    if body_wrapper is None:
        _logger.warning(f"Could not find body wrapper prim for capsule '{source_name}'")
        return False
    parent_path = body_wrapper.GetPath().GetParentPath()
    capsule_path = parent_path.AppendChild(source_name)

    # Collect transform from the body geometry prim before removal
    body_xf = UsdGeom.Xformable(body_geom_prim)
    xform_ops_data: list[tuple] = []
    if body_xf:
        for op in body_xf.GetOrderedXformOps():
            xform_ops_data.append((op.GetOpType(), op.GetPrecision(), op.GetOpName().replace("xformOp:", ""), op.Get()))

    # Collect purpose
    body_purpose = UsdGeom.Imageable(body_geom_prim).GetPurposeAttr().Get()
    has_collision = body_geom_prim.HasAPI(UsdPhysics.CollisionAPI)

    # Remove old prims
    for path in prims_to_remove:
        stage.RemovePrim(path)

    # Create capsule
    capsule = UsdGeom.Capsule.Define(stage, capsule_path)
    capsule.GetRadiusAttr().Set(float(radius))
    capsule.GetHeightAttr().Set(float(height))
    capsule.GetAxisAttr().Set(axis)

    # Apply transform
    capsule_prim = capsule.GetPrim()
    if xform_ops_data:
        xf = UsdGeom.Xformable(capsule_prim)
        for op_type, precision, suffix, val in xform_ops_data:
            new_op = xf.AddXformOp(op_type, precision, suffix)
            if val is not None:
                new_op.Set(val)

    # Apply purpose
    if body_purpose:
        UsdGeom.Imageable(capsule_prim).GetPurposeAttr().Set(body_purpose)

    # Apply collision API
    if has_collision:
        UsdPhysics.CollisionAPI.Apply(capsule_prim)

    _logger.debug(f"Reconstructed Capsule '{source_name}' at {capsule_path}")
    return True


def _reconstruct_cone(stage: Usd.Stage, link_name: str, source_name: str, entry: SourceGeometryInfo) -> bool:
    """Replace a mesh prim with a UsdGeom.Cone.

    Args:
        stage: USD stage to modify.
        link_name: URDF link name (for logging).
        source_name: Original USD prim name for the reconstructed cone.
        entry: Breadcrumb for this cone instance.

    Returns:
        ``True`` if a cone was created and the wrapper removed successfully.
    """
    params = entry.params
    radius = params.get("radius", 1.0)
    height = params.get("height", 2.0)
    axis = params.get("axis", "Z")

    wrapper = _find_prim_by_name(stage, entry.element_name)
    if wrapper is None:
        _logger.warning(f"Could not find wrapper prim for cone '{source_name}'")
        return False

    geom_child = _find_geometry_child(wrapper)
    if geom_child is None:
        _logger.warning(f"No geometry child under '{wrapper.GetPath()}' for cone '{source_name}'")
        return False

    # Collect data before removal
    geom_xf = UsdGeom.Xformable(geom_child)
    xform_ops_data: list[tuple] = []
    if geom_xf:
        for op in geom_xf.GetOrderedXformOps():
            xform_ops_data.append((op.GetOpType(), op.GetPrecision(), op.GetOpName().replace("xformOp:", ""), op.Get()))

    geom_purpose = UsdGeom.Imageable(geom_child).GetPurposeAttr().Get()
    has_collision = geom_child.HasAPI(UsdPhysics.CollisionAPI)

    parent_path = wrapper.GetPath().GetParentPath()
    cone_path = parent_path.AppendChild(source_name)

    # Remove the wrapper (and its children)
    stage.RemovePrim(wrapper.GetPath())

    # Create cone
    cone = UsdGeom.Cone.Define(stage, cone_path)
    cone.GetRadiusAttr().Set(float(radius))
    cone.GetHeightAttr().Set(float(height))
    cone.GetAxisAttr().Set(axis)

    cone_prim = cone.GetPrim()
    if xform_ops_data:
        xf = UsdGeom.Xformable(cone_prim)
        for op_type, precision, suffix, val in xform_ops_data:
            new_op = xf.AddXformOp(op_type, precision, suffix)
            if val is not None:
                new_op.Set(val)

    if geom_purpose:
        UsdGeom.Imageable(cone_prim).GetPurposeAttr().Set(geom_purpose)

    if has_collision:
        UsdPhysics.CollisionAPI.Apply(cone_prim)

    _logger.debug(f"Reconstructed Cone '{source_name}' at {cone_path}")
    return True
