#!/usr/bin/env python3
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

"""Generate a Robot Schema class diagram from RobotSchema.usda.

Parses the USDA schema definition file and produces an SVG class diagram
(with optional PNG export) showing all schema classes, their attributes,
relationships, and inter-class connections.

The diagram uses orthogonal routing for all connection lines: segments
are strictly vertical or horizontal, joined by quarter-circle arcs whose
radius is controlled by ``_BEND_RADIUS``.  Dashed lines denote USD
relationships; solid lines denote associations.

Fonts
-----
The script embeds NVIDIA Sans (``.ttf`` files) as base64-encoded
``@font-face`` rules inside the SVG so the diagram renders identically
on any system.  If the font files are absent it falls back to
``Arial, sans-serif``.

Output
------
* An SVG file is always written (same path as ``--output`` but with
  ``.svg`` suffix).
* A 2x-scaled PNG is written when ``cairosvg`` is available
  (``pip install cairosvg``).

Usage::

    python generate_schema_diagram.py
    python generate_schema_diagram.py --usda path/to/RobotSchema.usda
    python generate_schema_diagram.py --output path/to/diagram.png
    python generate_schema_diagram.py --fonts-dir path/to/fonts/

All paths default to locations relative to this script inside the
``isaacsim.robot.schema`` extension tree.
"""

from __future__ import annotations

import argparse
import re
import xml.etree.ElementTree as ET
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any

# ---------------------------------------------------------------------------
# USDA parser (unchanged)
# ---------------------------------------------------------------------------

_CLASS_RE = re.compile(r'^class\s+(?:\w+\s+)?"(\w+)"', re.MULTILINE)
_INHERITS_RE = re.compile(r"inherits\s*=\s*</([\w]+)>")
_ATTR_RE = re.compile(
    r"^\s+(uniform\s+)?(\w[\w\[\]]*)\s+(isaac:\S+)\s*=",
    re.MULTILINE,
)
_REL_RE = re.compile(r"^\s+rel\s+(isaac:\S+)", re.MULTILINE)


@dataclass
class SchemaClass:
    """A single USD schema class extracted from a ``.usda`` file.

    Attributes:
        name: Class identifier (e.g. ``"IsaacRobotAPI"``).
        schema_type: Either ``"Applied API Schema"`` or ``"Typed Schema"``,
            derived from the ``inherits`` declaration.
        attributes: List of ``(name, type)`` tuples for every ``isaac:``
            attribute declared in the class body.
        relationships: List of relationship names (``rel isaac:…``).
    """

    name: str
    schema_type: str
    attributes: list[tuple[str, str]] = field(default_factory=list)
    relationships: list[str] = field(default_factory=list)


def parse_usda(path: Path) -> list[SchemaClass]:
    """Parse a ``.usda`` file and return a list of :class:`SchemaClass` instances.

    The parser recognises ``class "ClassName"`` blocks and extracts
    ``inherits``, ``attributes`` (prefixed ``isaac:``), and
    ``relationships`` (``rel isaac:…``).  Internal/global classes
    (names starting with ``_`` or equal to ``GLOBAL``) are skipped.

    Args:
        path: Filesystem path to the ``.usda`` schema definition file.

    Returns:
        Ordered list of schema classes as they appear in the file.
    """
    text = path.read_text()
    classes: list[SchemaClass] = []
    class_starts = list(_CLASS_RE.finditer(text))
    for i, m in enumerate(class_starts):
        name = m.group(1)
        if name == "GLOBAL" or name.startswith("_"):
            continue
        start = m.start()
        end = class_starts[i + 1].start() if i + 1 < len(class_starts) else len(text)
        block = text[start:end]
        inh = _INHERITS_RE.search(block)
        schema_type = "Applied API Schema" if (inh and inh.group(1) == "APISchemaBase") else "Typed Schema"
        brace_open = block.find("{")
        brace_close = block.rfind("}")
        if brace_open == -1 or brace_close == -1:
            continue
        body = block[brace_open + 1 : brace_close]
        attrs = [(am.group(3), am.group(2)) for am in _ATTR_RE.finditer(body)]
        rels = [rm.group(1) for rm in _REL_RE.finditer(body)]
        classes.append(SchemaClass(name, schema_type, attrs, rels))
    return classes


# ---------------------------------------------------------------------------
# Color palette
# ---------------------------------------------------------------------------

_BORDER_COLOR = "#bebec8"
_SHADOW_COLOR = "#b4b4b9"
_TEXT_DARK = "#28282d"
_TEXT_LIGHT = "#ffffff"
_TEXT_ATTR = "#46464f"
_TEXT_TYPE = "#828291"
_TAG_TEXT = "#d2e6c8"
_LINE_COLOR = "#8c9b82"
_ARROW_COLOR = "#64785a"
_LABEL_BG = "#f5f8f2"
_GREEN_PRIMARY = "#76b900"
_GREEN_SECONDARY = "#508000"
_GREEN_TERTIARY = "#3a5e00"

_HEADER_COLORS = {
    "IsaacRobotAPI": _GREEN_PRIMARY,
    "IsaacLinkAPI": _GREEN_SECONDARY,
    "IsaacJointAPI": _GREEN_SECONDARY,
    "IsaacSiteAPI": _GREEN_SECONDARY,
    "IsaacNamedPose": _GREEN_SECONDARY,
    "IsaacAttachmentPointAPI": _GREEN_TERTIARY,
    "IsaacSurfaceGripper": _GREEN_TERTIARY,
}

_SUBLABELS = {
    "IsaacRobotAPI": "Root definition \u2014 applied to the robot\u2019s top-level prim",
    "IsaacLinkAPI": "Applied to rigid bodies (links)",
    "IsaacJointAPI": "Applied to physics joints",
    "IsaacSiteAPI": "Points of interest (tool mounts, sensors, EEF)",
    "IsaacNamedPose": "Stored joint configurations with IK target transforms",
    "IsaacAttachmentPointAPI": "Surface gripper attachment points",
    "IsaacSurfaceGripper": "Surface gripper mechanics (forces, distances)",
}

_REL_TARGETS = {
    "isaac:physics:robotLinks": ["IsaacLinkAPI", "IsaacSiteAPI"],
    "isaac:physics:robotJoints": ["IsaacJointAPI"],
    "isaac:robot:namedPoses": ["IsaacNamedPose"],
    "isaac:attachmentPoints": ["IsaacAttachmentPointAPI"],
}

_ROW_POSITIONS = {
    0: ["IsaacRobotAPI"],
    1: ["IsaacLinkAPI", "IsaacSiteAPI", "IsaacJointAPI", "IsaacNamedPose"],
    2: ["IsaacSurfaceGripper", "IsaacAttachmentPointAPI"],
}

# ---------------------------------------------------------------------------
# Layout constants
# ---------------------------------------------------------------------------

_HEADER_H = 34
_HEADER_TOP_PAD = 8
_ATTR_LINE_H = 19
_REL_LINE_H = 19
_SECTION_LINE_H = 22
_PADDING_TOP = 22
_PADDING_BOTTOM = 50
_CORNER_R = 12
_BEND_RADIUS = 30
_LINE_W = 4
_DASH = "8,5"
_ARROW_SIZE = 10


# ---------------------------------------------------------------------------
# BoxLayout
# ---------------------------------------------------------------------------


@dataclass
class BoxLayout:
    """Resolved position and size of a single schema box in the diagram.

    Attributes:
        x: Left edge in SVG user units.
        y: Top edge in SVG user units.
        w: Box width.
        h: Box height (computed from member count).
        header_color: Fill colour for the title bar.
        schema: The :class:`SchemaClass` this box represents.
        sublabel: Optional italicised description rendered at the bottom
            of the box.
    """

    x: int
    y: int
    w: int
    h: int
    header_color: str
    schema: SchemaClass
    sublabel: str = ""

    @property
    def cx(self) -> int:
        """Horizontal centre of the box."""
        return self.x + self.w // 2

    @property
    def top(self) -> int:
        """Top edge (alias for *y*)."""
        return self.y

    @property
    def bottom(self) -> int:
        """Bottom edge (*y + h*)."""
        return self.y + self.h


def _box_height(sc: SchemaClass) -> int:
    """Return the total pixel height of the box for *sc*.

    Accounts for the header bar, section titles (``Relationships``,
    ``Attributes``), individual member lines, and top/bottom padding.

    Args:
        sc: Schema class to measure.

    Returns:
        Total box height in SVG user units.
    """
    ns = (1 if sc.relationships else 0) + (1 if sc.attributes else 0)
    body = ns * _SECTION_LINE_H + len(sc.attributes) * _ATTR_LINE_H + len(sc.relationships) * _REL_LINE_H
    return _HEADER_H + _PADDING_TOP + max(body, 8) + _PADDING_BOTTOM


def _layout(classes: list[SchemaClass], cw: int) -> list[BoxLayout]:
    """Arrange schema classes into a multi-row grid of :class:`BoxLayout` boxes.

    Row membership and ordering are controlled by ``_ROW_POSITIONS``.
    Widths, horizontal gaps, and vertical gaps are computed so that each
    row is centred within the canvas of width *cw*.

    Args:
        classes: Schema classes returned by :func:`parse_usda`.
        cw: Total canvas width (SVG user units).

    Returns:
        List of positioned boxes in row-major order.
    """
    nc = {c.name: c for c in classes}
    boxes: dict[str, BoxLayout] = {}
    mx = 16
    gap = 70

    r0 = [n for n in _ROW_POSITIONS[0] if n in nc]
    if r0:
        c = nc[r0[0]]
        bw = min(680, cw - 2 * mx)
        bh = _box_height(c)
        boxes[c.name] = BoxLayout(
            (cw - bw) // 2, 8, bw, bh, _HEADER_COLORS.get(c.name, _GREEN_PRIMARY), c, _SUBLABELS.get(c.name, "")
        )

    r0b = max((b.bottom for b in boxes.values()), default=30)
    r1y = r0b + gap
    r1 = [n for n in _ROW_POSITIONS[1] if n in nc]
    if r1:
        n = len(r1)
        bw = min(340, (cw - mx * 2 - 30 * (n - 1)) // n)
        tw = n * bw + (n - 1) * 30
        sx = (cw - tw) // 2
        for i, nm in enumerate(r1):
            c = nc[nm]
            bh = _box_height(c)
            boxes[nm] = BoxLayout(
                sx + i * (bw + 30), r1y, bw, bh, _HEADER_COLORS.get(nm, _GREEN_SECONDARY), c, _SUBLABELS.get(nm, "")
            )

    r1b = max((b.bottom for b in boxes.values() if b.schema.name in r1), default=r1y)
    r2y = r1b + gap
    r2 = [n for n in _ROW_POSITIONS[2] if n in nc]
    if r2:
        n = len(r2)
        g2 = 120
        bw = min(380, (cw - mx * 2 - g2 * (n - 1)) // n)
        tw = n * bw + (n - 1) * g2
        sx = (cw - tw) // 2
        for i, nm in enumerate(r2):
            c = nc[nm]
            bh = _box_height(c)
            boxes[nm] = BoxLayout(
                sx + i * (bw + g2), r2y, bw, bh, _HEADER_COLORS.get(nm, _GREEN_TERTIARY), c, _SUBLABELS.get(nm, "")
            )

    order = _ROW_POSITIONS[0] + _ROW_POSITIONS[1] + _ROW_POSITIONS[2]
    return [boxes[n] for n in order if n in boxes]


# ---------------------------------------------------------------------------
# SVG helpers
# ---------------------------------------------------------------------------


def _rgb(hex_color: str) -> str:
    """Pass through colour values.

    Args:
        hex_color: Colour value to return.

    Returns:
        The input colour value.
    """
    return hex_color


def _font_family(fonts_dir: Path) -> str:
    """Return CSS ``@font-face`` rules that base64-embed NVIDIA Sans ``.ttf`` files.

    Each font variant (Bold, Regular, Light, Italic, Medium) is encoded
    inline so the SVG renders identically on any system.  Missing files
    are silently skipped; the diagram then falls back to ``Arial``.

    Args:
        fonts_dir: Directory containing font files.

    Returns:
        CSS font-face rules for available fonts.
    """
    faces = []
    mapping = [
        ("NVIDIASans_Bd.ttf", "NVIDIASans", "bold", "normal"),
        ("NVIDIASans_Rg.ttf", "NVIDIASans", "normal", "normal"),
        ("NVIDIASans_Lt.ttf", "NVIDIASans", "300", "normal"),
        ("NVIDIASans_It.ttf", "NVIDIASans", "normal", "italic"),
        ("NVIDIASans_Md.ttf", "NVIDIASans", "500", "normal"),
    ]
    for fname, family, weight, style in mapping:
        p = fonts_dir / fname
        if p.exists():
            import base64

            b64 = base64.b64encode(p.read_bytes()).decode()
            faces.append(
                f"@font-face {{ font-family: '{family}'; font-weight: {weight}; "
                f"font-style: {style}; src: url('data:font/truetype;base64,{b64}') format('truetype'); }}"
            )
    return "\n".join(faces)


def _ortho_path_vhv(x1: Any, y1: Any, x2: Any, y2: Any, r: Any = _BEND_RADIUS) -> str:
    """Build an SVG path string: vertical, horizontal, vertical with arcs.

    The path exits ``(x1, y1)`` vertically, turns with a quarter-circle
    arc of radius *r*, runs horizontally to the target column, turns
    again, and arrives at ``(x2, y2)`` vertically.

    Falls back to a straight line when the displacement is too small
    for the requested bend radius.

    Args:
        x1: Starting x coordinate.
        y1: Starting y coordinate.
        x2: Ending x coordinate.
        y2: Ending y coordinate.
        r: Bend radius.

    Returns:
        SVG path data string.
    """
    dx = x2 - x1
    dy = y2 - y1
    if abs(dx) < 2:
        return f"M{x1},{y1} L{x2},{y2}"
    if abs(dy) < 2:
        return f"M{x1},{y1} L{x2},{y2}"
    r = min(r, abs(dx) // 2, abs(dy) // 2, 30)
    if r < 2:
        r = 2
    mid_y = (y1 + y2) / 2
    hs = 1 if dx > 0 else -1
    vs1 = 1 if mid_y > y1 else -1
    vs2 = 1 if y2 > mid_y else -1

    # First vertical segment
    v1_end = mid_y - vs1 * r
    # First arc: turn from vertical to horizontal
    arc1_ey = mid_y
    arc1_ex = x1 + hs * r
    sweep1 = 0 if (vs1 > 0) == (hs > 0) else 1
    # Horizontal segment
    h_end = x2 - hs * r
    # Second arc: turn from horizontal to vertical
    arc2_ex = x2
    arc2_ey = mid_y + vs2 * r
    sweep2 = 0 if (hs > 0) != (vs2 > 0) else 1
    # Second vertical segment to target

    return (
        f"M{x1},{y1} "
        f"L{x1},{v1_end} "
        f"A{r},{r} 0 0,{sweep1} {arc1_ex},{arc1_ey} "
        f"L{h_end},{mid_y} "
        f"A{r},{r} 0 0,{sweep2} {arc2_ex},{arc2_ey} "
        f"L{x2},{y2}"
    )


def _ortho_path_hvh(x1: Any, y1: Any, x2: Any, y2: Any, r: Any = _BEND_RADIUS) -> str:
    """SVG path: horizontal → vertical → horizontal with two quarter-arc bends.

    Exits horizontally from (x1,y1), transitions vertically at the
    midpoint X, and arrives horizontally at (x2,y2).  Both endpoints
    meet their boxes orthogonally.

    Args:
        x1: Starting x coordinate.
        y1: Starting y coordinate.
        x2: Ending x coordinate.
        y2: Ending y coordinate.
        r: Bend radius.

    Returns:
        SVG path data string.
    """
    dx = x2 - x1
    dy = y2 - y1
    if abs(dy) < 2:
        return f"M{x1},{y1} L{x2},{y2}"
    if abs(dx) < 2:
        return f"M{x1},{y1} L{x2},{y2}"
    r = min(r, abs(dx) // 4, abs(dy) // 2)
    if r < 2:
        r = 2
    mid_x = (x1 + x2) / 2
    hs1 = 1 if dx > 0 else -1  # horizontal direction out of source
    vs = 1 if dy > 0 else -1  # vertical direction
    hs2 = hs1  # horizontal direction into target (same)

    # First horizontal leg: (x1,y1) → (mid_x - r, y1)
    h1_end = mid_x - hs1 * r
    # First arc: horizontal → vertical
    arc1_ex = mid_x
    arc1_ey = y1 + vs * r
    sweep1 = 1 if (hs1 > 0) == (vs > 0) else 0

    # Vertical leg: (mid_x, y1+r) → (mid_x, y2-r)
    v_end = y2 - vs * r

    # Second arc: vertical → horizontal
    arc2_ex = mid_x + hs2 * r
    arc2_ey = y2
    sweep2 = 1 if (vs > 0) != (hs2 > 0) else 0

    # Second horizontal leg: → (x2, y2)
    return (
        f"M{x1},{y1} "
        f"L{h1_end},{y1} "
        f"A{r},{r} 0 0,{sweep1} {arc1_ex},{arc1_ey} "
        f"L{mid_x},{v_end} "
        f"A{r},{r} 0 0,{sweep2} {arc2_ex},{arc2_ey} "
        f"L{x2},{y2}"
    )


def _arrow_marker(svg: ET.Element, marker_id: str, color: str) -> None:
    """Append an SVG ``<marker>`` arrowhead definition to *svg*'s ``<defs>``.

    Args:
        svg: Root SVG element that receives the marker definition.
        marker_id: Marker identifier.
        color: Marker fill colour.
    """
    marker = ET.SubElement(svg.find("{http://www.w3.org/2000/svg}defs") or ET.SubElement(svg, "defs"), "marker")
    marker.set("id", marker_id)
    marker.set("markerWidth", str(_ARROW_SIZE))
    marker.set("markerHeight", str(_ARROW_SIZE))
    marker.set("refX", str(_ARROW_SIZE))
    marker.set("refY", str(_ARROW_SIZE // 2))
    marker.set("orient", "auto")
    marker.set("markerUnits", "userSpaceOnUse")
    poly = ET.SubElement(marker, "polygon")
    s = _ARROW_SIZE
    poly.set("points", f"{s},{s // 2} 0,0 0,{s}")
    poly.set("fill", color)


# ---------------------------------------------------------------------------
# SVG box rendering
# ---------------------------------------------------------------------------


def _add_box(g: ET.Element, box: BoxLayout) -> None:
    """Render a single schema box into SVG group *g*.

    Draws the shadow, body rectangle, coloured header bar, class name,
    schema-type tag, relationship and attribute listings, and an
    optional sublabel at the bottom of the box.

    Args:
        g: SVG group element that receives the box elements.
        box: Positioned box to render.
    """
    x, y, w, h = box.x, box.y, box.w, box.h
    sc = box.schema

    # Shadow
    ET.SubElement(
        g,
        "rect",
        x=str(x + 3),
        y=str(y + 3),
        width=str(w),
        height=str(h),
        rx=str(_CORNER_R),
        fill=_SHADOW_COLOR,
        opacity="0.25",
    )
    # Body
    ET.SubElement(
        g,
        "rect",
        x=str(x),
        y=str(y),
        width=str(w),
        height=str(h),
        rx=str(_CORNER_R),
        fill="white",
        stroke=_BORDER_COLOR,
    )
    # Header background — use clipPath for top-rounded, bottom-straight
    clip_id = f"clip_{sc.name}"
    defs = g.find("defs")
    if defs is None:
        defs = ET.SubElement(g, "defs")
    cp = ET.SubElement(defs, "clipPath", id=clip_id)
    ET.SubElement(cp, "rect", x=str(x), y=str(y), width=str(w), height=str(_HEADER_H), rx="0")
    # Header rect (full rounded, clipped to header height)
    ET.SubElement(
        g,
        "rect",
        x=str(x),
        y=str(y),
        width=str(w),
        height=str(_HEADER_H + _CORNER_R),
        rx=str(_CORNER_R),
        fill=box.header_color,
    )
    ET.SubElement(g, "rect", x=str(x), y=str(y + _HEADER_H), width=str(w), height=str(_CORNER_R), fill=box.header_color)

    # Title text (centered vertically in header with top bias)
    title_y = y + _HEADER_H // 2 + _HEADER_TOP_PAD // 2
    t = ET.SubElement(g, "text", x=str(x + 14), y=str(title_y))
    t.set("font-family", "NVIDIASans, Arial, sans-serif")
    t.set("font-weight", "bold")
    t.set("font-size", "17")
    t.set("fill", _TEXT_LIGHT)
    t.set("dominant-baseline", "central")
    t.text = sc.name

    # Tag text
    tag = ET.SubElement(g, "text", x=str(x + w - 14), y=str(title_y))
    tag.set("font-family", "NVIDIASans, Arial, sans-serif")
    tag.set("font-style", "italic")
    tag.set("font-size", "11")
    tag.set("fill", _TAG_TEXT)
    tag.set("text-anchor", "end")
    tag.set("dominant-baseline", "central")
    tag.text = sc.schema_type

    cy = y + _HEADER_H + _PADDING_TOP

    if sc.relationships:
        sec = ET.SubElement(g, "text", x=str(x + 14), y=str(cy + 12))
        sec.set("font-family", "NVIDIASans, Arial, sans-serif")
        sec.set("font-weight", "bold")
        sec.set("font-size", "13")
        sec.set("fill", _TEXT_DARK)
        sec.text = "Relationships"
        cy += _SECTION_LINE_H
        for rn in sc.relationships:
            tn = ET.SubElement(g, "text", x=str(x + 20), y=str(cy + 12))
            tn.set("font-family", "NVIDIASans, Arial, sans-serif")
            tn.set("font-size", "13")
            tn.set("fill", _TEXT_ATTR)
            tn.text = rn
            tt = ET.SubElement(g, "text", x=str(x + w - 14), y=str(cy + 12))
            tt.set("font-family", "NVIDIASans, Arial, sans-serif")
            tt.set("font-size", "12")
            tt.set("font-weight", "300")
            tt.set("fill", _TEXT_TYPE)
            tt.set("text-anchor", "end")
            tt.text = "rel"
            cy += _REL_LINE_H

    if sc.attributes:
        sec = ET.SubElement(g, "text", x=str(x + 14), y=str(cy + 12))
        sec.set("font-family", "NVIDIASans, Arial, sans-serif")
        sec.set("font-weight", "bold")
        sec.set("font-size", "13")
        sec.set("fill", _TEXT_DARK)
        sec.text = "Attributes"
        cy += _SECTION_LINE_H
        for an, at in sc.attributes:
            tn = ET.SubElement(g, "text", x=str(x + 20), y=str(cy + 12))
            tn.set("font-family", "NVIDIASans, Arial, sans-serif")
            tn.set("font-size", "13")
            tn.set("fill", _TEXT_ATTR)
            tn.text = an
            tt = ET.SubElement(g, "text", x=str(x + w - 14), y=str(cy + 12))
            tt.set("font-family", "NVIDIASans, Arial, sans-serif")
            tt.set("font-size", "12")
            tt.set("font-weight", "300")
            tt.set("fill", _TEXT_TYPE)
            tt.set("text-anchor", "end")
            tt.text = at
            cy += _ATTR_LINE_H

    if box.sublabel:
        sl = ET.SubElement(g, "text", x=str(x + 14), y=str(y + h - 8))
        sl.set("font-family", "NVIDIASans, Arial, sans-serif")
        sl.set("font-style", "italic")
        sl.set("font-size", "11")
        sl.set("fill", _TEXT_TYPE)
        sl.text = box.sublabel


# ---------------------------------------------------------------------------
# Main generation
# ---------------------------------------------------------------------------


def generate_diagram(usda_path: Path, output_path: Path, fonts_dir: Path) -> None:
    """Parse the schema and write SVG (and optionally PNG) class diagram.

    This is the top-level entry point that orchestrates:
    1. Schema parsing via :func:`parse_usda`.
    2. Box layout via :func:`_layout`.
    3. SVG construction — fonts, markers, connection lines, boxes, labels,
       and footer.
    4. Writing the ``.svg`` file alongside *output_path*.
    5. PNG export at 2x scale when ``cairosvg`` is installed.

    Args:
        usda_path: Path to ``RobotSchema.usda``.
        output_path: Destination for the PNG file (the SVG is written to
            the same directory with a ``.svg`` suffix).
        fonts_dir: Directory containing ``NVIDIASans_*.ttf`` font files.

    Raises:
        RuntimeError: If no schema classes are found in *usda_path*.
    """
    classes = parse_usda(usda_path)
    if not classes:
        raise RuntimeError(f"No schema classes found in {usda_path}")

    layout_w = 1400
    boxes = _layout(classes, layout_w)
    name_to_box = {b.schema.name: b for b in boxes}

    # Compute tight viewBox from actual content bounds
    pad = 6
    shadow = 3
    min_x = min(b.x for b in boxes) - pad
    min_y = min(b.y for b in boxes) - pad
    max_x = max(b.x + b.w + shadow for b in boxes) + pad
    max_y = max(b.bottom + shadow for b in boxes) + 30  # room for footer text
    vb_w = max_x - min_x
    vb_h = max_y - min_y

    ns = "http://www.w3.org/2000/svg"
    ET.register_namespace("", ns)
    svg = ET.Element(
        "svg", xmlns=ns, width=str(int(vb_w)), height=str(int(vb_h)), viewBox=f"{min_x} {min_y} {vb_w} {vb_h}"
    )

    # Embed fonts
    style_el = ET.SubElement(svg, "style")
    style_el.text = _font_family(fonts_dir)

    defs = ET.SubElement(svg, "defs")
    # Arrow markers
    for mid, col in [("arrow-rel", _ARROW_COLOR), ("arrow-assoc", _ARROW_COLOR)]:
        marker = ET.SubElement(
            defs,
            "marker",
            id=mid,
            markerWidth=str(_ARROW_SIZE),
            markerHeight=str(_ARROW_SIZE),
            refX=str(_ARROW_SIZE - 1),
            refY=str(_ARROW_SIZE // 2),
            orient="auto",
            markerUnits="userSpaceOnUse",
        )
        ET.SubElement(marker, "polygon", points=f"{_ARROW_SIZE},{_ARROW_SIZE // 2} 0,0 0,{_ARROW_SIZE}", fill=col)

    # --- Connection lines (under boxes) ---
    conn_g = ET.SubElement(svg, "g", id="connections")
    gap_labels: list[tuple[float, float, float, str]] = []

    robot = name_to_box.get("IsaacRobotAPI")
    if robot:
        all_tgts: list[str] = []
        for rn in robot.schema.relationships:
            all_tgts.extend(_REL_TARGETS.get(rn, []))
        n_tgts = len(all_tgts)
        for rel_name in robot.schema.relationships:
            for tgt_name in _REL_TARGETS.get(rel_name, []):
                tb = name_to_box.get(tgt_name)
                if not tb:
                    continue
                idx = all_tgts.index(tgt_name)
                frac = (idx + 1) / (n_tgts + 1)
                sx = robot.x + int(robot.w * frac)
                sy = robot.bottom
                ex = tb.cx
                ey = tb.top
                d = _ortho_path_vhv(sx, sy, ex, ey)
                p = ET.SubElement(conn_g, "path", d=d, fill="none", stroke=_LINE_COLOR)
                p.set("stroke-width", str(_LINE_W))
                p.set("stroke-dasharray", _DASH)
                p.set("marker-end", "url(#arrow-rel)")
                gap_labels.append(((sx + ex) / 2, sy, ey, rel_name.split(":")[-1]))

    gripper = name_to_box.get("IsaacSurfaceGripper")
    attach = name_to_box.get("IsaacAttachmentPointAPI")
    if gripper and attach:
        for rel_name in gripper.schema.relationships:
            for tgt_name in _REL_TARGETS.get(rel_name, []):
                tb = name_to_box.get(tgt_name)
                if not tb:
                    continue
                sx = gripper.x + gripper.w
                sy = gripper.y + gripper.h // 2
                ex = tb.x
                ey = tb.y + tb.h // 2
                d = _ortho_path_hvh(sx, sy, ex, ey)
                p = ET.SubElement(conn_g, "path", d=d, fill="none", stroke=_LINE_COLOR)
                p.set("stroke-width", str(_LINE_W))
                p.set("stroke-dasharray", _DASH)
                p.set("marker-end", "url(#arrow-rel)")
                mid_x = (sx + ex) / 2
                gap_labels.append((mid_x, min(sy, ey) - 20, min(sy, ey), rel_name.split(":")[-1]))

    link_box = name_to_box.get("IsaacLinkAPI")
    if link_box and gripper:
        sx = link_box.cx
        sy = link_box.bottom
        ex = gripper.cx
        ey = gripper.top
        d = _ortho_path_vhv(sx, sy, ex, ey)
        p = ET.SubElement(conn_g, "path", d=d, fill="none", stroke=_LINE_COLOR)
        p.set("stroke-width", str(_LINE_W))
        p.set("marker-end", "url(#arrow-assoc)")

    # --- Boxes ---
    box_g = ET.SubElement(svg, "g", id="boxes")
    for b in boxes:
        _add_box(box_g, b)

    # --- Relationship labels (in gaps, above boxes) ---
    label_g = ET.SubElement(svg, "g", id="labels")
    for lx, gt, gb, txt in gap_labels:
        ly = (gt + gb) / 2
        tw_est = len(txt) * 6.5
        ET.SubElement(
            label_g,
            "rect",
            x=str(lx - tw_est / 2 - 5),
            y=str(ly - 9),
            width=str(tw_est + 10),
            height="18",
            rx="3",
            fill=_LABEL_BG,
            stroke=_BORDER_COLOR,
        )
        lt = ET.SubElement(label_g, "text", x=str(lx), y=str(ly + 4))
        lt.set("font-family", "NVIDIASans, Arial, sans-serif")
        lt.set("font-size", "11")
        lt.set("fill", _ARROW_COLOR)
        lt.set("text-anchor", "middle")
        lt.text = txt

    # --- Footer ---
    footer_x = (min_x + max_x) / 2
    footer_y = max_y - 8
    ft = ET.SubElement(svg, "text", x=str(footer_x), y=str(footer_y))
    ft.set("font-family", "NVIDIASans, Arial, sans-serif")
    ft.set("font-weight", "500")
    ft.set("font-size", "16")
    ft.set("fill", _TEXT_DARK)
    ft.set("text-anchor", "middle")
    ft.text = "Isaac Sim Robot Schema"

    # --- Write SVG ---
    svg_path = output_path.with_suffix(".svg")
    tree = ET.ElementTree(svg)
    ET.indent(tree, space="  ")
    svg_path.parent.mkdir(parents=True, exist_ok=True)
    tree.write(str(svg_path), xml_declaration=True, encoding="utf-8")
    print(f"Saved SVG to {svg_path}")

    # --- Convert to PNG ---
    try:
        import cairosvg

        cairosvg.svg2png(url=str(svg_path), write_to=str(output_path), scale=2)
        print(f"Saved PNG to {output_path}")
        svg_path.unlink(missing_ok=True)
        print(f"Removed intermediate SVG {svg_path}")
    except ImportError:
        print("cairosvg not available — PNG export skipped. Install with: pip install cairosvg")


def main() -> None:
    """CLI entry point — resolve default paths and invoke :func:`generate_diagram`."""
    script_dir = Path(__file__).resolve().parent
    ext_dir = script_dir.parent
    repo_root = ext_dir.parent.parent.parent

    default_usda = repo_root / "source" / "extensions" / "isaacsim.robot.schema" / "robot_schema" / "RobotSchema.usda"
    default_output = repo_root / "docs" / "isaacsim" / "images" / "isim_6.0_base_ref_gui_robot_schema_description.png"
    default_fonts = ext_dir / "data" / "fonts"

    parser = argparse.ArgumentParser(description="Generate Robot Schema class diagram from USDA.")
    parser.add_argument("--usda", type=Path, default=default_usda, help="Path to RobotSchema.usda")
    parser.add_argument("--output", type=Path, default=default_output, help="Output PNG path")
    parser.add_argument("--fonts-dir", type=Path, default=default_fonts, help="Directory with NVIDIA Sans .ttf files")
    args = parser.parse_args()

    generate_diagram(args.usda, args.output, args.fonts_dir)


if __name__ == "__main__":
    main()
