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
"""URDF XML serialization using stdlib xml.etree.ElementTree."""

from __future__ import annotations

import xml.dom.minidom
import xml.etree.ElementTree as ET

from .geometry_reader import GeometryData
from .inertia_utils import InertiaData
from .joint_reader import JointData, LoopJointData
from .link_reader import CollisionData, LinkData, VisualData
from .material_reader import MaterialData
from .transform_utils import is_origin_identity


def write_urdf(
    robot_name: str,
    links: list[LinkData],
    joints: list[JointData],
    materials: list[MaterialData],
    output_path: str,
    loop_joints: list[LoopJointData] | None = None,
) -> None:
    """Write a complete URDF XML file.

    Args:
        robot_name: The robot name attribute.
        links: All link data.
        joints: All joint data.
        materials: All material data (for global definitions).
        output_path: File path to write.
        loop_joints: Optional loop joint data for closed kinematic chains.

    """
    robot = ET.Element("robot", name=robot_name)

    non_mesh_mat_names = set()
    for link in links:
        for visual in link.visuals:
            if visual.material_name and visual.geometry and visual.geometry.geom_type != "mesh":
                non_mesh_mat_names.add(visual.material_name)

    for mat in materials:
        if mat.name in non_mesh_mat_names:
            _write_material(robot, mat)

    for link in links:
        _write_link(robot, link)

    for joint in joints:
        _write_joint(robot, joint)

    if loop_joints:
        for lj in loop_joints:
            _write_loop_joint(robot, lj)

    rough_xml = ET.tostring(robot, encoding="unicode", xml_declaration=False)
    dom = xml.dom.minidom.parseString(rough_xml)
    pretty = dom.toprettyxml(indent="  ", encoding="utf-8")

    with open(output_path, "wb") as f:
        f.write(pretty)


def _write_material(parent: ET.Element, mat: MaterialData) -> None:
    """Write a global <material> element.

    Args:
        parent: Parent XML element.
        mat: Matrix to convert or material data to write.
    """
    mat_elem = ET.SubElement(parent, "material", name=mat.name)

    if mat.color_rgba is not None:
        rgba_str = " ".join(f"{v:.6f}" for v in mat.color_rgba)
        ET.SubElement(mat_elem, "color", rgba=rgba_str)

    if mat.texture_filename is not None:
        ET.SubElement(mat_elem, "texture", filename=mat.texture_filename)


def _write_link(parent: ET.Element, link: LinkData) -> None:
    """Write a <link> element with inertial, visual, and collision children.

    Args:
        parent: Parent XML element.
        link: Link data to write.
    """
    link_elem = ET.SubElement(parent, "link", name=link.name)

    if link.inertial:
        _write_inertial(link_elem, link.inertial)

    for visual in link.visuals:
        _write_visual(link_elem, visual)

    for collision in link.collisions:
        _write_collision(link_elem, collision)


def _write_inertial(parent: ET.Element, inertial: InertiaData) -> None:
    """Write an <inertial> element.

    Args:
        parent: Parent XML element.
        inertial: Inertial data to write.
    """
    inertial_elem = ET.SubElement(parent, "inertial")

    if not is_origin_identity(inertial.origin_xyz, inertial.origin_rpy):
        _write_origin(inertial_elem, inertial.origin_xyz, inertial.origin_rpy)

    ET.SubElement(inertial_elem, "mass", value=_fmt(inertial.mass))

    ET.SubElement(
        inertial_elem,
        "inertia",
        ixx=_fmt(inertial.ixx),
        ixy=_fmt(inertial.ixy),
        ixz=_fmt(inertial.ixz),
        iyy=_fmt(inertial.iyy),
        iyz=_fmt(inertial.iyz),
        izz=_fmt(inertial.izz),
    )


def _write_visual(parent: ET.Element, visual: VisualData) -> None:
    """Write a <visual> element.

    Args:
        parent: Parent XML element.
        visual: Visual data to write.
    """
    attrs = {}
    if visual.name:
        attrs["name"] = visual.name
    vis_elem = ET.SubElement(parent, "visual", **attrs)

    if not is_origin_identity(visual.origin_xyz, visual.origin_rpy):
        _write_origin(vis_elem, visual.origin_xyz, visual.origin_rpy)

    _write_geometry(vis_elem, visual.geometry)

    if visual.material_name and visual.geometry and visual.geometry.geom_type != "mesh":
        ET.SubElement(vis_elem, "material", name=visual.material_name)

    _write_source_geometry_breadcrumb(vis_elem, visual.geometry)


def _write_collision(parent: ET.Element, collision: CollisionData) -> None:
    """Write a <collision> element.

    Args:
        parent: Parent XML element.
        collision: Value to use.
    """
    attrs = {}
    if collision.name:
        attrs["name"] = collision.name
    col_elem = ET.SubElement(parent, "collision", **attrs)

    if not is_origin_identity(collision.origin_xyz, collision.origin_rpy):
        _write_origin(col_elem, collision.origin_xyz, collision.origin_rpy)

    _write_geometry(col_elem, collision.geometry)
    _write_source_geometry_breadcrumb(col_elem, collision.geometry)


def _write_geometry(parent: ET.Element, geom: GeometryData | None) -> None:
    """Write a <geometry> element.

    Args:
        parent: Parent XML element.
        geom: Geometry data to write.
    """
    if geom is None:
        return

    geom_elem = ET.SubElement(parent, "geometry")

    if geom.geom_type == "box":
        size_str = " ".join(_fmt(v) for v in geom.box_size)
        ET.SubElement(geom_elem, "box", size=size_str)

    elif geom.geom_type == "sphere":
        ET.SubElement(geom_elem, "sphere", radius=_fmt(geom.sphere_radius))

    elif geom.geom_type == "cylinder":
        ET.SubElement(
            geom_elem,
            "cylinder",
            radius=_fmt(geom.cylinder_radius),
            length=_fmt(geom.cylinder_length),
        )

    elif geom.geom_type == "mesh":
        mesh_attrs = {"filename": geom.mesh_filename or ""}
        if geom.mesh_scale is not None:
            mesh_attrs["scale"] = " ".join(_fmt(v) for v in geom.mesh_scale)
        ET.SubElement(geom_elem, "mesh", **mesh_attrs)


def _write_source_geometry_breadcrumb(parent_elem: ET.Element, geom: GeometryData | None) -> None:
    """Append an XML comment with original geometry metadata for round-trip.

    The importer can parse these ``isaac:source_geometry`` comments to
    reconstruct the original USD primitive type (e.g. Capsule, Cone).

    Args:
        parent_elem: Parent XML element.
        geom: Geometry data to write.
    """
    if geom is None or geom.original_type is None:
        return

    import json

    meta: dict = {"type": geom.original_type}
    if geom.original_params:
        meta.update(geom.original_params)
    if geom.name_suffix:
        meta["role"] = geom.name_suffix.lstrip("_")
    parent_elem.append(ET.Comment(f" isaac:source_geometry {json.dumps(meta, sort_keys=True)} "))


def _write_source_joint_breadcrumb(parent_elem: ET.Element, joint: JointData) -> None:
    """Append an XML comment with original USD joint metadata for round-trip.

    The importer can parse these ``isaac:source_joint`` comments to
    reconstruct multi-DOF USD joint types (SphericalJoint, D6Joint)
    from their chained single-DOF URDF representation.

    Args:
        parent_elem: Parent XML element.
        joint: Joint data or USD physics joint to read.
    """
    if joint.original_usd_type is None:
        return

    import json

    meta: dict = {"type": joint.original_usd_type}
    if joint.original_params:
        meta.update(joint.original_params)
    parent_elem.append(ET.Comment(f" isaac:source_joint {json.dumps(meta, sort_keys=True)} "))


def _write_joint(parent: ET.Element, joint: JointData) -> None:
    """Write a <joint> element.

    Args:
        parent: Parent XML element.
        joint: Joint data or USD physics joint to read.
    """
    joint_elem = ET.SubElement(parent, "joint", name=joint.name, type=joint.joint_type)

    if not is_origin_identity(joint.origin_xyz, joint.origin_rpy):
        _write_origin(joint_elem, joint.origin_xyz, joint.origin_rpy)

    ET.SubElement(joint_elem, "parent", link=joint.parent_link)
    ET.SubElement(joint_elem, "child", link=joint.child_link)

    if joint.joint_type in ("revolute", "continuous", "prismatic", "planar"):
        axis_str = " ".join(_fmt(v) for v in joint.axis)
        ET.SubElement(joint_elem, "axis", xyz=axis_str)

    if joint.joint_type in ("revolute", "prismatic"):
        limit_attrs = {}
        if joint.limit_lower is not None:
            limit_attrs["lower"] = _fmt(joint.limit_lower)
        if joint.limit_upper is not None:
            limit_attrs["upper"] = _fmt(joint.limit_upper)
        if joint.limit_effort is not None:
            limit_attrs["effort"] = _fmt(joint.limit_effort)
        if joint.limit_velocity is not None:
            limit_attrs["velocity"] = _fmt(joint.limit_velocity)
        if limit_attrs:
            ET.SubElement(joint_elem, "limit", **limit_attrs)

    has_dynamics = joint.dynamics_damping is not None or joint.dynamics_friction is not None
    if has_dynamics:
        dyn_attrs = {}
        if joint.dynamics_damping is not None:
            dyn_attrs["damping"] = _fmt(joint.dynamics_damping)
        if joint.dynamics_friction is not None:
            dyn_attrs["friction"] = _fmt(joint.dynamics_friction)
        ET.SubElement(joint_elem, "dynamics", **dyn_attrs)

    has_cal = (
        joint.calibration_rising is not None
        or joint.calibration_falling is not None
        or joint.calibration_reference_position is not None
    )
    if has_cal:
        cal_attrs = {}
        if joint.calibration_rising is not None:
            cal_attrs["rising"] = _fmt(joint.calibration_rising)
        if joint.calibration_falling is not None:
            cal_attrs["falling"] = _fmt(joint.calibration_falling)
        if joint.calibration_reference_position is not None:
            cal_attrs["reference_position"] = _fmt(joint.calibration_reference_position)
        ET.SubElement(joint_elem, "calibration", **cal_attrs)

    has_safety = (
        joint.safety_k_velocity is not None
        or joint.safety_k_position is not None
        or joint.safety_soft_lower is not None
        or joint.safety_soft_upper is not None
    )
    if has_safety:
        safe_attrs = {}
        if joint.safety_k_velocity is not None:
            safe_attrs["k_velocity"] = _fmt(joint.safety_k_velocity)
        if joint.safety_k_position is not None:
            safe_attrs["k_position"] = _fmt(joint.safety_k_position)
        if joint.safety_soft_lower is not None:
            safe_attrs["soft_lower_limit"] = _fmt(joint.safety_soft_lower)
        if joint.safety_soft_upper is not None:
            safe_attrs["soft_upper_limit"] = _fmt(joint.safety_soft_upper)
        ET.SubElement(joint_elem, "safety_controller", **safe_attrs)

    if joint.mimic_joint is not None:
        mimic_attrs = {"joint": joint.mimic_joint}
        if joint.mimic_multiplier is not None:
            mimic_attrs["multiplier"] = _fmt(joint.mimic_multiplier)
        if joint.mimic_offset is not None:
            mimic_attrs["offset"] = _fmt(joint.mimic_offset)
        ET.SubElement(joint_elem, "mimic", **mimic_attrs)

    _write_source_joint_breadcrumb(joint_elem, joint)
    _write_source_drive_breadcrumb(joint_elem, joint)


def _write_source_drive_breadcrumb(parent_elem: ET.Element, joint: JointData) -> None:
    """Append an XML comment with DriveAPI / MjcActuator / armature data.

    The importer can parse these ``isaac:source_drive`` comments to
    restore actuation parameters that have no URDF equivalent.

    Args:
        parent_elem: Parent XML element.
        joint: Joint data or USD physics joint to read.
    """
    if joint.source_drive is None:
        return

    import json

    parent_elem.append(ET.Comment(f" isaac:source_drive {json.dumps(joint.source_drive, sort_keys=True)} "))


def _write_origin(
    parent: ET.Element,
    xyz: tuple[float, float, float],
    rpy: tuple[float, float, float],
) -> None:
    """Write an <origin> element.

    Args:
        parent: Parent XML element.
        xyz: Translation values.
        rpy: Roll-pitch-yaw rotation values.
    """
    origin_attrs = {}
    origin_attrs["xyz"] = " ".join(_fmt(v) for v in xyz)
    origin_attrs["rpy"] = " ".join(_fmt(v) for v in rpy)
    ET.SubElement(parent, "origin", **origin_attrs)


def _write_loop_joint(parent: ET.Element, lj: LoopJointData) -> None:
    """Write a <loop_joint> element for closed kinematic chains.

    Isaac Sim custom URDF tag:
      <loop_joint name="..." type="...">
        <link1 link="..." xyz="..." rpy="..."/>
        <link2 link="..." xyz="..." rpy="..."/>
      </loop_joint>

    Args:
        parent: Parent XML element.
        lj: Loop joint data to write.
    """
    lj_elem = ET.SubElement(parent, "loop_joint", name=lj.name, type=lj.joint_type)

    link1_attrs = {"link": lj.link1_name}
    link1_attrs["xyz"] = " ".join(_fmt(v) for v in lj.link1_xyz)
    link1_attrs["rpy"] = " ".join(_fmt(v) for v in lj.link1_rpy)
    ET.SubElement(lj_elem, "link1", **link1_attrs)

    link2_attrs = {"link": lj.link2_name}
    link2_attrs["xyz"] = " ".join(_fmt(v) for v in lj.link2_xyz)
    link2_attrs["rpy"] = " ".join(_fmt(v) for v in lj.link2_rpy)
    ET.SubElement(lj_elem, "link2", **link2_attrs)


def _fmt(value: float) -> str:
    """Format a float for URDF output, trimming trailing zeros.

    Args:
        value: Value to format.

    Returns:
        Formatted value string.
    """
    if value is None:
        return "0"
    if abs(value) < 1e-10:
        return "0"
    s = f"{value:.8f}"
    if "." in s:
        s = s.rstrip("0").rstrip(".")
    return s
