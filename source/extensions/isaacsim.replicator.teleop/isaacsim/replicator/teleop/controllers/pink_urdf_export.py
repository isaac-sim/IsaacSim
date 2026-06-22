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

"""Internal teleop-specific URDF export helpers for PINK."""

from __future__ import annotations

import os
import tempfile
import xml.etree.ElementTree as ET
from typing import Any

import numpy as np


def _is_path_in_scope(path: str, root_path: str) -> bool:
    """Whether *path* is at or below *root_path*.

    Args:
        path: Value for path.
        root_path: Value for root path.

    Returns:
        The requested value.
    """
    return path == root_path or path.startswith(f"{root_path}/")


def _resolve_converter_root(stage: Any, articulation_path: str) -> str:
    """Resolve a stable robot subtree root for PINK URDF export.

    ``Articulation.fetch_articulation_root_api_prim_paths`` may return a
    root-joint prim (e.g. ``.../ur3e/root_joint``). The custom PINK exporter
    expects the robot subtree root instead, so export from the parent prim
    (``.../ur3e``) when the articulation root resolves to a joint prim.

    Args:
        stage: Value for stage.
        articulation_path: Value for articulation path.

    Returns:
        The requested value.
    """
    from pxr import UsdPhysics

    articulation_prim = stage.GetPrimAtPath(articulation_path)
    if not articulation_prim:
        return articulation_path

    if articulation_prim.IsA(UsdPhysics.Joint):
        parent = articulation_prim.GetParent()
        if parent and parent.IsValid() and str(parent.GetPath()) != "/":
            return str(parent.GetPath())

    return articulation_path


def _format_urdf_floats(values: tuple[float, ...] | list[float]) -> str:
    """Format float tuples for URDF attributes.

    Args:
        values: Value for values.

    Returns:
        The requested value.
    """
    return " ".join(f"{float(value):.9g}" for value in values)


def _relative_export_parts(path: str, root_path: str) -> list[str]:
    """Return path parts under the export root, keeping names useful for URDF.

    Args:
        path: Value for path.
        root_path: Value for root path.

    Returns:
        The requested value.
    """
    path_parts = [part for part in path.split("/") if part]
    root_parts = [part for part in root_path.split("/") if part]
    if len(path_parts) < len(root_parts) or path_parts[: len(root_parts)] != root_parts:
        return [path_parts[-1]] if path_parts else [path]

    relative_parts = path_parts[len(root_parts) :]
    useful_parts = [root_parts[-1]]
    useful_parts.extend(part for part in relative_parts if part not in {"joints", "Looks"})
    return useful_parts


def _make_export_name(path: str, root_path: str) -> str:
    """Build a stable URDF-safe name for a prim path.

    Args:
        path: Value for path.
        root_path: Value for root path.

    Returns:
        The requested value.
    """
    return "_".join(_relative_export_parts(path, root_path))


def _build_export_name_map(paths: list[str], root_path: str) -> dict[str, str]:
    """Build unique export names for the provided paths.

    Args:
        paths: Value for paths.
        root_path: Value for root path.

    Returns:
        The requested value.
    """
    name_map = {path: _make_export_name(path, root_path) for path in paths}
    collisions: dict[str, list[str]] = {}
    for path, name in name_map.items():
        collisions.setdefault(name, []).append(path)
    dupes = {name: dup_paths for name, dup_paths in collisions.items() if len(dup_paths) > 1}
    if dupes:
        raise RuntimeError(f"Duplicate export names generated under '{root_path}': {dupes}")
    return name_map


def _quat_to_rpy_xyz(quat: Any) -> tuple[float, float, float]:
    """Convert a USD quaternion to URDF XYZ fixed-axis roll/pitch/yaw.

    Args:
        quat: Value for quat.

    Returns:
        The requested value.
    """
    imag = quat.GetImaginary()
    x = float(imag[0])
    y = float(imag[1])
    z = float(imag[2])
    w = float(quat.GetReal())

    sinr_cosp = 2.0 * (w * x + y * z)
    cosr_cosp = 1.0 - 2.0 * (x * x + y * y)
    roll = float(np.arctan2(sinr_cosp, cosr_cosp))

    sinp = 2.0 * (w * y - z * x)
    if abs(sinp) >= 1.0:
        pitch = float(np.copysign(np.pi / 2.0, sinp))
    else:
        pitch = float(np.arcsin(sinp))

    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    yaw = float(np.arctan2(siny_cosp, cosy_cosp))
    return roll, pitch, yaw


def _joint_origin_from_parent(joint: Any) -> tuple[tuple[float, float, float], tuple[float, float, float]]:
    """Read a joint origin in the parent-link frame from USD joint local attrs.

    Args:
        joint: Value for joint.

    Returns:
        The requested value.
    """
    pos_attr = joint.GetLocalPos0Attr()
    rot_attr = joint.GetLocalRot0Attr()

    pos = pos_attr.Get() if pos_attr and pos_attr.Get() is not None else (0.0, 0.0, 0.0)
    rot = rot_attr.Get() if rot_attr and rot_attr.Get() is not None else None

    xyz = (float(pos[0]), float(pos[1]), float(pos[2]))
    rpy = (0.0, 0.0, 0.0) if rot is None else _quat_to_rpy_xyz(rot)
    return xyz, rpy


def _axis_attr_to_xyz(axis_value: Any) -> tuple[float, float, float]:
    """Map USD axis tokens to URDF axis vectors.

    Args:
        axis_value: Value for axis value.

    Returns:
        The requested value.
    """
    axis_map = {
        "X": (1.0, 0.0, 0.0),
        "Y": (0.0, 1.0, 0.0),
        "Z": (0.0, 0.0, 1.0),
        "-X": (-1.0, 0.0, 0.0),
        "-Y": (0.0, -1.0, 0.0),
        "-Z": (0.0, 0.0, -1.0),
    }
    return axis_map.get(str(axis_value), (1.0, 0.0, 0.0))


def _joint_type_and_limits(joint_prim: Any) -> tuple[str, tuple[float, float, float] | None, dict[str, str] | None]:
    """Convert a USD joint prim into URDF joint metadata.

    Args:
        joint_prim: Value for joint prim.

    Returns:
        The requested value.
    """
    from pxr import UsdPhysics

    limit_defaults = {"effort": "1000", "velocity": "1000"}

    if joint_prim.IsA(UsdPhysics.RevoluteJoint):
        rev_joint = UsdPhysics.RevoluteJoint(joint_prim)
        axis = _axis_attr_to_xyz(rev_joint.GetAxisAttr().Get())
        lower_attr = rev_joint.GetLowerLimitAttr()
        upper_attr = rev_joint.GetUpperLimitAttr()
        lower = float(np.radians(lower_attr.Get())) if lower_attr and lower_attr.Get() is not None else None
        upper = float(np.radians(upper_attr.Get())) if upper_attr and upper_attr.Get() is not None else None
        if lower is None or upper is None or lower > upper:
            return "continuous", axis, limit_defaults
        return (
            "revolute",
            axis,
            {
                **limit_defaults,
                "lower": f"{lower:.9g}",
                "upper": f"{upper:.9g}",
            },
        )

    if joint_prim.IsA(UsdPhysics.PrismaticJoint):
        pri_joint = UsdPhysics.PrismaticJoint(joint_prim)
        axis = _axis_attr_to_xyz(pri_joint.GetAxisAttr().Get())
        lower_attr = pri_joint.GetLowerLimitAttr()
        upper_attr = pri_joint.GetUpperLimitAttr()
        lower = float(lower_attr.Get()) if lower_attr and lower_attr.Get() is not None else -1e6
        upper = float(upper_attr.Get()) if upper_attr and upper_attr.Get() is not None else 1e6
        if lower > upper:
            lower, upper = -1e6, 1e6
        return (
            "prismatic",
            axis,
            {
                **limit_defaults,
                "lower": f"{lower:.9g}",
                "upper": f"{upper:.9g}",
            },
        )

    return "fixed", None, None


def _collect_minimal_urdf_graph(stage: Any, export_root_path: str) -> tuple[str, list[str], list[dict], dict[str, str]]:
    """Collect an in-scope kinematic graph for minimal URDF export.

    Args:
        stage: Value for stage.
        export_root_path: Value for export root path.

    Returns:
        The requested value.
    """
    from pxr import Usd, UsdPhysics

    root_prim = stage.GetPrimAtPath(export_root_path)
    if not root_prim or not root_prim.IsValid():
        raise RuntimeError(f"Export root '{export_root_path}' does not exist on the stage")

    joint_records: list[dict] = []
    link_paths: set[str] = set()
    child_link_paths: set[str] = set()
    root_link_candidates: list[str] = []

    for prim in Usd.PrimRange(root_prim):
        if not prim.IsA(UsdPhysics.Joint):
            continue

        joint = UsdPhysics.Joint(prim)
        body0_targets = joint.GetBody0Rel().GetTargets()
        body1_targets = joint.GetBody1Rel().GetTargets()
        body0_path = str(body0_targets[0]) if body0_targets else None
        body1_path = str(body1_targets[0]) if body1_targets else None

        if not body1_path or not _is_path_in_scope(body1_path, export_root_path):
            continue

        if not body0_path:
            root_link_candidates.append(body1_path)
            link_paths.add(body1_path)
            continue

        if not _is_path_in_scope(body0_path, export_root_path):
            continue

        joint_type, axis_xyz, limit_attrs = _joint_type_and_limits(prim)
        origin_xyz, origin_rpy = _joint_origin_from_parent(joint)
        joint_records.append(
            {
                "prim_path": str(prim.GetPath()),
                "parent_path": body0_path,
                "child_path": body1_path,
                "joint_type": joint_type,
                "origin_xyz": origin_xyz,
                "origin_rpy": origin_rpy,
                "axis_xyz": axis_xyz,
                "limit_attrs": limit_attrs,
            }
        )
        link_paths.add(body0_path)
        link_paths.add(body1_path)
        child_link_paths.add(body1_path)

    if not link_paths:
        raise RuntimeError(f"No in-scope links found under '{export_root_path}'")

    if root_link_candidates:
        root_link_path = root_link_candidates[0]
    else:
        standalone_candidates = sorted(link_paths - child_link_paths)
        if not standalone_candidates:
            raise RuntimeError(f"Could not determine a root link under '{export_root_path}'")
        root_link_path = standalone_candidates[0]

    ordered_link_paths = sorted(link_paths)
    link_name_by_path = _build_export_name_map(ordered_link_paths, export_root_path)
    joint_name_by_path = _build_export_name_map([record["prim_path"] for record in joint_records], export_root_path)

    children_by_parent: dict[str, list[dict]] = {}
    for record in joint_records:
        record["parent_name"] = link_name_by_path[record["parent_path"]]
        record["child_name"] = link_name_by_path[record["child_path"]]
        record["joint_name"] = joint_name_by_path[record["prim_path"]]
        children_by_parent.setdefault(record["parent_path"], []).append(record)

    ordered_joints: list[dict] = []
    queue = [root_link_path]
    visited_links = {root_link_path}
    while queue:
        parent_path = queue.pop(0)
        for record in sorted(children_by_parent.get(parent_path, []), key=lambda item: item["joint_name"]):
            ordered_joints.append(record)
            child_path = record["child_path"]
            if child_path not in visited_links:
                visited_links.add(child_path)
                queue.append(child_path)

    ordered_export_links = [root_link_path]
    ordered_export_links.extend(sorted(path for path in visited_links if path != root_link_path))

    return root_link_path, ordered_export_links, ordered_joints, link_name_by_path


def _write_minimal_urdf(
    urdf_path: str,
    robot_name: str,
    ordered_link_paths: list[str],
    ordered_joints: list[dict],
    link_name_by_path: dict[str, str],
) -> None:
    """Write a minimal kinematic URDF for Pinocchio.

    Args:
        urdf_path: Value for urdf path.
        robot_name: Value for robot name.
        ordered_link_paths: Value for ordered link paths.
        ordered_joints: Value for ordered joints.
        link_name_by_path: Value for link name by path.
    """
    robot_el = ET.Element("robot", name=robot_name)

    for link_path in ordered_link_paths:
        ET.SubElement(robot_el, "link", name=link_name_by_path[link_path])

    for record in ordered_joints:
        joint_el = ET.SubElement(
            robot_el,
            "joint",
            name=record["joint_name"],
            type=record["joint_type"],
        )
        ET.SubElement(joint_el, "parent", link=record["parent_name"])
        ET.SubElement(joint_el, "child", link=record["child_name"])
        ET.SubElement(
            joint_el,
            "origin",
            xyz=_format_urdf_floats(record["origin_xyz"]),
            rpy=_format_urdf_floats(record["origin_rpy"]),
        )
        if record["axis_xyz"] is not None:
            ET.SubElement(joint_el, "axis", xyz=_format_urdf_floats(record["axis_xyz"]))
        if record["limit_attrs"] is not None:
            ET.SubElement(joint_el, "limit", **record["limit_attrs"])

    tree = ET.ElementTree(robot_el)
    try:
        ET.indent(tree, space="  ")
    except AttributeError:
        pass
    tree.write(urdf_path, encoding="utf-8", xml_declaration=True)


def _export_minimal_urdf(
    articulation_path: str,
    export_root_path: str | None = None,
) -> tuple[str, str, str]:
    """Export the minimal kinematic URDF used by PINK.

    PINK only needs the in-scope kinematic tree and Pinocchio loads URDFs via
    ``urdfdom``, which requires valid joint ``effort`` / ``velocity`` limit
    attributes. The generic USD exporter can omit those attributes for teleop
    assets, so teleop maintains this minimal exporter to produce a small,
    deterministic URDF that Pinocchio accepts.

    Args:
        articulation_path: Value for articulation path.
        export_root_path: Value for export root path.

    Returns:
        The requested value.
    """
    import omni.usd
    from pxr import Sdf

    stage = omni.usd.get_context().get_stage()
    if not stage:
        raise RuntimeError("No USD stage available for minimal URDF export")

    requested_root_path = export_root_path or articulation_path
    requested_root_prim = stage.GetPrimAtPath(requested_root_path) if requested_root_path else None
    if requested_root_prim and requested_root_prim.IsValid():
        converter_root_path = _resolve_converter_root(stage, requested_root_path)
    else:
        converter_root_path = _resolve_converter_root(stage, articulation_path)
    root_prim = stage.GetPrimAtPath(converter_root_path)
    if not root_prim or not root_prim.IsValid():
        raise RuntimeError(f"Minimal URDF export root '{converter_root_path}' is invalid")

    robot_name = _make_export_name(converter_root_path, converter_root_path)
    temp_dir = tempfile.mkdtemp(prefix="teleop_pink_minimal_")
    urdf_path = os.path.join(temp_dir, f"{Sdf.Path(converter_root_path).name}.urdf")
    mesh_dir = os.path.join(temp_dir, "meshes")
    os.makedirs(mesh_dir, exist_ok=True)

    root_link_path, ordered_link_paths, ordered_joints, link_name_by_path = _collect_minimal_urdf_graph(
        stage, converter_root_path
    )
    _write_minimal_urdf(
        urdf_path=urdf_path,
        robot_name=robot_name,
        ordered_link_paths=ordered_link_paths,
        ordered_joints=ordered_joints,
        link_name_by_path=link_name_by_path,
    )
    return urdf_path, mesh_dir, root_link_path


def _export_urdf(
    articulation_path: str,
    export_root_path: str | None = None,
) -> tuple[str, str, str]:
    """Export the custom URDF used by PINK.

    Args:
        articulation_path: Value for articulation path.
        export_root_path: Value for export root path.

    Returns:
        The requested value.
    """
    return _export_minimal_urdf(
        articulation_path=articulation_path,
        export_root_path=export_root_path,
    )
