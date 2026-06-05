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
"""Main USD-to-URDF converter orchestrator.

Discovers robot structure from the physics graph (ArticulationRootAPI +
RigidBodyAPI + UsdPhysicsJoint), reads all link/joint/material data,
exports meshes, and writes a URDF file.
"""

from __future__ import annotations

import logging
import math
import os

from pxr import Usd

from .joint_reader import JointData, read_joints, read_loop_joints
from .link_reader import CollisionData, LinkData, VisualData, read_link
from .material_reader import collect_materials, populate_material_colors
from .mesh_exporter import MeshExporter
from .robot_finder import find_robot
from .transform_utils import get_prim_name, is_unit_scale, matrix4_to_origin_and_scale
from .urdf_frames import (
    build_urdf_frames,
    compute_geom_origin_from_frames,
    compute_geom_to_link_transform,
    compute_mesh_bake_transform,
)
from .urdf_writer import write_urdf

WORLD_LINK_NAME = "world"

_logger = logging.getLogger(__name__)


class UsdToUrdfConverter:
    """Convert a USD stage's articulated robot to URDF.

    Discovers robot structure from physics graph, reads link/joint/material
    data, exports meshes to OBJ, and writes URDF XML.

    Args:
        stage: An open ``Usd.Stage`` or a file-system path (str / os.PathLike)
            to a USD file (``.usd``, ``.usda``, ``.usdc``).  When a path is
            given the stage is opened with ``Usd.Stage.Open``.
        root_prim_path: Prim path of the articulation root.  ``None`` uses the
            stage default prim.
        mesh_dir_name: Subdirectory name for exported mesh files.
        mesh_path_prefix: Path prefix written into URDF mesh references.
        visualize_collision_meshes: If ``True``, duplicate collision meshes as
            visual geometry.
        variant_selections: Optional mapping of variant-set names to variant
            selections to apply on the root prim before conversion.  For
            example ``{"Physics": "PhysX", "LOD": "high"}``.  Selections are
            applied before robot discovery, so the chosen composition arcs
            determine which links, joints, and meshes are exported.  Variant
            sets that already have a selection are overridden.

    """

    def __init__(
        self,
        stage: Usd.Stage | str | os.PathLike,
        root_prim_path: str | None = None,
        mesh_dir_name: str = "meshes",
        mesh_path_prefix: str = "./",
        visualize_collision_meshes: bool = False,
        variant_selections: dict[str, str] | None = None,
    ) -> None:
        """Initialize the converter; see class docstring for parameter descriptions."""
        if isinstance(stage, (str, os.PathLike)):
            usd_path = str(stage)
            stage = Usd.Stage.Open(usd_path)
            if stage is None:
                raise ValueError(f"Failed to open USD stage from path: {usd_path}")
        self._stage = stage
        self._root_prim_path = root_prim_path
        self._mesh_dir_name = mesh_dir_name
        self._mesh_path_prefix = mesh_path_prefix
        self._visualize_collision_meshes = visualize_collision_meshes
        self._variant_selections = variant_selections

    def convert(self, output_path: str | None = None) -> str:
        """Convert the USD stage to URDF and write to *output_path*.

        When *output_path* is ``None`` the URDF is written next to the source
        USD file using the same base name (``robot.usd`` -> ``robot.urdf``).
        For stages loaded from ``omniverse://`` URLs, ``omni.client`` is used
        to parse the source path; if the source is remote the current working
        directory is used as the output location.

        Raises:
            ValueError: If *output_path* is ``None`` and no source path can
                be determined from the stage.
        """
        if output_path is None:
            output_path = self._resolve_default_output_path()

        _logger.info(f"Starting USD to URDF conversion, output: {output_path}")

        if self._variant_selections:
            self._apply_variant_selections()

        desc = find_robot(self._stage, self._root_prim_path)
        _logger.info(
            f"Found robot '{desc.name}' with {len(desc.ordered_links)} links and {len(desc.ordered_joints)} joints"
        )

        output_dir = os.path.dirname(os.path.abspath(output_path))
        mesh_dir = os.path.join(output_dir, self._mesh_dir_name)
        mesh_prefix = self._mesh_path_prefix
        if not mesh_prefix or mesh_prefix == "./":
            mesh_prefix = f"./{self._mesh_dir_name}/"
        elif mesh_prefix == "file://":
            abs_mesh_dir = os.path.abspath(mesh_dir)
            mesh_prefix = f"file://{abs_mesh_dir}/"
        elif mesh_prefix.startswith("package://"):
            if not mesh_prefix.endswith("/"):
                mesh_prefix += "/"
            mesh_prefix = f"{mesh_prefix}{self._mesh_dir_name}/"
        else:
            if not mesh_prefix.endswith("/"):
                mesh_prefix += "/"

        mesh_exporter = MeshExporter(mesh_dir, mesh_prefix)

        # Build URDF frames for all links using joint poses from robot_schema
        urdf_frames, axis_flips = build_urdf_frames(desc)

        link_name_map: dict[str, str] = {}
        links_data: list[LinkData] = []

        for link_prim in desc.ordered_links:
            link_path = str(link_prim.GetPath())
            link_data = read_link(link_prim)
            link_name_map[link_path] = link_data.name

            # Recompute geometry origins and export meshes
            for visual in link_data.visuals:
                _process_element_geometry(visual, link_path, urdf_frames, desc.root_prim, mesh_exporter)

            for collision in link_data.collisions:
                _process_element_geometry(collision, link_path, urdf_frames, desc.root_prim, mesh_exporter)

            links_data.append(link_data)

        actuator_map = _build_actuator_map(desc.root_prim)

        joints_data, ghost_links = read_joints(desc, link_name_map, urdf_frames, axis_flips, actuator_map)
        links_data.extend(ghost_links)
        _logger.info(f"Read {len(joints_data)} joints for URDF export")

        if not desc.is_fixed_base and links_data:
            world_link = LinkData(name=WORLD_LINK_NAME)
            floating_joint = JointData(
                name="floating_base",
                joint_type="floating",
                parent_link=WORLD_LINK_NAME,
                child_link=links_data[0].name,
            )
            links_data.insert(0, world_link)
            joints_data.insert(0, floating_joint)

        loop_joints_data = read_loop_joints(desc, link_name_map)
        if loop_joints_data:
            _logger.info(f"Read {len(loop_joints_data)} loop joints for URDF export")

        for site in desc.sites:
            site_name = get_prim_name(site.prim)
            parent_path = str(site.parent_link_prim.GetPath())
            parent_name = link_name_map.get(parent_path, "")
            if not parent_name:
                continue

            site_link = LinkData(name=site_name)
            links_data.append(site_link)

            origin_xyz, origin_rpy = compute_geom_origin_from_frames(
                urdf_frames, parent_path, site.prim, desc.root_prim
            )

            site_joint = JointData(
                name=f"{site_name}_fixed_joint",
                joint_type="fixed",
                parent_link=parent_name,
                child_link=site_name,
                origin_xyz=origin_xyz,
                origin_rpy=origin_rpy,
            )
            joints_data.append(site_joint)

        if desc.sites:
            _logger.info(f"Added {len(desc.sites)} site frames as ghost links")

        materials = collect_materials(links_data, mesh_dir)
        populate_material_colors(materials, self._stage, mesh_dir)

        write_urdf(desc.name, links_data, joints_data, materials, output_path, loop_joints=loop_joints_data)
        _logger.info(f"URDF written to {output_path}")

        return output_path

    def _resolve_default_output_path(self) -> str:
        """Derive a ``.urdf`` output path from the stage's source layer."""
        layer = self._stage.GetRootLayer()
        real_path = layer.realPath
        identifier = layer.identifier

        # Local file — use its directory and basename
        if real_path and os.path.isabs(real_path):
            dirname = os.path.dirname(real_path)
            stem = os.path.splitext(os.path.basename(real_path))[0]
            return os.path.join(dirname, f"{stem}.urdf")

        # omniverse:// URL — use omni.client to resolve directory and basename
        if identifier and identifier.startswith("omniverse://"):
            dirurl, basename = _split_omniverse_url(identifier)
            if not basename:
                raise ValueError(
                    f"Cannot extract file name from omniverse URL: {identifier}. "
                    "Pass an explicit output_path to convert()."
                )
            stem = os.path.splitext(basename)[0]
            if not dirurl:
                raise ValueError(
                    f"Cannot resolve directory from omniverse URL: {identifier}. "
                    "Pass an explicit output_path to convert()."
                )
            return f"{dirurl}/{stem}.urdf"

        # Fallback: treat identifier as a local path
        if identifier and not identifier.startswith("anon"):
            dirname = os.path.dirname(os.path.abspath(identifier))
            stem = os.path.splitext(os.path.basename(identifier))[0]
            return os.path.join(dirname, f"{stem}.urdf")

        raise ValueError(
            "Cannot derive output path: stage has no source file. " "Pass an explicit output_path to convert()."
        )

    def _apply_variant_selections(self) -> None:
        """Set variant selections on the root prim before robot discovery."""
        root_prim = None
        if self._root_prim_path:
            path = self._root_prim_path if self._root_prim_path.startswith("/") else f"/{self._root_prim_path}"
            prim = self._stage.GetPrimAtPath(path)
            if prim and prim.IsValid():
                root_prim = prim
        if root_prim is None:
            root_prim = self._stage.GetDefaultPrim()
        if root_prim is None or not root_prim.IsValid():
            _logger.warning("Cannot apply variant selections: no valid root prim found")
            return

        vsets = root_prim.GetVariantSets()
        for set_name, selection in self._variant_selections.items():
            if vsets.HasVariantSet(set_name):
                vsets.GetVariantSet(set_name).SetVariantSelection(selection)
                _logger.info(f"Set variant '{set_name}' = '{selection}' on {root_prim.GetPath()}")
            else:
                _logger.warning(f"Variant set '{set_name}' not found on {root_prim.GetPath()}")


def _split_omniverse_url(url: str) -> tuple[str, str]:
    """Split an ``omniverse://`` URL into ``(directory_url, basename)``.

    Uses ``omni.client.break_url`` / ``make_url`` when available, falling
    back to naive string splitting otherwise.
    """
    try:
        import omni.client

        result = omni.client.break_url(url)
        path = result.path or ""
        if "/" in path:
            dir_path, basename = path.rsplit("/", 1)
        else:
            dir_path, basename = "", path
        dir_url = omni.client.make_url(result.scheme, result.user, result.host, result.port, dir_path)
        return dir_url, basename
    except (ImportError, AttributeError):
        pass
    # Fallback for environments without omni.client
    idx = url.rfind("/")
    if idx > 0:
        return url[:idx], url[idx + 1 :]
    return "", url


def _process_element_geometry(
    element: VisualData | CollisionData,
    link_path: str,
    urdf_frames: dict,
    root_prim: Usd.Prim,
    mesh_exporter: MeshExporter,
) -> None:
    """Compute origin and export mesh data for a single visual/collision element."""
    geom = element.geometry
    if geom is None:
        return

    if geom.geom_type == "mesh" and geom.mesh_prim is not None:
        _export_mesh_geometry(element, link_path, urdf_frames, root_prim, mesh_exporter)
    elif geom.geom_type == "mesh" and geom.mesh_prim is None and geom.original_type == "Cone":
        _export_procedural_cone(element, link_path, urdf_frames, root_prim, mesh_exporter)
    elif geom.source_prim:
        origin_xyz, origin_rpy = compute_geom_origin_from_frames(urdf_frames, link_path, geom.source_prim, root_prim)
        if geom.local_offset_xyz != (0.0, 0.0, 0.0):
            origin_xyz = _compose_local_offset(origin_xyz, origin_rpy, geom.local_offset_xyz)
        element.origin_xyz = origin_xyz
        element.origin_rpy = origin_rpy


def _compose_local_offset(
    origin_xyz: tuple[float, float, float],
    origin_rpy: tuple[float, float, float],
    local_offset: tuple[float, float, float],
) -> tuple[float, float, float]:
    """Rotate a local-frame offset by the origin's RPY and add to origin XYZ.

    The local offset is a translation in the geometry prim's local frame.
    In the URDF link frame it becomes ``R(rpy) @ offset + xyz``.
    """
    r, p, y = origin_rpy
    cr, sr = math.cos(r), math.sin(r)
    cp, sp = math.cos(p), math.sin(p)
    cy, sy = math.cos(y), math.sin(y)

    # ZYX Euler rotation matrix (URDF convention)
    ox, oy, oz = local_offset
    rx = cy * cp * ox + (cy * sp * sr - sy * cr) * oy + (cy * sp * cr + sy * sr) * oz
    ry = sy * cp * ox + (sy * sp * sr + cy * cr) * oy + (sy * sp * cr - cy * sr) * oz
    rz = -sp * ox + cp * sr * oy + cp * cr * oz

    return (origin_xyz[0] + rx, origin_xyz[1] + ry, origin_xyz[2] + rz)


def _export_procedural_cone(
    element: VisualData | CollisionData,
    link_path: str,
    urdf_frames: dict,
    root_prim: Usd.Prim,
    mesh_exporter: MeshExporter,
) -> None:
    """Generate a cone mesh OBJ procedurally and assign it to the element."""
    geom = element.geometry
    params = geom.original_params
    name = params.get("source_prim_name", "cone")
    radius = params["radius"]
    height = params["height"]
    axis = params.get("axis", "Z")

    filename = mesh_exporter.export_cone(name, radius, height, axis)
    geom.mesh_filename = filename
    geom.mesh_scale = None

    if geom.source_prim:
        origin_xyz, origin_rpy = compute_geom_origin_from_frames(urdf_frames, link_path, geom.source_prim, root_prim)
        element.origin_xyz = origin_xyz
        element.origin_rpy = origin_rpy
    else:
        element.origin_xyz = (0.0, 0.0, 0.0)
        element.origin_rpy = (0.0, 0.0, 0.0)


def _build_actuator_map(root_prim: Usd.Prim) -> dict[str, Usd.Prim]:
    """Build a map from joint prim path to its MjcActuator prim.

    Traverses the subtree under *root_prim* looking for ``MjcActuator``
    prims and resolves their ``mjc:target`` relationship to identify the
    target joint.  Returns an empty dict when no actuators are present.
    """
    actuator_map: dict[str, Usd.Prim] = {}
    for prim in Usd.PrimRange(root_prim, Usd.TraverseInstanceProxies()):
        if prim.GetTypeName() != "MjcActuator":
            continue
        target_rel = prim.GetRelationship("mjc:target")
        if not target_rel or not target_rel.IsValid():
            continue
        targets = target_rel.GetTargets()
        if targets:
            actuator_map[str(targets[0])] = prim
    return actuator_map


def _is_instance_proxy(prim: Usd.Prim) -> bool:
    """Check if a prim is a USD instance proxy (lives inside an instanceable subtree)."""
    if prim.IsInstanceProxy():
        return True
    current = prim.GetParent()
    while current and current.IsValid() and not current.IsPseudoRoot():
        if current.IsInstanceProxy():
            return True
        current = current.GetParent()
    return False


def _export_mesh_geometry(
    element: VisualData | CollisionData,
    link_path: str,
    urdf_frames: dict,
    root_prim: Usd.Prim,
    mesh_exporter: MeshExporter,
) -> None:
    """Export a mesh visual or collision, sharing OBJ files for USD instances.

    Instance proxy meshes are exported without baking transforms so that
    multiple links referencing the same prototype reuse a single OBJ file.
    The mesh-to-link placement is expressed via the URDF origin instead.

    Non-instanced meshes are baked as before (origin = identity).
    """
    geom = element.geometry
    mesh_prim = geom.mesh_prim
    source_prim = geom.source_prim

    if _is_instance_proxy(mesh_prim):
        # Instanced meshes are exported untransformed so multiple links can
        # share a single OBJ. The mesh-to-link placement is split into a
        # URDF <origin> (rotation+translation) and a <mesh scale=...>
        # attribute (scale on the geom path, e.g. an ancestor xform).
        relative = compute_geom_to_link_transform(urdf_frames, link_path, source_prim, root_prim)
        origin_xyz, origin_rpy, scale = matrix4_to_origin_and_scale(relative)

        filename = mesh_exporter.export_mesh(mesh_prim, bake_transform=None)
        geom.mesh_filename = filename
        geom.mesh_scale = None if is_unit_scale(scale) else scale
        element.origin_xyz = origin_xyz
        element.origin_rpy = origin_rpy
    else:
        # Non-instanced meshes bake the full transform (including any scale)
        # into vertex coordinates so the URDF geometry origin is identity
        # and no <mesh scale=...> is required.
        bake_xf = compute_mesh_bake_transform(urdf_frames, link_path, source_prim, root_prim)
        filename = mesh_exporter.export_mesh(mesh_prim, bake_transform=bake_xf)
        geom.mesh_filename = filename
        geom.mesh_scale = None
        element.origin_xyz = (0.0, 0.0, 0.0)
        element.origin_rpy = (0.0, 0.0, 0.0)
