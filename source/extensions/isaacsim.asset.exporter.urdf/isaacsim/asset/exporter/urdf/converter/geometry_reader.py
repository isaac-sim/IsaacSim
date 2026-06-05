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
"""Read geometry primitives from USD for URDF export."""

from __future__ import annotations

import logging
from dataclasses import dataclass, field

from pxr import Usd, UsdGeom

from .transform_utils import get_prim_name

_logger = logging.getLogger(__name__)


@dataclass
class GeometryData:
    """URDF geometry element data."""

    geom_type: str = ""  # "box", "sphere", "cylinder", "mesh"
    source_prim: Usd.Prim | None = None
    # Box
    box_size: tuple[float, float, float] | None = None
    # Sphere
    sphere_radius: float | None = None
    # Cylinder
    cylinder_radius: float | None = None
    cylinder_length: float | None = None
    # Mesh
    mesh_prim: Usd.Prim | None = None
    mesh_scale: tuple[float, float, float] | None = None
    mesh_filename: str | None = None
    # Compound / converted primitives
    name_suffix: str = ""
    local_offset_xyz: tuple[float, float, float] = (0.0, 0.0, 0.0)
    # Round-trip breadcrumb for the importer to reconstruct the original prim
    original_type: str | None = None
    original_params: dict = field(default_factory=dict)


def read_geometry(prim: Usd.Prim) -> list[GeometryData]:
    """Read geometry data from a USD prim.

    Args:
        prim: A UsdGeomGprim (Cube, Sphere, Cylinder, Capsule, Cone, Mesh, etc.)

    Returns:
        List of GeometryData (empty if the prim type is unsupported).
        Most types yield a single element; Capsule yields three (cylinder body
        + two sphere caps).

    """
    type_name = prim.GetTypeName()

    if type_name == "Cube":
        return [_read_cube(prim)]
    elif type_name == "Sphere":
        return [_read_sphere(prim)]
    elif type_name == "Cylinder":
        return [_read_cylinder(prim)]
    elif type_name == "Capsule":
        _logger.warning(
            "Capsule '%s' has no URDF equivalent; converting to cylinder + 2 sphere caps",
            prim.GetPath(),
        )
        return _read_capsule(prim)
    elif type_name == "Cone":
        _logger.warning(
            "Cone '%s' has no URDF equivalent; converting to tessellated mesh",
            prim.GetPath(),
        )
        return [_read_cone(prim)]
    elif type_name == "Mesh":
        return [_read_mesh(prim)]

    _logger.warning(
        "Unsupported geometry type '%s' at '%s'; skipping prim",
        type_name,
        prim.GetPath(),
    )
    return []


def _read_cube(prim: Usd.Prim) -> GeometryData:
    """Read box geometry from a UsdGeomCube.

    Newton convention: size=1.0 with scale XformOp for dimensions.
    General: size attribute * any scale.
    """
    cube = UsdGeom.Cube(prim)
    size_attr = cube.GetSizeAttr()
    size = float(size_attr.Get()) if size_attr and size_attr.Get() is not None else 2.0

    scale = _get_scale(prim)

    box_size = (size * scale[0], size * scale[1], size * scale[2])

    return GeometryData(geom_type="box", source_prim=prim, box_size=box_size)


def _read_sphere(prim: Usd.Prim) -> GeometryData:
    """Read sphere geometry from a UsdGeomSphere."""
    sphere = UsdGeom.Sphere(prim)
    radius_attr = sphere.GetRadiusAttr()
    radius = float(radius_attr.Get()) if radius_attr and radius_attr.Get() is not None else 1.0

    scale = _get_scale(prim)
    max_scale = max(abs(scale[0]), abs(scale[1]), abs(scale[2]))
    if max_scale != 1.0:
        radius *= max_scale

    return GeometryData(geom_type="sphere", source_prim=prim, sphere_radius=radius)


def _read_cylinder(prim: Usd.Prim) -> GeometryData:
    """Read cylinder geometry from a UsdGeomCylinder."""
    cyl = UsdGeom.Cylinder(prim)
    radius_attr = cyl.GetRadiusAttr()
    height_attr = cyl.GetHeightAttr()
    radius = float(radius_attr.Get()) if radius_attr and radius_attr.Get() is not None else 1.0
    height = float(height_attr.Get()) if height_attr and height_attr.Get() is not None else 2.0

    scale = _get_scale(prim)
    axis_attr = cyl.GetAxisAttr()
    axis = axis_attr.Get() if axis_attr else "Z"

    if axis == "Z":
        radius *= max(abs(scale[0]), abs(scale[1]))
        height *= abs(scale[2])
    elif axis == "Y":
        radius *= max(abs(scale[0]), abs(scale[2]))
        height *= abs(scale[1])
    else:
        radius *= max(abs(scale[1]), abs(scale[2]))
        height *= abs(scale[0])

    return GeometryData(geom_type="cylinder", source_prim=prim, cylinder_radius=radius, cylinder_length=height)


def _read_capsule(prim: Usd.Prim) -> list[GeometryData]:
    """Read capsule geometry from a UsdGeomCapsule.

    URDF has no capsule primitive, so decompose into a cylinder (the body)
    and two spheres (end caps).  Each piece carries ``original_type`` /
    ``original_params`` so the importer can reconstruct the capsule.
    """
    capsule = UsdGeom.Capsule(prim)
    radius_attr = capsule.GetRadiusAttr()
    height_attr = capsule.GetHeightAttr()
    radius = float(radius_attr.Get()) if radius_attr and radius_attr.Get() is not None else 0.5
    height = float(height_attr.Get()) if height_attr and height_attr.Get() is not None else 1.0

    scale = _get_scale(prim)
    axis_attr = capsule.GetAxisAttr()
    axis = axis_attr.Get() if axis_attr else "Z"

    if axis == "Z":
        radius *= max(abs(scale[0]), abs(scale[1]))
        height *= abs(scale[2])
    elif axis == "Y":
        radius *= max(abs(scale[0]), abs(scale[2]))
        height *= abs(scale[1])
    else:
        radius *= max(abs(scale[1]), abs(scale[2]))
        height *= abs(scale[0])

    half_h = height / 2.0
    prim_name = get_prim_name(prim)
    breadcrumb = {
        "radius": radius,
        "height": height,
        "axis": axis,
        "source_prim_name": prim_name,
    }

    axis_offsets = {"X": (1.0, 0.0, 0.0), "Y": (0.0, 1.0, 0.0), "Z": (0.0, 0.0, 1.0)}
    ax = axis_offsets.get(axis, (0.0, 0.0, 1.0))
    top_offset = (ax[0] * half_h, ax[1] * half_h, ax[2] * half_h)
    bot_offset = (-ax[0] * half_h, -ax[1] * half_h, -ax[2] * half_h)

    body = GeometryData(
        geom_type="cylinder",
        source_prim=prim,
        cylinder_radius=radius,
        cylinder_length=height,
        name_suffix="_body",
        original_type="Capsule",
        original_params=breadcrumb,
    )
    top_cap = GeometryData(
        geom_type="sphere",
        source_prim=prim,
        sphere_radius=radius,
        name_suffix="_top_cap",
        local_offset_xyz=top_offset,
        original_type="Capsule",
        original_params=breadcrumb,
    )
    bottom_cap = GeometryData(
        geom_type="sphere",
        source_prim=prim,
        sphere_radius=radius,
        name_suffix="_bottom_cap",
        local_offset_xyz=bot_offset,
        original_type="Capsule",
        original_params=breadcrumb,
    )
    return [body, top_cap, bottom_cap]


def _read_cone(prim: Usd.Prim) -> GeometryData:
    """Read cone geometry from a UsdGeomCone.

    URDF has no cone primitive, so it is exported as a procedurally
    tessellated mesh.  The original parameters are preserved in
    ``original_type`` / ``original_params`` for round-trip reconstruction.
    """
    cone = UsdGeom.Cone(prim)
    radius_attr = cone.GetRadiusAttr()
    height_attr = cone.GetHeightAttr()
    radius = float(radius_attr.Get()) if radius_attr and radius_attr.Get() is not None else 1.0
    height = float(height_attr.Get()) if height_attr and height_attr.Get() is not None else 2.0

    scale = _get_scale(prim)
    axis_attr = cone.GetAxisAttr()
    axis = axis_attr.Get() if axis_attr else "Z"

    if axis == "Z":
        radius *= max(abs(scale[0]), abs(scale[1]))
        height *= abs(scale[2])
    elif axis == "Y":
        radius *= max(abs(scale[0]), abs(scale[2]))
        height *= abs(scale[1])
    else:
        radius *= max(abs(scale[1]), abs(scale[2]))
        height *= abs(scale[0])

    prim_name = get_prim_name(prim)
    breadcrumb = {
        "radius": radius,
        "height": height,
        "axis": axis,
        "source_prim_name": prim_name,
    }

    return GeometryData(
        geom_type="mesh",
        source_prim=prim,
        mesh_prim=None,
        original_type="Cone",
        original_params=breadcrumb,
    )


def _read_mesh(prim: Usd.Prim) -> GeometryData:
    """Read mesh geometry reference from a UsdGeomMesh.

    ``mesh_scale`` is left as ``None`` here. The exporter's orchestrator
    fills it in (when needed) from the composed mesh-to-link transform,
    which captures scale authored on the Mesh prim itself *or* on any
    ancestor Xform between the link and the mesh. Reading scale only
    from the Mesh prim's local xformOps misses the latter (common in
    Isaac Sim assets where ``<link>/geometry/<Mesh>`` carries scale on
    the intermediate ``geometry`` Xform).
    """
    return GeometryData(geom_type="mesh", source_prim=prim, mesh_prim=prim)


def _get_scale(prim: Usd.Prim) -> tuple[float, float, float]:
    """Extract scale from a prim's XformOps."""
    xformable = UsdGeom.Xformable(prim)
    if not xformable:
        return (1.0, 1.0, 1.0)

    for op in xformable.GetOrderedXformOps():
        if op.GetOpType() == UsdGeom.XformOp.TypeScale:
            scale_val = op.Get()
            if scale_val is not None:
                return (float(scale_val[0]), float(scale_val[1]), float(scale_val[2]))

    return (1.0, 1.0, 1.0)
