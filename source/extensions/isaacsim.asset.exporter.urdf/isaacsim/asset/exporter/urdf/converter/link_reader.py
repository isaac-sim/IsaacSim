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
"""Read link (rigid body) data from USD prims for URDF export."""

from __future__ import annotations

from collections.abc import Generator
from dataclasses import dataclass, field

from pxr import Usd, UsdGeom, UsdPhysics, UsdShade

from .geometry_reader import GeometryData, read_geometry
from .inertia_utils import InertiaData, read_inertial_from_prim
from .transform_utils import get_prim_name, matrix4_to_origin

_GEOM_TYPES = {"Mesh", "Cube", "Sphere", "Capsule", "Cylinder", "Cone"}


@dataclass
class VisualData:
    """A single visual element for a URDF link."""

    name: str = ""
    origin_xyz: tuple[float, float, float] = (0.0, 0.0, 0.0)
    origin_rpy: tuple[float, float, float] = (0.0, 0.0, 0.0)
    geometry: GeometryData | None = None
    material_name: str | None = None


@dataclass
class CollisionData:
    """A single collision element for a URDF link."""

    name: str = ""
    origin_xyz: tuple[float, float, float] = (0.0, 0.0, 0.0)
    origin_rpy: tuple[float, float, float] = (0.0, 0.0, 0.0)
    geometry: GeometryData | None = None


@dataclass
class LinkData:
    """Complete data for a URDF link."""

    name: str = ""
    prim_path: str = ""
    inertial: InertiaData | None = None
    visuals: list[VisualData] = field(default_factory=list)
    collisions: list[CollisionData] = field(default_factory=list)


def read_link(prim: Usd.Prim) -> LinkData:
    """Read all URDF link data from a rigid body prim.

    Classifies children as visuals or collisions based on CollisionAPI
    and purpose attributes.  Geometry origins are set to identity here;
    they are recomputed by the orchestrator using URDF frames.

    Args:
        prim: USD prim with RigidBodyAPI.

    Returns:
        LinkData with inertial, visuals, and collisions populated.

    """
    link = LinkData(name=get_prim_name(prim), prim_path=str(prim.GetPath()))
    link.inertial = read_inertial_from_prim(prim)

    for child in _iter_geometry_children(prim):
        is_collision = _is_collision_prim(child)
        is_visual = _is_visual_prim(child)

        geom_list = read_geometry(child)
        if not geom_list:
            continue

        base_name = get_prim_name(child)
        origin_xyz = (0.0, 0.0, 0.0)
        origin_rpy = (0.0, 0.0, 0.0)

        for geom in geom_list:
            child_name = base_name + geom.name_suffix

            if is_collision:
                link.collisions.append(
                    CollisionData(
                        name=child_name,
                        origin_xyz=origin_xyz,
                        origin_rpy=origin_rpy,
                        geometry=geom,
                    )
                )

            if is_visual:
                mat_name = _get_bound_material_name(child)
                link.visuals.append(
                    VisualData(
                        name=child_name,
                        origin_xyz=origin_xyz,
                        origin_rpy=origin_rpy,
                        geometry=geom,
                        material_name=mat_name,
                    )
                )

            if not is_collision and not is_visual:
                mat_name = _get_bound_material_name(child)
                link.visuals.append(
                    VisualData(
                        name=child_name,
                        origin_xyz=origin_xyz,
                        origin_rpy=origin_rpy,
                        geometry=geom,
                        material_name=mat_name,
                    )
                )

    return link


def _iter_geometry_children(prim: Usd.Prim) -> Generator[Usd.Prim, None, None]:
    """Yield descendant Gprims that represent geometry, including instance proxies.

    Traverses the full subtree under the rigid body prim using
    Usd.TraverseInstanceProxies so that instanced geometry (common in
    Isaac Sim assets) is found.  Stops descending into child rigid bodies
    and joints to stay within the current link's scope.

    For instance proxies, reads from the prototype to ignore overrides.

    Args:
        prim: USD prim to read.

    Yields:
        Descendant geometry prims.
    """
    instance_pred = Usd.TraverseInstanceProxies()

    for desc in Usd.PrimRange(prim, instance_pred):
        if desc == prim:
            continue
        source = _resolve_prototype(desc)
        if source.HasAPI(UsdPhysics.RigidBodyAPI):
            continue
        if source.IsA(UsdPhysics.Joint):
            continue
        if desc.GetTypeName() in _GEOM_TYPES:
            yield desc


def _resolve_prototype(prim: Usd.Prim) -> Usd.Prim:
    """For instance proxies, return the prototype prim (ignoring overrides).

    Overrides authored on instance proxy paths are erroneous and should
    be ignored. The prototype carries the authoritative data.
    For non-proxy prims, returns the prim itself.

    Args:
        prim: USD prim to read.

    Returns:
        Prototype prim for instance proxies, or the original prim.
    """
    if prim.IsInstanceProxy():
        proto = prim.GetPrimInPrototype()
        if proto and proto.IsValid():
            return proto
    return prim


def _is_collision_prim(prim: Usd.Prim) -> bool:
    """Check if prim is collision geometry.

    Reads from the prototype for instance proxies to ignore overrides.

    Args:
        prim: USD prim to read.

    Returns:
        True if the prim is collision geometry, False otherwise.
    """
    source = _resolve_prototype(prim)
    if source.HasAPI(UsdPhysics.CollisionAPI):
        return True
    imageable = UsdGeom.Imageable(source)
    purpose = imageable.GetPurposeAttr().Get()
    return purpose == UsdGeom.Tokens.guide


def _is_visual_prim(prim: Usd.Prim) -> bool:
    """Check if prim is visual geometry (not exclusively collision).

    Reads from the prototype for instance proxies to ignore overrides.

    Args:
        prim: USD prim to read.

    Returns:
        True if the prim is visual geometry, False otherwise.
    """
    source = _resolve_prototype(prim)
    imageable = UsdGeom.Imageable(source)
    purpose = imageable.GetPurposeAttr().Get()
    if purpose == UsdGeom.Tokens.guide:
        return False
    return purpose in (UsdGeom.Tokens.default_, UsdGeom.Tokens.render, None, "")


def _get_child_origin(
    child: Usd.Prim, parent: Usd.Prim
) -> tuple[tuple[float, float, float], tuple[float, float, float]]:
    """Compute the origin of a child geometry prim relative to its link prim.

    Args:
        child: Child prim to evaluate.
        parent: Parent XML element.

    Returns:
        Origin translation and rotation relative to the parent.
    """
    from pxr import Gf, UsdGeom

    xfc = UsdGeom.XformCache()
    child_world = Gf.Matrix4d(xfc.GetLocalToWorldTransform(child))
    parent_world = Gf.Matrix4d(xfc.GetLocalToWorldTransform(parent))

    relative = child_world * parent_world.GetInverse()
    return matrix4_to_origin(relative)


def _get_bound_material_name(prim: Usd.Prim) -> str | None:
    """Get the name of the material bound to a prim, if any.

    Args:
        prim: USD prim to read.

    Returns:
        Bound material name, or None if no material is bound.
    """
    binding_api = UsdShade.MaterialBindingAPI(prim)
    if not binding_api:
        return None
    bound = binding_api.ComputeBoundMaterial()
    if bound and bound[0]:
        mat_prim = bound[0].GetPrim()
        return get_prim_name(mat_prim)
    return None
