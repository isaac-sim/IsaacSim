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
"""Export UsdGeomMesh prims to OBJ files for URDF mesh references."""

from __future__ import annotations

import logging
import math
import os
from typing import IO

from pxr import Gf, Usd, UsdGeom, UsdShade

from .transform_utils import get_prim_name, linear_to_srgb

_logger = logging.getLogger(__name__)


class MeshExporter:
    """Exports UsdGeomMesh prims to OBJ files, deduplicating by content hash.

    Args:
        mesh_dir: Directory where mesh files are written.
        mesh_prefix: Path prefix written into URDF mesh references.
    """

    def __init__(self, mesh_dir: str, mesh_prefix: str = "./") -> None:
        self._mesh_dir = mesh_dir
        self._mesh_prefix = mesh_prefix
        self._exported_by_path: dict[str, str] = {}
        self._exported_by_hash: dict[str, str] = {}
        self._exported_by_proto_xf: dict[tuple[str, tuple[float, ...]], str] = {}

    def export_mesh(self, prim: Usd.Prim, bake_transform: Gf.Matrix4d | None = None) -> str:
        """Export a UsdGeomMesh prim to an OBJ file.

        Args:
            prim: USD prim to read.
            bake_transform: Optional transform to bake into exported geometry.

        Returns:
            URDF-relative mesh filename.
        """
        prim_path = str(prim.GetPath())

        if prim_path in self._exported_by_path:
            return self._exported_by_path[prim_path]

        proto_xf_key = _make_proto_xf_key(prim, bake_transform)
        if proto_xf_key is not None and proto_xf_key in self._exported_by_proto_xf:
            urdf_ref = self._exported_by_proto_xf[proto_xf_key]
            self._exported_by_path[prim_path] = urdf_ref
            return urdf_ref

        if bake_transform is None:
            mesh_hash = _compute_mesh_hash(prim)
            if mesh_hash and mesh_hash in self._exported_by_hash:
                urdf_ref = self._exported_by_hash[mesh_hash]
                self._exported_by_path[prim_path] = urdf_ref
                return urdf_ref
        else:
            mesh_hash = None

        mesh_name = _sanitize_filename(get_prim_name(prim))
        obj_filename = self._unique_filename(mesh_name, ".obj")
        obj_path = os.path.join(self._mesh_dir, obj_filename)

        os.makedirs(self._mesh_dir, exist_ok=True)

        _write_obj(prim, obj_path, bake_transform)

        urdf_ref = self._mesh_prefix + obj_filename
        self._exported_by_path[prim_path] = urdf_ref
        if proto_xf_key is not None:
            self._exported_by_proto_xf[proto_xf_key] = urdf_ref
        if mesh_hash:
            self._exported_by_hash[mesh_hash] = urdf_ref
        return urdf_ref

    def export_cone(self, name: str, radius: float, height: float, axis: str = "Z", segments: int = 32) -> str:
        """Procedurally tessellate a cone and write it to an OBJ file.

        Args:
            name: Base filename for the OBJ.
            radius: Base radius of the cone.
            height: Total height of the cone.
            axis: Principal axis ("X", "Y", or "Z").
            segments: Number of circumferential segments.

        Returns:
            URDF-relative filename (with prefix).

        """
        obj_filename = self._unique_filename(_sanitize_filename(name), ".obj")
        obj_path = os.path.join(self._mesh_dir, obj_filename)
        os.makedirs(self._mesh_dir, exist_ok=True)
        _write_cone_obj(obj_path, radius, height, axis, segments)
        return self._mesh_prefix + obj_filename

    def _unique_filename(self, base: str, ext: str) -> str:
        candidate = f"{base}{ext}"
        counter = 1
        while os.path.exists(os.path.join(self._mesh_dir, candidate)):
            candidate = f"{base}_{counter}{ext}"
            counter += 1
        return candidate


def _write_cone_obj(obj_path: str, radius: float, height: float, axis: str = "Z", segments: int = 32) -> None:
    """Write a procedurally generated cone mesh to an OBJ file.

    The cone is centred at the origin with its principal axis along *axis*.
    Apex is at +height/2, base circle at -height/2.

    Args:
        obj_path: OBJ output file path.
        radius: Cone radius.
        height: Cone height.
        axis: Principal axis token.
        segments: Number of cone segments.
    """
    half_h = height / 2.0

    # Generate vertices in Z-up, remap at write time
    def _remap(x: float, y: float, z: float) -> tuple[float, float, float]:
        if axis == "X":
            return (z, x, y)
        elif axis == "Y":
            return (x, z, y)
        return (x, y, z)

    apex = _remap(0.0, 0.0, half_h)
    center = _remap(0.0, 0.0, -half_h)

    base_verts = []
    for i in range(segments):
        theta = 2.0 * math.pi * i / segments
        bx = radius * math.cos(theta)
        by = radius * math.sin(theta)
        base_verts.append(_remap(bx, by, -half_h))

    with open(obj_path, "w") as f:
        f.write("# Procedural cone mesh\n")
        f.write(f"v {apex[0]:.8f} {apex[1]:.8f} {apex[2]:.8f}\n")
        f.write(f"v {center[0]:.8f} {center[1]:.8f} {center[2]:.8f}\n")
        for v in base_verts:
            f.write(f"v {v[0]:.8f} {v[1]:.8f} {v[2]:.8f}\n")

        # Vertex indices: 1 = apex, 2 = center, 3..3+N-1 = base ring
        for i in range(segments):
            bi = i + 3
            bn = (i + 1) % segments + 3
            # Side face (apex, base[i], base[i+1])
            f.write(f"f 1 {bi} {bn}\n")
            # Base face (center, base[i+1], base[i]) — reversed for outward normal
            f.write(f"f 2 {bn} {bi}\n")


def _resolve_prototype_path(prim: Usd.Prim) -> str | None:
    """Return the prototype prim path for an instance proxy, or None.

    Walks up the prim tree via :meth:`Usd.Prim.GetParent` because
    ``Usd.Prim`` does not expose ``GetAncestorsRange`` in the Python
    bindings (only the C++ API does).

    Args:
        prim: USD prim to read.

    Returns:
        Prototype prim path, or None if the prim is not instanced.
    """
    if prim.IsInstanceProxy():
        proto = prim.GetPrimInPrototype()
        if proto and proto.IsValid():
            return str(proto.GetPath())
    ancestor = prim.GetParent()
    while ancestor and ancestor.IsValid() and not ancestor.IsPseudoRoot():
        if ancestor.IsInstanceProxy():
            proto = ancestor.GetPrimInPrototype()
            if proto and proto.IsValid():
                rel = prim.GetPath().MakeRelativePath(ancestor.GetPath())
                return str(proto.GetPath().AppendPath(rel))
        ancestor = ancestor.GetParent()
    return None


def _matrix4d_to_tuple(mat: Gf.Matrix4d) -> tuple[float, ...]:
    """Convert a Gf.Matrix4d to a hashable tuple of rounded floats.

    Precision of 6 decimals (~micrometer) is sufficient for dedup while
    absorbing floating-point noise from composed world-transform chains.

    Args:
        mat: Matrix to convert or material data to write.

    Returns:
        Hashable matrix tuple.
    """
    return tuple(round(mat[r][c], 6) for r in range(4) for c in range(4))


def _make_proto_xf_key(prim: Usd.Prim, bake_transform: Gf.Matrix4d | None) -> tuple[str, tuple[float, ...]] | None:
    """Build a dedup key from (prototype_path, bake_transform) for instances.

    Returns None for non-instanced prims (they use prim-path dedup instead).

    Args:
        prim: USD prim to read.
        bake_transform: Optional transform to bake into exported geometry.

    Returns:
        Prototype transform key, or None for non-instanced prims.
    """
    proto_path = _resolve_prototype_path(prim)
    if proto_path is None:
        return None
    xf_tuple = _matrix4d_to_tuple(bake_transform) if bake_transform is not None else ()
    return (proto_path, xf_tuple)


def _compute_mesh_hash(prim: Usd.Prim) -> str | None:
    """Compute a content hash for a mesh prim based on topology.

    Hashes the points array and face vertex indices to identify
    identical mesh content across different prim paths (instances).

    Args:
        prim: USD prim to read.

    Returns:
        Mesh content hash, or None if no mesh data exists.
    """
    import hashlib

    meshes = []
    if prim.IsA(UsdGeom.Mesh):
        meshes.append(prim)
    else:
        for child in Usd.PrimRange(prim):
            if child.IsA(UsdGeom.Mesh):
                meshes.append(child)

    if not meshes:
        return None

    h = hashlib.sha256()
    for mesh_prim in meshes:
        mesh = UsdGeom.Mesh(mesh_prim)
        points = mesh.GetPointsAttr().Get()
        face_indices = mesh.GetFaceVertexIndicesAttr().Get()
        face_counts = mesh.GetFaceVertexCountsAttr().Get()

        if points is not None:
            for p in points:
                h.update(f"{p[0]:.6f},{p[1]:.6f},{p[2]:.6f}|".encode())
        if face_indices is not None:
            for fi in face_indices:
                h.update(f"{fi},".encode())
        if face_counts is not None:
            for fc in face_counts:
                h.update(f"{fc};".encode())

    return h.hexdigest()


def _write_obj(prim: Usd.Prim, obj_path: str, bake_transform: Gf.Matrix4d | None = None) -> None:
    """Write a UsdGeomMesh to a Wavefront OBJ file with MTL sidecar.

    Handles UsdGeomSubset children for per-face-group material assignment.
    If bake_transform is provided, all vertices and normals are transformed
    by it so the OBJ contains geometry in the URDF link frame.

    Args:
        prim: USD prim to read.
        obj_path: OBJ output file path.
        bake_transform: Optional transform to bake into exported geometry.
    """
    meshes = []
    if prim.IsA(UsdGeom.Mesh):
        meshes.append(prim)
    else:
        for child in Usd.PrimRange(prim):
            if child.IsA(UsdGeom.Mesh):
                meshes.append(child)

    if not meshes:
        _logger.warning(f"No mesh data found under {prim.GetPath()}")
        return

    mtl_basename = os.path.splitext(os.path.basename(obj_path))[0] + ".mtl"
    mtl_path = os.path.join(os.path.dirname(obj_path), mtl_basename)
    materials_to_write: dict[str, _MtlData] = {}

    with open(obj_path, "w") as f:
        f.write(f"# Exported from USD: {prim.GetPath()}\n")
        f.write(f"mtllib {mtl_basename}\n\n")

        vertex_offset = 0
        texcoord_offset = 0

        for mesh_prim in meshes:
            mesh = UsdGeom.Mesh(mesh_prim)
            group_name = mesh_prim.GetName()
            f.write(f"g {group_name}\n")

            points = mesh.GetPointsAttr().Get()
            face_counts = mesh.GetFaceVertexCountsAttr().Get()
            face_indices = mesh.GetFaceVertexIndicesAttr().Get()

            if points is None or face_counts is None or face_indices is None:
                _logger.warning(f"Mesh {mesh_prim.GetPath()} has incomplete topology")
                continue

            if bake_transform is not None:
                for p in points:
                    tp = bake_transform.Transform(Gf.Vec3d(float(p[0]), float(p[1]), float(p[2])))
                    f.write(f"v {tp[0]:.8f} {tp[1]:.8f} {tp[2]:.8f}\n")
            else:
                for p in points:
                    f.write(f"v {p[0]:.8f} {p[1]:.8f} {p[2]:.8f}\n")

            texcoords, tc_indices, tc_interp = _get_texcoords_full(mesh_prim)
            has_texcoords = texcoords is not None and len(texcoords) > 0
            if has_texcoords:
                for tc in texcoords:
                    f.write(f"vt {tc[0]:.8f} {tc[1]:.8f}\n")

            face_data = _FaceIndexData(
                face_counts=face_counts,
                face_indices=face_indices,
                v_off=vertex_offset,
                n_off=0,
                t_off=texcoord_offset,
                has_n=False,
                has_t=has_texcoords,
                tc_interp=tc_interp,
                tc_indices=tc_indices,
            )

            subsets = _get_geom_subsets(mesh_prim)

            if subsets:
                _write_faces_with_subsets(
                    f,
                    face_data,
                    subsets,
                    mesh_prim,
                    materials_to_write,
                )
            else:
                mat_name = _collect_material(mesh_prim, materials_to_write)
                if mat_name:
                    f.write(f"usemtl {mat_name}\n")

                _write_faces(f, face_data)

            vertex_offset += len(points)
            if has_texcoords:
                texcoord_offset += len(texcoords)

    if materials_to_write:
        _write_mtl(mtl_path, materials_to_write)


# --- Face writing ---


from dataclasses import dataclass as _dataclass


@_dataclass
class _FaceIndexData:
    """Collected indexing data for face writing."""

    face_counts: list
    face_indices: list
    v_off: int
    n_off: int
    t_off: int
    has_n: bool
    has_t: bool
    normal_interp: str = "faceVarying"
    tc_interp: str = "faceVarying"
    tc_indices: list | None = None


def _write_faces(f: IO[str], fd: _FaceIndexData) -> None:
    """Write face definitions for all faces.

    Args:
        f: Open OBJ file handle.
        fd: Face index data to write.
    """
    idx = 0
    for count in fd.face_counts:
        f.write("f")
        for _ in range(count):
            _write_face_vertex(f, fd, idx)
            idx += 1
        f.write("\n")


def _write_faces_with_subsets(
    f: IO[str],
    fd: _FaceIndexData,
    subsets: list[tuple[Usd.Prim, list[int]]],
    mesh_prim: Usd.Prim,
    materials_dict: dict[str, _MtlData],
) -> None:
    """Write faces grouped by GeomSubset material assignments.

    Args:
        f: Open OBJ file handle.
        fd: Face index data to write.
        subsets: GeomSubset face assignments.
        mesh_prim: Mesh prim to inspect.
        materials_dict: Material dictionary to update.
    """
    face_to_subset_mat: dict[int, str] = {}
    for subset_prim, subset_indices in subsets:
        mat_name = _collect_material(subset_prim, materials_dict)
        if mat_name:
            for fi in subset_indices:
                face_to_subset_mat[int(fi)] = mat_name

    mesh_mat = _collect_material(mesh_prim, materials_dict)

    current_mat = None
    idx = 0
    for face_idx, count in enumerate(fd.face_counts):
        mat = face_to_subset_mat.get(face_idx, mesh_mat)
        if mat and mat != current_mat:
            f.write(f"usemtl {mat}\n")
            current_mat = mat

        f.write("f")
        for _ in range(count):
            _write_face_vertex(f, fd, idx)
            idx += 1
        f.write("\n")


def _write_face_vertex(f: IO[str], fd: _FaceIndexData, idx: int) -> None:
    """Write a single face vertex reference with correct indexing.

    Handles vertex, faceVarying, and indexed primvar interpolation modes:
    - vertex: use the same index as the position vertex (face_indices[idx])
    - faceVarying: use the running face-vertex index (idx)
    - indexed faceVarying: use the primvar's own index array

    Args:
        f: Open OBJ file handle.
        fd: Face index data to write.
        idx: Face-vertex index to write.
    """
    vi = fd.face_indices[idx] + 1 + fd.v_off

    if fd.has_n:
        if fd.normal_interp == "vertex":
            ni = fd.face_indices[idx] + 1 + fd.n_off
        else:
            ni = idx + 1 + fd.n_off
    else:
        ni = None

    if fd.has_t:
        if fd.tc_indices is not None:
            ti = fd.tc_indices[idx] + 1 + fd.t_off
        elif fd.tc_interp == "vertex":
            ti = fd.face_indices[idx] + 1 + fd.t_off
        else:
            ti = idx + 1 + fd.t_off
    else:
        ti = None

    if ni is not None and ti is not None:
        f.write(f" {vi}/{ti}/{ni}")
    elif ni is not None:
        f.write(f" {vi}//{ni}")
    elif ti is not None:
        f.write(f" {vi}/{ti}")
    else:
        f.write(f" {vi}")


# --- GeomSubset handling ---


def _get_geom_subsets(mesh_prim: Usd.Prim) -> list[tuple[Usd.Prim, list[int]]]:
    """Get GeomSubset children with their face indices.

    Args:
        mesh_prim: Mesh prim to inspect.

    Returns:
        GeomSubset children and face indices.
    """
    subsets = []
    for child in mesh_prim.GetChildren():
        if not child.IsA(UsdGeom.Subset):
            continue
        subset = UsdGeom.Subset(child)
        indices_attr = subset.GetIndicesAttr()
        if not indices_attr:
            continue
        indices = indices_attr.Get()
        if indices is None:
            continue
        subsets.append((child, list(indices)))
    return subsets


# --- Material reading ---


class _MtlData:
    """Material data for MTL file output."""

    def __init__(self) -> None:
        self.kd: tuple[float, float, float] = (0.8, 0.8, 0.8)
        self.ks: tuple[float, float, float] = (0.0, 0.0, 0.0)
        self.ke: tuple[float, float, float] = (0.0, 0.0, 0.0)
        self.ns: float = 0.0
        self.metallic: float = 0.0
        self.roughness: float = 0.5
        self.opacity: float = 1.0
        self.texture_file: str | None = None


def _collect_material(prim: Usd.Prim, materials_dict: dict[str, _MtlData]) -> str | None:
    """Read material from a prim's binding and add to the materials dict.

    Searches the prim itself, then walks up ancestors to find inherited
    bindings. Handles instance proxies by also checking the non-instanced
    hierarchy.

    Args:
        prim: USD prim to read.
        materials_dict: Material dictionary to update.

    Returns:
        Material name, or None if no material is bound.
    """
    material = _find_bound_material(prim)
    if not material:
        return None

    mat_name = _sanitize_filename(get_prim_name(material.GetPrim()))

    if mat_name not in materials_dict:
        mtl_data = _read_material_data(material)
        materials_dict[mat_name] = mtl_data

    return mat_name


def _find_bound_material(prim: Usd.Prim) -> UsdShade.Material | None:
    """Find the bound material for a prim, handling instance proxies.

    Tries ComputeBoundMaterial first. If that fails (common with instance
    proxies), walks up the prim hierarchy checking each ancestor. Also
    checks material:binding relationships directly for MDL materials
    that may not be found by ComputeBoundMaterial.

    Args:
        prim: USD prim to read.

    Returns:
        Bound material, or None if absent.
    """
    binding_api = UsdShade.MaterialBindingAPI(prim)
    if binding_api:
        bound = binding_api.ComputeBoundMaterial()
        if bound and bound[0]:
            return bound[0]

    current = prim
    while current and current.IsValid() and not current.IsPseudoRoot():
        rel = current.GetRelationship("material:binding")
        if rel and rel.IsValid():
            targets = rel.GetTargets()
            if targets:
                mat_prim = current.GetStage().GetPrimAtPath(targets[0])
                if mat_prim and mat_prim.IsValid():
                    mat = UsdShade.Material(mat_prim)
                    if mat:
                        return mat

        current = current.GetParent()

    return None


def _read_material_data(material: UsdShade.Material) -> _MtlData:
    """Extract diffuse color, opacity, and texture from a UsdShadeMaterial.

    Supports multiple shader types by scanning all child shader prims
    for known diffuse color inputs. Works with UsdPreviewSurface, OmniPBR,
    and other MDL shaders.

    Args:
        material: USD shade material to read.

    Returns:
        Material data for MTL output.
    """
    data = _MtlData()

    mat_prim = material.GetPrim()

    # Strategy 1: scan all child shader prims for known color inputs
    for child in mat_prim.GetChildren():
        shader = UsdShade.Shader(child)
        if not shader:
            continue
        if _read_shader_color(shader, data):
            return data

    # Strategy 2: follow surface output connections
    for render_context in ("", "mdl", "ri"):
        surface_output = material.GetSurfaceOutput(render_context)
        if not surface_output:
            continue
        sources = _get_sources(surface_output)
        for src in sources or []:
            if not src.source:
                continue
            shader = UsdShade.Shader(src.source.GetPrim())
            if shader and _read_shader_color(shader, data):
                return data

    return data


def _read_shader_color(shader: UsdShade.Shader, data: _MtlData) -> bool:
    """Try to read diffuse color from a shader using known input names.

    Returns True if a color was found.

    Args:
        shader: USD shader to read.
        data: Material data to populate.

    Returns:
        True if a diffuse color was found, False otherwise.
    """
    _COLOR_INPUTS = [
        "diffuse_color_constant",
        "diffuseColor",
        "base_color",
        "albedo",
        "BaseColor",
    ]
    _OPACITY_INPUTS = ["opacity_constant", "opacity", "enable_opacity"]
    _TEXTURE_INPUTS = ["diffuse_texture", "albedo_texture"]

    found_color = False

    for name in _COLOR_INPUTS:
        inp = shader.GetInput(name)
        if not inp:
            continue
        # Check for texture connection first
        conn_sources = _get_sources(inp)
        if conn_sources:
            for cs in conn_sources:
                if cs.source:
                    tex_shader = UsdShade.Shader(cs.source.GetPrim())
                    if tex_shader:
                        _read_texture_from_shader(tex_shader, data)
                        fallback = tex_shader.GetInput("fallback")
                        if fallback and fallback.Get() is not None:
                            fb = fallback.Get()
                            if hasattr(fb, "__len__") and len(fb) >= 3:
                                data.kd = _linear_color_to_srgb(fb)
                                found_color = True
            if found_color:
                break
            continue

        val = inp.Get()
        if val is not None and hasattr(val, "__len__") and len(val) >= 3:
            data.kd = _linear_color_to_srgb(val)
            found_color = True
            break

    for name in _OPACITY_INPUTS:
        inp = shader.GetInput(name)
        if inp and inp.Get() is not None:
            val = inp.Get()
            if isinstance(val, (int, float)):
                data.opacity = float(val)
            break

    for name in _TEXTURE_INPUTS:
        inp = shader.GetInput(name)
        if inp and inp.Get() is not None:
            val = inp.Get()
            from pxr import Sdf as _Sdf

            if isinstance(val, _Sdf.AssetPath):
                resolved = val.resolvedPath or val.path
                if resolved:
                    data.texture_file = resolved

    _METALLIC_INPUTS = ["metallic_constant", "metallic"]
    for name in _METALLIC_INPUTS:
        inp = shader.GetInput(name)
        if inp and inp.Get() is not None:
            val = inp.Get()
            if isinstance(val, (int, float)):
                data.metallic = float(val)
                m = data.metallic
                data.ks = (m, m, m)
                data.ns = (1.0 - data.roughness) * 1000.0
            break

    _ROUGHNESS_INPUTS = ["reflection_roughness_constant", "roughness"]
    for name in _ROUGHNESS_INPUTS:
        inp = shader.GetInput(name)
        if inp and inp.Get() is not None:
            val = inp.Get()
            if isinstance(val, (int, float)):
                data.roughness = float(val)
                data.ns = (1.0 - data.roughness) * 1000.0
            break

    _EMISSIVE_COLOR_INPUTS = ["emissive_color", "emissiveColor"]
    for name in _EMISSIVE_COLOR_INPUTS:
        inp = shader.GetInput(name)
        if inp and inp.Get() is not None:
            val = inp.Get()
            if hasattr(val, "__len__") and len(val) >= 3:
                data.ke = _linear_color_to_srgb(val)
            break

    _EMISSIVE_INTENSITY_INPUTS = ["emissive_intensity"]
    for name in _EMISSIVE_INTENSITY_INPUTS:
        inp = shader.GetInput(name)
        if inp and inp.Get() is not None:
            val = float(inp.Get())
            if val > 0 and data.ke == (0.0, 0.0, 0.0):
                data.ke = _linear_color_to_srgb((val, val, val))
            break

    return found_color


def _linear_color_to_srgb(val: tuple[float, ...] | list[float]) -> tuple[float, float, float]:
    """Convert a linear RGB color to sRGB for MTL output.

    All USD shader color values (UsdPreviewSurface, OmniPBR, etc.) are
    stored in linear space. MTL Kd values are interpreted as sRGB.

    Args:
        val: Linear color values.

    Returns:
        sRGB color values.
    """
    return (
        linear_to_srgb(float(val[0])),
        linear_to_srgb(float(val[1])),
        linear_to_srgb(float(val[2])),
    )


def _read_texture_from_shader(shader: UsdShade.Shader, data: _MtlData) -> None:
    """Read texture file path from a texture shader node.

    Args:
        shader: USD shader to read.
        data: Material data to populate.
    """
    file_input = shader.GetInput("file")
    if not file_input:
        return
    val = file_input.Get()
    if not val:
        return
    from pxr import Sdf as _Sdf

    if isinstance(val, _Sdf.AssetPath):
        resolved = val.resolvedPath or val.path
        if resolved:
            data.texture_file = resolved
    elif val:
        data.texture_file = str(val)


def _get_sources(connectable: UsdShade.ConnectableAPI) -> list:
    """Safely extract the sources list from GetConnectedSources().

    Args:
        connectable: Connectable USD shading object.

    Returns:
        Connected source information.
    """
    result = connectable.GetConnectedSources()
    if not result:
        return []
    if isinstance(result, tuple):
        return result[0] if result[0] else []
    return result


def _get_texcoords(prim: Usd.Prim) -> object | None:
    """Read 'st' primvar (texture coordinates) from a mesh prim.

    Args:
        prim: USD prim to read.

    Returns:
        Texture coordinate values, or None if absent.
    """
    primvar_api = UsdGeom.PrimvarsAPI(prim)
    st_primvar = primvar_api.GetPrimvar("st")
    if st_primvar and st_primvar.IsDefined():
        return st_primvar.Get()
    return None


def _get_texcoords_full(prim: Usd.Prim) -> tuple:
    """Read 'st' primvar with interpolation mode and optional indices.

    Args:
        prim: USD prim to read.

    Returns:
        Texture coordinates, optional indices, and interpolation mode.
    """
    primvar_api = UsdGeom.PrimvarsAPI(prim)
    for name in ("st", "UVMap", "st0", "st_0"):
        st_primvar = primvar_api.GetPrimvar(name)
        if st_primvar and st_primvar.IsDefined():
            values = st_primvar.Get()
            if values is None or len(values) == 0:
                continue
            interp = str(st_primvar.GetInterpolation())
            indices = None
            if st_primvar.IsIndexed():
                indices = list(st_primvar.GetIndices())
            return values, indices, interp
    return None, None, "faceVarying"


# --- MTL output ---


def _write_mtl(mtl_path: str, materials: dict[str, _MtlData]) -> None:
    """Write a Wavefront MTL file with PBR-lite values.

    Maps USD/OmniPBR properties to MTL:
    - Kd: diffuse color (sRGB)
    - Ks: specular color (from metallic)
    - Ke: emissive color (sRGB)
    - Ns: specular exponent (from 1-roughness)
    - d: opacity
    - map_Kd: diffuse texture

    Args:
        mtl_path: MTL output file path.
        materials: Material data to write.
    """
    with open(mtl_path, "w") as f:
        f.write("# Exported from USD\n\n")
        for mat_name, data in materials.items():
            f.write(f"newmtl {mat_name}\n")
            f.write(f"Kd {data.kd[0]:.6f} {data.kd[1]:.6f} {data.kd[2]:.6f}\n")
            f.write("Ka 0.000000 0.000000 0.000000\n")
            f.write(f"Ks {data.ks[0]:.6f} {data.ks[1]:.6f} {data.ks[2]:.6f}\n")
            f.write(f"Ns {data.ns:.6f}\n")
            f.write(f"d {data.opacity:.6f}\n")
            if data.ke != (0.0, 0.0, 0.0):
                f.write(f"Ke {data.ke[0]:.6f} {data.ke[1]:.6f} {data.ke[2]:.6f}\n")
            f.write("illum 2\n")
            if data.texture_file:
                basename = os.path.basename(data.texture_file)
                f.write(f"map_Kd {basename}\n")
            f.write("\n")


def _sanitize_filename(name: str) -> str:
    """Sanitize a string for use as a filename.

    Args:
        name: Name to use.

    Returns:
        Sanitized filename.
    """
    return "".join(c if c.isalnum() or c in ("_", "-", ".") else "_" for c in name)
