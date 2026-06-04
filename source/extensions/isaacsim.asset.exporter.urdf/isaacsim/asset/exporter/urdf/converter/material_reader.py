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
"""Read material data from USD UsdPreviewSurface for URDF export."""

from __future__ import annotations

import logging
import os
import shutil
from dataclasses import dataclass
from typing import Any

from pxr import Sdf, Usd, UsdShade

from .transform_utils import get_prim_name, linear_to_srgb

_logger = logging.getLogger(__name__)


def _get_sources(connectable: Any) -> list:
    """Safely extract the sources list from GetConnectedSources().

    GetConnectedSources() returns (list[ConnectionSourceInfo], list[SdfPath]).
    """
    result = connectable.GetConnectedSources()
    if not result:
        return []
    if isinstance(result, tuple):
        return result[0] if result[0] else []
    return result


@dataclass
class MaterialData:
    """URDF material element data."""

    name: str = ""
    color_rgba: tuple[float, float, float, float] | None = None
    texture_filename: str | None = None


def collect_materials(links_data: list[Any], output_dir: str | None = None) -> list[MaterialData]:
    """Collect all unique materials referenced by link visuals.

    Args:
        links_data: List of LinkData objects.
        output_dir: If provided, texture files are copied here.

    Returns:
        List of unique MaterialData.
    """
    seen: dict[str, MaterialData] = {}

    for link in links_data:
        for visual in link.visuals:
            if visual.material_name and visual.material_name not in seen:
                mat_data = MaterialData(name=visual.material_name)
                seen[visual.material_name] = mat_data

    return list(seen.values())


def read_material_from_prim(prim: Usd.Prim, output_dir: str | None = None) -> MaterialData | None:
    """Read material data from a geometry prim's material binding.

    Args:
        prim: Geometry prim that may have a bound material.
        output_dir: Directory for copying texture files.

    Returns:
        MaterialData or None.
    """
    binding_api = UsdShade.MaterialBindingAPI(prim)
    if not binding_api:
        return None

    bound = binding_api.ComputeBoundMaterial()
    if not bound or not bound[0]:
        return None

    material = bound[0]
    return read_material(material, output_dir)


def read_material(material: UsdShade.Material, output_dir: str | None = None) -> MaterialData | None:
    """Read material data from a UsdShadeMaterial.

    Args:
        material: USD shade material.
        output_dir: Directory for copying texture files.

    Returns:
        MaterialData with color and/or texture info.
    """
    mat_prim = material.GetPrim()
    data = MaterialData(name=get_prim_name(mat_prim))

    surface_output = material.GetSurfaceOutput()
    if not surface_output:
        return data

    sources = _get_sources(surface_output)
    if not sources:
        return data

    for source_info in sources:
        if not source_info.source:
            continue
        shader = UsdShade.Shader(source_info.source.GetPrim())
        if not shader:
            continue

        shader_id = shader.GetIdAttr().Get()
        if shader_id == "UsdPreviewSurface":
            _read_preview_surface(shader, data, output_dir)
            break

    return data


def _read_preview_surface(shader: UsdShade.Shader, data: MaterialData, output_dir: str | None) -> None:
    """Extract color and texture from a UsdPreviewSurface shader."""
    diffuse_input = shader.GetInput("diffuseColor")
    if diffuse_input:
        sources = _get_sources(diffuse_input)
        if sources:
            for src in sources:
                if src.source:
                    tex_shader = UsdShade.Shader(src.source.GetPrim())
                    _read_texture_shader(tex_shader, data, output_dir)
        else:
            val = diffuse_input.Get()
            if val is not None:
                r = linear_to_srgb(float(val[0]))
                g = linear_to_srgb(float(val[1]))
                b = linear_to_srgb(float(val[2]))
                opacity = _read_opacity(shader)
                data.color_rgba = (r, g, b, opacity)

    if data.color_rgba is None:
        opacity = _read_opacity(shader)
        data.color_rgba = (1.0, 1.0, 1.0, opacity)


def _read_opacity(shader: UsdShade.Shader) -> float:
    """Read opacity from a UsdPreviewSurface shader."""
    opacity_input = shader.GetInput("opacity")
    if opacity_input:
        val = opacity_input.Get()
        if val is not None:
            return float(val)
    return 1.0


def _read_texture_shader(shader: UsdShade.Shader, data: MaterialData, output_dir: str | None) -> None:
    """Read texture file path from a UsdUVTexture or image shader."""
    if not shader:
        return

    file_input = shader.GetInput("file")
    if not file_input:
        return

    val = file_input.Get()
    if val is None:
        return

    if isinstance(val, Sdf.AssetPath):
        resolved = val.resolvedPath or val.path
    else:
        resolved = str(val)

    if not resolved:
        return

    if output_dir and os.path.isfile(resolved):
        dest = os.path.join(output_dir, os.path.basename(resolved))
        if not os.path.exists(dest):
            try:
                shutil.copy2(resolved, dest)
            except OSError:
                _logger.warning(f"Failed to copy texture {resolved} to {dest}")
        data.texture_filename = os.path.basename(resolved)
    else:
        data.texture_filename = resolved

    fallback_input = shader.GetInput("fallback")
    if fallback_input:
        fb = fallback_input.Get()
        if fb is not None and len(fb) >= 3:
            r = linear_to_srgb(float(fb[0]))
            g = linear_to_srgb(float(fb[1]))
            b = linear_to_srgb(float(fb[2]))
            a = float(fb[3]) if len(fb) > 3 else 1.0
            data.color_rgba = (r, g, b, a)


def populate_material_colors(materials: list[MaterialData], stage: Usd.Stage, output_dir: str | None = None) -> None:
    """Fill in color/texture data for materials by finding them on the stage.

    Args:
        materials: List of MaterialData (with name set but color/texture empty).
        stage: USD stage to search for material prims.
        output_dir: Directory for copying texture files.
    """
    mat_by_name: dict[str, MaterialData] = {m.name: m for m in materials}
    needs_fill = {m.name for m in materials if m.color_rgba is None and m.texture_filename is None}

    if not needs_fill:
        return

    for prim in stage.Traverse():
        if not prim.IsA(UsdShade.Material):
            continue
        name = get_prim_name(prim)
        if name not in needs_fill:
            continue

        material = UsdShade.Material(prim)
        filled = read_material(material, output_dir)
        if filled:
            target = mat_by_name[name]
            target.color_rgba = filled.color_rgba
            target.texture_filename = filled.texture_filename
            needs_fill.discard(name)
            if not needs_fill:
                break
