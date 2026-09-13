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
import re
import shutil
from dataclasses import dataclass
from typing import Any
from urllib.parse import unquote, urlsplit

from pxr import Sdf, Usd, UsdShade

from .transform_utils import get_prim_name, linear_to_srgb

_logger = logging.getLogger(__name__)

_ENCODED_UDIM_TOKEN = re.compile(r"%3cudim%3e", re.IGNORECASE)


def _texture_identifier(value: Any) -> str:
    """Return the authored texture identifier from a shader input value."""
    if isinstance(value, Sdf.AssetPath):
        return value.path
    return str(value)


def _normalize_texture_identifier(identifier: str) -> str:
    """Normalize encoded UDIM tokens without decoding unrelated URL content."""
    return _ENCODED_UDIM_TOKEN.sub("<UDIM>", identifier)


def texture_reference_filename(value: Any) -> str | None:
    """Return a portable texture filename while preserving a UDIM token."""
    identifier = _normalize_texture_identifier(_texture_identifier(value))
    if not identifier:
        return None
    if "://" in identifier:
        filename = unquote(os.path.basename(urlsplit(identifier).path))
    else:
        filename = os.path.basename(identifier)
    return filename or None


def _texture_identifiers(value: Any) -> list[str]:
    """Return resolved and authored texture identifiers in preferred order."""
    candidates: list[str] = []
    if isinstance(value, Sdf.AssetPath):
        for identifier in (value.resolvedPath, value.path):
            normalized = _normalize_texture_identifier(identifier)
            if normalized and normalized not in candidates:
                candidates.append(normalized)
    else:
        normalized = _normalize_texture_identifier(str(value))
        if normalized:
            candidates.append(normalized)
    return candidates


def _authoring_layer(source_property: Any, prim: Usd.Prim) -> Sdf.Layer | None:
    """Return the strongest layer authoring a texture input, then the stage root."""
    attribute = source_property.GetAttr() if hasattr(source_property, "GetAttr") else source_property
    if attribute and hasattr(attribute, "GetPropertyStack"):
        try:
            for property_spec in attribute.GetPropertyStack():
                layer = getattr(property_spec, "layer", None)
                if layer:
                    return layer
        except Exception as exc:
            _logger.debug(f"Could not inspect texture property stack: {exc}")

    stage = prim.GetStage() if prim else None
    return stage.GetRootLayer() if stage else None


def resolve_texture_paths(value: Any, prim: Usd.Prim, source_property: Any | None = None) -> list[str]:
    """Resolve a texture input to concrete files, including all UDIM tiles."""
    identifiers = _texture_identifiers(value)
    if not identifiers:
        return []

    if any(UsdShade.UdimUtils.IsUdimIdentifier(identifier) for identifier in identifiers):
        layer = _authoring_layer(source_property, prim)
        if not layer:
            _logger.warning(f"Cannot resolve UDIM texture without an authoring layer: {identifiers[0]}")
            return []

        for identifier in identifiers:
            if not UsdShade.UdimUtils.IsUdimIdentifier(identifier):
                continue
            try:
                tiles = UsdShade.UdimUtils.ResolveUdimTilePaths(identifier, layer)
            except Exception as exc:
                _logger.warning(f"Failed to resolve UDIM texture {identifier}: {exc}")
                continue

            resolved_paths: list[str] = []
            for resolved_path, _tile in sorted(tiles, key=lambda item: item[1]):
                normalized = _normalize_texture_identifier(resolved_path)
                if normalized and normalized not in resolved_paths:
                    resolved_paths.append(normalized)
            if resolved_paths:
                return resolved_paths

        _logger.warning(f"No concrete tiles resolved for UDIM texture: {identifiers[0]}")
        return []

    return [identifiers[0]]


def copy_texture_payload(source_path: str, output_dir: str) -> str | None:
    """Copy a concrete local or remote texture beside exported mesh materials."""
    if not source_path or not output_dir:
        return None

    normalized_source = _normalize_texture_identifier(source_path)
    if UsdShade.UdimUtils.IsUdimIdentifier(normalized_source):
        _logger.warning(f"Refusing to copy unresolved UDIM texture template: {source_path}")
        return None

    if "://" in normalized_source:
        filename = unquote(os.path.basename(urlsplit(normalized_source).path))
    else:
        filename = os.path.basename(normalized_source)
    if not filename:
        _logger.warning(f"Texture source has no filename: {source_path}")
        return None

    os.makedirs(output_dir, exist_ok=True)
    if not os.path.isdir(output_dir):
        _logger.warning(f"Texture output directory is unavailable: {output_dir}")
        return None

    destination = os.path.join(output_dir, filename)
    if os.path.isfile(destination):
        return filename

    if os.path.isfile(normalized_source):
        try:
            shutil.copy2(normalized_source, destination)
        except OSError as exc:
            _logger.warning(f"Failed to copy texture {normalized_source} to {destination}: {exc}")
    elif "://" in normalized_source:
        try:
            import omni.client

            destination_url = omni.client.make_file_url_if_possible(os.path.abspath(destination))
            result = omni.client.copy(
                normalized_source,
                destination_url,
                omni.client.CopyBehavior.OVERWRITE,
            )
            if result != omni.client.Result.OK:
                _logger.warning(f"Failed to copy remote texture {normalized_source}: {result}")
        except Exception as exc:
            _logger.warning(f"Failed to copy remote texture {normalized_source} to {destination}: {exc}")
    else:
        _logger.warning(f"Resolved texture is not a readable file: {normalized_source}")

    return filename if os.path.isfile(destination) else None


def _get_sources(connectable: Any) -> list:
    """Safely extract the sources list from GetConnectedSources().

    GetConnectedSources() returns (list[ConnectionSourceInfo], list[SdfPath]).

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
    """Extract color and texture from a UsdPreviewSurface shader.

    Args:
        shader: USD shader to read.
        data: Material data to populate.
        output_dir: Directory for copied texture files.
    """
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
    """Read opacity from a UsdPreviewSurface shader.

    Args:
        shader: USD shader to read.

    Returns:
        Opacity value.
    """
    opacity_input = shader.GetInput("opacity")
    if opacity_input:
        val = opacity_input.Get()
        if val is not None:
            return float(val)
    return 1.0


def _read_texture_shader(shader: UsdShade.Shader, data: MaterialData, output_dir: str | None) -> None:
    """Read texture file path from a UsdUVTexture or image shader.

    Args:
        shader: USD shader to read.
        data: Material data to populate.
        output_dir: Directory for copied texture files.
    """
    if not shader:
        return

    file_input = shader.GetInput("file")
    if not file_input:
        return

    val = file_input.Get()
    if val is None:
        return

    identifier = _normalize_texture_identifier(_texture_identifier(val))
    is_udim = UsdShade.UdimUtils.IsUdimIdentifier(identifier)
    reference_filename = texture_reference_filename(val)
    resolved_paths = resolve_texture_paths(val, shader.GetPrim(), file_input)

    if output_dir:
        copied_filenames: list[str] = []
        for resolved in resolved_paths:
            filename = copy_texture_payload(resolved, output_dir)
            if filename:
                copied_filenames.append(filename)
        if is_udim and copied_filenames and len(copied_filenames) == len(resolved_paths):
            data.texture_filename = reference_filename
        elif is_udim and copied_filenames:
            _logger.warning(
                f"Not emitting incomplete UDIM texture reference {identifier}: "
                f"copied {len(copied_filenames)} of {len(resolved_paths)} tiles"
            )
        elif copied_filenames:
            data.texture_filename = copied_filenames[0]
    elif resolved_paths:
        data.texture_filename = reference_filename if is_udim else resolved_paths[0]

    if is_udim and not resolved_paths:
        _logger.warning(f"No concrete tiles resolved for UDIM texture: {identifier}")

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
