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

import filecmp
import logging
import os
import re
import shutil
from collections.abc import Callable
from dataclasses import dataclass
from typing import Any
from urllib.parse import unquote, urlsplit

from pxr import Ar, Sdf, Usd, UsdShade

from .transform_utils import get_prim_name, linear_to_srgb

_logger = logging.getLogger(__name__)

_ENCODED_UDIM_TOKEN = re.compile(r"%3cudim%3e", re.IGNORECASE)
_LITERAL_UDIM_TOKEN = re.compile(r"<udim>", re.IGNORECASE)
_UDIM_TOKEN = "<UDIM>"


def _texture_identifier(value: Any) -> str:
    """Return the authored texture identifier from a shader input value.

    Args:
        value: Authored shader input value.

    Returns:
        Authored texture identifier.
    """
    if isinstance(value, Sdf.AssetPath):
        return value.path
    return str(value)


def _normalize_texture_identifier(identifier: str) -> str:
    """Normalize literal and encoded UDIM tokens without decoding other URL content.

    Args:
        identifier: Authored or resolved texture identifier.

    Returns:
        Identifier with canonical UDIM tokens.
    """
    identifier = _ENCODED_UDIM_TOKEN.sub(_UDIM_TOKEN, identifier)
    return _LITERAL_UDIM_TOKEN.sub(_UDIM_TOKEN, identifier)


def _safe_texture_filename(identifier: str) -> str | None:
    """Return a decoded texture filename if it is exactly one safe path component.

    Args:
        identifier: Texture path or URL.

    Returns:
        Decoded basename, or None if it contains unsafe path components.
    """
    if "://" in identifier:
        filename = unquote(os.path.basename(urlsplit(identifier).path))
    else:
        filename = os.path.basename(identifier)

    if not filename:
        return None
    if filename in {".", ".."} or "/" in filename or "\\" in filename or "\0" in filename:
        _logger.warning(f"Refusing texture filename containing path components: {filename!r}")
        return None
    return filename


def texture_reference_filename(value: Any) -> str | None:
    """Return a portable texture filename while preserving a UDIM token.

    Args:
        value: Authored shader input value.

    Returns:
        Portable basename retaining any UDIM token, or None if invalid.
    """
    identifier = _normalize_texture_identifier(_texture_identifier(value))
    if not identifier:
        return None
    return _safe_texture_filename(identifier)


def _texture_identifiers(value: Any) -> list[str]:
    """Return resolved and authored texture identifiers in preferred order.

    Args:
        value: Authored shader input value.

    Returns:
        Unique normalized identifiers, preferring the resolved path.
    """
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
    """Return the strongest layer authoring a texture input, then the stage root.

    Args:
        source_property: Shader input or attribute authoring the texture.
        prim: Prim whose stage provides the fallback root layer.

    Returns:
        Strongest authoring layer, stage root layer, or None if unavailable.
    """
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


def get_texture_source_attribute(texture_input: UsdShade.Input) -> Usd.Attribute:
    """Resolve a texture input through a material interface connection.

    Args:
        texture_input: Asset-valued shader input.

    Returns:
        Attribute authoring the texture asset, including its authoring layer.
    """
    for attribute in texture_input.GetValueProducingAttributes():
        if attribute.GetTypeName() == Sdf.ValueTypeNames.Asset:
            return attribute
    return texture_input.GetAttr()


def resolve_texture_paths(value: Any, prim: Usd.Prim, source_property: Any | None = None) -> list[str]:
    """Resolve a texture input to concrete files, including all UDIM tiles.

    Args:
        value: Authored texture asset value.
        prim: Prim providing the stage context.
        source_property: Optional input or attribute supplying the authoring layer.

    Returns:
        Resolved concrete texture paths, including available UDIM tiles.
    """
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


def _texture_destination(
    filename: str, output_root: str, matches_payload: Callable[[str], bool]
) -> tuple[str, str] | None:
    """Select an unused filename or reuse a file with identical contents.

    Args:
        filename: Preferred texture basename.
        output_root: Resolved output directory.
        matches_payload: Callback comparing a candidate file with the source payload.

    Returns:
        Delivered filename and destination path, or None for an unsafe destination.
    """
    stem, extension = os.path.splitext(filename)
    candidate = filename
    index = 0
    while True:
        destination = os.path.realpath(os.path.join(output_root, candidate))
        try:
            is_contained = os.path.commonpath((output_root, destination)) == output_root
        except ValueError:
            is_contained = False
        if not is_contained:
            _logger.warning(f"Refusing texture destination outside output directory: {destination}")
            return None
        if not os.path.lexists(os.path.join(output_root, candidate)):
            return candidate, destination
        if os.path.isfile(destination) and matches_payload(destination):
            return candidate, destination
        index += 1
        candidate = f"{stem}_{index}{extension}"


def copy_texture_payload(source_path: str, output_dir: str) -> str | None:
    """Copy a concrete local or remote texture beside exported mesh materials.

    Reuse identical payloads and add a numeric suffix when different payloads share
    a basename. Return the delivered filename for use in material references.

    Args:
        source_path: Concrete local path or resolver-backed texture URL.
        output_dir: Directory for delivered textures.

    Returns:
        Delivered filename, or None when the payload cannot be copied safely.
    """
    if not source_path or not output_dir:
        return None

    normalized_source = _normalize_texture_identifier(source_path)
    if UsdShade.UdimUtils.IsUdimIdentifier(normalized_source):
        _logger.warning(f"Refusing to copy unresolved UDIM texture template: {source_path}")
        return None

    filename = _safe_texture_filename(normalized_source)
    if not filename:
        _logger.warning(f"Texture source has no filename: {source_path}")
        return None

    try:
        os.makedirs(output_dir, exist_ok=True)
    except OSError as exc:
        _logger.warning(f"Could not create texture output directory {output_dir}: {exc}")
        return None
    if not os.path.isdir(output_dir):
        _logger.warning(f"Texture output directory is unavailable: {output_dir}")
        return None

    output_root = os.path.realpath(output_dir)
    if os.path.isfile(normalized_source):
        try:
            selected = _texture_destination(
                filename, output_root, lambda path: filecmp.cmp(normalized_source, path, shallow=False)
            )
        except OSError as exc:
            _logger.warning(f"Failed to compare texture {normalized_source}: {exc}")
            return None
        if selected is None:
            return None
        filename, destination = selected
        if os.path.isfile(destination):
            return filename

        try:
            shutil.copy2(normalized_source, destination)
        except OSError as exc:
            _logger.warning(f"Failed to copy texture {normalized_source} to {destination}: {exc}")
    elif "://" in normalized_source:
        try:
            resolver = Ar.GetResolver()
            resolved_path = resolver.Resolve(normalized_source)
            if not resolved_path:
                resolved_path = Ar.ResolvedPath(normalized_source)
            asset = resolver.OpenAsset(resolved_path)
            if not asset:
                _logger.warning(f"Failed to resolve remote texture {normalized_source}")
                return None

            buffer = asset.GetBuffer()
            if buffer is None:
                _logger.warning(f"Failed to read remote texture {normalized_source}")
                return None

            payload = bytes(buffer)
        except Exception as exc:
            _logger.warning(f"Failed to read remote texture {normalized_source}: {exc}")
            return None

        def matches_payload(path: str) -> bool:
            """Compare a delivered texture with the resolved remote payload.

            Args:
                path: Candidate texture file.

            Returns:
                Whether the file contains the same bytes as the remote payload.
            """
            with open(path, "rb") as stream:
                return stream.read() == payload

        try:
            selected = _texture_destination(filename, output_root, matches_payload)
        except OSError as exc:
            _logger.warning(f"Failed to compare texture {normalized_source}: {exc}")
            return None
        if selected is None:
            return None
        filename, destination = selected
        if os.path.isfile(destination):
            return filename

        try:
            with open(destination, "wb") as stream:
                stream.write(payload)
        except OSError as exc:
            try:
                os.remove(destination)
            except OSError:
                pass
            _logger.warning(f"Failed to copy remote texture {normalized_source} to {destination}: {exc}")
    else:
        _logger.warning(f"Resolved texture is not a readable file: {normalized_source}")
        return None

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

    source_attribute = get_texture_source_attribute(file_input)
    val = source_attribute.Get()
    if val is None:
        return

    identifier = _normalize_texture_identifier(_texture_identifier(val))
    is_udim = UsdShade.UdimUtils.IsUdimIdentifier(identifier)
    reference_filename = texture_reference_filename(val)
    resolved_paths = resolve_texture_paths(val, shader.GetPrim(), source_attribute)

    if output_dir:
        copied_filenames = [
            filename
            for resolved in resolved_paths
            if (filename := copy_texture_payload(resolved, output_dir)) is not None
        ]
        if is_udim:
            if resolved_paths and copied_filenames == [texture_reference_filename(path) for path in resolved_paths]:
                data.texture_filename = reference_filename
            elif resolved_paths:
                _logger.warning(
                    f"Not emitting incomplete or renamed UDIM texture reference {identifier}: "
                    f"copied {len(copied_filenames)} of {len(resolved_paths)} tiles"
                )
        elif copied_filenames:
            data.texture_filename = copied_filenames[0]
        elif resolved_paths:
            data.texture_filename = resolved_paths[0]
    elif not is_udim and resolved_paths:
        data.texture_filename = resolved_paths[0]

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
