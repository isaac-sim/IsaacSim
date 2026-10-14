# SPDX-FileCopyrightText: Copyright (c) 2023-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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

"""Restore UDIM materials from explicit exporter metadata in OBJ material libraries."""

from __future__ import annotations

import json
import re
import shutil
import xml.etree.ElementTree as ET
from dataclasses import dataclass, field
from pathlib import Path

from pxr import Sdf, Usd, UsdShade

from .urdf_utils import _resolve_package_mesh_path

_PREFIX = "# isaac:source_udim "


@dataclass
class SourceUdim:
    """One exported material's UDIM tile set and concrete material variants."""

    name: str
    tiles: dict[int, Path]
    materials: set[str] = field(default_factory=set)


def read_source_udims(urdf_path: str, ros_packages: list[dict[str, str]]) -> dict[str, SourceUdim]:
    """Read explicit UDIM metadata from material libraries referenced by a URDF.

    Args:
        urdf_path: Original or relocated URDF file.
        ros_packages: Configured ROS package name/path mappings.

    Returns:
        UDIM groups keyed by their exporter identifiers.

    Raises:
        ValueError: If marked metadata is malformed, inconsistent, or references missing tiles.
    """
    root = Path(urdf_path).resolve().parent
    libraries: set[Path] = set()
    # Let the XML parser honor the declaration/BOM rather than decoding as UTF-8.
    # Retain the backend's tolerance for whitespace/comments before a declaration
    # by moving the declaration ahead of that prefix without changing its encoding.
    content = Path(urdf_path).read_bytes()
    offset = 0
    while offset < len(content):
        if content[offset] in b" \t\r\n":
            offset += 1
        elif content.startswith(b"<!--", offset):
            end = content.find(b"-->", offset + 4)
            if end == -1:
                break
            offset = end + 3
        else:
            break
    if (
        offset
        and content.startswith(b"<?xml", offset)
        and content[offset + 5 : offset + 6]
        in (
            b" ",
            b"\t",
            b"\r",
            b"\n",
        )
    ):
        end = content.find(b"?>", offset + 6)
        if end != -1:
            end += 2
            content = content[offset:end] + content[:offset] + content[end:]
    for mesh in ET.fromstring(content).iter("mesh"):
        filename = mesh.get("filename", "")
        if filename.startswith("package://"):
            filename = _resolve_package_mesh_path(filename, str(root), ros_packages) or ""
        filename = filename.removeprefix("file://")
        path = root / filename
        if path.suffix.lower() != ".obj" or not path.is_file():
            continue
        for line in path.read_text(encoding="utf-8", errors="replace").splitlines():
            parts = line.strip().split(maxsplit=1)
            if len(parts) == 2 and parts[0] == "mtllib":
                libraries.add((path.parent / parts[1]).resolve())

    groups: dict[str, SourceUdim] = {}
    for library in sorted(libraries):
        if not library.is_file():
            continue
        material = ""
        for line in library.read_text(encoding="utf-8", errors="replace").splitlines():
            line = line.strip()
            if line.startswith("newmtl "):
                material = line.split(maxsplit=1)[1]
            if not line.startswith(_PREFIX):
                continue
            try:
                meta = json.loads(line[len(_PREFIX) :])
                if meta.get("version") != 1:
                    continue
                group = meta["group"]
                tile = meta["tile"]
                if not re.fullmatch(r"[0-9a-f]{16}", group) or not isinstance(meta["name"], str):
                    raise ValueError("Invalid group identity")
                if not material.endswith(f"_udim_{group}_{tile}"):
                    raise ValueError("Material does not match its UDIM identity")
                tiles = {}
                for number, filename in meta["tiles"].items():
                    if not re.fullmatch(r"[1-9][0-9]{3}", number) or int(number) < 1001:
                        raise ValueError("Invalid UDIM tile number")
                    if not filename or Path(filename).name != filename or "\\" in filename:
                        raise ValueError("Tile must use a local basename")
                    path = (library.parent / filename).resolve()
                    if path.parent != library.parent or not path.is_file():
                        raise ValueError("Missing or unsafe UDIM tile")
                    tiles[int(number)] = path
                if tile not in tiles or len({path.suffix for path in tiles.values()}) != 1:
                    raise ValueError("Incomplete or inconsistent tile set")
                existing = groups.setdefault(group, SourceUdim(meta["name"], tiles))
                if existing.tiles != tiles or existing.name != meta["name"]:
                    raise ValueError("Conflicting UDIM group records")
                existing.materials.add(material)
            except (KeyError, TypeError, AttributeError, ValueError) as exc:
                raise ValueError(f"Invalid UDIM metadata in {library}: {exc}") from exc
    return groups


def _diffuse_texture_attributes(material: UsdShade.Material) -> list[Usd.Attribute]:
    """Find the authored asset inputs driving the material's diffuse texture.

    Args:
        material: Imported preview material.

    Returns:
        Asset-valued attributes supplying its diffuse texture.
    """
    surface, _, _ = material.ComputeSurfaceSource()
    if not surface:
        return []
    diffuse = surface.GetInput("diffuseColor")
    if not diffuse:
        return []
    attributes = []
    for output in diffuse.GetValueProducingAttributes():
        texture = UsdShade.Shader(output.GetPrim())
        file_input = texture.GetInput("file") if texture else None
        if file_input:
            attributes.extend(
                attr
                for attr in file_input.GetValueProducingAttributes()
                if attr.GetTypeName() == Sdf.ValueTypeNames.Asset
            )
    return attributes


def reconstruct_source_udims(
    stage: Usd.Stage, groups: dict[str, SourceUdim], output_dir: str, *, absolute_paths: bool = False
) -> int:
    """Restore shared UDIM material bindings and package their texture tiles.

    Original UDIM UV coordinates are retained by the OBJ exporter and importer.
    Only explicitly tagged material variants participate in reconstruction.

    Args:
        stage: Final imported USD stage to update.
        groups: Validated exporter UDIM records.
        output_dir: Directory for reconstructed texture delivery.
        absolute_paths: Whether to author absolute paths for subsequent asset packaging.

    Returns:
        Number of reconstructed UDIM groups.

    Raises:
        ValueError: If a tagged material cannot be reconstructed safely.
    """
    materials = [UsdShade.Material(prim) for prim in stage.Traverse() if prim.IsA(UsdShade.Material)]
    replacements: dict[Sdf.Path, Sdf.Path] = {}
    count = 0
    for group, source in groups.items():
        variants = [
            material
            for material in materials
            if (material.GetPrim().GetDisplayName() or material.GetPrim().GetName()) in source.materials
        ]
        if not variants:
            continue
        attributes = [_diffuse_texture_attributes(material) for material in variants]
        if not all(attributes):
            raise ValueError(f"Cannot reconstruct diffuse texture for UDIM material {source.name!r}")
        extension = next(iter(source.tiles.values())).suffix
        relative_dir = Path("textures") / f"udim_{group}"
        destination = Path(output_dir) / relative_dir
        destination.mkdir(parents=True, exist_ok=True)
        for tile, path in source.tiles.items():
            shutil.copyfile(path, destination / f"udim_{group}.{tile}{extension}")
        template = (relative_dir / f"udim_{group}.<UDIM>{extension}").as_posix()
        if absolute_paths:
            template = (Path(output_dir).resolve() / template).as_posix()
        for material, inputs in zip(variants, attributes):
            for attribute in inputs:
                attribute.Set(Sdf.AssetPath(template))
            replacements[material.GetPath()] = variants[0].GetPath()
        variants[0].GetPrim().SetDisplayName(source.name)
        count += 1

    for prim in stage.Traverse():
        for relationship in prim.GetRelationships():
            if relationship.GetName().startswith("material:binding"):
                targets = relationship.GetTargets()
                updated = [replacements.get(target, target) for target in targets]
                if targets != updated:
                    relationship.SetTargets(updated)
    return count
