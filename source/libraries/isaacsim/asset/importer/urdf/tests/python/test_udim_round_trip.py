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

"""Exercise USD/URDF/USD UDIM reconstruction through the public converters."""

from __future__ import annotations

import json
import os
import shutil
import struct
import subprocess
import sys
import zlib
from pathlib import Path

import pytest
from isaacsim.asset.exporter.urdf import UsdToUrdfConverter
from isaacsim.asset.importer.urdf import URDFImporter, URDFImporterConfig
from isaacsim.asset.importer.urdf.impl.material_reconstruction import (
    _diffuse_texture_attributes,
    read_source_udims,
)
from pxr import Sdf, Usd, UsdGeom, UsdPhysics, UsdShade


def _png(color: tuple[int, int, int]) -> bytes:
    """Encode a valid one-pixel RGB PNG.

    Args:
        color: RGB channels.

    Returns:
        Encoded image bytes.
    """

    def chunk(kind: bytes, payload: bytes) -> bytes:
        return struct.pack("!I", len(payload)) + kind + payload + struct.pack("!I", zlib.crc32(kind + payload))

    return (
        b"\x89PNG\r\n\x1a\n"
        + chunk(b"IHDR", struct.pack("!2I5B", 1, 1, 8, 2, 0, 0, 0))
        + chunk(b"IDAT", zlib.compress(b"\0" + bytes(color)))
        + chunk(b"IEND", b"")
    )


def _export_robot(root: Path) -> dict[str, dict[int, bytes]]:
    """Export two distinct UDIM materials whose tile basenames collide.

    Args:
        root: Temporary asset and export directory.

    Returns:
        Expected texture bytes indexed by source material name and tile.
    """
    stage = Usd.Stage.CreateInMemory()
    UsdGeom.SetStageMetersPerUnit(stage, 1.0)
    robot = UsdGeom.Xform.Define(stage, "/robot").GetPrim()
    stage.SetDefaultPrim(robot)
    UsdPhysics.ArticulationRootAPI.Apply(robot)
    link = UsdGeom.Xform.Define(stage, "/robot/base").GetPrim()
    UsdPhysics.RigidBodyAPI.Apply(link)
    UsdPhysics.FixedJoint.Define(stage, "/robot/fixed").CreateBody1Rel().SetTargets([link.GetPath()])
    expected = {}
    for index, name in enumerate(("first", "second")):
        directory = root / name
        directory.mkdir()
        expected[name] = {}
        for tile in (1001, 1012):
            payload = _png((index * 200, 100 if tile == 1001 else 200, 30))
            (directory / f"albedo.{tile}.png").write_bytes(payload)
            expected[name][tile] = payload
        material = UsdShade.Material.Define(stage, f"/robot/Looks/{name}")
        shader = UsdShade.Shader.Define(stage, f"{material.GetPath()}/surface")
        shader.CreateIdAttr("UsdPreviewSurface")
        material.CreateSurfaceOutput().ConnectToSource(shader.ConnectableAPI(), "surface")
        texture = UsdShade.Shader.Define(stage, f"{material.GetPath()}/texture")
        texture.CreateIdAttr("UsdUVTexture")
        texture.CreateInput("file", Sdf.ValueTypeNames.Asset).Set(Sdf.AssetPath(str(directory / "albedo.<UDIM>.png")))
        shader.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f).ConnectToSource(texture.ConnectableAPI(), "rgb")
        mesh = UsdGeom.Mesh.Define(stage, f"/robot/base/{name}")
        mesh.CreatePointsAttr([(index * 3 + x, y, 0) for x, y in ((0, 0), (1, 0), (0, 1), (1, 1))])
        mesh.CreateFaceVertexCountsAttr([3, 3])
        mesh.CreateFaceVertexIndicesAttr([0, 1, 2, 1, 3, 2])
        UsdGeom.PrimvarsAPI(mesh).CreatePrimvar(
            "st", Sdf.ValueTypeNames.TexCoord2fArray, UsdGeom.Tokens.faceVarying
        ).Set([(0.1, 0.1), (0.9, 0.1), (0.1, 0.9), (1.1, 1.1), (1.9, 1.1), (1.1, 1.9)])
        UsdShade.MaterialBindingAPI.Apply(mesh.GetPrim()).Bind(material)
    (root / "export").mkdir()
    UsdToUrdfConverter(stage).convert(str(root / "export/robot.urdf"))
    return expected


@pytest.mark.parametrize("transform", [False, True])
@pytest.mark.parametrize("merge_fixed", [False, True])
def test_udim_round_trip_survives_relocation(tmp_path: Path, transform: bool, merge_fixed: bool) -> None:
    """Restore shared UDIM shaders, tile bytes, and UVs in a portable USD package.

    Args:
        tmp_path: Temporary test directory.
        transform: Whether to run the default asset transformer during import.
        merge_fixed: Whether to preprocess the URDF by merging fixed joints.
    """
    if transform:
        pytest.importorskip(
            "isaacsim.asset.transformer.rules", reason="Default profile is delivered by the Kit adapter"
        )
    expected = _export_robot(tmp_path)
    imported = URDFImporter(
        URDFImporterConfig(
            urdf_path=str(tmp_path / "export/robot.urdf"),
            usd_path=str(tmp_path / "import"),
            run_asset_transformer=transform,
            merge_fixed_joints=merge_fixed,
            run_multi_physics_conversion=False,
        )
    ).import_urdf()
    relocated = tmp_path / "relocated"
    shutil.move(str(Path(imported).parent), relocated)
    for directory in ("export", "first", "second"):
        shutil.rmtree(tmp_path / directory)
    stage = Usd.Stage.Open(str(relocated / Path(imported).name))
    bound = {}
    coordinates = []
    for prim in Usd.PrimRange(stage.GetPseudoRoot(), Usd.TraverseInstanceProxies()):
        if prim.IsA(UsdGeom.Mesh) or prim.IsA(UsdGeom.Subset):
            material, _ = UsdShade.MaterialBindingAPI(prim).ComputeBoundMaterial()
            if material:
                bound[material.GetPath()] = material
        if prim.IsA(UsdGeom.Mesh):
            for uv in UsdGeom.PrimvarsAPI(prim).GetPrimvar("st").ComputeFlattened():
                coordinates.append(tuple(round(float(value), 6) for value in uv))
    assert len(bound) == 2
    assert sorted(coordinates) == sorted([(0.1, 0.1), (0.9, 0.1), (0.1, 0.9), (1.1, 1.1), (1.9, 1.1), (1.1, 1.9)] * 2)
    for material in bound.values():
        name = next(name for name in expected if material.GetPrim().GetName().startswith(name + "_udim_"))
        attributes = _diffuse_texture_attributes(material)
        assert len(attributes) == 1
        template = attributes[0].Get().path
        assert "<UDIM>" in template
        tiles = UsdShade.UdimUtils.ResolveUdimTilePaths(template, attributes[0].GetPropertyStack()[0].layer)
        assert {int(tile): Path(path).read_bytes() for path, tile in tiles} == expected[name]

    (tmp_path / "reexport").mkdir()
    UsdToUrdfConverter(stage).convert(str(tmp_path / "reexport/robot.urdf"))
    groups = read_source_udims(str(tmp_path / "reexport/robot.urdf"), [])
    assert len(groups) == 2
    assert sorted(sorted(path.read_bytes() for path in group.tiles.values()) for group in groups.values()) == sorted(
        sorted(tiles.values()) for tiles in expected.values()
    )


def test_unmarked_materials_are_not_inferred_as_udim(tmp_path: Path) -> None:
    """Leave arbitrary third-party material names alone when no metadata is present.

    Args:
        tmp_path: Temporary directory for OBJ and URDF files.
    """
    (tmp_path / "mesh.obj").write_text("mtllib mesh.mtl\n")
    (tmp_path / "mesh.mtl").write_text("newmtl ordinary_udim_1001\nmap_Kd texture.png\n")
    (tmp_path / "robot.urdf").write_text(
        '<robot><link><visual><geometry><mesh filename="mesh.obj"/></geometry></visual></link></robot>'
    )
    assert read_source_udims(str(tmp_path / "robot.urdf"), []) == {}


@pytest.mark.parametrize("corrupt", ["missing", "traversal", "conflict"])
def test_invalid_udim_metadata_is_rejected(tmp_path: Path, corrupt: str) -> None:
    """Reject incomplete tile sets and unsafe or inconsistent metadata.

    Args:
        tmp_path: Temporary export directory.
        corrupt: Metadata corruption to introduce.
    """
    _export_robot(tmp_path)
    urdf = tmp_path / "export/robot.urdf"
    groups = read_source_udims(str(urdf), [])
    if corrupt == "missing":
        next(iter(next(iter(groups.values())).tiles.values())).unlink()
    else:
        mtl = next((tmp_path / "export/meshes").glob("*.mtl"))
        lines = mtl.read_text().splitlines()
        for index, line in enumerate(lines):
            if line.startswith("# isaac:source_udim "):
                meta = json.loads(line.split(" ", 2)[2])
                if corrupt == "traversal":
                    meta["tiles"]["1001"] = "../outside.png"
                else:
                    meta["name"] += "_conflict"
                lines[index] = "# isaac:source_udim " + json.dumps(meta)
                break
        mtl.write_text("\n".join(lines))
    with pytest.raises(ValueError, match="Invalid UDIM metadata"):
        read_source_udims(str(urdf), [])


@pytest.mark.parametrize("encoding", ["utf-8", "iso-8859-1"])
@pytest.mark.parametrize("prefix", ["", "\n  \n", "<!-- robot asset -->\n"])
def test_udim_metadata_accepts_backend_xml_formatting(tmp_path: Path, prefix: str, encoding: str) -> None:
    """Read tagged materials with XML declaration placement accepted by the backend.

    Args:
        tmp_path: Temporary export directory.
        prefix: Whitespace or comments preceding the XML declaration.
        encoding: Declared encoding used for non-ASCII mesh references.
    """
    expected = _export_robot(tmp_path)
    urdf = tmp_path / "export/robot.urdf"
    content = urdf.read_text(encoding="utf-8")
    if content.startswith("<?xml"):
        content = content.split("?>", 1)[1]
    mesh = next((tmp_path / "export/meshes").glob("*.obj"))
    renamed = mesh.with_name("café_" + mesh.name)
    content = content.replace(mesh.name, renamed.name)
    mesh.rename(renamed)
    urdf.write_bytes((prefix + f'<?xml version="1.0" encoding="{encoding}" ?>\n' + content).encode(encoding))
    groups = read_source_udims(str(urdf), [])
    assert {
        group.name: {tile: path.read_bytes() for tile, path in group.tiles.items()} for group in groups.values()
    } == expected


def test_missing_urdf_preserves_backend_error(tmp_path: Path) -> None:
    """Keep the backend's public validation error for a nonexistent input file.

    Args:
        tmp_path: Temporary directory without the requested input file.
    """
    importer = URDFImporter(
        URDFImporterConfig(
            urdf_path=str(tmp_path / "missing.urdf"),
            usd_path=str(tmp_path / "import"),
            run_asset_transformer=False,
            merge_fixed_joints=False,
        )
    )
    with pytest.raises(ValueError, match="is not a readable file"):
        importer.import_urdf()


@pytest.mark.parametrize("encoding", ["utf-8", "iso-8859-1", "utf-16"])
def test_unmarked_urdf_honors_xml_encoding(tmp_path: Path, encoding: str) -> None:
    """Scan an ordinary URDF without discarding its declared character encoding.

    Args:
        tmp_path: Temporary directory for the URDF file.
        encoding: XML encoding, including UTF-16 with its byte-order mark.
    """
    urdf = tmp_path / "robot.urdf"
    urdf.write_bytes(
        (f'<?xml version="1.0" encoding="{encoding}"?>\n<robot name="café"><link name="base"/></robot>').encode(
            encoding
        )
    )
    assert read_source_udims(str(urdf), []) == {}


@pytest.mark.parametrize("suffix", [b"<robot/>", b'<?xml version="1.0"?><robot/>', b"<?xml "])
def test_xml_comment_prefix_completes_promptly(tmp_path: Path, suffix: bytes) -> None:
    """Bound parsing time for repeated comments, including incomplete declarations.

    Args:
        tmp_path: Temporary directory for the URDF file.
        suffix: XML content following the comment prefix.
    """
    urdf = tmp_path / "robot.urdf"
    urdf.write_bytes(b"<!-- robot -->\n" * 1000 + suffix)
    script = """
import sys
import xml.etree.ElementTree as ET
from isaacsim.asset.importer.urdf.impl.material_reconstruction import read_source_udims
try:
    assert read_source_udims(sys.argv[1], []) == {}
except ET.ParseError:
    assert sys.argv[2] == 'invalid'
else:
    assert sys.argv[2] == 'valid'
"""
    subprocess.run(
        [sys.executable, "-c", script, str(urdf), "invalid" if suffix == b"<?xml " else "valid"],
        env={**os.environ, "PYTHONPATH": os.pathsep.join(sys.path)},
        check=True,
        timeout=15,
        capture_output=True,
        text=True,
    )
