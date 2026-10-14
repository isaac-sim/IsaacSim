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

"""Preserve texture payloads and material references when source basenames collide."""

from __future__ import annotations

from pathlib import Path
from unittest import mock

import pytest
from isaacsim.asset.exporter.urdf import UsdToUrdfConverter
from isaacsim.asset.exporter.urdf.converter.material_reader import copy_texture_payload, read_material
from pxr import Ar, Sdf, Usd, UsdGeom, UsdPhysics, UsdShade


@pytest.mark.parametrize("remote", [False, True])
def test_collision_preserves_payloads_and_reuses_names(tmp_path: Path, remote: bool) -> None:
    """Preserve existing files and reuse identical local or resolver-backed payloads.

    Args:
        tmp_path: Temporary directory for sources and output.
        remote: Whether to read through the USD asset resolver.
    """
    output = tmp_path / "export"
    output.mkdir()
    (output / "albedo.png").write_bytes(b"original")
    (output / "albedo_1.png").write_bytes(b"occupied suffix")
    payload = b"different texture"
    source = tmp_path / "albedo.png"
    source.write_bytes(payload)
    identifier = "omniverse://example.test/textures/%61lbedo.png" if remote else str(source)
    resolver = mock.Mock()
    resolver.Resolve.return_value = Ar.ResolvedPath(identifier)
    resolver.OpenAsset.return_value.GetBuffer.return_value = payload
    with mock.patch("isaacsim.asset.exporter.urdf.converter.material_reader.Ar.GetResolver", return_value=resolver):
        delivered = copy_texture_payload(identifier, str(output))
        assert delivered is not None
        assert (output / delivered).read_bytes() == payload
        assert copy_texture_payload(identifier, str(output)) == delivered
        assert copy_texture_payload(str(source), str(output)) == delivered
    assert (output / "albedo.png").read_bytes() == b"original"
    assert (output / "albedo_1.png").read_bytes() == b"occupied suffix"
    assert len(list(output.iterdir())) == 3


@pytest.mark.parametrize("udim", [False, True])
def test_export_colliding_textures_keeps_material_references(tmp_path: Path, udim: bool) -> None:
    """Export two materials with different same-named textures and verify MTL payloads.

    Args:
        tmp_path: Temporary directory for sources and output.
        udim: Whether to author a UDIM template.
    """
    stage = Usd.Stage.CreateInMemory()
    root = UsdGeom.Xform.Define(stage, "/robot").GetPrim()
    stage.SetDefaultPrim(root)
    UsdPhysics.ArticulationRootAPI.Apply(root)
    link = UsdGeom.Xform.Define(stage, "/robot/base").GetPrim()
    UsdPhysics.RigidBodyAPI.Apply(link)
    UsdPhysics.FixedJoint.Define(stage, "/robot/fixed").CreateBody1Rel().SetTargets([link.GetPath()])
    payloads = {"first": b"first-texture", "second": b"second-texture"}
    for index, (name, payload) in enumerate(payloads.items()):
        source = tmp_path / name
        source.mkdir()
        filename = "albedo.1001.png" if udim else "albedo.png"
        (source / filename).write_bytes(payload)
        material = UsdShade.Material.Define(stage, f"/robot/Looks/{name}")
        shader = UsdShade.Shader.Define(stage, f"{material.GetPath()}/surface")
        shader.CreateIdAttr("UsdPreviewSurface")
        material.CreateSurfaceOutput().ConnectToSource(shader.ConnectableAPI(), "surface")
        texture = UsdShade.Shader.Define(stage, f"{material.GetPath()}/texture")
        texture.CreateIdAttr("UsdUVTexture")
        texture.CreateInput("file", Sdf.ValueTypeNames.Asset).Set(
            Sdf.AssetPath(str(source / ("albedo.<UDIM>.png" if udim else filename)))
        )
        shader.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f).ConnectToSource(texture.ConnectableAPI(), "rgb")
        mesh = UsdGeom.Mesh.Define(stage, f"/robot/base/{name}")
        mesh.CreatePointsAttr([(index * 2, 0, 0), (index * 2 + 1, 0, 0), (index * 2, 1, 0)])
        mesh.CreateFaceVertexCountsAttr([3])
        mesh.CreateFaceVertexIndicesAttr([0, 1, 2])
        UsdGeom.PrimvarsAPI(mesh).CreatePrimvar(
            "st", Sdf.ValueTypeNames.TexCoord2fArray, UsdGeom.Tokens.faceVarying
        ).Set([(0.1, 0.1), (0.9, 0.1), (0.1, 0.9)])
        UsdShade.MaterialBindingAPI.Apply(mesh.GetPrim()).Bind(material)

    output = tmp_path / "export"
    output.mkdir()
    converter = UsdToUrdfConverter(stage=stage, root_prim_path="/robot")
    converter.convert(str(output / "robot.urdf"))
    references = {}
    for mtl in output.rglob("*.mtl"):
        material_name = None
        for line in mtl.read_text().splitlines():
            if line.startswith("newmtl "):
                material_name = line.split(maxsplit=1)[1]
            elif line.startswith("map_Kd "):
                references[material_name] = (mtl.parent / line.split(maxsplit=1)[1]).read_bytes()
    for name, payload in payloads.items():
        matching = (
            [value for key, value in references.items() if key.startswith(name + "_udim_")]
            if udim
            else [references[name]]
        )
        assert matching == [payload]
    for name, payload in payloads.items():
        material = UsdShade.Material(stage.GetPrimAtPath(f"/robot/Looks/{name}"))
        data = read_material(material, str(output / "meshes"))
        assert data is not None
        if udim and name == "second":
            # A renamed tile set cannot safely retain the original template.
            assert data.texture_filename is None
        elif not udim:
            assert (output / "meshes" / data.texture_filename).read_bytes() == payload
    assert (output / "robot.urdf").is_file()
