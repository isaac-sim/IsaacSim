# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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

"""Standalone regressions for resolved physics bindings and texture directory casing."""

from __future__ import annotations

import tempfile
import unittest
from pathlib import Path

from isaacsim.asset.transformer.rules.perf.geometries import GeometriesRoutingRule
from isaacsim.asset.transformer.rules.perf.materials import MaterialsRoutingRule
from pxr import Gf, Sdf, Usd, UsdGeom, UsdPhysics, UsdShade


class TestPhysicsBindingResolution(unittest.TestCase):
    """Preserve resolved physics materials while geometry becomes instanceable."""

    def test_resolved_bindings_survive_instancing(self) -> None:
        """Cover direct, inherited, collection, strength, and all-purpose fallback cases."""
        for mode in ("direct", "inherited", "collection", "ancestor_collection", "strong_ancestor", "visual_fallback"):
            for merge in (True, False):
                with self.subTest(mode=mode, merge=merge), tempfile.TemporaryDirectory() as directory:
                    self._check_binding(Path(directory), mode, merge)

    def test_unbinds_only_local_physics_relationships(self) -> None:
        """Clear invalid direct/collection targets without changing visual bindings or referenced layers."""
        referenced = Usd.Stage.CreateInMemory()
        referenced.DefinePrim("/Geometry", "Xform")
        mesh = referenced.DefinePrim("/Geometry/Mesh", "Mesh")
        mesh.CreateRelationship("material:binding:physics").SetTargets(["/Outside/Physics"])
        original_referenced = referenced.GetRootLayer().ExportToString()

        stage = Usd.Stage.CreateInMemory()
        entry = stage.DefinePrim("/Instances/Entry", "Xform")
        entry.GetReferences().AddReference(referenced.GetRootLayer().identifier, "/Geometry")
        # An over is skipped by normal traversal, but still carries authored bindings.
        over = stage.OverridePrim("/Instances/Overrides/Child")
        visual_targets = {}
        for prim in (entry, over):
            api = UsdShade.MaterialBindingAPI.Apply(prim)
            for purpose in ("", "preview", "full", "physics"):
                direct = api.GetDirectBindingRel(purpose)
                if not direct:
                    name = "material:binding" + (f":{purpose}" if purpose else "")
                    direct = prim.CreateRelationship(name)
                direct.SetTargets([Sdf.Path("/Outside/Material")])
                for name in ("first", "second"):
                    rel_name = "material:binding:collection:" + (f"{purpose}:" if purpose else "") + name
                    relationship = prim.CreateRelationship(rel_name)
                    relationship.SetTargets(["/Outside.collection:members", "/Outside/Material"])
                    if purpose != "physics":
                        visual_targets[relationship.GetPath()] = relationship.GetTargets()
                if purpose != "physics":
                    visual_targets[direct.GetPath()] = direct.GetTargets()

        GeometriesRoutingRule._remove_instance_physics_bindings(stage.GetRootLayer())
        for prim in (entry, over):
            api = UsdShade.MaterialBindingAPI(prim)
            direct = api.GetDirectBindingRel("physics")
            self.assertTrue(direct.HasAuthoredTargets())
            self.assertEqual(direct.GetTargets(), [])
            for name in ("first", "second"):
                self.assertEqual(api.GetCollectionBindingRel(name, "physics").GetTargets(), [])
        for path, targets in visual_targets.items():
            self.assertEqual(stage.GetRelationshipAtPath(path).GetTargets(), targets)
        self.assertEqual(referenced.GetRootLayer().ExportToString(), original_referenced)
        self.assertIsNone(stage.GetRootLayer().GetPrimAtPath("/Instances/Entry/Mesh"))

        after = stage.GetRootLayer().ExportToString()
        GeometriesRoutingRule._remove_instance_physics_bindings(stage.GetRootLayer())
        self.assertEqual(stage.GetRootLayer().ExportToString(), after)

    def _check_binding(self, directory: Path, mode: str, merge: bool) -> None:
        stage = Usd.Stage.CreateNew(str(directory / "source.usda"))
        root = UsdGeom.Xform.Define(stage, "/root").GetPrim()
        stage.SetDefaultPrim(root)
        UsdGeom.Xform.Define(stage, "/root/body")
        mesh = UsdGeom.Mesh.Define(stage, "/root/body/Mesh")
        mesh.CreatePointsAttr([Gf.Vec3f(0, 0, 0), Gf.Vec3f(1, 0, 0), Gf.Vec3f(0, 1, 0)])
        mesh.CreateFaceVertexCountsAttr([3])
        mesh.CreateFaceVertexIndicesAttr([0, 1, 2])
        if not merge:
            sibling = UsdGeom.Xform.Define(stage, "/root/body/Sibling").GetPrim()
            sibling.CreateAttribute("keep", Sdf.ValueTypeNames.Bool).Set(True)

        visual = UsdShade.Material.Define(stage, "/root/Materials/Visual")
        physics = UsdShade.Material.Define(stage, "/root/Physics/Material")
        UsdPhysics.MaterialAPI.Apply(physics.GetPrim())
        mesh_binding = UsdShade.MaterialBindingAPI.Apply(mesh.GetPrim())
        mesh_binding.Bind(visual)
        strength = UsdShade.Tokens.weakerThanDescendants
        if mode == "direct":
            mesh_binding.Bind(physics, materialPurpose="physics")
        elif mode == "inherited":
            UsdShade.MaterialBindingAPI.Apply(root).Bind(physics, materialPurpose="physics")
        elif mode in ("collection", "ancestor_collection"):
            owner = mesh.GetPrim() if mode == "collection" else root
            collection = Usd.CollectionAPI.Apply(owner, "colliders")
            collection.CreateIncludesRel().SetTargets([mesh.GetPath()])
            UsdShade.MaterialBindingAPI.Apply(owner).Bind(collection, physics, "colliders", materialPurpose="physics")
            # The collection binding must win over a competing direct binding.
            other = UsdShade.Material.Define(stage, "/root/Physics/Other")
            UsdShade.MaterialBindingAPI.Apply(owner).Bind(other, materialPurpose="physics")
        elif mode == "strong_ancestor":
            other = UsdShade.Material.Define(stage, "/root/Physics/Other")
            mesh_binding.Bind(other, materialPurpose="physics")
            strength = UsdShade.Tokens.strongerThanDescendants
            UsdShade.MaterialBindingAPI.Apply(root).Bind(physics, bindingStrength=strength, materialPurpose="physics")

        expected = visual.GetPath() if mode == "visual_fallback" else physics.GetPath()
        material, _ = mesh_binding.ComputeBoundMaterial(materialPurpose="physics")
        self.assertEqual(material.GetPath(), expected)
        stage.Save()
        rule = GeometriesRoutingRule(stage, str(directory), "payloads", {"params": {}})
        self.assertIsNotNone(rule.process_rule())
        instance = rule.source_stage.GetPrimAtPath("/root/body" if merge else "/root/body/Mesh")
        self.assertTrue(instance.IsInstanceable())
        relationship = instance.GetRelationship("material:binding:physics")
        if mode == "visual_fallback":
            self.assertFalse(relationship and relationship.HasAuthoredTargets())
        else:
            self.assertTrue(instance.HasAPI(UsdShade.MaterialBindingAPI))
            self.assertEqual(relationship.GetTargets(), [expected])
            self.assertEqual(UsdShade.MaterialBindingAPI.GetMaterialBindingStrength(relationship), strength)
            material, _ = UsdShade.MaterialBindingAPI(instance).ComputeBoundMaterial(materialPurpose="physics")
            self.assertEqual(material.GetPath(), expected)
            for descendant in Usd.PrimRange(instance, Usd.TraverseInstanceProxies()):
                if descendant.IsA(UsdGeom.Mesh):
                    material, _ = UsdShade.MaterialBindingAPI(descendant).ComputeBoundMaterial(
                        materialPurpose="physics"
                    )
                    self.assertEqual(material.GetPath(), expected)

        instances = Usd.Stage.Open(str(directory / "payloads/instances.usda"))
        for prim in instances.Traverse():
            for relationship in prim.GetRelationships():
                name = relationship.GetName()
                if name == "material:binding:physics" or name.startswith("material:binding:collection:physics:"):
                    self.assertEqual(relationship.GetTargets(), [])


class TestTextureFolderCasing(unittest.TestCase):
    """Texture transfers and authored paths must agree on directory casing."""

    def test_texture_paths_use_default_override_and_existing_casing(self) -> None:
        """Check new packages, existing directories, overrides, and nested paths end to end."""
        cases = (
            (None, (), "textures"),
            (None, ("Textures",), "Textures"),
            ("Images", (), "Images"),
            ("images", ("IMAGES",), "IMAGES"),
            ("textures", ("Textures", "textures"), "textures"),
            ("assets/Images", (), "assets/Images"),
        )
        for configured, existing, expected in cases:
            with self.subTest(configured=configured, existing=existing), tempfile.TemporaryDirectory() as directory:
                package = Path(directory)
                for name in existing:
                    (package / name).mkdir(exist_ok=True)
                if len(list(package.iterdir())) != len(existing):
                    self.skipTest("Distinct directories differing only in case require a case-sensitive filesystem")
                source = package / "source"
                source.mkdir()
                texture = source / "albedo.png"
                texture.write_bytes(b"texture-content")
                stage = Usd.Stage.CreateNew(str(source / "scene.usda"))
                root = UsdGeom.Xform.Define(stage, "/root").GetPrim()
                stage.SetDefaultPrim(root)
                material = UsdShade.Material.Define(stage, "/root/Material")
                shader = UsdShade.Shader.Define(stage, "/root/Material/Texture")
                shader.CreateIdAttr("UsdUVTexture")
                shader.CreateInput("file", Sdf.ValueTypeNames.Asset).Set(Sdf.AssetPath(str(texture)))
                material.CreateSurfaceOutput().ConnectToSource(shader.ConnectableAPI(), "rgb")
                stage.Save()
                params = {"textures_folder": configured} if configured is not None else {}
                rule = MaterialsRoutingRule(stage, directory, "payloads", {"params": params})
                self.assertEqual(
                    next(p.default_value for p in rule.get_configuration_parameters() if p.name == "textures_folder"),
                    "textures",
                )
                rule.process_rule()
                destination = package / expected / texture.name
                self.assertEqual(destination.read_bytes(), b"texture-content")
                materials = Usd.Stage.Open(str(package / "payloads/materials.usda"))
                assets = [
                    prim.GetAttribute("inputs:file").Get()
                    for prim in materials.Traverse()
                    if prim.HasAttribute("inputs:file")
                ]
                self.assertTrue(assets)
                for asset in assets:
                    self.assertEqual(asset.path, f"../{expected}/albedo.png")
                    self.assertEqual(Path(asset.resolvedPath), destination)
                if expected == "Textures":
                    self.assertEqual(
                        sorted(p.name for p in package.iterdir() if p.name.lower() == "textures"), ["Textures"]
                    )


if __name__ == "__main__":
    unittest.main()
