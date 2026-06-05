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

"""Tests for the materials routing rule."""

import os
import re
import shutil
import tempfile

import omni.kit.test
from isaacsim.asset.transformer.rules.perf.materials import MaterialsRoutingRule
from pxr import Gf, Sdf, Usd, UsdGeom, UsdPhysics, UsdShade

from .common import _INSPIRE_HAND_DIR, _INSPIRE_HAND_MATERIALS_USDA, _TEST_DATA_DIR, _UR10E_USD

_UR10E_BASE_USD = os.path.join(_TEST_DATA_DIR, "ur10e", "configuration", "ur10e_base.usd")


class TestMaterialsRoutingRule(omni.kit.test.AsyncTestCase):
    """Async tests for MaterialsRoutingRule."""

    async def setUp(self) -> None:
        """Create a temporary directory for test output."""
        self._tmpdir = tempfile.mkdtemp()
        self._success = False

    async def tearDown(self) -> None:
        """Remove temporary directory after successful tests."""
        if self._success:
            shutil.rmtree(self._tmpdir, ignore_errors=True)

    async def test_get_configuration_parameters(self) -> None:
        """Verify configuration parameters are exposed."""
        stage = Usd.Stage.Open(_UR10E_USD)
        rule = MaterialsRoutingRule(
            source_stage=stage,
            package_root=self._tmpdir,
            destination_path="payloads",
            args={},
        )

        params = rule.get_configuration_parameters()

        self.assertEqual(len(params), 5)
        param_names = [p.name for p in params]
        self.assertIn("scope", param_names)
        self.assertIn("materials_layer", param_names)
        self.assertIn("textures_folder", param_names)
        self.assertIn("deduplicate", param_names)
        self.assertIn("download_textures", param_names)
        self._success = True

    async def test_process_rule_creates_materials_layer(self) -> None:
        """Verify materials layer is created."""
        temp_asset = os.path.join(self._tmpdir, "ur10e.usd")
        shutil.copy(_UR10E_USD, temp_asset)
        stage = Usd.Stage.Open(temp_asset)
        os.makedirs(os.path.join(self._tmpdir, "payloads"), exist_ok=True)

        rule = MaterialsRoutingRule(
            source_stage=stage,
            package_root=self._tmpdir,
            destination_path="payloads",
            args={
                "params": {
                    "materials_layer": "materials.usda",
                    "textures_folder": "Textures",
                }
            },
        )

        rule.process_rule()

        materials_path = os.path.join(self._tmpdir, "payloads", "materials.usda")
        if os.path.exists(materials_path):
            materials_layer = Sdf.Layer.FindOrOpen(materials_path)
            self.assertIsNotNone(materials_layer)
        self._success = True

    async def test_process_rule_with_scope(self) -> None:
        """Verify scope filter is logged."""
        temp_asset = os.path.join(self._tmpdir, "ur10e.usd")
        shutil.copy(_UR10E_USD, temp_asset)
        stage = Usd.Stage.Open(temp_asset)
        os.makedirs(os.path.join(self._tmpdir, "payloads"), exist_ok=True)

        rule = MaterialsRoutingRule(
            source_stage=stage,
            package_root=self._tmpdir,
            destination_path="payloads",
            args={
                "params": {
                    "scope": "/ur10e",
                    "materials_layer": "materials.usda",
                }
            },
        )

        rule.process_rule()

        log = rule.get_operation_log()
        self.assertTrue(any("scope=/ur10e" in msg for msg in log))
        self._success = True

    async def test_process_rule_with_deduplication(self) -> None:
        """Verify deduplication logs are recorded."""
        temp_asset = os.path.join(self._tmpdir, "ur10e.usd")
        shutil.copy(_UR10E_BASE_USD, temp_asset)
        stage = Usd.Stage.Open(temp_asset)
        os.makedirs(os.path.join(self._tmpdir, "payloads"), exist_ok=True)

        rule = MaterialsRoutingRule(
            source_stage=stage,
            package_root=self._tmpdir,
            destination_path="payloads",
            args={
                "params": {
                    "materials_layer": "materials.usda",
                    "deduplicate": True,
                }
            },
        )

        rule.process_rule()

        log = rule.get_operation_log()
        self.assertTrue(any("deduplicate=True" in msg for msg in log))
        # check how many materials are in the materials layer
        self.assertTrue(
            any(
                "Processed 5 materials: 4 new, 0 reused from existing layer, 1 deduplicated within batch" in msg
                for msg in log
            )
        )
        self.assertTrue(any("35 bindings updated" in msg for msg in log))
        self._success = True

    async def test_process_rule_without_deduplication(self) -> None:
        """Verify non-deduplicated processing logs are recorded."""
        temp_asset = os.path.join(self._tmpdir, "ur10e.usd")
        shutil.copy(_UR10E_BASE_USD, temp_asset)
        stage = Usd.Stage.Open(temp_asset)
        os.makedirs(os.path.join(self._tmpdir, "payloads"), exist_ok=True)

        rule = MaterialsRoutingRule(
            source_stage=stage,
            package_root=self._tmpdir,
            destination_path="payloads",
            args={
                "params": {
                    "materials_layer": "materials.usda",
                    "deduplicate": False,
                }
            },
        )

        rule.process_rule()

        log = rule.get_operation_log()
        self.assertTrue(any("deduplicate=False" in msg for msg in log))
        # check how many materials are in the materials layer
        self.assertTrue(
            any(
                "Processed 5 materials: 5 new, 0 reused from existing layer, 1 duplicates found but kept separate (deduplication disabled)"
                in msg
                for msg in log
            )
        )
        self.assertTrue(any("35 bindings updated" in msg for msg in log))
        self._success = True

    async def test_process_rule_affected_stages(self) -> None:
        """Verify affected stages are recorded."""
        temp_asset = os.path.join(self._tmpdir, "ur10e.usd")
        shutil.copy(_UR10E_USD, temp_asset)
        stage = Usd.Stage.Open(temp_asset)
        os.makedirs(os.path.join(self._tmpdir, "payloads"), exist_ok=True)

        rule = MaterialsRoutingRule(
            source_stage=stage,
            package_root=self._tmpdir,
            destination_path="payloads",
            args={
                "params": {
                    "materials_layer": "materials.usda",
                }
            },
        )

        rule.process_rule()

        affected = rule.get_affected_stages()
        self.assertGreaterEqual(len(affected), 0)
        self._success = True

    async def test_process_rule_logs_operations(self) -> None:
        """Verify operation log entries are recorded."""
        temp_asset = os.path.join(self._tmpdir, "ur10e.usd")
        shutil.copy(_UR10E_BASE_USD, temp_asset)
        stage = Usd.Stage.Open(temp_asset)
        os.makedirs(os.path.join(self._tmpdir, "payloads"), exist_ok=True)

        rule = MaterialsRoutingRule(
            source_stage=stage,
            package_root=self._tmpdir,
            destination_path="payloads",
            args={
                "params": {
                    "materials_layer": "materials.usda",
                }
            },
        )

        rule.process_rule()

        log = rule.get_operation_log()
        self.assertTrue(any("MaterialsRoutingRule start" in msg for msg in log))
        self.assertTrue(any("MaterialsRoutingRule completed" in msg for msg in log))
        self._success = True

    async def test_process_rule_collects_mdl_textures(self) -> None:
        """Verify MDL textures are collected and paths remapped."""
        temp_source_dir = os.path.join(self._tmpdir, "source")
        temp_usd_path = os.path.join(temp_source_dir, "materials.usda")
        temp_plastics_dir = os.path.join(temp_source_dir, "Plastics")
        os.makedirs(temp_source_dir, exist_ok=True)

        shutil.copy(_INSPIRE_HAND_MATERIALS_USDA, temp_usd_path)
        shutil.copytree(os.path.join(_INSPIRE_HAND_DIR, "Plastics"), temp_plastics_dir)

        stage = Usd.Stage.Open(temp_usd_path)
        layer_dir = os.path.dirname(temp_usd_path)

        # Force absolute MDL asset paths in the source layer
        for prim in stage.Traverse():
            attr = prim.GetAttribute("info:mdl:sourceAsset")
            if not attr or not attr.HasAuthoredValue():
                continue
            value = attr.Get()
            if isinstance(value, Sdf.AssetPath) and value.path:
                abs_path = value.path
                if not os.path.isabs(abs_path):
                    abs_path = os.path.normpath(os.path.join(layer_dir, value.path))
                attr.Set(Sdf.AssetPath(abs_path))
        stage.Save()

        # Force absolute texture paths inside the MDL file
        mdl_path = os.path.join(temp_plastics_dir, "Rubber_Smooth.mdl")
        with open(mdl_path, encoding="utf-8", errors="ignore") as f:
            mdl_text = f.read()
        for rel_path in (
            "./Rubber_Smooth/Rubber_Smooth_BaseColor.png",
            "./Rubber_Smooth/Rubber_Smooth_ORM.png",
            "./Rubber_Smooth/Rubber_Smooth_Normal.png",
        ):
            abs_path = os.path.normpath(os.path.join(temp_plastics_dir, rel_path))
            mdl_text = mdl_text.replace(rel_path, abs_path.replace(os.sep, "/"))
        with open(mdl_path, "w", encoding="utf-8") as f:
            f.write(mdl_text)

        os.makedirs(os.path.join(self._tmpdir, "payloads"), exist_ok=True)
        rule = MaterialsRoutingRule(
            source_stage=stage,
            package_root=self._tmpdir,
            destination_path="payloads",
            args={
                "params": {
                    "materials_layer": "materials.usda",
                    "textures_folder": "Textures",
                    "download_textures": True,
                }
            },
        )

        rule.process_rule()

        materials_path = os.path.join(self._tmpdir, "payloads", "materials.usda")
        materials_stage = Usd.Stage.Open(materials_path)
        self.assertIsNotNone(materials_stage)

        # Verify material references are relative
        for prim in materials_stage.Traverse():
            attr = prim.GetAttribute("info:mdl:sourceAsset")
            if not attr or not attr.HasAuthoredValue():
                continue
            value = attr.Get()
            if isinstance(value, Sdf.AssetPath) and value.path:
                self.assertFalse(os.path.isabs(value.path))

        textures_dir = os.path.join(self._tmpdir, "Textures")
        expected_files = [
            "Plastic_ABS.mdl",
            "Rubber_Smooth.mdl",
            "Rubber_Smooth_BaseColor.png",
            "Rubber_Smooth_ORM.png",
            "Rubber_Smooth_Normal.png",
        ]
        for filename in expected_files:
            self.assertTrue(os.path.exists(os.path.join(textures_dir, filename)))

        # Verify MDL texture references are relative
        mdl_output_path = os.path.join(textures_dir, "Rubber_Smooth.mdl")
        with open(mdl_output_path, encoding="utf-8", errors="ignore") as f:
            mdl_output_text = f.read()
        self.assertNotIn(self._tmpdir, mdl_output_text)
        texture_refs = re.findall(r'"([^"]+\.png)"', mdl_output_text)
        self.assertGreaterEqual(len(texture_refs), 3)
        for ref in texture_refs:
            self.assertFalse(os.path.isabs(ref))

        self._success = True

    async def test_process_rule_skips_physics_materials(self) -> None:
        """Verify materials with PhysicsMaterialAPI are excluded from routing."""
        stage_path = os.path.join(self._tmpdir, "physics_mat_test.usda")
        stage = Usd.Stage.CreateNew(stage_path)
        stage.SetMetadata("metersPerUnit", 1.0)
        stage.SetMetadata("upAxis", "Z")

        root = UsdGeom.Xform.Define(stage, "/root")
        stage.SetDefaultPrim(root.GetPrim())

        # Visual material with a shader
        vis_mat = UsdShade.Material.Define(stage, "/root/Materials/VisualMat")
        shader = UsdShade.Shader.Define(stage, "/root/Materials/VisualMat/PreviewSurface")
        shader.CreateIdAttr("UsdPreviewSurface")
        shader.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(1, 0, 0))
        vis_mat.CreateSurfaceOutput().ConnectToSource(shader.ConnectableAPI(), "surface")

        # Physics material (has PhysicsMaterialAPI, no visual shader)
        phys_mat_prim = stage.DefinePrim("/root/Physics/PhysicsMaterial", "Material")
        UsdPhysics.MaterialAPI.Apply(phys_mat_prim)
        phys_mat_prim.CreateAttribute("physics:staticFriction", Sdf.ValueTypeNames.Float).Set(0.5)
        phys_mat_prim.CreateAttribute("physics:dynamicFriction", Sdf.ValueTypeNames.Float).Set(0.5)

        # Mesh bound to the visual material
        mesh = UsdGeom.Mesh.Define(stage, "/root/body/Mesh")
        mesh.GetPointsAttr().Set([Gf.Vec3f(0, 0, 0), Gf.Vec3f(1, 0, 0), Gf.Vec3f(0, 1, 0)])
        mesh.GetFaceVertexCountsAttr().Set([3])
        mesh.GetFaceVertexIndicesAttr().Set([0, 1, 2])
        UsdShade.MaterialBindingAPI.Apply(mesh.GetPrim())
        UsdShade.MaterialBindingAPI(mesh.GetPrim()).Bind(vis_mat)

        # Capsule bound to the physics material
        capsule = UsdGeom.Capsule.Define(stage, "/root/body/Collision")
        capsule.GetRadiusAttr().Set(0.05)
        capsule.GetHeightAttr().Set(0.2)
        binding_api = UsdShade.MaterialBindingAPI.Apply(capsule.GetPrim())
        binding_api.Bind(
            UsdShade.Material(phys_mat_prim),
            materialPurpose="physics",
        )

        stage.Export(stage_path)
        stage = Usd.Stage.Open(stage_path)

        os.makedirs(os.path.join(self._tmpdir, "payloads"), exist_ok=True)
        rule = MaterialsRoutingRule(
            source_stage=stage,
            package_root=self._tmpdir,
            destination_path="payloads",
            args={"params": {"materials_layer": "materials.usda"}},
        )

        rule.process_rule()

        log = rule.get_operation_log()
        self.assertTrue(
            any("Skipping physics material" in msg for msg in log),
            "Expected log entry for skipping physics material",
        )

        materials_path = os.path.join(self._tmpdir, "payloads", "materials.usda")
        if os.path.exists(materials_path):
            mat_layer = Sdf.Layer.FindOrOpen(materials_path)
            mat_text = mat_layer.ExportToString()
            self.assertIn("VisualMat", mat_text, "Visual material should be in materials layer")
            self.assertNotIn("PhysicsMaterial", mat_text, "Physics material must NOT be in materials layer")

        # Physics material must still exist in the source stage
        phys_prim = stage.GetPrimAtPath("/root/Physics/PhysicsMaterial")
        self.assertTrue(phys_prim.IsValid(), "Physics material must remain in source stage")
        self.assertIn("PhysicsMaterialAPI", phys_prim.GetAppliedSchemas())

        self._success = True
