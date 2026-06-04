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

"""Tests for the geometries routing rule."""

import os
import shutil
import tempfile

import omni.kit.test
from isaacsim.asset.transformer.rules.perf.geometries import GeometriesRoutingRule
from pxr import Gf, Sdf, Usd, UsdGeom, UsdPhysics, UsdShade

from .common import _TEST_ADVANCED_USD, _TEST_COLLISION_FROM_VISUALS_USD, _UR10E_SHOULDER_USD

_TRANSFORM_TOLERANCE = 1e-6
_DECOMPOSE_TOLERANCE = 1e-6


def _build_mirrored_meshes_stage(stage_path: str) -> str:
    """Create a stage with two identical meshes under parents with different names.

    Simulates the structure produced by stage flattening when left/right robot parts
    share the same prototype mesh: both mesh prims have the same leaf name but sit
    under parents with distinct per-instance names.  The right-side parent also
    carries a negative-X scale to mirror the geometry.

    Args:
        stage_path: File path where the stage will be exported.

    Returns:
        The file path of the exported stage.

    """
    stage = Usd.Stage.CreateNew(stage_path)
    stage.SetMetadata("metersPerUnit", 1.0)
    stage.SetMetadata("upAxis", "Z")

    root = UsdGeom.Xform.Define(stage, "/root")
    stage.SetDefaultPrim(root.GetPrim())

    # Shared triangle data
    face_vertex_counts = [3]
    face_vertex_indices = [0, 1, 2]
    points = [Gf.Vec3f(0, 0, 0), Gf.Vec3f(1, 0, 0), Gf.Vec3f(0, 1, 0)]

    # --- left side (identity scale) ---
    left_parent = UsdGeom.Xform.Define(stage, "/root/left_link")
    left_xf = left_parent.AddTranslateOp()
    left_xf.Set(Gf.Vec3d(-1, 0, 0))

    left_mesh = UsdGeom.Mesh.Define(stage, "/root/left_link/shared_mesh")
    left_mesh.GetFaceVertexCountsAttr().Set(face_vertex_counts)
    left_mesh.GetFaceVertexIndicesAttr().Set(face_vertex_indices)
    left_mesh.GetPointsAttr().Set(points)

    # --- right side (negative-X scale for mirroring) ---
    right_parent = UsdGeom.Xform.Define(stage, "/root/right_link")
    right_parent.AddTranslateOp().Set(Gf.Vec3d(1, 0, 0))
    right_parent.AddOrientOp().Set(Gf.Quatf(0.7071068, 0, 0, 0.7071068))
    right_parent.AddScaleOp().Set(Gf.Vec3f(-1, 1, 1))

    right_mesh = UsdGeom.Mesh.Define(stage, "/root/right_link/shared_mesh")
    right_mesh.GetFaceVertexCountsAttr().Set(face_vertex_counts)
    right_mesh.GetFaceVertexIndicesAttr().Set(face_vertex_indices)
    right_mesh.GetPointsAttr().Set(points)

    stage.Export(stage_path)
    return stage_path


def _build_no_mesh_stage(stage_path: str) -> str:
    """Create a stage with only non-Mesh Gprim geometry (Spheres, Capsules).

    Simulates assets like MuJoCo-converted robots (e.g. ant) that contain
    only primitive collision shapes and no Mesh prims.

    Args:
        stage_path: File path where the stage will be exported.

    Returns:
        The file path of the exported stage.

    """
    stage = Usd.Stage.CreateNew(stage_path)
    stage.SetMetadata("metersPerUnit", 1.0)
    stage.SetMetadata("upAxis", "Z")

    root = UsdGeom.Xform.Define(stage, "/root")
    stage.SetDefaultPrim(root.GetPrim())

    body = UsdGeom.Xform.Define(stage, "/root/body")
    body.AddTranslateOp().Set(Gf.Vec3d(0, 0, 0.75))

    sphere = UsdGeom.Sphere.Define(stage, "/root/body/torso_geom")
    sphere.GetRadiusAttr().Set(0.25)

    capsule = UsdGeom.Capsule.Define(stage, "/root/body/leg_geom")
    capsule.GetRadiusAttr().Set(0.08)
    capsule.GetHeightAttr().Set(0.28)

    stage.Export(stage_path)
    return stage_path


class TestGeometriesRoutingRule(omni.kit.test.AsyncTestCase):
    """Async tests for GeometriesRoutingRule."""

    async def setUp(self) -> None:
        """Create a temporary stage for geometry routing tests."""
        self._tmpdir = tempfile.mkdtemp()
        self._temp_asset = os.path.join(self._tmpdir, "payloads/base.usd")
        stage = Usd.Stage.Open(_UR10E_SHOULDER_USD)
        stage.Export(self._temp_asset)
        self._success = False

    async def tearDown(self) -> None:
        """Remove temporary directory after successful tests."""
        if self._success:
            shutil.rmtree(self._tmpdir, ignore_errors=True)

    async def test_get_configuration_parameters(self) -> None:
        """Verify configuration parameters are exposed."""
        stage = Usd.Stage.Open(_UR10E_SHOULDER_USD)
        rule = GeometriesRoutingRule(
            source_stage=stage,
            package_root=self._tmpdir,
            destination_path="payloads",
            args={},
        )

        params = rule.get_configuration_parameters()

        self.assertEqual(len(params), 6)
        param_names = [p.name for p in params]
        self.assertIn("scope", param_names)
        self.assertIn("geometries_layer", param_names)
        self.assertIn("instance_layer", param_names)
        self.assertIn("deduplicate", param_names)
        self.assertIn("verbose", param_names)
        self.assertIn("save_base_as_usda", param_names)
        self._success = True

    async def test_process_rule_creates_geometries_layer(self) -> None:
        """Verify geometries and instances layers are created."""
        base_stage = Usd.Stage.Open(self._temp_asset)
        os.makedirs(os.path.join(self._tmpdir, "payloads"), exist_ok=True)

        rule = GeometriesRoutingRule(
            source_stage=base_stage,
            package_root=self._tmpdir,
            destination_path="payloads",
            args={
                "params": {
                    "geometries_layer": "geometries.usd",
                    "instance_layer": "instances.usda",
                }
            },
        )

        updated_stage_path = rule.process_rule()
        base_stage = Usd.Stage.Open(updated_stage_path)
        log = rule.get_operation_log()

        geometries_path = os.path.join(self._tmpdir, "payloads", "geometries.usd")
        if os.path.exists(geometries_path):
            geometries_layer = Sdf.Layer.FindOrOpen(geometries_path)
            self.assertIsNotNone(geometries_layer)
        # Open the geometries layer and check if it contains the correct number of geometries
        geometries_stage = Usd.Stage.Open(geometries_path)

        # Assert that the geometries layer contains the correct number of geometries
        self.assertEqual(len([prim for prim in geometries_stage.Traverse() if UsdGeom.Gprim(prim)]), 2)
        # Assert that the instances layer contains the correct number of instances
        instances_stage = Usd.Stage.Open(os.path.join(self._tmpdir, "payloads", "instances.usda"))
        self.assertEqual(len(list(instances_stage.GetPrimAtPath("/Instances").GetChildren())), 6)
        # Assert that the base stage has the correct architecture for meshes
        mesh_prim_paths_expected = {
            "/ur10e/torso_link/visuals/head_link/Cube_01",
            "/ur10e/shoulder_link/duplicated_instanceable_01/shoulder/Cube_01",
            "/ur10e/shoulder_link/duplicated_direct_mesh/direct_mesh_material",
            "/ur10e/shoulder_link/duplicated_instanceable_04/shoulder/Cube_01",
            "/ur10e/shoulder_link/direct_mesh_material/direct_mesh_material",
            "/ur10e/shoulder_link/duplicated_instanceable/shoulder/Cube_01",
            "/ur10e/torso_link/visuals_01/head_link/Cube_01",
            "/ur10e/shoulder_link/convex_hull_instanceable/Cube_01",
            "/ur10e/shoulder_link/duplicated_non_instanceable/shoulder/Cube_01",
            "/ur10e/torso_link/visuals/logo_link/Cube_01",
            "/ur10e/shoulder_link/duplicated_instanceable_03/shoulder/Cube_01",
            "/ur10e/shoulder_link/instanceable_collider_no_material_03/shoulder/Cube_01",
            "/ur10e/torso_link/visuals_01/torso_link_rev_1_0/Cube_01",
            "/ur10e/shoulder_link/duplicated_instanceable_02/shoulder/Cube_01",
            "/ur10e/shoulder_link/instanceable_collider_no_material/shoulder/Cube_01",
            "/ur10e/shoulder_link/instanceable_visuals_materials/shoulder/Cube_01",
            "/ur10e/torso_link/visuals_01/logo_link/Cube_01",
            "/ur10e/torso_link/visuals/torso_link_rev_1_0/Cube_01",
            "/ur10e/shoulder_link/instanceable_collider_no_material_2/shoulder/Cube_01",
        }
        mesh_global_transforms_expected = {
            "/ur10e/shoulder_link/instanceable_visuals_materials/shoulder/Cube_01": Gf.Matrix4d(
                0.04999999999999935,
                8.742277645101089e-09,
                0.0,
                0.0,
                -8.742277645101089e-09,
                0.04999999999999935,
                0.0,
                0.0,
                0.0,
                0.0,
                0.05,
                0.0,
                0.0,
                0.0,
                0.18070000410079956,
                1.0,
            ),
            "/ur10e/shoulder_link/direct_mesh_material/direct_mesh_material": Gf.Matrix4d(
                -0.04999999999999985,
                -4.371138822550555e-09,
                0.0,
                0.0,
                4.371138822550555e-09,
                -0.04999999999999985,
                0.0,
                0.0,
                0.0,
                0.0,
                0.05,
                0.0,
                0.0,
                0.0,
                0.35153456480788414,
                1.0,
            ),
            "/ur10e/shoulder_link/convex_hull_instanceable/Cube_01": Gf.Matrix4d(
                1.1102230357117887e-17,
                -1.268162312721288e-18,
                -0.05,
                0.0,
                -4.371138815159158e-09,
                0.04999999999999985,
                -1.268163283309083e-18,
                0.0,
                0.04999999999999985,
                4.371138815159158e-09,
                1.1102230246251566e-17,
                0.0,
                0.0,
                0.0,
                0.0,
                1.0,
            ),
            "/ur10e/shoulder_link/instanceable_collider_no_material/shoulder/Cube_01": Gf.Matrix4d(
                0.04999999999999935,
                8.742277645101089e-09,
                0.0,
                0.0,
                -8.742277645101089e-09,
                0.04999999999999935,
                0.0,
                0.0,
                0.0,
                0.0,
                0.05,
                0.0,
                -0.2030558236568815,
                -2.812414082680294e-22,
                0.18070000410079218,
                1.0,
            ),
            "/ur10e/shoulder_link/instanceable_collider_no_material_2/shoulder/Cube_01": Gf.Matrix4d(
                0.04999999999999935,
                8.742277645101089e-09,
                0.0,
                0.0,
                -8.742277645101089e-09,
                0.04999999999999935,
                0.0,
                0.0,
                0.0,
                0.0,
                0.05,
                0.0,
                -0.2030558236568815,
                -2.812414082680294e-22,
                0.18070000410079218,
                1.0,
            ),
            "/ur10e/shoulder_link/instanceable_collider_no_material_03/shoulder/Cube_01": Gf.Matrix4d(
                0.04999999999999935,
                8.742277645101089e-09,
                0.0,
                0.0,
                -8.742277645101089e-09,
                0.04999999999999935,
                0.0,
                0.0,
                0.0,
                0.0,
                0.05,
                0.0,
                -0.4400958513209592,
                -7.874759431504823e-22,
                0.18070000410078613,
                1.0,
            ),
            "/ur10e/shoulder_link/duplicated_direct_mesh/direct_mesh_material": Gf.Matrix4d(
                -0.035355339059327265,
                -3.090861902933278e-09,
                -0.03535533905932738,
                0.0,
                4.371138822550555e-09,
                -0.04999999999999985,
                0.0,
                0.0,
                -0.03535533905932727,
                -3.0908619029332784e-09,
                0.035355339059327376,
                0.0,
                0.15872725025768017,
                3.1763735522036263e-22,
                0.35153456480788403,
                1.0,
            ),
            "/ur10e/shoulder_link/duplicated_instanceable/shoulder/Cube_01": Gf.Matrix4d(
                2.220446106736341e-17,
                -6.5756339423677346e-18,
                -0.05000000000000002,
                0.0,
                -8.742277638525446e-09,
                0.04999999999999932,
                -6.575637207032326e-18,
                0.0,
                0.04999999999999932,
                8.742277638525446e-09,
                1.1102230246251574e-17,
                0.0,
                0.2339776999529507,
                3.308722450212111e-24,
                0.18070000410079662,
                1.0,
            ),
            "/ur10e/shoulder_link/duplicated_non_instanceable/shoulder/Cube_01": Gf.Matrix4d(
                2.220446106736341e-17,
                -6.5756339423677346e-18,
                -0.05000000000000002,
                0.0,
                -8.742277638525446e-09,
                0.04999999999999932,
                -6.575637207032326e-18,
                0.0,
                0.04999999999999932,
                8.742277638525446e-09,
                1.1102230246251574e-17,
                0.0,
                0.4500473744420806,
                1.8889410850703915e-08,
                0.18070000410078624,
                1.0,
            ),
            "/ur10e/shoulder_link/duplicated_instanceable_01/shoulder/Cube_01": Gf.Matrix4d(
                2.220446106736341e-17,
                -6.5756339423677346e-18,
                -0.05000000000000002,
                0.0,
                -8.742277638525446e-09,
                0.04999999999999932,
                -6.575637207032326e-18,
                0.0,
                0.04999999999999932,
                8.742277638525446e-09,
                1.1102230246251574e-17,
                0.0,
                0.23397769995295267,
                1.5881867761018131e-22,
                -0.06804603563804534,
                1.0,
            ),
            "/ur10e/shoulder_link/duplicated_instanceable_02/shoulder/Cube_01": Gf.Matrix4d(
                1.1102228984499468e-17,
                1.4432762274116413e-17,
                0.05,
                0.0,
                -4.371138830860082e-09,
                0.04999999999999985,
                -1.4432761303528665e-17,
                0.0,
                -0.04999999999999985,
                -4.371138830860082e-09,
                1.1102230246251566e-17,
                0.0,
                0.2339776999529581,
                6.518183226917858e-22,
                -0.5031511428954939,
                1.0,
            ),
            "/ur10e/shoulder_link/duplicated_instanceable_03/shoulder/Cube_01": Gf.Matrix4d(
                0.04999999999999983,
                4.371138815974918e-09,
                1.7114271111395855e-09,
                0.0,
                -4.371138666357197e-09,
                0.049999999999999704,
                -4.371138815974914e-09,
                0.0,
                -1.711427482174436e-09,
                4.371138666357196e-09,
                0.04999999999999982,
                0.0,
                0.23397769995295478,
                3.341809674714232e-22,
                -0.2692127111122363,
                1.0,
            ),
            "/ur10e/shoulder_link/duplicated_instanceable_04/shoulder/Cube_01": Gf.Matrix4d(
                2.220446106736341e-17,
                -6.5756339423677346e-18,
                -0.05000000000000002,
                0.0,
                -8.742277638525446e-09,
                0.04999999999999932,
                -6.575637207032326e-18,
                0.0,
                0.04999999999999932,
                8.742277638525446e-09,
                1.1102230246251574e-17,
                0.0,
                0.2339776999529561,
                0.2799383068893438,
                -0.06804603563804745,
                1.0,
            ),
            "/ur10e/torso_link/visuals/torso_link_rev_1_0/Cube_01": Gf.Matrix4d(
                2.2204460492503132e-17,
                0.0,
                -0.1,
                0.0,
                0.0,
                0.1,
                0.0,
                0.0,
                0.1,
                0.0,
                2.2204460492503132e-17,
                0.0,
                -0.003963499795645475,
                -0.5374244968549189,
                0.04399999976158142,
                1.0,
            ),
            "/ur10e/torso_link/visuals/head_link/Cube_01": Gf.Matrix4d(
                5.551115123125783e-18,
                0.05,
                -8.326672684688675e-18,
                0.0,
                0.0,
                5.551115123125783e-18,
                0.05,
                0.0,
                0.05,
                -8.326672684688675e-18,
                0.0,
                0.0,
                0.0,
                -0.5374244968549189,
                0.2,
                1.0,
            ),
            "/ur10e/torso_link/visuals/logo_link/Cube_01": Gf.Matrix4d(
                4.440892098500626e-18,
                0.0,
                -0.02,
                0.0,
                0.0,
                0.1,
                0.0,
                0.0,
                0.05,
                0.0,
                1.1102230246251566e-17,
                0.0,
                0.052638901207180566,
                -0.5374244968549189,
                0.0,
                1.0,
            ),
            "/ur10e/torso_link/visuals_01/torso_link_rev_1_0/Cube_01": Gf.Matrix4d(
                2.2204460492503132e-17,
                0.0,
                -0.1,
                0.0,
                0.0,
                0.1,
                0.0,
                0.0,
                0.1,
                0.0,
                2.2204460492503132e-17,
                0.0,
                -0.003963499795645475,
                -0.33804472772541205,
                0.04399999976158142,
                1.0,
            ),
            "/ur10e/torso_link/visuals_01/head_link/Cube_01": Gf.Matrix4d(
                5.551115123125783e-18,
                0.05,
                -8.326672684688675e-18,
                0.0,
                0.0,
                5.551115123125783e-18,
                0.05,
                0.0,
                0.05,
                -8.326672684688675e-18,
                0.0,
                0.0,
                0.0,
                -0.33804472772541205,
                0.2,
                1.0,
            ),
            "/ur10e/torso_link/visuals_01/logo_link/Cube_01": Gf.Matrix4d(
                4.440892098500626e-18,
                0.0,
                -0.02,
                0.0,
                0.0,
                0.1,
                0.0,
                0.0,
                0.05,
                0.0,
                1.1102230246251566e-17,
                0.0,
                0.052638901207180566,
                -0.33804472772541205,
                0.0,
                1.0,
            ),
        }
        # Open the base stage and get the paths of the mesh prims
        mesh_prims = {
            p.GetPath().pathString
            for p in Usd.PrimRange(base_stage.GetPrimAtPath("/"), Usd.TraverseInstanceProxies())
            if UsdGeom.Gprim(p)
        }
        self.assertEqual(mesh_prim_paths_expected, mesh_prims)
        xform_cache = UsdGeom.XformCache()
        mesh_global_transforms = {
            p.GetPath().pathString: xform_cache.GetLocalToWorldTransform(p)
            for p in Usd.PrimRange(base_stage.GetPrimAtPath("/"), Usd.TraverseInstanceProxies())
            if UsdGeom.Gprim(p)
        }
        self.assertEqual(
            set(mesh_global_transforms_expected.keys()),
            set(mesh_global_transforms.keys()),
            "Transform dict keys mismatch",
        )
        for prim_path, expected_xform in mesh_global_transforms_expected.items():
            actual_xform = mesh_global_transforms[prim_path]
            for row in range(4):
                for col in range(4):
                    self.assertAlmostEqual(
                        expected_xform[row][col],
                        actual_xform[row][col],
                        msg=f"Transform mismatch at {prim_path}[{row}][{col}]",
                        delta=_TRANSFORM_TOLERANCE,
                    )
        self._success = True

    async def test_process_rule_name_clash_instances(self) -> None:
        """Verify meshes with name clashes are routed to unique instances."""
        os.makedirs(os.path.join(self._tmpdir, "payloads"), exist_ok=True)
        temp_input = os.path.join(self._tmpdir, "test_advanced.usda")

        source_stage = Usd.Stage.Open(_TEST_ADVANCED_USD)
        source_stage.Export(temp_input)
        base_stage = Usd.Stage.Open(temp_input)

        rule = GeometriesRoutingRule(
            source_stage=base_stage,
            package_root=self._tmpdir,
            destination_path="payloads",
            args={
                "params": {
                    "geometries_layer": "geometries.usd",
                    "instance_layer": "instances.usda",
                }
            },
        )

        updated_stage_path = rule.process_rule()
        updated_stage = Usd.Stage.Open(updated_stage_path)

        geometries_stage = Usd.Stage.Open(os.path.join(self._tmpdir, "payloads", "geometries.usd"))
        instances_stage = Usd.Stage.Open(os.path.join(self._tmpdir, "payloads", "instances.usda"))

        mesh_geometries = [prim for prim in geometries_stage.Traverse() if prim.IsA(UsdGeom.Mesh)]
        self.assertEqual(len(mesh_geometries), 2)

        # Verify non-Mesh Gprims (Cylinders, Cubes, etc.) were NOT routed to geometries
        non_mesh_in_geometries = [
            prim for prim in geometries_stage.Traverse() if prim.IsA(UsdGeom.Gprim) and not prim.IsA(UsdGeom.Mesh)
        ]
        self.assertEqual(
            len(non_mesh_in_geometries), 0, f"Non-Mesh Gprims found in geometries: {non_mesh_in_geometries}"
        )

        instances_root = instances_stage.GetPrimAtPath("/Instances")
        self.assertTrue(instances_root.IsValid())

        mesh_instances = []
        for instance_root in instances_root.GetChildren():
            for child in instance_root.GetChildren():
                if child.IsA(UsdGeom.Mesh):
                    mesh_instances.append((instance_root, child))
                    break

        self.assertEqual(len(mesh_instances), 4)

        instances_layer = instances_stage.GetRootLayer()
        geometry_ref_counts = {}
        for instance_root, _ in mesh_instances:
            prim_spec = instances_layer.GetPrimAtPath(instance_root.GetPath())
            self.assertIsNotNone(prim_spec)
            references = list(prim_spec.referenceList.GetAddedOrExplicitItems())
            references.extend(prim_spec.referenceList.prependedItems)
            self.assertTrue(references)
            geometry_path = references[0].primPath.pathString
            geometry_ref_counts[geometry_path] = geometry_ref_counts.get(geometry_path, 0) + 1

        self.assertEqual(len(geometry_ref_counts), 2)
        self.assertTrue(all(count == 2 for count in geometry_ref_counts.values()))

        source_layer = updated_stage.GetRootLayer()
        source_mesh_paths = [
            "/test_advanced/Geometry/root_link/base_link/link_1/cylinder",
            "/test_advanced/Geometry/root_link/base_link/link_1/link_2/cylinder",
            "/test_advanced/Geometry/root_link/base_link/link_1/cylinder_1",
            "/test_advanced/Geometry/root_link/base_link/link_1/link_2/cylinder_1",
        ]
        expected_guide_paths = {
            "/test_advanced/Geometry/root_link/base_link/link_1/cylinder_1",
            "/test_advanced/Geometry/root_link/base_link/link_1/link_2/cylinder_1",
        }

        referenced_instances = set()
        for source_path in source_mesh_paths:
            prim_spec = source_layer.GetPrimAtPath(source_path)
            self.assertIsNotNone(prim_spec)

            references = list(prim_spec.referenceList.GetAddedOrExplicitItems())
            references.extend(prim_spec.referenceList.prependedItems)
            self.assertTrue(references)

            instance_path = references[0].primPath.pathString
            referenced_instances.add(instance_path)

            instance_root = instances_stage.GetPrimAtPath(instance_path)
            self.assertTrue(instance_root.IsValid())

            instance_mesh = None
            for child in instance_root.GetChildren():
                if child.IsA(UsdGeom.Mesh):
                    instance_mesh = child
                    break

            self.assertIsNotNone(instance_mesh)
            purpose_attr = instance_mesh.GetAttribute("purpose")
            has_guide = bool(purpose_attr and purpose_attr.HasAuthoredValue() and purpose_attr.Get() == "guide")

            if source_path in expected_guide_paths:
                self.assertTrue(has_guide)
            else:
                self.assertFalse(has_guide)

        self.assertEqual(len(referenced_instances), 4)

        # Verify the Cylinder prims in the source stage were NOT affected by the rule
        cyl_prim = updated_stage.GetPrimAtPath("/test_advanced/Geometry/root_link/base_link/link_1/visual_cyl")
        self.assertTrue(cyl_prim.IsValid())
        self.assertTrue(cyl_prim.IsA(UsdGeom.Cylinder))

        self._success = True

    async def test_process_rule_inherits_guide_from_grandparent_and_separates_visual(self) -> None:
        """Verify inherited purpose=guide on an ancestor is propagated and visual/collision usages are not merged.

        Builds a stage where the same prototype mesh is referenced both by a visual Xform
        (effective purpose=default) and by a collision Xform whose grandparent (not the
        immediate parent) authors purpose=guide. Verifies that:

        1. The deduplicated visual and collision sources end up referencing different
           ``/Instances`` entries -- without per-effective-purpose deduplication they
           would share one instance and the visual usage would be incorrectly hidden.
        2. The collision-side prototype mesh carries an authored purpose=guide attribute
           inside the instances layer, even though the value is inherited from an
           ancestor several levels above the mesh rather than from the immediate parent.
        3. The visual-side prototype mesh does NOT carry an authored purpose attribute.
        """
        os.makedirs(os.path.join(self._tmpdir, "payloads"), exist_ok=True)
        temp_input = os.path.join(self._tmpdir, "grandparent_guide.usda")

        # Build a stage with a shared prototype mesh under /meshes, referenced once for
        # visuals (no purpose) and once via a collisions group whose GRANDPARENT carries
        # purpose=guide. Purpose inheritance must be picked up from the grandparent.
        stage = Usd.Stage.CreateNew(temp_input)
        stage.SetMetadata("metersPerUnit", 1.0)
        stage.SetMetadata("upAxis", "Z")
        root = UsdGeom.Xform.Define(stage, "/root")
        stage.SetDefaultPrim(root.GetPrim())

        proto = UsdGeom.Xform.Define(stage, "/meshes/proto")
        proto_mesh = UsdGeom.Mesh.Define(stage, "/meshes/proto/mesh")
        proto_mesh.GetFaceVertexCountsAttr().Set([3])
        proto_mesh.GetFaceVertexIndicesAttr().Set([0, 1, 2])
        proto_mesh.GetPointsAttr().Set([Gf.Vec3f(0, 0, 0), Gf.Vec3f(1, 0, 0), Gf.Vec3f(0, 1, 0)])

        visual = UsdGeom.Xform.Define(stage, "/root/visuals/part")
        visual.GetPrim().GetReferences().AddInternalReference("/meshes/proto")

        collisions_group = UsdGeom.Xform.Define(stage, "/root/collisions")
        collisions_group.GetPrim().GetAttribute("purpose").Set(UsdGeom.Tokens.guide)
        collision_link = UsdGeom.Xform.Define(stage, "/root/collisions/link")
        collision = UsdGeom.Xform.Define(stage, "/root/collisions/link/part")
        collision.GetPrim().GetReferences().AddInternalReference("/meshes/proto")

        stage.Export(temp_input)

        base_stage = Usd.Stage.Open(temp_input)
        rule = GeometriesRoutingRule(
            source_stage=base_stage,
            package_root=self._tmpdir,
            destination_path="payloads",
            args={
                "params": {
                    "geometries_layer": "geometries.usd",
                    "instance_layer": "instances.usda",
                }
            },
        )

        updated_stage_path = rule.process_rule()
        self.assertIsNotNone(updated_stage_path)
        updated_stage = Usd.Stage.Open(updated_stage_path)
        instances_stage = Usd.Stage.Open(os.path.join(self._tmpdir, "payloads", "instances.usda"))

        def _instance_path_for(prim_path: str) -> str:
            prim_spec = updated_stage.GetRootLayer().GetPrimAtPath(prim_path)
            self.assertIsNotNone(prim_spec, f"No spec at {prim_path}")
            refs = list(prim_spec.referenceList.GetAddedOrExplicitItems())
            refs.extend(prim_spec.referenceList.prependedItems)
            self.assertTrue(refs, f"Expected reference on {prim_path}")
            return refs[0].primPath.pathString

        visual_inst_path = _instance_path_for("/root/visuals/part")
        collision_inst_path = _instance_path_for("/root/collisions/link/part")
        self.assertNotEqual(
            visual_inst_path,
            collision_inst_path,
            "Visual and collision usages of the same prototype must not share an instance",
        )

        def _proto_mesh_purpose(instance_root_path: str) -> tuple[bool, str]:
            instance_root = instances_stage.GetPrimAtPath(instance_root_path)
            self.assertTrue(instance_root.IsValid(), f"Missing instance root {instance_root_path}")
            for child in instance_root.GetChildren():
                if child.IsA(UsdGeom.Mesh):
                    attr = child.GetAttribute("purpose")
                    return bool(attr and attr.HasAuthoredValue()), attr.Get() if attr else ""
            self.fail(f"No Mesh child in {instance_root_path}")

        collision_authored, collision_value = _proto_mesh_purpose(collision_inst_path)
        self.assertTrue(
            collision_authored,
            "Collision prototype mesh must carry an authored purpose from inherited ancestor value",
        )
        self.assertEqual(collision_value, UsdGeom.Tokens.guide)

        visual_authored, _ = _proto_mesh_purpose(visual_inst_path)
        self.assertFalse(visual_authored, "Visual prototype mesh must not carry an authored purpose")

        self._success = True

    async def test_process_rule_is_idempotent_with_existing_instances_layer(self) -> None:
        """Verify re-running the rule against an existing instances layer is safe.

        Re-running must not crash AND must not mutate the source asset on disk.

        Reproduces two regressions:
        1. ``Sdf.Layer.FindOrOpen`` reuses a previously written ``instances.usda`` that
           already contains the propagated purpose attribute spec on a prototype mesh.
           The second run must not raise
           ``Cannot create spec ... because it already exists`` when re-authoring the spec.
        2. ``_make_references_non_instanceable`` used to flatten-and-export back to the
           source layer's ``realPath``, silently mutating the caller's input between
           runs. ``process_rule`` must treat its source as read-only; the working copy
           lives inside ``package_root`` instead.
        """
        os.makedirs(os.path.join(self._tmpdir, "payloads"), exist_ok=True)
        temp_input = os.path.join(self._tmpdir, "test_advanced.usda")

        source_stage = Usd.Stage.Open(_TEST_ADVANCED_USD)
        source_stage.Export(temp_input)

        # Snapshot the input bytes so we can assert the rule does not mutate it.
        with open(temp_input, "rb") as fh:
            input_bytes_before = fh.read()
        input_mtime_before = os.path.getmtime(temp_input)

        def _run() -> str:
            base_stage = Usd.Stage.Open(temp_input)
            rule = GeometriesRoutingRule(
                source_stage=base_stage,
                package_root=self._tmpdir,
                destination_path="payloads",
                args={
                    "params": {
                        "geometries_layer": "geometries.usd",
                        "instance_layer": "instances.usda",
                    }
                },
            )
            return rule.process_rule()

        first_path = _run()
        self.assertIsNotNone(first_path, "Initial rule run failed")

        # Source file must be unchanged after first run (no in-place mutation).
        with open(temp_input, "rb") as fh:
            input_bytes_after_first = fh.read()
        self.assertEqual(
            input_bytes_before,
            input_bytes_after_first,
            "process_rule mutated the source asset file in place; it must operate on a working copy",
        )
        self.assertEqual(
            input_mtime_before,
            os.path.getmtime(temp_input),
            "Source file mtime changed; process_rule mutated the source asset",
        )

        # Second run on the same (unmutated) input must not raise. The instances
        # layer from the first run is reopened via Sdf.Layer.FindOrOpen and
        # previously-authored purpose specs must be reused.
        second_path = _run()
        self.assertIsNotNone(second_path, "Re-running the rule with existing instances layer failed")
        self._success = True

    async def test_process_rule_preserves_parent_properties_on_merge(self) -> None:
        """Verify merged parent retains authored properties and schemas."""
        os.makedirs(os.path.join(self._tmpdir, "payloads"), exist_ok=True)
        temp_input = os.path.join(self._tmpdir, "test_collision_from_visuals.usda")

        source_stage = Usd.Stage.Open(_TEST_COLLISION_FROM_VISUALS_USD)
        source_stage.Export(temp_input)
        base_stage = Usd.Stage.Open(temp_input)

        rule = GeometriesRoutingRule(
            source_stage=base_stage,
            package_root=self._tmpdir,
            destination_path="payloads",
            args={
                "params": {
                    "geometries_layer": "geometries.usd",
                    "instance_layer": "instances.usda",
                }
            },
        )

        updated_stage_path = rule.process_rule()
        updated_stage = Usd.Stage.Open(updated_stage_path)

        prim_path = "/test_collision_from_visuals/Geometry/root_link/base_link/link_1/link_2/palm_link/finger_link_2"
        prim = updated_stage.GetPrimAtPath(prim_path)
        self.assertTrue(prim.IsValid())
        self.assertTrue(prim.IsInstanceable())

        prim_spec = updated_stage.GetRootLayer().GetPrimAtPath(prim_path)
        self.assertIsNotNone(prim_spec)
        references = list(prim_spec.referenceList.GetAddedOrExplicitItems())
        references.extend(prim_spec.referenceList.prependedItems)
        self.assertTrue(references)

        applied_schemas = prim.GetAppliedSchemas()
        self.assertIn("PhysicsRigidBodyAPI", applied_schemas)
        self.assertIn("PhysicsMassAPI", applied_schemas)

        mass_attr = prim.GetAttribute("physics:mass")
        self.assertTrue(mass_attr and mass_attr.HasAuthoredValue())
        self.assertEqual(mass_attr.Get(), 3)

        inertia_attr = prim.GetAttribute("physics:diagonalInertia")
        self.assertTrue(inertia_attr and inertia_attr.HasAuthoredValue())
        self.assertEqual(inertia_attr.Get(), Gf.Vec3f(2, 3, 4))

        # Verify non-Mesh Gprims (Sphere) were NOT routed to geometries
        geometries_stage = Usd.Stage.Open(os.path.join(self._tmpdir, "payloads", "geometries.usd"))
        non_mesh_in_geometries = [
            prim for prim in geometries_stage.Traverse() if prim.IsA(UsdGeom.Gprim) and not prim.IsA(UsdGeom.Mesh)
        ]
        self.assertEqual(
            len(non_mesh_in_geometries), 0, f"Non-Mesh Gprims found in geometries: {non_mesh_in_geometries}"
        )

        # Verify the Sphere in the source stage was NOT affected by the rule
        sphere_prim = updated_stage.GetPrimAtPath("/test_collision_from_visuals/Geometry/root_link/base_link/sphere")
        self.assertTrue(sphere_prim.IsValid())
        self.assertTrue(sphere_prim.IsA(UsdGeom.Sphere))

        self._success = True

    async def test_process_rule_with_scope(self) -> None:
        """Verify scope filter limits geometry routing."""
        stage = Usd.Stage.Open(self._temp_asset)
        os.makedirs(os.path.join(self._tmpdir, "payloads"), exist_ok=True)

        rule = GeometriesRoutingRule(
            source_stage=stage,
            package_root=self._tmpdir,
            destination_path="payloads",
            args={
                "params": {
                    "scope": "/ur10e/base_link",
                    "geometries_layer": "geometries.usd",
                }
            },
        )

        updated_stage_path = rule.process_rule()
        updated_stage = Usd.Stage.Open(updated_stage_path)
        log = rule.get_operation_log()
        self.assertTrue(any("scope=/ur10e/base_link" in msg for msg in log))
        self._success = True

    async def test_process_rule_without_deduplication(self) -> None:
        """Verify non-deduplicated processing logs are recorded."""
        stage = Usd.Stage.Open(self._temp_asset)
        os.makedirs(os.path.join(self._tmpdir, "payloads"), exist_ok=True)

        rule = GeometriesRoutingRule(
            source_stage=stage,
            package_root=self._tmpdir,
            destination_path="payloads",
            args={
                "params": {
                    "geometries_layer": "geometries.usd",
                    "deduplicate": False,
                }
            },
        )

        rule.process_rule()

        log = rule.get_operation_log()
        self.assertTrue(any("deduplicate=False" in msg for msg in log))
        self._success = True

    async def test_process_rule_affected_stages(self) -> None:
        """Verify affected stages are recorded."""
        stage = Usd.Stage.Open(self._temp_asset)
        os.makedirs(os.path.join(self._tmpdir, "payloads"), exist_ok=True)

        rule = GeometriesRoutingRule(
            source_stage=stage,
            package_root=self._tmpdir,
            destination_path="payloads",
            args={
                "params": {
                    "geometries_layer": "geometries.usd",
                }
            },
        )

        rule.process_rule()

        affected = rule.get_affected_stages()
        self.assertGreaterEqual(len(affected), 0)
        self._success = True

    async def test_process_rule_logs_operations(self) -> None:
        """Verify operation log entries are recorded."""
        stage = Usd.Stage.Open(self._temp_asset)
        os.makedirs(os.path.join(self._tmpdir, "payloads"), exist_ok=True)

        rule = GeometriesRoutingRule(
            source_stage=stage,
            package_root=self._tmpdir,
            destination_path="payloads",
            args={
                "params": {
                    "geometries_layer": "geometries.usd",
                }
            },
        )

        rule.process_rule()

        log = rule.get_operation_log()
        self.assertTrue(any("GeometriesRoutingRule start" in msg for msg in log))
        self.assertTrue(any("GeometriesRoutingRule completed" in msg for msg in log))
        self._success = True

    # ------------------------------------------------------------------
    # Tests for negative-scale decomposition and deduplicate=False path
    # ------------------------------------------------------------------

    async def test_decompose_transform_negative_scale(self) -> None:
        """Verify _decompose_transform correctly handles negative scale (det < 0).

        When a transform matrix has a negative determinant (reflection), the
        decomposition must produce a negative scale component AND a pure
        rotation quaternion (no reflection baked in).  Re-composing the TOS
        transform must reproduce the original matrix within tolerance.
        """
        os.makedirs(os.path.join(self._tmpdir, "payloads"), exist_ok=True)
        stage = Usd.Stage.Open(self._temp_asset)

        rule = GeometriesRoutingRule(
            source_stage=stage,
            package_root=self._tmpdir,
            destination_path="payloads",
            args={},
        )

        # Build a matrix with translation, 90-degree Y rotation, and negative-X scale
        rotation = Gf.Rotation(Gf.Vec3d(0, 1, 0), 90.0)
        rot_matrix = Gf.Matrix4d(1.0).SetRotate(rotation)
        scale_matrix = Gf.Matrix4d().SetScale(Gf.Vec3d(-1, 1, 1))
        translate_matrix = Gf.Matrix4d(1.0).SetTranslate(Gf.Vec3d(3, 4, 5))
        # Row-vector convention: point * Scale * Rotate * Translate
        original = scale_matrix * rot_matrix * translate_matrix

        self.assertLess(original.GetDeterminant(), 0, "Test matrix must have negative determinant")

        translate, orient, scale = rule._decompose_transform(original)

        # Scale must have a negative component
        self.assertLess(scale[0], 0, "scale_x should be negative for reflected transform")
        self.assertAlmostEqual(abs(scale[0]), 1.0, delta=_DECOMPOSE_TOLERANCE)
        self.assertAlmostEqual(scale[1], 1.0, delta=_DECOMPOSE_TOLERANCE)
        self.assertAlmostEqual(scale[2], 1.0, delta=_DECOMPOSE_TOLERANCE)

        # Translation must match
        self.assertAlmostEqual(translate[0], 3.0, delta=_DECOMPOSE_TOLERANCE)
        self.assertAlmostEqual(translate[1], 4.0, delta=_DECOMPOSE_TOLERANCE)
        self.assertAlmostEqual(translate[2], 5.0, delta=_DECOMPOSE_TOLERANCE)

        # Re-compose the TOS transform and compare against the original matrix
        recomposed_scale = Gf.Matrix4d().SetScale(scale)
        recomposed_rot = Gf.Matrix4d(1.0).SetRotate(Gf.Rotation(orient))
        recomposed_translate = Gf.Matrix4d(1.0).SetTranslate(translate)
        recomposed = recomposed_scale * recomposed_rot * recomposed_translate

        for row in range(4):
            for col in range(4):
                self.assertAlmostEqual(
                    original[row, col],
                    recomposed[row, col],
                    delta=_DECOMPOSE_TOLERANCE,
                    msg=f"Recomposed matrix mismatch at [{row}][{col}]",
                )

        self._success = True

    async def test_no_deduplicate_produces_separate_geometries(self) -> None:
        """Verify deduplicate=False creates independent geometry and instance entries.

        Two meshes with identical vertex data but different parent names must each
        get their own geometry (named after the parent), their own instance, and
        their own reference in the base layer.  The world-space transform of each
        mesh must be preserved exactly.
        """
        os.makedirs(os.path.join(self._tmpdir, "payloads"), exist_ok=True)
        input_path = os.path.join(self._tmpdir, "mirrored.usda")
        _build_mirrored_meshes_stage(input_path)

        base_stage = Usd.Stage.Open(input_path)
        # Capture world transforms of the original meshes before processing
        xform_cache_before = UsdGeom.XformCache()
        left_world_before = xform_cache_before.GetLocalToWorldTransform(
            base_stage.GetPrimAtPath("/root/left_link/shared_mesh")
        )
        right_world_before = xform_cache_before.GetLocalToWorldTransform(
            base_stage.GetPrimAtPath("/root/right_link/shared_mesh")
        )

        rule = GeometriesRoutingRule(
            source_stage=base_stage,
            package_root=self._tmpdir,
            destination_path="payloads",
            args={
                "params": {
                    "geometries_layer": "geometries.usd",
                    "instance_layer": "instances.usda",
                    "deduplicate": False,
                }
            },
        )

        updated_path = rule.process_rule()
        self.assertIsNotNone(updated_path)

        # --- geometries.usd: must have two separate geometries ---
        geometries_stage = Usd.Stage.Open(os.path.join(self._tmpdir, "payloads", "geometries.usd"))
        geom_scope = geometries_stage.GetPrimAtPath("/Geometries")
        self.assertTrue(geom_scope.IsValid())
        geom_children = list(geom_scope.GetChildren())
        geom_names = {c.GetName() for c in geom_children}
        self.assertEqual(len(geom_children), 2, f"Expected 2 geometries, got {len(geom_children)}: {geom_names}")
        self.assertIn("left_link", geom_names, "Geometry for left parent missing")
        self.assertIn("right_link", geom_names, "Geometry for right parent missing")

        # --- instances.usda: must have two separate instances ---
        instances_stage = Usd.Stage.Open(os.path.join(self._tmpdir, "payloads", "instances.usda"))
        inst_scope = instances_stage.GetPrimAtPath("/Instances")
        self.assertTrue(inst_scope.IsValid())
        inst_children = list(inst_scope.GetChildren())
        inst_names = {c.GetName() for c in inst_children}
        self.assertEqual(len(inst_children), 2, f"Expected 2 instances, got {len(inst_children)}: {inst_names}")
        self.assertIn("left_link", inst_names, "Instance for left parent missing")
        self.assertIn("right_link", inst_names, "Instance for right parent missing")

        # --- base layer: references must point to the correct instances ---
        updated_stage = Usd.Stage.Open(updated_path)
        source_layer = updated_stage.GetRootLayer()

        for parent_name in ("left_link", "right_link"):
            expected_instance = f"/Instances/{parent_name}"
            parent_spec = source_layer.GetPrimAtPath(f"/root/{parent_name}")
            self.assertIsNotNone(parent_spec, f"Parent spec missing for {parent_name}")

            # The reference may be on the parent itself (merge case, when the parent
            # had only one child) or on a child prim (standard case).
            ref_holder = None
            parent_refs = list(parent_spec.referenceList.GetAddedOrExplicitItems())
            parent_refs.extend(parent_spec.referenceList.prependedItems)
            if parent_refs:
                ref_holder = parent_spec
            else:
                for child in parent_spec.nameChildren:
                    child_refs = list(child.referenceList.GetAddedOrExplicitItems())
                    child_refs.extend(child.referenceList.prependedItems)
                    if child_refs:
                        ref_holder = child
                        break
            self.assertIsNotNone(ref_holder, f"No reference found at or under /root/{parent_name}")
            all_refs = list(ref_holder.referenceList.GetAddedOrExplicitItems())
            all_refs.extend(ref_holder.referenceList.prependedItems)
            ref_target = all_refs[0].primPath.pathString
            self.assertEqual(
                ref_target,
                expected_instance,
                f"Reference at /root/{parent_name} points to {ref_target}, expected {expected_instance}",
            )

        # --- world transforms of composed meshes must match the originals ---
        xform_cache_after = UsdGeom.XformCache()
        for prim in Usd.PrimRange(updated_stage.GetPrimAtPath("/root"), Usd.TraverseInstanceProxies()):
            if not prim.IsA(UsdGeom.Mesh):
                continue
            path = prim.GetPath().pathString
            world_after = xform_cache_after.GetLocalToWorldTransform(prim)
            if "left_link" in path:
                expected = left_world_before
            elif "right_link" in path:
                expected = right_world_before
            else:
                continue
            for row in range(4):
                for col in range(4):
                    self.assertAlmostEqual(
                        expected[row, col],
                        world_after[row, col],
                        delta=_TRANSFORM_TOLERANCE,
                        msg=f"World transform mismatch at {path}[{row}][{col}]",
                    )

    async def test_no_geometry_prims_saves_base_as_usda(self) -> None:
        """Verify save_base_as_usda converts the base layer even when no Mesh prims exist.

        Assets like MuJoCo-converted robots (ant, humanoid) may contain only
        Spheres, Capsules, and other non-Mesh Gprims.  The GeometriesRoutingRule
        must still honour save_base_as_usda so downstream rules can locate the
        .usda base layer.
        """
        os.makedirs(os.path.join(self._tmpdir, "payloads"), exist_ok=True)
        input_path = os.path.join(self._tmpdir, "payloads", "base.usd")
        _build_no_mesh_stage(input_path)

        self.assertTrue(input_path.endswith(".usd"))
        self.assertTrue(os.path.exists(input_path))

        base_stage = Usd.Stage.Open(input_path)

        rule = GeometriesRoutingRule(
            source_stage=base_stage,
            package_root=self._tmpdir,
            destination_path="payloads",
            args={
                "params": {
                    "geometries_layer": "geometries.usd",
                    "instance_layer": "instances.usda",
                    "save_base_as_usda": True,
                }
            },
        )

        updated_path = rule.process_rule()

        log = rule.get_operation_log()
        self.assertTrue(any("No geometry prims found" in msg for msg in log))

        # The returned path must be the .usda variant
        expected_usda = os.path.splitext(input_path)[0] + ".usda"
        self.assertIsNotNone(updated_path, "process_rule must return the converted path, not None")
        self.assertEqual(updated_path, expected_usda)
        self.assertTrue(os.path.exists(expected_usda), f"Expected .usda file not found: {expected_usda}")

        # The original .usd binary must have been removed
        self.assertFalse(os.path.exists(input_path), "Original .usd should be deleted after conversion")

        # The converted file must be a valid stage with the original content preserved
        converted_stage = Usd.Stage.Open(expected_usda)
        self.assertIsNotNone(converted_stage)
        sphere = converted_stage.GetPrimAtPath("/root/body/torso_geom")
        self.assertTrue(sphere.IsValid())
        self.assertTrue(sphere.IsA(UsdGeom.Sphere))
        capsule = converted_stage.GetPrimAtPath("/root/body/leg_geom")
        self.assertTrue(capsule.IsValid())
        self.assertTrue(capsule.IsA(UsdGeom.Capsule))

        self._success = True

    async def test_process_rule_preserves_physics_material_bindings(self) -> None:
        """Verify physics material bindings survive geometry routing unchanged.

        A mesh with both a visual ``material:binding`` and a physics-purpose
        ``material:binding:physics`` must have the physics binding preserved
        in the instance delta with its original target path, while the visual
        binding is routed through VisualMaterials as usual.
        """
        os.makedirs(os.path.join(self._tmpdir, "payloads"), exist_ok=True)
        input_path = os.path.join(self._tmpdir, "phys_binding.usda")

        stage = Usd.Stage.CreateNew(input_path)
        stage.SetMetadata("metersPerUnit", 1.0)
        stage.SetMetadata("upAxis", "Z")

        root = UsdGeom.Xform.Define(stage, "/root")
        stage.SetDefaultPrim(root.GetPrim())

        # Visual material
        vis_mat = UsdShade.Material.Define(stage, "/root/Materials/VisualMat")
        shader = UsdShade.Shader.Define(stage, "/root/Materials/VisualMat/PreviewSurface")
        shader.CreateIdAttr("UsdPreviewSurface")
        shader.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(0.8, 0.2, 0.1))
        vis_mat.CreateSurfaceOutput().ConnectToSource(shader.ConnectableAPI(), "surface")

        # Physics material
        phys_mat_prim = stage.DefinePrim("/root/Physics/PhysicsMat", "Material")
        UsdPhysics.MaterialAPI.Apply(phys_mat_prim)
        phys_mat_prim.CreateAttribute("physics:staticFriction", Sdf.ValueTypeNames.Float).Set(0.8)

        # Xform parent with a single mesh child (will merge-up)
        UsdGeom.Xform.Define(stage, "/root/body")
        mesh = UsdGeom.Mesh.Define(stage, "/root/body/Mesh")
        mesh.GetPointsAttr().Set([Gf.Vec3f(0, 0, 0), Gf.Vec3f(1, 0, 0), Gf.Vec3f(0, 1, 0)])
        mesh.GetFaceVertexCountsAttr().Set([3])
        mesh.GetFaceVertexIndicesAttr().Set([0, 1, 2])

        # Bind both visual and physics materials
        binding_api = UsdShade.MaterialBindingAPI.Apply(mesh.GetPrim())
        binding_api.Bind(vis_mat)
        binding_api.Bind(
            UsdShade.Material(phys_mat_prim),
            materialPurpose="physics",
        )

        stage.Export(input_path)
        stage = Usd.Stage.Open(input_path)

        rule = GeometriesRoutingRule(
            source_stage=stage,
            package_root=self._tmpdir,
            destination_path="payloads",
            args={
                "params": {
                    "geometries_layer": "geometries.usd",
                    "instance_layer": "instances.usda",
                }
            },
        )

        updated_path = rule.process_rule()
        self.assertIsNotNone(updated_path)

        # Open the instances layer and find the instance prim
        instances_path = os.path.join(self._tmpdir, "payloads", "instances.usda")
        instances_stage = Usd.Stage.Open(instances_path)
        instances_root = instances_stage.GetPrimAtPath("/Instances")
        self.assertTrue(instances_root.IsValid())

        # Walk instance children to find a prim with material:binding:physics
        phys_binding_found = False
        for inst_child in Usd.PrimRange(instances_root):
            rel = inst_child.GetRelationship("material:binding:physics")
            if rel and rel.HasAuthoredTargets():
                targets = rel.GetTargets()
                self.assertEqual(len(targets), 1)
                self.assertEqual(
                    targets[0].pathString,
                    "/root/Physics/PhysicsMat",
                    "Physics material binding must point to the original path",
                )
                phys_binding_found = True
                break

        self.assertTrue(phys_binding_found, "No material:binding:physics found in instances layer")

        # Visual material binding must NOT point to the original material path
        # (it should go through VisualMaterials instead)
        vis_binding_found = False
        for inst_child in Usd.PrimRange(instances_root):
            rel = inst_child.GetRelationship("material:binding")
            if rel and rel.HasAuthoredTargets():
                targets = rel.GetTargets()
                for t in targets:
                    self.assertNotEqual(
                        t.pathString,
                        "/root/Materials/VisualMat",
                        "Visual material binding should be rerouted through VisualMaterials",
                    )
                vis_binding_found = True
                break

        self.assertTrue(vis_binding_found, "No visual material:binding found in instances layer")

        self._success = True

    async def test_multiple_sibling_meshes_not_merged_into_parent(self) -> None:
        """Verify that a parent Xform with multiple Mesh children keeps all children as separate references.

        Reproduces the anymal_d pattern where /robot/link/visuals has N mesh children.
        After geometry routing, each mesh should be its own instanceable reference under
        the parent — the rule must NOT merge the last mesh into the parent and lose the
        others.  This tests that _is_subtree_empty correctly treats siblings with
        composition arcs (references) as non-empty, preventing premature merging.
        """
        # Build a stage: /Robot/base/visuals has 5 Mesh children
        stage_path = os.path.join(self._tmpdir, "payloads", "base.usd")
        os.makedirs(os.path.dirname(stage_path), exist_ok=True)
        stage = Usd.Stage.CreateNew(stage_path)
        stage.SetDefaultPrim(stage.DefinePrim("/Robot", "Xform"))
        base = stage.DefinePrim("/Robot/base", "Xform")
        visuals = stage.DefinePrim("/Robot/base/visuals", "Xform")
        mesh_names = ["mesh_a", "mesh_b", "mesh_c", "mesh_d", "mesh_e"]
        for name in mesh_names:
            mesh = stage.DefinePrim(f"/Robot/base/visuals/{name}", "Mesh")
            UsdGeom.Mesh(mesh).CreatePointsAttr([(-1, 0, 0), (1, 0, 0), (0, 1, 0)])
            UsdGeom.Mesh(mesh).CreateFaceVertexCountsAttr([3])
            UsdGeom.Mesh(mesh).CreateFaceVertexIndicesAttr([0, 1, 2])
        stage.Save()

        # Run GeometriesRoutingRule
        rule = GeometriesRoutingRule(
            source_stage=Usd.Stage.Open(stage_path),
            package_root=self._tmpdir,
            destination_path="payloads",
            args={
                "input_stage_path": stage_path,
                "params": {
                    "scope": "/",
                    "geometries_layer": "geometries.usd",
                    "instance_layer": "instances.usda",
                    "deduplicate": True,
                    "save_base_as_usda": False,
                },
            },
        )
        result = rule.process_rule()
        self.assertIsNotNone(result, "process_rule should return a path")

        # Re-open the output stage
        output_stage = Usd.Stage.Open(result)
        visuals_prim = output_stage.GetPrimAtPath("/Robot/base/visuals")
        self.assertTrue(visuals_prim.IsValid(), "/Robot/base/visuals should exist after routing")

        # visuals itself should NOT be instanceable (it's the parent container)
        self.assertFalse(
            visuals_prim.IsInstanceable(),
            "/Robot/base/visuals should not be merged/instanceable — it is a container for multiple meshes",
        )

        # All 5 mesh children should exist as Xform references under visuals
        children = [c.GetName() for c in visuals_prim.GetChildren()]
        for name in mesh_names:
            self.assertIn(
                name,
                children,
                f"{name} missing from visuals children after geometry routing: got {children}",
            )

        # Each child should be instanceable with a reference
        for name in mesh_names:
            child = output_stage.GetPrimAtPath(f"/Robot/base/visuals/{name}")
            self.assertTrue(child.IsValid(), f"{name} should exist")
            self.assertTrue(
                child.IsInstanceable(),
                f"{name} should be instanceable",
            )

        self._success = True
