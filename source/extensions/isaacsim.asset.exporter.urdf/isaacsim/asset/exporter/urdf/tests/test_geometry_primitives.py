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
"""Tests for Capsule/Cone geometry export, breadcrumbs, and round-trip reconstruction."""

from __future__ import annotations

import math
import os
import re
import tempfile
import xml.etree.ElementTree as ET

import omni.kit.test
from isaacsim.asset.exporter.urdf.converter.geometry_reader import GeometryData, read_geometry
from isaacsim.asset.exporter.urdf.converter.mesh_exporter import MeshExporter
from pxr import Gf, Usd, UsdGeom, UsdPhysics


class TestCapsuleGeometryReader(omni.kit.test.AsyncTestCase):
    """Verify that read_geometry decomposes a UsdGeomCapsule into cylinder + 2 spheres."""

    async def setUp(self) -> None:
        """Set up test fixtures."""
        await omni.usd.get_context().new_stage_async()
        await omni.kit.app.get_app().next_update_async()

    async def tearDown(self) -> None:
        """Tear down test fixtures."""
        await omni.kit.app.get_app().next_update_async()

    def _make_capsule(self, radius: float = 0.1, height: float = 0.4, axis: str = "Z") -> Usd.Prim:
        self._stage = Usd.Stage.CreateInMemory()
        capsule = UsdGeom.Capsule.Define(self._stage, "/capsule")
        capsule.GetRadiusAttr().Set(radius)
        capsule.GetHeightAttr().Set(height)
        capsule.GetAxisAttr().Set(axis)
        return capsule.GetPrim()

    async def test_capsule_returns_three_elements(self) -> None:
        """Capsule returns three elements."""
        prim = self._make_capsule()
        result = read_geometry(prim)
        self.assertEqual(len(result), 3)

    async def test_capsule_body_is_cylinder(self) -> None:
        """Capsule body is cylinder."""
        prim = self._make_capsule(radius=0.1, height=0.4)
        body, _, _ = read_geometry(prim)
        self.assertEqual(body.geom_type, "cylinder")
        self.assertAlmostEqual(body.cylinder_radius, 0.1)
        self.assertAlmostEqual(body.cylinder_length, 0.4)
        self.assertEqual(body.name_suffix, "_body")

    async def test_capsule_caps_are_spheres(self) -> None:
        """Capsule caps are spheres."""
        prim = self._make_capsule(radius=0.1, height=0.4)
        _, top, bottom = read_geometry(prim)

        self.assertEqual(top.geom_type, "sphere")
        self.assertAlmostEqual(top.sphere_radius, 0.1)
        self.assertEqual(top.name_suffix, "_top_cap")

        self.assertEqual(bottom.geom_type, "sphere")
        self.assertAlmostEqual(bottom.sphere_radius, 0.1)
        self.assertEqual(bottom.name_suffix, "_bottom_cap")

    async def test_capsule_cap_offsets_z_axis(self) -> None:
        """Capsule cap offsets z axis."""
        prim = self._make_capsule(radius=0.1, height=0.4, axis="Z")
        _, top, bottom = read_geometry(prim)
        self.assertAlmostEqual(top.local_offset_xyz[2], 0.2)
        self.assertAlmostEqual(bottom.local_offset_xyz[2], -0.2)
        self.assertAlmostEqual(top.local_offset_xyz[0], 0.0)
        self.assertAlmostEqual(top.local_offset_xyz[1], 0.0)

    async def test_capsule_cap_offsets_x_axis(self) -> None:
        """Capsule cap offsets x axis."""
        prim = self._make_capsule(radius=0.05, height=1.0, axis="X")
        _, top, bottom = read_geometry(prim)
        self.assertAlmostEqual(top.local_offset_xyz[0], 0.5)
        self.assertAlmostEqual(bottom.local_offset_xyz[0], -0.5)
        self.assertAlmostEqual(top.local_offset_xyz[1], 0.0)
        self.assertAlmostEqual(top.local_offset_xyz[2], 0.0)

    async def test_capsule_cap_offsets_y_axis(self) -> None:
        """Capsule cap offsets y axis."""
        prim = self._make_capsule(radius=0.05, height=0.6, axis="Y")
        _, top, bottom = read_geometry(prim)
        self.assertAlmostEqual(top.local_offset_xyz[1], 0.3)
        self.assertAlmostEqual(bottom.local_offset_xyz[1], -0.3)

    async def test_capsule_breadcrumb_metadata(self) -> None:
        """Capsule breadcrumb metadata."""
        prim = self._make_capsule(radius=0.1, height=0.4, axis="Z")
        result = read_geometry(prim)
        for geom in result:
            self.assertEqual(geom.original_type, "Capsule")
            self.assertIn("radius", geom.original_params)
            self.assertIn("height", geom.original_params)
            self.assertIn("axis", geom.original_params)
            self.assertIn("source_prim_name", geom.original_params)
            self.assertAlmostEqual(geom.original_params["radius"], 0.1)
            self.assertAlmostEqual(geom.original_params["height"], 0.4)
            self.assertEqual(geom.original_params["axis"], "Z")

    async def test_capsule_all_share_source_prim(self) -> None:
        """Capsule all share source prim."""
        prim = self._make_capsule()
        result = read_geometry(prim)
        for geom in result:
            self.assertIs(geom.source_prim, prim)

    async def test_capsule_body_has_zero_offset(self) -> None:
        """Capsule body has zero offset."""
        prim = self._make_capsule()
        body = read_geometry(prim)[0]
        self.assertEqual(body.local_offset_xyz, (0.0, 0.0, 0.0))


class TestConeGeometryReader(omni.kit.test.AsyncTestCase):
    """Verify that read_geometry converts a UsdGeomCone to a mesh GeometryData."""

    async def setUp(self) -> None:
        """Set up test fixtures."""
        await omni.usd.get_context().new_stage_async()
        await omni.kit.app.get_app().next_update_async()

    async def tearDown(self) -> None:
        """Tear down test fixtures."""
        await omni.kit.app.get_app().next_update_async()

    def _make_cone(self, radius: float = 0.5, height: float = 1.0, axis: str = "Z") -> Usd.Prim:
        self._stage = Usd.Stage.CreateInMemory()
        cone = UsdGeom.Cone.Define(self._stage, "/cone")
        cone.GetRadiusAttr().Set(radius)
        cone.GetHeightAttr().Set(height)
        cone.GetAxisAttr().Set(axis)
        return cone.GetPrim()

    async def test_cone_returns_single_element(self) -> None:
        """Cone returns single element."""
        prim = self._make_cone()
        result = read_geometry(prim)
        self.assertEqual(len(result), 1)

    async def test_cone_geom_type_is_mesh(self) -> None:
        """Cone geom type is mesh."""
        prim = self._make_cone()
        geom = read_geometry(prim)[0]
        self.assertEqual(geom.geom_type, "mesh")

    async def test_cone_mesh_prim_is_none(self) -> None:
        """Cone mesh prim is none."""
        prim = self._make_cone()
        geom = read_geometry(prim)[0]
        self.assertIsNone(geom.mesh_prim)

    async def test_cone_source_prim_is_set(self) -> None:
        """Cone source prim is set."""
        prim = self._make_cone()
        geom = read_geometry(prim)[0]
        self.assertIs(geom.source_prim, prim)

    async def test_cone_breadcrumb_metadata(self) -> None:
        """Cone breadcrumb metadata."""
        prim = self._make_cone(radius=0.5, height=1.0, axis="Y")
        geom = read_geometry(prim)[0]
        self.assertEqual(geom.original_type, "Cone")
        self.assertAlmostEqual(geom.original_params["radius"], 0.5)
        self.assertAlmostEqual(geom.original_params["height"], 1.0)
        self.assertEqual(geom.original_params["axis"], "Y")
        self.assertIn("source_prim_name", geom.original_params)


class TestExistingTypesUnchanged(omni.kit.test.AsyncTestCase):
    """Verify that Cube, Sphere, Cylinder, Mesh still return single-element lists."""

    async def setUp(self) -> None:
        """Set up test fixtures."""
        await omni.usd.get_context().new_stage_async()
        await omni.kit.app.get_app().next_update_async()

    async def tearDown(self) -> None:
        """Tear down test fixtures."""
        await omni.kit.app.get_app().next_update_async()

    async def test_cube_returns_single_list(self) -> None:
        """Cube returns single list."""
        stage = Usd.Stage.CreateInMemory()
        UsdGeom.Cube.Define(stage, "/cube")
        result = read_geometry(stage.GetPrimAtPath("/cube"))
        self.assertEqual(len(result), 1)
        self.assertEqual(result[0].geom_type, "box")
        self.assertIsNone(result[0].original_type)

    async def test_sphere_returns_single_list(self) -> None:
        """Sphere returns single list."""
        stage = Usd.Stage.CreateInMemory()
        UsdGeom.Sphere.Define(stage, "/sphere")
        result = read_geometry(stage.GetPrimAtPath("/sphere"))
        self.assertEqual(len(result), 1)
        self.assertEqual(result[0].geom_type, "sphere")

    async def test_cylinder_returns_single_list(self) -> None:
        """Cylinder returns single list."""
        stage = Usd.Stage.CreateInMemory()
        UsdGeom.Cylinder.Define(stage, "/cyl")
        result = read_geometry(stage.GetPrimAtPath("/cyl"))
        self.assertEqual(len(result), 1)
        self.assertEqual(result[0].geom_type, "cylinder")

    async def test_mesh_returns_single_list(self) -> None:
        """Mesh returns single list."""
        stage = Usd.Stage.CreateInMemory()
        UsdGeom.Mesh.Define(stage, "/mesh")
        result = read_geometry(stage.GetPrimAtPath("/mesh"))
        self.assertEqual(len(result), 1)
        self.assertEqual(result[0].geom_type, "mesh")

    async def test_unsupported_type_returns_empty(self) -> None:
        """Unsupported type returns empty."""
        stage = Usd.Stage.CreateInMemory()
        UsdGeom.Xform.Define(stage, "/xform")
        result = read_geometry(stage.GetPrimAtPath("/xform"))
        self.assertEqual(len(result), 0)


class TestConeMeshExporter(omni.kit.test.AsyncTestCase):
    """Verify procedural cone OBJ tessellation."""

    async def setUp(self) -> None:
        """Set up test fixtures."""
        await omni.usd.get_context().new_stage_async()
        await omni.kit.app.get_app().next_update_async()

    async def tearDown(self) -> None:
        """Tear down test fixtures."""
        await omni.kit.app.get_app().next_update_async()

    async def test_cone_obj_has_vertices_and_faces(self) -> None:
        """Cone obj has vertices and faces."""
        with tempfile.TemporaryDirectory() as td:
            exporter = MeshExporter(td, "./")
            filename = exporter.export_cone("test_cone", 0.5, 1.0, "Z", segments=16)
            self.assertTrue(filename.endswith(".obj"))

            obj_path = os.path.join(td, os.path.basename(filename))
            with open(obj_path) as f:
                content = f.read()

            verts = re.findall(r"^v\s+", content, re.MULTILINE)
            faces = re.findall(r"^f\s+", content, re.MULTILINE)
            # 1 apex + 1 center + 16 base = 18 vertices
            self.assertEqual(len(verts), 18)
            # 16 side + 16 base = 32 faces
            self.assertEqual(len(faces), 32)

    async def test_cone_obj_axis_x(self) -> None:
        """Cone obj axis x."""
        with tempfile.TemporaryDirectory() as td:
            exporter = MeshExporter(td, "./")
            exporter.export_cone("cone_x", 1.0, 2.0, "X", segments=8)
            obj_path = os.path.join(td, "cone_x.obj")
            with open(obj_path) as f:
                content = f.read()

            lines = [l for l in content.splitlines() if l.startswith("v ")]
            apex = lines[0].split()
            self.assertAlmostEqual(float(apex[1]), 1.0, places=4)
            self.assertAlmostEqual(float(apex[2]), 0.0, places=4)
            self.assertAlmostEqual(float(apex[3]), 0.0, places=4)

    async def test_cone_obj_deduplication(self) -> None:
        """Cone obj deduplication."""
        with tempfile.TemporaryDirectory() as td:
            exporter = MeshExporter(td, "./meshes/")
            f1 = exporter.export_cone("dup_cone", 0.5, 1.0, "Z")
            f2 = exporter.export_cone("dup_cone", 0.5, 1.0, "Z")
            self.assertNotEqual(f1, f2)


class TestUrdfWriterBreadcrumbs(omni.kit.test.AsyncTestCase):
    """Verify that _write_source_geometry_breadcrumb produces correct XML comments."""

    async def setUp(self) -> None:
        """Set up test fixtures."""
        await omni.usd.get_context().new_stage_async()
        await omni.kit.app.get_app().next_update_async()

    async def tearDown(self) -> None:
        """Tear down test fixtures."""
        await omni.kit.app.get_app().next_update_async()

    def _make_urdf_xml(self, geom: GeometryData, element_type: str = "visual") -> str:
        from isaacsim.asset.exporter.urdf.converter.link_reader import CollisionData, VisualData
        from isaacsim.asset.exporter.urdf.converter.urdf_writer import _write_collision, _write_visual

        root = ET.Element("link", name="test_link")
        if element_type == "visual":
            visual = VisualData(name="test_geom", geometry=geom)
            _write_visual(root, visual)
        else:
            collision = CollisionData(name="test_geom", geometry=geom)
            _write_collision(root, collision)
        return ET.tostring(root, encoding="unicode")

    async def test_capsule_body_breadcrumb(self) -> None:
        """Capsule body breadcrumb."""
        geom = GeometryData(
            geom_type="cylinder",
            cylinder_radius=0.1,
            cylinder_length=0.4,
            name_suffix="_body",
            original_type="Capsule",
            original_params={"radius": 0.1, "height": 0.4, "axis": "Z", "source_prim_name": "cap0"},
        )
        xml_str = self._make_urdf_xml(geom)
        self.assertIn("isaac:source_geometry", xml_str)
        self.assertIn('"type": "Capsule"', xml_str)
        self.assertIn('"role": "body"', xml_str)

    async def test_cone_breadcrumb(self) -> None:
        """Cone breadcrumb."""
        geom = GeometryData(
            geom_type="mesh",
            mesh_filename="./meshes/cone.obj",
            original_type="Cone",
            original_params={"radius": 0.5, "height": 1.0, "axis": "Z", "source_prim_name": "cone0"},
        )
        xml_str = self._make_urdf_xml(geom)
        self.assertIn("isaac:source_geometry", xml_str)
        self.assertIn('"type": "Cone"', xml_str)

    async def test_no_breadcrumb_for_native_types(self) -> None:
        """No breadcrumb for native types."""
        geom = GeometryData(geom_type="box", box_size=(1.0, 1.0, 1.0))
        xml_str = self._make_urdf_xml(geom)
        self.assertNotIn("isaac:source_geometry", xml_str)

    async def test_collision_breadcrumb(self) -> None:
        """Collision breadcrumb."""
        geom = GeometryData(
            geom_type="sphere",
            sphere_radius=0.1,
            name_suffix="_top_cap",
            original_type="Capsule",
            original_params={"radius": 0.1, "height": 0.4, "axis": "Z", "source_prim_name": "cap0"},
        )
        xml_str = self._make_urdf_xml(geom, element_type="collision")
        self.assertIn("isaac:source_geometry", xml_str)
        self.assertIn('"role": "top_cap"', xml_str)


class TestBreadcrumbParsing(omni.kit.test.AsyncTestCase):
    """Verify that parse_source_geometry_breadcrumbs extracts metadata from URDF XML."""

    async def setUp(self) -> None:
        """Set up test fixtures."""
        await omni.usd.get_context().new_stage_async()
        await omni.kit.app.get_app().next_update_async()

    async def tearDown(self) -> None:
        """Tear down test fixtures."""
        await omni.kit.app.get_app().next_update_async()

    def _write_urdf_with_breadcrumbs(self, path: str) -> None:
        content = """\
<?xml version="1.0" encoding="utf-8"?>
<robot name="test">
  <link name="base">
    <visual name="cap_body">
      <geometry><cylinder radius="0.1" length="0.4"/></geometry>
      <!-- isaac:source_geometry {"axis": "Z", "height": 0.4, "radius": 0.1, "role": "body", "source_prim_name": "capsule0", "type": "Capsule"} -->
    </visual>
    <visual name="cap_top_cap">
      <geometry><sphere radius="0.1"/></geometry>
      <!-- isaac:source_geometry {"axis": "Z", "height": 0.4, "radius": 0.1, "role": "top_cap", "source_prim_name": "capsule0", "type": "Capsule"} -->
    </visual>
    <visual name="cap_bottom_cap">
      <geometry><sphere radius="0.1"/></geometry>
      <!-- isaac:source_geometry {"axis": "Z", "height": 0.4, "radius": 0.1, "role": "bottom_cap", "source_prim_name": "capsule0", "type": "Capsule"} -->
    </visual>
    <visual name="cone_vis">
      <geometry><mesh filename="./meshes/cone.obj"/></geometry>
      <!-- isaac:source_geometry {"axis": "Z", "height": 1.0, "radius": 0.5, "source_prim_name": "cone0", "type": "Cone"} -->
    </visual>
  </link>
</robot>
"""
        with open(path, "w") as f:
            f.write(content)

    async def test_parse_capsule_breadcrumbs(self) -> None:
        """Parse capsule breadcrumbs."""
        from isaacsim.asset.importer.urdf.impl.geometry_reconstruction import (
            parse_source_geometry_breadcrumbs,
        )

        with tempfile.TemporaryDirectory() as td:
            urdf_path = os.path.join(td, "test.urdf")
            self._write_urdf_with_breadcrumbs(urdf_path)
            results = parse_source_geometry_breadcrumbs(urdf_path)

        capsule_entries = [r for r in results if r.original_type == "Capsule"]
        self.assertEqual(len(capsule_entries), 3)
        roles = {e.role for e in capsule_entries}
        self.assertEqual(roles, {"body", "top_cap", "bottom_cap"})
        for entry in capsule_entries:
            self.assertEqual(entry.link_name, "base")
            self.assertAlmostEqual(entry.params["radius"], 0.1)

    async def test_parse_cone_breadcrumbs(self) -> None:
        """Parse cone breadcrumbs."""
        from isaacsim.asset.importer.urdf.impl.geometry_reconstruction import (
            parse_source_geometry_breadcrumbs,
        )

        with tempfile.TemporaryDirectory() as td:
            urdf_path = os.path.join(td, "test.urdf")
            self._write_urdf_with_breadcrumbs(urdf_path)
            results = parse_source_geometry_breadcrumbs(urdf_path)

        cone_entries = [r for r in results if r.original_type == "Cone"]
        self.assertEqual(len(cone_entries), 1)
        self.assertEqual(cone_entries[0].link_name, "base")
        self.assertAlmostEqual(cone_entries[0].params["radius"], 0.5)

    async def test_parse_no_breadcrumbs(self) -> None:
        """Parse no breadcrumbs."""
        from isaacsim.asset.importer.urdf.impl.geometry_reconstruction import (
            parse_source_geometry_breadcrumbs,
        )

        with tempfile.TemporaryDirectory() as td:
            urdf_path = os.path.join(td, "plain.urdf")
            with open(urdf_path, "w") as f:
                f.write('<?xml version="1.0"?><robot name="r"><link name="a"/></robot>')
            results = parse_source_geometry_breadcrumbs(urdf_path)
        self.assertEqual(len(results), 0)


class TestReconstructSourceGeometry(omni.kit.test.AsyncTestCase):
    """Verify that reconstruct_source_geometry replaces converted geometry with originals."""

    async def setUp(self) -> None:
        """Set up test fixtures."""
        await omni.usd.get_context().new_stage_async()
        await omni.kit.app.get_app().next_update_async()

    async def tearDown(self) -> None:
        """Tear down test fixtures."""
        await omni.kit.app.get_app().next_update_async()

    def _build_capsule_stage(self) -> Usd.Stage:
        stage = Usd.Stage.CreateInMemory()
        root = UsdGeom.Xform.Define(stage, "/robot")
        stage.SetDefaultPrim(root.GetPrim())

        UsdGeom.Xform.Define(stage, "/robot/Geometry")
        UsdGeom.Xform.Define(stage, "/robot/Geometry/base")

        UsdGeom.Xform.Define(stage, "/robot/Geometry/base/cap_body")
        cyl = UsdGeom.Cylinder.Define(stage, "/robot/Geometry/base/cap_body/cylinder")
        cyl.GetRadiusAttr().Set(0.1)
        cyl.GetHeightAttr().Set(0.4)

        UsdGeom.Xform.Define(stage, "/robot/Geometry/base/cap_top_cap")
        sph_top = UsdGeom.Sphere.Define(stage, "/robot/Geometry/base/cap_top_cap/sphere")
        sph_top.GetRadiusAttr().Set(0.1)

        UsdGeom.Xform.Define(stage, "/robot/Geometry/base/cap_bottom_cap")
        sph_bot = UsdGeom.Sphere.Define(stage, "/robot/Geometry/base/cap_bottom_cap/sphere")
        sph_bot.GetRadiusAttr().Set(0.1)

        return stage

    def _build_cone_stage(self) -> Usd.Stage:
        stage = Usd.Stage.CreateInMemory()
        root = UsdGeom.Xform.Define(stage, "/robot")
        stage.SetDefaultPrim(root.GetPrim())

        UsdGeom.Xform.Define(stage, "/robot/Geometry")
        UsdGeom.Xform.Define(stage, "/robot/Geometry/base")

        UsdGeom.Xform.Define(stage, "/robot/Geometry/base/cone_vis")
        UsdGeom.Mesh.Define(stage, "/robot/Geometry/base/cone_vis/mesh")

        return stage

    async def test_capsule_reconstruction(self) -> None:
        """Capsule reconstruction."""
        from isaacsim.asset.importer.urdf.impl.geometry_reconstruction import (
            SourceGeometryInfo,
            reconstruct_source_geometry,
        )

        stage = self._build_capsule_stage()

        breadcrumbs = [
            SourceGeometryInfo(
                link_name="base",
                element_type="visual",
                element_name="cap_body",
                original_type="Capsule",
                role="body",
                params={"type": "Capsule", "radius": 0.1, "height": 0.4, "axis": "Z", "source_prim_name": "capsule0"},
            ),
            SourceGeometryInfo(
                link_name="base",
                element_type="visual",
                element_name="cap_top_cap",
                original_type="Capsule",
                role="top_cap",
                params={"type": "Capsule", "radius": 0.1, "height": 0.4, "axis": "Z", "source_prim_name": "capsule0"},
            ),
            SourceGeometryInfo(
                link_name="base",
                element_type="visual",
                element_name="cap_bottom_cap",
                original_type="Capsule",
                role="bottom_cap",
                params={"type": "Capsule", "radius": 0.1, "height": 0.4, "axis": "Z", "source_prim_name": "capsule0"},
            ),
        ]

        count = reconstruct_source_geometry(stage, breadcrumbs)
        self.assertEqual(count, 1)

        self.assertFalse(stage.GetPrimAtPath("/robot/Geometry/base/cap_body").IsValid())
        self.assertFalse(stage.GetPrimAtPath("/robot/Geometry/base/cap_top_cap").IsValid())
        self.assertFalse(stage.GetPrimAtPath("/robot/Geometry/base/cap_bottom_cap").IsValid())

        capsule_prim = stage.GetPrimAtPath("/robot/Geometry/base/capsule0")
        self.assertTrue(capsule_prim.IsValid())
        self.assertEqual(capsule_prim.GetTypeName(), "Capsule")

        capsule = UsdGeom.Capsule(capsule_prim)
        self.assertAlmostEqual(capsule.GetRadiusAttr().Get(), 0.1)
        self.assertAlmostEqual(capsule.GetHeightAttr().Get(), 0.4)
        self.assertEqual(capsule.GetAxisAttr().Get(), "Z")

    async def test_cone_reconstruction(self) -> None:
        """Cone reconstruction."""
        from isaacsim.asset.importer.urdf.impl.geometry_reconstruction import (
            SourceGeometryInfo,
            reconstruct_source_geometry,
        )

        stage = self._build_cone_stage()

        breadcrumbs = [
            SourceGeometryInfo(
                link_name="base",
                element_type="visual",
                element_name="cone_vis",
                original_type="Cone",
                role="",
                params={"type": "Cone", "radius": 0.5, "height": 1.0, "axis": "Z", "source_prim_name": "cone0"},
            ),
        ]

        count = reconstruct_source_geometry(stage, breadcrumbs)
        self.assertEqual(count, 1)

        self.assertFalse(stage.GetPrimAtPath("/robot/Geometry/base/cone_vis").IsValid())

        cone_prim = stage.GetPrimAtPath("/robot/Geometry/base/cone0")
        self.assertTrue(cone_prim.IsValid())
        self.assertEqual(cone_prim.GetTypeName(), "Cone")

        cone = UsdGeom.Cone(cone_prim)
        self.assertAlmostEqual(cone.GetRadiusAttr().Get(), 0.5)
        self.assertAlmostEqual(cone.GetHeightAttr().Get(), 1.0)

    async def test_no_breadcrumbs_returns_zero(self) -> None:
        """No breadcrumbs returns zero."""
        from isaacsim.asset.importer.urdf.impl.geometry_reconstruction import (
            reconstruct_source_geometry,
        )

        stage = Usd.Stage.CreateInMemory()
        count = reconstruct_source_geometry(stage, [])
        self.assertEqual(count, 0)

    async def test_capsule_collision_api_preserved(self) -> None:
        """Capsule collision api preserved."""
        from isaacsim.asset.importer.urdf.impl.geometry_reconstruction import (
            SourceGeometryInfo,
            reconstruct_source_geometry,
        )

        stage = self._build_capsule_stage()
        cyl_prim = stage.GetPrimAtPath("/robot/Geometry/base/cap_body/cylinder")
        UsdPhysics.CollisionAPI.Apply(cyl_prim)

        breadcrumbs = [
            SourceGeometryInfo(
                link_name="base",
                element_type="collision",
                element_name="cap_body",
                original_type="Capsule",
                role="body",
                params={"type": "Capsule", "radius": 0.1, "height": 0.4, "axis": "Z", "source_prim_name": "cap0"},
            ),
            SourceGeometryInfo(
                link_name="base",
                element_type="collision",
                element_name="cap_top_cap",
                original_type="Capsule",
                role="top_cap",
                params={"type": "Capsule", "radius": 0.1, "height": 0.4, "axis": "Z", "source_prim_name": "cap0"},
            ),
            SourceGeometryInfo(
                link_name="base",
                element_type="collision",
                element_name="cap_bottom_cap",
                original_type="Capsule",
                role="bottom_cap",
                params={"type": "Capsule", "radius": 0.1, "height": 0.4, "axis": "Z", "source_prim_name": "cap0"},
            ),
        ]

        reconstruct_source_geometry(stage, breadcrumbs)
        capsule_prim = stage.GetPrimAtPath("/robot/Geometry/base/cap0")
        self.assertTrue(capsule_prim.HasAPI(UsdPhysics.CollisionAPI))


class TestComposeLocalOffset(omni.kit.test.AsyncTestCase):
    """Verify _compose_local_offset rotation math."""

    async def setUp(self) -> None:
        """Set up test fixtures."""
        await omni.usd.get_context().new_stage_async()
        await omni.kit.app.get_app().next_update_async()

    async def tearDown(self) -> None:
        """Tear down test fixtures."""
        await omni.kit.app.get_app().next_update_async()

    async def test_identity_rotation(self) -> None:
        """Identity rotation."""
        from isaacsim.asset.exporter.urdf.converter.usd_to_urdf import _compose_local_offset

        result = _compose_local_offset((1.0, 2.0, 3.0), (0.0, 0.0, 0.0), (0.0, 0.0, 0.5))
        self.assertAlmostEqual(result[0], 1.0)
        self.assertAlmostEqual(result[1], 2.0)
        self.assertAlmostEqual(result[2], 3.5)

    async def test_90deg_yaw(self) -> None:
        """90deg yaw."""
        from isaacsim.asset.exporter.urdf.converter.usd_to_urdf import _compose_local_offset

        result = _compose_local_offset((0.0, 0.0, 0.0), (0.0, 0.0, math.pi / 2), (1.0, 0.0, 0.0))
        self.assertAlmostEqual(result[0], 0.0, places=6)
        self.assertAlmostEqual(result[1], 1.0, places=6)
        self.assertAlmostEqual(result[2], 0.0, places=6)

    async def test_zero_offset(self) -> None:
        """Zero offset."""
        from isaacsim.asset.exporter.urdf.converter.usd_to_urdf import _compose_local_offset

        result = _compose_local_offset((5.0, 6.0, 7.0), (0.1, 0.2, 0.3), (0.0, 0.0, 0.0))
        self.assertAlmostEqual(result[0], 5.0)
        self.assertAlmostEqual(result[1], 6.0)
        self.assertAlmostEqual(result[2], 7.0)


class TestEndToEndBreadcrumbRoundTrip(omni.kit.test.AsyncTestCase):
    """Write URDF with breadcrumbs via the exporter writer, parse them back."""

    async def setUp(self) -> None:
        """Set up test fixtures."""
        await omni.usd.get_context().new_stage_async()
        await omni.kit.app.get_app().next_update_async()

    async def tearDown(self) -> None:
        """Tear down test fixtures."""
        await omni.kit.app.get_app().next_update_async()

    async def test_write_then_parse_capsule(self) -> None:
        """Write then parse capsule."""
        from isaacsim.asset.exporter.urdf.converter.link_reader import LinkData, VisualData
        from isaacsim.asset.exporter.urdf.converter.urdf_writer import write_urdf
        from isaacsim.asset.importer.urdf.impl.geometry_reconstruction import (
            parse_source_geometry_breadcrumbs,
        )

        body = GeometryData(
            geom_type="cylinder",
            cylinder_radius=0.1,
            cylinder_length=0.4,
            name_suffix="_body",
            original_type="Capsule",
            original_params={"radius": 0.1, "height": 0.4, "axis": "Z", "source_prim_name": "cap"},
        )
        top = GeometryData(
            geom_type="sphere",
            sphere_radius=0.1,
            name_suffix="_top_cap",
            original_type="Capsule",
            original_params={"radius": 0.1, "height": 0.4, "axis": "Z", "source_prim_name": "cap"},
        )
        bot = GeometryData(
            geom_type="sphere",
            sphere_radius=0.1,
            name_suffix="_bottom_cap",
            original_type="Capsule",
            original_params={"radius": 0.1, "height": 0.4, "axis": "Z", "source_prim_name": "cap"},
        )

        link = LinkData(
            name="base",
            visuals=[
                VisualData(name="cap_body", geometry=body),
                VisualData(name="cap_top_cap", geometry=top),
                VisualData(name="cap_bottom_cap", geometry=bot),
            ],
        )

        with tempfile.TemporaryDirectory() as td:
            urdf_path = os.path.join(td, "test.urdf")
            write_urdf("test_robot", [link], [], [], urdf_path)
            self.assertTrue(os.path.exists(urdf_path))

            results = parse_source_geometry_breadcrumbs(urdf_path)

        capsules = [r for r in results if r.original_type == "Capsule"]
        self.assertEqual(len(capsules), 3)
        roles = {r.role for r in capsules}
        self.assertEqual(roles, {"body", "top_cap", "bottom_cap"})

    async def test_write_then_parse_cone(self) -> None:
        """Write then parse cone."""
        from isaacsim.asset.exporter.urdf.converter.link_reader import LinkData, VisualData
        from isaacsim.asset.exporter.urdf.converter.urdf_writer import write_urdf
        from isaacsim.asset.importer.urdf.impl.geometry_reconstruction import (
            parse_source_geometry_breadcrumbs,
        )

        cone_geom = GeometryData(
            geom_type="mesh",
            mesh_filename="./meshes/cone.obj",
            original_type="Cone",
            original_params={"radius": 0.5, "height": 1.0, "axis": "Z", "source_prim_name": "cone0"},
        )
        link = LinkData(
            name="base",
            visuals=[
                VisualData(name="cone0", geometry=cone_geom),
            ],
        )

        with tempfile.TemporaryDirectory() as td:
            urdf_path = os.path.join(td, "test.urdf")
            write_urdf("test_robot", [link], [], [], urdf_path)

            results = parse_source_geometry_breadcrumbs(urdf_path)

        cones = [r for r in results if r.original_type == "Cone"]
        self.assertEqual(len(cones), 1)
        self.assertAlmostEqual(cones[0].params["radius"], 0.5)
        self.assertAlmostEqual(cones[0].params["height"], 1.0)


# ---------------------------------------------------------------------------
# Joint breadcrumb round-trip tests
# ---------------------------------------------------------------------------


class TestJointBreadcrumbRoundTrip(omni.kit.test.AsyncTestCase):
    """Write URDF with joint breadcrumbs via the exporter writer, parse them back."""

    async def setUp(self) -> None:
        """Set up test fixtures."""
        await omni.usd.get_context().new_stage_async()
        await omni.kit.app.get_app().next_update_async()

    async def tearDown(self) -> None:
        """Tear down test fixtures."""
        await omni.kit.app.get_app().next_update_async()

    async def test_write_then_parse_spherical_chain(self) -> None:
        """Write then parse spherical chain."""
        from isaacsim.asset.exporter.urdf.converter.joint_reader import JointData
        from isaacsim.asset.exporter.urdf.converter.link_reader import LinkData
        from isaacsim.asset.exporter.urdf.converter.urdf_writer import write_urdf
        from isaacsim.asset.importer.urdf.impl.joint_reconstruction import (
            parse_source_joint_breadcrumbs,
        )

        parent = LinkData(name="torso")
        child = LinkData(name="upper_arm")
        g1 = LinkData(name="hip_ghost_1")
        g2 = LinkData(name="hip_ghost_2")

        j1 = JointData(
            name="hip_rotX",
            joint_type="revolute",
            parent_link="torso",
            child_link="hip_ghost_1",
            axis=(1, 0, 0),
            limit_lower=-1.5708,
            limit_upper=1.5708,
            original_usd_type="PhysicsSphericalJoint",
            original_params={
                "original_name": "hip",
                "chain_joints": ["hip_rotX", "hip_rotY", "hip_rotZ"],
                "ghost_links": ["hip_ghost_1", "hip_ghost_2"],
                "per_axis_limits": {
                    "rotX": {"low": -90, "high": 90},
                    "rotY": {"low": -90, "high": 90},
                    "rotZ": {"low": -180, "high": 180},
                },
            },
        )
        j2 = JointData(
            name="hip_rotY",
            joint_type="revolute",
            parent_link="hip_ghost_1",
            child_link="hip_ghost_2",
            axis=(0, 1, 0),
            limit_lower=-1.5708,
            limit_upper=1.5708,
        )
        j3 = JointData(
            name="hip_rotZ",
            joint_type="revolute",
            parent_link="hip_ghost_2",
            child_link="upper_arm",
            axis=(0, 0, 1),
            limit_lower=-3.14159,
            limit_upper=3.14159,
        )

        with tempfile.TemporaryDirectory() as td:
            urdf_path = os.path.join(td, "test.urdf")
            write_urdf("test_robot", [parent, child, g1, g2], [j1, j2, j3], [], urdf_path)
            self.assertTrue(os.path.exists(urdf_path))

            results = parse_source_joint_breadcrumbs(urdf_path)

        self.assertEqual(len(results), 1)
        bc = results[0]
        self.assertEqual(bc.original_type, "PhysicsSphericalJoint")
        self.assertEqual(bc.original_name, "hip")
        self.assertEqual(len(bc.chain_joints), 3)
        self.assertEqual(len(bc.ghost_links), 2)
        self.assertIn("rotX", bc.per_axis_limits)

    async def test_write_then_parse_d6_chain(self) -> None:
        """Write then parse d6 chain."""
        from isaacsim.asset.exporter.urdf.converter.joint_reader import JointData
        from isaacsim.asset.exporter.urdf.converter.link_reader import LinkData
        from isaacsim.asset.exporter.urdf.converter.urdf_writer import write_urdf
        from isaacsim.asset.importer.urdf.impl.joint_reconstruction import (
            parse_source_joint_breadcrumbs,
        )

        parent = LinkData(name="base")
        child = LinkData(name="platform")
        ghost = LinkData(name="slider_ghost_1")

        j1 = JointData(
            name="slider_rotX",
            joint_type="revolute",
            parent_link="base",
            child_link="slider_ghost_1",
            axis=(1, 0, 0),
            limit_lower=-0.7854,
            limit_upper=0.7854,
            original_usd_type="PhysicsJoint",
            original_params={
                "original_name": "slider",
                "chain_joints": ["slider_rotX", "slider_transZ"],
                "ghost_links": ["slider_ghost_1"],
                "per_axis_limits": {
                    "rotX": {"low": -45, "high": 45},
                    "transZ": {"low": 0, "high": 0.5},
                },
            },
        )
        j2 = JointData(
            name="slider_transZ",
            joint_type="prismatic",
            parent_link="slider_ghost_1",
            child_link="platform",
            axis=(0, 0, 1),
            limit_lower=0,
            limit_upper=0.5,
        )

        with tempfile.TemporaryDirectory() as td:
            urdf_path = os.path.join(td, "test.urdf")
            write_urdf("test_robot", [parent, child, ghost], [j1, j2], [], urdf_path)

            results = parse_source_joint_breadcrumbs(urdf_path)

        self.assertEqual(len(results), 1)
        self.assertEqual(results[0].original_type, "PhysicsJoint")
        self.assertEqual(len(results[0].chain_joints), 2)
        self.assertIn("transZ", results[0].per_axis_limits)
        self.assertAlmostEqual(results[0].per_axis_limits["transZ"]["high"], 0.5)

    async def test_no_breadcrumb_for_standard_joints(self) -> None:
        """No breadcrumb for standard joints."""
        from isaacsim.asset.exporter.urdf.converter.joint_reader import JointData
        from isaacsim.asset.exporter.urdf.converter.link_reader import LinkData
        from isaacsim.asset.exporter.urdf.converter.urdf_writer import write_urdf
        from isaacsim.asset.importer.urdf.impl.joint_reconstruction import (
            parse_source_joint_breadcrumbs,
        )

        parent = LinkData(name="base")
        child = LinkData(name="link1")
        j = JointData(
            name="joint1",
            joint_type="revolute",
            parent_link="base",
            child_link="link1",
            axis=(0, 0, 1),
            limit_lower=-1.57,
            limit_upper=1.57,
        )

        with tempfile.TemporaryDirectory() as td:
            urdf_path = os.path.join(td, "test.urdf")
            write_urdf("test_robot", [parent, child], [j], [], urdf_path)

            results = parse_source_joint_breadcrumbs(urdf_path)

        self.assertEqual(len(results), 0)


class TestMultiDofJointExpansion(omni.kit.test.AsyncTestCase):
    """Verify chain expansion for SphericalJoint and D6Joint."""

    async def setUp(self) -> None:
        """Set up test fixtures."""
        await omni.usd.get_context().new_stage_async()
        await omni.kit.app.get_app().next_update_async()

    async def tearDown(self) -> None:
        """Tear down test fixtures."""
        await omni.kit.app.get_app().next_update_async()

    def _build_two_body_stage(self) -> Usd.Stage:
        stage = Usd.Stage.CreateInMemory()
        robot = UsdGeom.Xform.Define(stage, "/Robot")
        stage.SetDefaultPrim(robot.GetPrim())

        body_a = UsdGeom.Xform.Define(stage, "/Robot/body_a")
        UsdPhysics.RigidBodyAPI.Apply(body_a.GetPrim())

        body_b = UsdGeom.Xform.Define(stage, "/Robot/body_b")
        UsdPhysics.RigidBodyAPI.Apply(body_b.GetPrim())

        return stage

    async def test_spherical_expansion(self) -> None:
        """Spherical expansion."""
        from isaacsim.asset.exporter.urdf.converter.joint_reader import (
            JointData,
            _expand_multi_dof_joint,
        )

        stage = self._build_two_body_stage()
        joint = UsdPhysics.SphericalJoint.Define(stage, "/Robot/spherical_joint")
        joint.CreateBody0Rel().SetTargets(["/Robot/body_a"])
        joint.CreateBody1Rel().SetTargets(["/Robot/body_b"])
        joint.CreateLocalPos0Attr().Set(Gf.Vec3f(0, 0, 0.5))

        base_jd = JointData(
            name="spherical_joint",
            joint_type="spherical",
            parent_link="body_a",
            child_link="body_b",
            origin_xyz=(0, 0, 0.5),
            origin_rpy=(0, 0, 0),
        )

        chain_joints, ghost_links = _expand_multi_dof_joint(joint.GetPrim(), base_jd)

        self.assertEqual(len(chain_joints), 3)
        self.assertEqual(len(ghost_links), 2)

        self.assertEqual(chain_joints[0].parent_link, "body_a")
        self.assertEqual(chain_joints[-1].child_link, "body_b")

        for cj in chain_joints:
            self.assertEqual(cj.joint_type, "revolute")

        self.assertEqual(chain_joints[0].name, "spherical_joint_rotX")
        self.assertEqual(chain_joints[1].name, "spherical_joint_rotY")
        self.assertEqual(chain_joints[2].name, "spherical_joint_rotZ")

        self.assertIsNotNone(chain_joints[0].original_usd_type)
        self.assertEqual(chain_joints[0].original_usd_type, "PhysicsSphericalJoint")
        self.assertIsNone(chain_joints[1].original_usd_type)

        self.assertEqual(ghost_links[0].name, "spherical_joint_ghost_1")
        self.assertEqual(ghost_links[1].name, "spherical_joint_ghost_2")

        self.assertAlmostEqual(chain_joints[0].origin_xyz[2], 0.5)
        self.assertAlmostEqual(chain_joints[1].origin_xyz[0], 0.0)

    async def test_d6_single_axis_no_chain(self) -> None:
        """D6 single axis no chain."""
        from isaacsim.asset.exporter.urdf.converter.joint_reader import (
            JointData,
            _expand_multi_dof_joint,
        )

        stage = self._build_two_body_stage()
        joint = UsdPhysics.Joint.Define(stage, "/Robot/d6_joint")
        joint.CreateBody0Rel().SetTargets(["/Robot/body_a"])
        joint.CreateBody1Rel().SetTargets(["/Robot/body_b"])

        lim = UsdPhysics.LimitAPI.Apply(joint.GetPrim(), "rotZ")
        lim.CreateLowAttr().Set(-90.0)
        lim.CreateHighAttr().Set(90.0)

        base_jd = JointData(
            name="d6_joint",
            joint_type="d6",
            parent_link="body_a",
            child_link="body_b",
        )

        chain_joints, ghost_links = _expand_multi_dof_joint(joint.GetPrim(), base_jd)

        self.assertEqual(len(chain_joints), 1)
        self.assertEqual(len(ghost_links), 0)
        self.assertEqual(chain_joints[0].joint_type, "revolute")
        self.assertEqual(chain_joints[0].parent_link, "body_a")
        self.assertEqual(chain_joints[0].child_link, "body_b")
        self.assertIsNotNone(chain_joints[0].original_usd_type)

    async def test_d6_multi_axis_expansion(self) -> None:
        """D6 multi axis expansion."""
        from isaacsim.asset.exporter.urdf.converter.joint_reader import (
            JointData,
            _expand_multi_dof_joint,
        )

        stage = self._build_two_body_stage()
        joint = UsdPhysics.Joint.Define(stage, "/Robot/d6_multi")
        joint.CreateBody0Rel().SetTargets(["/Robot/body_a"])
        joint.CreateBody1Rel().SetTargets(["/Robot/body_b"])

        lim_rx = UsdPhysics.LimitAPI.Apply(joint.GetPrim(), "rotX")
        lim_rx.CreateLowAttr().Set(-45.0)
        lim_rx.CreateHighAttr().Set(45.0)

        lim_ry = UsdPhysics.LimitAPI.Apply(joint.GetPrim(), "rotY")
        lim_ry.CreateLowAttr().Set(-30.0)
        lim_ry.CreateHighAttr().Set(30.0)

        lim_tz = UsdPhysics.LimitAPI.Apply(joint.GetPrim(), "transZ")
        lim_tz.CreateLowAttr().Set(0.0)
        lim_tz.CreateHighAttr().Set(0.5)

        base_jd = JointData(
            name="d6_multi",
            joint_type="d6",
            parent_link="body_a",
            child_link="body_b",
            origin_xyz=(1, 0, 0),
            origin_rpy=(0, 0, 0),
        )

        chain_joints, ghost_links = _expand_multi_dof_joint(joint.GetPrim(), base_jd)

        self.assertEqual(len(chain_joints), 3)
        self.assertEqual(len(ghost_links), 2)

        self.assertEqual(chain_joints[0].joint_type, "revolute")
        self.assertEqual(chain_joints[1].joint_type, "revolute")
        self.assertEqual(chain_joints[2].joint_type, "prismatic")

        self.assertAlmostEqual(chain_joints[0].limit_lower, math.radians(-45))
        self.assertAlmostEqual(chain_joints[0].limit_upper, math.radians(45))
        self.assertAlmostEqual(chain_joints[2].limit_lower, 0.0)
        self.assertAlmostEqual(chain_joints[2].limit_upper, 0.5)

        self.assertEqual(chain_joints[0].axis, (1.0, 0.0, 0.0))
        self.assertEqual(chain_joints[1].axis, (0.0, 1.0, 0.0))
        self.assertEqual(chain_joints[2].axis, (0.0, 0.0, 1.0))

    async def test_d6_all_locked_becomes_empty(self) -> None:
        """D6 all locked becomes empty."""
        from isaacsim.asset.exporter.urdf.converter.joint_reader import (
            JointData,
            _expand_multi_dof_joint,
        )

        stage = self._build_two_body_stage()
        joint = UsdPhysics.Joint.Define(stage, "/Robot/d6_locked")
        joint.CreateBody0Rel().SetTargets(["/Robot/body_a"])
        joint.CreateBody1Rel().SetTargets(["/Robot/body_b"])

        lim = UsdPhysics.LimitAPI.Apply(joint.GetPrim(), "rotZ")
        lim.CreateLowAttr().Set(10.0)
        lim.CreateHighAttr().Set(-10.0)

        base_jd = JointData(
            name="d6_locked",
            joint_type="d6",
            parent_link="body_a",
            child_link="body_b",
        )

        chain_joints, ghost_links = _expand_multi_dof_joint(joint.GetPrim(), base_jd)

        self.assertEqual(len(chain_joints), 0)
        self.assertEqual(len(ghost_links), 0)


class TestJointReconstruction(omni.kit.test.AsyncTestCase):
    """Verify that chain joints + ghost links collapse back into original USD joint types."""

    async def setUp(self) -> None:
        """Set up test fixtures."""
        await omni.usd.get_context().new_stage_async()
        await omni.kit.app.get_app().next_update_async()

    async def tearDown(self) -> None:
        """Tear down test fixtures."""
        await omni.kit.app.get_app().next_update_async()

    def _build_chain_stage(self) -> Usd.Stage:
        stage = Usd.Stage.CreateInMemory()
        robot = UsdGeom.Xform.Define(stage, "/Robot")
        stage.SetDefaultPrim(robot.GetPrim())

        UsdGeom.Scope.Define(stage, "/Robot/Geometry")
        body_a = UsdGeom.Xform.Define(stage, "/Robot/Geometry/body_a")
        UsdPhysics.RigidBodyAPI.Apply(body_a.GetPrim())
        body_b = UsdGeom.Xform.Define(stage, "/Robot/Geometry/body_b")
        UsdPhysics.RigidBodyAPI.Apply(body_b.GetPrim())

        g1 = UsdGeom.Xform.Define(stage, "/Robot/Geometry/hip_ghost_1")
        UsdPhysics.RigidBodyAPI.Apply(g1.GetPrim())
        g2 = UsdGeom.Xform.Define(stage, "/Robot/Geometry/hip_ghost_2")
        UsdPhysics.RigidBodyAPI.Apply(g2.GetPrim())

        UsdGeom.Scope.Define(stage, "/Robot/Physics")

        j1 = UsdPhysics.RevoluteJoint.Define(stage, "/Robot/Physics/hip_rotX")
        j1.CreateBody0Rel().SetTargets(["/Robot/Geometry/body_a"])
        j1.CreateBody1Rel().SetTargets(["/Robot/Geometry/hip_ghost_1"])
        j1.CreateAxisAttr("X")

        j2 = UsdPhysics.RevoluteJoint.Define(stage, "/Robot/Physics/hip_rotY")
        j2.CreateBody0Rel().SetTargets(["/Robot/Geometry/hip_ghost_1"])
        j2.CreateBody1Rel().SetTargets(["/Robot/Geometry/hip_ghost_2"])
        j2.CreateAxisAttr("Y")

        j3 = UsdPhysics.RevoluteJoint.Define(stage, "/Robot/Physics/hip_rotZ")
        j3.CreateBody0Rel().SetTargets(["/Robot/Geometry/hip_ghost_2"])
        j3.CreateBody1Rel().SetTargets(["/Robot/Geometry/body_b"])
        j3.CreateAxisAttr("Z")

        return stage

    async def test_reconstruct_spherical_from_chain(self) -> None:
        """Reconstruct spherical from chain."""
        from isaacsim.asset.importer.urdf.impl.joint_reconstruction import (
            SourceJointInfo,
            reconstruct_source_joints,
        )

        stage = self._build_chain_stage()

        bc = SourceJointInfo(
            joint_name="hip_rotX",
            original_type="PhysicsSphericalJoint",
            original_name="hip",
            chain_joints=["hip_rotX", "hip_rotY", "hip_rotZ"],
            ghost_links=["hip_ghost_1", "hip_ghost_2"],
            per_axis_limits={
                "rotX": {"low": -90, "high": 90},
                "rotY": {"low": -45, "high": 45},
                "rotZ": {"low": -180, "high": 180},
            },
            local_poses={
                "local_pos0": [0, 0, 0.5],
                "local_rot0": [1, 0, 0, 0],
            },
        )

        count = reconstruct_source_joints(stage, [bc])
        self.assertEqual(count, 1)

        self.assertFalse(stage.GetPrimAtPath("/Robot/Physics/hip_rotX").IsValid())
        self.assertFalse(stage.GetPrimAtPath("/Robot/Physics/hip_rotY").IsValid())
        self.assertFalse(stage.GetPrimAtPath("/Robot/Physics/hip_rotZ").IsValid())

        self.assertFalse(stage.GetPrimAtPath("/Robot/Geometry/hip_ghost_1").IsValid())
        self.assertFalse(stage.GetPrimAtPath("/Robot/Geometry/hip_ghost_2").IsValid())

        new_prim = stage.GetPrimAtPath("/Robot/Physics/hip")
        self.assertTrue(new_prim.IsValid())
        self.assertTrue(new_prim.IsA(UsdPhysics.SphericalJoint))

        joint = UsdPhysics.Joint(new_prim)
        body0 = joint.GetBody0Rel().GetTargets()
        body1 = joint.GetBody1Rel().GetTargets()
        self.assertEqual(str(body0[0]), "/Robot/Geometry/body_a")
        self.assertEqual(str(body1[0]), "/Robot/Geometry/body_b")

        pos0 = joint.GetLocalPos0Attr().Get()
        self.assertAlmostEqual(float(pos0[2]), 0.5)

    async def test_reconstruct_d6_from_chain(self) -> None:
        """Reconstruct d6 from chain."""
        from isaacsim.asset.importer.urdf.impl.joint_reconstruction import (
            SourceJointInfo,
            reconstruct_source_joints,
        )

        stage = Usd.Stage.CreateInMemory()
        robot = UsdGeom.Xform.Define(stage, "/Robot")
        stage.SetDefaultPrim(robot.GetPrim())

        UsdGeom.Scope.Define(stage, "/Robot/Geometry")
        ba = UsdGeom.Xform.Define(stage, "/Robot/Geometry/base")
        UsdPhysics.RigidBodyAPI.Apply(ba.GetPrim())
        pl = UsdGeom.Xform.Define(stage, "/Robot/Geometry/platform")
        UsdPhysics.RigidBodyAPI.Apply(pl.GetPrim())
        gh = UsdGeom.Xform.Define(stage, "/Robot/Geometry/slider_ghost_1")
        UsdPhysics.RigidBodyAPI.Apply(gh.GetPrim())

        UsdGeom.Scope.Define(stage, "/Robot/Physics")
        j1 = UsdPhysics.RevoluteJoint.Define(stage, "/Robot/Physics/slider_rotX")
        j1.CreateBody0Rel().SetTargets(["/Robot/Geometry/base"])
        j1.CreateBody1Rel().SetTargets(["/Robot/Geometry/slider_ghost_1"])

        j2 = UsdPhysics.PrismaticJoint.Define(stage, "/Robot/Physics/slider_transZ")
        j2.CreateBody0Rel().SetTargets(["/Robot/Geometry/slider_ghost_1"])
        j2.CreateBody1Rel().SetTargets(["/Robot/Geometry/platform"])

        bc = SourceJointInfo(
            joint_name="slider_rotX",
            original_type="PhysicsJoint",
            original_name="slider",
            chain_joints=["slider_rotX", "slider_transZ"],
            ghost_links=["slider_ghost_1"],
            per_axis_limits={
                "rotX": {"low": -45, "high": 45},
                "transZ": {"low": 0, "high": 0.5},
            },
            per_axis_drives={
                "rotX": {"damping": 10, "stiffness": 100},
                "transZ": {"damping": 50, "stiffness": 500},
            },
        )

        count = reconstruct_source_joints(stage, [bc])
        self.assertEqual(count, 1)

        self.assertFalse(stage.GetPrimAtPath("/Robot/Physics/slider_rotX").IsValid())
        self.assertFalse(stage.GetPrimAtPath("/Robot/Physics/slider_transZ").IsValid())
        self.assertFalse(stage.GetPrimAtPath("/Robot/Geometry/slider_ghost_1").IsValid())

        new_prim = stage.GetPrimAtPath("/Robot/Physics/slider")
        self.assertTrue(new_prim.IsValid())
        self.assertTrue(new_prim.IsA(UsdPhysics.Joint))

        lim_rx = UsdPhysics.LimitAPI.Get(new_prim, "rotX")
        self.assertIsNotNone(lim_rx)
        self.assertAlmostEqual(float(lim_rx.GetLowAttr().Get()), -45.0)
        self.assertAlmostEqual(float(lim_rx.GetHighAttr().Get()), 45.0)

        lim_tz = UsdPhysics.LimitAPI.Get(new_prim, "transZ")
        self.assertIsNotNone(lim_tz)
        self.assertAlmostEqual(float(lim_tz.GetLowAttr().Get()), 0.0)
        self.assertAlmostEqual(float(lim_tz.GetHighAttr().Get()), 0.5)

        drv_rx = UsdPhysics.DriveAPI(new_prim, "rotX")
        self.assertAlmostEqual(float(drv_rx.GetDampingAttr().Get()), 10.0)
        self.assertAlmostEqual(float(drv_rx.GetStiffnessAttr().Get()), 100.0)

    async def test_no_reconstruction_without_breadcrumbs(self) -> None:
        """No reconstruction without breadcrumbs."""
        from isaacsim.asset.importer.urdf.impl.joint_reconstruction import (
            reconstruct_source_joints,
        )

        stage = Usd.Stage.CreateInMemory()
        UsdGeom.Xform.Define(stage, "/Robot")

        count = reconstruct_source_joints(stage, [])
        self.assertEqual(count, 0)


# ---------------------------------------------------------------------------
# Drive breadcrumb round-trip tests
# ---------------------------------------------------------------------------


class TestDriveBreadcrumbRoundTrip(omni.kit.test.AsyncTestCase):
    """Write URDF with drive breadcrumbs, parse them back, and reconstruct."""

    async def setUp(self) -> None:
        """Set up test fixtures."""
        await omni.usd.get_context().new_stage_async()
        await omni.kit.app.get_app().next_update_async()

    async def tearDown(self) -> None:
        """Tear down test fixtures."""
        await omni.kit.app.get_app().next_update_async()

    async def test_write_then_parse_physx_drive(self) -> None:
        """Write URDF with physx drive breadcrumb, parse back."""
        from isaacsim.asset.exporter.urdf.converter.joint_reader import JointData
        from isaacsim.asset.exporter.urdf.converter.link_reader import LinkData
        from isaacsim.asset.exporter.urdf.converter.urdf_writer import write_urdf
        from isaacsim.asset.importer.urdf.impl.drive_reconstruction import (
            parse_source_drive_breadcrumbs,
        )

        parent = LinkData(name="base")
        child = LinkData(name="arm")

        j = JointData(
            name="arm_joint",
            joint_type="revolute",
            parent_link="base",
            child_link="arm",
            axis=(0, 0, 1),
            limit_lower=-1.57,
            limit_upper=1.57,
            source_drive={
                "source": "physx",
                "instance": "angular",
                "drive": {"stiffness": 1000, "damping": 100, "max_force": 50},
                "armature": 0.01,
            },
        )

        with tempfile.TemporaryDirectory() as td:
            urdf_path = os.path.join(td, "test.urdf")
            write_urdf("test_robot", [parent, child], [j], [], urdf_path)

            results = parse_source_drive_breadcrumbs(urdf_path)

        self.assertEqual(len(results), 1)
        bc = results[0]
        self.assertEqual(bc.joint_name, "arm_joint")
        self.assertEqual(bc.source, "physx")
        self.assertEqual(bc.instance, "angular")
        self.assertAlmostEqual(bc.drive["stiffness"], 1000)
        self.assertAlmostEqual(bc.drive["damping"], 100)
        self.assertAlmostEqual(bc.armature, 0.01)

    async def test_write_then_parse_mujoco_drive(self) -> None:
        """Write URDF with mujoco drive breadcrumb, parse back."""
        from isaacsim.asset.exporter.urdf.converter.joint_reader import JointData
        from isaacsim.asset.exporter.urdf.converter.link_reader import LinkData
        from isaacsim.asset.exporter.urdf.converter.urdf_writer import write_urdf
        from isaacsim.asset.importer.urdf.impl.drive_reconstruction import (
            parse_source_drive_breadcrumbs,
        )

        parent = LinkData(name="base")
        child = LinkData(name="arm")

        j = JointData(
            name="arm_joint",
            joint_type="revolute",
            parent_link="base",
            child_link="arm",
            axis=(0, 0, 1),
            limit_lower=-1.57,
            limit_upper=1.57,
            source_drive={
                "source": "mujoco",
                "actuator": {
                    "gainPrm": [100, 0, 0, 0, 0, 0, 0, 0, 0, 0],
                    "biasPrm": [0, -100, -10, 0, 0, 0, 0, 0, 0, 0],
                    "gainType": "fixed",
                    "biasType": "affine",
                    "forceRange_max": 200,
                },
                "armature": 0.02,
            },
        )

        with tempfile.TemporaryDirectory() as td:
            urdf_path = os.path.join(td, "test.urdf")
            write_urdf("test_robot", [parent, child], [j], [], urdf_path)

            results = parse_source_drive_breadcrumbs(urdf_path)

        self.assertEqual(len(results), 1)
        bc = results[0]
        self.assertEqual(bc.source, "mujoco")
        self.assertEqual(bc.actuator["gainPrm"][0], 100)
        self.assertEqual(bc.actuator["gainType"], "fixed")
        self.assertAlmostEqual(bc.armature, 0.02)

    async def test_no_breadcrumb_for_plain_joint(self) -> None:
        """No breadcrumb written when source_drive is None."""
        from isaacsim.asset.exporter.urdf.converter.joint_reader import JointData
        from isaacsim.asset.exporter.urdf.converter.link_reader import LinkData
        from isaacsim.asset.exporter.urdf.converter.urdf_writer import write_urdf
        from isaacsim.asset.importer.urdf.impl.drive_reconstruction import (
            parse_source_drive_breadcrumbs,
        )

        parent = LinkData(name="base")
        child = LinkData(name="arm")

        j = JointData(
            name="arm_joint",
            joint_type="revolute",
            parent_link="base",
            child_link="arm",
            axis=(0, 0, 1),
            limit_lower=-1.57,
            limit_upper=1.57,
        )

        with tempfile.TemporaryDirectory() as td:
            urdf_path = os.path.join(td, "test.urdf")
            write_urdf("test_robot", [parent, child], [j], [], urdf_path)

            results = parse_source_drive_breadcrumbs(urdf_path)

        self.assertEqual(len(results), 0)

    async def test_reconstruct_physx_drive(self) -> None:
        """Reconstruct DriveAPI from parsed breadcrumb onto a USD stage."""
        from isaacsim.asset.importer.urdf.impl.drive_reconstruction import (
            SourceDriveInfo,
            reconstruct_source_drives,
        )

        stage = Usd.Stage.CreateInMemory()
        robot = UsdGeom.Xform.Define(stage, "/Robot")
        stage.SetDefaultPrim(robot.GetPrim())

        body_a = UsdGeom.Xform.Define(stage, "/Robot/Geometry/base")
        UsdPhysics.RigidBodyAPI.Apply(body_a.GetPrim())

        body_b = UsdGeom.Xform.Define(stage, "/Robot/Geometry/arm")
        UsdPhysics.RigidBodyAPI.Apply(body_b.GetPrim())

        UsdGeom.Scope.Define(stage, "/Robot/Physics")
        joint = UsdPhysics.RevoluteJoint.Define(stage, "/Robot/Physics/arm_joint")
        joint.CreateBody0Rel().SetTargets(["/Robot/Geometry/base"])
        joint.CreateBody1Rel().SetTargets(["/Robot/Geometry/arm"])

        bc = SourceDriveInfo(
            joint_name="arm_joint",
            source="physx",
            instance="angular",
            drive={"stiffness": 1000, "damping": 100, "max_force": 50},
            armature=0.01,
        )

        count = reconstruct_source_drives(stage, [bc])
        self.assertEqual(count, 1)

        jp = stage.GetPrimAtPath("/Robot/Physics/arm_joint")
        drv = UsdPhysics.DriveAPI(jp, "angular")
        self.assertAlmostEqual(float(drv.GetStiffnessAttr().Get()), 1000.0)
        self.assertAlmostEqual(float(drv.GetDampingAttr().Get()), 100.0)
        self.assertAlmostEqual(float(drv.GetMaxForceAttr().Get()), 50.0)

        from pxr import PhysxSchema

        physx = PhysxSchema.PhysxJointAPI(jp)
        self.assertAlmostEqual(float(physx.GetArmatureAttr().Get()), 0.01)

    async def test_reconstruct_mujoco_actuator(self) -> None:
        """Reconstruct MjcActuator from parsed breadcrumb onto a USD stage."""
        from isaacsim.asset.importer.urdf.impl.drive_reconstruction import (
            SourceDriveInfo,
            reconstruct_source_drives,
        )

        stage = Usd.Stage.CreateInMemory()
        robot = UsdGeom.Xform.Define(stage, "/Robot")
        stage.SetDefaultPrim(robot.GetPrim())

        body_a = UsdGeom.Xform.Define(stage, "/Robot/Geometry/base")
        UsdPhysics.RigidBodyAPI.Apply(body_a.GetPrim())

        body_b = UsdGeom.Xform.Define(stage, "/Robot/Geometry/arm")
        UsdPhysics.RigidBodyAPI.Apply(body_b.GetPrim())

        UsdGeom.Scope.Define(stage, "/Robot/Physics")
        joint = UsdPhysics.RevoluteJoint.Define(stage, "/Robot/Physics/arm_joint")
        joint.CreateBody0Rel().SetTargets(["/Robot/Geometry/base"])
        joint.CreateBody1Rel().SetTargets(["/Robot/Geometry/arm"])

        bc = SourceDriveInfo(
            joint_name="arm_joint",
            source="mujoco",
            actuator={
                "gainPrm": [100, 0, 0, 0, 0, 0, 0, 0, 0, 0],
                "biasPrm": [0, -100, -10, 0, 0, 0, 0, 0, 0, 0],
                "gainType": "fixed",
                "biasType": "affine",
                "forceRange_max": 200,
            },
            armature=0.01,
        )

        count = reconstruct_source_drives(stage, [bc])
        self.assertEqual(count, 1)

        act_prim = stage.GetPrimAtPath("/Robot/Physics/arm_joint_actuator")
        self.assertTrue(act_prim.IsValid())
        self.assertEqual(act_prim.GetTypeName(), "MjcActuator")

        gain_prm = act_prim.GetAttribute("mjc:gainPrm").Get()
        self.assertAlmostEqual(float(gain_prm[0]), 100.0)

        bias_prm = act_prim.GetAttribute("mjc:biasPrm").Get()
        self.assertAlmostEqual(float(bias_prm[1]), -100.0)

        self.assertEqual(str(act_prim.GetAttribute("mjc:gainType").Get()), "fixed")

        fr_max = act_prim.GetAttribute("mjc:forceRange:max").Get()
        self.assertAlmostEqual(float(fr_max), 200.0)

        from pxr import PhysxSchema

        jp = stage.GetPrimAtPath("/Robot/Physics/arm_joint")
        physx = PhysxSchema.PhysxJointAPI(jp)
        self.assertAlmostEqual(float(physx.GetArmatureAttr().Get()), 0.01)

    async def test_reconstruct_armature_only(self) -> None:
        """Reconstruct just armature when no drive/actuator data present."""
        from isaacsim.asset.importer.urdf.impl.drive_reconstruction import (
            SourceDriveInfo,
            reconstruct_source_drives,
        )
        from pxr import PhysxSchema

        stage = Usd.Stage.CreateInMemory()
        robot = UsdGeom.Xform.Define(stage, "/Robot")
        stage.SetDefaultPrim(robot.GetPrim())

        UsdGeom.Scope.Define(stage, "/Robot/Physics")
        joint = UsdPhysics.RevoluteJoint.Define(stage, "/Robot/Physics/arm_joint")

        bc = SourceDriveInfo(
            joint_name="arm_joint",
            source="physx",
            instance="angular",
            armature=0.05,
        )

        count = reconstruct_source_drives(stage, [bc])
        self.assertEqual(count, 1)

        jp = stage.GetPrimAtPath("/Robot/Physics/arm_joint")
        physx = PhysxSchema.PhysxJointAPI(jp)
        self.assertAlmostEqual(float(physx.GetArmatureAttr().Get()), 0.05)
