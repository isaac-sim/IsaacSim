# SPDX-FileCopyrightText: Copyright (c) 2018-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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
"""Tests for the URDF exporter extension."""

from __future__ import annotations

import asyncio
import math
import os
import re
import tempfile
import xml.etree.ElementTree as ET
from typing import Any

import carb
import isaacsim.core.experimental.utils.stage as stage_utils
import omni.kit.test
from isaacsim.asset.exporter.urdf.converter import UsdToUrdfConverter
from isaacsim.asset.exporter.urdf.converter.inertia_utils import (
    read_inertial_from_prim,
    read_newton_inertia,
    reconstruct_inertia_tensor,
)
from isaacsim.asset.exporter.urdf.converter.joint_reader import _read_joint_velocity_limit
from isaacsim.storage.native import get_assets_root_path
from pxr import Gf, Sdf, Usd, UsdGeom, UsdPhysics, UsdShade


class TestUrdfExporter(omni.kit.test.AsyncTestCase):
    """Test suite for USD-to-URDF export functionality."""

    async def setUp(self) -> None:
        """Set up test fixtures."""
        self._timeline = omni.timeline.get_timeline_interface()
        await omni.usd.get_context().new_stage_async()
        await omni.kit.app.get_app().next_update_async()

    async def tearDown(self) -> None:
        """Tear down test fixtures."""
        while omni.usd.get_context().get_stage_loading_status()[2] > 0:
            await asyncio.sleep(1.0)

        if self._timeline and self._timeline.is_playing():
            self._timeline.stop()

        await omni.kit.app.get_app().next_update_async()
        await omni.usd.get_context().new_stage_async()
        await omni.kit.app.get_app().next_update_async()

    async def _open_robot(self, robot_path: str) -> Usd.Stage:
        """Open a robot USD from the assets server.

        Args:
            robot_path: Robot asset path relative to the assets root.

        Returns:
            Opened USD stage.

        The assets root resolves via :func:`get_assets_root_path`, which reads
        the ``/persistent/isaac/asset_root/default`` carb setting. To target a
        different server (e.g. a dev Nucleus), override that setting at launch
        with ``--/persistent/isaac/asset_root/default=<url>`` rather than
        hard-coding any server URL here.
        """
        assets_root = get_assets_root_path().rstrip("/")
        full_path = f"{assets_root}/{robot_path.lstrip('/')}"
        carb.log_warn(f"[test_exporter_urdf] Loading robot asset from: {full_path}")
        print(f"[test_exporter_urdf] Loading robot asset from: {full_path}")
        await stage_utils.open_stage_async(full_path)
        return stage_utils.get_current_stage()

    async def _export_and_validate_urdf(
        self, stage: Usd.Stage, root_prim: str | None, expected_links: int, expected_joints: int
    ) -> str:
        """Export a robot to URDF and validate basic structure.

        Args:
            stage: USD stage to read.
            root_prim: Robot root prim.
            expected_links: Minimum expected link count.
            expected_joints: Minimum expected joint count.

        Returns:
            Path to the exported URDF file.
        """
        with tempfile.TemporaryDirectory(ignore_cleanup_errors=True) as temp_dir:
            output_path = os.path.join(temp_dir, "robot.urdf")

            converter = UsdToUrdfConverter(
                stage=stage,
                root_prim_path=root_prim,
                mesh_dir_name="meshes",
                mesh_path_prefix="./",
            )
            converter.convert(output_path)

            self.assertTrue(os.path.exists(output_path), "URDF file was not created")

            tree = ET.parse(output_path)
            root = tree.getroot()
            self.assertEqual(root.tag, "robot", "Root element should be <robot>")

            links = root.findall("link")
            joints = root.findall("joint")

            self.assertGreaterEqual(
                len(links), expected_links, f"Expected at least {expected_links} links, got {len(links)}"
            )
            self.assertGreaterEqual(
                len(joints), expected_joints, f"Expected at least {expected_joints} joints, got {len(joints)}"
            )

            for link in links:
                self.assertIsNotNone(link.get("name"), "Link must have a name")

            for joint in joints:
                self.assertIsNotNone(joint.get("name"), "Joint must have a name")
                self.assertIsNotNone(joint.get("type"), "Joint must have a type")
                parent = joint.find("parent")
                child = joint.find("child")
                self.assertIsNotNone(parent, "Joint must have a parent")
                self.assertIsNotNone(child, "Joint must have a child")

            await stage_utils.create_new_stage_async()
            await omni.kit.app.get_app().next_update_async()

            return output_path

    async def _export_to_urdf(
        self, stage: Usd.Stage, temp_dir: str, root_prim: str | None = None, **converter_kwargs: Any  # noqa: ANN401
    ) -> tuple[str, ET.Element]:
        """Export robot to URDF, parse XML, and return (output_path, xml_root).

        Args:
            stage: USD stage to read.
            temp_dir: Temporary output directory.
            root_prim: Robot root prim.
            **converter_kwargs: Additional converter keyword arguments.

        Returns:
            URDF output path and parsed XML root.
        """
        output_path = os.path.join(temp_dir, "robot.urdf")
        kwargs: dict[str, Any] = {
            "stage": stage,
            "root_prim_path": root_prim,
            "mesh_dir_name": "meshes",
            "mesh_path_prefix": "./",
        }
        kwargs.update(converter_kwargs)
        converter = UsdToUrdfConverter(**kwargs)
        converter.convert(output_path)
        self.assertTrue(os.path.exists(output_path), "URDF file was not created")
        tree = ET.parse(output_path)
        root = tree.getroot()
        self.assertEqual(root.tag, "robot", "Root element should be <robot>")
        return output_path, root

    @staticmethod
    def _create_continuous_joint_stage() -> tuple[Usd.Stage, UsdPhysics.RevoluteJoint]:
        """Create a minimal stage containing a continuous revolute joint.

        Returns:
            Generated stage and continuous joint.
        """
        from pxr import Sdf, UsdGeom

        stage = Usd.Stage.CreateInMemory()
        robot = UsdGeom.Xform.Define(stage, "/robot")
        stage.SetDefaultPrim(robot.GetPrim())
        UsdPhysics.ArticulationRootAPI.Apply(robot.GetPrim())

        for path in ("/robot/base", "/robot/continuous_child"):
            link = UsdGeom.Xform.Define(stage, path)
            UsdPhysics.RigidBodyAPI.Apply(link.GetPrim())
            UsdPhysics.MassAPI.Apply(link.GetPrim()).CreateMassAttr().Set(1.0)

        world_joint = UsdPhysics.FixedJoint.Define(stage, "/robot/world_joint")
        world_joint.GetBody1Rel().SetTargets([Sdf.Path("/robot/base")])

        unbounded = UsdPhysics.RevoluteJoint.Define(stage, "/robot/continuous_joint")
        unbounded.GetBody0Rel().SetTargets([Sdf.Path("/robot/base")])
        unbounded.GetBody1Rel().SetTargets([Sdf.Path("/robot/continuous_child")])
        unbounded.GetPrim().CreateAttribute("urdf:limit:effort", Sdf.ValueTypeNames.Float).Set(42.0)

        return stage, unbounded

    async def test_unbounded_revolute_joint_exports_as_continuous(self) -> None:
        """Export a continuous joint with its finite Newton velocity limit."""
        from pxr import Sdf

        stage, unbounded = self._create_continuous_joint_stage()
        unbounded.GetPrim().ApplyAPI("NewtonJointAPI")
        newton_velocity_attr = unbounded.GetPrim().CreateAttribute("newton:velocityLimit", Sdf.ValueTypeNames.Float)
        newton_velocity_attr.Set(math.degrees(3.5))

        self.assertEqual(float(unbounded.GetLowerLimitAttr().Get()), -math.inf)
        self.assertEqual(float(unbounded.GetUpperLimitAttr().Get()), math.inf)

        with tempfile.TemporaryDirectory(ignore_cleanup_errors=True) as temp_dir:
            _, root = await self._export_to_urdf(stage, temp_dir)

        joints = {joint.get("name"): joint for joint in root.findall("joint")}
        continuous_joint = joints["continuous_joint"]
        self.assertEqual(continuous_joint.get("type"), "continuous")

        limit = continuous_joint.find("limit")
        self.assertIsNotNone(limit)
        self.assertIsNone(limit.get("lower"))
        self.assertIsNone(limit.get("upper"))
        self.assertAlmostEqual(float(limit.get("effort")), 42.0)
        self.assertAlmostEqual(float(limit.get("velocity")), 3.5, places=6)

    async def test_missing_joint_velocity_is_omitted(self) -> None:
        """Omit the URDF velocity when no supported source is authored."""
        stage, _ = self._create_continuous_joint_stage()

        with tempfile.TemporaryDirectory(ignore_cleanup_errors=True) as temp_dir:
            _, root = await self._export_to_urdf(stage, temp_dir)

        continuous_joint = root.find("joint[@name='continuous_joint']")
        self.assertIsNotNone(continuous_joint)
        limit = continuous_joint.find("limit")
        self.assertIsNotNone(limit)
        self.assertIsNone(limit.get("velocity"))

    async def test_joint_velocity_falls_back_to_legacy_urdf(self) -> None:
        """Use the legacy URDF velocity when Newton has no authored value."""
        from pxr import Sdf

        stage, unbounded = self._create_continuous_joint_stage()
        unbounded.GetPrim().CreateAttribute("urdf:limit:velocity", Sdf.ValueTypeNames.Float).Set(2.5)

        with self.assertLogs("isaacsim.asset.exporter.urdf.converter.joint_reader", level="WARNING") as logs:
            with tempfile.TemporaryDirectory(ignore_cleanup_errors=True) as temp_dir:
                _, root = await self._export_to_urdf(stage, temp_dir)

        self.assertTrue(any("uses legacy attribute urdf:limit:velocity" in message for message in logs.output))

        continuous_joint = root.find("joint[@name='continuous_joint']")
        self.assertIsNotNone(continuous_joint)
        limit = continuous_joint.find("limit")
        self.assertIsNotNone(limit)
        self.assertAlmostEqual(float(limit.get("velocity")), 2.5, places=6)

    async def test_infinite_newton_velocity_blocks_other_sources(self) -> None:
        """Treat an authored infinite Newton limit as authoritative and unbounded."""
        from pxr import PhysxSchema, Sdf

        stage, unbounded = self._create_continuous_joint_stage()
        unbounded.GetPrim().CreateAttribute("urdf:limit:velocity", Sdf.ValueTypeNames.Float).Set(2.5)
        unbounded.GetPrim().ApplyAPI("NewtonJointAPI")
        newton_velocity_attr = unbounded.GetPrim().CreateAttribute("newton:velocityLimit", Sdf.ValueTypeNames.Float)
        newton_velocity_attr.Set(math.inf)
        physx_joint = PhysxSchema.PhysxJointAPI.Apply(unbounded.GetPrim())
        physx_joint.CreateMaxJointVelocityAttr().Set(math.degrees(1.0))

        with tempfile.TemporaryDirectory(ignore_cleanup_errors=True) as temp_dir:
            _, root = await self._export_to_urdf(stage, temp_dir)

        continuous_joint = root.find("joint[@name='continuous_joint']")
        self.assertIsNotNone(continuous_joint)
        limit = continuous_joint.find("limit")
        self.assertIsNotNone(limit)
        self.assertIsNone(limit.get("velocity"))

    async def test_prismatic_velocity_uses_linear_units(self) -> None:
        """Read a Newton prismatic velocity without angular conversion."""
        from pxr import Sdf

        stage = Usd.Stage.CreateInMemory()
        slider = UsdPhysics.PrismaticJoint.Define(stage, "/slider_joint").GetPrim()
        slider.CreateAttribute("newton:velocityLimit", Sdf.ValueTypeNames.Float).Set(0.25)

        self.assertAlmostEqual(_read_joint_velocity_limit(slider), 0.25)

    async def test_inverted_revolute_limits_export_as_fixed(self) -> None:
        """Export a USD revolute joint with inverted limits as a URDF fixed joint."""
        from pxr import Sdf, UsdGeom

        stage = Usd.Stage.CreateInMemory()
        robot = UsdGeom.Xform.Define(stage, "/robot")
        stage.SetDefaultPrim(robot.GetPrim())
        UsdPhysics.ArticulationRootAPI.Apply(robot.GetPrim())

        for path in ("/robot/base", "/robot/locked_child"):
            link = UsdGeom.Xform.Define(stage, path)
            UsdPhysics.RigidBodyAPI.Apply(link.GetPrim())
            UsdPhysics.MassAPI.Apply(link.GetPrim()).CreateMassAttr().Set(1.0)

        world_joint = UsdPhysics.FixedJoint.Define(stage, "/robot/world_joint")
        world_joint.GetBody1Rel().SetTargets([Sdf.Path("/robot/base")])

        locked = UsdPhysics.RevoluteJoint.Define(stage, "/robot/locked_joint")
        locked.GetBody0Rel().SetTargets([Sdf.Path("/robot/base")])
        locked.GetBody1Rel().SetTargets([Sdf.Path("/robot/locked_child")])
        locked.GetLowerLimitAttr().Set(90.0)
        locked.GetUpperLimitAttr().Set(-90.0)

        with tempfile.TemporaryDirectory(ignore_cleanup_errors=True) as temp_dir:
            _, root = await self._export_to_urdf(stage, temp_dir)

        joints = {joint.get("name"): joint for joint in root.findall("joint")}
        locked_joint = joints["locked_joint"]
        self.assertEqual(locked_joint.get("type"), "fixed")
        self.assertIsNone(locked_joint.find("axis"))
        self.assertIsNone(locked_joint.find("limit"))

    @staticmethod
    def _parse_xyz(text: str) -> list[float]:
        """Parse a space-separated xyz or rpy string into a list of floats.

        Args:
            text: Value to use.

        Returns:
            Parsed numeric values.
        """
        return [float(v) for v in text.strip().split()]

    # ------------------------------------------------------------------
    # UR10e asset test — single load, single export, all validations
    # ------------------------------------------------------------------

    async def test_exporter_ur10e_comprehensive(self) -> None:
        """Load UR10e once, export once, validate structure + correctness + meshes."""
        stage = await self._open_robot("Isaac/Robots_Multiphysics/UniversalRobots/ur10e/ur10e.usda")
        if stage is None:
            self.skipTest("Could not open ur10e asset")
            return

        errors: list[str] = []

        with tempfile.TemporaryDirectory(ignore_cleanup_errors=True) as temp_dir:
            output_path, root = await self._export_to_urdf(stage, temp_dir)
            urdf_dir = os.path.dirname(output_path)
            mesh_dir = os.path.join(temp_dir, "meshes")

            # ---- XML well-formedness ----
            with open(output_path, "rb") as f:
                raw = f.read()
            if not raw.startswith(b"<?xml"):
                errors.append("URDF file should start with XML declaration")
            if b'encoding="utf-8"' not in raw.lower().replace(b"'", b'"'):
                errors.append("URDF file should declare utf-8 encoding")
            if not root.get("name"):
                errors.append("Robot name attribute must not be empty")

            # ---- Basic structure ----
            links = root.findall("link")
            joints = root.findall("joint")
            if len(links) < 2:
                errors.append(f"Expected at least 2 links, got {len(links)}")
            if len(joints) < 1:
                errors.append(f"Expected at least 1 joint, got {len(joints)}")

            for link in links:
                if not link.get("name"):
                    errors.append("Link has empty or missing name attribute")
            for joint in joints:
                if not joint.get("name"):
                    errors.append("Joint has empty or missing name attribute")
                if not joint.get("type"):
                    errors.append(f"Joint '{joint.get('name')}' has no type attribute")
                if joint.find("parent") is None:
                    errors.append(f"Joint '{joint.get('name')}' missing <parent>")
                if joint.find("child") is None:
                    errors.append(f"Joint '{joint.get('name')}' missing <child>")

            # ---- Link / joint name uniqueness ----
            link_names_list = [l.get("name") for l in links]
            if len(link_names_list) != len(set(link_names_list)):
                errors.append("Duplicate link names found")
            joint_names_list = [j.get("name") for j in joints]
            if len(joint_names_list) != len(set(joint_names_list)):
                errors.append("Duplicate joint names found")

            # ---- Tree connectivity ----
            link_names = set(link_names_list)
            child_counts: dict[str, int] = {}
            for joint in joints:
                jname = joint.get("name")
                p, c = joint.find("parent"), joint.find("child")
                if p is not None and p.get("link") not in link_names:
                    errors.append(f"Joint '{jname}' parent '{p.get('link')}' is not a declared link")
                if c is not None:
                    cname = c.get("link")
                    if cname not in link_names:
                        errors.append(f"Joint '{jname}' child '{cname}' is not a declared link")
                    child_counts[cname] = child_counts.get(cname, 0) + 1
            for name, count in child_counts.items():
                if count != 1:
                    errors.append(f"Link '{name}' appears as child in {count} joints (expected 1)")

            # ---- Joint types ----
            valid_types = {"revolute", "continuous", "prismatic", "fixed", "floating", "planar"}
            for joint in joints:
                jtype = joint.get("type")
                if jtype not in valid_types:
                    errors.append(f"Invalid joint type: {jtype}")

            # ---- Inertial data ----
            links_with_inertial = 0
            for link in links:
                inertial = link.find("inertial")
                if inertial is None:
                    continue
                links_with_inertial += 1
                mass_elem = inertial.find("mass")
                if mass_elem is None:
                    errors.append(f"Link '{link.get('name')}' inertial missing <mass>")
                elif float(mass_elem.get("value", "0")) <= 0.0:
                    errors.append(f"Link '{link.get('name')}' mass must be positive")
                inertia = inertial.find("inertia")
                if inertia is None:
                    errors.append(f"Link '{link.get('name')}' inertial missing <inertia>")
                else:
                    for attr in ("ixx", "iyy", "izz"):
                        if float(inertia.get(attr, "0")) < 0.0:
                            errors.append(f"Link '{link.get('name')}' {attr} must be non-negative")
            if links_with_inertial == 0:
                errors.append("Expected at least one link with inertial data")

            # ---- Revolute limits, axes, dynamics ----
            revolute_count = 0
            axis_joint_types = {"revolute", "continuous", "prismatic", "planar"}
            for joint in joints:
                jtype, jname = joint.get("type"), joint.get("name")
                if jtype == "revolute":
                    revolute_count += 1
                    limit = joint.find("limit")
                    if limit is None:
                        errors.append(f"Revolute joint '{jname}' missing <limit>")
                    else:
                        lo = float(limit.get("lower", "0"))
                        hi = float(limit.get("upper", "0"))
                        if not (math.isfinite(lo) and math.isfinite(hi)):
                            errors.append(f"Joint '{jname}' limits must be finite")
                        elif lo >= hi:
                            errors.append(f"Joint '{jname}' lower ({lo}) must be < upper ({hi})")
                        if abs(lo) >= 4 * math.pi or abs(hi) >= 4 * math.pi:
                            errors.append(f"Joint '{jname}' limits look like degrees")
                if jtype in axis_joint_types:
                    axis_elem = joint.find("axis")
                    if axis_elem is None:
                        errors.append(f"Joint '{jname}' (type={jtype}) missing <axis>")
                    else:
                        xyz = self._parse_xyz(axis_elem.get("xyz"))
                        mag = math.sqrt(sum(v * v for v in xyz))
                        if abs(mag - 1.0) > 1e-4:
                            errors.append(f"Joint '{jname}' axis magnitude {mag} != 1.0")
                dynamics = joint.find("dynamics")
                if dynamics is not None:
                    for da in ("damping", "friction"):
                        dv = dynamics.get(da)
                        if dv is not None:
                            fv = float(dv)
                            if fv < 0.0 or not math.isfinite(fv):
                                errors.append(f"Joint '{jname}' {da} invalid: {fv}")
            if revolute_count == 0:
                errors.append("UR10e should have at least one revolute joint")

            # ---- Origins ----
            origin_count = 0
            for origin in root.iter("origin"):
                for oa in ("xyz", "rpy"):
                    os_ = origin.get(oa)
                    if os_ is not None:
                        vals = self._parse_xyz(os_)
                        if len(vals) != 3:
                            errors.append(f"origin {oa} must have 3 values")
                        elif any(not math.isfinite(v) for v in vals):
                            errors.append(f"origin {oa} contains non-finite value")
                origin_count += 1
            if origin_count == 0:
                errors.append("Expected at least one <origin> element")

            # ---- Visual / collision geometry ----
            valid_geom_tags = {"box", "sphere", "cylinder", "mesh"}
            links_with_visual = 0
            for link in links:
                if link.findall("visual"):
                    links_with_visual += 1
                for vis in link.findall("visual"):
                    geom = vis.find("geometry")
                    if geom is None:
                        errors.append(f"Visual in link '{link.get('name')}' missing <geometry>")
                    elif any(c.tag not in valid_geom_tags for c in geom):
                        errors.append(f"Unknown geometry type in link '{link.get('name')}'")
                for col in link.findall("collision"):
                    if col.find("geometry") is None:
                        errors.append(f"Collision in link '{link.get('name')}' missing <geometry>")
            if links_with_visual == 0:
                errors.append("Expected at least one link with visual geometry")

            # ---- Mesh file integrity ----
            mesh_elems = list(root.iter("mesh"))
            if not mesh_elems:
                errors.append("Expected mesh elements in URDF")
            for mesh in mesh_elems:
                fn = mesh.get("filename", "")
                if not fn:
                    errors.append("Mesh element has empty filename")
                    continue
                resolved = os.path.normpath(os.path.join(urdf_dir, fn))
                if not os.path.isfile(resolved):
                    errors.append(f"Mesh file '{fn}' does not exist at '{resolved}'")

            if os.path.isdir(mesh_dir):
                obj_files = [f for f in os.listdir(mesh_dir) if f.endswith(".obj")]
                if not obj_files:
                    errors.append("No OBJ files exported")
                vertex_re = re.compile(r"^v\s+[-\d.eE+]+\s+[-\d.eE+]+\s+[-\d.eE+]+", re.MULTILINE)
                face_re = re.compile(r"^f\s+\S+", re.MULTILINE)
                mtllib_re = re.compile(r"^mtllib\s+(.+)$", re.MULTILINE)
                for obj_name in obj_files:
                    with open(os.path.join(mesh_dir, obj_name)) as f:
                        content = f.read()
                    if not vertex_re.findall(content):
                        errors.append(f"OBJ '{obj_name}' has no vertices")
                    if not face_re.findall(content):
                        errors.append(f"OBJ '{obj_name}' has no faces")
                    has_usemtl = re.search(r"^usemtl\s+", content, re.MULTILINE)
                    match = mtllib_re.search(content)
                    if match and has_usemtl:
                        mtl_name = match.group(1).strip()
                        if not os.path.isfile(os.path.join(mesh_dir, mtl_name)):
                            errors.append(f"OBJ '{obj_name}' references missing MTL '{mtl_name}'")
            else:
                errors.append("Mesh directory not created")

            # ---- Mesh prefix options (reuses same loaded stage) ----
            pkg_dir = os.path.join(temp_dir, "pkg")
            os.makedirs(pkg_dir)
            _, pkg_root = await self._export_to_urdf(stage, pkg_dir, mesh_path_prefix="package://my_robot_description/")
            for mesh in pkg_root.iter("mesh"):
                fn = mesh.get("filename", "")
                if not fn.startswith("package://my_robot_description/meshes/"):
                    errors.append(f"package:// prefix: '{fn}' has wrong prefix")
                if not fn.endswith(".obj"):
                    errors.append(f"package:// prefix: '{fn}' should end with .obj")

            file_dir = os.path.join(temp_dir, "file")
            os.makedirs(file_dir)
            _, file_root = await self._export_to_urdf(stage, file_dir, mesh_path_prefix="file://")
            for mesh in file_root.iter("mesh"):
                fn = mesh.get("filename", "")
                if not fn.startswith("file://"):
                    errors.append(f"file:// prefix: '{fn}' should start with 'file://'")

        self.assertEqual(errors, [], "UR10e export validation errors:\n  " + "\n  ".join(errors))

        await stage_utils.create_new_stage_async()
        await omni.kit.app.get_app().next_update_async()

    @staticmethod
    def _build_duplicate_site_name_stage() -> Usd.Stage:
        """Build a fixed-base robot whose base link carries a same-named site.

        Multiphysics USDA assets can tag ``/robot/base_link/base_link`` with
        ``IsaacSiteAPI``. That frame is redundant with the link itself and must
        not be exported as a second URDF link.

        Returns:
            Generated in-memory USD stage.
        """
        from pxr import Sdf, UsdGeom
        from usd.schema.isaac.robot_schema import ApplySiteAPI

        stage = Usd.Stage.CreateInMemory()
        UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
        UsdGeom.SetStageMetersPerUnit(stage, 1.0)

        robot = UsdGeom.Xform.Define(stage, "/robot")
        stage.SetDefaultPrim(robot.GetPrim())
        UsdPhysics.ArticulationRootAPI.Apply(robot.GetPrim())

        base = UsdGeom.Xform.Define(stage, "/robot/base_link")
        UsdPhysics.RigidBodyAPI.Apply(base.GetPrim())
        UsdPhysics.MassAPI.Apply(base.GetPrim()).CreateMassAttr().Set(1.0)

        world_joint = UsdPhysics.FixedJoint.Define(stage, "/robot/world_joint")
        world_joint.GetBody1Rel().SetTargets([Sdf.Path("/robot/base_link")])

        duplicate_site = UsdGeom.Xform.Define(stage, "/robot/base_link/base_link")
        ApplySiteAPI(duplicate_site.GetPrim())

        flange = UsdGeom.Xform.Define(stage, "/robot/base_link/flange")
        ApplySiteAPI(flange.GetPrim())

        return stage

    async def test_site_skipped_when_name_collides_with_link(self) -> None:
        """Sites whose resolved name matches an existing link are not exported."""
        stage = self._build_duplicate_site_name_stage()

        with tempfile.TemporaryDirectory(ignore_cleanup_errors=True) as temp_dir:
            _, root = await self._export_to_urdf(stage, temp_dir)
            link_names = [link.get("name") for link in root.findall("link")]
            self.assertEqual(link_names.count("base_link"), 1)
            self.assertIn("flange", link_names)

    async def test_site_renamed_when_duplicate_ghost_links_enabled(self) -> None:
        """Sites whose names collide with links receive a numeric suffix when enabled."""
        stage = self._build_duplicate_site_name_stage()

        with tempfile.TemporaryDirectory(ignore_cleanup_errors=True) as temp_dir:
            _, root = await self._export_to_urdf(stage, temp_dir, export_duplicate_ghost_links=True)
            link_names = [link.get("name") for link in root.findall("link")]
            self.assertEqual(link_names.count("base_link"), 1)
            self.assertIn("base_link_1", link_names)

            joint = root.find("joint[@name='base_link_1_fixed_joint']")
            self.assertIsNotNone(joint)
            self.assertEqual(joint.find("parent").get("link"), "base_link")
            self.assertEqual(joint.find("child").get("link"), "base_link_1")

    async def test_exporter_round_trip(self) -> None:
        """Test round-trip: import URDF -> USD -> export URDF, validate consistency."""
        try:
            from isaacsim.asset.importer.urdf import URDFImporter, URDFImporterConfig
        except ImportError:
            self.skipTest("URDF importer not available")
            return

        ext_manager = omni.kit.app.get_app().get_extension_manager()
        urdf_ext_id = ext_manager.get_enabled_extension_id("isaacsim.asset.importer.urdf")
        if not urdf_ext_id:
            self.skipTest("isaacsim.asset.importer.urdf extension not enabled")
            return
        urdf_ext_path = ext_manager.get_extension_path(urdf_ext_id)
        urdf_file = os.path.join(urdf_ext_path, "data", "urdf", "tests", "test_basic.urdf")

        if not os.path.exists(urdf_file):
            self.skipTest(f"Test URDF not found: {urdf_file}")
            return

        with tempfile.TemporaryDirectory(ignore_cleanup_errors=True) as temp_dir:
            import_config = URDFImporterConfig()
            import_config.urdf_path = urdf_file
            import_config.usd_path = temp_dir

            importer = URDFImporter(import_config)
            usd_path = importer.import_urdf()

            await stage_utils.open_stage_async(usd_path)
            stage = stage_utils.get_current_stage()

            if stage is None:
                self.skipTest("Could not open imported USD stage")
                return

            stage.GetPrimAtPath("/test_basic").GetVariantSet("Physics").SetVariantSelection("physics")
            await omni.kit.app.get_app().next_update_async()

            output_urdf = os.path.join(temp_dir, "exported.urdf")
            converter = UsdToUrdfConverter(
                stage=stage,
                mesh_dir_name="meshes",
                mesh_path_prefix="./",
            )
            converter.convert(output_urdf)

            self.assertTrue(os.path.exists(output_urdf), "Round-trip URDF not created")

            tree = ET.parse(output_urdf)
            root = tree.getroot()
            self.assertEqual(root.tag, "robot")

            links = root.findall("link")
            joints = root.findall("joint")
            self.assertGreater(len(links), 0, "Round-trip URDF should have links")

            await stage_utils.create_new_stage_async()
            await omni.kit.app.get_app().next_update_async()

    # (joint_types, mesh_export, structural_validity, content_correctness,
    # mesh_integrity, mesh_prefix_options merged into test_exporter_ur10e_comprehensive)

    # ------------------------------------------------------------------
    # MuJoCo (mjc:*) fallback tests
    # ------------------------------------------------------------------

    @staticmethod
    def _build_mjc_stage() -> Usd.Stage:
        """Create a minimal in-memory USD stage with MuJoCo-authored physics.

        Builds a two-link robot (base -> child) with a revolute joint that
        has ONLY mjc:* attributes -- no urdf:* custom attrs, no DriveAPI,
        no PhysxJointAPI.  Also creates an MjcActuator targeting the joint.

        Returns:
            Generated in-memory USD stage.
        """
        from pxr import Sdf, UsdGeom

        stage = Usd.Stage.CreateInMemory()
        UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
        UsdGeom.SetStageMetersPerUnit(stage, 1.0)

        robot = UsdGeom.Xform.Define(stage, "/robot")
        stage.SetDefaultPrim(robot.GetPrim())
        UsdPhysics.ArticulationRootAPI.Apply(robot.GetPrim())

        base = UsdGeom.Xform.Define(stage, "/robot/base")
        UsdPhysics.RigidBodyAPI.Apply(base.GetPrim())
        mass_api = UsdPhysics.MassAPI.Apply(base.GetPrim())
        mass_api.CreateMassAttr().Set(1.0)

        child = UsdGeom.Xform.Define(stage, "/robot/base/child")
        UsdPhysics.RigidBodyAPI.Apply(child.GetPrim())
        mass_api_c = UsdPhysics.MassAPI.Apply(child.GetPrim())
        mass_api_c.CreateMassAttr().Set(0.5)

        joint = UsdPhysics.RevoluteJoint.Define(stage, "/robot/base/child/joint")
        joint.GetBody0Rel().SetTargets([Sdf.Path("/robot/base")])
        joint.GetBody1Rel().SetTargets([Sdf.Path("/robot/base/child")])
        joint.GetAxisAttr().Set("Z")
        joint.GetLowerLimitAttr().Set(-90.0)
        joint.GetUpperLimitAttr().Set(90.0)

        jp = joint.GetPrim()
        jp.CreateAttribute("mjc:damping", Sdf.ValueTypeNames.Float).Set(2.5)
        jp.CreateAttribute("mjc:frictionloss", Sdf.ValueTypeNames.Float).Set(0.3)
        jp.CreateAttribute("mjc:ref", Sdf.ValueTypeNames.Float).Set(15.0)
        jp.CreateAttribute("mjc:actuatorfrcrange:max", Sdf.ValueTypeNames.Float).Set(50.0)

        world_joint = UsdPhysics.FixedJoint.Define(stage, "/robot/world_joint")
        world_joint.GetBody1Rel().SetTargets([Sdf.Path("/robot/base")])

        UsdGeom.Scope.Define(stage, "/robot/Physics")
        actuator = stage.DefinePrim("/robot/Physics/joint_actuator", "MjcActuator")
        actuator.CreateRelationship("mjc:target", custom=False).SetTargets([Sdf.Path("/robot/base/child/joint")])
        actuator.CreateAttribute("mjc:forceRange:max", Sdf.ValueTypeNames.Float).Set(200.0)
        actuator.CreateAttribute("mjc:gainPrm", Sdf.ValueTypeNames.FloatArray).Set([100.0, 0, 0, 0, 0, 0, 0, 0, 0, 0])
        actuator.CreateAttribute("mjc:biasPrm", Sdf.ValueTypeNames.FloatArray).Set(
            [0, -100.0, -10.0, 0, 0, 0, 0, 0, 0, 0]
        )
        actuator.CreateAttribute("mjc:gainType", Sdf.ValueTypeNames.Token).Set("fixed")
        actuator.CreateAttribute("mjc:biasType", Sdf.ValueTypeNames.Token).Set("affine")

        return stage

    def _export_mjc_stage(self, stage: Usd.Stage, temp_dir: str) -> ET.Element:
        """Export an in-memory stage to URDF and return the parsed XML root.

        Args:
            stage: USD stage to read.
            temp_dir: Temporary output directory.

        Returns:
            Parsed URDF XML root.
        """
        output_path = os.path.join(temp_dir, "mjc_robot.urdf")
        converter = UsdToUrdfConverter(stage=stage, mesh_dir_name="meshes", mesh_path_prefix="./")
        converter.convert(output_path)
        self.assertTrue(os.path.exists(output_path))
        return ET.parse(output_path).getroot()

    async def test_postprocess_modifies_robot_element(self) -> None:
        """Apply a postprocessor before writing the URDF."""
        stage = self._build_mjc_stage()

        with tempfile.TemporaryDirectory(ignore_cleanup_errors=True) as temp_dir:
            output_path = os.path.join(temp_dir, "robot.urdf")
            converter = UsdToUrdfConverter(stage=stage)

            def postprocess(root: ET.Element) -> None:
                ET.SubElement(root, "custom", name="test")

            converter.convert(output_path, postprocess=postprocess)
            root = ET.parse(output_path).getroot()

        self.assertIsNotNone(root.find("custom[@name='test']"))

    async def test_postprocess_failure_prevents_urdf_write(self) -> None:
        """Propagate postprocessor failures without writing a URDF file."""
        stage = self._build_mjc_stage()

        with tempfile.TemporaryDirectory(ignore_cleanup_errors=True) as temp_dir:
            output_path = os.path.join(temp_dir, "robot.urdf")
            converter = UsdToUrdfConverter(stage=stage)

            def postprocess(_root: ET.Element) -> None:
                raise RuntimeError("postprocess failed")

            with self.assertRaisesRegex(RuntimeError, "postprocess failed"):
                converter.convert(output_path, postprocess=postprocess)
            self.assertFalse(os.path.exists(output_path))

    async def test_mjc_joint_dynamics_export(self) -> None:
        """Verify that mjc:damping and mjc:frictionloss are exported as URDF <dynamics>."""
        stage = self._build_mjc_stage()

        with tempfile.TemporaryDirectory(ignore_cleanup_errors=True) as temp_dir:
            root = self._export_mjc_stage(stage, temp_dir)

            revolute_joints = [j for j in root.findall("joint") if j.get("type") == "revolute"]
            self.assertEqual(len(revolute_joints), 1)

            dynamics = revolute_joints[0].find("dynamics")
            self.assertIsNotNone(dynamics, "Expected <dynamics> element from mjc: fallback")
            self.assertAlmostEqual(float(dynamics.get("damping")), 2.5, places=4)
            self.assertAlmostEqual(float(dynamics.get("friction")), 0.3, places=4)

    async def test_mjc_effort_not_in_limit(self) -> None:
        """mjc:actuatorfrcrange:max is actuation data; it must NOT appear in <limit effort>."""
        stage = self._build_mjc_stage()

        with tempfile.TemporaryDirectory(ignore_cleanup_errors=True) as temp_dir:
            root = self._export_mjc_stage(stage, temp_dir)

            revolute_joints = [j for j in root.findall("joint") if j.get("type") == "revolute"]
            self.assertEqual(len(revolute_joints), 1)

            limit = revolute_joints[0].find("limit")
            if limit is not None:
                self.assertIsNone(limit.get("effort"), "Actuator effort should NOT be in <limit>")

    async def test_mjc_ref_calibration_export(self) -> None:
        """Verify that mjc:ref is exported as URDF <calibration reference_position>."""
        stage = self._build_mjc_stage()

        with tempfile.TemporaryDirectory(ignore_cleanup_errors=True) as temp_dir:
            root = self._export_mjc_stage(stage, temp_dir)

            revolute_joints = [j for j in root.findall("joint") if j.get("type") == "revolute"]
            self.assertEqual(len(revolute_joints), 1)

            calibration = revolute_joints[0].find("calibration")
            self.assertIsNotNone(calibration, "Expected <calibration> element from mjc:ref")
            self.assertAlmostEqual(float(calibration.get("reference_position")), 15.0, places=4)

    async def test_mjc_actuator_force_range_in_breadcrumb(self) -> None:
        """Actuator forceRange goes to breadcrumb, not <limit effort>."""
        import json

        stage = self._build_mjc_stage()

        with tempfile.TemporaryDirectory(ignore_cleanup_errors=True) as temp_dir:
            output_path = os.path.join(temp_dir, "mjc_robot.urdf")
            converter = UsdToUrdfConverter(stage=stage, mesh_dir_name="meshes", mesh_path_prefix="./")
            converter.convert(output_path)

            with open(output_path) as f:
                urdf_text = f.read()

            prefix = " isaac:source_drive "
            found = False
            for line in urdf_text.splitlines():
                if prefix in line:
                    json_str = line.split(prefix, 1)[1].rstrip(" ->\n").rstrip()
                    meta = json.loads(json_str)
                    if meta.get("source") == "mujoco":
                        self.assertIn("forceRange_max", meta["actuator"])
                        found = True
                        break
            self.assertTrue(found, "Expected mujoco breadcrumb with forceRange")

    async def test_mjc_actuator_damping_not_in_dynamics(self) -> None:
        """Actuator-derived Kd is actuation data; it must NOT appear in <dynamics>."""
        stage = self._build_mjc_stage()
        stage.GetPrimAtPath("/robot/base/child/joint").GetAttribute("mjc:damping").Clear()

        with tempfile.TemporaryDirectory(ignore_cleanup_errors=True) as temp_dir:
            root = self._export_mjc_stage(stage, temp_dir)

            revolute_joints = [j for j in root.findall("joint") if j.get("type") == "revolute"]
            dynamics = revolute_joints[0].find("dynamics")
            if dynamics is not None:
                self.assertIsNone(
                    dynamics.get("damping"),
                    "Actuator-derived Kd should NOT be in <dynamics damping>",
                )

    async def test_mjc_fallback_priority(self) -> None:
        """Verify urdf: attrs take priority over mjc: attrs."""
        from pxr import Sdf

        stage = self._build_mjc_stage()
        jp = stage.GetPrimAtPath("/robot/base/child/joint")
        jp.CreateAttribute("urdf:dynamics:damping", Sdf.ValueTypeNames.Float).Set(99.0)
        jp.CreateAttribute("urdf:dynamics:friction", Sdf.ValueTypeNames.Float).Set(88.0)
        jp.CreateAttribute("urdf:limit:effort", Sdf.ValueTypeNames.Float).Set(77.0)

        with tempfile.TemporaryDirectory(ignore_cleanup_errors=True) as temp_dir:
            root = self._export_mjc_stage(stage, temp_dir)

            revolute_joints = [j for j in root.findall("joint") if j.get("type") == "revolute"]
            dynamics = revolute_joints[0].find("dynamics")
            self.assertAlmostEqual(float(dynamics.get("damping")), 99.0, places=4)
            self.assertAlmostEqual(float(dynamics.get("friction")), 88.0, places=4)

            limit = revolute_joints[0].find("limit")
            self.assertAlmostEqual(float(limit.get("effort")), 77.0, places=4)

    # ------------------------------------------------------------------
    # Multi-DOF joint round-trip tests
    # ------------------------------------------------------------------

    @staticmethod
    def _build_multi_dof_stage() -> Usd.Stage:
        """Create a stage with a SphericalJoint and a D6Joint for round-trip testing.

        Returns:
            Generated in-memory USD stage.
        """
        from pxr import Gf, Sdf, UsdGeom

        stage = Usd.Stage.CreateInMemory()
        UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
        UsdGeom.SetStageMetersPerUnit(stage, 1.0)

        robot = UsdGeom.Xform.Define(stage, "/robot")
        stage.SetDefaultPrim(robot.GetPrim())
        UsdPhysics.ArticulationRootAPI.Apply(robot.GetPrim())

        link_a = UsdGeom.Xform.Define(stage, "/robot/link_a")
        UsdPhysics.RigidBodyAPI.Apply(link_a.GetPrim())
        UsdPhysics.MassAPI.Apply(link_a.GetPrim()).CreateMassAttr().Set(1.0)

        link_b = UsdGeom.Xform.Define(stage, "/robot/link_b")
        UsdPhysics.RigidBodyAPI.Apply(link_b.GetPrim())
        UsdPhysics.MassAPI.Apply(link_b.GetPrim()).CreateMassAttr().Set(0.5)

        link_c = UsdGeom.Xform.Define(stage, "/robot/link_c")
        UsdPhysics.RigidBodyAPI.Apply(link_c.GetPrim())
        UsdPhysics.MassAPI.Apply(link_c.GetPrim()).CreateMassAttr().Set(0.3)

        world_joint = UsdPhysics.FixedJoint.Define(stage, "/robot/world_joint")
        world_joint.GetBody1Rel().SetTargets([Sdf.Path("/robot/link_a")])

        # SphericalJoint: link_a -> link_b
        sph = UsdPhysics.SphericalJoint.Define(stage, "/robot/spherical_hip")
        sph.CreateBody0Rel().SetTargets([Sdf.Path("/robot/link_a")])
        sph.CreateBody1Rel().SetTargets([Sdf.Path("/robot/link_b")])
        sph.CreateLocalPos0Attr().Set(Gf.Vec3f(0, 0, 0.5))
        sph.CreateLocalRot0Attr().Set(Gf.Quatf(1, 0, 0, 0))

        # D6Joint: link_b -> link_c (2 free axes: rotX, transZ)
        d6 = UsdPhysics.Joint.Define(stage, "/robot/d6_shoulder")
        d6.CreateBody0Rel().SetTargets([Sdf.Path("/robot/link_b")])
        d6.CreateBody1Rel().SetTargets([Sdf.Path("/robot/link_c")])
        d6.CreateLocalPos0Attr().Set(Gf.Vec3f(0, 0, 0.3))

        d6_prim = d6.GetPrim()
        lim_rx = UsdPhysics.LimitAPI.Apply(d6_prim, "rotX")
        lim_rx.CreateLowAttr().Set(-45.0)
        lim_rx.CreateHighAttr().Set(45.0)

        lim_tz = UsdPhysics.LimitAPI.Apply(d6_prim, "transZ")
        lim_tz.CreateLowAttr().Set(0.0)
        lim_tz.CreateHighAttr().Set(0.5)

        drv_rx = UsdPhysics.DriveAPI.Apply(d6_prim, "rotX")
        drv_rx.CreateDampingAttr().Set(10.0)
        drv_rx.CreateStiffnessAttr().Set(100.0)

        UsdGeom.Scope.Define(stage, "/robot/Physics")
        return stage

    async def test_multi_dof_joint_export(self) -> None:
        """Verify multi-DOF joints export as chained single-DOF joints with breadcrumbs."""
        stage = self._build_multi_dof_stage()

        with tempfile.TemporaryDirectory(ignore_cleanup_errors=True) as temp_dir:
            output_path, root = await self._export_to_urdf(stage, temp_dir)

            joints = root.findall("joint")
            links = root.findall("link")

            joint_names = [j.get("name") for j in joints]
            link_names = [l.get("name") for l in links]

            # SphericalJoint -> 3 chain joints + 2 ghost links
            self.assertIn("spherical_hip_rotX", joint_names)
            self.assertIn("spherical_hip_rotY", joint_names)
            self.assertIn("spherical_hip_rotZ", joint_names)
            self.assertIn("spherical_hip_ghost_1", link_names)
            self.assertIn("spherical_hip_ghost_2", link_names)

            # D6Joint -> 2 chain joints + 1 ghost link
            self.assertIn("d6_shoulder_rotX", joint_names)
            self.assertIn("d6_shoulder_transZ", joint_names)
            self.assertIn("d6_shoulder_ghost_1", link_names)

            # Verify joint types
            joint_map = {j.get("name"): j for j in joints}
            for name in ("spherical_hip_rotX", "spherical_hip_rotY", "spherical_hip_rotZ"):
                self.assertEqual(joint_map[name].get("type"), "revolute")
            self.assertEqual(joint_map["d6_shoulder_rotX"].get("type"), "revolute")
            self.assertEqual(joint_map["d6_shoulder_transZ"].get("type"), "prismatic")

            # Verify breadcrumb exists on first chain joint
            with open(output_path) as f:
                urdf_text = f.read()
            self.assertIn("isaac:source_joint", urdf_text)
            self.assertIn("PhysicsSphericalJoint", urdf_text)
            self.assertIn("PhysicsJoint", urdf_text)

    # ------------------------------------------------------------------
    # Source drive breadcrumb tests
    # ------------------------------------------------------------------

    @staticmethod
    def _build_drive_stage(max_force: float | None = 50.0) -> Usd.Stage:
        """Create a stage with a RevoluteJoint that has DriveAPI and armature.

        Args:
            max_force: Drive maximum force to author, or None to leave it unauthored.

        Returns:
            Generated in-memory USD stage.
        """
        from pxr import PhysxSchema, Sdf, UsdGeom

        stage = Usd.Stage.CreateInMemory()
        UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
        UsdGeom.SetStageMetersPerUnit(stage, 1.0)

        robot = UsdGeom.Xform.Define(stage, "/robot")
        stage.SetDefaultPrim(robot.GetPrim())
        UsdPhysics.ArticulationRootAPI.Apply(robot.GetPrim())

        base = UsdGeom.Xform.Define(stage, "/robot/base")
        UsdPhysics.RigidBodyAPI.Apply(base.GetPrim())
        UsdPhysics.MassAPI.Apply(base.GetPrim()).CreateMassAttr().Set(1.0)

        child = UsdGeom.Xform.Define(stage, "/robot/child")
        UsdPhysics.RigidBodyAPI.Apply(child.GetPrim())
        UsdPhysics.MassAPI.Apply(child.GetPrim()).CreateMassAttr().Set(0.5)

        world_joint = UsdPhysics.FixedJoint.Define(stage, "/robot/world_joint")
        world_joint.GetBody1Rel().SetTargets([Sdf.Path("/robot/base")])

        joint = UsdPhysics.RevoluteJoint.Define(stage, "/robot/joint1")
        joint.CreateBody0Rel().SetTargets([Sdf.Path("/robot/base")])
        joint.CreateBody1Rel().SetTargets([Sdf.Path("/robot/child")])
        joint.GetAxisAttr().Set("Z")
        joint.GetLowerLimitAttr().Set(-90.0)
        joint.GetUpperLimitAttr().Set(90.0)

        jp = joint.GetPrim()
        drv = UsdPhysics.DriveAPI.Apply(jp, "angular")
        drv.CreateStiffnessAttr().Set(1000.0)
        drv.CreateDampingAttr().Set(100.0)
        if max_force is not None:
            drv.CreateMaxForceAttr().Set(max_force)
        drv.CreateTargetPositionAttr().Set(0.0)

        physx_api = PhysxSchema.PhysxJointAPI.Apply(jp)
        physx_api.CreateArmatureAttr().Set(0.01)

        UsdGeom.Scope.Define(stage, "/robot/Physics")
        return stage

    async def test_drive_breadcrumb_physx(self) -> None:
        """DriveAPI gains and armature are captured in isaac:source_drive breadcrumb."""
        import json

        stage = self._build_drive_stage()

        with tempfile.TemporaryDirectory(ignore_cleanup_errors=True) as temp_dir:
            output_path, root = await self._export_to_urdf(stage, temp_dir)

            with open(output_path) as f:
                urdf_text = f.read()

            self.assertIn("isaac:source_drive", urdf_text)

            prefix = " isaac:source_drive "
            for line in urdf_text.splitlines():
                if prefix in line:
                    json_str = line.split(prefix, 1)[1].rstrip(" ->\n")
                    if json_str.endswith(" "):
                        json_str = json_str.rstrip()
                    meta = json.loads(json_str)
                    self.assertEqual(meta["source"], "physx")
                    self.assertEqual(meta["instance"], "angular")
                    self.assertAlmostEqual(meta["drive"]["stiffness"], 1000.0)
                    self.assertAlmostEqual(meta["drive"]["damping"], 100.0)
                    self.assertAlmostEqual(meta["drive"]["max_force"], 50.0)
                    self.assertAlmostEqual(meta["armature"], 0.01)
                    break
            else:
                self.fail("No isaac:source_drive breadcrumb found")

    async def test_drive_does_not_leak_into_dynamics(self) -> None:
        """DriveAPI.damping must NOT appear in <dynamics damping>."""
        stage = self._build_drive_stage()

        with tempfile.TemporaryDirectory(ignore_cleanup_errors=True) as temp_dir:
            _, root = await self._export_to_urdf(stage, temp_dir)

            joints = [j for j in root.findall("joint") if j.get("type") == "revolute"]
            self.assertEqual(len(joints), 1)
            dynamics = joints[0].find("dynamics")
            if dynamics is not None:
                self.assertIsNone(
                    dynamics.get("damping"),
                    "DriveAPI.damping should NOT appear in <dynamics damping>",
                )

    async def test_drive_effort_limit_fallback(self) -> None:
        """Use native PhysX DriveAPI maxForce as the URDF effort limit."""
        stage = self._build_drive_stage()

        with tempfile.TemporaryDirectory(ignore_cleanup_errors=True) as temp_dir:
            _, root = await self._export_to_urdf(stage, temp_dir)

            joints = [j for j in root.findall("joint") if j.get("type") == "revolute"]
            limit = joints[0].find("limit")
            self.assertIsNotNone(limit)
            self.assertAlmostEqual(float(limit.get("effort")), 50.0, places=4)

    async def test_drive_effort_limit_takes_priority_over_urdf_attr(self) -> None:
        """Prefer DriveAPI maxForce when a legacy URDF effort attribute is also authored."""
        from pxr import Sdf

        stage = self._build_drive_stage()
        joint_prim = stage.GetPrimAtPath("/robot/joint1")
        joint_prim.CreateAttribute("urdf:limit:effort", Sdf.ValueTypeNames.Float).Set(75.0)

        with tempfile.TemporaryDirectory(ignore_cleanup_errors=True) as temp_dir:
            _, root = await self._export_to_urdf(stage, temp_dir)

            joints = [j for j in root.findall("joint") if j.get("type") == "revolute"]
            limit = joints[0].find("limit")
            self.assertIsNotNone(limit)
            self.assertAlmostEqual(float(limit.get("effort")), 50.0, places=4)

    async def test_drive_effort_limit_ignores_unauthored_fallback(self) -> None:
        """Do not export the infinite DriveAPI schema fallback as a URDF effort limit."""
        stage = self._build_drive_stage(max_force=None)

        with tempfile.TemporaryDirectory(ignore_cleanup_errors=True) as temp_dir:
            _, root = await self._export_to_urdf(stage, temp_dir)

            joints = [j for j in root.findall("joint") if j.get("type") == "revolute"]
            limit = joints[0].find("limit")
            self.assertIsNotNone(limit)
            self.assertIsNone(limit.get("effort"))

    async def test_drive_effort_limit_ignores_authored_infinity(self) -> None:
        """Do not export an infinite DriveAPI maxForce as a URDF effort limit."""
        stage = self._build_drive_stage(max_force=math.inf)

        with tempfile.TemporaryDirectory(ignore_cleanup_errors=True) as temp_dir:
            _, root = await self._export_to_urdf(stage, temp_dir)

            joints = [j for j in root.findall("joint") if j.get("type") == "revolute"]
            limit = joints[0].find("limit")
            self.assertIsNotNone(limit)
            self.assertIsNone(limit.get("effort"))

    async def test_drive_effort_limit_falls_back_to_legacy_urdf_attr(self) -> None:
        """Use a finite legacy URDF effort limit when DriveAPI maxForce is not authored."""
        from pxr import Sdf

        stage = self._build_drive_stage(max_force=None)
        joint_prim = stage.GetPrimAtPath("/robot/joint1")
        joint_prim.CreateAttribute("urdf:limit:effort", Sdf.ValueTypeNames.Float).Set(75.0)

        with tempfile.TemporaryDirectory(ignore_cleanup_errors=True) as temp_dir:
            _, root = await self._export_to_urdf(stage, temp_dir)

            joints = [j for j in root.findall("joint") if j.get("type") == "revolute"]
            limit = joints[0].find("limit")
            self.assertIsNotNone(limit)
            self.assertAlmostEqual(float(limit.get("effort")), 75.0, places=4)

    async def test_drive_does_not_leak_into_calibration(self) -> None:
        """DriveAPI.targetPosition must NOT appear in <calibration>."""
        stage = self._build_drive_stage()

        with tempfile.TemporaryDirectory(ignore_cleanup_errors=True) as temp_dir:
            _, root = await self._export_to_urdf(stage, temp_dir)

            joints = [j for j in root.findall("joint") if j.get("type") == "revolute"]
            calibration = joints[0].find("calibration")
            self.assertIsNone(calibration, "DriveAPI targetPosition should NOT produce <calibration>")

    async def test_passive_sources_still_work(self) -> None:
        """urdf:dynamics:damping populates <dynamics> while DriveAPI goes to breadcrumb."""
        from pxr import Sdf

        stage = self._build_drive_stage()
        jp = stage.GetPrimAtPath("/robot/joint1")
        jp.CreateAttribute("urdf:dynamics:damping", Sdf.ValueTypeNames.Float).Set(5.0)

        with tempfile.TemporaryDirectory(ignore_cleanup_errors=True) as temp_dir:
            _, root = await self._export_to_urdf(stage, temp_dir)

            joints = [j for j in root.findall("joint") if j.get("type") == "revolute"]
            dynamics = joints[0].find("dynamics")
            self.assertIsNotNone(dynamics)
            self.assertAlmostEqual(float(dynamics.get("damping")), 5.0, places=4)

    async def test_drive_breadcrumb_mujoco(self) -> None:
        """MjcActuator attributes are captured in breadcrumb with source=mujoco."""
        import json

        stage = self._build_mjc_stage()

        with tempfile.TemporaryDirectory(ignore_cleanup_errors=True) as temp_dir:
            output_path, root = await self._export_to_urdf(stage, temp_dir)

            with open(output_path) as f:
                urdf_text = f.read()

            self.assertIn("isaac:source_drive", urdf_text)

            prefix = " isaac:source_drive "
            for line in urdf_text.splitlines():
                if prefix in line:
                    json_str = line.split(prefix, 1)[1].rstrip(" ->\n").rstrip()
                    meta = json.loads(json_str)
                    self.assertEqual(meta["source"], "mujoco")
                    act = meta["actuator"]
                    self.assertEqual(act["gainPrm"][0], 100.0)
                    self.assertEqual(act["biasPrm"][1], -100.0)
                    self.assertEqual(act["gainType"], "fixed")
                    self.assertEqual(act["biasType"], "affine")
                    break
            else:
                self.fail("No isaac:source_drive breadcrumb found")

    async def test_drive_breadcrumb_mujoco_precedence(self) -> None:
        """When both DriveAPI and MjcActuator exist, breadcrumb uses mujoco."""
        import json

        stage = self._build_mjc_stage()
        jp = stage.GetPrimAtPath("/robot/base/child/joint")
        drv = UsdPhysics.DriveAPI.Apply(jp, "angular")
        drv.CreateStiffnessAttr().Set(999.0)
        drv.CreateDampingAttr().Set(888.0)

        with tempfile.TemporaryDirectory(ignore_cleanup_errors=True) as temp_dir:
            output_path, _ = await self._export_to_urdf(stage, temp_dir)

            with open(output_path) as f:
                urdf_text = f.read()

            prefix = " isaac:source_drive "
            for line in urdf_text.splitlines():
                if prefix in line:
                    json_str = line.split(prefix, 1)[1].rstrip(" ->\n").rstrip()
                    meta = json.loads(json_str)
                    self.assertEqual(meta["source"], "mujoco", "MuJoCo actuator should take precedence")
                    self.assertNotIn("drive", meta)
                    break

    async def test_drive_breadcrumb_armature_only(self) -> None:
        """Joint with only armature authored gets a breadcrumb with just armature."""
        import json

        from pxr import PhysxSchema, Sdf, UsdGeom

        stage = Usd.Stage.CreateInMemory()
        UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
        UsdGeom.SetStageMetersPerUnit(stage, 1.0)

        robot = UsdGeom.Xform.Define(stage, "/robot")
        stage.SetDefaultPrim(robot.GetPrim())
        UsdPhysics.ArticulationRootAPI.Apply(robot.GetPrim())

        base = UsdGeom.Xform.Define(stage, "/robot/base")
        UsdPhysics.RigidBodyAPI.Apply(base.GetPrim())
        UsdPhysics.MassAPI.Apply(base.GetPrim()).CreateMassAttr().Set(1.0)

        child = UsdGeom.Xform.Define(stage, "/robot/child")
        UsdPhysics.RigidBodyAPI.Apply(child.GetPrim())
        UsdPhysics.MassAPI.Apply(child.GetPrim()).CreateMassAttr().Set(0.5)

        world_joint = UsdPhysics.FixedJoint.Define(stage, "/robot/world_joint")
        world_joint.GetBody1Rel().SetTargets([Sdf.Path("/robot/base")])

        joint = UsdPhysics.RevoluteJoint.Define(stage, "/robot/arm_joint")
        joint.CreateBody0Rel().SetTargets([Sdf.Path("/robot/base")])
        joint.CreateBody1Rel().SetTargets([Sdf.Path("/robot/child")])
        joint.GetAxisAttr().Set("Z")
        joint.GetLowerLimitAttr().Set(-90.0)
        joint.GetUpperLimitAttr().Set(90.0)

        PhysxSchema.PhysxJointAPI.Apply(joint.GetPrim()).CreateArmatureAttr().Set(0.05)

        UsdGeom.Scope.Define(stage, "/robot/Physics")

        with tempfile.TemporaryDirectory(ignore_cleanup_errors=True) as temp_dir:
            output_path, _ = await self._export_to_urdf(stage, temp_dir)

            with open(output_path) as f:
                urdf_text = f.read()

            self.assertIn("isaac:source_drive", urdf_text)

            prefix = " isaac:source_drive "
            for line in urdf_text.splitlines():
                if prefix in line:
                    json_str = line.split(prefix, 1)[1].rstrip(" ->\n").rstrip()
                    meta = json.loads(json_str)
                    self.assertAlmostEqual(meta["armature"], 0.05)
                    break

    async def test_no_drive_breadcrumb_when_nothing_authored(self) -> None:
        """Plain RevoluteJoint with no drive/actuator/armature gets no breadcrumb."""
        from pxr import Sdf, UsdGeom

        stage = Usd.Stage.CreateInMemory()
        UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
        UsdGeom.SetStageMetersPerUnit(stage, 1.0)

        robot = UsdGeom.Xform.Define(stage, "/robot")
        stage.SetDefaultPrim(robot.GetPrim())
        UsdPhysics.ArticulationRootAPI.Apply(robot.GetPrim())

        base = UsdGeom.Xform.Define(stage, "/robot/base")
        UsdPhysics.RigidBodyAPI.Apply(base.GetPrim())
        UsdPhysics.MassAPI.Apply(base.GetPrim()).CreateMassAttr().Set(1.0)

        child = UsdGeom.Xform.Define(stage, "/robot/child")
        UsdPhysics.RigidBodyAPI.Apply(child.GetPrim())
        UsdPhysics.MassAPI.Apply(child.GetPrim()).CreateMassAttr().Set(0.5)

        world_joint = UsdPhysics.FixedJoint.Define(stage, "/robot/world_joint")
        world_joint.GetBody1Rel().SetTargets([Sdf.Path("/robot/base")])

        joint = UsdPhysics.RevoluteJoint.Define(stage, "/robot/plain_joint")
        joint.CreateBody0Rel().SetTargets([Sdf.Path("/robot/base")])
        joint.CreateBody1Rel().SetTargets([Sdf.Path("/robot/child")])
        joint.GetAxisAttr().Set("Z")
        joint.GetLowerLimitAttr().Set(-90.0)
        joint.GetUpperLimitAttr().Set(90.0)

        UsdGeom.Scope.Define(stage, "/robot/Physics")

        with tempfile.TemporaryDirectory(ignore_cleanup_errors=True) as temp_dir:
            output_path, _ = await self._export_to_urdf(stage, temp_dir)

            with open(output_path) as f:
                urdf_text = f.read()

            self.assertNotIn("isaac:source_drive", urdf_text)

    # ------------------------------------------------------------------
    # Variant selection tests
    # ------------------------------------------------------------------

    @staticmethod
    def _build_variant_stage() -> Usd.Stage:
        """Create a stage with an ``EndEffector`` variant set that swaps a child link.

        Variant ``gripper``  -> adds ``gripper_link`` + ``gripper_joint``
        Variant ``suction``  -> adds ``suction_cup``  + ``suction_joint``

        The variant selection is left empty so the caller can set it via
        ``variant_selections``.

        Returns:
            Generated in-memory USD stage.
        """
        from pxr import Sdf, UsdGeom

        stage = Usd.Stage.CreateInMemory()
        UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
        UsdGeom.SetStageMetersPerUnit(stage, 1.0)

        robot = UsdGeom.Xform.Define(stage, "/robot")
        stage.SetDefaultPrim(robot.GetPrim())
        UsdPhysics.ArticulationRootAPI.Apply(robot.GetPrim())

        base = UsdGeom.Xform.Define(stage, "/robot/base")
        UsdPhysics.RigidBodyAPI.Apply(base.GetPrim())
        UsdPhysics.MassAPI.Apply(base.GetPrim()).CreateMassAttr().Set(1.0)

        world_joint = UsdPhysics.FixedJoint.Define(stage, "/robot/world_joint")
        world_joint.GetBody1Rel().SetTargets([Sdf.Path("/robot/base")])

        root_prim = robot.GetPrim()
        vset = root_prim.GetVariantSets().AddVariantSet("EndEffector")
        vset.AddVariant("gripper")
        vset.AddVariant("suction")

        vset.SetVariantSelection("gripper")
        with vset.GetVariantEditContext():
            effector = UsdGeom.Xform.Define(stage, "/robot/gripper_link")
            UsdPhysics.RigidBodyAPI.Apply(effector.GetPrim())
            UsdPhysics.MassAPI.Apply(effector.GetPrim()).CreateMassAttr().Set(0.3)
            jnt = UsdPhysics.FixedJoint.Define(stage, "/robot/gripper_joint")
            jnt.CreateBody0Rel().SetTargets([Sdf.Path("/robot/base")])
            jnt.CreateBody1Rel().SetTargets([Sdf.Path("/robot/gripper_link")])

        vset.SetVariantSelection("suction")
        with vset.GetVariantEditContext():
            effector = UsdGeom.Xform.Define(stage, "/robot/suction_cup")
            UsdPhysics.RigidBodyAPI.Apply(effector.GetPrim())
            UsdPhysics.MassAPI.Apply(effector.GetPrim()).CreateMassAttr().Set(0.2)
            jnt = UsdPhysics.FixedJoint.Define(stage, "/robot/suction_joint")
            jnt.CreateBody0Rel().SetTargets([Sdf.Path("/robot/base")])
            jnt.CreateBody1Rel().SetTargets([Sdf.Path("/robot/suction_cup")])

        vset.SetVariantSelection("")

        UsdGeom.Scope.Define(stage, "/robot/Physics")
        return stage

    async def test_variant_selections_gripper(self) -> None:
        """Selecting the 'gripper' variant produces a URDF with the gripper link."""
        stage = self._build_variant_stage()

        with tempfile.TemporaryDirectory(ignore_cleanup_errors=True) as temp_dir:
            _, root = await self._export_to_urdf(stage, temp_dir, variant_selections={"EndEffector": "gripper"})
            link_names = {link.get("name") for link in root.findall("link")}
            self.assertIn("gripper_link", link_names)
            self.assertNotIn("suction_cup", link_names)

    async def test_variant_selections_suction(self) -> None:
        """Selecting the 'suction' variant produces a URDF with the suction cup link."""
        stage = self._build_variant_stage()

        with tempfile.TemporaryDirectory(ignore_cleanup_errors=True) as temp_dir:
            _, root = await self._export_to_urdf(stage, temp_dir, variant_selections={"EndEffector": "suction"})
            link_names = {link.get("name") for link in root.findall("link")}
            self.assertIn("suction_cup", link_names)
            self.assertNotIn("gripper_link", link_names)

    @staticmethod
    def _build_scaled_primitive_stage() -> Usd.Stage:
        """Create primitives with scale authored on the robot, parent, and primitive.

        Returns:
            Generated stage containing scaled primitives.
        """
        from pxr import Gf, Sdf, UsdGeom

        stage = Usd.Stage.CreateInMemory()
        UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
        UsdGeom.SetStageMetersPerUnit(stage, 1.0)

        robot = UsdGeom.Xform.Define(stage, "/robot")
        stage.SetDefaultPrim(robot.GetPrim())
        UsdPhysics.ArticulationRootAPI.Apply(robot.GetPrim())
        robot.AddScaleOp().Set(Gf.Vec3f(2.0, 2.0, 2.0))

        base = UsdGeom.Xform.Define(stage, "/robot/base")
        UsdPhysics.RigidBodyAPI.Apply(base.GetPrim())
        UsdPhysics.MassAPI.Apply(base.GetPrim()).CreateMassAttr().Set(1.0)

        world_joint = UsdPhysics.FixedJoint.Define(stage, "/robot/world_joint")
        world_joint.GetBody1Rel().SetTargets([Sdf.Path("/robot/base")])

        geometry = UsdGeom.Xform.Define(stage, "/robot/base/geometry")
        geometry.AddScaleOp().Set(Gf.Vec3f(3.0, 4.0, 5.0))

        cube = UsdGeom.Cube.Define(stage, "/robot/base/geometry/cube")
        cube.CreateSizeAttr().Set(1.0)
        cube.AddScaleOp().Set(Gf.Vec3f(0.5, 0.25, 0.1))

        sphere = UsdGeom.Sphere.Define(stage, "/robot/base/geometry/sphere")
        sphere.CreateRadiusAttr().Set(0.5)

        cylinder_x = UsdGeom.Cylinder.Define(stage, "/robot/base/geometry/cylinder_x")
        cylinder_x.CreateRadiusAttr().Set(0.5)
        cylinder_x.CreateHeightAttr().Set(2.0)
        cylinder_x.CreateAxisAttr().Set(UsdGeom.Tokens.x)

        cylinder_y = UsdGeom.Cylinder.Define(stage, "/robot/base/geometry/cylinder_y")
        cylinder_y.CreateRadiusAttr().Set(0.25)
        cylinder_y.CreateHeightAttr().Set(1.0)
        cylinder_y.CreateAxisAttr().Set(UsdGeom.Tokens.y)

        capsule_x = UsdGeom.Capsule.Define(stage, "/robot/base/geometry/capsule_x")
        capsule_x.CreateRadiusAttr().Set(0.2)
        capsule_x.CreateHeightAttr().Set(1.0)
        capsule_x.CreateAxisAttr().Set(UsdGeom.Tokens.x)

        cone_y = UsdGeom.Cone.Define(stage, "/robot/base/geometry/cone_y")
        cone_y.CreateRadiusAttr().Set(0.1)
        cone_y.CreateHeightAttr().Set(0.5)
        cone_y.CreateAxisAttr().Set(UsdGeom.Tokens.y)

        UsdGeom.Scope.Define(stage, "/robot/Physics")
        return stage

    async def test_parent_and_primitive_scale_applied_to_primitive_dimensions(self) -> None:
        """Composed parent and local scale must reach exported primitive dimensions."""
        stage = self._build_scaled_primitive_stage()

        with tempfile.TemporaryDirectory(ignore_cleanup_errors=True) as temp_dir:
            output_path, root = await self._export_to_urdf(stage, temp_dir)

            cube = root.find("./link/visual[@name='cube']/geometry/box")
            self.assertIsNotNone(cube)
            for actual, expected in zip(self._parse_xyz(cube.get("size", "")), (3.0, 2.0, 1.0)):
                self.assertAlmostEqual(actual, expected, places=6)

            sphere = root.find("./link/visual[@name='sphere']/geometry/sphere")
            self.assertIsNotNone(sphere)
            self.assertAlmostEqual(float(sphere.get("radius")), 5.0)

            cylinder_x = root.find("./link/visual[@name='cylinder_x']/geometry/cylinder")
            self.assertIsNotNone(cylinder_x)
            self.assertAlmostEqual(float(cylinder_x.get("radius")), 5.0)
            self.assertAlmostEqual(float(cylinder_x.get("length")), 12.0)

            capsule_body = root.find("./link/visual[@name='capsule_x_body']/geometry/cylinder")
            self.assertIsNotNone(capsule_body)
            self.assertAlmostEqual(float(capsule_body.get("radius")), 2.0)
            self.assertAlmostEqual(float(capsule_body.get("length")), 6.0)

            capsule_top = root.find("./link/visual[@name='capsule_x_top_cap']")
            capsule_bottom = root.find("./link/visual[@name='capsule_x_bottom_cap']")
            self.assertIsNotNone(capsule_top)
            self.assertIsNotNone(capsule_bottom)
            self.assertAlmostEqual(self._parse_xyz(capsule_top.find("origin").get("xyz"))[0], 3.0)
            self.assertAlmostEqual(self._parse_xyz(capsule_bottom.find("origin").get("xyz"))[0], -3.0)

            cone = root.find("./link/visual[@name='cone_y']/geometry/mesh")
            self.assertIsNotNone(cone)
            cone_path = os.path.join(os.path.dirname(output_path), cone.get("filename", "").lstrip("./"))
            vertices = []
            with open(cone_path) as cone_file:
                for line in cone_file:
                    if line.startswith("v "):
                        vertices.append(tuple(float(value) for value in line.split()[1:4]))
            self.assertTrue(vertices)
            self.assertAlmostEqual(max(abs(vertex[1]) for vertex in vertices), 2.0, places=6)
            self.assertAlmostEqual(
                max(max(abs(vertex[0]), abs(vertex[2])) for vertex in vertices),
                1.0,
                places=6,
            )

    async def test_cylinder_and_capsule_axis_rotations_exported(self) -> None:
        """USD X/Y primitive axes must be composed into URDF geometry origins."""
        stage = self._build_scaled_primitive_stage()

        with tempfile.TemporaryDirectory(ignore_cleanup_errors=True) as temp_dir:
            _, root = await self._export_to_urdf(stage, temp_dir)

            cylinder_x = root.find("./link/visual[@name='cylinder_x']/origin")
            cylinder_y = root.find("./link/visual[@name='cylinder_y']/origin")
            capsule_body = root.find("./link/visual[@name='capsule_x_body']/origin")
            self.assertIsNotNone(cylinder_x)
            self.assertIsNotNone(cylinder_y)
            self.assertIsNotNone(capsule_body)

            cylinder_x_rpy = self._parse_xyz(cylinder_x.get("rpy", ""))
            cylinder_y_rpy = self._parse_xyz(cylinder_y.get("rpy", ""))
            capsule_body_rpy = self._parse_xyz(capsule_body.get("rpy", ""))
            self.assertAlmostEqual(cylinder_x_rpy[1], math.pi / 2.0, places=6)
            self.assertAlmostEqual(cylinder_y_rpy[0], -math.pi / 2.0, places=6)
            self.assertAlmostEqual(capsule_body_rpy[1], math.pi / 2.0, places=6)

    # ------------------------------------------------------------------
    # Mesh scale on instanceable geometry (regression)
    # ------------------------------------------------------------------

    @staticmethod
    def _build_instanced_mesh_scale_stage(geometry_scale: tuple[float, float, float]) -> Usd.Stage:
        """Create a single-link articulation with an instance-proxy scaled geometry.

        Layout mirrors Isaac Sim collected assets (e.g. the franka.usd
        delivered with collected props), where each rigid-body link
        contains an ``Xform "geometry"`` child marked ``instanceable=true``
        that references a prototype ``Xform`` containing the Mesh. The
        scale is authored on the link-side Xform (so it is unique per
        instance) while the Mesh in the prototype has no local scale.

        The reference target lives in the same stage as a hidden class
        ``/_Proto/geom`` so the test stays self-contained.

        Args:
            geometry_scale: Scale to author on the instanceable geometry.

        Returns:
            Generated in-memory USD stage.
        """
        from pxr import Gf, Sdf, UsdGeom

        stage = Usd.Stage.CreateInMemory()
        UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
        UsdGeom.SetStageMetersPerUnit(stage, 1.0)

        # Hidden prototype that holds the actual mesh data. Defined as a
        # class so it does not appear as a renderable scene root.
        proto_root = stage.CreateClassPrim("/_Proto")
        proto_geom = UsdGeom.Xform.Define(stage, "/_Proto/geom")
        proto_mesh = UsdGeom.Mesh.Define(stage, "/_Proto/geom/mesh")
        # 1m unit cube around origin; the per-instance scale shrinks it.
        proto_mesh.CreatePointsAttr().Set(
            [
                Gf.Vec3f(-0.5, -0.5, -0.5),
                Gf.Vec3f(0.5, -0.5, -0.5),
                Gf.Vec3f(0.5, 0.5, -0.5),
                Gf.Vec3f(-0.5, 0.5, -0.5),
                Gf.Vec3f(-0.5, -0.5, 0.5),
                Gf.Vec3f(0.5, -0.5, 0.5),
                Gf.Vec3f(0.5, 0.5, 0.5),
                Gf.Vec3f(-0.5, 0.5, 0.5),
            ]
        )
        proto_mesh.CreateFaceVertexCountsAttr().Set([4, 4, 4, 4, 4, 4])
        proto_mesh.CreateFaceVertexIndicesAttr().Set(
            [0, 3, 2, 1, 4, 5, 6, 7, 0, 1, 5, 4, 2, 3, 7, 6, 1, 2, 6, 5, 0, 4, 7, 3]
        )

        robot = UsdGeom.Xform.Define(stage, "/robot")
        stage.SetDefaultPrim(robot.GetPrim())
        UsdPhysics.ArticulationRootAPI.Apply(robot.GetPrim())

        base = UsdGeom.Xform.Define(stage, "/robot/base")
        UsdPhysics.RigidBodyAPI.Apply(base.GetPrim())
        UsdPhysics.MassAPI.Apply(base.GetPrim()).CreateMassAttr().Set(1.0)

        world_joint = UsdPhysics.FixedJoint.Define(stage, "/robot/world_joint")
        world_joint.GetBody1Rel().SetTargets([Sdf.Path("/robot/base")])

        # Per-link geometry container: references the prototype, carries
        # the per-instance scale, and is flagged instanceable so USD
        # promotes the Mesh to an instance proxy at composition time.
        geom_xf = UsdGeom.Xform.Define(stage, "/robot/base/geometry")
        geom_xf.GetPrim().GetReferences().AddInternalReference(proto_geom.GetPath())
        geom_xf.AddScaleOp().Set(Gf.Vec3f(*geometry_scale))
        geom_xf.GetPrim().SetInstanceable(True)

        UsdGeom.Scope.Define(stage, "/robot/Physics")
        return stage

    async def test_instanced_geometry_scale_emitted_on_mesh(self) -> None:
        """Scale on an instanceable parent xform must reach <mesh scale=...>.

        Regression test for the franka.usd case where ``/robot/<link>/geometry``
        is an instanceable Xform with a 0.01 scale and the Mesh inside it has
        none. Without the fix, ``mesh_scale`` was always set to ``None`` for
        instance-proxy meshes, silently dropping the asset's scale.
        """
        stage = self._build_instanced_mesh_scale_stage((0.01, 0.01, 0.01))

        with tempfile.TemporaryDirectory(ignore_cleanup_errors=True) as temp_dir:
            _, root = await self._export_to_urdf(stage, temp_dir)
            mesh_elems = list(root.iter("mesh"))
            self.assertEqual(len(mesh_elems), 1, "Expected exactly one <mesh> element in URDF")

            scale_attr = mesh_elems[0].get("scale")
            self.assertIsNotNone(
                scale_attr,
                "Mesh scale attribute is missing; instanced scale was dropped during export",
            )
            scale_vals = self._parse_xyz(scale_attr)
            self.assertEqual(len(scale_vals), 3)
            for axis_idx, axis_name in enumerate("xyz"):
                self.assertAlmostEqual(
                    scale_vals[axis_idx],
                    0.01,
                    places=5,
                    msg=f"Mesh scale {axis_name} expected 0.01, got {scale_vals[axis_idx]}",
                )

    async def test_instanced_geometry_unit_scale_omits_attribute(self) -> None:
        """Unit scale must not produce a redundant ``scale="1 1 1"`` attribute."""
        stage = self._build_instanced_mesh_scale_stage((1.0, 1.0, 1.0))

        with tempfile.TemporaryDirectory(ignore_cleanup_errors=True) as temp_dir:
            _, root = await self._export_to_urdf(stage, temp_dir)
            mesh_elems = list(root.iter("mesh"))
            self.assertEqual(len(mesh_elems), 1)
            self.assertIsNone(
                mesh_elems[0].get("scale"),
                "Mesh element should not carry a scale attribute when scale is unit",
            )

    async def test_instanced_geometry_non_uniform_scale(self) -> None:
        """Non-uniform scale on an instanceable xform must round-trip per-axis."""
        stage = self._build_instanced_mesh_scale_stage((0.01, 0.02, 0.03))

        with tempfile.TemporaryDirectory(ignore_cleanup_errors=True) as temp_dir:
            _, root = await self._export_to_urdf(stage, temp_dir)
            mesh_elems = list(root.iter("mesh"))
            self.assertEqual(len(mesh_elems), 1)

            scale_attr = mesh_elems[0].get("scale")
            self.assertIsNotNone(scale_attr)
            scale_vals = self._parse_xyz(scale_attr)
            self.assertAlmostEqual(scale_vals[0], 0.01, places=5)
            self.assertAlmostEqual(scale_vals[1], 0.02, places=5)
            self.assertAlmostEqual(scale_vals[2], 0.03, places=5)

    async def test_non_instanced_scaled_geometry_bakes_into_vertices(self) -> None:
        """Scale on a non-instanced Xform between link and Mesh must bake into OBJ vertices.

        This is the alternative to the instance-proxy path. Vertices in
        the OBJ should reflect ``original_extent * scale`` and the URDF
        ``<mesh>`` element should carry no ``scale`` attribute.
        """
        from pxr import Gf, Sdf, UsdGeom

        scale = 0.01
        stage = Usd.Stage.CreateInMemory()
        UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
        UsdGeom.SetStageMetersPerUnit(stage, 1.0)

        robot = UsdGeom.Xform.Define(stage, "/robot")
        stage.SetDefaultPrim(robot.GetPrim())
        UsdPhysics.ArticulationRootAPI.Apply(robot.GetPrim())

        base = UsdGeom.Xform.Define(stage, "/robot/base")
        UsdPhysics.RigidBodyAPI.Apply(base.GetPrim())
        UsdPhysics.MassAPI.Apply(base.GetPrim()).CreateMassAttr().Set(1.0)

        world_joint = UsdPhysics.FixedJoint.Define(stage, "/robot/world_joint")
        world_joint.GetBody1Rel().SetTargets([Sdf.Path("/robot/base")])

        geom_xf = UsdGeom.Xform.Define(stage, "/robot/base/geometry")
        geom_xf.AddScaleOp().Set(Gf.Vec3f(scale, scale, scale))

        mesh = UsdGeom.Mesh.Define(stage, "/robot/base/geometry/mesh")
        mesh.CreatePointsAttr().Set(
            [
                Gf.Vec3f(-0.5, -0.5, -0.5),
                Gf.Vec3f(0.5, -0.5, -0.5),
                Gf.Vec3f(0.5, 0.5, -0.5),
                Gf.Vec3f(-0.5, 0.5, -0.5),
                Gf.Vec3f(-0.5, -0.5, 0.5),
                Gf.Vec3f(0.5, -0.5, 0.5),
                Gf.Vec3f(0.5, 0.5, 0.5),
                Gf.Vec3f(-0.5, 0.5, 0.5),
            ]
        )
        mesh.CreateFaceVertexCountsAttr().Set([4, 4, 4, 4, 4, 4])
        mesh.CreateFaceVertexIndicesAttr().Set([0, 3, 2, 1, 4, 5, 6, 7, 0, 1, 5, 4, 2, 3, 7, 6, 1, 2, 6, 5, 0, 4, 7, 3])

        UsdGeom.Scope.Define(stage, "/robot/Physics")

        with tempfile.TemporaryDirectory(ignore_cleanup_errors=True) as temp_dir:
            output_path, root = await self._export_to_urdf(stage, temp_dir)

            mesh_elems = list(root.iter("mesh"))
            self.assertEqual(len(mesh_elems), 1)
            self.assertIsNone(
                mesh_elems[0].get("scale"),
                "Non-instanced scaled mesh should bake into vertices, not emit <mesh scale>",
            )

            # Read the OBJ and verify vertex coordinates are within
            # the scaled bound (|coord| <= 0.5 * scale + slack).
            obj_filename = mesh_elems[0].get("filename", "").lstrip("./")
            obj_path = os.path.join(os.path.dirname(output_path), obj_filename)
            self.assertTrue(os.path.exists(obj_path), f"Expected OBJ at {obj_path}")

            max_abs = 0.0
            with open(obj_path) as f:
                for line in f:
                    if not line.startswith("v "):
                        continue
                    parts = line.split()
                    for s in parts[1:4]:
                        max_abs = max(max_abs, abs(float(s)))

            expected = 0.5 * scale
            self.assertLess(
                abs(max_abs - expected),
                1e-5,
                f"OBJ max |vertex| = {max_abs}, expected ~{expected} (vertices not baked with scale)",
            )

    async def test_root_scale_propagates_to_joint_origins_and_meshes(self) -> None:
        """A scale on the robot root prim must reach joint origins and mesh scales.

        The exporter expresses links and geometry in robot-local coordinates by
        multiplying by ``robot_world.GetInverse()``, which cancels translation,
        rotation *and* scale. Removing the root's placement is intended; removing
        its scale is not, so a robot scaled in the viewport used to export at its
        authored (unscaled) size. Both links are checked because the dropped
        scale did not affect every link equally.
        """
        from pxr import Gf, Sdf, UsdGeom

        root_scale = 2.0
        geometry_scale = 0.01
        child_offset = (0.4, 0.0, 0.0)

        stage = Usd.Stage.CreateInMemory()
        UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
        UsdGeom.SetStageMetersPerUnit(stage, 1.0)

        # Hidden prototype holding the mesh data, shared by both links.
        stage.CreateClassPrim("/_Proto")
        proto_geom = UsdGeom.Xform.Define(stage, "/_Proto/geom")
        proto_mesh = UsdGeom.Mesh.Define(stage, "/_Proto/geom/mesh")
        proto_mesh.CreatePointsAttr().Set(
            [
                Gf.Vec3f(-0.5, -0.5, -0.5),
                Gf.Vec3f(0.5, -0.5, -0.5),
                Gf.Vec3f(0.5, 0.5, -0.5),
                Gf.Vec3f(-0.5, 0.5, -0.5),
                Gf.Vec3f(-0.5, -0.5, 0.5),
                Gf.Vec3f(0.5, -0.5, 0.5),
                Gf.Vec3f(0.5, 0.5, 0.5),
                Gf.Vec3f(-0.5, 0.5, 0.5),
            ]
        )
        proto_mesh.CreateFaceVertexCountsAttr().Set([4, 4, 4, 4, 4, 4])
        proto_mesh.CreateFaceVertexIndicesAttr().Set(
            [0, 3, 2, 1, 4, 5, 6, 7, 0, 1, 5, 4, 2, 3, 7, 6, 1, 2, 6, 5, 0, 4, 7, 3]
        )

        # The scale under test lives on the robot root prim itself.
        robot = UsdGeom.Xform.Define(stage, "/robot")
        stage.SetDefaultPrim(robot.GetPrim())
        UsdPhysics.ArticulationRootAPI.Apply(robot.GetPrim())
        robot.AddScaleOp().Set(Gf.Vec3f(root_scale, root_scale, root_scale))

        for link_name, offset in (("base", (0.0, 0.0, 0.0)), ("child", child_offset)):
            link = UsdGeom.Xform.Define(stage, f"/robot/{link_name}")
            if any(offset):
                link.AddTranslateOp().Set(Gf.Vec3f(*offset))
            UsdPhysics.RigidBodyAPI.Apply(link.GetPrim())
            UsdPhysics.MassAPI.Apply(link.GetPrim()).CreateMassAttr().Set(1.0)

            geom_xf = UsdGeom.Xform.Define(stage, f"/robot/{link_name}/geometry")
            geom_xf.GetPrim().GetReferences().AddInternalReference(proto_geom.GetPath())
            geom_xf.AddScaleOp().Set(Gf.Vec3f(geometry_scale, geometry_scale, geometry_scale))
            geom_xf.GetPrim().SetInstanceable(True)

        world_joint = UsdPhysics.FixedJoint.Define(stage, "/robot/world_joint")
        world_joint.GetBody1Rel().SetTargets([Sdf.Path("/robot/base")])

        base_to_child = UsdPhysics.FixedJoint.Define(stage, "/robot/base_to_child")
        base_to_child.CreateBody0Rel().SetTargets([Sdf.Path("/robot/base")])
        base_to_child.CreateBody1Rel().SetTargets([Sdf.Path("/robot/child")])
        base_to_child.CreateLocalPos0Attr().Set(Gf.Vec3f(*child_offset))

        UsdGeom.Scope.Define(stage, "/robot/Physics")

        with tempfile.TemporaryDirectory(ignore_cleanup_errors=True) as temp_dir:
            _, root = await self._export_to_urdf(stage, temp_dir)

            # Joint offsets must be scaled by the root scale.
            joint = root.find("./joint[@name='base_to_child']")
            self.assertIsNotNone(joint, "Expected the base->child joint in the exported URDF")
            origin = joint.find("./origin")
            self.assertIsNotNone(origin)
            joint_xyz = self._parse_xyz(origin.get("xyz", ""))
            for axis_idx, axis_name in enumerate("xyz"):
                self.assertAlmostEqual(
                    joint_xyz[axis_idx],
                    child_offset[axis_idx] * root_scale,
                    places=5,
                    msg=(
                        f"Joint origin {axis_name} expected "
                        f"{child_offset[axis_idx] * root_scale}, got {joint_xyz[axis_idx]}; "
                        "the root scale was dropped from the joint offset"
                    ),
                )

            # Every link's mesh must carry geometry scale * root scale.
            mesh_elems = list(root.iter("mesh"))
            self.assertEqual(len(mesh_elems), 2, "Expected one <mesh> per link")
            expected_mesh_scale = geometry_scale * root_scale
            for mesh_elem in mesh_elems:
                scale_attr = mesh_elem.get("scale")
                self.assertIsNotNone(
                    scale_attr,
                    "Mesh scale attribute is missing; the root scale was dropped for this link",
                )
                mesh_scale_vals = self._parse_xyz(scale_attr)
                for axis_idx, axis_name in enumerate("xyz"):
                    self.assertAlmostEqual(
                        mesh_scale_vals[axis_idx],
                        expected_mesh_scale,
                        places=5,
                        msg=f"Mesh scale {axis_name} expected {expected_mesh_scale}",
                    )

    # ------------------------------------------------------------------
    # NewtonMassAPI newton:inertia tests
    # ------------------------------------------------------------------

    # Real inspire_hand base-link tensor, ordered [Ixx, Iyy, Izz, Ixy, Ixz, Iyz].
    _NEWTON_INERTIA = [
        0.00032283748353732,  # Ixx
        0.000208330666586235,  # Iyy
        0.000184844103234342,  # Izz
        0.00000136447935341448,  # Ixy
        -0.00000195391053679767,  # Ixz
        0.00000117907186759775,  # Iyz
    ]

    @staticmethod
    def _author_mass_prim(
        newton_inertia: list[float] | None = None,
        diagonal: tuple[float, float, float] | None = None,
        principal_axes: tuple[float, float, float, float] | None = None,
        mass: float = 0.5,
        com: tuple[float, float, float] = (0.0, 0.0, 0.0),
    ) -> tuple[Usd.Stage, Usd.Prim]:
        """Create an in-memory prim with MassAPI and optional inertia attrs.

        Args:
            newton_inertia: Value for the NewtonMassAPI ``newton:inertia`` array.
            diagonal: ``physics:diagonalInertia`` value.
            principal_axes: ``physics:principalAxes`` quaternion (w, x, y, z).
            mass: ``physics:mass`` value.
            com: ``physics:centerOfMass`` value.

        Returns:
            (stage, prim). The stage is returned so it stays alive.
        """
        from pxr import Gf, Sdf, UsdGeom

        stage = Usd.Stage.CreateInMemory()
        UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
        UsdGeom.SetStageMetersPerUnit(stage, 1.0)

        prim = UsdGeom.Xform.Define(stage, "/body").GetPrim()
        UsdPhysics.RigidBodyAPI.Apply(prim)
        mass_api = UsdPhysics.MassAPI.Apply(prim)
        mass_api.CreateMassAttr().Set(mass)
        mass_api.CreateCenterOfMassAttr().Set(Gf.Vec3f(*com))
        if diagonal is not None:
            mass_api.CreateDiagonalInertiaAttr().Set(Gf.Vec3f(*diagonal))
        if principal_axes is not None:
            mass_api.CreatePrincipalAxesAttr().Set(Gf.Quatf(*principal_axes))
        if newton_inertia is not None:
            prim.CreateAttribute("newton:inertia", Sdf.ValueTypeNames.DoubleArray).Set(newton_inertia)
        return stage, prim

    async def test_newton_inertia_reader_ordering(self) -> None:
        """read_newton_inertia returns the 6 elements as [Ixx, Iyy, Izz, Ixy, Ixz, Iyz]."""
        vals = self._NEWTON_INERTIA
        _stage, prim = self._author_mass_prim(newton_inertia=vals)

        result = read_newton_inertia(prim)
        self.assertIsNotNone(result)
        self.assertEqual(len(result), 6)
        for got, exp in zip(result, vals):
            self.assertAlmostEqual(got, exp, places=15)

        # Malformed (wrong length) -> None
        _stage2, bad = self._author_mass_prim(newton_inertia=[1.0, 2.0, 3.0])
        self.assertIsNone(read_newton_inertia(bad))

        # Absent attribute -> None
        _stage3, none_prim = self._author_mass_prim(diagonal=(0.1, 0.2, 0.3))
        self.assertIsNone(read_newton_inertia(none_prim))

    async def test_inertia_prefers_newton_over_principal_axes(self) -> None:
        """newton:inertia overrides diagonalInertia + principalAxes when both are authored."""
        newton = self._NEWTON_INERTIA
        _stage, prim = self._author_mass_prim(
            newton_inertia=newton,
            diagonal=(1.0, 2.0, 3.0),  # deliberately different from newton values
            principal_axes=(1.0, 0.0, 0.0, 0.0),  # identity
        )

        data = read_inertial_from_prim(prim)
        self.assertIsNotNone(data)
        self.assertAlmostEqual(data.ixx, newton[0], places=12)
        self.assertAlmostEqual(data.iyy, newton[1], places=12)
        self.assertAlmostEqual(data.izz, newton[2], places=12)
        self.assertAlmostEqual(data.ixy, newton[3], places=12)
        self.assertAlmostEqual(data.ixz, newton[4], places=12)
        self.assertAlmostEqual(data.iyz, newton[5], places=12)

        # Prove the diagonalInertia reconstruction path was NOT used.
        self.assertNotAlmostEqual(data.ixx, 1.0, places=6)

    async def test_inertia_fallback_to_principal_axes(self) -> None:
        """Without newton:inertia, the tensor is reconstructed from diagonal + principal axes."""
        _stage, prim = self._author_mass_prim(
            diagonal=(0.1, 0.2, 0.3),
            principal_axes=(1.0, 0.0, 0.0, 0.0),  # identity -> tensor is diagonal
        )

        data = read_inertial_from_prim(prim)
        self.assertIsNotNone(data)
        self.assertAlmostEqual(data.ixx, 0.1, places=6)
        self.assertAlmostEqual(data.iyy, 0.2, places=6)
        self.assertAlmostEqual(data.izz, 0.3, places=6)
        self.assertAlmostEqual(data.ixy, 0.0, places=6)
        self.assertAlmostEqual(data.ixz, 0.0, places=6)
        self.assertAlmostEqual(data.iyz, 0.0, places=6)

    async def test_inertia_reconstruction_uses_gf_row_vector_convention(self) -> None:
        """A rotated principal frame must produce body-frame products of inertia with the correct signs."""
        from pxr import Gf

        angle = math.pi / 4.0
        principal_axes = Gf.Quatf(math.cos(angle / 2.0), Gf.Vec3f(0.0, 0.0, math.sin(angle / 2.0)))
        inertia = reconstruct_inertia_tensor(principal_axes, Gf.Vec3f(1.0, 2.0, 3.0))

        expected = (1.5, -0.5, 0.0, 1.5, 0.0, 3.0)
        for actual, expected_value in zip(inertia, expected):
            self.assertAlmostEqual(actual, expected_value, places=6)

    async def test_inertia_ignores_empty_newton_array(self) -> None:
        """An empty (default) newton:inertia array is ignored and falls back to reconstruction."""
        _stage, prim = self._author_mass_prim(
            newton_inertia=[],
            diagonal=(0.1, 0.2, 0.3),
            principal_axes=(1.0, 0.0, 0.0, 0.0),
        )

        self.assertIsNone(read_newton_inertia(prim))
        data = read_inertial_from_prim(prim)
        self.assertIsNotNone(data)
        self.assertAlmostEqual(data.ixx, 0.1, places=6)
        self.assertAlmostEqual(data.iyy, 0.2, places=6)
        self.assertAlmostEqual(data.izz, 0.3, places=6)

    @staticmethod
    def _build_newton_inertia_stage(newton_inertia: list[float]) -> Usd.Stage:
        """Build a 2-link articulation with newton:inertia authored on the base link.

        The base link also carries a deliberately different diagonalInertia so
        that a passing export proves newton:inertia took precedence.

        Args:
            newton_inertia: The ``newton:inertia`` array authored on ``base``.

        Returns:
            Generated in-memory USD stage.
        """
        from pxr import Gf, Sdf, UsdGeom

        stage = Usd.Stage.CreateInMemory()
        UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
        UsdGeom.SetStageMetersPerUnit(stage, 1.0)

        robot = UsdGeom.Xform.Define(stage, "/robot")
        stage.SetDefaultPrim(robot.GetPrim())
        UsdPhysics.ArticulationRootAPI.Apply(robot.GetPrim())

        base = UsdGeom.Xform.Define(stage, "/robot/base")
        UsdPhysics.RigidBodyAPI.Apply(base.GetPrim())
        base_mass = UsdPhysics.MassAPI.Apply(base.GetPrim())
        base_mass.CreateMassAttr().Set(1.0)
        base_mass.CreateDiagonalInertiaAttr().Set(Gf.Vec3f(0.001, 0.001, 0.001))
        base_mass.CreatePrincipalAxesAttr().Set(Gf.Quatf(1, 0, 0, 0))
        base.GetPrim().CreateAttribute("newton:inertia", Sdf.ValueTypeNames.DoubleArray).Set(newton_inertia)

        child = UsdGeom.Xform.Define(stage, "/robot/child")
        UsdPhysics.RigidBodyAPI.Apply(child.GetPrim())
        UsdPhysics.MassAPI.Apply(child.GetPrim()).CreateMassAttr().Set(0.5)

        world_joint = UsdPhysics.FixedJoint.Define(stage, "/robot/world_joint")
        world_joint.GetBody1Rel().SetTargets([Sdf.Path("/robot/base")])

        joint = UsdPhysics.RevoluteJoint.Define(stage, "/robot/base/joint")
        joint.CreateBody0Rel().SetTargets([Sdf.Path("/robot/base")])
        joint.CreateBody1Rel().SetTargets([Sdf.Path("/robot/child")])
        joint.GetAxisAttr().Set("Z")
        joint.GetLowerLimitAttr().Set(-90.0)
        joint.GetUpperLimitAttr().Set(90.0)

        UsdGeom.Scope.Define(stage, "/robot/Physics")
        return stage

    async def test_export_uses_newton_inertia(self) -> None:
        """End-to-end: exported <inertia> comes from newton:inertia, not the diagonal fallback."""
        newton = self._NEWTON_INERTIA
        stage = self._build_newton_inertia_stage(newton)

        with tempfile.TemporaryDirectory(ignore_cleanup_errors=True) as temp_dir:
            _, root = await self._export_to_urdf(stage, temp_dir)

            base_link = next((l for l in root.findall("link") if l.get("name") == "base"), None)
            self.assertIsNotNone(base_link, "Expected a 'base' link in exported URDF")
            inertial = base_link.find("inertial")
            self.assertIsNotNone(inertial, "base link missing <inertial>")
            inertia = inertial.find("inertia")
            self.assertIsNotNone(inertia, "base link missing <inertia>")

            # Diagonal (~1e-4) survives %.8f formatting; must match newton:inertia,
            # not the 0.001 diagonalInertia that the reconstruction path would use.
            self.assertAlmostEqual(float(inertia.get("ixx")), newton[0], delta=1e-7)
            self.assertAlmostEqual(float(inertia.get("iyy")), newton[1], delta=1e-7)
            self.assertAlmostEqual(float(inertia.get("izz")), newton[2], delta=1e-7)

            # Off-diagonals from newton are non-zero; the identity-axes reconstruction
            # would have produced exactly "0".
            self.assertNotEqual(inertia.get("ixy"), "0", "Expected non-zero Ixy from newton:inertia")

            await stage_utils.create_new_stage_async()
            await omni.kit.app.get_app().next_update_async()

    async def test_root_link_ancestor_chain_is_rebased_from_joint_origin(self) -> None:
        """Verify that ancestor transforms do not leak into a joint origin."""
        from pxr import Gf, Sdf, UsdGeom

        stage = Usd.Stage.CreateInMemory()
        UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
        UsdGeom.SetStageMetersPerUnit(stage, 1.0)

        robot = UsdGeom.Xform.Define(stage, "/robot")
        stage.SetDefaultPrim(robot.GetPrim())
        UsdPhysics.ArticulationRootAPI.Apply(robot.GetPrim())

        ancestor_transform = Gf.Matrix4d(1.0)
        ancestor_transform.SetRotateOnly(Gf.Rotation(Gf.Vec3d(2.0, 1.0, 3.0), 23.0))
        ancestor_transform.SetTranslateOnly(Gf.Vec3d(-0.18, 0.27, 0.12))
        import_frame = UsdGeom.Xform.Define(stage, "/robot/import_frame")
        import_frame.AddTransformOp().Set(ancestor_transform)

        root_transform = Gf.Matrix4d(1.0)
        root_transform.SetRotateOnly(Gf.Rotation(Gf.Vec3d(1.0, 2.0, 3.0), 37.0))
        root_transform.SetTranslateOnly(Gf.Vec3d(0.31, -0.22, 0.47))
        root_frame = UsdGeom.Xform.Define(stage, "/robot/import_frame/root_frame")
        root_frame.AddTransformOp().Set(root_transform)
        base = UsdGeom.Xform.Define(stage, "/robot/import_frame/root_frame/base")
        UsdPhysics.RigidBodyAPI.Apply(base.GetPrim())
        UsdPhysics.MassAPI.Apply(base.GetPrim()).CreateMassAttr().Set(1.0)

        world_joint = UsdPhysics.FixedJoint.Define(stage, "/robot/world_joint")
        world_joint.GetBody1Rel().SetTargets([Sdf.Path("/robot/import_frame/root_frame/base")])

        child_offset = (0.41, -0.13, 0.29)
        child_to_root = Gf.Matrix4d(1.0)
        child_to_root.SetTranslateOnly(Gf.Vec3d(*child_offset))
        child = UsdGeom.Xform.Define(stage, "/robot/import_frame/root_frame/child")
        child.AddTransformOp().Set(child_to_root)
        UsdPhysics.RigidBodyAPI.Apply(child.GetPrim())
        UsdPhysics.MassAPI.Apply(child.GetPrim()).CreateMassAttr().Set(0.5)

        joint = UsdPhysics.FixedJoint.Define(stage, "/robot/child_joint")
        joint.CreateBody0Rel().SetTargets([Sdf.Path("/robot/import_frame/root_frame/base")])
        joint.CreateBody1Rel().SetTargets([Sdf.Path("/robot/import_frame/root_frame/child")])
        joint.CreateLocalPos0Attr().Set(Gf.Vec3f(*child_offset))

        UsdGeom.Scope.Define(stage, "/robot/Physics")

        with tempfile.TemporaryDirectory(ignore_cleanup_errors=True) as temp_dir:
            _, root = await self._export_to_urdf(stage, temp_dir)

            base_link = root.find("./link[@name='base']")
            self.assertIsNotNone(base_link)

            child_joint = root.find("./joint[@name='child_joint']")
            self.assertIsNotNone(child_joint)
            joint_origin = child_joint.find("./origin")
            self.assertIsNotNone(joint_origin)
            joint_xyz = self._parse_xyz(joint_origin.get("xyz", ""))
            joint_rpy = self._parse_xyz(joint_origin.get("rpy", ""))
            for actual, expected in zip(joint_xyz, child_offset):
                self.assertAlmostEqual(actual, expected, places=6)
            for actual in joint_rpy:
                self.assertAlmostEqual(actual, 0.0, places=6)

    async def test_non_rigid_ancestor_frames_are_rebased_into_rigid_root(self) -> None:
        """Verify that non-rigid ancestor frames do not leak into child joints."""
        from pxr import Gf, Sdf, UsdGeom

        stage = Usd.Stage.CreateInMemory()
        UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
        UsdGeom.SetStageMetersPerUnit(stage, 1.0)

        robot = UsdGeom.Xform.Define(stage, "/robot")
        stage.SetDefaultPrim(robot.GetPrim())
        UsdPhysics.ArticulationRootAPI.Apply(robot.GetPrim())

        base_link_transform = Gf.Matrix4d(1.0)
        base_link_transform.SetTranslateOnly(Gf.Vec3d(0.0, 0.0, 0.0762))
        base_link = UsdGeom.Xform.Define(stage, "/robot/base_link")
        base_link.AddTransformOp().Set(base_link_transform)

        mount_transform = Gf.Matrix4d(1.0)
        mount_transform.SetRotateOnly(Gf.Rotation(Gf.Vec3d(1.0, 1.0, 0.0), 29.0))
        mount_transform.SetTranslateOnly(Gf.Vec3d(0.11, -0.07, 0.04))
        chassis_mount = UsdGeom.Xform.Define(stage, "/robot/base_link/chassis_mount")
        chassis_mount.AddTransformOp().Set(mount_transform)

        # base_link and chassis_mount deliberately have no RigidBodyAPI.
        base_chassis = UsdGeom.Xform.Define(stage, "/robot/base_link/chassis_mount/base_chassis_link")
        UsdPhysics.RigidBodyAPI.Apply(base_chassis.GetPrim())
        UsdPhysics.MassAPI.Apply(base_chassis.GetPrim()).CreateMassAttr().Set(1.0)

        world_joint = UsdPhysics.FixedJoint.Define(stage, "/robot/world_joint")
        world_joint.GetBody1Rel().SetTargets([Sdf.Path("/robot/base_link/chassis_mount/base_chassis_link")])

        sensor_offset = (0.3375, 0.0, 0.1439106)
        sensor_to_chassis = Gf.Matrix4d(1.0)
        sensor_to_chassis.SetTranslateOnly(Gf.Vec3d(*sensor_offset))
        sensor = UsdGeom.Xform.Define(stage, "/robot/base_link/chassis_mount/front_sensor_link")
        sensor.AddTransformOp().Set(sensor_to_chassis)
        UsdPhysics.RigidBodyAPI.Apply(sensor.GetPrim())
        UsdPhysics.MassAPI.Apply(sensor.GetPrim()).CreateMassAttr().Set(0.5)

        sensor_joint = UsdPhysics.FixedJoint.Define(stage, "/robot/chassis_to_sensor")
        sensor_joint.CreateBody0Rel().SetTargets([Sdf.Path("/robot/base_link/chassis_mount/base_chassis_link")])
        sensor_joint.CreateBody1Rel().SetTargets([Sdf.Path("/robot/base_link/chassis_mount/front_sensor_link")])
        sensor_joint.CreateLocalPos0Attr().Set(Gf.Vec3f(*sensor_offset))

        UsdGeom.Scope.Define(stage, "/robot/Physics")

        with tempfile.TemporaryDirectory(ignore_cleanup_errors=True) as temp_dir:
            _, root = await self._export_to_urdf(stage, temp_dir)

            self.assertIsNone(root.find("./link[@name='base_link']"))
            chassis_link = root.find("./link[@name='base_chassis_link']")
            self.assertIsNotNone(chassis_link)

            exported_joint = root.find("./joint[@name='chassis_to_sensor']")
            self.assertIsNotNone(exported_joint)
            joint_origin = exported_joint.find("./origin")
            self.assertIsNotNone(joint_origin)
            joint_xyz = self._parse_xyz(joint_origin.get("xyz", ""))
            joint_rpy = self._parse_xyz(joint_origin.get("rpy", ""))
            for actual, expected in zip(joint_xyz, sensor_offset):
                self.assertAlmostEqual(actual, expected, places=6)
            for actual in joint_rpy:
                self.assertAlmostEqual(actual, 0.0, places=6)

    @staticmethod
    def _build_issue709_collision_role_stage() -> Usd.Stage:
        """Create visual-only, collision-only, and visible dual-role controls."""
        stage = Usd.Stage.CreateInMemory()
        UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
        UsdGeom.SetStageMetersPerUnit(stage, 1.0)

        robot = UsdGeom.Xform.Define(stage, "/robot")
        stage.SetDefaultPrim(robot.GetPrim())
        UsdPhysics.ArticulationRootAPI.Apply(robot.GetPrim())

        base = UsdGeom.Xform.Define(stage, "/robot/base")
        UsdPhysics.RigidBodyAPI.Apply(base.GetPrim())
        mass_api = UsdPhysics.MassAPI.Apply(base.GetPrim())
        mass_api.CreateMassAttr().Set(1.0)
        mass_api.CreateDiagonalInertiaAttr().Set(Gf.Vec3f(0.001, 0.001, 0.001))

        world_joint = UsdPhysics.FixedJoint.Define(stage, "/robot/world_joint")
        world_joint.GetBody1Rel().SetTargets([Sdf.Path("/robot/base")])

        visual = UsdGeom.Cube.Define(stage, "/robot/base/visual_marker")
        visual.CreateSizeAttr().Set(0.1)
        visual.CreatePurposeAttr().Set(UsdGeom.Tokens.render)
        visual.AddTranslateOp().Set(Gf.Vec3d(-0.2, 0.0, 0.0))

        collision_default = UsdGeom.Cube.Define(stage, "/robot/base/collision_marker_default_purpose")
        collision_default.CreateSizeAttr().Set(0.1)
        collision_default.CreateVisibilityAttr().Set(UsdGeom.Tokens.invisible)
        collision_default.AddTranslateOp().Set(Gf.Vec3d(0.0, 0.0, 0.0))
        UsdPhysics.CollisionAPI.Apply(collision_default.GetPrim())

        collision_dual_role = UsdGeom.Cube.Define(stage, "/robot/base/collision_dual_role_visible")
        collision_dual_role.CreateSizeAttr().Set(0.1)
        collision_dual_role.AddTranslateOp().Set(Gf.Vec3d(0.2, 0.0, 0.0))
        UsdPhysics.CollisionAPI.Apply(collision_dual_role.GetPrim())

        collision_guide = UsdGeom.Cube.Define(stage, "/robot/base/collision_guide_control")
        collision_guide.CreateSizeAttr().Set(0.1)
        collision_guide.CreatePurposeAttr().Set(UsdGeom.Tokens.guide)
        collision_guide.AddTranslateOp().Set(Gf.Vec3d(0.4, 0.0, 0.0))
        UsdPhysics.CollisionAPI.Apply(collision_guide.GetPrim())

        UsdGeom.Scope.Define(stage, "/robot/Physics")
        return stage

    @staticmethod
    def _build_issue709_udim_stage(texture_path: str) -> Usd.Stage:
        """Create a one-link mesh using a UsdPreviewSurface UDIM texture."""
        stage = Usd.Stage.CreateInMemory()
        UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
        UsdGeom.SetStageMetersPerUnit(stage, 1.0)

        robot = UsdGeom.Xform.Define(stage, "/robot")
        stage.SetDefaultPrim(robot.GetPrim())
        UsdPhysics.ArticulationRootAPI.Apply(robot.GetPrim())

        base = UsdGeom.Xform.Define(stage, "/robot/base")
        UsdPhysics.RigidBodyAPI.Apply(base.GetPrim())
        mass_api = UsdPhysics.MassAPI.Apply(base.GetPrim())
        mass_api.CreateMassAttr().Set(1.0)
        mass_api.CreateDiagonalInertiaAttr().Set(Gf.Vec3f(0.001, 0.001, 0.001))

        world_joint = UsdPhysics.FixedJoint.Define(stage, "/robot/world_joint")
        world_joint.GetBody1Rel().SetTargets([Sdf.Path("/robot/base")])

        mesh = UsdGeom.Mesh.Define(stage, "/robot/base/udim_mesh")
        mesh.CreatePointsAttr().Set(
            [
                Gf.Vec3f(0.0, 0.0, 0.0),
                Gf.Vec3f(0.1, 0.0, 0.0),
                Gf.Vec3f(0.0, 0.1, 0.0),
                Gf.Vec3f(0.2, 0.0, 0.0),
                Gf.Vec3f(0.3, 0.0, 0.0),
                Gf.Vec3f(0.2, 0.1, 0.0),
            ]
        )
        mesh.CreateFaceVertexCountsAttr().Set([3, 3])
        mesh.CreateFaceVertexIndicesAttr().Set([0, 1, 2, 3, 4, 5])
        st = UsdGeom.PrimvarsAPI(mesh).CreatePrimvar(
            "st",
            Sdf.ValueTypeNames.TexCoord2fArray,
            UsdGeom.Tokens.faceVarying,
        )
        st.Set(
            [
                Gf.Vec2f(0.1, 0.1),
                Gf.Vec2f(0.9, 0.1),
                Gf.Vec2f(0.1, 0.9),
                Gf.Vec2f(1.1, 0.1),
                Gf.Vec2f(1.9, 0.1),
                Gf.Vec2f(1.1, 0.9),
            ]
        )

        material = UsdShade.Material.Define(stage, "/robot/Looks/issue709_material")
        surface = UsdShade.Shader.Define(stage, "/robot/Looks/issue709_material/preview_surface")
        surface.CreateIdAttr("UsdPreviewSurface")
        material.CreateSurfaceOutput().ConnectToSource(surface.ConnectableAPI(), "surface")

        texture = UsdShade.Shader.Define(stage, "/robot/Looks/issue709_material/albedo_texture")
        texture.CreateIdAttr("UsdUVTexture")
        texture.CreateInput("file", Sdf.ValueTypeNames.Asset).Set(Sdf.AssetPath(texture_path))
        texture.CreateOutput("rgb", Sdf.ValueTypeNames.Float3)
        surface.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f).ConnectToSource(
            texture.ConnectableAPI(), "rgb"
        )
        UsdShade.MaterialBindingAPI.Apply(mesh.GetPrim()).Bind(material)

        UsdGeom.Scope.Define(stage, "/robot/Physics")
        return stage

    @staticmethod
    def _issue709_map_kd_references(mesh_dir: str) -> list[tuple[str, str]]:
        """Return MTL path and map_Kd payload pairs in deterministic order."""
        references: list[tuple[str, str]] = []
        for root_dir, dir_names, file_names in os.walk(mesh_dir):
            dir_names.sort()
            for file_name in sorted(file_names):
                if not file_name.lower().endswith(".mtl"):
                    continue
                mtl_path = os.path.join(root_dir, file_name)
                with open(mtl_path, encoding="utf-8") as stream:
                    for line in stream:
                        if line.lstrip().startswith("map_Kd "):
                            references.append((mtl_path, line.split(maxsplit=1)[1].strip()))
        return references

    async def test_issue709_collision_default_purpose_is_not_visual_when_disabled(self) -> None:
        """Hidden or guide collisions stay collision-only while visible dual-role geometry survives."""
        stage = self._build_issue709_collision_role_stage()

        with tempfile.TemporaryDirectory(ignore_cleanup_errors=True) as temp_dir:
            _, root = await self._export_to_urdf(
                stage,
                temp_dir,
                visualize_collision_meshes=False,
            )

            visual_names = {element.get("name") for element in root.findall(".//visual")}
            collision_names = {element.get("name") for element in root.findall(".//collision")}

            self.assertIn("visual_marker", visual_names)
            self.assertNotIn(
                "collision_marker_default_purpose",
                visual_names,
                "CollisionAPI geometry leaked into <visual> while collision visualization was disabled",
            )
            self.assertIn("collision_marker_default_purpose", collision_names)
            self.assertIn("collision_dual_role_visible", visual_names)
            self.assertIn("collision_dual_role_visible", collision_names)
            self.assertIn("collision_guide_control", collision_names)
            self.assertNotIn("collision_guide_control", visual_names)

    async def test_issue709_resolvable_udim_tiles_are_delivered(self) -> None:
        """A resolvable UDIM template must deliver its concrete texture tiles."""
        png_bytes = bytes.fromhex(
            "89504e470d0a1a0a0000000d49484452000000010000000108060000001f15c489"
            "0000000d4944415408996360000002000154a24f5d0000000049454e44ae426082"
        )

        with tempfile.TemporaryDirectory(ignore_cleanup_errors=True) as temp_dir:
            source_dir = os.path.join(temp_dir, "source")
            output_dir = os.path.join(temp_dir, "export")
            os.makedirs(source_dir)
            os.makedirs(output_dir)

            for tile in ("1001", "1002"):
                tile_path = os.path.join(source_dir, f"albedo_{tile}.png")
                with open(tile_path, "wb") as stream:
                    stream.write(png_bytes)

            template_path = os.path.join(source_dir, "albedo_<UDIM>.png").replace("\\", "/")
            stage = self._build_issue709_udim_stage(template_path)
            await self._export_to_urdf(stage, output_dir)

            references = self._issue709_map_kd_references(os.path.join(output_dir, "meshes"))
            self.assertEqual(
                sorted(reference for _, reference in references),
                ["albedo_1001.png", "albedo_1002.png"],
            )
            for mtl_path, reference in references:
                self.assertNotIn("<UDIM>", reference)
                concrete_path = os.path.join(os.path.dirname(mtl_path), reference)
                self.assertTrue(
                    os.path.isfile(concrete_path),
                    f"UDIM map_Kd payload was not delivered: {concrete_path}",
                )

    async def test_issue709_percent_encoded_udim_tiles_are_delivered(self) -> None:
        """A percent-encoded UDIM token must resolve to a concrete delivered tile."""
        png_bytes = bytes.fromhex(
            "89504e470d0a1a0a0000000d49484452000000010000000108060000001f15c489"
            "0000000d4944415408996360000002000154a24f5d0000000049454e44ae426082"
        )

        with tempfile.TemporaryDirectory(ignore_cleanup_errors=True) as temp_dir:
            source_dir = os.path.join(temp_dir, "source")
            output_dir = os.path.join(temp_dir, "export")
            os.makedirs(source_dir)
            os.makedirs(output_dir)

            for tile in ("1001", "1002"):
                tile_path = os.path.join(source_dir, f"albedo_{tile}.png")
                with open(tile_path, "wb") as stream:
                    stream.write(png_bytes)

            template_path = os.path.join(source_dir, "albedo_%3CUDIM%3E.png").replace("\\", "/")
            stage = self._build_issue709_udim_stage(template_path)
            await self._export_to_urdf(stage, output_dir)

            references = self._issue709_map_kd_references(os.path.join(output_dir, "meshes"))
            self.assertEqual(
                sorted(reference for _, reference in references),
                ["albedo_1001.png", "albedo_1002.png"],
            )
            for mtl_path, reference in references:
                self.assertNotIn("<UDIM>", reference)
                self.assertNotIn("%3CUDIM%3E", reference)
                self.assertTrue(
                    os.path.isfile(os.path.join(os.path.dirname(mtl_path), reference)),
                    f"Encoded UDIM map_Kd payload was not delivered: {reference}",
                )

    async def test_issue709_udim_face_crossing_tile_boundary_is_rejected(self) -> None:
        """A face spanning multiple UDIM tiles must fail instead of silently mis-texturing."""
        png_bytes = bytes.fromhex(
            "89504e470d0a1a0a0000000d49484452000000010000000108060000001f15c489"
            "0000000d4944415408996360000002000154a24f5d0000000049454e44ae426082"
        )

        with tempfile.TemporaryDirectory(ignore_cleanup_errors=True) as temp_dir:
            source_dir = os.path.join(temp_dir, "source")
            output_dir = os.path.join(temp_dir, "export")
            os.makedirs(source_dir)
            os.makedirs(output_dir)

            for tile in ("1001", "1002"):
                with open(os.path.join(source_dir, f"albedo_{tile}.png"), "wb") as stream:
                    stream.write(png_bytes)

            template_path = os.path.join(source_dir, "albedo_<UDIM>.png").replace("\\", "/")
            stage = self._build_issue709_udim_stage(template_path)
            st = UsdGeom.PrimvarsAPI(stage.GetPrimAtPath("/robot/base/udim_mesh")).GetPrimvar("st")
            st.Set(
                [
                    Gf.Vec2f(0.9, 0.1),
                    Gf.Vec2f(1.1, 0.1),
                    Gf.Vec2f(0.9, 0.9),
                    Gf.Vec2f(1.1, 0.1),
                    Gf.Vec2f(1.9, 0.1),
                    Gf.Vec2f(1.1, 0.9),
                ]
            )

            with self.assertRaisesRegex(ValueError, "crosses tile boundaries"):
                await self._export_to_urdf(stage, output_dir)

    async def test_issue709_unresolved_udim_does_not_emit_broken_map_reference(self) -> None:
        """An unresolved UDIM template must not produce a broken map_Kd."""
        with tempfile.TemporaryDirectory(ignore_cleanup_errors=True) as temp_dir:
            source_dir = os.path.join(temp_dir, "source")
            output_dir = os.path.join(temp_dir, "export")
            os.makedirs(source_dir)
            os.makedirs(output_dir)

            template_path = os.path.join(source_dir, "missing_<UDIM>.png").replace("\\", "/")
            stage = self._build_issue709_udim_stage(template_path)
            await self._export_to_urdf(stage, output_dir)

            references = self._issue709_map_kd_references(os.path.join(output_dir, "meshes"))
            self.assertFalse(
                references,
                f"Exporter emitted a map_Kd without a complete UDIM payload: {references}",
            )
