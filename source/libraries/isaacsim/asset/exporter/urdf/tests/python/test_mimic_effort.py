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

"""Verify passive mimic effort fallback without weakening other limit validation."""

from __future__ import annotations

import math
import tempfile
import unittest
import xml.etree.ElementTree as ET
from pathlib import Path

from isaacsim.asset.exporter.urdf import UsdToUrdfConverter
from pxr import Sdf, Usd, UsdGeom, UsdPhysics


def _make_robot() -> tuple[Usd.Stage, Usd.Prim]:
    """Create a driven source and a passive Newton mimic follower.

    Returns:
        Stage and follower joint prim.
    """
    stage = Usd.Stage.CreateInMemory()
    UsdGeom.SetStageMetersPerUnit(stage, 1.0)
    root = UsdGeom.Xform.Define(stage, "/Robot").GetPrim()
    stage.SetDefaultPrim(root)
    UsdPhysics.ArticulationRootAPI.Apply(root)
    for name in ("base", "middle", "tip"):
        prim = UsdGeom.Xform.Define(stage, f"/Robot/{name}").GetPrim()
        UsdPhysics.RigidBodyAPI.Apply(prim)
        UsdPhysics.MassAPI.Apply(prim).CreateMassAttr(1.0)
    for name, parent, child in (("source", "base", "middle"), ("follower", "middle", "tip")):
        joint = UsdPhysics.RevoluteJoint.Define(stage, f"/Robot/{name}")
        joint.CreateBody0Rel().SetTargets([f"/Robot/{parent}"])
        joint.CreateBody1Rel().SetTargets([f"/Robot/{child}"])
        joint.CreateLowerLimitAttr(-90)
        joint.CreateUpperLimitAttr(90)
        joint.GetPrim().CreateAttribute("newton:velocityLimit", Sdf.ValueTypeNames.Float).Set(180)
    source = stage.GetPrimAtPath("/Robot/source")
    drive = UsdPhysics.DriveAPI.Apply(source, "angular")
    drive.CreateMaxForceAttr(10)
    drive.CreateStiffnessAttr(100)
    follower = stage.GetPrimAtPath("/Robot/follower")
    follower.AddAppliedSchema("NewtonMimicAPI")
    follower.CreateRelationship("newton:mimicJoint").SetTargets([source.GetPath()])
    follower.CreateAttribute("newton:mimicCoef1", Sdf.ValueTypeNames.Float).Set(0.5)
    follower.CreateAttribute("newton:mimicCoef0", Sdf.ValueTypeNames.Float).Set(0)
    return stage, follower


class TestMimicEffort(unittest.TestCase):
    """Exercise public export with passive and independently driven followers."""

    def test_passive_follower_effort(self) -> None:
        """Export missing or unbounded effort as zero and preserve finite effort."""
        for effort in (None, math.inf, 0.0, 7.0):
            with self.subTest(effort=effort), tempfile.TemporaryDirectory() as directory:
                stage, follower = _make_robot()
                if effort is not None:
                    UsdPhysics.DriveAPI.Apply(follower, "angular").CreateMaxForceAttr(effort)
                output = str(Path(directory) / "robot.urdf")
                UsdToUrdfConverter(stage).convert(output)
                joint = ET.parse(output).find("./joint[@name='follower']")
                self.assertEqual(float(joint.find("limit").get("effort")), 7.0 if effort == 7 else 0.0)
                self.assertEqual(joint.find("mimic").get("joint"), "source")
                self.assertEqual(float(joint.find("mimic").get("multiplier")), 0.5)

    def test_active_follower_unauthored_effort_fallback(self) -> None:
        """Supply finite effort for position and velocity drives without authored limits."""
        for gain in ("stiffness", "damping"):
            with self.subTest(gain=gain), tempfile.TemporaryDirectory() as directory:
                stage, follower = _make_robot()
                drive = UsdPhysics.DriveAPI.Apply(follower, "angular")
                getattr(drive, f"Create{gain.title()}Attr")(1.0)
                output = str(Path(directory) / "robot.urdf")
                UsdToUrdfConverter(stage).convert(output)
                limit = ET.parse(output).find("./joint[@name='follower']/limit")
                self.assertEqual(float(limit.get("effort")), 1e6)

    def test_other_missing_limits_still_fail(self) -> None:
        """Retain velocity validation even when passive effort can be inferred."""
        stage, follower = _make_robot()
        follower.RemoveProperty("newton:velocityLimit")
        stage.GetPrimAtPath("/Robot/source").RemoveProperty("newton:velocityLimit")
        with tempfile.TemporaryDirectory() as directory, self.assertRaisesRegex(ValueError, "source: velocity"):
            UsdToUrdfConverter(stage).convert(str(Path(directory) / "robot.urdf"))

    def test_non_mimic_still_requires_effort(self) -> None:
        """Do not infer passive status from missing drives alone."""
        stage, follower = _make_robot()
        follower.RemoveProperty("newton:mimicJoint")
        with tempfile.TemporaryDirectory() as directory, self.assertRaisesRegex(ValueError, "follower: effort"):
            UsdToUrdfConverter(stage).convert(str(Path(directory) / "robot.urdf"))

    def test_physx_mimic_equation(self) -> None:
        """Preserve PhysX coupling direction and convert angular offsets to radians."""
        stage, follower = _make_robot()
        follower.RemoveAPI("NewtonMimicAPI")
        follower.AddAppliedSchema("PhysxMimicJointAPI:rotZ")
        follower.CreateRelationship("physxMimicJoint:rotZ:referenceJoint").SetTargets(["/Robot/source"])
        follower.CreateAttribute("physxMimicJoint:rotZ:gearing", Sdf.ValueTypeNames.Float).Set(-0.5)
        follower.CreateAttribute("physxMimicJoint:rotZ:offset", Sdf.ValueTypeNames.Float).Set(-30)
        with tempfile.TemporaryDirectory() as directory:
            output = str(Path(directory) / "robot.urdf")
            UsdToUrdfConverter(stage).convert(output)
            mimic = ET.parse(output).find("./joint[@name='follower']/mimic")
            self.assertEqual(float(mimic.get("multiplier")), 0.5)
            self.assertAlmostEqual(float(mimic.get("offset")), math.pi / 6)

    def test_newton_mimic_offset_units(self) -> None:
        """Convert a Newton angular mimic offset without changing its sign."""
        stage, follower = _make_robot()
        follower.GetAttribute("newton:mimicCoef0").Set(30)
        with tempfile.TemporaryDirectory() as directory:
            output = str(Path(directory) / "robot.urdf")
            UsdToUrdfConverter(stage).convert(output)
            mimic = ET.parse(output).find("./joint[@name='follower']/mimic")
            self.assertAlmostEqual(float(mimic.get("offset")), math.pi / 6)

    def test_passive_velocity_from_source(self) -> None:
        """Infer scaled speed including negative and zero ratios, preserving finite limits."""
        for multiplier in (1.0, -1.6, 2.4, 0.0):
            for authored in (None, math.inf, 0.0, 45.0):
                with self.subTest(multiplier=multiplier, authored=authored), tempfile.TemporaryDirectory() as directory:
                    stage, follower = _make_robot()
                    follower.GetAttribute("newton:mimicCoef1").Set(multiplier)
                    follower.RemoveProperty("newton:velocityLimit")
                    if authored is not None:
                        follower.CreateAttribute("newton:velocityLimit", Sdf.ValueTypeNames.Float).Set(authored)
                    output = str(Path(directory) / "robot.urdf")
                    UsdToUrdfConverter(stage).convert(output)
                    limit = ET.parse(output).find("./joint[@name='follower']/limit")
                    expected = abs(multiplier) * math.pi if authored in (None, math.inf) else math.radians(authored)
                    self.assertAlmostEqual(float(limit.get("velocity")), expected, places=6)
                    self.assertEqual(float(limit.get("effort")), 0.0)

    def test_chained_mimic_velocity(self) -> None:
        """Resolve a follower chain even when the downstream joint is read first."""
        from isaacsim.asset.exporter.urdf.converter.joint_reader import JointData, _resolve_mimic_limits

        source = JointData(name="source", limit_velocity=0.5)
        middle = JointData(name="middle", mimic_joint="source", mimic_multiplier=-1.6, is_passive_mimic=True)
        tip = JointData(name="tip", mimic_joint="middle", mimic_multiplier=2.4, is_passive_mimic=True)
        _resolve_mimic_limits([tip, middle, source])
        self.assertAlmostEqual(middle.limit_velocity, 0.8)
        self.assertAlmostEqual(tip.limit_velocity, 1.92)

    def test_invalid_mimic_graph(self) -> None:
        """Reject cycles, dangling references and non-finite mimic coefficients."""
        from isaacsim.asset.exporter.urdf.converter.joint_reader import JointData, _resolve_mimic_limits

        for reference, multiplier, message in (
            ("follower", 1.0, "cyclic"),
            ("missing", 1.0, "not an exported"),
            ("source", math.nan, "non-finite"),
        ):
            with self.subTest(reference=reference, multiplier=multiplier):
                joints = [
                    JointData(name="source", limit_velocity=1.0),
                    JointData(
                        name="follower", mimic_joint=reference, mimic_multiplier=multiplier, is_passive_mimic=True
                    ),
                ]
                with self.assertRaisesRegex(ValueError, message):
                    _resolve_mimic_limits(joints)

    def test_active_follower_velocity_not_inferred(self) -> None:
        """Do not infer velocity for an independently driven mimic joint."""
        stage, follower = _make_robot()
        follower.RemoveProperty("newton:velocityLimit")
        drive = UsdPhysics.DriveAPI.Apply(follower, "angular")
        drive.CreateMaxForceAttr(1.0)
        drive.CreateStiffnessAttr(100.0)
        with tempfile.TemporaryDirectory() as directory, self.assertRaisesRegex(ValueError, "follower: velocity"):
            UsdToUrdfConverter(stage).convert(str(Path(directory) / "robot.urdf"))

    def test_required_position_bounds_from_source(self) -> None:
        """Transform position intervals with sign and offset, preserving authored bounds."""
        from isaacsim.asset.exporter.urdf.converter.joint_reader import JointData, _resolve_mimic_limits

        for multiplier, expected in ((2.0, (-3.0, 7.0)), (-2.0, (-5.0, 5.0)), (0.0, (1.0, 1.0))):
            with self.subTest(multiplier=multiplier):
                source = JointData(
                    name="source", joint_type="prismatic", limit_lower=-2.0, limit_upper=3.0, limit_velocity=0.5
                )
                follower = JointData(
                    name="follower",
                    joint_type="prismatic",
                    mimic_joint="source",
                    mimic_multiplier=multiplier,
                    mimic_offset=1.0,
                    is_passive_mimic=True,
                    limit_lower=-math.inf,
                    limit_upper=math.inf,
                )
                _resolve_mimic_limits([follower, source])
                self.assertEqual((follower.limit_lower, follower.limit_upper), expected)
                follower.limit_lower = -0.1
                follower.limit_upper = 0.1
                _resolve_mimic_limits([follower, source])
                self.assertEqual((follower.limit_lower, follower.limit_upper), (-0.1, 0.1))

    def test_one_sided_position_bounds_rejected(self) -> None:
        """Reject one-sided bounds without replacing an existing export."""
        for joint_type in ("PhysicsRevoluteJoint", "PhysicsPrismaticJoint"):
            for lower, upper in ((100.0, math.inf), (-math.inf, -100.0), (10.0, math.inf), (-math.inf, 10.0)):
                with self.subTest(joint_type=joint_type, lower=lower, upper=upper):
                    stage, follower = _make_robot()
                    follower.SetTypeName(joint_type)
                    follower.GetAttribute("physics:lowerLimit").Set(lower)
                    follower.GetAttribute("physics:upperLimit").Set(upper)
                    with tempfile.TemporaryDirectory() as directory:
                        output = Path(directory) / "robot.urdf"
                        output.write_text("existing export")
                        with self.assertRaisesRegex(ValueError, "one-sided position bounds.*follower"):
                            UsdToUrdfConverter(stage).convert(str(output))
                        self.assertEqual(output.read_text(), "existing export")

    def test_authored_position_interval_validation(self) -> None:
        """Reject conflicting bounds while accepting authored zero-width intervals."""
        from isaacsim.asset.exporter.urdf.converter.joint_reader import JointData, _resolve_mimic_limits

        source = JointData(name="source", limit_lower=-2.0, limit_upper=3.0)
        for lower, upper in ((10.0, 7.0), (7.0, 7.0)):
            with self.subTest(lower=lower, upper=upper):
                follower = JointData(
                    name="follower",
                    joint_type="prismatic",
                    mimic_joint="source",
                    mimic_multiplier=2.0,
                    mimic_offset=1.0,
                    is_passive_mimic=True,
                    limit_lower=lower,
                    limit_upper=upper,
                )
                if lower > upper:
                    with self.assertRaisesRegex(ValueError, "conflicting position bounds.*follower"):
                        _resolve_mimic_limits([follower, source])
                else:
                    _resolve_mimic_limits([follower, source])
                    self.assertEqual((follower.limit_lower, follower.limit_upper), (lower, upper))

    def test_unbounded_source_does_not_invent_bounds(self) -> None:
        """Leave unknown source limits unresolved rather than inventing finite bounds."""
        from isaacsim.asset.exporter.urdf.converter.joint_reader import JointData, _resolve_mimic_limits

        source = JointData(name="source", joint_type="continuous")
        follower = JointData(
            name="follower", joint_type="prismatic", mimic_joint="source", mimic_multiplier=1.0, is_passive_mimic=True
        )
        _resolve_mimic_limits([follower, source])
        self.assertIsNone(follower.limit_lower)
        self.assertIsNone(follower.limit_upper)
        self.assertIsNone(follower.limit_velocity)

    def test_mixed_joint_ratio_units(self) -> None:
        """Convert linear/angular ratios before deriving follower limits."""
        from isaacsim.asset.exporter.urdf.converter.joint_reader import _mimic_multiplier_to_si

        stage = Usd.Stage.CreateInMemory()
        UsdGeom.SetStageMetersPerUnit(stage, 0.01)
        angular = UsdPhysics.RevoluteJoint.Define(stage, "/angular").GetPrim()
        linear = UsdPhysics.PrismaticJoint.Define(stage, "/linear").GetPrim()
        self.assertAlmostEqual(_mimic_multiplier_to_si(linear, str(angular.GetPath()), 2.0), 0.02 / math.radians(1))
        self.assertAlmostEqual(_mimic_multiplier_to_si(angular, str(linear.GetPath()), 2.0), math.radians(2) / 0.01)

    def test_prismatic_export_infers_all_required_limits(self) -> None:
        """Export a passive slider with no independent limits using source SI bounds."""
        stage, follower = _make_robot()
        UsdGeom.SetStageMetersPerUnit(stage, 0.01)
        source = stage.GetPrimAtPath("/Robot/source")
        for prim in (source, follower):
            prim.SetTypeName("PhysicsPrismaticJoint")
        source.RemoveAPI(UsdPhysics.DriveAPI, "angular")
        UsdPhysics.DriveAPI.Apply(source, "linear").CreateMaxForceAttr(10.0)
        source.GetAttribute("physics:lowerLimit").Set(-100.0)
        source.GetAttribute("physics:upperLimit").Set(200.0)
        for name in ("physics:lowerLimit", "physics:upperLimit", "newton:velocityLimit"):
            follower.RemoveProperty(name)
        follower.GetAttribute("newton:mimicCoef1").Set(-2.0)
        follower.GetAttribute("newton:mimicCoef0").Set(10.0)
        with tempfile.TemporaryDirectory() as directory:
            output = str(Path(directory) / "robot.urdf")
            UsdToUrdfConverter(stage).convert(output)
            limit = ET.parse(output).find("./joint[@name='follower']/limit")
            for name, expected in (("lower", -3.9), ("upper", 2.1), ("velocity", 3.6), ("effort", 0.0)):
                self.assertAlmostEqual(float(limit.get(name)), expected)
            from isaacsim.asset.importer.urdf.impl.drive_reconstruction import parse_source_drive_breadcrumbs

            records = parse_source_drive_breadcrumbs(output)
            record = next(record for record in records if record.joint_name == "follower")
            self.assertEqual(set(record.inferred_mimic_limits), {"velocity", "lower", "upper"})

    def test_invalid_velocity_not_inferred(self) -> None:
        """Reject invalid authored velocity values instead of replacing them."""
        for value in (math.nan, -math.inf):
            with self.subTest(value=value), tempfile.TemporaryDirectory() as directory:
                stage, follower = _make_robot()
                follower.GetAttribute("newton:velocityLimit").Set(value)
                with self.assertRaisesRegex(ValueError, "follower: velocity"):
                    UsdToUrdfConverter(stage).convert(str(Path(directory) / "robot.urdf"))

    def test_actuated_unauthored_effort_roundtrip(self) -> None:
        """Remove synthesized effort opinions after public export and import."""
        from isaacsim.asset.importer.urdf import URDFImporter
        from isaacsim.asset.importer.urdf.impl.config import URDFImporterConfig
        from isaacsim.asset.importer.urdf.impl.drive_reconstruction import parse_source_drive_breadcrumbs

        for kind in ("PhysicsRevoluteJoint", "PhysicsPrismaticJoint", "continuous", "mujoco"):
            with self.subTest(kind=kind), tempfile.TemporaryDirectory() as directory:
                stage, _ = _make_robot()
                source = stage.GetPrimAtPath("/Robot/source")
                source.RemoveProperty("drive:angular:physics:maxForce")
                if kind == "PhysicsPrismaticJoint":
                    source.SetTypeName(kind)
                    source.RemoveAPI(UsdPhysics.DriveAPI, "angular")
                    UsdPhysics.DriveAPI.Apply(source, "linear").CreateStiffnessAttr(100)
                if kind == "continuous":
                    source.GetAttribute("physics:lowerLimit").Set(-math.inf)
                    source.GetAttribute("physics:upperLimit").Set(math.inf)
                elif kind == "mujoco":
                    source.RemoveAPI(UsdPhysics.DriveAPI, "angular")
                    actuator = stage.DefinePrim("/Robot/Physics/source_actuator", "MjcActuator")
                    actuator.CreateRelationship("mjc:target").SetTargets([source.GetPath()])
                    actuator.CreateAttribute("mjc:gainType", Sdf.ValueTypeNames.Token).Set("fixed")
                    actuator.CreateAttribute("mjc:biasType", Sdf.ValueTypeNames.Token).Set("affine")
                    actuator.CreateAttribute("mjc:gainPrm", Sdf.ValueTypeNames.FloatArray).Set([100, 0, 0])
                    actuator.CreateAttribute("mjc:biasPrm", Sdf.ValueTypeNames.FloatArray).Set([0, -100, -1])
                output = str(Path(directory) / "robot.urdf")
                UsdToUrdfConverter(stage).convert(output)
                limit = ET.parse(output).find("./joint[@name='source']/limit")
                self.assertEqual(float(limit.get("effort")), 1e6)
                record = next(r for r in parse_source_drive_breadcrumbs(output) if r.joint_name == "source")
                self.assertTrue(record.unauthored_effort)
                importer = URDFImporter(
                    config=URDFImporterConfig(urdf_path=output, usd_path=directory, run_asset_transformer=False)
                )
                imported = Usd.Stage.Open(importer.import_urdf())
                joint = next(p for p in imported.Traverse() if p.GetName() == "source" and p.IsA(UsdPhysics.Joint))
                instance = "linear" if kind == "PhysicsPrismaticJoint" else "angular"
                force = UsdPhysics.DriveAPI(joint, instance).GetMaxForceAttr()
                self.assertFalse(force.HasAuthoredValueOpinion())
                self.assertEqual(force.Get(), math.inf)
                for prim in imported.Traverse():
                    target = prim.GetRelationship("mjc:target")
                    if target and joint.GetPath() in target.GetTargets():
                        for side in ("min", "max"):
                            attr = prim.GetAttribute(f"mjc:forceRange:{side}")
                            self.assertFalse(attr and attr.HasAuthoredValueOpinion())

    def test_authored_effort_never_marked_unauthored(self) -> None:
        """Preserve finite authored force limits including the fallback's exact value."""
        from isaacsim.asset.importer.urdf.impl.drive_reconstruction import parse_source_drive_breadcrumbs

        for value in (0.0, 10.0, 1e6):
            with self.subTest(value=value), tempfile.TemporaryDirectory() as directory:
                stage, _ = _make_robot()
                source = stage.GetPrimAtPath("/Robot/source")
                UsdPhysics.DriveAPI(source, "angular").CreateMaxForceAttr(value)
                output = str(Path(directory) / "robot.urdf")
                UsdToUrdfConverter(stage).convert(output)
                self.assertEqual(float(ET.parse(output).find("./joint[@name='source']/limit").get("effort")), value)
                record = next(r for r in parse_source_drive_breadcrumbs(output) if r.joint_name == "source")
                self.assertFalse(record.unauthored_effort)

    def test_continuous_drive_without_limits_omits_limit_element(self) -> None:
        """Do not synthesize a partial limit for a fully unspecified continuous drive."""
        stage, _ = _make_robot()
        source = stage.GetPrimAtPath("/Robot/source")
        source.GetAttribute("physics:lowerLimit").Set(-math.inf)
        source.GetAttribute("physics:upperLimit").Set(math.inf)
        source.RemoveProperty("drive:angular:physics:maxForce")
        source.RemoveProperty("newton:velocityLimit")
        with tempfile.TemporaryDirectory() as directory:
            output = str(Path(directory) / "robot.urdf")
            UsdToUrdfConverter(stage).convert(output)
            joint = ET.parse(output).find("./joint[@name='source']")
            self.assertEqual(joint.get("type"), "continuous")
            self.assertIsNone(joint.find("limit"))
