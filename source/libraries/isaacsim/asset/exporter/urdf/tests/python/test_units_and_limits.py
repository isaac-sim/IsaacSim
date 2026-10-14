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

"""Physical unit equivalence and incomplete-limit export regressions."""

import itertools
import json
import math
import shutil
import subprocess
import tempfile
import unittest
import xml.etree.ElementTree as ET
from pathlib import Path

from isaacsim.asset.exporter.urdf import UsdToUrdfConverter
from isaacsim.asset.exporter.urdf.converter.joint_reader import (
    JointData,
    _collect_source_drive_breadcrumb,
    _expand_multi_dof_joint,
    _read_limits,
)
from isaacsim.asset.exporter.urdf.converter.link_reader import LinkData
from isaacsim.asset.exporter.urdf.converter.urdf_writer import write_urdf
from pxr import Gf, Sdf, Usd, UsdGeom, UsdPhysics


class TestUnitsAndLimits(unittest.TestCase):
    """Exercise unitsandlimits regressions."""

    def test_equivalent_stage_units(self) -> None:
        """Verify equivalent stage units."""
        for unit in (1.0, 0.01, 0.001):
            with self.subTest(unit=unit), tempfile.TemporaryDirectory() as directory:
                stage = Usd.Stage.CreateNew(str(Path(directory) / "robot.usda"))
                UsdGeom.SetStageMetersPerUnit(stage, unit)
                UsdPhysics.SetStageKilogramsPerUnit(stage, 1.0)
                robot = UsdGeom.Xform.Define(stage, "/Robot").GetPrim()
                stage.SetDefaultPrim(robot)
                UsdPhysics.ArticulationRootAPI.Apply(robot)
                for name in ("base", "child"):
                    link = UsdGeom.Xform.Define(stage, f"/Robot/{name}").GetPrim()
                    UsdPhysics.RigidBodyAPI.Apply(link)
                    mass = UsdPhysics.MassAPI.Apply(link)
                    mass.CreateMassAttr(2)
                    mass.CreateCenterOfMassAttr(Gf.Vec3f(0.1 / unit, 0, 0))
                    mass.CreateDiagonalInertiaAttr(Gf.Vec3f(0.02 / unit**2))
                    cube = UsdGeom.Cube.Define(stage, f"/Robot/{name}/visual")
                    cube.CreateSizeAttr(0.2 / unit)
                UsdGeom.Xformable(stage.GetPrimAtPath("/Robot/child")).AddTranslateOp().Set(Gf.Vec3d(1 / unit, 0, 0))
                fixed = UsdPhysics.FixedJoint.Define(stage, "/Robot/fixed")
                fixed.CreateBody1Rel().SetTargets(["/Robot/base"])
                joint = UsdPhysics.RevoluteJoint.Define(stage, "/Robot/hinge")
                joint.CreateBody0Rel().SetTargets(["/Robot/base"])
                joint.CreateBody1Rel().SetTargets(["/Robot/child"])
                joint.CreateLocalPos0Attr(Gf.Vec3f(1 / unit, 0, 0))
                joint.CreateLowerLimitAttr(-90)
                joint.CreateUpperLimitAttr(90)
                joint.GetPrim().CreateAttribute("newton:velocityLimit", Sdf.ValueTypeNames.Float).Set(180)
                UsdPhysics.DriveAPI.Apply(joint.GetPrim(), "angular").CreateMaxForceAttr(5 / unit**2)
                stage.GetRootLayer().Save()
                # Exercise stage reopening and the public conversion entry point.
                output = str(Path(directory) / "robot.urdf")
                UsdToUrdfConverter(str(Path(directory) / "robot.usda")).convert(output)
                if shutil.which("check_urdf"):
                    parsed = subprocess.run(["check_urdf", output], capture_output=True, text=True)
                    self.assertEqual(parsed.returncode, 0, parsed.stdout + parsed.stderr)
                xml = ET.parse(output)
                origin = xml.find("./joint[@name='hinge']/origin")
                self.assertAlmostEqual(float(origin.get("xyz").split()[0]), 1.0)
                limit = xml.find("./joint[@name='hinge']/limit")
                self.assertAlmostEqual(float(limit.get("lower")), -math.pi / 2, places=5)
                self.assertAlmostEqual(float(limit.get("effort")), 5)
                self.assertAlmostEqual(float(limit.get("velocity")), math.pi, places=5)
                inertial = xml.find("./link[@name='child']/inertial")
                self.assertAlmostEqual(float(inertial.find("origin").get("xyz").split()[0]), 0.1)
                self.assertAlmostEqual(float(inertial.find("inertia").get("ixx")), 0.02)
                for box in xml.findall(".//box"):
                    self.assertAlmostEqual(float(box.get("size").split()[0]), 0.2)

    def test_prismatic_native_and_breadcrumb_units(self) -> None:
        """Verify prismatic native and breadcrumb units."""
        stage = Usd.Stage.CreateInMemory()
        UsdGeom.SetStageMetersPerUnit(stage, 0.01)
        joint = UsdPhysics.PrismaticJoint.Define(stage, "/slider")
        joint.CreateLowerLimitAttr(-100)
        joint.CreateUpperLimitAttr(100)
        prim = joint.GetPrim()
        prim.CreateAttribute("urdf:limit:effort", Sdf.ValueTypeNames.Double).Set(7)
        prim.CreateAttribute("urdf:limit:velocity", Sdf.ValueTypeNames.Double).Set(2)
        data = JointData(joint_type="prismatic")
        _read_limits(prim, data)
        self.assertEqual(
            (
                data.limit_lower,
                data.limit_upper,
                data.limit_effort,
                data.limit_velocity,
            ),
            (-1, 1, 7, 2),
        )
        prim.CreateAttribute("newton:velocityLimit", Sdf.ValueTypeNames.Double).Set(300)
        _read_limits(prim, data)
        self.assertEqual(data.limit_velocity, 3)

    def test_incomplete_limits_preserve_existing_export(self) -> None:
        """Verify incomplete limits preserve existing export."""
        for joint_type in ("revolute", "prismatic", "continuous"):
            for missing in ("effort", "velocity"):
                with (
                    self.subTest(joint_type=joint_type, missing=missing),
                    tempfile.TemporaryDirectory() as directory,
                ):
                    output = Path(directory) / "robot.urdf"
                    output.write_text("previous export")
                    joint = JointData(
                        name="hinge",
                        joint_type=joint_type,
                        parent_link="a",
                        child_link="b",
                        limit_lower=-1,
                        limit_upper=1,
                        limit_effort=3,
                        limit_velocity=4,
                    )
                    setattr(joint, f"limit_{missing}", None)
                    with self.assertRaisesRegex(ValueError, f"hinge: {missing}"):
                        write_urdf(
                            "robot",
                            [LinkData(name="a"), LinkData(name="b")],
                            [joint],
                            [],
                            str(output),
                        )
                    self.assertEqual(output.read_text(), "previous export")

    def test_fixed_joint_requires_no_limits(self) -> None:
        """Verify fixed joint requires no limits."""
        with tempfile.TemporaryDirectory() as directory:
            output = str(Path(directory) / "robot.urdf")
            write_urdf(
                "robot",
                [LinkData(name="a"), LinkData(name="b")],
                [JointData(name="fixed", parent_link="a", child_link="b")],
                [],
                output,
            )
            self.assertIsNone(ET.parse(output).find(".//limit"))

    def test_continuous_without_limits(self) -> None:
        """URDF permits an omitted limit element for continuous joints."""
        with tempfile.TemporaryDirectory() as directory:
            output = str(Path(directory) / "robot.urdf")
            write_urdf(
                "robot",
                [LinkData(name="a"), LinkData(name="b")],
                [JointData(name="hinge", joint_type="continuous", parent_link="a", child_link="b")],
                [],
                output,
            )
            self.assertIsNone(ET.parse(output).find(".//limit"))
            if shutil.which("check_urdf"):
                parsed = subprocess.run(["check_urdf", output], capture_output=True, text=True)
                self.assertEqual(parsed.returncode, 0, parsed.stdout + parsed.stderr)

    def test_expanded_axis_effort_units(self) -> None:
        """Expanded angular and linear axes retain their authored effort in SI units."""
        stage = Usd.Stage.CreateInMemory()
        UsdGeom.SetStageMetersPerUnit(stage, 0.01)
        UsdPhysics.SetStageKilogramsPerUnit(stage, 2.0)
        prim = UsdPhysics.Joint.Define(stage, "/joint").GetPrim()
        for token in ("rotX", "transZ"):
            limit = UsdPhysics.LimitAPI.Apply(prim, token)
            limit.CreateLowAttr(-10)
            limit.CreateHighAttr(10)
            UsdPhysics.DriveAPI.Apply(prim, token).CreateMaxForceAttr(100)
        prim.CreateAttribute("newton:velocityLimit", Sdf.ValueTypeNames.Float).Set(180)
        joints, _ = _expand_multi_dof_joint(
            prim, JointData(name="joint", joint_type="d6", parent_link="a", child_link="b")
        )
        axes = {joint.name: joint for joint in joints}
        self.assertAlmostEqual(axes["joint_rotX"].limit_effort, 0.02)
        self.assertAlmostEqual(axes["joint_transZ"].limit_effort, 2.0)
        self.assertAlmostEqual(axes["joint_rotX"].limit_velocity, math.pi)
        self.assertAlmostEqual(axes["joint_transZ"].limit_velocity, 1.8)

    def test_unauthored_units_use_usd_centimeter_default(self) -> None:
        """Unauthored stage units remain centimeters rather than being guessed as meters."""
        stage = Usd.Stage.CreateInMemory()
        self.assertFalse(UsdGeom.StageHasAuthoredMetersPerUnit(stage))
        prim = UsdPhysics.PrismaticJoint.Define(stage, "/joint").GetPrim()
        joint = UsdPhysics.PrismaticJoint(prim)
        joint.CreateLowerLimitAttr(-100)
        joint.CreateUpperLimitAttr(100)
        data = JointData(joint_type="prismatic")
        _read_limits(prim, data)
        self.assertEqual((data.limit_lower, data.limit_upper), (-1.0, 1.0))

    def test_drive_breadcrumbs_use_si_stage_units(self) -> None:
        """Keep serialized drive metadata consistent with the exported SI limit."""
        for angular, acceleration in itertools.product((False, True), repeat=2):
            with self.subTest(angular=angular, acceleration=acceleration), tempfile.TemporaryDirectory() as directory:
                source = Usd.Stage.CreateInMemory()
                UsdGeom.SetStageMetersPerUnit(source, 0.01)
                UsdPhysics.SetStageKilogramsPerUnit(source, 2.0)
                schema = UsdPhysics.RevoluteJoint if angular else UsdPhysics.PrismaticJoint
                prim = schema.Define(source, "/joint").GetPrim()
                instance = "angular" if angular else "linear"
                drive = UsdPhysics.DriveAPI.Apply(prim, instance)
                drive.CreateTypeAttr("acceleration" if acceleration else "force")
                drive.CreateMaxForceAttr(100)
                drive.CreateStiffnessAttr(50)
                drive.CreateDampingAttr(10)
                drive.CreateTargetPositionAttr(30)
                prim.CreateAttribute("physxJoint:armature", Sdf.ValueTypeNames.Float).Set(20)
                joint = JointData(name="joint", joint_type="revolute" if angular else "prismatic")
                _read_limits(prim, joint)
                joint.limit_lower, joint.limit_upper, joint.limit_velocity = -1, 1, 2
                joint.source_drive = _collect_source_drive_breadcrumb(prim, None)
                output = str(Path(directory) / "robot.urdf")
                write_urdf("robot", [], [joint], [], output)
                metadata = _read_breadcrumb(output, "isaac:source_drive")
                restored = metadata["drive"]
                effort = float(ET.parse(output).find("./joint/limit").get("effort"))
                self.assertAlmostEqual(restored["max_force"], effort)
                inertia_scale = 0.0002 if angular else 2.0
                gain_scale = 1.0 if acceleration else inertia_scale
                self.assertAlmostEqual(restored["stiffness"], 50 * gain_scale)
                self.assertAlmostEqual(restored["damping"], 10 * gain_scale)
                self.assertAlmostEqual(restored["target_position"], 30 if angular else 0.3)
                self.assertAlmostEqual(metadata["armature"], 20 * inertia_scale)

    def test_multi_dof_breadcrumbs_use_si_stage_units(self) -> None:
        """Serialize metric D6 poses, translational bounds, and per-axis drives."""
        source = Usd.Stage.CreateInMemory()
        UsdGeom.SetStageMetersPerUnit(source, 0.01)
        UsdPhysics.SetStageKilogramsPerUnit(source, 2.0)
        source_joint = UsdPhysics.Joint.Define(source, "/joint")
        source_joint.CreateLocalPos0Attr(Gf.Vec3f(100, 0, 0))
        source_joint.CreateLocalPos1Attr(Gf.Vec3f(0, 50, 0))
        prim = source_joint.GetPrim()
        for token in ("rotX", "transZ"):
            limit = UsdPhysics.LimitAPI.Apply(prim, token)
            limit.CreateLowAttr(-100)
            limit.CreateHighAttr(100)
            drive = UsdPhysics.DriveAPI.Apply(prim, token)
            drive.CreateMaxForceAttr(100)
            drive.CreateStiffnessAttr(50)
        prim.CreateAttribute("urdf:limit:velocity", Sdf.ValueTypeNames.Float).Set(2)
        joints, ghosts = _expand_multi_dof_joint(
            prim, JointData(name="joint", joint_type="d6", parent_link="a", child_link="b")
        )
        with tempfile.TemporaryDirectory() as directory:
            output = str(Path(directory) / "robot.urdf")
            write_urdf("robot", ghosts, joints, [], output)
            metadata = _read_breadcrumb(output, "isaac:source_joint")
        self.assertEqual(metadata["local_pos0"], [1, 0, 0])
        self.assertEqual(metadata["local_pos1"], [0, 0.5, 0])
        for token, bound, effort, stiffness in (("rotX", 100, 0.02, 0.01), ("transZ", 1, 2, 100)):
            limit = metadata["per_axis_limits"][token]
            self.assertEqual(limit["low"], -bound)
            self.assertEqual(limit["high"], bound)
            drive = metadata["per_axis_drives"][token]
            self.assertAlmostEqual(drive["max_force"], effort)
            self.assertAlmostEqual(drive["stiffness"], stiffness)


def _read_breadcrumb(output: str, prefix: str) -> dict:
    """Parse a serialized source metadata comment.

    Args:
        output: Exported URDF path.
        prefix: Metadata comment prefix to match.

    Returns:
        Deserialized metadata from the matching comment.
    """
    parser = ET.XMLParser(target=ET.TreeBuilder(insert_comments=True))
    root = ET.parse(output, parser=parser).getroot()
    for element in root.iter():
        if element.tag is ET.Comment and element.text.strip().startswith(prefix):
            return json.loads(element.text.strip()[len(prefix) :])
    raise AssertionError(f"Missing {prefix} metadata in {output}")


if __name__ == "__main__":
    unittest.main()
