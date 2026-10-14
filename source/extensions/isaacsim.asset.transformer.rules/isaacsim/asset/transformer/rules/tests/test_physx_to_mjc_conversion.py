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

"""Tests for PhysxToMjcConversionRule."""

import os
import shutil
import tempfile

import omni.kit.test
from isaacsim.asset.importer.utils.impl.physx_asset_to_mjc import (
    convert_physx_asset_to_mjc,
    make_delta_stage,
    warn_unconverted_multi_dof_joints,
)
from isaacsim.asset.transformer.rules.isaac_sim.physx_to_mjc_conversion import PhysxToMjcConversionRule
from pxr import PhysxSchema, Sdf, Usd, UsdGeom, UsdPhysics

_CONV_LOGGER = "isaacsim.asset.importer.utils.impl.physx_asset_to_mjc"


def _build_physx_stage() -> Usd.Stage:
    """Build a stage with revolute/prismatic joints, a mimic, and an articulation root.

    Returns:
        In-memory stage populated with the conversion fixture.
    """
    stage = Usd.Stage.CreateInMemory()
    root = UsdGeom.Xform.Define(stage, "/robot").GetPrim()
    stage.SetDefaultPrim(root)

    UsdPhysics.ArticulationRootAPI.Apply(root)
    PhysxSchema.PhysxArticulationAPI.Apply(root)
    root.CreateAttribute("physxArticulation:enabledSelfCollisions", Sdf.ValueTypeNames.Bool).Set(False)

    for name in ("base", "link1", "link2"):
        UsdPhysics.RigidBodyAPI.Apply(stage.DefinePrim(f"/robot/{name}", "Xform"))

    rev = UsdPhysics.RevoluteJoint.Define(stage, "/robot/revolute_j")
    rev.CreateBody0Rel().SetTargets([Sdf.Path("/robot/base")])
    rev.CreateBody1Rel().SetTargets([Sdf.Path("/robot/link1")])
    rp = rev.GetPrim()
    drive = UsdPhysics.DriveAPI.Apply(rp, "angular")
    drive.CreateStiffnessAttr().Set(100.0)
    drive.CreateDampingAttr().Set(10.0)
    drive.CreateTargetPositionAttr().Set(0.25)
    PhysxSchema.PhysxJointAPI.Apply(rp)
    rp.CreateAttribute("physxJoint:jointFriction", Sdf.ValueTypeNames.Float).Set(0.1)
    rp.CreateAttribute("physxJoint:armature", Sdf.ValueTypeNames.Float).Set(0.05)

    pri = UsdPhysics.PrismaticJoint.Define(stage, "/robot/prismatic_j")
    pri.CreateBody0Rel().SetTargets([Sdf.Path("/robot/link1")])
    pri.CreateBody1Rel().SetTargets([Sdf.Path("/robot/link2")])
    pp = pri.GetPrim()
    d2 = UsdPhysics.DriveAPI.Apply(pp, "linear")
    d2.CreateStiffnessAttr().Set(200.0)
    d2.CreateDampingAttr().Set(5.0)

    # Prismatic mimics the revolute joint (PhysxMimicJointAPI on the follower).
    PhysxSchema.PhysxMimicJointAPI.Apply(pp, "transX")
    pp.CreateRelationship("physxMimicJoint:transX:referenceJoint").SetTargets([Sdf.Path("/robot/revolute_j")])
    pp.CreateAttribute("physxMimicJoint:transX:gearing", Sdf.ValueTypeNames.Float).Set(2.0)
    pp.CreateAttribute("physxMimicJoint:transX:offset", Sdf.ValueTypeNames.Float).Set(0.1)

    return stage


def _build_multi_dof_stage() -> Usd.Stage:
    """Build a stage with a generic D6 joint and a spherical joint.

    Returns:
        In-memory stage populated with the multi-DOF fixture.
    """
    stage = Usd.Stage.CreateInMemory()
    root = UsdGeom.Xform.Define(stage, "/robot").GetPrim()
    stage.SetDefaultPrim(root)
    for name in ("base", "link1", "link2"):
        UsdPhysics.RigidBodyAPI.Apply(stage.DefinePrim(f"/robot/{name}", "Xform"))

    d6 = UsdPhysics.Joint.Define(stage, "/robot/d6_j")
    d6.CreateBody0Rel().SetTargets([Sdf.Path("/robot/base")])
    d6.CreateBody1Rel().SetTargets([Sdf.Path("/robot/link1")])
    UsdPhysics.LimitAPI.Apply(d6.GetPrim(), "rotX")
    UsdPhysics.DriveAPI.Apply(d6.GetPrim(), "rotX")

    sph = UsdPhysics.SphericalJoint.Define(stage, "/robot/spherical_j")
    sph.CreateBody0Rel().SetTargets([Sdf.Path("/robot/link1")])
    sph.CreateBody1Rel().SetTargets([Sdf.Path("/robot/link2")])

    return stage


class TestPhysxToMjcConversionRule(omni.kit.test.AsyncTestCase):
    """Async tests for PhysxToMjcConversionRule."""

    async def setUp(self) -> None:
        """Create a temporary output directory."""
        self._tmpdir = tempfile.mkdtemp()
        self._success = False

    async def tearDown(self) -> None:
        """Remove the temporary directory after a successful test."""
        if self._success:
            shutil.rmtree(self._tmpdir, ignore_errors=True)

    def _create_rule(
        self, stage: Usd.Stage, destination_path: str = "", params: dict | None = None
    ) -> PhysxToMjcConversionRule:
        return PhysxToMjcConversionRule(
            source_stage=stage,
            package_root=self._tmpdir,
            destination_path=destination_path,
            args={"params": params or {}},
        )

    async def test_empty_stage_is_noop(self) -> None:
        """The rule completes without error on an empty stage."""
        stage = Usd.Stage.CreateInMemory()
        stage.SetDefaultPrim(UsdGeom.Xform.Define(stage, "/World").GetPrim())

        rule = self._create_rule(stage)
        self.assertIsNone(rule.process_rule())
        self.assertIn("PhysxToMjcConversionRule completed", rule.get_operation_log())
        self._success = True

    async def test_revolute_gets_mjc_joint_api(self) -> None:
        """Revolute joints get MjcJointAPI applied."""
        stage = _build_physx_stage()
        self._create_rule(stage).process_rule()

        joint = stage.GetPrimAtPath("/robot/revolute_j")
        self.assertIn("MjcJointAPI", joint.GetAppliedSchemas())
        self._success = True

    async def test_mjc_actuator_created_with_gains(self) -> None:
        """A position-control MjcActuator is created with the expected gain/bias."""
        stage = _build_physx_stage()
        self._create_rule(stage).process_rule()

        actuator = stage.GetPrimAtPath("/robot/Physics/revolute_j_actuator")
        self.assertTrue(actuator.IsValid())
        self.assertEqual(actuator.GetTypeName(), "MjcActuator")
        self.assertEqual(str(actuator.GetRelationship("mjc:target").GetTargets()[0]), "/robot/revolute_j")

        gain_prm = list(actuator.GetAttribute("mjc:gainPrm").Get())
        bias_prm = list(actuator.GetAttribute("mjc:biasPrm").Get())
        self.assertAlmostEqual(gain_prm[0], 100.0)
        self.assertAlmostEqual(bias_prm[1], -100.0)
        self.assertAlmostEqual(bias_prm[2], -10.0)
        self.assertEqual(actuator.GetAttribute("mjc:gainType").Get(), "fixed")
        self.assertEqual(actuator.GetAttribute("mjc:biasType").Get(), "affine")
        self._success = True

    async def test_missing_source_gains_report_new_actuators(self) -> None:
        """Report gain review separately from structural success and existing motors."""
        stage = _build_physx_stage()
        for joint_name, drive_type in (("revolute_j", "angular"), ("prismatic_j", "linear")):
            joint = stage.GetPrimAtPath(f"/robot/{joint_name}")
            drive = UsdPhysics.DriveAPI(joint, drive_type)
            drive.GetStiffnessAttr().Clear()
            drive.GetDampingAttr().Clear()
        stage.DefinePrim("/robot/Physics/existing_motor", "MjcActuator")

        rule = self._create_rule(stage)
        rule.process_rule()
        log = "\n".join(rule.get_operation_log())
        self.assertIn("MuJoCo actuators generated: 2; new actuators without authored gain/bias: 2", log)
        self.assertIn("Control readiness warning: 2 new actuator(s)", log)
        self.assertIn("PhysxToMjcConversionRule completed", log)
        self._success = True

    async def test_overlay_with_gains_has_no_readiness_warning(self) -> None:
        """Authored PhysX gains produce complete new actuators in an overlay."""
        stage = _build_physx_stage()
        rule = self._create_rule(stage, destination_path="converted.usda")
        rule.process_rule()
        log = "\n".join(rule.get_operation_log())
        self.assertIn("MuJoCo actuators generated: 2; new actuators without authored gain/bias: 0", log)
        self.assertNotIn("Control readiness warning:", log)
        self._success = True

    async def test_physx_joint_attrs_to_mjc(self) -> None:
        """PhysX friction/armature and drive target become mjc:* attributes."""
        stage = _build_physx_stage()
        self._create_rule(stage).process_rule()

        joint = stage.GetPrimAtPath("/robot/revolute_j")
        self.assertAlmostEqual(joint.GetAttribute("mjc:frictionloss").Get(), 0.1)
        self.assertAlmostEqual(joint.GetAttribute("mjc:armature").Get(), 0.05)
        # mjc:ref is the joint zero point, not a drive target, so the drive targetPosition is not copied to it.
        self.assertFalse(joint.GetAttribute("mjc:ref").HasAuthoredValue())
        self._success = True

    async def test_prismatic_actuator_created(self) -> None:
        """Prismatic joints are processed into MjcActuators."""
        stage = _build_physx_stage()
        self._create_rule(stage).process_rule()

        actuator = stage.GetPrimAtPath("/robot/Physics/prismatic_j_actuator")
        self.assertTrue(actuator.IsValid())
        self.assertEqual(actuator.GetTypeName(), "MjcActuator")
        self._success = True

    async def test_physics_scope_created(self) -> None:
        """A Physics scope is created under the default prim."""
        stage = _build_physx_stage()
        self._create_rule(stage).process_rule()

        self.assertTrue(stage.GetPrimAtPath("/robot/Physics").IsValid())
        self._success = True

    async def test_duplicate_leaf_joint_names_get_unique_actuators(self) -> None:
        """Joints sharing a leaf name get distinct actuators (no silent overwrite/skip)."""
        stage = Usd.Stage.CreateInMemory()
        root = UsdGeom.Xform.Define(stage, "/robot").GetPrim()
        stage.SetDefaultPrim(root)
        joint_paths = ["/robot/arm/joint1", "/robot/leg/joint1"]
        for path in joint_paths:
            joint = UsdPhysics.RevoluteJoint.Define(stage, path).GetPrim()
            drive = UsdPhysics.DriveAPI.Apply(joint, "angular")
            drive.CreateStiffnessAttr().Set(100.0)
            drive.CreateDampingAttr().Set(10.0)

        self._create_rule(stage).process_rule()

        # Each source joint owns exactly one actuator, and the actuators are distinct.
        actuators_by_target = {}
        for prim in stage.Traverse():
            if prim.GetTypeName() != "MjcActuator":
                continue
            targets = prim.GetRelationship("mjc:target").GetTargets()
            self.assertEqual(len(targets), 1)
            actuators_by_target[str(targets[0])] = prim.GetPath().pathString

        self.assertEqual(set(actuators_by_target), set(joint_paths))
        self.assertEqual(len(set(actuators_by_target.values())), 2)
        self._success = True

    async def test_mimic_to_newton(self) -> None:
        """PhysxMimicJointAPI is ported to NewtonMimicAPI with the reference joint."""
        stage = _build_physx_stage()
        self._create_rule(stage).process_rule()

        follower = stage.GetPrimAtPath("/robot/prismatic_j")
        self.assertTrue(follower.HasAPI("NewtonMimicAPI"))
        targets = follower.GetRelationship("newton:mimicJoint").GetTargets()
        self.assertEqual(str(targets[0]), "/robot/revolute_j")
        # Fixture sets gearing=2.0, offset=0.1 (mapped to coef0/coef1 respectively).
        self.assertAlmostEqual(follower.GetAttribute("newton:mimicCoef0").Get(), 2.0)
        self.assertAlmostEqual(follower.GetAttribute("newton:mimicCoef1").Get(), 0.1)
        self._success = True

    async def test_articulation_root_to_newton(self) -> None:
        """PhysxArticulationAPI roots gain NewtonArticulationRootAPI."""
        stage = _build_physx_stage()
        self._create_rule(stage).process_rule()

        root = stage.GetPrimAtPath("/robot")
        self.assertTrue(root.HasAPI("NewtonArticulationRootAPI"))
        self.assertEqual(root.GetAttribute("newton:selfCollisionEnabled").Get(), False)
        self._success = True

    async def test_multi_dof_joints_not_converted(self) -> None:
        """D6 and spherical joints are warned about and left untouched."""
        stage = _build_multi_dof_stage()
        rule = self._create_rule(stage)

        with self.assertLogs(_CONV_LOGGER, level="WARNING") as cm:
            rule.process_rule()

        joined = "\n".join(cm.output)
        self.assertIn("d6_j", joined)
        self.assertIn("spherical_j", joined)

        # D6 stays active and is not expanded into per-axis joints.
        self.assertTrue(stage.GetPrimAtPath("/robot/d6_j").IsActive())
        self.assertFalse(stage.GetPrimAtPath("/robot/d6_j_rotX").IsValid())
        self.assertNotIn("MjcJointAPI", stage.GetPrimAtPath("/robot/d6_j").GetAppliedSchemas())
        self._success = True

    async def test_warn_unconverted_multi_dof_joints_count(self) -> None:
        """The warning helper reports the number of skipped multi-DOF joints."""
        stage = _build_multi_dof_stage()
        self.assertEqual(warn_unconverted_multi_dof_joints(stage), 2)
        self._success = True

    async def test_get_configuration_parameters(self) -> None:
        """PhysxToMjcConversionRule exposes no custom parameters (uses framework input/output)."""
        rule = self._create_rule(Usd.Stage.CreateInMemory())
        self.assertEqual(rule.get_configuration_parameters(), [])
        self._success = True

    async def test_output_layer_leaves_source_untouched(self) -> None:
        """With a distinct output layer, results go there and the source file is untouched."""
        src_path = os.path.join(self._tmpdir, "source.usda")
        _build_physx_stage().Export(src_path)
        source = Usd.Stage.Open(src_path)

        rule = self._create_rule(source, destination_path="out.usda")
        out_path = rule.process_rule()

        self.assertEqual(out_path, os.path.join(self._tmpdir, "out.usda"))
        self.assertTrue(os.path.exists(out_path))

        # The composed overlay carries the conversion output.
        composed = Usd.Stage.Open(out_path)
        self.assertTrue(composed.GetPrimAtPath("/robot/Physics/revolute_j_actuator").IsValid())

        # The source layer itself has no authored conversion opinions.
        src_layer = Sdf.Layer.FindOrOpen(src_path)
        self.assertFalse(src_layer.GetPrimAtPath("/robot/Physics/revolute_j_actuator"))
        self.assertNotIn("MjcJointAPI", source.GetPrimAtPath("/robot/revolute_j").GetAppliedSchemas())
        self._success = True

    async def test_delta_stage_overlay_leaves_source_untouched(self) -> None:
        """A valid delta_stage receives all edits, leaving the source layer clean."""
        stage = _build_physx_stage()
        delta = make_delta_stage(stage)

        convert_physx_asset_to_mjc(stage, delta_stage=delta)

        # Edits land on the delta root layer, not the source root layer.
        self.assertFalse(stage.GetRootLayer().GetPrimAtPath("/robot/Physics/revolute_j_actuator"))
        self.assertTrue(delta.GetPrimAtPath("/robot/Physics/revolute_j_actuator").IsValid())
        self._success = True

    async def test_unrelated_delta_stage_raises(self) -> None:
        """A delta_stage that does not overlay the source raises ValueError."""
        stage = _build_physx_stage()
        unrelated = Usd.Stage.CreateInMemory()

        with self.assertRaises(ValueError):
            convert_physx_asset_to_mjc(stage, delta_stage=unrelated)
        self._success = True
