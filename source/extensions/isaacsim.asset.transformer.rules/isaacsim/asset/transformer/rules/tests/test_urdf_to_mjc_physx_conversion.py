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

"""Tests for UrdfToMjcPhysxConversionRule."""

import shutil
import tempfile

import omni.kit.test
from isaacsim.asset.transformer.rules.isaac_sim.urdf_to_mjc_physx_conversion import UrdfToMjcPhysxConversionRule
from pxr import Sdf, Usd, UsdGeom, UsdPhysics


def _build_urdf_stage() -> Usd.Stage:
    """Build an in-memory stage mimicking URDF-imported joint prims."""
    stage = Usd.Stage.CreateInMemory()
    root = UsdGeom.Xform.Define(stage, "/robot").GetPrim()
    stage.SetDefaultPrim(root)

    base = stage.DefinePrim("/robot/base", "Xform")
    UsdPhysics.RigidBodyAPI.Apply(base)
    link1 = stage.DefinePrim("/robot/link1", "Xform")
    UsdPhysics.RigidBodyAPI.Apply(link1)
    link2 = stage.DefinePrim("/robot/link2", "Xform")
    UsdPhysics.RigidBodyAPI.Apply(link2)

    joint1 = UsdPhysics.RevoluteJoint.Define(stage, "/robot/revolute_j")
    joint1.CreateBody0Rel().SetTargets([Sdf.Path("/robot/base")])
    joint1.CreateBody1Rel().SetTargets([Sdf.Path("/robot/link1")])
    j1_prim = joint1.GetPrim()
    j1_prim.CreateAttribute("urdf:limit:effort", Sdf.ValueTypeNames.Float).Set(87.0)
    j1_prim.CreateAttribute("urdf:limit:velocity", Sdf.ValueTypeNames.Float).Set(2.175)
    j1_prim.CreateAttribute("urdf:dynamics:damping", Sdf.ValueTypeNames.Float).Set(10.0)
    j1_prim.CreateAttribute("urdf:dynamics:friction", Sdf.ValueTypeNames.Float).Set(0.1)
    j1_prim.CreateAttribute("urdf:calibration:reference_position", Sdf.ValueTypeNames.Float).Set(0.5)

    drive1 = UsdPhysics.DriveAPI.Apply(j1_prim, "angular")
    drive1.CreateStiffnessAttr().Set(100.0)
    drive1.CreateDampingAttr().Set(10.0)

    joint2 = UsdPhysics.PrismaticJoint.Define(stage, "/robot/prismatic_j")
    joint2.CreateBody0Rel().SetTargets([Sdf.Path("/robot/link1")])
    joint2.CreateBody1Rel().SetTargets([Sdf.Path("/robot/link2")])
    j2_prim = joint2.GetPrim()
    j2_prim.CreateAttribute("urdf:limit:effort", Sdf.ValueTypeNames.Float).Set(50.0)
    j2_prim.CreateAttribute("urdf:dynamics:damping", Sdf.ValueTypeNames.Float).Set(5.0)

    drive2 = UsdPhysics.DriveAPI.Apply(j2_prim, "linear")
    drive2.CreateStiffnessAttr().Set(200.0)
    drive2.CreateDampingAttr().Set(5.0)

    return stage


class TestUrdfToMjcPhysxConversionRule(omni.kit.test.AsyncTestCase):
    """Async tests for UrdfToMjcPhysxConversionRule."""

    async def setUp(self) -> None:
        """Create a temporary directory for test output."""
        self._tmpdir = tempfile.mkdtemp()
        self._success = False

    async def tearDown(self) -> None:
        """Remove temporary directory after successful tests."""
        if self._success:
            shutil.rmtree(self._tmpdir, ignore_errors=True)

    def _create_rule(self, stage: Usd.Stage) -> UrdfToMjcPhysxConversionRule:
        return UrdfToMjcPhysxConversionRule(
            source_stage=stage,
            package_root=self._tmpdir,
            destination_path="",
            args={"params": {}},
        )

    async def test_empty_stage_is_noop(self) -> None:
        """Rule should complete without error on an empty stage."""
        stage = Usd.Stage.CreateInMemory()
        root = UsdGeom.Xform.Define(stage, "/World").GetPrim()
        stage.SetDefaultPrim(root)

        rule = self._create_rule(stage)
        result = rule.process_rule()

        self.assertIsNone(result)
        log = rule.get_operation_log()
        self.assertIn("UrdfToMjcPhysxConversionRule completed", log)
        self._success = True

    async def test_effort_limit_to_max_force(self) -> None:
        """URDF effort limit should become drive max force."""
        stage = _build_urdf_stage()

        rule = self._create_rule(stage)
        rule.process_rule()

        joint = stage.GetPrimAtPath("/robot/revolute_j")
        drive = UsdPhysics.DriveAPI(joint, "angular")
        self.assertAlmostEqual(drive.GetMaxForceAttr().Get(), 87.0)
        self._success = True

    async def test_velocity_limit_to_max_velocity(self) -> None:
        """URDF velocity limit should become PhysX max joint velocity (deg/s)."""
        stage = _build_urdf_stage()

        rule = self._create_rule(stage)
        rule.process_rule()

        joint = stage.GetPrimAtPath("/robot/revolute_j")
        from pxr import PhysxSchema

        physx_joint = PhysxSchema.PhysxJointAPI(joint)
        expected_deg = 2.175 * 180 / 3.1415926
        self.assertAlmostEqual(physx_joint.GetMaxJointVelocityAttr().Get(), expected_deg, places=2)
        self._success = True

    async def test_damping_set_on_drive(self) -> None:
        """URDF dynamics damping should be set on the drive API."""
        stage = _build_urdf_stage()

        rule = self._create_rule(stage)
        rule.process_rule()

        joint = stage.GetPrimAtPath("/robot/revolute_j")
        drive = UsdPhysics.DriveAPI(joint, "angular")
        self.assertAlmostEqual(drive.GetDampingAttr().Get(), 10.0)
        self._success = True

    async def test_friction_to_physx_joint_friction(self) -> None:
        """URDF dynamics friction should become PhysX joint friction."""
        stage = _build_urdf_stage()

        rule = self._create_rule(stage)
        rule.process_rule()

        joint = stage.GetPrimAtPath("/robot/revolute_j")
        from pxr import PhysxSchema

        physx_joint = PhysxSchema.PhysxJointAPI(joint)
        self.assertAlmostEqual(physx_joint.GetJointFrictionAttr().Get(), 0.1)
        self._success = True

    async def test_reference_position_to_target_position(self) -> None:
        """URDF calibration reference position should become drive target position."""
        stage = _build_urdf_stage()

        rule = self._create_rule(stage)
        rule.process_rule()

        joint = stage.GetPrimAtPath("/robot/revolute_j")
        drive = UsdPhysics.DriveAPI(joint, "angular")
        self.assertAlmostEqual(drive.GetTargetPositionAttr().Get(), 0.5, places=2)
        self._success = True

    async def test_mjc_actuator_created(self) -> None:
        """An MjcActuator prim should be created under the Physics scope."""
        stage = _build_urdf_stage()

        rule = self._create_rule(stage)
        rule.process_rule()

        actuator = stage.GetPrimAtPath("/robot/Physics/revolute_j_actuator")
        self.assertTrue(actuator.IsValid())
        self.assertEqual(actuator.GetTypeName(), "MjcActuator")

        targets = actuator.GetRelationship("mjc:target").GetTargets()
        self.assertEqual(len(targets), 1)
        self.assertEqual(str(targets[0]), "/robot/revolute_j")
        self._success = True

    async def test_mjc_actuator_gain_params(self) -> None:
        """MjcActuator should have correct gain/bias parameters for position control."""
        stage = _build_urdf_stage()

        rule = self._create_rule(stage)
        rule.process_rule()

        actuator = stage.GetPrimAtPath("/robot/Physics/revolute_j_actuator")
        gain_prm = list(actuator.GetAttribute("mjc:gainPrm").Get())
        bias_prm = list(actuator.GetAttribute("mjc:biasPrm").Get())

        self.assertAlmostEqual(gain_prm[0], 100.0)
        self.assertAlmostEqual(bias_prm[1], -100.0)
        self.assertAlmostEqual(bias_prm[2], -10.0)
        self.assertEqual(actuator.GetAttribute("mjc:gainType").Get(), "fixed")
        self.assertEqual(actuator.GetAttribute("mjc:biasType").Get(), "affine")
        self._success = True

    async def test_mjc_ref_attribute_created(self) -> None:
        """Joint should get mjc:ref attribute from the PhysX target position."""
        stage = _build_urdf_stage()

        rule = self._create_rule(stage)
        rule.process_rule()

        joint = stage.GetPrimAtPath("/robot/revolute_j")
        ref_attr = joint.GetAttribute("mjc:ref")
        self.assertTrue(ref_attr.IsValid())
        self.assertAlmostEqual(ref_attr.Get(), 0.5, places=2)
        self._success = True

    async def test_prismatic_joint_processed(self) -> None:
        """Prismatic joints should also be processed."""
        stage = _build_urdf_stage()

        rule = self._create_rule(stage)
        rule.process_rule()

        actuator = stage.GetPrimAtPath("/robot/Physics/prismatic_j_actuator")
        self.assertTrue(actuator.IsValid())
        self.assertEqual(actuator.GetTypeName(), "MjcActuator")
        self._success = True

    async def test_physics_scope_created(self) -> None:
        """A Physics scope should be created under the default prim."""
        stage = _build_urdf_stage()

        rule = self._create_rule(stage)
        rule.process_rule()

        scope = stage.GetPrimAtPath("/robot/Physics")
        self.assertTrue(scope.IsValid())
        self._success = True

    async def test_get_configuration_parameters_empty(self) -> None:
        """UrdfToMjcPhysxConversionRule should have no configuration parameters."""
        stage = Usd.Stage.CreateInMemory()
        rule = self._create_rule(stage)
        self.assertEqual(rule.get_configuration_parameters(), [])
        self._success = True
