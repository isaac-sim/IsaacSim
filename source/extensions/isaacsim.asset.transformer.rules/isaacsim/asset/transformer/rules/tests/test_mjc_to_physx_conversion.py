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

"""Tests for MjcToPhysxConversionRule."""

import shutil
import tempfile

import omni.kit.test
from isaacsim.asset.transformer.rules.isaac_sim.mjc_to_physx_conversion import MjcToPhysxConversionRule
from pxr import Sdf, Usd, UsdGeom, UsdPhysics


def _build_mjc_stage() -> Usd.Stage:
    """Build an in-memory stage with MJCF actuator and joint prims."""
    stage = Usd.Stage.CreateInMemory()
    root = UsdGeom.Xform.Define(stage, "/robot").GetPrim()
    stage.SetDefaultPrim(root)

    base = stage.DefinePrim("/robot/base", "Xform")
    UsdPhysics.RigidBodyAPI.Apply(base)
    link = stage.DefinePrim("/robot/link1", "Xform")
    UsdPhysics.RigidBodyAPI.Apply(link)

    joint = UsdPhysics.RevoluteJoint.Define(stage, "/robot/joint1")
    joint.CreateBody0Rel().SetTargets([Sdf.Path("/robot/base")])
    joint.CreateBody1Rel().SetTargets([Sdf.Path("/robot/link1")])
    drive = UsdPhysics.DriveAPI.Apply(joint.GetPrim(), "angular")
    drive.CreateStiffnessAttr().Set(100.0)
    drive.CreateDampingAttr().Set(10.0)

    actuator = stage.DefinePrim("/robot/Physics/joint1_actuator", "MjcActuator")
    actuator.CreateRelationship("mjc:target", custom=False).SetTargets([Sdf.Path("/robot/joint1")])
    actuator.CreateAttribute("mjc:gainType", Sdf.ValueTypeNames.String).Set("fixed")
    actuator.CreateAttribute("mjc:biasType", Sdf.ValueTypeNames.String).Set("affine")
    actuator.CreateAttribute("mjc:gainPrm", Sdf.ValueTypeNames.FloatArray).Set([100.0, 0, 0, 0, 0, 0, 0, 0, 0, 0])
    actuator.CreateAttribute("mjc:biasPrm", Sdf.ValueTypeNames.FloatArray).Set([0, -100.0, -10.0, 0, 0, 0, 0, 0, 0, 0])
    actuator.CreateAttribute("mjc:forceRange:max", Sdf.ValueTypeNames.Float).Set(50.0)
    actuator.CreateAttribute("mjc:forceRange:min", Sdf.ValueTypeNames.Float).Set(-50.0)

    joint_prim = joint.GetPrim()
    joint_prim.CreateAttribute("mjc:frictionloss", Sdf.ValueTypeNames.Float).Set(0.5)
    joint_prim.CreateAttribute("mjc:armature", Sdf.ValueTypeNames.Float).Set(0.01)
    joint_prim.CreateAttribute("mjc:ref", Sdf.ValueTypeNames.Float).Set(1.57)

    return stage


class TestMjcToPhysxConversionRule(omni.kit.test.AsyncTestCase):
    """Async tests for MjcToPhysxConversionRule."""

    async def setUp(self) -> None:
        """Create a temporary directory for test output."""
        self._tmpdir = tempfile.mkdtemp()
        self._success = False

    async def tearDown(self) -> None:
        """Remove temporary directory after successful tests."""
        if self._success:
            shutil.rmtree(self._tmpdir, ignore_errors=True)

    def _create_rule(self, stage: Usd.Stage) -> MjcToPhysxConversionRule:
        return MjcToPhysxConversionRule(
            source_stage=stage,
            package_root=self._tmpdir,
            destination_path="",
            args={"params": {}},
        )

    async def test_empty_stage_is_noop(self) -> None:
        """Rule should complete without error on an empty stage."""
        stage = Usd.Stage.CreateInMemory()
        UsdGeom.Xform.Define(stage, "/World")
        stage.SetDefaultPrim(stage.GetPrimAtPath("/World"))

        rule = self._create_rule(stage)
        result = rule.process_rule()

        self.assertIsNone(result)
        log = rule.get_operation_log()
        self.assertIn("MjcToPhysxConversionRule completed", log)
        self._success = True

    async def test_actuator_gains_converted_to_drive(self) -> None:
        """MJCF actuator gain/bias should be converted to PhysX drive stiffness/damping."""
        stage = _build_mjc_stage()

        rule = self._create_rule(stage)
        rule.process_rule()

        joint = stage.GetPrimAtPath("/robot/joint1")
        drive = UsdPhysics.DriveAPI(joint, "angular")
        self.assertAlmostEqual(drive.GetStiffnessAttr().Get(), 100.0)
        self.assertAlmostEqual(drive.GetDampingAttr().Get(), 10.0)
        self._success = True

    async def test_max_force_set_from_actuator(self) -> None:
        """MJCF force range should be converted to drive max force."""
        stage = _build_mjc_stage()

        rule = self._create_rule(stage)
        rule.process_rule()

        joint = stage.GetPrimAtPath("/robot/joint1")
        drive = UsdPhysics.DriveAPI(joint, "angular")
        self.assertAlmostEqual(drive.GetMaxForceAttr().Get(), 50.0)
        self._success = True

    async def test_joint_friction_converted(self) -> None:
        """MJCF frictionloss should be converted to PhysX joint friction."""
        stage = _build_mjc_stage()

        rule = self._create_rule(stage)
        rule.process_rule()

        joint = stage.GetPrimAtPath("/robot/joint1")
        from pxr import PhysxSchema

        physx_joint = PhysxSchema.PhysxJointAPI(joint)
        self.assertAlmostEqual(physx_joint.GetJointFrictionAttr().Get(), 0.5)
        self._success = True

    async def test_joint_armature_converted(self) -> None:
        """MJCF armature should be converted to PhysX joint armature."""
        stage = _build_mjc_stage()

        rule = self._create_rule(stage)
        rule.process_rule()

        joint = stage.GetPrimAtPath("/robot/joint1")
        from pxr import PhysxSchema

        physx_joint = PhysxSchema.PhysxJointAPI(joint)
        self.assertAlmostEqual(physx_joint.GetArmatureAttr().Get(), 0.01)
        self._success = True

    async def test_target_position_from_ref(self) -> None:
        """MJCF ref attribute should be converted to drive target position."""
        stage = _build_mjc_stage()

        rule = self._create_rule(stage)
        rule.process_rule()

        joint = stage.GetPrimAtPath("/robot/joint1")
        drive = UsdPhysics.DriveAPI(joint, "angular")
        self.assertAlmostEqual(drive.GetTargetPositionAttr().Get(), 1.57, places=2)
        self._success = True

    async def test_get_configuration_parameters_empty(self) -> None:
        """MjcToPhysxConversionRule should have no configuration parameters."""
        stage = Usd.Stage.CreateInMemory()
        rule = self._create_rule(stage)
        self.assertEqual(rule.get_configuration_parameters(), [])
        self._success = True
