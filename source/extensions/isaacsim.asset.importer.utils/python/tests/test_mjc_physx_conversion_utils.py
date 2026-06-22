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

"""Verifies conversion utilities between MJCF actuator metadata and PhysX drive schemas. Covers limit propagation, actuator creation, overconstrained joint consolidation, property preservation, layer overrides, and mimic reference rewrites."""

import os
import tempfile

import omni.kit.test
from isaacsim.asset.importer.utils.impl import mjc_to_physx_conversion_utils, urdf_to_mjc_physx_conversion_utils
from isaacsim.asset.importer.utils.impl.physx_types import PhysxAttr, PhysxSchema
from pxr import Gf, Sdf, Usd, UsdGeom, UsdPhysics


class TestMjcPhysxConversionUtils(omni.kit.test.AsyncTestCase):
    """Tests for MJCF/PhysX conversion utilities."""

    async def test_convert_urdf_to_physx_applies_limits(self) -> None:
        """Apply URDF joint limits to PhysX drive and joint APIs."""
        stage = Usd.Stage.CreateInMemory()
        UsdGeom.Xform.Define(stage, "/World")
        joint = UsdPhysics.RevoluteJoint.Define(stage, "/World/Joint").GetPrim()

        joint.CreateAttribute("urdf:limit:effort", Sdf.ValueTypeNames.Float).Set(120.0)
        joint.CreateAttribute("urdf:limit:velocity", Sdf.ValueTypeNames.Float).Set(6.0)
        joint.CreateAttribute("urdf:dynamics:damping", Sdf.ValueTypeNames.Float).Set(0.1)
        joint.CreateAttribute("urdf:dynamics:friction", Sdf.ValueTypeNames.Float).Set(0.2)
        joint.CreateAttribute("urdf:calibration:reference_position", Sdf.ValueTypeNames.Float).Set(10.0)
        urdf_to_mjc_physx_conversion_utils.convert_urdf_to_physx(joint)

        drive_api = UsdPhysics.DriveAPI(joint, "angular")
        self.assertTrue(drive_api.GetMaxForceAttr().IsValid())
        self.assertEqual(drive_api.GetMaxForceAttr().Get(), 120.0)

        self.assertTrue(drive_api.GetDampingAttr().IsValid())
        self.assertAlmostEqual(drive_api.GetDampingAttr().Get(), 0.1, delta=1e-2)

        self.assertTrue(drive_api.GetTargetPositionAttr().IsValid())
        self.assertAlmostEqual(drive_api.GetTargetPositionAttr().Get(), 10.0, delta=1e-2)

        max_vel_attr = joint.GetAttribute(PhysxAttr.JOINT_MAX_VELOCITY.name)
        self.assertTrue(max_vel_attr.IsValid())
        self.assertAlmostEqual(max_vel_attr.Get(), 6.0 * 180 / 3.1415926, delta=1e-2)

        friction_attr = joint.GetAttribute(PhysxAttr.JOINT_FRICTION.name)
        self.assertTrue(friction_attr.IsValid())
        self.assertAlmostEqual(friction_attr.Get(), 0.2, delta=1e-2)

    async def test_convert_physx_to_mjc_authors_mjc_attrs(self) -> None:
        """Author MJCF attributes from PhysX joint data."""
        stage = Usd.Stage.CreateInMemory()
        UsdGeom.Xform.Define(stage, "/World")
        joint = UsdPhysics.RevoluteJoint.Define(stage, "/World/Joint").GetPrim()

        drive_api = UsdPhysics.DriveAPI.Apply(joint, "angular")
        drive_api.CreateTargetPositionAttr().Set(1.25)

        joint.ApplyAPI(PhysxSchema.JOINT_API)
        joint.CreateAttribute(PhysxAttr.JOINT_FRICTION.name, PhysxAttr.JOINT_FRICTION.type).Set(0.4)
        joint.CreateAttribute(PhysxAttr.JOINT_ARMATURE.name, PhysxAttr.JOINT_ARMATURE.type).Set(0.02)

        urdf_to_mjc_physx_conversion_utils.convert_physx_to_mjc(joint)

        self.assertTrue(joint.GetAttribute("mjc:ref").IsValid())
        self.assertAlmostEqual(joint.GetAttribute("mjc:ref").Get(), 1.25, delta=1e-2)

        self.assertTrue(joint.GetAttribute("mjc:frictionloss").IsValid())
        self.assertAlmostEqual(joint.GetAttribute("mjc:frictionloss").Get(), 0.4, delta=1e-2)

        self.assertTrue(joint.GetAttribute("mjc:armature").IsValid())
        self.assertAlmostEqual(joint.GetAttribute("mjc:armature").Get(), 0.02, delta=1e-2)

    async def test_convert_joints_attributes_creates_actuator(self) -> None:
        """Create MJCF actuators and transfer joint data."""
        stage = Usd.Stage.CreateInMemory()
        default_prim = UsdGeom.Xform.Define(stage, "/World").GetPrim()
        stage.SetDefaultPrim(default_prim)

        joint = UsdPhysics.RevoluteJoint.Define(stage, "/World/Joint").GetPrim()
        drive_api = UsdPhysics.DriveAPI.Apply(joint, "angular")
        drive_api.CreateMaxForceAttr().Set(55.0)
        drive_api.CreateStiffnessAttr().Set(10.0)
        drive_api.CreateDampingAttr().Set(1.5)
        drive_api.CreateTargetPositionAttr().Set(0.5)

        joint.ApplyAPI(PhysxSchema.JOINT_API)
        joint.CreateAttribute(PhysxAttr.JOINT_FRICTION.name, PhysxAttr.JOINT_FRICTION.type).Set(0.2)

        urdf_to_mjc_physx_conversion_utils.convert_joints_attributes(stage)

        actuator_path = "/World/Physics/Joint_actuator"
        actuator = stage.GetPrimAtPath(actuator_path)
        self.assertTrue(actuator.IsValid())
        self.assertEqual(actuator.GetTypeName(), "MjcActuator")

        targets = actuator.GetRelationship("mjc:target").GetTargets()
        self.assertEqual(len(targets), 1)
        self.assertEqual(targets[0], joint.GetPath())

        self.assertTrue(actuator.GetAttribute("mjc:gainPrm").IsValid())
        self.assertTrue(actuator.GetAttribute("mjc:biasPrm").IsValid())

        self.assertTrue(joint.GetAttribute("mjc:ref").IsValid())
        self.assertAlmostEqual(joint.GetAttribute("mjc:ref").Get(), 0.5, delta=1e-2)

        self.assertTrue(joint.GetAttribute("mjc:frictionloss").IsValid())
        self.assertAlmostEqual(joint.GetAttribute("mjc:frictionloss").Get(), 0.2, delta=1e-2)

    async def test_convert_mjc_to_physx_updates_drive(self) -> None:
        """Convert MJCF actuator and joint attributes into PhysX APIs."""
        stage = Usd.Stage.CreateInMemory()
        UsdGeom.Xform.Define(stage, "/World")
        joint = UsdPhysics.RevoluteJoint.Define(stage, "/World/Joint").GetPrim()
        joint.CreateAttribute("mjc:frictionloss", Sdf.ValueTypeNames.Float).Set(0.3)
        joint.CreateAttribute("mjc:ref", Sdf.ValueTypeNames.Float).Set(0.75)

        actuator = stage.DefinePrim("/World/Actuator", "MjcActuator")
        actuator.CreateRelationship("mjc:target", custom=False).SetTargets([joint.GetPath()])
        actuator.CreateAttribute("mjc:gainType", Sdf.ValueTypeNames.String).Set("fixed")
        actuator.CreateAttribute("mjc:biasType", Sdf.ValueTypeNames.String).Set("affine")
        actuator.CreateAttribute("mjc:gainPrm", Sdf.ValueTypeNames.FloatArray).Set(
            [12.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        )
        actuator.CreateAttribute("mjc:biasPrm", Sdf.ValueTypeNames.FloatArray).Set(
            [0.0, -12.0, -3.5, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        )
        actuator.CreateAttribute("mjc:forceRange:max", Sdf.ValueTypeNames.Float).Set(88.0)
        actuator.CreateAttribute("mjc:forceRange:min", Sdf.ValueTypeNames.Float).Set(-88.0)

        mjc_to_physx_conversion_utils.convert_mjc_to_physx(stage)

        drive_api = UsdPhysics.DriveAPI.Get(joint, "angular")
        self.assertTrue(drive_api.GetStiffnessAttr().IsValid())
        self.assertAlmostEqual(drive_api.GetStiffnessAttr().Get(), 12.0, delta=1e-2)
        self.assertTrue(drive_api.GetDampingAttr().IsValid())
        self.assertAlmostEqual(drive_api.GetDampingAttr().Get(), 3.5, delta=1e-2)
        self.assertTrue(drive_api.GetMaxForceAttr().IsValid())
        self.assertAlmostEqual(drive_api.GetMaxForceAttr().Get(), 88.0, delta=1e-2)
        self.assertTrue(drive_api.GetTargetPositionAttr().IsValid())
        self.assertAlmostEqual(drive_api.GetTargetPositionAttr().Get(), 0.75, delta=1e-2)

        friction_attr = joint.GetAttribute(PhysxAttr.JOINT_FRICTION.name)
        self.assertTrue(friction_attr.IsValid())
        self.assertAlmostEqual(friction_attr.Get(), 0.3, delta=1e-2)

    async def test_create_mjc_actuator_from_physics_copies_drive_limits(self) -> None:
        """Create MJCF actuator attributes from PhysX drive limits."""
        stage = Usd.Stage.CreateInMemory()
        default_prim = UsdGeom.Xform.Define(stage, "/World").GetPrim()
        stage.SetDefaultPrim(default_prim)

        joint = UsdPhysics.RevoluteJoint.Define(stage, "/World/Joint").GetPrim()
        drive_api = UsdPhysics.DriveAPI.Apply(joint, "angular")
        drive_api.CreateMaxForceAttr().Set(42.0)
        drive_api.CreateStiffnessAttr().Set(2.5)
        drive_api.CreateDampingAttr().Set(0.4)
        urdf_to_mjc_physx_conversion_utils.create_mjc_actuator_from_physics(joint, stage, "/World/Physics")
        actuator = stage.GetPrimAtPath("/World/Physics/Joint_actuator")
        self.assertTrue(actuator.IsValid())
        self.assertEqual(actuator.GetTypeName(), "MjcActuator")

        self.assertTrue(actuator.GetAttribute("mjc:forceRange:max").IsValid())
        self.assertAlmostEqual(actuator.GetAttribute("mjc:forceRange:max").Get(), 42.0, delta=1e-2)
        self.assertTrue(actuator.GetAttribute("mjc:forceRange:min").IsValid())
        self.assertAlmostEqual(actuator.GetAttribute("mjc:forceRange:min").Get(), -42.0, delta=1e-2)

        self.assertTrue(actuator.GetAttribute("mjc:gainPrm").IsValid())
        self.assertTrue(actuator.GetAttribute("mjc:biasPrm").IsValid())
        gain_prm = actuator.GetAttribute("mjc:gainPrm").Get()
        bias_prm = actuator.GetAttribute("mjc:biasPrm").Get()
        self.assertAlmostEqual(gain_prm[0], 2.5, delta=1e-2)
        self.assertAlmostEqual(gain_prm[1], 0.0, delta=1e-2)
        self.assertAlmostEqual(gain_prm[2], 0.0, delta=1e-2)
        self.assertAlmostEqual(bias_prm[0], 0.0, delta=1e-2)
        self.assertAlmostEqual(bias_prm[1], -2.5, delta=1e-2)
        self.assertAlmostEqual(bias_prm[2], -0.4, delta=1e-2)

        self.assertTrue(actuator.GetAttribute("mjc:gainType").IsValid())
        self.assertEqual(actuator.GetAttribute("mjc:gainType").Get(), "fixed")
        self.assertTrue(actuator.GetAttribute("mjc:biasType").IsValid())
        self.assertEqual(actuator.GetAttribute("mjc:biasType").Get(), "affine")

    def _make_revolute_joint(
        self,
        stage: Usd.Stage,
        path: str,
        body0: str,
        body1: str,
        axis: str,
        lower: float,
        upper: float,
    ) -> Usd.Prim:
        """Construct a ``UsdPhysics.RevoluteJoint`` with the given parameters.

        Args:
            stage: Stage on which to define the joint.
            path: Prim path for the joint.
            body0: Target path for the joint's body0 relationship.
            body1: Target path for the joint's body1 relationship.
            axis: Joint axis token.
            lower: Lower limit authored on the joint.
            upper: Upper limit authored on the joint.

        Returns:
            The authored revolute joint prim.
        """
        joint = UsdPhysics.RevoluteJoint.Define(stage, path)
        joint.CreateBody0Rel().SetTargets([Sdf.Path(body0)])
        joint.CreateBody1Rel().SetTargets([Sdf.Path(body1)])
        joint.CreateAxisAttr().Set(axis)
        joint.CreateLowerLimitAttr().Set(lower)
        joint.CreateUpperLimitAttr().Set(upper)
        joint.CreateLocalPos0Attr().Set(Gf.Vec3f(0, 0, 0))
        joint.CreateLocalPos1Attr().Set(Gf.Vec3f(0, 0, 0))
        joint.CreateLocalRot0Attr().Set(Gf.Quatf(1, 0, 0, 0))
        joint.CreateLocalRot1Attr().Set(Gf.Quatf(1, 0, 0, 0))
        return joint.GetPrim()

    async def test_combine_overconstrained_joints_to_d6_combines_three_axes(self) -> None:
        """Three single-axis joints sharing a body pair collapse into one D6 joint."""
        stage = Usd.Stage.CreateInMemory()
        UsdGeom.Xform.Define(stage, "/World")
        UsdGeom.Xform.Define(stage, "/World/parent")
        UsdGeom.Xform.Define(stage, "/World/child")
        UsdPhysics.RigidBodyAPI.Apply(stage.GetPrimAtPath("/World/parent"))
        UsdPhysics.RigidBodyAPI.Apply(stage.GetPrimAtPath("/World/child"))

        joint_x = self._make_revolute_joint(
            stage, "/World/child/hip_x", "/World/parent", "/World/child", "X", -45.0, 15.0
        )
        joint_z = self._make_revolute_joint(
            stage, "/World/child/hip_z", "/World/parent", "/World/child", "Z", -60.0, 35.0
        )
        joint_y = self._make_revolute_joint(
            stage, "/World/child/hip_y", "/World/parent", "/World/child", "Y", -120.0, 45.0
        )

        UsdPhysics.DriveAPI.Apply(joint_x, "angular").CreateStiffnessAttr().Set(100.0)
        UsdPhysics.DriveAPI(joint_x, "angular").CreateDampingAttr().Set(5.0)

        joint_x.ApplyAPI(PhysxSchema.JOINT_API)
        joint_x.CreateAttribute(PhysxAttr.JOINT_ARMATURE.name, PhysxAttr.JOINT_ARMATURE.type).Set(0.01)

        converted = mjc_to_physx_conversion_utils.combine_overconstrained_joints_to_d6(stage)
        self.assertEqual(converted, 1)

        # Primary joint has been retyped to PhysicsJoint (D6 host); name preserved.
        d6_prim = stage.GetPrimAtPath("/World/child/hip_x")
        self.assertTrue(d6_prim.IsValid())
        self.assertEqual(d6_prim.GetTypeName(), "PhysicsJoint")
        self.assertTrue(UsdPhysics.Joint(d6_prim).GetJointEnabledAttr().Get())

        # Per-axis LimitAPIs are present and carry the original limits.
        for axis_token, low, high in (("rotX", -45.0, 15.0), ("rotY", -120.0, 45.0), ("rotZ", -60.0, 35.0)):
            self.assertTrue(d6_prim.HasAPI(UsdPhysics.LimitAPI, axis_token), axis_token)
            limit = UsdPhysics.LimitAPI(d6_prim, axis_token)
            self.assertAlmostEqual(limit.GetLowAttr().Get(), low, delta=1e-3)
            self.assertAlmostEqual(limit.GetHighAttr().Get(), high, delta=1e-3)

        # Drive properties from joint_x copied to the matching D6 axis.
        self.assertTrue(d6_prim.HasAPI(UsdPhysics.DriveAPI, "rotX"))
        rotx_drive = UsdPhysics.DriveAPI(d6_prim, "rotX")
        self.assertAlmostEqual(rotx_drive.GetStiffnessAttr().Get(), 100.0, delta=1e-3)
        self.assertAlmostEqual(rotx_drive.GetDampingAttr().Get(), 5.0, delta=1e-3)

        # PhysxJointAPI armature carried over from primary joint.
        self.assertTrue(d6_prim.HasAPI(PhysxSchema.JOINT_API))
        self.assertAlmostEqual(d6_prim.GetAttribute(PhysxAttr.JOINT_ARMATURE.name).Get(), 0.01, delta=1e-3)

        # Other joints removed from the PhysX variant via ``active = false``.
        for path in ("/World/child/hip_y", "/World/child/hip_z"):
            other = stage.GetPrimAtPath(path)
            self.assertTrue(other.IsValid(), path)
            self.assertFalse(other.IsActive(), path)

    async def test_combine_overconstrained_joints_to_d6_skips_singletons(self) -> None:
        """Body pairs with a single joint should be left unmodified."""
        stage = Usd.Stage.CreateInMemory()
        UsdGeom.Xform.Define(stage, "/World")
        UsdGeom.Xform.Define(stage, "/World/parent")
        UsdGeom.Xform.Define(stage, "/World/child")

        self._make_revolute_joint(stage, "/World/child/knee", "/World/parent", "/World/child", "Y", -160.0, 2.0)

        converted = mjc_to_physx_conversion_utils.combine_overconstrained_joints_to_d6(stage)
        self.assertEqual(converted, 0)

        knee = stage.GetPrimAtPath("/World/child/knee")
        self.assertEqual(knee.GetTypeName(), "PhysicsRevoluteJoint")

    async def test_combine_overconstrained_joints_handles_mixed_revolute_prismatic(self) -> None:
        """A revolute + prismatic pair sharing bodies collapse with both axis kinds."""
        stage = Usd.Stage.CreateInMemory()
        UsdGeom.Xform.Define(stage, "/World")
        UsdGeom.Xform.Define(stage, "/World/parent")
        UsdGeom.Xform.Define(stage, "/World/child")

        rev = self._make_revolute_joint(stage, "/World/child/twist", "/World/parent", "/World/child", "Z", -90.0, 90.0)

        prism = UsdPhysics.PrismaticJoint.Define(stage, "/World/child/slide")
        prism.CreateBody0Rel().SetTargets([Sdf.Path("/World/parent")])
        prism.CreateBody1Rel().SetTargets([Sdf.Path("/World/child")])
        prism.CreateAxisAttr().Set("X")
        prism.CreateLowerLimitAttr().Set(-0.5)
        prism.CreateUpperLimitAttr().Set(0.5)
        prism.CreateLocalRot0Attr().Set(Gf.Quatf(1, 0, 0, 0))
        prism.CreateLocalRot1Attr().Set(Gf.Quatf(1, 0, 0, 0))

        converted = mjc_to_physx_conversion_utils.combine_overconstrained_joints_to_d6(stage)
        self.assertEqual(converted, 1)

        d6_prim = stage.GetPrimAtPath("/World/child/twist")
        self.assertEqual(d6_prim.GetTypeName(), "PhysicsJoint")
        self.assertTrue(d6_prim.HasAPI(UsdPhysics.LimitAPI, "rotZ"))
        self.assertTrue(d6_prim.HasAPI(UsdPhysics.LimitAPI, "transX"))

        slide = stage.GetPrimAtPath("/World/child/slide")
        self.assertFalse(slide.IsActive())

    async def test_combine_overconstrained_joints_in_physx_layer_overrides_only_physx(self) -> None:
        """Authoring goes only into the supplied PhysX overlay layer."""
        with tempfile.TemporaryDirectory() as tmp_dir:
            physics_path = os.path.join(tmp_dir, "physics.usda")
            physx_path = os.path.join(tmp_dir, "physx.usda")

            physics_layer = Sdf.Layer.CreateNew(physics_path)
            physics_stage = Usd.Stage.Open(physics_layer)
            world = UsdGeom.Xform.Define(physics_stage, "/robot").GetPrim()
            physics_stage.SetDefaultPrim(world)
            UsdGeom.Xform.Define(physics_stage, "/robot/parent")
            UsdGeom.Xform.Define(physics_stage, "/robot/child")
            UsdPhysics.RigidBodyAPI.Apply(physics_stage.GetPrimAtPath("/robot/parent"))
            UsdPhysics.RigidBodyAPI.Apply(physics_stage.GetPrimAtPath("/robot/child"))
            self._make_revolute_joint(
                physics_stage, "/robot/child/hip_x", "/robot/parent", "/robot/child", "X", -45.0, 15.0
            )
            self._make_revolute_joint(
                physics_stage, "/robot/child/hip_y", "/robot/parent", "/robot/child", "Y", -120.0, 45.0
            )
            physics_layer.Save()

            # PhysX overlay sublayers physics (the asset-transformer layout).
            physx_layer = Sdf.Layer.CreateNew(physx_path)
            physx_layer.subLayerPaths.append("./physics.usda")
            physx_layer.defaultPrim = "robot"
            physx_layer.Save()

            converted = mjc_to_physx_conversion_utils.combine_overconstrained_joints_in_physx_layer(physx_path)
            self.assertEqual(converted, 1)

            # physics.usda must be untouched.
            physics_layer_after = Sdf.Layer.FindOrOpen(physics_path)
            for joint_path in ("/robot/child/hip_x", "/robot/child/hip_y"):
                spec = physics_layer_after.GetPrimAtPath(joint_path)
                self.assertIsNotNone(spec, joint_path)
                self.assertEqual(spec.typeName, "PhysicsRevoluteJoint")
                self.assertFalse(spec.HasInfo("active"), joint_path)

            # physx.usda: D6 def at primary path + active=false override on the secondary.
            physx_layer_after = Sdf.Layer.FindOrOpen(physx_path)
            primary_spec = physx_layer_after.GetPrimAtPath("/robot/child/hip_x")
            self.assertIsNotNone(primary_spec)
            self.assertEqual(primary_spec.typeName, "PhysicsJoint")

            secondary_spec = physx_layer_after.GetPrimAtPath("/robot/child/hip_y")
            self.assertIsNotNone(secondary_spec)
            self.assertEqual(secondary_spec.specifier, Sdf.SpecifierOver)
            self.assertEqual(secondary_spec.GetInfo("active"), False)

    async def test_combine_overconstrained_joints_preserves_all_properties(self) -> None:
        """3 revolute + 3 prismatic joints between the same body pair collapse.

        into a single D6 that preserves every per-axis limit/drive plus the
        primary's PhysxJointAPI/break/collision tuning.
        """
        stage = Usd.Stage.CreateInMemory()
        UsdGeom.Xform.Define(stage, "/Robot")
        UsdGeom.Xform.Define(stage, "/Robot/parent")
        UsdGeom.Xform.Define(stage, "/Robot/child")
        UsdPhysics.RigidBodyAPI.Apply(stage.GetPrimAtPath("/Robot/parent"))
        UsdPhysics.RigidBodyAPI.Apply(stage.GetPrimAtPath("/Robot/child"))

        primary_local_pos0 = Gf.Vec3f(0.1, 0.2, 0.3)
        primary_local_pos1 = Gf.Vec3f(-0.4, -0.5, -0.6)
        primary_local_rot0 = Gf.Quatf(0.7071068, 0.7071068, 0.0, 0.0)
        primary_local_rot1 = Gf.Quatf(0.7071068, 0.0, 0.7071068, 0.0)

        # (name, type, axis, low, high, damping, stiffness, max_force, target_pos)
        specs = [
            ("joint_rev_x", "revolute", "X", -45.0, 15.0, 1.5, 100.0, 250.0, 0.25),
            ("joint_rev_y", "revolute", "Y", -120.0, 45.0, 2.5, 200.0, 175.0, -0.5),
            ("joint_rev_z", "revolute", "Z", -60.0, 35.0, 0.5, 50.0, 90.0, 0.0),
            ("joint_prs_x", "prismatic", "X", -0.10, 0.20, 0.1, 10.0, 30.0, 0.05),
            ("joint_prs_y", "prismatic", "Y", -0.30, 0.40, 0.2, 20.0, 35.0, -0.10),
            ("joint_prs_z", "prismatic", "Z", -0.50, 0.05, 0.3, 30.0, 40.0, 0.15),
        ]
        spec_to_axis = {
            "joint_rev_x": "rotX",
            "joint_rev_y": "rotY",
            "joint_rev_z": "rotZ",
            "joint_prs_x": "transX",
            "joint_prs_y": "transY",
            "joint_prs_z": "transZ",
        }

        primary_path = f"/Robot/child/{specs[0][0]}"

        for idx, (name, kind, axis, lo, hi, damping, stiff, max_force, target_pos) in enumerate(specs):
            path = f"/Robot/child/{name}"
            if kind == "revolute":
                joint = UsdPhysics.RevoluteJoint.Define(stage, path)
                drive_instance = "angular"
            else:
                joint = UsdPhysics.PrismaticJoint.Define(stage, path)
                drive_instance = "linear"

            joint.CreateBody0Rel().SetTargets([Sdf.Path("/Robot/parent")])
            joint.CreateBody1Rel().SetTargets([Sdf.Path("/Robot/child")])
            joint.CreateAxisAttr().Set(axis)
            joint.CreateLowerLimitAttr().Set(lo)
            joint.CreateUpperLimitAttr().Set(hi)
            joint.CreateLocalPos0Attr().Set(primary_local_pos0)
            joint.CreateLocalPos1Attr().Set(primary_local_pos1)
            joint.CreateLocalRot0Attr().Set(primary_local_rot0)
            joint.CreateLocalRot1Attr().Set(primary_local_rot1)

            drive = UsdPhysics.DriveAPI.Apply(joint.GetPrim(), drive_instance)
            drive.CreateDampingAttr().Set(damping)
            drive.CreateStiffnessAttr().Set(stiff)
            drive.CreateMaxForceAttr().Set(max_force)
            drive.CreateTargetPositionAttr().Set(target_pos)
            if idx == 0:
                drive.CreateTypeAttr().Set(UsdPhysics.Tokens.acceleration)

        primary_prim = stage.GetPrimAtPath(primary_path)
        primary_prim.ApplyAPI(PhysxSchema.JOINT_API)
        primary_prim.CreateAttribute(PhysxAttr.JOINT_ARMATURE.name, PhysxAttr.JOINT_ARMATURE.type).Set(0.123)
        primary_prim.CreateAttribute(PhysxAttr.JOINT_FRICTION.name, PhysxAttr.JOINT_FRICTION.type).Set(0.456)
        primary_prim.CreateAttribute(PhysxAttr.JOINT_MAX_VELOCITY.name, PhysxAttr.JOINT_MAX_VELOCITY.type).Set(99.0)
        UsdPhysics.Joint(primary_prim).CreateBreakForceAttr().Set(1234.0)
        UsdPhysics.Joint(primary_prim).CreateBreakTorqueAttr().Set(5678.0)
        UsdPhysics.Joint(primary_prim).CreateCollisionEnabledAttr().Set(True)
        UsdPhysics.Joint(primary_prim).CreateExcludeFromArticulationAttr().Set(False)

        converted = mjc_to_physx_conversion_utils.combine_overconstrained_joints_to_d6(stage)
        self.assertEqual(converted, 1)

        d6_prim = stage.GetPrimAtPath(primary_path)
        self.assertEqual(d6_prim.GetTypeName(), "PhysicsJoint")

        d6_joint = UsdPhysics.Joint(d6_prim)
        self.assertEqual(list(d6_joint.GetBody0Rel().GetTargets()), [Sdf.Path("/Robot/parent")])
        self.assertEqual(list(d6_joint.GetBody1Rel().GetTargets()), [Sdf.Path("/Robot/child")])
        self.assertEqual(d6_joint.GetLocalPos0Attr().Get(), primary_local_pos0)
        self.assertEqual(d6_joint.GetLocalPos1Attr().Get(), primary_local_pos1)
        self.assertEqual(d6_joint.GetLocalRot0Attr().Get(), primary_local_rot0)
        self.assertEqual(d6_joint.GetLocalRot1Attr().Get(), primary_local_rot1)
        self.assertTrue(d6_joint.GetJointEnabledAttr().Get())
        self.assertTrue(d6_prim.IsActive())

        for name, *_ in specs[1:]:
            other = stage.GetPrimAtPath(f"/Robot/child/{name}")
            self.assertTrue(other.IsValid(), name)
            self.assertFalse(other.IsActive(), f"{name} should be inactive")

        for name, kind, axis, lo, hi, damping, stiff, max_force, target_pos in specs:
            token = spec_to_axis[name]
            self.assertTrue(d6_prim.HasAPI(UsdPhysics.LimitAPI, token), token)
            limit = UsdPhysics.LimitAPI(d6_prim, token)
            self.assertAlmostEqual(limit.GetLowAttr().Get(), lo, delta=1e-4, msg=f"{name} low")
            self.assertAlmostEqual(limit.GetHighAttr().Get(), hi, delta=1e-4, msg=f"{name} high")

            self.assertTrue(d6_prim.HasAPI(UsdPhysics.DriveAPI, token), token)
            drive = UsdPhysics.DriveAPI(d6_prim, token)
            self.assertAlmostEqual(drive.GetDampingAttr().Get(), damping, delta=1e-4, msg=f"{name} damping")
            self.assertAlmostEqual(drive.GetStiffnessAttr().Get(), stiff, delta=1e-4, msg=f"{name} stiffness")
            self.assertAlmostEqual(drive.GetMaxForceAttr().Get(), max_force, delta=1e-4, msg=f"{name} max force")
            self.assertAlmostEqual(
                drive.GetTargetPositionAttr().Get(), target_pos, delta=1e-4, msg=f"{name} target pos"
            )

        primary_token = spec_to_axis[specs[0][0]]
        self.assertEqual(
            UsdPhysics.DriveAPI(d6_prim, primary_token).GetTypeAttr().Get(), UsdPhysics.Tokens.acceleration
        )

        self.assertTrue(d6_prim.HasAPI(PhysxSchema.JOINT_API))
        self.assertAlmostEqual(d6_prim.GetAttribute(PhysxAttr.JOINT_ARMATURE.name).Get(), 0.123, delta=1e-5)
        self.assertAlmostEqual(d6_prim.GetAttribute(PhysxAttr.JOINT_FRICTION.name).Get(), 0.456, delta=1e-5)
        self.assertAlmostEqual(d6_prim.GetAttribute(PhysxAttr.JOINT_MAX_VELOCITY.name).Get(), 99.0, delta=1e-3)

        # Joint break/collision tuning carried from primary.
        self.assertAlmostEqual(d6_joint.GetBreakForceAttr().Get(), 1234.0, delta=1e-3)
        self.assertAlmostEqual(d6_joint.GetBreakTorqueAttr().Get(), 5678.0, delta=1e-3)
        self.assertTrue(d6_joint.GetCollisionEnabledAttr().Get())
        self.assertFalse(d6_joint.GetExcludeFromArticulationAttr().Get())

        # Legacy single-axis attrs must not remain authored on the edit
        # layer's D6 spec — they are stale on a PhysicsJoint host.
        edit_spec = stage.GetEditTarget().GetLayer().GetPrimAtPath(primary_path)
        self.assertIsNotNone(edit_spec)
        for prop_name in ("physics:axis", "physics:lowerLimit", "physics:upperLimit"):
            self.assertNotIn(prop_name, edit_spec.attributes, prop_name)

    async def test_combine_overconstrained_joints_promotes_primary_when_first_joint_dropped(self) -> None:
        """A leading joint without a recognizable axis is dropped; the D6.

        host is built at the path of the next joint with a valid axis. The
        dropped joint is deactivated; the new D6 host stays active.
        """
        stage = Usd.Stage.CreateInMemory()
        UsdGeom.Xform.Define(stage, "/Robot")
        UsdGeom.Xform.Define(stage, "/Robot/parent")
        UsdGeom.Xform.Define(stage, "/Robot/child")
        UsdPhysics.RigidBodyAPI.Apply(stage.GetPrimAtPath("/Robot/parent"))
        UsdPhysics.RigidBodyAPI.Apply(stage.GetPrimAtPath("/Robot/child"))

        # First joint in traversal order has an unrecognized physics:axis
        # token ("W"), so it gets dropped from D6 axis assignment.
        # ``RevoluteJoint``'s schema fallback for physics:axis is "X", so we
        # must explicitly author an invalid token to trigger the drop path.
        broken = UsdPhysics.RevoluteJoint.Define(stage, "/Robot/child/aaa_broken")
        broken.CreateBody0Rel().SetTargets([Sdf.Path("/Robot/parent")])
        broken.CreateBody1Rel().SetTargets([Sdf.Path("/Robot/child")])
        broken.CreateAxisAttr().Set("W")
        broken.CreateLocalRot0Attr().Set(Gf.Quatf(1, 0, 0, 0))
        broken.CreateLocalRot1Attr().Set(Gf.Quatf(1, 0, 0, 0))

        # Second joint has a valid axis -> becomes the D6 host.
        self._make_revolute_joint(stage, "/Robot/child/zzz_hip_y", "/Robot/parent", "/Robot/child", "Y", -90.0, 90.0)

        converted = mjc_to_physx_conversion_utils.combine_overconstrained_joints_to_d6(stage)
        self.assertEqual(converted, 1)

        # The broken joint must be deactivated (and not retyped).
        broken_prim = stage.GetPrimAtPath("/Robot/child/aaa_broken")
        self.assertFalse(broken_prim.IsActive(), "broken joint must be deactivated")
        self.assertEqual(broken_prim.GetTypeName(), "PhysicsRevoluteJoint")

        # The D6 host must live at the next joint's path and be active.
        d6_prim = stage.GetPrimAtPath("/Robot/child/zzz_hip_y")
        self.assertEqual(d6_prim.GetTypeName(), "PhysicsJoint")
        self.assertTrue(d6_prim.IsActive(), "D6 host must remain active")
        self.assertTrue(UsdPhysics.Joint(d6_prim).GetJointEnabledAttr().Get())
        self.assertTrue(d6_prim.HasAPI(UsdPhysics.LimitAPI, "rotY"))

    async def test_combine_overconstrained_joints_rewrites_newton_mimic_references(self) -> None:
        """``NewtonMimicAPI.newton:mimicJoint`` targeting a converted joint is redirected to the D6 host."""
        stage = Usd.Stage.CreateInMemory()
        UsdGeom.Xform.Define(stage, "/Robot")
        UsdGeom.Xform.Define(stage, "/Robot/parent")
        UsdGeom.Xform.Define(stage, "/Robot/child")
        # Use two distinct follower bodies so the two mimic-host joints
        # don't form a second over-constrained group themselves.
        UsdGeom.Xform.Define(stage, "/Robot/follower_a")
        UsdGeom.Xform.Define(stage, "/Robot/follower_b")
        UsdPhysics.RigidBodyAPI.Apply(stage.GetPrimAtPath("/Robot/parent"))
        UsdPhysics.RigidBodyAPI.Apply(stage.GetPrimAtPath("/Robot/child"))
        UsdPhysics.RigidBodyAPI.Apply(stage.GetPrimAtPath("/Robot/follower_a"))
        UsdPhysics.RigidBodyAPI.Apply(stage.GetPrimAtPath("/Robot/follower_b"))

        # Over-constrained hip group.
        self._make_revolute_joint(stage, "/Robot/child/hip_x", "/Robot/parent", "/Robot/child", "X", -45.0, 15.0)
        self._make_revolute_joint(stage, "/Robot/child/hip_y", "/Robot/parent", "/Robot/child", "Y", -120.0, 45.0)
        self._make_revolute_joint(stage, "/Robot/child/hip_z", "/Robot/parent", "/Robot/child", "Z", -60.0, 35.0)

        # Follower mimics hip_y (a secondary that gets folded into the D6).
        follower = self._make_revolute_joint(
            stage, "/Robot/follower_a/yaw", "/Robot/child", "/Robot/follower_a", "Y", -90.0, 90.0
        )
        follower.ApplyAPI("NewtonMimicAPI")
        follower.CreateRelationship("newton:mimicJoint").SetTargets([Sdf.Path("/Robot/child/hip_y")])
        follower.CreateAttribute("newton:mimicCoef0", Sdf.ValueTypeNames.Float).Set(0.25)
        follower.CreateAttribute("newton:mimicCoef1", Sdf.ValueTypeNames.Float).Set(1.5)

        # Follower #2 mimics hip_x (the primary; path is reused by the D6).
        follower2 = self._make_revolute_joint(
            stage, "/Robot/follower_b/roll", "/Robot/child", "/Robot/follower_b", "X", -90.0, 90.0
        )
        follower2.ApplyAPI("NewtonMimicAPI")
        follower2.CreateRelationship("newton:mimicJoint").SetTargets([Sdf.Path("/Robot/child/hip_x")])

        follower_prim = follower
        follower2_prim = follower2

        converted = mjc_to_physx_conversion_utils.combine_overconstrained_joints_to_d6(stage)
        self.assertEqual(converted, 1)

        # follower -> redirected from hip_y to the D6 host at hip_x.
        rel = follower_prim.GetRelationship("newton:mimicJoint")
        self.assertEqual(list(rel.GetTargets()), [Sdf.Path("/Robot/child/hip_x")])
        self.assertAlmostEqual(follower_prim.GetAttribute("newton:mimicCoef0").Get(), 0.25, delta=1e-5)
        self.assertAlmostEqual(follower_prim.GetAttribute("newton:mimicCoef1").Get(), 1.5, delta=1e-5)

        # follower2 already targets the primary; path unchanged.
        rel2 = follower2_prim.GetRelationship("newton:mimicJoint")
        self.assertEqual(list(rel2.GetTargets()), [Sdf.Path("/Robot/child/hip_x")])
