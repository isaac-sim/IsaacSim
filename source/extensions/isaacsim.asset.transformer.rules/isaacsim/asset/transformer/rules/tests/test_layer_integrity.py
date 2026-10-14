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

"""Durable routing and temporary-composition cleanup regressions."""

import tempfile
from pathlib import Path
from unittest.mock import patch

import omni.kit.test
from isaacsim.asset.transformer import AssetTransformerManager, RuleProfile
from isaacsim.asset.transformer.rules import DEFAULT_PROFILE_PATH, register_all_rules
from isaacsim.asset.transformer.rules.isaac_sim.robot_schema import RobotSchemaRule
from pxr import Sdf, Usd, UsdGeom, UsdPhysics


class TestLayerIntegrity(omni.kit.test.AsyncTestCase):
    """Exercise layerintegrity regressions."""

    def test_newton_joint_properties_remain_in_shared_physics_layer(self) -> None:
        """Keep Newton properties out of solver-specific MuJoCo layers."""
        register_all_rules()
        with tempfile.TemporaryDirectory() as tmp:
            source = str(Path(tmp) / "joint.usda")
            stage = Usd.Stage.CreateNew(source)
            stage.SetDefaultPrim(UsdGeom.Xform.Define(stage, "/Robot").GetPrim())
            joint = UsdPhysics.RevoluteJoint.Define(stage, "/Robot/elbow").GetPrim()
            self.assertTrue(joint.ApplyAPI("NewtonJointAPI"))
            self.assertTrue(joint.ApplyAPI("MjcJointAPI"))
            self.assertTrue(joint.ApplyAPI("PhysxJointAPI"))
            self.assertTrue(joint.GetAttribute("newton:friction").Set(0.1))
            self.assertTrue(joint.GetAttribute("newton:velocityLimit").Set(0.5))
            stage.GetRootLayer().Save()

            profile = RuleProfile.from_json(Path(DEFAULT_PROFILE_PATH).read_text())
            output = str(Path(tmp) / "output")
            report = AssetTransformerManager().run(source, profile, output)
            self.assertEqual(
                [(result.rule.name, result.error) for result in report.results if not result.success],
                [],
            )

            physics_layer = Sdf.Layer.FindOrOpen(str(Path(output) / "payloads/Physics/physics.usda"))
            mujoco_layer = Sdf.Layer.FindOrOpen(str(Path(output) / "payloads/Physics/mujoco.usda"))
            self.assertIsNotNone(physics_layer.GetPropertyAtPath("/Robot/elbow.newton:friction"))
            self.assertIsNone(mujoco_layer.GetPropertyAtPath("/Robot/elbow.newton:friction"))

            composed = Usd.Stage.Open(report.output_stage_path)
            variants = composed.GetDefaultPrim().GetVariantSet("Physics")
            variants.SetVariantSelection("physx")
            composed_joint = composed.GetPrimAtPath("/Robot/elbow")
            self.assertAlmostEqual(composed_joint.GetAttribute("newton:friction").Get(), 0.1)
            self.assertAlmostEqual(composed_joint.GetAttribute("newton:velocityLimit").Get(), 0.5)

    def test_routing_matrix(self) -> None:
        """Verify routing matrix."""
        register_all_rules()
        for robot_enabled in (False, True):
            for interface_enabled in (False, True):
                with (
                    self.subTest(robot=robot_enabled, interface=interface_enabled),
                    tempfile.TemporaryDirectory() as tmp,
                ):
                    source = str(Path(tmp) / "prop.usda")
                    stage = Usd.Stage.CreateNew(source)
                    stage.SetDefaultPrim(UsdGeom.Xform.Define(stage, "/Prop").GetPrim())
                    cube = UsdGeom.Cube.Define(stage, "/Prop/Cube").GetPrim()
                    UsdPhysics.RigidBodyAPI.Apply(cube)
                    UsdPhysics.CollisionAPI.Apply(cube)
                    UsdPhysics.MassAPI.Apply(cube)
                    stage.GetRootLayer().Save()
                    profile = RuleProfile.from_json(Path(DEFAULT_PROFILE_PATH).read_text())
                    for spec in profile.rules:
                        if spec.type.endswith("RobotSchemaRule"):
                            spec.enabled = robot_enabled
                        if spec.type.endswith("InterfaceConnectionRule"):
                            spec.enabled = interface_enabled
                    output = str(Path(tmp) / "output")
                    report = AssetTransformerManager().run(source, profile, output)
                    self.assertEqual(
                        [(r.rule.name, r.error) for r in report.results if not r.success],
                        [],
                    )
                    # Import the on-disk content into an independent layer, bypassing the cache.
                    disk_base = Sdf.Layer.CreateAnonymous("disk.usda")
                    disk_base.Import(str(Path(output) / "payloads/base.usda"))
                    spec = disk_base.GetPrimAtPath("/Prop/Cube")
                    self.assertNotIn("PhysicsRigidBodyAPI", str(spec.GetInfo("apiSchemas")))
                    self.assertNotIn("PhysicsMassAPI", str(spec.GetInfo("apiSchemas")))
                    # CollisionAPI is explicitly ignored by the shipped routing profile.
                    self.assertIn("PhysicsCollisionAPI", str(spec.GetInfo("apiSchemas")))
                    if interface_enabled:
                        composed = Usd.Stage.Open(report.output_stage_path)
                        variants = composed.GetDefaultPrim().GetVariantSet("Physics")
                        variants.SetVariantSelection("none")
                        self.assertFalse(composed.GetPrimAtPath("/Prop/Cube").HasAPI(UsdPhysics.RigidBodyAPI))
                        variants.SetVariantSelection("physics")
                        self.assertTrue(composed.GetPrimAtPath("/Prop/Cube").HasAPI(UsdPhysics.RigidBodyAPI))

    def test_robot_schema_preserves_dirty_edits_on_error(self) -> None:
        """Verify robot schema preserves dirty edits on error."""
        with tempfile.TemporaryDirectory() as tmp:
            stage = Usd.Stage.CreateNew(str(Path(tmp) / "base.usda"))
            root = UsdGeom.Xform.Define(stage, "/Robot").GetPrim()
            stage.SetDefaultPrim(root)
            stage.GetRootLayer().Save()
            root.CreateAttribute("unrelated", Sdf.ValueTypeNames.Int).Set(42)
            before = stage.GetRootLayer().ExportToString()
            rule = RobotSchemaRule(stage, tmp, "payloads", {"params": {"stage_name": "robot.usda"}})
            with patch(
                "usd.schema.isaac.robot_schema.utils.PopulateRobotSchemaFromArticulation",
                side_effect=RuntimeError("fixture failure"),
            ):
                with self.assertRaisesRegex(RuntimeError, "fixture failure"):
                    rule.process_rule()
            self.assertEqual(stage.GetRootLayer().ExportToString(), before)
            self.assertTrue(stage.GetRootLayer().dirty)
