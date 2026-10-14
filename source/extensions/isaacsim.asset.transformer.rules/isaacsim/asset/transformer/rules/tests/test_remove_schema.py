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

"""Tests for the remove schema rule."""

from __future__ import annotations

import os
import shutil
import tempfile

import omni.kit.test
from isaacsim.asset.transformer.rules.core.remove_schema import RemoveSchemaRule
from pxr import Sdf, Usd, UsdGeom, UsdPhysics, UsdShade

from .common import _TEST_DATA_DIR

_TEST_USD = os.path.join(_TEST_DATA_DIR, "test_prims", "base.usda")
_TEST_PRIM_PATH = "/ur10e/joints/shoulder_pan_joint"
_TEST_SCHEMA_PATTERN = r"Physics(Drive|JointState)API:angular"
_TEST_PROPERTY_PATTERN = r"drive:angular:physics:.*"


class TestRemoveSchemaRule(omni.kit.test.AsyncTestCase):
    """Async tests for RemoveSchemaRule."""

    async def setUp(self) -> None:
        """Create a temporary directory for test output."""
        self._tmpdir = tempfile.mkdtemp()
        self._success = False

    async def tearDown(self) -> None:
        """Remove temporary directory after successful tests."""
        if self._success:
            shutil.rmtree(self._tmpdir, ignore_errors=True)

    def _create_rule(self, stage: Usd.Stage, params: dict[str, object]) -> RemoveSchemaRule:
        """Create a rule with the given parameters.

        Args:
            stage: Source stage for the rule.
            params: Rule parameters.

        Returns:
            Configured rule instance.

        """
        return RemoveSchemaRule(
            source_stage=stage,
            package_root=self._tmpdir,
            destination_path="payloads",
            args={"params": params},
        )

    def _author_flattened_prim(self, filename: str = "base.usda") -> tuple[Usd.Stage, str]:
        """Export a prim carrying four applied API schemas as an explicit list op.

        ``Usd.Stage.Flatten().Export()`` collapses the composed ``apiSchemas``
        into an explicit ``SdfListOp``, which is the form every flattened asset
        reaching the transformer has.

        Args:
            filename: Name of the exported layer inside the temp directory.

        Returns:
            Tuple of the opened stage and its layer path.

        """
        authoring_stage = Usd.Stage.CreateInMemory()
        robot = UsdGeom.Xform.Define(authoring_stage, "/World/Robot").GetPrim()
        UsdPhysics.ArticulationRootAPI.Apply(robot)
        UsdPhysics.CollisionAPI.Apply(robot)
        UsdShade.MaterialBindingAPI.Apply(robot)
        UsdPhysics.DriveAPI.Apply(robot, "angular")

        layer_path = os.path.join(self._tmpdir, filename)
        authoring_stage.Flatten().Export(layer_path)
        return Usd.Stage.Open(layer_path), layer_path

    async def test_process_rule_keeps_untargeted_schemas_on_explicit_list_op(self) -> None:
        """Removing one applied schema must leave the other schemas applied.

        An ``SdfListOp`` is either explicit or composing; USD clears the
        explicit vector on the mode transition. Authoring a deletion onto a
        spec that already holds an explicit ``apiSchemas`` list therefore
        discards every schema on the prim, not just the matched one.
        """
        working_stage, layer_path = self._author_flattened_prim()
        prim_path = "/World/Robot"

        list_op = working_stage.GetRootLayer().GetPrimAtPath(prim_path).GetInfo("apiSchemas")
        self.assertTrue(list_op.isExplicit, "fixture must author an explicit list op for this test to mean anything")

        before = [str(schema) for schema in working_stage.GetPrimAtPath(prim_path).GetAppliedSchemas()]
        self.assertEqual(len(before), 4, f"fixture must start with four applied schemas: {before}")

        rule = self._create_rule(
            working_stage,
            {"schema_patterns": [r"PhysicsDriveAPI.*"], "prim_path_patterns": [prim_path]},
        )
        rule.process_rule()
        working_stage.GetRootLayer().Save()

        reopened = Usd.Stage.Open(layer_path)
        after = [str(schema) for schema in reopened.GetPrimAtPath(prim_path).GetAppliedSchemas()]

        expected = [schema for schema in before if not schema.startswith("PhysicsDriveAPI")]
        self.assertEqual(after, expected, f"schemas destroyed as collateral: {sorted(set(expected) - set(after))}")
        self._success = True

    async def test_process_rule_removes_schema_prepended_in_the_same_layer(self) -> None:
        """A schema this layer itself prepends must not survive its own deletion.

        Within one list op a deletion is applied before the prepended items are
        re-added, so recording a delete without also dropping the token from
        ``prependedItems`` leaves the schema applied and the removal silently
        does nothing.
        """
        layer_path = os.path.join(self._tmpdir, "prepended.usda")
        stage = Usd.Stage.CreateNew(layer_path)
        robot = UsdGeom.Xform.Define(stage, "/World/Robot").GetPrim()
        UsdPhysics.CollisionAPI.Apply(robot)
        UsdPhysics.DriveAPI.Apply(robot, "angular")
        stage.GetRootLayer().Save()

        prim_path = "/World/Robot"
        list_op = stage.GetRootLayer().GetPrimAtPath(prim_path).GetInfo("apiSchemas")
        self.assertFalse(list_op.isExplicit, "fixture must author a composing list op for this test")

        rule = self._create_rule(
            stage,
            {"schema_patterns": [r"PhysicsDriveAPI.*"], "prim_path_patterns": [prim_path]},
        )
        rule.process_rule()
        stage.GetRootLayer().Save()

        reopened = Usd.Stage.Open(layer_path)
        after = [str(schema) for schema in reopened.GetPrimAtPath(prim_path).GetAppliedSchemas()]

        self.assertNotIn("PhysicsDriveAPI:angular", after, "targeted schema must actually be removed")
        self.assertIn("PhysicsCollisionAPI", after, "untargeted schema must survive")
        self._success = True

    async def test_process_rule_keeps_untargeted_schemas_in_legacy_added_list_op(self) -> None:
        """Untargeted schemas in a legacy ``add apiSchemas`` list must survive.

        ``add apiSchemas = [...]`` populates the list op's ``addedItems``
        vector, which older authoring tools and hand-written layers still use.
        Rebuilding the op from the other vectors alone silently discards every
        token it held, so removing one schema destroys the rest.
        """
        layer_path = os.path.join(self._tmpdir, "added.usda")
        layer = Sdf.Layer.CreateNew(layer_path)
        layer.ImportFromString("""#usda 1.0

def Xform "Robot" (
    add apiSchemas = ["PhysicsCollisionAPI", "PhysicsRigidBodyAPI"]
)
{
}
""")
        layer.Save()

        prim_path = "/Robot"
        list_op = layer.GetPrimAtPath(prim_path).GetInfo("apiSchemas")
        self.assertFalse(list_op.isExplicit, "fixture must author a composing list op for this test")
        self.assertEqual(
            list(list_op.addedItems),
            ["PhysicsCollisionAPI", "PhysicsRigidBodyAPI"],
            "fixture must land its tokens in addedItems for this test to mean anything",
        )

        stage = Usd.Stage.Open(layer_path)
        rule = self._create_rule(
            stage,
            {"schema_patterns": [r"PhysicsRigidBodyAPI"], "prim_path_patterns": [prim_path]},
        )
        rule.process_rule()
        stage.GetRootLayer().Save()

        reopened = Usd.Stage.Open(layer_path)
        after = [str(schema) for schema in reopened.GetPrimAtPath(prim_path).GetAppliedSchemas()]

        self.assertNotIn("PhysicsRigidBodyAPI", after, "targeted schema must actually be removed")
        self.assertIn("PhysicsCollisionAPI", after, "untargeted schema must survive")
        self._success = True

    async def test_process_rule_skips_without_patterns(self) -> None:
        """Verify rule skips when no patterns are provided."""
        stage = Usd.Stage.Open(_TEST_USD)
        rule = self._create_rule(stage, {"schema_patterns": [], "property_patterns": []})
        rule.process_rule()

        log = rule.get_operation_log()
        self.assertTrue(any("no schema or property patterns" in msg.lower() for msg in log))

    async def test_process_rule_writes_schema_properties(self) -> None:
        """Verify targeted schemas and properties are dropped in the destination layer.

        The fixture authors ``apiSchemas`` explicitly, so the removal has to
        filter that list rather than record a deletion against it, and the
        untargeted schema has to survive on the composed prim.
        """
        temp_asset = os.path.join(self._tmpdir, "ur10e.usd")
        shutil.copy(_TEST_USD, temp_asset)
        stage = Usd.Stage.Open(temp_asset)
        os.makedirs(os.path.join(self._tmpdir, "payloads"), exist_ok=True)

        params = {
            "schema_patterns": [_TEST_SCHEMA_PATTERN],
            "prim_path_patterns": [r"/ur10e/joints/.*"],
            "stage_name": "schema_override.usda",
            "property_patterns": [_TEST_PROPERTY_PATTERN],
            "clear_properties": True,
        }
        output_path = os.path.join(self._tmpdir, "payloads", "schema_override.usda")
        shutil.copy(temp_asset, output_path)

        rule = self._create_rule(stage, params)
        rule.process_rule()

        output_layer = Sdf.Layer.FindOrOpen(output_path)
        self.assertIsNotNone(output_layer)

        prim_spec = output_layer.GetPrimAtPath(_TEST_PRIM_PATH)
        self.assertIsNotNone(prim_spec)

        output_stage = Usd.Stage.Open(output_path)
        applied = [str(schema) for schema in output_stage.GetPrimAtPath(_TEST_PRIM_PATH).GetAppliedSchemas()]
        self.assertNotIn("PhysicsDriveAPI:angular", applied)
        self.assertNotIn("PhysicsJointStateAPI:angular", applied)
        self.assertIn("PhysxJointAPI", applied, "untargeted schema must survive the removal")

        removed_props = [
            attr_spec.name for attr_spec in prim_spec.attributes if attr_spec.name.startswith("drive:angular:physics:")
        ]
        self.assertEqual(removed_props, [])

        self._success = True

    async def test_process_rule_reads_routed_physics_layer(self) -> None:
        """Delete routed drive schemas from a stronger MuJoCo overlay."""
        physics_dir = os.path.join(self._tmpdir, "payloads", "Physics")
        os.makedirs(physics_dir)
        physics_path = os.path.join(physics_dir, "physics.usda")
        mujoco_path = os.path.join(physics_dir, "mujoco.usda")
        base_path = os.path.join(self._tmpdir, "payloads", "base.usda")

        physics_stage = Usd.Stage.CreateNew(physics_path)
        robot = UsdGeom.Xform.Define(physics_stage, "/robot")
        physics_stage.SetDefaultPrim(robot.GetPrim())
        joint = UsdPhysics.RevoluteJoint.Define(physics_stage, "/robot/Physics/joint").GetPrim()
        UsdPhysics.DriveAPI.Apply(joint, "angular")
        joint.ApplyAPI("PhysicsJointStateAPI", "angular")
        physics_stage.GetRootLayer().Save()

        base_stage = Usd.Stage.CreateNew(base_path)
        base_robot = UsdGeom.Xform.Define(base_stage, "/robot")
        base_stage.SetDefaultPrim(base_robot.GetPrim())
        base_stage.GetRootLayer().Save()

        mujoco_layer = Sdf.Layer.CreateNew(mujoco_path)
        mujoco_layer.subLayerPaths.append("./physics.usda")
        mujoco_layer.Save()

        rule = RemoveSchemaRule(
            source_stage=base_stage,
            package_root=self._tmpdir,
            destination_path="payloads/Physics",
            args={
                "params": {
                    "stage_name": "mujoco.usda",
                    "input_stage_path": "payloads/Physics/physics.usda",
                    "schema_patterns": [r"Physics(Drive|JointState)API:angular"],
                    "prim_path_patterns": [r"/robot/Physics/.*"],
                }
            },
        )
        rule.process_rule()

        output_layer = Sdf.Layer.FindOrOpen(mujoco_path)
        prim_spec = output_layer.GetPrimAtPath("/robot/Physics/joint")
        deleted_items = list(prim_spec.GetInfo("apiSchemas").deletedItems or [])
        self.assertCountEqual(deleted_items, ["PhysicsDriveAPI:angular", "PhysicsJointStateAPI:angular"])

        mujoco_stage = Usd.Stage.Open(mujoco_path)
        composed_joint = mujoco_stage.GetPrimAtPath("/robot/Physics/joint")
        applied_schemas = [str(schema) for schema in composed_joint.GetAppliedSchemas()]
        self.assertNotIn("PhysicsDriveAPI:angular", applied_schemas)
        self.assertNotIn("PhysicsJointStateAPI:angular", applied_schemas)
        self._success = True
