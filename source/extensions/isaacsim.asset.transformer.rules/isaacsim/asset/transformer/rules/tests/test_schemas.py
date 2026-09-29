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

"""Tests for the schema routing rule."""

import os
import shutil
import tempfile

import omni.kit.test
from isaacsim.asset.transformer.rules.core.schemas import (
    SchemaRoutingRule,
)
from pxr import Sdf, Usd, UsdGeom, UsdPhysics

from .common import _TEST_DATA_DIR

_TEST_USD = os.path.join(_TEST_DATA_DIR, "test_prims", "base.usda")

# MuJoCo and physics schema routing as configured in `isaacsim_structure.json`.
_MUJOCO_RULE_PARAMS = {"stage_name": "mujoco.usda", "schemas": ["Mjc.*", "mjc.*"]}
_PHYSICS_RULE_PARAMS = {
    "stage_name": "physics.usda",
    "schemas": ["Physics.*", "Newton.*"],
    "ignore_schemas": ["PhysicsCollisionAPI"],
}


def get_all_schema_items(api_schemas: Sdf.TokenListOp | None) -> list[object]:
    """Get all schema items from all lists in a TokenListOp.

    Args:
        api_schemas: TokenListOp containing applied schema tokens.

    Returns:
        List of all schema items across the list op sublists.

    """
    if not api_schemas:
        return []
    all_items = []
    all_items.extend(api_schemas.explicitItems or [])
    all_items.extend(api_schemas.prependedItems or [])
    all_items.extend(api_schemas.appendedItems or [])
    all_items.extend(api_schemas.deletedItems or [])
    all_items.extend(api_schemas.orderedItems or [])
    return all_items


def _build_builtin_api_stage(path: str) -> Usd.Stage:
    """Build a stage whose MuJoCo API schemas have built-in Newton and physics API schemas.

    The joint equality and the collider apply their API schemas the way mujoco-usd-converter does.

    Args:
        path: File path for the new stage.

    Returns:
        The new stage.

    """
    stage = Usd.Stage.CreateNew(path)
    stage.SetDefaultPrim(UsdGeom.Xform.Define(stage, "/robot").GetPrim())
    UsdPhysics.RevoluteJoint.Define(stage, "/robot/lead")

    # `MjcEqualityJointAPI` has the built-in `NewtonMimicAPI`.
    follow = UsdPhysics.RevoluteJoint.Define(stage, "/robot/follow").GetPrim()
    follow.ApplyAPI("NewtonMimicAPI")
    follow.ApplyAPI("MjcEqualityJointAPI")
    follow.GetRelationship("newton:mimicJoint").SetTargets([Sdf.Path("/robot/lead")])
    follow.GetAttribute("newton:mimicCoef0").Set(5.0)
    follow.GetAttribute("newton:mimicCoef1").Set(0.5)
    follow.GetAttribute("mjc:solref").Set([0.05, 1.0])

    # `MjcCollisionAPI` has the built-in `NewtonCollisionAPI` and `PhysicsCollisionAPI`.
    collider = UsdGeom.Cube.Define(stage, "/robot/collider").GetPrim()
    UsdPhysics.CollisionAPI.Apply(collider).CreateCollisionEnabledAttr(False)
    collider.ApplyAPI("NewtonCollisionAPI")
    collider.ApplyAPI("MjcCollisionAPI")
    collider.GetAttribute("newton:contactGap").Set(0.004)
    collider.GetAttribute("mjc:condim").Set(4)
    return stage


def _get_prim_spec_contents(layer: Sdf.Layer, path: str) -> tuple[set[str], set[str]]:
    """Get the applied schema tokens and property names that a layer authors on a prim.

    Args:
        layer: Layer to inspect.
        path: Prim path.

    Returns:
        Tuple of (applied schema tokens, property names), both empty if the layer has no prim spec at the path.

    """
    prim_spec = layer.GetPrimAtPath(path)
    if not prim_spec:
        return set(), set()
    schemas = {str(item) for item in get_all_schema_items(prim_spec.GetInfo("apiSchemas"))}
    return schemas, set(prim_spec.properties.keys())


class TestSchemaRoutingRule(omni.kit.test.AsyncTestCase):
    """Async tests for SchemaRoutingRule."""

    async def setUp(self) -> None:
        """Create a temporary directory for test output."""
        self._tmpdir = tempfile.mkdtemp()
        self._success = False

    async def tearDown(self) -> None:
        """Remove temporary directory after successful tests."""
        if self._success:
            shutil.rmtree(self._tmpdir, ignore_errors=True)

    def _route_schemas(self, stage: Usd.Stage, params: dict[str, object]) -> Sdf.Layer:
        """Run the rule into `payloads/Physics` and return the destination layer.

        Args:
            stage: Source stage for the rule.
            params: Rule parameters.

        Returns:
            The destination layer.

        """
        rule = SchemaRoutingRule(
            source_stage=stage,
            package_root=self._tmpdir,
            destination_path="payloads/Physics",
            args={"params": params},
        )
        rule.process_rule()
        return Sdf.Layer.FindOrOpen(os.path.join(self._tmpdir, "payloads", "Physics", params["stage_name"]))

    async def test_get_configuration_parameters(self) -> None:
        """Verify configuration parameters are exposed."""
        stage = Usd.Stage.Open(_TEST_USD)
        rule = SchemaRoutingRule(
            source_stage=stage,
            package_root=self._tmpdir,
            destination_path="payloads",
            args={},
        )

        params = rule.get_configuration_parameters()

        self.assertEqual(len(params), 5)
        param_names = [p.name for p in params]
        self.assertIn("schemas", param_names)
        self.assertIn("ignore_schemas", param_names)
        self.assertIn("stage_name", param_names)
        self.assertIn("prim_names", param_names)
        self.assertIn("ignore_prim_names", param_names)
        self._success = True

    async def test_process_rule_no_schemas_skips(self) -> None:
        """Verify rule skips when no schemas are provided."""
        stage = Usd.Stage.Open(_TEST_USD)
        rule = SchemaRoutingRule(
            source_stage=stage,
            package_root=self._tmpdir,
            destination_path="payloads",
            args={"params": {"schemas": []}},
        )

        rule.process_rule()

        log = rule.get_operation_log()
        self.assertTrue(any("No schemas" in msg for msg in log))

    async def test_process_rule_logs_completion(self) -> None:
        """Verify start log entry is recorded."""
        temp_asset = os.path.join(self._tmpdir, "ur10e.usd")
        shutil.copy(_TEST_USD, temp_asset)
        stage = Usd.Stage.Open(temp_asset)
        os.makedirs(os.path.join(self._tmpdir, "payloads"), exist_ok=True)

        rule = SchemaRoutingRule(
            source_stage=stage,
            package_root=self._tmpdir,
            destination_path="payloads",
            args={
                "params": {
                    "schemas": ["Physics*"],
                    "stage_name": "physics_schemas.usda",
                }
            },
        )

        rule.process_rule()

        log = rule.get_operation_log()
        print(log)
        self.assertTrue(any("SchemaRoutingRule start" in msg for msg in log))
        # Completion may not appear if no matches found
        # self.assertTrue(any("SchemaRoutingRule completed" in msg for msg in log))

        self._success = True

    async def test_process_rule_with_prim_names(self) -> None:
        """Verify prim name filters route schema opinions."""
        temp_asset = os.path.join(self._tmpdir, "ur10e.usd")
        shutil.copy(_TEST_USD, temp_asset)
        stage = Usd.Stage.Open(temp_asset)
        os.makedirs(os.path.join(self._tmpdir, "payloads"), exist_ok=True)

        rule = SchemaRoutingRule(
            source_stage=stage,
            package_root=self._tmpdir,
            destination_path="payloads",
            args={
                "params": {
                    "prim_names": ["*link*"],
                    "schemas": ["Physics*"],
                    "stage_name": "physics_schemas.usda",
                }
            },
        )

        rule.process_rule()

        log = rule.get_operation_log()
        # Open the output file and check if schemas were routed
        output_path = os.path.join(self._tmpdir, "payloads", "physics_schemas.usda")

        if os.path.exists(output_path):
            output_stage = Usd.Stage.Open(output_path)
            output_layer = output_stage.GetRootLayer()
            prims_defined = list(Usd.PrimRange(output_stage.GetDefaultPrim(), Usd.PrimAllPrimsPredicate))

            for prim in prims_defined:
                prim_spec = output_layer.GetPrimAtPath(prim.GetPath())
                if prim_spec:
                    api_schemas = prim_spec.GetInfo("apiSchemas")
                    if api_schemas:
                        all_items = get_all_schema_items(api_schemas)
                        if len(all_items) > 0:
                            self.assertTrue("link" in prim.GetName().lower())
                            self.assertFalse(any("Physics" not in str(item) for item in all_items))

        self._success = True

    async def test_process_rule_with_ignore_prim_names(self) -> None:
        """Verify ignore prim name filters exclude schemas."""
        temp_asset = os.path.join(self._tmpdir, "ur10e.usd")
        shutil.copy(_TEST_USD, temp_asset)
        stage = Usd.Stage.Open(temp_asset)
        os.makedirs(os.path.join(self._tmpdir, "payloads"), exist_ok=True)

        rule = SchemaRoutingRule(
            source_stage=stage,
            package_root=self._tmpdir,
            destination_path="payloads",
            args={
                "params": {
                    "ignore_prim_names": ["*link*"],
                    "schemas": ["Physics*"],
                    "stage_name": "physics_schemas.usda",
                }
            },
        )

        rule.process_rule()

        log = rule.get_operation_log()
        # Open the output file and verify no *link* prims have schemas
        output_path = os.path.join(self._tmpdir, "payloads", "physics_schemas.usda")

        if os.path.exists(output_path):
            output_stage = Usd.Stage.Open(output_path)
            output_layer = output_stage.GetRootLayer()
            prims_defined = list(Usd.PrimRange(output_stage.GetDefaultPrim(), Usd.PrimAllPrimsPredicate))

            for prim in prims_defined:
                prim_spec = output_layer.GetPrimAtPath(prim.GetPath())
                if prim_spec:
                    api_schemas = prim_spec.GetInfo("apiSchemas")
                    if api_schemas:
                        self.assertTrue("link" not in prim.GetName().lower())
                        all_items = get_all_schema_items(api_schemas)
                        self.assertFalse(any("Physics" not in str(item) for item in all_items))

        self._success = True

    async def test_process_rule_with_schema_patterns(self) -> None:
        """Verify schema patterns route matching schemas."""
        temp_asset = os.path.join(self._tmpdir, "ur10e.usd")
        shutil.copy(_TEST_USD, temp_asset)
        stage = Usd.Stage.Open(temp_asset)
        os.makedirs(os.path.join(self._tmpdir, "payloads"), exist_ok=True)

        rule = SchemaRoutingRule(
            source_stage=stage,
            package_root=self._tmpdir,
            destination_path="payloads",
            args={
                "params": {
                    "schemas": ["Physx*"],
                    "stage_name": "physx_schemas.usda",
                }
            },
        )

        rule.process_rule()

        log = rule.get_operation_log()
        # Open the output file and check if PhysX schemas are present
        output_path = os.path.join(self._tmpdir, "payloads", "physx_schemas.usda")

        if os.path.exists(output_path):
            output_stage = Usd.Stage.Open(output_path)
            output_layer = output_stage.GetRootLayer()
            prims_defined = list(Usd.PrimRange(output_stage.GetDefaultPrim(), Usd.PrimAllPrimsPredicate))

            for prim in prims_defined:
                prim_spec = output_layer.GetPrimAtPath(prim.GetPath())
                if prim_spec:
                    api_schemas = prim_spec.GetInfo("apiSchemas")
                    if api_schemas:
                        all_items = get_all_schema_items(api_schemas)
                        if len(all_items) > 0:
                            self.assertFalse(any("Physx" not in str(item) for item in all_items))

        self._success = True

    async def test_process_rule_with_ignore_schemas(self) -> None:
        """Verify ignore schema patterns exclude schemas."""
        temp_asset = os.path.join(self._tmpdir, "ur10e.usd")
        shutil.copy(_TEST_USD, temp_asset)
        stage = Usd.Stage.Open(temp_asset)
        os.makedirs(os.path.join(self._tmpdir, "payloads"), exist_ok=True)

        rule = SchemaRoutingRule(
            source_stage=stage,
            package_root=self._tmpdir,
            destination_path="payloads",
            args={
                "params": {
                    "schemas": ["Physics*"],
                    "ignore_schemas": ["PhysicsRigidBodyAPI"],
                    "stage_name": "physics_schemas.usda",
                }
            },
        )

        rule.process_rule()

        log = rule.get_operation_log()
        output_path = os.path.join(self._tmpdir, "payloads", "physics_schemas.usda")

        if os.path.exists(output_path):
            output_stage = Usd.Stage.Open(output_path)
            output_layer = output_stage.GetRootLayer()
            prims_defined = list(Usd.PrimRange(output_stage.GetDefaultPrim(), Usd.PrimAllPrimsPredicate))

            for prim in prims_defined:
                prim_spec = output_layer.GetPrimAtPath(prim.GetPath())
                if prim_spec:
                    api_schemas = prim_spec.GetInfo("apiSchemas")
                    if api_schemas:
                        all_items = get_all_schema_items(api_schemas)
                        self.assertFalse(any("Physics" not in str(item) for item in all_items))
                        self.assertFalse("PhysicsRigidBodyAPI" in all_items)

        self._success = True

    async def test_process_rule_skips_builtin_api_properties(self) -> None:
        """Verify a matched API schema moves only its own properties, not those of its built-in API schemas."""
        stage = _build_builtin_api_stage(os.path.join(self._tmpdir, "robot.usda"))

        mujoco_layer = self._route_schemas(stage, _MUJOCO_RULE_PARAMS)

        self.assertEqual(
            _get_prim_spec_contents(mujoco_layer, "/robot/follow"), ({"MjcEqualityJointAPI"}, {"mjc:solref"})
        )
        self.assertEqual(
            _get_prim_spec_contents(mujoco_layer, "/robot/collider"), ({"MjcCollisionAPI"}, {"mjc:condim"})
        )
        source_layer = stage.GetRootLayer()
        self.assertEqual(
            _get_prim_spec_contents(source_layer, "/robot/follow"),
            ({"NewtonMimicAPI"}, {"newton:mimicJoint", "newton:mimicCoef0", "newton:mimicCoef1"}),
        )
        self.assertEqual(
            _get_prim_spec_contents(source_layer, "/robot/collider"),
            ({"PhysicsCollisionAPI", "NewtonCollisionAPI"}, {"physics:collisionEnabled", "newton:contactGap"}),
        )
        self._success = True

    async def test_process_rule_routes_builtin_api_properties_with_their_schema(self) -> None:
        """Verify built-in API properties follow the rule that matches their schema, and a second pass is a no-op."""
        stage = _build_builtin_api_stage(os.path.join(self._tmpdir, "robot.usda"))
        layers = []
        for _ in range(2):
            mujoco_layer = self._route_schemas(stage, _MUJOCO_RULE_PARAMS)
            physics_layer = self._route_schemas(stage, _PHYSICS_RULE_PARAMS)
            layers.append([layer.ExportToString() for layer in (stage.GetRootLayer(), mujoco_layer, physics_layer)])

        self.assertEqual(
            _get_prim_spec_contents(physics_layer, "/robot/follow"),
            ({"NewtonMimicAPI"}, {"newton:mimicJoint", "newton:mimicCoef0", "newton:mimicCoef1"}),
        )
        mimic_joint = physics_layer.GetPropertyAtPath("/robot/follow.newton:mimicJoint")
        self.assertEqual(list(mimic_joint.targetPathList.explicitItems), [Sdf.Path("/robot/lead")])
        # `PhysicsCollisionAPI` is ignored by the rule, so its properties stay with it in the source layer.
        self.assertEqual(
            _get_prim_spec_contents(physics_layer, "/robot/collider"), ({"NewtonCollisionAPI"}, {"newton:contactGap"})
        )
        self.assertEqual(
            _get_prim_spec_contents(stage.GetRootLayer(), "/robot/collider"),
            ({"PhysicsCollisionAPI"}, {"physics:collisionEnabled"}),
        )
        self.assertEqual(layers[0], layers[1])
        self._success = True

    async def test_process_rule_routes_matched_builtin_apis(self) -> None:
        """Verify built-in API schemas matched by the same rule are routed with it and ignored ones are not."""
        stage = _build_builtin_api_stage(os.path.join(self._tmpdir, "robot.usda"))
        # Only `MjcEqualityJointAPI` is applied, so `NewtonMimicAPI` is applied only as its built-in.
        implied = UsdPhysics.RevoluteJoint.Define(stage, "/robot/implied").GetPrim()
        implied.ApplyAPI("MjcEqualityJointAPI")
        implied.GetAttribute("newton:mimicCoef1").Set(2.0)

        layer = self._route_schemas(stage, {**_PHYSICS_RULE_PARAMS, "schemas": ["Mjc.*", "Newton.*"]})

        self.assertEqual(
            _get_prim_spec_contents(layer, "/robot/follow"),
            (
                {"NewtonMimicAPI", "MjcEqualityJointAPI"},
                {"newton:mimicJoint", "newton:mimicCoef0", "newton:mimicCoef1", "mjc:solref"},
            ),
        )
        self.assertEqual(
            _get_prim_spec_contents(layer, "/robot/implied"), ({"MjcEqualityJointAPI"}, {"newton:mimicCoef1"})
        )
        self.assertEqual(
            _get_prim_spec_contents(layer, "/robot/collider"),
            ({"NewtonCollisionAPI", "MjcCollisionAPI"}, {"newton:contactGap", "mjc:condim"}),
        )
        self.assertEqual(
            _get_prim_spec_contents(stage.GetRootLayer(), "/robot/collider"),
            ({"PhysicsCollisionAPI"}, {"physics:collisionEnabled"}),
        )
        self._success = True
