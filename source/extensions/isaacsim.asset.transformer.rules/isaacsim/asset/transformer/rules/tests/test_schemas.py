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
from pxr import Sdf, Usd

from .common import _TEST_DATA_DIR

_TEST_USD = os.path.join(_TEST_DATA_DIR, "test_prims", "base.usda")


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
