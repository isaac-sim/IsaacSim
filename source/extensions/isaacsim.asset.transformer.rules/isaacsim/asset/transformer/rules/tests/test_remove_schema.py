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
from pxr import Sdf, Usd

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

    async def test_process_rule_skips_without_patterns(self) -> None:
        """Verify rule skips when no patterns are provided."""
        stage = Usd.Stage.Open(_TEST_USD)
        rule = self._create_rule(stage, {"schema_patterns": [], "property_patterns": []})
        rule.process_rule()

        log = rule.get_operation_log()
        self.assertTrue(any("no schema or property patterns" in msg.lower() for msg in log))

    async def test_process_rule_writes_schema_properties(self) -> None:
        """Verify schema properties are authored in the destination layer."""
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
        api_schemas = prim_spec.GetInfo("apiSchemas")
        self.assertTrue(api_schemas)
        deleted_items = list(api_schemas.deletedItems or [])
        self.assertTrue(any("PhysicsDriveAPI:angular" in str(item) for item in deleted_items))
        self.assertTrue(any("PhysicsJointStateAPI:angular" in str(item) for item in deleted_items))

        removed_props = [
            attr_spec.name for attr_spec in prim_spec.attributes if attr_spec.name.startswith("drive:angular:physics:")
        ]
        self.assertEqual(removed_props, [])

        self._success = True
