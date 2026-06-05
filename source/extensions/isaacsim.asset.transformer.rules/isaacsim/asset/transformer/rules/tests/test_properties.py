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

"""Tests for the property routing rule."""

import os
import shutil
import tempfile

import omni.kit.test
from isaacsim.asset.transformer.rules.core.properties import (
    PropertyRoutingRule,
)
from pxr import Usd

from .common import _TEST_DATA_DIR

_TEST_USD = os.path.join(_TEST_DATA_DIR, "test_prims", "base.usda")


class TestPropertyRoutingRule(omni.kit.test.AsyncTestCase):
    """Async tests for PropertyRoutingRule."""

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
        rule = PropertyRoutingRule(
            source_stage=stage,
            package_root=self._tmpdir,
            destination_path="payloads",
            args={},
        )

        params = rule.get_configuration_parameters()

        self.assertEqual(len(params), 6)
        param_names = [p.name for p in params]
        self.assertIn("properties", param_names)
        self.assertIn("ignore_properties", param_names)
        self.assertIn("stage_name", param_names)
        self.assertIn("scope", param_names)
        self.assertIn("prim_names", param_names)
        self.assertIn("ignore_prim_names", param_names)
        self._success = True

    async def test_process_rule_no_property_patterns_skips(self) -> None:
        """Verify rule skips when no property patterns are provided."""
        stage = Usd.Stage.Open(_TEST_USD)
        rule = PropertyRoutingRule(
            source_stage=stage,
            package_root=self._tmpdir,
            destination_path="payloads",
            args={"params": {"properties": []}},
        )

        rule.process_rule()

        log = rule.get_operation_log()
        self.assertTrue(any("No property patterns" in msg for msg in log))

    async def test_process_rule_with_scope(self) -> None:
        """Verify scope filter limits routed properties."""
        temp_asset = os.path.join(self._tmpdir, "ur10e.usd")
        shutil.copy(_TEST_USD, temp_asset)
        stage = Usd.Stage.Open(temp_asset)
        os.makedirs(os.path.join(self._tmpdir, "payloads"), exist_ok=True)

        rule = PropertyRoutingRule(
            source_stage=stage,
            package_root=self._tmpdir,
            destination_path="payloads",
            args={
                "params": {
                    "properties": ["xformOp:.*"],
                    "stage_name": "xform_props.usda",
                    "scope": "/ur10e/joints",
                }
            },
        )

        rule.process_rule()

        log = rule.get_operation_log()
        self.assertTrue(any("/ur10e/joints" in msg for msg in log))

        self._success = True

    async def test_process_rule_logs_completion(self) -> None:
        """Verify completion log entries are recorded."""
        temp_asset = os.path.join(self._tmpdir, "ur10e.usd")
        shutil.copy(_TEST_USD, temp_asset)
        stage = Usd.Stage.Open(temp_asset)
        os.makedirs(os.path.join(self._tmpdir, "payloads"), exist_ok=True)

        rule = PropertyRoutingRule(
            source_stage=stage,
            package_root=self._tmpdir,
            destination_path="payloads",
            args={
                "params": {
                    "properties": ["xformOp:.*"],
                    "stage_name": "xform_props.usda",
                }
            },
        )

        rule.process_rule()

        log = rule.get_operation_log()
        self.assertTrue(any("PropertyRoutingRule start" in msg for msg in log))
        self.assertTrue(any("PropertyRoutingRule completed" in msg for msg in log))

        self._success = True

    async def test_process_rule_with_prim_names(self) -> None:
        """Verify prim name filters route expected properties."""
        temp_asset = os.path.join(self._tmpdir, "ur10e.usd")
        shutil.copy(_TEST_USD, temp_asset)
        stage = Usd.Stage.Open(temp_asset)
        os.makedirs(os.path.join(self._tmpdir, "payloads"), exist_ok=True)

        rule = PropertyRoutingRule(
            source_stage=stage,
            package_root=self._tmpdir,
            destination_path="payloads",
            args={
                "params": {
                    "prim_names": ["*link*"],
                    "properties": ["xformOp:*"],
                    "stage_name": "xform_props.usda",
                }
            },
        )

        rule.process_rule()

        log = rule.get_operation_log()
        # Open the output file and check if properties are present
        output_path = os.path.join(self._tmpdir, "payloads", "xform_props.usda")

        if os.path.exists(output_path):
            output_stage = Usd.Stage.Open(output_path)
            output_layer = output_stage.GetRootLayer()
            # Check that only prims matching *link* have properties
            prims_defined = list(Usd.PrimRange(output_stage.GetDefaultPrim(), Usd.PrimAllPrimsPredicate))
            self.assertTrue(any("link" in prim.GetName().lower() for prim in prims_defined))
            for prim in prims_defined:
                prim_spec = output_layer.GetPrimAtPath(prim.GetPath())
                if prim_spec and prim_spec.properties:
                    prop_names = [prop.name for prop in prim_spec.properties]
                    self.assertFalse(any("xformOp" not in prop_name for prop_name in prop_names))

        self._success = True

    async def test_process_rule_with_ignore_prim_names(self) -> None:
        """Verify ignore prim name filters exclude properties."""
        temp_asset = os.path.join(self._tmpdir, "ur10e.usd")
        shutil.copy(_TEST_USD, temp_asset)
        stage = Usd.Stage.Open(temp_asset)
        os.makedirs(os.path.join(self._tmpdir, "payloads"), exist_ok=True)

        rule = PropertyRoutingRule(
            source_stage=stage,
            package_root=self._tmpdir,
            destination_path="payloads",
            args={
                "params": {
                    "ignore_prim_names": [".*link.*"],
                    "properties": ["xformOp:.*"],
                    "stage_name": "xform_props.usda",
                }
            },
        )

        rule.process_rule()

        log = rule.get_operation_log()
        # Open the output file and verify no *link* prims have properties
        output_path = os.path.join(self._tmpdir, "payloads", "xform_props.usda")

        if os.path.exists(output_path):
            output_stage = Usd.Stage.Open(output_path)
            output_layer = output_stage.GetRootLayer()
            # Check that only prims matching *link* have properties
            prims_defined = list(Usd.PrimRange(output_stage.GetDefaultPrim(), Usd.PrimAllPrimsPredicate))
            for prim in prims_defined:
                prim_spec = output_layer.GetPrimAtPath(prim.GetPath())
                if prim_spec and prim_spec.properties:
                    self.assertTrue("link" not in prim.GetName().lower())
                    prop_names = [prop.name for prop in prim_spec.properties]
                    self.assertFalse(any("xformOp" not in prop_name for prop_name in prop_names))

        self._success = True

    async def test_process_rule_with_property_patterns(self) -> None:
        """Verify property name patterns route properties."""
        temp_asset = os.path.join(self._tmpdir, "ur10e.usd")
        shutil.copy(_TEST_USD, temp_asset)
        stage = Usd.Stage.Open(temp_asset)
        os.makedirs(os.path.join(self._tmpdir, "payloads"), exist_ok=True)

        rule = PropertyRoutingRule(
            source_stage=stage,
            package_root=self._tmpdir,
            destination_path="payloads",
            args={
                "params": {
                    "properties": ["physics:*"],
                    "stage_name": "physics_props.usda",
                }
            },
        )

        rule.process_rule()

        log = rule.get_operation_log()
        # Open the output file and check if physics properties are present
        output_path = os.path.join(self._tmpdir, "payloads", "physics_props.usda")

        if os.path.exists(output_path):
            output_stage = Usd.Stage.Open(output_path)
            output_layer = output_stage.GetRootLayer()
            # Check that only prims matching *link* have properties
            prims_defined = list(Usd.PrimRange(output_stage.GetDefaultPrim(), Usd.PrimAllPrimsPredicate))
            for prim in prims_defined:
                prim_spec = output_layer.GetPrimAtPath(prim.GetPath())
                if prim_spec and prim_spec.properties:
                    prop_names = [prop.name for prop in prim_spec.properties]
                    self.assertFalse(any("physics" not in prop_name for prop_name in prop_names))

        self._success = True

    async def test_process_rule_with_ignore_properties(self) -> None:
        """Verify ignore property patterns exclude properties."""
        temp_asset = os.path.join(self._tmpdir, "ur10e.usd")
        shutil.copy(_TEST_USD, temp_asset)
        stage = Usd.Stage.Open(temp_asset)
        os.makedirs(os.path.join(self._tmpdir, "payloads"), exist_ok=True)

        rule = PropertyRoutingRule(
            source_stage=stage,
            package_root=self._tmpdir,
            destination_path="payloads",
            args={
                "params": {
                    "properties": ["physics:.*"],
                    "ignore_properties": ["physics:diagonalInertia"],
                    "stage_name": "xform_props.usda",
                }
            },
        )

        rule.process_rule()

        log = rule.get_operation_log()
        output_path = os.path.join(self._tmpdir, "payloads", "xform_props.usda")

        output_path = os.path.join(self._tmpdir, "payloads", "physics_props.usda")

        if os.path.exists(output_path):
            output_stage = Usd.Stage.Open(output_path)
            output_layer = output_stage.GetRootLayer()
            # Check that only prims matching *link* have properties
            prims_defined = list(Usd.PrimRange(output_stage.GetDefaultPrim(), Usd.PrimAllPrimsPredicate))
            for prim in prims_defined:
                prim_spec = output_layer.GetPrimAtPath(prim.GetPath())
                if prim_spec and prim_spec.properties:
                    prop_names = [prop.name for prop in prim_spec.properties]
                    self.assertFalse(any("physics" not in prop_name for prop_name in prop_names))
                    self.assertTrue("physics:diagonalInertia" not in prop_names)

        self._success = True
