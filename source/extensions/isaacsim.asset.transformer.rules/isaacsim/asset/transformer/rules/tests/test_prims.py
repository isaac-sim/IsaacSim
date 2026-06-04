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

"""Tests for the prim routing rule."""

import os
import shutil
import tempfile

import omni.kit.test
from isaacsim.asset.transformer.rules.core.prims import (
    PrimRoutingRule,
)
from pxr import Sdf, Usd

from .common import _TEST_DATA_DIR

_TEST_USD = os.path.join(_TEST_DATA_DIR, "test_prims", "base.usda")


class TestPrimRoutingRule(omni.kit.test.AsyncTestCase):
    """Async tests for PrimRoutingRule."""

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
        rule = PrimRoutingRule(
            source_stage=stage,
            package_root=self._tmpdir,
            destination_path="payloads",
            args={},
        )

        params = rule.get_configuration_parameters()

        self.assertEqual(len(params), 6)
        param_names = [p.name for p in params]
        self.assertIn("prim_types", param_names)
        self.assertIn("ignore_prim_types", param_names)
        self.assertIn("stage_name", param_names)
        self.assertIn("scope", param_names)
        self.assertIn("prim_names", param_names)
        self.assertIn("ignore_prim_names", param_names)
        self._success = True

    async def test_process_rule_no_prim_types_raises(self) -> None:
        """Verify rule raises ValueError when prim_types is unconfigured.

        ``prim_types`` is the rule's primary selector; running without it is a
        misconfiguration, not a legitimate no-op, so the rule must surface a
        failure rather than silently skipping with success=True.
        """
        stage = Usd.Stage.Open(_TEST_USD)
        rule = PrimRoutingRule(
            source_stage=stage,
            package_root=self._tmpdir,
            destination_path="payloads",
            args={"params": {"prim_types": []}},
        )

        with self.assertRaises(ValueError):
            rule.process_rule()

        # Same expectation when the param is missing entirely (default None).
        rule_missing = PrimRoutingRule(
            source_stage=stage,
            package_root=self._tmpdir,
            destination_path="payloads",
            args={"params": {}},
        )
        with self.assertRaises(ValueError):
            rule_missing.process_rule()

    async def test_process_rule_with_scope(self) -> None:
        """Verify scope filter limits routed prims."""
        temp_asset = os.path.join(self._tmpdir, "ur10e.usd")
        shutil.copy(_TEST_USD, temp_asset)
        stage = Usd.Stage.Open(temp_asset)
        os.makedirs(os.path.join(self._tmpdir, "payloads"), exist_ok=True)

        # Search for PhysicsRevoluteJoint prims under /ur10e/joints scope
        rule = PrimRoutingRule(
            source_stage=stage,
            package_root=self._tmpdir,
            destination_path="payloads",
            args={
                "params": {
                    "prim_types": ["PhysicsRevoluteJoint"],
                    "stage_name": "joints.usda",
                    "scope": "/ur10e/joints",
                }
            },
        )

        rule.process_rule()

        # Open the output file and verify only prims under /joints scope are included
        output_path = os.path.join(self._tmpdir, "payloads", "joints.usda")
        output_stage = Usd.Stage.Open(output_path)
        output_layer = output_stage.GetRootLayer()
        prims_defined = [
            prim
            for prim in Usd.PrimRange(output_stage.GetDefaultPrim(), Usd.PrimAllPrimsPredicate)
            if output_layer.GetPrimAtPath(prim.GetPath())
            and output_layer.GetPrimAtPath(prim.GetPath()).specifier == Sdf.SpecifierDef
        ]
        # All defined prims should be under /joints scope
        self.assertTrue(len(prims_defined) > 0, "Expected at least one prim to be routed")
        self.assertTrue(all("/joints" in str(prim.GetPath()) for prim in prims_defined))
        # Check if there are any prims defined that do not contain /joints in their path
        self.assertFalse(any("/joints" not in str(prim.GetPath()) for prim in prims_defined))
        self._success = True

    async def test_process_rule_logs_completion(self) -> None:
        """Verify completion log entries are recorded."""
        temp_asset = os.path.join(self._tmpdir, "ur10e.usd")
        shutil.copy(_TEST_USD, temp_asset)
        stage = Usd.Stage.Open(temp_asset)
        os.makedirs(os.path.join(self._tmpdir, "payloads"), exist_ok=True)

        rule = PrimRoutingRule(
            source_stage=stage,
            package_root=self._tmpdir,
            destination_path="payloads",
            args={
                "params": {
                    "prim_types": ["Scope"],
                    "stage_name": "scopes.usda",
                }
            },
        )

        rule.process_rule()

        log = rule.get_operation_log()
        self.assertTrue(any("PrimRoutingRule start" in msg for msg in log))
        self.assertTrue(any("PrimRoutingRule completed" in msg for msg in log))

        self._success = True

    async def test_process_rule_with_prim_names(self) -> None:
        """Verify prim name filters route expected prims."""
        temp_asset = os.path.join(self._tmpdir, "ur10e.usd")
        shutil.copy(_TEST_USD, temp_asset)
        stage = Usd.Stage.Open(temp_asset)
        os.makedirs(os.path.join(self._tmpdir, "payloads"), exist_ok=True)

        rule = PrimRoutingRule(
            source_stage=stage,
            package_root=self._tmpdir,
            destination_path="payloads",
            args={
                "params": {
                    "prim_names": ["Looks"],
                    "prim_types": ["Scope"],
                    "stage_name": "scopes.usda",
                }
            },
        )

        rule.process_rule()

        log = rule.get_operation_log()
        # Open the output file and check if the Looks prim is present
        output_path = os.path.join(self._tmpdir, "payloads", "scopes.usda")

        output_stage = Usd.Stage.Open(output_path)
        # Use PrimAllPrimsPredicate to traverse children of over prims, filter to prims with def specifier in this layer
        output_layer = output_stage.GetRootLayer()
        prims_defined = [
            prim
            for prim in Usd.PrimRange(output_stage.GetDefaultPrim(), Usd.PrimAllPrimsPredicate)
            if output_layer.GetPrimAtPath(prim.GetPath())
            and output_layer.GetPrimAtPath(prim.GetPath()).specifier == Sdf.SpecifierDef
        ]
        self.assertIsNotNone(prims_defined)
        self.assertTrue(any("Looks" in prim.GetName() for prim in prims_defined))
        self.assertFalse(any("VisualMaterials" in prim.GetName() for prim in prims_defined))

        self._success = True

    async def test_process_rule_with_ignore_prim_names(self) -> None:
        """Verify ignore prim name filters exclude prims."""
        temp_asset = os.path.join(self._tmpdir, "ur10e.usd")
        shutil.copy(_TEST_USD, temp_asset)
        stage = Usd.Stage.Open(temp_asset)
        os.makedirs(os.path.join(self._tmpdir, "payloads"), exist_ok=True)

        rule = PrimRoutingRule(
            source_stage=stage,
            package_root=self._tmpdir,
            destination_path="payloads",
            args={
                "params": {
                    "ignore_prim_names": ["joints"],
                    "prim_types": ["Scope"],
                    "stage_name": "scopes.usda",
                }
            },
        )

        rule.process_rule()

        log = rule.get_operation_log()
        # Open the output file and check if the Looks prim is present
        output_path = os.path.join(self._tmpdir, "payloads", "scopes.usda")
        output_stage = Usd.Stage.Open(output_path)

        # Use PrimAllPrimsPredicate to traverse children of over prims, filter to prims with def specifier in this layer
        output_layer = output_stage.GetRootLayer()
        prims_defined = [
            prim
            for prim in Usd.PrimRange(output_stage.GetDefaultPrim(), Usd.PrimAllPrimsPredicate)
            if output_layer.GetPrimAtPath(prim.GetPath())
            and output_layer.GetPrimAtPath(prim.GetPath()).specifier == Sdf.SpecifierDef
        ]

        self.assertFalse(any("joints" in prim.GetName() for prim in prims_defined))
        self.assertTrue(any("VisualMaterials" in prim.GetName() for prim in prims_defined))

        self._success = True

    async def test_process_rule_with_prim_types(self) -> None:
        """Verify prim type filters route expected prims."""
        temp_asset = os.path.join(self._tmpdir, "ur10e.usd")
        shutil.copy(_TEST_USD, temp_asset)
        stage = Usd.Stage.Open(temp_asset)
        os.makedirs(os.path.join(self._tmpdir, "payloads"), exist_ok=True)

        rule = PrimRoutingRule(
            source_stage=stage,
            package_root=self._tmpdir,
            destination_path="payloads",
            args={
                "params": {
                    "prim_types": [".*Joint"],
                    "stage_name": "joints.usda",
                }
            },
        )

        rule.process_rule()

        log = rule.get_operation_log()
        # Open the output file and check if the Looks prim is present
        output_path = os.path.join(self._tmpdir, "payloads", "joints.usda")

        output_stage = Usd.Stage.Open(output_path)
        # Use PrimAllPrimsPredicate to traverse children of over prims, filter to prims with def specifier in this layer
        output_layer = output_stage.GetRootLayer()
        prims_defined = [
            prim
            for prim in Usd.PrimRange(output_stage.GetDefaultPrim(), Usd.PrimAllPrimsPredicate)
            if output_layer.GetPrimAtPath(prim.GetPath())
            and output_layer.GetPrimAtPath(prim.GetPath()).specifier == Sdf.SpecifierDef
        ]

        self.assertIsNotNone(prims_defined)
        self.assertTrue(any("Joint" in prim.GetTypeName() for prim in prims_defined))
        self.assertFalse(any("xform" in prim.GetTypeName() for prim in prims_defined))

        self._success = True

    async def test_process_rule_with_ignore_prim_types(self) -> None:
        """Verify ignore prim type filters override includes."""
        temp_asset = os.path.join(self._tmpdir, "ur10e.usd")
        shutil.copy(_TEST_USD, temp_asset)
        stage = Usd.Stage.Open(temp_asset)
        os.makedirs(os.path.join(self._tmpdir, "payloads"), exist_ok=True)

        rule = PrimRoutingRule(
            source_stage=stage,
            package_root=self._tmpdir,
            destination_path="payloads",
            args={
                "params": {
                    "prim_types": [".*Joint"],
                    "ignore_prim_types": [".*Revolute.*Joint"],
                    "stage_name": "scopes.usda",
                }
            },
        )

        rule.process_rule()

        log = rule.get_operation_log()
        # Open the output file and check if the Looks prim is present
        output_path = os.path.join(self._tmpdir, "payloads", "scopes.usda")
        output_stage = Usd.Stage.Open(output_path)

        # Use PrimAllPrimsPredicate to traverse children of over prims, filter to prims with def specifier in this layer
        output_layer = output_stage.GetRootLayer()
        prims_defined = [
            prim
            for prim in Usd.PrimRange(output_stage.GetDefaultPrim(), Usd.PrimAllPrimsPredicate)
            if output_layer.GetPrimAtPath(prim.GetPath())
            and output_layer.GetPrimAtPath(prim.GetPath()).specifier == Sdf.SpecifierDef
        ]

        self.assertFalse(any("Revolute" in prim.GetTypeName() for prim in prims_defined))
        self.assertTrue(any("Fixed" in prim.GetTypeName() for prim in prims_defined))

        self._success = True
