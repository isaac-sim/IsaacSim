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

"""Tests for the interface connection rule."""

import os
import shutil
import tempfile

import omni.kit.test
from isaacsim.asset.transformer import AssetTransformerManager, RuleProfile, RuleRegistry, RuleSpec
from isaacsim.asset.transformer.rules import DEFAULT_PROFILE_PATH, register_all_rules
from isaacsim.asset.transformer.rules.structure.interface import (
    CONNECTION_PAYLOAD,
    CONNECTION_REFERENCE,
    CONNECTION_SUBLAYER,
    InterfaceConnectionRule,
)
from pxr import Sdf, Usd

from .common import _TEST_DATA_DIR, _UR10E_SHOULDER_USD

# Path to UR10e interface test asset (pre-organized payloads for interface testing)
_TEST_DATA_INTERFACE_DIR = os.path.join(_TEST_DATA_DIR, "ur10e_interface")
_UR10E_INTERFACE_USD = os.path.join(_TEST_DATA_INTERFACE_DIR, "payloads", "base.usda")


class TestInterfaceConnectionRule(omni.kit.test.AsyncTestCase):
    """Async tests for InterfaceConnectionRule."""

    async def setUp(self) -> None:
        """Create temporary payload structure for tests."""
        self._tmpdir = tempfile.mkdtemp()
        # Create payloads directory with base layer from UR10e
        self._setup_test_structure()
        self._success = False

    async def tearDown(self) -> None:
        """Remove temporary directory after successful tests."""
        if self._success:
            shutil.rmtree(self._tmpdir, ignore_errors=True)

    def _setup_test_structure(self) -> None:
        """Create a payloads directory populated with test assets."""
        # Create payloads directory with a flattened base layer
        # Copy the payloads directory from the UR10E_INTERFACE_USD to the temp directory
        src_payloads_dir = os.path.join(_TEST_DATA_INTERFACE_DIR, "payloads")
        payloads_dir = os.path.join(self._tmpdir, "payloads")
        if os.path.exists(src_payloads_dir):
            shutil.copytree(src_payloads_dir, payloads_dir)
        else:
            self.fail(f"Payloads directory not found: {src_payloads_dir}")
        self._base_usd_path = os.path.join(self._tmpdir, "payloads", "base.usda")

    async def test_get_configuration_parameters(self) -> None:
        """Verify configuration parameters are exposed."""
        stage = Usd.Stage.Open(_UR10E_INTERFACE_USD)
        rule = InterfaceConnectionRule(
            source_stage=stage,
            package_root=self._tmpdir,
            destination_path="",
            args={},
        )

        params = rule.get_configuration_parameters()

        self.assertEqual(len(params), 7)
        param_names = [p.name for p in params]
        self.assertIn("base_layer", param_names)
        self.assertIn("base_connection_type", param_names)
        self.assertIn("generate_folder_variants", param_names)
        self.assertIn("payloads_folder", param_names)
        self.assertIn("connections", param_names)
        self.assertIn("default_variant_selections", param_names)
        self.assertIn("clear_default_variant_sets", param_names)
        self._success = True

    async def test_process_rule_creates_interface_layer(self) -> None:
        """Verify interface layer is created from base layer."""
        stage = Usd.Stage.Open(_UR10E_INTERFACE_USD)

        rule = InterfaceConnectionRule(
            source_stage=stage,
            package_root=self._tmpdir,
            destination_path="",
            args={
                "input_stage_path": _UR10E_INTERFACE_USD,
                "interface_asset_name": "robot.usda",
                "params": {
                    "base_layer": "payloads/base.usda",
                    "base_connection_type": CONNECTION_REFERENCE,
                },
            },
        )

        rule.process_rule()

        interface_path = os.path.join(self._tmpdir, "robot.usda")
        self.assertTrue(os.path.exists(interface_path))

        interface_layer = Sdf.Layer.FindOrOpen(interface_path)
        self.assertIsNotNone(interface_layer)
        self.assertEqual(interface_layer.defaultPrim, "ur10e")
        self._success = True

    async def test_process_rule_no_base_layer_skips(self) -> None:
        """Verify rule skips when base layer is missing."""
        stage = Usd.Stage.Open(_UR10E_INTERFACE_USD)

        rule = InterfaceConnectionRule(
            source_stage=stage,
            package_root=self._tmpdir,
            destination_path="",
            args={
                "input_stage_path": _UR10E_INTERFACE_USD,
                "interface_asset_name": "robot.usda",
                "params": {
                    "base_layer": "payloads/nonexistent.usda",
                },
            },
        )

        rule.process_rule()

        log = rule.get_operation_log()
        self.assertTrue(any("No referenced files exist" in msg for msg in log))
        self._success = True

    async def test_process_rule_reference_connection(self) -> None:
        """Verify reference connection is authored on interface layer."""
        stage = Usd.Stage.Open(_UR10E_INTERFACE_USD)

        rule = InterfaceConnectionRule(
            source_stage=stage,
            package_root=self._tmpdir,
            destination_path="",
            args={
                "input_stage_path": _UR10E_INTERFACE_USD,
                "interface_asset_name": "robot.usda",
                "params": {
                    "base_layer": "payloads/base.usda",
                    "base_connection_type": CONNECTION_REFERENCE,
                },
            },
        )

        rule.process_rule()

        interface_path = os.path.join(self._tmpdir, "robot.usda")
        interface_layer = Sdf.Layer.FindOrOpen(interface_path)
        prim_spec = interface_layer.GetPrimAtPath("/ur10e")
        self.assertIsNotNone(prim_spec)
        self.assertTrue(prim_spec.hasReferences)
        self._success = True

    async def test_process_rule_payload_connection(self) -> None:
        """Verify payload connection is authored on interface layer."""
        stage = Usd.Stage.Open(_UR10E_INTERFACE_USD)

        rule = InterfaceConnectionRule(
            source_stage=stage,
            package_root=self._tmpdir,
            destination_path="",
            args={
                "input_stage_path": _UR10E_INTERFACE_USD,
                "interface_asset_name": "robot.usda",
                "params": {
                    "base_layer": "payloads/base.usda",
                    "base_connection_type": CONNECTION_PAYLOAD,
                },
            },
        )

        rule.process_rule()

        interface_path = os.path.join(self._tmpdir, "robot.usda")
        interface_layer = Sdf.Layer.FindOrOpen(interface_path)
        prim_spec = interface_layer.GetPrimAtPath("/ur10e")
        self.assertIsNotNone(prim_spec)
        self.assertTrue(prim_spec.hasPayloads)
        self._success = True

    async def test_process_rule_sublayer_connection(self) -> None:
        """Verify sublayer connection adds sublayer path."""
        stage = Usd.Stage.Open(_UR10E_INTERFACE_USD)

        rule = InterfaceConnectionRule(
            source_stage=stage,
            package_root=self._tmpdir,
            destination_path="",
            args={
                "input_stage_path": _UR10E_INTERFACE_USD,
                "interface_asset_name": "robot.usda",
                "params": {
                    "base_layer": "payloads/base.usda",
                    "base_connection_type": CONNECTION_SUBLAYER,
                },
            },
        )

        rule.process_rule()

        interface_path = os.path.join(self._tmpdir, "robot.usda")
        interface_layer = Sdf.Layer.FindOrOpen(interface_path)
        self.assertTrue(len(interface_layer.subLayerPaths) > 0)
        self._success = True

    async def test_process_rule_derives_interface_name_from_input(self) -> None:
        """Verify interface name derives from input stage path."""
        stage = Usd.Stage.Open(_UR10E_INTERFACE_USD)

        rule = InterfaceConnectionRule(
            source_stage=stage,
            package_root=self._tmpdir,
            destination_path="",
            args={
                "input_stage_path": _UR10E_INTERFACE_USD,
                # No interface_asset_name provided
                "params": {
                    "base_layer": "payloads/base.usda",
                },
            },
        )

        rule.process_rule()

        # Should create interface named after source file
        interface_path = os.path.join(self._tmpdir, "base.usda")
        self.assertTrue(os.path.exists(interface_path))
        self._success = True

    async def test_process_rule_invalid_connection_type_fallback(self) -> None:
        """Verify invalid connection type falls back to reference."""
        stage = Usd.Stage.Open(_UR10E_INTERFACE_USD)

        rule = InterfaceConnectionRule(
            source_stage=stage,
            package_root=self._tmpdir,
            destination_path="",
            args={
                "input_stage_path": _UR10E_INTERFACE_USD,
                "interface_asset_name": "robot.usda",
                "params": {
                    "base_layer": "payloads/base.usda",
                    "base_connection_type": "InvalidType",
                },
            },
        )

        rule.process_rule()

        log = rule.get_operation_log()
        self.assertTrue(any("Invalid connection type" in msg for msg in log))

        # Should fall back to Reference
        interface_path = os.path.join(self._tmpdir, "robot.usda")
        interface_layer = Sdf.Layer.FindOrOpen(interface_path)
        prim_spec = interface_layer.GetPrimAtPath("/ur10e")
        self.assertTrue(prim_spec.hasReferences)
        self._success = True

    async def test_process_rule_generate_folder_variants(self) -> None:
        """Verify folder variants are generated from payloads directory."""
        stage = Usd.Stage.Open(_UR10E_INTERFACE_USD)

        # Create variant folders with USD files
        gripper_dir = os.path.join(self._tmpdir, "payloads", "Gripper")
        os.makedirs(gripper_dir, exist_ok=True)
        gripper_140 = Sdf.Layer.CreateNew(os.path.join(gripper_dir, "2F_140.usda"))
        gripper_140.Save()
        gripper_85 = Sdf.Layer.CreateNew(os.path.join(gripper_dir, "2F_85.usda"))
        gripper_85.Save()

        rule = InterfaceConnectionRule(
            source_stage=stage,
            package_root=self._tmpdir,
            destination_path="",
            args={
                "input_stage_path": _UR10E_INTERFACE_USD,
                "interface_asset_name": "robot.usda",
                "params": {
                    "base_layer": "payloads/base.usda",
                    "generate_folder_variants": True,
                    "payloads_folder": "payloads",
                },
            },
        )

        rule.process_rule()

        log = rule.get_operation_log()
        self.assertTrue(any("variant set" in msg.lower() for msg in log))
        self._success = True

    async def test_process_rule_affected_stages(self) -> None:
        """Verify affected stages list is populated."""
        stage = Usd.Stage.Open(_UR10E_INTERFACE_USD)

        rule = InterfaceConnectionRule(
            source_stage=stage,
            package_root=self._tmpdir,
            destination_path="",
            args={
                "input_stage_path": _UR10E_INTERFACE_USD,
                "interface_asset_name": "robot.usda",
                "params": {
                    "base_layer": "payloads/base.usda",
                },
            },
        )

        rule.process_rule()

        affected = rule.get_affected_stages()
        self.assertTrue(len(affected) >= 1)
        self._success = True

    async def test_process_rule_logs_completion(self) -> None:
        """Verify completion log entries are recorded."""
        stage = Usd.Stage.Open(_UR10E_INTERFACE_USD)

        rule = InterfaceConnectionRule(
            source_stage=stage,
            package_root=self._tmpdir,
            destination_path="",
            args={
                "input_stage_path": _UR10E_INTERFACE_USD,
                "interface_asset_name": "robot.usda",
                "params": {
                    "base_layer": "payloads/base.usda",
                },
            },
        )

        rule.process_rule()

        log = rule.get_operation_log()
        self.assertTrue(any("InterfaceConnectionRule start" in msg for msg in log))
        self.assertTrue(any("InterfaceConnectionRule completed" in msg for msg in log))
        self._success = True

    async def test_full_interface_configuration(self) -> None:
        """End-to-end test with full GenerateInterface configuration."""
        errors = []

        base_usd = os.path.join(self._tmpdir, "payloads", "base.usda")
        stage = Usd.Stage.Open(base_usd)

        # Full configuration from GenerateInterface
        rule = InterfaceConnectionRule(
            source_stage=stage,
            package_root=self._tmpdir,
            destination_path="",
            args={
                "input_stage_path": base_usd,
                "interface_asset_name": "ur10e.usda",
                "params": {
                    "base_layer": "payloads/base.usda",
                    "base_connection_type": CONNECTION_REFERENCE,
                    "generate_folder_variants": True,
                    "payloads_folder": "payloads",
                    "connections": [
                        {
                            "asset_path": "payloads/Physics/physx.usda",
                            "target_path": "payloads/Physics/physics.usda",
                            "connection_type": CONNECTION_SUBLAYER,
                        },
                        {
                            "asset_path": "payloads/Physics/mujoco.usda",
                            "target_path": "payloads/Physics/physics.usda",
                            "connection_type": CONNECTION_SUBLAYER,
                        },
                        {
                            "asset_path": "payloads/base.usda",
                            "target_path": "payloads/robot.usda",
                            "connection_type": CONNECTION_SUBLAYER,
                        },
                    ],
                },
            },
        )

        rule.process_rule()

        # Validate interface layer exists
        interface_path = os.path.join(self._tmpdir, "ur10e.usda")
        if not os.path.exists(interface_path):
            errors.append(f"Interface layer not created: {interface_path}")

        interface_layer = Sdf.Layer.FindOrOpen(interface_path)
        if not interface_layer:
            errors.append(f"Failed to open interface layer: {interface_path}")
        else:
            # Validate default prim
            if interface_layer.defaultPrim != "ur10e":
                errors.append(f"Expected defaultPrim 'ur10e', got '{interface_layer.defaultPrim}'")

            # Validate reference to base layer
            prim_spec = interface_layer.GetPrimAtPath("/ur10e")
            if not prim_spec:
                errors.append("Default prim /ur10e not found in interface layer")
            elif not prim_spec.hasReferences:
                errors.append("Interface prim /ur10e should have references to base layer")

            # Validate variant sets
            if prim_spec:
                variant_set_names = list(prim_spec.variantSets.keys())  # noqa: SIM118
                for vs_name in ["Physics", "Gripper", "Sensor"]:
                    if vs_name not in variant_set_names:
                        errors.append(f"Missing variant set '{vs_name}' in interface layer")

                # Validate Physics variants
                physics_vs = prim_spec.variantSets.get("Physics")
                if physics_vs:
                    physics_variants = list(physics_vs.variants.keys())  # noqa: SIM118
                    for v_name in ["None", "physics", "physx"]:
                        if v_name not in physics_variants:
                            errors.append(f"Missing variant '{v_name}' in Physics variant set")

                # Validate Gripper variants
                gripper_vs = prim_spec.variantSets.get("Gripper")
                if gripper_vs:
                    gripper_variants = list(gripper_vs.variants.keys())  # noqa: SIM118
                    for v_name in ["None", "Robotiq_2F_140", "Robotiq_2F_85"]:
                        if v_name not in gripper_variants:
                            errors.append(f"Missing variant '{v_name}' in Gripper variant set")

        # Validate physx sublayer
        physx_path = os.path.join(self._tmpdir, "payloads", "Physics", "physx.usda")
        physx_layer = Sdf.Layer.FindOrOpen(physx_path)
        if not physx_layer:
            errors.append(f"Failed to open physx layer: {physx_path}")
        else:
            physx_sublayers = list(physx_layer.subLayerPaths)
            if "./physics.usda" not in physx_sublayers:
                errors.append(f"physx.usda missing './physics.usda' sublayer, got: {physx_sublayers}")

        # Validate base sublayer
        base_layer = Sdf.Layer.FindOrOpen(base_usd)
        if not base_layer:
            errors.append(f"Failed to open base layer: {base_usd}")
        else:
            base_sublayers = list(base_layer.subLayerPaths)
            if "./robot.usda" not in base_sublayers:
                errors.append(f"base.usda missing './robot.usda' sublayer, got: {base_sublayers}")

        # Validate mujoco was skipped (does not exist)
        mujoco_path = os.path.join(self._tmpdir, "payloads", "Physics", "mujoco.usda")
        if os.path.exists(mujoco_path):
            errors.append("mujoco.usda should not exist but was found")

        # Validate mujoco skip is logged
        log = rule.get_operation_log()
        if not any("Asset layer not found" in msg and "mujoco" in msg for msg in log):
            errors.append("Operation log should indicate mujoco asset layer was not found")

        # Validate affected stages
        affected = rule.get_affected_stages()
        if len(affected) < 1:
            errors.append(f"Expected at least 1 affected stage, got {len(affected)}")

        # Report all errors
        if errors:
            self.fail("\n".join(errors))

        self._success = True

    async def test_default_variant_selections(self) -> None:
        """Validate default_variant_selections parameter sets variant selections."""
        errors = []

        base_usd = os.path.join(self._tmpdir, "payloads", "base.usda")
        stage = Usd.Stage.Open(base_usd)

        rule = InterfaceConnectionRule(
            source_stage=stage,
            package_root=self._tmpdir,
            destination_path="",
            args={
                "input_stage_path": base_usd,
                "interface_asset_name": "ur10e.usda",
                "params": {
                    "base_layer": "payloads/base.usda",
                    "base_connection_type": CONNECTION_REFERENCE,
                    "generate_folder_variants": True,
                    "payloads_folder": "payloads",
                    "default_variant_selections": {
                        "Physics": "physx",
                        "Gripper": "Robotiq_2F_85",
                    },
                },
            },
        )

        rule.process_rule()

        interface_path = os.path.join(self._tmpdir, "ur10e.usda")
        interface_layer = Sdf.Layer.FindOrOpen(interface_path)
        if not interface_layer:
            errors.append(f"Failed to open interface layer: {interface_path}")
        else:
            prim_spec = interface_layer.GetPrimAtPath("/ur10e")
            if not prim_spec:
                errors.append("Default prim /ur10e not found")
            else:
                selections = dict(prim_spec.variantSelections)
                if selections.get("Physics") != "physx":
                    errors.append(f"Expected Physics='physx', got '{selections.get('Physics')}'")
                if selections.get("Gripper") != "Robotiq_2F_85":
                    errors.append(f"Expected Gripper='Robotiq_2F_85', got '{selections.get('Gripper')}'")
                # Sensor should default to None since not specified
                if selections.get("Sensor") != "none":
                    errors.append(f"Expected Sensor='none' (default), got '{selections.get('Sensor')}'")

        if errors:
            self.fail("\n".join(errors))

        self._success = True

    async def test_physics_variant_defaults_to_none_without_clear_config(self) -> None:
        """Physics is no longer special-cased: without clear_default_variant_sets it defaults to 'none'."""
        base_usd = os.path.join(self._tmpdir, "payloads", "base.usda")
        stage = Usd.Stage.Open(base_usd)

        rule = InterfaceConnectionRule(
            source_stage=stage,
            package_root=self._tmpdir,
            destination_path="",
            args={
                "input_stage_path": base_usd,
                "interface_asset_name": "ur10e.usda",
                "params": {
                    "base_layer": "payloads/base.usda",
                    "base_connection_type": CONNECTION_REFERENCE,
                    "generate_folder_variants": True,
                    "payloads_folder": "payloads",
                },
            },
        )
        rule.process_rule()

        interface_path = os.path.join(self._tmpdir, "ur10e.usda")
        interface_layer = Sdf.Layer.FindOrOpen(interface_path)
        self.assertIsNotNone(interface_layer, f"Failed to open interface layer: {interface_path}")

        prim_spec = interface_layer.GetPrimAtPath("/ur10e")
        self.assertIsNotNone(prim_spec, "Default prim /ur10e not found")
        selections = dict(prim_spec.variantSelections)
        self.assertEqual(selections.get("Physics"), "none")
        self.assertEqual(selections.get("Sensor"), "none")

    async def test_clear_default_variant_sets_leaves_selection_empty(self) -> None:
        """Variant sets listed in clear_default_variant_sets stay unselected."""
        base_usd = os.path.join(self._tmpdir, "payloads", "base.usda")
        stage = Usd.Stage.Open(base_usd)

        rule = InterfaceConnectionRule(
            source_stage=stage,
            package_root=self._tmpdir,
            destination_path="",
            args={
                "input_stage_path": base_usd,
                "interface_asset_name": "ur10e.usda",
                "params": {
                    "base_layer": "payloads/base.usda",
                    "base_connection_type": CONNECTION_REFERENCE,
                    "generate_folder_variants": True,
                    "payloads_folder": "payloads",
                    "clear_default_variant_sets": ["Physics"],
                },
            },
        )
        rule.process_rule()

        interface_path = os.path.join(self._tmpdir, "ur10e.usda")
        interface_layer = Sdf.Layer.FindOrOpen(interface_path)
        self.assertIsNotNone(interface_layer, f"Failed to open interface layer: {interface_path}")

        prim_spec = interface_layer.GetPrimAtPath("/ur10e")
        self.assertIsNotNone(prim_spec, "Default prim /ur10e not found")
        selections = dict(prim_spec.variantSelections)
        self.assertNotIn("Physics", selections)
        # Sets not listed still default to 'none'.
        self.assertEqual(selections.get("Sensor"), "none")

    async def test_clear_default_variant_sets_overridden_by_explicit_default(self) -> None:
        """An explicit default_variant_selections entry wins over clear_default_variant_sets."""
        base_usd = os.path.join(self._tmpdir, "payloads", "base.usda")
        stage = Usd.Stage.Open(base_usd)

        rule = InterfaceConnectionRule(
            source_stage=stage,
            package_root=self._tmpdir,
            destination_path="",
            args={
                "input_stage_path": base_usd,
                "interface_asset_name": "ur10e.usda",
                "params": {
                    "base_layer": "payloads/base.usda",
                    "base_connection_type": CONNECTION_REFERENCE,
                    "generate_folder_variants": True,
                    "payloads_folder": "payloads",
                    "clear_default_variant_sets": ["Physics"],
                    "default_variant_selections": {"Physics": "physx"},
                },
            },
        )
        rule.process_rule()

        interface_path = os.path.join(self._tmpdir, "ur10e.usda")
        interface_layer = Sdf.Layer.FindOrOpen(interface_path)
        self.assertIsNotNone(interface_layer, f"Failed to open interface layer: {interface_path}")

        prim_spec = interface_layer.GetPrimAtPath("/ur10e")
        self.assertIsNotNone(prim_spec, "Default prim /ur10e not found")
        self.assertEqual(dict(prim_spec.variantSelections).get("Physics"), "physx")

    async def test_default_variant_selection_nonexistent_variant_not_set(self) -> None:
        """Validate that a requested default variant that does not exist is not selected."""
        errors = []

        base_usd = os.path.join(self._tmpdir, "payloads", "base.usda")
        stage = Usd.Stage.Open(base_usd)

        rule = InterfaceConnectionRule(
            source_stage=stage,
            package_root=self._tmpdir,
            destination_path="",
            args={
                "input_stage_path": base_usd,
                "interface_asset_name": "ur10e.usda",
                "params": {
                    "base_layer": "payloads/base.usda",
                    "base_connection_type": CONNECTION_REFERENCE,
                    "generate_folder_variants": True,
                    "payloads_folder": "payloads",
                    "default_variant_selections": {
                        # This variant does not exist in the Physics variant set
                        "Physics": "does_not_exist",
                    },
                },
            },
        )

        rule.process_rule()

        interface_path = os.path.join(self._tmpdir, "ur10e.usda")
        interface_layer = Sdf.Layer.FindOrOpen(interface_path)
        if not interface_layer:
            errors.append(f"Failed to open interface layer: {interface_path}")
        else:
            prim_spec = interface_layer.GetPrimAtPath("/ur10e")
            if not prim_spec:
                errors.append("Default prim /ur10e not found")
            else:
                selections = dict(prim_spec.variantSelections)
                # The nonexistent variant should not be selected at all
                if "Physics" in selections:
                    errors.append(
                        f"Physics selection should be unset for nonexistent variant, got '{selections.get('Physics')}'"
                    )

        # The rule should log that the requested variant was not found
        log = rule.get_operation_log()
        if not any("does_not_exist" in msg and "not found" in msg for msg in log):
            errors.append("Log should indicate the requested default variant was not found")

        if errors:
            self.fail("\n".join(errors))

        self._success = True

    async def test_connection_reference_type(self) -> None:
        """Validate connections with Reference connection type."""
        errors = []

        base_usd = os.path.join(self._tmpdir, "payloads", "base.usda")
        stage = Usd.Stage.Open(base_usd)

        rule = InterfaceConnectionRule(
            source_stage=stage,
            package_root=self._tmpdir,
            destination_path="",
            args={
                "input_stage_path": base_usd,
                "interface_asset_name": "ur10e.usda",
                "params": {
                    "base_layer": "payloads/base.usda",
                    "base_connection_type": CONNECTION_REFERENCE,
                    "generate_folder_variants": False,
                    "connections": [
                        {
                            "asset_path": "payloads/Physics/physx.usda",
                            "target_path": "payloads/Physics/physics.usda",
                            "connection_type": CONNECTION_REFERENCE,
                        },
                    ],
                },
            },
        )

        rule.process_rule()

        physx_path = os.path.join(self._tmpdir, "payloads", "Physics", "physx.usda")
        physx_layer = Sdf.Layer.FindOrOpen(physx_path)
        if not physx_layer:
            errors.append(f"Failed to open physx layer: {physx_path}")
        else:
            prim_spec = physx_layer.GetPrimAtPath("/ur10e")
            if not prim_spec:
                errors.append("Default prim /ur10e not found in physx layer")
            elif not prim_spec.hasReferences:
                errors.append("physx.usda /ur10e should have references after Reference connection")

        if errors:
            self.fail("\n".join(errors))
        self._success = True

    async def test_connection_payload_type(self) -> None:
        """Validate connections with Payload connection type."""
        errors = []

        base_usd = os.path.join(self._tmpdir, "payloads", "base.usda")
        stage = Usd.Stage.Open(base_usd)

        rule = InterfaceConnectionRule(
            source_stage=stage,
            package_root=self._tmpdir,
            destination_path="",
            args={
                "input_stage_path": base_usd,
                "interface_asset_name": "ur10e.usda",
                "params": {
                    "base_layer": "payloads/base.usda",
                    "base_connection_type": CONNECTION_REFERENCE,
                    "generate_folder_variants": False,
                    "connections": [
                        {
                            "asset_path": "payloads/Physics/physx.usda",
                            "target_path": "payloads/Physics/physics.usda",
                            "connection_type": CONNECTION_PAYLOAD,
                        },
                    ],
                },
            },
        )

        rule.process_rule()

        physx_path = os.path.join(self._tmpdir, "payloads", "Physics", "physx.usda")
        physx_layer = Sdf.Layer.FindOrOpen(physx_path)
        if not physx_layer:
            errors.append(f"Failed to open physx layer: {physx_path}")
        else:
            prim_spec = physx_layer.GetPrimAtPath("/ur10e")
            if not prim_spec:
                errors.append("Default prim /ur10e not found in physx layer")
            elif not prim_spec.hasPayloads:
                errors.append("physx.usda /ur10e should have payloads after Payload connection")

        if errors:
            self.fail("\n".join(errors))
        self._success = True

    async def test_connection_empty_asset_path_adds_to_interface(self) -> None:
        """Validate connection with empty asset_path adds to interface layer."""
        errors = []

        base_usd = os.path.join(self._tmpdir, "payloads", "base.usda")
        stage = Usd.Stage.Open(base_usd)

        rule = InterfaceConnectionRule(
            source_stage=stage,
            package_root=self._tmpdir,
            destination_path="",
            args={
                "input_stage_path": base_usd,
                "interface_asset_name": "ur10e.usda",
                "params": {
                    "base_layer": "payloads/base.usda",
                    "payloads_folder": "payloads",
                    "base_connection_type": CONNECTION_REFERENCE,
                    "generate_folder_variants": False,
                    "connections": [
                        {
                            "target_path": "payloads/robot.usda",
                            "connection_type": CONNECTION_SUBLAYER,
                        },
                    ],
                },
            },
        )

        rule.process_rule()

        interface_path = os.path.join(self._tmpdir, "ur10e.usda")
        interface_layer = Sdf.Layer.FindOrOpen(interface_path)
        if not interface_layer:
            errors.append(f"Failed to open interface layer: {interface_path}")
        else:
            sublayers = list(interface_layer.subLayerPaths)
            # Should have robot.usda as sublayer on interface (and base.usda from base_layer)
            has_robot = any("robot.usda" in sl for sl in sublayers)
            if not has_robot:
                errors.append(f"Interface layer should have robot.usda sublayer, got: {sublayers}")

        if errors:
            self.fail("\n".join(errors))
        self._success = True

    async def test_connection_invalid_connection_type_skipped(self) -> None:
        """Validate connection with invalid connection_type is skipped."""
        errors = []

        base_usd = os.path.join(self._tmpdir, "payloads", "base.usda")
        stage = Usd.Stage.Open(base_usd)

        rule = InterfaceConnectionRule(
            source_stage=stage,
            package_root=self._tmpdir,
            destination_path="",
            args={
                "input_stage_path": base_usd,
                "interface_asset_name": "ur10e.usda",
                "params": {
                    "base_layer": "payloads/base.usda",
                    "base_connection_type": CONNECTION_REFERENCE,
                    "generate_folder_variants": False,
                    "connections": [
                        {
                            "asset_path": "payloads/Physics/physx.usda",
                            "target_path": "payloads/Physics/physics.usda",
                            "connection_type": "InvalidType",
                        },
                    ],
                },
            },
        )

        rule.process_rule()

        log = rule.get_operation_log()
        if not any("Invalid connection type" in msg for msg in log):
            errors.append("Log should indicate invalid connection type was encountered")

        # physx.usda should not have physics.usda as sublayer since connection was skipped
        physx_path = os.path.join(self._tmpdir, "payloads", "Physics", "physx.usda")
        physx_layer = Sdf.Layer.FindOrOpen(physx_path)
        if physx_layer:
            sublayers = list(physx_layer.subLayerPaths)
            if "./physics.usda" in sublayers:
                errors.append("physx.usda should not have './physics.usda' sublayer (invalid connection skipped)")

        if errors:
            self.fail("\n".join(errors))
        self._success = True

    async def test_connection_missing_target_path_skipped(self) -> None:
        """Validate connection with missing target_path is skipped."""
        errors = []

        base_usd = os.path.join(self._tmpdir, "payloads", "base.usda")
        stage = Usd.Stage.Open(base_usd)

        rule = InterfaceConnectionRule(
            source_stage=stage,
            package_root=self._tmpdir,
            destination_path="",
            args={
                "input_stage_path": base_usd,
                "interface_asset_name": "ur10e.usda",
                "params": {
                    "base_layer": "payloads/base.usda",
                    "base_connection_type": CONNECTION_REFERENCE,
                    "generate_folder_variants": False,
                    "connections": [
                        {
                            "asset_path": "payloads/Physics/physx.usda",
                            "connection_type": CONNECTION_SUBLAYER,
                        },
                    ],
                },
            },
        )

        rule.process_rule()

        log = rule.get_operation_log()
        if not any("Missing target_path" in msg for msg in log):
            errors.append("Log should indicate missing target_path")

        if errors:
            self.fail("\n".join(errors))
        self._success = True

    async def test_connection_nonexistent_target_skipped(self) -> None:
        """Validate connection with nonexistent target_path is skipped."""
        errors = []

        base_usd = os.path.join(self._tmpdir, "payloads", "base.usda")
        stage = Usd.Stage.Open(base_usd)

        rule = InterfaceConnectionRule(
            source_stage=stage,
            package_root=self._tmpdir,
            destination_path="",
            args={
                "input_stage_path": base_usd,
                "interface_asset_name": "ur10e.usda",
                "params": {
                    "base_layer": "payloads/base.usda",
                    "base_connection_type": CONNECTION_REFERENCE,
                    "generate_folder_variants": False,
                    "connections": [
                        {
                            "asset_path": "payloads/Physics/physx.usda",
                            "target_path": "payloads/nonexistent.usda",
                            "connection_type": CONNECTION_SUBLAYER,
                        },
                    ],
                },
            },
        )

        rule.process_rule()

        log = rule.get_operation_log()
        if not any("Target path not found" in msg for msg in log):
            errors.append("Log should indicate target path not found")

        if errors:
            self.fail("\n".join(errors))
        self._success = True

    async def test_connection_non_dict_item_skipped(self) -> None:
        """Validate non-dict items in connections list are skipped."""
        errors = []

        base_usd = os.path.join(self._tmpdir, "payloads", "base.usda")
        stage = Usd.Stage.Open(base_usd)

        rule = InterfaceConnectionRule(
            source_stage=stage,
            package_root=self._tmpdir,
            destination_path="",
            args={
                "input_stage_path": base_usd,
                "interface_asset_name": "ur10e.usda",
                "params": {
                    "base_layer": "payloads/base.usda",
                    "base_connection_type": CONNECTION_REFERENCE,
                    "generate_folder_variants": False,
                    "connections": [
                        "invalid_string_item",
                        123,
                        None,
                    ],
                },
            },
        )

        rule.process_rule()

        log = rule.get_operation_log()
        invalid_count = sum(1 for msg in log if "not a dict" in msg)
        if invalid_count < 3:
            errors.append(f"Expected 3 'not a dict' log entries, got {invalid_count}")

        # Interface layer should still be created
        interface_path = os.path.join(self._tmpdir, "ur10e.usda")
        if not os.path.exists(interface_path):
            errors.append("Interface layer should still be created despite invalid connections")

        if errors:
            self.fail("\n".join(errors))
        self._success = True

    async def test_payloads_folder_not_found(self) -> None:
        """Validate generate_folder_variants with nonexistent payloads folder."""
        errors = []

        base_usd = os.path.join(self._tmpdir, "payloads", "base.usda")
        stage = Usd.Stage.Open(base_usd)

        rule = InterfaceConnectionRule(
            source_stage=stage,
            package_root=self._tmpdir,
            destination_path="",
            args={
                "input_stage_path": base_usd,
                "interface_asset_name": "ur10e.usda",
                "params": {
                    "base_layer": "payloads/base.usda",
                    "base_connection_type": CONNECTION_REFERENCE,
                    "generate_folder_variants": True,
                    "payloads_folder": "nonexistent_folder",
                },
            },
        )

        rule.process_rule()

        log = rule.get_operation_log()
        if not any("not found" in msg.lower() and "folder" in msg.lower() for msg in log):
            errors.append("Log should indicate payloads folder not found")

        # Interface should still be created with base connection
        interface_path = os.path.join(self._tmpdir, "ur10e.usda")
        if not os.path.exists(interface_path):
            errors.append("Interface layer should still be created")

        if errors:
            self.fail("\n".join(errors))
        self._success = True


class TestInterfacePackageReplacement(omni.kit.test.AsyncTestCase):
    """Use the real interface rule to verify reused package isolation."""

    async def test_stale_physics_files_and_interface_do_not_change_export(self) -> None:
        """NVBug 6380931: seeded Physics files must not create variants."""
        with tempfile.TemporaryDirectory() as directory:
            source_path = os.path.join(directory, "input.usda")
            source = Usd.Stage.CreateNew(source_path)
            source.SetDefaultPrim(source.DefinePrim("/Robot", "Xform"))
            source.GetRootLayer().Save()
            registry = RuleRegistry()
            registry.register(InterfaceConnectionRule)
            profile = RuleProfile(
                profile_name="replacement",
                base_name="base.usda",
                interface_asset_name="robot.usda",
                rules=[
                    RuleSpec(
                        name="Generate interface",
                        type=f"{InterfaceConnectionRule.__module__}.{InterfaceConnectionRule.__qualname__}",
                        params={"generate_folder_variants": True},
                    )
                ],
            )
            manager = AssetTransformerManager(registry)
            clean = manager.run(source_path, profile, os.path.join(directory, "clean"))
            clean_layer = Sdf.Layer.FindOrOpen(clean.output_stage_path)
            expected = clean_layer.ExportToString()
            clean_layer = None
            output = os.path.join(directory, "reused")
            stale_path = os.path.join(output, "payloads", "Physics", "mujoco.usda")
            os.makedirs(os.path.dirname(stale_path))
            stale = Sdf.Layer.CreateNew(stale_path)
            Sdf.CreatePrimInLayer(stale, "/Robot/Stale")
            stale.defaultPrim = "Robot"
            stale.Save()
            stale = None
            with open(stale_path, "rb") as stream:
                stale_bytes = stream.read()
            # Leave both an obsolete interface variant and a cached output
            # layer behind, as in an earlier export still open in the UI.
            old = Usd.Stage.CreateNew(os.path.join(output, "robot.usda"))
            old.SetDefaultPrim(old.DefinePrim("/Robot", "Xform"))
            old.GetDefaultPrim().GetVariantSets().AddVariantSet("Physics").AddVariant("obsolete")
            old.GetRootLayer().Save()
            for _ in range(2):
                report = manager.run(source_path, profile, output)
                self.assertTrue(all(result.success for result in report.results))
                self.assertEqual(report.composition_errors, [])
                self.assertEqual(old.GetRootLayer().ExportToString(), expected)
                self.assertEqual(old.GetDefaultPrim().GetVariantSet("Physics").GetVariantNames(), [])
                self.assertTrue(
                    all(os.path.isfile(path) for result in report.results for path in result.affected_stages)
                )
                with open(stale_path, "rb") as stream:
                    self.assertEqual(stream.read(), stale_bytes)
            # Re-export the open interface into its own package, resolving its
            # existing dependencies before replacing any of them.
            profile.flatten_source = True
            report = manager.run(old, profile, output)
            self.assertTrue(all(result.success for result in report.results))
            self.assertEqual(report.composition_errors, [])
            self.assertEqual(old.GetRootLayer().ExportToString(), expected)
            old = None
            source = None

    async def test_default_profile_ignores_stale_physics_payload(self) -> None:
        """Cover the public-API UR10e reproduction from NVBug 6380931."""
        register_all_rules()
        with open(DEFAULT_PROFILE_PATH, encoding="utf-8") as stream:
            profile_json = stream.read()
        with tempfile.TemporaryDirectory() as directory:
            clean_root = os.path.join(directory, "clean")
            stale_root = os.path.join(directory, "stale")
            stale_path = os.path.join(stale_root, "payloads", "Physics", "mujoco.usda")
            os.makedirs(os.path.dirname(stale_path))
            stale = Sdf.Layer.CreateNew(stale_path)
            stale.ImportFromString('#usda 1.0\n(defaultPrim="STALE")\ndef Xform "STALE" {}\n')
            stale.Save()
            # Keep the old payload cached as in the reported reproduction.
            expected = None
            for output in (clean_root, stale_root, stale_root):
                with self.subTest(output=output):
                    report = AssetTransformerManager().run(
                        _UR10E_SHOULDER_USD, RuleProfile.from_json(profile_json), package_root=output
                    )
                    self.assertEqual([(r.rule.name, r.error) for r in report.results if not r.success], [])
                    self.assertEqual(report.composition_errors, [])
                    self.assertEqual(report.output_stage_path, os.path.join(output, "ur10e.usda"))
                    # Read disk contents independently of any cached output layer.
                    layer = Sdf.Layer.CreateAnonymous("interface.usda")
                    self.assertTrue(layer.Import(report.output_stage_path))
                    prim = layer.GetPrimAtPath("/" + layer.defaultPrim)
                    self.assertEqual([name for name, _ in prim.variantSets.items() if "hysics" in name], [])
                    contents = layer.ExportToString()
                    if expected is None:
                        expected = contents
                    else:
                        self.assertEqual(contents, expected)
                    prim = None
                    layer = None
            stale = None
