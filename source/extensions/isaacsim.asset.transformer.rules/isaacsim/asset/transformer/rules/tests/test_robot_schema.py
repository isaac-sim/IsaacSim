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

"""Tests for the robot schema rule."""

import os
import shutil
import tempfile

import omni.kit.test
from isaacsim.asset.transformer.rules.isaac_sim.robot_schema import RobotSchemaRule
from pxr import Sdf, Usd

from .common import _TEST_DATA_DIR

# Use test_prims/base.usda which has physics joints but no robot schema pre-applied

_TEST_DATA_INTERFACE_DIR = os.path.join(_TEST_DATA_DIR, "ur10e_interface")

# Default parameters for RobotSchemaRule tests
_DEFAULT_PARAMS = {
    "prim_path": None,
    "stage_name": "robot_schema.usda",
    "add_sites": True,
    "sites_last": False,
    "sublayer": "Physics/physics.usda",
}

# Common path lists used across tests
_BASE_LINK_PATHS = [
    "/ur10e/base_link",
    "/ur10e/shoulder_link",
    "/ur10e/upper_arm_link",
    "/ur10e/forearm_link",
    "/ur10e/wrist_1_link",
    "/ur10e/wrist_2_link",
    "/ur10e/wrist_3_link",
]

_JOINT_PATHS = [
    "/ur10e/joints/shoulder_pan_joint",
    "/ur10e/joints/shoulder_lift_joint",
    "/ur10e/joints/elbow_joint",
    "/ur10e/joints/wrist_1_joint",
    "/ur10e/joints/wrist_2_joint",
    "/ur10e/joints/wrist_3_joint",
]


class TestRobotSchemaRule(omni.kit.test.AsyncTestCase):
    """Async tests for RobotSchemaRule."""

    async def setUp(self) -> None:
        """Create a temporary test environment and load the base stage."""
        self._tmpdir = tempfile.mkdtemp()
        self._setup_test_structure()
        self._success = False
        self.stage = Usd.Stage.Open(self._base_usd_path)
        self.log = None

    def _setup_test_structure(self) -> None:
        """Populate the temporary payloads directory for tests."""
        # Create payloads directory with a flattened base layer
        # Copy the payloads directory from the UR10E_INTERFACE_USD to the temp directory
        src_payloads_dir = os.path.join(_TEST_DATA_INTERFACE_DIR, "payloads")
        payloads_dir = os.path.join(self._tmpdir, "payloads")
        if os.path.exists(src_payloads_dir):
            shutil.copytree(src_payloads_dir, payloads_dir)
        else:
            self.fail(f"Payloads directory not found: {src_payloads_dir}")
        self._base_usd_path = os.path.join(self._tmpdir, "payloads", "base.usda")

        # Remove the robot.usda sublayer from base.usda so we test applying schema to clean asset
        base_layer = Sdf.Layer.FindOrOpen(self._base_usd_path)
        if base_layer:
            sublayers = list(base_layer.subLayerPaths)
            sublayers = [s for s in sublayers if "robot.usda" not in s]
            base_layer.subLayerPaths = sublayers
            base_layer.Save()

    async def tearDown(self) -> None:
        """Clean up the temporary directory or persist diagnostics."""
        if self._success:
            shutil.rmtree(self._tmpdir, ignore_errors=True)
        else:
            print(self.log)
            self.stage.Save()

    def _get_params(self, **overrides: object) -> dict[str, object]:
        """Return default params with specified overrides applied.

        Args:
            **overrides: Parameter overrides keyed by parameter name.

        Returns:
            Dictionary of parameter values.

        """
        params = _DEFAULT_PARAMS.copy()
        params.update(overrides)
        return params

    def _create_rule(self, stage: Usd.Stage, **param_overrides: object) -> RobotSchemaRule:
        """Create a RobotSchemaRule with default params and specified overrides.

        Args:
            stage: Source stage for the rule.
            **param_overrides: Parameter overrides keyed by parameter name.

        Returns:
            Configured RobotSchemaRule instance.

        """
        params = self._get_params(**param_overrides)
        return RobotSchemaRule(
            source_stage=stage,
            package_root=self._tmpdir,
            destination_path="payloads",
            args={"params": params},
        )

    def _get_output_path(self, stage_name: str | None = None) -> str:
        """Get the output path for the robot schema file.

        Args:
            stage_name: Optional stage filename to override defaults.

        Returns:
            Absolute output path for the robot schema file.

        """
        name = stage_name or _DEFAULT_PARAMS["stage_name"]
        return os.path.join(self._tmpdir, "payloads", name)

    def _run_rule(self, rule: RobotSchemaRule) -> None:
        """Run the rule and capture the operation log.

        Args:
            rule: Rule instance to execute.

        """
        rule.process_rule()
        self.log = rule.get_operation_log()
        return

    def _get_output_layer(self, stage_name: str | None = None) -> Sdf.Layer:
        """Get the output layer and assert it exists.

        Args:
            stage_name: Optional stage filename to override defaults.

        Returns:
            Opened output layer.

        """
        output_path = self._get_output_path(stage_name)
        output_layer = Sdf.Layer.FindOrOpen(output_path)
        self.assertIsNotNone(output_layer)
        return output_layer

    def _get_api_schemas(self, layer: Sdf.Layer, prim_path: str) -> list[object]:
        """Get the list of API schemas applied to a prim.

        Args:
            layer: Layer to inspect for the prim spec.
            prim_path: Path to the prim in the layer.

        Returns:
            List of applied schema tokens.

        """
        prim_spec = layer.GetPrimAtPath(prim_path)
        if not prim_spec:
            return []
        api_schemas = prim_spec.GetInfo("apiSchemas")
        if not api_schemas:
            return []
        return list(api_schemas.prependedItems or [])

    def _has_api_schema(self, layer: Sdf.Layer, prim_path: str, schema_name: str) -> bool:
        """Check if a prim has a specific API schema applied.

        Args:
            layer: Layer to inspect for the prim spec.
            prim_path: Path to the prim in the layer.
            schema_name: API schema token to look for.

        Returns:
            True if the schema token is present.

        """
        schemas = self._get_api_schemas(layer, prim_path)
        return any(schema_name in str(item) for item in schemas)

    def _get_relationship_targets(self, layer: Sdf.Layer, prim_path: str, rel_name: str) -> list[Sdf.Path]:
        """Get the target paths from a relationship.

        Args:
            layer: Layer to inspect for the prim spec.
            prim_path: Path to the prim in the layer.
            rel_name: Relationship name to query.

        Returns:
            List of relationship target paths.

        """
        prim_spec = layer.GetPrimAtPath(prim_path)
        if not prim_spec:
            return []
        rel = prim_spec.relationships.get(rel_name)
        if not rel:
            return []
        return list(rel.targetPathList.prependedItems)

    def _log_contains(self, substring: str, case_sensitive: bool = True) -> bool:
        """Check if any log message contains the given substring.

        Args:
            substring: Substring to search for in log messages.
            case_sensitive: Whether to match case-sensitively.

        Returns:
            True if any log line contains the substring.

        """
        if case_sensitive:
            return any(substring in msg for msg in self.log)
        return any(substring.lower() in msg.lower() for msg in self.log)

    def _run_default_rule(self, **param_overrides: object) -> RobotSchemaRule:
        """Create and run a rule with default stage, returning the rule.

        Args:
            **param_overrides: Parameter overrides keyed by parameter name.

        Returns:
            Executed RobotSchemaRule instance.

        """
        rule = self._create_rule(self.stage, **param_overrides)
        self._run_rule(rule)
        return rule

    async def test_get_configuration_parameters(self) -> None:
        """Verify configuration parameters are exposed."""
        rule = self._create_rule(self.stage)

        params = rule.get_configuration_parameters()

        self.assertEqual(len(params), 5)
        param_names = [p.name for p in params]
        self.assertIn("prim_path", param_names)
        self.assertIn("stage_name", param_names)
        self.assertIn("add_sites", param_names)
        self.assertIn("sites_last", param_names)
        self.assertIn("sublayer", param_names)
        self._success = True

    async def test_process_rule_adds_affected_stage(self) -> None:
        """Verify affected stages are recorded."""
        rule = self._run_default_rule()
        self.assertGreater(len(rule.get_affected_stages()), 0)
        self._success = True

    async def test_process_rule_applies_joint_api_to_joints(self) -> None:
        """Verify joint API schemas are applied to joint prims."""
        self._run_default_rule()
        output_layer = self._get_output_layer()
        found_joint_api = any(self._has_api_schema(output_layer, p, "IsaacJointAPI") for p in _JOINT_PATHS)
        self.assertTrue(found_joint_api)
        self._success = True

    async def test_process_rule_applies_link_api_to_links(self) -> None:
        """Verify link API schemas are applied to link prims."""
        self._run_default_rule()
        output_layer = self._get_output_layer()
        found_link_api = any(self._has_api_schema(output_layer, p, "IsaacLinkAPI") for p in _BASE_LINK_PATHS)
        self.assertTrue(found_link_api)
        self._success = True

    async def test_process_rule_applies_robot_api(self) -> None:
        """Verify robot API schema is applied to the root prim."""
        self._run_default_rule()
        self.assertTrue(os.path.exists(self._get_output_path()))
        output_layer = self._get_output_layer()
        self.assertTrue(self._has_api_schema(output_layer, "/ur10e", "IsaacRobotAPI"))
        self._success = True

    async def test_process_rule_detects_root_link(self) -> None:
        """Verify root link detection logs a message."""
        self._run_default_rule()
        self.assertTrue(self._log_contains("root link", case_sensitive=False))
        self._success = True

    async def test_process_rule_invalid_prim_path_skips(self) -> None:
        """Verify invalid prim path causes the rule to skip."""
        self._run_default_rule(prim_path="/nonexistent/prim")
        self.assertTrue(self._log_contains("skipped", case_sensitive=False))
        self._success = True

    async def test_process_rule_logs_completion(self) -> None:
        """Verify start and completion logs are recorded."""
        self._run_default_rule()
        self.assertTrue(self._log_contains("RobotSchemaRule start"))
        self.assertTrue(self._log_contains("RobotSchemaRule completed"))
        self._success = True

    async def test_process_rule_populates_links_and_joints(self) -> None:
        """Verify robot link and joint relationships are populated."""
        self._run_default_rule()
        output_layer = self._get_output_layer()

        expected_links = [Sdf.Path(p) for p in _BASE_LINK_PATHS]
        expected_links.insert(1, Sdf.Path("/ur10e/base_link/base_link"))
        expected_links.insert(3, Sdf.Path("/ur10e/shoulder_link/sensor_mount"))
        expected_links.extend([Sdf.Path("/ur10e/wrist_3_link/flange"), Sdf.Path("/ur10e/wrist_3_link/ft_frame")])

        expected_joints = [Sdf.Path("/ur10e/root_joint")] + [Sdf.Path(p) for p in _JOINT_PATHS]

        link_targets = self._get_relationship_targets(output_layer, "/ur10e", "isaac:physics:robotLinks")
        joint_targets = self._get_relationship_targets(output_layer, "/ur10e", "isaac:physics:robotJoints")

        self.assertEqual(link_targets, expected_links)
        self.assertEqual(joint_targets, expected_joints)
        self._success = True

    async def test_process_rule_returns_none(self) -> None:
        """Verify process_rule returns None."""
        rule = self._create_rule(self.stage)
        result = rule.process_rule()
        self.log = rule.get_operation_log()
        self.assertIsNone(result)
        self._success = True

    async def test_process_rule_uses_default_prim_when_no_prim_path(self) -> None:
        """Verify default prim is used when prim_path is None."""
        self._run_default_rule(prim_path=None)
        self.assertTrue(self._log_contains("/ur10e"))
        self._success = True

    async def test_process_rule_with_add_sites_disabled(self) -> None:
        """Verify sites are not added when add_sites is False."""
        self._run_default_rule(add_sites=False)
        self.assertTrue(self._log_contains("add_sites=False"))

        output_layer = self._get_output_layer()
        expected_links = [Sdf.Path(p) for p in _BASE_LINK_PATHS]
        link_targets = self._get_relationship_targets(output_layer, "/ur10e", "isaac:physics:robotLinks")
        self.assertEqual(link_targets, expected_links)
        self._success = True

    async def test_process_rule_with_add_sites_enabled(self) -> None:
        """Verify sites are added when add_sites is True."""
        self._run_default_rule(add_sites=True)
        self.assertTrue(self._log_contains("add_sites=True"))

        output_layer = self._get_output_layer()

        # Verify site API applied to flange
        self.assertTrue(
            self._has_api_schema(output_layer, "/ur10e/wrist_3_link/flange", "IsaacSiteAPI"),
            "IsaacSiteAPI not found on flange prim",
        )
        # Verify visuals prim not present in output layer
        self.assertIsNone(output_layer.GetPrimAtPath("/ur10e/wrist_3_link/visuals"))

        expected_links = [Sdf.Path(p) for p in _BASE_LINK_PATHS]
        expected_links.insert(1, Sdf.Path("/ur10e/base_link/base_link"))
        expected_links.insert(3, Sdf.Path("/ur10e/shoulder_link/sensor_mount"))
        expected_links.extend([Sdf.Path("/ur10e/wrist_3_link/flange"), Sdf.Path("/ur10e/wrist_3_link/ft_frame")])

        link_targets = self._get_relationship_targets(output_layer, "/ur10e", "isaac:physics:robotLinks")
        self.assertEqual(link_targets, expected_links)
        self._success = True

    async def test_process_rule_with_custom_stage_name(self) -> None:
        """Verify custom stage name output is created."""
        custom_stage_name = "custom_robot.usda"
        self._run_default_rule(stage_name=custom_stage_name)
        self.assertTrue(os.path.exists(self._get_output_path(custom_stage_name)))
        self._success = True

    async def test_process_rule_with_explicit_prim_path(self) -> None:
        """Verify explicit prim path is respected."""
        self._run_default_rule(prim_path="/ur10e")
        self.assertTrue(self._log_contains("prim=/ur10e"))
        self._success = True

    async def test_process_rule_with_sites_last(self) -> None:
        """Verify sites_last orders sites after links."""
        self._run_default_rule(add_sites=True, sites_last=True)

        # Sites at the end instead of interleaved
        expected_links = [Sdf.Path(p) for p in _BASE_LINK_PATHS] + [
            Sdf.Path("/ur10e/base_link/base_link"),
            Sdf.Path("/ur10e/shoulder_link/sensor_mount"),
            Sdf.Path("/ur10e/wrist_3_link/flange"),
            Sdf.Path("/ur10e/wrist_3_link/ft_frame"),
        ]
        output_layer = self._get_output_layer()
        link_targets = self._get_relationship_targets(output_layer, "/ur10e", "isaac:physics:robotLinks")
        self.assertEqual(link_targets, expected_links)
        self.assertTrue(self._log_contains("sites_last=True"))
        self._success = True

    async def test_process_rule_sublayer_path_is_relative_forward_slash(self) -> None:
        """Verify the destination layer is added as a relative forward-slash sublayer path.

        Regression test for Windows where os.path produces backslash paths and
        raw layer identifiers may not resolve correctly as sublayer references.
        The rule must use a relative path with forward slashes so that
        Usd.EditContext can locate the layer in the stage's local layer stack.

        The root layer is intentionally reloaded after processing to discard
        temporary sublayer edits, so we verify the path format via the
        operation log rather than inspecting the final sublayer list.
        """
        self._run_default_rule()

        # The rule logs the sublayer path it added; extract and validate it.
        sublayer_log = [msg for msg in self.log if "Added robot schema layer as sublayer:" in msg]
        self.assertEqual(len(sublayer_log), 1, f"Expected one sublayer log entry, got: {sublayer_log}")
        logged_path = sublayer_log[0].split("Added robot schema layer as sublayer:")[-1].strip()
        self.assertNotIn("\\", logged_path, "Sublayer path must use forward slashes")
        self.assertFalse(os.path.isabs(logged_path), "Sublayer path should be relative")

        # Verify the edit context succeeded by checking the output layer has content
        output_layer = self._get_output_layer()
        self.assertTrue(self._has_api_schema(output_layer, "/ur10e", "IsaacRobotAPI"))
        self._success = True

    async def test_process_rule_update_deprecated_schemas(self) -> None:
        """Verify deprecated schemas are updated and reordered."""
        custom_stage_name = "robot.usda"
        self._run_default_rule(stage_name=custom_stage_name, update_deprecated_schemas=True)

        output_layer = self._get_output_layer(custom_stage_name)
        self.assertTrue(self._log_contains("Updated deprecated schemas"))

        # Verify IsaacReferencePointAPI replaced with IsaacSiteAPI on flange
        self.assertFalse(self._has_api_schema(output_layer, "/ur10e/wrist_3_link/flange", "IsaacReferencePointAPI"))
        self.assertTrue(self._has_api_schema(output_layer, "/ur10e/wrist_3_link/flange", "IsaacSiteAPI"))

        # Check wrist_2_joint (PhysicsRevoluteJoint) - deprecated attr removed, no DofOffsetOpOrder
        joint_spec = output_layer.GetPrimAtPath("/ur10e/joints/wrist_2_joint")
        self.assertIsNotNone(joint_spec)
        self.assertNotIn("isaac:physics:Rot_X:DoFOffset", joint_spec.attributes)
        self.assertNotIn("isaac:physics:DofOffsetOpOrder", joint_spec.attributes)

        # Check wrist_3_joint (PhysicsJoint) - deprecated attr removed, DofOffsetOpOrder added
        joint_spec = output_layer.GetPrimAtPath("/ur10e/joints/wrist_3_joint")
        self.assertIsNotNone(joint_spec)
        self.assertNotIn("isaac:physics:Rot_X:DoFOffset", joint_spec.attributes)
        self.assertIn("isaac:physics:DofOffsetOpOrder", joint_spec.attributes)

        # Verify DofOffsetOpOrder value
        output_stage = Usd.Stage.Open(self._get_output_path(custom_stage_name))
        dof_offset_attr = output_stage.GetPrimAtPath("/ur10e/joints/wrist_3_joint").GetAttribute(
            "isaac:physics:DofOffsetOpOrder"
        )
        self.assertTrue(dof_offset_attr.HasAuthoredValue())
        self.assertEqual(list(dof_offset_attr.Get()), ["RotX", "RotZ", "RotY"])

        # Verify links and joints - deprecated schema update reorders based on joint hierarchy
        expected_links = [
            Sdf.Path("/ur10e/base_link"),
            Sdf.Path("/ur10e/base_link/base_link"),
            Sdf.Path("/ur10e/shoulder_link"),
            Sdf.Path("/ur10e/upper_arm_link"),
            Sdf.Path("/ur10e/forearm_link"),
            Sdf.Path("/ur10e/wrist_3_link"),
            Sdf.Path("/ur10e/wrist_2_link"),
            Sdf.Path("/ur10e/wrist_1_link"),
            Sdf.Path("/ur10e/wrist_3_link/flange"),
            Sdf.Path("/ur10e/shoulder_link/sensor_mount"),
            Sdf.Path("/ur10e/wrist_3_link/ft_frame"),
        ]
        expected_joints = [
            Sdf.Path("/ur10e/root_joint"),
            Sdf.Path("/ur10e/joints/shoulder_pan_joint"),
            Sdf.Path("/ur10e/joints/shoulder_lift_joint"),
            Sdf.Path("/ur10e/joints/elbow_joint"),
            Sdf.Path("/ur10e/joints/wrist_3_joint"),
            Sdf.Path("/ur10e/joints/wrist_2_joint"),
            Sdf.Path("/ur10e/joints/wrist_1_joint"),
        ]

        link_targets = self._get_relationship_targets(output_layer, "/ur10e", "isaac:physics:robotLinks")
        joint_targets = self._get_relationship_targets(output_layer, "/ur10e", "isaac:physics:robotJoints")
        self.assertEqual(link_targets, expected_links)
        self.assertEqual(joint_targets, expected_joints)
        self._success = True
