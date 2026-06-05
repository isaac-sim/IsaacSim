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

"""Tests for variant routing rule behavior."""

import os
import re
import shutil
import tempfile

import omni.kit.test
from isaacsim.asset.transformer.rules.structure.variants import VariantRoutingRule
from isaacsim.asset.transformer.rules.utils import sanitize_prim_name
from pxr import Sdf, Usd

from .common import _TEST_DATA_DIR

_G1_USD = os.path.join(_TEST_DATA_DIR, "G1", "g1.usda")
_EXCLUDED_VARIANTS = ["none", "default", "physx"]
_ASSET_PATH_RE = re.compile(r"@([^@]+)@")

_EXPECTED_DEPENDENCIES = {
    "left_hand": {
        "inspire_left_base.usda",
        "inspire_left_hand.usda",
        "inspire_left_physics.usda",
        "inspire_left_robot.usda",
        "three_finger_hand_base_left.usda",
        "three_finger_hand_physics_left.usda",
        "three_fingers_left_hand_robot.usda",
        "three_fingers_left_hand.usda",
    },
    "right_hand": {
        "inspire_right_base.usda",
        "inspire_right_hand.usda",
        "inspire_right_physics.usda",
        "inspire_right_robot.usda",
        "three_finger_hand_base_right.usda",
        "three_finger_hand_physics_right.usda",
        "three_fingers_right_hand_robot.usda",
        "three_fingers_right_hand.usda",
    },
    "Thor": {
        "DefaultMaterial.mdl",
        "g1_29dof_NVBP_base.usda",
        "g1_29dof_NVBP_physics.usda",
        "g1_29dof_NVBP_robot.usda",
        "g1_29dof_NVBP_sensor.usda",
        "g1_29dof_NVBP.usda",
    },
}


class TestVariantRoutingRule(omni.kit.test.AsyncTestCase):
    """Async tests for the variant routing rule."""

    async def setUp(self) -> None:
        """Create a temporary directory for test output."""
        self._tmpdir = tempfile.mkdtemp()
        self._success = False

    async def tearDown(self) -> None:
        """Remove temporary directories after successful tests."""
        if self._success:
            shutil.rmtree(self._tmpdir, ignore_errors=True)

    def _variant_set_dir(self, variant_set_name: str) -> str:
        """Get the output directory for a variant set.

        Args:
            variant_set_name: Variant set name to sanitize.

        Returns:
            Absolute path to the variant set output directory.

        """
        return os.path.join(self._tmpdir, "payloads", sanitize_prim_name(variant_set_name))

    def _variant_file_path(self, variant_set_name: str, variant_name: str) -> str:
        """Get the output file path for a specific variant.

        Args:
            variant_set_name: Variant set containing the variant.
            variant_name: Variant option name.

        Returns:
            Absolute path to the variant USDA file.

        """
        variant_file = f"{sanitize_prim_name(variant_name).lower()}.usda"
        return os.path.join(self._variant_set_dir(variant_set_name), variant_file)

    def _layer_asset_paths(self, layer_path: str) -> list[str]:
        """Extract referenced asset paths from a layer.

        Args:
            layer_path: Path to the layer file to scan.

        Returns:
            List of asset paths referenced by the layer.

        """
        layer = Sdf.Layer.FindOrOpen(layer_path)
        self.assertIsNotNone(layer)
        return _ASSET_PATH_RE.findall(layer.ExportToString())

    def _assert_dependency_assets_exist(self, layer_path: str) -> None:
        """Assert dependency assets exist next to a layer.

        Args:
            layer_path: Path to the variant layer file.

        """
        layer_dir = os.path.dirname(layer_path)
        for asset_path in self._layer_asset_paths(layer_path):
            normalized = asset_path.replace("\\", "/")
            if "dependencies/" not in normalized:
                continue
            relative = normalized.lstrip("./")
            abs_path = os.path.normpath(os.path.join(layer_dir, relative))
            self.assertTrue(os.path.exists(abs_path), f"Missing dependency asset: {abs_path}")

    async def test_get_configuration_parameters(self) -> None:
        """Verify configuration parameters are exposed by the rule."""
        stage = Usd.Stage.Open(_G1_USD)
        rule = VariantRoutingRule(
            source_stage=stage,
            package_root=self._tmpdir,
            destination_path="payloads",
            args={"input_stage_path": _G1_USD},
        )

        params = rule.get_configuration_parameters()

        self.assertEqual(len(params), 4)
        param_names = [p.name for p in params]
        self.assertIn("variant_sets", param_names)
        self.assertIn("case_insensitive", param_names)
        self.assertIn("collect_dependencies", param_names)
        self.assertIn("excluded_variants", param_names)
        self._success = True

    async def test_process_rule_creates_variants_and_dependencies(self) -> None:
        """Verify variant files and dependencies are generated."""
        stage = Usd.Stage.Open(_G1_USD)
        rule = VariantRoutingRule(
            source_stage=stage,
            package_root=self._tmpdir,
            destination_path="payloads",
            args={
                "input_stage_path": _G1_USD,
                "params": {"excluded_variants": _EXCLUDED_VARIANTS},
            },
        )

        rule.process_rule()

        default_prim = stage.GetDefaultPrim()
        self.assertTrue(default_prim.IsValid())
        variant_sets = default_prim.GetVariantSets()
        variant_set_names = variant_sets.GetNames()
        self.assertGreater(len(variant_set_names), 0)

        for variant_set_name in variant_set_names:
            output_dir = self._variant_set_dir(variant_set_name)
            self.assertTrue(os.path.isdir(output_dir))
            variant_set = variant_sets.GetVariantSet(variant_set_name)
            for variant_name in variant_set.GetVariantNames():
                variant_path = self._variant_file_path(variant_set_name, variant_name)
                self.assertTrue(os.path.isfile(variant_path))
                self._assert_dependency_assets_exist(variant_path)

        for variant_set_name, expected_files in _EXPECTED_DEPENDENCIES.items():
            dependencies_dir = os.path.join(self._variant_set_dir(variant_set_name), "dependencies")
            self.assertTrue(os.path.isdir(dependencies_dir))
            self.assertEqual(set(os.listdir(dependencies_dir)), expected_files)

        self._success = True

    async def test_process_rule_no_default_prim_and_logging(self) -> None:
        """No-default-prim skip, affected-stages tracking, start/completion log entries."""
        failures = []

        # -- No default prim --
        no_prim_dir = tempfile.mkdtemp()
        try:
            layer_path = os.path.join(no_prim_dir, "empty.usda")
            empty_layer = Sdf.Layer.CreateNew(layer_path)
            empty_layer.Save()
            empty_stage = Usd.Stage.Open(layer_path)
            rule = VariantRoutingRule(
                source_stage=empty_stage,
                package_root=self._tmpdir,
                destination_path="payloads",
                args={"input_stage_path": layer_path},
            )
            rule.process_rule()
            log = rule.get_operation_log()
            if not any("No valid default prim" in m for m in log):
                failures.append("No-default-prim skip not logged")
        finally:
            shutil.rmtree(no_prim_dir, ignore_errors=True)

        # -- Logging and affected stages on real asset --
        stage = Usd.Stage.Open(_G1_USD)
        rule2 = VariantRoutingRule(
            source_stage=stage,
            package_root=self._tmpdir,
            destination_path="payloads",
            args={
                "input_stage_path": _G1_USD,
                "params": {"excluded_variants": _EXCLUDED_VARIANTS},
            },
        )
        rule2.process_rule()

        log2 = rule2.get_operation_log()
        if not any("VariantRoutingRule start" in m for m in log2):
            failures.append("Missing start log entry")
        if not any("VariantRoutingRule completed" in m for m in log2):
            failures.append("Missing completion log entry")

        affected = rule2.get_affected_stages()
        if not affected:
            failures.append("No affected stages recorded")

        self.assertEqual(failures, [], "\n".join(failures))
        self._success = True

    async def test_process_rule_options(self) -> None:
        """variant_sets filter, case_insensitive=False, collect_dependencies=False."""
        stage = Usd.Stage.Open(_G1_USD)
        default_prim = stage.GetDefaultPrim()
        all_vs_names = default_prim.GetVariantSets().GetNames()
        failures = []

        # -- Variant-set filter: only first set --
        filter_dir = tempfile.mkdtemp()
        try:
            target_vs = all_vs_names[0]
            rule_filter = VariantRoutingRule(
                source_stage=stage,
                package_root=filter_dir,
                destination_path="payloads",
                args={
                    "input_stage_path": _G1_USD,
                    "params": {
                        "variant_sets": [target_vs],
                        "excluded_variants": _EXCLUDED_VARIANTS,
                    },
                },
            )
            rule_filter.process_rule()
            output_base = os.path.join(filter_dir, "payloads")
            created_dirs = [d for d in os.listdir(output_base) if os.path.isdir(os.path.join(output_base, d))]
            if len(created_dirs) != 1:
                failures.append(f"variant_sets filter: expected 1 dir, got {len(created_dirs)}: {created_dirs}")
        finally:
            shutil.rmtree(filter_dir, ignore_errors=True)

        # -- case_insensitive=False: at least one filename with uppercase --
        case_dir = tempfile.mkdtemp()
        try:
            rule_case = VariantRoutingRule(
                source_stage=stage,
                package_root=case_dir,
                destination_path="payloads",
                args={
                    "input_stage_path": _G1_USD,
                    "params": {
                        "case_insensitive": False,
                        "excluded_variants": ["None", "Default", "PhysX"],
                    },
                },
            )
            rule_case.process_rule()
            found_upper = False
            for root, _dirs, files in os.walk(os.path.join(case_dir, "payloads")):
                for f in files:
                    if f.endswith(".usda") and f != f.lower():
                        found_upper = True
                        break
            if not found_upper:
                failures.append("case_insensitive=False: no uppercase filenames found")
        finally:
            shutil.rmtree(case_dir, ignore_errors=True)

        # -- collect_dependencies=False: no dependencies/ dirs --
        nodep_dir = tempfile.mkdtemp()
        try:
            rule_nodep = VariantRoutingRule(
                source_stage=stage,
                package_root=nodep_dir,
                destination_path="payloads",
                args={
                    "input_stage_path": _G1_USD,
                    "params": {
                        "collect_dependencies": False,
                        "excluded_variants": _EXCLUDED_VARIANTS,
                    },
                },
            )
            rule_nodep.process_rule()
            for vs_name in all_vs_names:
                dep_dir = os.path.join(nodep_dir, "payloads", sanitize_prim_name(vs_name), "dependencies")
                if os.path.isdir(dep_dir):
                    failures.append(f"collect_dependencies=False: {dep_dir} should not exist")
        finally:
            shutil.rmtree(nodep_dir, ignore_errors=True)

        self.assertEqual(failures, [], "\n".join(failures))
        self._success = True
