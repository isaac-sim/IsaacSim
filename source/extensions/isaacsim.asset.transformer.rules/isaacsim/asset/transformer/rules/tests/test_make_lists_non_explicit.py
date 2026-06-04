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

"""Tests for MakeListsNonExplicitRule."""

import os
import shutil
import tempfile

import omni.kit.test
from isaacsim.asset.transformer.rules.isaac_sim.make_lists_non_explicit import MakeListsNonExplicitRule
from isaacsim.asset.transformer.rules.utils import compile_patterns, matches_any_pattern
from pxr import Sdf, Usd

from .common import _TEST_ADVANCED_USD


def _matches_any(name: str, patterns: list[str]) -> bool:
    """Check if a name fully matches any regex pattern.

    Args:
        name: Name to check.
        patterns: List of regex pattern strings.

    Returns:
        True if name matches any pattern, False otherwise.

    """
    return matches_any_pattern(name, compile_patterns(patterns))


class TestMakeListsNonExplicitRule(omni.kit.test.AsyncTestCase):
    """Async tests for MakeListsNonExplicitRule."""

    async def setUp(self) -> None:
        """Create a temporary directory for test output."""
        self._tmpdir = tempfile.mkdtemp()
        self._success = False

    async def tearDown(self) -> None:
        """Remove temporary directory after successful tests."""
        if self._success:
            shutil.rmtree(self._tmpdir, ignore_errors=True)

    def _copy_test_asset(self) -> str:
        """Copy the test asset to a writable temporary path.

        Returns:
            Absolute path to the copied test asset.

        """
        temp_asset = os.path.join(self._tmpdir, "test_advanced.usda")
        shutil.copy(_TEST_ADVANCED_USD, temp_asset)
        return temp_asset

    def _find_explicit_metadata_list_op(
        self,
        stage: Usd.Stage,
        metadata_patterns: list[str],
    ) -> tuple[str, str, list[object]] | None:
        """Find a prim with explicit metadata list op matching patterns.

        Args:
            stage: Stage to inspect.
            metadata_patterns: Metadata name patterns to match.

        Returns:
            Tuple of (prim_path, metadata_key, explicit_items) or None if not found.

        """
        layer = stage.GetRootLayer()
        for prim in Usd.PrimRange(stage.GetPseudoRoot()):
            prim_spec = layer.GetPrimAtPath(prim.GetPath())
            if not prim_spec:
                continue
            for key in prim_spec.ListInfoKeys():
                if not _matches_any(key, metadata_patterns):
                    continue
                list_op = prim_spec.GetInfo(key)
                if not hasattr(list_op, "isExplicit") or not list_op.isExplicit:
                    continue
                explicit_items = list(list_op.explicitItems or [])
                if explicit_items:
                    return prim_spec.path.pathString, key, explicit_items
        return None

    def _find_explicit_relationship_list_op(
        self,
        stage: Usd.Stage,
        property_patterns: list[str],
    ) -> tuple[str, str, list[Sdf.Path]] | None:
        """Find a prim relationship with explicit target list op matching patterns.

        Args:
            stage: Stage to inspect.
            property_patterns: Property name patterns to match.

        Returns:
            Tuple of (prim_path, property_name, explicit_items) or None if not found.

        """
        layer = stage.GetRootLayer()
        for prim in Usd.PrimRange(stage.GetPseudoRoot()):
            prim_spec = layer.GetPrimAtPath(prim.GetPath())
            if not prim_spec:
                continue
            for prop_name, prop_spec in prim_spec.properties.items():
                if not _matches_any(prop_name, property_patterns):
                    continue
                if not isinstance(prop_spec, Sdf.RelationshipSpec):
                    continue
                list_op = prop_spec.targetPathList
                if not hasattr(list_op, "isExplicit") or not list_op.isExplicit:
                    continue
                explicit_items = list(list_op.explicitItems or [])
                if explicit_items:
                    return prim_spec.path.pathString, prop_name, explicit_items
        return None

    async def test_get_configuration_parameters(self) -> None:
        """Verify configuration parameters are exposed."""
        stage = Usd.Stage.Open(_TEST_ADVANCED_USD)
        rule = MakeListsNonExplicitRule(
            source_stage=stage,
            package_root=self._tmpdir,
            destination_path="",
            args={},
        )

        params = rule.get_configuration_parameters()
        param_names = [p.name for p in params]
        self.assertIn("metadata_names", param_names)
        self.assertIn("property_names", param_names)
        self.assertIn("list_op_type", param_names)
        self._success = True

    async def test_process_rule_converts_explicit_lists_prepend(self) -> None:
        """Verify explicit list ops are converted to prepended list ops."""
        temp_asset = self._copy_test_asset()
        stage = Usd.Stage.Open(temp_asset)

        metadata_patterns = ["apiSchemas"]
        property_patterns = ["material:binding"]

        metadata_match = self._find_explicit_metadata_list_op(stage, metadata_patterns)
        rel_match = self._find_explicit_relationship_list_op(stage, property_patterns)
        self.assertIsNotNone(metadata_match)
        self.assertIsNotNone(rel_match)

        rule = MakeListsNonExplicitRule(
            source_stage=stage,
            package_root=self._tmpdir,
            destination_path="",
            args={
                "params": {
                    "metadata_names": metadata_patterns,
                    "property_names": property_patterns,
                    "list_op_type": "prepend",
                }
            },
        )

        rule.process_rule()

        layer = Sdf.Layer.FindOrOpen(temp_asset)
        self.assertIsNotNone(layer)

        prim_path, metadata_key, metadata_items = metadata_match
        prim_spec = layer.GetPrimAtPath(prim_path)
        self.assertIsNotNone(prim_spec)
        updated_metadata = prim_spec.GetInfo(metadata_key)
        self.assertFalse(updated_metadata.isExplicit)
        self.assertEqual(list(updated_metadata.prependedItems), metadata_items)

        rel_prim_path, rel_name, rel_items = rel_match
        rel_prim_spec = layer.GetPrimAtPath(rel_prim_path)
        self.assertIsNotNone(rel_prim_spec)
        rel_spec = rel_prim_spec.relationships.get(rel_name)
        self.assertIsNotNone(rel_spec)
        rel_list_op = rel_spec.targetPathList
        self.assertFalse(rel_list_op.isExplicit)
        self.assertEqual(list(rel_list_op.prependedItems), rel_items)

        self._success = True

    async def test_process_rule_converts_explicit_lists_append(self) -> None:
        """Verify explicit list ops are converted to appended list ops."""
        temp_asset = self._copy_test_asset()
        stage = Usd.Stage.Open(temp_asset)

        metadata_patterns = ["apiSchemas"]

        metadata_match = self._find_explicit_metadata_list_op(stage, metadata_patterns)
        self.assertIsNotNone(metadata_match)

        rule = MakeListsNonExplicitRule(
            source_stage=stage,
            package_root=self._tmpdir,
            destination_path="",
            args={
                "params": {
                    "metadata_names": metadata_patterns,
                    "property_names": [],
                    "list_op_type": "append",
                }
            },
        )

        rule.process_rule()

        layer = Sdf.Layer.FindOrOpen(temp_asset)
        self.assertIsNotNone(layer)

        prim_path, metadata_key, metadata_items = metadata_match
        prim_spec = layer.GetPrimAtPath(prim_path)
        self.assertIsNotNone(prim_spec)
        updated_metadata = prim_spec.GetInfo(metadata_key)
        self.assertFalse(updated_metadata.isExplicit)
        self.assertEqual(list(updated_metadata.appendedItems), metadata_items)

        self._success = True

    async def test_process_rule_converts_explicit_schemas_prepend(self) -> None:
        """Verify explicit apiSchemas are converted to prepended list ops."""
        temp_asset = self._copy_test_asset()
        stage = Usd.Stage.Open(temp_asset)

        metadata_patterns = ["apiSchemas"]
        metadata_match = self._find_explicit_metadata_list_op(stage, metadata_patterns)
        self.assertIsNotNone(metadata_match)

        rule = MakeListsNonExplicitRule(
            source_stage=stage,
            package_root=self._tmpdir,
            destination_path="",
            args={
                "params": {
                    "metadata_names": metadata_patterns,
                    "property_names": [],
                    "list_op_type": "prepend",
                }
            },
        )

        rule.process_rule()

        layer = Sdf.Layer.FindOrOpen(temp_asset)
        self.assertIsNotNone(layer)

        prim_path, metadata_key, metadata_items = metadata_match
        prim_spec = layer.GetPrimAtPath(prim_path)
        self.assertIsNotNone(prim_spec)
        updated_metadata = prim_spec.GetInfo(metadata_key)
        self.assertFalse(updated_metadata.isExplicit)
        self.assertEqual(list(updated_metadata.prependedItems), metadata_items)

        self._success = True


def _build_sublayered_stage(
    tmpdir: str,
    *,
    rel_name: str = "isaac:physics:robotJoints",
    explicit_targets: list[str] | None = None,
) -> tuple[Usd.Stage, str, str]:
    """Build a root USDA that sublayers a payload with an explicit-list-op relationship.

    Args:
        tmpdir: Temporary directory to author the layers in.
        rel_name: Relationship name to author on ``/Robot`` in the sublayer.
        explicit_targets: Target paths for the explicit-list-op relationship.

    Returns:
        Tuple of (opened stage, root layer path, sublayer path).

    """
    if explicit_targets is None:
        explicit_targets = ["/Robot/joints/j1", "/Robot/joints/j2"]

    sublayer_path = os.path.join(tmpdir, "payload.usda")
    root_path = os.path.join(tmpdir, "root.usda")

    # Author the sublayer: /Robot prim with an explicit-list-op relationship.
    sublayer = Sdf.Layer.CreateNew(sublayer_path)
    prim_spec = Sdf.CreatePrimInLayer(sublayer, "/Robot")
    prim_spec.specifier = Sdf.SpecifierDef
    prim_spec.typeName = "Xform"
    rel_spec = Sdf.RelationshipSpec(prim_spec, rel_name, custom=True)
    rel_spec.targetPathList.ClearEditsAndMakeExplicit()
    rel_spec.targetPathList.explicitItems = [Sdf.Path(p) for p in explicit_targets]
    sublayer.Save()

    # Root layer sublayers the payload.
    root = Sdf.Layer.CreateNew(root_path)
    root.subLayerPaths.append(os.path.basename(sublayer_path))
    root.Save()

    stage = Usd.Stage.Open(root_path)
    return stage, root_path, sublayer_path


class TestSublayerRelationshipWalk(omni.kit.test.AsyncTestCase):
    """Regression tests for the per-prim ``PrimStack`` walk.

    The property-conversion branch of ``MakeListsNonExplicitRule.process_rule``
    must enumerate every contributing ``PrimSpec`` via ``prim.GetPrimStack()``
    so that explicit-list-op relationships authored in sublayers (e.g. in a
    payload) are converted to ``prepend``. Root-layer-authored relationships
    (existing behavior) remain converted.
    """

    async def setUp(self) -> None:
        """Create a temporary directory for test output."""
        self._tmpdir = tempfile.mkdtemp()
        self._success = False

    async def tearDown(self) -> None:
        """Remove temporary directory after successful tests."""
        if self._success:
            shutil.rmtree(self._tmpdir, ignore_errors=True)

    def _invoke_rule(
        self,
        stage: Usd.Stage,
        *,
        property_names: list[str],
        list_op_type: str = "prepend",
        metadata_names: list[str] | None = None,
    ) -> MakeListsNonExplicitRule:
        """Invoke `MakeListsNonExplicitRule` on a stage with the given params.

        Args:
            stage: Working stage.
            property_names: Property-name regex patterns to convert.
            list_op_type: Target list-op type (``prepend`` or ``append``).
            metadata_names: Metadata-name regex patterns (root-layer-only path).

        Returns:
            The invoked rule instance.

        """
        rule = MakeListsNonExplicitRule(
            source_stage=stage,
            package_root=self._tmpdir,
            destination_path="",
            args={
                "params": {
                    "metadata_names": metadata_names or [],
                    "property_names": property_names,
                    "list_op_type": list_op_type,
                }
            },
        )
        rule.process_rule()
        return rule

    async def test_sublayer_relationship_converted_to_prepend(self) -> None:
        """Per-prim ``PrimStack`` walk reaches the sublayer's relationship spec."""
        stage, _root_path, sublayer_path = _build_sublayered_stage(self._tmpdir)
        self._invoke_rule(
            stage,
            property_names=["isaac:physics:robotJoints"],
            list_op_type="prepend",
        )

        sublayer = Sdf.Layer.FindOrOpen(sublayer_path)
        self.assertIsNotNone(sublayer, "Expected sublayer to reopen successfully.")
        prim_spec = sublayer.GetPrimAtPath("/Robot")
        self.assertIsNotNone(prim_spec, "Expected `/Robot` spec in sublayer.")
        rel_spec = prim_spec.relationships.get("isaac:physics:robotJoints")
        self.assertIsNotNone(rel_spec)
        # After conversion: targetPathList is no longer explicit; items moved to prepend.
        self.assertFalse(
            rel_spec.targetPathList.isExplicit,
            "Expected list-op to be non-explicit after conversion to prepend.",
        )
        self.assertEqual(
            sorted(str(p) for p in rel_spec.targetPathList.prependedItems),
            ["/Robot/joints/j1", "/Robot/joints/j2"],
        )
        self._success = True

    async def test_root_layer_relationship_still_converted(self) -> None:
        """Regression: root-layer-authored rels are still converted (existing behavior)."""
        root_path = os.path.join(self._tmpdir, "root_only.usda")
        root_layer = Sdf.Layer.CreateNew(root_path)
        prim_spec = Sdf.CreatePrimInLayer(root_layer, "/Robot")
        prim_spec.specifier = Sdf.SpecifierDef
        prim_spec.typeName = "Xform"
        rel_spec = Sdf.RelationshipSpec(prim_spec, "isaac:physics:robotJoints", custom=True)
        rel_spec.targetPathList.ClearEditsAndMakeExplicit()
        rel_spec.targetPathList.explicitItems = [Sdf.Path("/Robot/j1"), Sdf.Path("/Robot/j2")]
        root_layer.Save()

        # Sanity: confirm the authored list-op is explicit before invoking the rule.
        self.assertTrue(
            rel_spec.targetPathList.isExplicit,
            "Fixture precondition: expected explicit list-op on authored relationship.",
        )

        stage = Usd.Stage.Open(root_path)
        self._invoke_rule(
            stage,
            property_names=["isaac:physics:robotJoints"],
            list_op_type="prepend",
        )

        # Re-check root spec.
        reopened_layer = Sdf.Layer.FindOrOpen(root_path)
        self.assertIsNotNone(reopened_layer)
        rel_prim_spec = reopened_layer.GetPrimAtPath("/Robot")
        self.assertIsNotNone(rel_prim_spec)
        updated_rel = rel_prim_spec.relationships.get("isaac:physics:robotJoints")
        self.assertIsNotNone(updated_rel)
        self.assertFalse(
            updated_rel.targetPathList.isExplicit,
            "Expected list-op to be non-explicit after conversion to prepend.",
        )
        self.assertEqual(
            sorted(str(p) for p in updated_rel.targetPathList.prependedItems),
            ["/Robot/j1", "/Robot/j2"],
        )
        self._success = True

    async def test_prepend_relationship_is_noop(self) -> None:
        """Idempotency: rel already `prepend` -> no change, no crash."""
        sublayer_path = os.path.join(self._tmpdir, "payload_prepend.usda")
        root_path = os.path.join(self._tmpdir, "root_prepend.usda")

        sublayer = Sdf.Layer.CreateNew(sublayer_path)
        prim_spec = Sdf.CreatePrimInLayer(sublayer, "/Robot")
        prim_spec.specifier = Sdf.SpecifierDef
        prim_spec.typeName = "Xform"
        rel_spec = Sdf.RelationshipSpec(prim_spec, "isaac:physics:robotJoints", custom=True)
        rel_spec.targetPathList.prependedItems = [Sdf.Path("/Robot/j1")]
        sublayer.Save()

        root = Sdf.Layer.CreateNew(root_path)
        root.subLayerPaths.append(os.path.basename(sublayer_path))
        root.Save()

        stage = Usd.Stage.Open(root_path)
        self._invoke_rule(
            stage,
            property_names=["isaac:physics:robotJoints"],
            list_op_type="prepend",
        )

        # No change expected — already non-explicit, so the rule should leave it alone.
        reopened = Sdf.Layer.FindOrOpen(sublayer_path)
        reopened_rel = reopened.GetPrimAtPath("/Robot").relationships.get("isaac:physics:robotJoints")
        self.assertIsNotNone(reopened_rel)
        self.assertFalse(
            reopened_rel.targetPathList.isExplicit,
            "Expected already-prepend list-op to remain non-explicit.",
        )
        self.assertEqual(
            sorted(str(p) for p in reopened_rel.targetPathList.prependedItems),
            ["/Robot/j1"],
        )
        self._success = True

    async def test_unrelated_property_untouched(self) -> None:
        """Property-name filter: `material:binding` should NOT be converted by the new entry."""
        sublayer_path = os.path.join(self._tmpdir, "payload_other.usda")
        root_path = os.path.join(self._tmpdir, "root_other.usda")

        sublayer = Sdf.Layer.CreateNew(sublayer_path)
        prim_spec = Sdf.CreatePrimInLayer(sublayer, "/Robot")
        prim_spec.specifier = Sdf.SpecifierDef
        prim_spec.typeName = "Xform"
        # Unrelated relationship authored as explicit list-op.
        other_spec = Sdf.RelationshipSpec(prim_spec, "material:binding", custom=True)
        other_spec.targetPathList.explicitItems = [Sdf.Path("/Materials/red")]
        sublayer.Save()

        root = Sdf.Layer.CreateNew(root_path)
        root.subLayerPaths.append(os.path.basename(sublayer_path))
        root.Save()

        stage = Usd.Stage.Open(root_path)
        self._invoke_rule(
            stage,
            property_names=["isaac:physics:robotJoints"],  # filter does NOT match material:binding
            list_op_type="prepend",
        )

        reopened = Sdf.Layer.FindOrOpen(sublayer_path)
        reopened_other = reopened.GetPrimAtPath("/Robot").relationships.get("material:binding")
        self.assertIsNotNone(reopened_other)
        # Still explicit — filter did not match, no conversion occurred.
        self.assertTrue(reopened_other.targetPathList.isExplicit)
        self.assertEqual(
            [str(p) for p in reopened_other.targetPathList.explicitItems],
            ["/Materials/red"],
        )
        self._success = True
