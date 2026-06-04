# SPDX-FileCopyrightText: Copyright (c) 2021-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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

# NOTE:
#   omni.kit.test - std python's unittest module with additional wrapping to add support for async/await tests
#   For most things refer to unittest docs: https://docs.python.org/3/library/unittest.html

"""Tests for semantic labeling utility functions."""

from typing import Any

import omni.kit.test
import Semantics
from isaacsim.core.utils.prims import get_prim_at_path

# Import extension python module we are testing with absolute import path, as if we are external user (other extension)
from isaacsim.core.utils.semantics import (
    add_labels,
    check_incorrect_labels,
    check_missing_labels,
    count_labels_in_scene,
    get_labels,
    remove_labels,
    upgrade_prim_semantics_to_labels,
)


# Having a test class derived from omni.kit.test.AsyncTestCase declared on the root of module will make it auto-discoverable by omni.kit.test
class TestSemantics(omni.kit.test.AsyncTestCase):
    """Test cases for Semantics."""

    async def setUp(self) -> None:
        """Set up test fixtures."""
        await omni.usd.get_context().new_stage_async()
        await omni.kit.app.get_app().next_update_async()

    async def tearDown(self) -> None:
        """Tear down test fixtures."""
        await omni.kit.app.get_app().next_update_async()

    def create_test_environment_new_labels(self) -> list:
        """Create a test environment with semantic labels.

        Returns:
            List of created prim paths.
        """
        # creates a test environment with 4 cubes meshes using the new LabelsAPI
        # assign labels "cube" for 2, "sphere" for 1, and leave the last one missing
        # Also add a nested prim with labels
        result, path_0 = omni.kit.commands.execute("CreateMeshPrimCommand", prim_type="Cube", prim_path="/World/Cube_0")
        result, path_1 = omni.kit.commands.execute("CreateMeshPrimCommand", prim_type="Cube", prim_path="/World/Cube_1")
        result, path_2 = omni.kit.commands.execute("CreateMeshPrimCommand", prim_type="Cube", prim_path="/World/Cube_2")
        result, path_3 = omni.kit.commands.execute("CreateMeshPrimCommand", prim_type="Cube", prim_path="/World/Cube_3")
        result, path_4 = omni.kit.commands.execute(
            "CreateMeshPrimCommand", prim_type="Cube", prim_path="/World/Cube_0/Nested_Cube"
        )

        add_labels(prim=get_prim_at_path(path_0), labels=["cube"], instance_name="class")
        add_labels(prim=get_prim_at_path(path_1), labels=["cube"], instance_name="class")
        add_labels(prim=get_prim_at_path(path_2), labels=["sphere"], instance_name="class")
        add_labels(prim=get_prim_at_path(path_4), labels=["nested"], instance_name="shape")  # Nested prim

        return [path_0, path_1, path_2, path_3, path_4]

    def _apply_old_semantics(self, prim: Any, semantic_label: Any, type_label: Any = "class", suffix: str = "") -> None:
        """Apply old SemanticsAPI directly for testing upgrade functionality.

        Args:
            prim: The prim to apply semantics to.
            semantic_label: The semantic label value.
            type_label: The semantic type label.
            suffix: Optional suffix for the semantics API instance.
        """
        semantic_api = Semantics.SemanticsAPI.Get(prim, "Semantics" + suffix)
        if not semantic_api:
            semantic_api = Semantics.SemanticsAPI.Apply(prim, "Semantics" + suffix)
            semantic_api.CreateSemanticTypeAttr()
            semantic_api.CreateSemanticDataAttr()

        type_attr = semantic_api.GetSemanticTypeAttr()
        data_attr = semantic_api.GetSemanticDataAttr()

        if type_label is not None:
            type_attr.Set(type_label)
        if semantic_label is not None:
            data_attr.Set(semantic_label)

    async def test_upgrade_prim_semantics_to_labels(self) -> None:
        """Test upgrade prim semantics to labels."""
        stage = omni.usd.get_context().get_stage()
        prim = stage.DefinePrim("/test_upgrade", "Xform")

        # 1) Create old semantics using the helper
        self._apply_old_semantics(prim, semantic_label="old_data", type_label="old_type")
        self.assertTrue(
            bool(Semantics.SemanticsAPI.Get(prim, "Semantics")), "Old SemanticsAPI should be valid after creation"
        )

        # 2) Upgrade semantics
        # The upgrade function now always removes the old semantics.
        upgraded_count = upgrade_prim_semantics_to_labels(prim)
        self.assertGreater(upgraded_count, 0, "At least one instance should be upgraded")

        # 3) Verify old semantics are removed
        # Old semantics should be removed (API object should be invalid)
        self.assertFalse(
            bool(Semantics.SemanticsAPI.Get(prim, "Semantics")),
            msg="Old SemanticsAPI should be invalid after upgrade",
        )

        # 4) Verify new labels exist (Optional: and are correct)
        # This part can be expanded to check the actual labels if needed.
        # For now, we confirm the old API is gone and new one might exist via get_labels.
        new_labels = get_labels(prim)
        self.assertTrue(
            "old_type" in new_labels and new_labels["old_type"] == ["old_data"],
            "New labels should be present and correct after upgrade.",
        )

    async def test_new_labels_api(self) -> None:
        """Test new labels api."""
        stage = omni.usd.get_context().get_stage()
        # Create a test prim and a nested prim
        prim = stage.DefinePrim("/new_labels_test", "Xform")
        nested_prim = stage.DefinePrim("/new_labels_test/nested", "Xform")

        # 1) Add labels using the new API
        add_labels(prim=prim, labels=["label_a", "label_b"], instance_name="class")
        add_labels(prim=prim, labels=["shape_a"], instance_name="shape")
        add_labels(prim=nested_prim, labels=["nested_label"], instance_name="class")

        labels_dict = get_labels(prim)
        self.assertIn("class", labels_dict)
        self.assertEqual(sorted(labels_dict["class"]), sorted(["label_a", "label_b"]))  # Sort for consistent comparison
        self.assertIn("shape", labels_dict)
        self.assertEqual(labels_dict["shape"], ["shape_a"])
        nested_labels_dict = get_labels(nested_prim)
        self.assertIn("class", nested_labels_dict)
        self.assertEqual(nested_labels_dict["class"], ["nested_label"])

        # 2) Re-invoke add_labels with overwrite=False (should append)
        add_labels(prim=prim, labels=["label_c"], instance_name="class", overwrite=False)
        labels_dict = get_labels(prim)
        self.assertEqual(sorted(labels_dict["class"]), sorted(["label_a", "label_b", "label_c"]))

        # 3) Overwrite existing labels for a specific instance
        add_labels(prim=prim, labels=["replaced_label"], instance_name="class", overwrite=True)
        labels_dict = get_labels(prim)
        self.assertEqual(labels_dict["class"], ["replaced_label"])
        self.assertIn("shape", labels_dict)  # Other instance should remain untouched
        self.assertEqual(labels_dict["shape"], ["shape_a"])

        # 4) Test remove_labels: Remove specific instance
        remove_labels(prim, instance_name="shape")
        labels_dict = get_labels(prim)
        self.assertIn("class", labels_dict)
        self.assertNotIn("shape", labels_dict)

        # 5) Test remove_labels: Remove all instances on prim
        add_labels(prim=prim, labels=["shape_b"], instance_name="shape")  # Re-add shape instance
        remove_labels(prim)  # Remove all
        labels_dict = get_labels(prim)
        self.assertEqual(len(labels_dict), 0)  # Should be empty

        # 6) Test remove_labels: Recursive removal
        add_labels(prim=prim, labels=["label_root"], instance_name="class")
        add_labels(prim=nested_prim, labels=["label_nested"], instance_name="class")
        remove_labels(prim, include_descendants=True)
        labels_dict = get_labels(prim)
        nested_labels_dict = get_labels(nested_prim)
        self.assertEqual(len(labels_dict), 0)
        self.assertEqual(len(nested_labels_dict), 0)  # Nested should also be removed

        # 7) Test remove_labels: Recursive removal of specific instance
        add_labels(prim=prim, labels=["label_root"], instance_name="class")
        add_labels(prim=prim, labels=["shape_root"], instance_name="shape")
        add_labels(prim=nested_prim, labels=["label_nested"], instance_name="class")
        add_labels(prim=nested_prim, labels=["shape_nested"], instance_name="shape")
        remove_labels(prim, instance_name="class", include_descendants=True)  # Remove only 'class' recursively
        labels_dict = get_labels(prim)
        nested_labels_dict = get_labels(nested_prim)
        self.assertNotIn("class", labels_dict)
        self.assertIn("shape", labels_dict)  # shape should remain
        self.assertNotIn("class", nested_labels_dict)
        self.assertIn("shape", nested_labels_dict)  # shape should remain

    async def test_check_missing_labels(self) -> None:
        """Test the check_missing_labels function using the new LabelsAPI."""
        cube_paths = self.create_test_environment_new_labels()
        # Check from root
        missing_paths = check_missing_labels()

        # Cubes 0, 1, 2, and nested cube 4 have labels. Cube 3 does not.
        # The check includes all Mesh types, including the nested one.
        # Expected missing: Cube_3
        self.assertEqual(len(missing_paths), 1)
        self.assertIn(cube_paths[3], missing_paths)  # Cube_3 should be missing

        # Check from specific path /World/Cube_0
        missing_paths_subtree = check_missing_labels(prim_path="/World/Cube_0")
        # Cube_0 has labels, Nested_Cube has labels. None missing in this subtree.
        self.assertEqual(len(missing_paths_subtree), 0)

    async def test_check_incorrect_labels(self) -> None:
        """Test the check_incorrect_labels function using the new LabelsAPI."""
        cube_paths = self.create_test_environment_new_labels()
        # Check from root
        mismatch_prims = check_incorrect_labels()

        # Valid semantic labels do not need to appear in the prim path.
        self.assertEqual(len(mismatch_prims), 0)

        # The legacy path-substring heuristic remains available when explicitly requested.
        mismatch_prims = check_incorrect_labels(validate_against_prim_path=True)

        # Cube_0 path: /World/Cube_0, label: cube -> OK
        # Cube_1 path: /World/Cube_1, label: cube -> OK
        # Cube_2 path: /World/Cube_2, label: sphere -> MISMATCH
        # Cube_3 path: /World/Cube_3, label: (none) -> OK (no label to mismatch)
        # Nested_Cube path: /World/Cube_0/Nested_Cube, label: nested -> OK
        # Expected mismatch: [Cube_2 path, "sphere"]
        self.assertEqual(len(mismatch_prims), 1)
        self.assertEqual(mismatch_prims[0][0], cube_paths[2])  # Path of Cube_2
        self.assertEqual(mismatch_prims[0][1], "sphere")  # Incorrect label

        # Check from specific path /World/Cube_0
        mismatch_prims_subtree = check_incorrect_labels(prim_path="/World/Cube_0")
        # Cube_0: OK, Nested_Cube: OK. None mismatching in this subtree.
        self.assertEqual(len(mismatch_prims_subtree), 0)

    async def test_count_labels_in_scene_new(self) -> None:
        """Test the count_labels_in_scene function using the new LabelsAPI."""
        self.create_test_environment_new_labels()
        # Count from root
        labels_dict = count_labels_in_scene()

        # Expected counts:
        # missing_labels: 1 (Cube_3)
        # cube: 2 (Cube_0, Cube_1)
        # sphere: 1 (Cube_2)
        # nested: 1 (Nested_Cube)
        # Total prims checked: 5 (Cube_0, Cube_1, Cube_2, Cube_3, Nested_Cube)
        print("Labels Dict (Root):", labels_dict)  # Debug print
        self.assertEqual(labels_dict.get("missing_labels", 0), 1)
        self.assertEqual(labels_dict.get("cube", 0), 2)
        self.assertEqual(labels_dict.get("sphere", 0), 1)
        self.assertEqual(labels_dict.get("nested", 0), 1)
        # Check total keys excluding 'missing_labels' match the unique labels + 'missing_labels'
        self.assertEqual(len(labels_dict), 4)  # cube, sphere, nested, missing_labels

        # Count from specific path /World/Cube_0
        labels_dict_subtree = count_labels_in_scene(prim_path="/World/Cube_0")
        # Prims checked: Cube_0, Nested_Cube
        # Expected counts:
        # missing_labels: 0
        # cube: 1 (Cube_0)
        # nested: 1 (Nested_Cube)
        print("Labels Dict (Subtree):", labels_dict_subtree)  # Debug print
        self.assertEqual(labels_dict_subtree.get("missing_labels", 0), 0)
        self.assertEqual(labels_dict_subtree.get("cube", 0), 1)
        self.assertEqual(labels_dict_subtree.get("nested", 0), 1)
        # Expect 3 keys: cube, nested, and missing_labels (even if 0)
        self.assertEqual(len(labels_dict_subtree), 3)
