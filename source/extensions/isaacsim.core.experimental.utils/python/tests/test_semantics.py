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

"""Verifies semantic label utilities for adding, reading, removing, and migrating labels. Covers conversion from legacy semantics on single prims, paths, multiple instances, descendants, and empty legacy data."""

import isaacsim.core.experimental.utils.semantics as semantics_utils
import isaacsim.core.experimental.utils.stage as stage_utils
import omni.kit.test


class TestSemantics(omni.kit.test.AsyncTestCase):
    """Test semantics."""

    async def setUp(self) -> None:
        """Method called to prepare the test fixture."""
        super().setUp()
        # create new stage
        await stage_utils.create_new_stage_async()

    async def tearDown(self) -> None:
        """Method called immediately after the test method has been called."""
        super().tearDown()

    # --------------------------------------------------------------------

    async def test_add_labels(self) -> None:
        """Test add labels."""
        prim = stage_utils.define_prim("/World/A", "Cube")
        self.assertDictEqual(semantics_utils.get_labels(prim), {})
        # test cases
        # - add labels
        semantics_utils.add_labels(prim, labels=["label_0", "label_1"])
        semantics_utils.add_labels(prim, labels=["label_1", "label_2", "label_3"], taxonomy="test")
        match = {"class": ["label_0", "label_1"], "test": ["label_1", "label_2", "label_3"]}
        self.assertDictEqual(semantics_utils.get_labels(prim), match)
        # - add a new label to existing ones
        semantics_utils.add_labels(prim, labels="label_4")
        semantics_utils.add_labels(prim, labels="label_4")  # add same label again
        match = {"class": ["label_0", "label_1", "label_4"], "test": ["label_1", "label_2", "label_3"]}
        self.assertDictEqual(semantics_utils.get_labels(prim), match)

    async def test_get_labels(self) -> None:
        """Test get labels."""
        stage_utils.define_prim("/World/A", "Cube")
        stage_utils.define_prim("/World/B", "Xform")
        self.assertDictEqual(semantics_utils.get_labels("/World/A"), {})
        self.assertDictEqual(semantics_utils.get_labels("/World/B"), {})
        # add labels
        semantics_utils.add_labels("/World/A", labels=["label_0", "label_1"])
        semantics_utils.add_labels("/World/B", labels=["label_1", "label_2", "label_3"], taxonomy="test")
        # test cases
        # - get labels from specific prims
        match = {"class": ["label_0", "label_1"]}
        self.assertDictEqual(semantics_utils.get_labels("/World/A"), match)
        match = {"test": ["label_1", "label_2", "label_3"]}
        self.assertDictEqual(semantics_utils.get_labels("/World/B"), match)
        # - get labels from prim without semantics applied
        self.assertDictEqual(semantics_utils.get_labels("/World"), {})
        # - get labels from prim without semantics applied (but with descendants)
        match = {"class": ["label_0", "label_1"], "test": ["label_1", "label_2", "label_3"]}
        self.assertDictEqual(semantics_utils.get_labels("/World", include_descendants=True), match)

    async def test_remove_labels(self) -> None:
        """Test remove labels."""
        prim = stage_utils.define_prim("/World/A", "Cube")
        self.assertDictEqual(semantics_utils.get_labels(prim), {})
        # add labels
        semantics_utils.add_labels(prim, labels=["label_0", "label_1", "label_4"])
        semantics_utils.add_labels(prim, labels=["label_1", "label_2", "label_3"], taxonomy="test")
        match = {"class": ["label_0", "label_1", "label_4"], "test": ["label_1", "label_2", "label_3"]}
        self.assertDictEqual(semantics_utils.get_labels(prim), match)
        # test cases
        # - remove label from specific instance
        semantics_utils.remove_labels(prim, labels="label_2", taxonomy="test")
        match = {"class": ["label_0", "label_1", "label_4"], "test": ["label_1", "label_3"]}
        self.assertDictEqual(semantics_utils.get_labels(prim), match)
        # - remove label from all instances
        semantics_utils.remove_labels(prim, labels="label_1")
        match = {"class": ["label_0", "label_4"], "test": ["label_3"]}
        self.assertDictEqual(semantics_utils.get_labels(prim), match)
        # - remove label from descendants
        semantics_utils.remove_labels("/World", labels="label_4", include_descendants=True)
        match = {"class": ["label_0"], "test": ["label_3"]}
        self.assertDictEqual(semantics_utils.get_labels(prim), match)
        # remove unexisting label
        semantics_utils.remove_labels(prim, labels="label_5")
        match = {"class": ["label_0"], "test": ["label_3"]}
        self.assertDictEqual(semantics_utils.get_labels(prim), match)

    async def test_remove_all_labels(self) -> None:
        """Test remove all labels."""
        prim = stage_utils.define_prim("/World/A", "Cube")
        self.assertDictEqual(semantics_utils.get_labels(prim), {})
        # add labels
        semantics_utils.add_labels(prim, labels=["label_0", "label_1", "label_4"])
        semantics_utils.add_labels(prim, labels=["label_1", "label_2", "label_3"], taxonomy="test")
        match = {"class": ["label_0", "label_1", "label_4"], "test": ["label_1", "label_2", "label_3"]}
        self.assertDictEqual(semantics_utils.get_labels(prim), match)
        # test cases
        # - remove all labels (keep taxonomies)
        semantics_utils.remove_all_labels(prim)
        match = {"class": [], "test": []}
        self.assertDictEqual(semantics_utils.get_labels(prim), match)
        # - remove all labels (remove taxonomies)
        # -- current prim
        semantics_utils.remove_all_labels("/World", remove_taxonomies=True)
        self.assertDictEqual(semantics_utils.get_labels(prim), match)
        # -- current prim and descendants
        semantics_utils.remove_all_labels("/World", remove_taxonomies=True, include_descendants=True)
        self.assertDictEqual(semantics_utils.get_labels(prim), {})

    async def test_upgrade_prim_semantics_to_labels_single_prim(self) -> None:
        """Test upgrade_prim_semantics_to_labels on a single prim."""
        import Semantics

        prim = stage_utils.define_prim("/World/A", "Cube")
        # apply old-style SemanticsAPI
        old_api = Semantics.SemanticsAPI.Apply(prim, "class")
        old_api.CreateSemanticTypeAttr().Set("class")
        old_api.CreateSemanticDataAttr().Set("box")

        self.assertEqual(semantics_utils.get_labels(prim), {})

        upgraded = semantics_utils.upgrade_prim_semantics_to_labels(prim)
        self.assertEqual(len(upgraded), 1)
        self.assertEqual(upgraded[0], str(prim.GetPath()))
        self.assertEqual(semantics_utils.get_labels(prim), {"class": ["box"]})
        self.assertFalse(prim.HasAPI(Semantics.SemanticsAPI, "class"))

    async def test_upgrade_prim_semantics_to_labels_by_path(self) -> None:
        """Test upgrade_prim_semantics_to_labels using a string path."""
        import Semantics

        prim = stage_utils.define_prim("/World/A", "Cube")
        old_api = Semantics.SemanticsAPI.Apply(prim, "class")
        old_api.CreateSemanticTypeAttr().Set("class")
        old_api.CreateSemanticDataAttr().Set("cube")

        upgraded = semantics_utils.upgrade_prim_semantics_to_labels("/World/A")
        self.assertEqual(len(upgraded), 1)
        self.assertEqual(upgraded[0], "/World/A")
        self.assertEqual(semantics_utils.get_labels("/World/A"), {"class": ["cube"]})
        self.assertFalse(prim.HasAPI(Semantics.SemanticsAPI, "class"))

    async def test_upgrade_prim_semantics_to_labels_multiple_instances(self) -> None:
        """Test upgrade with multiple SemanticsAPI instances on one prim."""
        import Semantics

        prim = stage_utils.define_prim("/World/A", "Cube")
        api_class = Semantics.SemanticsAPI.Apply(prim, "class")
        api_class.CreateSemanticTypeAttr().Set("class")
        api_class.CreateSemanticDataAttr().Set("box")
        api_color = Semantics.SemanticsAPI.Apply(prim, "color")
        api_color.CreateSemanticTypeAttr().Set("color")
        api_color.CreateSemanticDataAttr().Set("red")

        upgraded = semantics_utils.upgrade_prim_semantics_to_labels(prim)
        self.assertEqual(len(upgraded), 1)
        self.assertEqual(upgraded[0], str(prim.GetPath()))
        labels = semantics_utils.get_labels(prim)
        self.assertEqual(labels.get("class"), ["box"])
        self.assertEqual(labels.get("color"), ["red"])
        self.assertFalse(prim.HasAPI(Semantics.SemanticsAPI, "class"))
        self.assertFalse(prim.HasAPI(Semantics.SemanticsAPI, "color"))

    async def test_upgrade_prim_semantics_to_labels_no_old_semantics(self) -> None:
        """Test upgrade on a prim with no old-style semantics returns empty list."""
        stage_utils.define_prim("/World/A", "Cube")
        upgraded = semantics_utils.upgrade_prim_semantics_to_labels("/World/A")
        self.assertEqual(upgraded, [])

    async def test_upgrade_prim_semantics_to_labels_with_descendants(self) -> None:
        """Test upgrade with include_descendants=True."""
        import Semantics

        stage_utils.define_prim("/World/Parent", "Xform")
        child = stage_utils.define_prim("/World/Parent/Child", "Cube")
        old_api = Semantics.SemanticsAPI.Apply(child, "class")
        old_api.CreateSemanticTypeAttr().Set("class")
        old_api.CreateSemanticDataAttr().Set("child_obj")

        upgraded = semantics_utils.upgrade_prim_semantics_to_labels("/World/Parent", include_descendants=True)
        self.assertEqual(len(upgraded), 1)
        self.assertEqual(upgraded[0], str(child.GetPath()))
        self.assertEqual(semantics_utils.get_labels(child), {"class": ["child_obj"]})
        self.assertFalse(child.HasAPI(Semantics.SemanticsAPI, "class"))

    async def test_upgrade_prim_semantics_to_labels_empty_type_or_data(self) -> None:
        """Test upgrade skips instances with empty type or data."""
        import Semantics

        prim = stage_utils.define_prim("/World/A", "Cube")
        # apply old-style SemanticsAPI with empty data
        old_api = Semantics.SemanticsAPI.Apply(prim, "empty_data")
        old_api.CreateSemanticTypeAttr().Set("class")
        old_api.CreateSemanticDataAttr().Set("")

        # apply another with empty type
        old_api2 = Semantics.SemanticsAPI.Apply(prim, "empty_type")
        old_api2.CreateSemanticTypeAttr().Set("")
        old_api2.CreateSemanticDataAttr().Set("something")

        upgraded = semantics_utils.upgrade_prim_semantics_to_labels(prim)
        self.assertEqual(upgraded, [])
        self.assertDictEqual(semantics_utils.get_labels(prim), {})
