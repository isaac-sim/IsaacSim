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

"""Tests for the examples browser model."""

import omni.kit.app
import omni.kit.test
from isaacsim.examples.browser.model import (
    ExampleBrowserModel,
    ExampleCategoryItem,
    ExampleDetailItem,
    ExampleFolderDetailItem,
)


class TestExampleBrowserModel(omni.kit.test.AsyncTestCase):
    """Test examples browser category/detail lookup behavior."""

    async def test_parent_categories_show_folder_tiles_for_subcategories(self) -> None:
        """A synthetic parent (no direct examples) shows one folder tile per immediate child."""
        model = ExampleBrowserModel()
        self.addCleanup(model.destroy)

        model.register_example(name="Nova Carter", category="ROS2/Navigation")
        model.register_example(name="Hospital Scene", category="ROS2/Navigation/Multiple Robots")

        ros2_category = model.get_category_items(None)[0]

        details = model.get_detail_items(ros2_category)
        self.assertEqual(len(details), 1)
        self.assertIsInstance(details[0], ExampleFolderDetailItem)
        self.assertEqual(details[0].name, "Navigation")
        self.assertEqual(details[0].category_path, "ROS2/Navigation")

    async def test_mixed_category_shows_direct_examples_and_subfolder_tiles(self) -> None:
        """A category with direct examples AND child categories shows both, like a file browser."""
        model = ExampleBrowserModel()
        self.addCleanup(model.destroy)

        model.register_example(name="Nova Carter", category="ROS2/Navigation")
        model.register_example(name="Hospital Scene", category="ROS2/Navigation/Multiple Robots")

        ros2_category = model.get_category_items(None)[0]
        navigation_category = ros2_category.children[0]

        details = model.get_detail_items(navigation_category)
        # 1 direct example + 1 folder tile, sorted by name -> "Multiple Robots", "Nova Carter"
        self.assertEqual(len(details), 2)
        names = [d.name for d in details]
        self.assertIn("Nova Carter", names)
        self.assertIn("Multiple Robots", names)

        folder_tiles = [d for d in details if isinstance(d, ExampleFolderDetailItem)]
        example_tiles = [d for d in details if isinstance(d, ExampleDetailItem)]
        self.assertEqual(len(folder_tiles), 1)
        self.assertEqual(folder_tiles[0].category_path, "ROS2/Navigation/Multiple Robots")
        self.assertEqual(len(example_tiles), 1)
        self.assertEqual(example_tiles[0].example.name, "Nova Carter")

    async def test_leaf_category_shows_only_its_direct_examples(self) -> None:
        """A leaf category shows just its examples; no recursion, no folder tiles."""
        model = ExampleBrowserModel()
        self.addCleanup(model.destroy)

        model.register_example(name="Nova Carter", category="ROS2/Navigation")
        model.register_example(name="Hospital Scene", category="ROS2/Navigation/Multiple Robots")

        ros2_category = model.get_category_items(None)[0]
        navigation_category = ros2_category.children[0]
        multiple_robots_category = navigation_category.children[0]

        details = model.get_detail_items(multiple_robots_category)
        self.assertEqual(len(details), 1)
        self.assertIsInstance(details[0], ExampleDetailItem)
        self.assertEqual(details[0].example.name, "Hospital Scene")

    async def test_folder_tile_execute_navigates_tree(self) -> None:
        """Double-clicking a folder tile must drive ``widget.category_selection`` to the live target."""
        model = ExampleBrowserModel()
        self.addCleanup(model.destroy)

        model.register_example(name="Nova Carter", category="ROS2/Navigation")
        model.register_example(name="Hospital Scene", category="ROS2/Navigation/Multiple Robots")

        # Force category-tree construction so `_find_category_by_path` has a fresh tree to walk.
        roots = model.get_category_items(None)

        class _FakeWidget:
            def __init__(self):
                self.category_selection = []

        fake_widget = _FakeWidget()
        model.set_widget(fake_widget)

        navigation_target = roots[0].children[0]
        tile = ExampleFolderDetailItem("ROS2/Navigation", "Navigation")
        model.execute(tile)

        # `execute()` defers the selection change to the next update to avoid mutating the widget
        # tree during the double-click event chain. Pump two frames: the first satisfies the coroutine's
        # internal `next_update_async()` await, the second ensures its continuation has executed.
        await omni.kit.app.get_app().next_update_async()
        await omni.kit.app.get_app().next_update_async()

        self.assertEqual(fake_widget.category_selection, [navigation_target])
        self.assertIsInstance(fake_widget.category_selection[0], ExampleCategoryItem)

    async def test_folder_tile_with_unknown_path_is_a_noop(self) -> None:
        """An unresolved folder tile must not crash and must not change selection."""
        model = ExampleBrowserModel()
        self.addCleanup(model.destroy)

        model.register_example(name="Nova Carter", category="ROS2/Navigation")
        model.get_category_items(None)

        class _FakeWidget:
            def __init__(self):
                self.category_selection = ["sentinel"]

        fake_widget = _FakeWidget()
        model.set_widget(fake_widget)

        model.execute(ExampleFolderDetailItem("Does/Not/Exist", "Exist"))
        # Selection must remain untouched when the target can't be resolved.
        self.assertEqual(fake_widget.category_selection, ["sentinel"])
