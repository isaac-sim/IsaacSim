# SPDX-FileCopyrightText: Copyright (c) 2018-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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

"""Tests for basic UI components and helper callbacks."""

# NOTE:
#   omni.kit.test - std python's unittest module with additional wrapping to add suport for async/await tests
#   For most things refer to unittest docs: https://docs.python.org/3/library/unittest.html
import omni.kit.test
from isaacsim.gui.components.callbacks import (
    on_docs_link_clicked,
    on_open_IDE_clicked,
)
from isaacsim.gui.components.ui_utils import SearchListItemModel


# Having a test class dervived from omni.kit.test.AsyncTestCase declared on the root of module will make it auto-discoverable by omni.kit.test
class TestUI(omni.kit.test.AsyncTestCase):
    """Test suite for basic UI components."""

    # Before running each test
    async def setUp(self) -> None:
        """Set up the test environment."""
        await omni.kit.app.get_app().next_update_async()

    # After running each test
    async def tearDown(self) -> None:
        """Clean up after each test."""
        await omni.kit.app.get_app().next_update_async()

    # Run for a single frame and exit
    async def test_ui(self) -> None:
        """Test basic UI frame update."""
        await omni.kit.app.get_app().next_update_async()

    # TODO: Disabling this test as it hangs on TC on shutdown
    # async def test_clipboard(self):
    #     import pyperclip

    #     on_copy_to_clipboard("test")
    #     try:
    #         self.assertEqual(pyperclip.paste(), "test")
    #     except pyperclip.PyperclipException:
    #         carb.log_warn(pyperclip.EXCEPT_MSG)
    #         return

    async def test_ide(self) -> None:
        """Test opening IDE from the UI."""
        import os

        on_open_IDE_clicked(os.path.dirname(__file__), __file__)

    # TODO: this test causes TC to hang on exit, disabling
    async def test_docs(self) -> None:
        """Test opening documentation link."""
        # on_open_folder_clicked(os.path.dirname(__file__)) # TODO: this test fails on TC due to permissions
        on_docs_link_clicked("https://docs.omniverse.nvidia.com")

    async def test_search_list_item_model_accepts_sequence_filter_text(self):
        """Test search list filtering with list and tuple text input."""
        for search_text in (["find", "this"], ("find", "this")):
            with self.subTest(search_text=search_text):
                model = SearchListItemModel("find this item", "other item")
                model.filter_text(search_text)

                self.assertEqual([item.name() for item in model.get_item_children(None)], ["find this item"])
