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

"""Tests for Isaac Sim application layout configurations."""

from __future__ import annotations

import itertools
import os
from pathlib import Path

import carb
import omni.kit.app
import omni.kit.ui_test as ui_test
from omni.kit.quicklayout import QuickLayout
from omni.kit.test import AsyncTestCase
from omni.ui.workspace_utils import CompareDelegate

_EXTENSION_FOLDER_PATH = Path(omni.kit.app.get_app().get_extension_manager().get_extension_path_by_module(__name__))


class TestLayoutsExtensions(AsyncTestCase):
    """Test suite for Isaac Sim window layout configurations.

    Validates that all layout JSON files in the extension's layouts folder
    can be loaded correctly and produce consistent window arrangements.
    """

    async def setUp(self) -> None:
        """Set up test environment by creating a new stage."""
        omni.usd.get_context().new_stage()
        await omni.kit.app.get_app().next_update_async()

    async def tearDown(self) -> None:
        """Clean up test environment after each test."""
        await omni.kit.app.get_app().next_update_async()
        super().tearDown()
        await omni.kit.app.get_app().next_update_async()

    async def test_isaacsim_layouts(self) -> None:
        """Test that all Isaac Sim layouts load correctly in all combinations.

        Iterates through all layout files in the layouts folder and tests loading
        them in all possible pair combinations. For each layout, verifies that
        the loaded state matches the expected configuration from the JSON file.
        """
        layouts_dir = _EXTENSION_FOLDER_PATH / "layouts"
        layout_files = [str(layouts_dir / f) for f in os.listdir(layouts_dir) if f.endswith(".json")]

        compare_delegate = CompareDelegate()

        async def test_layout(layout_path: str) -> None:
            """Load a layout file and verify it matches the expected configuration.

            Args:
                layout_path: Full path to the layout JSON file to test.
            """
            # Load layout twice as some layouts don't fully load on first attempt
            QuickLayout.load_file(layout_path, keep_windows_open=False)
            await ui_test.human_delay(50)
            QuickLayout.load_file(layout_path, keep_windows_open=False)
            await ui_test.human_delay(500)

            compare_errors = QuickLayout.compare_file(layout_path, compare_delegate)
            if compare_errors:
                layout_name = os.path.basename(layout_path)
                for error in compare_errors:
                    carb.log_error(f"{layout_name} compare_errors: {error}")

        for layout_a, layout_b in itertools.combinations(layout_files, 2):
            await test_layout(layout_a)
            await test_layout(layout_b)
