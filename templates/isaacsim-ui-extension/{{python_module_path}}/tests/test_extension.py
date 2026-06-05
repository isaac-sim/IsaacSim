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

"""Unit tests for {{extension_name}}."""

from __future__ import annotations

import omni.kit.app
import omni.kit.test
from isaacsim.examples.browser import get_instance as get_browser_instance


class TestExtension(omni.kit.test.AsyncTestCase):
    """Test cases for {{title}}."""

    async def setUp(self) -> None:
        """Set up test fixtures."""
        pass

    async def tearDown(self) -> None:
        """Clean up after each test."""
        pass

    async def test_extension_loaded(self) -> None:
        """Verify the extension is enabled and accessible."""
        ext_manager = omni.kit.app.get_app().get_extension_manager()
        self.assertTrue(ext_manager.is_extension_enabled("{{extension_name}}"))

    async def test_module_importable(self) -> None:
        """Verify the extension module can be imported."""
        import {{extension_name}}

        self.assertIsNotNone({{extension_name}})

    async def test_example_registered_in_browser(self) -> None:
        """Verify the example is registered with the Examples Browser."""
        browser = get_browser_instance()
        self.assertIsNotNone(browser)

        # Check the example is actually registered under the expected category.
        examples_by_category = browser._browser_model._examples
        self.assertIn("{{category}}", examples_by_category)
        example_names = [e.name for e in examples_by_category["{{category}}"]]
        self.assertIn("{{title}}", example_names)
