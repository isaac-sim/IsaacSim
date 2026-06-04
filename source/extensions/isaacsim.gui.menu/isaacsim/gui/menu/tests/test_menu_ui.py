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

"""UI tests for menu behaviors and links."""

import omni.kit.test
import omni.usd
from omni.kit.ui_test.menu import *
from omni.kit.ui_test.query import *
from omni.ui.tests.test_base import OmniUiTest


class TestMenuUI(OmniUiTest):
    """UI tests for menu link validation."""

    async def setUp(self) -> None:
        """Create a new stage before each test."""
        self._timeline = omni.timeline.get_timeline_interface()
        omni.usd.get_context().new_stage()
        # Make sure the stage loaded
        await omni.kit.app.get_app().next_update_async()

    # After running each test

    async def tearDown(self) -> None:
        """Wait for asset loads before teardown."""
        await omni.kit.app.get_app().next_update_async()
        # In some cases the test will end before the asset is loaded, in this case wait for assets to load
        while omni.usd.get_context().get_stage_loading_status()[2] > 0:
            await omni.kit.app.get_app().next_update_async()
        await super().tearDown()

    async def test_physics_reference_link(self) -> None:
        """Verify the physics reference link resolves to a live URL."""
        # Allow the UI to update so that the main menu is populated
        from urllib.error import URLError
        from urllib.request import urlopen

        from isaacsim.gui.menu.help_menu import resolve_physics_ref_url

        await omni.kit.app.get_app().next_update_async()

        physics_ref_url = resolve_physics_ref_url()

        def is_website_online(url: str) -> bool:
            """Check whether a URL responds successfully.

            Args:
                url: URL to check.

            Returns:
                True if the response is successful, False otherwise.
            """
            try:
                res = urlopen(url, timeout=1.0)
                print(f"testing link url", url)
                return res.status == 200
            except URLError:
                print("URL failed", url)
                return False

        self.assertTrue(is_website_online(physics_ref_url))
