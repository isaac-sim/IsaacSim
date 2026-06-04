# SPDX-FileCopyrightText: Copyright (c) 2024-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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

"""Tests for the grasping UI window."""

import omni.ui as ui
from isaacsim.replicator.grasping.ui.grasping_ui_extension import GraspingUIExtension
from isaacsim.test.utils import MenuUITestCase

WINDOW_TITLE = GraspingUIExtension.WINDOW_NAME
MENU_PATH = f"{GraspingUIExtension.MENU_GROUP}/{GraspingUIExtension.WINDOW_NAME}"


class TestGraspingUIWindow(MenuUITestCase):
    """Test the grasping UI window lifecycle."""

    async def test_window_ui(self) -> None:
        """Verify the grasping UI window opens from the menu and renders without errors."""
        window = None
        try:
            await self.menu_click_with_retry(MENU_PATH, window_name=WINDOW_TITLE)
            await self.wait_n_frames(5)

            window = ui.Workspace.get_window(WINDOW_TITLE)
            self.assertIsNotNone(window, "Grasping window should exist after opening via the menu")

            widget = await self.find_widget_with_retry(WINDOW_TITLE)
            self.assertIsNotNone(widget, "Grasping window widget should be findable after opening")

            await self.menu_click_with_retry(MENU_PATH)
            window = None
            await self.wait_n_frames(5)
        finally:
            if window is not None:
                window.destroy()
