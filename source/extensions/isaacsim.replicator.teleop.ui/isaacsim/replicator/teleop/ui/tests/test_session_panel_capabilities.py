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

"""Tests for partial-support behavior in the teleop Session panel."""

from types import SimpleNamespace
from unittest.mock import MagicMock, patch

import omni.kit.test
from isaacsim.replicator.teleop import TeleopCapabilities
from isaacsim.replicator.teleop.ui.session_panel import SessionPanel


class TestSessionPanelCapabilities(omni.kit.test.AsyncTestCase):
    """Keep debug controls usable when native teleop input is unavailable."""

    async def test_unavailable_live_input_disables_only_connect(self) -> None:
        """Disable live Connect while leaving Debug Mode enabled."""
        capabilities = TeleopCapabilities(
            debug_input=True,
            scripted_motion=True,
            live_input=False,
            mcap_replay=False,
            pink_ik=False,
            native_input_unavailable_reason="Isaac Teleop unavailable; use Debug Mode instead",
            pink_ik_unavailable_reason="PINK unavailable",
        )
        manager = MagicMock()
        manager.is_connected = False
        manager.debug_tracking_enabled = False
        manager.carry_tracking_space_available = False
        manager.carry_tracking_space_enabled = False
        with patch("isaacsim.replicator.teleop.ui.session_panel.get_teleop_capabilities", return_value=capabilities):
            panel = SessionPanel(manager, MagicMock(), {})
        self.addCleanup(panel.destroy)

        panel._connect_btn = SimpleNamespace(enabled=True)
        panel._debug_tracking_cb = SimpleNamespace(enabled=False)
        panel._status_label = SimpleNamespace(text="", style={})
        panel._sync_ui()

        self.assertFalse(panel._connect_btn.enabled)
        self.assertTrue(panel._debug_tracking_cb.enabled)
        panel._on_connect()
        manager.connect.assert_not_called()
        self.assertIn("Debug Mode", panel._status_label.text)
