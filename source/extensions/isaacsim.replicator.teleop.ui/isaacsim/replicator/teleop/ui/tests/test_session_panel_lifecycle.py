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

"""Tests for the desktop teleop session lifecycle."""

from types import SimpleNamespace
from unittest.mock import MagicMock, call, patch

import omni.kit.test
from isaacsim.replicator.teleop import TeleopCapabilities, TeleopCommand
from isaacsim.replicator.teleop.ui.session_panel import SessionPanel


def _live_capabilities() -> TeleopCapabilities:
    """Return capabilities for a platform with live teleop input."""
    return TeleopCapabilities(
        debug_input=True,
        scripted_motion=True,
        live_input=True,
        mcap_replay=True,
        pink_ik=True,
        native_input_unavailable_reason="",
        pink_ik_unavailable_reason="",
    )


class TestSessionPanelLifecycle(omni.kit.test.AsyncTestCase):
    """Keep desktop buttons on the complete TeleopManager command lifecycle."""

    def _make_panel(self) -> tuple[SessionPanel, MagicMock, MagicMock]:
        """Create a panel with mocked managers and live-input capabilities."""
        manager = MagicMock()
        manager.is_connected = False
        manager.is_live_tracking = False
        manager.debug_tracking_enabled = False
        manager.carry_tracking_space_available = False
        manager.carry_tracking_space_enabled = False
        markers = MagicMock()
        markers.has_active_markers = False
        with patch(
            "isaacsim.replicator.teleop.ui.session_panel.get_teleop_capabilities",
            return_value=_live_capabilities(),
        ):
            panel = SessionPanel(manager, markers, {})
        self.addCleanup(panel.destroy)
        panel._status_label = SimpleNamespace(text="", style={})
        return panel, manager, markers

    async def test_connect_uses_complete_command_lifecycle(self) -> None:
        """Route desktop Connect through TeleopManager.execute_command."""
        panel, manager, markers = self._make_panel()
        manager.execute_command.return_value = (True, "Connected")

        with (
            patch("isaacsim.replicator.teleop.ui.session_panel.stage_utils.is_stage_set", return_value=True),
            patch("isaacsim.replicator.teleop.ui.session_panel.stage_utils.is_stage_loading", return_value=False),
        ):
            panel._on_connect()

        manager.execute_command.assert_called_once_with(TeleopCommand.CONNECT)
        manager.connect.assert_not_called()
        markers.ensure_marker.assert_not_called()

    async def test_panel_registers_for_live_status_updates(self) -> None:
        """Keep detailed connection and data-status reporting on the command path."""
        panel, manager, _markers = self._make_panel()

        manager.set_on_status_changed.assert_called_once_with(panel._on_session_status_changed)
        status_callback = manager.set_on_status_changed.call_args.args[0]
        status_callback("Connected (no data)")

        self.assertEqual(panel._status_label.text, "Connected (no data)")

    async def test_connect_surfaces_command_failure(self) -> None:
        """Display a command-level connection failure without duplicating lifecycle work."""
        panel, manager, _markers = self._make_panel()
        manager.execute_command.return_value = (False, "Connection failed")

        with (
            patch("isaacsim.replicator.teleop.ui.session_panel.stage_utils.is_stage_set", return_value=True),
            patch("isaacsim.replicator.teleop.ui.session_panel.stage_utils.is_stage_loading", return_value=False),
        ):
            panel._on_connect()

        self.assertEqual(panel._status_label.text, "Connection failed")

    async def test_disconnect_uses_complete_command_lifecycle(self) -> None:
        """Route desktop Disconnect through TeleopManager.execute_command."""
        panel, manager, markers = self._make_panel()
        manager.execute_command.return_value = (True, "Disconnected")

        panel._on_disconnect()

        manager.execute_command.assert_called_once_with(TeleopCommand.DISCONNECT)
        manager.disconnect.assert_not_called()
        manager.disable_tracking_space.assert_not_called()
        markers.remove_all_markers.assert_not_called()

    async def test_disabling_debug_mode_removes_its_markers(self) -> None:
        """Stop debug input and drop its marker layer when the UI toggle turns off."""
        panel, manager, markers = self._make_panel()
        manager.debug_tracking_enabled = True

        panel._on_debug_tracking_toggled(False)

        manager.assert_has_calls(
            [call.execute_command(TeleopCommand.STOP), call.set_debug_tracking(False)],
            any_order=False,
        )
        manager.execute_command.assert_called_once_with(TeleopCommand.STOP)
        manager.set_debug_tracking.assert_called_once_with(False)
        manager.set_live_tracking.assert_called_once_with(False)
        manager.disable_tracking_space.assert_called_once_with()
        markers.remove_all_markers.assert_called_once_with()

    async def test_rejected_debug_enable_keeps_existing_markers(self) -> None:
        """A reset checkbox callback must not remove separately shown markers."""
        panel, manager, markers = self._make_panel()
        markers.has_active_markers = True

        panel._on_debug_tracking_toggled(False)

        markers.remove_all_markers.assert_not_called()

    async def test_debug_mode_cannot_start_after_timeline_play(self) -> None:
        """Reject late debug input while preserving the ability to switch an active input off."""
        panel, manager, markers = self._make_panel()
        panel._debug_tracking_cb = SimpleNamespace(enabled=True, model=MagicMock())

        with patch("isaacsim.replicator.teleop.ui.session_panel.app_utils.is_stopped", return_value=False):
            panel._sync_ui()
            self.assertFalse(panel._debug_tracking_cb.enabled)
            panel._on_debug_tracking_toggled(True)

            manager.set_debug_tracking.assert_not_called()
            markers.ensure_marker.assert_not_called()
            panel._debug_tracking_cb.model.set_value.assert_called_once_with(False)

            manager.debug_tracking_enabled = True
            panel._sync_ui()
            self.assertTrue(panel._debug_tracking_cb.enabled)
            panel._on_debug_tracking_toggled(False)

        manager.execute_command.assert_called_once_with(TeleopCommand.STOP)
        markers.remove_all_markers.assert_called_once_with()
