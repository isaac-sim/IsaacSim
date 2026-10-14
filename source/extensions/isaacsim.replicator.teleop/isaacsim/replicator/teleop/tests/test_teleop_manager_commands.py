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

"""Tests for TeleopManager command lifecycle behavior."""

from collections.abc import Callable
from unittest.mock import MagicMock, call, patch

import omni.kit.test
from isaacsim.replicator.teleop import AnchorRotationMode, TeleopManager
from isaacsim.replicator.teleop.teleop_input import TeleopInputMode
from isaacsim.replicator.teleop.xr_anchor_manager import KitXrRuntimeState
from pxr import Gf


class TestTeleopManagerCommands(omni.kit.test.AsyncTestCase):
    """Validate command-only state that does not require a native OpenXR session."""

    async def test_connect_success_runs_full_lifecycle(self) -> None:
        """Set up tracking and the XR anchor without replacing the registered status callback."""
        manager = TeleopManager.__new__(TeleopManager)
        manager._is_connected = False
        manager._markers_manager = None
        manager.set_live_tracking = MagicMock()
        manager._reapply_tracking_space = MagicMock(return_value=(True, "Tracking Space: world origin"))
        manager._setup_xr_anchor = MagicMock(return_value=True)
        registered_callback = MagicMock()
        manager.set_on_status_changed(registered_callback)

        def _succeed_connect(*, on_status_changed: Callable[[str], None]) -> bool:
            on_status_changed("Connected")
            manager._on_status_changed = on_status_changed
            return True

        manager.connect = MagicMock(side_effect=_succeed_connect)

        success, message = manager._cmd_connect()

        self.assertTrue(success)
        self.assertEqual(message, "Connected")
        registered_callback.assert_called_once_with("Connected")
        manager.set_live_tracking.assert_called_once_with(True)
        manager._reapply_tracking_space.assert_called_once_with()
        manager._setup_xr_anchor.assert_called_once_with()
        self.assertIs(manager._on_status_changed, registered_callback)

    async def test_connect_failure_preserves_registered_status_callback(self) -> None:
        """Return detailed errors without retaining a reconnect wrapper."""
        manager = TeleopManager.__new__(TeleopManager)
        manager._is_connected = False
        registered_callback = MagicMock()
        manager.set_on_status_changed(registered_callback)

        def _fail_connect(*, on_status_changed: Callable[[str], None]) -> bool:
            on_status_changed("Error: CloudXR is not ready")
            manager._on_status_changed = on_status_changed
            return False

        manager.connect = MagicMock(side_effect=_fail_connect)

        success, message = manager._cmd_connect()

        self.assertFalse(success)
        self.assertEqual(message, "Error: CloudXR is not ready")
        registered_callback.assert_called_once_with("Error: CloudXR is not ready")
        self.assertIs(manager._on_status_changed, registered_callback)

    async def test_provider_open_failure_preserves_registered_status_callback(self) -> None:
        """Retain the attempt callback only for a provider that opened."""
        manager = TeleopManager.__new__(TeleopManager)
        manager._is_connected = False
        manager._debug_tracking_enabled = False
        registered_callback = MagicMock()
        manager.set_on_status_changed(registered_callback)

        provider = MagicMock()
        provider.input_mode = TeleopInputMode.MCAP_REPLAY
        provider.__enter__.side_effect = RuntimeError("MCAP open failed")

        attempt_callback = MagicMock()

        self.assertFalse(manager.connect_provider(provider, on_status_changed=attempt_callback))
        attempt_callback.assert_called_once_with("Error: MCAP open failed")
        self.assertIs(manager._on_status_changed, registered_callback)

    async def test_connect_rolls_back_when_anchor_prerequisites_fail(self) -> None:
        """Close the provider and remove runtime resources after partial setup."""
        manager = TeleopManager.__new__(TeleopManager)
        manager._is_connected = False
        manager._on_status_changed = None
        manager._xr_anchor = None
        manager._markers_manager = MagicMock()
        manager._markers_manager.ensure_marker.return_value = (True, "Created")
        manager.connect = MagicMock(return_value=True)
        manager.disconnect = MagicMock()
        manager.set_live_tracking = MagicMock()
        manager._reapply_tracking_space = MagicMock(return_value=(False, "Custom anchor missing"))
        manager._setup_xr_anchor = MagicMock(return_value=True)

        success, message = manager._cmd_connect()

        self.assertFalse(success)
        self.assertIn("Custom anchor missing", message)
        self.assertEqual(manager.set_live_tracking.call_args_list, [call(True), call(False)])
        manager._markers_manager.remove_all_markers.assert_called_once_with()
        manager.disconnect.assert_called_once_with()
        manager._setup_xr_anchor.assert_not_called()

    async def test_anchor_configuration_set_before_connect_is_applied_on_setup(self) -> None:
        """Apply cached profile and UI values when the XR anchor is created."""
        manager = TeleopManager.__new__(TeleopManager)
        manager._xr_anchor = None
        manager._input_provider = None
        manager._tracking_space_prim_path = "/World/TeleopSpace"
        manager._active_tracking_space_prim_path = "/World/TeleopSpace"
        manager._cached_tracking_space_frame = -1

        manager.set_xr_anchor_pos((1.5, -2.5, 0.75))
        manager.set_xr_anchor_rotation_mode(AnchorRotationMode.FOLLOW_PRIM_SMOOTHED)
        manager.set_xr_anchor_smoothing_time(0.75)
        manager.set_xr_anchor_fixed_height(False)

        with (
            patch(
                "isaacsim.replicator.teleop.teleop_manager.get_kit_xr_runtime_state",
                return_value=KitXrRuntimeState.UNAVAILABLE,
            ),
            patch("isaacsim.replicator.teleop.teleop_manager.XrAnchorManager") as anchor_type,
        ):
            anchor_type.return_value.setup.return_value = True
            self.assertTrue(manager._setup_xr_anchor())

        anchor_type.assert_called_once_with(
            anchor_pos=(1.5, -2.5, 0.75),
            tracking_space_prim_path="/World/TeleopSpace",
            rotation_mode=AnchorRotationMode.FOLLOW_PRIM_SMOOTHED,
            smoothing_time=0.75,
            fixed_height=False,
        )
        anchor_type.return_value.setup.assert_called_once_with()

    async def test_2d_live_connect_does_not_require_kit_xr_profile_settings(self) -> None:
        """Controller tracking remains usable when the 2D application does not load Kit XR."""
        manager = TeleopManager.__new__(TeleopManager)
        manager._xr_anchor = None
        manager._input_provider = MagicMock(input_mode=TeleopInputMode.LIVE)
        manager._tracking_space_prim_path = "/World/TeleopSpace"
        manager._active_tracking_space_prim_path = "/World/TeleopSpace"
        manager._cached_tracking_space_frame = -1
        manager._xr_anchor_pos = (0.0, 0.0, 0.0)
        manager._xr_anchor_rotation_mode = AnchorRotationMode.FIXED
        manager._xr_anchor_smoothing_time = 1.0
        manager._xr_anchor_fixed_height = True

        with (
            patch(
                "isaacsim.replicator.teleop.teleop_manager.get_kit_xr_runtime_state",
                return_value=KitXrRuntimeState.UNAVAILABLE,
            ),
            patch("isaacsim.replicator.teleop.teleop_manager.XrAnchorManager") as anchor_type,
        ):
            anchor_type.return_value.setup.return_value = True
            anchor_type.return_value.is_xr_profile_configured = False
            self.assertTrue(manager._setup_xr_anchor())

        anchor_type.return_value.cleanup.assert_not_called()
        self.assertIs(manager._xr_anchor, anchor_type.return_value)

    async def test_active_xr_connect_requires_verified_profile_anchor(self) -> None:
        """Fail safely when stereo rendering cannot consume the canonical Teleop anchor."""
        manager = TeleopManager.__new__(TeleopManager)
        manager._xr_anchor = None
        manager._input_provider = MagicMock(input_mode=TeleopInputMode.LIVE)
        manager._tracking_space_prim_path = "/World/TeleopSpace"
        manager._active_tracking_space_prim_path = "/World/TeleopSpace"
        manager._cached_tracking_space_frame = -1
        manager._xr_anchor_pos = (0.0, 0.0, 0.0)
        manager._xr_anchor_rotation_mode = AnchorRotationMode.FIXED
        manager._xr_anchor_smoothing_time = 1.0
        manager._xr_anchor_fixed_height = True

        with (
            patch(
                "isaacsim.replicator.teleop.teleop_manager.get_kit_xr_runtime_state",
                return_value=KitXrRuntimeState.ACTIVE,
            ),
            patch("isaacsim.replicator.teleop.teleop_manager.XrAnchorManager") as anchor_type,
            patch("isaacsim.replicator.teleop.teleop_manager.carb.log_error") as log_error,
        ):
            anchor_type.return_value.setup.return_value = True
            anchor_type.return_value.is_xr_profile_configured = False
            self.assertFalse(manager._setup_xr_anchor())

        anchor_type.return_value.cleanup.assert_called_once_with()
        log_error.assert_called_once()
        self.assertIsNone(manager._xr_anchor)

    async def test_anchor_configuration_changes_are_cached_and_applied_live(self) -> None:
        """Keep an active anchor and the next reconnect configuration in sync."""
        manager = TeleopManager.__new__(TeleopManager)
        manager._xr_anchor = MagicMock()
        manager._tracking_space_prim_path = "/World/TeleopSpace"
        manager._active_tracking_space_prim_path = "/World/TeleopSpace"
        manager._cached_tracking_space_frame = -1

        manager.set_xr_anchor_pos((1, 2, 3))
        manager.set_xr_anchor_rotation_mode(AnchorRotationMode.FOLLOW_PRIM)
        manager.set_xr_anchor_smoothing_time(0.0)
        manager.set_xr_anchor_fixed_height(False)

        self.assertEqual(manager._xr_anchor_pos, (1.0, 2.0, 3.0))
        self.assertEqual(manager._xr_anchor_rotation_mode, AnchorRotationMode.FOLLOW_PRIM)
        self.assertEqual(manager._xr_anchor_smoothing_time, 0.01)
        self.assertFalse(manager._xr_anchor_fixed_height)
        manager._xr_anchor.set_anchor_pos.assert_called_once_with((1.0, 2.0, 3.0))
        manager._xr_anchor.set_rotation_mode.assert_called_once_with(AnchorRotationMode.FOLLOW_PRIM)
        manager._xr_anchor.set_smoothing_time.assert_called_once_with(0.01)
        manager._xr_anchor.set_fixed_height.assert_called_once_with(False)

    async def test_pose_composition_uses_the_same_resolved_transform_as_xr(self) -> None:
        """Apply the final XR anchor pose instead of recomputing a divergent scene transform."""
        manager = TeleopManager.__new__(TeleopManager)
        manager._xr_anchor = MagicMock()
        manager._xr_anchor.sync.return_value = True
        yaw = Gf.Rotation(Gf.Vec3d(0.0, 0.0, 1.0), 90.0).GetQuat()
        manager._xr_anchor.get_world_pose.return_value = (Gf.Vec3d(10.0, 20.0, 0.0), yaw)
        manager._tracking_space_prim_path = "/World/CustomAnchor"
        manager._markers_manager = MagicMock()
        manager._frame_count = 1
        manager._cached_tracking_space_frame = -1
        manager._cached_tracking_space = None

        tracking_space = manager._get_tracking_space_transform()
        position, _orientation = manager._apply_tracking_space_offset(
            (1.0, 0.0, 0.0),
            (0.0, 0.0, 0.0, 1.0),
            tracking_space,
        )

        self.assertAlmostEqual(position[0], 10.0, places=6)
        self.assertAlmostEqual(position[1], 21.0, places=6)
        manager._markers_manager.set_origin_world_pose.assert_called_once()
