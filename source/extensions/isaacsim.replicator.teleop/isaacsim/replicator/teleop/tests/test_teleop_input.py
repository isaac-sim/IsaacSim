# SPDX-FileCopyrightText: Copyright (c) 2025-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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

"""Tests for typed live/debug/MCAP teleop input providers."""

from __future__ import annotations

from dataclasses import dataclass, field
from pathlib import Path
from tempfile import TemporaryDirectory
from types import SimpleNamespace
from unittest.mock import MagicMock, patch

import omni.kit.test
from isaacsim.replicator.teleop import (
    CoordinateSystem,
    LiveTeleopFrameProvider,
    McapTeleopFrameProvider,
    TeleopControllerState,
    TeleopFrame,
    TeleopFrameProvider,
    TeleopHeadState,
    TeleopInputMode,
    TeleopInputUnavailableError,
    TeleopManager,
    TeleopPollResult,
    TeleopPollStatus,
    TeleopPose,
)
from isaacsim.replicator.teleop.xr_anchor_manager import KitXrRuntimeState


@dataclass
class _Vec3:
    x: float
    y: float
    z: float


@dataclass
class _Quat:
    x: float
    y: float
    z: float
    w: float


@dataclass
class _Pose:
    position: _Vec3
    orientation: _Quat


@dataclass
class _TrackedPose:
    pose: _Pose | None = None
    is_valid: bool = True


@dataclass
class _Inputs:
    trigger_value: float = 0.0
    squeeze_value: float = 0.0
    thumbstick_x: float = 0.0
    thumbstick_y: float = 0.0
    primary_click: bool = False
    secondary_click: bool = False
    thumbstick_click: bool = False


@dataclass
class _ControllerSnapshot:
    inputs: _Inputs = field(default_factory=_Inputs)
    aim_pose: _TrackedPose = field(default_factory=_TrackedPose)


@dataclass
class _HeadSnapshot:
    pose: _Pose | None = None
    is_valid: bool = True


@dataclass
class _Tracked:
    data: object | None


def _pose(x: float, y: float, z: float, qx: float, qy: float, qz: float, qw: float) -> _TrackedPose:
    return _TrackedPose(_Pose(_Vec3(x, y, z), _Quat(qx, qy, qz, qw)))


class _FakeSession:
    def __init__(self, left: object | None, right: object | None, head: object | None) -> None:
        self.left = left
        self.right = right
        self.head = head
        self.update_count = 0
        self.close_count = 0

    def __enter__(self):  # noqa: ANN204
        return self

    def __exit__(self, _exc_type, _exc, _traceback) -> None:  # noqa: ANN001
        self.close_count += 1

    def update(self) -> None:
        self.update_count += 1


class _FakeControllerTracker:
    def get_left_controller(self, session: _FakeSession) -> _Tracked:
        return _Tracked(session.left)

    def get_right_controller(self, session: _FakeSession) -> _Tracked:
        return _Tracked(session.right)


class _FakeHeadTracker:
    def get_head(self, session: _FakeSession) -> _Tracked:
        return _Tracked(session.head)


class _FakeOpenXRSession:
    def __init__(self, app_name: str, extensions: list[str]) -> None:
        self.app_name = app_name
        self.extensions = extensions
        self.close_count = 0

    def __enter__(self):  # noqa: ANN204
        return self

    def __exit__(self, _exc_type, _exc, _traceback) -> None:  # noqa: ANN001
        self.close_count += 1

    def get_handles(self) -> str:
        return "handles"


class _FakeDeviceIOBindings:
    def __init__(self, sessions: list[_FakeSession]) -> None:
        self.sessions = sessions
        self.required_trackers: list[object] | None = None
        self.live_run_args: tuple | None = None
        self.replay_configs: list[object] = []
        self.recording_config: object | None = None
        self._next_session = 0

        owner = self

        class DeviceIOSession:
            @staticmethod
            def get_required_extensions(trackers: list[object]) -> list[str]:
                owner.required_trackers = trackers
                return ["XR_FAKE_extension"]

            @staticmethod
            def run(trackers: list[object], handles: object, recording_config: object) -> _FakeSession:
                owner.live_run_args = (trackers, handles, recording_config)
                return owner._take_session()

        class McapRecordingConfig:
            def __init__(self, filename: str, tracker_names: list[tuple[object, str]]) -> None:
                self.filename = filename
                self.tracker_names = tracker_names
                owner.recording_config = self

        class McapReplayConfig:
            def __init__(self, filename: str, tracker_names: list[tuple[object, str]]) -> None:
                self.filename = filename
                self.tracker_names = tracker_names
                owner.replay_configs.append(self)

        class ReplaySession:
            @staticmethod
            def run(_config: object) -> _FakeSession:
                return owner._take_session()

        self.api = SimpleNamespace(
            ControllerTracker=_FakeControllerTracker,
            HeadTracker=_FakeHeadTracker,
            DeviceIOSession=DeviceIOSession,
            McapRecordingConfig=McapRecordingConfig,
            McapReplayConfig=McapReplayConfig,
            ReplaySession=ReplaySession,
        )

    def _take_session(self) -> _FakeSession:
        session = self.sessions[self._next_session]
        self._next_session += 1
        return session


class TestTeleopInputProviders(omni.kit.test.AsyncTestCase):
    """Verify native lifecycle and frame semantics without requiring a headset."""

    async def test_live_provider_reads_frame_when_native_update_returns_none(self) -> None:
        """DeviceIOSession.update() returning None must not discard the frame."""
        left = _ControllerSnapshot(inputs=_Inputs(trigger_value=0.25), aim_pose=_pose(1, 2, 3, 0, 0, 0, 1))
        right = _ControllerSnapshot(aim_pose=_pose(4, 5, 6, 0.1, 0.2, 0.3, 0.9))
        head = _HeadSnapshot(pose=_Pose(_Vec3(7, 8, 9), _Quat(0.2, 0.3, 0.4, 0.8)))
        session = _FakeSession(left, right, head)
        bindings = _FakeDeviceIOBindings([session])

        with TemporaryDirectory() as tmp_dir:
            recording_path = Path(tmp_dir) / "input.mcap"
            with (
                patch("isaacsim.replicator.teleop.teleop_input._load_deviceio", return_value=bindings.api),
                patch(
                    "isaacsim.replicator.teleop.teleop_input._load_oxr",
                    return_value=SimpleNamespace(OpenXRSession=_FakeOpenXRSession),
                ),
                patch("isaacsim.replicator.teleop.teleop_input._get_kit_xr_session_handles", return_value=None),
            ):
                with LiveTeleopFrameProvider(mcap_recording_path=recording_path) as provider:
                    result = provider.poll()

        self.assertEqual(result.status, TeleopPollStatus.FRAME)
        assert result.frame is not None
        self.assertIs(result.frame.left.snapshot, left)
        self.assertIs(result.frame.right.snapshot, right)
        self.assertIs(result.frame.head.snapshot, head)
        self.assertEqual(result.frame.left.source_aim_pose.position, (1.0, 2.0, 3.0))
        self.assertEqual(result.frame.right.source_aim_pose.orientation_xyzw, (0.1, 0.2, 0.3, 0.9))
        self.assertEqual(session.update_count, 1)
        self.assertEqual(len(bindings.required_trackers or []), 2)
        self.assertEqual(bindings.live_run_args[1], "handles")
        self.assertEqual(bindings.recording_config.filename, str(recording_path))
        self.assertEqual([name for _, name in bindings.recording_config.tracker_names], ["controllers", "head"])
        self.assertEqual(session.close_count, 1)

    async def test_live_provider_shares_the_active_kit_openxr_session(self) -> None:
        """Stereo mode must attach DeviceIO to Kit's handles instead of creating a second session."""
        session = _FakeSession(None, None, None)
        bindings = _FakeDeviceIOBindings([session])
        openxr_session_type = MagicMock(side_effect=AssertionError("must not create a second OpenXR session"))
        handles_type = MagicMock(return_value="kit-shared-handles")
        oxr = SimpleNamespace(OpenXRSession=openxr_session_type, OpenXRSessionHandles=handles_type)
        bridge = SimpleNamespace(
            get_instance_handle=lambda: 11,
            get_session_handle=lambda: 22,
            get_stage_space_handle=lambda: 33,
            get_instance_proc_addr=lambda: 44,
        )

        with (
            patch("isaacsim.replicator.teleop.teleop_input._load_deviceio", return_value=bindings.api),
            patch("isaacsim.replicator.teleop.teleop_input._load_oxr", return_value=oxr),
            patch(
                "isaacsim.replicator.teleop.teleop_input.get_kit_xr_runtime_state",
                return_value=KitXrRuntimeState.ACTIVE,
            ),
            patch("isaacsim.replicator.teleop.teleop_input.importlib.import_module", return_value=bridge),
        ):
            with LiveTeleopFrameProvider():
                pass

        handles_type.assert_called_once_with(11, 22, 33, 44)
        openxr_session_type.assert_not_called()
        self.assertEqual(bindings.live_run_args[1], "kit-shared-handles")
        self.assertEqual(session.close_count, 1)

    async def test_active_kit_xr_without_bridge_fails_before_creating_another_session(self) -> None:
        """Report a configuration error instead of falling back to an invalid second XR session."""
        session = _FakeSession(None, None, None)
        bindings = _FakeDeviceIOBindings([session])
        openxr_session_type = MagicMock(side_effect=AssertionError("must not create a second OpenXR session"))
        oxr = SimpleNamespace(OpenXRSession=openxr_session_type)

        with (
            patch("isaacsim.replicator.teleop.teleop_input._load_deviceio", return_value=bindings.api),
            patch("isaacsim.replicator.teleop.teleop_input._load_oxr", return_value=oxr),
            patch(
                "isaacsim.replicator.teleop.teleop_input.get_kit_xr_runtime_state",
                return_value=KitXrRuntimeState.ACTIVE,
            ),
            patch(
                "isaacsim.replicator.teleop.teleop_input.importlib.import_module",
                side_effect=ModuleNotFoundError("bridge"),
            ),
        ):
            with self.assertRaisesRegex(TeleopInputUnavailableError, "handle bridge"):
                LiveTeleopFrameProvider().open()

        openxr_session_type.assert_not_called()

    async def test_live_provider_reports_missing_native_package(self) -> None:
        """Raise an actionable error when a caller bypasses TeleopManager capability checks."""
        capabilities = SimpleNamespace(
            live_input=False,
            native_input_unavailable_reason="Isaac Teleop unavailable; use Debug Mode instead",
        )
        with patch("isaacsim.replicator.teleop.teleop_input.get_teleop_capabilities", return_value=capabilities):
            with self.assertRaisesRegex(TeleopInputUnavailableError, "Debug Mode"):
                LiveTeleopFrameProvider().open()

    async def test_mcap_provider_is_headless_and_all_missing_is_still_a_frame(self) -> None:
        """Replay polls trackers after a None-returning update and does not infer EOF."""
        session = _FakeSession(None, None, None)
        bindings = _FakeDeviceIOBindings([session])

        with TemporaryDirectory() as tmp_dir:
            replay_path = Path(tmp_dir) / "input.mcap"
            replay_path.touch()
            with (
                patch("isaacsim.replicator.teleop.teleop_input._load_deviceio", return_value=bindings.api),
                patch(
                    "isaacsim.replicator.teleop.teleop_input._load_oxr",
                    side_effect=AssertionError("MCAP replay must not initialize OpenXR"),
                ),
            ):
                provider = McapTeleopFrameProvider(replay_path)
                provider.open()
                result = provider.poll()
                provider.close()
                provider.close()

        self.assertEqual(result.status, TeleopPollStatus.FRAME)
        assert result.frame is not None
        self.assertIsNone(result.frame.left.snapshot)
        self.assertIsNone(result.frame.right.snapshot)
        self.assertIsNone(result.frame.head.snapshot)
        self.assertEqual(result.frame.sequence_id, 1)
        self.assertEqual(session.update_count, 1)
        self.assertEqual(session.close_count, 1)
        self.assertEqual(bindings.replay_configs[0].filename, str(replay_path))
        self.assertEqual(
            [name for _, name in bindings.replay_configs[0].tracker_names],
            ["controllers", "head"],
        )

    async def test_mcap_rewind_recreates_replay_session(self) -> None:
        """Rewind closes and recreates the native replay session."""
        first = _FakeSession(None, None, None)
        second = _FakeSession(None, None, None)
        bindings = _FakeDeviceIOBindings([first, second])

        with TemporaryDirectory() as tmp_dir:
            replay_path = Path(tmp_dir) / "input.mcap"
            replay_path.touch()
            with patch("isaacsim.replicator.teleop.teleop_input._load_deviceio", return_value=bindings.api):
                provider = McapTeleopFrameProvider(replay_path)
                provider.open()
                provider.poll()
                provider.rewind()
                result = provider.poll()
                provider.close()

        self.assertEqual(first.close_count, 1)
        self.assertEqual(second.close_count, 1)
        self.assertEqual(len(bindings.replay_configs), 2)
        assert result.frame is not None
        self.assertEqual(result.frame.sequence_id, 1)

    async def test_missing_replay_file_fails_before_native_import(self) -> None:
        """A missing MCAP path must produce a useful error without loading OpenXR."""
        with TemporaryDirectory() as tmp_dir:
            provider = McapTeleopFrameProvider(Path(tmp_dir) / "missing.mcap")
            with patch(
                "isaacsim.replicator.teleop.teleop_input._load_deviceio",
                side_effect=AssertionError("native modules should not load for a missing file"),
            ):
                with self.assertRaisesRegex(FileNotFoundError, "MCAP replay file does not exist"):
                    provider.open()

    async def test_live_recording_rejects_existing_file_by_default(self) -> None:
        """Avoid silently truncating an existing MCAP recording."""
        with TemporaryDirectory() as tmp_dir:
            recording_path = Path(tmp_dir) / "existing.mcap"
            recording_path.touch()
            provider = LiveTeleopFrameProvider(mcap_recording_path=recording_path)
            with patch(
                "isaacsim.replicator.teleop.teleop_input._load_deviceio",
                side_effect=AssertionError("native modules should not load before overwrite validation"),
            ):
                with self.assertRaisesRegex(FileExistsError, "Pass overwrite=True"):
                    provider.open()


class _SequenceProvider(TeleopFrameProvider):
    def __init__(self, frame: TeleopFrame, mode: TeleopInputMode = TeleopInputMode.LIVE) -> None:
        self._frame = frame
        self._mode = mode
        self.open_count = 0
        self.poll_count = 0
        self.close_count = 0

    @property
    def input_mode(self) -> TeleopInputMode:
        return self._mode

    def open(self) -> None:
        self.open_count += 1

    def poll(self) -> TeleopPollResult:
        self.poll_count += 1
        return TeleopPollResult(TeleopPollStatus.FRAME, self._frame)

    def close(self) -> None:
        self.close_count += 1


class _FailingRewindProvider(_SequenceProvider):
    def rewind(self) -> bool:
        raise RuntimeError("rewind failed")


class _FakeIKController:
    def __init__(self) -> None:
        self.updates: list[tuple] = []

    def update_targets(self, *args) -> None:  # noqa: ANN002
        self.updates.append(args)

    def set_coordinate_system(self, _coordinate_system: object) -> None:
        return None

    def is_configured(self, _side: str) -> bool:
        return False

    def is_running(self, _side: str) -> bool:
        return False

    def destroy(self, _side: str) -> None:
        return None


class TestTeleopManagerInputRouting(omni.kit.test.AsyncTestCase):
    """Verify a provider frame traverses the existing manager pipeline once."""

    async def test_timeline_commands_only_request_needed_transitions(self) -> None:
        """Start and stop issue one uncommitted transition when state changes."""
        timeline = MagicMock()
        with patch("omni.timeline.get_timeline_interface", return_value=timeline):
            manager = TeleopManager()

            timeline.is_playing.return_value = False
            self.assertEqual(manager._cmd_start(), (True, "Timeline playing"))
            timeline.play.assert_called_once_with()
            timeline.commit.assert_not_called()

            timeline.reset_mock()
            timeline.is_playing.return_value = True
            self.assertEqual(manager._cmd_stop(), (True, "Timeline stopped"))
            timeline.stop.assert_called_once_with()
            timeline.commit.assert_not_called()

            manager.destroy()

    async def test_timeline_stop_keeps_the_input_session_connected(self) -> None:
        """Pause controllers without tearing down OpenXR or its external runtime."""
        frame = TeleopFrame(sequence_id=1, monotonic_time_ns=10, input_mode=TeleopInputMode.LIVE)
        provider = _SequenceProvider(frame)
        manager = TeleopManager()
        with (
            patch.object(manager, "_subscribe_xr_command_bus"),
            patch(
                "isaacsim.replicator.teleop.teleop_manager.app_utils.is_playing",
                return_value=True,
            ),
            patch("isaacsim.replicator.teleop.teleop_manager.app_utils.stop") as stop,
        ):
            self.assertTrue(manager.connect_provider(provider))
            self.assertEqual(manager._cmd_stop(), (True, "Timeline stopped"))

        stop.assert_called_once_with(commit=False)
        self.assertTrue(manager.is_connected)
        self.assertEqual(provider.close_count, 0)

        manager.destroy()
        self.assertEqual(provider.close_count, 1)

    async def test_connect_rejects_unavailable_live_input_before_cloudxr(self) -> None:
        """Keep live connection attempts deterministic when Isaac Teleop is absent."""
        capabilities = SimpleNamespace(
            live_input=False,
            mcap_replay=False,
            native_input_unavailable_reason="Isaac Teleop native input is unavailable; use Debug Mode instead",
        )
        statuses: list[str] = []
        manager = TeleopManager()
        with (
            patch("isaacsim.replicator.teleop.teleop_manager.get_teleop_capabilities", return_value=capabilities),
            patch("isaacsim.replicator.teleop.teleop_manager.prepare_live_cloudxr_env") as prepare_cloudxr,
        ):
            self.assertFalse(manager.connect(statuses.append))

        prepare_cloudxr.assert_not_called()
        self.assertIn("Debug Mode", statuses[0])
        manager.destroy()

    async def test_connect_rejects_unavailable_mcap_before_provider_creation(self) -> None:
        """Report that MCAP needs Isaac Teleop instead of opening a native provider."""
        capabilities = SimpleNamespace(
            live_input=False,
            mcap_replay=False,
            native_input_unavailable_reason="Isaac Teleop native input is unavailable; use Debug Mode instead",
        )
        statuses: list[str] = []
        manager = TeleopManager()
        with (
            patch("isaacsim.replicator.teleop.teleop_manager.get_teleop_capabilities", return_value=capabilities),
            patch("isaacsim.replicator.teleop.teleop_manager.McapTeleopFrameProvider") as provider_type,
        ):
            self.assertFalse(
                manager.connect(statuses.append, input_mode=TeleopInputMode.MCAP_REPLAY, mcap_path="input.mcap")
            )

        provider_type.assert_not_called()
        self.assertIn("Isaac Teleop", statuses[0])
        manager.destroy()

    async def test_live_provider_frame_routes_to_legacy_and_typed_observers(self) -> None:
        """Preserve raw snapshots while exposing finalized source/local/world poses."""
        left = _ControllerSnapshot(aim_pose=_pose(1, 2, 3, 0, 0, 0, 1))
        right = _ControllerSnapshot(aim_pose=_pose(4, 5, 6, 0.1, 0.2, 0.3, 0.9))
        head = _HeadSnapshot(pose=_Pose(_Vec3(7, 8, 9), _Quat(0.2, 0.3, 0.4, 0.8)))
        frame = TeleopFrame(
            sequence_id=1,
            monotonic_time_ns=10,
            input_mode=TeleopInputMode.LIVE,
            left=TeleopControllerState(snapshot=left, source_aim_pose=TeleopPose.from_values((1, 2, 3), (0, 0, 0, 1))),
            right=TeleopControllerState(
                snapshot=right,
                source_aim_pose=TeleopPose.from_values((4, 5, 6), (0.1, 0.2, 0.3, 0.9)),
            ),
            head=TeleopHeadState(
                snapshot=head,
                source_pose=TeleopPose.from_values((7, 8, 9), (0.2, 0.3, 0.4, 0.8)),
            ),
        )
        provider = _SequenceProvider(frame)
        manager = TeleopManager()
        manager.set_coordinate_system(CoordinateSystem.RAW)
        manager._tracking_space_retry_failed = True
        legacy_controller_frames: list[tuple[object | None, object | None]] = []
        legacy_head_frames: list[object | None] = []
        typed_frames: list[TeleopFrame] = []
        manager.add_controller_inputs_observer(
            lambda left_ctrl, right_ctrl: legacy_controller_frames.append((left_ctrl, right_ctrl))
        )
        manager.add_head_observer(legacy_head_frames.append)
        manager.add_input_frame_observer(typed_frames.append)
        ik = _FakeIKController()
        manager.set_ik_controller(ik)

        with patch.object(manager, "_subscribe_xr_command_bus"):
            self.assertTrue(manager.connect_provider(provider))
        manager._on_update(None)

        self.assertEqual(provider.poll_count, 1)
        self.assertEqual(legacy_controller_frames, [(left, right)])
        self.assertEqual(legacy_head_frames, [head])
        self.assertEqual(len(typed_frames), 1)
        self.assertEqual(typed_frames[0].left.local_aim_pose.position, (1, 2, 3))
        self.assertEqual(typed_frames[0].left.world_aim_pose.position, (1, 2, 3))
        self.assertEqual(manager.get_input_world_position("right"), (4, 5, 6))
        self.assertEqual(ik.updates[0][0], (1, 2, 3))
        self.assertEqual(ik.updates[0][2], (4, 5, 6))

        manager.destroy()
        self.assertEqual(provider.close_count, 1)

    async def test_mcap_provider_does_not_advance_while_timeline_is_stopped(self) -> None:
        """Do not consume caller-paced MCAP frames while physics is paused."""
        frame = TeleopFrame(sequence_id=1, monotonic_time_ns=10, input_mode=TeleopInputMode.MCAP_REPLAY)
        provider = _SequenceProvider(frame, TeleopInputMode.MCAP_REPLAY)
        manager = TeleopManager()
        timeline = SimpleNamespace(is_playing=lambda: False)
        manager.connect_provider(provider)

        with patch(
            "isaacsim.replicator.teleop.teleop_manager.omni.timeline.get_timeline_interface",
            return_value=timeline,
        ):
            manager._on_update(None)

        self.assertEqual(provider.poll_count, 0)
        manager.destroy()

    async def test_raw_observers_run_before_frame_finalization(self) -> None:
        """A coordinate or marker failure must not drop raw recorder/button callbacks."""
        left = _ControllerSnapshot()
        frame = TeleopFrame(
            sequence_id=1,
            monotonic_time_ns=10,
            input_mode=TeleopInputMode.LIVE,
            left=TeleopControllerState(snapshot=left),
        )
        provider = _SequenceProvider(frame)
        manager = TeleopManager()
        observed: list[object | None] = []
        manager.add_controller_inputs_observer(lambda left_ctrl, _right_ctrl: observed.append(left_ctrl))
        with patch.object(manager, "_subscribe_xr_command_bus"):
            manager.connect_provider(provider)

        with (
            patch.object(manager, "_finalize_source_frame", side_effect=RuntimeError("finalization failed")),
            self.assertRaisesRegex(RuntimeError, "finalization failed"),
        ):
            manager._on_update(None)

        self.assertEqual(observed, [left])
        manager.destroy()

    async def test_failed_mcap_rewind_disconnects_provider(self) -> None:
        """Do not report connected after RESET leaves a replay provider closed."""
        frame = TeleopFrame(sequence_id=1, monotonic_time_ns=10, input_mode=TeleopInputMode.MCAP_REPLAY)
        provider = _FailingRewindProvider(frame, TeleopInputMode.MCAP_REPLAY)
        manager = TeleopManager()
        manager.connect_provider(provider)
        timeline = SimpleNamespace(is_playing=lambda: False, set_current_time=lambda _time: None)

        with (
            patch(
                "isaacsim.replicator.teleop.teleop_manager.omni.timeline.get_timeline_interface",
                return_value=timeline,
            ),
            patch.object(manager, "_reapply_tracking_space", return_value=(True, "")),
        ):
            success, message = manager._cmd_reset()

        self.assertFalse(success)
        self.assertIn("replay was disconnected", message)
        self.assertFalse(manager.is_connected)
        self.assertEqual(provider.close_count, 1)
        manager.destroy()

    async def test_second_provider_is_rejected_without_taking_ownership(self) -> None:
        """Do not silently ignore or close a caller's provider while another source is active."""
        frame = TeleopFrame(sequence_id=1, monotonic_time_ns=10, input_mode=TeleopInputMode.LIVE)
        first = _SequenceProvider(frame)
        second = _SequenceProvider(frame)
        statuses: list[str] = []
        manager = TeleopManager()
        with patch.object(manager, "_subscribe_xr_command_bus"):
            self.assertTrue(manager.connect_provider(first))
            self.assertFalse(manager.connect_provider(second, statuses.append))

        self.assertEqual(second.open_count, 0)
        self.assertEqual(second.close_count, 0)
        self.assertIn("already connected", statuses[0].lower())
        manager.destroy()
