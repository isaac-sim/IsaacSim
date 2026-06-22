# SPDX-FileCopyrightText: Copyright (c) 2025-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""Tests for the Episode Recorder UI panel covering session, capture, and replay flows."""

import glob
import os
import tempfile

import isaacsim.core.experimental.utils.stage as stage_utils
import omni.kit.app
import omni.timeline
import omni.ui as ui
from isaacsim.replicator.episode_recorder.ui.episode_recorder_extension import EpisodeRecorderUIExtension
from isaacsim.replicator.episode_recorder.ui.episode_recorder_panel import EpisodeRecorderPanel
from isaacsim.replicator.episode_recorder.ui.episode_recorder_window import EpisodeRecorderWindow
from isaacsim.test.utils import MenuUITestCase
from pxr import Gf, UsdGeom, UsdPhysics

WINDOW_TITLE = EpisodeRecorderUIExtension.WINDOW_NAME
MENU_PATH = f"{EpisodeRecorderUIExtension.MENU_GROUP}/{EpisodeRecorderUIExtension.WINDOW_NAME}"

_FILE_PREFIX = "ui_test"
_NUM_FRAMES = 5


def _build_rigid_cube_scene() -> None:
    """Build a minimal scene with one recordable rigid cube."""
    stage_utils.define_prim("/World", "Xform")
    stage_utils.define_prim("/World/PhysicsScene", "PhysicsScene")
    cube_prim = stage_utils.define_prim("/World/Cube", "Cube")

    cube = UsdGeom.Cube(cube_prim)
    cube.GetSizeAttr().Set(0.25)

    xformable = UsdGeom.Xformable(cube_prim)
    xformable.ClearXformOpOrder()
    xformable.AddTranslateOp().Set(Gf.Vec3d(0.0, 0.0, 0.5))

    UsdPhysics.CollisionAPI.Apply(cube_prim)
    UsdPhysics.RigidBodyAPI.Apply(cube_prim)
    mass_api = UsdPhysics.MassAPI.Apply(cube_prim)
    mass_api.CreateMassAttr(1.0)


async def _open_panel(test: MenuUITestCase) -> tuple[EpisodeRecorderWindow, EpisodeRecorderPanel]:
    await test.menu_click_with_retry(MENU_PATH, window_name=WINDOW_TITLE)
    await test.find_widget_with_retry(WINDOW_TITLE)
    window = ui.Workspace.get_window(WINDOW_TITLE)
    test.assertIsNotNone(window, "Episode Recorder window should exist after opening via the menu")
    window.visible = True
    window.focus()
    await test.wait_n_frames(5)

    panel = window._panel
    test.assertIsNotNone(panel, "Episode Recorder panel should be initialized")
    return window, panel


def _configure_panel(panel: EpisodeRecorderPanel, output_dir: str, file_prefix: str = _FILE_PREFIX) -> None:
    panel._output_dir_field.model.set_value(output_dir)
    panel._file_prefix_field.model.set_value(file_prefix)
    panel._root_path_field.model.set_value("/World")


async def _record_session(
    test: MenuUITestCase,
    panel: EpisodeRecorderPanel,
    output_dir: str,
    file_prefix: str = _FILE_PREFIX,
) -> str:
    """Drive discover, session open, capture start/stop, and session close through the panel.

    Args:
        test: Test case used for UI assertions.
        panel: Episode Recorder panel under test.
        output_dir: Directory where the HDF5 session file is written.
        file_prefix: Prefix used when naming the session file.

    Returns:
        Path to the HDF5 file written by the recording session.
    """
    _configure_panel(panel, output_dir, file_prefix)

    panel._on_discover_clicked()
    await omni.kit.app.get_app().next_update_async()
    test.assertIn(
        "/World/Cube",
        set(panel._rigid_body_paths.values()),
        "Discovery should find the rigid body cube",
    )

    panel._auto_start_model.set_value(False)
    panel._on_session_toggle()
    await omni.kit.app.get_app().next_update_async()
    test.assertIsNotNone(panel._recorder, "Recorder should exist after Open Session")
    test.assertTrue(panel._recorder.is_session_open, "Session should be open after click")
    test.assertEqual(panel._session_btn.text, "Close Session")

    panel._on_episode_toggle()
    panel._sync_controls()
    test.assertTrue(panel._recorder.is_recording, "Start button should start recording")
    test.assertEqual(panel._episode_btn.text, "End")

    for _ in range(_NUM_FRAMES):
        panel._recorder._tick()
    test.assertEqual(panel._recorder.current_episode_frames, _NUM_FRAMES)

    panel._on_episode_toggle()
    panel._sync_controls()
    omni.timeline.get_timeline_interface().stop()
    test.assertFalse(panel._recorder.is_recording, "End button should stop recording")
    test.assertEqual(panel._episode_btn.text, "Start")

    panel._on_session_toggle()
    await omni.kit.app.get_app().next_update_async()
    test.assertIsNone(panel._recorder, "Recorder should be None after Close Session")
    test.assertEqual(panel._session_btn.text, "Open Session")

    files = sorted(glob.glob(os.path.join(output_dir, f"{file_prefix}_*.hdf5")))
    test.assertEqual(len(files), 1, f"Expected exactly one HDF5 file, got {files}")
    return files[0]


def _close_panel(window: EpisodeRecorderWindow | None, panel: EpisodeRecorderPanel | None) -> None:
    if panel is not None:
        panel._close_replayer_silently()
        panel._close_recorder_silently()
    if window is not None:
        window.destroy()


class TestEpisodeRecorderUICaptureReplay(MenuUITestCase):
    """Drive the Episode Recorder UI through capture and replay against a temp directory."""

    async def tearDown(self) -> None:
        """Stop the timeline and clean up the temporary UI test state."""
        omni.timeline.get_timeline_interface().stop()
        await super().tearDown()

    async def test_open_close_session_round_trip(self) -> None:
        """Opening then closing a session via the UI button leaves no recorder attached."""
        _build_rigid_cube_scene()
        window = panel = None

        with tempfile.TemporaryDirectory(prefix="episode_recorder_ui_") as tmp_dir:
            try:
                window, panel = await _open_panel(self)
                _configure_panel(panel, tmp_dir)

                panel._on_discover_clicked()
                await omni.kit.app.get_app().next_update_async()

                panel._on_session_toggle()
                await omni.kit.app.get_app().next_update_async()
                self.assertIsNotNone(panel._recorder, "Recorder should be opened by the toggle button")
                self.assertEqual(panel._session_btn.text, "Close Session")
                self.assertTrue(panel._episode_btn.enabled, "Episode button should enable once a session is open")

                panel._on_session_toggle()
                await omni.kit.app.get_app().next_update_async()
                self.assertIsNone(panel._recorder, "Recorder should be cleared by the toggle button")
                self.assertEqual(panel._session_btn.text, "Open Session")
                self.assertFalse(panel._episode_btn.enabled, "Episode button should disable when no session is open")
            finally:
                _close_panel(window, panel)

    async def test_capture_writes_hdf5_with_recorded_frames(self) -> None:
        """Driving the panel through start/stop produces an HDF5 file with the expected frame count."""
        from isaacsim.replicator.episode_recorder import SessionReader

        _build_rigid_cube_scene()
        window = panel = None

        with tempfile.TemporaryDirectory(prefix="episode_recorder_ui_") as tmp_dir:
            try:
                window, panel = await _open_panel(self)
                hdf5_path = await _record_session(self, panel, tmp_dir)

                reader = SessionReader(hdf5_path)
                try:
                    episodes = reader.list_episodes()
                    self.assertEqual(len(episodes), 1, "Recorded session should contain exactly one episode")
                    self.assertEqual(reader.num_frames(0), _NUM_FRAMES, "Frame count should match the tick count")
                finally:
                    reader.close()
            finally:
                _close_panel(window, panel)

    async def test_replay_loads_session_and_lists_episodes(self) -> None:
        """Loading a recorded session through the Replay panel populates the episode combo."""
        _build_rigid_cube_scene()
        window = panel = None

        with tempfile.TemporaryDirectory(prefix="episode_recorder_ui_") as tmp_dir:
            try:
                window, panel = await _open_panel(self)
                hdf5_path = await _record_session(self, panel, tmp_dir)

                panel._replay_file_field.model.set_value(hdf5_path)
                await omni.kit.app.get_app().next_update_async()
                panel._on_replay_load()
                await omni.kit.app.get_app().next_update_async()

                self.assertIsNotNone(panel._replayer, "Replayer should be created after Load")
                self.assertEqual(len(panel._replay_episodes), 1, "Replay should list one episode")
                self.assertEqual(panel._replay_frame_counts[0], _NUM_FRAMES)
                self.assertTrue(panel._replay_btn.enabled, "Replay Play button should be enabled after Load")
            finally:
                _close_panel(window, panel)

    async def test_replay_latest_button_fills_replay_field(self) -> None:
        """The 'Latest' shortcut points the replay file field at the most recent capture."""
        _build_rigid_cube_scene()
        window = panel = None

        with tempfile.TemporaryDirectory(prefix="episode_recorder_ui_") as tmp_dir:
            try:
                window, panel = await _open_panel(self)
                hdf5_path = await _record_session(self, panel, tmp_dir)

                panel._replay_file_field.model.set_value("")
                await omni.kit.app.get_app().next_update_async()
                panel._on_replay_latest()
                await omni.kit.app.get_app().next_update_async()

                self.assertEqual(
                    panel._replay_file_field.model.get_value_as_string(),
                    hdf5_path,
                    "Latest should fill the replay field with the most recent recording",
                )
            finally:
                _close_panel(window, panel)

    async def test_replay_start_stop_round_trip(self) -> None:
        """Start replay attaches the replayer; stopping detaches it without leaking state."""
        _build_rigid_cube_scene()
        window = panel = None

        with tempfile.TemporaryDirectory(prefix="episode_recorder_ui_") as tmp_dir:
            try:
                window, panel = await _open_panel(self)
                hdf5_path = await _record_session(self, panel, tmp_dir)

                panel._replay_file_field.model.set_value(hdf5_path)
                await omni.kit.app.get_app().next_update_async()
                panel._on_replay_load()
                await omni.kit.app.get_app().next_update_async()
                self.assertIsNotNone(panel._replayer, "Replayer should be loaded before Start Replay")

                panel._on_replay_toggle()

                attached = False
                for _ in range(60):
                    await omni.kit.app.get_app().next_update_async()
                    if panel._replay_attached:
                        attached = True
                        break
                self.assertTrue(attached, "Replay should attach within the polling window")
                self.assertIsNotNone(panel._replayer, "Replayer should still exist while attached")
                self.assertTrue(panel._replayer.is_replaying, "Replayer should report is_replaying while attached")

                panel._on_replay_toggle()
                for _ in range(10):
                    await omni.kit.app.get_app().next_update_async()

                self.assertFalse(panel._replay_attached, "Replay should detach after second toggle")
                self.assertFalse(
                    panel._replayer.is_replaying if panel._replayer is not None else False,
                    "Replayer should not be replaying after stop",
                )
            finally:
                _close_panel(window, panel)

    async def test_replay_pause_resume_roundtrip(self) -> None:
        """Pause toggle freezes the replay cursor; resume lets it advance again."""
        _build_rigid_cube_scene()
        window = panel = None

        with tempfile.TemporaryDirectory(prefix="episode_recorder_ui_") as tmp_dir:
            try:
                window, panel = await _open_panel(self)
                hdf5_path = await _record_session(self, panel, tmp_dir)

                panel._replay_file_field.model.set_value(hdf5_path)
                await omni.kit.app.get_app().next_update_async()
                panel._on_replay_load()
                await omni.kit.app.get_app().next_update_async()

                panel._on_replay_toggle()
                for _ in range(60):
                    await omni.kit.app.get_app().next_update_async()
                    if panel._replay_attached:
                        break
                self.assertTrue(panel._replay_attached, "Replay should attach before pause test")

                panel._on_replay_pause_toggle()
                await omni.kit.app.get_app().next_update_async()
                self.assertTrue(panel._replayer.is_paused, "Pause toggle should put replayer in paused state")
                paused_frame = panel._replayer.current_frame
                for _ in range(10):
                    await omni.kit.app.get_app().next_update_async()
                self.assertEqual(
                    panel._replayer.current_frame,
                    paused_frame,
                    "Frame cursor should not advance while paused",
                )

                panel._on_replay_pause_toggle()
                await omni.kit.app.get_app().next_update_async()
                self.assertFalse(panel._replayer.is_paused, "Second pause toggle should resume the replay")
            finally:
                _close_panel(window, panel)

    async def test_replay_step_clamps_at_bounds(self) -> None:
        """Step buttons are disabled at frame boundaries and `step_frame` clamps."""
        _build_rigid_cube_scene()
        window = panel = None

        with tempfile.TemporaryDirectory(prefix="episode_recorder_ui_") as tmp_dir:
            try:
                window, panel = await _open_panel(self)
                hdf5_path = await _record_session(self, panel, tmp_dir)

                panel._replay_file_field.model.set_value(hdf5_path)
                await omni.kit.app.get_app().next_update_async()
                panel._on_replay_load()
                await omni.kit.app.get_app().next_update_async()

                panel._on_replay_toggle()
                for _ in range(60):
                    await omni.kit.app.get_app().next_update_async()
                    if panel._replay_attached:
                        break

                panel._on_replay_pause_toggle()
                await omni.kit.app.get_app().next_update_async()

                panel._replayer._replay_frame = 0
                panel._sync_controls()
                self.assertFalse(
                    panel._replay_prev_btn.enabled,
                    "Step Backward button should be disabled at frame 0",
                )
                panel._on_replay_step(-1)
                await omni.kit.app.get_app().next_update_async()
                self.assertEqual(panel._replayer.current_frame, 0, "step_frame(-1) at frame 0 should be a no-op")

                last_frame = panel._replayer.total_frames - 1
                panel._replayer._replay_frame = last_frame
                panel._sync_controls()
                self.assertFalse(
                    panel._replay_next_btn.enabled,
                    "Step Forward button should be disabled at the last frame",
                )
                panel._on_replay_step(1)
                await omni.kit.app.get_app().next_update_async()
                self.assertEqual(
                    panel._replayer.current_frame,
                    last_frame,
                    "step_frame(+1) at the last frame should not advance past total_frames - 1",
                )
            finally:
                _close_panel(window, panel)

    async def test_replay_step_auto_pauses(self) -> None:
        """Stepping while playing must auto-pause so the stepped frame survives the next tick."""
        _build_rigid_cube_scene()
        window = panel = None

        with tempfile.TemporaryDirectory(prefix="episode_recorder_ui_") as tmp_dir:
            try:
                window, panel = await _open_panel(self)
                hdf5_path = await _record_session(self, panel, tmp_dir)

                panel._replay_file_field.model.set_value(hdf5_path)
                await omni.kit.app.get_app().next_update_async()
                panel._on_replay_load()
                await omni.kit.app.get_app().next_update_async()

                panel._on_replay_toggle()
                for _ in range(60):
                    await omni.kit.app.get_app().next_update_async()
                    if panel._replay_attached:
                        break
                self.assertFalse(panel._replayer.is_paused, "Replay should be running before step")

                panel._on_replay_step(1)
                await omni.kit.app.get_app().next_update_async()
                self.assertTrue(panel._replayer.is_paused, "Step should auto-pause the replay")
            finally:
                _close_panel(window, panel)

    async def test_replay_locks_recording_session(self) -> None:
        """Open Session and recorder-side widgets must be disabled while a replay is attached."""
        _build_rigid_cube_scene()
        window = panel = None

        with tempfile.TemporaryDirectory(prefix="episode_recorder_ui_") as tmp_dir:
            try:
                window, panel = await _open_panel(self)
                hdf5_path = await _record_session(self, panel, tmp_dir)

                panel._replay_file_field.model.set_value(hdf5_path)
                await omni.kit.app.get_app().next_update_async()
                panel._on_replay_load()
                await omni.kit.app.get_app().next_update_async()

                panel._on_replay_toggle()
                for _ in range(60):
                    await omni.kit.app.get_app().next_update_async()
                    if panel._replay_attached:
                        break
                self.assertTrue(panel._replay_attached, "Replay must be attached for the lock test")

                self.assertFalse(panel._session_btn.enabled, "Open Session should be disabled while replay is attached")
                self.assertFalse(
                    panel._episode_btn.enabled, "Start episode should be disabled while replay is attached"
                )
                self.assertFalse(panel._discover_btn.enabled, "Discover should be disabled while replay is attached")
                self.assertFalse(
                    panel._auto_start_check.enabled, "Auto-start checkbox should be disabled during replay"
                )
            finally:
                _close_panel(window, panel)

    async def test_recording_locks_replay(self) -> None:
        """Replay file/load/play widgets must be disabled while a recording session is open."""
        _build_rigid_cube_scene()
        window = panel = None

        with tempfile.TemporaryDirectory(prefix="episode_recorder_ui_") as tmp_dir:
            try:
                window, panel = await _open_panel(self)
                _configure_panel(panel, tmp_dir)

                panel._on_discover_clicked()
                await omni.kit.app.get_app().next_update_async()
                panel._auto_start_model.set_value(False)
                panel._on_session_toggle()
                await omni.kit.app.get_app().next_update_async()
                self.assertTrue(panel._recorder.is_session_open, "Session should be open before lock check")

                self.assertFalse(panel._replay_file_field.enabled, "Replay file field should be locked during session")
                self.assertFalse(panel._replay_load_btn.enabled, "Replay Load should be locked during session")
                self.assertFalse(panel._replay_btn.enabled, "Replay Play should be locked during session")

                panel._on_session_toggle()
                await omni.kit.app.get_app().next_update_async()
            finally:
                _close_panel(window, panel)

    async def test_replay_load_missing_file_shows_error(self) -> None:
        """Loading a non-existent path must report red status and leave no replayer attached."""
        window = panel = None

        with tempfile.TemporaryDirectory(prefix="episode_recorder_ui_") as tmp_dir:
            try:
                window, panel = await _open_panel(self)
                missing = os.path.join(tmp_dir, "does_not_exist.hdf5")
                panel._replay_file_field.model.set_value(missing)
                await omni.kit.app.get_app().next_update_async()
                panel._on_replay_load()
                await omni.kit.app.get_app().next_update_async()

                self.assertIsNone(panel._replayer, "No replayer should be attached after a missing-file load")
                self.assertFalse(panel._replay_btn.enabled, "Replay Play should remain disabled after a failed load")
            finally:
                _close_panel(window, panel)

    async def test_seek_timeline_checkbox_persists(self) -> None:
        """Toggling Seek timeline writes a persistent setting that survives panel rebuild."""
        import carb.settings

        window = panel = None
        settings = carb.settings.get_settings()
        key = "/persistent/exts/isaacsim.replicator.episode_recorder.ui/replay_seek_timeline"
        original = settings.get(key)
        try:
            window, panel = await _open_panel(self)
            self.assertIsNotNone(panel._replay_seek_timeline_model, "Seek timeline model should exist")

            panel._replay_seek_timeline_model.set_value(False)
            for _ in range(2):
                await omni.kit.app.get_app().next_update_async()
            self.assertFalse(bool(settings.get(key)), "Setting Seek timeline to False should persist as False")

            panel._replay_seek_timeline_model.set_value(True)
            for _ in range(2):
                await omni.kit.app.get_app().next_update_async()
            self.assertTrue(bool(settings.get(key)), "Setting Seek timeline to True should persist as True")
        finally:
            _close_panel(window, panel)
            if original is None:
                settings.destroy_item(key)
            else:
                settings.set_bool(key, bool(original))

    async def test_binding_badge_reflects_attach_detach_events(self) -> None:
        """Dispatching attach/detach binding events must update the panel badge text and tooltip."""
        from isaacsim.replicator.episode_recorder import dispatch_episode_binding

        window = panel = None

        try:
            window, panel = await _open_panel(self)
            self.assertIsNotNone(panel._binding_badge, "Binding badge UI element should be built")
            self.assertEqual(panel._binding_badge.text, "", "Badge should be empty before any bindings attach")

            dispatch_episode_binding(
                "attach",
                binding_id="vr_left_secondary",
                source="vr_button",
                label="VR Left Secondary",
                command="toggle",
            )
            for _ in range(3):
                await omni.kit.app.get_app().next_update_async()
            self.assertIn(
                "VR Left Secondary",
                panel._binding_badge.text,
                "Single attach should show the binding label in the badge",
            )
            self.assertIn(
                "VR Left Secondary",
                panel._binding_badge.tooltip or "",
                "Tooltip should describe the active binding",
            )
            self.assertIn(
                "toggle",
                panel._binding_badge.tooltip or "",
                "Tooltip should describe the bound command",
            )

            dispatch_episode_binding(
                "attach",
                binding_id="vr_right_secondary",
                source="vr_button",
                label="VR Right Secondary",
                command="end",
            )
            for _ in range(3):
                await omni.kit.app.get_app().next_update_async()
            self.assertIn(
                "2 bindings",
                panel._binding_badge.text,
                "Two active bindings should aggregate into a count",
            )

            dispatch_episode_binding(
                "detach",
                binding_id="vr_left_secondary",
                source="vr_button",
            )
            for _ in range(3):
                await omni.kit.app.get_app().next_update_async()
            self.assertIn(
                "VR Right Secondary",
                panel._binding_badge.text,
                "After one detach, the remaining binding label should be visible",
            )

            dispatch_episode_binding(
                "detach",
                binding_id="vr_right_secondary",
                source="vr_button",
            )
            for _ in range(3):
                await omni.kit.app.get_app().next_update_async()
            self.assertEqual(
                panel._binding_badge.text,
                "",
                "Badge should be empty once all bindings detach",
            )
        finally:
            _close_panel(window, panel)

    async def test_binding_badge_ignores_other_session_ids(self) -> None:
        """Binding events scoped to a different ``session_id`` must not appear in this panel's badge."""
        from isaacsim.replicator.episode_recorder import dispatch_episode_binding

        window = panel = None

        try:
            window, panel = await _open_panel(self)
            self.assertIsNone(panel._recorder, "Panel should not own a recorder before opening a session")

            dispatch_episode_binding(
                "attach",
                binding_id="vr_left_secondary",
                source="vr_button",
                label="VR Left Secondary",
                command="toggle",
                session_id="session-from-other-recorder",
            )
            for _ in range(3):
                await omni.kit.app.get_app().next_update_async()
            self.assertEqual(
                panel._binding_badge.text,
                "",
                "Scoped binding events must be ignored when no recorder is bound to this panel",
            )

            dispatch_episode_binding(
                "attach",
                binding_id="vr_left_secondary",
                source="vr_button",
                label="VR Left Secondary",
                command="toggle",
            )
            for _ in range(3):
                await omni.kit.app.get_app().next_update_async()
            self.assertIn(
                "VR Left Secondary",
                panel._binding_badge.text,
                "Unscoped binding events should always be tracked",
            )
            dispatch_episode_binding("detach", binding_id="vr_left_secondary", source="vr_button")
            for _ in range(3):
                await omni.kit.app.get_app().next_update_async()
            self.assertEqual(panel._binding_badge.text, "", "Badge should clear after detach")
        finally:
            _close_panel(window, panel)

    async def test_replay_warning_when_recorded_prims_missing_in_stage(self) -> None:
        """Loading a session whose prims are absent from the current stage must surface a warning."""
        _build_rigid_cube_scene()
        window = panel = None

        with tempfile.TemporaryDirectory(prefix="episode_recorder_ui_") as tmp_dir:
            try:
                window, panel = await _open_panel(self)
                hdf5_path = await _record_session(self, panel, tmp_dir)

                await self.new_stage()
                await self.wait_n_frames(3)

                panel._replay_file_field.model.set_value(hdf5_path)
                await omni.kit.app.get_app().next_update_async()
                panel._on_replay_load()
                await omni.kit.app.get_app().next_update_async()

                self.assertIsNotNone(panel._replay_warning_label, "Warning label should be built into the replay UI")
                self.assertTrue(
                    panel._replay_warning_row.visible,
                    "Warning row must be visible when recorded prims are missing from the current stage",
                )
                self.assertIn(
                    "/World/Cube",
                    panel._replay_warning_label.text,
                    "Warning should name the missing prim path",
                )
            finally:
                _close_panel(window, panel)

    async def test_replay_warning_clears_when_stage_matches_session(self) -> None:
        """When the recorded prims exist in the current stage, no warning must be shown."""
        _build_rigid_cube_scene()
        window = panel = None

        with tempfile.TemporaryDirectory(prefix="episode_recorder_ui_") as tmp_dir:
            try:
                window, panel = await _open_panel(self)
                hdf5_path = await _record_session(self, panel, tmp_dir)

                panel._replay_file_field.model.set_value(hdf5_path)
                await omni.kit.app.get_app().next_update_async()
                panel._on_replay_load()
                await omni.kit.app.get_app().next_update_async()

                self.assertIsNotNone(panel._replay_warning_label, "Warning label should be built into the replay UI")
                self.assertFalse(
                    panel._replay_warning_row.visible,
                    "Warning must stay hidden when all recorded prims resolve in the current stage",
                )
            finally:
                _close_panel(window, panel)
