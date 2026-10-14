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

"""Tests for the Teleop UI Profiles panel covering profile load and save round-trips."""

import os
import tempfile

import isaacsim.core.experimental.utils.app as app_utils
import isaacsim.core.experimental.utils.stage as stage_utils
import omni.ui as ui
from isaacsim.replicator.teleop import (
    AnchorRotationMode,
    BimanualControllerProfile,
    ControllerSideProfile,
    TeleopProfile,
    TeleopSettingsProfile,
    save_teleop_profile,
)
from isaacsim.replicator.teleop.ui.teleop_ui_extension import TeleopUIExtension
from isaacsim.test.utils import MenuUITestCase

WINDOW_TITLE = TeleopUIExtension.WINDOW_NAME
MENU_PATH = f"{TeleopUIExtension.MENU_GROUP}/{TeleopUIExtension.WINDOW_NAME}"


class TestTeleopUIProfile(MenuUITestCase):
    """Round-trip a unified teleop profile through the live Profiles panel."""

    async def test_new_stage_resets_debug_mode_and_validation_status(self) -> None:
        """A stage replacement clears debug runtime input without discarding the selected profile."""
        app_utils.stop()
        await stage_utils.create_new_stage_async()
        window = None

        with tempfile.TemporaryDirectory(prefix="teleop_ui_stage_reset_") as tmp_dir:
            try:
                await self.menu_click_with_retry(MENU_PATH, window_name=WINDOW_TITLE)
                window = ui.Workspace.get_window(WINDOW_TITLE)
                self.assertIsNotNone(window)
                window._last_profile_path = os.path.join(tmp_dir, "last_profile.yaml")
                window.visible = True
                await self.wait_n_frames(3)

                session = window._session_panel
                profiles = window._teleop_profile_panel
                debug_checkbox = session._debug_tracking_cb
                grasp_slider = next(
                    widget for widget in session._debug_input_widgets if isinstance(widget, ui.FloatSlider)
                )
                profiles._status_label.text = "Ready"
                debug_checkbox.model.set_value(True)
                grasp_slider.model.set_value(0.75)
                await self.wait_n_frames(2)
                self.assertTrue(window._teleop_manager.debug_tracking_enabled)
                self.assertAlmostEqual(window._teleop_manager._debug_left_snapshot.inputs.trigger_value, 0.75)

                await stage_utils.create_new_stage_async()
                await self.wait_n_frames(2)
                self.assertFalse(window._teleop_manager.debug_tracking_enabled)
                self.assertFalse(debug_checkbox.model.get_value_as_bool())
                self.assertAlmostEqual(grasp_slider.model.get_value_as_float(), 0.0)
                self.assertAlmostEqual(window._teleop_manager._debug_left_snapshot.inputs.trigger_value, 0.0)
                self.assertEqual(profiles._status_label.text, "Stage changed - revalidate profile")

                debug_checkbox.model.set_value(True)
                await self.wait_n_frames(2)
                self.assertTrue(window._teleop_manager.debug_tracking_enabled)
            finally:
                app_utils.stop()
                if window is not None:
                    window.destroy()
                stage_utils.close_stage()

    async def test_load_requires_stopped_timeline(self) -> None:
        """Disable profile loading during Play and Pause without changing live panel state."""
        window = None
        app_utils.stop()

        with tempfile.TemporaryDirectory(prefix="teleop_ui_profile_") as tmp_dir:
            try:
                await self.menu_click_with_retry(MENU_PATH, window_name=WINDOW_TITLE)
                window = ui.Workspace.get_window(WINDOW_TITLE)
                self.assertIsNotNone(window)
                window._last_profile_path = os.path.join(tmp_dir, "last_profile.yaml")
                window.visible = True
                await self.wait_n_frames(5)

                profile = TeleopProfile(session=TeleopSettingsProfile(marker_scale=0.137))
                ok, message = save_teleop_profile(os.path.join(tmp_dir, "while_playing.yaml"), profile)
                self.assertTrue(ok, message)
                panel = window._teleop_profile_panel
                debug_checkbox = window._session_panel._debug_tracking_cb
                marker_scale_field = window._session_panel._marker_scale_field
                marker_scale_field.model.set_value(0.211)
                panel._dir_field.model.set_value(tmp_dir)
                panel._on_directory_changed()
                await self.wait_n_frames(2)
                names = [name for name, _ in panel._profiles]
                panel._profile_combo.model.get_item_value_model().set_value(names.index("while_playing"))
                self.assertTrue(panel._load_button.enabled)
                self.assertTrue(debug_checkbox.enabled)

                app_utils.play()
                await self.wait_n_frames(3)
                self.assertFalse(panel._load_button.enabled)
                self.assertFalse(debug_checkbox.enabled)
                panel._on_load_clicked()
                self.assertAlmostEqual(marker_scale_field.model.get_value_as_float(), 0.211, places=4)
                applied, reason = window.apply_teleop_profile(profile)
                self.assertFalse(applied, reason)

                app_utils.pause()
                await self.wait_n_frames(3)
                self.assertFalse(panel._load_button.enabled)
                self.assertFalse(debug_checkbox.enabled)
                panel._on_load_clicked()
                self.assertAlmostEqual(marker_scale_field.model.get_value_as_float(), 0.211, places=4)

                app_utils.stop()
                await self.wait_n_frames(3)
                self.assertTrue(panel._load_button.enabled)
                self.assertTrue(debug_checkbox.enabled)
                panel._on_load_clicked()
                self.assertAlmostEqual(marker_scale_field.model.get_value_as_float(), 0.137, places=4)
            finally:
                app_utils.stop()
                if window is not None:
                    window._last_profile_path = os.path.join(tmp_dir, "last_profile.yaml")
                    window.destroy()

    async def test_load_profile_applies_to_session_panel(self) -> None:
        """A YAML profile loaded through the Profiles panel must update session state."""
        profile_name = "ui_test_profile"
        window = None

        with tempfile.TemporaryDirectory(prefix="teleop_ui_profile_") as tmp_dir:
            try:
                await self.menu_click_with_retry(MENU_PATH, window_name=WINDOW_TITLE)
                window = ui.Workspace.get_window(WINDOW_TITLE)
                self.assertIsNotNone(window, "Teleop window should exist after opening via the menu")
                window._last_profile_path = os.path.join(tmp_dir, "last_profile.yaml")
                window.visible = True
                window.focus()
                await self.wait_n_frames(5)

                profile = TeleopProfile(
                    session=TeleopSettingsProfile(
                        coordinate_system="raw",
                        marker_scale=0.137,
                        anchor_x=1.5,
                        anchor_y=-2.5,
                        anchor_z=0.75,
                        anchor_rotation_mode=AnchorRotationMode.FOLLOW_PRIM_SMOOTHED.value,
                        anchor_smoothing=0.75,
                        anchor_fixed_height=False,
                    ),
                )
                profile_path = os.path.join(tmp_dir, f"{profile_name}.yaml")
                ok, message = save_teleop_profile(profile_path, profile)
                self.assertTrue(ok, message)

                profile_panel = window._teleop_profile_panel
                session_panel = window._session_panel
                self.assertIsNotNone(profile_panel, "Profiles panel should be initialized")
                self.assertIsNotNone(session_panel, "Session panel should be initialized")
                self.assertIsNotNone(profile_panel._dir_field, "Profile directory field should exist")
                self.assertIsNotNone(profile_panel._profile_combo, "Profile selection combo should exist")

                profile_panel._dir_field.model.set_value(tmp_dir)
                profile_panel._on_directory_changed()
                await self.wait_n_frames(2)

                profile_names = [name for name, _ in profile_panel._profiles]
                self.assertIn(profile_name, profile_names, f"Profile '{profile_name}' should be discovered")
                index = profile_names.index(profile_name)
                profile_panel._profile_combo.model.get_item_value_model().set_value(index)

                profile_panel._on_load_clicked()
                await self.wait_n_frames(5)

                self.assertIsNotNone(session_panel._marker_scale_field, "Marker scale field should exist after load")
                self.assertAlmostEqual(session_panel._marker_scale_field.model.get_value_as_float(), 0.137, places=4)
                self.assertAlmostEqual(session_panel._anchor_x_field.model.get_value_as_float(), 1.5, places=4)
                self.assertAlmostEqual(session_panel._anchor_y_field.model.get_value_as_float(), -2.5, places=4)
                self.assertAlmostEqual(session_panel._anchor_z_field.model.get_value_as_float(), 0.75, places=4)

                manager = window._teleop_manager
                self.assertEqual(manager._xr_anchor_pos, (1.5, -2.5, 0.75))
                self.assertEqual(manager._xr_anchor_rotation_mode, AnchorRotationMode.FOLLOW_PRIM_SMOOTHED)
                self.assertAlmostEqual(manager._xr_anchor_smoothing_time, 0.75, places=4)
                self.assertFalse(manager._xr_anchor_fixed_height)

                applied = window.collect_teleop_profile()
                self.assertEqual(applied.session.coordinate_system, "raw")
            finally:
                if window is not None:
                    window._last_profile_path = os.path.join(tmp_dir, "last_profile.yaml")
                    window.destroy()

    async def test_save_then_reload_round_trip(self) -> None:
        """Saving via the Save row and then loading must restore the saved values."""
        profile_name = "ui_test_profile"
        window = None

        with tempfile.TemporaryDirectory(prefix="teleop_ui_profile_") as tmp_dir:
            try:
                await self.menu_click_with_retry(MENU_PATH, window_name=WINDOW_TITLE)
                window = ui.Workspace.get_window(WINDOW_TITLE)
                self.assertIsNotNone(window, "Teleop window should exist after opening via the menu")
                window._last_profile_path = os.path.join(tmp_dir, "last_profile.yaml")
                window.visible = True
                window.focus()
                await self.wait_n_frames(5)

                profile_panel = window._teleop_profile_panel
                session_panel = window._session_panel
                self.assertIsNotNone(profile_panel, "Profiles panel should be initialized")
                self.assertIsNotNone(session_panel, "Session panel should be initialized")

                profile_panel._dir_field.model.set_value(tmp_dir)
                profile_panel._on_directory_changed()
                await self.wait_n_frames(2)

                marker_scale_field = session_panel._marker_scale_field
                self.assertIsNotNone(marker_scale_field, "Session panel marker scale field should exist")
                marker_scale_field.model.set_value(0.211)
                await self.wait_n_frames(2)

                self.assertIsNotNone(profile_panel._save_name_field, "Save name field should exist")
                profile_panel._save_name_field.model.set_value(profile_name)
                profile_panel._on_save_confirm()
                await self.wait_n_frames(2)

                saved_path = os.path.join(tmp_dir, f"{profile_name}.yaml")
                self.assertTrue(os.path.isfile(saved_path), f"Profile file was not written: {saved_path}")

                marker_scale_field.model.set_value(0.099)
                await self.wait_n_frames(2)

                profile_names = [name for name, _ in profile_panel._profiles]
                self.assertIn(profile_name, profile_names, "Saved profile should appear in the rescanned list")
                index = profile_names.index(profile_name)
                profile_panel._profile_combo.model.get_item_value_model().set_value(index)
                profile_panel._on_load_clicked()
                await self.wait_n_frames(5)

                self.assertAlmostEqual(marker_scale_field.model.get_value_as_float(), 0.211, places=4)
            finally:
                if window is not None:
                    window._last_profile_path = os.path.join(tmp_dir, "last_profile.yaml")
                    window.destroy()

    async def test_save_existing_profile_shows_overwrite_dialog(self) -> None:
        """Saving over an existing profile must open a confirm dialog before writing."""
        profile_name = "existing_profile"
        window = None

        with tempfile.TemporaryDirectory(prefix="teleop_ui_profile_") as tmp_dir:
            try:
                await self.menu_click_with_retry(MENU_PATH, window_name=WINDOW_TITLE)
                window = ui.Workspace.get_window(WINDOW_TITLE)
                self.assertIsNotNone(window, "Teleop window should exist after opening via the menu")
                window._last_profile_path = os.path.join(tmp_dir, "last_profile.yaml")
                window.visible = True
                window.focus()
                await self.wait_n_frames(5)

                profile_panel = window._teleop_profile_panel
                session_panel = window._session_panel

                profile_panel._dir_field.model.set_value(tmp_dir)
                profile_panel._on_directory_changed()
                await self.wait_n_frames(2)

                seed = TeleopProfile(session=TeleopSettingsProfile(marker_scale=0.111))
                saved_path = os.path.join(tmp_dir, f"{profile_name}.yaml")
                ok, message = save_teleop_profile(saved_path, seed)
                self.assertTrue(ok, message)
                profile_panel._rescan_profiles()
                await self.wait_n_frames(2)

                session_panel._marker_scale_field.model.set_value(0.222)
                profile_panel._save_name_field.model.set_value(profile_name)

                profile_panel._on_save_confirm()
                await self.wait_n_frames(2)

                self.assertIsNotNone(
                    profile_panel._confirm_dialog,
                    "Saving over an existing file must open the overwrite confirm dialog",
                )
                self.assertEqual(
                    profile_panel._pending_overwrite_path,
                    saved_path,
                    "Pending overwrite path must point at the file being replaced",
                )

                with open(saved_path, encoding="utf-8") as fh:
                    before_confirm = fh.read()
                self.assertIn("0.111", before_confirm, "Showing the dialog must not modify the existing YAML")

                profile_panel._confirm_dialog._on_cancel()
                await self.wait_n_frames(2)
                with open(saved_path, encoding="utf-8") as fh:
                    after_cancel = fh.read()
                self.assertIn("0.111", after_cancel, "Cancelling the dialog must not modify the existing YAML")
                self.assertEqual(
                    profile_panel._pending_overwrite_path,
                    "",
                    "Cancel must clear the pending overwrite path",
                )

                profile_panel._on_save_confirm()
                await self.wait_n_frames(2)
                self.assertIsNotNone(profile_panel._confirm_dialog, "Second Save click must reopen the dialog")
                profile_panel._confirm_dialog._on_okay()
                await self.wait_n_frames(2)

                with open(saved_path, encoding="utf-8") as fh:
                    after_overwrite = fh.read()
                self.assertIn("0.222", after_overwrite, "Confirming the dialog must overwrite the YAML")
                self.assertEqual(
                    profile_panel._pending_overwrite_path,
                    "",
                    "Overwrite must clear the pending overwrite path",
                )
            finally:
                if window is not None:
                    window._last_profile_path = os.path.join(tmp_dir, "last_profile.yaml")
                    window.destroy()

    async def test_delete_removes_file_and_clears_last_profile_setting(self) -> None:
        """Delete must remove the file from disk, rescan the list, and clear the last_profile setting."""
        import carb.settings

        profile_name = "to_be_deleted"
        window = None
        settings = carb.settings.get_settings()
        last_profile_key = "/persistent/exts/isaacsim.replicator.teleop/teleop_profiles/last_profile"

        with tempfile.TemporaryDirectory(prefix="teleop_ui_profile_") as tmp_dir:
            try:
                await self.menu_click_with_retry(MENU_PATH, window_name=WINDOW_TITLE)
                window = ui.Workspace.get_window(WINDOW_TITLE)
                self.assertIsNotNone(window, "Teleop window should exist after opening via the menu")
                window._last_profile_path = os.path.join(tmp_dir, "last_profile.yaml")
                window.visible = True
                window.focus()
                await self.wait_n_frames(5)

                profile_panel = window._teleop_profile_panel
                profile_panel._dir_field.model.set_value(tmp_dir)
                profile_panel._on_directory_changed()
                await self.wait_n_frames(2)

                saved_path = os.path.join(tmp_dir, f"{profile_name}.yaml")
                ok, message = save_teleop_profile(saved_path, TeleopProfile())
                self.assertTrue(ok, message)
                settings.set_string(last_profile_key, saved_path)
                profile_panel._rescan_profiles()
                await self.wait_n_frames(2)

                profile_names = [name for name, _ in profile_panel._profiles]
                self.assertIn(profile_name, profile_names, "Profile must be discovered before delete")
                index = profile_names.index(profile_name)
                profile_panel._profile_combo.model.get_item_value_model().set_value(index)

                profile_panel._on_delete_clicked()
                await self.wait_n_frames(2)
                self.assertIsNotNone(
                    profile_panel._confirm_dialog,
                    "Delete must open a confirm dialog before removing the file",
                )
                self.assertTrue(os.path.exists(saved_path), "Profile must remain on disk while the dialog is open")

                profile_panel._confirm_dialog._on_cancel()
                await self.wait_n_frames(2)
                self.assertTrue(os.path.exists(saved_path), "Cancelling the dialog must not delete the file")
                profile_names_after_cancel = [name for name, _ in profile_panel._profiles]
                self.assertIn(profile_name, profile_names_after_cancel, "Cancel must keep the profile listed")

                profile_panel._on_delete_clicked()
                await self.wait_n_frames(2)
                profile_panel._confirm_dialog._on_okay()
                await self.wait_n_frames(2)

                self.assertFalse(os.path.exists(saved_path), "Profile file should be removed from disk")
                self.assertEqual(
                    settings.get_as_string(last_profile_key) or "",
                    "",
                    "last_profile setting must be cleared when its target is deleted",
                )
                profile_names_after = [name for name, _ in profile_panel._profiles]
                self.assertNotIn(profile_name, profile_names_after, "Deleted profile must disappear from the dropdown")
            finally:
                settings.set_string(last_profile_key, "")
                if window is not None:
                    window._last_profile_path = os.path.join(tmp_dir, "last_profile.yaml")
                    window.destroy()

    async def test_filename_normalization_appends_yaml(self) -> None:
        """Save with a name that lacks .yaml extension must write `<name>.yaml`."""
        window = None

        with tempfile.TemporaryDirectory(prefix="teleop_ui_profile_") as tmp_dir:
            try:
                await self.menu_click_with_retry(MENU_PATH, window_name=WINDOW_TITLE)
                window = ui.Workspace.get_window(WINDOW_TITLE)
                self.assertIsNotNone(window, "Teleop window should exist after opening via the menu")
                window._last_profile_path = os.path.join(tmp_dir, "last_profile.yaml")
                window.visible = True
                window.focus()
                await self.wait_n_frames(5)

                profile_panel = window._teleop_profile_panel
                profile_panel._dir_field.model.set_value(tmp_dir)
                profile_panel._on_directory_changed()
                await self.wait_n_frames(2)

                profile_panel._save_name_field.model.set_value("noext_profile")
                profile_panel._on_save_confirm()
                await self.wait_n_frames(2)

                self.assertTrue(
                    os.path.isfile(os.path.join(tmp_dir, "noext_profile.yaml")),
                    "Save should append .yaml when the name has no extension",
                )

                profile_panel._save_name_field.model.set_value("explicit_profile.yml")
                profile_panel._on_save_confirm()
                await self.wait_n_frames(2)
                self.assertTrue(
                    os.path.isfile(os.path.join(tmp_dir, "explicit_profile.yml")),
                    "Save must keep .yml extension when explicitly provided",
                )
            finally:
                if window is not None:
                    window._last_profile_path = os.path.join(tmp_dir, "last_profile.yaml")
                    window.destroy()

    async def test_load_sparse_profile_clears_previous_controller_paths(self) -> None:
        """Loading a profile that omits IK/floating paths must not keep the previous prims."""
        window = None

        with tempfile.TemporaryDirectory(prefix="teleop_ui_profile_") as tmp_dir:
            try:
                await self.menu_click_with_retry(MENU_PATH, window_name=WINDOW_TITLE)
                window = ui.Workspace.get_window(WINDOW_TITLE)
                self.assertIsNotNone(window, "Teleop window should exist after opening via the menu")
                window._last_profile_path = os.path.join(tmp_dir, "last_profile.yaml")
                window.visible = True
                window.focus()
                await self.wait_n_frames(5)

                populated = TeleopProfile(
                    floating=BimanualControllerProfile(
                        right=ControllerSideProfile(
                            enabled=True,
                            settings={"prim_path": "/World/stale_floating"},
                        ),
                    ),
                    ik=BimanualControllerProfile(
                        left=ControllerSideProfile(
                            enabled=True,
                            settings={"robot_path": "/World/stale_ik_left"},
                        ),
                        right=ControllerSideProfile(
                            enabled=True,
                            settings={"robot_path": "/World/stale_ik_right"},
                        ),
                    ),
                )
                sparse = TeleopProfile(
                    floating=BimanualControllerProfile(
                        right=ControllerSideProfile(
                            enabled=True,
                            settings={"prim_path": "/World/teleop_xarm"},
                        ),
                    ),
                )
                populated_path = os.path.join(tmp_dir, "populated.yaml")
                sparse_path = os.path.join(tmp_dir, "sparse.yaml")
                ok, message = save_teleop_profile(populated_path, populated)
                self.assertTrue(ok, message)
                ok, message = save_teleop_profile(sparse_path, sparse)
                self.assertTrue(ok, message)

                profile_panel = window._teleop_profile_panel
                profile_panel._dir_field.model.set_value(tmp_dir)
                profile_panel._on_directory_changed()
                await self.wait_n_frames(2)

                profile_names = [name for name, _ in profile_panel._profiles]
                profile_panel._profile_combo.model.get_item_value_model().set_value(profile_names.index("populated"))
                profile_panel._on_load_clicked()
                await self.wait_n_frames(5)

                applied = window.collect_teleop_profile()
                self.assertEqual(applied.ik.left.settings.get("robot_path"), "/World/stale_ik_left")
                self.assertEqual(applied.ik.right.settings.get("robot_path"), "/World/stale_ik_right")
                self.assertEqual(applied.floating.right.settings.get("prim_path"), "/World/stale_floating")

                profile_panel._profile_combo.model.get_item_value_model().set_value(profile_names.index("sparse"))
                profile_panel._on_load_clicked()
                await self.wait_n_frames(5)

                applied = window.collect_teleop_profile()
                self.assertEqual(applied.ik.left.settings.get("robot_path", "").strip(), "")
                self.assertEqual(applied.ik.right.settings.get("robot_path", "").strip(), "")
                self.assertFalse(applied.ik.left.enabled)
                self.assertFalse(applied.ik.right.enabled)
                self.assertEqual(applied.floating.left.settings.get("prim_path", "").strip(), "")
                self.assertEqual(applied.floating.right.settings.get("prim_path"), "/World/teleop_xarm")
            finally:
                if window is not None:
                    window._last_profile_path = os.path.join(tmp_dir, "last_profile.yaml")
                    window.destroy()
