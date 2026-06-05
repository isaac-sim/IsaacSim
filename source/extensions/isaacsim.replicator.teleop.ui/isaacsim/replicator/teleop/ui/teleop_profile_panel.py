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

"""Teleop profile panel for saving, loading, and validating unified teleop profiles."""

from __future__ import annotations

import os
from collections.abc import Callable

import carb.settings
import omni.kit.window.popup_dialog
import omni.ui as ui
from isaacsim.gui.components.ui_utils import get_style
from isaacsim.replicator.teleop import (
    SEVERITY_ERROR,
    STAGE_STATE_READY,
    TeleopProfile,
    get_builtin_teleop_profiles_dir,
    get_last_teleop_profile_path,
    load_teleop_profile,
    resolve_teleop_profile,
    save_teleop_profile,
    scan_teleop_profiles,
)
from omni.kit.window.filepicker import FilePickerDialog

from .ui_helpers import (
    CLR_DIM,
    CLR_GREEN,
    CLR_RED,
    CLR_YELLOW,
    GLYPHS,
    INDENT,
    ROW_HEIGHT,
    ROW_SPACING,
    SECTION_SPACING,
    STATUS_HEIGHT,
)
from .ui_helpers import set_status as _set_status_base

_PANEL_NAME = "Profiles"
_LOG_NAMESPACE = "Profiles"
_SETTINGS_PREFIX = "/persistent/exts/isaacsim.replicator.teleop/teleop_profiles"


def set_status(label: ui.Label | None, text: str, color: int = CLR_DIM, emit_terminal: bool = False) -> None:
    """Set the status label text and color for this panel."""
    _set_status_base(label, text, color, source=_LOG_NAMESPACE, emit_terminal=emit_terminal)


class TeleopProfilePanel:
    """Save and load unified teleop profiles."""

    def __init__(
        self,
        collect_profile: Callable[[], TeleopProfile],
        apply_profile: Callable[[TeleopProfile], tuple[bool, str]],
        collapsed_states: dict,
    ) -> None:
        self._collect_profile = collect_profile
        self._apply_profile = apply_profile
        self._collapsed = collapsed_states
        self._settings = carb.settings.get_settings()

        self._profiles: list[tuple[str, str]] = []
        self._dir_field: ui.StringField | None = None
        self._profile_combo: ui.ComboBox | None = None
        self._status_label: ui.Label | None = None
        self._save_row: ui.HStack | None = None
        self._save_name_field: ui.StringField | None = None
        self._folder_picker: FilePickerDialog | None = None
        self._confirm_dialog: omni.kit.window.popup_dialog.MessageDialog | None = None
        self._pending_overwrite_path: str = ""
        self._pending_delete_path: str = ""

        self._settings.set_default_string(f"{_SETTINGS_PREFIX}/directory", "")
        self._settings.set_default_string(f"{_SETTINGS_PREFIX}/last_profile", "")

    def build(self) -> None:
        """Build the profile panel UI."""
        frame = ui.CollapsableFrame(
            _PANEL_NAME,
            height=0,
            collapsed=self._collapsed.get(_PANEL_NAME, False),
            style=get_style(),
        )
        with frame:
            frame.set_collapsed_changed_fn(lambda c, k=_PANEL_NAME: self._collapsed.__setitem__(k, c))
            with ui.VStack(spacing=SECTION_SPACING):
                with ui.HStack(spacing=ROW_SPACING, height=ROW_HEIGHT):
                    ui.Spacer(width=INDENT)
                    ui.Label("Dir:", width=25, tooltip="Directory containing teleop profiles")
                    self._dir_field = ui.StringField(
                        width=ui.Fraction(1),
                        tooltip="Working directory for unified teleop profile YAML files",
                    )
                    saved_dir = self._settings.get_as_string(f"{_SETTINGS_PREFIX}/directory") or ""
                    if not saved_dir:
                        saved_dir = get_builtin_teleop_profiles_dir()
                    self._dir_field.model.set_value(saved_dir)
                    self._dir_field.model.add_end_edit_fn(lambda _m: self._on_directory_changed())
                    ui.Button(
                        f"{GLYPHS['open_folder']}",
                        width=22,
                        clicked_fn=self._on_open_dir,
                        tooltip="Open profile directory in file browser",
                    )

                with ui.HStack(spacing=ROW_SPACING, height=ROW_HEIGHT):
                    ui.Spacer(width=INDENT)
                    self._profile_combo = ui.ComboBox(
                        0,
                        width=ui.Fraction(1),
                        tooltip="Select a unified teleop profile to load",
                    )
                    ui.Button(
                        "Load",
                        width=45,
                        clicked_fn=self._on_load_clicked,
                        tooltip="Load the selected teleop profile into all panels and apply it to the current stage",
                    )
                    ui.Button(
                        "Save",
                        width=40,
                        clicked_fn=self._on_save_clicked,
                        tooltip="Save the current teleop state as a unified profile",
                    )
                    ui.Button(
                        "Validate",
                        width=55,
                        clicked_fn=self._on_validate_clicked,
                        tooltip="Check all panel settings against the current stage",
                    )
                    ui.Button(
                        f"{GLYPHS['delete']}",
                        width=22,
                        clicked_fn=self._on_delete_clicked,
                        tooltip="Delete the selected teleop profile from disk",
                    )

                self._save_row = ui.HStack(spacing=ROW_SPACING, height=ROW_HEIGHT, visible=False)
                with self._save_row:
                    ui.Spacer(width=INDENT)
                    ui.Label("Name:", width=40)
                    self._save_name_field = ui.StringField(
                        width=ui.Fraction(1),
                        tooltip="Filename for the new teleop profile (without .yaml extension)",
                    )
                    ui.Button(
                        "Confirm",
                        width=55,
                        clicked_fn=self._on_save_confirm,
                        tooltip="Write the current teleop profile to disk",
                    )

                with ui.HStack(spacing=ROW_SPACING, height=STATUS_HEIGHT):
                    ui.Spacer(width=INDENT)
                    self._status_label = ui.Label("", style={"color": CLR_DIM}, word_wrap=True)

        self._rescan_profiles()

    def _rescan_profiles(self) -> None:
        if self._dir_field is None or self._profile_combo is None:
            return

        directory = self._dir_field.model.get_value_as_string()
        self._profiles = scan_teleop_profiles(directory) if directory else []
        last_profile_path = get_last_teleop_profile_path()
        if last_profile_path and os.path.isfile(last_profile_path):
            known_paths = {path for _, path in self._profiles}
            if last_profile_path not in known_paths:
                self._profiles.insert(0, ("last_profile", last_profile_path))

        model = self._profile_combo.model
        for child in model.get_item_children():
            model.remove_item(child)
        for display_name, _ in self._profiles:
            model.append_child_item(None, ui.SimpleStringModel(display_name))
        if self._profiles:
            model.get_item_value_model().set_value(0)

        last = self._settings.get_as_string(f"{_SETTINGS_PREFIX}/last_profile") or ""
        if last:
            for index, (_, path) in enumerate(self._profiles):
                if path == last:
                    model.get_item_value_model().set_value(index)
                    break

    def _on_directory_changed(self) -> None:
        directory = self._dir_field.model.get_value_as_string() if self._dir_field else ""
        self._settings.set_string(f"{_SETTINGS_PREFIX}/directory", directory)
        self._rescan_profiles()

    def _on_open_dir(self) -> None:
        def on_selected(_filename: str, dirname: str) -> None:
            if self._dir_field and dirname:
                path = dirname.rstrip("/")
                self._dir_field.model.set_value(path)
                self._on_directory_changed()
            if self._folder_picker:
                self._folder_picker.hide()

        def on_canceled(_a: str, _b: str) -> None:
            if self._folder_picker:
                self._folder_picker.hide()

        self._folder_picker = FilePickerDialog(
            "Select Profile Directory",
            allow_multi_selection=False,
            apply_button_label="Select",
            click_apply_handler=on_selected,
            click_cancel_handler=on_canceled,
        )

    def remember_last_profile(self, filepath: str) -> None:
        """Store the most recent profile path and refresh the profile list."""
        if not filepath:
            return
        self._settings.set_string(f"{_SETTINGS_PREFIX}/last_profile", filepath)
        self._rescan_profiles()

    def _on_load_clicked(self) -> None:
        if not self._profiles or self._profile_combo is None:
            set_status(self._status_label, "No profiles available", CLR_YELLOW)
            return

        index = self._profile_combo.model.get_item_value_model().get_value_as_int()
        if index < 0 or index >= len(self._profiles):
            return

        name, filepath = self._profiles[index]
        self._settings.set_string(f"{_SETTINGS_PREFIX}/last_profile", filepath)
        profile, errors = load_teleop_profile(filepath)
        if profile is None:
            set_status(self._status_label, "; ".join(errors), CLR_RED, emit_terminal=True)
            return

        ok, message = self._apply_profile(profile)
        if errors:
            message = f"{message} ({'; '.join(errors)})" if message else "; ".join(errors)
        color = CLR_GREEN if ok and not errors else CLR_YELLOW if ok else CLR_RED
        display = f"{message} '{name}'" if message else f"Loaded '{name}'"
        set_status(self._status_label, display, color, emit_terminal=True)

    def _on_delete_clicked(self) -> None:
        if not self._profiles or self._profile_combo is None:
            set_status(self._status_label, "No profiles to delete", CLR_YELLOW)
            return

        index = self._profile_combo.model.get_item_value_model().get_value_as_int()
        if index < 0 or index >= len(self._profiles):
            return

        name, filepath = self._profiles[index]
        self._pending_delete_path = filepath
        self._show_confirm_dialog(
            title="Delete profile",
            message=f"Permanently delete '{name}'?\n\n{filepath}",
            ok_label="Delete",
            cancel_label="Cancel",
            on_ok=lambda: self._perform_delete(filepath, name),
            on_cancel=self._on_delete_cancel,
        )

    def _perform_delete(self, filepath: str, display_name: str) -> None:
        """Perform the actual deletion. Public-test-friendly entry point."""
        self._pending_delete_path = ""
        try:
            os.remove(filepath)
        except Exception as exc:
            set_status(self._status_label, f"Delete failed: {exc}", CLR_RED, emit_terminal=True)
            return
        set_status(self._status_label, f"Deleted '{display_name}'", CLR_DIM, emit_terminal=True)
        last = self._settings.get_as_string(f"{_SETTINGS_PREFIX}/last_profile") or ""
        if last == filepath:
            self._settings.set_string(f"{_SETTINGS_PREFIX}/last_profile", "")
        self._rescan_profiles()

    def _on_delete_cancel(self) -> None:
        self._pending_delete_path = ""
        set_status(self._status_label, "Delete cancelled", CLR_DIM)

    def _on_save_clicked(self) -> None:
        if self._save_row:
            self._save_row.visible = not self._save_row.visible
            self._pending_overwrite_path = ""

    def _on_save_confirm(self) -> None:
        if self._save_name_field is None or self._dir_field is None:
            return

        filename = self._save_name_field.model.get_value_as_string().strip()
        if not filename:
            set_status(self._status_label, "Enter a filename", CLR_YELLOW)
            return
        if not filename.endswith((".yaml", ".yml")):
            filename += ".yaml"

        directory = self._dir_field.model.get_value_as_string()
        if not directory:
            set_status(self._status_label, "Set a working directory first", CLR_YELLOW)
            return

        filepath = os.path.join(directory, filename)

        if os.path.exists(filepath):
            self._pending_overwrite_path = filepath
            self._show_confirm_dialog(
                title="Overwrite profile",
                message=f"'{filename}' already exists.\n\nReplace the existing profile on disk?",
                ok_label="Overwrite",
                cancel_label="Cancel",
                on_ok=lambda: self._perform_save(filepath),
                on_cancel=self._on_save_cancel,
            )
            return

        self._perform_save(filepath)

    def _perform_save(self, filepath: str) -> None:
        """Perform the actual save (with or without overwrite). Public-test-friendly entry point."""
        self._pending_overwrite_path = ""
        profile = self._collect_profile()
        ok, message = save_teleop_profile(filepath, profile)
        if ok:
            self._settings.set_string(f"{_SETTINGS_PREFIX}/last_profile", filepath)
            set_status(self._status_label, message, CLR_GREEN, emit_terminal=True)
            if self._save_row is not None:
                self._save_row.visible = False
            self._rescan_profiles()
        else:
            set_status(self._status_label, message, CLR_RED, emit_terminal=True)

    def _on_save_cancel(self) -> None:
        self._pending_overwrite_path = ""
        set_status(self._status_label, "Save cancelled", CLR_DIM)

    def destroy(self) -> None:
        """Tear down panel resources, including any open confirmation dialog.

        Called from :meth:`TeleopWindow.destroy` so that closing the window with an
        unanswered Save-overwrite or Delete dialog does not leave a modal pointing at
        callbacks bound to a destroyed panel.
        """
        self._dismiss_confirm_dialog()
        self._pending_overwrite_path = ""
        self._pending_delete_path = ""

    def _dismiss_confirm_dialog(self) -> None:
        """Hide and forget any currently displayed confirmation dialog."""
        dialog = self._confirm_dialog
        if dialog is None:
            return
        self._confirm_dialog = None
        try:
            dialog.hide()
        except Exception:
            pass

    def _show_confirm_dialog(
        self,
        *,
        title: str,
        message: str,
        ok_label: str,
        cancel_label: str,
        on_ok: Callable[[], None],
        on_cancel: Callable[[], None] | None = None,
    ) -> None:
        """Show a modal yes/cancel confirm dialog and route the answer to ``on_ok`` / ``on_cancel``."""
        self._dismiss_confirm_dialog()

        def _handle_ok(dialog) -> None:
            try:
                dialog.hide()
            finally:
                self._confirm_dialog = None
            try:
                on_ok()
            except Exception as exc:  # noqa: BLE001
                set_status(self._status_label, f"Action failed: {exc}", CLR_RED, emit_terminal=True)

        def _handle_cancel(dialog) -> None:
            try:
                dialog.hide()
            finally:
                self._confirm_dialog = None
            if on_cancel is not None:
                try:
                    on_cancel()
                except Exception as exc:  # noqa: BLE001
                    set_status(self._status_label, f"Cancel failed: {exc}", CLR_RED, emit_terminal=True)

        self._confirm_dialog = omni.kit.window.popup_dialog.MessageDialog(
            title=title,
            message=message,
            ok_label=ok_label,
            cancel_label=cancel_label,
            ok_handler=_handle_ok,
            cancel_handler=_handle_cancel,
        )
        self._confirm_dialog.show()

    def _on_validate_clicked(self) -> None:
        profile = self._collect_profile()
        report = resolve_teleop_profile(profile)
        if report.stage_state != STAGE_STATE_READY:
            set_status(self._status_label, report.stage_message, CLR_YELLOW, emit_terminal=True)
            return
        if report.error_count == 0 and report.warning_count == 0:
            set_status(self._status_label, "Ready", CLR_GREEN, emit_terminal=True)
        else:
            summary = f"{report.error_count} error(s), {report.warning_count} warning(s)"
            set_status(
                self._status_label,
                summary,
                CLR_RED if report.error_count else CLR_YELLOW,
                emit_terminal=True,
            )
            for issue in report.issues:
                tag = "ERR" if issue.severity == SEVERITY_ERROR else "WARN"
                print(f"[Teleop][Profiles][{tag}] {issue.source}: {issue.message}")
