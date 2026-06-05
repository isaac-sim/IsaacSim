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

"""Grasp controller panel with per-side configure and enable workflow."""

from __future__ import annotations

from dataclasses import dataclass

import carb.settings
import omni.timeline
import omni.ui as ui
from isaacsim.gui.components.ui_utils import get_style
from isaacsim.replicator.teleop import (
    GraspControllerProfile,
    GraspSideProfile,
    TeleopManager,
)
from isaacsim.replicator.teleop.controllers import (
    GraspConfig,
    GraspController,
    get_builtin_grasp_configs,
    load_grasp_config,
    normalize_grasp_config_path,
)

from .ui_helpers import (
    CLR_DIM,
    CLR_GREEN,
    CLR_RED,
    CLR_YELLOW,
    INDENT,
    ROW_HEIGHT,
    ROW_SPACING,
    SECTION_SPACING,
    STATUS_HEIGHT,
    build_prim_path_row,
)
from .ui_helpers import set_status as _set_status_base

_PANEL_NAME = "Grasp Controller"
_LOG_NAMESPACE = "Grasp"


def set_status(
    label: ui.Label | None,
    text: str,
    color: int = CLR_DIM,
    emit_terminal: bool = False,
    side: str | None = None,
) -> None:
    """Set the status label text and color for this panel."""
    _set_status_base(label, text, color, source=_LOG_NAMESPACE, emit_terminal=emit_terminal, side=side)


_SETTINGS_PREFIX = "/persistent/exts/isaacsim.replicator.teleop/grasp"


@dataclass
class _SideState:
    prim_field: ui.StringField | None = None
    plus_btn: ui.Button | None = None
    del_btn: ui.Button | None = None
    prim_apply_btn: ui.Button | None = None
    config_combo: ui.ComboBox | None = None
    config_path_field: ui.StringField | None = None
    status_label: ui.Label | None = None
    enable_btn: ui.Button | None = None
    clear_btn: ui.Button | None = None
    loaded_config: GraspConfig | None = None
    is_configured: bool = False
    desired_enabled: bool = False


class GraspPanel:
    """Independent grasp panel with per-side configure and enable state."""

    def __init__(
        self,
        grasp_controller: GraspController,
        teleop_manager: TeleopManager,
        collapsed_states: dict,
    ) -> None:
        self._gc = grasp_controller
        self._tm = teleop_manager
        self._collapsed = collapsed_states
        self._settings = carb.settings.get_settings()

        self._sides: dict[str, _SideState] = {"left": _SideState(), "right": _SideState()}
        self._builtin_configs: list[tuple[str, str]] = []
        self._is_playing: bool = False
        self._timeline_sub = (
            omni.timeline.get_timeline_interface()
            .get_timeline_event_stream()
            .create_subscription_to_pop(self._on_timeline_event, name="GraspPanel_timeline")
        )

        for side in ("left", "right"):
            self._settings.set_default_string(f"{_SETTINGS_PREFIX}/{side}/prim_path", "")
            self._settings.set_default_string(f"{_SETTINGS_PREFIX}/{side}/config_path", "")

    def _ss(self, side: str) -> _SideState:
        return self._sides[side]

    def _save_setting(self, side: str, key: str, value: str) -> None:
        self._settings.set_string(f"{_SETTINGS_PREFIX}/{side}/{key}", value)

    def _load_setting(self, side: str, key: str) -> str:
        return self._settings.get_as_string(f"{_SETTINGS_PREFIX}/{side}/{key}") or ""

    def build(self) -> None:
        """Build the grasp controller panel UI."""
        self._builtin_configs = get_builtin_grasp_configs()

        frame = ui.CollapsableFrame(
            _PANEL_NAME, height=0, collapsed=self._collapsed.get(_PANEL_NAME, True), style=get_style()
        )
        with frame:
            frame.set_collapsed_changed_fn(lambda c, k=_PANEL_NAME: self._collapsed.__setitem__(k, c))
            with ui.VStack(spacing=0):
                self._build_side("left")
                self._build_side("right")

    def _build_side(self, side: str) -> None:
        ss = self._ss(side)
        display_names = [name for name, _path in self._builtin_configs]
        side_key = f"{_PANEL_NAME}:{side}"
        with ui.CollapsableFrame(
            side.capitalize(),
            height=0,
            collapsed=self._collapsed.get(side_key, True),
            style=get_style(),
        ) as side_frame:
            side_frame.set_collapsed_changed_fn(lambda c, k=side_key: self._collapsed.__setitem__(k, c))
            with ui.VStack(spacing=SECTION_SPACING):
                path_btns = {}
                ss.prim_field = build_prim_path_row(
                    "Prim Path:",
                    on_apply_clicked=lambda s=side: self._on_configure(s),
                    apply_tooltip="Validate and apply the grasp prim path and config",
                    buttons_out=path_btns,
                )
                ss.prim_apply_btn = path_btns.get("apply")
                ss.plus_btn = path_btns.get("plus")
                ss.del_btn = path_btns.get("delete")
                ss.prim_field.model.add_end_edit_fn(lambda _m, s=side: self._on_prim_field_edited(s))

                with ui.HStack(spacing=ROW_SPACING, height=ROW_HEIGHT):
                    ui.Spacer(width=INDENT)
                    ui.Label("Config:", width=40)
                    ss.config_combo = ui.ComboBox(
                        0,
                        *display_names,
                        width=120,
                        tooltip="Select a built-in grasp config or edit the path below",
                    )
                    ss.config_combo.model.add_item_changed_fn(
                        lambda model, item, s=side: self._on_combo_changed(s, model)
                    )
                    ss.config_path_field = ui.StringField(width=ui.Fraction(1))
                    if self._builtin_configs:
                        _display, default_path = self._builtin_configs[0]
                        ss.config_path_field.model.set_value(default_path)
                        self._save_setting(side, "config_path", default_path)
                    else:
                        ss.config_path_field.model.set_value("")
                    ss.config_path_field.model.add_end_edit_fn(lambda _m, s=side: self._on_config_path_edited(s))

                with ui.HStack(spacing=ROW_SPACING, height=ROW_HEIGHT):
                    ui.Spacer(width=INDENT)
                    ss.enable_btn = ui.Button(
                        "Enable",
                        width=60,
                        clicked_fn=lambda s=side: self._on_toggle_tracking(s),
                        tooltip=f"Enable/disable {side} trigger tracking",
                        enabled=False,
                    )
                    ss.clear_btn = ui.Button(
                        "Clear",
                        width=45,
                        clicked_fn=lambda s=side: self._on_clear(s),
                        tooltip=f"Destroy {side} grasp resources (paths preserved)",
                        enabled=False,
                    )

                with ui.HStack(spacing=ROW_SPACING, height=STATUS_HEIGHT):
                    ui.Spacer(width=INDENT)
                    ss.status_label = ui.Label("", style={"color": CLR_DIM}, word_wrap=True)

        self._sync_side_controls(side)

    def _sync_combo_to_path(self, side: str, config_path: str) -> None:
        ss = self._ss(side)
        if not ss.config_combo:
            return
        config_path = normalize_grasp_config_path(config_path)
        for i, (_name, path) in enumerate(self._builtin_configs):
            if path == config_path:
                ss.config_combo.model.get_item_value_model().set_value(i)
                return

    def _on_prim_field_edited(self, side: str) -> None:
        if self._is_playing:
            return
        ss = self._ss(side)
        path = ss.prim_field.model.get_value_as_string() if ss.prim_field else ""
        prev = self._load_setting(side, "prim_path")
        if path == prev:
            return
        self._save_setting(side, "prim_path", path)
        ss.is_configured = False
        ss.loaded_config = None
        self._gc.set_side_tracking_enabled(side, False)
        self._sync_manager_tracking()
        self._sync_side_controls(side)
        if path:
            set_status(ss.status_label, "Path edited - click Apply", CLR_DIM)
        else:
            set_status(ss.status_label, "", CLR_DIM)

    def _on_combo_changed(self, side: str, model: object) -> None:
        if self._is_playing:
            return
        ss = self._ss(side)
        idx = model.get_item_value_model().get_value_as_int()
        if 0 <= idx < len(self._builtin_configs):
            _display, path = self._builtin_configs[idx]
            if ss.config_path_field:
                ss.config_path_field.model.set_value(path)
            self._save_setting(side, "config_path", path)
            ss.loaded_config = None
            ss.is_configured = False
            self._gc.set_side_tracking_enabled(side, False)
            self._sync_manager_tracking()
            self._sync_side_controls(side)
            set_status(ss.status_label, "Config changed - click Apply", CLR_DIM)

    def _on_config_path_edited(self, side: str) -> None:
        if self._is_playing:
            return
        ss = self._ss(side)
        path = ss.config_path_field.model.get_value_as_string() if ss.config_path_field else ""
        path = normalize_grasp_config_path(path)
        if ss.config_path_field and ss.config_path_field.model.get_value_as_string() != path:
            ss.config_path_field.model.set_value(path)
        prev = normalize_grasp_config_path(self._load_setting(side, "config_path"))
        if path == prev:
            return
        self._save_setting(side, "config_path", path)
        ss.is_configured = False
        ss.loaded_config = None
        self._gc.set_side_tracking_enabled(side, False)
        self._sync_manager_tracking()
        self._sync_side_controls(side)
        if path:
            set_status(ss.status_label, "Config path edited - click Apply", CLR_DIM)
        else:
            set_status(ss.status_label, "No config selected", CLR_DIM)

    def _load_config(self, side: str, path: str, emit_terminal: bool = False) -> None:
        ss = self._ss(side)
        config, errors = load_grasp_config(path)
        if errors:
            ss.loaded_config = None
            set_status(ss.status_label, "; ".join(errors), CLR_RED, emit_terminal=emit_terminal, side=side)
            return

        ss.loaded_config = config
        n = len(config.joints) if config else 0
        name = config.name if config else "?"
        set_status(
            ss.status_label,
            f'Config "{name}" loaded ({n} joint{"s" if n != 1 else ""})',
            CLR_YELLOW,
            emit_terminal=emit_terminal,
            side=side,
        )

    def _on_timeline_event(self, event: object) -> None:
        if event.type == int(omni.timeline.TimelineEventType.PLAY):
            self._is_playing = True
            for side in ("left", "right"):
                self._sync_side_controls(side)
            self._refresh_side_statuses()
        elif event.type == int(omni.timeline.TimelineEventType.STOP):
            self._is_playing = False
            for side in ("left", "right"):
                self._sync_side_controls(side)
            self._refresh_side_statuses()

    def _on_configure(self, side: str) -> None:
        if self._is_playing:
            return
        ss = self._ss(side)
        if ss.is_configured:
            set_status(ss.status_label, "Configured - press Clear to reconfigure", CLR_YELLOW)
            self._sync_side_controls(side)
            return
        path = ss.prim_field.model.get_value_as_string() if ss.prim_field else ""
        self._save_setting(side, "prim_path", path)
        if not path:
            set_status(ss.status_label, "Set path first", CLR_DIM)
            return
        validation = self._gc.validate_prim(path)
        if not validation.is_valid:
            set_status(
                ss.status_label,
                "; ".join(validation.errors) if validation.errors else "Invalid path",
                CLR_RED,
                emit_terminal=True,
                side=side,
            )
            self._sync_side_controls(side)
            return

        config_path = ss.config_path_field.model.get_value_as_string() if ss.config_path_field else ""
        config_path = normalize_grasp_config_path(config_path)
        if ss.config_path_field and ss.config_path_field.model.get_value_as_string() != config_path:
            ss.config_path_field.model.set_value(config_path)
        self._save_setting(side, "config_path", config_path)
        if config_path and ss.loaded_config is None:
            self._load_config(side, config_path, emit_terminal=True)
        elif ss.loaded_config is None:
            set_status(ss.status_label, "No config selected", CLR_DIM)
            self._sync_side_controls(side)
            return
        if ss.loaded_config is None:
            ss.is_configured = False
            self._sync_side_controls(side)
            return

        ok = self._gc.configure(path, side, ss.loaded_config)
        if ok:
            ss.is_configured = True
            self._gc.set_side_tracking_enabled(side, False)
            set_status(ss.status_label, "Configured", CLR_YELLOW, emit_terminal=True, side=side)
        else:
            ss.is_configured = False
            set_status(ss.status_label, "Apply failed - check config/path", CLR_RED, emit_terminal=True, side=side)
        self._sync_manager_tracking()
        self._sync_side_controls(side)

    def _on_toggle_tracking(self, side: str) -> None:
        if self._is_playing:
            return
        ss = self._ss(side)
        if ss.desired_enabled:
            ss.desired_enabled = False
            self._gc.set_side_tracking_enabled(side, False)
            set_status(ss.status_label, "Disabled", CLR_YELLOW, emit_terminal=True, side=side)
        else:
            if not ss.is_configured:
                set_status(ss.status_label, "Apply first", CLR_YELLOW)
                self._sync_side_controls(side)
                return
            ss.desired_enabled = True
            self._gc.set_side_tracking_enabled(side, True)
            set_status(ss.status_label, "Standby", CLR_YELLOW, emit_terminal=True, side=side)
        self._sync_manager_tracking()
        self._sync_side_controls(side)

    def _on_clear(self, side: str) -> None:
        """Destroy grasp resources for a side (paths preserved)."""
        if self._is_playing:
            return
        ss = self._ss(side)
        self._gc.set_side_tracking_enabled(side, False)
        self._gc.remove(side)
        ss.is_configured = False
        ss.loaded_config = None
        ss.desired_enabled = False
        self._sync_manager_tracking()
        self._sync_side_controls(side)
        set_status(ss.status_label, "Cleared", CLR_DIM, emit_terminal=True, side=side)

    def _sync_manager_tracking(self) -> None:
        self._tm.set_grasp_tracking(self._gc.has_any_side_tracking_enabled)

    def collect_profile(self) -> GraspControllerProfile:
        """Collect the current grasp-controller state into a teleop profile section."""

        def _collect_side(side: str) -> GraspSideProfile:
            ss = self._ss(side)
            config_path = ss.config_path_field.model.get_value_as_string() if ss.config_path_field else ""
            config_path = normalize_grasp_config_path(config_path)
            prim_path = ss.prim_field.model.get_value_as_string() if ss.prim_field else ""
            return GraspSideProfile(
                enabled=ss.desired_enabled,
                prim_path=prim_path,
                config_path=config_path,
            )

        return GraspControllerProfile(
            left=_collect_side("left"),
            right=_collect_side("right"),
        )

    def apply_profile(self, profile: GraspControllerProfile, resolve_stage: bool) -> None:
        """Apply a grasp-controller teleop profile section."""
        for side, side_profile in (("left", profile.left), ("right", profile.right)):
            ss = self._ss(side)
            self._gc.set_side_tracking_enabled(side, False)
            self._gc.remove(side)
            ss.is_configured = False
            ss.loaded_config = None
            desired_enabled = bool(side_profile.enabled)
            ss.desired_enabled = False
            config_path = normalize_grasp_config_path(side_profile.config_path)

            if ss.prim_field:
                ss.prim_field.model.set_value(side_profile.prim_path)
            self._save_setting(side, "prim_path", side_profile.prim_path)

            if ss.config_path_field:
                ss.config_path_field.model.set_value(config_path)
            self._save_setting(side, "config_path", config_path)
            self._sync_combo_to_path(side, config_path)

            config, errors = load_grasp_config(config_path) if config_path else (None, [])

            status = ss.status_label
            if errors:
                set_status(status, "; ".join(errors), CLR_RED)
            elif config is not None:
                ss.loaded_config = config
                set_status(status, f'Loaded "{config.name or side}" grasp profile', CLR_YELLOW)
            else:
                set_status(status, "", CLR_DIM)

            if not side_profile.prim_path or ss.loaded_config is None:
                self._sync_side_controls(side)
            elif resolve_stage:
                self._on_configure(side)
                if desired_enabled and ss.is_configured:
                    self._on_toggle_tracking(side)
            else:
                ss.desired_enabled = desired_enabled
                set_status(status, "Profile loaded - stage resolution deferred", CLR_YELLOW)
                self._sync_side_controls(side)

        self._sync_manager_tracking()

    def _sync_side_controls(self, side: str) -> None:
        ss = self._ss(side)
        running = self._gc.is_side_tracking_enabled(side)
        path_editable = (not self._is_playing) and (not ss.is_configured)
        for widget in (ss.prim_field, ss.plus_btn, ss.del_btn, ss.prim_apply_btn):
            if widget:
                widget.enabled = path_editable
        for widget in (ss.config_combo, ss.config_path_field):
            if widget:
                widget.enabled = (not self._is_playing) and (not ss.is_configured)
        if ss.enable_btn:
            ss.enable_btn.enabled = (not self._is_playing) and (ss.is_configured or ss.desired_enabled)
            ss.enable_btn.text = "Disable" if ss.desired_enabled else "Enable"
        if ss.clear_btn:
            ss.clear_btn.enabled = (not self._is_playing) and ss.is_configured

    def _refresh_side_statuses(self) -> None:
        for side in ("left", "right"):
            ss = self._ss(side)
            if not ss.status_label:
                continue
            if ss.is_configured and self._is_playing and self._gc.is_side_tracking_enabled(side):
                set_status(ss.status_label, "Active", CLR_GREEN, emit_terminal=True, side=side)
            elif ss.is_configured:
                set_status(ss.status_label, "Standby", CLR_YELLOW, emit_terminal=True, side=side)

    def reset_ui(self) -> None:
        """Reset all UI widgets to idle state."""
        self._is_playing = False
        self._tm.set_grasp_tracking(False)
        for side in ("left", "right"):
            ss = self._ss(side)
            ss.is_configured = False
            ss.loaded_config = None
            ss.desired_enabled = False
            self._gc.set_side_tracking_enabled(side, False)
            self._sync_side_controls(side)
            if ss.status_label:
                set_status(ss.status_label, "", CLR_DIM)

    def on_stage_closed(self) -> None:
        """Clear stage-bound runtime state while preserving the configured profile in the UI."""
        self._is_playing = False
        self._tm.set_grasp_tracking(False)
        for side in ("left", "right"):
            ss = self._ss(side)
            ss.is_configured = False
            self._gc.set_side_tracking_enabled(side, False)
            self._sync_side_controls(side)
            if ss.status_label:
                prim_path = ss.prim_field.model.get_value_as_string().strip() if ss.prim_field else ""
                config_path = ss.config_path_field.model.get_value_as_string().strip() if ss.config_path_field else ""
                has_profile = bool(prim_path or ss.loaded_config is not None or config_path or ss.desired_enabled)
                if has_profile:
                    set_status(ss.status_label, "Configuration retained - apply after opening a stage.", CLR_YELLOW)
                else:
                    set_status(ss.status_label, "", CLR_DIM)

    def destroy(self) -> None:
        """Release the timeline subscription."""
        self._timeline_sub = None
