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

"""Locomotion panel - kinematic base movement via VR thumbstick input.

Controls:
- Base prim path for the kinematic base/root prim to move
- Slide step for left-thumbstick translation and right-button vertical motion
- Turn step for right-thumbstick yaw
- Enable / Disable toggle (becomes active on Play)
"""

from __future__ import annotations

import carb.settings
import omni.timeline
import omni.ui as ui
from isaacsim.gui.components.ui_utils import get_style
from isaacsim.replicator.teleop import LocomotionProfile, TeleopManager
from isaacsim.replicator.teleop.controllers import LocomotionController

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

_PANEL_NAME = "Locomotion"
_LOG_NAMESPACE = "Locomotion"


def set_status(label: ui.Label | None, text: str, color: int = CLR_DIM, emit_terminal: bool = False) -> None:
    """Set the status label text and color for this panel."""
    _set_status_base(label, text, color, source=_LOG_NAMESPACE, emit_terminal=emit_terminal)


_SETTINGS_PREFIX = "/persistent/exts/isaacsim.replicator.teleop/locomotion"


class LocomotionPanel:
    """Panel for kinematic slide locomotion via VR controller input."""

    def __init__(
        self,
        locomotion_controller: LocomotionController,
        teleop_manager: TeleopManager,
        collapsed_states: dict,
    ) -> None:
        self._loco = locomotion_controller
        self._tm = teleop_manager
        self._collapsed = collapsed_states
        self._settings = carb.settings.get_settings()

        self._path_field: ui.StringField | None = None
        self._plus_btn: ui.Button | None = None
        self._del_btn: ui.Button | None = None
        self._configure_btn: ui.Button | None = None
        self._enable_btn: ui.Button | None = None
        self._clear_btn: ui.Button | None = None
        self._status_label: ui.Label | None = None
        self._linear_step_slider: ui.FloatSlider | None = None
        self._angular_step_slider: ui.FloatSlider | None = None
        self._configured: bool = False
        self._desired_enabled: bool = False
        self._is_playing: bool = False
        self._timeline_sub = (
            omni.timeline.get_timeline_interface()
            .get_timeline_event_stream()
            .create_subscription_to_pop(self._on_timeline_event, name="LocomotionPanel_timeline")
        )

        self._settings.set_default_string(f"{_SETTINGS_PREFIX}/path", "")

    def _save(self, key: str, value: object) -> None:
        path = f"{_SETTINGS_PREFIX}/{key}"
        if isinstance(value, str):
            self._settings.set_string(path, value)

    def _load_str(self, key: str) -> str:
        return self._settings.get_as_string(f"{_SETTINGS_PREFIX}/{key}") or ""

    def build(self) -> None:
        """Build the locomotion panel UI."""
        base_path_tooltip = (
            "Base prim moved kinematically by locomotion. Use the robot root or mobile base that should carry "
            "the attached teleop content."
        )
        linear_step_tooltip = (
            "Slide distance per app update at full input. Affects forward/backward and left/right slide on the "
            "left thumbstick, plus vertical motion from the right face buttons (`A` = down, `B` = up on "
            "Meta-style controllers)."
        )
        angular_step_tooltip = "Turn angle per app update at full right-thumbstick left/right yaw input."
        enable_tooltip = (
            "Enable or disable slide locomotion for the next Play session. During Play, enabled locomotion reads "
            "the left thumbstick for slide motion, the right thumbstick for turn, and the right face buttons for "
            "vertical movement. Press the left primary face button (`X` on Meta-style controllers) to toggle "
            "Carry Tracking Space. When enabled, locomotion also moves the Session panel's Tracking Space prim "
            "with the base, including turn rotation around the base pivot; the current toggle state is printed to "
            "the terminal."
        )
        clear_tooltip = "Clear the configured locomotion state and keep the saved base path."

        frame = ui.CollapsableFrame(
            _PANEL_NAME,
            height=0,
            collapsed=self._collapsed.get(_PANEL_NAME, True),
            style=get_style(),
        )
        with frame:
            frame.set_collapsed_changed_fn(lambda c, k=_PANEL_NAME: self._collapsed.__setitem__(k, c))
            with ui.VStack(spacing=SECTION_SPACING):
                path_btns = {}
                self._path_field = build_prim_path_row(
                    "Prim Path:",
                    tooltip=base_path_tooltip,
                    on_apply_clicked=self._on_configure,
                    apply_tooltip="Validate and apply the locomotion base prim path",
                    buttons_out=path_btns,
                )
                self._path_field.tooltip = base_path_tooltip
                self._configure_btn = path_btns.get("apply")
                self._plus_btn = path_btns.get("plus")
                self._del_btn = path_btns.get("delete")

                with ui.HStack(spacing=ROW_SPACING, height=ROW_HEIGHT):
                    ui.Spacer(width=INDENT)
                    ui.Label("Slide Step:", width=85, tooltip=linear_step_tooltip)
                    self._linear_step_slider = ui.FloatSlider(
                        min=0.0,
                        max=0.1,
                        step=0.001,
                        width=ui.Fraction(1),
                        tooltip=linear_step_tooltip,
                    )
                    self._linear_step_slider.model.set_value(self._loco.linear_step)
                    self._linear_step_slider.model.add_value_changed_fn(
                        lambda m: self._loco.set_linear_step(m.get_value_as_float())
                    )

                with ui.HStack(spacing=ROW_SPACING, height=ROW_HEIGHT):
                    ui.Spacer(width=INDENT)
                    ui.Label("Turn Step:", width=85, tooltip=angular_step_tooltip)
                    self._angular_step_slider = ui.FloatSlider(
                        min=0.0,
                        max=0.1,
                        step=0.001,
                        width=ui.Fraction(1),
                        tooltip=angular_step_tooltip,
                    )
                    self._angular_step_slider.model.set_value(self._loco.angular_step)
                    self._angular_step_slider.model.add_value_changed_fn(
                        lambda m: self._loco.set_angular_step(m.get_value_as_float())
                    )

                with ui.HStack(spacing=ROW_SPACING, height=ROW_HEIGHT):
                    ui.Spacer(width=INDENT)
                    self._enable_btn = ui.Button(
                        "Enable",
                        width=55,
                        clicked_fn=self._on_toggle,
                        tooltip=enable_tooltip,
                        enabled=False,
                    )
                    self._clear_btn = ui.Button(
                        "Clear",
                        width=45,
                        clicked_fn=self._on_clear,
                        tooltip=clear_tooltip,
                        enabled=False,
                    )
                with ui.HStack(spacing=ROW_SPACING, height=STATUS_HEIGHT):
                    ui.Spacer(width=INDENT)
                    self._status_label = ui.Label("", style={"color": CLR_DIM}, word_wrap=True)

    # ------------------------------------------------------------------
    # Timeline-driven UI locking
    # ------------------------------------------------------------------

    def _on_timeline_event(self, event: object) -> None:
        if event.type == int(omni.timeline.TimelineEventType.PLAY):
            self._is_playing = True
            self._sync_controls()
            if self._loco.is_running:
                self._tm.set_locomotion_tracking(True)
                set_status(self._status_label, "Active", CLR_GREEN, emit_terminal=True)
        elif event.type == int(omni.timeline.TimelineEventType.STOP):
            self._is_playing = False
            self._sync_controls()
            if self._configured:
                set_status(self._status_label, "Standby", CLR_YELLOW, emit_terminal=True)
            else:
                set_status(self._status_label, "", CLR_DIM)

    # ------------------------------------------------------------------
    # Callbacks
    # ------------------------------------------------------------------

    def _on_configure(self) -> None:
        """Validate the base path and transitions to configured state."""
        if self._is_playing:
            return
        path = self._path_field.model.get_value_as_string() if self._path_field else ""
        self._save("path", path)

        if self._configured:
            set_status(self._status_label, "Configured - press Clear to reconfigure", CLR_YELLOW)
            self._sync_controls()
            return

        if not path:
            self._configured = False
            set_status(self._status_label, "Set path first", CLR_DIM)
            self._sync_controls()
            return

        self._loco.set_prim_path(path)
        ok, msg = self._loco.validate()
        if ok:
            self._configured = True
            set_status(self._status_label, f"Configured - {msg}", CLR_YELLOW, emit_terminal=True)
        else:
            self._configured = False
            set_status(self._status_label, msg, CLR_RED, emit_terminal=True)
        self._sync_controls()

    def _on_toggle(self) -> None:
        """Enable or disables locomotion for the next Play session."""
        if self._is_playing:
            return
        if self._desired_enabled:
            self._desired_enabled = False
            self._loco.disable()
            self._tm.set_locomotion_tracking(False)
            set_status(self._status_label, "Disabled", CLR_YELLOW, emit_terminal=True)
        else:
            if not self._configured:
                set_status(self._status_label, "Apply first", CLR_YELLOW)
                self._sync_controls()
                return
            path = self._path_field.model.get_value_as_string() if self._path_field else ""
            self._loco.set_prim_path(path)
            ok, msg = self._loco.enable()
            if ok:
                self._desired_enabled = True
                set_status(self._status_label, "Standby", CLR_YELLOW, emit_terminal=True)
            else:
                self._desired_enabled = False
                self._configured = False
                set_status(self._status_label, msg, CLR_RED, emit_terminal=True)

        self._sync_controls()

    def _on_clear(self) -> None:
        """Clear the configured locomotion state (path is preserved)."""
        if self._is_playing:
            return
        self._loco.disable()
        self._tm.set_locomotion_tracking(False)
        self._configured = False
        self._desired_enabled = False
        self._sync_controls()
        set_status(self._status_label, "Cleared", CLR_DIM, emit_terminal=True)

    def _sync_controls(self) -> None:
        path_editable = (not self._is_playing) and (not self._configured)
        enabled_for_play = self._loco.is_running or self._tm.is_locomotion_tracking
        for widget in (self._path_field, self._plus_btn, self._del_btn, self._configure_btn):
            if widget:
                widget.enabled = path_editable
        if self._enable_btn:
            self._enable_btn.enabled = (not self._is_playing) and (self._configured or self._desired_enabled)
            self._enable_btn.text = "Disable" if self._desired_enabled else "Enable"
        if self._clear_btn:
            self._clear_btn.enabled = (not self._is_playing) and self._configured

    def collect_profile(self) -> LocomotionProfile:
        """Collect the current locomotion-controller state into a teleop profile section."""
        path = self._path_field.model.get_value_as_string() if self._path_field else ""
        settings = {
            "prim_path": path,
            "linear_step": self._loco.linear_step,
            "angular_step": self._loco.angular_step,
        }
        return LocomotionProfile(
            enabled=self._desired_enabled,
            settings=settings,
        )

    def apply_profile(self, profile: LocomotionProfile, resolve_stage: bool) -> None:
        """Apply a locomotion-controller teleop profile section."""
        self._loco.disable()
        self._tm.set_locomotion_tracking(False)
        self._configured = False
        desired_enabled = bool(profile.enabled)
        self._desired_enabled = False

        path = str(profile.settings.get("prim_path", ""))
        if self._path_field:
            self._path_field.model.set_value(path)
        self._save("path", path)
        self._loco.set_prim_path(path)

        linear_step = float(profile.settings.get("linear_step", self._loco.linear_step))
        angular_step = float(profile.settings.get("angular_step", self._loco.angular_step))
        if self._linear_step_slider:
            self._linear_step_slider.model.set_value(linear_step)
        else:
            self._loco.set_linear_step(linear_step)
        if self._angular_step_slider:
            self._angular_step_slider.model.set_value(angular_step)
        else:
            self._loco.set_angular_step(angular_step)

        if not path:
            self._sync_controls()
            if self._status_label:
                set_status(self._status_label, "", CLR_DIM)
        elif resolve_stage:
            self._on_configure()
            if desired_enabled and self._configured:
                self._on_toggle()
        else:
            self._desired_enabled = desired_enabled
            self._sync_controls()
            if self._status_label:
                set_status(self._status_label, "Profile loaded - stage resolution deferred", CLR_YELLOW)

    def reset_ui(self) -> None:
        """Reset all UI widgets to idle state (e.g. after stage close)."""
        self._is_playing = False
        self._configured = False
        self._desired_enabled = False
        self._sync_controls()
        if self._status_label:
            set_status(self._status_label, "", CLR_DIM)

    def on_stage_closed(self) -> None:
        """Clear stage-bound runtime state while preserving the configured profile in the UI."""
        self._is_playing = False
        self._configured = False
        self._sync_controls()
        if self._status_label:
            path = self._path_field.model.get_value_as_string().strip() if self._path_field else ""
            if path or self._desired_enabled:
                set_status(self._status_label, "Configuration retained - apply after opening a stage.", CLR_YELLOW)
            else:
                set_status(self._status_label, "", CLR_DIM)

    def destroy(self) -> None:
        """Release the timeline subscription."""
        self._timeline_sub = None
