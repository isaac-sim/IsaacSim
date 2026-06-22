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

"""Floating controller panel for rigid-body velocity teleop.

Provides per-side (left/right) controls:
- Prim path (auto-validates on change)
- Target local rotation offsets
- PD gain tuning (editable during Play for live tuning)
- Enable / Disable button

During Play the prim-path controls are locked, while target rotation offsets and PD gains stay live-editable. The controller
activates automatically on Play and deactivates on Stop.
"""

from __future__ import annotations

import carb.settings
import omni.timeline
import omni.ui as ui
from isaacsim.gui.components.ui_utils import get_style
from isaacsim.replicator.teleop import (
    BimanualControllerProfile,
    ControllerSideProfile,
    FloatingRigidBodyController,
    TeleopManager,
)
from isaacsim.replicator.teleop.controllers._utils import (
    DEFAULT_ROTATION_OFFSET_DEG,
    ROTATION_OFFSET_DEGREES,
    ROTATION_OFFSET_LABELS,
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

_PANEL_NAME = "Floating Controller"
_LOG_NAMESPACE = "Floating"


def set_status(
    label: ui.Label | None,
    text: str,
    color: int = CLR_DIM,
    emit_terminal: bool = False,
    side: str | None = None,
) -> None:
    """Set the status label text and color for this panel.

    Args:
        label: Label widget to update.
        text: Status text to display.
        color: Label text color.
        emit_terminal: Whether to print the status change to the terminal.
        side: Optional controller side for terminal log tagging.
    """
    _set_status_base(label, text, color, source=_LOG_NAMESPACE, emit_terminal=emit_terminal, side=side)


_SETTINGS_PREFIX = "/persistent/exts/isaacsim.replicator.teleop/floating"


class FloatingPanel:
    """Floating controller panel with per-side path, gains, and enable/disable.

    Args:
        floating_controller: Controller backing floating rigid-body tracking.
        teleop_manager: Shared teleop manager for controller assignment state.
        collapsed_states: Mutable panel collapsed-state cache.
    """

    _ROTATION_OFFSET_VALUES = list(ROTATION_OFFSET_DEGREES)
    _ROTATION_OFFSET_LABELS = list(ROTATION_OFFSET_LABELS)

    def __init__(
        self, floating_controller: FloatingRigidBodyController, teleop_manager: TeleopManager, collapsed_states: dict
    ) -> None:
        self._fc = floating_controller
        self._tm = teleop_manager
        self._collapsed = collapsed_states
        self._settings = carb.settings.get_settings()

        self._widgets: dict[str, dict] = {"left": {}, "right": {}}
        self._configured: dict[str, bool] = {"left": False, "right": False}
        self._desired_enabled: dict[str, bool] = {"left": False, "right": False}
        self._is_playing: bool = False
        self._timeline_sub = (
            omni.timeline.get_timeline_interface()
            .get_timeline_event_stream()
            .create_subscription_to_pop(self._on_timeline_event, name="FloatingPanel_timeline")
        )

        # Register persistent setting defaults
        for side in ("left", "right"):
            self._settings.set_default_string(f"{_SETTINGS_PREFIX}/{side}/path", "")
            self._settings.set_default_int(f"{_SETTINGS_PREFIX}/{side}/target_rot_x_deg", DEFAULT_ROTATION_OFFSET_DEG)
            self._settings.set_default_int(f"{_SETTINGS_PREFIX}/{side}/target_rot_y_deg", DEFAULT_ROTATION_OFFSET_DEG)
            self._settings.set_default_int(f"{_SETTINGS_PREFIX}/{side}/target_rot_z_deg", DEFAULT_ROTATION_OFFSET_DEG)

    # ------------------------------------------------------------------
    # Persistent settings helpers
    # ------------------------------------------------------------------

    def _save(self, side: str, key: str, value: object) -> None:
        path = f"{_SETTINGS_PREFIX}/{side}/{key}"
        if isinstance(value, str):
            self._settings.set_string(path, value)
        elif isinstance(value, (int, bool)):
            self._settings.set_int(path, int(value))
        elif isinstance(value, float):
            self._settings.set_float(path, value)

    def _load_str(self, side: str, key: str) -> str:
        return self._settings.get_as_string(f"{_SETTINGS_PREFIX}/{side}/{key}") or ""

    def _load_int(self, side: str, key: str) -> int:
        return int(self._settings.get_as_int(f"{_SETTINGS_PREFIX}/{side}/{key}"))

    # ------------------------------------------------------------------
    # Build
    # ------------------------------------------------------------------

    def build(self) -> None:
        """Build the floating controller panel UI."""
        frame = ui.CollapsableFrame(
            _PANEL_NAME, height=0, collapsed=self._collapsed.get(_PANEL_NAME, True), style=get_style()
        )
        with frame:
            frame.set_collapsed_changed_fn(lambda c, k=_PANEL_NAME: self._collapsed.__setitem__(k, c))
            with ui.VStack(spacing=0):
                self._build_side("left")
                self._build_side("right")

    def _build_side(self, side: str) -> None:
        w = self._widgets[side]
        side_key = f"{_PANEL_NAME}:{side}"
        with ui.CollapsableFrame(
            f"{side.capitalize()}",
            height=0,
            collapsed=self._collapsed.get(side_key, True),
            style=get_style(),
        ) as side_frame:
            side_frame.set_collapsed_changed_fn(lambda c, k=side_key: self._collapsed.__setitem__(k, c))
            with ui.VStack(spacing=SECTION_SPACING):
                path_btns = {}
                w["path"] = build_prim_path_row(
                    "Prim Path:",
                    on_apply_clicked=lambda s=side: self._on_path_changed(s),
                    apply_tooltip="Validate and apply this floating rigid-body prim path",
                    buttons_out=path_btns,
                )
                w["configure_btn"] = path_btns.get("apply")
                w["plus_btn"] = path_btns.get("plus")
                w["del_btn"] = path_btns.get("delete")

                with ui.HStack(spacing=ROW_SPACING, height=ROW_HEIGHT):
                    ui.Spacer(width=INDENT)
                    ui.Label(
                        "Target Rot:",
                        width=75,
                        tooltip=(
                            "Additional local rotation offset for the selected rigid body target.\n"
                            "Read the body's local axes in the viewport and apply 90/180 degree corrections here.\n"
                            "Offsets are composed in local X -> Y -> Z order.\n"
                            "Live-editable during Play."
                        ),
                    )
                    for axis_name in ("x", "y", "z"):
                        ui.Label(f"{axis_name.upper()}:", width=18)
                        combo_key = f"rot_{axis_name}_combo"
                        w[combo_key] = ui.ComboBox(
                            2,
                            *self._ROTATION_OFFSET_LABELS,
                            width=60,
                            tooltip=f"Local {axis_name.upper()} rotation offset in degrees.",
                        )
                        w[combo_key].model.add_item_changed_fn(
                            lambda m, _i, s=side, a=axis_name: self._on_rotation_offset_changed(
                                s, a, m.get_item_value_model().as_int
                            )
                        )
                    self._apply_rotation_offsets(side)

                # PD gains
                with ui.HStack(spacing=ROW_SPACING, height=ROW_HEIGHT):
                    ui.Spacer(width=INDENT)
                    ui.Label("Pos:", width=30, tooltip="Position PD gains")
                    ui.Label("Kp:", width=20)
                    w["kp"] = ui.FloatDrag(width=65, min=0.1, max=10000.0, step=1.0)
                    w["kp"].model.set_value(15.0)
                    w["kp"].model.add_value_changed_fn(lambda _m, s=side: self._push_gains(s))
                    ui.Label("Kd:", width=20)
                    w["kd"] = ui.FloatDrag(width=65, min=0.0, max=1000.0, step=1.0)
                    w["kd"].model.set_value(0.5)
                    w["kd"].model.add_value_changed_fn(lambda _m, s=side: self._push_gains(s))

                with ui.HStack(spacing=ROW_SPACING, height=ROW_HEIGHT):
                    ui.Spacer(width=INDENT)
                    ui.Label("Rot:", width=30, tooltip="Orientation PD gains")
                    ui.Label("Kp:", width=20)
                    w["rkp"] = ui.FloatDrag(width=65, min=0.1, max=10000.0, step=1.0)
                    w["rkp"].model.set_value(15.0)
                    w["rkp"].model.add_value_changed_fn(lambda _m, s=side: self._push_gains(s))
                    ui.Label("Kd:", width=20)
                    w["rkd"] = ui.FloatDrag(width=65, min=0.0, max=1000.0, step=1.0)
                    w["rkd"].model.set_value(0.2)
                    w["rkd"].model.add_value_changed_fn(lambda _m, s=side: self._push_gains(s))

                with ui.HStack(spacing=ROW_SPACING, height=ROW_HEIGHT):
                    ui.Spacer(width=INDENT)
                    w["enable_btn"] = ui.Button(
                        "Enable",
                        width=55,
                        clicked_fn=lambda s=side: self._on_toggle(s),
                        tooltip="Enable or disable floating rigid-body velocity tracking for this side",
                        enabled=False,
                    )
                    w["clear_btn"] = ui.Button(
                        "Clear",
                        width=45,
                        clicked_fn=lambda s=side: self._on_clear(s),
                        tooltip="Destroy controller resources (prim path is preserved)",
                        enabled=False,
                    )
                with ui.HStack(spacing=ROW_SPACING, height=STATUS_HEIGHT):
                    ui.Spacer(width=INDENT)
                    w["status"] = ui.Label("", style={"color": CLR_DIM}, word_wrap=True)

    def _apply_config_to_side(self, side: str, cfg: dict) -> None:
        w = self._widgets[side]

        if "prim_path" in cfg and cfg["prim_path"]:
            path_field = w.get("path")
            if path_field:
                path_field.model.set_value(cfg["prim_path"])
                self._save(side, "path", cfg["prim_path"])
                self._fc.set_prim_path(side, cfg["prim_path"])

        for key, widget_key in (
            ("pos_kp", "kp"),
            ("pos_kd", "kd"),
            ("orient_kp", "rkp"),
            ("orient_kd", "rkd"),
        ):
            if key in cfg:
                widget = w.get(widget_key)
                if widget:
                    widget.model.set_value(float(cfg[key]))

        for axis_name in ("x", "y", "z"):
            key = f"target_rot_{axis_name}_deg"
            if key in cfg:
                degrees = int(cfg[key])
                self._set_rotation_offset_combo(side, axis_name, degrees)
                self._save(side, key, degrees)

        self._apply_rotation_offsets(side)
        self._push_gains(side)
        self._sync_side_controls(side)

    def _collect_side_settings(self, side: str) -> dict:
        w = self._widgets[side]
        settings: dict = {}
        path_field = w.get("path")
        if path_field:
            settings["prim_path"] = path_field.model.get_value_as_string()
        kp, kd, rkp, rkd = self._read_gains(side)
        settings["pos_kp"] = kp
        settings["pos_kd"] = kd
        settings["orient_kp"] = rkp
        settings["orient_kd"] = rkd
        settings["target_rot_x_deg"] = self._get_rotation_offset_value(side, "x")
        settings["target_rot_y_deg"] = self._get_rotation_offset_value(side, "y")
        settings["target_rot_z_deg"] = self._get_rotation_offset_value(side, "z")
        return settings

    def collect_profile(self) -> BimanualControllerProfile:
        """Collect the current floating-controller state into a teleop profile section.

        Returns:
            Floating-controller profile section.
        """
        return BimanualControllerProfile(
            left=ControllerSideProfile(
                enabled=self._desired_enabled["left"],
                settings=self._collect_side_settings("left"),
            ),
            right=ControllerSideProfile(
                enabled=self._desired_enabled["right"],
                settings=self._collect_side_settings("right"),
            ),
        )

    def apply_profile(self, profile: BimanualControllerProfile, resolve_stage: bool) -> None:
        """Apply a floating-controller teleop profile section.

        Args:
            profile: Floating-controller profile section to apply.
            resolve_stage: Whether to validate profile paths against the active stage.
        """
        for side, side_profile in (("left", profile.left), ("right", profile.right)):
            self._fc.destroy(side)
            self._tm.clear_floating_side(side)
            self._configured[side] = False
            desired_enabled = bool(side_profile.enabled)
            self._desired_enabled[side] = False
            self._apply_config_to_side(side, side_profile.settings)

            status = self._get_field(side, "status")
            path = str(side_profile.settings.get("prim_path", "")).strip()
            if not path:
                set_status(status, "", CLR_DIM)
                self._sync_side_controls(side)
                continue

            if resolve_stage:
                self._on_path_changed(side)
                if desired_enabled and self._configured[side]:
                    self._on_toggle(side)
            else:
                self._desired_enabled[side] = desired_enabled
                set_status(status, "Profile loaded - stage resolution deferred", CLR_YELLOW)
                self._sync_side_controls(side)

    # ------------------------------------------------------------------
    # Helpers
    # ------------------------------------------------------------------

    def _get_field(self, side: str, key: str) -> object:
        """Return the widget stored under the given key for this side.

        Args:
            side: Controller side containing the widget.
            key: Widget key to read.

        Returns:
            Stored widget, or None if the key is not present.
        """
        return self._widgets[side].get(key)

    def _get_path(self, side: str) -> str:
        f = self._get_field(side, "path")
        return f.model.get_value_as_string() if f else ""

    def _get_rotation_offset_value(self, side: str, axis_name: str) -> int:
        key = f"rot_{axis_name}_combo"
        combo = self._get_field(side, key)
        if combo is None:
            return DEFAULT_ROTATION_OFFSET_DEG
        idx = combo.model.get_item_value_model().get_value_as_int()
        if 0 <= idx < len(self._ROTATION_OFFSET_VALUES):
            return self._ROTATION_OFFSET_VALUES[idx]
        return DEFAULT_ROTATION_OFFSET_DEG

    def _set_rotation_offset_combo(self, side: str, axis_name: str, degrees: int) -> None:
        key = f"rot_{axis_name}_combo"
        combo = self._get_field(side, key)
        if combo is None:
            return
        try:
            idx = self._ROTATION_OFFSET_VALUES.index(int(degrees))
        except ValueError:
            idx = self._ROTATION_OFFSET_VALUES.index(DEFAULT_ROTATION_OFFSET_DEG)
        combo.model.get_item_value_model().set_value(idx)

    def _apply_rotation_offsets(self, side: str) -> None:
        self._fc.set_target_rotation_offsets(
            side,
            self._get_rotation_offset_value(side, "x"),
            self._get_rotation_offset_value(side, "y"),
            self._get_rotation_offset_value(side, "z"),
        )

    def _read_gains(self, side: str) -> tuple[float, float, float, float]:
        w = self._widgets[side]
        kp = w["kp"].model.get_value_as_float() if w.get("kp") else 15.0
        kd = w["kd"].model.get_value_as_float() if w.get("kd") else 0.5
        rkp = w["rkp"].model.get_value_as_float() if w.get("rkp") else 15.0
        rkd = w["rkd"].model.get_value_as_float() if w.get("rkd") else 0.2
        return kp, kd, rkp, rkd

    def _push_gains(self, side: str) -> None:
        kp, kd, rkp, rkd = self._read_gains(side)
        self._fc.set_gains(kp, kd, rkp, rkd, side=side)

    # ------------------------------------------------------------------
    # Timeline-driven UI locking
    # ------------------------------------------------------------------

    def _on_timeline_event(self, event: object) -> None:
        if event.type == int(omni.timeline.TimelineEventType.PLAY):
            self._is_playing = True
            for side in ("left", "right"):
                self._sync_side_controls(side)
                status = self._get_field(side, "status")
                if self._fc.is_running(side):
                    set_status(status, "Active", CLR_GREEN, emit_terminal=True, side=side)
        elif event.type == int(omni.timeline.TimelineEventType.STOP):
            self._is_playing = False
            for side in ("left", "right"):
                self._sync_side_controls(side)
                status = self._get_field(side, "status")
                if self._fc.is_configured(side):
                    set_status(status, "Standby", CLR_YELLOW, emit_terminal=True, side=side)
                else:
                    set_status(status, "", CLR_DIM)

    # ------------------------------------------------------------------
    # Callbacks
    # ------------------------------------------------------------------

    def _on_rotation_offset_changed(self, side: str, axis_name: str, index: int) -> None:
        degrees = (
            self._ROTATION_OFFSET_VALUES[index]
            if 0 <= index < len(self._ROTATION_OFFSET_VALUES)
            else DEFAULT_ROTATION_OFFSET_DEG
        )
        self._save(side, f"target_rot_{axis_name}_deg", degrees)
        self._apply_rotation_offsets(side)

    def _on_path_changed(self, side: str) -> None:
        """Auto-validate when the prim path field changes and destroy stale controller state.

        Args:
            side: Controller side whose path changed.
        """
        if self._is_playing:
            return
        path = self._get_path(side)
        status = self._get_field(side, "status")
        self._save(side, "path", path)

        if self._configured[side]:
            set_status(status, "Configured - press Clear to reconfigure", CLR_YELLOW)
            self._sync_side_controls(side)
            return

        if not path:
            self._fc.destroy(side)
            self._configured[side] = False
            set_status(status, "Set path first", CLR_DIM)
            self._sync_side_controls(side)
            return

        self._fc.set_prim_path(side, path)
        valid, msg = self._fc.validate(side)
        if valid:
            self._configured[side] = True
            set_status(status, f"Configured - {msg}", CLR_YELLOW, emit_terminal=True, side=side)
        else:
            self._configured[side] = False
            set_status(status, f"Apply failed - {msg}", CLR_RED, emit_terminal=True, side=side)
        self._sync_side_controls(side)

    def _on_clear(self, side: str) -> None:
        """Destroy controller resources for a side.

        Args:
            side: Controller side to clear.
        """
        if self._is_playing:
            return
        self._fc.destroy(side)
        self._configured[side] = False
        self._desired_enabled[side] = False
        self._tm.clear_floating_side(side)
        self._sync_side_controls(side)
        status = self._get_field(side, "status")
        set_status(status, "Cleared", CLR_DIM, emit_terminal=True, side=side)

    def _on_toggle(self, side: str) -> None:
        if self._is_playing:
            return
        status = self._get_field(side, "status")

        if self._desired_enabled[side]:
            self._desired_enabled[side] = False
            self._fc.disable(side)
            self._tm.clear_floating_side(side)
            set_status(status, "Disabled", CLR_YELLOW, emit_terminal=True, side=side)
        else:
            if not self._configured[side]:
                set_status(status, "Apply first", CLR_YELLOW)
                self._sync_side_controls(side)
                return
            path = self._get_path(side)
            self._fc.set_prim_path(side, path)
            self._apply_rotation_offsets(side)
            self._push_gains(side)
            if self._fc.configure(side):
                self._desired_enabled[side] = True
                self._tm.set_floating_side_assigned(side, True)
                set_status(status, "Standby", CLR_YELLOW, emit_terminal=True, side=side)
            else:
                self._desired_enabled[side] = False
                self._configured[side] = False
                _, msg = self._fc.validate(side)
                set_status(
                    status,
                    f"Apply failed - {msg}" if msg else "Apply failed",
                    CLR_RED,
                    emit_terminal=True,
                    side=side,
                )

        self._sync_side_controls(side)

    def _sync_side_controls(self, side: str) -> None:
        configured = self._configured[side]
        running = self._fc.is_configured(side)
        path_editable = (not self._is_playing) and (not configured)
        for key in ("path", "plus_btn", "del_btn", "configure_btn"):
            widget = self._get_field(side, key)
            if widget:
                widget.enabled = path_editable
        enable_btn = self._get_field(side, "enable_btn")
        clear_btn = self._get_field(side, "clear_btn")
        if enable_btn:
            enable_btn.enabled = (not self._is_playing) and (configured or self._desired_enabled[side])
            enable_btn.text = "Disable" if self._desired_enabled[side] else "Enable"
        if clear_btn:
            clear_btn.enabled = (not self._is_playing) and configured

    def update_tracking_button(self) -> None:
        """Public refresh for cross-panel coordination."""
        for side in ("left", "right"):
            self._sync_side_controls(side)

    def reset_ui(self) -> None:
        """Reset all UI widgets to idle state (e.g. after stage close)."""
        self._is_playing = False
        self._configured = {"left": False, "right": False}
        self._desired_enabled = {"left": False, "right": False}
        for side in ("left", "right"):
            self._sync_side_controls(side)
            status = self._get_field(side, "status")
            if status:
                set_status(status, "", CLR_DIM)

    def on_stage_closed(self) -> None:
        """Clear stage-bound runtime state while preserving the configured profile in the UI."""
        self._is_playing = False
        self._configured = {"left": False, "right": False}
        for side in ("left", "right"):
            self._sync_side_controls(side)
            status = self._get_field(side, "status")
            if status:
                path = self._get_path(side).strip()
                if path or self._desired_enabled[side]:
                    set_status(status, "Configuration retained - apply after opening a stage.", CLR_YELLOW)
                else:
                    set_status(status, "", CLR_DIM)

    def destroy(self) -> None:
        """Releases the timeline subscription."""
        self._timeline_sub = None
