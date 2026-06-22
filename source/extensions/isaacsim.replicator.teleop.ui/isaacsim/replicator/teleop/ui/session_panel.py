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

"""Session panel - OpenXR connection, frame markers, and synthetic debug controls."""

from __future__ import annotations

import carb.settings
import omni.ui as ui
import omni.usd
from isaacsim.gui.components.ui_utils import get_style
from isaacsim.replicator.teleop import (
    CoordinateSystem,
    MarkersManager,
    TeleopCommand,
    TeleopManager,
    TeleopSettingsProfile,
    get_teleop_backend,
    set_teleop_backend,
)
from isaacsim.replicator.teleop.xr_anchor_manager import AnchorRotationMode
from pxr import Sdf, UsdGeom

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

_PANEL_NAME = "Session"
_LOG_NAMESPACE = "Session"


def set_status(label: ui.Label | None, text: str, color: int = CLR_DIM, emit_terminal: bool = False) -> None:
    """Set the status label text and color for this panel.

    Args:
        label: Label widget to update.
        text: Status text to display.
        color: Label text color.
        emit_terminal: Whether to print the status change to the terminal.
    """
    _set_status_base(label, text, color, source=_LOG_NAMESPACE, emit_terminal=emit_terminal)


_SETTINGS_PREFIX = "/persistent/exts/isaacsim.replicator.teleop/session"

# Dropdown options: display name -> enum value
_COORD_SYSTEMS = [
    ("Isaac Sim (Z-up)", CoordinateSystem.ISAAC_SIM),
    ("Raw (no conversion)", CoordinateSystem.RAW),
]

_ROTATION_MODES = [
    ("Fixed", AnchorRotationMode.FIXED),
    ("Follow Prim", AnchorRotationMode.FOLLOW_PRIM),
    ("Follow (Smoothed)", AnchorRotationMode.FOLLOW_PRIM_SMOOTHED),
]

_MARKER_BACKENDS = [
    ("USD", "usd"),
    ("USD-RT", "usdrt"),
    ("Fabric", "fabric"),
]


class SessionPanel:
    """Teleop session panel: connection, frame markers, and synthetic debug controls.

    Args:
        teleop_manager: Shared teleop manager for session state.
        markers_manager: Manager for session marker prims.
        collapsed_states: Mutable panel collapsed-state cache.
    """

    def __init__(
        self,
        teleop_manager: TeleopManager,
        markers_manager: MarkersManager,
        collapsed_states: dict,
    ) -> None:
        self._tm = teleop_manager
        self._mm = markers_manager
        self._collapsed = collapsed_states
        self._settings = carb.settings.get_settings()

        self._settings.set_default_string(f"{_SETTINGS_PREFIX}/tracking_space_path", "")
        self._settings.set_default_float(f"{_SETTINGS_PREFIX}/anchor_x", 0.0)
        self._settings.set_default_float(f"{_SETTINGS_PREFIX}/anchor_y", 0.0)
        self._settings.set_default_float(f"{_SETTINGS_PREFIX}/anchor_z", 0.0)
        self._settings.set_default_int(f"{_SETTINGS_PREFIX}/anchor_rot_mode", 0)
        self._settings.set_default_float(f"{_SETTINGS_PREFIX}/anchor_smoothing", 1.0)
        self._settings.set_default_bool(f"{_SETTINGS_PREFIX}/anchor_fixed_height", True)

        self._status_label: ui.Label | None = None
        self._connect_btn: ui.Button | None = None
        self._disconnect_btn: ui.Button | None = None
        self._marker_status: ui.Label | None = None
        self._show_btn: ui.Button | None = None
        self._remove_btn: ui.Button | None = None
        self._marker_scale_field: ui.FloatDrag | None = None
        self._coord_system_combo: ui.ComboBox | None = None
        self._tracking_space_field: ui.StringField | None = None
        self._tracking_space_status: ui.Label | None = None
        self._tracking_space_toggle_btn: ui.Button | None = None
        self._anchor_x_field: ui.FloatField | None = None
        self._anchor_y_field: ui.FloatField | None = None
        self._anchor_z_field: ui.FloatField | None = None
        self._anchor_rotation_combo: ui.ComboBox | None = None
        self._anchor_smoothing_slider: ui.FloatSlider | None = None
        self._anchor_fixed_height_cb: ui.CheckBox | None = None
        self._debug_tracking_cb: ui.CheckBox | None = None
        self._debug_backend_combo: ui.ComboBox | None = None

        # User intent for the custom XR anchor. Decoupled from runtime so Clear
        # records ``enabled=False`` even though the built-in marker is active,
        # and so Show Markers / Connect skip auto-apply on a profile loaded as
        # disabled with a non-empty path.
        self._tracking_space_intended_active: bool = False

    def build(self) -> None:
        """Build the session panel UI."""
        frame = ui.CollapsableFrame(
            _PANEL_NAME, height=0, collapsed=self._collapsed.get(_PANEL_NAME, False), style=get_style()
        )
        with frame:
            frame.set_collapsed_changed_fn(lambda c, k=_PANEL_NAME: self._collapsed.__setitem__(k, c))
            with ui.VStack(spacing=0):
                with ui.HStack(spacing=ROW_SPACING):
                    ui.Spacer(width=INDENT)
                    self._connect_btn = ui.Button(
                        "Connect", clicked_fn=self._on_connect, tooltip="Connect to OpenXR teleop session"
                    )
                    self._disconnect_btn = ui.Button(
                        "Disconnect", clicked_fn=self._on_disconnect, tooltip="Disconnect from session", enabled=False
                    )
                with ui.HStack(spacing=ROW_SPACING, height=ROW_HEIGHT):
                    ui.Spacer(width=INDENT)
                    ui.Label("Status:", width=50)
                    self._status_label = ui.Label("Disconnected", style={"color": CLR_RED})

                markers_key = f"{_PANEL_NAME}:Frame Markers"
                with ui.CollapsableFrame(
                    "Frame Markers",
                    height=0,
                    collapsed=self._collapsed.get(markers_key, True),
                    style=get_style(),
                ) as markers_frame:
                    markers_frame.set_collapsed_changed_fn(lambda c, k=markers_key: self._collapsed.__setitem__(k, c))
                    with ui.VStack(spacing=SECTION_SPACING):
                        with ui.HStack(spacing=ROW_SPACING, height=ROW_HEIGHT):
                            ui.Spacer(width=INDENT)
                            self._show_btn = ui.Button(
                                "Show",
                                width=55,
                                clicked_fn=self._on_show_markers,
                                tooltip="Create frame markers and start VR tracking",
                            )
                            self._remove_btn = ui.Button(
                                "Remove",
                                width=55,
                                clicked_fn=self._on_remove_markers,
                                tooltip="Remove all markers and stop tracking",
                                enabled=False,
                            )
                            ui.Spacer(width=INDENT)
                            ui.Label("Scale:", width=35, tooltip="Visual scale for frame marker axes")
                            self._marker_scale_field = ui.FloatDrag(
                                min=0.001,
                                max=1.0,
                                step=0.005,
                                width=55,
                                tooltip="Visual size of frame marker axes",
                            )
                            self._marker_scale_field.model.set_value(self._mm.frame_scale)
                            self._marker_scale_field.model.add_value_changed_fn(
                                lambda m: self._mm.set_frame_scale(m.get_value_as_float())
                            )

                        with ui.HStack(spacing=ROW_SPACING, height=STATUS_HEIGHT):
                            ui.Spacer(width=INDENT)
                            self._marker_status = ui.Label("", style={"color": CLR_DIM}, word_wrap=True)

                anchor_key = f"{_PANEL_NAME}:XR Anchor"
                with ui.CollapsableFrame(
                    "XR Anchor",
                    height=0,
                    collapsed=self._collapsed.get(anchor_key, True),
                    style=get_style(),
                ) as anchor_frame:
                    anchor_frame.set_collapsed_changed_fn(lambda c, k=anchor_key: self._collapsed.__setitem__(k, c))
                    with ui.VStack(spacing=SECTION_SPACING):
                        with ui.HStack(spacing=ROW_SPACING, height=ROW_HEIGHT):
                            ui.Spacer(width=INDENT)
                            ui.Label("Coordinate Frame:", width=110)
                            self._coord_system_combo = ui.ComboBox(
                                0,
                                *[name for name, _ in _COORD_SYSTEMS],
                                width=160,
                                tooltip="Coordinate system of incoming VR pose data (can be changed live)",
                            )
                            self._coord_system_combo.model.add_item_changed_fn(
                                lambda model, _item: self._on_coord_system_changed(model.get_item_value_model().as_int)
                            )

                        path_btns: dict = {}
                        self._tracking_space_field = build_prim_path_row(
                            "Custom Anchor:",
                            tooltip=(
                                "Scene prim to anchor the VR headset and controllers to (Kit's 'Custom USD Anchor').\n"
                                "Empty path resolves to the built-in origin marker under /Teleop/Markers/.\n"
                                "Paths under /Teleop/Markers/ are reserved and will fall back to the built-in origin.\n"
                                "Set applies a typed path live; Clear reverts the active anchor to the built-in origin."
                            ),
                            on_apply_clicked=self._on_tracking_space_toggle,
                            apply_label="Set",
                            apply_tooltip="Validate and activate the anchor prim; follows the prim live",
                            buttons_out=path_btns,
                        )
                        self._tracking_space_toggle_btn = path_btns.get("apply")
                        self._tracking_space_field.model.add_value_changed_fn(
                            lambda _m: self._sync_tracking_space_controls()
                        )
                        self._tracking_space_field.model.add_end_edit_fn(
                            lambda _m: self._on_tracking_space_field_edited()
                        )

                        with ui.HStack(spacing=ROW_SPACING, height=STATUS_HEIGHT):
                            ui.Spacer(width=INDENT)
                            self._tracking_space_status = ui.Label("", style={"color": CLR_DIM}, word_wrap=True)

                        with ui.HStack(spacing=ROW_SPACING, height=ROW_HEIGHT):
                            ui.Spacer(width=INDENT)
                            ui.Label(
                                "Offset:",
                                width=90,
                                tooltip=(
                                    "Position offset (metres) for the VR headset camera.\n"
                                    "No Custom Anchor: this is an absolute world position.\n"
                                    "With a Custom Anchor: offset relative to that prim."
                                ),
                            )
                            ui.Label("X", width=10)
                            self._anchor_x_field = ui.FloatField(
                                width=55, tooltip="X offset - forward in Isaac Sim (Z-up)"
                            )
                            self._anchor_x_field.model.set_value(0.0)
                            ui.Label("Y", width=10)
                            self._anchor_y_field = ui.FloatField(
                                width=55, tooltip="Y offset - left in Isaac Sim (Z-up)"
                            )
                            self._anchor_y_field.model.set_value(0.0)
                            ui.Label("Z", width=10)
                            self._anchor_z_field = ui.FloatField(width=55, tooltip="Z offset - up in Isaac Sim (Z-up)")
                            self._anchor_z_field.model.set_value(0.0)
                            self._anchor_x_field.model.add_value_changed_fn(
                                lambda _m: self._on_anchor_offset_changed(
                                    self._anchor_x_field, self._anchor_y_field, self._anchor_z_field
                                )
                            )
                            self._anchor_y_field.model.add_value_changed_fn(
                                lambda _m: self._on_anchor_offset_changed(
                                    self._anchor_x_field, self._anchor_y_field, self._anchor_z_field
                                )
                            )
                            self._anchor_z_field.model.add_value_changed_fn(
                                lambda _m: self._on_anchor_offset_changed(
                                    self._anchor_x_field, self._anchor_y_field, self._anchor_z_field
                                )
                            )

                        with ui.HStack(spacing=ROW_SPACING, height=ROW_HEIGHT):
                            ui.Spacer(width=INDENT)
                            ui.Label(
                                "Rotation:",
                                width=90,
                                tooltip=(
                                    "How the headset camera rotation tracks the Custom Anchor prim.\n"
                                    "Only relevant when a Custom Anchor prim is set above.\n"
                                    "- Fixed: ignore anchor rotation entirely.\n"
                                    "- Follow Prim: track yaw only (roll/pitch stripped).\n"
                                    "- Smoothed: yaw follows with slerp damping."
                                ),
                            )
                            self._anchor_rotation_combo = ui.ComboBox(
                                0,
                                *[name for name, _ in _ROTATION_MODES],
                                width=140,
                                tooltip=(
                                    "Fixed: headset orientation uses offset only.\n"
                                    "Follow Prim: yaw tracks anchor prim rotation.\n"
                                    "Smoothed: yaw tracks with slerp damping."
                                ),
                            )
                            self._anchor_rotation_combo.model.add_item_changed_fn(
                                lambda model, _item: self._on_anchor_rotation_mode_changed(
                                    model.get_item_value_model().as_int
                                )
                            )
                            ui.Label(
                                "Smooth:", width=48, tooltip="Slerp time constant in seconds (only for Smoothed mode)"
                            )
                            self._anchor_smoothing_slider = ui.FloatSlider(
                                min=0.05,
                                max=3.0,
                                step=0.05,
                                width=ui.Fraction(1),
                                tooltip="Lower = snappier tracking, Higher = smoother (only used in Smoothed mode)",
                            )
                            self._anchor_smoothing_slider.model.set_value(1.0)
                            self._anchor_smoothing_slider.model.add_value_changed_fn(
                                lambda m: self._on_anchor_smoothing_changed(m.get_value_as_float())
                            )

                        with ui.HStack(spacing=ROW_SPACING, height=ROW_HEIGHT):
                            ui.Spacer(width=INDENT)
                            self._anchor_fixed_height_cb = ui.CheckBox(
                                width=20,
                                tooltip=(
                                    "Lock the headset camera height (Z) to the value it had\n"
                                    "on the first frame. Prevents vertical bobbing when the\n"
                                    "Custom Anchor prim moves up/down (e.g. uneven terrain)."
                                ),
                            )
                            self._anchor_fixed_height_cb.model.set_value(True)
                            self._anchor_fixed_height_cb.model.add_value_changed_fn(
                                lambda m: self._on_anchor_fixed_height_changed(m.get_value_as_bool())
                            )
                            ui.Label(
                                "Fixed Height",
                                width=80,
                                tooltip="Lock headset camera Z to its initial value during dynamic anchoring",
                            )

                debug_key = f"{_PANEL_NAME}:Debug"
                with ui.CollapsableFrame(
                    "Debug",
                    height=0,
                    collapsed=self._collapsed.get(debug_key, True),
                    style=get_style(),
                ) as debug_frame:
                    debug_frame.set_collapsed_changed_fn(lambda c, k=debug_key: self._collapsed.__setitem__(k, c))
                    with ui.VStack(spacing=SECTION_SPACING):
                        with ui.HStack(spacing=ROW_SPACING, height=ROW_HEIGHT):
                            ui.Spacer(width=INDENT)
                            ui.Label(
                                "Write Backend:", width=85, tooltip="Global XformPrim backend for all teleop writes"
                            )
                            initial_debug_backend_idx = next(
                                (i for i, (_, val) in enumerate(_MARKER_BACKENDS) if val == get_teleop_backend()), 0
                            )
                            self._debug_backend_combo = ui.ComboBox(
                                initial_debug_backend_idx,
                                *[name for name, _ in _MARKER_BACKENDS],
                                width=100,
                                tooltip=(
                                    "Global teleop backend for XformPrim writes.\n"
                                    "USD: plain attribute writes (default).\n"
                                    "USD-RT / Fabric: faster paths (require FSD)."
                                ),
                            )
                            self._debug_backend_combo.model.add_item_changed_fn(
                                lambda model, _item: self._on_debug_backend_changed(model.get_item_value_model().as_int)
                            )

                        with ui.HStack(spacing=ROW_SPACING, height=ROW_HEIGHT):
                            ui.Spacer(width=INDENT)
                            self._debug_tracking_cb = ui.CheckBox(
                                width=20,
                                tooltip=(
                                    "When enabled, IK / floating / grasp / locomotion read\n"
                                    "their target poses from the Left and Right frame\n"
                                    "markers instead of real VR controllers.\n\n"
                                    "Drag the markers in the viewport to drive the robot.\n"
                                    "No VR connection is required.\n\n"
                                    "Use the controls below to simulate grasp and\n"
                                    "locomotion input.\n\n"
                                    "Mutually exclusive with a live VR connection.\n"
                                    "Disconnect first to enable debug tracking."
                                ),
                            )
                            self._debug_tracking_cb.model.set_value(self._tm.debug_tracking_enabled)
                            self._debug_tracking_cb.model.add_value_changed_fn(
                                lambda m: self._on_debug_tracking_toggled(m.get_value_as_bool())
                            )
                            ui.Label(
                                "Debug Tracking",
                                width=90,
                                tooltip=(
                                    "Synthetic pose source: markers become inputs.\n"
                                    "All downstream controllers are driven by marker\n"
                                    "world poses and the controls below."
                                ),
                            )

                        with ui.HStack(spacing=ROW_SPACING, height=ROW_HEIGHT):
                            ui.Spacer(width=INDENT)
                            ui.Label(
                                "L Grasp:",
                                width=55,
                                tooltip="Left hand trigger analog value (0 = open, 1 = fully closed)",
                            )
                            lt_slider = ui.FloatSlider(
                                min=0.0,
                                max=1.0,
                                step=0.01,
                                width=ui.Fraction(1),
                                tooltip="Simulated left trigger — fed to grasp controller as trigger_value",
                            )
                            lt_slider.model.set_value(0.0)
                            lt_slider.model.add_value_changed_fn(
                                lambda m: self._tm.set_debug_trigger("left", m.get_value_as_float())
                            )
                            ui.Label(
                                "R Grasp:",
                                width=55,
                                tooltip="Right hand trigger analog value (0 = open, 1 = fully closed)",
                            )
                            rt_slider = ui.FloatSlider(
                                min=0.0,
                                max=1.0,
                                step=0.01,
                                width=ui.Fraction(1),
                                tooltip="Simulated right trigger — fed to grasp controller as trigger_value",
                            )
                            rt_slider.model.set_value(0.0)
                            rt_slider.model.add_value_changed_fn(
                                lambda m: self._tm.set_debug_trigger("right", m.get_value_as_float())
                            )

                        with ui.HStack(spacing=ROW_SPACING, height=ROW_HEIGHT):
                            ui.Spacer(width=INDENT)
                            ui.Label(
                                "Slide X:",
                                width=55,
                                tooltip="Synthetic left-thumbstick X for locomotion lateral slide (-1 = right, 1 = left)",
                            )
                            slide_x_slider = ui.FloatSlider(
                                min=-1.0,
                                max=1.0,
                                step=0.01,
                                width=ui.Fraction(1),
                                tooltip="Synthetic left-thumbstick X for locomotion lateral slide",
                            )
                            slide_x_slider.model.set_value(0.0)
                            slide_x_slider.model.add_value_changed_fn(
                                lambda m: self._tm.set_debug_thumbstick("left", x=m.get_value_as_float())
                            )
                            ui.Label(
                                "Slide Y:",
                                width=55,
                                tooltip="Synthetic left-thumbstick Y for locomotion forward/back slide (-1 = back, 1 = forward)",
                            )
                            slide_y_slider = ui.FloatSlider(
                                min=-1.0,
                                max=1.0,
                                step=0.01,
                                width=ui.Fraction(1),
                                tooltip="Synthetic left-thumbstick Y for locomotion forward/back slide",
                            )
                            slide_y_slider.model.set_value(0.0)
                            slide_y_slider.model.add_value_changed_fn(
                                lambda m: self._tm.set_debug_thumbstick("left", y=m.get_value_as_float())
                            )

                        with ui.HStack(spacing=ROW_SPACING, height=ROW_HEIGHT):
                            ui.Spacer(width=INDENT)
                            ui.Label(
                                "Turn:",
                                width=40,
                                tooltip="Synthetic right-thumbstick X for locomotion yaw turn (-1 = right, 1 = left)",
                            )
                            turn_slider = ui.FloatSlider(
                                min=-1.0,
                                max=1.0,
                                step=0.01,
                                width=ui.Fraction(1),
                                tooltip="Synthetic right-thumbstick X for locomotion yaw turn",
                            )
                            turn_slider.model.set_value(0.0)
                            turn_slider.model.add_value_changed_fn(
                                lambda m: self._tm.set_debug_thumbstick("right", x=m.get_value_as_float())
                            )
                            ui.Label(
                                "Up/Down:",
                                width=55,
                                tooltip="Synthetic right-side locomotion buttons for vertical motion",
                            )
                            up_button = ui.Button(
                                "Up",
                                width=50,
                                tooltip="Hold to press the synthetic right secondary locomotion button",
                            )
                            up_button.set_mouse_pressed_fn(
                                lambda x, y, b, m: self._tm.set_debug_button("right", "secondary_click", True)
                            )
                            up_button.set_mouse_released_fn(
                                lambda x, y, b, m: self._tm.set_debug_button("right", "secondary_click", False)
                            )
                            down_button = ui.Button(
                                "Down",
                                width=60,
                                tooltip="Hold to press the synthetic right primary locomotion button",
                            )
                            down_button.set_mouse_pressed_fn(
                                lambda x, y, b, m: self._tm.set_debug_button("right", "primary_click", True)
                            )
                            down_button.set_mouse_released_fn(
                                lambda x, y, b, m: self._tm.set_debug_button("right", "primary_click", False)
                            )
                            carry_button = ui.Button(
                                "Carry Origin",
                                width=90,
                                tooltip="Hold to toggle Carry Tracking Space (synthetic left primary button)",
                            )
                            carry_button.set_mouse_pressed_fn(
                                lambda x, y, b, m: self._tm.set_debug_button("left", "primary_click", True)
                            )
                            carry_button.set_mouse_released_fn(
                                lambda x, y, b, m: self._tm.set_debug_button("left", "primary_click", False)
                            )

    # ------------------------------------------------------------------
    # Marker backend callback
    # ------------------------------------------------------------------

    def _on_debug_backend_changed(self, index: int) -> None:
        if 0 <= index < len(_MARKER_BACKENDS):
            _, backend = _MARKER_BACKENDS[index]
            set_teleop_backend(backend)

    # ------------------------------------------------------------------
    # Debug tracking callbacks
    # ------------------------------------------------------------------

    def _on_debug_tracking_toggled(self, enabled: bool) -> None:
        """Ensure markers exist before enabling debug tracking mode.

        Args:
            enabled: Whether debug tracking should be enabled.
        """
        if enabled:
            if self._tm.is_connected:
                if self._debug_tracking_cb:
                    self._debug_tracking_cb.model.set_value(False)
                return
            ctx = omni.usd.get_context()
            if ctx.get_stage() is None:
                set_status(self._marker_status, "No stage - open a scene first", CLR_RED, emit_terminal=True)
                if self._debug_tracking_cb:
                    self._debug_tracking_cb.model.set_value(False)
                return
            for name in ("origin", "left", "right", "head"):
                ok, msg = self._mm.ensure_marker(name)
                if not ok:
                    set_status(self._marker_status, msg, CLR_RED, emit_terminal=True)
                    if self._debug_tracking_cb:
                        self._debug_tracking_cb.model.set_value(False)
                    return
        self._tm.set_debug_tracking(enabled)
        if enabled:
            self._tm.set_builtin_tracking_space()
        self._sync_ui()

    # ------------------------------------------------------------------
    # Marker callbacks
    # ------------------------------------------------------------------

    def _on_show_markers(self) -> None:
        """Create frame markers (if needed) and starts live tracking."""
        ctx = omni.usd.get_context()
        if ctx.get_stage() is None:
            set_status(self._marker_status, "No stage - open a scene first", CLR_RED, emit_terminal=True)
            return
        _, _, remaining = ctx.get_stage_loading_status()
        if remaining > 0:
            set_status(self._marker_status, "Stage still loading - try again shortly", CLR_YELLOW, emit_terminal=True)
            return

        selection = ctx.get_selection()
        prev_selection = selection.get_selected_prim_paths()
        selection.clear_selected_prim_paths()

        for name in ("origin", "left", "right", "head"):
            ok, msg = self._mm.ensure_marker(name)
            if not ok:
                set_status(self._marker_status, msg, CLR_RED, emit_terminal=True)
                if prev_selection:
                    selection.set_selected_prim_paths(prev_selection, False)
                return

        if prev_selection:
            selection.set_selected_prim_paths(prev_selection, False)

        self._tm.set_live_tracking(True)
        self._sync_marker_buttons()
        set_status(self._marker_status, "Tracking", CLR_GREEN, emit_terminal=True)
        if (
            self._tracking_space_intended_active
            and self._tracking_space_field
            and self._tracking_space_field.model.get_value_as_string().strip()
        ):
            self._set_tracking_space()
        else:
            self._sync_tracking_space_controls()

    def _on_remove_markers(self) -> None:
        """Stop tracking, disables debug mode, and removes all markers from the stage."""
        if self._tm.debug_tracking_enabled:
            self._tm.set_debug_tracking(False)
            if self._debug_tracking_cb:
                self._debug_tracking_cb.model.set_value(False)
        self._tm.set_live_tracking(False)
        self._tm.disable_tracking_space()
        self._mm.remove_all_markers()
        set_status(self._marker_status, "", CLR_DIM)
        set_status(
            self._tracking_space_status,
            "XR Anchor inactive - reactivates when markers are shown again.",
            CLR_YELLOW,
        )
        self._sync_marker_buttons()
        self._sync_tracking_space_controls()

    def _on_tracking_space_field_edited(self) -> None:
        """Persist a finalized field edit to settings and surface a hint.

        Wired to ``add_end_edit_fn`` so settings are written and status is
        emitted only on commit (Enter / focus loss). The cheaper
        ``_sync_tracking_space_controls`` runs separately on every keystroke
        to keep the Set / Clear label live without per-character settings
        writes. Activation still requires a Set click.
        """
        path = self._tracking_space_field.model.get_value_as_string() if self._tracking_space_field else ""
        self._settings.set_string(f"{_SETTINGS_PREFIX}/tracking_space_path", path)
        active = self._tm.tracking_space_prim_path
        stripped = path.strip()
        if stripped and active and stripped != active:
            set_status(self._tracking_space_status, "Field changed - click Set to apply", CLR_DIM)
        self._sync_tracking_space_controls()

    def _on_tracking_space_toggle(self) -> None:
        """Apply the field as the tracking space, or clear back to the built-in origin."""
        if self._is_tracking_space_clearable():
            self._clear_tracking_space()
        else:
            self._set_tracking_space()

    def _set_tracking_space(self) -> None:
        """Validate the field and activate the tracking space, following the prim live.

        Empty path or invalid input falls back to the built-in origin marker
        when session markers are active; otherwise activation is deferred
        until markers are shown.
        """
        path = self._tracking_space_field.model.get_value_as_string().strip() if self._tracking_space_field else ""
        self._settings.set_string(f"{_SETTINGS_PREFIX}/tracking_space_path", path)

        ok, deferred, msg = self._activate_tracking_space(path)
        if not ok:
            set_status(self._tracking_space_status, f"Failed — {msg}", CLR_RED, emit_terminal=True)
            self._sync_tracking_space_controls()
            return

        self._tracking_space_intended_active = bool(path)

        if path and self._mm.has_active_markers and self._is_valid_xformable(path):
            self._mm.move_tracking_space_to(path)

        set_status(
            self._tracking_space_status,
            msg if deferred else f"Set - {msg}",
            CLR_YELLOW if deferred else CLR_GREEN,
            emit_terminal=True,
        )
        self._sync_tracking_space_controls()

    def _clear_tracking_space(self) -> None:
        """Revert the active tracking space to the built-in origin marker at world (0,0,0).

        Keeps the typed path in the field (the bin glyph is the dedicated
        clear) and resets the origin marker to identity so the headset
        re-anchors at the world origin. When markers are not yet active the
        built-in origin is rearmed implicitly on the next Show Markers.
        Drops the intent flag so the next ``collect_profile`` saves
        ``enabled=False`` and Connect / Show Markers skip auto-apply.
        """
        self._tracking_space_intended_active = False
        if self._mm.has_active_markers:
            self._mm.reset_marker_transform("origin")
            ok, msg = self._tm.set_builtin_tracking_space()
            color = CLR_GREEN if ok else CLR_RED
            if ok:
                text = f"Cleared - {msg}" if msg else "Cleared"
            else:
                text = f"Failed — {msg}" if msg else "Failed"
            set_status(self._tracking_space_status, text, color, emit_terminal=True)
        else:
            self._tm.disable_tracking_space()
            set_status(
                self._tracking_space_status,
                "Cleared - built-in anchor will activate when session markers are shown.",
                CLR_YELLOW,
                emit_terminal=True,
            )
        self._sync_tracking_space_controls()

    def _activate_tracking_space(self, path: str) -> tuple[bool, bool, str]:
        """Activate the given path or fall back to the built-in anchor.

        Args:
            path: Tracking-space prim path to activate.

        Returns:
            Tuple containing whether the operation succeeded, whether activation
            was deferred, and the message to surface in the UI or log.
        """
        if not path or path.startswith(MarkersManager.MARKERS_SCOPE):
            if not self._mm.has_active_markers:
                return True, True, "Built-in anchor will activate when session markers are active."
            ok, msg = self._tm.set_builtin_tracking_space()
            return ok, False, msg

        if self._is_valid_xformable(path):
            ok, msg = self._tm.set_tracking_space_prim_path(path)
            return ok, False, msg

        if not self._mm.has_active_markers:
            return True, True, "Built-in anchor will activate when session markers are active."
        ok, msg = self._tm.set_builtin_tracking_space()
        return ok, False, msg

    def _is_valid_xformable(self, path: str) -> bool:
        """Return True when ``path`` resolves to a valid Xformable prim on the active stage.

        Args:
            path: Prim path to validate.

        Returns:
            True if the path resolves to a valid Xformable prim, False otherwise.
        """
        stage = omni.usd.get_context().get_stage()
        if stage is None or not Sdf.Path.IsValidPathString(path):
            return False
        prim = stage.GetPrimAtPath(path)
        return bool(prim and prim.IsValid() and UsdGeom.Xformable(prim))

    def _is_tracking_space_clearable(self) -> bool:
        """True when the user has opted in to the typed path as the custom anchor.

        Tracks the in-memory ``intended_active`` flag rather than runtime
        state so Clear remains available while activation is deferred (no
        markers / no stage), and so editing the field after a Set still
        offers Clear as the obvious revert action.

        Returns:
            True if the tracking-space field can be cleared, False otherwise.
        """
        if self._tracking_space_field is None:
            return False
        field_path = self._tracking_space_field.model.get_value_as_string().strip()
        if not field_path:
            return False
        return self._tracking_space_intended_active

    def _sync_tracking_space_controls(self) -> None:
        """Toggle the row button between Set and Clear based on field-vs-active state."""
        if self._tracking_space_toggle_btn is None:
            return
        if self._is_tracking_space_clearable():
            self._tracking_space_toggle_btn.text = "Clear"
            self._tracking_space_toggle_btn.tooltip = (
                "Revert the active custom origin to the built-in origin marker at world (0,0,0). "
                "The field text is preserved — use the bin glyph to clear it."
            )
        else:
            self._tracking_space_toggle_btn.text = "Set"
            self._tracking_space_toggle_btn.tooltip = "Validate and activate the anchor prim; follows the prim live"

    # ------------------------------------------------------------------
    # Anchor callbacks
    # ------------------------------------------------------------------

    def _on_anchor_offset_changed(self, ax: ui.FloatField, ay: ui.FloatField, az: ui.FloatField) -> None:
        x = ax.model.get_value_as_float()
        y = ay.model.get_value_as_float()
        z = az.model.get_value_as_float()
        self._settings.set_float(f"{_SETTINGS_PREFIX}/anchor_x", x)
        self._settings.set_float(f"{_SETTINGS_PREFIX}/anchor_y", y)
        self._settings.set_float(f"{_SETTINGS_PREFIX}/anchor_z", z)
        self._tm.set_xr_anchor_pos((x, y, z))

    def _on_anchor_rotation_mode_changed(self, index: int) -> None:
        self._settings.set_int(f"{_SETTINGS_PREFIX}/anchor_rot_mode", index)
        if 0 <= index < len(_ROTATION_MODES):
            _, mode = _ROTATION_MODES[index]
            self._tm.set_xr_anchor_rotation_mode(mode)

    def _on_anchor_smoothing_changed(self, value: float) -> None:
        self._settings.set_float(f"{_SETTINGS_PREFIX}/anchor_smoothing", value)
        self._tm.set_xr_anchor_smoothing_time(value)

    def _on_anchor_fixed_height_changed(self, fixed: bool) -> None:
        self._settings.set_bool(f"{_SETTINGS_PREFIX}/anchor_fixed_height", fixed)
        self._tm.set_xr_anchor_fixed_height(fixed)

    def collect_profile(self) -> TeleopSettingsProfile:
        """Collect the current session panel values into a teleop settings profile.

        Returns:
            Teleop settings profile section.
        """
        coordinate_system = CoordinateSystem.ISAAC_SIM.value
        if self._coord_system_combo is not None:
            index = self._coord_system_combo.model.get_item_value_model().get_value_as_int()
            if 0 <= index < len(_COORD_SYSTEMS):
                coordinate_system = _COORD_SYSTEMS[index][1].value

        anchor_rotation_mode = AnchorRotationMode.FIXED.value
        if self._anchor_rotation_combo is not None:
            index = self._anchor_rotation_combo.model.get_item_value_model().get_value_as_int()
            if 0 <= index < len(_ROTATION_MODES):
                anchor_rotation_mode = _ROTATION_MODES[index][1].value

        tracking_space_path = (
            self._tracking_space_field.model.get_value_as_string() if self._tracking_space_field else ""
        )
        anchor_x = self._anchor_x_field.model.get_value_as_float() if self._anchor_x_field else 0.0
        anchor_y = self._anchor_y_field.model.get_value_as_float() if self._anchor_y_field else 0.0
        anchor_z = self._anchor_z_field.model.get_value_as_float() if self._anchor_z_field else 0.0
        anchor_smoothing = (
            self._anchor_smoothing_slider.model.get_value_as_float() if self._anchor_smoothing_slider else 1.0
        )
        anchor_fixed_height = (
            self._anchor_fixed_height_cb.model.get_value_as_bool() if self._anchor_fixed_height_cb else True
        )

        return TeleopSettingsProfile(
            coordinate_system=coordinate_system,
            tracking_space_enabled=self._tracking_space_intended_active,
            tracking_space_path=tracking_space_path,
            marker_scale=self._mm.frame_scale,
            anchor_x=anchor_x,
            anchor_y=anchor_y,
            anchor_z=anchor_z,
            anchor_rotation_mode=anchor_rotation_mode,
            anchor_smoothing=anchor_smoothing,
            anchor_fixed_height=anchor_fixed_height,
        )

    def apply_profile(self, profile: TeleopSettingsProfile, resolve_stage: bool) -> None:
        """Apply a teleop settings profile to the panel and shared manager state.

        Args:
            profile: Teleop settings profile section to apply.
            resolve_stage: Whether to validate profile paths against the active stage.
        """
        coord_index = 0
        for index, (_, system) in enumerate(_COORD_SYSTEMS):
            if system.value == profile.coordinate_system:
                coord_index = index
                break
        if self._coord_system_combo is not None:
            self._coord_system_combo.model.get_item_value_model().set_value(coord_index)
        self._on_coord_system_changed(coord_index)

        if self._marker_scale_field is not None:
            self._marker_scale_field.model.set_value(float(profile.marker_scale))
        else:
            self._mm.set_frame_scale(float(profile.marker_scale))

        if self._tracking_space_field is not None:
            self._tracking_space_field.model.set_value(profile.tracking_space_path)
        self._settings.set_string(f"{_SETTINGS_PREFIX}/tracking_space_path", profile.tracking_space_path)

        if self._anchor_x_field is not None:
            self._anchor_x_field.model.set_value(float(profile.anchor_x))
        if self._anchor_y_field is not None:
            self._anchor_y_field.model.set_value(float(profile.anchor_y))
        if self._anchor_z_field is not None:
            self._anchor_z_field.model.set_value(float(profile.anchor_z))
        self._settings.set_float(f"{_SETTINGS_PREFIX}/anchor_x", float(profile.anchor_x))
        self._settings.set_float(f"{_SETTINGS_PREFIX}/anchor_y", float(profile.anchor_y))
        self._settings.set_float(f"{_SETTINGS_PREFIX}/anchor_z", float(profile.anchor_z))
        self._tm.set_xr_anchor_pos((float(profile.anchor_x), float(profile.anchor_y), float(profile.anchor_z)))

        rotation_index = 0
        for index, (_, mode) in enumerate(_ROTATION_MODES):
            if mode.value == profile.anchor_rotation_mode:
                rotation_index = index
                break
        if self._anchor_rotation_combo is not None:
            self._anchor_rotation_combo.model.get_item_value_model().set_value(rotation_index)
        self._on_anchor_rotation_mode_changed(rotation_index)

        if self._anchor_smoothing_slider is not None:
            self._anchor_smoothing_slider.model.set_value(float(profile.anchor_smoothing))
        self._settings.set_float(f"{_SETTINGS_PREFIX}/anchor_smoothing", float(profile.anchor_smoothing))
        self._tm.set_xr_anchor_smoothing_time(float(profile.anchor_smoothing))

        if self._anchor_fixed_height_cb is not None:
            self._anchor_fixed_height_cb.model.set_value(bool(profile.anchor_fixed_height))
        self._settings.set_bool(f"{_SETTINGS_PREFIX}/anchor_fixed_height", bool(profile.anchor_fixed_height))
        self._tm.set_xr_anchor_fixed_height(bool(profile.anchor_fixed_height))

        # Strict honor: a profile saved with the toggle off stays quiescent
        # even if the path field is non-empty. The intent flag (set / clear /
        # apply / collect all share it) is the single source of truth.
        wants_active = bool(profile.tracking_space_enabled) and bool(profile.tracking_space_path)
        self._tracking_space_intended_active = wants_active
        if resolve_stage and wants_active:
            self._set_tracking_space()
        elif wants_active:
            set_status(
                self._tracking_space_status,
                "XR Anchor loaded; activation deferred until stage is ready.",
                CLR_YELLOW,
            )
        else:
            set_status(self._tracking_space_status, "", CLR_DIM)
        self._sync_tracking_space_controls()

    def sync_from_command(self, command: TeleopCommand, success: bool, message: str) -> None:
        """Update session UI after an external command bus execution.

        Only reacts to connection-related commands.  Timeline commands
        (START / STOP / RESET) are handled by the controller panels.

        Args:
            command: The :class:`TeleopCommand` that was executed.
            success: Whether the command succeeded.
            message: Human-readable result.
        """
        self._sync_ui()
        self._sync_marker_buttons()

        if command == TeleopCommand.CONNECT:
            if success and self._tm.is_live_tracking:
                set_status(self._marker_status, "Tracking", CLR_GREEN, emit_terminal=True)
                if (
                    self._tracking_space_intended_active
                    and self._tracking_space_field
                    and self._tracking_space_field.model.get_value_as_string().strip()
                ):
                    self._set_tracking_space()
                self._set_status("Connected - markers active", emit_terminal=True)

    def _sync_marker_buttons(self) -> None:
        """Enables/disables marker buttons based on current state."""
        is_tracking = self._tm.is_live_tracking
        if self._show_btn:
            self._show_btn.enabled = not is_tracking
        if self._remove_btn:
            self._remove_btn.enabled = is_tracking or self._mm.has_active_markers

    # ------------------------------------------------------------------
    # Connection callbacks
    # ------------------------------------------------------------------

    def _on_connect(self) -> None:
        ctx = omni.usd.get_context()
        if ctx.get_stage() is None:
            self._set_status("No stage - open a scene first", emit_terminal=True)
            return
        _, _, remaining = ctx.get_stage_loading_status()
        if remaining > 0:
            self._set_status("Stage still loading - try again shortly", emit_terminal=True)
            return

        self._tm.connect(on_status_changed=lambda text: self._set_status(text, emit_terminal=True))
        self._sync_ui()
        if self._tm.is_connected:
            self._on_show_markers()
            self._set_status("Connected - markers active", emit_terminal=True)

    def _on_disconnect(self) -> None:
        self._on_remove_markers()
        self._tm.disconnect()
        self.reset_ui()

    def reset_ui(self) -> None:
        """Reset all UI widgets to the disconnected/idle state."""
        self._set_status("Disconnected", emit_terminal=True)
        self._sync_ui()
        set_status(self._marker_status, "", CLR_DIM)
        self._tm.disable_tracking_space()
        set_status(self._tracking_space_status, "", CLR_DIM)
        self._sync_tracking_space_controls()
        self._sync_marker_buttons()

    def on_stage_closed(self) -> None:
        """Clear stage-bound runtime state while preserving the configured profile in the UI."""
        self._set_status("Disconnected")
        self._sync_ui()
        set_status(self._marker_status, "", CLR_DIM)
        if self._tracking_space_field and self._tracking_space_field.model.get_value_as_string().strip():
            set_status(
                self._tracking_space_status,
                "XR Anchor retained - click Set after opening a stage.",
                CLR_YELLOW,
            )
        else:
            set_status(self._tracking_space_status, "", CLR_DIM)
        self._sync_tracking_space_controls()
        self._sync_marker_buttons()

    def _on_coord_system_changed(self, index: int) -> None:
        if 0 <= index < len(_COORD_SYSTEMS):
            _, system = _COORD_SYSTEMS[index]
            self._tm.set_coordinate_system(system)

    def _set_status(self, text: str, emit_terminal: bool = False) -> None:
        if self._status_label:
            if self._status_label.text == text:
                return
            self._status_label.text = text
            if "no data" in text.lower():
                self._status_label.style = {"color": CLR_YELLOW}
            elif self._tm.is_connected:
                self._status_label.style = {"color": CLR_GREEN}
            else:
                self._status_label.style = {"color": CLR_RED}
        if text and emit_terminal:
            print(f"[Teleop][{_LOG_NAMESPACE}] {text}")

    def _sync_ui(self) -> None:
        connected = self._tm.is_connected
        debug = self._tm.debug_tracking_enabled
        if self._connect_btn:
            self._connect_btn.enabled = not connected and not debug
        if self._disconnect_btn:
            self._disconnect_btn.enabled = connected
        if self._debug_tracking_cb:
            self._debug_tracking_cb.enabled = not connected
        if self._debug_backend_combo:
            self._debug_backend_combo.enabled = not connected
