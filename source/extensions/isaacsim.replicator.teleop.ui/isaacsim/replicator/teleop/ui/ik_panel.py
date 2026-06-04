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

"""IK controller panel - Jacobian-based differential IK for robot arm VR teleop.

Provides per-side (left/right) controls:
- Articulation prim path (auto-validates on change, populates EE links)
- End-effector link dropdown
- IK method selection (for position-based solver)
- End-effector local rotation offsets
- Enable / Disable button
"""

from __future__ import annotations

import carb.settings
import omni.timeline
import omni.ui as ui
from isaacsim.gui.components.ui_utils import get_style
from isaacsim.replicator.teleop import BimanualControllerProfile, ControllerSideProfile, TeleopManager
from isaacsim.replicator.teleop.controllers import (
    IKMethod,
    IKSolverType,
    RobotIKController,
)
from isaacsim.replicator.teleop.controllers._utils import (
    DEFAULT_ROTATION_OFFSET_DEG,
    ROTATION_OFFSET_DEGREES,
    ROTATION_OFFSET_LABELS,
)
from isaacsim.replicator.teleop.controllers.robot_ik import _METHOD_VALUE_MAP

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

_PANEL_NAME = "IK Controller"
_LOG_NAMESPACE = "IK"


def set_status(
    label: ui.Label | None,
    text: str,
    color: int = CLR_DIM,
    emit_terminal: bool = False,
    side: str | None = None,
) -> None:
    """Set the status label text and color for this panel."""
    _set_status_base(label, text, color, source=_LOG_NAMESPACE, emit_terminal=emit_terminal, side=side)


_SETTINGS_PREFIX = "/persistent/exts/isaacsim.replicator.teleop/ik"


class IKPanel:
    """Panel for robot arm IK: path, EE link dropdown, tuning, enable/disable."""

    def __init__(
        self,
        ik_controller: RobotIKController,
        teleop_manager: TeleopManager,
        collapsed_states: dict,
    ) -> None:
        self._ik = ik_controller
        self._tm = teleop_manager
        self._collapsed = collapsed_states
        self._settings = carb.settings.get_settings()

        self._widgets: dict[str, dict] = {"left": {}, "right": {}}
        self._updating_ee_combo: dict[str, bool] = {"left": False, "right": False}
        self._updating_solver_combo: dict[str, bool] = {"left": False, "right": False}
        self._updating_method_combo: dict[str, bool] = {"left": False, "right": False}
        self._updating_pink_qp_solver_combo: dict[str, bool] = {"left": False, "right": False}
        self._link_names: dict[str, list[str]] = {"left": [], "right": []}
        self._configured: dict[str, bool] = {"left": False, "right": False}
        self._desired_enabled: dict[str, bool] = {"left": False, "right": False}
        self._pending_ee_link: dict[str, str] = {"left": "", "right": ""}
        self._is_playing: bool = False
        self._timeline_sub = (
            omni.timeline.get_timeline_interface()
            .get_timeline_event_stream()
            .create_subscription_to_pop(self._on_timeline_event, name="IKPanel_timeline")
        )

        self._available_solvers: list[IKSolverType] = []
        for s in IKSolverType:
            available, reason = self._ik.get_solver_availability(s)
            if available:
                self._available_solvers.append(s)

        all_pink_qp_solvers = list(self._ik.get_pink_qp_solver_names())
        self._pink_qp_solvers: list[str] = []
        self._pink_qp_unavailable: dict[str, str] = {}
        for solver_name in all_pink_qp_solvers:
            available, reason = self._ik.get_pink_qp_solver_availability(solver_name)
            if available:
                self._pink_qp_solvers.append(solver_name)
            else:
                self._pink_qp_unavailable[solver_name] = reason

        for side in ("left", "right"):
            self._settings.set_default_string(f"{_SETTINGS_PREFIX}/{side}/path", "")
            self._settings.set_default_int(f"{_SETTINGS_PREFIX}/{side}/ee_rot_x_deg", DEFAULT_ROTATION_OFFSET_DEG)
            self._settings.set_default_int(f"{_SETTINGS_PREFIX}/{side}/ee_rot_y_deg", DEFAULT_ROTATION_OFFSET_DEG)
            self._settings.set_default_int(f"{_SETTINGS_PREFIX}/{side}/ee_rot_z_deg", DEFAULT_ROTATION_OFFSET_DEG)
            self._settings.set_default_string(f"{_SETTINGS_PREFIX}/{side}/ik_method", IKMethod.SVD.value)
            if self._pink_qp_solvers:
                self._settings.set_default_string(f"{_SETTINGS_PREFIX}/{side}/pink_qp_solver", self._pink_qp_solvers[0])

    # ------------------------------------------------------------------
    # Persistent settings helpers
    # ------------------------------------------------------------------

    def _save(self, side: str, key: str, value: object) -> None:
        path = f"{_SETTINGS_PREFIX}/{side}/{key}"
        if isinstance(value, str):
            self._settings.set_string(path, value)
        elif isinstance(value, (int, bool)):
            self._settings.set_int(path, int(value))

    def _load_str(self, side: str, key: str) -> str:
        return self._settings.get_as_string(f"{_SETTINGS_PREFIX}/{side}/{key}") or ""

    def _load_int(self, side: str, key: str) -> int:
        return int(self._settings.get_as_int(f"{_SETTINGS_PREFIX}/{side}/{key}"))

    # ------------------------------------------------------------------
    # Build
    # ------------------------------------------------------------------

    def build(self) -> None:
        """Build the IK controller panel UI."""
        frame = ui.CollapsableFrame(
            _PANEL_NAME,
            height=0,
            collapsed=self._collapsed.get(_PANEL_NAME, True),
            style=get_style(),
        )
        with frame:
            frame.set_collapsed_changed_fn(lambda c, k=_PANEL_NAME: self._collapsed.__setitem__(k, c))
            with ui.VStack(spacing=0):
                self._build_arm_section("left")
                self._build_arm_section("right")

    # ------------------------------------------------------------------
    # Arm section builder
    # ------------------------------------------------------------------

    def _build_arm_section(self, side: str) -> None:
        """Build one arm section with all controls."""
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
                    on_apply_clicked=lambda s=side: self._configure_side(s),
                    apply_tooltip="Validate and apply this articulation prim path",
                    buttons_out=path_btns,
                )
                w["configure_btn"] = path_btns.get("apply")
                w["plus_btn"] = path_btns.get("plus")
                w["del_btn"] = path_btns.get("delete")

                with ui.HStack(spacing=ROW_SPACING, height=ROW_HEIGHT):
                    ui.Spacer(width=INDENT)
                    ui.Label("EE Link:", width=55, tooltip="End-effector link from the articulation")
                    w["ee_combo"] = ui.ComboBox(
                        0,
                        style={"ComboBox": {"font_size": 14}},
                        tooltip="Select the end-effector link for IK target",
                    )
                    w["ee_combo"].model.add_item_changed_fn(lambda m, _i, s=side: self._on_ee_changed(s))

                with ui.HStack(spacing=ROW_SPACING, height=ROW_HEIGHT):
                    ui.Spacer(width=INDENT)
                    ui.Label("Solver:", width=55, tooltip="IK solver backend.")
                    solver_labels = [s.label for s in self._available_solvers]
                    w["solver_combo"] = ui.ComboBox(
                        0,
                        *solver_labels,
                        width=160,
                        tooltip="",
                    )
                    w["solver_combo"].model.add_item_changed_fn(
                        lambda m, _i, s=side: self._on_solver_changed(s, m.get_item_value_model().as_int)
                    )

                w["method_row"] = ui.HStack(spacing=ROW_SPACING, height=ROW_HEIGHT)
                with w["method_row"]:
                    ui.Spacer(width=INDENT)
                    ui.Label(
                        "Method:",
                        width=55,
                        tooltip="Jacobian inversion method (only for solvers that support it).",
                    )
                    w["method_combo"] = ui.ComboBox(
                        0,
                        *self._METHOD_NAMES,
                        width=160,
                        tooltip="",
                    )
                    w["method_combo"].model.add_item_changed_fn(
                        lambda m, _i, s=side: self._on_method_changed(s, m.get_item_value_model().as_int)
                    )
                    self._sync_method_combo(side)
                    self._refresh_solver_method_tooltips(side)

                with ui.HStack(spacing=ROW_SPACING, height=ROW_HEIGHT):
                    ui.Spacer(width=INDENT)
                    ui.Label(
                        "EE Rot:",
                        width=50,
                        tooltip=(
                            "Additional local rotation offset for the end-effector target.\n"
                            "Read the EE local axes in the viewport and apply 90/180 degree corrections here.\n"
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

                w["gain_row"] = ui.HStack(spacing=ROW_SPACING, height=ROW_HEIGHT, visible=False)
                with w["gain_row"]:
                    ui.Spacer(width=INDENT)
                    ui.Label(
                        "Gain:",
                        width=30,
                        tooltip=(
                            "Velocity-based IK gain (only for Velocity solver).\n"
                            "Controls how aggressively the EE tracks the VR target:\n"
                            "  1-5: smooth, conservative tracking\n"
                            "  10-20: fast, aggressive tracking\n"
                            "  >30: may overshoot or oscillate"
                        ),
                    )
                    w["gain"] = ui.FloatDrag(min=0.1, max=50.0, step=0.5, width=65)
                    w["gain"].model.set_value(self._ik.get_gain(side))
                    w["gain"].model.add_value_changed_fn(lambda m, s=side: self._ik.set_gain(s, m.get_value_as_float()))

                with ui.HStack(spacing=ROW_SPACING, height=ROW_HEIGHT):
                    ui.Spacer(width=INDENT)
                    ui.Label(
                        "VR Target Filter:",
                        width=90,
                        tooltip=(
                            "Exponential moving average (EMA) low-pass filter on the incoming VR target pose before IK.\n"
                            "Each new target is blended with the previous filtered target.\n"
                            "Higher values reduce jitter, but add command delay and make the robot feel less responsive.\n"
                            "Default: 0.0 (no filtering).\n"
                            "  0.0: raw VR target\n"
                            "  0.5: moderate filtering\n"
                            "  0.9: heavy filtering with noticeable delay"
                        ),
                    )
                    w["vr_target_filter"] = ui.FloatDrag(min=0.0, max=0.95, step=0.05, width=65)
                    w["vr_target_filter"].model.set_value(0.0)
                    w["vr_target_filter"].model.add_value_changed_fn(
                        lambda m, s=side: self._ik.set_vr_target_filter(s, m.get_value_as_float())
                    )
                    ui.Label(
                        "Max Joint Step:",
                        width=90,
                        tooltip=(
                            "Maximum joint angle change per simulation step (radians).\n"
                            "Safety clamp applied after the IK solve.\n"
                            "Use it to prevent sudden joint jumps.\n"
                            "Default: 0.0 (disabled, does not interfere with IK).\n"
                            "  0.0: disabled\n"
                            "  0.01-0.05: very conservative, slow but safe\n"
                            "  0.10-0.20: balanced safety cap\n"
                            "This is not a true velocity limit."
                        ),
                    )
                    w["max_joint_step"] = ui.FloatDrag(min=0.0, max=0.5, step=0.01, width=65)
                    w["max_joint_step"].model.set_value(0.0)
                    w["max_joint_step"].model.add_value_changed_fn(
                        lambda m, s=side: self._ik.set_max_joint_step(s, m.get_value_as_float())
                    )

                w["pink_row1"] = ui.HStack(spacing=ROW_SPACING, height=ROW_HEIGHT, visible=False)
                with w["pink_row1"]:
                    ui.Spacer(width=INDENT)
                    ui.Label("PINK:", width=35, tooltip="PINK-specific task tuning.")
                    ui.Label(
                        "Task Gain:",
                        width=60,
                        tooltip=(
                            "PINK FrameTask response gain.\n"
                            "Higher values make the end-effector track more aggressively.\n"
                            "Lower values make tracking softer/slower without filtering the target."
                        ),
                    )
                    w["pink_task_gain"] = ui.FloatDrag(min=0.05, max=1.0, step=0.05, width=65)
                    w["pink_task_gain"].model.set_value(self._ik.get_pink_task_gain(side))
                    w["pink_task_gain"].model.add_value_changed_fn(
                        lambda m, s=side: self._ik.set_pink_task_gain(s, m.get_value_as_float())
                    )
                    ui.Label(
                        "Posture:",
                        width=50,
                        tooltip=(
                            "PINK posture regularization cost.\n"
                            "Higher values keep the arm closer to its current posture.\n"
                            "Lower values give the end-effector task more freedom."
                        ),
                    )
                    w["pink_posture_cost"] = ui.FloatDrag(min=0.0, max=0.1, step=0.001, width=70)
                    w["pink_posture_cost"].model.set_value(self._ik.get_pink_posture_cost(side))
                    w["pink_posture_cost"].model.add_value_changed_fn(
                        lambda m, s=side: self._ik.set_pink_posture_cost(s, m.get_value_as_float())
                    )

                w["pink_row2"] = ui.HStack(spacing=ROW_SPACING, height=ROW_HEIGHT, visible=False)
                with w["pink_row2"]:
                    ui.Spacer(width=INDENT)
                    ui.Spacer(width=35)
                    ui.Label(
                        "QP:",
                        width=25,
                        tooltip=(
                            "PINK quadratic-program solver backend.\n"
                            "Use this to compare solve quality and performance across QP solvers."
                        ),
                    )
                    pink_qp_solver_labels = [solver.upper() for solver in self._pink_qp_solvers]
                    qp_combo_tooltip = "Choose the PINK QP solver backend."
                    if self._pink_qp_unavailable:
                        unavailable_summary = ", ".join(sorted(self._pink_qp_unavailable))
                        qp_combo_tooltip += f"\nHidden (not importable): {unavailable_summary}"
                    w["pink_qp_solver"] = ui.ComboBox(
                        0,
                        *pink_qp_solver_labels,
                        width=70,
                        tooltip=qp_combo_tooltip,
                    )
                    w["pink_qp_solver"].model.add_item_changed_fn(
                        lambda m, _i, s=side: self._on_pink_qp_solver_changed(s, m.get_item_value_model().as_int)
                    )
                    saved_pink_qp_solver = self._load_str(side, "pink_qp_solver") or self._ik.get_pink_qp_solver(side)
                    success, _ = self._ik.set_pink_qp_solver(side, saved_pink_qp_solver)
                    if not success:
                        self._ik.set_pink_qp_solver(side, self._ik.get_pink_qp_solver(side))
                    self._set_pink_qp_solver_combo(side, self._ik.get_pink_qp_solver(side))
                    ui.Label(
                        "LM Damp:",
                        width=60,
                        tooltip=(
                            "PINK FrameTask damping.\n"
                            "Higher values improve stability near hard configurations but slow response.\n"
                            "Lower values are more reactive but may twitch more."
                        ),
                    )
                    w["pink_lm_damping"] = ui.FloatDrag(min=0.001, max=10.0, step=0.05, width=70)
                    w["pink_lm_damping"].model.set_value(self._ik.get_pink_lm_damping(side))
                    w["pink_lm_damping"].model.add_value_changed_fn(
                        lambda m, s=side: self._ik.set_pink_lm_damping(s, m.get_value_as_float())
                    )

                with ui.HStack(spacing=ROW_SPACING, height=ROW_HEIGHT):
                    ui.Spacer(width=INDENT)
                    w["enable_btn"] = ui.Button(
                        "Enable",
                        width=55,
                        clicked_fn=lambda s=side: self._on_toggle(s),
                        tooltip="Enable/disable IK tracking for this arm",
                        enabled=False,
                    )
                    w["clear_btn"] = ui.Button(
                        "Clear",
                        width=45,
                        clicked_fn=lambda s=side: self._on_clear(s),
                        tooltip="Destroy solver and articulation resources (prim path is preserved)",
                        enabled=False,
                    )
                with ui.HStack(spacing=ROW_SPACING, height=STATUS_HEIGHT):
                    ui.Spacer(width=INDENT)
                    w["status"] = ui.Label("", style={"color": CLR_DIM}, word_wrap=True)

    # ------------------------------------------------------------------
    # Helpers
    # ------------------------------------------------------------------

    def _get_field(self, side: str, key: str) -> object:
        """Return the widget stored under the given key for this side."""
        return self._widgets[side].get(key)

    def _get_path(self, side: str) -> str:
        f = self._get_field(side, "path")
        return f.model.get_value_as_string() if f else ""

    def _get_ee_link_name(self, side: str) -> str:
        combo = self._get_field(side, "ee_combo")
        if combo is None:
            return ""
        names = self._link_names[side]
        idx = combo.model.get_item_value_model().get_value_as_int()
        if 0 <= idx < len(names):
            return names[idx]
        return ""

    def _set_combo_silent(self, side: str, guard_key: str, combo: object, value: int) -> None:
        """Set a combo box value without triggering its change callback."""
        guard = f"_updating_{guard_key}"
        flags = getattr(self, guard, None)
        if flags is None:
            return
        flags[side] = True
        try:
            combo.model.get_item_value_model().set_value(value)
        finally:
            flags[side] = False

    def _get_solver(self, side: str) -> IKSolverType:
        """Return the current solver type from the controller."""
        return self._ik.get_solver_type(side)

    def _get_selected_pink_qp_solver(self, side: str) -> str:
        combo = self._get_field(side, "pink_qp_solver")
        if combo is None:
            return self._ik.get_pink_qp_solver(side)
        idx = combo.model.get_item_value_model().get_value_as_int()
        if 0 <= idx < len(self._pink_qp_solvers):
            return self._pink_qp_solvers[idx]
        return self._ik.get_pink_qp_solver(side)

    def _set_pink_qp_solver_combo(self, side: str, solver_name: str) -> None:
        combo = self._get_field(side, "pink_qp_solver")
        if combo is None:
            return
        try:
            idx = self._pink_qp_solvers.index(solver_name)
        except ValueError:
            idx = 0
        self._set_combo_silent(side, "pink_qp_solver_combo", combo, idx)

    def _populate_ee_combo(self, side: str, link_names: list[str]) -> None:
        """Replace ComboBox items with the given link names."""
        self._link_names[side] = list(link_names)
        combo = self._get_field(side, "ee_combo")
        if combo is None:
            return
        self._updating_ee_combo[side] = True
        try:
            model = combo.model
            for child in model.get_item_children():
                model.remove_item(child)
            for name in link_names:
                model.append_child_item(None, ui.SimpleStringModel(name))
            if link_names:
                model.get_item_value_model().set_value(len(link_names) - 1)
        finally:
            self._updating_ee_combo[side] = False

    def _push_side_settings(self, side: str) -> None:
        """Push current widget values to the controller for one side."""
        path = self._get_path(side)
        if path:
            self._ik.set_articulation_path(side, path)
        self._ik.set_ee_link_name(side, self._get_ee_link_name(side))
        solver_combo = self._get_field(side, "solver_combo")
        if solver_combo:
            idx = solver_combo.model.get_item_value_model().get_value_as_int()
            if 0 <= idx < len(self._available_solvers):
                self._ik.set_solver_type(side, self._available_solvers[idx])
        self._ik.set_pink_qp_solver(side, self._get_selected_pink_qp_solver(side))
        method = self._get_selected_method(side)
        if method is not None:
            self._ik.set_ik_method(side, method)

    # ------------------------------------------------------------------
    # Timeline-driven UI locking
    # ------------------------------------------------------------------

    def _on_timeline_event(self, event: object) -> None:
        if event.type == int(omni.timeline.TimelineEventType.PLAY):
            self._is_playing = True
            for side in ("left", "right"):
                self._sync_side_controls(side)
                status = self._get_field(side, "status")
                if self._ik.is_running(side):
                    set_status(status, "Active", CLR_GREEN, emit_terminal=True, side=side)
        elif event.type == int(omni.timeline.TimelineEventType.STOP):
            self._is_playing = False
            for side in ("left", "right"):
                self._sync_side_controls(side)
                status = self._get_field(side, "status")
                if self._ik.is_configured(side):
                    set_status(status, "Standby", CLR_YELLOW, emit_terminal=True, side=side)
                else:
                    set_status(status, "", CLR_DIM)

    # ------------------------------------------------------------------
    # Callbacks
    # ------------------------------------------------------------------

    def _on_ee_changed(self, side: str) -> None:
        if self._updating_ee_combo.get(side, False):
            return
        self._apply_ee_selection(side, update_status=True)

    def _apply_ee_selection(self, side: str, update_status: bool) -> None:
        """Apply current EE selection to controller and optionally refresh status."""
        ee_name = self._get_ee_link_name(side)
        self._ik.set_ee_link_name(side, ee_name)
        if ee_name:
            chain_dofs = self._ik.compute_arm_dofs(side)
            if chain_dofs is not None and chain_dofs > 0:
                self._ik.set_num_arm_dofs(side, chain_dofs)
        if not update_status:
            return
        result = self._ik.validate(side)
        status = self._get_field(side, "status")
        if result.valid:
            set_status(status, f"Configured - {result.message}", CLR_YELLOW)
        elif result.link_names:
            set_status(status, result.message, CLR_YELLOW)

    _METHOD_MAP = [
        IKMethod.DAMPED_LEAST_SQUARES,
        IKMethod.PSEUDOINVERSE,
        IKMethod.TRANSPOSE,
        IKMethod.SVD,
    ]
    _METHOD_NAMES = ["Damped LS", "Pseudoinverse", "Transpose", "SVD"]
    _ROTATION_OFFSET_VALUES = list(ROTATION_OFFSET_DEGREES)
    _ROTATION_OFFSET_LABELS = list(ROTATION_OFFSET_LABELS)

    def _on_solver_changed(self, side: str, index: int) -> None:
        """Switch IK solver type (lightweight swap — no USD queries)."""
        if self._updating_solver_combo.get(side, False):
            return
        solver = self._available_solvers[index] if index < len(self._available_solvers) else IKSolverType.POSITION_BASED
        success, message = self._ik.set_solver_type(side, solver)
        status = self._get_field(side, "status")
        if not success:
            actual_solver = self._get_solver(side)
            solver_combo = self._get_field(side, "solver_combo")
            if solver_combo is not None and actual_solver in self._available_solvers:
                self._set_combo_silent(side, "solver_combo", solver_combo, self._available_solvers.index(actual_solver))
            set_status(status, message, CLR_RED, emit_terminal=True, side=side)
            self._sync_side_controls(side)
            return
        self._save(side, "solver", solver.value)
        if self._ik.is_running(side):
            set_status(status, message, CLR_GREEN, emit_terminal=True, side=side)
        self._sync_side_controls(side)

    def _set_method_from_string(self, side: str, method_str: str) -> None:
        method = _METHOD_VALUE_MAP.get(method_str)
        self._ik.set_ik_method(side, method if method is not None else IKMethod.SVD)

    def _get_selected_method(self, side: str) -> IKMethod | None:
        if not self._get_solver(side).supports_method:
            return None
        combo = self._get_field(side, "method_combo")
        if combo is None:
            return None
        idx = combo.model.get_item_value_model().get_value_as_int()
        if 0 <= idx < len(self._METHOD_MAP):
            return self._METHOD_MAP[idx]
        return IKMethod.SVD

    def _sync_method_combo(self, side: str) -> None:
        """Sync the method combo index to match the controller's current method."""
        combo = self._get_field(side, "method_combo")
        if combo is None:
            return
        current = self._ik.get_ik_method(side)
        try:
            idx = self._METHOD_MAP.index(current)
        except ValueError:
            idx = len(self._METHOD_MAP) - 1
        current_idx = combo.model.get_item_value_model().get_value_as_int()
        if current_idx != idx:
            self._set_combo_silent(side, "method_combo", combo, idx)

    def _on_method_changed(self, side: str, index: int) -> None:
        if self._updating_method_combo.get(side, False):
            return
        if not self._get_solver(side).supports_method:
            return
        method = self._METHOD_MAP[index] if 0 <= index < len(self._METHOD_MAP) else IKMethod.SVD
        self._ik.set_ik_method(side, method)
        self._save(side, "ik_method", method.value)
        self._refresh_solver_method_tooltips(side)
        status = self._get_field(side, "status")
        if self._ik.is_running(side):
            set_status(status, f"Switched method to {method.value}", CLR_GREEN, emit_terminal=True, side=side)

    def _on_pink_qp_solver_changed(self, side: str, index: int) -> None:
        if self._updating_pink_qp_solver_combo.get(side, False):
            return
        if not self._pink_qp_solvers:
            return
        solver_name = (
            self._pink_qp_solvers[index] if 0 <= index < len(self._pink_qp_solvers) else self._pink_qp_solvers[0]
        )
        success, message = self._ik.set_pink_qp_solver(side, solver_name)
        status = self._get_field(side, "status")
        if not success:
            self._set_pink_qp_solver_combo(side, self._ik.get_pink_qp_solver(side))
            set_status(status, message, CLR_RED, emit_terminal=True, side=side)
            return
        actual_solver = self._ik.get_pink_qp_solver(side)
        self._set_pink_qp_solver_combo(side, actual_solver)
        self._save(side, "pink_qp_solver", actual_solver)
        if self._ik.is_running(side) and self._get_solver(side).supports_pink_advanced:
            set_status(status, message, CLR_GREEN, emit_terminal=True, side=side)

    def _refresh_solver_method_tooltips(self, side: str) -> None:
        solver = self._get_solver(side)
        solver_combo = self._get_field(side, "solver_combo")
        method_combo = self._get_field(side, "method_combo")
        if solver_combo:
            solver_combo.tooltip = f"{solver.description}\nHot-swappable during play for configured arms."
        if method_combo:
            if not solver.supports_method:
                method_combo.tooltip = "Not selectable for this solver."
            else:
                method = self._get_selected_method(side) or IKMethod.SVD
                method_combo.tooltip = method.description

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
        self._ik.set_ee_rotation_offsets(
            side,
            self._get_rotation_offset_value(side, "x"),
            self._get_rotation_offset_value(side, "y"),
            self._get_rotation_offset_value(side, "z"),
        )

    def _on_rotation_offset_changed(self, side: str, axis_name: str, index: int) -> None:
        degrees = (
            self._ROTATION_OFFSET_VALUES[index]
            if 0 <= index < len(self._ROTATION_OFFSET_VALUES)
            else DEFAULT_ROTATION_OFFSET_DEG
        )
        self._save(side, f"ee_rot_{axis_name}_deg", degrees)
        self._apply_rotation_offsets(side)

    # ------------------------------------------------------------------
    # Config presets
    # ------------------------------------------------------------------

    def _apply_config_to_side(self, side: str, cfg: dict) -> None:
        """Populate one side's widgets from a config dict (does not auto-configure)."""
        w = self._widgets[side]

        if "robot_path" in cfg:
            path_field = w.get("path")
            if path_field:
                path_field.model.set_value(cfg["robot_path"])
                self._save(side, "path", cfg["robot_path"])
                self._ik.set_articulation_path(side, cfg["robot_path"])

        if "ee_link" in cfg:
            self._pending_ee_link[side] = cfg["ee_link"]

        if "solver" in cfg:
            try:
                solver = IKSolverType(cfg["solver"])
                solver_combo = w.get("solver_combo")
                if solver_combo and solver in self._available_solvers:
                    idx = self._available_solvers.index(solver)
                    solver_combo.model.get_item_value_model().set_value(idx)
                else:
                    _, reason = self._ik.get_solver_availability(solver)
                    if reason:
                        set_status(self._get_field(side, "status"), reason, CLR_YELLOW)
            except ValueError:
                pass

        if "method" in cfg:
            self._set_method_from_string(side, cfg["method"])
            self._sync_method_combo(side)
            self._save(side, "ik_method", cfg["method"])

        if "gain" in cfg:
            gain_widget = w.get("gain")
            if gain_widget:
                gain_widget.model.set_value(float(cfg["gain"]))

        if "vr_target_filter" in cfg:
            filter_widget = w.get("vr_target_filter")
            if filter_widget:
                filter_widget.model.set_value(float(cfg["vr_target_filter"]))

        if "max_joint_step" in cfg:
            step_widget = w.get("max_joint_step")
            if step_widget:
                step_widget.model.set_value(float(cfg["max_joint_step"]))

        if "pink_task_gain" in cfg:
            pink_gain_widget = w.get("pink_task_gain")
            if pink_gain_widget:
                pink_gain_widget.model.set_value(float(cfg["pink_task_gain"]))

        if "pink_qp_solver" in cfg:
            success, reason = self._ik.set_pink_qp_solver(side, str(cfg["pink_qp_solver"]))
            if success:
                self._set_pink_qp_solver_combo(side, self._ik.get_pink_qp_solver(side))
                self._save(side, "pink_qp_solver", self._ik.get_pink_qp_solver(side))
            else:
                set_status(self._get_field(side, "status"), reason, CLR_YELLOW)

        if "pink_posture_cost" in cfg:
            pink_posture_widget = w.get("pink_posture_cost")
            if pink_posture_widget:
                pink_posture_widget.model.set_value(float(cfg["pink_posture_cost"]))

        if "pink_lm_damping" in cfg:
            pink_lm_widget = w.get("pink_lm_damping")
            if pink_lm_widget:
                pink_lm_widget.model.set_value(float(cfg["pink_lm_damping"]))

        for axis_name in ("x", "y", "z"):
            key = f"ee_rot_{axis_name}_deg"
            if key in cfg:
                degrees = int(cfg[key])
                self._set_rotation_offset_combo(side, axis_name, degrees)
                self._save(side, key, degrees)

        self._apply_rotation_offsets(side)
        self._sync_side_controls(side)

    def _collect_side_settings(self, side: str) -> dict:
        """Read current widget values for a side into a config dict."""
        w = self._widgets[side]
        settings: dict = {}
        path_field = w.get("path")
        if path_field:
            settings["robot_path"] = path_field.model.get_value_as_string()

        ee_name = self._get_ee_link_name(side)
        if ee_name:
            settings["ee_link"] = ee_name

        solver = self._get_solver(side)
        settings["solver"] = solver.value

        if solver.supports_method:
            method = self._get_selected_method(side)
            if method is not None:
                settings["method"] = method.value

        gain_widget = w.get("gain")
        if gain_widget:
            settings["gain"] = round(gain_widget.model.get_value_as_float(), 3)

        filter_widget = w.get("vr_target_filter")
        if filter_widget:
            settings["vr_target_filter"] = round(filter_widget.model.get_value_as_float(), 3)

        step_widget = w.get("max_joint_step")
        if step_widget:
            settings["max_joint_step"] = round(step_widget.model.get_value_as_float(), 3)

        if solver.supports_pink_advanced:
            settings["pink_qp_solver"] = self._ik.get_pink_qp_solver(side)
            pink_gain_widget = w.get("pink_task_gain")
            if pink_gain_widget:
                settings["pink_task_gain"] = round(pink_gain_widget.model.get_value_as_float(), 3)

            pink_posture_widget = w.get("pink_posture_cost")
            if pink_posture_widget:
                settings["pink_posture_cost"] = round(pink_posture_widget.model.get_value_as_float(), 4)

            pink_lm_widget = w.get("pink_lm_damping")
            if pink_lm_widget:
                settings["pink_lm_damping"] = round(pink_lm_widget.model.get_value_as_float(), 4)

        settings["ee_rot_x_deg"] = self._get_rotation_offset_value(side, "x")
        settings["ee_rot_y_deg"] = self._get_rotation_offset_value(side, "y")
        settings["ee_rot_z_deg"] = self._get_rotation_offset_value(side, "z")
        return settings

    def collect_profile(self) -> BimanualControllerProfile:
        """Collect the current IK-controller state into a teleop profile section."""
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
        """Apply an IK-controller teleop profile section."""
        for side, side_profile in (("left", profile.left), ("right", profile.right)):
            self._ik.disable(side)
            self._ik.destroy(side)
            self._configured[side] = False
            desired_enabled = bool(side_profile.enabled)
            self._desired_enabled[side] = False
            self._pending_ee_link[side] = ""
            self._apply_config_to_side(side, side_profile.settings)

            status = self._get_field(side, "status")
            path = str(side_profile.settings.get("robot_path", "")).strip()
            if not path:
                self._populate_ee_combo(side, [])
                set_status(status, "", CLR_DIM)
                self._sync_side_controls(side)
                continue

            if resolve_stage:
                self._configure_side(side)
                if desired_enabled and self._configured[side]:
                    self._on_toggle(side)
            else:
                self._desired_enabled[side] = desired_enabled
                set_status(status, "Profile loaded - stage resolution deferred", CLR_YELLOW)
                self._sync_side_controls(side)

    # ------------------------------------------------------------------
    # Configure / Clear / Toggle
    # ------------------------------------------------------------------

    def _configure_side(self, side: str) -> None:
        """Configure one arm side and populate the EE dropdown from the articulation.

        Changing the path destroys any existing solver so stale resources
        are not kept.
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
            self._ik.destroy(side)
            self._configured[side] = False
            set_status(status, "Set path first", CLR_DIM)
            self._populate_ee_combo(side, [])
            self._sync_side_controls(side)
            return

        # set_articulation_path calls destroy() internally when path changed
        self._ik.set_articulation_path(side, path)
        self._ik.set_ee_link_name(side, "")
        result = self._ik.validate(side)

        if result.link_names:
            self._populate_ee_combo(side, result.link_names)
            pending = self._pending_ee_link[side]
            if pending and pending in result.link_names:
                idx = result.link_names.index(pending)
                combo = self._get_field(side, "ee_combo")
                if combo:
                    self._updating_ee_combo[side] = True
                    try:
                        combo.model.get_item_value_model().set_value(idx)
                    finally:
                        self._updating_ee_combo[side] = False
            self._pending_ee_link[side] = ""
            self._apply_ee_selection(side, update_status=False)
            result = self._ik.validate(side)

        if result.valid:
            self._configured[side] = True
            set_status(status, f"Configured - {result.message}", CLR_YELLOW, emit_terminal=True, side=side)
        else:
            self._configured[side] = False
            color = CLR_YELLOW if result.link_names else CLR_RED
            set_status(status, result.message, color, emit_terminal=True, side=side)

        self._sync_side_controls(side)

    def _on_clear(self, side: str) -> None:
        """Disable (if running) and destroys solver and articulation resources."""
        if self._is_playing:
            return
        self._ik.disable(side)
        self._ik.destroy(side)
        self._configured[side] = False
        self._desired_enabled[side] = False
        self._sync_side_controls(side)
        status = self._get_field(side, "status")
        set_status(status, "Cleared", CLR_DIM, emit_terminal=True, side=side)

    def _on_toggle(self, side: str) -> None:
        if self._is_playing:
            return
        status = self._get_field(side, "status")

        if self._desired_enabled[side]:
            self._desired_enabled[side] = False
            self._ik.disable(side)
            set_status(status, "Disabled", CLR_YELLOW, emit_terminal=True, side=side)
        else:
            if not self._configured[side]:
                set_status(status, "Apply first", CLR_YELLOW)
                self._sync_side_controls(side)
                return
            self._push_side_settings(side)
            result = self._ik.validate(side)
            if not result.valid:
                self._desired_enabled[side] = False
                self._configured[side] = False
                set_status(status, result.message, CLR_RED, emit_terminal=True, side=side)
                self._sync_side_controls(side)
                return
            if self._ik.enable(side):
                self._desired_enabled[side] = True
                set_status(status, "Standby", CLR_YELLOW, emit_terminal=True, side=side)
            else:
                self._desired_enabled[side] = False
                set_status(status, result.message, CLR_RED, emit_terminal=True, side=side)

        self._sync_side_controls(side)

    def _sync_side_controls(self, side: str) -> None:
        """Single method that synchronises all UI widgets from controller state."""
        configured = self._configured[side]
        running = self._ik.is_running(side)
        solver = self._get_solver(side)
        path_editable = (not self._is_playing) and (not configured)

        for key in ("path", "plus_btn", "del_btn", "configure_btn"):
            widget = self._get_field(side, key)
            if widget:
                widget.enabled = path_editable

        ee_combo = self._get_field(side, "ee_combo")
        if ee_combo:
            ee_combo.enabled = configured and (not self._is_playing)

        for key in ("rot_x_combo", "rot_y_combo", "rot_z_combo"):
            combo = self._get_field(side, key)
            if combo:
                combo.enabled = configured

        enable_btn = self._get_field(side, "enable_btn")
        clear_btn = self._get_field(side, "clear_btn")
        if enable_btn:
            enable_btn.enabled = (not self._is_playing) and (configured or self._desired_enabled[side])
            enable_btn.text = "Disable" if self._desired_enabled[side] else "Enable"
        if clear_btn:
            clear_btn.enabled = (not self._is_playing) and configured

        solver_combo = self._get_field(side, "solver_combo")
        if solver_combo:
            solver_combo.enabled = True

        method_row = self._get_field(side, "method_row")
        if method_row:
            method_row.visible = solver.supports_method

        gain_row = self._get_field(side, "gain_row")
        if gain_row:
            gain_row.visible = solver.supports_gain

        pink_row1 = self._get_field(side, "pink_row1")
        if pink_row1:
            pink_row1.visible = solver.supports_pink_advanced

        pink_row2 = self._get_field(side, "pink_row2")
        if pink_row2:
            pink_row2.visible = solver.supports_pink_advanced

        self._sync_method_combo(side)
        self._refresh_solver_method_tooltips(side)

    def update_enable_buttons(self) -> None:
        """Refresh Enable/Disable button labels (e.g. after external disable)."""
        for side in ("left", "right"):
            self._sync_side_controls(side)

    def on_reachability_changed(self, side: str, reachable: bool) -> None:
        """Called by the IK controller when reachability changes during play."""
        if not self._ik.is_running(side):
            return
        status = self._get_field(side, "status")
        if reachable:
            set_status(status, "Active", CLR_GREEN, emit_terminal=True, side=side)
        else:
            set_status(status, "Out of reach", CLR_YELLOW, emit_terminal=True, side=side)

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
                ee_combo = self._get_field(side, "ee_combo")
                has_ee_selection = False
                if ee_combo:
                    ee_index = ee_combo.model.get_item_value_model().get_value_as_int()
                    has_ee_selection = ee_index >= 0 and len(ee_combo.model.get_item_children()) > 0
                if path or has_ee_selection or self._desired_enabled[side]:
                    set_status(status, "Configuration retained - apply after opening a stage.", CLR_YELLOW)
                else:
                    set_status(status, "", CLR_DIM)

    def destroy(self) -> None:
        """Release the timeline subscription."""
        self._timeline_sub = None
