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

"""Named Pose property panel for the Kit Property Window.

Registers a ``SimplePropertyWidget`` that appears when an
``IsaacNamedPose`` prim is selected.  Uses shared helpers from
``fk_helpers`` for IK tracking, FK computation, and IK failure outlines
so that no logic is duplicated with the Robot Poser window.

Tracking is fully standalone -- does not require the Robot Poser window.
"""

from __future__ import annotations

from typing import Any

import carb
import isaacsim.robot.poser.robot_poser as robot_poser
import numpy as np
import omni.kit.app
import omni.ui as ui
import omni.usd
from omni.kit.window.property.templates import (
    HORIZONTAL_SPACING,
    LABEL_HEIGHT,
    LABEL_WIDTH,
    SimplePropertyWidget,
)
from pxr import Sdf, Tf, Usd
from usd.schema.isaac import robot_schema

from .style import get_property_style
from .ui.site_widget import SiteSearchComboBox
from .utils.fk_helpers import (
    apply_ik_chain_outline,
    build_pose_from_current_joints,
    build_tracking_cache,
    clear_ik_chain_outline,
    compute_fk_and_write_transform,
    find_robot_ancestor,
    force_manipulator_refresh,
    get_site_candidates,
    read_joint_limits_native,
    solve_ik_from_cache,
)


class NamedPosePropertiesWidget(SimplePropertyWidget):
    """Property panel for IsaacNamedPose prims.

    Args:
        title: Panel title shown in the property window.
        collapsed: Whether the panel starts collapsed.
    """

    def __init__(self, title: str, collapsed: bool = False) -> None:
        super().__init__(title, collapsed)
        self._prim: Usd.Prim | None = None
        self._robot_prim: Usd.Prim | None = None
        self._prim_path: str = ""

        # Joint table state
        self._joint_drag_models: list[ui.SimpleFloatModel] = []
        self._joint_fixed_models: list[ui.SimpleBoolModel] = []
        self._joint_paths: list[str] = []
        self._joint_is_revolute: list[bool] = []
        self._joint_lock_images: list[ui.Image] = []
        self._joint_sliders: list[ui.FloatSlider] = []

        # Site combo references
        self._start_site_combo: SiteSearchComboBox | None = None
        self._end_site_combo: SiteSearchComboBox | None = None

        # Guard against recursive updates
        self._updating: bool = False

        # USD change listener
        self._usd_listener = None

        # Retained RobotPoser for the current pose's chain (direct apply / tracking init).
        self._poser: robot_poser.RobotPoser | None = None

        # Standalone tracking state
        self._is_tracking: bool = False
        self._tracking_cache: dict | None = None
        self._tracking_update_sub = None
        self._tracking_dirty: bool = False

        # Window tracking sync: set from Extension when the Robot Poser window
        # enables/disables tracking for the same named pose.
        self._window_is_tracking: bool = False
        # Optional callable set by Extension so the Track Target button in this
        # panel can also control the Robot Poser window's tracking.
        self._notify_window_tracking_fn = None
        # Optional callable set by Extension to query the window's current state
        # for the prim that is being loaded (used in on_new_payload).
        self._query_window_tracking_fn = None
        # Guard: prevents feedback loops when syncing tracking state.
        self._syncing_tracking: bool = False

        # IK failure outline state (uses shared helpers from fk_helpers)
        self._outline_active: bool = False
        self._outlined_gprim_paths: list[str] = []
        self._ik_failing: bool = False

        # Track Target button references
        self._track_btn_rect: ui.Rectangle | None = None
        self._track_btn_label: ui.Label | None = None
        self._track_btn_icon: ui.Image | None = None

    # ==================================================================
    # Lifecycle
    # ==================================================================

    def destroy(self) -> None:
        """Stop tracking, clear outline, and release resources."""
        self._stop_standalone_tracking()
        self._clear_outline()
        self._unregister_usd_listener()
        self._cleanup_combos()
        self._poser = None
        self._notify_window_tracking_fn = None
        self._query_window_tracking_fn = None

    def _rebuild_poser(self, stage: Usd.Stage) -> None:
        """Build a RobotPoser for the current pose's start/end chain and retain it.

        Called after a new prim is loaded or when the start/end links change.

        Args:
            stage: USD stage containing the pose and robot prims.
        """
        self._poser = None
        if not self._prim or not self._prim.IsValid() or not self._robot_prim:
            return
        start_rel = self._prim.GetRelationship(robot_schema.Relations.POSE_START_LINK.name)
        end_rel = self._prim.GetRelationship(robot_schema.Relations.POSE_END_LINK.name)
        start_targets = start_rel.GetTargets() if start_rel else []
        end_targets = end_rel.GetTargets() if end_rel else []
        if not start_targets or not end_targets:
            return
        start_prim = stage.GetPrimAtPath(start_targets[0])
        end_prim = stage.GetPrimAtPath(end_targets[0])
        if not start_prim or not start_prim.IsValid() or not end_prim or not end_prim.IsValid():
            return
        self._poser = robot_poser.RobotPoser(stage, self._robot_prim, start_prim, end_prim)

    def _ensure_poser(self, stage: Usd.Stage, pose: Any) -> None:
        """Build self._poser from resolved pose link data if it is not yet set.

        Uses the start/end link paths already resolved by get_named_pose so that
        the poser is built at the same time (and from the same data) as tracking
        cache, rather than at property-panel load time when the tree may not be ready.

        Args:
            stage: USD stage containing the robot and link prims.
            pose: Resolved pose result (e.g. from get_named_pose) with start_link/end_link.
        """
        if self._poser is not None:
            return
        if not self._robot_prim or not pose or not pose.success:
            return
        start_prim = stage.GetPrimAtPath(pose.start_link) if pose.start_link else None
        end_prim = stage.GetPrimAtPath(pose.end_link) if pose.end_link else None
        if not start_prim or not start_prim.IsValid() or not end_prim or not end_prim.IsValid():
            return
        self._poser = robot_poser.RobotPoser(stage, self._robot_prim, start_prim, end_prim)

    def _cleanup_combos(self) -> None:
        """Destroy and clear start/end site combo references."""
        if self._start_site_combo:
            self._start_site_combo.destroy()
            self._start_site_combo = None
        if self._end_site_combo:
            self._end_site_combo.destroy()
            self._end_site_combo = None

    # ==================================================================
    # Payload handling
    # ==================================================================

    def on_new_payload(self, payload: Any) -> bool:
        """Accept payload when it is a single IsaacNamedPose prim under a robot.

        Args:
            payload: Property window payload (paths and stage).

        Returns:
            True if the payload was accepted and the panel is bound.
        """
        if self._is_tracking:
            self._stop_standalone_tracking()
            self._is_tracking = False
        self._window_is_tracking = False
        self._clear_outline()
        self._unregister_usd_listener()

        if not super().on_new_payload(payload):
            return False
        if len(self._payload) != 1:
            return False

        prim_path = self._payload.get_paths()[0]
        stage = self._payload.get_stage()
        if not stage:
            return False

        prim = stage.GetPrimAtPath(prim_path)
        if not prim or not prim.IsValid():
            return False
        if prim.GetTypeName() != "IsaacNamedPose":
            return False

        self._prim = prim
        self._prim_path = str(prim_path)
        self._robot_prim = find_robot_ancestor(stage, prim)
        if not self._robot_prim:
            return False

        # Sync initial window-tracking state so the button and xformOp guard
        # are correct if the Robot Poser window was already tracking this pose.
        if self._query_window_tracking_fn is not None:
            self._window_is_tracking = self._query_window_tracking_fn(self._prim_path)

        self._rebuild_poser(stage)
        self._update_transform_from_stored_joints(stage)
        self._register_usd_listener()
        return True

    # ==================================================================
    # UI construction
    # ==================================================================

    def build_items(self) -> None:
        """Build the property panel UI (site combos, actions, joint table)."""
        if not self._prim or not self._robot_prim:
            return
        stage = self._payload.get_stage()
        if not stage:
            return

        self._cleanup_combos()
        style = get_property_style()

        with ui.VStack(spacing=6, style=style):
            self._build_site_section(stage)
            ui.Spacer(height=4)
            self._build_action_buttons()
            ui.Spacer(height=4)
            self._build_joint_frame(stage)

    # -- Robot label ---------------------------------------------------

    def _build_robot_label(self) -> None:
        """Build the robot name label in the property panel."""
        robot_path = str(self._robot_prim.GetPath()) if self._robot_prim else "None"
        robot_name = robot_path.rsplit("/", 1)[-1] if "/" in robot_path else robot_path
        with ui.HStack(height=LABEL_HEIGHT, spacing=HORIZONTAL_SPACING):
            ui.Label("Robot", width=LABEL_WIDTH, name="label")
            ui.Label(robot_name, tooltip=robot_path)

    # -- Start / end link combos ---------------------------------------

    def _build_site_section(self, stage: Usd.Stage) -> None:
        """Build start/end link combo boxes from site candidates.

        Args:
            stage: The USD stage.
        """
        site_candidates = get_site_candidates(stage, self._robot_prim) if self._robot_prim else []

        start_link = ""
        end_link = ""
        if self._prim is None:
            return
        start_rel = self._prim.GetRelationship(robot_schema.Relations.POSE_START_LINK.name)
        if start_rel and start_rel.GetTargets():
            start_link = str(start_rel.GetTargets()[0])
        end_rel = self._prim.GetRelationship(robot_schema.Relations.POSE_END_LINK.name)
        if end_rel and end_rel.GetTargets():
            end_link = str(end_rel.GetTargets()[0])

        with ui.HStack(height=22, spacing=HORIZONTAL_SPACING):
            ui.Label("Start Link", width=LABEL_WIDTH, name="label")
            self._start_site_combo = SiteSearchComboBox(
                items=site_candidates,
                current_value=start_link,
                on_selection_changed_fn=self._on_start_link_changed,
                identifier="poser_prop_start_link",
            )
        ui.Spacer(height=2)
        with ui.HStack(height=22, spacing=HORIZONTAL_SPACING):
            ui.Label("End Link", width=LABEL_WIDTH, name="label")
            self._end_site_combo = SiteSearchComboBox(
                items=site_candidates,
                current_value=end_link,
                on_selection_changed_fn=self._on_end_link_changed,
                identifier="poser_prop_end_link",
            )

    # -- Joint table inside a collapsable frame ------------------------

    def _build_joint_frame(self, stage: Usd.Stage) -> None:
        """Wrap the joint table in a collapsable frame.

        Args:
            stage: The USD stage.
        """
        with ui.CollapsableFrame(title="Joint Values", collapsed=False):
            with ui.VStack(spacing=2):
                self._build_joint_table(stage)

    def _build_joint_table(self, stage: Usd.Stage) -> None:
        """Build the joint value/fixed table from the prim's POSE_JOINTS and attributes.

        Args:
            stage: The USD stage.
        """
        self._joint_drag_models = []
        self._joint_fixed_models = []
        self._joint_paths = []
        self._joint_is_revolute = []
        self._joint_lock_images = []
        self._joint_sliders = []

        if not self._prim or not self._prim.IsValid():
            return

        joints_rel = self._prim.GetRelationship(robot_schema.Relations.POSE_JOINTS.name)
        joint_paths = [str(p) for p in joints_rel.GetTargets()] if joints_rel else []

        values_attr = self._prim.GetAttribute(robot_schema.Attributes.POSE_JOINT_VALUES.name)
        joint_values = list(values_attr.Get()) if values_attr and values_attr.Get() is not None else []

        fixed_attr = self._prim.GetAttribute(robot_schema.Attributes.POSE_JOINT_FIXED.name)
        fixed_flags = list(fixed_attr.Get()) if fixed_attr and fixed_attr.Get() is not None else []

        self._joint_paths = joint_paths

        if not joint_paths:
            ui.Label("No joints configured", height=20)
            return

        # Header with darker background
        with ui.ZStack(height=24):
            ui.Rectangle(name="table_header")
            with ui.HStack(height=24, spacing=4):
                ui.Spacer(width=2)
                ui.Label("Lock", width=40, alignment=ui.Alignment.CENTER, name="header")
                ui.Label("Joint", width=ui.Fraction(1), name="header")
                ui.Label("Value", width=ui.Fraction(1.5), name="header")
                ui.Spacer(width=2)

        # Rows
        for i, joint_path in enumerate(joint_paths):
            val = float(joint_values[i]) if i < len(joint_values) else 0.0
            fixed = bool(fixed_flags[i]) if i < len(fixed_flags) else False

            display_name, is_rev, lo, hi = read_joint_limits_native(stage, joint_path)
            self._joint_is_revolute.append(is_rev)

            fixed_model = ui.SimpleBoolModel(fixed)
            self._joint_fixed_models.append(fixed_model)

            drag_model = ui.SimpleFloatModel(val)
            self._joint_drag_models.append(drag_model)

            s_lo = lo if not np.isinf(lo) else (-360.0 if is_rev else -10.0)
            s_hi = hi if not np.isinf(hi) else (360.0 if is_rev else 10.0)

            with ui.HStack(height=22, spacing=4):
                ui.Spacer(width=2)
                with ui.HStack(width=40):
                    ui.Spacer()
                    with ui.ZStack(width=18, height=18):
                        lock_img = ui.Image(
                            name="lock_closed" if fixed else "lock_open",
                            width=18,
                            height=18,
                        )
                        idx = i
                        ui.InvisibleButton(width=18, height=18).set_mouse_pressed_fn(
                            lambda x, y, b, m, _idx=idx: self._on_lock_clicked(_idx) if b == 0 else None
                        )
                    ui.Spacer()
                self._joint_lock_images.append(lock_img)

                ui.Label(
                    display_name,
                    width=ui.Fraction(1),
                    tooltip=joint_path,
                    elided_text=True,
                )

                slider = ui.FloatSlider(
                    model=drag_model,
                    min=s_lo,
                    max=s_hi,
                    step=0.5 if is_rev else 0.001,
                    width=ui.Fraction(1.5),
                    enabled=not fixed,
                    identifier=f"poser_joint_slider_{i}",
                )
                self._joint_sliders.append(slider)
                ui.Spacer(width=2)

            idx = i
            drag_model.add_value_changed_fn(lambda m, _idx=idx: self._on_joint_value_changed(_idx))
            fixed_model.add_value_changed_fn(lambda m, _idx=idx: self._on_joint_fixed_changed(_idx))

    # -- Track button appearance helpers --------------------------------

    def _update_track_btn_appearance(self, active: bool) -> None:
        """Update all Track Target button widgets to reflect the active state.

        Args:
            active: True when tracking (standalone or via Robot Poser window).
        """
        if self._track_btn_rect:
            self._track_btn_rect.name = "track_active" if active else "track_inactive"
        if self._track_btn_icon:
            self._track_btn_icon.name = "target_active" if active else "target"
        if self._track_btn_label:
            self._track_btn_label.text = "Tracking" if active else "Track Target"
            self._track_btn_label.name = "track_active" if active else "track"

    # -- Public API called by Extension ---------------------------------

    def set_window_tracking(self, prim_path: str, enabled: bool) -> None:
        """Called by Extension when the Robot Poser window toggles tracking.

        Keeps the Track Target button in sync and stops conflicting standalone
        tracking when the window takes over.

        Args:
            prim_path: The named pose prim path whose tracking changed.
            enabled: True if the window started tracking, False if stopped.
        """
        if prim_path != self._prim_path or self._syncing_tracking:
            return
        self._window_is_tracking = enabled
        if enabled and self._is_tracking:
            # Window started tracking the same pose; stop standalone to avoid conflict.
            self._is_tracking = False
            self._stop_standalone_tracking()
        self._update_track_btn_appearance(self._is_tracking or self._window_is_tracking)

    # -- Action buttons ------------------------------------------------

    def _build_action_buttons(self) -> None:
        """Build Set Robot to Pose and Track Target buttons."""
        with ui.HStack(height=28, spacing=8):
            # Set Robot to Pose -- custom ZStack button with play icon
            with ui.ZStack(width=ui.Fraction(1), height=28):
                ui.Rectangle(name="set_pose")
                with ui.HStack(spacing=6):
                    ui.Spacer()
                    with ui.VStack(width=0):
                        ui.Spacer()
                        ui.Image(name="play", width=14, height=14)
                        ui.Spacer()
                    ui.Label("Set Robot to Pose", name="set_pose", width=0)
                    ui.Spacer()
                ui.InvisibleButton(height=28, identifier="poser_set_robot_to_pose").set_mouse_pressed_fn(
                    lambda x, y, b, m: self._on_set_robot_to_pose() if b == 0 else None
                )

            # Track Target -- custom ZStack button with target icon
            with ui.ZStack(width=ui.Fraction(1), height=28):
                active = self._is_tracking or self._window_is_tracking
                self._track_btn_rect = ui.Rectangle(name="track_active" if active else "track_inactive")
                with ui.HStack(spacing=6):
                    ui.Spacer()
                    with ui.VStack(width=0):
                        ui.Spacer()
                        self._track_btn_icon = ui.Image(
                            name="target_active" if active else "target",
                            width=14,
                            height=14,
                        )
                        ui.Spacer()
                    self._track_btn_label = ui.Label(
                        "Tracking" if active else "Track Target",
                        name="track_active" if active else "track",
                        width=0,
                    )
                    ui.Spacer()
                ui.InvisibleButton(height=28, identifier="poser_track_target_prop").set_mouse_pressed_fn(
                    lambda x, y, b, m: self._on_track_target_toggled() if b == 0 else None
                )

    # ==================================================================
    # Joint value / fixed change handlers
    # ==================================================================

    def _on_joint_value_changed(self, index: int) -> None:
        """Write slider value to USD and optionally update FK and robot pose.

        Args:
            index: Index of the joint in the table.
        """
        if self._updating:
            return
        if not self._prim or not self._prim.IsValid():
            return

        stage = omni.usd.get_context().get_stage()
        if not stage:
            return

        values_attr = self._prim.GetAttribute(robot_schema.Attributes.POSE_JOINT_VALUES.name)
        if not values_attr:
            return
        current_values = list(values_attr.Get()) if values_attr.Get() is not None else []

        new_val = self._joint_drag_models[index].get_value_as_float()
        while len(current_values) <= index:
            current_values.append(0.0)
        current_values[index] = new_val

        self._updating = True
        try:
            with Sdf.ChangeBlock():
                values_attr.Set(current_values)
                pose_name = self._prim.GetName()
                pose = robot_poser.get_named_pose(stage, self._robot_prim, pose_name)
                if pose and pose.success:
                    compute_fk_and_write_transform(stage, self._robot_prim, self._prim_path, pose)
                    if self._is_tracking and pose.joints and self._poser is not None:
                        self._poser.apply_pose(pose.joints)
        finally:
            self._updating = False

    def _on_lock_clicked(self, index: int) -> None:
        """Toggle the fixed state for a joint when its lock icon is clicked.

        Args:
            index: Index of the joint in the table.
        """
        if index >= len(self._joint_fixed_models):
            return
        current = self._joint_fixed_models[index].get_value_as_bool()
        self._joint_fixed_models[index].set_value(not current)

    def _on_joint_fixed_changed(self, index: int) -> None:
        """Write fixed state to USD and refresh lock icon / slider enabled state.

        Args:
            index: Index of the joint in the table.
        """
        is_fixed = self._joint_fixed_models[index].get_value_as_bool()

        # Always update visuals, even when called during a sync (_updating=True).
        if index < len(self._joint_lock_images):
            img = self._joint_lock_images[index]
            if img:
                img.name = "lock_closed" if is_fixed else "lock_open"
        if index < len(self._joint_sliders):
            sl = self._joint_sliders[index]
            if sl:
                sl.enabled = not is_fixed

        if self._updating:
            return
        if not self._prim or not self._prim.IsValid():
            return

        fixed_attr = self._prim.GetAttribute(robot_schema.Attributes.POSE_JOINT_FIXED.name)
        if not fixed_attr:
            return
        current_fixed = list(fixed_attr.Get()) if fixed_attr.Get() is not None else []

        while len(current_fixed) <= index:
            current_fixed.append(False)
        current_fixed[index] = is_fixed

        self._updating = True
        try:
            fixed_attr.Set(current_fixed)
        finally:
            self._updating = False

    # ==================================================================
    # USD change listener
    # ==================================================================

    def _register_usd_listener(self) -> None:
        """Register a USD ObjectsChanged notice on the current stage."""
        if self._usd_listener is not None:
            return
        stage = omni.usd.get_context().get_stage()
        if not stage:
            return
        self._usd_listener = Tf.Notice.Register(
            Usd.Notice.ObjectsChanged,
            self._on_usd_objects_changed,
            stage,
        )

    def _unregister_usd_listener(self) -> None:
        """Revoke the USD ObjectsChanged notice listener."""
        if self._usd_listener is not None:
            self._usd_listener.Revoke()
            self._usd_listener = None

    def _on_usd_objects_changed(self, notice: Any, sender: Any) -> None:
        """Handle USD ObjectsChanged: sync sliders from prim or mark tracking dirty.

        Args:
            notice: The ObjectsChanged notice.
            sender: The stage that sent the notice.
        """
        if self._updating:
            return

        prim_path = self._prim_path
        need_sync = False

        for p in notice.GetChangedInfoOnlyPaths():
            changed_prim = str(p.GetPrimPath())
            if changed_prim != prim_path:
                continue
            full_path = str(p)

            if (
                robot_schema.Attributes.POSE_JOINT_VALUES.name in full_path
                or robot_schema.Attributes.POSE_JOINT_FIXED.name in full_path
            ):
                need_sync = True
            if "xformOp" in full_path and self._is_tracking:
                self._tracking_dirty = True

        if need_sync:
            self._sync_drag_models_from_usd()

    def _sync_drag_models_from_usd(self) -> None:
        """Push current prim attribute values into the slider/checkbox models."""
        if not self._prim or not self._prim.IsValid():
            return

        self._updating = True
        try:
            values_attr = self._prim.GetAttribute(robot_schema.Attributes.POSE_JOINT_VALUES.name)
            if values_attr and values_attr.Get() is not None:
                joint_values = list(values_attr.Get())
                for i, model in enumerate(self._joint_drag_models):
                    val = float(joint_values[i]) if i < len(joint_values) else 0.0
                    if abs(model.get_value_as_float() - val) > 1e-6:
                        model.set_value(val)

            fixed_attr = self._prim.GetAttribute(robot_schema.Attributes.POSE_JOINT_FIXED.name)
            if fixed_attr and fixed_attr.Get() is not None:
                fixed_flags = list(fixed_attr.Get())
                for i, model in enumerate(self._joint_fixed_models):
                    flag = bool(fixed_flags[i]) if i < len(fixed_flags) else False
                    if model.get_value_as_bool() != flag:
                        model.set_value(flag)
        finally:
            self._updating = False

    # ==================================================================
    # Site change handlers
    # ==================================================================

    def _on_start_link_changed(self, new_path: str) -> None:
        """Handle start link combo selection. Forwards to _on_site_changed.

        Args:
            new_path: Selected start link/site path.
        """
        self._on_site_changed(new_path, is_start=True)

    def _on_end_link_changed(self, new_path: str) -> None:
        """Handle end link combo selection. Forwards to _on_site_changed.

        Args:
            new_path: Selected end link/site path.
        """
        self._on_site_changed(new_path, is_start=False)

    def _on_site_changed(self, new_path: str, *, is_start: bool) -> None:
        """Update start or end link relationship and rebuild pose from current joints.

        Args:
            new_path: New link/site path.
            is_start: True to set start link, False for end link.
        """
        if self._updating:
            return
        if not self._prim or not self._prim.IsValid() or not self._robot_prim:
            return

        stage = omni.usd.get_context().get_stage()
        if not stage:
            return

        start_rel = self._prim.GetRelationship(robot_schema.Relations.POSE_START_LINK.name)
        end_rel = self._prim.GetRelationship(robot_schema.Relations.POSE_END_LINK.name)
        start_link = str(start_rel.GetTargets()[0]) if start_rel and start_rel.GetTargets() else ""
        end_link = str(end_rel.GetTargets()[0]) if end_rel and end_rel.GetTargets() else ""

        if is_start:
            start_link = new_path
        else:
            end_link = new_path

        if not start_link or not end_link:
            return

        result = build_pose_from_current_joints(stage, self._robot_prim, start_link, end_link)
        if result is None:
            return

        was_tracking = self._is_tracking
        if was_tracking:
            self._stop_standalone_tracking()

        self._updating = True
        try:
            robot_poser.store_named_pose(stage, self._robot_prim, self._prim.GetName(), result)
        finally:
            self._updating = False

        self._rebuild_poser(stage)
        self._request_refresh()

    # ==================================================================
    # Action button handlers
    # ==================================================================

    def _on_set_robot_to_pose(self) -> None:
        """Apply the stored named pose joint state to the robot."""
        if not self._prim or not self._robot_prim:
            return
        stage = omni.usd.get_context().get_stage()
        if not stage:
            return
        pose_name = self._prim.GetName()
        pose = robot_poser.get_named_pose(stage, self._robot_prim, pose_name)
        if pose and pose.success and pose.joints:
            self._ensure_poser(stage, pose)
            if self._poser is not None:
                self._poser.apply_pose(pose.joints)
                force_manipulator_refresh()

    def _on_track_target_toggled(self) -> None:
        """Toggle tracking on or off and update button appearance.

        When wired to the Robot Poser window via ``_notify_window_tracking_fn``,
        the button controls the window's tracking and falls back to standalone
        tracking only when the callback is not available.  Prevents feedback
        loops via ``_syncing_tracking``.
        """
        if not self._prim or self._syncing_tracking:
            return

        combined_active = self._is_tracking or self._window_is_tracking
        enable = not combined_active

        self._syncing_tracking = True
        try:
            if not enable:
                # --- Stop all tracking ---
                if self._is_tracking:
                    self._is_tracking = False
                    self._stop_standalone_tracking()
                if self._window_is_tracking and self._notify_window_tracking_fn is not None:
                    self._notify_window_tracking_fn(self._prim_path, False)
                    self._window_is_tracking = False
            else:
                # --- Start tracking ---
                delegated = False
                if self._notify_window_tracking_fn is not None:
                    delegated = bool(self._notify_window_tracking_fn(self._prim_path, True))
                    if delegated:
                        self._window_is_tracking = True
                if not delegated:
                    self._is_tracking = True
                    self._start_standalone_tracking()
        finally:
            self._syncing_tracking = False

        self._update_track_btn_appearance(self._is_tracking or self._window_is_tracking)

    # ==================================================================
    # Standalone tracking  (uses shared helpers from fk_helpers)
    # ==================================================================

    def _start_standalone_tracking(self) -> None:
        """Build tracking cache and subscribe to per-frame updates for IK solving."""
        stage = omni.usd.get_context().get_stage()
        if not stage or not self._prim or not self._robot_prim:
            self._is_tracking = False
            return

        cache = build_tracking_cache(stage, self._robot_prim, self._prim_path)
        if cache is None:
            carb.log_warn("Named Pose Properties: could not build tracking cache")
            self._is_tracking = False
            return
        self._tracking_cache = cache
        self._tracking_dirty = False

        pose_name = self._prim.GetName()
        pose = robot_poser.get_named_pose(stage, self._robot_prim, pose_name)
        if pose and pose.success and pose.joints:
            self._ensure_poser(stage, pose)
            if self._poser is not None:
                self._poser.apply_pose(pose.joints)
                force_manipulator_refresh()

        if self._tracking_update_sub is None:
            self._tracking_update_sub = (
                omni.kit.app.get_app().get_update_event_stream().create_subscription_to_pop(self._on_tracking_update)
            )

    def _stop_standalone_tracking(self) -> None:
        """Cancel the update subscription and clear tracking cache and outline."""
        self._tracking_update_sub = None
        self._tracking_cache = None
        self._tracking_dirty = False
        self._ik_failing = False
        self._clear_outline()

    def _on_tracking_update(self, event: Any) -> None:
        """Per-frame callback: solve IK when tracking is dirty and update prim/joints.

        Args:
            event: Update event (unused).
        """
        if not self._tracking_dirty or self._updating:
            return
        if not self._is_tracking or self._tracking_cache is None:
            return

        self._tracking_dirty = False

        stage = omni.usd.get_context().get_stage()
        if not stage or not self._robot_prim or not self._robot_prim.IsValid():
            return

        self._solve_tracking_ik(stage)

    def _solve_tracking_ik(self, stage: Usd.Stage) -> None:
        """Run IK from cache; on success write joints and sync UI; on failure apply outline.

        Args:
            stage: The USD stage.
        """
        cache = self._tracking_cache
        if not cache:
            return

        result = solve_ik_from_cache(cache)

        if result is None:
            # IK failed -- show red outline
            if not self._ik_failing:
                self._ik_failing = True
                self._outlined_gprim_paths = apply_ik_chain_outline(stage, [cache], self._outlined_gprim_paths)
                self._outline_active = bool(self._outlined_gprim_paths)
            return

        # IK succeeded
        if self._ik_failing:
            self._ik_failing = False
            self._clear_outline()

        result_joints, joint_values_native = result

        self._updating = True
        try:
            with Sdf.ChangeBlock():
                cache["poser"].apply_pose(result_joints)
                pose_prim = cache["pose_prim"]
                jv_attr = pose_prim.GetAttribute(robot_schema.Attributes.POSE_JOINT_VALUES.name)
                if jv_attr:
                    jv_attr.Set(joint_values_native)
        finally:
            self._updating = False

        self._sync_drag_models_from_usd()

    # ==================================================================
    # IK failure outline  (delegates to shared helpers in fk_helpers)
    # ==================================================================

    def _clear_outline(self) -> None:
        """Remove the IK failure red outline from previously outlined prims."""
        if not self._outline_active:
            return
        self._outlined_gprim_paths = clear_ik_chain_outline(self._outlined_gprim_paths)
        self._outline_active = False

    # ==================================================================
    # Helpers
    # ==================================================================

    def _update_transform_from_stored_joints(self, stage: Usd.Stage) -> None:
        """Compute FK from stored joint values and write the result to the named pose prim.

        Args:
            stage: The USD stage.
        """
        if not self._prim or not self._robot_prim:
            return
        pose_name = self._prim.GetName()
        pose = robot_poser.get_named_pose(stage, self._robot_prim, pose_name)
        if not pose or not pose.success:
            return
        self._updating = True
        try:
            compute_fk_and_write_transform(stage, self._robot_prim, self._prim_path, pose)
        finally:
            self._updating = False

    def _request_refresh(self) -> None:
        """Request a rebuild of the property window frame so the panel updates."""
        try:
            import omni.kit.window.property

            w = omni.kit.window.property.get_window()
            if w and hasattr(w, "_window") and w._window:  # noqa: SLF001
                w._window.frame.rebuild()  # noqa: SLF001
        except Exception:
            pass
