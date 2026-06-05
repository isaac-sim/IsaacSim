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

"""Main Robot Poser window UI: robot selection, named poses table, and tracking."""

from __future__ import annotations

import asyncio
from collections.abc import Callable
from typing import Any

import carb
import isaacsim.robot.poser.robot_poser as robot_poser
import omni.timeline
import omni.ui as ui
import omni.usd
from isaacsim.gui.components.element_wrappers import CollapsableFrame, DropDown
from pxr import Sdf, Tf, Usd
from usd.schema.isaac import robot_schema

from ..style import get_style
from ..utils.fk_helpers import (
    apply_ik_chain_outline,
    build_pose_from_current_joints,
    build_tracking_cache,
    clear_ik_chain_outline,
    compute_fk_and_write_transform,
    force_manipulator_refresh,
    get_site_candidates,
    solve_ik_from_cache,
)
from .named_pose_table import NamedPoseItem, NamedPosesTable

NAMED_POSES_SCOPE = robot_poser.NAMED_POSES_SCOPE


class UIBuilder:
    """Build and manage the Robot Poser extension UI."""

    def __init__(self) -> None:
        self._timeline = omni.timeline.get_timeline_interface()

        self._robot_prim_path: str | None = None
        self._robot_prim: Usd.Prim | None = None

        self._wrapped_ui_elements: list = []
        self._site_candidates: list[str] = []

        self._selection_dropdown: DropDown | None = None
        self._named_poses_table: NamedPosesTable | None = None

        # Track Target state
        self._tracked_paths: set = set()
        self._dirty_tracked_paths: set = set()
        self._tracking_cache: dict[str, dict] = {}
        self._poser_cache: dict[str, robot_poser.RobotPoser] = {}
        self._usd_listener = None
        self._updating_transform: bool = False

        # Remembered add-row site selections (preserved across refreshes)
        self._last_add_start_site: str = ""
        self._last_add_end_site: str = ""

        # IK failure outline state
        self._outline_active: bool = False
        self._outlined_gprim_paths: list = []
        self._ik_fail_paths: set = set()

        # Guard flag for prim renames triggered from the table.
        self._renaming_prim: bool = False
        self._refresh_pending: bool = False

        # Guard flag to suppress _on_named_pose_selected when programmatically
        # restoring the table row selection after a model refresh.
        self._restoring_selection: bool = False

        # Callbacks set by Extension to manage the per-frame update subscription.
        # Only active while at least one Track Target is enabled.
        self._request_update_subscription = lambda: None
        self._release_update_subscription = lambda: None

        # Optional callback: called with (prim_path, enabled) whenever any named
        # pose's tracking state is toggled.  Wired by Extension to the property panel.
        self._on_tracking_state_changed_fn: Callable[[str, bool], None] | None = None

    # ###################################################################
    # Called by extension.py
    # ###################################################################

    def on_menu_callback(self) -> None:
        """Refresh dropdown and named poses when window is toggled from menu."""
        self._refresh_articulation_dropdown()
        self._refresh_named_poses()

    def on_timeline_event(self, event: Any) -> None:
        """Refresh articulation dropdown when timeline play state changes.

        Args:
            event: Timeline event payload.
        """
        if self._timeline.is_playing():
            self._refresh_articulation_dropdown()

    def on_assets_loaded(self) -> None:
        """Refresh articulation dropdown when stage assets are loaded."""
        # Only refresh the articulation dropdown so new robots appear.
        # Named poses are refreshed when the user selects a robot.
        self._refresh_articulation_dropdown()

    def on_simulation_stop_play(self) -> None:
        """Refresh dropdown and named poses when simulation stops."""
        if self._timeline.is_stopped():
            self._refresh_articulation_dropdown()
            self._refresh_named_poses()

    def on_update(self, dt: float) -> None:
        """Per-frame update: solve IK for dirty tracked poses.

        Args:
            dt: Elapsed time since last update (unused).
        """
        if not self._dirty_tracked_paths or self._updating_transform:
            return
        if self._robot_prim is None:
            return
        stage = omni.usd.get_context().get_stage()
        if stage is None:
            return

        dirty = self._dirty_tracked_paths.copy()
        self._dirty_tracked_paths.clear()

        for prim_path in dirty:
            if prim_path not in self._tracked_paths:
                continue
            item = self._find_tracked_item(prim_path)
            if item is not None:
                self._solve_ik_from_target(stage, item)

    def _find_tracked_item(self, prim_path: str) -> NamedPoseItem | None:
        """Return the NamedPoseItem with the given prim path, or None.

        Args:
            prim_path: USD path of the named pose prim.

        Returns:
            The matching item or None.
        """
        if self._named_poses_table is None:
            return None
        for item in self._named_poses_table.get_items():
            if isinstance(item, NamedPoseItem) and item.prim_path == prim_path:
                return item
        return None

    def cleanup(self) -> None:
        """Clear IK outline, release wrapped UI elements, and reset state."""
        self._clear_ik_fail_outline()
        for elem in self._wrapped_ui_elements:
            elem.cleanup()
        self._wrapped_ui_elements = []
        self._robot_prim = None
        self._robot_prim_path = None
        self._site_candidates = []
        self._tracked_paths = set()
        self._dirty_tracked_paths = set()
        self._tracking_cache = {}
        self._poser_cache = {}
        if hasattr(self, "_unregister_usd_listener"):
            self._unregister_usd_listener()
        self._release_update_subscription()

    # ###################################################################
    # UI Construction
    # ###################################################################

    def _make_heading(self, heading_title: str, width: int | float = 0) -> None:
        """Build a heading row with title and horizontal line.

        Args:
            heading_title: Label text.
            width: Optional width for the label (0 for auto).
        """
        with ui.HStack():
            ui.Label(heading_title, width=width, name="sub_title")
            ui.Spacer(width=5)
            ui.Line(width=ui.Fraction(1.0))
            ui.Spacer(width=5)

    def _make_info_display(self, info_text: str) -> None:
        """Build an info row with icon and wrapped text.

        Args:
            info_text: Description text to show.
        """
        with ui.HStack(style=get_style()):
            with ui.VStack(width=20):
                ui.Spacer()
                ui.Image(name="info", width=20, height=20)
                ui.Spacer()
            ui.Spacer(width=12)
            with ui.VStack():
                ui.Spacer()
                ui.Label(info_text, name="info", width=ui.Fraction(1.0), height=10, word_wrap=True)
                ui.Spacer()
            ui.Spacer(width=25)

    def _make_info_heading(self, heading_title: str, info_text: str) -> None:
        """Build a heading plus info block.

        Args:
            heading_title: Heading label.
            info_text: Info description below the heading.
        """
        self._make_heading(heading_title)
        self._make_info_display(info_text)

    def build_ui(self) -> None:
        """Build the main Robot Poser window content."""
        with ui.VStack(style=get_style(), spacing=8):
            self._build_selection_frame()
            self._build_named_poses_frame()

    def _build_selection_frame(self) -> None:
        """Build the collapsable Active Robot frame and register it for cleanup."""
        frame = CollapsableFrame("Active Robot", collapsed=False, build_fn=self._build_selection_frame_contents)
        self._wrapped_ui_elements.append(frame)

    def _build_selection_frame_contents(self) -> None:
        """Build the contents of the Active Robot frame (heading, dropdown)."""
        with ui.HStack(style=get_style(), spacing=6, height=0):
            ui.Spacer(width=16)
            with ui.VStack(style=get_style(), spacing=12, height=0):
                heading_title = "1. Select Active Robot"
                heading_text = (
                    "Select the robot to pose from the dropdown. The robot must have "
                    "the IsaacRobotAPI schema applied."
                )
                self._make_info_heading(heading_title, heading_text)

                self._selection_dropdown = DropDown(
                    "Set Active Robot",
                    tooltip="Select the robot articulation to manage named poses for.",
                    populate_fn=self._populate_articulations,
                    on_selection_fn=self._on_select_articulation,
                    keep_old_selections=True,
                )
                self._selection_dropdown.repopulate()
                self._wrapped_ui_elements.append(self._selection_dropdown)

    def _build_named_poses_frame(self) -> None:
        """Build the collapsable Named Poses frame with build_fn for contents."""
        ui.CollapsableFrame(
            title="Named Poses",
            name="Named Poses",
            collapsed=False,
            build_fn=self._build_named_poses_contents,
            style=get_style(),
            style_type_name_override="CollapsableFrame",
            horizontal_scrollbar_policy=ui.ScrollBarPolicy.SCROLLBAR_ALWAYS_OFF,
            vertical_scrollbar_policy=ui.ScrollBarPolicy.SCROLLBAR_ALWAYS_OFF,
            identifier="poser_named_poses_frame",
        )

    def _build_named_poses_contents(self) -> None:
        """Build the Named Poses heading and table and wire selection/refresh."""
        with ui.VStack(style=get_style(), spacing=6, height=ui.Fraction(1)):
            with ui.HStack(style=get_style(), spacing=6, height=0):
                ui.Spacer(width=16)
                with ui.VStack(style=get_style(), spacing=12, height=0):
                    heading_title = "2. Named Poses"
                    heading_text = "View and manage named poses for the selected robot."
                    self._make_info_heading(heading_title, heading_text)

            self._named_poses_table = NamedPosesTable(
                on_selection_fn=self._on_named_pose_selected,
                on_track_target_fn=self._on_track_target_toggled,
                on_add_fn=self._on_add_named_pose,
                on_remove_fn=self._on_remove_named_pose,
                on_name_changed_fn=self._on_named_pose_name_changed,
                on_apply_pose_fn=self._on_apply_named_pose_from_table,
                on_site_changed_fn=self._on_named_pose_site_changed,
                visible=True,
            )
            if self._selection_dropdown is not None:
                items = self._selection_dropdown.get_items()
                if len(items) > 1:
                    self._selection_dropdown.set_selection(items[1])
                    self._refresh_site_candidates()
                    self._schedule_named_poses_refresh()

    # ###################################################################
    # Selection / Refresh helpers
    # ###################################################################

    def _populate_articulations(self) -> list[str]:
        """Return a list of robot prim paths (prims with IsaacRobotAPI) with 'None' prepended.

        Returns:
            List of path strings for the dropdown.
        """
        stage = omni.usd.get_context().get_stage()
        if stage is None:
            return ["None"]
        robot_api_name = robot_schema.Classes.ROBOT_API.value
        items = [
            str(prim.GetPath())
            for prim in Usd.PrimRange(stage.GetPseudoRoot())
            if prim.HasAPI(robot_api_name) and not prim.IsInPrototype()
        ]
        items.insert(0, "None")
        return items

    def _refresh_articulation_dropdown(self) -> None:
        """Repopulate the Active Robot dropdown from the stage."""
        if self._selection_dropdown is None:
            return
        self._selection_dropdown.repopulate()

    def _on_select_articulation(self, articulation_path: str) -> None:
        """Handle Active Robot selection: set robot prim, refresh sites and named poses.

        Args:
            articulation_path: Selected robot prim path or 'None'.
        """
        self._clear_ik_fail_outline()
        if articulation_path == "None" or not articulation_path:
            self._robot_prim = None
            self._robot_prim_path = None
            self._site_candidates = []
            self._tracked_paths = set()
            self._dirty_tracked_paths = set()
            self._tracking_cache = {}
            self._unregister_usd_listener()
            self._release_update_subscription()
            self._refresh_named_poses()
            return

        stage = omni.usd.get_context().get_stage()
        if stage is None:
            return

        prim = stage.GetPrimAtPath(articulation_path)
        if not prim or not prim.IsValid():
            return

        if not robot_poser.validate_robot_schema(prim):
            carb.log_warn(f"Robot Poser: selected prim does not have IsaacRobotAPI: {articulation_path}")

        self._robot_prim_path = articulation_path
        self._robot_prim = prim
        self._tracked_paths = set()
        self._dirty_tracked_paths = set()
        self._tracking_cache = {}
        self._poser_cache = {}
        self._unregister_usd_listener()
        self._release_update_subscription()
        self._refresh_site_candidates()
        self._refresh_named_poses()
        self._register_usd_listener()

    def _refresh_site_candidates(self) -> None:
        """Refresh site list from the active robot and update the table's site combos."""
        self._site_candidates = []
        if self._robot_prim is None or not self._robot_prim.IsValid():
            return

        stage = omni.usd.get_context().get_stage()
        if stage is None:
            return

        self._site_candidates = get_site_candidates(stage, self._robot_prim)

        if self._named_poses_table:
            self._named_poses_table.update_start_site(self._site_candidates)
            self._named_poses_table.update_end_site(self._site_candidates)

    def _refresh_named_poses(self) -> None:
        """Reload named poses from USD and set table items; preserve tracking state."""
        if self._named_poses_table is None:
            return

        self._refresh_site_candidates()
        if self._robot_prim is None or not self._robot_prim.IsValid():
            self._named_poses_table.set_items([])
            self._update_add_row_defaults([])
            return

        stage = omni.usd.get_context().get_stage()
        if stage is None:
            self._named_poses_table.set_items([])
            self._update_add_row_defaults([])
            return

        # Preserve tracking state
        tracked_paths: set = set()
        for item in self._named_poses_table.get_items():
            if isinstance(item, NamedPoseItem) and item.tracking:
                tracked_paths.add(item.prim_path)

        names = robot_poser.list_named_poses(stage, self._robot_prim)
        items: list[NamedPoseItem] = []
        robot_path = str(self._robot_prim.GetPath())

        for name in names:
            pose = robot_poser.get_named_pose(stage, self._robot_prim, name)
            prim_path = f"{robot_path}/{NAMED_POSES_SCOPE}/{name}"
            start_site = pose.start_link if pose else ""
            end_site = pose.end_link if pose else ""
            item = NamedPoseItem(
                name=name,
                start_site=start_site,
                end_site=end_site,
                prim_path=prim_path,
            )
            if prim_path in tracked_paths:
                item.tracking = True
            items.append(item)

        self._named_poses_table.set_items(items)
        self._update_add_row_defaults(items)
        self._restore_table_selection(items)

    def _update_add_row_defaults(self, items: list[NamedPoseItem]) -> None:
        """Set the add-row site defaults based on existing items or site candidates.

        If there are existing named poses, use the last pose's start/end sites.
        Otherwise fall back to root link / last link.

        Args:
            items: Current list of named pose items (may be empty).
        """
        start_path = self._last_add_start_site
        end_path = self._last_add_end_site

        # If there are existing named poses, default to the last one's sites
        if items:
            last_item = items[-1]
            if not start_path:
                start_path = last_item.start_site.get_value_as_string()
            if not end_path:
                end_path = last_item.end_site.get_value_as_string()

        # Fallback to first/last site candidate
        if not start_path and self._site_candidates:
            start_path = self._site_candidates[0]
        if not end_path and self._site_candidates:
            end_path = self._site_candidates[-1]

        if self._named_poses_table is not None:
            self._named_poses_table.set_add_row_defaults(start_path or "", end_path or "")

    # ###################################################################
    # Named Pose Actions
    # ###################################################################

    def _get_current_stage_named_pose_selection(self) -> str:
        """Return the stage-selected named-pose prim path if it belongs to the active robot.

        Returns:
            The prim path string, or empty string if none is selected.
        """
        if not self._robot_prim_path:
            return ""
        paths = omni.usd.get_context().get_selection().get_selected_prim_paths()
        if not paths:
            return ""
        path = paths[0]
        prefix = f"{self._robot_prim_path}/{NAMED_POSES_SCOPE}/"
        return path if path.startswith(prefix) else ""

    def _restore_table_selection(self, items: list) -> None:
        """Re-select the table row whose prim path matches the current stage selection.

        Called after the table model is rebuilt so that USD-triggered refreshes do not
        silently drop the user's row selection.  The selection callback is suppressed to
        avoid re-running FK computation or scheduling another refresh.

        Args:
            items: The freshly built list of NamedPoseItems now in the table.
        """
        if self._named_poses_table is None:
            return
        selected_path = self._get_current_stage_named_pose_selection()
        if not selected_path:
            return
        for item in items:
            if isinstance(item, NamedPoseItem) and item.prim_path == selected_path:
                self._restoring_selection = True
                try:
                    self._named_poses_table.select_item(item)
                finally:
                    self._restoring_selection = False
                break

    def _on_named_pose_selected(self, item: NamedPoseItem) -> None:
        """Select the prim, compute FK for the pose, and optionally apply joints if tracking.

        Args:
            item: The selected named pose item.
        """
        if self._restoring_selection:
            return

        self._clear_ik_fail_outline()

        if not item.prim_path or self._robot_prim is None:
            return

        stage = omni.usd.get_context().get_stage()
        if stage is None:
            return

        usd_context = omni.usd.get_context()
        usd_context.get_selection().set_selected_prim_paths([item.prim_path], True)

        pose_name = item.name.get_value_as_string()
        pose = robot_poser.get_named_pose(stage, self._robot_prim, pose_name)
        if pose is None or not pose.success:
            return

        # Compute FK and update the named pose prim only (do NOT move the robot).
        self._compute_fk_and_update_named_pose(stage, item.prim_path, pose)

        # If this named pose tracker is active, update robot pose
        if item.tracking:
            self._apply_named_pose_joints(item)

    def _compute_fk_and_update_named_pose(
        self, stage: Usd.Stage, pose_prim_path: str, pose: robot_poser.PoseResult
    ) -> None:
        """Compute FK for the stored joint values and update the named pose prim.

        Does NOT apply joint state to the robot.

        Args:
            stage: The USD stage.
            pose_prim_path: Path of the IsaacNamedPose prim.
            pose: Pose result with joint values and start/end links.
        """
        self._updating_transform = True
        try:
            compute_fk_and_write_transform(stage, self._robot_prim, pose_prim_path, pose)
        finally:
            self._updating_transform = False

    def _build_tracking_cache(self, stage: Usd.Stage, item: NamedPoseItem) -> dict | None:
        """Build and store per-item IK cache when tracking is enabled.

        Args:
            stage: The USD stage.
            item: The named pose item to build cache for.

        Returns:
            The cache dict (pose_prim, poser) or None if build failed.
        """
        cache = build_tracking_cache(stage, self._robot_prim, item.prim_path)
        if cache is not None:
            self._tracking_cache[item.prim_path] = cache
        return cache

    def _get_or_build_poser(self, stage: Usd.Stage, item: NamedPoseItem) -> robot_poser.RobotPoser | None:
        """Return a retained RobotPoser for item's chain, building one if necessary.

        Uses the tracking cache's poser when available so the IK seed is shared.
        Falls back to a standalone cached poser, creating it on first use.

        Args:
            stage: The USD stage.
            item: The named pose item whose chain is needed.

        Returns:
            A RobotPoser, or None when the chain cannot be built.
        """
        # Prefer the tracking cache's poser (has IK seed).
        tracking = self._tracking_cache.get(item.prim_path)
        if tracking is not None:
            return tracking["poser"]

        if item.prim_path in self._poser_cache:
            return self._poser_cache[item.prim_path]

        if self._robot_prim is None:
            return None
        start_site = item.start_site.get_value_as_string()
        end_site = item.end_site.get_value_as_string()
        if not start_site or not end_site:
            return None
        start_prim = stage.GetPrimAtPath(start_site)
        end_prim = stage.GetPrimAtPath(end_site)
        if not start_prim or not start_prim.IsValid() or not end_prim or not end_prim.IsValid():
            return None

        poser = robot_poser.RobotPoser(stage, self._robot_prim, start_prim, end_prim)
        self._poser_cache[item.prim_path] = poser
        return poser

    # ###################################################################
    # IK failure outline helpers
    # ###################################################################

    def _apply_ik_fail_outline(self) -> None:
        """Apply red outline to Gprim descendants of the active IK chain links."""
        if self._robot_prim is None:
            return
        stage = omni.usd.get_context().get_stage()
        if stage is None:
            return
        caches = [self._tracking_cache.get(fp) for fp in self._ik_fail_paths]
        self._outlined_gprim_paths = apply_ik_chain_outline(stage, caches, self._outlined_gprim_paths)
        self._outline_active = bool(self._outlined_gprim_paths)

    def _clear_ik_fail_outline(self) -> None:
        """Remove the red outline from previously outlined prims."""
        if not self._outline_active:
            return
        self._outlined_gprim_paths = clear_ik_chain_outline(self._outlined_gprim_paths)
        self._outline_active = False
        self._ik_fail_paths.clear()

    def _teardown_tracking_for_item(self, item: NamedPoseItem) -> None:
        """Remove an item from all tracking state and update outline/subscription.

        Args:
            item: The named pose item to stop tracking.
        """
        self._tracked_paths.discard(item.prim_path)
        self._dirty_tracked_paths.discard(item.prim_path)
        self._tracking_cache.pop(item.prim_path, None)
        self._ik_fail_paths.discard(item.prim_path)
        if not self._ik_fail_paths:
            self._clear_ik_fail_outline()
        else:
            self._apply_ik_fail_outline()
        if not self._tracked_paths:
            self._release_update_subscription()

    def _on_track_target_toggled(self, item: NamedPoseItem) -> None:
        """Enable or disable tracking for the item; update caches and subscription.

        Args:
            item: The named pose item whose tracking was toggled.
        """
        if item.tracking:
            if self._named_poses_table is not None:
                self._named_poses_table.select_item(item)
            self._on_named_pose_selected(item)
            self._tracked_paths.add(item.prim_path)
            if len(self._tracked_paths) == 1:
                self._request_update_subscription()
            stage = omni.usd.get_context().get_stage()
            if stage is not None and self._robot_prim is not None:
                self._build_tracking_cache(stage, item)
            self._apply_named_pose_joints(item)
        else:
            self._teardown_tracking_for_item(item)

        if self._on_tracking_state_changed_fn is not None:
            self._on_tracking_state_changed_fn(item.prim_path, item.tracking)

    def _apply_named_pose_joints(self, item: NamedPoseItem) -> None:
        """Apply the stored joint state of a named pose to the robot.

        Uses the retained RobotPoser for the item's chain when available so
        the cached kinematic tree is reused.

        Args:
            item: The named pose item whose joints to apply.
        """
        if self._robot_prim is None or not self._robot_prim.IsValid():
            return
        stage = omni.usd.get_context().get_stage()
        if stage is None:
            return
        pose = robot_poser.get_named_pose(stage, self._robot_prim, item.name.get_value_as_string())
        if pose is None or not pose.success or not pose.joints:
            return
        poser = self._get_or_build_poser(stage, item)
        self._updating_transform = True
        try:
            if poser is not None:
                poser.apply_pose(pose.joints)
            else:
                anchor = stage.GetPrimAtPath(pose.start_link) if pose.start_link else None
                if anchor and anchor.IsValid():
                    robot_poser.apply_joint_state_anchored(stage, self._robot_prim, pose.joints, anchor)
                else:
                    robot_poser.apply_joint_state(stage, self._robot_prim, pose.joints)
        finally:
            self._updating_transform = False
        force_manipulator_refresh()

    def _register_usd_listener(self) -> None:
        """Register a USD ObjectsChanged notice listener for tracked prims.

        Returns:
            None.
        """
        if self._usd_listener is not None:
            return
        stage = omni.usd.get_context().get_stage()
        if stage is None:
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
        """Handle USD ObjectsChanged notices for tracked prims and NamedPoses resyncs.

        Args:
            notice: The ObjectsChanged notice.
            sender: The stage that sent the notice.
        """
        if self._updating_transform:
            return

        # Detect NamedPoses scope resyncs (prim renamed/added/removed on stage).
        if not self._renaming_prim and self._robot_prim_path:
            named_poses_prefix = f"{self._robot_prim_path}/{NAMED_POSES_SCOPE}"
            for p in notice.GetResyncedPaths():
                if str(p).startswith(named_poses_prefix):
                    self._schedule_named_poses_refresh()
                    break

        if not self._tracked_paths:
            return
        for p in notice.GetChangedInfoOnlyPaths():
            prim_path = str(p.GetPrimPath())
            if prim_path in self._tracked_paths:
                self._dirty_tracked_paths.add(prim_path)
        for p in notice.GetResyncedPaths():
            prim_path = str(p)
            if prim_path in self._tracked_paths:
                self._dirty_tracked_paths.add(prim_path)

    def _on_add_named_pose(self, start_site_path: str, end_site_path: str) -> None:
        """Add a new named pose from the robot's current joint configuration.

        Args:
            start_site_path: Start link/site path for the chain.
            end_site_path: End link/site path for the chain.
        """
        if self._robot_prim is None or not self._robot_prim.IsValid():
            return

        stage = omni.usd.get_context().get_stage()
        if stage is None:
            return

        if not start_site_path or not end_site_path:
            carb.log_warn("Robot Poser: start and end sites must be set to create a named pose")
            return

        # Remember the selection for next time
        self._last_add_start_site = start_site_path
        self._last_add_end_site = end_site_path

        result = build_pose_from_current_joints(stage, self._robot_prim, start_site_path, end_site_path)
        if result is None:
            return

        names = robot_poser.list_named_poses(stage, self._robot_prim)
        name = "pose_1"
        i = 1
        while name in names:
            i += 1
            name = f"pose_{i}"

        robot_poser.store_named_pose(stage, self._robot_prim, name, result)
        self._refresh_named_poses()

    def _on_remove_named_pose(self, item: NamedPoseItem) -> None:
        """Delete the named pose from USD and refresh the table.

        Args:
            item: The named pose item to remove.
        """
        if self._robot_prim is None or not self._robot_prim.IsValid():
            return

        stage = omni.usd.get_context().get_stage()
        if stage is None:
            return

        pose_name = item.name.get_value_as_string()
        robot_poser.delete_named_pose(stage, self._robot_prim, pose_name)
        self._tracking_cache.pop(item.prim_path, None)
        self._poser_cache.pop(item.prim_path, None)
        self._refresh_named_poses()

    def _on_named_pose_name_changed(self, item: NamedPoseItem, old_name: str, new_name: str) -> None:
        """Rename the USD prim when the table name is edited.

        Args:
            item: The named pose item that was renamed.
            old_name: Previous pose name.
            new_name: New pose name (may be sanitized).
        """
        if self._robot_prim is None or not self._robot_prim.IsValid():
            return
        stage = omni.usd.get_context().get_stage()
        if stage is None:
            return

        safe_name = robot_poser._sanitize_name(new_name)
        if safe_name != new_name:
            item.name.set_value(safe_name)
            new_name = safe_name

        if old_name == new_name:
            return

        pose = robot_poser.get_named_pose(stage, self._robot_prim, old_name)
        if pose is None:
            return

        old_prim_path = item.prim_path
        new_prim_path = f"{self._robot_prim_path}/{NAMED_POSES_SCOPE}/{new_name}"

        self._renaming_prim = True
        self._updating_transform = True
        try:
            robot_poser.store_named_pose(stage, self._robot_prim, new_name, pose)
            robot_poser.delete_named_pose(stage, self._robot_prim, old_name)
        finally:
            self._updating_transform = False
            self._renaming_prim = False

        # Update item and tracking state to reflect the new path.
        item.prim_path = new_prim_path
        item.refresh_text()
        if old_prim_path in self._tracked_paths:
            self._tracked_paths.discard(old_prim_path)
            self._tracked_paths.add(new_prim_path)
        if old_prim_path in self._dirty_tracked_paths:
            self._dirty_tracked_paths.discard(old_prim_path)
            self._dirty_tracked_paths.add(new_prim_path)
        if old_prim_path in self._tracking_cache:
            cache = self._tracking_cache.pop(old_prim_path)
            new_pose_prim = stage.GetPrimAtPath(new_prim_path)
            if new_pose_prim and new_pose_prim.IsValid():
                cache["pose_prim"] = new_pose_prim
            self._tracking_cache[new_prim_path] = cache
        if old_prim_path in self._poser_cache:
            self._poser_cache[new_prim_path] = self._poser_cache.pop(old_prim_path)
        if old_prim_path in self._ik_fail_paths:
            self._ik_fail_paths.discard(old_prim_path)
            self._ik_fail_paths.add(new_prim_path)

    def _on_apply_named_pose_from_table(self, item: NamedPoseItem) -> None:
        """Apply the named pose joints when the play button is pressed.

        Args:
            item: The named pose item to apply.
        """
        self._apply_named_pose_joints(item)

    def _on_named_pose_site_changed(self, item: NamedPoseItem) -> None:
        """Handle start or end site change on an existing named pose row.

        Args:
            item: The named pose item whose sites changed.
        """
        if self._robot_prim is None or not self._robot_prim.IsValid():
            return
        stage = omni.usd.get_context().get_stage()
        if stage is None:
            return

        was_tracking = item.tracking

        self._poser_cache.pop(item.prim_path, None)
        if was_tracking:
            item.tracking = False
            self._teardown_tracking_for_item(item)

        # -- Read new site paths (already updated by the combobox) --
        start_site_path = item.start_site.get_value_as_string()
        end_site_path = item.end_site.get_value_as_string()
        if not start_site_path or not end_site_path:
            return

        result = build_pose_from_current_joints(stage, self._robot_prim, start_site_path, end_site_path)
        if result is None:
            carb.log_warn("Robot Poser: could not build joint chain between updated sites")
            return

        pose_name = item.name.get_value_as_string()
        self._updating_transform = True
        self._renaming_prim = True  # Prevent deferred refresh from USD listener
        try:
            robot_poser.store_named_pose(stage, self._robot_prim, pose_name, result)
        finally:
            self._updating_transform = False
            self._renaming_prim = False

        # Re-enable tracking if it was active
        if was_tracking:
            item.tracking = True
            self._tracked_paths.add(item.prim_path)
            if len(self._tracked_paths) == 1:
                self._request_update_subscription()
            self._build_tracking_cache(stage, item)
            self._apply_named_pose_joints(item)

    def _schedule_named_poses_refresh(self) -> None:
        """Defer a named-poses table refresh to the next frame."""
        if self._refresh_pending:
            return
        self._refresh_pending = True

        async def _deferred() -> None:
            """Run after one frame: clear refresh pending and refresh named poses."""
            import omni.kit.app

            await omni.kit.app.get_app().next_update_async()
            self._refresh_pending = False
            self._refresh_named_poses()

        asyncio.ensure_future(_deferred())

    # ###################################################################
    # Public API for external callers (e.g. property panel)
    # ###################################################################

    def toggle_tracking_for_path(self, prim_path: str, enable: bool) -> bool:
        """Toggle tracking for a named-pose prim path.

        Args:
            prim_path: Path of the IsaacNamedPose prim.
            enable: True to enable tracking, False to disable.

        Returns:
            True if the window handled the request, False if the caller should
            fall back to standalone tracking (e.g. window not open).
        """
        if self._named_poses_table is None or self._robot_prim is None:
            return False
        for item in self._named_poses_table.get_items():
            if isinstance(item, NamedPoseItem) and item.prim_path == prim_path:
                if item.tracking != enable:
                    item.tracking = enable
                    self._on_track_target_toggled(item)
                    model = self._named_poses_table._named_pose_model
                    if model is not None:
                        model._item_changed(item)
                return True
        return False

    # ###################################################################
    # Track Target IK solving
    # ###################################################################

    def _solve_ik_from_target(self, stage: Usd.Stage, item: NamedPoseItem) -> None:
        """Solve IK from the tracked named-pose prim's current transform.

        Args:
            stage: The USD stage.
            item: The named pose item to solve IK for.
        """
        if self._robot_prim is None or not self._robot_prim.IsValid():
            return

        cache = self._tracking_cache.get(item.prim_path)
        if cache is None:
            return

        result = solve_ik_from_cache(cache)

        if result is None:
            self._ik_fail_paths.add(item.prim_path)
            self._apply_ik_fail_outline()
            return

        result_joints, joint_values_native = result

        self._updating_transform = True
        try:
            with Sdf.ChangeBlock():
                cache["poser"].apply_pose(result_joints)
                pose_prim = cache["pose_prim"]
                jv_attr = pose_prim.GetAttribute(robot_schema.Attributes.POSE_JOINT_VALUES.name)
                if jv_attr:
                    jv_attr.Set(joint_values_native)
        finally:
            self._updating_transform = False

        self._ik_fail_paths.discard(item.prim_path)
        if not self._ik_fail_paths:
            self._clear_ik_fail_outline()
        else:
            self._apply_ik_fail_outline()
