# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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

"""Robot Inspector window for joint tree visualization and component masking."""

__all__ = ["RobotInspectorWindow"]

import asyncio
from concurrent.futures import ThreadPoolExecutor
from pathlib import Path
from typing import Any

import carb.eventdispatcher
import carb.settings
import omni.kit.app
import omni.ui as ui
import omni.usd
import usdrt
from omni.kit.widget.stage import StageWidget
from pxr import Sdf, Tf, Usd
from usd.schema.isaac import robot_schema

from .inspector_stage_delegate import InspectorStageDelegate
from .masking_state import MaskingState
from .model import ConnectionItem
from .scene import ConnectionInstance
from .selection_watch import SelectionWatch
from .utils import (
    HierarchyMode,
    PathMap,
    deserialize_hierarchy_result,
    generate_robot_hierarchy_stage,
    generate_robot_hierarchy_stage_in_background,
)


class RobotInspectorWindow(ui.Window):
    """Robot Inspector window: robot structure view with component inspection and masking.

    Provides a configurable view of the robot's links and joints (flat, linked,
    or MuJoCo-style tree), with per-component deactivation and bypass controls.
    Synchronizes selection with the main stage.

    Args:
        usd_context_name: Name of the USD context. Empty string uses the default.
    """

    SETTING_HIERARCHY_MODE = "/persistent/exts/isaacsim.robot.schema.ui/hierarchyMode"

    _MODE_LABELS = ("Flat", "Tree", "MuJoCo")
    _MODE_VALUES = (HierarchyMode.FLAT, HierarchyMode.LINKED, HierarchyMode.MUJOCO)
    _MODE_IDENTIFIERS = (
        "robot_inspector_mode_flat",
        "robot_inspector_mode_tree",
        "robot_inspector_mode_mujoco",
    )
    _MODE_TOOLTIPS = (
        "Flat List: all links under a Links scope, all joints under a Joints scope",
        "Tree: parent link → joint → child link chain",
        "MuJoCo: link-rooted tree with joint-to-parent as last child of each link",
    )

    _MODE_NAME_MAP = {m.name: m for m in HierarchyMode}

    def __init__(self, usd_context_name: str = "") -> None:
        self._usd_context = omni.usd.get_context(usd_context_name)
        self._visibility_changed_listener: Any | None = None
        self._selection_watch: SelectionWatch | None = None
        self._joint_connections: list[Any] = []
        self._original_stage_id = None
        self._current_joints: set[str] = set()
        self._current_hierarchy_mode: HierarchyMode | None = None
        self._active_robot_root_path: Sdf.Path | None = None
        self._tracked_robot_prim_paths: set[str] = set()
        self._stage_listener: Any | None = None
        self._refresh_ui_pending: bool = False
        self._refresh_ui_task: asyncio.Task[Any] | None = None
        self._tabbed_out: bool = False
        self._joint_connections_full: list[Any] = []
        self._last_pushed_viewport_keys: tuple[str, ...] | None = None
        self._hierarchy_executor: ThreadPoolExecutor | None = None
        self._hierarchy_stage: Usd.Stage | None = None
        self._path_map: PathMap | None = None
        self._hierarchy_mode: HierarchyMode = self._load_mode_preference()
        self._view_mode_collection: Any = None
        self._option_hovered: list[bool] = []
        self._option_frames: list[Any] = []
        self._selection_event_sub: Any | None = None
        self._usdrt_stage: usdrt.Usd.Stage | None = None
        self._usdrt_stage_id: int | None = None
        self._robot_paths_cache: set[str] | None = None

        super().__init__(
            "Robot Inspector",
            width=600,
            height=800,
            flags=ui.WINDOW_FLAGS_NO_SCROLLBAR,
            dockPreference=ui.DockPreference.RIGHT_TOP,
        )

        self.set_visibility_changed_fn(self._on_visibility_changed)
        self.set_selected_in_dock_changed_fn(self._on_selected_in_dock_changed)
        self.deferred_dock_in("Stage", ui.DockPolicy.CURRENT_WINDOW_IS_ACTIVE)
        self.dock_order = 0

        self._settings = carb.settings.get_settings()
        self._columns_changed_subscription = None
        self._stage_widget: StageWidget | None = None

        with self.frame:
            with ui.VStack(spacing=0):
                self._build_mode_toolbar()
                self._stage_widget = StageWidget(
                    None,
                    columns_enabled=["Deactivate", "Bypass", "Anchor"],
                    stage_delegate=InspectorStageDelegate(),
                )
        self._patch_stage_model_for_in_memory_stage()

        self._stage_subscriptions: list[Any] | None = self._create_stage_subscriptions()
        self._selection_watch = SelectionWatch(usd_context=self._usd_context)
        self._stage_widget.set_selection_watch(self._selection_watch)

        self._masking_state: MaskingState | None = MaskingState.get_instance()
        self._masking_state.subscribe_changed(self._on_masking_changed)

        self._on_stage_opened()
        # Create the selection subscription up-front when the window is born
        # effectively visible. The dock callback may not fire on construction
        # (no dock change happens), so without this call the selection
        # subscription would only be created the first time the user toggled
        # visibility or the dock state.
        if self._is_effectively_visible():
            self._create_selection_subscription()

    def _get_schema_ui_icons_dir(self) -> str:
        """Resolve this extension's data/icons directory for view-mode radio icons.

        Returns:
            Absolute path string to the icons directory, or an empty string on failure.
        """
        try:
            ext_mgr = omni.kit.app.get_app().get_extension_manager()
            ext_path = ext_mgr.get_extension_path_by_module("isaacsim.robot.schema.ui")
            if ext_path:
                return str(Path(ext_path) / "data" / "icons")
        except Exception:
            pass
        return ""

    def _build_mode_toolbar(self) -> None:
        """Build the view-mode radio group (Flat / Tree / MuJoCo) above the stage widget.

        Each option is a Frame (row) with build_fn so the radio icon reflects both
        selection and hover over the entire row (same hitbox for click and hover).
        """
        icons_dir = self._get_schema_ui_icons_dir()
        off_url = f"{icons_dir}/radio_off.svg" if icons_dir else ""
        on_url = f"{icons_dir}/radio_on.svg" if icons_dir else ""
        on_hover_url = f"{icons_dir}/radio_on_hover.svg" if icons_dir else ""

        self._view_mode_collection = ui.RadioCollection()
        initial_index = self._MODE_VALUES.index(self._hierarchy_mode)
        self._view_mode_collection.model.set_value(initial_index)
        self._option_hovered = [False, False, False]
        self._option_frames = []
        self._option_hover_overlays: list[ui.Image | None] = []

        with ui.HStack(height=26, spacing=2, identifier="robot_inspector_mode_toolbar"):
            ui.Label(
                "Show Robot View:",
                width=0,
                style={"margin_width": 4},
                tooltip="How to display the robot hierarchy: Flat list, Tree (link–joint–link), or MuJoCo-style.",
                identifier="robot_inspector_view_label",
            )
            with ui.HStack():
                for i, (label, tooltip, frame_id) in enumerate(
                    zip(self._MODE_LABELS, self._MODE_TOOLTIPS, self._MODE_IDENTIFIERS)
                ):
                    frame = ui.Frame(
                        width=0,
                        height=26,
                        mouse_pressed_fn=lambda x, y, m, w, idx=i: self._on_view_mode_option_clicked(idx),
                        mouse_hovered_fn=lambda hovered, idx=i: self._on_view_mode_option_hovered(idx, hovered),
                        tooltip=tooltip,
                        identifier=frame_id,
                    )
                    ui.Spacer(width=12)
                    frame.set_build_fn(
                        lambda idx=i, lbl=label, tp=tooltip, fid=frame_id: self._build_view_mode_option(
                            idx, lbl, tp, off_url, on_url, on_hover_url, fid
                        )
                    )
                    self._option_frames.append(frame)
            ui.Spacer()

        self._view_mode_collection.model.add_value_changed_fn(self._on_view_mode_changed)

    def _build_view_mode_option(
        self, index: int, label: str, tooltip: str, off_url: str, on_url: str, on_hover_url: str, identifier: str = ""
    ) -> None:
        """Build one option row: ZStack with base icon+label and a hover overlay (visibility only, no rebuild).

        Args:
            index: Zero-based index of this option in the radio collection.
            label: Display text for the option.
            tooltip: Tooltip string shown on hover.
            off_url: URL of the unselected radio icon.
            on_url: URL of the selected radio icon.
            on_hover_url: URL of the hover-state radio icon.
            identifier: UI identifier prefix for the widgets in this option.
        """
        selected = (
            self._view_mode_collection is not None and self._view_mode_collection.model.get_value_as_int() == index
        )
        hovered = index < len(self._option_hovered) and self._option_hovered[index]
        base_icon_url = on_url if selected else off_url
        show_hover_overlay = hovered and not selected

        with ui.ZStack():
            with ui.HStack(width=0, spacing=4):
                with ui.VStack():
                    ui.Spacer()
                    ui.Image(
                        base_icon_url or "",
                        width=20,
                        height=20,
                        identifier=f"{identifier}_icon" if identifier else "",
                    )
                    ui.Spacer()
                with ui.VStack(height=26):
                    ui.Spacer()
                    ui.Label(
                        label,
                        width=0,
                        height=26,
                        tooltip=tooltip if tooltip else None,
                        identifier=f"{identifier}_label" if identifier else "",
                    )
                    ui.Spacer()
            with ui.HStack(width=0, spacing=4):
                with ui.VStack(height=26):
                    ui.Spacer()
                    hover_img = ui.Image(on_hover_url or "", width=20, height=20)
                    ui.Spacer()
                ui.Spacer(width=4)
            while len(self._option_hover_overlays) <= index:
                self._option_hover_overlays.append(None)
            self._option_hover_overlays[index] = hover_img
            hover_img.visible = show_hover_overlay

    def _on_view_mode_option_hovered(self, index: int, hovered: bool) -> None:
        """Show/hide the hover overlay for the option row (no rebuild).

        Args:
            index: Zero-based index of the option being hovered.
            hovered: True when the pointer enters the row, False when it leaves.
        """
        if index >= len(self._option_hovered) or self._option_hovered[index] == hovered:
            return
        self._option_hovered[index] = hovered
        overlay = self._option_hover_overlays[index] if index < len(self._option_hover_overlays) else None
        if overlay is not None:
            selected = (
                self._view_mode_collection is not None and self._view_mode_collection.model.get_value_as_int() == index
            )
            overlay.visible = hovered and not selected

    def _on_view_mode_option_clicked(self, index: int) -> None:
        """Sync radio selection when user clicks anywhere on the option (radio + label area).

        Args:
            index: Zero-based index of the clicked option.
        """
        if self._view_mode_collection and self._view_mode_collection.model.get_value_as_int() != index:
            self._view_mode_collection.model.set_value(index)

    def _on_view_mode_changed(self, model: Any) -> None:
        """Apply the selected view mode when the radio collection changes.

        Args:
            model: The radio collection model that fired the change event.
        """
        index = model.get_value_as_int()
        if 0 <= index < len(self._MODE_VALUES):
            self._set_hierarchy_mode(self._MODE_VALUES[index])
        for frame in self._option_frames:
            if frame is not None:
                frame.rebuild()

    def _load_mode_preference(self) -> HierarchyMode:
        """Read the persisted hierarchy mode from Carbonite settings.

        Returns:
            Stored `HierarchyMode` value, or `HierarchyMode.LINKED` if the
            setting is absent or unrecognized.
        """
        stored = carb.settings.get_settings().get(self.SETTING_HIERARCHY_MODE)
        if stored and stored in self._MODE_NAME_MAP:
            return self._MODE_NAME_MAP[stored]
        return HierarchyMode.LINKED

    def _save_mode_preference(self) -> None:
        """Write the current hierarchy mode to Carbonite persistent settings."""
        carb.settings.get_settings().set(self.SETTING_HIERARCHY_MODE, self._hierarchy_mode.name)

    def _set_hierarchy_mode(self, mode: HierarchyMode) -> None:
        """Switch the hierarchy display mode and refresh the panel.

        Args:
            mode: The new display mode to activate.
        """
        if mode == self._hierarchy_mode:
            return
        self._hierarchy_mode = mode
        self._save_mode_preference()
        if self._view_mode_collection:
            idx = self._MODE_VALUES.index(mode)
            if self._view_mode_collection.model.get_value_as_int() != idx:
                self._view_mode_collection.model.set_value(idx)
        self._current_joints = set()
        self.refresh_ui()

    def _create_stage_subscriptions(self) -> list[Any]:
        """Create event subscriptions for stage lifecycle events.

        Only ``OPENED`` and ``CLOSING`` are wired here so the inspector
        always tracks stage swaps. ``SELECTION_CHANGED`` is gated on
        effective visibility via :meth:`_create_selection_subscription`
        so background tabs do not pay the per-click handler cost.

        Returns:
            List of event subscription handles.
        """
        return [
            carb.eventdispatcher.get_eventdispatcher().observe_event(
                observer_name="isaacsim.robot.schema.ui",
                event_name=self._usd_context.stage_event_name(event),
                on_event=handler,
            )
            for event, handler in (
                (omni.usd.StageEventType.OPENED, lambda _: self._on_stage_opened()),
                (omni.usd.StageEventType.CLOSING, lambda _: self._on_stage_closing()),
            )
        ]

    def _create_selection_subscription(self) -> None:
        """Subscribe to USD ``SELECTION_CHANGED`` events.

        Called from :meth:`_apply_effective_visibility` when the window
        becomes effectively visible. Idempotent: a second call is a no-op.
        """
        if self._selection_event_sub is not None:
            return
        self._selection_event_sub = carb.eventdispatcher.get_eventdispatcher().observe_event(
            observer_name="isaacsim.robot.schema.ui:window_selection_changed",
            event_name=self._usd_context.stage_event_name(omni.usd.StageEventType.SELECTION_CHANGED),
            on_event=lambda _: self._on_stage_selection_changed(),
        )

    def _destroy_selection_subscription(self) -> None:
        """Drop the ``SELECTION_CHANGED`` subscription if one is active.

        Called from :meth:`_apply_effective_visibility` when the window
        becomes hidden or tabbed out. Idempotent.
        """
        self._selection_event_sub = None

    def _is_effectively_visible(self) -> bool:
        """True if the window is visible and not tabbed out (we only set tabbed-out when the dock callback fires False).

        Returns:
            True if visible and not tabbed out, False otherwise.
        """
        return bool(self.visible and not self._tabbed_out)

    def _get_usdrt_stage(self) -> usdrt.Usd.Stage | None:
        """Return a cached usdrt stage handle attached to the current USD context.

        The cached handle is reattached whenever the USD context's stage id
        changes (e.g. after stage open/close). Returns ``None`` if no stage
        is currently attached or the usdrt attach call fails.

        Returns:
            Cached usdrt stage, or ``None`` on failure.
        """
        stage_id = self._usd_context.get_stage_id()
        if stage_id is None or stage_id < 0:
            self._reset_usdrt_stage_cache()
            return None
        if self._usdrt_stage is not None and self._usdrt_stage_id == stage_id:
            return self._usdrt_stage
        try:
            self._usdrt_stage = usdrt.Usd.Stage.Attach(stage_id)
            self._usdrt_stage_id = stage_id
        except Exception as error:
            carb.log_warn(f"Failed to attach usdrt stage for robot inspector: {error}")
            self._reset_usdrt_stage_cache()
            return None
        return self._usdrt_stage

    def _reset_usdrt_stage_cache(self) -> None:
        """Drop the cached usdrt stage handle and the derived robot-paths set.

        Called on stage open/close and on window destruction so the next
        :meth:`_get_usdrt_stage` call re-attaches against whatever stage is
        currently bound to the USD context.
        """
        self._usdrt_stage = None
        self._usdrt_stage_id = None
        self._invalidate_robot_paths_cache()

    def _get_robot_root_path_set(self) -> set[str]:
        """Return path strings for every prim in the stage carrying the Robot API.

        Uses usdrt's ``GetPrimsWithAppliedAPIName`` so the filter runs against
        Fabric's columnar storage rather than a USD-side traversal. The
        result is cached for the lifetime of the current stage attachment;
        the cache is invalidated by :meth:`_invalidate_robot_paths_cache` on
        stage open/close, and explicitly by :meth:`_on_objects_changed` when
        a notice could change the Robot-API prim set.

        Returns:
            Set of robot root path strings; empty if no usdrt stage is
            available or no prim carries the Robot API.
        """
        if self._robot_paths_cache is not None:
            return self._robot_paths_cache
        usdrt_stage = self._get_usdrt_stage()
        if usdrt_stage is None:
            return set()
        try:
            paths = usdrt_stage.GetPrimsWithAppliedAPIName(robot_schema.Classes.ROBOT_API.value)
        except Exception as error:
            carb.log_warn(f"usdrt GetPrimsWithAppliedAPIName failed for robot inspector: {error}")
            return set()
        self._robot_paths_cache = {str(path) for path in paths}
        return self._robot_paths_cache

    def _invalidate_robot_paths_cache(self) -> None:
        """Drop the cached robot-root path set so the next access re-queries usdrt."""
        self._robot_paths_cache = None

    def _resolve_active_robot_root(self, stage: Usd.Stage | None) -> Sdf.Path | None:
        """Return the robot root path implied by the current USD selection.

        For each selected prim, walks ancestors toward the stage root and
        returns the first one that appears in the set of prims carrying the
        Robot API (queried via usdrt for efficient Fabric-side filtering).
        If multiple selected prims belong to different robots, the first
        robot reached is chosen so that the inspector always represents a
        single robot.

        Args:
            stage: USD stage to resolve against. ``None`` short-circuits.

        Returns:
            Path of the active robot root, or ``None`` if the selection does
            not resolve to any robot.
        """
        if not stage:
            return None
        selection = self._usd_context.get_selection()
        selected_paths = list(selection.get_selected_prim_paths() or [])
        if not selected_paths:
            return None

        robot_paths = self._get_robot_root_path_set()
        if not robot_paths:
            return None

        for path in selected_paths:
            sdf_path = path if isinstance(path, Sdf.Path) else Sdf.Path(str(path))
            current = sdf_path
            while current and current != Sdf.Path.absoluteRootPath:
                if current.pathString in robot_paths:
                    return current
                current = current.GetParentPath()
        return None

    def _pinned_robot_still_valid(self, stage: Usd.Stage | None) -> bool:
        """Return True if the cached active robot path still resolves to a Robot-API prim.

        Used by visibility/stage-open paths to preserve the user's pinned
        scope across hide/show cycles instead of re-resolving from whatever
        selection happens to be active at the time of the toggle.

        Args:
            stage: USD stage to validate against. ``None`` returns ``False``.

        Returns:
            True if ``_active_robot_root_path`` is set and points at a prim
            that still carries the Robot API on ``stage``.
        """
        if not stage or self._active_robot_root_path is None:
            return False
        prim = stage.GetPrimAtPath(self._active_robot_root_path)
        if not prim or not prim.IsValid():
            return False
        return prim.HasAPI(robot_schema.Classes.ROBOT_API.value)

    def _collect_active_robot_prim_paths(self, stage: Usd.Stage | None) -> set[str]:
        """Collect the path strings owned by the active robot for change tracking.

        The active robot's own path plus every link/joint listed in its
        ``robotLinks`` and ``robotJoints`` relationships are returned. The
        full stage is **not** traversed.

        Args:
            stage: USD stage hosting the active robot.

        Returns:
            Set of path strings to compare against incoming USD notices.
        """
        if not stage or self._active_robot_root_path is None:
            return set()
        robot_prim = stage.GetPrimAtPath(self._active_robot_root_path)
        if not robot_prim or not robot_prim.IsValid():
            return set()
        if not robot_prim.HasAPI(robot_schema.Classes.ROBOT_API.value):
            return set()
        paths = {self._active_robot_root_path.pathString}
        for rel_name in (robot_schema.Relations.ROBOT_LINKS.name, robot_schema.Relations.ROBOT_JOINTS.name):
            rel = robot_prim.GetRelationship(rel_name)
            if not rel:
                continue
            for target in rel.GetTargets():
                paths.add(str(target))
        return paths

    def _update_viewport_connections_for_selection(self) -> None:
        """Set viewport joint lines to only the active robot, or none if no selection.

        Memoizes the last-pushed connection set by joint-prim path so that
        repeated calls with an identical payload do not re-enter the
        ``ConnectionModel`` rebuild path. Pushing the same list still triggers
        an overlay/clustering refresh inside the model, which manifests as a
        visible viewport flash; the early-return below avoids that flash when
        the active scope (and therefore the filtered set) has not changed.
        """
        if not self._is_effectively_visible():
            return
        if not self._joint_connections_full or self._active_robot_root_path is None:
            if self._last_pushed_viewport_keys == ():
                return
            ConnectionInstance.get_instance().set_joint_connections([])
            self._last_pushed_viewport_keys = ()
            return
        active_str = self._active_robot_root_path.pathString
        filtered = [c for c in self._joint_connections_full if c.robot_root_path.pathString == active_str]
        keys = tuple(c.joint_prim_path.pathString for c in filtered)
        if keys == self._last_pushed_viewport_keys:
            return
        ConnectionInstance.get_instance().set_joint_connections(filtered)
        self._last_pushed_viewport_keys = keys

    def _on_stage_selection_changed(self) -> None:
        """React to USD selection changes by re-targeting the active robot.

        The inspector pins to the most recently selected robot scope. Selecting
        a different robot (or any descendant of one) switches the scope and
        forces a refresh. Selecting prims that do not resolve to any robot is
        ignored so the user can interact with non-robot prims in the stage
        without losing the inspector view of the previously selected robot.

        When the active scope is unchanged, the viewport-connection refresh is
        also skipped: the filtered connection set is identical to the one
        already on screen, so re-pushing it would only cause a visible flash
        as the underlying ``ConnectionModel`` re-runs its overlay clustering
        and rebuild pipeline.
        """
        if not self._is_effectively_visible():
            return
        stage = self._usd_context.get_stage()
        new_active = self._resolve_active_robot_root(stage)
        # Pin the scope: only re-target when the new selection points at a
        # different robot. A selection that does not resolve to any robot
        # leaves the saved scope intact.
        if new_active is None or new_active == self._active_robot_root_path:
            return
        self._active_robot_root_path = new_active
        self._tracked_robot_prim_paths = self._collect_active_robot_prim_paths(stage)
        self.refresh_ui()
        self._update_viewport_connections_for_selection()

    def _apply_effective_visibility(self) -> None:
        """Apply teardown or setup based on effective visibility (visible and selected in dock).

        When effectively visible, the USD stage objects-changed listener and
        the ``SELECTION_CHANGED`` event subscription are both attached. When
        not effectively visible, both are torn down so the inspector pays no
        per-stage-tick or per-selection cost while it is hidden or tabbed out.

        The pinned ``_active_robot_root_path`` is preserved across hide/show
        cycles when the prim still resolves on the stage. If the prim is
        gone (or the field was never set), the active scope is re-resolved
        from the current USD selection.
        """
        if self._is_effectively_visible():
            self._destroy_selection_watch()
            self._selection_watch = SelectionWatch(usd_context=self._usd_context)
            if self._path_map:
                self._selection_watch.update_path_map(self._path_map)
            stage = self._usd_context.get_stage()
            # Preserve the previously pinned robot across visibility cycles.
            # Only re-resolve from the current selection when the prior pin is
            # missing or no longer carries the Robot API on the current stage.
            if not self._pinned_robot_still_valid(stage):
                self._active_robot_root_path = self._resolve_active_robot_root(stage)
            self._tracked_robot_prim_paths = self._collect_active_robot_prim_paths(stage)
            self._create_stage_listener(stage)
            self._create_selection_subscription()
            self.refresh_ui()
            if self._stage_widget:
                self._stage_widget.set_selection_watch(self._selection_watch)
            self._update_viewport_connections_for_selection()
        else:
            self._destroy_selection_subscription()
            self._destroy_stage_listener()
            if self._refresh_ui_task is not None:
                self._refresh_ui_task.cancel()
                self._refresh_ui_task = None
            self._refresh_ui_pending = False
            self._destroy_selection_watch()
            ConnectionInstance.get_instance().set_joint_connections([])
            self._last_pushed_viewport_keys = ()
            if self._stage_widget:
                self._stage_widget.set_selection_watch(self._selection_watch)

    def _on_visibility_changed(self, visible: bool) -> None:
        """Handle window visibility changes.

        When not effectively visible (closed or tabbed out), the stage listener
        is removed and viewport lines cleared. When effectively visible again,
        the listener is reattached and UI refreshed.

        Args:
            visible: True if window became visible, False if hidden.
        """
        if self._visibility_changed_listener:
            self._visibility_changed_listener(visible)
        self._apply_effective_visibility()

    def _on_selected_in_dock_changed(self, selected: bool) -> None:
        """Handle tab selection in dock: only treat as tabbed-out when the callback explicitly reports False.

        Args:
            selected: True if the window is selected in the dock, False if tabbed out.
        """
        self._tabbed_out = not selected
        self._apply_effective_visibility()

    def _destroy_selection_watch(self) -> None:
        """Destroy the selection watch and clean up references."""
        if self._selection_watch:
            self._selection_watch.destroy()
            self._selection_watch = None

    def _destroy_stage_listener(self) -> None:
        """Destroy the stage listener for prim changes."""
        if self._stage_listener:
            self._stage_listener.Revoke()
            self._stage_listener = None

    def _create_stage_listener(self, stage: Usd.Stage | None) -> None:
        """Create a listener for stage prim changes.

        Args:
            stage: The USD stage to listen to.
        """
        self._destroy_stage_listener()
        if stage:
            self._stage_listener = Tf.Notice.Register(Usd.Notice.ObjectsChanged, self._on_objects_changed, stage)

    def _on_objects_changed(self, notice: Any, stage: Any) -> None:
        """Handle USD stage objects changed notice.

        Walks only the paths reported by the notice and updates the
        ``_tracked_robot_prim_paths`` set incrementally. The stage is **not**
        re-scanned, so the cost of a notice is bounded by the number of
        affected paths rather than the number of robots in the stage.

        A refresh is scheduled when:

        - The active robot's own prim was resynced (added, removed, or
          re-targeted by a Robot API change),
        - A prim listed in (or now missing from) the active robot's
          ``robotLinks`` / ``robotJoints`` relationships was resynced,
        - A property change touches the ``robotLinks`` /
          ``robotJoints`` relationships on the active robot root.

        Refresh is deferred to the next frame so multiple changes in a single
        ``app.update()`` coalesce into one refresh. Camera, selection, and
        plain transform writes are ignored.

        Args:
            notice: The USD change notice.
            stage: The USD stage that changed.
        """
        if self._active_robot_root_path is None:
            # Even with no active scope, a structural change (e.g. a Robot API
            # being added/removed) invalidates the cached robot path set so
            # the next selection-driven resolve sees the updated stage.
            if any(True for _ in notice.GetResyncedPaths()):
                self._invalidate_robot_paths_cache()
            return

        active_root_str = self._active_robot_root_path.pathString
        tracked = self._tracked_robot_prim_paths
        needs_refresh = False
        robot_set_dirty = False

        for resynced_path in notice.GetResyncedPaths():
            robot_set_dirty = True
            path_str = str(resynced_path)
            if (
                path_str == active_root_str
                or path_str in tracked
                or resynced_path.HasPrefix(self._active_robot_root_path)
            ):
                needs_refresh = True
                break

        if not needs_refresh:
            for changed_path in notice.GetChangedInfoOnlyPaths():
                if self._is_robot_structure_change(changed_path):
                    needs_refresh = True
                    break

        if robot_set_dirty:
            self._invalidate_robot_paths_cache()

        if needs_refresh:
            self._tracked_robot_prim_paths = self._collect_active_robot_prim_paths(stage)
            self._schedule_refresh_ui()

    def _schedule_refresh_ui(self) -> None:
        """Schedule a single refresh_ui on the next frame.

        Multiple calls before the next frame coalesce into one refresh.
        """
        if self._refresh_ui_pending:
            return
        self._refresh_ui_pending = True
        self._refresh_ui_task = asyncio.ensure_future(self._deferred_refresh_ui())

    async def _deferred_refresh_ui(self) -> None:
        """Run hierarchy generation in a background task, then update UI on the main thread."""
        try:
            await omni.kit.app.get_app().next_update_async()
        except asyncio.CancelledError:
            return
        finally:
            self._refresh_ui_task = None
            self._refresh_ui_pending = False
        if self._stage_widget is None:
            return
        if self._active_robot_root_path is None:
            self._apply_hierarchy_result(None, PathMap(), [])
            return
        stage = self._usd_context.get_stage()
        if not stage:
            return
        root_layer = stage.GetRootLayer()
        if not root_layer:
            return
        root_layer_identifier = root_layer.identifier
        masking_layer_id = None
        if self._masking_state:
            masking_layer_id = self._masking_state.get_masking_layer_id()
        if self._hierarchy_executor is None:
            self._hierarchy_executor = ThreadPoolExecutor(max_workers=1, thread_name_prefix="robot_hierarchy")
        loop = asyncio.get_event_loop()
        active_root_str = self._active_robot_root_path.pathString
        try:
            data = await loop.run_in_executor(
                self._hierarchy_executor,
                generate_robot_hierarchy_stage_in_background,
                root_layer_identifier,
                masking_layer_id,
                self._hierarchy_mode,
                active_root_str,
            )
        except asyncio.CancelledError:
            return
        hierarchy_stage, path_map, connection_tuples = deserialize_hierarchy_result(data)
        if hierarchy_stage is None:
            return
        joint_connections = [ConnectionItem.from_paths(*t) for t in connection_tuples if t[0] is not None]
        self._apply_hierarchy_result(hierarchy_stage, path_map, joint_connections)

    def _is_robot_structure_change(self, changed_path: Any) -> bool:
        """Check if a USD change affects the active robot's link/joint relationships.

        Only property paths on the active robot root prim qualify. All other
        property writes (transforms, attributes on links, etc.) are ignored
        so the inspector does not refresh on every stage tick.

        Args:
            changed_path: The USD path change to inspect.

        Returns:
            True if the change targets ``robotLinks`` or ``robotJoints`` on
            the active robot.
        """
        if self._active_robot_root_path is None:
            return False
        if not changed_path.IsPropertyPath():
            return False
        if changed_path.GetPrimPath() != self._active_robot_root_path:
            return False
        path_string = str(changed_path)
        if "." not in path_string:
            return False
        property_name = path_string.rsplit(".", 1)[-1]
        return property_name in {
            robot_schema.Relations.ROBOT_LINKS.name,
            robot_schema.Relations.ROBOT_JOINTS.name,
        }

    def set_visibility_changed_listener(self, listener: Any | None) -> None:
        """Set a callback for visibility changes.

        Args:
            listener: Callback function receiving visibility state as argument.

        Example:

        .. code-block:: python

            window.set_visibility_changed_listener(lambda visible: None)
        """
        self._visibility_changed_listener = listener

    def destroy(self) -> None:
        """Clean up resources before window destruction.

        Releases all subscriptions and references for proper hot reloading.

        Example:

        .. code-block:: python

            window.destroy()
        """
        self._visibility_changed_listener = None
        self._view_mode_collection = None
        self._option_hovered = []
        self._option_frames = []
        self._option_hover_overlays = []
        if self._refresh_ui_task is not None:
            self._refresh_ui_task.cancel()
            self._refresh_ui_task = None
        self._refresh_ui_pending = False
        if self._hierarchy_executor is not None:
            self._hierarchy_executor.shutdown(wait=False)
            self._hierarchy_executor = None
        self._destroy_selection_subscription()
        self._destroy_stage_listener()
        self._reset_usdrt_stage_cache()
        if self._masking_state:
            self._masking_state.unsubscribe_changed(self._on_masking_changed)
            self._masking_state.path_map = None
        self._masking_state = None
        self._hierarchy_stage = None
        self._path_map = None
        if self._selection_watch:
            self._selection_watch._tree_view = None
            self._selection_watch._events = None
            self._selection_watch._stage_event_sub = None
            self._selection_watch = None
        if self._stage_widget:
            self._stage_widget.destroy()
            self._stage_widget = None
        self._stage_subscriptions = None
        self._settings = None
        self._columns_changed_subscription = None
        super().destroy()

    def refresh_ui(self) -> None:
        """Refresh the inspector view from the current stage (synchronous path).

        Regenerates the hierarchy stage for the active robot and updates the
        widget. When no robot is selected the inspector is cleared instead of
        scanning the stage. For deferred updates from USD changes, hierarchy
        generation runs in a background thread and the result is applied via
        callback.

        Example:

        .. code-block:: python

            window.refresh_ui()
        """
        if self._active_robot_root_path is None:
            self._apply_hierarchy_result(None, PathMap(), [])
            return
        hierarchy_stage, path_map, joint_connections = generate_robot_hierarchy_stage(
            self._active_robot_root_path,
            self._hierarchy_mode,
        )
        self._apply_hierarchy_result(hierarchy_stage, path_map, joint_connections)

    def _apply_hierarchy_result(
        self,
        hierarchy_stage: Usd.Stage | None,
        path_map: PathMap,
        joint_connections: list[Any],
    ) -> None:
        """Apply a precomputed hierarchy result to the UI (main thread only).

        Args:
            hierarchy_stage: USD stage for the hierarchy view, or None to clear.
            path_map: Path mapping between hierarchy and original stage.
            joint_connections: List of connection items for viewport visualization.

        Returns:
            None.
        """
        current_joints = self._extract_joint_paths(joint_connections)
        mode_unchanged = self._hierarchy_mode == self._current_hierarchy_mode
        if current_joints and self._current_joints == current_joints and mode_unchanged:
            return

        self._current_joints = current_joints
        self._current_hierarchy_mode = self._hierarchy_mode
        self._hierarchy_stage = hierarchy_stage
        self._path_map = path_map
        if self._masking_state:
            self._masking_state.path_map = path_map

        if self._stage_widget is None:
            return

        usd_context = omni.usd.get_context()
        preserved_paths = list(usd_context.get_selection().get_selected_prim_paths())

        self._apply_masking_icons()
        self._stage_widget.open_stage(hierarchy_stage)
        self._patch_stage_model_for_in_memory_stage()
        if self._selection_watch:
            self._selection_watch.update_path_map(path_map)

            async def _deferred_sync() -> None:
                await omni.kit.app.get_app().next_update_async()
                if self._selection_watch and self._stage_widget:
                    tree_view = getattr(self._stage_widget, "_tree_view", None)
                    if tree_view is None:
                        getter = getattr(self._stage_widget, "get_treeview", None)
                        if callable(getter):
                            tree_view = getter()
                    if tree_view is not None:
                        self._selection_watch.set_tree_view(tree_view)
                if preserved_paths and usd_context:
                    usd_context.get_selection().set_selected_prim_paths(preserved_paths, True)
                if self._selection_watch:
                    self._selection_watch.sync_from_stage()
                self._update_viewport_connections_for_selection()

            asyncio.ensure_future(_deferred_sync())
        self._joint_connections_full = joint_connections
        self._update_viewport_connections_for_selection()

    def _extract_joint_paths(self, joint_connections: list[Any]) -> set[str]:
        """Extract joint paths as a set for comparison.

        Args:
            joint_connections: List of connection items.

        Returns:
            Set of joint prim path strings.
        """
        return {joint.joint_prim_path.pathString for joint in joint_connections}

    def _patch_stage_model_for_in_memory_stage(self) -> None:
        """Defensively pre-seed ``StageModel.__usdrt_stage`` to ``None``.

        The hierarchy view drives the stage widget with an in-memory
        ``Usd.Stage`` (no attached USD context), and upstream
        ``omni.kit.widget.stage.StageModel`` only assigns its
        ``__usdrt_stage`` attribute when both a stage **and** a USD
        context are present at construction. Without that attribute,
        ``StageModel._prefilter`` raises ``AttributeError`` the first
        time the user types in the inspector's search field after a
        view-mode switch (or any other ``open_stage`` call into an
        in-memory stage). Setting the name-mangled attribute to ``None``
        here lets the unconditional ``if self.__usdrt_stage:`` check
        evaluate falsy and fall through to the USD code path.

        This is a workaround for a bug in
        ``omni.kit.widget.stage`` ``StageModel.__init__`` /
        ``StageModel._prefilter``; remove once the upstream version is
        fixed and the inspector pins to it.
        """
        if self._stage_widget is None:
            return
        model = self._stage_widget.get_model()
        if model is None:
            return
        if not hasattr(model, "_StageModel__usdrt_stage"):
            setattr(model, "_StageModel__usdrt_stage", None)

    def _apply_masking_icons(self) -> None:
        """Set customData on hierarchy prims to signal deactivation state.

        The ``isaacsim:deactivated`` custom data key is read by the icon
        override callback registered in the extension to swap the prim icon
        to its disabled variant.
        """
        if not self._hierarchy_stage or not self._path_map or not self._masking_state:
            return

        deactivated = self._masking_state.get_deactivated_paths()

        root_prim = self._hierarchy_stage.GetPrimAtPath("/")
        if not root_prim:
            return

        for prim in Usd.PrimRange(root_prim):
            original_path = self._path_map.get_original_path(prim.GetPath())
            if not original_path:
                continue

            is_deactivated = original_path.pathString in deactivated
            prim.SetCustomDataByKey("isaacsim:deactivated", is_deactivated)

    def _on_masking_changed(self) -> None:
        """Handle masking state changes from the MaskingState singleton.

        Sets customData on hierarchy prims and forces the tree view to
        rebuild every cached item so both the Name column icons and the
        Deactivate column reflect the new state. Uses ``refresh_item_names``
        which calls ``_item_changed(item)`` per item, preserving expansion.

        Selection is blocked during the refresh because ``refresh_item_names``
        may trigger transient widget-selection callbacks that would push a
        stale selection to USD.  After the refresh the tree is re-synced from
        the (unchanged) USD selection.
        """
        self._apply_masking_icons()
        if self._stage_widget:
            model = self._stage_widget.get_model()
            if model:
                if self._selection_watch:
                    self._selection_watch._is_in_selection = True
                try:
                    model.refresh_item_names()
                finally:
                    if self._selection_watch:
                        self._selection_watch._is_in_selection = False
                        self._selection_watch.sync_from_stage()

    def _on_stage_opened(self) -> None:
        """Handle stage open event.

        Drops the cached usdrt stage so it will be re-attached against the
        new stage on the next robot-root resolve. When the inspector is
        effectively visible, preserves the previously pinned robot if its
        path still resolves on the new stage; otherwise resolves the active
        robot from the current selection. Then primes the tracked-path set,
        attaches the stage objects-changed listener, and refreshes the
        inspector UI.

        When the inspector is not effectively visible (closed or tabbed out),
        listener attachment and UI refresh are skipped: both will be performed
        the next time the window becomes visible via
        :meth:`_apply_effective_visibility`.
        """
        stage = self._usd_context.get_stage()
        self._original_stage_id = self._usd_context.get_stage_id()
        self._reset_usdrt_stage_cache()
        if not self._is_effectively_visible():
            self._active_robot_root_path = None
            self._tracked_robot_prim_paths = set()
            return
        # Preserve the prior pin if the freshly opened stage still hosts a
        # Robot-API prim at the same path (re-open of the same scene). When
        # the path no longer resolves, fall back to selection-derived resolve.
        if not self._pinned_robot_still_valid(stage):
            self._active_robot_root_path = self._resolve_active_robot_root(stage)
        self._tracked_robot_prim_paths = self._collect_active_robot_prim_paths(stage)
        self._create_stage_listener(stage)
        self.refresh_ui()

    def _on_stage_closing(self) -> None:
        """Handle stage closing event.

        Clears the inspector view, resets connections, masking state, removes
        the stage listener, and drops the cached usdrt stage handle so the
        next robot-root resolve will re-attach against the freshly opened
        stage.
        """
        self._destroy_stage_listener()
        self._reset_usdrt_stage_cache()
        self._active_robot_root_path = None
        self._tracked_robot_prim_paths.clear()
        self._current_joints.clear()
        self._joint_connections_full.clear()
        self._last_pushed_viewport_keys = None
        self._hierarchy_stage = None
        self._path_map = None
        if self._masking_state:
            self._masking_state.clear()
            self._masking_state.path_map = None
        if self._stage_widget:
            self._stage_widget.open_stage(None)
        if self._selection_watch:
            self._selection_watch.update_path_map(None)
        ConnectionInstance.get_instance().set_joint_connections([])

    def get_widget(self) -> StageWidget | None:
        """Get the underlying stage widget.

        Returns:
            The widget instance used by this window.

        Example:

        .. code-block:: python

            stage_widget = window.get_widget()
        """
        return self._stage_widget
