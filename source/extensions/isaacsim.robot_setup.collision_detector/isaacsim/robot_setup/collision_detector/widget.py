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

"""Collision detector widget and supporting data/view classes."""

__all__ = ["CollisionDetectorWidget", "deregister_selection_groups"]

import colorsys
from collections.abc import Callable
from dataclasses import dataclass
from typing import Any

import carb
import carb.eventdispatcher
import carb.input
import omni.ui as ui
import omni.usd
from omni.kit.widget.filter import FilterButton
from omni.kit.widget.options_button import OptionsButton
from omni.kit.widget.searchfield import SearchField
from omni.physx.scripts.physicsUtils import get_initial_collider_pairs
from pxr import Gf, Sdf, Tf, Usd, UsdGeom, UsdPhysics

try:
    from usd.schema.isaac import robot_schema
except ImportError:
    robot_schema = None

from .style import STYLES

_OUTLINE_SETTING = "/app/viewport/outline/enabled"

# -- Color palette -----------------------------------------------------------


def _generate_palette(n: int) -> list[tuple[float, float, float]]:
    """Generate perceptually distinct colors using golden-angle hue stepping.

    Args:
        n: Number of colors to generate.

    Returns:
        List of RGB tuples with component values in [0, 1].
    """
    palette: list[tuple[float, float, float]] = []
    golden_angle = 137.508 / 360.0
    for i in range(n):
        hue = (i * golden_angle) % 1.0
        lightness = 0.55 + 0.10 * (i % 3)
        saturation = 0.60 + 0.10 * ((i // 3) % 3)
        r, g, b = colorsys.hls_to_rgb(hue, lightness, saturation)
        palette.append((r, g, b))
    return palette


PASTEL_PALETTE: list[tuple[float, float, float]] = _generate_palette(64)
#: Pre-computed palette of 64 perceptually distinct pastel colours.


# -- Selection-group management ----------------------------------------------

_registered_group_ids: list[int] = []


def _ensure_group_count(n: int) -> None:
    """Register viewport selection groups until at least ``n`` are available.

    Args:
        n: Minimum number of groups that must exist after the call.
    """
    ctx = omni.usd.get_context()
    while len(_registered_group_ids) < n:
        gid = ctx.register_selection_group()
        if gid == 0:
            carb.log_error("register_selection_group returned 0; refusing to use the default group")
            return
        _registered_group_ids.append(gid)


def _get_group_id(index: int) -> int:
    """Return the persistent group ID at the given index.

    Lazily registers new groups when ``index`` exceeds the current count.

    Args:
        index: Zero-based index into the registered group list.

    Returns:
        The selection-group identifier.
    """
    _ensure_group_count(index + 1)
    if index >= len(_registered_group_ids):
        carb.log_error(
            f"Selection group {index} unavailable: register_selection_group failed "
            f"(only {len(_registered_group_ids)} groups registered)"
        )
        # Return last registered group, since 0 is reserved for the default group and should not be used, and 0 only as ultimate last resort
        return _registered_group_ids[-1] if _registered_group_ids else 0
    return _registered_group_ids[index]


def _reset_group_colors(gid: int) -> None:
    """Clear outline and shade colours for a selection group.

    Args:
        gid: Selection-group identifier to reset.
    """
    if gid == 0:
        return
    ctx = omni.usd.get_context()
    ctx.set_selection_group_outline_color(gid, (1.0, 1.0, 1.0, 0.0))
    ctx.set_selection_group_shade_color(gid, (0.0, 0.0, 0.0, 0.0))


def deregister_selection_groups() -> None:
    """Reset and release all registered selection groups."""
    for gid in _registered_group_ids:
        _reset_group_colors(gid)
    _registered_group_ids.clear()


# -- Data --------------------------------------------------------------------


@dataclass
class RigidBodyPairData:
    """Data for one collision pair shown as a row in the TreeView.

    Args:
        body_a_name: Display name of the first rigid body.
        body_b_name: Display name of the second rigid body.
        body_a_path: USD prim path of the first rigid body.
        body_b_path: USD prim path of the second rigid body.
        filtered: Whether this pair has a ``FilteredPairsAPI`` relationship.
        filter_source_path: Prim path where the ``FilteredPairsAPI`` is authored,
            or ``None`` when the pair was not loaded from an existing filter.
    """

    body_a_name: str
    body_b_name: str
    body_a_path: str = ""
    body_b_path: str = ""
    filtered: bool = False
    filter_source_path: str | None = None


# -- TreeView item -----------------------------------------------------------


class RigidBodyPairItem(ui.AbstractItem):
    """Single row item in the flat collision-pair TreeView.

    Args:
        data: Backing data for this row.
    """

    def __init__(self, data: RigidBodyPairData) -> None:
        super().__init__()
        self.data = data
        self.model_a = ui.SimpleStringModel(data.body_a_name)
        self.model_b = ui.SimpleStringModel(data.body_b_name)
        self.model_filtered = ui.SimpleBoolModel(data.filtered)


# -- TreeView model ----------------------------------------------------------


class CollisionListModel(ui.AbstractItemModel):
    """Flat two-column model with search filtering and per-column sorting."""

    def __init__(self) -> None:
        super().__init__()
        self._all_items: list[RigidBodyPairItem] = []
        self._visible_items: list[RigidBodyPairItem] = []
        self._filter_texts: list[str] = []
        self.sort_column: int = 0
        self.sort_ascending: bool = True

    def set_data(self, pairs: list[RigidBodyPairData]) -> None:
        """Replace the model contents and reset sorting.

        Args:
            pairs: New collision-pair data to display.
        """
        self._all_items = [RigidBodyPairItem(p) for p in pairs]
        self.sort_column = 0
        self.sort_ascending = True
        self._apply_filter()

    def filter_by_text(self, filters: list[str]) -> None:
        """Apply free-text search filters to the visible rows.

        Args:
            filters: Whitespace-separated search terms (case-insensitive).
        """
        self._filter_texts = [f.lower() for f in filters if f]
        self._apply_filter()

    def sort_by_column(self, column_id: int) -> None:
        """Toggle sort on the given column.

        Columns 0/1 sort alphabetically, column 2 sorts by filtered state.
        Detects current order and always reverses it.

        Args:
            column_id: Column index to sort by.
        """

        def sort_val(item: RigidBodyPairItem) -> str | int:
            if column_id == 0:
                return item.data.body_a_name.lower()
            elif column_id == 1:
                return item.data.body_b_name.lower()
            else:
                return int(item.data.filtered)

        items = self._all_items
        if len(items) > 1:
            vals = [sort_val(it) for it in items]
            is_ascending = all(vals[i] <= vals[i + 1] for i in range(len(vals) - 1))
            items.sort(key=sort_val, reverse=is_ascending)
            self.sort_ascending = not is_ascending
        else:
            self.sort_ascending = True
        self.sort_column = column_id
        self._apply_filter()

    def _apply_filter(self) -> None:
        """Recompute ``_visible_items`` from the current filter texts."""
        if not self._filter_texts:
            self._visible_items = list(self._all_items)
        else:
            self._visible_items = [item for item in self._all_items if self._matches(item)]
        self._item_changed(None)

    def _matches(self, item: RigidBodyPairItem) -> bool:
        """Check whether an item matches all active filter texts.

        Args:
            item: Row to test.

        Returns:
            True if every filter term appears in the combined body names.
        """
        text = f"{item.data.body_a_name} {item.data.body_b_name}".lower()
        return all(t in text for t in self._filter_texts)

    # -- AbstractItemModel interface ------------------------------------------

    def get_item_children(self, item: ui.AbstractItem | None) -> list[ui.AbstractItem]:
        """Return visible children for the given item.

        Args:
            item: Parent item, or None for root.

        Returns:
            Visible row items when ``item`` is None, otherwise an empty list.
        """
        if item is None:
            return self._visible_items
        return []

    def get_item_value_model_count(self, item: ui.AbstractItem | None) -> int:
        """Return the number of value-model columns.

        Args:
            item: Row item (unused).

        Returns:
            Always 3 (body A, body B, filtered checkbox).
        """
        return 3

    def get_item_value_model(self, item: ui.AbstractItem | None, column_id: int) -> ui.AbstractValueModel:
        """Return the value model for the given item and column.

        Args:
            item: Row item.
            column_id: Column index (0=body A, 1=body B, 2=filtered).

        Returns:
            The corresponding value model, or an empty string model as fallback.
        """
        if isinstance(item, RigidBodyPairItem):
            if column_id == 0:
                return item.model_a
            elif column_id == 1:
                return item.model_b
            else:
                return item.model_filtered
        return ui.SimpleStringModel("")


# -- TreeView delegate -------------------------------------------------------


class CollisionListDelegate(ui.AbstractItemDelegate):
    """Delegate for rendering column headers and row cells in the collision list.

    Args:
        on_select_collision_prim: Callback invoked with the row item and body
            prim path when the focal icon is clicked.
        on_sort: Callback invoked with a column index when a header sort icon
            is clicked.
        on_filtered_toggled: Callback invoked with the toggled item and new
            checked state.
    """

    def __init__(
        self,
        on_select_collision_prim: Callable[["RigidBodyPairItem", str], None],
        on_sort: Callable[[int], None],
        on_filtered_toggled: Callable[["RigidBodyPairItem", bool], None],
    ) -> None:
        super().__init__()
        self._on_select_collision_prim: Callable[[RigidBodyPairItem, str], None] | None = on_select_collision_prim
        self._on_sort: Callable[[int], None] | None = on_sort
        self._on_filtered_toggled: Callable[[RigidBodyPairItem, bool], None] | None = on_filtered_toggled
        self._sort_frames: dict[int, ui.Frame] = {}
        self._sort_column: int = 0
        self._sort_ascending: bool = True
        self._body_color_map: dict[str, tuple[float, float, float]] = {}
        self._swatch_frames: dict[str, list[ui.Frame]] = {}

    def destroy(self) -> None:
        """Release callbacks and cached UI frames."""
        self._on_select_collision_prim = None
        self._on_sort = None
        self._on_filtered_toggled = None
        self._sort_frames.clear()
        self._body_color_map.clear()
        self._swatch_frames.clear()

    def update_sort_indicator(self, sort_column: int, ascending: bool) -> None:
        """Refresh sort-indicator icons to reflect the current sort state.

        Args:
            sort_column: Column index currently sorted.
            ascending: Whether the sort order is ascending.
        """
        self._sort_column = sort_column
        self._sort_ascending = ascending
        for frame in self._sort_frames.values():
            frame.rebuild()

    def update_body_colors(self, color_map: dict[str, tuple[float, float, float]]) -> None:
        """Update the body-path-to-colour mapping and rebuild colour swatches.

        Args:
            color_map: Mapping from USD body path to an RGB tuple. Pass an
                empty dict to clear all swatches.
        """
        self._body_color_map = color_map
        for frames in self._swatch_frames.values():
            for f in frames:
                f.rebuild()
        if not color_map:
            self._swatch_frames.clear()

    @staticmethod
    def _rgb_to_hex(r: float, g: float, b: float) -> int:
        """Convert floating-point RGB components to an ABGR hex integer.

        Args:
            r: Red channel in [0, 1].
            g: Green channel in [0, 1].
            b: Blue channel in [0, 1].

        Returns:
            32-bit ABGR colour value with full opacity.
        """
        return 0xFF000000 | (int(b * 255) << 16) | (int(g * 255) << 8) | int(r * 255)

    def _sort_icon_name(self, column_id: int) -> str:
        """Return the icon name reflecting the sort state of a column.

        Args:
            column_id: Column index.

        Returns:
            One of ``"sort"``, ``"sort_up"``, or ``"sort_down"``.
        """
        if column_id == self._sort_column:
            return "sort_up" if self._sort_ascending else "sort_down"
        return "sort"

    def _sort_tooltip(self, column_id: int) -> str:
        """Return a tooltip string describing the next sort action for a column.

        Args:
            column_id: Column index.

        Returns:
            Human-readable sort direction label.
        """
        if column_id == self._sort_column:
            return "Sort A-Z" if self._sort_ascending else "Sort Z-A"
        return "Sort A-Z"

    # -- header ---------------------------------------------------------------

    def _build_sort_icon(self, column_id: int) -> None:
        """Build the clickable sort-indicator icon for a column header.

        Args:
            column_id: Column index.
        """
        icon_name = self._sort_icon_name(column_id)
        tooltip = self._sort_tooltip(column_id)
        img = ui.Image(name=icon_name, width=8, height=12, tooltip=tooltip)
        img.set_mouse_pressed_fn(
            lambda x, y, b, m, col=column_id: self._on_sort(col) if b == 0 and self._on_sort else None
        )

    def build_header(self, column_id: int = 0) -> None:
        """Build the header widget for a column.

        Args:
            column_id: Column index (0=body A, 1=body B, 2=filtered).
        """
        if column_id <= 1:
            with ui.HStack(height=25, spacing=4):
                ui.Spacer(width=4)
                label_text = "Rigid Body  A" if column_id == 0 else "Rigid Body  B"
                header_id = "collision_detector_header_body_a" if column_id == 0 else "collision_detector_header_body_b"
                ui.Label(label_text, name="header", width=0, identifier=header_id)
                ui.Spacer()
                with ui.VStack(width=14):
                    ui.Spacer()
                    frame = ui.Frame(width=8, height=12)
                    frame.set_build_fn(lambda col=column_id: self._build_sort_icon(col))
                    self._sort_frames[column_id] = frame
                    ui.Spacer()
        else:
            with ui.HStack(height=25, spacing=4):
                ui.Spacer(width=4)
                ui.Label("Filtered Pair", name="header", width=0, identifier="collision_detector_header_filtered")
                ui.Spacer()
                with ui.VStack(width=14):
                    ui.Spacer()
                    frame = ui.Frame(width=8, height=12)
                    frame.set_build_fn(lambda col=column_id: self._build_sort_icon(col))
                    self._sort_frames[column_id] = frame
                    ui.Spacer()

    # -- branch (flat list, no expand arrow) ----------------------------------

    def build_branch(
        self, model: ui.AbstractItemModel, item: ui.AbstractItem, column_id: int, level: int, expanded: bool
    ) -> None:
        """Build the branch widget (no-op for this flat list).

        Args:
            model: The item model.
            item: Current row item.
            column_id: Column index.
            level: Tree depth.
            expanded: Whether the branch is expanded.
        """

    # -- cell widget ----------------------------------------------------------

    def build_widget(
        self, model: ui.AbstractItemModel, item: ui.AbstractItem, column_id: int, level: int, expanded: bool
    ) -> None:
        """Build the cell widget for a row and column.

        Args:
            model: The item model.
            item: Current row item.
            column_id: Column index.
            level: Tree depth.
            expanded: Whether the branch is expanded.
        """
        if not isinstance(item, RigidBodyPairItem):
            return

        if column_id <= 1:
            self._build_body_cell(item, column_id)
        else:
            self._build_filtered_cell(item)

    def _build_body_cell(self, item: RigidBodyPairItem, column_id: int) -> None:
        """Build a rigid-body name cell with colour swatch and focal icon.

        Args:
            item: Row data.
            column_id: 0 for body A, 1 for body B.
        """
        body_name = item.data.body_a_name if column_id == 0 else item.data.body_b_name
        body_path = item.data.body_a_path if column_id == 0 else item.data.body_b_path

        with ui.ZStack(height=22):
            ui.Rectangle(name="focal_background")
            with ui.HStack(height=22, spacing=4):
                ui.Spacer(width=4)
                with ui.VStack(width=14):
                    ui.Spacer()
                    swatch = ui.Frame(width=10, height=10)
                    swatch.set_build_fn(lambda bp=body_path: self._build_swatch(bp))
                    self._swatch_frames.setdefault(body_path, []).append(swatch)
                    ui.Spacer()
                body_label_id = "collision_detector_body_a" if column_id == 0 else "collision_detector_body_b"
                ui.Label(body_name, name="body_child", width=0, tooltip=body_path, identifier=body_label_id)
                ui.Spacer()
                with ui.VStack(width=23):
                    ui.Spacer()
                    img = ui.Image(
                        name="focal",
                        width=13,
                        height=13,
                        tooltip="Select collision prim in stage",
                        identifier="collision_detector_focal",
                    )
                    img.set_mouse_pressed_fn(
                        lambda x, y, b, m, it=item, p=body_path: (
                            self._on_select_collision_prim(it, p) if b == 0 and self._on_select_collision_prim else None
                        )
                    )
                    ui.Spacer()

    def _build_swatch(self, body_path: str) -> None:
        """Build a small coloured rectangle for the given body path.

        Args:
            body_path: USD prim path whose assigned colour is drawn.
        """
        color = self._body_color_map.get(body_path)
        if color:
            ui.Rectangle(
                width=10,
                height=10,
                style={
                    "background_color": self._rgb_to_hex(*color),
                    "border_radius": 2,
                },
            )

    def _build_filtered_cell(self, item: RigidBodyPairItem) -> None:
        """Build the filtered-pair checkbox cell.

        Args:
            item: Row data.
        """
        with ui.ZStack(height=22):
            ui.Rectangle(name="focal_background")
            with ui.HStack(height=22):
                ui.Spacer()
                cb = ui.CheckBox(
                    model=item.model_filtered, width=16, height=16, identifier="collision_detector_filtered_cb"
                )
                cb.model.add_value_changed_fn(
                    lambda m, it=item: (
                        self._on_filtered_toggled(it, m.get_value_as_bool()) if self._on_filtered_toggled else None
                    )
                )
                ui.Spacer()


# -- Widget ------------------------------------------------------------------


class CollisionDetectorWidget:
    """Self-contained widget for detecting and managing robot self-collision pairs.

    Instantiate inside any ``omni.ui`` container context.  The widget builds its
    own UI tree, subscribes to stage events, and manages viewport overlays.

    Args:
        usd_context_name: Name of the USD context.  Empty string uses the default.
    """

    def __init__(self, usd_context_name: str = "") -> None:
        self._usd_context = omni.usd.get_context(usd_context_name)
        self._robot_prim_path: str = ""
        self._robot_paths: list[str] = []
        self._robot_names: list[str] = []
        self._pairs: list[RigidBodyPairData] = []
        self._highlighted_prims: list[str] = []
        self._active_groups: dict[str, int] = {}
        self._group_prims: dict[str, set[str]] = {}
        self._color_assignments: dict[str, int] = {}
        self._batch_updating: bool = False
        self._suppressing_notices: bool = False
        self._focal_item: RigidBodyPairItem | None = None
        self._widget_stage_selection: set[str] = set()
        self._last_tree_selection: list[RigidBodyPairItem] = []
        self._include_env_collisions: bool = False
        self._outline_setting_overridden: bool = False
        self._original_outline_enabled: bool | None = None

        self._tree_model: CollisionListModel | None = None
        self._tree_delegate: CollisionListDelegate | None = None
        self._tree_view: ui.TreeView | None = None

        self._robot_combo: ui.ComboBox | None = None
        self._robot_combo_frame: ui.Frame | None = None
        self._no_collisions_label: ui.Label | None = None
        self._usd_notice_listener: Any = None

        self._build_ui()
        self._stage_subscriptions: list[Any] | None = self._create_stage_subscriptions()
        self._register_usd_notice_listener()
        self._refresh_robot_list()

    # ------------------------------------------------------------------
    # Stage event subscriptions
    # ------------------------------------------------------------------

    def _create_stage_subscriptions(self) -> list[Any]:
        """Subscribe to stage-opened and stage-closing events.

        Returns:
            List of event-dispatcher observation handles.
        """
        return [
            carb.eventdispatcher.get_eventdispatcher().observe_event(
                observer_name="isaacsim.robot_setup.collision_detector",
                event_name=self._usd_context.stage_event_name(event),
                on_event=handler,
            )
            for event, handler in (
                (omni.usd.StageEventType.OPENED, lambda _: self._on_stage_opened()),
                (omni.usd.StageEventType.CLOSING, lambda _: self._on_stage_closing()),
                (omni.usd.StageEventType.SELECTION_CHANGED, lambda _: self._on_stage_selection_changed()),
            )
        ]

    def _on_stage_opened(self) -> None:
        """Reset widget state and rescan for robots after a new stage opens."""
        self._robot_prim_path = ""
        self._robot_paths.clear()
        self._robot_names.clear()
        self._pairs.clear()
        self._color_assignments.clear()
        if self._tree_delegate:
            self._tree_delegate.update_body_colors({})
        if self._tree_model:
            self._tree_model.set_data([])
        if self._no_collisions_label:
            self._no_collisions_label.visible = False
        self._register_usd_notice_listener()
        self._refresh_robot_list()

    def _on_stage_closing(self) -> None:
        """Clear overlays and cached data before the stage is released."""
        self._clear_viewport_overlay()
        self._restore_selection_outline()
        self._last_tree_selection.clear()
        self._color_assignments.clear()
        if self._tree_delegate:
            self._tree_delegate.update_body_colors({})
        self._deregister_usd_notice_listener()
        self._robot_prim_path = ""
        self._robot_paths.clear()
        self._robot_names.clear()
        self._pairs.clear()
        if self._robot_combo_frame:
            self._robot_combo_frame.rebuild()
        if self._tree_model:
            self._tree_model.set_data([])

    def _on_stage_selection_changed(self) -> None:
        """Clear the TreeView selection when the stage selection changes externally."""
        current = set(self._usd_context.get_selection().get_selected_prim_paths())
        if current and current == self._widget_stage_selection:
            return
        self._widget_stage_selection = set()
        if self._tree_view and self._tree_view.selection:
            self._tree_view.selection = []

    # ------------------------------------------------------------------
    # USD notice listener (detect prim add / remove)
    # ------------------------------------------------------------------

    def _register_usd_notice_listener(self) -> None:
        """Start listening for ``Usd.Notice.ObjectsChanged`` on the current stage."""
        self._deregister_usd_notice_listener()
        stage = self._usd_context.get_stage()
        if stage:
            self._usd_notice_listener = Tf.Notice.Register(Usd.Notice.ObjectsChanged, self._on_objects_changed, stage)

    def _deregister_usd_notice_listener(self) -> None:
        """Revoke the active USD notice listener, if any."""
        if self._usd_notice_listener:
            self._usd_notice_listener.Revoke()
            self._usd_notice_listener = None

    def _on_objects_changed(self, notice: Usd.Notice.ObjectsChanged, sender: Usd.Stage) -> None:
        """Handle USD object changes by refreshing the robot list on resyncs.

        Args:
            notice: The USD objects-changed notice.
            sender: The stage that sent the notice.
        """
        if self._suppressing_notices:
            return
        if notice.GetResyncedPaths():
            self._refresh_robot_list()

    # ------------------------------------------------------------------
    # UI construction
    # ------------------------------------------------------------------

    def _build_ui(self) -> None:
        """Build the widget's top-level layout."""
        with ui.VStack(style=STYLES, spacing=4):
            self._build_search_bar()
            self._build_robot_header()
            self._build_options_row()
            self._build_pairs_tree()
            ui.Spacer(height=10)

    def _build_search_bar(self) -> None:
        """Build the search field, filter, and options toolbar."""
        with ui.HStack(height=26, spacing=4):
            self._search = SearchField(on_search_fn=self._on_search, show_tokens=False, separator=None)
            self._filter_button = FilterButton([], width=20)
            self._option_button = OptionsButton([], width=20)

    def _build_robot_header(self) -> None:
        """Build the robot selector row with combo box and check-collisions button."""
        with ui.HStack(height=22, spacing=4):
            ui.Spacer(width=8)
            ui.Label(
                "Robot:",
                width=0,
                style={"font_size": 14, "color": 0xFFCCCCCC},
            )
            ui.Spacer(width=4)
            self._robot_combo_frame = ui.Frame(height=22)
            self._robot_combo_frame.set_build_fn(self._build_robot_combo)
            ui.Spacer()
            btn = ui.ZStack(width=150, identifier="collision_detector_check_collisions")
            with btn:
                ui.Rectangle(name="button_background")
                with ui.HStack(spacing=4):
                    ui.Spacer()
                    with ui.VStack():
                        ui.Spacer()
                        ui.Image(name="check_collisions", width=14, height=14)
                        ui.Spacer()
                    ui.Label("Check Collisions", width=0, identifier="collision_detector_check_collisions_label")
                    ui.Spacer()
                btn.set_mouse_pressed_fn(self._on_check_collisions)
            ui.Spacer(width=8)

    def _build_options_row(self) -> None:
        """Build the row with the environment-collisions checkbox."""
        with ui.HStack(height=22, spacing=4):
            ui.Spacer(width=8)
            self._env_collision_model = ui.SimpleBoolModel(self._include_env_collisions)
            cb = ui.CheckBox(
                model=self._env_collision_model, width=16, height=16, identifier="collision_detector_env_collisions"
            )
            cb.model.add_value_changed_fn(self._on_env_collisions_toggled)
            ui.Label(
                "Include environment collisions",
                width=0,
                style={"font_size": 14, "color": 0xFFCCCCCC},
                identifier="collision_detector_env_collisions_label",
            )
            ui.Spacer()

    def _on_env_collisions_toggled(self, model: ui.AbstractValueModel) -> None:
        """Handle environment-collisions checkbox toggle.

        Args:
            model: The boolean value model from the checkbox.
        """
        self._include_env_collisions = model.get_value_as_bool()

    def _build_robot_combo(self) -> None:
        """Build the robot ``ComboBox`` inside the reusable frame."""
        self._robot_combo = None
        if self._robot_names:
            idx = 0
            if self._robot_prim_path in self._robot_paths:
                idx = self._robot_paths.index(self._robot_prim_path)
            self._robot_combo = ui.ComboBox(
                idx, *self._robot_names, name="robot_selector", identifier="collision_detector_robot_combo"
            )
            self._robot_combo.model.get_item_value_model().add_value_changed_fn(self._on_robot_combo_changed)
        else:
            self._robot_combo = ui.ComboBox(
                0, "No robots found", name="robot_selector", identifier="collision_detector_robot_combo"
            )
            self._robot_combo.enabled = False

    def _build_pairs_tree(self) -> None:
        """Build the scrolling TreeView for collision pairs and the no-collisions label."""
        with ui.ZStack():
            with ui.ScrollingFrame():
                self._tree_model = CollisionListModel()
                self._tree_delegate = CollisionListDelegate(
                    on_select_collision_prim=self._select_collision_prim,
                    on_sort=self._on_sort,
                    on_filtered_toggled=self._on_filtered_toggled,
                )
                self._tree_view = ui.TreeView(
                    self._tree_model,
                    delegate=self._tree_delegate,
                    root_visible=False,
                    header_visible=True,
                    columns_resizable=True,
                    column_widths=[ui.Fraction(1), ui.Fraction(1), ui.Pixel(100)],
                    style_type_name_override="TreeView",
                    height=0,
                    identifier="collision_detector_tree_view",
                )
                self._tree_view.set_selection_changed_fn(self._on_tree_selection_changed)
            self._tree_view.set_key_pressed_fn(self._on_tree_key_pressed)
            self._no_collisions_label = ui.Label(
                "No self-collisions detected.\n(Is articulation self-collisions disabled?)",
                alignment=ui.Alignment.CENTER,
                style={"font_size": 14, "color": 0xFF999999},
                visible=False,
                identifier="collision_detector_no_collisions",
            )

    # ------------------------------------------------------------------
    # Robot dropdown
    # ------------------------------------------------------------------

    def _refresh_robot_list(self) -> None:
        """Scan the stage for robots and update the dropdown.

        Selection policy:
        - If the previously selected robot still exists, keep it (no re-check).
        - If no robot was previously selected, or the old one was removed,
          select the first available robot and auto-check collisions.
        """
        robots = self._find_all_robots()
        new_paths = [str(r.GetPath()) for r in robots]
        new_names = [r.GetName() for r in robots]

        if new_paths == self._robot_paths:
            return

        old_path = self._robot_prim_path
        self._robot_paths = new_paths
        self._robot_names = new_names

        if old_path and old_path in new_paths:
            self._robot_prim_path = old_path
        elif new_paths:
            self._robot_prim_path = new_paths[0]
        else:
            self._robot_prim_path = ""

        if self._robot_combo_frame:
            self._robot_combo_frame.rebuild()

        if self._robot_prim_path and self._robot_prim_path != old_path:
            self._on_check_collisions()

    def _on_robot_combo_changed(self, model: ui.AbstractValueModel) -> None:
        """Handle robot combo selection changes and trigger collision check.

        Args:
            model: The value model from the combo box.
        """
        idx = model.get_value_as_int()
        if 0 <= idx < len(self._robot_paths):
            new_path = self._robot_paths[idx]
            if new_path != self._robot_prim_path:
                self._robot_prim_path = new_path
                self._on_check_collisions()

    # ------------------------------------------------------------------
    # Search / sort
    # ------------------------------------------------------------------

    def _on_search(self, filters: list[str]) -> None:
        """Forward search terms to the tree model.

        Args:
            filters: Search terms entered by the user.
        """
        if self._tree_model:
            self._tree_model.filter_by_text(filters if filters else [])

    def _on_sort(self, column_id: int) -> None:
        """Sort the tree model by the given column and update the indicator.

        Args:
            column_id: Column index to sort by.
        """
        if self._tree_model:
            self._tree_model.sort_by_column(column_id)
            if self._tree_delegate:
                self._tree_delegate.update_sort_indicator(self._tree_model.sort_column, self._tree_model.sort_ascending)

    # ------------------------------------------------------------------
    # Event handlers
    # ------------------------------------------------------------------

    def _on_check_collisions(self, x: float = 0, y: float = 0, button: int = 0, modifier: int = 0) -> None:
        """Run collision detection on the current robot and populate the list.

        Args:
            x: Mouse x position (unused, required by mouse-pressed signature).
            y: Mouse y position (unused, required by mouse-pressed signature).
            button: Mouse button index (unused).
            modifier: Keyboard modifier flags (unused).
        """
        if not self._robot_prim_path:
            carb.log_warn("No robot selected. Pick one from the dropdown.")
            return

        stage = self._usd_context.get_stage()
        if not stage:
            return

        robot_prim = stage.GetPrimAtPath(self._robot_prim_path)
        if not robot_prim or not robot_prim.IsValid():
            carb.log_warn(f"Robot prim no longer valid: {self._robot_prim_path}")
            return

        self._suppressing_notices = True
        try:
            self._pairs = self._detect_collisions(robot_prim, stage)
            self._clear_viewport_overlay()
            self._build_body_color_map()
            if self._tree_model:
                self._tree_model.set_data(self._pairs)
            self._last_tree_selection.clear()
            if self._tree_view:
                self._tree_view.selection = []
            if self._tree_delegate:
                self._tree_delegate.update_sort_indicator(0, True)
            if self._no_collisions_label:
                self._no_collisions_label.visible = len(self._pairs) == 0
            carb.log_info(f"Self-collision check: {len(self._pairs)} pair(s) found.")
        finally:
            self._suppressing_notices = False

    def _build_body_color_map(self) -> None:
        """Assign a unique palette colour to each body path and push to the delegate."""
        self._color_assignments.clear()
        unique: list[str] = []
        seen: set[str] = set()
        for p in self._pairs:
            for path in (p.body_a_path, p.body_b_path):
                if path not in seen:
                    seen.add(path)
                    unique.append(path)
        for i, body_path in enumerate(unique):
            self._color_assignments[body_path] = i % len(PASTEL_PALETTE)
        color_map = {bp: PASTEL_PALETTE[idx] for bp, idx in self._color_assignments.items()}
        if self._tree_delegate:
            self._tree_delegate.update_body_colors(color_map)

    def _on_tree_selection_changed(self, selection: list[ui.AbstractItem]) -> None:
        """Highlight selected collision pairs in the viewport.

        Args:
            selection: Currently selected TreeView items.
        """
        selected = [s for s in selection if isinstance(s, RigidBodyPairItem)]

        if selected == self._last_tree_selection:
            return

        # When a focal-icon click is active, ignore the intermediate clear
        # and redundant re-select that the TreeView's click handling emits.
        focal_active = self._focal_item is not None
        if focal_active:
            if not selected:
                return
            if selected != [self._focal_item]:
                self._focal_item = None
                focal_active = False

        if not focal_active:
            self._focal_item = None
        self._last_tree_selection = list(selected)

        if selected:
            self._enable_selection_outline()
        else:
            self._restore_selection_outline()

        new_bodies: list[str] = []
        seen: set[str] = set()
        for item in selected:
            for path in (item.data.body_a_path, item.data.body_b_path):
                if path not in seen:
                    seen.add(path)
                    new_bodies.append(path)
        new_body_set = set(new_bodies)

        old_active_groups = self._active_groups
        old_group_prims = self._group_prims
        old_highlighted = self._highlighted_prims
        self._active_groups = {}
        self._group_prims = {}
        self._highlighted_prims = []

        self._suppressing_notices = True
        try:
            stage = self._usd_context.get_stage()
            group_index = 0
            for body_path in new_bodies:
                idx = self._color_assignments.get(body_path)
                if idx is None:
                    continue
                color = PASTEL_PALETTE[idx]

                gid = _get_group_id(group_index)
                group_index += 1
                self._active_groups[body_path] = gid
                self._usd_context.set_selection_group_outline_color(gid, (*color, 1.0))
                self._usd_context.set_selection_group_shade_color(gid, (*color, 0.3))

                if stage:
                    gprims = self._collect_gprims(stage, body_path)
                    self._group_prims[body_path] = gprims
                    for p in gprims:
                        self._usd_context.set_selection_group(gid, p)
                    with Usd.EditContext(stage, stage.GetSessionLayer()):
                        self._apply_display_color(stage, body_path, Gf.Vec3f(*color))
                    self._highlighted_prims.append(body_path)

            for body_path, gprims in old_group_prims.items():
                if body_path not in new_body_set:
                    for p in gprims:
                        self._usd_context.set_selection_group(0, p)
            for gid in old_active_groups.values():
                if gid not in self._active_groups.values():
                    _reset_group_colors(gid)

            if stage and old_highlighted:
                with Usd.EditContext(stage, stage.GetSessionLayer()):
                    for body_path in old_highlighted:
                        if body_path not in new_body_set:
                            prim = stage.GetPrimAtPath(body_path)
                            if not prim or not prim.IsValid():
                                continue
                            for desc in Usd.PrimRange(prim):
                                if desc.IsA(UsdGeom.Gprim):
                                    gprim = UsdGeom.Gprim(desc)
                                    for attr in (gprim.GetDisplayColorAttr(), gprim.GetDisplayOpacityAttr()):
                                        if attr and attr.IsAuthored():
                                            attr.Clear()
        finally:
            self._suppressing_notices = False

        if new_bodies and not focal_active:
            self._widget_stage_selection = set(new_bodies)
            self._usd_context.get_selection().set_selected_prim_paths(new_bodies, True)
            self._reapply_selection_groups()

    def _on_tree_key_pressed(self, key_index: int, key_flags: int, key_down: bool) -> None:
        """Move the TreeView selection with Ctrl+UP / Ctrl+DOWN.

        Plain UP/DOWN is reserved by the Stage panel for parent
        selection, so the Ctrl modifier is required.  When multiple
        rows are selected the entire block shifts by one row.

        Args:
            key_index: Integer key code from the input system.
            key_flags: Modifier flags (e.g. Ctrl, Shift).
            key_down: True on key-down, False on key-up.
        """
        if not key_down:
            return
        CTRL = 2
        if not (key_flags & CTRL):
            return
        key = carb.input.KeyboardInput(key_index)
        if key not in (carb.input.KeyboardInput.UP, carb.input.KeyboardInput.DOWN):
            return
        if not self._tree_view or not self._tree_model:
            return

        visible = self._tree_model.get_item_children(None)
        if not visible:
            return

        selected = self._tree_view.selection
        if not selected:
            self._tree_view.selection = [visible[0]]
            return

        indices = sorted({visible.index(s) for s in selected if s in visible})
        if not indices:
            return

        if key == carb.input.KeyboardInput.UP:
            if indices[0] == 0:
                return
            new_indices = [i - 1 for i in indices]
        else:
            if indices[-1] >= len(visible) - 1:
                return
            new_indices = [i + 1 for i in indices]

        self._tree_view.selection = [visible[i] for i in new_indices]

    def _select_collision_prim(self, item: RigidBodyPairItem, body_path: str) -> None:
        """Select the row of the clicked focal icon and set stage selection to that body.

        Sets the TreeView selection to the row (populating groups for both
        bodies if needed), then narrows the viewport highlight down to only
        the clicked body.

        Args:
            item: The TreeView row item containing the focal icon.
            body_path: USD prim path of the clicked body.
        """
        self._focal_item = item
        if self._tree_view:
            current_sel = self._tree_view.selection
            if not current_sel or current_sel != [item]:
                self._tree_view.selection = [item]

        stage = self._usd_context.get_stage()
        if not stage:
            return

        # Clear all existing group assignments
        for gprims in self._group_prims.values():
            for p in gprims:
                self._usd_context.set_selection_group(0, p)
        for gid in self._active_groups.values():
            _reset_group_colors(gid)
        self._active_groups.clear()
        self._group_prims.clear()

        # Set up a single group for the focal body only
        idx = self._color_assignments.get(body_path)
        if idx is not None:
            color = PASTEL_PALETTE[idx]
            gid = _get_group_id(0)
            self._active_groups[body_path] = gid
            self._usd_context.set_selection_group_outline_color(gid, (*color, 1.0))
            self._usd_context.set_selection_group_shade_color(gid, (*color, 0.3))
            gprims = self._collect_gprims(stage, body_path)
            self._group_prims[body_path] = gprims
            for p in gprims:
                self._usd_context.set_selection_group(gid, p)

        # Resolve collision prims and set stage selection
        prim = stage.GetPrimAtPath(body_path)
        if not prim or not prim.IsValid():
            return

        collision_paths: list[str] = []
        for desc in Usd.PrimRange(prim):
            if desc.HasAPI(UsdPhysics.CollisionAPI):
                collision_paths.append(str(desc.GetPath()))

        if not collision_paths:
            collision_paths = [body_path]

        self._widget_stage_selection = set(collision_paths)
        self._usd_context.get_selection().set_selected_prim_paths(collision_paths, True)
        self._reapply_selection_groups()

    def _on_filtered_toggled(self, item: RigidBodyPairItem, checked: bool) -> None:
        """Add or remove filtered-pair relationships, batching across multi-selection.

        Args:
            item: The row item whose checkbox was toggled.
            checked: New checked state.
        """
        if self._batch_updating:
            return

        items_to_toggle = [item]
        if self._tree_view:
            selected = [s for s in self._tree_view.selection if isinstance(s, RigidBodyPairItem)]
            if item in selected and len(selected) > 1:
                items_to_toggle = selected

        stage = self._usd_context.get_stage()
        if not stage:
            return

        self._batch_updating = True
        self._suppressing_notices = True
        try:
            for it in items_to_toggle:
                it.data.filtered = checked
                if it is not item:
                    it.model_filtered.set_value(checked)
                self._write_filtered_pair(stage, it.data, checked)
        finally:
            self._batch_updating = False
            self._suppressing_notices = False

    def _write_filtered_pair(self, stage: Usd.Stage, data: RigidBodyPairData, checked: bool) -> None:
        """Write or remove a ``FilteredPairsAPI`` relationship on the stage.

        Args:
            stage: The active USD stage.
            data: Pair data identifying the two bodies.
            checked: True to add the filtered-pair target, False to remove it.
        """
        prim_a = stage.GetPrimAtPath(data.body_a_path)
        if not prim_a or not prim_a.IsValid():
            carb.log_warn(f"Prim not found: {data.body_a_path}")
            return

        if checked:
            if data.filter_source_path and data.filter_source_path == data.body_b_path:
                owner_path, target_path = data.body_b_path, data.body_a_path
            else:
                owner_path, target_path = data.body_a_path, data.body_b_path
            owner = stage.GetPrimAtPath(owner_path)
            if not owner or not owner.IsValid():
                carb.log_warn(f"Prim not found: {owner_path}")
                return
            filtered_api = UsdPhysics.FilteredPairsAPI.Apply(owner)
            rel = filtered_api.CreateFilteredPairsRel()
            rel.AddTarget(Sdf.Path(target_path))
            data.filter_source_path = owner_path
            carb.log_info(f"Added filtered pair: {data.body_a_name} <-> {data.body_b_name}")
        else:
            for prim_path, target_path in [
                (data.body_a_path, data.body_b_path),
                (data.body_b_path, data.body_a_path),
            ]:
                prim = stage.GetPrimAtPath(prim_path)
                if prim and prim.HasAPI(UsdPhysics.FilteredPairsAPI):
                    rel = UsdPhysics.FilteredPairsAPI(prim).GetFilteredPairsRel()
                    if rel:
                        rel.RemoveTarget(Sdf.Path(target_path))
            carb.log_info(f"Removed filtered pair: {data.body_a_name} <-> {data.body_b_name}")

    # ------------------------------------------------------------------
    # Robot discovery
    # ------------------------------------------------------------------

    def _find_all_robots(self) -> list[Usd.Prim]:
        """Traverse the stage and return all prims with the robot API applied.

        Returns:
            List of robot prims found on the current stage.
        """
        stage = self._usd_context.get_stage()
        if not stage:
            return []
        return [prim for prim in Usd.PrimRange(stage.GetPrimAtPath("/")) if self._is_robot_prim(prim)]

    @staticmethod
    def _is_robot_prim(prim: Usd.Prim) -> bool:
        """Check whether a prim has the Isaac robot API schema applied.

        Args:
            prim: USD prim to test.

        Returns:
            True if the prim carries the robot API.
        """
        return bool(robot_schema and prim.HasAPI(robot_schema.Classes.ROBOT_API.value))

    # ------------------------------------------------------------------
    # Collision detection
    # ------------------------------------------------------------------

    def _detect_collisions(self, robot_prim: Usd.Prim, stage: Usd.Stage) -> list[RigidBodyPairData]:
        """Detect colliding pairs via physics, merged with existing filtered pairs.

        Args:
            robot_prim: The robot root prim.
            stage: The active USD stage.

        Returns:
            Sorted list of collision-pair data, with ``filtered`` flags
            reflecting existing ``FilteredPairsAPI`` relationships.
        """
        robot_body_paths = {str(b.GetPath()) for b in self._get_rigid_bodies(robot_prim, stage)}
        if not robot_body_paths:
            return []

        pairs: dict[tuple[str, str], str | None] = {}

        collider_pairs = get_initial_collider_pairs(stage)
        for collider_a, collider_b in collider_pairs:
            body_a = self._find_rigid_body_ancestor(stage, collider_a)
            body_b = self._find_rigid_body_ancestor(stage, collider_b)
            if not body_a or not body_b:
                continue
            path_a, path_b = str(body_a.GetPath()), str(body_b.GetPath())
            if path_a == path_b:
                continue
            if self._include_env_collisions:
                if path_a not in robot_body_paths and path_b not in robot_body_paths:
                    continue
            else:
                if path_a not in robot_body_paths or path_b not in robot_body_paths:
                    continue
            key = (min(path_a, path_b), max(path_a, path_b))
            pairs.setdefault(key, None)

        for body_path in robot_body_paths:
            prim = stage.GetPrimAtPath(body_path)
            if not prim or not prim.HasAPI(UsdPhysics.FilteredPairsAPI):
                continue
            rel = UsdPhysics.FilteredPairsAPI(prim).GetFilteredPairsRel()
            if not rel:
                continue
            source_path = body_path
            for target in rel.GetTargets():
                target_body = self._find_rigid_body_ancestor(stage, str(target))
                if not target_body:
                    target_prim = stage.GetPrimAtPath(target)
                    if not target_prim or not target_prim.IsValid():
                        continue
                    target_body = target_prim
                target_path = str(target_body.GetPath())
                if source_path == target_path:
                    continue
                if not self._include_env_collisions and target_path not in robot_body_paths:
                    continue
                key = (min(source_path, target_path), max(source_path, target_path))
                pairs[key] = source_path

        results = [
            RigidBodyPairData(
                body_a_name=Sdf.Path(pa).name,
                body_b_name=Sdf.Path(pb).name,
                body_a_path=pa,
                body_b_path=pb,
                filtered=src is not None,
                filter_source_path=src,
            )
            for (pa, pb), src in pairs.items()
        ]
        results.sort(key=lambda r: (r.body_a_name.lower(), r.body_b_name.lower()))

        return results

    @staticmethod
    def _get_rigid_bodies(robot_prim: Usd.Prim, stage: Usd.Stage) -> list[Usd.Prim]:
        """Collect rigid-body prims belonging to a robot.

        Prefers the robot-schema links relationship; falls back to a subtree
        traversal if no links relationship is found.

        Args:
            robot_prim: The robot root prim.
            stage: The active USD stage.

        Returns:
            List of prims carrying ``RigidBodyAPI``.
        """
        bodies: list[Usd.Prim] = []
        if robot_schema:
            links_rel = robot_prim.GetRelationship(robot_schema.Relations.ROBOT_LINKS.name)
            if links_rel:
                for target in links_rel.GetTargets():
                    prim = stage.GetPrimAtPath(target)
                    if prim and prim.IsValid() and prim.HasAPI(UsdPhysics.RigidBodyAPI):
                        bodies.append(prim)
        if bodies:
            return bodies
        for prim in Usd.PrimRange(robot_prim):
            if prim.HasAPI(UsdPhysics.RigidBodyAPI):
                bodies.append(prim)
        return bodies

    @staticmethod
    def _find_rigid_body_ancestor(stage: Usd.Stage, prim_path: str) -> Usd.Prim | None:
        """Walk up from a collider prim to find its owning rigid body.

        Args:
            stage: The active USD stage.
            prim_path: Path to start the upward search from.

        Returns:
            The nearest ancestor prim with ``RigidBodyAPI``, or None.
        """
        prim = stage.GetPrimAtPath(prim_path)
        while prim and prim.IsValid() and prim.GetPath() != Sdf.Path("/"):
            if prim.HasAPI(UsdPhysics.RigidBodyAPI):
                return prim
            prim = prim.GetParent()
        return None

    # ------------------------------------------------------------------
    # Viewport highlighting
    # ------------------------------------------------------------------

    @staticmethod
    def _collect_gprims(stage: Usd.Stage, body_path: str) -> set[str]:
        """Collect all ``UsdGeom.Gprim`` paths under a rigid body.

        Args:
            stage: The active USD stage.
            body_path: Root prim path to search under.

        Returns:
            Set of prim paths for geometry primitives.
        """
        prim = stage.GetPrimAtPath(body_path)
        if not prim or not prim.IsValid():
            return set()
        result: set[str] = set()
        for p in Usd.PrimRange(prim, Usd.TraverseInstanceProxies(Usd.PrimAllPrimsPredicate)):
            if p.IsA(UsdGeom.Gprim):
                result.add(str(p.GetPath()))
        return result

    @staticmethod
    def _apply_display_color(stage: Usd.Stage, body_path: str, color: Gf.Vec3f) -> None:
        """Set ``displayColor`` and ``displayOpacity`` on all gprims under a body.

        Args:
            stage: The active USD stage.
            body_path: Root prim path whose children receive the colour.
            color: RGB colour to apply.
        """
        prim = stage.GetPrimAtPath(body_path)
        if not prim or not prim.IsValid():
            return
        for desc in Usd.PrimRange(prim):
            if desc.IsA(UsdGeom.Gprim):
                gprim = UsdGeom.Gprim(desc)
                gprim.CreateDisplayColorAttr().Set([color])
                gprim.CreateDisplayOpacityAttr().Set([0.6])

    def _reapply_selection_groups(self) -> None:
        """Re-assign all active selection groups.

        ``set_selected_prim_paths`` moves selected prims into the default
        group (0).  Calling this immediately afterwards restores the custom
        group assignments so the colored outlines remain visible.
        """
        for body_path, gid in self._active_groups.items():
            for p in self._group_prims.get(body_path, set()):
                self._usd_context.set_selection_group(gid, p)

    def _clear_viewport_overlay(self) -> None:
        """Remove all selection-group assignments and session-layer display overrides."""
        for gprims in self._group_prims.values():
            for p in gprims:
                self._usd_context.set_selection_group(0, p)
        for gid in self._active_groups.values():
            _reset_group_colors(gid)
        self._group_prims.clear()
        self._active_groups.clear()

        if not self._highlighted_prims:
            return
        stage = self._usd_context.get_stage()
        if not stage:
            self._highlighted_prims.clear()
            return
        with Usd.EditContext(stage, stage.GetSessionLayer()):
            for body_path in self._highlighted_prims:
                prim = stage.GetPrimAtPath(body_path)
                if not prim or not prim.IsValid():
                    continue
                for desc in Usd.PrimRange(prim):
                    if desc.IsA(UsdGeom.Gprim):
                        gprim = UsdGeom.Gprim(desc)
                        for attr in (gprim.GetDisplayColorAttr(), gprim.GetDisplayOpacityAttr()):
                            if attr and attr.IsAuthored():
                                attr.Clear()
        self._highlighted_prims.clear()

    # ------------------------------------------------------------------
    # Viewport outline setting override
    # ------------------------------------------------------------------

    def _enable_selection_outline(self) -> None:
        """Temporarily force the viewport selection outline on.

        Saves the current user setting so it can be restored later.
        No-op if already overridden.
        """
        if self._outline_setting_overridden:
            return
        settings = carb.settings.get_settings()
        self._original_outline_enabled = settings.get(_OUTLINE_SETTING)
        settings.set(_OUTLINE_SETTING, True)
        self._outline_setting_overridden = True

    def _restore_selection_outline(self) -> None:
        """Restore the viewport selection outline to its original value.

        No-op if the setting has not been overridden.
        """
        if not self._outline_setting_overridden:
            return
        settings = carb.settings.get_settings()
        value = self._original_outline_enabled if self._original_outline_enabled is not None else True
        settings.set(_OUTLINE_SETTING, value)
        self._outline_setting_overridden = False
        self._original_outline_enabled = None

    # ------------------------------------------------------------------
    # Lifecycle
    # ------------------------------------------------------------------

    def invalidate_tree(self) -> None:
        """Force a full redraw of the collision-pair TreeView."""
        if self._tree_model:
            self._tree_model._item_changed(None)

    def on_visibility_changed(self, visible: bool) -> None:
        """Respond to the owning window's visibility changes.

        Clears viewport overlays when the widget becomes hidden. Removes listeners when hidden and restores them when shown.

        Args:
            visible: Whether the widget is now visible.
        """
        if not visible:
            self._deregister_usd_notice_listener()
            self._clear_viewport_overlay()
            self._restore_selection_outline()
        else:
            self._register_usd_notice_listener()
            self._refresh_robot_list()

    def destroy(self) -> None:
        """Release all resources, subscriptions, and UI references."""
        self._clear_viewport_overlay()
        self._restore_selection_outline()
        self._last_tree_selection.clear()
        self._deregister_usd_notice_listener()
        self._pairs.clear()
        self._robot_paths.clear()
        self._robot_names.clear()
        self._robot_combo = None
        self._robot_combo_frame = None
        self._color_assignments.clear()
        if self._tree_view:
            self._tree_view.set_key_pressed_fn(None)
            self._tree_view.set_selection_changed_fn(None)
        if self._tree_delegate:
            self._tree_delegate.destroy()
            self._tree_delegate = None
        self._tree_model = None
        self._tree_view = None
        self._stage_subscriptions = None
