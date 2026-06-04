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

"""Selection synchronization between the hierarchy view and USD stage."""

import asyncio
from typing import Any

import carb.eventdispatcher
import carb.input
import omni.appwindow
import omni.kit.app
import omni.kit.commands
import omni.usd
from omni.kit.widget.stage import StageItem
from pxr import Sdf, Trace

from .utils import PathMap


class SelectionWatch:
    """Synchronizes selection between the hierarchy tree view and the USD stage.

    Monitors selection changes in both the tree view widget and the USD stage,
    translating paths between the hierarchy stage representation and the
    original stage using a PathMap.

    Args:
        tree_view: Optional tree view widget to synchronize selections with.
        usd_context: Optional USD context. Uses the default context if None.
    """

    def __init__(self, tree_view: Any | None = None, usd_context: Any | None = None) -> None:
        self._usd_context: Any = usd_context or omni.usd.get_context()
        self._is_in_selection = False
        self._tree_view: Any | None = None
        self._selection: Any = self._usd_context.get_selection()
        self._stage_model_selection_subscription: Any | None = None
        self._selected_items: set[StageItem] = set()
        self._filter_string: str | None = None
        self._path_map: PathMap | None = None
        self._is_setting_usd_selection = False
        self._is_filter_checking_enabled = False
        self._expected_selection_paths: set[str] = set()
        self._ctrl_held_on_last_click = False
        self._events: Any | None = None

        self.set_tree_view(tree_view)
        self._stage_event_sub: Any | None = carb.eventdispatcher.get_eventdispatcher().observe_event(
            observer_name="isaacsim.robot.schema.ui:selection_watch_selection_changed",
            event_name=self._usd_context.stage_event_name(omni.usd.StageEventType.SELECTION_CHANGED),
            on_event=self._on_stage_selection_changed_event,
        )

    def update_path_map(self, path_map: PathMap | None) -> None:
        """Update the path mapping for hierarchy-to-original translation.

        Args:
            path_map: Path mapping object, or None to disable translation.

        Example:

        .. code-block:: python

            watch.update_path_map(path_map)
        """
        self._path_map = path_map

    def sync_from_stage(self) -> None:
        """Reapply current USD stage selection to the tree view and expand to show it.

        Call after opening a new hierarchy stage or updating the path map so the
        same prims stay selected and the tree is expanded to reveal them.
        """
        self._on_selection_changed()

    def destroy(self) -> None:
        """Clean up resources and subscriptions.

        Example:

        .. code-block:: python

            watch.destroy()
        """
        self._is_setting_usd_selection = False
        self._expected_selection_paths = set()
        self._ctrl_held_on_last_click = False
        self._usd_context = None
        self._selection = None
        self._stage_event_sub = None
        if self._tree_view is not None:
            self._tree_view.set_selection_changed_fn(None)
            self._tree_view.set_mouse_pressed_fn(None)
            self._tree_view = None
        self._stage_model_selection_subscription = None

    def set_tree_view(self, tree_view: Any | None) -> None:
        """Set or replace the tree view for selection synchronization.

        Always re-registers callbacks even when the tree view widget is
        the same object, because ``open_stage`` may clear the widget's
        internal callback registrations while reusing the same instance.

        Args:
            tree_view: The tree view widget to synchronize with.

        Example:

        .. code-block:: python

            watch.set_tree_view(tree_view)
        """
        if self._tree_view is not None:
            self._tree_view.set_selection_changed_fn(None)
            self._tree_view.set_mouse_pressed_fn(None)
        self._tree_view = tree_view
        if self._tree_view is not None:
            self._tree_view.set_selection_changed_fn(self._on_widget_selection_changed)
            self._tree_view.set_mouse_pressed_fn(self._on_tree_mouse_pressed)

        if self._tree_view:
            self._on_stage_items_selection_changed()
            self._stage_model_selection_subscription = self._tree_view.model.subscribe_stage_items_selection_changed(
                self._on_stage_items_selection_changed
            )

    def _on_stage_selection_changed_event(self, event: Any) -> None:
        """Handle stage selection change events.

        Ignores events while we are processing a widget-initiated selection
        change (``_is_in_selection``) or while we are waiting for the deferred
        guard reset after pushing a selection to USD
        (``_is_setting_usd_selection``).

        Args:
            event: Event payload (unused).

        Returns:
            None.
        """
        if self._is_in_selection or self._is_setting_usd_selection:
            return
        self._on_selection_changed()

    def _on_selection_changed(self) -> None:
        """Process a selection change from the USD stage.

        Translates stage paths to hierarchy paths and updates the tree view.
        Skips when the tree view model is not yet a stage model (e.g. during
        open_stage transitions where the model is a bare AbstractItemModel).

        When the resolved set of tree-view items is identical to the one
        already applied, this method short-circuits before touching the model
        or widget. The unconditional ``model.update_dirty()`` /
        ``_expand_to_selected_items`` / ``_apply_tree_view_selection`` chain
        would otherwise force the tree view to redraw on every USD
        ``SELECTION_CHANGED`` event — including the feedback events that the
        inspector's own selection round-trip emits — which is visible to the
        user as a flash on every click within the same scope.

        Returns:
            None.
        """
        if not self._path_map or not self._tree_view:
            return

        model = self._tree_view.model
        if not hasattr(model, "update_dirty"):
            return

        selection_paths = self._usd_context.get_selection().get_selected_prim_paths()
        selected_items = self._resolve_selected_items(selection_paths)

        if selected_items == self._selected_items:
            return

        self._update_selected_items(selected_items)
        model.update_dirty()
        self._expand_to_selected_items(selected_items)
        self._apply_tree_view_selection(selected_items)

    def _resolve_selected_items(self, selection_paths: list[str]) -> set[StageItem]:
        """Resolve stage paths to tree view items.

        Args:
            selection_paths: List of selected prim path strings.

        Returns:
            Set of selected items.
        """
        if self._path_map is None or self._tree_view is None:
            return set()
        path_map = self._path_map
        tree_view = self._tree_view
        selected_items = set()
        for path in selection_paths:
            hierarchy_path = path_map.get_hierarchy_path(Sdf.Path(path))
            if hierarchy_path:
                stage_item = tree_view.model._get_stage_item_from_cache(str(hierarchy_path), True)
                if stage_item:
                    selected_items.add(stage_item)
        return selected_items

    def _expand_to_selected_items(self, selected_items: set[StageItem]) -> None:
        """Expand the tree view to show all selected items.

        Args:
            selected_items: Set of selected items to expand to.

        Returns:
            None.
        """
        if self._tree_view is None:
            return
        tree_view = self._tree_view
        for selected_item in selected_items:
            path = selected_item.path
            full_chain = tree_view.model.find_full_chain(path)
            if not full_chain or full_chain[-1].path != path:
                continue
            for item in full_chain[:-1]:
                tree_view.set_expanded(item, True, False)

    def _apply_tree_view_selection(self, selected_items: set[StageItem]) -> None:
        """Apply selection to the tree view.

        Args:
            selected_items: Set of selected items to select.

        Returns:
            None.
        """
        if self._tree_view is None:
            return
        tree_view = self._tree_view
        self._is_in_selection = True
        tree_view.selection = list(selected_items)
        self._is_in_selection = False

    def set_filtering(self, filter_string: str | None) -> None:
        """Set the filter string for selection filtering.

        Args:
            filter_string: The filter string (converted to lowercase), or None.

        Example:

        .. code-block:: python

            watch.set_filtering("arm")
        """
        if filter_string:
            self._filter_string = filter_string.lower()
        else:
            self._filter_string = filter_string

    def enable_filtering_checking(self, enable: bool) -> None:
        """Enable or disable selection filtering.

        Args:
            enable: True to enable filter checking during selection.

        Example:

        .. code-block:: python

            watch.enable_filtering_checking(True)
        """
        self._is_filter_checking_enabled = enable

    @Trace.TraceFunction
    def _on_stage_items_selection_changed(self) -> None:
        """Handle selection changes from the stage model.

        Currently disabled; reserved for future bidirectional sync.

        Returns:
            None.
        """
        return

    def _on_tree_mouse_pressed(self, x: float, y: float, button: int, modifier: int) -> None:
        """Capture modifier state at the moment a left-click lands on the tree.

        This fires synchronously during the mouse-press event, before the
        tree view updates its selection.  The captured flag is consumed by
        :meth:`_merge_cross_type_selection` in the subsequent
        ``selection_changed_fn`` callback.

        Args:
            x: Click x-coordinate relative to the tree view widget.
            y: Click y-coordinate relative to the tree view widget.
            button: Mouse button index (0 = left).
            modifier: Bitmask of active keyboard modifiers.
        """
        if button == 0:
            self._ctrl_held_on_last_click = self._is_ctrl_modifier(modifier)

    @staticmethod
    def _is_ctrl_modifier(modifier: int) -> bool:
        """Check whether the Ctrl modifier is active in a UI modifier bitmask.

        Falls back to polling ``carb.input`` when the bitmask alone is
        inconclusive (e.g. always zero on some Kit builds).

        Args:
            modifier: Modifier bitmask from an ``omni.ui`` mouse callback.

        Returns:
            True when Ctrl (or Cmd on macOS) is held.
        """
        if modifier & 2:
            return True
        try:
            app_window = omni.appwindow.get_default_app_window()
            keyboard = app_window.get_keyboard()
            inp = carb.input.acquire_input_provider()
            for key in (
                carb.input.KeyboardInput.LEFT_CONTROL,
                carb.input.KeyboardInput.RIGHT_CONTROL,
            ):
                if inp.get_keyboard_value(keyboard, key) > 0:
                    return True
        except Exception:
            pass
        return False

    @Trace.TraceFunction
    def _on_widget_selection_changed(self, selection: list[StageItem]) -> None:
        """Handle selection changes from the tree view widget.

        Translates hierarchy paths back to original stage paths and
        updates the USD stage selection.  When a multi-select modifier
        (Ctrl / Cmd) is held, items that the tree view dropped because
        they live under a different parent scope are merged back so that
        mixed link+joint selections work.

        Also suppressed while ``_is_setting_usd_selection`` is active
        so that deferred tree-view normalization callbacks do not push
        a reduced selection back to USD.

        Args:
            selection: List of selected items.

        Returns:
            None.
        """
        if self._is_in_selection or self._is_setting_usd_selection or not self._tree_view or not self._path_map:
            return

        self._is_in_selection = True
        try:
            selection = self._merge_cross_type_selection(selection)

            prim_paths = self._translate_to_original_paths(selection)

            if self._filter_string or self._is_filter_checking_enabled:
                selection = self._apply_filter_to_selection(selection, prim_paths)

            self.set_selected_stage_items(selection, enable_undo=True)
        finally:
            self._is_in_selection = False
            self._ctrl_held_on_last_click = False

    def _merge_cross_type_selection(self, selection: list[StageItem]) -> list[StageItem]:
        """Merge previously selected items back when Ctrl/Cmd is held.

        The tree view may not support Ctrl+click across different parent
        scopes (e.g. Links/ vs Joints/ in flat mode).  When that happens
        the tree replaces the old selection instead of extending it.  This
        method detects the situation -- new items appeared AND old items
        vanished while a multi-select modifier is pressed -- and unions
        both sets so the combined selection reaches USD.

        Args:
            selection: Items reported by the tree view.

        Returns:
            Possibly augmented list of selected items.
        """
        if not self._selected_items or not self._ctrl_held_on_last_click:
            return selection

        new_set = set(selection)
        added = new_set - self._selected_items
        lost = self._selected_items - new_set

        if added and lost:
            merged = list(new_set | lost)
            return merged

        return selection

    def _translate_to_original_paths(self, selection: list[StageItem]) -> list[Sdf.Path | None]:
        """Translate hierarchy paths to original stage paths.

        Args:
            selection: List of selected items.

        Returns:
            List of original paths.
        """
        if self._path_map is None:
            return []
        path_map = self._path_map
        return [path_map.get_original_path(Sdf.Path(item.path)) for item in selection if item]

    def _apply_filter_to_selection(
        self, selection: list[StageItem], prim_paths: list[Sdf.Path | None]
    ) -> list[StageItem]:
        """Apply filter to selection and update the tree view if needed.

        Args:
            selection: List of selected items.
            prim_paths: List of corresponding original paths.

        Returns:
            Filtered selection list.
        """
        if self._tree_view is None:
            return selection
        filtered_paths = [item.path for item in selection if item and item.filtered]
        if filtered_paths != [path.pathString for path in prim_paths if path]:
            selection = [item for item in selection if item and item.path in filtered_paths]
            self._tree_view.selection = selection
        return selection

    def _update_selected_items(self, selections: set[StageItem]) -> None:
        """Update the internal selected items set.

        Args:
            selections: Set of selected items.
        """
        self._selected_items = set(selections)

    def set_selected_stage_items(self, selections: list[StageItem], enable_undo: bool = False) -> None:
        """Set the selected items and update the USD stage selection.

        The ``_is_setting_usd_selection`` guard is kept active until the next
        frame via :meth:`_schedule_selection_guard_reset` so that any feedback
        ``SELECTION_CHANGED`` events (e.g. from the main-stage widget
        re-processing the selection) are absorbed without overwriting the
        tree-view selection.

        Args:
            selections: List of selected items to select.
            enable_undo: If True, the selection change can be undone.

        Returns:
            None.

        Example:

        .. code-block:: python

            watch.set_selected_stage_items([item], enable_undo=True)
        """
        if self._path_map is None:
            return
        path_map = self._path_map
        all_items = {item for item in selections if item}
        if all_items != self._selected_items:
            if self._usd_context:
                new_paths = [
                    path.pathString
                    for item in selections
                    if item and (path := path_map.get_original_path(Sdf.Path(item.path)))
                ]
                self._expected_selection_paths = set(new_paths)
                self._is_setting_usd_selection = True
                if not enable_undo:
                    self._selection.set_selected_prim_paths(new_paths, True)
                else:
                    old_paths = [
                        path.pathString
                        for item in self._selected_items
                        if item and (path := path_map.get_original_path(Sdf.Path(item.path)))
                    ]
                    omni.kit.commands.execute(
                        "SelectPrims",
                        old_selected_paths=old_paths,
                        new_selected_paths=new_paths,
                        expand_in_stage=False,
                    )
                self._schedule_selection_guard_reset()
            self._update_selected_items(all_items)

    def _schedule_selection_guard_reset(self) -> None:
        """Reset ``_is_setting_usd_selection`` on the next frame.

        Keeping the guard active for one full frame absorbs any feedback
        ``SELECTION_CHANGED`` events that other widgets (e.g. the main-stage
        ``SelectionWatch``) may emit while processing our selection push.

        After the reset, the current USD selection is compared with the
        latest expected one (stored on ``self``).  If another widget
        overwrote it (e.g. dropped one type), the expected selection is
        re-applied so cross-type selections survive.  Reading from
        ``self._expected_selection_paths`` (instead of a captured
        snapshot) ensures that rapid successive clicks always honour the
        most recent selection, not a stale one.
        """

        async def _reset() -> None:
            await omni.kit.app.get_app().next_update_async()
            if not (self._usd_context and self._path_map and self._tree_view):
                self._is_setting_usd_selection = False
                return
            expected = self._expected_selection_paths
            current = set(self._usd_context.get_selection().get_selected_prim_paths())
            if expected and current != expected:
                self._selection.set_selected_prim_paths(list(expected), True)
            self._is_setting_usd_selection = False

        asyncio.ensure_future(_reset())
