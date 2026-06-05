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

"""Provides UI components for selecting robot assets from USD stages with type filtering and selection constraints."""

import weakref
from functools import partial
from typing import Optional

import omni.kit.app
import omni.ui as ui
import omni.usd
from omni.kit.widget.stage import StageWidget

from ..style import get_asset_picker_style


def filter_prims(stage: object, prim_list: list, type_list: list) -> None:
    """Filter prims from a list based on their USD schema types.

    Checks each prim in the list against the provided type list and returns only those that match at least one type.
    If the type list is empty, returns the original prim list unchanged.

    Args:
        stage: The USD stage containing the prims.
        prim_list: List of prim items to filter.
        type_list: List of USD schema types to filter by.

    Returns:
        Filtered list of prims that match the specified types, or the original list if no filtering is needed.
    """
    if len(type_list) != 0:
        filtered_selection = []
        for item in prim_list:
            prim = stage.GetPrimAtPath(item.path)
            if prim:
                for _type in type_list:
                    if prim.IsA(_type):
                        filtered_selection.append(item)
                        break
        if filtered_selection != prim_list:
            return filtered_selection
    return prim_list


class SelectionWatch:
    """Monitors and manages prim selection changes in a USD stage with filtering and limit constraints.

    This class provides functionality to watch for selection changes in a stage tree view, apply type-based
    filtering to selected prims, enforce selection limits, and trigger callbacks when valid selections occur.
    It integrates with Omniverse Kit's stage widget system to provide controlled selection behavior.

    Args:
        stage: The USD stage to monitor for prim selections.
        on_selection_changed_fn: Callback function invoked when the selection changes with valid prim paths.
        filter_type_list: List of USD prim types to filter selections by.
        targets_limit: Maximum number of prims that can be selected simultaneously.
    """

    def __init__(
        self,
        stage: object,
        on_selection_changed_fn: callable,
        filter_type_list: list | None = None,
        targets_limit: int = 0,
    ) -> None:
        self._stage = stage
        self._last_selected_prim_paths = None
        self._filter_type_list = filter_type_list if filter_type_list is not None else []
        self._on_selection_changed_fn = on_selection_changed_fn
        self._targets_limit = targets_limit
        self._tree_view = None

    def reset(self, stage: object) -> None:
        """Reset the selection watch with a new stage.

        Args:
            stage: The USD stage to watch for selections.
        """
        self.clear_selection()
        self._stage = weakref.ref(stage)

    def set_tree_view(self, tree_view: object) -> None:
        """Set the tree view widget for selection monitoring.

        Args:
            tree_view: The tree view widget to monitor for selection changes.
        """
        self._tree_view = tree_view
        self._tree_view.set_selection_changed_fn(self._on_widget_selection_changed)
        self._last_selected_prim_paths = None

    def clear_selection(self) -> None:
        """Clear the current selection in the tree view.

        Returns:
            None if the tree view is not set.
        """
        if not self._tree_view:
            return

        self._tree_view.model.update_dirty()
        self._tree_view.selection = []
        if self._on_selection_changed_fn:
            self._on_selection_changed_fn([])

    def _on_widget_selection_changed(self, selection: list) -> None:
        """Handle selection changes from the tree view widget.

        Args:
            selection: The new selection from the tree view.

        Returns:
            None if the stage is not available or the selection has not changed.
        """
        stage = self._stage()
        if not stage:
            return

        prim_paths = [str(item.path) for item in selection if item]

        # Deselect instance proxy items if they were selected
        selection = [item for item in selection if item and not item.instance_proxy]

        # Although the stage view has filter, you can still select the ancestor of filtered prims, which might not match the type.
        selection = filter_prims(stage, selection, self._filter_type_list)

        # Deselect if over the limit
        if self._targets_limit > 0 and len(selection) > self._targets_limit:
            selection = selection[: self._targets_limit]

        if self._tree_view.selection != selection:
            self._tree_view.selection = selection
            prim_paths = [str(item.path) for item in selection]

        if prim_paths == self._last_selected_prim_paths:
            return

        self._last_selected_prim_paths = prim_paths
        if self._on_selection_changed_fn:
            self._on_selection_changed_fn(self._last_selected_prim_paths)

    def enable_filtering_checking(self, enable: bool) -> None:
        """It is used to prevent selecting the prims that are filtered out but.

            still displayed when such prims have filtered children. When `enable`
            is True, SelectionWatch should consider filtering when changing Kit's
            selection.

        Args:
            enable: Whether to enable filtering checking during selection changes.
        """

    def set_filtering(self, filter_string: Optional[str]) -> None:
        """Set the filter string for prim selection.

        Args:
            filter_string: The filter string to apply, or None to clear filtering.
        """


class RobotAssetPicker:
    """A UI widget for selecting robot assets from a USD stage.

    Provides an interactive window with a stage browser that allows users to select robot prims based on type
    filtering and target limits. The picker displays the current selection and provides a select button to confirm
    the choice.

    Args:
        title: Window title for the asset picker dialog.
        stage: The USD stage to browse for robot assets.
        filter_type_list: List of USD prim types to filter by. Only prims matching these types will be selectable.
        on_targets_selected: Callback function invoked when assets are selected. Receives a list of selected prim
            paths.
        modal_window: Whether to display the picker as a modal dialog.
        targets_limit: Maximum number of assets that can be selected simultaneously.
        target_name: Display name for the selected assets shown in the UI labels.
    """

    def __init__(
        self,
        title: str,
        stage: object,
        filter_type_list: list | None = None,
        on_targets_selected: callable = None,
        modal_window: bool = False,
        targets_limit: int = 1,
        target_name: str = "",
    ) -> None:
        self._weak_stage = weakref.ref(stage)
        self._selected_paths = []
        self._on_targets_selected = on_targets_selected
        self._filter_type_list = filter_type_list
        self._use_modal = modal_window
        self._targets_limit = targets_limit
        self._target_name = target_name
        self._selection_watch = SelectionWatch(
            stage=self._weak_stage,
            on_selection_changed_fn=self._on_selection_changed,
            filter_type_list=self._filter_type_list,
            targets_limit=self._targets_limit,
        )
        self._stage_widget = None
        self._window = ui.Window(
            f"{title}",
            width=600,
            height=800,
            visible=False,
            flags=ui.WINDOW_FLAGS_MODAL if self._use_modal else ui.WINDOW_FLAGS_NONE,
            visibility_changed_fn=self.on_window_visibility_changed,
        )
        self._window.frame.set_build_fn(self._build_frame)

    def _build_frame(self) -> None:
        """Builds the UI frame for the robot asset picker.

        Sets up the stage widget for asset selection and creates the control buttons and labels.
        """
        with ui.VStack():
            with ui.Frame():
                self._stage_widget = StageWidget(None, columns_enabled=["Type"])
                self._stage_widget.set_selection_watch(self._selection_watch)

            with ui.VStack(height=0, style=get_asset_picker_style(), spacing=2):
                ui.Spacer(height=2)
                self._label = ui.Label(f"Selected {self._target_name}:\nNone")
                self._button = ui.Button(
                    "Select",
                    height=10,
                    clicked_fn=partial(RobotAssetPicker._on_select, weak_self=weakref.ref(self)),
                    enabled=False,
                    identifier="select_button",
                )
                ui.Spacer(height=4)
            self.on_window_visibility_changed(True)

    def on_window_visibility_changed(self, visible: bool) -> None:
        """Handles window visibility changes for the robot asset picker.

        Manages stage attachment and filtering based on window visibility state.

        Args:
            visible: Whether the window is visible.

        Returns:
            None if the stage widget has not been built yet.
        """
        # the _stage_widget not build yet, will call again in build frame
        if not self._stage_widget:
            return
        if not visible:
            if self._stage_widget:
                self._stage_widget.open_stage(None)
        else:
            # Only attach the stage when picker is open. Otherwise the Tf notice listener in StageWidget kills perf
            stage = omni.usd.get_context().get_stage()
            self._selection_watch.reset(stage)
            self._weak_stage = weakref.ref(stage)
            self._stage_widget.open_stage(self._weak_stage())
            if self._filter_type_list:
                self._stage_widget._filter_by_type(self._filter_type_list, False)
                self._stage_widget._filter_button.enable_filters(self._filter_type_list)

    def destroy(self) -> None:
        """Destroys the robot asset picker window and cleans up resources."""
        if self._window:
            self._window.destroy()
        self._window = None
        self._weak_stage = None

    @staticmethod
    def _on_select(weak_self: callable) -> None:
        """Handles the select button click event.

        Triggers the selection callback with currently selected paths and hides the window.

        Args:
            weak_self: Weak reference to the RobotAssetPicker instance.

        Returns:
            None if the weak reference is no longer valid.
        """
        # pylint: disable=protected-access
        weak_self = weak_self()
        if not weak_self:
            return

        if weak_self._on_targets_selected:
            weak_self._on_targets_selected(weak_self._selected_paths)
        weak_self._window.visible = False

    def set_on_selected(self, on_select: callable) -> None:
        """Sets the callback function to be invoked when targets are selected.

        Args:
            on_select: Callback function to execute when selection is confirmed.
        """
        self._on_targets_selected = on_select

    def clean(self) -> None:
        """Cleans up all resources and references used by the robot asset picker.

        Clears callbacks, destroys widgets, and resets internal state.
        """
        self._window.set_visibility_changed_fn(None)
        self._window = None
        self._selection_watch = None
        self._stage_widget.open_stage(None)
        self._stage_widget.destroy()
        self._stage_widget = None
        self._filter_type_list = None
        self._on_targets_selected = None

    @property
    def visible(self) -> bool:
        """Window visibility state of the robot asset picker.

        Returns:
            True if the picker window is visible.
        """
        return self._window.visible

    @visible.setter
    def visible(self, visible: bool) -> None:
        self._window.visible = visible

    def _on_selection_changed(self, paths: list[str]) -> None:
        """Handles selection changes in the stage widget.

        Updates the UI elements and internal state based on the selected asset paths.

        Args:
            paths: List of selected asset paths.
        """
        self._selected_paths = paths
        if self._button:
            self._button.enabled = len(self._selected_paths) > 0
        if self._label:
            text = "\n".join(self._selected_paths)
            label_text = f"Selected {self._target_name}"
            if self._targets_limit > 0:
                label_text += f" ({len(self._selected_paths)}/{self._targets_limit})"
            label_text += f":\n{text if text else 'None'}"
            self._label.text = label_text
