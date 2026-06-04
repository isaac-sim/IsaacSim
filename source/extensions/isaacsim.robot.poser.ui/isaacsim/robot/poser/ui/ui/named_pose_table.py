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

"""Named poses table and model for the Robot Poser UI."""

from __future__ import annotations

from collections.abc import Callable
from typing import Any

import omni.ui as ui
from omni.kit.widget.searchfield import SearchField

from ..utils.treeview_delegate import (
    SearchableItem,
    TreeViewIDColumn,
    TreeViewWithPlacerHolderDelegate,
    TreeViewWithPlacerHolderModel,
)


class NamedPoseItem(SearchableItem):
    """Represents a single named pose row in the table.

    Args:
        name: Display name of the pose.
        start_site: Start site path.
        end_site: End site path.
        prim_path: USD prim path of the IsaacNamedPose prim (optional).
    """

    def __init__(
        self,
        name: str,
        start_site: str,
        end_site: str,
        prim_path: str = "",
    ) -> None:
        super().__init__()
        self.name = ui.SimpleStringModel(name)
        self.start_site = ui.SimpleStringModel(start_site)
        self.end_site = ui.SimpleStringModel(end_site)
        self.prim_path = prim_path
        self.tracking = False
        self.text = name

    def refresh_text(self) -> None:
        """Sync text from the name model."""
        self.text = self.name.get_value_as_string()

    def set_property(self, property_name: str, value: object) -> None:
        """Set a model property by name and refresh display text.

        Args:
            property_name: Name of the attribute (e.g. 'name', 'start_site').
            value: Value to set on the underlying model.
        """
        if hasattr(self, property_name):
            getattr(self, property_name).set_value(value)
            self.refresh_text()


class NamedPosesModel(TreeViewWithPlacerHolderModel):
    """Model for the Named Poses TreeView.

    Args:
        items: Initial list of NamedPoseItem instances.
        on_track_target_fn: Callback when track target is toggled.
        on_add_fn: Callback when add is requested (receives start/end paths).
        on_remove_fn: Callback when an item is removed.
    """

    def __init__(
        self,
        items: list,
        on_track_target_fn: Callable[..., Any] | None = None,
        on_add_fn: Callable[..., Any] | None = None,
        on_remove_fn: Callable[..., Any] | None = None,
    ) -> None:
        super().__init__(items)
        self._edit_window = None
        self._on_track_target_fn = on_track_target_fn
        self._on_add_fn = on_add_fn
        self._on_remove_fn = on_remove_fn

    def destroy(self) -> None:
        """Close edit window and destroy the model."""
        if self._edit_window:
            self._edit_window.destroy()
        self._edit_window = None
        super().destroy()

    def get_item_value_model(self, item: object, column_id: int) -> object:
        """Return the value model for the given item and column (name, start_site, or end_site).

        Args:
            item: The row item (NamedPoseItem).
            column_id: Column index (1=name, 2=start_site, 3=end_site).

        Returns:
            The SimpleStringModel for that column, or None.
        """
        if isinstance(item, NamedPoseItem):
            if column_id == 1:
                return item.name
            if column_id == 2:
                return item.start_site
            if column_id == 3:
                return item.end_site
        return None

    def add_item(self, item: NamedPoseItem | None = None) -> None:
        """Add a new named pose row or invoke add callback.

        Args:
            item: Item to add, or None to use callback or create default.
        """
        if self._on_add_fn:
            self._on_add_fn(self._add_start_site_path, self._add_end_site_path)
            return
        if item is None:
            item = NamedPoseItem(name=self.find_unique_name("named_pose"), start_site="", end_site="")
        super().add_item(item)

    def remove_item(self, item: NamedPoseItem) -> None:
        """Remove the item; delegate to callback if registered.

        Args:
            item: The NamedPoseItem to remove.
        """
        if self._on_remove_fn and isinstance(item, NamedPoseItem):
            self._on_remove_fn(item)
            return
        super().remove_item(item)

    def drop_accepted(self, target_item: object, source: object, drop_location: int = -1) -> bool:
        """Return True if source and target are NamedPoseItems.

        Args:
            target_item: Drop target row.
            source: Dragged row.
            drop_location: Drop index (unused).

        Returns:
            True if both are NamedPoseItem.
        """
        if not source:
            return False
        if target_item and isinstance(target_item, NamedPoseItem) and isinstance(source, NamedPoseItem):
            return True
        return False

    def drop(self, target_item: object, source: object, drop_location: int = -1) -> None:
        """Reorder items: move source to target position.

        Args:
            target_item: Drop target row.
            source: Dragged row.
            drop_location: Unused.
        """
        if not source or not target_item:
            return
        if isinstance(target_item, NamedPoseItem) and isinstance(source, NamedPoseItem):
            target_pos = self._children.index(target_item)
            self._children.remove(source)
            self._children.insert(target_pos, source)
            self._item_changed(None)

    def get_drag_mime_data(self, item: object) -> str:
        """Return the pose name as mime data for drag operations.

        Args:
            item: The dragged row (NamedPoseItem).

        Returns:
            Pose name string or empty.
        """
        if isinstance(item, NamedPoseItem):
            return item.name.get_value_as_string() if item else ""
        return ""


class NamedPosesTable:
    """Table widget for viewing and managing named poses.

    Args:
        on_selection_fn: Called when selection changes.
        on_track_target_fn: Called when track target is toggled.
        on_add_fn: Called when add row is used.
        on_remove_fn: Called when an item is removed.
        on_name_changed_fn: Called when a pose name is edited.
        on_apply_pose_fn: Called when apply is clicked.
        on_site_changed_fn: Called when start/end site changes.
        visible: Initial visibility of the frame.
    """

    def __init__(
        self,
        on_selection_fn: Callable[..., Any] | None = None,
        on_track_target_fn: Callable[..., Any] | None = None,
        on_add_fn: Callable[..., Any] | None = None,
        on_remove_fn: Callable[..., Any] | None = None,
        on_name_changed_fn: Callable[..., Any] | None = None,
        on_apply_pose_fn: Callable[..., Any] | None = None,
        on_site_changed_fn: Callable[..., Any] | None = None,
        visible: bool = True,
    ) -> None:
        self.visible = visible
        self._tree_view = None  # type: Any
        self.__subscription = None  # type: Any
        self._named_pose_model = None  # type: NamedPosesModel | None
        self._delegate = None  # type: TreeViewWithPlacerHolderDelegate | None
        self.id_column = None  # type: TreeViewIDColumn | None
        self._on_selection_fn = on_selection_fn
        self._on_track_target_fn = on_track_target_fn
        self._on_add_fn = on_add_fn
        self._on_remove_fn = on_remove_fn
        self._on_name_changed_fn = on_name_changed_fn
        self._on_apply_pose_fn = on_apply_pose_fn
        self._on_site_changed_fn = on_site_changed_fn
        self.frame = ui.Frame(visible=visible)
        self.frame.set_build_fn(self._build_frame)

    def destroy(self) -> None:
        """Unregister subscription and destroy model, tree view, and delegate."""
        self.__subscription = None
        if self._named_pose_model:
            self._named_pose_model.destroy()
        self._named_pose_model = None
        if self._tree_view:
            self._tree_view.destroy()
        self._tree_view = None

    def _build_frame(self) -> None:
        """Build the table frame with search field and tree view."""
        with ui.VStack(spacing=2, name="margin_vstack", height=ui.Fraction(1)):
            with ui.ZStack():
                ui.Rectangle(name="treeview")
                with ui.HStack():
                    ui.Spacer(width=2)
                    with ui.VStack():
                        ui.Spacer(height=4)
                        with ui.HStack(spacing=4, height=0):
                            self._search = SearchField(
                                on_search_fn=self._filter_by_text,
                                subscribe_edit_changed=True,
                                show_tokens=False,
                            )
                        ui.Spacer(height=4)
                        self._build_tree_view()
                        ui.Spacer(height=4)
                    ui.Spacer(width=2)

    def _filter_by_text(self, filters: Any) -> None:
        """Forward filter text to the model.

        Args:
            filters: Filter text or tokens from the search field.
        """
        if self._named_pose_model:
            self._named_pose_model.filter_by_text(filters)

    def _model_changed(self, model: Any, item: Any) -> None:
        """Update ID column count when model items or filter change.

        Args:
            model: The item model.
            item: The item that changed (or None).

        Returns:
            None.
        """
        if item:
            if self._tree_view and self._tree_view.selection and item == self._tree_view.selection[0]:
                return
        if self.id_column and self._named_pose_model:
            if self._named_pose_model._filter_texts:
                new_count = sum(
                    1
                    for child in self._named_pose_model._children
                    if isinstance(child, SearchableItem) and child.filtered_by_text
                )
            else:
                new_count = self._named_pose_model._searchable_num
            if self.id_column.item_count != new_count:
                self.id_column.update(new_count)

    def set_visible(self, visible: bool) -> None:
        """Show or hide the table frame.

        Args:
            visible: True to show, False to hide.
        """
        if self.frame:
            self.frame.visible = visible

    def __selection_changed(self, selection: list) -> None:
        """Invoke selection callback when tree view selection changes.

        Args:
            selection: Current list of selected items.
        """
        if selection:
            item = selection[0]
            if isinstance(item, NamedPoseItem) and self._on_selection_fn:
                self._on_selection_fn(item)

    def update_start_site(self, start_site: list) -> None:
        """Update the start-site options list for the add row.

        Args:
            start_site: List of start-site path strings.
        """
        if self._named_pose_model:
            self._named_pose_model.start_site = start_site

    def update_end_site(self, end_site: list) -> None:
        """Update the end-site options list for the add row.

        Args:
            end_site: List of end-site path strings.
        """
        if self._named_pose_model:
            self._named_pose_model.end_site = end_site

    def set_items(self, items: list) -> None:
        """Replace table items and refresh the ID column.

        Args:
            items: New list of NamedPoseItem instances.
        """
        if self._delegate:
            self._delegate._destroy_site_combos()
        if self._named_pose_model:
            self._named_pose_model.set_items(items)
            if hasattr(self, "id_column") and self.id_column:
                self.id_column.update(self._named_pose_model._searchable_num)

    def get_items(self) -> list:
        """Return the list of NamedPoseItem rows (excluding placeholders).

        Returns:
            List of NamedPoseItem instances.
        """
        if self._named_pose_model:
            return [item for item in self._named_pose_model._children if isinstance(item, NamedPoseItem)]
        return []

    def select_item(self, item: NamedPoseItem) -> None:
        """Programmatically select item in the tree view.

        Args:
            item: The NamedPoseItem to select.
        """
        if self._tree_view is not None:
            self._tree_view.selection = [item]

    def set_add_row_defaults(self, start_path: str, end_path: str) -> None:
        """Set add-row combobox defaults without triggering a rebuild.

        Args:
            start_path: Default start site path for the add row.
            end_path: Default end site path for the add row.
        """
        if self._named_pose_model:
            self._named_pose_model.set_add_row_defaults(start_path, end_path)

    def _build_tree_view(self) -> None:
        """Build the TreeView, model, delegate, and ID column; wire selection and model change."""
        scrolling_frame = ui.ScrollingFrame(
            name="treeview", height=ui.Fraction(1), identifier="poser_named_poses_scroll"
        )
        with scrolling_frame:
            if not self._named_pose_model:
                self._named_pose_model = NamedPosesModel(
                    [],
                    on_track_target_fn=self._on_track_target_fn,
                    on_add_fn=self._on_add_fn,
                    on_remove_fn=self._on_remove_fn,
                )
            model = self._named_pose_model
            assert model is not None
            self.__subscription = model.subscribe_item_changed_fn(self._model_changed)
            headers = ["Named Pose", "Start Site", "End Site"]
            self._delegate = TreeViewWithPlacerHolderDelegate(
                headers,
                [2, 3],
                model,
                on_name_changed_fn=self._on_name_changed_fn,
                on_apply_pose_fn=self._on_apply_pose_fn,
                on_site_changed_fn=self._on_site_changed_fn,
            )
            with ui.HStack():
                self.id_column = TreeViewIDColumn()
                with ui.ZStack():
                    self._tree_view = ui.TreeView(
                        self._named_pose_model,
                        delegate=self._delegate,
                        root_visible=False,
                        header_visible=True,
                        column_widths=[
                            20,
                            ui.Fraction(1),
                            ui.Fraction(0.6),
                            ui.Fraction(0.6),
                            25,
                        ],
                        selection_changed_fn=self.__selection_changed,
                        identifier="poser_named_poses_tree",
                    )

    def cleanup(self) -> None:
        """Destroy model, tree view, delegate, and clear references."""
        if self._named_pose_model:
            self._named_pose_model.destroy()
        self._named_pose_model = None
        if self._tree_view:
            self._tree_view.destroy()
        self._tree_view = None
        if self._delegate:
            self._delegate.destroy()
        self._delegate = None
        self.id_column = None
