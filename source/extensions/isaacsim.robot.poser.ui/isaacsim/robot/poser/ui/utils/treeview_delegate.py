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

"""TreeView delegate and model utilities for the Robot Poser named-poses table."""

import asyncio
from collections.abc import Callable
from typing import Any

import omni.ui as ui

from ..ui.site_widget import SiteSearchComboBox


def _build_icon_button(
    icon_name: str,
    on_clicked_fn: Callable[[], None],
    bg_name: str | None = None,
    enabled: bool = True,
    identifier: str | None = None,
) -> None:
    """Build a small icon button with a hover overlay.

    Args:
        icon_name: Style name for the ui.Image (e.g. 'remove', 'play').
        on_clicked_fn: No-arg callback invoked on click.
        bg_name: Optional Rectangle style name drawn behind the icon.
        enabled: When False the button is inert (no hover highlight, no click).
        identifier: Optional identifier to simplify UI queries.
    """
    with ui.VStack(width=20, style={"margin": 0}):
        ui.Spacer()
        with ui.ZStack(width=18, height=18, spacing=0):
            if bg_name:
                ui.Rectangle(name=bg_name)
            hover_rect = ui.Rectangle(name="button_hover", visible=False)
            with ui.VStack(width=18, height=18, spacing=0):
                ui.Spacer(height=2)
                with ui.HStack(width=16, height=16, spacing=0):
                    ui.Spacer(width=2)
                    ui.Image(name=icon_name, width=14, height=14, enabled=enabled)
                    ui.Spacer(width=1)
                ui.Spacer(height=1)
            btn = ui.InvisibleButton(identifier=identifier)
            if enabled:
                btn.set_mouse_pressed_fn(lambda x, y, b, a: on_clicked_fn())
                btn.set_mouse_hovered_fn(lambda hovered, _hr=hover_rect: setattr(_hr, "visible", hovered))
        ui.Spacer()


class TreeViewIDColumn:
    """Column that displays row indices (1, 2, 3, ...) beside the TreeView.

    Args:
        item_count: Initial row count to display.
    """

    DEFAULT_ITEM_NUM = 15

    def __init__(self, item_count: int = 0) -> None:
        self.frame = ui.Frame()
        self.item_count = item_count
        self.frame.set_build_fn(self._build_frame)

    def _build_frame(self) -> None:
        """Build the column of row number labels."""
        self.column = ui.VStack(width=25)
        with self.column:
            ui.Rectangle(name="treeview_background", height=25)
            for i in range(self.item_count):
                with ui.ZStack(height=30):
                    ui.Rectangle(name="treeview_item")
                    with ui.VStack():
                        ui.Spacer()
                        ui.Label(str(i + 1), alignment=ui.Alignment.CENTER, height=0)
                        ui.Spacer()

    def update(self, item_count: int) -> None:
        """Update the number of labeled rows and rebuild.

        Args:
            item_count: New number of rows to display.
        """
        self.item_count = item_count
        self.frame.rebuild()

    def add_item(self) -> None:
        """Increment row count and rebuild."""
        self.item_count += 1
        self.num = max(TreeViewIDColumn.DEFAULT_ITEM_NUM, self.item_count)
        self.frame.rebuild()

    def remove_item(self) -> None:
        """Decrement row count and rebuild."""
        if self.item_count > 0:
            self.item_count -= 1
        self.num = max(TreeViewIDColumn.DEFAULT_ITEM_NUM, self.item_count)
        self.frame.rebuild()


class SearchableItem(ui.AbstractItem):
    """Base item with text and filter flags for the TreeView."""

    def __init__(self) -> None:
        super().__init__()
        self.filtered_by_text = True
        self.filtered_by_condition = True
        self.text = ""

    def refresh_text(self) -> None:
        """Sync text from the model; override in subclasses."""


class PlacerHolderItem(ui.AbstractItem):
    """Placeholder row to fill empty space in the table."""

    def __init__(self) -> None:
        super().__init__()


class AddNamedPoseItem(ui.AbstractItem):
    """Special row that shows start/end site comboboxes and a '+' button."""

    def __init__(self) -> None:
        super().__init__()


class TreeViewWithPlacerHolderModel(ui.AbstractItemModel):
    """Flat-list model with placeholder rows, an 'add' row, and site options.

    Args:
        items: Initial list of items (e.g. NamedPoseItem).
        *args: Unused; for compatibility with subclasses.
    """

    def __init__(self, items: list, *args: object) -> None:
        super().__init__()
        self._children = items
        self._searchable_num = len(self._children)
        self._children.append(AddNamedPoseItem())

        self._default_children_num = 15
        self._filter_texts: list = []
        self._filter_conditions: list | None = None
        while len(self._children) < self._default_children_num:
            self._children.append(PlacerHolderItem())

        self._start_site: list = ["start site"]
        self._end_site: list = ["end site"]

        # Add-row site selections (full path strings)
        self._add_start_site_path = ""
        self._add_end_site_path = ""

        self._on_track_target_fn: Callable[..., Any] | None = None
        self._on_add_fn: Callable[..., Any] | None = None
        self._on_remove_fn: Callable[..., Any] | None = None

    @property
    def start_site(self) -> list:
        """List of start-site options for the add row combobox."""
        return self._start_site

    @start_site.setter
    def start_site(self, value: list) -> None:  # type: ignore[no-redef]
        if value == self._start_site:
            return
        self._start_site = value
        self._item_changed(None)

    @property
    def end_site(self) -> list:
        """List of end-site options for the add row combobox."""
        return self._end_site

    @end_site.setter
    def end_site(self, value: list) -> None:  # type: ignore[no-redef]
        if value == self._end_site:
            return
        self._end_site = value
        self._item_changed(None)

    def destroy(self) -> None:
        """Clear children and release the model."""
        self._children = []

    def get_item_value_model_count(self, item: object) -> int:
        """Return the number of value models per item (5 columns).

        Args:
            item: The item (unused; same count for all).

        Returns:
            5.
        """
        return 5

    def get_item_children(self, item: object) -> list:
        """Return the (possibly filtered) list of child items.

        Args:
            item: Parent item; None for root.

        Returns:
            Filtered list of children.
        """
        if item is not None:
            return []
        children = self._children
        if self._filter_texts:
            children = [child for child in children if isinstance(child, SearchableItem) and child.filtered_by_text]
        if self._filter_conditions:
            children = [
                child for child in children if isinstance(child, SearchableItem) and child.filtered_by_condition
            ]
        return children

    def add_item(self, item: SearchableItem | None = None) -> None:
        """Insert a new item or invoke add callback if registered.

        Args:
            item: Item to add, or None to use callback or create default.
        """
        if self._on_add_fn:
            self._on_add_fn(self._add_start_site_path, self._add_end_site_path)
            return
        if item is None:
            item = SearchableItem()
        self._children.remove(self._children[self._searchable_num])
        for existing_item in self._children:
            if (
                hasattr(existing_item, "name")
                and hasattr(item, "name")
                and existing_item.name.get_value_as_string() == item.name.get_value_as_string()
            ):
                self._children.remove(existing_item)
                self._searchable_num -= 1

        if len(self._children) < self._default_children_num:
            self._children[self._searchable_num] = item
            self._children.append(PlacerHolderItem())
            self._children[self._searchable_num + 1] = AddNamedPoseItem()
        else:
            self._children.append(item)
            self._children.append(AddNamedPoseItem())
        self._searchable_num += 1
        self._item_changed(None)

    def remove_item(self, item: SearchableItem) -> None:
        """Remove the item from the model and update placeholders.

        Args:
            item: The item to remove.
        """
        if item in self._children:
            self._children.remove(self._children[self._searchable_num])
            self._searchable_num -= 1
            self._children.remove(item)
            self._children[self._searchable_num] = AddNamedPoseItem()
            while len(self._children) < self._default_children_num:
                self._children.append(PlacerHolderItem())
            self._item_changed(None)

    def set_items(self, items: list) -> None:
        """Replace children with the given items and reset placeholders.

        Args:
            items: New list of items (e.g. NamedPoseItem).
        """
        self._children = list(items)
        self._searchable_num = len(items)
        self._children.append(AddNamedPoseItem())
        while len(self._children) < self._default_children_num:
            self._children.append(PlacerHolderItem())
        self._item_changed(None)

    def edit_item(self, item: SearchableItem) -> None:
        """Required by AbstractItemModel; no-op here.

        Args:
            item: The item to edit.
        """

    def filter_by_text(self, filter_texts: list) -> None:
        """Filter visible items by substring match on text.

        Args:
            filter_texts: List of substrings; items must contain all.
        """
        if not self._children:
            return
        if self._filter_texts == filter_texts:
            return
        self._filter_texts = filter_texts
        for child in self._children:
            if not isinstance(child, SearchableItem):
                continue
            if not filter_texts:
                child.filtered_by_text = True
            else:
                child.filtered_by_text = all(filter_text in child.text for filter_text in filter_texts)
        self._item_changed(None)

    def filter_by_conditions(self, conditions: list) -> None:
        """Apply a list of conditions to filter items.

        Args:
            conditions: List of filter conditions.
        """
        self._filter_conditions = conditions
        for condition in conditions:
            self.filter_by_condition(condition)
        self._item_changed(None)

    def filter_by_condition(self, condition: object) -> None:
        """Apply a single condition; no-op in base.

        Args:
            condition: Filter condition (unused in base).
        """

    def find_unique_name(self, name: str) -> str:
        """Return a unique name by appending a counter if needed.

        Args:
            name: Base name (e.g. 'named_pose').

        Returns:
            A name not yet used by any SearchableItem in the model.
        """
        counter = 1
        current_names = [item.name.get_value_as_string() for item in self._children if isinstance(item, SearchableItem)]
        new_name = f"{name}{counter}"
        while new_name in current_names:
            counter += 1
            new_name = f"{name}{counter}"
        return new_name

    def track_target(self, item: SearchableItem) -> None:
        """Toggle tracking state for an item and notify callback.

        Args:
            item: The searchable item (e.g. NamedPoseItem) to toggle.
        """
        if not isinstance(item, SearchableItem):
            return
        item.tracking = not getattr(item, "tracking", False)
        self._item_changed(item)
        if self._on_track_target_fn:
            self._on_track_target_fn(item)

    def set_add_row_defaults(self, start_path: str, end_path: str) -> None:
        """Set the default site paths for the add row without triggering a rebuild.

        Args:
            start_path: Default start site path for the add row.
            end_path: Default end site path for the add row.
        """
        self._add_start_site_path = start_path
        self._add_end_site_path = end_path


class TreeViewWithPlacerHolderDelegate(ui.AbstractItemDelegate):
    """Delegate that builds named-pose table cells (labels, combos, buttons).

    Args:
        headers: Column header strings.
        combo_ids: Column indices that use site comboboxes.
        model: The TreeViewWithPlacerHolderModel.
        on_name_changed_fn: Callback (item, old_name, new_name).
        on_apply_pose_fn: Callback (item) when apply is clicked.
        on_site_changed_fn: Callback (item) when start/end site changes.
    """

    def __init__(
        self,
        headers: list,
        combo_ids: list,
        model: TreeViewWithPlacerHolderModel,
        on_name_changed_fn: Callable[..., Any] | None = None,
        on_apply_pose_fn: Callable[..., Any] | None = None,
        on_site_changed_fn: Callable[..., Any] | None = None,
    ) -> None:
        super().__init__()
        self.subscription = None
        self.headers = headers
        self.combo_ids = combo_ids
        self.__model = model
        self.__name_sort_options_menu = None
        self._on_name_changed_fn = on_name_changed_fn
        self._on_apply_pose_fn = on_apply_pose_fn
        self._on_site_changed_fn = on_site_changed_fn
        self._site_combos: list = []

    def destroy(self) -> None:
        """Release site combos and clear the options menu."""
        self._destroy_site_combos()
        self.__name_sort_options_menu = None

    def _destroy_site_combos(self) -> None:
        """Destroy all tracked SiteSearchComboBox popup windows."""
        for combo in self._site_combos:
            combo.destroy()
        self._site_combos = []

    def build_branch(
        self,
        model: TreeViewWithPlacerHolderModel,
        item: object,
        column_id: int,
        level: int,
        expanded: bool,
    ) -> None:
        """Required by AbstractItemDelegate; no branch expansion used.

        Args:
            model: The item model.
            item: The item.
            column_id: Column index.
            level: Nesting level.
            expanded: Whether the branch is expanded.
        """

    def __build_rename_field(
        self,
        item: Any,
        item_model: Any,
        label: Any,
        value: str,
        parent_stack: Any,
    ) -> None:
        """Build inline rename field and wire end-edit and double-click to toggle edit mode.

        Args:
            item: The row item (SearchableItem).
            item_model: Value model for the name column.
            label: UI Label showing the name.
            value: Initial string value.
            parent_stack: Parent ZStack for mouse handler.
        """

        def on_end_edit(label: Any, field: Any) -> None:
            """Commit rename and notify callback; show label again.

            Args:
                label: Label widget to show.
                field: StringField widget to hide.
            """
            new_str = field.model.get_value_as_string()
            old_str = item_model.get_value_as_string()
            item_model.set_value(new_str)
            item.refresh_text()
            label.visible = True
            field.visible = False
            self.end_edit_subscription = None
            if old_str != new_str and self._on_name_changed_fn:
                self._on_name_changed_fn(item, old_str, new_str)
            # Re-read from model in case callback sanitized the name
            label.text = item_model.get_value_as_string()
            self.__model._item_changed(item)

        def on_mouse_double_clicked(button: int, label: Any, field: Any) -> None:
            """Show rename field and subscribe to end-edit; focus field next frame.

            Args:
                button: Mouse button (0 for left).
                label: Label widget to hide.
                field: StringField widget to show and focus.

            Returns:
                None.
            """
            if button != 0:
                return
            label.visible = False
            field.visible = True
            self.end_edit_subscription = field.model.subscribe_end_edit_fn(lambda _: on_end_edit(label, field))
            import omni.kit.app

            async def focus(field: Any) -> None:
                """Defer focus to the next frame so the field is ready.

                Args:
                    field: StringField to focus.
                """
                await omni.kit.app.get_app().next_update_async()
                field.focus_keyboard()

            asyncio.ensure_future(focus(field))

        field = ui.StringField(name="rename_field", visible=False, identifier="poser_rename_field")
        field.model.set_value(value)
        parent_stack.set_mouse_double_clicked_fn(lambda x, y, b, _: on_mouse_double_clicked(b, label, field))

    def build_widget(
        self,
        model: TreeViewWithPlacerHolderModel,
        item: object,
        column_id: int,
        level: int,
        expanded: bool,
    ) -> None:
        """Build the cell widget for the given item and column.

        Args:
            model: The item model.
            item: The item (SearchableItem, AddNamedPoseItem, or PlacerHolderItem).
            column_id: Column index.
            level: Nesting level.
            expanded: Whether the row is expanded.
        """
        with ui.ZStack(height=30):
            if isinstance(item, SearchableItem):
                if column_id != 0 and column_id != model.get_item_value_model_count(item) - 1:
                    ui.Rectangle(name="treeview_item")
                else:
                    ui.Rectangle(name="treeview_item_button")
            elif isinstance(item, AddNamedPoseItem):
                add_enabled = len(self.__model.start_site) > 0 and "/" in self.__model.start_site[0]
                ui.Rectangle(name="add_row")
                if add_enabled:
                    ui.Rectangle(name="add_row_highlight")

            with ui.HStack():
                if isinstance(item, SearchableItem):
                    if column_id == 0:
                        with ui.ZStack(style={"margin": 2}):
                            ui.InvisibleButton(clicked_fn=lambda: model.edit_item(item))
                            ui.Image(name="sort")
                    elif column_id == model.get_item_value_model_count(item) - 1:
                        with ui.HStack():
                            ui.Spacer()
                            _build_icon_button("remove", lambda _item=item: model.remove_item(_item), identifier="poser_remove_pose")  # type: ignore[misc]
                            ui.Spacer()
                    else:
                        item_model = model.get_item_value_model(item, column_id)
                        value = item_model.get_value_as_string()
                        if column_id in self.combo_ids:
                            id_index = self.combo_ids.index(column_id)
                            combo_lists = [self.__model.start_site, self.__model.end_site]

                            def _on_existing_site_selected(
                                val: str,
                                _item_model: Any = item_model,
                                _item: Any = item,
                            ) -> None:
                                """Update model from combo and invoke site-changed callback.

                                Args:
                                    val: Selected site path.
                                    _item_model: Value model for the cell.
                                    _item: Row item.
                                """
                                old_val = _item_model.get_value_as_string()
                                _item_model.set_value(val)
                                _item.refresh_text()
                                if old_val != val and self._on_site_changed_fn:
                                    self._on_site_changed_fn(_item)
                                model._item_changed(_item)

                            with ui.VStack():
                                ui.Spacer()
                                combo = SiteSearchComboBox(
                                    items=combo_lists[id_index],
                                    current_value=value,
                                    on_selection_changed_fn=_on_existing_site_selected,
                                    identifier=f"poser_{'start' if id_index == 0 else 'end'}_site",
                                )
                                self._site_combos.append(combo)
                                ui.Spacer()
                        else:
                            stack = ui.ZStack(height=30, style={"margin": 1})
                            with stack:
                                with ui.HStack():
                                    ui.Spacer(width=4)
                                    label = ui.Label(value)

                                    if column_id == 1:
                                        ui.Spacer()
                                        # Play (apply pose) button
                                        _build_icon_button(
                                            "play",
                                            lambda _item=item: (  # type: ignore[misc]
                                                self._on_apply_pose_fn(_item) if self._on_apply_pose_fn else None
                                            ),
                                            bg_name="play_button",
                                            identifier="poser_apply_pose",
                                        )
                                        # Track target button
                                        ui.Spacer(width=4)
                                        tracking = getattr(item, "tracking", False)
                                        _build_icon_button(
                                            "target_active" if tracking else "target",
                                            lambda _item=item: model.track_target(_item),  # type: ignore[misc]
                                            bg_name="track_target_active" if tracking else "track_target_inactive",
                                            identifier="poser_track_target",
                                        )
                                        ui.Spacer(width=2)
                                if column_id == 1:
                                    with ui.HStack():
                                        self.__build_rename_field(item, item_model, label, value, stack)
                                        ui.Spacer(width=42)

                # --- AddNamedPoseItem row: comboboxes + "+" button ---
                elif isinstance(item, AddNamedPoseItem):
                    if column_id in self.combo_ids and add_enabled:
                        id_index = self.combo_ids.index(column_id)
                        combo_lists = [self.__model.start_site, self.__model.end_site]
                        paths = [self.__model._add_start_site_path, self.__model._add_end_site_path]
                        current_path = paths[id_index]

                        def _on_add_site_selected(val: str, _id_index: int = id_index) -> None:
                            """Store selected path for the add row (start or end).

                            Args:
                                val: Selected site path.
                                _id_index: 0 for start site, 1 for end site.
                            """
                            if _id_index == 0:
                                self.__model._add_start_site_path = val
                            else:
                                self.__model._add_end_site_path = val

                        with ui.VStack():
                            ui.Spacer()
                            combo = SiteSearchComboBox(
                                items=combo_lists[id_index],
                                current_value=current_path,
                                on_selection_changed_fn=_on_add_site_selected,
                                identifier=f"poser_add_{'start' if id_index == 0 else 'end'}_site",
                            )
                            self._site_combos.append(combo)
                            ui.Spacer()
                    elif column_id == model.get_item_value_model_count(item) - 1:
                        with ui.HStack():
                            ui.Spacer(width=4)
                            if add_enabled:
                                _build_icon_button(
                                    "add", lambda: model.add_item(), enabled=add_enabled, identifier="poser_add_pose"
                                )
                            ui.Spacer()

    def build_header(self, column_id: int = 0) -> None:
        """Build the TreeView header row.

        Args:
            column_id: Column index (0-based).

        Returns:
            None.
        """
        header_widget = ui.HStack(height=25)
        with header_widget:
            if column_id == len(self.headers) + 1:
                with ui.HStack():
                    ui.Spacer(width=4)
                    with ui.VStack():
                        ui.Spacer()
                        ui.Image(name="remove_header", width=18, height=18)
                        ui.Spacer()
            elif column_id > 0:
                with ui.HStack():
                    ui.Spacer(width=4)
                    if column_id == 4:
                        ui.Spacer(width=6)
                    ui.Label(self.headers[column_id - 1], name="treeview_header")
        return header_widget
