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

"""Utilities for creating tree view components with searchable items, filtering, and placeholder support in Isaac Sim robot setup wizard."""

import asyncio
from enum import Enum
from functools import partial
from typing import Any

import omni.ui as ui


class TreeViewIDColumn:
    """This is the ID (first) column of the TreeView. It's not part of the treeview delegate, because it's cheaper to do.

    item remove in this way. And we don't need to update it when the treeview list is smaller than DEFAULT_ITEM_NUM.
    """

    DEFAULT_ITEM_NUM = 15
    """Default number of items to display in the ID column."""

    def __init__(self) -> None:
        self.frame = ui.Frame()
        self.num = TreeViewIDColumn.DEFAULT_ITEM_NUM
        self.frame.set_build_fn(self._build_frame)

    def _build_frame(self) -> None:
        """Builds the UI frame containing the ID column with numbered items.

        Creates a vertical stack with a background rectangle and numbered labels for each item,
        up to the current item count.
        """
        self.column = ui.VStack(width=25)
        with self.column:
            ui.Rectangle(name="treeview_background", height=25)
            for i in range(self.num):
                with ui.ZStack(height=30):
                    ui.Rectangle(name="treeview_item")
                    ui.Label(str(i + 1), alignment=ui.Alignment.CENTER, height=0)

    def add_item(self) -> None:
        """Adds a new numbered item to the ID column.

        Creates a new UI element with a rectangle background and a centered label showing
        the next sequential number.
        """
        self.num += 1
        with self.column:
            with ui.ZStack(height=30):
                ui.Rectangle(name="treeview_item")
                ui.Label(str(self.num), alignment=ui.Alignment.CENTER, height=0)

    def remove_item(self) -> None:
        """Removes the last item from the ID column if above the default count.

        Reduces the item count and rebuilds the frame only when the number of items
        exceeds the default minimum number.
        """
        if self.num > TreeViewIDColumn.DEFAULT_ITEM_NUM:
            self.num -= 1
            self.frame.rebuild()


class SearchableItemSortPolicy(Enum):
    """Sort policy for stage items."""

    DEFAULT = 0
    """The default sort policy."""

    A_TO_Z = 1
    """Sort by name from A to Z."""

    Z_TO_A = 2
    """Sort by name from Z to A."""


class SearchableItem(ui.AbstractItem):
    """Base class for items that can be searched and filtered in tree views.

    This class extends Omni UI's AbstractItem to provide filtering capabilities based on text search
    and custom conditions. Items can be filtered by matching text content and by custom filter
    conditions that subclasses can implement.

    The class maintains filtering state through boolean flags that determine whether the item
    matches current filter criteria. It also provides a text representation that can be used
    for searching and sorting operations.

    Subclasses should override the refresh_text method to update the item's text representation
    when underlying data changes.
    """

    def __init__(self) -> None:
        super().__init__()
        # filtered is True when match the filter otherwise is False
        self.filtered_by_text = True
        self.filtered_by_condition = True
        self.text = ""

    def refresh_text(self) -> None:
        """Updates the text representation of the searchable item."""


class PlacerHolderItem(ui.AbstractItem):
    """A placeholder item used in tree view models to maintain consistent visual spacing.

    This class serves as a visual placeholder in tree views when the number of actual items is less than the
    desired minimum display count. It extends ui.AbstractItem to integrate seamlessly with Omni UI tree view
    components, ensuring that tree views maintain a consistent appearance with empty rows displayed as needed.

    The placeholder items are automatically managed by TreeViewWithPlacerHolderModel to fill gaps between
    actual content and the minimum row count, providing a cleaner user interface experience.
    """

    def __init__(self) -> None:
        super().__init__()


class TreeViewWithPlacerHolderModel(ui.AbstractItemModel):
    """A tree view model that manages searchable items with placeholder support.

    This model extends the Omni UI AbstractItemModel to provide a flat list structure with filtering,
    sorting, and placeholder functionality. It maintains a minimum number of items by adding placeholder
    items when the actual item count falls below the default threshold.

    The model supports text-based filtering, conditional filtering, and sorting by item properties.
    When items are filtered or sorted, only the searchable items are affected while placeholders
    remain to maintain the visual structure.

    Args:
        items: Initial list of items to populate the model.
        *args: Additional positional arguments passed to the parent class.
    """

    def __init__(self, items: list, *args: object) -> None:
        super().__init__()

        self._children = items

        self._searchable_num = len(self._children)
        self._default_children_num = 15
        self._filter_texts = []
        self._filter_conditions = None
        self._items_sort_func = None
        self._items_sort_reversed = False

        if self._searchable_num < self._default_children_num:
            for i in range(self._default_children_num - self._searchable_num):
                self._children.append(PlacerHolderItem())

    def destroy(self) -> None:
        """Clears all the children from the model."""
        self._children = []

    def get_item_value_model_count(self, item: object) -> None:
        """The number of columns.

        Args:
            item: The item to get column count for.

        Returns:
            The number of columns.
        """
        return 8

    def get_item_children(self, item: object) -> None:
        """Returns all the children when the widget asks it.

        Args:
            item: The parent item to get children for.

        Returns:
            List of child items with applied filters and sorting.
        """
        if item is not None:
            # Since we are doing a flat list, we return the children of root only.
            # If it's not root we return.
            return []

        children = self._children
        if self._filter_texts:
            children = [child for child in children if isinstance(child, SearchableItem) and child.filtered_by_text]

        if self._filter_conditions:
            children = [
                child for child in children if isinstance(child, SearchableItem) and child.filtered_by_condition
            ]

        if self._items_sort_func:
            sortable_children = children[: self._searchable_num]
            children[: self._searchable_num] = sorted(
                sortable_children, key=self._items_sort_func, reverse=self._items_sort_reversed
            )

        return children

    def add_item(self, item: object) -> None:
        """Adds an item to the model, replacing any existing item with the same name.

        Args:
            item: The item to add to the model.
        """
        # Check if an item with the same name already exists
        for existing_item in self._children:
            if (
                hasattr(existing_item, "name")
                and hasattr(item, "name")
                and existing_item.name.get_value_as_string() == item.name.get_value_as_string()
            ):
                # If item with same name exists, remove the existing item
                self._children.remove(existing_item)
                self._searchable_num -= 1

        # If no duplicate found, add the new item
        if self._searchable_num < self._default_children_num:
            self._children[self._searchable_num] = item
        else:
            self._children.append(item)
        self._searchable_num += 1
        # trigger the delegate update
        self._item_changed(None)

    def remove_item(self, item: object, enabled: bool) -> None:
        """Removes an item from the model if enabled.

        Args:
            item: The item to remove from the model.
            enabled: Whether the removal operation is enabled.

        Returns:
            None if the removal operation is not enabled.
        """
        if not enabled:
            return
        if item in self._children:
            self._searchable_num -= 1
            self._children.remove(item)
            if self._searchable_num < self._default_children_num:
                self._children.append(PlacerHolderItem())
            self._item_changed(None)

    def edit_item(self, item: object) -> None:
        """Initiates editing of an item.

        Args:
            item: The item to edit.
        """

    def filter_by_text(self, filter_texts: list) -> None:
        """Filters items by text, showing only items that contain all filter texts.

        Args:
            filter_texts: List of text strings to filter by.

        Returns:
            None if there are no children or the filter texts have not changed.
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
        """Filters items by multiple conditions.

        Args:
            conditions: List of conditions to apply for filtering.
        """
        self._filter_conditions = conditions
        # TODO: implement the logic in subclass
        for condition in conditions:
            self.filter_by_condition(condition)
        self._item_changed(None)

    def filter_by_condition(self, condition: object) -> None:
        """Filters items by a single condition. Should be implemented in subclass.

        Args:
            condition: The condition to apply for filtering.
        """
        print("should implement the logic in subclass")

    def sort_by_name(self, policy: object, column_id: int) -> None:
        """Sorts items by name according to the specified policy.

        Args:
            policy: The sort policy to apply.
            column_id: The column ID to sort by.
        """
        if policy == SearchableItemSortPolicy.Z_TO_A:
            self._items_sort_reversed = True
        else:
            self._items_sort_reversed = False

        self._items_sort_func = lambda item: self.get_item_value_model(item, column_id).get_value_as_string().lower()
        self._item_changed(None)


class TreeViewWithPlacerHolderDelegate(ui.AbstractItemDelegate):
    """Delegate is the representation layer. TreeView calls the methods.

    of the delegate to create custom widgets for each item.

    Args:
        headers: Column headers for the tree view.
        combo_lists: Lists of options for combo box columns.
        combo_ids: Column IDs that should use combo boxes.
        model: The tree view model instance.
    """

    def __init__(self, headers: list, combo_lists: list, combo_ids: list, model: object) -> None:
        super().__init__()
        self.subscription = None
        self.headers = headers
        self.combo_lists = combo_lists
        self.combo_ids = combo_ids
        self.__model = model
        self.__name_sort_options_menu = None
        self.__items_sort_policy = [SearchableItemSortPolicy.DEFAULT] * self.__model.get_item_value_model_count(None)

    def destroy(self) -> None:
        """Cleans up resources and references held by the delegate."""
        self.__name_sort_options_menu = None

    def build_branch(self, model: object, item: object, column_id: int, level: int, expanded: bool) -> None:
        """Create a branch widget that opens or closes subtree.

        Args:
            model: The tree view model.
            item: The item to create a branch widget for.
            column_id: The column identifier.
            level: The nesting level of the item.
            expanded: Whether the branch is expanded.
        """

    def __build_rename_field(
        self, item: object, item_model: object, label: object, value: str, parent_stack: object
    ) -> None:
        """Creates a rename field widget that allows editing item names on double-click.

        Args:
            item: The item to be renamed.
            item_model: The model for the item value.
            label: The label widget to display the current value.
            value: The current value to display.
            parent_stack: The parent stack container for mouse event handling.
        """

        def on_end_edit(label: Any, field: Any) -> None:
            new_str = field.model.get_value_as_string()
            item_model.set_value(new_str)
            item.refresh_text()
            label.text = new_str
            label.visible = True
            field.visible = False
            self.end_edit_subscription = None
            self.__model._item_changed(item)

        def on_mouse_double_clicked(button: Any, label: Any, field: Any) -> None:
            if button != 0:
                return

            label.visible = False
            field.visible = True

            self.end_edit_subscription = field.model.subscribe_end_edit_fn(lambda _: on_end_edit(label, field))

            import omni.kit.app

            async def focus(field: Any) -> None:
                await omni.kit.app.get_app().next_update_async()
                field.focus_keyboard()

            asyncio.ensure_future(focus(field))

        field = ui.StringField(name="rename_field", visible=False)
        field.model.set_value(value)
        parent_stack.set_mouse_double_clicked_fn(lambda x, y, b, _: on_mouse_double_clicked(b, label, field))

    def build_widget(self, model: object, item: object, column_id: int, level: int, expanded: bool) -> None:
        """Create a widget per column per item.

        Args:
            model: The tree view model.
            item: The item to create a widget for.
            column_id: The column identifier.
            level: The nesting level of the item.
            expanded: Whether the item is expanded.
        """
        with ui.ZStack(height=30):
            if isinstance(item, SearchableItem):
                if column_id != 0 and column_id != model.get_item_value_model_count(item) - 1:
                    ui.Rectangle(name="treeview_item")
                else:
                    ui.Rectangle(name="treeview_item_button")
            with ui.HStack():
                if isinstance(item, SearchableItem):
                    if column_id == 0:
                        with ui.ZStack(style={"margin": 2}):
                            ui.InvisibleButton(clicked_fn=lambda: model.edit_item(item))
                            ui.Image(name="pencil")
                    elif column_id == model.get_item_value_model_count(item) - 1:
                        with ui.ZStack(style={"margin": 2}):
                            enabled = item.editable.get_value_as_bool()
                            ui.InvisibleButton(clicked_fn=lambda: model.remove_item(item, enabled))
                            ui.Image(name="remove", enabled=enabled)
                    else:
                        item_model = model.get_item_value_model(item, column_id)
                        value = item_model.get_value_as_string()
                        if column_id in self.combo_ids:
                            id_index = self.combo_ids.index(column_id)
                            index = self.combo_lists[id_index].index(value)
                            with ui.VStack():
                                ui.Spacer()
                                field = ui.ComboBox(index, *self.combo_lists[id_index], name="treeview", height=0)
                                ui.Spacer()

                            def update_combobox_value(combobox_model: Any) -> None:
                                idx = combobox_model.get_item_value_model().get_value_as_int()
                                value = self.combo_lists[id_index][idx]
                                item_model.set_value(value)
                                item.refresh_text()
                                model._item_changed(item)

                            field.model.add_item_changed_fn(lambda m, _: update_combobox_value(m))
                        else:
                            stack = ui.ZStack(height=30, style={"margin": 1})
                            with stack:
                                with ui.HStack():
                                    ui.Spacer(width=4)
                                    label = ui.Label(value)
                                # name column
                                if column_id == 1:
                                    self.__build_rename_field(item, item_model, label, value, stack)

    def sort_button_pressed_fn(self, b: int, column_id: int) -> None:
        """Handles sort button press events to display sorting options menu.

        Args:
            b: The mouse button that was pressed.
            column_id: The column identifier for the sort button.

        Returns:
            None if the mouse button is not the primary button.
        """
        if b != 0:
            return

        def on_sort_policy_changed(policy: Any, value: Any) -> None:
            if self.__items_sort_policy[column_id] != policy:
                self.__items_sort_policy[column_id] = policy
                self.__model.sort_by_name(policy, column_id)

        items_sort_policy = self.__items_sort_policy[column_id]
        self.__name_sort_options_menu = ui.Menu("Sort Options")
        with self.__name_sort_options_menu:
            ui.MenuItem("Sort By", enabled=False)
            ui.Separator()
            ui.MenuItem(
                "A to Z",
                checkable=True,
                checked=items_sort_policy == SearchableItemSortPolicy.A_TO_Z,
                checked_changed_fn=partial(on_sort_policy_changed, SearchableItemSortPolicy.A_TO_Z),
                hide_on_click=False,
            )
            ui.MenuItem(
                "Z to A",
                checkable=True,
                checked=items_sort_policy == SearchableItemSortPolicy.Z_TO_A,
                checked_changed_fn=partial(on_sort_policy_changed, SearchableItemSortPolicy.Z_TO_A),
                hide_on_click=False,
            )
        self.__name_sort_options_menu.show()

    def build_header(self, column_id: int = 0) -> None:
        """Creates header widgets for tree view columns.

        Args:
            column_id: The column identifier to build the header for.

        Returns:
            The created header widget.
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
                    ui.Image(
                        name="sort",
                        height=23,
                        width=23,
                        mouse_pressed_fn=lambda x, y, b, a, column_id=column_id: self.sort_button_pressed_fn(
                            b, column_id
                        ),
                    )
        return header_widget
