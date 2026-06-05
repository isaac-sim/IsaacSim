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

"""Base table widget components for the robot setup gain tuner interface."""

from enum import Enum
from functools import partial

import omni.ui as ui

ITEM_HEIGHT = 28


class SearchableItemSortPolicy(Enum):
    """Sort policy for stage items."""

    DEFAULT = 0
    """The default sort policy."""

    A_TO_Z = 1
    """Sort by name from A to Z."""

    Z_TO_A = 2
    """Sort by name from Z to A."""


class TableItem(ui.AbstractItem):
    """Table item for robot joint gain tuning interface.

    Represents a single row in the gain tuning table, containing joint-specific data and handling value change
    notifications. This class extends Omni UI's AbstractItem to provide custom table functionality for the robot
    setup gain tuner extension.

    Args:
        joint_index: Index of the joint this table item represents.
        value_changed_fn: Callback function invoked when item values change.
    """

    def __init__(self, joint_index: int, value_changed_fn: callable = None) -> None:
        super().__init__()
        self.joint_index = joint_index
        self._value_changed_fn = value_changed_fn

    def _on_value_changed(self, model: object, col_id: int = None, adjusted_col_id: int = None) -> None:
        """Handles value changes for the table item.

        Args:
            model: The model that triggered the value change.
            col_id: Column identifier where the change occurred.
            adjusted_col_id: Adjusted column identifier for the change.
        """
        if self._value_changed_fn:
            self._value_changed_fn(self, col_id, adjusted_col_id)

    def get_item_value(self, col_id: int = 0) -> None:
        """Gets the value for the specified column.

        Args:
            col_id: Column identifier to retrieve value from.
        """

    def set_item_value(self, col_id: int, value: object) -> None:
        """Sets the value for the specified column.

        Args:
            col_id: Column identifier to set value for.
            value: The value to set for the column.
        """

    def get_value_model(self, col_id: int = 0) -> None:
        """Gets the value model for the specified column.

        Args:
            col_id: Column identifier to retrieve the value model from.
        """


class TableItemDelegate(ui.AbstractItemDelegate):
    """A delegate for handling table item rendering and interaction in the gain tuner interface.

    This delegate manages the visual representation and user interactions for table items, including
    sorting functionality, header construction, and widget building. It provides sorting capabilities
    with ascending and descending options through interactive sort buttons in column headers.

    Args:
        model: The table model that provides data and manages table items.
    """

    def __init__(self, model: object) -> None:
        super().__init__()
        self._model = model
        self.__name_sort_options_menu = None
        self.__items_sort_policy = [SearchableItemSortPolicy.DEFAULT] * self._model.get_item_value_model_count(None)
        self.column_headers = {}
        self.init_model()

    def init_model(self) -> None:
        """Initializes the table model for the delegate."""

    def set_mode(self, mode: object) -> None:
        """Sets the operating mode for the table delegate.

        Args:
            mode: The mode to set for the delegate.
        """
        self.__mode = mode

    def build_branch(
        self, model: object, item: object = None, column_id: int = 0, level: int = 0, expanded: bool = False
    ) -> None:
        """Builds the branch widget for tree view items.

        Args:
            model: The item model containing the data.
            item: The current item being rendered.
            column_id: The column identifier for the branch.
            level: The nesting level of the item in the tree.
            expanded: Whether the branch is expanded.
        """

    def build_sort_button(self, column_id: int = 0) -> None:
        """Builds a sort button for the specified column.

        Args:
            column_id: The column identifier to create the sort button for.
        """
        ui.Image(
            name="sort",
            height=19,
            width=19,
            mouse_pressed_fn=lambda x, y, b, a, column_id=column_id: self.sort_button_pressed_fn(b, column_id),
        )

    def build_header(self, column_id: int = 0) -> None:
        """Builds the header widget for the specified column.

        Args:
            column_id: The column identifier to create the header for.
        """

    def update_defaults(self) -> None:
        """Updates the default values for the table delegate."""

    def build_widget(
        self, model: object, item: object = None, index: int = 0, level: int = 0, expanded: bool = False
    ) -> None:
        """Builds the widget for displaying table items.

        Args:
            model: The item model containing the data.
            item: The current item being rendered.
            index: The index of the item in the list.
            level: The nesting level of the item in the tree.
            expanded: Whether the item is expanded.
        """

    def get_children(self) -> list:
        """Gets the child items from the model.

        Returns:
            The list of child items from the associated model.
        """
        return self._model._children

    def sort_button_pressed_fn(self, b: int, column_id: int) -> None:
        """Handles sort button press events to display sorting options.

        Args:
            b: The mouse button that was pressed.
            column_id: The column identifier for the sort button that was pressed.
        """
        if b != 0:
            return

        def on_sort_policy_changed(policy: object, value: object) -> None:
            if self.__items_sort_policy[column_id] != policy:
                self.__items_sort_policy[column_id] = policy
                self._model.sort_by_name(policy, column_id)

        items_sort_policy = self.__items_sort_policy[column_id]
        self.__name_sort_options_menu = ui.Menu("Sort Options")
        with self.__name_sort_options_menu:
            ui.MenuItem("Sort By", enabled=False)
            ui.Separator()
            ui.MenuItem(
                "Ascending",
                checkable=True,
                checked=items_sort_policy == SearchableItemSortPolicy.A_TO_Z,
                checked_changed_fn=partial(on_sort_policy_changed, SearchableItemSortPolicy.A_TO_Z),
                hide_on_click=False,
            )
            ui.MenuItem(
                "Descending",
                checkable=True,
                checked=items_sort_policy == SearchableItemSortPolicy.Z_TO_A,
                checked_changed_fn=partial(on_sort_policy_changed, SearchableItemSortPolicy.Z_TO_A),
                hide_on_click=False,
            )
        self.__name_sort_options_menu.show()


class TableModel(ui.AbstractItemModel):
    """A model for managing tabular data in a tree view widget.

    This class provides functionality for organizing and displaying data in a table format with sorting capabilities.
    It manages table items, handles value change notifications, and supports different display modes.
    The model can sort items by column values in ascending or descending order and tracks changes to individual items.

    Args:
        value_changed_fn: Callback function invoked when a joint value changes.
            Receives the joint item, column ID, and optional adjusted column ID as arguments.
        **kwargs: Additional keyword arguments passed to the parent class.
    """

    def __init__(self, value_changed_fn: callable, **kwargs: object) -> None:
        super().__init__()
        self._joint_changed_fn = value_changed_fn
        self._items_sort_func = None
        self._items_sort_reversed = False
        self._children = []
        self._mode = None
        self.init_model()

    def init_model(self) -> None:
        """Initializes the table model by resetting the mode to None."""
        self._mode = None

    def _on_joint_changed(self, joint: object, col_id: int = None, adjusted_col_id: int = None) -> None:
        """Handles joint change events by invoking the registered callback function.

        Args:
            joint: The joint that has changed.
            col_id: Column identifier.
            adjusted_col_id: Adjusted column identifier.
        """
        if self._joint_changed_fn:
            self._joint_changed_fn(joint, col_id, adjusted_col_id)

    def get_item_children(self, item: object = None) -> list:
        """Returns all the children when the widget asks it.

        Args:
            item: The parent item to get children for.

        Returns:
            List of child items, sorted if a sorting function is set.
        """
        if item is not None:
            return []
        else:
            children = self._children
            if self._items_sort_func:
                children = sorted(children, key=self._items_sort_func, reverse=self._items_sort_reversed)

            return children

    def get_item_value(self, item: object, column_id: int) -> object:
        """Gets the value of a specific column for the given item.

        Args:
            item: The item to get the value from.
            column_id: The identifier of the column.

        Returns:
            The value of the specified column for the item.
        """
        if item:
            return item.get_item_value(column_id)
        return None

    def get_item_value_model_count(self, item: object) -> int:
        """The number of columns.

        Args:
            item: The item to get the column count for.

        Returns:
            The number of columns (always 0 in base implementation).
        """
        return 0

    def get_item_value_model(self, item: object, column_id: int) -> object:
        """Return value model.

            It's the object that tracks the specific value.

        Args:
            item: The item to get the value model from.
            column_id: The identifier of the column.

        Returns:
            The value model for the specified column of the item.
        """
        if item:
            return item.get_value_model(column_id)
        return None

    def sort_by_name(self, policy: object, column_id: int) -> None:
        """Sorts items by name according to the specified policy.

        Args:
            policy: The sorting policy to apply.
            column_id: The column identifier to sort by.
        """
        if policy == SearchableItemSortPolicy.Z_TO_A:
            self._items_sort_reversed = True
        else:
            self._items_sort_reversed = False
        self._items_sort_func = lambda item: self.get_item_value(item, column_id)
        self._item_changed(None)

    def set_mode(self, mode: object) -> None:
        """Sets the mode for the table model and updates all child items.

        Args:
            mode: The mode to set for the table model.
        """
        if self._mode != mode:
            self._mode = mode
            for item in self.get_item_children():
                item.mode = mode
                self._item_changed(item)
            self._item_changed(None)


class TreeViewIDItem(ui.AbstractItem):
    """A tree view item that wraps an identifier or value for display in a table widget.

    This class extends Omni UI's AbstractItem to create items suitable for ID columns in tree views.
    It converts the provided item to a string model and provides methods for accessing item values
    and models required by the tree view framework.

    Args:
        item: The identifier or value to wrap as a tree view item.
    """

    def __init__(self, item: object) -> None:
        super().__init__()
        self.model = ui.SimpleStringModel(str(item))

    def get_item_value(self, item: object, column_id: int) -> str:
        """Returns the string value for display in the tree view.

        Args:
            item: The item to get the value from.
            column_id: The column identifier.

        Returns:
            The string representation of the item's model value.
        """
        return self.model.get_value_as_string()

    def get_item_value_model_count(self, item: object) -> int:
        """Returns the number of columns available for this item.

        Args:
            item: The item to get the column count for.

        Returns:
            The number of columns (always 1).
        """
        return 1

    def get_value_model(self, item: object, column_id: int) -> ui.SimpleStringModel:
        """Returns the underlying value model for the specified column.

        Args:
            item: The item to get the value model from.
            column_id: The column identifier.

        Returns:
            The SimpleStringModel instance for this item.
        """
        return self.model

    def get_item_children(self, item: object = None) -> list:
        """Returns the child items for the tree view.

        Args:
            item: The parent item to get children from.

        Returns:
            An empty list as this item has no children.
        """
        return []


class TreeViewIDListModel(ui.AbstractItemModel):
    """A UI model for displaying a list of numbered items in a tree view.

    This class extends the Omni UI AbstractItemModel to provide a simple sequential list of numbered items,
    starting from 1. Each item is represented as a TreeViewIDItem containing a string model with the item number.
    The model is designed for use with tree view widgets that need to display row numbers or indices.

    Args:
        length: The number of items to create in the list.
    """

    def __init__(self, length: int) -> None:
        super().__init__()
        self._children = [TreeViewIDItem(item + 1) for item in range(length)]

    def get_item_children(self, item: object = None) -> list:
        """Child items for the tree view.

        Args:
            item: The parent item to get children for. If None, returns root level items.

        Returns:
            List of child items. Empty list if item is not None, otherwise returns all root items.
        """
        if item is not None:
            return []
        else:
            return self._children

    def get_item_value(self, item: object, column_id: int) -> str:
        """String value for the specified item and column.

        Args:
            item: Index of the item to get value from.
            column_id: Column identifier.

        Returns:
            String representation of the item value.
        """
        return self._children[item].model.get_value_as_string()

    def get_item_value_model_count(self, item: object) -> int:
        """Number of columns in the tree view model.

        Args:
            item: The item to get column count for.

        Returns:
            Number of columns (always 1 for this model).
        """
        return 1


class TreeViewIDColumnDelegate(ui.AbstractItemDelegate):
    """A delegate for rendering ID column items in a TreeView widget.

    This delegate handles the visual representation of ID items in a table's identification column,
    providing methods to build headers, widgets, and branches for tree view display. It manages
    the rendering of numbered ID labels centered within rectangular containers and maintains
    column header configuration.
    """

    def __init__(self) -> None:
        super().__init__()
        self.column_headers = []

    def build_branch(
        self, model: object, item: object = None, column_id: int = 0, level: int = 0, expanded: bool = False
    ) -> None:
        """Builds the branch widget for tree view items.

        Args:
            model: The tree view model containing the data.
            item: The tree item to build the branch for.
            column_id: The column identifier.
            level: The tree hierarchy level.
            expanded: Whether the branch is expanded.
        """

    def build_header(self, column_id: int = 0) -> None:
        """Builds the header widget for the ID column.

        Args:
            column_id: The column identifier.
        """
        ui.Rectangle(name="Header", style_type_name_override="TreeView", width=15, height=19)

    def build_widget(
        self, model: object, item: object = None, index: int = 0, level: int = 0, expanded: bool = False
    ) -> None:
        """Builds the widget for displaying tree view ID items.

        Args:
            model: The tree view model containing the data.
            item: The tree item to build the widget for.
            index: The item index.
            level: The tree hierarchy level.
            expanded: Whether the item is expanded.
        """
        if item:
            ui.Rectangle(name="treeview_id", height=19)
            with ui.ZStack(height=ITEM_HEIGHT):
                ui.Rectangle(name="treeview_id")
                with ui.VStack():
                    ui.Spacer()
                    ui.Label(str(item.get_item_value(item, 0)), alignment=ui.Alignment.CENTER, height=0)
                    ui.Spacer()


class TableWidget:
    """A table widget for displaying and managing joint parameter data in a tabular format.

    This widget creates a scrollable table interface with an ID column and main content area. It supports
    bulk editing operations, mode switching, and column resizing. The table displays joint parameters with
    sortable columns and provides interactive controls for parameter adjustment.

    Args:
        value_changed_fn: Callback function triggered when table values change.
        model: The table model containing the data to display.
        delegate: The item delegate responsible for rendering table cells.
        mode: The display mode for the table.
        width: The width of the table widget.
    """

    def __init__(
        self,
        value_changed_fn: callable = None,
        model: object = None,
        delegate: object = None,
        mode: object = None,
        width: object = None,
    ) -> None:
        self.model = model
        self.delegate = delegate
        self.id_model = TreeViewIDListModel(len(self.model._children))
        self.id_delegate = TreeViewIDColumnDelegate()
        self._enable_bulk_edit = True
        self._value_changed_fn = value_changed_fn
        self.mode = mode
        self._build_ui()

    def _build_ui(self) -> None:
        """Builds the table widget user interface with scrollable frame and tree views."""
        with ui.ScrollingFrame(
            horizontal_scrollbar_policy=ui.ScrollBarPolicy.SCROLLBAR_ALWAYS_OFF,
            vertical_scrollbar_policy=ui.ScrollBarPolicy.SCROLLBAR_AS_NEEDED,
            style_type_name_override="TreeView",
            width=ui.Fraction(1),
            height=ui.Fraction(1),
        ):
            with ui.HStack(width=ui.Fraction(1)):
                self.id_column = ui.TreeView(
                    self.id_model,
                    delegate=self.id_delegate,
                    alignment=ui.Alignment.CENTER_TOP,
                    column_widths=[15],
                    columns_resizable=False,
                    header_visible=True,
                    resizeable_on_columns_resized=True,
                )
                # HStack does not stretch the main TreeView without an explicit flex frame; otherwise the table
                # sizes to intrinsic width and leaves empty space (Tune Gains vs Test Gains).
                with ui.Frame(width=ui.Fraction(1), height=ui.Fraction(1)):
                    self.build_tree_view()

    def build_tree_view(self) -> None:
        """Creates the main tree view component with resizable columns and headers."""
        self.list = ui.TreeView(
            self.model,
            delegate=self.delegate,
            alignment=ui.Alignment.LEFT_TOP,
            column_widths=[ui.Fraction(1), ui.Pixel(210), ui.Pixel(210)],
            min_column_widths=[80, 80, 80],
            columns_resizable=False,
            header_visible=True,
            height=ui.Fraction(1),
        )

    def set_bulk_edit(self, enable_bulk_edit: bool) -> None:
        """Enables or disables bulk editing mode for the table.

        Args:
            enable_bulk_edit: Whether to enable bulk editing functionality.
        """
        self._enable_bulk_edit = enable_bulk_edit

    def switch_mode(self, switch: object) -> None:
        """Switches the table widget to a different operational mode.

        Args:
            switch: The new mode to switch to.
        """
        self.delegate.set_mode(switch)
        self.model.set_mode(switch)
        self.mode = switch

    def _on_value_changed(self, joint_item: object, col_id: int = 1, adjusted_col_id: int = None) -> None:
        """Handles value changes in table cells and propagates changes in bulk edit mode.

        Args:
            joint_item: The joint item that had its value changed.
            col_id: The column identifier where the change occurred.
            adjusted_col_id: Optional adjusted column identifier for the change.
        """
        if self._enable_bulk_edit:
            for item in self.list.selection:
                if item is not joint_item:
                    pass

        if self._value_changed_fn:
            self._value_changed_fn(joint_item.joint)
