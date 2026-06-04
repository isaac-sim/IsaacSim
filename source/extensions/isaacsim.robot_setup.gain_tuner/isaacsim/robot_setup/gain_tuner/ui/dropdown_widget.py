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

"""Provides UI components and models for dropdown/combo box widgets using Omni UI."""

import omni.ui as ui

###########################################
### copied from robot_setup.wizard.utils
###########################################


class ComboListItem(ui.AbstractItem):
    """A UI item for use in combo box lists.

    This class extends omni.ui.AbstractItem to represent a single selectable item in a combo box or dropdown
    list. It wraps a string value in a ui.SimpleStringModel for integration with Omni UI components.

    Args:
        item: The string value to display in the combo box item.
    """

    def __init__(self, item: str) -> None:
        super().__init__()
        self.model = ui.SimpleStringModel(item)
        self.item = item


class ComboListModel(ui.AbstractItemModel):
    """Model for managing a list of items in a combo box UI component.

    This class extends omni.ui.AbstractItemModel to provide a data model for combo box widgets. It manages a
    list of string items, tracks the currently selected item, and provides methods to modify selections and
    retrieve item information. The model supports adding new items, refreshing the entire list, and checking
    whether the current selection matches the default value.

    Args:
        item_list: List of string items to populate the combo box.
        default_index: Index of the default selected item in the list.
    """

    def __init__(self, item_list: list, default_index: int) -> None:
        super().__init__()
        self._default_index = default_index
        self._current_index = ui.SimpleIntModel(default_index)
        self._current_index.add_value_changed_fn(self.selection_changed)
        self._item_list = item_list
        self._items = []
        if item_list:
            for item in item_list:
                self._items.append(ComboListItem(item))

    def get_item_children(self, item: object) -> list:
        """Returns the list of child items.

        Args:
            item: The parent item to get children from.

        Returns:
            The list of ComboListItem objects.
        """
        return self._items

    def get_item_value_model(self, item: object, column_id: int) -> object:
        """Returns the value model for the specified item and column.

        Args:
            item: The item to get the value model for.
            column_id: The column identifier.

        Returns:
            The current index model if item is None, otherwise the item's string model.
        """
        if item is None:
            return self._current_index
        return item.model

    def get_current_index(self) -> int:
        """Current index of the selected item.

        Returns:
            The current index as an integer.
        """
        return self._current_index.get_value_as_int()

    def set_current_index(self, index: int) -> None:
        """Sets the current index to the specified value.

        Args:
            index: The index to set as current.
        """
        self._current_index.set_value(index)

    def get_current_string(self) -> str:
        """Current string value of the selected item.

        Returns:
            The string value of the currently selected item.
        """
        return self._items[self._current_index.get_value_as_int()].model.get_value_as_string()

    def set_current_string(self, string: str) -> None:
        """Sets the current selection to the item with the matching string value.

        Args:
            string: The string value to match and set as current.
        """
        for index, item in enumerate(self._items):
            if item.model.get_value_as_string() == string:
                self.set_current_index(index)
                break

    def get_current_item(self) -> object:
        """Current item object that is selected.

        Returns:
            The item object of the currently selected item.
        """
        return self._items[self._current_index.get_value_as_int()].item

    def is_default(self) -> bool:
        """Checks if the current selection is the default index.

        Returns:
            True if the current index matches the default index.
        """
        return self.get_current_index() == self._default_index

    def add_item(self, item: str) -> None:
        """Adds a new item to the combo list.

        Args:
            item: The string item to add to the list.
        """
        self._items.append(ComboListItem(item))
        self._item_changed(None)

    def refresh_list(self, items_list: list[str]) -> None:
        """Refreshes the combo list with a new set of items.

        Args:
            items_list: The new list of string items to replace the current list.
        """
        self._items = []
        for item in items_list:
            self._items.append(ComboListItem(item))
        self._item_changed(None)

    def selection_changed(self, model: object) -> None:
        """Handles selection changes in the combo list model.

        Triggered when the current index changes and notifies listeners of the model change.

        Args:
            model: The model that triggered the selection change.
        """
        self._item_changed(None)

    def has_item(self) -> bool:
        """Whether the combo list model contains any items.

        Returns:
            True if the model has at least one item, False otherwise.
        """
        return len(self._items) > 0


def create_combo_list_model(items_list: list[str], index: int) -> "ComboListModel":
    """Creates a ComboListModel for use in combo box UI elements.

    Args:
        items_list: List of string items to populate the combo box.
        index: Default index for the initially selected item.

    Returns:
        A ComboListModel instance configured with the provided items and default selection.
    """
    return ComboListModel(items_list, index)
