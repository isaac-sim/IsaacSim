# SPDX-FileCopyrightText: Copyright (c) 2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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

import omni.ui as ui

###########################################
### copied from robot_setup.wizard.utils
###########################################


class ComboListItem(ui.AbstractItem):
    def __init__(self, item):
        """
        item is a string
        """
        super().__init__()
        self.model = ui.SimpleStringModel(item)
        self.item = item


class ComboListModel(ui.AbstractItemModel):
    def __init__(self, item_list, default_index):
        super().__init__()
        self._default_index = default_index
        self._current_index = ui.SimpleIntModel(default_index)
        self._current_index.add_value_changed_fn(self.selection_changed)
        self._item_list = item_list
        self._items = []
        if item_list:
            for item in item_list:
                self._items.append(ComboListItem(item))

    def get_item_children(self, item):
        return self._items

    def get_item_value_model(self, item, column_id):
        if item is None:
            return self._current_index
        return item.model

    def get_current_index(self):
        return self._current_index.get_value_as_int()

    def set_current_index(self, index):
        self._current_index.set_value(index)

    def get_current_string(self):
        return self._items[self._current_index.get_value_as_int()].model.get_value_as_string()

    def set_current_string(self, string):
        for index, item in enumerate(self._items):
            if item.model.get_value_as_string() == string:
                self.set_current_index(index)
                break

    def get_current_item(self):
        return self._items[self._current_index.get_value_as_int()].item

    def is_default(self):
        return self.get_current_index() == self._default_index

    def add_item(self, item):
        self._items.append(ComboListItem(item))
        self._item_changed(None)

    def refresh_list(self, items_list):
        self._items = []
        for item in items_list:
            self._items.append(ComboListItem(item))
        self._item_changed(None)

    def selection_changed(self, model):
        self._item_changed(None)

    def has_item(self):
        return len(self._items) > 0


def create_combo_list_model(items_list, index):
    return ComboListModel(items_list, index)
