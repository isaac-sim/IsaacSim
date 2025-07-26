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
from enum import Enum
from functools import partial

import omni.ui as ui

from .cell_widget import CellLabelField

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

    def __init__(self, joint_index, value_changed_fn=None):
        super().__init__()
        self.joint_index = joint_index
        self._value_changed_fn = value_changed_fn

    def _on_value_changed(self, model, col_id=None, adjusted_col_id=None):
        if self._value_changed_fn:
            self._value_changed_fn(self, col_id, adjusted_col_id)

    def get_item_value(self, col_id=0):
        pass

    def set_item_value(self, col_id, value):
        pass

    def get_value_model(self, col_id=0):
        pass


class TableItemDelegate(ui.AbstractItemDelegate):
    def __init__(self, model):
        super().__init__()
        self._model = model
        self.__name_sort_options_menu = None
        self.__items_sort_policy = [SearchableItemSortPolicy.DEFAULT] * self._model.get_item_value_model_count(None)
        self.column_headers = {}
        self.init_model()

    def init_model(self):
        pass

    def set_mode(self, mode):
        self.__mode = mode

    def build_branch(self, model, item=None, column_id=0, level=0, expanded=False):
        pass

    def build_sort_button(self, column_id=0):
        ui.Image(
            name="sort",
            height=19,
            width=19,
            mouse_pressed_fn=lambda x, y, b, a, column_id=column_id: self.sort_button_pressed_fn(b, column_id),
        )

    def build_header(self, column_id=0):
        pass

    def update_defaults(self):
        pass

    def build_widget(self, model, item=None, index=0, level=0, expanded=False):
        pass

    def get_children(self):
        return self._model._children

    def sort_button_pressed_fn(self, b, column_id):
        if b != 0:
            return

        def on_sort_policy_changed(policy, value):
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
    def __init__(self, value_changed_fn, **kwargs):
        super().__init__()
        self._joint_changed_fn = value_changed_fn
        self._items_sort_func = None
        self._items_sort_reversed = False
        self._children = []
        self._mode = None
        self.init_model()

    def init_model(self):
        self._mode = None

    def _on_joint_changed(self, joint, col_id=None, adjusted_col_id=None):
        if self._joint_changed_fn:
            self._joint_changed_fn(joint, col_id, adjusted_col_id)

    def get_item_children(self, item=None):
        """Returns all the children when the widget asks it."""
        if item is not None:
            return []
        else:
            children = self._children
            if self._items_sort_func:
                children = sorted(children, key=self._items_sort_func, reverse=self._items_sort_reversed)

            return children

    def get_item_value(self, item, column_id):
        if item:
            return item.get_item_value(column_id)

    def get_item_value_model_count(self, item):
        """The number of columns"""
        return 0

    def get_item_value_model(self, item, column_id):
        """
        Return value model.
        It's the object that tracks the specific value.
        """
        if item:
            return item.get_value_model(column_id)

    def sort_by_name(self, policy, column_id):
        if policy == SearchableItemSortPolicy.Z_TO_A:
            self._items_sort_reversed = True
        else:
            self._items_sort_reversed = False
        self._items_sort_func = lambda item: self.get_item_value(item, column_id)
        self._item_changed(None)

    def set_mode(self, mode):
        if self._mode != mode:
            self._mode = mode
            for item in self.get_item_children():
                item.mode = mode
                self._item_changed(item)
            self._item_changed(None)


class TreeViewIDItem(ui.AbstractItem):
    def __init__(self, item):
        super().__init__()
        self.model = ui.SimpleStringModel(str(item))

    def get_item_value(self, item, column_id):
        return self.model.get_value_as_string()

    def get_item_value_model_count(self, item):
        return 1

    def get_value_model(self, item, column_id):
        return self.model

    def get_item_children(self, item=None):
        return []


class TreeViewIDListModel(ui.AbstractItemModel):
    def __init__(self, length):
        super().__init__()
        self._children = [TreeViewIDItem(item + 1) for item in range(length)]

    def get_item_children(self, item=None):
        if item is not None:
            return []
        else:
            return self._children

    def get_item_value(self, item, column_id):
        return self._children[item].model.get_value_as_string()

    def get_item_value_model_count(self, item):
        return 1


class TreeViewIDColumnDelegate(ui.AbstractItemDelegate):

    def __init__(self):
        super().__init__()
        self.column_headers = []

    def build_branch(self, model, item=None, column_id=0, level=0, expanded=False):
        pass

    def build_header(self, column_id=0):
        ui.Rectangle(name="Header", style_type_name_override="TreeView", width=15, height=19)

    def build_widget(self, model, item=None, index: int = 0, level: int = 0, expanded: bool = False) -> None:
        if item:
            ui.Rectangle(name="treeview_id", height=19)
            with ui.ZStack(height=ITEM_HEIGHT):
                ui.Rectangle(name="treeview_id")
                with ui.VStack():
                    ui.Spacer()
                    ui.Label(str(item.get_item_value(item, 0)), alignment=ui.Alignment.CENTER, height=0)
                    ui.Spacer()


class TableWidget:
    def __init__(self, value_changed_fn=None, model=None, delegate=None, mode=None, width=None):
        self.model = model
        self.delegate = delegate
        self.id_model = TreeViewIDListModel(len(self.model._children))
        self.id_delegate = TreeViewIDColumnDelegate()
        self._enable_bulk_edit = True
        self._value_changed_fn = value_changed_fn
        self.mode = mode
        self._build_ui()

    def _build_ui(self):
        with ui.ScrollingFrame(
            horizontal_scrollbar_policy=ui.ScrollBarPolicy.SCROLLBAR_ALWAYS_OFF,
            vertical_scrollbar_policy=ui.ScrollBarPolicy.SCROLLBAR_AS_NEEDED,
            style_type_name_override="TreeView",
        ):
            with ui.HStack():
                self.id_column = ui.TreeView(
                    self.id_model,
                    delegate=self.id_delegate,
                    alignment=ui.Alignment.CENTER_TOP,
                    column_widths=[15],
                    columns_resizable=False,
                    header_visible=True,
                    resizeable_on_columns_resized=True,
                )
                self.build_tree_view()

    def build_tree_view(self):
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

    def set_bulk_edit(self, enable_bulk_edit: bool):
        self._enable_bulk_edit = enable_bulk_edit

    def switch_mode(self, switch):
        self.delegate.set_mode(switch)
        self.model.set_mode(switch)
        self.mode = switch

    def _on_value_changed(self, joint_item, col_id=1, adjusted_col_id=None):
        if self._enable_bulk_edit:
            for item in self.list.selection:
                if item is not joint_item:
                    pass

        if self._value_changed_fn:
            self._value_changed_fn(joint_item.joint)
