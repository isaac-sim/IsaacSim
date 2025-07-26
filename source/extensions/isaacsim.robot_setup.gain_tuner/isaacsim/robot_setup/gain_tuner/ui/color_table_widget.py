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
import colorsys
from enum import Enum
from functools import partial

import omni.ui as ui

from .base_table_widget import ITEM_HEIGHT, TableItem, TableItemDelegate, TableModel, TableWidget
from .cell_widget import CellColor
from .style import get_style


def generate_distinct_colors(n):
    """
    Generate n visually distinct colors and represent them in 32-bit hexadecimal format.

    :param n: The number of colors to generate
    :return: A list of n colors, each represented as a 32-bit hexadecimal integer
    """
    group_colors = []
    for i in range(n):
        colors = []
        for s in [1, 0.4, 0.2]:
            h = i / n  # Hue
            v = 0.9  # Value
            r, g, b = colorsys.hsv_to_rgb(h, s, v)
            # Set the alpha channel to 1.0 (fully opaque)
            hex_color = "#{:02X}{:02X}{:02X}{:02X}".format(255, int(r * 255), int(g * 255), int(b * 255))
            hex_color_int = int(hex_color[1:], 16)  # Convert to integer
            colors.append(hex_color_int)
        group_colors.append(colors)
    return group_colors


class ColorJointItem(TableItem):

    def __init__(self, name, joint_index, color):
        super().__init__(joint_index)
        self.colors = color
        self.name = name
        self.color_cell = None

    def get_item_value(self, col_id=0):
        if col_id == 0:
            return self.colors
        elif col_id == 1:
            return self.name

    def set_item_value(self, col_id, value):
        if col_id == 0:
            self.colors = value
        elif col_id == 1:
            self.name = value

    def get_value_model(self, col_id=0):
        return None


class ColorJointItemDelegate(TableItemDelegate):
    header_tooltip = ["Color", "Joint"]
    header = ["", "Joint"]

    def __init__(self, model):
        super().__init__(model)
        self.column_headers = {}

    def init_model(self):
        pass

    def build_header(self, column_id=0):
        alignment = ui.Alignment.LEFT
        if column_id == 0:
            with ui.HStack():
                ui.Spacer()
                self.build_sort_button(column_id)
        elif column_id == 1:
            self.column_headers[column_id] = ui.HStack()
            with self.column_headers[column_id]:
                with ui.VStack():
                    ui.Spacer(height=ui.Pixel(3))
                    ui.Label(
                        ColorJointItemDelegate.header[1],
                        tooltip=ColorJointItemDelegate.header_tooltip[1],
                        elided_text=True,
                        name="header",
                        style_type_name_override="TreeView",
                        alignment=alignment,
                    )
                self.build_sort_button(column_id)

    def update_defaults(self):
        pass

    def build_widget(self, model, item=None, index=0, level=0, expanded=False):
        if item:
            if index == 0:
                with ui.ZStack(height=ITEM_HEIGHT):
                    ui.Rectangle(name="treeview_item")
                    with ui.VStack():
                        ui.Spacer()
                        item.color_cell = CellColor(item.colors)
                        ui.Spacer()

            elif index == 1:
                with ui.ZStack(height=ITEM_HEIGHT, style_type_name_override="TreeView"):
                    ui.Rectangle(name="treeview_first_item")
                    with ui.VStack():
                        ui.Spacer()
                        with ui.HStack(height=0):
                            ui.Label(
                                item.name,
                                tooltip=item.name,
                                name="treeview_item",
                                elided_text=True,
                                height=0,
                            )
                            ui.Spacer(width=1)
                        ui.Spacer()

    def select_changed(self, selection):
        for item in self.get_children():
            if item.color_cell:
                item.color_cell.selected = False
        for item in selection:
            if item.color_cell:
                item.color_cell.selected = True


class ColorJointModel(TableModel):
    def __init__(self, gains_tuner, value_changed_fn, **kwargs):
        super().__init__(value_changed_fn)
        self.gains_tuner = gains_tuner
        colors = generate_distinct_colors(self.gains_tuner.get_articulation().num_dofs)
        self._children = [
            ColorJointItem(
                self.gains_tuner.get_articulation().dof_names[joint_index],
                joint_index,
                colors[joint_index],
            )
            for joint_index in range(self.gains_tuner.get_articulation().num_dofs)
        ]

    def get_item_value_model_count(self, item):
        """The number of columns"""
        return 2


class ColorJointWidget(TableWidget):
    def __init__(self, gains_tuner, value_changed_fn=None, selected_changed_fn=None):
        self.gains_tuner = gains_tuner
        model = ColorJointModel(gains_tuner, self._on_value_changed)
        delegate = ColorJointItemDelegate(model)
        self.selected_items = []
        self.selected_changed_fn = selected_changed_fn
        super().__init__(value_changed_fn, model, delegate)

    def _build_ui(self):
        with ui.ScrollingFrame(
            horizontal_scrollbar_policy=ui.ScrollBarPolicy.SCROLLBAR_ALWAYS_OFF,
            vertical_scrollbar_policy=ui.ScrollBarPolicy.SCROLLBAR_AS_NEEDED,
            style_type_name_override="TreeView",
            height=ui.Fraction(1),
            width=150,
        ):
            with ui.HStack():
                self.build_tree_view()

    def build_tree_view(self):
        self.list = ui.TreeView(
            self.model,
            delegate=self.delegate,
            alignment=ui.Alignment.LEFT_TOP,
            column_widths=[ui.Pixel(26), ui.Pixel(110)],
            min_column_widths=[26, 80],
            columns_resizable=False,
            selection_changed_fn=self.__selection_changed,
            header_visible=True,
            height=ui.Fraction(1),
            width=120,
        )

        # set the default selection to the first item
        default_item = self.model.get_item_children()[0]
        self.list.selection = [default_item]

    def __selection_changed(self, selection):
        if not selection:
            return
        self.delegate.select_changed(selection)
        if self.selected_changed_fn:
            self.selected_changed_fn(selection)
