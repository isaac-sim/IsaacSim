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

"""Provides color-coded table widget for visualizing and selecting robot joints in the gains tuner interface."""

import colorsys

import omni.ui as ui

from .base_table_widget import ITEM_HEIGHT, TableItem, TableItemDelegate, TableModel, TableWidget
from .cell_widget import CellColor


def generate_distinct_colors(n: int) -> list[list[int]]:
    """Generate n visually distinct colors and represent them in 32-bit hexadecimal format.

    Each color group contains three variations with different saturation levels (1.0, 0.4, 0.2) to provide
    visual variety for joint coloring in the robot setup gain tuner interface.

    Args:
        n: The number of color groups to generate.

    Returns:
        A list of n color groups, where each group contains three color variations represented as 32-bit
        hexadecimal integers with full alpha opacity.
    """
    group_colors = []
    for i in range(n):
        colors = []
        for s in [1, 0.4, 0.2]:
            h = i / n  # Hue
            v = 0.9  # Value
            r, g, b = colorsys.hsv_to_rgb(h, s, v)
            # Set the alpha channel to 1.0 (fully opaque)
            hex_color = f"#{255:02X}{int(r * 255):02X}{int(g * 255):02X}{int(b * 255):02X}"
            hex_color_int = int(hex_color[1:], 16)  # Convert to integer
            colors.append(hex_color_int)
        group_colors.append(colors)
    return group_colors


class ColorJointItem(TableItem):
    """Represents a joint item with color visualization for the gain tuner table.

    This class extends TableItem to provide a specialized item for displaying joint information
    along with associated color data in the gain tuner interface. Each item represents a single
    joint with its name, index, and color scheme for visual identification.

    Args:
        name: The name of the joint.
        joint_index: The index of the joint in the articulation.
        color: List of colors associated with the joint for visual representation.
    """

    def __init__(self, name: str, joint_index: int, color: list) -> None:
        super().__init__(joint_index)
        self.colors = color
        self.name = name
        self.color_cell = None

    def get_item_value(self, col_id: int = 0) -> list | str:
        """Retrieves the value for the specified column.

        Args:
            col_id: Column identifier (0 for colors, 1 for joint name).

        Returns:
            The colors list if col_id is 0, or the joint name if col_id is 1.
        """
        if col_id == 0:
            return self.colors
        elif col_id == 1:
            return self.name

    def set_item_value(self, col_id: int, value: object) -> None:
        """Sets the value for the specified column.

        Args:
            col_id: Column identifier (0 for colors, 1 for joint name).
            value: The new value to set.
        """
        if col_id == 0:
            self.colors = value
        elif col_id == 1:
            self.name = value

    def get_value_model(self, col_id: int = 0) -> None:
        """Gets the value model for the specified column.

        Args:
            col_id: Column identifier.

        Returns:
            Always returns None as no value model is used.
        """
        return


class ColorJointItemDelegate(TableItemDelegate):
    """A delegate for rendering color and joint name items in the robot joint selection table.

    This delegate handles the visual representation and interaction for table items that display
    robot joint information with associated color coding. It manages two columns: a color indicator
    column showing visual color swatches and a joint name column displaying the joint names.
    The delegate provides sorting functionality and handles selection state changes to update
    the visual appearance of color cells.

    Args:
        model: The table model containing the joint data to be displayed.
    """

    header_tooltip = ["Color", "Joint"]
    """Tooltip text for each column header in the color joint table."""
    header = ["", "Joint"]
    """Header labels for each column in the color joint table."""

    def __init__(self, model: object) -> None:
        super().__init__(model)
        self.column_headers = {}

    def init_model(self) -> None:
        """Initialize the model for the delegate."""

    def build_header(self, column_id: int = 0) -> None:
        """Build the header widget for a specific column.

        Args:
            column_id: The column identifier to build the header for.
        """
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

    def update_defaults(self) -> None:
        """Update default settings for the delegate."""

    def build_widget(
        self, model: object, item: ColorJointItem | None = None, index: int = 0, level: int = 0, expanded: bool = False
    ) -> None:
        """Build the widget for a table item at the specified column.

        Args:
            model: The table model containing the data.
            item: The color joint item to build the widget for.
            index: The column index to build the widget for.
            level: The nesting level of the item.
            expanded: Whether the item is expanded.
        """
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

    def select_changed(self, selection: list) -> None:
        """Handle changes in item selection by updating the visual state of color cells.

        Args:
            selection: The list of selected items.
        """
        for item in self.get_children():
            if item.color_cell:
                item.color_cell.selected = False
        for item in selection:
            if item.color_cell:
                item.color_cell.selected = True


class ColorJointModel(TableModel):
    """A table model for managing joint color assignments in the gains tuner interface.

    This model creates a collection of ColorJointItem instances, with each item representing a joint
    from the articulation and its associated distinct color. The colors are automatically generated
    to provide visual distinction between different joints in the gains tuning interface.

    Args:
        gains_tuner: The gains tuner instance that provides access to the articulation and its joints.
        value_changed_fn: Callback function invoked when a value in the model changes.
        **kwargs: Additional keyword arguments passed to the parent TableModel class.
    """

    def __init__(self, gains_tuner: object, value_changed_fn: callable, **kwargs: object) -> None:
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

    def get_item_value_model_count(self, item: object) -> int:
        """The number of columns.

        Args:
            item: The table item to query.

        Returns:
            The number of columns for the table.
        """
        return 2


class ColorJointWidget(TableWidget):
    """A UI widget for displaying and selecting robot joints with color-coded visualization.

    Provides a table-based interface showing robot joints with associated color indicators. Each joint is
    displayed with a unique color and its name, allowing users to visually distinguish and select joints
    for gain tuning operations. The widget automatically generates distinct colors for each joint and
    supports selection callbacks for integration with other components.

    The widget displays joints in a two-column format: a color cell showing the joint's assigned color
    and a text cell showing the joint name. Users can select joints to perform operations or highlight
    specific joints in the tuning interface.

    Args:
        gains_tuner: The gains tuner instance containing the robot articulation and joint information.
        value_changed_fn: Callback function called when widget values change.
        selected_changed_fn: Callback function called when joint selection changes.
    """

    def __init__(
        self, gains_tuner: object, value_changed_fn: callable = None, selected_changed_fn: callable = None
    ) -> None:
        self.gains_tuner = gains_tuner
        model = ColorJointModel(gains_tuner, self._on_value_changed)
        delegate = ColorJointItemDelegate(model)
        self.selected_items = []
        self.selected_changed_fn = selected_changed_fn
        super().__init__(value_changed_fn, model, delegate)

    def _build_ui(self) -> None:
        """Builds the UI components for the color joint widget.

        Creates a scrolling frame containing the tree view for displaying joint colors and names.
        """
        with ui.ScrollingFrame(
            horizontal_scrollbar_policy=ui.ScrollBarPolicy.SCROLLBAR_ALWAYS_OFF,
            vertical_scrollbar_policy=ui.ScrollBarPolicy.SCROLLBAR_AS_NEEDED,
            style_type_name_override="TreeView",
            height=ui.Fraction(1),
            width=150,
        ):
            with ui.HStack():
                self.build_tree_view()

    def build_tree_view(self) -> None:
        """Builds the tree view component for displaying joints with their colors.

        Creates a TreeView widget with fixed column widths for color and joint name display,
        sets up selection handling, and initializes the selection to the first joint.
        """
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

    def __selection_changed(self, selection: list) -> None:
        """Handles selection changes in the joint tree view.

        Updates the visual selection state of color cells and notifies the parent widget
        of the selection change through the callback function.

        Args:
            selection: The list of selected joint items in the tree view.
        """
        if not selection:
            return
        self.delegate.select_changed(selection)
        if self.selected_changed_fn:
            self.selected_changed_fn(selection)
