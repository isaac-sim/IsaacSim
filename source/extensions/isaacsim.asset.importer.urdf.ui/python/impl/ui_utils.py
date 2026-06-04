# SPDX-FileCopyrightText: Copyright (c) 2023-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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
"""URDF-specific UI widgets (ROS package table)."""

import typing

import omni.ui as ui
from isaacsim.gui.components.ui_utils import add_folder_picker_icon

from .style import get_style


class RosPackageItem(ui.AbstractItem):
    """Row item for ROS package table entries.

    Args:
        name: Package name.
        path: Package path.
        row_index: Optional row index.
    """

    def __init__(self, name: str, path: str, row_index: int | None = None) -> None:
        super().__init__()
        self.name_model = ui.SimpleStringModel(name)
        self.path_model = ui.SimpleStringModel(path)
        self.row_index = row_index


class RosPackageModel(ui.AbstractItemModel):
    """Model for ROS package list entries.

    Args:
        rows: Optional list of (name, path) tuples.
    """

    def __init__(self, rows: list[tuple[str, str]] | None = None) -> None:
        super().__init__()
        self._items: list[RosPackageItem] = []
        self._row_index = 0
        if rows:
            for name, path in rows:
                self._items.append(RosPackageItem(name, path, self._row_index))
                self._row_index += 1

    def get_item_children(self, item: RosPackageItem | None) -> list[RosPackageItem]:
        """Return children for the requested item.

        Args:
            item: Item whose children are requested.

        Returns:
            List of child items for the requested item.
        """
        if item is not None:
            return []
        return self._items

    def get_item_value_model_count(self, item: RosPackageItem | None) -> int:
        """Return the number of value models per item.

        Args:
            item: Item being queried.

        Returns:
            Number of value models for the item.
        """
        return 2

    def get_item_value_model(self, item: RosPackageItem | None, column_id: int) -> ui.AbstractValueModel | None:
        """Return the value model for the given item and column.

        Args:
            item: Item whose value model is requested.
            column_id: Column index.

        Returns:
            Value model for the requested column, or None if item is missing.
        """
        if not item:
            return None
        if column_id == 0:
            return item.name_model
        return item.path_model

    def add_row(self, name: str = "", path: str = "") -> None:
        """Add a new row to the model.

        Args:
            name: Package name.
            path: Package path.
        """
        self._items.append(RosPackageItem(name, path, self._row_index))
        self._item_changed(None)

    def remove_row(self, item: RosPackageItem) -> None:
        """Remove a row from the model.

        Args:
            item: Row item to remove.
        """
        if item in self._items:
            self._items.remove(item)
            self._item_changed(None)

    def get_rows(self) -> list[tuple[str, str]]:
        """Return the list of package rows.

        Returns:
            List of (name, path) tuples.
        """
        return [(item.name_model.as_string, item.path_model.as_string) for item in self._items]


class RosPackageDelegate(ui.AbstractItemDelegate):
    """Delegate that renders ROS package table rows.

    Args:
        row_height: Row height in pixels.
        border_width: Border width in pixels.
        on_delete: Callback invoked when a row is deleted.
    """

    def __init__(self, row_height: int, border_width: int, on_delete: typing.Callable) -> None:
        super().__init__()
        self._row_height = row_height
        self._border_width = border_width
        self._on_delete = on_delete
        self._row_style = {
            "background_color": 0xFF23211F,
            "border_color": 0xFF5A5A5A,
            "border_width": border_width,
        }

    def build_branch(
        self,
        model: ui.AbstractItemModel,
        item: ui.AbstractItem,
        column_id: int,
        level: int,
        expanded: bool,
    ) -> None:
        """Build the branch cell for tree rows.

        Args:
            model: Tree view model.
            item: Current item to render.
            column_id: Column index.
            level: Tree depth level.
            expanded: Whether the row is expanded.
        """

    def build_header(self, column_id: int) -> None:
        """Build the header cell for the given column.

        Args:
            column_id: Column index for the header.
        """
        label_text = ""
        if column_id == 0:
            label_text = "Package"
        else:
            label_text = "Path"

        row_height = self._row_height
        border = self._border_width
        header_model = ui.SimpleStringModel(label_text)

        with ui.ZStack(height=row_height):
            ui.Rectangle(height=row_height, style=self._row_style)
            with ui.HStack(height=row_height, spacing=0):
                ui.Spacer(width=border, height=border)
                with ui.VStack(width=ui.Fraction(1), height=row_height):
                    ui.Spacer(height=border)
                    ui.StringField(
                        header_model,
                        width=ui.Fraction(1),
                        height=row_height - border * 2,
                        alignment=ui.Alignment.LEFT_CENTER,
                        read_only=True,
                        enabled=False,
                        identifier=f"ros_package_table_header_{label_text}",
                    )
                    ui.Spacer(height=border)
                ui.Spacer(width=border, height=border)
            ui.Rectangle(
                height=row_height,
                style={"background_color": 0x0, "border_color": 0xFF5A5A5A, "border_width": border},
            )

    def build_widget(
        self,
        model: ui.AbstractItemModel,
        item: ui.AbstractItem,
        column_id: int,
        level: int,
        expanded: bool,
    ) -> None:
        """Build a widget for the given row and column.

        Args:
            model: Tree view model.
            item: Current item to render.
            column_id: Column index.
            level: Tree depth level.
            expanded: Whether the row is expanded.

        Returns:
            None.
        """
        if not item:
            return

        row_height = self._row_height
        border = self._border_width

        with ui.ZStack(height=row_height):
            ui.Rectangle(height=row_height, style=self._row_style)
            with ui.HStack(height=row_height, spacing=0):
                ui.Spacer(width=border, height=border)
                if column_id == 0:
                    with ui.VStack(width=ui.Fraction(1), height=row_height):
                        ui.Spacer(height=border)
                        value_model = model.get_item_value_model(item, column_id)
                        ui.StringField(
                            value_model,
                            width=ui.Fraction(1),
                            height=row_height - 10,
                            alignment=ui.Alignment.LEFT_CENTER,
                            identifier=f"ros_package_table_name_field_{item.row_index}",
                        )
                        ui.Spacer(height=border)
                else:
                    with ui.VStack(width=ui.Fraction(1), height=row_height):
                        ui.Spacer(height=border)
                        value_model = model.get_item_value_model(item, column_id)
                        with ui.HStack(height=0, spacing=2):
                            ui.StringField(
                                value_model,
                                width=ui.Fraction(1),
                                height=row_height - 10,
                                alignment=ui.Alignment.LEFT_CENTER,
                                identifier=f"ros_package_table_path_field_{item.row_index}",
                            )

                            def update_field(filename: str, path: str, vm: ui.AbstractValueModel = value_model) -> None:
                                if filename == "":
                                    val = path
                                elif filename[0] != "/" and path[-1] != "/":
                                    val = path + "/" + filename
                                elif filename[0] == "/" and path[-1] == "/":
                                    val = path + filename[1:]
                                else:
                                    val = path + filename
                                vm.set_value(val)

                            add_folder_picker_icon(
                                update_field,
                                dialog_title="Select ROS Package Folder",
                                button_title="Select Folder",
                                size=16,
                            )
                            ui.Button(
                                name="Remove",
                                style_type_name_override="IconButton",
                                style=get_style(),
                                width=ui.Pixel(18),
                                height=row_height - border * 2,
                                clicked_fn=lambda i=item: self._on_delete(i),
                                alignment=ui.Alignment.CENTER,
                                identifier=f"ros_package_table_remove_button_{item.row_index}",
                            )
                ui.Spacer(width=border, height=border)
