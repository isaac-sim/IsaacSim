# SPDX-FileCopyrightText: Copyright (c) 2018-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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

"""Module providing custom UI widgets and components for GUI applications."""

from __future__ import annotations

from collections import namedtuple
from typing import Any

import omni
import omni.ui as ui
from omni.kit.property.usd.relationship import RelationshipTargetPicker
from omni.kit.window.popup_dialog.dialog import get_field_value


class DynamicComboBoxItem(ui.AbstractItem):
    """A UI item for use in dynamic combo box widgets.

    This class extends Omni UI's AbstractItem to create individual items that can be displayed
    in combo box controls. Each item wraps a text value in a SimpleStringModel for integration
    with the Omni UI framework.

    Args:
        text: The display text for this combo box item.
    """

    def __init__(self, text: str) -> None:
        super().__init__()
        self.model = ui.SimpleStringModel(text)


class DynamicComboBoxModel(ui.AbstractItemModel):
    """A dynamic combo box model that manages a list of selectable items for UI combo box widgets.

    This class extends omni.ui.AbstractItemModel to provide a data model for combo box UI elements
    that can be populated with a dynamic list of string options. It maintains the current selection
    state and notifies listeners when the selected item changes.

    Args:
        args: List of string values to populate the combo box options.
    """

    def __init__(self, args: list) -> None:
        super().__init__()

        self._current_index = ui.SimpleIntModel()
        self._current_index.add_value_changed_fn(lambda a: self._item_changed(None))
        self._items: list[DynamicComboBoxItem] = []
        for i in range(len(args)):
            self._items.append(DynamicComboBoxItem(args[i]))

    def get_item_children(self, item: object) -> list[DynamicComboBoxItem]:
        """Returns the list of all combo box items.

        Args:
            item: The parent item (unused in this implementation).

        Returns:
            List of all combo box items.
        """
        return self._items

    def get_item_value_model(self, item: ui.AbstractItem | None = None, column_id: int = 0) -> ui.AbstractValueModel:
        """Returns the value model for the specified item or current selection.

        Args:
            item: The item to get the value model for. If None, returns the current index model.
            column_id: The column identifier.

        Returns:
            The value model for the item or the current index model.
        """
        if item is None:
            return self._current_index
        return item.model

    def set_item_value_model(self, item: ui.AbstractItem | None = None, column_id: int = 0) -> None:
        """Sets the current index model and updates change listeners.

        Args:
            item: The item to set as the current index model.
            column_id: The column identifier.
        """
        self._current_index = item
        self._item_changed(None)
        self._current_index.add_value_changed_fn(lambda a: self._item_changed(None))


class SelectPrimWidget:
    """Modeled after FormWidget (from omni.kit.window.popup_dialog.form_dialog) to add a widget that opens relationship selector.

    Args:
        label: The label text for the widget.
        default: The default prim path value.
        tooltip: Tooltip text displayed when hovering over the label.
    """

    def __init__(self, label: str | None = None, default: str | None = None, tooltip: str = "") -> None:
        self._label = label
        self._default_path = default
        self._tooltip = tooltip

        self._build_ui()

    def _build_ui(self) -> None:
        """Builds the UI components for the prim selection widget.

        Creates a horizontal layout with a label, string field for the prim path, and an "Add" button that opens the relationship target picker.
        """
        with ui.HStack(height=0):
            ui.Label(
                self._label,
                width=ui.Percent(29),
                style_type_name_override="Field.Label",
                word_wrap=True,
                name="prefix",
                tooltip=self._tooltip,
            )
            ui.Spacer(width=ui.Percent(1))
            path_model = ui.SimpleStringModel()
            path_model.set_value(self._default_path)
            self._prim = ui.StringField(path_model, width=ui.Percent(50))
            ui.Spacer(width=ui.Percent(1))
            ui.Button("Add", width=ui.Percent(19), clicked_fn=self._on_select_prim)

    def _on_select_prim(self) -> None:
        """Opens the relationship target picker for prim selection.

        Shows the RelationshipTargetPicker dialog to allow the user to select a prim from the stage.
        """
        stage = omni.usd.get_context().get_stage()
        additional_widget_kwargs = {"target_name": "Prim"}
        self.stage_picker = RelationshipTargetPicker(
            stage,
            [],
            None,
            additional_widget_kwargs,
        )
        self.stage_picker.show(1, on_targets_selected=self._on_target_selected)

    def _on_target_selected(self, paths: list) -> None:
        """Updates the string field with the selected prim path.

        Args:
            paths: List of selected prim paths from the relationship target picker.
        """
        self._prim.model.set_value(paths[0])

    def get_value(self) -> str:
        """The current prim path value from the string field.

        Returns:
            The prim path as a string.
        """
        return self._prim.model.get_value_as_string()

    def destroy(self) -> None:
        """Cleans up the widget by clearing the prim field reference."""
        self._prim = None


class ParamWidget:
    """Modified FormWidget (from omni.kit.window.popup_dialog.form_dialog) to better format for parameter collection use.

    Args:
        field_def: A namedtuple defining the input field configuration.

    Note:
        ParamWidget.FieldDef:
            A namedtuple of (name, label, type, default value) for describing the input field,
            e.g. FormDialog.FieldDef("name", "Name:  ", omni.ui.StringField, "Bob").
    """

    FieldDef = namedtuple("FieldDef", "name label type default tooltip focused", defaults=["", False])

    def __init__(self, field_def: FieldDef) -> None:
        self._field: Any = None
        self._build_ui(field_def)

    def _build_ui(self, field_def: FieldDef) -> None:
        """Creates the user interface for the parameter widget.

        Builds a horizontal layout with a label, spacer, and input field based on the field definition.
        The field is populated with the default value if the field type supports it.

        Args:
            field_def: Field definition containing name, label, type, default value, tooltip, and focus state.
        """
        with ui.HStack(height=0):
            ui.Label(
                field_def.label,
                width=ui.Percent(29),
                style_type_name_override="Field.Label",
                word_wrap=True,
                name="prefix",
                tooltip=field_def.tooltip,
            )
            ui.Spacer(width=ui.Percent(1))
            self._field = field_def.type(width=ui.Percent(70))
            if "set_value" in dir(self._field.model):
                self._field.model.set_value(field_def.default)

    def get_value(self) -> Any:
        """Current value of the parameter field.

        Returns:
            The value from the parameter field.
        """
        return get_field_value(self._field)

    def destroy(self) -> None:
        """Cleans up the parameter widget by releasing the field reference."""
        self._field = None
