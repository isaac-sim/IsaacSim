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

"""Provides UI components with reset functionality for input fields, combo boxes, and labels that can revert to initial values."""

from typing import Any

import omni.ui as ui


class ResetButton:
    """A UI reset button component that appears when a value differs from its initial state.

    This button provides visual feedback when a value has been modified from its original state
    and allows users to restore the original value with a single click. The button automatically
    shows or hides based on whether the current value matches the initial value.

    Args:
        init_value: The initial value used for comparison to determine when the reset button should be visible.
        on_reset_fn: Callback function executed when the reset button is clicked.
    """

    def __init__(self, init_value: object, on_reset_fn: callable) -> None:
        self._init_value = init_value
        self._on_reset_fn = on_reset_fn
        self._enable = True
        self._build_ui()

    @property
    def enable(self) -> bool:
        """Current enable state of the reset button."""
        return self._enable

    @enable.setter
    def enable(self, enable: Any) -> None:
        self._enable = enable
        self._reset_button.enabled = enable

    def refresh(self, new_value: object) -> None:
        """Updates the reset button visibility based on value changes.

        Args:
            new_value: The new value to compare against the initial value.
        """
        self._reset_button.visible = bool(self._init_value != new_value)

    def _build_ui(self) -> None:
        """Constructs the reset button UI components including the visual indicator and button."""
        with ui.VStack(width=0, height=0):
            ui.Spacer()
            with ui.ZStack(width=15, height=15):
                with ui.HStack(style={"margin_width": 0}):
                    ui.Spacer()
                    with ui.VStack(width=0):
                        ui.Spacer()
                        ui.Rectangle(width=5, height=5, name="reset_invalid")
                        ui.Spacer()
                    ui.Spacer()
                self._reset_button = ui.Rectangle(width=12, height=12, name="reset", tooltip="Click to reset value")
                self._reset_button.visible = False
            self._reset_button.set_mouse_pressed_fn(lambda x, y, m, w: self._restore_defaults())
            ui.Spacer()

    def _restore_defaults(self) -> None:
        """Restores the initial value by hiding the reset button and executing the reset callback.

        Returns:
            None if the button is not enabled.
        """
        if not self._enable:
            return
        self._reset_button.visible = False
        if self._on_reset_fn:
            self._on_reset_fn()


class ResetableField:
    """A UI field widget that supports resetting to its initial value.

    Provides an input field with an integrated reset button that appears when the value changes
    from its initial state. The reset button allows users to quickly restore the original value.

    Args:
        value_model: The data model that stores and manages the field's value.
        field_type: The UI field type to create (e.g., ui.StringField, ui.IntField).
        alignment: Text alignment within the field.
    """

    def __init__(self, value_model: object, field_type: type, alignment: object = ui.Alignment.LEFT) -> None:
        self._value_model = value_model
        self._init_value = self.get_model_value(value_model)
        self._field_type = field_type
        self._alignment = alignment
        self._enable = True
        self._build_ui()

    @property
    def enable(self) -> bool:
        """Current enabled state of the resetable field.

        Returns:
            True if the field is enabled, False otherwise.
        """
        return self._enable

    @enable.setter
    def enable(self, enable: Any) -> None:
        self._enable = enable
        self._field.enabled = enable
        self._reset_button.enable = enable

    def get_model_value(self, model: object) -> None:
        """Extracts the appropriate value from the given model based on its type.

        Args:
            model: The UI model to extract the value from.

        Returns:
            The extracted value as string, int, or float depending on model type, or empty string if unsupported.
        """
        if isinstance(model, ui.SimpleStringModel):
            return model.get_value_as_string()
        if isinstance(model, ui.SimpleIntModel):
            return model.get_value_as_int()
        if isinstance(model, ui.SimpleFloatModel):
            return model.get_value_as_float()
        return ""

    def _build_ui(self) -> None:
        """Constructs the UI components for the resetable field including the input field and reset button."""
        with ui.HStack(height=22, spacing=10):
            with ui.ZStack():
                self._field = self._field_type(name="resetable", alignment=self._alignment)
            self._field.model.set_value(self._init_value)
            self._field.model.add_value_changed_fn(lambda m: self._update_value(m))
            with ui.VStack(width=8):
                ui.Spacer()
                self._reset_button = ResetButton(self._init_value, self._on_reset_fn)
                ui.Spacer()

    def _on_reset_fn(self) -> None:
        """Resets the field value to its initial state when the reset button is clicked."""
        current_value = self.get_model_value(self._field.model)
        if current_value != self._init_value:
            self._field.model.set_value(self._init_value)
            self._value_model.set_value(self._init_value)

    def _update_value(self, model: object) -> None:
        """Updates the value model and reset button state when the field value changes.

        Args:
            model: The UI model containing the new value.
        """
        new_value = self.get_model_value(model)
        self._value_model.set_value(new_value)
        self._reset_button.refresh(new_value)


class ResetableComboBox:
    """A combo box widget with reset functionality that allows users to select from predefined values.

    Provides a dropdown selection interface with an integrated reset button that appears when the current
    selection differs from the initial value. Users can restore the original selection by clicking the reset
    button.

    Args:
        value_model: The data model that stores and manages the selected string value.
        values: List of string options available for selection in the dropdown.
        on_change: Callback function invoked when the selection changes or when reset is performed.
    """

    def __init__(self, value_model: object, values: list, on_change: callable) -> None:
        self._value_model = value_model
        self._init_value = value_model.get_value_as_string()
        self._values = values
        self._init_index = values.index(self._init_value)
        self._on_change = on_change
        self._build_ui()

    def _build_ui(self) -> None:
        """Builds the user interface for the resetable combo box.

        Creates a horizontal layout containing the combo box widget and a reset button. The combo box is populated with the provided values and the reset button is positioned to the right.
        """
        with ui.HStack(height=22, spacing=10):
            self._box = ui.ComboBox(self._init_index, *self._values)
            self._box.model.add_item_changed_fn(lambda m, i: self._update_value(m, i))
            with ui.VStack(width=8):
                ui.Spacer()
                self._reset_button = ResetButton(self._init_index, self._on_reset_fn)
                ui.Spacer()

    def _on_reset_fn(self) -> None:
        """Handles the reset button click event.

        Resets the combo box to its initial index and value if it has been changed, then triggers the on_change callback if provided.
        """
        if self._current_index != self._init_index:
            self._box.model.get_item_value_model().set_value(self._init_index)
            self._value_model.set_value(self._init_value)
        if self._on_change:
            self._on_change()

    def _update_value(self, model: object, item: object) -> None:
        """Updates the combo box value when selection changes.

        Retrieves the selected index from the model, updates the current index, sets the corresponding string value in the value model, refreshes the reset button visibility, and triggers the on_change callback.

        Args:
            model: The combo box model containing the item data.
            item: The selected item in the combo box.
        """
        root_model = model.get_item_value_model(item)
        index = root_model.get_value_as_int()
        self._current_index = index
        value_str = self._values[index]
        self._value_model.set_value(value_str)
        self._reset_button.refresh(index)
        if self._on_change:
            self._on_change()


class ResetableLabelField:
    """A UI field that displays a formatted label and switches to an editable input field when clicked.

    This component provides a label-style display that shows a formatted numeric value (with one decimal place)
    and transforms into an editable field when the user clicks on it. It includes a reset button that appears
    when the value differs from the initial value, allowing users to restore the original value.

    The field automatically switches between label and input modes: clicking the label reveals the editable
    field, and completing the edit (pressing Enter or losing focus) returns to label display mode.

    Args:
        value_model: The data model that stores and manages the field's value. Supports SimpleStringModel,
            SimpleIntModel, and SimpleFloatModel.
        field_type: The Omni UI field type to use for editing (e.g., ui.FloatField, ui.StringField).
        alignment: Text alignment for the field display.
    """

    def __init__(self, value_model: object, field_type: type, alignment: object = ui.Alignment.RIGHT) -> None:
        self._value_model = value_model
        self._init_value = self.get_model_value(value_model)
        self._field_type = field_type
        self._alignment = alignment
        self._enable = True
        self._build_ui()

    @property
    def enable(self) -> bool:
        """Whether the field is enabled for user interaction.

        Returns:
            True if the field is enabled, False otherwise.
        """
        return self._enable

    @enable.setter
    def enable(self, enable: Any) -> None:
        self._enable = enable
        self._field.enabled = enable
        self._reset_button.enable = enable
        self._rect.enabled = enable
        self._label.enabled = enable
        # self._label_degree.enabled = enable

    def get_model_value(self, model: object) -> None:
        """Extracts the appropriate value from different model types.

        Args:
            model: The model to extract the value from.

        Returns:
            The extracted value as string, int, or float depending on model type.
        """
        if isinstance(model, ui.SimpleStringModel):
            return model.get_value_as_string()
        if isinstance(model, ui.SimpleIntModel):
            return model.get_value_as_int()
        if isinstance(model, ui.SimpleFloatModel):
            return model.get_value_as_float()
        return ""

    def _build_ui(self) -> None:
        """Constructs the UI components including the field, label overlay, and reset button."""
        with ui.HStack(height=22, spacing=10):
            with ui.ZStack():
                self._field = self._field_type(name="resetable", alignment=self._alignment)
                self._rect = ui.Rectangle(
                    name="reset_mask",
                )
                self._rect.set_mouse_pressed_fn(lambda x, y, b, m: self._begin_edit())
                with ui.HStack():
                    ui.Spacer()
                    self._label = ui.Label(
                        format(self._init_value, ".1f"), name="resetable", alignment=self._alignment, width=0
                    )
                    # ui.Spacer(width=3)
                    # self._label_degree = ui.Label("o", name="degree", alignment=ui.Alignment.RIGHT_TOP, width=0)
                    ui.Spacer(width=4)
            self._field.model.set_value(self._init_value)
            self._field.model.add_value_changed_fn(lambda m: self._update_value(m))
            self.subscription = self._field.model.subscribe_end_edit_fn(lambda m: self._end_edit(m))
            self._field.visible = False
            with ui.VStack(width=8):
                ui.Spacer()
                self._reset_button = ResetButton(self._init_value, self._on_reset_fn)
                ui.Spacer()

    def _on_reset_fn(self) -> None:
        """Handles the reset button click by restoring the field to its initial value."""
        current_value = self.get_model_value(self._field.model)
        if current_value != self._init_value:
            self._field.model.set_value(self._init_value)
            self._value_model.set_value(self._init_value)
            self._label.text = format(self._init_value, ".1f")

    def _update_value(self, model: object) -> None:
        """Updates the label display and value model when the field value changes.

        Args:
            model: The field model containing the new value.
        """
        new_value = self.get_model_value(model)
        self._label.text = format(new_value, ".1f")
        self._value_model.set_value(new_value)
        self._reset_button.refresh(new_value)

    def _end_edit(self, model: object) -> None:
        """Switches from edit mode back to label display mode.

        Args:
            model: The field model that finished editing.
        """
        self._rect.visible = True
        self._label.visible = True
        # self._label_degree.visible = True
        self._field.visible = False

    def _begin_edit(self) -> None:
        """Switches from label display mode to edit mode when the field is clicked.

        Returns:
            None if the field is not enabled.
        """
        if not self._enable:
            return
        self._rect.visible = False
        self._label.visible = False
        # self._label_degree.visible = False
        self._field.visible = True
