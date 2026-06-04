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

"""Provides UI components for creating resettable input fields with reset functionality."""

import omni.ui as ui


class ResetButton:
    """A UI button component that allows users to reset a value to its initial state.

    This class creates a visual reset button that becomes visible when the current value differs from the initial
    value. The button provides a tooltip and visual feedback, and when clicked, executes a callback function to
    restore the value to its original state.

    Args:
        init_value: The initial value that the button will reset to.
        on_reset_fn: Callback function to execute when the reset button is clicked.
    """

    def __init__(self, init_value: object, on_reset_fn: callable) -> None:
        self._init_value = init_value
        self._on_reset_fn = on_reset_fn
        self._enable = True
        self._build_ui()

    @property
    def enable(self) -> bool:
        """Whether the reset button is enabled."""
        return self._enable

    @enable.setter
    def enable(self, enable: bool) -> None:
        self._enable = enable
        self._reset_button.enabled = enable

    def refresh(self, new_value: object) -> None:
        """Updates the visibility of the reset button based on value changes.

        Args:
            new_value: The new value to compare against the initial value.
        """
        self._reset_button.visible = bool(self._init_value != new_value)

    def _build_ui(self) -> None:
        """Constructs the reset button UI with layered visual elements."""
        with ui.VStack(width=0, height=0):
            ui.Spacer()
            with ui.ZStack(width=15, height=15):
                with ui.HStack():
                    ui.Spacer()
                    with ui.VStack(width=0):
                        ui.Spacer()
                        ui.Rectangle(width=5, height=5, name="reset_invalid")
                        ui.Spacer()
                    ui.Spacer()
                with ui.HStack(width=15, height=15):
                    ui.Spacer(width=2)
                    with ui.VStack(height=15):
                        ui.Spacer()
                        self._reset_real_button = ui.Button(
                            " ", width=10, height=10, name="reset", clicked_fn=lambda: self._restore_defaults()
                        )
                        ui.Spacer()
                    ui.Spacer(width=2)
                with ui.HStack(width=15, height=15):
                    ui.Spacer(width=2)
                    with ui.VStack(height=15):
                        ui.Spacer(height=5)
                        self._reset_button = ui.Rectangle(
                            width=10, height=10, name="reset", tooltip="Click to reset value"
                        )
                        ui.Spacer(height=5)
                    ui.Spacer(width=2)
                self._reset_button.visible = False
            # self._reset_button.set_mouse_pressed_fn(lambda x, y, m, w: self._restore_defaults())
            ui.Spacer()

    def _restore_defaults(self) -> None:
        """Restores the value to its initial state and triggers the reset callback."""
        if not self._enable:
            return
        self._reset_button.visible = False
        if self._on_reset_fn:
            self._on_reset_fn()


class ResetableLabelField:
    """A resettable input field with an associated reset button for UI value management.

    This class creates a UI field that can be reset to its initial value. It displays a reset button when
    the current value differs from the initial value, allowing users to quickly restore the original value.
    The field automatically synchronizes with its value model and handles value changes bidirectionally.

    Args:
        value_model: The data model that stores and manages the field's value. Can be a SimpleStringModel,
            SimpleIntModel, or SimpleFloatModel.
        field_type: The type of UI field to create (e.g., ui.StringField, ui.FloatField).
        format: Format specification for displaying the field value.
        alignment: Text alignment within the field.
    """

    def __init__(
        self, value_model: object, field_type: object, format: str, alignment: object = ui.Alignment.RIGHT_CENTER
    ) -> None:
        self._value_model = value_model
        self._init_value = self.get_model_value(value_model)
        self._field_type = field_type
        self._alignment = alignment
        self._enable = True
        self._frame = ui.HStack(height=26, spacing=2)
        self._format = format
        self._field = None
        self._build_ui()

    @property
    def enabled(self) -> bool:
        """Whether the field is enabled for user interaction.

        Returns:
            True if the field is enabled, False otherwise.
        """
        return self._enable

    @property
    def field(self) -> object:
        """The UI field widget used for value input.

        Returns:
            The field widget instance.
        """
        return self._field

    def update_default_value(self) -> None:
        """Updates the default value used by the reset button to the current model value."""
        self._init_value = self.get_model_value(self._value_model)
        self._reset_button._init_value = self._init_value
        self._reset_button.refresh(self._init_value)

    @property
    def visible(self) -> bool:
        """Whether the field frame is visible.

        Returns:
            True if the field frame is visible, False otherwise.
        """
        return self._frame.visible

    @visible.setter
    def visible(self, value: bool) -> None:
        self._frame.visible = value

    @enabled.setter
    def enabled(self, enable: bool) -> None:
        self._enable = enable
        self._field.enabled = enable
        self._reset_button.enable = enable

    def get_model_value(self, model: object) -> str | int | float:
        """Extracts the appropriate value from a UI model based on its type.

        Args:
            model: The UI model to extract value from.

        Returns:
            The extracted value as string, int, float, or empty string if unsupported type.
        """
        if isinstance(model, ui.SimpleStringModel):
            return model.get_value_as_string()
        if isinstance(model, ui.SimpleIntModel):
            return model.get_value_as_int()
        if isinstance(model, ui.SimpleFloatModel):
            return model.get_value_as_float()
        return ""

    def _build_ui(self) -> None:
        """Builds the UI components including the input field and reset button."""
        with self._frame:
            ui.Spacer(width=1)
            with ui.ZStack():
                with ui.VStack(height=0):
                    ui.Spacer(height=2)
                    self._field = self._field_type(
                        name="cell", style_type_name_override="Field", alignment=self._alignment, height=18
                    )
                    ui.Spacer(height=2)
            self._field.model.set_value(self._init_value)
            self._field.model.add_value_changed_fn(lambda m: self._update_value(m))
            # it used to bulk edit, we need the field hook with the value model' value
            self._value_model.add_value_changed_fn(lambda m: self._update_field(m))
            self.subscription = self._field.model.subscribe_end_edit_fn(lambda m: self._end_edit(m))
            with ui.VStack(width=8):
                ui.Spacer()
                self._reset_button = ResetButton(self._init_value, self._on_reset_fn)
                ui.Spacer()

    def _on_reset_fn(self) -> None:
        """Handles the reset button click by restoring the field and value model to the initial value."""
        current_value = self.get_model_value(self._field.model)
        if current_value != self._init_value:
            self._field.model.set_value(self._init_value)
            self._value_model.set_value(self._init_value)

    def _update_value(self, model: object) -> None:
        """Updates the value model when the field value changes and refreshes the reset button visibility.

        Args:
            model: The field model that changed.
        """
        new_value = self.get_model_value(model)
        current_value = self.get_model_value(self._value_model)
        if new_value != current_value:
            self._value_model.set_value(new_value)
            self._reset_button.refresh(new_value)

    def _update_field(self, model: object) -> None:
        """Updates the field value when the value model changes and refreshes the reset button visibility.

        Args:
            model: The value model that changed.
        """
        new_value = self.get_model_value(model)
        current_value = self.get_model_value(self._field.model)
        if new_value != current_value:
            self._field.model.set_value(new_value)
            self._reset_button.refresh(new_value)

    def _end_edit(self, model: object) -> None:
        """Called when field editing ends.

        Args:
            model: The field model that finished editing.
        """

    def _begin_edit(self) -> None:
        """Initiates the editing state for the field.

        Checks if the field is enabled before allowing edit operations to proceed.
        """
        if not self._enable:
            return
