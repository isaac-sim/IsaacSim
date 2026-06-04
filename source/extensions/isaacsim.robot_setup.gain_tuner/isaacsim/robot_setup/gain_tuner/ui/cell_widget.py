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

"""UI components for cell widgets used in the gain tuner interface."""

import omni.ui as ui


class CellLabelField:
    """A UI component that creates an editable field synchronized with a data model.

    This class creates a formatted input field that displays and modifies values from a data model. The field
    automatically synchronizes bidirectionally with the underlying model, updating the display when the model
    changes and updating the model when the user modifies the field. It supports different field types and
    formatting options for displaying various data types.

    Args:
        value_model: The data model containing the value to display and edit. Can be a SimpleStringModel,
            SimpleIntModel, or SimpleFloatModel.
        field_type: The UI field type to create for user input.
        format: Format string used to display the model value.
        alignment: Text alignment within the field.
    """

    def __init__(
        self, value_model: object, field_type: object, format: str, alignment: object = ui.Alignment.LEFT_CENTER
    ) -> None:
        self._value_model = value_model
        self._init_value = format % (self.get_model_value(value_model))
        self._field_type = field_type
        self._alignment = alignment
        self._enable = True
        self._frame = ui.HStack(height=26, spacing=2)
        self._format = format
        self._build_ui()

    @property
    def enabled(self) -> bool:
        """Whether the cell label field is enabled for user interaction.

        Returns:
            True if the field is enabled, False otherwise.
        """
        return self._enable

    def update_default_value(self) -> None:
        """Updates the default value of the field to the current model value."""
        self._init_value = self.get_model_value(self._value_model)

    @property
    def visible(self) -> bool:
        """Whether the cell label field is visible in the UI.

        Returns:
            True if the field is visible, False otherwise.
        """
        return self._frame.visible

    @visible.setter
    def visible(self, value: bool) -> None:
        self._frame.visible = value

    @enabled.setter
    def enabled(self, enable: bool) -> None:
        self._enable = enable
        self._field.enabled = enable

    @property
    def field(self) -> object:
        """The underlying UI field widget for the cell label.

        Returns:
            The UI field widget instance.
        """
        return self._field

    def get_model_value(self, model: object) -> str | int | float:
        """Extracts the value from a UI model based on its type.

        Args:
            model: The UI model to extract the value from.

        Returns:
            The extracted value as a string, integer, or float depending on the model type, or empty string if unsupported.
        """
        if isinstance(model, ui.SimpleStringModel):
            return model.get_value_as_string()
        if isinstance(model, ui.SimpleIntModel):
            return model.get_value_as_int()
        if isinstance(model, ui.SimpleFloatModel):
            return model.get_value_as_float()
        return ""

    def _build_ui(self) -> None:
        """Builds the UI components for the cell label field including the frame, field widget, and event handlers."""
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

    def _update_value(self, model: object) -> None:
        """Updates the value model when the field value changes.

        Args:
            model: The field model that triggered the value change.
        """
        new_value = self.get_model_value(model)
        current_value = self.get_model_value(self._value_model)
        if new_value != current_value:
            self._value_model.set_value(new_value)

    def _update_field(self, model: object) -> None:
        """Updates the field display when the value model changes.

        Args:
            model: The value model that triggered the change.
        """
        new_value = self.get_model_value(model)
        current_value = self.get_model_value(self._field.model)
        if new_value != current_value:
            self._field.model.set_value(new_value)

    def _end_edit(self, model: object) -> None:
        """Handles the end of an edit operation on the field.

        Args:
            model: The model associated with the edit operation.
        """

    def _begin_edit(self) -> None:
        """Initiates an edit operation on the field if the field is enabled."""
        if not self._enable:
            return


class CellColor:
    """A UI component that displays colored rectangles in table cells with selection state visualization.

    This class creates a visual cell component that shows different color layouts based on its selection state.
    When unselected, it displays a single colored rectangle spanning the full width. When selected, it displays
    three narrow colored rectangles side by side, each showing a different color from the provided colors list.

    The component automatically rebuilds its visual representation when the selection state changes, providing
    dynamic visual feedback in table or grid interfaces.

    Args:
        colors: A list or sequence of color values used to render the rectangles. The first color is always
            used. When selected, up to three colors are displayed as separate rectangles.
    """

    def __init__(self, colors: list) -> None:
        self._colors = colors
        self._selected = ui.SimpleBoolModel(False)
        self._selected.add_value_changed_fn(lambda m: self._update_value(m))
        self._frame = ui.HStack(height=26, width=26, spacing=0)
        self._build_ui()

    @property
    def selected(self) -> bool:
        """Selection state of the color cell.

        Returns:
            True if the color cell is selected, False otherwise.
        """
        return self._selected.get_value_as_bool()

    @selected.setter
    def selected(self, value: bool) -> None:
        self._selected.set_value(value)

    @property
    def visible(self) -> bool:
        """Visibility state of the color cell frame.

        Returns:
            True if the color cell frame is visible, False otherwise.
        """
        return self._frame.visible

    @visible.setter
    def visible(self, value: bool) -> None:
        self._frame.visible = value

    def _build_ui(self) -> None:
        """Builds the UI elements for the color cell.

        Creates rectangles displaying the colors based on the selection state. When selected, displays three
        separate color rectangles. When not selected, displays a single combined color rectangle.
        """
        with self._frame:
            ui.Spacer(width=1)
            if self.selected:
                ui.Rectangle(
                    name="selected_color",
                    height=26,
                    width=8,
                    style={"background_color": self._colors[0], "border_radius": 0},
                )
                ui.Rectangle(
                    name="selected_color",
                    height=26,
                    width=8,
                    style={"background_color": self._colors[1], "border_radius": 0},
                )
                ui.Rectangle(
                    name="selected_color",
                    height=26,
                    width=8,
                    style={"background_color": self._colors[2], "border_radius": 0},
                )
            else:
                ui.Rectangle(
                    name="selected_color",
                    height=26,
                    width=24,
                    style={"background_color": self._colors[0], "border_radius": 0},
                )

    def _update_value(self, model: object) -> None:
        """Updates the color cell display when the selection state changes.

        Clears the current frame and rebuilds the UI to reflect the new selection state.

        Args:
            model: The boolean model containing the new selection state.
        """
        self._frame.clear()
        self._build_ui()
