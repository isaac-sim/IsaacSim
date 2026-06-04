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

"""UI widget components for robot setup assembler interface."""

from collections.abc import Callable

import omni.ui as ui
from isaacsim.gui.components.element_wrappers import CheckBox, DropDown, Frame
from isaacsim.gui.components.widgets import DynamicComboBoxModel

LABEL_WIDTH = 160


class CheckBoxWithNoReset(CheckBox):
    """A checkbox UI component that inherits from CheckBox but excludes the reset functionality.

    This class creates a checkbox widget with a label positioned to the left, similar to the standard CheckBox
    component, but without any reset mechanism. The checkbox maintains its state changes without providing
    a way to reset to default values.

    The UI layout consists of a horizontal stack with a fixed-width label on the left and the checkbox
    control on the right. The label width is set to a standard 160 pixels for consistent alignment
    with other form elements.
    """

    def _create_ui_widget(self, label: str, default_value: bool, tooltip: str) -> ui.Frame:
        """Creates the UI widget components for the checkbox.

        Args:
            label: Label text displayed to the left of the checkbox.
            default_value: Initial checked state of the checkbox.
            tooltip: Tooltip text shown when hovering over the checkbox.

        Returns:
            The containing frame widget with the checkbox and label.
        """
        containing_frame = Frame().frame
        with containing_frame:
            with ui.HStack():
                self._label = ui.Label(label, width=LABEL_WIDTH, alignment=ui.Alignment.LEFT_CENTER)
                model = ui.SimpleBoolModel()
                model.set_value(default_value)
                self._checkbox = ui.CheckBox(model=model, tooltip=tooltip)
                model.add_value_changed_fn(self._on_click_fn_wrapper)

        return containing_frame


class DropDownWithPicker(DropDown):
    """A dropdown UI component with an integrated picker button for file or asset selection.

    Extends the standard dropdown functionality by adding a clickable picker icon that triggers
    a selection dialog or custom selection callback. The picker button appears to the left of
    the dropdown menu and provides an intuitive way for users to browse and select items
    rather than typing them manually.

    The component creates a horizontal layout with a label, picker icon, and dropdown menu.
    When the picker button is clicked, it executes the provided selection function to handle
    the selection logic.

    Args:
        *args: Variable length argument list passed to the parent DropDown class.
        **kwargs: Additional keyword arguments passed to the parent DropDown class.
            Special keyword arguments include on_selection_fn for handling picker button clicks.
    """

    def __init__(self, *args: object, **kwargs: object) -> None:
        super().__init__(*args, **kwargs)
        self.selection_fn = kwargs.get("on_selection_fn", None)

    def _create_ui_widget(self, label: str, tooltip: str) -> ui.Frame:
        """Creates the UI widget for the dropdown with picker.

        Creates a horizontal layout containing a label, a picker image button, and a combobox.
        The picker image triggers the selection function when clicked.

        Args:
            label: The text label for the dropdown.
            tooltip: The tooltip text to display for the label.

        Returns:
            The containing frame widget.
        """
        items = []
        combobox_model = DynamicComboBoxModel(items)
        containing_frame = Frame().frame
        with containing_frame:
            with ui.HStack():
                self._label = ui.Label(
                    label, name="dropdown_label", width=LABEL_WIDTH, alignment=ui.Alignment.LEFT_CENTER, tooltip=tooltip
                )
                with ui.HStack():
                    self._picker = ui.Image(
                        name="picker",
                        width=18,
                        height=18,
                        tooltip="Click to show import dialog",
                        mouse_pressed_fn=lambda x, y, b, a: self._on_pressed_fn(b),
                    )
                    ui.Spacer(width=3)
                    self._combobox = ui.ComboBox(combobox_model)

        return containing_frame

    def _on_pressed_fn(self, b: int) -> None:
        """Handles mouse button press events on the picker image.

        Executes the selection function if it exists and the left mouse button was pressed.

        Args:
            b: The mouse button that was pressed (0 for left button).
        """
        if b != 0:
            return
        if self.selection_fn is not None:
            print("Selection fn")
            self.selection_fn(b)


class DropDownWithSelect(DropDown):
    """A dropdown widget with an integrated selection button for interactive prim selection.

    This class extends the DropDown component to include a selection button that allows users to
    interactively select prims in the viewport. The widget combines a standard dropdown menu with a
    selection icon, providing both manual dropdown selection and direct viewport selection capabilities.

    The selection button triggers a callback function that can be used to implement custom selection
    logic, such as opening selection dialogs or enabling viewport picking modes. The widget also supports
    a populate function for dynamically updating the dropdown options based on the current selection.

    Args:
        *args: Variable length argument list passed to the parent DropDown class.
        **kwargs: Additional keyword arguments passed to the parent DropDown class.
    """

    def __init__(self, *args: object, **kwargs: object) -> None:
        super().__init__(*args, **kwargs)
        self._articulation_path_to = None
        self.selection_fn = kwargs.get("on_selection_fn", None)
        self.populate_fn = kwargs.get("populate_fn", None)

    @property
    def articulation_path_to(self) -> str | None:
        """The path to the articulation prim.

        Returns:
            The articulation path currently set for this dropdown.
        """
        return self._articulation_path_to

    @articulation_path_to.setter
    def articulation_path_to(self, path: str | None) -> None:
        self._articulation_path_to = path

    def _create_ui_widget(self, label: str, tooltip: str) -> ui.Frame:
        """Creates the UI widget components for the dropdown with select functionality.

        Args:
            label: The label text to display next to the dropdown.
            tooltip: The tooltip text to show on hover.

        Returns:
            The containing frame widget for the dropdown with select button.
        """
        items = []
        combobox_model = DynamicComboBoxModel(items)
        containing_frame = Frame().frame
        with containing_frame:
            with ui.HStack():
                self._label = ui.Label(
                    label, name="dropdown_label", width=LABEL_WIDTH, alignment=ui.Alignment.LEFT_CENTER, tooltip=tooltip
                )
                with ui.HStack(width=ui.Fraction(1)):
                    self._combobox = ui.ComboBox(combobox_model)
                    ui.Spacer(width=3)
                    self._select = ui.Image(
                        name="select",
                        width=18,
                        height=18,
                        tooltip="Click to select prim in viewport",
                        mouse_pressed_fn=lambda x, y, b, a: self._on_pressed_fn(b),
                    )

        return containing_frame

    def _on_pressed_fn(self, b: int) -> None:
        """Handles mouse press events on the select button.

        Args:
            b: The mouse button that was pressed.
        """
        if b != 0:
            return
        if self.selection_fn is not None:
            print("Selection fn")
            self.selection_fn(b)


def help_frame_header(collapsed: bool, title: str) -> None:
    """Creates a collapsible header UI with a triangle indicator, title label, and help icon.

    Creates a horizontal stack containing a triangle that changes orientation based on collapsed state,
    a title label, and a help documentation icon.

    Args:
        collapsed: Whether the header is in collapsed state, affecting triangle orientation.
        title: Text to display as the header title.
    """
    with ui.HStack(height=22):
        ui.Spacer(width=4)
        with ui.VStack(width=10):
            ui.Spacer()
            if collapsed:
                triangle = ui.Triangle(height=9, width=7)
                triangle.alignment = ui.Alignment.RIGHT_CENTER
            else:
                triangle = ui.Triangle(height=7, width=9)
                triangle.alignment = ui.Alignment.CENTER_BOTTOM
            ui.Spacer()
        ui.Spacer(width=4)
        ui.Label(title, name="collapsable_header", width=0)
        ui.Spacer()
        ui.Image(name="help", width=20, height=20, tooltip="Click to see help documentation")


class ButtonWithIcon:
    """A custom button component that combines an icon and text label.

    This class creates a clickable button with an optional icon and text label using Omni UI components.
    The button consists of a layered structure with an invisible button for interaction, a styled rectangle
    for visual appearance, and a horizontal layout containing the icon and label.

    Args:
        text: The text to display on the button.
        image_width: Width of the icon in pixels. Set to 0 to hide the icon.
        *args: Additional positional arguments passed to ui.ZStack and child components.
        **kwargs: Additional keyword arguments passed to ui.ZStack and child components.
    """

    def __init__(self, text: str = "", image_width: int = 14, *args: object, **kwargs: object) -> None:
        with ui.ZStack(*args, **kwargs):
            self.button = ui.InvisibleButton(*args, **kwargs)
            self.rect = ui.Rectangle(style_type_name_override="Button.Rect", *args, **kwargs)
            with ui.HStack(spacing=8):
                ui.Spacer()
                if image_width > 0:
                    self.image = ui.Image(width=image_width, style_type_name_override=f"Button.Image", *args, **kwargs)
                self.label = ui.Label(text, width=0, style_type_name_override=f"Button.Label", *args, **kwargs)
                ui.Spacer()

    @property
    def enabled(self) -> bool:
        """Whether the button is enabled and can be clicked."""
        return self.button.enabled

    @enabled.setter
    def enabled(self, value: bool) -> None:
        self.button.enabled = value
        self.rect.enabled = value
        self.image.enabled = value
        self.label.enabled = value

    def set_clicked_fn(self, fn: Callable[[], None]) -> None:
        """Sets the callback function to be called when the button is clicked.

        Args:
            fn: Callback function to execute when the button is clicked.
        """
        self.button.set_clicked_fn(fn)
