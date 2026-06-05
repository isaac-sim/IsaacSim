# SPDX-FileCopyrightText: Copyright (c) 2022-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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

"""UI builder for the UI component library template."""

import os

import omni.ui as ui
from isaacsim.gui.components.element_wrappers import (
    Button,
    CheckBox,
    CollapsableFrame,
    ColorPicker,
    DropDown,
    FloatField,
    IntField,
    StateButton,
    StringField,
    TextBlock,
    XYPlot,
)
from isaacsim.gui.components.ui_utils import get_style


class UIBuilder:
    """Build and manage the UI for the UI component library."""

    def __init__(self) -> None:
        # Frames are sub-windows that can contain multiple UI elements
        self.frames = []

        # UI elements created using a UIElementWrapper from isaacsim.gui.components.element_wrappers
        self.wrapped_ui_elements = []

    ###################################################################################
    #           The Functions Below Are Called Automatically By extension.py
    ###################################################################################

    def on_menu_callback(self) -> None:
        """Callback for when the UI is opened from the toolbar.

        This is called directly after build_ui().
        """

    def on_timeline_event(self, event: object) -> None:
        """Callback for Timeline events (Play, Pause, Stop).

        Args:
            event: Event type.
        """

    def on_physics_step(self, step: object) -> None:
        """Callback for Physics Step.

        Physics steps only occur when the timeline is playing.

        Args:
            step: Size of physics step.
        """

    def on_stage_event(self, event: object) -> None:
        """Callback for Stage Events.

        Args:
            event: Event type.
        """

    def cleanup(self) -> None:
        """Called when the stage is closed or the extension is hot reloaded.

        Perform any necessary cleanup such as removing active callback functions
        Buttons imported from isaacsim.gui.components.element_wrappers implement a cleanup function that should be called.
        """
        # None of the UI elements in this template actually have any internal state that needs to be cleaned up.
        # But it is best practice to call cleanup() on all wrapped UI elements to simplify development.
        for ui_elem in self.wrapped_ui_elements:
            ui_elem.cleanup()

    def build_ui(self) -> None:
        """Build a custom UI tool to run your extension.

        This function will be called any time the UI window is closed and reopened.
        """
        # Create a UI frame that prints the latest UI event.
        self._create_status_report_frame()

        # Create a UI frame demonstrating simple UI elements for user input
        self._create_simple_editable_fields_frame()

        # Create a UI frame with different button types
        self._create_buttons_frame()

        # Create a UI frame with different selection widgets
        self._create_selection_widgets_frame()

        # Create a UI frame with different plotting tools
        self._create_plotting_frame()

    def _create_status_report_frame(self) -> None:
        self._status_report_frame = CollapsableFrame("Status Report", collapsed=False)
        with self._status_report_frame:
            with ui.VStack(style=get_style(), spacing=5, height=0):
                self._status_report_field = TextBlock(
                    "Last UI Event",
                    num_lines=3,
                    tooltip="Prints the latest change to this UI",
                    include_copy_button=True,
                )

    def _create_simple_editable_fields_frame(self) -> None:
        self._simple_fields_frame = CollapsableFrame("Simple Editable Fields", collapsed=False)

        with self._simple_fields_frame:
            with ui.VStack(style=get_style(), spacing=5, height=0):
                int_field = IntField(
                    "Int Field",
                    default_value=1,
                    tooltip="Type an int or click and drag to set a new value.",
                    lower_limit=-100,
                    upper_limit=100,
                    on_value_changed_fn=self._on_int_field_value_changed_fn,
                )
                self.wrapped_ui_elements.append(int_field)

                float_field = FloatField(
                    "Float Field",
                    default_value=1.0,
                    tooltip="Type a float or click and drag to set a new value.",
                    step=0.5,
                    format="%.2f",
                    lower_limit=-100.0,
                    upper_limit=100.0,
                    on_value_changed_fn=self._on_float_field_value_changed_fn,
                )
                self.wrapped_ui_elements.append(float_field)

                def is_usd_or_python_path(file_path: str) -> bool:
                    # Filter file paths shown in the file picker to only be USD or Python files
                    _, ext = os.path.splitext(file_path.lower())
                    return ext == ".usd" or ext == ".py"

                string_field = StringField(
                    "String Field",
                    default_value="Type Here or Use File Picker on the Right",
                    tooltip="Type a string or use the file picker to set a value",
                    read_only=False,
                    multiline_okay=False,
                    on_value_changed_fn=self._on_string_field_value_changed_fn,
                    use_folder_picker=True,
                    item_filter_fn=is_usd_or_python_path,
                )
                self.wrapped_ui_elements.append(string_field)

    def _create_buttons_frame(self) -> None:
        buttons_frame = CollapsableFrame("Buttons Frame", collapsed=False)

        with buttons_frame:
            with ui.VStack(style=get_style(), spacing=5, height=0):
                button = Button(
                    "Button",
                    "CLICK ME",
                    tooltip="Click This Button to activate a callback function",
                    on_click_fn=self._on_button_clicked_fn,
                )
                self.wrapped_ui_elements.append(button)

                state_button = StateButton(
                    "State Button",
                    "State A",
                    "State B",
                    tooltip="Click this button to transition between two states",
                    on_a_click_fn=self._on_state_btn_a_click_fn,
                    on_b_click_fn=self._on_state_btn_b_click_fn,
                    physics_callback_fn=None,  # See Loaded Scenario Template for example usage
                )
                self.wrapped_ui_elements.append(state_button)

                check_box = CheckBox(
                    "Check Box",
                    default_value=False,
                    tooltip=" Click this checkbox to activate a callback function",
                    on_click_fn=self._on_checkbox_click_fn,
                )
                self.wrapped_ui_elements.append(check_box)

    def _create_selection_widgets_frame(self) -> None:
        self._selection_widgets_frame = CollapsableFrame("Selection Widgets", collapsed=False)

        with self._selection_widgets_frame:
            with ui.VStack(style=get_style(), spacing=5, height=0):

                def dropdown_populate_fn() -> list:
                    return ["Option A", "Option B", "Option C"]

                dropdown = DropDown(
                    "Drop Down",
                    tooltip=" Select an option from the DropDown",
                    populate_fn=dropdown_populate_fn,
                    on_selection_fn=self._on_dropdown_item_selection,
                )
                self.wrapped_ui_elements.append(dropdown)

                dropdown.repopulate()  # This does not happen automatically, and it triggers the on_selection_fn

                color_picker = ColorPicker(
                    "Color Picker",
                    default_value=[0.69, 0.61, 0.39, 1.0],
                    tooltip="Select a Color",
                    on_color_picked_fn=self._on_color_picked,
                )
                self.wrapped_ui_elements.append(color_picker)

    def _create_plotting_frame(self) -> None:
        self._plotting_frame = CollapsableFrame("Plotting Tools", collapsed=False)

        with self._plotting_frame:
            with ui.VStack(style=get_style(), spacing=5, height=0):
                import numpy as np

                x = np.arange(-1, 6.01, 0.01)
                y = np.sin((x - 0.5) * np.pi)
                plot = XYPlot(
                    "XY Plot",
                    tooltip="Press mouse over the plot for data label",
                    x_data=[x[:300], x[100:400], x[200:]],
                    y_data=[y[:300], y[100:400], y[200:]],
                    x_min=None,  # Use default behavior to fit plotted data to entire frame
                    x_max=None,
                    y_min=-1.5,
                    y_max=1.5,
                    x_label="X [rad]",
                    y_label="Y",
                    plot_height=10,
                    legends=["Line 1", "Line 2", "Line 3"],
                    show_legend=True,
                    plot_colors=[
                        [255, 0, 0],
                        [0, 255, 0],
                        [0, 100, 200],
                    ],  # List of [r,g,b] values; not necessary to specify
                )

    ######################################################################################
    # Functions Below This Point Are Callback Functions Attached to UI Element Wrappers
    ######################################################################################

    def _on_int_field_value_changed_fn(self, new_value: int) -> None:
        status = f"Value was changed in int field to {new_value}"
        self._status_report_field.set_text(status)

    def _on_float_field_value_changed_fn(self, new_value: float) -> None:
        status = f"Value was changed in float field to {new_value}"
        self._status_report_field.set_text(status)

    def _on_string_field_value_changed_fn(self, new_value: str) -> None:
        status = f"Value was changed in string field to {new_value}"
        self._status_report_field.set_text(status)

    def _on_button_clicked_fn(self) -> None:
        status = "The Button was Clicked!"
        self._status_report_field.set_text(status)

    def _on_state_btn_a_click_fn(self) -> None:
        status = "State Button was Clicked in State A!"
        self._status_report_field.set_text(status)

    def _on_state_btn_b_click_fn(self) -> None:
        status = "State Button was Clicked in State B!"
        self._status_report_field.set_text(status)

    def _on_checkbox_click_fn(self, value: bool) -> None:
        status = f"CheckBox was set to {value}!"
        self._status_report_field.set_text(status)

    def _on_dropdown_item_selection(self, item: str) -> None:
        status = f"{item} was selected from DropDown"
        self._status_report_field.set_text(status)

    def _on_color_picked(self, color: list[float]) -> None:
        formatted_color = [float(f"{i:0.2f}") for i in color]
        status = f"RGBA Color {formatted_color} was picked in the ColorPicker"
        self._status_report_field.set_text(status)
