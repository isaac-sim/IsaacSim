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

"""Provides UI utility functions and classes for building wizard interfaces in the Isaac Sim robot setup workflow."""

import os
from collections.abc import Callable
from typing import Any

import isaacsim.robot_setup.wizard
import omni.ui as ui
from omni.kit.widget.filebrowser import FileBrowserItem
from omni.kit.window.filepicker import FilePickerDialog

from ..progress import ProgressColorState, ProgressRegistry
from .utils import Singleton


def custom_header(collapsed: bool, title: str) -> None:
    """Creates a custom collapsible header with a triangle indicator and title.

    Args:
        collapsed: Whether the header is in collapsed state.
        title: Text to display as the header title.
    """
    with ui.HStack(height=30):
        ui.Spacer(width=8)
        with ui.VStack(width=13):
            ui.Spacer()
            triangle = ui.Triangle(height=12)
            if collapsed:
                triangle.alignment = ui.Alignment.RIGHT_CENTER
            else:
                triangle.alignment = ui.Alignment.CENTER_BOTTOM
            ui.Spacer()
        ui.Spacer(width=10)
        ui.Label(title, name="separator", width=0)
        ui.Spacer()
        # with ui.ZStack(content_clipping=True, width=38):
        #     ui.Image(name="help", width=28, height=28, mouse_pressed_fn=lambda x, y, b, a: print("Help button clicked"))


def info_header(collapsed: bool, title: str) -> None:
    """Creates an information header with a triangle indicator, title, and info icon.

    Args:
        collapsed: Whether the header is in collapsed state.
        title: Text to display as the header title.
    """
    with ui.HStack(height=20):
        with ui.VStack(width=10):
            ui.Spacer()
            triangle = ui.Triangle(height=8)
            if collapsed:
                triangle.alignment = ui.Alignment.RIGHT_CENTER
            else:
                triangle.alignment = ui.Alignment.CENTER_BOTTOM
            ui.Spacer()
        ui.Spacer(width=10)
        ui.Label(title, name="collapsable_header", width=0)
        ui.Spacer()
        with ui.ZStack(content_clipping=True, width=20):
            ui.Image(name="info", width=20, height=20)


def separator(text: str) -> None:
    """Creates a visual separator with text label and horizontal line.

    Args:
        text: Text to display before the separator line.
    """
    with ui.HStack(height=16, spacing=10):
        ui.Label(text, name="separator", width=0)
        ui.Line()


def info_frame(infos: list, collapse_fn: callable) -> None:
    """Creates a collapsible information frame displaying a list of info items with bullet points.

    Args:
        infos: List of information strings to display.
        collapse_fn: Callback function executed when the frame is collapsed or expanded.
    """
    with ui.CollapsableFrame(
        "Info", name="info", height=0, collapsed=True, build_header_fn=info_header, collapsed_changed_fn=collapse_fn
    ):
        with ui.HStack():
            ui.Spacer(width=40)
            with ui.VStack(spacing=5):
                for info in infos:
                    with ui.HStack(height=15, spacing=10):
                        ui.Circle(name="dot", width=3)
                        ui.Label(info, height=0, width=0)


class ComboListItem(ui.AbstractItem):
    """A wrapper class for string items used in combo box UI components.

    This class extends ui.AbstractItem to provide a standardized way to represent string items
    within combo box models. It encapsulates a string value and creates the necessary UI model
    for display and selection in dropdown interfaces.

    Args:
        item: The string value to be wrapped as a combo box item.
    """

    def __init__(self, item: str) -> None:
        super().__init__()
        self.model = ui.SimpleStringModel(item)
        self.item = item


class ComboListModel(ui.AbstractItemModel):
    """A model for managing selectable items in a combo box widget.

    This class extends Omni UI's AbstractItemModel to provide a data model for combo box controls.
    It manages a list of items where one can be selected at a time, tracks the current selection,
    and provides methods to get and set the selected item by index or string value.

    Args:
        item_list: List of string items to populate the combo box.
        default_index: Index of the item to select by default.
    """

    def __init__(self, item_list: list, default_index: int) -> None:
        super().__init__()
        self._default_index = default_index
        self._current_index = ui.SimpleIntModel(default_index)
        self._current_index.add_value_changed_fn(self.selection_changed)
        self._item_list = item_list
        self._items = []
        if item_list:
            for item in item_list:
                self._items.append(ComboListItem(item))

    def get_item_children(self, item: object) -> None:
        """Returns all child items of the given item.

        Args:
            item: The parent item to get children for.

        Returns:
            List of all child items in the model.
        """
        return self._items

    def get_item_value_model(self, item: object, column_id: int) -> None:
        """Returns the value model for a specific item and column.

        Args:
            item: The item to get the value model for.
            column_id: The column identifier.

        Returns:
            The value model for the specified item and column, or the current index model if item is None.
        """
        if item is None:
            return self._current_index
        return item.model

    def get_current_index(self) -> int:
        """Current index of the selected item.

        Returns:
            The current selected index as an integer.
        """
        return self._current_index.get_value_as_int()

    def set_current_index(self, index: int) -> None:
        """Sets the current selected index.

        Args:
            index: The index to set as the current selection.
        """
        self._current_index.set_value(index)

    def get_current_string(self) -> str:
        """Current string value of the selected item.

        Returns:
            The string representation of the currently selected item.
        """
        return self._items[self._current_index.get_value_as_int()].model.get_value_as_string()

    def set_current_string(self, string: str) -> None:
        """Sets the current selection by matching the provided string.

        Args:
            string: The string value to match and select.
        """
        for index, item in enumerate(self._items):
            if item.model.get_value_as_string() == string:
                self.set_current_index(index)
                break

    def get_current_item(self) -> None:
        """Current selected item.

        Returns:
            The currently selected item object.
        """
        return self._items[self._current_index.get_value_as_int()].item

    def is_default(self) -> bool:
        """Checks if the current selection is the default selection.

        Returns:
            True if the current index matches the default index, False otherwise.
        """
        return self.get_current_index() == self._default_index

    def add_item(self, item: str) -> None:
        """Adds a new item to the combo list.

        Args:
            item: The item to add to the list.
        """
        self._items.append(ComboListItem(item))
        self._item_changed(None)

    def selection_changed(self, index: object) -> None:
        """Handles selection change events.

        Args:
            index: The new selected index.
        """
        #     """
        #     reset progress, parse progress on each page, and turn the ones that are updated to inprogress.

        #     """
        self._item_changed(None)
        # print("selection changed to ", self.get_current_item().name)
        # call the __parse_robot_param function in every single page, and update progress accordingly

    def has_item(self) -> bool:
        """Whether the model contains any items.

        Returns:
            True if the model has one or more items, False otherwise.
        """
        return len(self._items) > 0


def create_combo_list_model(items_list: list, index: int) -> None:
    """Creates a combo box list model for UI selection.

    Args:
        items_list: List of items to populate the combo box.
        index: Default index of the selected item.

    Returns:
        A ComboListModel instance for use in UI combo box widgets.
    """
    return ComboListModel(items_list, index)


def next_step(curret_step_name: str, next_step_name: str, verify_fn: callable = None) -> None:
    """Advances to the next step in the wizard workflow.

    Marks the current step as complete and navigates to the specified next step.

    Args:
        curret_step_name: Name of the current step to mark as complete.
        next_step_name: Name of the next step to navigate to.
        verify_fn: Function to verify the current step before moving to the next step.
    """
    if verify_fn and callable(verify_fn):
        verify_fn()

    ProgressRegistry().set_step_progress(curret_step_name, ProgressColorState.COMPLETE)
    isaacsim.robot_setup.wizard.get_window().update_page(next_step_name)


def text_with_dot(text: str) -> None:
    """Creates a text label with a bullet point indicator.

    Args:
        text: Text to display next to the bullet point.
    """
    with ui.HStack(height=15, spacing=10):
        ui.Circle(name="dot", width=4)
        ui.Label(text, height=0)


class ButtonWithIcon:
    """A custom UI button widget that displays an icon alongside text.

    This widget creates a clickable button with an optional icon image positioned to the left of the text label.
    The button is built using a ZStack layout with an invisible button for interaction and a styled rectangle
    for visual appearance.

    Args:
        text: The text label to display on the button.
        image_width: The width of the icon image in pixels. Set to 0 to hide the icon.
        *args: Additional positional arguments passed to the underlying UI components.
        **kwargs: Additional keyword arguments passed to the underlying UI components.
    """

    def __init__(self, text: str = "", image_width: int = 14, *args: Any, **kwargs: Any) -> None:
        with ui.ZStack(*args, **kwargs):
            self.button = ui.InvisibleButton(*args, **kwargs)
            self.rect = ui.Rectangle(style_type_name_override="Button.Rect", *args, **kwargs)
            with ui.HStack(spacing=8):
                ui.Spacer()
                if image_width > 0:
                    self.image = ui.Image(width=image_width, style_type_name_override="Button.Image", *args, **kwargs)
                self.label = ui.Label(text, width=0, style_type_name_override="Button.Label", *args, **kwargs)
                ui.Spacer()

    @property
    def enabled(self) -> bool:
        """Whether the button is enabled.

        Returns:
            True if the button is enabled, False otherwise.
        """
        return self.button.enabled

    @enabled.setter
    def enabled(self, value: Any) -> None:
        self.button.enabled = value
        self.rect.enabled = value
        self.image.enabled = value
        self.label.enabled = value

    def set_clicked_fn(self, fn: Callable) -> None:
        """Sets the callback function to be executed when the button is clicked.

        Args:
            fn: The callback function to execute on button click.
        """
        self.button.set_clicked_fn(fn)


class FileSorter:
    """A utility class for classifying robot-related files by type and extension.

    This class provides static methods to analyze file paths and categorize them as simulation-ready files
    (URDF, XML) or 3D model files (USD, OBJ, STL, DAE, FBX). It supports file validation and type
    classification for robot setup workflows.
    """

    # Define the valid extensions for each type
    SIM_READY_EXTENSIONS = {".urdf", ".xml"}
    """File extensions for simulation-ready files."""
    MODEL_EXTENSIONS = {".usd", ".obj", ".stl", ".dae", ".fbx", ".usda"}
    """File extensions for 3D model files."""

    # Define named parameters for the return values
    SIM_READY = 1
    """Classification constant for simulation-ready files."""
    MODEL = 2
    """Classification constant for 3D model files."""
    INVALID = 0
    """Classification constant for invalid files."""

    @staticmethod
    def get_file_extension(filepath: str) -> str:
        """Extract the file extension from the filepath.

        Args:
            filepath: Path to the file.

        Returns:
            The file extension in lowercase.
        """
        _, extension = os.path.splitext(filepath)
        return extension.lower()  # Return in lowercase to avoid case-sensitivity issues

    @staticmethod
    def classify_file(filepath: str) -> int:
        """Classify the file and return a numeric value with named parameters.

        Args:
            filepath: Path to the file to classify.

        Returns:
            Classification value: SIM_READY (1) for sim-ready files, MODEL (2) for 3D model files, or INVALID (0) for unsupported files.
        """
        extension = FileSorter.get_file_extension(filepath)

        if extension in FileSorter.SIM_READY_EXTENSIONS:
            return FileSorter.SIM_READY
        elif extension in FileSorter.MODEL_EXTENSIONS:
            return FileSorter.MODEL
        else:
            return FileSorter.INVALID

    @staticmethod
    def is_sim_ready(filepath: str) -> bool:
        """Return True if the file is 'sim-ready', otherwise False.

        Args:
            filepath: Path to the file to check.

        Returns:
            True if the file is sim-ready, False otherwise.
        """
        return FileSorter.classify_file(filepath) == FileSorter.SIM_READY

    @staticmethod
    def is_3d_model(filepath: str) -> bool:
        """Return True if the file is a '3D model', otherwise False.

        Args:
            filepath: Path to the file to check.

        Returns:
            True if the file is a 3D model, False otherwise.
        """
        return FileSorter.classify_file(filepath) == FileSorter.MODEL

    @staticmethod
    def is_valid(filepath: str) -> bool:
        """Return True if the file is either 'sim-ready' or a '3D model', otherwise False.

        Args:
            filepath: Path to the file to validate.

        Returns:
            True if the file is either sim-ready or a 3D model, False otherwise.
        """
        classification = FileSorter.classify_file(filepath)
        return classification == FileSorter.SIM_READY or classification == FileSorter.MODEL


def open_extension(ext_name: str, action_id: str = None) -> None:
    """Opens the extension with the given name.

    Enables the extension if it's not already enabled and executes its action.

    Args:
        ext_name: Name of the extension to open.
        action_id: Specific action ID to execute. If None, uses default action ID based on extension name.
    """
    import omni.kit.app

    ext_manager = omni.kit.app.get_app().get_extension_manager()
    extension_enabled = ext_manager.is_extension_enabled(ext_name)
    if not extension_enabled:
        result = ext_manager.set_extension_enabled_immediate(ext_name, True)

    action_registry = omni.kit.actions.core.get_action_registry()
    ext_id = ext_manager.get_extension_id_by_module(ext_name)
    if action_id is None:
        action_id = f'{ext_id.replace(" ", "_")}{ext_name.replace(" ", "_")}'
    action_registry.execute_action(ext_id, action_id)


@Singleton
class FilteredFileDialog:
    """A singleton file dialog that filters files based on specified formats.

    This dialog provides a filtered file picker interface that only displays files with extensions
    matching the specified formats. It uses the Omniverse Kit file picker system to create a
    consistent file selection experience with custom filtering capabilities.

    Args:
        formats: List of file extensions to filter by (e.g., [".urdf", ".xml"]).
        handler: Callback function to handle file selection events.
    """

    def __init__(self, formats: list[str], handler: Callable) -> None:
        self.formats = formats
        self.handler = handler
        self._filepicker = None
        self._initialize_filepicker()

    def _on_filter_files(self, item: FileBrowserItem) -> bool:
        """Callback to filter the choices of file names in the open or save dialog.

        Args:
            item: File browser item to filter.

        Returns:
            True if the item should be shown in the dialog.
        """
        if not item or item.is_folder:
            return True
        # Show only files with listed extensions
        return os.path.splitext(item.path)[1].lower() in self.formats

    def _initialize_filepicker(self) -> None:
        """Initialize the file picker dialog with current formats and handler."""
        self._filepicker = FilePickerDialog(
            "Select File",
            allow_multi_selection=False,
            apply_button_label="Open",
            click_apply_handler=self.handler,
            item_filter_options=[f"*{ext.lower()}" for ext in self.formats],
            item_filter_fn=self._on_filter_files,
        )
        self._filepicker.hide()

    def __call__(self, formats: list[str], handler: Callable) -> None:
        """Update the dialog with new file formats and handler.

        Args:
            formats: List of file extensions to filter.
            handler: Callback function to handle file selection.
        """
        self.formats = formats
        self.handler = handler
        self._initialize_filepicker()

    def open_dialog(self) -> None:
        """Show the file picker dialog."""
        self._filepicker.show()

    def close_dialog(self) -> None:
        """Hide the file picker dialog."""
        self._filepicker.hide()


def open_folder_picker(on_click_fn: callable) -> None:
    """Opens a folder picker dialog for selecting an output directory.

    Args:
        on_click_fn: Callback function executed when a folder is selected, receives filename and path arguments.
    """

    def on_selected(filename: Any, path: Any) -> None:
        on_click_fn(filename, path)
        file_picker.hide()

    def on_canceled(a: Any, b: Any) -> None:
        file_picker.hide()

    file_picker = FilePickerDialog(
        "Select Output Folder",
        allow_multi_selection=False,
        apply_button_label="Select Folder",
        click_apply_handler=lambda a, b: on_selected(a, b),
        click_cancel_handler=lambda a, b: on_canceled(a, b),
    )
