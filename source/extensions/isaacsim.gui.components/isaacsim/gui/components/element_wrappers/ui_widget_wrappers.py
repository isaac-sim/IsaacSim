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

"""UI widget wrapper classes for creating Isaac Sim GUI components with standardized interfaces."""

import sys
from cmath import inf
from collections.abc import Callable, Iterable

import carb
import numpy as np
import omni
import omni.ui as ui
from isaacsim.gui.components.ui_utils import (
    BUTTON_WIDTH,
    add_line_rect_flourish,
    add_separator,
    format_tt,
    get_style,
    on_copy_to_clipboard,
)
from isaacsim.gui.components.widgets import DynamicComboBoxModel
from omni.kit.window.filepicker import FilePickerDialog
from omni.kit.window.property.templates import LABEL_HEIGHT, LABEL_WIDTH
from omni.usd import get_context
from pxr import Usd, UsdGeom, UsdPhysics

from .base_ui_element_wrappers import UIWidgetWrapper


def get_prim_object_type(prim_path: str) -> str | None:
    """Get the dynamic control object type of the USD Prim at the given path.

    Determines the type of physics or dynamic control object based on the USD APIs and schema types
    applied to the prim. Returns specific type strings that can be used for categorizing prims
    in physics simulations or UI controls.

    Args:
        prim_path: Path to the USD prim to analyze.

    Returns:
        Type string for the prim: "articulation" for ArticulationRootAPI prims, "rigid_body" for
        RigidBodyAPI prims, "joint" for PrismaticJoint/RevoluteJoint/SphericalJoint prims,
        "d6joint" for general Joint prims, "xform" for Xformable prims, or None if no matching type.
    """
    prim = omni.usd.get_context().get_stage().GetPrimAtPath(prim_path)
    if prim.HasAPI(UsdPhysics.ArticulationRootAPI):
        return "articulation"
    elif prim.HasAPI(UsdPhysics.RigidBodyAPI):
        return "rigid_body"
    elif (
        prim.IsA(UsdPhysics.PrismaticJoint) or prim.IsA(UsdPhysics.RevoluteJoint) or prim.IsA(UsdPhysics.SphericalJoint)
    ):
        return "joint"
    elif prim.IsA(UsdPhysics.Joint):
        return "d6joint"
    elif prim.IsA(UsdGeom.Xformable):
        return "xform"
    else:
        return None


##########################################################################################
#                                 UI Frame Wrappers
##########################################################################################


class ScrollingWindow(ui.Window):
    """A window with built-in vertical scrolling capabilities.

    This class extends omni.ui.Window to automatically include a ScrollingFrame with always-on vertical scrollbar,
    making it suitable for content that may exceed the window's visible area. The scrolling frame becomes the primary
    container for all window content.

    Args:
        **kwargs: Additional keyword arguments passed to the parent ui.Window class.
    """

    def __init__(self, **kwargs: object) -> None:
        super().__init__(**kwargs)
        original_frame = ui.Window.frame.fget(self)
        with original_frame:
            self._scrolling_frame = ui.ScrollingFrame(vertical_scrollbar_policy=ui.ScrollBarPolicy.SCROLLBAR_ALWAYS_ON)

    @property
    def frame(self) -> ui.ScrollingFrame:
        """The scrolling frame contained within this window.

        Returns:
            The UI ScrollingFrame element that provides scrollable content.
        """
        return self._scrolling_frame


class Frame(UIWidgetWrapper):
    """Create a Frame UI element.

    Args:
        enabled: Frame is enabled.
        visible: Frame is visible.
        build_fn: A function that can be called to specify what should fill the Frame.
            Function should take no arguments. Return values will not be used.
    """

    def __init__(self, enabled: bool = True, visible: bool = True, build_fn: Callable = None) -> None:
        # Create a Frame UI element
        self._frame = self._create_frame(enabled, visible, build_fn)
        super().__init__(self.frame)

    @property
    def frame(self) -> ui.Frame:
        """A UI Frame.

        Returns:
            A UI Frame.
        """
        return self._frame

    def rebuild(self) -> None:
        """Rebuild the Frame using the specified build_fn."""
        self.frame.rebuild()

    def set_build_fn(self, build_fn: Callable) -> None:
        """Set the build_fn to use when rebuilding the frame.

        Args:
            build_fn: Build function to use when rebuilding the frame.
                Function should take no arguments. Return values will not be used.
        """
        self.frame.set_build_fn(build_fn)

    def __enter__(self) -> None:
        """Enter the context manager for this Frame."""
        self.frame.__enter__()

    def __exit__(self, *args: object) -> None:
        """Exit the context manager for this Frame.

        Args:
            *args: Exception arguments passed from context manager.
        """
        self.frame.__exit__(*args)

    def _create_frame(self, enabled: bool, visible: bool, build_fn: Callable) -> ui.CollapsableFrame:
        """Create the underlying UI Frame with specified properties.

        Args:
            enabled: Whether the Frame is enabled.
            visible: Whether the Frame is visible.
            build_fn: Build function to use when building the Frame.
                Function should take no arguments. Return values will not be used.

        Returns:
            The created UI CollapsableFrame.
        """
        frame = ui.Frame(
            visible=visible,
            enabled=enabled,
            build_fn=build_fn,
            style=get_style(),
            style_type_name_override="Frame",
            horizontal_scrollbar_policy=ui.ScrollBarPolicy.SCROLLBAR_AS_NEEDED,
            vertical_scrollbar_policy=ui.ScrollBarPolicy.SCROLLBAR_ALWAYS_ON,
        )

        return frame


class CollapsableFrame(Frame):
    """Create a CollapsableFrame UI element.

    Args:
        title: Title of Collapsable Frame
        collapsed: Frame is collapsed.
        enabled: Frame is enabled.
        visible: Frame is visible.
        build_fn: A function that can be called to specify what should fill the Frame.
            Function should take no arguments.  Return values will not be used.
    """

    def __init__(
        self, title: str, collapsed: bool = True, enabled: bool = True, visible: bool = True, build_fn: Callable = None
    ) -> None:
        # Create a Frame UI element
        self._frame = self._create_frame(title, collapsed, enabled, visible, build_fn)
        UIWidgetWrapper.__init__(self, self.frame)

    @property
    def collapsed(self) -> bool:
        """Whether CollapsableFrame is collapsed.

        Returns:
            True if CollapsableFrame is collapsed.
        """
        return self.frame.collapsed

    @collapsed.setter
    def collapsed(self, value: bool) -> None:
        self.frame.collapsed = value

    @property
    def title(self) -> str:
        """Title text of CollapsableFrame.

        Returns:
            Title text of CollapsableFrame.
        """
        return self.frame.title

    @title.setter
    def title(self, value: str) -> None:
        self.frame.title = value

    def _create_frame(
        self, title: str, collapsed: bool, enabled: bool, visible: bool, build_fn: Callable
    ) -> ui.CollapsableFrame:
        """Creates the underlying CollapsableFrame UI element.

        Args:
            title: Title of the collapsable frame.
            collapsed: Whether the frame is collapsed.
            enabled: Whether the frame is enabled.
            visible: Whether the frame is visible.
            build_fn: Function to call when building the frame content.

        Returns:
            The created CollapsableFrame UI element.
        """
        frame = ui.CollapsableFrame(
            title=title,
            name=title,
            height=0,
            collapsed=collapsed,
            visible=visible,
            enabled=enabled,
            build_fn=build_fn,
            style=get_style(),
            style_type_name_override="CollapsableFrame",
            horizontal_scrollbar_policy=ui.ScrollBarPolicy.SCROLLBAR_AS_NEEDED,
            vertical_scrollbar_policy=ui.ScrollBarPolicy.SCROLLBAR_ALWAYS_ON,
        )

        return frame


class ScrollingFrame(Frame):
    """Create a ScrollingFrame UI element with a specified size.

    Args:
        num_lines: Determines height of ScrollingFrame element in terms of the typical line height of UI
            elements. If not specified, the ScrollingFrame will fill the space it can in the UI Window.
        enabled: Frame is enabled.
        visible: Frame is visible.
        build_fn: A function that can be called to specify what should fill the Frame. Function should
            take no arguments. Return values will not be used.
    """

    def __init__(
        self, num_lines: int = None, enabled: bool = True, visible: bool = True, build_fn: Callable = None
    ) -> None:
        self._bottom_line_buffer_size = 4

        # Create a Frame UI element
        self._frame = self._create_frame(num_lines, enabled, visible, build_fn)
        UIWidgetWrapper.__init__(self, self.frame)

    def set_num_lines(self, num_lines: int) -> None:
        """Set the height of the ScrollingFrame element in terms of the typical line height of.

        other UI elements.

        Args:
            num_lines: Number of lines that should be shown in a ScrollingFrame.
        """
        self.frame.height = ui.Length(LABEL_HEIGHT * num_lines + self._bottom_line_buffer_size)

    def _create_frame(
        self, num_lines: int | None, enabled: bool, visible: bool, build_fn: Callable
    ) -> ui.ScrollingFrame:
        """Create the underlying scrolling frame UI element.

        Args:
            num_lines: Number of lines to display. If None, frame fills available space.
            enabled: Whether the frame is enabled for user interaction.
            visible: Whether the frame is visible.
            build_fn: Function called to populate the frame content.

        Returns:
            The created scrolling frame UI element.
        """
        if num_lines is not None:
            height = ui.Length(LABEL_HEIGHT * num_lines + self._bottom_line_buffer_size)
        else:
            height = ui.Fraction(1)
        frame = ui.ScrollingFrame(
            height=height,
            visible=visible,
            enabled=enabled,
            build_fn=build_fn,
            style=get_style(),
            style_type_name_override="ScrollingFrame",
            horizontal_scrollbar_policy=ui.ScrollBarPolicy.SCROLLBAR_AS_NEEDED,
            vertical_scrollbar_policy=ui.ScrollBarPolicy.SCROLLBAR_ALWAYS_ON,
        )

        return frame


##########################################################################################
#                           Simple Editable UI Field Wrappers
##########################################################################################


class IntField(UIWidgetWrapper):
    """Creates a IntField UI element.

    Args:
        label: Short descriptive text to the left of the IntField.
        tooltip: Text to appear when the mouse hovers over the IntField.
        default_value: Default value of the IntField.
        lower_limit: Lower limit of integer.
        upper_limit: Upper limit of integer.
        on_value_changed_fn: Function to be called when the value of the int is changed.
            The function should take an int as an argument. The return value will not be used.
    """

    def __init__(
        self,
        label: str,
        tooltip: str = "",
        default_value: int = 0,
        lower_limit: int = None,
        upper_limit: int = None,
        on_value_changed_fn: Callable = None,
    ) -> None:
        self._lower_limit = int(lower_limit) if lower_limit is not None else None
        self._upper_limit = int(upper_limit) if upper_limit is not None else None

        self._default_value = int(default_value)

        self._on_value_changed_fn = on_value_changed_fn

        int_field_frame = self._create_ui_widget(label, tooltip, self._default_value)

        super().__init__(int_field_frame)

    @property
    def label(self) -> ui.Label:
        """Get the UI Label element that contains the descriptive text."""
        return self._label

    @property
    def int_field(self) -> ui.IntField:
        """Get the UI IntField element."""
        return self._int_field

    def get_value(self) -> int:
        """Get the current value of the int field.

        Returns:
            int: Current value of the int field
        """
        return self.int_field.model.get_value_as_int()

    def get_upper_limit(self) -> int:
        """Get the upper limit on the IntField.

        Returns:
            int: Upper Limit on IntField
        """
        if self._upper_limit is None:
            return sys.maxsize
        return self._upper_limit

    def get_lower_limit(self) -> int:
        """Get the lower limit on the IntField.

        Returns:
            int: Lower Limit on IntField
        """
        if self._lower_limit is None:
            return sys.maxsize * -1
        return self._lower_limit

    def set_value(self, val: int) -> None:
        """Set the value in the IntField.

        Args:
            val: Value to fill IntField
        """
        self.int_field.model.set_value(int(val))

    def set_upper_limit(self, upper_limit: int) -> None:
        """Set upper limit of IntField.

        If current value is higher than upper_limit, the current value will be clipped to upper_limit.

        Args:
            upper_limit: Upper limit of IntField
        """
        upper_limit = int(upper_limit)
        self._upper_limit = upper_limit
        if self.get_value() > upper_limit:
            self.set_value(upper_limit)

    def set_lower_limit(self, lower_limit: int) -> None:
        """Set lower limit of IntField.

        If current value is lower than lower_limit, the current value will be clipped to lower_limit.

        Args:
            lower_limit: lower limit of IntField
        """
        lower_limit = int(lower_limit)
        self._lower_limit = lower_limit
        if self.get_value() < lower_limit:
            self.set_value(lower_limit)

    def set_on_value_changed_fn(self, on_value_changed_fn: Callable) -> None:
        """Set function that is called when the value of the IntField is modified.

        Args:
            on_value_changed_fn: Function that is called when the value of the IntField is modified.
                Function should take a int as the argument. The return value will not be used.
        """
        self._on_value_changed_fn = on_value_changed_fn

    def _on_value_changed_fn_wrapper(self, model: object) -> None:
        # Enforces upper and lower limits on value change
        model.set_max(self.get_upper_limit())
        model.set_min(self.get_lower_limit())

        val = model.get_value_as_int()
        if val < self.get_lower_limit():
            model.set_value(self.get_lower_limit())
            return
        if val > self.get_upper_limit():
            model.set_value(self.get_upper_limit())
            return

        if self._on_value_changed_fn is not None:
            self._on_value_changed_fn(val)

    def _create_ui_widget(self, label: str, tooltip: str, default_value: int) -> object:
        """Creates the UI widget containing a label and integer input field.

        Args:
            label: Short descriptive text to the left of the IntField.
            tooltip: Text to appear when the mouse hovers over the IntField.
            default_value: Default value of the IntField.

        Returns:
            The containing frame with the label and IntField elements.
        """
        containing_frame = Frame().frame
        with containing_frame:
            with ui.HStack():
                self._label = ui.Label(
                    label, width=LABEL_WIDTH, alignment=ui.Alignment.LEFT_CENTER, tooltip=format_tt(tooltip)
                )
                self._int_field = ui.IntDrag(
                    name="Field",
                    height=LABEL_HEIGHT,
                    alignment=ui.Alignment.LEFT_CENTER,
                    min=sys.maxsize * -1,
                    max=sys.maxsize,
                )
                self.int_field.model.set_value(default_value)
                add_line_rect_flourish(False)
            self.int_field.model.add_value_changed_fn(self._on_value_changed_fn_wrapper)
        return containing_frame


class FloatField(UIWidgetWrapper):
    """Creates a FloatField UI element.

    Args:
        label: Short descriptive text to the left of the FloatField.
        tooltip: Text to appear when the mouse hovers over the FloatField.
        default_value: Default value of the Float Field.
        step: Smallest increment that the user can change the float by when dragging mouse.
        format: Formatting string for float.
        lower_limit: Lower limit of float.
        upper_limit: Upper limit of float.
        on_value_changed_fn: Function to be called when the value of the float is changed.
            The function should take a float as an argument. The return value will not be used.
        on_end_edit_fn: Function to be called when the user finishes editing the FloatField.
            The function should take a float as an argument. The return value will not be used.
    """

    def __init__(
        self,
        label: str,
        tooltip: str = "",
        default_value: float = 0.0,
        step: float = 0.01,
        format: str = "%.2f",
        lower_limit: float = None,
        upper_limit: float = None,
        on_value_changed_fn: Callable = None,
        on_end_edit_fn: Callable = None,
    ) -> None:
        self._lower_limit = float(lower_limit) if lower_limit is not None else None
        self._upper_limit = float(upper_limit) if upper_limit is not None else None

        self._default_value = float(default_value)

        self._on_value_changed_fn = on_value_changed_fn
        self._on_end_edit_fn = on_end_edit_fn

        float_field_frame = self._create_ui_widget(label, tooltip, self._default_value, step, format)

        super().__init__(float_field_frame)

    @property
    def label(self) -> ui.Label:
        """Get the UI Label element that contains the descriptive text."""
        return self._label

    @property
    def float_field(self) -> ui.FloatField:
        """Get the UI FloatField element."""
        return self._float_field

    def get_value(self) -> float:
        """Return the current value of the FloatField.

        Returns:
            float: Current value of the FloatField
        """
        return self.float_field.model.get_value_as_float()

    def get_upper_limit(self) -> float:
        """Get the upper limit on the FloatField.

        Returns:
            float: Upper limit on FloatField
        """
        if self._upper_limit is None:
            return inf
        return self._upper_limit

    def get_lower_limit(self) -> float:
        """Get the lower limit on the FloatField.

        Returns:
            float: Lower limit on FloatField
        """
        if self._lower_limit is None:
            return -inf
        return self._lower_limit

    def set_value(self, val: float) -> None:
        """Set the value in the FloatField.

        Args:
            val: Value to fill FloatField
        """
        self.float_field.model.set_value(float(val))

    def set_upper_limit(self, upper_limit: float) -> None:
        """Set upper limit of FloatField.

        If current value is higher than upper_limit, the current value will be clipped to upper_limit.

        Args:
            upper_limit: Upper limit of FloatField
        """
        upper_limit = float(upper_limit)
        self._upper_limit = upper_limit
        if self.get_value() > upper_limit:
            self.set_value(upper_limit)

    def set_lower_limit(self, lower_limit: float) -> None:
        """Set lower limit of FloatField.

        If current value is lower than lower_limit, the current value will be clipped to lower_limit.

        Args:
            lower_limit: lower limit of FloatField
        """
        lower_limit = float(lower_limit)
        self._lower_limit = lower_limit
        if self.get_value() < lower_limit:
            self.set_value(lower_limit)

    def set_on_value_changed_fn(self, on_value_changed_fn: Callable) -> None:
        """Set function that is called when the value of the FloatField is modified.

        Args:
            on_value_changed_fn: Function that is called when the value of the FloatField is modified.
                Function should take a float as the argument. The return value will not be used.
        """
        self._on_value_changed_fn = on_value_changed_fn

    def set_on_end_edit_fn(self, on_end_edit_fn: Callable) -> None:
        """Set function that is called when the user finishes editing the FloatField.

        Args:
            on_end_edit_fn: Function that is called when the user finishes editing the FloatField.
                Function should take a float as the argument. The return value will not be used.
        """
        self._on_end_edit_fn = on_end_edit_fn

    def _on_value_changed_fn_wrapper(self, model: object) -> None:
        """Handle value changes in the float field model with limit enforcement.

        Enforces upper and lower limits and calls the user-defined callback if set.

        Args:
            model: The float field model that changed.
        """
        # Enforces upper and lower limits on value change
        model.set_max(self.get_upper_limit())
        model.set_min(self.get_lower_limit())
        val = model.get_value_as_float()
        if self.get_upper_limit() < val:
            val = self._upper_limit
            model.set_value(float(val + 1))
            return
        elif self.get_lower_limit() > val:
            val = self._lower_limit
            model.set_value(float(val - 1))
            return

        if self._on_end_edit_fn is not None:
            self._on_end_edit_fn(val)
        elif self._on_value_changed_fn is not None:
            self._on_value_changed_fn(val)

    def _create_ui_widget(self, label: str, tooltip: str, default_value: float, step: float, format: str) -> object:
        """Create the UI widget components for the FloatField.

        Sets up the containing frame, label, float drag widget, and event handlers.

        Args:
            label: Short descriptive text to the left of the FloatField.
            tooltip: Text to appear when the mouse hovers over the FloatField.
            default_value: Default value of the Float Field.
            step: Smallest increment that the user can change the float by when dragging mouse.
            format: Formatting string for float.

        Returns:
            The containing frame with all UI components.
        """
        containing_frame = Frame().frame
        with containing_frame:
            with ui.HStack():
                self._label = ui.Label(
                    label, width=LABEL_WIDTH, alignment=ui.Alignment.LEFT_CENTER, tooltip=format_tt(tooltip)
                )
                self._float_field = ui.FloatDrag(
                    name="FloatField",
                    width=ui.Fraction(1),
                    height=0,
                    alignment=ui.Alignment.LEFT_CENTER,
                    min=-inf,
                    max=inf,
                    step=step,
                    format=format,
                )
                self.float_field.model.set_value(default_value)
                add_line_rect_flourish(False)

            if self._on_end_edit_fn is None:
                self.float_field.model.add_value_changed_fn(self._on_value_changed_fn_wrapper)
            else:
                self.float_field.model.add_end_edit_fn(self._on_value_changed_fn_wrapper)

        return containing_frame


class StringField(UIWidgetWrapper):
    """Create StringField UI Element.

    Starting at use_folder_picker, the arguments to the StringField all pertain to the folder_picker.
    If the folder_picker is not used, these arguments may all be ignored.

    Args:
        label: Label to the left of the UI element.
        tooltip: Tooltip to display over the UI elements.
        default_value: Text to initialize in Stringfield.
        read_only: Prevents editing.
        multiline_okay: If True, allow newline character in input strings.
        on_value_changed_fn: Function called when value of StringField is changed.
            The function should take a string as an argument. The return value will not be used.
        use_folder_picker: Add a folder picker button to the right.
        item_filter_fn: Filter function to pass to the FilePicker. This function should take a string
            as an argument and return a boolean. When the user opens the file picker, every file in the directory they are
            viewing will be passed to item_filter_fn, and when True is returned, the file will be shown. When False is
            returned, the file will not be shown. This can be used to ensure that the user may only select valid file types.
        bookmark_label: Bookmark label to pass to the FilePicker. This will create a bookmark when the
            file picker is used with the label specified here.
        bookmark_path: Bookmark path to pass to the FilePicker. This will create a bookmark when the file
            picker is used with the path specified here.
        folder_dialog_title: Title for the folder picker dialog.
        folder_button_title: Title for the folder picker button.
    """

    def __init__(
        self,
        label: str,
        tooltip: str = "",
        default_value: str = "",
        read_only: bool = False,
        multiline_okay: bool = False,
        on_value_changed_fn: Callable = None,
        use_folder_picker: bool = False,
        item_filter_fn: Callable = None,
        bookmark_label: str = None,
        bookmark_path: str = None,
        folder_dialog_title: str = "Select Output Folder",
        folder_button_title: str = "Select Folder",
    ) -> None:
        self._default_value = default_value

        self._on_value_changed_fn = on_value_changed_fn

        self._item_filter_fn = item_filter_fn

        self._file_picker_frame = None
        self._file_picker_btn = None

        string_field_frame = self._create_ui_widget(
            label,
            default_value,
            tooltip,
            use_folder_picker,
            read_only,
            multiline_okay,
            bookmark_label,
            bookmark_path,
            folder_dialog_title,
            folder_button_title,
        )

        super().__init__(string_field_frame)

    @property
    def label(self) -> ui.Label:
        """UI Label element that contains the descriptive text.

        Returns:
            UI Label element that contains the descriptive text.
        """
        return self._label

    @property
    def string_field(self) -> ui.StringField:
        """UI StringField element.

        Returns:
            UI StringField element.
        """
        return self._string_field

    @property
    def file_picker_frame(self) -> ui.Frame:
        """UI Frame containing FilePicker.

        Returns:
            UI Frame containing FilePicker.
        """
        return self._file_picker_frame

    @property
    def file_picker_btn(self) -> ui.Button:
        """Button to activate file picker.

        Returns:
            Button to activate file picker.
        """
        return self._file_picker_btn

    def get_value(self) -> str:
        """Return the current value of the StringField.

        Returns:
            Current value of the StringField.
        """
        return self.string_field.model.get_value_as_string()

    def set_value(self, val: str) -> None:
        """Set the value of the StringField.

        Args:
            val: Value to fill StringField
        """
        self.string_field.model.set_value(val)

    def set_on_value_changed_fn(self, on_value_changed_fn: Callable) -> None:
        """Set function that is called when the value of the StringField is modified.

        Args:
            on_value_changed_fn: Function that is called when the value of the StringField is modified.
                Function should take a string as the argument. The return value will not be used.
        """
        self._on_value_changed_fn = on_value_changed_fn

    def set_item_filter_fn(self, item_filter_fn: Callable) -> None:
        """Set the filter function that will be used with the file picker.

        Args:
            item_filter_fn: Filter function that will be called to filter the files shown in the
                picker.  This function should take a string file_path as the argument. The return value
                should be a bool, with True indicating the the file should be shown to the user in the file picker.
        """
        self._item_filter_fn = item_filter_fn

    def set_read_only(self, read_only: bool) -> None:
        """Set this StringField to be read only.

        Args:
            read_only: If True, StringField cannot be modified through the UI; it can still be
                modified programmatically with set_value()
        """
        self.string_field.read_only = read_only

    def set_multiline_okay(self, multiline_okay: bool) -> None:
        """Set the StringFiled to allow the newline character.

        Args:
            multiline_okay: If True, allow newline character in strings.
        """
        self.string_field.multiline = multiline_okay

    def _item_filter_fn_wrapper(self, file: object) -> object:
        """Internal wrapper for the file filter function used in the folder picker.

        Args:
            file: File object to filter in the file picker dialog.

        Returns:
            The result of the filter function, or None if no filter is set.
        """
        if self._item_filter_fn is not None:
            return self._item_filter_fn(file.path)

    def _on_value_changed_fn_wrapper(self, model: object) -> None:
        """Internal wrapper for the value changed callback function.

        Args:
            model: The UI model that triggered the value change event.
        """
        val = model.get_value_as_string()
        if self._on_value_changed_fn is not None:
            self._on_value_changed_fn(val)

    def _create_ui_widget(
        self,
        label: str = "",
        default_val: str = " ",
        tooltip: str = "",
        use_folder_picker: bool = False,
        read_only: bool = False,
        multiline_okay: bool = True,
        bookmark_label: str = None,
        bookmark_path: str = None,
        folder_dialog_title: str = "Select Output Folder",
        folder_button_title: str = "Select Folder",
    ) -> object:
        """Create the UI widget components for the StringField.

        Args:
            label: Label to the left of the UI element.
            default_val: Text to initialize in StringField.
            tooltip: Tooltip to display over the UI elements.
            use_folder_picker: Add a folder picker button to the right.
            read_only: Prevents editing.
            multiline_okay: If True, allow newline character in input strings.
            bookmark_label: Bookmark label to pass to the FilePicker.
            bookmark_path: Bookmark path to pass to the FilePicker.
            folder_dialog_title: Title for the folder picker dialog.
            folder_button_title: Title for the folder picker button.

        Returns:
            The containing frame for the UI widget components.
        """
        containing_frame = Frame().frame
        with containing_frame:
            with ui.HStack():
                self._label = ui.Label(
                    label, width=LABEL_WIDTH, alignment=ui.Alignment.LEFT_CENTER, tooltip=format_tt(tooltip)
                )
                self._string_field = ui.StringField(
                    name="StringField",
                    width=ui.Fraction(1),
                    height=0,
                    alignment=ui.Alignment.LEFT_CENTER,
                    read_only=read_only,
                    multiline=multiline_okay,
                )
                self.string_field.model.set_value(default_val)

                if use_folder_picker:

                    def update_field(filename: object, path: object) -> None:
                        if filename == "":
                            val = path
                        elif filename[0] != "/" and path[-1] != "/":
                            val = path + "/" + filename
                        elif filename[0] == "/" and path[-1] == "/":
                            val = path + filename[1:]
                        else:
                            val = path + filename
                        self.string_field.model.set_value(val)

                    self.add_folder_picker_icon(
                        update_field,
                        self._item_filter_fn_wrapper,
                        bookmark_label,
                        bookmark_path,
                        dialog_title=folder_dialog_title,
                        button_title=folder_button_title,
                    )
                else:
                    add_line_rect_flourish(False)

                self.string_field.model.add_value_changed_fn(self._on_value_changed_fn_wrapper)
        return containing_frame

    def add_folder_picker_icon(
        self,
        on_click_fn: Callable,
        item_filter_fn: Callable = None,
        bookmark_label: str = None,
        bookmark_path: str = None,
        dialog_title: str = "Select Output Folder",
        button_title: str = "Select Folder",
    ) -> None:
        """Add a folder picker icon button to the StringField.

        Args:
            on_click_fn: Function to call when a file is selected from the picker.
            item_filter_fn: Filter function to pass to the FilePicker.
            bookmark_label: Bookmark label to pass to the FilePicker.
            bookmark_path: Bookmark path to pass to the FilePicker.
            dialog_title: Title for the folder picker dialog.
            button_title: Title for the folder picker button.
        """

        def open_file_picker() -> None:
            def on_selected(filename: object, path: object) -> None:
                on_click_fn(filename, path)
                file_picker.hide()

            def on_canceled(a: object, b: object) -> None:
                file_picker.hide()

            file_picker = FilePickerDialog(
                dialog_title,
                allow_multi_selection=False,
                apply_button_label=button_title,
                click_apply_handler=lambda a, b: on_selected(a, b),
                click_cancel_handler=lambda a, b: on_canceled(a, b),
                item_filter_fn=item_filter_fn,
                enable_versioning_pane=True,
            )
            if bookmark_label and bookmark_path:
                file_picker.toggle_bookmark_from_path(bookmark_label, bookmark_path, True)

        self._file_picker_frame = ui.Frame(width=0, tooltip=button_title)
        with self.file_picker_frame:
            self._file_picker_btn = ui.Button(
                name="IconButton",
                width=24,
                height=24,
                clicked_fn=open_file_picker,
                style=get_style()["IconButton.Image::FolderPicker"],
                alignment=ui.Alignment.RIGHT_TOP,
            )


##########################################################################################
#                               UI Button Wrappers
##########################################################################################


class Button(UIWidgetWrapper):
    """Create a Button UI Element.

    Args:
        label: Short descriptive text to the left of the Button.
        text: Text on the Button.
        tooltip: Text to appear when the mouse hovers over the Button.
        on_click_fn: Callback function that will be called when the button is pressed.
            Function should take no arguments.  The return value will not be used.
    """

    def __init__(self, label: str, text: str, tooltip: str = "", on_click_fn: Callable = None) -> None:
        self._on_click_fn = on_click_fn

        button_frame = self._create_ui_widget(label, text, tooltip)
        super().__init__(button_frame)

    @property
    def label(self) -> ui.Label:
        """UI Label element that contains the descriptive text.

        Returns:
            UI Label element that contains the descriptive text.
        """
        return self._label

    @property
    def button(self) -> ui.Button:
        """UI Button element.

        Returns:
            UI Button element.
        """
        return self._button

    def set_on_click_fn(self, on_click_fn: Callable) -> None:
        """Set the callback function for when the Button is clicked.

        Args:
            on_click_fn: Callback function for when Button is clicked.
                The function should take a single bool argument.  The return value will not be used.
        """
        self._on_click_fn = on_click_fn

    def trigger_click(self) -> None:
        """Trigger identical behavior as if the user pressed the Button through the UI."""
        self._on_clicked_fn_wrapper()

    def _on_clicked_fn_wrapper(self) -> None:
        """Internal wrapper that executes the user-defined click callback function if set."""
        if self._on_click_fn is not None:
            self._on_click_fn()

    def _create_ui_widget(self, label: str, text: str, tooltip: str) -> object:
        """Create the UI widget layout with label, button, and styling.

        Args:
            label: Short descriptive text to the left of the Button.
            text: Text on the Button.
            tooltip: Text to appear when the mouse hovers over the Button.

        Returns:
            The containing frame with the complete Button UI layout.
        """
        containing_frame = Frame().frame
        with containing_frame:
            with ui.HStack():
                self._label = ui.Label(
                    label, width=LABEL_WIDTH, alignment=ui.Alignment.LEFT_CENTER, tooltip=format_tt(tooltip)
                )
                self._button = ui.Button(
                    text.upper(),
                    name="Button",
                    width=BUTTON_WIDTH,
                    clicked_fn=self._on_clicked_fn_wrapper,
                    style=get_style(),
                    alignment=ui.Alignment.LEFT_CENTER,
                )
                ui.Spacer(width=5)
                add_line_rect_flourish(True)

        return containing_frame


class StateButton(UIWidgetWrapper):
    """Creates a State Button UI element.

    A StateButton is a button that changes between two states A and B when clicked.
    In state A, the StateButton has a_text written on it, and
    in state B, the StateButton has b_text written on it.

    Args:
        label: Short descriptive text to the left of the StateButton.
        a_text: Text on the StateButton in one of its two states.
        b_text: Text on the StateButton in the other of its two states.
        tooltip: Text that appears when the mouse hovers over the button.
        on_a_click_fn: A function that should be called when the button is clicked while in
            state A. Function should have 0 arguments. The return value will not be used.
        on_b_click_fn: A function that should be called when the button is clicked while in
            state B. Function should have 0 arguments. The return value will not be used.
        physics_callback_fn: A function that will be called on every physics step while the
            button is in state B (a_text was pressed). The function should have one argument for physics step size (float).
            The return value will not be used.

    Raises:
        ValueError: if state A and state B use the same text.
    """

    def __init__(
        self,
        label: str,
        a_text: str,
        b_text: str,
        tooltip: str = "",
        on_a_click_fn: Callable = None,
        on_b_click_fn: Callable = None,
        physics_callback_fn: Callable = None,
    ) -> None:
        self.a_text = a_text.upper()
        self.b_text = b_text.upper()

        if self.a_text == self.b_text:
            raise ValueError(f"State A and B must have different text: {self.a_text}")

        self._on_a_click_fn = on_a_click_fn
        self._on_b_click_fn = on_b_click_fn

        self._physics_callback_fn = physics_callback_fn
        self._physics_subscription = None
        self._physics_simulation_interface = omni.physics.core.get_physics_simulation_interface()

        state_btn_frame = self._creat_ui_widget(label, a_text, b_text, tooltip)

        super().__init__(state_btn_frame)

    @property
    def label(self) -> ui.Label:
        """UI Label element that contains the descriptive text.

        Returns:
            UI Label element that contains the descriptive text.
        """
        return self._label

    @property
    def state_button(self) -> ui.Button:
        """UI Button element.

        Returns:
            UI Button element.
        """
        return self._state_button

    def set_physics_callback_fn(self, physics_callback_fn: Callable) -> None:
        """Set a physics callback function that will be called on every physics step while the StateButton is.

            in state B.

        Args:
            physics_callback_fn: A function that will be called on every physics step while the
                button is in state B (a_text was pressed). The function should have one argument for physics step size (float).
                The return value will not be used.
        """
        self._physics_callback_fn = physics_callback_fn

    def set_on_a_click_fn(self, on_a_click_fn: Callable) -> None:
        """Set a function that is called when the button is clicked in state A.

        Args:
            on_a_click_fn: A function that is called when the button is clicked in state A.
                Function should take no arguments.  The return value will not be used.
        """
        self._on_a_click_fn = on_a_click_fn

    def set_on_b_click_fn(self, on_b_click_fn: Callable) -> None:
        """Set a function that is called when the button is clicked in state B.

        Args:
            on_b_click_fn: A function that is called when the button is clicked in state B.
                Function should take no arguments.  The return value will not be used.
        """
        self._on_b_click_fn = on_b_click_fn

    def is_in_a_state(self) -> bool:
        """Return True if the StateButton is in the a state.  False implies that it is in the b state.

        Returns:
            True when the StateButton is in the a state; False when in the b state.
        """
        return self.state_button.text == self.a_text

    def trigger_click_if_a_state(self) -> None:
        """If in the A state, trigger button to execute the same behavior as if it were clicked by the.

        user.  If the button is in the B state, nothing will happen.
        """
        if self.is_in_a_state():
            self.state_button.text = self.b_text
            self._on_clicked_fn_wrapper(True)

    def trigger_click_if_b_state(self) -> None:
        """If in the B state, trigger button to execute the same behavior as if it were clicked by the.

        user.  If the button is in the A state, nothing will happen.  This is distinct from calling
        reset() because the user on_b_click_fn() will be triggered.
        """
        if not self.is_in_a_state():
            self.state_button.text = self.a_text
            self._on_clicked_fn_wrapper(False)

    def get_current_text(self) -> str:
        """Get the current text on the button.

        Returns:
            The current text on the button.
        """
        return self.state_button.text

    def reset(self) -> None:
        """Reset StateButton to state A."""
        self.state_button.text = self.a_text
        self._remove_physics_callback()

    def cleanup(self) -> None:
        """Remove physics callback created by the StateButton if exists."""
        self._remove_physics_callback()

    def _create_physics_callback(self) -> None:
        """Create physics callback subscription for the StateButton."""
        self._physics_subscription = self._physics_simulation_interface.subscribe_physics_on_step_events(
            pre_step=False, order=0, on_update=self._physics_callback_fn
        )

    def _remove_physics_callback(self) -> None:
        """Remove the physics callback subscription."""
        self._physics_subscription = None

    def _on_clicked_fn_wrapper(self, value: object) -> None:
        """Handle button click events and manage state transitions.

        Args:
            value: True when button is clicked while in state A, False when clicked while in state B.
        """
        # Button pressed while saying a_text
        if value:
            if self._on_a_click_fn is not None:
                self._on_a_click_fn()
            if self._physics_callback_fn is not None:
                self._create_physics_callback()

        # Button pressed while saying b_text
        else:
            if self._on_b_click_fn is not None:
                self._on_b_click_fn()
            if self._physics_callback_fn is not None:
                self._remove_physics_callback()

    def _creat_ui_widget(self, label: str, a_text: str, b_text: str, tooltip: str) -> object:
        """Create the UI widget for the StateButton.

        Args:
            label: Short descriptive text to the left of the StateButton.
            a_text: Text on the StateButton in one of its two states.
            b_text: Text on the StateButton in the other of its two states.
            tooltip: Text that appears when the mouse hovers over the button.

        Returns:
            The containing frame for the StateButton UI widget.
        """

        def toggle() -> None:
            if self.state_button.text == a_text.upper():
                self.state_button.text = b_text.upper()
                self._on_clicked_fn_wrapper(True)
            else:
                self.state_button.text = a_text.upper()
                self._on_clicked_fn_wrapper(False)

        containing_frame = Frame().frame
        with containing_frame:
            with ui.HStack():
                self._label = ui.Label(
                    label, width=LABEL_WIDTH, alignment=ui.Alignment.LEFT_CENTER, tooltip=format_tt(tooltip)
                )
                self._state_button = ui.Button(
                    a_text.upper(),
                    name="Button",
                    width=BUTTON_WIDTH,
                    clicked_fn=toggle,
                    style=get_style(),
                    alignment=ui.Alignment.LEFT_CENTER,
                )
                ui.Spacer(width=5)
                ui.Spacer(width=ui.Fraction(1))
                ui.Spacer(width=10)
                with ui.Frame(width=0):
                    with ui.VStack():
                        ui.Spacer()
                        with ui.Placer(offset_x=0, offset_y=5):
                            ui.Rectangle(height=5, width=5, alignment=ui.Alignment.CENTER)
                        ui.Spacer()
                ui.Spacer(width=5)
        return containing_frame


class CheckBox(UIWidgetWrapper):
    """Create a CheckBox UI Element.

    Args:
        label: Short descriptive text to the left of the CheckBox
        default_value: If True, CheckBox will be checked.
        tooltip: Text to appear when the mouse hovers over the CheckBox.
        on_click_fn: Callback function that will be called when the CheckBox is pressed.
            Function should take a single bool argument.  The return value will not be used.
    """

    def __init__(
        self, label: str, default_value: bool = False, tooltip: str = "", on_click_fn: Callable = None
    ) -> None:
        self._on_click_fn = on_click_fn

        checkbox_frame = self._create_ui_widget(label, bool(default_value), tooltip)
        super().__init__(checkbox_frame)

    @property
    def label(self) -> ui.Label:
        """UI Label element that contains the descriptive text.

        Returns:
            UI Label element that contains the descriptive text.
        """
        return self._label

    @property
    def checkbox(self) -> ui.CheckBox:
        """UI CheckBox element.

        Returns:
            UI CheckBox element.
        """
        return self._checkbox

    def get_value(self) -> bool:
        """Get the current checked state of the CheckBox.

        Returns:
            True if CheckBox is checked, False otherwise.
        """
        return self.checkbox.model.get_value_as_bool()

    def set_value(self, val: bool) -> None:
        """Set the checked state of the CheckBox.

        Args:
            val: If True, set CheckBox to checked state.
        """
        self.checkbox.model.set_value(bool(val))

    def set_on_click_fn(self, on_click_fn: Callable) -> None:
        """Set the function that will be called when the CheckBox is clicked.

        Args:
            on_click_fn: Callback function for when CheckBox is clicked.
                The function should take a single bool argument. The return value will not be used.
        """
        self._on_click_fn = on_click_fn

    def _on_click_fn_wrapper(self, model: object) -> None:
        """Internal wrapper that handles CheckBox click events and calls the user-defined callback.

        Args:
            model: The UI model that triggered the click event.
        """
        if self._on_click_fn is not None:
            self._on_click_fn(model.get_value_as_bool())

    def _create_ui_widget(self, label: str, default_value: bool, tooltip: str) -> object:
        """Create the UI widget components for the CheckBox.

        Args:
            label: Descriptive text to display to the left of the CheckBox.
            default_value: Initial checked state of the CheckBox.
            tooltip: Text to appear when the mouse hovers over the CheckBox.

        Returns:
            The containing frame with the CheckBox UI elements.
        """
        containing_frame = Frame().frame
        with containing_frame:
            with ui.HStack():
                self._label = ui.Label(
                    label, width=LABEL_WIDTH, alignment=ui.Alignment.LEFT_CENTER, tooltip=format_tt(tooltip)
                )
                model = ui.SimpleBoolModel()
                model.set_value(default_value)
                self._checkbox = ui.CheckBox(model=model, tooltip=tooltip)
                model.add_value_changed_fn(self._on_click_fn_wrapper)

                add_line_rect_flourish()
        return containing_frame


##########################################################################################
#                          UI Selection Widget Wrappers
##########################################################################################


class DropDown(UIWidgetWrapper):
    """Create a DropDown UI element.

    A DropDown menu can be populated by the user, with a callback function specified
    for when an item is selected.

    Args:
        label: Short descriptive text to the left of the DropDown.
        tooltip: Text to appear when the mouse hovers over the DropDown.
        populate_fn: A user-defined function that returns a list[str] of items
            that should populate the drop-down menu. This Function should have 0 arguments.
        on_selection_fn: A user-defined callback function for when an element is selected
            from the DropDown. The function should take in a string argument of the selection.
            The return value will not be used.
        keep_old_selections: When the DropDown is repopulated with the user-defined populate_fn,
            the default behavior is to reset the selection in the DropDown to be at index 0. If the user
            sets keep_old_selections=True, when the DropDown is repopulated and the old selection is still one of
            the options, the new selection will match the old selection.
        add_flourish: Whether to add visual flourish to the DropDown element.
    """

    def __init__(
        self,
        label: str,
        tooltip: str = "",
        populate_fn: Callable = None,
        on_selection_fn: Callable = None,
        keep_old_selections: bool = False,
        add_flourish: bool = True,
    ) -> None:
        self._populate_fn = populate_fn
        self._on_selection_fn = on_selection_fn
        self._keep_old_selection = keep_old_selections
        self._items = []
        self._add_flourish = add_flourish

        combobox_frame = self._create_ui_widget(label, tooltip)
        super().__init__(combobox_frame)

    @property
    def label(self) -> ui.Label:
        """UI Label element that contains the descriptive text.

        Returns:
            UI Label element that contains the descriptive text.
        """
        return self._label

    @property
    def combobox(self) -> ui.ComboBox:
        """UI ComboBox element.

        Returns:
            UI ComboBox element.
        """
        return self._combobox

    def repopulate(self) -> None:
        """A function that the user can call to make the DropDown menu repopulate.

        This will call the populate_fn set by the user.
        """
        if self._populate_fn is None:
            carb.log_warn("Unable to repopulate drop-down meny without a populate_fn being specified")
            return
        else:
            new_items = self._populate_fn()

            old_selection = self.get_selection()
            self.set_items(new_items)
            new_selection = self.get_selection()

            if self._on_selection_fn is not None and new_selection != old_selection:
                # Call the user on_selection_fn if the selection has changed as a result of repopulate()
                self._on_selection_fn(new_selection)

    def set_populate_fn(self, populate_fn: Callable, repopulate: bool = True) -> None:
        """Set the populate_fn for this DropDown.

        Args:
            populate_fn: Function used to specify the options that fill the DropDown.
                Function should take no arguments and return a list[str].
            repopulate: If true, repopulate the DropDown using the new populate_fn.
        """
        self._populate_fn = populate_fn
        if repopulate:
            self.repopulate()

    def get_items(self) -> list[str]:
        """Get the items in the DropDown.

        Returns:
            A list of the options in the DropDown
        """
        return self._items

    def get_selection_index(self) -> int:
        """Get index of selection in DropDown menu.

        Returns:
            Index of selection in DropDown menu
        """
        return self.combobox.model.get_item_value_model().as_int

    def get_selection(self) -> str:
        """Get current selection in DropDown.

        Returns:
            Current selection in DropDown
        """
        if len(self._items) == 0:
            return None
        return self._items[self.get_selection_index()]

    def set_items(self, items: list[str], select_index: int = None) -> None:
        """Set the items in the DropDown explicitly.

        Args:
            items: New set of items in the DropDown
            select_index: Index of item to select.  If left as None, behavior is determined by the
                keep_old_selections flag.
        """
        if self._keep_old_selection and select_index is None:
            selection = self.get_selection()
            if selection is not None and selection in items:
                select_index = items.index(selection)

        self._items = items
        self.combobox.model = DynamicComboBoxModel(items)

        if select_index is not None and select_index < len(items):
            self.combobox.model.get_item_value_model().set_value(select_index)

        self.combobox.model.add_item_changed_fn(self._item_changed_fn_wrapper)

    def set_selection(self, selection: str) -> None:
        """Set the selected item in the DropDown.

        If the specifified selection is not in the DropDown, nothing will happen.

        Args:
            selection: Item to select in the DropDown
        """
        if selection in self.get_items():
            select_index = self.get_items().index(selection)
            self.set_selection_by_index(select_index)
        else:
            carb.log_warn(f"Item {selection} is not present in DropDown, and cannot be set as the selected item.")

    def set_selection_by_index(self, select_index: int) -> None:
        """Set the selected item in the DropDown by index.

        If the provided index is out of bounds, nothing will happen.

        Args:
            select_index: Index of item to select from DropDown
        """
        if select_index < len(self.get_items()):
            self.combobox.model.get_item_value_model().set_value(select_index)
        else:
            carb.log_warn(
                f"Index {select_index} is out of bounds. The DropDown currently has {len(self.get_items())} items in it."
            )

    def set_on_selection_fn(self, on_selection_fn: Callable) -> None:
        """Set the function that gets called when a new item is selected from the DropDown.

        Args:
            on_selection_fn: A function that is called when a new item is selected from the DropDown.
                The function should take in a string argument of the selection.  Its return value is not used.
        """
        self._on_selection_fn = on_selection_fn

    def set_keep_old_selection(self, val: bool) -> None:
        """Set keep_old_selection flag to determine behavior when repopulating the DropDown.

        Args:
            val: When the DropDown is repopulated with the user-defined populate_fn,
                the default behavior is to reset the selection in the DropDown to be at index 0.  If the user
                sets keep_old_selections=True, when the DropDown is repopulated and the old selection is still one of
                the options, the new selection will match the old selection, and the on_selection_fn() will not be called.
        """
        self._keep_old_selection = val

    def set_populate_fn_to_find_all_usd_objects_of_type(self, object_type: str, repopulate: bool = True) -> None:
        """Set the populate_fn to find all objects of a specified type on the USD stage.

        This is included as a convenience function to fulfill one common use-case for a DropDown menu.
        This overrides the populate_fn set by the user.

        Args:
            object_type: A string name of the type of USD object matching the output of
                get_prim_object_type(prim_path)
            repopulate: Repopulate the DropDown immediately. Defaults to True.
        """
        self.set_populate_fn(lambda: self._find_all_usd_objects_of_type(object_type), repopulate=repopulate)

    def trigger_on_selection_fn_with_current_selection(self) -> None:
        """Call the user on_selection_fn() with whatever item is currently selected in the DropDown."""
        if self._on_selection_fn is not None:
            self._on_selection_fn(self.get_selection())

    def _item_changed_fn_wrapper(self, model: object, val: object) -> None:
        """Handle item selection changes in the combo box model."""
        if self._on_selection_fn is not None:
            selected_item = self._items[model.get_item_value_model().as_int]
            self._on_selection_fn(selected_item)

    def _create_ui_widget(self, label: object, tooltip: object) -> object:
        """Create the UI widget components for the DropDown."""
        items = []
        combobox_model = DynamicComboBoxModel(items)
        containing_frame = Frame().frame
        with containing_frame:
            with ui.HStack():
                self._label = ui.Label(label, width=LABEL_WIDTH, alignment=ui.Alignment.LEFT_CENTER, tooltip=tooltip)
                self.combo_adorner_hstack = ui.HStack()
                self._combobox = ui.ComboBox(combobox_model)
                if self._add_flourish:
                    add_line_rect_flourish(False)

        return containing_frame

    def _find_all_usd_objects_of_type(self, obj_type: str) -> object:
        """Find all USD objects of the specified type on the stage."""
        items = []
        stage = get_context().get_stage()
        if stage:
            for prim in Usd.PrimRange(stage.GetPrimAtPath("/")):
                path = str(prim.GetPath())
                # Get prim type get_prim_object_type
                _type = get_prim_object_type(path)
                if _type == obj_type:
                    items.append(path)

        return items


class ColorPicker(UIWidgetWrapper):
    """Create a ColorPicker UI element to allow user-selection of an RGBA color.

    Args:
        label: Short descriptive text to the left of the ColorPicker
        default_value: RGBA color values between 0 and 1.
        tooltip: Text to appear when the mouse hovers over the ColorPicker.
        on_color_picked_fn: Function that will be called if the user picks a new color.
            Function should expect a List[float] as an argument with four RGBA color values between 0 and 1.
            The return value will not be used.
    """

    def __init__(
        self,
        label: str,
        default_value: list[float] = None,
        tooltip: str = "",
        on_color_picked_fn: Callable = None,
    ) -> None:
        if default_value is None:
            default_value = [1.0, 1.0, 1.0, 1.0]
        self._on_color_picked_fn = on_color_picked_fn
        default_value = list(default_value)

        color_picker_frame = self._create_ui_widget(label, default_value, tooltip)
        super().__init__(color_picker_frame)

    @property
    def label(self) -> ui.Label:
        """UI Label element that contains the descriptive text.

        Returns:
            UI Label element that contains the descriptive text.
        """
        return self._label

    @property
    def color_picker(self) -> ui.ColorWidget:
        """UI ColorWidget element.

        Returns:
            UI ColorWidget element.
        """
        return self._color_picker

    def get_color(self) -> list[float]:
        """Get the RGBA value of the selected color.

        Returns:
            RGBA color value with four values between 0 and 1.
        """
        color = []
        for item in self.color_picker.model.get_item_children():
            val = self.color_picker.model.get_item_value_model(item).get_value_as_float()
            color.append(val)
        return color

    def set_color(self, color: list[float]) -> None:
        """Set the RGBA color value of the selected color.

        Args:
            color: RGBA color value with four values between 0 and 1.
        """
        color = list(color)
        for i, item in enumerate(self.color_picker.model.get_item_children()):
            val = self.color_picker.model.get_item_value_model(item).set_value(color[i])
            color.append(val)

        self._on_color_picked_fn_wrapper()

    def set_on_color_picked_fn(self, on_color_picked_fn: Callable) -> None:
        """Set the function that should be called if the user picks a new color.

        Args:
            on_color_picked_fn: Function that will be called if the user picks a new color.
                Function should expect a List[float] as an argument with four RGBA color values between 0 and 1.
                The return value will not be used.
        """
        self._on_color_picked_fn = on_color_picked_fn

    def _on_color_picked_fn_wrapper(self, *worthless_args: object) -> None:
        """Internal wrapper that calls the user-defined color picker callback function.

        Args:
            *worthless_args: Arguments passed by the UI event system that are not used.
        """
        if self._on_color_picked_fn is not None:
            self._on_color_picked_fn(self.get_color())

    def _create_ui_widget(self, label: str, default_value: list[float], tooltip: str) -> object:
        """Create the UI widget elements for the ColorPicker.

        Args:
            label: Short descriptive text to the left of the ColorPicker.
            default_value: RGBA color values between 0 and 1.
            tooltip: Text to appear when the mouse hovers over the ColorPicker.

        Returns:
            The containing frame with the ColorPicker UI elements.
        """
        containing_frame = Frame().frame
        with containing_frame:
            with ui.HStack():
                self._label = ui.Label(
                    label, width=LABEL_WIDTH, alignment=ui.Alignment.LEFT_CENTER, tooltip=format_tt(tooltip)
                )
                self._color_picker = ui.ColorWidget(*default_value, width=BUTTON_WIDTH)
                self.color_picker.model.add_end_edit_fn(self._on_color_picked_fn_wrapper)
                ui.Spacer(width=5)
                add_line_rect_flourish()
        return containing_frame


##########################################################################################
#                      UI Information Communication Wrappers
##########################################################################################


class TextBlock(UIWidgetWrapper):
    """Create a text block that is only modifiable through code. The user may not set.

    the value of the text in the UI.

    Args:
        label: Short description of the contents of the TextBlock.
        text: Text to put in the TextBlock.
        tooltip: Text to appear when the mouse hovers over the TextBlock.
        num_lines: Number of lines that should be visible in the TextBlock at one time.
        include_copy_button: Include a copy_to_clipboard button.
    """

    def __init__(
        self, label: str, text: str = "", tooltip: str = "", num_lines: int = 5, include_copy_button: bool = True
    ) -> None:
        self._copy_btn = None

        text_block_frame = self._create_ui_widget(num_lines, label, text, tooltip, include_copy_button)
        super().__init__(text_block_frame)

    @property
    def label(self) -> ui.Label:
        """UI Label element that contains the descriptive text.

        Returns:
            UI Label element that contains the descriptive text.
        """
        return self._label

    @property
    def scrolling_frame(self) -> ui.ScrollingFrame:
        """Scrolling Frame that contains the TextBlock text.

        Returns:
            Scrolling Frame that contains the TextBlock text.
        """
        return self._scrolling_frame

    @property
    def copy_btn(self) -> ui.Button:
        """Copy Button. If the TextBlock was built without a copy button, this will return None.

        Returns:
            Copy Button. If the TextBlock was built without a copy button, this will return None.
        """
        return self._copy_btn

    @property
    def text_block(self) -> ui.Label:
        """UI element that contains the text in the text block.

        Returns:
            UI element that contains the text in the text block.
        """
        return self._text_block

    def get_text(self) -> str:
        """Get the current text in the text block.

        Returns:
            Text in the text block.
        """
        return self.text_block.text

    def set_text(self, text: str) -> None:
        """Set the text content in the text block.

        Args:
            text: Text to set in the text block.
        """
        self.text_block.text = text

    def set_num_lines(self, num_lines: int) -> None:
        """Set the number of lines that should be visible in the TextBlock at one time.

        Args:
            num_lines: Number of lines that should be visible in the TextBlock at one time.
        """
        self.scrolling_frame.set_num_lines(num_lines)

    def _create_ui_widget(self, num_lines: int, label: str, text: str, tooltip: str, include_copy_btn: bool) -> object:
        """Create the UI widget components for the TextBlock.

        Args:
            num_lines: Number of visible lines in the TextBlock.
            label: Descriptive text label for the TextBlock.
            text: Initial text content.
            tooltip: Tooltip text for the TextBlock.
            include_copy_btn: Whether to include a copy to clipboard button.

        Returns:
            The containing frame with all UI components.
        """
        containing_frame = Frame().frame
        with containing_frame:
            with ui.VStack(style=get_style(), spacing=5):
                with ui.HStack():
                    self._label = ui.Label(
                        label, width=LABEL_WIDTH / 2, alignment=ui.Alignment.LEFT_TOP, tooltip=format_tt(tooltip)
                    )
                    self._scrolling_frame = ScrollingFrame(num_lines=num_lines)
                    with self._scrolling_frame:
                        self._text_block = ui.Label(
                            text,
                            style_type_name_override="Label::label",
                            word_wrap=True,
                            alignment=ui.Alignment.LEFT_TOP,
                        )
                    if include_copy_btn:
                        with ui.Frame(width=0, tooltip="Copy To Clipboard"):
                            self._copy_btn = ui.Button(
                                name="IconButton",
                                width=20,
                                height=20,
                                clicked_fn=lambda: on_copy_to_clipboard(to_copy=self._text_block.text),
                                style=get_style()["IconButton.Image::CopyToClipboard"],
                                alignment=ui.Alignment.RIGHT_TOP,
                            )
        return containing_frame


class XYPlot(UIWidgetWrapper):
    """Creates an interactive XY plot widget for visualizing data with multiple plot support, axis scaling, and legends.

    The plot widget supports multiple overlapping datasets, customizable axis ranges, interactive tooltips that display
    coordinates and function values on mouse hover, and optional legends with color coding. Data is automatically
    interpolated for smooth visualization when plotted points exceed the internal limit.

    Args:
        label: Short descriptive text to the left of the plot.
        tooltip: Text to appear when the mouse hovers over the plot label.
        x_data: X coordinates for plotting. Can be a single list of floats for one plot, or a list of lists where
            each inner list contains x coordinates for a separate plot. Must have the same shape as y_data.
        y_data: Y coordinates for plotting. Can be a single list of floats for one plot, or a list of lists where
            each inner list contains y coordinates for a separate plot. Must have the same shape as x_data.
        x_min: Minimum value of x shown on the plot. If not specified, automatically determined from x_data.
        x_max: Maximum value of x shown on the plot. If not specified, automatically determined from x_data.
        y_min: Minimum value of y shown on the plot. If not specified, automatically determined from y_data.
        y_max: Maximum value of y shown on the plot. If not specified, automatically determined from y_data.
        x_label: Label for the X axis.
        y_label: Label for the Y axis.
        plot_height: Height of the plot in text line units.
        show_legend: Whether to display a legend on the plot.
        legends: Custom legend labels for each plot. If not specified, automatic names 'F_i(x)' are generated.
        plot_colors: RGB color values for each plot as lists of [r,g,b] integers in range [0,255]. If not specified,
            colors are automatically generated.
    """

    def __init__(
        self,
        label: str,
        tooltip: str = "",
        x_data: list[list] | list = None,
        y_data: list[list] | list = None,
        x_min: float = None,
        x_max: float = None,
        y_min: float = None,
        y_max: float = None,
        x_label: str = "X",
        y_label: str = "Y",
        plot_height: int = 10,
        show_legend: bool = False,
        legends: list[str] = None,
        plot_colors: list[list[int]] = None,
    ) -> None:
        if y_data is None:
            y_data = []
        if x_data is None:
            x_data = []
        self._has_built = False

        self._x_min = float(x_min) if x_min is not None else None
        self._x_max = float(x_max) if x_max is not None else None

        self._y_min = float(y_min) if y_min is not None else None
        self._y_max = float(y_max) if y_max is not None else None

        self._x_axis_float_fields = []
        self._y_axis_float_fields = []

        # These will be immediately resolved in set_data
        self._no_data = None
        self._is_plot_visible = None

        # Check assertions around data shape, reshape data; set class variables: self._no_data, self._is_plot_visible
        self.set_data(x_data, y_data)

        self._show_legend = bool(show_legend)
        self._legends = legends

        self._label = label
        self._tooltip = tooltip
        self._x_label = x_label
        self._y_label = y_label
        self._plot_num_lines = int(plot_height)

        self._data_colors = None
        if plot_colors is not None:
            self.set_plot_colors(plot_colors)

        self._num_points_per_plot = 3000

        plot_frame = self._create_ui_widget()
        super().__init__(plot_frame)

    def get_x_data(self) -> list[list[float]]:
        """x_data in the plot.

        Returns:
            A possibly ragged list of lists where each ith inner list is the x coordinates for plot i.
        """
        return self._x_data

    def get_y_data(self) -> list[list[float]]:
        """y_data in the plot.

        Returns:
            A possibly ragged list of lists where each ith inner list is the y coordinates for plot i.
        """
        return self._y_data

    def get_x_min(self) -> float:
        """Get the minimum value of x shown in the plot.

        Returns:
            Minimum value of x shown in the plot.
        """
        if self._x_min is not None:
            return float(self._x_min)
        elif self._x_data is not None:
            return self._get_ragged_data_min(self._x_data)
        else:
            return None

    def get_y_min(self) -> float:
        """Get the minimum value of y shown in the plot.

        Returns:
            Minimum value of y shown in the plot.
        """
        if self._y_min is not None:
            return float(self._y_min)
        elif self._y_data is not None:
            return self._get_ragged_data_min(self._y_data)
        else:
            return None

    def get_x_max(self) -> float:
        """Get the maximum value of x shown in the plot.

        Returns:
            Maximum value of x shown in the plot.
        """
        if self._x_max is not None:
            return float(self._x_max)
        elif self._x_data is not None:
            return self._get_ragged_data_max(self._x_data)
        else:
            return None

    def get_y_max(self) -> float:
        """Get the maximum value of y shown in the plot.

        Returns:
            Maximum value of y shown in the plot.
        """
        if self._y_max is not None:
            return float(self._y_max)
        elif self._y_data is not None:
            return self._get_ragged_data_max(self._y_data)
        else:
            return None

    def get_legends(self) -> list[str]:
        """Get the legends for the plotted data.

        Returns:
            Legends for the plotted data
        """
        legends = []
        if self._legends is not None:
            for legend in self._legends:
                legends.append(legend)

        i = len(legends)
        while i < len(self.get_x_data()):
            legends.append(f"F_{i}(x)")
            i += 1

        if len(legends) > len(self._x_data):
            return legends[: len(self._x_data)]

        return legends

    def get_plot_height(self) -> int:
        """Get the height of the plot, proportional to the height of a line of text.

        Returns:
            Height of the plot
        """
        return self._plot_num_lines

    def get_plot_colors(self) -> list[list[int]]:
        """Get the colors of the data in the plot.

        Returns:
            List of lists where each inner list has length 3 corresponding to r,g,b values.
        """
        if self._data_colors is None:
            return [self._convert_hex_to_rgb(data_color) for data_color in self._get_distinct_colors(len(self._x_data))]
        else:
            return [self._convert_hex_to_rgb(data_color) for data_color in self._data_colors]

    def set_plot_color_by_index(self, index: int, r: int, g: int, b: int) -> None:
        """Set the color of a specific plot.

        Args:
            index: Index of the plot corresponding to the rows of x_data and y_data.
            r: Value for red in [0,255]
            g: Value for green in [0,255]
            b: Value for blue in [0,255]
        """
        if index >= len(self._x_data):
            carb.log_error("Index out of bounds for color on plot")
            return
        self._data_colors = self._get_data_colors(len(self._x_data))

        self._data_colors[index] = self._convert_rgb_to_hex(r, g, b)

        if self._has_built:
            self._plot_frames[index].rebuild()
            self._legend_frame.rebuild()

    def set_plot_colors(self, plot_colors: list[list[int]]) -> None:
        """Set the colors for every plot.

        Args:
            plot_colors: A list of lists where each index corresponds to the rows of x_data and y_data.
                Each inner list must contain [r,g,b] color values in [0,255]
        """
        if self._data_colors is None:
            self._data_colors = [self._convert_rgb_to_hex(r, g, b) for (r, g, b) in plot_colors]
        else:
            for i in range(len(plot_colors)):
                if i <= len(self._data_colors):
                    self._data_colors[i] = self._convert_rgb_to_hex(*plot_colors[i])
                else:
                    self._data_colors.append(self._convert_rgb_to_hex(*plot_colors[i]))

        if self._has_built:
            for plot in self._plot_frames:
                plot.rebuild()
            self._legend_frame.rebuild()

    def set_x_min(self, x_min: float) -> None:
        """Set the minimum value of x shown on the plot.

        If this value is not less than x_max, x_max will be updated to x_min + 1.

        Args:
            x_min: Minimum value of x shown on the plot.
        """
        x_min = float(x_min)
        self._x_min = x_min

        if self._has_built:
            self._x_min_float_drag.model.set_value(x_min)

    def set_y_min(self, y_min: float) -> None:
        """Set the minimum value of y shown on the plot.

        If this value is not less than y_max, y_max will be updated to y_min + 1.

        Args:
            y_min: Minimum value of y shown on the plot.
        """
        y_min = float(y_min)
        self._y_min = y_min

        if self._has_built:
            self._y_min_float_drag.model.set_value(y_min)

    def set_x_max(self, x_max: float) -> None:
        """Set maximum value of x shown on the plot.

        If this value is not greater than x_min, x_min will be updated to x_max - 1.

        Args:
            x_max: Maximum value of x shown on the plot.
        """
        x_max = float(x_max)
        self._x_max = x_max

        if self._has_built:
            self._x_max_float_drag.model.set_value(x_max)

    def set_y_max(self, y_max: float) -> None:
        """Set maximum value of y shown on the plot.

        If this value is not greater than y_min, y_min will be updated to y_max - 1.

        Args:
            y_max: Maximum value of y shown on the plot.
        """
        y_max = float(y_max)
        self._y_max = y_max

        if self._has_built:
            self._y_max_float_drag.model.set_value(y_max)

    def set_legend_by_index(self, idx: int, legend: str) -> None:
        """Set the legend for a specific plot whose index corresponds to the rows of x_data and y_data.

        Args:
            idx: Index of legend to set.
            legend: Legend
        """
        if idx >= len(self._x_data) or idx < -len(self._x_data):
            carb.log_error("Legend index out of bounds")
        legends = self.get_legends()
        legends[idx] = legend

        self._legends = legends
        if self._has_built:
            self._legend_frame.invalidate_raster()
            self._legend_frame.rebuild()

    def set_legends(self, legends: list[str]) -> None:
        """Set legends for each plot.

        Args:
            legends: List of legends for each plot.
        """
        if len(legends) != len(self._x_data):
            carb.log_error("Number of legends must match the number of plots")
        self._legends = legends
        if self._has_built:
            self._legend_frame.invalidate_raster()
            self._legend_frame.rebuild()

    def set_plot_height(self, plot_height: int) -> None:
        """Set the height of the plot.

        Args:
            plot_height: Height of the plot, proportional to the height of a line of text.
        """
        plot_height = int(plot_height)
        self._plot_num_lines = plot_height
        if self._has_built:
            self.container_frame.rebuild()

    def set_show_legend(self, show_legend: bool) -> None:
        """Hide or show the legend for this Widget.

        Args:
            show_legend: If True, show a legend for the Widget.
        """
        show_legend = bool(show_legend)
        self._show_legend = show_legend
        if self._has_built:
            self._show_legend_cb.model.set_value(show_legend)

    def set_data(self, x_data: list[list] | list, y_data: list[list] | list) -> None:
        """Set the data to plot.

        Args:
            x_data: A possibly ragged list of lists where each ith inner list is the x coordinates for plot i.
                For a single plot, the data may be provided as a list of floats.  x_data must have exactly the same shape as y_data.
            y_data: A possibly ragged list of lists where each ith inner list is the y coordinates for plot i.
                For a single plot, the data may be provided as a list of floats.  y_data must have exactly the same shape as x_data.
        """
        if len(x_data) != len(y_data):
            carb.log_error("x_data and y_data arguments must have the same shape.")

        self._no_data = True

        if len(x_data) == 0:
            self._x_data = [[]]
            self._y_data = [[]]
            self._is_plot_visible = []
            return

        if not isinstance(x_data[0], Iterable) and not isinstance(y_data[0], Iterable):
            if len(x_data) > 1:
                self._no_data = False
            x_data = [x_data]
            y_data = [y_data]

        for i in range(len(x_data)):
            if len(x_data[i]) != len(y_data[i]):
                carb.log_error(f"x_data and y_data arguments must have the same shape.  Mismatch found at index {i}")
            if len(x_data[i]) > 1:
                self._no_data = False

        self._x_data = x_data
        self._y_data = y_data

        self._is_plot_visible = [True] * len(self._x_data)

        if self._has_built:
            self.container_frame.rebuild()

    def set_plot_visible_by_index(self, index: int, visible: bool) -> None:
        """Hide or show a specific plot.

        Args:
            index: Index of plot to show.
            visible: If True, show the plot, otherwise hide it.
        """
        if index >= len(self._x_data):
            carb.log_error("Index out of bounds for plot.")
            return

        self._is_plot_visible[index] = visible

        if self._has_built:
            if len(self._show_plot_cbs) == 0:
                self._plot_frames[index].visible = visible
            else:
                self._show_plot_cbs[index].model.set_value(visible)

    def _get_data_colors(self, num_colors: int) -> list[int]:
        """Get hex colors to use in the plot.

        Args:
            num_colors: Number of colors to generate.

        Returns:
            List of hex color values for the plot data.
        """
        # Get hex colors to use in the plot

        if self._data_colors is None:
            return self._get_distinct_colors(num_colors)
        elif len(self._data_colors) < num_colors:
            return self._data_colors + self._get_distinct_colors(num_colors - len(self._data_colors))
        else:
            return self._data_colors

    def _get_ragged_data_min(self, data: object) -> float:
        """Get the minimum value in a set of ragged data.

        Args:
            data: Ragged data structure to find minimum value in.

        Returns:
            Minimum value found in the data.
        """
        # Get the minimum value in a set of ragged data

        data_min = None

        for row in data:
            if len(row) > 0:
                m = np.min(row)
                if data_min is None or m < data_min:
                    data_min = m
        if data_min is None:
            return 0
        return float(data_min)

    def _get_ragged_data_max(self, data: object) -> float:
        """Get the maximum value in a set of ragged data.

        Args:
            data: Ragged data structure to find maximum value in.

        Returns:
            Maximum value found in the data.
        """
        # Get the maximum value in a set of ragged data

        data_max = None

        for row in data:
            if len(row) > 0:
                m = np.max(row)
                if data_max is None or m > data_max:
                    data_max = m
        if data_max is None:
            return 0
        return float(data_max)

    def _get_interpolated_data(self, x_min: float = None, x_max: float = None) -> object:
        """Get all data necessary for plotting.

        Args:
            x_min: Minimum x value for interpolation range.
            x_max: Maximum x value for interpolation range.

        Returns:
            A tuple containing (x_fracs, y_vals, x_min, x_max) where:
            - x_fracs: Array (N x 2) corresponding to the fraction of x values covered by min and max x values for each plot
            - y_vals: Array (N x ?) list of y values corresponding to each plot (shape may be ragged)
            - x_min: Minimum value of x to be shown in the plot
            - x_max: Maximum value of x to be shown in the plot
        """
        if self._no_data:
            # There is no data at all
            return [[0, 1]], [[]], 0, 1

        if x_min is None:
            x_min = self._get_ragged_data_min(self._x_data)
        if x_max is None:
            x_max = self._get_ragged_data_max(self._x_data)

        if x_max <= x_min:
            return [[0, 1]] * len(self._y_data), [[]] * len(self._y_data), 0, 1

        spacing = (x_max - x_min) / self._num_points_per_plot
        x_val_range = np.arange(x_min, x_max + 0.5 * spacing, spacing)

        y_vals = []
        x_fracs = []

        for i in range(len(self._y_data)):
            y_vals.append([])
            for j in range(len(self._y_data[i]) - 1):
                y_low = self._y_data[i][j]
                y_high = self._y_data[i][j + 1]
                inds = np.where((x_val_range >= self._x_data[i][j]) & (x_val_range < self._x_data[i][j + 1]))[0]
                if len(inds) == 0:
                    continue
                x_frac = (x_val_range[inds] - self._x_data[i][j]) / (self._x_data[i][j + 1] - self._x_data[i][j])
                interp_fun = lambda x: y_low + x_frac * (y_high - y_low)
                y_vals[-1].extend(interp_fun(x_val_range[inds]))

            if len(self._x_data[i]) <= 1:
                x_fracs.append([0.0, 1.0])
                continue

            inds = np.where((x_val_range >= self._x_data[i][0]) & (x_val_range <= self._x_data[i][-1]))[0]
            if len(inds) > 0:
                x_fracs.append(
                    [
                        (x_val_range[inds[0]] - x_min) / (x_max - x_min),
                        (x_val_range[inds[-1]] - x_min) / (x_max - x_min),
                    ]
                )
            elif self._x_data[i][0] > x_val_range[-1]:
                x_fracs.append([1.0, 1.0])
            else:
                x_fracs.append([0.0, 0.0])

        return x_fracs, y_vals, x_min, x_max

    def _convert_rgb_to_hex(self, r: int, g: int, b: int) -> int:
        """Convert an RGB color to a hex value.

        Args:
            r: Red color component.
            g: Green color component.
            b: Blue color component.

        Returns:
            Hex color value.
        """
        # convert an rgb color to a hex value
        return 0xFF * 16**6 + b * 16**4 + g * 16**2 + r

    def _convert_hex_to_rgb(self, hex_color: int) -> tuple[int]:
        """Convert a hex color value to RGB components.

        Args:
            hex_color: Hex color value to convert.

        Returns:
            Tuple of (r, g, b) color components.
        """

        def residue(number: object) -> object:
            mask = 0xFFFFFF00
            number = number - (number & mask)
            return number

        r = residue(hex_color)
        g = residue(int(hex_color / 16**2))
        b = residue(int(hex_color / 16**4))
        return r, g, b

    def _get_distinct_colors(self, num_colors: int) -> list[int]:
        """This function returns a list of distinct colors for plotting.

        Args:
            num_colors: The number of colors to generate.

        Returns:
            A list of distinct colors in hexadecimal format 0xFFBBGGRR.
        """
        import colorsys

        colors = []

        hues = [0, 0.25, 0.5, 0.9]
        sorted_hues = [0, 0.25, 0.5, 0.9]
        while len(hues) < num_colors:
            ind = np.argmax(np.diff(sorted_hues))
            hues.append((sorted_hues[ind] + sorted_hues[ind + 1]) / 2)
            sorted_hues.append(hues[-1])
            sorted_hues.sort()

        for i in range(num_colors):
            hue = hues[i]
            saturation = 0.8
            value = 1
            rgb = colorsys.hsv_to_rgb(hue, saturation, value)
            r, g, b = [int(255 * x) for x in rgb]
            color_hex = self._convert_rgb_to_hex(r, g, b)
            colors.append(color_hex)

        return colors

    def _get_fn_y_value(self, idx: int, x_value: float, decimals: int) -> float:
        """Get value of each plot evaluated at x_value using linear interpolation if needed.

        Args:
            idx: Index of the plot to evaluate.
            x_value: X coordinate to evaluate the function at.
            decimals: Number of decimal places to round the result to.

        Returns:
            Y value of the plot at the given x coordinate.
        """
        # Get value of each plot evaluated at x_value.  This may require linear interpolation
        x_data = self._x_data[idx]
        y_data = self._y_data[idx]

        if x_value == x_data[-1]:
            return np.round(y_data[-1], decimals=decimals)

        for i in range(len(x_data) - 1):
            if x_value >= x_data[i] and x_value < x_data[i + 1]:
                frac = (x_value - x_data[i]) / (x_data[i + 1] - x_data[i])
                return np.round(y_data[i] + frac * (y_data[i + 1] - y_data[i]), decimals=decimals)

        return ""

    def _mouse_moved_on_plot(self, x: float, y: float, *args: object) -> None:
        """Show a tooltip with x,y and function values when mouse moves over the plot.

        Args:
            x: Mouse x coordinate.
            y: Mouse y coordinate.
            *args: Additional arguments from the mouse event.
        """
        # Show a tooltip with x,y and function values
        if self._no_data:
            # There is no data in the plots, so do nothing
            return

        tt_frame = self._tooltip_frame
        width = tt_frame.computed_width - 14
        height = tt_frame.computed_height - 8
        x_pos = tt_frame.screen_position_x + 8
        y_pos = tt_frame.screen_position_y + 5

        tt_frame.tooltip_offset_x = 0
        tt_frame.tooltip_offset_y = 0

        x_min = self.get_x_min()
        x_max = self.get_x_max()
        y_max = self.get_y_max()
        y_min = self.get_y_min()

        if x_max == x_min or y_max == y_min:
            # Can only happen when the user is intentionally trying to break things
            return

        num_decimals_x = int(max(0, np.log10(5 / (x_max - x_min)) + 2))
        num_decimals_y = int(max(0, np.log10(5 / (y_max - y_min)) + 2))

        x_value = np.round((x_max - x_min) * (x - x_pos) / width + x_min, decimals=num_decimals_x)
        y_value = np.round(y_max - (y_max - y_min) * (y - y_pos) / height, decimals=num_decimals_y)
        tt_string = f"x = {x_value},"
        tt_string += f"y = {y_value}\n\n"
        legends = self.get_legends()
        for i in range(len(self._interpolated_y_data)):
            if not self._plot_frames[i].visible:
                continue
            tt_string += legends[i] + " : "
            tt_string += f"{self._get_fn_y_value(i,x_value,num_decimals_y)}\n"
        tt_frame.set_tooltip(tt_string)

    def _on_mouse_released(self, *args: object) -> None:
        """Handles mouse release events to hide the plot tooltip.

        Args:
            *args: Variable arguments from the mouse release event.
        """
        # Hide the tooltip off screen
        self._tooltip_frame.tooltip = " "
        self._tooltip_frame.tooltip_offset_x = -2000
        self._tooltip_frame.tooltip_offset_y = -2000

    def _create_ui_widget(self) -> object:
        """Creates the main UI container frame for the XY plot widget.

        Returns:
            The container frame with the build function set to construct the plot widget.
        """
        containing_frame = ui.Frame(build_fn=self._build_widget)

        return containing_frame

    def _build_widget(self) -> None:
        """Builds the complete XY plot widget UI including plots, axes, legends, and controls.

        Creates all UI elements including the plot area, axis labels, legend frame, and interactive
        tooltip functionality. Sets up event handlers for mouse interactions and axis scaling.
        """
        self._plot_frames = []
        self._plots = []

        self._color_widgets = []
        self._show_plot_cbs = []

        LINE_HEIGHT = 23

        label = self._label
        tooltip = self._tooltip
        x_label = self._x_label
        y_label = self._y_label
        plot_num_lines = self._plot_num_lines

        plot_height = plot_num_lines * LINE_HEIGHT

        x_fracs, y_data, x_min, x_max = self._get_interpolated_data(self._x_min, self._x_max)
        self._x_min = x_min
        self._x_max = x_max
        self._interpolated_y_data = y_data
        self._x_fracs = x_fracs

        self._data_colors = self._get_data_colors(len(y_data))

        def on_show_legend(model: object) -> None:
            if model.get_value_as_bool():
                self._show_legend = True
                self._legend_frame.visible = True
                self._legend_frame.rebuild()
            else:
                self._show_legend = False
                self._legend_frame.visible = False

        def on_color_widget_set_plot_color(idx: object) -> None:
            plot_frame = self._plot_frames[idx]
            color_widget = self._color_widgets[idx]

            rgb_color = []
            for item in color_widget.model.get_item_children():
                val = color_widget.model.get_item_value_model(item).get_value_as_float()
                rgb_color.append(int(255 * val))
            hex_color = self._convert_rgb_to_hex(*rgb_color[:3])
            self._data_colors[idx] = hex_color

            plot_frame.rebuild()

        def build_legend_frame() -> None:
            def toggle_plot_visibility(show: object, i: object) -> None:
                self._is_plot_visible[i] = show
                if i < len(self._plot_frames):
                    self._plot_frames[i].visible = show

            legends = self.get_legends()
            self._color_widgets = []
            self._show_plot_cbs = []
            with ui.HStack(spacing=3):
                ui.Spacer()
                for i in range(len(self._x_data)):
                    label_frame_width = LABEL_HEIGHT * len(legends[i]) / 3 + 8
                    ui.Label(legends[i], width=label_frame_width, alignment=ui.Alignment.RIGHT)

                    color_widget = ui.ColorWidget(width=10)
                    color_widget.model.add_end_edit_fn(lambda *args, idx=i: on_color_widget_set_plot_color(idx))
                    self._color_widgets.append(color_widget)

                    color = list(self._convert_hex_to_rgb(self._data_colors[i]))
                    color.append(255)
                    color = np.array(color) / 255.0
                    for j, item in enumerate(color_widget.model.get_item_children()):
                        color_widget.model.get_item_value_model(item).set_value(color[j])

                    model = ui.SimpleBoolModel()
                    model.set_value(self._is_plot_visible[i])
                    show_plot_cb = ui.CheckBox(model=model)
                    self._show_plot_cbs.append(show_plot_cb)
                    model.add_value_changed_fn(
                        lambda model, idx=i: toggle_plot_visibility(model.get_value_as_bool(), idx)
                    )

        def set_x_axis_values(low: object, high: object) -> None:
            assert high >= low

            if high == low:
                high = low + 0.0001

                # Handle float overflow
                if high == low:
                    return

            num_fields = len(self._x_axis_float_fields)
            spacing = (high - low) / num_fields
            num_decimals = int(max(0, np.log10(5 / (high - low)) + 2))

            self._x_max_float_drag.step = (high - low) / 20
            self._x_min_float_drag.step = (high - low) / 20
            for i, float_field in enumerate(self._x_axis_float_fields):
                float_field.model.set_value(np.round(low + spacing / 2 + spacing * i, decimals=num_decimals))

        def set_y_axis_values(low: object, high: object) -> None:
            assert high >= low

            if high == low:
                high = low + 0.0001

                # Handle float overflow
                if high == low:
                    return

            num_fields = len(self._y_axis_float_fields)
            spacing = (high - low) / num_fields
            num_decimals = int(max(0, np.log10(5 / (high - low)) + 2))
            self._y_min_float_drag.step = (high - low) / 20
            self._y_max_float_drag.step = (high - low) / 20
            for i, float_field in enumerate(self._y_axis_float_fields):
                float_field.model.set_value(np.round(high - spacing / 2 - spacing * i, decimals=num_decimals))

        def build_x_axis_frame() -> None:
            self._x_axis_float_fields = []

            def update_x_min(model: object) -> None:
                self._x_min = model.as_float
                if self._x_min >= self._x_max:
                    self._x_max = self._x_min + 1.0
                    self._x_max_float_drag.model.set_value(self._x_max)
                self._x_fracs, self._interpolated_y_data, _, _ = self._get_interpolated_data(
                    x_min=self._x_min, x_max=self._x_max
                )

                for i, plot_frame in enumerate(self._plot_frames):
                    plot_frame.set_build_fn(
                        build_fn=lambda x_fracs=self._x_fracs[i], y_data=self._interpolated_y_data[
                            i
                        ], color_idx=i: build_plot(x_fracs, y_data, color_idx)
                    )

                set_x_axis_values(self._x_min, self._x_max)

            def update_x_max(model: object) -> None:
                self._x_max = model.as_float
                if self._x_max <= self._x_min:
                    self._x_min = self._x_max - 1.0
                    self._x_min_float_drag.model.set_value(self._x_min)
                self._x_fracs, self._interpolated_y_data, _, _ = self._get_interpolated_data(
                    x_min=self._x_min, x_max=self._x_max
                )

                for i, plot_frame in enumerate(self._plot_frames):
                    plot_frame.set_build_fn(
                        build_fn=lambda x_fracs=self._x_fracs[i], y_data=self._interpolated_y_data[
                            i
                        ], color_idx=i: build_plot(x_fracs, y_data, color_idx)
                    )

                set_x_axis_values(self._x_min, self._x_max)

            with ui.VStack(spacing=0):
                with ui.ZStack():
                    ui.FloatDrag(
                        enabled=False
                    )  # This fills in the whole x axis label area with the right color.  The number will get covered up.

                    with ui.HStack():
                        ui.Spacer(
                            width=6
                        )  # There is blank space between the plotted line and the sides of the plot's colored rectangle
                        for i in range(plot_num_lines):
                            float_field = ui.FloatDrag(enabled=False, alignment=ui.Alignment.CENTER)
                            self._x_axis_float_fields.append(float_field)
                        ui.Spacer(width=6)

                ui.Spacer(height=3)

                with ui.HStack():
                    # Add fields for controlling min and max x values on plot
                    self._x_min_float_drag = ui.FloatDrag(
                        name="Field", alignment=ui.Alignment.LEFT_TOP, tooltip="X Min"
                    )
                    x_min_model = self._x_min_float_drag.model
                    x_min_model.set_value(self._x_min)

                    ui.Label(x_label, alignment=ui.Alignment.CENTER_TOP, height=LABEL_HEIGHT)

                    self._x_max_float_drag = ui.FloatDrag(
                        name="Field", alignment=ui.Alignment.LEFT_BOTTOM, tooltip="X Max"
                    )
                    x_max_model = self._x_max_float_drag.model
                    x_max_model.set_value(self._x_max)

                    x_min_model.add_value_changed_fn(update_x_min)
                    x_max_model.add_value_changed_fn(update_x_max)

            set_x_axis_values(x_min, x_max)

        def build_y_axis_frame() -> None:
            # Add fields for controlling min and max y values on plot
            def update_y_min(model: object) -> None:
                self._y_min = model.as_float

                if self._y_min >= self._y_max:
                    self._y_max = self._y_min + 1.0
                    self._y_max_float_drag.model.set_value(self._y_max)

                for plot in self._plots:
                    plot.scale_min = model.as_float

                set_y_axis_values(self._y_min, self._y_max)

            def update_y_max(model: object) -> None:
                self._y_max = model.as_float
                if self._y_max <= self._y_min:
                    self._y_min = self._y_max - 1.0
                    self._y_min_float_drag.model.set_value(self._y_min)

                for plot in self._plots:
                    plot.scale_max = model.as_float

                set_y_axis_values(self._y_min, self._y_max)

            y_max = self.get_y_max()
            y_min = self.get_y_min()

            if y_min is None:
                y_min = 0.0
            self._y_min = y_min

            if y_max is None:
                y_max = y_min + 1.0
            self._y_max = y_max

            with ui.VStack():
                with ui.HStack(spacing=2):
                    # Make it wide enough to contain the text on one line with a minimum size of 60 pixels
                    label_frame_width = max(LABEL_HEIGHT * len(y_label) / 3 + 3, 60)

                    with ui.Frame(width=label_frame_width, height=plot_height):

                        # Make a frame with y axis max, min and label that has width label_frame_width
                        with ui.VStack():
                            self._y_max_float_drag = ui.FloatDrag(
                                name="Field",
                                width=label_frame_width,
                                height=LABEL_HEIGHT,
                                alignment=ui.Alignment.CENTER,
                                tooltip="Y Max",
                            )
                            y_max_model = self._y_max_float_drag.model

                            y_max_model.set_value(y_max)

                            with ui.HStack():
                                ui.Spacer()
                                ui.Label(y_label, word_wrap=True, alignment=ui.Alignment.LEFT_CENTER)
                                ui.Spacer()

                            self._y_min_float_drag = ui.FloatDrag(
                                name="Field",
                                width=label_frame_width,
                                height=LABEL_HEIGHT,
                                alignment=ui.Alignment.CENTER,
                                tooltip="Y Min",
                            )
                            y_min_model = self._y_min_float_drag.model
                            y_min_model.set_value(y_min)

                            y_min_model.add_value_changed_fn(update_y_min)
                            y_max_model.add_value_changed_fn(update_y_max)

                    # Make axis markers
                    self._y_axis_float_fields = []
                    with ui.Frame(height=plot_height):
                        with ui.ZStack():
                            ui.FloatDrag(
                                enabled=False
                            )  # This fills in the whole y axis label area with the right color.  The number will get covered up.

                            with ui.VStack(spacing=0):
                                ui.Spacer(
                                    height=3
                                )  # There is blank space between the plotted line and the edges of the plot's colored rectangle
                                for i in range(plot_num_lines):
                                    float_field = ui.FloatDrag(enabled=False, alignment=ui.Alignment.CENTER)
                                    self._y_axis_float_fields.append(float_field)
                                ui.Spacer(height=3)

                # Fill "Show Legend" Frame
                with ui.HStack():
                    ui.Label(" Show\nLegend")
                    model = ui.SimpleBoolModel()
                    model.set_value(self._show_legend)
                    ui.Spacer(width=3)

                    with ui.VStack():
                        ui.Spacer()
                        self._show_legend_cb = ui.CheckBox(model=model, alignment=ui.Alignment.CENTER)
                        ui.Spacer()

                    ui.Spacer()
                    model.add_value_changed_fn(on_show_legend)

            set_y_axis_values(y_min, y_max)

        def build_plot(x_fracs: object, y_data: object, color_idx: int) -> None:
            """Build the frame for a plot.

            Args:
                x_fracs: Fraction of available space that the plot should cover
                y_data: data with which to fill the plot
                color_idx: Index of the color to use for this plot
            """
            color = self._data_colors[color_idx]
            with ui.HStack():
                # ui.Frame is the only omni.ui object that seems to consistently obey the given spacing rules
                # So each plot is on a Frame between two other invisible frames to get the placement right
                w = self._base_plot.computed_width
                if w == 0:
                    return

                # Plots have 6 pixel margins on them, so the fraction of the figure occupied needs to be modified
                x_low = max(0.0, x_fracs[0] - 6 / w)
                x_high = min(1.0, x_fracs[1] + 6 / w)
                y_min = self.get_y_min()
                y_max = self.get_y_max()
                if y_min is None:
                    y_min = 0.0
                if y_max is None:
                    y_max = 1.0

                ui.Frame(width=ui.Fraction(x_low))
                with ui.Frame(width=ui.Fraction(x_high - x_low)):
                    plot = ui.Plot(
                        ui.Type.LINE,
                        y_min,
                        y_max,
                        *y_data,
                        height=plot_height,
                        style={"color": color, "background_color": 0x0},
                    )
                    self._plots.append(plot)
                ui.Frame(width=ui.Fraction(1 - x_high))

        with ui.VStack(spacing=3):
            with ui.HStack():
                # Write the leftmost label for what this plot is
                ui.Label(label, width=LABEL_WIDTH, alignment=ui.Alignment.LEFT_TOP, tooltip=format_tt(tooltip))

                with ui.HStack(spacing=0):
                    # Make the y axis label
                    ui.Frame(build_fn=build_y_axis_frame, width=ui.Fraction(0.25))

                    # VStack the plot on top of the X axis label
                    with ui.VStack(spacing=0):

                        # ZStacking everything that goes on the actual data plot
                        with ui.ZStack():
                            ui.Rectangle(height=plot_height)
                            self._base_plot = ui.Plot(ui.Type.LINE, 0.0, 1.0, 0.0, style={"background_color": 0x0})
                            for i in range(len(y_data)):
                                plot_frame = Frame(
                                    build_fn=lambda x_fracs=x_fracs[i], y_data=y_data[i], color_idx=i: build_plot(
                                        x_fracs, y_data, color_idx
                                    )
                                )
                                plot_frame.visible = (
                                    self._is_plot_visible[i] if i < len(self._is_plot_visible) else False
                                )
                                self._plot_frames.append(plot_frame)

                            # Create an invisible frame on top that will give a helpful tooltip
                            self._tooltip_frame = ui.Plot(
                                mouse_moved_fn=self._mouse_moved_on_plot,
                                style={"color": 0xFFFFFFFF, "background_color": 0x0},
                            )
                            self._tooltip_frame.set_mouse_pressed_fn(self._mouse_moved_on_plot)
                            self._tooltip_frame.set_mouse_released_fn(self._on_mouse_released)
                            self._tooltip_frame.set_computed_content_size_changed_fn(
                                lambda *args: [plot.rebuild() for plot in self._plot_frames]
                            )
                            # Make the tooltip invisible
                            self._on_mouse_released()

                        # Create the x axis label
                        x_axis_frame = Frame(build_fn=build_x_axis_frame).frame
                        x_axis_frame.rebuild()

                ui.Spacer(width=20)

            with ui.HStack():
                ui.Spacer(width=LABEL_WIDTH)
                self._legend_frame = ui.Frame(build_fn=build_legend_frame)
                self._legend_frame.rebuild()
                self._legend_frame.visible = self._show_legend
                ui.Spacer(width=20)

            add_separator()

        self._has_built = True
