# SPDX-FileCopyrightText: Copyright (c) 2021-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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

"""Provides utility functions for creating stylized UI components and widgets for Isaac Sim extensions."""

from __future__ import annotations

import os
import sys
from cmath import inf
from collections.abc import Callable
from typing import TYPE_CHECKING

import carb.settings
import omni.appwindow
import omni.ext
import omni.ui as ui
from omni.kit.window.extensions import SimpleCheckBox
from omni.kit.window.filepicker import FilePickerDialog

if TYPE_CHECKING:
    from omni.kit.window.extensions.ext_components import SearchWidget

from omni.kit.window.property.templates import LABEL_HEIGHT, LABEL_WIDTH

from .callbacks import on_copy_to_clipboard, on_docs_link_clicked, on_open_folder_clicked, on_open_IDE_clicked
from .style import BUTTON_WIDTH, COLOR_W, COLOR_X, COLOR_Y, COLOR_Z, get_folder_picker_icon_button_style, get_style


def btn_builder(
    label: str = "", type: str = "button", text: str = "button", tooltip: str = "", on_clicked_fn: object = None
) -> ui.Button:
    """Creates a stylized button.

    Args:
        label: Label to the left of the UI element.
        type: Type of UI element.
        text: Text rendered on the button.
        tooltip: Tooltip to display over the Label.
        on_clicked_fn: Call-back function when clicked.

    Returns:
        Button
    """
    with ui.HStack():
        ui.Label(label, width=LABEL_WIDTH, alignment=ui.Alignment.LEFT_CENTER, tooltip=format_tt(tooltip))
        btn = ui.Button(
            text.upper(),
            name="Button",
            width=BUTTON_WIDTH,
            clicked_fn=on_clicked_fn,
            style=get_style(),
            tooltip=format_tt(tooltip),
            alignment=ui.Alignment.LEFT_CENTER,
        )
        ui.Spacer(width=5)
        add_line_rect_flourish(True)
        # ui.Spacer(width=ui.Fraction(1))
        # ui.Spacer(width=10)
        # with ui.Frame(width=0):
        #     with ui.VStack():
        #         with ui.Placer(offset_x=0, offset_y=7):
        #             ui.Rectangle(height=5, width=5, alignment=ui.Alignment.CENTER)
        # ui.Spacer(width=5)
    return btn


def state_btn_builder(
    label: str = "",
    type: str = "state_button",
    a_text: str = "STATE A",
    b_text: str = "STATE B",
    tooltip: str = "",
    on_clicked_fn: object = None,
) -> ui.Button:
    """Creates a State Change Button that changes text when pressed.

    Args:
        label: Label to the left of the UI element.
        type: Type of UI element.
        a_text: Text rendered on the button for State A.
        b_text: Text rendered on the button for State B.
        tooltip: Tooltip to display over the Label.
        on_clicked_fn: Call-back function when clicked.

    Returns:
        The created button widget.
    """

    def toggle() -> None:
        if btn.text == a_text.upper():
            btn.text = b_text.upper()
            on_clicked_fn(True)
        else:
            btn.text = a_text.upper()
            on_clicked_fn(False)

    with ui.HStack():
        ui.Label(label, width=LABEL_WIDTH, alignment=ui.Alignment.LEFT_CENTER, tooltip=format_tt(tooltip))
        btn = ui.Button(
            a_text.upper(),
            name="Button",
            width=BUTTON_WIDTH,
            clicked_fn=toggle,
            style=get_style(),
            alignment=ui.Alignment.LEFT_CENTER,
        )
        ui.Spacer(width=5)
        # add_line_rect_flourish(False)
        ui.Spacer(width=ui.Fraction(1))
        ui.Spacer(width=10)
        with ui.Frame(width=0):
            with ui.VStack():
                with ui.Placer(offset_x=0, offset_y=7):
                    ui.Rectangle(height=5, width=5, alignment=ui.Alignment.CENTER)
        ui.Spacer(width=5)
    return btn


def cb_builder(
    label: str = "", type: str = "checkbox", default_val: bool = False, tooltip: str = "", on_clicked_fn: object = None
) -> ui.SimpleBoolModel:
    """Creates a Stylized Checkbox.

    Args:
        label: Label to the left of the UI element.
        type: Type of UI element.
        default_val: Whether the checkbox is checked.
        tooltip: Tooltip to display over the Label.
        on_clicked_fn: Call-back function when clicked.

    Returns:
        model
    """
    with ui.HStack():
        ui.Label(label, width=LABEL_WIDTH - 12, alignment=ui.Alignment.LEFT_CENTER, tooltip=format_tt(tooltip))
        model = ui.SimpleBoolModel()
        callable = on_clicked_fn
        if callable is None:
            callable = lambda x: None
        SimpleCheckBox(default_val, callable, model=model)

        add_line_rect_flourish()
        return model


def multi_btn_builder(
    label: str = "",
    type: str = "multi_button",
    count: int = 2,
    text: list = None,
    tooltip: list = None,
    on_clicked_fn: list = None,
) -> list[ui.Button]:
    """Creates a Row of Stylized Buttons.

    Args:
        label: Label to the left of the UI element.
        type: Type of UI element.
        count: Number of UI elements to create.
        text: List of text rendered on the UI elements.
        tooltip: List of tooltips to display over the UI elements.
        on_clicked_fn: List of call-backs function when clicked.

    Returns:
        List of Buttons
    """
    if on_clicked_fn is None:
        on_clicked_fn = [None, None]
    if tooltip is None:
        tooltip = ["", "", ""]
    if text is None:
        text = ["button", "button"]
    btns = []
    with ui.HStack():
        ui.Label(label, width=LABEL_WIDTH, alignment=ui.Alignment.LEFT_CENTER, tooltip=format_tt(tooltip[0]))
        for i in range(count):
            btn = ui.Button(
                text[i].upper(),
                name="Button",
                width=BUTTON_WIDTH,
                clicked_fn=on_clicked_fn[i],
                tooltip=format_tt(tooltip[i + 1]),
                style=get_style(),
                alignment=ui.Alignment.LEFT_CENTER,
            )
            btns.append(btn)
            if i < count:
                ui.Spacer(width=5)
        add_line_rect_flourish()
    return btns


def multi_cb_builder(
    label: str = "",
    type: str = "multi_checkbox",
    count: int = 2,
    text: list = None,
    default_val: list = None,
    tooltip: list = None,
    on_clicked_fn: list = None,
) -> list[ui.SimpleBoolModel]:
    """Creates a Row of Stylized Checkboxes.

    Args:
        label: Label to the left of the UI element.
        type: Type of UI element.
        count: Number of UI elements to create.
        text: List of text rendered on the UI elements.
        default_val: List of default values. Checked is True, Unchecked is False.
        tooltip: List of tooltips to display over the UI elements.
        on_clicked_fn: List of call-backs function when clicked.

    Returns:
        List of models
    """
    if on_clicked_fn is None:
        on_clicked_fn = [None, None]
    if tooltip is None:
        tooltip = ["", "", ""]
    if default_val is None:
        default_val = [False, False]
    if text is None:
        text = [" ", " "]
    cbs = []
    with ui.HStack():
        ui.Label(label, width=LABEL_WIDTH - 12, alignment=ui.Alignment.LEFT_CENTER, tooltip=format_tt(tooltip[0]))
        for i in range(count):
            cb = ui.SimpleBoolModel(default_value=default_val[i])
            callable = on_clicked_fn[i]
            if callable is None:
                callable = lambda x: None
            SimpleCheckBox(default_val[i], callable, model=cb)
            ui.Label(
                text[i], width=BUTTON_WIDTH / 2, alignment=ui.Alignment.LEFT_CENTER, tooltip=format_tt(tooltip[i + 1])
            )
            if i < count - 1:
                ui.Spacer(width=5)
            cbs.append(cb)
        add_line_rect_flourish()
    return cbs


def str_builder(
    label: str = "",
    type: str = "stringfield",
    default_val: str = " ",
    tooltip: str = "",
    on_clicked_fn: object = None,
    use_folder_picker: bool = False,
    read_only: bool = False,
    item_filter_fn: object = None,
    bookmark_label: str | None = None,
    bookmark_path: str | None = None,
    folder_dialog_title: str = "Select Output Folder",
    folder_button_title: str = "Select Folder",
    identifier: str | None = None,
    label_width: int | None = None,
) -> ui.AbstractValueModel:
    """Creates a Stylized Stringfield Widget.

    Args:
        label: Label to the left of the UI element.
        type: Type of UI element.
        default_val: Text to initialize in Stringfield.
        tooltip: Tooltip to display over the UI elements.
        on_clicked_fn: Callback function when the field value changes.
        use_folder_picker: Add a folder picker button to the right.
        read_only: Prevents editing.
        item_filter_fn: filter function to pass to the FilePicker
        bookmark_label: bookmark label to pass to the FilePicker
        bookmark_path: bookmark path to pass to the FilePicker
        folder_dialog_title: Title for the folder picker dialog.
        folder_button_title: Title for the folder picker button.
        identifier: Optional identifier to simplify UI queries.
        label_width: Width of the label in pixels. Defaults to LABEL_WIDTH.

    Returns:
        model of Stringfield
    """
    with ui.HStack():
        lbl_width = ui.Pixel(label_width) if label_width is not None else LABEL_WIDTH
        ui.Label(label, width=lbl_width, alignment=ui.Alignment.LEFT_CENTER, tooltip=format_tt(tooltip))
        sf_kwargs = {}
        if identifier is not None:
            sf_kwargs["identifier"] = identifier
        str_field = ui.StringField(
            name="StringField",
            width=ui.Fraction(1),
            height=0,
            alignment=ui.Alignment.LEFT_CENTER,
            read_only=read_only,
            **sf_kwargs,
        ).model
        str_field.set_value(default_val)
        str_field.add_value_changed_fn(on_clicked_fn)

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
                str_field.set_value(val)

            add_folder_picker_icon(
                update_field,
                item_filter_fn,
                bookmark_label,
                bookmark_path,
                dialog_title=folder_dialog_title,
                button_title=folder_button_title,
            )
        else:
            add_line_rect_flourish(False)
        return str_field


def int_builder(
    label: str = "",
    type: str = "intfield",
    default_val: int = 0,
    tooltip: str = "",
    min: int = sys.maxsize * -1,
    max: int = sys.maxsize,
) -> ui.AbstractValueModel:
    """Creates a Stylized Intfield Widget.

    Args:
        label: Label to the left of the UI element.
        type: Type of UI element.
        default_val: Default Value of UI element.
        tooltip: Tooltip to display over the UI elements.
        min: Minimum limit for int field.
        max: Maximum limit for int field.

    Returns:
        AbstractValueModel: model
    """
    with ui.HStack():
        ui.Label(label, width=LABEL_WIDTH, alignment=ui.Alignment.LEFT_CENTER, tooltip=format_tt(tooltip))
        int_field = ui.IntDrag(
            name="Field", height=LABEL_HEIGHT, min=min, max=max, alignment=ui.Alignment.LEFT_CENTER
        ).model
        int_field.set_value(default_val)
        add_line_rect_flourish(False)
    return int_field


def float_builder(
    label: str = "",
    type: str = "floatfield",
    default_val: float = 0,
    tooltip: str = "",
    min: float = -inf,
    max: float = inf,
    step: float = 0.1,
    format: str = "%.2f",
) -> object:
    """Creates a Stylized Floatfield Widget.

    Args:
        label: Label to the left of the UI element.
        type: Type of UI element.
        default_val: Default Value of UI element.
        tooltip: Tooltip to display over the UI elements.
        min: Minimum Value.
        max: Maximum Value.
        step: Step size.
        format: Format string for display.

    Returns:
        AbstractValueModel: model
    """
    with ui.HStack():
        ui.Label(label, width=LABEL_WIDTH, alignment=ui.Alignment.LEFT_CENTER, tooltip=format_tt(tooltip))
        float_field = ui.FloatDrag(
            name="FloatField",
            width=ui.Fraction(1),
            height=0,
            alignment=ui.Alignment.LEFT_CENTER,
            min=min,
            max=max,
            step=step,
            format=format,
        ).model
        float_field.set_value(default_val)
        add_line_rect_flourish(False)
        return float_field


def combo_cb_str_builder(
    label: str = "",
    type: str = "checkbox_stringfield",
    default_val: list = None,
    tooltip: str = "",
    on_clicked_fn: object = lambda x: None,
    use_folder_picker: bool = False,
    read_only: bool = False,
    folder_dialog_title: str = "Select Output Folder",
    folder_button_title: str = "Select Folder",
) -> object:
    """Creates a Stylized Checkbox + Stringfield Widget.

    Args:
        label: Label to the left of the UI element.
        type: Type of UI element.
        default_val: Text to initialize in Stringfield.
        tooltip: Tooltip to display over the UI elements.
        on_clicked_fn: Call-back function when clicked.
        use_folder_picker: Add a folder picker button to the right.
        read_only: Prevents editing.
        folder_dialog_title: Dialog title for folder picker.
        folder_button_title: Button title for folder picker.

    Returns:
        (cb_model, str_field_model)
    """
    if default_val is None:
        default_val = [False, " "]
    with ui.HStack():
        ui.Label(label, width=LABEL_WIDTH - 12, alignment=ui.Alignment.LEFT_CENTER, tooltip=format_tt(tooltip))
        cb = ui.SimpleBoolModel(default_value=default_val[0])
        SimpleCheckBox(default_val[0], on_clicked_fn, model=cb)
        str_field = ui.StringField(
            name="StringField", width=ui.Fraction(1), height=0, alignment=ui.Alignment.LEFT_CENTER, read_only=read_only
        ).model
        str_field.set_value(default_val[1])

        if use_folder_picker:

            def update_field(val: object) -> None:
                str_field.set_value(val)

            add_folder_picker_icon(update_field, dialog_title=folder_dialog_title, button_title=folder_button_title)
        else:
            add_line_rect_flourish(False)
        return cb, str_field


def dropdown_builder(
    label: str = "",
    type: str = "dropdown",
    default_val: int = 0,
    items: list[str] | None = None,
    tooltip: str = "",
    on_clicked_fn: Callable | None = None,
    identifier: str | None = None,
    show_flourish: bool = True,
    label_width: int | None = None,
) -> ui.AbstractItemModel:
    """Creates a Stylized Dropdown Combobox.

    Args:
        label: Label to the left of the UI element.
        type: Type of UI element.
        default_val: Default index of dropdown items.
        items: List of items for dropdown box.
        tooltip: Tooltip to display over the Label.
        on_clicked_fn: Call-back function when clicked.
        identifier: Optional identifier to simplify UI queries.
        show_flourish: Whether to show the decorative rectangle flourish.
        label_width: Width of the label in pixels. Defaults to LABEL_WIDTH.

    Returns:
        AbstractItemModel: model
    """
    if items is None:
        items = ["Option 1", "Option 2", "Option 3"]
    with ui.HStack():
        lbl_width = ui.Pixel(label_width) if label_width is not None else LABEL_WIDTH
        ui.Label(label, width=lbl_width, alignment=ui.Alignment.LEFT_CENTER, tooltip=format_tt(tooltip))
        cb_kwargs = {}
        if identifier is not None:
            cb_kwargs["identifier"] = identifier
        combo_box = ui.ComboBox(
            default_val,
            *items,
            name="ComboBox",
            width=ui.Fraction(1),
            alignment=ui.Alignment.LEFT_CENTER,
            **cb_kwargs,
        ).model
        if show_flourish:
            add_line_rect_flourish(False)

        def on_clicked_wrapper(model: object, val: object) -> None:
            on_clicked_fn(items[model.get_item_value_model().as_int])

        if on_clicked_fn is not None:
            combo_box.add_item_changed_fn(on_clicked_wrapper)

    return combo_box


def checkbox_builder(
    label: str = "",
    type: str = "checkbox",
    default_val: bool = False,
    tooltip: str = "",
    on_clicked_fn: Callable | None = None,
    identifier: str | None = None,
) -> ui.SimpleBoolModel:
    """Creates a Stylized Checkbox using ui.CheckBox.

    Unlike :func:`cb_builder` which uses ``SimpleCheckBox``, this builder uses
    ``ui.CheckBox`` directly and supports an ``identifier`` for UI testing.

    Args:
        label: Label to the right of the checkbox.
        type: Type of UI element.
        default_val: Initial state of the checkbox.
        tooltip: Tooltip to display over the label.
        on_clicked_fn: Call-back function when clicked.
        identifier: Optional identifier to simplify UI queries.

    Returns:
        ui.SimpleBoolModel: Checkbox model.
    """
    with ui.HStack():
        chk_kwargs = {}
        if identifier is not None:
            chk_kwargs["identifier"] = identifier
        check_box = ui.CheckBox(width=10, height=0, **chk_kwargs)
        ui.Spacer(width=8)
        check_box.model.set_value(default_val)

        if on_clicked_fn is not None:

            def on_click(value_model: object, cb: object = on_clicked_fn) -> None:
                cb(value_model.get_value_as_bool())

            check_box.model.add_value_changed_fn(on_click)
        ui.Label(label, width=0, height=0, tooltip=tooltip)
        return check_box.model


def string_filed_builder(
    default_val: str = " ",
    tooltip: str = "",
    read_only: bool = False,
    item_filter_fn: Callable | None = None,
    folder_dialog_title: str = "Select Output Folder",
    folder_button_title: str = "Select Folder",
    bookmark_label: str | None = None,
    bookmark_path: str | None = None,
    use_folder_picker: bool = True,
    identifier: str | None = None,
) -> ui.AbstractValueModel:
    """Creates a Stylized String Field with an optional folder picker.

    Unlike :func:`str_builder`, this builder omits the label column and always
    opens the folder picker when the field is clicked.

    Args:
        default_val: Text to initialize in the string field.
        tooltip: Tooltip to display over the UI elements.
        read_only: Prevents editing.
        item_filter_fn: Filter function to pass to the FilePicker.
        folder_dialog_title: Title for the folder picker dialog.
        folder_button_title: Label for the folder picker button.
        bookmark_label: Bookmark label to pass to the FilePicker.
        bookmark_path: Bookmark path to pass to the FilePicker.
        use_folder_picker: Whether to show the folder picker button.
        identifier: Optional identifier to simplify UI queries.

    Returns:
        ui.AbstractValueModel: model of the string field.
    """
    with ui.HStack():
        sfb_kwargs = {}
        if identifier is not None:
            sfb_kwargs["identifier"] = identifier
        str_field = ui.StringField(
            name="StringField",
            tooltip=format_tt(tooltip),
            width=ui.Fraction(1),
            height=0,
            alignment=ui.Alignment.LEFT_CENTER,
            read_only=read_only,
            **sfb_kwargs,
        )
        str_field.enabled = False
        str_field.model.set_value(default_val)
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
                str_field.model.set_value(val)

            ui.Spacer(width=4)
            file_pick_fn = add_folder_picker_icon(
                update_field,
                item_filter_fn,
                bookmark_label=bookmark_label,
                bookmark_path=bookmark_path,
                dialog_title=folder_dialog_title,
                button_title=folder_button_title,
                size=16,
            )
            ui.Spacer(width=2)
            str_field.set_mouse_pressed_fn(lambda a, b, c, d: file_pick_fn())
        return str_field.model


def combo_intfield_slider_builder(
    label: str = "",
    type: str = "intfield_stringfield",
    default_val: float = 0.5,
    min: int = 0,
    max: int = 1,
    step: float = 0.01,
    tooltip: list = None,
) -> object:
    """Creates a Stylized IntField + Stringfield Widget.

    Args:
        label: Label to the left of the UI element.
        type: Type of UI element.
        default_val: Default Value.
        min: Minimum Value.
        max: Maximum Value.
        step: Step.
        tooltip: List of tooltips.

    Returns:
        Tuple(AbstractValueModel, IntSlider): (flt_field_model, flt_slider_model)
    """
    if tooltip is None:
        tooltip = ["", ""]
    with ui.HStack():
        ui.Label(label, width=LABEL_WIDTH, alignment=ui.Alignment.LEFT_CENTER, tooltip=format_tt(tooltip[0]))
        ff = ui.IntDrag(
            name="Field", width=BUTTON_WIDTH / 2, alignment=ui.Alignment.LEFT_CENTER, tooltip=format_tt(tooltip[1])
        ).model
        ff.set_value(default_val)
        ui.Spacer(width=5)
        fs = ui.IntSlider(
            width=ui.Fraction(1), alignment=ui.Alignment.LEFT_CENTER, min=min, max=max, step=step, model=ff
        )

        add_line_rect_flourish(False)
        return ff, fs


def combo_floatfield_slider_builder(
    label: str = "",
    type: str = "floatfield_stringfield",
    default_val: float = 0.5,
    min: int = 0,
    max: int = 1,
    step: float = 0.01,
    tooltip: list = None,
) -> object:
    """Creates a Stylized FloatField + FloatSlider Widget.

    Args:
        label: Label to the left of the UI element.
        type: Type of UI element.
        default_val: Default Value.
        min: Minimum Value.
        max: Maximum Value.
        step: Step.
        tooltip: List of tooltips.

    Returns:
        (flt_field_model, flt_slider_model)
    """
    if tooltip is None:
        tooltip = ["", ""]
    with ui.HStack():
        ui.Label(label, width=LABEL_WIDTH, alignment=ui.Alignment.LEFT_CENTER, tooltip=format_tt(tooltip[0]))
        ff = ui.FloatField(
            name="Field", width=BUTTON_WIDTH / 2, alignment=ui.Alignment.LEFT_CENTER, tooltip=format_tt(tooltip[1])
        ).model
        ff.set_value(default_val)
        ui.Spacer(width=5)
        fs = ui.FloatSlider(
            width=ui.Fraction(1), alignment=ui.Alignment.LEFT_CENTER, min=min, max=max, step=step, model=ff
        )

        add_line_rect_flourish(False)
        return ff, fs


def multi_dropdown_builder(
    label: str = "",
    type: str = "multi_dropdown",
    count: int = 2,
    default_val: list = None,
    items: list = None,
    tooltip: str = "",
    on_clicked_fn: list = None,
) -> object:
    """Creates a Stylized Multi-Dropdown Combobox.

    Args:
        label: Label to the left of the UI element.
        type: Type of UI element.
        count: Number of UI elements.
        default_val: List of default indices of dropdown items.
        items: List of list of items for dropdown boxes.
        tooltip: Tooltip to display over the Label.
        on_clicked_fn: List of call-back function when clicked.

    Returns:
        list(AbstractItemModel): list(models)
    """
    if on_clicked_fn is None:
        on_clicked_fn = [None, None]
    if items is None:
        items = [["Option 1", "Option 2", "Option 3"], ["Option A", "Option B", "Option C"]]
    if default_val is None:
        default_val = [0, 0]
    elems = []
    with ui.HStack():
        ui.Label(label, width=LABEL_WIDTH, alignment=ui.Alignment.LEFT_CENTER, tooltip=format_tt(tooltip))
        for i in range(count):
            elem = ui.ComboBox(
                default_val[i], *items[i], name="ComboBox", width=ui.Fraction(1), alignment=ui.Alignment.LEFT_CENTER
            )

            def on_clicked_wrapper(model: object, val: object, index: object) -> None:
                on_clicked_fn[index](items[index][model.get_item_value_model().as_int])

            elem.model.add_item_changed_fn(lambda m, v, index=i: on_clicked_wrapper(m, v, index))
            elems.append(elem)
            if i < count - 1:
                ui.Spacer(width=5)
        add_line_rect_flourish(False)
        return elems


def combo_cb_dropdown_builder(
    label: str = "",
    type: str = "checkbox_dropdown",
    default_val: list = None,
    items: list = None,
    tooltip: str = "",
    on_clicked_fn: list = None,
) -> object:
    """Creates a Stylized Dropdown Combobox with an Enable Checkbox.

    Args:
        label: Label to the left of the UI element.
        type: Type of UI element.
        default_val: list(cb_default, dropdown_default).
        items: List of items for dropdown box.
        tooltip: Tooltip to display over the Label.
        on_clicked_fn: List of callback functions.

    Returns:
        (cb_model, combobox)
    """
    if on_clicked_fn is None:
        on_clicked_fn = [lambda x: None, None]
    if items is None:
        items = ["Option 1", "Option 2", "Option 3"]
    if default_val is None:
        default_val = [False, 0]
    with ui.HStack():
        ui.Label(label, width=LABEL_WIDTH - 12, alignment=ui.Alignment.LEFT_CENTER, tooltip=format_tt(tooltip))
        cb = ui.SimpleBoolModel(default_value=default_val[0])
        SimpleCheckBox(default_val[0], on_clicked_fn[0], model=cb)
        combo_box = ui.ComboBox(
            default_val[1], *items, name="ComboBox", width=ui.Fraction(1), alignment=ui.Alignment.LEFT_CENTER
        )

        def on_clicked_wrapper(model: object, val: object) -> None:

            on_clicked_fn[1](items[model.get_item_value_model().as_int])

        combo_box.model.add_item_changed_fn(on_clicked_wrapper)

        add_line_rect_flourish(False)

        return cb, combo_box


def scrolling_frame_builder(
    label: str = "", type: str = "scrolling_frame", default_val: str = "No Data", tooltip: str = ""
) -> object:
    """Creates a Labeled Scrolling Frame with CopyToClipboard button.

    Args:
        label: Label to the left of the UI element.
        type: Type of UI element.
        default_val: Default Text.
        tooltip: Tooltip to display over the Label.

    Returns:
        ui.Label: label
    """
    with ui.VStack(style=get_style(), spacing=5):
        with ui.HStack():
            ui.Label(label, width=LABEL_WIDTH, alignment=ui.Alignment.LEFT_TOP, tooltip=format_tt(tooltip))
            with ui.ScrollingFrame(
                height=LABEL_HEIGHT * 5,
                style_type_name_override="ScrollingFrame",
                alignment=ui.Alignment.LEFT_TOP,
                horizontal_scrollbar_policy=ui.ScrollBarPolicy.SCROLLBAR_AS_NEEDED,
                vertical_scrollbar_policy=ui.ScrollBarPolicy.SCROLLBAR_ALWAYS_ON,
            ):
                text = ui.Label(
                    default_val,
                    style_type_name_override="Label::label",
                    word_wrap=True,
                    alignment=ui.Alignment.LEFT_TOP,
                )
            with ui.Frame(width=0, tooltip="Copy To Clipboard"):
                ui.Button(
                    name="IconButton",
                    width=20,
                    height=20,
                    clicked_fn=lambda: on_copy_to_clipboard(to_copy=text.text),
                    style=get_style()["IconButton.Image::CopyToClipboard"],
                    alignment=ui.Alignment.RIGHT_TOP,
                )
    return text


def combo_cb_scrolling_frame_builder(
    label: str = "",
    type: str = "cb_scrolling_frame",
    default_val: list = None,
    tooltip: str = "",
    on_clicked_fn: object = lambda x: None,
) -> object:
    """Creates a Labeled, Checkbox-enabled Scrolling Frame with CopyToClipboard button.

    Args:
        label: Label to the left of the UI element.
        type: Type of UI element.
        default_val: List of Checkbox and Frame Defaults.
        tooltip: Tooltip to display over the Label.
        on_clicked_fn: Callback function when clicked.

    Returns:
        (model, label)
    """
    if default_val is None:
        default_val = [False, "No Data"]
    with ui.VStack(style=get_style(), spacing=5):
        with ui.HStack():
            ui.Label(label, width=LABEL_WIDTH - 12, alignment=ui.Alignment.LEFT_TOP, tooltip=format_tt(tooltip))
            with ui.VStack(width=0):
                cb = ui.SimpleBoolModel(default_value=default_val[0])
                SimpleCheckBox(default_val[0], on_clicked_fn, model=cb)
                ui.Spacer(height=18 * 4)
            with ui.ScrollingFrame(
                height=18 * 5,
                style_type_name_override="ScrollingFrame",
                alignment=ui.Alignment.LEFT_TOP,
                horizontal_scrollbar_policy=ui.ScrollBarPolicy.SCROLLBAR_AS_NEEDED,
                vertical_scrollbar_policy=ui.ScrollBarPolicy.SCROLLBAR_ALWAYS_ON,
            ):
                text = ui.Label(
                    default_val[1],
                    style_type_name_override="Label::label",
                    word_wrap=True,
                    alignment=ui.Alignment.LEFT_TOP,
                )

            with ui.Frame(width=0, tooltip="Copy to Clipboard"):
                ui.Button(
                    name="IconButton",
                    width=20,
                    height=20,
                    clicked_fn=lambda: on_copy_to_clipboard(to_copy=text.text),
                    style=get_style()["IconButton.Image::CopyToClipboard"],
                    alignment=ui.Alignment.RIGHT_TOP,
                )
    return cb, text


def xyz_builder(
    label: str = "",
    tooltip: str = "",
    axis_count: int = 3,
    default_val: list[float] = None,
    min: float = float("-inf"),
    max: float = float("inf"),
    step: float = 0.001,
    on_value_changed_fn: list = None,
) -> list:
    """Create a multi-axis float drag widget with X, Y, Z, W labels.

    Args:
        label: Label to the left of the UI element.
        tooltip: Tooltip text for the widget.
        axis_count: Number of axes to display (1-4).
        default_val: List of default values.
        min: Minimum float value.
        max: Maximum float value.
        step: Drag step size.
        on_value_changed_fn: List of callback functions for each axis.

    Returns:
        List of value models for each axis.
    """
    # These styles & colors are taken from omni.kit.property.transform_builder.py _create_multi_float_drag_matrix_with_labels
    if on_value_changed_fn is None:
        on_value_changed_fn = [None, None, None, None]
    if default_val is None:
        default_val = [0.0, 0.0, 0.0, 0.0]
    if axis_count <= 0 or axis_count > 4:
        import builtins

        carb.log_warn("Invalid axis_count: must be in range 1 to 4. Clamping to default range.")
        axis_count = builtins.max(builtins.min(axis_count, 4), 1)

    field_labels = [("X", COLOR_X), ("Y", COLOR_Y), ("Z", COLOR_Z), ("W", COLOR_W)]
    field_tooltips = ["X Value", "Y Value", "Z Value", "W Value"]
    RECT_WIDTH = 13
    # SPACING = 4
    val_models = [None] * axis_count
    with ui.HStack():
        ui.Label(label, width=LABEL_WIDTH, alignment=ui.Alignment.LEFT_CENTER, tooltip=format_tt(tooltip))
        with ui.ZStack():
            with ui.HStack():
                ui.Spacer(width=RECT_WIDTH)
                for i in range(axis_count):
                    val_models[i] = ui.FloatDrag(
                        name="Field",
                        height=LABEL_HEIGHT,
                        min=min,
                        max=max,
                        step=step,
                        alignment=ui.Alignment.LEFT_CENTER,
                        tooltip=field_tooltips[i],
                    ).model
                    val_models[i].set_value(default_val[i])
                    if on_value_changed_fn[i] is not None:
                        val_models[i].add_value_changed_fn(on_value_changed_fn[i])
                    if i != axis_count - 1:
                        ui.Spacer(width=19)
            with ui.HStack():
                for i in range(axis_count):
                    if i != 0:
                        ui.Spacer()  # width=BUTTON_WIDTH - 1)
                    field_label = field_labels[i]
                    with ui.ZStack(width=RECT_WIDTH + 2 * i):
                        ui.Rectangle(name="vector_label", style={"background_color": field_label[1]})
                        ui.Label(field_label[0], name="vector_label", alignment=ui.Alignment.CENTER)
                ui.Spacer()
        add_line_rect_flourish(False)
        return val_models


def color_picker_builder(
    label: str = "", type: str = "color_picker", default_val: list = None, tooltip: str = "Color Picker"
) -> object:
    """Creates a Color Picker Widget.

    Args:
        label: Label to the left of the UI element.
        type: Type of UI element.
        default_val: List of (R,G,B,A) default values.
        tooltip: Tooltip to display over the Label.

    Returns:
        ui.ColorWidget.model
    """
    if default_val is None:
        default_val = [1.0, 1.0, 1.0, 1.0]
    with ui.HStack():
        ui.Label(label, width=LABEL_WIDTH, alignment=ui.Alignment.LEFT_CENTER, tooltip=format_tt(tooltip))
        model = ui.ColorWidget(*default_val, width=BUTTON_WIDTH).model
        ui.Spacer(width=5)
        add_line_rect_flourish()
    return model


def progress_bar_builder(
    label: str = "", type: str = "progress_bar", default_val: float = 0, tooltip: str = "Progress"
) -> object:
    """Creates a Progress Bar Widget.

    Args:
        label: Label to the left of the UI element.
        type: Type of UI element.
        default_val: Starting Value.
        tooltip: Tooltip to display over the Label.

    Returns:
        AbstractValueModel: ui.ProgressBar().model
    """
    with ui.HStack():
        ui.Label(label, width=LABEL_WIDTH, alignment=ui.Alignment.LEFT_CENTER)
        model = ui.ProgressBar().model
        model.set_value(default_val)
        add_line_rect_flourish(False)
    return model


def plot_builder(
    label: str = "",
    data: object = None,
    min: float = -1,
    max: float = 1,
    type: object = ui.Type.LINE,
    value_stride: int = 1,
    color: object = None,
    tooltip: str = "",
) -> object:
    """Creates a stylized static plot.

    Args:
        label: Label to the left of the UI element.
        data: Data to plot.
        min: Minimum Y Value.
        max: Maximum Y Value.
        type: Plot Type.
        value_stride: Width of plot stride.
        color: Plot color.
        tooltip: Tooltip to display over the Label.

    Returns:
        ui.Plot: plot
    """
    with ui.VStack(spacing=5):
        with ui.HStack():
            ui.Label(label, width=LABEL_WIDTH, alignment=ui.Alignment.LEFT_TOP, tooltip=format_tt(tooltip))

            plot_height = LABEL_HEIGHT * 2 + 13
            plot_width = ui.Fraction(1)
            with ui.ZStack():
                ui.Rectangle(width=plot_width, height=plot_height)
                if not color:
                    color = 0xFFDDDDDD
                plot = ui.Plot(
                    type,
                    min,
                    max,
                    *data,
                    value_stride=value_stride,
                    width=plot_width,
                    height=plot_height,
                    style={"color": color, "background_color": 0x0},
                )

            def update_min(model: object) -> None:
                plot.scale_min = model.as_float

            def update_max(model: object) -> None:
                plot.scale_max = model.as_float

            ui.Spacer(width=5)
            with ui.Frame(width=0):
                with ui.VStack(spacing=5):
                    max_model = ui.FloatDrag(
                        name="Field", width=40, alignment=ui.Alignment.LEFT_BOTTOM, tooltip="Max"
                    ).model
                    max_model.set_value(max)
                    min_model = ui.FloatDrag(
                        name="Field", width=40, alignment=ui.Alignment.LEFT_TOP, tooltip="Min"
                    ).model
                    min_model.set_value(min)

                    min_model.add_value_changed_fn(update_min)
                    max_model.add_value_changed_fn(update_max)

            ui.Spacer(width=20)
        add_separator()

    return plot


def xyz_plot_builder(
    label: str = "", data: list = None, min: int = -1, max: int = 1, tooltip: str = ""
) -> list[ui.Plot]:
    """Creates a stylized static XYZ plot.

    Args:
        label: Label to the left of the UI element.
        data: Data to plot.
        min: Minimum Y Value.
        max: Maximum Y Value.
        tooltip: Tooltip to display over the Label.

    Returns:
        list(x_plot, y_plot, z_plot)
    """
    if data is None:
        data = []
    with ui.VStack(spacing=5):
        with ui.HStack():
            ui.Label(label, width=LABEL_WIDTH, alignment=ui.Alignment.LEFT_TOP, tooltip=format_tt(tooltip))

            plot_height = LABEL_HEIGHT * 2 + 13
            plot_width = ui.Fraction(1)
            with ui.ZStack():
                ui.Rectangle(width=plot_width, height=plot_height)

                plot_0 = ui.Plot(
                    ui.Type.LINE,
                    min,
                    max,
                    *data[0],
                    width=plot_width,
                    height=plot_height,
                    style=get_style()["PlotLabel::X"],
                )
                plot_1 = ui.Plot(
                    ui.Type.LINE,
                    min,
                    max,
                    *data[1],
                    width=plot_width,
                    height=plot_height,
                    style=get_style()["PlotLabel::Y"],
                )
                plot_2 = ui.Plot(
                    ui.Type.LINE,
                    min,
                    max,
                    *data[2],
                    width=plot_width,
                    height=plot_height,
                    style=get_style()["PlotLabel::Z"],
                )

            def update_min(model: object) -> None:
                plot_0.scale_min = model.as_float
                plot_1.scale_min = model.as_float
                plot_2.scale_min = model.as_float

            def update_max(model: object) -> None:
                plot_0.scale_max = model.as_float
                plot_1.scale_max = model.as_float
                plot_2.scale_max = model.as_float

            ui.Spacer(width=5)
            with ui.Frame(width=0):
                with ui.VStack(spacing=5):
                    max_model = ui.FloatDrag(
                        name="Field", width=40, alignment=ui.Alignment.LEFT_BOTTOM, tooltip="Max"
                    ).model
                    max_model.set_value(max)
                    min_model = ui.FloatDrag(
                        name="Field", width=40, alignment=ui.Alignment.LEFT_TOP, tooltip="Min"
                    ).model
                    min_model.set_value(min)

                    min_model.add_value_changed_fn(update_min)
                    max_model.add_value_changed_fn(update_max)
            ui.Spacer(width=20)

        add_separator()
        return [plot_0, plot_1, plot_2]


def combo_cb_plot_builder(
    label: str = "",
    default_val: bool = False,
    on_clicked_fn: object = lambda x: None,
    data: object = None,
    min: int = -1,
    max: int = 1,
    type: object = ui.Type.LINE,
    value_stride: int = 1,
    color: object = None,
    tooltip: str = "",
) -> object:
    """Creates a Checkbox-Enabled dyanamic plot.

    Args:
        label: Label to the left of the UI element.
        default_val: Checkbox default.
        on_clicked_fn: Checkbox Callback function.
        data: Data to plat.
        min: Min Y Value.
        max: Max Y Value.
        type: Plot Type.
        value_stride: Width of plot stride.
        color: Plot color.
        tooltip: Tooltip to display over the Label.

    Returns:
        (cb_model, plot)
    """
    with ui.VStack(spacing=5):
        with ui.HStack():
            # Label
            ui.Label(label, width=LABEL_WIDTH, alignment=ui.Alignment.LEFT_TOP, tooltip=format_tt(tooltip))
            # Checkbox
            with ui.Frame(width=0):
                with ui.Placer(offset_x=-10, offset_y=0):
                    with ui.VStack():
                        SimpleCheckBox(default_val, on_clicked_fn)
                        ui.Spacer(height=ui.Fraction(1))
                        ui.Spacer()
            # Plot
            plot_height = LABEL_HEIGHT * 2 + 13
            plot_width = ui.Fraction(1)
            with ui.ZStack():
                ui.Rectangle(width=plot_width, height=plot_height)
                if not color:
                    color = 0xFFDDDDDD
                plot = ui.Plot(
                    type,
                    min,
                    max,
                    *data,
                    value_stride=value_stride,
                    width=plot_width,
                    height=plot_height,
                    style={"color": color, "background_color": 0x0},
                )

            # Min/Max Helpers
            def update_min(model: object) -> None:
                plot.scale_min = model.as_float

            def update_max(model: object) -> None:
                plot.scale_max = model.as_float

            ui.Spacer(width=5)
            with ui.Frame(width=0):
                with ui.VStack(spacing=5):
                    # Min/Max Fields
                    max_model = ui.FloatDrag(
                        name="Field", width=40, alignment=ui.Alignment.LEFT_BOTTOM, tooltip="Max"
                    ).model
                    max_model.set_value(max)
                    min_model = ui.FloatDrag(
                        name="Field", width=40, alignment=ui.Alignment.LEFT_TOP, tooltip="Min"
                    ).model
                    min_model.set_value(min)

                    min_model.add_value_changed_fn(update_min)
                    max_model.add_value_changed_fn(update_max)
            ui.Spacer(width=20)
        with ui.HStack():
            ui.Spacer(width=LABEL_WIDTH + 29)
            # Current Value Field (disabled by default)
            val_model = ui.FloatDrag(
                name="Field",
                width=BUTTON_WIDTH,
                height=LABEL_HEIGHT,
                enabled=False,
                alignment=ui.Alignment.LEFT_CENTER,
                tooltip="Value",
            ).model
        add_separator()
    return plot, val_model


def combo_cb_xyz_plot_builder(
    label: str = "",
    default_val: bool = False,
    on_clicked_fn: object = lambda x: None,
    data: list = None,
    min: int = -1,
    max: int = 1,
    type: object = ui.Type.LINE,
    value_stride: int = 1,
    tooltip: str = "",
) -> object:
    """Create a checkbox-enabled XYZ plot widget.

    Args:
        label: Label to the left of the UI element.
        default_val: Checkbox default state.
        on_clicked_fn: Checkbox callback function.
        data: Data to plot for each axis.
        min: Minimum Y value.
        max: Maximum Y value.
        type: Plot type.
        value_stride: Width of plot stride.
        tooltip: Tooltip to display over the Label.

    Returns:
        Tuple of plot list and value model list.
    """
    if data is None:
        data = []
    with ui.VStack(spacing=5):
        with ui.HStack():
            ui.Label(label, width=LABEL_WIDTH, alignment=ui.Alignment.LEFT_TOP, tooltip=format_tt(tooltip))
            # Checkbox
            with ui.Frame(width=0):
                with ui.Placer(offset_x=-10, offset_y=0):
                    with ui.VStack():
                        SimpleCheckBox(default_val, on_clicked_fn)
                        ui.Spacer(height=ui.Fraction(1))
                        ui.Spacer()
            # Plots
            plot_height = LABEL_HEIGHT * 2 + 13
            plot_width = ui.Fraction(1)
            with ui.ZStack():
                ui.Rectangle(width=plot_width, height=plot_height)

                plot_0 = ui.Plot(
                    type,
                    min,
                    max,
                    *data[0],
                    value_stride=value_stride,
                    width=plot_width,
                    height=plot_height,
                    style=get_style()["PlotLabel::X"],
                )
                plot_1 = ui.Plot(
                    type,
                    min,
                    max,
                    *data[1],
                    value_stride=value_stride,
                    width=plot_width,
                    height=plot_height,
                    style=get_style()["PlotLabel::Y"],
                )
                plot_2 = ui.Plot(
                    type,
                    min,
                    max,
                    *data[2],
                    value_stride=value_stride,
                    width=plot_width,
                    height=plot_height,
                    style=get_style()["PlotLabel::Z"],
                )

            def update_min(model: object) -> None:
                plot_0.scale_min = model.as_float
                plot_1.scale_min = model.as_float
                plot_2.scale_min = model.as_float

            def update_max(model: object) -> None:
                plot_0.scale_max = model.as_float
                plot_1.scale_max = model.as_float
                plot_2.scale_max = model.as_float

            ui.Spacer(width=5)
            with ui.Frame(width=0):
                with ui.VStack(spacing=5):
                    max_model = ui.FloatDrag(
                        name="Field", width=40, alignment=ui.Alignment.LEFT_BOTTOM, tooltip="Max"
                    ).model
                    max_model.set_value(max)
                    min_model = ui.FloatDrag(
                        name="Field", width=40, alignment=ui.Alignment.LEFT_TOP, tooltip="Min"
                    ).model
                    min_model.set_value(min)

                    min_model.add_value_changed_fn(update_min)
                    max_model.add_value_changed_fn(update_max)
            ui.Spacer(width=20)

        # with ui.HStack():
        #     ui.Spacer(width=40)
        #     val_models = xyz_builder()#**{"args":args})

        field_labels = [("X", COLOR_X), ("Y", COLOR_Y), ("Z", COLOR_Z), ("W", COLOR_W)]
        RECT_WIDTH = 13
        # SPACING = 4
        with ui.HStack():
            ui.Spacer(width=LABEL_WIDTH + 29)

            with ui.ZStack():
                with ui.HStack():
                    ui.Spacer(width=RECT_WIDTH)
                    # value_widget = ui.MultiFloatDragField(
                    #     *args, name="multivalue", min=min, max=max, step=step, h_spacing=RECT_WIDTH + SPACING, v_spacing=2
                    # ).model
                    val_model_x = ui.FloatDrag(
                        name="Field",
                        width=BUTTON_WIDTH - 5,
                        height=LABEL_HEIGHT,
                        enabled=False,
                        alignment=ui.Alignment.LEFT_CENTER,
                        tooltip="X Value",
                    ).model
                    ui.Spacer(width=19)
                    val_model_y = ui.FloatDrag(
                        name="Field",
                        width=BUTTON_WIDTH - 5,
                        height=LABEL_HEIGHT,
                        enabled=False,
                        alignment=ui.Alignment.LEFT_CENTER,
                        tooltip="Y Value",
                    ).model
                    ui.Spacer(width=19)
                    val_model_z = ui.FloatDrag(
                        name="Field",
                        width=BUTTON_WIDTH - 5,
                        height=LABEL_HEIGHT,
                        enabled=False,
                        alignment=ui.Alignment.LEFT_CENTER,
                        tooltip="Z Value",
                    ).model
                with ui.HStack():
                    for i in range(3):
                        if i != 0:
                            ui.Spacer(width=BUTTON_WIDTH - 1)
                        field_label = field_labels[i]
                        with ui.ZStack(width=RECT_WIDTH + 1):
                            ui.Rectangle(name="vector_label", style={"background_color": field_label[1]})
                            ui.Label(field_label[0], name="vector_label", alignment=ui.Alignment.CENTER)

        add_separator()
        return [plot_0, plot_1, plot_2], [val_model_x, val_model_y, val_model_z]


def add_line_rect_flourish(draw_line: bool = True) -> None:
    """Aesthetic element that adds a Line + Rectangle after all UI elements in the row.

    Args:
        draw_line: Set false to only draw rectangle.
    """
    if draw_line:
        ui.Line(style={"color": 0x338A8777}, width=ui.Fraction(1), alignment=ui.Alignment.CENTER)
    ui.Spacer(width=10)
    with ui.Frame(width=0):
        with ui.VStack():
            with ui.Placer(offset_x=0, offset_y=7):
                ui.Rectangle(height=5, width=5, alignment=ui.Alignment.CENTER)
    ui.Spacer(width=5)


def add_separator() -> None:
    """Aesthetic element to adds a Line Separator."""
    with ui.VStack(spacing=5):
        ui.Spacer()
        with ui.HStack():
            ui.Spacer(width=LABEL_WIDTH)
            ui.Line(style={"color": 0x338A8777}, width=ui.Fraction(1))
            ui.Spacer(width=20)
        ui.Spacer()


def add_folder_picker_icon(
    on_click_fn: object,
    item_filter_fn: object = None,
    bookmark_label: str | None = None,
    bookmark_path: str | None = None,
    dialog_title: str = "Select Output Folder",
    button_title: str = "Select Folder",
    size: int = 24,
) -> object:
    """Creates a folder picker icon button that opens a file dialog with advanced options.

    Args:
        on_click_fn: Callback function called when a folder is selected.
            Receives (filename, path) parameters.
        item_filter_fn: Filter function to pass to the FilePicker.
        bookmark_label: Bookmark label to pass to the FilePicker.
        bookmark_path: Bookmark path to pass to the FilePicker.
        dialog_title: Title for the folder picker dialog.
        button_title: Title for the folder picker button.
        size: Size of the icon button in pixels.

    Returns:
        Callable that opens the file picker when invoked.
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

    with ui.VStack(width=size, tooltip=button_title):
        ui.Spacer()
        ui.Button(
            name="IconButton",
            width=size,
            height=size,
            clicked_fn=open_file_picker,
            style=get_folder_picker_icon_button_style(),
            alignment=ui.Alignment.RIGHT_TOP,
        )
        ui.Spacer()

    return open_file_picker


def add_folder_picker_btn(on_click_fn: object) -> None:
    """Creates a folder picker button that opens a file dialog.

    Args:
        on_click_fn: Callback function called when a folder is selected.
            Receives (filename, path) parameters.
    """

    def open_folder_picker() -> None:
        def on_selected(a: object, b: object) -> None:
            on_click_fn(a, b)
            folder_picker.hide()

        def on_canceled(a: object, b: object) -> None:
            folder_picker.hide()

        folder_picker = FilePickerDialog(
            "Select Output Folder",
            allow_multi_selection=False,
            apply_button_label="Select Folder",
            click_apply_handler=lambda a, b: on_selected(a, b),
            click_cancel_handler=lambda a, b: on_canceled(a, b),
        )

    with ui.Frame(width=0):
        ui.Button("SELECT", width=BUTTON_WIDTH, clicked_fn=open_folder_picker, tooltip="Select Folder")


def format_tt(tt: str) -> object:
    """Format tooltip text by capitalizing words appropriately.

    Converts all-uppercase words to title case, capitalizes longer words and the first word,
    and keeps shorter words lowercase.

    Args:
        tt: The tooltip text to format.

    Returns:
        The formatted tooltip text.
    """
    import string

    formated = ""
    i = 0
    for w in tt.split():
        if w.isupper():
            formated += w + " "
        elif len(w) > 3 or i == 0:
            formated += string.capwords(w) + " "
        else:
            formated += w.lower() + " "
        i += 1
    return formated


def setup_ui_headers(
    ext_id: str,
    file_path: str,
    title: str = "My Custom Extension",
    doc_link: str = "https://docs.isaacsim.omniverse.nvidia.com/latest/index.html",
    overview: str = "",
    info_collapsed: bool = True,
) -> None:
    """Creates the Standard UI Elements at the top of each Isaac Extension.

    Args:
        ext_id: Extension ID.
        file_path: File path to source code.
        title: Name of Extension.
        doc_link: Hyperlink to Documentation.
        overview: Overview Text explaining the Extension.
        info_collapsed: Whether the info frame is collapsed.
    """
    ext_manager = omni.kit.app.get_app().get_extension_manager()
    extension_path = ext_manager.get_extension_path(ext_id)
    ext_path = os.path.dirname(extension_path) if os.path.isfile(extension_path) else extension_path
    build_header(ext_path, file_path, title, doc_link)
    build_info_frame(overview, info_collapsed)


def build_header(
    ext_path: str,
    file_path: str,
    title: str = "My Custom Extension",
    doc_link: str = "https://docs.isaacsim.omniverse.nvidia.com/latest/index.html",
) -> None:
    """Title Header with Quick Access Utility Buttons.

    Args:
        ext_path: Extension directory path.
        file_path: File path to source code.
        title: Name of Extension.
        doc_link: Hyperlink to Documentation.
    """

    def build_icon_bar() -> None:
        """Adds the Utility Buttons to the Title Header."""
        with ui.Frame(style=get_style(), width=0):
            with ui.VStack():
                with ui.HStack():
                    icon_size = 24
                    with ui.Frame(tooltip="Open Source Code"):
                        ui.Button(
                            name="IconButton",
                            width=icon_size,
                            height=icon_size,
                            clicked_fn=lambda: on_open_IDE_clicked(ext_path, file_path),
                            style=get_style()["IconButton.Image::OpenConfig"],
                            # style_type_name_override="IconButton.Image::OpenConfig",
                            alignment=ui.Alignment.LEFT_CENTER,
                            # tooltip="Open in IDE",
                        )
                    with ui.Frame(tooltip="Open Containing Folder"):
                        ui.Button(
                            name="IconButton",
                            width=icon_size,
                            height=icon_size,
                            clicked_fn=lambda: on_open_folder_clicked(file_path),
                            style=get_style()["IconButton.Image::OpenFolder"],
                            alignment=ui.Alignment.LEFT_CENTER,
                        )
                    with ui.Placer(offset_x=0, offset_y=3):
                        with ui.Frame(tooltip="Link to Docs"):
                            ui.Button(
                                name="IconButton",
                                width=icon_size - icon_size * 0.25,
                                height=icon_size - icon_size * 0.25,
                                clicked_fn=lambda: on_docs_link_clicked(doc_link),
                                style=get_style()["IconButton.Image::OpenLink"],
                                alignment=ui.Alignment.LEFT_TOP,
                            )

    with ui.ZStack():
        ui.Rectangle(style={"border_radius": 5, "background_color": 0xFF292929})
        with ui.HStack():
            ui.Spacer(width=5)
            ui.Label(title, width=0, name="title", style={"font_size": 16, "color": 0xFFC7C7C7})
            ui.Spacer(width=ui.Fraction(1))
            build_icon_bar()
            ui.Spacer(width=5)


def build_info_frame(overview: str = "", info_collapse: bool = True) -> None:
    """Info Frame with Overview, Instructions, and Metadata for an Extension.

    Args:
        overview: Overview text explaining the Extension.
        info_collapse: Whether the info frame is collapsed.
    """
    frame = ui.CollapsableFrame(
        title="Information",
        height=0,
        collapsed=info_collapse,
        horizontal_clipping=False,
        style=get_style(),
        style_type_name_override="CollapsableFrame",
        horizontal_scrollbar_policy=ui.ScrollBarPolicy.SCROLLBAR_AS_NEEDED,
        vertical_scrollbar_policy=ui.ScrollBarPolicy.SCROLLBAR_ALWAYS_ON,
    )
    with frame:
        label = "Overview"
        default_val = overview
        tooltip = "Overview"
        with ui.VStack(style=get_style(), spacing=5):
            with ui.HStack():
                ui.Label(label, width=LABEL_WIDTH / 2, alignment=ui.Alignment.LEFT_TOP, tooltip=format_tt(tooltip))
                with ui.ScrollingFrame(
                    height=LABEL_HEIGHT * 5,
                    style_type_name_override="ScrollingFrame",
                    alignment=ui.Alignment.LEFT_TOP,
                    horizontal_scrollbar_policy=ui.ScrollBarPolicy.SCROLLBAR_AS_NEEDED,
                    vertical_scrollbar_policy=ui.ScrollBarPolicy.SCROLLBAR_ALWAYS_ON,
                ):
                    text = ui.Label(
                        default_val,
                        style_type_name_override="Label::label",
                        word_wrap=True,
                        alignment=ui.Alignment.LEFT_TOP,
                    )
                with ui.Frame(width=0, tooltip="Copy To Clipboard"):
                    ui.Button(
                        name="IconButton",
                        width=20,
                        height=20,
                        clicked_fn=lambda: on_copy_to_clipboard(to_copy=text.text),
                        style=get_style()["IconButton.Image::CopyToClipboard"],
                        alignment=ui.Alignment.RIGHT_TOP,
                    )
    return


# def build_settings_frame(log_filename="extension.log", log_to_file=False, save_settings=False):
#     """Settings Frame for Common Utilities Functions"""
#     frame = ui.CollapsableFrame(
#         title="Settings",
#         height=0,
#         collapsed=True,
#         horizontal_clipping=False,
#         style=get_style(),
#         style_type_name_override="CollapsableFrame",
#     )

#     def on_log_to_file_enabled(val):
#         # TO DO
#         carb.log_info(f"Logging to {model.get_value_as_string()}:", val)

#     def on_save_out_settings(val):
#         # TO DO
#         carb.log_info("Save Out Settings?", val)


#     with frame:
#         with ui.VStack(style=get_style(), spacing=5):

#             # # Log to File Settings
#             # default_output_path = os.path.realpath(os.getcwd())
#             # kwargs = {
#             #     "label": "Log to File",
#             #     "type": "checkbox_stringfield",
#             #     "default_val": [log_to_file, default_output_path + "/" + log_filename],
#             #     "on_clicked_fn": on_log_to_file_enabled,
#             #     "tooltip": "Log Out to File",
#             #     "use_folder_picker": True,
#             # }
#             # model = combo_cb_str_builder(**kwargs)[1]

#             # Save Settings on Exit
#             # kwargs = {
#             #     "label": "Save Settings",
#             #     "type": "checkbox",
#             #     "default_val": save_settings,
#             #     "on_clicked_fn": on_save_out_settings,
#             #     "tooltip": "Save out GUI Settings on Exit.",
#             # }
#             # cb_builder(**kwargs)


class SearchListItem(ui.AbstractItem):
    """A search list item that represents a single entry in a searchable list.

    This class extends ui.AbstractItem to create individual items that can be displayed in search results.
    Each item contains a text string that can be searched and filtered.

    Args:
        text: The text content of the search list item.
    """

    def __init__(self, text: str) -> None:
        super().__init__()
        self.name_model = ui.SimpleStringModel(text)

    def __repr__(self) -> str:
        """String representation of the search list item.

        Returns:
            The item name enclosed in double quotes.
        """
        return f'"{self.name_model.as_string}"'

    def name(self) -> str:
        """Name of the search list item.

        Returns:
            The string value of the item name.
        """
        return self.name_model.as_string


def _normalize_search_filter_text(text: object) -> str:
    """Normalize search filter input to a text string.

    Args:
        text: Search text or a sequence of search terms.

    Returns:
        Text to pass through the search filter.
    """
    if text is None:
        return ""

    if isinstance(text, (list, tuple)):
        return " ".join(str(part) for part in text if part is not None)

    return str(text)


class SearchListItemModel(ui.AbstractItemModel):
    """Represents the model for lists. It's very easy to initialize it.

    with any string list:

    .. code-block:: python

        string_list = ["Hello", "World"]
        model = ListModel(*string_list)
        ui.TreeView(model)

    Args:
        *args: String items to populate the list model.
    """

    def __init__(self, *args: str) -> None:
        super().__init__()
        self._children = [SearchListItem(t) for t in args]
        self._filtered = [SearchListItem(t) for t in args]

    def get_item_children(self, item: object) -> object:
        """Returns all the children when the widget asks it.

        Args:
            item: The parent item to get children for. If None, returns root children.

        Returns:
            List of child items.
        """
        if item is not None:
            # Since we are doing a flat list, we return the children of root only.
            # If it's not root we return.
            return []

        return self._filtered

    def filter_text(self, text: object) -> None:
        """Filter the list items by the given text pattern.

        Args:
            text: Text pattern to filter items by.
        """
        import fnmatch

        filter_text = _normalize_search_filter_text(text)
        self._filtered = []
        if len(filter_text) == 0:
            for c in self._children:
                self._filtered.append(c)
        else:
            parts = filter_text.split()
            # for i in range(len(parts) - 1, -1, -1):
            #     w = parts[i]

            leftover = " ".join(parts)
            if len(leftover) > 0:
                filter_str = f"*{leftover.lower()}*"
                for c in self._children:
                    if fnmatch.fnmatch(c.name().lower(), filter_str):
                        self._filtered.append(c)

        # This tells the Delegate to update the TreeView
        self._item_changed(None)

    def get_item_value_model_count(self, item: object) -> int:
        """The number of columns.

        Args:
            item: The item to get the column count for.

        Returns:
            Number of columns.
        """
        return 1

    def get_item_value_model(self, item: object, column_id: int) -> object:
        """Return value model.

        It's the object that tracks the specific value.
        In our case we use ui.SimpleStringModel.

        Args:
            item: The item to get the value model for.
            column_id: The column identifier.

        Returns:
            The value model for the item.
        """
        return item.name_model


class SearchListItemDelegate(ui.AbstractItemDelegate):
    """Delegate is the representation layer. TreeView calls the methods.

    of the delegate to create custom widgets for each item.

    Args:
        on_double_click_fn: Callback function for double-click events.
    """

    def __init__(self, on_double_click_fn: object = None) -> None:
        super().__init__()
        self._on_double_click_fn = on_double_click_fn

    def build_branch(self, model: object, item: object, column_id: int, level: int, expanded: bool) -> None:
        """Create a branch widget that opens or closes subtree.

        Args:
            model: The tree model containing the data.
            item: The current item in the tree.
            column_id: The column identifier.
            level: The tree level depth.
            expanded: Whether the branch is currently expanded.
        """

    def build_widget(self, model: object, item: object, column_id: int, level: int, expanded: bool) -> None:
        """Create a widget per column per item.

        Args:
            model: The tree model containing the data.
            item: The current item in the tree.
            column_id: The column identifier.
            level: The tree level depth.
            expanded: Whether the branch is currently expanded.
        """
        stack = ui.ZStack(height=20, style=get_style())
        with stack:
            with ui.HStack():
                ui.Spacer(width=5)
                value_model = model.get_item_value_model(item, column_id)
                label = ui.Label(value_model.as_string, name="TreeView.Item")

        if not self._on_double_click_fn:
            self._on_double_click_fn = self.on_double_click

        # Set a double click function
        stack.set_mouse_double_clicked_fn(
            lambda x, y, b, m, item_label=label: self._on_double_click_fn(b, m, item_label)
        )

    def on_double_click(self, button: int, model: object, label: object) -> None:
        """Called when the user double-clicked the item in TreeView.

        Args:
            button: The mouse button that was clicked.
            model: The tree model.
            label: The UI label that was double-clicked.
        """
        if button != 0:
            return


def build_simple_search(
    label: str = "", type: str = "search", model: object = None, delegate: object = None, tooltip: str = ""
) -> tuple[SearchWidget, ui.TreeView]:
    """A Simple Search Bar + TreeView Widget.

    Pass a list of items through the model, and a custom on_click_fn through the delegate.

    Returns the SearchWidget so user can destroy it on_shutdown.

    Args:
        label: Label to the left of the UI element.
        type: Type of UI element.
        model: Item Model for Search.
        delegate: Item Delegate for Search.
        tooltip: Tooltip to display over the Label.

    Returns:
        Tuple(Search Widget, Treeview):
    """
    with ui.HStack():
        ui.Label(label, width=LABEL_WIDTH, alignment=ui.Alignment.LEFT_TOP, tooltip=format_tt(tooltip))

        with ui.VStack(spacing=5):

            def filter_text(item: object) -> None:
                model.filter_text(item)

            from omni.kit.window.extensions.ext_components import SearchWidget

            search_bar = SearchWidget(filter_text)

            with ui.ScrollingFrame(
                height=LABEL_HEIGHT * 5,
                horizontal_scrollbar_policy=ui.ScrollBarPolicy.SCROLLBAR_ALWAYS_OFF,
                vertical_scrollbar_policy=ui.ScrollBarPolicy.SCROLLBAR_ALWAYS_ON,
                style=get_style(),
                style_type_name_override="TreeView.ScrollingFrame",
            ):
                treeview = ui.TreeView(
                    model,
                    delegate=delegate,
                    root_visible=False,
                    header_visible=False,
                    style={
                        "TreeView.ScrollingFrame": {"background_color": 0xFFE0E0E0},
                        "TreeView.Item": {"color": 0xFF535354, "font_size": 16},
                        "TreeView.Item:selected": {"color": 0xFF23211F},
                        "TreeView:selected": {"background_color": 0x409D905C},
                    },
                    # name="TreeView",
                    # style_type_name_override="TreeView",
                )
        add_line_rect_flourish(False)
    return search_bar, treeview
