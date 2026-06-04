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

"""Provides UI styling and color definitions for the robot setup wizard interface components."""

import pathlib
from typing import Any

import omni.kit.app
import omni.ui as ui
from omni.ui import color as cl

EXTENSION_FOLDER_PATH = pathlib.Path(
    omni.kit.app.get_app().get_extension_manager().get_extension_path_by_module(__name__)
)

## colors
BUTTON_BG_COLOR = 0xFF24211F
BUTTON_DISABLED_BG_COLOR = 0xFF2F2E2C
FRAME_HEADER_COLOR = 0xFF2F2F2E
FRAME_BG_COLOR = 0xFF343433
HIGHLIGHT_COLOR = 0xFFB0703B
LABEL_COLOR = 0xFF9E9E9E
LABEL_DISABLED_COLOR = 0xFF545452
LABEL_TITLE_COLOR = 0xFFCCCCCC
LABEL_WARNING_COLOR = 0xFFFF8080
LABEL_ERROR_COLOR = 0xFF0000FF
LINE_COLOR = 0x16FFFFFF
STEP_GREEN_COLOR = 0xFF00B976
STEP_GREEN_STACK_COLOR = 0x2200B976
STEP_BLUE_COLOR = 0xFFE39724
STEP_BLUE_STACK_COLOR = 0x22E39724
STEP_GRAY_STACK_COLOR = 0xFF545455
STEP_ACTIVE_BORDER_COLOR = 0xFFFFC266
TRIANGLE_COLOR = 0xFFD9D9D9
TREEVIEW_BG_COLOR = 0xFF23211F
TREEVIEW_SELECTED_COLOR = 0xFF4B4A42
TREEVIEW_ITEM_COLOR = 0xFF343432
TREEVIEW_HEADER_BG_COLOR = 0xFF2D2D2D
WINDOW_BG_COLOR = 0xFF454545


def get_style() -> dict[str, dict[str, Any]]:
    """Main UI style configuration for the robot setup wizard.

    Returns:
        A comprehensive dictionary of UI style settings for all wizard components including buttons, labels, frames,
        trees, and other interface elements.
    """
    style = {
        "Button": {"stack_direction": ui.Direction.LEFT_TO_RIGHT},
        "Button:disabled": {"background_color": BUTTON_DISABLED_BG_COLOR},
        "Button.Rect": {"background_color": BUTTON_BG_COLOR},
        "Button.Rect:disabled": {"background_color": BUTTON_DISABLED_BG_COLOR},
        "Button.Image::next": {"image_url": f"{EXTENSION_FOLDER_PATH}/icons/next_icon.svg"},
        "Button.Image::next:disabled": {"image_url": f"{EXTENSION_FOLDER_PATH}/icons/next_icon_disable.svg"},
        "Button.Image::add": {"image_url": f"{EXTENSION_FOLDER_PATH}/icons/add.svg"},
        "Button.Image::browse": {"image_url": f"{EXTENSION_FOLDER_PATH}/icons/browse_dark.svg"},
        "Button.Image::save": {"image_url": f"{EXTENSION_FOLDER_PATH}/icons/save_dark.svg"},
        "Button.Image::link_xform": {"image_url": f"{EXTENSION_FOLDER_PATH}/icons/link_xform.svg"},
        "Button.Image::link_add": {"image_url": f"{EXTENSION_FOLDER_PATH}/icons/link_add.svg"},
        "Button.Image::link_delete": {"image_url": f"{EXTENSION_FOLDER_PATH}/icons/link_delete.svg"},
        "Button.Image::up": {"image_url": f"{EXTENSION_FOLDER_PATH}/icons/arrow_up.svg"},
        "Button.Image::down": {"image_url": f"{EXTENSION_FOLDER_PATH}/icons/arrow_down.svg"},
        "Button.Label": {
            "color": LABEL_COLOR,
            "font_size": 16,
            "font": f"{EXTENSION_FOLDER_PATH}/data/fonts/NVIDIASans_Md.ttf",
        },
        "Button.Label:disabled": {"color": LABEL_DISABLED_COLOR},
        "Button::visualize": {"background_color": BUTTON_BG_COLOR},
        "Button::visualize:hovered": {"background_color": BUTTON_DISABLED_BG_COLOR},
        "Button::link_xform": {"margin": 0, "border_radius": 0, "background_color": BUTTON_BG_COLOR},
        "Button::apply_new_structure": {"background_color": BUTTON_BG_COLOR},
        "Button::apply_new_structure:hovered": {"background_color": BUTTON_DISABLED_BG_COLOR},
        "Button::start_over": {"color": FRAME_HEADER_COLOR, "background_color": BUTTON_BG_COLOR},
        "Circle::dot": {"background_color": LABEL_COLOR},
        "Circle::step_index": {"background_color": LABEL_COLOR},
        "CheckBox": {"border_radius": 2},
        "CollapsableFrame": {"background_color": FRAME_BG_COLOR, "secondary_color": FRAME_HEADER_COLOR},
        "CollapsableFrame:hovered": {"background_color": FRAME_BG_COLOR, "secondary_color": FRAME_HEADER_COLOR},
        "ComboBox": {"background_color": BUTTON_BG_COLOR, "color": LABEL_COLOR},
        "ComboBox::treeview": {
            "color": LABEL_COLOR,
            "border_radius": 0,
            "font_size": 16,
            "font": f"{EXTENSION_FOLDER_PATH}/data/fonts/NVIDIASans_Rg.ttf",
            "background_color": TREEVIEW_ITEM_COLOR,
        },
        "ComboBox::treeview:selected": {
            "background_color": TREEVIEW_SELECTED_COLOR,
            "color": LABEL_COLOR,
            "border_radius": 0,
        },
        "Field": {
            "background_color": BUTTON_BG_COLOR,
            "color": LABEL_COLOR,
            "font_size": 16,
            "font": f"{EXTENSION_FOLDER_PATH}/data/fonts/NVIDIASans_It.ttf",
        },
        "Field:checked": {
            "background_color": BUTTON_BG_COLOR,
            "color": LABEL_COLOR,
            "font_size": 16,
            "font": f"{EXTENSION_FOLDER_PATH}/data/fonts/NVIDIASans_Md.ttf",
        },
        "Field::resetable": {
            "background_color": BUTTON_BG_COLOR,
            "color": LABEL_COLOR,
            "font_size": 16,
            "font": f"{EXTENSION_FOLDER_PATH}/data/fonts/NVIDIASans_Md.ttf",
        },
        "Field::resetable:disabled": {
            "color": LABEL_DISABLED_COLOR,
            "font_size": 16,
            "font": f"{EXTENSION_FOLDER_PATH}/data/fonts/NVIDIASans_Md.ttf",
        },
        "Image::add": {"image_url": f"{EXTENSION_FOLDER_PATH}/icons/add.svg"},
        "Image::add_environment": {"image_url": f"{EXTENSION_FOLDER_PATH}/icons/add_environment.svg"},
        "Image::add_robot": {"image_url": f"{EXTENSION_FOLDER_PATH}/icons/add_robot.png"},
        "Image::add_sensors": {"image_url": f"{EXTENSION_FOLDER_PATH}/icons/add_sensors.svg"},
        "Image::attach_robots": {"image_url": f"{EXTENSION_FOLDER_PATH}/icons/attach_robots.svg"},
        "Image::convert_to_rigid_body": {"image_url": f"{EXTENSION_FOLDER_PATH}/icons/convert_to_rigid_body.svg"},
        "Image::generic_tool": {"image_url": f"{EXTENSION_FOLDER_PATH}/icons/icoGeneric.svg"},
        "Image::define": {"image_url": f"{EXTENSION_FOLDER_PATH}/icons/prepare_files.png"},
        "Image::drag_and_drop": {
            "image_url": f"{EXTENSION_FOLDER_PATH}/icons/drag_and_drop.png",
            "margin_width": 10,
            "margin_height": 10,
        },
        "Image::folder": {"image_url": f"{EXTENSION_FOLDER_PATH}/icons/folder.svg", "margin": 4},
        "Image::go_to": {"image_url": f"{EXTENSION_FOLDER_PATH}/icons/go_to.svg", "margin": 4},
        "Image::help": {"image_url": f"{EXTENSION_FOLDER_PATH}/icons/help_inactive.svg"},
        "Image::help:hovered": {"image_url": f"{EXTENSION_FOLDER_PATH}/icons/help_active.svg"},
        "Image::help:pressed": {"image_url": f"{EXTENSION_FOLDER_PATH}/icons/help_active.svg"},
        "Image::import_robot": {"image_url": f"{EXTENSION_FOLDER_PATH}/icons/import_robot.png"},
        "Image::pencil": {"image_url": f"{EXTENSION_FOLDER_PATH}/icons/PencilIcon.svg"},
        "Image::remove": {"image_url": f"{EXTENSION_FOLDER_PATH}/icons/remove_active.svg"},
        "Image::remove:disabled": {"image_url": f"{EXTENSION_FOLDER_PATH}/icons/remove_active_inactive.svg"},
        "Image::remove_header": {"image_url": f"{EXTENSION_FOLDER_PATH}/icons/remove_header.svg"},
        "Image::sample": {"image_url": f"{EXTENSION_FOLDER_PATH}/icons/sample.svg", "margin": 4},
        "Image::sort": {"image_url": f"{EXTENSION_FOLDER_PATH}/icons/sort_icon.svg", "margin": 4},
        "Line": {"color": LINE_COLOR},
        "Label": {
            "color": LABEL_COLOR,
            "font_size": 16,
            "font": f"{EXTENSION_FOLDER_PATH}/data/fonts/NVIDIASans_Rg.ttf",
        },
        "Label::normal": {
            "color": LABEL_COLOR,
            "font_size": 14,
            "font": f"{EXTENSION_FOLDER_PATH}/data/fonts/NVIDIASans_Md.ttf",
        },
        "Label::choose_options": {
            "color": LABEL_TITLE_COLOR,
            "font_size": 16,
            "font": f"{EXTENSION_FOLDER_PATH}/data/fonts/NVIDIASans_Md.ttf",
        },
        "Label::empty_treeview_title": {
            "color": LABEL_COLOR,
            "font_size": 26,
            "font": f"{EXTENSION_FOLDER_PATH}/data/fonts/NVIDIASans_Md.ttf",
        },
        "Label::launch_on_startup": {"color": LABEL_TITLE_COLOR, "font_size": 16},
        "Label::property": {"font": f"{EXTENSION_FOLDER_PATH}/data/fonts/NVIDIASans_Md.ttf"},
        "Label::robot_picker": {
            "color": LABEL_COLOR,
            "font_size": 20,
            "font": f"{EXTENSION_FOLDER_PATH}/data/fonts/NVIDIASans_Md.ttf",
        },
        "Label::separator": {
            "color": LABEL_TITLE_COLOR,
            "font_size": 18,
            "font": f"{EXTENSION_FOLDER_PATH}/data/fonts/NVIDIASans_Md.ttf",
        },
        "Label::sub_separator": {
            "color": LABEL_COLOR,
            "font_size": 15,
            "font": f"{EXTENSION_FOLDER_PATH}/data/fonts/NVIDIASans_Lt.ttf",
        },
        "Label::step_index": {"color": cl.black, "font_size": 18},
        "Label::title": {
            "color": LABEL_TITLE_COLOR,
            "font_size": 32,
            "font": f"{EXTENSION_FOLDER_PATH}/data/fonts/NVIDIASans_Lt.ttf",
        },
        "Label::sub_title": {
            "color": LABEL_TITLE_COLOR,
            "font_size": 15,
            "font": f"{EXTENSION_FOLDER_PATH}/data/fonts/NVIDIASans_Md.ttf",
        },
        "Label::treeview_header": {
            "color": LABEL_COLOR,
            "font_size": 16,
            "font": f"{EXTENSION_FOLDER_PATH}/data/fonts/NVIDIASans_Md.ttf",
        },
        "Label::wizard_steps": {
            "color": LABEL_TITLE_COLOR,
            "font_size": 20,
            "font": f"{EXTENSION_FOLDER_PATH}/data/fonts/NVIDIASans_Bd.ttf",
        },
        "Label::degree": {
            "color": LABEL_COLOR,
            "font_size": 8,
            "font": f"{EXTENSION_FOLDER_PATH}/data/fonts/NVIDIASans_Md.ttf",
        },
        "Label::degree:disabled": {
            "color": LABEL_DISABLED_COLOR,
            "font_size": 8,
            "font": f"{EXTENSION_FOLDER_PATH}/data/fonts/NVIDIASans_Md.ttf",
        },
        "Label::resetable:disabled": {
            "color": LABEL_DISABLED_COLOR,
            "font_size": 16,
            "font": f"{EXTENSION_FOLDER_PATH}/data/fonts/NVIDIASans_Md.ttf",
        },
        "RadioButton": {"background_color": cl.transparent, "padding": 0},
        "RadioButton:checked": {"background_color": cl.transparent},
        "RadioButton:hovered": {"background_color": cl.transparent},
        "RadioButton.Image": {
            "image_url": f"{EXTENSION_FOLDER_PATH}/icons/radio_off.svg",
        },
        "RadioButton.Image:checked": {"image_url": f"{EXTENSION_FOLDER_PATH}/icons/radio_on.svg"},
        "RadioButton:pressed": {"background_color": cl.transparent},
        "Rectangle::d_area": {"background_color": 0xFF4A4A4A},
        "Rectangle::drag_area": {"background_color": TREEVIEW_BG_COLOR},
        "Rectangle::title_background": {"background_color": 0xFF2F2F2D},
        "Rectangle::splitter": {"background_color": WINDOW_BG_COLOR},
        "Rectangle::splitter_highlight": {"background_color": WINDOW_BG_COLOR},
        "Rectangle::splitter_highlight:hovered": {"background_color": HIGHLIGHT_COLOR},
        "Rectangle::splitter_highlight:pressed": {"background_color": HIGHLIGHT_COLOR},
        "Rectangle::save_stack": {"background_color": 0xFF2F2F2D, "border_radius": 6},
        "Rectangle::step": {"background_color": FRAME_HEADER_COLOR},
        "Rectangle::step_background": {"background_color": FRAME_BG_COLOR},
        "Rectangle::step_item_background": {"background_color": FRAME_BG_COLOR},
        "Rectangle::step_item_background:hovered": {"background_color": STEP_GRAY_STACK_COLOR},
        "Rectangle::treeview_background": {"background_color": TREEVIEW_BG_COLOR},
        "Rectangle::treeview": {"background_color": WINDOW_BG_COLOR},
        "Rectangle::treeview_item": {"margin": 1, "background_color": TREEVIEW_ITEM_COLOR},
        "Rectangle::treeview_item:selected": {"margin": 1, "background_color": TREEVIEW_SELECTED_COLOR},
        "Rectangle::treeview_item_button": {"margin": 1, "background_color": TREEVIEW_ITEM_COLOR},
        "Rectangle::reset_invalid": {"background_color": 0xFF505050, "border_radius": 2},
        "Rectangle::reset": {"background_color": 0xFFA07D4F, "border_radius": 2},
        "Rectangle::reset_mask": {"background_color": BUTTON_BG_COLOR},
        "ScrollingFrame": {"background_color": FRAME_BG_COLOR},
        "ScrollingFrame::treeview": {"background_color": TREEVIEW_BG_COLOR},
        "Triangle": {"background_color": LABEL_TITLE_COLOR},
        "TreeView": {"background_selected_color": TREEVIEW_BG_COLOR},  # the hover color of the TreeView selected item
        "TreeView.Header": {"background_color": TREEVIEW_BG_COLOR},
        "TreeView.Header::background": {"margin": 1, "background_color": TREEVIEW_HEADER_BG_COLOR},
        "TreeView:selected": {
            "background_color": TREEVIEW_BG_COLOR
        },  # selected margin color, set to scrollingFrame background color
        "VStack::margin_vstack": {"margin_width": 15, "margin_height": 10},
        "VStack::setting_margin_vstack": {"margin_width": 10},
        "VStack::setting_content_vstack": {"margin_width": 10, "margin_height": 10},
    }
    return style


def get_popup_window_style() -> dict[str, dict[str, Any]]:
    """UI style configuration for popup window components.

    Returns:
        A dictionary of UI style settings for buttons, fields, labels, and other UI elements in popup windows.
    """
    style = {
        "Button": {"background_color": BUTTON_BG_COLOR},
        "Button:disabled": {"background_color": BUTTON_DISABLED_BG_COLOR},
        "Button.Image::add": {"image_url": f"{EXTENSION_FOLDER_PATH}/icons/add.svg"},
        "Button.Image::add:disabled": {"image_url": f"{EXTENSION_FOLDER_PATH}/icons/add.svg", "color": 0x88888888},
        "Button.Image::save": {"image_url": f"{EXTENSION_FOLDER_PATH}/icons/save_dark.svg"},
        "Button.Rect": {"background_color": BUTTON_BG_COLOR, "border_radius": 2},
        "Button.Rect:disabled": {"background_color": BUTTON_DISABLED_BG_COLOR, "border_radius": 2},
        "Button.Label:disabled": {"color": LABEL_DISABLED_COLOR},
        "Circle::dot": {"background_color": LABEL_COLOR},
        "CollapsableFrame::info": {"background_color": WINDOW_BG_COLOR, "secondary_color": WINDOW_BG_COLOR},
        "CollapsableFrame::info:hovered": {"background_color": WINDOW_BG_COLOR, "secondary_color": WINDOW_BG_COLOR},
        "ComboBox": {
            "background_color": BUTTON_BG_COLOR,
            "color": LABEL_COLOR,
            "font_size": 16,
            "font": f"{EXTENSION_FOLDER_PATH}/data/fonts/NVIDIASans_Md.ttf",
        },
        "ComboBox:disabled": {
            "color": LABEL_DISABLED_COLOR,
            "font_size": 16,
            "font": f"{EXTENSION_FOLDER_PATH}/data/fonts/NVIDIASans_Md.ttf",
        },
        "Field": {
            "background_color": BUTTON_BG_COLOR,
            "color": LABEL_COLOR,
            "font_size": 16,
            "font": f"{EXTENSION_FOLDER_PATH}/data/fonts/NVIDIASans_It.ttf",
        },
        "Field:checked": {
            "background_color": BUTTON_BG_COLOR,
            "color": LABEL_COLOR,
            "font_size": 16,
            "font": f"{EXTENSION_FOLDER_PATH}/data/fonts/NVIDIASans_Md.ttf",
        },
        "Field:disabled": {
            "color": LABEL_DISABLED_COLOR,
            "font_size": 16,
            "font": f"{EXTENSION_FOLDER_PATH}/data/fonts/NVIDIASans_It.ttf",
        },
        "Image::folder": {"image_url": f"{EXTENSION_FOLDER_PATH}/icons/folder.svg", "margin": 2},
        "Image::info": {"image_url": f"{EXTENSION_FOLDER_PATH}/icons/info_icon.svg"},
        "Image::sample": {"image_url": f"{EXTENSION_FOLDER_PATH}/icons/sample.svg", "margin": 2},
        "Image::switch": {"image_url": f"{EXTENSION_FOLDER_PATH}/icons/connect.svg"},
        "Label": {
            "color": LABEL_COLOR,
            "font_size": 16,
            "font": f"{EXTENSION_FOLDER_PATH}/data/fonts/NVIDIASans_Rg.ttf",
        },
        "Label::collapsable_header": {
            "color": LABEL_TITLE_COLOR,
            "font_size": 18,
            "font": f"{EXTENSION_FOLDER_PATH}/data/fonts/NVIDIASans_Bd.ttf",
        },
        "Label::dot": {"color": LABEL_COLOR, "font_size": 15},
        "RadioButton": {"background_color": cl.transparent, "padding": 0},
        "RadioButton:checked": {"background_color": cl.transparent},
        "RadioButton:hovered": {"background_color": cl.transparent},
        "RadioButton:pressed": {"background_color": cl.transparent},
        "RadioButton.Image": {
            "image_url": f"{EXTENSION_FOLDER_PATH}/icons/radio_off.svg",
        },
        "RadioButton.Image:checked": {"image_url": f"{EXTENSION_FOLDER_PATH}/icons/radio_on.svg"},
        "Triangle": {"background_color": TRIANGLE_COLOR},
    }
    return style


def get_progress_none_style() -> dict[str, dict[str, Any]]:
    """UI style configuration for default progress step display.

    Returns:
        A dictionary of UI style settings for displaying a progress step in its default state with standard gray colors.
    """
    style = {
        "Label::step_name": {
            "color": LABEL_COLOR,
            "font_size": 20,
            "font": f"{EXTENSION_FOLDER_PATH}/data/fonts/NVIDIASans_Rg.ttf",
        },
        "Line::step_line": {"color": LABEL_COLOR, "border_width": 2},
        "Rectangle::step_item_background:hovered": {"background_color": STEP_GRAY_STACK_COLOR},
    }
    return style


def get_progress_complete_style() -> dict[str, dict[str, Any]]:
    """UI style configuration for completed progress step display.

    Returns:
        A dictionary of UI style settings for displaying a completed progress step with green colors.
    """
    style = {
        "Circle::step_index": {"background_color": STEP_GREEN_COLOR},
        "Label::step_name": {
            "color": STEP_GREEN_COLOR,
            "font_size": 20,
            "font": f"{EXTENSION_FOLDER_PATH}/data/fonts/NVIDIASans_Rg.ttf",
        },
        "Line::step_line": {"color": STEP_GREEN_COLOR, "border_width": 2},
        "Rectangle::step_item_background:hovered": {"background_color": STEP_GREEN_STACK_COLOR},
    }
    return style


def get_progress_edit_style() -> dict[str, dict[str, Any]]:
    """UI style configuration for editable progress step display.

    Returns:
        A dictionary of UI style settings for displaying an editable progress step with blue colors.
    """
    style = {
        "Circle::step_index": {"background_color": STEP_BLUE_COLOR},
        "Label::step_name": {
            "color": STEP_BLUE_COLOR,
            "font_size": 20,
            "font": f"{EXTENSION_FOLDER_PATH}/data/fonts/NVIDIASans_Rg.ttf",
        },
        "Line::step_line": {"color": STEP_BLUE_COLOR, "border_width": 2},
        "Rectangle::step_item_background:hovered": {"background_color": STEP_BLUE_STACK_COLOR},
    }
    return style


def get_progress_active_style() -> dict[str, dict[str, Any]]:
    """UI style configuration for active progress step display.

    Returns:
        A dictionary of UI style settings for displaying an active progress step with blue colors and border highlighting.
    """
    style = {
        "Circle::step_index": {
            "background_color": STEP_BLUE_COLOR,
            "border_color": STEP_ACTIVE_BORDER_COLOR,
            "border_width": 2.0,
        },
        "Label::step_name": {
            "color": STEP_BLUE_COLOR,
            "font_size": 20,
            "font": f"{EXTENSION_FOLDER_PATH}/data/fonts/NVIDIASans_Rg.ttf",
        },
        "Line::step_line": {"color": STEP_BLUE_COLOR, "border_width": 2},
        "Rectangle::step_item_background:hovered": {"background_color": STEP_BLUE_STACK_COLOR},
    }
    return style


def get_asset_picker_style() -> dict[str, dict[str, Any]]:
    """UI style configuration for asset picker components.

    Returns:
        A dictionary of UI style settings for buttons and labels in the asset picker interface.
    """
    style = {
        "Button": {"background_color": BUTTON_BG_COLOR},
        "Button.Label:disabled": {"color": LABEL_DISABLED_COLOR},
    }
    return style
