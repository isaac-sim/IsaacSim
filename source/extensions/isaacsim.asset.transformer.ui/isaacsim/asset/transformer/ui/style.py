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

"""Omni UI style dictionary for the Asset Transformer window."""

import omni.ui as ui
from omni.ui import color as cl

from .constants import (
    ACTION_TITLE_FONT_SIZE,
    BORDER_RADIUS,
    CONTENT_BACKGROUND_COLOR,
    DISABLED_BACKGROUND_COLOR,
    DISABLED_TEXT_COLOR,
    EXECUTE_ACCENT_COLOR,
    HEADER_BACKGROUND_COLOR,
    HEADER_FONT,
    HEADER_FONT_SIZE,
    ICON_PATH,
    INDENT_SIZE,
    INFO_FONT,
    INNER_BACKGROUND_COLOR,
    NORMAL_FONT,
    NORMAL_FONT_SIZE,
    PLACEHOLDER_TEXT_COLOR,
    SECONDARY_TEXT_COLOR,
    SMALL_FONT_SIZE,
    SMALL_MARGIN,
)

STYLE = {
    # ==================== Button Styles ====================
    "Button": {
        "stack_direction": ui.Direction.LEFT_TO_RIGHT,
        "padding": 4,
    },
    "Button:disabled": {
        "background_color": DISABLED_BACKGROUND_COLOR,
    },
    "Button.Label:disabled": {
        "color": DISABLED_TEXT_COLOR,
    },
    "Button.Image:disabled": {
        "color": DISABLED_TEXT_COLOR,
    },
    "Button::image_button": {
        "padding": 0,
        "background_color": cl.transparent,
    },
    "Button.Image::image_button": {
        "color": cl.lightgray,
    },
    "Button::image_button:hovered": {
        "background_color": cl.transparent,
    },
    "Button.Image::image_button:hovered": {
        "color": cl.white,
    },
    "Button::image_button:pressed": {
        "background_color": cl.transparent,
    },
    "Button.Image::image_button:pressed": {
        "color": cl.gray,
    },
    "Button::add_action": {
        "padding": 4,
        "margin": 0,
    },
    "Button::execute_action": {
        "stack_direction": ui.Direction.LEFT_TO_RIGHT,
        "padding": 8,
        "border_radius": BORDER_RADIUS,
    },
    "Button.Image::execute_action": {
        "color": EXECUTE_ACCENT_COLOR,
    },
    # ==================== CheckBox Styles ====================
    "CheckBox::action_row": {
        "alignment": ui.Alignment.CENTER,
    },
    # ==================== CollapsableFrame Styles ====================
    "CollapsableFrame": {
        "background_color": CONTENT_BACKGROUND_COLOR,
        "secondary_color": HEADER_BACKGROUND_COLOR,
    },
    "CollapsableFrame:hovered": {
        "secondary_color": HEADER_BACKGROUND_COLOR,
    },
    "CollapsableFrame:pressed": {
        "secondary_color": HEADER_BACKGROUND_COLOR,
    },
    "CollapsableFrame.Header": {
        "color": cl.lightgray,
        "font_size": HEADER_FONT_SIZE,
        "font": HEADER_FONT,
    },
    "CollapsableFrame.Header:hovered": {
        "color": cl.white,
    },
    "CollapsableFrame.Header:pressed": {
        "color": cl.white,
    },
    "CollapsableFrame.Triangle": {
        "background_color": cl.lightgray,
    },
    "CollapsableFrame.Triangle:hovered": {
        "background_color": cl.white,
    },
    # ==================== HStack Styles ====================
    "HStack::no_margin": {
        "margin": 0,
    },
    # ==================== Image Styles ====================
    "Image::drag_handle": {
        "color": cl.dimgray,
    },
    # ==================== Label Styles ====================
    "Label": {
        "color": cl.darkgray,
        "font_size": NORMAL_FONT_SIZE,
        "font": NORMAL_FONT,
    },
    "Label::action": {
        "color": cl.lightgray,
        "font_size": NORMAL_FONT_SIZE,
        "font": HEADER_FONT,
    },
    "Label::action_title": {
        "color": cl.lightgray,
        "font_size": ACTION_TITLE_FONT_SIZE,
        "font": HEADER_FONT,
    },
    "Label::button": {
        "color": cl.lightgray,
        "font_size": NORMAL_FONT_SIZE,
        "font": NORMAL_FONT,
    },
    "Label::header": {
        "color": cl.lightgray,
        "font_size": HEADER_FONT_SIZE,
        "font": HEADER_FONT,
    },
    "Label::info": {
        "color": cl.darkgray,
        "font_size": NORMAL_FONT_SIZE,
        "font": INFO_FONT,
    },
    "Label::no_config": {
        "color": DISABLED_TEXT_COLOR,
        "font_size": SMALL_FONT_SIZE,
    },
    "Label::placeholder_config": {
        "color": PLACEHOLDER_TEXT_COLOR,
        "font_size": SMALL_FONT_SIZE,
    },
    "Label::secondary": {
        "color": SECONDARY_TEXT_COLOR,
    },
    # ==================== RadioButton Styles ====================
    "RadioButton": {
        "color": cl.lightgray,
        "font_size": NORMAL_FONT_SIZE,
        "stack_direction": ui.Direction.LEFT_TO_RIGHT,
        "background_color": cl.transparent,
        "padding": 0,
    },
    "RadioButton:checked": {
        "stack_direction": ui.Direction.LEFT_TO_RIGHT,
        "background_color": cl.transparent,
        "padding": 0,
    },
    "RadioButton:hovered": {
        "stack_direction": ui.Direction.LEFT_TO_RIGHT,
        "background_color": cl.transparent,
        "padding": 0,
    },
    "RadioButton:pressed": {
        "stack_direction": ui.Direction.LEFT_TO_RIGHT,
        "background_color": cl.transparent,
        "padding": 0,
    },
    "RadioButton.Image": {"image_url": f"{ICON_PATH}/radio_off.svg"},
    "RadioButton.Image:checked": {"image_url": f"{ICON_PATH}/radio_on.svg"},
    "RadioButton.Image:hovered": {"image_url": f"{ICON_PATH}/radio_part.svg"},
    # ==================== Rectangle Styles ====================
    "Rectangle::list_background": {
        "background_color": INNER_BACKGROUND_COLOR,
        "padding": 0,
    },
    "Rectangle::action_row": {
        "background_color": CONTENT_BACKGROUND_COLOR,
        "border_radius": BORDER_RADIUS,
        "corner_flag": ui.CornerFlag.ALL,
        "margin": 3,
        "padding": 0,
        "border_width": 0,
    },
    # ==================== TreeView Styles ====================
    "TreeView::action_list": {
        "margin": 4,
        "background_color": cl.transparent,
        "background_selected_color": cl.transparent,
        "background_selected_hover_color": cl.transparent,
    },
    "TreeView::action_list:selected": {
        "background_color": 0xFFFFC16E,
    },
    # ==================== Triangle Styles ====================
    "Triangle::action_row": {
        "background_color": cl.lightgray,
        "margin_height": 0,
    },
    # ==================== VStack Styles ====================
    "VStack::action_row": {
        "margin": 4,
    },
    "VStack::center_conten  t": {
        "margin": 0,
    },
    "VStack::indent": {
        "margin": INDENT_SIZE,
    },
    "VStack::small_margin": {
        "margin": SMALL_MARGIN,
    },
    "VStack::no_margin": {
        "margin": 0,
    },
}
