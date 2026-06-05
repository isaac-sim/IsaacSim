"""UI style definitions for the conveyor builder extension."""

# SPDX-FileCopyrightText: Copyright (c) 2018-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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

"""UI styling configuration for the conveyor asset generation interface."""


import os
from pathlib import Path

import carb
import omni.ui as ui

settings = carb.settings.get_settings()

CURRENT_PATH = Path(__file__).parent
ICON_PATH = CURRENT_PATH.parent.parent.parent.parent.parent.parent.joinpath("data")

UI_STYLES = {}

UI_STYLES["NvidiaLight"] = {
    "Button.Image::filter": {"image_url": os.path.join(ICON_PATH, "icons/filter.svg"), "color": 0xFF535354},
    "Button.Image::options": {"image_url": os.path.join(ICON_PATH, "icons/options.svg"), "color": 0xFF535354},
    "Button.Image::arrow_up": {
        "image_url": os.path.join(ICON_PATH, "icons/arrow_up.svg"),
        "color": 0xFF535354,
        "margin": 0,
    },
    "Button.Image::arrow_down": {
        "image_url": os.path.join(ICON_PATH, "icons/arrow_down.svg"),
        "color": 0xFF535354,
        "margin": 0,
    },
    "Button.Image::accept": {"image_url": os.path.join(ICON_PATH, "icons/check_square.svg"), "color": 0xFF535354},
    "Button.Image::cancel": {"image_url": os.path.join(ICON_PATH, "icons/times_circle.svg"), "color": 0xFF535354},
    "Image::assembly": {"image_url": os.path.join(ICON_PATH, "icons/assembly.svg"), "color": 0xFF535354, "margin": 0},
    "Image::part_studio": {
        "image_url": os.path.join(ICON_PATH, "icons/part_studio.svg"),
        "color": 0xFF535354,
        "margin": 0,
    },
    "Image::blob": {"image_url": os.path.join(ICON_PATH, "icons/blob.svg"), "color": 0xFF535354, "margin": 0},
    "Image::bom": {"image_url": os.path.join(ICON_PATH, "icons/bom.svg"), "color": 0xFF535354, "margin": 0},
    "Image::part": {"image_url": os.path.join(ICON_PATH, "icons/part_studio.svg"), "color": 0xFF535354, "margin": 0},
    "Button::filter": {"background_color": 0x0, "margin": 0},
    "Button::options": {"background_color": 0x0, "margin": 0},
    "Button::arrow_up": {"background_color": 0x0, "margin": 0},
    "Button::arrow_down": {"background_color": 0x0, "margin": 0},
    "Rectangle::Splitter": {"background_color": 0xFFE0E0E0, "margin_width": 2},
    "Rectangle::Splitter:hovered": {"background_color": 0xFFB0703B},
    "Rectangle::Splitter:pressed": {"background_color": 0xFFB0703B},
    "Splitter": {"background_color": 0xFFE0E0E0, "margin_width": 2},
    "TreeView": {
        "background_color": 0xFF535354,
        "background_selected_color": 0xFF6E6E6E,
        "secondary_color": 0xFFACACAC,
    },
    "TreeView:hovered": {"background_color": 0xFF6E6E6E},
    "TreeView:selected": {"background_color": 0xFFBEBBAE},
    "TreeView.Column": {"background_color": 0x0, "color": 0xFFD6D6D6, "margin": 0},
    "TreeView.Header": {
        "background_color": 0xFF535354,
        "color": 0xFFD6D6D6,
        "border_color": 0xFF707070,
        "border_width": 0.5,
    },
    "TreeView.Header::name": {"margin": 3, "alignment": ui.Alignment.LEFT},
    "TreeView.Header::date": {"margin": 3, "alignment": ui.Alignment.CENTER},
    "TreeView.Header::size": {"margin": 3, "alignment": ui.Alignment.RIGHT},
    "TreeView.Icon:selected": {"color": 0xFF535354},
    "TreeView.Header.Icon": {"color": 0xFF8A8777},
    "TreeView.Icon::default": {"color": 0xFF8A8777},
    "TreeView.Icon::file": {"color": 0xFF8A8777},
    "TreeView.Item": {"color": 0xFFD6D6D6},
    "TreeView.Item:selected": {"color": 0xFF2A2825},
    "TreeView.ScrollingFrame": {"background_color": 0xFF535354, "secondary_color": 0xFFE0E0E0},
    "GridView.ScrollingFrame": {"background_color": 0xFF535354, "secondary_color": 0xFFE0E0E0},
    "GridView.Grid": {"background_color": 0x0, "margin_width": 10},
    "ZoomBar": {"background_color": 0x0, "border_radius": 2},
    "ZoomBar.Slider": {
        "draw_mode": ui.SliderDrawMode.HANDLE,
        "background_color": 0xFF23211F,
        "secondary_color": 0xFF9D9D9D,
        "color": 0x0,
        "alignment": ui.Alignment.CENTER,
        "padding": 0,
        "margin": 5,
        "font_size": 8,
    },
    "ZoomBar.Button": {"background_color": 0x0, "margin": 0, "padding": 0},
    "ZoomBar.Button.Image": {"color": 0xFFFFFFFF, "alignment": ui.Alignment.CENTER},
    "Card": {"background_color": 0x0, "margin": 8},
    "Card:hovered": {"background_color": 0xFF6E6E6E, "border_color": 0xFF3A3A3A, "border_width": 0},
    "Card:selected": {"background_color": 0xFFBEBBAE, "border_color": 0xFF8A8777, "border_width": 0},
    "Card.Image": {
        "background_color": 0xFFC9C9C9,
        "color": 0xFFFFFFFF,
        "corner_flag": ui.CornerFlag.TOP,
        "alignment": ui.Alignment.CENTER,
        "margin": 8,
    },
    "Card.Badge": {"background_color": 0xFFC9C9C9, "color": 0xFFFFFFFF},
    "Card.Badge::shadow": {"background_color": 0xFFC9C9C9, "color": 0xDD444444},
    "Card.Label": {
        "background_color": 0xFFC9C9C9,
        "color": 0xFFD6D6D6,
        "font_size": 12,
        "alignment": ui.Alignment.CENTER_TOP,
        "margin_width": 8,
        "margin_height": 2,
    },
    "Card.Label:checked": {"color": 0xFF23211F},
}

UI_STYLES["NvidiaDark"] = {
    "ConveyorCardBackground:selected": {"background_color": 0x110099FF, "border_radius": 8},
    "ConveyorCardBackground": {"background_color": 0x1199FF99, "border_radius": 8},
    "ConveyorCard": {"background_color": 0x33CCDDCC, "border_radius": 8},
    "ConveyorCard::current": {"background_color": 0x33666666, "border_radius": 8},
    "Button:selected": {"background_color": 0xFF777777},
    "Button": {"background_color": 0x22FFFFDD},
    "Button:disabled": {"background_color": 0x33000000},
    "Button.Image": {"color": 0x88FFFFDD},
    "CardButton": {"background_color": 0x88DDDDCC, "border_radius": 6, "margin": 3, "padding": 0},
    "CardButton.Image": {"aligmnent": ui.Alignment.CENTER, "stack_direction": ui.Direction.TOP_TO_BOTTOM},
    "CardButton:hovered": {"background_color": 0xDDDDDDDD},
    "CardButton:pressed": {"background_color": 0xFFEEEEEE},
    "CardButton:selected": {"background_color": 0xFF777777},
    "CardButton.Image::add_box": {"image_url": os.path.join(ICON_PATH, "icons/add_box.svg")},
    "CardButton.Image::delete": {"image_url": os.path.join(ICON_PATH, "icons/delete.svg")},
    "Button.Image:selected": {"color": 0xFF00B976},
    "Button.Image:disabled": {"color": 0x33000000},
    "Button.Image::arrow_down": {"image_url": os.path.join(ICON_PATH, "icons/arrow_down.svg")},
    "Button::square": {"padding": 0, "margin": 0},
    "Button::square:selected": {"padding": 0, "margin": 0},
    "Button.Image::square": {"image_url": os.path.join(ICON_PATH, "icons/square.svg"), "padding": 0, "margin": 0},
    "Button.Image::delete": {"image_url": os.path.join(ICON_PATH, "icons/delete.svg")},
    "Button.Image::add_box": {"image_url": os.path.join(ICON_PATH, "icons/add_box.svg")},
    "Button.Image::flip": {"image_url": os.path.join(ICON_PATH, "icons/flip.svg")},
    "Button.Image::rotate_left": {"image_url": os.path.join(ICON_PATH, "icons/rotate_left.svg")},
    "Button.Image::rotate_right": {"image_url": os.path.join(ICON_PATH, "icons/rotate_right.svg")},
    # Arrows
    "Button.Image::arrow_90_left": {"image_url": os.path.join(ICON_PATH, "icons/arrows/arrow_90_left.svg")},
    "Button.Image::arrow_90_right": {"image_url": os.path.join(ICON_PATH, "icons/arrows/arrow_90_right.svg")},
    "Button.Image::arrow_180_left": {"image_url": os.path.join(ICON_PATH, "icons/arrows/arrow_180_left.svg")},
    "Button.Image::arrow_180_right": {"image_url": os.path.join(ICON_PATH, "icons/arrows/arrow_180_right.svg")},
    "Button.Image::arrow_up": {"image_url": os.path.join(ICON_PATH, "icons/arrows/arrow_up.svg")},
    # styles
    "Button.Image::conveyor_belt": {"image_url": os.path.join(ICON_PATH, "icons/conveyor_styles/conveyor_belt.svg")},
    "Button.Image::conveyor_combine": {
        "image_url": os.path.join(ICON_PATH, "icons/conveyor_styles/conveyor_combine.svg")
    },
    "Button.Image::conveyor_roller": {
        "image_url": os.path.join(ICON_PATH, "icons/conveyor_styles/conveyor_roller.svg")
    },
    # Ramps
    "Button.Image::ramp_1l": {"image_url": os.path.join(ICON_PATH, "icons/ramps/ramp_1l.svg")},
    "Button.Image::ramp_1r": {"image_url": os.path.join(ICON_PATH, "icons/ramps/ramp_1r.svg")},
    "Button.Image::ramp_2l": {"image_url": os.path.join(ICON_PATH, "icons/ramps/ramp_2l.svg")},
    "Button.Image::ramp_2r": {"image_url": os.path.join(ICON_PATH, "icons/ramps/ramp_2r.svg")},
    "Button.Image::ramp_flat": {"image_url": os.path.join(ICON_PATH, "icons/ramps/ramp_flat.svg")},
    # Types
    "Button.Image::type_straight": {"image_url": os.path.join(ICON_PATH, "icons/types/straight.svg")},
    "Button.Image::type_start": {"image_url": os.path.join(ICON_PATH, "icons/types/start.svg")},
    "Button.Image::type_end": {"image_url": os.path.join(ICON_PATH, "icons/types/end.svg")},
    "Button.Image::type_y": {"image_url": os.path.join(ICON_PATH, "icons/types/y_merge.svg")},
    "Button.Image::type_t": {"image_url": os.path.join(ICON_PATH, "icons/types/t_merge.svg")},
    # Others
    "Button.Image::accept": {"image_url": os.path.join(ICON_PATH, "icons/check_square.svg"), "margin": 0, "padding": 0},
    "Button.Image::cancel": {"image_url": os.path.join(ICON_PATH, "icons/times_circle.svg"), "margin": 0, "padding": 0},
    "Button.Image::options": {"image_url": os.path.join(ICON_PATH, "icons/options.svg")},
    "Image::processing": {"image_url": os.path.join(ICON_PATH, "icons/spinner.svg"), "margin": 0},
    "Image::error": {"image_url": os.path.join(ICON_PATH, "icons/times_circle.svg"), "color": 0x88AAAAFF, "margin": 0},
    "Image::warning": {"image_url": os.path.join(ICON_PATH, "icons/exclamation.svg"), "color": 0x88AAFFFF, "margin": 0},
    "Image::changed": {"image_url": os.path.join(ICON_PATH, "icons/info.svg"), "margin": 0},
    "Button::filter": {"background_color": 0x0, "margin": 0},
    "Button::options": {"background_color": 0x0, "margin": 0},
    "Button::accept": {"padding": 3, "margin": 1},
    "Button::cancel": {"padding": 3, "margin": 1},
    "Label::search": {"color": 0xFF808080, "margin_width": 4},
    "Label::error": {"color": 0xFF1E1EFF},
    "Label::warning": {"color": 0xFF5ECCCC},
    "Label::changed": {"color": 0xFFEE3E3E},
    "Splitter": {"background_color": 0x0, "margin_width": 0},
    "Splitter:hovered": {"background_color": 0xFFB0703B},
    "Splitter:pressed": {"background_color": 0xFFB0703B},
    "TreeView.ScrollingFrame": {"background_color": 0xFF23211F},
    "TreeView.Frame": {"background_color": 0xFF23211F},
    "TreeView": {
        "background_color": 0xFF23211F,
        "background_selected_color": 0x223A3A3A,
        "color": 0xFF9E9E9E,
        "selected_color": 0xFF3E3E3E,
    },
    "TreeView:selected": {"background_color": 0xFF8A8777, "color": 0xFF2A2825},
    "TreeView.Column": {"background_color": 0x0, "color": 0xFFADAC9F, "margin": 0},
    "TreeView.Header": {"background_color": 0xFF343432, "color": 0xFF9E9E9E},
    "TreeView.Icon": {"color": 0xFFFFFFFF, "padding": 0},
    "TreeView.Icon::expand": {"color": 0xFFFFFFFF},
    "TreeView.Icon:selected": {"color": 0xFFFFFFFF},
    "TreeView.Checked": {"color": 0xFF444444},
    "Rectangle::Splitter": {"background_color": 0x0, "margin": 3, "border_radius": 2},
    "Rectangle::Splitter:hovered": {"background_color": 0xFFB0703B},
    "Rectangle::Splitter:pressed": {"background_color": 0xFFB0703B},
    "ZoomBar.Button": {"background_color": 0x0, "margin": 0, "padding": 0},
    "ZoomBar.Button.Image": {"color": 0xFFFFFFFF, "alignment": ui.Alignment.CENTER},
    "GridView": {"background_color": 0xFF13110F},
    "GridView.Grid": {"background_color": 0xFF13110F, "margin": 2},
    "Card": {"background_color": 0x33FFFFFF, "margin_width": 8, "border_radius": 10},
    "Card:hovered": {"background_color": 0xFF8A8777, "border_color": 0xFF8A8777, "border_width": 2},
    # "Card:selected": {"background_color": 0x0023212F, "border_color": 0x0023212F, "border_width": 0,"debug_color": 0x44FFFFFF},
    "Card:checked": {
        "background_color": 0xFF232120,
        "color": 0xFF232120,
        "border_color": 0xFFFFFFFF,
        "border_width": 2,
    },
    "Card.Image": {
        "background_color": 0x0,
        "color": 0xFFFFFFFF,
        "corner_flag": ui.CornerFlag.TOP,
        "alignment": ui.Alignment.CENTER,
        "margin": 8,
    },
    "Card.Badge": {"background_color": 0x0, "color": 0xFFFFFFFF},
    "Card.Badge::shadow": {"background_color": 0x0, "color": 0xDD444444},
    "Card.Label": {
        "background_color": 0x0,
        "color": 0xFF9E9E9E,
        "font_size": 13,
        "alignment": ui.Alignment.CENTER_TOP,
        "margin_width": 8,
        "margin_height": 2,
    },
    "Card.Label:checked": {"color": 0xFF1E1E1E},
}
