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

"""Style and color constants for the Robot Poser UI."""

import pathlib
from typing import Any

import omni.kit.app
import omni.ui as ui

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
STEP_ACTIVE_BORDER_COLOR = 0xFFFFC266
TREEVIEW_BG_COLOR = 0xFF23211F
TREEVIEW_SELECTED_COLOR = 0xFF4B4A42
TREEVIEW_ITEM_COLOR = 0xFF343432
TREEVIEW_HEADER_BG_COLOR = 0xFF2D2D2D
TRACK_TARGET_ACTIVE_COLOR = 0xFF4444AB  # dark red (ABGR)
TRACK_TARGET_INACTIVE_COLOR = 0xFF444444  # light grey (ABGR)
BUTTON_HOVER_COLOR = 0x22FFFFFF  # 30% opaque white overlay (ABGR)
ADD_ROW_HIGHLIGHT_COLOR = 0x11888888  # 30% opaque white overlay (ABGR)
TRACK_TARGET_ACTIVE_IMAGE_COLOR = 0xFF00008B  # bright red (ABGR)
PLAY_BUTTON_COLOR = TRACK_TARGET_INACTIVE_COLOR
PLAY_IMAGE_COLOR = LABEL_COLOR
WINDOW_BG_COLOR = 0xFF454545

# Font paths
FONT_REGULAR = f"{EXTENSION_FOLDER_PATH}/data/fonts/NVIDIASans_Rg.ttf"
FONT_MEDIUM = f"{EXTENSION_FOLDER_PATH}/data/fonts/NVIDIASans_Md.ttf"
FONT_SIZE = 16


def get_property_style() -> dict:
    """Return the omni.ui style dictionary for the Named Pose property panel.

    Uses the same NVIDIA Sans fonts and color palette as the Robot Poser
    window so the two UIs look consistent.

    Returns:
        Style dictionary for the property panel.
    """
    return {
        "CollapsableFrame": {
            "background_color": FRAME_BG_COLOR,
            "secondary_color": TREEVIEW_HEADER_BG_COLOR,
            "font_size": FONT_SIZE,
            "font": FONT_MEDIUM,
            "color": LABEL_TITLE_COLOR,
        },
        "CollapsableFrame:hovered": {
            "background_color": FRAME_BG_COLOR,
            "secondary_color": TREEVIEW_HEADER_BG_COLOR,
        },
        "Label": {
            "color": LABEL_COLOR,
            "font_size": FONT_SIZE,
            "font": FONT_REGULAR,
        },
        "Label::header": {
            "color": LABEL_COLOR,
            "font_size": FONT_SIZE,
            "font": FONT_MEDIUM,
        },
        "CheckBox": {
            "font_size": FONT_SIZE,
        },
        "Slider": {
            "draw_mode": ui.SliderDrawMode.FILLED,
            "background_color": TREEVIEW_BG_COLOR,
            "secondary_color": HIGHLIGHT_COLOR,
            "color": LABEL_TITLE_COLOR,
            "border_radius": 2,
            "font_size": FONT_SIZE,
            "font": FONT_REGULAR,
        },
        # -- Set Robot to Pose button --
        "Rectangle::set_pose": {
            "background_color": TRACK_TARGET_INACTIVE_COLOR,
            "border_radius": 4,
        },
        "Rectangle::set_pose:hovered": {
            "background_color": 0xFF555555,
        },
        "Label::set_pose": {
            "color": LABEL_COLOR,
            "font_size": FONT_SIZE,
            "font": FONT_MEDIUM,
        },
        "Image::play": {
            "image_url": f"{EXTENSION_FOLDER_PATH}/icons/icoSetRobotPose.svg",
        },
        # -- Track Target button --
        "Rectangle::track_active": {
            "background_color": TRACK_TARGET_ACTIVE_COLOR,
            "border_radius": 4,
        },
        "Rectangle::track_active:hovered": {
            "background_color": 0xFF5555BB,
            "border_radius": 4,
        },
        "Rectangle::track_inactive": {
            "background_color": TRACK_TARGET_INACTIVE_COLOR,
            "border_radius": 4,
        },
        "Rectangle::track_inactive:hovered": {
            "background_color": 0xFF555555,
            "border_radius": 4,
        },
        "Image::target": {
            "image_url": f"{EXTENSION_FOLDER_PATH}/icons/target.svg",
        },
        "Image::target_active": {
            "image_url": f"{EXTENSION_FOLDER_PATH}/icons/target.svg",
            "color": TRACK_TARGET_ACTIVE_IMAGE_COLOR,
        },
        "Label::track": {
            "color": LABEL_COLOR,
            "font_size": FONT_SIZE,
            "font": FONT_MEDIUM,
        },
        "Label::track_active": {
            "color": TRACK_TARGET_ACTIVE_IMAGE_COLOR,
            "font_size": FONT_SIZE,
            "font": FONT_MEDIUM,
        },
        "Rectangle::table_header": {
            "background_color": TREEVIEW_HEADER_BG_COLOR,
            "border_radius": 2,
        },
        # -- Lock icons for joint fixed state --
        "Image::lock_open": {
            "image_url": f"{EXTENSION_FOLDER_PATH}/icons/lock_open.svg",
            "color": LABEL_COLOR,
        },
        "Image::lock_open:hovered": {
            "image_url": f"{EXTENSION_FOLDER_PATH}/icons/lock_open.svg",
            "color": LABEL_TITLE_COLOR,
        },
        "Image::lock_closed": {
            "image_url": f"{EXTENSION_FOLDER_PATH}/icons/lock_closed.svg",
            "color": HIGHLIGHT_COLOR,
        },
        "Image::lock_closed:hovered": {
            "image_url": f"{EXTENSION_FOLDER_PATH}/icons/lock_closed.svg",
            "color": STEP_ACTIVE_BORDER_COLOR,
        },
        # -- Disabled slider (fixed joint) --
        "Slider:disabled": {
            "draw_mode": ui.SliderDrawMode.FILLED,
            "background_color": 0xFF1E1D1C,
            "secondary_color": 0xFF3A3530,
            "color": LABEL_DISABLED_COLOR,
            "border_radius": 2,
            "font_size": FONT_SIZE,
            "font": FONT_REGULAR,
        },
    }


def get_style() -> dict[str, Any]:
    """Return the omni.ui style dictionary for the Robot Poser extension.

    Returns:
        A style mapping compatible with omni.ui widget styling.
    """
    style = {
        "Button": {"stack_direction": ui.Direction.LEFT_TO_RIGHT},
        "Button:disabled": {"background_color": BUTTON_DISABLED_BG_COLOR},
        "Button.Rect": {"background_color": BUTTON_BG_COLOR},
        "Button.Rect:disabled": {"background_color": BUTTON_DISABLED_BG_COLOR},
        "Button.Label": {
            "color": LABEL_COLOR,
            "font_size": 16,
            "font": f"{EXTENSION_FOLDER_PATH}/data/fonts/NVIDIASans_Md.ttf",
        },
        "Button.Label:disabled": {"color": LABEL_DISABLED_COLOR},
        "CollapsableFrame": {"background_color": FRAME_BG_COLOR, "secondary_color": FRAME_HEADER_COLOR},
        "CollapsableFrame:hovered": {"background_color": FRAME_BG_COLOR, "secondary_color": FRAME_HEADER_COLOR},
        "Image::add": {"image_url": f"{EXTENSION_FOLDER_PATH}/icons/add.svg"},
        "Image::remove": {"image_url": f"{EXTENSION_FOLDER_PATH}/icons/remove_active.svg"},
        "Image::remove:disabled": {"image_url": f"{EXTENSION_FOLDER_PATH}/icons/remove_active_inactive.svg"},
        "Image::remove_header": {"image_url": f"{EXTENSION_FOLDER_PATH}/icons/remove_header.svg"},
        "Image::info": {"image_url": f"{EXTENSION_FOLDER_PATH}/icons/info_icon.svg"},
        "Image::sort": {"image_url": f"{EXTENSION_FOLDER_PATH}/icons/sort.svg"},
        "Image::target": {
            "image_url": f"{EXTENSION_FOLDER_PATH}/icons/target.svg",
        },
        "Image::target_active": {
            "image_url": f"{EXTENSION_FOLDER_PATH}/icons/target.svg",
            "color": TRACK_TARGET_ACTIVE_IMAGE_COLOR,
        },
        "Label": {
            "color": LABEL_COLOR,
            "font_size": 16,
            "font": f"{EXTENSION_FOLDER_PATH}/data/fonts/NVIDIASans_Rg.ttf",
        },
        "Label::info": {
            "color": LABEL_COLOR,
            "font_size": 14,
            "font": f"{EXTENSION_FOLDER_PATH}/data/fonts/NVIDIASans_Md.ttf",
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
        "Rectangle::add_row": {"margin": 1, "background_color": TREEVIEW_BG_COLOR},
        "Rectangle::add_row_highlight": {"margin": 1, "background_color": ADD_ROW_HIGHLIGHT_COLOR},
        "Rectangle::button_hover": {"background_color": BUTTON_HOVER_COLOR, "border_radius": 3},
        "Rectangle::treeview_background": {"background_color": TREEVIEW_BG_COLOR},
        "Rectangle::treeview": {"background_color": WINDOW_BG_COLOR},
        "Rectangle::treeview_item": {"margin": 1, "background_color": TREEVIEW_ITEM_COLOR},
        "Rectangle::treeview_item:selected": {"margin": 1, "background_color": TREEVIEW_SELECTED_COLOR},
        "Rectangle::treeview_item_button": {"margin": 1, "background_color": TREEVIEW_ITEM_COLOR},
        "Rectangle::track_target_active": {"background_color": TRACK_TARGET_ACTIVE_COLOR, "border_radius": 3},
        "Rectangle::track_target_inactive": {"background_color": TRACK_TARGET_INACTIVE_COLOR, "border_radius": 3},
        "Image::play": {
            "image_url": f"{EXTENSION_FOLDER_PATH}/icons/icoSetRobotPose.svg",
            "color": PLAY_IMAGE_COLOR,
        },
        "Rectangle::play_button": {"background_color": PLAY_BUTTON_COLOR, "border_radius": 3},
        "ScrollingFrame": {"background_color": FRAME_BG_COLOR},
        "ScrollingFrame::treeview": {"background_color": TREEVIEW_BG_COLOR},
        "TreeView": {"background_selected_color": TREEVIEW_BG_COLOR},  # the hover color of the TreeView selected item
        "TreeView.Header": {"background_color": TREEVIEW_BG_COLOR},
        "TreeView.Header::background": {"margin": 1, "background_color": TREEVIEW_HEADER_BG_COLOR},
        "TreeView:selected": {
            "background_color": TREEVIEW_BG_COLOR
        },  # selected margin color, set to scrollingFrame background color
        "VStack::margin_vstack": {"margin_width": 15, "margin_height": 10},
    }
    return style
