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

"""Defines UI styling constants and functions for the robot setup assembler extension."""

import omni
import omni.kit.app

EXTENSION_FOLDER_PATH = omni.kit.app.get_app().get_extension_manager().get_extension_path_by_module(__name__)

## colors
BUTTON_BG_COLOR = 0xFF24211F
BUTTON_DISABLED_BG_COLOR = 0xFF2F2E2C
FRAME_HEADER_COLOR = 0xFF2F2F2E
FRAME_BG_COLOR = 0xFF343433
HIGHLIGHT_COLOR = 0xFFB0703B
LABEL_COLOR = 0xFF9E9E9E
LABEL_DISABLED_COLOR = 0xFF545452
LABEL_TITLE_COLOR = 0xFFCCCCCC
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


def get_style() -> dict[str, dict[str, str | int]]:
    """Returns the UI style dictionary for the robot setup assembler extension.

    Provides styling configurations for various UI components including buttons, labels, images,
    lines, and other visual elements used in the robot setup assembler interface.

    Returns:
        A dictionary mapping UI element selectors to their style properties such as colors,
        font sizes, and image URLs.
    """
    style = {
        "Button:disabled": {"background_color": BUTTON_DISABLED_BG_COLOR},
        "Button:hovered": {"background_color": BUTTON_DISABLED_BG_COLOR},
        "Button.Rect": {"background_color": BUTTON_BG_COLOR},
        "Button.Rect:disabled": {"background_color": BUTTON_DISABLED_BG_COLOR},
        "Button.Image::save": {"image_url": f"{EXTENSION_FOLDER_PATH}/icons/save_dark.svg"},
        "Button.Image::play": {"image_url": f"{EXTENSION_FOLDER_PATH}/icons/play.svg"},
        "Button.Label": {
            # "color": LABEL_COLOR,
            "font_size": 14,
        },
        "Button.Label:disabled": {"color": LABEL_DISABLED_COLOR},
        # this will also change the collapsable frame header height
        # "CollapsableFrame": {"padding": 16},
        "Image::info": {"image_url": f"{EXTENSION_FOLDER_PATH}/icons/info_icon.svg"},
        "Image::help": {"image_url": f"{EXTENSION_FOLDER_PATH}/icons/help_active.svg"},
        "Image::picker": {"image_url": f"{EXTENSION_FOLDER_PATH}/icons/go_to.svg"},
        "Image::select": {"image_url": f"{EXTENSION_FOLDER_PATH}/icons/go_to.svg"},
        "Label::info": {"color": STEP_GRAY_STACK_COLOR, "font_size": 12},
        "Line": {"color": LINE_COLOR},
        "Triangle": {"background_color": TRIANGLE_COLOR, "color": TRIANGLE_COLOR},
    }
    return style
