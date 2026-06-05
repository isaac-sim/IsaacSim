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

"""UI style constants and helpers for the URDF exporter options panel."""

from __future__ import annotations

import pathlib

import omni.kit.app

EXTENSION_FOLDER_PATH = pathlib.Path(
    omni.kit.app.get_app().get_extension_manager().get_extension_path_by_module(__name__)
)

BUTTON_BG_COLOR = 0xFF24211F
FRAME_BG_COLOR = 0xFF343433
FRAME_HEAD_COLOR = 0xFF8F8F8F
LABEL_COLOR = 0xFFD8D8D8
DISABLED_LABEL_COLOR = 0xFF6E6E6E
UNIT_COLOR = 0xFF6E6E6E
LINE_COLOR = 0xFF8F8F8F
TRIANGLE_COLOR = 0xFF8F8F8F
TREEVIEW_BG_COLOR = 0xFF23211F
TREEVIEW_SELECTED_COLOR = 0xFF4B4A42
TREEVIEW_ITEM_COLOR = 0xFF343432
TREEVIEW_HEADER_BG_COLOR = 0xFF2D2D2D
TREEVIEW_ITEM_FONT = 14
HEADER_FONT_SIZE = 16
FONT_SIZE = 14


def get_option_style() -> dict:
    """Return the omni.ui style dictionary for export option widgets."""
    style = {
        "Button::reset": {"background_color": 0x0, "border_radius": 1},
        "Button::reset:disabled": {"background_color": 0x0, "color": 0x0, "border_radius": 1},
        "Button::reset:hovered": {"background_color": 0x0, "border_radius": 1},
        "Button::reset:pressed": {"background_color": 0x0, "border_radius": 1},
        "CheckBox": {"border_radius": 2, "font_size": 12},
        "CollapsableFrame": {"background_color": FRAME_BG_COLOR, "secondary_color": FRAME_BG_COLOR},
        "CollapsableFrame:hovered": {"background_color": FRAME_BG_COLOR, "secondary_color": FRAME_BG_COLOR},
        "Field::StringField": {
            "background_color": BUTTON_BG_COLOR,
            "color": LINE_COLOR,
            "font_size": FONT_SIZE,
        },
        "Field::FloatField": {
            "color": LABEL_COLOR,
            "font_size": FONT_SIZE,
        },
        "Label": {
            "color": LABEL_COLOR,
            "font_size": FONT_SIZE,
        },
        "Label::header": {
            "color": FRAME_HEAD_COLOR,
            "font_size": FONT_SIZE,
        },
        "Triangle": {"background_color": TRIANGLE_COLOR, "color": TRIANGLE_COLOR},
        "ScrollingFrame": {"background_color": FRAME_BG_COLOR},
    }
    return style
