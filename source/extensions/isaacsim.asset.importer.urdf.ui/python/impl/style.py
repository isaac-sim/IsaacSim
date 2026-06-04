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
"""Style definitions for the URDF importer UI.

Re-uses the base style from :mod:`isaacsim.gui.components.style` and extends it
with URDF-specific icon entries. The option panel style references
extension-local font assets and cannot be shared.
"""

from __future__ import annotations

import pathlib

import omni
import omni.kit.app
import omni.ui as ui
from isaacsim.gui.components.style import get_style as _get_base_style
from omni.ui import color as cl


def get_style() -> dict:
    """Get the primary UI style dictionary.

    Returns the shared base style from ``isaacsim.gui.components`` merged with
    URDF-specific icon entries for accept, autotune, cancel, and remove buttons.

    Returns:
        Style dictionary used by the URDF importer UI.
    """
    style = _get_base_style()

    extension_manager = omni.kit.app.get_app().get_extension_manager()
    ext_id = extension_manager.get_enabled_extension_id("isaacsim.asset.importer.urdf.ui")
    ext_root = extension_manager.get_extension_path(ext_id) if ext_id else None
    custom_icons_path = f"{ext_root}/icons" if ext_root else None

    if custom_icons_path:
        style.update(
            {
                "Button.Image::accept": {
                    "image_url": f"{custom_icons_path}/check_square.svg",
                    "color": 0x88FFFFDD,
                    "background_color": 0x0,
                    "margin": 0,
                    "padding": 0,
                },
                "Button.Image::autotune": {
                    "image_url": f"{custom_icons_path}/config.svg",
                    "color": 0x88FFFFDD,
                    "background_color": 0x0,
                    "margin": 0,
                    "padding": 0,
                },
                "Button.Image::cancel": {
                    "image_url": f"{custom_icons_path}/times_circle.svg",
                    "color": 0x88FFFFDD,
                    "background_color": 0x0,
                    "margin": 0,
                    "padding": 0,
                },
                "IconButton.Image::Remove": {
                    "image_url": f"{custom_icons_path}/remove_dark.svg",
                    "background_color": 0x0,
                    "color": cl("CB6A6A"),
                    "alignment": ui.Alignment.CENTER,
                },
                "IconButton.Image::Remove:hovered": {
                    "image_url": f"{custom_icons_path}/remove_selected.svg",
                    "background_color": 0x0,
                    "color": cl("FA3420"),
                    "alignment": ui.Alignment.CENTER,
                },
            }
        )

    return style


## -------------------------------new options start--------------------------------

EXTENSION_FOLDER_PATH = pathlib.Path(
    omni.kit.app.get_app().get_extension_manager().get_extension_path_by_module(__name__)
)

## colors
BUTTON_BG_COLOR = 0xFF24211F
FRAME_BG_COLOR = 0xFF343433
FRAME_HEAD_COLOR = 0xFF8F8F8F
STRING_FIELD_LABEL_COLOR = 0xFF8F8F8F
LABEL_COLOR = 0xFFD8D8D8
LABEL_TITLE_COLOR = 0xFFCCCCCC
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
    """Get the option panel style dictionary.

    Returns:
        Style dictionary for option widgets and tables.
    """
    style = {
        "Button::reset": {"background_color": 0x0, "border_radius": 1},
        "Button::reset:disabled": {"background_color": 0x0, "color": 0x0, "border_radius": 1},
        "Button::reset:hovered": {"background_color": 0x0, "border_radius": 1},
        "Button::reset:pressed": {"background_color": 0x0, "border_radius": 1},
        "CheckBox": {"border_radius": 2, "font_size": 12},
        "CollapsableFrame": {"background_color": FRAME_BG_COLOR, "secondary_color": FRAME_BG_COLOR},
        "CollapsableFrame:hovered": {"background_color": FRAME_BG_COLOR, "secondary_color": FRAME_BG_COLOR},
        "ComboBox::treeview_item": {
            "color": LABEL_COLOR,
            "background_selected_color": 0x00,
            "background_color": TREEVIEW_BG_COLOR,
            "secondary_color": TREEVIEW_BG_COLOR,
            "border_radius": 0,
            "padding": 0,
            "font_size": TREEVIEW_ITEM_FONT,
            "font": f"{EXTENSION_FOLDER_PATH}/data/fonts/NVIDIASans_Rg.ttf",
        },
        "ComboBox::treeview_item:selected": {
            "color": LABEL_COLOR,
            "background_selected_color": 0x0,
            "background_color": TREEVIEW_BG_COLOR,
            "secondary_color": TREEVIEW_BG_COLOR,
            "border_radius": 0,
            "padding": 0,
            "font_size": TREEVIEW_ITEM_FONT,
            "font": f"{EXTENSION_FOLDER_PATH}/data/fonts/NVIDIASans_Rg.ttf",
        },
        "ComboBox::treeview_item:hovered": {
            "color": LABEL_COLOR,
            "background_selected_color": 0x0,
            "background_color": TREEVIEW_BG_COLOR,
            "secondary_color": TREEVIEW_BG_COLOR,
            "border_radius": 0,
            "padding": 0,
            "font_size": TREEVIEW_ITEM_FONT,
            "font": f"{EXTENSION_FOLDER_PATH}/data/fonts/NVIDIASans_Rg.ttf",
        },
        "ComboBox::treeview_item:disabled": {
            "color": DISABLED_LABEL_COLOR,
            "background_selected_color": 0x0,
            "background_color": TREEVIEW_BG_COLOR,
            "secondary_color": TREEVIEW_BG_COLOR,
            "border_radius": 0,
            "padding": 0,
            "font_size": TREEVIEW_ITEM_FONT,
            "font": f"{EXTENSION_FOLDER_PATH}/data/fonts/NVIDIASans_Rg.ttf",
        },
        "Field::StringField": {
            "background_color": BUTTON_BG_COLOR,
            "color": STRING_FIELD_LABEL_COLOR,
            "font_size": FONT_SIZE,
            "font": f"{EXTENSION_FOLDER_PATH}/data/fonts/NVIDIASans_It.ttf",
        },
        "Field::FloatField": {
            "color": LABEL_COLOR,
            "font_size": FONT_SIZE,
            "font": f"{EXTENSION_FOLDER_PATH}/data/fonts/NVIDIASans_Rg.ttf",
        },
        "Field::resetable": {
            "background_color": 0x00,
            "color": LABEL_COLOR,
            "font_size": TREEVIEW_ITEM_FONT,
            "padding": 4,
            "font": f"{EXTENSION_FOLDER_PATH}/data/fonts/NVIDIASans_Rg.ttf",
        },
        "Field::resetable:disabled": {
            "background_color": 0x00,
            "color": DISABLED_LABEL_COLOR,
            "font_size": TREEVIEW_ITEM_FONT,
            "padding": 4,
            "font": f"{EXTENSION_FOLDER_PATH}/data/fonts/NVIDIASans_Rg.ttf",
        },
        "Field::resetable:hovered": {
            "background_color": 0x00,
            "color": LABEL_COLOR,
            "font_size": TREEVIEW_ITEM_FONT,
            "padding": 4,
            "font": f"{EXTENSION_FOLDER_PATH}/data/fonts/NVIDIASans_Rg.ttf",
        },
        "Field::resetable:pressed": {
            "background_color": 0x00,
            "color": LABEL_COLOR,
            "font_size": TREEVIEW_ITEM_FONT,
            "padding": 4,
            "font": f"{EXTENSION_FOLDER_PATH}/data/fonts/NVIDIASans_Rg.ttf",
        },
        "Line": {"color": LINE_COLOR},
        "Label": {
            "color": LABEL_COLOR,
            "font_size": FONT_SIZE,
            "font": f"{EXTENSION_FOLDER_PATH}/data/fonts/NVIDIASans_Md.ttf",
        },
        "Label::header": {
            "color": FRAME_HEAD_COLOR,
            "font_size": FONT_SIZE,
            "font": f"{EXTENSION_FOLDER_PATH}/data/fonts/NVIDIASans_Md.ttf",
        },
        "Label::collapsable_header": {
            "color": FRAME_HEAD_COLOR,
            "font_size": HEADER_FONT_SIZE,
            "font": f"{EXTENSION_FOLDER_PATH}/data/fonts/NVIDIASans_Rg.ttf",
        },
        "Label::treeview_item": {
            "color": LABEL_COLOR,
            "font_size": TREEVIEW_ITEM_FONT,
            "font": f"{EXTENSION_FOLDER_PATH}/data/fonts/NVIDIASans_Rg.ttf",
        },
        "Label::index": {
            "color": LABEL_COLOR,
            "font_size": TREEVIEW_ITEM_FONT,
            "font": f"{EXTENSION_FOLDER_PATH}/data/fonts/NVIDIASans_Rg.ttf",
        },
        "Label::density": {
            "color": UNIT_COLOR,
            "font_size": FONT_SIZE,
            "font": f"{EXTENSION_FOLDER_PATH}/data/fonts/NVIDIASans_Lt.ttf",
        },
        "Label::exponent": {
            "color": UNIT_COLOR,
            "font_size": 8,
            "font": f"{EXTENSION_FOLDER_PATH}/data/fonts/NVIDIASans_Lt.ttf",
        },
        "RadioButton": {"background_color": cl.transparent, "padding": 0},
        "RadioButton:checked": {"background_color": cl.transparent, "padding": 0},
        "RadioButton:hovered": {"background_color": cl.transparent, "padding": 0},
        "RadioButton.Image": {
            "image_url": f"{EXTENSION_FOLDER_PATH}/icons/radio_off.svg",
            "color": LABEL_COLOR,
        },
        "RadioButton.Image:hovered": {
            "image_url": f"{EXTENSION_FOLDER_PATH}/icons/radio_off.svg",
            "color": LABEL_COLOR,
        },
        "RadioButton.Image:checked": {"image_url": f"{EXTENSION_FOLDER_PATH}/icons/radio_on.svg", "color": LABEL_COLOR},
        "RadioButton:pressed": {"background_color": cl.transparent},
        "Triangle": {"background_color": TRIANGLE_COLOR, "color": TRIANGLE_COLOR},
        "Triangle::mask": {"color": LABEL_COLOR},
        "Rectangle::mask": {"background_color": TREEVIEW_BG_COLOR, "border_radius": 1},
        "Rectangle::reset_invalid": {"background_color": 0xFF505050, "border_radius": 1},
        "Rectangle::reset": {"background_color": 0xFFA07D4F, "border_radius": 1},
        "Rectangle::reset:disabled": {"background_color": 0x0, "border_radius": 1},
        "Rectangle::treeview_item": {
            "background_color": TREEVIEW_BG_COLOR,
            "border_width": 1,
            "border_color": 0xFF505050,
        },
        "Rectangle::treeview_first_item": {"background_color": 0x0, "border_width": 1, "border_color": 0xFF505050},
        "Rectangle::treeview_first_item:hovered": {
            "background_color": TREEVIEW_SELECTED_COLOR,
            "border_width": 1,
            "border_color": 0xFF505050,
        },
        "Rectangle::treeview_first_item:pressed": {
            "background_color": TREEVIEW_SELECTED_COLOR,
            "border_width": 1,
            "border_color": 0xFF505050,
        },
        "Rectangle::treeview_id": {"margin": 1, "background_color": TREEVIEW_ITEM_COLOR},
        "ScrollingFrame": {"background_color": FRAME_BG_COLOR},
        "ScrollingFrame::treeview": {"background_color": TREEVIEW_BG_COLOR},
        "TreeView": {
            "background_selected_color": TREEVIEW_SELECTED_COLOR,
            "background_color": TREEVIEW_BG_COLOR,
            "secondary_color": TREEVIEW_BG_COLOR,
        },
        "TreeView.Header": {
            "background_color": TREEVIEW_BG_COLOR,
            "font_size": TREEVIEW_ITEM_FONT,
            "font": f"{EXTENSION_FOLDER_PATH}/data/fonts/NVIDIASans_Rg.ttf",
        },
        "TreeView.Header::background": {"margin": 1, "background_color": TREEVIEW_HEADER_BG_COLOR},
        "TreeView:selected": {"background_color": TREEVIEW_SELECTED_COLOR},
    }
    return style
