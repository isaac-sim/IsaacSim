# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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

"""UI constants for the Asset Transformer extension.

Defines colors, fonts, sizes, icon paths, and string literals used throughout
the Asset Transformer UI.
"""

from pathlib import Path

import omni.ui as ui

HEADER_BACKGROUND_COLOR = 0xFF2D2F2F
CONTENT_BACKGROUND_COLOR = 0xFF323434
INNER_BACKGROUND_COLOR = 0xFF454545
DISABLED_BACKGROUND_COLOR = 0xFF3A3A3A
DISABLED_TEXT_COLOR = 0xFF666666
PLACEHOLDER_TEXT_COLOR = 0xFF888888
SECONDARY_TEXT_COLOR = 0xFFAAAAAA

EXECUTE_BUTTON_BACKGROUND = 0xFF3A5C3A
EXECUTE_BUTTON_HOVER_BACKGROUND = 0xFF4A6C4A
EXECUTE_BUTTON_PRESSED_BACKGROUND = 0xFF2A4A2A
EXECUTE_ACCENT_COLOR = 0xFF2E7D32

INDENT_SIZE = 6
TRIANGLE_SIZE = ui.Pixel(12)

HEADER_FONT_SIZE = 18
NORMAL_FONT_SIZE = 16
ACTION_TITLE_FONT_SIZE = 16
SMALL_FONT_SIZE = 14

BORDER_RADIUS = 6.0
SMALL_MARGIN = 4

EXTENSION_PATH = Path(__file__).parent.parent.parent.parent.parent
DATA_PATH = EXTENSION_PATH.joinpath("data")
ICON_PATH = DATA_PATH.joinpath("ui_icons")
FONT_PATH = DATA_PATH.joinpath("fonts")

HEADER_FONT = f"{FONT_PATH}/NVIDIASans_Bd.ttf"
INFO_FONT = f"{FONT_PATH}/NVIDIASans_Rg.ttf"
NORMAL_FONT = f"{FONT_PATH}/NVIDIASans_Md.ttf"

HEADER_TEXT_INPUT = "Choose Input File"
HEADER_TEXT_ACTIONS = "Set Actions"
HEADER_TEXT_REVIEW = "Execute"

AUTOLOAD_LABEL_TEXT = "Load Restructured File"

INPUT_FILE_INFO_TEXT = "Choose between transforming the active file in the Stage or choose an input file from disk."
ACTION_SET_INFO_TEXT = "Actions will define how the USD files are transformed. Actions can be added from presets or individually. When added, an Action can be enabled or disabled with the checkbox. Execution order matter, and the last action should output the file that will be loaded into the Stage."
REVIEW_INFO_TEXT = "Execute the actions to transform the input file and save the output file to the specified location."

ADD_ICON_URL = f"{ICON_PATH}/add.svg"
DRAG_ICON_URL = f"{ICON_PATH}/drag.svg"
REMOVE_ICON_URL = f"{ICON_PATH}/remove_active.svg"
EXECUTE_ICON_URL = f"{ICON_PATH}/execute_all.svg"
FOLDER_ICON_URL = f"{ICON_PATH}/folder.svg"
HELP_ICON_URL = f"{ICON_PATH}/help_active.svg"
INFO_ICON_URL = f"{ICON_PATH}/info_icon.svg"
FILTER_ICON_URL = f"{ICON_PATH}/filter.svg"
SAVE_ICON_URL = f"{ICON_PATH}/save_dark.svg"

SETTING_RECENT_PRESETS = "/persistent/exts/isaacsim.asset.transformer/recent_presets"
MAX_RECENT_PRESETS = 10
