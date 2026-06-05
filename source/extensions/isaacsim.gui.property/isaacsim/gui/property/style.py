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

"""Shared style constants for Isaac Robot Schema property widgets."""

from omni.ui import color as cl

# --- Layout constants ---
ROW_HEIGHT = 24
TABLE_ROW_HEIGHT = 26
TABLE_ROW_GAP = 4
MAX_VISIBLE_ROWS = 8
LIST_FIXED_HEIGHT = TABLE_ROW_GAP + MAX_VISIBLE_ROWS * (TABLE_ROW_HEIGHT + TABLE_ROW_GAP)
LIST_FONT_SIZE = 14
FIELD_LABEL_WIDTH = 120

# --- Colors ---
# Base palette derived from the Kit dark theme defaults.

BG_PANEL = cl("#323434")
"""Main panel background (content area behind fields)."""

BG_HEADER = cl("#2D2F2F")
"""CollapsableFrame header / section header background."""

BG_HEADER_HOVER = cl("#3E4040")
"""CollapsableFrame header hover highlight."""

BG_INPUT = cl("#1F2124")
"""Text field / combo box input background (Kit dark-theme input default)."""

CELL_BG = cl("#2A2C2C")
"""List row cell background."""

SELECTED_BG = cl("#424A4B")
"""Selected row highlight."""

BUTTON_BG = cl("#2D2D2D")
"""Generic small button background (Add Joint / Add Link)."""

TEXT_PRIMARY = cl("#D8D8D8")
"""Primary text / label color."""

TEXT_DIM = cl("#9E9E9E")
"""Dimmed text (indices, hints, column headers)."""

TEXT_HINT = cl("#777775")
"""Hint text in the joint inspector."""

TEXT_JOINT_NAME = cl("#C6C6C6")
"""Joint name labels in the inspector table."""

SLIDER_TEXT = cl("#E0E0E0")
"""Slider value text color."""

DROP_INDICATOR_COLOR = cl("#4A90D9")
"""Drag-and-drop insertion indicator."""

DROP_INDICATOR_TRANSPARENT = cl(0, 0, 0, 0)
"""Invisible drop indicator (default state)."""

GRIP_COLOR = cl("#474747")
"""Drag handle grip bar color."""

# --- Inspector column colors ---
INSPECTOR_COLUMN_COLORS = {
    "States Position": cl("#1D5AA5"),
    "Drives Position": cl("#144683"),
    "Drives Velocities": cl("#B03B60"),
    "Limits (Min, Max)": cl("#006600"),
    "Stiffness | Damping": cl("#006600"),
}

INSPECTOR_DEFAULT_SLIDER_COLOR = cl("#1D5AA5")
"""Fallback slider fill for left inspector columns."""

INSPECTOR_DEFAULT_DUAL_COLOR = cl("#006600")
"""Fallback slider fill for right (dual) inspector columns."""

# --- Composite styles ---
COMBO_STYLE = {
    "ComboBox": {
        "color": TEXT_PRIMARY,
        "background_color": BG_INPUT,
        "secondary_color": BG_INPUT,
        "font_size": 12,
        "border_radius": 2,
    }
}

TOOLTIP_STYLE = {
    "Tooltip": {
        "color": 0xFF333333,
        "background_color": 0xFFD1F7FF,
        "border_radius": 1.5,
        "padding": 2,
    }
}
"""Shared dark-text-on-light-cyan tooltip style for Isaac property widgets.

Both ``color`` and ``background_color`` are explicit so the tooltip renders the
same regardless of the active Kit theme. Mirrors
``isaacsim.gui.components.style`` ``TOOLTIP_STYLE`` and the Robot Schema
property panel's tooltip styling so all Isaac inspectors stay visually consistent.
"""

COLLAPSABLE_FRAME_STYLE = {
    "CollapsableFrame": {
        "background_color": BG_HEADER,
        "secondary_color": BG_HEADER,
        "border_radius": 4,
    },
    "CollapsableFrame:hovered": {
        "secondary_color": BG_HEADER_HOVER,
    },
    "CollapsableFrame.Header": {
        "color": TEXT_PRIMARY,
        "font_size": 13,
    },
    "CollapsableFrame.Header:hovered": {
        "color": TEXT_PRIMARY,
    },
}

# --- Inspector layout constants ---
INSPECTOR_ROW_HEIGHT = 22
INSPECTOR_ROW_SPACING = 8
INSPECTOR_HEADER_HEIGHT = 24
INSPECTOR_JOINT_COL_WIDTH = 150
