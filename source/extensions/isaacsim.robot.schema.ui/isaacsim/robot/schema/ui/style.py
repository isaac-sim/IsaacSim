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

"""Omni UI stylesheet for robot schema UI (bypass column, etc.). Colors in 0xAABBGGRR."""

# 0xAABBGGRR
_RADIO_LABEL_COLOR = 0xFFD8D8D8
_BYPASS_HEADER_COLOR = 0xFFFFFFFF  # white
_BYPASS_DEACTIVATED_COLOR = 0xFF444444  # 40% gray
_BYPASS_ACTIVE_COLOR = 0xFF66B3FF  # pastel orange (R=255, G=179, B=102)

# 0xAABBGGRR – cancel column active is the original reddish #CB6A6A
_CANCEL_HEADER_COLOR = 0xFFFFFFFF  # white
_CANCEL_DEACTIVATED_COLOR = 0xFF444444  # 40% gray
_CANCEL_ACTIVE_COLOR = 0xFF6A6ACB  # reddish pink (#CB6A6A)

# Custom type names – no global stylesheet uses these, so no conflicts.
# Apply with style_type_name_override on each Image widget and cascade the
# style dict from the parent container so the selectors resolve.
BYPASS_IMAGE_TYPE = "BypassImage"

BYPASS_IMAGE_STYLE = {
    "BypassImage::bypass_header": {"color": _BYPASS_HEADER_COLOR},
    "BypassImage::bypass_cell": {"color": _BYPASS_DEACTIVATED_COLOR},
    "BypassImage::bypass_cell:checked": {"color": _BYPASS_ACTIVE_COLOR},
}

CANCEL_IMAGE_TYPE = "CancelImage"

CANCEL_IMAGE_STYLE = {
    "CancelImage::cancel_header": {"color": _CANCEL_HEADER_COLOR},
    "CancelImage::cancel_cell": {"color": _CANCEL_DEACTIVATED_COLOR},
    "CancelImage::cancel_cell:checked": {"color": _CANCEL_ACTIVE_COLOR},
}

# 0xAABBGGRR – anchor column: 80% dark blue (#2B5EA7 at 80% alpha)
_ANCHOR_HEADER_COLOR = 0xFFFFFFFF  # white
_ANCHOR_DEACTIVATED_COLOR = 0xFF444444  # 25% gray
_ANCHOR_ACTIVE_COLOR = 0xFFDDDDDD  # light gray

ANCHOR_IMAGE_TYPE = "AnchorImage"

ANCHOR_IMAGE_STYLE = {
    "AnchorImage::anchor_header": {"color": _ANCHOR_HEADER_COLOR},
    "AnchorImage::anchor_cell": {"color": _ANCHOR_DEACTIVATED_COLOR},
    "AnchorImage::anchor_cell:checked": {"color": _ANCHOR_ACTIVE_COLOR},
}


def get_view_mode_radio_style(icons_dir: str) -> dict:
    """Return style dict for view-mode radio options (Flat / Tree / MuJoCo).

    Uses `radio_off`, `radio_on`, and `radio_on_hover` from `icons_dir`.
    Unselected hover shows `radio_on_hover`; selected state unchanged.

    Args:
        icons_dir: Directory path containing radio SVG icons.

    Returns:
        Style dictionary for RadioButton and Image selectors.
    """
    base = f"{icons_dir}/radio_off.svg" if icons_dir else ""
    on_url = f"{icons_dir}/radio_on.svg" if icons_dir else ""
    on_hover_url = f"{icons_dir}/radio_on_hover.svg" if icons_dir else ""
    return {
        "RadioButton": {"background_color": 0x0, "padding": 0},
        "RadioButton:checked": {"background_color": 0x0, "padding": 0},
        "RadioButton:hovered": {"background_color": 0x0, "padding": 0},
        "RadioButton.Image": {"image_url": base, "color": _RADIO_LABEL_COLOR},
        "RadioButton.Image:hovered": {"image_url": on_hover_url, "color": _RADIO_LABEL_COLOR},
        "RadioButton.Image:checked": {"image_url": on_url, "color": _RADIO_LABEL_COLOR},
        "RadioButton.Image:checked:hovered": {"image_url": on_url, "color": _RADIO_LABEL_COLOR},
        "RadioButton:pressed": {"background_color": 0x0},
    }
