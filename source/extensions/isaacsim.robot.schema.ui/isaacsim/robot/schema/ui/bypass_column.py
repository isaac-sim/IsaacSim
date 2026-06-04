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

"""Bypass column delegate for the Robot Inspector stage widget."""

from __future__ import annotations

from .column_base import MaskingToggleColumnDelegate
from .masking_state import get_selected_maskable_paths, is_maskable_type
from .style import BYPASS_IMAGE_STYLE, BYPASS_IMAGE_TYPE


class BypassColumnDelegate(MaskingToggleColumnDelegate):
    """Column delegate for toggling bypass on maskable prims.

    Displays a clickable bypass icon (arrow routing around a circle) for
    each maskable prim. The icon switches between active (orange) and idle (dim).

    Args:
        icons_dir: Directory containing the column icons (e.g. icoBypass.svg).
    """

    def __init__(self, icons_dir: str = "") -> None:
        super().__init__(
            icons_dir=icons_dir,
            icon_filename="icoBypass.svg",
            image_style=BYPASS_IMAGE_STYLE,
            image_style_type=BYPASS_IMAGE_TYPE,
            order=-99,
            tooltip="Bypass: disable and reconnect the kinematic chain around this prim.",
            header_name="bypass_header",
            cell_name="bypass_cell",
            header_identifier="robot_inspector_bypass_header",
            cell_identifier="robot_inspector_bypass_cell",
            prim_filter=lambda prim: is_maskable_type(prim),
            is_checked_fn=lambda state, path: state.is_bypassed(path),
            get_selected_paths_fn=get_selected_maskable_paths,
            set_batch_fn=lambda state, paths, value: state.set_bypassed_batch(paths, value),
            toggle_fn=lambda state, path: state.toggle_bypassed(path),
        )
