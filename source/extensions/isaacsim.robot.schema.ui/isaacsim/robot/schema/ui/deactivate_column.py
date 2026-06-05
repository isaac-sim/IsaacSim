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

"""Deactivate column delegate for the Robot Inspector stage widget."""

from __future__ import annotations

from .column_base import MaskingToggleColumnDelegate
from .masking_state import get_selected_maskable_paths, is_maskable_type
from .style import CANCEL_IMAGE_STYLE, CANCEL_IMAGE_TYPE


class DeactivateColumnDelegate(MaskingToggleColumnDelegate):
    """Column delegate for toggling prim deactivation (robot masking).

    Displays a clickable cancel icon for each maskable prim (joints and links).
    The icon toggles between active and deactivated states.

    Args:
        icons_dir: Directory containing the column icons (e.g. icoCancelHeader.svg).
    """

    def __init__(self, icons_dir: str = "") -> None:
        super().__init__(
            icons_dir=icons_dir,
            icon_filename="icoCancelHeader.svg",
            image_style=CANCEL_IMAGE_STYLE,
            image_style_type=CANCEL_IMAGE_TYPE,
            order=-100,
            tooltip="Deactivate: disable this joint or link for simulation.",
            header_name="cancel_header",
            cell_name="cancel_cell",
            header_identifier="robot_inspector_deactivate_header",
            cell_identifier="robot_inspector_deactivate_cell",
            prim_filter=lambda prim: is_maskable_type(prim),
            is_checked_fn=lambda state, path: state.is_deactivated(path),
            get_selected_paths_fn=get_selected_maskable_paths,
            set_batch_fn=lambda state, paths, value: state.set_deactivated_batch(paths, value),
            toggle_fn=lambda state, path: state.toggle_deactivated(path),
        )
