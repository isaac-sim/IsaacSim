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

"""Anchor column delegate for the Robot Inspector stage widget."""

from __future__ import annotations

from .column_base import MaskingToggleColumnDelegate
from .masking_state import get_selected_anchorable_link_paths, is_anchorable_link
from .style import ANCHOR_IMAGE_STYLE, ANCHOR_IMAGE_TYPE


class AnchorColumnDelegate(MaskingToggleColumnDelegate):
    """Column delegate for anchoring robot links to the world.

    Displays a clickable anchor icon for each link prim. When active,
    a temporary fixed joint pins the link to the world at its current pose.
    Joints are not shown in this column (anchor applies only to links).

    Args:
        icons_dir: Directory containing the column icons (e.g. icoAnchor.svg).
    """

    def __init__(self, icons_dir: str = "") -> None:
        super().__init__(
            icons_dir=icons_dir,
            icon_filename="icoAnchor.svg",
            image_style=ANCHOR_IMAGE_STYLE,
            image_style_type=ANCHOR_IMAGE_TYPE,
            order=-98,
            tooltip="Anchor to World: pin this link (with RigidBodyAPI) to the world at its current pose.",
            header_name="anchor_header",
            cell_name="anchor_cell",
            header_identifier="robot_inspector_anchor_header",
            cell_identifier="robot_inspector_anchor_cell",
            prim_filter=lambda prim: is_anchorable_link(prim),
            is_checked_fn=lambda state, path: state.is_anchored(path),
            get_selected_paths_fn=get_selected_anchorable_link_paths,
            set_batch_fn=lambda state, paths, value: state.set_anchored_batch(paths, value),
            toggle_fn=lambda state, path: state.toggle_anchored(path),
        )
