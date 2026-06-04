# SPDX-FileCopyrightText: Copyright (c) 2025-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""Shared layout constants and a small status-label helper.

Kept self-contained so the extension has no runtime dependency on teleop
UI helpers — `isaacsim.replicator.teleop.ui` previously owned an equivalent
module but was coupled to teleop-specific logging tags.
"""

from __future__ import annotations

import os

import carb
import omni.ui as ui
from omni.kit.window.extensions.utils import open_file_using_os_default

CLR_GREEN = 0xFF66AA66
CLR_RED = 0xFF6666CC
CLR_YELLOW = 0xFF44CCEE
CLR_DIM = 0xFF888888

INDENT = 10
ROW_SPACING = 5
ROW_HEIGHT = 22
STATUS_HEIGHT = 16
SECTION_SPACING = 3

GLYPHS = {
    "open_folder": ui.get_custom_glyph_code("${glyphs}/folder_open.svg"),
    "timeline_prev": ui.get_custom_glyph_code("${glyphs}/timeline_prev.svg"),
    "timeline_next": ui.get_custom_glyph_code("${glyphs}/timeline_next.svg"),
    "timeline_play": ui.get_custom_glyph_code("${glyphs}/timeline_play.svg"),
    "timeline_pause": ui.get_custom_glyph_code("${glyphs}/timeline_pause.svg"),
    "timeline_stop": ui.get_custom_glyph_code("${glyphs}/timeline_stop.svg"),
}

_LOG_TAG = "[EpisodeRecorder][UI]"


def open_dir(path: str | None) -> None:
    """Open ``path`` (or its parent, if it's a file) in the OS file explorer.

    Warns via ``carb.log_warn`` when the resolved path does not exist on disk.
    """
    if not path:
        carb.log_warn(f"{_LOG_TAG} open_dir called with empty path.")
        return
    target = path if os.path.isdir(path) else os.path.dirname(path)
    if not target or not os.path.isdir(target):
        carb.log_warn(f"{_LOG_TAG} Could not open directory: {path!r}")
        return
    open_file_using_os_default(target)


def set_status(
    label: ui.Label | None,
    text: str,
    color: int = CLR_DIM,
    *,
    emit_terminal: bool = False,
) -> None:
    """Set a status label's text / color, optionally echoing to the terminal.

    Skips redundant updates when the label already shows the same text.
    """
    if label is not None:
        if label.text == text:
            return
        label.text = text
        label.style = {"color": color}
    if text and emit_terminal:
        print(f"{_LOG_TAG} {text}")
