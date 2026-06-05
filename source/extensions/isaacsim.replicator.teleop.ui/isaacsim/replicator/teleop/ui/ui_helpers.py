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

"""Shared UI constants and helper functions for teleop panels."""

from __future__ import annotations

from collections.abc import Callable

import omni.ui as ui
import omni.usd

# Color constants (0xAABBGGRR for omni.ui)
CLR_GREEN = 0xFF66AA66
CLR_RED = 0xFF6666CC
CLR_YELLOW = 0xFF44CCEE
CLR_DIM = 0xFF888888

# Layout constants (pixels) — single source of truth for all panels
INDENT = 10
ROW_SPACING = 5
ROW_HEIGHT = 22
STATUS_HEIGHT = 16
SECTION_SPACING = 3

GLYPHS = {
    "plus": ui.get_custom_glyph_code("${glyphs}/plus.svg"),
    "delete": ui.get_custom_glyph_code("${glyphs}/menu_delete.svg"),
    "open_folder": ui.get_custom_glyph_code("${glyphs}/folder_open.svg"),
}


# =========================================================================
# Field helpers
# =========================================================================


def get_selected_prim_path() -> str | None:
    """Return the first selected prim path in the stage, or None."""
    selection = omni.usd.get_context().get_selection()
    paths = selection.get_selected_prim_paths()
    if not paths:
        return None
    if len(paths) > 1:
        print(f"[Teleop][UI] Multiple prims selected, using first: {paths[0]}")
    return paths[0]


def set_status(
    label: ui.Label | None,
    text: str,
    color: int = CLR_DIM,
    source: str = "",
    emit_terminal: bool = False,
    side: str | None = None,
) -> None:
    """Set a status label's text and color, and optionally prints the change to terminal.

    Skips redundant updates when the label already shows the same text. When
    ``side`` is provided, the terminal message is tagged ``[Teleop][Source][Side]``
    (e.g. ``[Teleop][Floating][Left] Active``) so per-side logs can be
    distinguished at a glance.
    """
    if label:
        if label.text == text:
            return
        label.text = text
        label.style = {"color": color}
    if text and emit_terminal:
        tag = f"[{source}]" if source else ""
        side_tag = f"[{side.capitalize()}]" if side else ""
        print(f"[Teleop]{tag}{side_tag} {text}")


def build_prim_path_row(
    label: str,
    tooltip: str = "",
    on_apply_clicked: Callable | None = None,
    apply_label: str = "Apply",
    apply_tooltip: str = "Validate and apply this prim path",
    buttons_out: dict | None = None,
    label_width: int = 65,
) -> ui.StringField:
    """Create a prim path row with +/clear and optional apply button.

    Args:
        label: Text for the row label widget.
        tooltip: Tooltip shown on the label widget.
        on_apply_clicked: Optional callback invoked when the Apply button is clicked.
        apply_label: Button label for the apply action.
        apply_tooltip: Tooltip for the apply button.
        buttons_out: If provided, populated with ``"plus"``, ``"delete"``,
            and ``"apply"`` keys referencing the created ``ui.Button`` widgets
            so callers can enable/disable them.
        label_width: Width for the label widget in pixels.
    """
    with ui.HStack(spacing=ROW_SPACING, height=ROW_HEIGHT):
        ui.Spacer(width=INDENT)
        ui.Label(label, width=label_width, tooltip=tooltip)
        field = ui.StringField(width=ui.Fraction(1))
        field.model.set_value("")
        plus_btn = ui.Button(
            f"{GLYPHS['plus']}",
            width=22,
            clicked_fn=lambda f=field: _set_field_from_selection(f.model),
            tooltip="Set from selected prim in stage",
        )
        del_btn = ui.Button(
            f"{GLYPHS['delete']}", width=22, clicked_fn=lambda f=field: f.model.set_value(""), tooltip="Clear field"
        )
        apply_btn = None
        if on_apply_clicked is not None:
            apply_btn = ui.Button(apply_label, width=56, clicked_fn=on_apply_clicked, tooltip=apply_tooltip)
        if buttons_out is not None:
            buttons_out["plus"] = plus_btn
            buttons_out["delete"] = del_btn
            buttons_out["apply"] = apply_btn
    return field


def _set_field_from_selection(model: object) -> None:
    path = get_selected_prim_path()
    if path:
        model.set_value(path)
    else:
        print("[Teleop][UI] No prim selected.")
