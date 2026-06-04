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

"""Base column delegate for masking toggle columns (deactivate, bypass, anchor)."""

from __future__ import annotations

from collections.abc import Callable
from typing import Any

import omni.ui as ui
import omni.usd
from omni.kit.widget.stage import AbstractStageColumnDelegate, StageColumnItem
from pxr import Sdf

from .masking_state import MaskingState


class MaskingToggleColumnDelegate(AbstractStageColumnDelegate):
    """Base delegate for a 24px icon column that toggles a masking state per prim.

    Subclasses supply icon path, style dict, style type name, order, prim filter,
    state checker, selection getter, and batch/single toggle methods.

    Args:
        icons_dir: Directory containing the column icon SVG.
        icon_filename: Filename of the header/cell icon.
        image_style: Style dict for the image (e.g. BYPASS_IMAGE_STYLE).
        image_style_type: Style type name for the image (e.g. BYPASS_IMAGE_TYPE).
        order: Column order value (lower appears first).
        tooltip: Tooltip text for the column.
        header_name: Style name for the header image.
        cell_name: Style name for the cell image.
        header_identifier: UI test identifier for the header image widget.
        cell_identifier: UI test identifier for the cell image widget.
        prim_filter: Callable(prim) returning True if the prim shows the toggle.
        is_checked_fn: Callable(state, path) returning True if path is active.
        get_selected_paths_fn: Callable returning list of selected original paths.
        set_batch_fn: Callable(state, paths, value) to set state for multiple paths.
        toggle_fn: Callable(state, path) to toggle state for a single path; returns new state.
    """

    def __init__(
        self,
        *,
        icons_dir: str = "",
        icon_filename: str = "",
        image_style: dict,
        image_style_type: str,
        order: int,
        tooltip: str = "",
        header_name: str = "cell_header",
        cell_name: str = "cell",
        header_identifier: str = "",
        cell_identifier: str = "",
        prim_filter: Callable[[Any], bool],
        is_checked_fn: Callable[[MaskingState, str], bool],
        get_selected_paths_fn: Callable[[], list[str]],
        set_batch_fn: Callable[[MaskingState, list[str], bool], None],
        toggle_fn: Callable[[MaskingState, str], bool],
    ) -> None:
        super().__init__()
        self._masking_state: MaskingState | None = MaskingState.get_instance()
        self._icon = f"{icons_dir}/{icon_filename}" if icons_dir and icon_filename else ""
        self._image_style = image_style
        self._image_style_type = image_style_type
        self._order = order
        self._tooltip = tooltip
        self._header_name = header_name
        self._cell_name = cell_name
        self._header_identifier = header_identifier
        self._cell_identifier = cell_identifier
        self._prim_filter = prim_filter
        self._is_checked_fn = is_checked_fn
        self._get_selected_paths_fn = get_selected_paths_fn
        self._set_batch_fn = set_batch_fn
        self._toggle_fn = toggle_fn

    def destroy(self) -> None:
        """Release references when the column delegate is destroyed."""
        self._masking_state = None

    @property
    def initial_width(self) -> ui.Pixel:
        """Return the initial column width (24px)."""
        return ui.Pixel(24)

    @property
    def minimum_width(self) -> ui.Pixel:
        """Return the minimum column width (24px)."""
        return ui.Pixel(24)

    @property
    def resizable(self) -> bool:
        """Return False; column is not resizable."""
        return False

    @property
    def sortable(self) -> bool:
        """Return False; column is not sortable."""
        return False

    @property
    def order(self) -> int:
        """Return the column order value."""
        return self._order

    def build_header(self, **kwargs: Any) -> None:
        """Build the column header with icon and tooltip.

        Args:
            **kwargs: Additional keyword arguments forwarded from the column framework.
        """
        with ui.HStack(style=self._image_style):
            ui.Spacer()
            with ui.VStack(width=0):
                ui.Spacer()
                img_kwargs = {
                    "width": 18,
                    "height": 18,
                    "name": self._header_name,
                    "style_type_name_override": self._image_style_type,
                }
                if self._tooltip:
                    img_kwargs["tooltip"] = self._tooltip
                if self._header_identifier:
                    img_kwargs["identifier"] = self._header_identifier
                ui.Image(self._icon or "", **img_kwargs)
                ui.Spacer()
            ui.Spacer()

    async def build_widget(self, item: StageColumnItem | None, **kwargs: Any) -> None:
        """Build the cell widget (icon with toggle) for a stage item.

        Args:
            item: Stage column item; None skips build.
            **kwargs: Additional arguments; may include stage_item.
        """
        stage_item = kwargs.get("stage_item", None)
        if not item or not stage_item or not stage_item.prim:
            return

        prim = stage_item.prim
        if not self._prim_filter(prim):
            return

        path_map = self._masking_state.path_map if self._masking_state else None
        if not path_map:
            return

        hierarchy_path = Sdf.Path(str(item.path))
        original_path = path_map.get_original_path(hierarchy_path)
        if not original_path:
            return

        original_path_str = original_path.pathString
        is_checked = self._is_checked_fn(self._masking_state, original_path_str) if self._masking_state else False

        with ui.HStack(height=20, style=self._image_style):
            ui.Spacer()
            cell_kwargs = {
                "width": 18,
                "height": 18,
                "name": self._cell_name,
                "style_type_name_override": self._image_style_type,
                "checked": is_checked,
            }
            if self._cell_identifier:
                cell_kwargs["identifier"] = self._cell_identifier
            img = ui.Image(self._icon or "", **cell_kwargs)
            img.set_mouse_pressed_fn(lambda x, y, b, m, p=original_path_str: self._on_clicked(b, p))
            ui.Spacer()

    def _on_clicked(self, button: int, original_path: str) -> None:
        """Handle cell click: toggle single path or batch if multiple selected.

        Preserves the current USD selection across the masking operation so
        that model refreshes triggered by the state change do not clear it.

        Args:
            button: Mouse button (only 0 is handled).
            original_path: Original stage prim path for the clicked cell.
        """
        if button != 0 or not self._masking_state:
            return

        usd_context = omni.usd.get_context()
        preserved_selection = list(usd_context.get_selection().get_selected_prim_paths())

        new_state = not self._is_checked_fn(self._masking_state, original_path)
        selected_paths = self._get_selected_paths_fn()
        if original_path in selected_paths and len(selected_paths) > 1:
            self._set_batch_fn(self._masking_state, selected_paths, new_state)
        else:
            self._toggle_fn(self._masking_state, original_path)

        if preserved_selection:
            usd_context.get_selection().set_selected_prim_paths(preserved_selection, True)
