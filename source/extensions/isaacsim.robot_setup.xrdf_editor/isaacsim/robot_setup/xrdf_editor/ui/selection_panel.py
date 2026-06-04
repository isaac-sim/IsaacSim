# SPDX-FileCopyrightText: Copyright (c) 2022-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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

"""Articulation and link selection combobox panel."""

from __future__ import annotations

from typing import TYPE_CHECKING, Callable

import omni.ui as ui
from isaacsim.gui.components.ui_utils import add_line_rect_flourish, get_style
from isaacsim.gui.components.widgets import DynamicComboBoxModel
from omni.kit.window.property.templates import LABEL_WIDTH

if TYPE_CHECKING:
    from ..editor_state import EditorState


class SelectionPanel:
    """Builds the 'Select Articulation' and 'Select Link' comboboxes."""

    def __init__(
        self,
        state: "EditorState",
        on_articulation_selected: Callable[[str], None],
        on_link_selected: Callable[[str], None],
        find_articulation_paths: Callable[[], list[str]],
    ) -> None:
        self._state = state
        self._on_articulation_selected = on_articulation_selected
        self._on_link_selected = on_link_selected
        self._find_articulation_paths = find_articulation_paths

        self.articulation_list: list[str] = []
        self._articulation_model: DynamicComboBoxModel | None = None
        self._articulation_combobox: ui.ComboBox | None = None

        self._link_model: DynamicComboBoxModel | None = None
        self._link_combobox: ui.ComboBox | None = None

        self._selected_articulation_index: int | None = None
        self._selected_articulation_path: str | None = None

    def build(self) -> None:
        """Build the panel widgets inside the current container."""
        frame = ui.CollapsableFrame(
            title="Selection Panel",
            height=0,
            collapsed=False,
            style=get_style(),
            style_type_name_override="CollapsableFrame",
            horizontal_scrollbar_policy=ui.ScrollBarPolicy.SCROLLBAR_AS_NEEDED,
            vertical_scrollbar_policy=ui.ScrollBarPolicy.SCROLLBAR_ALWAYS_ON,
        )
        with frame:
            with ui.VStack(style=get_style(), spacing=5, height=0):
                self.articulation_list = []
                self._articulation_model = DynamicComboBoxModel(self.articulation_list)
                with ui.HStack():
                    ui.Label(
                        "Select Articulation",
                        width=LABEL_WIDTH,
                        alignment=ui.Alignment.LEFT_CENTER,
                        tooltip="Select Articulation",
                    )
                    self._articulation_combobox = ui.ComboBox(self._articulation_model)
                    add_line_rect_flourish(False)
                self._articulation_combobox.model.add_item_changed_fn(self._on_articulation_combobox)

                self._link_model = DynamicComboBoxModel(list(self._state.link_to_meshes.keys()))
                with ui.HStack():
                    ui.Label(
                        "Select Link",
                        width=LABEL_WIDTH,
                        alignment=ui.Alignment.LEFT_CENTER,
                        tooltip=(
                            "Select under which to generate spheres.  " "Only links with nested meshes can be chosen"
                        ),
                    )
                    self._link_combobox = ui.ComboBox(self._link_model)
                    add_line_rect_flourish(False)
                self._link_combobox.model.add_item_changed_fn(self._on_link_combobox)

    # ------------------------------------------------------------------
    # Public API
    # ------------------------------------------------------------------
    def refresh_articulations(self) -> None:
        """Rescan the stage for articulations and repopulate the combobox."""
        if self._articulation_combobox is None:
            return

        self.articulation_list = self._find_articulation_paths()
        self.articulation_list.insert(0, "None")
        self._articulation_model = DynamicComboBoxModel(self.articulation_list)
        self._articulation_combobox.model = self._articulation_model
        self._articulation_combobox.model.add_item_changed_fn(self._on_articulation_combobox)

        # Re-derive the selection index from the cached path so a reordered
        # articulation list (prims added/removed/renamed) does not silently
        # select a different articulation.
        if self._selected_articulation_path is not None and self._selected_articulation_path in self.articulation_list:
            restored_index = self.articulation_list.index(self._selected_articulation_path)
            self._selected_articulation_index = restored_index
            self._articulation_combobox.model.set_item_value_model(ui.SimpleIntModel(restored_index))

    def clear_articulations(self) -> None:
        """Reset both comboboxes to empty states."""
        self._selected_articulation_index = None
        self._selected_articulation_path = None
        self.articulation_list = []
        if self._articulation_combobox is not None:
            self._articulation_model = DynamicComboBoxModel(self.articulation_list)
            self._articulation_combobox.model = self._articulation_model
            self._articulation_combobox.model.add_item_changed_fn(self._on_articulation_combobox)
        self.clear_links()

    def refresh_links(self) -> None:
        """Repopulate the link combobox from the current ``state.link_to_meshes``."""
        if self._link_combobox is None:
            return
        self._link_model = DynamicComboBoxModel(list(self._state.link_to_meshes.keys()))
        self._link_combobox.model = self._link_model
        self._link_combobox.model.add_item_changed_fn(self._on_link_combobox)
        # Fire the link callback to keep dependent panels in sync.
        self._on_link_combobox(self._link_model, None)

    def clear_links(self) -> None:
        """Empty the link combobox."""
        if self._link_combobox is None:
            return
        self._link_model = DynamicComboBoxModel([])
        self._link_combobox.model = self._link_model
        self._link_combobox.model.add_item_changed_fn(self._on_link_combobox)

    def get_selected_articulation_path(self) -> str:
        """Return the prim path currently selected in the articulation combobox."""
        if self._articulation_model is None:
            return "None"
        index = self._articulation_model.get_item_value_model().as_int
        if index < 0 or index >= len(self.articulation_list):
            return "None"
        return self.articulation_list[index]

    def get_selected_link_index(self) -> int:
        """Return the currently-selected index in the link combobox (or 0)."""
        if self._link_model is None:
            return 0
        return self._link_model.get_item_value_model().as_int

    def get_selected_link_name(self) -> str | None:
        """Return the currently-selected link subpath, or ``None`` if no selection exists."""
        keys = list(self._state.link_to_meshes.keys())
        if not keys:
            return None
        idx = self.get_selected_link_index()
        if 0 <= idx < len(keys):
            return keys[idx]
        return None

    # ------------------------------------------------------------------
    # Callbacks
    # ------------------------------------------------------------------
    def _on_articulation_combobox(self, model=None, val=None) -> None:
        if self._articulation_model is None:
            return
        index = self._articulation_model.get_item_value_model().as_int
        if 0 <= index < len(self.articulation_list):
            self._selected_articulation_index = index
            self._selected_articulation_path = self.articulation_list[index]
            self._on_articulation_selected(self.articulation_list[index])

    def _on_link_combobox(self, model=None, val=None) -> None:
        link_name = self.get_selected_link_name()
        if link_name is None:
            return
        self._on_link_selected(link_name)
