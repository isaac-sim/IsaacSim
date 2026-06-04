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

"""Scrollable frame that hosts the drag-and-drop action TreeView."""

from typing import Any

import omni.ui as ui

from .action_widget import ActionRowDelegate


class ActionListFrame(ui.Frame):
    """Scrollable frame containing the drag-and-drop action list TreeView.

    Wraps an ``ActionListModel`` inside a ``TreeView`` with an
    ``ActionRowDelegate`` and disables persistent selection so that rows
    are never highlighted after interaction.

    Args:
        model: The ``ActionListModel`` driving the tree view.
        *args: Additional positional arguments forwarded to ``ui.Frame``.
        **kwargs: Additional keyword arguments forwarded to ``ui.Frame``.
    """

    def __init__(self, model: ui.AbstractItemModel, *args: Any, **kwargs: Any) -> None:
        super().__init__(*args, height=0, **kwargs)
        self.__list_model = model
        self.__delegate = ActionRowDelegate()

        with ui.ScrollingFrame(
            height=ui.Pixel(300),
            vertical_scrollbar_policy=ui.ScrollBarPolicy.SCROLLBAR_AS_NEEDED,
            horizontal_scrollbar_policy=ui.ScrollBarPolicy.SCROLLBAR_ALWAYS_OFF,
        ):
            with ui.ZStack(height=0):
                ui.Rectangle(name="list_background")
                self.__tree_view = ui.TreeView(
                    self.__list_model,
                    root_visible=False,
                    header_visible=False,
                    drop_between_items=True,
                    delegate=self.__delegate,
                    name="action_list",
                )
        self.__list_model.add_item_changed_fn(self._on_item_changed)
        self.__tree_view.set_selection_changed_fn(self._on_selection_changed)

    def _on_item_changed(self, model: ui.AbstractItemModel, item: ui.AbstractItem | None) -> None:
        """Clear tree-view selection whenever the model changes.

        Args:
            model: The item model that changed.
            item: The specific item that changed, or None.
        """
        self.__tree_view.selection = []

    def _on_selection_changed(self, selection: list) -> None:
        """Immediately deselect any row the user clicks on.

        Args:
            selection: List of currently selected items.
        """
        if selection:
            self.__tree_view.selection = []
