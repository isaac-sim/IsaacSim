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

"""TreeView delegate for rendering expandable action rows."""

import omni.ui as ui

from .action_models import ActionListItem, ActionListModel, ActionProtocol
from .constants import DRAG_ICON_URL, REMOVE_ICON_URL, TRIANGLE_SIZE


class ActionRowDelegate(ui.AbstractItemDelegate):
    """Delegate for rendering action rows in the TreeView.

    Each row is expandable -- clicking the triangle toggles expansion to show
    the action's custom configuration UI (drawn by the action instance itself).
    Expansion state is managed by the ``ActionListItem``, not the TreeView.
    """

    def build_branch(
        self,
        model: ui.AbstractItemModel,
        item: ui.AbstractItem | None = None,
        column_id: int = 0,
        level: int = 0,
        expanded: bool = False,
    ) -> None:
        """Build the branch indicator (intentionally empty).

        The triangle is handled in ``build_widget`` for more control over its
        placement and click behavior.

        Args:
            model: The item model owning the item.
            item: The item to build the branch for.
            column_id: Column index.
            level: Nesting depth.
            expanded: Whether the item is expanded.
        """

    def build_widget(
        self,
        model: ui.AbstractItemModel,
        item: ui.AbstractItem | None = None,
        index: int = 0,
        level: int = 0,
        expanded: bool = False,
    ) -> None:
        """Build the widget for an action row.

        When collapsed, shows the header row with drag handle, checkbox,
        triangle, action name, and remove button. When expanded
        (self-managed state), also shows the action's custom UI below the
        header.

        Args:
            model: The ``ActionListModel`` driving the tree view.
            item: The ``ActionListItem`` to render.
            index: Column index.
            level: Nesting depth.
            expanded: Whether the TreeView considers this item expanded.
        """
        assert type(model) is ActionListModel
        assert type(item) is ActionListItem
        action = item.action_model.get_action()
        is_expanded = item.expanded

        with ui.VStack(spacing=0):
            # Top spacer creates a gap where the TreeView draws its
            # built-in drop-between-items indicator line.
            ui.Spacer(height=3)
            with ui.ZStack():
                ui.Rectangle(name="action_row")
                with ui.VStack(name="action_row", spacing=0):
                    self._build_header_row(model, item, action, is_expanded)
                    if hasattr(action, "set_remove_callback"):
                        action.set_remove_callback(lambda m=model, i=item: m.remove_item(i))
                    if hasattr(action, "set_rebuild_callback"):
                        action.set_rebuild_callback(lambda m=model, i=item: m._item_changed(i))
                    if is_expanded:
                        self._build_expanded_content(action)
            ui.Spacer(height=3)

    def _build_header_row(
        self,
        model: ActionListModel,
        item: ActionListItem,
        action: ActionProtocol,
        is_expanded: bool,
    ) -> None:
        """Build the header row with controls and action name.

        Args:
            model: The list model owning the item.
            item: The list item being rendered.
            action: The action instance attached to the item.
            is_expanded: Whether the item is currently expanded.
        """
        with ui.HStack(height=28):
            ui.Spacer(width=2)

            # Drag handle
            with ui.VStack(name="center_content", width=0):
                ui.Spacer()
                ui.Image(
                    DRAG_ICON_URL,
                    height=20,
                    width=20,
                    name="drag_handle",
                )
                ui.Spacer()

            ui.Spacer(width=4)

            # Enabled checkbox
            with ui.VStack(name="center_content", width=0):
                ui.Spacer()
                ui.CheckBox(height=0, name="action_row", model=action.model)
                ui.Spacer()

            ui.Spacer(width=20)

            # Expansion triangle (clickable to toggle)
            with ui.VStack(name="center_content", width=0):
                ui.Spacer()
                triangle = ui.Triangle(
                    width=TRIANGLE_SIZE,
                    height=TRIANGLE_SIZE,
                    name="action_row",
                    alignment=(ui.Alignment.CENTER_BOTTOM if is_expanded else ui.Alignment.RIGHT_CENTER),
                    tooltip="Toggle action options",
                )
                triangle.set_mouse_pressed_fn(
                    lambda x, y, btn, mod, m=model, i=item: self._on_triangle_clicked(btn, m, i)
                )
                ui.Spacer()

            ui.Spacer(width=TRIANGLE_SIZE)

            # Action name — static label when collapsed, editable field
            # when expanded (if the action exposes a name_model).
            # Pale red tint when the rule type is missing from the registry.
            missing = getattr(action, "is_rule_type_missing", False)
            title_style = {"color": 0xFF7777CC} if missing else {}

            if is_expanded and hasattr(action, "name_model"):
                with ui.VStack():
                    ui.Spacer()
                    ui.StringField(
                        model=action.name_model,
                        width=300,
                        height=22,
                        name="action_title",
                        style=title_style,
                    )
                    ui.Spacer()
            else:
                ui.Label(
                    action.name,
                    width=0,
                    name="action_title",
                    style=title_style,
                    mouse_pressed_fn=lambda x, y, btn, mod, m=model, i=item: self._on_triangle_clicked(btn, m, i),
                )

            ui.Spacer()

            # Remove button
            ui.Button(
                image_url=REMOVE_ICON_URL,
                image_width=20,
                image_height=20,
                width=0,
                tooltip="Remove this action",
                clicked_fn=lambda m=model, i=item: m.remove_item(i),
            )
            ui.Spacer(width=2)

    def _build_expanded_content(self, action: ActionProtocol) -> None:
        """Build the expanded content area showing the action's custom UI.

        The action's ``build_ui()`` method is responsible for drawing its own
        configuration interface.

        Args:
            action: The action instance whose UI should be drawn.
        """
        with ui.HStack():
            # Indent the expanded content to align with the action name
            ui.Spacer(width=70)

            with ui.VStack(name="action_row", height=0, spacing=0):
                ui.Spacer(height=4)

                # Let the action draw its own UI
                # Actions should implement build_ui() to provide custom controls
                if hasattr(action, "build_ui") and callable(action.build_ui):
                    action.build_ui()
                else:
                    # Fallback for actions that don't implement build_ui
                    ui.Label(
                        "No configuration available",
                        name="no_config",
                    )

                ui.Spacer(height=8)

            ui.Spacer(width=8)

    def _on_triangle_clicked(self, button: int, model: ActionListModel, item: ActionListItem) -> None:
        """Handle triangle click to toggle expansion.

        Args:
            button: Mouse button index (0 = left).
            model: The list model owning the item.
            item: The list item whose expansion state should toggle.
        """
        if button == 0:  # Left click
            model.toggle_item_expanded(item)
