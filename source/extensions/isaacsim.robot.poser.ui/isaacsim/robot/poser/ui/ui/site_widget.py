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

"""Searchable site combo box for the Robot Poser extension."""

from collections.abc import Callable

import omni.ui as ui

from ..style import (
    EXTENSION_FOLDER_PATH,
    LABEL_COLOR,
    LABEL_DISABLED_COLOR,
    TREEVIEW_BG_COLOR,
    TREEVIEW_SELECTED_COLOR,
)


def _short_name(path: str) -> str:
    """Return the last path segment (the prim/joint name).

    Args:
        path: Full prim path.

    Returns:
        Last path segment, or path unchanged if it contains no slash.
    """
    return path.rsplit("/", 1)[-1] if "/" in path else path


class SiteItem(ui.AbstractItem):
    """Single item representing a site path in the searchable dropdown.

    Args:
        path: Full prim path of the site.
    """

    def __init__(self, path: str) -> None:
        super().__init__()
        self._path = path
        self._display_name = _short_name(path)

    @property
    def path(self) -> str:
        """Full prim path of the site.

        Returns:
            Full prim path of the site.
        """
        return self._path

    @property
    def display_name(self) -> str:
        """Short display name (last path segment).

        Returns:
            Short display name (last path segment).
        """
        return self._display_name


class SiteListModel(ui.AbstractItemModel):
    """Flat item model backing the site searchable dropdown.

    Holds all site paths and exposes a filtered view driven by a
    case-insensitive substring query matched against both the short
    display name and the full path.
    """

    def __init__(self) -> None:
        super().__init__()
        self._all_items: list[SiteItem] = []
        self._children: list[SiteItem] = []

    def set_items(self, paths: list[str]) -> None:
        """Replace the full item list with paths and reset the filter.

        Args:
            paths: Full list of site prim path strings.
        """
        self._all_items = [SiteItem(p) for p in paths]
        self._children = list(self._all_items)
        self._item_changed(None)

    def filter_items(self, query: str = "") -> None:
        """Show only items whose display name contains query.

        Args:
            query: Case-insensitive substring to match.
        """
        q = query.lower()
        self._children = [item for item in self._all_items if q in item.display_name.lower()]
        self._item_changed(None)

    def get_item_value(self, item: ui.AbstractItem | None = None, column_id: int = 0) -> str | None:
        """Return the display name for the item.

        Args:
            item: The row item (SiteItem).
            column_id: Column index (unused).

        Returns:
            Display name or None.
        """
        if isinstance(item, SiteItem):
            return item.display_name
        return None

    def get_item_value_model(self, item: ui.AbstractItem | None = None, column_id: int = 0) -> ui.AbstractValueModel:
        """Return the value model for the item and column.

        Args:
            item: The row item.
            column_id: Column index.

        Returns:
            The value model from the parent implementation.
        """
        return super().get_item_value_model(item=item, column_id=column_id)

    def get_item_value_model_count(self, item: ui.AbstractItem | None = None) -> int:
        """Return 1 (single column).

        Args:
            item: The row item (unused).

        Returns:
            1.
        """
        return 1

    def get_item_children(self, item: ui.AbstractItem | None = None) -> list[SiteItem]:
        """Return the filtered list of site items (root children).

        Args:
            item: Parent item; None for root.

        Returns:
            List of SiteItem children.
        """
        if item is not None:
            return []
        return self._children


class SiteItemDelegate(ui.AbstractItemDelegate):
    """Delegate that renders a single-column label for each site item."""

    def build_branch(
        self,
        model: object,
        item: object = None,
        column_id: int = 0,
        level: int = 0,
        expanded: bool = False,
    ) -> None:
        """Required by AbstractItemDelegate; no branch expansion.

        Args:
            model: The item model.
            item: The item.
            column_id: Column index.
            level: Nesting level.
            expanded: Whether expanded.
        """

    def build_widget(
        self,
        model: object,
        item: object = None,
        index: int = 0,
        level: int = 0,
        expanded: bool = False,
    ) -> None:
        """Build a single site label for the TreeView.

        Args:
            model: The item model.
            item: The item (SiteItem).
            index: Row index.
            level: Nesting level.
            expanded: Whether expanded.
        """
        if isinstance(item, SiteItem):
            with ui.HStack(height=22):
                ui.Spacer(width=4)
                ui.Label(item.display_name, tooltip=item.path)


class SiteSearchComboBox:
    """Searchable dropdown for selecting a site path.

    Displays a collapsed label showing the short site name (last path segment).
    Clicking opens a popup with a search field and a single-column TreeView
    listing site names.  The popup uses ``WINDOW_FLAGS_POPUP`` so it
    auto-closes when the user clicks outside.

    The widget stores and returns full prim paths but displays only the
    short name, matching the original ``ui.ComboBox`` behaviour.

    Args:
        items: Initial list of site path strings.
        current_value: Pre-selected full site path.
        on_selection_changed_fn: Callback invoked with the selected full
            path string whenever the user picks a new site.
        identifier: Optional identifier to simplify UI queries.
    """

    def __init__(
        self,
        items: list[str] | None = None,
        current_value: str = "",
        on_selection_changed_fn: Callable[[str], None] | None = None,
        identifier: str | None = None,
    ) -> None:
        self._on_selection_changed_fn = on_selection_changed_fn
        self._delegate = SiteItemDelegate()
        self._current_value = current_value
        self._identifier = identifier

        self._list_model = SiteListModel()
        if items:
            self._list_model.set_items(items)

        self._frame = ui.Frame(height=22, identifier=identifier)
        self._popup = ui.Window(
            f"_site_search_{id(self)}",
            width=100,
            height=200,
            flags=(
                ui.WINDOW_FLAGS_NO_RESIZE
                | ui.WINDOW_FLAGS_NO_SCROLLBAR
                | ui.WINDOW_FLAGS_NO_TITLE_BAR
                | ui.WINDOW_FLAGS_NO_MOVE
                | ui.WINDOW_FLAGS_NO_SAVED_SETTINGS
                | ui.WINDOW_FLAGS_POPUP
            ),
            visible=False,
        )
        self._popup.set_visibility_changed_fn(self._on_popup_visibility_changed)

        self._build_ui()

    # -- public API ---------------------------------------------------------

    def destroy(self) -> None:
        """Hide the popup and release callbacks."""
        self._popup.set_visibility_changed_fn(None)
        self._popup.visible = False

    @property
    def value(self) -> str:
        """Return the currently selected full site path, or empty string."""
        return self._current_value

    @value.setter
    def value(self, v: str) -> None:
        self._current_value = v
        self._update_display_label()

    def set_items(self, items: list[str]) -> None:
        """Replace the available site list.

        Args:
            items: Full list of site path strings.
        """
        self._list_model.set_items(items)

    # -- internal helpers ---------------------------------------------------

    def _get_display_name(self) -> str:
        """Return the short name for the collapsed label.

        Returns:
            Last path segment of current value, or '-- Select Site --' if empty.
        """
        if self._current_value:
            return _short_name(self._current_value)
        return "-- Select Site --"

    def _build_ui(self) -> None:
        """Build the collapsed label and popup window with search and tree view."""
        _arrow_style = {"background_color": LABEL_COLOR}

        # -- collapsed display (inherits parent style via name= attributes) --
        with self._frame:
            with ui.ZStack(width=ui.Percent(100), height=22):
                ui.Rectangle(name="treeview_item")
                with ui.HStack():
                    ui.Spacer(width=6)
                    self._display_label = ui.Label(
                        self._get_display_name(),
                        elided_text=True,
                        tooltip=self._current_value,
                    )
                    with ui.VStack(width=14):
                        ui.Spacer()
                        ui.Triangle(
                            width=ui.Pixel(8),
                            height=ui.Pixel(6),
                            alignment=ui.Alignment.CENTER_BOTTOM,
                            style=_arrow_style,
                        )
                        ui.Spacer()
                    ui.Spacer(width=4)
                ui.InvisibleButton(height=22)
            self._frame.set_mouse_pressed_fn(self._enter_edit_mode)

        # -- popup with search field + tree view -------------------------------
        popup_bg = {"background_color": TREEVIEW_BG_COLOR}
        _placeholder_style = {"color": LABEL_DISABLED_COLOR}
        with self._popup.frame:
            with ui.VStack(
                style=popup_bg,
            ):
                # Search row: field with placeholder overlay + close triangle
                with ui.HStack(height=22, spacing=0):
                    with ui.ZStack():
                        ui.Rectangle(name="search_row")
                        with ui.HStack():
                            ui.Spacer(width=16)
                            self._query_field = ui.StringField(
                                height=22,
                                style={
                                    "background_color": TREEVIEW_BG_COLOR,
                                    "color": LABEL_COLOR,
                                },
                                identifier=f"{self._identifier}_search" if self._identifier else None,
                            )
                            with ui.ZStack(width=16, height=22):
                                with ui.VStack():
                                    ui.Spacer()
                                    with ui.HStack():
                                        ui.Spacer()
                                        ui.Triangle(
                                            width=ui.Pixel(8),
                                            height=ui.Pixel(6),
                                            alignment=ui.Alignment.CENTER_TOP,
                                            style=_arrow_style,
                                        )
                                        ui.Spacer()
                                    ui.Spacer()
                                close_btn = ui.InvisibleButton()
                                close_btn.set_mouse_pressed_fn(
                                    lambda x, y, b, m: self._close_popup() if b == 0 else None
                                )
                        # Placeholder overlay (search icon + "Search" text)
                        self._search_placeholder = ui.HStack(height=22)
                        with self._search_placeholder:
                            ui.Spacer(width=6)
                            with ui.VStack(width=0):
                                ui.Spacer()
                                ui.Image(
                                    f"{EXTENSION_FOLDER_PATH}/icons/search.svg",
                                    width=14,
                                    height=14,
                                    style={"color": LABEL_COLOR},
                                )
                                ui.Spacer()
                            ui.Spacer(width=4)
                            with ui.VStack(width=0):
                                ui.Spacer()
                                ui.Label("Search", style=_placeholder_style, height=0)
                                ui.Spacer()
                            ui.Spacer()

                            ui.Spacer(width=6)
                    # Subscription MUST be stored or it is garbage-collected.
                    self._query_value_sub = self._query_field.model.subscribe_value_changed_fn(
                        lambda _: self._on_query_changed()
                    )
                    # Close triangle (pointing up)
                    # ui.Spacer(width=8)

                    ui.Spacer(width=1)

                with ui.ScrollingFrame(style=popup_bg):
                    self._tree_view = ui.TreeView(
                        self._list_model,
                        delegate=self._delegate,
                        header_visible=False,
                        root_visible=False,
                        style={
                            "TreeView": {
                                "background_color": TREEVIEW_BG_COLOR,
                                "color": LABEL_COLOR,
                            },
                            "TreeView:selected": {"background_color": TREEVIEW_SELECTED_COLOR},
                            "TreeView.Item": {"color": LABEL_COLOR},
                        },
                    )
                    self._tree_view.set_selection_changed_fn(self._on_item_selected)

    # -- popup visibility ----------------------------------------------------

    def _on_popup_visibility_changed(self, visible: bool) -> None:
        """Sync the display label when the popup closes for any reason.

        Args:
            visible: Whether the popup became visible (False when closing).
        """
        if not visible:
            self._update_display_label()

    # -- edit mode -----------------------------------------------------------

    def _enter_edit_mode(self, *args: object) -> None:
        """Open the popup, positioning it over the collapsed label.

        Args:
            *args: Mouse event args (x, y, button, modifier); ignored if button != 0.
        """
        if len(args) >= 3 and args[2] != 0:
            return
        if self._popup.visible:
            return
        self._tree_view.clear_selection()
        self._query_field.model.set_value("")
        self._search_placeholder.visible = True
        self._popup.position_x = self._frame.screen_position_x - 4
        self._popup.position_y = self._frame.screen_position_y - 4
        self._popup.width = self._frame.computed_width + 6
        self._popup.visible = True
        self._on_query_changed()

    def _close_popup(self) -> None:
        """Close the popup and refresh the collapsed display label."""
        self._popup.visible = False
        self._update_display_label()

    def _update_display_label(self) -> None:
        """Update the collapsed label text and tooltip."""
        self._display_label.text = self._get_display_name()
        self._display_label.set_tooltip(self._current_value)

    # -- selection / query ---------------------------------------------------

    def _on_item_selected(self, items: list) -> None:
        """Handle user selecting an item in the TreeView.

        Args:
            items: List of selected items (first is used as the new value).
        """
        if not self._popup.visible:
            return
        if not items:
            return
        selected = items[0]
        if isinstance(selected, SiteItem):
            self._current_value = selected.path
            self._close_popup()
            if self._on_selection_changed_fn:
                self._on_selection_changed_fn(selected.path)

    def _on_query_changed(self) -> None:
        """Re-filter the item list based on the current search text."""
        query = self._query_field.model.get_value_as_string()
        self._search_placeholder.visible = len(query) == 0
        self._list_model.filter_items(query)
        self._query_field.focus_keyboard()
