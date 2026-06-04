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

"""Data models, protocols, and item classes for the action list TreeView."""

from __future__ import annotations

from collections.abc import Callable
from typing import Protocol, runtime_checkable

import carb
import omni.ui as ui
from isaacsim.asset.transformer import RuleConfigurationParam, RuleRegistry, RuleSpec
from pxr import Usd

from .constants import ADD_ICON_URL, FILTER_ICON_URL, REMOVE_ICON_URL, TRIANGLE_SIZE

# ---------------------------------------------------------------------------
# Rule Type Searchable Dropdown
# ---------------------------------------------------------------------------


def _extract_package_from_fqcn(fqcn: str) -> str:
    """Derive the package category from a fully-qualified rule class name.

    Extracts the sub-category segment following a known sub-package marker
    such as ``.rules.``, ``.impl.``, or ``.scripts.``.  For example,
    ``isaacsim.asset.transformer.rules.core.prims.PrimRoutingRule`` yields
    ``core``.  Falls back to the full module path when no marker is found.

    Args:
        fqcn: Fully-qualified class name (e.g.
            ``isaacsim.asset.transformer.rules.perf.materials.MaterialsRoutingRule``).

    Returns:
        The sub-category name (e.g. ``core``, ``perf``, ``structure``, ``isaac_sim``).
    """
    parts = fqcn.rsplit(".", 1)
    if len(parts) < 2:
        return fqcn
    module_path = parts[0]
    for marker in (".rules.", ".impl.", ".scripts."):
        idx = module_path.find(marker)
        if idx != -1:
            remainder = module_path[idx + len(marker) :]
            # First segment after the marker is the sub-category
            sub_parts = remainder.split(".", 1)
            if sub_parts and sub_parts[0]:
                return sub_parts[0]
            return module_path[:idx]
    return module_path


class RuleTypeItem(ui.AbstractItem):
    """Single item representing a registered rule type in the searchable dropdown.

    Args:
        rule_type: Fully-qualified class name of the rule.
    """

    def __init__(self, rule_type: str) -> None:
        super().__init__()
        self._rule_type = rule_type
        self._rule_name = rule_type.rsplit(".", 1)[-1]
        self._package = _extract_package_from_fqcn(rule_type)

    @property
    def rule_type(self) -> str:
        """Fully-qualified class name of the rule.

        Returns:
            The rule type class name.
        """
        return self._rule_type

    @property
    def rule_name(self) -> str:
        """Short display name derived from the class name.

        Returns:
            The rule name.
        """
        return self._rule_name

    @rule_name.setter
    def rule_name(self, value: str) -> None:
        self._rule_name = value

    @property
    def package(self) -> str:
        """Extension/package name that provides this rule.

        Returns:
            The package name.
        """
        return self._package

    def get_value(self, column_id: int) -> str:
        """Return the display value for the given column.

        Args:
            column_id: 0 for rule name, 1 for package.

        Returns:
            The display string for the column.
        """
        if column_id == 0:
            return self._rule_name
        if column_id == 1:
            return self._package
        return ""


class RuleTypeListModel(ui.AbstractItemModel):
    """Item model backing the rule-type searchable dropdown.

    Args:
        registry: Rule registry used to enumerate available rule types.
    """

    def __init__(self, registry: RuleRegistry) -> None:
        super().__init__()
        self._selection: RuleTypeItem | None = None
        self._all_items: list[RuleTypeItem] = []
        self._children: list[RuleTypeItem] = []

        rules = registry.list_rules()
        for rule_type in sorted(rules.keys()):
            self._all_items.append(RuleTypeItem(rule_type))
        self._children = list(self._all_items)
        self._packages: list[str] = sorted({item.package for item in self._all_items})

    # -- selection ----------------------------------------------------------

    @property
    def selection(self) -> RuleTypeItem | None:
        """Currently selected rule-type item, or None.

        Returns:
            The selected rule-type item, or None if no selection.
        """
        return self._selection

    @selection.setter
    def selection(self, sel: RuleTypeItem | None) -> None:
        self._selection = sel
        self._item_changed(None)

    def set_selection_by_type(self, rule_type: str) -> None:
        """Select the item whose FQCN matches *rule_type*.

        If the type is not found in the registry a placeholder item marked
        as missing is created and selected.

        Args:
            rule_type: Fully-qualified class name to select.
        """
        for item in self._all_items:
            if item.rule_type == rule_type:
                self._selection = item
                self._item_changed(None)
                return
        # Not in registry — create a placeholder marked as missing.
        if rule_type:
            missing = RuleTypeItem(rule_type)
            missing.rule_name = f"{rule_type.rsplit('.', 1)[-1]} (missing)"
            self._selection = missing
            self._item_changed(None)

    # -- filtering ----------------------------------------------------------

    def filter_items(self, query: str = "", package_filter: list[str] | None = None) -> None:
        """Apply a text query and optional package filter to the visible items.

        Args:
            query: Case-insensitive substring to match against rule names.
            package_filter: If provided, only items whose package is in this
                list are shown.
        """
        self._children = [
            item
            for item in self._all_items
            if query.lower() in item.rule_name.lower() and (not package_filter or item.package in package_filter)
        ]
        self._item_changed(None)

    # -- AbstractItemModel overrides ----------------------------------------

    def get_item_value(self, item: ui.AbstractItem | None = None, column_id: int = 0) -> str | None:
        """Return the display value for a given item and column.

        Args:
            item: The ``RuleTypeItem`` to query.
            column_id: Column index.

        Returns:
            The string value for the column, or None if item is not a
            ``RuleTypeItem``.
        """
        if item is not None and isinstance(item, RuleTypeItem):
            return item.get_value(column_id)
        return None

    def get_item_value_model(self, item: ui.AbstractItem | None = None, column_id: int = 0) -> ui.AbstractValueModel:
        """Return the value model for the given item and column.

        Args:
            item: The item to query.
            column_id: Column index.

        Returns:
            The value model provided by the parent class.
        """
        return super().get_item_value_model(item=item, column_id=column_id)

    def get_item_value_model_count(self, item: ui.AbstractItem | None = None) -> int:
        """Return the number of columns (always 2: name and package).

        Args:
            item: Unused.

        Returns:
            The column count.
        """
        return 2

    def get_item_children(self, item: ui.AbstractItem | None = None) -> list[RuleTypeItem]:
        """Return the visible child items.

        Args:
            item: Parent item. Only the root (None) has children.

        Returns:
            List of currently visible ``RuleTypeItem`` instances.
        """
        if item is not None:
            return []
        return self._children

    def get_packages(self) -> list[str]:
        """Return the sorted list of unique package names.

        Returns:
            Sorted list of package name strings.
        """
        return self._packages


class RuleTypeItemDelegate(ui.AbstractItemDelegate):
    """Delegate that renders rule-type rows with *Rule Name* and *Package* columns."""

    _HEADERS = ["Rule Name", "Package"]
    """Column header labels for the rule type dropdown TreeView."""

    def build_branch(
        self,
        model: ui.AbstractItemModel,
        item: ui.AbstractItem | None = None,
        column_id: int = 0,
        level: int = 0,
        expanded: bool = False,
    ) -> None:
        """Build branch indicator (intentionally empty).

        Args:
            model: The item model.
            item: The item to build the branch for.
            column_id: Column index.
            level: Nesting depth.
            expanded: Whether the item is expanded.
        """

    def build_header(self, column_id: int = 0) -> None:
        """Build the column header label.

        Args:
            column_id: Column index.
        """
        ui.Label(self._HEADERS[column_id] if column_id < len(self._HEADERS) else "")

    def build_widget(
        self,
        model: ui.AbstractItemModel,
        item: ui.AbstractItem | None = None,
        index: int = 0,
        level: int = 0,
        expanded: bool = False,
    ) -> None:
        """Build a single cell widget displaying the item's text.

        Args:
            model: The item model.
            item: The item to render.
            index: Column index.
            level: Nesting depth.
            expanded: Whether the item is expanded.
        """
        value = model.get_item_value(item, index)
        if value is not None:
            ui.Label(str(value))


class RuleTypeSearchWidget:
    """Searchable dropdown widget for selecting a rule type from the registry.

    Displays a collapsed label showing the current selection.  Clicking opens
    a floating popup with a search field, package filter menu, and a two-column
    TreeView (Rule Name | Package).

    Args:
        registry: Rule registry used to populate the dropdown.
        current_type: FQCN of the initially selected rule type.
        on_selection_changed_fn: Callback invoked with the selected FQCN
            whenever the user picks a new rule type.
    """

    # Style constants matching Kit's NvidiaDark ComboBox theme.
    _BG = 0xFF23211F
    """Background color value for the dropdown widget styling."""
    _TEXT = 0xFFD5D5D5
    """Text color value for the dropdown widget styling."""
    _ARROW = 0xFF9E9E9E
    """Arrow color value for the dropdown widget styling."""
    _BORDER_RADIUS = 5
    """Border radius value for the dropdown widget styling."""
    _FONT_SIZE = 14.0
    """Font size value for the dropdown widget styling."""

    def __init__(
        self,
        registry: RuleRegistry,
        current_type: str = "",
        on_selection_changed_fn: Callable[[str], None] | None = None,
    ) -> None:
        self._on_selection_changed_fn = on_selection_changed_fn
        self._delegate = RuleTypeItemDelegate()
        self._package_menu_items: list[ui.MenuItem] = []
        self._filter_active = False

        self._list_model = RuleTypeListModel(registry)
        if current_type:
            self._list_model.set_selection_by_type(current_type)

        self._frame = ui.Frame(height=22)
        self._popup = ui.Window(
            f"_rule_type_search_{id(self)}",
            width=100,
            height=200,
            flags=(
                ui.WINDOW_FLAGS_NO_RESIZE
                | ui.WINDOW_FLAGS_NO_SCROLLBAR
                | ui.WINDOW_FLAGS_NO_TITLE_BAR
                | ui.WINDOW_FLAGS_NO_MOVE
                | ui.WINDOW_FLAGS_NO_SAVED_SETTINGS
            ),
            visible=False,
        )
        # Close the popup when it loses focus (the user clicked outside).
        # This mirrors ComboBox-like behaviour without using
        # WINDOW_FLAGS_POPUP, which would cause ImGui to auto-close this
        # window the moment the filter sub-menu (another popup) opens.
        self._popup.set_focused_changed_fn(self._on_popup_focused_changed)
        self._popup.set_visibility_changed_fn(self._on_popup_visibility_changed)

        self._build_ui()

    # -- public API ---------------------------------------------------------

    def destroy(self) -> None:
        """Hide the popup and release callbacks.

        Example:

        .. code-block:: python

            widget = RuleTypeSearchWidget(registry)
            widget.destroy()
        """
        self._popup.set_focused_changed_fn(None)
        self._popup.set_visibility_changed_fn(None)
        self._popup.visible = False

    @property
    def selected_rule_type(self) -> str:
        """FQCN of the currently selected rule type, or empty string.

        Returns:
            FQCN of the currently selected rule type, or empty string if no rule is selected.
        """
        sel = self._list_model.selection
        return sel.rule_type if sel else ""

    # -- internal helpers ---------------------------------------------------

    def _get_display_text(self) -> str:
        """Return text to show in the collapsed combo-box label.

        Returns:
            The selected rule name or a placeholder string.
        """
        sel = self._list_model.selection
        return sel.rule_name if sel else "-- Select Rule Type --"

    def _build_ui(self) -> None:
        """Build the collapsed display and floating popup widgets."""
        combo_style = {
            "background_color": self._BG,
            "border_radius": self._BORDER_RADIUS,
        }
        combo_text_style = {
            "color": self._TEXT,
            "font_size": self._FONT_SIZE,
        }

        # -- collapsed display (always visible; popup overlays on top) ------
        with self._frame:
            with ui.ZStack(width=ui.Percent(100), height=22):
                ui.Rectangle(style=combo_style)
                with ui.HStack():
                    ui.Spacer(width=6)
                    self._display_label = ui.Label(
                        self._get_display_text(),
                        elided_text=True,
                        tooltip=self._get_display_text(),
                        style=combo_text_style,
                    )
                    with ui.VStack(width=0):
                        ui.Spacer()
                        ui.Triangle(
                            width=TRIANGLE_SIZE,
                            height=TRIANGLE_SIZE,
                            alignment=ui.Alignment.CENTER_BOTTOM,
                            style={"background_color": self._ARROW},
                        )
                        ui.Spacer()
                    ui.Spacer(width=4)
                ui.InvisibleButton(height=22)
            self._frame.set_mouse_pressed_fn(self._enter_edit_mode)

        # -- popup with search / filter / tree view -------------------------
        bg_style = {"background_color": self._BG}
        with self._popup.frame:
            with ui.VStack(style=bg_style):
                with ui.HStack(height=22, spacing=2):
                    ui.Spacer(width=2)
                    self._query_field = ui.StringField(
                        height=22,
                        style={
                            "background_color": self._BG,
                            "color": self._TEXT,
                            "font_size": self._FONT_SIZE,
                            "border_radius": self._BORDER_RADIUS,
                        },
                    )
                    # Subscription MUST be stored or it is garbage-collected.
                    self._query_value_sub = self._query_field.model.subscribe_value_changed_fn(
                        lambda _: self._on_query_changed()
                    )
                    self._filter_btn = ui.Button(
                        image_url=FILTER_ICON_URL,
                        image_width=16,
                        image_height=16,
                        width=22,
                        height=22,
                        tooltip="Filter by package",
                        clicked_fn=self._show_filter_menu,
                        style={"background_color": 0x00000000},
                    )
                    # Hidden MenuBar hosts the filter popup.
                    self._menu_bar = ui.MenuBar()
                    self._menu_bar.visible = False
                    with self._menu_bar:
                        self._filter_menu = ui.Menu("Filter")
                        with self._filter_menu:
                            ui.MenuItem(
                                "Clear selection",
                                checkable=False,
                                triggered_fn=self._clear_package_filter,
                            )
                            ui.Separator()
                            self._package_menu_items = []
                            for pkg in self._list_model.get_packages():
                                mi = ui.MenuItem(pkg, checkable=True, checked=False)
                                mi.set_checked_changed_fn(lambda _: self._on_query_changed())
                                self._package_menu_items.append(mi)

                with ui.ScrollingFrame(style=bg_style):
                    self._tree_view = ui.TreeView(
                        self._list_model,
                        delegate=self._delegate,
                        header_visible=True,
                        root_visible=False,
                        columns_resizable=True,
                        column_widths=[ui.Fraction(2), ui.Fraction(1)],
                        style={
                            "TreeView": {
                                "background_color": self._BG,
                                "color": self._TEXT,
                                "font_size": self._FONT_SIZE,
                            },
                            "TreeView:selected": {"background_color": 0xFF33312F},
                            "TreeView.Item": {
                                "color": self._TEXT,
                                "font_size": self._FONT_SIZE,
                            },
                        },
                    )
                    self._tree_view.set_selection_changed_fn(self._on_item_selected)

    # -- popup focus / visibility callbacks (ComboBox-style) ----------------

    def _on_popup_focused_changed(self, focused: bool) -> None:
        """Close the popup when it loses focus (user clicked outside).

        Args:
            focused: Whether the popup gained or lost focus.
        """
        if not self._popup.visible:
            return
        if focused:
            # Popup regained focus (e.g. filter menu was dismissed and
            # focus returned here).  Safe to clear the guard flag.
            self._filter_active = False
            return
        # Popup lost focus.  While the filter sub-menu is open it
        # repeatedly steals focus; keep the popup alive until the menu
        # interaction finishes and focus returns (handled above).
        if self._filter_active:
            return
        self._close_popup()

    def _on_popup_visibility_changed(self, visible: bool) -> None:
        """Sync display label if visibility changes externally.

        Args:
            visible: Whether the popup is now visible.
        """
        if not visible:
            self._update_display_label()

    # -- edit / display mode toggle -----------------------------------------

    def _show_filter_menu(self) -> None:
        """Open the package filter sub-menu."""
        self._filter_active = True
        self._filter_menu.show()

    def _clear_package_filter(self) -> None:
        """Uncheck all package filter menu items."""
        for mi in self._package_menu_items:
            mi.checked = False

    def _enter_edit_mode(self, *args: object) -> None:
        """Open the popup dropdown, positioning it over the collapsed label.

        Args:
            *args: Mouse event arguments forwarded by Omni UI.
        """
        if len(args) >= 3 and args[2] != 0:
            return
        if self._popup.visible:
            return
        self._tree_view.clear_selection()
        self._popup.position_x = self._frame.screen_position_x - 4
        self._popup.position_y = self._frame.screen_position_y - 4
        self._popup.width = self._frame.computed_width + 6
        self._popup.visible = True
        self._on_query_changed()

    def _close_popup(self) -> None:
        """Close the popup and refresh the collapsed display label."""
        self._filter_active = False
        self._popup.visible = False
        self._update_display_label()

    def _update_display_label(self) -> None:
        """Update the collapsed display label text and tooltip."""
        text = self._get_display_text()
        self._display_label.text = text
        self._display_label.set_tooltip(text)

    # -- selection / query --------------------------------------------------

    def _on_item_selected(self, items: list) -> None:
        """Handle user selecting an item in the TreeView.

        Args:
            items: List of selected ``RuleTypeItem`` instances.
        """
        if not self._popup.visible:
            return
        if not items:
            return
        selected = items[0]
        if isinstance(selected, RuleTypeItem):
            self._list_model.selection = selected
            self._close_popup()
            if self._on_selection_changed_fn:
                self._on_selection_changed_fn(selected.rule_type)

    def _on_query_changed(self) -> None:
        """Re-filter the item list based on the current search text and package checkboxes."""
        self._filter_active = False
        query = self._query_field.model.get_value_as_string()
        active_packages = [mi.text for mi in self._package_menu_items if mi.checked]
        self._list_model.filter_items(query, active_packages if active_packages else None)
        self._query_field.focus_keyboard()


@runtime_checkable
class ActionProtocol(Protocol):
    """Protocol defining the interface that all actions must implement.

    This allows both ``PLACEHOLDER_BaseActionItem`` and real
    ``AssetTransformerAction`` subclasses to be used interchangeably in the UI.

    Args:
        *args: Additional positional arguments.
        **kwargs: Additional keyword arguments.
    """

    @property
    def name(self) -> str:
        """Human-readable name of this action.

        Returns:
            The action's display name.
        """
        ...

    @property
    def model(self) -> ui.AbstractValueModel:
        """Boolean value model tracking the enabled state.

        Returns:
            The boolean value model for the enabled state.
        """
        ...

    @property
    def enabled(self) -> bool:
        """Whether this action is enabled.

        Returns:
            True if the action is enabled.
        """
        ...

    @enabled.setter
    def enabled(self, value: bool) -> None: ...

    def run(self) -> bool:
        """Execute the action.

        Returns:
            True if the action completed successfully.
        """
        return True

    def build_ui(self) -> None:
        """Build custom configuration UI for this action."""
        ...

    def to_dict(self) -> dict:
        """Serialize this action to a dictionary for JSON storage.

        Returns:
            Dictionary representation of this action.
        """
        return {}


class PLACEHOLDER_BaseActionItem:  # noqa: N801
    """Placeholder action item for testing the UI before real actions are available.

    This class mimics the interface of ``AssetTransformerAction`` and can be
    replaced once the real action registry is implemented.

    Args:
        name: Display name for the action.
        enabled: Whether the action starts enabled.
        example_option: Example string configuration value.
        feature_enabled: Example boolean configuration value.
    """

    # Type identifier for serialization - real actions should override this
    ACTION_TYPE = "PLACEHOLDER_BaseActionItem"
    """Type identifier for serialization - real actions should override this."""

    def __init__(
        self,
        name: str,
        enabled: bool = True,
        example_option: str = "",
        feature_enabled: bool = False,
    ) -> None:
        self.__name = name
        self.__model = ui.SimpleBoolModel()
        self.__model.set_value(enabled)

        # Config models - these store the actual values that get serialized
        self.__example_option_model = ui.SimpleStringModel()
        self.__example_option_model.set_value(example_option)

        self.__feature_enabled_model = ui.SimpleBoolModel()
        self.__feature_enabled_model.set_value(feature_enabled)

    def get_name(self) -> str:
        """Return the display name of this action.

        Returns:
            The action name.
        """
        return self.__name

    def run(self) -> bool:
        """Execute the placeholder action (logs a warning stub).

        Returns:
            Always True.
        """
        carb.log_warn(f"Action [{self.__name}] STUB - no implementation")
        return True

    def build_ui(self) -> None:
        """Build placeholder UI for this action.

        Real actions will override this to provide custom configuration
        controls.  The UI widgets are bound to internal models so changes are
        automatically saved.
        """
        with ui.VStack(name="action_row", height=0, spacing=4):
            ui.Label(
                f"Configuration for '{self.__name}' (placeholder)",
                name="placeholder_config",
            )
            # Widgets bound to models - changes are automatically tracked
            with ui.HStack(height=0, spacing=8):
                ui.Label("Example Option:", width=120)
                ui.StringField(height=22, model=self.__example_option_model)
            with ui.HStack(height=0, spacing=8):
                ui.Label("Another Option:", width=120)
                ui.CheckBox(height=0, model=self.__feature_enabled_model)
                ui.Label("Enable feature", name="secondary")

    @property
    def name(self) -> str:
        """Human-readable name of this action.

        Returns:
            The action name.
        """
        return self.__name

    @property
    def model(self) -> ui.AbstractValueModel:
        """Boolean value model tracking the enabled state.

        Returns:
            The boolean value model for the enabled state.
        """
        return self.__model

    @property
    def enabled(self) -> bool:
        """Whether this action is enabled.

        Returns:
            True if the action is enabled.
        """
        return self.__model.get_value_as_bool()

    @enabled.setter
    def enabled(self, value: bool) -> None:
        self.__model.set_value(value)

    def to_dict(self) -> dict:
        """Serialize this action to a dictionary for JSON storage.

        Real actions should override this to include their specific
        configuration.

        Returns:
            Dictionary representation of this action.
        """
        return {
            "type": self.ACTION_TYPE,
            "name": self.__name,
            "enabled": self.enabled,
            "config": {
                "example_option": self.__example_option_model.get_value_as_string(),
                "feature_enabled": self.__feature_enabled_model.get_value_as_bool(),
            },
        }

    @classmethod
    def from_dict(cls, data: dict) -> "PLACEHOLDER_BaseActionItem":
        """Create an action instance from a dictionary.

        Real actions should override this to restore their specific
        configuration.

        Args:
            data: Dictionary previously produced by ``to_dict()``.

        Returns:
            A new ``PLACEHOLDER_BaseActionItem`` instance.
        """
        config = data.get("config", {})
        return cls(
            name=data.get("name", "Unnamed Action"),
            enabled=data.get("enabled", True),
            example_option=config.get("example_option", ""),
            feature_enabled=config.get("feature_enabled", False),
        )


class RuleActionItem:
    """Action wrapper for a transformer rule specification.

    Bridges a ``RuleSpec`` with the UI layer, exposing editable models for the
    rule name, type, destination, and dynamic parameters.

    Args:
        spec: The rule specification to wrap.
        registry: Rule registry for resolving rule classes.
    """

    ACTION_TYPE = "RuleActionItem"
    """Type identifier for serialization."""

    def __init__(self, spec: RuleSpec, registry: RuleRegistry | None = None) -> None:
        self._spec = spec
        self._registry = registry or RuleRegistry()
        self._frame: ui.CollapsableFrame | None = None
        self._remove_callback: Callable | None = None
        self._rebuild_callback: Callable | None = None
        derived_name = spec.type.split(".")[-1] if spec.type else ""
        self._auto_name = spec.name == derived_name
        self._updating_name = False
        self._rule_type_widget: RuleTypeSearchWidget | None = None

        self._enabled_model = ui.SimpleBoolModel()
        self._enabled_model.set_value(spec.enabled)
        self._enabled_model.add_value_changed_fn(self._on_enabled_changed)

        self._name_model = ui.SimpleStringModel()
        self._name_model.set_value(spec.name)
        self._name_model.add_value_changed_fn(self._on_name_changed)

        self._destination_model = ui.SimpleStringModel()
        self._destination_model.set_value(spec.destination or "")
        self._destination_model.add_value_changed_fn(self._on_destination_changed)

    def set_frame(self, frame: ui.CollapsableFrame) -> None:
        """Associate a collapsable frame with this action.

        Args:
            frame: The UI frame whose title will mirror the rule name.
        """
        self._frame = frame
        self._frame.title = self._spec.name

    def set_remove_callback(self, callback: Callable | None) -> None:
        """Register a callback invoked when this action requests removal.

        Args:
            callback: Callable to invoke, or None to clear.
        """
        self._remove_callback = callback

    def set_rebuild_callback(self, callback: Callable | None) -> None:
        """Register a callback invoked when the action UI needs rebuilding.

        Args:
            callback: Callable to invoke, or None to clear.
        """
        self._rebuild_callback = callback

    def get_rule_spec(self) -> RuleSpec:
        """Return the underlying rule specification.

        Returns:
            The ``RuleSpec`` wrapped by this action.
        """
        return self._spec

    def _on_enabled_changed(self, model: ui.AbstractValueModel) -> None:
        """Sync the spec enabled flag when the model changes.

        Args:
            model: The boolean value model that changed.
        """
        self._spec.enabled = model.get_value_as_bool()

    def _on_name_changed(self, model: ui.AbstractValueModel) -> None:
        """Sync the spec name when the model changes.

        Args:
            model: The string value model that changed.
        """
        name = model.get_value_as_string()
        self._spec.name = name
        if not self._updating_name:
            self._auto_name = False
        if self._frame is not None:
            self._frame.title = name

    def _set_name(self, name: str) -> None:
        """Programmatically set the rule name without breaking auto-name tracking.

        Args:
            name: The new rule name.
        """
        self._updating_name = True
        self._name_model.set_value(name)
        self._updating_name = False
        self._spec.name = name
        if self._frame is not None:
            self._frame.title = name

    def _on_destination_changed(self, model: ui.AbstractValueModel) -> None:
        """Sync the spec destination when the model changes.

        Args:
            model: The string value model that changed.
        """
        value = model.get_value_as_string().strip()
        self._spec.destination = value if value else None

    def _resolve_rule_parameters(self) -> list[RuleConfigurationParam]:
        """Instantiate the rule on a temporary stage and query its parameters.

        Returns:
            List of configuration parameter descriptors for the rule type.
        """
        if not self._spec.type:
            return []
        rule_cls = self._registry.get(self._spec.type)
        if rule_cls is None:
            return []
        try:
            temp_stage = Usd.Stage.CreateInMemory()
            rule = rule_cls(temp_stage, "", "", {"params": dict(self._spec.params)})
            return rule.get_configuration_parameters()
        except Exception as exc:  # noqa: BLE001
            carb.log_warn(f"Failed to load parameters for rule '{self._spec.type}': {exc}")
        return []

    def _apply_param_value(self, name: str, value: object) -> None:
        """Store a parameter value into the rule spec.

        Args:
            name: Parameter name.
            value: The value to store.
        """
        self._spec.params[name] = value

    def _ensure_param_defaults(self, param_defs: list[RuleConfigurationParam]) -> None:
        """Fill missing spec params with their declared defaults.

        Args:
            param_defs: Parameter descriptors from the rule class.
        """
        for param in param_defs:
            if param.name in self._spec.params:
                continue
            default_value = param.default_value
            if default_value is None and param.param_type in (list, dict):
                default_value = [] if param.param_type is list else {}
            self._spec.params[param.name] = default_value

    # -- list / dict parameter editors --------------------------------------

    _EDITOR_ICON = 15
    """Icon size in pixels for add/remove buttons in parameter editors."""
    _EDITOR_BTN_STYLE: dict = {"padding": 2, "margin": 0}
    """Style dictionary for icon buttons with minimal padding and margin."""

    @staticmethod
    def _icon_btn(url: str, tooltip: str, clicked_fn: Callable) -> None:
        """Render a small icon button (add / remove) with consistent sizing.

        Args:
            url: SVG icon URL.
            tooltip: Tooltip text.
            clicked_fn: Callback invoked when the button is clicked.
        """
        sz = RuleActionItem._EDITOR_ICON
        with ui.VStack(height=22, spacing=0, width=22):
            ui.Spacer(height=3)
            ui.Button(
                image_url=url,
                image_width=sz,
                image_height=sz,
                width=sz,
                height=sz,
                tooltip=tooltip,
                clicked_fn=clicked_fn,
                style=RuleActionItem._EDITOR_BTN_STYLE,
            )
            ui.Spacer(height=3)

    def _cascading_row(self, label: str, remove_fn: Callable, sub_build_fn: Callable) -> None:
        """Header row (label + remove button) with an indented sub-editor below.

        Args:
            label: Display label for the row header.
            remove_fn: Callback to remove the element.
            sub_build_fn: Callable that builds the nested editor widgets.
        """
        with ui.VStack(height=0, spacing=2):
            with ui.HStack(height=22, spacing=4):
                ui.Label(label, width=30)
                ui.Spacer()
                self._icon_btn(REMOVE_ICON_URL, "Remove", remove_fn)
            with ui.HStack():
                ui.Spacer(width=16)
                sub_build_fn()

    def _build_list_editor(self, items: list) -> None:
        """Editable list: one StringField per element, add/remove buttons.

        Dict elements cascade into an indented dict sub-editor.

        Args:
            items: The mutable list to edit in-place.
        """
        frame = ui.Frame(height=0)

        def _rebuild() -> None:
            with frame:
                with ui.VStack(height=0, spacing=2):
                    for idx in range(len(items)):
                        elem = items[idx]
                        if isinstance(elem, dict):
                            self._cascading_row(
                                f"[{idx}]",
                                lambda i=idx: _remove(i),
                                lambda e=elem: self._build_dict_editor(e),
                            )
                        else:
                            with ui.HStack(height=22, spacing=4):
                                mdl = ui.SimpleStringModel()
                                mdl.set_value("" if elem is None else str(elem))
                                mdl.add_value_changed_fn(lambda m, i=idx: items.__setitem__(i, m.get_value_as_string()))
                                ui.StringField(model=mdl, height=22)
                                self._icon_btn(REMOVE_ICON_URL, "Remove element", lambda i=idx: _remove(i))
                    self._icon_btn(ADD_ICON_URL, "Add element", _add)

        def _add() -> None:
            items.append({} if items and isinstance(items[-1], dict) else "")
            _rebuild()

        def _remove(idx: int) -> None:
            if 0 <= idx < len(items):
                items.pop(idx)
                _rebuild()

        _rebuild()

    def _build_dict_editor(self, data: dict) -> None:
        """Editable dict: key + value StringFields per entry, add/remove buttons.

        List or dict values cascade into indented sub-editors.

        Args:
            data: The mutable dict to edit in-place.
        """
        frame = ui.Frame(height=0)

        def _key_field(key: str) -> None:
            mdl = ui.SimpleStringModel()
            mdl.set_value(key)

            def _on_rename(m: ui.SimpleStringModel, old: str = key) -> None:
                new = m.get_value_as_string()
                if new == old or new in data:
                    return
                entries = list(data.items())
                data.clear()
                for k, v in entries:
                    data[new if k == old else k] = v

            mdl.add_value_changed_fn(_on_rename)
            ui.StringField(model=mdl, height=22)

        def _rebuild() -> None:
            with frame:
                with ui.VStack(height=0, spacing=2):
                    for key in list(data.keys()):
                        value = data[key]
                        if isinstance(value, (list, dict)):
                            sub = (
                                (lambda v=value: self._build_list_editor(v))
                                if isinstance(value, list)
                                else (lambda v=value: self._build_dict_editor(v))
                            )
                            self._cascading_row(key, lambda k=key: _remove(k), sub)
                        else:
                            with ui.HStack(height=22, spacing=4):
                                _key_field(key)
                                val_mdl = ui.SimpleStringModel()
                                val_mdl.set_value("" if value is None else str(value))
                                val_mdl.add_value_changed_fn(
                                    lambda m, k=key: data.__setitem__(k, m.get_value_as_string())
                                )
                                ui.StringField(model=val_mdl, height=22)
                                self._icon_btn(REMOVE_ICON_URL, "Remove entry", lambda k=key: _remove(k))
                    self._icon_btn(ADD_ICON_URL, "Add entry", _add)

        def _add() -> None:
            n, new_key = 0, "key"
            while new_key in data:
                n += 1
                new_key = f"key_{n}"
            data[new_key] = ""
            _rebuild()

        def _remove(key: str) -> None:
            data.pop(key, None)
            _rebuild()

        _rebuild()

    # -- scalar parameter field builder -------------------------------------

    _SCALAR_SPECS: dict[type, tuple] = {
        bool: (ui.SimpleBoolModel, bool, "get_value_as_bool", ui.CheckBox, {"height": 0}),
        int: (ui.SimpleIntModel, lambda v: int(v or 0), "get_value_as_int", ui.IntField, {}),
        float: (ui.SimpleFloatModel, lambda v: float(v or 0), "get_value_as_float", ui.FloatField, {}),
    }
    """Mapping of Python types to their corresponding UI model class, converter function, getter method name, widget class, and widget kwargs for creating scalar parameter editors."""

    def _build_scalar_field(self, name: str, param_type: type, value: object) -> None:
        """Create a model + widget for a scalar parameter, wired to ``_apply_param_value``.

        Args:
            name: Parameter name.
            param_type: Expected Python type (bool, int, float, or str).
            value: Current parameter value.
        """
        spec = self._SCALAR_SPECS.get(param_type)
        if spec:
            model_cls, convert, getter, widget_cls, kwargs = spec
            mdl = model_cls()
            mdl.set_value(convert(value))
            mdl.add_value_changed_fn(lambda m, n=name, g=getter: self._apply_param_value(n, getattr(m, g)()))
            widget_cls(model=mdl, **kwargs)
        else:
            mdl = ui.SimpleStringModel()
            mdl.set_value("" if value is None else str(value))
            mdl.add_value_changed_fn(lambda m, n=name: self._apply_param_value(n, m.get_value_as_string()))
            ui.StringField(model=mdl)

    def _update_rule_type(self, selected_type: str) -> None:
        """Handle a new rule type being selected from the dropdown.

        Resets parameters and optionally updates the auto-generated name.

        Args:
            selected_type: FQCN of the newly selected rule type.
        """
        if selected_type == self._spec.type:
            return
        self._spec.type = selected_type
        self._spec.params = {}
        if self._auto_name:
            default_name = selected_type.split(".")[-1] if selected_type else "Rule"
            self._set_name(default_name)
        if self._rebuild_callback is not None:
            self._rebuild_callback()

    def run(self) -> bool:
        """Execute the action (stub; rules execute via the profile runner).

        Returns:
            Always True.
        """
        carb.log_warn(f"Action [{self._spec.name}] STUB - rules execute via profile runner")
        return True

    @property
    def name_model(self) -> ui.AbstractValueModel:
        """Editable model for the rule name (used by the header row).

        Returns:
            The name model for the rule.
        """
        return self._name_model

    @property
    def is_rule_type_missing(self) -> bool:
        """True when the configured rule type is not found in the registry.

        Returns:
            Whether the rule type is missing from the registry.
        """
        return bool(self._spec.type) and self._registry.get(self._spec.type) is None

    def build_ui(self) -> None:
        """Build the configuration UI for this rule action.

        Renders the rule-type dropdown, destination field, and dynamic
        parameter editors resolved from the registry.
        """
        if self._rule_type_widget is not None:
            self._rule_type_widget.destroy()

        with ui.VStack(name="rule_action", height=0, spacing=6):
            with ui.HStack(height=0, spacing=8):
                ui.Label("Rule Type", width=ui.Percent(20))
                self._rule_type_widget = RuleTypeSearchWidget(
                    registry=self._registry,
                    current_type=self._spec.type or "",
                    on_selection_changed_fn=self._update_rule_type,
                )

            with ui.HStack(height=0, spacing=8):
                ui.Label("Destination", width=ui.Percent(20))
                ui.StringField(model=self._destination_model)

            param_defs = self._resolve_rule_parameters()
            if not param_defs:
                ui.Label("No configuration parameters for this rule", name="secondary")
                return

            self._ensure_param_defaults(param_defs)
            ui.Spacer(height=4)
            for param in param_defs:
                value = self._spec.params.get(param.name, param.default_value)
                if value is None and param.param_type in (list, dict):
                    value = [] if param.param_type is list else {}

                # -- list / dict parameters get multi-row editors ----------
                if param.param_type in (list, dict):
                    # Ensure the spec holds a mutable reference.
                    if param.param_type is list and not isinstance(self._spec.params.get(param.name), list):
                        self._spec.params[param.name] = list(value)
                    elif param.param_type is dict and not isinstance(self._spec.params.get(param.name), dict):
                        self._spec.params[param.name] = dict(value)

                    with ui.HStack(height=0, spacing=8):
                        with ui.VStack(width=ui.Percent(40)):
                            ui.Label(
                                param.display_name,
                                tooltip=param.description or "",
                            )
                            ui.Spacer(height=ui.Percent(100))
                        if param.param_type is list:
                            self._build_list_editor(self._spec.params[param.name])
                        else:
                            self._build_dict_editor(self._spec.params[param.name])
                    continue

                # -- scalar parameters --------------------------------------
                with ui.HStack(height=0, spacing=8):
                    ui.Label(
                        param.display_name,
                        width=ui.Percent(40),
                        tooltip=param.description or "",
                    )
                    self._build_scalar_field(param.name, param.param_type, value)

    @property
    def name(self) -> str:
        """Human-readable name of this rule action.

        Returns:
            The action name.
        """
        return self._spec.name

    @property
    def model(self) -> ui.AbstractValueModel:
        """Boolean value model tracking the enabled state.

        Returns:
            The value model for the enabled state.
        """
        return self._enabled_model

    @property
    def enabled(self) -> bool:
        """Whether this action is enabled.

        Returns:
            True if the action is enabled.
        """
        return self._enabled_model.get_value_as_bool()

    @enabled.setter
    def enabled(self, value: bool) -> None:
        self._enabled_model.set_value(value)

    def to_dict(self) -> dict:
        """Serialize this action to a dictionary.

        Returns:
            Dictionary representation delegated to ``RuleSpec.to_dict()``.
        """
        return self._spec.to_dict()

    @classmethod
    def from_dict(cls, data: dict, registry: RuleRegistry | None = None) -> "RuleActionItem":
        """Create an instance from a serialized dictionary.

        Args:
            data: Dictionary previously produced by ``to_dict()``.
            registry: Optional rule registry.

        Returns:
            A new ``RuleActionItem`` wrapping the deserialized spec.
        """
        return cls(RuleSpec.from_dict(data), registry=registry)


class ActionItemValueModel(ui.AbstractValueModel):
    """Value model wrapper for action items.

    Wraps an action implementing ``ActionProtocol`` for use in the TreeView
    model.

    Args:
        item: Action instance to wrap.
    """

    def __init__(self, item: ActionProtocol) -> None:
        super().__init__()
        self.__action = item

    def get_action(self) -> ActionProtocol:
        """Return the wrapped action instance.

        Returns:
            The underlying action.
        """
        return self.__action

    def get_value_as_string(self) -> str:
        """Return the string representation of the model value.

        Returns:
            The action name.
        """
        return self.as_string

    @property
    def as_string(self) -> str:
        """String representation of the wrapped action (its name).

        Returns:
            The action name.
        """
        return self.__action.name

    @as_string.setter
    def as_string(self, arg1: str) -> None:
        carb.log_error("ActionItemValueModel cannot set action item from str")


class ActionListItem(ui.AbstractItem):
    """Single item of the model representing an action in the list.

    Manages its own expanded state for showing/hiding the action's
    configuration UI.

    Args:
        action: Value model wrapping the action.
    """

    def __init__(self, action: ActionItemValueModel) -> None:
        super().__init__()
        self.action_model = action
        self.expanded = False

    def toggle_expanded(self) -> None:
        """Toggle the expanded state of this item."""
        self.expanded = not self.expanded

    def __repr__(self) -> str:
        """Return a debug representation including the action name.

        Returns:
            Quoted action name string.
        """
        return f'"{self.action_model.as_string}"'


class ActionListModel(ui.AbstractItemModel):
    """Item model for the action list TreeView with drag-and-drop reordering.

    Args:
        *args: Initial ``ActionItemValueModel`` instances to populate with.
    """

    def __init__(self, *args: ActionItemValueModel) -> None:
        super().__init__(*args)
        self._children: list[ui.AbstractItem] = [ActionListItem(t) for t in args]

    def _item_changed(self, arg0: ui.AbstractItem | None) -> None:
        """Notify observers that an item or the list changed.

        Args:
            arg0: The changed item, or None for a full-list change.
        """
        super()._item_changed(arg0)  # pyright: ignore[reportArgumentType]

    def toggle_item_expanded(self, item: ActionListItem) -> None:
        """Toggle an item's expanded state and notify the view to rebuild.

        Args:
            item: The item to toggle.
        """
        item.toggle_expanded()
        self._item_changed(item)

    def append_child_item(self, parentItem: ui.AbstractItem | None, model: ui.AbstractValueModel) -> ui.AbstractItem:
        """Append an action to the list and subscribe to its enabled-state changes.

        Args:
            parentItem: Unused (flat list).
            model: An ``ActionItemValueModel`` to append.

        Returns:
            The newly created ``ActionListItem``.
        """
        assert type(model) is ActionItemValueModel
        item = ActionListItem(model)
        self._children.append(item)

        # Subscribe to action's enabled state changes to update button states
        action = model.get_action()
        action.model.add_value_changed_fn(lambda _: self._item_changed(None))

        self._item_changed(None)

        return item

    def remove_item(self, item: ui.AbstractItem) -> None:
        """Remove an item from the list.

        Args:
            item: The item to remove.
        """
        if item in self._children:
            self._children.remove(item)
            self._item_changed(None)

    def clear_all(self) -> None:
        """Remove all items from the list."""
        self._children.clear()
        self._item_changed(None)

    def has_enabled_actions(self) -> bool:
        """Check if there is at least one enabled action in the list.

        Returns:
            True if any action has ``enabled == True``.
        """
        for child in self._children:
            assert type(child) is ActionListItem
            if child.action_model.get_action().enabled:
                return True
        return False

    def get_item_children(self, parentItem: ui.AbstractItem | None = None) -> list[ui.AbstractItem]:
        """Return all children (root only; flat list).

        Args:
            parentItem: Parent item. Only root (None) returns children.

        Returns:
            List of ``ActionListItem`` instances.
        """
        if parentItem is not None:
            # Flat list - only root has children
            return []

        return self._children

    def get_item_value_model_count(self, item: ui.AbstractItem | None = None) -> int:
        """Return the number of columns (always 1).

        Args:
            item: Unused.

        Returns:
            The column count.
        """
        return 1

    def get_item_value_model(self, item: ui.AbstractItem | None = None, column_id: int = 0) -> ActionItemValueModel:
        """Return the value model for the given item.

        Args:
            item: The ``ActionListItem`` to query.
            column_id: Column index (unused; single column).

        Returns:
            The ``ActionItemValueModel`` attached to the item.
        """
        assert type(item) is ActionListItem
        return item.action_model

    def get_drag_mime_data(self, item: ui.AbstractItem | None = None) -> str:
        """Return MIME data so the item can be dragged and dropped.

        Args:
            item: The ``ActionListItem`` being dragged.

        Returns:
            The action name string used as drag payload.
        """
        # As we don't do Drag and Drop to the operating system, we return the string.
        #
        assert type(item) is ActionListItem
        return item.action_model.as_string

    def drop_accepted(  # pyright: ignore[reportIncompatibleMethodOverride]
        self,
        item_target: ui.AbstractItem | None,
        item_source: ui.AbstractItem | None,
        drop_location: int = -1,
    ) -> bool:
        """Determine whether a drag-and-drop operation is accepted.

        Args:
            item_target: The item being hovered over.
            item_source: The item being dragged.
            drop_location: Position index.

        Returns:
            True if the source item belongs to this model.
        """
        # reportIncompatibleMethodOverride is ignored because of a typo (tagget <- target) in the C++ bindings
        # Always accept drops from items in this list so the TreeView
        # draws its built-in between-items indicator line.
        try:
            self._children.index(item_source)
        except (ValueError, TypeError):
            return False
        return True

    def drop(  # pyright: ignore[reportIncompatibleMethodOverride]
        self,
        item_target: ui.AbstractItem | None,
        item_source: ui.AbstractItem | None,
        drop_location: int = -1,
    ) -> None:
        """Handle a drop event by reordering items in the list.

        Args:
            item_target: The item the source was dropped onto.
            item_source: The item that was dragged.
            drop_location: Position index where the item was dropped.
        """
        # reportIncompatibleMethodOverride is ignored because of a typo (tagget <- target) in the C++ bindings
        try:
            source_id = self._children.index(item_source)
        except ValueError:
            # Not in the list. This is the source from another model.
            return

        # If dropped on an item (not between items), compute the target index
        if drop_location < 0 and item_target is not None:
            try:
                drop_location = self._children.index(item_target)
            except ValueError:
                return

        if drop_location < 0 or source_id == drop_location:
            return

        self._children.remove(item_source)

        if drop_location > len(self._children):
            self._children.append(item_source)
        else:
            if source_id < drop_location:
                drop_location = drop_location - 1
            self._children.insert(drop_location, item_source)

        self._item_changed(None)
