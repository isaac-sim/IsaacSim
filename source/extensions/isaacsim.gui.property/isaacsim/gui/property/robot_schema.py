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

"""Property widgets for Isaac Robot Schema APIs (Robot, Link, Joint, Site)."""

from __future__ import annotations

from functools import cache, partial

import carb
import omni
import omni.ui as ui
from omni.kit.property.usd.prim_selection_payload import PrimSelectionPayload
from omni.kit.property.usd.usd_property_widget import UsdPropertiesWidget
from omni.kit.property.usd.widgets import ICON_PATH
from pxr import Sdf, Usd, UsdGeom, UsdPhysics, Vt
from usd.schema.isaac import robot_schema

from . import style
from .style import BG_INPUT as _BG_INPUT
from .style import BG_PANEL as _BG_PANEL
from .style import CELL_BG as _CELL_BG
from .style import COMBO_STYLE as _COMBO_STYLE
from .style import DROP_INDICATOR_COLOR as _DROP_INDICATOR_COLOR
from .style import DROP_INDICATOR_TRANSPARENT as _DROP_INDICATOR_TRANSPARENT
from .style import FIELD_LABEL_WIDTH as _FIELD_LABEL_WIDTH
from .style import LIST_FIXED_HEIGHT as _LIST_FIXED_HEIGHT
from .style import LIST_FONT_SIZE as _LIST_FONT_SIZE
from .style import ROW_HEIGHT as _ROW_HEIGHT
from .style import TABLE_ROW_GAP as _TABLE_ROW_GAP
from .style import TABLE_ROW_HEIGHT as _TABLE_ROW_HEIGHT
from .style import TEXT_DIM as _TEXT_DIM
from .style import TEXT_PRIMARY as _TEXT_PRIMARY
from .style import TOOLTIP_STYLE as _TOOLTIP_STYLE

_EXCLUSIVE_SCHEMAS = (
    robot_schema.Classes.ROBOT_API,
    robot_schema.Classes.LINK_API,
    robot_schema.Classes.JOINT_API,
)

# Alias exported to sibling widget modules so every widget in this extension
# gets its instance cached with the same policy. `functools.cache` memoizes the
# wrapped callable on its args tuple; all widgets are only ever instantiated
# once with a fixed title, so a single instance per decorated class is returned.
_singleton = cache


class _RobotSchemaWidgetBase(UsdPropertiesWidget):
    """Base class for USD Robot Schema property widgets in Isaac Sim.

    Provides a unified interface for creating property widgets that manage robot schema APIs
    (Robot API, Link API, Joint API) in the property panel. The widget displays schema-specific
    attributes and relationships, handles schema application/removal, and provides contextual
    menu integration for applying schemas to USD prims.

    The widget automatically filters and displays only the attributes and relationships
    specified during initialization, with proper display names and validation. It includes
    a remove button in the header for cleaning up applied schemas and their properties.

    Args:
        title: Display title for the widget in the property panel.
        collapsed: Whether the widget should start in a collapsed state.
        schema_class: The USD schema class enum value (e.g., robot_schema.Classes.ROBOT_API).
        attributes: List of schema attribute objects to display in the widget.
        menu_label: Label text for the context menu entry used to apply the schema.
        apply_fn: Function to call when applying the schema to a prim.
        relationships: List of schema relationship objects to display in the widget.
        exclusive_classes: List of schema classes that prevent this widget from being shown or applied.
    """

    _MENU_PREFIX = "Isaac/Robot Schema"
    """Prefix used for menu entries in the property window button menu."""

    def __init__(
        self,
        title: str,
        collapsed: bool,
        schema_class: object,
        attributes: list,
        menu_label: str,
        apply_fn: object,
        relationships: object = None,
        exclusive_classes: object = None,
    ) -> None:
        super().__init__(title, collapsed)
        from omni.kit.property.usd import PrimPathWidget

        self._schema_class = schema_class
        self._attributes = attributes
        self._relationships = relationships or []
        self._apply_fn = apply_fn
        self._menu_label = menu_label
        self._attr_map = {attr.name: attr for attr in attributes}
        self._relationship_map = {rel.name: rel for rel in self._relationships}
        self._prim = None
        self._old_payload = None
        self._exclusive_classes = exclusive_classes or _EXCLUSIVE_SCHEMAS

        self._menu_entries = [
            PrimPathWidget.add_button_menu_entry(
                f"{self._MENU_PREFIX}/{menu_label}", show_fn=self._button_show, onclick_fn=self._button_onclick
            )
        ]

    def destroy(self) -> None:
        """Clean up resources and remove menu entries."""
        from omni.kit.property.usd import PrimPathWidget

        for menu in self._menu_entries:
            PrimPathWidget.remove_button_menu_entry(menu)
        self._menu_entries = []

    def _button_show(self, objects: dict) -> bool:
        """Determines if the button should be shown based on prim selection.

        Args:
            objects: Dictionary containing stage and prim_list for evaluation.

        Returns:
            True if button should be shown, False otherwise.
        """
        stage = objects.get("stage")
        prim_list = objects.get("prim_list")
        if not stage or not prim_list:
            return False
        for item in prim_list:
            prim = stage.GetPrimAtPath(item) if isinstance(item, Sdf.Path) else item
            if prim and not self._has_exclusive_schema(prim):
                return True
        return False

    def _button_onclick(self, payload: PrimSelectionPayload) -> None:
        """Handles button click to apply schema to selected prims.

        Args:
            payload: The prim selection payload containing paths to process.
        """
        stage = self._payload.get_stage() if self._payload else omni.usd.get_context().get_stage()
        if not stage:
            return

        for path in payload:
            if not path:
                continue
            prim = stage.GetPrimAtPath(path)
            if not prim or self._has_exclusive_schema(prim):
                continue
            self._apply_fn(prim)

            instanceable = [p for p in Usd.PrimRange(prim) if p.IsInstanceable()]
            if instanceable:
                prim.SetInstanceable(True)
                prim.ClearMetadata("instanceable")
        self._request_refresh()

    def _request_refresh(self) -> None:
        """Refresh the property window without touching USD selection."""
        property_window = omni.kit.window.property.get_window()
        if property_window and property_window._window:  # noqa: SLF001
            property_window._window.frame.rebuild()  # noqa: SLF001

    def _on_usd_changed(self, notice: object, stage: object) -> None:
        """Handles USD change notifications.

        Args:
            notice: The USD change notice.
            stage: The USD stage that changed.
        """
        targets = notice.GetChangedInfoOnlyPaths()
        if self._old_payload != self.on_new_payload(self._payload):
            self._request_refresh()
        else:
            super()._on_usd_changed(notice, stage)

    def _get_prim(self, prim_path: object) -> Usd.Prim | None:
        """Retrieves a prim with the required schema from the given path.

        Args:
            prim_path: The path to the prim.

        Returns:
            The prim if it exists and has the required schema, None otherwise.
        """
        if prim_path:
            stage = self._payload.get_stage()
            if stage:
                prim = stage.GetPrimAtPath(prim_path)
                if prim and prim.HasAPI(self._schema_class.value):
                    return prim
        return None

    def on_new_payload(self, payload: list) -> Usd.Prim | bool:
        """See ``PropertyWidget.on_new_payload``.

        Args:
            payload: The new prim selection payload.

        Returns:
            The prim if found, or ``False`` if the widget should not be shown.
        """
        if not super().on_new_payload(payload):
            return False

        if len(self._payload) != 1:
            return False
        prim_path = self._payload.get_paths()[0]
        self._prim = self._get_prim(prim_path)
        self._old_payload = self._prim
        if not self._prim:
            return False

        return self._prim

    def on_remove_attr(self) -> None:
        """Removes the schema and associated attributes and relationships from the prim."""
        stage = self._payload.get_stage()
        if not stage or not self._payload:
            return

        prim = self._get_prim(self._payload.get_paths()[0])
        if not prim:
            return

        if prim.HasAPI(self._schema_class.value):
            prim.RemoveAppliedSchema(self._schema_class.value)
        for attr in self._attributes:
            if prim.HasAttribute(attr.name):
                prim.RemoveProperty(attr.name)
        for rel in self._relationships:
            if prim.HasRelationship(rel.name):
                prim.RemoveProperty(rel.name)
        self._request_refresh()

    def _filter_props_to_build(self, props: list) -> list:
        """Filters properties to build based on the schema's attributes and relationships.

        Args:
            props: List of properties to filter.

        Returns:
            Filtered list of properties with display names set.
        """
        filtered = []
        for prop in props:
            if isinstance(prop, Usd.Attribute) and prop.GetName() in self._attr_map:
                attr = self._attr_map[prop.GetName()]
                prop.SetDisplayName(attr.display_name)
                filtered.append(prop)
            elif isinstance(prop, Usd.Relationship) and prop.GetName() in self._relationship_map:
                relationship = self._relationship_map[prop.GetName()]
                prop.SetDisplayName(relationship.display_name)
                filtered.append(prop)
        return filtered

    def _has_exclusive_schema(self, prim: object) -> bool:
        """Checks if the prim has any exclusive schema applied.

        Args:
            prim: The prim to check.

        Returns:
            True if prim has exclusive schema, False otherwise.
        """
        return any(prim.HasAPI(schema.value) for schema in self._exclusive_classes)

    def build_items(self) -> None:
        """Builds property widget items for the robot schema.

        Constructs the property items only when the collapsible frame is expanded and a valid prim is available.
        """
        if self._collapsable_frame and not self._collapsable_frame.collapsed and self._prim:
            super().build_items()

    def _build_frame_header(self, collapsed: bool, text: str, id: str | None = None) -> None:
        """Build a custom header for the CollapsableFrame with a remove button.

        Args:
            collapsed: Whether the frame is currently collapsed.
            text: The header label text.
            id: Optional identifier for the header.
        """
        if id is None:
            id = text

        if collapsed:
            alignment = ui.Alignment.RIGHT_CENTER
            width = 5
            height = 7
        else:
            alignment = ui.Alignment.CENTER_BOTTOM
            width = 7
            height = 5

        header_stack = ui.HStack(spacing=8)
        with header_stack:
            with ui.VStack(width=0):
                ui.Spacer()
                ui.Triangle(
                    style_type_name_override="CollapsableFrame.Header", width=width, height=height, alignment=alignment
                )
                ui.Spacer()
            ui.Label(text, style_type_name_override="CollapsableFrame.Header", width=ui.Fraction(1))

            button_style = {"Button.Image": {"color": 0xFFFFFFFF, "alignment": ui.Alignment.CENTER}}

            ui.Spacer(width=ui.Fraction(6))
            button_style["Button.Image"]["image_url"] = "${icons}/Cancel_64.png"

            with ui.ZStack(content_clipping=True, width=16, height=16):
                ui.Button(
                    "",
                    style=button_style,
                    clicked_fn=self.on_remove_attr,
                    identifier=f"remove_{self._schema_class.value}",
                    tooltip=f"Remove {self._schema_class.value}",
                )


@_singleton
class RobotAPIWidget(_RobotSchemaWidgetBase):
    """Property widget for prims with the IsaacRobotAPI schema applied.

    Provides a custom Figma-style layout with editable attribute fields and drag-reorderable
    joints and links lists backed by USD relationship targets.

    Args:
        title: Display title for the widget.
        collapsed: Whether the widget starts collapsed.
    """

    def __init__(self, title: str, collapsed: bool = False) -> None:
        super().__init__(
            title,
            collapsed,
            robot_schema.Classes.ROBOT_API,
            [
                robot_schema.Attributes.DESCRIPTION,
                robot_schema.Attributes.NAMESPACE,
                robot_schema.Attributes.ROBOT_TYPE,
                robot_schema.Attributes.LICENSE,
                robot_schema.Attributes.VERSION,
                robot_schema.Attributes.SOURCE,
                robot_schema.Attributes.CHANGELOG,
            ],
            "Robot API",
            robot_schema.ApplyRobotAPI,
            relationships=[
                robot_schema.Relations.ROBOT_LINKS,
                robot_schema.Relations.ROBOT_JOINTS,
            ],
        )
        self._selected_joint_index: int | None = None
        self._selected_link_index: int | None = None
        self._joints_list_frame = None
        self._links_list_frame = None
        self._joint_drop_indicators: list = []
        self._link_drop_indicators: list = []
        self._joint_row_rects: list = []
        self._link_row_rects: list = []
        self._joints_scroll_frame = None
        self._links_scroll_frame = None
        self._expanded_joints: set[int] = set()
        self._expanded_links: set[int] = set()
        self._robot_type_other_mode: bool = False
        self._robot_type_tokens: list[str] = []
        self._robot_type_frame = None
        self._changelog_frame = None
        self._force_update_recalculate: bool = False
        self._joint_picker = None
        self._link_picker = None

    def destroy(self) -> None:
        """Clean up pickers and delegate base-class teardown."""
        if self._joint_picker is not None:
            self._joint_picker.clean()
            self._joint_picker = None
        if self._link_picker is not None:
            self._link_picker.clean()
            self._link_picker = None
        super().destroy()

    def _on_usd_changed(self, notice: object, stage: object) -> None:
        if not self._prim:
            return super()._on_usd_changed(notice, stage)
        prim_path_str = str(self._prim.GetPath())
        # Resyncs on any descendant of the robot prim can reorder joint/link
        # relationships, so both lists are rebuilt in that case.
        for changed_path in notice.GetResyncedPaths():
            if str(changed_path).startswith(prim_path_str):
                self._rebuild_joints_list()
                self._rebuild_links_list()
                return None
        # For info-only notices only rebuild the specific list whose relationship
        # was touched, so unrelated attribute edits on a sibling prim don't
        # trigger both rebuilds.
        rebuild_joints = False
        rebuild_links = False
        for changed_path in notice.GetChangedInfoOnlyPaths():
            cp = str(changed_path)
            if not cp.startswith(prim_path_str):
                continue
            if "robotJoints" in cp:
                rebuild_joints = True
            if "robotLinks" in cp:
                rebuild_links = True
            if rebuild_joints and rebuild_links:
                break
        if rebuild_joints:
            self._rebuild_joints_list()
        if rebuild_links:
            self._rebuild_links_list()
        if rebuild_joints or rebuild_links:
            return None
        super()._on_usd_changed(notice, stage)

    def build_items(self) -> None:
        """Builds the custom Robot Schema UI with attribute fields and relationship target lists."""
        if not self._collapsable_frame or self._collapsable_frame.collapsed or not self._prim:
            return
        with ui.VStack(height=0, spacing=8):
            self._build_robot_schema_section()
            self._build_robot_joints_section()
            self._build_robot_links_section()
            self._build_bottom_section()

    # --- Attribute helpers ---
    def _get_attr_str(self, attr_def: object) -> str:
        """Read a string attribute from the prim, returning '' if missing."""
        attr = self._prim.GetAttribute(attr_def.name)
        if attr and attr.Get() is not None:
            return str(attr.Get())
        return ""

    def _get_allowed_tokens(self, attr_def: object) -> list[str]:
        """Read the allowedTokens metadata from a token attribute on the prim."""
        attr = self._prim.GetAttribute(attr_def.name)
        if attr:
            tokens = attr.GetMetadata("allowedTokens")
            if tokens is not None:
                return [str(t) for t in tokens]
        return []

    def _set_attr_str(self, attr_def: object, value: str) -> None:
        """Write a string attribute on the prim."""
        attr = self._prim.GetAttribute(attr_def.name)
        if attr:
            attr.Set(value)

    def _get_attr_str_array(self, attr_def: object) -> list[str]:
        """Read a string array attribute from the prim, returning [] if missing."""
        attr = self._prim.GetAttribute(attr_def.name)
        if attr and attr.Get() is not None:
            return list(attr.Get())
        return []

    def _set_attr_str_array(self, attr_def: object, values: list[str]) -> None:
        """Write a string array attribute on the prim."""
        attr = self._prim.GetAttribute(attr_def.name)
        if attr:
            attr.Set(Vt.StringArray(values))

    def _get_relationship_targets(self, rel_def: object) -> list[str]:
        """Read relationship targets as a list of path strings."""
        rel = self._prim.GetRelationship(rel_def.name)
        if rel:
            return [str(t) for t in rel.GetTargets()]
        return []

    def _set_relationship_targets(self, rel_def: object, targets: list[str]) -> None:
        """Write relationship targets from a list of path strings."""
        rel = self._prim.GetRelationship(rel_def.name)
        if rel:
            rel.SetTargets([Sdf.Path(t) for t in targets])

    # --- Sub-robot helpers ---
    def _target_has_robot_api(self, path: str) -> bool:
        """Returns True if the prim at the given path has Robot API applied.

        Args:
            path: USD prim path to check.
        """
        stage = self._payload.get_stage()
        if not stage:
            return False
        prim = stage.GetPrimAtPath(path)
        return bool(prim and prim.IsValid() and prim.HasAPI(robot_schema.Classes.ROBOT_API.value))

    def _get_sub_robot_targets(self, path: str, rel_def: object) -> list[str]:
        """Returns relationship targets from a sub-robot prim.

        Args:
            path: USD prim path of the sub-robot.
            rel_def: The relationship definition to query (e.g. ROBOT_JOINTS, ROBOT_LINKS).

        Returns:
            List of target path strings, empty if the prim or relationship doesn't exist.
        """
        stage = self._payload.get_stage()
        if not stage:
            return []
        prim = stage.GetPrimAtPath(path)
        if not prim or not prim.IsValid():
            return []
        rel = prim.GetRelationship(rel_def.name)
        return [str(t) for t in rel.GetTargets()] if rel else []

    def _count_sub_robot_descendants(self, path: str, rel_def: object, visited: set | None = None) -> int:
        """Recursively counts all descendant items under a sub-robot for a given relationship type.

        Args:
            path: USD path of the sub-robot prim.
            rel_def: The relationship definition (ROBOT_JOINTS or ROBOT_LINKS).
            visited: Set of already-visited paths to prevent infinite recursion.

        Returns:
            Total number of descendant items (including nested sub-robots' children).
        """
        if visited is None:
            visited = set()
        if path in visited:
            return 0
        visited = visited | {path}
        children = self._get_sub_robot_targets(path, rel_def)
        count = len(children)
        for child_path in children:
            if self._target_has_robot_api(child_path):
                count += self._count_sub_robot_descendants(child_path, rel_def, visited)
        return count

    _MAX_EXPAND_DEPTH = 4

    @staticmethod
    def _display_name_for_path(path: str) -> str:
        """Returns the last path segment as a human-readable display name."""
        return path.rsplit("/", 1)[-1] if "/" in path else path

    def _navigate_to_prim(self, path: str) -> None:
        """Selects the prim at the given path in the USD stage and frames it in the viewport."""
        ctx = omni.usd.get_context()
        if not ctx:
            return
        selection = ctx.get_selection()
        if not selection:
            return
        if not selection.set_selected_prim_paths([path], True):
            carb.log_warn(f"Failed to select prim at {path}")

    # --- Field row helper ---
    def _field_row(self, label_text: str, widget_builder: callable) -> None:
        """One row: label left, input widget right."""
        with ui.HStack(height=_ROW_HEIGHT):
            ui.Spacer(width=4)
            ui.Label(
                label_text,
                width=_FIELD_LABEL_WIDTH,
                height=_ROW_HEIGHT,
                style={"color": _TEXT_PRIMARY, "font_size": 12},
            )
            ui.Spacer(width=8)
            with ui.HStack(width=ui.Fraction(1), height=_ROW_HEIGHT):
                widget_builder()
            ui.Spacer(width=4)

    # --- Robot Schema section (scalar attrs) ---
    def _build_robot_schema_section(self) -> None:
        """Builds the collapsible section for scalar robot attributes."""
        with ui.CollapsableFrame("Robot Schema", height=0, style=style.COLLAPSABLE_FRAME_STYLE):
            with ui.ZStack(height=0):
                ui.Rectangle(style={"background_color": _BG_PANEL})
                with ui.VStack(height=0, spacing=4):
                    ui.Spacer(height=8)
                    self._field_row(
                        "Description", lambda: self._build_string_field(robot_schema.Attributes.DESCRIPTION)
                    )
                    self._field_row("License", lambda: self._build_license_combo())
                    self._field_row("Namespace", lambda: self._build_string_field(robot_schema.Attributes.NAMESPACE))
                    self._field_row("Robot Type", self._build_robot_type_field)
                    self._field_row("Source", lambda: self._build_string_field(robot_schema.Attributes.SOURCE))
                    self._field_row("Version", lambda: self._build_string_field(robot_schema.Attributes.VERSION))
                    self._build_changelog_section()
                    ui.Spacer(height=8)

    def _build_string_field(self, attr_def: object) -> None:
        """Creates a StringField bound to a USD attribute."""
        field = ui.StringField(height=20)
        field.model.set_value(self._get_attr_str(attr_def))
        field.model.add_end_edit_fn(lambda m, a=attr_def: self._set_attr_str(a, m.get_value_as_string()))

    def _build_license_combo(self) -> None:
        """Creates a ComboBox for the license attribute."""
        tokens = self._get_allowed_tokens(robot_schema.Attributes.LICENSE)
        current = self._get_attr_str(robot_schema.Attributes.LICENSE)
        idx = tokens.index(current) if current in tokens else 0
        combo = ui.ComboBox(idx, *tokens, height=22, style=_COMBO_STYLE)
        combo.model.add_item_changed_fn(
            lambda m, _, t=tokens: self._set_attr_str(
                robot_schema.Attributes.LICENSE, t[m.get_item_value_model().get_value_as_int()]
            )
        )

    # --- Joints section ---
    def _build_robot_joints_section(self) -> None:
        """Builds the collapsible section with the drag-reorderable joints list."""
        with ui.CollapsableFrame("Robot Joints", height=0, style=style.COLLAPSABLE_FRAME_STYLE):
            self._joints_list_frame = ui.Frame(height=0, width=ui.Fraction(1), build_fn=self._build_joints_list_content)

    def _build_joints_list_content(self) -> None:
        """Builds the scrollable joint list with drag-reorder, add/remove, and expandable sub-robot rows."""
        joints = self._get_relationship_targets(robot_schema.Relations.ROBOT_JOINTS)
        self._joint_drop_indicators = []
        self._joint_row_rects = []

        with ui.VStack(height=0, width=ui.Fraction(1), spacing=0):
            with ui.ZStack(height=ui.Pixel(_LIST_FIXED_HEIGHT), width=ui.Fraction(1)):
                ui.Rectangle(style={"background_color": _BG_INPUT})
                self._joints_scroll_frame = ui.ScrollingFrame(
                    height=ui.Pixel(_LIST_FIXED_HEIGHT),
                    horizontal_scrollbar_policy=ui.ScrollBarPolicy.SCROLLBAR_ALWAYS_OFF,
                    vertical_scrollbar_policy=ui.ScrollBarPolicy.SCROLLBAR_AS_NEEDED,
                    style={"ScrollingFrame": {"background_color": _BG_INPUT}},
                )
                with self._joints_scroll_frame:
                    with ui.VStack(height=0, width=ui.Fraction(1), spacing=0):
                        ui.Spacer(height=2)
                        display_counter = 0
                        for i, path in enumerate(joints):
                            is_sub_robot = self._target_has_robot_api(path)
                            is_expanded = i in self._expanded_joints
                            self._build_list_row(
                                i,
                                path,
                                selected=(self._selected_joint_index == i),
                                select_fn=partial(self._on_joint_row_clicked, i),
                                remove_fn=partial(self._on_remove_joint, i),
                                drag_prefix="joint",
                                drop_accept_fn=partial(self._show_joint_drop_indicator, i),
                                drop_fn=partial(self._on_reorder_joint, i),
                                has_children=is_sub_robot,
                                is_expanded=is_expanded,
                                toggle_expand_fn=partial(self._toggle_joint_expand, i),
                                display_index=None if is_sub_robot else display_counter,
                            )
                            if is_sub_robot:
                                child_count = self._count_sub_robot_descendants(
                                    path, robot_schema.Relations.ROBOT_JOINTS
                                )
                                if is_expanded:
                                    self._build_sub_robot_children(
                                        path,
                                        robot_schema.Relations.ROBOT_JOINTS,
                                        indent_level=1,
                                        visited=set(),
                                        counter=display_counter,
                                    )
                                display_counter += child_count
                            else:
                                display_counter += 1
                        end_idx = len(joints)
                        with ui.ZStack(height=ui.Pixel(_TABLE_ROW_HEIGHT), width=ui.Fraction(1)) as end_drop:
                            indicator = ui.Rectangle(
                                height=ui.Pixel(2),
                                width=ui.Fraction(1),
                                style={"background_color": _DROP_INDICATOR_TRANSPARENT},
                            )
                            self._joint_drop_indicators.append(indicator)
                            ui.Spacer()
                        end_drop.set_accept_drop_fn(partial(self._show_joint_drop_indicator, end_idx))
                        end_drop.set_drop_fn(partial(self._on_reorder_joint, end_idx))
            with ui.HStack(height=0, width=ui.Fraction(1)):
                ui.Spacer(width=2)
                ui.Button(
                    "Add Joint",
                    height=26,
                    clicked_fn=self._on_add_joint,
                    style={"Button": {"background_color": style.BUTTON_BG, "border_radius": 2}},
                )
                ui.Spacer(width=2)

    # --- Links section ---
    def _build_robot_links_section(self) -> None:
        """Builds the collapsible section with the drag-reorderable links list."""
        with ui.CollapsableFrame("Robot Links", height=0, style=style.COLLAPSABLE_FRAME_STYLE):
            self._links_list_frame = ui.Frame(height=0, width=ui.Fraction(1), build_fn=self._build_links_list_content)

    def _build_links_list_content(self) -> None:
        """Builds the scrollable link list with drag-reorder, add/remove, and expandable sub-robot rows."""
        links = self._get_relationship_targets(robot_schema.Relations.ROBOT_LINKS)
        self._link_drop_indicators = []
        self._link_row_rects = []

        with ui.VStack(height=0, width=ui.Fraction(1), spacing=0):
            with ui.ZStack(height=ui.Pixel(_LIST_FIXED_HEIGHT), width=ui.Fraction(1)):
                ui.Rectangle(style={"background_color": _BG_INPUT})
                self._links_scroll_frame = ui.ScrollingFrame(
                    height=ui.Pixel(_LIST_FIXED_HEIGHT),
                    horizontal_scrollbar_policy=ui.ScrollBarPolicy.SCROLLBAR_ALWAYS_OFF,
                    vertical_scrollbar_policy=ui.ScrollBarPolicy.SCROLLBAR_AS_NEEDED,
                    style={"ScrollingFrame": {"background_color": _BG_INPUT}},
                )
                with self._links_scroll_frame:
                    with ui.VStack(height=0, width=ui.Fraction(1), spacing=0):
                        ui.Spacer(height=2)
                        display_counter = 0
                        for i, path in enumerate(links):
                            is_sub_robot = self._target_has_robot_api(path)
                            is_expanded = i in self._expanded_links
                            self._build_list_row(
                                i,
                                path,
                                selected=(self._selected_link_index == i),
                                select_fn=partial(self._on_link_row_clicked, i),
                                remove_fn=partial(self._on_remove_link, i),
                                drag_prefix="link",
                                drop_accept_fn=partial(self._show_link_drop_indicator, i),
                                drop_fn=partial(self._on_reorder_link, i),
                                has_children=is_sub_robot,
                                is_expanded=is_expanded,
                                toggle_expand_fn=partial(self._toggle_link_expand, i),
                                display_index=None if is_sub_robot else display_counter,
                            )
                            if is_sub_robot:
                                child_count = self._count_sub_robot_descendants(
                                    path, robot_schema.Relations.ROBOT_LINKS
                                )
                                if is_expanded:
                                    self._build_sub_robot_children(
                                        path,
                                        robot_schema.Relations.ROBOT_LINKS,
                                        indent_level=1,
                                        visited=set(),
                                        counter=display_counter,
                                    )
                                display_counter += child_count
                            else:
                                display_counter += 1
                        end_idx = len(links)
                        with ui.ZStack(height=ui.Pixel(_TABLE_ROW_HEIGHT), width=ui.Fraction(1)) as end_drop:
                            indicator = ui.Rectangle(
                                height=ui.Pixel(2),
                                width=ui.Fraction(1),
                                style={"background_color": _DROP_INDICATOR_TRANSPARENT},
                            )
                            self._link_drop_indicators.append(indicator)
                            ui.Spacer()
                        end_drop.set_accept_drop_fn(partial(self._show_link_drop_indicator, end_idx))
                        end_drop.set_drop_fn(partial(self._on_reorder_link, end_idx))
            with ui.HStack(height=0, width=ui.Fraction(1)):
                ui.Spacer(width=2)
                ui.Button(
                    "Add Link",
                    height=26,
                    clicked_fn=self._on_add_link,
                    style={"Button": {"background_color": style.BUTTON_BG, "border_radius": 2}},
                )
                ui.Spacer(width=2)

    # --- Bottom section (Re-Calculate) ---
    def _build_bottom_section(self) -> None:
        """Builds the bottom section with the recalculate button and a force-update toggle."""
        with ui.VStack(height=0, spacing=6):
            with ui.HStack(height=28, spacing=6):
                ui.Spacer(width=4)
                ui.Button(
                    "Re-Calculate Robot Tree",
                    height=28,
                    width=ui.Fraction(1),
                    style={"Button": {"background_color": _BG_INPUT}},
                    clicked_fn=self._on_recalculate_tree,
                )
                # Vertically-centered checkbox + label cluster (fixed width)
                with ui.VStack(width=0, height=28):
                    ui.Spacer(height=ui.Fraction(1))
                    with ui.HStack(
                        height=18,
                        spacing=4,
                        tooltip=(
                            "Discard the existing robotLinks/robotJoints order "
                            "and rewrite both relationships from scratch."
                        ),
                        style=_TOOLTIP_STYLE,
                    ):
                        force_chk = ui.CheckBox(width=18, height=18)
                        force_chk.model.set_value(self._force_update_recalculate)
                        force_chk.model.add_value_changed_fn(self._on_force_update_changed)
                        ui.Label(
                            "Force Update",
                            width=0,
                            height=18,
                            style={"color": _TEXT_PRIMARY, "font_size": 12},
                        )
                    ui.Spacer(height=ui.Fraction(1))
                ui.Spacer(width=4)
            with ui.HStack(
                height=28,
                spacing=6,
                tooltip=(
                    "Flush the current robotLinks/robotJoints order to the layer "
                    "that authors IsaacRobotAPI, writing delta prepends and matching "
                    "deletes so opinions from other layers (references, attachments, "
                    "sublayers) stay untouched."
                ),
                style=_TOOLTIP_STYLE,
            ):
                ui.Spacer(width=4)
                ui.Button(
                    "Save to Robot Layer",
                    height=28,
                    width=ui.Fraction(1),
                    style={"Button": {"background_color": _BG_INPUT}},
                    clicked_fn=self._on_save_to_robot_layer,
                )
                ui.Spacer(width=4)
            ui.Spacer(height=8)

    def _on_force_update_changed(self, model: ui.AbstractValueModel) -> None:
        """Persist the Force Update checkbox state on the widget."""
        self._force_update_recalculate = bool(model.get_value_as_bool())

    def _build_robot_type_field(self) -> None:
        """Creates the robot type combo with (Other) support inside a rebuildable frame."""
        self._robot_type_frame = ui.Frame(height=0, width=ui.Fraction(1), build_fn=self._build_robot_type_content)

    def _build_robot_type_content(self) -> None:
        """Builds the robot type combo, optionally followed by a text field for custom values."""
        self._robot_type_tokens = self._get_allowed_tokens(robot_schema.Attributes.ROBOT_TYPE)
        current = self._get_attr_str(robot_schema.Attributes.ROBOT_TYPE)
        options = self._robot_type_tokens + ["(Other)"]
        is_other = self._robot_type_other_mode or current not in self._robot_type_tokens
        idx = len(self._robot_type_tokens) if is_other else self._robot_type_tokens.index(current)

        with ui.HStack(height=_ROW_HEIGHT, spacing=4):
            combo = ui.ComboBox(
                idx,
                *options,
                height=22,
                style=_COMBO_STYLE,
                width=ui.Pixel(160) if is_other else ui.Fraction(1),
            )
            combo.model.add_item_changed_fn(lambda m, _: self._on_robot_type_combo_changed(m))
            if is_other:
                field = ui.StringField(height=20, width=ui.Fraction(1))
                field.model.set_value(current if current else "")
                field.model.add_end_edit_fn(
                    lambda m: self._set_attr_str(robot_schema.Attributes.ROBOT_TYPE, m.get_value_as_string())
                )

    def _on_robot_type_combo_changed(self, model: object) -> None:
        """Handles robot type combo selection changes."""
        idx = model.get_item_value_model().get_value_as_int()
        options = self._robot_type_tokens + ["(Other)"]
        selected = options[idx]
        if selected == "(Other)":
            self._robot_type_other_mode = True
            self._set_attr_str(robot_schema.Attributes.ROBOT_TYPE, "")
        else:
            self._robot_type_other_mode = False
            self._set_attr_str(robot_schema.Attributes.ROBOT_TYPE, selected)
        if self._robot_type_frame:
            self._robot_type_frame.rebuild()

    # --- Changelog list ---
    def _build_changelog_section(self) -> None:
        """Builds the changelog label with a + button and the list of editable entries."""
        with ui.HStack(height=_ROW_HEIGHT):
            ui.Spacer(width=4)
            ui.Label(
                "Changelog",
                width=_FIELD_LABEL_WIDTH,
                height=_ROW_HEIGHT,
                style={"color": _TEXT_PRIMARY, "font_size": 12},
            )
            ui.Spacer(width=ui.Fraction(1))
            _add_icon = str(ICON_PATH.joinpath("plus.svg"))
            with ui.ZStack(width=24, height=24):
                ui.Rectangle(style={"background_color": _BG_PANEL, "border_radius": 2})
                hover_rect = ui.Rectangle(
                    visible=False,
                    style={"background_color": style.BG_HEADER_HOVER, "border_radius": 2},
                )
                with ui.VStack():
                    ui.Spacer()
                    with ui.HStack(height=12):
                        ui.Spacer()
                        ui.Image(_add_icon, width=12, height=12, style={"color": 0xFFFFFFFF})
                        ui.Spacer()
                    ui.Spacer()
                add_btn = ui.InvisibleButton()
                add_btn.set_clicked_fn(self._on_add_changelog_entry)
                add_btn.set_mouse_hovered_fn(lambda hovered, _hr=hover_rect: setattr(_hr, "visible", hovered))
            ui.Spacer(width=4)
        self._changelog_frame = ui.Frame(height=0, build_fn=self._build_changelog_entries)

    def _build_changelog_entries(self) -> None:
        """Builds individual editable rows for each changelog entry."""
        entries = self._get_attr_str_array(robot_schema.Attributes.CHANGELOG)
        if not entries:
            ui.Spacer(height=0)
            return
        _remove_icon = str(ICON_PATH.joinpath("remove.svg"))
        with ui.VStack(height=0, spacing=4):
            for i, entry in enumerate(entries):
                with ui.HStack(height=_ROW_HEIGHT, spacing=0):
                    ui.Spacer(width=4 + _FIELD_LABEL_WIDTH + 8)
                    field = ui.StringField(height=20, width=ui.Fraction(1))
                    field.model.set_value(entry)
                    field.model.add_end_edit_fn(
                        lambda m, idx=i: self._on_changelog_entry_changed(idx, m.get_value_as_string())
                    )
                    ui.Button(
                        "",
                        width=24,
                        height=24,
                        clicked_fn=partial(self._on_remove_changelog_entry, i),
                        style={
                            "Button.Image": {
                                "image_url": _remove_icon,
                                "color": 0xFFFFFFFF,
                                "alignment": ui.Alignment.CENTER,
                            },
                        },
                    )
                    ui.Spacer(width=4)

    def _on_add_changelog_entry(self) -> None:
        """Prepends a new empty changelog entry and rebuilds the list."""
        entries = self._get_attr_str_array(robot_schema.Attributes.CHANGELOG)
        entries.insert(0, "")
        self._set_attr_str_array(robot_schema.Attributes.CHANGELOG, entries)
        if self._changelog_frame:
            self._changelog_frame.rebuild()

    def _on_remove_changelog_entry(self, index: int) -> None:
        """Removes a changelog entry at the given index and rebuilds the list."""
        entries = self._get_attr_str_array(robot_schema.Attributes.CHANGELOG)
        if 0 <= index < len(entries):
            entries.pop(index)
            self._set_attr_str_array(robot_schema.Attributes.CHANGELOG, entries)
            if self._changelog_frame:
                self._changelog_frame.rebuild()

    def _on_changelog_entry_changed(self, index: int, value: str) -> None:
        """Updates a changelog entry at the given index."""
        entries = self._get_attr_str_array(robot_schema.Attributes.CHANGELOG)
        if 0 <= index < len(entries):
            entries[index] = value
            self._set_attr_str_array(robot_schema.Attributes.CHANGELOG, entries)

    def _on_recalculate_tree(self) -> None:
        """Re-discover joints and links from the articulation and update the schema relationships."""
        if not self._prim:
            return
        stage = self._payload.get_stage() if self._payload else omni.usd.get_context().get_stage()
        if not stage:
            return
        from usd.schema.isaac.robot_schema import utils as robot_utils

        try:
            robot_utils.RecalculateRobotSchema(stage, self._prim, force_update=self._force_update_recalculate)
        except ValueError:
            return
        self._rebuild_joints_list()
        self._rebuild_links_list()

    def _on_save_to_robot_layer(self) -> None:
        """Flush the current robotLinks/robotJoints to the IsaacRobotAPI-authoring layer."""
        if not self._prim:
            return
        stage = self._payload.get_stage() if self._payload else omni.usd.get_context().get_stage()
        if not stage:
            return
        from usd.schema.isaac.robot_schema import utils as robot_utils

        try:
            written_layer = robot_utils.SaveRobotSchemaToRobotLayer(stage, self._prim)
        except ValueError:
            return
        if written_layer is None:
            carb.log_warn(
                f"Save to Robot Layer: could not find a layer authoring IsaacRobotAPI on {self._prim.GetPath()}; "
                "nothing was written."
            )
            return
        carb.log_info(f"Save to Robot Layer: wrote robot schema to {written_layer.identifier}")

    # --- Shared list row builder ---
    _INDENT_PX = 16
    _EXPAND_TRIANGLE_WIDTH = 14

    def _build_list_row(
        self,
        row_index: int,
        path: str,
        selected: bool,
        select_fn: callable,
        remove_fn: callable,
        drag_prefix: str,
        drop_accept_fn: callable,
        drop_fn: callable,
        indent_level: int = 0,
        has_children: bool = False,
        is_expanded: bool = False,
        toggle_expand_fn: callable | None = None,
        display_index: int | None = None,
    ) -> None:
        """Builds a single row in the joints or links list.

        When has_children is True, a disclosure triangle is shown at the left edge of the label
        cell (not in the index cell), and the index cell is left blank.

        Args:
            row_index: 0-based index of the row in the underlying list (used for drag/drop and remove).
            path: USD prim path displayed in the row.
            selected: Whether this row is the currently selected list entry.
            select_fn: Callback invoked when the row is clicked (receives ``row_index``).
            remove_fn: Callback invoked when the row's remove button is clicked (receives ``row_index``).
            drag_prefix: Namespace string (``"joint"`` or ``"link"``) used for drag/drop payload keys.
            drop_accept_fn: Callback returning True when a drag payload may be dropped on this row.
            drop_fn: Callback invoked on drop, receiving the drag payload string.
            indent_level: Nesting depth for sub-robot rows; multiplied by ``_INDENT_PX`` for visual indent.
            has_children: If True, renders a disclosure triangle for expandable sub-robot rows.
            is_expanded: If ``has_children`` is True, whether the row is currently expanded.
            toggle_expand_fn: Callback invoked when the disclosure triangle is clicked.
            display_index: 0-based display index shown in the index cell, or None to leave the cell blank
                (used for sub-robot child rows).
        """
        _SELECTED_BG = style.SELECTED_BG
        DROP_ZONE_HEIGHT = _TABLE_ROW_HEIGHT + _TABLE_ROW_GAP
        indent_px = indent_level * self._INDENT_PX

        with ui.ZStack(height=ui.Pixel(DROP_ZONE_HEIGHT), width=ui.Fraction(1)) as drop_zone:
            with ui.VStack(spacing=0):
                indicator = ui.Rectangle(
                    height=ui.Pixel(2),
                    width=ui.Fraction(1),
                    style={"background_color": _DROP_INDICATOR_TRANSPARENT},
                )
                if drag_prefix == "joint":
                    self._joint_drop_indicators.append(indicator)
                else:
                    self._link_drop_indicators.append(indicator)
                with ui.HStack(height=ui.Pixel(_TABLE_ROW_HEIGHT), width=ui.Fraction(1), spacing=2):
                    if indent_px > 0:
                        ui.Spacer(width=ui.Pixel(indent_px))
                    with ui.ZStack(width=ui.Pixel(21), height=ui.Pixel(_TABLE_ROW_HEIGHT)):
                        ui.Rectangle(
                            style={"background_color": _CELL_BG, "border_radius": 2, "corner_flag": ui.CornerFlag.LEFT}
                        )
                        if display_index is not None:
                            ui.Label(
                                str(display_index),
                                alignment=ui.Alignment.CENTER,
                                style={"color": _TEXT_DIM, "font_size": _LIST_FONT_SIZE},
                            )
                    self._build_grab_handle(
                        f"{drag_prefix}:{row_index}", str(display_index if display_index is not None else ""), path
                    )
                    with ui.ZStack(
                        width=ui.Fraction(1), height=ui.Pixel(_TABLE_ROW_HEIGHT), tooltip=path, style=_TOOLTIP_STYLE
                    ):
                        text_bg_rect = ui.Rectangle(
                            style={
                                "background_color": _SELECTED_BG if selected else _CELL_BG,
                                "border_radius": 2,
                            }
                        )
                        if drag_prefix == "joint":
                            self._joint_row_rects.append(text_bg_rect)
                        else:
                            self._link_row_rects.append(text_bg_rect)
                        with ui.HStack(height=ui.Pixel(_TABLE_ROW_HEIGHT), spacing=0):
                            if has_children:
                                ui.Spacer(width=ui.Pixel(4))
                                tri_w = 7 if is_expanded else 5
                                tri_h = 5 if is_expanded else 7
                                tri_align = ui.Alignment.CENTER_BOTTOM if is_expanded else ui.Alignment.RIGHT_CENTER
                                with ui.ZStack(
                                    width=ui.Pixel(self._EXPAND_TRIANGLE_WIDTH), height=ui.Pixel(_TABLE_ROW_HEIGHT)
                                ):
                                    with ui.VStack():
                                        ui.Spacer()
                                        ui.Triangle(
                                            width=tri_w,
                                            height=tri_h,
                                            alignment=tri_align,
                                            style={"background_color": _TEXT_PRIMARY},
                                        )
                                        ui.Spacer()
                                    if toggle_expand_fn:
                                        expand_btn = ui.InvisibleButton()
                                        expand_btn.set_clicked_fn(toggle_expand_fn)
                            btn = ui.InvisibleButton(
                                mouse_double_clicked_fn=lambda x, y, b, m, p=path: self._navigate_to_prim(p),
                            )
                            btn.set_clicked_fn(select_fn)
                        label_margin = (self._EXPAND_TRIANGLE_WIDTH + 6) if has_children else 4
                        display_name = self._display_name_for_path(path)
                        ui.Label(
                            display_name,
                            alignment=ui.Alignment.LEFT_CENTER,
                            style={"color": _TEXT_PRIMARY, "font_size": _LIST_FONT_SIZE, "margin_width": label_margin},
                        )
                    _remove_icon = str(ICON_PATH.joinpath("remove.svg"))
                    with ui.ZStack(width=ui.Pixel(26), height=ui.Pixel(26)):
                        ui.Rectangle(style={"background_color": style.BUTTON_BG})
                        with ui.VStack():
                            ui.Spacer(height=ui.Pixel(1))
                            with ui.HStack(height=ui.Pixel(24)):
                                ui.Spacer(width=ui.Pixel(1))
                                ui.Button(
                                    "",
                                    width=24,
                                    height=24,
                                    clicked_fn=remove_fn,
                                    style={
                                        "Button": {"background_color": _CELL_BG, "border_radius": 2},
                                        "Button.Image": {
                                            "image_url": _remove_icon,
                                            "color": 0xFFFFFFFF,
                                            "alignment": ui.Alignment.CENTER,
                                        },
                                    },
                                )
                                ui.Spacer(width=ui.Pixel(1))
                            ui.Spacer(height=ui.Pixel(1))
            drop_zone.set_accept_drop_fn(drop_accept_fn)
            drop_zone.set_drop_fn(drop_fn)

    def _build_child_row(self, display_index: int, path: str, indent_level: int = 1) -> None:
        """Builds a read-only child row for a sub-robot's relationship target.

        Same visual style as parent rows (index cell + label cell) but without the drag handle
        or remove button. No click interaction.

        Args:
            display_index: The 0-based display index for the child row.
            path: USD prim path to display.
            indent_level: Nesting depth for indentation.
        """
        indent_px = indent_level * self._INDENT_PX
        index_text = str(display_index)

        with ui.VStack(height=0, spacing=0):
            ui.Spacer(height=ui.Pixel(_TABLE_ROW_GAP))
            with ui.HStack(height=ui.Pixel(_TABLE_ROW_HEIGHT), width=ui.Fraction(1), spacing=2):
                if indent_px > 0:
                    ui.Spacer(width=ui.Pixel(indent_px))
                with ui.ZStack(width=ui.Pixel(21), height=ui.Pixel(_TABLE_ROW_HEIGHT)):
                    ui.Rectangle(
                        style={"background_color": _CELL_BG, "border_radius": 2, "corner_flag": ui.CornerFlag.LEFT}
                    )
                    ui.Label(
                        index_text,
                        alignment=ui.Alignment.CENTER,
                        style={"color": _TEXT_DIM, "font_size": _LIST_FONT_SIZE},
                    )
                with ui.ZStack(
                    width=ui.Fraction(1), height=ui.Pixel(_TABLE_ROW_HEIGHT), tooltip=path, style=_TOOLTIP_STYLE
                ):
                    ui.Rectangle(style={"background_color": _CELL_BG, "border_radius": 2})
                    dbl_click_btn = ui.InvisibleButton(
                        mouse_double_clicked_fn=lambda x, y, b, m, p=path: self._navigate_to_prim(p),
                    )
                    display_name = self._display_name_for_path(path)
                    ui.Label(
                        display_name,
                        alignment=ui.Alignment.LEFT_CENTER,
                        style={"color": _TEXT_PRIMARY, "font_size": _LIST_FONT_SIZE, "margin_width": 4},
                    )

    def _build_grab_handle(self, drag_data: str, index_text: str, path: str) -> None:
        """Builds a drag grab handle (3 vertical bars)."""
        GRIP_COLOR = style.GRIP_COLOR
        with ui.ZStack(width=ui.Pixel(14), height=ui.Pixel(_TABLE_ROW_HEIGHT)) as handle:
            ui.Rectangle(style={"background_color": _CELL_BG})
            with ui.VStack():
                ui.Spacer(height=ui.Fraction(1))
                with ui.HStack(height=ui.Pixel(17)):
                    ui.Spacer(width=ui.Fraction(1))
                    for i in range(3):
                        ui.Rectangle(
                            width=ui.Pixel(2),
                            height=ui.Pixel(17),
                            style={"background_color": GRIP_COLOR, "border_radius": 1},
                        )
                        if i < 2:
                            ui.Spacer(width=ui.Pixel(1))
                    ui.Spacer(width=ui.Fraction(1))
                ui.Spacer(height=ui.Fraction(1))

            def _on_drag(d: str = drag_data) -> str:
                return d

            handle.set_drag_fn(_on_drag)

    # --- Expand / collapse sub-robot rows ---
    def _toggle_joint_expand(self, index: int) -> None:
        """Toggle expansion of a joint row that has Robot API."""
        if index in self._expanded_joints:
            self._expanded_joints.discard(index)
        else:
            self._expanded_joints.add(index)
        self._rebuild_joints_list()

    def _toggle_link_expand(self, index: int) -> None:
        """Toggle expansion of a link row that has Robot API."""
        if index in self._expanded_links:
            self._expanded_links.discard(index)
        else:
            self._expanded_links.add(index)
        self._rebuild_links_list()

    def _build_sub_robot_children(
        self,
        parent_path: str,
        rel_def: object,
        indent_level: int,
        visited: set,
        counter: int = 0,
    ) -> int:
        """Recursively builds read-only child rows for a sub-robot's matching relationship targets.

        Only shows the relationship type matching the parent list (joints in the joints list,
        links in the links list). Returns the running display counter so subsequent rows can
        resume numbering.

        Args:
            parent_path: USD path of the sub-robot prim.
            rel_def: The relationship definition to display (ROBOT_JOINTS or ROBOT_LINKS).
            indent_level: Current nesting depth for indentation.
            visited: Set of already-visited paths to prevent infinite recursion.
            counter: The starting 0-based display index for child rows.

        Returns:
            The next display index after all children have been built.
        """
        if indent_level > self._MAX_EXPAND_DEPTH or parent_path in visited:
            return counter
        visited = visited | {parent_path}

        children = self._get_sub_robot_targets(parent_path, rel_def)
        for child_path in children:
            self._build_child_row(counter, child_path, indent_level=indent_level)
            counter += 1
            if self._target_has_robot_api(child_path):
                counter = self._build_sub_robot_children(child_path, rel_def, indent_level + 1, visited, counter)
        return counter

    # --- Scroll-preserving rebuild ---
    def _rebuild_joints_list(self) -> None:
        """Rebuild the joints list frame, restoring scroll position after the rebuild."""
        scroll_y = self._joints_scroll_frame.scroll_y if self._joints_scroll_frame else 0
        if self._joints_list_frame:
            self._joints_list_frame.rebuild()
        if scroll_y and self._joints_scroll_frame:
            import asyncio

            async def _restore() -> None:
                app = omni.kit.app.get_app()
                await app.next_update_async()
                await app.next_update_async()
                if self._joints_scroll_frame:
                    self._joints_scroll_frame.scroll_y = scroll_y

            asyncio.ensure_future(_restore())

    def _rebuild_links_list(self) -> None:
        """Rebuild the links list frame, restoring scroll position after the rebuild."""
        scroll_y = self._links_scroll_frame.scroll_y if self._links_scroll_frame else 0
        if self._links_list_frame:
            self._links_list_frame.rebuild()
        if scroll_y and self._links_scroll_frame:
            import asyncio

            async def _restore() -> None:
                app = omni.kit.app.get_app()
                await app.next_update_async()
                await app.next_update_async()
                if self._links_scroll_frame:
                    self._links_scroll_frame.scroll_y = scroll_y

            asyncio.ensure_future(_restore())

    # --- Joint/Link list interactions ---
    def _on_joint_row_clicked(self, index: int) -> None:
        self._selected_joint_index = index
        for i, rect in enumerate(self._joint_row_rects):
            rect.set_style({"background_color": style.SELECTED_BG if i == index else _CELL_BG, "border_radius": 2})

    def _on_link_row_clicked(self, index: int) -> None:
        self._selected_link_index = index
        for i, rect in enumerate(self._link_row_rects):
            rect.set_style({"background_color": style.SELECTED_BG if i == index else _CELL_BG, "border_radius": 2})

    def _show_joint_drop_indicator(self, index: int, data: object) -> bool:
        if not str(data).startswith("joint:"):
            return False
        for i, ind in enumerate(self._joint_drop_indicators):
            ind.set_style({"background_color": _DROP_INDICATOR_COLOR if i == index else _DROP_INDICATOR_TRANSPARENT})
        return True

    def _show_link_drop_indicator(self, index: int, data: object) -> bool:
        if not str(data).startswith("link:"):
            return False
        for i, ind in enumerate(self._link_drop_indicators):
            ind.set_style({"background_color": _DROP_INDICATOR_COLOR if i == index else _DROP_INDICATOR_TRANSPARENT})
        return True

    def _on_add_joint(self) -> None:
        """Opens a stage prim picker filtered to JointAPI / RobotAPI prims."""
        stage = self._payload.get_stage() if self._payload else omni.usd.get_context().get_stage()
        if not stage:
            return
        from omni.kit.property.usd.relationship import RelationshipTargetPicker

        # Close any previously opened picker so repeatedly clicking Add Joint
        # does not leak dangling `RelationshipTargetPicker` instances.
        if self._joint_picker is not None:
            self._joint_picker.clean()
        self._joint_picker = RelationshipTargetPicker(
            stage,
            [],
            lambda prim: (
                prim.HasAPI(robot_schema.Classes.JOINT_API.value) or prim.HasAPI(robot_schema.Classes.ROBOT_API.value)
            ),
            {"target_name": "Joint", "target_plural_name": "Joints"},
        )
        self._joint_picker.show(0, on_targets_selected=self._on_joint_targets_selected)

    def _on_joint_targets_selected(self, paths: list) -> None:
        """Appends picker-selected joint paths to the relationship, skipping duplicates."""
        if not paths:
            return
        joints = self._get_relationship_targets(robot_schema.Relations.ROBOT_JOINTS)
        existing = set(joints)
        for path in paths:
            if path not in existing:
                joints.append(path)
        self._set_relationship_targets(robot_schema.Relations.ROBOT_JOINTS, joints)
        self._rebuild_joints_list()

    def _on_add_link(self) -> None:
        """Opens a stage prim picker filtered to LinkAPI / SiteAPI / RobotAPI prims."""
        stage = self._payload.get_stage() if self._payload else omni.usd.get_context().get_stage()
        if not stage:
            return
        from omni.kit.property.usd.relationship import RelationshipTargetPicker

        # Close any previously opened picker so repeatedly clicking Add Link
        # does not leak dangling `RelationshipTargetPicker` instances.
        if self._link_picker is not None:
            self._link_picker.clean()
        self._link_picker = RelationshipTargetPicker(
            stage,
            [],
            lambda prim: (
                prim.HasAPI(robot_schema.Classes.LINK_API.value)
                or prim.HasAPI(robot_schema.Classes.SITE_API.value)
                or prim.HasAPI(robot_schema.Classes.ROBOT_API.value)
            ),
            {"target_name": "Link", "target_plural_name": "Links"},
        )
        self._link_picker.show(0, on_targets_selected=self._on_link_targets_selected)

    def _on_link_targets_selected(self, paths: list) -> None:
        """Appends picker-selected link paths to the relationship, skipping duplicates."""
        if not paths:
            return
        links = self._get_relationship_targets(robot_schema.Relations.ROBOT_LINKS)
        existing = set(links)
        for path in paths:
            if path not in existing:
                links.append(path)
        self._set_relationship_targets(robot_schema.Relations.ROBOT_LINKS, links)
        self._rebuild_links_list()

    def _on_remove_joint(self, index: int) -> None:
        """Removes a joint target at the given index from the relationship."""
        joints = self._get_relationship_targets(robot_schema.Relations.ROBOT_JOINTS)
        if 0 <= index < len(joints):
            joints.pop(index)
            self._set_relationship_targets(robot_schema.Relations.ROBOT_JOINTS, joints)
            if self._selected_joint_index is not None:
                if self._selected_joint_index == index:
                    self._selected_joint_index = None
                elif self._selected_joint_index > index:
                    self._selected_joint_index -= 1
            self._rebuild_joints_list()

    def _on_remove_link(self, index: int) -> None:
        """Removes a link target at the given index from the relationship."""
        links = self._get_relationship_targets(robot_schema.Relations.ROBOT_LINKS)
        if 0 <= index < len(links):
            links.pop(index)
            self._set_relationship_targets(robot_schema.Relations.ROBOT_LINKS, links)
            if self._selected_link_index is not None:
                if self._selected_link_index == index:
                    self._selected_link_index = None
                elif self._selected_link_index > index:
                    self._selected_link_index -= 1
            self._rebuild_links_list()

    @staticmethod
    def _track_selection_after_reorder(selected: int | None, source: int, insert_at: int) -> int | None:
        """Compute the new selected index after a reorder operation.

        Args:
            selected: Currently selected index (or None).
            source: The original index of the moved item.
            insert_at: The index where the item was inserted after removal.

        Returns:
            The updated selected index that follows the same item.
        """
        if selected is None:
            return None
        if selected == source:
            return insert_at
        if source < selected <= insert_at:
            return selected - 1
        if insert_at <= selected < source:
            return selected + 1
        return selected

    def _on_reorder_joint(self, target_index: int, drag_data: object) -> None:
        """Reorders a joint in the relationship targets based on drag-and-drop."""
        for ind in self._joint_drop_indicators:
            ind.set_style({"background_color": _DROP_INDICATOR_TRANSPARENT})
        try:
            source_index = int(str(drag_data).split(":")[1])
        except (IndexError, ValueError):
            return
        joints = self._get_relationship_targets(robot_schema.Relations.ROBOT_JOINTS)
        if source_index == target_index or not (0 <= source_index < len(joints)):
            return
        item = joints.pop(source_index)
        insert_at = target_index - 1 if source_index < target_index else target_index
        joints.insert(insert_at, item)
        self._selected_joint_index = self._track_selection_after_reorder(
            self._selected_joint_index, source_index, insert_at
        )
        self._set_relationship_targets(robot_schema.Relations.ROBOT_JOINTS, joints)
        self._rebuild_joints_list()

    def _on_reorder_link(self, target_index: int, drag_data: object) -> None:
        """Reorders a link in the relationship targets based on drag-and-drop."""
        for ind in self._link_drop_indicators:
            ind.set_style({"background_color": _DROP_INDICATOR_TRANSPARENT})
        try:
            source_index = int(str(drag_data).split(":")[1])
        except (IndexError, ValueError):
            return
        links = self._get_relationship_targets(robot_schema.Relations.ROBOT_LINKS)
        if source_index == target_index or not (0 <= source_index < len(links)):
            return
        item = links.pop(source_index)
        insert_at = target_index - 1 if source_index < target_index else target_index
        links.insert(insert_at, item)
        self._selected_link_index = self._track_selection_after_reorder(
            self._selected_link_index, source_index, insert_at
        )
        self._set_relationship_targets(robot_schema.Relations.ROBOT_LINKS, links)
        self._rebuild_links_list()


@_singleton
class LinkAPIWidget(_RobotSchemaWidgetBase):
    """Property widget for prims with the IsaacLinkAPI schema applied.

    Args:
        title: Display title for the widget.
        collapsed: Whether the widget starts collapsed.
    """

    def __init__(self, title: str, collapsed: bool = False) -> None:
        super().__init__(
            title,
            collapsed,
            robot_schema.Classes.LINK_API,
            [
                robot_schema.Attributes.NAME_OVERRIDE,
            ],
            "Link API",
            robot_schema.ApplyLinkAPI,
        )


@_singleton
class JointAPIWidget(_RobotSchemaWidgetBase):
    """Property widget for prims with the IsaacJointAPI schema applied.

    Args:
        title: Display title for the widget.
        collapsed: Whether the widget starts collapsed.
    """

    def __init__(self, title: str, collapsed: bool = False) -> None:
        super().__init__(
            title,
            collapsed,
            robot_schema.Classes.JOINT_API,
            [
                robot_schema.Attributes.JOINT_NAME_OVERRIDE,
                robot_schema.Attributes.DOF_OFFSET_OP_ORDER,
                robot_schema.Attributes.ACTUATOR,
            ],
            "Joint API",
            robot_schema.ApplyJointAPI,
        )


@_singleton
class AttachmentPointAPIWidget(_RobotSchemaWidgetBase):
    """Property widget for prims with the IsaacAttachmentPointAPI schema applied.

    Displays ``Forward Axis`` and ``Clearance Offset`` attributes for the attachment point.
    The ``+`` button menu entry appears only for physics joint prims that do not yet carry the API.

    Args:
        title: Display title for the widget.
        collapsed: Whether the widget starts collapsed.
    """

    def __init__(self, title: str, collapsed: bool = False) -> None:
        super().__init__(
            title,
            collapsed,
            robot_schema.Classes.ATTACHMENT_POINT_API,
            [
                robot_schema.Attributes.FORWARD_AXIS,
                robot_schema.Attributes.CLEARANCE_OFFSET,
            ],
            "Attachment Point API",
            robot_schema.ApplyAttachmentPointAPI,
            exclusive_classes=[robot_schema.Classes.ATTACHMENT_POINT_API],
        )

    def _button_show(self, objects: dict) -> bool:
        """Show the menu entry only for physics joint prims that lack the API."""
        stage = objects.get("stage")
        prim_list = objects.get("prim_list")
        if not stage or not prim_list:
            return False
        for item in prim_list:
            prim = stage.GetPrimAtPath(item) if isinstance(item, Sdf.Path) else item
            if prim and prim.IsA(UsdPhysics.Joint) and not prim.HasAPI(self._schema_class.value):
                return True
        return False

    def _button_onclick(self, payload: PrimSelectionPayload) -> None:
        """Apply ``IsaacAttachmentPointAPI`` to all selected physics joint prims."""
        stage = self._payload.get_stage() if self._payload else omni.usd.get_context().get_stage()
        if not stage:
            return
        for path in payload:
            if not path:
                continue
            prim = stage.GetPrimAtPath(path)
            if not prim or not prim.IsA(UsdPhysics.Joint) or prim.HasAPI(self._schema_class.value):
                continue
            self._apply_fn(prim)
        self._request_refresh()


@_singleton
class SiteAPIWidget(_RobotSchemaWidgetBase):
    """Property widget for applying the IsaacSiteAPI to Xformable prims.

    Args:
        title: Display title for the widget.
        collapsed: Whether the widget starts collapsed.
    """

    def __init__(self, title: str, collapsed: bool = False) -> None:
        super().__init__(
            title,
            collapsed,
            robot_schema.Classes.SITE_API,
            [
                robot_schema.Attributes.REFERENCE_DESCRIPTION,
                robot_schema.Attributes.FORWARD_AXIS,
            ],
            "Site API",
            robot_schema.ApplySiteAPI,
            exclusive_classes=[robot_schema.Classes.SITE_API],
        )

    def _button_show(self, objects: dict) -> bool:
        stage = objects.get("stage")
        prim_list = objects.get("prim_list")
        if not stage or not prim_list:
            return False
        for item in prim_list:
            prim = stage.GetPrimAtPath(item) if isinstance(item, Sdf.Path) else item
            if prim and prim.IsA(UsdGeom.Xformable) and not prim.HasAPI(self._schema_class.value):
                return True
        return False

    def _button_onclick(self, payload: PrimSelectionPayload) -> None:
        stage = self._payload.get_stage() if self._payload else omni.usd.get_context().get_stage()
        if not stage:
            return
        for path in payload:
            if not path:
                continue
            prim = stage.GetPrimAtPath(path)
            if not prim or not prim.IsA(UsdGeom.Xformable) or prim.HasAPI(self._schema_class.value):
                continue
            self._apply_fn(prim)
            instanceable = [p for p in Usd.PrimRange(prim) if p.IsInstanceable()]
            if instanceable:
                prim.SetInstanceable(True)
                prim.ClearMetadata("instanceable")
        self._request_refresh()
