# SPDX-FileCopyrightText: Copyright (c) 2018-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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

"""Property widget for the Isaac Name Override attribute on prims."""

import omni
import omni.ui as ui
from omni.kit.property.usd.prim_selection_payload import PrimSelectionPayload
from omni.kit.property.usd.usd_property_widget import UsdPropertiesWidget
from pxr import Sdf, Usd
from usd.schema.isaac import robot_schema

from .robot_schema import _singleton

_ROBOT_SCHEMA_CLASSES = (
    robot_schema.Classes.ROBOT_API,
    robot_schema.Classes.LINK_API,
    robot_schema.Classes.JOINT_API,
)


def _prim_has_robot_schema(prim: object) -> bool:
    """Checks if a USD prim has any robot schema API applied.

    Args:
        prim: USD prim to check for robot schema APIs.

    Returns:
        True if the prim has any robot schema API (ROBOT_API, LINK_API, or JOINT_API), False otherwise.
    """
    if not prim:
        return False
    return any(prim.HasAPI(schema.value) for schema in _ROBOT_SCHEMA_CLASSES)


@_singleton
class NameOverrideWidget(UsdPropertiesWidget):
    """Property widget for the Isaac Name Override attribute.

    Args:
        title: Display title for the widget.
        collapsed: Whether the widget starts collapsed.
    """

    def __init__(self, title: str, collapsed: bool = False) -> None:
        super().__init__(title, collapsed)
        from omni.kit.property.usd import PrimPathWidget

        self._add_button_menus = []
        self._add_button_menus.append(
            PrimPathWidget.add_button_menu_entry(
                "Isaac/NameOverride", show_fn=self._button_show, onclick_fn=self._button_onclick
            )
        )
        self._old_payload = None

    def destroy(self) -> None:
        """Remove button menu entries and clean up resources."""
        from omni.kit.property.usd import PrimPathWidget

        for menu in self._add_button_menus:
            PrimPathWidget.remove_button_menu_entry(menu)
        self._add_button_menus = []

    def _button_show(self, objects: dict) -> bool:
        """Determines whether to show the Name Override button in the prim path widget.

        Args:
            objects: Dictionary containing stage and prim_list information.

        Returns:
            True if any prim without robot schema lacks the name override attribute.
        """
        if "prim_list" not in objects or "stage" not in objects:
            return False
        stage = objects["stage"]
        if not stage:
            return False
        prim_list = objects["prim_list"]
        if len(prim_list) < 1:
            return False
        for item in prim_list:
            if isinstance(item, Sdf.Path):
                prim = stage.GetPrimAtPath(item)
            elif isinstance(item, Usd.Prim):
                prim = item
            else:
                prim = None
            if not prim or _prim_has_robot_schema(prim):
                continue
            if not prim.HasAttribute(robot_schema.Attributes.NAME_OVERRIDE.name):
                return True
        return False

    def _button_onclick(self, payload: PrimSelectionPayload) -> None:
        """Handles button click to add name override attributes to selected prims.

        Args:
            payload: The prim selection payload containing paths to process.
        """
        stage = self._payload.get_stage()
        for path in payload:
            if path:
                prim = stage.GetPrimAtPath(path)
                if _prim_has_robot_schema(prim):
                    continue
                if not prim.HasAttribute(robot_schema.Attributes.NAME_OVERRIDE.name):
                    prim.CreateAttribute(
                        robot_schema.Attributes.NAME_OVERRIDE.name, Sdf.ValueTypeNames.String, True
                    ).Set(prim.GetName())
        self._request_refresh()

    def _request_refresh(self) -> None:
        """Refresh the entire property window."""
        selection = omni.usd.get_context().get_selection()
        selected_paths = selection.get_selected_prim_paths()
        window = omni.kit.window.property.get_window()._window  # noqa: PLW0212

        selection.clear_selected_prim_paths()
        window.frame.rebuild()
        selection.set_selected_prim_paths(selected_paths, True)
        window.frame.rebuild()

    def _on_usd_changed(self, notice: object, stage: object) -> None:
        """Handles USD stage change notifications by refreshing the widget if needed.

        Args:
            notice: The USD change notice containing modification information.
            stage: The USD stage that changed.
        """
        targets = notice.GetChangedInfoOnlyPaths()
        if self._old_payload != self.on_new_payload(
            self._payload
        ):  # if selection didn't change, check if attribute still exists, and force rebuild if so
            self._request_refresh()
        else:
            super()._on_usd_changed(notice, stage)

    def _get_prim(self, prim_path: object) -> Usd.Prim | None:
        """Retrieves a prim if it exists, lacks robot schema, and has a name override attribute.

        Args:
            prim_path: Path to the prim to retrieve.

        Returns:
            The prim if it meets all criteria, None otherwise.
        """
        if prim_path:
            stage = self._payload.get_stage()
            if stage:
                prim = stage.GetPrimAtPath(prim_path)
                if (
                    prim
                    and not _prim_has_robot_schema(prim)
                    and prim.HasAttribute(robot_schema.Attributes.NAME_OVERRIDE.name)
                ):
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

        if len(self._payload) != 1:  # avoids allowing changing multiple prims at the same time.
            return False
        prim_path = self._payload.get_paths()[0]
        self._prim = self._get_prim(prim_path)
        self._old_payload = self._prim
        if not self._prim:
            return False

        return self._prim

    def on_remove_attr(self) -> None:
        """Remove the Name Override attribute from the selected prim."""
        stage = self._payload.get_stage()
        if stage:
            prim = self._get_prim(self._payload.get_paths()[0])
            if prim and prim.HasAttribute(robot_schema.Attributes.NAME_OVERRIDE.name):
                prim.RemoveProperty(robot_schema.Attributes.NAME_OVERRIDE.name)

    def _filter_props_to_build(self, props: list) -> list[Usd.Attribute]:
        """Filters properties to only include name override attributes and sets their display properties.

        Args:
            props: List of properties to filter.

        Returns:
            Filtered list containing only name override attributes with updated display settings.
        """
        props = [
            prop
            for prop in props
            if isinstance(prop, Usd.Attribute) and (prop.GetName() == robot_schema.Attributes.NAME_OVERRIDE.name)
        ]
        if props:
            props[0].SetDisplayName("Name Override")
            props[0].SetDocumentation("Name override for prim lookup in base name search")
        return props

    def build_items(self) -> None:
        """Build property items only when the frame is expanded and a prim is selected."""
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
                    identifier="remove_name_override",
                    tooltip="Remove Name Override",
                )
