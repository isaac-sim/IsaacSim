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

"""Editor tools panel: undo/redo, visibility, colors, clear-all, scale-all, and import/export."""

from __future__ import annotations

from collections.abc import Callable
from typing import TYPE_CHECKING

import carb
import numpy as np
import omni.kit.commands
import omni.ui as ui
from isaacsim.gui.components.element_wrappers import Button, CheckBox, CollapsableFrame, DropDown
from isaacsim.gui.components.ui_utils import (
    btn_builder,
    color_picker_builder,
    float_builder,
    get_style,
    state_btn_builder,
    str_builder,
)

from .. import lula_io, xrdf_io
from ..constants import XRDF_VERSION_1, XRDF_VERSION_2

if TYPE_CHECKING:
    from ..editor_state import EditorState


class EditorToolsPanel:
    """Editor Tools, Export To File, and Import From File frames.

    Args:
        state: Shared editor state.
        get_selected_link_name: Callback returning the selected link subpath.
        get_selected_link_path: Callback returning the selected link path.
        refresh_sphere_comboboxes: Callback refreshing sphere dropdowns.
        rebuild_joint_properties: Callback rebuilding joint property widgets.
    """

    def __init__(
        self,
        state: "EditorState",
        get_selected_link_name: Callable[[], str | None],
        get_selected_link_path: Callable[[], str | None],
        refresh_sphere_comboboxes: Callable[..., None],
        rebuild_joint_properties: Callable[[], None],
    ) -> None:
        self._state = state
        self._get_selected_link_name = get_selected_link_name
        self._get_selected_link_path = get_selected_link_path
        self._refresh_sphere_comboboxes = refresh_sphere_comboboxes
        self._rebuild_joint_properties = rebuild_joint_properties

        # Frames
        self._tools_frame: ui.CollapsableFrame | None = None
        self._export_frame: ui.CollapsableFrame | None = None
        self._import_frame: CollapsableFrame | None = None

        # Buttons that need to be re-enabled by the orchestrator.
        self._hide_link_btn = None
        self._hide_robot_btn = None
        self._undo_btn = None
        self._redo_btn = None
        self._link_color_picker = None
        self._base_color_picker = None
        self._scale_all_factor = None

        self._robot_description_output_file = None
        self._robot_description_export_btn = None
        self._xrdf_output_file = None
        self._xrdf_export_btn = None
        self._xrdf_version_dropdown: DropDown | None = None
        self._xrdf_merge_cb: CheckBox | None = None

        self._lula_input_file = None
        self._robot_description_import_btn = None
        self._xrdf_input_file = None
        self._xrdf_import_btn = None

        self._hiding_link = False
        self._hiding_robot = False
        self._prev_link: str | None = None

    # ------------------------------------------------------------------
    # Build
    # ------------------------------------------------------------------
    def build(self) -> None:
        """Build the Editor Tools + Export + Import frames."""
        self._tools_frame = ui.CollapsableFrame(
            title="Editor Tools",
            height=0,
            collapsed=True,
            style=get_style(),
            name="editorFrame",
            horizontal_scrollbar_policy=ui.ScrollBarPolicy.SCROLLBAR_AS_NEEDED,
            vertical_scrollbar_policy=ui.ScrollBarPolicy.SCROLLBAR_ALWAYS_ON,
        )
        self._tools_frame.enabled = False

        with self._tools_frame:
            with ui.VStack(style=get_style(), spacing=5, height=0):
                self._undo_btn = btn_builder("Undo", text="Undo", on_clicked_fn=self._on_undo)
                self._undo_btn.enabled = True

                self._redo_btn = btn_builder("Redo", text="Redo", on_clicked_fn=self._on_redo)
                self._redo_btn.enabled = True

                self._hide_link_btn = state_btn_builder(
                    label="Toggle Link Visibility",
                    a_text=" Hide",
                    b_text="Show",
                    tooltip="Hide the Selected Link",
                    on_clicked_fn=self._on_toggle_link_visible,
                )
                self._hide_robot_btn = state_btn_builder(
                    label="Toggle Robot Visibility",
                    a_text="Hide",
                    b_text="Show",
                    tooltip="Hide the Robot",
                    on_clicked_fn=self._on_toggle_robot_visible,
                )

                self._link_color_picker = color_picker_builder(
                    label="Link Sphere Color",
                    default_val=self._state.collision_sphere_editor.filter_in_sphere_color,
                    tooltip="Set the color of all collision spheres in the selected link",
                )
                self._link_color_picker.add_end_edit_fn(self._on_link_color_change)

                self._base_color_picker = color_picker_builder(
                    label="Base Sphere Color",
                    default_val=self._state.collision_sphere_editor.filter_out_sphere_color,
                    tooltip="Set the color of all collision spheres outside the selected link",
                )
                self._base_color_picker.add_end_edit_fn(self._on_base_color_change)

                btn = btn_builder("Clear All Spheres", text="Clear", on_clicked_fn=self._on_clear_all)
                btn.enabled = True

                scale_frame = ui.CollapsableFrame(
                    title="Scale All Spheres",
                    name="subFrame",
                    height=0,
                    collapsed=True,
                    style=get_style(),
                    style_type_name_override="CollapsableFrame",
                    horizontal_scrollbar_policy=ui.ScrollBarPolicy.SCROLLBAR_AS_NEEDED,
                    vertical_scrollbar_policy=ui.ScrollBarPolicy.SCROLLBAR_ALWAYS_ON,
                )
                with scale_frame:
                    with ui.VStack(style=get_style(), spacing=5, height=0):
                        self._scale_all_factor = float_builder(
                            label="Scaling Factor",
                            default_val=1.0,
                            min=0.001,
                            tooltip="Scaling factor for the radii of the specified spheres",
                        )
                        scale_btn = btn_builder(
                            "Scale All Spheres",
                            text="Scale All Spheres",
                            on_clicked_fn=self._on_scale_all_spheres,
                        )
                        scale_btn.enabled = True

        self._build_export_frame()
        self._build_import_frame()

    def _build_export_frame(self) -> None:
        self._export_frame = ui.CollapsableFrame(
            title="Export To File",
            name="subFrame",
            height=0,
            collapsed=True,
            style=get_style(),
            style_type_name_override="CollapsableFrame",
            horizontal_scrollbar_policy=ui.ScrollBarPolicy.SCROLLBAR_AS_NEEDED,
            vertical_scrollbar_policy=ui.ScrollBarPolicy.SCROLLBAR_ALWAYS_ON,
        )
        self._export_frame.enabled = False

        with self._export_frame:
            with ui.VStack(style=get_style(), spacing=5, height=0):
                # Lula export
                lula_frame = CollapsableFrame("Export to Lula Robot Description File", collapsed=True)
                with lula_frame:
                    with ui.VStack(style=get_style(), spacing=5, height=0):
                        self._robot_description_output_file = str_builder(
                            label="Output File",
                            default_val="",
                            tooltip="Click the Folder Icon to Set Filepath",
                            use_folder_picker=True,
                            item_filter_fn=lula_io.on_filter_item,
                            folder_dialog_title="Write all sphere to a YAML file",
                            folder_button_title="Select YAML",
                        )
                        self._robot_description_output_file.add_value_changed_fn(
                            self._check_robot_description_file_type
                        )

                        self._robot_description_export_btn = btn_builder(
                            "Save", text="Save", on_clicked_fn=self._on_save_robot_description
                        )
                        self._robot_description_export_btn.enabled = False

                # XRDF export
                xrdf_frame = CollapsableFrame("Export to cuMotion XRDF", collapsed=True)
                with xrdf_frame:
                    with ui.VStack(style=get_style(), spacing=5, height=0):
                        self._xrdf_output_file = str_builder(
                            label="Output File",
                            default_val="",
                            tooltip="Click the Folder Icon to Set Filepath",
                            use_folder_picker=True,
                            item_filter_fn=xrdf_io.on_filter_xrdf_item,
                            folder_dialog_title="Write all sphere to an XRDF file",
                            folder_button_title="Select XRDF",
                        )
                        self._xrdf_output_file.add_value_changed_fn(self._on_select_xrdf_output_file)

                        self._xrdf_version_dropdown = DropDown(
                            "XRDF Version",
                            tooltip=(
                                "Select the XRDF format version to export. Version 1.0 uses 'collision', "
                                "version 2.0 uses 'world_collision'."
                            ),
                            populate_fn=lambda: ["1.0", "2.0"],
                        )
                        self._xrdf_version_dropdown.repopulate()
                        self._xrdf_version_dropdown.set_selection_by_index(1)  # Default to 2.0

                        self._xrdf_export_btn = Button("Export XRDF", "Export", on_click_fn=self._on_export_xrdf)
                        self._xrdf_export_btn.enabled = False

                        cb_tooltip = (
                            "Merge with the XRDF that already exists at the specified path. "
                            "Merging will maintain any data written into the XRDF file that is "
                            "not represented in the Robot Description Editor. Specifically, "
                            "self_collision ignore rules and buffer distances, modifiers, "
                            "tool_frames, and spheres for unrecognized robot frames."
                        )
                        self._xrdf_merge_cb = CheckBox("Merge With Existing XRDF", tooltip=cb_tooltip)
                        self._xrdf_merge_cb.visible = False
                        self._xrdf_merge_cb.set_value(False)

    def _build_import_frame(self) -> None:
        self._import_frame = CollapsableFrame("Import From File", collapsed=True, enabled=False)

        with self._import_frame:
            with ui.VStack(style=get_style(), spacing=5, height=0):
                # Lula import
                lula_frame = ui.CollapsableFrame(
                    title="Import Lula Robot Description File",
                    name="subFrame",
                    height=0,
                    collapsed=True,
                    style=get_style(),
                    style_type_name_override="CollapsableFrame",
                    horizontal_scrollbar_policy=ui.ScrollBarPolicy.SCROLLBAR_AS_NEEDED,
                    vertical_scrollbar_policy=ui.ScrollBarPolicy.SCROLLBAR_ALWAYS_ON,
                )
                with lula_frame:
                    with ui.VStack(style=get_style(), spacing=5, height=0):
                        self._lula_input_file = str_builder(
                            label="Input File",
                            default_val="",
                            tooltip="Click the Folder Icon to Set Filepath",
                            use_folder_picker=True,
                            item_filter_fn=lula_io.on_filter_item,
                            folder_dialog_title="Select Robot Description YAML file, clearing all spheres",
                            folder_button_title="Select YAML",
                        )
                        self._lula_input_file.add_value_changed_fn(self._check_lula_input_file)

                        self._robot_description_import_btn = btn_builder(
                            "Import", text="Import", on_clicked_fn=self._on_import_lula
                        )
                        self._robot_description_import_btn.enabled = False

                # XRDF import
                xrdf_frame = CollapsableFrame("Import cuMotion XRDF", collapsed=True)
                with xrdf_frame:
                    with ui.VStack(style=get_style(), spacing=5, height=0):
                        self._xrdf_input_file = str_builder(
                            label="Input File",
                            default_val="",
                            tooltip="Click the Folder Icon to Set Filepath",
                            use_folder_picker=True,
                            item_filter_fn=xrdf_io.on_filter_xrdf_item,
                            folder_dialog_title="Select cuMotion XRDF file, clearing all spheres",
                            folder_button_title="Select YAML",
                        )
                        self._xrdf_input_file.add_value_changed_fn(self._check_xrdf_input_file)

                        self._xrdf_import_btn = Button("Import XRDF", "Import", on_click_fn=self._on_import_xrdf)
                        self._xrdf_import_btn.enabled = False

    # ------------------------------------------------------------------
    # Public API
    # ------------------------------------------------------------------
    def show(self) -> None:
        """Show and enable the tools, export, and import frames."""
        if self._tools_frame is not None:
            self._tools_frame.collapsed = False
            self._tools_frame.enabled = True
        if self._export_frame is not None:
            self._export_frame.enabled = True
        if self._import_frame is not None:
            self._import_frame.enabled = True

    def hide(self) -> None:
        """Hide and disable the tools, export, and import frames."""
        if self._tools_frame is not None:
            self._tools_frame.collapsed = True
            self._tools_frame.enabled = False
        if self._export_frame is not None:
            self._export_frame.collapsed = True
            self._export_frame.enabled = False
        if self._import_frame is not None:
            self._import_frame.collapsed = True
            self._import_frame.enabled = False

    def show_robot_if_hidden(self) -> None:
        """Restore visibility of any link/robot that was hidden via the toggle buttons."""
        if self._hiding_robot and self._hide_robot_btn is not None:
            self._hide_robot_btn.call_clicked_fn()
        if self._hiding_link and self._hide_link_btn is not None:
            self._hide_link_btn.call_clicked_fn()

    def on_link_selected(self, link_name: str) -> None:
        """Apply pending hide/show state to a freshly selected link.

        Args:
            link_name: Newly selected link subpath.
        """
        if self._hiding_link != self._hiding_robot:
            self._hide_link(link_name)
            if self._prev_link is not None:
                self._hide_link(self._prev_link)
        self._prev_link = link_name

    def update_import_button_states(self) -> None:
        """Refresh enabled state of import buttons based on the currently-entered paths."""
        if (
            self._lula_input_file is not None
            and self._robot_description_import_btn is not None
            and lula_io.is_yaml_file(self._lula_input_file.get_value_as_string())
        ):
            self._robot_description_import_btn.enabled = True

        if (
            self._xrdf_input_file is not None
            and self._xrdf_import_btn is not None
            and xrdf_io.is_xrdf_file(self._xrdf_input_file.get_value_as_string())
        ):
            self._xrdf_import_btn.enabled = True

    # ------------------------------------------------------------------
    # Field validators
    # ------------------------------------------------------------------
    def _check_robot_description_file_type(self, model: object = None) -> None:
        if self._robot_description_export_btn is None:
            return
        path = model.get_value_as_string()
        if lula_io.is_yaml_file(path) and "omniverse:" not in path.lower():
            self._robot_description_export_btn.enabled = True
        else:
            self._robot_description_export_btn.enabled = False
            carb.log_warn(f"Invalid path to Robot Description YAML: {path}")

    def _on_select_xrdf_output_file(self, model: object = None) -> None:
        if self._xrdf_export_btn is None or self._xrdf_merge_cb is None:
            return
        path = model.get_value_as_string()
        if xrdf_io.is_xrdf_file(path) and "omniverse:" not in path.lower():
            self._xrdf_export_btn.enabled = True
            if xrdf_io.is_valid_xrdf_file(path):
                self._xrdf_merge_cb.visible = True
                self._xrdf_merge_cb.set_value(True)
            else:
                self._xrdf_merge_cb.visible = False
                self._xrdf_merge_cb.set_value(False)
        else:
            self._xrdf_export_btn.enabled = False
            self._xrdf_merge_cb.visible = False
            self._xrdf_merge_cb.set_value(False)
            carb.log_warn(f"Invalid path to XRDF: {path}")

    def _check_lula_input_file(self, model: object = None) -> None:
        if self._robot_description_import_btn is None:
            return
        path = model.get_value_as_string()
        if lula_io.is_yaml_file(path) and self._state.articulation is not None:
            self._robot_description_import_btn.enabled = True
        elif self._state.articulation is None:
            self._robot_description_import_btn.enabled = False
            carb.log_warn(
                "Robot Articulation must be selected in the Selection Panel in order to import spheres for a robot"
            )
        else:
            self._robot_description_import_btn.enabled = False
            carb.log_warn(f"Invalid path to Robot Description YAML: {path}")

    def _check_xrdf_input_file(self, model: object = None) -> None:
        if self._xrdf_import_btn is None:
            return
        path = model.get_value_as_string()
        if xrdf_io.is_xrdf_file(path) and self._state.articulation is not None:
            self._xrdf_import_btn.enabled = True
        elif self._state.articulation is None:
            self._xrdf_import_btn.enabled = False
            carb.log_warn(
                "Robot Articulation must be selected in the Selection Panel in order to import spheres for a robot"
            )
        else:
            self._xrdf_import_btn.enabled = False
            carb.log_warn(f"Invalid path to XRDF: {path}")

    # ------------------------------------------------------------------
    # Action callbacks
    # ------------------------------------------------------------------
    def _on_undo(self) -> None:
        self._state.collision_sphere_editor.undo()
        self._refresh_sphere_comboboxes()

    def _on_redo(self) -> None:
        self._state.collision_sphere_editor.redo()
        self._refresh_sphere_comboboxes()

    def _on_clear_all(self) -> None:
        self._state.collision_sphere_editor.clear_spheres()
        self._refresh_sphere_comboboxes()

    def _on_scale_all_spheres(self) -> None:
        if self._scale_all_factor is None:
            return
        if self._state.articulation_base_path is None:
            return
        self._state.collision_sphere_editor.scale_spheres(
            self._state.articulation_base_path, self._scale_all_factor.get_value_as_float()
        )

    def _on_link_color_change(self, a1: object, a2: object) -> None:
        if self._link_color_picker is None:
            return
        link_path = self._get_selected_link_path()
        if link_path is None:
            return
        color = self._extract_color(self._link_color_picker)
        self._state.collision_sphere_editor.set_sphere_colors(link_path, color_in=color)

    def _on_base_color_change(self, a1: object, a2: object) -> None:
        if self._base_color_picker is None:
            return
        link_path = self._get_selected_link_path()
        if link_path is None:
            return
        color = self._extract_color(self._base_color_picker)
        self._state.collision_sphere_editor.set_sphere_colors(link_path, color_out=color)

    @staticmethod
    def _extract_color(picker: object) -> np.ndarray:
        vals: list[float] = []
        for item in picker.get_item_children():
            vals.append(picker.get_item_value_model(item).get_value_as_float())
        return np.array(vals[:3])

    # ------------------------------------------------------------------
    # Visibility toggles
    # ------------------------------------------------------------------
    def _on_toggle_link_visible(self, model: object = None) -> None:
        link_name = self._get_selected_link_name()
        if link_name is None:
            return
        self._hide_link(link_name)
        self._hiding_link = not self._hiding_link

    def _on_toggle_robot_visible(self, model: object = None) -> None:
        selected_link = self._get_selected_link_name()
        links = list(self._state.link_to_meshes.keys())
        for link in links:
            if selected_link != link:
                self._hide_link(link)
        self._hiding_robot = not self._hiding_robot

    def _hide_link(self, link_name: str) -> None:
        if self._state.articulation_base_path is None:
            return
        meshes = self._state.link_to_meshes.get(link_name, [])
        link_path = self._state.articulation_base_path + link_name
        mesh_paths = [link_path + mesh for mesh in meshes]
        omni.kit.commands.execute("ToggleVisibilitySelectedPrims", selected_paths=mesh_paths)

    # ------------------------------------------------------------------
    # Save / load callbacks
    # ------------------------------------------------------------------
    def _on_save_robot_description(self, model: object = None) -> None:
        if self._robot_description_output_file is None:
            return
        if self._state.articulation is None:
            return
        path = self._robot_description_output_file.get_value_as_string()
        if not path:
            carb.log_error(f"Cannot Save to Invalid Path {path}")
            return
        try:
            self._state.export_lula(path)
        except ValueError as exc:
            carb.log_error(str(exc))

    def _on_export_xrdf(self, model: object = None) -> None:
        if self._xrdf_output_file is None or self._xrdf_version_dropdown is None or self._xrdf_merge_cb is None:
            return
        if self._state.articulation is None:
            return
        path = self._xrdf_output_file.get_value_as_string()
        selection_index = self._xrdf_version_dropdown.get_selection_index()
        format_version = XRDF_VERSION_1 if selection_index == 0 else XRDF_VERSION_2
        merge = self._xrdf_merge_cb.get_value()
        carb.log_info(f"Exporting XRDF with version: {format_version}")
        self._state.export_xrdf(path, format_version=format_version, merge_with_existing=merge)

    def _on_import_lula(self, model: object = None) -> None:
        if self._lula_input_file is None:
            return
        if self._state.articulation is None:
            return
        path = self._lula_input_file.get_value_as_string()
        self._state.import_lula(path)
        self._rebuild_joint_properties()

    def _on_import_xrdf(self, model: object = None) -> None:
        if self._xrdf_input_file is None:
            return
        if self._state.articulation is None:
            return
        path = self._xrdf_input_file.get_value_as_string()
        try:
            self._state.import_xrdf(path)
        except ValueError as exc:
            carb.log_error(str(exc))
            return
        self._rebuild_joint_properties()
