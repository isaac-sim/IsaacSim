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

"""Link sphere editor panel: generate, add, connect, scale, and clear spheres."""

from __future__ import annotations

from typing import TYPE_CHECKING

import carb
import numpy as np
import omni.ui as ui
from isaacsim.gui.components.ui_utils import (
    add_line_rect_flourish,
    btn_builder,
    float_builder,
    get_style,
    int_builder,
    state_btn_builder,
    xyz_builder,
)
from isaacsim.gui.components.widgets import DynamicComboBoxModel
from omni.kit.window.property.templates import LABEL_WIDTH

from .. import sphere_generation

if TYPE_CHECKING:
    from ..editor_state import EditorState


class SphereEditorPanel:
    """UI for link-scoped collision sphere operations.

    Args:
        state: Shared editor state.
    """

    def __init__(self, state: "EditorState") -> None:
        self._state = state

        # Frame + sub-frames
        self._frame: ui.CollapsableFrame | None = None

        # Mesh selection
        self._mesh_model: DynamicComboBoxModel | None = None
        self._mesh_combobox: ui.ComboBox | None = None

        # Generate spheres widgets
        self._num_spheres_field = None
        self._radius_offset_field = None
        self._preview_btn = None
        self._generate_btn = None
        self._preview_active = True

        # Add sphere widgets
        self._add_sphere_radius = None
        self._add_sphere_translation_x = None
        self._add_sphere_translation_y = None
        self._add_sphere_translation_z = None

        # Connect spheres widgets
        self._connect_sphere_0_model: DynamicComboBoxModel | None = None
        self._connect_sphere_0_combobox: ui.ComboBox | None = None
        self._connect_sphere_1_model: DynamicComboBoxModel | None = None
        self._connect_sphere_1_combobox: ui.ComboBox | None = None
        self._connect_sphere_num = None
        self._connect_sphere_0_options: list[str] = []
        self._connect_sphere_1_options: list[str] = []

        # Scale + clear
        self._scale_factor_field = None

        # Selected link from the SelectionPanel
        self._selected_link: str | None = None

    # ------------------------------------------------------------------
    # Build
    # ------------------------------------------------------------------
    def build(self) -> None:
        """Build all sub-frames of the Link Sphere Editor panel."""
        self._frame = ui.CollapsableFrame(
            title="Link Sphere Editor",
            height=0,
            collapsed=True,
            style=get_style(),
            name="editorFrame",
            horizontal_scrollbar_policy=ui.ScrollBarPolicy.SCROLLBAR_AS_NEEDED,
            vertical_scrollbar_policy=ui.ScrollBarPolicy.SCROLLBAR_ALWAYS_ON,
        )
        self._frame.enabled = False

        with self._frame:
            with ui.VStack(style=get_style(), spacing=5, height=0):
                self._build_generate_section()
                self._build_add_section()
                self._build_connect_section()
                self._build_scale_section()
                self._build_clear_section()

    def _build_generate_section(self) -> None:
        frame = ui.CollapsableFrame(
            title="Generate Spheres",
            name="subFrame",
            height=0,
            collapsed=True,
            style_type_name_override="CollapsableFrame",
            horizontal_scrollbar_policy=ui.ScrollBarPolicy.SCROLLBAR_AS_NEEDED,
            vertical_scrollbar_policy=ui.ScrollBarPolicy.SCROLLBAR_ALWAYS_ON,
        )
        with frame:
            with ui.VStack(style=get_style(), spacing=5, height=0):
                self._mesh_model = DynamicComboBoxModel([])
                with ui.HStack():
                    ui.Label(
                        "Select Mesh",
                        width=LABEL_WIDTH,
                        alignment=ui.Alignment.LEFT_CENTER,
                        tooltip="Select Mesh to be Used for Sphere Generation",
                    )
                    self._mesh_combobox = ui.ComboBox(self._mesh_model)
                    add_line_rect_flourish(False)

                self._num_spheres_field = int_builder(
                    label="Number of Spheres",
                    default_val=0,
                    min=0,
                    tooltip="Number of Spheres to Generate for Link",
                )
                self._num_spheres_field.add_value_changed_fn(
                    lambda m: self._trigger_preview_generate_spheres_for_link()
                )

                self._radius_offset_field = float_builder(
                    label="Radius Offset",
                    default_val=0.01,
                    tooltip=(
                        "Extent to which spheres may extend beyond the mesh.  "
                        "A positive value means that spheres may exceed the mesh by up to the given value.\n"
                        "A negative value specifies that all spheres are at least radius_offset from the mesh surface."
                    ),
                )
                self._radius_offset_field.add_value_changed_fn(
                    lambda m: self._trigger_preview_generate_spheres_for_link()
                )

                self._preview_btn = state_btn_builder(
                    label="Preview Spheres",
                    b_text="Show Preview",
                    a_text="Hide Preview",
                    tooltip="Show a preview of the spheres that will be generated.",
                    on_clicked_fn=self._on_toggle_preview,
                )

                self._generate_btn = btn_builder(
                    label="Generate Spheres",
                    text="Generate Spheres",
                    tooltip="Generate Spheres for Robot Link",
                    on_clicked_fn=self._on_generate_spheres,
                )

    def _build_add_section(self) -> None:
        frame = ui.CollapsableFrame(
            title="Add Sphere",
            name="subFrame",
            height=0,
            collapsed=True,
            style=get_style(),
            style_type_name_override="CollapsableFrame",
            horizontal_scrollbar_policy=ui.ScrollBarPolicy.SCROLLBAR_AS_NEEDED,
            vertical_scrollbar_policy=ui.ScrollBarPolicy.SCROLLBAR_ALWAYS_ON,
        )
        with frame:
            with ui.VStack(style=get_style(), spacing=5, height=0):
                self._add_sphere_radius = float_builder(
                    label="Radius", default_val=0.1, min=0.001, tooltip="Desired Radius"
                )
                val_models = xyz_builder(
                    label="Relative Translation",
                    tooltip="Relative translation of sphere in the local frame of the selected Prim path.",
                    axis_count=3,
                    default_val=[0.0, 0.0, 0.0],
                )
                self._add_sphere_translation_x = val_models[0]
                self._add_sphere_translation_y = val_models[1]
                self._add_sphere_translation_z = val_models[2]

                btn_builder("Add Sphere", text="Add Sphere", on_clicked_fn=self._on_add_sphere)

    def _build_connect_section(self) -> None:
        frame = ui.CollapsableFrame(
            title="Connect Spheres",
            name="subFrame",
            height=0,
            collapsed=True,
            style=get_style(),
            style_type_name_override="CollapsableFrame",
            horizontal_scrollbar_policy=ui.ScrollBarPolicy.SCROLLBAR_AS_NEEDED,
            vertical_scrollbar_policy=ui.ScrollBarPolicy.SCROLLBAR_ALWAYS_ON,
        )
        with frame:
            with ui.VStack(style=get_style(), spacing=5, height=0):
                self._connect_sphere_0_model = DynamicComboBoxModel([])
                with ui.HStack():
                    ui.Label(
                        "Select Collision Sphere",
                        width=LABEL_WIDTH,
                        alignment=ui.Alignment.LEFT_CENTER,
                        tooltip="Select First Collision Sphere to Connect",
                    )
                    self._connect_sphere_0_combobox = ui.ComboBox(self._connect_sphere_0_model)
                    add_line_rect_flourish(False)
                self._connect_sphere_0_combobox.model.add_item_changed_fn(self._on_collision_sphere_select_0)

                self._connect_sphere_1_model = DynamicComboBoxModel([])
                with ui.HStack():
                    ui.Label(
                        "Select Collision Sphere",
                        width=LABEL_WIDTH,
                        alignment=ui.Alignment.LEFT_CENTER,
                        tooltip="Select First Collision Sphere to Connect",
                    )
                    self._connect_sphere_1_combobox = ui.ComboBox(self._connect_sphere_1_model)
                    add_line_rect_flourish(False)

                self._connect_sphere_num = int_builder(
                    label="Number of Spheres",
                    default_val=0,
                    tooltip="Create the specified number of spheres interpolated between the selected spheres",
                )

                btn = btn_builder("Connect Spheres", text="Connect Spheres", on_clicked_fn=self._on_connect_spheres)
                btn.enabled = True

    def _build_scale_section(self) -> None:
        frame = ui.CollapsableFrame(
            title="Scale Spheres in Link",
            name="subFrame",
            height=0,
            collapsed=True,
            style=get_style(),
            style_type_name_override="CollapsableFrame",
            horizontal_scrollbar_policy=ui.ScrollBarPolicy.SCROLLBAR_AS_NEEDED,
            vertical_scrollbar_policy=ui.ScrollBarPolicy.SCROLLBAR_ALWAYS_ON,
        )
        with frame:
            with ui.VStack(style=get_style(), spacing=5, height=0):
                self._scale_factor_field = float_builder(
                    label="Scaling Factor",
                    default_val=1.0,
                    min=0.001,
                    tooltip="Scaling factor for the radii of the specified spheres",
                )
                btn = btn_builder("Scale Spheres", text="Scale Spheres", on_clicked_fn=self._on_scale_spheres)
                btn.enabled = True

    def _build_clear_section(self) -> None:
        frame = ui.CollapsableFrame(
            title="Clear Spheres in Link",
            name="subFrame",
            height=0,
            collapsed=True,
            style=get_style(),
            style_type_name_override="CollapsableFrame",
            horizontal_scrollbar_policy=ui.ScrollBarPolicy.SCROLLBAR_AS_NEEDED,
            vertical_scrollbar_policy=ui.ScrollBarPolicy.SCROLLBAR_ALWAYS_ON,
        )
        with frame:
            with ui.VStack(style=get_style(), spacing=5, height=0):
                btn = btn_builder("Clear Link Spheres", text="Clear", on_clicked_fn=self._on_clear_link_spheres)
                btn.enabled = True

    # ------------------------------------------------------------------
    # Public API
    # ------------------------------------------------------------------
    def show(self) -> None:
        """Show and enable the sphere editor frame."""
        if self._frame is None:
            return
        self._frame.collapsed = False
        self._frame.visible = True
        self._frame.enabled = True

    def hide(self) -> None:
        """Hide and disable the sphere editor frame."""
        if self._frame is None:
            return
        self._frame.collapsed = True
        self._frame.enabled = False

    def on_link_selected(self, link_name: str) -> None:
        """Called when the SelectionPanel reports a new link selection.

        Re-populates the mesh dropdown, refreshes connect-sphere dropdowns, applies
        sphere highlighting, and re-runs the preview.

        Args:
            link_name: Newly selected link subpath.
        """
        self._selected_link = link_name
        meshes = self._state.link_to_meshes.get(link_name, [])
        if self._mesh_combobox is not None:
            self._mesh_model = DynamicComboBoxModel(meshes)
            self._mesh_combobox.model = self._mesh_model
            self._mesh_combobox.model.add_item_changed_fn(
                lambda m, v: self._trigger_preview_generate_spheres_for_link()
            )

        self._generate_spheres_for_link()
        self.refresh_collision_sphere_comboboxes()
        link_path = self.get_selected_link_path()
        if link_path is not None:
            self._state.collision_sphere_editor.set_sphere_colors(link_path)

    def refresh_collision_sphere_comboboxes(self, keep_sphere_selection: bool = False) -> None:
        """Re-populate the two 'Select Collision Sphere' dropdowns for the current link.

        Args:
            keep_sphere_selection: Whether to preserve the current sphere selection.
        """
        if self._connect_sphere_0_combobox is None or self._connect_sphere_0_model is None:
            return
        link_path = self.get_selected_link_path()
        if link_path is None:
            return

        sphere_0_name, _ = self.get_selected_collision_spheres()

        sphere_names = self._state.collision_sphere_editor.get_sphere_names_by_link(link_path)
        self._connect_sphere_0_options = sphere_names
        self._connect_sphere_0_model = DynamicComboBoxModel(sphere_names)
        self._connect_sphere_0_combobox.model = self._connect_sphere_0_model
        self._connect_sphere_0_model.add_item_changed_fn(self._on_collision_sphere_select_0)

        if keep_sphere_selection and sphere_0_name in sphere_names:
            self._connect_sphere_0_model.get_item_value_model().set_value(int(sphere_names.index(sphere_0_name)))

        self._on_collision_sphere_select_0(None, None)

    def get_selected_link_path(self) -> str | None:
        """Return the full USD path to the selected link.

        Returns:
            Full selected link path, or None if no link is selected.
        """
        if self._selected_link is None or self._state.articulation_base_path is None:
            return None
        return self._state.articulation_base_path + self._selected_link

    def get_selected_link_name(self) -> str | None:
        """Return the selected link subpath.

        Returns:
            Selected link subpath, or None if no link is selected.
        """
        return self._selected_link

    def get_selected_collision_spheres(self) -> tuple[str | None, str | None]:
        """Return the sphere names selected in the connect-spheres comboboxes.

        Returns:
            First and second selected sphere names. Either value may be None.
        """
        if not self._connect_sphere_0_options:
            return None, None
        if self._connect_sphere_0_model is None:
            return None, None
        c0 = self._connect_sphere_0_options[self._connect_sphere_0_model.get_item_value_model().as_int]

        if not self._connect_sphere_1_options or self._connect_sphere_1_model is None:
            return c0, None
        c1 = self._connect_sphere_1_options[self._connect_sphere_1_model.get_item_value_model().as_int]
        return c0, c1

    def get_preview_enabled(self) -> bool:
        """Return whether sphere-generation preview is currently active.

        Returns:
            True if preview generation is active.
        """
        return self._preview_active

    # ------------------------------------------------------------------
    # Callbacks
    # ------------------------------------------------------------------
    def _trigger_preview_generate_spheres_for_link(self) -> None:
        self._generate_spheres_for_link()

    def _on_collision_sphere_select_0(self, model: object = None, val: object = None) -> None:
        if self._connect_sphere_1_combobox is None:
            return
        sphere_0_name, sphere_1_name = self.get_selected_collision_spheres()
        if sphere_0_name is not None:
            pruned_names = self._connect_sphere_0_options[:]
            pruned_names.pop(self._connect_sphere_0_options.index(sphere_0_name))
        else:
            pruned_names = []

        self._connect_sphere_1_options = pruned_names
        self._connect_sphere_1_model = DynamicComboBoxModel(pruned_names)
        self._connect_sphere_1_combobox.model = self._connect_sphere_1_model

        if sphere_1_name in pruned_names:
            self._connect_sphere_1_model.get_item_value_model().set_value(int(pruned_names.index(sphere_1_name)))

    def _on_toggle_preview(self, model: object = None) -> None:
        if self._preview_active:
            self._preview_active = False
            self._state.collision_sphere_editor.clear_preview()
        else:
            self._preview_active = True
            self._generate_spheres_for_link()

    def _on_generate_spheres(self) -> None:
        self._generate_spheres_for_link(preview=False)
        self.refresh_collision_sphere_comboboxes(keep_sphere_selection=True)
        if self._num_spheres_field is not None:
            self._num_spheres_field.set_value(0)

    def _on_add_sphere(self) -> None:
        if (
            self._add_sphere_radius is None
            or self._add_sphere_translation_x is None
            or self._add_sphere_translation_y is None
            or self._add_sphere_translation_z is None
        ):
            return
        link_path = self.get_selected_link_path()
        if link_path is None:
            return
        radius = self._add_sphere_radius.get_value_as_float()
        translation = np.array(
            [
                self._add_sphere_translation_x.get_value_as_float(),
                self._add_sphere_translation_y.get_value_as_float(),
                self._add_sphere_translation_z.get_value_as_float(),
            ]
        )
        self._state.collision_sphere_editor.add_sphere(link_path, translation, radius)
        self.refresh_collision_sphere_comboboxes(keep_sphere_selection=True)

    def _on_connect_spheres(self) -> None:
        if self._connect_sphere_num is None:
            return
        c0, c1 = self.get_selected_collision_spheres()
        link_path = self.get_selected_link_path()
        if link_path is None:
            return
        if c1 is None:
            carb.log_warn("Please select two distinct collision spheres to Connect Spheres")
            return

        num = self._connect_sphere_num.get_value_as_int()
        self._state.collision_sphere_editor.interpolate_spheres(link_path + c0, link_path + c1, num)
        self.refresh_collision_sphere_comboboxes(keep_sphere_selection=True)

    def _on_scale_spheres(self) -> None:
        if self._scale_factor_field is None:
            return
        link_path = self.get_selected_link_path()
        if link_path is None:
            return
        factor = self._scale_factor_field.get_value_as_float()
        self._state.collision_sphere_editor.scale_spheres(link_path, factor)

    def _on_clear_link_spheres(self) -> None:
        link_path = self.get_selected_link_path()
        if link_path is None:
            return
        self._state.collision_sphere_editor.clear_link_spheres(link_path)
        self.refresh_collision_sphere_comboboxes()

    # ------------------------------------------------------------------
    # Sphere generation
    # ------------------------------------------------------------------
    def _generate_spheres_for_link(self, preview: bool = True) -> None:
        """Generate or preview spheres for the currently selected link and mesh.

        Args:
            preview: Whether to generate preview spheres instead of committed spheres.
        """
        if preview and not self._preview_active:
            return

        link_name = self._selected_link
        if link_name is None:
            return
        mesh_list = self._state.link_to_meshes.get(link_name, [])
        if not mesh_list:
            carb.log_warn(
                f"Could not generate spheres for any meshes in link {link_name}.  This is likely "
                f"due to all meshes nested under {link_name} being instanceable"
            )
            return

        if self._mesh_model is None:
            return
        mesh_index = self._mesh_model.get_item_value_model().as_int
        if mesh_index < 0 or mesh_index >= len(mesh_list):
            return
        mesh = mesh_list[mesh_index]

        if self._num_spheres_field is None or self._radius_offset_field is None:
            return
        num_spheres = self._num_spheres_field.get_value_as_int()
        if num_spheres <= 0:
            self._state.collision_sphere_editor.clear_preview()
            return

        radius_offset = self._radius_offset_field.get_value_as_float()

        link_path = self.get_selected_link_path()
        if link_path is None:
            return
        mesh_path = link_path + mesh

        link_frame_points, face_inds, vert_cts = sphere_generation.compute_link_frame_mesh(link_path, mesh_path)
        self._state.collision_sphere_editor.generate_spheres(
            link_path, link_frame_points, face_inds, vert_cts, num_spheres, radius_offset, preview
        )
