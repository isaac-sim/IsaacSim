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

"""Extension module for the Mesh Merge Tool UI."""

import gc
import weakref

import carb
import omni.ext
import omni.kit.commands
import omni.kit.utils
import omni.ui as ui
import omni.usd
from isaacsim.gui.components.element_wrappers import ScrollingWindow
from isaacsim.gui.components.menu import make_menu_item_description
from isaacsim.gui.components.style import get_style
from isaacsim.gui.components.ui_utils import btn_builder, cb_builder, combo_cb_str_builder, str_builder
from omni.kit.menu.utils import MenuItemDescription, add_menu_items, remove_menu_items

from .mesh_merger import MeshMerger

EXTENSION_NAME = "Mesh Merge Tool"


class Extension(omni.ext.IExt):
    """Mesh Merge Tool extension for combining multiple meshes into a single mesh."""

    def on_startup(self, ext_id: str) -> None:
        """Initialize the extension and create the UI window.

        Args:
            ext_id: The unique identifier for this extension instance.

        """
        carb.log_warn(
            f"Extension {EXTENSION_NAME} is deprecated since ISaac Sim 6.0.0. Replaced with the Scene Optimizer"
        )

        self._stage = omni.usd.get_context().get_stage()
        self._window = ScrollingWindow(
            title=EXTENSION_NAME, width=600, height=400, visible=False, dockPreference=ui.DockPreference.LEFT_BOTTOM
        )
        self._window.deferred_dock_in("Console", omni.ui.DockPolicy.DO_NOTHING)
        self._window.set_visibility_changed_fn(self._on_window)
        menu_entry = [
            make_menu_item_description(ext_id, EXTENSION_NAME, lambda a=weakref.proxy(self): a._menu_callback())
        ]
        self._menu_items = [MenuItemDescription("Robotics", sub_menu=menu_entry)]
        add_menu_items(self._menu_items, "Tools")
        self.models = {}
        self.parent_xform = None

        self.mesh_merger = MeshMerger(self._stage)

    def build_ui(self) -> None:
        """Build the user interface for the Mesh Merge Tool window."""
        with self._window.frame:
            with ui.HStack(spacing=5):
                with ui.VStack(height=0, spacing=5):
                    input_frame = ui.CollapsableFrame(
                        title="Input",
                        style=get_style(),
                        style_type_name_override="CollapsableFrame",
                        horizontal_scrollbar_policy=ui.ScrollBarPolicy.SCROLLBAR_AS_NEEDED,
                        vertical_scrollbar_policy=ui.ScrollBarPolicy.SCROLLBAR_ALWAYS_ON,
                    )

                    def input_changed(model: object) -> None:
                        if input_frame.collapsed:
                            input_frame.title = f"Input ({model.get_value_as_string()})"

                    def collapsed_changed_fn(frame: object, base_txt: str, model: object, collapsed: bool) -> None:
                        if collapsed:
                            frame.title = base_txt + f" ({model.get_value_as_string()})"
                        else:
                            frame.title = base_txt

                    with input_frame:
                        with ui.VStack(spacing=2, height=0):
                            self.models["input_mesh"] = str_builder("Source Prim", read_only=True)
                            self.models["input_mesh"].set_value("No Mesh Selected")
                            self.models["input_mesh"].add_value_changed_fn(lambda a: input_changed(a))
                            self.models["submesh"] = str_builder("Submeshes", read_only=True)
                            self.models["subset"] = str_builder("Geometry Subsets", read_only=True)
                            self.models["materials"] = str_builder("Materials", read_only=True)

                    input_frame.set_collapsed_changed_fn(
                        lambda c, f=input_frame, m=self.models["input_mesh"]: collapsed_changed_fn(f, "Input", m, c)
                    )
                    output_frame = ui.CollapsableFrame(
                        title="Output",
                        style=get_style(),
                        style_type_name_override="CollapsableFrame",
                        horizontal_scrollbar_policy=ui.ScrollBarPolicy.SCROLLBAR_AS_NEEDED,
                        vertical_scrollbar_policy=ui.ScrollBarPolicy.SCROLLBAR_ALWAYS_ON,
                    )
                    with output_frame:
                        with ui.VStack(spacing=2, height=0):
                            self.models["output_mesh"] = str_builder("Destination Prim", read_only=True)
                            self.models["output_mesh"].set_value("No Mesh Selected")

                            self.models["output_subset"] = str_builder("Geometry Subsets", read_only=True)
                    output_frame.set_collapsed_changed_fn(
                        lambda c, f=output_frame, m=self.models["output_mesh"]: collapsed_changed_fn(f, "Output", m, c)
                    )
                with ui.VStack(spacing=3, height=0):
                    self.parent_xform = cb_builder(
                        label="Clear Parent Transform",
                        tooltip="If selected, Creates merged mesh with origin at global orign, otherwise keeps origin at parent's origin",
                        on_clicked_fn=lambda a: setattr(self.mesh_merger, "clear_parent_xform", a),
                    )
                    self.deactivate_source = cb_builder(
                        label="Deactivate source assets",
                        tooltip="If selected, Deactivates all meshes used for creating the merged mesh",
                        on_clicked_fn=lambda a: setattr(self.mesh_merger, "deactivate_source", a),
                    )
                    self.override_looks_directory = combo_cb_str_builder(
                        "Combine Materials",
                        tooltip="If selected, replaces the path to all selected materials with the prim path provided.\n If the path doesn't exist, a Material scope will be created and all materials used will be moved to it",
                        on_clicked_fn=lambda a: self._on_stage_event(),
                        default_val=[False, ""],
                    )
                    self.override_looks_directory[1].add_value_changed_fn(
                        lambda a: self.on_mat_dest_changed(a.get_value_as_string())
                    )
                    self.mesh_merger.on_materials_changed_fn = self.on_mat_changed
                    btn_builder(label="Merge Selected Prim", text="Merge", on_clicked_fn=self._merge_mesh)

    def on_mat_changed(self, value: object) -> None:
        """Handle material change events from the mesh merger.

        Args:
            value: The new material destination path value.

        """
        if type(value) == str:
            self.override_looks_directory[1].set_value(value)

    def on_mat_dest_changed(self, value: str) -> None:
        """Handle material destination path change events from the UI.

        Args:
            value: The new material destination path entered by the user.

        """
        if self.mesh_merger.materials_destination != value:
            self.mesh_merger.materials_destination = value

    def _menu_callback(self) -> None:
        """Toggle the visibility of the Mesh Merge Tool window."""
        self._window.visible = not self._window.visible

    def _on_window(self, visible: bool) -> None:
        """Handle window visibility change events.

        Args:
            visible: Whether the window is now visible.

        """
        if self._window.visible:
            self._usd_context = omni.usd.get_context()
            if self._usd_context is not None:
                self._selection = self._usd_context.get_selection()
                self._events = self._usd_context.get_stage_event_stream()
                self._stage_event_sub = self._events.create_subscription_to_pop(
                    self._on_stage_event, name="Mesh merge tool stage event"
                )
            self.build_ui()
            self._on_stage_event()
        else:
            self._stage_event_sub = None

    def _on_stage_event(self, event: object = None) -> None:
        """Handle stage events such as selection changes.

        Updates the UI to reflect the currently selected prims and their mesh properties.

        Args:
            event: The stage event that triggered this callback. If None, forces a UI update.

        """
        if self._window.visible:
            self._stage = omni.usd.get_context().get_stage()
            if not event or event.type == int(omni.usd.StageEventType.SELECTION_CHANGED):
                selection = self._selection.get_selected_prim_paths()
                if selection:
                    self.mesh_merger.combine_materials = self.override_looks_directory[0].get_value_as_bool()
                    self.mesh_merger.materials_destination = self.override_looks_directory[1].get_value_as_string()
                    self.mesh_merger.update_selection(selection=selection, stage=self._stage)
                    curr_prim = self._stage.GetPrimAtPath(selection[0])

                    if len(selection) == 1:
                        self.models["input_mesh"].set_value(selection[0])
                    else:
                        self.models["input_mesh"].set_value(f"{selection[0]} (and {len(selection)-1} more)")
                    self.models["submesh"].set_value(self.mesh_merger.total_meshes)
                    self.models["subset"].set_value(self.mesh_merger.total_subsets)
                    self.models["materials"].set_value(self.mesh_merger.total_materials)
                    self.mesh_merger.output_mesh = "/Merged/" + str(curr_prim.GetName())
                    self.models["output_mesh"].set_value(self.mesh_merger.output_mesh)
                    self.models["output_subset"].set_value(self.mesh_merger.total_materials)
                else:
                    self.models["input_mesh"].set_value(f"None Selected")
                    self.models["submesh"].set_value(0)
                    self.models["subset"].set_value(0)
                    self.models["materials"].set_value(0)
                    self.models["output_mesh"].set_value("")
                    self.models["output_subset"].set_value(0)

    def _merge_mesh(self) -> None:
        """Execute the mesh merge operation using the current settings."""
        self.mesh_merger.merge_meshes()

    def on_shutdown(self) -> None:
        """Clean up resources when the extension is unloaded."""
        remove_menu_items(self._menu_items, "Tools")
        self._window = None
        gc.collect()
