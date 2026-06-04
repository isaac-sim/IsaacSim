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

"""Export options widget for the URDF exporter UI."""

from __future__ import annotations

from typing import Any

import omni.ui as ui
from isaacsim.gui.components.element_wrappers import CheckBox, DropDown, StringField

from .style import get_option_style


class OptionWidget:
    """Builds and manages the export options panel."""

    def __init__(self) -> None:
        self._mesh_dir = "meshes"
        self._mesh_path_prefix = "./"
        self._root = None
        self._visualize_collision_meshes = False
        self._package_name = ""
        self._use_physx_inertia = True

    @property
    def mesh_dir_name(self) -> str:
        """Mesh directory name."""
        return self._mesh_dir or "meshes"

    @property
    def mesh_path_prefix(self) -> str:
        """Mesh path prefix."""
        return self._mesh_path_prefix

    @property
    def root_prim_path(self) -> str | None:
        """Root prim path."""
        return self._root

    @property
    def visualize_collision_meshes(self) -> bool:
        """Whether to visualize collision meshes."""
        return self._visualize_collision_meshes

    @property
    def package_name(self) -> str:
        """ROS package name."""
        return self._package_name

    @property
    def use_physx_inertia(self) -> bool:
        """Whether to use PhysX-computed inertia."""
        return self._use_physx_inertia

    def cleanup(self) -> None:
        """Reset widget state."""
        self._mesh_dir = None
        self._mesh_path_prefix = ""
        self._root = None
        self._visualize_collision_meshes = False

    def _on_value_changed(self, param_name: str, new_value: Any) -> None:
        setattr(self, f"_{param_name}", new_value)

    def build(self) -> None:
        """Build the export options UI panel."""
        with ui.VStack(style=get_option_style(), spacing=5, height=0):
            StringField(
                "Mesh Folder Name",
                default_value="meshes",
                tooltip="Folder name for mesh files. Defaults to 'meshes'.",
                use_folder_picker=False,
                on_value_changed_fn=lambda v: self._on_value_changed("mesh_dir", v),
            )

            mesh_path_prefix_options = ["./", "file://", "package://"]
            self._mesh_path_prefix = "./"

            def on_mesh_path_prefix_changed(new_value: str) -> None:
                self._on_value_changed("mesh_path_prefix", new_value)
                self._mesh_path_prefix = new_value
                self._package_name_frame.visible = new_value == "package://"

            dropdown = DropDown(
                label="Mesh Path Prefix",
                tooltip="Prefix to add to URDF mesh filename values.",
                populate_fn=lambda: mesh_path_prefix_options,
                on_selection_fn=on_mesh_path_prefix_changed,
                keep_old_selections=False,
                add_flourish=True,
            )

            self._package_name_frame = ui.Frame()
            with self._package_name_frame:
                with ui.HStack():
                    ui.Spacer(width=20)
                    StringField(
                        "Package Name",
                        default_value="",
                        tooltip="Name of the ROS package for 'package://' mesh paths.",
                        on_value_changed_fn=lambda v: setattr(self, "_package_name", v),
                    )
            dropdown.repopulate()

            StringField(
                "Root Prim Path",
                default_value="",
                tooltip="Root prim path of the robot to be exported. Defaults to the default prim.",
                on_value_changed_fn=lambda v: self._on_value_changed("root", v),
            )

            CheckBox(
                "Visualize Collisions",
                default_value=False,
                tooltip="Visualization collider meshes even if their visibility is disabled.",
                on_click_fn=lambda v: self._on_value_changed("visualize_collision_meshes", v),
            )
