# SPDX-FileCopyrightText: Copyright (c) 2023-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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

"""UI option widgets for the URDF importer."""

from collections.abc import Callable

import omni.ui as ui
from isaacsim.asset.importer.urdf import URDFImporterConfig
from isaacsim.gui.components.ui_utils import checkbox_builder, dropdown_builder, string_filed_builder

from .style import get_option_style
from .ui_utils import RosPackageDelegate, RosPackageItem, RosPackageModel


def option_header(collapsed: bool, title: str) -> None:
    """Build the collapsable frame header UI.

    Args:
        collapsed: Whether the frame is collapsed.
        title: Header title to display.

    """
    with ui.HStack(height=22):
        ui.Spacer(width=4)
        with ui.VStack(width=10):
            ui.Spacer()
            if collapsed:
                triangle = ui.Triangle(height=7, width=5)
                triangle.alignment = ui.Alignment.RIGHT_CENTER
            else:
                triangle = ui.Triangle(height=5, width=7)
                triangle.alignment = ui.Alignment.CENTER_BOTTOM
            ui.Spacer()
        ui.Spacer(width=4)
        ui.Label(title, name="collapsable_header", width=0)
        ui.Spacer(width=3)
        ui.Line()


def option_frame(
    title: str,
    build_content_fn: Callable[[], None],
    collapse_fn: Callable[[bool], None] | None = None,
) -> None:
    """Build a collapsable options frame.

    Args:
        title: Frame title to display.
        build_content_fn: Callback that builds the frame content.
        collapse_fn: Optional callback invoked on collapse changes.

    """
    with ui.CollapsableFrame(
        title, name="option", height=0, collapsed=False, build_header_fn=option_header, collapsed_changed_fn=collapse_fn
    ):
        with ui.HStack():
            ui.Spacer(width=2)
            build_content_fn()
            ui.Spacer(width=2)


class OptionWidget:
    """Build and manage option widgets for the URDF importer UI.

    Args:
        models: Dictionary used to store UI models.
        config: Import configuration to update from the UI.

    """

    def __init__(self, models: dict[str, ui.AbstractValueModel], config: URDFImporterConfig) -> None:
        self._models = models
        self._config = config
        self._ros_package_table_frame = None
        self._ros_package_model: RosPackageModel | None = None
        self._ros_package_delegate = None
        self._ros_package_tree = None

    @property
    def models(self) -> dict[str, ui.AbstractValueModel]:
        """Return the models dictionary used by the widget.

        Returns:
            Map of option keys to Omni UI value models.
        """
        return self._models

    @property
    def config(self) -> URDFImporterConfig:
        """Return the importer configuration instance.

        Returns:
            Live ``URDFImporterConfig`` edited by this widget.
        """
        return self._config

    def build_options(self) -> None:
        """Build all option frames in the UI."""
        with ui.VStack(style=get_option_style()):
            self._build_model_frame()
            self._build_colliders_frame()
            self._build_options_frame()

    def _build_model_frame(self) -> None:
        """Build the Model options frame (USD output path and ROS package table)."""

        def build_model_content() -> None:
            with ui.VStack(spacing=0):
                ui.Label("USD Output")
                self._models["dst_path"] = string_filed_builder(
                    tooltip="USD file to store instanceable meshes in",
                    default_val="Same as Imported Model(Default)",
                    folder_dialog_title="Select Output File",
                    folder_button_title="Select File",
                    read_only=True,
                    identifier="urdf_output_path",
                )
                ui.Spacer(height=4)

                ui.Label("ROS Package List")
                ui.Spacer(height=2)
                self._ros_package_table_frame = ui.Frame()
                self._build_ros_package_table([("", "")])
                ui.Spacer(height=2)
                with ui.HStack():
                    ui.Spacer(width=ui.Fraction(1))
                    ui.Button(
                        "Add Row",
                        clicked_fn=self._add_ros_package_row,
                        width=ui.Pixel(90),
                        identifier="urdf_add_ros_package_row",
                    )

        option_frame("Model", build_model_content)

    def _build_colliders_frame(self) -> None:
        """Build the Colliders options frame.

        Creates UI elements for:
        - Collision from visuals checkbox
        - Collision type dropdown (shown only when collision from visuals is checked)
        - Allow self-collision checkbox
        """

        def build_colliders_content() -> None:
            def set_collision_type(value: str) -> None:
                self._config.collision_type = value

            def set_allow_self_collision(value: bool) -> None:
                self._config.allow_self_collision = value

            with ui.VStack(spacing=4):
                # Collision from visuals checkbox
                self._models["collision_from_visuals"] = checkbox_builder(
                    "Collision From Visuals",
                    tooltip="If True, collision geoms will be generated from visual geometries",
                    default_val=False,
                    on_clicked_fn=self._on_collision_from_visuals_changed,
                    identifier="urdf_collision_from_visuals",
                )

                # Collision type dropdown (initially hidden, on separate line)
                self._collision_type_frame = ui.VStack(spacing=0, height=0)
                with self._collision_type_frame:
                    ui.Spacer(height=4)
                    with ui.HStack():
                        ui.Spacer(width=4)
                        with ui.VStack(width=ui.Fraction(1)):
                            self._models["collision_type"] = dropdown_builder(
                                "Collision Type",
                                tooltip="Type of collision geometry to use when generating from visuals",
                                default_val=0,
                                items=[
                                    "Convex Hull",
                                    "Convex Decomposition",
                                    "Bounding Sphere",
                                    "Bounding Cube",
                                    # "SDF Mesh",
                                    # "Sphere Approximation",
                                ],
                                on_clicked_fn=set_collision_type,
                                identifier="urdf_collision_type",
                            )
                        ui.Spacer(width=4)
                    ui.Spacer(height=4)

                # Initially hide collision type dropdown
                self._collision_type_frame.visible = False

                # Self-collision checkbox
                self._models["allow_self_collision"] = checkbox_builder(
                    "Allow Self-Collision",
                    tooltip="If true, allows self intersection between links in the robot, can cause instability if collision meshes between links are self intersecting",
                    default_val=False,
                    on_clicked_fn=set_allow_self_collision,
                    identifier="urdf_allow_self_collision",
                )

        option_frame("Colliders", build_colliders_content)

    _BASE_TYPE_ITEMS = ["Source", "Fixed", "Mobile"]
    _BASE_TYPE_TO_FIX_BASE = {"Source": None, "Fixed": True, "Mobile": False}

    def _set_base_type(self, value: str) -> None:
        """Map the Base Type dropdown selection onto ``config.fix_base``.

        Args:
            value: Selected base type label.
        """
        self._config.fix_base = self._BASE_TYPE_TO_FIX_BASE[value]

    def _build_options_frame(self) -> None:
        """Build the Options frame.

        Creates UI elements for:
        - Robot Type dropdown
        - Base Type dropdown (Source / Fixed / Mobile)
        - Merge Mesh checkbox (default: False)
        - Debug Mode checkbox (default: False)
        """

        def build_options_content() -> None:
            from isaacsim.asset.importer.utils.impl.importer_utils import ROBOT_TYPE_TOKENS

            def set_robot_type(value: str) -> None:
                self._config.robot_type = value

            def set_merge_mesh(value: bool) -> None:
                self._config.merge_mesh = value

            def set_debug_mode(value: bool) -> None:
                self._config.debug_mode = value

            with ui.VStack(spacing=4):

                self._models["robot_type"] = dropdown_builder(
                    "Robot Type",
                    tooltip="Category of robot for the Isaac robot schema (e.g. Manipulator, Humanoid)",
                    default_val=ROBOT_TYPE_TOKENS.index(self._config.robot_type),
                    items=ROBOT_TYPE_TOKENS,
                    on_clicked_fn=set_robot_type,
                    identifier="urdf_robot_type",
                    show_flourish=False,
                    label_width=90,
                )

                base_type_default = 0
                if self._config.fix_base is True:
                    base_type_default = 1
                elif self._config.fix_base is False:
                    base_type_default = 2
                self._models["base_type"] = dropdown_builder(
                    "Base Type",
                    tooltip=(
                        "How to anchor the robot's root link. "
                        "Source leaves the source URDF authoring untouched; "
                        "Fixed adds a world-to-root fixed joint; "
                        "Mobile removes any world-to-root fixed joint."
                    ),
                    default_val=base_type_default,
                    items=self._BASE_TYPE_ITEMS,
                    on_clicked_fn=self._set_base_type,
                    identifier="urdf_base_type",
                    show_flourish=False,
                    label_width=90,
                )

                self._models["merge_mesh"] = checkbox_builder(
                    "Merge Mesh",
                    tooltip="If True, merges meshes where possible to optimize the model",
                    default_val=False,
                    on_clicked_fn=set_merge_mesh,
                    identifier="urdf_merge_mesh",
                )

                self._models["debug_mode"] = checkbox_builder(
                    "Debug Mode",
                    tooltip="If True, enables debug mode with additional logging and visualization",
                    default_val=False,
                    on_clicked_fn=set_debug_mode,
                    identifier="urdf_debug_mode",
                )

        option_frame("Options", build_options_content)

    def _on_collision_from_visuals_changed(self, value: bool) -> None:
        """Toggle visibility of collision type dropdown.

        Args:
            value: Boolean indicating if collision from visuals is enabled.

        """
        self._config.collision_from_visuals = value
        self._collision_type_frame.visible = value

    def _add_ros_package_row(self) -> None:
        if not self._ros_package_model:
            return
        self._ros_package_model.add_row("", "")

    def _build_ros_package_table(self, rows_data: list[tuple[str, str]]) -> None:
        if not self._ros_package_table_frame:
            return
        self._ros_package_table_frame.clear()
        self.table_border_width = 2
        row_height = 28
        column_widths = [ui.Fraction(0.3), ui.Fraction(0.7)]

        if self._ros_package_model is None:
            self._ros_package_model = RosPackageModel(rows_data)
        elif not self._ros_package_model.get_rows():
            self._ros_package_model.add_row("", "")

        self._ros_package_delegate = RosPackageDelegate(
            row_height=row_height,
            border_width=self.table_border_width,
            on_delete=self._delete_ros_package_row,
        )

        total_rows = len(self._ros_package_model.get_rows()) + 1
        table_height = total_rows * row_height
        self._ros_package_table_frame.height = ui.Pixel(table_height)

        with self._ros_package_table_frame:
            self._ros_package_tree = ui.TreeView(
                self._ros_package_model,
                delegate=self._ros_package_delegate,
                root_visible=False,
                header_visible=True,
                columns_resizable=False,
                column_widths=column_widths,
                height=ui.Pixel(table_height),
                style={
                    "TreeView": {"background_color": 0x0},
                    "TreeView.Item": {"margin": 0},
                    "TreeView:selected": {"background_color": 0x0},
                },
                identifier="ros_package_table",
            )

    def get_ros_package_map(self) -> list[dict[str, str]]:
        """Get the ROS package mappings from the table.

        Returns:
            List of ROS package name/path mappings.

        """
        if not self._ros_package_model:
            return []
        ros_packages = []
        for name, path in self._ros_package_model.get_rows():
            name = name.strip()
            path = path.strip()
            if name:
                ros_packages.append({"name": name, "path": path})
        return ros_packages

    def populate_packages(self, packages: list[tuple[str, str]]) -> None:
        """Replace the current ROS package table rows with *packages*.

        Each entry is a ``(name, path)`` tuple.  The user can still edit,
        add, or remove rows after pre-population.

        Args:
            packages: Discovered package name/path pairs.

        """
        if not packages:
            return
        self._ros_package_model = RosPackageModel(packages)
        self._build_ros_package_table(packages)

    def _delete_ros_package_row(self, item: RosPackageItem) -> None:
        if not self._ros_package_model:
            return
        self._ros_package_model.remove_row(item)
        if not self._ros_package_model.get_rows():
            self._ros_package_model.add_row("", "")
