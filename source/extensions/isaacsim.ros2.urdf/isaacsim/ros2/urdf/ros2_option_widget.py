# SPDX-FileCopyrightText: Copyright (c) 2024-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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


"""UI option widgets for the ROS 2 URDF node importer."""

import typing

import omni.ui as ui
from isaacsim.asset.importer.urdf import URDFImporterConfig
from isaacsim.asset.importer.urdf.ui.impl.style import get_option_style
from isaacsim.gui.components.ui_utils import checkbox_builder, dropdown_builder, str_builder, string_filed_builder


def _option_header(collapsed: bool, title: str) -> None:
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


def _option_frame(
    title: str,
    build_content_fn: typing.Callable[[], None],
    collapse_fn: typing.Callable[[bool], None] | None = None,
) -> None:
    """Build a collapsable options frame.

    Args:
        title: Frame title to display.
        build_content_fn: Callback that builds the frame content.
        collapse_fn: Optional callback invoked on collapse changes.
    """
    with ui.CollapsableFrame(
        title,
        name="option",
        height=0,
        collapsed=False,
        build_header_fn=_option_header,
        collapsed_changed_fn=collapse_fn,
    ):
        with ui.HStack():
            ui.Spacer(width=2)
            build_content_fn()
            ui.Spacer(width=2)


class Ros2UrdfOptionWidget:
    """Build and manage option widgets for ROS 2 URDF node imports.

    Args:
        models: Dictionary used to store UI models.
        config: Import configuration to update from the UI.
        on_node_changed: Optional callback invoked on node changes.
        on_import_clicked: Optional callback invoked on import.

    Example:

    .. code-block:: python

        >>> models = {}
        >>> config = URDFImporterConfig()
        >>> widget = Ros2UrdfOptionWidget(models, config)
        >>> widget.build_options()  # doctest: +SKIP
    """

    def __init__(
        self,
        models: dict[str, typing.Any],
        config: URDFImporterConfig,
        on_node_changed: typing.Callable[[ui.AbstractValueModel], None] | None = None,
        on_import_clicked: typing.Callable[[], None] | None = None,
    ) -> None:
        self._models = models
        self._config = config
        self._on_node_changed = on_node_changed
        self._on_import_clicked = on_import_clicked
        self._collision_type_frame = None
        self._refresh_button = None
        self._status_label = None
        self._import_button = None

    @property
    def models(self) -> dict[str, typing.Any]:
        """Models dictionary used by the widget.

        Returns:
            Dictionary mapping model names to their UI model instances.
        """
        return self._models

    @property
    def config(self) -> URDFImporterConfig:
        """Importer configuration instance.

        Returns:
            The URDFImporterConfig object used for import settings.
        """
        return self._config

    def build_options(self) -> None:
        """Build all option frames in the UI.

        Example:

        .. code-block:: python

            >>> widget.build_options()  # doctest: +SKIP
        """
        with ui.VStack(style=get_option_style()):
            self._build_model_frame()
            self._build_colliders_frame()
            self._build_options_frame()
            self._build_import_button()

    def set_refresh_enabled(self, enabled: bool) -> None:
        """Enable or disable the refresh button.

        Args:
            enabled: Whether the refresh button is enabled.

        Example:

        .. code-block:: python

            >>> widget.set_refresh_enabled(True)  # doctest: +SKIP
        """
        if self._refresh_button is not None:
            self._refresh_button.enabled = enabled

    def get_ros2_node(self) -> str:
        """Return the ROS 2 node name from the UI.

        Returns:
            ROS 2 node name string, or an empty string when unset.

        Example:

        .. code-block:: python

            >>> widget.get_ros2_node()  # doctest: +SKIP
        """
        model = self._models.get("ros2_node")
        return model.get_value_as_string() if model else ""

    def set_status(self, text: str, color: int = 0xFF5CB85C) -> None:
        """Update the status label under the ROS 2 node field.

        Args:
            text: Status text to display.
            color: Text color to apply.

        Example:

        .. code-block:: python

            >>> widget.set_status("robot_state_publisher found")  # doctest: +SKIP
        """
        if self._status_label is None:
            return
        self._status_label.text = text
        self._status_label.visible = bool(text)
        if color is not None:
            self._status_label.style = {"color": color}

    def set_import_enabled(self, enabled: bool) -> None:
        """Enable or disable the import button.

        Args:
            enabled: Whether the import button is enabled.

        Example:

        .. code-block:: python

            >>> widget.set_import_enabled(True)  # doctest: +SKIP
        """
        if self._import_button is not None:
            self._import_button.enabled = enabled

    def _build_model_frame(self) -> None:
        """Build the Model options frame."""

        def build_model_content() -> None:
            with ui.VStack(spacing=0):
                self._models["ros2_node"] = str_builder(
                    label="ROS 2 Node",
                    default_val="",
                    tooltip="ROS 2 node containing the robot_description parameter",
                    use_folder_picker=False,
                    identifier="ros2_urdf_node_name",
                    label_width=90,
                )
                if self._on_node_changed is not None:
                    self._models["ros2_node"].add_end_edit_fn(self._on_node_changed)

                ui.Spacer(height=2)
                with ui.HStack(height=ui.Pixel(18)):
                    self._status_label = ui.Label("", name="ros2_node_status", width=0)
                    self._status_label.visible = False

                ui.Spacer(height=4)
                with ui.HStack():
                    ui.Spacer(width=ui.Fraction(1))
                    self._refresh_button = ui.Button(
                        "Find Node",
                        clicked_fn=self._on_refresh_clicked,
                        width=ui.Pixel(90),
                        identifier="ros2_urdf_find_node",
                    )

                ui.Spacer(height=6)
                ui.Label("USD Output")
                self._models["dst_path"] = string_filed_builder(
                    tooltip="USD output folder for the imported robot",
                    default_val="System temp directory (Default)",
                    folder_dialog_title="Select Output Folder",
                    folder_button_title="Select Folder",
                    read_only=True,
                    identifier="ros2_urdf_output_path",
                )

        _option_frame("Model", build_model_content)

    def _build_colliders_frame(self) -> None:
        """Build the Colliders options frame."""

        def build_colliders_content() -> None:
            def set_collision_type(value: str) -> None:
                self._config.collision_type = value

            def set_allow_self_collision(value: bool) -> None:
                self._config.allow_self_collision = value

            with ui.VStack(spacing=4):
                self._models["collision_from_visuals"] = checkbox_builder(
                    "Collision From Visuals",
                    tooltip="If True, collision geoms will be generated from visual geometries",
                    default_val=False,
                    on_clicked_fn=self._on_collision_from_visuals_changed,
                    identifier="ros2_urdf_collision_from_visuals",
                )

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
                                ],
                                on_clicked_fn=set_collision_type,
                                identifier="ros2_urdf_collision_type",
                            )
                        ui.Spacer(width=4)
                    ui.Spacer(height=4)

                self._collision_type_frame.visible = False

                self._models["allow_self_collision"] = checkbox_builder(
                    "Allow Self-Collision",
                    tooltip=(
                        "If true, allows self intersection between links in the robot, can cause instability "
                        "if collision meshes between links are self intersecting"
                    ),
                    default_val=False,
                    on_clicked_fn=set_allow_self_collision,
                    identifier="ros2_urdf_allow_self_collision",
                )

        _option_frame("Colliders", build_colliders_content)

    _BASE_TYPE_ITEMS = ["Source", "Fixed", "Mobile"]
    _BASE_TYPE_TO_FIX_BASE = {"Source": None, "Fixed": True, "Mobile": False}

    def _set_base_type(self, value: str) -> None:
        """Map the Base Type dropdown selection onto ``config.fix_base``."""
        self._config.fix_base = self._BASE_TYPE_TO_FIX_BASE[value]

    def _build_options_frame(self) -> None:
        """Build the Options frame."""

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
                    identifier="ros2_urdf_robot_type",
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
                    identifier="ros2_urdf_base_type",
                    show_flourish=False,
                    label_width=90,
                )

                self._models["merge_mesh"] = checkbox_builder(
                    "Merge Mesh",
                    tooltip="If True, merges meshes where possible to optimize the model",
                    default_val=False,
                    on_clicked_fn=set_merge_mesh,
                    identifier="ros2_urdf_merge_mesh",
                )

                self._models["debug_mode"] = checkbox_builder(
                    "Debug Mode",
                    tooltip="If True, enables debug mode with additional logging and visualization",
                    default_val=False,
                    on_clicked_fn=set_debug_mode,
                    identifier="ros2_urdf_debug_mode",
                )

        _option_frame("Options", build_options_content)

    def _build_import_button(self) -> None:
        """Build the import button at the bottom of the UI."""
        callback = self._on_import_clicked
        if callback is None:
            callback = lambda: None
        ui.Spacer(height=8)
        with ui.HStack():
            ui.Spacer(width=ui.Fraction(1))
            self._import_button = ui.Button(
                "Import",
                clicked_fn=callback,
                width=ui.Pixel(90),
                height=ui.Pixel(24),
                identifier="ros2_urdf_import",
            )

    def _on_collision_from_visuals_changed(self, value: bool) -> None:
        """Toggle visibility of collision type dropdown.

        Args:
            value: Boolean indicating if collision from visuals is enabled.
        """
        self._config.collision_from_visuals = value
        if self._collision_type_frame is not None:
            self._collision_type_frame.visible = value

    def _on_refresh_clicked(self) -> None:
        """Handle the refresh button click."""
        if self._on_node_changed is None:
            return
        model = self._models.get("ros2_node")
        if model is None:
            return
        self._on_node_changed(model)
