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

"""ROS 2 URDF importer UI workflow."""

import os
import re
import tempfile
import typing
from functools import partial

import carb
import omni
import omni.ui as ui
import omni.usd
from isaacsim.asset.importer.urdf import URDFImporter, URDFImporterConfig

from .robot_definition_reader import RobotDefinitionReader
from .ros2_option_widget import Ros2UrdfOptionWidget

EXTENSION_NAME = "Import from ROS2 URDF Node"


class RobotDescription:
    """Manage the ROS 2 URDF node import workflow.

    Example:

    .. code-block:: python

        >>> robot_description = RobotDescription()  # doctest: +SKIP
    """

    def __init__(self) -> None:
        self._models: dict[str, typing.Any] = {}
        self._config = URDFImporterConfig()
        self._reset_config()
        self.urdf_importer = URDFImporter(self._config)
        self._last_urdf_path: str | None = None

        self._option_widget = Ros2UrdfOptionWidget(
            self._models,
            self._config,
            self._on_node_changed,
            self._on_import_clicked,
        )
        self._window = ui.Window(EXTENSION_NAME, width=500, height=400, visible=False)
        with self._window.frame:
            with ui.ScrollingFrame(horizontal_scrollbar_policy=ui.ScrollBarPolicy.SCROLLBAR_ALWAYS_OFF):
                self._option_widget.build_options()

        self._sync_stage_settings()
        self._option_widget.set_refresh_enabled(False)
        self._option_widget.set_import_enabled(False)

    def _on_description_received(self, urdf_description: str, package_found: bool = False) -> None:
        """Handle URDF text retrieved from the ROS 2 node.

        Args:
            urdf_description: URDF document string from the node.
            package_found: Whether ROS package URLs were resolved.
        """
        match = re.search(r'<robot[^>]*name=["\']([^"\']+)["\']', urdf_description)
        self.package_name = match.group(1) if match else "robot"

        out_dir = tempfile.mkdtemp(prefix="ros2_urdf_")
        out_path = os.path.normpath(os.path.join(out_dir, f"{self.package_name}.urdf"))
        carb.log_info(f"Writing URDF to {out_path}")
        with open(out_path, "w", encoding="utf-8") as f:
            f.write(urdf_description)

        self.urdf_description = urdf_description
        self._last_urdf_path = os.path.normpath(out_path)
        if package_found:
            self._set_status("ROS package paths resolved")
        else:
            self._set_status("ROS package paths could not be resolved", color=0xFF0000FF)

        def _enable_ui() -> None:
            self._option_widget.set_refresh_enabled(True)
            self._option_widget.set_import_enabled(True)

        app = omni.kit.app.get_app()
        if hasattr(app, "post_to_main_thread"):
            app.post_to_main_thread(_enable_ui)
        else:
            _enable_ui()

    def _on_node_changed(self, model: ui.AbstractValueModel) -> None:
        """Request a new URDF description for the provided node.

        Args:
            model: String model containing the ROS 2 node name.
        """
        value = model.get_value_as_string().strip()
        self._set_status("")
        self._option_widget.set_import_enabled(False)
        self._last_urdf_path = None
        if not value:
            return
        self._sync_config_from_models()
        if hasattr(self.urdf_importer, "robot_frame"):
            self.urdf_importer.robot_frame.visible = False
        lister = RobotDefinitionReader()
        lister.description_received_fn = partial(self._on_description_received)
        lister.status_fn = self._set_status
        lister.start_get_robot_description(value)

    def _on_import_clicked(self) -> None:
        """Import the last received URDF file into the stage."""
        carb.log_info("ROS2 URDF Importer: Import button clicked")

        normalized_urdf_path = os.path.normpath(self._last_urdf_path)
        if not os.path.exists(normalized_urdf_path):
            if self.urdf_description:
                with open(normalized_urdf_path, "w", encoding="utf-8") as f:
                    f.write(self.urdf_description)
            else:
                carb.log_error(f"ROS2 URDF Importer: failed to import or save URDF file {normalized_urdf_path}")
                return

        carb.log_info(f"ROS2 URDF Importer: Importing {normalized_urdf_path}")
        self._sync_config_from_models()
        no_output_dir = self._config.usd_path is None
        if no_output_dir:
            carb.log_warn(
                f"ROS2 URDF Importer: No USD output directory specified, using output path '{os.path.dirname(normalized_urdf_path)}'. "
                "Set the 'USD Output' folder in the import window to control where the USD is saved."
            )
        self._config.urdf_path = normalized_urdf_path
        self.urdf_importer.config = self._config
        output_path = os.path.normpath(self.urdf_importer.import_urdf())
        if not output_path:
            carb.log_error(f"ROS2 URDF Importer: Failed to import {normalized_urdf_path}")
            return
        if no_output_dir:
            carb.log_warn(f"ROS2 URDF Importer: USD written to {output_path}")
        carb.log_info(f"ROS2 URDF Importer: Opening stage {output_path}")
        result = omni.usd.get_context().open_stage(output_path)
        if not result:
            carb.log_error(f"ROS2 URDF Importer: Failed to open stage at {output_path}")
            return

        try:
            os.remove(normalized_urdf_path)
        except OSError as exc:
            carb.log_warn(f"ROS2 URDF Importer: Failed to delete URDF file {normalized_urdf_path}: {exc}")

        if self._config.debug_mode:
            with open(
                os.path.normpath(os.path.join(os.path.dirname(output_path), f"{self.package_name}.urdf")),
                "w",
                encoding="utf-8",
            ) as f:
                f.write(self.urdf_description)
        return

    def _set_status(self, text: str, color: int = 0xFF5CB85C) -> None:
        """Update the status label from any thread.

        Args:
            text: Status text to display.
            color: Status text color.
        """
        if not hasattr(self._option_widget, "set_status"):
            return
        app = omni.kit.app.get_app()
        if hasattr(app, "post_to_main_thread"):
            app.post_to_main_thread(lambda: self._option_widget.set_status(text, color))
        else:
            self._option_widget.set_status(text, color)

    def shutdown(self) -> None:
        """Release UI resources and detach from the importer.

        Example:

        .. code-block:: python

            >>> robot_description.shutdown()  # doctest: +SKIP
        """
        if self.urdf_importer:
            if self._window is not None:
                self._window.visible = False
                self._window.destroy()
                self._window = None

    def show_window(self) -> None:
        """Show and focus the ROS 2 URDF import window.

        Example:

        .. code-block:: python

            >>> robot_description.show_window()  # doctest: +SKIP
        """
        self._window.visible = True
        self._window.focus()

    def _reset_config(self) -> None:
        """Reset the importer configuration to defaults."""
        self._config.usd_path = None
        self._config.merge_mesh = False
        self._config.debug_mode = False
        self._config.collision_from_visuals = False
        self._config.collision_type = "Convex Hull"
        self._config.allow_self_collision = False
        self._config.ros_package_paths = []

    def _sync_stage_settings(self) -> None:
        """Sync output path settings from the current stage."""
        if "dst_path" not in self._models:
            return
        self._models["dst_path"].set_value(self._get_dest_folder())

    def _get_dest_folder(self) -> str:
        """Return the destination folder derived from the active stage.

        Returns:
            Destination folder path, or a default label when unavailable.
        """
        stage = omni.usd.get_context().get_stage()
        if stage:
            path = stage.GetRootLayer().identifier
            if not path.startswith("anon"):
                basepath = path[: path.rfind("/")]
                if path.rfind("/") < 0:
                    basepath = path[: path.rfind("\\")]
                return os.path.normpath(basepath)
        return "System temp directory (Default)"

    def _sync_config_from_models(self) -> None:
        """Update configuration values from UI models."""
        output_path = self._get_output_path()
        self._config.usd_path = output_path
        self._config.ros_package_paths = []

    def _get_output_path(self) -> str | None:
        """Return the resolved USD output folder from the UI.

        Returns:
            Output folder path, or None if not configured.
        """
        model = self._models.get("dst_path")
        if not model:
            return None
        output_path = model.get_value_as_string()
        if output_path == "System temp directory (Default)":
            return None
        output_path = os.path.normpath(output_path)
        if output_path and not os.path.isdir(output_path):
            output_path = os.path.normpath(os.path.dirname(output_path))
        return os.path.normpath(output_path) if output_path else None
