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

"""Extension module for the isaacsim.util.camera_inspector that provides a Camera Inspector window for examining and interacting with cameras in the scene."""

from __future__ import annotations

import asyncio
import gc
import weakref
from collections.abc import Callable
from typing import Any, Final

import carb
import isaacsim.core.experimental.utils.prim as prim_utils
import isaacsim.core.experimental.utils.stage as stage_utils
import isaacsim.core.experimental.utils.transform as transform_utils
import numpy as np
import omni
import omni.kit.commands
import omni.ui as ui
import omni.usd
from isaacsim.core.experimental.objects import Camera
from isaacsim.core.rendering_manager import ViewportManager
from isaacsim.gui.components.element_wrappers import TextBlock
from isaacsim.gui.components.menu import make_menu_item_description
from isaacsim.gui.components.style import COLOR_W, COLOR_X, COLOR_Y, COLOR_Z
from isaacsim.gui.components.ui_utils import (
    BUTTON_WIDTH,
    add_line_rect_flourish,
    btn_builder,
    get_style,
    setup_ui_headers,
)
from omni.kit.menu.utils import MenuItemDescription, add_menu_items, remove_menu_items
from omni.kit.viewport.window import ViewportWindow, get_viewport_window_instances
from omni.kit.window.property.templates import LABEL_WIDTH

EXTENSION_NAME: Final[str] = "Camera Inspector"
SUPPORTED_AXES: Final[list[str]] = ["world", "usd", "ros"]

# 3x3 rotation sub-matrices derived from the USD camera convention homogeneous transforms.
# USD camera convention: +Y up, -Z forward (OpenGL/computer-graphics).
# World camera convention: +Z up, +X forward (robotics).
# ROS camera convention:   +Y down, +Z forward (computer vision).
#
# Both transforms are applied on the right of the USD rotation matrix:
#   R_target = R_usd @ _USD_TO_<target>
_USD_TO_WORLD_ROT: np.ndarray = np.array([[0, -1, 0], [0, 0, 1], [-1, 0, 0]], dtype=np.float32)
_USD_TO_ROS_ROT: np.ndarray = np.array([[1, 0, 0], [0, -1, 0], [0, 0, -1]], dtype=np.float32)


class Extension(omni.ext.IExt):
    """Extension class for the isaacsim.util.camera_inspector extension.

    Provides a Camera Inspector window that allows users to examine and interact with cameras in the scene.
    The extension creates a dockable UI window with controls for selecting cameras, creating viewports, and viewing
    camera pose information in both world and local coordinate systems.

    The main features include:
    - Automatic discovery and listing of all cameras in the scene
    - Real-time display of camera position and orientation in world and local coordinates
    - Support for different coordinate axis conventions (world, USD, ROS)
    - Ability to create new viewports for selected cameras
    - Assignment of cameras to existing viewports
    - Live updating of camera statistics as the scene changes

    The extension window is accessible through the Tools > Sensors menu and can be docked within the main
    Omniverse interface. Camera pose information is continuously updated and displayed in both numeric form
    and as copyable text output for use in other applications or scripts.
    """

    def on_startup(self, ext_id: str):
        """Initialize extension and UI elements.

        Args:
            ext_id: The extension identifier.
        """
        # Events
        self._usd_context = omni.usd.get_context()

        # Build Window
        self._window = ui.Window(
            title=EXTENSION_NAME,
            height=500,
            width=500,
            visible=False,
            dockPreference=ui.DockPreference.LEFT_BOTTOM,  # width_changed_fn=self._on_refresh
        )
        self._window.set_visibility_changed_fn(self._on_window)

        # UI
        self._task_ui_elements: dict[str, Any] = {}

        self._ext_id = ext_id
        menu_items = [
            make_menu_item_description(ext_id, EXTENSION_NAME, lambda a=weakref.proxy(self): a._menu_callback())
        ]
        self._menu_items = [MenuItemDescription(name="Sensors", sub_menu=menu_items)]
        add_menu_items(self._menu_items, "Tools")
        self._all_cameras: list[Camera] = []
        self._all_viewports: list[ViewportWindow] = []

        self.colors = {"X": COLOR_X, "Y": COLOR_Y, "Z": COLOR_Z, "W": COLOR_W}

        # Selection
        self._selected_axis_world = SUPPORTED_AXES[0]
        self._selected_axis_local = SUPPORTED_AXES[0]
        self._selected_camera = None
        self._selected_viewport = None

        self._camera_state_subscriber = None

    def on_shutdown(self):
        """Clean up extension resources and remove menu items."""
        self._usd_context = None
        remove_menu_items(self._menu_items, "Tools")
        if self._window:
            self._window = None
        self._camera_state_subscriber = None
        gc.collect()

    def _on_window(self, visible: bool) -> None:
        """Handle window visibility changes.

        When the window becomes visible, subscribes to stage events and builds the UI.
        When hidden, cleans up the USD context.

        Args:
            visible: Whether the window is visible.
        """
        if self._window.visible:
            # Subscribe to Stage and Timeline Events
            self._usd_context = omni.usd.get_context()

            self._build_ui()
        else:
            self._usd_context = None
            self._camera_state_subscriber = None

    def _menu_callback(self):
        """Toggle the window visibility when the menu item is clicked."""
        self._window.visible = not self._window.visible

    def _build_ui(self):
        """Build the main UI components including info panel and camera controls."""
        with self._window.frame:
            with ui.VStack(spacing=5, height=0):

                self._build_info_ui()

                self._build_camera_pane()

                self._on_refresh()

        async def dock_window():
            await omni.kit.app.get_app().next_update_async()

            def dock(space, name, location, pos=0.5):
                window = omni.ui.Workspace.get_window(name)
                if window and space:
                    window.dock_in(space, location, pos)
                return window

            tgt = ui.Workspace.get_window("Viewport")
            dock(tgt, EXTENSION_NAME, omni.ui.DockPosition.LEFT, 0.33)
            await omni.kit.app.get_app().next_update_async()

        self._task = asyncio.ensure_future(dock_window())

    ##################################
    # Callbacks
    ##################################

    def _on_refresh(self, width: object = None) -> None:
        """Get all cameras in the scene and add them to the camera manager.

        Args:
            width: Optional width parameter for refresh operation.
        """
        self._all_cameras = self._get_all_camera_objects()
        self._all_viewports = list(get_viewport_window_instances())

        self._update_camera_dropdown()
        self._update_viewport_dropdown()

        if self._all_cameras:
            if self._selected_camera is None:
                self._selected_camera = self._all_cameras[0]

            if self._camera_state_subscriber is None:
                self._camera_state_subscriber = carb.eventdispatcher.get_eventdispatcher().observe_event(
                    event_name=omni.kit.app.GLOBAL_EVENT_UPDATE,
                    on_event=self._update_camera_stats_ui,
                    observer_name="CameraInspectorExtension._on_refresh._update_camera_stats_ui",
                )

    def _on_camera_changed_event(self, option: int) -> None:
        """Handle camera selection changes from the dropdown.

        Updates the selected camera based on the dropdown selection and refreshes the camera statistics display.

        Args:
            option: The selected option from the camera dropdown.
        """
        option = self._task_ui_elements["Combo Camera"].get_item_value_model().as_int
        if option < len(self._all_cameras):
            self._selected_camera = self._all_cameras[option]
        else:
            err = f"Selected option {option} not available; available cameras: {[camera.paths[0] for camera in self._all_cameras]}"
            carb.log_warn(err)

        self._update_camera_stats_ui()

    def _on_viewport_changed_event(self, option: int) -> None:
        """Handle viewport selection changes from the dropdown.

        Updates the selected viewport based on the dropdown selection.

        Args:
            option: The selected option from the viewport dropdown.

        Raises:
            ValueError: If the selected option is not available in the viewport list.
        """
        option = self._task_ui_elements["Combo Viewport"].get_item_value_model().as_int
        if option < len(self._all_viewports):
            self._selected_viewport = self._all_viewports[option]
        else:
            err = f"Selected option {option} not available; available cameras: {[viewport.name for viewport in self._all_viewports]}"
            raise ValueError(err)

    def _on_axis_changed_event(self, option: int) -> None:
        """Handle axis selection changes for both world and local coordinate systems.

        Updates the selected axis for world and local camera coordinate systems and refreshes the camera statistics display.

        Args:
            option: The selected option from the axis dropdown.
        """
        option = self._task_ui_elements["World Camera Axis"].get_item_value_model().as_int
        self._selected_axis_world = SUPPORTED_AXES[option]

        option = self._task_ui_elements["Local Camera Axis"].get_item_value_model().as_int
        self._selected_axis_local = SUPPORTED_AXES[option]

        self._update_camera_stats_ui()

    def _on_create_viewport(self):
        """Create a viewport for the current camera.

        Creates a new viewport window configured to display the view from the currently selected camera.
        """
        if self._selected_camera is not None:
            ViewportManager.create_viewport_window(
                title=self._selected_camera.paths[0], camera=self._selected_camera.paths[0]
            )

            self._on_refresh()
        else:
            carb.log_warn("No camera selected. Cannot create viewport.")

    def _on_assign_camera(self):
        """Assigns the selected camera to the selected viewport."""
        if self._selected_camera is not None and self._selected_viewport is not None:
            omni.kit.commands.execute(
                "SetViewportCamera",
                camera_path=self._selected_camera.paths[0],
                viewport_api=self._selected_viewport.viewport_api,
            )
        else:
            carb.log_warn("No camera or viewport selected. Cannot assign camera to viewport.")

    ##################################
    # UI Builders
    ##################################

    def _build_info_ui(self):
        """Builds the information UI section with extension title, documentation link, and overview."""
        title = EXTENSION_NAME
        doc_link = "https://docs.isaacsim.omniverse.nvidia.com/latest/sensors/isaacsim_sensors_camera.html#camera-inspector-extension"

        overview = "This utility is used to inspect cameras in the scene.  "
        overview += "\n\nPress the 'Open in IDE' button to view the source code."

        setup_ui_headers(self._ext_id, __file__, title, doc_link, overview)

    def _build_camera_pane(self):
        """Builds the main camera pane containing all camera-related UI controls and display elements."""
        self._frame = ui.CollapsableFrame(
            title="Cameras",
            height=0,
            name="subFrame",
            collapsed=False,
            style=get_style(),
            horizontal_scrollbar_policy=ui.ScrollBarPolicy.SCROLLBAR_AS_NEEDED,
            vertical_scrollbar_policy=ui.ScrollBarPolicy.SCROLLBAR_ALWAYS_ON,
        )
        with self._frame:
            with ui.VStack(style=get_style(), spacing=5, height=0):
                btn_config = {
                    "label": "Get all cameras",
                    "text": "Refresh",
                    "tooltip": "Finds all cameras in the scene and adds them to the camera manager.",
                    "on_clicked_fn": self._on_refresh,
                }
                self._task_ui_elements["refresh_btn"] = btn_builder(**btn_config)

                # self._task_ui_elements["refresh_btn"].enabled = True
                # self._frame.visible = True

                self._task_ui_elements["CameraTextField"] = TextBlock(
                    "Camera State",
                    num_lines=6,
                    tooltip="Prints camera state to this field",
                    include_copy_button=True,
                )

                self._build_camera_viewport_dropdown(
                    button_text="Create Viewport",
                    label_text="Viewport",
                    button_fn=self._on_create_viewport,
                    dropdown_fn=self._on_viewport_changed_event,
                )

                self._build_camera_viewport_dropdown(
                    button_text="Assign Camera",
                    label_text="Camera",
                    button_fn=self._on_assign_camera,
                    dropdown_fn=self._on_camera_changed_event,
                )

                self._camera_axes_builder_config: dict[str, Any] = {
                    "label": "World Camera Axis",
                    "default_val": 0,
                    "items": SUPPORTED_AXES,
                    "tooltip": "Select the axis to use",
                    "on_clicked_fn": self._on_axis_changed_event,
                    "add_line": False,
                }
                self._task_ui_elements["World Camera Axis"] = self._build_single_dropdown(
                    **self._camera_axes_builder_config
                )

                world_transform_models = self._build_pos_quat_display(label="World")
                self._task_ui_elements["World Camera Position"] = world_transform_models[0:3]
                self._task_ui_elements["World Camera Orientation"] = world_transform_models[3:7]

                self._camera_axes_builder_config["label"] = "Local Camera Axis"
                self._task_ui_elements["Local Camera Axis"] = self._build_single_dropdown(
                    **self._camera_axes_builder_config
                )

                local_transform_models = self._build_pos_quat_display(label="Local")
                self._task_ui_elements["Local Camera Position"] = local_transform_models[0:3]
                self._task_ui_elements["Local Camera Orientation"] = local_transform_models[3:7]

    def _build_camera_viewport_dropdown(
        self,
        button_text: str,
        label_text: str,
        button_fn: Callable[[], None] | None = None,
        dropdown_fn: Callable[[int], None] | None = None,
    ) -> None:
        """Builds a combined UI element with a dropdown and button for camera-viewport operations.

        Args:
            button_text: Text to display on the button.
            label_text: Label text for the dropdown.
            button_fn: Function to call when button is clicked.
            dropdown_fn: Function to call when dropdown selection changes.
        """
        with ui.HStack(style=get_style()):
            ui.Label(label_text, width=LABEL_WIDTH, alignment=ui.Alignment.LEFT_CENTER)

            self._task_ui_elements[f"Combo {label_text}"] = ui.ComboBox(
                0, *self._all_viewports, name="ComboBox", alignment=ui.Alignment.LEFT_CENTER
            ).model

            self._task_ui_elements["Assign camera button"] = ui.Button(
                button_text.upper(),
                name="Button",
                width=BUTTON_WIDTH,
                clicked_fn=button_fn,
                style=get_style(),
                alignment=ui.Alignment.LEFT_CENTER,
            )

            add_line_rect_flourish(draw_line=False)

            def on_clicked_wrapper(model, val):
                if dropdown_fn is not None:
                    dropdown_fn(model.get_item_value_model().as_int)

            self._task_ui_elements[f"Combo {label_text}"].add_item_changed_fn(on_clicked_wrapper)

    def _build_single_dropdown(
        self,
        label: str = "",
        default_val: int = 0,
        items: list = [],
        tooltip: str = "",
        on_clicked_fn: Callable[[int], None] | None = None,
        add_line: bool = False,
        label_width: int = LABEL_WIDTH,
    ):
        """Builds a single dropdown UI element with label.

        Args:
            label: Label to display next to the dropdown.
            default_val: Index of the default selected item.
            items: List of items to populate the dropdown.
            tooltip: Tooltip text for the label.
            on_clicked_fn: Function to call when dropdown selection changes.
            add_line: Whether to add a decorative line after the dropdown.
            label_width: Width of the label in pixels.

        Returns:
            The ComboBox model for the created dropdown.
        """
        with ui.HStack():
            ui.Label(label, width=label_width, alignment=ui.Alignment.LEFT_CENTER, tooltip=tooltip)
            combo_box = ui.ComboBox(default_val, *items, name="ComboBox", alignment=ui.Alignment.LEFT_CENTER).model
            add_line_rect_flourish(add_line)

            def on_clicked_wrapper(model, val):
                if on_clicked_fn is not None:
                    on_clicked_fn(model.get_item_value_model().as_int)

            if on_clicked_fn is not None:
                combo_box.add_item_changed_fn(on_clicked_wrapper)

        return combo_box

    def _build_pos_quat_display(self, label: str = "World"):
        """Builds position and quaternion display UI elements for camera transform visualization.

        Args:
            label: Label prefix for the transform display.

        Returns:
            List of FloatDrag models for position (X, Y, Z) and orientation (W, X, Y, Z) values.
        """
        models = []

        def _build_model(label, all_axis):
            with ui.HStack():
                with ui.HStack(width=LABEL_WIDTH):
                    ui.Label(label, name="transform", width=50)
                    ui.Spacer()

                for axis in all_axis:
                    with ui.HStack():
                        with ui.ZStack(width=15):
                            ui.Rectangle(
                                width=15,
                                height=20,
                                style={
                                    "background_color": self.colors[axis],
                                    "border_radius": 3,
                                    "corner_flag": ui.CornerFlag.LEFT,
                                },
                            )
                            ui.Label(
                                axis, name="transform_label", alignment=ui.Alignment.CENTER, style={"color": 0xFFFFFFFF}
                            )
                        model = ui.FloatDrag(name="transform", enabled=False).model

                        models.append(model)
                        ui.Spacer(width=4)

        _build_model(f"{label} Position", all_axis=["X", "Y", "Z"])
        _build_model(f"{label} Orientation", all_axis=["W", "X", "Y", "Z"])

        return models

    ##################################
    # UI Updaters
    ##################################

    def _update_camera_dropdown(self):
        """Updates the camera dropdown with the current list of available cameras in the scene."""
        # Remove all old camera names
        for child in self._task_ui_elements["Combo Camera"].get_item_children():
            self._task_ui_elements["Combo Camera"].remove_item(child)

        # add all cameras to the dropdown
        for camera in self._all_cameras:
            self._task_ui_elements["Combo Camera"].append_child_item(None, ui.SimpleStringModel(camera.paths[0]))

    def _update_viewport_dropdown(self):
        """Updates the viewport dropdown with the current list of available viewport windows."""
        # Remove all old viewport names
        for child in self._task_ui_elements["Combo Viewport"].get_item_children():
            self._task_ui_elements["Combo Viewport"].remove_item(child)

        # add all viewports to the dropdown
        for viewport in self._all_viewports:
            self._task_ui_elements["Combo Viewport"].append_child_item(None, ui.SimpleStringModel(viewport.name))

    def _update_camera_stats_ui(self, e: carb.events.IEvent = None) -> None:
        """Updates the camera statistics UI with current transform data from the selected camera.

        Args:
            e: Event object from the global update event dispatcher.
        """
        # if camera prim path has been updated, set self._selected_camera to None
        if stage_utils.get_current_stage(backend="usd") is None:
            return

        if self._selected_camera and not prim_utils.get_prim_at_path(self._selected_camera.paths[0]).IsValid():
            self._selected_camera = None
            self._on_refresh()

        # Update the camera translation and rotation
        if self._selected_camera is not None:
            world_pos, world_quat = self._get_camera_pose(self._selected_camera, "world", self._selected_axis_world)
            local_pos, local_quat = self._get_camera_pose(self._selected_camera, "local", self._selected_axis_local)

            for i in range(3):
                self._task_ui_elements["World Camera Position"][i].set_value(float(world_pos[i]))
                self._task_ui_elements["Local Camera Position"][i].set_value(float(local_pos[i]))

            for i in range(4):
                self._task_ui_elements["World Camera Orientation"][i].set_value(float(world_quat[i]))
                self._task_ui_elements["Local Camera Orientation"][i].set_value(float(local_quat[i]))

            status = f"# World Axis: {self._selected_axis_world}\nworld_position={world_pos.tolist()}\nworld_quat_wxyz={world_quat.tolist()}\n# Local Axis: {self._selected_axis_local}\nlocal_position={local_pos.tolist()}\nlocal_quat_wxyz={local_quat.tolist()}"
            self._task_ui_elements["CameraTextField"].set_text(status)
        else:
            for i in range(3):
                self._task_ui_elements["World Camera Position"][i].set_value(float(0.0))
                self._task_ui_elements["Local Camera Position"][i].set_value(float(0.0))
            for i in range(4):
                self._task_ui_elements["World Camera Orientation"][i].set_value(float(0.0))
                self._task_ui_elements["Local Camera Orientation"][i].set_value(float(0.0))

            self._task_ui_elements["CameraTextField"].set_text(
                "# No camera selected. Please use dropdown to select a camera."
            )

    def _get_all_camera_objects(self, root_prim: str = "/") -> list[Camera]:
        """Retrieve Camera objects for each camera in the scene.

        Args:
            root_prim: Root prim where the world exists.

        Returns:
            A list of Camera objects
        """
        predicate = lambda prim, _: prim.GetPrimTypeInfo().GetTypeName() == "Camera"
        camera_prims = prim_utils.get_all_matching_child_prims(root_prim, predicate=predicate)
        return [Camera(prim_utils.get_prim_path(prim)) for prim in camera_prims]

    def _get_camera_pose(
        self, camera: Camera, frame: str = "world", camera_axes: str = "usd"
    ) -> tuple[np.ndarray, np.ndarray]:
        """Return the camera pose with orientation expressed in the requested axes convention.

        Args:
            camera: Camera object whose pose should be queried.
            frame: Frame of the pose to query.
            camera_axes: Target convention.

        Returns:
            Tuple (position, orientation) in the requested camera axes convention.
        """
        positions, orientations = camera.get_local_poses() if frame == "local" else camera.get_world_poses()
        position = positions.numpy()[0].astype(np.float32)
        orientation = orientations.numpy()[0].astype(np.float32)
        if camera_axes == "world":
            R = transform_utils.quaternion_to_rotation_matrix(orientation).numpy() @ _USD_TO_WORLD_ROT
            orientation = transform_utils.rotation_matrix_to_quaternion(R).numpy()
        elif camera_axes == "ros":
            R = transform_utils.quaternion_to_rotation_matrix(orientation).numpy() @ _USD_TO_ROS_ROT
            orientation = transform_utils.rotation_matrix_to_quaternion(R).numpy()
        return position, orientation
