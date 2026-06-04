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

"""Page for adding and configuring robots in the Isaac Sim robot setup wizard."""

import os
from functools import partial
from typing import Any

import omni.kit.app
import omni.ui as ui
from omni.usd import Sdf

from ..builders.robot_templates import *
from ..utils.robot_asset_picker import RobotAssetPicker
from ..utils.ui_utils import (
    ButtonWithIcon,
    FileSorter,
    FilteredFileDialog,
    create_combo_list_model,
    custom_header,
    next_step,
    open_extension,
    separator,
    text_with_dot,
)
from ..utils.utils import get_stage_default_prim_path


class AddRobot:
    """A wizard interface for adding and configuring robots in Isaac Sim.

    Provides a multi-step UI workflow for users to select, configure, and prepare robots for simulation. Supports
    multiple robot acquisition methods including configuring existing robots on stage, converting URDF/MJCF files,
    and using external importers. The wizard guides users through robot type selection, naming, parent transform
    selection, and preparation for the next workflow steps.

    The interface includes drag-and-drop functionality for robot files, asset picking for parent transforms, and
    validation of user inputs. It integrates with the robot registry system to maintain robot configuration state
    across workflow steps.

    Args:
        visible: Whether the wizard interface is initially visible.
        *args: Additional positional arguments.
        **kwargs: Additional keyword arguments including window_title for customizing the wizard window title.
    """

    def __init__(self, visible: bool, *args: object, **kwargs: object) -> None:
        self.visible = visible
        self.window_title = kwargs.pop("window_title", "Robot Wizard")
        self._select_robot_asset_window = None
        self._add_robot_frames = []
        # indicate which frame in self._add_robot_frames is visible
        self._prev_frame_index = 0
        self.external_drag_drops = {}
        self.frame = ui.Frame(visible=visible)
        self.frame.set_build_fn(self._build_frame)
        self._select_parent_xform_window = None

        self._reset_params()

    def destroy(self) -> None:
        """Destroys the AddRobot instance and cleans up all associated UI elements and resources."""
        if self._select_parent_xform_window:
            self._select_parent_xform_window.destroy()
        self._select_parent_xform_window = None
        for f in self._add_robot_frames:
            f.destroy()
        self._add_robot_frames = []
        self.destroy_external_drag_drop()
        RobotRegistry().reset()
        self.frame.destroy()

    def _reset_params(self) -> None:
        """Resets the robot configuration parameters to their default values."""
        self._robot_type = "Custom"
        self._robot_name = ""
        self._robot_parent_prim_path = ""

    def _build_frame(self) -> None:
        """Build the frame for the add robot page."""
        with ui.ScrollingFrame():
            with ui.VStack(spacing=10):
                with ui.CollapsableFrame("Add Robot", height=0, build_header_fn=custom_header):
                    with ui.VStack(spacing=10, name="margin_vstack"):
                        separator("1. Getting Started: Select a Robot to Configure or Inspect")
                        with ui.HStack(spacing=10, height=0):
                            ui.Image(name="add_robot", width=140)
                            ui.Spacer(width=16)
                            with ui.VStack():
                                ui.Label("Choose from these options", name="choose_options", height=15)
                                ui.Spacer(height=12)
                                options = [
                                    "Configure a Robot On Stage",
                                    # "Inspect an Issac Sim Robot",
                                    "Convert a Sim-Ready Robot (URDF/MJCF)",
                                ]
                                collection = ui.RadioCollection()
                                for i in range(len(options)):
                                    with ui.HStack(spacing=2, height=23):
                                        ui.RadioButton(radio_collection=collection, width=23)
                                        ui.Label(options[i], width=0)

                        ui.Spacer(height=10)

                        # step 2 and 3 in horizontal stack
                        with ui.HStack(spacing=10):
                            with ui.VStack(height=0):
                                separator("2. Robot Type")
                                ui.Spacer(height=6)
                                with ui.VStack():
                                    ui.Spacer()
                                    robot_type_list = [
                                        "Wheeled Robot",
                                        "Manipulator",
                                        "Gripper",
                                        "Quadruped",
                                        "Humanoid",
                                        "Custom",
                                    ]
                                    self._robot_type_model = create_combo_list_model(robot_type_list, 0)
                                    ui.ComboBox(self._robot_type_model, name="robot_type", height=22)
                                    ui.Spacer()
                            ui.Spacer(width=20)
                            with ui.VStack():
                                separator("3. Robot Name (Optional)")
                                ui.Spacer(height=6)
                                self._robot_name_widget = ui.StringField()
                                self._robot_name_widget.model.set_value(self._robot_name)
                                self._robot_name_widget.model.add_value_changed_fn(
                                    lambda m: self._on_robot_name_changed(m)
                                )
                                self._invalid_robot_name_label = ui.Label(
                                    "Invalid Robot Name", visible=False, style={"color": "red"}
                                )

                ## windows for add robot selections
                self._configure_robot_on_stage()
                # self._inspect_isaac_sim_robot()
                # self._convert_sim_ready_robot()           # TODO: integrate it into the wizard, for now just highlight the tool
                self._highlight_tool()
                # self._import_robot_mesh()
                # self._import_onshape_robot()

                def _update_add_robot_frame(m: Any) -> None:
                    checked = m.get_value_as_int()
                    self._add_robot_frames[self._prev_frame_index].visible = False
                    self._add_robot_frames[checked].visible = True
                    self._prev_frame_index = checked

                collection_model = collection.model
                collection_model.add_value_changed_fn(lambda m: _update_add_robot_frame(m))

                # display the first option to start
                self._add_robot_frames[0].visible = True
                self._prev_frame_index = 0

                def _update_robot_type(m: Any, n: Any) -> None:
                    self._robot_type = m.get_current_string()

                ## robot type selection
                self._robot_type_model.add_item_changed_fn(_update_robot_type)

    def _on_robot_name_changed(self, m: object) -> None:
        """Handles changes to the robot name input field.

        Args:
            m: The model containing the updated robot name value.
        """
        robot_name_value = m.get_value_as_string()
        if Sdf.Path.IsValidIdentifier(robot_name_value):
            self._robot_name = robot_name_value
            self._invalid_robot_name_label.visible = False
        else:
            self._invalid_robot_name_label.visible = True

    def set_visible(self, visible: bool) -> None:
        """When this Add Robot page is visible or hidden.

        Args:
            visible: Whether the page should be visible.
        """
        if self.frame:
            if visible:
                self._preprocess_params()
                self._update_widgets()
            self.frame.visible = visible

    def __next_step(self, verify_fn: callable = None) -> None:
        """Creates the next step button for the robot configuration wizard.

        Args:
            verify_fn: Optional verification function to call when proceeding to the next step.

        Returns:
            The ButtonWithIcon widget for the next step.
        """
        with ui.VStack():
            ui.Spacer(height=16)
            separator("Next: Prepare Files")
            ui.Spacer(height=16)
            button = ButtonWithIcon(
                "Prepare Files",
                name="next",
                clicked_fn=lambda: next_step("Add Robot", "Prepare Files", verify_fn),
                height=44,
                image_width=18,
                enabled=False,
            )
        return button

    def _preprocess_params(self) -> None:
        """Preprocesses parameters from the registered robot for display in the UI.

        Returns:
            None if no robot is registered.
        """
        self._robot = RobotRegistry().get()
        if not self._robot:
            return
        # get the variables that are modifiable on this page
        self._robot_name = self._robot.name
        self._robot_type = self._robot.robot_type
        self._robot_parent_prim_path = self._robot.parent_prim_path

    def _update_widgets(self) -> None:
        """Updates UI widgets with current robot parameter values."""
        if self._robot_name_widget:
            self._robot_name_widget.model.set_value(self._robot_name)
        if self._robot_type_model:
            self._robot_type_model.set_current_string(self._robot_type)
        if self._parent_xform_widget:
            self._parent_xform_widget.model.set_value(self._robot_parent_prim_path)

    def _add_robot(self) -> bool:
        """Before moving on, initialize (or overwrite)the robot with the given params.

        Returns:
            True if the robot was successfully added.
        """
        # get parameters from UI: robot name, type, and output folder, also parent prim path
        robot_name = self._robot_name_widget.model.get_value_as_string()
        robot_type = self._robot_type_model.get_current_string()
        if self._parent_xform_model:
            parent_prim_path = self._parent_xform_model.get_value_as_string()

        # create and registerthe robot based on type
        if robot_type == "Custom":
            current_robot = CustomRobot(name=robot_name)  # these calls also register the robot
        elif robot_type == "Wheeled Robot":
            current_robot = WheeledRobot(name=robot_name)
        elif robot_type == "Manipulator":
            current_robot = Manipulator(name=robot_name)
        elif robot_type == "Gripper":
            current_robot = Gripper(robot_name)
        elif robot_type == "Quadruped":
            current_robot = Quadruped(robot_name)
        elif robot_type == "Humanoid":
            current_robot = Humanoid(robot_name)
        else:
            raise ValueError(f"Invalid robot type: {robot_type}")

        ## register all the params to the RobotRegistry
        current_robot = RobotRegistry().get()
        current_robot.name = robot_name
        current_robot.robot_type = robot_type
        current_robot.parent_prim_path = parent_prim_path

        return True

    def select(self, selected_paths: list) -> None:
        """Handles selection of parent transform paths from the asset picker.

        Args:
            selected_paths: List of selected prim paths from the asset picker.
        """
        self._select_parent_xform_window.visible = False
        self._selected_paths = selected_paths
        if self._selected_paths:
            self._parent_xform_widget.model.set_value(self._selected_paths[0])

    def select_parent_xform(self, model: Any, title: Any) -> None:
        """Open the asset picker window to select a parent transform prim.

        Args:
            model: The model associated with the path widget.
            title: Title for the asset picker window.
        """
        if not self._select_parent_xform_window:
            stage = omni.usd.get_context().get_stage()
            self._select_parent_xform_window = RobotAssetPicker(
                title,
                stage,
                on_targets_selected=partial(self.select),
            )
        self._select_parent_xform_window.set_on_selected(partial(self.select))
        self._select_parent_xform_window.visible = True

    def _on_parent_xform_changed(self, model: Any) -> None:
        # check if the parent xform is valid before enabling the next step button
        prim_path = model.get_value_as_string()
        stage = omni.usd.get_context().get_stage()
        if stage.GetPrimAtPath(prim_path):
            self._next_step_button.enabled = True
        else:
            self._next_step_button.enabled = False

        # also if the robot name is still the default, update it to the parent prim name
        if self._robot_name_widget.model.get_value_as_string() == "":
            self._robot_name_widget.model.set_value(prim_path.split("/")[-1])

    def _configure_robot_on_stage(self) -> None:
        # configure rest of the page
        _configure_robot_on_stage_frame = ui.CollapsableFrame(
            "Configure a Robot On Stage", build_header_fn=custom_header, visible=False
        )
        self._add_robot_frames.append(_configure_robot_on_stage_frame)
        with _configure_robot_on_stage_frame:
            with ui.VStack(spacing=10, name="margin_vstack"):
                # if the robot is on stage already, pick the parent prim path of the robot
                self._parent_xform_stack = ui.HStack(height=18, width=486)
                with self._parent_xform_stack:
                    ui.Label("Select Robot Parent Xform", width=0)
                    ui.Spacer(width=5)
                    self._parent_xform_widget = ui.StringField()
                    self._parent_xform_model = self._parent_xform_widget.model
                    parent_value = get_stage_default_prim_path(omni.usd.get_context().get_stage()).pathString
                    self._parent_xform_model.set_value(parent_value)
                    ui.Image(
                        name="sample",
                        height=25,
                        width=25,
                        mouse_pressed_fn=lambda x, y, b, a: self.select_parent_xform(
                            self._parent_xform_model, "Select Parent Xform"
                        ),
                    )
                    ui.Spacer(width=5)
                self._parent_xform_model.add_value_changed_fn(self._on_parent_xform_changed)
                self._next_step_button = self.__next_step(verify_fn=self._add_robot)
                if parent_value:
                    self._robot_name_widget.model.set_value(parent_value.split("/")[-1])
                    self._next_step_button.enabled = True
                else:
                    self._next_step_button.enabled = False

    def __open_sim_ready_robot(self, frame: Any, title: Any) -> None:
        def _on_accept_drop(url: Any) -> None:
            if FileSorter.is_sim_ready(url):
                return True
            else:
                return False

        def _on_filename_value(field: Any) -> None:
            new_str = field.model.get_value_as_string()
            if new_str:
                field.checked = True
                if FileSorter.is_sim_ready(new_str):
                    parse_robot_button.enabled = True
                else:
                    parse_robot_button.enabled = False
                next_step_button.enabled = False

        def _open_file_picker(x: Any, y: Any, b: Any, a: Any) -> None:
            def _on_select_file_from_folder(filename: Any, filepath: Any) -> None:
                path_widget.model.set_value(os.path.join(filepath, filename))
                parse_robot_button.enabled = True
                next_step_button.enabled = False
                file_selector.close_dialog()

            accepted_formats = [".urdf", ".xml", "mjcf"]
            file_selector = FilteredFileDialog(accepted_formats, _on_select_file_from_folder)
            file_selector.open_dialog()

        with frame:
            with ui.VStack(spacing=5, name="margin_vstack"):
                separator(title)
                text_with_dot("Drag and drop from a file from a folder on your desktop or content browser")
                text_with_dot("Paste the file path into the text field")
                text_with_dot("Click on the folder icon to open a file browser")
                ui.Spacer(height=10)

                with ui.HStack():
                    ui.Spacer()
                    with ui.ZStack(width=488):
                        ui.Rectangle(name="d_area", height=202)
                        with ui.VStack(height=0):
                            ui.Spacer(height=1)
                            with ui.HStack():
                                ui.Spacer(width=1)
                                with ui.ZStack(height=171, width=486):
                                    drop_area = ui.Rectangle(
                                        name="drag_area",
                                        accept_drop_fn=lambda url: _on_accept_drop(url),
                                        mouse_hovered_fn=lambda hovered: _mouse_hovered_fn(hovered),
                                    )
                                    ui.Image(name="drag_and_drop", height=171, width=486)
                            ui.Spacer(height=4)
                            with ui.HStack(height=23):
                                ui.Spacer(width=3)
                                path_widget = ui.StringField()
                                path_widget.model.add_value_changed_fn(lambda _: _on_filename_value(path_widget))
                                ui.Spacer(width=2)
                                ui.Image(
                                    name="folder",
                                    width=22,
                                    mouse_pressed_fn=_open_file_picker,
                                )
                                ui.Spacer(width=2)
                    ui.Spacer()
                ui.Spacer(height=4)
                parse_robot_button = ButtonWithIcon("Import Robot", name="parse_robot", height=44, enabled=False)
                ui.Spacer(height=5)
                next_step_button = self.__next_step(verify_fn=self._add_robot)

                def _on_drop(e: ui.WidgetMouseDropEvent) -> None:
                    path = e.mime_data
                    path_widget.model.set_value(path)
                    parse_robot_button.enabled = True

                def _open_importer() -> None:
                    url = path_widget.model.get_value_as_string()
                    if url.endswith(".urdf"):
                        open_extension("omni.importer.urdf", "open_urdf_importer")
                    elif url.endswith(".xml"):
                        open_extension("omni.importer.mjcf", "open_mjcf_importer")
                    else:
                        return False
                    next_step_button.enabled = True

                # this is to make sure that the drop area is hovered
                def _mouse_hovered_fn(hovered: Any) -> None:
                    drop_area.checked = hovered

                drop_area.set_drop_fn(_on_drop)  # internal drop
                self.setup_external_drag_drop(
                    self.window_title, title, drop_area, path_widget, parse_robot_button
                )  # external drop

                parse_robot_button.set_clicked_fn(_open_importer)

    def __open_robot(self, frame: Any, title: Any) -> None:
        def _on_accept_drop(url: Any) -> None:
            if FileSorter.is_3d_model(url):
                return True
            else:
                return False

        def _on_filename_value(field: Any) -> None:
            new_str = field.model.get_value_as_string()
            if new_str:
                field.checked = True
                if FileSorter.is_3d_model(new_str):
                    next_step_button.enabled = True
                else:
                    next_step_button.enabled = False

        def _verify_asset_fn() -> None:
            # load the robot on stage first, and then run parse_robot()
            url = path_widget.model.get_value_as_string()
            omni.kit.actions.core.execute_action("omni.kit.window.file", "open_stage", url)

            # find the path that the open stage made for the robot
            stage = omni.usd.get_context().get_stage()
            robot_path_onstage = get_stage_default_prim_path(stage).pathString

            # params that needs to get updated
            robot_name = self._robot_name_widget.model.get_value_as_string()
            robot_type = self._robot_type_model.get_current_string()
            # robot_path_onstage = something else!!
            # register this robot

            # RobotParams(robot_name, robot_type)

            # set the current robot's params
            current_robot = RobotRegistry.get(robot_name)
            current_robot.parent_prim_path = robot_path_onstage

        def _open_file_picker(x: Any, y: Any, b: Any, a: Any) -> None:
            def _on_select_file_from_folder(filename: Any, filepath: Any) -> None:
                path_widget.model.set_value(os.path.join(filepath, filename))
                next_step_button.enabled = True
                file_selector.close_dialog()

            accepted_formats = [".usd", ".stl", ".obj", ".dae", ".fbx", ".usda"]
            file_selector = FilteredFileDialog(accepted_formats, _on_select_file_from_folder)
            file_selector.open_dialog()

        with frame:
            with ui.VStack(spacing=5, name="margin_vstack"):
                separator(title)
                text_with_dot("Drag and drop from a file from a folder on your desktop or content browser")
                text_with_dot("Paste the file path into the text field")
                text_with_dot("Click on the folder icon to open a file browser")
                ui.Spacer(height=10)

                with ui.HStack():
                    ui.Spacer()
                    with ui.ZStack(width=488):
                        ui.Rectangle(name="d_area", height=202)
                        with ui.VStack(height=0):
                            ui.Spacer(height=1)
                            with ui.HStack():
                                ui.Spacer(width=1)
                                with ui.ZStack(height=171, width=486):
                                    drop_area = ui.Rectangle(
                                        name="drag_area",
                                        accept_drop_fn=lambda url: _on_accept_drop(url),
                                        mouse_hovered_fn=lambda hovered: _mouse_hovered_fn(hovered),
                                    )
                                    ui.Image(name="drag_and_drop", height=171, width=486)
                            ui.Spacer(height=4)
                            with ui.HStack(height=23):
                                ui.Spacer(width=3)
                                path_widget = ui.StringField()
                                path_widget.model.add_value_changed_fn(lambda _: _on_filename_value(path_widget))
                                ui.Spacer(width=2)
                                ui.Image(
                                    name="folder",
                                    width=22,
                                    mouse_pressed_fn=_open_file_picker,
                                )
                                ui.Spacer(width=2)
                    ui.Spacer()
                ui.Spacer(height=5)
                next_step_button = self.__next_step(verify_fn=_verify_asset_fn)

                def _on_drop(e: ui.WidgetMouseDropEvent) -> None:
                    path = e.mime_data
                    path_widget.model.set_value(path)

                # this is to make sure that the drop area is hovered
                def _mouse_hovered_fn(hovered: Any) -> None:
                    drop_area.checked = hovered

                drop_area.set_drop_fn(_on_drop)  # internal drop
                self.setup_external_drag_drop(
                    self.window_title, title, drop_area, path_widget, next_step_button
                )  # external drop

    def _inspect_isaac_sim_robot(self) -> None:
        _inspect_isaac_robot_frame = ui.CollapsableFrame(
            "Inspect an Isaac Sim Robot", build_header_fn=custom_header, visible=False
        )
        self._add_robot_frames.append(_inspect_isaac_robot_frame)
        with _inspect_isaac_robot_frame:
            with ui.ScrollingFrame():
                with ui.VStack(spacing=5, name="margin_vstack"):
                    separator("Inspect Isaac Sim Robot")
                    open_button = ButtonWithIcon(
                        "Open Isaac Sim Asset Explorer", name="browse", height=44, image_width=18
                    )
                    ui.Spacer(height=5)
                    next_step_button = self.__next_step()

                    def isaac_robot_explorer() -> None:
                        print("opening up isaac sim robot asset browser")
                        next_step_button.enabled = True

                    open_button.set_clicked_fn(isaac_robot_explorer)

    def _highlight_tool(self) -> None:
        """Highlight importer tools external to the wizard."""
        _highlight_tool_frame = ui.CollapsableFrame("Use a Importer", build_header_fn=custom_header, visible=False)
        self._add_robot_frames.append(_highlight_tool_frame)
        with _highlight_tool_frame:
            with ui.ScrollingFrame():
                with ui.VStack(spacing=5, name="margin_vstack"):
                    ui.Spacer(height=5)
                    msg = "Convert the robot from URDF/MJCF/CAD to USD first by going to File > Import. You can examine the converted robot by loading the converted usd file onto stage and inspecting the robot."
                    ui.Label(msg, name="highlight_tool", height=0, word_wrap=True)

    def _convert_sim_ready_robot(self) -> None:
        _import_sim_ready_robot_frame = ui.CollapsableFrame(
            "Import a Sim-Ready Robot", build_header_fn=custom_header, visible=False
        )
        self._add_robot_frames.append(_import_sim_ready_robot_frame)
        self.__open_sim_ready_robot(_import_sim_ready_robot_frame, "Convert Sim-Ready Robot")

    # def _import_robot_mesh(self):
    #     _import_robot_mesh_frame = ui.CollapsableFrame(
    #         "Import a Robot Mesh", build_header_fn=custom_header, visible=False
    #     )
    #     self._add_robot_frames.append(_import_robot_mesh_frame)
    #     self.__open_robot(_import_robot_mesh_frame, "Open Robot Mesh")

    # def _import_onshape_robot(self):
    #     _import_onshape_robot_frame = ui.CollapsableFrame(
    #         "Import a Onshape Robot", build_header_fn=custom_header, visible=False
    #     )
    #     self._add_robot_frames.append(_import_onshape_robot_frame)
    #     with _import_onshape_robot_frame:
    #         with ui.VStack(spacing=5, name="margin_vstack"):
    #             separator("Open Onshape Importer")
    #             open_button = ButtonWithIcon("Open Onshape Explorer", name="browse", height=44, image_width=18)
    #             ui.Spacer(height=5)
    #             next_step_button = self.__next_step(verify_fn=self._add_robot)

    #             def onshape_explorer():
    #                 open_extension("omni.importer.onshape", "import_from_onshape")
    #                 next_step_button.enabled = True

    #             open_button.set_clicked_fn(onshape_explorer)

    def setup_external_drag_drop(
        self, window_name: str, title: Any, drop_area: Any, path_widget: Any, button: Any
    ) -> None:
        """Set up external drag and drop support for a drop area.

        Args:
            window_name: Name of the parent window.
            title: Identifier for this drag-drop area.
            drop_area: The UI drop area widget.
            path_widget: Widget to update with the dropped file path.
            button: Button to enable when a valid file is dropped.
        """
        if title in self.external_drag_drops:
            dd = self.external_drag_drops[title]
            dd.destroy()
            dd = None
            if dd in self.external_drag_drops:
                self.external_drag_drops.pop(dd)
        try:
            from omni.kit.window.drop_support import ExternalDragDrop

            self.external_drag_drops[title] = ExternalDragDrop(
                window_name=window_name,
                drag_drop_fn=lambda e, p, drop_area=drop_area, path_widget=path_widget, button=button: self._on_ext_drag_drop(
                    e, p, drop_area, title, path_widget, button
                ),
            )
        except ImportError:
            pass

    def destroy_external_drag_drop(self) -> None:
        """Destroy all external drag and drop handlers."""
        if self.external_drag_drops:
            for _, d in self.external_drag_drops.items():
                d.destroy()
                d = None
            self.external_drag_drops = {}

    def _on_ext_drag_drop(
        self, e: object, payload: list[str], drop_area: object, title: str, path_widget: object, button: object
    ) -> None:
        """Handles external drag and drop operations for robot file imports.

        Validates and processes dropped files based on the drop area title and file type compatibility.
        Updates the path widget and enables the associated button when a valid file is dropped.

        Args:
            e: The drag and drop event.
            payload: List of file paths being dropped.
            drop_area: The UI drop area widget.
            title: Title identifying the drop area type (e.g., "Sim-Ready", "Robot Mesh").
            path_widget: The UI string field widget to update with the file path.
            button: The UI button widget to enable when a valid file is dropped.

        Returns:
            None if the drop area is not hovered.
        """
        # if drop_area is not hovered, we dont want to do anything
        if not drop_area.checked:
            return

        path = payload[0]
        if "Sim-Ready" in title and FileSorter.is_sim_ready(path):
            path_widget.model.set_value(path)
            path_widget.checked = True
            button.enabled = True
        elif "Robot Mesh" in title and FileSorter.is_3d_model(path):
            path_widget.model.set_value(path)
            path_widget.checked = True
            button.enabled = True
        else:
            pass
