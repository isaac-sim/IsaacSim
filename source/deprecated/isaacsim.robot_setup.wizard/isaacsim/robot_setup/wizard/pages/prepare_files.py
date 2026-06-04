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

"""UI component for preparing robot configuration files in the Robot Setup Wizard."""

import os

import omni.ui as ui
import omni.usd
from omni.usd import Sdf

from ..builders.robot_templates import RobotRegistry
from ..style import LABEL_COLOR, LABEL_ERROR_COLOR, LABEL_WARNING_COLOR
from ..utils.ui_utils import ButtonWithIcon, custom_header, next_step, open_folder_picker, separator
from ..utils.utils import can_create_dir


class PrepareFiles:
    """A UI component for preparing robot configuration files in the Robot Setup Wizard.

    This class provides an interface for users to configure file paths and settings before creating robot
    configuration files. It handles the selection of robot names, root folders, stage saving options, and
    file allocation for the robot setup process.

    The component displays several key sections:
    - Robot name and root folder selection with folder picker integration
    - Current stage saving options with overwrite and copy functionality
    - Original robot file information when available
    - Preview of allocated robot files with color-coded status indicators

    File status indicators use color coding where purple indicates existing files that will be overwritten,
    red indicates files without write permission, and default color indicates new files that can be created.

    The component integrates with the RobotRegistry to store configuration parameters and validates robot
    names using USD path identifier rules. It also checks stage permissions and unsaved changes to determine
    which saving options to present to the user.

    Args:
        visible: Whether the UI component should be visible when created.
        *args: Variable length argument list passed to parent initialization.
        **kwargs: Additional keyword arguments passed to parent initialization.
    """

    def __init__(self, visible: bool, *args: object, **kwargs: object) -> None:
        self.visible = visible

        self._next_step_button = None
        self._additional_info_frame = None
        self._save_current_stage_frame = None
        self._robot_file_frame = None
        self.frame = ui.Frame(visible=visible)
        self.frame.set_build_fn(self._build_frame)

        self._robot_name_widget = None
        self._robot_folder_widget = None
        self._current_stage_path_widget = None
        self._original_robot_path_widget = None
        self._save_copy_in_robot_root_folder_path_widget = None
        self._save_current_stage_frame = None
        self._overwrite_stage_frame = None
        self._save_copy_in_robot_root_folder_path_frame = None

        self._current_stage_is_unsaved = False

        ## param defaults
        self._reset_params()

    def destroy(self) -> None:
        """Clean up resources and reset the PrepareFiles instance.

        Destroys all UI frames, resets parameters, and clears the robot registry.
        """
        self._reset_params()
        if self._additional_info_frame:
            self._additional_info_frame.destroy()
        if self._save_current_stage_frame:
            self._save_current_stage_frame.destroy()
        if self._robot_file_frame:
            self._robot_file_frame.destroy()
        if self._overwrite_stage_frame:
            self._overwrite_stage_frame.destroy()
        if self._save_copy_in_robot_root_folder_path_frame:
            self._save_copy_in_robot_root_folder_path_frame.destroy()
        self.frame.destroy()
        RobotRegistry().reset()

    def _reset_params(self) -> None:
        """Reset all internal parameters to their default values.

        Sets robot files path, root folder, current stage path, robot name, and robot instance to default states.
        """
        self._original_robot_files_path = "None"
        self._robot_root_folder = os.path.expanduser("~")
        self._current_stage_path = omni.usd.get_context().get_stage().GetRootLayer().realPath or os.path.expanduser(
            "~"
        )  # current stage path or users home directory
        self._robot_name = ""
        self._robot = None

    def __next_step(self, verfiy_fn: callable = None) -> None:
        """Create the next step button for navigating to Robot Hierarchy page.

        Args:
            verfiy_fn: Verification function to call before proceeding to the next step.

        Returns:
            The next step button widget.
        """
        with ui.VStack():
            separator("Next: Robot Hierarchy")
            ui.Spacer(height=16)
            button = ButtonWithIcon(
                "Robot Hierarchy",
                name="next",
                clicked_fn=lambda: next_step("Prepare Files", "Robot Hierarchy", verfiy_fn),
                height=44,
                enabled=True,
            )
        return button

    def _build_frame(self) -> None:
        """Build the main UI frame for the Prepare Files page.

        Creates all UI elements including robot name input, folder selection, stage saving options, and file allocation display.
        """
        with ui.CollapsableFrame("Prepare Files", build_header_fn=custom_header):
            with ui.ScrollingFrame():
                with ui.VStack(spacing=2, name="margin_vstack"):
                    # step 1: pick the folder to store the newly configuredrobot
                    with ui.VStack():
                        ui.Spacer(height=10)
                        separator("Select the Root Folder for Configuration Files")
                        ui.Spacer(height=10)
                        with ui.HStack(height=0):
                            ui.Label("Robot Name", width=0)
                            ui.Spacer(width=5)
                            self._robot_name_widget = ui.StringField(width=600)
                            self._robot_name_widget.model.set_value(self._robot_name)
                            self._robot_name_widget.model.add_value_changed_fn(
                                lambda x: self._build_robot_files_frame()
                            )
                        ui.Spacer(height=5)
                        with ui.HStack(height=0):
                            ui.Label("Root Folder", width=0)
                            ui.Spacer(width=5)
                            self._robot_folder_widget = ui.StringField(width=600)
                            self._robot_folder_widget.model.set_value(self._robot_root_folder)
                            self._robot_folder_widget.model.add_value_changed_fn(
                                lambda x: self._build_robot_files_frame()
                            )
                            ui.Image(
                                name="folder",
                                height=25,
                                width=25,
                                mouse_pressed_fn=lambda x, y, b, a: self.select_robot_folder(self._robot_folder_widget),
                            )
                            ui.Spacer(width=5)
                        self._invalid_robot_name_label = ui.Label(
                            "Invalid Robot Name", visible=False, style={"color": "red"}
                        )
                        ui.Spacer(height=10)

                    # step 2: actions for the current stage if the current stage is not saved
                    self._save_current_stage_frame = ui.Frame(visible=True, height=0)
                    with self._save_current_stage_frame:
                        with ui.VStack(height=0):
                            ui.Spacer(height=10)
                            separator("Save the Current Stage")
                            ui.Spacer(height=10)
                            with ui.VStack(height=0):
                                with ui.HStack(height=0):
                                    self._save_current_stage_widget_frame = ui.Frame(visible=False, width=600)
                                    with self._save_current_stage_widget_frame:
                                        with ui.HStack(height=0):
                                            ui.Label("Overwrite Current Stage File", width=0)
                                            ui.Spacer(width=5)
                                            self._overwrite_current_stage_widget = ui.CheckBox(
                                                label="Overwrite Current Stage File", width=32
                                            )
                                            self._overwrite_current_stage_widget.model.set_value(False)
                                            self._overwrite_current_stage_widget.model.add_value_changed_fn(
                                                self._overwrite_current_stage_widget_changed
                                            )
                                    self._overwrite_stage_frame = ui.Frame(visible=False, width=600)
                                    with self._overwrite_stage_frame:
                                        with ui.HStack(height=0):
                                            self._current_stage_path_widget = ui.StringField(width=600)
                                            self._current_stage_path_widget.model.set_value(self._current_stage_path)
                                            self._current_stage_path_widget.model.add_value_changed_fn(
                                                lambda x: self._build_robot_files_frame()
                                            )
                                            ui.Image(
                                                name="folder",
                                                height=25,
                                                width=25,
                                                mouse_pressed_fn=lambda x, y, b, a: self.select_robot_folder(
                                                    self._current_stage_path_widget
                                                ),
                                            )
                                ui.Spacer(height=10)
                                with ui.HStack(height=0):
                                    ui.Label("Save A Copy In Robot Root Folder", width=0)
                                    ui.Spacer(width=5)
                                    self._save_copy_in_robot_root_folder_widget = ui.CheckBox(
                                        label="Save A Copy In Robot Root Folder", width=32
                                    )
                                    self._save_copy_in_robot_root_folder_widget.model.set_value(False)
                                    self._save_copy_in_robot_root_folder_widget.model.add_value_changed_fn(
                                        self._save_copy_in_robot_root_folder_widget_changed
                                    )
                                    self._save_copy_in_robot_root_folder_path_frame = ui.Frame(visible=False)
                                    with self._save_copy_in_robot_root_folder_path_frame:
                                        with ui.HStack(height=0):
                                            self._save_copy_in_robot_root_folder_path_widget = ui.StringField(width=600)
                                            self._save_copy_in_robot_root_folder_path_widget.model.set_value(
                                                self._robot_root_folder
                                            )
                                            self._save_copy_in_robot_root_folder_path_widget.model.add_value_changed_fn(
                                                lambda x: self._build_robot_files_frame()
                                            )

                    ui.Spacer(height=10)

                    # step 3: inform users about the original robot files if they exist
                    self._additional_info_frame = ui.Frame(visible=True, height=0)
                    with self._additional_info_frame:
                        with ui.VStack(height=0):
                            ui.Spacer(height=10)
                            separator("Additional Information")
                            ui.Spacer(height=10)
                            with ui.HStack(height=0):
                                ui.Label("Original Robot Files Traced Back To:", width=0)
                                ui.Spacer(width=5)
                                self._original_robot_path_widget = ui.StringField(width=600)
                                self._original_robot_path_widget.model.set_value(self._original_robot_files_path)
                                self._original_robot_path_widget.read_only = True
                            ui.Spacer(height=20)

                    # step 4: inform users about the robot files that will be saved
                    self._robot_files_frame = ui.Frame(visible=True, height=0)

                    robot_file_msg = "Main Robot File"
                    base_file_msg = "Base File"
                    physics_file_msg = "Physics File"
                    robot_schema_file_msg = "Robot Schema File"
                    # current_stage_msg = f"Current Stage Saved At:"

                    with self._robot_files_frame:
                        with ui.VStack(height=0):
                            ui.Spacer(height=10)
                            separator("Robot Files Allocated")
                            ui.Label(
                                "\t\t purple colored files are existing files that will be overwritten \n\t\t red colored files are files that do not have write permission",
                                width=600,
                                height=0,
                                name="sub_separator",
                                word_wrap=True,
                            )
                            ui.Spacer(height=10)
                            with ui.HStack(height=0):
                                ui.Spacer(width=15)
                                ui.Label(robot_file_msg, width=160)
                                ui.Spacer(width=5)
                                self._robot_file_path_label = ui.Label("", width=0)
                            ui.Spacer(height=5)
                            with ui.HStack(height=0):
                                ui.Spacer(width=15)
                                ui.Label(base_file_msg, width=160)
                                ui.Spacer(width=5)
                                self._base_file_path_label = ui.Label("", width=0)
                            ui.Spacer(height=5)
                            with ui.HStack(height=0):
                                ui.Spacer(width=15)
                                ui.Label(physics_file_msg, width=160)
                                ui.Spacer(width=5)
                                self._physics_file_path_label = ui.Label("", width=0)
                            ui.Spacer(height=5)
                            with ui.HStack(height=0):
                                ui.Spacer(width=15)
                                ui.Label(robot_schema_file_msg, width=160)
                                ui.Spacer(width=5)
                                self._robot_schema_file_path_label = ui.Label("", width=0)
                            # ui.Spacer(height=5)
                            # with ui.HStack(height=0):
                            #     ui.Spacer(width=15)
                            #     ui.Label(current_stage_msg, width=300)
                            #     ui.Spacer(width=5)
                            #     self._current_stage_path_label = ui.Label("", width=0)
                            ui.Spacer(height=20)

                    self._next_step_button = self.__next_step(self._prepare_files)
                    self._build_robot_files_frame()

    def set_visible(self, visible: bool) -> None:
        """Set the visibility of the PrepareFiles frame.

        Args:
            visible: Whether the frame should be visible.
        """
        if self.frame:
            self.frame.visible = visible

            if visible:
                self._preprocess_page()
                self._update_widgets()

    def _overwrite_current_stage_widget_changed(self, value: object) -> None:
        """Handle changes to the overwrite current stage checkbox.

        Args:
            value: The new checkbox value.
        """
        if value:
            self._overwrite_stage_frame.visible = True
        else:
            self._overwrite_stage_frame.visible = False

    def _save_copy_in_robot_root_folder_widget_changed(self, value: object) -> None:
        """Handle changes to the save copy in robot root folder checkbox.

        Args:
            value: The new checkbox value.
        """
        if value:
            self._save_copy_in_robot_root_folder_path_frame.visible = True
        else:
            self._save_copy_in_robot_root_folder_path_frame.visible = False

    def _build_robot_files_frame(self) -> None:
        """Update the robot files frame with current file paths and validation status.

        Validates robot name, constructs file paths, and updates UI labels with appropriate colors based on file existence and write permissions.
        """
        robot_name = self._robot_name_widget.model.get_value_as_string()
        if not Sdf.Path.IsValidIdentifier(robot_name):
            self._invalid_robot_name_label.visible = True
        else:
            self._invalid_robot_name_label.visible = False
        robot_root_folder = self._robot_folder_widget.model.get_value_as_string()
        current_stage_path = self._current_stage_path_widget.model.get_value_as_string()
        base_file_path = os.path.join(robot_root_folder, "configurations", robot_name + "_base.usd")
        physics_file_path = os.path.join(robot_root_folder, "configurations", robot_name + "_physics.usd")
        robot_schema_file_path = os.path.join(robot_root_folder, "configurations", robot_name + "_robot.usd")
        robot_file_path = os.path.join(robot_root_folder, robot_name + ".usd")
        stage_copy_path = os.path.join(robot_root_folder, "stage_copy.usd")

        self._save_copy_in_robot_root_folder_path_widget.model.set_value(stage_copy_path)

        self._base_file_path_label.text = base_file_path
        self._physics_file_path_label.text = physics_file_path
        self._robot_schema_file_path_label.text = robot_schema_file_path
        self._robot_file_path_label.text = robot_file_path
        # self._current_stage_path_label.text = current_stage_path

        if os.path.exists(base_file_path):
            self._base_file_path_label.style = {"color": LABEL_WARNING_COLOR}
        elif not can_create_dir(os.path.dirname(base_file_path)):
            self._base_file_path_label.style = {"color": LABEL_ERROR_COLOR}
        else:
            self._base_file_path_label.style = {"color": LABEL_COLOR}

        if os.path.exists(physics_file_path):
            self._physics_file_path_label.style = {"color": LABEL_WARNING_COLOR}
        elif not can_create_dir(os.path.dirname(physics_file_path)):
            self._physics_file_path_label.style = {"color": LABEL_ERROR_COLOR}
        else:
            self._physics_file_path_label.style = {"color": LABEL_COLOR}

        if os.path.exists(robot_schema_file_path):
            self._robot_schema_file_path_label.style = {"color": LABEL_WARNING_COLOR}
        elif not can_create_dir(os.path.dirname(robot_schema_file_path)):
            self._robot_schema_file_path_label.style = {"color": LABEL_ERROR_COLOR}
        else:
            self._robot_schema_file_path_label.style = {"color": LABEL_COLOR}

        if os.path.exists(robot_file_path):
            self._robot_file_path_label.style = {"color": LABEL_WARNING_COLOR}
        elif not can_create_dir(os.path.dirname(robot_file_path)):
            self._robot_file_path_label.style = {"color": LABEL_ERROR_COLOR}
        else:
            self._robot_file_path_label.style = {"color": LABEL_COLOR}

        # if os.path.exists(current_stage_path):
        #     self._current_stage_path_label.style = {"color": LABEL_WARNING_COLOR}
        # elif not can_create_dir(os.path.dirname(current_stage_path)):
        #     self._current_stage_path_label.style = {"color": LABEL_ERROR_COLOR}
        # else:
        #     self._current_stage_path_label.style = {"color": LABEL_COLOR}

        can_write_to_stage = False
        stage = omni.usd.get_context().get_stage()
        if stage:
            root_layer = stage.GetRootLayer()
            can_write_to_stage = root_layer.permissionToEdit and root_layer.permissionToSave

        if self._current_stage_is_unsaved and can_write_to_stage:
            self._save_current_stage_widget_frame.visible = True
        else:
            self._save_current_stage_widget_frame.visible = False

    def select_robot_folder(self, widget: object) -> None:
        """Open a folder picker dialog to select a robot folder.

        Args:
            widget: The string field widget to update with the selected folder path.
        """
        open_folder_picker(lambda filename, path: widget.model.set_value(path))

    def _preprocess_page(self) -> None:
        """Processes to run every time the page is shown.

        Gets data from the registry and updates the widgets with current robot information, stage path, and original robot file path.

        Returns:
            None if the stage or robot is not available.
        """
        stage = omni.usd.get_context().get_stage()
        self._robot = RobotRegistry().get()

        if not stage or not self._robot:
            return

        # get updates to the four variables modifiable on this page
        # 1) robot name
        self._robot_name = self._robot.name

        # 2) robot root folder
        # if the currently registered robot already has these parameters, use them
        registered_root_folder = self._robot.robot_root_folder
        if registered_root_folder:
            self._robot_root_folder = registered_root_folder
        else:
            self._robot_root_folder = os.path.expanduser("~")

        # 3) current stage path
        self._current_stage_path = stage.GetRootLayer().realPath or os.path.expanduser("~")

        # 4) original robot path
        # if the robot doesn't have a registered original robot path,
        registered_original_robot_path = self._robot.original_robot_path
        if not registered_original_robot_path:
            # check if the robot on stage is a reference or payload, get the full path of the reference
            robot_prim_on_stage = stage.GetPrimAtPath(self._robot.parent_prim_path)
            ref_and_layers = omni.usd.get_composed_references_from_prim(robot_prim_on_stage)
            payload_and_layers = omni.usd.get_composed_payloads_from_prim(robot_prim_on_stage)
            if ref_and_layers:
                robot_direct_path = ref_and_layers[0][0].assetPath
            elif payload_and_layers:
                robot_direct_path = payload_and_layers[0][0].assetPath
            else:
                robot_direct_path = None

            if robot_direct_path:
                self._original_robot_files_path = robot_direct_path
                if self._additional_info_frame:
                    self._additional_info_frame.visible = True
            else:
                self._original_robot_files_path = None
                if self._additional_info_frame:
                    self._additional_info_frame.visible = False

        is_unsaved = omni.usd.get_context().has_pending_edit()  # has unsaved changes
        is_new_stage = omni.usd.get_context().is_new_stage()  # new stage, has no file associated with it
        self._current_stage_is_unsaved = is_unsaved or is_new_stage

    def _update_widgets(self) -> None:
        """Updates all widget values from the current instance variables.

        Synchronizes the UI widgets with the current state by setting widget model values from instance variables
        like robot name, robot folder, current stage path, original robot path, and save copy path.
        """
        if self._robot_name_widget:
            self._robot_name_widget.model.set_value(self._robot_name)
        if self._robot_folder_widget:
            self._robot_folder_widget.model.set_value(self._robot_root_folder)
        if self._current_stage_path_widget:
            self._current_stage_path_widget.model.set_value(self._current_stage_path)
        if self._original_robot_path_widget:
            self._original_robot_path_widget.model.set_value(self._original_robot_files_path)
        if self._save_copy_in_robot_root_folder_path_widget:
            self._save_copy_in_robot_root_folder_path_widget.model.set_value(self._robot_root_folder)

    def _prepare_files(self) -> bool:
        """Prepares and validates robot configuration files based on current UI settings.

        Collects values from UI widgets and registers them with the RobotRegistry, including robot name,
        root folder, file paths for base/physics/robot schema files, and stage save options.

        Returns:
            True if robot preparation was successful, False if no robot is registered.
        """
        # register all the params collected in the current page to the registry
        self._robot = RobotRegistry().get()
        if not self._robot:
            return False

        self._robot.name = self._robot_name_widget.model.get_value_as_string()
        robot_root_folder = self._robot_folder_widget.model.get_value_as_string()
        self._robot.robot_root_folder = robot_root_folder
        self._robot.original_stage_path = self._current_stage_path_widget.model.get_value_as_string()
        # if the original robot path is not a reference or payload already, save the current stage as the original file
        original_robot_path = self._original_robot_path_widget.model.get_value_as_string()
        self._robot.original_robot_path = original_robot_path

        # save the base, physics, and robot files
        self._robot.base_file_path = os.path.join(robot_root_folder, "configurations", self._robot.name + "_base.usd")
        self._robot.physics_file_path = os.path.join(
            robot_root_folder, "configurations", self._robot.name + "_physics.usd"
        )
        self._robot.robot_schema_file_path = os.path.join(
            robot_root_folder, "configurations", self._robot.name + "_robot.usd"
        )
        # save the main robot file
        self._robot.robot_file_path = os.path.join(robot_root_folder, self._robot.name + ".usd")

        # save the stage as it is
        self._robot.save_stage_original = self._overwrite_current_stage_widget.model.get_value_as_bool()
        # save the a copy of the stage in the robot root folder
        self._robot.save_stage_copy = self._save_copy_in_robot_root_folder_widget.model.get_value_as_bool()

        RobotRegistry().update(self._robot)

        return True
