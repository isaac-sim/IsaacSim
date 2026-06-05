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

"""Extension for testing robot motion planning using Lula kinematics solvers and RmpFlow motion generation."""

from __future__ import annotations

import asyncio
import gc
import os
import weakref
from typing import Any

import carb
import carb.eventdispatcher
import omni
import omni.kit.commands
import omni.physics.core
import omni.timeline
import omni.ui as ui
import omni.usd
from isaacsim.core.prims import SingleArticulation
from isaacsim.core.utils.prims import get_prim_object_type
from isaacsim.gui.components.menu import make_menu_item_description
from isaacsim.gui.components.ui_utils import (
    add_line_rect_flourish,
    btn_builder,
    float_builder,
    get_style,
    setup_ui_headers,
    state_btn_builder,
    str_builder,
)
from isaacsim.gui.components.widgets import DynamicComboBoxModel
from omni.kit.menu.utils import MenuItemDescription, add_menu_items, remove_menu_items
from omni.kit.window.extensions import SimpleCheckBox
from omni.kit.window.property.templates import LABEL_WIDTH
from pxr import Usd

from .test_scenarios import LulaTestScenarios

EXTENSION_NAME = "Lula Test Widget"

MAX_DOF_NUM = 100


def is_yaml_file(path: str) -> bool:
    """Check if a file path has a YAML extension.

    Args:
        path: File path to check.

    Returns:
        True if the file has a .yaml or .YAML extension.
    """
    _, ext = os.path.splitext(path.lower())
    return ext in [".yaml", ".YAML"]


def is_urdf_file(path: str) -> bool:
    """Check if a file path has a URDF extension.

    Args:
        path: File path to check.

    Returns:
        True if the file has a .urdf or .URDF extension.
    """
    _, ext = os.path.splitext(path.lower())
    return ext in [".urdf", ".URDF"]


def on_filter_yaml_item(item: object) -> bool:
    """Filter function for YAML file browser items.

    Args:
        item: File browser item to filter.

    Returns:
        True if the item should be displayed in the YAML file browser.
    """
    if not item or item.is_folder:
        return not (item.name == "Omniverse" or item.path.startswith("omniverse:"))
    return is_yaml_file(item.path)


def on_filter_urdf_item(item: object) -> bool:
    """Filter function for URDF file browser items.

    Args:
        item: File browser item to filter.

    Returns:
        True if the item should be displayed in the URDF file browser.
    """
    if not item or item.is_folder:
        return not (item.name == "Omniverse" or item.path.startswith("omniverse:"))
    return is_urdf_file(item.path)


class Extension(omni.ext.IExt):
    """Extension class for the Lula Test Widget.

    This extension provides a comprehensive testing interface for robot motion planning using Lula kinematics
    solvers and RmpFlow motion generation. It enables users to select articulated robots from the scene,
    load robot configuration files, and test various motion planning scenarios including inverse kinematics,
    trajectory generation, and RmpFlow-based motion control.

    The extension creates a dockable UI window with panels for robot selection, kinematics testing, trajectory
    generation, and RmpFlow configuration. Users can visualize end-effector poses, follow targets, generate
    custom trajectories, and test sinusoidal motion patterns with configurable parameters.

    Key features include:
    - Interactive robot articulation selection from the current stage
    - Support for YAML robot description and URDF file loading
    - Inverse kinematics solver testing with target following
    - Custom trajectory generation and playback
    - RmpFlow motion planning with obstacle avoidance
    - Real-time end-effector visualization
    - Debugging mode for motion planning analysis
    """

    def on_startup(self, ext_id: str) -> None:
        """Initialize extension and UI elements.

        Args:
            ext_id: The extension identifier.
        """
        # Events
        self._usd_context = omni.usd.get_context()
        self._physics_simulation_interface = omni.physics.core.get_physics_simulation_interface()
        self._physics_subscription = None
        self._stage_event_sub = None
        self._timeline = omni.timeline.get_timeline_interface()

        # Build Window
        self._window = ui.Window(
            title=EXTENSION_NAME, width=600, height=500, visible=False, dockPreference=ui.DockPreference.LEFT_BOTTOM
        )
        self._window.set_visibility_changed_fn(self._on_window)

        # UI
        self._models = {}
        self._ext_id = ext_id
        menu_entry = [
            make_menu_item_description(ext_id, EXTENSION_NAME, lambda a=weakref.proxy(self): a._menu_callback())
        ]

        self._menu_items = [MenuItemDescription("Robotics", sub_menu=menu_entry)]

        add_menu_items(self._menu_items, "Tools")

        # Selection
        self._new_window = True
        self.new_selection = True
        self._selected_index = None
        self._selected_prim_path = None
        self._prev_art_prim_path = None

        # Articulation
        self.articulation = None
        self.num_dof = 0
        self.dof_names = []
        self.link_names = []

        # Lula Config Files
        self._selected_robot_description_file = None
        self._selected_robot_urdf_file = None
        self._robot_description_file = None
        self._robot_urdf_file = None
        self._ee_frame_options = []

        self._rmpflow_config_yaml = None

        # Lula Test Scenarios
        self._test_scenarios = LulaTestScenarios()

        # Visualize End Effector
        self._visualize_end_effector = True

    def on_shutdown(self) -> None:
        """Clean up resources when the extension is unloaded."""
        self._test_scenarios.full_reset()
        self.articulation = None
        self._usd_context = None
        self._stage_event_sub = None
        self._timeline_event_sub = None
        self._physics_subscription = None
        self._models = {}
        remove_menu_items(self._menu_items, "Tools")
        if self._window:
            self._window = None
        gc.collect()

    def _on_window(self, visible: bool) -> None:
        if self._window.visible:
            # Subscribe to Stage and Timeline Events
            self._usd_context = omni.usd.get_context()
            self._stage_event_sub_selection = carb.eventdispatcher.get_eventdispatcher().observe_event(
                event_name=self._usd_context.stage_event_name(omni.usd.StageEventType.SELECTION_CHANGED),
                on_event=self._on_stage_selection_changed,
                observer_name="isaacsim.robot_motion.lula_test_widget.Extension._on_stage_selection_changed",
            )
            self._stage_event_sub_opened = carb.eventdispatcher.get_eventdispatcher().observe_event(
                event_name=self._usd_context.stage_event_name(omni.usd.StageEventType.OPENED),
                on_event=self._on_stage_opened,
                observer_name="isaacsim.robot_motion.lula_test_widget.Extension._on_stage_opened",
            )
            self._stage_event_sub_closed = carb.eventdispatcher.get_eventdispatcher().observe_event(
                event_name=self._usd_context.stage_event_name(omni.usd.StageEventType.CLOSED),
                on_event=self._on_stage_closed,
                observer_name="isaacsim.robot_motion.lula_test_widget.Extension._on_stage_closed",
            )
            self._stage_event_sub_sim_play = carb.eventdispatcher.get_eventdispatcher().observe_event(
                event_name=self._usd_context.stage_event_name(omni.usd.StageEventType.SIMULATION_START_PLAY),
                on_event=self._on_timeline_play,
                observer_name="isaacsim.robot_motion.lula_test_widget.Extension._on_timeline_play",
            )
            self._stage_event_sub_sim_stop = carb.eventdispatcher.get_eventdispatcher().observe_event(
                event_name=self._usd_context.stage_event_name(omni.usd.StageEventType.SIMULATION_STOP_PLAY),
                on_event=self._on_timeline_stop,
                observer_name="isaacsim.robot_motion.lula_test_widget.Extension._on_timeline_stop",
            )

            self._build_ui()
            if not self._new_window and self.articulation:
                self._refresh_ui(self.articulation)
            self._new_window = False
        else:
            self._usd_context = None
            self._stage_event_sub_selection = None
            self._stage_event_sub_opened = None
            self._stage_event_sub_closed = None
            self._stage_event_sub_sim_play = None
            self._stage_event_sub_sim_stop = None

    def _menu_callback(self) -> None:
        self._window.visible = not self._window.visible
        # Update the Selection Box if the Timeline is already playing
        if self._timeline.is_playing():
            self._refresh_selection_combobox()

    def _build_ui(self) -> None:
        # if not self._window:
        with self._window.frame:
            with ui.VStack(spacing=5, height=0):

                self._build_info_ui()

                self._build_selection_ui()

                self._build_kinematics_ui()

                self._build_trajectory_generation_ui()

                self._build_rmpflow_ui()

        async def dock_window() -> None:
            await omni.kit.app.get_app().next_update_async()

            def dock(space: Any, name: str, location: Any, pos: float = 0.5) -> Any:
                window = omni.ui.Workspace.get_window(name)
                if window and space:
                    window.dock_in(space, location, pos)
                return window

            tgt = ui.Workspace.get_window("Viewport")
            dock(tgt, EXTENSION_NAME, omni.ui.DockPosition.LEFT, 0.33)
            await omni.kit.app.get_app().next_update_async()

        self._task = asyncio.ensure_future(dock_window())

    def _on_selection(self, prim_path: str) -> None:
        """Creates an Articulation Object from the selected articulation prim path.

           Updates the UI with the Selected articulation.

        Args:
            prim_path: path to selected articulation
        """
        if prim_path == self._prev_art_prim_path:
            return
        else:
            self._prev_art_prim_path = prim_path

        self.new_selection = True
        self._prev_link = None

        if self.articulation_list and prim_path != "None":

            # Create and Initialize the Articulation
            self.articulation = SingleArticulation(prim_path)
            if not self.articulation.handles_initialized:
                self.articulation.initialize()

            # Update the entire UI with the selected articulaiton
            self._refresh_ui(self.articulation)

            # start event subscriptions
            if not self._physics_subscription:
                self._physics_subscription = self._physics_simulation_interface.subscribe_physics_on_step_events(
                    pre_step=False, order=0, on_update=self._on_physics_step
                )

        # Deselect and Reset
        else:
            if self.articulation is not None:
                self._reset_ui()
                self._refresh_selection_combobox()
            self.articulation = None
            # carb.log_warn("Resetting Articulation Inspector")

    def _on_combobox_selection(self, model: Any = None, val: Any = None) -> None:
        # index = model.get_item_value_model().as_int
        index = self._models["ar_selection_model"].get_item_value_model().as_int
        if index >= 0 and index < len(self.articulation_list):
            self._selected_index = index
            item = self.articulation_list[index]
            self._selected_prim_path = item
            self._on_selection(item)

    def _refresh_selection_combobox(self) -> None:
        self.articulation_list = self.get_all_articulations()
        if self._prev_art_prim_path is not None and self._prev_art_prim_path not in self.articulation_list:
            self._reset_ui()
        self._models["ar_selection_model"] = DynamicComboBoxModel(self.articulation_list)
        self._models["ar_selection_combobox"].model = self._models["ar_selection_model"]
        self._models["ar_selection_combobox"].model.add_item_changed_fn(self._on_combobox_selection)
        # If something was already selected, reselect after refresh
        if self._selected_index is not None and self._selected_prim_path is not None:
            # If the item is still in the articulation list
            if self._selected_prim_path in self.articulation_list:
                self._models["ar_selection_combobox"].model.set_item_value_model(
                    ui.SimpleIntModel(self._selected_index)
                )

    def _clear_selection_combobox(self) -> None:
        self._selected_index = None
        self._selected_prim_path = None
        self.articulation_list = []
        self._models["ar_selection_model"] = DynamicComboBoxModel(self.articulation_list)
        self._models["ar_selection_combobox"].model = self._models["ar_selection_model"]
        self._models["ar_selection_combobox"].model.add_item_changed_fn(self._on_combobox_selection)

    def get_all_articulations(self) -> list:
        """Get all the articulation objects from the Stage.

        Returns:
            list(str): list of prim_paths as strings
        """
        articulations = ["None"]
        if self._timeline.is_stopped():
            return articulations
        stage = self._usd_context.get_stage()
        if stage:
            for prim in Usd.PrimRange(stage.GetPrimAtPath("/")):
                path = str(prim.GetPath())
                # Get prim type get_prim_object_type
                type = get_prim_object_type(path)
                if type == "articulation":
                    articulations.append(path)

        return articulations

    def get_articulation_values(self, articulation: object) -> None:
        """Get and store the latest dof_properties from the articulation.

           Update the Properties UI.

        Args:
            articulation: Selected Articulation
        """
        # Update static dof properties on new selection
        if self.new_selection:
            self.num_dof = articulation.num_dof
            self.dof_names = articulation.dof_names
            self.new_selection = False

            self._joint_positions = articulation.get_joint_positions()

    def _refresh_ee_frame_combobox(self) -> None:
        if self._robot_description_file is not None and self._robot_urdf_file is not None:
            self._test_scenarios.initialize_ik_solver(self._robot_description_file, self._robot_urdf_file)
            ee_frames = self._test_scenarios.get_ik_frames()

        else:
            ee_frames = []

        name = "ee_frame"
        self._models[name] = DynamicComboBoxModel(ee_frames)
        self._models[name + "_combobox"].model = self._models[name]

        if len(ee_frames) > 0:
            self._models[name].get_item_value_model().set_value(len(ee_frames) - 1)

        self._models[name].add_item_changed_fn(self._reset_scenario)

        self._ee_frame_options = ee_frames

    def _reset_scenario(self, model: Any = None, value: Any = None) -> None:
        self._enable_lula_dropdowns()
        self._set_enable_trajectory_panel(False)

        if self.articulation is not None:
            self.articulation.post_reset()

    def _refresh_ui(self, articulation: object) -> None:
        """Updates the GUI with a new Articulation's properties.

        Args:
            articulation: The articulation to display in the UI.
        """
        # Get the latest articulation values and update the Properties UI
        self.get_articulation_values(articulation)

        if is_yaml_file(self._models["input_robot_description_file"].get_value_as_string()):
            self._enable_load_button()

    def _reset_ui(self) -> None:
        """Reset / Hide UI Elements."""
        self._clear_selection_combobox()
        self._disable_lula_dropdowns()
        self._test_scenarios.full_reset()
        self._prev_art_prim_path = None
        self._visualize_end_effector = True

    ##################################
    # Callbacks
    ##################################

    def _on_stage_selection_changed(self, event: object) -> None:
        """Callback for Stage Selection Changed Event.

        Args:
            event: Event
        """
        # On every stage event check if any articulations have been added/removed from the Stage
        self._refresh_selection_combobox()

    def _on_stage_opened(self, event: object) -> None:
        """Callback for Stage Opened Event.

        Args:
            event: Event
        """
        # On every stage event check if any articulations have been added/removed from the Stage
        self._refresh_selection_combobox()
        # stage was opened, cleanup
        self._physics_subscription = None

    def _on_stage_closed(self, event: object) -> None:
        """Callback for Stage Closed Event.

        Args:
            event: Event
        """
        # On every stage event check if any articulations have been added/removed from the Stage
        self._refresh_selection_combobox()
        # stage was closed, cleanup
        self._physics_subscription = None

    def _on_timeline_play(self, event: object) -> None:
        """Callback for Timeline Played Event.

        Args:
            event: Event
        """
        self._refresh_selection_combobox()
        index = self._models["ar_selection_model"].get_item_value_model().as_int
        selected_articulation = self.articulation_list[index]
        self._on_selection(selected_articulation)

    def _on_timeline_stop(self, event: object) -> None:
        """Callback for Timeline Stopped Event.

        Args:
            event: Event
        """
        if self._timeline.is_stopped():
            self._on_selection("None")

    def _on_physics_step(self, step: float, context: object) -> None:
        """Callback for Physics Step.

        Args:
            step: Physics step size in seconds.
            context: Physics context information.
        """
        if self.articulation is not None:
            if not self.articulation.handles_initialized:
                self.articulation.initialize()
            # Get the latest values from the articulation
            self.get_articulation_values(self.articulation)

            action = self._get_next_action()
            self.articulation.get_articulation_controller().apply_action(action)

        return

    def _get_next_action(self) -> Any:
        """Calculates the next control action for the selected articulation.

        Returns:
            The control action to apply to the articulation controller.
        """
        if self._test_scenarios.scenario_name == "Sinusoidal Target":
            w_xy = self._models["rmpflow_follow_sinusoid_w_xy"].get_value_as_float()
            w_z = self._models["rmpflow_follow_sinusoid_w_z"].get_value_as_float()
            rad_z = self._models["rmpflow_follow_sinusoid_rad_z"].get_value_as_float()
            rad_xy = self._models["rmpflow_follow_sinusoid_rad_xy"].get_value_as_float()
            height = self._models["rmpflow_follow_sinusoid_height"].get_value_as_float()

            return self._test_scenarios.get_next_action(w_xy=w_xy, w_z=w_z, rad_z=rad_z, rad_xy=rad_xy, height=height)
        else:
            return self._test_scenarios.get_next_action()

    ##################################
    # UI Builders
    ##################################

    def _build_info_ui(self) -> None:
        """Builds the information panel UI section with title, documentation link, and overview text."""
        title = EXTENSION_NAME
        doc_link = (
            "https://docs.isaacsim.omniverse.nvidia.com/latest/manipulators/manipulators_configure_rmpflow_denso.html"
        )

        overview = "This utility is used to help generate and refine the collision sphere representation of a robot.  "
        overview += "Select the Articulation for which you would like to edit spheres from the dropdown menu.  Then select a link from the robot Articulation to begin using the Sphere Editor."
        overview += "\n\nPress the 'Open in IDE' button to view the source code."

        setup_ui_headers(self._ext_id, __file__, title, doc_link, overview)

    def _build_selection_ui(self) -> None:
        """Builds the selection panel UI with articulation dropdown, file pickers for robot description and URDF,.

        and end effector frame selection controls.
        """
        frame = ui.CollapsableFrame(
            title="Selection Panel",
            height=0,
            collapsed=False,
            style=get_style(),
            style_type_name_override="CollapsableFrame",
            horizontal_scrollbar_policy=ui.ScrollBarPolicy.SCROLLBAR_AS_NEEDED,
            vertical_scrollbar_policy=ui.ScrollBarPolicy.SCROLLBAR_ALWAYS_ON,
        )
        with frame:
            with ui.VStack(style=get_style(), spacing=5, height=0):

                # Create a dynamic ComboBox for Articulation Selection

                self.articulation_list = []
                self._models["ar_selection_model"] = DynamicComboBoxModel(self.articulation_list)
                with ui.HStack():
                    ui.Label(
                        "Select Articulation",
                        width=LABEL_WIDTH,
                        alignment=ui.Alignment.LEFT_CENTER,
                        tooltip="Select Articulation",
                    )
                    self._models["ar_selection_combobox"] = ui.ComboBox(self._models["ar_selection_model"])
                    add_line_rect_flourish(False)
                self._models["ar_selection_combobox"].model.add_item_changed_fn(self._on_combobox_selection)

                # Select Robot Description YAML file

                def check_file_type(model: Any = None) -> None:
                    path = model.get_value_as_string()
                    if is_yaml_file(path):
                        self._selected_robot_description_file = model.get_value_as_string()
                        self._enable_load_button()
                    else:
                        self._selected_robot_description_file = None
                        carb.log_warn(f"Invalid path to Robot Desctiption YAML: {path}")

                kwargs = {
                    "label": "Robot Description YAML",
                    "default_val": "",
                    "tooltip": "Click the Folder Icon to Set Filepath",
                    "use_folder_picker": True,
                    "item_filter_fn": on_filter_yaml_item,
                    "folder_dialog_title": "Select Robot Description YAML file",
                    "folder_button_title": "Select YAML",
                }
                self._models["input_robot_description_file"] = str_builder(**kwargs)
                self._models["input_robot_description_file"].add_value_changed_fn(check_file_type)

                # Select Robot URDF file

                def check_urdf_file_type(model: Any = None) -> None:
                    path = model.get_value_as_string()
                    if is_urdf_file(path):
                        self._selected_robot_urdf_file = model.get_value_as_string()
                        self._enable_load_button()
                    else:
                        self._selected_robot_urdf_file = None
                        carb.log_warn(f"Invalid path to Robot URDF: {path}")

                kwargs = {
                    "label": "Robot URDF",
                    "default_val": "",
                    "tooltip": "Click the Folder Icon to Set Filepath",
                    "use_folder_picker": True,
                    "item_filter_fn": on_filter_urdf_item,
                    "folder_dialog_title": "Select Robot URDF file",
                    "folder_button_title": "Select URDF",
                }
                self._models["input_robot_urdf_file"] = str_builder(**kwargs)
                self._models["input_robot_urdf_file"].add_value_changed_fn(check_urdf_file_type)

                # Load the currently selected config files
                def on_load_config(model: Any = None, val: Any = None) -> None:
                    self._robot_description_file = self._selected_robot_description_file
                    self._robot_urdf_file = self._selected_robot_urdf_file
                    self._refresh_ee_frame_combobox()
                    self._enable_lula_dropdowns()
                    self._set_enable_trajectory_panel(False)

                self._models["load_config_btn"] = btn_builder(
                    label="Load Selected Config",
                    text="Load",
                    tooltip="Load the selected Lula config files",
                    on_clicked_fn=on_load_config,
                )

                # Select End Effector Frame Name
                name = "ee_frame"
                self._models[name] = DynamicComboBoxModel([])

                with ui.HStack():
                    ui.Label(
                        "Select End Effector Frame",
                        width=LABEL_WIDTH,
                        alignment=ui.Alignment.LEFT_CENTER,
                        tooltip="End Effector Frame to Use when following a target",
                    )
                    self._models[name + "_combobox"] = ui.ComboBox(self._models[name])
                    add_line_rect_flourish(False)

                self._models[name].add_item_changed_fn(self._reset_scenario)

                # Button for ignoring IK targets
                def on_clicked_fn(use_orientation: bool) -> None:
                    self._test_scenarios.set_use_orientation(use_orientation)

                with ui.HStack(width=0):
                    label = "Use Orientation Targets"
                    ui.Label(label, width=LABEL_WIDTH - 12, alignment=ui.Alignment.LEFT_TOP)
                    cb = ui.SimpleBoolModel(default_value=1)
                    SimpleCheckBox(1, on_clicked_fn, model=cb)

                # Button for visualizing end effector
                def on_vis_ee_clicked_fn(visualize_ee: bool) -> None:
                    self._visalize_end_effector = visualize_ee
                    if visualize_ee:
                        self._test_scenarios.visualize_ee_frame(self.articulation, self._get_selected_ee_frame())
                    else:
                        self._test_scenarios.stop_visualize_ee_frame()

                with ui.HStack(width=0):
                    label = "Visualize End Effector Pose"
                    ui.Label(label, width=LABEL_WIDTH - 12, alignment=ui.Alignment.LEFT_TOP)
                    cb = ui.SimpleBoolModel(default_value=1)
                    SimpleCheckBox(1, on_vis_ee_clicked_fn, model=cb)

    def _build_kinematics_ui(self) -> None:
        """Builds the Lula Kinematics Solver UI panel with inverse kinematics target following controls."""
        frame = ui.CollapsableFrame(
            title="Lula Kinematics Solver",
            height=0,
            collapsed=True,
            enabled=False,
            style=get_style(),
            style_type_name_override="CollapsableFrame",
            horizontal_scrollbar_policy=ui.ScrollBarPolicy.SCROLLBAR_AS_NEEDED,
            vertical_scrollbar_policy=ui.ScrollBarPolicy.SCROLLBAR_ALWAYS_ON,
        )

        self._models["kinematics_frame"] = frame

        with frame:
            with ui.VStack(style=get_style(), spacing=5, height=0):

                def ik_follow_target(model: Any = None) -> None:
                    ee_frame = self._get_selected_ee_frame()
                    self.articulation.post_reset()
                    self._test_scenarios.on_ik_follow_target(self.articulation, ee_frame)

                self._models["kinematics_follow_target_btn"] = btn_builder(
                    label="Follow Target",
                    text="Follow Target",
                    tooltip="Use IK to follow a target",
                    on_clicked_fn=ik_follow_target,
                )

    def _build_trajectory_generation_ui(self) -> None:
        """Builds the Lula Trajectory Generator UI panel with custom trajectory creation and waypoint.

        management controls.
        """
        frame = ui.CollapsableFrame(
            title="Lula Trajectory Generator",
            height=0,
            collapsed=True,
            enabled=False,
            style=get_style(),
            style_type_name_override="CollapsableFrame",
            horizontal_scrollbar_policy=ui.ScrollBarPolicy.SCROLLBAR_AS_NEEDED,
            vertical_scrollbar_policy=ui.ScrollBarPolicy.SCROLLBAR_ALWAYS_ON,
        )

        self._models["trajectory_frame"] = frame

        with frame:
            with ui.VStack(style=get_style(), spacing=5, height=0):

                def on_custom_trajectory(model: Any = None, val: Any = None) -> None:
                    self.articulation.post_reset()
                    self._test_scenarios.on_custom_trajectory(self._robot_description_file, self._robot_urdf_file)
                    self._set_enable_trajectory_panel(True)

                self._models["custom_trajectory_btn"] = btn_builder(
                    label="Custom Trajectory",
                    text="Custom Trajectory",
                    tooltip="Create a basic customizable trajectory and unlock the Custom Trajectory Panel",
                    on_clicked_fn=on_custom_trajectory,
                )

                frame = ui.CollapsableFrame(
                    title="Custom Trajectory Panel",
                    height=0,
                    collapsed=True,
                    enabled=False,
                    style=get_style(),
                    style_type_name_override="CollapsableFrame",
                    horizontal_scrollbar_policy=ui.ScrollBarPolicy.SCROLLBAR_AS_NEEDED,
                    vertical_scrollbar_policy=ui.ScrollBarPolicy.SCROLLBAR_ALWAYS_ON,
                )

                self._models["trajectory_panel"] = frame

                def follow_trajectory(model: Any = None, val: Any = None) -> None:
                    self._test_scenarios.create_trajectory_controller(self.articulation, self._get_selected_ee_frame())

                def on_add_waypoint(model: Any = None, val: Any = None) -> None:
                    self._test_scenarios.add_waypoint()

                def on_delete_waypoint(model: Any = None, val: Any = None) -> None:
                    self._test_scenarios.delete_waypoint()

                with frame:
                    with ui.VStack(style=get_style(), spacing=5, height=0):
                        self._models["follow_trajectory_btn"] = btn_builder(
                            label="Follow Trajectory",
                            text="Follow Trajectory",
                            tooltip="Follow the trajectory shown in front of the robot",
                            on_clicked_fn=follow_trajectory,
                        )

                        self._models["add_trajectory_waypoint_btn"] = btn_builder(
                            label="Add Waypoint",
                            text="Add Waypoint",
                            tooltip="Add waypoint to trajectory",
                            on_clicked_fn=on_add_waypoint,
                        )

                        self._models["remove_trajectory_waypoint_btn"] = btn_builder(
                            label="Remove Waypoint",
                            text="Remove Waypoint",
                            tooltip="Remove waypoint from trajectory",
                            on_clicked_fn=on_delete_waypoint,
                        )

    def _build_rmpflow_ui(self) -> None:
        """Builds the RmpFlow UI panel with configuration file selection, target following controls,.

        and sinusoidal trajectory parameters.
        """
        frame = ui.CollapsableFrame(
            title="RmpFlow",
            height=0,
            collapsed=True,
            enabled=False,
            style=get_style(),
            style_type_name_override="CollapsableFrame",
            horizontal_scrollbar_policy=ui.ScrollBarPolicy.SCROLLBAR_AS_NEEDED,
            vertical_scrollbar_policy=ui.ScrollBarPolicy.SCROLLBAR_ALWAYS_ON,
        )

        self._models["rmpflow_frame"] = frame

        with frame:
            with ui.VStack(style=get_style(), spacing=5, height=0):

                def check_file_type(model: Any = None) -> None:
                    path = model.get_value_as_string()
                    if is_yaml_file(path):
                        self._rmpflow_config_yaml = model.get_value_as_string()
                        self._set_enable_rmpflow_buttons(True)
                    else:
                        self._rmpflow_config_yaml = None
                        self._set_enable_rmpflow_buttons(False)
                        carb.log_warn(f"Invalid path to RmpFlow config YAML: {path}")

                kwargs = {
                    "label": "RmpFlow Config YAML",
                    "default_val": "",
                    "tooltip": "Click the Folder Icon to Set Filepath",
                    "use_folder_picker": True,
                    "item_filter_fn": on_filter_yaml_item,
                    "folder_dialog_title": "Select RmpFlow config YAML file",
                    "folder_button_title": "Select YAML",
                }
                self._models["input_rmp_config_file"] = str_builder(**kwargs)
                self._models["input_rmp_config_file"].add_value_changed_fn(check_file_type)

                def toggle_rmpflow_debug_mode(model: Any = None) -> None:
                    self._test_scenarios.toggle_rmpflow_debug_mode()

                self._models["rmpflow_debug_mode"] = state_btn_builder(
                    label="Debugger",
                    a_text="Debugging Mode",
                    b_text="Normal Mode",
                    tooltip="Toggle Debugging Mode",
                    on_clicked_fn=toggle_rmpflow_debug_mode,
                )

                ######################################################
                #                    Follow Target
                ######################################################

                def rmpflow_follow_target(model: Any = None) -> None:
                    ee_frame = self._get_selected_ee_frame()
                    rmpflow_config_dict = {
                        "end_effector_frame_name": ee_frame,
                        "maximum_substep_size": 0.0034,
                        "ignore_robot_state_updates": False,
                        "robot_description_path": self._robot_description_file,
                        "urdf_path": self._robot_urdf_file,
                        "rmpflow_config_path": self._rmpflow_config_yaml,
                    }
                    self.articulation.post_reset()
                    self._test_scenarios.on_rmpflow_follow_target_obstacles(self.articulation, **rmpflow_config_dict)

                self._models["rmpflow_follow_target_btn"] = btn_builder(
                    label="Follow Target",
                    text="Follow Target",
                    tooltip="Use RmpFlow to follow a target",
                    on_clicked_fn=rmpflow_follow_target,
                )
                self._models["rmpflow_follow_target_btn"].enabled = False

                #######################################################
                #                Sinusoidal Target
                #######################################################

                def rmpflow_follow_sinusoidal_target(model: Any = None) -> None:
                    ee_frame = self._get_selected_ee_frame()
                    rmpflow_config_dict = {
                        "end_effector_frame_name": ee_frame,
                        "maximum_substep_size": 0.0034,
                        "ignore_robot_state_updates": False,
                        "robot_description_path": self._robot_description_file,
                        "urdf_path": self._robot_urdf_file,
                        "rmpflow_config_path": self._rmpflow_config_yaml,
                    }
                    self.articulation.post_reset()
                    self._test_scenarios.on_rmpflow_follow_sinusoidal_target(self.articulation, **rmpflow_config_dict)

                self._models["rmpflow_follow_sinusoid_btn"] = btn_builder(
                    label="Follow Sinusoid",
                    text="Follow Sinusoid",
                    tooltip="Use RmpFlow to follow a rotating sinusoidal target",
                    on_clicked_fn=rmpflow_follow_sinusoidal_target,
                )
                self._models["rmpflow_follow_sinusoid_btn"].enabled = False

                frame = ui.CollapsableFrame(
                    title="Sinusoid Parameters",
                    height=0,
                    collapsed=True,
                    enabled=False,
                    style=get_style(),
                    style_type_name_override="CollapsableFrame",
                    horizontal_scrollbar_policy=ui.ScrollBarPolicy.SCROLLBAR_AS_NEEDED,
                    vertical_scrollbar_policy=ui.ScrollBarPolicy.SCROLLBAR_ALWAYS_ON,
                )

                self._models["rmpflow_sinusoidal_target_frame"] = frame

                with frame:
                    with ui.VStack(style=get_style(), spacing=5, height=0):

                        self._models["rmpflow_follow_sinusoid_w_z"] = float_builder(
                            label="Vertical Wave Frequency",
                            default_val=0.05,
                            tooltip="Speed [rad/sec] at which the target makes vertical oscilations",
                        )
                        self._models["rmpflow_follow_sinusoid_rad_z"] = float_builder(
                            label="Vertical Wave Radius", default_val=0.2, tooltip="Height [m] of vertical oscilations"
                        )

                        self._models["rmpflow_follow_sinusoid_w_xy"] = float_builder(
                            label="Z Axis Rotation Frequency",
                            default_val=0.05,
                            tooltip="Speed [rad/sec] at which the target makes a full circle about the z axis",
                        )
                        self._models["rmpflow_follow_sinusoid_rad_xy"] = float_builder(
                            label="Distance From Origin",
                            default_val=0.5,
                            tooltip="Distance on the XY plane from the origin [m] of the target",
                        )

                        self._models["rmpflow_follow_sinusoid_height"] = float_builder(
                            label="Sinusoid Height", default_val=0.5, tooltip="Average height of target [m]"
                        )

    def _disable_lula_dropdowns(self) -> None:
        """Disables and collapses the Lula kinematics, trajectory, and RmpFlow UI panels."""
        frame_names = ["kinematics_frame", "trajectory_frame", "rmpflow_frame", "trajectory_panel"]
        for n in frame_names:
            frame = self._models[n]
            frame.enabled = False
            frame.collapsed = True

    def _enable_load_button(self) -> None:
        """Enables the config file load button when valid robot description file is selected."""
        self._models["load_config_btn"].enabled = True

    def _enable_lula_dropdowns(self) -> None:
        """Enables the Lula kinematics, trajectory, and RmpFlow UI panels when articulation and.

        configuration files are loaded.
        """
        if self.articulation is None or self._robot_description_file is None or self._robot_urdf_file is None:
            return

        frame_names = ["kinematics_frame", "trajectory_frame", "rmpflow_frame"]
        for n in frame_names:
            frame = self._models[n]
            frame.enabled = True

        self._test_scenarios.scenario_reset()
        self._test_scenarios.initialize_ik_solver(self._robot_description_file, self._robot_urdf_file)

        if self._visualize_end_effector:
            self._test_scenarios.visualize_ee_frame(self.articulation, self._get_selected_ee_frame())

    def _set_enable_trajectory_panel(self, enable: bool) -> None:
        """Enables or disables the trajectory panel frame.

        Args:
            enable: Whether to enable the trajectory panel.
        """
        frame = self._models["trajectory_panel"]
        frame.enabled = enable
        frame.collapsed = not enable

    def _set_enable_rmpflow_buttons(self, enable: bool) -> None:
        """Enables or disables the RmpFlow buttons.

        Args:
            enable: Whether to enable the RmpFlow buttons.
        """
        self._models["rmpflow_follow_target_btn"].enabled = enable
        self._models["rmpflow_follow_sinusoid_btn"].enabled = enable

    def _get_selected_ee_frame(self) -> str:
        """Get the selected end effector frame name from the dropdown options.

        Returns:
            The name of the selected end effector frame.
        """
        name = "ee_frame"
        return self._ee_frame_options[self._models[name].get_item_value_model().as_int]
