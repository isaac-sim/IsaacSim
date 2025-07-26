# SPDX-FileCopyrightText: Copyright (c) 2022-2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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

import asyncio
import os
from typing import List

import carb
import numpy as np
import omni.kit.commands
import omni.timeline
import omni.ui as ui
import omni.usd
import pxr
from isaacsim.core.utils.stage import update_stage_async
from isaacsim.gui.components.element_wrappers import (
    CollapsableFrame,
    DropDown,
    FloatField,
    Frame,
    StateButton,
    StringField,
    TextBlock,
)
from isaacsim.gui.components.ui_utils import add_line_rect_flourish, setup_ui_headers
from omni.kit.widget.filebrowser import FileBrowserItem
from omni.kit.window.filepicker import FilePickerDialog
from pxr import Gf, Sdf, Usd, UsdGeom, UsdPhysics
from usd.schema.isaac import robot_schema

from ..global_variables import EXTENSION_TITLE
from ..robot_assembler import RobotAssembler
from .style import get_style
from .widget import LABEL_WIDTH, DropDownWithPicker, DropDownWithSelect


class UIBuilder:

    AUTO_CREATE = "AUTO_CREATE_FRAME"

    def __init__(self):
        # Get access to the timeline to control stop/pause/play programmatically
        self._timeline = omni.timeline.get_timeline_interface()

        self.wrapped_ui_elements = []

        self._robot_assembler = RobotAssembler()

    ###################################################################################
    #           The Functions Below Are Called Automatically By extension.py
    ###################################################################################

    def on_menu_callback(self):
        """Callback for when the UI is opened from the toolbar.
        This is distinct from the creation of the UI in build_ui()
        because it can happen more than once if the user repeatedly
        closes and reopens the window.

        This callback happens after build_ui() when the extension is first opened
        """
        # Handles the edge case where the user loads their Articulation and
        # presses play before opening this extension
        if self._timeline.is_playing():
            self._repopulate_all_dropdowns()

        pass

    def on_timeline_event(self, event):
        """Callback for Timeline events (Play, Pause, Stop)

        Args:
            event (omni.timeline.TimelineEventType): Event Type
        """
        pass

    def on_physics_step(self, step):
        """Callback for Physics Step.
        Physics steps only occur when the timeline is playing

        Args:
            step (float): Size of physics step
        """
        pass

    def reset_ui(self):
        self._robot_frame.collapsed = False
        self._robot_frame.visible = True
        self._assembler_frame.collapsed = True
        self._assembler_frame.visible = False

        self.original_attachment_pose = None

        self._begin_assemble_btn.enabled = True

        for robot_frame in self._robot_frames:
            robot_frame.visible = True

        self.simulate_and_assemble_button.enabled = True
        self.end_simulation_and_continue_button.enabled = False

        self._repopulate_all_dropdowns()

    def on_stage_event(self, event: omni.usd.StageEventType):
        """Callback for Stage Events

        Args:
            event (omni.usd.StageEventType): Event Type
        """

        if event.type == int(omni.usd.StageEventType.ASSETS_LOADED):
            self._repopulate_all_dropdowns()
        elif event.type == int(omni.usd.StageEventType.OPENED):
            self.reset_ui()
        elif event.type == int(omni.usd.StageEventType.SIMULATION_START_PLAY):  # Timeline played
            self._wait_and_reselect_articulations()
        elif event.type == int(omni.usd.StageEventType.SIMULATION_STOP_PLAY):  # Timeline played
            if self._timeline.is_stopped():
                self._reselect_articulations()
                self._articulation_options = []

    def cleanup(self):
        """
        Called when the stage is closed or the extension is hot reloaded.
        Perform any necessary cleanup such as removing active callback functions
        Buttons imported from isaacsim.gui.components.element_wrappers implement a cleanup function that should be called
        """
        for ui_elem in self.wrapped_ui_elements:
            ui_elem.cleanup()
        self._robot_assembler.reset()

    ######################################################################################################
    #                                           Build UI
    ######################################################################################################

    def build_ui(self):
        """
        Build a custom UI tool to run your extension.
        This function will be called once when your extension is opened.
        Closing and reopening the extension from the toolbar will maintain the state of the UI.
        If the user hot reloads this extension, this function will be called again.
        """
        names = ["Base Robot", "Attach Robot"]
        self._robot_frames = []
        self._robot_control_frames = []
        self._robot_dropdowns = []
        self._articulation_attach_point_dropdowns = []
        self._attach_map = []
        self._articulations = [None] * len(names)
        self._articulation_options = []
        self._articulation_options_pre_nest = []
        self._collapsable_robot_control_frames = [None] * len(names)
        self._show_art_cbs = []
        self._show_rigid_body_cbs = []

        self._converted_rigid_bodies = []

        self.wrapped_ui_elements = []

        self._joint_control_frames = []
        self._joint_position_float_fields = []

        self._articulations_nested = False
        self._assembled_robot = None

        self._assemble_namespace = "Gripper"

        # self._make_info_frame()
        ui.Spacer(height=10)
        self._robot_frame = ui.CollapsableFrame(
            "Select Robots", style=get_style(), name="option", height=0, collapsed=False
        )
        self._robot_frames.append(self._robot_frame)
        with self._robot_frame:
            with ui.HStack():
                ui.Spacer(width=16)
                with ui.VStack(style=get_style(), spacing=12, height=0):
                    heading_title = "Assemble Robots"
                    heading_text = "Attach two robots together. For example, you can attach a gripper to an arm."
                    self._make_info_heading(heading_title, heading_text)

                    for idx in range(len(names)):
                        #
                        self._attach_map.append([])

                        self._make_heading(f"{names[idx]}")

                        robot_selection_menu = DropDown(
                            f"Select {names[idx]}",
                            tooltip=f"Load a USD file containing a robot.",
                            on_selection_fn=lambda selection, ind=idx: self._on_prim_selection(ind, selection),
                            keep_old_selections=True,
                            populate_fn=lambda ind=idx: self._dropdown_populate_robot_asset_fn(ind),
                            add_flourish=False,
                        )
                        # with robot_selection_menu.combo_adorner_hstack:
                        #     # ui.Button("X", width=25, spacing=5, clicked_fn=lambda ind=idx: self._on_load_asset(ind))
                        #     ui.Button(
                        #         "",
                        #         width=25,
                        #         spacing=5,
                        #         clicked_fn=lambda ind=idx: self._on_load_asset(ind),
                        #         style={"image_url": "resources/icons/find.png"},
                        #     )

                        attachment_selection_menu = DropDown(
                            f"Attach Point",
                            tooltip="Select a Prim to use as an attach point.",
                            keep_old_selections=True,
                            populate_fn=lambda ind=idx: self._attach_point_populate_fn(ind),
                            on_selection_fn=lambda selection, ind=idx: self._on_prim_selection(ind, selection),
                            add_flourish=False,
                        )
                        self._robot_dropdowns.append(robot_selection_menu)
                        self._articulation_attach_point_dropdowns.append(attachment_selection_menu)
                        self.wrapped_ui_elements.append(robot_selection_menu)
                        self.wrapped_ui_elements.append(attachment_selection_menu)

                    with ui.HStack():
                        ui.Line(width=ui.Fraction(1.0))

                    with ui.HStack():
                        ui.Label("Assembly Namespace", width=LABEL_WIDTH)
                        self._assembly_namespace_field = ui.StringField(
                            ui.SimpleStringModel(self._assemble_namespace),
                            tooltip="The namespace to use for the assembly.",
                        )

                    def on_toggle_joint_gizmos_btn_clicked():
                        settings = carb.settings.get_settings()
                        joint_gizmo_setting = "/persistent/physics/visualizationDisplayJoints"
                        settings.set(joint_gizmo_setting, not settings.get(joint_gizmo_setting))

                    def on_begin_assemble_btn_clicked():
                        """
                        if (
                            self._robot_dropdowns[0].get_selection() in self._articulation_options
                            and self._robot_dropdowns[0].get_selection() in self._articulation_options
                        ):
                            self.single_robot_cb.visible = True
                        else:
                            self.single_robot_cb.visible = False
                        """
                        self._attach_selection()

                        self._assembler_frame.collapsed = False
                        self._assembler_frame.visible = True
                        self._begin_assemble_btn.enabled = False
                        self._robot_frame.visible = False
                        self._robot_frame.collapsed = True
                        self.simulate_and_assemble_button.visible = True
                        self.simulate_and_assemble_button.enabled = True
                        self.end_simulation_and_continue_button.visible = False
                        self.end_simulation_and_continue_button.enabled = False

                        for robot_frame in self._robot_frames:
                            robot_frame.visible = False

                    with ui.HStack():
                        ui.Spacer()
                        self._begin_assemble_btn = ui.Button(
                            "Begin Assembling Process",
                            tooltip="Press this to begin assembling the selected robots",
                            width=400,
                            height=32,
                            clicked_fn=on_begin_assemble_btn_clicked,
                        )
                        ui.Spacer()
                    ui.Spacer(width=12)
                ui.Spacer(width=16)

        self._make_assemble_frame(names)

    def apply_rotation(self, axis, angle):
        stage = omni.usd.get_context().get_stage()
        prim = stage.GetPrimAtPath(self._robot_assembler._attachment_robot_prim)

        xformable = UsdGeom.Xformable(prim)
        old_matrix = xformable.GetLocalTransformation()
        old_rotation = old_matrix.ExtractRotation()
        rotation = Gf.Rotation(axis, angle)
        new_matrix = Gf.Matrix4d().SetRotateOnly(rotation) * old_matrix

        omni.kit.commands.execute(
            "TransformPrimCommand",
            path=prim.GetPath(),
            old_transform_matrix=old_matrix,
            new_transform_matrix=new_matrix,
        )

    def on_rotate_x_90_pos_clicked(self):
        self.apply_rotation((1, 0, 0), 90)

    def on_rotate_x_90_neg_clicked(self):
        self.apply_rotation((1, 0, 0), -90)

    def on_rotate_y_90_pos_clicked(self):
        self.apply_rotation((0, 1, 0), 90)

    def on_rotate_y_90_neg_clicked(self):
        self.apply_rotation((0, 1, 0), -90)

    def on_rotate_z_90_pos_clicked(self):
        self.apply_rotation((0, 0, 1), 90)

    def on_rotate_z_90_neg_clicked(self):
        self.apply_rotation((0, 0, 1), -90)

    def _make_assemble_frame(self, names):
        def on_cancel_assemble_btn_clicked():
            if self._timeline.is_playing():
                self._timeline.stop()

            async def async_cancel():
                await omni.kit.app.get_app().next_update_async()
                self._robot_assembler.cancel_assembly()

            asyncio.ensure_future(async_cancel())

            self.reset_ui()

        def on_simulate_and_assemble_button_clicked(make_single_robot):

            async def async_assemble():
                self._robot_assembler.assemble()
                await omni.kit.app.get_app().next_update_async()
                self._timeline.play()

            asyncio.ensure_future(async_assemble())

            #
            self.simulate_and_assemble_button.enabled = False
            self.simulate_and_assemble_button.visible = False
            self.end_simulation_and_continue_button.enabled = True
            self.end_simulation_and_continue_button.visible = True

        def on_end_simulation_and_continue_button_clicked(make_single_robot):
            #

            async def async_finish_assemble():
                if self._timeline.is_playing():
                    self._timeline.stop()
                    await omni.kit.app.get_app().next_update_async()
                self._robot_assembler.finish_assemble()

            asyncio.ensure_future(async_finish_assemble())
            #
            self.reset_ui()

        self._assembler_frame = ui.CollapsableFrame("Robot Assembler", style=get_style(), collapsed=True, visible=False)
        with self._assembler_frame:
            with ui.HStack():
                ui.Spacer(width=12)
                with ui.VStack(style=get_style(), spacing=12, height=0):

                    heading_title = "Adjust Attachment Robot Transform"
                    heading_text = "If the attachment needs to be adjusted, Selec the attach point and adjust by moving or rotating."
                    ui.Spacer(height=12)
                    self._make_info_heading(heading_title, heading_text)

                    select_attach_point_btn = ui.Button(
                        "Select Attach Point Prim",
                        tooltip="Select the attach point frame of the attach robot in order to specify the relative pose.",
                        clicked_fn=self._select_attach_point_prim,
                    )

                    with ui.HStack(style=get_style(), spacing=5, height=0):
                        with ui.VStack(style=get_style(), spacing=5, height=0):
                            Rotate_X_90_pos = ui.Button("X +90", clicked_fn=self.on_rotate_x_90_pos_clicked)
                            Rotate_X_90_neg = ui.Button("X -90", clicked_fn=self.on_rotate_x_90_neg_clicked)
                        with ui.VStack(style=get_style(), spacing=5, height=0):
                            Rotate_Y_90_pos = ui.Button("Y +90", clicked_fn=self.on_rotate_y_90_pos_clicked)
                            Rotate_Y_90_neg = ui.Button("Y -90", clicked_fn=self.on_rotate_y_90_neg_clicked)
                        with ui.VStack(style=get_style(), spacing=5, height=0):
                            Rotate_Z_90_pos = ui.Button("Z +90", clicked_fn=self.on_rotate_z_90_pos_clicked)
                            Rotate_Z_90_neg = ui.Button("Z -90", clicked_fn=self.on_rotate_z_90_neg_clicked)

                    # self.single_robot_cb = CheckBox(
                    #     "Assemble Into Single Robot",
                    #     default_value=False,
                    #     tooltip="When checked, the attached robots will be treated as a single Articulation that can be accessed at the prim path of the base robot."
                    #     + "  When not checked, the attached robots will still be treated as separate Articulations that are controlled independently.",
                    # )
                    ui.Spacer(height=12)
                    info_text = 'Simulate and Assemble will test the created joint that attaches the two robots. Once tested, clicking "End Simulation And Continue" will stop the simulation and complete the assembly.'
                    self._make_info_display(info_text)
                    ui.Spacer(height=12)
                    with ui.VStack(spacing=6):
                        self.simulate_and_assemble_button = ui.Button(
                            "Assemble and Simulate",
                            tooltip="Assemble the selected robots and simulate final robot.",
                            height=32,
                            clicked_fn=lambda: on_simulate_and_assemble_button_clicked(False),
                        )

                        self.end_simulation_and_continue_button = ui.Button(
                            "End Simulation And Finish",
                            tooltip="End Simulation And Finish",
                            height=32,
                            clicked_fn=lambda: on_end_simulation_and_continue_button_clicked(False),
                        )
                        self.end_simulation_and_continue_button.visible = False
                        self.end_simulation_and_continue_button.enabled = False

                        cancel_assemble_btn = ui.Button(
                            "Cancel Assemble",
                            tooltip="Press this to cancel the assembly.",
                            height=32,
                            clicked_fn=on_cancel_assemble_btn_clicked,
                        )
                ui.Spacer(width=12)

    def _build_set_robot_position_frame(self, idx):
        pass

    ##########################################################################################
    #                              Robot Assembler Frame Functions
    ##########################################################################################

    def _get_attach_point(self, ind):
        pt = self._articulation_attach_point_dropdowns[ind].get_selection()
        if pt == self.AUTO_CREATE:
            return ""
        else:
            return self._attach_map[ind][pt]

    # To create a variant for the gripper, we need to move the asset into the variant layer
    # BUT, if the gripper is a payload, we have to move the prim with the payload
    # This assumes the asset doesn't have any other nesting happening
    def _find_payload_path(self, stage: Usd.Stage, attach_path: str) -> str:
        prim = stage.GetPrimAtPath(attach_path)
        while prim:
            payload_metadata = prim.GetMetadata("payload")
            if payload_metadata:
                return prim.GetPath().pathString
            prim = prim.GetParent()

        # no payload found
        return attach_path

    def _attach_selection(self):
        stage = omni.usd.get_context().get_stage()

        base_robot_path = self._robot_dropdowns[0].get_selection()
        base_mount_path = self._get_attach_point(0)

        attachment_robot_path = self._robot_dropdowns[1].get_selection()
        attachment_mount_path = self._get_attach_point(1)

        base_attach_path = self._get_attach_point(0)
        gripper_attach_path = self._get_attach_point(1)

        if base_mount_path is None or attachment_mount_path is None:
            carb.log_error("Begin Assemble Button was Clicked before valid robots were selected")

        variant_set = self._assembly_namespace_field.model.get_value_as_string()

        prim = stage.GetPrimAtPath(attachment_robot_path)
        variant_name = attachment_robot_path.rsplit("/", 1)[-1]
        composed_refs = omni.usd.get_composed_references_from_prim(prim)
        composed_payloads = omni.usd.get_composed_payloads_from_prim(prim)
        if len(composed_refs) != 0:
            print(composed_refs[0])
            variant_name = composed_refs[0][0].assetPath.split("/")[-1].split(".")[0]
        elif len(composed_payloads) != 0:
            variant_name = composed_payloads[0][0].assetPath.split("/")[-1].split(".")[0]

        self._robot_assembler.begin_assembly(
            stage,
            base_robot_path,
            base_mount_path,
            attachment_robot_path,
            attachment_mount_path,
            variant_set,
            variant_name,
        )

        self._select_attach_point_prim()

    def _select_attach_point_prim(self):
        stage = omni.usd.get_context().get_stage()
        asset_path = self._find_payload_path(stage, self._robot_assembler._attachment_robot_prim)
        omni.kit.commands.execute(
            "SelectPrimsCommand",
            old_selected_paths=[],
            new_selected_paths=[asset_path],
            expand_in_stage=False,
        )

    ############################################################################################
    #                               Assembly Frame Functions
    ############################################################################################

    def _attach_point_populate_fn(self, art_ind: int) -> List[str]:
        selected_robot = self._robot_dropdowns[art_ind].get_selection()
        if selected_robot is None:
            return [self.AUTO_CREATE]
        attach_points = self._get_attach_points(selected_robot)
        if art_ind == 0:
            attach_points.reverse()

        # Offer to auto-create a frame only if the object is NOT an Articulation
        if selected_robot not in self._articulation_options:
            self._attach_map[art_ind] = {self.AUTO_CREATE: ""}
        else:
            self._attach_map[art_ind] = {pxr.Sdf.Path(p).name: p for p in attach_points}
        attach_points = [p for p in self._attach_map[art_ind].keys()]
        return attach_points

    def _get_attach_points(self, selected_robot):
        stage = omni.usd.get_context().get_stage()

        reference_points = []
        if stage and selected_robot is not None:
            reference_points = [
                str(p.GetPath())
                for p in Usd.PrimRange(stage.GetPrimAtPath(selected_robot))
                if p.HasAPI(robot_schema.Classes.REFERENCE_POINT_API.value)
            ]
            reference_points += [
                str(p)
                for p in stage.GetPrimAtPath(selected_robot)
                .GetRelationship(robot_schema.Relations.ROBOT_LINKS.name)
                .GetTargets()
            ]

        return reference_points

    ##########################################################################################
    #                            Robot Selection Frame Functions
    ##########################################################################################

    def _wait_and_reselect_articulations(self):
        # Certain physics things will occasionally take two frames to start working.
        async def wait_and_reselect():
            await update_stage_async()
            await update_stage_async()
            for dropdown in self._robot_dropdowns:
                dropdown.trigger_on_selection_fn_with_current_selection()

        asyncio.ensure_future(wait_and_reselect())

    def _reselect_articulations(self):
        for dropdown in self._robot_dropdowns:
            dropdown.trigger_on_selection_fn_with_current_selection()

    def _on_prim_selection(self, art_ind: int, selection: str):

        self._repopulate_all_dropdowns()

        # self._articulation_attach_point_dropdowns[art_ind].repopulate()

    def _on_set_joint_position_target(self, robot_index: int, joint_index: int, position_target: float):
        pass

    def _repopulate_all_dropdowns(self):
        for d in self._robot_dropdowns:
            d.repopulate()
        for d in self._articulation_attach_point_dropdowns:
            d.repopulate()

        # Repopulating articulation menus will recursively repopulate articulation_attach_point dropdowns

    def _filter_usd_files(self, item: FileBrowserItem) -> bool:
        # help the filebrowser properly select stuff
        if not item or item.name.endswith(".usd") or item.name.endswith(".usda") or item.is_folder:
            return True
        return False

    def _dropdown_populate_robot_asset_fn(self, ind: int) -> List[str]:
        # Pick an articulation from the stage that has not been selected already
        selections = [d.get_selection() for d in self._robot_dropdowns[:ind]]
        stage = omni.usd.get_context().get_stage()
        prims = [stage.GetPrimAtPath(p) for p in selections if p]

        links = [p.GetRelationship(robot_schema.Relations.ROBOT_LINKS.name).GetTargets() for p in prims]

        options = self._find_all_robot_assets()
        self._articulation_options = options.copy()
        stage = omni.usd.get_context().get_stage()

        for selection in selections:
            if selection in options:
                options.remove(selection)
        for option in options:
            for robot_links in links:
                if pxr.Sdf.Path(option) in robot_links:
                    options.remove(option)

        return options

    def _find_all_robot_assets(self) -> List[str]:
        robots = []
        stage = omni.usd.get_context().get_stage()

        # JW - This could also try to filter out grippers already in variants
        robots = [
            str(p.GetPath())
            for p in pxr.Usd.PrimRange(stage.GetPrimAtPath("/"))
            if p.HasAPI(robot_schema.Classes.ROBOT_API.value)
        ]

        return robots

    def _make_heading(self, heading_title: str, width=0):
        with ui.HStack():
            ui.Label(heading_title, width=width)
            ui.Spacer(width=5)
            ui.Line(width=ui.Fraction(1.0))
            ui.Spacer(width=5)

    def _make_info_display(self, info_text: str):

        with ui.HStack(style=get_style()):
            with ui.VStack(width=20):
                ui.Spacer()
                ui.Image(name="info", width=20, height=20)
                ui.Spacer()
            ui.Spacer(width=12)
            with ui.VStack():
                ui.Spacer()
                ui.Label(info_text, name="info", width=ui.Fraction(1.0), height=10, word_wrap=True)
                ui.Spacer()
            ui.Spacer(width=25)

    def _make_info_heading(self, heading_title: str, info_text: str):
        self._make_heading(heading_title)
        self._make_info_display(info_text)

    # handle loading robot assets
    def _robot_file_asset_selected(self, filepicker, dirname: str, filename: str, index: int):
        # done with dialog
        filepicker.hide()
        filepicker = None

        stage = omni.usd.get_context().get_stage()

        # create the robot prim path
        robot_prim_path = filename.rsplit(".", 1)[0]
        robot_prim_path = robot_prim_path.replace(" ", "_")
        robot_prim_path = f"/World/Robot_{index}_{robot_prim_path}"
        while stage.GetPrimAtPath(robot_prim_path):
            index += 1
            robot_prim_path = f"/World/Robot_{index}_{robot_prim_path}"

        # load the robot asset
        robot_asset_prim = stage.DefinePrim(robot_prim_path, "Xform")
        payloads = robot_asset_prim.GetPayloads()
        payload = Sdf.Payload(assetPath=f"{dirname}/{filename}")
        payloads.AddPayload(payload)

        #
        self._repopulate_all_dropdowns()

    def _on_load_asset(self, index: int):
        filepicker = FilePickerDialog(
            "Select Robot Asset",
            apply_button_label="Select",
            click_apply_handler=lambda filename, dirname: self._robot_file_asset_selected(
                filepicker, dirname, filename, index
            ),
            item_filter_fn=lambda item: self._filter_usd_files(item),
        )
        filepicker.show()
        pass
