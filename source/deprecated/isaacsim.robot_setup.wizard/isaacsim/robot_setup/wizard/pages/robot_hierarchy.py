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

"""Robot hierarchy wizard page for organizing robot meshes into hierarchical link structures."""

import asyncio
import weakref
from typing import Any

import carb
import omni.kit.commands
import omni.ui as ui
import omni.usd
import omni.usd.commands
from omni.kit.widget.stage import StageWidget
from omni.kit.window.popup_dialog import FormDialog
from pxr import Sdf, Usd, UsdGeom

from ..builders.hierarchy_helper import apply_hierarchy
from ..builders.robot_templates import RobotRegistry
from ..utils.robot_asset_picker import SelectionWatch
from ..utils.robot_stage_delegate import RobotStageDelegate
from ..utils.ui_utils import ButtonWithIcon, custom_header, next_step, separator
from ..utils.utils import copy_prim_hierarchy


class RobotHierarchy:
    """User interface for organizing robot meshes into a hierarchical link structure.

    Provides an interactive interface where users can reorganize robot components by creating and managing links,
    then assigning mesh objects to appropriate links through drag-and-drop operations. The interface displays
    two main areas: a "New Link Structure" section for building the desired hierarchy, and a robot browser
    below showing available mesh components that can be assigned to links.

    Users can add or remove links, parent mesh objects to specific links, and apply the new structure to
    update the robot's organization. The interface includes buttons for parenting/unparenting objects,
    clearing the structure, and copying existing hierarchies.

    Args:
        visible: Whether the UI frame is initially visible.
        *args: Variable length argument list passed to parent initialization.
        **kwargs: Additional keyword arguments passed to parent initialization.
    """

    def __init__(self, visible: bool, *args: object, **kwargs: object) -> None:
        self._robot = None
        self._next_step_button = None
        self._links_selected = []  # the staging area for new hierarchy
        self._step_selected = []  # start out as a copy of the current robot hierarchy
        self.frame = ui.Frame(visible=visible)
        self.frame.set_build_fn(self._build_frame)
        self._link_temp_stage = {}  # the temporary stage for staging new hierarchy
        self._link_popup_window = None
        self._history = {}  # the popup window for adding new links
        self._reference_mesh = (
            {}
        )  # the dictionary that keeps track of the reference mesh for each link {"link_name": "reference_mesh_name"}

        self._initial_link_window_height = 100
        self._initial_robot_window_height = 150

    def destroy(self) -> None:
        """Destroy the robot hierarchy widget and clean up resources."""
        self.frame.destroy()
        self._robot = None
        if self._link_popup_window:
            self._link_popup_window.destroy()
            self._link_popup_window = None

    def clean(self) -> None:
        """Clean up stage widgets, selection watches, and reset internal state."""
        self._links_selection_watch = None
        self._links_stage_widget.open_stage(None)
        self._links_stage_widget.destroy()
        self._links_stage_widget = None

        self._step_selection_watch = None
        self._step_stage_widget.open_stage(None)
        self._step_stage_widget.destroy()
        self._step_stage_widget = None

        self._history = {}
        self._reference_mesh = {}

    def _build_frame(self) -> None:
        """Build the UI frame for the robot hierarchy widget."""
        with ui.CollapsableFrame("Robot Hierarchy", build_header_fn=custom_header):
            with ui.ScrollingFrame():
                with ui.VStack(spacing=2, name="margin_vstack"):
                    with ui.CollapsableFrame("Instruction", build_header_fn=custom_header, height=0, collapsed=True):
                        ui.Label(
                            "Organize meshes by link. \nSelect the target link under 'New Hierarchy' and the relevant objects from the bottom list, click 'Parent' to parent the objects to the target link. \nAdd/Remove links using the buttons at the bottom-right corner of the 'New Hierarchy'; Rename the link by clicking the link name."
                        )
                    ui.Spacer(height=8)
                    ui.Label("New Link Structure", name="sub_title", height=0)
                    with ui.ZStack(height=self._initial_link_window_height):
                        with ui.VStack():
                            self._links_stage_widget = StageWidget(
                                None, columns_enabled=[], stage_delegate=RobotStageDelegate()
                            )
                            stage_model = self._links_stage_widget.get_model()
                            stage_model.filter_by_text(None)
                            self._links_stage_widget._search.visible = False
                            self._links_stage_widget._filter_button._container.visible = False
                            self._links_stage_widget._option_button.button.visible = False
                            self._links_stage_widget._tree_view.header_visible = False
                        self._links_splitter = ui.Placer(
                            offset_y=self._initial_link_window_height,
                            drag_axis=ui.Axis.Y,
                            draggable=True,
                        )

                        with self._links_splitter:
                            with ui.ZStack(height=4):
                                ui.Rectangle(name="splitter_highlight")
                        self._links_splitter.set_offset_y_changed_fn(self._on_link_splitter_dragged)

                    with ui.HStack(height=0):
                        ui.Spacer()
                        self._add_link_button = ui.Button(
                            width=32,
                            height=32,
                            name="link_add",
                            clicked_fn=lambda: self._add_link_item(),
                        )
                        self._delete_link_button = ui.Button(
                            width=32,
                            height=32,
                            name="link_delete",
                            clicked_fn=lambda: self._delete_link_item(),
                        )
                        ui.Button(
                            "Clear ALL",
                            width=50,
                            height=32,
                            clicked_fn=self._clear_link_stage,
                            enabled=True,
                        )
                        ui.Button(
                            "Copy ALL",
                            width=50,
                            height=32,
                            clicked_fn=self._copy_all_link_to_stage,
                            enabled=True,
                        )
                    ui.Spacer(height=5)
                    with ui.HStack(height=24):
                        self._parent_button = ButtonWithIcon(
                            "Parent",
                            name="up",
                            clicked_fn=lambda: self._parent_button_clicked(),
                            height=24,
                            enabled=False,
                        )
                        ui.Spacer(width=5)
                        self._unparent_button = ButtonWithIcon(
                            "Unparent",
                            name="down",
                            clicked_fn=lambda: self._unparent_button_clicked(),
                            height=24,
                            enabled=False,
                        )

                    ui.Spacer(height=20)
                    with ui.ZStack(height=self._initial_robot_window_height):
                        with ui.VStack():
                            self._robot_stage_widget = StageWidget(
                                None, columns_enabled=[], stage_delegate=RobotStageDelegate()
                            )
                            self._robot_stage_widget._tree_view.header_visible = False
                            # self._robot_stage_widget.filter_by_type(
                            #     [UsdGeom.Mesh, UsdGeom.Sphere, UsdGeom.Capsule, UsdGeom.Cylinder, UsdGeom.Cone],
                            #     True,
                            # )
                        self._robot_splitter = ui.Placer(
                            offset_y=self._initial_robot_window_height,
                            drag_axis=ui.Axis.Y,
                            draggable=True,
                        )
                        with self._robot_splitter:
                            with ui.ZStack(height=4):
                                ui.Rectangle(name="splitter_highlight")

                        self._robot_splitter.set_offset_y_changed_fn(self._on_robot_splitter_dragged)

                    # # NOTE: not attaching either stage for visualization this time around, what's showing in the viewport should be the original stage

                    ## Next Steps
                    ui.Spacer(height=20)
                    separator("Next: Apply New Link Structure and Add Colliders")
                    ui.Spacer(height=10)
                    self._next_step_button = ButtonWithIcon(
                        "Add Colliders",
                        name="next",
                        tooltip="Apply the new link structure and move to adding colliders",
                        clicked_fn=lambda: next_step(
                            "Robot Hierarchy", "Add Colliders", verify_fn=self._apply_new_structure
                        ),
                        height=44,
                        image_width=18,
                        enabled=True,
                    )

        self._populate_link_stage()
        self._populate_robot_stage()

    def _on_link_splitter_dragged(self, position_y: object) -> None:
        """Handle dragging of the link splitter to adjust the link stage window height.

        Args:
            position_y: The new Y position of the splitter.
        """
        self._links_splitter.offset_y = max(self._initial_link_window_height, position_y.value)

    def _on_robot_splitter_dragged(self, position_y: object) -> None:
        """Handle dragging of the robot splitter to adjust the robot stage window height.

        Args:
            position_y: The new Y position of the splitter.
        """
        self._robot_splitter.offset_y = max(self._initial_robot_window_height, position_y.value)

    def _clear_link_stage(self) -> None:
        """Clear the link stage and create a new empty stage with the robot prim if available."""
        self._new_link_stage()
        if self._robot:
            UsdGeom.Xform.Define(self._links_temp_stage, f"/{self._robot.name}")

    def _copy_all_link_to_stage(self) -> None:
        """Copy all links from the robot temp stage to a new link stage."""
        self._new_link_stage()
        if self._robot_temp_stage and self._robot:
            robot_prim = self._robot_temp_stage.GetPrimAtPath(Sdf.Path(self._robot.parent_prim_path))
            copy_prim_hierarchy(robot_prim, self._links_temp_stage, Sdf.Path(self._robot.parent_prim_path))

    def _populate_link_stage(self) -> None:
        """Populate the link stage with the robot prim and registered links.

        Returns:
            None if no robot is registered.
        """
        if not self._robot:
            return
        self._new_link_stage()
        UsdGeom.Xform.Define(self._links_temp_stage, f"/{self._robot.name}")
        for link in self._robot.links:
            UsdGeom.Xform.Define(self._links_temp_stage, f"/{self._robot.name}/{link}")

    def _new_link_stage(self) -> None:
        """Helper method to set up the selection watch for the links stage."""
        self._links_temp_stage = Usd.Stage.CreateInMemory()
        self._links_stage_widget.open_stage(self._links_temp_stage)
        self._links_selection_watch = SelectionWatch(
            stage=weakref.ref(self._links_temp_stage),
            on_selection_changed_fn=self._on_links_selection_changed,
        )
        self._links_stage_widget.set_selection_watch(self._links_selection_watch)

    def _populate_robot_stage(self) -> None:
        """Populate the robot stage by copying the selected robot prim to a temporary stage.

        Returns:
            None if no robot is registered.
        """
        # make a copy of the selected robot prim on this stage, copy only path names, no attributes
        if not self._robot:
            return

        # create a temporary stage with the current robot prim on stage
        self._robot_temp_stage = Usd.Stage.CreateInMemory()

        # copy the robot prim from the current stage to the temporary stage
        robot_prim = omni.usd.get_context().get_stage().GetPrimAtPath(Sdf.Path(self._robot.parent_prim_path))
        if robot_prim and robot_prim.IsValid():
            copy_prim_hierarchy(robot_prim, self._robot_temp_stage, Sdf.Path(self._robot.parent_prim_path))

        self._robot_stage_widget.open_stage(self._robot_temp_stage)

        self._robot_selection_watch = SelectionWatch(
            stage=weakref.ref(self._robot_temp_stage),
            on_selection_changed_fn=self._on_robot_selection_changed,
        )
        self._robot_stage_widget.set_selection_watch(self._robot_selection_watch)

    def _attach_stage(self, stage: object) -> None:
        """Attaches a USD stage to the current USD context asynchronously.

        Args:
            stage: The USD stage to attach to the context.
        """
        usd_context = omni.usd.get_context()
        # it will refresh the default kit stage, is this expected?
        asyncio.ensure_future(
            usd_context.attach_stage_async(stage)
        )  ## seems necessary for the copy and clear buttons to work

    def set_visible(self, visible: bool) -> None:
        """Sets the visibility of the robot hierarchy widget.

        Args:
            visible: Whether the widget should be visible.
        """
        if self.frame:
            self.frame.visible = visible
            if visible:
                self._robot = RobotRegistry().get()

    def _add_link_item(self, link_name: str = None) -> None:
        """Opens a dialog to add a new link to the robot hierarchy.

        Args:
            link_name: Initial name for the link.

        Returns:
            None if the links temp stage is not available.
        """
        if not self._links_temp_stage:
            return

        def _on_add_link_ok(d: Any) -> None:
            # get the link name from the popup window
            link_name = d.get_value("link_name")
            # if not Sdf.Path.IsValidIdentifier(link_name):
            #     print(f"Invalid link name: {link_name}")
            #     return
            try:
                UsdGeom.Xform.Define(self._links_temp_stage, Sdf.Path(link_name))
                self._link_popup_window.hide()
            except Exception as e:
                carb.log_error(f"Error adding link, Check if the link name is valid: {e}")
                return

        def _on_add_link_cancel(d: Any) -> None:
            self._link_popup_window.hide()

        if self._links_selected:
            default_link_name = self._links_selected[0]
        elif self._robot:
            default_link_name = f"/{self._robot.name}/"
        else:
            default_link_name = "/Robot"

        # get the link name from the popup window
        field_defs = [
            FormDialog.FieldDef("link_name", "Link Name:  ", ui.StringField, default_link_name),
        ]
        self._link_popup_window = FormDialog(
            message="Add Link",
            ok_handler=_on_add_link_ok,
            cancel_handler=_on_add_link_cancel,
            field_defs=field_defs,
        )
        self._link_popup_window.show()

    def _delete_link_item(self) -> None:
        """Deletes the currently selected links from the temporary stage.

        Returns:
            None if no links are selected.
        """
        if len(self._links_selected) == 0:
            return

        for path in self._links_selected:
            self._links_temp_stage.RemovePrim(Sdf.Path(path))

    def _parent_button_clicked(self) -> None:
        """Handles parenting selected robot objects to the selected link in the new hierarchy."""
        step_path = self._step_selected
        link_path = self._links_selected[0]
        for step in step_path:
            step_prim = self._robot_temp_stage.GetPrimAtPath(Sdf.Path(step))
            target_path = link_path + "/" + step.split("/")[-1]
            # add it to the link_temp_stage
            copy_prim_hierarchy(step_prim, self._links_temp_stage, Sdf.Path(target_path))
            self._history[step] = target_path
            # delete it from the robot_temp_stage
            self._robot_temp_stage.RemovePrim(Sdf.Path(step))

    def _unparent_button_clicked(self) -> None:
        """Handles unparenting selected links by moving them back to the robot stage and removing them from history."""
        # need to remove the link from history
        for link_selected in self._links_selected:
            # Find the link path (key) that corresponds to the selected link (value)
            path_to_remove = []

            def check_link_and_children(link_path: Any) -> None:
                # Check if this link is directly in history
                for key, value in self._history.items():
                    if link_path == value:
                        if key not in path_to_remove:
                            path_to_remove.append(key)

                # Check children recursively
                prim = self._links_temp_stage.GetPrimAtPath(Sdf.Path(link_path))
                if prim and prim.IsValid():
                    # If we found a match for this prim, don't check its children
                    if any(link_path == self._history[key] for key in self._history):
                        return

                    # Otherwise check all children
                    for child in prim.GetChildren():
                        child_path = str(child.GetPath())
                        check_link_and_children(child_path)

            # Start recursive check with the selected link
            check_link_and_children(link_selected)

            for path in path_to_remove:
                # add it back to the robot stage
                UsdGeom.Xform.Define(self._robot_temp_stage, path)
                # delete it from the history
                self._history.pop(path)

        # delete them from the link stage
        self._delete_link_item()

    def _on_links_selection_changed(self, selection: list) -> None:
        """Handles selection changes in the links stage widget.

        Args:
            selection: List of selected link paths in the links stage.
        """
        self._links_selected = selection
        self._update_parent_button_enabled()

    def _update_parent_button_enabled(self) -> None:
        """Updates the enabled state of parent and unparent buttons based on current selections."""
        if len(self._links_selected) > 0 and len(self._step_selected) > 0:
            self._parent_button.enabled = True
            self._unparent_button.enabled = True
        else:
            self._parent_button.enabled = False
            self._unparent_button.enabled = False

    def _on_robot_selection_changed(self, selection: list) -> None:
        """Handles selection changes in the robot stage widget.

        Args:
            selection: List of selected object paths in the robot stage.
        """
        self._step_selected = selection
        self._update_parent_button_enabled()

    def _apply_new_structure(self) -> None:
        """Applies the new link structure to the robot by updating links and applying the hierarchy changes.

        Returns:
            None if no robot is registered or no history of changes exists.
        """
        self._robot = RobotRegistry().get()

        if not self._robot or not self._history:
            return

        # add the links to the robot
        self._robot.links = [
            link.GetName() for link in self._links_temp_stage.GetPrimAtPath(f"/{self._robot.name}").GetChildren()
        ]
        # check if robot has the reference mesh property
        if hasattr(self._robot, "reference_mesh"):
            self._reference_mesh = self._robot.reference_mesh
        else:
            self._reference_mesh = {}

        if hasattr(self._robot, "delete_prim_paths"):
            self._delete_prim_paths = self._robot.delete_prim_paths
        else:
            self._delete_prim_paths = []

        apply_hierarchy(self._history, self._reference_mesh, self._delete_prim_paths)
