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
import copy
import gc
import os
import weakref
from functools import partial
from typing import OrderedDict

import carb
import numpy as np
import omni
import omni.kit.commands
import omni.physx as _physx
import omni.timeline
import omni.ui as ui
import omni.usd
import yaml
from isaacsim.core.prims import Articulation, SingleArticulation, SingleXFormPrim
from isaacsim.core.utils.articulations import find_all_articulation_base_paths
from isaacsim.core.utils.numpy.rotations import quats_to_rot_matrices
from isaacsim.core.utils.prims import get_prim_at_path, get_prim_object_type

# New way of making UI being integrated in through feature updates
from isaacsim.gui.components.element_wrappers import (
    Button,
    CheckBox,
    CollapsableFrame,
    DropDown,
    FloatField,
    ScrollingWindow,
)
from isaacsim.gui.components.menu import make_menu_item_description

# Old way of making UI
from isaacsim.gui.components.ui_utils import (
    add_line_rect_flourish,
    btn_builder,
    color_picker_builder,
    float_builder,
    get_style,
    int_builder,
    setup_ui_headers,
    state_btn_builder,
    str_builder,
    xyz_builder,
)
from isaacsim.gui.components.widgets import DynamicComboBoxModel
from omni.kit.menu.utils import MenuItemDescription, add_menu_items, remove_menu_items
from omni.kit.window.property.templates import LABEL_WIDTH
from pxr import Usd, UsdGeom, UsdPhysics

from .collision_sphere_editor import CollisionSphereEditor

EXTENSION_NAME = "Lula Robot Description Editor"

DEFAULT_JERK_LIMIT = 10000
DEFAULT_ACCELERATION_LIMIT = 10
MAX_DOF_NUM = 100


def is_yaml_file(path: str):
    _, ext = os.path.splitext(path.lower())
    return ext in [".yaml", ".yml"]


def is_xrdf_file(path: str):
    _, ext = os.path.splitext(path.lower())
    return ext in [".yaml", ".yml", ".xrdf"]


def on_filter_xrdf_item(item) -> bool:
    if not item or item.is_folder:
        return not (item.name == "Omniverse" or item.path.startswith("omniverse:"))
    return is_xrdf_file(item.path)


def on_filter_item(item) -> bool:
    if not item or item.is_folder:
        return not (item.name == "Omniverse" or item.path.startswith("omniverse:"))
    return is_yaml_file(item.path)


class Extension(omni.ext.IExt):
    def on_startup(self, ext_id: str):
        """Initialize extension and UI elements"""

        # Events
        self._usd_context = omni.usd.get_context()
        self._physxIFace = _physx.get_physx_interface()
        self._physx_subscription = None
        self._stage_event_sub = None
        self._timeline = omni.timeline.get_timeline_interface()

        # Build Window
        self._window = ScrollingWindow(
            title=EXTENSION_NAME, width=600, height=500, visible=False, dockPreference=ui.DockPreference.LEFT_BOTTOM
        )
        self._window.set_visibility_changed_fn(self._on_window)

        # UI
        self._models = {}
        self._ext_id = ext_id
        menu_entry = [
            make_menu_item_description(ext_id, EXTENSION_NAME, lambda a=weakref.proxy(self): a._menu_callback())
        ]
        self._menu_items = [MenuItemDescription(name="Robotics", sub_menu=menu_entry)]
        add_menu_items(self._menu_items, "Tools")

        # Selection
        self._new_window = True
        self.new_selection = True
        self._selected_index = None
        self._selected_prim_path = None
        self._prev_art_prim_path = None

        # Articulation
        self._articulation_base_path = None
        self.articulation = None
        self.num_dof = 0
        self.dof_names = []
        self.upper_joint_limits = np.zeros(MAX_DOF_NUM)
        self.lower_joint_limits = np.zeros(MAX_DOF_NUM)

        # Animation
        self._set_joint_positions_on_step = False

        # Sphere generation
        self._selected_link = None
        self._sphere_gen_link_2_mesh = OrderedDict()
        self._preview_spheres = True

        # Connect Spheres
        self._connect_sphere_0_options = []
        self._connect_sphere_1_options = []

        # Link Visibility
        self._hiding_link = False
        self._hiding_robot = False
        self._prev_link = None

        # Active Joints
        self._joint_positions = np.zeros(MAX_DOF_NUM)
        self._active_joints = np.zeros(MAX_DOF_NUM, dtype=bool)
        self._acceleration_limits = np.full(MAX_DOF_NUM, DEFAULT_ACCELERATION_LIMIT)
        self._jerk_limits = np.full(MAX_DOF_NUM, DEFAULT_JERK_LIMIT)

        self._collision_sphere_editor = CollisionSphereEditor()

    def on_shutdown(self):
        self._show_robot_if_hidden()
        self._collision_sphere_editor.on_shutdown()
        self._usd_context = None
        self._stage_event_sub = None
        self._timeline_event_sub = None
        self._physx_subscription = None
        self._models = {}
        remove_menu_items(self._menu_items, "Tools")
        if self._window:
            self._window = None
        gc.collect()

    def _on_window(self, visible):
        if self._window.visible:
            # Subscribe to Stage and Timeline Events
            self._usd_context = omni.usd.get_context()
            events = self._usd_context.get_stage_event_stream()
            self._stage_event_sub = events.create_subscription_to_pop(self._on_stage_event)
            stream = self._timeline.get_timeline_event_stream()
            self._timeline_event_sub = stream.create_subscription_to_pop(self._on_timeline_event)

            self._build_ui()
            if not self._new_window and self.articulation:
                self._refresh_ui(self.articulation)
            self._new_window = False
        else:
            self._usd_context = None
            self._stage_event_sub = None
            self._timeline_event_sub = None

    def _menu_callback(self):
        self._window.visible = not self._window.visible
        # Update the Selection Box if the Timeline is already playing
        if not self._timeline.is_stopped():
            self._refresh_selection_combobox()
            self._on_selection(self._get_selected_articulation())

    def _build_ui(self):
        # if not self._window:
        with self._window.frame:
            with ui.VStack(spacing=5, height=0):

                self._build_info_ui()

                self._build_selection_ui()

                self._build_command_ui()

                self._build_editor_ui()

                self._build_tools_ui()

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

    def _on_selection(self, prim_path):
        """Creates an Articulation Object from the selected articulation prim path.
           Updates the UI with the Selected articulation.

        Args:
            prim_path (string): path to selected articulation
        """
        if prim_path == self._prev_art_prim_path:
            return
        elif not self._timeline.is_stopped():
            self._prev_art_prim_path = prim_path

        self.new_selection = True
        self._prev_link = None

        if self.articulation_list and prim_path != "None" and not self._timeline.is_stopped():
            # Create and Initialize the Articulation
            self._articulation_base_path = prim_path
            self.articulation = SingleArticulation(prim_path)

            if not self.articulation.handles_initialized:
                self.articulation.initialize()

            # Get list of all links and populate link selection combobox
            self.get_all_sphere_gen_meshes()
            self._refresh_sphere_gen_link_combobox()

            # Update the entire UI with the selected articulaiton
            self._refresh_ui(self.articulation)

            # start event subscriptions
            if not self._physx_subscription:
                self._physx_subscription = self._physxIFace.subscribe_physics_step_events(self._on_physics_step)

        # Deselect and Reset
        else:
            if self.articulation is not None:
                self._show_robot_if_hidden()
                self._reset_ui()
                self._refresh_selection_combobox()
            self._articulation_base_path = None
            self.articulation = None

    def _on_combobox_selection(self, model=None, val=None):
        index = self._models["ar_selection_model"].get_item_value_model().as_int
        if index >= 0 and index < len(self.articulation_list):
            self._selected_index = index
            item = self.articulation_list[index]
            self._selected_prim_path = item
            self._on_selection(item)

    def _refresh_selection_combobox(self):
        self.articulation_list = find_all_articulation_base_paths()
        self.articulation_list.insert(0, "None")
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

    def _clear_selection_combobox(self):
        self._selected_index = None
        self._selected_prim_path = None
        self.articulation_list = []
        self._models["ar_selection_model"] = DynamicComboBoxModel(self.articulation_list)
        self._models["ar_selection_combobox"].model = self._models["ar_selection_model"]
        self._models["ar_selection_combobox"].model.add_item_changed_fn(self._on_combobox_selection)

    def _clear_link_selection_combobox(self):
        self._sphere_gen_link_2_mesh = OrderedDict()
        self._models["sphere_gen_link_selection_model"] = DynamicComboBoxModel([])
        self._models["sphere_gen_link_selection_model_combobox"].model = self._models["sphere_gen_link_selection_model"]
        self._models["sphere_gen_link_selection_model_combobox"].model.add_item_changed_fn(
            self._on_select_sphere_gen_link
        )

    def _on_select_sphere_gen_link(self, model, val):
        index = model.get_item_value_model().as_int
        if len(self._sphere_gen_link_2_mesh.keys()) == 0:
            carb.log_error("No links could be identified for this asset. This is possibly because no meshes exist.")
            return
        item = list(self._sphere_gen_link_2_mesh.keys())[index]
        self._selected_sphere_gen_link = item
        self._models["sphere_gen_mesh_selection_model"] = DynamicComboBoxModel(self._sphere_gen_link_2_mesh[item])
        self._models["sphere_gen_mesh_selection_model_combobox"].model = self._models["sphere_gen_mesh_selection_model"]
        self._models["sphere_gen_mesh_selection_model"].add_item_changed_fn(
            self._trigger_preview_generate_spheres_for_link
        )
        self._generate_spheres_for_link()

        self._refresh_collision_sphere_comboboxes()

        self._collision_sphere_editor.set_sphere_colors(self._get_selected_link_path())

        if self._hiding_link != self._hiding_robot:
            self._hide_link(self._get_selected_link())
            if self._prev_link is not None:
                self._hide_link(self._prev_link)

        self._prev_link = self._get_selected_link()

    def _refresh_sphere_gen_link_combobox(self):
        self._models["sphere_gen_link_selection_model"] = DynamicComboBoxModel(
            list(self._sphere_gen_link_2_mesh.keys())
        )
        self._models["sphere_gen_link_selection_model_combobox"].model = self._models["sphere_gen_link_selection_model"]
        self._models["sphere_gen_link_selection_model_combobox"].model.add_item_changed_fn(
            self._on_select_sphere_gen_link
        )
        self._on_select_sphere_gen_link(self._models["sphere_gen_link_selection_model"], None)

        self._refresh_collision_sphere_comboboxes()

    def _refresh_collision_sphere_comboboxes(self, keep_sphere_selection=False):
        sphere_0_name, _ = self._get_selected_collision_spheres()

        sphere_names = self._collision_sphere_editor.get_sphere_names_by_link(self._get_selected_link_path())
        self._connect_sphere_0_options = sphere_names
        self._models["connect_sphere_selection_0"] = DynamicComboBoxModel(sphere_names)
        self._models["connect_sphere_selection_0_combobox"].model = self._models["connect_sphere_selection_0"]
        self._models["connect_sphere_selection_0"].add_item_changed_fn(self._on_collision_sphere_select_0)

        # Keep currently selected collision sphere when reloading if it still exists
        if keep_sphere_selection and sphere_0_name in sphere_names:
            self._models["connect_sphere_selection_0"].get_item_value_model().set_value(
                int(sphere_names.index(sphere_0_name))
            )

        self._on_collision_sphere_select_0(None, None)

    def _on_collision_sphere_select_0(self, model, val):
        sphere_0_name, sphere_1_name = self._get_selected_collision_spheres()
        if sphere_0_name is not None:
            sphere_names = self._connect_sphere_0_options
            pruned_names = sphere_names[:]  # shallow copy
            pruned_names.pop(sphere_names.index(sphere_0_name))
        else:
            pruned_names = []

        self._connect_sphere_1_options = pruned_names

        name = "connect_sphere_selection_1"
        self._models[name] = DynamicComboBoxModel(pruned_names)
        self._models[name + "_combobox"].model = self._models[name]

        if sphere_1_name in pruned_names:
            self._models[name].get_item_value_model().set_value(int(pruned_names.index(sphere_1_name)))

    def get_all_sphere_gen_meshes(self):
        stage = self._usd_context.get_stage()
        self._sphere_gen_link_2_mesh = OrderedDict()

        if self.articulation is None:
            return

        links = self.articulation._articulation_view.body_names

        if len(links) == 0:
            carb.log_error(
                "No links can be found as part of the selected Articulation.  Make sure "
                + "that the selected articulation has at least one joint."
            )
            return

        num_art_path_components = len(self._articulation_base_path.split("/"))
        art_path_len = len(self._articulation_base_path)
        for prim in Usd.PrimRange(stage.GetPrimAtPath(self._articulation_base_path), Usd.TraverseInstanceProxies()):
            path = str(prim.GetPath())

            type = get_prim_object_type(path)

            if type == "xform":
                geom_mesh = UsdGeom.Mesh(prim)
                if geom_mesh.GetPointsAttr().HasValue():
                    is_instanced = prim.IsInstanceProxy()

                    # Check which link this mesh belongs to.
                    link_subpath = None

                    # Find the length of the path of the link.
                    link_path_len = art_path_len
                    for s in path.split("/")[num_art_path_components:]:
                        # Accumulate the length of link path including path components and "/" that
                        # were removed when splitting.
                        link_path_len += 1 + len(s)
                        if s in links:
                            # link_path is the subpath of the link nested under the articulation
                            # base path.
                            link_subpath = path[art_path_len:link_path_len]
                            break

                    if link_subpath is not None and is_instanced:
                        # Do not include instanceable meshes in the meshes under a link because these
                        # cannot be used to generate spheres.  However, still make sure that any link
                        # that contains an instanceable mesh shows up as a key in
                        # self._sphere_gen_link_2_mesh so that the user can still author spheres by
                        # hand.
                        carb.log_warn(
                            f"Found instanceable mesh at path {path}.  Instanceable meshes are not fully "
                            + "compatible with the Robot Description Editor.  They cannot be used to generate spheres "
                            + "automatically.  You may author spheres by hand or stop the timeline and "
                            + "uncheck 'Instanceable' in the Properties panel for this path and all parent "
                            + "paths."
                        )

                        if link_subpath not in self._sphere_gen_link_2_mesh.keys():
                            self._sphere_gen_link_2_mesh[link_subpath] = []
                    elif link_subpath is not None:
                        mesh_subpath = path[link_path_len:]
                        l = self._sphere_gen_link_2_mesh.get(link_subpath, [])
                        l.append(mesh_subpath)
                        self._sphere_gen_link_2_mesh[link_subpath] = l
                    else:
                        carb.log_warn(
                            f"The mesh at path {path} was not determined to be a part of "
                            + f"any link in the Articulation {self._articulation_base_path}"
                        )

    def get_articulation_values(self, articulation):
        """Get and store the latest dof_properties from the articulation.
           Update the Properties UI.

        Args:
            articulation (SingleArticulation): Selected Articulation
        """
        # Update static dof properties on new selection
        if self.new_selection:
            self.num_dof = articulation.num_dof
            self.dof_names = articulation.dof_names
            self.new_selection = False

            self._joint_positions = articulation.get_joint_positions()
            self._active_joints = np.zeros(MAX_DOF_NUM, dtype=bool)
            self._acceleration_limits = np.full(MAX_DOF_NUM, DEFAULT_ACCELERATION_LIMIT)
            self._jerk_limits = np.full(MAX_DOF_NUM, DEFAULT_JERK_LIMIT)

            self.lower_joint_limits = articulation.dof_properties["lower"]
            self.upper_joint_limits = articulation.dof_properties["upper"]

    def _refresh_ui(self, articulation):
        """Updates the GUI with a new Articulation's properties.

        Args:
            articulation (SingleArticulation): [description]
        """
        # Get the latest articulation values and update the Properties UI
        self.get_articulation_values(articulation)

        self._update_editor_ui()

        self._models["frame_command_ui"].collapsed = False
        self._models["frame_command_ui"].enabled = True

        self._models["sphere_editor_ui"].collapsed = False
        self._models["sphere_editor_ui"].enabled = True

        self._models["editor_tools_ui"].collapsed = False
        self._models["editor_tools_ui"].enabled = True

        self._models["save_spheres_ui"].enabled = True

        self._models["load_spheres_ui"].enabled = True

        self._update_command_ui()

    def _reset_ui(self):
        self._show_robot_if_hidden()

        """Reset / Hide UI Elements."""
        self._clear_selection_combobox()
        self._clear_link_selection_combobox()
        self._prev_art_prim_path = None

        # Reset & Disable Button
        self._models["frame_command_ui"].collapsed = True
        self._models["frame_command_ui"].enabled = False

        for joint_frame in self._joint_frames:
            joint_frame.rebuild()

        self._models["sphere_editor_ui"].collapsed = True
        self._models["sphere_editor_ui"].enabled = False

        self._models["editor_tools_ui"].collapsed = True
        self._models["editor_tools_ui"].enabled = False

        self._models["save_spheres_ui"].collapsed = True
        self._models["save_spheres_ui"].enabled = False

        self._models["load_spheres_ui"].collapsed = True
        self._models["load_spheres_ui"].enabled = False

    ##################################
    # Callbacks
    ##################################

    def _on_stage_event(self, event):
        """Callback for Stage Events

        Args:
            event (omni.usd.StageEventType): Event Type
        """

        # On every stage event check if any articulations have been added/removed from the Stage
        self._refresh_selection_combobox()

        if event.type == int(omni.usd.StageEventType.SELECTION_CHANGED):
            self._collision_sphere_editor.copy_all_sphere_data()
            self._refresh_collision_sphere_comboboxes(keep_sphere_selection=True)
            pass

        elif event.type == int(omni.usd.StageEventType.OPENED) or event.type == int(omni.usd.StageEventType.CLOSED):
            # stage was opened or closed, cleanup
            self._physx_subscription = None

        elif event.type == int(omni.usd.StageEventType.SIMULATION_START_PLAY):  # Timeline played
            self._refresh_selection_combobox()
            self._on_selection(self._get_selected_articulation())

        elif event.type == int(omni.usd.StageEventType.SIMULATION_STOP_PLAY):  # Timeline stopped
            if self._timeline.is_stopped():
                self._reset_ui()
                self._on_selection("None")

    def _on_physics_step(self, step):
        """Callback for Physics Step.

        Args:
            step ([type]): [description]
        """
        if self.articulation is not None:
            if not self.articulation.handles_initialized:
                self.articulation.initialize()
            # Get the latest values from the articulation
            self.get_articulation_values(self.articulation)

            # Handle animation
            if self._set_joint_positions_on_step:
                self._set_joint_positions(step)
        return

    def _on_timeline_event(self, e):
        """Callback for Timeline Events

        Args:
            event (omni.timeline.TimelineEventType): Event Type
        """

        pass

    def _set_joint_positions(self, step):
        if self.articulation is not None:
            joint_velocities = np.zeros_like(self._joint_positions)
            self.articulation.set_joint_positions(self._joint_positions)
            self.articulation.set_joint_velocities(joint_velocities)
        self._set_joint_positions_on_step = False
        return

    ##################################
    # UI Builders
    ##################################

    def _build_info_ui(self):
        title = EXTENSION_NAME
        doc_link = (
            "https://docs.isaacsim.omniverse.nvidia.com/latest/manipulators/manipulators_robot_description_editor.html"
        )

        overview = "This utility is used to help generate a Lula Robot Description YAML file required to use Lula-based algorithms like RmpFlow, RRT, and Lula Kinematics, or to generate an XRDF file for use with Isaac cuMotion or future releases of Lula. "
        overview += "Both file types contain a collision sphere representation of the robot that is used for collision avoidance, and information that is required to interpret the robot URDF.\n\n"

        overview += (
            "To begin using this editor, load a robot USD file onto the stage and press the 'Play' button.  In the 'Selection Panel', select your robot from the 'Select Articulation' drop-down menu.  "
            + "The 'Select Link' drop-down menu will populate once an Articulation has been selected.  The user may create collision spheres for the robot one link at a time. \n\n"
        )

        overview += (
            "Joint Properties Panel:\nIn the Joint Properties Panel, the user may select the default positions of robot joints and choose a subset of joints that are considered 'Active Joints'. "
            + "'Active Joints' are considered to be directly controllable, while 'Fixed Joints' are assumed to never move. "
            + "The default positions that 'Active Joints' are set to are used by Lula algorithms to resolve null-space behavior.  For example, RmpFlow is typically configured to control "
            + "only the joints in a robot arm, and assume the gripper to be in a fixed position.  While moving the gripper to a target, it will choose a path that moves the robot close to"
            + " the default 'Active Joints' configuration.  By default, all joints are marked as 'Fixed Joints', which will cause Lula not to control the robot at all.  The user must determine"
            + "a set of joints that should be considered 'Active'.  Maximum jerk and accelerations are required to be specified for each active joint in the robot.\n\n"
        )

        overview += (
            "Adding Collision Spheres:\nIn the 'Link Sphere Editor' panel paired with 'Editor Tools', the user may add collision spheres on a per-link basis.  Spheres are added with positions specified relative to the base of the "
            + "selected link, with their position relative to the link being fixed.  Once a sphere has been created, the user may move it around, resize or delete it on the USD stage until it looks right.  "
            + "Additionally, the user may generate spheres for a link automatically or select any two spheres under a link and linearly interpolate to create more spheres connecting them.  In general,"
            + " the user will want to fully cover the robot in spheres, using around 40-60 spheres total.  It is easiest to create such a set of spheres when individual spheres are allowed to slightly exceed"
            + " the volume of the robot. \n\n"
        )

        overview += (
            "Importing and Exporting:\nLula robot description YAML files and cuMotion XRDF files are both supported file types for importing and exporting data from the Robot Description Editor. "
            + "The Robot Description Editor does not represent every possible field in an XRDF file, and so when exporting to an existing XRDF file path, "
            + "the user will have an option to pull data from the existing file that should not be overwritten by leaving the default setting to 'Merge With Existing XRDF'."
        )

        setup_ui_headers(self._ext_id, __file__, title, doc_link, overview)

    def _build_selection_ui(self):
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

                name = "sphere_gen_link_selection_model"
                self._models[name] = DynamicComboBoxModel(list(self._sphere_gen_link_2_mesh.keys()))

                with ui.HStack():
                    ui.Label(
                        "Select Link",
                        width=LABEL_WIDTH,
                        alignment=ui.Alignment.LEFT_CENTER,
                        tooltip="Select under which to generate spheres.  Only links with nested meshes can be chosen",
                    )
                    self._models[name + "_combobox"] = ui.ComboBox(self._models[name])
                    add_line_rect_flourish(False)

                self._models[name + "_combobox"].model.add_item_changed_fn(self._on_select_sphere_gen_link)

    def _build_command_ui(self):
        def command_panel_build_fn():
            self._joint_frames = []

            if self.articulation is None:
                return

            def on_set_joint_position(i, value):
                self._joint_positions[i] = value
                self._set_joint_positions_on_step = True

            def on_max_acceleration_changed(i, value):
                self._acceleration_limits[i] = value

            def on_max_jerk_changed(i, value):
                self._jerk_limits[i] = value

            def update_active_joints(i, value):
                if value == "Active Joint":
                    is_active = True
                else:
                    is_active = False
                self._active_joints[i] = is_active

                self._joint_frames[i].rebuild()

            def joint_frame_build_fn(i):
                if i >= self.articulation.num_dof:
                    return

                lower_joint_limit = self.articulation.dof_properties["lower"][i]
                upper_joint_limit = self.articulation.dof_properties["upper"][i]
                with ui.VStack(style=get_style(), spacing=5, height=0):
                    position_field = FloatField(
                        "Joint Position",
                        default_value=self._joint_positions[i],
                        tooltip="If an active joint, this indicates a default position.  If a fixed joint, this indicates a fixed position.",
                        on_value_changed_fn=partial(on_set_joint_position, i),
                        lower_limit=lower_joint_limit,
                        upper_limit=upper_joint_limit,
                    )

                    if self._active_joints[i]:
                        acceleration_field = FloatField(
                            "Acceleration Limit",
                            tooltip="Maximum acceleration that can be commanded for this joint.",
                            default_value=self._acceleration_limits[i],
                            lower_limit=0.0001,
                            on_value_changed_fn=partial(on_max_acceleration_changed, i),
                        )
                        jerk_field = FloatField(
                            "Jerk Limit",
                            tooltip="Maximum jerk that can be commanded for this joint.",
                            default_value=self._jerk_limits[i],
                            lower_limit=0.0001,
                            step=1.0,
                            on_value_changed_fn=partial(on_max_jerk_changed, i),
                        )

                    joint_status = DropDown(
                        "Joint Status",
                        tooltip="Active Joint: Lula will directly control this joint, using a default position equal to the value set above.\n"
                        + "Fixed Joint: Lula will assume a fixed position of the joint equal to the value set above.",
                        populate_fn=lambda: ["Fixed Joint", "Active Joint"],
                    )
                    joint_status.repopulate()
                    joint_status.set_selection_by_index(self._active_joints[i])

                    joint_status.set_on_selection_fn(partial(update_active_joints, i))

            with ui.VStack(style=get_style(), spacing=5, height=0):
                for i in range(self.articulation.num_dof):
                    frame = CollapsableFrame(
                        self.articulation.dof_names[i], build_fn=partial(joint_frame_build_fn, i), collapsed=False
                    )
                    self._joint_frames.append(frame)

        self._models["frame_command_ui"] = CollapsableFrame("Set Joint Properties", build_fn=command_panel_build_fn)
        self._models["frame_command_ui"].enabled = False

    def _build_editor_ui(self):
        self._models["sphere_editor_ui"] = ui.CollapsableFrame(
            title="Link Sphere Editor",
            height=0,
            collapsed=True,
            style=get_style(),
            name="editorFrame",
            horizontal_scrollbar_policy=ui.ScrollBarPolicy.SCROLLBAR_AS_NEEDED,
            vertical_scrollbar_policy=ui.ScrollBarPolicy.SCROLLBAR_ALWAYS_ON,
        )
        self._models["sphere_editor_ui"].enabled = False

        with self._models["sphere_editor_ui"]:
            with ui.VStack(style=get_style(), spacing=5, height=0):

                ###################################################################
                #                          Generate Spheres
                ###################################################################

                frame = ui.CollapsableFrame(
                    title="Generate Spheres",
                    name="subFrame",
                    height=0,
                    collapsed=True,
                    style_type_name_override="CollapsableFrame",
                    horizontal_scrollbar_policy=ui.ScrollBarPolicy.SCROLLBAR_AS_NEEDED,
                    vertical_scrollbar_policy=ui.ScrollBarPolicy.SCROLLBAR_ALWAYS_ON,
                )
                self._models["sphere_generator_ui"] = frame
                with frame:
                    with ui.VStack(style=get_style(), spacing=5, height=0):
                        num_sphere_kwargs = {
                            "label": "Number of Spheres",
                            "default_val": 0,
                            "min": 0,
                            "tooltip": "Number of Spheres to Generate for Link",
                        }

                        rad_offset_kwargs = {
                            "label": "Radius Offset",
                            "default_val": 0.01,
                            "tooltip": "Extent to which spheres may extend beyond the mesh.  A positive value means that spheres may exceed the mesh by up to the given value.\n A negative value specifies that all spheres are at least radius_offset from the mesh surface.",
                        }

                        with frame:
                            with ui.VStack(style=get_style(), spacing=5, height=0):
                                name = "sphere_gen_mesh_selection_model"
                                self._models[name] = DynamicComboBoxModel([])

                                with ui.HStack():
                                    ui.Label(
                                        "Select Mesh",
                                        width=LABEL_WIDTH,
                                        alignment=ui.Alignment.LEFT_CENTER,
                                        tooltip="Select Mesh to be Used for Sphere Generation",
                                    )
                                    self._models[name + "_combobox"] = ui.ComboBox(self._models[name])
                                    add_line_rect_flourish(False)

                                self._models["sphere_gen_num_spheres"] = int_builder(**num_sphere_kwargs)
                                self._models["sphere_gen_num_spheres"].add_value_changed_fn(
                                    self._trigger_preview_generate_spheres_for_link
                                )
                                self._models["sphere_gen_radius_offset"] = float_builder(**rad_offset_kwargs)
                                self._models["sphere_gen_radius_offset"].add_value_changed_fn(
                                    self._trigger_preview_generate_spheres_for_link
                                )

                                self._models["sphere_gen_preview"] = state_btn_builder(
                                    label="Preview Spheres",
                                    b_text="Show Preview",
                                    a_text="Hide Preview",
                                    tooltip="Show a preview of the spheres that will be generated.",
                                    on_clicked_fn=self._preview_collision_spheres,
                                )

                                def generate_spheres():
                                    self._generate_spheres_for_link(preview=False)
                                    self._refresh_collision_sphere_comboboxes(keep_sphere_selection=True)
                                    self._models["sphere_gen_num_spheres"].set_value(int(0))

                                self._models["sphere_gen_add"] = btn_builder(
                                    label="Generate Spheres",
                                    text="Generate Spheres",
                                    tooltip="Generate Spheres for Robot Link",
                                    on_clicked_fn=generate_spheres,
                                )

                ###################################################################
                #                            Add Sphere
                ###################################################################

                frame = ui.CollapsableFrame(
                    title="Add Sphere",
                    name="subFrame",
                    height=0,
                    collapsed=True,
                    style=get_style(),
                    style_type_name_override="CollapsableFrame",
                    horizontal_scrollbar_policy=ui.ScrollBarPolicy.SCROLLBAR_AS_NEEDED,
                    vertical_scrollbar_policy=ui.ScrollBarPolicy.SCROLLBAR_ALWAYS_ON,
                )

                with frame:
                    with ui.VStack(style=get_style(), spacing=5, height=0):
                        kwargs = {"label": "Radius", "default_val": 0.1, "min": 0.001, "tooltip": "Desired Radius"}
                        self._models["add_sphere_radius"] = float_builder(**kwargs)

                        kwargs = {
                            "label": "Relative Translation",
                            "tooltip": "Relative translation of sphere in the local frame of the selected Prim path.",
                            "axis_count": 3,
                            "default_val": [0.0, 0.0, 0.0],
                        }

                        val_models = xyz_builder(**kwargs)
                        self._models["add_sphere_translation_x"] = val_models[0]
                        self._models["add_sphere_translation_y"] = val_models[1]
                        self._models["add_sphere_translation_z"] = val_models[2]

                        def on_add_sphere():
                            radius = self._models["add_sphere_radius"].get_value_as_float()
                            translation = np.zeros(3)
                            translation[0] = self._models["add_sphere_translation_x"].get_value_as_float()
                            translation[1] = self._models["add_sphere_translation_y"].get_value_as_float()
                            translation[2] = self._models["add_sphere_translation_z"].get_value_as_float()
                            link_path = self._get_selected_link_path()

                            self._collision_sphere_editor.add_sphere(link_path, translation, radius)
                            self._refresh_collision_sphere_comboboxes(keep_sphere_selection=True)

                        self._models["add_sphere_btn"] = btn_builder(
                            "Add Sphere", text="Add Sphere", on_clicked_fn=on_add_sphere
                        )

                ###################################################################
                #                           Connect Spheres
                ###################################################################
                frame = ui.CollapsableFrame(
                    title="Connect Spheres",
                    name="subFrame",
                    height=0,
                    collapsed=True,
                    style=get_style(),
                    style_type_name_override="CollapsableFrame",
                    horizontal_scrollbar_policy=ui.ScrollBarPolicy.SCROLLBAR_AS_NEEDED,
                    vertical_scrollbar_policy=ui.ScrollBarPolicy.SCROLLBAR_ALWAYS_ON,
                )

                with frame:
                    with ui.VStack(style=get_style(), spacing=5, height=0):

                        name = "connect_sphere_selection_0"
                        self._models[name] = DynamicComboBoxModel([])

                        with ui.HStack():
                            ui.Label(
                                "Select Collision Sphere",
                                width=LABEL_WIDTH,
                                alignment=ui.Alignment.LEFT_CENTER,
                                tooltip="Select First Collision Sphere to Connect",
                            )
                            self._models[name + "_combobox"] = ui.ComboBox(self._models[name])
                            add_line_rect_flourish(False)
                        self._models[name + "_combobox"].model.add_item_changed_fn(self._on_collision_sphere_select_0)

                        name = "connect_sphere_selection_1"
                        self._models[name] = DynamicComboBoxModel([])

                        with ui.HStack():
                            ui.Label(
                                "Select Collision Sphere",
                                width=LABEL_WIDTH,
                                alignment=ui.Alignment.LEFT_CENTER,
                                tooltip="Select First Collision Sphere to Connect",
                            )
                            self._models[name + "_combobox"] = ui.ComboBox(self._models[name])
                            add_line_rect_flourish(False)

                        kwargs = {
                            "label": "Number of Spheres",
                            "default_val": 0,
                            "tooltip": "Create the specified number of spheres interpolated between the selected spheres",
                        }
                        self._models["connect_sphere_num"] = int_builder(**kwargs)

                        def on_connect_spheres():
                            c0, c1 = self._get_selected_collision_spheres()
                            link_path = self._get_selected_link_path()
                            if c1 is None:
                                carb.log_warn("Please select two distinct collision spheres to Connect Spheres")

                            num = self._models["connect_sphere_num"].get_value_as_int()

                            self._collision_sphere_editor.interpolate_spheres(link_path + c0, link_path + c1, num)
                            self._refresh_collision_sphere_comboboxes(keep_sphere_selection=True)

                        self._models["connect_sphere_btn"] = btn_builder(
                            "Connect Spheres", text="Connect Spheres", on_clicked_fn=on_connect_spheres
                        )
                        self._models["connect_sphere_btn"].enabled = True

                ###################################################################
                #                           Scale Spheres
                ###################################################################
                frame = ui.CollapsableFrame(
                    title="Scale Spheres in Link",
                    name="subFrame",
                    height=0,
                    collapsed=True,
                    style=get_style(),
                    style_type_name_override="CollapsableFrame",
                    horizontal_scrollbar_policy=ui.ScrollBarPolicy.SCROLLBAR_AS_NEEDED,
                    vertical_scrollbar_policy=ui.ScrollBarPolicy.SCROLLBAR_ALWAYS_ON,
                )

                with frame:
                    with ui.VStack(style=get_style(), spacing=5, height=0):
                        kwargs = {
                            "label": "Scaling Factor",
                            "default_val": 1.0,
                            "min": 0.001,
                            "tooltip": "Scaling factor for the radii of the specified spheres",
                        }
                        self._models["scale_spheres_factor"] = float_builder(**kwargs)

                        def on_scale_spheres():
                            path = self._get_selected_link_path()
                            factor = self._models["scale_spheres_factor"].get_value_as_float()

                            self._collision_sphere_editor.scale_spheres(path, factor)

                        self._models["scale_sphere_btn"] = btn_builder(
                            "Scale Spheres", text="Scale Spheres", on_clicked_fn=on_scale_spheres
                        )
                        self._models["scale_sphere_btn"].enabled = True

                ###################################################################
                #                           Clear Spheres
                ###################################################################
                frame = ui.CollapsableFrame(
                    title="Clear Spheres in Link",
                    name="subFrame",
                    height=0,
                    collapsed=True,
                    style=get_style(),
                    style_type_name_override="CollapsableFrame",
                    horizontal_scrollbar_policy=ui.ScrollBarPolicy.SCROLLBAR_AS_NEEDED,
                    vertical_scrollbar_policy=ui.ScrollBarPolicy.SCROLLBAR_ALWAYS_ON,
                )
                with frame:
                    with ui.VStack(style=get_style(), spacing=5, height=0):

                        def clear_link_spheres_fn():
                            self._collision_sphere_editor.clear_link_spheres(self._get_selected_link_path())
                            self._refresh_collision_sphere_comboboxes()

                        self._models["link_clear_btn"] = btn_builder(
                            "Clear Link Spheres", text="Clear", on_clicked_fn=clear_link_spheres_fn
                        )
                        self._models["link_clear_btn"].enabled = True

    def _build_tools_ui(self):
        self._models["editor_tools_ui"] = ui.CollapsableFrame(
            title="Editor Tools",
            height=0,
            collapsed=True,
            style=get_style(),
            name="editorFrame",
            horizontal_scrollbar_policy=ui.ScrollBarPolicy.SCROLLBAR_AS_NEEDED,
            vertical_scrollbar_policy=ui.ScrollBarPolicy.SCROLLBAR_ALWAYS_ON,
        )
        self._models["editor_tools_ui"].enabled = False

        with self._models["editor_tools_ui"]:
            with ui.VStack(style=get_style(), spacing=5, height=0):

                def on_undo():
                    self._collision_sphere_editor.undo()
                    self._refresh_collision_sphere_comboboxes()

                self._models["undo_btn"] = btn_builder("Undo", text="Undo", on_clicked_fn=on_undo)
                self._models["undo_btn"].enabled = True

                def on_redo():
                    self._collision_sphere_editor.redo()
                    self._refresh_collision_sphere_comboboxes()

                self._models["redo_btn"] = btn_builder("Redo", text="Redo", on_clicked_fn=on_redo)
                self._models["redo_btn"].enabled = True

                kwargs = {
                    "label": "Toggle Link Visibility",
                    "a_text": " Hide",
                    "b_text": "Show",
                    "tooltip": "Hide the Selected Link",
                    "on_clicked_fn": self._on_toggle_link_visible,
                }
                self._models["hide_link_btn"] = state_btn_builder(**kwargs)

                kwargs = {
                    "label": "Toggle Robot Visibility",
                    "a_text": "Hide",
                    "b_text": "Show",
                    "tooltip": "Hide the Robot",
                    "on_clicked_fn": self._on_toggle_robot_visible,
                }
                self._models["hide_robot_btn"] = state_btn_builder(**kwargs)

                def on_link_color_change(a1, a2):
                    sphere_color = []
                    for item in self._models["link_color_picker"].get_item_children():
                        val = self._models["link_color_picker"].get_item_value_model(item).get_value_as_float()
                        sphere_color.append(val)
                    sphere_color = np.array(sphere_color[:3])
                    self._collision_sphere_editor.set_sphere_colors(
                        self._get_selected_link_path(), color_in=sphere_color
                    )

                kwargs = {
                    "label": "Link Sphere Color",
                    "default_val": self._collision_sphere_editor.filter_in_sphere_color,
                    "tooltip": "Set the color of all collision spheres in the selected link",
                }
                self._models["link_color_picker"] = color_picker_builder(**kwargs)
                self._models["link_color_picker"].add_end_edit_fn(on_link_color_change)

                def on_color_change(a1, a2):
                    sphere_color = []
                    for item in self._models["color_picker"].get_item_children():
                        val = self._models["color_picker"].get_item_value_model(item).get_value_as_float()
                        sphere_color.append(val)
                    sphere_color = np.array(sphere_color[:3])
                    self._collision_sphere_editor.set_sphere_colors(
                        self._get_selected_link_path(), color_out=sphere_color
                    )

                kwargs = {
                    "label": "Base Sphere Color",
                    "default_val": self._collision_sphere_editor.filter_out_sphere_color,
                    "tooltip": "Set the color of all collision spheres outside the selected link",
                }
                self._models["color_picker"] = color_picker_builder(**kwargs)
                self._models["color_picker"].add_end_edit_fn(on_color_change)

                def clear_spheres_fn():
                    self._collision_sphere_editor.clear_spheres()
                    self._refresh_collision_sphere_comboboxes()

                self._models["clear_btn"] = btn_builder(
                    "Clear All Spheres", text="Clear", on_clicked_fn=clear_spheres_fn
                )
                self._models["clear_btn"].enabled = True

                frame = ui.CollapsableFrame(
                    title="Scale All Spheres",
                    name="subFrame",
                    height=0,
                    collapsed=True,
                    style=get_style(),
                    style_type_name_override="CollapsableFrame",
                    horizontal_scrollbar_policy=ui.ScrollBarPolicy.SCROLLBAR_AS_NEEDED,
                    vertical_scrollbar_policy=ui.ScrollBarPolicy.SCROLLBAR_ALWAYS_ON,
                )
                with frame:
                    with ui.VStack(style=get_style(), spacing=5, height=0):
                        kwargs = {
                            "label": "Scaling Factor",
                            "default_val": 1.0,
                            "min": 0.001,
                            "tooltip": "Scaling factor for the radii of the specified spheres",
                        }
                        self._models["scale_all_spheres_factor"] = float_builder(**kwargs)

                        def on_scale_all_spheres():
                            path = self._articulation_base_path
                            factor = self._models["scale_all_spheres_factor"].get_value_as_float()

                            self._collision_sphere_editor.scale_spheres(path, factor)

                        self._models["scale_all_sphere_btn"] = btn_builder(
                            "Scale All Spheres", text="Scale All Spheres", on_clicked_fn=on_scale_all_spheres
                        )
                        self._models["scale_all_sphere_btn"].enabled = True

        ###################################################################
        #                            Save to File
        ###################################################################
        export_frame = ui.CollapsableFrame(
            title="Export To File",
            name="subFrame",
            height=0,
            collapsed=True,
            style=get_style(),
            style_type_name_override="CollapsableFrame",
            horizontal_scrollbar_policy=ui.ScrollBarPolicy.SCROLLBAR_AS_NEEDED,
            vertical_scrollbar_policy=ui.ScrollBarPolicy.SCROLLBAR_ALWAYS_ON,
        )
        self._models["save_spheres_ui"] = export_frame
        export_frame.enabled = False

        def check_robot_description_file_type(model=None):
            path = model.get_value_as_string()
            if is_yaml_file(path) and "omniverse:" not in path.lower():
                self._models["robot_description_export_btn"].enabled = True
            else:
                self._models["robot_description_export_btn"].enabled = False
                carb.log_warn(f"Invalid path to Robot Desctiption YAML: {path}")

        def on_select_xrdf_output_file(model=None):
            path = model.get_value_as_string()
            if is_xrdf_file(path) and "omniverse:" not in path.lower():
                self._models["xrdf_export_btn"].enabled = True
                if self._is_valid_xrdf_file(path):
                    self._models["xrdf_merge_cb"].visible = True
                    self._models["xrdf_merge_cb"].set_value(True)
                else:
                    self._models["xrdf_merge_cb"].visible = False
                    self._models["xrdf_merge_cb"].set_value(False)
            else:
                self._models["robot_description_export_btn"].enabled = False
                carb.log_warn(f"Invalid path to XRDF: {path}")

        with export_frame:
            with ui.VStack(style=get_style(), spacing=5, height=0):
                frame = CollapsableFrame("Export to Lula Robot Description File", collapsed=True)
                kwargs = {
                    "label": "Output File",
                    "default_val": "",
                    "tooltip": "Click the Folder Icon to Set Filepath",
                    "use_folder_picker": True,
                    "item_filter_fn": on_filter_item,
                    "folder_dialog_title": "Write all sphere to a YAML file",
                    "folder_button_title": "Select YAML",
                }
                with frame:
                    with ui.VStack(style=get_style(), spacing=5, height=0):
                        self._models["robot_description_output_file"] = str_builder(**kwargs)
                        self._models["robot_description_output_file"].add_value_changed_fn(
                            check_robot_description_file_type
                        )

                        self._models["robot_description_export_btn"] = btn_builder(
                            "Save", text="Save", on_clicked_fn=self._save_robot_description_file
                        )
                        self._models["robot_description_export_btn"].enabled = False

                kwargs = {
                    "label": "Output File",
                    "default_val": "",
                    "tooltip": "Click the Folder Icon to Set Filepath",
                    "use_folder_picker": True,
                    "item_filter_fn": on_filter_xrdf_item,
                    "folder_dialog_title": "Write all sphere to an XRDF file",
                    "folder_button_title": "Select XRDF",
                }

                frame = CollapsableFrame("Export to cuMotion XRDF", collapsed=True)
                with frame:
                    with ui.VStack(style=get_style(), spacing=5, height=0):
                        self._models["xrdf_output_file"] = str_builder(**kwargs)
                        self._models["xrdf_output_file"].add_value_changed_fn(on_select_xrdf_output_file)

                        self._models["xrdf_export_btn"] = Button("Export XRDF", "Export", on_click_fn=self._export_xrdf)
                        self._models["xrdf_export_btn"].enabled = False

                        cb_tooltip = (
                            "Merge with the XRDF that already exists at the specified path. "
                            + "Merging will maintain any data written into the XRDF file that is "
                            + "not represented in the Robot Description Editor. Specifically, "
                            + "self_collision ignore rules and buffer distances, modifiers, "
                            + "tool_frames, and spheres for unrecognized robot frames."
                        )
                        self._models["xrdf_merge_cb"] = CheckBox("Merge With Existing XRDF", tooltip=cb_tooltip)
                        self._models["xrdf_merge_cb"].visible = False
                        self._models["xrdf_merge_cb"].set_value(False)

        ###################################################################
        #                   Import Robot Description File
        ###################################################################

        import_frame = CollapsableFrame("Import From File", collapsed=True, enabled=False)
        self._models["load_spheres_ui"] = import_frame

        def check_lula_robot_description_file_type(model=None):
            path = model.get_value_as_string()
            if is_yaml_file(path) and self.articulation is not None:
                self._models["robot_description_import_btn"].enabled = True
            elif self.articulation is None:
                self._models["robot_description_import_btn"].enabled = False
                carb.log_warn(
                    "Robot Articulation must be selected in the Selection Panel in order to import spheres for a robot"
                )
            else:
                self._models["robot_description_import_btn"].enabled = False
                carb.log_warn(f"Invalid path to Robot Desctiption YAML: {path}")

        def check_xrdf_file_type(model=None):
            path = model.get_value_as_string()
            if is_xrdf_file(path) and self.articulation is not None:
                self._models["xrdf_import_btn"].enabled = True
            elif self.articulation is None:
                self._models["xrdf_import_btn"].enabled = False
                carb.log_warn(
                    "Robot Articulation must be selected in the Selection Panel in order to import spheres for a robot"
                )
            else:
                self._models["xrdf_import_btn"].enabled = False
                carb.log_warn(f"Invalid path to XRDF: {path}")

        with import_frame:
            with ui.VStack(style=get_style(), spacing=5, height=0):
                frame = ui.CollapsableFrame(
                    title="Import Lula Robot Description File",
                    name="subFrame",
                    height=0,
                    collapsed=True,
                    style=get_style(),
                    style_type_name_override="CollapsableFrame",
                    horizontal_scrollbar_policy=ui.ScrollBarPolicy.SCROLLBAR_AS_NEEDED,
                    vertical_scrollbar_policy=ui.ScrollBarPolicy.SCROLLBAR_ALWAYS_ON,
                )

                with frame:
                    with ui.VStack(style=get_style(), spacing=5, height=0):
                        kwargs = {
                            "label": "Input File",
                            "default_val": "",
                            "tooltip": "Click the Folder Icon to Set Filepath",
                            "use_folder_picker": True,
                            "item_filter_fn": on_filter_item,
                            "folder_dialog_title": "Select Robot Description YAML file, clearing all spheres",
                            "folder_button_title": "Select YAML",
                        }
                        self._models["lula_robot_description_input_file"] = str_builder(**kwargs)
                        self._models["lula_robot_description_input_file"].add_value_changed_fn(
                            check_lula_robot_description_file_type
                        )

                        self._models["robot_description_import_btn"] = btn_builder(
                            "Import", text="Import", on_clicked_fn=self._load_robot_description_file
                        )
                        self._models["robot_description_import_btn"].enabled = False

                frame = CollapsableFrame("Import cuMotion XRDF", collapsed=True)
                with frame:
                    with ui.VStack(style=get_style(), spacing=5, height=0):
                        kwargs = {
                            "label": "Input File",
                            "default_val": "",
                            "tooltip": "Click the Folder Icon to Set Filepath",
                            "use_folder_picker": True,
                            "item_filter_fn": on_filter_xrdf_item,
                            "folder_dialog_title": "Select cuMotion XRDF file, clearing all spheres",
                            "folder_button_title": "Select YAML",
                        }
                        self._models["xrdf_input_file"] = str_builder(**kwargs)
                        self._models["xrdf_input_file"].add_value_changed_fn(check_xrdf_file_type)

                        self._models["xrdf_import_btn"] = Button("Import XRDF", "Import", on_click_fn=self._load_xrdf)
                        self._models["xrdf_import_btn"].enabled = False

    def _update_editor_ui(self):
        self._models["sphere_editor_ui"].collapsed = False
        self._models["sphere_editor_ui"].visible = True

        if is_yaml_file(self._models["lula_robot_description_input_file"].get_value_as_string()):
            self._models["robot_description_import_btn"].enabled = True
        if is_xrdf_file(self._models["xrdf_input_file"].get_value_as_string()):
            self._models["xrdf_import_btn"].enabled = True

        self._refresh_collision_sphere_comboboxes()

    def _update_command_ui(self):
        if self.articulation is None:
            return

        self._models["frame_command_ui"].enabled = True
        self._models["frame_command_ui"].rebuild()

        self.articulation.set_joint_positions(self._joint_positions)

    def _trigger_preview_generate_spheres_for_link(self, model=None, val=None):
        self._generate_spheres_for_link()

    def _generate_spheres_for_link(self, preview=True):
        if preview and not self._preview_spheres:
            return

        link = self._get_selected_link()
        mesh_index = self._models["sphere_gen_mesh_selection_model"].get_item_value_model().as_int
        if len(self._sphere_gen_link_2_mesh[link]) == 0:
            carb.log_warn(
                f"Could not generate spheres for any meshes in link {link}.  This is likely "
                + f"due to all meshes nested under {link} being instanceable"
            )
            return
        mesh = self._sphere_gen_link_2_mesh[link][mesh_index]

        num_spheres = self._models["sphere_gen_num_spheres"].get_value_as_int()
        if num_spheres <= 0:
            self._collision_sphere_editor.clear_preview()
            return

        radius_offset = self._models["sphere_gen_radius_offset"].get_value_as_float()

        link_path = self._articulation_base_path + link
        mesh_path = link_path + mesh
        geom_mesh = UsdGeom.Mesh(get_prim_at_path(mesh_path))
        points = np.array(geom_mesh.GetPointsAttr().Get())
        face_inds = np.array(geom_mesh.GetFaceVertexIndicesAttr().Get())
        vert_cts = np.array(geom_mesh.GetFaceVertexCountsAttr().Get())

        # Transform coordinates of points into Link frame
        mesh_xform = SingleXFormPrim(mesh_path)
        link_xform = SingleXFormPrim(link_path)

        mesh_trans, mesh_rot = mesh_xform.get_world_pose()
        link_trans, link_rot = link_xform.get_world_pose()
        link_rot, mesh_rot = quats_to_rot_matrices(np.array([link_rot, mesh_rot]))

        inv_rot = link_rot.T @ mesh_rot
        inv_trans = (link_rot.T @ (mesh_trans - link_trans)).reshape((3, 1))

        link_frame_points = (inv_rot @ points.T + inv_trans).T

        self._collision_sphere_editor.generate_spheres(
            link_path, link_frame_points, face_inds, vert_cts, num_spheres, radius_offset, preview
        )

    def _get_selected_collision_spheres(self):
        if not self._connect_sphere_0_options:
            return None, None

        name = "connect_sphere_selection_0"
        c0 = self._connect_sphere_0_options[self._models[name].get_item_value_model().as_int]

        if not self._connect_sphere_1_options:
            return c0, None

        name = "connect_sphere_selection_1"
        c1 = self._connect_sphere_1_options[self._models[name].get_item_value_model().as_int]
        return c0, c1

    def _get_selected_link_path(self):
        link = self._get_selected_link()
        if link is None:
            return None
        return self._articulation_base_path + link

    def _get_selected_link(self):
        if self.articulation is None:
            return None

        link_index = self._models["sphere_gen_link_selection_model"].get_item_value_model().as_int
        link = list(self._sphere_gen_link_2_mesh.keys())[link_index]

        return link

    def _get_selected_articulation(self):
        index = self._models["ar_selection_model"].get_item_value_model().as_int
        return self.articulation_list[index]

    def _preview_collision_spheres(self, model=None):
        if self._preview_spheres:
            self._preview_spheres = False
            self._collision_sphere_editor.clear_preview()
        else:
            self._preview_spheres = True
            self._generate_spheres_for_link()

    def _hide_link(self, link_name):
        meshes = self._sphere_gen_link_2_mesh[link_name]
        link_path = self._articulation_base_path + link_name
        mesh_paths = []
        for mesh in meshes:
            mesh_path = link_path + mesh
            mesh_paths.append(mesh_path)
        omni.kit.commands.execute("ToggleVisibilitySelectedPrims", selected_paths=mesh_paths)

    def _on_toggle_link_visible(self, model=None):
        self._hide_link(self._get_selected_link())
        self._hiding_link = not self._hiding_link

    def _on_toggle_robot_visible(self, model=None):
        selected_link = self._get_selected_link()
        links = list(self._sphere_gen_link_2_mesh.keys())
        for link in links:
            if selected_link != link:
                self._hide_link(link)

        self._hiding_robot = not self._hiding_robot

    def _show_robot_if_hidden(self):
        if self._hiding_robot:
            self._models["hide_robot_btn"].call_clicked_fn()
        if self._hiding_link:
            self._models["hide_link_btn"].call_clicked_fn()

    def _load_xrdf(self):
        if self.articulation is None:
            return
        path = self._models["xrdf_input_file"].get_value_as_string()

        parsed_file = self.safe_load_yaml(path)

        if "format" not in parsed_file or parsed_file["format"] != "xrdf":
            carb.log_error(
                "XRDF file is expected to contain the line \nformat: xrdf\n"
                + "but this line is missing.  Aborting Import."
            )
        if "format_version" not in parsed_file:
            carb.log_error(
                "XRDF file is expected to have a field:\nformat_version\nBut this field "
                + "is missing. Aborting Import."
            )
        elif parsed_file["format_version"] != 1.0:
            carb.log_warn(
                "Attempting to read an XRDF file that does not have format version 1.0.  This may not be supported."
            )

        self._active_joints = np.zeros(MAX_DOF_NUM, dtype=bool)
        self._acceleration_limits = np.full(MAX_DOF_NUM, DEFAULT_ACCELERATION_LIMIT)
        self._jerk_limits = np.full(MAX_DOF_NUM, DEFAULT_JERK_LIMIT)
        self._joint_positions[:] = 0
        dof_names = np.array(self.dof_names)

        cspace = parsed_file["cspace"]["joint_names"]
        file_acceleration_limits = parsed_file["cspace"]["acceleration_limits"]
        file_jerk_limits = parsed_file["cspace"]["jerk_limits"]

        default_q_map = parsed_file["default_joint_positions"]

        in_mask = np.in1d(cspace, dof_names)
        if not np.all(in_mask):
            carb.log_warn(
                "Some joints listed in the cspace of the provided robot_description YAML file are not present in the robot Articulation:"
                + f" {cspace[~in_mask]}"
            )
            cspace = cspace[in_mask]

        for i, joint in enumerate(cspace):
            ind = self.dof_names.index(joint)
            self._active_joints[ind] = True
            self._acceleration_limits[ind] = file_acceleration_limits[i]
            self._jerk_limits[ind] = file_jerk_limits[i]

        # Maps joint names to default joint positions
        for dof_name in default_q_map:
            if dof_name in self.articulation.dof_names:
                dof_index = self.articulation.dof_names.index(dof_name)
                self._joint_positions[dof_index] = default_q_map[dof_name]
            else:
                carb.log_warn(
                    "Invalid DOF name ["
                    + dof_name
                    + "] specified in XRDF file "
                    + "'default_joint_positions' field that could not be found in the currently "
                    + "selected Articulation."
                )

        lower_limit = self.articulation.dof_properties["lower"]
        upper_limit = self.articulation.dof_properties["upper"]
        self._joint_positions[: self.articulation.num_dof] = np.clip(
            self._joint_positions[: self.articulation.num_dof], lower_limit, upper_limit
        )

        self._collision_sphere_editor.load_xrdf_spheres(self._articulation_base_path, parsed_file)

        self._update_command_ui()

    def _load_robot_description_file(self, model=None):
        if self.articulation is None:
            return
        path = self._models["lula_robot_description_input_file"].get_value_as_string()

        parsed_file = self.safe_load_yaml(path)

        self._active_joints = np.zeros(MAX_DOF_NUM, dtype=bool)
        dof_names = np.array(self.dof_names)

        cspace = parsed_file["cspace"]
        default_q = parsed_file["default_q"]

        self._acceleration_limits = np.full(MAX_DOF_NUM, DEFAULT_ACCELERATION_LIMIT)
        self._jerk_limits = np.full(MAX_DOF_NUM, DEFAULT_JERK_LIMIT)

        file_acceleration_limits = parsed_file.get("acceleration_limits", None)

        file_jerk_limits = parsed_file.get("jerk_limits", None)

        in_mask = np.in1d(cspace, dof_names)
        if not np.all(in_mask):
            carb.log_warn(
                "Some joints listed in the cspace of the provided robot_description YAML file are not present in the robot Articulation:"
                + f" {cspace[~in_mask]}"
            )
            cspace = cspace[in_mask]

        for i, joint in enumerate(cspace):
            ind = self.dof_names.index(joint)
            self._active_joints[ind] = True
            self._joint_positions[ind] = default_q[i]

            if file_acceleration_limits is not None:
                self._acceleration_limits[ind] = file_acceleration_limits[i]
            if file_jerk_limits is not None:
                self._jerk_limits[ind] = file_jerk_limits[i]

        fixed_joints = parsed_file.get("cspace_to_urdf_rules", None)

        if fixed_joints is not None:
            for item in fixed_joints:
                if item["rule"] != "fixed":
                    continue
                joint_name = item["name"]
                if joint_name not in self.dof_names:
                    carb.log_warn(
                        f"Fixed joint specified for a joint that is not present in the robot Articulation: {joint_name}"
                    )
                    return
                ind = self.dof_names.index(item["name"])
                self._active_joints[ind] = False
                self._joint_positions[ind] = item["value"]

        self._collision_sphere_editor.load_spheres(self._articulation_base_path, path)

        self._update_command_ui()

    def _is_valid_xrdf_file(self, path):
        warning_msg = f"XRDF file {path} is not a valid XRDF file for Merging. Save to a new XRDF file."
        if not os.path.isfile(path):
            carb.log_warn(warning_msg)
            return False
        with open(path, "r") as stream:
            try:
                parsed_file = yaml.safe_load(stream)
            except yaml.YAMLError as exc:
                carb.log_warn(warning_msg + f" {exc}")
                return False

        if "format" in parsed_file and parsed_file["format"] == "xrdf" and "format_version" in parsed_file:
            if parsed_file["format_version"] != 1.0:
                carb.log_warn(
                    "Attempting to read an XRDF file that does not have format version 1.0.  This may not be supported."
                )
            return True
        else:
            carb.log_warn(warning_msg)
            return False

    def recursive_cast_to_float(self, d):
        from collections.abc import Iterable

        for k, v in d.items():
            if isinstance(v, str):
                try:
                    f = float(v)
                    d[k] = f
                except:
                    pass
            elif isinstance(v, dict):
                self.recursive_cast_to_float(v)
            elif isinstance(v, Iterable):
                l = []
                for item in v:
                    f = item
                    if isinstance(item, str):
                        try:
                            f = float(item)
                        except:
                            pass
                    l.append(f)
                d[k] = l

    def safe_load_yaml(self, path):
        with open(path, "r") as stream:
            try:
                parsed_file = yaml.safe_load(stream)
            except yaml.YAMLError as exc:
                carb.log_error("Attempted to load invalid yaml file " + str(exc))

        self.recursive_cast_to_float(parsed_file)
        return parsed_file

    def _copy_information_from_existing_xrdf(self, path):
        parsed_file = {}
        if not self._is_valid_xrdf_file(path):
            return parsed_file

        articulation_frames = {link_path[1:] for link_path in self._sphere_gen_link_2_mesh.keys()}
        parsed_file = self.safe_load_yaml(path)

        parsed_file.pop("default_joint_positions", None)
        parsed_file.pop("cspace", None)

        if (
            "self_collision" in parsed_file
            and "geometry" in parsed_file["self_collision"]
            and "collision" in parsed_file
            and "geometry" in parsed_file["collision"]
        ):
            if parsed_file["self_collision"]["geometry"] == parsed_file["collision"]["geometry"]:
                # Since buffer distances in the "collision" group are going to be set to zero,
                # to keep the relative sphere sizes between "collision" and "self_collision" the
                # same, "collision" buffer distances will be subtracted "self_collision" buffer
                # distances.
                if "buffer_distance" in parsed_file["collision"]:
                    collision_buffer_distance = parsed_file["collision"]["buffer_distance"]
                    self_collision_buffer_distance = parsed_file["self_collision"].get("buffer_distance", {})
                    for k, v in collision_buffer_distance.items():
                        if k in articulation_frames:
                            if k in self_collision_buffer_distance:
                                self_collision_buffer_distance[k] -= v
                            else:
                                self_collision_buffer_distance[k] = -v
                            collision_buffer_distance[k] = 0
            else:
                parsed_file["self_collision"] = {"geometry": parsed_file["collision"]["geometry"]}

        if "collision" not in parsed_file or "geometry" not in parsed_file["collision"]:
            parsed_file.pop("geometry", None)
            parsed_file.pop("collision", None)
            parsed_file.pop("self_collision", None)
        else:
            for k in list(parsed_file["geometry"].keys()):
                if k != parsed_file["collision"]["geometry"]:
                    parsed_file["geometry"].pop(k, None)
                else:
                    parsed_file["geometry"][k].pop("clone", None)

        return parsed_file

    def get_ignore_dict(self, ordered_links):
        articulation_path = self._articulation_base_path
        ignore_dict = {}

        # Any links conencted by a joint should ignore each other
        for p in Usd.PrimRange(get_prim_at_path(articulation_path)):
            if UsdPhysics.Joint(p):
                b0 = p.GetProperty("physics:body0").GetTargets()
                b1 = p.GetProperty("physics:body1").GetTargets()

                if len(b0) == 1 and len(b1) == 1:
                    b0 = str(b0[0])
                    b1 = str(b1[0])
                    l0 = b0.split("/")[-1]
                    l1 = b1.split("/")[-1]
                    if l0 in ordered_links and l1 in ordered_links:
                        if l0 in ignore_dict:
                            ignore_dict[l0].append(l1)
                        else:
                            ignore_dict[l0] = [l1]

        # If A connects to B,C,D then B,C,D should all ignore each other.
        extended_ignore_dict = copy.deepcopy(ignore_dict)
        for k, v in ignore_dict.items():
            for i in range(len(v) - 1):
                for j in range(i + 1, len(v)):
                    if v[i] in extended_ignore_dict:
                        extended_ignore_dict[v[i]].append(v[j])
                    else:
                        extended_ignore_dict[v[i]] = [v[j]]

        return extended_ignore_dict

    def _export_xrdf(self, model=None):
        if self.articulation is None:
            return

        path = self._models["xrdf_output_file"].get_value_as_string()
        if self._models["xrdf_merge_cb"].get_value():
            parsed_file = self._copy_information_from_existing_xrdf(path)
        else:
            parsed_file = {}

        art_view = Articulation(self._articulation_base_path)
        art_view.initialize()
        ordered_links = art_view.body_names  # Links in order from root to end effector

        parsed_file["format"] = "xrdf"
        parsed_file["format_version"] = 1.0

        active_joints_mask = self._active_joints[: self.num_dof]
        acceleration_limits = self._acceleration_limits[: self.num_dof][active_joints_mask]
        jerk_limits = self._jerk_limits[: self.num_dof][active_joints_mask]
        dof_names = np.array(self.dof_names)

        default_joint_positions_dict = dict()
        for i, dof_name in enumerate(dof_names):
            default_joint_positions_dict[dof_name] = self._joint_positions[i]
        parsed_file["default_joint_positions"] = default_joint_positions_dict

        cspace_dict = {"joint_names": [], "acceleration_limits": [], "jerk_limits": []}
        for i, dof_name in enumerate(dof_names[active_joints_mask]):
            cspace_dict["joint_names"].append(dof_name)
            cspace_dict["acceleration_limits"].append(acceleration_limits[i])
            cspace_dict["jerk_limits"].append(jerk_limits[i])
        parsed_file["cspace"] = cspace_dict

        if "geometry" not in parsed_file or "collision" not in parsed_file:
            default_name = "auto_generated_collision_sphere_group"
            parsed_file["collision"] = {"geometry": default_name}
            parsed_file["geometry"] = {default_name: {"spheres": {}}}
            parsed_file["self_collision"] = {"geometry": default_name}

        if "ignore" not in parsed_file["self_collision"]:
            ignore_dict = self.get_ignore_dict(ordered_links)
            parsed_file["self_collision"]["ignore"] = ignore_dict

        geometry_group_name = parsed_file["collision"]["geometry"]
        sphere_dict = parsed_file["geometry"][geometry_group_name].get("spheres", None)
        if sphere_dict is None:
            sphere_dict = {}
            parsed_file["geometry"][geometry_group_name]["spheres"] = sphere_dict
        for link in ordered_links:
            sphere_dict.pop(link, None)
        self._collision_sphere_editor.write_spheres_to_dict(self._articulation_base_path, sphere_dict)

        key_order = [
            "format",
            "format_version",
            "modifiers",
            "default_joint_positions",
            "cspace",
            "tool_frames",
            "collision",
            "self_collision",
            "geometry",
        ]

        def write_item(f, item, tabbing):
            if isinstance(item, dict):
                for k in list(item.keys()):
                    f.write(f"{tabbing}{k}: ")
                    tabbing = " " * len(tabbing)
                    if isinstance(item[k], dict):
                        f.write("\n")
                    write_item(f, item[k], tabbing + "  ")
            elif isinstance(item, list) or isinstance(item, np.ndarray):
                # Assume all elements are the same type
                if len(item) == 0:
                    f.write(f"[]\n")
                    return
                if isinstance(item[0], dict):
                    f.write("\n")
                    for d in item:
                        write_item(f, d, tabbing + "- ")
                elif isinstance(item[0], str):
                    f.write("\n")
                    for val in item:
                        f.write(tabbing + "- ")
                        write_item(f, val, "")
                else:
                    f.write(f"[")
                    for val in item[:-1]:
                        f.write(f"{str(np.around(val, 4))}, ")
                    f.write(f"{str(np.around(item[-1],4))}]\n")
            else:
                if isinstance(item, str):
                    f.write(f'"{item}"\n')
                else:
                    f.write(f"{str(np.around(item, 4))}\n")

        with open(path, "w") as f:
            for key in key_order:
                if key in parsed_file:
                    f.write(f"{key}: ")
                    value = parsed_file[key]
                    if isinstance(value, dict):
                        f.write("\n")
                    write_item(f, value, "  ")
                    if key != key_order[-1]:
                        f.write("\n")

    def _save_robot_description_file(self, model=None):
        if self.articulation is None:
            return

        active_joints_mask = self._active_joints[: self.num_dof]
        if np.sum(active_joints_mask) == 0:
            carb.log_error(
                "There are no Active Joints in this robot description (Reference the Information Panel subsection: Command Panel).  This means that Lula will not control the robot at all.  Aborting Save Operation."
            )
            return

        fixed_joints_mask = ~active_joints_mask

        acceleration_limits = self._acceleration_limits[: self.num_dof]
        jerk_limits = self._jerk_limits[: self.num_dof]

        dof_names = np.array(self.dof_names)

        path = self._models["robot_description_output_file"].get_value_as_string()
        if not path:
            carb.log_error(f"Cannot Save to Invalid Path {path}")
            return

        with open(path, "w") as f:
            f.write(
                "# The robot description defines the generalized coordinates and how to map those\n"
                + "# to the underlying URDF dofs.\n\n"
                + "api_version: 1.0\n\n"
                + "# Defines the generalized coordinates. Each generalized coordinate is assumed\n"
                + "# to have an entry in the URDF.\n"
                + "# Lula will only use these joints to control the robot position.\n"
                + "cspace:\n"
            )
            for name in dof_names[active_joints_mask]:
                f.write(f"    - {name}\n")

            f.write("default_q: [\n")
            f.write("    ")
            for joint_pos in self._joint_positions[active_joints_mask][:-1]:
                pos = np.around(joint_pos, 4)
                f.write(f"{str(pos)},")
            f.write(f"{str(np.around(self._joint_positions[active_joints_mask][-1],4))}\n")
            f.write("]\n\n")

            f.write("acceleration_limits: [\n")
            f.write("   ")
            for accel_limit in acceleration_limits[active_joints_mask][:-1]:
                l = np.around(accel_limit, 2)
                f.write(f"{str(l)},")
            f.write(f"{str(np.around(acceleration_limits[active_joints_mask][-1],2))}\n")
            f.write("]\n\n")

            f.write("jerk_limits: [\n")
            f.write("   ")
            for jerk_limit in jerk_limits[active_joints_mask][:-1]:
                l = np.around(jerk_limit, 2)
                f.write(f"{str(l)},")
            f.write(f"{str(np.around(jerk_limits[active_joints_mask][-1],2))}\n")
            f.write("]\n\n")

            f.write("# Most dimensions of the cspace have a direct corresponding element\n")
            f.write("# in the URDF. This list of rules defines how unspecified coordinates\n")
            f.write("# should be extracted or how values in the URDF should be overwritten.\n\n")

            f.write("cspace_to_urdf_rules:\n")
            for name, position in zip(dof_names[fixed_joints_mask], self._joint_positions[fixed_joints_mask]):
                pos = np.around(position, 4)
                f.write(f"    - {{name: {name}, rule: fixed, value: {str(pos)}}}\n")
            f.write("\n")

            f.write("# Lula uses collision spheres to define the robot geometry in order to avoid\n")
            f.write("# collisions with external obstacles.  If no spheres are specified, Lula will\n")
            f.write("# not be able to avoid obstacles.\n\n")

            self._collision_sphere_editor.save_spheres(self._articulation_base_path, f)
