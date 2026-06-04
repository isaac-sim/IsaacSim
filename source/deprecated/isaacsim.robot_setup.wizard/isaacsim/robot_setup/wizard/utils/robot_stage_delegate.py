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

"""Stage delegate and context menu for robot setup operations within the Isaac Sim robot setup wizard."""

import carb
from omni.kit.widget.stage import ContextMenu
from omni.kit.widget.stage.stage_delegate import StageDelegate

from ..builders.robot_templates import RobotRegistry


class RobotContextMenu(ContextMenu):
    """A context menu for robot setup operations in the stage window.

    This class extends the standard stage context menu to provide robot-specific functionality,
    including marking reference meshes for parent link alignment and deleting robot components.
    It handles mouse events to display contextual menu options based on the selected prims
    in the stage hierarchy.

    The context menu provides the following operations:

    - **Mark as Reference to Align Parent Link**: Marks a selected prim as a reference mesh
      for aligning the parent link's origin. This is useful for establishing reference points
      during robot setup and configuration.
    - **Delete**: Removes selected prims from the robot temporary stage and marks them for
      deletion during reorganization.

    Menu items are dynamically shown based on the current selection and prim properties.
    The class integrates with the RobotRegistry to store reference mesh information and
    track prims marked for deletion.
    """

    def on_mouse_event(self, event: object) -> None:
        """Handles mouse events to display context menu for robot prims.

        Args:
            event: Mouse event containing stage, prim path, and other context data.

        Returns:
            None if the event type is not ACTIVATE, context menu is not available, or stage is not available.
        """
        import omni.kit.menu.core

        # check its expected event
        if event.type != int(omni.kit.menu.core.MenuEventType.ACTIVATE):
            return

        try:
            import omni.kit.context_menu
        except ModuleNotFoundError:
            return
        # get context menu core functionality & check its enabled
        context_menu = omni.kit.context_menu.get_instance()
        if context_menu is None:
            carb.log_error("context_menu is disabled!")
            return

        # get stage
        stage = event.payload.get("stage", None)
        if stage is None:
            carb.log_error("stage not avaliable")
            return

        # get parameters passed by event
        prim_path = event.payload["prim_path"]

        # setup objects, this is passed to all functions
        objects = {}
        objects["use_hovered"] = True if prim_path else False
        objects["stage_win"] = self._stage_win
        objects["node_open"] = event.payload["node_open"]
        objects["stage"] = stage
        objects["function_list"] = self.function_list
        objects["stage_model"] = self._stage_model

        prim_list = []
        hovered_prim = event.payload["prim_path"]
        paths = omni.usd.get_context().get_selection().get_selected_prim_paths()
        if len(paths) > 0:
            for path in paths:
                prim = stage.GetPrimAtPath(path)
                if prim:
                    prim_list.append(prim)
                    if prim == hovered_prim:
                        hovered_prim = None

        elif prim_path is not None:
            prim = stage.GetPrimAtPath(prim_path)
            if prim:
                prim_list.append(prim)

        if prim_list:
            objects["prim_list"] = prim_list
        if hovered_prim:
            objects["hovered_prim"] = stage.GetPrimAtPath(hovered_prim)

        # setup menu
        menu_dict = [
            {
                "name": "Mark as Reference to Align Parent Link",
                "glyph": "menu_rename.svg",
                "show_fn": [self._can_align_parent_link_origin, self._is_prim_selected, self.is_one_prim_selected],
                "onclick_fn": self._align_parent_link_origin,
            },
            {
                "name": "Delete",
                "glyph": "menu_delete.svg",
                "show_fn": [self._is_prim_selected, self._can_delete],
                "onclick_fn": self._delete_prims,
            },
        ]

        # show menu
        try:
            context_menu.show_context_menu("robot_stage_window", objects, menu_dict)
        except Exception as e:
            print(f"Error showing context menu: {str(e)}")

    def _can_align_parent_link_origin(self, objects: dict) -> bool:
        """Checks if the selected prim can be aligned to parent link origin.

        Args:
            objects: Context menu data containing prim information.

        Returns:
            True if prim path has at least 4 parts (robot/link/link_name/component).
        """
        if "prim_list" not in objects:
            return False
        prim_list = objects["prim_list"][0]  # only process one prim at a time
        # break down path of the prim and it needs to have at least 3 parts (robot/link/link_name)
        path_parts = prim_list.GetPath().pathString.split("/")
        if len(path_parts) < 4:
            return False
        else:
            return True

    def _align_parent_link_origin(self, objects: dict) -> bool:
        """Marks the selected prim as reference mesh for aligning parent link origin.

        Args:
            objects: Context menu data containing the selected prim.

        Returns:
            True if reference mesh was successfully registered.
        """
        registered_robot = RobotRegistry().get()
        if registered_robot is None:
            return False
        ref_prim_path = objects["prim_list"][0].GetPath().pathString
        # if registered_robot already has reference mesh property, get it, otherwise,= add_property
        if not hasattr(registered_robot, "reference_mesh"):
            registered_robot.add_property(registered_robot.__class__, "reference_mesh", {})

        parent_link_path = "/".join(ref_prim_path.split("/")[:3])  # the first three parts of the path

        reference_mesh_table = registered_robot.reference_mesh
        reference_mesh_table[parent_link_path] = ref_prim_path
        registered_robot.reference_mesh = reference_mesh_table

        return True

    def _is_prim_selected(self, objects: dict) -> bool:
        """Checks if any prims are selected.

        Args:
            objects: Context menu data.

        Returns:
            True if one or more prim is selected otherwise False.
        """
        if not any(item in objects for item in ["prim", "prim_list"]):
            return False
        return True

    def is_one_prim_selected(self, objects: dict) -> bool:
        """Checks if one prim is selected.

        Args:
            objects: Context menu data.

        Returns:
            True if one prim is selected otherwise False.
        """
        if "prim_list" not in objects:
            return False
        return len(objects["prim_list"]) == 1

    def _can_delete(self, objects: dict) -> bool:
        """Checks if prims can be deleted.

        Args:
            objects: Context menu data.

        Returns:
            True if prim can be deleted otherwise False.
        """
        if not any(item in objects for item in ["prim", "prim_list"]):
            return False
        prim_list = [objects["prim"]] if "prim" in objects else objects["prim_list"]

        for prim in prim_list:
            if not prim.IsValid():
                return False
            no_delete = prim.GetMetadata("no_delete")
            if no_delete is not None and no_delete is True:
                return False
        return True

    def _delete_prims(self, objects: dict, destructive: bool = True) -> bool:
        """Removes prims from the robot stage and marks them for deletion during reorg.

        Args:
            objects: Context menu data containing prims to delete.
            destructive: Whether to perform destructive deletion.

        Returns:
            True if prims were successfully processed for deletion.
        """
        # remove the prim from the robot_temp_stage
        prims = objects.get("prim_list", [])
        stage = objects["stage"]
        if prims:
            for prim in prims:
                prim_path = prim.GetPath()
                stage.RemovePrim(prim_path)

        # marking it to be deleted during reorg
        registered_robot = RobotRegistry().get()
        if registered_robot is None:
            return False
        to_delete_prim_path = objects["prim_list"][0].GetPath().pathString
        # if registered_robot already has reference mesh property, get it, otherwise,= add_property
        if not hasattr(registered_robot, "delete_prim_paths"):
            registered_robot.add_property(registered_robot.__class__, "delete_prim_paths", [])

        delete_prim_paths = registered_robot.delete_prim_paths
        delete_prim_paths.append(to_delete_prim_path)
        registered_robot.delete_prim_paths = delete_prim_paths

        return True


class RobotStageDelegate(StageDelegate):
    """Stage delegate for robot setup and management within the Isaac Sim robot setup wizard.

    This class extends the standard stage delegate functionality with robot-specific context menu operations
    and behaviors. It integrates with the robot setup wizard to provide specialized stage interactions
    for robot configuration, including marking reference meshes for link alignment and managing prim
    deletion within the robot context.

    The delegate uses a custom RobotContextMenu that provides robot-specific operations such as marking
    prims as reference points for parent link origin alignment and handling deletion of robot components
    with proper cleanup through the RobotRegistry system.
    """

    def __init__(self) -> None:
        super().__init__(context_menu=RobotContextMenu())
