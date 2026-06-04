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

"""Isaac Sim physics utilities extension that provides a UI for applying and removing physics APIs on USD prims."""

import asyncio
import gc
import weakref

import omni.ext
import omni.kit.commands
import omni.kit.utils
import omni.ui as ui
import omni.usd
from isaacsim.gui.components.element_wrappers import ScrollingWindow
from isaacsim.gui.components.menu import make_menu_item_description
from isaacsim.gui.components.ui_utils import (
    btn_builder,
    cb_builder,
    dropdown_builder,
    multi_btn_builder,
    progress_bar_builder,
)
from omni.kit.menu.utils import MenuItemDescription, add_menu_items, remove_menu_items
from omni.physx.scripts import utils
from pxr import Usd, UsdGeom, UsdPhysics

EXTENSION_NAME = "Physics API Editor"


class Extension(omni.ext.IExt):
    """Extension providing a UI for applying and removing physics APIs on USD prims."""

    def on_startup(self, ext_id: str):
        """Initialize the extension UI and menu items.

        Args:
            ext_id: The unique identifier for this extension instance.
        """
        self._usd_context = omni.usd.get_context()
        self._selected_prim = None
        self._selection = self._usd_context.get_selection()

        self._window = ScrollingWindow(
            title=EXTENSION_NAME, width=600, height=400, visible=False, dockPreference=ui.DockPreference.LEFT_BOTTOM
        )
        self._window.deferred_dock_in("Console", omni.ui.DockPolicy.DO_NOTHING)
        menu_entry = [
            make_menu_item_description(ext_id, EXTENSION_NAME, lambda a=weakref.proxy(self): a._menu_callback())
        ]
        self._menu_items = [MenuItemDescription("Physics", sub_menu=menu_entry)]

        add_menu_items(self._menu_items, "Tools")

        with self._window.frame:
            with ui.VStack(spacing=5, height=0):
                ui.Label("Collision APIs:")
                self._children_checkbox = cb_builder(
                    label="Apply To Children",
                    default_val=True,
                    tooltip="Apply collision API to child prims if possible",
                )
                self._visible_checkbox = cb_builder(
                    label="Visible Only", default_val=True, tooltip="Only apply collision API to prims that are visible"
                )
                self._collision_type = dropdown_builder(
                    label="Collision Type", items=["Triangle Mesh", "Convex Hull", "Convex Decomposition"]
                )
                btn_builder(
                    label="For Current Selection",
                    text="Apply Static",
                    tooltip="The following buttons work on the currently selected prims",
                    on_clicked_fn=self.apply_collision_on_selected,
                )
                ui.Line()
                ui.Label("Remove APIs on Current Selection:")
                multi_btn_builder(
                    label="For Current Selection",
                    text=["Remove Collision API", "Remove All Physics APIs"],
                    tooltip=[
                        "The following buttons work on the currently selected prims",
                        "Remove Collision API on selected",
                        "NOTE: This cannot delete usd prims on referenced assets, but will still unapply USD APIs",
                    ],
                    on_clicked_fn=[self.clear_collision_on_selected, self.remove_physics_apis_on_selected],
                )
                ui.Line()
                self._progress_bar = progress_bar_builder("Current Progress")
                self._progress_bar.set_value(0)

    def _menu_callback(self):
        """Toggle the visibility of the extension window."""
        self._window.visible = not self._window.visible

    def apply_collision_on_selected(self):
        """Apply collision APIs to the currently selected prims.

        Creates an async task that traverses all selected prims and applies the appropriate
        collision API based on the selected collision type (Triangle Mesh, Convex Hull, or
        Convex Decomposition). Updates the progress bar as prims are processed.
        """

        async def _task():
            self._stage = self._usd_context.get_stage()
            selection = self._selection.get_selected_prim_paths()
            if (len(selection)) == 0:
                return
            all_prims = self.traverse_prims(
                selection, include_xform=False, visible_only=self._visible_checkbox.get_value_as_bool()
            )
            count = len(all_prims)
            index = 0

            approximation_type = ["none", "convexHull", "convexDecomposition"]

            for prim in all_prims:
                selection = self._collision_type.get_item_value_model().as_int
                self.apply_collision_to_prim(prim, approximation_type[selection])
                index = index + 1
                if index % 100 == 0 or index == count:
                    self._progress_bar.set_value(index / count)
                    await omni.kit.app.get_app().next_update_async()

        asyncio.ensure_future(_task())

    def clear_collision_on_selected(self):
        """Remove collision APIs from the currently selected prims.

        Creates an async task that traverses all selected prims and removes any collision
        APIs that have been applied. Updates the progress bar as prims are processed.
        """

        async def _task():
            self._stage = self._usd_context.get_stage()
            selection = self._selection.get_selected_prim_paths()
            if (len(selection)) == 0:
                return
            all_prims = self.traverse_prims(
                selection, include_xform=True, visible_only=self._visible_checkbox.get_value_as_bool()
            )
            count = len(all_prims)
            index = 0
            for prim in all_prims:
                self.unapply_collision_on_prim(prim)
                index = index + 1
                if index % 100 == 0 or index == count:
                    self._progress_bar.set_value(index / count)
                    await omni.kit.app.get_app().next_update_async()

        asyncio.ensure_future(_task())

    def remove_physics_apis_on_selected(self):
        """Remove all physics APIs from the currently selected prims.

        Creates an async task that traverses all selected prims and removes all physics-related
        APIs including rigid body, collision, articulation, joints, and other physics components.
        Updates the progress bar as prims are processed.
        """

        async def _task():
            self._stage = self._usd_context.get_stage()
            selection = self._selection.get_selected_prim_paths()
            if (len(selection)) == 0:
                return
            all_prims = self.traverse_prims(selection, include_xform=True, ignore_rigid=False, visible_only=False)
            count = len(all_prims)
            index = 0
            for prim in all_prims:
                self.remove_physics_apis_on_prim(prim)
                index = index + 1
                if index % 100 == 0 or index == count:
                    self._progress_bar.set_value(index / count)
                    await omni.kit.app.get_app().next_update_async()

        asyncio.ensure_future(_task())

    def traverse_prims(
        self, selection: list, include_xform: bool = False, ignore_rigid: bool = True, visible_only: bool = True
    ) -> list:
        """Traverse and collect valid prims from the given selection.

        Iterates through the selected prim paths and their children (if enabled), filtering
        based on visibility, rigid body status, and prim type.

        Args:
            selection: List of selected prim paths to traverse.
            include_xform: Whether to include Xformable prims in the results.
            ignore_rigid: Whether to skip prims with PhysicsRigidBodyAPI and their children.
            visible_only: Whether to only include visible prims.

        Returns:
            List of valid prims matching the filter criteria.
        """
        prims = []
        for s in selection:
            curr_prim = self._stage.GetPrimAtPath(s)
            if self._children_checkbox.get_value_as_bool():
                prim_range_iter = iter(Usd.PrimRange(curr_prim))
                for prim in prim_range_iter:
                    # process the current prim if its an instance, but prune children as we cannot process them anyways
                    if prim.IsInstanceable():
                        prim_range_iter.PruneChildren()
                    imageable = UsdGeom.Imageable(prim)
                    # ignore non stage prims
                    if prim.GetMetadata("hide_in_stage_window"):
                        prim_range_iter.PruneChildren()
                        continue
                    # If a prim is hidden and visible only is checked, skip all children of that prim
                    if visible_only:
                        if imageable:
                            visibility = imageable.ComputeVisibility(Usd.TimeCode.Default())
                            if visibility == UsdGeom.Tokens.invisible:
                                prim_range_iter.PruneChildren()
                                continue
                    # Ignore rigid bodies and its children
                    if ignore_rigid and utils.hasSchema(prim, "PhysicsRigidBodyAPI"):
                        prim_range_iter.PruneChildren()
                        continue
                    if self.prim_is_valid(
                        prim, include_xform or prim.IsInstanceable(), self._visible_checkbox.get_value_as_bool()
                    ):
                        prims.append(prim)
            else:
                if self.prim_is_valid(
                    curr_prim, include_xform or curr_prim.IsInstanceable(), self._visible_checkbox.get_value_as_bool()
                ):
                    prims.append(curr_prim)
        return prims

    def prim_is_valid(self, prim: Usd.Prim, include_xform: bool = False, visible_only: bool = True) -> bool:
        """Check if a prim is valid for physics API application.

        A prim is considered valid if it is a geometric primitive (Cylinder, Capsule, Cone,
        Sphere, Cube), a Mesh with valid points, or an Xformable (when include_xform is True).

        Args:
            prim: The USD prim to validate.
            include_xform: Whether to consider Xformable prims as valid.
            visible_only: Whether visibility should be checked (unused in this method).

        Returns:
            True if the prim is valid for physics API application, False otherwise.
        """
        if (
            prim.IsA(UsdGeom.Cylinder)
            or prim.IsA(UsdGeom.Capsule)
            or prim.IsA(UsdGeom.Cone)
            or prim.IsA(UsdGeom.Sphere)
            or prim.IsA(UsdGeom.Cube)
        ):
            return True
        elif prim.IsA(UsdGeom.Mesh):
            usdMesh = UsdGeom.Mesh(prim)
            attr = usdMesh.GetPointsAttr().Get()
            if attr is None or len(attr) == 0:
                return False
            else:
                return True
        elif include_xform and prim.IsA(UsdGeom.Xformable):
            return True
        return False

    def apply_collision_to_prim(self, prim: Usd.Prim, approximationShape: str = "none") -> None:
        """Apply collision API to a single prim.

        For instanceable prims, applies CollisionAPI and MeshCollisionAPI directly.
        For other prims, uses the physx utility to set the collider with the specified
        approximation shape.

        Args:
            prim: The USD prim to apply collision to.
            approximationShape: The collision approximation type. One of "none" (triangle mesh),
                "convexHull", or "convexDecomposition".
        """
        # TODO: add checks for rigid body parent type, we cannot use regular collision mesh in that case
        if prim.IsInstanceable():
            UsdPhysics.CollisionAPI.Apply(prim)
            UsdPhysics.MeshCollisionAPI.Apply(prim)
        else:
            utils.setCollider(prim, approximationShape)

    def unapply_collision_on_prim(self, prim: Usd.Prim) -> None:
        """Remove collision API from a single prim.

        Args:
            prim: The USD prim to remove collision from.
        """
        utils.removeCollider(prim)

    def remove_physics_apis_on_prim(self, prim: Usd.Prim) -> None:
        """Remove all physics-related APIs from a single prim.

        Removes a comprehensive list of physics APIs including rigid body, collision,
        articulation, character controller, contact report, trigger, material, mass,
        and various joint types.

        Args:
            prim: The USD prim to remove physics APIs from.
        """
        apis = [
            "PhysicsRigidBodyAPI",
            "PhysicsCollisionAPI",
            "PhysicsArticulationRootAPI",
            "CharacterControllerAPI",
            "ContactReportAPI",
            "PhysicsFilteredPairsAPI",
            "TriggerAPI",
            "PhysicsMaterialAPI",
            "PhysicsMassAPI",
            "PhysicsRevoluteJoint",
            "PhysicsPrismaticJoint",
            "PhysicsSphericalJoint",
            "PhysicsDistanceJoint",
            "PhysicsFixedJoint",
            "PhysicsLimit",
            "PhysicsDrive",
        ]
        for component in apis:
            omni.kit.commands.execute("RemovePhysicsComponentCommand", usd_prim=prim, component=component)

    def on_shutdown(self):
        """Clean up extension resources on shutdown.

        Removes menu items and triggers garbage collection.
        """
        remove_menu_items(self._menu_items, "Tools")
        gc.collect()
