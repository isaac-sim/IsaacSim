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

"""Provides UI components and models for managing collision physics setup in the Isaac Sim robot setup wizard."""

from typing import Any

import omni.ui as ui
import omni.usd
from omni.kit.widget.filter import FilterButton
from omni.kit.widget.searchfield import SearchField
from pxr import UsdGeom

from ..builders.collider_helper import MESH_APPROXIMATIONS, MESH_TYPES, apply_collider
from ..builders.robot_templates import RobotRegistry
from ..style import get_popup_window_style
from ..utils.robot_asset_picker import RobotAssetPicker
from ..utils.treeview_delegate import (
    SearchableItem,
    TreeViewIDColumn,
    TreeViewWithPlacerHolderDelegate,
    TreeViewWithPlacerHolderModel,
)
from ..utils.ui_utils import ButtonWithIcon, custom_header, info_frame, next_step, separator

APPROXIMATION_LIST = list(MESH_APPROXIMATIONS.keys()) + MESH_TYPES


class ColliderItem(SearchableItem):
    """Represents a collider configuration item for robot setup.

    This class encapsulates the properties of a collider that can be applied to robot parts, including
    the target name, collision mesh approximation type, and editability settings. It extends SearchableItem
    to provide text-based filtering capabilities in UI components.

    Args:
        name: The name or path of the robot part to apply the collider to.
        approx: The collision mesh approximation type (e.g., 'convexHull', 'trimesh', 'boundingCube').
        editable: Whether the collider item can be modified in the UI.
    """

    def __init__(self, name: str, approx: str = "None", editable: bool = True) -> None:
        super().__init__()
        self.name = ui.SimpleStringModel(name)
        self.approx = ui.SimpleStringModel(approx)
        self.editable = ui.SimpleBoolModel(editable)
        self.collider_target = "Pick part to add collider to"

        self.text = " ".join((name, approx))

    def set_name(self, name: str) -> None:
        """Sets the name of the collider item.

        Args:
            name: The new name for the collider item.
        """
        self.name.set_value(name)
        self.refresh_text()

    def set_approx(self, approx: str) -> None:
        """Sets the collision mesh approximation type for the collider item.

        Args:
            approx: The approximation type from the available mesh approximations and types.
        """
        self.approx.set_value(approx)
        self.refresh_text()

    def refresh_text(self) -> None:
        """Refreshes the searchable text by combining the current name and approximation values."""
        # TODO: should refresh text when item edited, so should add some on_item_changed callback in build function?
        self.text = " ".join((self.name.get_value_as_string(), self.approx.get_value_as_string()))


class ColliderModel(TreeViewWithPlacerHolderModel):
    """Model for managing collider items in a tree view structure.

    This model extends TreeViewWithPlacerHolderModel to provide specialized functionality for handling
    collider configuration items. It manages collider items that define collision mesh approximations
    for robot parts, supporting operations like adding, editing, and organizing collider configurations
    in a hierarchical tree structure.

    The model integrates with the collider creation workflow, allowing users to define collision
    properties for different robot components through a structured interface. It handles the display
    and manipulation of collider items, each containing information about the target geometry and
    the type of collision mesh approximation to apply.

    Args:
        items: Initial list of collider items to populate the model.
    """

    def __init__(self, items: list) -> None:
        super().__init__(items)
        self._edit_window = None

    def destroy(self) -> None:
        """Cleans up the ColliderModel by destroying the edit window and calling parent cleanup."""
        if self._edit_window:
            self._edit_window.destroy()
        self._edit_window = None

        super().destroy()

    def get_item_value_model_count(self, item: object) -> int:
        """The number of columns.

        Args:
            item: The collider item to get column count for.

        Returns:
            Number of columns in the tree view.
        """
        return 4

    def get_item_value_model(self, item: object, column_id: int) -> ui.AbstractValueModel:
        """Return value model.

            It's the object that tracks the specific value.
            In our case we use ui.SimpleStringModel for the first column
            and SimpleFloatModel for the second column.

        Args:
            item: The collider item to get the value model from.
            column_id: The column identifier (1 for name, 2 for approximation).

        Returns:
            The value model for the specified column, or None if not found.
        """
        if isinstance(item, ColliderItem):
            if column_id == 1:
                return item.name
            elif column_id == 2:
                return item.approx

    def edit_item(self, item: object) -> None:
        """Opens the edit window for the specified collider item.

        Args:
            item: The collider item to edit.
        """
        if not self._edit_window:
            self._edit_window = AddColliderWindow("Edit Colliders", self, item)
        else:
            self._edit_window._window.visible = True
            self._edit_window._update_ui(item)


class AddColliderWindow:
    """A modal window for adding or editing collision properties on robot parts.

    This window provides an interface for configuring collision mesh approximations on selected robot components.
    Users can specify the target mesh or parent object, choose from various collision approximation types
    (convex hull, triangle mesh, etc.), and create or modify collider configurations. The window supports both
    creation of new colliders and editing of existing ones, with validation to ensure proper configuration
    before applying changes.

    Args:
        window_title: Title displayed in the window header.
        model: The ColliderModel instance that manages collider data and operations.
        item: The ColliderItem to edit when opening in edit mode.
    """

    def __init__(self, window_title: str, model: object, item: object = None) -> None:
        self._is_edit = window_title == "Edit Colliders"
        self._window_height = 335
        self._collapsed_height = 235
        self._collider_model = model
        self._current_collider = item
        self._collider_name_model = None
        self._create_button = None
        self._create_close_button = None
        self._collider_approximation = None
        self._select_collider_target_window = None
        self.__subscription = self._collider_model.subscribe_item_changed_fn(self._model_changed)
        window_flags = ui.WINDOW_FLAGS_NO_SCROLLBAR | ui.WINDOW_FLAGS_NO_RESIZE
        self._window = ui.Window(window_title, width=600, height=self._collapsed_height, flags=window_flags)
        self._window.frame.set_build_fn(self._rebuild_frame)

    def destroy(self) -> None:
        """Clean up resources and destroy the AddColliderWindow.

        Destroys the subscription, collider target selection window, and main window.
        """
        self.__subscription = None
        if self._select_collider_target_window:
            self._select_collider_target_window.destroy()
        self._select_collider_target_window = None
        if self._window:
            self._window.destroy()
        self._window = None

    def _model_changed(self, model: object, item: object) -> None:
        """Handle changes to the collider model.

        Updates the UI when the collider model changes and the window is visible.

        Args:
            model: The collider model that changed.
            item: The specific item that was modified.
        """
        # when model item changes, we need to update the ui if the window is visible
        if item and self._window.visible:
            self._update_ui(item)

    def _update_ui(self, item: object) -> None:
        """Update the UI elements based on the current collider item.

        Sets the collider name field, checkbox state, and approximation dropdown based on the provided item.

        Args:
            item: The ColliderItem to display in the UI, or None to clear the UI.
        """
        self._current_collider = item
        self._collider_name_model.set_value(
            item.name.get_value_as_string() if item else "Pick the part to add collider to"
        )
        self._collider_name_widget.checked = bool(item)

        idx = APPROXIMATION_LIST.index(item.approx.get_value_as_string()) if item else 8
        self._collider_approximation.model.get_item_value_model().set_value(idx)
        # force the frame to update
        self._window.frame.invalidate_raster()

    def _update_item(self) -> None:
        """Update the current collider item with values from the UI.

        Syncs the collider name and approximation type from the UI controls to the current collider item.
        """
        # update item from ui
        self._current_collider.set_name(self._collider_name_model.get_value_as_string())
        self._current_collider.set_approx(
            APPROXIMATION_LIST[self._collider_approximation.model.get_item_value_model().get_value_as_int()]
        )
        self._collider_model._item_changed(self._current_collider)

    def on_collapsed_changed(self, collapsed: bool) -> None:
        """Handle window height changes when the collapsible frame is toggled.

        Adjusts the window height between collapsed and expanded states.

        Args:
            collapsed: Whether the frame is in collapsed state.
        """
        if collapsed:
            self._window.height = self._collapsed_height
        else:
            self._window.height = self._window_height

    def _update_collider_name(self, model: object) -> None:
        """Handle changes to the collider name field.

        Updates UI state and creates a new ColliderItem when a valid name is entered. Only active in non-edit mode.

        Args:
            model: The string model containing the collider name.

        Returns:
            None if in edit mode.
        """
        if self._is_edit:
            return

        name = model.get_value_as_string()
        checked = name != "Pick the part to add collider to" and name != ""

        self._collider_approximation.enabled = checked
        self._create_button.enabled = checked
        self._create_close_button.enabled = checked
        if checked:
            # use the default value for approximation and type
            self._current_collider = ColliderItem(name, APPROXIMATION_LIST[-1])
            self._current_collider.set_name(name)
        else:
            self._current_collider = None

    def _update_collider_approx(self, model: object, root_item: object) -> None:
        """Handle changes to the collider approximation dropdown.

        Updates the current collider's approximation type when the dropdown selection changes. Only active in non-edit mode.

        Args:
            model: The combo box model.
            root_item: The root item of the model.

        Returns:
            None if in edit mode.
        """
        if self._is_edit:
            return
        root_model = model.get_item_value_model(root_item)
        value = root_model.get_value_as_int()
        approx = APPROXIMATION_LIST[value]
        if approx and self._current_collider:
            self._current_collider.set_approx(approx)

    def _on_cancel_clicked(self) -> None:
        """Handle the Cancel button click.

        Hides the AddColliderWindow without saving changes.
        """
        self._window.visible = False

    def _on_create_clicked(self, closed: bool = False) -> None:
        """Handle the Create or Create & Close button click.

        Adds the current collider to the model and optionally closes the window.

        Args:
            closed: Whether to close the window after creating the collider.
        """
        if closed:
            self._window.visible = False

        self._collider_model.add_item(self._current_collider)
        # clear the current collider from the pop up window
        self._update_ui(None)

    def _on_save_clicked(self) -> None:
        """Handle the Save button click in edit mode.

        Updates the current collider item with UI values and closes the window.

        Returns:
            None if no current collider is set.
        """
        if not self._current_collider:
            return
        # write to model
        self._update_item()
        self._window.visible = False

    def _rebuild_frame(self) -> None:
        """Rebuilds the UI frame with collider configuration controls.

        Creates the complete user interface including radio buttons for collision target selection,
        name field for the collider part, approximation type dropdown, and action buttons.
        """
        with ui.ScrollingFrame():
            with ui.HStack(height=0):
                ui.Spacer(width=10)
                with ui.VStack(spacing=10, style=get_popup_window_style()):
                    infos = [
                        "Choose to apply collider to the mesh or the parent of the mesh",
                        "Pick the part to add collider to",
                        "Choose the type of the collision mesh to generate",
                    ]
                    info_frame(infos, self.on_collapsed_changed)

                    with ui.VStack(height=0):
                        ui.Label("Apply Collision To", height=0)
                        with ui.HStack(height=30, spacing=4):
                            collection = ui.RadioCollection()
                            ui.RadioButton(width=18, radio_collection=collection)
                            ui.Label("Mesh", width=60)
                            ui.RadioButton(width=18, radio_collection=collection)
                            ui.Label("Parent of Mesh")
                        ui.Spacer(height=10)
                        with ui.HStack(height=30):
                            self._collider_name_widget = ui.StringField(height=10, checked=bool(self._current_collider))
                            self._collider_name_model = self._collider_name_widget.model
                            name_value = (
                                self._current_collider.name.get_value_as_string()
                                if self._current_collider
                                else "Pick the part to add collider to"
                            )
                            self._collider_name_model.set_value(name_value)
                            self._collider_name_model.add_value_changed_fn(lambda m: self._update_collider_name(m))
                            ui.Image(
                                name="sample",
                                height=18,
                                width=22,
                                mouse_pressed_fn=lambda x, y, b, a: print("sample button clicked"),
                            )
                            ui.Image(
                                name="folder",
                                alignment=ui.Alignment.CENTER,
                                height=18,
                                width=22,
                                mouse_pressed_fn=lambda x, y, b, a: self.select_collider_target(),
                            )
                            ui.Spacer(width=5)
                        ui.Spacer(height=10)
                        with ui.HStack(height=30):
                            ui.Label("Collision Mesh Approximation", height=20, width=200)
                            index = (
                                APPROXIMATION_LIST.index(self._current_collider.approx.get_value_as_string())
                                if self._current_collider
                                else 8
                            )
                            self._collider_approximation = ui.ComboBox(
                                index, *APPROXIMATION_LIST, enabled=self._is_edit
                            )
                            self._collider_approximation.model.add_item_changed_fn(
                                lambda m, i: self._update_collider_approx(m, i)
                            )
                            ui.Spacer(width=5)
                        ui.Spacer(height=10)
                        if not self._is_edit:
                            with ui.HStack(height=22):
                                ui.Spacer()
                                with ui.HStack(width=77):
                                    ButtonWithIcon("Cancel", image_width=0, clicked_fn=self._on_cancel_clicked)
                                ui.Spacer(width=14)
                                with ui.HStack(width=80):
                                    self._create_button = ButtonWithIcon(
                                        "Create",
                                        name="add",
                                        image_width=12,
                                        enabled=False,
                                        clicked_fn=self._on_create_clicked,
                                    )
                                ui.Spacer(width=14)
                                with ui.HStack(width=120):
                                    self._create_close_button = ButtonWithIcon(
                                        "Create & Close",
                                        name="add",
                                        image_width=12,
                                        enabled=False,
                                        clicked_fn=lambda: self._on_create_clicked(True),
                                    )
                                ui.Spacer(width=5)
                        else:
                            with ui.HStack(height=22):
                                ui.Spacer()
                                with ui.HStack(width=77):
                                    ButtonWithIcon("Cancel", image_width=0, clicked_fn=self._on_cancel_clicked)
                                ui.Spacer(width=14)
                                with ui.HStack(width=80):
                                    self._create_button = ButtonWithIcon(
                                        "Save", name="save", image_width=12, clicked_fn=lambda: self._on_save_clicked()
                                    )
                                ui.Spacer(width=5)

    def select(self, selected_paths: list) -> None:
        """Handles selection of collider target paths from the asset picker.

        Updates the collider name field with the first selected path and marks the field as checked.

        Args:
            selected_paths: List of selected prim paths for collider targets.
        """
        self._select_collider_target_window.visible = False
        self._selected_paths = selected_paths
        if self._selected_paths:
            self._collider_name_model.set_value(self._selected_paths[0])
            self._collider_name_widget.checked = True

    def select_collider_target(self) -> None:
        """Opens the robot asset picker window for selecting collider targets.

        Creates a new RobotAssetPicker window if it doesn't exist, or shows the existing one.
        Filters for UsdGeom.Xform type prims and uses the select method as callback.
        """
        if not self._select_collider_target_window:
            stage = omni.usd.get_context().get_stage()
            self._select_collider_target_window = RobotAssetPicker(
                "Select Collider Target",
                stage,
                # TODO: may be type is physics colliders?
                filter_type_list=[UsdGeom.Xform],
                on_targets_selected=self.select,
                target_name="the part to add collider to",
            )
        self._select_collider_target_window.visible = True


class AddColliders:
    """A UI component for managing robot collider configuration in the Isaac Sim robot setup wizard.

    This class provides an interactive interface for adding and configuring physics colliders on robot parts.
    It displays a tree view of available robot components, allows users to select collision mesh approximation
    types, and manages the collider creation workflow. The component includes search and filtering
    capabilities, validation of collider configurations, and integration with the robot setup pipeline.

    The interface supports various collision mesh approximations including convex hull, convex decomposition,
    box, sphere, capsule, cylinder, and mesh types. Users can specify which robot parts require collision
    detection and choose appropriate approximation methods for optimal physics simulation performance.

    Args:
        visible: Whether the UI component is initially visible.
        *args: Additional positional arguments.
        **kwargs: Additional keyword arguments.
    """

    def __init__(self, visible: bool, *args: object, **kwargs: object) -> None:
        self.visible = visible
        self._add_collider_window = None
        self._tree_view = None
        self._treeview_empty_page = None
        self.__subscription = None
        self._next_step_button = None
        self._treeview_initial_height = 200
        self.frame = ui.Frame(visible=visible)
        self.frame.set_build_fn(self._build_frame)
        self._collider_model = None

    def destroy(self) -> None:
        """Destroys the AddColliders widget and cleans up resources.

        Cleans up internal components including the add collider window, tree view,
        collider model, and any active subscriptions.
        """
        self.__subscription = None
        if self._add_collider_window:
            self._add_collider_window.destroy()
        self._add_collider_window = None
        if self._tree_view:
            self._tree_view.destroy()
        self._tree_view = None
        if self._collider_model:
            self._collider_model.destroy()

    def _build_frame(self) -> None:
        """Builds the main UI frame for the add colliders widget.

        Creates the collapsible frame containing the colliders section with search functionality,
        tree view for displaying colliders, and the next step button for proceeding to joints and drivers.
        """
        with ui.CollapsableFrame("Colliders", build_header_fn=custom_header):
            with ui.ScrollingFrame():
                with ui.VStack(spacing=2, name="margin_vstack"):
                    separator("Add Colliders")
                    ui.Label(
                        "Add colliders to the robot sections that cannot be penetrated.",
                        height=0,
                        name="sub_separator",
                        word_wrap=True,
                    )
                    ui.Spacer(height=20)

                    # with ui.HStack(height=30):
                    #     ui.CheckBox(width=25).model.set_value(False)
                    #     with ui.VStack():
                    #         ui.Label("Allow Full Robot Self Collision", tooltip= "Self collision between two adjacent links are disabled by default. Check this box if you want self collision disabled on all links pairs.", width=0, height=0)
                    #         ui.Spacer()
                    # ui.Spacer(height=8)
                    with ui.ZStack(height=0):
                        ui.Rectangle(name="treeview")
                        with ui.HStack():
                            ui.Spacer(width=2)
                            with ui.VStack():
                                ui.Spacer(height=4)
                                with ui.HStack(spacing=4, height=0):
                                    # Search field
                                    self._search = SearchField(on_search_fn=self._filter_by_text)
                                    # Filter button
                                    self._filter_button = FilterButton([], width=20)
                                ui.Spacer(height=4)
                                self._build_tree_view()
                                ui.Spacer(height=4)
                            ui.Spacer(width=2)

                    ui.Spacer(height=10)
                    separator("Next: Add Joints and Drivers")
                    ui.Spacer(height=12)
                    self._next_step_button = ButtonWithIcon(
                        "Add Joints and Drivers",
                        name="next",
                        clicked_fn=lambda: next_step(
                            "Add Colliders", "Add Joints & Drivers", verify_fn=self._add_colliders
                        ),
                        height=44,
                        image_width=18,
                        enabled=True,
                    )

    def _filter_by_text(self, filters: object) -> None:
        """Filters the collider model items by text search.

        Args:
            filters: Text filters to apply to the collider model.
        """
        if self._collider_model:
            self._collider_model.filter_by_text(filters)

    def set_visible(self, visible: bool) -> None:
        """Sets the visibility of the add colliders widget.

        When making the widget visible, preprocesses the page to populate collider data.

        Args:
            visible: Whether the widget should be visible.
        """
        if self.frame:
            if visible:
                self._preprocess_page()
            self.frame.visible = visible

    def _preprocess_page(self) -> None:
        """Preprocesses the page by loading colliders from the robot on stage.

        Recursively searches through the /colliders scope prim to find mesh primitives
        and adds them as collider items to the model.

        Returns:
            None if the stage or robot is not available.
        """
        # get the colliders from the robot on stage
        self._robot = RobotRegistry().get()
        stage = omni.usd.get_context().get_stage()
        if not stage or not self._robot:
            return

        def __recursively_find_mesh(prim: Any, collider_model: ColliderModel) -> ColliderModel:
            if not prim:
                return None
            children = prim.GetChildren()
            if children:
                for child in children:
                    __recursively_find_mesh(child, collider_model)
            else:
                collider_name = "/".join(prim.GetPath().pathString.split("/")[2:])
                if prim.GetTypeName() in MESH_TYPES:
                    collider_model.add_item(ColliderItem(collider_name, prim.GetTypeName()))

            return collider_model

        if not self._collider_model:
            self._collider_model = ColliderModel([])

        mesh_scope_prim = stage.GetPrimAtPath("/colliders")
        updated_collider_model = __recursively_find_mesh(mesh_scope_prim, self._collider_model)
        self._collider_model = updated_collider_model

    def _add_colliders(self) -> None:
        """The actual applying collider part. Get the links from the table and add colliders accordingly."""
        if self._collider_model:
            for item in self._collider_model.get_item_children(None):
                if isinstance(item, ColliderItem):  ## as oppose to PlaceHolderItem
                    link_name = item.name.get_value_as_string()
                    approximation_type = item.approx.get_value_as_string()
                    apply_collider(link_name, approximation_type)

    def add_colliders(self) -> None:
        """Opens the add collider window for creating new colliders.

        Creates the AddColliderWindow if it doesn't exist, otherwise makes the existing window visible.
        """
        if not self._add_collider_window:
            self._add_collider_window = AddColliderWindow("Add Colliders", self._collider_model)
        self._add_collider_window._window.visible = True

    def _model_changed(self, model: object, item: object) -> None:
        """Handles changes to the collider model.

        Updates the UI state based on the number of items in the model, including enabling/disabling
        the next step button and showing/hiding the empty page placeholder.

        Args:
            model: The collider model that changed.
            item: The specific item that was modified.

        Returns:
            None if item or model is not provided.
        """
        if not item or not model:
            return

        # Check if there are items in the model
        if self._collider_model._searchable_num > 0:
            # Enable the next step button if there are items
            self._next_step_button.enabled = True
            # Hide the empty page if there are items
            if self._treeview_empty_page.visible:
                self._treeview_empty_page.visible = False
        elif self._collider_model._searchable_num == 0:
            # Disable the next step button if there are no items
            self._next_step_button.enabled = False
            # Show the empty page if there are no items
            if not self._treeview_empty_page.visible:
                self._treeview_empty_page.visible = True

        # Update the id_column delegate based on the number of items
        if self._collider_model._searchable_num < TreeViewIDColumn.DEFAULT_ITEM_NUM:
            return
        if self._collider_model._searchable_num > self.id_column.num:
            self.id_column.add_item()
        elif self._collider_model._searchable_num < self.id_column.num:
            self.id_column.remove_item()

    def treeview_empty_page(self) -> None:
        """Creates the empty state page displayed when no colliders exist.

        Shows instructional text guiding users to click the 'Add Colliders' button
        to begin the collider creation process.
        """
        self._treeview_empty_page = ui.VStack(visible=True, height=self._treeview_initial_height)
        with self._treeview_empty_page:
            ui.Spacer(height=ui.Fraction(3))
            ui.Label("Collider list is empty", alignment=ui.Alignment.CENTER, name="empty_treeview_title")
            ui.Spacer(height=ui.Fraction(2))
            ui.Label("There are no colliders created yet", alignment=ui.Alignment.CENTER)
            ui.Spacer(height=ui.Fraction(2))
            ui.Label("Click the 'Add Colliders' button", alignment=ui.Alignment.CENTER)
            ui.Label("to begin the collider creation process", alignment=ui.Alignment.CENTER)
            ui.Spacer(height=ui.Fraction(2))

        self._treeview_empty_page.visible = bool(self._collider_model._searchable_num == 0)

    def _build_tree_view(self) -> None:
        """Builds the tree view component for displaying colliders.

        Creates the tree view with headers, delegate, and empty page placeholder.
        Includes a draggable splitter for resizing the tree view height.
        """
        with ui.ZStack():
            scrolling_frame = ui.ScrollingFrame(name="treeview", height=self._treeview_initial_height)
            with scrolling_frame:
                if not self._collider_model:
                    self._collider_model = ColliderModel([])
                self.__subscription = self._collider_model.subscribe_item_changed_fn(self._model_changed)
                headers = ["Colliders", "Approximation"]
                self._delegate = TreeViewWithPlacerHolderDelegate(
                    headers, [APPROXIMATION_LIST], [2], self._collider_model
                )
                with ui.HStack():
                    self.id_column = TreeViewIDColumn()
                    with ui.ZStack():
                        self._tree_view = ui.TreeView(
                            self._collider_model,
                            delegate=self._delegate,
                            root_visible=False,
                            header_visible=True,
                            column_widths=[25, ui.Fraction(1), ui.Fraction(1), 25],
                        )
                        self.treeview_empty_page()
            placer = ui.Placer(drag_axis=ui.Axis.Y, offset_y=self._treeview_initial_height, draggable=True)
            with placer:
                with ui.ZStack(height=4):
                    splitter_highlight = ui.Rectangle(name="splitter_highlight")

            def move(y: Any) -> None:
                placer.offset_y = max(20, y.value)
                scrolling_frame.height = y

            placer.set_offset_y_changed_fn(move)
