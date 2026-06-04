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

"""Occupancy map UI extension for Isaac Sim."""

from __future__ import annotations

import asyncio
import gc
import os
from typing import Optional

import carb
import isaacsim.core.experimental.utils.stage as stage_utils
import omni
import omni.ext
import omni.kit.actions.core
import omni.kit.app
import omni.kit.usd.layers
import omni.ui as ui
from isaacsim.asset.gen.omap.bindings import _omap
from isaacsim.asset.gen.omap.utils import compute_coordinates, generate_image, update_location
from isaacsim.gui.components.ui_utils import (
    btn_builder,
    cb_builder,
    color_picker_builder,
    dropdown_builder,
    float_builder,
    multi_btn_builder,
    xyz_builder,
)
from omni.kit.menu.utils import (
    MenuHelperExtensionFull,
    MenuHelperWindow,
    MenuItemDescription,
    add_menu_items,
    remove_menu_items,
)
from omni.physx.scripts import utils
from PIL import Image
from pxr import Gf, Sdf, Usd, UsdGeom, UsdPhysics

# Constants
DEFAULT_CELL_SIZE_METERS = 0.05
DEFAULT_LOWER_BOUND = [-1.0, -1.0]
DEFAULT_UPPER_BOUND = [1.0, 1.0]
DEFAULT_Z_VALUE = 0.0
WINDOW_DEFAULT_WIDTH = 600
WINDOW_DEFAULT_HEIGHT = 400
FALLBACK_CELL_SIZE_METERS = 0.01
ROS_OCCUPIED_THRESHOLD = 0.65
ROS_FREE_THRESHOLD = 0.196
IMAGE_ROTATION_ANGLES = [180, 0, -90, 90]
IMAGE_ROTATION_LABELS = ["180", "0", "-90", "90"]
MIN_AXIS_SIZE_FALLBACK = 0.1
LINE_WIDTH_DEFAULT = 2.0
AXIS_LINE_WIDTH_MULTIPLIER = 2.0
FRAME_DELAY_COUNT = 3


async def _load_layout(layout_file: str, keep_windows_open: bool = False) -> None:
    """Loads a UI layout from a JSON file.

    Loads a QuickLayout configuration file and applies it to the current workspace.
    Includes delays to avoid conflicts with main window layout and sets up the
    viewport camera to use top view.

    Args:
        layout_file: Path to the layout JSON file.
        keep_windows_open: Whether to keep existing windows open when loading the layout.

    Example:

    .. code-block:: python

        >>> await _load_layout("/path/to/layout.json", keep_windows_open=True)
    """
    try:
        from omni.kit.quicklayout import QuickLayout

        # few frames delay to avoid the conflict with the layout of omni.kit.mainwindow
        for i in range(FRAME_DELAY_COUNT):
            await omni.kit.app.get_app().next_update_async()
        QuickLayout.load_file(layout_file, keep_windows_open)
        # few frames delay to load the window first
        for i in range(FRAME_DELAY_COUNT):
            await omni.kit.app.get_app().next_update_async()

        # make sure viewport 2's camera is "Top View"
        from omni.kit.viewport.utility import get_viewport_from_window_name

        # Get viewport API for a specific named viewport
        viewport_api = get_viewport_from_window_name("Viewport 2")
        if viewport_api:
            viewport_api.camera_path = Sdf.Path("/OmniverseKit_Top")

    except (ImportError, RuntimeError, AttributeError) as exc:
        carb.log_warn(
            f"Failed to load layout with advanced settings: {exc}. "
            "Attempting to load basic layout without viewport configuration."
        )
        try:
            from omni.kit.quicklayout import QuickLayout

            QuickLayout.load_file(layout_file)
        except Exception as fallback_exc:
            carb.log_error(
                f"Failed to load layout file {layout_file}: {fallback_exc}. "
                "Please check that the layout file exists and is valid JSON."
            )


class Extension(omni.ext.IExt, MenuHelperExtensionFull):
    """Occupancy Map UI Extension.

    This extension provides the user interface for generating occupancy maps from USD stages.
    It creates menu items in the Tools/Robotics menu and provides a layout template
    for the occupancy map generation workflow.
    """

    EXTENSION_NAME = "Occupancy Map"

    def on_startup(self, ext_id: str) -> None:
        """Called when the extension is enabled.

        Sets up menu items and layout templates for the occupancy map UI.

        Args:
            ext_id: The unique identifier for this extension instance.
        """
        self._ext_name = omni.ext.get_extension_name(ext_id)

        self.menu_startup(
            lambda: OccupancyMapWindow(),
            Extension.EXTENSION_NAME,
            Extension.EXTENSION_NAME,
            "Tools/Robotics",
        )

        action_registry = omni.kit.actions.core.get_action_registry()
        action_registry.register_action(
            self._ext_name,
            "open_omap_layout",
            lambda *_: self._open_layout_fn(ext_id),
            display_name="Occupancy Map Generation",
            description="Open the Occupancy Map Generation layout",
        )

        self._menu_items = [
            MenuItemDescription(name="Occupancy Map Generation", onclick_action=(self._ext_name, "open_omap_layout"))
        ]
        add_menu_items(self._menu_items, "Layouts")

    def _open_layout_fn(self, ext_id: str) -> asyncio.Task[None] | None:
        """Opens the occupancy map layout.

        Loads the predefined layout for occupancy map generation from the extension's
        data directory.

        Args:
            ext_id: The unique identifier for this extension instance.

        Returns:
            An asyncio future for the layout loading task, or None if the layout file doesn't exist.
        """
        ext_manager = omni.kit.app.get_app().get_extension_manager()
        extension_path = ext_manager.get_extension_path(ext_id)
        layout_file = f"{extension_path}/data/omap.json"
        if not os.path.exists(layout_file):
            carb.log_warn(f"Layout file {layout_file} does not exist")
            return None

        return asyncio.ensure_future(_load_layout(layout_file))

    def on_shutdown(self) -> None:
        """Called when the extension is disabled.

        Removes menu items and cleans up resources.
        """
        remove_menu_items(self._menu_items, "Layouts")
        action_registry = omni.kit.actions.core.get_action_registry()
        action_registry.deregister_action(self._ext_name, "open_omap_layout")
        self.menu_shutdown()


class OccupancyMapWindow(MenuHelperWindow):
    """Main window for occupancy map generation.

    This window provides the user interface for configuring and generating occupancy maps.
    It includes controls for setting the map origin, bounds, cell size, and other parameters,
    as well as buttons for calculating and visualizing the occupancy map.

    Sets up the window UI, initializes internal variables, and establishes connections
    to the occupancy map interface and USD stage.
    """

    def __init__(self) -> None:
        super().__init__(
            Extension.EXTENSION_NAME, width=WINDOW_DEFAULT_WIDTH, height=WINDOW_DEFAULT_HEIGHT, focused=True
        )
        self.deferred_dock_in("Property", ui.DockPolicy.CURRENT_WINDOW_IS_ACTIVE)

        # Initialize variables
        self._timeline = omni.timeline.get_timeline_interface()
        self._om = _omap.acquire_omap_interface()
        self._layers = omni.kit.usd.layers.get_layers()
        self._filepicker: Optional[object] = None
        self._yaml_filepicker: Optional[object] = None
        self._ros_yaml_text: Optional[str] = None
        self._map_bottom_left: Optional[object] = None
        self._map_scale: Optional[float] = None
        self._map_scale_to_meters: Optional[float] = None
        self._models = {}
        self._stage_open_callback: Optional[object] = None
        self._image: Optional[list[int]] = None
        self._im: Optional[object] = None

        self.prev_origin: list[float] = [0.0, 0.0]
        self.lower_bound: list[float] = list(DEFAULT_LOWER_BOUND)
        self.upper_bound: list[float] = list(DEFAULT_UPPER_BOUND)

        self.wait_bound_update: bool = False
        self.bound_update_case: int = 0

        self.units: float = DEFAULT_CELL_SIZE_METERS
        if omni.usd.get_context().get_stage():
            self.units = DEFAULT_CELL_SIZE_METERS / stage_utils.get_stage_units()[0]

        self.build_ui()

    def build_ui(self) -> None:
        """Builds the window UI.

        Creates all UI elements including origin controls, bounds inputs, cell size settings,
        and action buttons for map generation and visualization.
        """
        with self.frame:
            with ui.HStack(spacing=10):
                with ui.VStack(spacing=5, height=0):
                    change_fn = [self.on_update_location, self.on_update_location, self.on_update_location]
                    self._models["origin"] = xyz_builder(label="Origin", on_value_changed_fn=change_fn)

                    self._models["upper_bound"] = xyz_builder(
                        label="Upper Bound",
                        on_value_changed_fn=change_fn,
                        default_val=[self.upper_bound[0], self.upper_bound[1], DEFAULT_Z_VALUE],
                    )
                    self._models["lower_bound"] = xyz_builder(
                        label="Lower Bound",
                        on_value_changed_fn=change_fn,
                        default_val=[self.lower_bound[0], self.lower_bound[1], DEFAULT_Z_VALUE],
                    )

                    self._models["center_bound"] = multi_btn_builder(
                        "Positioning",
                        text=["Center to Selection", "Bound Selection"],
                        on_clicked_fn=[self._on_center_selection, self._on_bound_selection],
                    )

                    self._models["cell_size"] = float_builder(
                        label="Cell Size",
                        default_val=self.units,
                        min=0.001,
                        step=0.001,
                        format="%.3f",
                        tooltip=f"Size of each pixel in stage units in output occupancy map image. Default: {DEFAULT_CELL_SIZE_METERS}m",
                    )
                    self._models["cell_size"].add_value_changed_fn(self.on_update_cell_size)
                    self._models["compute"] = multi_btn_builder(
                        "Occupancy Map",
                        text=["Calculate", "Visualize Image"],
                        on_clicked_fn=[self._generate_map, self._generate_image],
                    )

                    self._models["physx_geom"] = cb_builder(
                        "Use PhysX Collision Geometry",
                        tooltip="If True, the current collision approximations are used, if False the original USD meshes are used. for PhysX based lidar use True for RTX lidar use False. Only visible meshes are used",
                        on_clicked_fn=None,
                        default_val=True,
                    )

        if self.visible:
            self._models["cell_size"].set_value(self.units)
            self._usd_context = omni.usd.get_context()
            self._stage_open_callback = carb.eventdispatcher.get_eventdispatcher().observe_event(
                event_name=self._usd_context.stage_event_name(omni.usd.StageEventType.OPENED),
                on_event=self._stage_open_callback_fn,
                observer_name="isaacsim.asset.gen.omap.ui._stage_open_callback_fn",
            )

    def _stage_open_callback_fn(self, event: object) -> None:
        """Callback when a new stage is opened.

        Updates the cell size to match the new stage's units.

        Args:
            event: The stage event that triggered this callback.
        """
        carb.log_info(f"New stage opened, setting cell_size to {self.units} to match stage units")
        self._models["cell_size"].set_value(self.units)

    def _on_center_selection(self) -> None:
        """Centers the map origin on the selected prims.

        Calculates the bounding box center of the selected prims and sets it as the
        new origin, then updates the bounds to maintain their position relative to the
        new origin.
        """
        origin = self.calculate_bounds(True, True)

        self._models["origin"][0].set_value(origin[0])
        self._models["origin"][1].set_value(origin[1])

        self.lower_bound, self.upper_bound = self.calculate_bounds(False, True)
        self.set_bound_value_ui()

    def calculate_bounds(
        self, origin_calc: bool, stationary_bounds: bool
    ) -> list[float] | tuple[list[float], list[float]]:
        """Calculates bounds based on selected prims.

        Computes either the origin point or the bounding box limits based on the selected
        prims in the stage. Can optionally keep bounds stationary relative to world space
        when the origin changes.

        Args:
            origin_calc: If True, calculates and returns the origin point. If False, calculates bounds.
            stationary_bounds: If True, adjusts bounds to maintain world position when origin changes.

        Returns:
            If origin_calc is True, returns the origin as [x, y].
            If origin_calc is False, returns a tuple of (lower_bound, upper_bound) as ([x, y], [x, y]).
        """
        origin_coord = [self._models["origin"][0].get_value_as_float(), self._models["origin"][1].get_value_as_float()]

        if not origin_calc and stationary_bounds:
            lower_bound = [
                self.lower_bound[0] + self.prev_origin[0] - origin_coord[0],
                self.lower_bound[1] + self.prev_origin[1] - origin_coord[1],
            ]

            upper_bound = [
                self.upper_bound[0] + self.prev_origin[0] - origin_coord[0],
                self.upper_bound[1] + self.prev_origin[1] - origin_coord[1],
            ]
            return lower_bound, upper_bound

        selected_prims = omni.usd.get_context().get_selection().get_selected_prim_paths()
        stage = omni.usd.get_context().get_stage()
        bbox_cache = UsdGeom.BBoxCache(Usd.TimeCode.Default(), includedPurposes=[UsdGeom.Tokens.default_])
        bbox_cache.Clear()
        total_bounds = Gf.BBox3d()

        if len(selected_prims) > 0:
            for prim_path in selected_prims:
                prim = stage.GetPrimAtPath(prim_path)
                bounds = bbox_cache.ComputeWorldBound(prim)
                total_bounds = Gf.BBox3d.Combine(total_bounds, Gf.BBox3d(bounds.ComputeAlignedRange()))
            range = total_bounds.GetBox()
            mid_point = range.GetMidpoint()
            if origin_calc:
                self.prev_origin = origin_coord
                origin_value = mid_point
                return origin_value

            min_point = range.GetMin()
            max_point = range.GetMax()

            lower_bound = [None] * 2
            upper_bound = [None] * 2

            lower_bound[0] = min_point[0] - origin_coord[0]
            lower_bound[1] = min_point[1] - origin_coord[1]

            upper_bound[0] = max_point[0] - origin_coord[0]
            upper_bound[1] = max_point[1] - origin_coord[1]

            return lower_bound, upper_bound
        else:
            if origin_calc:
                return [0] * 2
        return [0] * 2, [0] * 2

    def set_bound_value_ui(self) -> None:
        """Updates the UI fields with the current bound values.

        Sets the lower and upper bound UI fields with the internal bound values.
        Uses a special update case tracking mechanism to avoid triggering change
        callbacks during the update process.
        """
        self.wait_bound_update = True
        self.bound_update_case = 0
        self._models["lower_bound"][0].set_value(self.lower_bound[0])

        # Updating Case every time bound value is updating
        self.bound_update_case += 1

        self._models["lower_bound"][1].set_value(self.lower_bound[1])

        self.bound_update_case += 1

        self._models["upper_bound"][0].set_value(self.upper_bound[0])

        self.bound_update_case += 1

        self._models["upper_bound"][1].set_value(self.upper_bound[1])

        self.wait_bound_update = False

    def _on_bound_selection(self) -> None:
        """Sets the map bounds to match the selected prims.

        Calculates the bounding box of selected prims and uses it directly as the
        map bounds without adjusting for origin changes.
        """
        self.lower_bound, self.upper_bound = self.calculate_bounds(False, False)
        self.set_bound_value_ui()

    def on_update_location(self, value: float) -> None:
        """Callback when origin or bounds are changed in the UI.

        Updates the internal bounds and calls the occupancy map interface to update
        the visualization with the new transform parameters.

        Args:
            value: The new value from the UI control (not used directly).
        """
        if (
            self._models["lower_bound"][0].get_value_as_float() >= self._models["upper_bound"][0].get_value_as_float()
            or self._models["lower_bound"][1].get_value_as_float()
            >= self._models["upper_bound"][1].get_value_as_float()
            or self._models["lower_bound"][2].get_value_as_float() > self._models["upper_bound"][2].get_value_as_float()
        ):
            return
        if self.wait_bound_update:
            if self.bound_update_case == 0:
                self.lower_bound[0] = self._models["lower_bound"][0].get_value_as_float()
            elif self.bound_update_case == 1:
                self.lower_bound[1] = self._models["lower_bound"][1].get_value_as_float()
            elif self.bound_update_case == 2:
                self.upper_bound[0] = self._models["upper_bound"][0].get_value_as_float()
            elif self.bound_update_case == 3:
                self.upper_bound[1] = self._models["upper_bound"][1].get_value_as_float()
        else:
            self.lower_bound[0] = self._models["lower_bound"][0].get_value_as_float()
            self.lower_bound[1] = self._models["lower_bound"][1].get_value_as_float()
            self.upper_bound[0] = self._models["upper_bound"][0].get_value_as_float()
            self.upper_bound[1] = self._models["upper_bound"][1].get_value_as_float()

        update_location(
            self._om,
            [
                self._models["origin"][0].get_value_as_float(),
                self._models["origin"][1].get_value_as_float(),
                self._models["origin"][2].get_value_as_float(),
            ],
            [self.lower_bound[0], self.lower_bound[1], self._models["lower_bound"][2].get_value_as_float()],
            [self.upper_bound[0], self.upper_bound[1], self._models["upper_bound"][2].get_value_as_float()],
        )

    def on_update_cell_size(self, value: float) -> None:
        """Callback when cell size is changed in the UI.

        Updates the occupancy map interface with the new cell size.

        Args:
            value: The new cell size value from the UI control (not used directly).
        """
        self._om.set_cell_size(self._models["cell_size"].get_value_as_float())

    def _draw_instances(self) -> None:
        """Draws occupied cells as point instances in the USD stage.

        Creates a PointInstancer prim with cube instances at all occupied cell positions.
        Each cube is scaled to match the cell size and colored cyan. This provides a
        3D visualization of the occupied space in the occupancy map.
        """
        instancePath = "/occupancyMap/occupiedInstances"
        cubePath = "/occupancyMap/occupiedCube"
        pos_list = self._om.get_occupied_positions()
        scale = self._models["cell_size"].get_value_as_float() * 0.5
        color = (0.0, 1.0, 1.0)
        stage = omni.usd.get_context().get_stage()
        if stage.GetPrimAtPath(instancePath):
            stage.RemovePrim(instancePath)
        point_instancer = UsdGeom.PointInstancer(stage.DefinePrim(instancePath, "PointInstancer"))
        positions_attr = point_instancer.CreatePositionsAttr()
        if stage.GetPrimAtPath(cubePath):
            stage.RemovePrim(cubePath)
        occupiedCube = UsdGeom.Cube(stage.DefinePrim(cubePath, "Cube"))
        occupiedCube.AddScaleOp().Set(Gf.Vec3d(1, 1, 1) * scale)
        occupiedCube.CreateDisplayColorPrimvar().Set([color])

        point_instancer.CreatePrototypesRel().SetTargets([occupiedCube.GetPath()])
        proto_indices_attr = point_instancer.CreateProtoIndicesAttr()
        carb.log_info(f"Drawing {len(pos_list)} occupied cells as point instances")
        positions_attr.Set(pos_list)
        proto_indices_attr.Set([0] * len(pos_list))

    def _generate_map(self) -> None:
        """Generates the occupancy map.

        Validates bounds, stops the timeline, and generates the occupancy map using either
        PhysX collision geometry or original USD meshes based on the user's selection.
        The generation process runs asynchronously to allow for stage updates.

        For non-PhysX mode, a temporary session layer is created to modify collision
        properties without affecting the original stage.
        """
        if (
            self._models["lower_bound"][0].get_value_as_float() >= self._models["upper_bound"][0].get_value_as_float()
            or self._models["lower_bound"][1].get_value_as_float()
            >= self._models["upper_bound"][1].get_value_as_float()
            or self._models["lower_bound"][2].get_value_as_float() > self._models["upper_bound"][2].get_value_as_float()
        ):
            carb.log_warn("lower bound is >= upper bound, cannot calculate map")
            return

        self.on_update_location(0)
        self.on_update_cell_size(0)

        async def generate_task() -> None:
            self._timeline.stop()
            await omni.kit.app.get_app().next_update_async()
            if not self._models["physx_geom"].get_value_as_bool():
                layer = Sdf.Layer.CreateAnonymous("anon_occupancy_map")
                stage = omni.usd.get_context().get_stage()
                session = stage.GetSessionLayer()
                session.subLayerPaths.append(layer.identifier)
                with Usd.EditContext(stage, layer):
                    with Sdf.ChangeBlock():
                        for prim in stage.Traverse():
                            if prim.HasAPI(UsdPhysics.CollisionAPI) and prim.HasAPI(UsdPhysics.RigidBodyAPI):
                                utils.removePhysics(prim)
                    await omni.kit.app.get_app().next_update_async()
                    with Sdf.ChangeBlock():
                        for prim in stage.Traverse():
                            # Skip invisible
                            imageable = UsdGeom.Imageable(prim)
                            if imageable:
                                visibility = imageable.ComputeVisibility(Usd.TimeCode.Default())
                                if visibility == UsdGeom.Tokens.invisible:
                                    continue
                            # Skip meshes with no points
                            if prim.IsA(UsdGeom.Mesh):
                                usdMesh = UsdGeom.Mesh(prim)
                                attr = usdMesh.GetPointsAttr().Get()
                                if attr is None or len(attr) == 0:
                                    continue
                            if prim.HasAPI(UsdPhysics.CollisionAPI):
                                if prim.HasAPI(UsdPhysics.MeshCollisionAPI):
                                    collision_api = UsdPhysics.MeshCollisionAPI(prim)
                                    approx = collision_api.GetApproximationAttr().Get()
                                    if approx == "none":
                                        continue
                                if prim.IsA(UsdGeom.Gprim):
                                    if prim.IsInstanceable():
                                        UsdPhysics.CollisionAPI.Apply(prim)
                                        UsdPhysics.MeshCollisionAPI.Apply(prim)
                                    else:
                                        # Skip if we have errors here
                                        try:
                                            utils.setCollider(prim, "none")
                                        except Exception as e:
                                            continue
                            elif prim.IsA(UsdGeom.Xformable) and prim.IsInstanceable():
                                UsdPhysics.CollisionAPI.Apply(prim)
                                UsdPhysics.MeshCollisionAPI.Apply(prim)
                            elif prim.IsA(UsdGeom.Gprim):
                                UsdPhysics.CollisionAPI.Apply(prim)
                                UsdPhysics.MeshCollisionAPI.Apply(prim)

                self._timeline.play()
                await omni.kit.app.get_app().next_update_async()
                self._om.generate()
                await omni.kit.app.get_app().next_update_async()
                self._timeline.stop()
                session.subLayerPaths.remove(layer.identifier)
                layer = None
            else:
                self._timeline.play()
                await omni.kit.app.get_app().next_update_async()
                self._om.generate()
                await omni.kit.app.get_app().next_update_async()
                self._timeline.stop()

        asyncio.ensure_future(generate_task())

    def _fill_image(self) -> None:
        """Generates a colored image from the occupancy map buffer.

        Creates an RGBA image representation of the occupancy map, applies rotation,
        computes corner coordinates, and generates configuration text for ROS or other
        coordinate systems. Updates the image visualization in the UI.
        """
        dims = self._om.get_dimensions()
        scale = self._models["cell_size"].get_value_as_float()
        if scale <= 0:
            carb.log_warn(
                f"Invalid cell size: {scale}. Must be positive. "
                f"Using fallback value of {FALLBACK_CELL_SIZE_METERS} meters. "
                "Please adjust your cell size setting."
            )
            scale = FALLBACK_CELL_SIZE_METERS
        # Clockwise rotation
        current_image_rotation_index = self._models["rotation"].get_item_value_model().as_int
        rotate_image_angle = IMAGE_ROTATION_ANGLES[current_image_rotation_index]

        if current_image_rotation_index == 0:  # 180 degrees
            bottom_right, bottom_left, top_right, top_left, image_coords = compute_coordinates(self._om, scale)
        elif current_image_rotation_index == 1:  # 0 degrees
            top_left, top_right, bottom_left, bottom_right, image_coords = compute_coordinates(self._om, scale)
        elif current_image_rotation_index == 2:  # -90 degrees
            top_right, bottom_right, top_left, bottom_left, image_coords = compute_coordinates(self._om, scale)
        elif current_image_rotation_index == 3:  # 90 degrees
            bottom_left, top_left, bottom_right, top_right, image_coords = compute_coordinates(self._om, scale)

        occupied_col = []
        for item in self._models["occupied_color"].get_item_children():
            component = self._models["occupied_color"].get_item_value_model(item)
            occupied_col.append(int(component.get_value_as_float() * 255))

        freespace_col = []
        for item in self._models["freespace_color"].get_item_children():
            component = self._models["freespace_color"].get_item_value_model(item)
            freespace_col.append(int(component.get_value_as_float() * 255))

        unknown_col = []
        for item in self._models["unknown_color"].get_item_children():
            component = self._models["unknown_color"].get_item_value_model(item)
            unknown_col.append(int(component.get_value_as_float() * 255))

        self._image = generate_image(self._om, occupied_col, unknown_col, freespace_col)

        self._im = Image.frombytes("RGBA", (dims.x, dims.y), bytes(self._image))
        self._im = self._im.rotate(-rotate_image_angle, expand=True)
        self._image = list(self._im.tobytes())

        image_width = self._im.width
        image_height = self._im.height

        size = [0, 0, 0]

        size[0] = image_width * scale
        size[1] = image_height * scale

        self._rgb_byte_provider.set_bytes_data(self._image, [int(size[0] / scale), int(size[1] / scale)])
        self._image_frame.rebuild()

        image_details_text = f"Top Left: {top_left}\t\t Top Right: {top_right}\n Bottom Left: {bottom_left}\t\t Bottom Right: {bottom_right}"
        image_details_text += f"\nCoordinates of top left of image (pixel 0,0) as origin, + X down, + Y right:\n{float(image_coords[0][0]), float(image_coords[1][0])}"
        image_details_text += f"\nImage size in pixels: {int(size[0] / scale)}, {int(size[1] / scale)}"

        scale_to_meters = 1.0 / stage_utils.get_stage_units()[0]

        self._map_bottom_left = bottom_left
        self._map_scale = scale
        self._map_scale_to_meters = scale_to_meters

        stage = omni.usd.get_context().get_stage()
        root = stage.GetRootLayer()
        default_image_stem = root.GetDisplayName().rsplit(".", 1)[0]

        image_name_model = self._models.get("image_name")
        if image_name_model is not None:
            stem = image_name_model.as_string.strip()
            image_name = (stem if stem else default_image_stem) + ".png"
        else:
            image_name = default_image_stem + ".png"

        ros_yaml_file_text = "image: " + image_name
        ros_yaml_file_text += f"\nresolution: {float(scale / scale_to_meters)}"
        ros_yaml_file_text += (
            f"\norigin: [{float(bottom_left[0] / scale_to_meters)}, {float(bottom_left[1] / scale_to_meters)}, 0.0000]"
        )
        ros_yaml_file_text += "\nnegate: 0"
        ros_yaml_file_text += f"\noccupied_thresh: {ROS_OCCUPIED_THRESHOLD}"
        ros_yaml_file_text += f"\nfree_thresh: {ROS_FREE_THRESHOLD}"

        self._ros_yaml_text = ros_yaml_file_text

        current_data_output_index = self._models["config_type"].get_item_value_model().as_int
        if current_data_output_index == 0:
            self._models["config_data"].set_value(ros_yaml_file_text)
        elif current_data_output_index == 1:
            self._models["config_data"].set_value(image_details_text)

    def _update_yaml(self) -> None:
        """Rebuilds the YAML text using the current image filename field value."""
        if self._map_bottom_left is None:
            carb.log_warn("No map data available. Please generate the image first.")
            return

        stem = self._models["image_name"].as_string.strip()
        if not stem:
            return

        ros_yaml_file_text = "image: " + stem + ".png"
        ros_yaml_file_text += f"\nresolution: {float(self._map_scale / self._map_scale_to_meters)}"
        ros_yaml_file_text += f"\norigin: [{float(self._map_bottom_left[0] / self._map_scale_to_meters)}, {float(self._map_bottom_left[1] / self._map_scale_to_meters)}, 0.0000]"
        ros_yaml_file_text += "\nnegate: 0"
        ros_yaml_file_text += f"\noccupied_thresh: {ROS_OCCUPIED_THRESHOLD}"
        ros_yaml_file_text += f"\nfree_thresh: {ROS_FREE_THRESHOLD}"

        self._ros_yaml_text = ros_yaml_file_text

        current_data_output_index = self._models["config_type"].get_item_value_model().as_int
        if current_data_output_index == 0:
            self._models["config_data"].set_value(ros_yaml_file_text)

    def save_image(self, file: str, folder: str) -> None:
        """Saves the occupancy map image to a PNG file.

        Args:
            file: The filename for the saved image (will add .png extension if not present).
            folder: The directory path where the image should be saved.
        """
        if self._image is None or not hasattr(self, "_im"):
            carb.log_warn("No image available to save. Please generate visualization first.")
            return

        try:
            image_width = self._im.width
            image_height = self._im.height
            file = file if file[-4:].lower() == ".png" else f"{file}.png"
            im = Image.frombytes("RGBA", (image_width, image_height), bytes(self._image))
            save_path = os.path.join(folder, file)
            carb.log_info(f"Saving occupancy map image to {save_path}")
            im.save(save_path)
            carb.log_info(f"Image saved successfully")
        except Exception as e:
            carb.log_error(f"Failed to save image: {e}")
        finally:
            if self._filepicker is not None:
                self._filepicker.hide()

    def save_file(self) -> None:
        """Opens a file picker dialog for saving the occupancy map image.

        Creates a file picker dialog that filters for PNG files and allows the user
        to select a save location for the generated occupancy map image.
        """
        image_name_model = self._models.get("image_name")
        if image_name_model is not None and not image_name_model.as_string.strip():
            carb.log_warn("Image filename is empty. Please enter a filename before saving.")
            return

        from omni.kit.widget.filebrowser import FileBrowserItem
        from omni.kit.window.filepicker import FilePickerDialog

        def _on_filter_png_files(item: FileBrowserItem) -> bool:
            """Callback to filter the choices of file names in the open or save dialog.

            Args:
                item: The file browser item to check.

            Returns:
                True if the item should be shown, False otherwise.
            """
            if not item or item.is_folder:
                return True
            # Show only files with listed extensions
            return os.path.splitext(item.path)[1] == ".png"

        self._filepicker = None
        self._filepicker = FilePickerDialog(
            "Save .png image",
            allow_multi_selection=False,
            apply_button_label="Save",
            click_apply_handler=self.save_image,
            item_filter_options=[".png Files (*.png, *.PNG)"],
            item_filter_fn=_on_filter_png_files,
        )
        stage = omni.usd.get_context().get_stage()
        default_stem = stage.GetRootLayer().GetDisplayName().rsplit(".", 1)[0]
        image_name_model = self._models.get("image_name")
        if image_name_model is not None:
            stem = image_name_model.as_string.strip() or default_stem
        else:
            stem = default_stem
        self._filepicker.set_filename(stem)

    def save_yaml(self, file: str, folder: str) -> None:
        """Saves the ROS occupancy map YAML parameters to a file.

        Args:
            file: The filename for the saved YAML (will add .yaml extension if not present).
            folder: The directory path where the YAML file should be saved.
        """
        if self._ros_yaml_text is None:
            carb.log_warn("No YAML data available. Please generate visualization first.")
            return

        try:
            if os.path.splitext(file)[1].lower() not in (".yaml", ".yml"):
                file = f"{file}.yaml"
            save_path = os.path.join(folder, file)
            carb.log_info(f"Saving occupancy map YAML to {save_path}")
            with open(save_path, "w") as f:
                f.write(self._ros_yaml_text)
            carb.log_info("YAML saved successfully")
        except Exception as e:
            carb.log_error(f"Failed to save YAML: {e}")
        finally:
            if self._yaml_filepicker is not None:
                self._yaml_filepicker.hide()

    def save_yaml_file(self) -> None:
        """Opens a file picker dialog for saving the occupancy map YAML parameters file."""
        image_name_model = self._models.get("image_name")
        if image_name_model is not None and not image_name_model.as_string.strip():
            carb.log_warn("Image filename is empty. Please enter a filename before saving.")
            return

        from omni.kit.widget.filebrowser import FileBrowserItem
        from omni.kit.window.filepicker import FilePickerDialog

        def _on_filter_yaml_files(item: FileBrowserItem) -> bool:
            if not item or item.is_folder:
                return True
            return os.path.splitext(item.path)[1].lower() in (".yaml", ".yml")

        self._yaml_filepicker = None
        self._yaml_filepicker = FilePickerDialog(
            "Save .yaml file",
            allow_multi_selection=False,
            apply_button_label="Save",
            click_apply_handler=self.save_yaml,
            item_filter_options=[".yaml Files (*.yaml, *.yml)"],
            item_filter_fn=_on_filter_yaml_files,
        )
        stage = omni.usd.get_context().get_stage()
        default_stem = stage.GetRootLayer().GetDisplayName().rsplit(".", 1)[0]
        image_name_model = self._models.get("image_name")
        if image_name_model is not None:
            stem = image_name_model.as_string.strip() or default_stem
        else:
            stem = default_stem
        self._yaml_filepicker.set_filename(stem)

    def rebuild_frame(self) -> None:
        """Rebuilds the image visualization frame.

        Creates UI elements to display the occupancy map image and a save button.
        This method is called whenever the image needs to be updated in the UI.
        """
        if self._image is not None:
            with ui.VStack():
                omni.ui.ImageWithProvider(self._rgb_byte_provider)
                with ui.HStack(height=0):
                    ui.Button("Save Image", clicked_fn=self.save_file, height=0)
                    ui.Button("Save YAML", clicked_fn=self.save_yaml_file, height=0)

    def _generate_image(self) -> None:
        """Creates the visualization window for the occupancy map.

        Opens a new window with controls for selecting colors for occupied, free, and
        unknown cells, image rotation, and coordinate system output format. Generates
        the initial image visualization immediately upon opening.
        """
        self._image = None
        # check to make sure image has data first
        dims = self._om.get_dimensions()
        if dims.x == 0 or dims.y == 0:
            carb.log_warn(
                "Occupancy map is empty, press CALCULATE first and make sure there is collision geometry in the mapping bounds"
            )
            return
        stage = omni.usd.get_context().get_stage()
        root = stage.GetRootLayer()
        default_image_stem = root.GetDisplayName().rsplit(".", 1)[0]

        self._rgb_byte_provider = omni.ui.ByteImageProvider()
        self.visualize_window = omni.ui.Window("Occupancy Map Visualization", width=500, height=600)
        with self.visualize_window.frame:
            with ui.VStack(spacing=5):
                with ui.VStack(height=0, spacing=5):
                    kwargs = {"label": "Occupied Color", "default_val": [0, 0, 0, 1]}
                    self._models["occupied_color"] = color_picker_builder(**kwargs)
                    kwargs = {"label": "Freespace Color", "default_val": [1, 1, 1, 1]}
                    self._models["freespace_color"] = color_picker_builder(**kwargs)
                    kwargs = {"label": "Unknown Color", "default_val": [0.5, 0.5, 0.5, 1]}
                    self._models["unknown_color"] = color_picker_builder(**kwargs)
                    self._models["rotation"] = dropdown_builder(
                        label="Rotate Image",
                        items=IMAGE_ROTATION_LABELS,
                        tooltip="Clockwise rotation of image in degrees",
                    )
                    self._models["config_type"] = dropdown_builder(
                        label="Coordinate Type",
                        items=["ROS Occupancy Map Parameters File (YAML)", "Coordinates in Stage Space"],
                        tooltip="Type of config output generated",
                    )
                    self._models["generate"] = btn_builder(
                        label="Occupancy map", text="Re-Generate Image", on_clicked_fn=self._fill_image
                    )
                    with ui.HStack(height=0, spacing=5):
                        ui.Label(
                            "Map Name",
                            width=120,
                            alignment=ui.Alignment.LEFT_CENTER,
                            tooltip="Stem used as the default filename when saving the PNG image or YAML file, and written into the YAML 'image' field.",
                        )
                        self._models["image_name"] = ui.StringField(
                            name="StringField",
                            width=ui.Fraction(1),
                            height=0,
                            alignment=ui.Alignment.LEFT_CENTER,
                        ).model
                        self._models["image_name"].set_value(default_image_stem)
                        self._models["image_name"].add_value_changed_fn(lambda _: self._update_yaml())
                    self._models["config_data"] = ui.StringField(height=100, multiline=True).model
                self._image_frame = ui.Frame()
                self._image_frame.set_build_fn(self.rebuild_frame)
        # generate image immediately when this window appears
        self._fill_image()

    def destroy(self) -> None:
        """Cleans up the window and releases resources.

        Releases the occupancy map interface, clears internal references, and calls
        the parent class destroy method. Also triggers garbage collection to ensure
        proper cleanup of resources.
        """
        self._stage_open_callback = None
        # Initialize variables
        self._timeline = None
        _omap.release_omap_interface(self._om)
        self._om = None
        self._layers = None
        if self._filepicker is not None:
            self._filepicker.hide()
            self._filepicker = None
        if self._yaml_filepicker is not None:
            self._yaml_filepicker.hide()
            self._yaml_filepicker = None
        self._ros_yaml_text = None
        self._map_bottom_left = None
        self._map_scale = None
        self._map_scale_to_meters = None
        self._models = {}
        self._image = None

        super().destroy()
        gc.collect()
