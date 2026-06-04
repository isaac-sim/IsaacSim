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

"""Interactive demonstration extension for PhysX LIDAR sensors in Isaac Sim."""

import asyncio
import weakref

import carb
import omni
import omni.isaac.RangeSensorSchema as RangeSensorSchema
import omni.ui as ui
from isaacsim.core.utils.prims import delete_prim, get_prim_at_path
from isaacsim.core.utils.viewports import set_camera_view
from isaacsim.examples.browser import get_instance as get_browser_instance
from isaacsim.gui.components.ui_utils import btn_builder, combo_cb_scrolling_frame_builder, get_style, setup_ui_headers
from isaacsim.sensors.physx import _range_sensor
from pxr import Gf, Sdf, UsdGeom, UsdLux, UsdPhysics

EXTENSION_NAME = "LIDAR Info"


class Extension(omni.ext.IExt):
    """Extension class for the PhysX LIDAR sensor example.

    Provides an interactive demonstration of creating and configuring PhysX LIDAR sensors in Isaac Sim.
    Users can load LIDAR sensors, configure their properties, create test environments with obstacles,
    and view real-time data streams from active sensors.

    The extension creates a UI panel with controls to:
    - Load and configure LIDAR sensors with customizable parameters (FOV, resolution, range, rotation rate)
    - Create obstacle environments for testing sensor functionality
    - Display live data streams showing depth, azimuth, and zenith measurements in a formatted table
    - Visualize LIDAR rays and detection points in the viewport

    The LIDAR implementation uses PhysX line tracing for collision detection and requires a PhysicsScene
    to be present in the stage. The extension automatically configures the stage with appropriate settings
    including Z-up axis orientation and physics scene setup.
    """

    def on_startup(self, ext_id: str) -> None:
        """Initialize extension and UI elements.

        Args:
            ext_id: Extension identifier.
        """
        self._ext_id = ext_id
        self.example_name = "Physx Lidar Sensor"
        self.category = "Sensors"

        get_browser_instance().register_example(
            name=self.example_name,
            execute_entrypoint=lambda a=weakref.proxy(self): a.build_window(),
            ui_hook=lambda a=weakref.proxy(self): a.build_ui(),
            category=self.category,
        )

    def build_window(self) -> None:
        """Build the main window for the extension."""

    def build_ui(self) -> None:
        """Build the user interface components for the LIDAR sensor extension.

        Creates command panel with buttons to load LIDAR sensor, spawn obstacles, and display data stream.
        Sets up the UI headers and collapsible frame containing control buttons and data visualization components.
        """
        # This just defines the window we will use to access the lidar_info GUI.  Note that clicking on the menu item
        # does not create an instance of lidar_info; that is done by the extension when it is loaded by kit.  All this
        # menu does is show or hide our GUI we will use for interacting with lidar_info

        self._li = _range_sensor.acquire_lidar_sensor_interface()
        self.lidar = None
        self._timeline = omni.timeline.get_timeline_interface()

        self._load_lidar_button = None
        self._load_lidar_scene_button = None
        with ui.VStack(spacing=5, height=10):
            title = "Read a LIDAR Data Stream"
            doc_link = "https://docs.isaacsim.omniverse.nvidia.com/latest/sensors/isaacsim_sensors_physx_lidar.html"

            overview = "This example shows how to create a LIDAR, set its properties, and read data streaming from it. "
            overview += "First press the 'Load LIDAR' button and then press PLAY to simulate."
            overview += "\n\nPress the 'Open in IDE' button to view the source code."
            overview += "\nNote: The buttons above only work with a LIDAR made by the 'Load LIDAR' button; not existing ones in the stage."

            setup_ui_headers(self._ext_id, __file__, title, doc_link, overview, info_collapsed=False)

            frame = ui.CollapsableFrame(
                title="Command Panel",
                height=0,
                collapsed=False,
                style=get_style(),
                style_type_name_override="CollapsableFrame",
                horizontal_scrollbar_policy=ui.ScrollBarPolicy.SCROLLBAR_AS_NEEDED,
                vertical_scrollbar_policy=ui.ScrollBarPolicy.SCROLLBAR_ALWAYS_ON,
            )
            with frame:
                with ui.VStack(style=get_style(), spacing=5, height=0):
                    dict = {
                        "label": "Load LIDAR",
                        "type": "button",
                        "text": "Load",
                        "tooltip": "Loads a LIDAR Sensor and sets its properties",
                        "on_clicked_fn": self._on_spawn_lidar_button,
                    }
                    self._load_lidar_button = btn_builder(**dict)

                    dict = {
                        "label": "Load LIDAR Scene",
                        "type": "button",
                        "text": "Load",
                        "tooltip": "Loads a obstacles for the LIDAR to sense",
                        "on_clicked_fn": self._on_spawn_obstacles_button,
                    }
                    self._load_lidar_scene_button = btn_builder(**dict)

                    dict = {
                        "label": "Show Data Stream",
                        "type": "checkbox_scrolling_frame",
                        "default_val": [False, "No Data To Display"],
                        "tooltip": "Show incoming data from an active LIDAR",
                    }
                    self._info_cb, self._info_label = combo_cb_scrolling_frame_builder(**dict)

    def on_shutdown(self) -> None:
        """Perform cleanup when the extension shuts down.

        Deregisters the example from the browser instance and clears event subscriptions.
        """
        # Perform cleanup once the sample closes
        get_browser_instance().deregister_example(name=self.example_name, category=self.category)
        self._editor_event_subscription = None

    async def _spawn_lidar_function(self, task: object) -> None:
        """Create and configure a LIDAR sensor in the stage.

        Sets up the physics scene, creates the LIDAR prim with specified attributes including field of view,
        rotation rate, resolution, and range parameters. Configures the stage coordinate system and camera view.

        Args:
            task: Asyncio task for stage creation to wait for completion.
        """
        # Wait for stage clear to complete before creating LIDAR
        # Disable buttons while waiting to avoid issues if user keeps clicking button
        self._load_lidar_button.enabled = False
        self._load_lidar_scene_button.enabled = False
        done, pending = await asyncio.wait({task})
        if task in done:
            stage = omni.usd.get_context().get_stage()

            # Set up axis to z.  The LIDAR extension scans in the XZ plane, which is assumed to be perpendicular to the
            # rotational plane, and so to use the LIDAR as it is currently written, Z must be up.
            UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
            UsdGeom.SetStageMetersPerUnit(stage, 1.0)

            # Create the PhysicsScene.  The lidar is going to execute line trace calls in PhysX, and return a value based
            # on how far it travels before colliding with an object that is using the PhysX collision API.  Because of this,
            # to use the LIDAR extension, you MUST have a physics scene defined
            UsdPhysics.Scene.Define(stage, Sdf.Path("/World/physicsScene"))

            # create the LIDAR.  Before we can set any attributes on our LIDAR, we must first create the prim using our
            # LIDAR schema, and then populate it with the parameters we will be manipulating.  If you try to manipulate
            # a parameter before creating it, you will get a runtime error
            self.lidarPath = "/World/Lidar"
            self.lidar = RangeSensorSchema.Lidar.Define(stage, Sdf.Path(self.lidarPath))

            # Horizontal and vertical field of view in degrees
            self.lidar.CreateHorizontalFovAttr().Set(360.0)
            self.lidar.CreateVerticalFovAttr().Set(10)

            # Rotation rate in Hz
            self.lidar.CreateRotationRateAttr().Set(20.0)

            # Horizontal and vertical resolution in degrees.  Rays will be fired on the bin boundries defined by the
            # resolution.  If your FOV is 45 degrees and your resolution is 15 degrees, you will get rays at
            # 0, 15, 30, and 45 degrees.
            self.lidar.CreateHorizontalResolutionAttr().Set(1.0)
            self.lidar.CreateVerticalResolutionAttr().Set(1.0)

            # Min and max range for the LIDAR.  This defines the starting and stopping locations for the linetrace
            self.lidar.CreateMinRangeAttr().Set(0.4)
            self.lidar.CreateMaxRangeAttr().Set(100.0)

            # These attributes affect drawing the lidar in the viewport.  High Level Of Detail (HighLod) = True will draw
            # all rays.  If false it will only draw horizontal rays.  Draw Lidar Points = True will draw the actual
            # LIDAR rays in the viewport.
            self.lidar.CreateHighLodAttr().Set(True)
            self.lidar.CreateDrawPointsAttr().Set(False)
            self.lidar.CreateDrawLinesAttr().Set(False)

            # We set the attributes we created.  We could have just set the attributes at creation, but this was
            # more illustrative.  It's important to remember that attributes do not exist until you create them; even
            # if they are defined in the schema.
            self.lidar.GetRotationRateAttr().Set(0.5)
            self.lidar.GetDrawLinesAttr().Set(True)
            UsdGeom.Xformable(self.lidar.GetPrim()).AddTranslateOp().Set(Gf.Vec3f(0.0, 0.0, 0.250))

            # we want to make sure we can see the lidar we made, so we set the camera position and look target
            set_camera_view(eye=[5.00, 5.00, 5.00], target=[0.0, 0.0, 0.0], camera_prim_path="/OmniverseKit_Persp")

            # Re-enable buttons
            self._load_lidar_button.enabled = True
            self._load_lidar_scene_button.enabled = True

    def _on_spawn_lidar_button(self) -> None:
        """Handle the spawn LIDAR button click event.

        Creates a new stage and spawns a LIDAR sensor. Sets up event subscription for editor updates
        and clears the data stream display.
        """
        # wait for new stage before creating lidar
        task = asyncio.ensure_future(omni.usd.get_context().new_stage_async())
        asyncio.ensure_future(self._spawn_lidar_function(task))
        # refresh data stream box
        self._info_label.text = ""

        self._editor_event_subscription = carb.eventdispatcher.get_eventdispatcher().observe_event(
            event_name=omni.kit.app.GLOBAL_EVENT_UPDATE,
            on_event=self._on_editor_step,
            observer_name="isaacsim.sensors.physx.examples.lidar_info.Extension._on_editor_step",
        )

    def _on_editor_step(self, step: object) -> None:
        """Handle editor step updates to refresh LIDAR data display.

        Updates the data stream display when the info checkbox is enabled and timeline is playing.

        Args:
            step: Editor step event information.
        """
        if self._info_cb.get_value_as_bool():
            if self._timeline.is_playing():
                self._get_info_function()
        else:
            self._info_label.text = ""

    def _on_spawn_obstacles_button(self) -> None:
        """Handle the spawn obstacles button click event.

        Creates a cube obstacle and distant light in the scene for the LIDAR to detect.
        Applies collision API to the cube for physics interactions.
        """
        stage = omni.usd.get_context().get_stage()
        self.CubePath = "/World/Cube"
        offset = Gf.Vec3f(-2.00, 0.0, 0.500)
        size = 1.00

        # Define a light so we can see the obstacle better
        if get_prim_at_path("/DistantLight"):
            delete_prim("/DistantLight")
        distantLight = UsdLux.DistantLight.Define(stage, Sdf.Path("/DistantLight"))
        distantLight.CreateIntensityAttr(500)
        distantLight.AddRotateXYZOp().Set((-36, 36, 0))

        # To create a cube, we first define our geometry at our chosen path.  Then, becuase
        # we will need the primitive later, we query the prim from the stage. If the prim already exists, skip creation
        if stage.GetPrimAtPath(self.CubePath):
            return
        cubeGeom = UsdGeom.Cube.Define(stage, self.CubePath)
        cubePrim = stage.GetPrimAtPath(self.CubePath)

        # Remember!  Attributes do not exist until they are created.  Here we set the value to the non defualt at
        # creation.  Note that moving the cube to a different location involves adding a translation operation to
        # our primitive.
        cubeGeom.CreateSizeAttr(size)
        cubeGeom.AddTranslateOp().Set(offset)

        # In order for our cube to interact with the LIDAR, it needs to be able to colide with our physX line traces.
        # to do this, we give our cube the collision API, and set it's material and collision group.
        UsdPhysics.CollisionAPI.Apply(cubePrim)

    def _get_info_function(self, val: bool = False) -> None:
        """Retrieve and display LIDAR sensor data.

        Gets depth, zenith, and azimuth data from the LIDAR sensor and formats it into a table
        for display in the info label.

        Args:
            val: Optional boolean parameter for function behavior.
        """
        if not self.lidar:
            return
        maxDepth = self.lidar.GetMaxRangeAttr().Get()
        self._info_label.text = ""
        # The LIDAR itself exists as a C++ object.  In order to retrieve data from this object we need to call
        # C++ code, but this is handled for us through the use of python bindings.  Here we get the depth value of
        # each ray, and the spherical coordinates of each ray in (azimuth, zenith).
        depth = self._li.get_depth_data(self.lidarPath)
        zenith = self._li.get_zenith_data(self.lidarPath)
        azimuth = self._li.get_azimuth_data(self.lidarPath)

        # most of the below is string formatting in order to display our data in a nice table within our GUI.
        tableString = ""

        numCols = len(zenith)
        rowString = ""
        for i in range(numCols):
            rowString += "{" + str(i + 2) + ":." + str(5) + "f}   "
        rowString = "{0:16}  {1:10}" + rowString + "\n"

        tableString += rowString.format("Azimuth \\ Zenith", " | ", *zenith)
        tableString += "-" * len(tableString) + "\n"
        for row, cols in enumerate(depth):
            # The data on the c++ side is stored as uint16.  in order to get our depth values into centimeters, we
            # must first convert from uint16 into float on [0,1], and then scale to the maximum distance.
            entry = [ray * maxDepth / 65535.0 for ray in cols]
            tableString += rowString.format(f"{azimuth[row]:.5f}", " | ", *entry)

        self._info_label.text = tableString


__all__ = []
