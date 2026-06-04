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

"""Module containing the PhysX Generic Range Sensor example extension for Isaac Sim."""

import asyncio
import time
import weakref

import carb
import numpy as np
import omni
import omni.isaac.RangeSensorSchema as RangeSensorSchema
import omni.ui as ui
from isaacsim.core.utils.prims import delete_prim, get_prim_at_path
from isaacsim.core.utils.viewports import set_camera_view
from isaacsim.examples.browser import get_instance as get_browser_instance
from isaacsim.gui.components.ui_utils import btn_builder, get_style, setup_ui_headers, str_builder
from isaacsim.sensors.physx import _range_sensor
from pxr import Gf, Sdf, UsdGeom, UsdLux, UsdPhysics

EXTENSION_NAME = "Generic Info"


class Extension(omni.ext.IExt):
    """Extension for demonstrating PhysX Generic Range Sensor functionality.

    This extension provides an interactive example showcasing the Generic Range Sensor Python API in Isaac Sim.
    It demonstrates how to create a Generic Range Sensor, configure its properties, set custom scanning patterns,
    and stream data from the sensor. The extension includes UI controls for loading sensors, creating obstacle
    scenes, setting custom sensor patterns, and saving pattern visualization images.

    The extension registers with the Isaac Sim examples browser and provides a complete workflow for:
    - Creating PhysX-based range sensors with custom scanning patterns
    - Configuring sensor properties like range limits and sampling rates
    - Setting up physics scenes with collision objects
    - Streaming or batch processing sensor data
    - Visualizing scanning patterns through point cloud data
    - Saving pattern images for analysis

    The Generic Range Sensor uses PhysX line tracing to detect collisions and measure distances, making it
    suitable for applications requiring custom LIDAR-like sensing patterns. The extension supports both
    streaming mode for continuous data flow and repeating mode for batch processing of predefined patterns.
    """

    def on_startup(self, ext_id: str) -> None:
        """Initialize extension and UI elements.

        Args:
            ext_id: Extension identifier string.
        """
        self._ext_id = ext_id
        self.example_name = "Custom Pattern Range Sensor"
        self.category = "Sensors"

        get_browser_instance().register_example(
            name=self.example_name,
            execute_entrypoint=lambda a=weakref.proxy(self): a.build_window(),
            ui_hook=lambda a=weakref.proxy(self): a.build_ui(),
            category=self.category,
        )

    def on_shutdown(self) -> None:
        """Clean up extension resources and deregister example from browser."""
        get_browser_instance().deregister_example(name=self.example_name, category=self.category)
        self._editor_event_subscription = None

    def build_window(self) -> None:
        """Build the extension window interface."""

    def build_ui(self) -> None:
        """Build the user interface with command panel and output controls for the Generic Range Sensor example."""
        self._sensor = _range_sensor.acquire_generic_sensor_interface()
        self._timeline = omni.timeline.get_timeline_interface()

        self._pattern_set = False
        self._generic = False
        self._plot = False
        self._sampling_rate = 2.4e5  # number of samples per second
        self._plot_duration = 4  # seconds to collect sample before plotting
        self._record_start = time.perf_counter()
        self._streaming = True

        with ui.VStack(spacing=5, height=0):
            title = "Read a Generic Range Sensor Data Stream"
            doc_link = "https://docs.isaacsim.omniverse.nvidia.com/latest/sensors/isaacsim_sensors_physx_generic.html"

            overview = "This sample demonstrates the Generic range sensor python API for Isaac Sim. It shows how to create an Generic Range Sensor, set its properties, and read data streaming from it. "
            overview += "First press the 'Load Sensor' button and then press PLAY to simulate."
            overview += "\n\nPress the 'Open in IDE' button to view the source code."
            overview += "\nNote: The buttons above only work with an Generic range sensor made by the 'Load Sensor' button; not existing ones in the stage."

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
                        "label": "Load Sensor",
                        "type": "button",
                        "text": "Load",
                        "tooltip": "Loads a Range Sensor and sets its properties",
                        "on_clicked_fn": self._on_spawn_generic_button,
                    }
                    btn_builder(**dict)

                    dict = {
                        "label": "Load Scene",
                        "type": "button",
                        "text": "Load",
                        "tooltip": "Loads a obstacles for the Range Sensor to sense",
                        "on_clicked_fn": self._on_spawn_obstacles_button,
                    }
                    btn_builder(**dict)

                    dict = {
                        "label": "Set Sensor Pattern",
                        "type": "button",
                        "text": "Set",
                        "tooltip": "Sets a Custom Sensor pattern",
                        "on_clicked_fn": self._set_sensor_pattern,
                    }
                    btn_builder(**dict)

            self._output_frame = ui.CollapsableFrame(
                title="Save Sensor Pattern Images",
                height=0,
                collapsed=False,
                style=get_style(),
                style_type_name_override="CollapsableFrame",
                horizontal_scrollbar_policy=ui.ScrollBarPolicy.SCROLLBAR_AS_NEEDED,
                vertical_scrollbar_policy=ui.ScrollBarPolicy.SCROLLBAR_ALWAYS_ON,
            )
            with self._output_frame:
                with ui.VStack(style=get_style(), spacing=5, height=0):
                    dict = {
                        "label": "Output Directory",
                        "type": "stringfield",
                        "default_val": "/home/",
                        "tooltip": "Save the Scanning Pattern Image on the Wall",
                        "use_folder_picker": True,
                    }
                    self._filepath = str_builder(**dict)
                    btn_builder("", "button", "Save Pattern Image", "", self._on_save_pattern)

    async def _spawn_generic_function(self, task: object) -> None:
        """Create a Generic Range Sensor in the stage with physics scene setup.

        Args:
            task: Asyncio task for stage creation completion.
        """
        # Wait for stage clear to complete before creating Generic
        done, pending = await asyncio.wait({task})
        if task in done:
            stage = omni.usd.get_context().get_stage()

            # Set up axis to z
            UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
            UsdGeom.SetStageMetersPerUnit(stage, 1.0)

            # Create the PhysicsScene.  The generic is going to execute line trace calls in PhysX, and return a value based
            # on how far it travels before colliding with an object that is using the PhysX collision API.  Because of this,
            # to use the sensor extension, you MUST have a physics scene defined
            UsdPhysics.Scene.Define(stage, Sdf.Path("/World/physicsScene"))

            # create the Generic Sensor.  Before we can set any attributes on our sensor, we must first create the prim using our
            # Generic schema, and then populate it with the parameters we will be manipulating.  If you try to manipulate
            # a parameter before creating it, you will get a runtime error
            self._genericPath = "/World/GenericSensor"
            self._generic = RangeSensorSchema.Generic.Define(stage, Sdf.Path(self._genericPath))

            # Streaming data bool: True if constantly streaming lidar points in (e.g. non-repeatable patterns)
            # False if only scan in a batch once and repeat it
            self._generic.CreateStreamingAttr().Set(self._streaming)

            # Min and max range for the sensor.  This defines the starting and stopping locations for the linetrace
            self._generic.CreateMinRangeAttr().Set(0.4)
            self._generic.CreateMaxRangeAttr().Set(100.0)

            # sampling rate for the custom data
            self._generic.CreateSamplingRateAttr().Set(self._sampling_rate)

            # These attributes affect drawing the sensor in the viewport.
            # Draw Points = True will draw the actual rays in the viewport.
            self._generic.CreateDrawPointsAttr().Set(False)
            self._generic.CreateDrawLinesAttr().Set(False)

            # We set the attributes we created.  We could have just set the attributes at creation, but this was
            # more illustrative.  It's important to remember that attributes do not exist until you create them; even
            # if they are defined in the schema.
            self._generic.GetDrawLinesAttr().Set(True)
            # self._generic.AddTranslateOp().Set(Gf.Vec3f(0.0, 0.0, 25.0))

            # we want to make sure we can see the sensor we made, so we set the camera position and look target
            set_camera_view(eye=[-5.00, 5.00, 5.00], target=[0.0, 0.0, 0.0], camera_prim_path="/OmniverseKit_Persp")

            self._editor_event_subscription = carb.eventdispatcher.get_eventdispatcher().observe_event(
                event_name=omni.kit.app.GLOBAL_EVENT_UPDATE,
                on_event=self._on_editor_step,
                observer_name="isaacsim.sensors.physx.examples.generic_info.Extension._on_editor_step",
            )

    def _on_spawn_generic_button(self) -> None:
        """Handle the Load Sensor button click to create a new stage and spawn the Generic Range Sensor."""
        # wait for new stage before creating sensor
        task = asyncio.ensure_future(omni.usd.get_context().new_stage_async())
        asyncio.ensure_future(self._spawn_generic_function(task))

    def _on_spawn_obstacles_button(self) -> None:
        """Handle the Load Scene button click to create obstacles and lighting for the Range Sensor to detect."""
        stage = omni.usd.get_context().get_stage()
        self.CubePath = "/World/Wall"
        offset = Gf.Vec3f(2.00, 0.0, 0.0)
        size = 1

        # Define a light so we can see the obstacle better
        if get_prim_at_path("/DistantLight"):
            delete_prim("/DistantLight")
        distantLight = UsdLux.DistantLight.Define(stage, Sdf.Path("/DistantLight"))
        distantLight.CreateIntensityAttr(500)
        distantLight.AddRotateXYZOp().Set((-36, -36, 0))

        # To create a cube, we first define our geometry at our chosen path.  Then, becuase
        # we will need the primitive later, we query the prim from the stage. If the prim already exists, skip creation
        if stage.GetPrimAtPath(self.CubePath):
            return
        self.cubeGeom = UsdGeom.Cube.Define(stage, self.CubePath)
        self.cubePrim = stage.GetPrimAtPath(self.CubePath)

        # Remember!  Attributes do not exist until they are created.  Here we set the value to the non defualt at
        # creation.  Note that moving the cube to a different location involves adding a translation operation to
        # our primitive.
        self.cubeGeom.CreateSizeAttr(size)
        self.cubeGeom.AddTranslateOp().Set(offset)
        UsdGeom.XformCommonAPI(self.cubePrim).SetScale((1.00, 5.00, 4.00))

        # In order for our cube to interact with the LIDAR, it needs to be able to colide with our physX line traces.
        # to do this, we give our cube the collision API, and set it's material and collision group.
        UsdPhysics.CollisionAPI.Apply(self.cubePrim)

    def _set_sensor_pattern(self) -> None:
        """Set the custom scanning pattern for the Range Sensor based on streaming or repeating mode."""
        if self._streaming:
            self.sensor_pattern, self.origin_offsets = self._test_streaming_data()
        else:
            self.sensor_pattern, self.origin_offsets = self._test_repeating_data()

        self._pattern_set = True

    def _test_streaming_data(self) -> tuple:
        """Custom generated data for testing streaming data mode.

            Data profile: zigzag left to right, slowly going up and down.

        Returns:
            A tuple containing the sensor pattern array and origin offsets array.
        """
        # send data in batch that are at least large enough to run a few rendering frames without running out of data.
        # if batch_size > (sampling rate/rendering rate), the sensor will process all of the batch and ask for the next batch right before it runs out.
        # if batch_size < (sampling rate/rendering_rate), the sensor will scan only the provided rays in a given frame, which means it will be scanning slower than intended
        batch_size = int(1e6)  # size of each batch of data being processed
        half_batch = int(batch_size / 2)
        # example scanning pattern is a zigzag
        # each ray specified by an azimuth (horizontal angle measured from x-axis) and a zenith angle (vertical angle measured from z-axis)
        frequency = 10
        N_pts = int(batch_size / frequency / 2)
        # azimuth angle zigzag between the limits (frequency) times every batch
        azimuth = np.tile(
            np.append(np.linspace(-np.pi / 4, np.pi / 4, N_pts), np.linspace(np.pi / 4, -np.pi / 4, N_pts)), frequency
        )
        # zenith angle goes up and down once every batch
        zenith = np.append(
            np.linspace(-np.pi / 4, np.pi / 4, half_batch), np.linspace(np.pi / 4, -np.pi / 4, half_batch)
        )

        # custom pattern must be sent as an arrya of [azimuth, zenith] angles.
        sensor_pattern = np.stack((azimuth, zenith))

        # # # import data from file
        # sensor_pattern = np.loadtxt("filename.csv", delimiter=",")
        # batch_size = np.shape(sensor_pattern)[0]
        # sensor_pattern = np.deg2rad(sensor_pattern).T.copy()        ##  MUST USE .copy()

        # individual rays can have an offset at the origin
        # adding random offsets to the origin for the example pattern
        origin_offsets = 0.05 * np.random.random((batch_size, 3))
        # self.origin_offsets = np.zeros((batch_size,3))                  # no offsets

        return sensor_pattern, origin_offsets

    def _test_repeating_data(self) -> tuple:
        """Custom data to test repeating (non-streaming) mode.

            Data profile: zigzag left and right, half of it scanning high in zenith, the other half scanning low.
            Expected behavior: switch between the two sides scanning with no additional data being sent.

        Returns:
            A tuple containing the sensor pattern array and origin offsets array.
        """
        batch_size = int(1e6)  # size of each batch of data being processed
        half_batch = int(batch_size / 2)
        frequency = 10
        N_pts = int(batch_size / frequency / 2)
        azimuth = np.tile(
            np.append(np.linspace(-np.pi / 4, np.pi / 4, N_pts), np.linspace(np.pi / 4, -np.pi / 4, N_pts)), frequency
        )
        zenith = np.append(-0.5 * np.ones(half_batch), 0.5 * np.ones(half_batch))
        sensor_pattern = np.stack((azimuth, zenith))

        origin_offsets = 0.05 * np.random.random((batch_size, 3))

        return sensor_pattern, origin_offsets

    def _on_editor_step(self, step: object) -> None:
        """Updates the range sensor data processing during timeline playback.

        This method is called on each editor update step and handles sending new sensor pattern data batches when the
        sensor runs out of data, and collects point cloud data for pattern plotting.

        Args:
            step: The editor step event data.
        """
        if not self._timeline.is_playing():
            return

        if self._timeline.is_playing():
            if self._generic:
                if self._pattern_set:
                    if self._sensor.send_next_batch(
                        self._genericPath
                    ):  # send_next_batch will turn True if the sensor is running out data and needs more
                        print("sending more data")
                        self._sensor.set_next_batch_rays(
                            self._genericPath, self.sensor_pattern
                        )  # set the next batch data using set_next_batch_rays()
                        self._sensor.set_next_batch_offsets(
                            self._genericPath, self.origin_offsets
                        )  # (Optional) add indiviaul ray offsets if there are any

                    # one way to visually examine the scanning pattern is to plot the pattern that's hit the wall
                    if self._plot:
                        if (time.perf_counter() - self._record_start) < self._plot_duration:
                            self._point_cloud_data = np.append(
                                self._point_cloud_data, self._sensor.get_point_cloud_data(self._genericPath), axis=0
                            )
                        else:
                            self._plot = False
                            import threading

                            data = self._point_cloud_data
                            cube_scale = self.cubePrim.GetAttribute("xformOp:scale").Get()
                            cube_pos = self.cubePrim.GetAttribute("xformOp:translate").Get()
                            filepath = self._filepath.get_value_as_string()
                            threading.Thread(
                                target=self._plot_pattern,
                                args=(data, cube_scale, cube_pos, filepath),
                                daemon=True,
                            ).start()
                else:
                    print("sensor not added or pattern not set")

    def _on_save_pattern(self) -> None:
        """Initiates the collection and saving of sensor pattern data.

        Starts recording point cloud data from the range sensor for the configured plot duration. The collected data
        will be processed and saved as a pattern image when the recording period completes.
        """
        if not self._timeline.is_playing():
            print("press play first")
            return

        self._point_cloud_data = np.empty((0, 3))
        self._plot = True
        self._record_start = time.perf_counter()

    def _plot_pattern(self, data: object, cube_scale: object, cube_pos: object, filepath: str) -> None:
        """Creates and saves a visual representation of the sensor scanning pattern.

        Processes the collected point cloud data to generate a 2D image showing where the sensor rays hit the wall,
        then saves it as a PNG file to the specified output directory.

        Args:
            data: Point cloud data array containing the sensor hit points.
            cube_scale: Pre-fetched xformOp:scale from the wall prim.
            cube_pos: Pre-fetched xformOp:translate from the wall prim.
            filepath: Output directory path string.
        """
        import PIL.Image as Image
        import PIL.ImageDraw as ImageDraw

        # set up plot window
        window_length = 600
        window_height = 400
        origin = [window_length / 2.0, window_height / 2.0]

        # scale data with the wall size
        height_ratio = window_height / float(cube_scale[2])
        length_ratio = window_length / float(cube_scale[1])
        plot_scale = min(height_ratio, length_ratio)

        # get data that's hit the wall
        hit_yz = self._data_processing(data, cube_scale, cube_pos)

        # scale, axis_align, and center data to plot on PIL coordinate
        hit_yz = plot_scale * hit_yz
        plot_x = origin[0] - hit_yz[:, 0]
        plot_y = origin[1] - hit_yz[:, 1]

        plot_data = np.stack([plot_x, plot_y], axis=1)
        xy = plot_data.ravel()

        # actual plotting
        im = Image.new("RGB", (window_length, window_height))
        draw = ImageDraw.Draw(im)
        draw.point(xy.tolist(), fill=255)
        filename = filepath + "sensor_pattern.png"
        im.save(filename)

    @staticmethod
    def _data_processing(data: object, cube_scale: object, cube_pos: object) -> object:
        """Filters point cloud data to extract points that hit the wall surface.

        Args:
            data: Point cloud data array containing sensor hit points.
            cube_scale: Pre-fetched xformOp:scale from the wall prim.
            cube_pos: Pre-fetched xformOp:translate from the wall prim.

        Returns:
            Array of Y-Z coordinates for points that hit the wall, or empty array if no hits are found.
        """
        wall_loc = cube_pos[0] - np.sign(cube_pos[0]) * cube_scale[0] / 2

        # find in data the group that has the right offset
        hit_idx = np.where(np.isclose(data[:, 0], wall_loc, rtol=1e2))
        if len(hit_idx) == 0:
            print("no ray hit the wall")
            return np.array([])
        else:
            hit_pts = np.squeeze(data[hit_idx, 1:3])
            return hit_pts

    def data_processing(self, data: object) -> object:
        """Filters point cloud data to extract points that hit the wall surface.

        Calculates the wall surface location and filters the input data to return only the Y and Z coordinates
        of points that hit the wall at the correct X position.

        Args:
            data: Point cloud data array containing sensor hit points.

        Returns:
            Array of Y-Z coordinates for points that hit the wall, or empty array if no hits are found.
        """
        cube_pos = self.cubePrim.GetAttribute("xformOp:translate").Get()
        cube_scale = self.cubePrim.GetAttribute("xformOp:scale").Get()
        return self._data_processing(data, cube_scale, cube_pos)


__all__ = []
