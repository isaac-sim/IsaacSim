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

"""Demonstrates the LightBeam sensor functionality in Isaac Sim with an interactive example showing real-time object detection and distance measurement using physics-based ray casting."""

import asyncio
import weakref

import carb
import carb.eventdispatcher
import omni
import omni.graph.core as og
import omni.kit.commands
import omni.physics.core
import omni.ui as ui
import omni.usd
from isaacsim.core.utils.prims import delete_prim, get_prim_at_path
from isaacsim.core.utils.viewports import set_camera_view
from isaacsim.examples.browser import get_instance as get_browser_instance
from isaacsim.gui.components.ui_utils import LABEL_WIDTH, get_style, setup_ui_headers
from isaacsim.sensors.physx import _range_sensor
from isaacsim.storage.native import get_assets_root_path
from pxr import Gf, Sdf, UsdGeom, UsdLux, UsdPhysics

EXTENSION_NAME = "LightBeam Sensor Example"


class LightBeamSensorDemo(omni.ext.IExt):
    """A demonstration extension for the LightBeam sensor functionality in Isaac Sim.

    This extension provides an interactive example showing how to use the LightBeam sensor to detect
    objects and measure distances using physics-based ray casting. The demo creates a scene with a
    moveable cube and a LightBeam sensor that continuously scans for hits, displaying real-time data
    including beam hit status, linear depth measurements, and hit positions.

    The extension integrates with the Isaac Sim examples browser and provides a user interface that
    visualizes sensor output data. When activated, it creates a physics scene with a cube that can be
    dragged around, allowing users to observe how the sensor responds to object movement and positioning.

    Key features:
    - Real-time visualization of 5 light beam rays with color-coded display
    - Live updates of beam hit detection, linear depth, and hit position data
    - Interactive cube manipulation for testing sensor behavior
    - Integration with Isaac Sim's physics simulation system
    - Automatic scene setup with proper lighting and camera positioning
    - Action graph configuration for sensor data processing and visualization
    """

    def on_startup(self, ext_id: str) -> None:
        """Initializes the LightBeam sensor extension.

        Args:
            ext_id: The extension identifier.
        """
        ext_manager = omni.kit.app.get_app().get_extension_manager()
        self._ext_id = ext_id
        self._extension_path = ext_manager.get_extension_path(ext_id)
        self._window = None

        get_browser_instance().register_example(
            name="LightBeam Sensor",
            execute_entrypoint=self.build_window,
            ui_hook=lambda a=weakref.proxy(self): a.build_ui(),
            category="Sensors",
        )

    def build_window(self) -> None:
        """Placeholder method for building the extension window."""

    def _on_stage_closed(self, event: object) -> None:
        """Stage closed event callback.

        Args:
            event: The stage closed event.
        """
        self.on_closed()

    def build_ui(self) -> None:
        """Builds the user interface for the LightBeam sensor example.

        Creates the main UI with header information, documentation links, and a button to load
        the sensor demonstration scene.
        """
        with ui.VStack(spacing=5, height=0):

            title = "LightBeam Sensor Example"
            doc_link = "https://docs.isaacsim.omniverse.nvidia.com/latest/sensors/isaacsim_sensors_physx_lightbeam.html"

            overview = "This Example shows the output of the LightBeam sensor."
            overview += "The LightBeam sensor constantly scans for any hits to its beams and outputs linear depth and hit position."
            overview += "\nPress PLAY to start the simulation and visualize the light beams, left click the cube/sensor origin to drag it around."
            overview += "\n\nPress the 'Open in IDE' button to view the source code."

            setup_ui_headers(self._ext_id, __file__, title, doc_link, overview, info_collapsed=False)
            ui.Button("Load Scene", clicked_fn=lambda: self._load_scene())

    def _load_scene(self) -> None:
        """Loads and initializes the LightBeam sensor demonstration scene.

        Sets up the sensor interface, physics simulation subscription, UI window with real-time
        data display labels for beam hits, linear depth, and hit positions for each of the 5 rays.
        """
        if self._window:
            # clear existing window
            self.on_closed()

        self.beam_hit_labels = []
        self.linear_depth_labels = []
        self.hit_pos_labels = []
        self.num_rays = 5
        self.meters_per_unit = 1.00

        self._ls = _range_sensor.acquire_lightbeam_sensor_interface()

        self._timeline = omni.timeline.get_timeline_interface()
        self.sub = omni.physics.core.get_physics_simulation_interface().subscribe_physics_on_step_events(
            pre_step=False, order=0, on_update=self._on_update
        )

        self.colors = [
            0xFFBBBBFF,
            0xFFBBFFBB,
            0xBBFFBBBB,
            0xBBAAEEFF,
            0xAABBFFEE,
            0xFFEEAABB,
            0xFFC8D5D0,
            0xFFC89BD0,
            0xFFAF9BA7,
            0xFFA4B99A,
        ]

        style = {"background_color": 0xFF888888, "color": 0xFF333333, "secondary_color": self.colors[0]}

        self._window = ui.Window(
            title=EXTENSION_NAME, width=800, height=0, visible=True, dockPreference=ui.DockPreference.LEFT_BOTTOM
        )
        self._window.set_visibility_changed_fn(self._on_visibility_changed)

        with self._window.frame:
            with ui.VStack(style=get_style(), spacing=3):
                for i in range(self.num_rays):
                    # Displaying light beam number and data for each light beam
                    with ui.HStack():
                        ui.Label(
                            f"Lightbeam {i+1}",
                            width=LABEL_WIDTH / 2,
                            tooltip="Light beam number",
                            style={"secondary_color": self.colors[i]},
                        )

                        # Displaying beam hit status (initially empty, to be updated in _on_update)
                        self.beam_hit_labels.append(
                            ui.Label(
                                "",
                                width=LABEL_WIDTH / 1.7,
                                tooltip="beam hit t/f",
                                style={"secondary_color": self.colors[i]},
                            )
                        )

                        # Displaying linear depth (initially empty, to be updated in _on_update)
                        self.linear_depth_labels.append(
                            ui.Label(
                                "",
                                width=LABEL_WIDTH * 1.3,
                                tooltip="linear depth in meters",
                                style={"secondary_color": self.colors[i]},
                            )
                        )

                        # Displaying hit position with specific labels for x, y, and z (initially empty, to be updated in _on_update)
                        self.hit_pos_labels.append(
                            ui.Label(
                                "",
                                width=LABEL_WIDTH,
                                tooltip="hit position in meters",
                                style={"secondary_color": self.colors[i]},
                            )
                        )

        asyncio.ensure_future(self.create_scenario())

    def on_shutdown(self) -> None:
        """Cleans up resources when the extension is shut down.

        Closes any open windows and deregisters the example from the browser.
        """
        self.on_closed()
        # remove_menu_items(self._menu_items, "Isaac Examples")
        get_browser_instance().deregister_example(name="LightBeam Sensor", category="Sensors")

    def _on_visibility_changed(self, visible: bool) -> None:
        """Handles window visibility changes.

        Args:
            visible: Whether the window is visible.
        """
        if not visible:
            self.on_closed()

    def on_closed(self) -> None:
        """Cleans up resources when the demonstration window is closed.

        Destroys the window, unsubscribes from physics events, and clears timeline and stage
        event subscriptions.
        """
        if self._window:
            self.sub = None
            self._timeline = None
            self._stage_event_subscription = None
            self._window.destroy()
            self._window = None

    def _on_update(self, dt: float, context: object) -> None:
        """Updates the UI with real-time LightBeam sensor data during simulation.

        Retrieves linear depth, hit position, and beam hit data from the sensor and updates
        the corresponding UI labels for each ray.

        Args:
            dt: Delta time since last update.
            context: The physics simulation context.
        """
        if self._timeline.is_playing():
            lin_depth = self._ls.get_linear_depth_data(self.sensor_path)
            hit_pos = self._ls.get_hit_pos_data(self.sensor_path)
            # cast from uint8 to bool
            beam_hit = self._ls.get_beam_hit_data(self.sensor_path).astype(bool)

            for i in range(self.num_rays):

                # Update UI labels with the new data
                self.beam_hit_labels[i].text = f"beamhit: {beam_hit[i]}"
                self.linear_depth_labels[i].text = f"linearDepth: {lin_depth[i]}"
                self.hit_pos_labels[i].text = (
                    f"hitPos x: {hit_pos[i][0]}, hitPos y: {hit_pos[i][1]}, hitPos z: {hit_pos[i][2]}"
                )

    async def create_scenario(self) -> None:
        """Creates the demonstration scenario with a cube target and LightBeam sensor.

        Sets up a new USD stage with physics scene, creates a cube with collision properties,
        configures a LightBeam sensor with 5 rays, positions the camera for optimal viewing,
        and creates an action graph for real-time ray visualization.
        """
        self._assets_root_path = get_assets_root_path()
        if self._assets_root_path is None:
            carb.log_error("Could not find Isaac Sim assets folder")
            return

        await omni.usd.get_context().new_stage_async()
        stage = omni.usd.get_context().get_stage()
        offset = Gf.Vec3f(2.00, 0.0, 0.0)
        size = 1

        # we need a physics scene for physx raycasts
        UsdPhysics.Scene.Define(stage, Sdf.Path("/World/physicsScene"))

        self.cube_path = "/World/Cube"
        self.sensor_path = "/LightBeam_Sensor"

        # Define a light so we can see the cube better
        if get_prim_at_path("/DistantLight"):
            delete_prim("/DistantLight")
        distantLight = UsdLux.DistantLight.Define(stage, Sdf.Path("/DistantLight"))
        distantLight.CreateIntensityAttr(500)
        distantLight.AddRotateXYZOp().Set((-36, -36, 0))

        self.cube_geom = UsdGeom.Cube.Define(stage, self.cube_path)
        self.cube_prim = stage.GetPrimAtPath(self.cube_path)

        self.cube_geom.CreateSizeAttr(size)
        self.cube_geom.AddTranslateOp().Set(offset)
        UsdGeom.XformCommonAPI(self.cube_prim).SetScale((1.00, 1.00, 1.00))

        # In order for our cube to interact with the light beam sensor, it needs to be able to collide with our physX line traces.
        # to do this, we give our cube the collision API, and set it's material and collision group.
        UsdPhysics.CollisionAPI.Apply(self.cube_prim)

        await omni.kit.app.get_app().next_update_async()

        self.meters_per_unit = UsdGeom.GetStageMetersPerUnit(omni.usd.get_context().get_stage())

        result, sensor = omni.kit.commands.execute(
            "IsaacSensorCreateLightBeamSensor",
            path=self.sensor_path,
            parent=None,
            min_range=0.2,
            max_range=10.0,
            translation=Gf.Vec3d(0, 0, 0),
            orientation=Gf.Quatd(1, 0, 0, 0),
            forward_axis=Gf.Vec3d(1, 0, 0),
            num_rays=5,
            curtain_length=0.5,
        )

        if not result:
            carb.log_error("Could not create Light Beam Sensor")
            return

        await omni.kit.app.get_app().next_update_async()

        # we want to make sure we can see the sensor we made, so we set the camera position and look target
        set_camera_view(eye=[-5.00, 5.00, 3.50], target=[0.0, 0.0, 0.0], camera_prim_path="/OmniverseKit_Persp")

        self._usd_context = omni.usd.get_context()
        self._stage_event_subscription = carb.eventdispatcher.get_eventdispatcher().observe_event(
            event_name=self._usd_context.stage_event_name(omni.usd.StageEventType.CLOSED),
            on_event=self._on_stage_closed,
            observer_name="isaacsim.sensors.physx.examples.lightbeam_sensor._on_stage_closed",
        )

        action_graph, new_nodes, _, _ = og.Controller.edit(
            {"graph_path": "/ActionGraph", "evaluator_name": "execution"},
            {
                og.Controller.Keys.CREATE_NODES: [
                    ("OnPlaybackTick", "omni.graph.action.OnPlaybackTick"),
                    ("IsaacReadLightBeam", "isaacsim.sensors.physx.IsaacReadLightBeam"),
                    ("DebugDrawRayCast", "isaacsim.util.debug_draw.DebugDrawRayCast"),
                ],
                og.Controller.Keys.SET_VALUES: [
                    ("IsaacReadLightBeam.inputs:lightbeamPrim", self.sensor_path),
                ],
                og.Controller.Keys.CONNECT: [
                    ("OnPlaybackTick.outputs:tick", "IsaacReadLightBeam.inputs:execIn"),
                    ("IsaacReadLightBeam.outputs:execOut", "DebugDrawRayCast.inputs:exec"),
                    ("IsaacReadLightBeam.outputs:beamOrigins", "DebugDrawRayCast.inputs:beamOrigins"),
                    ("IsaacReadLightBeam.outputs:beamEndPoints", "DebugDrawRayCast.inputs:beamEndPoints"),
                    ("IsaacReadLightBeam.outputs:numRays", "DebugDrawRayCast.inputs:numRays"),
                ],
            },
        )

        await og.Controller.evaluate(action_graph)


__all__ = []
