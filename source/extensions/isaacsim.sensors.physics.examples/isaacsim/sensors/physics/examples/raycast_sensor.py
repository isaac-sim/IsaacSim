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

"""Physics raycast sensor example UI and scene setup."""

import asyncio
import math
import weakref
from typing import Any

import carb
import carb.eventdispatcher
import omni
import omni.graph.core as og
import omni.physics.core
import omni.ui as ui
import omni.usd
from isaacsim.examples.browser import get_instance as get_browser_instance
from isaacsim.gui.components.ui_utils import LABEL_WIDTH, get_style, setup_ui_headers
from isaacsim.sensors.experimental.physics import Raycast, RaycastSensor
from pxr import Gf, Sdf, UsdGeom, UsdLux, UsdPhysics

EXTENSION_NAME = "Physics Raycast Sensor Example"


def _generate_solid_state_rays(
    h_count: int = 10, v_count: int = 5, h_fov: float = 60.0, v_fov: float = 20.0
) -> tuple[list[list[float]], list[list[float]], None]:
    """Generate a rectangular grid of rays for a solid state physics raycast sensor.

    Args:
        h_count: Number of horizontal rays.
        v_count: Number of vertical rays.
        h_fov: Horizontal field of view in degrees.
        v_fov: Vertical field of view in degrees.

    Returns:
        Ray origins, ray directions, and no time offsets.
    """
    origins = []
    directions = []
    for vi in range(v_count):
        v_angle = math.radians(-v_fov / 2 + v_fov * vi / max(v_count - 1, 1))
        for hi in range(h_count):
            h_angle = math.radians(-h_fov / 2 + h_fov * hi / max(h_count - 1, 1))
            dx = math.cos(v_angle) * math.cos(h_angle)
            dy = math.cos(v_angle) * math.sin(h_angle)
            dz = math.sin(v_angle)
            origins.append([0.0, 0.0, 0.0])
            directions.append([dx, dy, dz])
    return origins, directions, None


def _generate_rotating_rays(
    v_count: int = 8, azimuth_steps: int = 36, v_fov: float = 30.0, rotation_rate: float = 1.0
) -> tuple[list[list[float]], list[list[float]], list[float]]:
    """Generate rays for a rotating physics raycast sensor with time offsets for sweep.

    Each azimuthal column is assigned a time offset within one sweep
    period.  The C++ plugin fires only the rays whose offsets fall inside
    the current physics step, producing a sweeping beam pattern.

    Args:
        v_count: Number of vertical rays per azimuth step.
        azimuth_steps: Number of azimuth columns in one sweep.
        v_fov: Vertical field of view in degrees.
        rotation_rate: Sweep rotation rate in hertz.

    Returns:
        Ray origins, ray directions, and per-ray time offsets.
    """
    period = 1.0 / rotation_rate
    origins = []
    directions = []
    time_offsets = []
    for ai in range(azimuth_steps):
        h_angle = math.radians(360.0 * ai / azimuth_steps)
        t_offset = period * ai / azimuth_steps
        for vi in range(v_count):
            v_angle = math.radians(-v_fov / 2 + v_fov * vi / max(v_count - 1, 1))
            dx = math.cos(v_angle) * math.cos(h_angle)
            dy = math.cos(v_angle) * math.sin(h_angle)
            dz = math.sin(v_angle)
            origins.append([0.0, 0.0, 0.0])
            directions.append([dx, dy, dz])
            time_offsets.append(t_offset)
    return origins, directions, time_offsets


def _generate_curtain_rays(
    beam_count: int = 16, curtain_height: float = 0.75
) -> tuple[list[list[float]], list[list[float]], None]:
    """Generate parallel rays spread vertically for a beam curtain physics raycast sensor.

    Args:
        beam_count: Number of vertical curtain beams.
        curtain_height: Total vertical spread of the curtain.

    Returns:
        Ray origins, ray directions, and no time offsets.
    """
    origins = []
    directions = []
    for i in range(beam_count):
        z = -curtain_height / 2 + curtain_height * i / max(beam_count - 1, 1)
        origins.append([0.0, 0.0, z])
        directions.append([1.0, 0.0, 0.0])
    return origins, directions, None


class Extension(omni.ext.IExt):
    """Extension that hosts the physics raycast sensor example UI."""

    def on_startup(self, ext_id: str) -> None:
        """Initialize the extension and register the example.

        Args:
            ext_id: Extension identifier provided by the extension manager.
        """
        ext_manager = omni.kit.app.get_app().get_extension_manager()
        self._ext_id = ext_id
        self._extension_path = ext_manager.get_extension_path(ext_id)
        self._window: ui.Window | None = None
        self._stage_event_subscription: Any | None = None

        get_browser_instance().register_example(
            name="Physics Raycast Sensor",
            execute_entrypoint=self.build_window,
            ui_hook=lambda a=weakref.proxy(self): a.build_ui(),
            category="Sensors",
        )

    def build_window(self) -> None:
        """Build the example window entrypoint."""

    def _on_stage_closed(self, event: Any) -> None:
        """Handle stage-closed events.

        Args:
            event: Stage event payload.
        """
        self.on_closed()

    def build_ui(self) -> None:
        """Build the UI controls for the physics raycast sensor example."""
        with ui.VStack(spacing=5, height=0):
            title = "Physics Raycast Sensor Example"
            doc_link = "https://docs.isaacsim.omniverse.nvidia.com/latest/sensors/isaacsim_sensors_physics_raycast.html"

            overview = "This example demonstrates three physics raycast sensor configurations: "
            overview += "solid state, rotating, and beam curtain. "
            overview += "Each sensor is visualized with debug ray lines in the viewport."
            overview += "\nPress PLAY to start the simulation."
            overview += "\n\nPress the 'Open in IDE' button to view the source code."

            setup_ui_headers(self._ext_id, __file__, title, doc_link, overview, info_collapsed=False)
            ui.Button("Load Scene", clicked_fn=lambda: self._load_scene())

    def _load_scene(self) -> None:
        """Load the physics raycast sensor example scene and initialize UI state."""
        if self._window:
            self.on_closed()

        self._readers: dict[str, RaycastSensor] = {}
        self._timeline = omni.timeline.get_timeline_interface()
        self.sub = omni.physics.core.get_physics_simulation_interface().subscribe_physics_on_step_events(
            pre_step=False, order=0, on_update=self._on_update
        )

        self._sensor_paths = {
            "solid_state": "/World/Sensors/Solid_State_Physics_Raycast_Sensor",
            "rotating": "/World/Sensors/Rotating_Physics_Raycast_Sensor",
            "curtain": "/World/Sensors/Beam_Curtain_Physics_Raycast_Sensor",
        }
        # Per-sensor max_range; misses report exactly maxRange so a hit is `d < max_range`.
        self._sensor_max_ranges = {
            "solid_state": 100.0,
            "rotating": 100.0,
            "curtain": 10.0,
        }

        self.sliders = {}
        style = {"background_color": 0xFF888888, "color": 0xFF333333, "secondary_color": 0xFFBBFFBB}

        sensor_labels = [
            ("solid_state", "Solid State", 0xFFBBFFBB),
            ("rotating", "Rotating", 0xFFBBBBFF),
            ("curtain", "Curtain", 0xFFFFBBBB),
        ]

        window = ui.Window(
            title=EXTENSION_NAME, width=320, height=0, visible=True, dockPreference=ui.DockPreference.LEFT_BOTTOM
        )
        self._window = window
        window.set_visibility_changed_fn(self._on_visibility_changed)

        with window.frame:
            with ui.VStack(style=get_style(), spacing=5):
                for key, label, color in sensor_labels:
                    style["secondary_color"] = color
                    with ui.HStack():
                        ui.Label(f"{label} hits", width=LABEL_WIDTH, tooltip="Number of rays that hit geometry")
                        slider = ui.FloatDrag(min=0.0, max=1000.0, step=1.0, style=style)
                        slider.enabled = False
                        self.sliders[f"{key}_hits"] = slider
                        ui.Spacer(width=20)
                    with ui.HStack():
                        ui.Label(f"{label} min depth", width=LABEL_WIDTH, tooltip="Minimum hit depth in stage units")
                        slider = ui.FloatDrag(min=0.0, max=200.0, step=0.01, style=style)
                        slider.enabled = False
                        self.sliders[f"{key}_min"] = slider
                        ui.Spacer(width=20)

        asyncio.ensure_future(self.create_scenario())

    def on_shutdown(self) -> None:
        """Clean up resources when the extension is unloaded."""
        self.on_closed()
        get_browser_instance().deregister_example(name="Physics Raycast Sensor", category="Sensors")

    def _on_visibility_changed(self, visible: bool) -> None:
        """Handle window visibility changes.

        Args:
            visible: Whether the window is visible.
        """
        if not visible:
            self.on_closed()

    def on_closed(self) -> None:
        """Tear down the example window and subscriptions."""
        if self._window:
            self.sub = None
            self._timeline = None
            self._stage_event_subscription = None
            self._readers.clear()
            self._window.destroy()
            self._window = None

    def _on_update(self, dt: float, context: Any) -> None:
        """Update UI values from the physics raycast sensor readings.

        Args:
            dt: Simulation time step.
            context: Physics update context.
        """
        if not self._timeline.is_playing() or not self.sliders:
            return

        for key, path in self._sensor_paths.items():
            if path not in self._readers:
                self._readers[path] = RaycastSensor(path)
            reading = self._readers[path].get_sensor_reading()
            if reading.is_valid and reading.ray_count > 0:
                depths = reading.depths
                max_range = self._sensor_max_ranges[key]
                hit_count = sum(1 for d in depths if d < max_range)
                min_depth = min(depths) if len(depths) > 0 else 0.0
                self.sliders[f"{key}_hits"].model.set_value(float(hit_count))
                self.sliders[f"{key}_min"].model.set_value(float(min_depth))
            else:
                self.sliders[f"{key}_hits"].model.set_value(0)
                self.sliders[f"{key}_min"].model.set_value(0)

    async def create_scenario(self) -> None:
        """Create the physics raycast sensor example scene with obstacles and sensors."""
        await omni.usd.get_context().new_stage_async()
        stage = omni.usd.get_context().get_stage()

        UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
        UsdGeom.SetStageMetersPerUnit(stage, 1.0)

        UsdPhysics.Scene.Define(stage, Sdf.Path("/World/PhysicsScene"))

        light = UsdLux.DistantLight.Define(stage, Sdf.Path("/World/DistantLight"))
        light.CreateIntensityAttr(500)
        light.AddRotateXYZOp().Set((-45, -45, 0))

        # Ground plane
        ground = UsdGeom.Cube.Define(stage, "/World/GroundPlane")
        ground.GetSizeAttr().Set(1.0)
        ground.AddTranslateOp().Set(Gf.Vec3d(0, 0, -0.05))
        ground.AddScaleOp().Set(Gf.Vec3f(50, 50, 0.1))
        UsdPhysics.CollisionAPI.Apply(ground.GetPrim())

        # Obstacle boxes
        obstacles = [
            ("/World/Obstacles/Wall", Gf.Vec3d(5, 0, 1.5), Gf.Vec3f(0.2, 8, 3)),
            ("/World/Obstacles/Box1", Gf.Vec3d(3, -3, 0.5), Gf.Vec3f(1, 1, 1)),
            ("/World/Obstacles/Box2", Gf.Vec3d(4, 4, 0.75), Gf.Vec3f(1.5, 1.5, 1.5)),
            ("/World/Obstacles/Column", Gf.Vec3d(-3, 2, 1.5), Gf.Vec3f(0.5, 0.5, 3)),
        ]

        for path, translate, scale in obstacles:
            cube = UsdGeom.Cube.Define(stage, path)
            cube.GetSizeAttr().Set(1.0)
            cube.AddTranslateOp().Set(translate)
            cube.AddScaleOp().Set(scale)
            UsdPhysics.CollisionAPI.Apply(cube.GetPrim())

        # Sensor parent xform
        UsdGeom.Xform.Define(stage, "/World/Sensors")

        await omni.kit.app.get_app().next_update_async()

        # Solid state physics raycast sensor
        ss_origins, ss_dirs, _ = _generate_solid_state_rays()
        Raycast.create(
            "/World/Sensors/Solid_State_Physics_Raycast_Sensor",
            min_range=0.4,
            max_range=100.0,
            ray_origins=ss_origins,
            ray_directions=ss_dirs,
            output_frame="WORLD",
            translations=[[0.0, 0.0, 1.5]],
        )

        # Rotating physics raycast sensor — rays fire in a sweeping pattern at 1 Hz
        rot_origins, rot_dirs, rot_offsets = _generate_rotating_rays(rotation_rate=1.0)
        Raycast.create(
            "/World/Sensors/Rotating_Physics_Raycast_Sensor",
            min_range=0.4,
            max_range=100.0,
            ray_origins=rot_origins,
            ray_directions=rot_dirs,
            ray_time_offsets=rot_offsets,
            output_frame="WORLD",
            translations=[[0.0, 3.0, 1.5]],
        )

        # Beam curtain physics raycast sensor
        cur_origins, cur_dirs, _ = _generate_curtain_rays()
        Raycast.create(
            "/World/Sensors/Beam_Curtain_Physics_Raycast_Sensor",
            min_range=0.2,
            max_range=10.0,
            ray_origins=cur_origins,
            ray_directions=cur_dirs,
            output_frame="WORLD",
            translations=[[0.0, -3.0, 1.0]],
        )

        await omni.kit.app.get_app().next_update_async()

        from isaacsim.core.utils.viewports import set_camera_view

        set_camera_view(eye=[-8.0, -8.0, 6.0], target=[0.0, 0.0, 1.0], camera_prim_path="/OmniverseKit_Persp")

        # Build OmniGraph for debug draw visualization
        sensor_configs = [
            ("SolidState", self._sensor_paths["solid_state"], (0.2, 1.0, 0.2, 1.0)),
            ("Rotating", self._sensor_paths["rotating"], (0.3, 0.5, 1.0, 1.0)),
            ("Curtain", self._sensor_paths["curtain"], (1.0, 0.3, 0.3, 1.0)),
        ]

        create_nodes = [("OnPlaybackTick", "omni.graph.action.OnPlaybackTick")]
        set_values = []
        connections = []

        for name, path, color in sensor_configs:
            read_node = f"ReadRaycast{name}"
            draw_node = f"DebugDraw{name}"
            create_nodes.append((read_node, "isaacsim.sensors.physics.IsaacReadRaycastSensor"))
            create_nodes.append((draw_node, "isaacsim.util.debug_draw.DebugDrawRayCast"))
            set_values.append((f"{read_node}.inputs:raycastSensorPrim", path))
            set_values.append((f"{draw_node}.inputs:color", color))
            set_values.append((f"{draw_node}.inputs:doTransform", False))
            connections.append(("OnPlaybackTick.outputs:tick", f"{read_node}.inputs:execIn"))
            connections.append((f"{read_node}.outputs:execOut", f"{draw_node}.inputs:exec"))
            connections.append((f"{read_node}.outputs:beamOrigins", f"{draw_node}.inputs:beamOrigins"))
            connections.append((f"{read_node}.outputs:beamEndPoints", f"{draw_node}.inputs:beamEndPoints"))
            connections.append((f"{read_node}.outputs:numRays", f"{draw_node}.inputs:numRays"))

        action_graph, _, _, _ = og.Controller.edit(
            {"graph_path": "/World/ActionGraph", "evaluator_name": "execution"},
            {
                og.Controller.Keys.CREATE_NODES: create_nodes,
                og.Controller.Keys.SET_VALUES: set_values,
                og.Controller.Keys.CONNECT: connections,
            },
        )

        await og.Controller.evaluate(action_graph)

        self._usd_context = omni.usd.get_context()
        self._stage_event_subscription = carb.eventdispatcher.get_eventdispatcher().observe_event(
            event_name=self._usd_context.stage_event_name(omni.usd.StageEventType.CLOSED),
            on_event=self._on_stage_closed,
            observer_name="isaacsim.sensors.physics.examples.raycast_sensor._on_stage_closed",
        )


__all__ = []
