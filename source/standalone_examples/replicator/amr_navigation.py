# SPDX-FileCopyrightText: Copyright (c) 2023-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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

"""Generate synthetic data from an AMR navigating to random locations."""

from isaacsim import SimulationApp

simulation_app = SimulationApp(launch_config={"headless": False})

import argparse
import builtins
import os
import random
from itertools import cycle

import carb.eventdispatcher
import carb.settings
import omni.client
import omni.replicator.core as rep
import omni.timeline
import omni.usd
import omni.usd.commands
from isaacsim.core.utils.stage import create_new_stage
from isaacsim.storage.native import get_assets_root_path
from pxr import Gf, UsdGeom

DEFAULT_ENV_URLS = [
    "/Isaac/Environments/Grid/default_environment.usd",
    "/Isaac/Environments/Simple_Warehouse/warehouse.usd",
    "/Isaac/Environments/Grid/gridroom_black.usd",
    None,
]


def _parse_env_url_arg(env_url: str) -> str | None:
    """Parse CLI environment arguments, where None/null selects the generic environment."""
    return None if env_url.lower() in {"none", "null"} else env_url


parser = argparse.ArgumentParser()
parser.add_argument("--num_frames", type=int, default=9, help="The number of frames to capture")
parser.add_argument("--env_interval", type=int, default=3, help="Interval at which to change the environments")
parser.add_argument(
    "--env_urls",
    nargs="+",
    type=_parse_env_url_arg,
    default=None,
    help="Replace DEFAULT_ENV_URLS entirely. Use None for the generic environment.",
)
parser.add_argument("--use_temp_rp", action="store_true", help="Create and destroy render products for each SDG frame")
args, unknown = parser.parse_known_args()


class NavSDGDemo:
    """Demonstration of synthetic data generation using an AMR navigating towards a target."""

    CARTER_URL = "/Isaac/Samples/Replicator/OmniGraph/nova_carter_nav_only.usd"
    DOLLY_URL = "/Isaac/Props/Dolly/dolly.usd"
    PROPS_URL = "/Isaac/Props/YCB/Axis_Aligned_Physics"
    LEFT_CAMERA_REL_PATH = "sensors/front_hawk/left/camera_left"
    RIGHT_CAMERA_REL_PATH = "sensors/front_hawk/right/camera_right"
    ENVIRONMENT_SCOPE_PATH = "/Environment"

    def __init__(self) -> None:
        """Initialize the navigation SDG demo with default values."""
        self._carter_chassis = None
        self._carter_nav_target = None
        self._dolly = None
        self._dolly_light = None
        self._props = []
        self._cycled_env_urls = None
        self._env_interval = 1
        self._timeline = None
        self._timeline_sub = None
        self._stage_event_sub = None
        self._stage = None
        self._trigger_distance = 2.0
        self._num_frames = 0
        self._frame_counter = 0
        self._writer = None
        self._out_dir = None
        self._render_products = []
        self._use_temp_rp = False
        self._in_running_state = False

    def start(
        self,
        num_frames: int = 10,
        out_dir: str | None = None,
        env_urls: list[str | None] | None = None,
        env_interval: int = 3,
        use_temp_rp: bool = False,
        seed: int | None = None,
    ) -> None:
        """Start the SDG demo with the given configuration."""
        print(f"[SDG] Starting")
        if seed is not None:
            rep.set_global_seed(seed)
            random.seed(seed)
        selected_env_urls = env_urls if env_urls is not None else DEFAULT_ENV_URLS
        self._num_frames = num_frames
        self._out_dir = out_dir if out_dir is not None else os.path.join(os.getcwd(), "_out_nav_sdg_demo")
        self._cycled_env_urls = cycle(selected_env_urls)
        self._env_interval = env_interval
        self._use_temp_rp = use_temp_rp
        self._frame_counter = 0
        self._trigger_distance = 2.0
        self._load_env()
        self._randomize_dolly_pose()
        self._randomize_dolly_light()
        self._randomize_prop_poses()
        self._setup_sdg()
        self._timeline = omni.timeline.get_timeline_interface()
        self._timeline.play()
        self._timeline_sub = carb.eventdispatcher.get_eventdispatcher().observe_event(
            event_name=omni.timeline.GLOBAL_EVENT_CURRENT_TIME_TICKED,
            on_event=self._on_timeline_event,
            observer_name="amr_navigation.NavSDGDemo._on_timeline_event",
        )
        self._stage_event_sub = carb.eventdispatcher.get_eventdispatcher().observe_event(
            event_name=omni.usd.get_context().stage_event_name(omni.usd.StageEventType.CLOSING),
            on_event=self._on_stage_closing_event,
            observer_name="amr_navigation.NavSDGDemo._on_stage_closing_event",
        )
        self._in_running_state = True

    def clear(self) -> None:
        """Reset all state variables and unsubscribe from events."""
        self._cycled_env_urls = None
        self._carter_chassis = None
        self._carter_nav_target = None
        self._dolly = None
        self._dolly_light = None
        self._timeline = None
        self._frame_counter = 0
        self._stage_event_sub = None
        self._timeline_sub = None
        self._clear_sdg_render_products()
        self._stage = None
        self._in_running_state = False

    def is_running(self) -> bool:
        """Return whether the SDG demo is currently running."""
        return self._in_running_state

    def _is_running_in_script_editor(self) -> bool:
        """Return whether the script is running in the Isaac Sim script editor."""
        return builtins.ISAAC_LAUNCHED_FROM_TERMINAL is True

    def _on_stage_closing_event(self, e: carb.eventdispatcher.Event) -> None:
        """Handle stage closing event by clearing state."""
        self.clear()

    def _load_env(self) -> None:
        """Create a new stage and load environment, robot, dolly, light, and props."""
        create_new_stage()
        self._stage = omni.usd.get_context().get_stage()
        assets_root_path = get_assets_root_path()
        rep.functional.physics.create_physics_scene(
            "/PhysicsScene", enableCCD=True, broadphaseType="MBP", enableGPUDynamics=False
        )

        # Environment
        self._load_environment(next(self._cycled_env_urls))

        # Nova Carter
        rep.functional.create.scope(name="NavWorld")
        carter = rep.functional.create.reference(
            position=(0, 0, 0),
            rotation=(0, 0, 0),
            usd_path=assets_root_path + self.CARTER_URL,
            parent="/NavWorld",
            name="CarterNav",
        )

        # Iterate children until targetXform (for navigation target) and chassis_link (for current location) are found
        for child in carter.GetChildren():
            if child.GetName() == "targetXform":
                self._carter_nav_target = child
                break
        for child in carter.GetChildren():
            if child.GetName() == "chassis_link":
                self._carter_chassis = child
                break

        # Dolly
        self._dolly = rep.functional.create.reference(
            position=(0, 0, 0),
            rotation=(0, 0, 0),
            usd_path=assets_root_path + self.DOLLY_URL,
            parent="/NavWorld",
            name="Dolly",
        )

        # # Add colliders to the dolly and its geometry primitives
        for desc_prim in self._dolly.GetChildren():
            if desc_prim.IsA(UsdGeom.Gprim):
                rep.functional.physics.apply_rigid_body(desc_prim)

        # Light
        self._dolly_light = rep.functional.create.sphere_light(
            position=(0, 0, 0),
            intensity=250000,
            radius=0.3,
            color=(1.0, 1.0, 1.0),
            parent="/NavWorld",
            name="DollyLight",
        )

        # Props
        props_urls = []
        props_folder_path = assets_root_path + self.PROPS_URL
        result, entries = omni.client.list(props_folder_path)
        if result != omni.client.Result.OK:
            carb.log_error(f"Could not list assets in path: {props_folder_path}")
            return
        for entry in entries:
            _, ext = os.path.splitext(entry.relative_path)
            if ext == ".usd":
                props_urls.append(f"{props_folder_path}/{entry.relative_path}")

        cycled_props_url = cycle(props_urls)
        for i in range(15):
            prop_url = next(cycled_props_url)
            prop_name = os.path.splitext(os.path.basename(prop_url))[0]
            path = f"/NavWorld/Props/Prop_{prop_name}_{i}"
            prim = self._stage.DefinePrim(path, "Xform")
            prim.GetReferences().AddReference(prop_url)
            self._props.append(prim)

    def _randomize_dolly_pose(self) -> None:
        """Set random dolly position ensuring minimum distance from Carter."""
        min_dist_from_carter = 4
        carter_loc = self._carter_chassis.GetAttribute("xformOp:translate").Get()
        for _ in range(100):
            x, y = random.uniform(-6, 6), random.uniform(-6, 6)
            dist = (Gf.Vec2f(x, y) - Gf.Vec2f(carter_loc[0], carter_loc[1])).GetLength()
            if dist > min_dist_from_carter:
                self._dolly.GetAttribute("xformOp:translate").Set((x, y, 0))
                self._carter_nav_target.GetAttribute("xformOp:translate").Set((x, y, 0))
                break
        self._dolly.GetAttribute("xformOp:rotateXYZ").Set((0, 0, random.uniform(-180, 180)))

    def _randomize_dolly_light(self) -> None:
        """Position light above dolly with random color."""
        dolly_loc = self._dolly.GetAttribute("xformOp:translate").Get()
        self._dolly_light.GetAttribute("xformOp:translate").Set(dolly_loc + (0, 0, 3))
        self._dolly_light.GetAttribute("inputs:color").Set(
            (random.uniform(0, 1), random.uniform(0, 1), random.uniform(0, 1))
        )

    def _randomize_prop_poses(self) -> None:
        """Stack props above the dolly with random horizontal offsets."""
        spawn_loc = self._dolly.GetAttribute("xformOp:translate").Get()
        spawn_loc[2] = spawn_loc[2] + 0.5
        for prop in self._props:
            prop.GetAttribute("xformOp:translate").Set(spawn_loc + (random.uniform(-1, 1), random.uniform(-1, 1), 0))
            spawn_loc[2] = spawn_loc[2] + 0.2

    def _setup_sdg(self) -> None:
        """Configure SDG settings, camera parameters, writer, and render products."""
        rep.orchestrator.set_capture_on_play(False)

        # Set DLSS to Quality mode (2) for best SDG results , options: 0 (Performance), 1 (Balanced), 2 (Quality), 3 (Auto)
        carb.settings.get_settings().set("rtx/post/dlss/execMode", 2)

        # fStop=0 for well-lit sharp images; tickRate=0 forces autotrigger so the sensor
        # cameras stay in sync with rep.orchestrator.step_async under multi-tick rendering.
        for rel_path in (self.LEFT_CAMERA_REL_PATH, self.RIGHT_CAMERA_REL_PATH):
            camera_prim = self._stage.GetPrimAtPath(self._carter_chassis.GetPath().AppendPath(rel_path))
            camera_prim.GetAttribute("fStop").Set(0.0)
            if camera_prim.HasAttribute("omni:sensor:tickRate"):
                camera_prim.GetAttribute("omni:sensor:tickRate").Set(0.0)

        backend = rep.backends.get("DiskBackend")
        backend.initialize(output_dir=self._out_dir)
        print(f"[SDG] Writing data to: {self._out_dir}")
        self._writer = rep.writers.get("BasicWriter")
        self._writer.initialize(backend=backend, rgb=True)
        self._setup_sdg_render_products()

    def _setup_sdg_render_products(self) -> None:
        """Create and attach render products for left and right cameras."""
        print(f"[SDG] Creating SDG render products")
        left_camera_path = self._carter_chassis.GetPath().AppendPath(self.LEFT_CAMERA_REL_PATH)
        rp_left = rep.create.render_product(
            str(left_camera_path),
            (1024, 1024),
            name="left_sensor",
            force_new=True,
        )
        right_camera_path = self._carter_chassis.GetPath().AppendPath(self.RIGHT_CAMERA_REL_PATH)
        rp_right = rep.create.render_product(
            str(right_camera_path),
            (1024, 1024),
            name="right_sensor",
            force_new=True,
        )
        self._render_products = [rp_left, rp_right]
        # For better performance the render products can be disabled when not in use, and re-enabled only during SDG
        if self._use_temp_rp:
            self._disable_render_products()
        self._writer.attach(self._render_products)

    def _clear_sdg_render_products(self) -> None:
        """Detach writer and destroy all render products."""
        print(f"[SDG] Clearing SDG render products")
        if self._writer:
            self._writer.detach()
        for rp in self._render_products:
            rp.destroy()
        self._render_products.clear()
        if self._stage.GetPrimAtPath("/Replicator"):
            omni.kit.commands.execute("DeletePrimsCommand", paths=["/Replicator"])

    def _enable_render_products(self) -> None:
        """Enable texture updates on all render products."""
        print(f"[SDG] Enabling render products for SDG..")
        for rp in self._render_products:
            rp.hydra_texture.set_updates_enabled(True)

    def _disable_render_products(self) -> None:
        """Disable texture updates on all render products."""
        print(f"[SDG] Disabling render products (enabled only during SDG)..")
        for rp in self._render_products:
            rp.hydra_texture.set_updates_enabled(False)

    def _run_sdg(self) -> None:
        """Execute one SDG capture step synchronously."""
        if self._use_temp_rp:
            self._enable_render_products()
        rep.orchestrator.step(rt_subframes=16)
        if self._use_temp_rp:
            self._disable_render_products()

    async def _run_sdg_async(self) -> None:
        """Execute one SDG capture step asynchronously."""
        if self._use_temp_rp:
            self._enable_render_products()
        await rep.orchestrator.step_async(rt_subframes=16)
        if self._use_temp_rp:
            self._disable_render_products()

    def _load_next_env(self) -> None:
        """Replace current environment with the next one from the cycle."""
        self._load_environment(next(self._cycled_env_urls))

    def _load_environment(self, env_url: str | None) -> None:
        """Load the next environment under a shared scope."""
        if self._stage.GetPrimAtPath(self.ENVIRONMENT_SCOPE_PATH):
            omni.kit.commands.execute("DeletePrimsCommand", paths=[self.ENVIRONMENT_SCOPE_PATH])

        rep.functional.create.scope(name="Environment")
        if env_url:
            assets_root_path = get_assets_root_path()
            rep.functional.create.reference(
                usd_path=assets_root_path + env_url, parent=self.ENVIRONMENT_SCOPE_PATH, name="Scene"
            )
            return

        rep.functional.create.dome_light(intensity=500, parent=self.ENVIRONMENT_SCOPE_PATH, name="DomeLight")
        ground = rep.functional.create.plane(
            parent=self.ENVIRONMENT_SCOPE_PATH, name="GroundPlane", scale=(100, 100, 1)
        )
        rep.functional.physics.apply_collider(ground)

    def _on_sdg_done(self, task: object) -> None:
        """Callback invoked when async SDG step completes."""
        self._setup_next_frame()

    def _setup_next_frame(self) -> None:
        """Prepare scene for next frame or finish if all frames captured."""
        self._frame_counter += 1
        if self._frame_counter >= self._num_frames:
            print(f"[SDG] Finished")
            # Make sure the data has been written to disk before clearing the state
            if self._is_running_in_script_editor():
                import asyncio

                task = asyncio.ensure_future(rep.orchestrator.wait_until_complete_async())
                task.add_done_callback(lambda t: self.clear())
            else:
                rep.orchestrator.wait_until_complete()
                self.clear()
            return

        self._randomize_dolly_pose()
        self._randomize_dolly_light()
        self._randomize_prop_poses()
        if self._frame_counter % self._env_interval == 0:
            self._load_next_env()
        # Set a new random distance from which to take capture the next frame
        self._trigger_distance = random.uniform(1.75, 2.5)
        self._timeline.play()
        self._timeline_sub = carb.eventdispatcher.get_eventdispatcher().observe_event(
            event_name=omni.timeline.GLOBAL_EVENT_CURRENT_TIME_TICKED,
            on_event=self._on_timeline_event,
            observer_name="amr_navigation.NavSDGDemo._on_timeline_event",
        )

    def _on_timeline_event(self, e: carb.eventdispatcher.Event) -> None:
        """Check distance to dolly and trigger SDG capture when close enough."""
        carter_loc = self._carter_chassis.GetAttribute("xformOp:translate").Get()
        dolly_loc = self._dolly.GetAttribute("xformOp:translate").Get()
        dist = (Gf.Vec2f(dolly_loc[0], dolly_loc[1]) - Gf.Vec2f(carter_loc[0], carter_loc[1])).GetLength()
        if dist < self._trigger_distance:
            print(f"[SDG] Starting SDG for frame no. {self._frame_counter}")
            self._timeline.pause()
            if self._is_running_in_script_editor():
                import asyncio

                task = asyncio.ensure_future(self._run_sdg_async())
                task.add_done_callback(self._on_sdg_done)
            else:
                self._run_sdg()
                self._setup_next_frame()


out_dir = os.path.join(os.getcwd(), "_out_nav_sdg_demo", "")
selected_env_urls = args.env_urls if args.env_urls is not None else DEFAULT_ENV_URLS
nav_demo = NavSDGDemo()
nav_demo.start(
    num_frames=args.num_frames,
    out_dir=out_dir,
    env_urls=selected_env_urls,
    env_interval=args.env_interval,
    use_temp_rp=args.use_temp_rp,
    seed=22,
)

while simulation_app.is_running() and nav_demo.is_running():
    simulation_app.update()

simulation_app.close()
