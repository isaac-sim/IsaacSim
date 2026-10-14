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
import asyncio
import builtins
import os
import random
from itertools import cycle

import carb
import carb.eventdispatcher
import carb.settings
import isaacsim.core.experimental.utils.app as app_utils
import numpy as np
import omni.client
import omni.replicator.core as rep
import omni.timeline
import omni.usd
from isaacsim.core.experimental.prims import XformPrim
from isaacsim.core.experimental.utils.prim import get_prim_at_path
from isaacsim.core.experimental.utils.stage import add_reference_to_stage, create_new_stage, define_prim, delete_prim
from isaacsim.storage.native import get_assets_root_path
from pxr import UsdGeom

DEFAULT_ENV_URLS = [
    "/Isaac/Environments/Grid/default_environment.usd",
    "/Isaac/Environments/Simple_Warehouse/warehouse.usd",
    "/Isaac/Environments/Grid/gridroom_black.usd",
    None,
]


def _parse_env_url_arg(env_url: str) -> str | None:
    """Parse CLI environment arguments, where None/null selects the generic environment.

    Args:
        env_url: Command-line environment value to normalize.

    Returns:
        The supplied URL, or ``None`` when the value requests the generated environment.
    """
    return None if env_url.lower() in {"none", "null"} else env_url


parser = argparse.ArgumentParser()
parser.add_argument("--num-captures", type=int, default=4, help="The number of frames to capture")
parser.add_argument("--env-interval", type=int, default=1, help="Interval at which to change the environments")
parser.add_argument(
    "--env-urls",
    nargs="+",
    type=_parse_env_url_arg,
    default=None,
    help="Replace DEFAULT_ENV_URLS entirely. Use None for the generic environment.",
)
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
        self._carter = None
        self._carter_chassis = None
        self._carter_nav_target = None
        self._dolly = None
        self._dolly_light = None
        self._props = []
        self._cycled_env_urls = None
        self._env_interval = 1
        self._timeline_sub = None
        self._stage_event_sub = None
        self._stage = None
        self._trigger_distance = 2.0
        self._num_frames = 0
        self._frame_counter = 0
        self._writer = None
        self._out_dir = None
        self._render_products = []
        self._in_running_state = False

    def start(
        self,
        num_frames: int = 4,
        out_dir: str | None = None,
        env_urls: list[str | None] | None = None,
        env_interval: int = 1,
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
        self._frame_counter = 0
        self._trigger_distance = 2.0
        self._load_env()
        self._randomize_dolly_pose()
        self._randomize_dolly_light()
        self._randomize_prop_poses()
        self._setup_sdg()
        app_utils.play()
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
        self._carter = None
        self._carter_chassis = None
        self._carter_nav_target = None
        self._dolly = None
        self._dolly_light = None
        self._frame_counter = 0
        if self._timeline_sub:
            self._timeline_sub.reset()
            self._timeline_sub = None
        if self._stage_event_sub:
            self._stage_event_sub.reset()
            self._stage_event_sub = None
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
            position=(0, 0, 0.02),
            rotation=(0, 0, 0),
            usd_path=assets_root_path + self.CARTER_URL,
            parent="/NavWorld",
            name="CarterNav",
        )
        carter_path = str(carter.GetPrimPath())
        self._carter = carter
        self._carter_nav_target = get_prim_at_path(f"{carter_path}/targetXform")
        self._carter_chassis = get_prim_at_path(f"{carter_path}/chassis_link")

        # Dolly
        self._dolly = rep.functional.create.reference(
            position=(0, 0, 0),
            rotation=(0, 0, 0),
            usd_path=assets_root_path + self.DOLLY_URL,
            parent="/NavWorld",
            name="Dolly",
        )

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
            define_prim(path, "Xform")
            add_reference_to_stage(usd_path=prop_url, path=path)
            self._props.append(path)

    def _randomize_dolly_pose(self) -> None:
        """Set random dolly position ensuring minimum distance from Carter."""
        min_dist_from_carter = 4
        carter_loc = XformPrim(str(self._carter_chassis.GetPath())).get_local_poses()[0].numpy()[0]
        for _ in range(100):
            x, y = random.uniform(-6, 6), random.uniform(-6, 6)
            dist = float(np.linalg.norm(np.array([x, y]) - carter_loc[:2]))
            if dist > min_dist_from_carter:
                rep.functional.modify.pose(self._dolly, position_value=(x, y, 0))
                rep.functional.modify.pose(self._carter_nav_target, position_value=(x, y, 0))
                break
        rep.functional.modify.pose(self._dolly, rotation_value=(0, 0, random.uniform(-180, 180)))

    def _randomize_dolly_light(self) -> None:
        """Position light above dolly with random color."""
        dolly_loc = XformPrim(str(self._dolly.GetPath())).get_local_poses()[0].numpy()[0]
        rep.functional.modify.pose(
            self._dolly_light, position_value=(float(dolly_loc[0]), float(dolly_loc[1]), float(dolly_loc[2]) + 3)
        )
        rep.functional.modify.attribute(
            self._dolly_light, "inputs:color", (random.uniform(0, 1), random.uniform(0, 1), random.uniform(0, 1))
        )

    def _randomize_prop_poses(self) -> None:
        """Stack props above the dolly with random horizontal offsets."""
        spawn_loc = XformPrim(str(self._dolly.GetPath())).get_local_poses()[0].numpy()[0].copy()
        spawn_loc[2] = spawn_loc[2] + 0.5
        for prop_path in self._props:
            rep.functional.modify.pose(
                get_prim_at_path(prop_path),
                position_value=(
                    float(spawn_loc[0] + random.uniform(-1, 1)),
                    float(spawn_loc[1] + random.uniform(-1, 1)),
                    float(spawn_loc[2]),
                ),
            )
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
            (640, 640),
            name="left_sensor",
            force_new=True,
        )
        right_camera_path = self._carter_chassis.GetPath().AppendPath(self.RIGHT_CAMERA_REL_PATH)
        rp_right = rep.create.render_product(
            str(right_camera_path),
            (640, 640),
            name="right_sensor",
            force_new=True,
        )
        self._render_products = [rp_left, rp_right]
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
        if self._stage and self._stage.GetPrimAtPath("/Replicator"):
            delete_prim("/Replicator")

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
        self._enable_render_products()
        try:
            rep.orchestrator.step(rt_subframes=16)
            rep.orchestrator.wait_until_complete()
        finally:
            self._disable_render_products()

    async def _run_sdg_async(self) -> None:
        """Execute one SDG capture step asynchronously."""
        self._enable_render_products()
        try:
            await rep.orchestrator.step_async(rt_subframes=16)
            await rep.orchestrator.wait_until_complete_async()
        finally:
            self._disable_render_products()

    def _load_next_env(self) -> None:
        """Replace current environment with the next one from the cycle."""
        # Reset Carter Z to the spawn floor clearance so wheels clear the new surface
        carter = XformPrim(str(self._carter.GetPath()))
        translation = carter.get_local_poses()[0].numpy()[0]
        carter.set_local_poses(translations=[[translation[0], translation[1], 0.02]])
        self._load_environment(next(self._cycled_env_urls))

    def _load_environment(self, env_url: str | None) -> None:
        """Load the next environment under a shared scope."""
        if self._stage.GetPrimAtPath(self.ENVIRONMENT_SCOPE_PATH):
            delete_prim(self.ENVIRONMENT_SCOPE_PATH)

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

    def _on_sdg_done(self, task: asyncio.Task) -> None:
        """Callback invoked when async SDG step completes."""
        self._setup_next_frame()

    async def _clear_after_writer_drain_async(self) -> None:
        """Clear the demo after the final asynchronous writer drain."""
        try:
            await rep.orchestrator.wait_until_complete_async()
        finally:
            self.clear()

    def _setup_next_frame(self) -> None:
        """Prepare scene for next frame or finish if all frames captured."""
        self._frame_counter += 1
        if self._frame_counter >= self._num_frames:
            print(f"[SDG] Finished")
            if self._is_running_in_script_editor():
                asyncio.ensure_future(self._clear_after_writer_drain_async())
            else:
                try:
                    rep.orchestrator.wait_until_complete()
                finally:
                    self.clear()
            return

        self._randomize_dolly_pose()
        self._randomize_dolly_light()
        self._randomize_prop_poses()
        if self._frame_counter % self._env_interval == 0:
            self._load_next_env()
        # Set a new random distance from which to capture the next frame
        self._trigger_distance = random.uniform(1.75, 2.5)
        app_utils.play()
        self._timeline_sub = carb.eventdispatcher.get_eventdispatcher().observe_event(
            event_name=omni.timeline.GLOBAL_EVENT_CURRENT_TIME_TICKED,
            on_event=self._on_timeline_event,
            observer_name="amr_navigation.NavSDGDemo._on_timeline_event",
        )

    def _on_timeline_event(self, e: carb.eventdispatcher.Event) -> None:
        """Check distance to dolly and trigger SDG capture when close enough."""
        carter_loc = XformPrim(str(self._carter_chassis.GetPath())).get_local_poses()[0].numpy()[0]
        dolly_loc = XformPrim(str(self._dolly.GetPath())).get_local_poses()[0].numpy()[0]
        dist = float(np.linalg.norm(dolly_loc[:2] - carter_loc[:2]))
        if dist < self._trigger_distance:
            print(f"[SDG] Starting SDG for frame no. {self._frame_counter}")
            app_utils.pause()
            if self._timeline_sub:
                self._timeline_sub.reset()
                self._timeline_sub = None
            if self._is_running_in_script_editor():
                task = asyncio.ensure_future(self._run_sdg_async())
                task.add_done_callback(self._on_sdg_done)
            else:
                self._run_sdg()
                self._setup_next_frame()


out_dir = os.path.join(os.getcwd(), "_out_nav_sdg_demo", "")
selected_env_urls = args.env_urls if args.env_urls is not None else DEFAULT_ENV_URLS
nav_demo = NavSDGDemo()
nav_demo.start(
    num_frames=args.num_captures,
    out_dir=out_dir,
    env_urls=selected_env_urls,
    env_interval=args.env_interval,
    seed=22,
)

while simulation_app.is_running() and nav_demo.is_running():
    simulation_app.update()

# <start-amr-navigation-test>
import argparse
import sys

from isaacsim.core.utils.extensions import enable_extension

enable_extension("isaacsim.test.utils")
from isaacsim.test.utils.file_validation import validate_folder_contents

num_frames = args.num_captures
parser = argparse.ArgumentParser()
parser.add_argument(
    "--test",
    action="store_true",
    help="Validate captured output files against expected counts and exit.",
)
args, _ = parser.parse_known_args()

if args.test:
    import omni.kit.app
    from isaacsim.test.utils.image_comparison import compare_images_in_directories

    # BasicWriter rgb on left+right cameras → 1 png per camera per frame.
    expected_pngs = num_frames * 2
    ok = validate_folder_contents(
        path=out_dir,
        recursive=True,
        expected_counts={"png": expected_pngs},
        fail_on_empty_files=True,
    )
    if not ok:
        print(f"[SDG][Test][FAIL] Output validation failed for {out_dir}")
        sys.exit(1)

    # The second capture follows a physics-driven navigation step, so allow for small
    # platform-dependent differences in the robot pose and rendered image.
    rgb_mean_diff_tolerance = 10
    enable_extension("isaacsim.replicator.examples")
    replicator_examples_ext_path = (
        omni.kit.app.get_app().get_extension_manager().get_extension_path_by_module("isaacsim.replicator.examples")
    )
    golden_dir = os.path.join(
        replicator_examples_ext_path,
        "isaacsim",
        "replicator",
        "examples",
        "tests",
        "data",
        "golden",
        "_out_nav_sdg_demo",
    )
    rgb_results = []
    for golden_root, _, golden_files in os.walk(golden_dir):
        if any(file_name.endswith(".png") for file_name in golden_files):
            relative_root = os.path.relpath(golden_root, golden_dir)
            test_root = out_dir if relative_root == "." else os.path.join(out_dir, relative_root)
            rgb_results.append(
                compare_images_in_directories(
                    golden_dir=golden_root,
                    test_dir=test_root,
                    path_pattern=r"^rgb_.*\.png$",
                    allclose_rtol=None,
                    allclose_atol=None,
                    mean_tolerance=rgb_mean_diff_tolerance,
                    print_all_stats=False,
                )
            )
    if not rgb_results or not all(result["all_passed"] for result in rgb_results):
        print(
            f"[SDG][Test][FAIL] RGB image comparison failed (tol={rgb_mean_diff_tolerance}). "
            f"Golden dir: {golden_dir}, output dir: {out_dir}"
        )
        sys.exit(1)
    print(f"[SDG][Test][PASS] Output validation succeeded for {out_dir} ({expected_pngs} pngs)")
# <end-amr-navigation-test>

simulation_app.close()
