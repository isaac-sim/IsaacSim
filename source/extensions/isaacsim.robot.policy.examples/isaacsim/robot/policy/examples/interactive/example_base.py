# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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

"""Shared scaffolding for the interactive policy example UIs.

Covers examples-browser registration, the physics-callback lifecycle, and the keyboard-driven
locomotion sample base.
"""

from __future__ import annotations

import carb
import isaacsim.core.experimental.utils.stage as stage_utils
import numpy as np
import omni.appwindow
import omni.ext
import omni.usd
from isaacsim.core.simulation_manager import IsaacEvents, SimulationManager
from isaacsim.examples.base import BaseSample, BaseSampleUITemplate
from isaacsim.examples.browser import get_instance as get_browser_instance
from isaacsim.robot.policy.examples import RobotPolicyRunner
from isaacsim.robot.policy.examples.interactive.utils import (
    restore_physics_simulation_state,
    snapshot_physics_simulation_state,
)
from isaacsim.storage.native import get_assets_root_path
from pxr import UsdPhysics, UsdShade


def make_velocity_keyboard_mapping(scale: float) -> dict[str, list[float]]:
    """Build the arrow/numpad keyboard mapping from key name to ``[vx, vy, yaw_rate]`` increments.

    Args:
        scale: Command magnitude applied on every axis.

    Returns:
        The resulting dict.
    """
    return {
        "NUMPAD_8": [scale, 0.0, 0.0],
        "UP": [scale, 0.0, 0.0],
        "NUMPAD_2": [-scale, 0.0, 0.0],
        "DOWN": [-scale, 0.0, 0.0],
        "NUMPAD_6": [0.0, -scale, 0.0],
        "RIGHT": [0.0, -scale, 0.0],
        "NUMPAD_4": [0.0, scale, 0.0],
        "LEFT": [0.0, scale, 0.0],
        "NUMPAD_7": [0.0, 0.0, scale],
        "N": [0.0, 0.0, scale],
        "NUMPAD_9": [0.0, 0.0, -scale],
        "M": [0.0, 0.0, -scale],
    }


# ---------------------------------------------------------------------------------------------
# Examples-browser registration
# ---------------------------------------------------------------------------------------------


class PolicyExampleExtension(omni.ext.IExt):
    """Register a policy example UI in the examples browser.

    Attributes:
        example_name: Browser entry name; also the deregistration key on shutdown.
        category: Browser category; every policy example lives under ``Policy``.
        title: Window title of the sample UI.
        doc_link: Documentation URL behind the UI's docs button; defaults to the policy
            deployment tutorial.
        overview: Description text shown in the sample UI.
        sample_class: The :class:`BaseSample` subclass instantiated for the UI.
        file_path: The subclass's own module file, backing the 'Open in IDE' button.
    """

    example_name: str = ""
    category: str = "Policy"
    title: str = ""
    doc_link: str = (
        "https://docs.isaacsim.omniverse.nvidia.com/latest/isaac_lab_tutorials/tutorial_policy_deployment.html"
    )
    overview: str = ""
    sample_class: type[BaseSample] = BaseSample
    file_path: str = ""

    def on_startup(self, ext_id: str) -> None:
        """Register the example without constructing its sample or accessing assets.

        Args:
            ext_id: The extension identifier.
        """
        self._ext_id = ext_id
        self._ui_handle = None
        get_browser_instance().register_example(
            name=self.example_name,
            ui_hook=self._build_ui,
            category=self.category,
        )

    def _build_ui(self) -> None:
        """Construct the sample on first selection, then reuse it when rebuilding the UI."""
        if self._ext_id is None:
            return
        if self._ui_handle is None:
            self._ui_handle = BaseSampleUITemplate(
                ext_id=self._ext_id,
                file_path=self.file_path,
                title=self.title,
                doc_link=self.doc_link,
                overview=self.overview,
                sample=self.sample_class(),
            )
        self._ui_handle.build_ui()

    def on_shutdown(self) -> None:
        """Deregister the example from the examples browser."""
        self._ext_id = None
        get_browser_instance().deregister_example(name=self.example_name, category=self.category)
        if self._ui_handle is not None:
            self._ui_handle.on_shutdown()
            self._ui_handle = None


# ---------------------------------------------------------------------------------------------
# Physics-callback sample base
# ---------------------------------------------------------------------------------------------


class PolicySampleBase(BaseSample):
    """Physics-callback scaffolding shared by the interactive policy samples.

    Owns the configured physics-step and TIMELINE_STOP callback registration and the
    ``_physics_ready`` first-step latch; reset and stop events clear the latch so subclasses
    reinitialize their runner on the next physics step.
    """

    physics_callback_event = IsaacEvents.POST_PHYSICS_STEP

    def __init__(self) -> None:
        super().__init__()
        self._world_settings["stage_units_in_meters"] = 1.0
        self._physics_ready = False
        self._policy_failed = False
        self._physics_callback_id = None
        self._timeline_stop_callback_id = None

    def _register_physics_callback(self) -> None:
        """Register the configured physics-step and timeline-stop callbacks (idempotent)."""
        if self._physics_callback_id is None:
            self._physics_callback_id = SimulationManager.register_callback(
                self.on_physics_step, self.physics_callback_event
            )
        if self._timeline_stop_callback_id is None:
            self._timeline_stop_callback_id = SimulationManager.register_callback(
                self._on_timeline_stop, IsaacEvents.TIMELINE_STOP
            )

    def _deregister_physics_callback(self) -> None:
        """Deregister the physics-step and timeline-stop callbacks."""
        if self._physics_callback_id is not None:
            SimulationManager.deregister_callback(self._physics_callback_id)
            self._physics_callback_id = None
        if self._timeline_stop_callback_id is not None:
            SimulationManager.deregister_callback(self._timeline_stop_callback_id)
            self._timeline_stop_callback_id = None

    def _on_timeline_stop(self, event: object) -> None:
        """Require a clean policy restart after the timeline stops.

        Args:
            event: Timeline stop event data.
        """
        del event
        self._physics_ready = False
        self._policy_failed = False

    async def setup_pre_reset(self) -> None:
        """Reset the physics-ready flag before a world reset."""
        self._physics_ready = False
        self._policy_failed = False

    async def setup_post_reset(self) -> None:
        """Reset the physics-ready flag after a reset so the runner reinitializes on next play."""
        self._physics_ready = False
        self._policy_failed = False

    async def setup_post_clear(self) -> None:
        """Release physics resources after the scene is cleared."""
        self.physics_cleanup()


# ---------------------------------------------------------------------------------------------
# Keyboard-driven locomotion sample base
# ---------------------------------------------------------------------------------------------


class LocomotionPolicySample(PolicySampleBase):
    """Keyboard-driven locomotion policy sample deployed through the generic ``RobotPolicyRunner``.

    This base owns GPU device selection, scene setup, keyboard plumbing, the physics-step
    policy loop, and cleanup with physics-state restoration. Subclasses configure it through
    class attributes:

    Attributes:
        _spec_getter: Bundled policy spec getter (wrap in ``staticmethod``); called at deploy
            time so asset-root resolution happens on load, not import.
        _prim_path: Stage path the robot is spawned at.
        _spawn_position: Spawn translation; height matches each robot's stand pose.
        _physics_dt: Physics step in seconds; matches the policy's training rate.
        _rendering_dt: Rendering step in seconds (a whole number of physics steps).
        _input_keyboard_mapping: Key name to ``[vx, vy, yaw_rate]`` command increments applied
            while the key is held.
    """

    _spec_getter = None
    _prim_path: str = "/World/Robot"
    _spawn_position: tuple[float, float, float] = (0.0, 0.0, 0.0)
    _physics_dt: float = 1.0 / 200.0
    _rendering_dt: float = 1.0 / 50.0
    _input_keyboard_mapping: dict[str, list[float]] = {}

    def __init__(self) -> None:
        super().__init__()
        # Configure simulation settings for GPU dynamics matching Isaac Lab training.
        self._world_settings["physics_dt"] = self._physics_dt
        self._world_settings["rendering_dt"] = self._rendering_dt
        self._world_settings["device"] = "cuda"
        self._world_settings["backend"] = "torch"

        self._base_command = np.zeros(3, dtype=np.float32)
        self._runner: RobotPolicyRunner | None = None
        self._sub_keyboard = None
        self._input = None
        self._keyboard = None
        self._prev_physics_sim_device: str | None = None
        self._prev_fabric_enabled: bool | None = None

    def _apply_ground_material(self, static_friction: float, dynamic_friction: float, restitution: float) -> None:
        """Apply a physics material to the ground plane.

        Args:
            static_friction: Static friction coefficient.
            dynamic_friction: Dynamic friction coefficient.
            restitution: Restitution coefficient.
        """
        stage = omni.usd.get_context().get_stage()
        material = UsdShade.Material.Define(stage, "/World/ground/Looks/PhysicsMaterial")
        physics_material = UsdPhysics.MaterialAPI.Apply(material.GetPrim())
        physics_material.CreateStaticFrictionAttr().Set(static_friction)
        physics_material.CreateDynamicFrictionAttr().Set(dynamic_friction)
        physics_material.CreateRestitutionAttr().Set(restitution)

        ground_geom = stage.GetPrimAtPath("/World/ground/GroundPlane/CollisionPlane")
        if ground_geom.IsValid():
            UsdShade.MaterialBindingAPI.Apply(ground_geom).Bind(material)

    def setup_scene(self) -> None:
        """Set up the scene with the robot and the ground environment."""
        # Snapshot prior physics device/fabric state so cleanup can restore it.
        self._prev_physics_sim_device, self._prev_fabric_enabled = snapshot_physics_simulation_state()

        # Set device and backend BEFORE creating the robot so it uses the GPU.
        SimulationManager.set_backend(self._world_settings["backend"])
        SimulationManager.set_physics_sim_device(self._world_settings["device"])
        SimulationManager.get_available_physics_engines(verbose=True)

        assets_root_path = get_assets_root_path()
        if assets_root_path is None:
            raise RuntimeError("Could not find the Isaac Sim assets folder.")

        # Ground plane with a physics material matching the training configuration.
        stage_utils.add_reference_to_stage(
            usd_path=assets_root_path + "/Isaac/Environments/Grid/default_environment.usd",
            path="/World/ground",
        )
        self._apply_ground_material(static_friction=1.0, dynamic_friction=1.0, restitution=0.0)

        # Author the robot through the runner; policy initialization happens on the first physics step.
        self._runner = RobotPolicyRunner(
            self._spec_getter(),
            prim_path=self._prim_path,
            position=list(self._spawn_position),
        )
        self._runner.spawn()

    async def setup_post_load(self) -> None:
        """Set up keyboard input and the physics callback after initial load."""
        appwindow = omni.appwindow.get_default_app_window()
        self._input = carb.input.acquire_input_interface()
        self._keyboard = appwindow.get_keyboard()
        self._sub_keyboard = self._input.subscribe_to_keyboard_events(self._keyboard, self._sub_keyboard_event)

        self._base_command.fill(0.0)
        self._physics_ready = False
        self._policy_failed = False
        self._register_physics_callback()

    def on_physics_step(self, dt: float, context: object) -> None:
        """Initialize the runner on the first step, then step the policy with the current command.

        Args:
            dt: Time delta for the physics step.
            context: Physics step context information.
        """
        runner = self._runner
        if runner is None or runner.articulation is None or self._policy_failed:
            return

        # If the physics tensors were invalidated, reinitialize on this step.
        if not runner.articulation.is_physics_tensor_entity_valid():
            self._physics_ready = False

        # A failed restart leaves _physics_ready False and a failed control tick leaves the
        # runner's tick unadvanced, so without the latch the same call retries every step.
        try:
            if self._physics_ready:
                runner.step(dt, self._base_command.tolist())
            else:
                runner.restart_from_default_state(self._base_command.tolist())
                self._physics_ready = True
        except Exception as error:  # noqa: BLE001 - a physics callback must not raise
            self._policy_failed = True
            carb.log_error(f"{type(self).__name__}: policy deployment failed, stopping: {error}")

    def _sub_keyboard_event(self, event: object, *args: object, **kwargs: object) -> bool:
        """Increment or decrement the base command on key press or release.

        Args:
            event: Keyboard event data.
            *args: Additional positional arguments.
            **kwargs: Additional keyword arguments.

        Returns:
            True to indicate the event was handled.
        """
        increment = self._input_keyboard_mapping.get(event.input.name)
        if increment is None:
            return True
        delta = np.asarray(increment, dtype=np.float32)
        if event.type == carb.input.KeyboardEventType.KEY_PRESS:
            self._base_command += delta
        elif event.type == carb.input.KeyboardEventType.KEY_RELEASE:
            self._base_command -= delta
        return True

    def _unsubscribe_keyboard(self) -> None:
        """Unsubscribe from keyboard events if currently subscribed."""
        if self._sub_keyboard is not None:
            self._input.unsubscribe_to_keyboard_events(self._keyboard, self._sub_keyboard)
            self._sub_keyboard = None

    def physics_cleanup(self) -> None:
        """Clean up physics resources and restore the physics state captured in ``setup_scene``."""
        self._deregister_physics_callback()
        self._unsubscribe_keyboard()

        if self._runner is not None:
            self._runner.close()
        self._runner = None
        self._physics_ready = False

        restore_physics_simulation_state(self._prev_physics_sim_device, self._prev_fabric_enabled)
        self._prev_physics_sim_device = None
        self._prev_fabric_enabled = None
