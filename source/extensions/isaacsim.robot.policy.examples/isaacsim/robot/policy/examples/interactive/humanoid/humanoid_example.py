# SPDX-FileCopyrightText: Copyright (c) 2020-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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

"""Interactive humanoid robot simulation example using H1 robot with GPU-accelerated physics and keyboard control."""

import carb
import isaacsim.core.experimental.utils.stage as stage_utils
import omni
import omni.appwindow
from isaacsim.core.deprecation_manager import import_module
from isaacsim.core.simulation_manager import SimulationManager
from isaacsim.core.simulation_manager.impl.isaac_events import IsaacEvents
from isaacsim.examples.base.base_sample_experimental import BaseSample
from isaacsim.robot.policy.examples.interactive.utils import (
    restore_physics_simulation_state,
    snapshot_physics_simulation_state,
)
from isaacsim.robot.policy.examples.robots import H1FlatTerrainPolicy
from isaacsim.storage.native import get_assets_root_path
from pxr import UsdPhysics, UsdShade


class HumanoidExample(BaseSample):
    """A humanoid robot simulation example using H1 robot with GPU-accelerated physics.

    This class demonstrates a complete humanoid robot simulation setup with real-time control capabilities.
    It configures a high-frequency physics simulation (200 Hz) with GPU acceleration and provides keyboard-based
    control for the H1 humanoid robot. The example includes proper scene setup, physics callbacks, and cleanup
    management.

    The simulation uses optimized settings with 200 Hz physics timestep and 25 Hz rendering to ensure smooth
    real-time performance. The H1 robot is controlled through a policy-based system that processes movement
    commands and maintains balance during locomotion.

    Keyboard controls:
        - NUMPAD_8 or UP: Move forward
        - NUMPAD_4 or LEFT: Turn left
        - NUMPAD_6 or RIGHT: Turn right

    The example automatically handles robot initialization after scene reset and manages GPU memory resources
    through proper cleanup routines. Physics tensors are validated each step to ensure robust simulation
    restart capabilities.
    """

    def __init__(self) -> None:
        super().__init__()
        # Configure simulation settings for GPU dynamics with high-frequency physics
        self._world_settings["stage_units_in_meters"] = 1.0
        self._world_settings["physics_dt"] = 1.0 / 200.0  # 200 Hz physics
        self._world_settings["rendering_dt"] = 8.0 / 200.0  # 25 Hz rendering (8 physics steps per render)
        self._world_settings["device"] = "cuda"
        self._world_settings["backend"] = "torch"

        self._base_command = None
        self._physics_ready = False
        self.h1 = None
        self._physics_callback_id = None
        self._event_timer_callback = None
        self._sub_keyboard = None
        self._input = None
        self._keyboard = None
        self._prev_physics_sim_device: str | None = None
        self._prev_fabric_enabled: bool | None = None

        # Bindings for keyboard to command
        self._input_keyboard_mapping = {
            # forward command
            "NUMPAD_8": [0.75, 0.0, 0.0],
            "UP": [0.75, 0.0, 0.0],
            # yaw command (positive)
            "NUMPAD_4": [0.0, 0.0, 0.75],
            "LEFT": [0.0, 0.0, 0.75],
            # yaw command (negative)
            "NUMPAD_6": [0.0, 0.0, -0.75],
            "RIGHT": [0.0, 0.0, -0.75],
        }

    def _apply_ground_material(self, static_friction: float, dynamic_friction: float, restitution: float) -> None:
        """Apply physics material to the ground plane.

        Args:
            static_friction: Static friction coefficient.
            dynamic_friction: Dynamic friction coefficient.
            restitution: Restitution coefficient.
        """
        stage = omni.usd.get_context().get_stage()
        material_path = "/World/ground/Looks/PhysicsMaterial"

        material = UsdShade.Material.Define(stage, material_path)
        physics_material = UsdPhysics.MaterialAPI.Apply(material.GetPrim())
        physics_material.CreateStaticFrictionAttr().Set(static_friction)
        physics_material.CreateDynamicFrictionAttr().Set(dynamic_friction)
        physics_material.CreateRestitutionAttr().Set(restitution)

        ground_geom_path = "/World/ground/GroundPlane/CollisionPlane"
        ground_geom = stage.GetPrimAtPath(ground_geom_path)
        if ground_geom.IsValid():
            binding_api = UsdShade.MaterialBindingAPI.Apply(ground_geom)
            binding_api.Bind(material)

    def setup_scene(self) -> None:
        """Set up the scene with robot and environment."""
        # Snapshot prior physics device/fabric state so cleanup can restore it.
        self._prev_physics_sim_device, self._prev_fabric_enabled = snapshot_physics_simulation_state()

        # Set device and backend BEFORE creating robot so it uses GPU
        SimulationManager.set_backend(self._world_settings["backend"])
        SimulationManager.set_physics_sim_device(self._world_settings["device"])
        SimulationManager.get_available_physics_engines(verbose=True)

        assets_root_path = get_assets_root_path()
        if assets_root_path is None:
            carb.log_error("Could not find Isaac Sim assets folder")

        stage_utils.add_reference_to_stage(
            usd_path=assets_root_path + "/Isaac/Environments/Grid/default_environment.usd",
            path="/World/ground",
        )

        # Apply physics material to ground to match training configuration
        self._apply_ground_material(static_friction=1.0, dynamic_friction=1.0, restitution=0.0)

        # Create H1 robot (auto-detects active physics engine for policy selection)
        self.h1 = H1FlatTerrainPolicy(
            prim_path="/World/H1",
            position=[0, 0, 1.05],
        )

    async def setup_post_load(self) -> None:
        """Setup keyboard input and physics callback after initial load."""
        self._appwindow = omni.appwindow.get_default_app_window()
        self._input = carb.input.acquire_input_interface()
        self._keyboard = self._appwindow.get_keyboard()
        self._sub_keyboard = self._input.subscribe_to_keyboard_events(self._keyboard, self._sub_keyboard_event)

        torch = import_module("torch")
        self._base_command = torch.tensor([0.0, 0.0, 0.0], device="cuda")
        self._physics_ready = False

        # Register physics callback using SimulationManager
        if self._physics_callback_id is None:
            self._physics_callback_id = SimulationManager.register_callback(
                self.on_physics_step, IsaacEvents.POST_PHYSICS_STEP
            )

    async def setup_pre_reset(self) -> None:
        """Called before world reset."""
        # Reset physics ready flag before reset
        self._physics_ready = False

    async def setup_post_reset(self) -> None:
        """Called after world reset."""
        # Reset physics ready flag after reset so robot reinitializes on next play
        self._physics_ready = False

    async def setup_post_clear(self) -> None:
        """Called after clearing the scene."""
        # Deregister physics callback
        if self._physics_callback_id is not None:
            try:
                SimulationManager.deregister_callback(self._physics_callback_id)
            except Exception as e:
                carb.log_warn(f"Could not deregister callback {self._physics_callback_id}: {e}")
            self._physics_callback_id = None

        self._event_timer_callback = None
        self._unsubscribe_keyboard()
        self.h1 = None
        self._physics_ready = False
        self._restore_physics_simulation_state()

    def on_physics_step(self, dt: float, context: object) -> None:
        """Physics step callback - initialize on first step, then run policy.

        Args:
            dt: Delta time for the physics step.
            context: Physics step context.
        """
        if not self.h1:
            return

        # Check if physics tensors are valid, if not, reinitialize
        if not self.h1.robot.is_physics_tensor_entity_valid():
            self._physics_ready = False

        if self._physics_ready:
            # Robot is initialized, run the policy
            self.h1.forward(dt, self._base_command)
        else:
            # First physics step after play - initialize the robot
            self._physics_ready = True
            self.h1.initialize()  # This already sets default state internally
            self.h1.post_reset()

    def _sub_keyboard_event(self, event: object, *args: object, **kwargs: object) -> bool:
        """Handle keyboard input for robot control.

        Args:
            event: The keyboard event.
            *args: Additional positional arguments.
            **kwargs: Additional keyword arguments.

        Returns:
            bool: True to indicate the event was handled.
        """
        torch = import_module("torch")
        if event.type == carb.input.KeyboardEventType.KEY_PRESS:
            # On pressing, the command is incremented
            if event.input.name in self._input_keyboard_mapping:
                self._base_command += torch.tensor(
                    self._input_keyboard_mapping[event.input.name], device=self._base_command.device
                )
        elif event.type == carb.input.KeyboardEventType.KEY_RELEASE:
            # On release, the command is decremented
            if event.input.name in self._input_keyboard_mapping:
                self._base_command -= torch.tensor(
                    self._input_keyboard_mapping[event.input.name], device=self._base_command.device
                )
        return True

    def _unsubscribe_keyboard(self) -> None:
        """Unsubscribe from keyboard events if currently subscribed."""
        if self._sub_keyboard is not None:
            self._input.unsubscribe_to_keyboard_events(self._keyboard, self._sub_keyboard)
            self._sub_keyboard = None

    def physics_cleanup(self) -> None:
        """Clean up physics resources."""
        # Deregister physics callback
        if self._physics_callback_id is not None:
            try:
                SimulationManager.deregister_callback(self._physics_callback_id)
            except Exception as e:
                carb.log_warn(f"Could not deregister callback {self._physics_callback_id}: {e}")
            self._physics_callback_id = None

        self._event_timer_callback = None
        self._unsubscribe_keyboard()
        self.h1 = None
        self._physics_ready = False
        self._restore_physics_simulation_state()

    def _restore_physics_simulation_state(self) -> None:
        """Restore the physics sim device and fabric state captured in ``setup_scene``."""
        restore_physics_simulation_state(self._prev_physics_sim_device, self._prev_fabric_enabled)
        self._prev_physics_sim_device = None
        self._prev_fabric_enabled = None
