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

"""Go2 robot locomotion example with keyboard control."""

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
from isaacsim.robot.policy.examples.robots import Go2FlatTerrainPolicy
from isaacsim.storage.native import get_assets_root_path
from pxr import UsdPhysics, UsdShade


class Go2Example(BaseSample):
    """Go2 robot locomotion example with keyboard control.

    The physics engine is determined by the currently active engine in SimulationManager.
    Users can switch engines using the Physics Engine menu in the viewport before loading.
    """

    def __init__(self) -> None:

        super().__init__()

        # Configure simulation settings matching Isaac Lab training
        self._world_settings["stage_units_in_meters"] = 1.0
        self._world_settings["physics_dt"] = 0.005  # 200 Hz physics (matches training)
        self._world_settings["rendering_dt"] = 0.02  # 50 Hz rendering (4 physics steps per render)
        self._world_settings["device"] = "cuda"  # GPU dynamics
        self._world_settings["backend"] = "torch"  # PyTorch backend

        self._base_command = None
        self._physics_ready = False
        self.go2 = None
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
            "NUMPAD_8": [1.5, 0.0, 0.0],
            "UP": [1.5, 0.0, 0.0],
            # back command
            "NUMPAD_2": [-1.5, 0.0, 0.0],
            "DOWN": [-1.5, 0.0, 0.0],
            # left command
            "NUMPAD_6": [0.0, -1.5, 0.0],
            "RIGHT": [0.0, -1.5, 0.0],
            # right command
            "NUMPAD_4": [0.0, 1.5, 0.0],
            "LEFT": [0.0, 1.5, 0.0],
            # yaw command (positive)
            "NUMPAD_7": [0.0, 0.0, 1.5],
            "N": [0.0, 0.0, 1.5],
            # yaw command (negative)
            "NUMPAD_9": [0.0, 0.0, -1.5],
            "M": [0.0, 0.0, -1.5],
        }

    def _apply_ground_material(self, static_friction: float, dynamic_friction: float, restitution: float) -> None:
        """Apply physics material to the ground plane.

        Args:
            static_friction: Static friction coefficient
            dynamic_friction: Dynamic friction coefficient
            restitution: Restitution coefficient (bounciness)
        """
        stage = omni.usd.get_context().get_stage()
        material_path = "/World/ground/Looks/PhysicsMaterial"

        # Create physics material
        material = UsdShade.Material.Define(stage, material_path)
        physics_material = UsdPhysics.MaterialAPI.Apply(material.GetPrim())
        physics_material.CreateStaticFrictionAttr().Set(static_friction)
        physics_material.CreateDynamicFrictionAttr().Set(dynamic_friction)
        physics_material.CreateRestitutionAttr().Set(restitution)

        # Apply material to ground geometry
        ground_geom_path = "/World/ground/GroundPlane/CollisionPlane"
        ground_geom = stage.GetPrimAtPath(ground_geom_path)
        if ground_geom.IsValid():
            binding_api = UsdShade.MaterialBindingAPI.Apply(ground_geom)
            binding_api.Bind(material)

    async def load_world_async(self) -> None:
        """Load world with desired physics engine."""
        await super().load_world_async()

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

        # Add ground plane environment for physics simulation
        ground_plane = stage_utils.add_reference_to_stage(
            usd_path=get_assets_root_path() + "/Isaac/Environments/Grid/default_environment.usd",
            path="/World/ground",
        )

        # Apply physics material to ground to match training configuration
        self._apply_ground_material(static_friction=1.0, dynamic_friction=1.0, restitution=0.0)

        # Create Go2 robot (auto-detects active physics engine for policy selection)
        self.go2 = Go2FlatTerrainPolicy(
            prim_path="/World/Go2",
            position=[0, 0, 0.5],
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
        self._physics_ready = False

    async def setup_post_reset(self) -> None:
        """Called after world reset."""
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
        self.go2 = None
        self._physics_ready = False
        self._restore_physics_simulation_state()

    def on_physics_step(self, dt: float, context: object) -> None:
        """Physics step callback - initialize on first step, then run policy at decimated rate.

        Args:
            dt: Time delta for the physics step.
            context: Physics step context information.
        """
        if not self.go2:
            return

        # Check if physics tensors are valid, if not, reinitialize
        if not self.go2.robot.is_physics_tensor_entity_valid():
            self._physics_ready = False

        if self._physics_ready:
            self.go2.forward(dt, self._base_command)
        else:
            self._physics_ready = True
            self.go2.initialize()
            self.go2.post_reset()

    def _sub_keyboard_event(self, event: object, *args: object, **kwargs: object) -> bool:
        """Handle keyboard input for robot control.

        Args:
            event: Keyboard event data.
            *args: Additional positional arguments.
            **kwargs: Additional keyword arguments.

        Returns:
            bool: True to indicate event was handled.
        """
        torch = import_module("torch")
        if event.type == carb.input.KeyboardEventType.KEY_PRESS:
            if event.input.name in self._input_keyboard_mapping:
                self._base_command += torch.tensor(
                    self._input_keyboard_mapping[event.input.name], device=self._base_command.device
                )
        elif event.type == carb.input.KeyboardEventType.KEY_RELEASE:
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
        self.go2 = None
        self._physics_ready = False
        self._restore_physics_simulation_state()

    def _restore_physics_simulation_state(self) -> None:
        """Restore the physics sim device and fabric state captured in ``setup_scene``."""
        restore_physics_simulation_state(self._prev_physics_sim_device, self._prev_fabric_enabled)
        self._prev_physics_sim_device = None
        self._prev_fabric_enabled = None
