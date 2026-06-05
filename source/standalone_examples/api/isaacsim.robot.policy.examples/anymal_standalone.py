# SPDX-FileCopyrightText: Copyright (c) 2021-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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

"""Demonstrate interactive ANYmal robot simulation with keyboard control."""

from isaacsim import SimulationApp

simulation_app = SimulationApp({"headless": False})

import argparse

import carb
import omni.appwindow  # Contains handle to keyboard
import omni.timeline
from isaacsim.core.deprecation_manager import import_module
from isaacsim.core.experimental.utils.stage import define_prim
from isaacsim.core.rendering_manager import RenderingManager
from isaacsim.core.simulation_manager import SimulationManager
from isaacsim.core.simulation_manager.impl.isaac_events import IsaacEvents
from isaacsim.robot.policy.examples.robots import AnymalFlatTerrainPolicy
from isaacsim.storage.native import get_assets_root_path

torch = import_module("torch")

parser = argparse.ArgumentParser(description="Select simulation device.")
parser.add_argument("--test", default=False, action="store_true", help="Run in test mode")
parser.add_argument("--device", type=str, choices=["cpu", "cuda"], default="cpu", help="Simulation device")

args, unknown = parser.parse_known_args()
print(f"Using device: {args.device}")


class Anymal_runner(object):
    """Interactive ANYmal robot simulation runner with keyboard control.

    Creates a simulation environment with an ANYmal robot in a warehouse setting,
    handling physics simulation, visualization, and keyboard-based velocity commands.
    Supports forward/backward motion, lateral movement, and turning through numpad
    or arrow key controls.
    """

    def __init__(self, physics_dt: float, render_dt: float) -> None:
        """Initialize the simulation environment with ANYmal robot in a warehouse.

        Args:
            physics_dt: Physics simulation timestep in seconds
            render_dt: Rendering timestep in seconds for visualization updates
        """
        # spawn physics scene
        # TODO: physics scene should be created by simulation manager
        define_prim("/World/PhysicsScene", "PhysicsScene")

        # set rendering manager
        RenderingManager.set_dt(8.0 / 200.0)

        # spawn simulation manager
        SimulationManager.set_physics_sim_device(args.device)
        SimulationManager.set_physics_dt(1.0 / 200.0)

        assets_root_path = get_assets_root_path()
        if assets_root_path is None:
            carb.log_error("Could not find Isaac Sim assets folder")

        # spawn warehouse scene
        prim = define_prim("/World/Warehouse", "Xform")
        asset_path = assets_root_path + "/Isaac/Environments/Simple_Warehouse/warehouse.usd"
        prim.GetReferences().AddReference(asset_path)

        self._anymal = AnymalFlatTerrainPolicy(
            prim_path="/World/Anymal",
            position=[0, 0, 0.7],
            usd_path=assets_root_path + "/Isaac/Robots/ANYbotics/anymal_c/anymal_c.usd",
        )

        self._base_command = torch.zeros(3, device=args.device, dtype=torch.float32)

        # bindings for keyboard to command
        self._input_keyboard_mapping = {
            # forward command
            "NUMPAD_8": torch.tensor([1.0, 0.0, 0.0], device=args.device),
            "UP": torch.tensor([1.0, 0.0, 0.0], device=args.device),
            # back command
            "NUMPAD_2": torch.tensor([-1.0, 0.0, 0.0], device=args.device),
            "DOWN": torch.tensor([-1.0, 0.0, 0.0], device=args.device),
            # left command
            "NUMPAD_6": torch.tensor([0.0, -1.0, 0.0], device=args.device),
            "RIGHT": torch.tensor([0.0, -1.0, 0.0], device=args.device),
            # right command
            "NUMPAD_4": torch.tensor([0.0, 1.0, 0.0], device=args.device),
            "LEFT": torch.tensor([0.0, 1.0, 0.0], device=args.device),
            # yaw command (positive)
            "NUMPAD_7": torch.tensor([0.0, 0.0, 1.0], device=args.device),
            "N": torch.tensor([0.0, 0.0, 1.0], device=args.device),
            # yaw command (negative)
            "NUMPAD_9": torch.tensor([0.0, 0.0, -1.0], device=args.device),
            "M": torch.tensor([0.0, 0.0, -1.0], device=args.device),
        }
        self.needs_reset = False
        self.first_step = True

    def setup(self) -> None:
        """Configure simulation input handling and physics callbacks.

        Sets up the keyboard event listener for robot control and registers
        the physics step callback for robot state updates and control.
        """
        self._appwindow = omni.appwindow.get_default_app_window()
        self._input = carb.input.acquire_input_interface()
        self._keyboard = self._appwindow.get_keyboard()
        self._sub_keyboard = self._input.subscribe_to_keyboard_events(self._keyboard, self._sub_keyboard_event)
        _physics_callback_id = SimulationManager.register_callback(self.on_physics_step, IsaacEvents.POST_PHYSICS_STEP)

    def on_physics_step(self, step_size: float, context) -> None:
        """Physics simulation step callback handler.

        Manages robot initialization on first step, handles simulation resets,
        and executes the robot's control policy to apply joint torques based
        on the current command velocity.

        Args:
            step_size: Physics timestep duration in seconds
            context: Physics simulation context
        """
        if self.first_step:
            self._anymal.initialize()
            self.first_step = False
        elif self.needs_reset:
            self.needs_reset = False
            self.first_step = True
        else:
            self._anymal.forward(step_size, self._base_command)

    def run(self) -> None:
        """Main simulation loop.

        Continuously steps the physics simulation with rendering enabled,
        monitoring for simulation stop conditions that trigger resets.
        Runs until the simulation application is closed.
        """
        # change to sim running
        frame_count = 0
        while simulation_app.is_running():
            simulation_app.update()
            if not SimulationManager.is_simulating():
                self.needs_reset = True
            frame_count += 1
            if args.test and frame_count >= 10:
                break
        return

    def _sub_keyboard_event(self, event: carb.input.KeyboardEvent, *args, **kwargs) -> bool:
        """Handle keyboard input events for robot control.

        Processes key press and release events to update the robot's command velocity.
        Supports numpad and arrow keys for movement control:
        - 8/Up: Forward motion
        - 2/Down: Backward motion
        - 4/Left: Leftward motion
        - 6/Right: Rightward motion
        - 7/N: Turn left
        - 9/M: Turn right

        Args:
            event: Keyboard event containing key press/release information
            *args: Variable positional arguments
            **kwargs: Variable keyword arguments

        Returns:
            True to continue processing keyboard events
        """
        # when a key is pressed for released  the command is adjusted w.r.t the key-mapping
        if event.type == carb.input.KeyboardEventType.KEY_PRESS:
            if event.input.name in self._input_keyboard_mapping:
                self._base_command += self._input_keyboard_mapping[event.input.name]

        elif event.type == carb.input.KeyboardEventType.KEY_RELEASE:
            # on release, the command is decremented
            if event.input.name in self._input_keyboard_mapping:
                self._base_command -= self._input_keyboard_mapping[event.input.name]
        return True


def main() -> None:
    """Entry point for the ANYmal simulation demo.

    Sets up and runs an interactive simulation of an ANYmal robot in a warehouse
    environment with keyboard-based velocity control. Uses a 200Hz physics update
    rate and 60Hz rendering rate for smooth visualization.
    """
    physics_dt = 1 / 200.0
    render_dt = 1 / 60.0

    runner = Anymal_runner(physics_dt=physics_dt, render_dt=render_dt)
    simulation_app.update()
    runner.setup()
    simulation_app.update()
    omni.timeline.get_timeline_interface().play()
    simulation_app.update()
    runner.run()
    simulation_app.close()


if __name__ == "__main__":
    main()
