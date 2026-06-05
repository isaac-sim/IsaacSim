# SPDX-FileCopyrightText: Copyright (c) 2022-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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

"""Interactive Franka robot control system using the Cortex framework for behavior-based manipulation tasks."""

import numpy as np
import omni
from isaacsim.core.api.objects import DynamicCuboid
from isaacsim.cortex.examples.cortex.cortex_base import CortexBase
from isaacsim.cortex.framework.cortex_utils import load_behavior_module
from isaacsim.cortex.framework.dfb import DfDiagnosticsMonitor
from isaacsim.cortex.framework.robot import add_franka_to_stage


class CubeSpec:
    """Specification for creating cube objects in the Isaac Sim interactive examples.

    This class encapsulates the properties needed to define a cube object, including its identifier and visual
    appearance. It is used to specify cube parameters before creating actual cube objects in the simulation scene.

    Args:
        name: Identifier for the cube object.
        color: RGB color values for the cube's appearance.
    """

    def __init__(self, name: str, color: list) -> None:
        self.name = name
        self.color = np.array(color)


class ContextStateMonitor(DfDiagnosticsMonitor):
    """State monitor to read the context and pass it to the UI.

    For these behaviors, the context has a `diagnostic_message` that contains the text to be displayed, and each
    behavior implements its own monitor to update that.

    Args:
        print_dt: Time interval between diagnostic prints.
        diagnostic_fn: Optional function to call with the context for diagnostics.
    """

    def __init__(self, print_dt: float, diagnostic_fn: callable = None) -> None:
        super().__init__(print_dt=print_dt)
        self.diagnostic_fn = diagnostic_fn

    def print_diagnostics(self, context: object) -> None:
        """Prints diagnostic information from the context.

        Calls the diagnostic function if one is provided to display context information.

        Args:
            context: The context containing diagnostic information to be displayed.
        """
        if self.diagnostic_fn:
            self.diagnostic_fn(context)


class FrankaCortex(CortexBase):
    """Cortex-based Franka robot control system for interactive demonstrations.

    Provides a complete framework for controlling a Franka robot using Cortex behaviors in an interactive
    environment. The class sets up a scene with the Franka robot and colored cube obstacles, manages behavior
    loading and execution, and provides real-time monitoring capabilities for diagnostic information and
    decision stack visualization.

    The system creates a world with:
    - A Franka robot at /World/Franka
    - Four colored cube obstacles (Red, Blue, Yellow, Green) positioned in front of the robot
    - Ground plane for physics simulation

    Supports loading and switching between different Cortex behaviors dynamically, with each behavior
    implementing its own decision network for robot control. The monitor function receives diagnostic
    messages and decision stack information for UI display or logging purposes.

    Args:
        monitor_fn: Optional callback function that receives diagnostic messages and decision stack
            information. Called with two string arguments: diagnostic_message and decision_stack.
    """

    def __init__(self, monitor_fn: callable = None) -> None:
        super().__init__()
        self._monitor_fn = monitor_fn
        self.behavior = None
        self.robot = None
        self.context_monitor = ContextStateMonitor(print_dt=0.25, diagnostic_fn=self._on_monitor_update)

    def setup_scene(self) -> None:
        """Sets up the simulation scene with a Franka robot and colored cube obstacles.

        Adds a Franka robot to the world and creates four colored cubes (Red, Blue, Yellow, Green) as obstacles that the robot must navigate around. Also adds a default ground plane to the scene.
        """
        world = self.get_world()
        self.robot = world.add_robot(add_franka_to_stage(name="franka", prim_path="/World/Franka"))

        obs_specs = [
            CubeSpec("RedCube", [0.7, 0.0, 0.0]),
            CubeSpec("BlueCube", [0.0, 0.0, 0.7]),
            CubeSpec("YellowCube", [0.7, 0.7, 0.0]),
            CubeSpec("GreenCube", [0.0, 0.7, 0.0]),
        ]
        width = 0.0515
        for i, (x, spec) in enumerate(zip(np.linspace(0.3, 0.7, len(obs_specs)), obs_specs)):
            obj = world.scene.add(
                DynamicCuboid(
                    prim_path=f"/World/Obs/{spec.name}",
                    name=spec.name,
                    size=width,
                    color=spec.color,
                    position=np.array([x, -0.4, width / 2]),
                )
            )
            self.robot.register_obstacle(obj)
        world.scene.add_default_ground_plane()

    async def load_behavior(self, behavior: object) -> None:
        """Loads a behavior module and creates its decider network for the robot.

        Args:
            behavior: The behavior module to load and execute.
        """
        world = self.get_world()
        self.behavior = behavior
        self.decider_network = load_behavior_module(self.behavior).make_decider_network(self.robot)
        self.decider_network.context.add_monitor(self.context_monitor.monitor)
        world.add_decider_network(self.decider_network)

    def clear_behavior(self) -> None:
        """Clears all loaded behaviors and logical state monitors from the world."""
        world = self.get_world()
        world._logical_state_monitors.clear()
        world._behaviors.clear()

    async def setup_post_load(self, soft: bool = False) -> None:
        """Sets up the decider network after loading a behavior module.

        Creates and configures the decider network from the loaded behavior module, adds monitoring capabilities, and integrates it with the world.

        Args:
            soft: Whether to perform a soft setup.
        """
        world = self.get_world()
        prim_path = "/World/Franka"
        if not self.robot:
            self.robot = world._robots["franka"]
        self.decider_network = load_behavior_module(self.behavior).make_decider_network(self.robot)
        self.decider_network.context.add_monitor(self.context_monitor.monitor)
        world.add_decider_network(self.decider_network)
        await omni.kit.app.get_app().next_update_async()

    def _on_monitor_update(self, context: object) -> None:
        """Handles context updates from the behavior monitoring system.

        Extracts diagnostic messages and decision stack information from the context and passes them to the registered monitor function for UI display.

        Args:
            context: The context object containing diagnostic information and current state.
        """
        diagnostic = ""
        decision_stack = ""
        if hasattr(context, "diagnostics_message"):
            diagnostic = context.diagnostics_message
        if self.decider_network._decider_state.stack:
            decision_stack = "\n".join(
                [
                    "{0}{1}".format("  " * i, element)
                    for i, element in enumerate(str(i) for i in self.decider_network._decider_state.stack)
                ]
            )

        if self._monitor_fn:
            self._monitor_fn(diagnostic, decision_stack)

    def _on_physics_step(self, step_size: float) -> None:
        """Handles physics simulation step updates.

        Called during each physics simulation step to advance the world state without rendering.

        Args:
            step_size: The physics simulation step size.
        """
        world = self.get_world()

        world.step(False, False)

    async def on_event_async(self) -> None:
        """Handles asynchronous event processing for simulation control.

        Resets the Cortex world, registers physics step callbacks, and starts the simulation playback.
        """
        world = self.get_world()
        await omni.kit.app.get_app().next_update_async()
        world.reset_cortex()
        world.add_physics_callback("sim_step", self._on_physics_step)
        await world.play_async()

    async def setup_pre_reset(self) -> None:
        """Prepares the world for reset by cleaning up physics callbacks.

        Removes any existing physics step callbacks before performing a world reset.
        """
        world = self.get_world()
        if world.physics_callback_exists("sim_step"):
            world.remove_physics_callback("sim_step")

    def world_cleanup(self) -> None:
        """Performs cleanup operations when the world is being destroyed."""
