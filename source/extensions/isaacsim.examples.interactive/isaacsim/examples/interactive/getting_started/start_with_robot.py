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

"""Interactive example demonstrating basic robot simulation setup with physics integration using experimental Isaac Sim API."""

import time

import isaacsim.core.experimental.utils.stage as stage_utils
import omni.timeline
from isaacsim.core.rendering_manager import ViewportManager
from isaacsim.core.simulation_manager import IsaacEvents, SimulationManager
from isaacsim.examples.base.base_sample_experimental import BaseSample
from isaacsim.storage.native import get_assets_root_path


class GettingStartedRobot(BaseSample):
    """A foundational example class demonstrating basic robot simulation setup and physics integration.

    This class serves as an introductory example for Isaac Sim users, showcasing how to create a basic robot
    simulation environment using the experimental API. It demonstrates essential concepts including scene setup with
    a ground plane environment, camera positioning, physics callback management, and timeline control.

    The class handles the complete simulation lifecycle from initial scene creation through physics stepping and
    cleanup. It sets up a default grid environment, positions the camera for optimal viewing, and registers physics
    callbacks to monitor and interact with simulated objects during runtime.

    Key features include:
    - Scene setup with default grid environment using experimental stage utilities
    - Camera positioning for better scene visualization
    - Physics callback registration for real-time simulation monitoring
    - Timeline management for simulation control
    - Proper resource cleanup during reset and clear operations

    The class maintains handles for potential robot components (car and arm) and provides a foundation for
    monitoring their joint states during physics simulation steps. The physics callback demonstrates how to
    integrate custom logic into the simulation loop using the SimulationManager's event system.

    This example is ideal for users new to Isaac Sim who want to understand the basic structure and flow of a
    robot simulation application, including proper initialization, scene management, and cleanup procedures.
    """

    def __init__(self) -> None:
        super().__init__()
        self._timeline = omni.timeline.get_timeline_interface()
        self.print_state = False
        self.car_handle = None
        self.arm_handle = None
        self._physics_callback_id = None

    def setup_scene(self) -> None:
        """Set up the scene with ground plane using experimental API."""
        # Add default environment using experimental stage utils
        stage_utils.add_reference_to_stage(
            usd_path=get_assets_root_path() + "/Isaac/Environments/Grid/default_environment.usd",
            path="/World/ground",
        )

    async def setup_post_load(self) -> None:
        """Sets up the scene after loading.

        Moves the camera to a better vantage point, registers physics callbacks, and performs a quick
        start/stop cycle to reset the physics timeline.
        """
        # move camera to a better vanatage point
        ViewportManager.set_camera_view("/OmniverseKit_Persp", eye=[5.0, 0.0, 1.5], target=[0.00, 0.00, 1.00])

        # Add physics callback using SimulationManager (experimental API)
        self._physics_callback_id = SimulationManager.register_callback(
            self.on_physics_step, event=IsaacEvents.POST_PHYSICS_STEP
        )

        # do a quick start and stop to reset the physics timeline
        self._timeline.play()
        time.sleep(1)
        self._timeline.stop()

    def on_physics_step(self, step_size: float, context: object) -> None:
        """Physics callback - note the signature includes context parameter.

        Args:
            step_size: The physics simulation step size.
            context: The physics simulation context.
        """
        if self.print_state:
            if self.arm_handle:
                print("arm joint state: ", self.arm_handle.get_dof_positions())
            if self.car_handle:
                print("car joint state: ", self.car_handle.get_dof_positions())

    async def setup_pre_reset(self) -> None:
        """Prepares the scene before reset.

        Removes the physics callback to ensure clean reset state.
        """
        # Remove physics callback before reset
        if self._physics_callback_id is not None:
            SimulationManager.deregister_callback(self._physics_callback_id)
            self._physics_callback_id = None

    async def setup_post_reset(self) -> None:
        """Sets up the scene after reset.

        Re-registers the physics callback and stops the timeline to ensure proper reset state.
        """
        # Re-register physics callback after reset
        self._physics_callback_id = SimulationManager.register_callback(
            self.on_physics_step, event=IsaacEvents.POST_PHYSICS_STEP
        )
        self._timeline.stop()

    async def setup_post_clear(self) -> None:
        """Called after clearing the scene."""
        # Remove physics callback on clear
        if self._physics_callback_id is not None:
            SimulationManager.deregister_callback(self._physics_callback_id)
            self._physics_callback_id = None

    def physics_cleanup(self) -> None:
        """Clean up physics resources."""
        if self._physics_callback_id is not None:
            SimulationManager.deregister_callback(self._physics_callback_id)
            self._physics_callback_id = None
