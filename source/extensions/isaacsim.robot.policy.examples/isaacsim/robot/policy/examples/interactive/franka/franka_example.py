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

"""Example demonstration of a Franka robot performing an open drawer task using a policy-based approach."""

import numpy as np
import omni.timeline
from isaacsim.core.experimental.prims import Articulation
from isaacsim.core.experimental.utils.stage import add_reference_to_stage
from isaacsim.core.simulation_manager import SimulationManager
from isaacsim.examples.base.base_sample_experimental import BaseSample
from isaacsim.robot.policy.examples.robots.franka import FrankaOpenDrawerPolicy
from isaacsim.storage.native import get_assets_root_path


class FrankaExample(BaseSample):
    """Example demonstration of a Franka robot performing an open drawer task using a policy-based approach.

    This class creates a complete simulation scene with a Franka Emika Panda robot and a cabinet, where the robot
    autonomously attempts to open a cabinet drawer using a learned policy. The scene includes a ground plane,
    a Sektion cabinet positioned in front of the robot, and the Franka robot equipped with an open drawer policy.

    The simulation runs with physics at 200 Hz and rendering at 60 Hz. The robot is initialized on the first
    physics step after play begins, and then continuously executes its policy to interact with the cabinet drawer.
    The simulation automatically resets every 10 seconds to demonstrate the task repeatedly.

    The class manages the complete lifecycle of the simulation, including scene setup, robot initialization,
    physics stepping, and cleanup. It uses the SimulationManager to register physics callbacks for real-time
    control and provides automatic reset functionality for continuous demonstration.
    """

    def __init__(self) -> None:
        super().__init__()
        self._world_settings["stage_units_in_meters"] = 1.0
        self._world_settings["physics_dt"] = 1.0 / 200.0
        self._world_settings["rendering_dt"] = 1.0 / 60.0

        self._physics_ready = False
        self.franka = None
        self.cabinet = None
        self._timeline = omni.timeline.get_timeline_interface()
        self._physics_callback_id = None
        self._time_elapsed = 0.0

    def setup_scene(self) -> None:
        """Set up the scene with robot, cabinet, and environment."""
        # Add ground plane
        add_reference_to_stage(
            usd_path=get_assets_root_path() + "/Isaac/Environments/Grid/default_environment.usd",
            path="/World/defaultGroundPlane",
        )

        # Add cabinet
        cabinet_prim_path = "/World/cabinet"
        cabinet_usd_path = get_assets_root_path() + "/Isaac/Props/Sektion_Cabinet/sektion_cabinet_instanceable.usd"

        cabinet_position = [0.8, 0.0, 0.4]
        cabinet_orientation = [0.0, 0.0, 0.0, 1.0]

        add_reference_to_stage(cabinet_usd_path, cabinet_prim_path)

        self.cabinet = Articulation(
            paths=cabinet_prim_path, positions=cabinet_position, orientations=cabinet_orientation
        )

        # Create Franka robot with policy
        self.franka = FrankaOpenDrawerPolicy(prim_path="/World/franka", cabinet=self.cabinet)
        print("Scene setup complete with Franka robot and cabinet")

    async def setup_post_load(self) -> None:
        """Setup physics callback after initial load."""
        self._physics_ready = False

        # Register physics callback using SimulationManager
        from isaacsim.core.simulation_manager.impl.isaac_events import IsaacEvents

        if self._physics_callback_id is None:
            self._physics_callback_id = SimulationManager.register_callback(
                self.on_physics_step, IsaacEvents.POST_PHYSICS_STEP
            )
        print("Franka open drawer scene loaded successfully")

    async def setup_pre_reset(self) -> None:
        """Called before world reset."""
        # Reset physics ready flag before reset
        self._physics_ready = False
        self._time_elapsed = 0.0

    async def setup_post_reset(self) -> None:
        """Called after world reset."""
        # Reset physics ready flag after reset so robot reinitializes on next play
        self._physics_ready = False
        self._time_elapsed = 0.0

        if self.franka:
            # Reset previous action for clean state
            self.franka.previous_action = np.zeros(9)

    async def setup_post_clear(self) -> None:
        """Called after clearing the scene."""
        # Deregister physics callback
        if self._physics_callback_id is not None:
            try:
                SimulationManager.deregister_callback(self._physics_callback_id)
            except Exception as e:
                print(f"Note: Could not deregister callback {self._physics_callback_id}: {e}")
            self._physics_callback_id = None

        self.franka = None
        self.cabinet = None
        self._physics_ready = False
        self._time_elapsed = 0.0

    def on_physics_step(self, dt: float, context: object) -> None:
        """Physics step callback - initialize on first step, then run policy.

        Args:
            dt: Time delta for the physics step.
            context: Physics step context information.
        """
        if not self.franka:
            return

        # Auto-reset at 10 seconds
        if self._physics_ready:
            self._time_elapsed += dt
            if self._time_elapsed >= 10.0:
                self._physics_ready = False
                self._time_elapsed = 0.0
                self._timeline.stop()
                self._timeline.play()
                print("Simulation reset at 10 seconds")
                return

        # Check if physics tensors are valid, if not, reinitialize
        if not self.franka.robot.is_physics_tensor_entity_valid():
            self._physics_ready = False

        if self._physics_ready:
            # Robot is initialized, run the policy
            self.franka.forward(dt)
        else:
            # First physics step after play - initialize the robot
            self._physics_ready = True
            self.franka.initialize()
            self.franka.post_reset()

    def physics_cleanup(self) -> None:
        """Clean up physics resources."""
        # Deregister physics callback
        if self._physics_callback_id is not None:
            try:
                SimulationManager.deregister_callback(self._physics_callback_id)
            except Exception as e:
                print(f"Note: Could not deregister callback {self._physics_callback_id}: {e}")
            self._physics_callback_id = None

        self.franka = None
        self.cabinet = None
        self._physics_ready = False
        self._time_elapsed = 0.0
