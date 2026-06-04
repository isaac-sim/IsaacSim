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

"""Simple Franka pick-and-place interactive example."""

from __future__ import annotations

import isaacsim.core.experimental.utils.app as app_utils
from isaacsim.core.simulation_manager import SimulationManager
from isaacsim.examples.base.base_sample_experimental import BaseSample
from isaacsim.robot.experimental.manipulators.examples.franka import FrankaPickPlace


class FrankaPickPlaceInteractive(BaseSample):
    """Interactive sample for simple pick-and-place demonstration."""

    def __init__(self) -> None:
        super().__init__()
        self.controller: FrankaPickPlace = None
        self._is_executing = False
        self._physics_callback_id = None

    def setup_scene(self) -> None:
        """Set up the scene with robot and cube."""
        # Create controller and setup scene
        self.controller = FrankaPickPlace()
        self.controller.setup_scene()
        print("Scene setup complete with simplified controller")

    async def setup_post_load(self) -> None:
        """Called after the scene is loaded."""
        print("Simple pick-place scene loaded successfully")

    async def setup_pre_reset(self) -> None:
        """Called before world reset."""
        # Stop any ongoing execution and remove callbacks
        if self._physics_callback_id is not None:
            SimulationManager.deregister_callback(self._physics_callback_id)
            self._physics_callback_id = None

        if self.controller:
            self.controller.reset()
        self._is_executing = False

    async def setup_post_reset(self) -> None:
        """Called after world reset."""
        if self.controller:
            # Ensure robot starts in a good position
            self.controller.reset_robot()

    async def setup_post_clear(self) -> None:
        """Called after clearing the scene."""
        # Stop any ongoing execution and remove callbacks
        if self._physics_callback_id is not None:
            SimulationManager.deregister_callback(self._physics_callback_id)
            self._physics_callback_id = None
        self.controller = None
        self._is_executing = False

    def physics_cleanup(self) -> None:
        """Clean up world resources."""
        # Stop any ongoing execution and remove callbacks
        if self._physics_callback_id is not None:
            SimulationManager.deregister_callback(self._physics_callback_id)
            self._physics_callback_id = None

        self.controller = None
        self._is_executing = False

    def _pick_place_physics_callback(self, dt: float, context: object) -> None:
        """Physics callback to execute pick-and-place step by step.

        Args:
            dt: Time delta since last physics step.
            context: Physics simulation context.
        """
        if not self._is_executing or self.controller is None:
            return

        if self.controller.is_done():
            print("Pick-and-place completed successfully!")
            self._is_executing = False

            # Remove the physics callback
            if self._physics_callback_id is not None:
                SimulationManager.deregister_callback(self._physics_callback_id)
                self._physics_callback_id = None
            return

        # Execute one step of the pick-and-place operation
        try:
            step_executed = self.controller.forward()
            if not step_executed:
                print("Forward step failed!")
                self._is_executing = False
                if self._physics_callback_id is not None:
                    SimulationManager.deregister_callback(self._physics_callback_id)
                    self._physics_callback_id = None
        except Exception as e:
            print(f"Error during pick-and-place step: {e}")
            self._is_executing = False
            if self._physics_callback_id is not None:
                SimulationManager.deregister_callback(self._physics_callback_id)
                self._physics_callback_id = None

    def get_controller_status(self) -> dict:
        """Get current status of the controller.

        Returns:
            Current status information of the pick-and-place controller.
        """
        if self.controller:
            return self.controller.get_current_status()
        else:
            return {"error": "Controller not initialized"}

    def is_executing(self) -> bool:
        """Check if pick-and-place is currently executing.

        Returns:
            True if pick-and-place operation is in progress.
        """
        return self._is_executing

    # Methods for UI integration
    async def execute_pick_place_async(self) -> bool | None:
        """Async wrapper for pick-and-place execution.

        Returns:
            False if execution cannot start due to controller not being initialized or already running.
        """
        if self.controller is None:
            print("ERROR: Controller not initialized")
            return False

        if self._is_executing:
            print("Pick-and-place already in progress...")
            return False

        print("Starting pick-and-place execution...")
        self._is_executing = True

        # Reset the robot/controller to ensure clean start
        self.controller.reset()
        print("Robot reset for clean start")

        # Register physics callback using SimulationManager
        from isaacsim.core.simulation_manager import SimulationEvent

        self._physics_callback_id = SimulationManager.register_callback(
            self._pick_place_physics_callback, event=SimulationEvent.PHYSICS_POST_STEP
        )

        # Start timeline playback via app_utils
        app_utils.play()
        await app_utils.update_app_async()
