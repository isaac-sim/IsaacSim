# SPDX-FileCopyrightText: Copyright (c) 2018-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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

"""Unit tests for Spot robot policy examples."""

import asyncio

import isaacsim.core.experimental.utils.prim as prim_utils
import isaacsim.core.experimental.utils.stage as stage_utils
import isaacsim.core.experimental.utils.transform as transform_utils
import numpy as np

# NOTE:
#   omni.kit.test - std python's unittest module with additional wrapping to add suport for async/await tests
#   For most things refer to unittest docs: https://docs.python.org/3/library/unittest.html
import omni.kit.test
import omni.timeline
from isaacsim.core.deprecation_manager import import_module
from isaacsim.core.simulation_manager import SimulationManager
from isaacsim.core.simulation_manager.impl.isaac_events import IsaacEvents
from isaacsim.robot.policy.examples.robots.spot import SpotFlatTerrainPolicy
from isaacsim.storage.native import get_assets_root_path
from pxr import UsdPhysics

torch = import_module("torch")


class TestSpotCPU(omni.kit.test.AsyncTestCase):
    """Test suite for the Spot robot policy on CPU using Isaac Sim.

    This test class validates the functionality of the SpotFlatTerrainPolicy by running comprehensive tests
    for robot spawning, movement commands, and physics simulation on CPU. It inherits from AsyncTestCase
    to support asynchronous test execution.

    The test suite includes:
    - Spawning and validating the Spot robot in the simulation environment
    - Testing forward movement commands and verifying displacement
    - Testing turning commands and verifying rotational changes
    - Physics simulation integration with proper setup and teardown

    Each test creates a new stage with physics scene, spawns the Spot robot, and runs physics
    simulation to verify expected behaviors. The tests use tensor operations on CPU and measure
    actual robot pose changes to validate command execution.
    """

    def get_device(self) -> torch.device:
        """Return the device to use for tensors. Override in subclasses.

        Returns:
            The PyTorch device for tensor operations.
        """
        return torch.device("cpu")

    async def setUp(self) -> None:
        """Set up the test environment with physics scene and ground plane.

        Initializes a new USD stage, configures physics simulation parameters, spawns a ground plane,
        and sets up the simulation manager with CPU device configuration.
        """
        await stage_utils.create_new_stage_async()
        # This needs to be set so that kit updates match physics updates
        self._physics_rate = 500

        device_str = str(self.get_device())
        backend = "torch" if device_str != "cpu" else "numpy"

        print(f"Setting up test with device: {device_str}, backend: {backend}")

        self._physics_dt = 1 / self._physics_rate
        stage_utils.define_prim("/World/PhysicsScene", "PhysicsScene")

        # spawn simulation manager
        SimulationManager.set_physics_sim_device(device_str)
        SimulationManager.set_physics_dt(self._physics_dt)

        ground_plane = stage_utils.add_reference_to_stage(
            usd_path=get_assets_root_path() + "/Isaac/Environments/Grid/default_environment.usd",
            path="/World/ground",
        )

        self._base_command = torch.zeros(3, dtype=torch.float32, device=self.get_device())
        self._stage = omni.usd.get_context().get_stage()
        self._timeline = omni.timeline.get_timeline_interface()

    async def tearDown(self) -> None:
        """Clean up the test environment.

        Stops the timeline, deregisters physics callbacks, and waits for all assets to finish loading
        before completing teardown.
        """
        await omni.kit.app.get_app().next_update_async()
        self._timeline.stop()
        SimulationManager.deregister_callback(self._physics_callback_id)
        while omni.usd.get_context().get_stage_loading_status()[2] > 0:
            print("tearDown, assets still loading, waiting to finish...")
            await asyncio.sleep(1.0)
        await omni.kit.app.get_app().next_update_async()

    async def test_spot_add(self) -> None:
        """Test spawning a Spot robot and verify its configuration.

        Verifies that the robot has the expected 12 degrees of freedom and that the robot prim
        exists in the stage with proper ArticulationRootAPI.
        """
        await self.spawn_spot()
        await omni.kit.app.get_app().next_update_async()
        self.assertEqual(self._spot.robot.num_dofs, 12)

        # Verify root prim exists at spawn path
        root_prim = stage_utils.get_current_stage().GetPrimAtPath(self._prim_path)
        self.assertIsNotNone(root_prim, f"Robot root prim should exist at {self._prim_path}")
        self.assertTrue(root_prim.IsValid(), "Robot root prim should be valid")

        # Verify articulation root (may be nested under root for some USD assets) has ArticulationRootAPI
        articulation_root_path = self._spot.robot.paths[0]
        articulation_prim = stage_utils.get_current_stage().GetPrimAtPath(articulation_root_path)
        self.assertTrue(
            prim_utils.has_api(articulation_prim, UsdPhysics.ArticulationRootAPI),
            f"Articulation root prim at {articulation_root_path} should have ArticulationRootAPI",
        )

    async def test_robot_move_forward_command(self) -> None:
        """Test robot forward movement command.

        Commands the robot to move forward and verifies that it moves the expected distance
        within reasonable bounds after 1 second of simulation.
        """
        await self.spawn_spot()
        await omni.kit.app.get_app().next_update_async()

        # Get current poses and convert to numpy arrays for efficient operations
        start_positions_wp, _ = self._spot.robot.get_world_poses()

        self.start_pos = start_positions_wp.numpy()[0]

        self._base_command = torch.tensor([2, 0, 0], dtype=torch.float32, device=self.get_device())

        # Simulate for 1 second (60 steps at 60 Hz default)
        for _ in range(60):
            await omni.kit.app.get_app().next_update_async()

        current_positions_wp, _ = self._spot.robot.get_world_poses()

        self.current_pos = current_positions_wp.numpy()[0]

        delta = abs(self.current_pos[0] - self.start_pos[0])

        self.assertGreater(delta, 0.5)
        self.assertLess(delta, 2.0)

    async def test_robot_turn_command(self) -> None:
        """Test robot turn command.

        Commands the robot to turn and verifies that it rotates at least 90 degrees
        after 2 seconds of simulation.
        """
        await self.spawn_spot()
        await omni.kit.app.get_app().next_update_async()

        # Get current poses and convert to numpy arrays for efficient operations
        _, start_orientations_wp = self._spot.robot.get_world_poses()

        self.start_orientation = start_orientations_wp.numpy()[0]

        self._base_command = torch.tensor([0, 0, 1], dtype=torch.float32, device=self.get_device())

        # Simulate for 2 seconds (120 steps at 60 Hz default)
        for _ in range(120):
            await omni.kit.app.get_app().next_update_async()

        _, current_orientations_wp = self._spot.robot.get_world_poses()

        self.current_orientation = current_orientations_wp.numpy()[0]

        # Convert quaternions to rotation matrices and extract yaw angles
        start_rot_matrix = transform_utils.quaternion_to_rotation_matrix(self.start_orientation)
        current_rot_matrix = transform_utils.quaternion_to_rotation_matrix(self.current_orientation)

        # Convert Warp arrays to numpy arrays for indexing
        start_rot_matrix_np = start_rot_matrix.numpy()
        current_rot_matrix_np = current_rot_matrix.numpy()

        # Extract yaw angle from rotation matrix (element [1,0] / [0,0] gives tan(yaw))
        start_yaw = np.arctan2(start_rot_matrix_np[1, 0], start_rot_matrix_np[0, 0])
        current_yaw = np.arctan2(current_rot_matrix_np[1, 0], current_rot_matrix_np[0, 0])

        heading_delta = abs(current_yaw - start_yaw)

        # should have turned at least 90 deg
        self.assertGreater(heading_delta, 1.5)

    async def spawn_spot(self, name: str = "spot") -> None:
        """Spawn a Spot robot in the simulation.

        Creates a SpotFlatTerrainPolicy instance, starts the timeline, initializes the robot,
        and registers a physics callback for robot control.

        Args:
            name: The name for the robot prim in the stage.
        """
        self._prim_path = "/World/" + name

        self._spot = SpotFlatTerrainPolicy(prim_path=self._prim_path, position=[0, 0, 0.7])
        self._timeline.play()
        await omni.kit.app.get_app().next_update_async()
        self._spot.initialize()
        await omni.kit.app.get_app().next_update_async()

        self._physics_callback_id = SimulationManager.register_callback(
            self.on_physics_step, IsaacEvents.POST_PHYSICS_STEP
        )
        await omni.kit.app.get_app().next_update_async()

    def on_physics_step(self, step_size: float, context: object) -> None:
        """Physics step callback to control the Spot robot.

        Called on each physics simulation step to send movement commands to the robot.

        Args:
            step_size: The physics simulation step size.
            context: The simulation context.
        """
        if self._spot:
            self._spot.forward(step_size, self._base_command)


class TestSpotGPU(TestSpotCPU):
    """GPU-based test suite for the Spot quadruped robot policy examples.

    This test class extends TestSpotCPU to run all Spot robot policy tests on GPU using CUDA acceleration.
    It inherits comprehensive test coverage for robot spawning, movement commands, and turning behaviors
    while leveraging GPU compute for improved performance with tensor operations.

    The test suite validates:
    - Robot spawning and DOF configuration
    - Forward movement commands and position tracking
    - Turning commands and orientation changes
    - Physics simulation integration with GPU tensors

    All tests use CUDA device for tensor computations, making it suitable for performance testing
    and validation of GPU-accelerated robot policy implementations.
    """

    def get_device(self) -> object:
        """Return the device to use for tensors.

        Returns:
            The CUDA device for tensor operations.
        """
        return torch.device("cuda")
