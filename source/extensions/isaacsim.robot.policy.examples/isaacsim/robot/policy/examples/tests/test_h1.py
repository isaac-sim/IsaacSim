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

"""Unit tests for H1 humanoid robot policy examples in Isaac Sim."""

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
from isaacsim.robot.policy.examples.robots.h1 import H1FlatTerrainPolicy
from isaacsim.storage.native import get_assets_root_path
from pxr import UsdPhysics

torch = import_module("torch")


class TestH1ExampleExtension(omni.kit.test.AsyncTestCase):
    """Test case for the H1 humanoid robot policy examples.

    This class provides comprehensive testing for the H1FlatTerrainPolicy, validating robot spawning,
    movement commands, and basic locomotion behaviors. The tests ensure the H1 robot can be properly
    instantiated in the simulation environment and respond correctly to movement commands.

    The test suite includes verification of:
    - Robot spawning and articulation setup with proper degrees of freedom
    - Forward movement commands and position validation
    - Turning commands and orientation changes
    - Physics simulation integration and callback registration

    Tests run on CPU by default but can be overridden for GPU execution through device selection.
    """

    def get_device(self):
        """Return the device to use for tensors. Override in subclasses.

        Returns:
            The device to use for tensors.
        """
        return torch.device("cpu")

    async def setUp(self):
        """Set up the test environment with physics scene and ground plane."""
        await stage_utils.create_new_stage_async()
        # This needs to be set so that kit updates match physics updates
        self._physics_rate = 200

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
        await omni.kit.app.get_app().next_update_async()

    async def tearDown(self):
        """Clean up the test environment by stopping timeline and deregistering callbacks."""
        await omni.kit.app.get_app().next_update_async()
        self._timeline.stop()
        if self._physics_callback_id is not None:
            try:
                SimulationManager.deregister_callback(self._physics_callback_id)
            except Exception as e:
                print(f"Note: Could not deregister callback {self._physics_callback_id}: {e}")
            self._physics_callback_id = None

        while omni.usd.get_context().get_stage_loading_status()[2] > 0:
            print("tearDown, assets still loading, waiting to finish...")
            await asyncio.sleep(1.0)
        await omni.kit.app.get_app().next_update_async()

    async def test_h1_add(self):
        """Test spawning H1 robot and verify it has correct number of DOFs and valid physics setup."""
        await self.spawn_h1()
        await omni.kit.app.get_app().next_update_async()
        self.assertEqual(self._h1.robot.num_dofs, 19)

        # Verify root prim exists at spawn path
        root_prim = stage_utils.get_current_stage().GetPrimAtPath(self._prim_path)
        self.assertIsNotNone(root_prim, f"Robot root prim should exist at {self._prim_path}")
        self.assertTrue(root_prim.IsValid(), "Robot root prim should be valid")

        # Verify articulation root (may be nested under root for some USD assets) has ArticulationRootAPI
        articulation_root_path = self._h1.robot.paths[0]
        articulation_prim = stage_utils.get_current_stage().GetPrimAtPath(articulation_root_path)
        self.assertTrue(
            prim_utils.has_api(articulation_prim, UsdPhysics.ArticulationRootAPI),
            f"Articulation root prim at {articulation_root_path} should have ArticulationRootAPI",
        )

    async def test_robot_move_forward_command(self):
        """Test robot forward movement command and verify position change is within expected range."""
        await self.spawn_h1()
        await omni.kit.app.get_app().next_update_async()

        # Get current poses and convert to numpy arrays for efficient operations
        start_positions_wp, _ = self._h1.robot.get_world_poses()

        self.start_pos = start_positions_wp.numpy()[0]

        self._base_command = torch.tensor([1, 0, 0], dtype=torch.float32, device=self.get_device())

        # Simulate for 2 seconds (120 steps at 60 Hz default)
        for i in range(120):
            await omni.kit.app.get_app().next_update_async()

        current_positions_wp, _ = self._h1.robot.get_world_poses()

        self.current_pos = current_positions_wp.numpy()[0]

        delta = abs(self.current_pos[0] - self.start_pos[0])

        print(f"delta forward: {delta}")
        self.assertGreater(delta, 1.0)
        self.assertLess(delta, 2.0)

    async def test_robot_turn_command(self):
        """Test robot turn command and verify heading change is at least 90 degrees."""
        await self.spawn_h1()
        await omni.kit.app.get_app().next_update_async()

        # Get current poses and convert to numpy arrays for efficient operations
        _, start_orientations_wp = self._h1.robot.get_world_poses()

        self.start_orientation = start_orientations_wp.numpy()[0]

        self._base_command = torch.tensor([0, 0, 1], dtype=torch.float32, device=self.get_device())

        # Simulate for 2 seconds (120 steps at 60 Hz default)
        for _ in range(120):
            await omni.kit.app.get_app().next_update_async()

        _, current_orientations_wp = self._h1.robot.get_world_poses()

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
        self.assertGreater(heading_delta, 1.4)

    async def spawn_h1(self, name: str = "h1"):
        """Spawn H1 robot in the scene and initialize physics simulation.

        Args:
            name: Name for the robot prim in the scene.
        """
        self._prim_path = "/World/" + name

        self._h1 = H1FlatTerrainPolicy(prim_path=self._prim_path, position=[0, 0, 1.05])
        await omni.kit.app.get_app().next_update_async()
        self._timeline.play()
        await omni.kit.app.get_app().next_update_async()
        self._h1.initialize()
        await omni.kit.app.get_app().next_update_async()
        self._physics_callback_id = SimulationManager.register_callback(
            self.on_physics_step, IsaacEvents.POST_PHYSICS_STEP
        )
        await omni.kit.app.get_app().next_update_async()

    def on_physics_step(self, step_size: float, context: object) -> None:
        """Physics step callback that applies base command to the H1 robot.

        Args:
            step_size: Time step size for physics simulation.
            context: Physics simulation context.
        """
        if self._h1:
            self._h1.forward(step_size, self._base_command)


class TestH1GPU(TestH1ExampleExtension):
    """GPU-accelerated test suite for the H1 humanoid robot using PyTorch CUDA backend.

    This test class extends the base H1 robot test functionality to run on GPU using CUDA,
    providing accelerated tensor operations and physics computations. It inherits all test
    methods from TestH1ExampleExtension while configuring the robot to use GPU device for
    enhanced performance in simulation scenarios.

    The class automatically configures PyTorch tensors and robot operations to use CUDA
    device, enabling faster execution of robot control policies and physics simulations
    compared to CPU-based testing.
    """

    def get_device(self):
        """Return the device to use for tensors.

        Returns:
            The torch device to use for tensor operations.
        """
        return torch.device("cuda")
