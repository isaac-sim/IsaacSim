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

"""Test module for validating Anymal quadruped robot policy functionality on both CPU and GPU devices."""

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
from isaacsim.robot.policy.examples.robots.anymal import AnymalFlatTerrainPolicy
from isaacsim.storage.native import get_assets_root_path
from pxr import UsdPhysics

torch = import_module("torch")


class TestAnymalCPU(omni.kit.test.AsyncTestCase):
    """Test class for validating Anymal robot functionality on CPU.

    This test class provides comprehensive validation of the Anymal robot's basic operations including
    spawning, movement commands, and physics integration when running on CPU devices. It inherits from
    omni.kit.test.AsyncTestCase to support asynchronous testing patterns required for Isaac Sim operations.

    The test suite validates:
        - Robot spawning and proper USD stage integration
        - ArticulationRootAPI configuration on the robot base
        - Forward movement commands and displacement verification
        - Rotational commands and heading change validation
        - Physics callback registration and simulation stepping

    Each test method sets up a clean simulation environment with a physics scene, ground plane, and
    the AnymalFlatTerrainPolicy robot instance. The tests use torch tensors on CPU for command inputs
    and verify robot behavior through position and orientation changes over simulation steps.
    """

    def get_device(self):
        """Return the device to use for tensors. Override in subclasses.

        Returns:
            The device to use for tensors.
        """
        return torch.device("cpu")

    async def setUp(self):
        """Set up the test environment with physics scene, simulation manager, and ground plane."""
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
        """Clean up the test environment by stopping timeline and deregistering physics callbacks."""
        await omni.kit.app.get_app().next_update_async()
        self._timeline.stop()
        SimulationManager.deregister_callback(self._physics_callback_id)
        while omni.usd.get_context().get_stage_loading_status()[2] > 0:
            print("tearDown, assets still loading, waiting to finish...")
            await asyncio.sleep(1.0)
        await omni.kit.app.get_app().next_update_async()

    async def test_anymal_add(self):
        """Test spawning an Anymal robot and verifying its DOFs and stage prims."""
        await self.spawn_anymal()
        await omni.kit.app.get_app().next_update_async()

        self.assertEqual(self._anymal.robot.num_dofs, 12)

        root_prim = stage_utils.get_current_stage().GetPrimAtPath(self._prim_path)
        self.assertIsNotNone(root_prim, f"Robot root prim should exist at {self._prim_path}")
        self.assertTrue(root_prim.IsValid(), "Robot root prim should be valid")

        articulation_root_path = self._anymal.robot.paths[0]
        articulation_prim = stage_utils.get_current_stage().GetPrimAtPath(articulation_root_path)
        self.assertTrue(
            prim_utils.has_api(articulation_prim, UsdPhysics.ArticulationRootAPI),
            f"Articulation root prim at {articulation_root_path} should have ArticulationRootAPI",
        )

    async def test_robot_move_forward_command(self):
        """Test robot forward movement by sending a forward command and verifying position change."""
        await self.spawn_anymal()
        await omni.kit.app.get_app().next_update_async()

        # Get current poses and convert to numpy arrays for efficient operations
        start_positions_wp, _ = self._anymal.robot.get_world_poses()

        self.start_pos = start_positions_wp.numpy()[0]

        self._base_command = torch.tensor([1, 0, 0], dtype=torch.float32, device=self.get_device())

        for _ in range(120):
            await omni.kit.app.get_app().next_update_async()

        # Get current poses and convert to numpy arrays for efficient operations
        current_positions_wp, _ = self._anymal.robot.get_world_poses()

        self.current_pos = current_positions_wp.numpy()[0]

        delta = abs(self.current_pos[0] - self.start_pos[0])
        self.assertGreater(delta, 0.4)
        self.assertLess(delta, 2.0)

    async def test_robot_turn_command(self):
        """Test robot turning by sending a turn command and verifying orientation change."""
        await self.spawn_anymal()
        await omni.kit.app.get_app().next_update_async()

        # Get current poses and convert to torch tensors for efficient operations
        _, start_orientations_wp = self._anymal.robot.get_world_poses()

        self.start_orientation = start_orientations_wp.numpy()[0]

        self._base_command = torch.tensor([0, 0, 1], dtype=torch.float32, device=self.get_device())
        # Simulate for 2 seconds (120 steps at 60 Hz default)
        for _ in range(120):
            await omni.kit.app.get_app().next_update_async()

        _, current_orientations_wp = self._anymal.robot.get_world_poses()

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

    async def spawn_anymal(self, name: str = "anymal"):
        """Spawn an Anymal robot in the simulation environment.

        Args:
            name: Name of the robot prim to create.
        """
        self._prim_path = "/World/" + name

        self._anymal = AnymalFlatTerrainPolicy(prim_path=self._prim_path, position=[0, 0, 0.60])
        self._timeline.play()
        await omni.kit.app.get_app().next_update_async()

        self._anymal.initialize()
        await omni.kit.app.get_app().next_update_async()

        self._physics_callback_id = SimulationManager.register_callback(
            self.on_physics_step, IsaacEvents.POST_PHYSICS_STEP
        )
        await omni.kit.app.get_app().next_update_async()

    def on_physics_step(self, step_size: float, context: object) -> None:
        """Physics step callback that applies base commands to the Anymal robot.

        Args:
            step_size: Time step size for physics simulation.
            context: Simulation context information.
        """
        if self._anymal:
            self._anymal.forward(step_size, self._base_command)


class TestAnymalGPU(TestAnymalCPU):
    """GPU-based test suite for the Anymal quadruped robot policy.

    This test class extends TestAnymalCPU to run all Anymal robot tests on GPU hardware using CUDA tensors.
    It validates robot spawning, movement commands, and turning behaviors with GPU-accelerated computation for
    performance testing and GPU-specific functionality verification.

    The test suite includes:
    - Robot spawning and initialization validation
    - Forward movement command testing with position delta verification
    - Turning command testing with orientation change validation
    - ArticulationRootAPI and prim structure verification

    All tensor operations and physics simulations run on CUDA device, making it suitable for testing
    GPU-accelerated robot control policies and simulation performance.
    """

    def get_device(self):
        """Return the device to use for tensors.

        Returns:
            The GPU device for tensor operations.
        """
        return torch.device("cuda")
