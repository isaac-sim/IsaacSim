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

"""Tests for Franka robot policy examples with both CPU and GPU simulation environments."""

import asyncio

import isaacsim.core.experimental.utils.prim as prim_utils
import isaacsim.core.experimental.utils.stage as stage_utils

# NOTE:
#   omni.kit.test - std python's unittest module with additional wrapping to add suport for async/await tests
#   For most things refer to unittest docs: https://docs.python.org/3/library/unittest.html
import omni.kit.test
import omni.timeline
from isaacsim.core.deprecation_manager import import_module
from isaacsim.core.experimental.prims import Articulation
from isaacsim.core.simulation_manager import SimulationManager
from isaacsim.core.simulation_manager.impl.isaac_events import IsaacEvents
from isaacsim.robot.policy.examples.robots.franka import FrankaOpenDrawerPolicy
from isaacsim.storage.native import get_assets_root_path
from pxr import UsdPhysics

torch = import_module("torch")


class TestFrankaExampleExtension(omni.kit.test.AsyncTestCase):
    """Test suite for the Franka robot policy example extension.

    This test class validates the functionality of the Franka robot drawer opening policy by setting up
    a simulation environment with a Franka robot and cabinet, then verifying the robot can successfully
    open a drawer. The tests ensure proper robot initialization, cabinet interaction, and physics
    simulation behavior.

    The test suite creates a physics scene with a ground plane and Sektion cabinet, spawns a Franka
    robot with the FrankaOpenDrawerPolicy, and validates that the robot can open the cabinet drawer
    to a specified threshold. It uses CPU-based simulation by default with a 200Hz physics rate.
    """

    def get_device(self):
        """Return the device to use for tensors. Override in subclasses.

        Returns:
            The torch device for tensor operations.
        """
        return torch.device("cpu")

    def get_physics_rate(self):
        """Return the physics rate for the drawer opening test.

        Returns:
            The physics rate in Hz.
        """
        return 200  # CPU needs a lower rate

    async def setUp(self):
        """Set up the test environment with physics scene, ground plane, and cabinet."""
        await stage_utils.create_new_stage_async()
        # This needs to be set so that kit updates match physics updates
        self._physics_rate = self.get_physics_rate()

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

        cabinet_prim_path = "/World/cabinet"
        cabinet_usd_path = get_assets_root_path() + "/Isaac/Props/Sektion_Cabinet/sektion_cabinet_instanceable.usd"

        cabinet_position = [0.8, 0.0, 0.4]
        cabinet_orientation = [0.0, 0.0, 0.0, 1.0]

        stage_utils.add_reference_to_stage(cabinet_usd_path, cabinet_prim_path)

        self.cabinet = Articulation(cabinet_prim_path, positions=cabinet_position, orientations=cabinet_orientation)

        self._timeline = omni.timeline.get_timeline_interface()
        await omni.kit.app.get_app().next_update_async()

    async def tearDown(self):
        """Clean up test environment by stopping timeline and deregistering callbacks."""
        await omni.kit.app.get_app().next_update_async()
        self._timeline.stop()
        SimulationManager.deregister_callback(self._physics_callback_id)

        while omni.usd.get_context().get_stage_loading_status()[2] > 0:
            print("tearDown, assets still loading, waiting to finish...")
            await asyncio.sleep(1.0)
        await omni.kit.app.get_app().next_update_async()

    async def test_franka_add(self):
        """Test adding a Franka robot to the scene and verify its properties."""
        await self.spawn_franka()
        await omni.kit.app.get_app().next_update_async()
        await omni.kit.app.get_app().next_update_async()
        self.assertEqual(self._franka.robot.num_dofs, 9)

        # Verify robot prim exists
        robot_prim = stage_utils.get_current_stage().GetPrimAtPath("/World/franka")
        self.assertIsNotNone(robot_prim, "Robot prim should exist in stage at /World/franka")
        self.assertTrue(robot_prim.IsValid(), "Robot prim should be valid")
        self.assertTrue(
            prim_utils.has_api(robot_prim, UsdPhysics.ArticulationRootAPI),
            "Robot base prim should have ArticulationRootAPI",
        )

        # Verify cabinet prim exists
        cabinet_prim = stage_utils.get_current_stage().GetPrimAtPath("/World/cabinet")
        self.assertIsNotNone(cabinet_prim, "Cabinet prim should exist in stage at /World/cabinet")
        self.assertTrue(cabinet_prim.IsValid(), "Cabinet prim should be valid")

    async def test_franka_open_drawer(self):
        """Test the Franka robot's ability to open a drawer using the FrankaOpenDrawerPolicy."""
        await self.spawn_franka()
        await omni.kit.app.get_app().next_update_async()
        max_drawer_opening = 0
        drawer_link_idx = self.cabinet.get_dof_indices("drawer_top_joint")
        min_drawer_opening = 0.25

        def is_drawer_open():
            nonlocal max_drawer_opening
            drawer_joint_position = self.cabinet.get_dof_positions(indices=[0], dof_indices=drawer_link_idx)
            drawer_position_numpy = drawer_joint_position.numpy()
            drawer_position_scalar = float(drawer_position_numpy[0][0])
            if drawer_position_scalar > max_drawer_opening:
                max_drawer_opening = drawer_position_scalar
            return max_drawer_opening > min_drawer_opening

        condition_met = await self.simulate_until_condition(is_drawer_open, max_frames=300)

        # drawer is closed at 0.0 and open at 0.4, the robot should be able to open the drawer
        # to at least 0.3
        self.assertTrue(condition_met, f"Expected drawer to open past {min_drawer_opening}, got {max_drawer_opening}")

    async def simulate_until_condition(self, condition_func, max_frames=180):
        """Simulate until condition is met or maximum frames reached."""
        frames_run = 0
        while frames_run < max_frames:
            await omni.kit.app.get_app().next_update_async()
            frames_run += 1
            if condition_func():
                return True
        return False

    async def spawn_franka(self, name: str = "franka", add_physics_callback: bool = True):
        """Spawn a Franka robot with drawer opening policy in the simulation.

        Args:
            name: Name for the robot prim in the stage.
            add_physics_callback: Whether to register a physics step callback for the robot.
        """
        self._prim_path = "/World/" + name

        self._franka = FrankaOpenDrawerPolicy(prim_path=self._prim_path, position=[0, 0, 0], cabinet=self.cabinet)
        self._timeline.play()
        await omni.kit.app.get_app().next_update_async()
        self._franka.initialize()
        self._franka.post_reset()
        self._franka.robot.set_dof_positions(self._franka.default_pos.cpu().numpy().tolist())
        await omni.kit.app.get_app().next_update_async()

        if add_physics_callback:
            self._physics_callback_id = SimulationManager.register_callback(
                self.on_physics_step, IsaacEvents.POST_PHYSICS_STEP
            )
        await omni.kit.app.get_app().next_update_async()

    def on_physics_step(self, step_size: float, context: object) -> None:
        """Physics step callback that advances the Franka robot's policy.

        Args:
            step_size: The physics time step size.
            context: The simulation context.
        """
        if self._franka:
            self._franka.forward(step_size)


class TestFrankaGPU(TestFrankaExampleExtension):
    """GPU-accelerated test class for Franka robot policy examples.

    This class extends the base Franka test functionality to run on CUDA-enabled GPUs,
    providing accelerated physics simulation and tensor operations. It configures the
    test environment to use CUDA devices and higher physics rates optimized for GPU
    performance.

    The class automatically sets up a physics simulation environment with a Franka robot
    and cabinet, then runs validation tests for robot spawning and drawer opening tasks.
    GPU acceleration enables faster convergence and higher simulation rates compared to
    CPU-based testing.
    """

    def get_device(self):
        """Return the device to use for tensors.

        Returns:
            The torch device to use for tensor operations.
        """
        return torch.device("cuda")

    def get_physics_rate(self):
        """Return the physics rate for the drawer opening test.

        Returns:
            The physics simulation rate in Hz.
        """
        return 400  # GPU converges faster
