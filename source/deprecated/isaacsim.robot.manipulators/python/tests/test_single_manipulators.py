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

"""Unit tests for single manipulator robots with various gripper configurations in Isaac Sim."""

import numpy as np
import omni.kit.test
from isaacsim.core.api import World
from isaacsim.core.utils.physics import simulate_async
from isaacsim.core.utils.stage import add_reference_to_stage, create_new_stage_async
from isaacsim.core.utils.types import ArticulationAction
from isaacsim.robot.manipulators import SingleManipulator
from isaacsim.robot.manipulators.grippers import ParallelGripper, SurfaceGripper
from isaacsim.storage.native import get_assets_root_path_async


class TestSingleManipulators(omni.kit.test.AsyncTestCase):
    """Test suite for SingleManipulator functionality with various gripper configurations.

    This test class validates the behavior of SingleManipulator robots with different gripper types,
    including surface grippers and parallel grippers. It tests initialization, configuration,
    and basic operations of manipulator systems in Isaac Sim.
    """

    async def setUp(self) -> None:
        """Set up the test environment by creating a new world, stage, and timeline interface."""
        World.clear_instance()
        await create_new_stage_async()
        self._my_world = World()
        await self._my_world.initialize_simulation_context_async()
        self._assets_root_path = await get_assets_root_path_async()
        self._my_world.scene.add_default_ground_plane()
        self._timeline = omni.timeline.get_timeline_interface()

    async def test_single_manipulators(self) -> None:
        """Test SingleManipulator creation and initialization with a UR10 robot and SurfaceGripper."""
        asset_path = self._assets_root_path + "/Isaac/Robots/UniversalRobots/ur10/ur10.usd"
        robot = add_reference_to_stage(usd_path=asset_path, prim_path="/World/UR10")
        robot.GetVariantSet("Gripper").SetVariantSelection("Short_Suction")
        gripper = SurfaceGripper(
            end_effector_prim_path="/World/UR10/ee_link", surface_gripper_path="/World/UR10/ee_link/SurfaceGripper"
        )
        ur10 = self._my_world.scene.add(
            SingleManipulator(
                prim_path="/World/UR10", name="my_ur10", end_effector_prim_path="/World/UR10/ee_link", gripper=gripper
            )
        )
        ur10.set_joints_default_state(
            positions=np.array([-np.pi / 2, -np.pi / 2, -np.pi / 2, -np.pi / 2, np.pi / 2, 0])
        )
        ur10.gripper.set_default_state(opened=True)
        await self._my_world.reset_async()
        self.assertFalse(ur10.gripper is None)

    async def test_parallel_gripper(self) -> None:
        """Test ParallelGripper functionality with a UR10e robot and Robotiq gripper."""
        asset_path = self._assets_root_path + "/Isaac/Robots/UniversalRobots/ur10e/ur10e.usd"
        robot = add_reference_to_stage(usd_path=asset_path, prim_path="/World/ur10e")
        robot.GetVariantSet("Gripper").SetVariantSelection("Robotiq_2f_140")
        await omni.kit.app.get_app().next_update_async()

        gripper = ParallelGripper(
            # We chose the following values while inspecting the articulation
            end_effector_prim_path="/World/ur10e/ee_link/robotiq_base_link",
            joint_prim_names=["finger_joint"],
            joint_opened_positions=np.array([0]),
            joint_closed_positions=np.array([45]),
            action_deltas=np.array([-45]),
            use_mimic_joints=True,
        )
        await omni.kit.app.get_app().next_update_async()

        ur10 = self._my_world.scene.add(
            SingleManipulator(
                prim_path="/World/ur10e",
                name="my_ur10e",
                end_effector_prim_path="/World/ur10e/ee_link/robotiq_base_link",
                gripper=gripper,
            )
        )
        await omni.kit.app.get_app().next_update_async()

        self._timeline.play()
        await omni.kit.app.get_app().next_update_async()

        ur10.initialize()
        await omni.kit.app.get_app().next_update_async()

        ur10.gripper.apply_action(ArticulationAction(joint_positions=[45]))

        await simulate_async(0.4)

        self.assertAlmostEqual(ur10.gripper.get_joint_positions()[0], 45 / 180 * np.pi, delta=1e-2)

    async def tearDown(self) -> None:
        """Clean up the test environment by stopping the timeline."""
        await omni.kit.app.get_app().next_update_async()
        self._timeline.stop()
        await omni.kit.app.get_app().next_update_async()
