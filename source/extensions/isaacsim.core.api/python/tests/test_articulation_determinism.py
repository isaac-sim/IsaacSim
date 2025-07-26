# SPDX-FileCopyrightText: Copyright (c) 2021-2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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

import asyncio

import carb
import numpy as np
import omni.kit.test
from isaacsim.core.api import SimulationContext, World
from isaacsim.core.api.robots import Robot
from isaacsim.core.utils.stage import add_reference_to_stage, open_stage_async, update_stage_async
from isaacsim.core.utils.types import ArticulationAction
from isaacsim.storage.native import get_assets_root_path_async

from .common import CoreTestCase


class TestArticulationDeterminism(CoreTestCase):

    # Before running each test
    async def setUp(self):
        await super().setUp()
        World.clear_instance()
        self._timeline = omni.timeline.get_timeline_interface()

        self._assets_root_path = await get_assets_root_path_async()
        if self._assets_root_path is None:
            carb.log_error("Could not find Isaac Sim assets folder")
            return

        pass

    async def tearDown(self):
        await super().tearDown()
        pass

    async def test_inconsistent_result(self):
        frames_to_converge = np.empty(5)
        for i in range(5):
            num_frames = await self._test_franka_slow_convergence()
            frames_to_converge[i] = num_frames

        # Takes the same number of frames to converge every time
        print(f"Over 5 trials, the Franka converged to target in {frames_to_converge} frames.")
        self.assertTrue(
            np.unique(frames_to_converge).shape[0] == 1,
            f"Non-deterministic test converged in varying number of frames: {frames_to_converge}",
        )

        # On the develop branch, this test always takes 31 frames to converge
        print(f"frames_to_converge[0] = {frames_to_converge[0]}")
        self.assertEqual(frames_to_converge[0], 27, "Took a different number of frames to converge!")

    async def _test_franka_slow_convergence(self):
        World.clear_instance()
        (result, error) = await open_stage_async(
            self._assets_root_path + "/Isaac/Robots/FrankaRobotics/FrankaPanda/franka.usd"
        )
        carb.settings.get_settings().set_bool("/app/runLoops/main/rateLimitEnabled", True)
        carb.settings.get_settings().set_int("/app/runLoops/main/rateLimitFrequency", int(60))
        carb.settings.get_settings().set_int("/persistent/simulation/minFrameRate", int(60))
        omni.usd.get_context().get_stage().SetTimeCodesPerSecond(60)
        robot_prim_path = "/panda"
        my_world = World(device="cpu")  # Create a new default world to reset any physics settings.
        await my_world.initialize_simulation_context_async()
        # Start Simulation and wait
        self._timeline.play()
        await update_stage_async()

        self._robot = Robot(robot_prim_path)
        self._robot.initialize()
        self._robot.get_articulation_controller().set_gains(1e4 * np.ones(9), 1e3 * np.ones(9))
        self._robot.set_solver_position_iteration_count(64)
        self._robot.set_solver_velocity_iteration_count(64)
        self._robot.post_reset()
        await update_stage_async()

        timeout = 200

        action = ArticulationAction(
            joint_positions=np.array(
                [
                    -0.40236897393760085,
                    -0.44815597748391767,
                    -0.16028112816211953,
                    -2.4554393933564986,
                    -0.34608791253975374,
                    2.9291361940824485,
                    0.4814803907662416,
                    0.0,
                    0.0,
                ]
            )
        )

        self._robot.get_articulation_controller().apply_action(action)

        for i in range(timeout):
            await update_stage_async()
            diff = self._robot.get_joint_positions() - action.joint_positions
            if np.linalg.norm(diff) < 0.01:
                return i

        return timeout
