# SPDX-FileCopyrightText: Copyright (c) 2024-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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

"""Tests for WheeledRobot experimental wrapper."""

import isaacsim.core.experimental.utils.app as app_utils
import isaacsim.core.experimental.utils.stage as stage_utils
import numpy as np
import omni.kit.test
from isaacsim.core.experimental.objects import GroundPlane
from isaacsim.core.simulation_manager import SimulationManager
from isaacsim.robot.experimental.wheeled_robots.robots.wheeled_robot import WheeledRobot
from isaacsim.storage.native import get_assets_root_path_async


class TestWheeledRobotValidation(omni.kit.test.AsyncTestCase):
    """Tests for WheeledRobot input validation."""

    async def test_requires_wheel_dof_names_or_indices(self) -> None:
        """Verify missing wheel DOF identifiers fail with a clear exception."""
        with self.assertRaises(ValueError):
            WheeledRobot("/World/MissingWheelConfig")


class TestWheeledRobot(omni.kit.test.AsyncTestCase):
    """Tests for the WheeledRobot experimental class."""

    async def setUp(self) -> None:
        """Set up test environment with a Jetbot wheeled robot."""
        assets_root_path = await get_assets_root_path_async()
        self._jetbot_usd = assets_root_path + "/Isaac/Robots/NVIDIA/Jetbot/jetbot.usd"
        await stage_utils.create_new_stage_async()
        stage_utils.set_stage_up_axis("Z")
        stage_utils.set_stage_units(meters_per_unit=1.0)
        GroundPlane("/World/GroundPlane")
        self._robot = WheeledRobot(
            "/World/Jetbot", wheel_dof_names=["left_wheel_joint", "right_wheel_joint"], usd_path=self._jetbot_usd
        )
        await app_utils.update_app_async()
        SimulationManager.setup_simulation(dt=1.0 / 60.0, device="cpu")
        app_utils.play()
        await app_utils.update_app_async()

    async def tearDown(self) -> None:
        """Stop simulation and wait for stage cleanup."""
        app_utils.stop()
        await app_utils.update_app_async()
        while omni.usd.get_context().get_stage_loading_status()[2] > 0:
            await app_utils.update_app_async()

    async def test_wheeled_robot(self) -> None:
        """Verify construction, DOF resolution, and wheel actions."""
        stage = omni.usd.get_context().get_stage()
        self.assertTrue(stage.GetPrimAtPath("/World/Jetbot").IsValid())

        indices = self._robot._resolve_wheel_dof_indices()
        self.assertEqual(len(indices), 2)
        self.assertIsInstance(indices[0], int)
        self.assertIs(self._robot._resolve_wheel_dof_indices(), indices)

        with self.assertRaises(ValueError):
            self._robot.apply_wheel_actions([1.0, 2.0, 3.0])

        # Jetbot wheel_radius=0.0335 m. At steady-state with
        # target 5.0 rad/s: v = 0.0335 * 5.0 = 0.1675 m/s, so 1 s => ~0.1675 m.
        self._robot.apply_wheel_actions([5.0, 5.0])
        await app_utils.update_app_async(steps=30)
        starting_pos = self._robot.get_world_poses()[0].numpy()[0].copy()
        await app_utils.update_app_async(steps=60)
        current_pos = self._robot.get_world_poses()[0].numpy()[0]
        actual_distance = np.linalg.norm(current_pos - starting_pos)
        expected_distance = 0.0335 * 5.0 * 1.0
        self.assertAlmostEqual(actual_distance, expected_distance, delta=0.05)
