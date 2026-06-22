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

"""Verifies core simulation OmniGraph nodes report simulation time and physics step counts. Covers timeline-driven simulation updates through the SimulationManager."""

import carb
import isaacsim.core.experimental.utils.stage as stage_utils
import omni.kit.test
from isaacsim.core.simulation_manager import SimulationManager
from isaacsim.storage.native import get_assets_root_path_async


class TestCoreNodes(omni.kit.test.AsyncTestCase):
    """Verify core simulation timing nodes on a loaded Franka stage."""

    async def setUp(self) -> None:
        """Set up  test environment, to be torn down when done."""
        self._timeline = omni.timeline.get_timeline_interface()
        # add franka robot for test
        assets_root_path = await get_assets_root_path_async()
        if assets_root_path is None:
            carb.log_error("Could not find Isaac Sim assets folder")
            return
        await stage_utils.open_stage_async(assets_root_path + "/Isaac/Robots/FrankaRobotics/FrankaPanda/franka.usd")

    # ----------------------------------------------------------------------
    async def tearDown(self) -> None:
        """Get rid of temporary data used by the test."""
        await omni.kit.stage_templates.new_stage_async()

    # ----------------------------------------------------------------------
    async def test_simulation_time(self) -> None:
        """Verify IsaacReadSimulationTime outputs the SimulationManager simulation time."""
        await omni.kit.app.get_app().next_update_async()
        a = SimulationManager.get_simulation_time()
        b = SimulationManager._simulation_manager_interface.get_simulation_time_monotonic()
        c = SimulationManager._simulation_manager_interface.get_system_time()
        await omni.kit.app.get_app().next_update_async()
        a = SimulationManager._simulation_manager_interface.get_simulation_time_at_time((0, 1))
        b = SimulationManager._simulation_manager_interface.get_simulation_time_monotonic_at_time((0, 1))
        c = SimulationManager._simulation_manager_interface.get_system_time_at_time((0, 1))
        self._timeline.play()
        await omni.kit.app.get_app().next_update_async()
        a = SimulationManager.get_simulation_time()
        b = SimulationManager._simulation_manager_interface.get_simulation_time_monotonic()
        c = SimulationManager._simulation_manager_interface.get_system_time()
        a = SimulationManager._simulation_manager_interface.get_simulation_time_at_time((0, 0))
        b = SimulationManager._simulation_manager_interface.get_simulation_time_monotonic_at_time((0, 0))
        c = SimulationManager._simulation_manager_interface.get_system_time_at_time((0, 0))

    # ----------------------------------------------------------------------
    async def test_physics_num_steps(self) -> None:
        """Verify IsaacReadSimulationTime outputs the current physics step count."""
        steps = SimulationManager.get_num_physics_steps()
        self.assertEqual(steps, 0)
        self._timeline.play()
        await omni.kit.app.get_app().next_update_async()
        steps = SimulationManager.get_num_physics_steps()
        self.assertEqual(steps, 3)
        await omni.kit.app.get_app().next_update_async()
        steps = SimulationManager.get_num_physics_steps()
        self.assertEqual(steps, 4)
        self._timeline.stop()
        await omni.kit.app.get_app().next_update_async()
        steps = SimulationManager.get_num_physics_steps()
        self.assertEqual(steps, 0)
