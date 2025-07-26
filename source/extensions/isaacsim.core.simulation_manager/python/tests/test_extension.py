# SPDX-FileCopyrightText: Copyright (c) 2024-2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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
"""
The Kit extension system tests for Python has additional wrapping 
to make test auto-discoverable add support for async/await tests.
The easiest way to set up the test class is to have it derive from
the omni.kit.test.AsyncTestCase class that implements them.

Visit the next link for more details:
  https://docs.omniverse.nvidia.com/kit/docs/kit-manual/latest/guide/testing_exts_python.html
"""

import os
import unittest

import omni.kit.test
import omni.timeline
from isaacsim.core.simulation_manager import IsaacEvents, SimulationManager
from isaacsim.core.utils.stage import create_new_stage_async, update_stage_async


class TestExtension(omni.kit.test.AsyncTestCase):
    async def setUp(self):
        """Method called to prepare the test fixture"""
        super().setUp()
        # ---------------
        # Do custom setUp
        # ---------------

    async def tearDown(self):
        """Method called immediately after the test method has been called"""
        # ------------------
        # Do custom tearDown
        # ------------------
        super().tearDown()

    # --------------------------------------------------------------------
    async def test_extension(self):
        # Kit extension system test for Python is based on the unittest module.
        # Visit https://docs.python.org/3/library/unittest.html to see the
        # available assert methods to check for and report failures.
        print("Test case: test_extension")
        await create_new_stage_async()
        self._callbacks = []
        self._callbacks.append(
            SimulationManager.register_callback(lambda x: print("working"), event=IsaacEvents.PHYSICS_READY)
        )
        self._callbacks.append(
            SimulationManager.register_callback(lambda x: print("working"), event=IsaacEvents.POST_RESET)
        )
        timeline = omni.timeline.get_timeline_interface()
        timeline.play()
        await update_stage_async()
        await update_stage_async()
        for callback_id in self._callbacks:
            SimulationManager.deregister_callback(callback_id)

    async def test_physics_callbacks(self):
        await create_new_stage_async()
        global var
        var = 1

        def add_one(dt):
            global var
            var += 1

        def multiply_two(dt):
            global var
            var *= 2

        self._callbacks = []
        self._callbacks.append(SimulationManager.register_callback(add_one, event=IsaacEvents.PRE_PHYSICS_STEP))
        self._callbacks.append(
            SimulationManager.register_callback(add_one, event=IsaacEvents.POST_PHYSICS_STEP, order=2)
        )
        self._callbacks.append(
            SimulationManager.register_callback(multiply_two, event=IsaacEvents.POST_PHYSICS_STEP, order=0)
        )
        timeline = omni.timeline.get_timeline_interface()
        timeline.play()
        timeline.commit()
        SimulationManager.step(render=False)
        SimulationManager.step(render=False)
        for callback_id in self._callbacks:
            SimulationManager.deregister_callback(callback_id)
        self.assertEqual(var, 13)

    # TODO: ETM will always have 2 steps of simulation at start and not increment.
    @unittest.skipIf(os.getenv("ETM_ACTIVE"), "skipped in ETM. Physics steps are not handled the same way in ETM")
    async def test_simulation_manager_interface(self):
        timeline = omni.timeline.get_timeline_interface()
        await create_new_stage_async()

        self.assertEqual(SimulationManager.get_num_physics_steps(), 0)
        self.assertEqual(SimulationManager.get_simulation_time(), 0.0)
        self.assertEqual(SimulationManager.is_simulating(), False)
        self.assertEqual(SimulationManager.is_paused(), False)
        self.assertEqual(SimulationManager.get_physics_dt(), 1.0 / 60.0)

        timeline.play()
        await update_stage_async()
        self.assertEqual(SimulationManager.is_simulating(), True)
        self.assertEqual(SimulationManager.is_paused(), False)
        # The number of steps is offset by 2 because we do two steps of simulation to warm up
        self.assertEqual(SimulationManager.get_num_physics_steps(), 3)
        self.assertAlmostEqual(SimulationManager.get_simulation_time(), 1.0 / 60.0 * 3)

        timeline.pause()
        await update_stage_async()
        self.assertEqual(SimulationManager.is_simulating(), True)
        self.assertEqual(SimulationManager.is_paused(), True)
        self.assertEqual(SimulationManager.get_num_physics_steps(), 3)
        self.assertAlmostEqual(SimulationManager.get_simulation_time(), 1.0 / 60.0 * 3)

        timeline.play()
        await update_stage_async()
        await update_stage_async()
        self.assertEqual(SimulationManager.is_simulating(), True)
        self.assertEqual(SimulationManager.is_paused(), False)
        self.assertEqual(SimulationManager.get_num_physics_steps(), 5)
        self.assertAlmostEqual(SimulationManager.get_simulation_time(), 1.0 / 60.0 * 5)

        timeline.stop()
        await update_stage_async()
        self.assertEqual(SimulationManager.is_simulating(), False)
        self.assertEqual(SimulationManager.is_paused(), False)
        self.assertEqual(SimulationManager.get_num_physics_steps(), 0)
        self.assertEqual(SimulationManager.get_simulation_time(), 0)
