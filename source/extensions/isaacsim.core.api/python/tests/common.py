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
import asyncio
import gc
import time

import omni
import torch
from isaacsim.core.api import World
from isaacsim.core.utils.stage import update_stage_async


class CoreTestCase(omni.kit.test.AsyncTestCase):
    """Base test class that automatically times all test methods.

    This class extends omni.kit.test.AsyncTestCase to automatically print
    the execution time of each test method. All test classes should inherit
    from this instead of omni.kit.test.AsyncTestCase directly.
    """

    async def setUp(self):
        """Set up test timing before each test method."""
        self._test_start_time = time.time()
        self._timeline = omni.timeline.get_timeline_interface()

    async def tearDown(self):
        self._timeline.stop()
        await omni.kit.app.get_app().next_update_async()
        """Print test execution time after each test method."""
        while omni.usd.get_context().get_stage_loading_status()[2] > 0:
            print("tearDown, assets still loading, waiting to finish...")
            await asyncio.sleep(1.0)
        await update_stage_async()
        World.clear_instance()

        test_duration = time.time() - self._test_start_time
        test_name = self._testMethodName
        print(f"\n[TEST TIMING] {test_name}: {test_duration:.3f} seconds")
        gc.collect()


class TestProperties:
    async def scalar_prop_test(self, getFunc, setFunc, set_value=0.2, is_stopped=False):
        await self.my_world.reset_async()
        if is_stopped:
            await self.my_world.stop_async()
        setFunc(set_value)
        cur_value = getFunc()
        self.assertTrue(
            torch.isclose(cur_value, torch.tensor(set_value), atol=1e-5), f"getFunc={getFunc}\nsetFunc={setFunc}"
        )
        self.my_world.step_async(0)
        await omni.kit.app.get_app().next_update_async()
        cur_value = getFunc()
        self.assertTrue(torch.isclose(cur_value, torch.tensor(set_value)), f"getFunc={getFunc}\nsetFunc={setFunc}")

    async def bool_prop_test(self, getFunc, setFunc, set_value_1=False, set_value_2=True, is_stopped=False):
        await self.my_world.reset_async()
        if is_stopped:
            await self.my_world.stop_async()
        setFunc(set_value_1)
        cur_value = getFunc()
        self.assertTrue(cur_value == set_value_1, f"getFunc={getFunc}\nsetFunc={setFunc}")
        setFunc(set_value_2)
        cur_value = getFunc()
        self.my_world.step_async(0)
        await omni.kit.app.get_app().next_update_async()
        cur_value = getFunc()
        self.assertTrue(cur_value == set_value_2, f"getFunc={getFunc}\nsetFunc={setFunc}")

    async def int_prop_test(self, getFunc, setFunc, set_value=27, set_value_2=28, is_stopped=False):
        await self.my_world.reset_async()
        if is_stopped:
            await self.my_world.stop_async()
        setFunc(set_value)
        cur_value = getFunc()
        self.assertTrue(cur_value == set_value, f"getFunc={getFunc}\nsetFunc={setFunc}")
        setFunc(set_value_2)
        cur_value = getFunc()
        self.my_world.step_async(0)
        await omni.kit.app.get_app().next_update_async()
        cur_value = getFunc()
        self.assertTrue(cur_value == set_value_2, f"getFunc={getFunc}\nsetFunc={setFunc}")

    async def vector_prop_test(
        self,
        getFunc,
        setFunc,
        set_value_1=torch.Tensor([10, 12, 18]),
        set_value_2=torch.Tensor([100, 102, 120]),
        is_stopped=False,
    ):
        await self.my_world.reset_async()
        if is_stopped:
            await self.my_world.stop_async()
        setFunc(set_value_1)
        cur_value = getFunc()
        self.assertTrue(
            torch.isclose(set_value_1, torch.Tensor(cur_value)).all(), f"getFunc={getFunc}\nsetFunc={setFunc}"
        )
        setFunc(set_value_2)
        cur_value = getFunc()
        self.my_world.step_async()
        await omni.kit.app.get_app().next_update_async()
        cur_value = getFunc()
        self.assertTrue(
            torch.isclose(set_value_2, torch.Tensor(cur_value)).all(), f"getFunc={getFunc}\nsetFunc={setFunc}"
        )
