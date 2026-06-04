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

"""Common test utilities."""

import asyncio
import gc
import time
from typing import Any

import omni
from isaacsim.core.api import World
from isaacsim.core.deprecation_manager import import_module
from isaacsim.core.utils.stage import update_stage_async

torch = import_module("torch")


class CoreTestCase(omni.kit.test.AsyncTestCase):
    """Base test class that automatically times all test methods.

    This class extends omni.kit.test.AsyncTestCase to automatically print
    the execution time of each test method. All test classes should inherit
    from this instead of omni.kit.test.AsyncTestCase directly.
    """

    async def setUp(self) -> None:
        """Set up test timing before each test method."""
        self._test_start_time = time.time()
        self._timeline = omni.timeline.get_timeline_interface()

    async def tearDown(self) -> None:
        """Tear down test environment."""
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
    """Test properties."""

    async def scalar_prop_test(self, getFunc: Any, setFunc: Any, set_value: Any = 0.2, is_stopped: Any = False) -> None:
        """Scalar prop test.

        Args:
            getFunc: Getter function to retrieve the current value.
            setFunc: Setter function to set the value.
            set_value: Value to set and verify.
            is_stopped: Whether the simulation is stopped.
        """
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

    async def bool_prop_test(
        self,
        getFunc: Any,
        setFunc: Any,
        set_value_1: Any = False,
        set_value_2: Any = True,
        is_stopped: Any = False,
    ) -> None:
        """Bool prop test.

        Args:
            getFunc: Getter function to retrieve the current value.
            setFunc: Setter function to set the value.
            set_value_1: First value to set and verify.
            set_value_2: Second value to set and verify.
            is_stopped: Whether the simulation is stopped.
        """
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

    async def int_prop_test(
        self,
        getFunc: Any,
        setFunc: Any,
        set_value: Any = 27,
        set_value_2: Any = 28,
        is_stopped: Any = False,
    ) -> None:
        """Int prop test.

        Args:
            getFunc: Getter function to retrieve the current value.
            setFunc: Setter function to set the value.
            set_value: First value to set and verify.
            set_value_2: Second value to set and verify.
            is_stopped: Whether the simulation is stopped.
        """
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
        getFunc: Any,
        setFunc: Any,
        set_value_1: Any = torch.Tensor([10, 12, 18]),
        set_value_2: Any = torch.Tensor([100, 102, 120]),
        is_stopped: Any = False,
    ) -> None:
        """Vector prop test.

        Args:
            getFunc: Getter function to retrieve the current value.
            setFunc: Setter function to set the value.
            set_value_1: First vector value to set and verify.
            set_value_2: Second vector value to set and verify.
            is_stopped: Whether the simulation is stopped.
        """
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
