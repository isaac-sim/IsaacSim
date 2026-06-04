# SPDX-FileCopyrightText: Copyright (c) 2025-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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

"""Provides a base test class that automatically times test method execution for performance monitoring."""

import gc
import time

import carb
import omni.kit.test
from isaacsim.storage.native import get_assets_root_path


class TimedAsyncTestCase(omni.kit.test.AsyncTestCase):
    """Base test class that automatically times all test methods.

    This class extends omni.kit.test.AsyncTestCase to automatically measure and print
    the execution time of each test method. The timing information is displayed after
    each test completes, helping with performance monitoring and optimization.

    All test classes should inherit from this class instead of omni.kit.test.AsyncTestCase
    directly to benefit from automatic test timing functionality.

    Example:

    .. code-block:: python

        >>> import asyncio
        >>> from isaacsim.test.utils.timed_async_test import TimedAsyncTestCase
        >>>
        >>> class MyTestCase(TimedAsyncTestCase):
        ...     async def test_example(self):
        ...         await asyncio.sleep(0.1)
        ...         self.assertTrue(True)
        >>>
        >>> # When running the test, output will include:
        >>> # [TEST TIMING] test_example: 0.100 seconds
    """

    async def setUp(self) -> None:
        """Set up test timing before each test method.

        This method is called before each test method execution to record the start time.
        It first calls the parent setUp method to ensure proper test initialization,
        then records the current timestamp in _test_start_time, which will be used
        later in tearDown to calculate the test execution duration.

        This order ensures we measure only the test method execution time, excluding
        any framework setup overhead.
        """
        super().setUp()
        self._test_start_time = time.time()
        # In order to run tests faster, we assume the asset server is available and don't try connecting to it for each test
        self._assets_root_path = get_assets_root_path(skip_check=False)
        if self._assets_root_path is None:
            carb.log_error("Could not find Isaac Sim assets folder")
            return

    async def tearDown(self) -> None:
        """Clean up and display test timing after each test method.

        This method is called after each test method execution to calculate and display
        the test execution time. It computes the duration by subtracting the start time
        (recorded in setUp) from the current time, then prints the timing information
        in a standardized format.

        The timing output includes the test method name and execution duration in seconds
        with three decimal places of precision. If setUp failed before recording the start
        time, timing information is skipped to avoid errors.

        The method also calls the parent tearDown method to ensure proper test cleanup.
        """
        if hasattr(self, "_test_start_time"):
            test_duration = time.time() - self._test_start_time
            test_name = self._testMethodName
            print(f"\n[TEST TIMING] {test_name}: {test_duration:.3f} seconds")
        super().tearDown()
        gc.collect()
