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

"""Tests for the ScreenPrinter UI component."""

import asyncio

import numpy as np

# NOTE:
#   omni.kit.test - std python's unittest module with additional wrapping to add suport for async/await tests
#   For most things refer to unittest docs: https://docs.python.org/3/library/unittest.html
import omni.kit.test
from isaacsim.gui.components import ScreenPrinter


# Having a test class dervived from omni.kit.test.AsyncTestCase declared on the root of module will make it auto-discoverable by omni.kit.test
class TestScreenPrinter(omni.kit.test.AsyncTestCase):
    """Test suite for the ScreenPrinter UI component."""

    # Before running each test
    async def setUp(self) -> None:
        """Set up the test environment with a new stage and timeline."""
        await omni.usd.get_context().new_stage_async()
        self._timeline = omni.timeline.get_timeline_interface()
        self._timeline.play()
        await omni.kit.app.get_app().next_update_async()

        self.printers = []

    # After running each test
    async def tearDown(self) -> None:
        """Clean up the test environment."""
        self._timeline.stop()
        while omni.usd.get_context().get_stage_loading_status()[2] > 0:
            print("tearDown, assets still loading, waiting to finish...")
            await asyncio.sleep(0.1)
        await omni.kit.app.get_app().next_update_async()

        # This clears the
        [printer.exit() for printer in self.printers]

    # Run for a single frame and exit
    async def test_screen_printer(self) -> None:
        """Test ScreenPrinter text display and configuration."""
        printer = ScreenPrinter()
        self.printers.append(printer)
        printer.set_text("This is a test")
        await omni.kit.app.get_app().next_update_async()
        printer.set_text_color(np.array([1.5, 1.5, 0, 1]))
        printer.set_text_max_width(5)
        printer.set_text_position(50, 50)
        printer.set_text_size(20)
        await omni.kit.app.get_app().next_update_async()

        printer2 = ScreenPrinter()
        printer2.set_text("Printing two things!")
        self.printers.append(printer2)

        await omni.kit.app.get_app().next_update_async()
