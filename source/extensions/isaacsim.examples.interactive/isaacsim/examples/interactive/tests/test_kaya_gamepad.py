"""Tests for the Kaya gamepad interactive example."""

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

import carb
import isaacsim.core.experimental.utils.app as app_utils
import omni.kit

# NOTE:
#   omni.kit.test - std python's unittest module with additional wrapping to add suport for async/await tests
#   For most things refer to unittest docs: https://docs.python.org/3/library/unittest.html
import omni.kit.test
from isaacsim.core.experimental.prims import RigidPrim

# Import extension python module we are testing with absolute import path, as if we are external user (other extension)
from isaacsim.examples.interactive.kaya_gamepad import KayaGamepad
from omni.kit.app import get_app


class TestKayaGamepadSample(omni.kit.test.AsyncTestCase):
    """Test cases for the Kaya gamepad sample."""

    # Before running each test
    async def setUp(self) -> None:
        """Set up the gamepad provider and Kaya sample."""
        self._provider = carb.input.acquire_input_provider()
        self._gamepad = self._provider.create_gamepad("test", "0")
        await get_app().next_update_async()

        self._sample = KayaGamepad()
        self._sample.set_world_settings(physics_dt=1.0 / 60, stage_units_in_meters=1.0)
        await self._sample.load_world_async()

    # After running each test
    async def tearDown(self) -> None:
        """Tear down by stopping timeline and cleaning up resources."""
        # Stop timeline if running
        if app_utils.is_playing():
            app_utils.stop()

        await get_app().next_update_async()

        # Clean up physics callbacks if any
        if hasattr(self, "_sample") and self._sample:
            self._sample.physics_cleanup()

        self._sample = None
        await get_app().next_update_async()
        self._provider.destroy_gamepad(self._gamepad)
        await get_app().next_update_async()

    # Run all functions with simulation enabled
    async def test_simulation(self) -> None:
        """Test that gamepad input moves the Kaya robot forward."""
        await get_app().next_update_async()

        # Access the kaya robot prim directly by path
        kaya_prim = RigidPrim("/kaya/base_link")

        # Connect the gamepad so OmniGraph nodes can detect it
        self._provider.set_gamepad_connected(self._gamepad, True)
        await get_app().next_update_async()

        # Check initial position (convert warp array to numpy)
        initial_position = kaya_prim.get_world_poses()[0].numpy()
        self.assertLess(initial_position[0][0], 0.1)

        # Start the simulation - Action Graphs only execute when timeline is playing
        app_utils.play()
        await get_app().next_update_async()

        # Send gamepad input to move the robot forward (enough steps for motion in CI)
        for i in range(600):
            self._provider.buffer_gamepad_event(self._gamepad, carb.input.GamepadInput.LEFT_STICK_UP, 1.0)
            await get_app().next_update_async()

        # Stop simulation and disconnect gamepad
        app_utils.pause()
        self._provider.set_gamepad_connected(self._gamepad, False)
        await get_app().next_update_async()

        # Verify robot moved forward (positive X direction, convert warp array to numpy)
        final_position = kaya_prim.get_world_poses()[0].numpy()
        self.assertGreater(final_position[0][0], 0.2)
