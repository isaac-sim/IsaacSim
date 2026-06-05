# SPDX-FileCopyrightText: Copyright (c) 2022-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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

"""Tests for simulation context crash scenarios to ensure timeline operations don't cause crashes."""

import asyncio

import isaacsim.core.experimental.utils.app as app_utils
import isaacsim.core.experimental.utils.stage as stage_utils
import omni.kit.test
import omni.timeline
from isaacsim.core.experimental.prims import Articulation
from isaacsim.storage.native import get_assets_root_path_async


# Having a test class derived from omni.kit.test.AsyncTestCase declared on the root of module will
# make it auto-discoverable by omni.kit.test
class TestSimulationContextCrash(omni.kit.test.AsyncTestCase):
    """Tests for simulation context crash scenarios."""

    # Before running each test
    async def setUp(self):
        """Set up test environment with new stage."""
        self._physics_dt = 1 / 60  # duration of physics frame in seconds

        self._timeline = omni.timeline.get_timeline_interface()

        await stage_utils.create_new_stage_async()
        await app_utils.update_app_async()

    # After running each test
    async def tearDown(self):
        """Clean up test environment and stop timeline."""
        self._timeline.stop()
        while omni.usd.get_context().get_stage_loading_status()[2] > 0:
            print("tearDown, assets still loading, waiting to finish...")
            await asyncio.sleep(1.0)
        await app_utils.update_app_async()

    async def test_simulation_context_crash(self):
        """Test that stopping timeline after articulation creation does not crash."""
        usd_path = await get_assets_root_path_async()
        usd_path += "/Isaac/Robots/Denso/CobottaPro900/cobotta_pro_900.usd"
        robot_prim_path = "/cobotta_pro_900"

        stage_utils.add_reference_to_stage(usd_path, robot_prim_path)

        self._timeline = omni.timeline.get_timeline_interface()

        # Start Simulation and wait
        self._timeline.play()
        await app_utils.update_app_async()

        # Create Articulation while timeline is playing
        self._robot = Articulation(robot_prim_path)
        await omni.kit.app.get_app().next_update_async()

        # Stop the timeline to mimic the old World initialization behavior
        self._timeline.stop()
        await app_utils.update_app_async()

        self.assertEqual(self._timeline.is_playing(), False)

        # Make sure this call doesn't crash due to invalid physx handles
        # Use experimental API to disable gravity on the articulation links
        self._robot.set_link_enabled_gravities(False)
