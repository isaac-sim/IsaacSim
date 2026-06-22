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

"""Verifies that SimulationManager and PhysxScene expose complete public API docstrings. The tests prepare a clean stage and run the shared docstring checker against the simulation manager classes."""

import isaacsim.core.experimental.utils.impl.stage as stage_utils
import isaacsim.test.docstring
import warp as wp
from isaacsim.core.simulation_manager import PhysxScene, SimulationManager

wp.init()  # init warp to avoid undesired stdout output


class TestExtensionDocstrings(isaacsim.test.docstring.AsyncDocTestCase):
    """Test extension docstrings."""

    async def setUp(self) -> None:
        """Method called to prepare the test fixture."""
        super().setUp()
        # create new stage
        await stage_utils.create_new_stage_async()

    async def tearDown(self) -> None:
        """Method called immediately after the test method has been called."""
        super().tearDown()

    async def test_physx_scene_docstrings(self) -> None:
        """Test physx scene docstrings."""
        await self.assertDocTests(PhysxScene)

    async def test_simulation_manager_docstrings(self) -> None:
        """Test simulation manager docstrings."""
        if (SimulationManager.get_default_engine() or "").lower() == "newton":
            self.skipTest("Skipping SimulationManager docstrings test (engine: newton)")
        SimulationManager.initialize_physics()
        await self.assertDocTests(SimulationManager)
        SimulationManager.invalidate_physics()
