# SPDX-FileCopyrightText: Copyright (c) 2021-2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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

import omni.kit.test
from isaacsim.core.api import SimulationContext, World
from isaacsim.core.utils.stage import create_new_stage_async, get_current_stage, get_stage_units, set_stage_units

from .common import CoreTestCase


class TestSimulationContext(CoreTestCase):
    # Before running each test
    async def setUp(self):
        await super().setUp()
        await create_new_stage_async()
        World.clear_instance()
        pass

    # After running each test
    async def tearDown(self):
        await super().tearDown()
        pass

    async def test_singleton(self):
        my_world_1 = World()
        my_world_2 = World()
        self.assertTrue(my_world_1 == my_world_2)
        await omni.kit.app.get_app().next_update_async()

        # try to delete the previous one
        my_world_2.clear_instance()
        self.assertTrue(my_world_1.instance() is None)
        my_world_3 = World()
        self.assertTrue(my_world_1 != my_world_3)
        self.assertTrue(my_world_1.instance() == my_world_3.instance())
        my_world_3.clear_instance()
        return

    async def test_set_defaults(self):
        await create_new_stage_async()
        set_stage_units(stage_units_in_meters=1.0)
        simulation_context = SimulationContext(set_defaults=False)
        await simulation_context.initialize_simulation_context_async()
        self.assertTrue(get_stage_units() == 1.0)
        await create_new_stage_async()
        set_stage_units(stage_units_in_meters=1.0)
        simulation_context = SimulationContext(set_defaults=True)
        await simulation_context.initialize_simulation_context_async()
        self.assertTrue(get_stage_units() == 1.0)
        await create_new_stage_async()
        set_stage_units(stage_units_in_meters=1.0)
        simulation_context = SimulationContext(stage_units_in_meters=100.0, set_defaults=True)
        await simulation_context.initialize_simulation_context_async()
        self.assertTrue(get_stage_units() == 100.0)
        await create_new_stage_async()
        set_stage_units(stage_units_in_meters=1.0)
        simulation_context = SimulationContext(stage_units_in_meters=100.0, set_defaults=False)
        await simulation_context.initialize_simulation_context_async()
        self.assertTrue(get_stage_units() == 100.0)
        # try set simulation dt with Nones
        simulation_context.set_simulation_dt(physics_dt=None, rendering_dt=None)
        return

    async def test_default_dt(self):
        await create_new_stage_async()
        stage = get_current_stage()
        stage.SetTimeCodesPerSecond(90)
        set_stage_units(stage_units_in_meters=1.0)
        simulation_context = SimulationContext(set_defaults=False)
        await simulation_context.initialize_simulation_context_async()
        assert 1.0 / simulation_context.get_rendering_dt() == stage.GetTimeCodesPerSecond()
        return
