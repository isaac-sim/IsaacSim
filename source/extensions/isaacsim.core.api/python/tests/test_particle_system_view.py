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

import numpy as np

# NOTE:
#   omni.kit.test - std python's unittest module with additional wrapping to add support for async/await tests
#   For most things refer to unittest docs: https://docs.python.org/3/library/unittest.html
import omni.kit.test
import torch
from isaacsim.core.api import World
from isaacsim.core.prims import ParticleSystem, SingleParticleSystem
from isaacsim.core.utils.stage import create_new_stage_async, update_stage_async

from .common import CoreTestCase


class TestParticleSystemView(CoreTestCase):
    async def setUp(self):
        await super().setUp()
        World.clear_instance()
        await create_new_stage_async()
        self.my_world = World(backend="torch", device="cuda")
        await self.my_world.initialize_simulation_context_async()
        self._test_cfg = dict()
        pass

    async def tearDown(self):
        await super().tearDown()
        pass

    async def test_cloth_prim_vie(self):
        self.isclose = torch.isclose
        self._array_container = lambda x: torch.tensor(x, device=self._device, dtype=torch.float32)
        await update_stage_async()
        self.stage = omni.usd.get_context().get_stage()
        await self._runner()

    async def _runner(self):

        await update_stage_async()
        self.num_envs = 10
        radius = 0.5 * (0.6 / 5.0)
        restOffset = radius
        contactOffset = restOffset * 1.5

        for i in range(self.num_envs):
            self.particle_system = SingleParticleSystem(
                prim_path="/World/particleSystem_" + str(i),
                simulation_owner=self.my_world.get_physics_context().prim_path,
                rest_offset=restOffset,
                contact_offset=contactOffset,
                solid_rest_offset=restOffset,
                fluid_rest_offset=restOffset,
                particle_contact_offset=contactOffset,
            )

        # create a view to deal with all the cloths
        self.particle_system_view = ParticleSystem(prim_paths_expr="/World/particleSystem_*")
        self.my_world.scene.add(self.particle_system_view)
        await update_stage_async()

        for indexed in [False, True]:
            self._test_cfg["indexed"] = indexed
            print(self._test_cfg)
            await self.particle_contact_offsets_test()
            await self.solid_rest_offsets_test()
            await self.fluid_rest_offsets_test()
            await self.wind_test()

        await self.my_world.stop_async()
        self.my_world.clear_instance()

    async def particle_contact_offsets_test(self):
        await self.my_world.reset_async()
        indices = [1, 2] if self._test_cfg["indexed"] else None
        prev_values = self.particle_system_view.get_particle_contact_offsets(indices)
        new_values = prev_values + np.random.uniform(low=0.0, high=1.0, size=(prev_values.shape[0], 1)).astype(
            np.single
        )
        self.particle_system_view.set_particle_contact_offsets(new_values, indices)
        cur_values = self.particle_system_view.get_particle_contact_offsets(indices)
        self.assertTrue(self.isclose(new_values, cur_values).all())
        expected_shape = torch.Size([len(indices) if self._test_cfg["indexed"] else self.particle_system_view.count, 1])
        self.assertTrue(cur_values.shape == expected_shape)

    async def solid_rest_offsets_test(self):
        await self.my_world.reset_async()
        indices = [1, 2] if self._test_cfg["indexed"] else None
        prev_values = self.particle_system_view.get_solid_rest_offsets(indices)
        new_values = prev_values + np.random.uniform(low=0.0, high=1.0, size=(prev_values.shape[0], 1)).astype(
            np.single
        )
        self.particle_system_view.set_solid_rest_offsets(new_values, indices)
        cur_values = self.particle_system_view.get_solid_rest_offsets(indices)
        self.assertTrue(self.isclose(new_values, cur_values).all())
        expected_shape = torch.Size([len(indices) if self._test_cfg["indexed"] else self.particle_system_view.count, 1])
        self.assertTrue(cur_values.shape == expected_shape)

    async def fluid_rest_offsets_test(self):
        await self.my_world.reset_async()
        indices = [1, 2] if self._test_cfg["indexed"] else None
        prev_values = self.particle_system_view.get_fluid_rest_offsets(indices)
        new_values = prev_values + np.random.uniform(low=0.0, high=1.0, size=(prev_values.shape[0], 1)).astype(
            np.single
        )
        self.particle_system_view.set_fluid_rest_offsets(new_values, indices)
        cur_values = self.particle_system_view.get_fluid_rest_offsets(indices)
        self.assertTrue(self.isclose(new_values, cur_values).all())
        expected_shape = torch.Size([len(indices) if self._test_cfg["indexed"] else self.particle_system_view.count, 1])
        self.assertTrue(cur_values.shape == expected_shape)

    async def wind_test(self):
        await self.my_world.reset_async()
        indices = [1, 2] if self._test_cfg["indexed"] else None
        prev_values = self.particle_system_view.get_winds(indices)
        new_values = prev_values + np.random.uniform(low=0.0, high=1.0, size=(prev_values.shape[0], 3)).astype(
            np.single
        )
        self.particle_system_view.set_winds(new_values, indices)
        cur_values = self.particle_system_view.get_winds(indices)
        self.assertTrue(self.isclose(new_values, cur_values).all())
        expected_shape = torch.Size([len(indices) if self._test_cfg["indexed"] else self.particle_system_view.count, 3])
        self.assertTrue(cur_values.shape == expected_shape)
