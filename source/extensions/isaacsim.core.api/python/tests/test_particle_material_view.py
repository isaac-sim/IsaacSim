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

import asyncio

import numpy as np

# NOTE:
#   omni.kit.test - std python's unittest module with additional wrapping to add support for async/await tests
#   For most things refer to unittest docs: https://docs.python.org/3/library/unittest.html
import omni.kit.test
import torch
from isaacsim.core.api import World
from isaacsim.core.api.materials.particle_material import ParticleMaterial
from isaacsim.core.api.materials.particle_material_view import ParticleMaterialView
from isaacsim.core.utils.stage import create_new_stage_async, get_current_stage, update_stage_async

from .common import CoreTestCase


class TestParticleMaterialView(CoreTestCase):
    async def setUp(self):
        await super().setUp()
        World.clear_instance()
        await create_new_stage_async()
        self.my_world = World(backend="torch")  # , device="cuda")
        await self.my_world.initialize_simulation_context_async()
        await update_stage_async()
        self._test_cfg = dict()
        pass

    async def tearDown(self):
        await super().tearDown()
        pass

    async def test_particle_material_view(self):
        self.isclose = torch.isclose
        self._array_container = lambda x: torch.tensor(x, device=self._device, dtype=torch.float32)
        self.stage = get_current_stage()
        await update_stage_async()
        await self._runner()
        pass

    async def _runner(self):
        self.num_envs = 10
        for i in range(self.num_envs):
            self.particle_material = ParticleMaterial(
                prim_path="/World/particleMaterial_" + str(i), drag=0.1, lift=0.3, friction=0.6
            )

        # create a view to deal with all the cloths
        self.particle_material_view = ParticleMaterialView(prim_paths_expr="/World/particleMaterial_*")
        self.my_world.scene.add(self.particle_material_view)
        await update_stage_async()

        for indexed in [False, True]:
            self._test_cfg["indexed"] = indexed
            print(self._test_cfg)
            await self.friction_test()
            await self.damping_test()
            await self.lift_test()
            await self.drag_test()
            await self.gravity_scale_test()

        await self.my_world.stop_async()

    async def friction_test(self):
        await self.my_world.reset_async()
        indices = [1, 2] if self._test_cfg["indexed"] else None
        prev_values = self.particle_material_view.get_frictions(indices)
        new_values = prev_values + np.random.uniform(low=0.0, high=1.0, size=(prev_values.shape[0], 1)).astype(
            np.single
        )
        self.particle_material_view.set_frictions(new_values, indices)
        cur_values = self.particle_material_view.get_frictions(indices)
        self.assertTrue(self.isclose(new_values, cur_values).all())
        expected_shape = torch.Size(
            [len(indices) if self._test_cfg["indexed"] else self.particle_material_view.count, 1]
        )
        self.assertTrue(cur_values.shape == expected_shape)
        print(expected_shape)

    async def damping_test(self):
        await self.my_world.reset_async()
        indices = [1, 2] if self._test_cfg["indexed"] else None
        prev_values = self.particle_material_view.get_dampings(indices)
        new_values = prev_values + np.random.uniform(low=0.0, high=1.0, size=(prev_values.shape[0], 1)).astype(
            np.single
        )
        self.particle_material_view.set_dampings(new_values, indices)
        cur_values = self.particle_material_view.get_dampings(indices)
        self.assertTrue(self.isclose(new_values, cur_values).all())
        expected_shape = torch.Size(
            [len(indices) if self._test_cfg["indexed"] else self.particle_material_view.count, 1]
        )
        self.assertTrue(cur_values.shape == expected_shape)
        print(expected_shape)

    async def damping_test(self):
        await self.my_world.reset_async()
        indices = [1, 2] if self._test_cfg["indexed"] else None
        prev_values = self.particle_material_view.get_dampings(indices)
        new_values = prev_values + np.random.uniform(low=0.0, high=1.0, size=(prev_values.shape[0], 1)).astype(
            np.single
        )
        self.particle_material_view.set_dampings(new_values, indices)
        cur_values = self.particle_material_view.get_dampings(indices)
        self.assertTrue(self.isclose(new_values, cur_values).all())
        expected_shape = torch.Size(
            [len(indices) if self._test_cfg["indexed"] else self.particle_material_view.count, 1]
        )
        self.assertTrue(cur_values.shape == expected_shape)
        print(expected_shape)

    async def lift_test(self):
        await self.my_world.reset_async()
        indices = [1, 2] if self._test_cfg["indexed"] else None
        prev_values = self.particle_material_view.get_lifts(indices)
        new_values = prev_values + np.random.uniform(low=0.0, high=1.0, size=(prev_values.shape[0], 1)).astype(
            np.single
        )
        self.particle_material_view.set_lifts(new_values, indices)
        cur_values = self.particle_material_view.get_lifts(indices)
        self.assertTrue(self.isclose(new_values, cur_values).all())
        expected_shape = torch.Size(
            [len(indices) if self._test_cfg["indexed"] else self.particle_material_view.count, 1]
        )
        self.assertTrue(cur_values.shape == expected_shape)
        print(expected_shape)

    async def drag_test(self):
        await self.my_world.reset_async()
        indices = [1, 2] if self._test_cfg["indexed"] else None
        prev_values = self.particle_material_view.get_drags(indices)
        new_values = prev_values + np.random.uniform(low=0.0, high=1.0, size=(prev_values.shape[0], 1)).astype(
            np.single
        )
        self.particle_material_view.set_drags(new_values, indices)
        cur_values = self.particle_material_view.get_drags(indices)
        self.assertTrue(self.isclose(new_values, cur_values).all())
        expected_shape = torch.Size(
            [len(indices) if self._test_cfg["indexed"] else self.particle_material_view.count, 1]
        )
        self.assertTrue(cur_values.shape == expected_shape)
        print(expected_shape)

    async def gravity_scale_test(self):
        await self.my_world.reset_async()
        indices = [1, 2] if self._test_cfg["indexed"] else None
        prev_values = self.particle_material_view.get_gravity_scales(indices)
        new_values = prev_values + np.random.uniform(low=0.0, high=1.0, size=(prev_values.shape[0], 1)).astype(
            np.single
        )
        self.particle_material_view.set_gravity_scales(new_values, indices)
        cur_values = self.particle_material_view.get_gravity_scales(indices)
        self.assertTrue(self.isclose(new_values, cur_values).all())
        expected_shape = torch.Size(
            [len(indices) if self._test_cfg["indexed"] else self.particle_material_view.count, 1]
        )
        self.assertTrue(cur_values.shape == expected_shape)
        print(expected_shape)
