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
from isaacsim.core.api import World
from isaacsim.core.api.materials.particle_material import ParticleMaterial
from isaacsim.core.api.tests.common import TestProperties

# NOTE:
#   omni.kit.test - std python's unittest module with additional wrapping to add support for async/await tests
#   For most things refer to unittest docs: https://docs.python.org/3/library/unittest.html
from isaacsim.core.utils.prims import define_prim
from isaacsim.core.utils.stage import create_new_stage_async, update_stage_async

# Import extension python module we are testing with absolute import path, as if we are external user (other extension)
from pxr import Gf, Usd, UsdGeom

from .common import CoreTestCase


class TestParticleMaterial(CoreTestCase, TestProperties):
    async def setUp(self):
        await super().setUp()
        World.clear_instance()
        await create_new_stage_async()
        self.my_world = World(backend="torch")  # , device="cuda")
        await self.my_world.initialize_simulation_context_async()
        self._test_cfg = dict()
        pass

    async def tearDown(self):
        await super().tearDown()
        pass

    async def test_cloth_prim(self):
        await update_stage_async()
        self.stage = omni.usd.get_context().get_stage()
        self.particle_material = ParticleMaterial(prim_path="/particleMaterial", drag=0.1, lift=0.3, friction=0.6)
        self.my_world.scene.add(self.particle_material)
        await self.my_world.reset_async(soft=False)
        await self.my_world.stop_async()

        await self.scalar_prop_test(self.particle_material.get_lift, self.particle_material.set_lift, is_stopped=True)
        await self.scalar_prop_test(self.particle_material.get_drag, self.particle_material.set_drag, is_stopped=True)
        await self.scalar_prop_test(
            self.particle_material.get_damping, self.particle_material.set_damping, is_stopped=True
        )
        await self.scalar_prop_test(
            self.particle_material.get_friction, self.particle_material.set_friction, is_stopped=True
        )
        await self.scalar_prop_test(
            self.particle_material.get_viscosity, self.particle_material.set_viscosity, is_stopped=True
        )
        await self.scalar_prop_test(
            self.particle_material.get_cohesion, self.particle_material.set_cohesion, is_stopped=True
        )
        await self.scalar_prop_test(
            self.particle_material.get_adhesion, self.particle_material.set_adhesion, is_stopped=True
        )
        await self.scalar_prop_test(
            self.particle_material.get_surface_tension, self.particle_material.set_surface_tension, is_stopped=True
        )
        await self.scalar_prop_test(
            self.particle_material.get_vorticity_confinement,
            self.particle_material.set_vorticity_confinement,
            is_stopped=True,
        )
        await self.scalar_prop_test(
            self.particle_material.get_gravity_scale, self.particle_material.set_gravity_scale, is_stopped=True
        )
        await self.scalar_prop_test(
            self.particle_material.get_adhesion_offset_scale,
            self.particle_material.set_adhesion_offset_scale,
            is_stopped=True,
        )
        await self.scalar_prop_test(
            self.particle_material.get_particle_friction_scale,
            self.particle_material.set_particle_friction_scale,
            is_stopped=True,
        )
        await self.scalar_prop_test(
            self.particle_material.get_particle_adhesion_scale,
            self.particle_material.set_particle_adhesion_scale,
            is_stopped=True,
        )

        await self.my_world.play_async()
        await self.scalar_prop_test(self.particle_material.get_lift, self.particle_material.set_lift, is_stopped=False)
        await self.scalar_prop_test(self.particle_material.get_drag, self.particle_material.set_drag, is_stopped=False)
        await self.scalar_prop_test(
            self.particle_material.get_damping, self.particle_material.set_damping, is_stopped=False
        )
        await self.scalar_prop_test(
            self.particle_material.get_friction, self.particle_material.set_friction, is_stopped=False
        )
        await self.scalar_prop_test(
            self.particle_material.get_viscosity, self.particle_material.set_viscosity, is_stopped=False
        )
        await self.scalar_prop_test(
            self.particle_material.get_cohesion, self.particle_material.set_cohesion, is_stopped=False
        )
        await self.scalar_prop_test(
            self.particle_material.get_adhesion, self.particle_material.set_adhesion, is_stopped=False
        )
        await self.scalar_prop_test(
            self.particle_material.get_surface_tension, self.particle_material.set_surface_tension, is_stopped=False
        )
        await self.scalar_prop_test(
            self.particle_material.get_vorticity_confinement,
            self.particle_material.set_vorticity_confinement,
            is_stopped=False,
        )
        await self.scalar_prop_test(
            self.particle_material.get_gravity_scale, self.particle_material.set_gravity_scale, is_stopped=False
        )
        await self.scalar_prop_test(
            self.particle_material.get_adhesion_offset_scale,
            self.particle_material.set_adhesion_offset_scale,
            is_stopped=False,
        )
        await self.scalar_prop_test(
            self.particle_material.get_particle_friction_scale,
            self.particle_material.set_particle_friction_scale,
            is_stopped=False,
        )
        await self.scalar_prop_test(
            self.particle_material.get_particle_adhesion_scale,
            self.particle_material.set_particle_adhesion_scale,
            is_stopped=False,
        )

        self.my_world.clear_instance()
