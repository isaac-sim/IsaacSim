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

import isaacsim.core.utils.deformable_mesh_utils as DeformableMeshUtils
import omni.kit.test
import torch
from isaacsim.core.api import World
from isaacsim.core.api.materials.particle_material import ParticleMaterial
from isaacsim.core.api.tests.common import TestProperties
from isaacsim.core.prims import SingleDeformablePrim

# NOTE:
#   omni.kit.test - std python's unittest module with additional wrapping to add support for async/await tests
#   For most things refer to unittest docs: https://docs.python.org/3/library/unittest.html
from isaacsim.core.utils.prims import define_prim
from isaacsim.core.utils.stage import create_new_stage_async, update_stage_async
from omni.physx.scripts import deformableUtils, physicsUtils

# Import extension python module we are testing with absolute import path, as if we are external user (other extension)
from pxr import Gf, Usd, UsdGeom

from .common import CoreTestCase


class TestSingleDeformablePrim(CoreTestCase, TestProperties):
    async def setUp(self):
        await super().setUp()
        World.clear_instance()
        await create_new_stage_async()
        self.my_world = World(backend="torch", device="cuda")
        await self.my_world.initialize_simulation_context_async()

    async def tearDown(self):
        await super().tearDown()

    async def test_deformable_prim(self):
        await update_stage_async()
        self.stage = omni.usd.get_context().get_stage()
        env_path = "/World/Env"
        env = UsdGeom.Xform.Define(self.stage, env_path)
        # set up the geometry
        deformable_path = env.GetPrim().GetPath().AppendChild("deformable")
        self.plane_mesh = UsdGeom.Mesh.Define(self.stage, deformable_path)
        tri_points, tri_indices = DeformableMeshUtils.createTriangleMeshCube(8)
        self.plane_mesh.GetPointsAttr().Set(tri_points)
        self.plane_mesh.GetFaceVertexIndicesAttr().Set(tri_indices)
        self.plane_mesh.GetFaceVertexCountsAttr().Set([3] * (len(tri_indices) // 3))
        physicsUtils.setup_transform_as_scale_orient_translate(self.plane_mesh)
        physicsUtils.set_or_add_translate_op(self.plane_mesh, Gf.Vec3f(2, 0.0, 2.0))
        physicsUtils.set_or_add_orient_op(self.plane_mesh, Gf.Rotation(Gf.Vec3d([1, 0, 0]), 15).GetQuat())
        # self.particle_material = ParticleMaterial(prim_path=particle_material_path, drag=0.1, lift=0.3, friction=0.6)
        self.deformable = SingleDeformablePrim(prim_path=str(deformable_path))
        self.my_world.scene.add(self.deformable)
        await self.my_world.reset_async(soft=False)
        await self.my_world.stop_async()

        for timeline in [True, False]:
            await self.int_prop_test(
                self.deformable.get_solver_position_iteration_count,
                self.deformable.set_solver_position_iteration_count,
                is_stopped=timeline,
            )
            await self.bool_prop_test(
                self.deformable.get_self_collision, self.deformable.set_self_collision, is_stopped=timeline
            )
            await self.scalar_prop_test(
                self.deformable.get_self_collision_filter_distance,
                self.deformable.set_self_collision_filter_distance,
                is_stopped=timeline,
            )
            await self.scalar_prop_test(
                self.deformable.get_settling_threshold, self.deformable.set_settling_threshold, is_stopped=timeline
            )
            await self.scalar_prop_test(
                self.deformable.get_sleep_threshold, self.deformable.set_sleep_threshold, is_stopped=timeline
            )
            await self.scalar_prop_test(
                self.deformable.get_sleep_damping, self.deformable.set_sleep_damping, is_stopped=timeline
            )
            await self.scalar_prop_test(
                self.deformable.get_vertex_velocity_damping,
                self.deformable.set_vertex_velocity_damping,
                is_stopped=timeline,
            )

            if not self.my_world.is_playing():
                await self.my_world.play_async()
