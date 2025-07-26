# SPDX-FileCopyrightText: Copyright (c) 2022-2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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
from isaacsim.core.api.materials.physics_material import PhysicsMaterial

# Import extension python module we are testing with absolute import path, as if we are external user (other extension)
from isaacsim.core.prims import SingleGeometryPrim

# NOTE:
#   omni.kit.test - std python's unittest module with additional wrapping to add support for async/await tests
#   For most things refer to unittest docs: https://docs.python.org/3/library/unittest.html
from isaacsim.core.utils.prims import define_prim
from pxr import UsdPhysics

from .common import CoreTestCase


class TestSingleGeometryPrim(CoreTestCase):
    async def setUp(self):
        await super().setUp()
        await omni.usd.get_context().new_stage_async()
        await omni.kit.app.get_app().next_update_async()
        pass

    async def tearDown(self):
        await super().tearDown()
        pass

    async def test_collision_approximation(self):
        define_prim("/test", prim_type="cube")
        geometry_prim = SingleGeometryPrim("/test", "test", collision=True)
        approximations = ["convexHull", "convexDecomposition"]
        for possible_approx in approximations:
            geometry_prim.set_collision_approximation(possible_approx)
            self.assertEqual(possible_approx, geometry_prim.get_collision_approximation())
        return

    async def test_collision_enabled(self):
        define_prim("/test", prim_type="cube")
        geometry_prim = SingleGeometryPrim("/test", "test")
        api = UsdPhysics.CollisionAPI.Apply(geometry_prim.prim)
        api.GetCollisionEnabledAttr().Set(True)
        self.assertTrue(geometry_prim.get_collision_enabled())
        return

    async def test_physics_material(self):
        define_prim("/test", prim_type="cube")
        geometry_prim = SingleGeometryPrim("/test", "test")
        physics_material = PhysicsMaterial(
            prim_path="/Physics_material_1", dynamic_friction=0.2, static_friction=0.2, restitution=0.0
        )
        geometry_prim.apply_physics_material(physics_material=physics_material)
        self.assertEqual(geometry_prim.get_applied_physics_material(), physics_material)
        return
