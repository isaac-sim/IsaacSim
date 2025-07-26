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
import omni.kit.test
from isaacsim.core.api import World

# Import extension python module we are testing with absolute import path, as if we are external user (other extension)
from isaacsim.core.prims import SingleRigidPrim

# NOTE:
#   omni.kit.test - std python's unittest module with additional wrapping to add support for async/await tests
#   For most things refer to unittest docs: https://docs.python.org/3/library/unittest.html
from isaacsim.core.utils.prims import define_prim
from isaacsim.core.utils.rotations import euler_angles_to_quat
from isaacsim.core.utils.stage import create_new_stage_async, update_stage_async

from .common import CoreTestCase


class TestSingleRigidPrimPose(CoreTestCase):
    async def setUp(self):
        await super().setUp()
        World.clear_instance()
        await create_new_stage_async()
        self._my_world = World(device="cpu")
        await self._my_world.initialize_simulation_context_async()
        pass

    async def tearDown(self):
        await super().tearDown()
        pass

    async def test_position_orientation_scale(self):
        # Test constructor setting of pose
        position = [1.0, 2.0, 3.0]
        orientation = np.array(euler_angles_to_quat([45, -60, 180], degrees=True))
        scale = np.array([0.1, 0.1, 0.1])
        define_prim("/test", prim_type="Cube")
        await update_stage_async()
        rigid_prim = SingleRigidPrim("/test", "test", position=np.array(position), orientation=orientation)
        rigid_prim.set_local_scale(scale)
        real_position, real_orientation = rigid_prim.get_local_pose()
        # TODO: this is buggy for some reason, world scale is equal to 1.0 in this case for some reason
        # real_scale = rigid_prim.get_world_scale()
        real_scale = rigid_prim.get_local_scale()
        for i in range(3):
            self.assertAlmostEqual(real_position[i], position[i])
            self.assertAlmostEqual(real_orientation[i], orientation[i])
            self.assertAlmostEqual(scale[i], real_scale[i])

        define_prim("/test_2", prim_type="Cube")
        await update_stage_async()
        rigid_prim = SingleRigidPrim("/test_2", "test")
        rigid_prim.set_local_scale(scale)
        real_position, real_orientation = rigid_prim.get_local_pose()
        real_scale = rigid_prim.get_local_scale()
        for i in range(3):
            self.assertAlmostEqual(scale[i], real_scale[i])

        define_prim("/test_3", prim_type="Cube")
        rigid_prim = SingleRigidPrim("/test_3", "test")
        rigid_prim.set_local_scale(scale)
        real_position, real_orientation = rigid_prim.get_local_pose()
        real_scale = rigid_prim.get_local_scale()
        for i in range(3):
            self.assertAlmostEqual(scale[i], real_scale[i])
        return

    async def test_set_local_pose(self):
        # Test constructor setting of pose
        position = [1.0, 2.0, 3.0]
        orientation = np.array(euler_angles_to_quat([45, -60, 180], degrees=True))
        define_prim("/test_5", prim_type="Cube")
        await update_stage_async()
        rigid_prim = SingleRigidPrim(
            "/test_5", "test_5", position=np.array(position), orientation=orientation, mass=1.0
        )
        self._my_world.scene.add(rigid_prim)
        await self._my_world.reset_async()
        rigid_prim.set_local_pose(translation=np.array([0, 2.0, 0]))
        return
