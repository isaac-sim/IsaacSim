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

# Import extension python module we are testing with absolute import path, as if we are external user (other extension)
from isaacsim.core.prims import SingleXFormPrim
from isaacsim.core.simulation_manager import SimulationManager
from isaacsim.core.utils.rotations import euler_angles_to_quat

from .common import CoreTestCase


class TestXformPrimPose(CoreTestCase):
    async def setUp(self):
        await super().setUp()
        await omni.usd.get_context().new_stage_async()
        await omni.kit.app.get_app().next_update_async()
        SimulationManager.set_backend("numpy")
        pass

    async def tearDown(self):
        await super().tearDown()
        pass

    async def test_position_orientation_scale(self):
        # Test constructor setting of pose
        position = [1.0, 2.0, 3.0]
        orientation = np.array(euler_angles_to_quat([45, -60, 180], degrees=True))
        scale = np.array([0.1, 0.1, 0.1])
        xform_prim = SingleXFormPrim("/test", "test", position=np.array(position), orientation=orientation, scale=scale)

        real_position, real_orientation = xform_prim.get_local_pose()
        real_scale = xform_prim.get_world_scale()
        for i in range(3):
            self.assertAlmostEqual(real_position[i], position[i])
            self.assertAlmostEqual(real_orientation[i], orientation[i])
            self.assertAlmostEqual(scale[i], real_scale[i])

        xform_prim = SingleXFormPrim("/test_2", "test", scale=scale)
        real_position, real_orientation = xform_prim.get_local_pose()
        real_scale = xform_prim.get_world_scale()
        for i in range(3):
            # print(scale[i])
            self.assertAlmostEqual(scale[i], real_scale[i])

        xform_prim = SingleXFormPrim("/test_3", "test")

        xform_prim.set_local_scale(scale)
        real_position, real_orientation = xform_prim.get_local_pose()
        real_scale = xform_prim.get_world_scale()
        for i in range(3):
            # print(scale[i])
            self.assertAlmostEqual(scale[i], real_scale[i])
