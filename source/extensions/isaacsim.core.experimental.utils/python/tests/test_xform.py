# SPDX-FileCopyrightText: Copyright (c) 2021-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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

"""Test for xform."""

import isaacsim.core.experimental.utils.backend as backend_utils
import isaacsim.core.experimental.utils.stage as stage_utils
import isaacsim.core.experimental.utils.xform as xform_utils
import numpy as np
import omni.kit.test


class TestXform(omni.kit.test.AsyncTestCase):
    """Test xform."""

    async def setUp(self):
        """Method called to prepare the test fixture."""
        super().setUp()
        # create new stage
        await stage_utils.create_new_stage_async()

    async def tearDown(self):
        """Method called immediately after the test method has been called."""
        super().tearDown()

    # --------------------------------------------------------------------

    async def test_world_and_local_pose(self):
        """Test world and local pose."""

        def _check_pose(pose, position, orientation, *, rtol: float = 1e-03, atol: float = 1e-05):
            np.testing.assert_allclose(pose[0].numpy(), position, rtol=rtol, atol=atol)
            np.testing.assert_allclose(pose[1].numpy(), orientation, rtol=rtol, atol=atol)

        path_a = "/World/A"
        path_b = "/World/A/B"
        stage_utils.define_prim(path_a)
        stage_utils.define_prim(path_b)
        # test cases
        with backend_utils.use_backend("usdrt"):
            # set local poses (A: euler-XYZ:90,0,0 | B: euler-XYZ:0,-90,00)
            xform_utils.set_local_pose(path_a, translation=[1.0, 2.0, 3.0], orientation=[0.7071, 0.7071, 0.0, 0.0])
            xform_utils.set_local_pose(path_b, translation=[-6.0, -5.0, -4.0], orientation=[0.7071, 0.0, -0.7071, 0])
            # get local poses
            _check_pose(xform_utils.get_local_pose(path_a), [1.0, 2.0, 3.0], [0.7071, 0.7071, 0.0, 0.0])
            _check_pose(xform_utils.get_local_pose(path_b), [-6.0, -5.0, -4.0], [0.7071, 0.0, -0.7071, 0])
            # get world poses
            _check_pose(xform_utils.get_world_pose(path_a), [1.0, 2.0, 3.0], [0.7071, 0.7071, 0.0, 0.0])
            _check_pose(xform_utils.get_world_pose(path_b), [-5.0, 6.0, -2.0], [0.5, 0.5, -0.5, -0.5])
            # set world poses (B: euler-XYZ:0,0,180)
            xform_utils.set_world_pose(path_b, position=[4.0, 5.0, 6.0], orientation=[0.0, 0.0, 0.0, 1.0])
            # get local poses
            _check_pose(xform_utils.get_local_pose(path_b), [3.0, 3.0, -3.0], [0.0, 0.0, 0.7071, 0.7071])

    async def test_get_relative_transform(self):
        """Verify get_relative_transform resolves USD prims and returns a valid 4x4 matrix."""
        path_a = "/World/A"
        path_b = "/World/B"
        stage_utils.define_prim(path_a)
        stage_utils.define_prim(path_b)
        with backend_utils.use_backend("usdrt"):
            xform_utils.set_local_pose(path_a, translation=[5.0, 3.0, 1.0], orientation=[0.7071, 0.7071, 0.0, 0.0])
            xform_utils.set_local_pose(path_b, translation=[2.0, 0.0, 0.0])

        result = xform_utils.get_relative_transform(path_a, path_b)
        self.assertEqual(result.shape, (4, 4))
        np.testing.assert_allclose(xform_utils.get_relative_transform(path_a, path_a), np.eye(4), atol=1e-6)

    async def test_get_relative_transform_accepts_prim_objects(self):
        """get_relative_transform should accept Usd.Prim objects in addition to paths."""
        import omni.usd

        path_a = "/World/A"
        path_b = "/World/B"
        stage_utils.define_prim(path_a)
        stage_utils.define_prim(path_b)
        with backend_utils.use_backend("usdrt"):
            xform_utils.set_local_pose(path_a, translation=[2.0, 0.0, 0.0])
            xform_utils.set_local_pose(path_b, translation=[0.0, 3.0, 0.0])

        stage = omni.usd.get_context().get_stage()
        prim_a = stage.GetPrimAtPath(path_a)
        prim_b = stage.GetPrimAtPath(path_b)

        result_paths = xform_utils.get_relative_transform(path_a, path_b)
        result_prims = xform_utils.get_relative_transform(prim_a, prim_b)
        np.testing.assert_allclose(result_paths, result_prims, atol=1e-6)
