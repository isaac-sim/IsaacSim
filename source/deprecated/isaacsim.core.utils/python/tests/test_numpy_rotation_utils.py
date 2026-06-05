# SPDX-FileCopyrightText: Copyright (c) 2022-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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

"""Tests for NumPy rotation utility functions."""

import isaacsim.core.utils.numpy.rotations as rotation_conversions
import numpy as np
import omni.kit.test


# Having a test class derived from omni.kit.test.AsyncTestCase declared on the root of module will make it auto-discoverable by omni.kit.test
class TestRotationUtils(omni.kit.test.AsyncTestCase):
    """Test cases for RotationUtils."""

    async def setUp(self) -> None:
        """Set up test fixtures."""
        await omni.usd.get_context().new_stage_async()
        await omni.kit.app.get_app().next_update_async()

    async def tearDown(self) -> None:
        """Tear down test fixtures."""
        await omni.kit.app.get_app().next_update_async()

    async def test_rotation_conversions(self) -> None:
        """Test rotation conversions."""
        # ith index of rotations are equivalent

        quats = np.array([[0.4374512, 0.4021533, 0.8043066, 0], [0.7602446, -0.4593627, 0, 0.4593627]])
        euler_angs = np.array([[2.62352489, 0.78057745, 1.99712744], [-0.879511, 0.43568131, 0.879511]])
        rot_vecs = np.array([[1, 2, 0], [-1, 0, 1]])

        rot_mats = np.array(
            [
                [
                    [-0.2938183, 0.6469092, 0.7036898],
                    [0.6469092, 0.6765454, -0.3518449],
                    [-0.7036898, 0.3518449, -0.6172729],
                ],
                [
                    [0.5779719, -0.6984560, -0.4220282],
                    [0.6984560, 0.1559437, 0.6984560],
                    [-0.4220282, -0.6984560, 0.5779719],
                ],
            ]
        )

        self.assertTrue(np.all(np.isclose(rotation_conversions.quats_to_rot_matrices(quats), rot_mats, rtol=1e-3)))
        self.assertTrue(np.all(np.isclose(rotation_conversions.quats_to_rotvecs(quats), rot_vecs, rtol=1e-3)))

        self.assertTrue(np.all(np.isclose(rotation_conversions.quats_to_euler_angles(quats), euler_angs, rtol=1e-3)))
        self.assertTrue(np.all(np.isclose(rotation_conversions.rot_matrices_to_quats(rot_mats), quats, rtol=1e-3)))
        self.assertTrue(np.all(np.isclose(rotation_conversions.euler_angles_to_quats(euler_angs), quats, rtol=1e-3)))
        self.assertTrue(np.all(np.isclose(rotation_conversions.rotvecs_to_quats(rot_vecs), quats, rtol=1e-3)))

        self.assertTrue(rotation_conversions.quats_to_rot_matrices(quats[0]).shape == (3, 3))
        self.assertTrue(rotation_conversions.quats_to_rotvecs(quats[0]).shape == (3,))
        self.assertTrue(rotation_conversions.quats_to_euler_angles(quats[0]).shape == (3,))
        self.assertTrue(rotation_conversions.rot_matrices_to_quats(rot_mats[0]).shape == (4,))
        self.assertTrue(rotation_conversions.rotvecs_to_quats(rot_vecs[0]).shape == (4,))
        self.assertTrue(rotation_conversions.euler_angles_to_quats(euler_angs[0]).shape == (4,))
