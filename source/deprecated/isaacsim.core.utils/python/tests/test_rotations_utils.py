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

"""Tests for rotation utility functions."""

import numpy as np
import omni.kit.test
from isaacsim.core.utils.rotations import (
    euler_angles_to_quat,
    euler_to_rot_matrix,
    matrix_to_euler_angles,
    quat_to_euler_angles,
    rot_matrix_to_quat,
)
from scipy.spatial.transform import Rotation


class TestRotations(omni.kit.test.AsyncTestCase):
    """Test cases for Rotations."""

    # Before running each test
    async def setUp(self) -> None:
        """Set up test fixtures."""

    # After running each test
    async def tearDown(self) -> None:
        """Tear down test fixtures."""

    async def test_euler_angles_to_quat(self) -> None:
        """Test euler angles to quat."""
        roll, pitch, yaw = np.pi * np.random.rand(3)
        rot = Rotation.from_euler("xyz", [roll, pitch, yaw], degrees=False)

        x, y, z, w = rot.as_quat()
        self.assertTrue(
            np.all(
                np.isclose(
                    euler_angles_to_quat(np.array([roll, pitch, yaw])),
                    np.array([w, x, y, z]),
                    atol=1e-05,
                )
            )
            or np.all(
                np.isclose(
                    euler_angles_to_quat(np.array([roll, pitch, yaw])),
                    np.array([w, x, y, z]) * -1,
                    atol=1e-05,
                )
            ),
            f"{euler_angles_to_quat(np.array([roll, pitch, yaw]))} != {np.array([w, x, y, z])}",
        )

    async def test_quat_to_euler_angles(self) -> None:
        """Test quat to euler angles."""
        roll, pitch, yaw = np.pi * np.random.rand(3)
        rot = Rotation.from_euler("xyz", [roll, pitch, yaw], degrees=False)
        x, y, z, w = rot.as_quat()
        self.assertTrue(
            np.all(
                np.isclose(
                    euler_angles_to_quat(quat_to_euler_angles(np.array([w, x, y, z]))),
                    np.array([w, x, y, z]),
                    atol=1e-05,
                )
            )
            or np.all(
                np.isclose(
                    euler_angles_to_quat(quat_to_euler_angles(np.array([w, x, y, z]))),
                    -1 * np.array([w, x, y, z]),
                    atol=1e-05,
                )
            )
        )
        roll, pitch, yaw = [0.94965366, 2.3579875, 1.43057573]
        rot = Rotation.from_euler("xyz", [roll, pitch, yaw], degrees=False)
        x, y, z, w = rot.as_quat()
        self.assertTrue(
            np.all(
                np.isclose(
                    euler_angles_to_quat(quat_to_euler_angles(np.array([w, x, y, z]))),
                    np.array([w, x, y, z]),
                    atol=1e-05,
                )
            )
            or np.all(
                np.isclose(
                    euler_angles_to_quat(quat_to_euler_angles(np.array([w, x, y, z]))),
                    -1 * np.array([w, x, y, z]),
                    atol=1e-05,
                )
            )
        )

    async def test_euler_angles_to_matrix_to_quat(self) -> None:
        """Test euler angles to matrix to quat."""
        gt_quat = np.array([-0.5, -0.5, -0.5, 0.5])
        quat = rot_matrix_to_quat(
            euler_to_rot_matrix(matrix_to_euler_angles(np.array([[0, 1, 0], [0, 0, -1], [-1, 0, 0]])))
        )
        self.assertTrue(
            np.all(
                np.isclose(
                    gt_quat,
                    quat,
                    atol=1e-05,
                )
            )
            or np.all(
                np.isclose(
                    -1 * gt_quat,
                    quat,
                    atol=1e-05,
                )
            )
        )
