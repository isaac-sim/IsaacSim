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

"""Tests for distance metric functions."""

from typing import Any

import numpy as np

# NOTE:
#   omni.kit.test - std python's unittest module with additional wrapping to add support for async/await tests
#   For most things refer to unittest docs: https://docs.python.org/3/library/unittest.html
import omni.kit.test

# Import extension python module we are testing with absolute import path, as if we are external user (other extension)
from isaacsim.core.utils.distance_metrics import (
    rotational_distance_angle,
    rotational_distance_identity_matrix_deviation,
    rotational_distance_single_axis,
    weighted_translational_distance,
)
from pxr import Gf
from scipy.spatial.transform import Rotation as R


# Having a test class derived from omni.kit.test.AsyncTestCase declared on the root of module will make it auto-discoverable by omni.kit.test
class TestDistanceMetrics(omni.kit.test.AsyncTestCase):
    """Test cases for DistanceMetrics."""

    async def setUp(self) -> None:
        """Set up test fixtures."""

    async def tearDown(self) -> None:
        """Tear down test fixtures."""

    async def is_distance_metric(self, t1: Any, t2: Any, t3: Any, dist_fun: Any, *args: Any) -> None:
        """Verify that a function satisfies distance metric properties.

        Args:
            t1: First test value.
            t2: Second test value.
            t3: Third test value.
            dist_fun: Distance function to test.
            *args: Additional arguments passed to dist_fun.
        """
        # satisfies identity
        self.assertAlmostEqual(dist_fun(t1, t1, *args), 0, delta=1e-07)
        self.assertAlmostEqual(dist_fun(t2, t2, *args), 0, delta=1e-07)
        self.assertAlmostEqual(dist_fun(t3, t3, *args), 0, delta=1e-07)

        # satisfies symmetry
        self.assertAlmostEqual(dist_fun(t1, t2, *args), dist_fun(t2, t1, *args))
        self.assertAlmostEqual(dist_fun(t1, t3, *args), dist_fun(t3, t1, *args))
        self.assertAlmostEqual(dist_fun(t3, t2, *args), dist_fun(t2, t3, *args))

        # satisfies triangle inequality
        self.assertGreaterEqual(dist_fun(t1, t2, *args) + dist_fun(t2, t3, *args), dist_fun(t1, t3, *args))
        self.assertGreaterEqual(dist_fun(t1, t3, *args) + dist_fun(t2, t3, *args), dist_fun(t1, t2, *args))
        self.assertGreaterEqual(dist_fun(t1, t2, *args) + dist_fun(t1, t3, *args), dist_fun(t2, t3, *args))

    """
    There are distance metrics implemented for 3d translation and rotation independently.  The inputs
    to these functions can be 4x4 transformation matrices, or they can be purely translations/rotations.
    """

    async def test_weighted_translational_distance(self) -> None:
        """Test weighted translational distance."""
        # Check that the conditions for being a distance metric are met for 200 random translations/weights.
        for i in range(200):
            transform1 = np.random.uniform(size=(4, 4))
            transform2 = np.random.uniform(size=(4, 4))
            transform3 = Gf.Matrix4d().SetTranslate(Gf.Vec3d(*np.random.uniform(size=3)))

            translate1 = transform1[:3, 3]
            translate2 = transform2[:3, 3]

            # Check that translation is extracted properly from transformation matrices.
            self.assertTrue(
                weighted_translational_distance(translate1, translate2)
                == weighted_translational_distance(transform1, transform2)
            )

            # A weight matrix can be added to the distance calculation to put weight on a specific axis.
            weight_matrix = np.eye(3)
            weight_matrix[0, 0] = np.random.uniform()
            weight_matrix[1, 1] = np.random.uniform()
            weight_matrix[2, 2] = np.random.uniform()  # z distance is multiplied by 3

            await self.is_distance_metric(
                transform1, transform2, transform3, weighted_translational_distance, weight_matrix
            )

        """
        The weight matrix can be used to weight the difference of two translations along one or more 
        arbitrary axes
        """

        # Example: Only weight the difference along the z axis of translate1 - translate2.
        weight_matrix[0, 0] = 0
        weight_matrix[1, 1] = 0
        weight_matrix[2, 2] = 3
        self.assertAlmostEqual(
            weighted_translational_distance(translate1, translate2, weight_matrix),
            np.sqrt(3) * abs(translate1[-1] - translate2[-1]),
        )

        # Example: Weight the difference along the axis [1,1,0] by sqrt(2)
        translate1 = np.zeros(3)
        # translate2 is made of 3 basis vectors. The weight will cause the norm of each basis vector to be one
        translate2 = np.array([1, 1, 0]) / 2 + np.array([-1, 1, 0]) / np.sqrt(2) + np.array([0, 0, 1])
        weight_matrix = np.eye(3)
        weight_matrix[0, 0] = 2  # weight the x axis
        rot = R.from_rotvec([0, 0, -np.pi / 4]).as_matrix()  # rotates [1,1,0] onto the x axis
        # Thus the [1,1,0] axis is weighted by sqrt(2) in the resulting weight matrix
        weight_matrix = rot.T @ weight_matrix @ rot
        self.assertAlmostEqual(weighted_translational_distance(translate1, translate2, weight_matrix), np.sqrt(3))

    async def test_rotational_distance_angle(self) -> None:
        """Test rotational distance angle."""
        rad2deg = 360 / np.pi / 2
        transform1_gf = Gf.Matrix4d().SetRotate(Gf.Rotation(Gf.Vec3d(0.0, 0.0, 1.0), np.pi / 4 * rad2deg))
        transform1_np = np.eye(4)
        transform1_np[:3, :3] = R.from_rotvec([0, 0, np.pi / 4]).as_matrix()

        rotate1_np = R.from_rotvec([0, 0, np.pi]).as_matrix()

        # Rotational distance is equal to the magnitude of the angle from r1 ro r2.
        self.assertAlmostEqual(rotational_distance_angle(transform1_gf, transform1_np), 0)
        self.assertAlmostEqual(rotational_distance_angle(transform1_np, rotate1_np), 3 * np.pi / 4)
        self.assertAlmostEqual(rotational_distance_angle(transform1_gf, rotate1_np), 3 * np.pi / 4)

        # Check that the conditions for being a distance metric are met for 200 random rotations.
        for i in range(200):
            n = np.random.uniform(size=3)
            r1 = R.from_rotvec(n / np.linalg.norm(n) * np.random.uniform(high=np.pi / 2)).as_matrix()
            n = np.random.uniform(size=3)
            r2 = R.from_rotvec(n / np.linalg.norm(n) * np.random.uniform(high=np.pi / 2)).as_matrix()
            r3 = Gf.Matrix4d().SetRotate(Gf.Rotation(Gf.Vec3d(*np.random.uniform(size=3)), np.random.uniform(high=180)))

            await self.is_distance_metric(r1, r2, r3, rotational_distance_angle)

    async def test_rotational_distance_identity_matrix_deviation(self) -> None:
        """Test rotational distance identity matrix deviation."""
        # Check that the conditions for being a distance metric are met for 200 random rotations.
        for i in range(200):
            n = np.random.uniform(size=3)
            r1 = R.from_rotvec(n / np.linalg.norm(n) * np.random.uniform(high=np.pi / 2)).as_matrix()
            n = np.random.uniform(size=3)
            r2 = R.from_rotvec(n / np.linalg.norm(n) * np.random.uniform(high=np.pi / 2)).as_matrix()
            r3 = Gf.Matrix4d().SetRotate(Gf.Rotation(Gf.Vec3d(*np.random.uniform(size=3)), np.random.uniform(high=180)))

            await self.is_distance_metric(r1, r2, r3, rotational_distance_identity_matrix_deviation)

    async def test_rotational_distance_single_axis(self) -> None:
        """Test rotational distance single axis."""
        r1 = R.from_rotvec([0, 0, np.pi / 2]).as_matrix()
        r2 = R.from_rotvec([0.1, 0, 0]).as_matrix()

        axis = [0, 0, 1]  # ignore rotation about the z axis

        self.assertAlmostEqual(rotational_distance_single_axis(r1, r2, axis), 0.1)

        # Check that the conditions for being a distance metric are met for 200 random rotations.
        for i in range(200):
            n = np.random.uniform(size=3)
            r1 = R.from_rotvec(n / np.linalg.norm(n) * np.random.uniform(high=np.pi / 2)).as_matrix()
            n = np.random.uniform(size=3)
            r2 = R.from_rotvec(n / np.linalg.norm(n) * np.random.uniform(high=np.pi / 2)).as_matrix()
            r3 = Gf.Matrix4d().SetRotate(Gf.Rotation(Gf.Vec3d(*np.random.uniform(size=3)), np.random.uniform(high=180)))

            axis = np.random.uniform(size=3)
            await self.is_distance_metric(r1, r2, r3, rotational_distance_single_axis, axis)
