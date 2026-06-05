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


"""Test for transform."""

import isaacsim.core.experimental.utils.transform as transform_utils
import numpy as np
import omni.kit.test
import warp as wp


class TestTransform(omni.kit.test.AsyncTestCase):
    """Test transform."""

    async def setUp(self):
        """Method called to prepare the test fixture."""
        super().setUp()
        # ---------------
        # Do custom setUp
        # ---------------
        self.tolerance = 1e-6

    async def tearDown(self):
        """Method called immediately after the test method has been called."""
        # ------------------
        # Do custom tearDown
        # ------------------
        super().tearDown()

    # --------------------------------------------------------------------

    async def test_rotation_matrix_to_quaternion(self):
        """Test rotation_matrix_to_quaternion with single and batch inputs."""
        # Test identity matrix with different input types
        identity_inputs = [
            np.eye(3),  # numpy
            wp.array(np.eye(3)),  # warp
        ]

        for identity in identity_inputs:
            result = transform_utils.rotation_matrix_to_quaternion(identity)

            # Check that result is a warp array
            self.assertIsInstance(result, wp.array)
            self.assertEqual(result.shape, (4,))

            # Identity should produce quaternion [1, 0, 0, 0] (w, x, y, z)
            result_np = result.numpy()
            expected = np.array([1.0, 0.0, 0.0, 0.0])
            self.assertTrue(np.allclose(result_np, expected, atol=self.tolerance))

        # Test batch of identity matrices
        batch = np.array([np.eye(3), np.eye(3)], dtype=np.float32)
        result_batch = transform_utils.rotation_matrix_to_quaternion(batch)

        # Check that result is a warp array with correct shape
        self.assertIsInstance(result_batch, wp.array)
        self.assertEqual(result_batch.shape, (2, 4))

        # Check that all quaternions are unit quaternions
        result_batch_np = result_batch.numpy()
        norms = np.linalg.norm(result_batch_np, axis=1)
        expected_norms = np.ones(2)
        self.assertTrue(np.allclose(norms, expected_norms, atol=self.tolerance))

    async def test_euler_angles_to_rotation_matrix(self):
        """Test euler_angles_to_rotation_matrix with single and batch inputs."""
        # Test zero rotation with different input types
        euler_inputs = [
            [0.0, 0.0, 0.0],  # list
            np.array([0.0, 0.0, 0.0]),  # numpy
            wp.array([0.0, 0.0, 0.0]),  # warp
        ]

        for euler in euler_inputs:
            result = transform_utils.euler_angles_to_rotation_matrix(euler)

            # Check that result is a warp array
            self.assertIsInstance(result, wp.array)
            self.assertEqual(result.shape, (3, 3))

            # Should produce identity matrix
            result_np = result.numpy()
            expected = np.eye(3)
            self.assertTrue(np.allclose(result_np, expected, atol=self.tolerance))

        # Test batch of zero rotations
        euler_batch = np.array([[0.0, 0.0, 0.0], [0.0, 0.0, 0.0]], dtype=np.float32)
        result_batch = transform_utils.euler_angles_to_rotation_matrix(euler_batch)

        # Check that result is a warp array with correct shape
        self.assertIsInstance(result_batch, wp.array)
        self.assertEqual(result_batch.shape, (2, 3, 3))

        # Check that all matrices are orthogonal (det = 1)
        result_batch_np = result_batch.numpy()
        for i in range(2):
            det = np.linalg.det(result_batch_np[i])
            self.assertAlmostEqual(det, 1.0, places=5)

    async def test_euler_angles_to_quaternion(self):
        """Test euler_angles_to_quaternion with single and batch inputs."""
        # Test zero rotation with different input types
        euler_inputs = [
            [0.0, 0.0, 0.0],  # list
            np.array([0.0, 0.0, 0.0]),  # numpy
            wp.array([0.0, 0.0, 0.0]),  # warp
        ]

        for euler in euler_inputs:
            result = transform_utils.euler_angles_to_quaternion(euler)

            # Check that result is a warp array
            self.assertIsInstance(result, wp.array)
            self.assertEqual(result.shape, (4,))

            # Should produce identity quaternion [1, 0, 0, 0]
            result_np = result.numpy()
            expected = np.array([1.0, 0.0, 0.0, 0.0])
            self.assertTrue(np.allclose(result_np, expected, atol=self.tolerance))

        # Test batch of zero rotations
        euler_batch = np.array([[0.0, 0.0, 0.0], [0.0, 0.0, 0.0]], dtype=np.float32)
        result_batch = transform_utils.euler_angles_to_quaternion(euler_batch)

        # Check that result is a warp array with correct shape
        self.assertIsInstance(result_batch, wp.array)
        self.assertEqual(result_batch.shape, (2, 4))

        # Check that all quaternions are unit quaternions
        result_batch_np = result_batch.numpy()
        norms = np.linalg.norm(result_batch_np, axis=1)
        expected_norms = np.ones(2)
        self.assertTrue(np.allclose(norms, expected_norms, atol=self.tolerance))

    async def test_degrees_vs_radians(self):
        """Test that degrees and radians produce consistent results."""
        # Test conversion consistency for zero rotation
        euler_deg = [0.0, 0.0, 0.0]
        euler_rad = [0.0, 0.0, 0.0]

        # Test rotation matrix
        result_deg = transform_utils.euler_angles_to_rotation_matrix(euler_deg, degrees=True)
        result_rad = transform_utils.euler_angles_to_rotation_matrix(euler_rad, degrees=False)

        self.assertTrue(np.allclose(result_deg.numpy(), result_rad.numpy(), atol=self.tolerance))

        # Test quaternion
        result_deg = transform_utils.euler_angles_to_quaternion(euler_deg, degrees=True)
        result_rad = transform_utils.euler_angles_to_quaternion(euler_rad, degrees=False)

        self.assertTrue(np.allclose(result_deg.numpy(), result_rad.numpy(), atol=self.tolerance))

    async def test_basic_mathematical_properties(self):
        """Test basic mathematical properties of the results."""
        # Test identity rotation
        euler = [0.0, 0.0, 0.0]

        # Test rotation matrix properties
        rotation_matrix = transform_utils.euler_angles_to_rotation_matrix(euler)
        R = rotation_matrix.numpy()

        # Check that it's close to identity
        identity = np.eye(3)
        self.assertTrue(np.allclose(R, identity, atol=self.tolerance))

        # Check determinant is 1
        det = np.linalg.det(R)
        self.assertAlmostEqual(det, 1.0, places=5)

        # Test quaternion properties
        quaternion = transform_utils.euler_angles_to_quaternion(euler)
        q = quaternion.numpy()

        # Check that it's a unit quaternion
        norm = np.linalg.norm(q)
        self.assertAlmostEqual(norm, 1.0, places=5)

    async def test_quaternion_multiplication(self):
        """Test quaternion_multiplication with single and batch inputs."""
        # Test identity quaternion multiplication with different input types
        quaternion_inputs = [
            [1.0, 0.0, 0.0, 0.0],  # list
            np.array([1.0, 0.0, 0.0, 0.0]),  # numpy
            wp.array([1.0, 0.0, 0.0, 0.0]),  # warp
        ]

        for quaternion in quaternion_inputs:
            result = transform_utils.quaternion_multiplication(quaternion, quaternion)

            # Check that result is a warp array
            self.assertIsInstance(result, wp.array)
            self.assertEqual(result.shape, (4,))

            # Identity * Identity should be identity
            result_numpy = result.numpy()
            expected = np.array([1.0, 0.0, 0.0, 0.0])
            self.assertTrue(np.allclose(result_numpy, expected, atol=self.tolerance))

        # Test batch of identity quaternions
        identity_batch = np.array([[1.0, 0.0, 0.0, 0.0], [1.0, 0.0, 0.0, 0.0]], dtype=np.float32)
        result_batch = transform_utils.quaternion_multiplication(identity_batch, identity_batch)

        # Check that result is a warp array with correct shape
        self.assertIsInstance(result_batch, wp.array)
        self.assertEqual(result_batch.shape, (2, 4))

        # Check that all results are unit quaternions
        result_batch_numpy = result_batch.numpy()
        norms = np.linalg.norm(result_batch_numpy, axis=1)
        expected_norms = np.ones(2)
        self.assertTrue(np.allclose(norms, expected_norms, atol=self.tolerance))

    async def test_quaternion_conjugate(self):
        """Test quaternion_conjugate with single and batch inputs."""
        # Test with a rotation around X axis using different input types
        quaternion_inputs = [
            [0.7071, 0.7071, 0.0, 0.0],  # list
            np.array([0.7071, 0.7071, 0.0, 0.0]),  # numpy
            wp.array([0.7071, 0.7071, 0.0, 0.0]),  # warp
        ]

        for quaternion in quaternion_inputs:
            result = transform_utils.quaternion_conjugate(quaternion)

            # Check that result is a warp array
            self.assertIsInstance(result, wp.array)
            self.assertEqual(result.shape, (4,))

            # Conjugate should negate vector components
            result_numpy = result.numpy()
            expected = np.array([0.7071, -0.7071, 0.0, 0.0])
            self.assertTrue(np.allclose(result_numpy, expected, atol=self.tolerance))

        # Test batch of rotations
        quaternion_batch = np.array(
            [[0.7071, 0.7071, 0.0, 0.0], [0.7071, 0.0, 0.7071, 0.0]],  # 90 degrees around X  # 90 degrees around Y
            dtype=np.float32,
        )
        result_batch = transform_utils.quaternion_conjugate(quaternion_batch)

        # Check that result is a warp array with correct shape
        self.assertIsInstance(result_batch, wp.array)
        self.assertEqual(result_batch.shape, (2, 4))

        # Check that all results are unit quaternions
        result_batch_numpy = result_batch.numpy()
        norms = np.linalg.norm(result_batch_numpy, axis=1)
        expected_norms = np.ones(2)
        self.assertTrue(np.allclose(norms, expected_norms, atol=self.tolerance))

    async def test_quaternion_to_rotation_matrix(self):
        """Test quaternion_to_rotation_matrix with single and batch inputs, including round-trip validation."""
        # Test identity quaternion with different input types
        quaternion_inputs = [
            [1.0, 0.0, 0.0, 0.0],  # list
            np.array([1.0, 0.0, 0.0, 0.0]),  # numpy
            wp.array([1.0, 0.0, 0.0, 0.0]),  # warp
        ]

        for quaternion in quaternion_inputs:
            result = transform_utils.quaternion_to_rotation_matrix(quaternion)

            # Check that result is a warp array
            self.assertIsInstance(result, wp.array)
            self.assertEqual(result.shape, (3, 3))

            # Identity quaternion should produce identity matrix
            result_numpy = result.numpy()
            expected = np.eye(3)
            self.assertTrue(np.allclose(result_numpy, expected, atol=self.tolerance))

        # Test batch of identity quaternions
        identity_batch = np.array([[1.0, 0.0, 0.0, 0.0], [1.0, 0.0, 0.0, 0.0]], dtype=np.float32)
        result_batch = transform_utils.quaternion_to_rotation_matrix(identity_batch)

        # Check that result is a warp array with correct shape
        self.assertIsInstance(result_batch, wp.array)
        self.assertEqual(result_batch.shape, (2, 3, 3))

        # Check that all matrices are orthogonal (det = 1)
        result_batch_numpy = result_batch.numpy()
        for i in range(2):
            det = np.linalg.det(result_batch_numpy[i])
            self.assertAlmostEqual(det, 1.0, places=5)

        # Test round-trip conversion: quaternion -> matrix -> quaternion
        quaternion = [0.7071, 0.7071, 0.0, 0.0]  # 90 degrees around X

        # Convert to rotation matrix
        rotation_matrix = transform_utils.quaternion_to_rotation_matrix(quaternion)

        # Convert back to quaternion
        quaternion_back = transform_utils.rotation_matrix_to_quaternion(rotation_matrix)

        # Results should be approximately equal (allowing for sign differences)
        original_np = np.array(quaternion)
        back_np = quaternion_back.numpy()

        # Check that either the quaternions are equal or one is the negative of the other
        # (both represent the same rotation)
        self.assertTrue(
            np.allclose(original_np, back_np, atol=self.tolerance)
            or np.allclose(original_np, -back_np, atol=self.tolerance)
        )

        # Test round-trip conversion with batch of quaternions
        quaternion_batch = np.array(
            [
                [1.0, 0.0, 0.0, 0.0],  # Identity
                [0.7071, 0.7071, 0.0, 0.0],  # 90 degrees around X
                [0.7071, 0.0, 0.7071, 0.0],  # 90 degrees around Y
                [0.7071, 0.0, 0.0, 0.7071],  # 90 degrees around Z
            ],
            dtype=np.float32,
        )

        # Convert to rotation matrices
        rotation_matrices = transform_utils.quaternion_to_rotation_matrix(quaternion_batch)

        # Convert back to quaternions
        quaternions_back = transform_utils.rotation_matrix_to_quaternion(rotation_matrices)

        # Check that all roundtrip conversions are valid
        original_np = quaternion_batch
        back_np = quaternions_back.numpy()

        for i in range(4):
            # Check that either the quaternions are equal or one is the negative of the other
            self.assertTrue(
                np.allclose(original_np[i], back_np[i], atol=self.tolerance)
                or np.allclose(original_np[i], -back_np[i], atol=self.tolerance)
            )

        # Test mathematical properties of quaternion_to_rotation_matrix results
        quaternion = [0.7071, 0.7071, 0.0, 0.0]  # 90 degrees around X
        result = transform_utils.quaternion_to_rotation_matrix(quaternion)
        R = result.numpy()

        # Check that it's a valid rotation matrix
        # 1. Determinant should be 1
        det = np.linalg.det(R)
        self.assertAlmostEqual(det, 1.0, places=5)

        # 2. Should be orthogonal (R * R^T = I)
        R_transpose = R.T
        identity = np.eye(3)
        self.assertTrue(np.allclose(np.dot(R, R_transpose), identity, atol=self.tolerance))

        # 3. Check specific values for 90-degree X rotation
        expected = np.array([[1.0, 0.0, 0.0], [0.0, 0.0, -1.0], [0.0, 1.0, 0.0]])
        self.assertTrue(np.allclose(R, expected, atol=self.tolerance))

    async def test_quaternion_multiplication_associativity(self):
        """Test that quaternion multiplication is associative."""
        # Test (first_quaternion * second_quaternion) * third_quaternion = first_quaternion * (second_quaternion * third_quaternion)
        first_quaternion = np.array([0.7071, 0.7071, 0.0, 0.0])  # 90 deg around X
        second_quaternion = np.array([0.7071, 0.0, 0.7071, 0.0])  # 90 deg around Y
        third_quaternion = np.array([0.7071, 0.0, 0.0, 0.7071])  # 90 deg around Z

        # Compute (first_quaternion * second_quaternion) * third_quaternion
        quaternion_first_second = transform_utils.quaternion_multiplication(first_quaternion, second_quaternion)
        quaternion_result_left = transform_utils.quaternion_multiplication(quaternion_first_second, third_quaternion)

        # Compute first_quaternion * (second_quaternion * third_quaternion)
        quaternion_second_third = transform_utils.quaternion_multiplication(second_quaternion, third_quaternion)
        quaternion_result_right = transform_utils.quaternion_multiplication(first_quaternion, quaternion_second_third)

        # Results should be equal
        self.assertTrue(
            np.allclose(quaternion_result_left.numpy(), quaternion_result_right.numpy(), atol=self.tolerance)
        )

    async def test_quaternion_to_euler_angles(self):
        """Test quaternion_to_euler_angles with single and batch inputs."""
        # Test identity quaternion with different input types
        quaternion_inputs = [
            [1.0, 0.0, 0.0, 0.0],  # list
            np.array([1.0, 0.0, 0.0, 0.0]),  # numpy
            wp.array([1.0, 0.0, 0.0, 0.0]),  # warp
        ]

        for quaternion in quaternion_inputs:
            result = transform_utils.quaternion_to_euler_angles(quaternion)

            # Check that result is a warp array
            self.assertIsInstance(result, wp.array)
            self.assertEqual(result.shape, (3,))

            # Identity quaternion should produce zero euler angles
            result_np = result.numpy()
            expected = np.array([0.0, 0.0, 0.0])
            self.assertTrue(np.allclose(result_np, expected, atol=self.tolerance))

        # Test batch of identity quaternions
        identity_batch = np.array([[1.0, 0.0, 0.0, 0.0], [1.0, 0.0, 0.0, 0.0]], dtype=np.float32)
        result_batch = transform_utils.quaternion_to_euler_angles(identity_batch)

        # Check that result is a warp array with correct shape
        self.assertIsInstance(result_batch, wp.array)
        self.assertEqual(result_batch.shape, (2, 3))

        # Check all results are zero
        result_batch_np = result_batch.numpy()
        expected_batch = np.zeros((2, 3))
        self.assertTrue(np.allclose(result_batch_np, expected_batch, atol=self.tolerance))

    async def test_quaternion_to_euler_angles_degrees(self):
        """Test quaternion_to_euler_angles with degrees output."""
        # Identity quaternion should produce zero angles in both radians and degrees
        identity = np.array([1.0, 0.0, 0.0, 0.0])

        result_rad = transform_utils.quaternion_to_euler_angles(identity, degrees=False)
        result_deg = transform_utils.quaternion_to_euler_angles(identity, degrees=True)

        # Both should be zero
        self.assertTrue(np.allclose(result_rad.numpy(), np.zeros(3), atol=self.tolerance))
        self.assertTrue(np.allclose(result_deg.numpy(), np.zeros(3), atol=self.tolerance))

        # Test 90 degree rotation around X axis
        # Quaternion for 90 deg around X: [cos(45), sin(45), 0, 0] = [0.7071, 0.7071, 0, 0]
        quat_90x = np.array([0.7071067811865476, 0.7071067811865476, 0.0, 0.0])

        result_rad = transform_utils.quaternion_to_euler_angles(quat_90x, degrees=False, extrinsic=True)
        result_deg = transform_utils.quaternion_to_euler_angles(quat_90x, degrees=True, extrinsic=True)

        # For extrinsic convention, output order is [X, Y, Z], so X rotation is at index 0
        # Should be approximately [pi/2, 0, 0] in radians and [90, 0, 0] in degrees
        self.assertTrue(np.allclose(result_rad.numpy()[0], np.pi / 2, atol=self.tolerance))
        self.assertTrue(np.allclose(result_deg.numpy()[0], 90.0, atol=self.tolerance))

    async def test_quaternion_to_euler_angles_intrinsic(self):
        """Test quaternion_to_euler_angles with intrinsic convention."""
        # Identity quaternion should produce zero angles
        identity = np.array([1.0, 0.0, 0.0, 0.0])
        result_identity = transform_utils.quaternion_to_euler_angles(identity, extrinsic=False)
        self.assertTrue(np.allclose(result_identity.numpy(), np.zeros(3), atol=self.tolerance))

        # 90 degree rotation around X should be [pi/2, 0, 0] in intrinsic [X, Y, Z] order
        quat_90x = np.array([0.7071067811865476, 0.7071067811865476, 0.0, 0.0])
        result_90x = transform_utils.quaternion_to_euler_angles(quat_90x, extrinsic=False)
        result_90x_np = result_90x.numpy()
        self.assertTrue(np.allclose(result_90x_np[0], np.pi / 2, atol=self.tolerance))
        self.assertTrue(np.allclose(result_90x_np[1:], np.zeros(2), atol=self.tolerance))

        # 90 degree rotation around Y should be [0, pi/2, 0]
        quat_90y = np.array([0.7071067811865476, 0.0, 0.7071067811865476, 0.0])
        result_90y = transform_utils.quaternion_to_euler_angles(quat_90y, extrinsic=False)
        result_90y_np = result_90y.numpy()
        self.assertTrue(np.allclose(result_90y_np[1], np.pi / 2, atol=self.tolerance))
        self.assertTrue(np.allclose(result_90y_np[[0, 2]], np.zeros(2), atol=self.tolerance))

    async def test_quaternion_to_euler_angles_intrinsic_degrees(self):
        """Test quaternion_to_euler_angles with intrinsic convention in degrees."""
        # 90 degree rotation around Z should be [0, 0, 90] in intrinsic [X, Y, Z] order
        quat_90z = np.array([0.7071067811865476, 0.0, 0.0, 0.7071067811865476])
        result_deg = transform_utils.quaternion_to_euler_angles(quat_90z, degrees=True, extrinsic=False)
        result_deg_np = result_deg.numpy()
        self.assertTrue(np.allclose(result_deg_np[2], 90.0, atol=self.tolerance))
        self.assertTrue(np.allclose(result_deg_np[:2], np.zeros(2), atol=self.tolerance))

    async def test_quaternion_to_euler_angles_batch_intrinsic(self):
        """Test quaternion_to_euler_angles batch path with intrinsic convention."""
        sqrt_half = np.sqrt(0.5)
        quaternion_batch = np.array(
            [
                [1.0, 0.0, 0.0, 0.0],  # Identity
                [sqrt_half, sqrt_half, 0.0, 0.0],  # 90 deg around X
                [sqrt_half, 0.0, sqrt_half, 0.0],  # 90 deg around Y
                [sqrt_half, 0.0, 0.0, sqrt_half],  # 90 deg around Z
            ],
            dtype=np.float32,
        )

        result = transform_utils.quaternion_to_euler_angles(quaternion_batch, extrinsic=False)

        # Check shape
        self.assertEqual(result.shape, (4, 3))

        result_np = result.numpy()

        # For intrinsic convention, output order is [X, Y, Z]
        # Check identity gives zero angles
        self.assertTrue(np.allclose(result_np[0], np.zeros(3), atol=self.tolerance))

        # Check 90 deg X rotation gives [pi/2, 0, 0]
        self.assertTrue(np.allclose(result_np[1, 0], np.pi / 2, atol=self.tolerance))
        self.assertTrue(np.allclose(result_np[1, 1:], np.zeros(2), atol=self.tolerance))

        # Check 90 deg Y rotation gives [0, pi/2, 0]
        self.assertTrue(np.allclose(result_np[2, 1], np.pi / 2, atol=self.tolerance))
        self.assertTrue(np.allclose(result_np[2, [0, 2]], np.zeros(2), atol=self.tolerance))

        # Check 90 deg Z rotation gives [0, 0, pi/2]
        self.assertTrue(np.allclose(result_np[3, 2], np.pi / 2, atol=self.tolerance))
        self.assertTrue(np.allclose(result_np[3, :2], np.zeros(2), atol=self.tolerance))

    async def test_euler_quaternion_roundtrip(self):
        """Test round-trip conversion: euler -> quaternion -> euler."""
        # Test various euler angles
        test_angles = [
            np.array([0.0, 0.0, 0.0]),  # Identity
            np.array([0.5, 0.0, 0.0]),  # Small X rotation
            np.array([0.0, 0.5, 0.0]),  # Small Y rotation
            np.array([0.0, 0.0, 0.5]),  # Small Z rotation
            np.array([0.3, 0.4, 0.5]),  # Combined rotation
        ]

        for euler_original in test_angles:
            # Convert to quaternion (input is [roll, pitch, yaw])
            quaternion = transform_utils.euler_angles_to_quaternion(euler_original, extrinsic=True)

            # Convert back to euler (output is [roll, pitch, yaw])
            euler_back = transform_utils.quaternion_to_euler_angles(quaternion, extrinsic=True)

            # Should be approximately equal
            self.assertTrue(
                np.allclose(euler_original, euler_back.numpy(), atol=1e-5),
                f"Round-trip failed for {euler_original}: got {euler_back.numpy()}",
            )

    async def test_quaternion_euler_roundtrip(self):
        """Test round-trip conversion: quaternion -> euler -> quaternion."""
        # Test various quaternions (all unit quaternions)
        test_quaternions = [
            np.array([1.0, 0.0, 0.0, 0.0]),  # Identity
            np.array([0.7071, 0.7071, 0.0, 0.0]),  # 90 deg around X
            np.array([0.7071, 0.0, 0.7071, 0.0]),  # 90 deg around Y
            np.array([0.7071, 0.0, 0.0, 0.7071]),  # 90 deg around Z
            np.array([0.5, 0.5, 0.5, 0.5]),  # Combined rotation
        ]

        for quat_original in test_quaternions:
            # Normalize to ensure it's a unit quaternion
            quat_original = quat_original / np.linalg.norm(quat_original)

            # Convert to euler (output is [roll, pitch, yaw])
            euler = transform_utils.quaternion_to_euler_angles(quat_original, extrinsic=True)

            # Convert back to quaternion (input is [roll, pitch, yaw])
            quat_back = transform_utils.euler_angles_to_quaternion(euler, extrinsic=True)

            # Quaternions q and -q represent the same rotation
            quat_back_np = quat_back.numpy()
            self.assertTrue(
                np.allclose(quat_original, quat_back_np, atol=1e-5)
                or np.allclose(quat_original, -quat_back_np, atol=1e-5),
                f"Round-trip failed for {quat_original}: got {quat_back_np}",
            )

    async def test_euler_quaternion_roundtrip_intrinsic(self):
        """Test round-trip conversion with intrinsic convention."""
        test_angles = [
            np.array([0.0, 0.0, 0.0]),
            np.array([0.5, 0.0, 0.0]),
            np.array([0.0, 0.5, 0.0]),
            np.array([0.0, 0.0, 0.5]),
            np.array([0.3, 0.4, 0.5]),
        ]

        for euler_original in test_angles:
            quaternion = transform_utils.euler_angles_to_quaternion(euler_original, extrinsic=False)
            euler_back = transform_utils.quaternion_to_euler_angles(quaternion, extrinsic=False)
            self.assertTrue(
                np.allclose(euler_original, euler_back.numpy(), atol=1e-5),
                f"Intrinsic round-trip failed for {euler_original}: got {euler_back.numpy()}",
            )

    async def test_quaternion_euler_roundtrip_intrinsic(self):
        """Test round-trip conversion: quaternion -> euler -> quaternion with intrinsic convention."""
        test_quaternions = [
            np.array([1.0, 0.0, 0.0, 0.0]),  # Identity
            np.array([0.7071, 0.7071, 0.0, 0.0]),  # 90 deg around X
            np.array([0.7071, 0.0, 0.7071, 0.0]),  # 90 deg around Y
            np.array([0.7071, 0.0, 0.0, 0.7071]),  # 90 deg around Z
            np.array([0.5, 0.5, 0.5, 0.5]),  # Combined rotation
        ]

        for quat_original in test_quaternions:
            quat_original = quat_original / np.linalg.norm(quat_original)
            euler = transform_utils.quaternion_to_euler_angles(quat_original, extrinsic=False)
            quat_back = transform_utils.euler_angles_to_quaternion(euler, extrinsic=False)

            quat_back_np = quat_back.numpy()
            self.assertTrue(
                np.allclose(quat_original, quat_back_np, atol=1e-5)
                or np.allclose(quat_original, -quat_back_np, atol=1e-5),
                f"Intrinsic round-trip failed for {quat_original}: got {quat_back_np}",
            )

    async def test_quaternion_to_euler_angles_batch(self):
        """Test quaternion_to_euler_angles with batch of different rotations."""
        quaternion_batch = np.array(
            [
                [1.0, 0.0, 0.0, 0.0],  # Identity
                [0.7071, 0.7071, 0.0, 0.0],  # 90 deg around X
                [0.7071, 0.0, 0.7071, 0.0],  # 90 deg around Y
                [0.7071, 0.0, 0.0, 0.7071],  # 90 deg around Z
            ],
            dtype=np.float32,
        )

        result = transform_utils.quaternion_to_euler_angles(quaternion_batch, extrinsic=True)

        # Check shape
        self.assertEqual(result.shape, (4, 3))

        result_np = result.numpy()

        # For extrinsic convention, output order is [X, Y, Z]
        # Check identity gives zero angles
        self.assertTrue(np.allclose(result_np[0], np.zeros(3), atol=self.tolerance))

        # Check 90 deg X rotation gives [pi/2, 0, 0]
        self.assertTrue(np.allclose(result_np[1, 0], np.pi / 2, atol=self.tolerance))
        self.assertTrue(np.allclose(result_np[1, 1:], np.zeros(2), atol=self.tolerance))

        # Check 90 deg Y rotation gives [0, pi/2, 0]
        self.assertTrue(np.allclose(result_np[2, 1], np.pi / 2, atol=self.tolerance))

        # Check 90 deg Z rotation gives [0, 0, pi/2]
        self.assertTrue(np.allclose(result_np[3, 2], np.pi / 2, atol=self.tolerance))

    # ----------------------------------------------------------------
    # look_at_quaternion
    # ----------------------------------------------------------------

    async def test_look_at_quaternion_single(self):
        """Test look_at_quaternion with a single eye/target pair."""
        eye = np.array([5.0, 5.0, 5.0])
        target = np.array([0.0, 0.0, 0.0])
        result = transform_utils.look_at_quaternion(eye=eye, target=target)

        self.assertIsInstance(result, wp.array)
        self.assertEqual(result.shape, (4,))

        # Verify the quaternion is unit-length
        q = result.numpy()
        self.assertAlmostEqual(float(np.linalg.norm(q)), 1.0, places=5)

    async def test_look_at_quaternion_z_aligned_fallback(self):
        """Test that looking straight down uses the up-vector fallback."""
        eye = np.array([0.0, 0.0, 10.0])
        target = np.array([0.0, 0.0, 0.0])
        result = transform_utils.look_at_quaternion(eye=eye, target=target)

        q = result.numpy()
        self.assertAlmostEqual(float(np.linalg.norm(q)), 1.0, places=5)

    async def test_look_at_quaternion_batch(self):
        """Test look_at_quaternion with batched inputs."""
        eyes = np.array([[5, 5, 5], [10, 0, 0], [0, 10, 0]], dtype=np.float32)
        targets = np.zeros((3, 3), dtype=np.float32)
        result = transform_utils.look_at_quaternion(eye=eyes, target=targets)

        self.assertIsInstance(result, wp.array)
        self.assertEqual(result.shape, (3, 4))

        quats = result.numpy()
        for i in range(3):
            self.assertAlmostEqual(float(np.linalg.norm(quats[i])), 1.0, places=5)

    async def test_look_at_quaternion_custom_up(self):
        """Test look_at_quaternion with a custom up vector."""
        eye = np.array([5.0, 5.0, 5.0])
        target = np.array([0.0, 0.0, 0.0])
        result_z = transform_utils.look_at_quaternion(eye=eye, target=target)
        result_y = transform_utils.look_at_quaternion(eye=eye, target=target, up=np.array([0.0, 1.0, 0.0]))

        # Different up vectors should produce different orientations
        self.assertFalse(np.allclose(result_z.numpy(), result_y.numpy(), atol=1e-4))

    async def test_look_at_quaternion_roundtrip(self):
        """Test that look_at quaternion produces correct forward direction."""
        eye = np.array([10.0, 0.0, 0.0], dtype=np.float32)
        target = np.array([0.0, 0.0, 0.0], dtype=np.float32)
        quat = transform_utils.look_at_quaternion(eye=eye, target=target)

        # Convert quaternion to rotation matrix, extract -Z column (forward)
        rot_mat = transform_utils.quaternion_to_rotation_matrix(quat).numpy()
        forward = -rot_mat[:, 2]  # -Z axis in the rotated frame

        # Forward should point from eye toward target: [-1, 0, 0]
        expected = np.array([-1.0, 0.0, 0.0])
        self.assertTrue(np.allclose(forward, expected, atol=1e-4))

    async def test_look_at_quaternion_shape_mismatch(self):
        """Test that mismatched eye/target shapes raise ValueError."""
        eye = np.array([[5, 5, 5], [10, 0, 0]], dtype=np.float32)
        target = np.array([[0, 0, 0]], dtype=np.float32)
        with self.assertRaises(ValueError):
            transform_utils.look_at_quaternion(eye=eye, target=target)

    # ----------------------------------------------------------------
    # look_at_matrix — equivalence with Gf.Matrix4d.SetLookAt
    # ----------------------------------------------------------------

    async def test_look_at_matrix_matches_gf_set_look_at(self):
        """Test that look_at_matrix produces the same result as Gf.Matrix4d.SetLookAt.GetInverse."""
        from pxr import Gf

        test_cases = [
            (Gf.Vec3d(5, 5, 5), Gf.Vec3d(0, 0, 0), Gf.Vec3d(0, 0, 1)),
            (Gf.Vec3d(10, 0, 0), Gf.Vec3d(0, 0, 0), Gf.Vec3d(0, 0, 1)),
            (Gf.Vec3d(0, 10, 0), Gf.Vec3d(0, 0, 0), Gf.Vec3d(0, 0, 1)),
            (Gf.Vec3d(0, 0, 10), Gf.Vec3d(0, 0, 5), Gf.Vec3d(0, 1, 0)),
            (Gf.Vec3d(3, 4, 5), Gf.Vec3d(1, 2, 3), Gf.Vec3d(0, 0, 1)),
            (Gf.Vec3d(-5, -5, 10), Gf.Vec3d(5, 5, 0), Gf.Vec3d(0, 0, 1)),
            (Gf.Vec3d(100, 0, 50), Gf.Vec3d(0, 0, 0), Gf.Vec3d(0, 1, 0)),
        ]

        for eye, target, up in test_cases:
            expected = Gf.Matrix4d(1).SetLookAt(eye, target, up).GetInverse()
            result = transform_utils.look_at_matrix(eye, target, up)

            for row in range(4):
                for col in range(4):
                    self.assertAlmostEqual(
                        result[row][col],
                        expected[row][col],
                        places=10,
                        msg=f"Mismatch at [{row}][{col}] for eye={eye}, target={target}, up={up}",
                    )

    async def test_look_at_matrix_with_numpy_inputs(self):
        """Test that look_at_matrix accepts numpy arrays and matches Gf.SetLookAt."""
        from pxr import Gf

        eye_np = np.array([5.0, 5.0, 5.0])
        target_np = np.array([0.0, 0.0, 0.0])
        up_np = np.array([0.0, 0.0, 1.0])

        eye_gf = Gf.Vec3d(5.0, 5.0, 5.0)
        target_gf = Gf.Vec3d(0.0, 0.0, 0.0)
        up_gf = Gf.Vec3d(0.0, 0.0, 1.0)

        result_np = transform_utils.look_at_matrix(eye_np, target_np, up_np)
        result_gf = Gf.Matrix4d(1).SetLookAt(eye_gf, target_gf, up_gf).GetInverse()

        for row in range(4):
            for col in range(4):
                self.assertAlmostEqual(result_np[row][col], result_gf[row][col], places=10)

    async def test_look_at_matrix_with_list_inputs(self):
        """Test that look_at_matrix accepts Python lists and matches Gf.SetLookAt."""
        from pxr import Gf

        result_list = transform_utils.look_at_matrix([10, 0, 5], [0, 0, 0], [0, 0, 1])
        expected = Gf.Matrix4d(1).SetLookAt(Gf.Vec3d(10, 0, 5), Gf.Vec3d(0, 0, 0), Gf.Vec3d(0, 0, 1)).GetInverse()

        for row in range(4):
            for col in range(4):
                self.assertAlmostEqual(result_list[row][col], expected[row][col], places=10)

    async def test_look_at_matrix_default_up_is_z(self):
        """Test that omitting the up parameter defaults to Z-up and matches Gf.SetLookAt."""
        from pxr import Gf

        eye = Gf.Vec3d(5, 5, 5)
        target = Gf.Vec3d(0, 0, 0)
        up = Gf.Vec3d(0, 0, 1)

        result_default = transform_utils.look_at_matrix(eye, target)
        expected = Gf.Matrix4d(1).SetLookAt(eye, target, up).GetInverse()

        for row in range(4):
            for col in range(4):
                self.assertAlmostEqual(result_default[row][col], expected[row][col], places=10)

    async def test_look_at_matrix_returns_gf_matrix4d(self):
        """Test that look_at_matrix returns a Gf.Matrix4d instance."""
        from pxr import Gf

        result = transform_utils.look_at_matrix([5, 5, 5], [0, 0, 0])
        self.assertIsInstance(result, Gf.Matrix4d)

    async def test_look_at_matrix_collinearity_fallback(self):
        """Test that look_at_matrix handles collinear forward/up without error."""
        from pxr import Gf

        result = transform_utils.look_at_matrix([0, 0, 10], [0, 0, 0], [0, 0, 1])
        self.assertIsInstance(result, Gf.Matrix4d)

        translation = result.ExtractTranslation()
        self.assertAlmostEqual(translation[0], 0.0, places=5)
        self.assertAlmostEqual(translation[1], 0.0, places=5)
        self.assertAlmostEqual(translation[2], 10.0, places=5)

    async def test_look_at_matrix_translation_is_eye_position(self):
        """Test that the translation component of the returned matrix equals the eye position."""
        from pxr import Gf

        test_eyes = [
            Gf.Vec3d(5, 5, 5),
            Gf.Vec3d(10, 0, 0),
            Gf.Vec3d(0, -3, 7),
            Gf.Vec3d(100, 200, 300),
        ]

        for eye in test_eyes:
            result = transform_utils.look_at_matrix(eye, Gf.Vec3d(0, 0, 0))
            translation = result.ExtractTranslation()
            for i in range(3):
                self.assertAlmostEqual(
                    translation[i],
                    eye[i],
                    places=10,
                    msg=f"Translation[{i}] mismatch for eye={eye}",
                )

    async def test_look_at_matrix_negative_z_points_toward_target(self):
        """Test that the -Z axis of the result matrix points toward the target."""
        from pxr import Gf

        eye = Gf.Vec3d(10, 0, 0)
        target = Gf.Vec3d(0, 0, 0)
        result = transform_utils.look_at_matrix(eye, target)

        neg_z = -Gf.Vec3d(result[2][0], result[2][1], result[2][2])

        expected_forward = (target - eye).GetNormalized()
        for i in range(3):
            self.assertAlmostEqual(neg_z[i], expected_forward[i], places=5)

    # ----------------------------------------------------------------
    # compute_relative_transform
    # ----------------------------------------------------------------

    async def test_compute_relative_transform_identity_inputs(self):
        """Both transforms are identity — result should be identity."""
        result = transform_utils.compute_relative_transform(np.eye(4), np.eye(4))
        np.testing.assert_allclose(result, np.eye(4), atol=1e-10)

    async def test_compute_relative_transform_same_transform(self):
        """Same non-trivial transform for source and target — result should be identity."""
        t = np.eye(4)
        t[3, :3] = [5.0, -3.0, 2.0]  # row-major translation
        result = transform_utils.compute_relative_transform(t, t)
        np.testing.assert_allclose(result, np.eye(4), atol=1e-10)

    async def test_compute_relative_transform_translation_only(self):
        """Pure translation offset between source and target."""
        source = np.eye(4)
        source[3, :3] = [1.0, 2.0, 3.0]

        target = np.eye(4)
        target[3, :3] = [4.0, 6.0, 8.0]

        result = transform_utils.compute_relative_transform(source, target)
        np.testing.assert_allclose(result[:3, 3], [-3.0, -4.0, -5.0], atol=1e-10)
        np.testing.assert_allclose(result[:3, :3], np.eye(3), atol=1e-10)

    async def test_compute_relative_transform_inverse(self):
        """A->B composed with B->A should give identity."""
        source = np.eye(4)
        source[3, :3] = [1.0, 0.0, 0.0]

        # 90-degree rotation around Z in row-major
        target = np.array(
            [
                [0.0, 1.0, 0.0, 0.0],
                [-1.0, 0.0, 0.0, 0.0],
                [0.0, 0.0, 1.0, 0.0],
                [0.0, 2.0, 0.0, 1.0],
            ]
        )

        ab = transform_utils.compute_relative_transform(source, target)
        ba = transform_utils.compute_relative_transform(target, source)
        np.testing.assert_allclose(ab @ ba, np.eye(4), atol=1e-10)

    async def test_compute_relative_transform_accepts_list(self):
        """Should accept plain Python lists."""
        source = [[1, 0, 0, 0], [0, 1, 0, 0], [0, 0, 1, 0], [3, 0, 0, 1]]
        target = [[1, 0, 0, 0], [0, 1, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]]
        result = transform_utils.compute_relative_transform(source, target)
        np.testing.assert_allclose(result[:3, 3], [3.0, 0.0, 0.0], atol=1e-10)

    async def test_compute_relative_transform_accepts_warp(self):
        """Should accept warp arrays."""
        source_np = np.eye(4)
        source_np[3, :3] = [1.0, 2.0, 3.0]
        target_np = np.eye(4)

        source_wp = wp.array(source_np, dtype=wp.float64)
        target_wp = wp.array(target_np, dtype=wp.float64)

        result_np = transform_utils.compute_relative_transform(source_np, target_np)
        result_wp = transform_utils.compute_relative_transform(source_wp, target_wp)
        np.testing.assert_allclose(result_wp, result_np, atol=1e-10)
